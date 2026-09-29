"""Build Kaede -- a 2D mesh-deform anime bust rigged entirely with usdRig.

    cd <repo> && source bin/_env.sh
    "$PY" examples/2d/bust_dd_a/build_bust.py

writes, next to this file,

    textures/*.png            one painted texture atlas per art mesh
    bust_dd_a_rig.usda        the character at rest: textured art meshes + the rig
    bust_dd_a_anim.usda       a 180-frame performance (sublayers the rig)
    bust_dd_a_sweep.usda      a parameter sweep for the rig reveal (sublayers the rig)

Deterministic. Nothing is baked: every frame is evaluated live by RigExec
from the control avars in the animation layers (the hair and accessory
chains are keyed from an offline spring solve, like a layered mesh physics bake
would be, but they are ordinary FK avars).

How the rig is built -- the layered mesh idea, in stock RigExec prims:

* PARAMETERS are RigExecControls; their avars are the parameter values.
* KEYFORMS are RigExecBlendSamples: each channel holds the art at one or two
  parameter values and the blend-shape mover interpolates between them.
* "one parameter drives many deformers": a channel's inputs:weight is
  CONNECTED to the avar that drives it; where the parameter must be negated,
  RigExecFloatMathMovers read it (add) and flip it (multiply -1). The mouth
  vowels are a pose-space RBF (RigExecPoseInterpolator) whose pose weights
  fan out to the lip keyforms AND the jaw keyforms.
* WARP DEFORMERS are RigExecLatticeMovers: one 7x9x2 Bernstein cage wraps
  the head; its Turn/Nod keyforms are least-squares fits of a pseudo-3D head
  (field.py) and the art's z is its depth in the cage, so one warp gives
  every layer its own parallax. The face meshes add exact correctives on
  top (silhouette, nose). A second cage carries breath/turn/shrug of the body.
* ROTATION DEFORMERS are joints: an FK spine (lean, neck, head tilt) and an
  FK chain in every lock of hair, both ribbon tails and the earring.
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import art  # noqa: E402
import art_body  # noqa: E402
import art_hair  # noqa: E402
import design as D  # noqa: E402
import field  # noqa: E402
import pieces as PC  # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdSkel, Vt  # noqa: E402

NAME = "bust_dd_a"
ROOT = "/Kaede"
RIG = ROOT + "/Rig"
CTL = RIG + "/Controls"
JNT = RIG + "/Joints"
GEOM = ROOT + "/Geom"
LOOKS = ROOT + "/Looks"
CAGES = ROOT + "/Cages"
TARGETS = ROOT + "/Targets"
BI = RIG + "/BlendInputs"
MOV = RIG + "/Movers"

FPS = 30
T = Sdf.ValueTypeNames

HEAD_CAGE_LO = np.array([-16.5, -20.0, -2.3])
HEAD_CAGE_HI = np.array([16.5, 16.0, 2.1])
HEAD_CAGE_DIV = (7, 9, 2)
BODY_CAGE_LO = np.array([-25.0, -37.0, -1.5])
BODY_CAGE_HI = np.array([25.0, -11.0, -0.5])
BODY_CAGE_DIV = (6, 5, 2)

# group -> (kind for the head field, stack)
GROUPS = [
    ("BackHair", "back_hair", "hair"),
    ("Neck", "neck", "neck"),
    ("Body", None, "body"),
    ("Collar", None, "body"),
    ("Ribbon", None, "ribbon"),
    ("Ears", "ear", "head"),
    ("Earring", "ear", "hair"),
    ("Face", "face", "head"),
    ("FaceShade", "face", "head"),
    ("FaceLine", "face", "head"),
    ("EyeWhite", "face", "head"),
    ("Iris", "face", "head"),
    ("Catch", "face", "head"),
    ("EyeMask", "face", "head"),
    ("BlushPatch", "face", "head"),
    ("BlushHatch", "face", "head"),
    ("EyeLines", "face", "head"),
    ("Nose", "face", "head"),
    ("MouthInside", "face", "head"),
    ("MouthLines", "face", "head"),
    ("SideHair", "side_hair", "hair"),
    ("FrontHair", "front_hair", "hair"),
    ("Pins", "front_hair", "head"),
    ("Brows", "face", "head"),
    ("Fx", "face", "head"),
]
GROUP_KIND = {g: k for g, k, s in GROUPS}
GROUP_STACK = {g: s for g, k, s in GROUPS}
TRANSLUCENT = {"BlushPatch"}

JAW = dict(A=1.25, I=0.30, U=0.40, E=0.65, O=0.95)
JAW_YELL = 4.0
VISEME_PAD = dict(rest=(0.0, 0.0), A=(30.0, 0.0), I=(0.0, 30.0), U=(0.0, -30.0),
                  E=(21.0, 21.0), O=(21.0, -21.0))


# USD helpers

def define(stage, path, typename="", schemas=()):
    prim = stage.DefinePrim(path, typename)
    if schemas:
        op = Sdf.TokenListOp()
        op.prependedItems = list(schemas)
        prim.SetMetadata("apiSchemas", op)
    return prim


def scope(stage, path):
    return stage.DefinePrim(path, "Scope")


def attr(prim, name, tname, value=None, uniform=False):
    a = prim.CreateAttribute(name, tname, custom=False,
                             variability=Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying)
    if value is not None:
        a.Set(value)
    return a


def rel(prim, name, targets, phase=None):
    r = prim.CreateRelationship(name, custom=False)
    r.SetTargets([Sdf.Path(str(t)) for t in targets])
    if phase:
        r.SetMetadata("rigExecReadPhase", phase)
    return r


def translate(x, y, z=0.0):
    return Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(x, y, z))


def v3f(a):
    a = np.round(np.asarray(a, dtype=np.float64), 4).astype(np.float32)
    return Vt.Vec3fArray.FromNumpy(a)


def r4(a):
    return np.round(np.asarray(a, dtype=np.float64), 4)


# lattice helpers (the Bernstein basis the mover evaluates)

def bernstein_basis(P, lo, hi, div):
    from math import comb
    uvw = np.clip((P - lo) / (hi - lo), 0, 1)
    mats = []
    for a in range(3):
        n = div[a] - 1
        t = uvw[:, a:a + 1]
        k = np.arange(div[a])[None, :]
        c = np.array([comb(n, i) for i in range(div[a])], dtype=float)[None, :]
        mats.append(c * t ** k * (1 - t) ** (n - k))
    Bx, By, Bz = mats
    return (Bz[:, :, None, None] * By[:, None, :, None] * Bx[:, None, None, :]).reshape(len(P), -1)


def cage_points(lo, hi, div):
    xs = np.linspace(lo[0], hi[0], div[0])
    ys = np.linspace(lo[1], hi[1], div[1])
    zs = np.linspace(lo[2], hi[2], div[2])
    return np.array([(x, y, z) for z in zs for y in ys for x in xs])


def fit_cage(samples, disp, lo, hi, div, lam=2e-4, weights=None, fill=None, fill_weight=0.35):
    """Least-squares cage deltas reproducing ``disp`` at ``samples`` through
    the Bernstein lattice the mover evaluates. ``fill`` = (points, disp) of the
    same field on a regular grid over the whole cage, at a lower weight, so
    cage points far from the art stay calm (and the guide grid reads)."""
    B = bernstein_basis(samples, lo, hi, div)
    w = np.ones(len(samples)) if weights is None else weights
    rows = [B * w[:, None]]
    rhs = [disp[:, :2] * w[:, None]]
    if fill is not None:
        fp, fd = fill
        rows.append(fill_weight * bernstein_basis(fp, lo, hi, div))
        rhs.append(fill_weight * fd[:, :2])
    A = np.vstack(rows + [math.sqrt(lam) * np.eye(B.shape[1])])
    rhs = np.vstack(rhs + [np.zeros((B.shape[1], 2))])
    C, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    out = np.zeros((B.shape[1], 3))
    out[:, :2] = C
    return out


def lattice_apply(pts3, C, lo, hi, div):
    return bernstein_basis(pts3, lo, hi, div) @ C[:, :2]


# the body field (breath / turn / shrug), fitted onto the body cage

def invert_lattice(P, target, C, iters=8):
    """d such that (P + d) + lattice(P + d) == target, per point: a damped 2D
    Newton solve with a numerical Jacobian (the cage deltas C are smooth)."""
    lo, hi, div = HEAD_CAGE_LO, HEAD_CAGE_HI, HEAD_CAGE_DIV
    Q = P.copy()
    Q[:, :2] = target - lattice_apply(P, C, lo, hi, div)
    eps = 1e-3
    for _ in range(iters):
        L0 = lattice_apply(Q, C, lo, hi, div)
        F = Q[:, :2] + L0 - target
        Qx = Q.copy()
        Qx[:, 0] += eps
        Qy = Q.copy()
        Qy[:, 1] += eps
        Jx = (Qx[:, :2] + lattice_apply(Qx, C, lo, hi, div) - Q[:, :2] - L0) / eps
        Jy = (Qy[:, :2] + lattice_apply(Qy, C, lo, hi, div) - Q[:, :2] - L0) / eps
        det = Jx[:, 0] * Jy[:, 1] - Jy[:, 0] * Jx[:, 1]
        det = np.where(np.abs(det) < 0.05, np.sign(det + 1e-9) * 0.05, det)
        dx = (Jy[:, 1] * F[:, 0] - Jy[:, 0] * F[:, 1]) / det
        dy = (-Jx[:, 1] * F[:, 0] + Jx[:, 0] * F[:, 1]) / det
        step = np.stack([dx, dy], axis=1)
        n = np.linalg.norm(step, axis=1, keepdims=True)
        step = step * np.minimum(1.0, 1.0 / np.maximum(n, 1e-9))
        Q[:, :2] -= step
    return Q[:, :2] - P[:, :2]


def body_field(xyz, breath=0.0, turn=0.0, shrug=0.0):
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    d = np.zeros((len(xyz), 2))
    sm = art.smooth
    if breath:
        lift = 0.30 * sm(-30.0, -16.0, y) * (1.0 - 0.3 * sm(6.0, 20.0, np.abs(x)))
        widen = 0.012 * x * np.exp(-((y + 22.0) / 6.0) ** 2)
        d[:, 0] += breath * widen
        d[:, 1] += breath * lift
    if turn:
        th = math.radians(turn)
        r = 21.0
        phi = np.arcsin(np.clip(x / r, -1, 1))
        the = th * np.cos(phi) ** 0.9
        sx = r * np.sin(phi) * (np.cos(the) - 1) + 8.0 * np.cos(phi) * np.sin(the)
        d[:, 0] += (sx + (z + 1.0) * math.sin(th) * 2.0) * sm(-11.0, -15.0, y)
    if shrug:
        side = np.sign(shrug)
        k = abs(shrug)
        w = sm(2.0, 12.0, x * side) * sm(-30.0, -15.0, y)
        d[:, 1] += k * 1.1 * w
        d[:, 0] -= k * 0.25 * w * side
    return d


# the builder

class Builder(object):
    def __init__(self, path):
        if os.path.exists(path):
            os.remove(path)
        self.path = path
        self.stage = Usd.Stage.CreateNew(path)
        s = self.stage
        s.SetMetadata("upAxis", "Y")
        s.SetMetadata("metersPerUnit", 0.01)
        s.SetStartTimeCode(1)
        s.SetEndTimeCode(1)
        s.SetTimeCodesPerSecond(FPS)
        s.SetFramesPerSecond(FPS)
        root = UsdGeom.Xform.Define(s, ROOT)
        s.SetDefaultPrim(root.GetPrim())
        s.GetRootLayer().documentation = (
            "Kaede: a 2D mesh-deform anime bust (painted textures on ArtMesh-style "
            "meshes), rigged entirely with stock RigExec prims. Generated by "
            "build_bust.py -- edit that, not this file.")
        self.groups = {}

    def paint(self):
        self.pieces = PC.all_pieces()
        by = {}
        for p in self.pieces:
            by.setdefault(p.group, []).append(p)
        for g, kind, stack in GROUPS:
            assert g in by or g == "Body", g
        # shirt + cardigan share one mesh
        self.by_group = by
        self.piece = {p.name: p for p in self.pieces}

    def atlases(self):
        """Pack each group's piece textures into one atlas; UVs per piece."""
        import cv2
        tex_dir = os.path.join(HERE, "textures")
        os.makedirs(tex_dir, exist_ok=True)
        for f in os.listdir(tex_dir):
            if f.endswith(".png"):
                os.remove(os.path.join(tex_dir, f))
        self.uv = {}
        self.tex_file = {}
        PAD = 8
        for g, kind, stack in GROUPS:
            ps = [p for p in self.by_group.get(g, []) if "texture_of" not in p.meta]
            if not ps:
                continue
            # shelf packing, tallest first
            order = sorted(ps, key=lambda p: -p.rgba.shape[0])
            width = max(max(p.rgba.shape[1] for p in ps) + 2 * PAD, 256)
            total = sum((p.rgba.shape[0] + 2 * PAD) * (p.rgba.shape[1] + 2 * PAD) for p in ps)
            width = max(width, int(math.sqrt(total) * 1.15))
            x = y = shelf = 0
            place = {}
            for p in order:
                h, w = p.rgba.shape[:2]
                if x + w + 2 * PAD > width:
                    x = 0
                    y += shelf
                    shelf = 0
                place[p.name] = (x + PAD, y + PAD)
                x += w + 2 * PAD
                shelf = max(shelf, h + 2 * PAD)
            height = y + shelf
            atlas = np.zeros((height, width, 4), np.uint8)
            for p in ps:
                ax, ay = place[p.name]
                h, w = p.rgba.shape[:2]
                img = p.rgba
                # pad by edge-replicating the (already bled) colour, alpha 0
                padded = cv2.copyMakeBorder(img, PAD, PAD, PAD, PAD, cv2.BORDER_REPLICATE)
                padded[:PAD, :, 3] = 0
                padded[-PAD:, :, 3] = 0
                padded[:, :PAD, 3] = 0
                padded[:, -PAD:, 3] = 0
                atlas[ay - PAD:ay + h + PAD, ax - PAD:ax + w + PAD] = padded
                x0, y0, x1, y1 = p.bbox
                u = (ax + (p.ref[:, 0] - x0) / (x1 - x0) * w) / width
                v = 1.0 - (ay + (y1 - p.ref[:, 1]) / (y1 - y0) * h) / height
                self.uv[p.name] = np.stack([u, v], axis=1)
                p.meta["atlas"] = (ax, ay, w, h, width, height)
            fn = "%s.png" % g
            cv2.imwrite(os.path.join(tex_dir, fn), cv2.cvtColor(atlas, cv2.COLOR_RGBA2BGRA),
                        [cv2.IMWRITE_PNG_COMPRESSION, 9])
            self.tex_file[g] = "./textures/" + fn
        # pieces that sample another piece's texture (the eye masks)
        for p in self.pieces:
            if "texture_of" in p.meta:
                src = self.piece[p.meta["texture_of"]]
                ax, ay, w, h, width, height = src.meta["atlas"]
                x0, y0, x1, y1 = src.bbox
                u = (ax + (p.ref[:, 0] - x0) / (x1 - x0) * w) / width
                v = 1.0 - (ay + (y1 - p.ref[:, 1]) / (y1 - y0) * h) / height
                self.uv[p.name] = np.stack([u, v], axis=1)
                self.tex_file[p.group] = self.tex_file[src.group]

    def material(self, g):
        s = self.stage
        path = "%s/%s_mat" % (LOOKS, g)
        mat = UsdShade.Material.Define(s, path)
        sh = UsdShade.Shader.Define(s, path + "/Surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", T.Float).Set(1.0)
        sh.CreateInput("metallic", T.Float).Set(0.0)
        sh.CreateInput("useSpecularWorkflow", T.Int).Set(1)
        sh.CreateInput("specularColor", T.Color3f).Set(Gf.Vec3f(0, 0, 0))
        st = UsdShade.Shader.Define(s, path + "/UV")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", T.Token).Set("st")
        tx = UsdShade.Shader.Define(s, path + "/Texture")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", T.Asset).Set(self.tex_file[g])
        tx.CreateInput("sourceColorSpace", T.Token).Set("sRGB")
        tx.CreateInput("wrapS", T.Token).Set("clamp")
        tx.CreateInput("wrapT", T.Token).Set("clamp")
        tx.CreateInput("st", T.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        rgb = tx.CreateOutput("rgb", T.Float3)
        a = tx.CreateOutput("a", T.Float)
        sh.CreateInput("diffuseColor", T.Color3f).ConnectToSource(rgb)
        sh.CreateInput("opacity", T.Float).ConnectToSource(a)
        if g not in TRANSLUCENT:
            sh.CreateInput("opacityThreshold", T.Float).Set(0.5)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    def meshes(self):
        s = self.stage
        scope(s, GEOM)
        scope(s, LOOKS)
        self.mesh_rest = {}          # group -> (N x 3) rest points
        self.ranges = {}             # piece name -> (a, b) in its group's points
        for g, kind, stack in GROUPS:
            ps = self.by_group.get(g, [])
            if not ps:
                continue
            pts, tris, uvs = [], [], []
            n = 0
            for p in ps:
                r = p.rest
                z = p.zs()
                pts.append(np.column_stack([r, z]))
                tris.append(p.tris + n)
                uvs.append(self.uv[p.name])
                self.ranges[p.name] = (n, n + len(r))
                n += len(r)
            P = np.vstack(pts)
            Tr = np.vstack(tris)
            UV = np.vstack(uvs)
            mesh = UsdGeom.Mesh.Define(s, GEOM + "/" + g)
            mesh.CreatePointsAttr(v3f(P))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(Tr)))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(Tr.reshape(-1).astype(np.int32)))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            lo, hi = P.min(axis=0), P.max(axis=0)
            mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
            pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", T.TexCoord2fArray, UsdGeom.Tokens.vertex)
            pv.Set(Vt.Vec2fArray.FromNumpy(np.round(UV, 6).astype(np.float32)))
            mat = self.material(g)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
            self.mesh_rest[g] = r4(P)

    def group_pos(self, g, st):
        """(N x 2) positions of every point of group g at expression state st."""
        return np.vstack([p.pos(st) for p in self.by_group[g]])

    def backdrop(self):
        import backdrop
        s = self.stage
        path, bbox = backdrop.paint(os.path.join(HERE, "textures", "Backdrop.png"))
        x0, y0, x1, y1 = bbox
        z = -9.0
        mesh = UsdGeom.Mesh.Define(s, ROOT + "/Set/Backdrop")
        mesh.CreatePointsAttr(v3f([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y1, z)]))
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", T.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
        self.tex_file["Backdrop"] = "./textures/Backdrop.png"
        mat = self.material("Backdrop")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

    def control(self, path, pos_local, shape="sphere", scale=(0.5, 0.5, 0.5), offset=(0, 0, 0),
                color=(1.0, 0.85, 0.25), width=0.06):
        p = define(self.stage, path, "RigExecControl", ["RigExecControlAPI"])
        attr(p, "rest:space", T.Matrix4d, translate(*pos_local))
        attr(p, "purpose", T.Token, "guide", uniform=True)
        attr(p, "guide:shape", T.Token, shape, uniform=True)
        attr(p, "guide:scaleX", T.Double, float(scale[0]))
        attr(p, "guide:scaleY", T.Double, float(scale[1]))
        attr(p, "guide:scaleZ", T.Double, float(scale[2]))
        attr(p, "guide:offset", T.Double3, Gf.Vec3d(*offset))
        attr(p, "guide:displayColor", T.Color3f, Gf.Vec3f(*color))
        attr(p, "guide:wireWidth", T.Double, width / max(scale[0], 1e-3))
        return p

    def joint(self, path, pos_local, radius=0.3, color=(0.95, 0.35, 0.45), opacity=0.9):
        p = define(self.stage, path, "RigExecJoint")
        attr(p, "rest:space", T.Matrix4d, translate(*pos_local))
        attr(p, "guide:radius", T.Double, float(radius))
        attr(p, "guide:displayColor", T.Color3f, Gf.Vec3f(*color))
        attr(p, "guide:displayOpacity", T.Float, float(opacity))
        return p

    def fk(self, name, controls, joints):
        p = define(self.stage, RIG + "/Solvers/" + name, "RigExecFkChain")
        rel(p, "rigExec:controls", controls)
        rel(p, "rigExec:joints", joints)
        attr(p, "rigExec:controlSpace", T.Token, "parentRelative", uniform=True)
        attr(p, "guide:radius", T.Double, 0.0)
        return p

    def chain(self, name, pts, cparent, jparent, parent_pos, color_c, color_j, size=0.3):
        cpaths, jpaths = [], []
        prev = np.asarray(parent_pos, float)
        prev_j = np.asarray(self.joint_world[jparent], float)
        for k, p in enumerate(pts):
            p = np.asarray(p, float)
            cpath = cparent + "/%s_%d_ctl" % (name, k)
            lc = p - prev
            self.control(cpath, (lc[0], lc[1], 0.0), "sphere", (size, size, size), (0, 0, 0), color_c, 0.05)
            jpath = jparent + "/%s_%d" % (name, k)
            lj = p - prev_j
            self.joint(jpath, (lj[0], lj[1], 0.0), size * 0.7, color_j, 0.95)
            self.joint_world[jpath] = p
            cpaths.append(cpath)
            jpaths.append(jpath)
            cparent, jparent = cpath, jpath
            prev = p
            prev_j = p
        self.fk(name, cpaths, jpaths)
        self.chain_joints[name] = jpaths
        self.chain_controls[name] = cpaths
        self.chain_pts[name] = [np.asarray(p, float) for p in pts]

    def skeleton(self):
        s = self.stage
        rig = define(s, RIG, "RigExecRoot")
        attr(rig, "rigExec:partition", T.Token, "Kaede", uniform=True)
        for sc in ("Controls", "Joints", "Solvers", "BlendInputs", "PoseInterpolators", "Movers", "Guides"):
            scope(s, RIG + "/" + sc)
        ORANGE = (1.0, 0.62, 0.22)
        YELLOW = (1.0, 0.86, 0.22)
        AQUA = (0.30, 0.95, 0.88)
        PINK = (1.0, 0.45, 0.75)
        bx, by = D.BODY_PIVOT
        nx, ny = D.NECK_PIVOT
        hx, hy = D.HEAD_PIVOT
        self.joint_world = {}
        body_ctl = CTL + "/Body_ctl"
        neck_ctl = body_ctl + "/Neck_ctl"
        head_ctl = neck_ctl + "/Head_ctl"
        self.control(body_ctl, (bx, by, 0), "cube", (19.0, 0.9, 0.2), (0, 12.5, 0.0), ORANGE, 0.10)
        self.control(neck_ctl, (nx - bx, ny - by, 0), "cube", (4.2, 0.6, 0.2), (0, 0.0, 0), ORANGE, 0.08)
        self.control(head_ctl, (hx - nx, hy - ny, 0), "cube", (11.5, 12.5, 0.2), (0, 10.3, 0), ORANGE, 0.08)
        self.control(body_ctl + "/Torso_ctl", (0 - bx, -22.0 - by, 0), "diamond", (0.8, 0.8, 0.8), (0, 0, 0), YELLOW)
        for side, tag in ((1, "R"), (-1, "L")):
            self.control(body_ctl + "/Shoulder_%s_ctl" % tag, (15.5 * side - bx, -17.5 - by, 0), "diamond",
                         (0.6, 0.6, 0.6), (0, 0, 0), YELLOW)
        H = np.array([hx, hy])
        face = {
            "FaceAngle_ctl": ((0.0, -5.0), "sphere", (0.9, 0.9, 0.9), (0, 0, 4.5)),
            "Look_ctl": ((0.0, -2.6), "diamond", (0.5, 0.5, 0.5), (0, 0, 0)),
            "Eye_R_ctl": ((4.0, -0.8), "diamond", (0.4, 0.4, 0.4), (0, 0, 0)),
            "Eye_L_ctl": ((-4.0, -0.8), "diamond", (0.4, 0.4, 0.4), (0, 0, 0)),
            "Brow_R_ctl": ((4.0, 2.6), "diamond", (0.4, 0.4, 0.4), (0, 0, 0)),
            "Brow_L_ctl": ((-4.0, 2.6), "diamond", (0.4, 0.4, 0.4), (0, 0, 0)),
            "Mouth_ctl": ((0.0, -8.6), "sphere", (0.5, 0.5, 0.5), (0, 0, 2.2)),
            "Corner_R_ctl": ((2.2, -6.7), "diamond", (0.28, 0.28, 0.28), (0, 0, 0)),
            "Corner_L_ctl": ((-2.2, -6.7), "diamond", (0.28, 0.28, 0.28), (0, 0, 0)),
            "Fx_ctl": ((8.0, 8.5), "cube", (0.45, 0.45, 0.45), (0, 0, 0)),
        }
        for name, (pos, shape, sc, off) in face.items():
            local = np.array(pos) - H
            self.control(head_ctl + "/" + name, (local[0], local[1], 2.5), shape, sc, off, YELLOW)
        self.joint(JNT + "/Body", (bx, by, 0), 0.45, (1.0, 0.55, 0.25), 0.7)
        self.joint(JNT + "/Body/Neck", (nx - bx, ny - by, 0), 0.4, (1.0, 0.55, 0.25), 0.7)
        self.joint(JNT + "/Body/Neck/Head", (hx - nx, hy - ny, 0), 0.45, (1.0, 0.55, 0.25), 0.7)
        self.joint_world[JNT + "/Body"] = np.array([bx, by])
        self.joint_world[JNT + "/Body/Neck"] = np.array([nx, ny])
        self.joint_world[JNT + "/Body/Neck/Head"] = np.array([hx, hy])
        self.fk("Spine", [body_ctl, neck_ctl, head_ctl],
                [JNT + "/Body", JNT + "/Body/Neck", JNT + "/Body/Neck/Head"])
        self.head_ctl, self.body_ctl, self.neck_ctl = head_ctl, body_ctl, neck_ctl
        self.head_j = JNT + "/Body/Neck/Head"
        self.neck_j = JNT + "/Body/Neck"
        self.body_j = JNT + "/Body"

        # hair: one FK chain per lock, under the head handle
        self.chain_joints, self.chain_controls, self.chain_pts = {}, {}, {}
        self.chain_meta = {}
        for sc in ("Hair", "Accessories"):
            scope(s, JNT + "/" + sc)
            self.joint_world[JNT + "/" + sc] = np.zeros(2)
        for c in art_hair.clumps():
            js = c.joint_ts()
            if len(js) < 2:
                continue
            from art import sample_at
            pts, _ = sample_at(c.spine, np.array(js))
            big = c.kind == "back"
            self.chain("Hair_" + c.name, pts, head_ctl, JNT + "/Hair", H, AQUA,
                       (0.62, 0.40, 1.0), 0.42 if big else 0.30)
            self.chain_meta["Hair_" + c.name] = c
        # ribbon tails under the body
        loops, tails = art_body.ribbon_parts()
        for tname, (ctrl, widths) in tails.items():
            sp = self.piece["Ribbon_" + tname].meta["spine"]
            from art import sample_at
            pts, _ = sample_at(sp, np.array([0.0, 0.34, 0.68]))
            self.chain("Ribbon_" + tname, pts, body_ctl, JNT + "/Accessories", np.array([bx, by]), PINK,
                       (1.0, 0.5, 0.8), 0.26)
        # the earring
        self.chain("Earring", [np.array(D_) for D_ in art_body.EARRING[:2]], head_ctl, JNT + "/Accessories", H, PINK,
                   (1.0, 0.5, 0.8), 0.14)

    def blend_channel(self, group, channel, samples):
        """One RigExecBlendInput on group with sparse UsdSkelBlendShape samples
        [(activation, (N x 2) positions)]."""
        s = self.stage
        rest = self.mesh_rest[group][:, :2]
        bpath = "%s/%s/%s" % (BI, group, channel)
        b = define(s, bpath, "RigExecBlendInput")
        attr(b, "inputs:weight", T.Float, 0.0)
        spaths = []
        any_move = False
        for act, xy in samples:
            d = r4(np.asarray(xy)[:, :2] - rest)
            nz = np.nonzero(np.abs(d).max(axis=1) > 2e-4)[0]
            if len(nz):
                any_move = True
            else:
                nz = np.array([0])
            off = np.zeros((len(nz), 3))
            off[:, :2] = d[nz]
            tag = ("%g" % act).replace(".", "p").replace("-", "m")
            shape_path = "%s/%s/%s_%s" % (TARGETS, group, channel, tag)
            bs = UsdSkel.BlendShape.Define(s, shape_path)
            bs.CreateOffsetsAttr(v3f(off))
            bs.CreatePointIndicesAttr(Vt.IntArray.FromNumpy(nz.astype(np.int32)))
            sp = define(s, "%s/K_%s" % (bpath, tag), "RigExecBlendSample")
            attr(sp, "rigExec:activation", T.Float, float(act))
            rel(sp, "rigExec:blendShape", [shape_path])
            spaths.append(sp.GetPath())
        rel(b, "rigExec:samples", spaths)
        self.channels.setdefault(group, []).append(channel)
        return any_move

    def add_channel(self, channel, samples, groups):
        """samples [(activation, state)] evaluated on every group; groups where
        nothing moves are skipped. Returns the groups that got the channel."""
        got = []
        for g in groups:
            if g not in self.mesh_rest:
                continue
            rest = self.mesh_rest[g][:, :2]
            evald = [(a, self.group_pos(g, st) if not callable(st) else st(g)) for a, st in samples]
            if all(np.abs(xy - rest).max() < 2e-4 for a, xy in evald):
                continue
            self.blend_channel(g, channel, evald)
            got.append(g)
        self.channel_groups[channel] = got
        return got

    def keyforms(self):
        s = self.stage
        scope(s, TARGETS)
        for g in list(self.mesh_rest) + ["HeadCage", "BodyCage"]:
            scope(s, TARGETS + "/" + g)
            scope(s, BI + "/" + g)
        self.channels = {}
        self.channel_groups = {}
        st = art.state
        EYE_G = ["EyeWhite", "Iris", "Catch", "EyeMask", "EyeLines"]
        for side, tag in ((1, "R"), (-1, "L")):
            self.add_channel("Blink_" + tag, [(0.5, st(eye={side: "half"})), (1.0, st(eye={side: "closed"}))], EYE_G)
            self.add_channel("Wide_" + tag, [(1.0, st(eye={side: "wide"}))], EYE_G)
            self.add_channel("Smile_" + tag, [(20.0, st(eye={side: "smile"}))], EYE_G)
            self.add_channel("Squint_" + tag, [(1.0, st(eye={side: "squint"}))], EYE_G)
            self.add_channel("Flat_" + tag, [(20.0, st(eye={side: "flat"}))], EYE_G)
            for ch, shape, act in (("BrowUp", "up", 1.0), ("BrowDown", "down", 1.0),
                                   ("BrowSad", "sad", 20.0), ("BrowAngry", "angry", 20.0)):
                self.add_channel("%s_%s" % (ch, tag), [(act, st(brow={side: shape}))], ["Brows"])
        self.add_channel("Look_R", [(1.0, st(look=(1.0, 0.0)))], ["Iris", "Catch"])
        self.add_channel("Look_L", [(1.0, st(look=(-1.0, 0.0)))], ["Iris", "Catch"])
        self.add_channel("Look_U", [(1.0, st(look=(0.0, 1.0)))], ["Iris", "Catch"])
        self.add_channel("Look_D", [(1.0, st(look=(0.0, -1.0)))], ["Iris", "Catch"])
        self.add_channel("Shrink", [(1.0, st(shrink=1.0))], ["Iris", "Catch"])
        MOUTH_G = ["MouthInside", "MouthLines"]
        for v in "AIUEO":
            self.add_channel("Vis_" + v, [(1.0, st(mouth=v))], MOUTH_G)
            self.add_channel("Jaw_" + v, [(1.0, st(jaw=JAW[v]))], ["Face", "FaceLine"])
        self.add_channel("Smile", [(20.0, st(mouth="smile"))], MOUTH_G)
        self.add_channel("Frown", [(20.0, st(mouth="frown"))], MOUTH_G)
        self.add_channel("Corner_R", [(1.0, st(mouth="cornerR"))], MOUTH_G)
        self.add_channel("Corner_L", [(1.0, st(mouth="cornerL"))], MOUTH_G)
        self.add_channel("Shift_R", [(1.0, st(mouth="shiftR"))], MOUTH_G)
        self.add_channel("Shift_L", [(1.0, st(mouth="shiftL"))], MOUTH_G)
        self.add_channel("Pout", [(1.0, st(mouth="pout"))], MOUTH_G)
        self.add_channel("Yell", [(1.0, st(mouth="yell"))], MOUTH_G)
        self.add_channel("Jaw_Yell", [(1.0, st(jaw=JAW_YELL))], ["Face", "FaceLine"])
        self.add_channel("Blush", [(1.0, st(blush=1.0))], ["BlushPatch", "BlushHatch"])
        # the far cheek's side plane grows with the turn (the face turning to
        # screen right shows the plane on its screen-right side)
        self.add_channel("Plane_R", [(15.0, st(plane={1: 0.55})), (30.0, st(plane={1: 1.0}))], ["FaceShade"])
        self.add_channel("Plane_L", [(15.0, st(plane={-1: 0.55})), (30.0, st(plane={-1: 1.0}))], ["FaceShade"])
        self.add_channel("Vein", [(1.0, st(vein=1.0))], ["Fx"])

        def sweat_slide(g):
            xy = self.group_pos(g, st(sweat=1.0))
            a, b = self.ranges["Sweat"]
            xy[a:b] += np.array([0.25, -1.8])
            return xy
        self.add_channel("Sweat", [(0.5, st(sweat=1.0)), (1.0, sweat_slide)], ["Fx"])

    def cages(self):
        s = self.stage
        scope(s, CAGES)
        self.head_cage = cage_points(HEAD_CAGE_LO, HEAD_CAGE_HI, HEAD_CAGE_DIV)
        self.body_cage = cage_points(BODY_CAGE_LO, BODY_CAGE_HI, BODY_CAGE_DIV)
        for name, pts in (("HeadCage", self.head_cage), ("BodyCage", self.body_cage)):
            p = UsdGeom.Points.Define(s, CAGES + "/" + name)
            p.CreatePointsAttr(v3f(pts))
            p.CreateWidthsAttr(Vt.FloatArray([0.25] * len(pts)))
            p.CreateVisibilityAttr("invisible")
            p.CreatePurposeAttr("guide")
            self.mesh_rest[name] = r4(pts)
        head_groups = [g for g, k, st in GROUPS if k is not None and g in self.mesh_rest]
        samples = np.vstack([self.mesh_rest[g] for g in head_groups])
        kinds = np.concatenate([[GROUP_KIND[g]] * len(self.mesh_rest[g]) for g in head_groups])
        # the face counts double in the fit: it is what the eye reads first
        wts = np.where(np.isin(kinds, ["face"]), 1.5, 1.0)
        self.fit_report = {}
        head_ch = {}
        self.head_C = {}
        for ch, sgn, axis, acts in (("Turn_R", 1, 0, (15.0, 30.0)), ("Turn_L", -1, 0, (15.0, 30.0)),
                                    ("Nod_Up", 1, 1, (10.0, 20.0)), ("Nod_Down", -1, 1, (10.0, 20.0))):
            ks = []
            for a in acts:
                ang = (sgn * a, 0.0) if axis == 0 else (0.0, sgn * a)
                env = field.face_envelope(ang[0]) if ang[0] else None
                disp = np.zeros((len(samples), 2))
                n = 0
                exact = {}
                for g in head_groups:
                    P = self.mesh_rest[g]
                    d = field.head_field(P[:, :2], GROUP_KIND[g], ang[0], ang[1],
                                         env if GROUP_KIND[g] == "face" else None)
                    exact[g] = d
                    disp[n:n + len(P)] = field.head_field(P[:, :2], GROUP_KIND[g], ang[0], ang[1], soft=True)
                    n += len(P)
                g = np.stack(np.meshgrid(np.linspace(HEAD_CAGE_LO[0], HEAD_CAGE_HI[0], 34),
                                         np.linspace(HEAD_CAGE_LO[1], HEAD_CAGE_HI[1], 36),
                                         np.linspace(HEAD_CAGE_LO[2], HEAD_CAGE_HI[2], 2)), axis=-1).reshape(-1, 3)
                gd = np.where((g[:, 2] > 0)[:, None],
                              field.head_field(g[:, :2], "front_hair", ang[0], ang[1]),
                              field.head_field(g[:, :2], "back_hair", ang[0], ang[1]))
                C = fit_cage(samples, disp, HEAD_CAGE_LO, HEAD_CAGE_HI, HEAD_CAGE_DIV, weights=wts, fill=(g, gd))
                err = np.linalg.norm(lattice_apply(samples, C, HEAD_CAGE_LO, HEAD_CAGE_HI, HEAD_CAGE_DIV) - disp, axis=1)
                self.fit_report["%s@%g" % (ch, a)] = (float(err.mean()), float(err.max()))
                ks.append((a, self.head_cage + C, exact, C, ang, env))
            head_ch[ch] = ks
        for ch, ks in head_ch.items():
            self.blend_channel("HeadCage", ch, [(x[0], x[1]) for x in ks])
        # correctives on the face-surface meshes: exact - lattice (fixed-point
        # iterations so the lattice applied to the corrected rest lands
        # on the exact target)
        self.corrective_groups = {}
        face_groups = [g for g in head_groups if GROUP_KIND[g] == "face"]
        for ch, ks in head_ch.items():
            cname = "Face" + ch
            for g in face_groups:
                P = self.mesh_rest[g]
                samples_g = []
                for a, cage, exact, C, ang, env in ks:
                    target = self.restroke(g, P[:, :2] + exact[g], ang, env)
                    d = invert_lattice(P, target, C)
                    samples_g.append((a, P[:, :2] + d))
                    if os.environ.get("KAEDE_DEBUG") and a == 30.0:
                        Q = P.copy()
                        Q[:, :2] += d
                        res = np.linalg.norm(Q[:, :2] + lattice_apply(Q, C, HEAD_CAGE_LO, HEAD_CAGE_HI,
                                                                       HEAD_CAGE_DIV) - target, axis=1)
                        print("    %s %-11s residual max %.3f p99 %.3f" % (ch, g, res.max(), np.percentile(res, 99)))
                if max(np.abs(xy - P[:, :2]).max() for a, xy in samples_g) < 2e-3:
                    continue
                self.blend_channel(g, cname, samples_g)
                self.corrective_groups.setdefault(cname, []).append(g)
        # body cage
        body_groups = ["Body", "Collar", "Ribbon", "Neck"]
        bs = np.vstack([self.mesh_rest[g] for g in body_groups])
        body = {}
        for ch, kw, act in (("Breath", dict(breath=1.0), 1.0), ("BodyTurn_R", dict(turn=10.0), 10.0),
                            ("BodyTurn_L", dict(turn=-10.0), 10.0), ("Shrug_R", dict(shrug=1.0), 1.0),
                            ("Shrug_L", dict(shrug=-1.0), 1.0)):
            disp = body_field(bs, **kw)
            g = np.stack(np.meshgrid(np.linspace(BODY_CAGE_LO[0], BODY_CAGE_HI[0], 30),
                                     np.linspace(BODY_CAGE_LO[1], BODY_CAGE_HI[1], 20),
                                     np.linspace(BODY_CAGE_LO[2], BODY_CAGE_HI[2], 2)), axis=-1).reshape(-1, 3)
            C = fit_cage(bs, disp, BODY_CAGE_LO, BODY_CAGE_HI, BODY_CAGE_DIV, fill=(g, body_field(g, **kw)))
            body[ch] = [(act, self.body_cage + C)]
        for ch, ks in body.items():
            self.blend_channel("BodyCage", ch, ks)

    def restroke(self, g, target, ang, env):
        """Line strips keep their width through the turn: move each strip's
        SPINE with the head field and rebuild its rows at the same offsets
        along the new normals (fills just follow the field)."""
        from art import arc_param
        target = target.copy()
        for p in self.by_group[g]:
            strip = p.meta.get("strip")
            if strip is None:
                continue
            a, b = self.ranges[p.name]
            sp = np.asarray(strip.spine_fn(art.state()), np.float64)
            s0 = arc_param(sp)
            sp2 = sp + field.head_field(sp, GROUP_KIND[g], ang[0], ang[1],
                                        env if GROUP_KIND[g] == "face" else None)
            tan = np.gradient(sp2, axis=0)
            tan /= np.linalg.norm(tan, axis=1, keepdims=True) + 1e-12
            P0 = np.stack([np.interp(strip.ts, s0, sp2[:, 0]), np.interp(strip.ts, s0, sp2[:, 1])], axis=1)
            tx = np.interp(strip.ts, s0, tan[:, 0])
            ty = np.interp(strip.ts, s0, tan[:, 1])
            nrm = np.hypot(tx, ty) + 1e-12
            N = np.stack([-ty / nrm, tx / nrm], axis=1) * strip.up
            pts = P0[:, None, :] + N[:, None, :] * strip.vs[None, :, None]
            target[a:b] = pts.reshape(-1, 2)
        return target

    def lattice_wire(self, name, lo, hi, nx, ny, z, color):
        s = self.stage
        xs = np.linspace(lo[0], hi[0], nx)
        ys = np.linspace(lo[1], hi[1], ny)
        pts, counts = [], []
        for x in xs:
            col = [(x, y, z) for y in np.linspace(ys[0], ys[-1], 48)]
            pts += col
            counts.append(len(col))
        for y in ys:
            row = [(x, y, z) for x in np.linspace(xs[0], xs[-1], 40)]
            pts += row
            counts.append(len(row))
        c = UsdGeom.BasisCurves.Define(s, RIG + "/Guides/" + name)
        c.CreateTypeAttr("linear")
        c.CreatePointsAttr(v3f(pts))
        c.CreateCurveVertexCountsAttr(Vt.IntArray(counts))
        c.CreateWidthsAttr(Vt.FloatArray([0.10]))
        c.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        c.CreatePurposeAttr("guide")
        c.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        c.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant).Set(Vt.FloatArray([0.8]))
        self.wire_rest[name] = r4(np.array(pts))

    def wiring(self):
        s = self.stage
        H = self.head_ctl
        B = self.body_ctl
        scope(s, MOV + "/Channels")
        avar = {
            "FaceAngle": H + "/FaceAngle_ctl", "Look": H + "/Look_ctl",
            "Eye_R": H + "/Eye_R_ctl", "Eye_L": H + "/Eye_L_ctl",
            "Brow_R": H + "/Brow_R_ctl", "Brow_L": H + "/Brow_L_ctl",
            "Mouth": H + "/Mouth_ctl", "Corner_R": H + "/Corner_R_ctl", "Corner_L": H + "/Corner_L_ctl",
            "Fx": H + "/Fx_ctl", "Torso": B + "/Torso_ctl",
            "Shoulder_R": B + "/Shoulder_R_ctl", "Shoulder_L": B + "/Shoulder_L_ctl",
        }
        self.avar_ctl = avar

        def weights(ch, groups):
            return ["%s/%s/%s.inputs:weight" % (BI, g, ch) for g in groups]

        def connect(ch, groups, ctl, a):
            for g in groups:
                w = s.GetPrimAtPath("%s/%s/%s" % (BI, g, ch)).GetAttribute("inputs:weight")
                w.SetConnections([Sdf.Path("%s.avars:%s" % (avar[ctl], a))])

        def negated(ch, groups, ctl, a):
            """weight = -avar: one pair of float-math movers per target."""
            for g in groups:
                target = "%s/%s/%s.inputs:weight" % (BI, g, ch)
                base = "%s/Channels/%s_%s" % (MOV, g, ch)
                scope(s, base)
                read = define(s, base + "/Read", "RigExecFloatMathMover", ["RigExecMoverAPI"])
                attr(read, "rigExec:operation", T.Token, "add", uniform=True)
                v = attr(read, "inputs:value", T.Float, 0.0)
                v.SetConnections([Sdf.Path("%s.avars:%s" % (avar[ctl], a))])
                rel(read, "rigExec:moves", [target])
                neg = define(s, base + "/Negate", "RigExecFloatMathMover", ["RigExecMoverAPI"])
                attr(neg, "rigExec:operation", T.Token, "multiply", uniform=True)
                attr(neg, "inputs:value", T.Float, -1.0)
                rel(neg, "rigExec:moves", [target])
                s.GetPrimAtPath(base).SetChildrenReorder(["Negate", "Read"])

        cg = self.channel_groups
        connect("Turn_R", ["HeadCage"], "FaceAngle", "ry")
        negated("Turn_L", ["HeadCage"], "FaceAngle", "ry")
        # +rx pitches the face DOWN
        connect("Nod_Down", ["HeadCage"], "FaceAngle", "rx")
        negated("Nod_Up", ["HeadCage"], "FaceAngle", "rx")
        connect("FaceTurn_R", self.corrective_groups.get("FaceTurn_R", []), "FaceAngle", "ry")
        negated("FaceTurn_L", self.corrective_groups.get("FaceTurn_L", []), "FaceAngle", "ry")
        connect("FaceNod_Down", self.corrective_groups.get("FaceNod_Down", []), "FaceAngle", "rx")
        negated("FaceNod_Up", self.corrective_groups.get("FaceNod_Up", []), "FaceAngle", "rx")
        connect("Breath", ["BodyCage"], "Torso", "ty")
        connect("BodyTurn_R", ["BodyCage"], "Torso", "ry")
        negated("BodyTurn_L", ["BodyCage"], "Torso", "ry")
        connect("Shrug_R", ["BodyCage"], "Shoulder_R", "ty")
        connect("Shrug_L", ["BodyCage"], "Shoulder_L", "ty")
        for tag in ("R", "L"):
            negated("Blink_" + tag, cg["Blink_" + tag], "Eye_" + tag, "ty")
            connect("Wide_" + tag, cg["Wide_" + tag], "Eye_" + tag, "ty")
            connect("Squint_" + tag, cg["Squint_" + tag], "Eye_" + tag, "tx")
            connect("Smile_" + tag, cg["Smile_" + tag], "Eye_" + tag, "rz")
            connect("Flat_" + tag, cg["Flat_" + tag], "Eye_" + tag, "rx")
            connect("BrowUp_" + tag, cg["BrowUp_" + tag], "Brow_" + tag, "ty")
            negated("BrowDown_" + tag, cg["BrowDown_" + tag], "Brow_" + tag, "ty")
            connect("BrowSad_" + tag, cg["BrowSad_" + tag], "Brow_" + tag, "rz")
            negated("BrowAngry_" + tag, cg["BrowAngry_" + tag], "Brow_" + tag, "rz")
            connect("Corner_" + tag, cg["Corner_" + tag], "Corner_" + tag, "ty")
        connect("Look_R", cg["Look_R"], "Look", "tx")
        negated("Look_L", cg["Look_L"], "Look", "tx")
        connect("Look_U", cg["Look_U"], "Look", "ty")
        negated("Look_D", cg["Look_D"], "Look", "ty")
        connect("Shrink", cg["Shrink"], "Look", "tz")
        connect("Smile", cg["Smile"], "Mouth", "rz")
        negated("Frown", cg["Frown"], "Mouth", "rz")
        connect("Shift_R", cg["Shift_R"], "Mouth", "tx")
        negated("Shift_L", cg["Shift_L"], "Mouth", "tx")
        connect("Pout", cg["Pout"], "Mouth", "ty")
        negated("Yell", cg["Yell"], "Mouth", "ty")
        negated("Jaw_Yell", cg["Jaw_Yell"], "Mouth", "ty")
        connect("Blush", cg["Blush"], "Fx", "tx")
        connect("Plane_R", cg["Plane_R"], "FaceAngle", "ry")
        negated("Plane_L", cg["Plane_L"], "FaceAngle", "ry")
        connect("Vein", cg["Vein"], "Fx", "ty")
        connect("Sweat", cg["Sweat"], "Fx", "tz")

        # the vowel pad: a pose-space RBF over the mouth handle's swing; every
        # vowel pose's weight fans out to the lip keyforms and the jaw keyforms
        pi = define(s, RIG + "/PoseInterpolators/Visemes", "RigExecPoseInterpolator")
        rel(pi, "rigExec:driver", [avar["Mouth"]])
        attr(pi, "rigExec:enableRotation", T.Bool, True)
        attr(pi, "rigExec:enableTranslation", T.Bool, False)
        attr(pi, "rigExec:kernel", T.Token, "gaussian", uniform=True)
        attr(pi, "rigExec:normalize", T.Bool, True)
        attr(pi, "rigExec:allowNegativeWeights", T.Bool, False)
        attr(pi, "rigExec:regularization", T.Float, 0.0)
        attr(pi, "rigExec:twistAxis", T.Token, "Z", uniform=True)
        for name, (rx, ry) in VISEME_PAD.items():
            p = define(s, RIG + "/PoseInterpolators/Visemes/" + name, "RigExecPose")
            q = (Gf.Rotation(Gf.Vec3d(1, 0, 0), rx) * Gf.Rotation(Gf.Vec3d(0, 1, 0), ry)).GetQuat()
            attr(p, "rigExec:poseType", T.Token, "swing", uniform=True)
            attr(p, "rigExec:rotation", T.Quatf, Gf.Quatf(q))
            attr(p, "rigExec:rotationRadius", T.Float, 0.42)
            attr(p, "outputs:weight", T.Float, 0.0)
            if name != "rest":
                rel(p, "rigExec:poseControls", [avar["Mouth"] + ".avars:rx", avar["Mouth"] + ".avars:ry"])
                attr(p, "rigExec:poseControlValues", T.DoubleArray, Vt.DoubleArray([rx, ry]))
                for g in cg["Vis_" + name] + cg["Jaw_" + name]:
                    ch = ("Vis_" if g in ("MouthInside", "MouthLines") else "Jaw_") + name
                    w = s.GetPrimAtPath("%s/%s/%s" % (BI, g, ch)).GetAttribute("inputs:weight")
                    w.SetConnections([p.GetPath().AppendProperty("outputs:weight")])

    def hair_skin(self, g):
        """Influences and 4-per-point weights of a hair group: every piece's
        points follow its clump's chain by their position along the spine,
        glued to the head at the root."""
        from art import sample_at
        n = len(self.mesh_rest[g])
        infl = [self.head_j]
        index = {}
        ES = 4
        idx = np.zeros((n, ES), np.int64)
        wts = np.zeros((n, ES))
        wts[:, 0] = 1.0
        for p in self.by_group[g]:
            a, b = self.ranges[p.name]
            c = p.meta.get("clump")
            cname = "Hair_" + c.name if c is not None else p.name
            if cname not in self.chain_joints:
                continue
            for jp in self.chain_joints[cname]:
                if jp not in index:
                    index[jp] = len(infl)
                    infl.append(jp)
            joints = [index[j] for j in self.chain_joints[cname]]
            ts = c.joint_ts()
            # t of every point: nearest spine sample
            sp = c.spine
            L = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(sp, axis=0), axis=1))])
            L /= L[-1]
            pts = p.rest
            dd = ((pts[:, None, :] - sp[None, :, :]) ** 2).sum(-1)
            t = L[np.argmin(dd, axis=1)]
            W = chain_bone_weights(t, ts)
            glue = 1.0 - art.smooth(0.0, 0.10 if c.kind != "back" else 0.18, t)
            for i in range(b - a):
                row = [(0, glue[i])]
                order = np.argsort(-W[i])[:ES - 1]
                for j in order:
                    row.append((joints[j], (1 - glue[i]) * W[i, j]))
                tot = sum(w for _, w in row)
                for k, (ji, w) in enumerate(row):
                    idx[a + i, k] = ji
                    wts[a + i, k] = w / tot
        return infl, idx, wts, ES

    def movers(self):
        s = self.stage
        G = MOV + "/Geometry"
        scope(s, G)
        scope(s, MOV + "/Cages")

        def mover(path, typename):
            return define(s, path, typename, ["RigExecMoverAPI"])

        def lattice(path, target, cage, div):
            m = mover(path, "RigExecLatticeMover")
            attr(m, "rigExec:basis", T.Token, "bernstein", uniform=True)
            rel(m, "rigExec:cage", [CAGES + "/" + cage], phase="final")
            attr(m, "rigExec:divisions", T.Int3, Gf.Vec3i(*div))
            rel(m, "rigExec:moves", [target + ".points"])
            return m

        def matrix(path, target, joint):
            m = mover(path, "RigExecMatrixMover")
            rel(m, "rigExec:transform", [joint])
            attr(m, "rigExec:transformReadPhase", T.Token, "final", uniform=True)
            rel(m, "rigExec:moves", [target + ".points"])
            return m

        def skin(path, target, influences, idx, wts, esize):
            m = mover(path, "RigExecSkinMover")
            rel(m, "rigExec:influences", influences)
            attr(m, "rigExec:jointIndices", T.IntArray, Vt.IntArray.FromNumpy(idx.reshape(-1).astype(np.int32)))
            attr(m, "rigExec:jointWeights", T.FloatArray,
                 Vt.FloatArray.FromNumpy(np.round(wts.reshape(-1), 4).astype(np.float32)))
            attr(m, "rigExec:elementSize", T.Int, esize, uniform=True)
            attr(m, "rigExec:skinningMethod", T.Token, "classicLinear", uniform=True)
            attr(m, "rigExec:transformReadPhase", T.Token, "final", uniform=True)
            rel(m, "rigExec:moves", [target + ".points"])
            return m

        def shapes(path, target, g):
            m = mover(path, "RigExecBlendShapeMover")
            rel(m, "rigExec:blendInputs", ["%s/%s/%s" % (BI, g, ch) for ch in self.channels[g]])
            rel(m, "rigExec:moves", [target + ".points"])
            return m

        for cage in ("HeadCage", "BodyCage"):
            m = mover("%s/Cages/%sKeyforms" % (MOV, cage), "RigExecBlendShapeMover")
            rel(m, "rigExec:blendInputs", ["%s/%s/%s" % (BI, cage, ch) for ch in self.channels[cage]])
            rel(m, "rigExec:moves", [CAGES + "/" + cage + ".points"])

        order = []
        for g, kind, stack in GROUPS:
            if g not in self.mesh_rest:
                continue
            base = G + "/" + g
            scope(s, base)
            tgt = GEOM + "/" + g
            kids = []
            if stack == "head":
                lattice(base + "/HeadWarp", tgt, "HeadCage", HEAD_CAGE_DIV)
                matrix(base + "/HeadTilt", tgt, self.head_j)
                kids = ["HeadWarp", "HeadTilt"]
                if g in self.channels:
                    shapes(base + "/Keyforms", tgt, g)
                    kids.append("Keyforms")
            elif stack == "hair":
                infl, idx, wts, ES = self.hair_skin(g) if g != "Earring" else self.earring_skin()
                lattice(base + "/HeadWarp", tgt, "HeadCage", HEAD_CAGE_DIV)
                skin(base + "/HairSkin", tgt, infl, idx, wts, ES)
                kids = ["HeadWarp", "HairSkin"]
            elif stack == "neck":
                y = self.mesh_rest[g][:, 1]
                wh = art.smooth(-11.6, -8.6, y)
                wb = art.smooth(-13.6, -16.5, y)
                wn = np.clip(1.0 - wh - wb, 0, 1)
                W = np.stack([wb, wn, wh], axis=1)
                W /= W.sum(axis=1, keepdims=True)
                idx = np.tile(np.array([0, 1, 2]), (len(y), 1))
                lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
                lattice(base + "/HeadWarp", tgt, "HeadCage", HEAD_CAGE_DIV)
                skin(base + "/SpineSkin", tgt, [self.body_j, self.neck_j, self.head_j], idx, W, 3)
                kids = ["BodyWarp", "HeadWarp", "SpineSkin"]
            elif stack == "body":
                lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
                matrix(base + "/Lean", tgt, self.body_j)
                kids = ["BodyWarp", "Lean"]
            elif stack == "ribbon":
                infl, idx, wts, ES = self.ribbon_skin()
                lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
                skin(base + "/RibbonSkin", tgt, infl, idx, wts, ES)
                kids = ["BodyWarp", "RibbonSkin"]
            s.GetPrimAtPath(base).SetChildrenReorder(kids)
            order.append(g)
        # lattice guide wires ride the same stacks
        for name, cage, div, joint in (("HeadLatticeWire", "HeadCage", HEAD_CAGE_DIV, self.head_j),
                                       ("BodyLatticeWire", "BodyCage", BODY_CAGE_DIV, self.body_j)):
            base = G + "/" + name
            scope(s, base)
            tgt = RIG + "/Guides/" + name
            lattice(base + "/Warp", tgt, cage, div)
            matrix(base + "/Follow", tgt, joint)
            s.GetPrimAtPath(base).SetChildrenReorder(["Warp", "Follow"])
            order.append(name)
        s.GetPrimAtPath(G).SetChildrenReorder(order)
        s.GetPrimAtPath(MOV).SetChildrenReorder(["Geometry", "Cages", "Channels"])

    def ribbon_skin(self):
        g = "Ribbon"
        n = len(self.mesh_rest[g])
        infl = [self.body_j]
        ES = 3
        idx = np.zeros((n, ES), np.int64)
        wts = np.zeros((n, ES))
        wts[:, 0] = 1.0
        for tname in ("TailL", "TailR"):
            cname = "Ribbon_" + tname
            p = self.piece[cname]
            a, b = self.ranges[cname]
            base = len(infl)
            infl += self.chain_joints[cname]
            sp = p.meta["spine"]
            L = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(sp, axis=0), axis=1))])
            L /= L[-1]
            dd = ((p.rest[:, None, :] - sp[None, :, :]) ** 2).sum(-1)
            t = L[np.argmin(dd, axis=1)]
            W = chain_bone_weights(t, [0.0, 0.34, 0.68])
            glue = 1.0 - art.smooth(0.0, 0.12, t)
            for i in range(b - a):
                order = np.argsort(-W[i])[:ES - 1]
                row = [(0, glue[i])] + [(base + j, (1 - glue[i]) * W[i, j]) for j in order]
                tot = sum(w for _, w in row)
                for k, (ji, w) in enumerate(row):
                    idx[a + i, k] = ji
                    wts[a + i, k] = w / tot
        return infl, idx, wts, ES

    def earring_skin(self):
        g = "Earring"
        P = self.mesh_rest[g]
        n = len(P)
        infl = [self.head_j] + self.chain_joints["Earring"]
        y = P[:, 1]
        y0, y1 = art_body.EARRING[0][1], art_body.EARRING[1][1]
        t = np.clip((y0 - y) / (y0 - y1), 0, 2)
        w1 = art.smooth(0.0, 0.25, t)
        w2 = art.smooth(0.8, 1.1, t)
        W = np.stack([1 - w1, w1 - w2, w2], axis=1)
        W = np.clip(W, 0, 1)
        W /= W.sum(axis=1, keepdims=True)
        idx = np.tile(np.array([0, 1, 2]), (n, 1))
        return infl, idx, W, 3

    def save(self):
        self.stage.GetRootLayer().Save()


def chain_bone_weights(t, ts):
    ts = list(ts) + [1.0]
    centres = [(ts[k] + ts[k + 1]) * 0.5 for k in range(len(ts) - 1)]
    m = len(centres)
    W = np.zeros((len(t), m))
    for i, tv in enumerate(t):
        if tv <= centres[0]:
            W[i, 0] = 1.0
        elif tv >= centres[-1]:
            W[i, -1] = 1.0
        else:
            for k in range(m - 1):
                if centres[k] <= tv <= centres[k + 1]:
                    s = (tv - centres[k]) / (centres[k + 1] - centres[k])
                    s = s * s * (3 - 2 * s)
                    W[i, k] = 1 - s
                    W[i, k + 1] = s
                    break
    return W


def build():
    import perform
    rig_path = os.path.join(HERE, NAME + "_rig.usda")
    b = Builder(rig_path)
    b.paint()
    b.atlases()
    b.meshes()
    b.backdrop()
    b.wire_rest = {}
    b.lattice_wire("HeadLatticeWire", (-13.5, -13.0), (13.5, 14.5), 12, 13, 2.0, (0.18, 0.80, 1.0))
    b.lattice_wire("BodyLatticeWire", (-22.0, -34.0), (22.0, -14.5), 13, 7, -0.55, (0.55, 0.45, 1.0))
    b.skeleton()
    b.keyforms()
    b.cages()
    b.wiring()
    b.movers()
    b.save()
    for k, (mean, mx) in sorted(b.fit_report.items()):
        print("  head cage fit %-14s mean %.3f  max %.3f" % (k, mean, mx))
    npts = sum(len(v) for k, v in b.mesh_rest.items() if not k.endswith("Cage"))
    print("wrote %s (%d art points in %d meshes)" % (rig_path, npts, len(b.by_group)))
    perform.write_all(b, HERE, NAME)
    # a layout sidecar for the QA scripts (outside the repo)
    import json
    qa = os.environ.get("RIGEXEC_PREVIEW_DIR", os.path.join(os.getcwd(), "build", "previews")) + "/bust_dd_a"
    if os.path.isdir(os.path.dirname(qa)):
        os.makedirs(qa, exist_ok=True)
        lay = {}
        for p in b.pieces:
            st = p.meta.get("strip")
            a, bb = b.ranges[p.name]
            lay.setdefault(p.group, []).append(dict(name=p.name, a=a, b=bb,
                                                    nt=len(st.ts) if st else 0, k=len(st.vs) if st else 0))
        json.dump(lay, open(qa + "/layout.json", "w"), indent=1)
    return b


if __name__ == "__main__":
    build()
