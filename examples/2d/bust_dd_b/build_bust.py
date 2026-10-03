"""Build Shion -- a 2D mesh-deform (layered mesh) anime bust rigged entirely
with stock RigExec prims.

    cd <repo> && source bin/_env.sh
    "$PY" examples/2d/bust_dd_b/build_bust.py

writes, next to this file,

    textures/*.png            the painted art (one RGBA texture per layer)
    bust_dd_b_rig.usda        the character at rest: textured art meshes + the rig
    bust_dd_b_anim.usda       the 240-frame performance (control avars only)
    bust_dd_b_sweep.usda      the 320-frame parameter sweep for the rig reveal

Deterministic. Nothing is baked: every frame is evaluated live by RigExec
from the control avars.

RIG STRUCTURE

* PARAMETERS are RigExecControls; their avars are the parameter values.
* ART MESHES are UsdGeomMeshes with a UsdPreviewSurface texture per layer
  (GeomSubsets bind one material per layer inside a merged mesh).
* KEYFORMS are RigExecBlendSamples on RigExecBlendInputs, one channel per
  parameter direction, in-betweens where a parameter needs them (blink half
  at 0.5; the mouth at 0.5 / 1.0 / 1.4); bilinear corner CORRECTIVES
  (blink x smile, open x wide/round/smile/frown) take the product of two
  parameters, computed by RigExecFloatMathMover chains.
* The WARP DEFORMER for the head angle is a RigExecLatticeMover (a Bernstein
  cage whose keyforms at +-15/30 turn and +-10/20 nod are FITTED to a
  part-specific projections in turn.py); the part of the target the
  cage cannot reach -- the jaw redraw, coherent eye planes and hair
  volumes -- is a per-mesh residual keyform on the same parameter. The
  cage's z axis spans the layers' depths, so the one warp gives every layer
  its own parallax. A second cage around the torso carries breathing and
  the body turn.
* ROTATION DEFORMERS are FK chains: body -> neck -> head (AngleZ, body lean)
  and one chain per lock of hair, the earring and the ribbon tails, skinned
  with RigExecSkinMovers. The hair/earring/ribbon avars are keyed from an
  offline damped-spring solve in the anim layers (secondary motion).
"""

import math
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import art_body as AB  # noqa: E402
import art_face as AF  # noqa: E402
import art_hair as AH  # noqa: E402
import art_limbs as AL  # noqa: E402
import design as D  # noqa: E402
import paint as P  # noqa: E402
import parts as PT  # noqa: E402
import state as ST  # noqa: E402
import turn as TU  # noqa: E402
from parts import ribbon_points  # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdSkel, Vt  # noqa: E402

ROOT = "/Shion"
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

HEAD_CAGE_LO = np.array([-14.5, -13.0, -3.1])
HEAD_CAGE_HI = np.array([14.5, 14.5, 1.0])
HEAD_CAGE_DIV = (6, 7, 3)
BODY_CAGE_LO = np.array([-40.0, -61.0, -2.1])
BODY_CAGE_HI = np.array([40.0, -12.0, -0.8])
BODY_CAGE_DIV = (6, 5, 2)

# which layers go into which mesh (one mesh per mover stack)

MESHES = [
    ("BackHair", ["BackHair"]),
    ("Body", ["CardiganBack", "CollarBack", "Shirt", "Cardigan", "Collar", "RibbonKnot"]),
    ("Ribbon", ["RibbonTailL", "RibbonTailR"]),
    ("Neck", ["Neck"]),
    ("Face", ["Ear", "Face", "BlushBase_R", "BlushBase_L", "FaceLine", "BlushPop_R", "BlushPop_L",
              "Hatch_R", "Hatch_L", "Sweat"]),
    ("Earring", ["Earring"]),
    ("Eye_R", ["Sclera_R", "LidShadow_R", "Iris_R", "Highlight_R", "EyeMask_R", "LowerLid_R", "Crease_R", "Lash_R"]),
    ("Eye_L", ["Sclera_L", "LidShadow_L", "Iris_L", "Highlight_L", "EyeMask_L", "LowerLid_L", "Crease_L", "Lash_L"]),
    ("Mouth", ["MouthInside", "Tongue", "Teeth", "LowerLip", "LowerEdge", "MouthLine", "TongueOut"]),
    ("Brow_R", ["Brow_R"]),
    ("Brow_L", ["Brow_L"]),
    ("Hair", ["TuckL", "Ahoge", "Crown", "LooseL", "LockR_A", "LockR_B", "F4", "F5", "F3", "F2", "F1", "Stray", "Pins"]),
]
for _tag in ("L", "R"):
    MESHES += [("Arm_" + _tag, ["UpperSleeve_" + _tag, "ForeSleeve_" + _tag]),
               ("Hand_" + _tag, [n + "_" + _tag for n in AL.FINGERS] + ["Palm_" + _tag] +
                [n + "Fold_" + _tag for n in AL.FOLDED_PATHS])]
HEAD_MESHES = ["BackHair", "Face", "Earring", "Eye_R", "Eye_L", "Mouth", "Brow_R", "Brow_L", "Hair"]

# depth of each layer in front of (+) / behind (-) the head surface: the
# parallax of the pseudo-3D turn
DEPTH_OFFSET = dict(Ear=-1.5, Earring=-1.5, Crown=0.6, F1=1.0, F2=1.0, F3=1.0, F4=0.9, F5=0.6, Stray=1.1,
                    Ahoge=0.3, Pins=0.8, LockR_A=0.4, LockR_B=0.45, LooseL=0.3, TuckL=-1.5, BackHair=-4.5,
                    Brow_R=0.2, Brow_L=0.2, Sweat=0.4)

# depth offset as a function of a layer's z (for the cage's fill field)
Z_OFFSET = [(-3.1, -4.6), (-2.9, -4.5), (-0.35, -1.5), (-0.1, -1.5), (0.0, 0.0), (0.2, 0.1), (0.4, 0.6),
            (0.55, 0.45), (0.65, 1.0), (0.9, 1.0), (1.0, 0.8)]

TURN_KEYS = [("Turn_R", "turn", 1, (15.0, 30.0)), ("Turn_L", "turn", -1, (15.0, 30.0)),
             ("Nod_Down", "nod", 1, (10.0, 20.0)), ("Nod_Up", "nod", -1, (10.0, 20.0))]

# the parameter channels: (channel, [(activation, state)], weight source)
# weight sources: ("pos", ctl, avar) | ("neg", ctl, avar) |
#                 ("prod", (sign, ctl, avar, clamp_hi), (sign, ctl, avar, scale))


def channel_specs():
    C = []
    for tag in ("R", "L"):
        e = "eye%s." % tag
        ctl = "Eye_" + tag
        C.append(("Blink_" + tag, [(0.5, {e + "close": 0.5}), (1.0, {e + "close": 1.0})], ("neg", ctl, "ty")))
        C.append(("Wide_" + tag, [(1.0, {e + "wide": 1.0})], ("pos", ctl, "ty")))
        C.append(("EyeSmile_" + tag, [(1.0, {e + "smile": 1.0})], ("pos", ctl, "tx")))
        C.append(("LidFlat_" + tag, [(1.0, {e + "flat": 1.0})], ("pos", ctl, "tz")))
        C.append(("BlinkFlat_" + tag, "corr", [{e + "close": 1.0}, {e + "flat": 1.0}],
                  ("prod", (-1, ctl, "ty", 1.0), (1, ctl, "tz", 1.0))))
        C.append(("BlinkSmile_" + tag, "corr", [{e + "close": 1.0}, {e + "smile": 1.0}],
                  ("prod", (-1, ctl, "ty", 1.0), (1, ctl, "tx", 1.0))))
        b = "brow%s." % tag
        ctl = "Brow_" + tag
        C.append(("BrowUp_" + tag, [(1.0, {b + "up": 1.0})], ("pos", ctl, "ty")))
        C.append(("BrowDown_" + tag, [(1.0, {b + "down": 1.0})], ("neg", ctl, "ty")))
        C.append(("BrowWorried_" + tag, [(20.0, {b + "worried": 1.0})], ("pos", ctl, "rz")))
        C.append(("BrowAngry_" + tag, [(20.0, {b + "angry": 1.0})], ("neg", ctl, "rz")))
    C.append(("Look_R", [(1.0, {"look.r": 1.0})], ("pos", "Look", "tx")))
    C.append(("Look_L", [(1.0, {"look.l": 1.0})], ("neg", "Look", "tx")))
    C.append(("Look_U", [(1.0, {"look.u": 1.0})], ("pos", "Look", "ty")))
    C.append(("Look_D", [(1.0, {"look.d": 1.0})], ("neg", "Look", "ty")))
    C.append(("Shrink", [(1.0, {"iris.shrink": 1.0})], ("pos", "Look", "tz")))
    C.append(("Open", [(0.15, {"mouth.open": 0.15}), (0.5, {"mouth.open": 0.5}),
                       (1.0, {"mouth.open": 1.0}), (1.5, {"mouth.open": 1.5})],
              ("neg", "Mouth", "ty")))
    C.append(("TongueOut", [(1.0, {"mouth.tongue": 1.0})], ("pos", "Tongue", "ty")))
    C.append(("LipFullness", [(1.0, {"mouth.lipThick": 1.0})], ("pos", "Lip", "ty")))
    C.append(("Teeth", "corr", [{"mouth.open": 1.0}, {"mouth.teeth": 1.0}],
              ("prod", (-1, "Mouth", "ty", 1.5), (1, "Teeth", "ty", 1.0))))
    C.append(("Wide", [(1.0, {"mouth.wide": 1.0})], ("pos", "Mouth", "tx")))
    C.append(("Round", [(1.0, {"mouth.round": 1.0})], ("neg", "Mouth", "tx")))
    C.append(("Smile", [(20.0, {"mouth.smile": 1.0})], ("pos", "Mouth", "rz")))
    C.append(("Frown", [(20.0, {"mouth.frown": 1.0})], ("neg", "Mouth", "rz")))
    C.append(("SmirkR", [(20.0, {"mouth.smirkR": 1.0})], ("pos", "Mouth", "ry")))
    C.append(("SmirkL", [(20.0, {"mouth.smirkL": 1.0})], ("neg", "Mouth", "ry")))
    C.append(("OpenWide", "corr", [{"mouth.open": 1.0}, {"mouth.wide": 1.0}],
              ("prod", (-1, "Mouth", "ty", 1.4), (1, "Mouth", "tx", 1.0))))
    C.append(("OpenRound", "corr", [{"mouth.open": 1.0}, {"mouth.round": 1.0}],
              ("prod", (-1, "Mouth", "ty", 1.4), (-1, "Mouth", "tx", 1.0))))
    C.append(("OpenSmile", "corr", [{"mouth.open": 1.0}, {"mouth.smile": 1.0}],
              ("prod", (-1, "Mouth", "ty", 1.4), (1, "Mouth", "rz", 1.0 / 20.0))))
    C.append(("OpenFrown", "corr", [{"mouth.open": 1.0}, {"mouth.frown": 1.0}],
              ("prod", (-1, "Mouth", "ty", 1.4), (-1, "Mouth", "rz", 1.0 / 20.0))))
    C.append(("BlushPop", [(1.0, {"blush": 1.0})], ("pos", "Fx", "tx")))
    C.append(("Sweat", [(0.35, {"sweat": 0.35}), (0.9, {"sweat": 0.9}), (1.0, {"sweat": 1.0})],
              ("pos", "Fx", "ty")))
    C.append(("Hatch", [(1.0, {"hatch": 1.0})], ("pos", "Fx", "tz")))
    return C


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
    r.SetTargets([Sdf.Path(t) for t in targets])
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


# cage fitting (the lattice mover evaluates a Bernstein basis)

def bernstein_basis(P3, lo, hi, div):
    from math import comb
    uvw = np.clip((P3 - lo) / (hi - lo), 0, 1)
    mats = []
    for a in range(3):
        n = div[a] - 1
        t = uvw[:, a:a + 1]
        k = np.arange(div[a])[None, :]
        c = np.array([comb(n, i) for i in range(div[a])], dtype=float)[None, :]
        mats.append(c * t ** k * (1 - t) ** (n - k))
    Bx, By, Bz = mats
    return (Bz[:, :, None, None] * By[:, None, :, None] * Bx[:, None, None, :]).reshape(len(P3), -1)


def cage_points(lo, hi, div):
    xs = np.linspace(lo[0], hi[0], div[0])
    ys = np.linspace(lo[1], hi[1], div[1])
    zs = np.linspace(lo[2], hi[2], div[2])
    return np.array([(x, y, z) for z in zs for y in ys for x in xs])


def fit_cage(B, disp, weights=None, lam=2e-3, fill=None):
    """Least-squares cage deltas (x, y) reproducing `disp` through the basis
    B, with a small ridge; `fill` = (B_grid, disp_grid, weight) samples the
    target field over the whole cage volume so cage points far from any art
    stay calm and the warp stays smooth where there is no art."""
    w = np.ones(len(B)) if weights is None else weights
    rows = [B * w[:, None]]
    rhs = [disp[:, :2] * w[:, None]]
    if fill is not None:
        Bg, dg, wg = fill
        rows.append(Bg * wg)
        rhs.append(dg[:, :2] * wg)
    A = np.vstack(rows + [math.sqrt(lam) * np.eye(B.shape[1])])
    rhs = np.vstack(rhs + [np.zeros((B.shape[1], 2))])
    C, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    out = np.zeros((B.shape[1], 3))
    out[:, :2] = C
    return out


# chains and skin weights

def chain_bone_weights(t, ts):
    """Smooth per-bone weights along a chain for along-parameter t."""
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


def spine_param(pts, spine):
    """Arc-length parameter t in [0, 1] of the spine point nearest each point."""
    from scipy.spatial import cKDTree
    sp = P.resample(spine, n=400)
    t = np.linspace(0, 1, len(sp))
    _, k = cKDTree(sp).query(pts)
    return t[k]


# the builder

class Builder(object):
    def __init__(self, path, log=print):
        self.log = log
        if os.path.exists(path):
            os.remove(path)
        self.path = path
        self.stage = Usd.Stage.CreateNew(path)
        s = self.stage
        s.SetMetadata("upAxis", "Y")
        s.SetMetadata("metersPerUnit", 0.01)
        s.SetStartTimeCode(1)
        s.SetEndTimeCode(240)
        s.SetTimeCodesPerSecond(FPS)
        s.SetFramesPerSecond(FPS)
        root = UsdGeom.Xform.Define(s, ROOT)
        s.SetDefaultPrim(root.GetPrim())
        s.GetRootLayer().documentation = (
            "RigExec example: Shion, a 2D mesh-deform (layered mesh) anime bust with painted "
            "textures, rigged entirely with stock RigExec prims. Generated by build_bust.py -- "
            "edit that, not this file.")

    def paint(self):
        t0 = __import__("time").time()
        hair_parts, hair_tex, pcs = AH.build()
        self.pieces = {p.name: p for p in pcs}
        face_parts, face_tex = AF.build(fringe_shadow=AH.fringe_shadow_fn(pcs))
        body_parts, body_tex = AB.build(cast=AH.fringe_shadow_fn(pcs, names=("LockR_A", "LockR_B"),
                                                                  offset=(0.45, -0.55)))
        limb_parts, limb_tex = AL.build()
        self.parts = {p.name: p for p in hair_parts + face_parts + body_parts + limb_parts}
        self.tex = {}
        for d in (hair_tex, face_tex, body_tex, limb_tex):
            self.tex.update(d)
        self.log("  painted %d layers, %d textures in %.1fs" % (len(self.parts), len(self.tex),
                                                                __import__("time").time() - t0))

    def write_textures(self):
        d = os.path.join(HERE, "textures")
        os.makedirs(d, exist_ok=True)
        for name, t in sorted(self.tex.items()):
            Image.fromarray(t.rgba, "RGBA").save(os.path.join(d, name + ".png"), compress_level=6)

    def materials(self):
        s = self.stage
        scope(s, LOOKS)
        blend = set(p.tex for p in self.parts.values() if p.material == "blend")
        self.mat = {}
        for name in sorted(self.tex):
            path = LOOKS + "/" + name
            mat = UsdShade.Material.Define(s, path)
            sh = UsdShade.Shader.Define(s, path + "/Surface")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("roughness", T.Float).Set(1.0)
            sh.CreateInput("metallic", T.Float).Set(0.0)
            sh.CreateInput("specularColor", T.Color3f).Set(Gf.Vec3f(0, 0, 0))
            sh.CreateInput("useSpecularWorkflow", T.Int).Set(1)
            st = UsdShade.Shader.Define(s, path + "/UV")
            st.CreateIdAttr("UsdPrimvarReader_float2")
            st.CreateInput("varname", T.Token).Set("st")
            tx = UsdShade.Shader.Define(s, path + "/Texture")
            tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", T.Asset).Set("./textures/%s.png" % name)
            tx.CreateInput("sourceColorSpace", T.Token).Set("sRGB")
            tx.CreateInput("wrapS", T.Token).Set("clamp")
            tx.CreateInput("wrapT", T.Token).Set("clamp")
            tx.CreateInput("st", T.Float2).ConnectToSource(st.ConnectableAPI(), "result")
            rgb = tx.CreateOutput("rgb", T.Float3)
            a = tx.CreateOutput("a", T.Float)
            sh.CreateInput("diffuseColor", T.Color3f).ConnectToSource(rgb)
            sh.CreateInput("opacity", T.Float).ConnectToSource(a)
            if name not in blend:
                sh.CreateInput("opacityThreshold", T.Float).Set(0.5)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self.mat[name] = mat

    def meshes(self):
        s = self.stage
        scope(s, GEOM)
        self.mesh = {}
        for mname, pnames in MESHES:
            pts, tris, uvs, subsets, zs, offs, owner = [], [], [], [], [], [], []
            off = 0
            ranges = {}
            for pn in pnames:
                p = self.parts[pn]
                t = self.tex[p.tex]
                n = len(p.rest)
                # orient every triangle counter-clockwise in its paint pose
                q = p.uv_pts
                a, b, c = q[p.tris[:, 0]], q[p.tris[:, 1]], q[p.tris[:, 2]]
                cr = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
                tri = p.tris.copy()
                tri[cr < 0] = tri[cr < 0][:, [0, 2, 1]]
                first_face = sum(len(x) for x in tris)
                tris.append(tri + off)
                subsets.append((pn, p.tex, np.arange(first_face, first_face + len(tri))))
                pts.append(p.rest)
                uvs.append(p.uv(t.bbox))
                zs.append(np.full(n, p.z))
                offs.append(np.full(n, DEPTH_OFFSET.get(pn, 0.0)))
                ranges[pn] = (off, off + n)
                owner += [pn] * n
                off += n
            P2 = np.vstack(pts)
            Z = np.concatenate(zs)
            P3 = np.column_stack([P2, Z])
            F = np.vstack(tris)
            UV = np.vstack(uvs)
            mesh = UsdGeom.Mesh.Define(s, GEOM + "/" + mname)
            mesh.CreatePointsAttr(v3f(P3))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(F)))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1).astype(np.int32)))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            lo, hi = P3.min(axis=0) - 4.0, P3.max(axis=0) + 4.0
            mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
            pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", T.TexCoord2fArray, UsdGeom.Tokens.vertex)
            pv.Set(Vt.Vec2fArray.FromNumpy(np.round(UV, 6).astype(np.float32)))
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
            for pn, tex, faces in subsets:
                sub = UsdGeom.Subset.CreateGeomSubset(mesh, pn, UsdGeom.Tokens.face,
                                                      Vt.IntArray.FromNumpy(faces.astype(np.int32)),
                                                      UsdShade.Tokens.materialBind, UsdGeom.Tokens.nonOverlapping)
                UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(self.mat[tex])
            UsdGeom.Subset.SetFamilyType(mesh, UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
            self.mesh[mname] = dict(rest=r4(P3), ranges=ranges, parts=pnames, offsets=np.concatenate(offs),
                                    owner=np.array(owner))
        n = sum(len(m["rest"]) for m in self.mesh.values())
        self.log("  %d meshes, %d points" % (len(self.mesh), n))

    def mesh_state(self, mname, st):
        """Points (N, 3) of a mesh for an expression state."""
        m = self.mesh[mname]
        out = m["rest"].copy()
        for pn in m["parts"]:
            p = self.parts[pn]
            if p.gen is None:
                continue
            a, b = m["ranges"][pn]
            out[a:b, :2] = p.gen(st)
        return out

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

    def lattice_wire(self, name, lo, hi, nx, ny, z, color, opacity=0.8, width=0.08):
        s = self.stage
        xs = np.linspace(lo[0], hi[0], nx)
        ys = np.linspace(lo[1], hi[1], ny)
        pts, counts = [], []
        for x in xs:
            col = [(x, y, z) for y in np.linspace(ys[0], ys[-1], 60)]
            pts += col
            counts.append(len(col))
        for y in ys:
            row = [(x, y, z) for x in np.linspace(xs[0], xs[-1], 50)]
            pts += row
            counts.append(len(row))
        c = UsdGeom.BasisCurves.Define(s, RIG + "/Guides/" + name)
        c.CreateTypeAttr("linear")
        c.CreatePointsAttr(v3f(pts))
        c.CreateCurveVertexCountsAttr(Vt.IntArray(counts))
        c.CreateWidthsAttr(Vt.FloatArray([width]))
        c.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        c.CreatePurposeAttr("guide")
        c.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        c.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant).Set(Vt.FloatArray([opacity]))
        self.wire_rest = getattr(self, "wire_rest", {})
        self.wire_rest[name] = np.array(pts)

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

    def joint(self, path, pos_local, radius=0.45, color=(0.95, 0.35, 0.45), opacity=0.9):
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

    def chain(self, name, pts, cparent, cparent_pos, jparent, jparent_pos, big=False, color=(0.30, 0.95, 0.88),
              jcolor=(0.62, 0.40, 1.0)):
        cpaths, jpaths = [], []
        prev_c = np.asarray(cparent_pos, float)
        prev_j = np.asarray(jparent_pos, float)
        for k, p in enumerate(pts):
            p = np.asarray(p, float)
            cpath = cparent + "/%s_%d_ctl" % (name, k)
            lc = p - prev_c
            self.control(cpath, (lc[0], lc[1], 0.0), "sphere",
                         (0.5, 0.5, 0.5) if big else (0.32, 0.32, 0.32), (0, 0, 0), color, 0.05)
            jpath = jparent + "/%s_%d" % (name, k)
            lj = p - prev_j
            self.joint(jpath, (lj[0], lj[1], 0.0), 0.3 if big else 0.2, jcolor, 0.95)
            cpaths.append(cpath)
            jpaths.append(jpath)
            cparent, jparent = cpath, jpath
            prev_c = p
            prev_j = p
        self.fk(name, cpaths, jpaths)
        self.chains[name] = dict(pts=[np.asarray(p, float) for p in pts], controls=cpaths, joints=jpaths)
        return cpaths, jpaths

    def skeleton(self):
        s = self.stage
        rig = define(s, RIG, "RigExecRoot")
        attr(rig, "rigExec:partition", T.Token, "Shion", uniform=True)
        for sc in ("Controls", "Joints", "Solvers", "BlendInputs", "Movers", "Guides"):
            scope(s, RIG + "/" + sc)
        ORANGE = (1.0, 0.62, 0.22)
        YELLOW = (1.0, 0.86, 0.22)
        bx, by = D.BODY_PIVOT
        nx, ny = D.NECK_PIVOT
        hx, hy = D.HEAD_PIVOT
        body_ctl = CTL + "/Body_ctl"
        neck_ctl = body_ctl + "/Neck_ctl"
        head_ctl = neck_ctl + "/Head_ctl"
        self.control(body_ctl, (bx, by, 0), "cube", (19.0, 0.9, 0.2), (0, 20.5, 0.0), ORANGE, 0.12)
        self.control(neck_ctl, (nx - bx, ny - by, 0), "cube", (4.4, 0.7, 0.2), (0, 0.0, 0), ORANGE, 0.10)
        self.control(head_ctl, (hx - nx, hy - ny, 0), "cube", (11.0, 11.6, 0.2), (0, 8.8, 0), ORANGE, 0.10)
        self.control(body_ctl + "/Torso_ctl", (0 - bx, -23.5 - by, 0), "diamond", (0.8, 0.8, 0.8), (0, 0, 0), YELLOW)
        H = np.array([hx, hy])
        face = {
            "FaceAngle_ctl": ((0.0, -4.6), "sphere", (0.9, 0.9, 0.9), (0, 0, 4.5)),
            "Look_ctl": ((0.0, -2.2), "diamond", (0.55, 0.55, 0.55), (0, 0, 0)),
            "Eye_R_ctl": ((4.05, -0.8), "diamond", (0.42, 0.42, 0.42), (0, 0, 0)),
            "Eye_L_ctl": ((-4.05, -0.8), "diamond", (0.42, 0.42, 0.42), (0, 0, 0)),
            "Brow_R_ctl": ((4.1, 2.4), "diamond", (0.42, 0.42, 0.42), (0, 0, 0)),
            "Brow_L_ctl": ((-4.1, 2.4), "diamond", (0.42, 0.42, 0.42), (0, 0, 0)),
            "Mouth_ctl": ((0.08, -6.9), "sphere", (0.5, 0.5, 0.5), (0, 0, 2.2)),
            "Teeth_ctl": ((-2.8, -7.4), "diamond", (0.3, 0.3, 0.3), (0, 0, 0)),
            "Tongue_ctl": ((0.08, -8.8), "diamond", (0.3, 0.3, 0.3), (0, 0, 0)),
            "Lip_ctl": ((2.8, -7.4), "diamond", (0.3, 0.3, 0.3), (0, 0, 0)),
            "Fx_ctl": ((8.4, 4.6), "diamond", (0.5, 0.5, 0.5), (0, 0, 0)),
        }
        for name, (pos, shape, sc, off) in face.items():
            local = np.array(pos) - H
            self.control(head_ctl + "/" + name, (local[0], local[1], 2.5), shape, sc, off, YELLOW)
        self.joint(JNT + "/Body", (bx, by, 0), 0.45, (1.0, 0.55, 0.25), 0.7)
        self.joint(JNT + "/Body/Neck", (nx - bx, ny - by, 0), 0.4, (1.0, 0.55, 0.25), 0.7)
        self.joint(JNT + "/Body/Neck/Head", (hx - nx, hy - ny, 0), 0.45, (1.0, 0.55, 0.25), 0.7)
        self.fk("Spine", [body_ctl, neck_ctl, head_ctl], [JNT + "/Body", JNT + "/Body/Neck", JNT + "/Body/Neck/Head"])
        self.head_ctl, self.neck_ctl, self.body_ctl = head_ctl, neck_ctl, body_ctl
        self.head_j, self.neck_j, self.body_j = JNT + "/Body/Neck/Head", JNT + "/Body/Neck", JNT + "/Body"
        self.ctl = {"FaceAngle": head_ctl + "/FaceAngle_ctl", "Look": head_ctl + "/Look_ctl",
                    "Eye_R": head_ctl + "/Eye_R_ctl", "Eye_L": head_ctl + "/Eye_L_ctl",
                    "Brow_R": head_ctl + "/Brow_R_ctl", "Brow_L": head_ctl + "/Brow_L_ctl",
                    "Mouth": head_ctl + "/Mouth_ctl", "Fx": head_ctl + "/Fx_ctl",
                    "Torso": body_ctl + "/Torso_ctl", "Head": head_ctl, "Neck": neck_ctl, "Body": body_ctl}
        for name in ("Teeth", "Tongue", "Lip"):
            self.ctl[name] = head_ctl + "/" + name + "_ctl"
        # hair chains, nested under the head handle
        scope(s, JNT + "/Hair")
        self.chains = {}
        for name, c in self.pieces.items():
            if c.kind != "clump" or c.chain <= 0:
                continue
            pts, ts = AH.chain_points(c)
            self.chain(name, pts, head_ctl, H, JNT + "/Hair", (0, 0), big=name.startswith("Lock"))
            self.chains[name]["ts"] = ts
        for name, pts in (("BackL", [(-8.9, 4.0), (-9.5, -3.0), (-10., -9.0), (-8.5, -15.0)]),
                          ("BackR", [(9.2, 4.0), (9.8, -3.0), (10.1, -9.0), (8.7, -15.0)])):
            self.chain(name, pts, head_ctl, H, JNT + "/Hair", (0, 0), big=True)
        self.chain("Earring", [D.EARRING_TOP, (D.EARRING_TOP[0], D.EARRING_TOP[1] - 1.5)], head_ctl, H,
                   JNT + "/Hair", (0, 0), color=(1.0, 0.8, 0.3), jcolor=(1.0, 0.75, 0.3))
        # ribbon tails under the torso
        scope(s, JNT + "/Ribbon")
        B = np.array([bx, by])
        for side, tag in ((-1, "L"), (1, "R")):
            ctrl = AB.tail_ctrl(side)
            sp = P.resample(P.catmull(ctrl, n=12), n=40)
            pts = [sp[0], sp[16], sp[30]]
            self.chain("Tail" + tag, pts, body_ctl, B, JNT + "/Ribbon", (0, 0), color=(0.5, 0.75, 1.0),
                       jcolor=(0.4, 0.6, 1.0))

    def articulation(self):
        """Parent-relative arm and finger chains; independent of hair dynamics."""
        self.limbs = {}

        def chain(name, labels, points, cp, jp, origin):
            controls, joints = [], []
            prev = np.array(origin)
            for label, point in zip(labels, points):
                local = np.asarray(point) - prev
                cp += "/" + label + "_ctl"
                jp += "/" + label
                self.control(cp, (*local, 0), "sphere", (.38, .38, .38), color=(.36, .80, .94))
                self.joint(jp, (*local, 0), .18, (.36, .80, .94))
                controls.append(cp)
                joints.append(jp)
                self.ctl[label] = cp
                prev = point
            self.fk(name, controls, joints)
            result = dict(controls=controls, joints=joints, pts=np.asarray(points))
            self.limbs[name] = result
            return result

        for side, tag in ((-1, "L"), (1, "R")):
            arm = chain("Arm_" + tag, [n + "_" + tag for n in ("Shoulder", "Elbow", "Hand")],
                        AL.arm_points(side), self.body_ctl, self.body_j, D.BODY_PIVOT)
            for finger in AL.FINGERS:
                chain(finger + "_" + tag, [finger + "_" + tag + "_%d" % i for i in range(3)],
                      AL.finger_points(side, finger)[:3], arm["controls"][-1], arm["joints"][-1], arm["pts"][-1])

    def hand_keyforms(self):
        for side, tag in ((-1, "L"), (1, "R")):
            name = "Hand_" + tag
            rest = self.mesh[name]["rest"]
            for sign, suffix in ((1, "R"), (-1, "L")):
                ks = []
                for angle in (15, 35):
                    k = rest.copy()
                    k[:, :2] = AL.turn_points(rest[:, :2], side, sign * angle)
                    ks.append((angle, k))
                channel = "PalmTurn_" + suffix
                self.blend_channel(name, channel, ks, rest)
                self.channel_src[(name, channel)] = ("pos" if sign > 0 else "neg", name, "turn")
            for finger in AL.FINGERS:
                a, b = self.mesh[name]["ranges"][finger + "_" + tag]
                ks = []
                for amount in (.25, .5, .65, .8, .9, 1.0):
                    k = rest.copy()
                    k[a:b, :2] = AL.curl_points(rest[a:b, :2], side, finger, amount)
                    if finger in AL.FOLDED_PATHS:
                        fold = finger + "Fold_" + tag
                        fa, fb = self.mesh[name]["ranges"][fold]
                        k[fa:fb, :2] = AL.folded_points(self.parts[fold].uv_pts, side, finger, amount)
                    ks.append((amount, k))
                ch = "Curl_" + finger
                self.blend_channel(name, ch, ks, rest)
                self.channel_src[(name, ch)] = ("sum", (name, "curl"), (finger + "_" + tag + "_0", "curl"))

    def limb_weights(self, mname):
        tag = mname[-1]
        arm = self.limbs["Arm_" + tag]
        m = self.mesh[mname]
        n = len(m["rest"])
        idx = np.zeros((n, 3), dtype=np.int64)
        wts = np.zeros((n, 3))
        if mname.startswith("Arm"):
            x, y = m["rest"][:, 0], m["rest"][:, 1]
            glue = (1-P.smooth(-18.0, -20.5, y)) * (1-P.smooth(11., 14., np.abs(x))) * .65
            idx[:] = [0, 1, 2]
            wts[:, 1] = 1
            a, b = m["ranges"]["UpperSleeve_"+tag]
            wts[a:b, 0] = glue[a:b]
            wts[a:b, 1] = 1-glue[a:b]
            a, b = m["ranges"]["ForeSleeve_"+tag]
            wts[a:b] = [0, 0, 1]
            return [self.body_j] + arm["joints"][:2], idx, wts
        infl = [arm["joints"][-1]]
        wts[:, 0] = 1
        for finger in AL.FINGERS:
            a, b = m["ranges"][finger + "_" + tag]
            f = self.limbs[finger + "_" + tag]
            start = len(infl)
            infl += f["joints"]
            side = -1 if tag == "L" else 1
            t = spine_param(m["rest"][a:b, :2], AL.finger_points(side, finger))
            W = chain_bone_weights(t, [0, .40, .76])
            idx[a:b] = np.arange(start, start+3)
            wts[a:b] = W
            if finger in AL.FOLDED_PATHS:
                fa, fb = m["ranges"][finger + "Fold_" + tag]
                idx[fa:fb, 0] = start
                wts[fa:fb] = [1, 0, 0]
        return infl, idx, wts

    def blend_channel(self, mesh, channel, samples, rest):
        s = self.stage
        bpath = "%s/%s/%s" % (BI, mesh, channel)
        b = define(s, bpath, "RigExecBlendInput")
        attr(b, "inputs:weight", T.Float, 0.0)
        spaths = []
        for act, xyz in samples:
            d = r4(np.asarray(xyz) - rest)
            nz = np.nonzero(np.abs(d).max(axis=1) > 2e-4)[0]
            if len(nz) == 0:
                nz = np.array([0])
            tag = ("%g" % act).replace(".", "p")
            shape_path = "%s/%s/%s_%s" % (TARGETS, mesh, channel, tag)
            bs = UsdSkel.BlendShape.Define(s, shape_path)
            bs.CreateOffsetsAttr(v3f(d[nz]))
            bs.CreatePointIndicesAttr(Vt.IntArray.FromNumpy(nz.astype(np.int32)))
            sp = define(s, "%s/K_%s" % (bpath, tag), "RigExecBlendSample")
            attr(sp, "rigExec:activation", T.Float, float(act))
            rel(sp, "rigExec:blendShape", [shape_path])
            spaths.append(sp.GetPath())
        rel(b, "rigExec:samples", [str(p) for p in spaths])
        self.channels.setdefault(mesh, []).append(channel)
        return b

    def expression_channels(self):
        s = self.stage
        scope(s, TARGETS)
        self.channels = {}
        self.channel_src = {}
        for mname in self.mesh:
            scope(s, TARGETS + "/" + mname)
            scope(s, BI + "/" + mname)
        rest = {m: self.mesh_state(m, ST.REST) for m in self.mesh}
        for m in self.mesh:
            assert np.allclose(rest[m], self.mesh[m]["rest"], atol=2e-3), m
        for spec in channel_specs():
            name = spec[0]
            if spec[1] == "corr":
                sa, sb = spec[2]
                both = dict(sa)
                both.update(sb)
                src = spec[3]
                for m in self.mesh:
                    K0 = rest[m]
                    Ka = self.mesh_state(m, ST.S(sa))
                    Kb = self.mesh_state(m, ST.S(sb))
                    Kab = self.mesh_state(m, ST.S(both))
                    delta = Kab - Ka - Kb + K0
                    if np.abs(delta).max() < 2e-3:
                        continue
                    self.blend_channel(m, name, [(1.0, K0 + delta)], K0)
                    self.channel_src[(m, name)] = src
            else:
                samples, src = spec[1], spec[2]
                for m in self.mesh:
                    K0 = rest[m]
                    ks = [(a, self.mesh_state(m, ST.S(st))) for a, st in samples]
                    if max(np.abs(k - K0).max() for _, k in ks) < 2e-3:
                        continue
                    self.blend_channel(m, name, ks, K0)
                    self.channel_src[(m, name)] = src
        n = sum(len(v) for v in self.channels.values())
        self.log("  %d expression channels on %d meshes" % (n, len(self.channels)))

    def turn_targets(self, mname, kind, ang):
        m = self.mesh[mname]
        P3 = m["rest"]
        fn = TU.turn_displacement if kind == "turn" else TU.nod_displacement
        out = P3.copy()
        for part, (a, b) in m["ranges"].items():
            out[a:b, :2] = TU.project_part(P3[a:b, :2], part, kind, ang)
        if mname == "Face":
            # the contour line is rebuilt along the turned outline, so its
            # width holds instead of squeezing with the far cheek
            o = D.face_outline()
            od = fn(o, np.zeros(len(o)), ang)
            sp = (o + od)[AF.face_line_indices()]
            a, b = m["ranges"]["FaceLine"]
            out[a:b, :2] = ribbon_points(sp, [-0.02, 0.24], P.normals(sp))
        return out

    def head_turn(self):
        s = self.stage
        scope(s, TARGETS + "/HeadCage")
        scope(s, BI + "/HeadCage")
        P_all = np.vstack([self.mesh[m]["rest"] for m in HEAD_MESHES])
        B_all = bernstein_basis(P_all, HEAD_CAGE_LO, HEAD_CAGE_HI, HEAD_CAGE_DIV)
        # Weight facial features most strongly; peripheral hair can deform more.
        w = np.concatenate([np.where(self.mesh[m]["rest"][:, 1] < -13.0, 0.15, 1.0) *
                            (2.0 if m in ("Face", "Eye_R", "Eye_L", "Mouth", "Brow_R", "Brow_L") else 1.0)
                            for m in HEAD_MESHES])
        gx, gy, gz = np.meshgrid(np.linspace(HEAD_CAGE_LO[0], HEAD_CAGE_HI[0], 17),
                                 np.linspace(HEAD_CAGE_LO[1], HEAD_CAGE_HI[1], 17),
                                 np.linspace(HEAD_CAGE_LO[2], HEAD_CAGE_HI[2], 6), indexing="ij")
        G3 = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        zo = np.array(Z_OFFSET)
        Goff = np.interp(G3[:, 2], zo[:, 0], zo[:, 1])
        Bg = bernstein_basis(G3, HEAD_CAGE_LO, HEAD_CAGE_HI, HEAD_CAGE_DIV)
        self.fit_report = {}
        cage_keys = {}
        for ch, kind, sgn, acts in TURN_KEYS:
            ks_cage = []
            per_mesh = {m: [] for m in HEAD_MESHES}
            for a in acts:
                ang = sgn * a
                targets = {m: self.turn_targets(m, kind, ang) for m in HEAD_MESHES}
                disp = np.vstack([targets[m] - self.mesh[m]["rest"] for m in HEAD_MESHES])
                fn = TU.turn_displacement if kind == "turn" else TU.nod_displacement
                gd = np.zeros((len(G3), 3))
                gd[:, :2] = fn(G3[:, :2], Goff, ang)
                C = fit_cage(B_all, disp, w, fill=(Bg, gd, 0.35))
                fitted = B_all @ C
                err = np.linalg.norm(fitted[:, :2] - disp[:, :2], axis=1)
                self.fit_report["%s@%g" % (ch, a)] = (float(err.mean()), float(err.max()))
                ks_cage.append((a, self.head_cage + C))
                off = 0
                for m in HEAD_MESHES:
                    n = len(self.mesh[m]["rest"])
                    resid = targets[m] - self.mesh[m]["rest"] - fitted[off:off + n]
                    resid[:, 2] = 0
                    per_mesh[m].append((a, self.mesh[m]["rest"] + resid))
                    off += n
            cage_keys[ch] = ks_cage
            for m, ks in per_mesh.items():
                if max(np.abs(k - self.mesh[m]["rest"]).max() for _, k in ks) < 5e-3:
                    continue
                self.blend_channel(m, ch, ks, self.mesh[m]["rest"])
                self.channel_src[(m, ch)] = TURN_SRC[ch]
        for ch, ks in cage_keys.items():
            self.blend_channel("HeadCage", ch, ks, r4(self.head_cage))
            self.channel_src[("HeadCage", ch)] = TURN_SRC[ch]

    def body_cage_channels(self):
        s = self.stage
        scope(s, TARGETS + "/BodyCage")
        scope(s, BI + "/BodyCage")
        pts = np.vstack([self.mesh[m]["rest"] for m in ("Body", "Neck", "Ribbon")])
        B = bernstein_basis(pts, BODY_CAGE_LO, BODY_CAGE_HI, BODY_CAGE_DIV)
        for ch, kw, act, src in (("Breath", dict(breath=1.0), 1.0, ("pos", "Torso", "ty")),
                                 ("BodyTurn_R", dict(turn=10.0), 10.0, ("pos", "Torso", "ry")),
                                 ("BodyTurn_L", dict(turn=-10.0), 10.0, ("neg", "Torso", "ry"))):
            disp = body_field(pts, **kw)
            C = fit_cage(B, disp, lam=1e-3)
            self.blend_channel("BodyCage", ch, [(act, self.body_cage + C)], r4(self.body_cage))
            self.channel_src[("BodyCage", ch)] = src

    def wiring(self):
        s = self.stage
        scope(s, MOV + "/Channels")
        avar = lambda c, a: "%s.avars:%s" % (self.ctl[c], a)  # noqa: E731
        for (mesh, ch), src in sorted(self.channel_src.items()):
            target = "%s/%s/%s.inputs:weight" % (BI, mesh, ch)
            w = s.GetAttributeAtPath(target)
            if src[0] == "pos":
                w.SetConnections([Sdf.Path(avar(src[1], src[2]))])
                continue
            base = "%s/Channels/%s_%s" % (MOV, mesh, ch)
            scope(s, base)
            order = []

            def math_mover(name, op, value=None, conn=None, lo=None, hi=None):
                m = define(s, base + "/" + name, "RigExecFloatMathMover", ["RigExecMoverAPI"])
                attr(m, "rigExec:operation", T.Token, op, uniform=True)
                v = attr(m, "inputs:value", T.Float, float(value or 0.0))
                if conn:
                    v.SetConnections([Sdf.Path(conn)])
                if lo is not None:
                    attr(m, "inputs:min", T.Float, float(lo))
                    attr(m, "inputs:max", T.Float, float(hi))
                rel(m, "rigExec:moves", [target])
                order.append(name)

            if src[0] == "neg":
                math_mover("Read", "add", conn=avar(src[1], src[2]))
                math_mover("Negate", "multiply", -1.0)
            elif src[0] == "prod":
                (sa, ca, aa, hi), (sb, cb, ab, scale) = src[1], src[2]
                math_mover("Read", "add", conn=avar(ca, aa))
                if sa < 0:
                    math_mover("Negate", "multiply", -1.0)
                math_mover("Clamp", "clamp", lo=0.0, hi=hi)
                math_mover("Times", "multiply", conn=avar(cb, ab))
                if sb * scale != 1.0:
                    math_mover("Scale", "multiply", sb * scale)
            elif src[0] == "sum":
                math_mover("Read", "add", conn=avar(*src[1]))
                math_mover("Add", "add", conn=avar(*src[2]))
                math_mover("Clamp", "clamp", lo=0, hi=1)
            # the LAST child runs first
            s.GetPrimAtPath(base).SetChildrenReorder(order[::-1])

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
            attr(m, "rigExec:cageReadPhase", T.Token, "final", uniform=True)
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

        def shapes(path, target, mesh, channels=None):
            m = mover(path, "RigExecBlendShapeMover")
            rel(m, "rigExec:blendInputs", ["%s/%s/%s" % (BI, mesh, ch) for ch in (self.channels[mesh] if channels is None else channels)])
            rel(m, "rigExec:moves", [target + ".points"])
            return m

        for cage in ("HeadCage", "BodyCage"):
            m = mover("%s/Cages/%sKeyforms" % (MOV, cage), "RigExecBlendShapeMover")
            rel(m, "rigExec:blendInputs", ["%s/%s/%s" % (BI, cage, ch) for ch in self.channels[cage]])
            rel(m, "rigExec:moves", [CAGES + "/" + cage + ".points"])

        order = []
        for mname, _ in MESHES:
            base = G + "/" + mname
            scope(s, base)
            tgt = GEOM + "/" + mname
            run = []            # in execution order
            turn_channels = [ch for ch in self.channels.get(mname, []) if ch in TURN_SRC] if mname in HEAD_MESHES else []
            expression_channels = [ch for ch in self.channels.get(mname, []) if ch not in turn_channels]
            if expression_channels:
                shapes(base + "/Keyforms", tgt, mname, expression_channels)
                run.append("Keyforms")
            if mname in HEAD_MESHES:
                lattice(base + "/HeadWarp", tgt, "HeadCage", HEAD_CAGE_DIV)
                run.append("HeadWarp")
                # The lattice and residual both add bind-space deltas.
                # Keep the authored redraw distinct from expression channels.
                if turn_channels:
                    shapes(base + "/TurnRefine", tgt, mname, turn_channels)
                    run.append("TurnRefine")
            if mname in ("Face", "Eye_R", "Eye_L", "Mouth", "Brow_R", "Brow_L"):
                matrix(base + "/HeadTilt", tgt, self.head_j)
                run.append("HeadTilt")
            elif mname in ("Hair", "BackHair", "Earring"):
                infl, idx, wts = self.hair_weights(mname)
                skin(base + "/HairSkin", tgt, infl, idx, wts, idx.shape[1])
                run.append("HairSkin")
            elif mname == "Neck":
                lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
                y = self.mesh["Neck"]["rest"][:, 1]
                wh = P.smooth(-12.0, -8.2, y)
                wb = P.smooth(-15.2, -17.5, y)
                wn = np.clip(1.0 - wh - wb, 0, 1)
                W = np.stack([wb, wn, wh], axis=1)
                W /= W.sum(axis=1, keepdims=True)
                idx = np.tile(np.array([0, 1, 2]), (len(y), 1))
                skin(base + "/SpineSkin", tgt, [self.body_j, self.neck_j, self.head_j], idx, W, 3)
                run += ["BodyWarp", "SpineSkin"]
            elif mname == "Body":
                lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
                matrix(base + "/Lean", tgt, self.body_j)
                run += ["BodyWarp", "Lean"]
            elif mname == "Ribbon":
                lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
                infl, idx, wts = self.ribbon_weights()
                skin(base + "/RibbonSkin", tgt, infl, idx, wts, idx.shape[1])
                run += ["BodyWarp", "RibbonSkin"]
            elif mname.startswith(("Arm_", "Hand_")):
                lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
                infl, idx, wts = self.limb_weights(mname)
                skin(base + "/LimbSkin", tgt, infl, idx, wts, 3)
                run += ["BodyWarp", "LimbSkin"]
            s.GetPrimAtPath(base).SetChildrenReorder(run[::-1])
            order.append(mname)
        for name, cage, div, joint in (("HeadLatticeWire", "HeadCage", HEAD_CAGE_DIV, self.head_j),
                                       ("BodyLatticeWire", "BodyCage", BODY_CAGE_DIV, self.body_j)):
            base = G + "/" + name
            scope(s, base)
            tgt = RIG + "/Guides/" + name
            lattice(base + "/Warp", tgt, cage, div)
            matrix(base + "/Follow", tgt, joint)
            s.GetPrimAtPath(base).SetChildrenReorder(["Follow", "Warp"])
            order.append(name)
        s.GetPrimAtPath(G).SetChildrenReorder(order)
        s.GetPrimAtPath(MOV).SetChildrenReorder(["Geometry", "Cages", "Channels"])

    def hair_weights(self, mname):
        m = self.mesh[mname]
        xyz = m["rest"]
        n = len(xyz)
        infl = [self.head_j]
        lookup = {}

        def add(chain):
            for jp in self.chains[chain]["joints"]:
                if jp not in lookup:
                    lookup[jp] = len(infl)
                    infl.append(jp)
            return [lookup[j] for j in self.chains[chain]["joints"]]

        ES = 4
        idx = np.zeros((n, ES), np.int64)
        wts = np.zeros((n, ES))
        wts[:, 0] = 1.0
        for pn in m["parts"]:
            a, b = m["ranges"][pn]
            pts = xyz[a:b, :2]
            rows = []
            if pn in self.chains and pn in self.pieces:
                c = self.pieces[pn]
                js = add(pn)
                t = spine_param(pts, c.spine)
                Wb = chain_bone_weights(t, self.chains[pn]["ts"])
                glue = 1.0 - P.smooth(0.0, 0.10, t)
                for i in range(len(pts)):
                    row = [(0, glue[i])] + [(js[j], (1 - glue[i]) * Wb[i, j]) for j in range(Wb.shape[1])]
                    rows.append(row)
            elif pn == "BackHair":
                L_ = add("BackL")
                R_ = add("BackR")
                x, y = pts[:, 0], pts[:, 1]
                t = np.clip((4.0 - y) / 30.0, 0, 1)
                attach = P.smooth(0.0, 0.25, t)
                sR = P.smooth(-4.0, 4.0, x)
                ts = [0.0, 0.27, 0.54, 0.8]
                Wb = chain_bone_weights(t, ts)
                for i in range(len(pts)):
                    row = [(0, 1 - attach[i])]
                    for j in range(4):
                        row.append((R_[j], attach[i] * sR[i] * Wb[i, j]))
                        row.append((L_[j], attach[i] * (1 - sR[i]) * Wb[i, j]))
                    rows.append(row)
            elif pn == "Earring":
                js = add("Earring")
                y = pts[:, 1]
                ty = D.EARRING_TOP[1]
                w1 = P.smooth(ty - 0.1, ty - 0.35, y)
                for i in range(len(pts)):
                    rows.append([(js[0], 1 - w1[i]), (js[1], w1[i])])
            else:
                continue
            for i, row in enumerate(rows):
                row = sorted([r for r in row if r[1] > 1e-4], key=lambda r: -r[1])[:ES]
                tot = sum(w for _, w in row)
                idx[a + i] = 0
                wts[a + i] = 0
                for k, (ji, w) in enumerate(row):
                    idx[a + i, k] = ji
                    wts[a + i, k] = w / tot
        return infl, idx, wts

    def ribbon_weights(self):
        m = self.mesh["Ribbon"]
        xyz = m["rest"]
        n = len(xyz)
        infl = [self.body_j]
        ES = 3
        idx = np.zeros((n, ES), np.int64)
        wts = np.zeros((n, ES))
        wts[:, 0] = 1.0
        for pn, tag in (("RibbonTailL", "TailL"), ("RibbonTailR", "TailR")):
            a, b = m["ranges"][pn]
            js = []
            for jp in self.chains[tag]["joints"]:
                js.append(len(infl))
                infl.append(jp)
            side = -1 if tag.endswith("L") else 1
            sp = P.resample(P.catmull(AB.tail_ctrl(side), n=12), n=40)
            t = spine_param(xyz[a:b, :2], sp)
            Wb = chain_bone_weights(t, [0.0, 0.4, 0.75])
            glue = 1.0 - P.smooth(0.0, 0.12, t)
            for i in range(b - a):
                row = [(0, glue[i])] + [(js[j], (1 - glue[i]) * Wb[i, j]) for j in range(3)]
                row = sorted([r for r in row if r[1] > 1e-4], key=lambda r: -r[1])[:ES]
                tot = sum(w for _, w in row)
                for k, (ji, w) in enumerate(row):
                    idx[a + i, k] = ji
                    wts[a + i, k] = w / tot
        return infl, idx, wts

    def save(self):
        self.stage.GetRootLayer().Save()


TURN_SRC = {"Turn_R": ("pos", "FaceAngle", "ry"), "Turn_L": ("neg", "FaceAngle", "ry"),
            "Nod_Down": ("pos", "FaceAngle", "rx"), "Nod_Up": ("neg", "FaceAngle", "rx")}


def body_field(xyz, breath=0.0, turn=0.0):
    """Breathing (shoulders rise, chest widens a touch) and a small torso
    turn (the far shoulder narrows, the near one widens, the V shifts)."""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    d = np.zeros_like(xyz)
    if breath:
        lift = 0.32 * P.smooth(-34.0, -18.0, y) * (1.0 - 0.3 * P.smooth(5.0, 18.0, np.abs(x)))
        widen = 0.012 * x * np.exp(-((y + 24.0) / 7.0) ** 2)
        d[:, 0] += breath * widen
        d[:, 1] += breath * lift
    if turn:
        th = math.radians(turn)
        r = 22.0
        phi = np.arcsin(np.clip(x / r, -1, 1))
        the = th * np.cos(phi) ** 0.85
        sx = r * np.sin(phi) * (np.cos(the) - 1) + 8.0 * np.cos(phi) * np.sin(the)
        d[:, 0] += sx + (z + 1.4) * math.sin(th) * 1.5
    return d


def build_rig(path, log=print):
    b = Builder(path, log)
    b.paint()
    b.write_textures()
    b.materials()
    b.meshes()
    b.cages()
    b.lattice_wire("HeadLatticeWire", (-11.0, -11.5), (11.0, 12.5), 9, 10, 0.08, (0.18, 0.80, 1.0))
    b.lattice_wire("BodyLatticeWire", (-22.0, -58.0), (22.0, -15.0), 11, 6, -0.75, (0.55, 0.45, 1.0))
    b.skeleton()
    b.articulation()
    b.expression_channels()
    b.hand_keyforms()
    b.head_turn()
    b.body_cage_channels()
    b.wiring()
    b.movers()
    import interaction
    interaction.author(b)
    b.save()
    for k, (mean, mx) in sorted(b.fit_report.items()):
        log("  head cage fit %-12s mean %.3f  max %.3f" % (k, mean, mx))
    return b


BACKDROP_BOX = (-50.0, -62.0, 50.0, 22.0)


def paint_backdrop():
    """The shot's graphic card: cool paper, a big vermilion disc behind the
    head, a halftone field and two thin rules -- flat colour, no gradient."""
    L = P.Layer("Backdrop", BACKDROP_BOX, tpu=20, ss=2)
    X, Y = L.XY()
    L.paint(np.ones_like(X), "#E3E7EC")
    # halftone: dots shrinking toward the upper right
    g = 1.35
    gx = (X / g) - np.floor(X / g) - 0.5
    gy = (Y / g) - np.floor(Y / g) - 0.5
    r = np.hypot(gx, gy) * g
    size = 0.46 * np.clip(1.0 - (X + 50) / 60.0 - (Y + 36) / 70.0, 0, 1) ** 0.8
    L.paint(np.clip((size - r) * L.s + 0.5, 0, 1), "#C8D0DA")
    disc = np.clip((15.5 - np.hypot(X - 5.5, Y - 0.2)) * L.s + 0.5, 0, 1)
    L.paint(disc, "#E0513A")
    ring = np.clip((0.22 - np.abs(np.hypot(X - 5.5, Y - 0.2) - 17.2)) * L.s + 0.5, 0, 1)
    L.paint(ring, "#E0513A")
    return L


def build_backdrop(path):
    if os.path.exists(path):
        os.remove(path)
    L = paint_backdrop()
    t = PT.finish(L, bleed=2)
    d = os.path.join(HERE, "textures")
    os.makedirs(d, exist_ok=True)
    Image.fromarray(t.rgba[..., :3], "RGB").save(os.path.join(d, "Backdrop.png"), compress_level=6)
    stage = Usd.Stage.CreateNew(path)
    stage.SetMetadata("upAxis", "Y")
    stage.SetMetadata("metersPerUnit", 0.01)
    stage.GetRootLayer().documentation = (
        "Shot dressing for the Shion bust shots: a flat graphic backdrop card behind the "
        "character (not part of the rig). Generated by build_bust.py.")
    x0, y0, x1, y1 = t.bbox
    z = -9.0
    UsdGeom.Xform.Define(stage, "/Backdrop")
    mesh = UsdGeom.Mesh.Define(stage, "/Backdrop/Card")
    mesh.CreatePointsAttr(v3f([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y1, z)]))
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", T.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec2fArray([(0, 0), (1, 0), (1, 1), (0, 1)]))
    mat = UsdShade.Material.Define(stage, "/Backdrop/Look")
    sh = UsdShade.Shader.Define(stage, "/Backdrop/Look/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", T.Float).Set(1.0)
    sh.CreateInput("specularColor", T.Color3f).Set(Gf.Vec3f(0, 0, 0))
    sh.CreateInput("useSpecularWorkflow", T.Int).Set(1)
    st = UsdShade.Shader.Define(stage, "/Backdrop/Look/UV")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", T.Token).Set("st")
    tx = UsdShade.Shader.Define(stage, "/Backdrop/Look/Texture")
    tx.CreateIdAttr("UsdUVTexture")
    tx.CreateInput("file", T.Asset).Set("./textures/Backdrop.png")
    tx.CreateInput("sourceColorSpace", T.Token).Set("sRGB")
    tx.CreateInput("st", T.Float2).ConnectToSource(st.ConnectableAPI(), "result")
    sh.CreateInput("diffuseColor", T.Color3f).ConnectToSource(tx.CreateOutput("rgb", T.Float3))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    stage.GetRootLayer().Save()


def main():
    import perf
    rig_path = os.path.join(HERE, "bust_dd_b_rig.usda")
    b = build_rig(rig_path)
    perf.write_all(b, HERE, os.path.basename(rig_path))
    build_backdrop(os.path.join(HERE, "bust_dd_b_backdrop.usda"))
    # Canonical text endings keep USD writer-version whitespace out of diffs.
    from pathlib import Path
    for name in ("rig", "anim", "sweep", "backdrop"):
        path = Path(HERE) / ("bust_dd_b_" + name + ".usda")
        path.write_text(path.read_text().rstrip() + "\n")


if __name__ == "__main__":
    main()
