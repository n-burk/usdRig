"""Build the 2D mesh-deform bust: art meshes, rig, keyforms and a performance.

    cd <repo> && source bin/_env.sh
    "$PY" examples/2d/bust/build_bust.py

writes, next to this file,

    bust_rig.usda    the character at rest: art meshes + the whole rig
    bust_anim.usda   a 180-frame looping performance, sublayering the rig

Deterministic: no randomness, no timestamps; re-running rewrites identical
files. Nothing is baked -- every frame of the performance is evaluated live
by RigExec from the control avars in bust_anim.usda.

HOW THE RIG IS BUILT (the layered mesh idea, in stock USD prims)

PARAMETERS are RigExecControls: the head-angle joystick, the eye, brow and
mouth handles, the torso/breath handle. Their avars are the parameter values
(degrees, or slider units).

KEYFORMS are RigExecBlendSamples: each channel (RigExecBlendInput) holds the
art sculpted at one or two parameter values -- e.g. Turn_R at 15 and 30
degrees -- and the blendshape mover interpolates between them exactly like a
layered mesh parameter interpolates its keyforms.

"PARAMETER DRIVES MANY DEFORMERS" is connection + property math: a channel's
inputs:weight is CONNECTED to the avar that drives it (a driven key), or,
where the parameter has to be negated for the opposite side, written by a
pair of RigExecFloatMathMovers (add the avar, multiply by -1). The mouth pad
is a pose-space RBF: a RigExecPoseInterpolator measures the mouth handle's
2D position against five vowel poses, and every vowel pose's outputs:weight
fans out to BOTH the lip keyforms and the jaw keyforms.

WARP DEFORMERS are RigExecLatticeMovers. One 7x9x2 Bernstein cage wraps the
whole head. Its keyforms (Turn / Nod) are FITTED by least squares to a
pseudo-3D turn of an ellipsoid head, and the cage's z axis spans the art's
layer depths -- back hair at z=-2 up to the fringe and hair clip at z~1.6 --
so ONE cage moves every layer by a different amount: parallax. A second cage
around the torso carries breathing and the body turn.

ROTATION DEFORMERS are joints: an FK spine (body lean, neck, head tilt) and
an FK chain in every lock of hair, skinned with RigExecSkinMovers. The hair
chains' avars are the output of a small spring simulation run here, so the
hair overlaps and settles after every head move.
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bust_geo as bg  # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt  # noqa: E402

ROOT = "/Bust"
RIG = ROOT + "/Rig"
CTL = RIG + "/Controls"
JNT = RIG + "/Joints"
GEOM = ROOT + "/Geom"
CAGES = ROOT + "/Cages"
TARGETS = ROOT + "/Targets"
BI = RIG + "/BlendInputs"
MOV = RIG + "/Movers"

FPS = 30
LOOP = 180                      # frames 1..180, frame 181 == frame 1

HEAD_PIVOT = (0.0, -9.5)
NECK_PIVOT = (0.0, -15.0)
BODY_PIVOT = (0.0, -40.0)

HEAD_CAGE_LO = np.array([-17.5, -20.0, -2.2])
HEAD_CAGE_HI = np.array([17.5, 19.0, 1.8])
HEAD_CAGE_DIV = (7, 9, 2)
BODY_CAGE_LO = np.array([-19.5, -38.5, -1.40])
BODY_CAGE_HI = np.array([19.5, -12.0, -0.70])
BODY_CAGE_DIV = (6, 5, 2)

# the vowel pad: (rx, ry) of the mouth joystick for each vowel pose
VISEME_PAD = dict(rest=(0.0, 0.0), A=(30.0, 0.0), I=(0.0, 30.0), U=(0.0, -30.0),
                  E=(21.0, 21.0), O=(21.0, -21.0))

HEAD_MESHES = ["BackHair", "Neck", "Face", "Eyes", "Brows", "Mouth", "FrontHair"]
BODY_MESHES = ["Body", "Neck"]

# USD helpers

T = Sdf.ValueTypeNames


def define(stage, path, typename="", schemas=()):
    prim = stage.DefinePrim(path, typename)
    if schemas:
        op = Sdf.TokenListOp()
        op.prependedItems = list(schemas)
        prim.SetMetadata("apiSchemas", op)
    return prim


def scope(stage, path):
    return stage.DefinePrim(path, "Scope")


def attr(prim, name, tname, value=None, uniform=False, custom=False):
    a = prim.CreateAttribute(
        name, tname, custom=custom,
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
    a = np.asarray(a, dtype=np.float32)
    a = np.round(a, 4)
    return Vt.Vec3fArray.FromNumpy(a)


def r4(a):
    return np.round(np.asarray(a, dtype=np.float64), 4)


# The pseudo-3D head turn that the cage keyforms are fitted to

RX, RY, RZ, CY = 10.5, 14.0, 8.0, -1.0
KZ = 0.55


def head_turn_field(xyz, ax, ay):
    """Displacement of art points for a head turned ax degrees (to screen
    right) and nodded ay degrees (up), modelling the head as an ellipsoid
    whose front surface slides around it while its silhouette holds, plus
    depth parallax from each layer's z."""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    th = math.radians(ax)
    ps = math.radians(ay)
    yy = y - CY
    fy = np.sqrt(np.clip(1 - (yy / RY) ** 2, 0, 1))
    r = np.maximum(RX * fy, 3.0)
    dz = RZ * fy
    phi = np.arcsin(np.clip(x / r, -1, 1))
    the = th * np.cos(phi) ** 0.85
    sx = r * np.sin(phi) * (np.cos(the) - 1) + dz * np.cos(phi) * np.sin(the)
    fade = bg.smoothstep(-19.0, -13.5, y)
    dx = (sx + z * math.sin(th) * KZ) * fade
    fx = np.sqrt(np.clip(1 - (x / RX) ** 2, 0, 1))
    rv = np.maximum(RY * fx, 3.0)
    dzv = RZ * fx
    phy = np.arcsin(np.clip(yy / rv, -1, 1))
    tye = ps * np.cos(phy) ** 0.85
    sy = rv * np.sin(phy) * (np.cos(tye) - 1) + dzv * np.cos(phy) * np.sin(tye)
    dy = (sy + z * math.sin(ps) * KZ) * fade
    return np.stack([dx, dy, np.zeros_like(dx)], axis=1)


def body_field(xyz, breath=0.0, turn=0.0):
    """Breathing (shoulders rise, chest widens) and a small torso turn."""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    d = np.zeros_like(xyz)
    if breath:
        lift = 0.42 * bg.smoothstep(-31.0, -18.0, y) * (1.0 - 0.25 * bg.smoothstep(4.0, 16.0, np.abs(x)))
        widen = 0.018 * x * np.exp(-((y + 24.0) / 6.0) ** 2)
        d[:, 0] += breath * widen
        d[:, 1] += breath * lift
    if turn:
        th = math.radians(turn)
        r = 18.0
        phi = np.arcsin(np.clip(x / r, -1, 1))
        the = th * np.cos(phi) ** 0.85
        sx = r * np.sin(phi) * (np.cos(the) - 1) + 9.0 * np.cos(phi) * np.sin(the)
        d[:, 0] += sx + (z + 1.0) * math.sin(th) * 1.2
    return d


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
    return (Bz[:, :, None, None] * By[:, None, :, None] *
            Bx[:, None, None, :]).reshape(len(P), -1)


def cage_points(lo, hi, div):
    xs = np.linspace(lo[0], hi[0], div[0])
    ys = np.linspace(lo[1], hi[1], div[1])
    zs = np.linspace(lo[2], hi[2], div[2])
    return np.array([(x, y, z) for z in zs for y in ys for x in xs])


def fit_cage(samples, displacement, lo, hi, div, lam=1e-4, field=None,
             fill_weight=0.3):
    """Least-squares cage deltas reproducing `displacement` at `samples`
    through the (Bernstein) lattice the mover evaluates.

    `field`, when given, is also sampled on a regular grid over the whole
    cage at a lower weight, so cage points far from any art stay calm
    instead of drifting wherever the fit leaves them free."""
    B = bernstein_basis(samples, lo, hi, div)
    rows = [B]
    rhs = [displacement[:, :2]]
    if field is not None:
        g = np.stack(np.meshgrid(np.linspace(lo[0], hi[0], 36),
                                 np.linspace(lo[1], hi[1], 40),
                                 np.linspace(lo[2], hi[2], 3)), axis=-1).reshape(-1, 3)
        rows.append(fill_weight * bernstein_basis(g, lo, hi, div))
        rhs.append(fill_weight * field(g)[:, :2])
    rows.append(math.sqrt(lam) * np.eye(B.shape[1]))
    rhs.append(np.zeros((B.shape[1], 2)))
    A = np.vstack(rows)
    rhs = np.vstack(rhs)
    C, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    out = np.zeros((B.shape[1], 3))
    out[:, :2] = C
    return out


# Hair chains

def hair_chains():
    """[(chain name, joint world positions, mesh, strand parts, t of joints)]"""
    chains = []
    for name, ctrl, w, lean in bg.BANGS:
        spine = bg.strand_spine(ctrl, 24)
        ts = [0.0, 0.40, 0.72]
        chains.append((name, [spine[int(round(t * 23))] for t in ts], "FrontHair", [name, name + "_line"], ts))
    for name, ctrl, w, lean in bg.SIDELOCKS:
        spine = bg.strand_spine(ctrl, 30)
        ts = [0.0, 0.28, 0.54, 0.78] if name.startswith("Side_") else [0.0, 0.45]
        chains.append((name, [spine[int(round(t * 29))] for t in ts], "FrontHair", [name, name + "_line"], ts))
    name, ctrl, w, lean = bg.AHOGE
    spine = bg.strand_spine(ctrl, 26)
    ts = [0.0, 0.34, 0.68]
    chains.append((name, [spine[int(round(t * 25))] for t in ts], "FrontHair", [name, name + "_line"], ts))
    for side, tag in ((1, "R"), (-1, "L")):
        pts = [(11.2 * side, 5.0), (12.6 * side, -4.5), (13.6 * side, -13.5), (14.4 * side, -21.5)]
        chains.append(("Back_" + tag, [np.array(p) for p in pts], "BackHair", None, None))
    return chains


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


# Keyform (blend target) generation

def group_xyz(parts):
    return bg.merge(parts)["xyz"]


def keyforms():
    """{mesh: {channel: [(activation, displaced xyz)]}} for the feature
    meshes, from the same generators as the rest art."""
    rest = {
        "Eyes": group_xyz(bg.eye(1) + bg.eye(-1)),
        "Brows": group_xyz(bg.brow(1) + bg.brow(-1)),
        "Mouth": group_xyz(bg.mouth("rest")),
    }
    eyesR = len(group_xyz(bg.eye(1)))
    k = {"Eyes": {}, "Brows": {}, "Mouth": {}, "Face": {}}

    def eyes(r_shape="open", l_shape="open", look=(0, 0)):
        return group_xyz(bg.eye(1, r_shape, look) + bg.eye(-1, l_shape, look))

    k["Eyes"]["Blink_R"] = [(0.5, eyes(r_shape="half")), (1.0, eyes(r_shape="closed"))]
    k["Eyes"]["Blink_L"] = [(0.5, eyes(l_shape="half")), (1.0, eyes(l_shape="closed"))]
    k["Eyes"]["EyeSmile_R"] = [(20.0, eyes(r_shape="smile"))]
    k["Eyes"]["EyeSmile_L"] = [(20.0, eyes(l_shape="smile"))]
    k["Eyes"]["Look_R"] = [(1.0, eyes(look=(0.85, 0.0)))]
    k["Eyes"]["Look_L"] = [(1.0, eyes(look=(-0.85, 0.0)))]
    k["Eyes"]["Look_U"] = [(1.0, eyes(look=(0.0, 0.55)))]
    k["Eyes"]["Look_D"] = [(1.0, eyes(look=(0.0, -0.50)))]

    def brows(r="rest", l="rest"):
        return group_xyz(bg.brow(1, r) + bg.brow(-1, l))

    for side, tag in ((1, "R"), (-1, "L")):
        for ch, shape, act in (("BrowUp", "up", 1.0), ("BrowDown", "down", 1.0),
                               ("BrowSad", "sad", 20.0), ("BrowAngry", "angry", 20.0)):
            k["Brows"]["%s_%s" % (ch, tag)] = [(act, brows(r=shape) if side > 0 else brows(l=shape))]

    for v in "AIUEO":
        k["Mouth"]["Vis_" + v] = [(1.0, group_xyz(bg.mouth(v)))]
    k["Mouth"]["Smile"] = [(20.0, group_xyz(bg.mouth("smile")))]
    k["Mouth"]["Frown"] = [(20.0, group_xyz(bg.mouth("frown")))]
    return rest, k


def jaw_keyforms(face_xyz):
    """The jaw drops with the open vowels: chin and jaw line move down."""
    out = {}
    drops = dict(A=0.62, I=0.16, U=0.30, E=0.34, O=0.55)
    x, y = face_xyz[:, 0], face_xyz[:, 1]
    w = bg.smoothstep(-8.6, -12.6, y) * bg.smoothstep(9.5, 3.0, np.abs(x))
    for v, d in drops.items():
        p = face_xyz.copy()
        p[:, 1] -= d * w
        p[:, 0] *= 1.0 - 0.025 * d * w * (v in "UO")
        out["Jaw_" + v] = [(1.0, p)]
    return out


# Stage authoring

class Builder(object):
    def __init__(self, path):
        if os.path.exists(path):
            os.remove(path)
        self.stage = Usd.Stage.CreateNew(path)
        s = self.stage
        s.SetMetadata("upAxis", "Y")
        s.SetMetadata("metersPerUnit", 0.01)
        s.SetStartTimeCode(1)
        s.SetEndTimeCode(LOOP)
        s.SetTimeCodesPerSecond(FPS)
        s.SetFramesPerSecond(FPS)
        root = UsdGeom.Xform.Define(s, ROOT)
        s.SetDefaultPrim(root.GetPrim())
        s.GetRootLayer().documentation = (
            "RigExec example: a 2D mesh-deform (layered mesh deformation) bust, "
            "rigged entirely with stock RigExec prims. Generated by "
            "build_bust.py -- edit that, not this file.")
        self.mesh_rest = {}
        self.ranges = {}
        self.parts = {}

    def meshes(self):
        s = self.stage
        scope(s, GEOM)
        groups = bg.mesh_groups()
        for name, parts in groups.items():
            m = bg.merge(parts)
            mesh = UsdGeom.Mesh.Define(s, GEOM + "/" + name)
            mesh.CreatePointsAttr(v3f(m["xyz"]))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(m["tris"])))
            mesh.CreateFaceVertexIndicesAttr(
                Vt.IntArray.FromNumpy(m["tris"].reshape(-1).astype(np.int32)))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            lo, hi = m["xyz"].min(axis=0), m["xyz"].max(axis=0)
            mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
            pv = mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
            pv.Set(v3f(np.clip(m["colors"], 0, 1)))
            self.mesh_rest[name] = r4(m["xyz"])
            self.ranges[name] = m["ranges"]
            self.parts[name] = m["parts"]
        return groups

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

    def lattice_wire(self, name, lo, hi, nx, ny, z, color):
        """Guide-purpose grid lines over the art, deformed by the same
        lattice, so the warp is visible in the viewport."""
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
        c.CreateWidthsAttr(Vt.FloatArray([0.09]))
        c.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        c.CreatePurposeAttr("guide")
        c.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        c.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant).Set(Vt.FloatArray([0.75]))
        self.mesh_rest[name] = r4(np.array(pts))
        return c

    def control(self, path, pos_local, shape="sphere", scale=(0.5, 0.5, 0.5),
                offset=(0, 0, 0), color=(1.0, 0.85, 0.25), width=0.06):
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

    def skeleton(self):
        s = self.stage
        rig = define(s, RIG, "RigExecRoot")
        attr(rig, "rigExec:partition", T.Token, "Bust", uniform=True)
        for sc in ("Controls", "Joints", "Solvers", "BlendInputs", "PoseInterpolators",
                   "Movers", "Guides"):
            scope(s, RIG + "/" + sc)

        ORANGE = (1.0, 0.62, 0.22)
        YELLOW = (1.0, 0.86, 0.22)
        AQUA = (0.30, 0.95, 0.88)
        bx, by = BODY_PIVOT
        nx, ny = NECK_PIVOT
        hx, hy = HEAD_PIVOT
        body_ctl = CTL + "/Body_ctl"
        neck_ctl = body_ctl + "/Neck_ctl"
        head_ctl = neck_ctl + "/Head_ctl"
        self.control(body_ctl, (bx, by, 0), "cube", (16.5, 0.9, 0.2), (0, 9.8, 0.0), ORANGE, 0.10)
        self.control(neck_ctl, (nx - bx, ny - by, 0), "cube", (3.9, 0.7, 0.2), (0, 0.0, 0), ORANGE, 0.08)
        self.control(head_ctl, (hx - nx, hy - ny, 0), "cube", (13.2, 13.6, 0.2), (0, 11.6, 0), ORANGE, 0.08)
        self.control(body_ctl + "/Torso_ctl", (0 - bx, -22.0 - by, 0), "diamond", (0.7, 0.7, 0.7), (0, 0, 0), YELLOW)
        # face parameters: children of the head handle, so they ride the tilt
        H = np.array([hx, hy])
        face = {
            "FaceAngle_ctl": ((0.0, -5.6), "sphere", (0.9, 0.9, 0.9), (0, 0, 4.5)),
            "Look_ctl": ((0.0, -2.9), "diamond", (0.55, 0.55, 0.55), (0, 0, 0)),
            "Eye_R_ctl": ((4.4, 0.05), "diamond", (0.42, 0.42, 0.42), (0, 0, 0)),
            "Eye_L_ctl": ((-4.4, 0.05), "diamond", (0.42, 0.42, 0.42), (0, 0, 0)),
            "Brow_R_ctl": ((4.1, 3.35), "diamond", (0.42, 0.42, 0.42), (0, 0, 0)),
            "Brow_L_ctl": ((-4.1, 3.35), "diamond", (0.42, 0.42, 0.42), (0, 0, 0)),
            "Mouth_ctl": ((0.0, -9.4), "sphere", (0.5, 0.5, 0.5), (0, 0, 2.2)),
        }
        for name, (pos, shape, sc, off) in face.items():
            local = np.array(pos) - H
            self.control(head_ctl + "/" + name, (local[0], local[1], 2.5), shape, sc, off, YELLOW)

        self.joint(JNT + "/Body", (bx, by, 0), 0.45, (1.0, 0.55, 0.25), 0.7)
        self.joint(JNT + "/Body/Neck", (nx - bx, ny - by, 0), 0.4, (1.0, 0.55, 0.25), 0.7)
        self.joint(JNT + "/Body/Neck/Head", (hx - nx, hy - ny, 0), 0.45, (1.0, 0.55, 0.25), 0.7)
        self.fk("Spine", [body_ctl, neck_ctl, head_ctl],
                [JNT + "/Body", JNT + "/Body/Neck", JNT + "/Body/Neck/Head"])
        self.head_ctl = head_ctl
        self.body_ctl = body_ctl

        # hair: one FK chain per lock, controls nested under the head handle
        scope(s, JNT + "/Hair")
        self.chains = hair_chains()
        self.chain_joints = {}
        self.chain_controls = {}
        for name, pts, mesh, parts, ts in self.chains:
            cparent, jparent = head_ctl, JNT + "/Hair"
            prev_c = np.array(H)
            prev_j = np.zeros(2)
            cpaths, jpaths = [], []
            for k, p in enumerate(pts):
                p = np.asarray(p, dtype=float)
                cpath = cparent + "/%s_%d_ctl" % (name, k)
                lc = p - prev_c
                big = name.startswith("Back")
                self.control(cpath, (lc[0], lc[1], 0.0), "sphere",
                             (0.55, 0.55, 0.55) if big else (0.36, 0.36, 0.36), (0, 0, 0), AQUA, 0.05)
                jpath = jparent + "/%s_%d" % (name, k)
                lj = p - prev_j
                self.joint(jpath, (lj[0], lj[1], 0.0), 0.34 if big else 0.24, (0.62, 0.40, 1.0), 0.95)
                cpaths.append(cpath)
                jpaths.append(jpath)
                cparent, jparent = cpath, jpath
                prev_c = p
                prev_j = p
            self.fk("Hair_" + name, cpaths, jpaths)
            self.chain_joints[name] = jpaths
            self.chain_controls[name] = cpaths

    def blend_channel(self, mesh, channel, samples, rest):
        """One RigExecBlendInput with sparse UsdSkelBlendShape samples."""
        s = self.stage
        bpath = "%s/%s/%s" % (BI, mesh, channel)
        b = define(s, bpath, "RigExecBlendInput")
        attr(b, "inputs:weight", T.Float, 0.0)
        spaths = []
        for act, xyz in samples:
            d = r4(np.asarray(xyz) - rest)
            nz = np.nonzero(np.abs(d).max(axis=1) > 1e-5)[0]
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
        return b

    def all_channels(self):
        s = self.stage
        scope(s, TARGETS)
        for m in ("Eyes", "Brows", "Mouth", "Face", "HeadCage", "BodyCage"):
            scope(s, TARGETS + "/" + m)
            scope(s, BI + "/" + m)
        rest, k = keyforms()
        for mesh in ("Eyes", "Brows", "Mouth"):
            assert np.allclose(rest[mesh], self.mesh_rest[mesh], atol=2e-4), mesh
        k["Face"] = jaw_keyforms(self.mesh_rest["Face"])
        self.channels = {}
        for mesh, chans in k.items():
            for ch, samples in chans.items():
                self.blend_channel(mesh, ch, samples, self.mesh_rest[mesh])
                self.channels.setdefault(mesh, []).append(ch)

        # --- head cage keyforms, fitted to the pseudo-3D turn -----------------
        samples = np.vstack([self.mesh_rest[m] for m in HEAD_MESHES])
        self.fit_report = {}
        head = {}
        for ch, sgn, axis, acts in (("Turn_R", 1, 0, (15.0, 30.0)), ("Turn_L", -1, 0, (15.0, 30.0)),
                                    ("Nod_Up", 1, 1, (10.0, 20.0)), ("Nod_Down", -1, 1, (10.0, 20.0))):
            ks = []
            for a in acts:
                ang = (sgn * a, 0.0) if axis == 0 else (0.0, sgn * a)
                D = head_turn_field(samples, *ang)
                C = fit_cage(samples, D, HEAD_CAGE_LO, HEAD_CAGE_HI, HEAD_CAGE_DIV,
                             field=lambda g, ang=ang: head_turn_field(g, *ang))
                B = bernstein_basis(samples, HEAD_CAGE_LO, HEAD_CAGE_HI, HEAD_CAGE_DIV)
                err = np.linalg.norm(B @ C[:, :2] - D[:, :2], axis=1)
                self.fit_report["%s@%g" % (ch, a)] = (float(err.mean()), float(err.max()))
                ks.append((a, self.head_cage + C))
            head[ch] = ks
        for ch, ks in head.items():
            self.blend_channel("HeadCage", ch, ks, r4(self.head_cage))
        # --- body cage keyforms --------------------------------------------------
        bsamples = np.vstack([self.mesh_rest[m] for m in BODY_MESHES])
        body = {}
        for ch, kw in (("Breath", dict(breath=1.0)), ("BodyTurn_R", dict(turn=10.0)),
                       ("BodyTurn_L", dict(turn=-10.0))):
            D = body_field(bsamples, **kw)
            C = fit_cage(bsamples, D, BODY_CAGE_LO, BODY_CAGE_HI, BODY_CAGE_DIV,
                         field=lambda g, kw=kw: body_field(g, **kw))
            act = 1.0 if ch == "Breath" else 10.0
            body[ch] = [(act, self.body_cage + C)]
        for ch, ks in body.items():
            self.blend_channel("BodyCage", ch, ks, r4(self.body_cage))

    def wiring(self):
        s = self.stage
        H = self.head_ctl
        B = self.body_ctl
        scope(s, MOV + "/Channels")
        avar = {
            "FaceAngle": H + "/FaceAngle_ctl", "Look": H + "/Look_ctl",
            "Eye_R": H + "/Eye_R_ctl", "Eye_L": H + "/Eye_L_ctl",
            "Brow_R": H + "/Brow_R_ctl", "Brow_L": H + "/Brow_L_ctl",
            "Mouth": H + "/Mouth_ctl", "Torso": B + "/Torso_ctl",
        }

        def connect(mesh, ch, ctl, channel):
            w = s.GetPrimAtPath("%s/%s/%s" % (BI, mesh, ch)).GetAttribute("inputs:weight")
            w.SetConnections([Sdf.Path("%s.avars:%s" % (avar[ctl], channel))])

        def negated(mesh, ch, ctl, channel):
            """weight = -avar: two property movers on the channel's weight."""
            target = "%s/%s/%s.inputs:weight" % (BI, mesh, ch)
            base = "%s/Channels/%s_%s" % (MOV, mesh, ch)
            scope(s, base)
            read = define(s, base + "/Read", "RigExecFloatMathMover", ["RigExecMoverAPI"])
            attr(read, "rigExec:operation", T.Token, "add", uniform=True)
            v = attr(read, "inputs:value", T.Float, 0.0)
            v.SetConnections([Sdf.Path("%s.avars:%s" % (avar[ctl], channel))])
            rel(read, "rigExec:moves", [target])
            neg = define(s, base + "/Negate", "RigExecFloatMathMover", ["RigExecMoverAPI"])
            attr(neg, "rigExec:operation", T.Token, "multiply", uniform=True)
            attr(neg, "inputs:value", T.Float, -1.0)
            rel(neg, "rigExec:moves", [target])
            # bottom sibling runs first: Read, then Negate
            s.GetPrimAtPath(base).SetChildrenReorder(["Negate", "Read"])

        connect("HeadCage", "Turn_R", "FaceAngle", "ry")
        negated("HeadCage", "Turn_L", "FaceAngle", "ry")
        # +rx pitches the face DOWN (the +Y axis rolls toward the camera)
        connect("HeadCage", "Nod_Down", "FaceAngle", "rx")
        negated("HeadCage", "Nod_Up", "FaceAngle", "rx")
        connect("BodyCage", "Breath", "Torso", "ty")
        connect("BodyCage", "BodyTurn_R", "Torso", "ry")
        negated("BodyCage", "BodyTurn_L", "Torso", "ry")
        for side in ("R", "L"):
            negated("Eyes", "Blink_" + side, "Eye_" + side, "ty")
            connect("Eyes", "EyeSmile_" + side, "Eye_" + side, "rz")
            connect("Brows", "BrowUp_" + side, "Brow_" + side, "ty")
            negated("Brows", "BrowDown_" + side, "Brow_" + side, "ty")
            connect("Brows", "BrowSad_" + side, "Brow_" + side, "rz")
            negated("Brows", "BrowAngry_" + side, "Brow_" + side, "rz")
        connect("Eyes", "Look_R", "Look", "tx")
        negated("Eyes", "Look_L", "Look", "tx")
        connect("Eyes", "Look_U", "Look", "ty")
        negated("Eyes", "Look_D", "Look", "ty")
        connect("Mouth", "Smile", "Mouth", "rz")
        negated("Mouth", "Frown", "Mouth", "rz")

        # --- the vowel pad: a pose-space RBF over the mouth handle's swing.
        # The handle is a joystick: rx tips it down (open), ry left/right
        # (round / wide). Its rz is the smile dial, which is TWIST about the
        # handle's Z axis and so invisible to these swing poses.
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
                for mesh, ch in (("Mouth", "Vis_" + name), ("Face", "Jaw_" + name)):
                    w = s.GetPrimAtPath("%s/%s/%s" % (BI, mesh, ch)).GetAttribute("inputs:weight")
                    w.SetConnections([p.GetPath().AppendProperty("outputs:weight")])

    def movers(self):
        s = self.stage
        G = MOV + "/Geometry"
        scope(s, G)
        scope(s, MOV + "/Cages")
        head_j = JNT + "/Body/Neck/Head"
        neck_j = JNT + "/Body/Neck"
        body_j = JNT + "/Body"

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
            rel(m, "rigExec:transform", [joint], phase="final")
            rel(m, "rigExec:moves", [target + ".points"])
            return m

        def skin(path, target, influences, idx, wts, esize):
            m = mover(path, "RigExecSkinMover")
            rel(m, "rigExec:influences", influences, phase="final")
            attr(m, "rigExec:jointIndices", T.IntArray, Vt.IntArray.FromNumpy(idx.reshape(-1).astype(np.int32)))
            attr(m, "rigExec:jointWeights", T.FloatArray, Vt.FloatArray.FromNumpy(np.round(wts.reshape(-1), 4).astype(np.float32)))
            attr(m, "rigExec:elementSize", T.Int, esize, uniform=True)
            attr(m, "rigExec:skinningMethod", T.Token, "classicLinear", uniform=True)
            rel(m, "rigExec:moves", [target + ".points"])
            return m

        def shapes(path, target, mesh):
            m = mover(path, "RigExecBlendShapeMover")
            rel(m, "rigExec:blendInputs", ["%s/%s/%s" % (BI, mesh, ch) for ch in self.channels[mesh]])
            rel(m, "rigExec:moves", [target + ".points"])
            return m

        # cage keyforms
        for cage, mesh in (("HeadCage", "HeadCage"), ("BodyCage", "BodyCage")):
            m = mover("%s/Cages/%sKeyforms" % (MOV, cage), "RigExecBlendShapeMover")
            rel(m, "rigExec:blendInputs", ["%s/%s/%s" % (BI, mesh, ch) for ch in
                                           [c.GetName() for c in s.GetPrimAtPath(BI + "/" + mesh).GetChildren()]])
            rel(m, "rigExec:moves", [CAGES + "/" + cage + ".points"])

        # feature meshes: keyforms -> head -> head lattice
        for mesh in ("Face", "Eyes", "Brows", "Mouth"):
            base = G + "/" + mesh
            scope(s, base)
            tgt = GEOM + "/" + mesh
            lattice(base + "/HeadWarp", tgt, "HeadCage", HEAD_CAGE_DIV)
            matrix(base + "/HeadTilt", tgt, head_j)
            shapes(base + "/Keyforms", tgt, mesh)
            s.GetPrimAtPath(base).SetChildrenReorder(["HeadWarp", "HeadTilt", "Keyforms"])

        # front hair: skinned to the lock chains (+ head for the cap)
        self._hair_skin(G, "FrontHair", head_j, skin, lattice)
        self._hair_skin(G, "BackHair", head_j, skin, lattice)

        # neck: spine skin, then the head warp (fading out down the neck), then breath
        base = G + "/Neck"
        scope(s, base)
        tgt = GEOM + "/Neck"
        y = self.mesh_rest["Neck"][:, 1]
        wh = bg.smoothstep(-15.5, -11.0, y)
        wb = bg.smoothstep(-15.5, -19.0, y)
        wn = np.clip(1.0 - wh - wb, 0, 1)
        W = np.stack([wb, wn, wh], axis=1)
        W /= W.sum(axis=1, keepdims=True)
        idx = np.tile(np.array([0, 1, 2]), (len(y), 1))
        lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
        lattice(base + "/HeadWarp", tgt, "HeadCage", HEAD_CAGE_DIV)
        skin(base + "/SpineSkin", tgt, [body_j, neck_j, head_j], idx, W, 3)
        s.GetPrimAtPath(base).SetChildrenReorder(["BodyWarp", "HeadWarp", "SpineSkin"])

        # body: lean, then breath/turn warp
        base = G + "/Body"
        scope(s, base)
        tgt = GEOM + "/Body"
        lattice(base + "/BodyWarp", tgt, "BodyCage", BODY_CAGE_DIV)
        matrix(base + "/Lean", tgt, body_j)
        s.GetPrimAtPath(base).SetChildrenReorder(["BodyWarp", "Lean"])

        # the lattice wires (guides) ride the same stacks
        for name, cage, div, joint in (("HeadLatticeWire", "HeadCage", HEAD_CAGE_DIV, head_j),
                                       ("BodyLatticeWire", "BodyCage", BODY_CAGE_DIV, body_j)):
            base = G + "/" + name
            scope(s, base)
            tgt = RIG + "/Guides/" + name
            lattice(base + "/Warp", tgt, cage, div)
            matrix(base + "/Follow", tgt, joint)
            s.GetPrimAtPath(base).SetChildrenReorder(["Warp", "Follow"])

        s.GetPrimAtPath(G).SetChildrenReorder(
            ["Face", "Eyes", "Brows", "Mouth", "FrontHair", "BackHair", "Neck", "Body",
             "HeadLatticeWire", "BodyLatticeWire"])
        s.GetPrimAtPath(MOV).SetChildrenReorder(["Geometry", "Cages", "Channels"])

    def _hair_skin(self, G, mesh, head_j, skin, lattice):
        s = self.stage
        xyz = self.mesh_rest[mesh]
        n = len(xyz)
        infl = [head_j]
        lookup = {}
        for name, pts, cmesh, parts, ts in self.chains:
            if cmesh != mesh:
                continue
            for jp in self.chain_joints[name]:
                lookup[jp] = len(infl)
                infl.append(jp)
        ES = 4
        idx = np.zeros((n, ES), dtype=np.int64)
        wts = np.zeros((n, ES))
        wts[:, 0] = 1.0                      # default: the head
        if mesh == "FrontHair":
            ranges = self.ranges[mesh]
            parts = {p.name: p for p in self.parts[mesh]}
            for name, pts, cmesh, pnames, ts in self.chains:
                if cmesh != mesh:
                    continue
                joints = [lookup[j] for j in self.chain_joints[name]]
                for pn in pnames:
                    a, b = ranges[pn]
                    t = np.asarray(parts[pn].attrs["t"])
                    Wb = chain_bone_weights(t, ts)
                    # the first few percent stay glued to the head
                    glue = 1.0 - bg.smoothstep(0.0, 0.08, t)
                    k = min(ES - 1, Wb.shape[1])
                    order = np.argsort(-Wb, axis=1)[:, :k]
                    for i in range(b - a):
                        row = [(0, glue[i])]
                        for j in order[i]:
                            row.append((joints[j], (1 - glue[i]) * Wb[i, j]))
                        tot = sum(w for _, w in row)
                        for c, (ji, w) in enumerate(row):
                            idx[a + i, c] = ji
                            wts[a + i, c] = w / tot
        else:
            x, y = xyz[:, 0], xyz[:, 1]
            t = np.clip((5.0 - y) / 26.5, 0, 1)
            attach = bg.smoothstep(0.0, 0.22, t)
            sR = bg.smoothstep(-5.0, 5.0, x)
            ts = [0.0, 0.358, 0.698, 1.0]
            Wb = chain_bone_weights(t, ts[:-1] + [0.999])
            R = [lookup[j] for j in self.chain_joints["Back_R"]]
            L = [lookup[j] for j in self.chain_joints["Back_L"]]
            for i in range(n):
                top = np.argsort(-Wb[i])[:2]
                row = [(0, 1 - attach[i])]
                for j in top:
                    row.append((R[j], attach[i] * sR[i] * Wb[i, j]))
                    row.append((L[j], attach[i] * (1 - sR[i]) * Wb[i, j]))
                row = sorted(row, key=lambda r: -r[1])[:ES]
                tot = sum(w for _, w in row)
                for c, (ji, w) in enumerate(row):
                    idx[i, c] = ji
                    wts[i, c] = w / tot
        base = G + "/" + mesh
        scope(s, base)
        tgt = GEOM + "/" + mesh
        lattice(base + "/HeadWarp", tgt, "HeadCage", HEAD_CAGE_DIV)
        skin(base + "/HairSkin", tgt, infl, idx, wts, ES)
        s.GetPrimAtPath(base).SetChildrenReorder(["HeadWarp", "HairSkin"])

    def save(self):
        self.stage.GetRootLayer().Save()


# The performance

def pchip_loop(keys, frames):
    """Monotone cubic through (frame, value) keys, sampled at frames."""
    from scipy.interpolate import PchipInterpolator
    k = sorted(keys)
    xs = np.array([f for f, _ in k], dtype=float)
    ys = np.array([v for _, v in k], dtype=float)
    f = PchipInterpolator(xs, ys, extrapolate=True)
    return f(np.clip(frames, xs[0], xs[-1]))


def blink_keys(starts, closed=-1.0, end=LOOP + 1):
    keys = [(1, 0.0)]
    for s0 in starts:
        keys += [(s0, 0.0), (s0 + 2, closed * 0.75), (s0 + 3, closed), (s0 + 4, closed), (s0 + 7, 0.0)]
    keys.append((end, 0.0))
    return keys


def performance():
    F = np.arange(1, LOOP + 2, dtype=float)       # 1..181 (181 == 1)
    A = {}
    A["FaceAngle.ry"] = pchip_loop([(1, 0), (12, 0), (30, 24), (46, 19), (62, 4), (70, 2), (86, -22),
                                    (100, -17), (114, -4), (126, 3), (142, 0), (156, 7), (170, 1), (181, 0)], F)
    A["FaceAngle.rx"] = pchip_loop([(1, 0), (16, -5), (34, 3), (50, -3), (64, 5), (80, 0), (96, -4),
                                    (108, -10), (118, -6), (128, -9), (140, 4), (158, 1), (181, 0)], F)
    A["Head.rz"] = pchip_loop([(1, 0), (14, 0), (32, -7), (48, -5), (64, 1), (88, 8), (102, 6),
                               (110, 2), (116, 5), (124, 1), (132, 4), (146, -2), (164, 0), (181, 0)], F)
    A["Neck.rz"] = A["Head.rz"] * 0.35
    A["Body.rz"] = pchip_loop([(1, 0), (36, -1.2), (70, 0.3), (96, 1.5), (120, 0.4), (150, -0.6), (181, 0)], F)
    A["Body.tx"] = pchip_loop([(1, 0), (40, 0.25), (95, -0.3), (140, 0.1), (181, 0)], F)
    # breathing: two full cycles per loop, always on
    A["Torso.ty"] = 0.5 - 0.5 * np.cos(2 * np.pi * (F - 1) / 90.0)
    # the torso turns with the head, a few frames late
    lag = np.roll(A["FaceAngle.ry"], 5)
    A["Torso.ry"] = 0.32 * lag
    A["Look.tx"] = pchip_loop([(1, 0), (8, 0), (15, 0.85), (34, 0.6), (48, 0.1), (60, -0.05), (68, -0.2),
                               (74, -0.9), (92, -0.7), (104, -0.1), (130, 0.0), (144, 0.25), (160, 0.1), (181, 0)], F)
    A["Look.ty"] = pchip_loop([(1, 0), (15, 0.3), (40, 0.0), (74, 0.15), (100, -0.2), (130, 0.1), (160, 0), (181, 0)], F)
    blinks = [6, 56, 94, 150]
    A["Eye_R.ty"] = pchip_loop(blink_keys(blinks), F)
    A["Eye_L.ty"] = A["Eye_R.ty"].copy()
    smile_eye = [(1, 0), (100, 0), (106, 20), (126, 20), (133, 0), (181, 0)]
    A["Eye_R.rz"] = pchip_loop(smile_eye, F)
    A["Eye_L.rz"] = pchip_loop(smile_eye, F)
    A["Brow_R.ty"] = pchip_loop([(1, 0), (14, 0), (24, 0.85), (44, 0.7), (58, 0.1), (74, 0.0), (80, -0.35),
                                 (90, -0.2), (102, 0.55), (126, 0.6), (140, 0.1), (181, 0)], F)
    A["Brow_L.ty"] = pchip_loop([(1, 0), (14, 0), (24, 0.95), (44, 0.75), (58, 0.1), (74, 0.3), (82, 0.55),
                                 (92, 0.2), (102, 0.55), (126, 0.6), (140, 0.1), (181, 0)], F)
    A["Brow_R.rz"] = pchip_loop([(1, 0), (60, 0), (72, -7), (86, -6), (100, 12), (126, 12), (140, 0), (181, 0)], F)
    A["Brow_L.rz"] = pchip_loop([(1, 0), (60, 0), (72, 4), (86, 2), (100, 12), (126, 12), (140, 0), (181, 0)], F)
    # mouth: the vowel pad (tx, ty) and the smile dial (rz)
    pad = VISEME_PAD
    talk = [(1, "rest"), (36, "rest"), (39, "A"), (44, "I"), (48, "rest"), (51, "U"), (55, "E"), (59, "rest"),
            (62, "O"), (66, "A"), (70, "rest"),
            (104, "rest"), (107, "A"), (111, "E"), (115, "A"), (119, "E"), (123, "A"), (129, "rest"),
            (136, "rest"), (139, "E"), (143, "I"), (147, "rest"), (150, "A"), (155, "O"), (160, "rest"),
            (181, "rest")]
    A["Mouth.rx"] = pchip_loop([(f, pad[v][0]) for f, v in talk], F)
    A["Mouth.ry"] = pchip_loop([(f, pad[v][1]) for f, v in talk], F)
    A["Mouth.rz"] = pchip_loop([(1, 5), (30, 8), (40, 4), (72, 5), (80, -6), (90, -3), (100, 12), (106, 20),
                                (126, 20), (136, 8), (160, 6), (181, 5)], F)
    return F, A


SWEEP = 120                     # the parameter sweep: frames 1..120


def sweep():
    """A layered mesh parameter demo for the rig reveal: the head angle
    swept through its full keyform range (turn +-30, nod +-15) with the tilt,
    then a blink and the five vowels -- every deformer family in 4 seconds."""
    F = np.arange(1, SWEEP + 2, dtype=float)
    A = {}
    E = SWEEP + 1
    A["FaceAngle.ry"] = pchip_loop([(1, 0), (10, 0), (28, 30), (36, 30), (56, -30), (64, -30), (78, 0), (E, 0)], F)
    A["FaceAngle.rx"] = pchip_loop([(1, 0), (74, 0), (86, -15), (92, -15), (102, 12), (110, 0), (E, 0)], F)
    A["Head.rz"] = pchip_loop([(1, 0), (10, 0), (28, -8), (36, -8), (56, 8), (64, 8), (78, 0), (E, 0)], F)
    A["Neck.rz"] = A["Head.rz"] * 0.35
    A["Body.rz"] = pchip_loop([(1, 0), (30, -1.0), (58, 1.0), (80, 0), (E, 0)], F)
    A["Body.tx"] = np.zeros_like(F)
    A["Torso.ty"] = 0.5 - 0.5 * np.cos(2 * np.pi * (F - 1) / 60.0)
    A["Torso.ry"] = 0.3 * np.roll(A["FaceAngle.ry"], 4)
    A["Look.tx"] = pchip_loop([(1, 0), (8, 0), (20, 0.85), (36, 0.6), (48, -0.85), (64, -0.6), (78, 0), (E, 0)], F)
    A["Look.ty"] = pchip_loop([(1, 0), (74, 0), (86, 0.45), (96, -0.3), (108, 0), (E, 0)], F)
    A["Eye_R.ty"] = pchip_loop(blink_keys([4, 66, 108], end=E), F)
    A["Eye_L.ty"] = A["Eye_R.ty"].copy()
    A["Eye_R.rz"] = np.zeros_like(F)
    A["Eye_L.rz"] = np.zeros_like(F)
    A["Brow_R.ty"] = pchip_loop([(1, 0), (20, 0.8), (40, 0.1), (86, 0.9), (100, -0.4), (112, 0), (E, 0)], F)
    A["Brow_L.ty"] = pchip_loop([(1, 0), (20, 0.8), (40, 0.1), (86, 0.9), (100, -0.4), (112, 0), (E, 0)], F)
    A["Brow_R.rz"] = pchip_loop([(1, 0), (44, 0), (52, -10), (62, 0), (E, 0)], F)
    A["Brow_L.rz"] = pchip_loop([(1, 0), (44, 0), (52, -10), (62, 0), (E, 0)], F)
    pad = VISEME_PAD
    talk = [(1, "rest"), (14, "rest"), (18, "A"), (24, "rest"), (40, "rest"), (44, "O"), (50, "rest"),
            (78, "rest"), (81, "A"), (85, "I"), (89, "U"), (93, "E"), (97, "O"), (102, "rest"), (E, "rest")]
    A["Mouth.rx"] = pchip_loop([(f, pad[v][0]) for f, v in talk], F)
    A["Mouth.ry"] = pchip_loop([(f, pad[v][1]) for f, v in talk], F)
    A["Mouth.rz"] = pchip_loop([(1, 6), (104, 6), (112, 18), (E, 6)], F)
    return F, A


def hair_sim(F, A, chains):
    """Secondary motion for every lock: a damped spring per joint chasing a
    target that follows the head (stiffer at the root, looser at the tip),
    kicked by the head's sideways acceleration, with a little breeze.
    Run over three loops so the last one starts where it ends."""
    head_rot = A["Body.rz"] + A["Neck.rz"] + A["Head.rz"]
    turn = A["FaceAngle.ry"]
    n = len(F) - 1
    rot3 = np.concatenate([head_rot[:-1]] * 3)
    turn3 = np.concatenate([turn[:-1]] * 3)
    lat = np.gradient(np.gradient(turn3 * 0.16 + rot3 * 0.12))
    out = {}
    dt = 1.0
    for ci, (name, pts, mesh, parts, ts) in enumerate(chains):
        njo = len(pts)
        if name.startswith("Bang"):
            stiff = [0.97, 0.9, 0.82][:njo]
            freq, zeta, kick = 0.19, 0.30, 5.0
        elif name.startswith("SideIn"):
            stiff = [0.9, 0.7]
            freq, zeta, kick = 0.16, 0.25, 7.0
        elif name.startswith("Side"):
            stiff = [0.85, 0.62, 0.45, 0.3]
            freq, zeta, kick = 0.12, 0.22, 9.0
        elif name == "Ahoge":
            stiff = [1.0, 1.0, 1.0]
            freq, zeta, kick = 0.22, 0.12, -14.0
        else:
            stiff = [0.8, 0.55, 0.38, 0.25]
            freq, zeta, kick = 0.09, 0.2, 11.0
        w = 2 * np.pi * freq
        beta = np.zeros((len(rot3), njo))
        vel = np.zeros(njo)
        b = np.zeros(njo)
        phase = ci * 1.7
        for f in range(len(rot3)):
            breeze = 1.2 * np.sin(2 * np.pi * (f / 90.0) + phase) * np.linspace(0.3, 1.0, njo)
            if name == "Ahoge":
                breeze *= 2.0
            target = rot3[f] * np.array(stiff) + (-kick * lat[f] * np.linspace(0.5, 1.4, njo)) + breeze
            # rotational inertia of the head drags the ahoge the other way
            acc = w * w * (target - b) - 2 * zeta * w * vel
            vel += acc * dt
            b += vel * dt
            beta[f] = b
        last = beta[2 * n:3 * n]
        # close the loop exactly: fade out the tiny residual over the loop
        resid = beta[3 * n - 1] - beta[2 * n - 1]
        ramp = np.linspace(0, 1, n)[:, None]
        last = last - resid * ramp
        last = np.vstack([last, last[:1]])
        local = np.zeros_like(last)
        local[:, 0] = last[:, 0] - head_rot
        for j in range(1, njo):
            local[:, j] = last[:, j] - last[:, j - 1]
        out[name] = local
    return out


def write_anim(path, rig_layer_name, builder, script=None, length=LOOP, doc=""):
    if os.path.exists(path):
        os.remove(path)
    layer = Sdf.Layer.CreateNew(path)
    layer.subLayerPaths = ["./" + rig_layer_name]
    layer.startTimeCode = 1
    layer.endTimeCode = length
    layer.timeCodesPerSecond = FPS
    layer.framesPerSecond = FPS
    layer.defaultPrim = "Bust"
    layer.documentation = doc
    stage = Usd.Stage.Open(layer)
    F, A = (script or performance)()
    H = builder.head_ctl
    B = builder.body_ctl
    ctl = {
        "FaceAngle": H + "/FaceAngle_ctl", "Look": H + "/Look_ctl",
        "Eye_R": H + "/Eye_R_ctl", "Eye_L": H + "/Eye_L_ctl",
        "Brow_R": H + "/Brow_R_ctl", "Brow_L": H + "/Brow_L_ctl",
        "Mouth": H + "/Mouth_ctl", "Torso": B + "/Torso_ctl",
        "Head": H, "Neck": B + "/Neck_ctl", "Body": B,
    }
    frames = F[:-1]

    def key(prim_path, channel, values):
        prim = stage.OverridePrim(prim_path)
        a = prim.CreateAttribute("avars:" + channel, Sdf.ValueTypeNames.Double, False)
        for f, v in zip(F, values):
            a.Set(float(round(v, 4)), Usd.TimeCode(float(f)))

    for k, values in A.items():
        c, ch = k.split(".")
        key(ctl[c], ch, values)
    hair = hair_sim(F, A, builder.chains)
    for name, local in hair.items():
        for j, cpath in enumerate(builder.chain_controls[name]):
            key(cpath, "rz", local[:, j])
    layer.Save()
    return A, hair


def main():
    rig_path = os.path.join(HERE, "bust_rig.usda")
    anim_path = os.path.join(HERE, "bust_anim.usda")
    b = Builder(rig_path)
    b.meshes()
    b.cages()
    b.lattice_wire("HeadLatticeWire", (-14.0, -16.0), (14.0, 16.0), 12, 13, 0.3, (0.18, 0.80, 1.0))
    b.lattice_wire("BodyLatticeWire", (-18.0, -36.0), (18.0, -17.0), 13, 7, -0.9, (0.55, 0.45, 1.0))
    b.skeleton()
    b.all_channels()
    b.wiring()
    b.movers()
    b.save()
    for k, (mean, mx) in sorted(b.fit_report.items()):
        print("  head cage fit %-14s mean %.3f  max %.3f" % (k, mean, mx))
    write_anim(anim_path, os.path.basename(rig_path), b, performance, LOOP, doc=(
        "A 180-frame looping performance for bust_rig.usda: head turns with "
        "parallax, tilts, blinks, talks through the vowel pad, laughs, "
        "breathes, and the hair follows through. Only control avars are "
        "authored here. Generated by build_bust.py."))
    sweep_path = os.path.join(HERE, "bust_sweep.usda")
    write_anim(sweep_path, os.path.basename(rig_path), b, sweep, SWEEP, doc=(
        "A 120-frame parameter sweep for bust_rig.usda, layered mesh style: angle "
        "X through +-30, angle Y through +-15, the tilt, a blink and the five "
        "vowels. Only control avars are authored here. Generated by "
        "build_bust.py."))
    npts = sum(len(v) for k, v in b.mesh_rest.items() if not k.endswith("Wire"))
    print("wrote %s (%d art points in %d meshes)" % (rig_path, npts, len(b.parts)))
    print("wrote %s" % anim_path)
    print("wrote %s" % sweep_path)


if __name__ == "__main__":
    main()
