#!/usr/bin/env python
"""Generate Pip, a 1930s rubber-hose cartoon character rigged with RigExec.

    source bin/_env.sh && "$PY" examples/2d/rubberhose/build_rubberhose.py

Writes, next to this script:

    rubberhose_rig.usda    the character at rest: flat 2D geometry + the rig
    rubberhose_anim.usda   a 120-frame (24 fps) loopable performance that
                           sublayers the rig and keys only controls/dials
    rubberhose_picker.usda Body and Face picker pages selecting the rig's
                           animator controls, sublayered by the rig

Everything is deterministic: the same script always writes the same files.

The character is flat cut-out geometry in the XY plane (Y up, facing +Z,
draw order by small Z offsets per layer).  Every part carries a dark ink
outline: a ring (or, for the limbs, a wider strip) one line-width outside
the fill, a hair behind it, in the SAME mesh, so whatever deforms the fill
deforms its outline identically.

The rig is ordinary RigExec schema -- nothing here is specific to 2D:

  * Root / COG / Hips / Chest controls, a spline-IK spine with volume
    preservation (squash & stretch), skinned with a skin mover;
  * rubber-hose limbs: a two-bone IK solves an (invisible) elbow/knee, a
    14-joint spline IK lays a hose along the smooth curve shoulder ->
    elbow -> hand, and a curve mover transports the dense limb strip (and
    its outline) through the hose frames -- the limb arcs instead of
    hinging;
  * IK hands and planted IK feet, glove/shoe controls for rotation and
    squash;
  * a head squash/stretch/drag lattice (a 3x3x2 Bernstein cage, itself
    skinned to two controls) that every face part is deformed through;
  * a 2D "face slide" (fake head turn with parallax) as localized
    clusters (matrix movers measured in the head's space);
  * pie-cut pupils aimed at a look-at control by aim constraints;
  * blink and mouth shapes as blend-shape movers with in-betweens, driven
    by dials on a face control;
  * a hat that a parent constraint hands to the glove and back (a
    space switch keyed through the constraint's envelope).
"""
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, Vt

HERE = os.path.dirname(os.path.abspath(__file__))
RIG_FILE = os.path.join(HERE, "rubberhose_rig.usda")
ANIM_FILE = os.path.join(HERE, "rubberhose_anim.usda")
PICKER_FILE = os.path.join(HERE, "rubberhose_picker.usda")

ROOT = "/Pip"
RIG = ROOT + "/Rig"
CTL = RIG + "/Controls"
JNT = RIG + "/Joints"
SOLV = RIG + "/Solvers"
MOV = RIG + "/Movers"
GEOM = ROOT + "/Geom"
TGT = ROOT + "/Targets"
PCK = RIG + "/Pip"

# Look

INK = (0.105, 0.078, 0.070)
FACE = (0.992, 0.925, 0.835)
CHEEK = (0.965, 0.600, 0.530)
JACKET = (0.835, 0.200, 0.165)
PANTS = (0.130, 0.110, 0.120)
LIMB = (0.130, 0.110, 0.120)
GOLD = (0.975, 0.745, 0.235)
GOLD_DARK = (0.800, 0.520, 0.120)
GLOVE = (0.995, 0.985, 0.955)
SHOE = (0.560, 0.270, 0.130)
SOLE = (0.290, 0.140, 0.080)
SHINE = (0.985, 0.900, 0.780)
EYE_WHITE = (1.0, 1.0, 1.0)
MOUTH_IN = (0.340, 0.055, 0.070)
TONGUE = (0.925, 0.370, 0.360)
HAT = JACKET
HAT_TOP = (0.905, 0.300, 0.250)
SHADOW = (0.835, 0.715, 0.545)

OL = 0.075          # ink line width, world units (~7 px at 1080p full body)
DZ = 0.008          # outline sits this far behind its fill

# guide colours
G_CENTER = (1.0, 0.82, 0.18)
G_LEFT = (0.25, 0.62, 1.0)
G_RIGHT = (1.0, 0.33, 0.33)
G_FACE = (1.0, 0.45, 0.85)
G_LOOK = (1.0, 0.55, 0.12)

# Proportions (world units; feet on y = 0, ~10 units tall)

Z_SHADOW = -0.50
Z_LEG = 0.00
Z_SHOE = 0.10
Z_BODY = 0.20
Z_BUTTON = 0.23
Z_HEAD = 0.30
Z_CHEEK = 0.305
Z_MOUTH = 0.32
Z_EYE = 0.34
PUPIL_DEPTH = 0.42                  # pupils ride a sphere this far in front
Z_PUPIL = Z_EYE + PUPIL_DEPTH       # 0.76
Z_LID = 0.80
Z_NOSE = 0.84
Z_BROW = 0.86
Z_HAT = 0.95
Z_ARM = {"L": 1.10, "R": 1.06}
Z_EPAULET = 1.16
Z_GLOVE = {"L": 1.24, "R": 1.20}

HEAD_C = (0.0, 7.35)
HEAD_RX, HEAD_RY = 1.92, 1.80
NECK = (0.0, 5.72)

BODY_C = (0.0, 4.28)
BODY_B = 1.44            # half height of the egg
BODY_A = 1.14            # half width at the equator
BODY_K = 0.20            # egg taper (narrower at the top)
BODY_SPLIT = 3.62        # pants below, jacket above
SPINE_Y0, SPINE_Y1 = 2.95, 5.62
SPINE_N = 6

SIDES = ("L", "R")
SGN = {"L": 1.0, "R": -1.0}          # character left is +X (faces +Z)

SHOULDER = {s: (0.90 * SGN[s], 5.02) for s in SIDES}
HAND = {s: (1.92 * SGN[s], 2.98) for s in SIDES}
ARM_POLE = {s: (3.40 * SGN[s], 5.10) for s in SIDES}
ARM_W = 0.25
ARM_EXTRA = 0.14         # per-bone IK length offset: a relaxed bow at rest

HIP = {s: (0.50 * SGN[s], 3.30) for s in SIDES}
ANKLE = {s: (1.00 * SGN[s], 0.74) for s in SIDES}
KNEE_POLE = {s: (2.90 * SGN[s], 2.30) for s in SIDES}
LEG_W = 0.27
LEG_EXTRA = 0.06
SHOE_PIVOT = {s: (1.00 * SGN[s], 0.05) for s in SIDES}

HOSE_N = 14              # joints per hose limb
LIMB_SEG = 48            # strip segments along a limb

EYE_C = {s: (0.37 * SGN[s], 7.80) for s in SIDES}
EYE_HW, EYE_HH = 0.355, 0.62
PUPIL_C = {s: (0.37 * SGN[s], 7.70) for s in SIDES}
PUPIL_HW, PUPIL_HH = 0.165, 0.30
NOSE_C = (0.0, 7.07)
MOUTH_C = (0.0, 6.30)
BROW_C = {s: (0.42 * SGN[s], 8.66) for s in SIDES}
CHEEK_C = {s: (1.06 * SGN[s], 6.60) for s in SIDES}

HAT_BASE = (0.56, 8.98)
HAT_TILT = -18.0

LOOK_Z = 4.4

# 2D geometry helpers


def v2(x, y):
    return (float(x), float(y))


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def mul(a, s):
    return (a[0] * s, a[1] * s)


def length(a):
    return math.hypot(a[0], a[1])


def norm(a):
    l = length(a)
    return (a[0] / l, a[1] / l) if l > 1e-12 else (0.0, 0.0)


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def rot2(p, deg, about=(0.0, 0.0)):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    x, y = p[0] - about[0], p[1] - about[1]
    return (about[0] + c * x - s * y, about[1] + s * x + c * y)


def signed_area(poly):
    a = 0.0
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        a += x0 * y1 - x1 * y0
    return 0.5 * a


def ccw(poly):
    return list(poly) if signed_area(poly) >= 0 else list(reversed(poly))


def ellipse(cx, cy, hw, hh, n=64, a0=0.0, a1=None, rot=0.0):
    """Closed polygon (a1 None) or open arc a0..a1 (radians)."""
    if a1 is None:
        angs = [a0 + 2 * math.pi * i / n for i in range(n)]
    else:
        angs = [a0 + (a1 - a0) * i / (n - 1) for i in range(n)]
    pts = [(hw * math.cos(a), hh * math.sin(a)) for a in angs]
    return [add(rot2(p, rot), (cx, cy)) for p in pts]


def capsule(p0, p1, r, n=10):
    """Stadium around segment p0-p1 (CCW)."""
    d = norm(sub(p1, p0))
    ang = math.atan2(d[1], d[0])
    pts = []
    for i in range(n + 1):          # cap around p1
        a = ang - math.pi / 2 + math.pi * i / n
        pts.append(add(p1, (r * math.cos(a), r * math.sin(a))))
    for i in range(n + 1):          # cap around p0
        a = ang + math.pi / 2 + math.pi * i / n
        pts.append(add(p0, (r * math.cos(a), r * math.sin(a))))
    return ccw(pts)


def inflate(poly, d, miter=2.5):
    """Offset a CCW polygon outward by d (miter joints, clamped)."""
    poly = ccw(poly)
    n = len(poly)
    out = []
    for i in range(n):
        p = poly[i]
        a = poly[i - 1]
        b = poly[(i + 1) % n]
        e1 = norm(sub(p, a))
        e2 = norm(sub(b, p))
        n1 = (e1[1], -e1[0])
        n2 = (e2[1], -e2[0])
        m = norm(add(n1, n2))
        if m == (0.0, 0.0):
            m = n1
        c = max(m[0] * n1[0] + m[1] * n1[1], 1.0 / miter)
        out.append(add(p, mul(m, d / c)))
    return out


def hull2(points):
    """Convex hull of 2D points, counter-clockwise (monotone chain)."""
    pts = sorted(set((round(p[0], 9), round(p[1], 9)) for p in points))
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and (
                    (out[-1][0] - out[-2][0]) * (p[1] - out[-2][1]) -
                    (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out
    lo, hi = half(pts), half(list(reversed(pts)))
    return lo[:-1] + hi[:-1]


def centroid(poly):
    a = signed_area(poly)
    if abs(a) < 1e-12:
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        return (sum(xs) / len(xs), sum(ys) / len(ys))
    cx = cy = 0.0
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        cr = x0 * y1 - x1 * y0
        cx += (x0 + x1) * cr
        cy += (y0 + y1) * cr
    return (cx / (6 * a), cy / (6 * a))


class MeshBuilder(object):
    """Flat faces with a per-face colour, optional per-point st."""

    def __init__(self):
        self.pts = []
        self.st = []
        self.counts = []
        self.idx = []
        self.cols = []

    def point(self, p, z, st=(0.0, 0.0)):
        self.pts.append((float(p[0]), float(p[1]), float(z)))
        self.st.append((float(st[0]), float(st[1])))
        return len(self.pts) - 1

    def face(self, ids, color):
        pts = [self.pts[i] for i in ids]
        if signed_area([(p[0], p[1]) for p in pts]) < 0:
            ids = list(reversed(ids))
        self.counts.append(len(ids))
        self.idx.extend(ids)
        self.cols.append(tuple(color))

    def fan(self, poly, z, color, center=None, st=None):
        poly = ccw(poly)
        c = center if center is not None else centroid(poly)
        uv = st if st is not None else (0.0, 0.0)
        ci = self.point(c, z, uv)
        ids = [self.point(p, z, uv) for p in poly]
        n = len(ids)
        for i in range(n):
            self.face([ci, ids[i], ids[(i + 1) % n]], color)
        return ids

    def ring(self, poly, z, color, width=OL, inset=0.012):
        """Outline ring: from slightly inside poly to width outside it."""
        poly = ccw(poly)
        inner = inflate(poly, -inset)
        outer = inflate(poly, width)
        a = [self.point(p, z) for p in inner]
        b = [self.point(p, z) for p in outer]
        n = len(a)
        for i in range(n):
            j = (i + 1) % n
            self.face([a[i], b[i], b[j], a[j]], color)

    def shape(self, poly, z, color, outline=True, center=None, width=OL):
        """Filled polygon with its ink ring a hair behind."""
        if outline:
            self.ring(poly, z - DZ, INK, width)
        self.fan(poly, z, color, center)

    def strip(self, row_a, row_b, z, color, st_a=None, st_b=None):
        a = [self.point(p, z, st_a[i] if st_a else (0, 0))
             for i, p in enumerate(row_a)]
        b = [self.point(p, z, st_b[i] if st_b else (0, 0))
             for i, p in enumerate(row_b)]
        for i in range(len(a) - 1):
            self.face([a[i], a[i + 1], b[i + 1], b[i]], color)
        return a, b

    def extend(self, other):
        base = len(self.pts)
        self.pts.extend(other.pts)
        self.st.extend(other.st)
        self.counts.extend(other.counts)
        self.idx.extend(i + base for i in other.idx)
        self.cols.extend(other.cols)


# USD authoring helpers

T = Sdf.ValueTypeNames


def mat(x_axis=(1, 0, 0), y_axis=(0, 1, 0), z_axis=(0, 0, 1), t=(0, 0, 0)):
    return Gf.Matrix4d(x_axis[0], x_axis[1], x_axis[2], 0,
                       y_axis[0], y_axis[1], y_axis[2], 0,
                       z_axis[0], z_axis[1], z_axis[2], 0,
                       t[0], t[1], t[2], 1)


def mat_rz(deg, t=(0, 0, 0)):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return mat((c, s, 0), (-s, c, 0), (0, 0, 1), t)


def mat_t(x, y, z=0.0):
    return mat(t=(x, y, z))


def mat_along(p0, p1, z):
    """Frame at p0 with +X toward p1, +Z out of the picture."""
    d = norm(sub(p1, p0))
    return mat((d[0], d[1], 0), (-d[1], d[0], 0), (0, 0, 1), (p0[0], p0[1], z))


class Author(object):
    def __init__(self, stage):
        self.stage = stage

    def prim(self, path, type_name, api=()):
        # grouping ancestors are Scopes, never typeless defs
        parent = Sdf.Path(path).GetParentPath()
        missing = []
        while parent != Sdf.Path.absoluteRootPath and                 not self.stage.GetPrimAtPath(parent):
            missing.append(parent)
            parent = parent.GetParentPath()
        for m in reversed(missing):
            self.stage.DefinePrim(m, "Scope")
        p = self.stage.DefinePrim(path, type_name)
        if api:
            op = Sdf.TokenListOp()
            op.prependedItems = list(api)
            p.SetMetadata("apiSchemas", op)
        return p

    @staticmethod
    def attr(prim, name, tname, value=None, uniform=False, custom=False):
        a = prim.CreateAttribute(
            name, tname, custom,
            Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying)
        if value is not None:
            a.Set(value)
        return a

    @staticmethod
    def rel(prim, name, targets):
        if isinstance(targets, str):
            targets = [targets]
        r = prim.CreateRelationship(name, False)
        r.SetTargets([Sdf.Path(t) for t in targets])
        return r

    def scope(self, path):
        return self.stage.DefinePrim(path, "Scope")


# The character


class Pip(object):
    def __init__(self, stage):
        self.stage = stage
        self.a = Author(stage)
        self.ctl_world = {}       # control path -> rest world matrix
        self.ctl_parent = {}      # control path -> parent control path/None
        self.ctl_local = {}       # control path -> rest:space (local)
        self.jnt_world = {}
        self.mesh_pts = {}        # mesh path -> rest points (list of 3-tuples)

    # -- controls & joints -------------------------------------------------

    def control(self, path, world, parent=None, shape="sphere",
                color=G_CENTER, size=(0.4, 0.4, 0.01), width=0.05,
                draw="wire", offset=None, hidden=False, doc=None):
        prim = self.a.prim(path, "RigExecControl", ["RigExecControlAPI"])
        local = world * self.ctl_world[parent].GetInverse() if parent else world
        self.a.attr(prim, "rest:space", T.Matrix4d, local)
        # guide purpose: the handles draw with the rig guides, never over the
        # clean 2D picture
        self.a.attr(prim, "purpose", T.Token, "guide", uniform=True)
        self.a.attr(prim, "guide:shape", T.Token, shape, uniform=True)
        self.a.attr(prim, "guide:drawMode", T.Token, draw, uniform=True)
        self.a.attr(prim, "guide:displayColor", T.Color3f, Gf.Vec3f(*color))
        sx, sy, sz = (0.0, 0.0, 0.0) if hidden else size
        self.a.attr(prim, "guide:scaleX", T.Double, sx)
        self.a.attr(prim, "guide:scaleY", T.Double, sy)
        self.a.attr(prim, "guide:scaleZ", T.Double, sz)
        if not hidden:
            self.a.attr(prim, "guide:wireWidth", T.Double, width)
        if offset is not None:
            self.a.attr(prim, "guide:offset", T.Double3, Gf.Vec3d(*offset))
        if doc:
            prim.SetMetadata("documentation", doc)
        self.ctl_world[path] = world
        self.ctl_parent[path] = parent
        self.ctl_local[path] = local
        return prim

    def joint(self, path, world, parent=None, radius=0.06,
              color=(0.35, 0.85, 0.75), opacity=0.9):
        prim = self.a.prim(path, "RigExecJoint")
        local = world * self.jnt_world[parent].GetInverse() if parent else world
        self.a.attr(prim, "rest:space", T.Matrix4d, local)
        self.a.attr(prim, "guide:radius", T.Double, radius)
        self.a.attr(prim, "guide:displayColor", T.Color3f, Gf.Vec3f(*color))
        self.a.attr(prim, "guide:displayOpacity", T.Float, opacity)
        self.jnt_world[path] = world
        return prim

    def chain(self, base, names, p0, p1, z, radius, color):
        """Straight nested joint chain p0 -> p1 (+X along the chain)."""
        paths = []
        parent = None
        n = len(names)
        frame = mat_along(p0, p1, z)
        for i, name in enumerate(names):
            p = lerp(p0, p1, i / float(n - 1))
            world = Gf.Matrix4d(frame)
            world.SetRow(3, Gf.Vec4d(p[0], p[1], z, 1))
            path = (parent + "/" + name) if parent else (base + "/" + name)
            self.joint(path, world, parent, radius, color)
            paths.append(path)
            parent = path
        return paths

    # -- meshes -------------------------------------------------------------

    def mesh(self, name, mb, st=False, doc=None):
        path = GEOM + "/" + name
        m = UsdGeom.Mesh.Define(self.stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in mb.pts]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(mb.counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(mb.idx))
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateDoubleSidedAttr(True)
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
            "displayColor", T.Color3fArray, UsdGeom.Tokens.uniform)
        pv.Set(Vt.Vec3fArray([Gf.Vec3f(*c) for c in mb.cols]))
        if st:
            stv = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
                "st", T.TexCoord2fArray, UsdGeom.Tokens.vertex)
            stv.Set(Vt.Vec2fArray([Gf.Vec2f(*u) for u in mb.st]))
        xs = [p[0] for p in mb.pts]
        ys = [p[1] for p in mb.pts]
        zs = [p[2] for p in mb.pts]
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(min(xs), min(ys), min(zs)),
                                          Gf.Vec3f(max(xs), max(ys), max(zs))]))
        if doc:
            m.GetPrim().SetMetadata("documentation", doc)
        self.mesh_pts[path] = list(mb.pts)
        return path

    def targets(self, name, pts):
        path = TGT + "/" + name
        p = UsdGeom.Points.Define(self.stage, path)
        p.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*q) for q in pts]))
        p.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        return path

    # -- movers -------------------------------------------------------------

    def mover(self, path, type_name, moves):
        prim = self.a.prim(path, type_name, ["RigExecMoverAPI"])
        self.a.rel(prim, "rigExec:moves", moves)
        return prim

    def matrix_mover(self, path, moves, transform, space=None, weight=None,
                     phase="final"):
        prim = self.mover(path, "RigExecMatrixMover", moves)
        transform_rel = self.a.rel(prim, "rigExec:transform", transform)
        if phase != "base":
            transform_rel.SetMetadata("rigExecReadPhase", phase)
        if space:
            self.a.rel(prim, "rigExec:transformSpace", space)
        if weight is not None:
            self.a.attr(prim, "inputs:defaultWeight", T.Float, float(weight))
        return prim

    def skin_mover(self, path, moves, influences, indices, weights, size):
        prim = self.mover(path, "RigExecSkinMover", moves)
        self.a.rel(prim, "rigExec:influences", influences).SetMetadata(
            "rigExecReadPhase", "final")
        self.a.attr(prim, "rigExec:jointIndices", T.IntArray,
                    Vt.IntArray(indices))
        self.a.attr(prim, "rigExec:jointWeights", T.FloatArray,
                    Vt.FloatArray(weights))
        self.a.attr(prim, "rigExec:elementSize", T.Int, size, uniform=True)
        return prim

    # Build

    def build(self):
        st = self.stage
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(st, 0.1)
        st.SetStartTimeCode(1)
        st.SetEndTimeCode(1)
        st.SetTimeCodesPerSecond(24)
        st.SetFramesPerSecond(24)
        root = UsdGeom.Xform.Define(st, ROOT).GetPrim()
        st.SetDefaultPrim(root)
        root.SetMetadata("kind", "component")
        root.SetMetadata(
            "documentation",
            "Pip -- a 1930s rubber-hose cartoon character, flat 2D cut-out "
            "geometry rigged entirely with RigExec. Generated by "
            "build_rubberhose.py; edit that, not this file.")

        rig = self.a.prim(RIG, "RigExecRoot")
        self.a.attr(rig, "rigExec:partition", T.Token, "Pip", uniform=True)
        for s in (CTL, JNT, MOV, SOLV, GEOM, TGT):
            self.a.scope(s)
        for s in ("/Geometry", "/Cage", "/Pose"):
            self.a.scope(MOV + s)
        self.stage.GetPrimAtPath(MOV).SetMetadata(
            "documentation",
            "Bottom sibling runs first: Pose constraints, then the head "
            "cage, then the geometry movers that read both at `final`.")

        self.build_controls()
        self.build_face_dials()
        self.build_body()
        for s in SIDES:
            self.build_arm(s)
            self.build_leg(s)
        self.build_head()
        self.build_hat()
        self.build_shadow()
        self.build_camera()

        # composed order = evaluation order, read bottom-up: Solvers are
        # defined last so they run first; the movers revise after them.
        rig.SetMetadata("documentation",
                        "Pip's rig. Solvers sit at the bottom (they run "
                        "first), movers revise and deform after them.")

    # -- controls ------------------------------------------------------------

    def build_controls(self):
        c = self.control
        c(CTL + "/Root_ctl", mat_t(0, 0, 0), None, "sphere", G_CENTER,
          (2.9, 0.34, 0.01), 0.03,
          doc="World placement. Everything, including the planted feet, "
              "rides it.")
        root = CTL + "/Root_ctl"
        c(root + "/COG_ctl", mat_t(0, 3.75, Z_BODY), root, "sphere", G_CENTER,
          (1.75, 1.75, 0.01), 0.035,
          doc="Centre of gravity: the body bob. Hands, poles and the "
              "look-at ride it; the feet do not.")
        cog = root + "/COG_ctl"
        c(cog + "/Hips_ctl", mat_t(0, SPINE_Y0 + 0.25, Z_BODY), cog, "cube",
          G_CENTER, (1.25, 0.28, 0.01), 0.04,
          doc="Spline-IK spine root (hip swivel).")
        hips = cog + "/Hips_ctl"
        c(cog + "/Chest_ctl", mat_t(0, SPINE_Y1 - 0.25, Z_BODY), cog,
          "sphere", G_CENTER, (0.95, 0.30, 0.01), 0.04,
          doc="Spline-IK spine end. Move it toward the hips to squash the "
              "body (volume preserved), away to stretch, sideways to lean.")
        chest = cog + "/Chest_ctl"
        for s in SIDES:
            col = G_LEFT if s == "L" else G_RIGHT
            c(hips + "/Hip_%s" % s, mat_t(HIP[s][0], HIP[s][1], Z_LEG), hips,
              "sphere", col, (0.12, 0.12, 0.01), 0.03, hidden=True)
            c(chest + "/Shoulder_%s_ctl" % s,
              mat_t(SHOULDER[s][0], SHOULDER[s][1], Z_ARM[s]), chest,
              "diamond", col, (0.22, 0.22, 0.01), 0.03,
              doc="Arm root: shrug.")
            c(cog + "/Hand_%s_ctl" % s, mat_t(HAND[s][0], HAND[s][1], Z_ARM[s]),
              cog, "cube", col, (0.36, 0.36, 0.01), 0.035,
              doc="IK hand: the hose arm follows it; translate only.")
            c(cog + "/Hand_%s_ctl/Glove_%s_ctl" % (s, s),
              mat_rz(12.0 * SGN[s], (HAND[s][0], HAND[s][1], Z_ARM[s])),
              cog + "/Hand_%s_ctl" % s, "sphere", col, (0.55, 0.55, 0.01),
              0.03, offset=(0.0, -0.55, 0.0),
              doc="Glove rotation and squash, pivoting at the wrist.")
            c(cog + "/ArmPole_%s_ctl" % s,
              mat_t(ARM_POLE[s][0], ARM_POLE[s][1], Z_ARM[s]), cog, "diamond",
              col, (0.16, 0.16, 0.01), 0.03,
              doc="Which way the hose arm bows.")
            c(root + "/Foot_%s_ctl" % s, mat_t(ANKLE[s][0], ANKLE[s][1], Z_LEG),
              root, "cube", col, (0.34, 0.2, 0.01), 0.035,
              doc="Planted IK foot (ankle); translate only.")
            c(root + "/Foot_%s_ctl/Shoe_%s_ctl" % (s, s),
              mat_t(SHOE_PIVOT[s][0], SHOE_PIVOT[s][1], Z_SHOE),
              root + "/Foot_%s_ctl" % s, "sphere", col, (0.95, 0.12, 0.01),
              0.03, offset=(0.28 * SGN[s], 0.0, 0.0),
              doc="Shoe tilt and squash, pivoting on the sole.")
            c(root + "/KneePole_%s_ctl" % s,
              mat_t(KNEE_POLE[s][0], KNEE_POLE[s][1], Z_LEG), root, "diamond",
              col, (0.16, 0.16, 0.01), 0.03,
              doc="Which way the hose leg bows.")
            # hose tangent helpers: hidden, at the limb's root and end,
            # turned with the root->end chord so the hose leaves the
            # shoulder and reaches the hand along the chord (not along the
            # rest pose's direction) whatever way the limb points
            c(chest + "/Shoulder_%s_ctl/ArmRoot_%s" % (s, s),
              mat_t(SHOULDER[s][0], SHOULDER[s][1], Z_ARM[s]),
              chest + "/Shoulder_%s_ctl" % s, hidden=True,
              doc="Arm IK / hose root: turns with the arm's chord.")
            c(cog + "/Hand_%s_ctl/ArmEnd_%s" % (s, s),
              mat_t(HAND[s][0], HAND[s][1], Z_ARM[s]),
              cog + "/Hand_%s_ctl" % s, hidden=True,
              doc="Arm IK / hose end: turns with the arm's chord.")
            c(root + "/Foot_%s_ctl/LegEnd_%s" % (s, s),
              mat_t(ANKLE[s][0], ANKLE[s][1], Z_LEG),
              root + "/Foot_%s_ctl" % s, hidden=True,
              doc="Leg IK / hose end: turns with the leg's chord.")
        # spine mid: a follow control (constrained halfway between hips and
        # chest) with the animator's belly/bend control nested inside it
        mid_y = 0.5 * (SPINE_Y0 + SPINE_Y1)
        c(cog + "/SpineMidFollow", mat_t(0, mid_y, Z_BODY), cog, hidden=True)
        c(cog + "/SpineMidFollow/SpineMid_ctl", mat_t(0, mid_y, Z_BODY),
          cog + "/SpineMidFollow", "sphere", G_CENTER, (0.3, 0.3, 0.01), 0.03,
          doc="Bends the belly without moving the hips or chest.")
        # head
        head = chest + "/Head_ctl"
        c(head, mat_t(NECK[0], NECK[1], Z_HEAD), chest, "sphere", G_CENTER,
          (0.62, 0.20, 0.01), 0.04,
          doc="Neck pivot: turn and tilt the whole head.")
        c(head + "/HeadSquash_ctl", mat_t(NECK[0], NECK[1], Z_HEAD), head,
          "cube", G_FACE, (2.15, 1.98, 0.01), 0.02, offset=(0, 1.62, 0),
          doc="Head squash & stretch (scale), pivoting at the neck; drives "
              "the head lattice cage.")
        c(head + "/HeadSquash_ctl/HeadTop_ctl",
          mat_t(0, HEAD_C[1] + HEAD_RY + 0.05, Z_HEAD),
          head + "/HeadSquash_ctl", "sphere", G_FACE, (0.24, 0.24, 0.01),
          0.03,
          doc="Drags the top of the head (lattice top row): overlap and "
              "smear.")
        hat_world = mat_rz(HAT_TILT, (HAT_BASE[0], HAT_BASE[1], Z_HAT))
        c(head + "/HeadSquash_ctl/HeadTop_ctl/Hat_ctl", hat_world,
          head + "/HeadSquash_ctl/HeadTop_ctl", "sphere", G_CENTER,
          (0.75, 0.75, 0.01), 0.03, offset=(0.0, 0.34, 0.0),
          doc="The hat. Rides the head top; a parent constraint hands it "
              "to the left glove for the hat tip.")
        face = head + "/FaceSlide_ctl"
        c(face, mat_t(HEAD_C[0], HEAD_C[1], Z_HEAD), head, "sphere", G_FACE,
          (0.20, 0.20, 0.01), 0.03, offset=(0, -1.25, 0.0),
          doc="2D head turn: slides the features across the head with "
              "parallax (nose most, cheeks least).")
        for s in SIDES:
            c(face + "/EyeAim_%s" % s,
              mat_t(EYE_C[s][0], EYE_C[s][1] - 0.06, Z_EYE), face, hidden=True)
            c(face + "/Brow_%s_ctl" % s,
              mat_t(BROW_C[s][0], BROW_C[s][1], Z_BROW), face, "sphere",
              G_FACE, (0.34, 0.12, 0.01), 0.025, offset=(0, 0.22, 0),
              doc="Brow: raise, drop, tilt.")
        c(face + "/Mouth_ctl", mat_t(MOUTH_C[0], MOUTH_C[1], Z_MOUTH), face,
          "sphere", G_FACE, (0.85, 0.34, 0.01), 0.025,
          doc="Moves, tilts and scales the whole mouth.")
        c(cog + "/LookAt_ctl", mat_t(0.0, EYE_C["L"][1], LOOK_Z), cog, "sphere",
          G_LOOK, (0.30, 0.30, 0.3), 0.04,
          doc="Where the pie-cut pupils look (aim constraints).")

    def build_face_dials(self):
        """The Face_ctl carries the blend-shape dials as custom avars."""
        root = CTL + "/Root_ctl/COG_ctl/Chest_ctl/Head_ctl/FaceSlide_ctl"
        path = root + "/Face_ctl"
        prim = self.control(
            path, mat_t(HEAD_C[0] + HEAD_RX + 0.55, HEAD_C[1] + 0.6, Z_HEAD),
            root, "cube", G_FACE, (0.16, 0.16, 0.01), 0.03,
            doc="Face dials: blink, mouth shapes (0..1).")
        for name, doc in (
                ("face:blinkL", "Left eye lids: 0 open, 0.5 half, 1 shut."),
                ("face:blinkR", "Right eye lids: 0 open, 0.5 half, 1 shut."),
                ("face:mouthOpen", "Open grin; 0.5 hits the in-between."),
                ("face:mouthOo", "Round 'ooh'."),
                ("face:mouthWide", "Wide smile.")):
            a = self.a.attr(prim, name, T.Float, 0.0, custom=True)
            a.SetDocumentation(doc)
        self.face_ctl = path

    # -- body --------------------------------------------------------------

    def body_outline(self, n=72):
        """Egg: wide at the bottom, narrow at the top."""
        pts = []
        for i in range(n):
            th = 2 * math.pi * i / n
            s = math.sin(th)
            a = BODY_A * (1.0 - BODY_K * s)
            pts.append((BODY_C[0] + a * math.cos(th), BODY_C[1] + BODY_B * s))
        return pts

    def body_halfwidth(self, y):
        s = max(-1.0, min(1.0, (y - BODY_C[1]) / BODY_B))
        a = BODY_A * (1.0 - BODY_K * s)
        return a * math.sqrt(max(0.0, 1.0 - s * s))

    def spine_weights(self, pts, joints):
        """Two influences per point: linear between neighbouring joints."""
        ys = [SPINE_Y0 + (SPINE_Y1 - SPINE_Y0) * i / (SPINE_N - 1)
              for i in range(SPINE_N)]
        idx, wts = [], []
        for p in pts:
            y = p[1]
            f = (y - ys[0]) / (ys[1] - ys[0])
            f = max(0.0, min(SPINE_N - 1.0, f))
            k = min(int(math.floor(f)), SPINE_N - 2)
            t = f - k
            idx += [k, k + 1]
            wts += [1.0 - t, t]
        return idx, wts

    def build_body(self):
        mb = MeshBuilder()
        # outline: ring around the silhouette (dense) behind the fill
        sil = self.body_outline(96)
        mb.ring(sil, Z_BODY - DZ, INK)
        # fill: a grid of rows so the skin bends it smoothly
        rows = []
        n_rows = 26
        for j in range(n_rows + 1):
            th = -math.pi / 2 + math.pi * j / n_rows
            rows.append(BODY_C[1] + BODY_B * math.sin(th))
        rows.append(BODY_SPLIT)
        rows = sorted(set(round(r, 6) for r in rows))
        ncol = 14
        grid = []
        for y in rows:
            hw = self.body_halfwidth(y)
            grid.append([mb.point((-hw + 2 * hw * i / ncol, y), Z_BODY)
                         for i in range(ncol + 1)])
        for j in range(len(rows) - 1):
            col = PANTS if rows[j] < BODY_SPLIT - 1e-6 else JACKET
            for i in range(ncol):
                mb.face([grid[j][i], grid[j][i + 1], grid[j + 1][i + 1],
                         grid[j + 1][i]], col)
        # belt line and buttons
        belt_y = BODY_SPLIT
        hw = self.body_halfwidth(belt_y)
        xs = [-hw + 0.02 + (2 * hw - 0.04) * i / 16.0 for i in range(17)]
        mb.strip([(x, belt_y - 0.035) for x in xs],
                 [(x, belt_y + 0.035) for x in xs], Z_BUTTON - 0.01, INK)
        for bx, by in ((-0.34, 4.92), (0.34, 4.92), (-0.36, 4.30),
                       (0.36, 4.30)):
            mb.shape(ellipse(bx, by, 0.105, 0.105, 24), Z_BUTTON, GOLD,
                     width=0.045)
            mb.fan(ellipse(bx - 0.03, by + 0.03, 0.035, 0.035, 12),
                   Z_BUTTON + 0.004, (1.0, 0.93, 0.70))
        # bowtie under the chin
        for g in (-1.0, 1.0):
            lobe = [(0.0, 5.39), (0.34 * g, 5.53), (0.40 * g, 5.39),
                    (0.34 * g, 5.24)]
            lobe = [add(p, (0, 0)) for p in lobe]
            mb.shape(lobe, Z_BUTTON + 0.01, INK, width=0.05)
            mb.fan([(0.09 * g, 5.42), (0.27 * g, 5.48), (0.29 * g, 5.44)],
                   Z_BUTTON + 0.013, (0.38, 0.33, 0.34))
        mb.shape(ellipse(0.0, 5.39, 0.085, 0.10, 20), Z_BUTTON + 0.016, INK,
                 width=0.04)
        path = self.mesh("Body", mb, doc="Jacket, pants, belt and buttons; "
                         "skinned to the spline-IK spine.")

        jn = ["Spine%d" % i for i in range(SPINE_N)]
        joints = self.chain(JNT, jn, (0, SPINE_Y0), (0, SPINE_Y1), Z_BODY,
                            0.07, (1.0, 0.85, 0.35))
        self.spine_joints = joints
        cog = CTL + "/Root_ctl/COG_ctl"
        sp = self.a.prim(SOLV + "/SpineIK", "RigExecSplineIk")
        self.a.rel(sp, "rigExec:rootControl", cog + "/Hips_ctl")
        self.a.rel(sp, "rigExec:midControl", cog + "/SpineMidFollow/SpineMid_ctl")
        self.a.rel(sp, "rigExec:endControl", cog + "/Chest_ctl")
        self.a.rel(sp, "rigExec:joints", joints)
        self.a.attr(sp, "rigExec:volumeWeights", T.FloatArray,
                    Vt.FloatArray([0.35, 0.8, 1.0, 1.0, 0.8, 0.35]),
                    uniform=True)
        self.a.attr(sp, "rigExec:restLength", T.Token, "curve", uniform=True)
        self.a.attr(sp, "inputs:preserveVolume", T.Double, 1.0)
        self.a.attr(sp, "inputs:midFollowWeight", T.Double, 0.5)
        self.a.attr(sp, "guide:radius", T.Double, 0.0)
        # the follow: halfway between hips and chest, offsets maintained
        mid_y = 0.5 * (SPINE_Y0 + SPINE_Y1)
        pc = self.mover(MOV + "/Pose/SpineMidFollow", "RigExecParentConstraint",
                        cog + "/SpineMidFollow")
        self.a.rel(pc, "rigExec:sources", [cog + "/Hips_ctl", cog + "/Chest_ctl"])
        self.a.attr(pc, "inputs:sourceWeights", T.FloatArray,
                    Vt.FloatArray([0.5, 0.5]))
        self.a.attr(pc, "inputs:translationOffsets", T.Double3Array,
                    Vt.Vec3dArray([Gf.Vec3d(0, mid_y - (SPINE_Y0 + 0.25), 0),
                                   Gf.Vec3d(0, mid_y - (SPINE_Y1 - 0.25), 0)]))
        self.a.attr(pc, "inputs:rotationOffsets", T.Double3Array,
                    Vt.Vec3dArray([Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 0, 0)]))
        idx, wts = self.spine_weights(self.mesh_pts[path], joints)
        self.skin_mover(MOV + "/Geometry/Body/Skin", path + ".points", joints,
                        idx, wts, 2)

    # -- limbs -------------------------------------------------------------

    def limb_strip(self, p0, p1, width, z, n_seg=LIMB_SEG, cap_n=12):
        """Dense strip + wider outline strip, both with st.u along the limb
        (0 at p0, 1 at p1) and round caps riding the end frames."""
        mb = MeshBuilder()
        d = norm(sub(p1, p0))
        L = length(sub(p1, p0))
        nrm = (-d[1], d[0])
        for hw, zz, col in ((width / 2 + OL * 0.85, z - DZ, INK),
                            (width / 2, z, LIMB)):
            left, right, st = [], [], []
            for i in range(n_seg + 1):
                s = L * i / n_seg
                c = add(p0, mul(d, s))
                left.append(add(c, mul(nrm, hw)))
                right.append(add(c, mul(nrm, -hw)))
                st.append((i / float(n_seg), 0.0))
            mb.strip(left, right, zz, col, st, st)
            for end, u, sgn in ((p1, 1.0, 1.0), (p0, 0.0, -1.0)):
                cap = []
                base = math.atan2(d[1] * sgn, d[0] * sgn)
                for k in range(cap_n + 1):
                    a = base - math.pi / 2 + math.pi * k / cap_n
                    cap.append(add(end, (hw * math.cos(a), hw * math.sin(a))))
                ci = mb.point(end, zz, (u, 0.0))
                ids = [mb.point(q, zz, (u, 0.0)) for q in cap]
                for k in range(cap_n):
                    mb.face([ci, ids[k], ids[k + 1]], col)
        return mb

    def hose(self, side, limb, p_root, p_end, width, z, root_ctl, end_ctl,
             pole_ctl, extra, color, jcolor):
        """Two-bone IK elbow + spline-IK hose + curve mover strip."""
        tag = "%s_%s" % (limb, side)
        mb = self.limb_strip(p_root, p_end, width, z)
        mesh = self.mesh(tag, mb, st=True,
                         doc="Rubber-hose %s: a dense strip carried along "
                             "the hose frames by a curve mover." % limb.lower())
        # Solver reads are POSITIONAL in the pose stack (bottom sibling
        # first): the hose reads the IK elbow, so the hose solver is defined
        # ABOVE the two-bone IK and therefore runs after it.
        sp = self.a.prim(SOLV + "/%sHose_%s" % (limb, side), "RigExecSplineIk")
        # two-bone IK (the 'skeleton' under the hose)
        ik = self.chain(JNT, ["%sIk_%s_%s" % (limb, side, n)
                              for n in ("Root", "Mid", "End")],
                        p_root, p_end, z, 0.10, jcolor)
        tb = self.a.prim(SOLV + "/%sIK_%s" % (limb, side), "RigExecTwoBoneIk")
        self.a.rel(tb, "rigExec:rootControl", root_ctl)
        self.a.rel(tb, "rigExec:effectorControl", end_ctl)
        self.a.rel(tb, "rigExec:poleControl", pole_ctl)
        self.a.rel(tb, "rigExec:joints", ik)
        self.a.attr(tb, "rigExec:upperLengthOffset", T.Double, extra)
        self.a.attr(tb, "rigExec:lowerLengthOffset", T.Double, extra)
        self.a.attr(tb, "inputs:stretch", T.Float, 1.0)
        self.a.attr(tb, "inputs:softness", T.Float, 0.0)
        self.a.attr(tb, "guide:radius", T.Double, 0.0)
        # the hose itself
        names = ["%sHose_%s_%02d" % (limb, side, i) for i in range(HOSE_N)]
        hose = self.chain(JNT, names, p_root, p_end, z, 0.045, color)
        self.a.rel(sp, "rigExec:rootControl", root_ctl)
        self.a.rel(sp, "rigExec:midControl", ik[1])
        self.a.rel(sp, "rigExec:endControl", end_ctl)
        self.a.rel(sp, "rigExec:joints", hose)
        self.a.attr(sp, "rigExec:restLength", T.Token, "curve", uniform=True)
        self.a.attr(sp, "inputs:preserveVolume", T.Double, 0.0)
        self.a.attr(sp, "inputs:midFollowWeight", T.Double, 0.5)
        self.a.attr(sp, "guide:radius", T.Double, 0.0)
        cm = self.mover(MOV + "/Geometry/%s/Hose" % tag, "RigExecCurveMover",
                        mesh + ".points")
        self.a.rel(cm, "rigExec:driverFrames", sp.GetPath().pathString)
        self.a.rel(cm, "rigExec:bindCoordinates", mesh + ".primvars:st")
        self.a.attr(cm, "rigExec:mode", T.Token, "ribbon", uniform=True)
        return mesh

    def glove_shape(self, side):
        """Four-finger cartoon glove in wrist space, fingers along -Y."""
        s = SGN[side]
        mb = MeshBuilder()
        parts = []
        # cuff: flared bell with a rolled rim
        cuff = [(-0.23, -0.20), (0.23, -0.20), (0.34, 0.10), (0.30, 0.16),
                (-0.30, 0.16), (-0.34, 0.10)]
        palm = ellipse(0.0, -0.50, 0.36, 0.34, 40)
        fingers = [capsule((0.17 * k, -0.62), (0.25 * k, -1.08 - (0.06 if k == 0
                                                                   else 0.0)),
                           0.108, 10) for k in (-1, 0, 1)]
        thumb = capsule((-0.24 * s, -0.42), (-0.55 * s, -0.56), 0.10, 10)
        parts = [cuff, palm, thumb] + fingers
        for p in parts:
            mb.ring(p, -DZ, INK)
        for p in parts:
            mb.fan(p, 0.0, GLOVE)
        # rim line across the cuff and three stitches on the back
        mb.strip([(-0.31, 0.05), (0.31, 0.05)], [(-0.30, 0.10), (0.30, 0.10)],
                 0.004, INK)
        for k in (-1, 0, 1):
            x = 0.13 * k
            mb.strip([(x - 0.022, -0.28), (x - 0.02 + 0.03 * k, -0.62)],
                     [(x + 0.022, -0.28), (x + 0.02 + 0.03 * k, -0.62)],
                     0.004, INK)
        # finger separation lines (the fingers overlap, the lines show it)
        for k in (-1, 1):
            a = (0.085 * k, -0.74)
            b = (0.125 * k, -0.98)
            mb.strip([add(a, (-0.018, 0)), add(b, (-0.018, 0))],
                     [add(a, (0.018, 0)), add(b, (0.018, 0))], 0.004, INK)
        return mb

    def placed(self, mb, frame2d_deg, origin, z):
        """Rotate/translate a local-space builder into world (in place)."""
        pts = []
        for x, y, zz in mb.pts:
            p = add(rot2((x, y), frame2d_deg), origin)
            pts.append((p[0], p[1], zz + z))
        mb.pts = pts
        return mb

    def build_arm(self, s):
        cog = CTL + "/Root_ctl/COG_ctl"
        col = G_LEFT if s == "L" else G_RIGHT
        self.hose(s, "Arm", SHOULDER[s], HAND[s], ARM_W, Z_ARM[s],
                  cog + "/Chest_ctl/Shoulder_%s_ctl/ArmRoot_%s" % (s, s),
                  cog + "/Hand_%s_ctl/ArmEnd_%s" % (s, s),
                  cog + "/ArmPole_%s_ctl" % s,
                  ARM_EXTRA, (0.55, 0.85, 1.0) if s == "L" else
                  (1.0, 0.62, 0.55), col)
        glove = self.placed(self.glove_shape(s), 12.0 * SGN[s], HAND[s],
                            Z_GLOVE[s])
        gpath = self.mesh("Glove_%s" % s, glove, doc="White four-finger glove.")
        self.matrix_mover(MOV + "/Geometry/Glove_%s/Follow" % s,
                          gpath + ".points",
                          cog + "/Hand_%s_ctl/Glove_%s_ctl" % (s, s))
        # epaulette: hides the hose root, rides the shoulder
        mb = MeshBuilder()
        sh = SHOULDER[s]
        ep = [add(sh, rot2(p, -8 * SGN[s])) for p in
              ellipse(0.03 * SGN[s], 0.10, 0.30, 0.17, 40)]
        mb.shape(ep, Z_EPAULET, GOLD)
        for k in range(4):
            x = sh[0] + SGN[s] * (-0.18 + 0.12 * k)
            y0 = sh[1] - 0.02 - 0.012 * k * SGN[s] * SGN[s]
            mb.shape(capsule((x, y0), (x + 0.01 * SGN[s], y0 - 0.20), 0.038, 6),
                     Z_EPAULET - 0.02, GOLD_DARK, width=0.035)
        epath = self.mesh("Epaulet_%s" % s, mb,
                          doc="Bellhop epaulette over the hose root.")
        self.matrix_mover(MOV + "/Geometry/Epaulet_%s/Follow" % s,
                          epath + ".points",
                          cog + "/Chest_ctl/Shoulder_%s_ctl" % s)

    def shoe_shape(self, side):
        """Big cartoon shoe in sole-pivot space: a bulbous toe cap pointing
        outward, a rounded heel under the ankle, a dark rounded sole."""
        s = SGN[side]
        mb = MeshBuilder()
        toe = [(x, max(y, 0.09)) for x, y in
               ellipse(0.44 * s, 0.37, 0.64, 0.36, 72)]
        heel = [(x, max(y, 0.09)) for x, y in
                ellipse(-0.16 * s, 0.31, 0.36, 0.29, 48)]
        sole = capsule((-0.42 * s, 0.075), (0.98 * s, 0.075), 0.085, 10)
        for p in (sole, heel, toe):
            mb.ring(p, -DZ, INK)
        mb.fan(sole, 0.0, SOLE)
        mb.fan(heel, 0.002, SHOE)
        mb.fan(toe, 0.003, SHOE)
        # the seam where the toe cap meets the heel, and a shine
        seam = [(0.02 * s + 0.07 * math.sin(math.pi * t) * s, 0.14 + 0.52 * t)
                for t in [i / 8.0 for i in range(9)]]
        mb.strip([(x - 0.018, y) for x, y in seam],
                 [(x + 0.018, y) for x, y in seam], 0.004, INK)
        mb.fan(ellipse(0.64 * s, 0.55, 0.16, 0.075, 20, rot=-22 * s), 0.005,
               SHINE)
        return mb

    def build_leg(self, s):
        root = CTL + "/Root_ctl"
        col = G_LEFT if s == "L" else G_RIGHT
        self.hose(s, "Leg", HIP[s], ANKLE[s], LEG_W, Z_LEG,
                  root + "/COG_ctl/Hips_ctl/Hip_%s" % s,
                  root + "/Foot_%s_ctl/LegEnd_%s" % (s, s),
                  root + "/KneePole_%s_ctl" % s,
                  LEG_EXTRA, (0.55, 0.85, 1.0) if s == "L" else
                  (1.0, 0.62, 0.55), col)
        shoe = self.placed(self.shoe_shape(s), 0.0, SHOE_PIVOT[s], Z_SHOE)
        path = self.mesh("Shoe_%s" % s, shoe, doc="Oversized shoe.")
        self.matrix_mover(MOV + "/Geometry/Shoe_%s/Follow" % s,
                          path + ".points",
                          root + "/Foot_%s_ctl/Shoe_%s_ctl" % (s, s))

    # -- head ----------------------------------------------------------------

    # The head is deformed LAST by a lattice whose cage rides the head
    # controls, so each face part first gets its local motion in rest space
    # (blend shapes, aim, brow/mouth clusters, face slide) and then the
    # lattice carries it with the head's squash, drag and world motion.

    def head_paths(self):
        head = CTL + "/Root_ctl/COG_ctl/Chest_ctl/Head_ctl"
        return {
            "head": head,
            "squash": head + "/HeadSquash_ctl",
            "top": head + "/HeadSquash_ctl/HeadTop_ctl",
            "hat": head + "/HeadSquash_ctl/HeadTop_ctl/Hat_ctl",
            "face": head + "/FaceSlide_ctl",
        }

    def face_part(self, name, mb, slide, local=None, shapes=None, doc=None):
        """Mesh + its mover stack: shapes -> local -> slide -> lattice."""
        hp = self.head_paths()
        path = self.mesh(name, mb, doc=doc)
        base = MOV + "/Geometry/" + name
        self.a.scope(base)
        # defined top-down = runs bottom-up
        lat = self.mover(base + "/HeadLattice", "RigExecLatticeMover",
                         path + ".points")
        self.a.rel(lat, "rigExec:cage", self.cage).SetMetadata(
            "rigExecReadPhase", "final")
        self.a.attr(lat, "rigExec:basis", T.Token, "bernstein", uniform=True)
        self.a.attr(lat, "rigExec:divisions", T.Int3, Gf.Vec3i(3, 3, 2))
        if slide:
            self.matrix_mover(base + "/FaceSlide", path + ".points",
                              hp["face"], space=hp["head"], weight=slide)
        if local:
            for i, (transform, space) in enumerate(local):
                self.matrix_mover(base + "/Local%d" % i if i else base + "/Local",
                                  path + ".points", transform, space=space)
        if shapes:
            prim = self.mover(base + "/Shapes", "RigExecBlendShapeMover",
                              path + ".points")
            self.a.rel(prim, "rigExec:blendInputs", shapes)
        return path

    def blend_input(self, name, dial, samples):
        """samples: [(activation, target points path)]"""
        path = RIG + "/BlendInputs/" + name
        prim = self.a.prim(path, "RigExecBlendInput")
        w = self.a.attr(prim, "inputs:weight", T.Float, 0.0)
        w.SetConnections([Sdf.Path(self.face_ctl + "." + dial)])
        spaths = []
        for i, (act, tgt) in enumerate(samples):
            sp = path + "/" + ("Full" if act >= 1.0 else "Half%d" % i)
            sprim = self.a.prim(sp, "RigExecBlendSample")
            self.a.attr(sprim, "rigExec:activation", T.Float, float(act))
            self.a.rel(sprim, "rigExec:targetPoints", tgt + ".points")
            spaths.append(sp)
        self.a.rel(prim, "rigExec:samples", spaths)
        return path


    LID_COLS = 19

    def lid_rows(self, s, amount_up, amount_low, band_scale):
        """Upper lid rows R0 (fixed top), R1 (fill edge), R2 (ink edge);
        lower lid rows Q0 (fixed bottom), Q1 (edge).  amount 0 = open,
        1 = shut on the closed line."""
        cx, cy = EYE_C[s]
        a, b = EYE_HW - 0.004, EYE_HH - 0.004
        rows = {"R0": [], "R1": [], "R2": [], "Q0": [], "Q1": []}
        n = self.LID_COLS
        for k in range(n):
            u = -math.cos(math.pi * k / (n - 1))
            x = cx + a * u
            h = b * math.sqrt(max(0.0, 1.0 - u * u))
            top, bot = cy + h, cy - h
            closed = cy - 0.12 * (1.0 - u * u)
            bt = 0.05 * math.sqrt(max(0.0, 1.0 - u * u)) * band_scale
            r1 = top + (closed + bt - top) * amount_up
            r2 = top + (closed - bt - top) * amount_up
            q1 = bot + (closed - bt - bot) * amount_low
            clamp = lambda y: max(bot, min(top, y))  # noqa: E731
            rows["R0"].append((x, top))
            rows["R1"].append((x, clamp(r1)))
            rows["R2"].append((x, clamp(min(r2, r1))))
            rows["Q0"].append((x, bot))
            rows["Q1"].append((x, clamp(min(q1, r2))))
        return rows

    def lids_mesh(self, pose):
        """pose: {'L': (up, low, band), 'R': (...)} -> MeshBuilder."""
        mb = MeshBuilder()
        for s in SIDES:
            up, low, band = pose[s]
            r = self.lid_rows(s, up, low, band)
            mb.strip(r["R0"], r["R1"], Z_LID, FACE)
            mb.strip(r["R1"], r["R2"], Z_LID + 0.003, INK)
            mb.strip(r["Q0"], r["Q1"], Z_LID, FACE)
        return mb


    MOUTH_COLS = 25

    def mouth_rows(self, shape):
        n = self.MOUTH_COLS
        cx, cy = MOUTH_C
        up, low = [], []
        for k in range(n):
            u = -1.0 + 2.0 * k / (n - 1)
            e = max(0.0, 1.0 - u * u)
            if shape == "rest":
                W = 0.60
                c = 0.20 * u * u - 0.03
                yu, yl = c + 0.013, c - 0.013
            elif shape == "openHalf":
                W = 0.62
                yu = 0.17 * u * u - 0.02
                yl = yu - 0.26 * e ** 0.8 - 0.013
            elif shape == "open":
                W = 0.66
                yu = 0.13 * u * u + 0.01
                yl = yu - 0.54 * e ** 0.72 - 0.013
            elif shape == "oo":
                W = 0.24
                r = math.sqrt(e)
                yu, yl = 0.25 * r - 0.14, -0.25 * r - 0.14
            elif shape == "wide":
                W = 0.80
                c = 0.30 * u * u - 0.05
                yu, yl = c + 0.013, c - 0.013 - 0.09 * e
            else:
                raise ValueError(shape)
            up.append((cx + W * u, cy + yu))
            low.append((cx + W * u, cy + yl))
        return up, low

    def mouth_mesh(self, shape):
        up, low = self.mouth_rows(shape)
        n = len(up)
        tongue = []
        for k in range(n):
            u = -1.0 + 2.0 * k / (n - 1)
            f = 0.46 * max(0.0, 1.0 - u * u)
            tongue.append(lerp(low[k], up[k], f))
        mb = MeshBuilder()
        poly = list(up) + list(reversed(low))
        # outline ring around the whole contour (same topology every shape)
        outer = inflate(poly, OL * 1.05)
        inner = inflate(poly, -0.004)
        a = [mb.point(p, Z_MOUTH - DZ) for p in inner]
        b = [mb.point(p, Z_MOUTH - DZ) for p in outer]
        m = len(a)
        for i in range(m):
            j = (i + 1) % m
            mb.face([a[i], b[i], b[j], a[j]], INK)
        mb.strip(low, tongue, Z_MOUTH, TONGUE)
        mb.strip(tongue, up, Z_MOUTH, MOUTH_IN)
        return mb


    def pupil_mesh(self, s):
        mb = MeshBuilder()
        cx, cy = PUPIL_C[s]
        z = Z_PUPIL
        w0, w1 = math.radians(22), math.radians(66)   # the pie cut
        n = 40
        apex = mb.point((cx, cy), z)
        arc = []
        for i in range(n + 1):
            ang = w1 + (2 * math.pi - (w1 - w0)) * i / n
            arc.append(mb.point((cx + PUPIL_HW * math.cos(ang),
                                 cy + PUPIL_HH * math.sin(ang)), z))
        for i in range(n):
            mb.face([apex, arc[i], arc[i + 1]], INK)
        return mb

    def brow_mesh(self, s):
        """A thick arched brush stroke, tapering at both ends."""
        mb = MeshBuilder()
        cx, cy = BROW_C[s]
        n = 18
        outer, inner = [], []
        for i in range(n + 1):
            v = -1.0 + 2.0 * i / n
            x = cx + 0.27 * v
            y = cy + 0.09 * (1.0 - v * v) - 0.02 * v * SGN[s]
            dx, dy = 0.27, -0.18 * v - 0.02 * SGN[s]
            nrm = norm((-dy, dx))
            th = 0.028 + 0.052 * (1.0 - v * v) ** 0.6
            outer.append(add((x, y), mul(nrm, th)))
            inner.append(add((x, y), mul(nrm, -th)))
        mb.strip(inner, outer, Z_BROW, INK)
        return mb

    def build_head(self):
        hp = self.head_paths()
        # the lattice cage: 3x3x2 Bernstein, x fastest, skinned to the
        # squash control (lower rows) and the head-top drag (upper rows)
        cage_path = RIG + "/Cages/HeadCage"
        self.a.scope(RIG + "/Cages")
        cage = UsdGeom.Points.Define(self.stage, cage_path)
        xs = (-2.35, 0.0, 2.35)
        ys = (5.15, HEAD_C[1], 9.55)
        zs = (0.0, 1.2)
        pts, idx, wts = [], [], []
        row_w = ((1.0, 0.0), (0.6, 0.4), (0.0, 1.0))
        for z in zs:
            for j, y in enumerate(ys):
                for x in xs:
                    pts.append(Gf.Vec3f(x, y, z))
                    idx += [0, 1]
                    wts += list(row_w[j])
        cage.CreatePointsAttr(Vt.Vec3fArray(pts))
        cage.CreateWidthsAttr(Vt.FloatArray([0.13] * len(pts)))
        UsdGeom.PrimvarsAPI(cage).CreatePrimvar(
            "displayColor", T.Color3fArray, UsdGeom.Tokens.constant).Set(
                Vt.Vec3fArray([Gf.Vec3f(*G_FACE)]))
        cage.CreatePurposeAttr(UsdGeom.Tokens.guide)
        cage.GetPrim().SetMetadata(
            "documentation",
            "Head lattice cage (3x3x2, x fastest). Default-time points are "
            "the bind cage; a skin mover poses it from HeadSquash_ctl and "
            "HeadTop_ctl and every face part is deformed through it.")
        self.cage = cage_path
        self.skin_mover(MOV + "/Cage/HeadCageSkin", cage_path + ".points",
                        [hp["squash"], hp["top"]], idx, wts, 2)

        # head disc
        mb = MeshBuilder()
        mb.shape(ellipse(HEAD_C[0], HEAD_C[1], HEAD_RX, HEAD_RY, 128), Z_HEAD,
                 FACE)
        self.face_part("Head", mb, None,
                       doc="Head disc; deformed by the head lattice.")

        mb = MeshBuilder()
        for s in SIDES:
            mb.fan(ellipse(CHEEK_C[s][0], CHEEK_C[s][1], 0.27, 0.17, 32),
                   Z_CHEEK, CHEEK)
        self.face_part("Cheeks", mb, 0.55, doc="Rosy cheeks.")

        mb = MeshBuilder()
        for s in SIDES:
            mb.shape(ellipse(EYE_C[s][0], EYE_C[s][1], EYE_HW, EYE_HH, 64),
                     Z_EYE, EYE_WHITE, width=OL * 0.9)
        self.face_part("Eyes", mb, 0.8, doc="Eye whites.")

        for s in SIDES:
            self.face_part(
                "Pupil_%s" % s, self.pupil_mesh(s), 0.8,
                local=[(hp["face"] + "/EyeAim_%s" % s, hp["face"])],
                doc="Pie-cut pupil riding a sphere in front of the eye; its "
                    "EyeAim control is aimed at LookAt_ctl.")
            aim = self.mover(MOV + "/Pose/EyeAim_%s" % s,
                             "RigExecAimConstraint",
                             hp["face"] + "/EyeAim_%s" % s)
            self.a.rel(aim, "rigExec:sources",
                       CTL + "/Root_ctl/COG_ctl/LookAt_ctl")
            self.a.attr(aim, "inputs:aimVector", T.Double3, Gf.Vec3d(0, 0, 1))
            self.a.attr(aim, "inputs:upVector", T.Double3, Gf.Vec3d(0, 1, 0))
            self.a.attr(aim, "inputs:worldUpVector", T.Double3,
                        Gf.Vec3d(0, 1, 0))
            self.a.attr(aim, "rigExec:worldUpType", T.Token, "vector",
                        uniform=True)

        # lids + blink shapes
        rest = self.lids_mesh({"L": (0, 0, 0), "R": (0, 0, 0)})
        tl_half = self.targets("BlinkL_Half", self.lids_mesh(
            {"L": (0.62, 0.30, 0.8), "R": (0, 0, 0)}).pts)
        tl_full = self.targets("BlinkL_Full", self.lids_mesh(
            {"L": (1, 1, 1), "R": (0, 0, 0)}).pts)
        tr_half = self.targets("BlinkR_Half", self.lids_mesh(
            {"R": (0.62, 0.30, 0.8), "L": (0, 0, 0)}).pts)
        tr_full = self.targets("BlinkR_Full", self.lids_mesh(
            {"R": (1, 1, 1), "L": (0, 0, 0)}).pts)
        bl = self.blend_input("BlinkL", "face:blinkL",
                              [(0.5, tl_half), (1.0, tl_full)])
        br = self.blend_input("BlinkR", "face:blinkR",
                              [(0.5, tr_half), (1.0, tr_full)])
        self.face_part("Lids", rest, 0.8, shapes=[bl, br],
                       doc="Upper and lower lids, collapsed on the eye rim "
                           "at rest; the blink shapes close them on a "
                           "curved line (in-between at 0.5).")

        # nose
        mb = MeshBuilder()
        mb.fan(ellipse(NOSE_C[0], NOSE_C[1], 0.27, 0.19, 40), Z_NOSE, INK)
        mb.fan(ellipse(NOSE_C[0] - 0.09, NOSE_C[1] + 0.06, 0.075, 0.045, 16,
                       rot=20), Z_NOSE + 0.003, (1.0, 0.97, 0.92))
        self.face_part("Nose", mb, 1.0, doc="Button nose (slides most).")

        # mouth + shapes
        rest = self.mouth_mesh("rest")
        t_half = self.targets("MouthOpen_Half", self.mouth_mesh("openHalf").pts)
        t_open = self.targets("MouthOpen_Full", self.mouth_mesh("open").pts)
        t_oo = self.targets("MouthOo_Full", self.mouth_mesh("oo").pts)
        t_wide = self.targets("MouthWide_Full", self.mouth_mesh("wide").pts)
        mo = self.blend_input("MouthOpen", "face:mouthOpen",
                              [(0.5, t_half), (1.0, t_open)])
        moo = self.blend_input("MouthOo", "face:mouthOo", [(1.0, t_oo)])
        mw = self.blend_input("MouthWide", "face:mouthWide", [(1.0, t_wide)])
        self.face_part("Mouth", rest, 0.9,
                       local=[(hp["face"] + "/Mouth_ctl", hp["face"])],
                       shapes=[mo, moo, mw],
                       doc="Mouth: closed smile at rest; open (with an "
                           "in-between), 'ooh' and wide shapes.")

        for s in SIDES:
            self.face_part("Brow_%s" % s, self.brow_mesh(s), 0.8,
                           local=[(hp["face"] + "/Brow_%s_ctl" % s, hp["face"])],
                           doc="Brow, moved by its own control.")

    def build_hat(self):
        hp = self.head_paths()
        W, H, HH, DEPTH, BAND = 0.72, 0.68, 0.17, 0.11, 0.22
        n = 32
        mb = MeshBuilder()
        bottom = [(W * (-1 + 2.0 * i / n),
                   -DEPTH * math.sqrt(max(0.0, 1 - (-1 + 2.0 * i / n) ** 2)))
                  for i in range(n + 1)]
        # the side's top edge hides under the top ellipse: keep it straight
        # (a cusp there would spike the mitred outline)
        side = bottom + [(W, H), (0.0, H), (-W, H)]
        band_top = [(x, y + BAND) for x, y in bottom]
        mb.ring(side, 0.000, INK)
        mb.fan(side, 0.004, HAT)
        mb.strip(bottom, band_top, 0.006, GOLD)
        mb.strip([(x, y + BAND - 0.02) for x, y in bottom],
                 [(x, y + BAND + 0.02) for x, y in bottom], 0.007, INK)
        top = ellipse(0.0, H, W, HH, 48)
        mb.ring(top, 0.009, INK, width=OL * 0.8)
        mb.fan(top, 0.012, HAT_TOP)
        knob = ellipse(0.0, H + 0.03, 0.11, 0.06, 20)
        mb.ring(knob, 0.014, INK, width=0.04)
        mb.fan(knob, 0.016, GOLD)
        # a strap line on the side for charm
        mb.strip([(0.40, 0.24), (0.34, 0.58)], [(0.45, 0.24), (0.39, 0.58)],
                 0.008, (0.62, 0.12, 0.10))
        self.placed(mb, HAT_TILT, HAT_BASE, Z_HAT)
        path = self.mesh("Hat", mb, doc="Bellhop pillbox hat.")
        self.matrix_mover(MOV + "/Geometry/Hat/Follow", path + ".points",
                          hp["hat"])
        # the hand-off: a parent constraint onto the left glove, off at
        # rest; a shot keys its envelope (and authors the grab offset)
        pc = self.mover(MOV + "/Pose/HatToGlove", "RigExecParentConstraint",
                        hp["hat"])
        self.a.rel(pc, "rigExec:sources",
                   CTL + "/Root_ctl/COG_ctl/Hand_L_ctl/Glove_L_ctl")
        self.a.attr(pc, "inputs:translationOffsets", T.Double3Array,
                    Vt.Vec3dArray([Gf.Vec3d(0, 0, 0)]))
        self.a.attr(pc, "inputs:rotationOffsets", T.Double3Array,
                    Vt.Vec3dArray([Gf.Vec3d(0, 0, 0)]))
        self.a.attr(pc, "inputs:defaultWeight", T.Float, 0.0)

    def build_shadow(self):
        mb = MeshBuilder()
        mb.fan(ellipse(0.0, 0.02, 2.45, 0.30, 64), Z_SHADOW, SHADOW)
        path = self.mesh("Shadow", mb, doc="Contact shadow on the floor.")
        self.matrix_mover(MOV + "/Geometry/Shadow/Follow", path + ".points",
                          CTL + "/Root_ctl")

    def build_camera(self):
        cam = UsdGeom.Camera.Define(self.stage, ROOT + "/MainCam")
        cam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        h = 12.0
        cam.CreateVerticalApertureAttr(h * 10.0)
        cam.CreateHorizontalApertureAttr(h * 10.0 * 16.0 / 9.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(1.0, 200.0))
        xf = UsdGeom.Xformable(cam)
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 5.0, 50.0))


# The performance: a 120-frame loop at 24 fps (frame 121 == frame 1)
#
#   1-49    four strut beats: the body drops into every contact and floats
#           over the passing position, the weight shifts over the support
#           foot before the other one lifts, hips lead / chest / head /
#           head-top / hat follow a few frames apart, hands swing on arcs
#           about the shoulders and the hoses drag behind them, feet peel,
#           arc up-and-over and slap down
#   49-58   anticipation: a slow sink into a deep squash, arms pulled in,
#           half-shut lids, "ooh", eyes up at the hat
#   58-64   the launch: hips first, then chest, head and hat; the left hand
#           sweeps an arc out and up round the head -- its hose straightens
#           and swings through to arc round the head -- and takes the hat
#   64-86   hat tip: a parent constraint hands the hat to the glove; big
#           grin, raised brows, the right arm arches up in a "ta-da", two
#           toe taps
#   86-97   hat back on (release), the head squashes under it, the left
#           hose swings back through, both arms drop with follow-through
#   97-121  two more strut beats back into frame 1
#
# The limbs are shaped, not left to chance: for every sample the generator
# decides where each hose should peak (a signed bow and a point along the
# chord), then keys the two-bone IK's length offsets so that it solves
# exactly that elbow/knee, and places the pole on that side of the chord.
# The pole therefore never crosses the limb except where the performance
# swings a hose through on purpose (declared in the layer's customLayerData,
# checked by check_limbs.py), and the limb never locks straight.

LOOP = 120
# written samples: every fifth of a frame -- a grid that contains every
# integer frame and two subframe cadences (0.8 and 0.4 frames per
# output frame), so a render never sees an interpolated IK pose
SUB = 5
NS = SUB * LOOP
FRAMES = [round(1.0 + k / float(SUB), 6) for k in range(NS + 1)]

# (contact frame, foot that lifts in this beat, amplitude): the strut beats.
# No two beats alike; the last one before the anticipation (37) is the
# biggest kick, building into the squash.
STRUT = ((1, "L", 0.97), (13, "R", 1.06), (25, "L", 0.92), (37, "R", 1.14),
         (97, "L", 1.03), (109, "R", 0.90))
# per beat: (arm swing amount, the planted-side arm's lag behind the
# lifted-side arm in frames, head tilt amount, hip sway amount)
STRUT_VAR = {1: (1.00, 1.4, 1.00, 1.00), 13: (0.85, 1.0, 0.62, 1.15),
             25: (1.12, 1.8, 1.30, 0.88), 37: (1.17, 1.2, 0.80, 1.12),
             97: (0.90, 1.6, 1.15, 0.94), 109: (1.06, 1.1, 0.72, 1.06)}
# per beat: frames the top of the step (foot peak, toe roll, arm fling)
# comes early (-) or late (+) -- no metronome; the big kick hangs longest
STRUT_DT = {1: 0.0, 13: 0.4, 25: -0.35, 37: 0.6, 97: -0.2, 109: 0.3}
GRAB, LET_GO = 63, 86

# limb rest geometry
LIMB_REST = {}
for _s in SIDES:
    LIMB_REST[("Arm", _s)] = (SHOULDER[_s], HAND[_s], ARM_POLE[_s], ARM_EXTRA)
    LIMB_REST[("Leg", _s)] = (HIP[_s], ANKLE[_s], KNEE_POLE[_s], LEG_EXTRA)


def wrap(f):
    return (f - 1.0) % LOOP + 1.0


def other(side):
    return "R" if side == "L" else "L"


class Spline(object):
    """Cubic Hermite through keys, periodic over the loop.

    A key is (frame, value), (frame, value, slope) or (frame, value,
    in_slope, out_slope); a slope of None is automatic.  Automatic slopes
    are the weighted Catmull-Rom (Bessel) slope, flat on extremes and
    limited so no segment overshoots its keys -- the 'auto' tangents of an
    animation package: motion flows through breakdown keys and eases only
    into real extremes.  An explicit slope breaks the tangent (a foot that
    hits the floor)."""

    def __init__(self, keys, periodic=True):
        ks = {}
        for k in keys:
            t = wrap(float(k[0])) if periodic else float(k[0])
            ks[round(t, 6)] = k
        items = sorted(ks.items())
        self.periodic = periodic
        self.t = [t for t, _ in items]
        self.v = [float(k[1]) for _, k in items]
        din, dout = [], []
        for _, k in items:
            if len(k) == 3:
                din.append(k[2])
                dout.append(k[2])
            elif len(k) >= 4:
                din.append(k[2])
                dout.append(k[3])
            else:
                din.append(None)
                dout.append(None)
        n = len(self.t)
        self.min, self.mout = [], []
        for i in range(n):
            m = self._auto(i)
            self.min.append(m if din[i] is None else float(din[i]))
            self.mout.append(m if dout[i] is None else float(dout[i]))

    def _nb(self, i):
        n = len(self.t)
        if self.periodic:
            if i < 0:
                return self.t[i % n] - LOOP, self.v[i % n]
            if i >= n:
                return self.t[i % n] + LOOP, self.v[i % n]
        i = max(0, min(n - 1, i))
        return self.t[i], self.v[i]

    def _auto(self, i):
        n = len(self.t)
        if n < 2:
            return 0.0
        if not self.periodic and (i == 0 or i == n - 1):
            return 0.0
        t, v = self.t[i], self.v[i]
        tp, vp = self._nb(i - 1)
        tn, vn = self._nb(i + 1)
        h0, h1 = t - tp, tn - t
        if h0 <= 0 or h1 <= 0:
            return 0.0
        d0, d1 = (v - vp) / h0, (vn - v) / h1
        if d0 * d1 <= 0.0:
            return 0.0
        m = (h1 * d0 + h0 * d1) / (h0 + h1)
        lim = 3.0 * min(abs(d0), abs(d1))
        return math.copysign(min(abs(m), lim), m)

    def __call__(self, f):
        t, v = self.t, self.v
        n = len(t)
        if n == 1:
            return v[0]
        if self.periodic:
            f = (f - t[0]) % LOOP + t[0]
            if f >= t[-1]:
                i0, t0, t1 = n - 1, t[-1], t[0] + LOOP
                i1 = 0
            else:
                i0 = self._find(f)
                i1, t0, t1 = i0 + 1, t[i0], t[i0 + 1]
        else:
            if f <= t[0]:
                return v[0]
            if f >= t[-1]:
                return v[-1]
            i0 = self._find(f)
            i1, t0, t1 = i0 + 1, t[i0], t[i0 + 1]
        h = t1 - t0
        s = (f - t0) / h
        h00 = 2 * s ** 3 - 3 * s ** 2 + 1
        h10 = s ** 3 - 2 * s ** 2 + s
        h01 = -2 * s ** 3 + 3 * s ** 2
        h11 = s ** 3 - s ** 2
        return (h00 * v[i0] + h10 * h * self.mout[i0] + h01 * v[i1] +
                h11 * h * self.min[i1])

    def _find(self, f):
        lo, hi = 0, len(self.t) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.t[mid] <= f:
                lo = mid
            else:
                hi = mid
        return lo


class Table(object):
    """A channel computed per written sample, periodic; linear in between.
    Built from a dict over FRAMES or a list of NS + 1 values."""

    def __init__(self, values):
        if isinstance(values, dict):
            values = [values[f] for f in FRAMES]
        self.arr = list(values)

    def __call__(self, f):
        k = (wrap(f) - 1.0) * SUB
        i = int(math.floor(k + 1e-9))
        s = k - i
        a = self.arr[i % NS]
        if s < 1e-9:
            return a
        return a + (self.arr[(i + 1) % NS] - a) * s


def spring(drive, hz, zeta, gain):
    """Periodic steady-state response of a damped spring hung on a moving
    attachment: x'' = -w^2 x - 2 zeta w x' - gain * drive(f), per frame.
    drive is the attachment's acceleration (units / frame^2).  Returns a
    Table over the written samples."""
    w = 2.0 * math.pi * hz / 24.0
    dt = 1.0 / SUB
    x = v = 0.0
    arr = [0.0] * (NS + 1)
    for loop in range(4):
        for k in range(NS):
            v += dt * (-w * w * x - 2.0 * zeta * w * v - gain * drive(FRAMES[k]))
            x += dt * v
            if loop == 3:
                arr[(k + 1) % NS] = x
    arr[NS] = arr[0]
    return Table(arr)


def softplus(x, k):
    if x / k > 40.0:
        return x
    return k * math.log1p(math.exp(x / k))


def soft_clip(x, lim):
    return lim * math.tanh(x / lim) if lim > 1e-9 else 0.0


def dot2(a, b):
    return a[0] * b[0] + a[1] * b[1]


# the relaxed hang of an arm (psi, degrees out from the rest pose): the
# rest pose's glove brushes the bowed-out knee, this one clears the thigh
HANG = 5.0
# The left hose's two swing-throughs (launch, drop): ease-in / ease-out
# widths in frames, and how much the hose lingers straight at the crossing.
SWING_IN = (5.0, 3.2)
SWING_OUT = (4.4, 6.5)
LINGER = 0.7
# the release swing: centred on the dropping hand's top speed, entered only
# after the hand has cleared the head; passes straight quickly (no linger)
RELEASE_C = 89.9
RELEASE_RATE = 0.35
# the hat hold's hose slack (sets how far the curl stands off the head)
HOLD_SLACK = 0.84
# |side| below which the hose's drag fades out (see Performance.limbs)
DRAG_FADE = 0.35
# the legs never lock straight: a floor on the knee bow, reached softly
LEG_HMIN = 0.18
LEG_SOFT = 0.04


def ease_through(x):
    """Odd ease from -1 (x = -1) through 0 to +1 (x = +1), zero slope at
    both ends.  LINGER = 1 is 2.5x^3 - 1.5x^5 (zero speed AND acceleration
    at the crossing: the hose rests straight for a moment); LINGER = 0 is
    1.5x - 0.5x^3 (it passes straight through at full speed)."""
    x = max(-1.0, min(1.0, x))
    return (LINGER * (2.5 * x ** 3 - 1.5 * x ** 5) +
            (1.0 - LINGER) * (1.5 * x - 0.5 * x ** 3))


def cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


class Performance(object):
    def __init__(self, pip):
        self.pip = pip
        c = CTL + "/Root_ctl"
        cog = c + "/COG_ctl"
        chest = cog + "/Chest_ctl"
        head = chest + "/Head_ctl"
        self.P = {
            "Root": c, "COG": cog, "Hips": cog + "/Hips_ctl", "Chest": chest,
            "Head": head, "HeadSquash": head + "/HeadSquash_ctl",
            "HeadTop": head + "/HeadSquash_ctl/HeadTop_ctl",
            "Hat": head + "/HeadSquash_ctl/HeadTop_ctl/Hat_ctl",
            "FaceSlide": head + "/FaceSlide_ctl",
            "Face": head + "/FaceSlide_ctl/Face_ctl",
            "Mouth": head + "/FaceSlide_ctl/Mouth_ctl",
            "LookAt": cog + "/LookAt_ctl",
            "SpineMid": cog + "/SpineMidFollow/SpineMid_ctl",
        }
        for s in SIDES:
            self.P["Hand_" + s] = cog + "/Hand_%s_ctl" % s
            self.P["Glove_" + s] = cog + "/Hand_%s_ctl/Glove_%s_ctl" % (s, s)
            self.P["ArmPole_" + s] = cog + "/ArmPole_%s_ctl" % s
            self.P["Shoulder_" + s] = chest + "/Shoulder_%s_ctl" % s
            self.P["Hip_" + s] = cog + "/Hips_ctl/Hip_%s" % s
            self.P["Foot_" + s] = c + "/Foot_%s_ctl" % s
            self.P["Shoe_" + s] = c + "/Foot_%s_ctl/Shoe_%s_ctl" % (s, s)
            self.P["KneePole_" + s] = c + "/KneePole_%s_ctl" % s
            self.P["ArmRoot_" + s] = chest + "/Shoulder_%s_ctl/ArmRoot_%s" % (
                s, s)
            self.P["ArmEnd_" + s] = cog + "/Hand_%s_ctl/ArmEnd_%s" % (s, s)
            self.P["LegEnd_" + s] = c + "/Foot_%s_ctl/LegEnd_%s" % (s, s)
            self.P["Brow_" + s] = head + "/FaceSlide_ctl/Brow_%s_ctl" % s
        self.ch = {}          # (ctl, avar) -> callable(frame)
        self.keys = {}        # (ctl, avar) -> [keys] before they are built
        self.solver_tables = {}
        self.dials = {}
        self.intent = {}
        self.contacts = {}
        self.debug = {}

    # -- channel plumbing ---------------------------------------------------

    def set(self, ctl, avar, fn):
        self.ch[(self.P.get(ctl, ctl), avar)] = fn

    def get(self, ctl, avar, f):
        fn = self.ch.get((self.P.get(ctl, ctl), avar))
        if fn is None:
            return 1.0 if avar in ("sx", "sy", "sz") else 0.0
        return fn(f)

    def key(self, ctl, avar, *keys):
        self.keys.setdefault((self.P.get(ctl, ctl), avar), []).extend(keys)

    def build_keys(self, rest=None):
        """Turn every collected key list into a periodic spline."""
        for (ctl, avar), ks in sorted(self.keys.items(), key=str):
            self.ch[(ctl, avar)] = Spline(ks)
        self.keys = {}

    # -- FK of the control hierarchy ---------------------------------------

    def avar_matrix(self, ctl, f):
        g = lambda a: self.get(ctl, a, f)  # noqa: E731
        S = Gf.Matrix4d(1).SetScale(Gf.Vec3d(g("sx"), g("sy"), g("sz")))
        R = Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), g("rz")))
        Tm = Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(g("tx"), g("ty"), g("tz")))
        return S * R * Tm

    def world(self, ctl, f):
        ctl = self.P.get(ctl, ctl)
        m = self.avar_matrix(ctl, f) * self.pip.ctl_local[ctl]
        parent = self.pip.ctl_parent[ctl]
        return m * self.world(parent, f) if parent else m

    def parent_world(self, ctl, f):
        ctl = self.P.get(ctl, ctl)
        parent = self.pip.ctl_parent[ctl]
        return self.world(parent, f) if parent else Gf.Matrix4d(1)

    def local_translate_for(self, ctl, world_pos, f):
        """The avars:t that puts `ctl` (no avar rotation/scale) at world_pos."""
        ctl = self.P.get(ctl, ctl)
        want = Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(*world_pos))
        loc = want * self.parent_world(ctl, f).GetInverse() * \
            self.pip.ctl_local[ctl].GetInverse()
        return loc.ExtractTranslation()

    # the body

    def body(self):
        for c, X, a in STRUT:
            g = SGN[X]
            _, _, hk, sk = STRUT_VAR[c]
            self.key("COG", "ty", (c, -0.28), (c + 2, -0.40 * a),
                     (c + 4.5, -0.10 * a), (c + 7.5, 0.10 * a), (c + 10, 0.01))
            self.key("Chest", "ty", (c + 1, -0.10), (c + 3.5, -0.20 * a),
                     (c + 8.5, 0.07 * a))
            # weight over the support foot BEFORE the other one peaks
            self.key("COG", "tx", (c + 4.5, -g * 0.11 * a * sk))
            # hips lead, chest counters 2.5 frames later, head 2 after that;
            # the head carries a slight habitual tilt (asymmetry)
            self.key("Hips", "rz", (c + 5.5, g * 4.5 * a * sk))
            self.key("Chest", "rz", (c + 8, -g * 3.5 * a))
            self.key("Head", "rz", (c + 10, g * 5.0 * a * hk + 1.2))
            self.key("SpineMid", "tx", (c + 7, -g * 0.05 * a * sk))
            self.key("HeadSquash", "sy", (c + 4, 1.0 - 0.06 * a * hk),
                     (c + 8.5, 1.0 + 0.03 * a))
            self.key("Shoulder_" + other(X), "ty", (c + 8, 0.05 * a))
            self.key("Shoulder_" + X, "ty", (c + 8, -0.02 * a))

        # anticipation -> launch -> hat tip -> hat back on -> strut
        self.key("COG", "ty", (49, -0.28), (51, -0.42), (53.5, -0.56),
                 (56, -0.86), (57.8, -0.95), (59, -0.76), (60, -0.26),
                 (61, 0.24), (62, 0.40), (63.5, 0.30), (65.5, 0.34),
                 (68, 0.27), (71.5, 0.21), (73.5, 0.18), (75, 0.22),
                 (77.5, 0.19), (79.5, 0.16), (81, 0.19), (84, 0.12),
                 (85.5, -0.06), (87, -0.20), (89.5, 0.04), (92, -0.05),
                 (94.5, -0.12))
        self.key("Chest", "ty", (50, -0.12), (52, -0.22), (55, -0.36),
                 (58.5, -0.46), (59.5, -0.38), (61, 0.12), (62.5, 0.30),
                 (64, 0.14), (66, 0.05), (69, 0.10), (73.5, 0.07),
                 (77, 0.11), (80.5, 0.075), (83.5, 0.09), (85.5, 0.04),
                 (87, -0.20), (89, 0.06), (91.5, -0.04), (94, -0.03))
        self.key("COG", "tx", (53, 0.03), (57, 0.07), (61, 0.14),
                 (64, 0.18), (70, 0.16), (80, 0.14), (86, 0.07), (91, 0.0),
                 (94, 0.02))
        self.key("Hips", "rz", (51, 1.0), (56, -2.0), (59.5, 0.0), (62, 4.0),
                 (65, 2.5), (72, 3.2), (80, 2.0), (86, 0.5), (90, -1.2),
                 (94, 0.5))
        self.key("Chest", "rz", (52, 1.5), (57, 3.5), (60, 0.5), (63, -6.0),
                 (66, -8.5), (72, -7.5), (80, -6.0), (85, -2.0), (88, 1.5),
                 (92, -0.5))
        self.key("Chest", "tx", (49, 0.0), (58, 0.0), (63, 0.08), (80, 0.10),
                 (88, 0.0), (96, 0.0))
        self.key("Head", "rz", (53, -2.0), (57.5, 4.0), (60.5, -4.0),
                 (64, 5.0), (67, 7.5), (70, 6.0), (74, 7.0), (78, 5.5),
                 (84, 3.0), (86.5, -3.0), (90, 4.0), (93.5, 1.0))
        self.key("SpineMid", "tx", (58, 0.0), (62, -0.10), (70, -0.10),
                 (82, -0.06), (88, 0.02), (92, 0.0))
        self.key("HeadSquash", "sy", (51, 0.97), (54, 0.92), (57, 0.89),
                 (58.4, 0.875), (59.3, 0.92), (60.2, 1.08), (61.3, 1.13),
                 (62.9, 0.95),
                 (64.5, 1.03), (66.5, 0.99), (68, 1.0), (73.5, 0.985),
                 (75, 1.01), (79.6, 0.985), (81.2, 1.01), (84.5, 1.0),
                 (86, 0.87), (87.5, 0.91), (89.2, 1.045), (91.2, 0.985),
                 (93, 1.005))
        for s in SIDES:
            self.key("Shoulder_" + s, "ty", (53, -0.03), (57, -0.07),
                     (60, 0.08 if s == "L" else 0.03),
                     (63, 0.14 if s == "L" else 0.08),
                     (66, 0.10), (82, 0.08), (88, -0.04), (91, 0.02),
                     (94, 0.0))
        self.build_keys()
        # head squash keeps its area: sx follows sy
        sy = self.ch[(self.P["HeadSquash"], "sy")]
        self.set("HeadSquash", "sx", lambda f: sy(f) ** -0.9)

    def secondary(self):
        """Head-top drag and the hat's hop and wobble: damped springs
        driven by the head top's acceleration (overlap for free)."""
        P = self.P
        top_local = self.pip.ctl_local[P["HeadTop"]]

        def top(f):
            m = top_local * self.world(P["HeadSquash"], f)
            p = m.ExtractTranslation()
            return p[0], p[1]

        pts = [top(f) for f in FRAMES]

        def acc(f, axis):
            k = int(round((wrap(f) - 1.0) * SUB)) % NS
            a, b, c = pts[(k - SUB) % NS], pts[k], pts[(k + SUB) % NS]
            return a[axis] - 2 * b[axis] + c[axis]

        tx = spring(lambda f: acc(f, 0), 2.4, 0.5, 0.55)
        ty = spring(lambda f: acc(f, 1), 2.8, 0.55, 0.28)
        self.set("HeadTop", "tx", lambda f: soft_clip(tx(f), 0.24))
        self.set("HeadTop", "ty", lambda f: soft_clip(ty(f), 0.16))
        hat_r = spring(lambda f: acc(f, 0), 2.0, 0.6, 1.0)
        hat_y = spring(lambda f: acc(f, 1), 2.2, 0.55, 1.0)

        def hat_ty(f):
            # a hat can only leave the head: the positive part, softly
            x = hat_y(f) * 0.24
            return soft_clip(0.5 * (x + math.sqrt(x * x + 0.0004)) - 0.01,
                             0.32)
        self.set("Hat", "ty", hat_ty)
        self.set("Hat", "rz", lambda f: soft_clip(-hat_r(f) * 45.0, 13.0))

    # the legs

    def legs(self):
        for c, X, a in STRUT:
            g = SGN[X]
            dt = STRUT_DT[c]
            self.key("Foot_" + X, "ty", (c + 0.5, 0.0, 0.0, None),
                     (c + 2, 0.10 * a), (c + 5.5 + dt, 0.62 * a),
                     (c + 9 + 0.5 * dt, 0.40 * a), (c + 12, 0.0, -0.21, 0.0))
            self.contacts.setdefault("Leg_" + X, []).append(
                float(wrap(c + 12)))
            self.key("Foot_" + X, "tx", (c + 0.5, 0.0, 0.0, None),
                     (c + 6.5 + dt, g * 0.17 * a), (c + 12, 0.0, None, 0.0))
            self.key("Shoe_" + X, "rz", (c + 0.3, 0.0), (c + 1.8, -g * 8.0),
                     (c + 4, g * 8.0), (c + 7 + dt, g * 22.0 * a),
                     (c + 10.5, g * 10.0), (c + 12, 0.0), (c + 13, -g * 3.0),
                     (c + 15, 0.0))
            self.key("Shoe_" + X, "sy", (c + 2, 1.0), (c + 6, 1.05),
                     (c + 11.5, 0.99), (c + 12.8, 0.86), (c + 15, 1.0))
        # the special section: both feet planted; the anticipation squashes
        # the shoes, the launch stretches them, the right one taps twice
        for s in SIDES:
            self.key("Shoe_" + s, "sy", (53.5, 0.97), (56, 0.90), (58, 0.87),
                     (59.4, 0.90), (60.3, 0.99), (61.3, 1.07), (62.8, 0.98),
                     (64, 1.0),
                     (85.5, 1.0), (87, 0.93), (89, 1.0))
        g = SGN["R"]
        self.key("Shoe_R", "rz", (53, 0.0), (69.5, 0.0), (71.5, 18.0 * g),
                 (72.4, 15.0 * g), (73.4, 0.0, -4.0 * g, 0.0),
                 (74.1, -3.0 * g), (75.2, 0.0), (75.8, 0.0),
                 (77.8, 18.0 * g), (78.6, 15.0 * g),
                 (79.6, 0.0, -4.0 * g, 0.0), (80.3, -3.0 * g), (81.5, 0.0),
                 (95, 0.0))
        self.key("Shoe_L", "rz", (53, 0.0), (95, 0.0))
        self.key("Shoe_R", "sy", (73.4, 0.97), (74.2, 0.92), (75.2, 1.0),
                 (79.6, 0.97), (80.4, 0.92), (81.4, 1.0))
        self.build_keys()
        for s in SIDES:
            sy = self.ch[(self.P["Shoe_" + s], "sy")]
            self.set("Shoe_" + s, "sx", lambda f, sy=sy: sy(f) ** -0.8)
        self.floor_clamp()

    def floor_clamp(self, e=0.03):
        """A shoe never sinks into the floor.  The roll keys turn the shoe
        about its ankle pivot (heel strike, toe slap, toe taps) and the
        contact squash widens it; wherever that would push the lowest point
        of the sole (ink included) below the planted sole line, the foot
        rises by exactly that much -- so the shoe rocks on its heel or its
        toe instead of dipping through the ground.  A smooth max (width e)
        keeps the foot's motion C1."""
        for s in SIDES:
            shoe = self.P["Shoe_" + s]
            pts = hull2([(p[0], p[1]) for p in
                         self.pip.mesh_pts[GEOM + "/Shoe_%s" % s]])
            low0 = min(p[1] for p in pts)
            W0i = self.pip.ctl_world[shoe].GetInverse()
            local = [W0i.Transform(Gf.Vec3d(p[0], p[1], Z_SHOE)) for p in pts]
            ty0 = self.ch[(self.P["Foot_" + s], "ty")]
            ty = {}
            for f in FRAMES:
                W = self.world(shoe, f)
                d = low0 - min(W.Transform(p)[1] for p in local)
                if d <= -e:
                    lift = 0.0
                elif d >= e:
                    lift = d
                else:
                    lift = (d + e) ** 2 / (4.0 * e)
                ty[f] = ty0(f) + lift
            self.set("Foot_" + s, "ty", Table(ty))

    # the arms: hands on arcs about the shoulders

    def arm_swing(self):
        """psi: degrees the chord turns from rest, positive up-and-out;
        reach: chord length as a fraction of rest."""
        # The strut: the arm on the lifted knee's side flings OUT and up,
        # its glove above and outside the rising knee; the planted side's
        # arm falls back to hang, its glove just outside the straight
        # support thigh.  No hose ever crosses the black pants and no glove
        # sits on a leg (check_limbs.py's ink tests): each arm swings as a
        # pendulum between the hang and the fling every other beat.  The
        # planted-side arm lags the lifted-side one by a frame or two
        # (overlap), and every beat swings by a different amount.
        for c, X, a in STRUT:
            Y = other(X)
            k, lag, _, _ = STRUT_VAR[c]
            t = c + 6.6 + STRUT_DT[c]
            self.key(("psi", X), "", (t, 30.0 * k))
            self.key(("reach", X), "", (t, 1.03 + 0.03 * (k - 1.0)))
            self.key(("psi", Y), "", (t + lag, HANG + 4.0 * (1.0 - k)))
            self.key(("reach", Y), "", (t + lag, 0.93))
        # left: the crouch flares the hands out and down, clear of the knees
        # and shoes (the elbows bow wide), then the sweep out and up round
        # the head to the hat (63..86 are solved from the hat).  After the
        # hat is set back the hand springs up and OUT, clear of the head,
        # before the arm drops and settles.
        self.key(("psi", "L"), "", (50, 18.0), (53, 20.0), (55.5, 23.0),
                 (57.2, 25.0), (58.3, 21.0), (59.2, 33.0), (60, 62.0),
                 (61.2, 106.0), (62.2, 131.0),
                 (86.9, 131.0), (87.8, 117.0), (88.9, 90.0), (89.9, 54.0),
                 (90.8, 24.0), (91.8, 8.5), (93.2, 2.0), (95.0, 8.0))
        self.key(("reach", "L"), "", (50, 1.0), (53, 0.97), (55.5, 0.95),
                 (57.2, 0.94), (58.4, 0.92), (59.4, 1.0), (60.5, 1.35),
                 (62, 1.78),
                 (86.9, 1.60), (87.8, 1.66), (88.9, 1.58), (89.9, 1.34),
                 (90.8, 1.12), (91.8, 0.99), (93, 0.94), (95.5, 0.96))
        # right: the "ta-da", a frame or two behind the left on the way up;
        # held out to the side (clear of the head), and it lands two to
        # three frames BEFORE the left on the way down, with less overshoot.
        # Both arms settle to the hang (overshooting it inward a little),
        # never onto the thighs.
        self.key(("psi", "R"), "", (49.5, 18.0), (53, 20.0), (56.2, 22.5),
                 (58.2, 23.5), (59.4, 20.0), (60.5, 26.0), (61.7, 52.0),
                 (63, 66.0), (64.3, 76.0), (65.8, 63.0), (67.5, 69.0),
                 (70, 66.0), (76, 68.0), (82, 65.0), (84.2, 63.0),
                 (85.8, 60.0), (87.2, 28.0), (88.5, 8.0), (89.8, -2.0),
                 (91.8, 8.0), (94.5, HANG + 3.0),
                 # the first strut step's dip swings it out a little (the
                 # planted knee bows out under it)
                 (98.8, 13.5))
        self.key(("reach", "R"), "", (50, 0.92), (53, 0.93), (56.2, 0.92),
                 (58.2, 0.91), (59.4, 0.90), (60.5, 0.96), (62, 1.10),
                 (64.3, 1.22), (67, 1.20), (82, 1.20), (85, 1.14),
                 (87.5, 1.0), (89.8, 0.94), (93, 0.95))

    def arm_geometry(self, s):
        sh, hd = SHOULDER[s], HAND[s]
        d0 = length(sub(hd, sh))
        th0 = math.degrees(math.atan2(hd[1] - sh[1], hd[0] - sh[0]))
        return d0, th0

    def hand_world(self, s, f):
        d0, th0 = self.arm_geometry(s)
        psi = self.ch[(("psi", s), "")](f)
        r = self.ch[(("reach", s), "")](f)
        ang = math.radians(th0 + SGN[s] * psi)
        sp = self.world("Shoulder_" + s, f).ExtractTranslation()
        return (sp[0] + r * d0 * math.cos(ang), sp[1] + r * d0 * math.sin(ang),
                Z_ARM[s])

    def polar_of(self, s, pos, f):
        d0, th0 = self.arm_geometry(s)
        sp = self.world("Shoulder_" + s, f).ExtractTranslation()
        dx, dy = pos[0] - sp[0], pos[1] - sp[1]
        ang = math.degrees(math.atan2(dy, dx))
        psi = (ang - th0) * SGN[s]
        while psi < -90:
            psi += 360
        while psi > 270:
            psi -= 360
        return psi, math.hypot(dx, dy) / d0

    # -- the hat hand-off ------------------------------------------------------

    def hat_lift(self, f):
        keys = [(63, 0, 0, 0), (64, 0, 0, 0), (66.5, 0.10, 0.40, -7.0),
                (69.5, 0.28, 0.70, -20.0), (72.5, 0.34, 0.80, -30.0),
                (75.5, 0.31, 0.73, -17.0), (78.5, 0.33, 0.77, -27.0),
                (81.5, 0.14, 0.34, -9.0), (83.8, 0.01, 0.05, -1.0),
                (85, 0, 0, 0), (86, 0, 0, 0)]
        cx = Spline([(k[0], k[1]) for k in keys], periodic=False)
        cy = Spline([(k[0], k[2]) for k in keys], periodic=False)
        cr = Spline([(k[0], k[3]) for k in keys], periodic=False)
        return (Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1),
                                                      cr(f))) *
                Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(cx(f), cy(f), 0)))

    def grip(self):
        """The left glove's pose in hat space while it holds the hat:
        fingers along the hat's -X, palm on the hat's right side."""
        ang = 170.0
        palm = rot2((0.0, -0.50), ang)
        want = (1.02, 0.44)
        wrist = sub(want, palm)
        dz = Z_ARM["L"] - Z_HAT
        return (Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), ang)) *
                Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(wrist[0], wrist[1], dz)))

    def hand_off(self):
        """While the hat is held the left hand is solved exactly from the
        hat (keys in the hand's polar channels), so attach and release
        never pop."""
        P = self.P
        grip = self.grip()
        self.glove_solved = {}
        for f in range(GRAB, LET_GO + 1):
            G = grip * self.hat_lift(f) * self.world(P["Hat"], f)
            p = G.ExtractTranslation()
            psi, r = self.polar_of("L", p, f)
            self.key(("psi", "L"), "", (f, psi))
            self.key(("reach", "L"), "", (f, r))
            x_axis = G.GetRow3(0)
            self.glove_solved[f] = math.degrees(math.atan2(x_axis[1],
                                                           x_axis[0]))
        off = grip.GetInverse()
        t = off.ExtractTranslation()
        xa = off.GetRow3(0)
        self.hat_offset = (Gf.Vec3d(t[0], t[1], t[2]),
                           Gf.Vec3d(0, 0, math.degrees(math.atan2(xa[1], xa[0]))))
        self.hat_weight = Spline([(1, 0), (GRAB, 0), (GRAB + 1, 1),
                                  (LET_GO - 1, 1), (LET_GO, 0)])

    def arms(self):
        self.arm_swing()
        self.hand_off()
        for s in SIDES:
            for name in ("psi", "reach"):
                self.ch[((name, s), "")] = Spline(self.keys.pop(((name, s), "")))
        # the hand controls: translate-only, in COG space
        for s in SIDES:
            tx, ty = {}, {}
            for f in FRAMES:
                t = self.local_translate_for("Hand_" + s, self.hand_world(s, f),
                                             f)
                tx[f], ty[f] = t[0], t[1]
            self.set("Hand_" + s, "tx", Table(tx))
            self.set("Hand_" + s, "ty", Table(ty))

    # limb shaping: bow design -> IK lengths + poles

    def bow_design(self):
        """Per limb: the side multiplier (the only thing that may swing a
        hose through), the length or slack that sets the bow, and the drag
        gain.  The left arm swings through twice, on purpose."""
        one = lambda f: 1.0  # noqa: E731
        # the swing-through: the bow melts while the hand sweeps out, the
        # hose runs straight through the fastest part of the sweep (a whip,
        # as the arm passes horizontal), then curls round the head as the
        # hand arrives; the reverse when the arm drops after the hat is set
        # back.  Each change is one eased S (2.5x^3 - 1.5x^5: zero speed
        # and zero acceleration at its ends, a soft linger through straight
        # in the middle) -- never a key-to-key zigzag.
        # (centre, ease-in width, ease-out width, side before, pass rate).
        # A pass rate of None is the launch's lingering S (ease_through);
        # a number is a C1 Hermite S whose slope through straight is that
        # fraction of the fastest monotone pass: the release, where the
        # hose must NOT rest straight -- it passes straight at the hand's
        # top speed and re-curves upward, trailing the falling hand.
        swings = ((60.0, SWING_IN[0], SWING_OUT[0], 1.0, None),
                  (RELEASE_C, SWING_IN[1], SWING_OUT[1], -1.0, RELEASE_RATE))
        shapes = []
        for tc, wa, wb, s0, rate in swings:
            if rate is None:
                shapes.append(None)
                continue
            m = -s0 * rate * 3.0 / (wa + wb)
            shapes.append(Spline([(tc - wa, s0, 0.0), (tc, 0.0, m),
                                  (tc + wb, -s0, 0.0)], periodic=False))

        def side_L(f):
            f = wrap(f)
            for (tc, wa, wb, s0, _), shape in zip(swings, shapes):
                if tc - wa <= f <= tc + wb:
                    if shape is not None:
                        return shape(f)
                    x = (f - tc) / (wa if f < tc else wb)
                    return -s0 * ease_through(x)
            return -1.0 if swings[0][0] + swings[0][2] < f < \
                swings[1][0] - swings[1][1] else 1.0
        self.intent["Arm_L"] = []
        for tc, wa, wb, _, _ in swings:
            self.intent["Arm_L"] += [tc - wa - 0.4, tc + wb + 0.4]
        # the strut: the flung-out arm gets a little more slack (a rounder
        # rubber-hose curve at the top of its swing); the hanging arm stays
        # fairly taut so its elbow bows out, away from the body
        strut = {s: [(c + 6.6 + STRUT_DT[c] +
                      (0.0 if s == X else STRUT_VAR[c][1]),
                      0.32 if s == X else 0.24)
                     for c, X, a in STRUT] for s in SIDES}
        self.bow = {
            # the hat hold: slack enough that the curl stands clear of the
            # head (no hairline tangent along the head outline)
            ("Arm", "L"): dict(side=side_L, slack=Spline(strut["L"] + [
                (50, 0.28), (56, 0.24), (59, 0.20), (61.5, 0.62),
                (62.5, HOLD_SLACK + 0.14), (63.6, HOLD_SLACK + 0.06),
                (65, HOLD_SLACK), (82.5, HOLD_SLACK),
                (85.0, HOLD_SLACK + 0.18), (86.4, HOLD_SLACK + 0.16),
                (87.6, 0.70), (89, 0.45), (90.5, 0.32), (93, 0.28)]),
                hmin=0.10, drag=0.030),
            ("Arm", "R"): dict(side=one, slack=Spline(strut["R"] + [
                (50, 0.28), (56, 0.26), (60, 0.24), (63, 0.15), (66, 0.07),
                (83, 0.07), (86.5, 0.16), (89, 0.24), (92, 0.28)]),
                hmin=0.10, drag=0.030),
        }
        for s in SIDES:
            L0 = 2.0 * (0.5 * length(sub(ANKLE[s], HIP[s])) + LEG_EXTRA)
            self.bow[("Leg", s)] = dict(side=one, L=Spline([
                (49, L0), (53, L0 - 0.08), (58, L0 - 0.30), (60, L0 + 0.05),
                (62, L0 + 0.26), (66, L0 + 0.20), (84, L0 + 0.16),
                (88, L0 + 0.02), (94, L0)]), hmin=LEG_HMIN, soft=LEG_SOFT,
                drag=0.010)

    def limb_rest(self, kind, s):
        root, end, pole, extra = LIMB_REST[(kind, s)]
        chord = sub(end, root)
        return dict(
            root=root, end=end, chord=chord, l=0.5 * length(chord),
            ref=1.0 if cross2(chord, sub(pole, root)) > 0 else -1.0,
            cv1=add(root, mul(chord, 1.0 / (HOSE_N - 1))),
            cv2=add(root, mul(chord, (HOSE_N - 2.0) / (HOSE_N - 1))),
            mid=lerp(root, end, 0.5),
            z=Z_ARM[s] if kind == "Arm" else Z_LEG)

    def limbs(self):
        self.bow_design()
        P = self.P
        for kind in ("Arm", "Leg"):
            for s in SIDES:
                rest = self.limb_rest(kind, s)
                z = rest["z"]
                root_ctl = P[("ArmRoot_" if kind == "Arm" else "Hip_") + s]
                end_ctl = P[("ArmEnd_" if kind == "Arm" else "LegEnd_") + s]
                pole_ctl = P[("ArmPole_" if kind == "Arm" else "KneePole_") + s]
                W0r = self.pip.ctl_world[root_ctl]
                W0e = self.pip.ctl_world[end_ctl]
                v3 = lambda p: Gf.Vec3d(p[0], p[1], z)  # noqa: E731
                # pass 0: the chord's angle; the tangent helpers turn with it
                # (minus whatever their parents already turn)
                ang0 = math.degrees(math.atan2(rest["chord"][1],
                                               rest["chord"][0]))
                ang, prev = {}, None
                for f in FRAMES:
                    R = self.world(root_ctl, f).ExtractTranslation()
                    H = self.world(end_ctl, f).ExtractTranslation()
                    a = math.degrees(math.atan2(H[1] - R[1], H[0] - R[0]))
                    if prev is not None:
                        while a - prev > 180:
                            a -= 360
                        while a - prev < -180:
                            a += 360
                    ang[f] = prev = a
                for helper in (root_ctl, end_ctl):
                    rz = {}
                    for f in FRAMES:
                        pw = self.parent_world(helper, f)
                        xa = pw.GetRow3(0)
                        prot = math.degrees(math.atan2(xa[1], xa[0]))
                        v = ang[f] - ang0 - prot
                        while v > 180:
                            v -= 360
                        while v < -180:
                            v += 360
                        rz[f] = v
                    # keep it continuous
                    prev = None
                    for f in FRAMES:
                        if prev is not None:
                            while rz[f] - prev > 180:
                                rz[f] -= 360
                            while rz[f] - prev < -180:
                                rz[f] += 360
                        prev = rz[f]
                    self.set(helper, "rz", Table(rz))
                # pass 1: the chord of every sample
                geo = {}
                for f in FRAMES:
                    Wr = self.world(root_ctl, f)
                    We = self.world(end_ctl, f)
                    rm = W0r.GetInverse() * Wr
                    em = W0e.GetInverse() * We
                    R = Wr.ExtractTranslation()
                    H = We.ExtractTranslation()
                    follow = 0.5 * (rm.Transform(v3(rest["mid"])) +
                                    em.Transform(v3(rest["mid"])))
                    base = 0.5 * (rm.Transform(v3(rest["cv1"])) +
                                  em.Transform(v3(rest["cv2"])))
                    geo[f] = dict(R=(R[0], R[1]), H=(H[0], H[1]),
                                  corr=(base[0] - follow[0],
                                        base[1] - follow[1]),
                                  cv2=em.Transform(v3(rest["cv2"])),
                                  follow=follow)
                # chord angle, unwrapped, and its rate toward the bow side
                ang, prev = {}, None
                for f in FRAMES:
                    c = sub(geo[f]["H"], geo[f]["R"])
                    a = math.degrees(math.atan2(c[1], c[0]))
                    if prev is not None:
                        while a - prev > 180:
                            a -= 360
                        while a - prev < -180:
                            a += 360
                    ang[f] = prev = a
                ang_t = Table(ang)
                bd = self.bow[(kind, s)]

                def omega(f):
                    return rest["ref"] * (ang_t(f + 0.5) - ang_t(f - 0.5))
                # the drag: the hose lags its swing (a spring, so it
                # overshoots and settles when the swing stops)
                w = 2 * math.pi * 3.0 / 24.0
                drag = spring(lambda f: omega(f) * bd["drag"] * w * w,
                              3.0, 0.65, 1.0)
                # the knee's extra bend while the foot is up follows the
                # foot a little late (it absorbs the landing, no snap)
                lift = self._lagged(
                    lambda f, s=s: self.get("Foot_" + s, "ty", f) / 0.62,
                    3.2, 0.9) if kind == "Leg" else (lambda f: 0.0)
                up, lo, ptx, pty, ptz, tau = {}, {}, {}, {}, {}, {}
                dbg = []
                for f in FRAMES:
                    g_ = geo[f]
                    R, H = g_["R"], g_["H"]
                    c = sub(H, R)
                    d = length(c)
                    u = mul(c, 1.0 / d)
                    n = (-u[1], u[0])
                    nref = mul(n, rest["ref"])
                    L = bd["L"](f) if "L" in bd else d + bd["slack"](f)
                    # bow magnitude: the hose's slack (or length) sets it
                    m = math.sqrt(bd["hmin"] ** 2 +
                                  softplus((L * L - d * d) / 4.0,
                                           bd.get("soft", 0.02)))
                    m += 0.16 * lift(f)
                    # The side curve is the only thing that may swing a hose
                    # through.  Everything stays IN THE PICTURE PLANE: the
                    # hose straightens and re-curves as the side curve eases
                    # through zero.  (Rolling the bend plane toward the
                    # camera instead -- an earlier try -- tilts the hose
                    # segments out of the plane, and a segment aimed nearly
                    # opposite its rest direction then has its minimal-
                    # rotation frame turned edge-on: the strip thins to a
                    # line for a frame or two.)
                    sd = max(-1.0, min(1.0, bd["side"](f)))
                    # the drag never flips a hose on its own: it is bounded
                    # by the bow, and it fades out smoothly (as sd^2) near a
                    # swing-through, so the hose's rate through straight is
                    # continuous (a bound proportional to |sd| alone would
                    # halve it on one side of straight and double it on the
                    # other: a snap)
                    fade = sd * sd * (1.0 + DRAG_FADE ** 2) / (
                        sd * sd + DRAG_FADE ** 2)
                    dg = soft_clip(drag(f), 0.6 * abs(sd) * m) * fade
                    h = sd * m + dg
                    a = 0.5 - 0.05 * lift(f) + 0.05 * math.tanh(dg / 0.3)
                    C = add(R, mul(u, a * d))          # apex's chord point
                    D = add(C, mul(nref, h))
                    E = sub(D, g_["corr"])
                    l1 = length(sub(E, R))
                    l2 = length(sub(H, E))
                    # Through the crossing itself the IK is held exactly
                    # straight (a hair shorter than the chord, so it is
                    # 'stretched' and ignores the pole): between two written
                    # samples the pole sweeps across the chord line, and a
                    # straight chain has no bend side to flick.
                    hn = dot2(sub(E, R), nref)
                    tight = max(0.0, 1.0 - (hn / 0.012) ** 2)
                    if tight > 0.0:
                        k = 1.0 - 0.00004 * tight
                        l1, l2 = l1 * k, l2 * k
                    up[f] = l1 - rest["l"]
                    lo[f] = l2 - rest["l"]
                    pole = add(C, mul(sub(E, C), 2.3))
                    t = self.local_translate_for(
                        pole_ctl, (pole[0], pole[1], z), f)
                    ptx[f], pty[f], ptz[f] = t[0], t[1], t[2]
                    # the hose's end tangent (for the glove)
                    O = sub(E, (g_["follow"][0], g_["follow"][1]))
                    P2 = (g_["cv2"][0] + O[0], g_["cv2"][1] + O[1])
                    tv = sub(H, P2)
                    tau[f] = math.degrees(math.atan2(tv[1], tv[0]))
                    dbg.append((f, d, h, hn, dg))
                name = "%sIK_%s" % (kind, s)
                self.solver_tables[name] = (Table(up), Table(lo))
                self.set(pole_ctl, "tx", Table(ptx))
                self.set(pole_ctl, "ty", Table(pty))
                self.set(pole_ctl, "tz", Table(ptz))
                self.debug["%s_%s" % (kind, s)] = dbg
                if kind == "Arm":
                    self.tau = getattr(self, "tau", {})
                    prev = None
                    for f in FRAMES:
                        a = tau[f]
                        if prev is not None:
                            while a - prev > 180:
                                a -= 360
                            while a - prev < -180:
                                a += 360
                        tau[f] = prev = a
                    self.tau[s] = Table(tau)

    def gloves(self):
        """The glove continues the hose's line (with a lag: follow-through)
        plus keyed accents; while the hat is held it is the solved grip."""
        for s in SIDES:
            tau = self.tau[s]
            t0 = tau(1.0)
            base0 = 12.0 * SGN[s]

            def follow(f, tau=tau, t0=t0):
                return 0.6 * (tau(f - 1.5) - t0)
            ks = []
            if s == "L":
                ks += [(50, 0.0), (56, 10.0), (59, 25.0), (61, 40.0)]
                for f in range(GRAB, LET_GO + 1):
                    want = self.glove_solved[f] - base0
                    while want - follow(f) > 180:
                        want -= 360
                    while want - follow(f) < -180:
                        want += 360
                    ks.append((f, want - follow(f)))
                # the release and drop: total angles (follow + accent); the
                # fingers trail the swing, then one overshoot
                for f, total in ((86.9, 147.0), (88.0, 136.0), (89.3, 92.0),
                                 (90.5, 24.0), (91.7, -14.0), (93.4, 6.0),
                                 (95.8, -2.0)):
                    ks.append((f, total - follow(f)))
            else:
                ks += [(50, 0.0), (58, 5.0), (61, -20.0), (63.5, -70.0),
                       (65, -104.0), (67, -88.0), (70, -96.0), (78, -92.0),
                       (83, -90.0)]
                for f, total in ((85.5, -112.0), (87.1, -52.0),
                                 (88.5, 4.0), (89.9, 14.0), (91.5, -4.0),
                                 (94.0, 1.0)):
                    ks.append((f, total - follow(f)))
            delta = Spline(ks)
            self.set("Glove_" + s, "rz",
                     lambda f, follow=follow, delta=delta: follow(f) + delta(f))
        self.key("Glove_R", "sx", (63, 1.0), (65, 1.16), (68, 1.0))
        self.key("Glove_R", "sy", (63, 1.0), (65, 1.16), (68, 1.0))
        self.build_keys()

    # the face

    def face(self):
        look = [  # (frame, tx, ty) in COG space, offsets from rest
            (1, 0.35, -0.35), (9, 0.35, -0.35), (11, -1.55, 0.05),
            (19, -1.55, 0.05), (21, 1.45, 0.35), (32, 1.45, 0.35),
            (34, 0.0, -0.10), (45, 0.0, -0.10), (48, 0.75, 2.2),
            (58, 0.75, 2.2), (61, 0.0, 0.25), (84, 0.0, 0.25),
            (87, -0.8, -1.2), (95, -0.8, -1.2), (97.5, 0.35, -0.35)]
        # saccades: two-frame darts, then holds (with a little drift)
        lx = Spline([(f, x) for f, x, _ in look])
        ly = Spline([(f, y) for f, _, y in look])
        self.set("LookAt", "tx", lx)
        self.set("LookAt", "ty", ly)
        # the head 'turn' (face slide) follows the eyes 2-3 frames later and
        # settles softly: a spring on the look target
        slide_x = self._lagged(lambda f: max(-0.16, min(0.16, 0.10 * lx(f))),
                               2.4, 0.75)
        slide_y = self._lagged(lambda f: max(-0.06, min(0.08, 0.035 * ly(f))),
                               2.4, 0.75)
        self.set("FaceSlide", "tx", slide_x)
        self.set("FaceSlide", "ty", slide_y)
        blink = Spline([(1, 0), (8.5, 0), (10, 1), (11, 1), (14.5, 0),
                        (25.5, 0), (27, 1), (28, 1), (31.5, 0), (52.5, 0),
                        (55, 0.5), (58, 0.5), (60, 0), (86.5, 0), (88, 1),
                        (89, 1), (92.5, 0), (104, 0), (105.5, 1), (106.3, 1),
                        (109.5, 0)])
        self.dials["face:blinkL"] = blink
        self.dials["face:blinkR"] = lambda f: blink(f - 0.5)
        self.dials["face:mouthOpen"] = Spline([
            (1, 0), (56, 0), (60, 0.35), (62.5, 0.8), (64.5, 1.0),
            (77, 1.0), (81.5, 0.72), (85.5, 0.30), (87.5, 0.55), (92, 0.0)])
        self.dials["face:mouthOo"] = Spline([
            (1, 0), (50, 0), (53, 0.8), (57, 0.9), (59, 0.4), (61, 0.0)])
        ks = []
        for c, X, a in STRUT:
            ks += [(c + 3, 0.28), (c + 8.5, 0.42)]
        ks += [(49, 0.30), (53, 0.0), (59, 0.0), (63, 0.3), (70, 0.5),
               (84, 0.5), (92, 0.6), (95, 0.40)]
        self.dials["face:mouthWide"] = Spline(ks)
        for c, X, a in STRUT:
            g = SGN[X]
            for s in SIDES:
                self.key("Brow_" + s, "ty", (c + 4, 0.0), (c + 9, 0.05))
                self.key("Brow_" + s, "rz", (c + 9, 0.0))
            self.key("Mouth", "rz", (c + 9, g * 2.5))
        for s in SIDES:
            g = SGN[s]
            self.key("Brow_" + s, "ty", (49, 0.0), (53, -0.10), (58, -0.10),
                     (61, 0.08), (63.5, 0.17), (80, 0.13), (86, 0.02),
                     (89, 0.10), (93, 0.03))
            self.key("Brow_" + s, "rz", (49, 0.0), (53, -9.0 * g),
                     (58, -9.0 * g), (61, 0.0), (64, 6.0 * g), (80, 5.0 * g),
                     (88, 0.0))
        self.key("Mouth", "rz", (57, 0.0), (66, -4.0), (80, -3.0), (88, 0.0))
        self.build_keys()

    def _lagged(self, target, hz, zeta):
        """A spring that chases target(f): x'' = w^2 (target - x) - ..."""
        w = 2 * math.pi * hz / 24.0
        dt = 1.0 / SUB
        x = v = 0.0
        arr = [0.0] * (NS + 1)
        for loop in range(4):
            for k in range(NS):
                v += dt * (w * w * (target(FRAMES[k]) - x) - 2 * zeta * w * v)
                x += dt * v
                if loop == 3:
                    arr[(k + 1) % NS] = x
        arr[NS] = arr[0]
        return Table(arr)

    def build(self):
        self.body()
        self.secondary()
        self.legs()
        self.arms()
        self.limbs()
        self.gloves()
        self.face()


# -- picker ------------------------------------------------------------------
#
# Pip's control picker: Body and Face pages selecting the rig's animator
# controls. Buttons carry the guide colour of the control they select, so
# the panel and the viewport agree; the hidden solver helpers (hip roots,
# hose ends, eye aims) get no buttons. Character left is +X, which the
# default camera sees on the RIGHT, so L buttons sit right of R ones --
# the same convention as the biped picker.

P_FILL = {
    "center": (1.0, 0.82, 0.18, 1.0),
    "left": (0.25, 0.62, 1.0, 1.0),
    "right": (1.0, 0.33, 0.33, 1.0),
    "face": (1.0, 0.45, 0.85, 1.0),
    "look": (1.0, 0.55, 0.12, 1.0),
    "dim": (0.4, 0.4, 0.4, 1.0),
    "ghost": (0.45, 0.45, 0.5, 0.25),
    "none": (0.0, 0.0, 0.0, 0.0),
}
P_INK = (0.06, 0.06, 0.06, 1.0)
P_PAPER = (0.92, 0.92, 0.92, 1.0)


class PickerAuthor(object):
    """One RigExecPickerButton per call, in panel units (y down)."""

    def __init__(self, stage):
        self.a = Author(stage)
        self.depth = 0.0

    def button(self, panel, name, x, y, w, h, shape="roundedRectangle",
               fill="center", text="", size=9.0, target=None, mirror=None,
               depth=None, bold=False, flat=False):
        prim = self.a.prim(panel + "/" + name, "RigExecPickerButton")
        self.a.attr(prim, "ui:position", T.Float2, Gf.Vec2f(x, y),
                    uniform=True)
        self.a.attr(prim, "ui:size", T.Float2, Gf.Vec2f(w, h), uniform=True)
        self.a.attr(prim, "ui:shape", T.Token, shape, uniform=True)
        if shape == "roundedRectangle":
            self.a.attr(prim, "ui:roundness", T.Float, 0.3, uniform=True)
        self.a.attr(prim, "ui:fill", T.Color4f, Gf.Vec4f(*P_FILL[fill]),
                    uniform=True)
        if text:
            self.a.attr(prim, "ui:text", T.String, text, uniform=True)
            self.a.attr(prim, "ui:fontSize", T.Float, size, uniform=True)
            if bold:
                self.a.attr(prim, "ui:bold", T.Bool, True, uniform=True)
            ink = P_PAPER if fill in ("dim", "none") else P_INK
            self.a.attr(prim, "ui:textColor", T.Color4f, Gf.Vec4f(*ink),
                        uniform=True)
        if flat:
            self.a.attr(prim, "ui:strokeWidth", T.Float, 0.0, uniform=True)
        if depth is None and target is not None:
            self.depth += 0.001
            depth = self.depth
        if depth is not None:
            self.a.attr(prim, "ui:depth", T.Float, depth, uniform=True)
        if isinstance(target, str) and target.startswith("cmd:"):
            self.a.attr(prim, "rigExec:picker:command", T.Token,
                        target[4:], uniform=True)
        elif target:
            self.a.rel(prim, "rigExec:picker:controls", target)
        if mirror:
            self.a.rel(prim, "rigExec:picker:mirror", mirror)
        return prim


def write_picker(path):
    layer = Sdf.Layer.CreateNew(path) if not os.path.exists(path) else \
        Sdf.Layer.FindOrOpen(path)
    layer.Clear()
    layer.documentation = (
        "Pip's control picker: Body and Face pages selecting the rig's "
        "animator controls. Generated by build_rubberhose.py.")
    stage = Usd.Stage.Open(layer)
    stage.SetEditTarget(layer)
    root = stage.DefinePrim(ROOT)
    stage.SetDefaultPrim(root)
    stage.DefinePrim(RIG)
    a = Author(stage)
    picker = a.prim(PCK, "RigExecPicker")
    a.rel(picker, "rigExec:picker:rig", RIG)

    ctl_root = CTL + "/Root_ctl"
    cog = ctl_root + "/COG_ctl"
    hips = cog + "/Hips_ctl"
    chest = cog + "/Chest_ctl"
    head = chest + "/Head_ctl"
    squash = head + "/HeadSquash_ctl"
    htop = squash + "/HeadTop_ctl"
    hat = htop + "/Hat_ctl"
    slide = head + "/FaceSlide_ctl"
    mouth = slide + "/Mouth_ctl"
    face = slide + "/Face_ctl"
    look = cog + "/LookAt_ctl"
    belly = cog + "/SpineMidFollow/SpineMid_ctl"
    sho = dict((s, chest + "/Shoulder_%s_ctl" % s) for s in SIDES)
    hand = dict((s, cog + "/Hand_%s_ctl" % s) for s in SIDES)
    glove = dict((s, hand[s] + "/Glove_%s_ctl" % s) for s in SIDES)
    apole = dict((s, cog + "/ArmPole_%s_ctl" % s) for s in SIDES)
    foot = dict((s, ctl_root + "/Foot_%s_ctl" % s) for s in SIDES)
    shoe = dict((s, foot[s] + "/Shoe_%s_ctl" % s) for s in SIDES)
    kpole = dict((s, ctl_root + "/KneePole_%s_ctl" % s) for s in SIDES)
    brow = dict((s, slide + "/Brow_%s_ctl" % s) for s in SIDES)

    def rx(x, w):
        """The same slot on the panel's right half (character left)."""
        return 400 - x - w

    pa = PickerAuthor(stage)

    # -- Body page: the character head to toe --
    body = PCK + "/Body"
    bp = a.prim(body, "RigExecPickerPanel")
    a.attr(bp, "ui:label", T.String, "Body", uniform=True)
    a.attr(bp, "ui:order", T.Int, 0, uniform=True)
    a.attr(bp, "ui:size", T.Float2, Gf.Vec2f(400, 600), uniform=True)
    pa.button(body, "d_title", 12, 8, 180, 22, shape="rectangle", fill="none",
              text="PIP - BODY", size=13.0, bold=True, flat=True, depth=0.05)
    pa.button(body, "d_head", 140, 50, 120, 110, shape="ellipse",
              fill="ghost", depth=-0.5)
    pa.button(body, "d_body", 145, 180, 110, 140, shape="ellipse",
              fill="ghost", depth=-0.5)
    pa.button(body, "b_Hat", 168, 24, 64, 24, text="Hat", target=hat)
    pa.button(body, "b_HeadTop", 186, 56, 28, 28, shape="circle",
              fill="face", target=htop)
    pa.button(body, "b_Head", 158, 90, 84, 26, text="Head", target=head)
    pa.button(body, "b_HeadSquash", 250, 90, 72, 26, fill="face",
              text="Squash", target=squash)
    pa.button(body, "b_Chest", 168, 150, 64, 26, text="Chest", target=chest)
    pa.button(body, "b_SpineMid", 176, 190, 48, 26, text="Belly",
              target=belly)
    pa.button(body, "b_Hips", 168, 228, 64, 26, text="Hips", target=hips)
    pa.button(body, "b_COG", 150, 266, 100, 30, text="COG", target=cog)
    pa.button(body, "b_Root", 140, 552, 120, 26, text="Root (world)",
              target=ctl_root)
    body_sides = (
        # stem, R-slot rect, shape, label, size, targets
        ("Shoulder", (92, 140, 60, 24), "roundedRectangle", "Shldr",
         9.0, sho),
        ("ArmPole", (40, 150, 40, 40), "hexagon", "Pole", 8.0, apole),
        ("Hand", (36, 226, 64, 28), "roundedRectangle", "Hand", 9.0, hand),
        ("Glove", (36, 260, 64, 28), "roundedRectangle", "Glove", 9.0,
         glove),
        ("KneePole", (40, 392, 40, 40), "hexagon", "Pole", 8.0, kpole),
        ("Foot", (104, 470, 64, 28), "roundedRectangle", "Foot", 9.0, foot),
        ("Shoe", (104, 504, 64, 28), "roundedRectangle", "Shoe", 9.0, shoe),
    )
    for stem, (x, y, w, h), shape, text, size, targets in body_sides:
        for s in ("R", "L"):
            o = "L" if s == "R" else "R"
            pa.button(body, "b_%s_%s" % (stem, s),
                      x if s == "R" else rx(x, w), y, w, h, shape=shape,
                      fill="left" if s == "L" else "right",
                      text="%s %s" % (text, s), size=size, target=targets[s],
                      mirror=body + "/b_%s_%s" % (stem, o))
    pa.button(body, "t_Zero_Ctrls", 308, 8, 84, 22, shape="rectangle",
              fill="dim", text="Zero Ctrls", target="cmd:zero_ctrls")

    # -- Face page: the head big, dials below --
    face_panel = PCK + "/Face"
    fp = a.prim(face_panel, "RigExecPickerPanel")
    a.attr(fp, "ui:label", T.String, "Face", uniform=True)
    a.attr(fp, "ui:order", T.Int, 1, uniform=True)
    a.attr(fp, "ui:size", T.Float2, Gf.Vec2f(400, 600), uniform=True)
    pa.button(face_panel, "d_title", 12, 8, 180, 22, shape="rectangle",
              fill="none", text="PIP - FACE", size=13.0, bold=True,
              flat=True, depth=0.05)
    pa.button(face_panel, "d_head", 90, 60, 220, 220, shape="circle",
              fill="ghost", depth=-0.5)
    for s in ("R", "L"):
        o = "L" if s == "R" else "R"
        pa.button(face_panel, "b_Brow_%s" % s,
                  116 if s == "R" else rx(116, 62), 148, 62, 24,
                  fill="face", text="Brow %s" % s, target=brow[s],
                  mirror=face_panel + "/b_Brow_%s" % o)
    pa.button(face_panel, "b_FaceSlide", 166, 192, 68, 28, fill="face",
              text="Slide", target=slide)
    pa.button(face_panel, "b_Mouth", 158, 236, 84, 28, fill="face",
              text="Mouth", target=mouth)
    pa.button(face_panel, "b_LookAt", 148, 304, 104, 28, fill="look",
              text="Look At", target=look)
    pa.button(face_panel, "b_Face", 148, 348, 104, 30, fill="face",
              text="Face Dials", target=face)
    pa.button(face_panel, "b_Head", 158, 392, 84, 26, text="Head",
              target=head)
    pa.button(face_panel, "t_Zero_Ctrls", 308, 8, 84, 22, shape="rectangle",
              fill="dim", text="Zero Ctrls", target="cmd:zero_ctrls")
    layer.Save()


def write_anim(pip, perf, path):
    layer = Sdf.Layer.CreateNew(path) if not os.path.exists(path) else \
        Sdf.Layer.FindOrOpen(path)
    layer.Clear()
    layer.subLayerPaths.append("./" + os.path.basename(RIG_FILE))
    stage = Usd.Stage.Open(layer)
    stage.SetEditTarget(layer)
    stage.SetStartTimeCode(1)
    stage.SetEndTimeCode(LOOP)
    stage.SetTimeCodesPerSecond(24)
    stage.SetFramesPerSecond(24)
    layer.documentation = (
        "Pip's performance: a %d-frame loop at 24 fps (frame %d == frame 1). "
        "Only control avars, face dials, the four limb IKs' length offsets "
        "and the hat hand-off constraint are keyed; everything else is the "
        "rig. The limb poles and length offsets are computed per sample so "
        "every hose bows where the performance wants it and only swings "
        "through where customLayerData "
        "'rubberhose:intendedBendChanges' says so (check_limbs.py verifies "
        "it). Generated by build_rubberhose.py." % (LOOP, LOOP + 1))
    layer.customLayerData = {
        "rubberhose:intendedBendChanges": {
            k: Vt.DoubleArray(v) for k, v in sorted(perf.intent.items())},
        "rubberhose:footContacts": {
            k: Vt.DoubleArray(sorted(v))
            for k, v in sorted(perf.contacts.items())}}

    def key(prim_path, name, tname, fn, custom=False):
        prim = stage.GetPrimAtPath(prim_path)
        attr = prim.GetAttribute(name)
        if not attr:
            attr = prim.CreateAttribute(name, tname, custom)
        vals = [fn(f) for f in FRAMES]
        if max(vals) - min(vals) < 1e-7 and abs(vals[0] - (1.0 if name.endswith(
                ("sx", "sy", "sz")) else 0.0)) < 1e-7:
            return
        for f, v in zip(FRAMES, vals):
            attr.Set(round(v, 5), Usd.TimeCode(f))

    for (ctl, avar), fn in sorted(perf.ch.items(), key=lambda kv: str(kv[0])):
        if not isinstance(ctl, str) or not avar:
            continue                     # generator-internal channels
        key(ctl, "avars:" + avar, T.Double, fn)
    for dial, fn in sorted(perf.dials.items()):
        key(perf.P["Face"], dial, T.Float, fn, custom=True)
    for solver, (up, lo) in sorted(perf.solver_tables.items()):
        key(SOLV + "/" + solver, "rigExec:upperLengthOffset", T.Double, up)
        key(SOLV + "/" + solver, "rigExec:lowerLengthOffset", T.Double, lo)
    pc = stage.GetPrimAtPath(MOV + "/Pose/HatToGlove")
    pc.GetAttribute("inputs:translationOffsets").Set(
        Vt.Vec3dArray([perf.hat_offset[0]]))
    pc.GetAttribute("inputs:rotationOffsets").Set(
        Vt.Vec3dArray([perf.hat_offset[1]]))
    key(pc.GetPath().pathString, "inputs:defaultWeight", T.Float,
        perf.hat_weight)
    layer.Save()


def build_rig(path):
    layer = Sdf.Layer.CreateNew(path) if not os.path.exists(path) else \
        Sdf.Layer.FindOrOpen(path)
    layer.Clear()
    stage = Usd.Stage.Open(layer)
    layer.subLayerPaths.append("./" + os.path.basename(PICKER_FILE))
    pip = Pip(stage)
    pip.build()
    layer.Save()
    return pip


def main():
    write_picker(PICKER_FILE)
    print("wrote", PICKER_FILE)
    pip = build_rig(RIG_FILE)
    print("wrote", RIG_FILE)
    perf = Performance(pip)
    perf.build()
    write_anim(pip, perf, ANIM_FILE)
    print("wrote", ANIM_FILE)
    return perf


if __name__ == "__main__":
    main()
