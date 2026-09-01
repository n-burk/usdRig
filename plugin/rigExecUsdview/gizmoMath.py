#
# RigExec usdview gizmo math: the evaluator's avar composition replicated
# in Python, edit targets that map world-space gizmo deltas back onto
# avars / rest offsets / xformOps, and the writer that decides whether a
# value lands on the animation (spline knot at the current frame) or on
# the default.
#
# Qt-free by design (see volumeWeightUI.py's banner rule): everything here
# is exercised headlessly by tests/python/test_gizmo_math.py, including a
# comparison against the native evaluator through the _rigexec binding.
#
# Conventions (libs/rigExec/computations.cpp:144-175, 196-215, 284-285),
# row-vector, leftmost factor applies first:
#
#   avars = S * R(rotationOrder) * Rspin(+X) * T
#   rest  = orthonormalize(compose(rest:t, rest:r, XYZ) * rest:space)
#   posed = avars * rest * parentRest^-1 * parentPosed
#
# where "parent" is the nearest namespace-ancestor RigExecXformable and the
# whole rig is in ASSET space: the world transform of the RigExecRoot's
# parent prim places it (libs/rigExec/rigEvaluator.h:79-84).
#
import math

from pxr import Gf, Sdf, Tf, Usd, UsdGeom

ROTATION_ORDERS = ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX")

_AXES = (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
_CYCLIC = ("XYZ", "YZX", "ZXY")
_IDENTITY = Gf.Matrix4d(1.0)

# Attribute names, kept as constants so the targets and the tests agree.
AVAR_T = ("avars:tx", "avars:ty", "avars:tz")
AVAR_R = ("avars:rx", "avars:ry", "avars:rz")
AVAR_S = ("avars:sx", "avars:sy", "avars:sz")
AVAR_RSPIN = "avars:rspin"
AVAR_ORDER = "avars:rotationOrder"
REST_T = ("rest:tx", "rest:ty", "rest:tz")
REST_R = ("rest:rx", "rest:ry", "rest:rz")
REST_SPACE = "rest:space"
POSED_SPACE = "posed:space"


# ---------------------------------------------------------------------------
# Scalars and rotations
# ---------------------------------------------------------------------------

def NormalizeAvarScale(value):
    """
    The evaluator's floor (libs/rigExecMath/avarScale.h:34-44): finite
    magnitudes below 1e-4 become signed 1e-4, non-finite becomes 1.
    Applied before writing so the stage holds what the viewport shows.

    copysign is deliberately unguarded for zero: the native contract keeps
    the sign of NEGATIVE zero (negative scale is a supported reflection),
    and -0.0 == 0.0 in Python, so a `value != 0.0` guard would silently
    flip a reflected axis back to positive and diverge from the evaluator.
    """
    if not math.isfinite(value):
        return 1.0
    if abs(value) < 1e-4:
        return math.copysign(1e-4, value)
    return value


def _NormalizeOrder(order):
    order = str(order or "XYZ").upper()
    return order if order in ROTATION_ORDERS else "XYZ"


def _AxisRotation(axisIndex, degrees):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(_AXES[axisIndex], degrees))
    return m


def RotationFromEuler(order, rx, ry, rz):
    """Rx/Ry/Rz applied in `order` sequence, row-vector (X first for XYZ)."""
    angles = (rx, ry, rz)
    m = Gf.Matrix4d(1.0)
    for axis in _NormalizeOrder(order):
        index = "XYZ".index(axis)
        if angles[index] != 0.0:
            m = m * _AxisRotation(index, angles[index])
    return m


def ComposeAvarMatrix(tx, ty, tz, sx, sy, sz, rx, ry, rz, rspin, order):
    """Mirror of _ComposeAvars: S * R(order) * Rspin * T."""
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(NormalizeAvarScale(sx), NormalizeAvarScale(sy),
                        NormalizeAvarScale(sz)))
    m = m * RotationFromEuler(order, rx, ry, rz)
    if rspin != 0.0:
        m = m * _AxisRotation(0, rspin)
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(tx, ty, tz))
    return m * t


def _Unwrap(angle, hint):
    """`angle` shifted by a multiple of 360 to land nearest `hint`."""
    return angle + 360.0 * round((hint - angle) / 360.0)


def DecomposeEuler(matrix, order, hint=None):
    """
    Euler angles (degrees) whose RotationFromEuler(order, ...) reproduces
    the rotation part of `matrix`. Of the two solutions the one nearest
    `hint` (a 3-tuple of degrees) wins, and each angle is unwrapped to
    the hint's turn, so a drag never flips 180 degrees between events.

    Derivation: with column-vector M = R^T the product is
    M = Rk(g) * Rj(b) * Ri(a) for order (i, j, k); parity eps is +1 for
    cyclic orders. Then sin(b) = -eps*M[k][i], a = atan2(eps*M[k][j],
    M[k][k]) and g = atan2(eps*M[j][i], M[i][i]) -- verified for XYZ and
    XZY by hand, and for all six orders by the round-trip test.
    """
    order = _NormalizeOrder(order)
    i, j, k = ("XYZ".index(axis) for axis in order)
    eps = 1.0 if order in _CYCLIC else -1.0
    rot = Gf.Matrix4d(matrix).GetOrthonormalized(False)

    def M(r, c):
        # Column-vector element = transpose of the row-vector matrix.
        return rot[c][r]

    cosBeta = math.hypot(M(k, j), M(k, k))
    beta = math.atan2(-eps * M(k, i), cosBeta)
    if cosBeta > 1e-9:
        alpha = math.atan2(eps * M(k, j), M(k, k))
        gamma = math.atan2(eps * M(j, i), M(i, i))
    else:
        # Gimbal lock: alpha is free; keep the hint's and solve gamma from
        # the residual rotation about axis k.
        alpha = math.radians(hint[i]) if hint is not None else 0.0
        partial = _AxisRotation(i, math.degrees(alpha)) * \
            _AxisRotation(j, math.degrees(beta))
        residual = partial.GetInverse() * rot
        gamma = math.atan2(eps * residual[i][j], residual[i][i])

    first = [0.0, 0.0, 0.0]
    first[i], first[j], first[k] = (
        math.degrees(alpha), math.degrees(beta), math.degrees(gamma))
    if hint is None:
        return tuple(first)
    second = [0.0, 0.0, 0.0]
    second[i] = first[i] + 180.0
    second[j] = 180.0 - first[j]
    second[k] = first[k] + 180.0
    best = None
    for candidate in (first, second):
        unwrapped = [_Unwrap(c, h) for c, h in zip(candidate, hint)]
        distance = sum(abs(u - h) for u, h in zip(unwrapped, hint))
        if best is None or distance < best[0]:
            best = (distance, unwrapped)
    return tuple(best[1])


# ---------------------------------------------------------------------------
# Rig frames (asset space)
# ---------------------------------------------------------------------------

def IsRigXformable(prim):
    xformable = Tf.Type.FindByName("RigExecXformable")
    if xformable.isUnknown:
        return prim.GetTypeName() in ("RigExecControl", "RigExecJoint")
    return prim.IsA(xformable)


def FindRigRoot(prim):
    """The enclosing RigExecRoot, or None."""
    parent = prim.GetParent()
    while parent and not parent.IsPseudoRoot():
        if parent.GetTypeName() == "RigExecRoot":
            return parent
        parent = parent.GetParent()
    return None


def _FindParentXformable(prim, rigRoot):
    parent = prim.GetParent()
    while parent and parent != rigRoot and not parent.IsPseudoRoot():
        if IsRigXformable(parent):
            return parent
        parent = parent.GetParent()
    return None


def SolverPosedPaths(rigRoot):
    """
    Every prim a solver poses: the union of all rigExec:joints targets
    under the rig. The evaluator injects those joints' frames as value
    overrides (computations.cpp:223-227), so their avars are inert.
    """
    paths = set()
    for prim in Usd.PrimRange(rigRoot):
        rel = prim.GetRelationship("rigExec:joints")
        if rel:
            paths.update(rel.GetTargets())
    return paths


def ScalarAvar(prim, name, time, fallback):
    attr = prim.GetAttribute(name)
    if attr:
        value = attr.Get(time)
        if value is not None:
            return float(value)
    return fallback


def _MatrixAttr(prim, name, time):
    attr = prim.GetAttribute(name)
    if attr:
        value = attr.Get(time)
        if value is not None:
            return Gf.Matrix4d(value)
    return Gf.Matrix4d(1.0)


def RestLocal(prim, time):
    """compose(rest:t, rest:r) -- XYZ order, no scale, no spin."""
    return ComposeAvarMatrix(
        ScalarAvar(prim, REST_T[0], time, 0.0),
        ScalarAvar(prim, REST_T[1], time, 0.0),
        ScalarAvar(prim, REST_T[2], time, 0.0),
        1.0, 1.0, 1.0,
        ScalarAvar(prim, REST_R[0], time, 0.0),
        ScalarAvar(prim, REST_R[1], time, 0.0),
        ScalarAvar(prim, REST_R[2], time, 0.0),
        0.0, "XYZ")


def RestSpace(prim, time):
    """Mirror of _JointRestSpace: orthonormalize(restLocal * rest:space)."""
    rest = RestLocal(prim, time) * _MatrixAttr(prim, REST_SPACE, time)
    return rest.GetOrthonormalized(False)


def AvarsMatrix(prim, time):
    order = prim.GetAttribute(AVAR_ORDER)
    orderValue = order.Get(time) if order else None
    return ComposeAvarMatrix(
        ScalarAvar(prim, AVAR_T[0], time, 0.0),
        ScalarAvar(prim, AVAR_T[1], time, 0.0),
        ScalarAvar(prim, AVAR_T[2], time, 0.0),
        ScalarAvar(prim, AVAR_S[0], time, 1.0),
        ScalarAvar(prim, AVAR_S[1], time, 1.0),
        ScalarAvar(prim, AVAR_S[2], time, 1.0),
        ScalarAvar(prim, AVAR_R[0], time, 0.0),
        ScalarAvar(prim, AVAR_R[1], time, 0.0),
        ScalarAvar(prim, AVAR_R[2], time, 0.0),
        ScalarAvar(prim, AVAR_RSPIN, time, 0.0),
        orderValue or "XYZ")


class RigFrames(object):
    """
    Everything the gizmo needs about one RigExecXformable at one time,
    in ASSET space (multiply by assetToWorld for world):

      rest       orthonormal rest frame (the evaluator's computeRestFrame)
      posed      avars * P   (the evaluator's computePointFrame)
      P          rest * parentRest^-1 * parentPosed: the frame the avars
                 are expressed in -- Pose mode edits happen relative to it
      Q          rest:space * parentRest^-1 * parentPosed: the frame the
                 rest offsets are expressed in -- Pivot mode edits
      restLocal  compose(rest:t, rest:r)
      reason     "" when the prim is editable through its avars, else why
                 not (solver-posed, posed:space authority, no rig root)
    """

    def __init__(self, prim):
        self.prim = prim
        self.rigRoot = None
        self.reason = ""
        self.rest = Gf.Matrix4d(1.0)
        self.posed = Gf.Matrix4d(1.0)
        self.P = Gf.Matrix4d(1.0)
        self.Q = Gf.Matrix4d(1.0)
        self.restLocal = Gf.Matrix4d(1.0)
        self.parentRest = Gf.Matrix4d(1.0)
        self.parentPosed = Gf.Matrix4d(1.0)
        self.assetToWorld = Gf.Matrix4d(1.0)


def ComputeRigFrames(stage, prim, time, solverPosed=None):
    frames = RigFrames(prim)
    frames.rigRoot = FindRigRoot(prim)
    if frames.rigRoot is None:
        frames.reason = "%s is not under a RigExecRoot" % prim.GetName()
        return frames
    if solverPosed is None:
        solverPosed = SolverPosedPaths(frames.rigRoot)

    parent = _FindParentXformable(prim, frames.rigRoot)
    if parent is not None:
        parentFrames = ComputeRigFrames(stage, parent, time, solverPosed)
        if parentFrames.reason:
            frames.reason = "parent %s: %s" % (
                parent.GetName(), parentFrames.reason)
        frames.parentRest = parentFrames.rest
        frames.parentPosed = parentFrames.posed

    if prim.GetPath() in solverPosed:
        frames.reason = ("%s is posed by a solver (rigExec:joints); its "
                         "avars are ignored" % prim.GetName())
    posed = prim.GetAttribute(POSED_SPACE)
    if posed:
        if posed.HasAuthoredConnections():
            frames.reason = ("%s has a connected posed:space; its avars "
                             "are ignored" % prim.GetName())
        elif _MatrixAttr(prim, POSED_SPACE, time) != _IDENTITY:
            frames.reason = ("%s has an authored posed:space; its avars "
                             "are ignored" % prim.GetName())

    frames.restLocal = RestLocal(prim, time)
    frames.rest = RestSpace(prim, time)
    toParent = frames.parentRest.GetInverse() * frames.parentPosed
    frames.P = frames.rest * toParent
    frames.Q = _MatrixAttr(prim, REST_SPACE, time) * toParent
    frames.posed = AvarsMatrix(prim, time) * frames.P

    assetRoot = frames.rigRoot.GetParent()
    if assetRoot and not assetRoot.IsPseudoRoot():
        frames.assetToWorld = UsdGeom.XformCache(time)\
            .GetLocalToWorldTransform(assetRoot)
    return frames
