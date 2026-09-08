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
#           * parentRest
#   default = defaultOffsets * rest * parentRest^-1 * parentDefault
#   posed = avars * posedDefault * parentDefault^-1 * parentPosed
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
DEFAULT_T = ("default:tx", "default:ty", "default:tz")
DEFAULT_R = ("default:rx", "default:ry", "default:rz")
DEFAULT_SPACE = "default:space"
AVAR_DEFAULT_SPACE = "avars:defaultSpace"
POSED_DEFAULT_SPACE = "posed:defaultSpace"
PARENT_SPACE = "parent:space"
PARENT_DEFAULT_SPACE = "parent:defaultSpace"
AVAR_UNIT_SCALE = "avars:unitScaleFactor"
_COMPUTED_SPACES = (DEFAULT_SPACE, AVAR_DEFAULT_SPACE, POSED_DEFAULT_SPACE,
                    PARENT_SPACE, PARENT_DEFAULT_SPACE)

# The concrete RigExecVolumeWeight subclasses. Used only as the fallback
# for ReadsScaleAvars and IsRigXformable when the schema plugin is not
# registered and Tf cannot answer an IsA question.
VOLUME_WEIGHT_TYPE_NAMES = (
    "RigExecSphereWeight",
    "RigExecPlaneWeight",
    "RigExecCurveWeight",
)


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
        return prim.GetTypeName() in (
            ("RigExecControl", "RigExecJoint") + VOLUME_WEIGHT_TYPE_NAMES)
    return prim.IsA(xformable)


def ReadsScaleAvars(prim):
    """
    Whether the evaluator feeds this prim's avars:sx/sy/sz into its frame.

    False for RigExecVolumeWeight: computations.cpp registers it with
    readScaleAvars = false on purpose, because volume shape is owned
    exclusively by inputs:scaleX/Y/Z and transform-scale avars would be
    silent no-ops. Reading them here anyway would give a volume weight --
    and everything parented under one -- a replica frame the evaluator
    disagrees with, so the gizmo would draw and edit in the wrong place.
    """
    volume = Tf.Type.FindByName("RigExecVolumeWeight")
    if volume.isUnknown:
        return prim.GetTypeName() not in VOLUME_WEIGHT_TYPE_NAMES
    return not prim.IsA(volume)


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


# The relationship SolverPosedPaths reads and SolverPosedCache watches
# for invalidation. Named once so the two cannot drift apart.
SOLVER_JOINTS_REL = "rigExec:joints"


def SolverPosedPaths(rigRoot):
    """
    Every prim a solver poses: the union of all rigExec:joints targets
    under the rig. The evaluator injects those joints' frames as value
    overrides (computations.cpp:223-227), so their avars are inert.
    """
    paths = set()
    for prim in Usd.PrimRange(rigRoot):
        rel = prim.GetRelationship(SOLVER_JOINTS_REL)
        if rel:
            paths.update(rel.GetTargets())
    return paths


class SolverPosedCache(object):
    """
    SolverPosedPaths memoised per rig root.

    Every Refresh() of a rig target re-derives its frames, and that walks
    the WHOLE rig looking for rigExec:joints. Outside a drag the
    controller refreshes on every relevant stage notice, so without this
    an avar change costs a full traversal for an answer that cannot have
    moved.

    The set changes in two notice shapes, not one, and missing either
    leaves the memo handing out an editable target for a control a
    solver has taken over. Probed on this USD build:

      SetTargets on a NEW relationship     resync   /Rig/Ik.rigExec:joints
      SetTargets on an existing one        INFO     /Rig/Ik.rigExec:joints
      ClearTargets                         resync   /Rig/Ik.rigExec:joints
      a new prim carrying the relationship resync   /Rig/Other

    So retargeting -- the common case, and what a panel or a script
    does -- is info-only. InvalidateChanged covers it; InvalidateResynced
    covers the rest.

    Invalidation is the caller's job -- both methods on a notice, Clear
    when the stage is replaced -- because only the caller sees notices.
    """

    def __init__(self):
        self._byRoot = {}

    def For(self, rigRoot):
        """The memoised SolverPosedPaths(rigRoot); empty without a root."""
        if not rigRoot:
            return set()
        path = rigRoot.GetPath()
        paths = self._byRoot.get(path)
        if paths is None:
            paths = SolverPosedPaths(rigRoot)
            self._byRoot[path] = paths
        return paths

    def Clear(self):
        self._byRoot = {}

    def InvalidateResynced(self, resyncedPaths):
        """
        Drop every rig root a resynced path could have changed: one
        INSIDE the rig (a joints relationship was authored, a prim added
        or removed) and one AT or ABOVE it (the rig itself was replaced
        or its composition changed).
        """
        for root in list(self._byRoot):
            for path in resyncedPaths:
                prim = path.GetPrimPath()
                if prim.HasPrefix(root) or root.HasPrefix(prim):
                    del self._byRoot[root]
                    break

    def InvalidateChanged(self, changedPaths):
        """
        Drop every rig root whose solver joints were RE-TARGETED.

        SetTargets on a relationship that already has targets is an
        info-only change (see the class docstring's probe table), so a
        memo invalidated on resync alone would keep answering with the
        joints from before the edit. The name matched here is the one
        SolverPosedPaths reads, so the two cannot drift apart.
        """
        for root in list(self._byRoot):
            for path in changedPaths:
                if (path.IsPropertyPath()
                        and path.name == SOLVER_JOINTS_REL
                        and path.GetPrimPath().HasPrefix(root)):
                    del self._byRoot[root]
                    break


def NoticeAffectsTarget(resyncedPaths, changedPaths, targetPath,
                        rigRootPath=None):
    """
    Whether an ObjectsChanged notice can have moved the gizmo's target.

    True when any resynced or changed-info path IS the target prim or
    one of its ancestors (an ancestor's transform carries it), and -- for
    a rig target -- when any of them lies under `rigRootPath`, because
    the evaluator composes a control's frame from every avar above it
    and a solver can pose it from a joint anywhere else in the rig.

    A property path answers for the prim that owns it: an avar edit
    arrives as </Rig/Arm.avars:tx>, and what moved is /Rig/Arm.

    Without the filter every notice on the stage -- the volume weight
    panel, the curvenet panel, a timeline scrub -- costs a full rig walk,
    a resolveCamera() (which is NOT side-effect free, see gizmoUI._Camera)
    and a reprojection of the whole manipulator, for a target that
    usually did not change. A target-less controller is always affected:
    the notice may be the very edit that gives the focus prim a target.
    """
    if targetPath is None:
        return True
    for paths in (resyncedPaths, changedPaths):
        for path in paths:
            prim = path.GetPrimPath()
            if targetPath.HasPrefix(prim):
                return True
            if rigRootPath is not None and prim.HasPrefix(rigRootPath):
                return True
    return False


def ScalarAvar(prim, name, time, fallback):
    attr = prim.GetAttribute(name)
    visiting = set()
    while attr and attr.GetPath() not in visiting:
        visiting.add(attr.GetPath())
        connections = attr.GetConnections()
        if len(connections) != 1:
            break
        source = prim.GetStage().GetAttributeAtPath(connections[0])
        if not source:
            break
        attr = source
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
    """
    Mirror of _JointRestSpace: the local rest carried into the parent's
    rest frame, orthonormalize(restLocal * rest:space) * parentRest.
    """
    rest = RestLocal(prim, time) * _MatrixAttr(prim, REST_SPACE, time)
    rest = rest.GetOrthonormalized(False)
    parent = _FindParentXformable(prim, FindRigRoot(prim))
    return rest * RestSpace(parent, time) if parent else rest


def DefaultLocal(prim, time):
    return ComposeAvarMatrix(
        *(tuple(ScalarAvar(prim, n, time, 0.0) for n in DEFAULT_T)
          + (1.0, 1.0, 1.0)
          + tuple(ScalarAvar(prim, n, time, 0.0) for n in DEFAULT_R)
          + (0.0, "XYZ")))


class _PoseAuthorityError(ValueError):
    """
    A parent:space that cannot be resolved because the POSE authority
    upstream is not the avars -- a solver-posed ancestor, or one with an
    authored or connected posed:space.

    Distinguished from every other ValueError raised here (a cyclic
    space connection, a singular matrix) because it says nothing about
    the rest side: rest:t/r upstream is still authored data, still read,
    and a rest frame reads no namespace ancestor at all. It refuses Pose
    only. See _ComputeRigFrames.
    """


def _ComputedSpace(prim, name, time, solverPosed, visiting=None, frameCache=None):
    """Evaluate the schema space expressions, including connected identity."""
    visiting = set() if visiting is None else visiting
    path = prim.GetPath().AppendProperty(name)
    if path in visiting:
        raise ValueError("cyclic space connection at %s" % path)
    visiting.add(path)
    try:
        attr = prim.GetAttribute(name)
        connections = attr.GetConnections() if attr else []
        if len(connections) == 1:
            source = prim.GetStage().GetAttributeAtPath(connections[0])
            if source:
                return _ComputedSpace(source.GetPrim(), source.GetName(),
                                      time, solverPosed, visiting, frameCache)
        raw = _MatrixAttr(prim, name, time)
        if name not in _COMPUTED_SPACES or not IsRigXformable(prim):
            return raw
        if raw != _IDENTITY:
            return raw
        if name == AVAR_DEFAULT_SPACE:
            return _ComputedSpace(prim, DEFAULT_SPACE, time, solverPosed, visiting, frameCache)
        if name == POSED_DEFAULT_SPACE:
            return _ComputedSpace(prim, AVAR_DEFAULT_SPACE, time, solverPosed, visiting, frameCache)
        parent = _FindParentXformable(prim, FindRigRoot(prim))
        if name == PARENT_DEFAULT_SPACE:
            return (_ComputedSpace(parent, DEFAULT_SPACE, time, solverPosed, visiting, frameCache)
                    if parent else Gf.Matrix4d(1.0))
        if name == PARENT_SPACE:
            if not parent:
                return Gf.Matrix4d(1.0)
            parentFrames = ComputeRigFrames(prim.GetStage(), parent, time, solverPosed, frameCache)
            if parentFrames.reason:
                raise _PoseAuthorityError("space source %s: %s" % (
                    parent.GetPath(), parentFrames.reason))
            return parentFrames.posed
        parentRest = RestSpace(parent, time) if parent else Gf.Matrix4d(1.0)
        return (DefaultLocal(prim, time) * RestSpace(prim, time)
                * parentRest.GetInverse()
                * _ComputedSpace(prim, PARENT_DEFAULT_SPACE, time, solverPosed, visiting, frameCache))
    finally:
        visiting.remove(path)


def AvarsMatrix(prim, time):
    order = prim.GetAttribute(AVAR_ORDER)
    orderValue = order.Get(time) if order else None
    # Mirrors _ComputeXformablePointFrame's readScaleAvars argument: a
    # volume weight substitutes identity scale rather than reading avars.
    scaled = ReadsScaleAvars(prim)
    units = ScalarAvar(prim, AVAR_UNIT_SCALE, time, 1.0)
    return ComposeAvarMatrix(
        ScalarAvar(prim, AVAR_T[0], time, 0.0) * units,
        ScalarAvar(prim, AVAR_T[1], time, 0.0) * units,
        ScalarAvar(prim, AVAR_T[2], time, 0.0) * units,
        ScalarAvar(prim, AVAR_S[0], time, 1.0) if scaled else 1.0,
        ScalarAvar(prim, AVAR_S[1], time, 1.0) if scaled else 1.0,
        ScalarAvar(prim, AVAR_S[2], time, 1.0) if scaled else 1.0,
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
      P          posedDefault * parentDefault^-1 * parentPosed: the frame the avars
                 are expressed in -- Pose mode edits happen relative to it
      Qrest      orthonormalize(rest:space): the frame the rest offsets
                 are ACTUALLY expressed in, and what Pivot mode edits
                 relative to. computeRestFrame composes rest:t/rest:r
                 against rest:space and reads no namespace ancestor
                 (computations.cpp:200-219), so a rest frame is absolute
                 in asset space and owes nothing to the parent's pose.
                 restLocal * Qrest is therefore `rest` itself, and the
                 pivot manipulator sits on the frame its own channels
                 define -- unmoved by an animated ancestor, and unmoved
                 by a solver posing this joint.
      Q          orthonormalize(rest:space) * parentRest^-1 * parentPosed:
                 the frame the rest offsets are carried into for POSE
                 purposes. Pivot mode does NOT use it. Rest editing acts
                 in the bind frame, before local default offsets: with
                 identity default channels, restLocal * Q reproduces P,
                 while a default offset moves the avar origin and leaves
                 the rest pivot in the bind frame. Raw rest:space is
                 orthonormalized here, matching the evaluator's rigid
                 rest-frame convention -- an approximation only for a
                 rest:space carrying scale or shear, which the evaluator
                 throws away too.
      restLocal  compose(rest:t, rest:r)
      default    computed default:space, including connected overrides
      unitScale  distance per translation avar unit; inverted on drag writes
      reason     "" when the prim is editable through its avars, else why
                 not (solver-posed, posed:space authority, no rig root)
      pivotReason
                 "" when the prim is editable through its REST offsets,
                 else why not. Much narrower than `reason`: everything
                 that only redirects the POSE authority away from the
                 avars -- a solver, an authored or connected posed:space,
                 an ancestor with either -- leaves rest:t/r authored,
                 read and honoured, so it is no reason to refuse a pivot.
                 Only a prim outside a RigExecRoot is refused here, and
                 for a hard reason: the frames are never computed, so
                 there is nothing to draw on.
    """

    def __init__(self, prim):
        self.prim = prim
        self.rigRoot = None
        self.reason = ""
        self.pivotReason = ""
        self.rest = Gf.Matrix4d(1.0)
        self.posed = Gf.Matrix4d(1.0)
        self.P = Gf.Matrix4d(1.0)
        self.Q = Gf.Matrix4d(1.0)
        self.Qrest = Gf.Matrix4d(1.0)
        self.restLocal = Gf.Matrix4d(1.0)
        self.parentRest = Gf.Matrix4d(1.0)
        self.parentPosed = Gf.Matrix4d(1.0)
        self.parentDefault = Gf.Matrix4d(1.0)
        self.default = Gf.Matrix4d(1.0)
        self.unitScale = 1.0
        self.assetToWorld = Gf.Matrix4d(1.0)


def _RefuseBothModes(frames, message):
    """
    Record a cause that refuses Pose AND Pivot.

    Pivot normally answers only to `pivotReason`, because a solver or a
    posed:space redirects the POSE authority and leaves rest:t/r live.
    That reasoning needs frames to exist: every site that gives up before
    building Qrest/restLocal, and every prim whose data is broken rather
    than merely solver-driven, refuses both modes through here.
    """
    frames.reason = message
    frames.pivotReason = message
    return frames


_publishedControlFrameReader = None


def SetPublishedControlFrameReader(reader):
    """Install the host's snapshot reader: (stage, path, time) -> matrix/None."""
    global _publishedControlFrameReader
    _publishedControlFrameReader = reader


def ComputeRigFrames(stage, prim, time, solverPosed=None, _frameCache=None):
    _frameCache = {} if _frameCache is None else _frameCache
    if prim.GetPath() in _frameCache:
        cached = _frameCache[prim.GetPath()]
        if cached is None:
            raise ValueError("cyclic posed-space dependency at %s" % prim.GetPath())
        return cached
    _frameCache[prim.GetPath()] = None
    try:
        frames = _ComputeRigFrames(stage, prim, time, solverPosed, _frameCache)
        _frameCache[prim.GetPath()] = frames
        return frames
    except ValueError:
        del _frameCache[prim.GetPath()]
        raise


def _ComputeRigFrames(stage, prim, time, solverPosed, _frameCache):
    frames = RigFrames(prim)
    frames.rigRoot = FindRigRoot(prim)
    if frames.rigRoot is None:
        # Returns before any frame is computed, so rest/Qrest/assetToWorld
        # stay identity and a pivot built from them would draw at the
        # world origin.
        return _RefuseBothModes(
            frames, "%s is not under a RigExecRoot" % prim.GetName())
    if prim.GetTypeName() == "RigExecCurvenetAdjustment":
        # The adjustment scope depends on preceding point revisions. Its
        # cached native frame is the authority; authored USD alone cannot
        # reconstruct it without evaluating those revisions a second time.
        matrix = (_publishedControlFrameReader(stage, prim.GetPath(), time)
                  if _publishedControlFrameReader is not None else None)
        if matrix is None:
            return _RefuseBothModes(
                frames,
                "Activate RigExec at this frame to edit the curvenet adjustment")
        avars = AvarsMatrix(prim, time)
        if not all(math.isfinite(avars[r][c]) for r in range(4) for c in range(4)) \
                or abs(avars.GetDeterminant()) < 1e-12:
            return _RefuseBothModes(
                frames, "The adjustment's avar matrix is not invertible")
        posedAttr = prim.GetAttribute(POSED_SPACE)
        if posedAttr and (posedAttr.HasAuthoredConnections()
                          or _MatrixAttr(prim, POSED_SPACE, time) != _IDENTITY):
            return _RefuseBothModes(
                frames, "posed:space drives this adjustment; edit its source")
        frames.posed = Gf.Matrix4d(matrix)
        frames.P = avars.GetInverse() * frames.posed
        frames.default = frames.P
        frames.Q = frames.P
        frames.rest = frames.P
        frames.unitScale = ScalarAvar(prim, AVAR_UNIT_SCALE, time, 1.0)
        if not math.isfinite(frames.unitScale) or abs(frames.unitScale) < 1e-12:
            _RefuseBothModes(
                frames,
                "The adjustment has a zero or non-finite translation unit scale")
        frames.pivotReason = "Curvenet adjustment pivots follow the preceding deformation"
        assetRoot = frames.rigRoot.GetParent()
        if assetRoot and not assetRoot.IsPseudoRoot():
            frames.assetToWorld = UsdGeom.XformCache(time).GetLocalToWorldTransform(assetRoot)
        return frames
    if solverPosed is None:
        solverPosed = SolverPosedPaths(frames.rigRoot)

    parent = _FindParentXformable(prim, frames.rigRoot)
    if parent is not None:
        parentFrames = ComputeRigFrames(stage, parent, time, solverPosed, _frameCache)
        parentSpace = prim.GetAttribute(PARENT_SPACE)
        explicitParent = parentSpace and (parentSpace.HasAuthoredConnections()
                                         or _MatrixAttr(prim, PARENT_SPACE, time) != _IDENTITY)
        if parentFrames.reason and not explicitParent:
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
    # Built here, before anything on the pose side can fail: Pivot needs
    # Qrest and restLocal and nothing else (RigPivotTarget._Qw), so a
    # pose-side give-up must not be allowed to return past this.
    frames.Qrest = _MatrixAttr(prim, REST_SPACE, time)\
        .GetOrthonormalized(False)
    try:
        frames.default = _ComputedSpace(prim, DEFAULT_SPACE, time, solverPosed, frameCache=_frameCache)
        frames.parentDefault = _ComputedSpace(prim, PARENT_DEFAULT_SPACE, time, solverPosed, frameCache=_frameCache)
        effectiveDefault = _ComputedSpace(prim, POSED_DEFAULT_SPACE, time, solverPosed, frameCache=_frameCache)
    except ValueError as error:
        # The default-space family. Pivot answers to these too -- the
        # loop below refuses a pivot on an authored default space -- so a
        # broken one refuses both modes.
        return _RefuseBothModes(frames, str(error))
    try:
        frames.parentPosed = _ComputedSpace(prim, PARENT_SPACE, time, solverPosed, frameCache=_frameCache)
    except _PoseAuthorityError as error:
        # An ancestor whose pose comes from somewhere other than its
        # avars: a solver, or an authored posed:space. That is a POSE
        # reason. The rest frames are already built above, and a rest
        # frame reads no namespace ancestor (computations.cpp:200-219),
        # so the pivot below such an ancestor is untouched by it -- which
        # is the case that matters, since a TwoBoneIk MEASURES its bone
        # lengths from these rests. Falls through with P/Q/posed built
        # from the parent's stale posed frame; Pose is refused, and
        # nothing else reads them.
        frames.reason = frames.reason or str(error)
    except ValueError as error:
        return _RefuseBothModes(frames, str(error))
    frames.unitScale = ScalarAvar(prim, AVAR_UNIT_SCALE, time, 1.0)
    if not math.isfinite(frames.unitScale) or abs(frames.unitScale) < 1e-12:
        # Only RigPoseTarget divides by it, but a prim carrying a broken
        # unit scale is not one to author against in either mode.
        _RefuseBothModes(
            frames,
            "%s has a zero or non-finite translation unit scale" % prim.GetName())
    toParent = frames.parentDefault.GetInverse() * frames.parentPosed
    frames.P = effectiveDefault * toParent
    # Rest editing still acts in the bind frame before local default
    # offsets. No parentRest^-1 here: rest:t/r are parent-relative now, so
    # restLocal is already in the parent's frame and dividing it out again
    # would land the pivot an ancestor offset away from the joint.
    frames.Q = frames.Qrest * frames.parentDefault * toParent
    for name in (DEFAULT_SPACE, AVAR_DEFAULT_SPACE, POSED_DEFAULT_SPACE):
        attr = prim.GetAttribute(name)
        if attr and (attr.HasAuthoredConnections()
                     or _MatrixAttr(prim, name, time) != _IDENTITY):
            frames.pivotReason = ("%s.%s selects a default space independently "
                                  "of rest; edit that source for pivot changes"
                                  % (prim.GetName(), name))
    frames.posed = AvarsMatrix(prim, time) * frames.P

    assetRoot = frames.rigRoot.GetParent()
    if assetRoot and not assetRoot.IsPseudoRoot():
        frames.assetToWorld = UsdGeom.XformCache(time)\
            .GetLocalToWorldTransform(assetRoot)
    _frameCache[prim.GetPath()] = frames
    return frames


# ---------------------------------------------------------------------------
# Writing values: animation (spline knot at the frame) or default
# ---------------------------------------------------------------------------

WRITE_ANIMATION = "animation"
WRITE_DEFAULT = "default"
CHANNELS_POSE = "pose"
CHANNELS_PIVOT = "pivot"


def SetAnimated(attr, value, time):
    """
    volumeWeightUI.SetAtTime's rule (volumeWeightUI.py:68-111), restated
    here so this module stays Qt-free (volumeWeightUI imports Qt at
    module scope): default time or existing time samples or a
    spline-incapable type -> Set(value, time); otherwise a
    curve-interpolated knot on the attribute's spline.

    The knot itself comes from graphModel.AuthorKnot, so a gizmo drag
    and a graph-editor insert produce the SAME key -- Maya's default new
    key, AutoEase on both tangents (graph editor design spec 1.2) -- and
    a key the artist has already shaped keeps its tangents.
    """
    from pxr import Ts
    if attr.GetVariability() == Sdf.VariabilityUniform:
        attr.Set(value)
        return
    valueType = attr.GetTypeName().type
    if (time.IsDefault() or attr.GetNumTimeSamples() > 0 or
            not Ts.Spline.IsSupportedValueType(valueType)):
        attr.Set(value, time)
        return
    # Imported inside the function: graphModel imports THIS module at
    # module scope, so a top-level import here would be a cycle.
    import graphModel
    spline = graphModel.SplineFor(attr)
    graphModel.AuthorKnot(spline, time.GetValue(), value)
    attr.SetSpline(spline)


class Writer(object):
    """
    Where a gizmo value lands. WRITE_ANIMATION authors at `time` through
    SetAnimated; WRITE_DEFAULT authors the default and records a warning
    for every attribute whose spline or time samples will outrank it
    (the default is then invisible in the viewport, and the toolbar says
    so rather than letting the drag look broken).
    """

    def __init__(self, stage, time, mode):
        self.stage = stage
        self.time = time
        self.mode = mode
        self._warnings = []

    def Set(self, attr, value):
        if self.mode == WRITE_DEFAULT:
            if attr.HasSpline() or attr.GetNumTimeSamples() > 0:
                message = ("%s: the default is outranked by its %s" % (
                    attr.GetPath(),
                    "spline" if attr.HasSpline() else "time samples"))
                if message not in self._warnings:
                    self._warnings.append(message)
            attr.Set(value)
        else:
            SetAnimated(attr, value, self.time)

    def Warnings(self):
        return list(self._warnings)


# ---------------------------------------------------------------------------
# Edit targets
# ---------------------------------------------------------------------------

def _Linear(matrix):
    """The 3x3 part as a 4x4 with zero translation."""
    m = Gf.Matrix4d(matrix)
    m.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
    return m


def _RotationOnly(matrix):
    return _Linear(matrix).GetOrthonormalized(False)


def _WorldRotation(axis, degrees):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(axis).GetNormalized(), degrees))
    return m


def _FrameAt(matrix, origin):
    """`matrix`'s rotation, moved to `origin`: a drawable handle frame."""
    m = _RotationOnly(matrix)
    m.SetTranslateOnly(Gf.Vec3d(origin))
    return m


def _SnapValue(value, step):
    """
    `value` rounded to the nearest multiple of `step` (Maya's Step Snap),
    or `value` unchanged when `step` is None or 0.

    Halves go AWAY from zero. Python's round() sends them to even, which
    makes a slow drag stick unevenly -- 0.5 snapping to 0 but 1.5 to 2 --
    and the asymmetry is visible on a step grid.
    """
    if not step:
        return value
    steps = math.floor(abs(value) / abs(step) + 0.5)
    return math.copysign(steps * abs(step), value)


def _SnapTranslation(base, delta, step, absolute):
    """
    base + delta with Step Snap applied where the channel values live.

    Relative (Maya's `J` hold) quantises the DELTA, so a drag advances in
    whole steps from wherever it started; absolute quantises the RESULT,
    so the channel value lands on the grid however the drag started.
    Snapping the WORLD delta instead would put the channels off the grid
    whenever the channel frame is rotated, which is the whole reason this
    lives here and not in the controller. No UI binds the absolute form
    any more: the world grid goes through gizmoSnap.GridPoint and a plain
    ApplyTranslate with no keyword (snapping design, sections 1-2), and
    this parameter stays only because it is still correct and tested.
    """
    if not step:
        return [base[i] + delta[i] for i in range(3)]
    if absolute:
        return [_SnapValue(base[i] + delta[i], step) for i in range(3)]
    return [base[i] + _SnapValue(delta[i], step) for i in range(3)]


def _ScaleAxes(axisIndex):
    """
    Which scale channels a drag touches: None for all three (the centre
    cube), an int for one (an axis handle), or ANY iterable of ints for a
    planar handle -- (0, 1) is Maya's XY square.

    The iterable case is duck-typed rather than a list of accepted
    classes: the controller hands over whatever its handle description
    carries, and an isinstance whitelist turned a perfectly good
    generator or sequence-like object into a TypeError deep inside a
    drag. Only the iteration is guarded, so a genuinely bad element
    still raises where it is written rather than being read as a
    single-axis index.
    """
    if axisIndex is None:
        return (0, 1, 2)
    try:
        axes = tuple(axisIndex)
    except TypeError:
        return (int(axisIndex),)
    return tuple(int(i) for i in axes)


def _RotationVector(matrix):
    """A rotation as axis * radians, the 3 free parameters of SO(3)."""
    rotation = matrix.ExtractRotation()
    axis = rotation.GetAxis()
    angle = math.radians(rotation.GetAngle())
    return [axis[i] * angle for i in range(3)]


def _FromRotationVector(vector):
    length = math.sqrt(sum(v * v for v in vector))
    if length < 1e-15:
        return Gf.Matrix4d(1.0)
    return _WorldRotation(Gf.Vec3d(*vector) / length, math.degrees(length))


# Newton refinement budget for SolveWorldRotation. The closed-form first
# guess is already exact for a rigid channel frame, so these steps only
# ever run for a sheared one. The tolerance is a residual in radians and
# cannot usefully go below ~1e-8: _RotationVector reads the angle back
# through Gf.Rotation, whose acos loses half the mantissa near identity.
_ROTATION_SOLVE_STEPS = 6
_ROTATION_SOLVE_TOLERANCE = 1e-7
_ROTATION_SOLVE_STEP = 1e-6


def SolveWorldRotation(base, channel, worldAxis, degrees):
    """
    The channel rotation R' whose ORTHONORMALIZED world frame is the
    current one turned by `degrees` about `worldAxis`, where the world
    frame is `base * channel` and only `base` may be edited.

    `channel` is the full linear part that follows the rotation channels
    (spin * P, Q, or the parent xform), NOT orthonormalized. When it is
    rigid -- the ordinary case -- the answer is the closed form of design
    spec section 2, R' = base * Cr * Rw * Cr^-1, and the loop below exits
    on its first residual check without touching it.

    When an ancestor carries non-uniform scale the channel frame is
    sheared and that conjugate is no longer a rotation, so no closed form
    exists. What the viewport shows is Gf's orthonormalization of a
    sheared matrix, and that operator is measurably neither Gram-Schmidt
    nor the polar factor, so it cannot be inverted algebraically. It is
    exactly invariant under a positive diagonal on the left and exactly
    equivariant under a rotation on the right, which is all the two lines
    above rely on. Newton on the three rotation parameters recovers the
    answer instead. The best iterate is kept rather than the last one, so
    the two escapes below (the step budget, and a singular Jacobian)
    return the closest orientation actually found, never a diverged one.
    A rigid REFLECTION is not one of those cases: the closed form is
    exact there too, since M C Rw C^-1 stays a rotation.

    The left invariance is why the avar scale S is absent here: uniform
    or not, a POSITIVE scale cannot change the drawn orientation. A
    negative one is a reflection and does; the gizmo turns the
    unreflected frame in that case.
    """
    channel = _Linear(channel)
    base = _Linear(base)
    rigid = channel.GetOrthonormalized(False)
    turn = _WorldRotation(worldAxis, degrees)
    target = (base * channel).GetOrthonormalized(False) * turn
    result = base * rigid * turn * rigid.GetInverse()

    def Residual(candidate):
        current = (candidate * channel).GetOrthonormalized(False)
        return _RotationVector(current.GetInverse() * target)

    best = None
    for _ in range(_ROTATION_SOLVE_STEPS):
        residual = Residual(result)
        length = math.sqrt(sum(v * v for v in residual))
        if best is None or length < best[0]:
            best = (length, Gf.Matrix4d(result))
        if length < _ROTATION_SOLVE_TOLERANCE:
            break
        columns = []
        for axis in range(3):
            nudge = [0.0, 0.0, 0.0]
            nudge[axis] = _ROTATION_SOLVE_STEP
            moved = Residual(result * _FromRotationVector(nudge))
            columns.append([(moved[r] - residual[r]) / _ROTATION_SOLVE_STEP
                            for r in range(3)])
        jacobian = Gf.Matrix3d(columns[0][0], columns[1][0], columns[2][0],
                               columns[0][1], columns[1][1], columns[2][1],
                               columns[0][2], columns[1][2], columns[2][2])
        if abs(jacobian.GetDeterminant()) < 1e-9:
            break
        inverse = jacobian.GetInverse()
        step = [sum(inverse[r][c] * -residual[c] for c in range(3))
                for r in range(3)]
        result = (result * _FromRotationVector(step))\
            .GetOrthonormalized(False)
    return best[1]


def GimbalAxes(order, rx, ry, rz, channelRotation):
    """
    The world unit axes of the X / Y / Z gimbal rings (design spec 8.3
    "Rotate Axis: Gimbal"), returned indexed by AXIS: [0] turns rx, [1]
    turns ry, [2] turns rz, whatever `order` is.

    For order (i, j, k) applied i first the composition is R = Ri*Rj*Rk
    (row-vector). Moving angle j alone gives R' = Ri*Rj*dj*Rk =
    R * (Rk^-1 dj Rk), and conjugation maps M^-1 d(x) M to d(x*M), so in
    CHANNEL space the j ring turns about e_j * Rk; likewise the k ring
    about e_k and the i ring about e_i * Rj * Rk (= e_i * R, since a
    rotation fixes its own axis). `channelRotation` carries those into
    world -- pass the target's GimbalFrame(), which is the frame the
    ROTATE channels compose in (it includes avars:rspin, which sits
    between the Euler product and P; ChannelFrame() does not).

    These axes are for DRAWING the rings and measuring the swept screen
    angle. To apply the drag, Gimbal mode calls the target's
    ApplyRotateChannel(axisIndex, degrees), which moves exactly the one
    channel whatever the channel frame is; every other Rotate Axis mode
    calls ApplyRotate(worldAxis, degrees). Feeding these axes back into
    ApplyRotate agrees with ApplyRotateChannel only while the channel
    frame is rigid -- see ApplyRotateChannel and SolveWorldRotation.
    """
    order = _NormalizeOrder(order)
    angles = (rx, ry, rz)
    i, j, k = ("XYZ".index(axis) for axis in order)
    rk = _AxisRotation(k, angles[k])
    rj = _AxisRotation(j, angles[j])
    axes = [None, None, None]
    axes[k] = _AXES[k]
    axes[j] = rk.TransformDir(_AXES[j])
    axes[i] = (rj * rk).TransformDir(_AXES[i])
    world = _RotationOnly(channelRotation)
    return [world.TransformDir(axis).GetNormalized() for axis in axes]


class Target(object):
    """
    One prim being edited by the gizmo. Subclasses know which attributes
    they own and how a WORLD-space delta maps back onto them. Every
    Apply* is computed from the values captured by BeginDrag(), never
    incrementally, so a drag cannot accumulate rounding drift and an
    aborted drag has one well-defined state to return to.

    Three frames, all orthonormal and all translated to the gizmo origin,
    feed the Axis Orientation option (design spec 8.2):

      ObjectFrame()   Maya "Object": the target's own posed orientation
      ChannelFrame()  Maya "Parent": the space the channels are written
                      in (P or Q for a rig prim, the parent xform
                      otherwise)
      GimbalFrame()   the space the ROTATE channels compose in; equal to
                      ChannelFrame() except on a rig pose target, where
                      avars:rspin sits between the Euler product and P

    World orientation needs no frame: the controller uses the identity.
    """

    kind = ""
    supportsTranslate = True
    supportsRotate = True
    supportsScale = True
    supportsPreserveChildren = False
    preserveChildrenReason = ""

    def __init__(self, stage, prim, writer):
        self.stage = stage
        self.prim = prim
        self.writer = writer
        self.label = prim.GetName()
        self.preserveChildren = False
        # Children a Preserve Children drag will NOT hold still, one
        # sentence each, for the toolbar to show: silently leaving a
        # child behind looks like a bug from the viewport.
        self.skippedChildren = []
        self._base = {}

    @property
    def time(self):
        return self.writer.time

    def Advisory(self):
        """
        A note about an edit that will succeed and change nothing
        visible, or "" when there is none. Sibling of the
        `the default is outranked by its spline` warning: the drag is
        not refused, it is explained.
        """
        return ""

    def Refresh(self):
        """Re-read the stage; call after a frame change or an undo."""

    def RigRootPath(self):
        """
        The enclosing RigExecRoot's path, or None for a plain xform.

        The controller needs it to decide whether a stage notice can
        have moved this target: a rig control's frame is composed from
        every avar above it and can be posed by a solver reading a joint
        elsewhere in the rig, so anything under the root is relevant
        (NoticeAffectsTarget), while an xform only follows its own
        ancestors.
        """
        return None

    def GizmoMatrix(self):
        raise NotImplementedError

    def ObjectFrame(self):
        return self.GizmoMatrix()

    def ChannelFrame(self):
        raise NotImplementedError

    def GimbalFrame(self):
        return self.ChannelFrame()

    def RotationState(self):
        """
        (rotation order, [rx, ry, rz]) for the Euler channels this target
        writes, or None when it has none. Read live from the stage, so
        the controller can draw gimbal rings outside a drag.
        """
        return None

    def SetPreserveChildren(self, enabled):
        """
        Maya "Preserve Children". A target that cannot honour it stays
        off however often it is asked, so a stale toolbar checkbox can
        never make a drag silently skip the compensation.
        """
        self.preserveChildren = bool(enabled) and \
            self.supportsPreserveChildren

    def AttributePaths(self):
        """
        Every attribute this target may write, for the undo recorder.

        STATEFUL on the xform pose target: with Preserve Children on it
        also returns the compensated children's channels. Call it AFTER
        SetPreserveChildren() and BEFORE the recorder's Begin(), or an
        undo will move the parent back and leave the children behind.
        """
        raise NotImplementedError

    def BeginDrag(self):
        self.Refresh()
        self._base = {name: ScalarAvar(self.prim, name, self.time, fallback)
                      for name, fallback in self._ScalarChannels()}

    def _ScalarChannels(self):
        return []

    def _Write(self, name, value):
        self.writer.Set(self.prim.GetAttribute(name), float(value))

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        """
        Move by a WORLD delta, optionally with Maya's Step Snap.

        `snapStep` quantises in CHANNEL space, where the values that get
        written live; see _SnapTranslation for why that is not the same
        as quantising the world delta, and for what `snapAbsolute` picks
        between. Passing no snapStep leaves the delta exactly as given.
        """
        raise NotImplementedError

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        """
        Turn the drawn world frame by `degrees` about a WORLD axis.

        `snapStep` quantises the ANGLE, relative to the drag base, so a
        snapped drag advances in whole steps (Maya's `J` hold, default
        step 15 degrees).
        """
        raise NotImplementedError

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        """
        Add `degrees` to ONE Euler channel (0 = X, 1 = Y, 2 = Z), the
        others held at their drag base. `snapStep` quantises the angle
        exactly as it does for ApplyRotate.

        This is Maya's Gimbal mode (design spec 8.3, "each ring changes
        exactly one Euler channel"), and it needs its own entry point
        because ApplyRotate cannot deliver it: ApplyRotate takes a world
        axis and promises the drawn frame turns by the dragged angle,
        and under a sheared channel frame (an ancestor with non-uniform
        scale) the rotation that does that moves all three channels.
        Here the guarantee is exact whatever the channel frame is,
        because the Euler product isolates the channel by construction.
        """
        raise NotImplementedError

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        """
        Multiply the scale channels by `factor`.

        `axisIndex` picks them: None for all three (the centre cube), an
        int for one (an axis handle), or a tuple or list for a planar
        handle -- (0, 1) is Maya's XY square. `snapStep` quantises the
        RESULTING value of each channel the drag touches, before the
        evaluator's 1e-4 floor. Channels the drag does not touch keep
        their authored value rather than being nudged onto the grid.
        """
        raise NotImplementedError


class _RigTarget(Target):
    """Shared frame bookkeeping for the two RigExecXformable modes."""

    # A control's children are placed by the evaluator from the parent's
    # published frame, so there is nothing to re-author: compensating
    # them here would fight the rig on the next evaluation.
    preserveChildrenReason = ("children of a rig control are evaluated by "
                              "the rig")

    def __init__(self, stage, prim, writer, solverPosed=None):
        Target.__init__(self, stage, prim, writer)
        # A SolverPosedCache, or None to walk the rig on every Refresh.
        self._solverPosed = solverPosed
        self.frames = None
        self.Refresh()

    def Refresh(self):
        posed = None
        if self._solverPosed is not None:
            posed = self._solverPosed.For(FindRigRoot(self.prim))
        self.frames = ComputeRigFrames(self.stage, self.prim, self.time,
                                       posed)

    def RigRootPath(self):
        root = self.frames.rigRoot if self.frames is not None else None
        return root.GetPath() if root else None

    def _WriteVector(self, names, values):
        with Sdf.ChangeBlock():
            for name, value in zip(names, values):
                self._Write(name, value)


class RigPoseTarget(_RigTarget):
    """Edits avars:t/r/s relative to P (see RigFrames)."""

    kind = "rig-pose"

    def _ScalarChannels(self):
        return ([(n, 0.0) for n in AVAR_T] + [(n, 0.0) for n in AVAR_R]
                + [(n, 1.0) for n in AVAR_S] + [(AVAR_RSPIN, 0.0)])

    def _Order(self):
        attr = self.prim.GetAttribute(AVAR_ORDER)
        value = attr.Get(self.time) if attr else None
        return _NormalizeOrder(value)

    def AttributePaths(self):
        return [self.prim.GetPath().AppendProperty(n)
                for n in AVAR_T + AVAR_R + AVAR_S]

    def GizmoMatrix(self):
        world = self.frames.posed * self.frames.assetToWorld
        return world.GetOrthonormalized(False)

    def _Pw(self):
        return self.frames.P * self.frames.assetToWorld

    def ChannelFrame(self):
        return _FrameAt(self._Pw(), self.GizmoMatrix().ExtractTranslation())

    def GimbalFrame(self):
        spin = _AxisRotation(0, ScalarAvar(
            self.prim, AVAR_RSPIN, self.time, 0.0))
        return _FrameAt(spin * self._Pw(),
                        self.GizmoMatrix().ExtractTranslation())

    def RotationState(self):
        return (self._Order(),
                [ScalarAvar(self.prim, n, self.time, 0.0) for n in AVAR_R])

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        local = _Linear(self._Pw()).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta)) / self.frames.unitScale
        base = [self._base[n] for n in AVAR_T]
        self._WriteVector(AVAR_T, _SnapTranslation(
            base, local, snapStep, snapAbsolute))

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        base = [self._base[n] for n in AVAR_R]
        order = self._Order()
        # World linear = S * R * spin * linear(P*assetToWorld); only R is
        # ours to move, so spin * P is the channel frame it turns inside.
        channel = _AxisRotation(0, self._base[AVAR_RSPIN]) * self._Pw()
        rNew = SolveWorldRotation(RotationFromEuler(order, *base), channel,
                                  worldAxis, _SnapValue(degrees, snapStep))
        self._WriteVector(AVAR_R, DecomposeEuler(rNew, order, hint=base))

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        values = [self._base[n] for n in AVAR_R]
        values[axisIndex] = values[axisIndex] + _SnapValue(degrees, snapStep)
        self._WriteVector(AVAR_R, values)

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        axes = _ScaleAxes(axisIndex)
        values = []
        for i, name in enumerate(AVAR_S):
            value = self._base[name]
            if i in axes:
                value = _SnapValue(value * factor, snapStep)
            values.append(NormalizeAvarScale(value))
        self._WriteVector(AVAR_S, values)


class RigPivotTarget(_RigTarget):
    """Edits rest:t/r relative to Q (see RigFrames); no scale."""

    kind = "rig-pivot"
    supportsScale = False
    # Rest offsets are parent-relative, so a pivot drag carries the whole
    # subtree with it. Preserve Children is therefore meaningful here in a
    # way it is not for a rig POSE edit: the compensation is authored into
    # each child's own rest channels, which the evaluator does read, rather
    # than into xformOps, which it ignores.
    supportsPreserveChildren = True
    preserveChildrenReason = ""

    def _ChildProviders(self):
        """
        Immediate child frame providers.

        Immediate only: rest is parent-relative, so a grandchild rides on
        a child this already holds still, and compensating it too would
        double-count the correction.
        """
        return [child for child in self.prim.GetChildren()
                if IsRigXformable(child)]

    def _ScalarChannels(self):
        return [(n, 0.0) for n in REST_T] + [(n, 0.0) for n in REST_R]

    def AttributePaths(self):
        paths = [self.prim.GetPath().AppendProperty(n)
                 for n in REST_T + REST_R]
        self.skippedChildren = []
        if self.preserveChildren:
            for child in self._ChildProviders():
                paths.extend(child.GetPath().AppendProperty(n)
                             for n in REST_T + REST_R)
        return paths

    def BeginDrag(self):
        _RigTarget.BeginDrag(self)
        # The world rest each child must be put back onto. Captured before
        # the first write, so a multi-event drag compensates against the
        # start of the drag rather than accumulating per event.
        self._preserved = [(child, RestSpace(child, self.time))
                           for child in self._ChildProviders()] \
            if self.preserveChildren else []

    def _RestoreChildren(self):
        """
        Put every captured child back on its starting world rest.

        restLocal(child) = restWorld(child) * restWorld(self)^-1, then the
        rest:t/r scalars that produce it against the child's own untouched
        rest:space -- so the bind basis stays the author's and only the
        offset channels move.
        """
        preserved = getattr(self, "_preserved", [])
        if not preserved:
            return
        inverse = RestSpace(self.prim, self.time).GetInverse()
        with Sdf.ChangeBlock():
            for child, worldRest in preserved:
                scalars = (worldRest * inverse
                           * _MatrixAttr(child, REST_SPACE,
                                         self.time).GetInverse())
                scalars = scalars.GetOrthonormalized(False)
                hint = [ScalarAvar(child, n, self.time, 0.0)
                        for n in REST_R]
                for name, value in zip(REST_T,
                                       scalars.ExtractTranslation()):
                    self.writer.Set(child.GetAttribute(name), float(value))
                for name, value in zip(
                        REST_R, DecomposeEuler(scalars, "XYZ", hint=hint)):
                    self.writer.Set(child.GetAttribute(name), float(value))

    def Advisory(self):
        # A bone length can no longer be authored -- rigExec:upperLength /
        # rigExec:lowerLength are computed from the joints' rests on every
        # evaluation and Compile rejects an authored opinion. So a pivot
        # drag always reaches the solve, and there is nothing to warn about.
        return ""

    def _Qw(self):
        # Qrest AND the parent's REST, not Q: rest:t/r are expressed
        # against rest:space carried into the parent's rest frame
        # (computations.cpp:230), so the parent's rest belongs in the frame
        # this drag inverts through and draws on -- but its POSE still does
        # not, which is what keeps a pivot still under an animated ancestor.
        return self.frames.Qrest * self.frames.parentRest \
            * self.frames.assetToWorld

    def GizmoMatrix(self):
        return (self.frames.restLocal * self._Qw()).GetOrthonormalized(False)

    def ChannelFrame(self):
        return _FrameAt(self._Qw(), self.GizmoMatrix().ExtractTranslation())

    def RotationState(self):
        # RestLocal composes rest:r in XYZ with no spin (computations.cpp
        # :196-215), so the pivot rings are always XYZ rings.
        return ("XYZ",
                [ScalarAvar(self.prim, n, self.time, 0.0) for n in REST_R])

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        local = _Linear(self._Qw()).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        base = [self._base[n] for n in REST_T]
        self._WriteVector(REST_T, _SnapTranslation(
            base, local, snapStep, snapAbsolute))
        self._RestoreChildren()

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        base = [self._base[n] for n in REST_R]
        rNew = SolveWorldRotation(RotationFromEuler("XYZ", *base),
                                  self._Qw(), worldAxis,
                                  _SnapValue(degrees, snapStep))
        self._WriteVector(REST_R, DecomposeEuler(rNew, "XYZ", hint=base))
        self._RestoreChildren()

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        values = [self._base[n] for n in REST_R]
        values[axisIndex] = values[axisIndex] + _SnapValue(degrees, snapStep)
        self._WriteVector(REST_R, values)
        self._RestoreChildren()

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        pass


_XFORM_ORDER_NAMES = {
    UsdGeom.XformCommonAPI.RotationOrderXYZ: "XYZ",
    UsdGeom.XformCommonAPI.RotationOrderXZY: "XZY",
    UsdGeom.XformCommonAPI.RotationOrderYXZ: "YXZ",
    UsdGeom.XformCommonAPI.RotationOrderYZX: "YZX",
    UsdGeom.XformCommonAPI.RotationOrderZXY: "ZXY",
    UsdGeom.XformCommonAPI.RotationOrderZYX: "ZYX",
}

_ZERO3F = Gf.Vec3f(0.0, 0.0, 0.0)


def _XformCommonMatrix(t, p, r, s, order):
    """
    The local matrix of an XformCommonAPI op stack, composed from its
    five components WITHOUT reading the stage.

    Preserve Children compensates the children against the parent's NEW
    world transform, and the write that produces it now shares one
    Sdf.ChangeBlock with the children's own writes -- reading a
    recomposed value back from inside the block that authored it is the
    documented Sdf.ChangeBlock hazard. Composing it here instead is what
    lets a whole Apply* be a single change notification.

    Probed against GetLocalTransformation and covered by the "xform
    local matrix" group in test_gizmo_math.py: the stack
    [translate, pivot, rotate, scale, !invert!pivot] composes row-vector
    as pivot^-1 * S * R * pivot * T -- ops apply to points last-to-first
    -- and R is RotationFromEuler in the op's own rotation order. The
    two pure translations collapse into one because pivot and translate
    commute.
    """
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(-Gf.Vec3d(p))
    scale = Gf.Matrix4d(1.0)
    scale.SetScale(Gf.Vec3d(s))
    m = m * scale * RotationFromEuler(order, r[0], r[1], r[2])
    back = Gf.Matrix4d(1.0)
    back.SetTranslate(Gf.Vec3d(p) + Gf.Vec3d(t))
    return m * back


class _PreservedChild(object):
    """
    One child of a Preserve Children drag: its world transform at the
    press, plus the values needed to re-author it afterwards.

    The Euler hint is the child's angles at the press, so the
    compensation stays on the same branch as whatever an animator
    already authored instead of jumping to an equivalent triple.
    """

    def __init__(self, prim, api, vectors, cache):
        self.prim = prim
        self.api = api
        self.order = vectors[4]
        self.angles = [float(v) for v in vectors[1]]
        self.world = cache.GetLocalToWorldTransform(prim)

    def AttributePaths(self):
        prefix = self.prim.GetPath()
        rotateName = "xformOp:rotate" + _XFORM_ORDER_NAMES[self.order]
        return [prefix.AppendProperty(n) for n in (
            "xformOp:translate", rotateName, "xformOp:scale",
            "xformOpOrder")]


class _XformTarget(Target):
    """
    A plain UsdGeomXformable through UsdGeomXformCommonAPI, whose op
    stack is [translate, pivot, rotate, scale, !invert!pivot]: ops apply
    to points last-to-first, so scale then rotation happen about the
    pivot and the translate is in PARENT space -- which is why world
    deltas are mapped through the parent's transform, not the prim's.
    """

    def __init__(self, stage, prim, writer):
        Target.__init__(self, stage, prim, writer)
        self.api = UsdGeom.XformCommonAPI(prim)
        self.vectors = None
        self.parentWorld = Gf.Matrix4d(1.0)
        self._preserved = []
        self.Refresh()

    def Refresh(self):
        cache = UsdGeom.XformCache(self.time)
        # An op stack beginning with !resetXformStack! ignores its
        # ancestors, and XformCommonAPI accepts one. GetParentToWorld-
        # Transform is just the parent's CTM (xformCache.cpp:37-41) and
        # does not consult the flag, so without this the gizmo would be
        # drawn and the children compensated against a parent frame the
        # prim does not actually sit in.
        if UsdGeom.Xformable(self.prim).GetResetXformStack():
            self.parentWorld = Gf.Matrix4d(1.0)
        else:
            self.parentWorld = cache.GetParentToWorldTransform(self.prim)
        self.vectors = self.api.GetXformVectors(self.time)

    def BeginDrag(self):
        self.Refresh()
        t, r, s, p, order = self.vectors
        self._base = {"t": Gf.Vec3d(t), "r": Gf.Vec3f(r), "s": Gf.Vec3f(s),
                      "p": Gf.Vec3f(p), "order": order}
        self._preserved = self._PreserveCandidates()

    def ChannelFrame(self):
        return _FrameAt(self.parentWorld,
                        self.GizmoMatrix().ExtractTranslation())

    def _Ops(self, *which):
        """
        Create ONLY the ops named in `which`, and answer all five slots.

        CreateXformOps always returns a 5-tuple in fixed slots --
        translate, pivot, rotate, scale, !invert!pivot -- filling the
        slots it was not asked for with invalid ops, and it leaves an
        already-authored op alone (probed: asking for translate, rotate
        and scale on a prim that already has a pivot keeps the pivot and
        still reports it in slot 1).

        Each caller asks for the one op it is about to write, so a drag
        adds no scene description beyond what it changes. Asking for all
        four instead gave a prim that had none a zero pivot pair and two
        more ops on the first translate, none of which the user asked
        for and all of which then had to be undone.
        """
        return self.api.CreateXformOps(self._base["order"], *which)

    def _PreserveCandidates(self):
        """
        The children a Preserve Children drag can hold still, and as a
        side effect the sentences in self.skippedChildren explaining the
        ones it cannot.

        A RigExec prim is never a candidate even though the schema makes
        it a UsdGeomXformable with an empty (so XformCommonAPI-
        compatible) op stack: RigExecXformable inherits Boundable
        (libs/rigExecSchema/schema.usda:197-198). The evaluator places
        those prims from their avars and never reads an xformOp, so
        authoring one would not hold the child still AND would break the
        project rule that RigExec prims carry no xformOps. FindRigRoot
        catches plain xforms grouped inside a rig too, whose children
        are the evaluator's business either way.

        The rest of the filter: an XformCommonAPI-compatible stack, a
        zero pivot (a non-zero pivot makes the local decomposition of
        the compensated matrix ambiguous) and a non-reflecting linear
        part (row lengths cannot recover a negative scale, so a mirrored
        child would silently come back unmirrored).
        """
        self.skippedChildren = []
        if not self.preserveChildren:
            return []
        cache = UsdGeom.XformCache(self.time)
        found = []
        for child in self.prim.GetChildren():
            name = child.GetName()
            if IsRigXformable(child) or FindRigRoot(child) is not None:
                self.skippedChildren.append(
                    "%s is placed by the rig, not by xformOps" % name)
                continue
            if not child.IsA(UsdGeom.Xformable):
                continue
            api = UsdGeom.XformCommonAPI(child)
            if not api:
                self.skippedChildren.append(
                    "%s: xformOp stack is not XformCommonAPI-compatible"
                    % name)
                continue
            vectors = api.GetXformVectors(self.time)
            if Gf.Vec3f(vectors[3]) != _ZERO3F:
                self.skippedChildren.append(
                    "%s has a non-zero pivot" % name)
                continue
            local = UsdGeom.Xformable(child).GetLocalTransformation(self.time)
            if _Linear(local).GetDeterminant() <= 0.0:
                self.skippedChildren.append(
                    "%s is mirrored or collapsed" % name)
                continue
            found.append(_PreservedChild(child, api, vectors, cache))
        return found

    def _ChildCompensation(self, parentWorld):
        """
        [(child, translation, angles, scale)] putting every recorded
        child back on its captured world matrix against the parent's NEW
        world transform: childLocal = childWorld * newParent^-1, split
        into row lengths (scale), the orthonormalized linear part
        (rotation) and the translation row -- exactly the decomposition
        XformCommonAPI recomposes from.

        Computation only, authoring nothing, so the caller can put every
        write of one Apply* -- the target's op and all 3N child channels
        -- inside a single Sdf.ChangeBlock.
        """
        if not self._preserved:
            return []
        if abs(_Linear(parentWorld).GetDeterminant()) < 1e-12:
            # A collapsed parent (a scale drag through zero) has no
            # inverse; Gf answers with FLT_MAX rather than raising, so
            # compensating here would author garbage on the children.
            # Leave them where they are and let the next event recover.
            return []
        inverse = parentWorld.GetInverse()
        writes = []
        for entry in self._preserved:
            local = entry.world * inverse
            linear = _Linear(local)
            scale = [Gf.Vec3d(linear[r][0], linear[r][1],
                              linear[r][2]).GetLength() for r in range(3)]
            angles = DecomposeEuler(linear, _XFORM_ORDER_NAMES[entry.order],
                                    hint=entry.angles)
            writes.append((entry, local.ExtractTranslation(),
                           Gf.Vec3f(*angles), Gf.Vec3f(*scale)))
        return writes

    def _WriteOp(self, opType, slot, value, t=None, r=None, s=None, p=None):
        """
        Author one XformCommonAPI op and every Preserve Children
        compensation, with all the VALUES in one Sdf.ChangeBlock, so a
        mouse-move costs the stage one change notification (design spec
        3.3 and 4.2).

        The RigExec evaluator republishes synchronously on
        ObjectsChanged, so the unbatched version of this cost 1 + 3N
        recompositions per event with N preserved children, all thrown
        away but the last.

        `t`/`r`/`s`/`p` name the component this call changes; the others
        come from the drag base. They are needed rather than read back
        because the children are compensated against the parent's new
        world transform, which _XformCommonMatrix composes instead of
        the stage recomposing it -- see that function for why.

        Op CREATION stays outside the block, deliberately. AddXformOp
        validates the op it just created with a composed stage query
        (xformable.cpp:200-207), and inside a change block a prim whose
        specs come only from a reference cannot see the spec that was
        just authored -- its index has not been rescanned -- so the op
        reads back undefined, AddXformOp posts a coding error, and the
        Apply* raises before it has written anything. Creating the ops
        first costs one extra notification on the first event of a drag
        that has an op to create, and nothing on any event after it,
        which is once per drag rather than once per mouse-move.
        """
        children = []
        if self._preserved:
            base = self._base
            local = _XformCommonMatrix(
                base["t"] if t is None else t,
                base["p"] if p is None else p,
                base["r"] if r is None else r,
                base["s"] if s is None else s,
                _XFORM_ORDER_NAMES[base["order"]])
            children = self._ChildCompensation(local * self.parentWorld)
        api = UsdGeom.XformCommonAPI
        target = self._Ops(opType)[slot].GetAttr()
        childOps = [entry.api.CreateXformOps(
            entry.order, api.OpTranslate, api.OpRotate, api.OpScale)
            for entry, _, _, _ in children]
        with Sdf.ChangeBlock():
            self.writer.Set(target, value)
            for ops, (_, translation, angles, scale) in zip(childOps,
                                                            children):
                self.writer.Set(ops[0].GetAttr(), translation)
                self.writer.Set(ops[2].GetAttr(), angles)
                self.writer.Set(ops[3].GetAttr(), scale)

    def AttributePaths(self):
        prefix = self.prim.GetPath()
        _, _, _, _, order = self.vectors
        rotateName = "xformOp:rotate" + _XFORM_ORDER_NAMES[order]
        paths = [prefix.AppendProperty(n) for n in (
            "xformOp:translate", "xformOp:translate:pivot", rotateName,
            "xformOp:scale", "xformOpOrder")]
        # The compensated children are re-authored by the same drag, so
        # they belong to the same undo entry as the target itself.
        for entry in self._PreserveCandidates():
            paths.extend(entry.AttributePaths())
        return paths


class XformPoseTarget(_XformTarget):
    kind = "xform-pose"
    supportsPreserveChildren = True

    def GizmoMatrix(self):
        """
        The object's orientation, drawn at the PIVOT (design spec 8.2).

        Maya centres all three manipulators on the point the rotate and
        scale ops turn about, which for this op stack is `pivot +
        translate` in parent space. Drawing at the prim's local origin
        instead would put the rotate rings off the point the object
        turns about, and the ring centre would orbit during the drag.
        """
        local = UsdGeom.Xformable(self.prim).GetLocalTransformation(self.time)
        m = (local * self.parentWorld).GetOrthonormalized(False)
        t, _, _, p, _ = self.vectors
        m.SetTranslateOnly(self.parentWorld.Transform(Gf.Vec3d(p) + t))
        return m

    def RotationState(self):
        # Read through the API rather than the cached self.vectors: the
        # base class promises a live answer, and the controller asks for
        # one between the writes of a drag to redraw the rings.
        _, r, _, _, order = self.api.GetXformVectors(self.time)
        return (_XFORM_ORDER_NAMES[order], [float(v) for v in r])

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        local = _Linear(self.parentWorld).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        base = self._base["t"]
        values = _SnapTranslation([base[i] for i in range(3)], local,
                                  snapStep, snapAbsolute)
        t = Gf.Vec3d(*values)
        self._WriteOp(UsdGeom.XformCommonAPI.OpTranslate, 0, t, t=t)

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        order = _XFORM_ORDER_NAMES[self._base["order"]]
        base = [float(v) for v in self._base["r"]]
        rNew = SolveWorldRotation(RotationFromEuler(order, *base),
                                  self.parentWorld, worldAxis,
                                  _SnapValue(degrees, snapStep))
        angles = Gf.Vec3f(*DecomposeEuler(rNew, order, hint=base))
        self._WriteOp(UsdGeom.XformCommonAPI.OpRotate, 2, angles, r=angles)

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        values = [float(v) for v in self._base["r"]]
        values[axisIndex] = values[axisIndex] + _SnapValue(degrees, snapStep)
        angles = Gf.Vec3f(*values)
        self._WriteOp(UsdGeom.XformCommonAPI.OpRotate, 2, angles, r=angles)

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        axes = _ScaleAxes(axisIndex)
        s = Gf.Vec3f(self._base["s"])
        for i in range(3):
            if i in axes:
                s[i] = _SnapValue(s[i] * factor, snapStep)
        self._WriteOp(UsdGeom.XformCommonAPI.OpScale, 3, s, s=s)


class XformPivotTarget(_XformTarget):
    kind = "xform-pivot"
    supportsRotate = False
    supportsScale = False
    # Children DO move with a pivot edit whenever the prim carries a
    # rotate or a scale, since the pivot is part of the local matrix.
    # The brief still puts Preserve Children out of scope for pivot
    # mode (design spec 1.2: a pivot edit is uncompensated by design,
    # like moving a Maya pivot without compensation), so say that
    # rather than claiming the children hold still on their own.
    preserveChildrenReason = ("pivot edits are not compensated, on this "
                              "prim or its children (spec 1.2)")

    def GizmoMatrix(self):
        t, r, s, p, order = self.vectors
        m = _RotationOnly(self.parentWorld)
        m.SetTranslateOnly(self.parentWorld.Transform(Gf.Vec3d(p) + t))
        return m

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        local = _Linear(self.parentWorld).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        # Both sides are rounded to float32 before the sum, so the
        # unsnapped result is the same value the plain Vec3f addition
        # this replaced produced.
        base = [float(v) for v in self._base["p"]]
        delta = [float(v) for v in Gf.Vec3f(local)]
        p = Gf.Vec3f(*_SnapTranslation(base, delta, snapStep, snapAbsolute))
        self._WriteOp(UsdGeom.XformCommonAPI.OpPivot, 1, p, p=p)

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        pass

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        pass

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        pass


def _ConnectedAvar(prim, names):
    for name in names:
        attr = prim.GetAttribute(name)
        if attr and attr.HasAuthoredConnections():
            return name
    return None


# The only rig prims the gizmo edits (design spec assumption 1.3). Volume
# weights also inherit RigExecXformable, but they carry no scale avars
# and have their own panel, so they are declined by concrete type rather
# than through IsRigXformable.
RIG_TARGET_TYPE_NAMES = ("RigExecControl", "RigExecJoint")


def _IsRigTargetType(prim):
    for name in RIG_TARGET_TYPE_NAMES:
        schemaType = Tf.Type.FindByName(name)
        if schemaType.isUnknown:
            # No schema plugin registered: fall back to the type name,
            # matching IsRigXformable's own fallback.
            if prim.GetTypeName() == name:
                return True
        elif prim.IsA(schemaType):
            return True
    return False


def MakeTarget(stage, prim, channels, writer, solverPosed=None):
    """
    (target, "") or (None, reason) for usdview's focus prim.

    `solverPosed` is an optional SolverPosedCache, shared with the
    target so a Refresh() reuses the rig walk instead of repeating it.
    """
    if not prim or not prim.IsValid():
        return None, "nothing selected"
    if IsRigXformable(prim):
        if not _IsRigTargetType(prim):
            if not ReadsScaleAvars(prim):
                return None, ("%s is a volume weight; use the Volume "
                              "Weight panel" % prim.GetName())
            return None, "%s (%s) is not a control or a joint" % (
                prim.GetName(), prim.GetTypeName() or "untyped")
        posed = (solverPosed.For(FindRigRoot(prim))
                 if solverPosed is not None else None)
        frames = ComputeRigFrames(stage, prim, writer.time, posed)
        # Pose answers to `reason`. Pivot edits rest:t/r, which no solver
        # and no posed:space takes away -- a TwoBoneIk measures its bone
        # lengths FROM the bound joints' rest frames -- so it answers to
        # `pivotReason` alone: the causes that leave the frames unusable,
        # a curvenet adjustment, and a default space that selects the
        # pivot independently of rest.
        blocking = (frames.pivotReason if channels == CHANNELS_PIVOT
                    else frames.reason)
        if blocking:
            return None, blocking
        names = (AVAR_T + AVAR_R + AVAR_S if channels == CHANNELS_POSE
                 else REST_T + REST_R)
        connected = _ConnectedAvar(prim, names)
        if connected:
            return None, "%s.%s is connected; edit its source instead" % (
                prim.GetName(), connected)
        if channels == CHANNELS_PIVOT:
            return RigPivotTarget(stage, prim, writer, solverPosed), ""
        return RigPoseTarget(stage, prim, writer, solverPosed), ""
    if prim.IsA(UsdGeom.Xformable):
        if not UsdGeom.XformCommonAPI(prim):
            return None, ("%s: xformOp stack is not XformCommonAPI-"
                          "compatible" % prim.GetName())
        if channels == CHANNELS_PIVOT:
            return XformPivotTarget(stage, prim, writer), ""
        return XformPoseTarget(stage, prim, writer), ""
    return None, "%s (%s) has no transform to edit" % (
        prim.GetName(), prim.GetTypeName() or "untyped")
