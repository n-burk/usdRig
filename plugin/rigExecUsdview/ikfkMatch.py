"""
IK/FK matching: pose one half of a limb onto what the other half is doing.

Qt-free. A limb is found from the rig itself: a RigExecBlendPointFrames
whose `inputs:weight` is connected to a control's avar (the switch), with a
RigExecFkChain on one input and a RigExecTwoBoneIk on the other. Nothing
here knows the biped's names.

Two actions, kept separate so either can be used alone:

    Match(...)   values for the INACTIVE half's controls that reproduce the
                 limb's current joints;
    Switch(...)  the value that hands the joints to that half.

A menu's "Switch to IK / FK" applies both as one edit (see Plan).

Every offset is measured on the rig, never chosen: an FK control's offset
from its joint, the IK effector's from the end joint, and the pole's
distance from the middle joint are all read at the limb's rest pose (every
limb channel at identity, the switch on the half being measured). A
control's channels are solved against the parent frame the evaluator
reports for it (posed = avars * P), so spaces, rotation orders and
`avars:rotationSign` need no special cases.

Conventions: USD row vectors (a point maps as p * M, a child's world frame
is local * parent); frames are asset space, as rigexec.Pose reports them.
"""

import math

from pxr import Gf, Sdf, Usd

import gizmoMath

FK_CHAIN = "RigExecFkChain"
TWO_BONE_IK = "RigExecTwoBoneIk"
BLEND = "RigExecBlendPointFrames"
CONTROL = "RigExecControl"
RIG_ROOT = "RigExecRoot"

# The channels a match may write: translate and rotate. Scale is never
# written -- an FK scale shears everything below it, so a stretched limb is
# matched with translation along the bone, the way the rig lengthens FK.
_TRANSLATE = ("tx", "ty", "tz")
_ROTATE = ("rx", "ry", "rz")
_IDENTITY = {"tx": 0.0, "ty": 0.0, "tz": 0.0, "rx": 0.0, "ry": 0.0,
             "rz": 0.0, "rspin": 0.0, "sx": 1.0, "sy": 1.0, "sz": 1.0}
# A limb straighter than this (the middle joint's distance from the line
# through the ends, as a fraction of the limb's length) has no bend plane
# to put the pole on.
_STRAIGHT = 1e-4
# A limb whose switch control carries these stretches and pins (the
# solver's stretchPolicy softDistance): a switch carries the bone lengths
# across in stretchTop / stretchBottom, as the FK chain lengthens by them.
_STRETCH_TOP = "avars:stretchTop"
_STRETCH_BOTTOM = "avars:stretchBottom"
_PIN = "avars:pvPin"
_TWIST = "avars:twist"
_STRETCH = "avars:stretch"
_SOFT = "avars:softStretch"


class Limb(object):
    """One switchable limb, found from its blend."""

    def __init__(self, blend, switchControl, switchAttr, ikValue, joints,
                 fkControls, ikControl, effector, pole, rigRoot,
                 ikSolver=None):
        self.blend = blend                  # Sdf.Path
        self.switchControl = switchControl  # Sdf.Path
        self.switchAttr = switchAttr        # "avars:ikfk"
        self.ikValue = ikValue              # switch value that selects IK
        self.joints = joints                # [Sdf.Path] x 3, root to end
        self.fkControls = fkControls        # [Sdf.Path] x 3
        self.ikControl = ikControl          # Sdf.Path, the animator's IK
        self.effector = effector            # Sdf.Path, the solver target
        self.pole = pole                    # Sdf.Path or None
        self.rigRoot = rigRoot              # Sdf.Path
        self.ikSolver = ikSolver            # Sdf.Path of the two-bone IK

    @property
    def fkValue(self):
        return 1.0 - self.ikValue

    @property
    def switchPath(self):
        return self.switchControl.AppendProperty(self.switchAttr)

    def Controls(self):
        """Every control this limb's switch concerns."""
        paths = [self.switchControl] + list(self.fkControls) + \
            [self.ikControl]
        if self.pole is not None:
            paths.append(self.pole)
        return paths

    def Stretches(self, stage):
        """Whether this limb stretches and pins: its switch control has
        the stretch channels the solver reads."""
        prim = stage.GetPrimAtPath(self.switchControl)
        return bool(prim and prim.GetAttribute(_STRETCH_TOP) and
                    prim.GetAttribute(_STRETCH_BOTTOM))

    def Channel(self, name):
        return self.switchControl.AppendProperty(name)

    def IsIk(self, stage, time, current=None):
        """Whether the IK half drives the joints now (the switch is nearer
        the IK value)."""
        if current and self.switchPath in current:
            value = float(current[self.switchPath])
        else:
            value = _Get(stage, self.switchPath, time, self.fkValue)
        return abs(value - self.ikValue) < abs(value - self.fkValue)

    def __repr__(self):
        return "<Limb %s>" % self.switchControl.name


# -- discovery ------------------------------------------------------------

def _Targets(prim, name):
    rel = prim.GetRelationship(name)
    return list(rel.GetTargets()) if rel else []


def _RigRoot(prim):
    walk = prim
    while walk and walk.GetPath() != Sdf.Path.absoluteRootPath:
        if walk.GetTypeName() == RIG_ROOT:
            return walk.GetPath()
        walk = walk.GetParent()
    return None


def _IkControl(effector, root):
    """The control the animator moves to place the IK effector: the
    outermost control at or above it that is not also above the solver's
    root. The effector hangs under it, sometimes through pivots that are
    not controls (a foot's roll stack); the controls above the root -- the
    rig's masters -- carry the whole limb and are not its IK control."""
    found = None
    walk = effector
    while walk and walk.GetPath() != Sdf.Path.absoluteRootPath:
        if root is not None and root.HasPrefix(walk.GetPath()):
            break
        if walk.GetTypeName() == CONTROL:
            found = walk
        walk = walk.GetParent()
    return found


def FindLimbs(stage):
    """Every switchable limb on the stage."""
    limbs = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != BLEND:
            continue
        weight = prim.GetAttribute("inputs:weight")
        sources = weight.GetConnections() if weight else []
        inputs = [_Targets(prim, "rigExec:inputA"),
                  _Targets(prim, "rigExec:inputB")]
        if len(sources) != 1 or not all(len(t) == 1 for t in inputs):
            continue
        solvers = [stage.GetPrimAtPath(t[0]) for t in inputs]
        kinds = [s.GetTypeName() if s else None for s in solvers]
        if sorted(kinds) != sorted([FK_CHAIN, TWO_BONE_IK]):
            continue
        fk = solvers[kinds.index(FK_CHAIN)]
        ik = solvers[kinds.index(TWO_BONE_IK)]
        # The blend's weight selects input B at 1.
        ikValue = 1.0 if kinds[1] == TWO_BONE_IK else 0.0
        joints = _Targets(prim, "rigExec:joints")
        fkControls = _Targets(fk, "rigExec:controls")
        effectors = _Targets(ik, "rigExec:effectorControl")
        poles = _Targets(ik, "rigExec:poleControl")
        roots = _Targets(ik, "rigExec:rootControl")
        if len(joints) != 3 or len(fkControls) != 3 or len(effectors) != 1:
            continue
        effector = stage.GetPrimAtPath(effectors[0])
        ikControl = _IkControl(effector, roots[0] if roots else None)             if effector else None
        if ikControl is None:
            continue
        source = sources[0]
        limbs.append(Limb(
            prim.GetPath(), source.GetPrimPath(), source.name, ikValue,
            joints, fkControls, ikControl.GetPath(), effectors[0],
            poles[0] if poles else None, _RigRoot(prim), ik.GetPath()))
    return limbs


def LimbFor(limbs, path):
    """The limb a selected prim belongs to: its switch control, an FK or IK
    control, the pole, or anything under the IK control (a foot's roll
    controls)."""
    path = Sdf.Path(str(path))
    for limb in limbs:
        if path in limb.Controls() or path.HasPrefix(limb.ikControl):
            return limb
    return None


# -- frames and channels --------------------------------------------------

def _Get(stage, attrPath, time, fallback):
    attr = stage.GetAttributeAtPath(attrPath)
    value = attr.Get(time) if attr else None
    return float(value) if value is not None else fallback


def _Matrix(values):
    return Gf.Matrix4d(*[values[i * 4:i * 4 + 4] for i in range(4)])


def JointFrame(pose, path):
    return _Matrix(pose.joint_frame(str(path), True).to_matrix4())


def ControlFrame(pose, path):
    return _Matrix(pose.control_frame(str(path)).to_matrix4())


def _Frame(pose, path):
    """A control's or joint's posed frame, whichever the pose has."""
    try:
        return ControlFrame(pose, path)
    except KeyError:
        return JointFrame(pose, path)


class _Channels(object):
    """One control's channel values at a time, and its avar matrix."""

    def __init__(self, stage, path, time, overrides=None):
        prim = stage.GetPrimAtPath(path)
        self.path = path
        self.prim = prim
        self.order = str(prim.GetAttribute("avars:rotationOrder").Get()
                         or "XYZ")
        self.sign = gizmoMath.RotationSign(prim)
        self.values = {}
        for name, fallback in _IDENTITY.items():
            attrPath = path.AppendProperty("avars:" + name)
            if overrides is not None and attrPath in overrides:
                self.values[name] = float(overrides[attrPath])
            else:
                self.values[name] = _Get(stage, attrPath, time, fallback)

    def Effective(self):
        """Rotation angles as the rig applies them (raw * sign)."""
        return tuple(self.values[r] * s for r, s in zip(_ROTATE, self.sign))

    def Matrix(self, values=None):
        v = dict(self.values, **(values or {}))
        rx, ry, rz = (v[r] * s for r, s in zip(_ROTATE, self.sign))
        return gizmoMath.ComposeAvarMatrix(
            v["tx"], v["ty"], v["tz"], v["sx"], v["sy"], v["sz"],
            rx, ry, rz, v["rspin"] * self.sign[0], self.order)

    def Solve(self, target, parent):
        """Translate and rotate values that put this control's posed frame
        on `target`, with `parent` the frame its avars compose onto."""
        local = target * parent.GetInverse()
        translate = local.ExtractTranslation()
        # local = S * R * Rspin * T: take the translation off, then the
        # scale (which a match never writes) and spin (likewise) from
        # either side of the rotation.
        linear = Gf.Matrix4d(local)
        linear.SetTranslateOnly(Gf.Vec3d(0.0))
        scale = Gf.Matrix4d(1.0)
        scale.SetScale(Gf.Vec3d(
            gizmoMath.NormalizeAvarScale(self.values["sx"]),
            gizmoMath.NormalizeAvarScale(self.values["sy"]),
            gizmoMath.NormalizeAvarScale(self.values["sz"])))
        spin = Gf.Matrix4d(1.0)
        if self.values["rspin"]:
            spin.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0),
                                       self.values["rspin"] * self.sign[0]))
        rotation = scale.GetInverse() * linear * spin.GetInverse()
        effective = gizmoMath.DecomposeEuler(rotation, self.order,
                                             hint=self.Effective())
        solved = {t: float(translate[i]) for i, t in enumerate(_TRANSLATE)}
        for r, angle, sign in zip(_ROTATE, effective, self.sign):
            solved[r] = float(angle * sign)
        return solved

    def SolveTranslate(self, point, parent):
        """Translate values that put this control's origin on `point`; its
        rotation and scale stay as they are."""
        local = parent.GetInverse().Transform(point)
        return {t: float(local[i]) for i, t in enumerate(_TRANSLATE)}


def _Writable(stage, path, names, written):
    """The channels of `names` this control has and the rig does not write
    or drive."""
    prim = stage.GetPrimAtPath(path)
    keep = []
    for name in names:
        attr = prim.GetAttribute("avars:" + name) if prim else None
        if not attr or attr.GetPath() in written or \
                attr.HasAuthoredConnections():
            continue
        keep.append(name)
    return keep


def RigWritten(stage):
    """Attribute paths a mover writes: overriding one would freeze the rig,
    so a match never does."""
    written = set()
    for prim in stage.Traverse():
        rel = prim.GetRelationship("rigExec:moves")
        if rel:
            written.update(rel.GetTargets())
    return written


# -- rest offsets ---------------------------------------------------------

class RestOffsets(object):
    """The limb's fixed relationships, measured at its rest pose."""

    def __init__(self, fk, effector, poleDistance, lengths=None):
        self.fk = fk                    # [control * joint^-1] x 3
        self.effector = effector        # end joint * effector^-1
        self.poleDistance = poleDistance
        self.lengths = lengths          # (upper, lower) rest bone lengths


def MeasureRest(limb, stage, evaluate, time):
    """Every limb channel at identity, the switch on IK (the effector and
    pole relationships are IK's) -- FK controls ride their joints in either
    half, which phase 0 measured, so one rest pose serves both."""
    overrides = {}
    for path in [limb.ikControl] + list(limb.fkControls) + \
            ([limb.pole] if limb.pole is not None else []):
        prim = stage.GetPrimAtPath(path)
        for name, value in _IDENTITY.items():
            attr = prim.GetAttribute("avars:" + name)
            if attr and not attr.HasAuthoredConnections():
                overrides[attr.GetPath()] = value
    overrides[limb.switchPath] = limb.ikValue
    if limb.Stretches(stage):
        # The rest is the unstretched, unpinned limb.
        overrides[limb.Channel(_STRETCH_TOP)] = 1.0
        overrides[limb.Channel(_STRETCH_BOTTOM)] = 1.0
        overrides[limb.Channel(_PIN)] = 0.0
        if stage.GetAttributeAtPath(limb.Channel(_TWIST)):
            overrides[limb.Channel(_TWIST)] = 0.0
    pose = evaluate(overrides, time)
    fkOverrides = dict(overrides)
    fkOverrides[limb.switchPath] = limb.fkValue
    fkPose = evaluate(fkOverrides, time)
    fk = [ControlFrame(fkPose, c) * JointFrame(fkPose, j).GetInverse()
          for c, j in zip(limb.fkControls, limb.joints)]
    effector = JointFrame(pose, limb.joints[2]) * \
        _Frame(pose, limb.effector).GetInverse()
    poleDistance = None
    if limb.pole is not None:
        middle = JointFrame(pose, limb.joints[1]).ExtractTranslation()
        poleDistance = (ControlFrame(pose, limb.pole).ExtractTranslation() -
                        middle).GetLength()
    points = [JointFrame(fkPose, j).ExtractTranslation() for j in limb.joints]
    lengths = ((points[1] - points[0]).GetLength(),
               (points[2] - points[1]).GetLength())
    return RestOffsets(fk, effector, poleDistance, lengths)


# -- limb lengths ---------------------------------------------------------

def LimbLengths(rest, dist, poleUpper, poleLower, stretch, soft, top,
                bottom, pin, calibration=0.0):
    """The two bone lengths the IK solves with: RigExecLimbSegmentLengths
    (rigExecMath/limbStretchKernel.h), the arithmetic it shares with the
    evaluator, restated so a switch can solve for the stretch that gives
    lengths it wants without evaluating the rig."""
    stretch = min(max(stretch, 0.0), 1.0)
    pin = min(max(pin, 0.0), 1.0)
    a = rest[0] * (top - calibration)
    b = rest[1] * (bottom - calibration)
    chain = a + b
    k = 1.0
    soft = max(soft, 0.0)
    if chain > 1e-12:
        if soft > 1e-12 and dist > chain - soft:
            eased = chain - soft * math.exp(-(dist - (chain - soft)) / soft)
            if eased > 1e-12:
                k = dist / eased
        elif soft <= 1e-12 and dist > chain:
            k = dist / chain
    sa = stretch * k * a + (1.0 - stretch) * a
    sb = stretch * k * b + (1.0 - stretch) * b
    ua = stretch * sa + (1.0 - stretch) * rest[0] * top
    ub = stretch * sb + (1.0 - stretch) * rest[1] * bottom
    return (pin * poleUpper + (1.0 - pin) * ua,
            pin * poleLower + (1.0 - pin) * ub)


def SolveStretch(rest, dist, poleUpper, poleLower, stretch, soft, pin,
                 calibration, want, start=(1.0, 1.0)):
    """stretchTop / stretchBottom whose LimbLengths are `want`: Newton's
    method on the two scales, with a finite-difference Jacobian. The map is
    smooth and nearly diagonal, so it converges in a few steps; a pinned
    limb, whose lengths ignore the scales, keeps `start`."""
    top, bottom = start
    for _ in range(30):
        u, l = LimbLengths(rest, dist, poleUpper, poleLower, stretch, soft,
                           top, bottom, pin, calibration)
        eu, el = u - want[0], l - want[1]
        if abs(eu) < 1e-10 and abs(el) < 1e-10:
            break
        h = 1e-6
        u1, l1 = LimbLengths(rest, dist, poleUpper, poleLower, stretch,
                             soft, top + h, bottom, pin, calibration)
        u2, l2 = LimbLengths(rest, dist, poleUpper, poleLower, stretch,
                             soft, top, bottom + h, pin, calibration)
        j = ((u1 - u) / h, (u2 - u) / h, (l1 - l) / h, (l2 - l) / h)
        det = j[0] * j[3] - j[1] * j[2]
        if abs(det) < 1e-14:
            break
        top -= (j[3] * eu - j[1] * el) / det
        bottom -= (-j[2] * eu + j[0] * el) / det
    return top, bottom


# -- the two actions ------------------------------------------------------

def PolePosition(joints, distance, fallback):
    """Where a pole goes for three joint positions: on their bend plane, on
    the side the middle joint bends toward, `distance` from it. A limb too
    straight to have a bend plane keeps `fallback`'s direction from the
    middle joint, projected off the limb's line."""
    root, middle, end = joints
    line = end - root
    length = line.GetLength()
    if length < 1e-9:
        return fallback
    axis = line / length
    offset = middle - root
    bend = offset - axis * Gf.Dot(offset, axis)
    if bend.GetLength() <= _STRAIGHT * length:
        bend = fallback - middle
        bend = bend - axis * Gf.Dot(bend, axis)
        if bend.GetLength() < 1e-9:
            return fallback
    return middle + bend.GetNormalized() * distance


def Match(limb, stage, evaluate, time, toIk, rest=None, written=None,
          current=None):
    """{attribute path: value} posing the half named by `toIk` onto the
    limb's current joints. `evaluate(overrides, time)` returns a
    rigexec.Pose for the stage with `overrides` ({attribute path: value})
    applied on top; `rest` is MeasureRest's answer, measured when omitted.
    `current` ({attribute path: value}) holds channel values that stand
    over the stage's -- the ones `evaluate` is applying already, such as an
    uncommitted preview -- so the solve starts from the pose that was
    evaluated.
    """
    if rest is None:
        rest = MeasureRest(limb, stage, evaluate, time)
    if written is None:
        written = RigWritten(stage)
    pose = evaluate({}, time)
    joints = [JointFrame(pose, j) for j in limb.joints]
    values = {}
    stretches = limb.Stretches(stage) and rest.lengths is not None
    if toIk:
        control = _Channels(stage, limb.ikControl, time, current)
        controlFrame = ControlFrame(pose, limb.ikControl)
        parent = control.Matrix().GetInverse() * controlFrame
        # The effector rides the IK control (through a foot's roll stack,
        # whose current values are kept): carry that relationship as it
        # stands now.
        carry = _Frame(pose, limb.effector) * controlFrame.GetInverse()
        effectorTarget = rest.effector.GetInverse() * joints[2]
        target = carry.GetInverse() * effectorTarget
        solved = control.Solve(target, parent)
        for name in _Writable(stage, limb.ikControl,
                              _TRANSLATE + _ROTATE, written):
            values[limb.ikControl.AppendProperty("avars:" + name)] = \
                solved[name]
        if limb.pole is not None and rest.poleDistance:
            # The pole may live in a space that follows the IK control (a
            # leg's pole follows the foot), so its parent frame is read
            # with the IK control already where the match puts it.
            moved = evaluate(values, time)
            pole = _Channels(stage, limb.pole, time, current)
            poleFrame = ControlFrame(moved, limb.pole)
            poleParent = pole.Matrix().GetInverse() * poleFrame
            points = [m.ExtractTranslation() for m in joints]
            where = PolePosition(points, rest.poleDistance,
                                 ControlFrame(pose, limb.pole)
                                 .ExtractTranslation())
            solvedPole = pole.SolveTranslate(where, poleParent)
            for name in _Writable(stage, limb.pole, _TRANSLATE, written):
                values[limb.pole.AppendProperty("avars:" + name)] = \
                    solvedPole[name]
        if stretches:
            # The FK bones' lengths, kept: solve the stretch scales that make
            # the IK solve with exactly these lengths, so the middle joint
            # lands where FK had it.
            points = [m.ExtractTranslation() for m in joints]
            want = ((points[1] - points[0]).GetLength(),
                    (points[2] - points[1]).GetLength())
            if limb.pole is not None and rest.poleDistance:
                polePoint = where
            else:
                polePoint = points[1]

            def channel(name, fallback):
                path = limb.Channel(name)
                if current and path in current:
                    return float(current[path])
                return _Get(stage, path, time, fallback)
            solver = stage.GetPrimAtPath(limb.ikSolver) \
                if limb.ikSolver is not None else None
            calibration = 0.0
            if solver:
                attr = solver.GetAttribute("rigExec:scaleCalibration")
                if attr and attr.Get() is not None:
                    calibration = float(attr.Get())
            top, bottom = SolveStretch(
                rest.lengths, (points[2] - points[0]).GetLength(),
                (polePoint - points[0]).GetLength(),
                (points[2] - polePoint).GetLength(),
                channel(_STRETCH, 1.0), channel(_SOFT, 0.0),
                channel(_PIN, 0.0), calibration, want,
                (channel(_STRETCH_TOP, 1.0), channel(_STRETCH_BOTTOM, 1.0)))
            values[limb.Channel(_STRETCH_TOP)] = top
            values[limb.Channel(_STRETCH_BOTTOM)] = bottom
    else:
        if stretches:
            # The IK bones' lengths carried into FK, whose chain lengthens
            # by stretchTop / stretchBottom; the pin and twist are IK's own
            # and go back to zero, as the FK pose has neither.
            points = [m.ExtractTranslation() for m in joints]
            values[limb.Channel(_STRETCH_TOP)] = \
                (points[1] - points[0]).GetLength() / rest.lengths[0]
            values[limb.Channel(_STRETCH_BOTTOM)] = \
                (points[2] - points[1]).GetLength() / rest.lengths[1]
            values[limb.Channel(_PIN)] = 0.0
            if stage.GetAttributeAtPath(limb.Channel(_TWIST)):
                values[limb.Channel(_TWIST)] = 0.0
            # The FK controls' parent frames move with the new lengths.
            pose = evaluate(values, time)

        def place(pose, base):
            # Root to end: each control's parent frame moves with the
            # control above it, so carry the change down rather than
            # re-evaluating. `base` holds the values `pose` was evaluated
            # with, so a control already moved there is solved from them.
            previousOld = previousNew = None
            for control, joint, offset in zip(limb.fkControls, joints,
                                              rest.fk):
                channels = _Channels(stage, control, time, base)
                old = ControlFrame(pose, control)
                parent = channels.Matrix().GetInverse() * old
                if previousOld is not None:
                    parent = parent * previousOld.GetInverse() * previousNew
                target = offset * joint
                solved = channels.Solve(target, parent)
                for name in _Writable(stage, control, _TRANSLATE + _ROTATE,
                                      written):
                    values[control.AppendProperty("avars:" + name)] = \
                        solved[name]
                new = channels.Matrix(solved) * parent
                previousOld, previousNew = old, new

        place(pose, current)
        if stretches:
            # The new lengths change the IK limb the pose above was taken
            # from, and with it anything that estimates that limb (an auto
            # clavicle turning the limb's root). Settle against the FK
            # limb itself: there the placed controls, not the IK, decide.
            for _ in range(2):
                trial = dict(current or {})
                trial.update(values)
                trial[limb.switchPath] = limb.fkValue
                place(evaluate(trial, time), trial)
    return values


def Switch(limb, toIk):
    """{attribute path: value} handing the limb's joints to the named
    half."""
    return {limb.switchPath: limb.ikValue if toIk else limb.fkValue}


def Plan(limb, stage, evaluate, time, toIk=None, rest=None, written=None,
         current=None):
    """Match then Switch, as one set of values: what a "Switch to IK / FK"
    command authors. `toIk` defaults to the half that is not driving now.
    """
    if toIk is None:
        toIk = not limb.IsIk(stage, time, current)
    values = Match(limb, stage, evaluate, time, toIk, rest, written,
                   current)
    values.update(Switch(limb, toIk))
    return values, toIk


def Residual(limb, stage, evaluate, time, values):
    """How far the limb's joints move when `values` are applied: (worst
    position error, worst rotation error in degrees) over its three
    joints. A match that worked moves nothing."""
    before = evaluate({}, time)
    after = evaluate(values, time)
    worstPosition = worstRotation = 0.0
    for joint in limb.joints:
        a, b = JointFrame(before, joint), JointFrame(after, joint)
        worstPosition = max(worstPosition, (a.ExtractTranslation() -
                                            b.ExtractTranslation())
                            .GetLength())
        qa = a.GetOrthonormalized().ExtractRotationQuat()
        qb = b.GetOrthonormalized().ExtractRotationQuat()
        dot = min(1.0, abs(Gf.Dot(qa, qb)))
        worstRotation = max(worstRotation, math.degrees(2.0 * math.acos(dot)))
    return worstPosition, worstRotation


# -- a private evaluator for a host --------------------------------------

# Second stages over a host's layers, keyed by their root and session
# layers, and the rigs compiled on them.
_TWINS = {}
_RIGS = {}


def _CompilableStage(stage):
    """A stage `rigexec.Rig` will accept, over the same layers.

    A host such as usdview hands out a stage held in a `UsdStageCache`,
    which the binding refuses. A second stage over the same root and
    session layers composes identically, and every edit lands in those
    shared layers and reaches both. (The Shape Editor and the Execution
    Stack panel use the same answer.)"""
    root = stage.GetRootLayer()
    session = stage.GetSessionLayer()
    key = (root.identifier, session.identifier if session else None)
    twin = _TWINS.get(key)
    if (twin is None or twin.GetRootLayer() != root or
            twin.GetSessionLayer() != session):
        twin = Usd.Stage.Open(root, session)
        _TWINS[key] = twin
        for stale in [k for k in _RIGS if k[0] == key]:
            del _RIGS[stale]
    return twin, key


def Evaluator(stage, rigPath):
    """evaluate(overrides, time) for `stage`'s rig at `rigPath`, on a
    private rig compiled once per stage. Weight fields are off: a match
    reads frames only."""
    import rigexec
    twin, key = _CompilableStage(stage)
    rig = _RIGS.get((key, str(rigPath)))
    if rig is None:
        rig = rigexec.Rig(twin, str(rigPath))
        rig.compile()
        rig.publish_weight_fields = False
        _RIGS[(key, str(rigPath))] = rig

    def evaluate(overrides, time):
        rig.set_interactive_overrides(
            [(str(p.GetPrimPath()), p.name, float(v))
             for p, v in overrides.items()])
        try:
            return rig.evaluate(-1.0 if time.IsDefault()
                                else time.GetValue())
        finally:
            rig.clear_interactive_overrides()

    evaluate.rig = rig
    return evaluate


# -- applying a switch -----------------------------------------------------

def PlanSwitch(stage, limbs, time, restCache=None):
    """({attribute path: value}, [(limb, toIk, channels)]) that match and
    switch every limb in `limbs` to its other half, without authoring.
    `restCache` ({key: RestOffsets}) keeps the rest measurements between
    calls; they change only with the rig's structure."""
    values = {}
    done = []
    for limb in limbs:
        evaluate = Evaluator(stage, limb.rigRoot)
        rest = None
        key = (limb.blend, evaluate.rig.binding_epoch_digest())
        if restCache is not None:
            rest = restCache.get(key)
        if rest is None:
            rest = MeasureRest(limb, stage, evaluate, time)
            if restCache is not None:
                restCache[key] = rest
        planned, toIk = Plan(limb, stage, evaluate, time, rest=rest)
        values.update(planned)
        done.append((limb, toIk, len(planned) - 1))
    return values, done


def SwitchLabel(done):
    """The undo label for PlanSwitch's `done`."""
    return "Switch %s" % ", ".join(
        "%s to %s" % (limb.switchControl.name, "IK" if toIk else "FK")
        for limb, toIk, _ in done)


def SwitchLimbs(stage, limbs, time, mode, undoStack=None, restCache=None):
    """Match and switch every limb in `limbs` to its other half, authored
    as ONE edit and one undo entry. `mode` is a gizmoMath write mode, so a
    switch lands the way a drag does (keyed in animation mode). Returns
    [(limb, toIk, channels)] for the limbs switched."""
    values, done = PlanSwitch(stage, limbs, time, restCache)
    if not values:
        return []
    label = SwitchLabel(done)
    scope = None
    if undoStack is not None:
        import rigExecUndo
        scope = rigExecUndo.SpecScope(stage, list(values), undoStack, label)
        scope.__enter__()
    try:
        writer = gizmoMath.Writer(stage, time, mode)
        for path, value in values.items():
            attr = stage.GetAttributeAtPath(path)
            if attr:
                writer.Set(attr, value)
        writer.CommitToStage()
    except Exception as error:
        if scope is not None:
            scope.__exit__(type(error), error, None)
        raise
    if scope is not None:
        scope.__exit__(None, None, None)
    return done
