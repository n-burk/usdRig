#
# RigExec usdview graph editor model: which attributes are animation
# curves, and the Ts.Spline edits behind the editor's gestures.
#
# Qt-free by design (volumeWeightUI.py's banner rule): every operation
# here is exercised headlessly by tests/python/test_graph_model.py, and
# graphEditorUI.py holds all the widgets.
#
# Two rules the whole module obeys
# (docs/superpowers/specs/2026-09-01-graph-editor-design.md:1.4, 2.4):
#
# 1. Every edit operation MUTATES the Ts.Spline it is handed. It never
#    touches an attribute. The panel copies the attribute's spline with
#    Ts.Spline(spline), runs a gesture's worth of operations on the copy,
#    and writes the copy back once through ApplySpline -- so one gesture
#    is one whole-spline write and one undo step.
# 2. Time is in FRAMES, matching usdview's current frame, so "snap" means
#    "round to a whole frame".
#
# Ts splines are scalar in this build (Ts.Spline.IsSupportedValueType is
# true for double/float/half, false for double3), so the editor lists
# scalar attributes only; vector xformOps carry time samples and are not
# curves here (spec section 1.1).
#
import math

from pxr import Sdf, Ts, Usd

import gizmoMath
import rigExecUndo


# ---------------------------------------------------------------------------
# Curve identity and colour
# ---------------------------------------------------------------------------

# Maya's channel colouring: the axis decides the hue, so tx/rx/sx share
# red and an artist reads the axis off the curve without the legend.
CURVE_RED = (0.95, 0.3, 0.3)
CURVE_GREEN = (0.3, 0.85, 0.3)
CURVE_BLUE = (0.35, 0.55, 1.0)
CURVE_YELLOW = (0.95, 0.85, 0.3)

# Everything with no axis of its own cycles through this, by list index,
# so two curves in one editor are rarely the same colour.
CURVE_PALETTE = (
    (0.85, 0.55, 0.95),
    (0.35, 0.85, 0.85),
    (0.95, 0.65, 0.35),
    (0.65, 0.85, 0.35),
    (0.95, 0.45, 0.75),
    (0.70, 0.70, 0.80),
)

_CHANNEL_COLORS = {
    "tx": CURVE_RED, "rx": CURVE_RED, "sx": CURVE_RED,
    "ty": CURVE_GREEN, "ry": CURVE_GREEN, "sy": CURVE_GREEN,
    "tz": CURVE_BLUE, "rz": CURVE_BLUE, "sz": CURVE_BLUE,
    "rspin": CURVE_YELLOW,
}

# The rig channels the editor offers even when they carry no spline yet,
# so an unanimated control can be keyed from the graph (spec 1.3). The
# ORDER is the listing order, and it is the gizmo's own channel order
# (gizmoMath.py:34-41) so the graph and the toolbar agree.
RIG_CHANNELS = (gizmoMath.AVAR_T + gizmoMath.AVAR_R + gizmoMath.AVAR_S
                + (gizmoMath.AVAR_RSPIN,)
                + gizmoMath.REST_T + gizmoMath.REST_R)


def CurveColor(attrName, index=0):
    """
    The drawing colour for `attrName`, `index` places into the curve list.

    The channel is the part after the last namespace separator, so
    `avars:tx`, `rest:tx` and a bare `tx` all read as the X axis.
    """
    color = _CHANNEL_COLORS.get(attrName.rsplit(":", 1)[-1])
    if color is not None:
        return color
    return CURVE_PALETTE[index % len(CURVE_PALETTE)]


class CurveRef(object):
    """
    One curve in the editor: an attribute, addressed by path, plus the
    colour it draws in.

    Deliberately NOT a Usd.Attribute handle: the editor outlives stage
    recomposition (usdview's signalStageReplaced, and every session-layer
    write the editor itself makes), and a stale handle would draw a curve
    that no longer exists. Paths re-resolve; handles do not.
    """

    def __init__(self, primPath, attrName, color):
        self.primPath = Sdf.Path(primPath)
        self.attrName = attrName
        self.color = tuple(color)

    @property
    def attrPath(self):
        return self.primPath.AppendProperty(self.attrName)

    def Label(self):
        return "%s.%s" % (self.primPath.name, self.attrName)

    def __eq__(self, other):
        if not isinstance(other, CurveRef):
            return NotImplemented
        return (self.primPath == other.primPath
                and self.attrName == other.attrName)

    def __ne__(self, other):
        result = self.__eq__(other)
        return result if result is NotImplemented else not result

    def __hash__(self):
        return hash((self.primPath, self.attrName))

    def __repr__(self):
        return "CurveRef(%s)" % self.attrPath


def IsCurveAttr(attr):
    """
    Whether a Ts spline can be authored on `attr`.

    Uniform attributes are rejected before the type test: `avars:
    rotationOrder` is a uniform token and would never animate, and
    gizmoMath.SetAnimated:501-503 refuses it for the same reason.
    """
    if not attr or attr.GetVariability() == Sdf.VariabilityUniform:
        return False
    return Ts.Spline.IsSupportedValueType(attr.GetTypeName().type)


def DiscoverCurves(stage, propPaths, primPaths):
    """
    The curve set for a usdview selection (spec section 1.3).

    The property browser wins when it holds at least one curve-capable
    attribute: an artist who picked `avars:tx` wants that one channel,
    not the control's other twelve. A selection of vectors and tokens
    only (which cannot carry a spline at all) is not a curve choice, so
    it falls through to the prims.

    Per prim, the attributes that already have a spline come first --
    they are what the artist can see in the viewport -- then the rig
    avar / rest channels that do not, so an unanimated control can be
    keyed from the editor. Duplicates are dropped, keeping the first
    position.
    """
    refs = []
    seen = set()

    def Add(attr):
        path = attr.GetPath()
        if path in seen:
            return
        seen.add(path)
        name = attr.GetName()
        refs.append(CurveRef(path.GetPrimPath(), name,
                             CurveColor(name, len(refs))))

    for path in propPaths or ():
        obj = stage.GetObjectAtPath(Sdf.Path(path))
        if isinstance(obj, Usd.Attribute) and IsCurveAttr(obj):
            Add(obj)
    if refs:
        return refs

    for path in primPaths or ():
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim:
            continue
        for attr in prim.GetAttributes():
            if attr.HasSpline() and IsCurveAttr(attr):
                Add(attr)
        # IsRigXformable covers controls, joints and volume weights; the
        # volume weight schema simply does not declare avars:sx/sy/sz
        # (generatedSchema.usda:233-244), so the validity test below
        # drops the scale channels the evaluator would ignore anyway
        # (gizmoMath.ReadsScaleAvars:182-197).
        if gizmoMath.IsRigXformable(prim):
            for name in RIG_CHANNELS:
                attr = prim.GetAttribute(name)
                if IsCurveAttr(attr):
                    Add(attr)
    return refs


# ---------------------------------------------------------------------------
# Knots
# ---------------------------------------------------------------------------

# Maya's default new key: Auto tangents on both sides and a curve segment
# after it (spec assumption 1.2).
DEFAULT_TAN_ALGORITHM = Ts.TangentAlgorithmAutoEase
DEFAULT_INTERP = Ts.InterpCurve

# Two knots closer together than this are the same key as far as an
# animator is concerned; used as the minimum separation when a move is
# clamped with frame snapping off.
MIN_TIME_GAP = 1e-6

# Slopes within this of each other draw as one line, so IsUnified treats
# them as the same tangent.
SLOPE_EPSILON = 1e-9

# Maya remembers that a key's tangents were broken even while the two
# sides still happen to agree; Ts has no such flag, so it is recorded in
# the knot's customData. `preTanAlgorithm` / `postTanAlgorithm` are the
# only reserved keys (ts/types.h:149-153), so a namespaced key of our own
# is safe, and it round trips through .usda with the knot.
BROKEN_KEY = "rigExec"
BROKEN_FIELD = "tangentsBroken"


def _IsBrokenMarked(knot):
    # Ts.Knot.GetCustomDataByKey SEGFAULTS on an absent key in this build
    # (verified: pxr.Ts, USD v0.26.8), so the whole dictionary is read.
    data = knot.GetCustomData()
    return bool(data.get(BROKEN_KEY, {}).get(BROKEN_FIELD, False))


def _MarkBroken(knot, broken):
    # Ts.Spline.SetKnot MERGES a knot's customData into what the spline
    # already holds for that time (verified: clearing the dictionary on
    # the knot and writing it back leaves the old key in place), so
    # unifying writes False over True rather than removing the key. A
    # knot that was never broken is left with no customData at all.
    if not broken and BROKEN_FIELD not in knot.GetCustomData().get(
            BROKEN_KEY, {}):
        return
    knot.SetCustomDataByKey("%s:%s" % (BROKEN_KEY, BROKEN_FIELD),
                            bool(broken))


def SplineFor(attr):
    """
    The attribute's spline, always TYPED and ready to be authored into.

    Usd.Attribute.GetSpline() on an attribute that has no spline yet
    hands back a default-constructed Ts.Spline whose value type name is
    empty, and Ts.Knot(typeName="") raises -- so the first key on an
    unanimated channel could never be made. Give that empty spline the
    attribute's own type. Callers must still check IsCurveAttr first:
    this does not make a vector attribute splineable.
    """
    spline = attr.GetSpline()
    if not spline.GetValueTypeName():
        spline = Ts.Spline(attr.GetTypeName().type.typeName)
    return spline


def AuthorKnot(spline, time, value):
    """
    Create or update the knot at `time` and return it.

    A NEW knot gets Maya's defaults: AutoEase on both tangents and a
    curve segment after it. An EXISTING knot keeps everything but its
    value -- re-keying a channel the artist has already shaped must not
    silently throw that shape away.

    This is the one place new knots are born, so gizmoMath.SetAnimated
    calls it too and a gizmo drag and a graph insert produce the same
    key.
    """
    frame = float(time)
    knot = spline.GetKnot(frame)
    if knot is None:
        knot = Ts.Knot(typeName=spline.GetValueTypeName(), time=frame,
                       value=value, nextInterp=DEFAULT_INTERP)
        knot.SetPreTanAlgorithm(DEFAULT_TAN_ALGORITHM)
        knot.SetPostTanAlgorithm(DEFAULT_TAN_ALGORITHM)
    else:
        knot.SetValue(value)
    spline.SetKnot(knot)
    return knot


def SnapTime(time):
    """
    `time` rounded to the nearest whole frame, halves AWAY from zero.

    Python's round() sends halves to even, which makes a slow drag stick
    unevenly -- 0.5 landing on 0 but 1.5 on 2 -- and the asymmetry is
    visible on a frame grid. gizmoMath._SnapValue:559-575 rounds the same
    way for the same reason.
    """
    value = float(time)
    return math.copysign(math.floor(abs(value) + 0.5), value)


def KeyTimes(spline):
    """Every knot time, ascending."""
    return [knot.GetTime() for knot in spline.GetKnots().values()]


def KeyNeighbours(spline, time):
    """
    The knot times STRICTLY either side of `time`, either one None.

    `time` need not be a knot: the canvas asks this for the cursor
    position too, to find the segment under it.
    """
    target = float(time)
    previous = following = None
    for knot in spline.GetKnots().values():
        current = knot.GetTime()
        if current < target:
            previous = current
        elif current > target:
            following = current
            break
    return previous, following


def IsUnified(knot):
    """
    Whether the knot's two tangents move as one (Maya's "unified").

    Broken-ness is read from the customData marker BreakTangents leaves,
    because two sides can be broken and still agree -- breaking a tangent
    does not bend the curve, it only unlocks it. Without a marker the
    only evidence is the tangents themselves: sides that share an
    automatic algorithm are smooth by construction, and authored sides
    agree only while their slopes match.
    """
    if knot is None:
        return False
    if _IsBrokenMarked(knot):
        return False
    if knot.GetPreTanAlgorithm() != knot.GetPostTanAlgorithm():
        return False
    if knot.GetPreTanAlgorithm() != Ts.TangentAlgorithmCustom:
        return True
    return abs(knot.GetPreTanSlope()
               - knot.GetPostTanSlope()) <= SLOPE_EPSILON


# ---------------------------------------------------------------------------
# Edit operations (each mutates the spline it is given)
# ---------------------------------------------------------------------------

def MoveKeys(spline, times, dt, dv, snapFrames=True):
    """
    Move the keys at `times` by `dt` frames and `dv` in value; return
    their new times, ascending.

    Maya keeps key order through a drag, so a moved key is clamped into
    the open interval between the nearest keys that are NOT moving, and
    the moving keys are placed in the direction of travel so they cannot
    collide with each other either. While snapping, the clamp stops one
    whole frame short of the neighbour, which keeps every key on its own
    frame; with snapping off it stops MIN_TIME_GAP short.

    Times that name no key are ignored: the canvas can ask to move a
    selection that a concurrent stage change has already invalidated.
    """
    knots = spline.GetKnots()
    moving = sorted(t for t in set(float(x) for x in times) if t in knots)
    if not moving:
        return []
    movingSet = set(moving)
    fixed = [t for t in KeyTimes(spline) if t not in movingSet]
    gap = 1.0 if snapFrames else MIN_TIME_GAP
    forward = dt >= 0.0

    placed = {}
    limit = None
    for time in (reversed(moving) if forward else moving):
        target = time + dt
        if snapFrames:
            target = SnapTime(target)
        below = [t for t in fixed if t < time]
        above = [t for t in fixed if t > time]
        if below:
            target = max(target, below[-1] + gap)
        if above:
            target = min(target, above[0] - gap)
        if limit is not None:
            target = (min(target, limit - gap) if forward
                      else max(target, limit + gap))
        placed[time] = target
        limit = target

    # Build every moved knot before removing any: a knot read out of the
    # spline resolves its automatic tangents against its neighbours, and
    # removing keys first would resolve them against a spline with holes.
    moved = []
    for time in moving:
        knot = spline.GetKnot(time)
        if knot.IsDualValued():
            knot.SetPreValue(knot.GetPreValue() + dv)
        knot.SetValue(knot.GetValue() + dv)
        knot.SetTime(placed[time])
        moved.append(knot)
    for time in moving:
        spline.RemoveKnot(time)
    for knot in moved:
        spline.SetKnot(knot)
    return sorted(placed.values())


def InsertKey(spline, time):
    """
    Add a key at `time` holding the curve's value there, or return the
    key already on that frame.

    The new key takes Auto tangents (spec 2.4), which DOES reshape the
    neighbouring segments slightly -- this is Maya's "Add Key", not its
    shape-preserving "Insert Key". Ts.Spline.Breakdown is the
    shape-preserving alternative if that is ever wanted.

    None when the spline has no knots at all: there is no value to
    evaluate. A caller keying an unanimated attribute uses AuthorKnot
    with the attribute's resolved value instead.
    """
    frame = float(time)
    knot = spline.GetKnot(frame)
    if knot is not None:
        return knot
    value = spline.Eval(frame)
    if value is None:
        return None
    return AuthorKnot(spline, frame, value)


def DeleteKeys(spline, times):
    """
    Remove the keys at `times`; return the times actually removed.

    Ts.Spline.RemoveKnot RAISES on a time that holds no knot, so every
    time is checked first: a Delete on a stale selection must not take
    the editor down.
    """
    knots = spline.GetKnots()
    removed = []
    for time in sorted(set(float(t) for t in times)):
        if time in knots:
            spline.RemoveKnot(time)
            removed.append(time)
    return removed


TANGENT_AUTO = "auto"
TANGENT_SPLINE = "spline"
TANGENT_LINEAR = "linear"
TANGENT_FLAT = "flat"
TANGENT_STEP = "step"
TANGENT_MODES = (TANGENT_AUTO, TANGENT_SPLINE, TANGENT_LINEAR,
                 TANGENT_FLAT, TANGENT_STEP)

SIDE_IN = "in"
SIDE_OUT = "out"
SIDE_BOTH = "both"


def _CatmullRomSlope(spline, time):
    """
    Maya's "Spline" tangent: the slope of the line through the
    neighbouring keys, (v_next - v_prev) / (t_next - t_prev).

    An end key has only one neighbour, so it takes the slope to that
    neighbour -- the same line, with the missing half folded away.
    """
    knot = spline.GetKnot(time)
    previousTime, followingTime = KeyNeighbours(spline, time)
    previous = (spline.GetKnot(previousTime)
                if previousTime is not None else None)
    following = (spline.GetKnot(followingTime)
                 if followingTime is not None else None)
    if previous is not None and following is not None:
        return ((following.GetValue() - previous.GetValue())
                / (followingTime - previousTime))
    if following is not None:
        return (following.GetValue() - knot.GetValue()) / (followingTime
                                                           - time)
    if previous is not None:
        return (knot.GetValue() - previous.GetValue()) / (time
                                                          - previousTime)
    return 0.0


def _SetSegmentInterp(spline, time, mode, side):
    """
    Put `mode` on the segment(s) `side` names around the key at `time`.

    A segment belongs to the knot BEFORE it (Ts.Knot.SetNextInterpolation),
    so the In side of a key is the previous knot's segment.
    """
    if side in (SIDE_OUT, SIDE_BOTH):
        knot = spline.GetKnot(time)
        knot.SetNextInterpolation(mode)
        spline.SetKnot(knot)
    if side in (SIDE_IN, SIDE_BOTH):
        previousTime = KeyNeighbours(spline, time)[0]
        if previousTime is not None:
            previous = spline.GetKnot(previousTime)
            previous.SetNextInterpolation(mode)
            spline.SetKnot(previous)


def SetTangentType(spline, times, mode, side=SIDE_BOTH):
    """
    Apply one of Maya's tangent-type buttons to the keys at `times`
    (spec 2.4).

    `auto`, `spline` and `flat` shape the tangents and restore
    InterpCurve on the segments they touch -- a key can go straight from
    Step back to a curve. `linear` and `step` are segment modes, not
    tangents: they set InterpLinear / InterpHeld on the segment after the
    key (or before it, for the In side) and leave the tangents alone,
    which is where they live in Ts.

    Maya's Clamped and Plateau are out of scope (spec 1.6); ask for
    `auto` instead.
    """
    if mode not in TANGENT_MODES:
        raise ValueError("unknown tangent mode %r; expected one of %s"
                         % (mode, ", ".join(TANGENT_MODES)))
    if side not in (SIDE_IN, SIDE_OUT, SIDE_BOTH):
        raise ValueError("unknown tangent side %r" % (side,))

    for time in sorted(set(float(t) for t in times)):
        if spline.GetKnot(time) is None:
            continue
        if mode == TANGENT_STEP:
            _SetSegmentInterp(spline, time, Ts.InterpHeld, side)
            continue
        if mode == TANGENT_LINEAR:
            _SetSegmentInterp(spline, time, Ts.InterpLinear, side)
            continue
        _SetSegmentInterp(spline, time, Ts.InterpCurve, side)
        # Read the knot only AFTER the segment writes above, or the
        # interpolation change would be dropped by this SetKnot.
        knot = spline.GetKnot(time)
        if mode == TANGENT_AUTO:
            if side in (SIDE_IN, SIDE_BOTH):
                knot.SetPreTanAlgorithm(Ts.TangentAlgorithmAutoEase)
            if side in (SIDE_OUT, SIDE_BOTH):
                knot.SetPostTanAlgorithm(Ts.TangentAlgorithmAutoEase)
        else:
            slope = (0.0 if mode == TANGENT_FLAT
                     else _CatmullRomSlope(spline, time))
            if side in (SIDE_IN, SIDE_BOTH):
                knot.SetPreTanAlgorithm(Ts.TangentAlgorithmCustom)
                knot.SetPreTanSlope(slope)
            if side in (SIDE_OUT, SIDE_BOTH):
                knot.SetPostTanAlgorithm(Ts.TangentAlgorithmCustom)
                knot.SetPostTanSlope(slope)
        spline.SetKnot(knot)


def SetTangent(spline, time, side, slope, width=None):
    """
    Author one tangent handle: the result of dragging its end.

    The side becomes Custom, because an automatic side would recompute
    the slope away on the next read. `width=None` keeps the width the
    handle already has, which is what an unweighted drag wants (spec
    2.4's Weighted toggle); a weighted drag passes the new width.

    Returns the knot, or None when `time` names no key.
    """
    if side not in (SIDE_IN, SIDE_OUT, SIDE_BOTH):
        raise ValueError("unknown tangent side %r" % (side,))
    knot = spline.GetKnot(float(time))
    if knot is None:
        return None
    if side in (SIDE_IN, SIDE_BOTH):
        knot.SetPreTanAlgorithm(Ts.TangentAlgorithmCustom)
        knot.SetPreTanSlope(float(slope))
        if width is not None:
            knot.SetPreTanWidth(max(0.0, float(width)))
    if side in (SIDE_OUT, SIDE_BOTH):
        knot.SetPostTanAlgorithm(Ts.TangentAlgorithmCustom)
        knot.SetPostTanSlope(float(slope))
        if width is not None:
            knot.SetPostTanWidth(max(0.0, float(width)))
    spline.SetKnot(knot)
    return knot


def BreakTangents(spline, times):
    """
    Unlock the two tangents of the keys at `times` so they move
    independently, WITHOUT changing the curve.

    A knot read back out of a spline carries its automatic tangents
    already resolved -- slope and width both -- so writing those same
    numbers back under the Custom algorithm freezes the exact shape the
    artist sees and hands them over to the mouse. The marker records that
    the pair is broken, because the two sides still agree at this moment
    and nothing in the spline would otherwise say so (see IsUnified).
    """
    for time in sorted(set(float(t) for t in times)):
        knot = spline.GetKnot(time)
        if knot is None:
            continue
        knot.SetPreTanSlope(knot.GetPreTanSlope())
        knot.SetPostTanSlope(knot.GetPostTanSlope())
        knot.SetPreTanWidth(knot.GetPreTanWidth())
        knot.SetPostTanWidth(knot.GetPostTanWidth())
        knot.SetPreTanAlgorithm(Ts.TangentAlgorithmCustom)
        knot.SetPostTanAlgorithm(Ts.TangentAlgorithmCustom)
        _MarkBroken(knot, True)
        spline.SetKnot(knot)


def UnifyTangents(spline, times):
    """
    Re-link the two tangents of the keys at `times`: the OUT slope wins
    and is copied onto the in side (Maya's Unify), and the broken marker
    is dropped.

    Both sides stay authored: unifying an automatic key would be a no-op,
    and unifying one automatic side with one dragged side has to keep the
    dragged slope, which only Custom can hold.
    """
    for time in sorted(set(float(t) for t in times)):
        knot = spline.GetKnot(time)
        if knot is None:
            continue
        slope = knot.GetPostTanSlope()
        knot.SetPreTanWidth(knot.GetPreTanWidth())
        knot.SetPostTanWidth(knot.GetPostTanWidth())
        knot.SetPreTanSlope(slope)
        knot.SetPostTanSlope(slope)
        knot.SetPreTanAlgorithm(Ts.TangentAlgorithmCustom)
        knot.SetPostTanAlgorithm(Ts.TangentAlgorithmCustom)
        _MarkBroken(knot, False)
        spline.SetKnot(knot)


# Maya's Infinity menu -> TsExtrapMode. Verified against
# /Users/burkard/work/usd-pr4156/pxr/base/ts/types.h:112-121, whose two
# looping modes are easy to read backwards:
#
#   TsExtrapLoopRepeat = "Knot curve repeated, OFFSET so ends meet"
#       -> Maya "Cycle with Offset" (each repeat starts where the last
#          ended, so a walk cycle keeps travelling).
#   TsExtrapLoopReset  = "Curve repeated EXACTLY, discontinuous joins"
#       -> Maya "Cycle" (every repeat is the same, and the value jumps
#          back at the seam unless the ends already match).
#
# So the names invert what they suggest: Repeat is the offset one.
# Oscillate is "like Reset, but every other copy reversed", which is
# Maya's Oscillate exactly.
EXTRAP_MODES = {
    "constant": Ts.ExtrapHeld,
    "linear": Ts.ExtrapLinear,
    "cycle": Ts.ExtrapLoopReset,
    "cycle_offset": Ts.ExtrapLoopRepeat,
    "oscillate": Ts.ExtrapLoopOscillate,
}

EXTRAP_NAMES = dict((mode, name) for name, mode in EXTRAP_MODES.items())


def SetExtrapolation(spline, pre=None, post=None):
    """
    Set the pre / post infinity of `spline` from Maya's names.

    `constant`, `linear`, `cycle`, `cycle_offset`, `oscillate`; None
    leaves that side as it is. See EXTRAP_MODES above for the mapping and
    why `cycle` is LoopReset while `cycle_offset` is LoopRepeat.

    A looping infinity on a single-knot spline behaves as Held
    (ts/types.h:110-111); that is Ts's rule, not something to work
    around.
    """
    for name, apply_ in ((pre, spline.SetPreExtrapolation),
                         (post, spline.SetPostExtrapolation)):
        if name is None:
            continue
        if name not in EXTRAP_MODES:
            raise ValueError(
                "unknown infinity %r; expected one of %s"
                % (name, ", ".join(sorted(EXTRAP_MODES))))
        apply_(Ts.Extrapolation(EXTRAP_MODES[name]))


# ---------------------------------------------------------------------------
# Writing
# ---------------------------------------------------------------------------

def ApplySpline(stage, attrPath, spline, undoStack, label):
    """
    Write `spline` onto `attrPath` as one undo step; True when anything
    changed.

    The whole spline goes through rigExecUndo.EditRecorder so the write
    lands in the edit target (usdview: the session layer) and Ctrl+Z from
    the viewport toolbar undoes it exactly, including removing a session
    spec that did not exist before (rigExecUndo.py:9-16). A gesture that
    left the spline as it was pushes nothing, so the undo stack has no
    empty steps in it.
    """
    path = Sdf.Path(attrPath)
    attr = stage.GetAttributeAtPath(path)
    if not attr:
        return False
    recorder = rigExecUndo.EditRecorder(stage, [path])
    recorder.Begin()
    try:
        attr.SetSpline(spline)
    except Exception:
        recorder.Abort()
        raise
    edit = recorder.Commit(label)
    if edit is None:
        return False
    if undoStack is not None:
        undoStack.Push(edit)
    return True
