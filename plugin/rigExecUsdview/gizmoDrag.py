#
# RigExec usdview gizmo: what one drag MEANS.
#
# Qt-free, and split out of gizmoUI.py for exactly that reason. The rules
# in here are the manipulation half of design spec 8.2-8.4 -- Ctrl+axis
# moves in the perpendicular plane, a ring keeps counting past 180, the
# ball is a trackball composed from the drag base, a gimbal ring is one
# Euler channel, planar scale is two axes, J snaps the step and X snaps
# the grid -- and every one of them is a rule a reviewer should be able
# to check without launching usdview. gizmoUI.py is now only the Qt shell
# that turns mouse events into ApplyDrag() calls.
#
# The layering below this file is unchanged: gizmoScreen answers "what
# did the mouse mean in screen space", gizmoMath's Target answers "what
# does that world delta do to the channels". This module is the join,
# and it holds no state of its own beyond the DragState it is handed.
#
# RELATIVE snapping is the target's job, not this module's. Maya's
# steps are steps of the CHANNEL values -- a move lands tx on a whole
# number, a scale lands sx on one -- and only the target knows those
# values and the frame they live in. So J and the Step Snap option
# travel as the `snapStep` keyword on Apply*, and nothing here rounds
# a world delta for them. The WORLD snaps (Maya's X grid, point, edge
# and surface: snapping design 4.2-4.4) are decided here instead, as
# plain world deltas through gizmoSnap, because the old channel-space
# X put round channel numbers at arbitrary world positions under a
# posed parent (snapping design section 2). The exceptions are
# _DisplayAngle below, which is a readout, not a decision, and the
# Rotate absolute grid, which re-bases one Euler channel here because
# gizmoMath.py is frozen and gains no new Target API.
#
import math

from pxr import Gf

try:
    import gizmoScreen
    import gizmoSettings
    import gizmoSnap
except ImportError:                    # loader that did not add our dir
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoScreen
    import gizmoSettings
    import gizmoSnap

TOOL_TRANSLATE = gizmoScreen.TOOL_TRANSLATE
TOOL_ROTATE = gizmoScreen.TOOL_ROTATE
TOOL_SCALE = gizmoScreen.TOOL_SCALE

# A ray/plane delta more than this many times the camera-plane delta for
# the same mouse travel is the intersection blowing up on a plane that
# has gone nearly edge-on since the press. gizmoScreen guards the
# obvious case up front; this catches the plane that tips over mid-drag.
PLANE_DELTA_SANITY = 50.0


class DragState(object):
    """
    One live manipulation: what was grabbed, where, and what has been
    written so far.

    `handle` is the handle AS IT WAS AT THE PRESS and stays frozen for
    the whole drag. A rotate drag must turn about the axis the artist
    grabbed even as the object (and, in Object orientation, the ring)
    turns underneath; recomputing the axis from the redrawn handles
    would make the manipulator chase itself.

    `gimbal` records that the rings were laid out on gimbal axes, which
    changes which Target entry point a ring drag writes through -- and it
    is taken from what the handles were BUILT with, not from the option,
    because Gimbal falls back to Object for a target with no Euler
    channels.
    """

    def __init__(self, tool, handle, target, press, camera, viewport,
                 origin2d=None, gimbal=False, recorder=None):
        self.tool = tool
        self.handle = handle
        self.target = target
        self.recorder = recorder
        self.press = press
        self.current = press
        self.camera = camera
        self.viewport = viewport
        self.origin2d = origin2d if origin2d is not None else handle.center
        self.gimbal = bool(gimbal)
        # Snap state (snapping design 4.1, 4.4): `snapResolver` is the
        # (x, y, mode) -> SnapCandidate | None callable gizmoUI injects
        # once at _BeginDrag (gizmoUI.py:1928-1936); the mode travels on
        # every call, never bound at the press, because _DragKey /
        # _ReleaseHold change the holds mid-drag
        # (gizmoUI.py:1855-1898). `snap` is the live candidate, `snapKind`
        # the mode it was resolved for, `lastPickPoint` the cursor point
        # of the last resolve (PHYSICAL px, like press/current), and
        # `snapReason` the last downgrade clause for the status line.
        # `rigWritten` / `rigWrittenByTarget` are the prim paths whose
        # points a mover writes (spec 4.4 step 3a), kept here so the
        # resolver can reject rig-deformed hits.
        self.snapResolver = None
        self.snap = None
        self.snapKind = None
        self.snapReason = ""
        self.lastPickPoint = None
        self.rigWritten = frozenset()
        self.rigWrittenByTarget = frozenset()
        # The Euler base for Rotate's absolute grid: RotationState()
        # reads LIVE from the stage (gizmoMath.py:850-855), so only the
        # press-time read is the pre-drag value; mid-drag would return
        # what the drag already wrote. getattr because not every double
        # carries it.
        self.rotationBase = None
        try:
            _read = getattr(target, "RotationState", None)
            _rot = _read() if callable(_read) else None
            if _rot is not None:
                self.rotationBase = list(_rot[1])
        except Exception:
            self.rotationBase = None
        # Maya's Ctrl+axis "move in the perpendicular plane", read from
        # every event rather than only the press: on macOS Qt turns a
        # Ctrl+left CLICK into a right-button press, so the reachable
        # gesture is to grab the axis first and then hold Ctrl.
        self.ctrl = False
        # Rotation bookkeeping: `raw` is the last wrapped angle from
        # RotationDragAngle, `total` the accumulated one, `angle` what
        # was actually applied (total, after snapping), `startParameter`
        # where on the ring the press landed so the pie slice can start
        # there.
        self.raw = 0.0
        self.total = 0.0
        self.angle = 0.0
        self.startParameter = 0.0
        if handle is not None and handle.kind in ("ring", "view"):
            self.startParameter = gizmoScreen.RingParameter(handle, press)
        # Free rotate composes each step into one running world rotation
        # applied from the drag base, so a curved drag rolls the ball
        # instead of snapping back to a single press-to-cursor axis.
        self.trackballLast = press
        self.trackball = Gf.Matrix4d(1.0)
        self.lastDelta = None

    def __repr__(self):
        return "<DragState %s %s>" % (
            self.tool, self.handle.name if self.handle else "?")


def _DisplayAngle(angle, step):
    """
    The angle the target will land on, for the pie wedge and the status
    readout ONLY.

    The target does the snapping now and returns nothing, but the
    overlay still has to draw a wedge that ends where the object ended.
    gizmoScreen.SnapRelative rounds halves away from zero, which is the
    rule gizmoMath._SnapValue documents and uses, so the two agree.
    """
    return gizmoScreen.SnapRelative(angle, step) if step else angle


def _SnapStep(settings, holdSnap):
    """The step a drag should quantise to, or None for no snapping."""
    if not (bool(settings.stepSnap) or bool(holdSnap)):
        return None
    step = float(settings.stepSize)
    return step if step > 0.0 else None


def ActiveSnapMode(settings, tool, holdGrid, holdPoint, holdEdge):
    """
    The snap mode in force, as a (mode, reason) tuple, always a tuple.

    `holdPoint` (V) outranks `holdEdge` (C) outranks `holdGrid` (X)
    outranks the sticky `settings.snapMode` (snapping design 1.7): a
    vertex fully determines the landing, an edge constrains it to a
    line and the grid only to a lattice. Press order is irrelevant;
    only the boolean flags are read. gizmoUI is the only production
    caller -- it needs the mode with no drag in flight for the
    armed-mode hover marker and the status clause -- and passes the
    result down as ApplyDrag(..., snapMode=mode). ApplyDrag resolves
    through here itself only when snapMode is None.

    `reason` is "" when the mode is what was asked for and a short
    status clause when the request was downgraded to SNAP_OFF. A
    downgraded mode never falls through to a lower hold: a Rotate drag
    with V+X held must not silently grid-snap while the status claims
    Point is unavailable.
    """
    if bool(holdPoint):
        wanted = gizmoSettings.SNAP_POINT
    elif bool(holdEdge):
        wanted = gizmoSettings.SNAP_EDGE
    elif bool(holdGrid):
        wanted = gizmoSettings.SNAP_GRID
    else:
        wanted = getattr(settings, "snapMode",
                         gizmoSettings.SNAP_OFF)
        if wanted is None:
            wanted = gizmoSettings.SNAP_OFF
    if wanted == gizmoSettings.SNAP_OFF:
        return (gizmoSettings.SNAP_OFF, "")
    if wanted in gizmoSettings.SnapChoices(tool):
        return (wanted, "")
    # Point/edge/surface are Move-only (snapping design 1.2); anything
    # else names the mode and the tool it was asked for.
    if wanted in (gizmoSettings.SNAP_POINT, gizmoSettings.SNAP_EDGE,
                  gizmoSettings.SNAP_SURFACE):
        reason = "%s is Move only" % gizmoSettings.SnapLabel(wanted)
    else:
        reason = "%s unavailable for %s" % (
            gizmoSettings.SnapLabel(wanted), tool)
    return (gizmoSettings.SNAP_OFF, reason)


def ApplyDrag(state, current, settings, holdSnap=False, holdGrid=False,
              ctrl=False, *, snapMode=None, gridSize=1.0):
    """
    Run one mouse-move of a live drag: map the travel from the press to
    `current` into a channel edit and write it through the target.

    `settings` is the active tool's gizmoSettings.ToolSettings;
    `holdSnap` is Maya's J (step snap for the duration of the drag) and
    `holdGrid` its X (grid snap). `snapMode` is the ALREADY-RESOLVED
    active mode from ActiveSnapMode (the holds live in gizmoUI, so only
    it can resolve them); None means resolve from `settings.snapMode`
    alone, which is what the Qt-free tests use. `gridSize` is the world
    grid spacing from GizmoSettings -- a keyword because `settings` here
    is a per-tool ToolSettings. Returns what was applied -- a Gf.Vec3d
    for a move, degrees for a rotate, a factor for a scale -- so a
    caller (or a test) can see the decision without re-deriving it.

    Every Apply* on the target recomputes from the values BeginDrag()
    captured, so calling this repeatedly with the same `current` is
    idempotent and re-running it after a modifier changes is correct.
    """
    state.current = current
    state.ctrl = bool(ctrl)
    if snapMode is None:
        mode, _ = ActiveSnapMode(settings, state.tool, holdGrid,
                                 False, False)
    else:
        mode = snapMode
    if state.tool == TOOL_TRANSLATE:
        return _Translate(state, settings, holdSnap, holdGrid, mode,
                          gridSize)
    if state.tool == TOOL_ROTATE:
        return _Rotate(state, settings, holdSnap, mode)
    if state.tool == TOOL_SCALE:
        return _Scale(state, settings, holdSnap)
    return None


# ---------------------------------------------------------------------------
# Translate
# ---------------------------------------------------------------------------

def PlaneDelta(state, origin, normal):
    """
    A ray/plane drag delta, with a sanity net.

    The intersection is what keeps the grabbed point under the cursor as
    the plane recedes, but a plane that tips towards edge-on sends it
    towards infinity. The camera-plane delta for the same travel is
    always bounded, so a result wildly larger than that one is a miss
    and the last good delta stands.
    """
    delta = gizmoScreen.RayPlaneDragDelta(
        state.camera, state.viewport, origin, normal, state.press,
        state.current)
    reference = gizmoScreen.PlaneDragDelta(
        state.camera, state.viewport, origin, state.press, state.current)
    limit = max(reference.GetLength(), 1e-9) * PLANE_DELTA_SANITY
    if state.lastDelta is not None and delta.GetLength() > limit:
        return state.lastDelta
    state.lastDelta = delta
    return delta


def TranslateDelta(state):
    """
    The unsnapped world delta a move drag has asked for (spec 8.2).

    Axis: screen travel projected onto the handle. Ctrl+axis: the plane
    PERPENDICULAR to it. Planar handle: its own plane, through the
    square the artist grabbed, so that point stays under the cursor.
    Centre: the camera plane.
    """
    handle = state.handle
    origin = Gf.Vec3d(handle.worldCenter)
    if handle.kind == "axis":
        if state.ctrl:
            return PlaneDelta(state, origin, handle.worldAxis)
        parameter = gizmoScreen.AxisDragParameter(
            handle, state.press, state.current)
        return Gf.Vec3d(handle.worldAxis) * (parameter * handle.worldLength)
    if handle.kind == "plane":
        return PlaneDelta(state, origin, handle.worldNormal)
    return gizmoScreen.PlaneDragDelta(
        state.camera, state.viewport, origin, state.press, state.current)


def TranslateSnap(settings, holdSnap, holdGrid):
    """
    (snapStep, snapAbsolute) for Target.ApplyTranslate (spec 8.2).

    Step Snap or a held J is Maya's Discrete Move: the DELTA advances in
    whole steps, so an object that started off the grid stays off it.
    `holdGrid` is accepted but IGNORED: X used to mean the channel-space
    absolute grid here, but that put round channel numbers at arbitrary
    world positions under a posed parent, so X now snaps the WORLD pivot
    through gizmoSnap.GridPoint instead (snapping design section 2) and
    this function keeps only the relative-step behaviour. The parameter
    stays so the five kwarg-equality assertions and every call site keep
    working.
    """
    _ = holdGrid
    step = float(settings.stepSize)
    if step <= 0.0:
        return None, False
    if bool(settings.stepSnap) or bool(holdSnap):
        return step, False
    return None, False


def _Pivot0(state):
    """The gizmo origin the pivot must land from (spec 3, 4.2)."""
    origin = getattr(state.handle, "worldOrigin", None)
    if origin is None:
        origin = state.handle.worldCenter
    return Gf.Vec3d(origin)


def _ResolveCandidate(state, mode, current):
    """
    The resolver candidate for THIS event, or None.

    Throttled on cursor travel AND on the mode: state.snap may only be
    reused while mode == state.snapKind, because _ReapplyDrag re-runs
    the drag at the identical current on every hold press and release
    (gizmoUI.py:1900-1903) and a cached point would otherwise pose as an
    edge (snapping design 4.4). PICK_MOVE_PIXELS is PHYSICAL px, like
    press/current. A resolver throw degrades to no candidate, never a
    dead drag.
    """
    if (state.snapKind != mode or state.lastPickPoint is None):
        pass
    else:
        try:
            travel = math.hypot(current[0] - state.lastPickPoint[0],
                                current[1] - state.lastPickPoint[1])
        except Exception:
            travel = float("inf")
        if travel <= gizmoSnap.PICK_MOVE_PIXELS:
            cached = state.snap
            if cached is None:
                return None
            if getattr(cached, "kind", None) == mode:
                return cached
    state.snap = None
    candidate = None
    if state.snapResolver is not None:
        try:
            candidate = state.snapResolver(current[0], current[1],
                                           mode)
        except Exception:
            candidate = None
    state.snap = candidate
    state.snapKind = mode
    try:
        state.lastPickPoint = (current[0], current[1])
    except Exception:
        state.lastPickPoint = current
    return candidate


def _GridReason(handle, ctrl):
    """Status clause for the grid branch (snapping design 4.3).

    GridPoint quantises world COORDINATES only while the direction
    it acts on is world-aligned; otherwise it quantises the travel
    from the pivot, which does not land on the world grid, so the
    status line must say so rather than claim `grid: world`.
    Mirrors GridPoint's direction choice (gizmoSnap.py:170-205):
    a plane acts on its normal, an axis on its axis whether or
    not Ctrl turns it into a plane (gizmoDrag.py:186-188), and a
    centre handle -- like a missing/zero direction, which falls
    back to SnapAbsolute on all three world components -- is the
    world grid, so "".
    """
    _ = ctrl  # same axis with or without Ctrl, kept for the shape
    kind = getattr(handle, "kind", None)
    if kind == "plane":
        direction = getattr(handle, "worldNormal", None)
    elif kind == "axis":                      # ctrl or not: same axis
        direction = getattr(handle, "worldAxis", None)
    else:                                     # centre rounds all three
        return ""
    if direction is None:
        return ""
    vector = Gf.Vec3d(direction)
    if vector.GetLength() < 1e-12:
        return ""
    if gizmoSnap.DirectionIsWorldAligned(vector) is None:
        return "grid: relative (frame not world-aligned)"
    return ""


def _Translate(state, settings, holdSnap, holdGrid, snapMode, gridSize):
    # The world path (grid / point / edge / surface) writes a plain
    # world delta with no snapStep; the relative path (J / option)
    # quantises the channels in the target. Grid outranks J, so a live
    # world mode ignores holdSnap entirely.
    mode = snapMode
    if mode is None:
        mode = gizmoSettings.SNAP_OFF
    if mode == gizmoSettings.SNAP_OFF:
        delta = TranslateDelta(state)
        step, absolute = TranslateSnap(settings, holdSnap, holdGrid)
        state.target.ApplyTranslate(delta, snapStep=step,
                                    snapAbsolute=absolute)
        state.snap = None
        state.snapKind = mode
        state.snapReason = ""
        return delta
    pivot0 = _Pivot0(state)
    if mode == gizmoSettings.SNAP_GRID:
        delta = TranslateDelta(state)
        unsnapped = pivot0 + delta
        grid = gridSize if gridSize is not None else 1.0
        world = gizmoSnap.GridPoint(state.handle, pivot0, unsnapped,
                                    grid, ctrl=state.ctrl)
        constrained = gizmoSnap.ConstrainToHandle(state.handle, pivot0,
                                                  world, ctrl=state.ctrl)
        write = constrained - pivot0
        state.target.ApplyTranslate(write, snapStep=None,
                                    snapAbsolute=False)
        try:
            view = gizmoScreen.ViewProjection(state.camera)
            screen = gizmoScreen.ProjectPoint(view, state.viewport,
                                              constrained)
        except Exception:
            screen = None
        state.snap = gizmoSnap.SnapCandidate(
            constrained, Gf.Vec3d(0, 0, 0),
            gizmoSettings.SNAP_GRID, None, -1, screen)
        state.snapKind = mode
        state.snapReason = _GridReason(state.handle, state.ctrl)
        try:
            state.lastPickPoint = (state.current[0], state.current[1])
        except Exception:
            state.lastPickPoint = state.current
        return write
    candidate = _ResolveCandidate(state, mode, state.current)
    if candidate is None:
        state.snapReason = ""
        return Gf.Vec3d(0, 0, 0)
    constrained = gizmoSnap.ConstrainToHandle(
        state.handle, pivot0, Gf.Vec3d(candidate.point),
        ctrl=state.ctrl)
    write = constrained - pivot0
    state.target.ApplyTranslate(write, snapStep=None, snapAbsolute=False)
    state.snapReason = ""
    return write


# ---------------------------------------------------------------------------
# Rotate
# ---------------------------------------------------------------------------

def RingAngle(state):
    """
    Degrees swept about the grabbed ring's own axis since the press,
    accumulated so a drag can run past 180 (spec 8.3, "Maya keeps
    counting"). Updates the state's running total.
    """
    handle = state.handle
    raw = gizmoScreen.RotationDragAngle(
        handle.center, state.press, state.current,
        gizmoScreen.AxisFacesCamera(state.camera, handle.worldAxis))
    state.total = gizmoScreen.AccumulateAngle(state.total, state.raw, raw)
    state.raw = raw
    return state.total


def TrackballRotation(state):
    """
    The running world rotation of a free-rotate drag, as a Gf.Rotation.

    Each event's step is composed onto the running rotation and the
    whole thing is applied from the drag base, so a curved drag rolls
    the ball instead of snapping back to a single press-to-cursor axis.
    """
    step = gizmoScreen.TrackballRotation(
        state.camera, state.trackballLast, state.current,
        state.handle.radiusPixels)
    state.trackballLast = state.current
    if step is not None:
        axis, degrees = step
        matrix = Gf.Matrix4d(1.0)
        matrix.SetRotate(Gf.Rotation(axis, degrees))
        # Row-vector composition: the running rotation first, then this
        # step, because the step is expressed in world space.
        state.trackball = state.trackball * matrix
    return state.trackball.ExtractRotation()


def _Rotate(state, settings, holdSnap, snapMode):
    handle = state.handle
    mode = snapMode
    if mode is None:
        mode = gizmoSettings.SNAP_OFF
    # Point/edge/surface are Move-only: inert on Rotate with the reason
    # ActiveSnapMode would have given (snapping design 1.2). Do not fall
    # through to a lower snap; the status must name the held mode.
    downgraded = False
    if mode not in (gizmoSettings.SNAP_OFF, gizmoSettings.SNAP_GRID):
        state.snapReason = "%s is Move only" % (
            gizmoSettings.SnapLabel(mode))
        mode = gizmoSettings.SNAP_OFF
        downgraded = True
    if mode == gizmoSettings.SNAP_GRID:
        if state.gimbal and handle.kind == "ring":
            # Rotate's grid is degrees on the dragged gimbal channel
            # (snapping design 1.8, 4.3): the LANDED channel lands on a
            # multiple of stepSize, so the delta is recomputed from the
            # press-time base. No new Target API: rotationBase was read
            # once at the press because RotationState() is live
            # (gizmoMath.py:850-855). state.angle is set from the
            # absolute result, not _DisplayAngle, or the wedge and the
            # deg readout disagree with what was written.
            angle = RingAngle(state)
            base = None
            try:
                if state.rotationBase is not None:
                    base = float(
                        state.rotationBase[handle.axisIndex])
            except Exception:
                base = None
            if base is not None:
                step = float(settings.stepSize)
                if step > 0.0:
                    landed = gizmoScreen.SnapAbsolute(base + angle,
                                                      step)
                else:
                    landed = base + angle
                state.angle = landed - base
                state.snapReason = ""
                state.snapKind = mode
                state.target.ApplyRotateChannel(handle.axisIndex,
                                                state.angle,
                                                snapStep=None)
                return state.angle
        else:
            # A world-axis ring or the ball reaches the drawn rotation by
            # moving all three channels (snapping design 4.3), so there
            # is no single resulting channel to quantise. ActiveSnapMode
            # cannot make this call: it sees neither handle nor gimbal.
            state.snapReason = "Grid needs a Gimbal ring"
        # Inert grid falls through to the relative path below, so J
        # still works while the grid does nothing.
    elif not downgraded:
        state.snapReason = ""
    step = _SnapStep(settings, holdSnap)
    if handle.kind == "sphere":
        # Maya's Snap Rotate applies to the ball too: the axis the
        # trackball found is kept and only the amount is quantised.
        rotation = TrackballRotation(state)
        angle = rotation.GetAngle()
        state.angle = _DisplayAngle(angle, step)
        state.target.ApplyRotate(rotation.GetAxis(), angle, snapStep=step)
        return state.angle
    angle = RingAngle(state)
    state.angle = _DisplayAngle(angle, step)
    if state.gimbal and handle.kind == "ring":
        # Maya Gimbal: the ring IS one Euler channel, so the angle goes
        # straight onto that channel. ApplyRotate would take the
        # world-axis route, which under a sheared channel frame (a
        # non-uniform scale anywhere above) reaches the same drawn
        # rotation by moving all three channels -- correct geometry, but
        # not what a gimbal ring promises.
        state.target.ApplyRotateChannel(handle.axisIndex, angle,
                                        snapStep=step)
    else:
        state.target.ApplyRotate(handle.worldAxis, angle, snapStep=step)
    return state.angle


# ---------------------------------------------------------------------------
# Scale
# ---------------------------------------------------------------------------

def ScaleFactor(state, settings):
    """
    Maya's scale ratio for the grabbed handle: how far the cursor is
    from the manipulator origin along the handle over how far it was at
    the press, clamped just above zero when Prevent Negative Scale is on
    (spec 8.4). The RESULTING channel value is what Step Snap quantises,
    and only the target knows that, so no snapping happens here.
    """
    return gizmoScreen.MayaScaleFactor(
        state.handle, state.origin2d, state.press, state.current,
        not settings.preventNegativeScale)


def ScaleAxes(handle):
    """
    Which channels the grabbed handle scales, in the form
    Target.ApplyScale takes: None for uniform, an int for one axis, a
    tuple of the two axes IN the plane for a planar handle (whose own
    axisIndex names the axis perpendicular to it).
    """
    if handle.kind == "center":
        return None
    if handle.kind == "plane":
        return tuple(i for i in range(3) if i != handle.axisIndex)
    return handle.axisIndex


def _Scale(state, settings, holdSnap):
    factor = ScaleFactor(state, settings)
    state.target.ApplyScale(ScaleAxes(state.handle), factor,
                            snapStep=_SnapStep(settings, holdSnap))
    return factor
