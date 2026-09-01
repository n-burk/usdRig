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
# SNAPPING IS THE TARGET'S JOB, not this module's. Maya's steps are
# steps of the CHANNEL values -- a move lands tx on a whole number, a
# scale lands sx on one -- and only the target knows those values and
# the frame they live in. So J, X and the Step Snap option travel as the
# `snapStep` / `snapAbsolute` keywords on Apply*, and nothing here
# rounds a world delta, an angle or a ratio. The one exception is
# _DisplayAngle below, which is a readout, not a decision.
#
from pxr import Gf

try:
    import gizmoScreen
except ImportError:                    # loader that did not add our dir
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoScreen

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


def ApplyDrag(state, current, settings, holdSnap=False, holdGrid=False,
              ctrl=False):
    """
    Run one mouse-move of a live drag: map the travel from the press to
    `current` into a channel edit and write it through the target.

    `settings` is the active tool's gizmoSettings.ToolSettings;
    `holdSnap` is Maya's J (step snap for the duration of the drag) and
    `holdGrid` its X (grid snap, move only). Returns what was applied --
    a Gf.Vec3d for a move, degrees for a rotate, a factor for a scale --
    so a caller (or a test) can see the decision without re-deriving it.

    Every Apply* on the target recomputes from the values BeginDrag()
    captured, so calling this repeatedly with the same `current` is
    idempotent and re-running it after a modifier changes is correct.
    """
    state.current = current
    state.ctrl = bool(ctrl)
    if state.tool == TOOL_TRANSLATE:
        return _Translate(state, settings, holdSnap, holdGrid)
    if state.tool == TOOL_ROTATE:
        return _Rotate(state, settings, holdSnap)
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
    whole steps, so an object that started off the grid stays off it. A
    held X is Maya's grid snap: the RESULT lands on the grid however the
    drag started. X wins when both are held, because it fully determines
    where the object ends up and the relative step then says nothing.
    """
    step = float(settings.stepSize)
    if step <= 0.0:
        return None, False
    if holdGrid:
        return step, True
    if bool(settings.stepSnap) or bool(holdSnap):
        return step, False
    return None, False


def _Translate(state, settings, holdSnap, holdGrid):
    delta = TranslateDelta(state)
    step, absolute = TranslateSnap(settings, holdSnap, holdGrid)
    state.target.ApplyTranslate(delta, snapStep=step,
                                snapAbsolute=absolute)
    return delta


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


def _Rotate(state, settings, holdSnap):
    handle = state.handle
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
