#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoDrag.py: the manipulation
half of design spec section 8, driven with a synthetic Gf.Camera and a
recording fake target so no Qt and no stage are needed.

The camera is the one test_gizmo_screen.py uses -- at (0, 0, 10) looking
down -Z into an 800x600 viewport, conformed so pixels are square -- so
+X is screen-right, +Y is screen-up, and the gizmo origin projects to
(400, 300).

What is asserted is the DECISION, not the arithmetic gizmoScreen and
gizmoMath already test: which Target entry point a handle writes
through, which axes it names, that a ring keeps counting past 180, that
the ball composes its steps, and -- since the snapping itself now
happens inside the target, where the channel values are -- that J and X
arrive there as the right `snapStep` / `snapAbsolute` keywords.

Usage: test_gizmo_drag.py [ignored]
"""
import math
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf  # noqa: E402

import gizmoDrag as gd  # noqa: E402
import gizmoScreen as gs  # noqa: E402
import gizmoSettings as gset  # noqa: E402

VIEWPORT = (0, 0, 800, 600)
RATIO = 1.0


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _Camera():
    camera = Gf.Camera()
    camera.transform = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 0, 10))
    # usdview's StageView conforms the camera window to the viewport
    # before drawing; without the same conformance here a world unit
    # would cover a different number of pixels horizontally than
    # vertically and the handle layout would not be square.
    camera.verticalAperture = (camera.horizontalAperture
                               * VIEWPORT[3] / float(VIEWPORT[2]))
    return camera


class FakeTarget(object):
    """
    A Target that records what it was asked to do instead of authoring.

    Every Apply* records its keyword arguments as well as its
    positionals, because the snapping contract IS those keywords now:
    gizmoMath quantises the channel values, and all this module has to
    get right is which step reaches it and whether it is absolute.
    """

    supportsTranslate = True
    supportsRotate = True
    supportsScale = True
    supportsPreserveChildren = False
    preserveChildrenReason = ""
    label = "Fake"
    kind = "fake"

    def __init__(self):
        self.calls = []

    def ChannelFrame(self):
        return Gf.Matrix4d(1.0)

    def ObjectFrame(self):
        return Gf.Matrix4d(1.0)

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        self.calls.append(("translate", Gf.Vec3d(worldDelta),
                           {"snapStep": snapStep,
                            "snapAbsolute": snapAbsolute}))

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        self.calls.append(("rotate", Gf.Vec3d(worldAxis), degrees,
                           {"snapStep": snapStep}))

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        self.calls.append(("rotateChannel", axisIndex, degrees,
                           {"snapStep": snapStep}))

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        self.calls.append(("scale", axisIndex, factor,
                           {"snapStep": snapStep}))

    def Last(self):
        return self.calls[-1] if self.calls else None

    def LastKwargs(self):
        return self.calls[-1][-1] if self.calls else {}


def _TiltedCamera():
    """
    A camera looking at the origin from a generic direction.

    The axis-aligned camera above is what makes the "+X is right"
    assertions readable, but it leaves every axis-normal plane exactly
    edge-on, where RayPlaneDragDelta deliberately falls back to the
    camera plane. Anything testing a planar constraint needs a view in
    which the planes are actually facing.
    """
    camera = Gf.Camera()
    view = Gf.Matrix4d(1.0).SetLookAt(Gf.Vec3d(6, 5, 8), Gf.Vec3d(0, 0, 0),
                                      Gf.Vec3d(0, 1, 0))
    camera.transform = view.GetInverse()
    camera.verticalAperture = (camera.horizontalAperture
                               * VIEWPORT[3] / float(VIEWPORT[2]))
    return camera


def _Handles(tool, camera=None, **kwargs):
    """The handle set for `tool` at the world origin, as BuildHandles."""
    return gs.BuildHandles(tool, Gf.Matrix4d(1.0),
                           camera if camera is not None else _Camera(),
                           VIEWPORT, RATIO, **kwargs)


def _Named(handles, name):
    for handle in handles:
        if handle.name == name:
            return handle
    raise AssertionError("no handle named %s in %s" % (name, handles))


def _State(tool, handle, press, target, camera=None, **kwargs):
    return gd.DragState(tool, handle, target, press,
                        camera if camera is not None else _Camera(),
                        VIEWPORT, **kwargs)


def _Settings(tool):
    return gset.MayaDefaults(tool)


# ---------------------------------------------------------------------
# Translate
# ---------------------------------------------------------------------

def TestAxisTranslate():
    """
    Dragging the X arrow moves along world X by the travel expressed in
    handle lengths -- and by nothing else.
    """
    target = FakeTarget()
    handles = _Handles(gs.TOOL_TRANSLATE)
    axis = _Named(handles, "x")
    press = (axis.points[0][0] + 20.0, axis.points[0][1])
    state = _State(gs.TOOL_TRANSLATE, axis, press, target)
    length = axis.points[1][0] - axis.points[0][0]

    delta = gd.ApplyDrag(state, (press[0] + length * 0.5, press[1]),
                         _Settings(gs.TOOL_TRANSLATE))
    _Check(target.Last()[0] == "translate", "the axis drag translated")
    _Check(_Close(delta[0], axis.worldLength * 0.5, 1e-6),
           "half a handle length along X: %s" % (delta,))
    _Check(_Close(delta[1], 0.0) and _Close(delta[2], 0.0),
           "an axis drag moves on ONE axis: %s" % (delta,))

    # Travel perpendicular to the axis is projected away, not obeyed.
    delta = gd.ApplyDrag(state, (press[0], press[1] - 60.0),
                         _Settings(gs.TOOL_TRANSLATE))
    _Check(_Close(delta.GetLength(), 0.0, 1e-9),
           "travel across the axis does not move it: %s" % (delta,))


def TestCtrlAxisTranslate():
    """
    Maya's Ctrl+axis: the drag moves in the plane PERPENDICULAR to the
    axis, so the axis's own component stays put.
    """
    # A tilted view: with the camera down -Z the plane normal to X is
    # exactly edge-on, and RayPlaneDragDelta correctly refuses it.
    camera = _TiltedCamera()
    target = FakeTarget()
    axis = _Named(_Handles(gs.TOOL_TRANSLATE, camera=camera), "x")
    press = ((axis.points[0][0] + axis.points[1][0]) * 0.5,
             (axis.points[0][1] + axis.points[1][1]) * 0.5)
    state = _State(gs.TOOL_TRANSLATE, axis, press, target, camera=camera)
    current = (press[0] + 40.0, press[1] - 30.0)

    plain = gd.ApplyDrag(state, current, _Settings(gs.TOOL_TRANSLATE))
    _Check(abs(plain[0]) > 1e-9 and _Close(plain[1], 0.0, 1e-9)
           and _Close(plain[2], 0.0, 1e-9),
           "without Ctrl the drag is on X only: %s" % (plain,))

    state.lastDelta = None
    held = gd.ApplyDrag(state, current, _Settings(gs.TOOL_TRANSLATE),
                        ctrl=True)
    _Check(_Close(held[0], 0.0, 1e-9),
           "Ctrl+axis leaves the axis component alone: %s" % (held,))
    _Check(held.GetLength() > 1e-6,
           "Ctrl+axis moves in the perpendicular plane: %s" % (held,))


def TestPlaneTranslate():
    """A planar handle moves in its own plane and nowhere else."""
    target = FakeTarget()
    plane = _Named(_Handles(gs.TOOL_TRANSLATE), "xy")
    press = plane.worldCenterScreen
    state = _State(gs.TOOL_TRANSLATE, plane, press, target)
    delta = gd.ApplyDrag(state, (press[0] + 50.0, press[1] - 25.0),
                         _Settings(gs.TOOL_TRANSLATE))
    _Check(_Close(Gf.Dot(delta, Gf.Vec3d(plane.worldNormal)), 0.0, 1e-9),
           "the delta lies in the handle's plane: %s" % (delta,))
    _Check(delta[0] > 0.0 and delta[1] > 0.0,
           "right and up on screen is +X +Y here: %s" % (delta,))


def TestCentreTranslate():
    """The centre handle slides in the camera plane."""
    target = FakeTarget()
    centre = _Named(_Handles(gs.TOOL_TRANSLATE), "center")
    press = centre.points[0]
    state = _State(gs.TOOL_TRANSLATE, centre, press, target)
    delta = gd.ApplyDrag(state, (press[0] + 40.0, press[1] + 40.0),
                         _Settings(gs.TOOL_TRANSLATE))
    _Check(_Close(delta[2], 0.0, 1e-9),
           "the camera looks down Z, so the drag has no Z: %s" % (delta,))
    _Check(delta[0] > 0.0 and delta[1] < 0.0,
           "screen y grows downward: %s" % (delta,))


def TestTranslateSnapping():
    """
    Maya's two move snaps reach the target as keywords, and the WORLD
    delta is handed over untouched -- quantising it here would put the
    channels off the grid the moment the channel frame is rotated.
    """
    axis = _Named(_Handles(gs.TOOL_TRANSLATE), "x")
    press = (axis.points[0][0] + 20.0, axis.points[0][1])
    current = (press[0] + 55.0, press[1])
    settings = _Settings(gs.TOOL_TRANSLATE)
    settings.stepSize = 0.25

    target = FakeTarget()
    state = _State(gs.TOOL_TRANSLATE, axis, press, target)

    raw = gd.ApplyDrag(state, current, settings)
    _Check(target.LastKwargs() == {"snapStep": None, "snapAbsolute": False},
           "no snap by default: %s" % (target.LastKwargs(),))

    held = gd.ApplyDrag(state, current, settings, holdSnap=True)
    _Check(target.LastKwargs() == {"snapStep": 0.25, "snapAbsolute": False},
           "J is the relative step: %s" % (target.LastKwargs(),))

    settings.stepSnap = True
    gd.ApplyDrag(state, current, settings)
    settings.stepSnap = False
    _Check(target.LastKwargs() == {"snapStep": 0.25, "snapAbsolute": False},
           "the Step Snap option matches the J hold: %s"
           % (target.LastKwargs(),))

    grid = gd.ApplyDrag(state, current, settings, holdGrid=True)
    _Check(target.LastKwargs() == {"snapStep": 0.25, "snapAbsolute": True},
           "X is the absolute grid: %s" % (target.LastKwargs(),))

    both = gd.ApplyDrag(state, current, settings, holdSnap=True,
                        holdGrid=True)
    _Check(target.LastKwargs()["snapAbsolute"] is True,
           "X wins when both are held: %s" % (target.LastKwargs(),))

    _Check(raw == held == grid == both,
           "the world delta is never touched by the snapping: %s %s %s %s"
           % (raw, held, grid, both))

    settings.stepSize = 0.0
    gd.ApplyDrag(state, current, settings, holdSnap=True, holdGrid=True)
    _Check(target.LastKwargs() == {"snapStep": None, "snapAbsolute": False},
           "a zero step is no step, not a division by zero: %s"
           % (target.LastKwargs(),))


def TestTranslateSnapChoice():
    """TranslateSnap on its own, which is the whole of the J / X rule."""
    settings = _Settings(gs.TOOL_TRANSLATE)
    settings.stepSize = 2.0
    _Check(gd.TranslateSnap(settings, False, False) == (None, False),
           "nothing held, nothing snapped")
    _Check(gd.TranslateSnap(settings, True, False) == (2.0, False),
           "J is relative")
    _Check(gd.TranslateSnap(settings, False, True) == (2.0, True),
           "X is absolute")
    _Check(gd.TranslateSnap(settings, True, True) == (2.0, True),
           "X wins over J")
    settings.stepSnap = True
    _Check(gd.TranslateSnap(settings, False, False) == (2.0, False),
           "the option is the same as the J hold")


# ---------------------------------------------------------------------
# Rotate
# ---------------------------------------------------------------------

def TestRingRotate():
    """A ring drag turns about that ring's own world axis."""
    target = FakeTarget()
    ring = _Named(_Handles(gs.TOOL_ROTATE), "z")
    press = ring.points[0]
    state = _State(gs.TOOL_ROTATE, ring, press, target)
    angle = gd.ApplyDrag(state, (press[0], press[1] + 40.0),
                         _Settings(gs.TOOL_ROTATE))
    kind, axis, degrees = target.Last()[:3]
    _Check(kind == "rotate", "a ring drag goes through ApplyRotate")
    _Check(_Close(Gf.Dot(axis, Gf.Vec3d(ring.worldAxis)), 1.0, 1e-9),
           "about the ring's own axis: %s" % (axis,))
    _Check(_Close(degrees, angle), "the applied angle is the returned one")
    _Check(abs(angle) > 1.0, "the drag actually swept something: %s" % angle)


def TestRotateAccumulatesPast180():
    """
    Maya keeps counting: walking the cursor right round the ring passes
    180 without wrapping and returns to ~360 for a full turn.
    """
    target = FakeTarget()
    ring = _Named(_Handles(gs.TOOL_ROTATE), "z")
    centre = ring.center
    radius = 60.0
    press = (centre[0] + radius, centre[1])
    state = _State(gs.TOOL_ROTATE, ring, press, target)
    seen = []
    steps = 24
    for step in range(1, steps + 1):
        theta = 2.0 * math.pi * step / steps
        point = (centre[0] + radius * math.cos(theta),
                 centre[1] - radius * math.sin(theta))
        seen.append(gd.ApplyDrag(state, point, _Settings(gs.TOOL_ROTATE)))
    _Check(max(abs(a) for a in seen) > 180.0,
           "the sweep passes 180: max %.1f" % max(abs(a) for a in seen))
    _Check(_Close(abs(seen[-1]), 360.0, 1e-6),
           "a full turn is 360, not 0: %.3f" % seen[-1])
    monotonic = all(abs(seen[i]) >= abs(seen[i - 1]) - 1e-9
                    for i in range(1, len(seen)))
    _Check(monotonic, "the accumulated angle never jumps back")


def TestViewRingRotate():
    """The view ring turns about the camera axis, through ApplyRotate."""
    target = FakeTarget()
    view = _Named(_Handles(gs.TOOL_ROTATE), "view")
    press = view.points[0]
    state = _State(gs.TOOL_ROTATE, view, press, target, gimbal=True)
    gd.ApplyDrag(state, (press[0], press[1] + 40.0),
                 _Settings(gs.TOOL_ROTATE))
    kind, axis, _ = target.Last()[:3]
    _Check(kind == "rotate",
           "the view ring uses the world route even in Gimbal: %s" % kind)
    _Check(_Close(abs(Gf.Dot(axis, Gf.Vec3d(0, 0, 1))), 1.0, 1e-6),
           "about the view direction: %s" % (axis,))


def TestGimbalRingRotate():
    """
    A gimbal ring writes ONE Euler channel, by exactly the swept angle
    (spec 8.3). The axis-to-channel mapping is the handle's axisIndex.
    """
    axes = [Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)]
    handles = _Handles(gs.TOOL_ROTATE, gimbalAxes=axes)
    for index, name in enumerate(("x", "y", "z")):
        target = FakeTarget()
        ring = _Named(handles, name)
        press = ring.frontPoints[0][0]
        state = _State(gs.TOOL_ROTATE, ring, press, target, gimbal=True)
        angle = gd.ApplyDrag(state, (press[0] + 25.0, press[1] + 25.0),
                             _Settings(gs.TOOL_ROTATE))
        kind, axisIndex, degrees = target.Last()[:3]
        _Check(kind == "rotateChannel",
               "%s gimbal ring writes a channel, not a world axis: %s"
               % (name, kind))
        _Check(axisIndex == index,
               "%s ring drives channel %d, got %d"
               % (name, index, axisIndex))
        _Check(_Close(degrees, angle), "by the swept angle")

    # Without the gimbal flag the same ring takes the world-axis route.
    target = FakeTarget()
    ring = _Named(handles, "z")
    press = ring.frontPoints[0][0]
    state = _State(gs.TOOL_ROTATE, ring, press, target, gimbal=False)
    gd.ApplyDrag(state, (press[0] + 25.0, press[1] + 25.0),
                 _Settings(gs.TOOL_ROTATE))
    _Check(target.Last()[0] == "rotate",
           "Object/World rings still use ApplyRotate")


def TestTrackballRotate():
    """
    The ball composes each step into a running rotation applied from the
    base, so a bent drag ends somewhere a single press-to-cursor axis
    could not reach.
    """
    ball = _Named(_Handles(gs.TOOL_ROTATE), "free")
    press = ball.center
    settings = _Settings(gs.TOOL_ROTATE)

    straight = FakeTarget()
    state = _State(gs.TOOL_ROTATE, ball, press, straight)
    gd.ApplyDrag(state, (press[0] + 30.0, press[1]), settings)
    gd.ApplyDrag(state, (press[0] + 60.0, press[1]), settings)
    _, straightAxis, straightAngle = straight.Last()[:3]

    bent = FakeTarget()
    state = _State(gs.TOOL_ROTATE, ball, press, bent)
    gd.ApplyDrag(state, (press[0] + 30.0, press[1]), settings)
    gd.ApplyDrag(state, (press[0] + 30.0, press[1] + 30.0), settings)
    _, bentAxis, bentAngle = bent.Last()[:3]

    _Check(straight.Last()[0] == "rotate", "the ball uses ApplyRotate")
    _Check(_Close(abs(Gf.Dot(straightAxis, Gf.Vec3d(0, 1, 0))), 1.0, 1e-6),
           "a horizontal drag spins about screen up: %s" % (straightAxis,))
    _Check(abs(Gf.Dot(bentAxis, straightAxis)) < 0.999,
           "the bent drag ends on a different axis: %s vs %s"
           % (bentAxis, straightAxis))
    _Check(bentAngle > 1.0 and straightAngle > 1.0,
           "both swept something: %.2f %.2f" % (bentAngle, straightAngle))

    # Re-applying the same point must not double-count the rotation.
    steady = FakeTarget()
    state = _State(gs.TOOL_ROTATE, ball, press, steady)
    gd.ApplyDrag(state, (press[0] + 30.0, press[1]), settings)
    first = steady.Last()[2]
    gd.ApplyDrag(state, (press[0] + 30.0, press[1]), settings)
    _Check(_Close(steady.Last()[2], first),
           "a repeated event does not advance the ball: %.4f -> %.4f"
           % (first, steady.Last()[2]))


def TestRotateSnapping():
    """
    The step reaches the target as `snapStep`, the raw swept angle goes
    with it, and what comes BACK is the snapped angle -- because the pie
    wedge and the status readout have to show where the object landed,
    and the target returns nothing.
    """
    settings = _Settings(gs.TOOL_ROTATE)
    _Check(settings.stepSize == 15.0, "the rotate step starts at 15 degrees")

    target = FakeTarget()
    ring = _Named(_Handles(gs.TOOL_ROTATE), "z")
    press = ring.points[0]
    current = (press[0], press[1] + 45.0)
    state = _State(gs.TOOL_ROTATE, ring, press, target)

    raw = gd.ApplyDrag(state, current, settings)
    _Check(target.LastKwargs() == {"snapStep": None},
           "no snap by default: %s" % (target.LastKwargs(),))
    rawAngle = target.Last()[2]

    held = gd.ApplyDrag(state, current, settings, holdSnap=True)
    _Check(target.LastKwargs() == {"snapStep": 15.0},
           "J is the rotate step: %s" % (target.LastKwargs(),))
    _Check(_Close(target.Last()[2], rawAngle),
           "the RAW angle still goes to the target: %.4f vs %.4f"
           % (target.Last()[2], rawAngle))
    _Check(_Close(held % 15.0, 0.0, 1e-9) or _Close(held % 15.0, 15.0, 1e-9),
           "the reported angle is the snapped one: %.4f" % held)
    _Check(abs(held - raw) <= 7.5 + 1e-9,
           "to the NEAREST step: %.4f vs %.4f" % (held, raw))
    _Check(_Close(state.angle, held),
           "and the state carries it for the pie wedge")

    target = FakeTarget()
    ball = _Named(_Handles(gs.TOOL_ROTATE), "free")
    state = _State(gs.TOOL_ROTATE, ball, ball.center, target)
    held = gd.ApplyDrag(state, (ball.center[0] + 37.0, ball.center[1]),
                        settings, holdSnap=True)
    _Check(target.LastKwargs() == {"snapStep": 15.0},
           "Maya's snap rotate applies to the ball too: %s"
           % (target.LastKwargs(),))
    _Check(_Close(held % 15.0, 0.0, 1e-9) or _Close(held % 15.0, 15.0, 1e-9),
           "and the ball reports the snapped angle: %.4f" % held)


# ---------------------------------------------------------------------
# Scale
# ---------------------------------------------------------------------

def TestAxisScale():
    """An axis cube scales that axis by the Maya distance ratio."""
    target = FakeTarget()
    axis = _Named(_Handles(gs.TOOL_SCALE), "x")
    press = axis.points[1]
    state = _State(gs.TOOL_SCALE, axis, press, target)
    length = axis.points[1][0] - axis.points[0][0]
    factor = gd.ApplyDrag(state, (press[0] + length, press[1]),
                          _Settings(gs.TOOL_SCALE))
    kind, axisIndex, applied = target.Last()[:3]
    _Check(kind == "scale" and axisIndex == 0,
           "the X cube scales axis 0: %s" % (target.Last(),))
    _Check(_Close(applied, factor), "the applied factor is the returned one")
    _Check(_Close(factor, 2.0, 1e-6),
           "dragging the handle to twice its distance doubles: %.4f"
           % factor)


def TestPlaneScale():
    """
    A planar handle names the TWO axes of its plane, as the tuple
    Target.ApplyScale takes -- and NOT the axis its own axisIndex
    carries, which is the one perpendicular to it.
    """
    target = FakeTarget()
    plane = _Named(_Handles(gs.TOOL_SCALE), "xy")
    press = plane.worldCenterScreen
    state = _State(gs.TOOL_SCALE, plane, press, target)
    gd.ApplyDrag(state, (press[0] + 20.0, press[1] - 20.0),
                 _Settings(gs.TOOL_SCALE))
    axes = target.Last()[1]
    _Check(axes == (0, 1),
           "the xy plane names axes 0 and 1 as a tuple: %r" % (axes,))


def TestCentreScale():
    """The centre cube is uniform scale (axis index None)."""
    target = FakeTarget()
    centre = _Named(_Handles(gs.TOOL_SCALE), "center")
    press = centre.points[0]
    state = _State(gs.TOOL_SCALE, centre, press, target)
    factor = gd.ApplyDrag(state, (press[0] + 30.0, press[1]),
                          _Settings(gs.TOOL_SCALE))
    axes = target.Last()[1]
    _Check(axes is None, "uniform scale passes None: %r" % (axes,))
    _Check(factor > 1.0, "rightward travel grows it: %.4f" % factor)


def TestScaleNegativeAndSnapping():
    """
    Prevent Negative Scale clamps a drag through the origin, and Step
    Snap quantises the ratio.
    """
    axis = _Named(_Handles(gs.TOOL_SCALE), "x")
    press = axis.points[1]
    through = (axis.points[0][0] - (press[0] - axis.points[0][0]),
               press[1])
    settings = _Settings(gs.TOOL_SCALE)

    target = FakeTarget()
    state = _State(gs.TOOL_SCALE, axis, press, target)
    factor = gd.ApplyDrag(state, through, settings)
    _Check(factor < 0.0,
           "by default a drag through the origin mirrors: %.4f" % factor)

    settings.preventNegativeScale = True
    target = FakeTarget()
    state = _State(gs.TOOL_SCALE, axis, press, target)
    factor = gd.ApplyDrag(state, through, settings)
    _Check(factor >= gs.MIN_SCALE_FACTOR,
           "Prevent Negative Scale clamps it: %.6f" % factor)
    settings.preventNegativeScale = False

    # Step Snap travels as a keyword; the RATIO is handed over untouched
    # because it is the resulting channel value that lands on the grid,
    # and only the target knows what that value was.
    settings.stepSize = 0.5
    target = FakeTarget()
    state = _State(gs.TOOL_SCALE, axis, press, target)
    length = axis.points[1][0] - axis.points[0][0]
    current = (press[0] + length * 0.6, press[1])
    raw = gd.ApplyDrag(state, current, settings)
    _Check(target.LastKwargs() == {"snapStep": None},
           "no snap by default: %s" % (target.LastKwargs(),))
    held = gd.ApplyDrag(state, current, settings, holdSnap=True)
    _Check(target.LastKwargs() == {"snapStep": 0.5},
           "J is the scale step: %s" % (target.LastKwargs(),))
    _Check(_Close(held, raw),
           "and the ratio itself is unchanged: %.6f vs %.6f" % (held, raw))


def TestScaleAxesMapping():
    """ScaleAxes names the right thing for every scale handle kind."""
    handles = _Handles(gs.TOOL_SCALE)
    _Check(gd.ScaleAxes(_Named(handles, "center")) is None, "centre uniform")
    _Check(gd.ScaleAxes(_Named(handles, "y")) == 1, "the Y cube is axis 1")
    yz = gd.ScaleAxes(_Named(handles, "yz"))
    _Check(yz == (1, 2),
           "the yz plane names axes 1 and 2, not 0: %r" % (yz,))
    _Check(gd.ScaleAxes(_Named(handles, "xz")) == (0, 2),
           "and the xz plane names 0 and 2")


def main():
    groups = [
        ("axis translate", TestAxisTranslate),
        ("ctrl+axis translate", TestCtrlAxisTranslate),
        ("plane translate", TestPlaneTranslate),
        ("centre translate", TestCentreTranslate),
        ("translate snapping", TestTranslateSnapping),
        ("translate snap choice", TestTranslateSnapChoice),
        ("ring rotate", TestRingRotate),
        ("rotate accumulates past 180", TestRotateAccumulatesPast180),
        ("view ring rotate", TestViewRingRotate),
        ("gimbal ring rotate", TestGimbalRingRotate),
        ("trackball rotate", TestTrackballRotate),
        ("rotate snapping", TestRotateSnapping),
        ("axis scale", TestAxisScale),
        ("plane scale", TestPlaneScale),
        ("centre scale", TestCentreScale),
        ("scale negative and snapping", TestScaleNegativeAndSnapping),
        ("scale axes mapping", TestScaleAxesMapping),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_DRAG_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
