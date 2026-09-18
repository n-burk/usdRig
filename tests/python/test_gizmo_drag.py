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
the ball composes its steps, that J arrives at the target as the right
`snapStep` keyword while the WORLD snaps (X grid, point, edge, surface)
arrive as plain world deltas through gizmoSnap -- X no longer sends any
keyword, because channel-space rounding puts round channels at arbitrary
world positions under a posed parent (snapping design section 2).

Usage: test_gizmo_drag.py [ignored]
"""
import math
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Usd, UsdGeom  # noqa: E402

import gizmoDrag as gd  # noqa: E402
import gizmoScreen as gs  # noqa: E402
import gizmoSettings as gset  # noqa: E402
import gizmoMath as gm  # noqa: E402
import gizmoSnap as snap  # noqa: E402

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
    positionals, because the relative-snapping contract IS those keywords
    now: gizmoMath quantises the channel values, and all this module has
    to get right for J is which step reaches it. The world snaps instead
    arrive as plain world deltas with no step, which the recorded delta
    shows.
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

    def RotationState(self):
        # The base class carries this (gizmoMath.py:850-855), so the
        # double does too; DragState reads it once at the press for
        # Rotate's absolute grid.
        return ("xyz", [0.0, 0.0, 0.0])

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
    return gset.ToolDefaults(tool)


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
    the conventional Ctrl+axis: the drag moves in the plane PERPENDICULAR to the
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
    J reaches the target as a keyword; X snaps the WORLD delta through
    gizmoSnap and sends no keyword at all (snapping design section 2).
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

    # The grid path rounds the WORLD position onto the lattice, so
    # the assertion names the landed station: with a 0.1 grid the
    # 0.28813 unsnapped X delta lands the pivot on 0.3. The old
    # gridSize-1.0 form snapped back to the pivot and proved only
    # "different from raw", which a branch returning (0, 0, 0) for
    # every grid drag would satisfy trivially.
    grid = gd.ApplyDrag(state, current, settings, holdGrid=True,
                        gridSize=0.1)
    _Check(target.LastKwargs() == {"snapStep": None, "snapAbsolute": False},
           "X sends no keyword, it snaps the world delta: %s"
           % (target.LastKwargs(),))
    pivot0 = Gf.Vec3d(axis.worldOrigin)
    landed = pivot0 + grid
    _Check(_Close(landed[0], 0.3, 1e-9)
           and _Close(landed[1], 0.0, 1e-9)
           and _Close(landed[2], 0.0, 1e-9),
           "the 0.28813 drag lands the pivot on the 0.1 grid: %s"
           % (landed,))

    both = gd.ApplyDrag(state, current, settings, holdSnap=True,
                        holdGrid=True, gridSize=0.1)
    _Check(target.LastKwargs() == {"snapStep": None, "snapAbsolute": False},
           "X wins when both are held, so still no keyword: %s"
           % (target.LastKwargs(),))

    _Check(raw == held,
           "J and the option never touch the world delta: %s %s"
           % (raw, held))
    _Check(grid == both and grid != raw,
           "the grid path snaps the world delta; J and the option "
           "do not: %s %s %s" % (raw, grid, both))

    settings.stepSize = 0.0
    gd.ApplyDrag(state, current, settings, holdSnap=True)
    _Check(target.LastKwargs() == {"snapStep": None, "snapAbsolute": False},
           "a zero step is no step, not a division by zero: %s"
           % (target.LastKwargs(),))
    # Grid reads gridSize, not stepSize: with a zero step the grid still
    # writes a multiple of 1e-4.
    fine = gd.ApplyDrag(state, current, settings, holdGrid=True,
                        gridSize=1e-4)
    _Check(target.LastKwargs() == {"snapStep": None, "snapAbsolute": False},
           "the grid path still sends no keyword: %s"
           % (target.LastKwargs(),))
    _Check(abs(fine[0] / 1e-4 - round(fine[0] / 1e-4)) < 1e-6,
           "the grid delta is a multiple of 1e-4: %s" % (fine,))


def TestTranslateSnapChoice():
    """TranslateSnap on its own, which is now only the J rule."""
    settings = _Settings(gs.TOOL_TRANSLATE)
    settings.stepSize = 2.0
    _Check(gd.TranslateSnap(settings, False, False) == (None, False),
           "nothing held, nothing snapped")
    _Check(gd.TranslateSnap(settings, True, False) == (2.0, False),
           "J is relative")
    _Check(gd.TranslateSnap(settings, False, True) == (None, False),
           "X no longer influences TranslateSnap: the grid goes "
           "through gizmoSnap.GridPoint")
    _Check(gd.TranslateSnap(settings, True, True) == (2.0, False),
           "J alone decides when both are held")
    settings.stepSnap = True
    _Check(gd.TranslateSnap(settings, False, False) == (2.0, False),
           "the option is the same as the J hold")


def TestActiveSnapMode():
    """ActiveSnapMode precedence, the way TranslateSnapChoice pins J."""
    move = _Settings(gs.TOOL_TRANSLATE)
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, False, False,
                             False) == (gset.SNAP_OFF, ""),
           "nothing held, sticky off: off")
    for sticky in (gset.SNAP_GRID, gset.SNAP_POINT, gset.SNAP_EDGE,
                   gset.SNAP_SURFACE):
        move.snapMode = sticky
        mode, reason = gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, False,
                                         False, False)
        _Check(mode == sticky and reason == "",
               "sticky %s alone is %s" % (sticky, mode))
    move.snapMode = gset.SNAP_OFF
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, True, False,
                             False)[0] == gset.SNAP_GRID,
           "X alone is grid")
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, False, True,
                             False)[0] == gset.SNAP_POINT,
           "V alone is point")
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, False, False,
                             True)[0] == gset.SNAP_EDGE,
           "C alone is edge")
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, True, True,
                             False)[0] == gset.SNAP_POINT,
           "V outranks X")
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, True, False,
                             True)[0] == gset.SNAP_EDGE,
           "C outranks X")
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, False, True,
                             True)[0] == gset.SNAP_POINT,
           "V outranks C")
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, True, True,
                             True)[0] == gset.SNAP_POINT,
           "V outranks both")
    # A hold outranks a conflicting sticky mode; press order is never
    # read, only the booleans.
    move.snapMode = gset.SNAP_POINT
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, True, False,
                             False)[0] == gset.SNAP_GRID,
           "X beats a sticky point")
    move.snapMode = gset.SNAP_GRID
    _Check(gd.ActiveSnapMode(move, gs.TOOL_TRANSLATE, False, True,
                             False)[0] == gset.SNAP_POINT,
           "V beats a sticky grid")
    # Rotate offers only off and grid: point/edge/surface holds are
    # inert with a reason, never a silent fall-through to grid.
    mode, reason = gd.ActiveSnapMode(move, gs.TOOL_ROTATE, True, False,
                                     False)
    _Check(mode == gset.SNAP_GRID and reason == "",
           "rotate keeps its grid hold")
    for holdGrid, holdPoint, holdEdge, label in (
            (False, True, False, "point"),
            (False, False, True, "edge"),
            (True, True, False, "point"),
            (False, True, True, "point")):
        mode, reason = gd.ActiveSnapMode(move, gs.TOOL_ROTATE, holdGrid,
                                         holdPoint, holdEdge)
        _Check(mode == gset.SNAP_OFF and reason != "",
               "rotate %s is off with a reason: %s" % (label, reason))
    _Check("Point" in gd.ActiveSnapMode(
        move, gs.TOOL_ROTATE, False, True, False)[1],
        "the rotate reason names the held mode")
    # Scale offers nothing: every hold is off with a reason.
    for holdGrid, holdPoint, holdEdge in ((True, False, False),
                                         (False, True, False),
                                         (False, False, True)):
        mode, reason = gd.ActiveSnapMode(move, gs.TOOL_SCALE, holdGrid,
                                         holdPoint, holdEdge)
        _Check(mode == gset.SNAP_OFF and reason != "",
               "scale holds are inert: %s" % (reason,))
    # A sticky mode the tool does not offer is inert too, with a reason.
    move.snapMode = gset.SNAP_POINT
    mode, reason = gd.ActiveSnapMode(move, gs.TOOL_ROTATE, False, False,
                                     False)
    _Check(mode == gset.SNAP_OFF and reason != "",
           "sticky point on rotate is inert: %s" % (reason,))


def TestSnapThrottle():
    """The pick throttle: travel AND mode key the cache (spec 4.4)."""
    centre = _Named(_Handles(gs.TOOL_TRANSLATE), "center")
    press = centre.points[0]
    calls = []

    def _Resolver(x, y, mode):
        calls.append((x, y, mode))
        return snap.SnapCandidate(Gf.Vec3d(5, 5, 5),
                                  Gf.Vec3d(0, 0, 1), mode,
                                  None, 7, (x, y))

    target = FakeTarget()
    state = _State(gs.TOOL_TRANSLATE, centre, press, target)
    state.snapResolver = _Resolver
    settings = _Settings(gs.TOOL_TRANSLATE)
    first = (press[0] + 20.0, press[1])
    gd.ApplyDrag(state, first, settings, snapMode=gset.SNAP_POINT)
    _Check(len(calls) == 1, "the first move picks: %d" % len(calls))
    _Check(state.snap is not None
           and state.snap.kind == gset.SNAP_POINT,
           "the candidate is cached with its kind")
    _Check(len(target.calls) == 1, "a candidate writes once")
    # RATIO is 1.0 here, so PHYSICAL px are cursor units: 1 px reuses.
    gd.ApplyDrag(state, (first[0] + 1.0, first[1]), settings,
                 snapMode=gset.SNAP_POINT)
    _Check(len(calls) == 1, "1 px away reuses without a pick: %d"
           % len(calls))
    gd.ApplyDrag(state, (first[0] + 5.0, first[1]), settings,
                 snapMode=gset.SNAP_POINT)
    _Check(len(calls) == 2, "5 px from the last pick picks again: %d"
           % len(calls))
    # Same cursor, different mode: the cached point must not pose as an
    # edge.
    at = (first[0] + 5.0, first[1])
    gd.ApplyDrag(state, at, settings, snapMode=gset.SNAP_EDGE)
    _Check(len(calls) == 3, "a mode change picks immediately: %d"
           % len(calls))
    _Check(state.snap.kind == gset.SNAP_EDGE,
           "the new candidate carries the new mode: %s" % state.snap)

    # A mode switch at an UNMOVED cursor after a mode that found
    # nothing must still re-pick: state.snap is None there, so the
    # cached-kind check never runs and only the snapKind key forces
    # the pick (gizmoDrag.py:346; spec 4.4, the _ReapplyDrag re-run).
    misses = []

    def _Sparse(x, y, mode):
        misses.append(mode)
        if mode == gset.SNAP_POINT:
            return None
        return snap.SnapCandidate(Gf.Vec3d(2, 2, 2),
                                  Gf.Vec3d(0, 0, 1), mode,
                                  None, 3, (x, y))

    target2 = FakeTarget()
    state2 = _State(gs.TOOL_TRANSLATE, centre, press, target2)
    state2.snapResolver = _Sparse
    still = (press[0] + 20.0, press[1])
    gd.ApplyDrag(state2, still, settings, snapMode=gset.SNAP_POINT)
    _Check(misses == [gset.SNAP_POINT] and state2.snap is None,
           "V found nothing and the miss is cached: %s" % (misses,))
    _Check(len(target2.calls) == 0, "no candidate means no write")
    gd.ApplyDrag(state2, still, settings, snapMode=gset.SNAP_EDGE)
    _Check(misses == [gset.SNAP_POINT, gset.SNAP_EDGE],
           "C at the same cursor re-picks over a cached miss: %s"
           % (misses,))
    _Check(state2.snap is not None
           and state2.snap.kind == gset.SNAP_EDGE,
           "the re-pick carries the new mode: %s" % (state2.snap,))
    _Check(len(target2.calls) == 1, "the re-pick writes once")


def TestSnapMissWritesNothing():
    """No candidate is no write at all (snapping design 1.4)."""
    centre = _Named(_Handles(gs.TOOL_TRANSLATE), "center")
    press = centre.points[0]
    current = (press[0] + 40.0, press[1] - 20.0)
    settings = _Settings(gs.TOOL_TRANSLATE)

    def _Miss(x, y, mode):
        return None

    def _Boom(x, y, mode):
        raise RuntimeError("the pick failed")

    for label, resolver in (("no resolver", None),
                            ("no candidate", _Miss),
                            ("the pick raised", _Boom)):
        target = FakeTarget()
        state = _State(gs.TOOL_TRANSLATE, centre, press, target)
        state.snapResolver = resolver
        out = gd.ApplyDrag(state, current, settings,
                           snapMode=gset.SNAP_POINT)
        _Check(target.calls == [],
               "%s: the object does not move, the conventional tool does not fall "
               "back to free dragging: %s" % (label, target.calls))
        _Check(out == Gf.Vec3d(0, 0, 0),
               "%s: and nothing is reported applied: %s"
               % (label, out))
        _Check(state.snap is None,
               "%s: the miss leaves no cached candidate: %s"
               % (label, state.snap))


def _FixedResolver(point):
    """A stub resolver that always offers the same off-axis world point."""
    def _Resolve(x, y, mode):
        return snap.SnapCandidate(Gf.Vec3d(point), Gf.Vec3d(0, 0, 1),
                                  mode, None, 3, (x, y))
    return _Resolve


def TestSnapCandidateWrite():
    """
    What a point/edge/surface candidate actually WRITES (spec 3, 4.2,
    1.4): measured from worldOrigin, constrained to the handle, and
    nothing at all when there is no candidate.
    """
    camera = _TiltedCamera()
    handles = _Handles(gs.TOOL_TRANSLATE, camera=camera)
    settings = _Settings(gs.TOOL_TRANSLATE)
    world = Gf.Vec3d(2.5, -1.25, 3.75)

    # A PLANAR handle carries its SQUARE's centre in worldCenter
    # (gizmoScreen.py:316-324); the pivot is worldOrigin, so a write
    # measured from worldCenter lands the offset square on the target.
    plane = _Named(handles, "xy")
    offset = (Gf.Vec3d(plane.worldCenter)
              - Gf.Vec3d(plane.worldOrigin)).GetLength()
    _Check(offset > 1e-3,
           "the fixture is only meaningful while the square is offset: "
           "%.6f" % offset)

    def _Miss(x, y, mode):
        return None

    def _Drag(handle, press, mode, resolver, ctrl=False):
        target = FakeTarget()
        state = _State(gs.TOOL_TRANSLATE, handle, press, target,
                       camera=camera)
        state.snapResolver = resolver
        out = gd.ApplyDrag(state, (press[0] + 40.0, press[1] - 20.0),
                           settings, snapMode=mode, ctrl=ctrl)
        return target, out

    resolve = _FixedResolver(world)

    # The centre takes the candidate whole: the write is the pivot's
    # travel to the world point, in every mode.
    centre = _Named(handles, "center")
    pivot0 = Gf.Vec3d(centre.worldOrigin)
    for mode in (gset.SNAP_POINT, gset.SNAP_EDGE, gset.SNAP_SURFACE):
        target, out = _Drag(centre, centre.points[0], mode, resolve)
        _Check(target.LastKwargs() == {"snapStep": None,
                                       "snapAbsolute": False},
               "%s writes with no keyword: %s" % (mode, target.Last()))
        _Check((out - (world - pivot0)).GetLength() < 1e-9,
               "%s lands the pivot on the candidate: %s"
               % (mode, out))

    # An axis keeps only the along-axis component of that travel.
    axis = _Named(handles, "x")
    pivot0 = Gf.Vec3d(axis.worldOrigin)
    direction = Gf.Vec3d(axis.worldAxis).GetNormalized()
    press = ((axis.points[0][0] + axis.points[1][0]) * 0.5,
             (axis.points[0][1] + axis.points[1][1]) * 0.5)
    target, out = _Drag(axis, press, gset.SNAP_POINT, resolve)
    expected = direction * Gf.Dot(world - pivot0, direction)
    _Check((out - expected).GetLength() < 1e-9,
           "the axis drag stops level with the target: %s" % (out,))

    # A plane drops the normal component; the write is measured from
    # the pivot, not the offset square (see the offset guard above,
    # without which the two readings would coincide).
    pivot0 = Gf.Vec3d(plane.worldOrigin)
    normal = Gf.Vec3d(plane.worldNormal).GetNormalized()
    target, out = _Drag(plane, plane.worldCenterScreen,
                        gset.SNAP_POINT, resolve)
    travel = world - pivot0
    expected = travel - normal * Gf.Dot(travel, normal)
    _Check((out - expected).GetLength() < 1e-9,
           "the planar write lands in the plane from the pivot: %s"
           % (out,))

    # Ctrl+axis is a plane on the snap path too (spec 4.2).
    travel = world - Gf.Vec3d(axis.worldOrigin)
    target, out = _Drag(axis, press, gset.SNAP_POINT, resolve,
                        ctrl=True)
    expected = travel - direction * Gf.Dot(travel, direction)
    _Check(abs(Gf.Dot(out, direction)) < 1e-9,
           "ctrl+axis keeps no along-axis component: %s" % (out,))
    _Check((out - expected).GetLength() < 1e-9,
           "ctrl+axis lands in the perpendicular plane: %s" % (out,))

    # And no candidate is no write, in every mode.
    for mode in (gset.SNAP_POINT, gset.SNAP_EDGE, gset.SNAP_SURFACE):
        target, out = _Drag(centre, centre.points[0], mode, _Miss)
        _Check(target.calls == [] and out == Gf.Vec3d(0, 0, 0),
               "%s miss writes nothing: %s %s"
               % (mode, target.calls, out))


def TestGridReason():
    """The grid status clause follows GridPoint's own branch (spec 4.3)."""
    clause = "grid: relative (frame not world-aligned)"
    handles = _Handles(gs.TOOL_TRANSLATE)
    _Check(gd._GridReason(_Named(handles, "x"), False) == "",
           "a world-aligned axis is the world grid")
    _Check(gd._GridReason(_Named(handles, "xy"), False) == "",
           "a world-aligned plane is the world grid")
    _Check(gd._GridReason(_Named(handles, "center"), False) == "",
           "the centre rounds all three world components")
    tilted = Gf.Matrix4d(1.0)
    tilted.SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), 45.0))
    turned = gs.BuildHandles(gs.TOOL_TRANSLATE, tilted, _Camera(),
                             VIEWPORT, RATIO)
    _Check(gd._GridReason(_Named(turned, "x"), False) == clause,
           "a tilted axis quantises the travel from the pivot")
    _Check(gd._GridReason(_Named(turned, "x"), True) == clause,
           "ctrl+axis reads the same axis either way")
    _Check(gd._GridReason(_Named(turned, "xy"), False) == clause,
           "a tilted plane quantises the travel from the pivot")
    # And the drag itself carries the clause on state.snapReason.
    axis = _Named(turned, "x")
    press = ((axis.points[0][0] + axis.points[1][0]) * 0.5,
             (axis.points[0][1] + axis.points[1][1]) * 0.5)
    state = _State(gs.TOOL_TRANSLATE, axis, press, FakeTarget())
    gd.ApplyDrag(state, (press[0] + 40.0, press[1] - 20.0),
                 _Settings(gs.TOOL_TRANSLATE), holdGrid=True)
    _Check(state.snapReason == clause,
           "the tilted grid drag says relative: %r"
           % state.snapReason)
    straight = _Named(handles, "x")
    press = ((straight.points[0][0] + straight.points[1][0]) * 0.5,
             (straight.points[0][1] + straight.points[1][1]) * 0.5)
    state = _State(gs.TOOL_TRANSLATE, straight, press, FakeTarget())
    gd.ApplyDrag(state, (press[0] + 40.0, press[1] - 20.0),
                 _Settings(gs.TOOL_TRANSLATE), holdGrid=True)
    _Check(state.snapReason == "",
           "the aligned grid drag reports nothing: %r"
           % state.snapReason)


def TestGridWorldNotChannels():
    """The assertion that proves X exists (spec section 6 item 5)."""
    stage = Usd.Stage.CreateInMemory()
    parent = UsdGeom.Xform.Define(stage, "/P")
    UsdGeom.XformCommonAPI(parent).SetRotate(Gf.Vec3f(0, 45, 0))
    child = UsdGeom.Xform.Define(stage, "/P/Child")
    UsdGeom.XformCommonAPI(child).SetTranslate(Gf.Vec3d(1.4, 0, 0))
    time = Usd.TimeCode.Default()
    writer = gm.Writer(stage, time, gm.WRITE_DEFAULT)
    target, reason = gm.MakeTarget(stage, child.GetPrim(),
                                   gm.CHANNELS_POSE, writer)
    _Check(target is not None, "xform target: %s" % reason)
    target.BeginDrag()
    matrix = target.GizmoMatrix()
    camera = _Camera()
    handles = gs.BuildHandles(gs.TOOL_TRANSLATE, matrix, camera,
                              VIEWPORT, RATIO)
    centre = _Named(handles, "center")
    press = centre.points[0]
    current = (press[0] + 50.0, press[1] - 25.0)
    state = gd.DragState(gs.TOOL_TRANSLATE, centre, target, press,
                         camera, VIEWPORT)
    settings = _Settings(gs.TOOL_TRANSLATE)
    gd.ApplyDrag(state, current, settings, holdGrid=True, gridSize=1.0)
    # A drag collects; a release authors (gizmoMath.Writer). The stage is read
    # below, so this is the release.
    writer.CommitToStage()
    cache = UsdGeom.XformCache(time)
    world = cache.GetLocalToWorldTransform(
        child.GetPrim()).ExtractTranslation()
    for index in range(3):
        _Check(abs(world[index] - round(world[index])) < 1e-6,
               "world lands on the grid: %s" % (world,))
    local = UsdGeom.XformCommonAPI(child).GetXformVectors(time)[0]
    off = [abs(local[i] - round(local[i])) > 1e-6 for i in range(3)]
    _Check(any(off),
           "the channels do not: %s (world %s)" % (local, world))


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
    the conventional tool keeps counting: walking the cursor right round the ring passes
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
           "the conventional snap rotate applies to the ball too: %s"
           % (target.LastKwargs(),))
    _Check(_Close(held % 15.0, 0.0, 1e-9) or _Close(held % 15.0, 15.0, 1e-9),
           "and the ball reports the snapped angle: %.4f" % held)


def TestRotateGrid():
    """Rotate's grid is absolute, and only on a Gimbal ring (spec 4.3)."""
    axes = [Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)]
    handles = _Handles(gs.TOOL_ROTATE, gimbalAxes=axes)
    settings = _Settings(gs.TOOL_ROTATE)
    settings.stepSize = 15.0

    target = FakeTarget()
    target.calls = []
    ring = _Named(handles, "x")
    press = ring.frontPoints[0][0]
    current = (press[0] + 25.0, press[1] + 25.0)
    state = _State(gs.TOOL_ROTATE, ring, press, target, gimbal=True)
    # A non-zero Euler base proves the grid is absolute, not relative:
    # ApplyRotateChannel(1, 40, snapStep=15) from base 3 lands 48, not
    # 45 (snapping design 4.3), while the grid lands base + angle on 45.
    state.rotationBase = [3.0, 0.0, 0.0]
    angle = gd.ApplyDrag(state, current, settings,
                         snapMode=gset.SNAP_GRID, gridSize=100.0)
    kind, index, applied = target.Last()[:3]
    _Check(kind == "rotateChannel",
           "the gimbal grid writes the channel: %s" % (target.Last(),))
    _Check(target.LastKwargs() == {"snapStep": None},
           "with no step keyword: %s" % (target.LastKwargs(),))
    _Check(_Close((3.0 + applied) % 15.0, 0.0, 1e-6)
           or _Close((3.0 + applied) % 15.0, 15.0, 1e-6),
           "the LANDED channel is on the step: base 3 + %.4f"
           % applied)
    _Check(_Close(state.angle, angle) and _Close(angle, applied),
           "the state carries the absolute angle for the wedge")
    _Check(state.snapReason == "",
           "no reason on the offered path: %r" % state.snapReason)

    # The same hold on a world-axis ring is inert: no single channel to
    # quantise, so the status names the ring it needs.
    target = FakeTarget()
    ring = _Named(_Handles(gs.TOOL_ROTATE), "z")
    press = ring.points[0]
    state = _State(gs.TOOL_ROTATE, ring, press, target, gimbal=False)
    gd.ApplyDrag(state, (press[0], press[1] + 40.0), settings,
                 snapMode=gset.SNAP_GRID)
    _Check(target.Last()[0] == "rotate",
           "the world ring stays on the world route: %s"
           % (target.Last(),))
    _Check(state.snapReason == "Grid needs a Gimbal ring",
           "and records why: %r" % state.snapReason)

    # The ball is inert the same way.
    target = FakeTarget()
    ball = _Named(_Handles(gs.TOOL_ROTATE), "free")
    state = _State(gs.TOOL_ROTATE, ball, ball.center, target)
    gd.ApplyDrag(state, (ball.center[0] + 37.0, ball.center[1]),
                 settings, snapMode=gset.SNAP_GRID)
    _Check(state.snapReason == "Grid needs a Gimbal ring",
           "the ball needs a ring too: %r" % state.snapReason)

    # Point is Move-only, even on a gimbal ring.
    target = FakeTarget()
    ring = _Named(handles, "x")
    press = ring.frontPoints[0][0]
    state = _State(gs.TOOL_ROTATE, ring, press, target, gimbal=True)
    state.rotationBase = [0.0, 0.0, 0.0]
    gd.ApplyDrag(state, (press[0] + 25.0, press[1] + 25.0), settings,
                 snapMode=gset.SNAP_POINT)
    _Check(state.snapReason != ""
           and "Move only" in state.snapReason,
           "point on rotate names itself inert: %r" % state.snapReason)


# ---------------------------------------------------------------------
# Scale
# ---------------------------------------------------------------------

def TestAxisScale():
    """An axis cube scales that axis by the distance ratio."""
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


def TestGlobalVersusLocalAxes():
    """
    The toolbar's Global/Local toggle changes the DRAWN AXES and the
    VALUES WRITTEN, not just a stored setting.

    Same prim, same handle, same screen travel, twice: Global lays the
    X arrow on world +X and Local on the prim's own +X, which on a prim
    rotated 40 degrees about Z is a different direction, a different
    world delta, and a different set of channel values. Driven through a
    real gizmoMath target rather than the FakeTarget above, because what
    is being asserted is the round trip -- orientation in, channels out.

    Measured on examples/biped/Biped.usda through the same path, dragging
    arm_l_fk_shoulder_l_bind's X arrow 120 px: Global wrote avars:t
    [5.694218, 0.361702, 5.758059] and moved it 8.1062 cm along world X;
    Local wrote [4.232632, 0, 0] -- a pure avars:tx, which is what local
    means -- and moved it 4.2326 cm along the shoulder's own X. The two
    world distances differ because the two arrows project to different
    screen lengths, so 120 px of cursor travel buys a different distance
    along each; that is the conventional behaviour too.
    """
    time = Usd.TimeCode.Default()
    results = {}
    for mode in ("global", "local"):
        stage = Usd.Stage.CreateInMemory()
        xform = UsdGeom.Xform.Define(stage, "/Box")
        api = UsdGeom.XformCommonAPI(xform)
        api.SetTranslate(Gf.Vec3d(0, 0, 0))
        api.SetRotate(Gf.Vec3f(0, 0, 40.0))
        api.SetScale(Gf.Vec3f(1, 1, 1))
        writer = gm.Writer(stage, time, gm.WRITE_DEFAULT)
        target, reason = gm.MakeTarget(stage, xform.GetPrim(),
                                       gm.CHANNELS_POSE, writer)
        _Check(target is not None, reason)
        orientation = (Gf.Matrix4d(1.0) if mode == "global"
                       else target.ObjectFrame())
        handles = gs.BuildHandles(gs.TOOL_TRANSLATE, target.GizmoMatrix(),
                                  _Camera(), VIEWPORT, RATIO,
                                  orientation=orientation)
        axis = _Named(handles, "x")
        press = (axis.points[0][0] * 0.4 + axis.points[1][0] * 0.6,
                 axis.points[0][1] * 0.4 + axis.points[1][1] * 0.6)
        state = _State(gs.TOOL_TRANSLATE, axis, press, target,
                       origin2d=axis.center)
        target.BeginDrag()
        delta = gd.ApplyDrag(state, (press[0] + 120.0, press[1]),
                             _Settings(gs.TOOL_TRANSLATE))
        writer.CommitToStage()
        results[mode] = (Gf.Vec3d(axis.worldAxis), Gf.Vec3d(delta),
                         Gf.Vec3d(api.GetXformVectors(time)[0]))

    worldAxis, worldDelta, worldValue = results["global"]
    localAxis, localDelta, localValue = results["local"]
    # The drawn axis: world +X against the prim's own +X, 40 degrees apart.
    _Check(_Close(worldAxis[0], 1.0) and _Close(worldAxis[1], 0.0),
           "Global draws the X arrow on world X: %s" % (worldAxis,))
    _Check(_Close(localAxis[0], math.cos(math.radians(40.0)), 1e-6)
           and _Close(localAxis[1], math.sin(math.radians(40.0)), 1e-6),
           "Local draws it on the prim's own X: %s" % (localAxis,))
    # The applied delta: along each of those, not along one of them twice.
    _Check(_Close(worldDelta[1], 0.0, 1e-9) and worldDelta[0] > 0.0,
           "the Global drag moved along world X: %s" % (worldDelta,))
    _Check(localDelta[1] > 0.0,
           "the Local drag moved off world X: %s" % (localDelta,))
    # And the values that landed on the stage differ, which is the part
    # a stored-setting-only bug would not reach.
    _Check((localValue - worldValue).GetLength() > 0.1,
           "different channel values: %s vs %s"
           % (worldValue, localValue))
    # WHICH channels each one needed is the mirror image of the rig
    # case in the docstring, and for a reason worth stating: an
    # XformCommonAPI translate op is in PARENT space and applies BEFORE
    # the rotate, so here it is GLOBAL that lands on one channel and
    # LOCAL that needs two. A rig control's avars:t is in P, which
    # carries the control's own rest orientation, so there it is the
    # other way round. Either way the two modes write different values,
    # which is the thing under test.
    _Check(abs(worldValue[1]) < 1e-9,
           "global is a pure parent-space X move: %s" % (worldValue,))
    _Check(abs(localValue[1]) > 1e-6,
           "local needed both channels: %s" % (localValue,))

def main():
    groups = [
        ("axis translate", TestAxisTranslate),
        ("ctrl+axis translate", TestCtrlAxisTranslate),
        ("plane translate", TestPlaneTranslate),
        ("centre translate", TestCentreTranslate),
        ("translate snapping", TestTranslateSnapping),
        ("translate snap choice", TestTranslateSnapChoice),
        ("active snap mode", TestActiveSnapMode),
        ("snap throttle", TestSnapThrottle),
        ("snap miss writes nothing", TestSnapMissWritesNothing),
        ("snap candidate write", TestSnapCandidateWrite),
        ("grid reason", TestGridReason),
        ("grid world not channels", TestGridWorldNotChannels),
        ("ring rotate", TestRingRotate),
        ("rotate accumulates past 180", TestRotateAccumulatesPast180),
        ("view ring rotate", TestViewRingRotate),
        ("gimbal ring rotate", TestGimbalRingRotate),
        ("trackball rotate", TestTrackballRotate),
        ("rotate snapping", TestRotateSnapping),
        ("rotate grid", TestRotateGrid),
        ("axis scale", TestAxisScale),
        ("plane scale", TestPlaneScale),
        ("centre scale", TestCentreScale),
        ("scale negative and snapping", TestScaleNegativeAndSnapping),
        ("scale axes mapping", TestScaleAxesMapping),
        ("global vs local axes", TestGlobalVersusLocalAxes),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_DRAG_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
