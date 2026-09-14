#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoMarquee.py -- the viewport's
native box selection of RigExec controls.

Usage: test_gizmo_marquee.py [ignored]

Everything is asserted in NUMBERS against examples/biped/Biped.usda and a
synthetic Gf.Camera at (0, 100, 400) looking down -Z into an 800x600
viewport, conformed the way usdview's StageView conforms its own camera.
The band coordinates below are the measured screen positions of the
biped's neck chain; if the rig moves they move with it and this test says
so rather than passing vacuously.
"""
import os
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
# SetupPluginTest is the shared form of test_rigexec_python's
# _setup_environment -- same sys.path, DLL-directory and plugin-path
# work, plus plugin/rigExecUsdview -- so this test runs from ctest, from
# bin/run_python_tests, and from a bare shell alike.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Sdf, Usd  # noqa: E402

import gizmoMarquee  # noqa: E402


VIEWPORT = (0, 0, 800, 600)

BIPED = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))),
    "examples", "biped", "Biped.usda")

CONTROLS = "/Biped/Rig/Controls"
NECK = CONTROLS + "/hips_ctl/torso_ctl/spine_end_pivot/spine_end_ctl" \
                  "/neck_root_ctl"

# The neck chain, top to bottom, measured at (400.0, 20.4), (400.0, 44.3),
# (400.0, 66.8) and (400.0, 88.5) physical pixels with the camera below.
NECK_CHAIN = sorted([
    NECK,
    NECK + "/neck_cv1",
    NECK + "/neck_end_pivot",
    NECK + "/neck_end_pivot/neck_end_ctl",
])

# The band that catches exactly those four and nothing else: the next
# control down the body, clavicle_l_ctl, is at y = 107.4.
NECK_BAND = (380.0, 5.0, 420.0, 100.0)

# Four of the 15 controls the rig poses outright through a
# RigExecParentConstraint. Named rather than only counted so a rig change
# that quietly stopped constraining one of them fails here.
OVERWRITTEN = (
    CONTROLS + "/spine_mid_follow",
    CONTROLS + "/index_001_l_bind_fk_follow",
    CONTROLS + "/thumbCup_r_bind_fk_follow",
    CONTROLS + "/arm_l_params",
)

# Measured on this asset: 126 RigExecControl prims, 15 of them overwritten
# by the rig, 111 box-selectable.
CONTROL_COUNT = 126
OVERWRITTEN_COUNT = 15
SELECTABLE_COUNT = 111


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Camera():
    camera = Gf.Camera()
    camera.transform = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 100, 400))
    # usdview's StageView conforms the camera window to the viewport
    # before drawing (see test_gizmo_screen), so the synthetic camera has
    # to conform too or a world unit covers a different number of pixels
    # horizontally than vertically.
    camera.verticalAperture = (camera.horizontalAperture
                               * VIEWPORT[3] / float(VIEWPORT[2]))
    return camera


def _Stage():
    """The biped on a fresh session layer.

    Usd.Stage.Open(path) reuses the globally cached SdfLayer, so opening
    by path would let one test's authored avars leak into the next.
    """
    layer = Sdf.Layer.FindOrOpen(BIPED)
    _Check(layer is not None, "cannot open %s" % BIPED)
    stage = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous())
    stage.SetEditTarget(stage.GetSessionLayer())
    return stage


def TestSelectable(stage):
    """Controls are box-selectable; the ones the rig overwrites are not."""
    controls = [str(p.GetPath()) for p in gizmoMarquee.ControlPrims(stage)]
    _Check(len(controls) == CONTROL_COUNT,
           "%d RigExecControl prims, expected %d"
           % (len(controls), CONTROL_COUNT))
    selectable = gizmoMarquee.SelectablePaths(stage)
    _Check(len(selectable) == SELECTABLE_COUNT,
           "%d selectable controls, expected %d"
           % (len(selectable), SELECTABLE_COUNT))
    dropped = set(controls) - set(selectable)
    _Check(len(dropped) == OVERWRITTEN_COUNT,
           "%d controls dropped, expected %d: %s"
           % (len(dropped), OVERWRITTEN_COUNT, sorted(dropped)))
    for path in OVERWRITTEN:
        _Check(path in dropped,
               "%s is a RigExecParentConstraint target and must not be "
               "box-selectable" % path)
    # Every dropped one is a path the rig poses outright -- the same set
    # the gizmo refuses to build a target for.
    root = stage.GetPrimAtPath(Sdf.Path("/Biped/Rig"))
    import gizmoMath
    posed = set(str(p) for p in gizmoMath.SolverPosedPaths(root))
    _Check(dropped <= posed, "dropped controls the rig does not pose: %s"
           % sorted(dropped - posed))


def TestBand(stage):
    """A band over a known screen region catches exactly one control set."""
    camera = _Camera()
    positions = gizmoMarquee.ScreenPositions(
        stage, camera, VIEWPORT, Usd.TimeCode.Default())
    _Check(len(positions) == SELECTABLE_COUNT,
           "%d projected controls, expected %d"
           % (len(positions), SELECTABLE_COUNT))
    caught = gizmoMarquee.PathsInBand(positions, *NECK_BAND)
    _Check(caught == NECK_CHAIN,
           "neck band caught %s, expected %s" % (caught, NECK_CHAIN))

    # The rectangle is normalised: dragging up-and-left is the same band.
    reversed_ = gizmoMarquee.PathsInBand(
        positions, NECK_BAND[2], NECK_BAND[3], NECK_BAND[0], NECK_BAND[1])
    _Check(reversed_ == NECK_CHAIN, "a band dragged backwards differs")

    # The convenience entry point agrees with the two-step form.
    direct = gizmoMarquee.ControlsInBand(
        stage, camera, VIEWPORT, Usd.TimeCode.Default(), NECK_BAND)
    _Check(direct == NECK_CHAIN, "ControlsInBand disagrees: %s" % direct)

    # A band that covers everything catches every selectable control and
    # not one overwritten one.
    everything = gizmoMarquee.PathsInBand(positions, -1e4, -1e4, 1e4, 1e4)
    _Check(everything == gizmoMarquee.SelectablePaths(stage),
           "a band over everything is not the selectable set")
    for path in OVERWRITTEN:
        _Check(path not in everything,
               "%s was box-selected" % path)

    # An empty patch of screen catches nothing. (100, 500) is clear of
    # every projected control by more than 20 px.
    _Check(gizmoMarquee.PathsInBand(positions, 95.0, 495.0, 105.0, 505.0)
           == [], "empty screen caught something")


def TestModes():
    """Plain replaces, Shift toggles, Ctrl only ever removes."""
    _Check(gizmoMarquee.Mode() == gizmoMarquee.MODE_REPLACE, "plain")
    _Check(gizmoMarquee.Mode(shift=True) == gizmoMarquee.MODE_TOGGLE,
           "shift")
    _Check(gizmoMarquee.Mode(ctrl=True) == gizmoMarquee.MODE_REMOVE, "ctrl")
    # Ctrl+Shift subtracts: there is one gesture that never adds.
    _Check(gizmoMarquee.Mode(ctrl=True, shift=True)
           == gizmoMarquee.MODE_REMOVE, "ctrl+shift must not add")

    a, b, c = "/a", "/b", "/c"
    replace = gizmoMarquee.Resolve([a, b], [b, c],
                                   gizmoMarquee.MODE_REPLACE)
    _Check(replace == [b, c], "replace: %s" % replace)

    # Shift ADDS what is not selected and REMOVES what is, in one band.
    toggle = gizmoMarquee.Resolve([a, b], [b, c],
                                  gizmoMarquee.MODE_TOGGLE)
    _Check(toggle == [a, c], "shift toggle: %s" % toggle)
    _Check(gizmoMarquee.Resolve([a], [b], gizmoMarquee.MODE_TOGGLE)
           == [a, b], "shift adds what is not selected")

    # Ctrl never adds, whatever the band contains.
    remove = gizmoMarquee.Resolve([a, b], [b, c],
                                  gizmoMarquee.MODE_REMOVE)
    _Check(remove == [a], "ctrl remove: %s" % remove)
    _Check(gizmoMarquee.Resolve([a], [b], gizmoMarquee.MODE_REMOVE) == [a],
           "ctrl must not add an unselected control")
    _Check(gizmoMarquee.Resolve([], [a, b], gizmoMarquee.MODE_REMOVE) == [],
           "ctrl on an empty selection stays empty")


def TestModesOnTheBand(stage):
    """The same three modes over the real neck band."""
    camera = _Camera()
    positions = gizmoMarquee.ScreenPositions(
        stage, camera, VIEWPORT, Usd.TimeCode.Default())
    caught = gizmoMarquee.PathsInBand(positions, *NECK_BAND)
    hips = CONTROLS + "/hips_ctl"

    plain = gizmoMarquee.Resolve([hips], caught,
                                 gizmoMarquee.MODE_REPLACE)
    _Check(plain == caught, "a plain band replaces the selection")

    added = gizmoMarquee.Resolve([hips], caught,
                                 gizmoMarquee.MODE_TOGGLE)
    _Check(added == [hips] + caught, "shift adds the band: %s" % added)

    # Shift over the same band again takes all four back off.
    _Check(gizmoMarquee.Resolve(added, caught, gizmoMarquee.MODE_TOGGLE)
           == [hips], "shift over an already-selected band must remove it")

    _Check(gizmoMarquee.Resolve(added, caught, gizmoMarquee.MODE_REMOVE)
           == [hips], "ctrl removes the band")


def TestSubSlopDrag():
    """A shaky click is not a marquee, so it changes nothing.

    gizmoUI only builds a band once PastSlop says so, and with no band
    the release falls through to usdview's own pick with the selection
    untouched -- which is the behaviour this guards.
    """
    _Check(gizmoMarquee.DRAG_SLOP == 3.0,
           "the slop is the 3.0 logical pixels touchPose uses")
    press = (100.0, 100.0)
    _Check(not gizmoMarquee.PastSlop(press, (102.0, 102.0)),
           "a 2-pixel wobble is a click")
    _Check(not gizmoMarquee.PastSlop(press, (103.0, 100.0)),
           "exactly the slop is still a click")
    _Check(gizmoMarquee.PastSlop(press, (104.0, 100.0)),
           "4 pixels across is a band")
    _Check(gizmoMarquee.PastSlop(press, (100.0, 96.0)),
           "4 pixels up is a band")
    # HiDPI: the threshold is in physical pixels, so it scales with the
    # device ratio and the gesture feels the same on the glass.
    _Check(not gizmoMarquee.PastSlop(press, (105.0, 100.0), 2.0),
           "5 physical pixels at ratio 2 is still a click")
    _Check(gizmoMarquee.PastSlop(press, (107.0, 100.0), 2.0),
           "7 physical pixels at ratio 2 is a band")

    # And a sub-slop gesture leaves the selection where it was: with no
    # band there is no wanted set, and no mode changes a selection by an
    # empty set except an explicit replace.
    current = ["/a", "/b"]
    for mode in (gizmoMarquee.MODE_TOGGLE, gizmoMarquee.MODE_REMOVE):
        _Check(gizmoMarquee.Resolve(current, [], mode) == current,
               "%s with nothing caught must not touch the selection" % mode)


def TestNoStage():
    """Everything answers empty rather than raising without a stage."""
    _Check(gizmoMarquee.ControlPrims(None) == [], "no stage: ControlPrims")
    _Check(gizmoMarquee.ScreenPositions(None, None, None, 0) == {},
           "no stage: ScreenPositions")
    _Check(gizmoMarquee.PathsInBand({}, 0, 0, 10, 10) == [],
           "no positions: PathsInBand")


def main():
    stage = _Stage()
    TestSelectable(stage)
    TestBand(stage)
    TestModes()
    TestModesOnTheBand(stage)
    TestSubSlopDrag()
    TestNoStage()
    print("test_gizmo_marquee: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
