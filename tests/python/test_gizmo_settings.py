#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoSettings.py: the Qt-free
per-tool settings model behind the gizmo's Tool Settings panel.

The Maya defaults are asserted literally rather than read back from the
module, because "what the tool starts as" is the behaviour a user
notices and the whole point of design spec section 8.2-8.4.

Usage: test_gizmo_settings.py [ignored]
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

import gizmoSettings as gset  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def TestOrientationConstants():
    """The four orientation tokens are distinct and stable strings."""
    tokens = (gset.ORIENT_WORLD, gset.ORIENT_OBJECT, gset.ORIENT_PARENT,
              gset.ORIENT_GIMBAL)
    _Check(tokens == ("world", "object", "parent", "gimbal"),
           "orientation tokens: %s" % (tokens,))
    _Check(len(set(tokens)) == 4, "orientation tokens are distinct")
    for token in tokens:
        _Check(gset.OrientationLabel(token)[0].isupper(),
               "%s has a display label" % token)


def TestMayaDefaults():
    """Design spec 8.2-8.4: what each tool starts as."""
    move = gset.MayaDefaults(gset.TOOL_TRANSLATE)
    _Check(move.orientation == gset.ORIENT_WORLD, "move starts in World")
    _Check(move.stepSnap is False, "move step snap starts off")
    _Check(move.stepSize == 1.0, "move step size is 1 unit")
    _Check(move.preserveChildren is False, "move preserve children is off")

    rotate = gset.MayaDefaults(gset.TOOL_ROTATE)
    _Check(rotate.orientation == gset.ORIENT_OBJECT, "rotate starts in Object")
    _Check(rotate.stepSnap is False, "rotate step snap starts off")
    _Check(rotate.stepSize == 15.0, "rotate step size is 15 degrees")
    _Check(rotate.freeRotate is True, "free rotate starts on")

    scale = gset.MayaDefaults(gset.TOOL_SCALE)
    _Check(scale.orientation == gset.ORIENT_WORLD, "scale starts in World")
    _Check(scale.stepSnap is False, "scale step snap starts off")
    _Check(scale.stepSize == 1.0, "scale step size is 1")
    _Check(scale.preventNegativeScale is False,
           "prevent negative scale starts off")

    # Every tool carries every field, so the panel and the controller can
    # read one without asking which tool it belongs to.
    for tool in (gset.TOOL_SELECT, gset.TOOL_TRANSLATE, gset.TOOL_ROTATE,
                 gset.TOOL_SCALE):
        settings = gset.MayaDefaults(tool)
        for field in ("orientation", "stepSnap", "stepSize", "freeRotate",
                      "preventNegativeScale", "preserveChildren",
                      "snapMode"):
            _Check(hasattr(settings, field),
                   "%s settings carry %s" % (tool, field))


def TestOrientationChoices():
    """Maya offers Gimbal only for rotate, and Parent only for the rest."""
    for tool in (gset.TOOL_TRANSLATE, gset.TOOL_SCALE):
        choices = gset.OrientationChoices(tool)
        _Check(choices == (gset.ORIENT_WORLD, gset.ORIENT_OBJECT,
                           gset.ORIENT_PARENT),
               "%s orientations: %s" % (tool, (choices,)))
    choices = gset.OrientationChoices(gset.TOOL_ROTATE)
    _Check(choices == (gset.ORIENT_OBJECT, gset.ORIENT_WORLD,
                       gset.ORIENT_GIMBAL),
           "rotate orientations: %s" % (choices,))
    # Every default is one of its own tool's choices; a combo box built
    # from the choices must be able to show the value it starts on.
    for tool in (gset.TOOL_TRANSLATE, gset.TOOL_ROTATE, gset.TOOL_SCALE):
        _Check(gset.MayaDefaults(tool).orientation
               in gset.OrientationChoices(tool),
               "%s default orientation is offered" % tool)


def TestSnapChoices():
    """Snap To offers all five for Move, off+grid for Rotate, none else."""
    _Check(gset.SnapChoices(gset.TOOL_TRANSLATE) == (
        gset.SNAP_OFF, gset.SNAP_GRID, gset.SNAP_POINT, gset.SNAP_EDGE,
        gset.SNAP_SURFACE), "move snap choices: %s"
        % (gset.SnapChoices(gset.TOOL_TRANSLATE),))
    _Check(gset.SnapChoices(gset.TOOL_ROTATE) == (
        gset.SNAP_OFF, gset.SNAP_GRID), "rotate snap choices: %s"
        % (gset.SnapChoices(gset.TOOL_ROTATE),))
    _Check(gset.SnapChoices(gset.TOOL_SCALE) == ()
           and gset.SnapChoices(gset.TOOL_SELECT) == (),
           "scale and select offer no snap modes")
    for token in (gset.SNAP_OFF, gset.SNAP_GRID, gset.SNAP_POINT,
                  gset.SNAP_EDGE, gset.SNAP_SURFACE):
        _Check(gset.SnapLabel(token)[0].isupper(),
               "%s has a display label" % token)
    # Every default is off, and the tools that offer any choices can show
    # the value they start on. Scale and select offer none, so they are
    # asserted separately and never fed to the loop -- the same reason
    # the orientation loop excludes TOOL_SELECT.
    for tool in (gset.TOOL_SELECT, gset.TOOL_TRANSLATE, gset.TOOL_ROTATE,
                 gset.TOOL_SCALE):
        _Check(gset.MayaDefaults(tool).snapMode == gset.SNAP_OFF,
               "%s snap starts off" % tool)
    for tool in (gset.TOOL_TRANSLATE, gset.TOOL_ROTATE):
        _Check(gset.MayaDefaults(tool).snapMode
               in gset.SnapChoices(tool),
               "%s default snap is offered" % tool)


def TestGridSize():
    """Grid Size is session-wide, clamped, notified, and never reset."""
    settings = gset.GizmoSettings()
    _Check(settings.gridSize == 1.0, "the grid starts at 1.0")
    settings.gridSize = 2.0
    _Check(settings.gridSize == 2.0, "the grid took the new value")
    settings.gridSize = 0.0
    _Check(settings.gridSize == gset.GRID_SIZE_MIN,
           "the grid is clamped up: %s" % settings.gridSize)
    settings.gridSize = 1e-4
    _Check(settings.gridSize == 1e-4, "the floor itself sticks")
    seen = []
    settings.AddListener(lambda: seen.append(len(seen)))
    settings.gridSize = 4.0
    _Check(len(seen) == 1, "a grid size change notifies")
    settings.gridSize = 4.0
    _Check(len(seen) == 1, "an unchanged grid write is silent")
    settings.For(gset.TOOL_TRANSLATE).snapMode = gset.SNAP_GRID
    settings.Reset(gset.TOOL_TRANSLATE)
    _Check(settings.gridSize == 4.0,
           "Reset(tool) leaves the world grid alone")
    _Check(settings.For(gset.TOOL_TRANSLATE).snapMode == gset.SNAP_OFF,
           "but the sticky mode does reset")


def TestPerToolIndependence():
    """Each tool keeps its own settings; editing one leaves the rest."""
    settings = gset.GizmoSettings()
    settings.For(gset.TOOL_TRANSLATE).stepSize = 4.0
    _Check(settings.For(gset.TOOL_TRANSLATE).stepSize == 4.0,
           "the move step size took the new value")
    _Check(settings.For(gset.TOOL_ROTATE).stepSize == 15.0,
           "the rotate step size is untouched")
    _Check(settings.For(gset.TOOL_TRANSLATE) is
           settings.For(gset.TOOL_TRANSLATE),
           "For() hands back the same object, not a copy")


def TestManipulatorSize():
    settings = gset.GizmoSettings()
    _Check(settings.manipulatorSize == 90.0,
           "the manipulator starts at 90 logical pixels")
    settings.manipulatorSize = 120.0
    _Check(settings.manipulatorSize == 120.0, "the size took the new value")
    # A zero or negative size would make every handle unhittable and a
    # huge one would fill the viewport, so the model clamps rather than
    # trusting whatever a spin box or a '-' keypress hands it.
    settings.manipulatorSize = 0.0
    _Check(settings.manipulatorSize == gset.MANIPULATOR_SIZE_MIN,
           "the size is clamped up: %s" % settings.manipulatorSize)
    settings.manipulatorSize = 1e6
    _Check(settings.manipulatorSize == gset.MANIPULATOR_SIZE_MAX,
           "the size is clamped down: %s" % settings.manipulatorSize)


def TestReset():
    settings = gset.GizmoSettings()
    rotate = settings.For(gset.TOOL_ROTATE)
    rotate.orientation = gset.ORIENT_GIMBAL
    rotate.stepSnap = True
    rotate.stepSize = 90.0
    rotate.freeRotate = False
    settings.For(gset.TOOL_TRANSLATE).stepSnap = True

    settings.Reset(gset.TOOL_ROTATE)
    _Check(settings.For(gset.TOOL_ROTATE) is rotate,
           "Reset keeps the same object so a bound widget stays wired")
    _Check(rotate.orientation == gset.ORIENT_OBJECT, "orientation reset")
    _Check(rotate.stepSnap is False, "step snap reset")
    _Check(rotate.stepSize == 15.0, "step size reset")
    _Check(rotate.freeRotate is True, "free rotate reset")
    _Check(settings.For(gset.TOOL_TRANSLATE).stepSnap is True,
           "Reset(rotate) leaves the move tool alone")


def TestListeners():
    settings = gset.GizmoSettings()
    seen = []
    settings.AddListener(lambda: seen.append(len(seen)))

    settings.For(gset.TOOL_TRANSLATE).stepSize = 2.0
    _Check(len(seen) == 1, "a per-tool field change notifies: %d" % len(seen))
    settings.manipulatorSize = 100.0
    _Check(len(seen) == 2, "a manipulator size change notifies")
    settings.Reset(gset.TOOL_TRANSLATE)
    _Check(len(seen) == 3, "Reset notifies")

    # Writing the value it already holds must not notify: the controller
    # rebuilds every handle on a settings change, and a combo box that
    # re-emits its current index on every rebuild would otherwise loop.
    settings.manipulatorSize = 100.0
    _Check(len(seen) == 3, "an unchanged write is silent: %d" % len(seen))
    settings.For(gset.TOOL_TRANSLATE).stepSize = 1.0
    _Check(len(seen) == 3, "an unchanged per-tool write is silent")


def TestUnknownToolIsSafe():
    """
    The Select tool has no manipulator, and a future tool must not make
    the panel raise; both resolve to a plain default set.
    """
    settings = gset.GizmoSettings()
    _Check(settings.For(gset.TOOL_SELECT) is not None,
           "the select tool has a settings object")
    _Check(gset.OrientationChoices(gset.TOOL_SELECT) == (),
           "the select tool offers no orientations")


def main():
    groups = [
        ("orientation constants", TestOrientationConstants),
        ("maya defaults", TestMayaDefaults),
        ("orientation choices", TestOrientationChoices),
        ("snap choices", TestSnapChoices),
        ("grid size", TestGridSize),
        ("per-tool independence", TestPerToolIndependence),
        ("manipulator size", TestManipulatorSize),
        ("reset", TestReset),
        ("listeners", TestListeners),
        ("unknown tool", TestUnknownToolIsSafe),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_SETTINGS_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
