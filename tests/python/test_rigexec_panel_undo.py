#!/usr/bin/env python
"""
Headless test: the authoring PANELS record what they do.

tests/python/test_rigexec_undo.py proves the undo primitives work. This
one proves they are actually wired up, which is the half that was
missing: the panels below authored to the stage with nothing reaching
the undo stack, and the picker LOOKED wired -- it opened an undo scope
on every zero, over an empty list of paths, so it recorded nothing and
pushed nothing while clearing the whole rig's pose.

The Qt panels are built through __new__ rather than their constructors,
the way testUsdviewRigExec.py builds a container for one branch: the
methods under test are decisions about the stage, and standing up a
QApplication to reach them would test Qt instead. The picker's wiring is
covered end to end by tests/testUsdviewPicker.py, which drives the real
button.

Usage: test_rigexec_panel_undo.py [<generated schema resources dir>]
"""
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Plug, Sdf, Usd  # noqa: E402

import curvenetUI  # noqa: E402
import rigExecUndo  # noqa: E402
import volumeWeightUI  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered; pass the generated "
           "resources dir (build/usd/rigExecSchema/resources)")


def _Stage():
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Rig", "RigExecRoot")
    return stage


# ---------------------------------------------------------------------
# The volume weight panel. Every write in that file goes through
# SetAtTime or SetVisibleAtTime, so that is where the recording is and
# that is what these check.
# ---------------------------------------------------------------------

def TestVolumeWeightSetAtTimeRecords():
    stage = _Stage()
    prim = stage.DefinePrim("/Rig/Weights/Sphere", "RigExecSphereWeight")
    attr = prim.GetAttribute("inputs:falloffMax")
    attr.Set(1.0)

    undo = rigExecUndo.UndoStack()
    volumeWeightUI.SetUndoStack(undo)
    try:
        volumeWeightUI.SetVisibleAtTime(attr, 7.0, Usd.TimeCode.Default())
        _Check(undo.CanUndo(), "a volume weight write pushed an entry")
        _Check(attr.Get() == 7.0, "and it took effect")
        undo.Undo()
        _Check(attr.Get() == 1.0,
               "undo put the old falloff back, got %s" % attr.Get())
        undo.Redo()
        _Check(attr.Get() == 7.0, "redo reapplied it")
    finally:
        volumeWeightUI.SetUndoStack(None)


def TestVolumeWeightWritesFoldIntoOneEntry():
    """A create authors a prim, a relationship and half a dozen avars."""
    stage = _Stage()
    prim = stage.DefinePrim("/Rig/Weights/Sphere", "RigExecSphereWeight")
    undo = rigExecUndo.UndoStack()
    volumeWeightUI.SetUndoStack(undo)
    try:
        with volumeWeightUI.Recording(stage, [prim.GetPath()], "Many"):
            for name, value in (("inputs:falloffMax", 3.0),
                                ("avars:tx", 1.0), ("avars:ty", 2.0)):
                volumeWeightUI.SetVisibleAtTime(
                    prim.GetAttribute(name), value, Usd.TimeCode.Default())
        _Check(undo.CanUndo(), "the outer scope pushed an entry")
        undo.Undo()
        _Check(not undo.CanUndo(),
               "the three writes were ONE entry, not three")
        _Check(not prim.GetAttribute("avars:tx").HasAuthoredValue()
               and not prim.GetAttribute("inputs:falloffMax").HasAuthoredValue(),
               "and one undo took all of them away")
    finally:
        volumeWeightUI.SetUndoStack(None)


def TestVolumeWeightParentPathCreatesNothing():
    """The scope has to name the Weights scope before it exists."""
    stage = _Stage()
    path, existed = volumeWeightUI.ChooseWeightParentPath(stage, [])
    _Check(str(path) == "/Rig/Weights",
           "the weights scope goes under the rig, got %s" % path)
    _Check(not existed, "and it is not there yet")
    _Check(not stage.GetPrimAtPath(path),
           "asking where it goes must not create it")

    made = volumeWeightUI.ChooseWeightParentPrim(stage, [])
    _Check(made and made.GetPath() == path,
           "and creating it puts it exactly there")
    again, existed = volumeWeightUI.ChooseWeightParentPath(stage, [])
    _Check(again == path and existed, "the second answer says it exists")


def TestVolumeWeightCreateIsOneUndo():
    stage = _Stage()
    undo = rigExecUndo.UndoStack()
    volumeWeightUI.SetUndoStack(undo)
    try:
        parentPath, parentExisted = volumeWeightUI.ChooseWeightParentPath(
            stage, [])
        newPath = parentPath.AppendChild("SphereWeight")
        paths = [newPath] + ([] if parentExisted else [parentPath])
        with volumeWeightUI.Recording(stage, paths, "Create"):
            volumeWeightUI.CreateVolumeWeightPrim(
                stage, "RigExecSphereWeight", [], Usd.TimeCode.Default())
        _Check(bool(stage.GetPrimAtPath(newPath)),
               "the weight was created at %s" % newPath)
        undo.Undo()
        _Check(not stage.GetPrimAtPath(newPath),
               "undo removed the new weight")
        _Check(not stage.GetPrimAtPath(parentPath),
               "and the Weights scope it brought with it")
        undo.Redo()
        _Check(bool(stage.GetPrimAtPath(newPath)),
               "redo brought the weight back")
    finally:
        volumeWeightUI.SetUndoStack(None)


# ---------------------------------------------------------------------
# The curvenet panel.
# ---------------------------------------------------------------------

class _Api(object):
    def __init__(self, stage):
        self.stage = stage
        self.selectedPrims = []


def _Panel(stage, undo):
    """A CurvenetPanel with no widgets: see the module docstring."""
    panel = curvenetUI.CurvenetPanel.__new__(curvenetUI.CurvenetPanel)
    panel._api = _Api(stage)
    panel._undo = undo
    panel._gesture = None
    panel._curvenetPath = None
    panel._chainKnot = -1
    panel._SetStatus = lambda *args, **kwargs: None
    panel._Refresh = lambda *args, **kwargs: None
    panel._UpdateDisplay = lambda *args, **kwargs: None
    return panel


def TestCurvenetCreateIsUndoable():
    stage = _Stage()
    undo = rigExecUndo.UndoStack()
    panel = _Panel(stage, undo)
    panel._OnNewCurvenet()

    path = panel._curvenetPath
    _Check(path is not None and bool(stage.GetPrimAtPath(path)),
           "the panel created a curvenet")
    _Check(undo.CanUndo(), "and pushed an entry for it")
    undo.Undo()
    _Check(not stage.GetPrimAtPath(path),
           "undo removed the curvenet prim, not just its points")
    undo.Redo()
    _Check(bool(stage.GetPrimAtPath(path)), "redo brought it back")


def TestCurvenetGestureIsOneEntry():
    """A drag is one Ctrl+Z, however many points it wrote."""
    stage = _Stage()
    undo = rigExecUndo.UndoStack()
    panel = _Panel(stage, undo)
    panel._OnNewCurvenet()
    prim = stage.GetPrimAtPath(panel._curvenetPath)
    curvenetUI.SetPoints(prim, [(0, 0, 0), (1, 0, 0)])
    undo.Clear()

    panel._BeginGesture("Edit curvenet")
    for x in range(1, 6):
        curvenetUI.SetPoints(prim, [(0, 0, 0), (float(x), 0, 0)])
    panel._EndGesture()

    _Check(undo.CanUndo(), "the gesture pushed an entry")
    undo.Undo()
    _Check(not undo.CanUndo(),
           "five writes inside one gesture are ONE entry")
    points = [tuple(p) for p in curvenetUI.GetPoints(prim)]
    _Check(points == [(0, 0, 0), (1, 0, 0)],
           "undo restored the points the gesture started from, got %s"
           % (points,))


def TestCurvenetGestureReopeningCommitsTheLastOne():
    """A press whose release never arrives must not swallow the edit."""
    stage = _Stage()
    undo = rigExecUndo.UndoStack()
    panel = _Panel(stage, undo)
    panel._OnNewCurvenet()
    prim = stage.GetPrimAtPath(panel._curvenetPath)
    undo.Clear()

    panel._BeginGesture("First")
    curvenetUI.SetPoints(prim, [(1, 0, 0)])
    panel._BeginGesture("Second")          # no _EndGesture in between
    _Check(undo.CanUndo() and undo.UndoText() == "First",
           "the abandoned gesture was committed, not dropped")
    curvenetUI.SetPoints(prim, [(1, 0, 0), (2, 0, 0)])
    panel._EndGesture()
    _Check(undo.UndoText() == "Second", "and the second one closed normally")


def TestCurvenetBindPathsNamesOnlyWhatAppears():
    """Binding must not snapshot a Movers scope full of other movers."""
    stage = _Stage()
    rig = stage.GetPrimAtPath("/Rig")
    paths = curvenetUI.CurvenetPanel._BindPaths(stage, rig)
    _Check(Sdf.Path("/Rig/Movers") in paths
           and Sdf.Path("/Rig/Movers/Geometry") in paths,
           "both absent scopes are recorded, got %s" % (paths,))

    stage.DefinePrim("/Rig/Movers", "Scope")
    stage.DefinePrim("/Rig/Movers/Geometry", "Scope")
    paths = curvenetUI.CurvenetPanel._BindPaths(stage, rig)
    _Check(Sdf.Path("/Rig/Movers") not in paths
           and Sdf.Path("/Rig/Movers/Geometry") not in paths,
           "scopes that already exist are left out: %s" % (paths,))
    _Check(paths == [Sdf.Path("/Rig/Movers/Geometry/ProfileMover")],
           "only the mover itself is recorded, got %s" % (paths,))


def TestPanelsAreQuietWithoutAStack():
    """Every panel still authors when nothing is recording."""
    stage = _Stage()
    panel = _Panel(stage, None)
    panel._OnNewCurvenet()
    _Check(bool(stage.GetPrimAtPath(panel._curvenetPath)),
           "the curvenet panel works with no undo stack")

    volumeWeightUI.SetUndoStack(None)
    prim = stage.DefinePrim("/Rig/Weights/Sphere", "RigExecSphereWeight")
    volumeWeightUI.SetVisibleAtTime(prim.GetAttribute("inputs:falloffMax"),
                                    2.0, Usd.TimeCode.Default())
    _Check(prim.GetAttribute("inputs:falloffMax").Get() == 2.0,
           "the volume weight panel works with no undo stack")


def main():
    _RegisterSchema()
    groups = [
        ("volume weight: a write records",
         TestVolumeWeightSetAtTimeRecords),
        ("volume weight: many writes fold into one entry",
         TestVolumeWeightWritesFoldIntoOneEntry),
        ("volume weight: the parent path creates nothing",
         TestVolumeWeightParentPathCreatesNothing),
        ("volume weight: create is one undo",
         TestVolumeWeightCreateIsOneUndo),
        ("curvenet: create is undoable", TestCurvenetCreateIsUndoable),
        ("curvenet: a gesture is one entry", TestCurvenetGestureIsOneEntry),
        ("curvenet: an abandoned gesture commits",
         TestCurvenetGestureReopeningCommitsTheLastOne),
        ("curvenet: bind records only what appears",
         TestCurvenetBindPathsNamesOnlyWhatAppears),
        ("panels work without a stack", TestPanelsAreQuietWithoutAStack),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_PANEL_UNDO_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
