#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/rigExecUndo.py.

Runs without Qt and without a usdview: the undo stack snapshots Sdf
attribute specs, so everything it does is observable on an in-memory
stage. Usage: test_rigexec_undo.py [<generated schema resources dir>]
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Plug, Sdf, Tf, Ts, Usd  # noqa: E402

import rigExecUndo  # noqa: E402


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
    stage.DefinePrim("/Rig/Ctl", "RigExecControl")
    return stage


def _Knot(time, value):
    knot = Ts.Knot(typeName="double")
    knot.SetTime(time)
    knot.SetValue(value)
    knot.SetNextInterpolation(Ts.InterpCurve)
    return knot


def TestSnapshotAbsentSpec():
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Ctl.avars:tx")
    before = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    _Check(not before.exists, "unauthored attribute must snapshot as absent")
    stage.GetAttributeAtPath(path).Set(3.0)
    _Check(layer.GetAttributeAtPath(path) is not None, "spec authored")
    before.Restore()
    _Check(layer.GetAttributeAtPath(path) is None,
           "restoring an absent snapshot must remove the spec")


def TestSnapshotDefaultAndTimeSamples():
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Ctl.avars:ty")
    attr = stage.GetAttributeAtPath(path)
    attr.Set(1.0)
    attr.Set(5.0, 10.0)
    attr.Set(7.0, 20.0)
    snap = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    _Check(snap.exists and snap.hasDefault and snap.default == 1.0,
           "default captured")
    _Check(snap.timeSamples == {10.0: 5.0, 20.0: 7.0}, "samples captured")
    attr.Set(2.0)
    attr.Set(9.0, 10.0)
    attr.Set(9.0, 30.0)
    snap.Restore()
    _Check(attr.Get() == 1.0, "default restored")
    _Check(layer.ListTimeSamplesForPath(path) == [10.0, 20.0],
           "extra sample removed: %s" % layer.ListTimeSamplesForPath(path))
    _Check(attr.Get(10.0) == 5.0, "sample value restored")
    _Check(snap == rigExecUndo.AttributeSnapshot.Capture(layer, path),
           "round trip is content-equal")


def TestSnapshotSpline():
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Ctl.avars:rz")
    attr = stage.GetAttributeAtPath(path)
    spline = Ts.Spline("double")
    spline.SetKnot(_Knot(1.0, 10.0))
    attr.SetSpline(spline)
    snap = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    _Check(snap.spline is not None and snap.spline == spline,
           "spline captured by value")
    spline.SetKnot(_Knot(2.0, 20.0))
    attr.SetSpline(spline)
    _Check(len(attr.GetSpline().GetKnots()) == 2, "second knot authored")
    snap.Restore()
    _Check(len(attr.GetSpline().GetKnots()) == 1,
           "spline restored to the captured single knot")
    _Check(abs(attr.Get(1.0) - 10.0) < 1e-9, "knot value restored")
    # A snapshot with no spline must clear one that appeared later.
    snapNoSpline = rigExecUndo.AttributeSnapshot.Capture(
        layer, Sdf.Path("/Rig/Ctl.avars:rx"))
    stage.GetAttributeAtPath("/Rig/Ctl.avars:rx").SetSpline(spline)
    snapNoSpline.Restore()
    _Check(layer.GetAttributeAtPath("/Rig/Ctl.avars:rx") is None,
           "absent spec restored to absent even after a spline write")


def TestUndoStack():
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Ctl.avars:tx")
    attr = stage.GetAttributeAtPath(path)
    stack = rigExecUndo.UndoStack()
    events = []
    stack.AddListener(lambda: events.append(1))
    _Check(not stack.CanUndo() and not stack.CanRedo(), "empty stack")
    before = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    attr.Set(4.0)
    after = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    stack.Push(rigExecUndo.Edit(
        "Translate", [rigExecUndo.EditEntry(layer, path, before, after)]))
    _Check(stack.CanUndo() and stack.UndoText() == "Translate", "pushed")
    _Check(len(events) == 1, "listener notified on push")
    _Check(stack.Undo(), "undo returns True")
    _Check(layer.GetAttributeAtPath(path) is None, "undo removed the spec")
    _Check(stack.CanRedo() and stack.RedoText() == "Translate", "redo")
    _Check(stack.Redo(), "redo returns True")
    _Check(attr.Get() == 4.0, "redo restored the value")
    stack.Undo()
    attr.Set(8.0)
    stack.Push(rigExecUndo.Edit("Other", [rigExecUndo.EditEntry(
        layer, path, before,
        rigExecUndo.AttributeSnapshot.Capture(layer, path))]))
    _Check(not stack.CanRedo(), "a push clears the redo branch")
    _Check(not stack.Redo(), "redo on an empty branch returns False")
    for i in range(rigExecUndo.UndoStack.LIMIT + 5):
        stack.Push(rigExecUndo.Edit("N%d" % i, []))
    count = 0
    while stack.Undo():
        count += 1
    _Check(count == rigExecUndo.UndoStack.LIMIT,
           "stack is bounded to LIMIT entries, undid %d" % count)
    stack.Clear()
    _Check(not stack.CanUndo() and not stack.CanRedo(), "cleared")


def TestEditRecorder():
    stage = _Stage()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    session = stage.GetSessionLayer()
    tx = Sdf.Path("/Rig/Ctl.avars:tx")
    ty = Sdf.Path("/Rig/Ctl.avars:ty")
    recorder = rigExecUndo.EditRecorder(stage, [tx, ty])
    recorder.Begin()
    _Check(recorder.Commit("nothing") is None,
           "no change commits to None")
    recorder.Begin()
    stage.GetAttributeAtPath(tx).Set(2.5)
    edit = recorder.Commit("Translate Ctl")
    _Check(edit is not None and edit.label == "Translate Ctl", "edit made")
    _Check(len(edit.entries) == 1 and edit.entries[0].layer == session,
           "only the changed attribute is recorded, in the session layer")
    _Check(session.GetAttributeAtPath(tx) is not None
           and stage.GetRootLayer().GetAttributeAtPath(tx) is None,
           "write landed in the edit target layer only")
    recorder.Begin()
    stage.GetAttributeAtPath(ty).Set(9.0)
    recorder.Abort()
    _Check(session.GetAttributeAtPath(ty) is None,
           "abort restores the pre-drag state")


class _NoticeCounter(object):
    """Counts Usd.Notice.ObjectsChanged rounds on one stage."""

    def __init__(self, stage):
        self.count = 0
        # The key must outlive the listener: dropping it revokes it.
        self._key = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._OnChanged, stage)

    def _OnChanged(self, notice, sender):
        self.count += 1

    def Revoke(self):
        self._key.Revoke()


def TestSingleNoticePerEdit():
    """
    A multi-attribute edit must reach the stage as ONE change round.

    An xform edit writes the op attributes and xformOpOrder separately.
    Restoring them in separate change blocks leaves the stage briefly
    holding an xformOpOrder that names ops which do not exist yet, and
    anything recomposing on every notice (the gizmo controller does,
    outside a drag) reads that inconsistent state and warns.
    """
    stage = _Stage()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    session = stage.GetSessionLayer()
    tx = Sdf.Path("/Rig/Ctl.avars:tx")
    ty = Sdf.Path("/Rig/Ctl.avars:ty")

    beforeTx = rigExecUndo.AttributeSnapshot.Capture(session, tx)
    beforeTy = rigExecUndo.AttributeSnapshot.Capture(session, ty)
    stage.GetAttributeAtPath(tx).Set(1.0)
    stage.GetAttributeAtPath(ty).Set(2.0)
    edit = rigExecUndo.Edit("Translate", [
        rigExecUndo.EditEntry(
            session, tx, beforeTx,
            rigExecUndo.AttributeSnapshot.Capture(session, tx)),
        rigExecUndo.EditEntry(
            session, ty, beforeTy,
            rigExecUndo.AttributeSnapshot.Capture(session, ty))])

    counter = _NoticeCounter(stage)
    edit.Undo()
    _Check(counter.count == 1,
           "a two-entry undo must fire exactly one ObjectsChanged, got %d"
           % counter.count)
    _Check(session.GetAttributeAtPath(tx) is None
           and session.GetAttributeAtPath(ty) is None, "undo took effect")

    counter.count = 0
    edit.Redo()
    _Check(counter.count == 1,
           "a two-entry redo must fire exactly one ObjectsChanged, got %d"
           % counter.count)
    counter.Revoke()

    # The same rule for an aborted drag: one notice, not one per path.
    recorder = rigExecUndo.EditRecorder(stage, [tx, ty])
    recorder.Begin()
    stage.GetAttributeAtPath(tx).Set(11.0)
    stage.GetAttributeAtPath(ty).Set(12.0)
    counter = _NoticeCounter(stage)
    recorder.Abort()
    _Check(counter.count == 1,
           "a two-path abort must fire exactly one ObjectsChanged, got %d"
           % counter.count)
    # Begin() ran after the redo above, so the pre-drag state it restores
    # is the redone value, not an absent spec.
    _Check(stage.GetAttributeAtPath(tx).Get() == 1.0
           and stage.GetAttributeAtPath(ty).Get() == 2.0,
           "abort took effect")
    counter.Revoke()


# ---------------------------------------------------------------------
# SpecSnapshot: the structural half, and the reason it exists.
# ---------------------------------------------------------------------

def TestSpecSnapshotUndoesPrimCreation():
    """The gap that let a panel build rig with an empty undo stack.

    AttributeSnapshot restores the ATTRIBUTE and leaves the prim, so a
    "New curvenet" or "Create sphere weight" undone through it removed
    the values and kept the prim -- a half-built thing on the stage with
    nothing left on the stack to finish taking it away.
    """
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Made")

    attrRec = rigExecUndo.EditRecorder(stage, ["/Rig/Made.avars:tx"])
    attrRec.Begin()
    stage.DefinePrim(path, "RigExecControl").GetAttribute("avars:tx").Set(4.0)
    edit = attrRec.Commit("attribute only")
    edit.Undo()
    _Check(bool(stage.GetPrimAtPath(path)),
           "precondition: an attribute recorder leaves the prim behind")

    stage.RemovePrim(path)
    rec = rigExecUndo.SpecRecorder(stage, [path])
    rec.Begin()
    stage.DefinePrim(path, "RigExecControl").GetAttribute("avars:tx").Set(4.0)
    edit = rec.Commit("Create")
    _Check(edit is not None, "creating a prim is a recordable change")
    edit.Undo()
    _Check(not stage.GetPrimAtPath(path),
           "undo must remove a prim that did not exist before")
    edit.Redo()
    _Check(bool(stage.GetPrimAtPath(path))
           and stage.GetAttributeAtPath("/Rig/Made.avars:tx").Get() == 4.0,
           "redo must bring the prim back with its values")


def TestSpecSnapshotRoundTripsRelationships():
    """A relationship is invisible to the attribute recorder entirely."""
    stage = _Stage()
    prim = stage.GetPrimAtPath("/Rig/Ctl")
    prim.CreateRelationship("rigExec:moves").SetTargets(["/Rig"])
    path = Sdf.Path("/Rig/Ctl.rigExec:moves")

    attrRec = rigExecUndo.EditRecorder(stage, [path])
    attrRec.Begin()
    prim.GetRelationship("rigExec:moves").SetTargets([])
    _Check(attrRec.Commit("attribute only") is None,
           "precondition: the attribute recorder sees no relationship")

    prim.GetRelationship("rigExec:moves").SetTargets(["/Rig"])
    rec = rigExecUndo.SpecRecorder(stage, [path])
    rec.Begin()
    prim.GetRelationship("rigExec:moves").SetTargets(["/Rig/Ctl"])
    edit = rec.Commit("Retarget")
    _Check(edit is not None, "retargeting is a recordable change")
    edit.Undo()
    _Check([str(t) for t in
            prim.GetRelationship("rigExec:moves").GetTargets()] == ["/Rig"],
           "undo must put the original targets back")
    edit.Redo()
    _Check([str(t) for t in
            prim.GetRelationship("rigExec:moves").GetTargets()]
           == ["/Rig/Ctl"], "redo must reapply the new targets")


def TestSpecSnapshotKeepsSiblingOrder():
    """Mover evaluation order IS sibling order, so undo must preserve it."""
    stage = _Stage()
    for name in ("A", "B", "C"):
        stage.DefinePrim("/Rig/%s" % name, "RigExecControl")
    rec = rigExecUndo.SpecRecorder(stage, ["/Rig/B"])
    rec.Begin()
    stage.GetAttributeAtPath("/Rig/B.avars:tx")
    stage.GetPrimAtPath("/Rig/B").GetAttribute("avars:tx").Set(1.0)
    edit = rec.Commit("Pose B")
    edit.Undo()
    names = [p.GetName() for p in stage.GetPrimAtPath("/Rig").GetChildren()]
    _Check(names == ["Ctl", "A", "B", "C"],
           "undo reordered the siblings: %s" % names)


def TestSpecScopeIsQuietWhenNothingChanged():
    stage = _Stage()
    undo = rigExecUndo.UndoStack()
    with rigExecUndo.SpecScope(stage, ["/Rig/Ctl"], undo, "No-op"):
        pass
    _Check(not undo.CanUndo(), "a scope that changed nothing pushes nothing")

    with rigExecUndo.SpecScope(stage, ["/Rig/Ctl"], undo, "Pose"):
        stage.GetPrimAtPath("/Rig/Ctl").GetAttribute("avars:tx").Set(2.0)
    _Check(undo.CanUndo() and undo.UndoText() == "Pose",
           "a scope that changed something pushes one labelled entry")
    undo.Undo()
    # HasAuthoredValue, not Get(): avars:tx has a schema fallback of 0,
    # so an unauthored avar reads 0.0 rather than None and a Get() check
    # would pass whether the spec came away or not.
    _Check(not stage.GetAttributeAtPath(
               "/Rig/Ctl.avars:tx").HasAuthoredValue(),
           "undo through the scope clears the authored value")
    undo.Redo()
    _Check(stage.GetAttributeAtPath("/Rig/Ctl.avars:tx").Get() == 2.0,
           "redo through the scope puts it back")


def TestSpecScopeAbortsOnException():
    stage = _Stage()
    undo = rigExecUndo.UndoStack()
    try:
        with rigExecUndo.SpecScope(stage, ["/Rig/Ctl"], undo, "Boom"):
            stage.GetPrimAtPath("/Rig/Ctl").GetAttribute("avars:tx").Set(9.0)
            raise RuntimeError("half way through")
    except RuntimeError:
        pass
    _Check(not stage.GetAttributeAtPath(
               "/Rig/Ctl.avars:tx").HasAuthoredValue(),
           "an exception inside the scope rolls the edit back")
    _Check(not undo.CanUndo(), "a rolled-back edit is not on the stack")


def main():
    _RegisterSchema()
    groups = [
        ("absent spec", TestSnapshotAbsentSpec),
        ("default + time samples", TestSnapshotDefaultAndTimeSamples),
        ("spline", TestSnapshotSpline),
        ("undo stack", TestUndoStack),
        ("edit recorder", TestEditRecorder),
        ("one notice per edit", TestSingleNoticePerEdit),
        ("spec snapshot undoes prim creation",
         TestSpecSnapshotUndoesPrimCreation),
        ("spec snapshot round-trips relationships",
         TestSpecSnapshotRoundTripsRelationships),
        ("spec snapshot keeps sibling order",
         TestSpecSnapshotKeepsSiblingOrder),
        ("spec scope is quiet when nothing changed",
         TestSpecScopeIsQuietWhenNothingChanged),
        ("spec scope aborts on exception", TestSpecScopeAbortsOnException),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_UNDO_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
