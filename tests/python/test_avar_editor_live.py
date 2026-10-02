#!/usr/bin/env python
"""
Headless test for the Avar Editor following a manipulator drag LIVE.

A viewport gizmo drag authors nothing until release: every mouse sample
lands in gizmoMath.Writer and goes to Hydra through gizmoPreview.Push,
and gizmoUI._EndDrag commits the lot once. The editor has to show the
number being dragged while it is dragged, from that same pending value,
without anyone starting to author per sample. This drives the real
panel widgets (Qt offscreen, no usdview) with the real Writer and the
real preview channel, and checks three things per drag:

  mid-drag   the field shows the pending value, the stage does not
  release    CommitToStage then End: the field shows the authored value
  abort      End then Clear: the field is back on the stage's value

and that a sample only touches the fields whose number changed.

Usage: test_avar_editor_live.py
"""
import os

# Before the first Qt import: the widgets are real, the screen is not.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402
from pxr.Usdviewq.qt import QtWidgets  # noqa: E402

import avarEditorUI  # noqa: E402
import gizmoMath  # noqa: E402
import gizmoPreview  # noqa: E402
import rigExecUndo  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


class _Signal(object):
    """A usdview dataModel signal the panel connects to and never fires."""

    def connect(self, slot):
        pass


class _Selection(object):
    signalPrimSelectionChanged = _Signal()


class _DataModel(object):
    def __init__(self, stage):
        self.stage = stage
        self.selection = _Selection()
        self.currentFrameChanged = _Signal()
        self.signalStageReplaced = _Signal()


class _Api(object):
    """Just enough usdviewApi for the panel: a stage and a selection."""

    def __init__(self, stage, prim):
        self.dataModel = _DataModel(stage)
        self.selectedPrims = [prim]
        self.prim = prim
        self.frame = Usd.TimeCode.Default()
        self.qMainWindow = None


def _Stage():
    """
    A control with custom avars, so no schema is needed: DiscoverChannels
    finds channels by prefix. The session layer is the edit target, as in
    usdview, so "authored" below means "a spec in the session layer".
    """
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/Rig/Ctrl", "Xform")
    for name in ("tx", "ty", "rz"):
        attr = prim.CreateAttribute("avars:" + name, Sdf.ValueTypeNames.Double,
                                    custom=True)
        attr.Set(0.0)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    return stage, prim


def _SessionSpec(stage, attr):
    return stage.GetSessionLayer().GetAttributeAtPath(attr.GetPath())


def _CountTextSets(field):
    """Count the setText calls that reach the widget, for the per-sample
    cost check. SetText already skips an unchanged string, so this counts
    real changes."""
    calls = []
    original = field.setText

    def setText(text):
        calls.append(text)
        original(text)

    field.setText = setText
    return calls


def TestDragShowsPendingAndAuthorsOnRelease():
    stage, prim = _Stage()
    panel = avarEditorUI.AvarEditorPanel(_Api(stage, prim),
                                         rigExecUndo.UndoStack())
    try:
        tx = panel.Row("tx")
        ty = panel.Row("ty")
        _Check(tx is not None and ty is not None,
               "the panel built rows for the custom avars: %s"
               % [r.name for r in panel.Rows()])
        _Check(tx.field.text() == "0" and ty.field.text() == "0",
               "both start on the stage's value: %r %r"
               % (tx.field.text(), ty.field.text()))
        txSets = _CountTextSets(tx.field)
        tySets = _CountTextSets(ty.field)
        txAttr = tx.channel.attr
        frame = Usd.TimeCode.Default()

        # The gizmo's drag: gizmoUI._BeginDrag builds a Writer, each
        # _UpdateDrag Set()s through the target and pushes the pending
        # values, _EndDrag commits once. Nothing else is wired here on
        # purpose -- no sink, no controller -- so what is being tested is
        # the Writer, the preview channel and the panel, nothing more.
        writer = gizmoMath.Writer(stage, frame, gizmoMath.WRITE_DEFAULT)
        for value in (1.0, 2.5, 2.5, 3.25):
            writer.Set(txAttr, value)
            gizmoPreview.Push(writer.Pending())
            _Check(tx.field.text() == tx._FormatValue(value),
                   "mid-drag the field shows the pending %s, got %r"
                   % (value, tx.field.text()))
            _Check(_SessionSpec(stage, txAttr) is None,
                   "nothing authored mid-drag (sample %s)" % value)
            _Check(txAttr.Get(frame) == 0.0,
                   "the stage still resolves the pre-drag value")
        _Check(tx.channel.Value(frame) == 3.25,
               "the model reports the pending value mid-drag")
        _Check(txSets == ["1", "2.5", "3.25"],
               "one text set per CHANGED value, none for the repeat: %s"
               % txSets)
        _Check(tySets == [],
               "a channel the drag did not touch was not touched: %s"
               % tySets)
        _Check(ty.field.text() == "0", "and still reads the stage")

        # Release, in _EndDrag's order: author, then drop the preview.
        authored = writer.CommitToStage()
        _Check([str(p) for p in authored] == [str(txAttr.GetPath())],
               "the release authored exactly the dragged channel: %s"
               % authored)
        gizmoPreview.End()
        _Check(_SessionSpec(stage, txAttr) is not None
               and txAttr.Get(frame) == 3.25,
               "the release authored the last value, got %s"
               % txAttr.Get(frame))
        _Check(tx.channel.PendingValue() is None,
               "nothing is pending after release")
        _Check(tx.field.text() == "3.25",
               "the field shows the authored value, got %r"
               % tx.field.text())
        _Check(tx.channel.Value(frame) == 3.25,
               "and the model reads it from the stage now")
        _Check(txSets == ["1", "2.5", "3.25"],
               "the release changed no text: the number was already right"
               " (%s)" % txSets)
    finally:
        panel.close()


def TestAbortFallsBackToTheStage():
    stage, prim = _Stage()
    panel = avarEditorUI.AvarEditorPanel(_Api(stage, prim),
                                         rigExecUndo.UndoStack())
    try:
        rz = panel.Row("rz")
        rzAttr = rz.channel.attr
        frame = Usd.TimeCode.Default()
        writer = gizmoMath.Writer(stage, frame, gizmoMath.WRITE_DEFAULT)
        writer.Set(rzAttr, 42.0)
        gizmoPreview.Push(writer.Pending())
        _Check(rz.field.text() == "42", "mid-drag shows 42, got %r"
               % rz.field.text())
        # gizmoUI._AbortDrag: End first, then the collector is cleared.
        gizmoPreview.End()
        writer.Clear()
        _Check(rz.field.text() == "0",
               "an abort puts the stage's value back, got %r"
               % rz.field.text())
        _Check(_SessionSpec(stage, rzAttr) is None,
               "and authored nothing")
    finally:
        panel.close()


def TestStageNoticeMidDragKeepsThePendingValue():
    """A refresh from a notice must not flash the authored number."""
    stage, prim = _Stage()
    panel = avarEditorUI.AvarEditorPanel(_Api(stage, prim),
                                         rigExecUndo.UndoStack())
    try:
        tx = panel.Row("tx")
        ty = panel.Row("ty")
        frame = Usd.TimeCode.Default()
        writer = gizmoMath.Writer(stage, frame, gizmoMath.WRITE_DEFAULT)
        writer.Set(tx.channel.attr, 7.0)
        gizmoPreview.Push(writer.Pending())
        # Something else edits the stage mid-drag (a script, another
        # panel): the ObjectsChanged notice refreshes every row.
        ty.channel.attr.Set(-1.0)
        _Check(ty.field.text() == "-1",
               "the notice refreshed the edited row, got %r"
               % ty.field.text())
        _Check(tx.field.text() == "7",
               "and the dragged row kept its pending value, got %r"
               % tx.field.text())
        gizmoPreview.End()
        writer.Clear()
        _Check(tx.field.text() == "0", "abort: back to the stage")
    finally:
        panel.close()


def TestClosedPanelStopsListening():
    stage, prim = _Stage()
    panel = avarEditorUI.AvarEditorPanel(_Api(stage, prim),
                                         rigExecUndo.UndoStack())
    listener = panel._previewListener
    _Check(listener is not None and listener in gizmoPreview._listeners,
           "an open panel listens to the preview channel")
    panel.close()
    _Check(listener not in gizmoPreview._listeners,
           "a closed panel does not")


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    _ = app
    gizmoPreview.SetSink(None)
    TestDragShowsPendingAndAuthorsOnRelease()
    TestAbortFallsBackToTheStage()
    TestStageNoticeMidDragKeepsThePendingValue()
    TestClosedPanelStopsListening()
    print("test_avar_editor_live OK")


if __name__ == "__main__":
    main()
