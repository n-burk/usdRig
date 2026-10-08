"""Owner-thread queued release, retained preview and lifecycle drain regressions."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import importlib
import importlib.util
import tempfile
import threading
import types
import unittest
from pathlib import Path

import rigexec_test_env
rigexec_test_env.SetupPluginTest()
if not any(importlib.util.find_spec(binding) for binding in ('PySide6', 'PySide2')):
    print('SKIP: deferred release lifecycle tests require the usdview Qt binding')
    raise SystemExit(77)
from pxr import Sdf, Usd
from pxr.Usdviewq.qt import QtCore, QtWidgets, QtActionWidgets
import gizmoMath
import gizmoPreview
import gizmoUI
import rigExecUndo

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
QtTest = importlib.import_module(QtCore.__name__.split('.')[0] + '.QtTest')


class Sink:
    def __init__(self): self.ends = []
    def Begin(self, paths): return len(paths.split('\n'))
    def Update(self, values): return True
    def End(self, publish=True): self.ends.append(publish); return True


class Writer(gizmoMath.Writer):
    def __init__(self, stage, mode):
        super().__init__(stage, Usd.TimeCode(1), mode)
        self.commits = []
    def CommitToStage(self):
        self.commits.append(threading.get_ident())
        return super().CommitToStage()


class Toolbar(QtWidgets.QToolBar):
    Sync = gizmoUI.ViewportToolbar.Sync
    def __init__(self, controller, window):
        super().__init__(window)
        self._controller = controller
        self._toolActions = {}; self._channelActions = {}; self._writeActions = {}
        self._SyncOrientation = self._SyncSnap = self._SyncGroupPivot = lambda: None
        self._status = types.SimpleNamespace(SetStatus=lambda status: None)
        self.undoAction = QtActionWidgets.QAction('Undo', self)
        self.undoAction.setShortcut('Ctrl+Z')
        self.undoAction.setShortcutContext(QtCore.Qt.ApplicationShortcut)
        self.undoAction.triggered.connect(controller.Undo)
        self.redoAction = QtActionWidgets.QAction('Redo', self)
        self.redoAction.triggered.connect(controller.Redo)
        self.addAction(self.undoAction); self.addAction(self.redoAction)
        window.addAction(self.undoAction)


class Controller(gizmoUI.GizmoController):
    """Real release/lifecycle methods with viewport drawing omitted."""
    def __init__(self, stage):
        QtCore.QObject.__init__(self)
        self.window = QtWidgets.QMainWindow()
        self.usdviewApi = types.SimpleNamespace(stage=stage, frame=Usd.TimeCode(1),
            qMainWindow=self.window, dataModel=types.SimpleNamespace(stage=stage),
            UpdateViewport=lambda: None)
        self.undoStack = rigExecUndo.UndoStack()
        self._drag = self._releasedEdit = self._committing = self._target = None
        self._frame = Usd.TimeCode(1)
        self._view = self.overlay = self._rubber = None
        self._controlPicking = None; self._viewDestroyed = False
        self._snapGeom = {}; self._solverPosed = types.SimpleNamespace(Clear=lambda: None)
        self._warnings = []; self._hoverRigStage = None
        self._openGraphEditor = lambda: 'opened'
        self.toolbar = Toolbar(self, self.window)
        self.window.addToolBar(self.toolbar)
        self.undoStack.AddListener(self._onUndoStackChanged)
        self._hotkeys = gizmoUI.ViewportHotkeyFilter(self)
        APP.installEventFilter(self._hotkeys)
        APP.focusWindowChanged.connect(self._onFocusWindowChanged)
        APP.aboutToQuit.connect(self.FlushReleasedEdit)
        self.toolbar.Sync()
    def _MainWindow(self): return self.window
    def _RebuildHandles(self): pass
    def _Repaint(self): pass
    def _AfterUndoRedo(self): pass
    def _ObserveStage(self, stage): pass
    def _PrimePreserveChildren(self): pass
    def _PrimeGroupPivot(self): pass
    def _RefreshHoverSnap(self): pass
    def RefreshTarget(self): self.FlushReleasedEdit()
    def HandleHotkey(self, *args): return False
    def Status(self): return ''
    def Tool(self): return gizmoUI.TOOL_TRANSLATE
    def Channels(self): return gizmoMath.CHANNELS_POSE
    def WriteMode(self): return gizmoMath.WRITE_DEFAULT


class DeferredRelease(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.attr = self.stage.DefinePrim('/Control').CreateAttribute('value', Sdf.ValueTypeNames.Double)
        self.attr.Set(0.)
        self.controller = Controller(self.stage)
        self.sink = Sink()
        gizmoPreview.SetSink(self.sink, session=self.controller.usdviewApi)
    def tearDown(self):
        self.controller.Detach()
        self.controller.window.close()
        APP.processEvents()
    def release(self, value=2., mode=gizmoMath.WRITE_DEFAULT):
        writer = Writer(self.stage, mode)
        recorder = rigExecUndo.EditRecorder(self.stage, [self.attr.GetPath()]); recorder.Begin()
        writer.Set(self.attr, value)
        target = types.SimpleNamespace(writer=writer, stage=self.stage, time=writer.time,
                                       label='Control', prim=self.attr.GetPrim())
        drag = types.SimpleNamespace(target=target, recorder=recorder, tool=gizmoUI.TOOL_TRANSLATE)
        self.controller._target = target
        self.controller._drag = drag
        gizmoPreview.Push(writer.Pending(), session=self.controller.usdviewApi, stage=self.stage)
        self.controller._EndDrag()
        return writer, drag
    def test_release_defers_authoring_and_keeps_preview(self):
        owner = threading.get_ident()
        writer, drag = self.release()
        self.assertEqual(self.attr.Get(), 0.)
        self.assertEqual(writer.commits, [])
        self.assertFalse(self.controller.IsDragging())
        self.assertTrue(gizmoPreview.Channel(self.controller.usdviewApi).IsPreviewing())
        self.assertEqual(self.sink.ends, [])
        APP.processEvents()
        self.assertEqual(self.attr.Get(), 2.)
        self.assertEqual(writer.commits, [owner])
        self.assertEqual(len(self.controller.undoStack._undo), 1)
        self.assertEqual(self.sink.ends, [False])
        self.controller._FlushReleasedToken(drag)
        self.assertEqual(len(writer.commits), 1)
    def test_first_release_immediate_toolbar_undo(self):
        writer, drag = self.release()
        self.assertFalse(self.controller.undoStack.CanUndo())
        self.assertTrue(self.controller.toolbar.undoAction.isEnabled())
        self.controller.toolbar.undoAction.trigger()
        self.assertEqual(self.attr.Get(), 0.)
        self.assertTrue(self.controller.undoStack.CanRedo())
        APP.processEvents()
        self.assertEqual(len(writer.commits), 1)
        self.assertEqual(self.attr.Get(), 0.)
        self.controller.Redo(); self.assertEqual(self.attr.Get(), 2.)
    def test_first_release_immediate_ctrl_z(self):
        self.controller.window.show(); APP.processEvents()
        writer, drag = self.release()
        QtTest.QTest.keyClick(self.controller.window, QtCore.Qt.Key_Z, QtCore.Qt.ControlModifier)
        self.assertEqual(self.attr.Get(), 0.)
        self.assertTrue(self.controller.undoStack.CanRedo())
        APP.processEvents()
        self.assertEqual(len(writer.commits), 1)
    def test_next_drag_flushes_before_reading_target(self):
        writer, drag = self.release()
        self.controller._target = None
        self.assertFalse(self.controller._BeginDrag(types.SimpleNamespace(grabbable=True), (0,0)))
        self.assertEqual(self.attr.Get(), 2.)
        APP.processEvents(); self.assertEqual(len(writer.commits), 1)
    def test_frame_change_preserves_original_author_time(self):
        writer, drag = self.release(mode=gizmoMath.WRITE_ANIMATION)
        self.controller._target = None
        self.controller._onFrameChanged(2.)
        self.assertEqual(writer.time, Usd.TimeCode(1))
        self.assertEqual(self.controller._frame, Usd.TimeCode(2))
        self.assertIsNotNone(self.attr.GetSpline().GetKnot(1.))
        self.assertIsNone(self.attr.GetSpline().GetKnot(2.))
        APP.processEvents(); self.assertEqual(len(writer.commits), 1)
    def test_save_input_flushes_before_host_action(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'saved.usda'
            button = QtWidgets.QPushButton('Save', self.controller.window)
            button.clicked.connect(lambda: self.stage.GetRootLayer().Export(str(path)))
            writer, drag = self.release()
            QtTest.QTest.mouseClick(button, QtCore.Qt.LeftButton)
            saved = Usd.Stage.Open(str(path))
            self.assertEqual(saved.GetAttributeAtPath('/Control.value').Get(), 2.)
            APP.processEvents(); self.assertEqual(len(writer.commits), 1)
    def test_stage_replacement_cancels_stale_callback(self):
        writer, drag = self.release()
        other = Usd.Stage.CreateInMemory(); other.DefinePrim('/Other')
        before = other.GetRootLayer().ExportToString()
        self.controller.usdviewApi.stage = other
        self.controller.usdviewApi.dataModel.stage = other
        self.controller._onStageReplaced()
        APP.processEvents()
        self.assertEqual(writer.commits, [])
        self.assertEqual(self.attr.Get(), 0.)
        self.assertEqual(other.GetRootLayer().ExportToString(), before)
        self.assertFalse(self.controller.undoStack.CanUndo())
        self.assertFalse(gizmoPreview.Channel(self.controller.usdviewApi).IsPreviewing())
    def test_destroyed_view_commits_without_touching_deleted_toolbar(self):
        writer, drag = self.release()
        self.controller.toolbar.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        self.controller._onViewDestroyed()
        self.assertEqual(self.attr.Get(), 2.)
        APP.processEvents(); self.assertEqual(len(writer.commits), 1)
    def test_quit_signal_flushes_once(self):
        writer, drag = self.release()
        APP.aboutToQuit.emit()
        self.assertEqual(self.attr.Get(), 2.)
        APP.processEvents(); self.assertEqual(len(writer.commits), 1)
    def test_old_callback_cannot_commit_a_new_release(self):
        first, first_drag = self.release()
        self.controller.FlushReleasedEdit()
        second, second_drag = self.release(3.)
        self.controller._FlushReleasedToken(first_drag)
        self.assertEqual(second.commits, [])
        self.assertEqual(self.attr.Get(), 2.)
        APP.processEvents()
        self.assertEqual(len(first.commits), 1)
        self.assertEqual(len(second.commits), 1)
        self.assertEqual(len(self.controller.undoStack._undo), 2)
    def test_target_changing_actions_flush(self):
        for action in ('RefreshTarget','OpenGraphEditor','_onSelectionChanged','_onSettingsChanged'):
            with self.subTest(action=action):
                writer, drag = self.release(3.)
                self.controller._target = None
                self.controller._panel = None
                getattr(self.controller,action)()
                self.assertEqual(self.attr.Get(), 3.)
                APP.processEvents(); self.assertEqual(len(writer.commits), 1)
                self.attr.Set(0.)


if __name__ == '__main__': unittest.main()
