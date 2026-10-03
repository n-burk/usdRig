#!/usr/bin/env python
"""
Headless test for per-session plugin state: several usdview sessions in ONE
process (usdOrchestrate's shared host) each get their own gizmo, view cube,
editors, preview channel, published-frame reader and container, with no
cross-talk.

usdview gives every window its own AppController and UsdviewApi, but a
module exists once per process, so anything a plugin kept at module scope
used to be shared: the gizmo and the view cube installed themselves into the
first window only. plugin/rigExecUsdview/sessionRegistry.py now files that
state under the session's main window.

Fake apis over real (never shown, never GL) QMainWindows on Qt's offscreen
platform: no usdview, no rigExecImaging, no display. The controllers
themselves are replaced by doubles -- what is under test is which session
gets which, not what a gizmo draws.

Usage: test_session_state.py
"""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Sdf, Usd  # noqa: E402
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets  # noqa: E402

import gizmoMath  # noqa: E402
import gizmoPreview  # noqa: E402
import gizmoUI  # noqa: E402
import sessionRegistry  # noqa: E402
import viewCubeUI  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
    ["test_session_state"])


class _Api(object):
    """What the plugins read off a UsdviewApi: its window and its stage."""

    def __init__(self, window, stage=None):
        self.qMainWindow = window
        self.stage = stage


def _Session(stage=None):
    window = QtWidgets.QMainWindow()
    return window, _Api(window, stage)


def _Destroy(window):
    """Destroy the window's C++ object now, as closing a session does."""
    try:
        import shiboken6
        shiboken6.delete(window)
        return
    except ImportError:
        pass
    window.deleteLater()
    for _ in range(5):
        QtCore.QCoreApplication.sendPostedEvents(
            None, QtCore.QEvent.DeferredDelete)
        _APP.processEvents()


class _ActiveWindow(object):
    """Pin sessionRegistry's view of QApplication.activeWindow()."""

    def __init__(self, window):
        self._window = window
        self._saved = None

    def __enter__(self):
        self._saved = sessionRegistry.ActiveWindow
        sessionRegistry.ActiveWindow = lambda: self._window
        return self

    def __exit__(self, *args):
        sessionRegistry.ActiveWindow = self._saved
        return False


class _FakeController(object):
    built = []

    def __init__(self, usdviewApi, *args, **kwargs):
        self.usdviewApi = usdviewApi
        _FakeController.built.append(self)


class _Patched(object):
    """Swap module attributes for the duration of a block."""

    def __init__(self, module, **values):
        self._module = module
        self._values = values
        self._saved = {}

    def __enter__(self):
        for name, value in self._values.items():
            self._saved[name] = getattr(self._module, name)
            setattr(self._module, name, value)
        return self

    def __exit__(self, *args):
        for name, value in self._saved.items():
            setattr(self._module, name, value)
        return False


def TestRegistry():
    """Weak keys, destroyed windows, and no winner by recency."""
    registry = sessionRegistry.SessionRegistry("test")
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    registry.Set(apiA, "A")
    registry.Set(windowB, "B")
    _Check(registry.Get(apiA) == "A" and registry.Get(windowA) == "A",
           "an api and its window are the same session")
    _Check(registry.Get(apiB) == "B", "B is its own session")
    _Check(len(registry) == 2, "two sessions")
    with _ActiveWindow(None):
        _Check(registry.Current() is None,
               "two sessions and no active window is ambiguous: no answer, "
               "and certainly not the most recent one")
    with _ActiveWindow(windowA):
        _Check(registry.Current() == "A", "the active window's session")
    # A panel: a top-level window parented to the main window.
    panel = QtWidgets.QDialog(windowB)
    child = QtWidgets.QLineEdit(panel)
    with _ActiveWindow(panel):
        _Check(registry.Current() == "B",
               "a panel's session is its owner's")
    _Check(registry.SessionOf(child) is windowB,
           "a widget inside a panel belongs to the panel's owner")
    _Check(sessionRegistry.SessionOfEvent(child, windowB) and
           not sessionRegistry.SessionOfEvent(child, windowA),
           "SessionOfEvent follows the same chain")

    _Destroy(windowA)
    _Check(len(registry) == 1 and registry.Get(apiA) is None,
           "destroying A's window dropped A's entry")
    with _ActiveWindow(None):
        _Check(registry.Current() == "B",
               "one session left: it is the answer, as in a plain usdview")

    # A headless key (no window at all) is weakly held.
    class _Headless(object):
        pass
    key = _Headless()
    registry.Set(key, "headless")
    _Check(registry.Get(key) == "headless", "headless keys work")
    del key
    _Check(len(registry) == 1, "a collected headless key drops its entry")
    _Destroy(windowB)
    _Check(len(registry) == 0, "and B goes with its window")

    # A host ending a session without destroying its window forgets it
    # everywhere at once.
    windowC, apiC = _Session()
    other = sessionRegistry.SessionRegistry("test other")
    registry.Set(apiC, "C")
    other.Set(windowC, "C2")
    _Check(sessionRegistry.ForgetSession(apiC) >= 2,
           "ForgetSession reached every registry holding C")
    _Check(registry.Get(apiC) is None and other.Get(apiC) is None,
           "and C is gone from both")
    _Destroy(windowC)
    print("  ok: registry keys, ownership and lifetimes")


def TestGizmoAndViewCubePerSession():
    """Each session installs its own gizmo and its own view cube."""
    _FakeController.built = []
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    view = object()
    with _Patched(gizmoUI, GizmoController=_FakeController,
                  StageView=lambda api: view), \
            _Patched(viewCubeUI, ViewCubeController=_FakeController,
                     StageView=lambda api: view):
        gizmoA = gizmoUI.InstallViewportTools(apiA, undoStack=None)
        gizmoB = gizmoUI.InstallViewportTools(apiB, undoStack=None)
        cubeA = viewCubeUI.InstallViewCube(apiA)
        cubeB = viewCubeUI.InstallViewCube(apiB)
        _Check(gizmoA is not None and gizmoB is not None,
               "the SECOND window got viewport tools too")
        _Check(gizmoA is not gizmoB, "and its own controller")
        _Check(gizmoA.usdviewApi is apiA and gizmoB.usdviewApi is apiB,
               "each controller is built on its own session's api")
        _Check(cubeA is not None and cubeB is not None and
               cubeA is not cubeB, "each window has its own view cube")
        _Check(gizmoUI.InstallViewportTools(apiA, None) is gizmoA and
               viewCubeUI.InstallViewCube(apiB) is cubeB,
               "installing again is a no-op per session")
        _Check(len(_FakeController.built) == 4, "four controllers, once")

        # GetController with and without an api.
        _Check(gizmoUI.GetController(apiA) is gizmoA and
               gizmoUI.GetController(apiB) is gizmoB and
               viewCubeUI.GetController(apiB) is cubeB,
               "GetController(api) answers for that session")
        _Check(gizmoUI.GetController(windowB) is gizmoB,
               "a main window names its session too")
        with _ActiveWindow(windowB):
            _Check(gizmoUI.GetController() is gizmoB and
                   viewCubeUI.GetController() is cubeB,
                   "without an api: the active window's session")
            _Check(gizmoUI._controller is gizmoB,
                   "the old module attribute is a view onto the same")
        with _ActiveWindow(None):
            _Check(gizmoUI.GetController() is None,
                   "two sessions, no active window: None, not the newest")

        # Closing one window drops its controllers and leaves the other.
        _Destroy(windowA)
        _Check(gizmoUI.GetController(apiA) is None and
               viewCubeUI.GetController(apiA) is None,
               "A's controllers went with A's window")
        _Check(gizmoUI.GetController(apiB) is gizmoB and
               viewCubeUI.GetController(apiB) is cubeB,
               "B's are untouched")
        with _ActiveWindow(None):
            _Check(gizmoUI.GetController() is gizmoB,
                   "one session left: GetController() is its controller, "
                   "exactly as in a plain usdview")
    _Destroy(windowB)
    _Check(gizmoUI.GetController() is None, "no sessions, no controller")
    print("  ok: gizmo and view cube install once per session")


def TestPendingInstallPerSession():
    """A queued retry in one session does not block another's."""
    _FakeController.built = []
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    views = {}
    with _Patched(gizmoUI, GizmoController=_FakeController,
                  StageView=lambda api: views.get(id(api))):
        _Check(gizmoUI.InstallViewportTools(apiA, None) is None,
               "no stage view yet: A queues")
        _Check(gizmoUI.InstallViewportTools(apiB, None) is None,
               "and so does B")
        _Check(gizmoUI._installPending.Get(apiA) and
               gizmoUI._installPending.Get(apiB),
               "each session has its own pending retry")
        views[id(apiA)] = object()
        views[id(apiB)] = object()
        for _ in range(10):
            _APP.processEvents()
            if (gizmoUI.GetController(apiA) is not None and
                    gizmoUI.GetController(apiB) is not None):
                break
        _Check(gizmoUI.GetController(apiA) is not None and
               gizmoUI.GetController(apiB) is not None,
               "both retries installed their own session's tools")
        _Check(not gizmoUI._installPending.Get(apiA, False) and
               not gizmoUI._installPending.Get(apiB, False),
               "and cleared their own pending flags")
    _Destroy(windowA)
    _Destroy(windowB)
    print("  ok: pending installs are per session")


class _FilterController(QtCore.QObject):
    """Enough of GizmoController for ViewportHotkeyFilter: the real
    ownership gate, recorded actions."""

    OwnsWindowEvent = gizmoUI.GizmoController.OwnsWindowEvent
    _MainWindow = gizmoUI.GizmoController._MainWindow
    _InMainWindow = gizmoUI.GizmoController._InMainWindow
    HandleHotkey = gizmoUI.GizmoController.HandleHotkey
    _TypingFocus = staticmethod(gizmoUI.GizmoController._TypingFocus)
    _Claim = staticmethod(gizmoUI.GizmoController._Claim)
    _DRAG_KEYS = gizmoUI.GizmoController._DRAG_KEYS

    def __init__(self, usdviewApi):
        super(_FilterController, self).__init__()
        self.usdviewApi = usdviewApi
        self.cleared = 0
        self.acted = []
        self._visible = True
        self._view = QtWidgets.QWidget(usdviewApi.qMainWindow)
        self._claimedKey = None
        self._drag = None
        self.toolbar = self

    def Sync(self):
        pass

    def _ClearHolds(self):
        self.cleared += 1

    def _ReleaseHold(self, event, key):
        self.acted.append(("release", key))
        return False

    def _Act(self, kind, key, handler):
        self.acted.append(("act", key))
        return False

    def _CursorOverView(self):
        return True

    def _ToolKey(self, kind, key):
        return False

    def _DragKey(self, kind, key):
        return False


def TestHotkeyRouting():
    """One window's keys and focus changes never drive another session."""
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    ctrlA = _FilterController(apiA)
    ctrlB = _FilterController(apiB)
    filterA = gizmoUI.ViewportHotkeyFilter(ctrlA)
    filterB = gizmoUI.ViewportHotkeyFilter(ctrlB)
    widgetA = QtWidgets.QWidget(windowA)
    panelB = QtWidgets.QDialog(windowB)
    try:
        deactivate = QtCore.QEvent(QtCore.QEvent.WindowDeactivate)
        for event_filter in (filterA, filterB):
            event_filter.eventFilter(widgetA, deactivate)
        _Check(ctrlA.cleared == 1 and ctrlB.cleared == 0,
               "A's window deactivating clears A's holds only: %d/%d"
               % (ctrlA.cleared, ctrlB.cleared))
        for event_filter in (filterA, filterB):
            event_filter.eventFilter(panelB, deactivate)
        _Check(ctrlA.cleared == 1 and ctrlB.cleared == 1,
               "B's panel deactivating is B's business only")

        press = QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_W,
                                QtCore.Qt.NoModifier)
        release = QtGui.QKeyEvent(QtCore.QEvent.KeyRelease, QtCore.Qt.Key_W,
                                  QtCore.Qt.NoModifier)
        _Check(filterB.eventFilter(widgetA, press) is False and
               filterB.eventFilter(widgetA, release) is False,
               "B's filter ignores keys in A's window")
        _Check(ctrlB.acted == [],
               "without acting on them, not even a hold release: %r"
               % ctrlB.acted)
        filterA.eventFilter(widgetA, press)
        filterA.eventFilter(widgetA, release)
        _Check(("act", QtCore.Qt.Key_W) in ctrlA.acted and
               ("release", QtCore.Qt.Key_W) in ctrlA.acted,
               "A's filter does act on them: %r" % ctrlA.acted)
    finally:
        _Destroy(windowA)
        _Destroy(windowB)
    print("  ok: hotkey filters act for their own session only")


class _ShortcutController(object):
    """A controller double with real undo / redo actions and the real
    scoping rule."""

    _ScopeShortcuts = gizmoUI.GizmoController._ScopeShortcuts
    _MainWindow = gizmoUI.GizmoController._MainWindow

    def __init__(self, usdviewApi, *args, **kwargs):
        self.usdviewApi = usdviewApi
        window = usdviewApi.qMainWindow
        self.actions = []
        for name in ("undo", "redo"):
            action = gizmoUI.QtActionWidgets.QAction(name, window)
            action.setShortcutContext(QtCore.Qt.ApplicationShortcut)
            self.actions.append(action)

    def _UndoActions(self):
        return list(self.actions)

    def Contexts(self):
        return set(a.shortcutContext() for a in self.actions)


def TestUndoShortcutScoping():
    """Only the session in front keeps application-wide undo / redo."""
    app = QtCore.Qt.ApplicationShortcut
    win = QtCore.Qt.WindowShortcut
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    view = object()
    with _Patched(gizmoUI, GizmoController=_ShortcutController,
                  StageView=lambda api: view):
        with _Patched(QtWidgets.QApplication,
                      activeWindow=staticmethod(lambda: windowA)):
            ctrlA = gizmoUI.InstallViewportTools(apiA, None)
            _Check(ctrlA.Contexts() == {app},
                   "alone in the process: application-wide, as ever")
            ctrlB = gizmoUI.InstallViewportTools(apiB, None)
            _Check(ctrlA.Contexts() == {app} and ctrlB.Contexts() == {win},
                   "A in front: A's undo is application-wide, B's is "
                   "window-scoped, so Ctrl+Z is never ambiguous")
        with _Patched(QtWidgets.QApplication,
                      activeWindow=staticmethod(lambda: windowB)):
            for controller in (ctrlA, ctrlB):
                controller._ScopeShortcuts()
            _Check(ctrlA.Contexts() == {win} and ctrlB.Contexts() == {app},
                   "focus moved to B: the roles swap")
        with _Patched(QtWidgets.QApplication,
                      activeWindow=staticmethod(lambda: None)):
            ctrlA._ScopeShortcuts()
            _Check(ctrlA.Contexts() == {win},
                   "a window no session owns changes nothing")
        _Destroy(windowB)
        ctrlA._ScopeShortcuts()
        _Check(ctrlA.Contexts() == {app},
               "the last session left is application-wide again")
    _Destroy(windowA)
    print("  ok: undo / redo shortcuts scoped to the session in front")


def _TwinStages():
    layer = Sdf.Layer.CreateAnonymous("twin.usda")
    stage = Usd.Stage.Open(layer)
    stage.DefinePrim("/Asset", "Scope")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    prim = stage.DefinePrim("/Asset/Rig/Ctrl", "Xform")
    prim.CreateAttribute("avars:tx", Sdf.ValueTypeNames.Double).Set(0.0)
    first = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous("sessionA"))
    second = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous("sessionB"))
    return first, second


def TestPublishedReadersPerSession():
    """Each session's reader answers for its own stage, same paths or not."""
    first, second = _TwinStages()
    windowA, apiA = _Session(first)
    windowB, apiB = _Session(second)
    path = Sdf.Path("/Asset/Rig/Ctrl")
    matrixA = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(1, 0, 0))
    matrixB = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 2, 0))

    def _Reader(own, matrix):
        return lambda stage, p, time: matrix if stage == own else None

    gizmoMath.SetPublishedControlFrameReader(_Reader(first, matrixA),
                                             session=apiA)
    gizmoMath.SetPublishedControlFrameReader(_Reader(second, matrixB),
                                             session=apiB)
    try:
        time = Usd.TimeCode(1)
        _Check(gizmoMath._ReadPublishedControlFrame(first, path, time)
               == matrixA, "A's stage reads A's published frame")
        _Check(gizmoMath._ReadPublishedControlFrame(second, path, time)
               == matrixB, "B's stage, same path, reads B's")
        _Destroy(windowA)
        _Check(gizmoMath._ReadPublishedControlFrame(first, path, time)
               is None, "A's reader went with A's window")
        _Check(gizmoMath._ReadPublishedControlFrame(second, path, time)
               == matrixB, "B's did not")
    finally:
        gizmoMath.SetPublishedControlFrameReader(None, session=apiB)
        _Destroy(windowB)
    _Check(not gizmoMath._HasPublishedControlFrameReader(),
           "no readers left")
    print("  ok: published-frame readers are per session")


def TestPreviewPerSession():
    """Previews: one channel per session, values per stage."""
    first, second = _TwinStages()
    windowA, apiA = _Session(first)
    windowB, apiB = _Session(second)
    attrPath = Sdf.Path("/Asset/Rig/Ctrl.avars:tx")
    attrA = first.GetAttributeAtPath(attrPath)
    attrB = second.GetAttributeAtPath(attrPath)

    class _Sink(object):
        def __init__(self):
            self.updates = []
            self.ends = 0

        def Begin(self, packed):
            return len(packed.split("\n"))

        def Update(self, values):
            self.updates.append(list(values))
            return True

        def End(self):
            self.ends += 1
            return True

    sinkA, sinkB = _Sink(), _Sink()
    gizmoPreview.SetSink(sinkA, session=apiA)
    gizmoPreview.SetSink(sinkB, session=apiB)
    try:
        writerA = gizmoMath.Writer(first, Usd.TimeCode(1),
                                   gizmoMath.WRITE_ANIMATION)
        writerA.Set(attrA, 5.0)
        _Check(gizmoMath._Previewed(attrA) == 5.0,
               "A's drag previews A's attribute")
        _Check(gizmoMath._Previewed(attrB) is None,
               "and NOT the same path in B's stage")
        _Check(gizmoPreview.Push(writerA.Pending(), session=apiA,
                                 stage=first),
               "A's sample reached A's sink")
        _Check(sinkA.updates == [[5.0]] and sinkB.updates == [],
               "and only A's sink")
        _Check(gizmoPreview.IsPreviewing(apiA) and
               not gizmoPreview.IsPreviewing(apiB),
               "only A is previewing")

        # B previews the same path at the same time.
        gizmoMath.SetPreviewValue(attrPath, 9.0, stage=second)
        _Check(gizmoPreview.Push({attrPath: 9.0}, session=apiB,
                                 stage=second), "B previews too")
        _Check(gizmoMath._Previewed(attrA) == 5.0 and
               gizmoMath._Previewed(attrB) == 9.0,
               "two stages, one path, two values")

        # B's End drops B's values and leaves A's drag alone.
        gizmoPreview.End(session=apiB, stage=second)
        _Check(gizmoMath._Previewed(attrB) is None,
               "B's end dropped B's value")
        _Check(gizmoMath._Previewed(attrA) == 5.0,
               "and A's drag still holds its own")
        _Check(sinkB.ends == 1 and sinkA.ends == 0, "B's sink ended only")
        writerA.Clear()
        gizmoPreview.End(session=apiA, stage=first)
        _Check(gizmoMath.PreviewValues() == {}, "everything is dropped")
    finally:
        gizmoPreview.SetSink(None, session=apiA)
        gizmoPreview.SetSink(None, session=apiB)
        gizmoMath.SetPreviewValues({})
        _Destroy(windowA)
        _Destroy(windowB)
    print("  ok: preview channels per session, values per stage")


def TestMemoScopePerStage():
    """A memo scope opened for one stage never answers another's."""
    first, second = _TwinStages()
    primA = first.GetPrimAtPath("/Asset/Rig/Ctrl")
    primB = second.GetPrimAtPath("/Asset/Rig/Ctrl")
    with gizmoMath.MemoScope(first) as scope:
        _Check(gizmoMath._AmbientScope(primA) is scope,
               "the scope serves its own stage")
        _Check(gizmoMath._AmbientScope(primB) is None,
               "and not the same path in another stage")
        ctxA = gizmoMath._Context(None, Usd.TimeCode(1), primA)
        ctxB = gizmoMath._Context(None, Usd.TimeCode(1), primB)
        _Check(ctxA is not ctxB, "so B gets a context of its own")
    with gizmoMath.MemoScope() as legacy:
        _Check(gizmoMath._AmbientScope(primB) is legacy,
               "a scope with no stage serves any call, as before")
    _Check(gizmoMath._AmbientScope(primA) is None, "scopes close")
    print("  ok: memo scopes are bound to their stage")


def TestPanelInstanceView():
    """`Cls._instance` of a per-session panel class is a view."""

    class _Panel(QtWidgets.QDialog,
                 metaclass=sessionRegistry.PerSessionInstanceMeta(
                     QtWidgets.QDialog)):
        _sessions = sessionRegistry.SessionRegistry("test panels")

        def __init__(self, api):
            super(_Panel, self).__init__(api.qMainWindow)
            self._api = api

        @classmethod
        def GetInstance(cls, api):
            panel = cls._sessions.Get(api)
            if panel is None:
                panel = cls._sessions.Set(api, cls(api))
            return panel

    windowA, apiA = _Session()
    windowB, apiB = _Session()
    panelA = _Panel.GetInstance(apiA)
    _Check(_Panel._instance is panelA, "one session: its panel")
    panelB = _Panel.GetInstance(apiB)
    _Check(panelA is not panelB, "each session has its own panel")
    _Check(_Panel.GetInstance(apiA) is panelA, "and keeps it")
    with _ActiveWindow(panelB):
        _Check(_Panel._instance is panelB,
               "_instance is the active session's panel")
        _Panel._instance = None
        _Check(_Panel._sessions.Get(apiB) is None and
               _Panel._sessions.Get(apiA) is panelA,
               "assigning None forgets the current session's panel only")
    _Destroy(windowA)
    _Check(len(_Panel._sessions) == 0, "A's panel went with its window")
    _Destroy(windowB)
    print("  ok: per-session _instance view")


def TestContainerPerSession():
    """rigExecUsdview files one container per session."""
    import rigExecUsdview

    class _Signal(object):
        def connect(self, slot):
            pass

    class _Registry(object):
        def registerCommandPlugin(self, name, label, callback):
            return (name, callback)

    def _ContainerApi(window):
        api = _Api(window)
        api.dataModel = type("DataModel", (), {})()
        api.dataModel.stage = None
        api.dataModel.signalStageReplaced = _Signal()
        api.dataModel.currentFrameChanged = _Signal()
        return api

    windowA, windowB = QtWidgets.QMainWindow(), QtWidgets.QMainWindow()
    apiA, apiB = _ContainerApi(windowA), _ContainerApi(windowB)
    containerA = rigExecUsdview.RigExecUsdviewContainer()
    containerB = rigExecUsdview.RigExecUsdviewContainer()
    containerA.registerPlugins(_Registry(), apiA)
    containerB.registerPlugins(_Registry(), apiB)
    _Check(rigExecUsdview.ContainerFor(apiA) is containerA and
           rigExecUsdview.ContainerFor(apiB) is containerB,
           "each session finds its own container")
    with _ActiveWindow(windowA):
        _Check(rigExecUsdview._container is containerA,
               "the old module attribute views the active session's")
    with _ActiveWindow(None):
        _Check(rigExecUsdview.ContainerFor() is None,
               "ambiguous without an active window")
    _Destroy(windowA)
    _Check(rigExecUsdview.ContainerFor(apiB) is containerB and
           rigExecUsdview._container is containerB,
           "A's container is dropped with its window; B's stays")
    _Destroy(windowB)
    print("  ok: one RigExec container per session")


def TestRegistryDestroyedSlots():
    """Filing and removing a session repeatedly adds no slots to its window.

    Every Set used to connect a fresh closure to the window's `destroyed`
    signal and Pop never disconnected it, so a panel closed and reopened
    (or an install retried) grew the long-lived window's signal forever.
    """
    class _Signal(object):
        def __init__(self):
            self.slots = []

        def connect(self, slot):
            self.slots.append(slot)

        def disconnect(self, slot):
            self.slots.remove(slot)

        def emit(self):
            for slot in list(self.slots):
                slot()

    class _Window(object):
        def __init__(self):
            self.destroyed = _Signal()

    registry = sessionRegistry.SessionRegistry("test slots")
    window = _Window()
    for _ in range(20):
        registry.Set(window, True)
        registry.Pop(window)
    _Check(len(window.destroyed.slots) == 0 and len(registry) == 0,
           "20 Set/Pop cycles leave no destroyed slot behind (%d)"
           % len(window.destroyed.slots))
    registry.Set(window, "a")
    registry.Set(window, "b")
    _Check(len(window.destroyed.slots) == 1,
           "replacing a live entry reuses its one connection")
    window.destroyed.emit()
    _Check(len(registry) == 0, "and that connection still drops the entry")
    registry.Set(window, "c")
    other = sessionRegistry.SessionRegistry("test slots other")
    other.Set(window, "d")
    _Check(len(window.destroyed.slots) == 3,
           "one slot per live entry (the emitted one stays until Qt drops "
           "it with the dying window)")
    _Check(sessionRegistry.ForgetSession(window) == 2 and
           len(window.destroyed.slots) == 1,
           "ForgetSession disconnects what it forgets")
    registry.Set(window, "e")
    registry.Clear()
    _Check(len(window.destroyed.slots) == 1, "and so does Clear")

    # The same on a real window: its receiver count stays put, and the
    # entry still goes with the window.
    windowQ, apiQ = _Session()
    signature = QtCore.SIGNAL("destroyed(QObject*)")
    # PySide keeps one receiver of its own on an object once any Python
    # slot was connected to it; the baseline is taken after that.
    registry.Set(apiQ, True)
    registry.Pop(apiQ)
    baseline = windowQ.receivers(signature)
    for _ in range(20):
        registry.Set(apiQ, True)
        registry.Pop(apiQ)
    _Check(windowQ.receivers(signature) == baseline,
           "a real window's destroyed signal did not grow (%d -> %d)"
           % (baseline, windowQ.receivers(signature)))
    registry.Set(apiQ, True)
    _Check(windowQ.receivers(signature) == baseline + 1,
           "a filed session is watched once")
    _Destroy(windowQ)
    _Check(len(registry) == 0, "and is dropped with its window")
    print("  ok: registry connections are owned by their entries")


def TestTouchPoseMarqueeUsesOwnSession():
    """TouchPose's marquee projects through ITS session's gizmo.

    It read gizmoUI's `_controller` view, which answers for the active
    window's session: nothing when that is ambiguous (every control dropped
    from the band), another session's camera when that window is active.
    """
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    touchPoseDir = os.path.join(root, "plugin", "touchPose")
    if touchPoseDir not in sys.path:
        sys.path.insert(0, touchPoseDir)
    import gizmoMarquee
    import touchPoseUI

    stageA = Usd.Stage.CreateInMemory()
    stageB = Usd.Stage.CreateInMemory()
    windowA, apiA = _Session(stageA)
    windowB, apiB = _Session(stageB)
    apiA.frame = apiB.frame = 1.0
    calls = []

    class _Gizmo(object):
        def __init__(self, usdviewApi, *args, **kwargs):
            self.usdviewApi = usdviewApi
            self._solverPosed = ("posed", usdviewApi)

        def _Camera(self):
            return ("camera", self.usdviewApi), ("viewport",), 1.0

    def _ControlsInBand(stage, camera, viewport, frame, band, posed):
        calls.append((stage, camera, posed))
        return ["/ctrl"]

    class _TouchPose(object):
        """Just what _ControlsInBand reads off the controller."""

        def __init__(self, api):
            self._api = api

        def _Stage(self):
            return self._api.stage

    view = object()
    with _Patched(gizmoUI, GizmoController=_Gizmo,
                  StageView=lambda api: view),             _Patched(gizmoMarquee, ControlsInBand=_ControlsInBand):
        gizmoA = gizmoUI.InstallViewportTools(apiA, undoStack=None)
        gizmoB = gizmoUI.InstallViewportTools(apiB, undoStack=None)
        band = touchPoseUI.TouchPoseController._ControlsInBand
        for active in (None, windowA, windowB):
            del calls[:]
            with _ActiveWindow(active):
                picked = band(_TouchPose(apiB), 0, 0, 10, 10)
            _Check(picked == ["/ctrl"],
                   "B's band picks controls whatever window is active "
                   "(%r)" % (active,))
            _Check(calls == [(stageB, ("camera", apiB), ("posed", apiB))],
                   "through B's own camera and posed cache, on B's stage")
        _Check(gizmoA is not gizmoB, "two gizmos")
    _Destroy(windowA)
    _Destroy(windowB)
    print("  ok: TouchPose marquee uses its own session's gizmo")


def main():
    TestRegistry()
    TestGizmoAndViewCubePerSession()
    TestPendingInstallPerSession()
    TestHotkeyRouting()
    TestUndoShortcutScoping()
    TestPublishedReadersPerSession()
    TestPreviewPerSession()
    TestMemoScopePerStage()
    TestPanelInstanceView()
    TestContainerPerSession()
    TestRegistryDestroyedSlots()
    TestTouchPoseMarqueeUsesOwnSession()
    print("SESSION_STATE_OK (12 groups)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
