#
# testusdview verification for the live RigExec Hydra integration:
# asserts the usdview plugin activated evaluation (an initial generation
# was published) and that timeline changes drive per-frame publications
# through the exact signal path the interactive app uses.
#
import ctypes
import os

from pxr import Usd, Sdf


MESH = "/Shot/HeroArm/Geom/ArmBody"


def _Observer():
    """A HydraObserver on the app's own TERMINAL scene index."""
    from pxr.Usdviewq._usdviewq import HydraObserver

    names = HydraObserver.GetRegisteredSceneIndexNames()
    if not names:
        raise AssertionError("no registered scene indices")
    observer = HydraObserver()
    observer.TargetToNamedSceneIndex(names[-1])
    return observer


def _Points(observer):
    """The deformed points the terminal scene index is serving."""
    primType, dataSource = observer.GetPrim(Sdf.Path(MESH))
    if not dataSource or "primvars" not in dataSource.GetNames():
        raise AssertionError("no primvars on %s" % MESH)
    primvars = dataSource.Get("primvars")
    entry = primvars.Get("points")
    value = entry.Get("primvarValue") if entry else None
    if not value:
        raise AssertionError("no points on %s" % MESH)
    return [tuple(round(float(c), 5) for c in p)
            for p in value.GetValue(0.0)]


def _AssertActivationFailureKeepsStageAlive():
    """A failed RigExec activation must not invalidate usdview's stage."""
    from rigExecUsdview import RigExecUsdviewContainer

    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Rig", "RigExecRoot")

    class _FailureLibrary:
        def __init__(self):
            self.activations = 0

        def RigExecImaging_Deactivate(self):
            pass

        def RigExecImaging_Activate(self, cacheId, rigPath, frame):
            self.activations += 1
            return 3

    dataModel = type("DataModel", (), {})()
    dataModel.stage = stage
    dataModel.currentFrame = Usd.TimeCode.Default()
    api = type("UsdviewApi", (), {})()
    api.dataModel = dataModel

    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._api = api
    container._lib = _FailureLibrary()
    container._active = False
    container._rigPaths = []
    container._cachedStage = None

    try:
        container._OnStageReplaced()
        if container._lib.activations != 1:
            raise AssertionError("activation-failure branch was not reached")
        if container._active:
            raise AssertionError("failed activation remained active")
        if container._cachedStage is not stage:
            raise AssertionError("failed activation released the live stage")
        if not stage or not stage.GetPrimAtPath("/Rig"):
            raise AssertionError("failed activation invalidated usdview's stage")
    finally:
        container._Shutdown()


def _AssertLiveRootAutoActivation():
    """A rig authored into the open blank stage activates by itself."""
    from rigExecUsdview import RigExecUsdviewContainer

    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World", "Xform")

    class _SuccessLibrary:
        def __init__(self):
            self.activations = 0
            self.deactivations = 0

        def RigExecImaging_Deactivate(self):
            self.deactivations += 1

        def RigExecImaging_Activate(self, cacheId, rigPath, frame):
            self.activations += 1
            return 0

    viewSettings = type("ViewSettings", (), {})()
    viewSettings.displayGuide = False
    dataModel = type("DataModel", (), {})()
    dataModel.stage = stage
    dataModel.currentFrame = Usd.TimeCode.Default()
    dataModel.viewSettings = viewSettings
    api = type("UsdviewApi", (), {})()
    api.dataModel = dataModel

    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._api = api
    container._lib = _SuccessLibrary()
    container._active = False
    container._rigPaths = []
    container._cachedStage = None
    container._stageNoticeKey = None
    container._activating = False

    try:
        container._OnStageReplaced()
        stage.DefinePrim("/Rig", "RigExecRoot")
        stage.DefinePrim("/Rig/Joints", "Scope")
        if container._lib.activations:
            raise AssertionError("empty live-authored rig activated too early")

        stage.DefinePrim("/Rig/Joints/J", "RigExecJoint")
        if container._lib.activations != 1 or not container._active:
            raise AssertionError("first live-authored joint did not activate")
        if container._rigPaths != [Sdf.Path("/Rig")]:
            raise AssertionError("live root set was not recorded")
        if not viewSettings.displayGuide:
            raise AssertionError("live activation did not enable guides")

        stage.RemovePrim("/Rig")
        if container._active or container._rigPaths:
            raise AssertionError("removing the last live root stayed active")
    finally:
        container._Shutdown()


def _AssertWarmingFlushOnRelease():
    """A pending edit-commit flushes at the held playhead, once per burst."""
    from rigExecUsdview import RigExecUsdviewContainer

    class _CommitLibrary:
        def __init__(self):
            self.commits = 0

        def RigExecImaging_OnEditCommitted(self):
            self.commits += 1

    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._warmingCommitPending = True
    container._warmingFlushArmed = True
    container._active = True
    container._lib = _CommitLibrary()

    # The release path consumes the pending commit and calls through once.
    container._FlushWarmingCommit()
    if container._warmingCommitPending or container._warmingFlushArmed:
        raise AssertionError("flush did not consume the pending commit")
    if container._lib.commits != 1:
        raise AssertionError("flush did not commit exactly once")

    # An idle flush is a no-op: nothing pending, no C-API call.
    container._FlushWarmingCommit()
    if container._lib.commits != 1:
        raise AssertionError("idle flush committed")

    # An inactive container fires nothing and leaves the flag standing for
    # the next frame tick, which is the no-Qt fallback.
    container._warmingCommitPending = True
    container._active = False
    container._FlushWarmingCommit()
    if container._lib.commits != 1:
        raise AssertionError("inactive flush committed")
    if not container._warmingCommitPending:
        raise AssertionError("inactive flush consumed the pending commit")

    # The scheduler coalesces a burst: an armed flush does not re-arm. (The
    # Qt timer itself needs the usdview event loop to fire, so only the
    # already-armed early-out is asserted here; the deferral below covers
    # the timer's own shape.)
    container._warmingFlushArmed = True
    container._ScheduleWarmingFlush()
    if not container._warmingFlushArmed:
        raise AssertionError("armed flush disarmed itself")


def _AssertWarmingCommitDeferred():
    """A notice schedules the commit; it must not run it (lag gate).

    The flush samples the whole neighbor+sweep band synchronously on the
    UI thread (benchCommitLag: ~136 ms on the biped), so scheduling it on
    the next event-loop turn put that freeze inside every gizmo release
    and every undo/redo. Scheduling arms a positive-delay idle timer and
    commits nothing; a second notice restarts the same timer. Deterministic
    throughout: no wall clock, only the timer shape and the call count.
    """
    from rigExecUsdview import (
        RigExecUsdviewContainer, _WARMING_COMMIT_DELAY_MS)

    if _WARMING_COMMIT_DELAY_MS <= 0:
        raise AssertionError(
            "the commit delay is not positive: the sampling burst runs "
            "inside the gesture again")

    class _CommitLibrary:
        def __init__(self):
            self.commits = 0

        def RigExecImaging_OnEditCommitted(self):
            self.commits += 1

    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._warmingCommitPending = True
    container._warmingFlushArmed = False
    container._warmingTimer = None
    container._active = True
    container._lib = _CommitLibrary()

    try:
        # Scheduling arms the idle timer and commits nothing: the release's
        # own repaint lands before the synchronous sampling burst runs.
        container._ScheduleWarmingFlush()
        if not container._warmingFlushArmed:
            raise AssertionError("schedule did not arm the flush")
        if container._lib.commits != 0:
            raise AssertionError("scheduling committed synchronously")
        timer = container._warmingTimer
        if timer is None:
            raise AssertionError("schedule armed no timer")
        if not timer.isSingleShot():
            raise AssertionError("commit timer is not single-shot")
        if timer.interval() != _WARMING_COMMIT_DELAY_MS:
            raise AssertionError("commit timer ignores the idle delay")
        if not timer.isActive():
            raise AssertionError("commit timer is not running")

        # A second notice restarts the same timer instead of arming (or
        # committing) again: a sustained stream warms once, when it stops.
        container._ScheduleWarmingFlush()
        if container._warmingTimer is not timer:
            raise AssertionError("re-schedule replaced the commit timer")
        if container._lib.commits != 0:
            raise AssertionError("re-schedule committed synchronously")

        # The flush itself still commits exactly once and disarms.
        container._FlushWarmingCommit()
        if container._lib.commits != 1:
            raise AssertionError("flush did not commit exactly once")
        if container._warmingFlushArmed:
            raise AssertionError("flush left the timer armed")
    finally:
        if container._warmingTimer is not None:
            container._warmingTimer.stop()


def _AssertWarmRangeRepushOnRangeChange():
    """A stage-range edit re-pushes the warm range on the next wake."""
    from rigExecUsdview import RigExecUsdviewContainer

    stage = Usd.Stage.CreateInMemory()
    stage.SetStartTimeCode(1.0)
    stage.SetEndTimeCode(4.0)

    class _WarmLibrary:
        def __init__(self):
            self.warmCalls = []

        def RigExecImaging_WarmRange(self, rigPath, frames, count):
            self.warmCalls.append((bytes(rigPath),
                                   [frames[i] for i in range(count)]))
            return 0

        def RigExecImaging_GetFrameStates(
                self, rigPath, frames, statesOut, count):
            for i in range(count):
                statesOut[i] = 2  # cached: the tick sleeps on states alone
            return count

    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._lib = _WarmLibrary()
    container._active = True
    container._rigPaths = [Sdf.Path("/Rig")]
    container._cachedStage = stage
    container._warmRangeFrames = [1.0, 2.0]
    container._warmingIdleTimer = None
    container._warmingIdleLastStates = None

    try:
        # A wake with a drifted range re-pushes first: the stale
        # [1, 2] becomes the stage's [1, 4], then the timer starts.
        container._WakeWarmingDriver()
        if container._lib.warmCalls != [(b"/Rig", [1.0, 2.0, 3.0, 4.0])]:
            raise AssertionError("wake did not re-push the drifted range")
        if container._warmRangeFrames != [1.0, 2.0, 3.0, 4.0]:
            raise AssertionError("re-push did not refresh the cached range")
        timer = container._warmingIdleTimer
        if timer is None or not timer.isActive():
            raise AssertionError("wake did not start the idle timer")

        # A range edit that lands mid-warm re-centers the sweep from
        # the next tick: states come back cached, so the tick sleeps
        # after re-pushing exactly once.
        stage.SetEndTimeCode(6.0)
        container._TickWarmingDriver()
        if len(container._lib.warmCalls) != 2:
            raise AssertionError("tick did not re-push the edited range")
        if container._lib.warmCalls[1] != (
                b"/Rig", [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]):
            raise AssertionError("tick pushed the wrong range: %r"
                                 % (container._lib.warmCalls[1],))
        if timer.isActive():
            raise AssertionError("all-cached tick did not sleep")

        # A wake with no drift pushes nothing further.
        container._WakeWarmingDriver()
        if len(container._lib.warmCalls) != 2:
            raise AssertionError("drift-free wake re-pushed the range")
    finally:
        if container._warmingIdleTimer is not None:
            container._warmingIdleTimer.stop()


def _AssertTimelineOverlay(appController):
    """The timeline overlay installs once and paints ticks' states."""
    from pxr.Usdviewq.qt import QtCore, QtWidgets
    from pxr.Usdviewq.usdviewApi import UsdviewApi
    from pxr.Usdviewq import frameSlider
    from rigExecUsdview import RigExecUsdviewContainer
    import cacheTimelineOverlay

    api = UsdviewApi(appController)
    first = cacheTimelineOverlay.InstallTimelineOverlay(api.qMainWindow)
    if first is None:
        raise AssertionError("the timeline overlay did not install")
    if cacheTimelineOverlay.InstallTimelineOverlay(
            api.qMainWindow) is not first:
        raise AssertionError("the overlay stacked a second band")
    parent = first.parentWidget()
    if not isinstance(parent, frameSlider.FrameSlider):
        raise AssertionError("the overlay is not on the timeline slider: %r"
                             % (parent,))
    if not first.testAttribute(QtCore.Qt.WA_TransparentForMouseEvents):
        raise AssertionError("the overlay eats the scrub clicks it covers")

    states = [2, 2, 1, 0, 3]
    frames = [1.0, 2.0, 3.0, 4.0, 5.0]
    if not first.Update(states, frames):
        raise AssertionError("the first overlay update painted nothing")
    if first.isHidden():
        raise AssertionError("the painted overlay hid itself")
    if first.Update(states, frames):
        raise AssertionError("an identical overlay update repainted")
    if first.Update([2, 2], frames):
        raise AssertionError("a skewed overlay update repainted")
    if first._states != states:
        raise AssertionError("a skewed update moved the overlay")

    # The container feeds driver ticks to the overlay with no C call of
    # its own.
    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._api = api
    container._timelineOverlay = None
    container._warmRangeFrames = frames
    container._stripPanel = None
    container._EnsureTimelineOverlay()
    if container._timelineOverlay is not first:
        raise AssertionError("the container did not adopt the overlay")
    first.Clear()
    container._NotifyStripTick(True, states)
    if first._states != states:
        raise AssertionError("the tick did not repaint the overlay")
    if first.isHidden():
        raise AssertionError("the tick left the overlay hidden")
    first.Clear()
    if not first.isHidden():
        raise AssertionError("the cleared overlay is still visible")


def _AssertFrameTickYieldsToTickingDriver():
    """A frame change skips its sync sweep while the driver ticks.

    The recurring timer sweeps the same budgeted band within 120 ms,
    so a synchronous sweep on every playback frame would serialize
    its sampling into the scrub for jobs already about to enqueue.
    A pending edit-commit never yields: it re-centers warming on the
    playhead the artist just chose.
    """
    from pxr.Usdviewq.qt import QtCore
    from rigExecUsdview import RigExecUsdviewContainer

    class _SweepLibrary:
        def __init__(self):
            self.setTimes = 0
            self.idles = 0
            self.commits = 0

        def RigExecImaging_SetTime(self, frame):
            self.setTimes += 1

        def RigExecImaging_OnIdle(self):
            self.idles += 1

        def RigExecImaging_OnEditCommitted(self):
            self.commits += 1

    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._active = True
    container._lib = _SweepLibrary()
    container._warmingCommitPending = False
    # No GetFrameStates on the double, so the wake at the end of each
    # frame tick never arms a real driver: the timer below is the only
    # ticking driver, planted by hand.
    timer = QtCore.QTimer()
    timer.setSingleShot(False)
    try:
        timer.start(120)
        container._warmingIdleTimer = timer
        container._OnFrameChanged(2.0)
        if container._lib.setTimes != 1:
            raise AssertionError("the yielding tick published nothing")
        if container._lib.idles != 0:
            raise AssertionError("the tick swept while the driver ticks")

        # A stopped driver stops yielding: the synchronous sweep is
        # the whole story again.
        timer.stop()
        container._OnFrameChanged(3.0)
        if container._lib.idles != 1:
            raise AssertionError("the tick swept nothing with no driver")

        # A pending commit runs even while the driver ticks.
        timer.start(120)
        container._warmingCommitPending = True
        container._OnFrameChanged(4.0)
        if container._lib.commits != 1:
            raise AssertionError("the commit yielded to the driver")
        if container._warmingCommitPending:
            raise AssertionError("the tick left the commit pending")
        if container._lib.idles != 1:
            raise AssertionError("the commit tick swept idle too")
    finally:
        timer.stop()


def testUsdviewInputFunction(appController):
    # The plugin owns the platform naming (.dll/.dylib/.so) and the
    # installed-vs-build search order; asking it keeps this script working on
    # every platform without repeating either rule.
    from rigExecUsdview import ImagingLibraryPath

    dll = ctypes.CDLL(ImagingLibraryPath())
    dll.RigExecImaging_GetGeneration.restype = ctypes.c_longlong

    dataModel = appController._dataModel

    generation0 = dll.RigExecImaging_GetGeneration()
    if generation0 < 1:
        raise AssertionError(
            "rigExec did not activate: generation=%d" % generation0)

    frames = [1002, 1013, 1024, 1040]
    for frame in frames:
        dataModel.currentFrame = Usd.TimeCode(frame)

    generation1 = dll.RigExecImaging_GetGeneration()
    if generation1 < generation0 + len(frames):
        raise AssertionError(
            "timeline changes did not publish: %d -> %d"
            % (generation0, generation1))

    # WHICH frame got published, not just how many publications happened.
    #
    # Counting generations is what let a one-frame-stale scrub ship:
    # RootDataModel's setter emits currentFrameChanged(value) and only
    # THEN assigns self._currentFrame (rootDataModel.py:158-160), so a
    # handler that re-read dataModel.currentFrame published the frame the
    # artist had just LEFT. Every publication still happened, on time and
    # in order, with the wrong pose in it -- and a drag hides that almost
    # perfectly because each step draws the step before it.
    #
    # RigExecImaging_SetTime(frame) is unambiguous by construction, so
    # driving the same frame both ways and comparing the POINTS is what
    # separates "published" from "published the right thing".
    observer = _Observer()

    dataModel.currentFrame = Usd.TimeCode(1001)
    viaSignalEarly = _Points(observer)
    dataModel.currentFrame = Usd.TimeCode(1024)
    viaSignal = _Points(observer)
    dll.RigExecImaging_SetTime(ctypes.c_double(1024.0))
    viaDirect = _Points(observer)

    if viaSignalEarly == viaDirect:
        raise AssertionError(
            "frames 1001 and 1024 deform identically -- this case cannot "
            "tell a stale frame from a fresh one")
    if viaSignal != viaDirect:
        raise AssertionError(
            "the timeline published the WRONG FRAME: setting "
            "currentFrame = 1024 gave %s, RigExecImaging_SetTime(1024) "
            "gives %s (use the frame the signal carries, never "
            "dataModel.currentFrame, inside currentFrameChanged)"
            % (viaSignal[0], viaDirect[0]))

    _AssertActivationFailureKeepsStageAlive()
    _AssertLiveRootAutoActivation()
    _AssertWarmingFlushOnRelease()
    _AssertWarmingCommitDeferred()
    _AssertWarmRangeRepushOnRangeChange()
    _AssertTimelineOverlay(appController)
    _AssertFrameTickYieldsToTickingDriver()

    print("RIGEXEC_USDVIEW_OK generations %d -> %d, frame 1024 published "
          "the frame 1024 pose" % (generation0, generation1))
