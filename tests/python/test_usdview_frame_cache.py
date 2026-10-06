#!/usr/bin/env python
"""Usdview cache driver lifecycle, with real USD edits and fake imaging."""
import os
import sys
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402
from pxr.Usdviewq.qt import QtWidgets  # noqa: E402

import cacheStripModel  # noqa: E402
import cacheStripUI  # noqa: E402
from rigExecUsdview import RigExecUsdviewContainer  # noqa: E402


_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
    ["test_usdview_frame_cache"])


class _Library:
    def __init__(self):
        self.ranges = []
        self.commits = 0
        self.idles = 0
        self.states = {}
        self.stateQueries = 0

    def RigExecImaging_WarmRange(self, rig, frames, count):
        self.ranges.append((rig, [frames[i] for i in range(count)]))
        return 0

    def RigExecImaging_GetFrameStates(self, rig, frames, states, count):
        self.stateQueries += 1
        for i in range(count):
            states[i] = self.states.get(frames[i], cacheStripModel.UNCACHED)
        return count

    def RigExecImaging_OnEditCommitted(self):
        self.commits += 1
        return 0

    def RigExecImaging_OnIdle(self):
        self.idles += 1
        return 0

    def RigExecImaging_SetWeightOverlay(self, path):
        return 0

    def RigExecImaging_BeginPreview(self, paths):
        return 1

    def RigExecImaging_EndPreview(self):
        return 0


def _Container():
    stage = Usd.Stage.CreateInMemory()
    stage.SetStartTimeCode(1)
    stage.SetEndTimeCode(4)
    stage.DefinePrim("/Rig", "RigExecRoot")
    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._api = SimpleNamespace(dataModel=SimpleNamespace(
        stage=stage, currentFrame=Usd.TimeCode(1)))
    container._lib = _Library()
    container._active = True
    container._activating = False
    container._rigPaths = [Sdf.Path("/Rig")]
    container._cachedStage = stage
    container._warmingCommitPending = False
    container._warmingFlushArmed = False
    container._warmRangeFrames = []
    return container, stage


def _Stop(container):
    container._RevokeStageNotice()
    container._ResetWarmingDriver()


def TestNoticeDebouncesEveryEdit():
    container, stage = _Container()
    root = stage.GetPrimAtPath("/Rig")
    attribute = root.CreateAttribute("test:value", Sdf.ValueTypeNames.Float)
    relation = root.CreateRelationship("test:target")
    original = stage.GetRootLayer().ExportToString()
    edits = [
        lambda: attribute.Set(3.0),
        lambda: relation.SetTargets([Sdf.Path("/Target")]),
        lambda: stage.SetEndTimeCode(6),
        lambda: stage.GetRootLayer().ImportFromString(original),
    ]
    try:
        container._ObserveStage(stage)
        for index, edit in enumerate(edits):
            container._WakeWarmingDriver()
            assert container._WarmingDriverTicking()
            edit()
            assert container._warmingCommitPending
            assert not container._WarmingDriverTicking(), (
                "An edit left idle warming active inside its debounce")
            # Even an already-dispatched timer event must respect the edit.
            container._TickWarmingDriver()
            assert container._lib.idles == 0
            assert container._lib.commits == index
            container._FlushWarmingCommit()
            assert container._lib.commits == index + 1
            assert not container._warmingCommitPending
            assert container._WarmingDriverTicking()
    finally:
        _Stop(container)


def TestEmptyRangeReplacesNativeRange():
    container, stage = _Container()
    try:
        container._PushWarmRangeFromStage()
        assert container._lib.ranges[-1] == (b"/Rig", [1., 2., 3., 4.])
        stage.SetStartTimeCode(1.2)
        stage.SetEndTimeCode(1.8)
        container._RepushWarmRangeIfChanged()
        assert container._warmRangeFrames == []
        assert container._lib.ranges[-1] == (b"/Rig", []), (
            "An empty timeline retained the native rig's previous range")
    finally:
        _Stop(container)


def TestDirtyDisplayRefreshesBeforeWarming():
    container, stage = _Container()
    window = QtWidgets.QMainWindow()
    container._api.qMainWindow = window
    frames = [1., 2., 3., 4.]
    container._lib.states = dict.fromkeys(frames, cacheStripModel.CACHED)
    # A display refresh must also work with an older native library whose
    # completion count does not move when entries are retired.
    container._lib.RigExecImaging_GetWarmingCompletedCount = lambda: 4
    overlayStates = []
    container._timelineOverlay = SimpleNamespace(
        Update=lambda states, frames: overlayStates.__setitem__(slice(None), states),
        Clear=overlayStates.clear)
    attribute = stage.GetPrimAtPath("/Rig").CreateAttribute(
        "inputs:wrinkleScale", Sdf.ValueTypeNames.Float)
    panel = cacheStripUI.CacheStripPanel(container._api, container)
    container._stripPanel = panel
    try:
        container._PushWarmRangeFromStage()
        panel.show()
        panel.PollTick()
        container._UpdateTimelineOverlay([cacheStripModel.CACHED] * 4)
        _APP.processEvents()
        assert panel._model.states == [cacheStripModel.CACHED] * 4
        container._ObserveStage(stage)
        beforeQueries = container._lib.stateQueries
        for value in [0.5, 0.4, 0.3]:
            attribute.Set(value)
        assert container._cacheDisplayTimer.isActive()
        assert container._lib.stateQueries == beforeQueries, (
            "The Python notice read cache state before native retirement")
        assert panel._model.states == [cacheStripModel.CACHED] * 4
        # Model native retirement after the Python listener has returned.
        container._lib.states = dict.fromkeys(frames, cacheStripModel.DIRTY)
        container._warmingTimer.start(10000)
        authored = stage.GetRootLayer().ExportToString()
        _APP.processEvents()
        assert panel._model.states == [cacheStripModel.DIRTY] * 4
        assert overlayStates == [cacheStripModel.DIRTY] * 4
        assert not container._cacheDisplayTimer.isActive()
        assert container._warmingCommitPending
        assert not container._WarmingDriverTicking()
        assert container._lib.commits == container._lib.idles == 0
        assert stage.GetRootLayer().ExportToString() == authored
        # The independent display callback must not consume the rewarm.
        container._FlushWarmingCommit()
        assert container._lib.commits == 1
        assert container._WarmingDriverTicking()
    finally:
        panel.close()
        window.close()
        _Stop(container)


def TestReplacementCancelsPreviousTimers():
    container, stage = _Container()
    try:
        container._WakeWarmingDriver()
        container._warmingCommitPending = True
        container._ScheduleWarmingFlush()
        container._ScheduleCacheDisplayRefresh()
        assert container._warmingTimer.isActive()
        assert container._cacheDisplayTimer.isActive()
        # Reactivating an empty stage also occurs after its last rig is removed.
        stage.RemovePrim("/Rig")
        container._DeactivateImaging = lambda: None
        container._ActivateCurrentStage()
        assert not container._active
        assert not container._WarmingDriverTicking()
        assert not container._warmingTimer.isActive()
        assert not container._cacheDisplayTimer.isActive()
        assert not container._warmingCommitPending
        assert not container._warmingFlushArmed
        assert container._warmRangeFrames == []
    finally:
        _Stop(container)


def TestDeferredSamplingGetsRetried():
    container, stage = _Container()
    try:
        container._WakeWarmingDriver()
        # A temporarily declined native job has no visible in-flight state;
        # its fallback can only run after native sampling exhausts retries.
        for _ in range(4):
            container._TickWarmingDriver()
        assert container._lib.idles == 4
        assert container._WarmingDriverTicking()
        container._lib.states = {frame: cacheStripModel.CACHED
                                 for frame in [1., 2., 3., 4.]}
        container._TickWarmingDriver()
        assert not container._WarmingDriverTicking()
        # A permanently disabled cache still parks after bounded retries.
        container._lib.states.clear()
        container._WakeWarmingDriver()
        for _ in range(5):
            container._TickWarmingDriver()
        assert not container._WarmingDriverTicking()
        assert container._lib.idles == 8
    finally:
        _Stop(container)


def TestNativeAttemptProgressKeepsDriverAwake():
    container, stage = _Container()
    try:
        container._lib.RigExecImaging_GetWarmingProgressCount = (
            lambda: container._lib.idles)
        container._WakeWarmingDriver()
        for _ in range(10):
            container._TickWarmingDriver()
        assert container._lib.idles == 10
        assert container._WarmingDriverTicking(), (
            "Failed early frames prevented later valid frames from warming")
    finally:
        _Stop(container)


def TestStripFollowsRangeAndRootEdits():
    container, stage = _Container()
    window = QtWidgets.QMainWindow()
    container._api.qMainWindow = window
    panel = cacheStripUI.CacheStripPanel(container._api, container)
    container._stripPanel = panel
    try:
        panel.show()
        stage.SetEndTimeCode(6)
        panel.PollTick()
        assert panel._model.frames == [1., 2., 3., 4., 5., 6.]
        # A click before a driver tick must still send the latest range.
        stage.SetEndTimeCode(2)
        panel.WarmRange()
        assert container._lib.ranges[-1] == (b"/Rig", [1., 2.])
        stage.DefinePrim("/OtherRig", "RigExecRoot")
        container._rigPaths = [Sdf.Path("/OtherRig"), Sdf.Path("/Rig")]
        panel.PollTick()
        assert panel._rootBox.count() == 2
        assert panel._model.rigPath == "/Rig"
        stage.RemovePrim("/Rig")
        stage.RemovePrim("/OtherRig")
        container._DeactivateImaging = lambda: None
        container._ActivateCurrentStage()
        assert panel._model.rigPath == ""
        assert panel._model.states is None
    finally:
        panel.close()
        window.close()
        _Stop(container)


def TestPlaybackPausesAndResumesIdle():
    container, stage = _Container()
    try:
        container._WakeWarmingDriver()
        container._api.dataModel.playing = True
        for _ in range(6):
            container._TickWarmingDriver()
        assert container._lib.idles == 0
        assert container._WarmingDriverTicking()
        container._api.dataModel.playing = False
        container._TickWarmingDriver()
        assert container._lib.idles == 1
    finally:
        _Stop(container)


def TestNonAuthoringActionsWakeCache():
    container, stage = _Container()
    try:
        container._PushWarmRangeFromStage()
        assert container._SetWeightOverlay("")
        assert container._WarmingDriverTicking()
        preview = container._PreviewSink()
        assert preview.Begin("/Rig.avars:test") == 1
        assert not container._WarmingDriverTicking()
        container._WakeWarmingDriver()
        assert not container._WarmingDriverTicking()
        assert preview.End()
        assert container._WarmingDriverTicking()
        container._ObserveStage(stage)
        assert preview.Begin("/Rig.avars:test") == 1
        stage.GetPrimAtPath("/Rig").SetDocumentation("Edited during preview")
        assert container._warmingCommitPending
        container._FlushWarmingCommit()
        assert container._lib.commits == 0
        assert preview.End()
        assert container._warmingFlushArmed
        container._FlushWarmingCommit()
        assert container._lib.commits == 1
        assert container._WarmingDriverTicking()
        # Clear in an older native library discarded its configured range.
        container._lib.ranges.clear()
        container.StripWakeDriver()
        assert container._lib.ranges[-1] == (b"/Rig", [1., 2., 3., 4.])
    finally:
        _Stop(container)


def main():
    tests = [TestNoticeDebouncesEveryEdit, TestEmptyRangeReplacesNativeRange,
             TestDirtyDisplayRefreshesBeforeWarming,
             TestReplacementCancelsPreviousTimers,
             TestDeferredSamplingGetsRetried,
             TestNativeAttemptProgressKeepsDriverAwake,
             TestStripFollowsRangeAndRootEdits,
             TestPlaybackPausesAndResumesIdle,
             TestNonAuthoringActionsWakeCache]
    for test in tests:
        test()
        print("  ok: %s" % test.__name__)
    print("USDVIEW_FRAME_CACHE_DRIVER_OK (%d groups)" % len(tests))
    return 0


if __name__ == "__main__":
    sys.exit(main())
