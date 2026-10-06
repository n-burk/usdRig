"""Verify automatic range warming in a real usdview session."""
import time

from pxr import Usd
from pxr.Usdviewq.qt import QtWidgets
from pxr.Usdviewq.usdviewApi import UsdviewApi

import cacheStripModel
import rigExecUndo
import rigExecUsdview


def testUsdviewInputFunction(appController):
    container = rigExecUsdview.ContainerFor(UsdviewApi(appController))
    assert container is not None and container._active
    frames = container._warmRangeFrames
    assert len(frames) > 1, "Use an animated stage with a frame range"
    stage = appController._dataModel.stage
    session = stage.GetSessionLayer()
    before = session.ExportToString()

    def states():
        return [state for rig in container._rigPaths
                for state in cacheStripModel.FetchFrameStates(
                    container._Imaging(), str(rig), frames)]

    def waitCached(reason):
        deadline = time.monotonic() + max(30.0, 0.3 * len(frames))
        while time.monotonic() < deadline:
            QtWidgets.QApplication.processEvents()
            current = states()
            if (not container._warmingCommitPending
                    and container._warmRangeFrames == frames
                    and current
                    and all(s == cacheStripModel.CACHED for s in current)):
                return
            time.sleep(0.02)
        raise AssertionError("%s did not warm automatically: %r"
                             % (reason, current))

    waitCached("Activation")

    # Exercise usdview's signal and Hydra time paths in both directions.
    appController._dataModel.playing = True
    try:
        for frame in frames + list(reversed(frames)):
            appController._dataModel.currentFrame = Usd.TimeCode(frame)
            appController._stageView.updateGL()
            QtWidgets.QApplication.processEvents()
    finally:
        appController._dataModel.playing = False
    assert all(s == cacheStripModel.CACHED for s in states())
    assert session.ExportToString() == before

    # Clear and warm at a held playhead, including the current frame.
    for rig in container._rigPaths:
        assert cacheStripModel.ClearFrameCache(container._Imaging(), str(rig))
    container.StripWakeDriver()
    waitCached("Cache clear")

    # Exercise the same attribute snapshots used by editor undo/redo.
    control = next((prim for prim in stage.Traverse()
                    if prim.GetTypeName() == "RigExecControl"
                    and prim.GetAttribute("avars:tx")), None)
    assert control is not None, "Use a stage containing a RigExecControl"
    attribute = control.GetAttribute("avars:tx")
    at = Usd.TimeCode(frames[0])
    originalPose = container._ReadPublishedControlFrame(stage, control.GetPath(), at)
    assert originalPose is not None
    with Usd.EditContext(stage, session):
        original = rigExecUndo.AttributeSnapshot.Capture(session, attribute.GetPath())
        attribute.Set(float(attribute.Get(at) or 0.0) + 1.0, at)
        edited = rigExecUndo.AttributeSnapshot.Capture(session, attribute.GetPath())
        waitCached("Control edit")
        changedPose = container._ReadPublishedControlFrame(stage, control.GetPath(), at)
        assert changedPose != originalPose, "Cache replay served a stale control pose"
        original.Restore()
        waitCached("Undo")
        assert container._ReadPublishedControlFrame(
            stage, control.GetPath(), at) == originalPose
        edited.Restore()
        waitCached("Redo")
        assert container._ReadPublishedControlFrame(
            stage, control.GetPath(), at) == changedPose

        # Metadata changes wake a sleeping driver without another scrub.
        stage.SetEndTimeCode(frames[-1] + 2.0)
        frames = frames + [frames[-1] + 1.0, frames[-1] + 2.0]
        waitCached("Range extension")

    session.ImportFromString(before)
    frames = cacheStripModel.FrameListForRange(
        stage.GetStartTimeCode(), stage.GetEndTimeCode())
    waitCached("Session restoration")
    assert session.ExportToString() == before
    print("RIGEXEC_FRAME_CACHE_USDVIEW_OK: %d frames, replay, clear, edit, "
          "undo, redo and range extension" % len(frames))
