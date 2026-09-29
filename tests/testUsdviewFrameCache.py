"""Verify automatic range warming in a real usdview session."""
import time

from pxr import Usd
from pxr.Usdviewq.qt import QtWidgets
from pxr.Usdviewq.usdviewApi import UsdviewApi

import cacheStripModel
import rigExecUsdview


def testUsdviewInputFunction(appController):
    container = rigExecUsdview.ContainerFor(UsdviewApi(appController))
    assert container is not None and container._active
    frames = container._warmRangeFrames
    assert len(frames) > 1, "Use an animated stage with a frame range"
    stage = appController._dataModel.stage
    before = stage.GetSessionLayer().ExportToString()

    def states():
        return [state for rig in container._rigPaths
                for state in cacheStripModel.FetchFrameStates(
                    container._Imaging(), str(rig), frames)]

    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        QtWidgets.QApplication.processEvents()
        current = states()
        if current and all(s == cacheStripModel.CACHED for s in current):
            break
        time.sleep(0.02)
    else:
        raise AssertionError("Range did not warm automatically: %r" % current)

    # Exercise usdview's signal and Hydra time paths in both directions.
    for frame in frames + list(reversed(frames)):
        appController._dataModel.currentFrame = Usd.TimeCode(frame)
        appController._stageView.updateGL()
        QtWidgets.QApplication.processEvents()
    assert all(s == cacheStripModel.CACHED for s in states())
    assert stage.GetSessionLayer().ExportToString() == before
    print("RIGEXEC_FRAME_CACHE_USDVIEW_OK: %d frames warmed and replayed" % len(frames))
