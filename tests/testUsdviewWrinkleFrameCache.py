"""Wrinkle parameter edits retire, display, rewarm and replay cached frames.

Run with bin/test/run_testusdview_wrinkle_framecache on the documented
wrinkle_mover example. All edits stay in the session layer and are restored.
"""
import time

from pxr import Sdf, Usd
from pxr.Usdviewq._usdviewq import HydraObserver
from pxr.Usdviewq.qt import QtWidgets
from pxr.Usdviewq.usdviewApi import UsdviewApi

import cacheStripModel
import rigexec
import rigExecUndo
import rigExecUsdview
import volumeWeightUI


_RIG = "/Rig"
_MOVER = "/Rig/Movers/Detail"
_MESH = "/Rig/Cloth"
_EDITS = [
    ("enabled", False),
    ("defaultWeight", 0.4),
    ("iterations", 35),
    ("neighborDistance", 3),
    ("restLengthScale", 0.96),
    ("stretchStiffness", 0.75),
    ("compressionStiffness", 0.5),
    ("bendStiffness", 0.2),
    ("maxDisplacement", 0.05),
    ("pinBorders", False),
    ("tangentPlaneCollisions", False),
    ("tangentPlaneInset", 0.01),
    ("wrinkleScale", 0.25),
    ("smoothingIterations", 4),
    ("topology", "surfaceStruts"),
]


def _Points(observer):
    _, source = observer.GetPrim(Sdf.Path(_MESH))
    primvars = source.Get("primvars") if source else None
    entry = primvars.Get("points") if primvars else None
    value = entry.Get("primvarValue") if entry else None
    assert value, "No published points for %s" % _MESH
    return list(value.GetValue(0.0))


def _AssertPoints(actual, expected, reason):
    assert len(actual) == len(expected), reason
    error = max((abs(float(a) - float(b))
                 for pa, pb in zip(actual, expected)
                 for a, b in zip(pa, pb)), default=0.0)
    assert error <= 1e-5, "%s: maximum coordinate error %g" % (reason, error)


def testUsdviewInputFunction(appController):
    api = UsdviewApi(appController)
    container = rigExecUsdview.ContainerFor(api)
    assert container is not None and container._active
    stage = appController._dataModel.stage
    mover = stage.GetPrimAtPath(_MOVER)
    assert mover and mover.GetTypeName() == "RigExecWrinkleMover"
    session = stage.GetSessionLayer()
    before = session.ExportToString()
    frames = [22., 23., 24., 25.]
    at = Usd.TimeCode(24)
    panel = container._OpenCacheStripPanel(api)

    def states():
        return cacheStripModel.FetchFrameStates(
            container._Imaging(), _RIG, frames)

    def waitCached(reason):
        deadline = time.monotonic() + 30.0
        current = None
        while time.monotonic() < deadline:
            QtWidgets.QApplication.processEvents()
            current = states()
            if (not container._warmingCommitPending
                    and container._warmRangeFrames == frames
                    and current == [cacheStripModel.CACHED] * len(frames)
                    and panel._model.states == current
                    and container._timelineOverlay._states == current):
                return
            time.sleep(0.01)
        raise AssertionError("%s did not rewarm automatically: %r"
                             % (reason, current))

    def assertDirtyDisplay(reason, structural=False):
        current = states()
        invalidated = ({cacheStripModel.DIRTY, cacheStripModel.UNCACHED}
                       if structural else {cacheStripModel.DIRTY})
        assert any(s in invalidated for s in current), (
            "%s left the native cache clean: %r" % (reason, current))
        assert container._warmingCommitPending, reason
        # Hold the normal debounce long enough to observe its display state
        # deterministically; the test never invokes warming directly.
        container._warmingTimer.start(10000)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            QtWidgets.QApplication.processEvents()
            if (panel._model.states == current
                    and container._timelineOverlay._states == current):
                break
            time.sleep(0.005)
        assert panel._model.states == current, (
            "%s left the Cache Strip green during the edit" % reason)
        assert container._timelineOverlay._states == current, (
            "%s left the timeline cache band green during the edit" % reason)
        assert container._warmingCommitPending, reason
        assert not container._WarmingDriverTicking(), reason
        container._warmingTimer.start(rigExecUsdview._WARMING_COMMIT_DELAY_MS)

    try:
        with Usd.EditContext(stage, session):
            stage.SetStartTimeCode(frames[0])
            stage.SetEndTimeCode(frames[-1])
            appController._dataModel.currentFrame = at
            waitCached("Initial range")
            observer = HydraObserver()
            names = HydraObserver.GetRegisteredSceneIndexNames()
            assert names, "No registered terminal scene index"
            observer.TargetToNamedSceneIndex(names[-1])
            # A second stage over the same layers is accepted by the Python
            # evaluator, whose live pulls do not use the imaging frame cache.
            twin = Usd.Stage.Open(stage.GetRootLayer(), session)
            live = rigexec.Rig(twin, _RIG)
            live.compile()
            originalPoints = _Points(observer)

            def verifyReplay(reason):
                appController._dataModel.playing = True
                try:
                    for frame in [22., 25., 23., 24.]:
                        appController._dataModel.currentFrame = Usd.TimeCode(frame)
                        appController._stageView.updateGL()
                        pose = live.evaluate(frame)
                        assert pose.valid, "%s: live evaluation failed" % reason
                        _AssertPoints(_Points(observer),
                                      pose.moved_property(_MESH + ".points"),
                                      "%s at frame %g" % (reason, frame))
                finally:
                    appController._dataModel.playing = False

            for name, value in _EDITS:
                attribute = mover.GetAttribute("inputs:" + name)
                assert attribute, "Missing input %s" % name
                snapshot = rigExecUndo.AttributeSnapshot.Capture(
                    session, attribute.GetPath())
                volumeWeightUI.SetAtTime(attribute, value, at)
                assertDirtyDisplay(name, structural=(name == "topology"))
                waitCached(name)
                verifyReplay(name)
                if name == "wrinkleScale":
                    assert _Points(observer) != originalPoints, (
                        "Wrinkle scale edit did not change the published mesh")
                snapshot.Restore()
                assertDirtyDisplay("Undo " + name,
                                   structural=(name == "topology"))
                waitCached("Undo " + name)
                verifyReplay("Undo " + name)
                _AssertPoints(_Points(observer), originalPoints, "Undo " + name)
                print("  ok: Wrinkle %s dirtied, repainted, rewarmed and replayed"
                      % name)
    finally:
        panel.close()
        session.ImportFromString(before)
    assert session.ExportToString() == before
    print("RIGEXEC_WRINKLE_FRAME_CACHE_USDVIEW_OK: %d parameters, dirty display, "
          "automatic rewarm, replay and undo" % len(_EDITS))
