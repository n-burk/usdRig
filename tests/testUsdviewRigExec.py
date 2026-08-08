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


def testUsdviewInputFunction(appController):
    dllPath = os.environ.get(
        "RIGEXEC_IMAGING_DLL",
        r"D:\work\usdRig\usdRig\build\rigExecImaging.dll")
    dll = ctypes.CDLL(dllPath)
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

    print("RIGEXEC_USDVIEW_OK generations %d -> %d, frame 1024 published "
          "the frame 1024 pose" % (generation0, generation1))
