#
# testusdview verification for the live RigExec Hydra integration:
# asserts the usdview plugin activated evaluation (an initial generation
# was published) and that timeline changes drive per-frame publications
# through the exact signal path the interactive app uses.
#
import ctypes
import os

from pxr import Usd


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

    print("RIGEXEC_USDVIEW_OK generations %d -> %d"
          % (generation0, generation1))
