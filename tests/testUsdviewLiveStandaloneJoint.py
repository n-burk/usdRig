#
# A joint authored into usdview's already-open blank stage must activate
# RigExec and draw immediately, without a reopen or manual Reactivate.
#
import ctypes
import os

from pxr import Gf, Sdf, UsdGeom
from pxr.Usdviewq.qt import QtWidgets


def _Observer():
    from pxr.Usdviewq._usdviewq import HydraObserver

    names = HydraObserver.GetRegisteredSceneIndexNames()
    if not names:
        raise AssertionError("no registered scene indices")
    observer = HydraObserver()
    observer.TargetToNamedSceneIndex(names[-1])
    return observer, names[-1]


def _Capture(appController):
    appController._stageView.updateGL()
    QtWidgets.QApplication.processEvents()
    image = appController._stageView.grabFrameBuffer()
    pixels = [image.pixelColor(x, y).rgba()
              for y in range(0, image.height(), 2)
              for x in range(0, image.width(), 2)]
    return image, pixels


def testUsdviewInputFunction(appController):
    from rigExecUsdview import ImagingLibraryPath

    dll = ctypes.CDLL(ImagingLibraryPath())
    dll.RigExecImaging_GetGeneration.restype = ctypes.c_longlong

    model = appController._dataModel
    stage = model.stage
    model.viewSettings.showHUD = False
    if any(prim.GetTypeName() == "RigExecRoot" for prim in stage.Traverse()):
        raise AssertionError("live-authoring fixture is not blank")
    generation0 = dll.RigExecImaging_GetGeneration()

    root = stage.DefinePrim("/RigExecRoot1", "RigExecRoot")
    stage.DefinePrim("/RigExecRoot1/Joints", "Scope")
    if dll.RigExecImaging_GetGeneration() != generation0:
        raise AssertionError("empty intermediate rig activated too early")

    joint = stage.DefinePrim(
        "/RigExecRoot1/Joints/RigExecJoint1", "RigExecJoint")
    appController._processEvents()
    generation1 = dll.RigExecImaging_GetGeneration()
    if generation1 <= generation0:
        raise AssertionError(
            "live-authored joint did not activate RigExec: %d -> %d" %
            (generation0, generation1))
    if not model.viewSettings.displayGuide:
        raise AssertionError("live activation did not enable guide purpose")

    cache = UsdGeom.BBoxCache(
        model.currentFrame,
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide])
    worldRange = cache.ComputeWorldBound(root).ComputeAlignedRange()
    if (worldRange.IsEmpty() or
            not Gf.IsClose(worldRange.GetMin(), Gf.Vec3d(-1), 1e-6) or
            not Gf.IsClose(worldRange.GetMax(), Gf.Vec3d(1), 1e-6)):
        raise AssertionError("unexpected live joint bound: %s" % worldRange)

    sphere = joint.GetPath().AppendChild("rigGuideSphere_0")
    observer, sceneIndexName = _Observer()
    primType, dataSource = observer.GetPrim(Sdf.Path(sphere))
    if not dataSource or str(primType) != "mesh":
        raise AssertionError(
            "live fallback sphere is absent from terminal index: %s" %
            primType)

    api = appController._usdviewApi
    api.ClearPrimSelection()
    api.AddPrimToSelection(joint)
    appController._frameSelection()
    appController._processEvents()

    visibleImage, visiblePixels = _Capture(appController)
    model.viewSettings.displayGuide = False
    appController._processEvents()
    _, hiddenPixels = _Capture(appController)
    model.viewSettings.displayGuide = True
    appController._processEvents()

    changed = sum(1 for shown, hidden in zip(visiblePixels, hiddenPixels)
                  if shown != hidden)
    fraction = float(changed) / max(len(visiblePixels), 1)
    if fraction < 0.02:
        raise AssertionError(
            "live joint did not reach framebuffer: %.3f%% changed" %
            (100.0 * fraction))

    shot = os.environ.get("RIGEXEC_LIVE_JOINT_SCREENSHOT")
    if shot:
        visibleImage.save(shot)
        print("RIGEXEC_LIVE_JOINT_SCREENSHOT %s" % shot)

    print("RIGEXEC_LIVE_STANDALONE_JOINT_OK generation=%d->%d "
          "bound=%s changed=%.2f%% si=%s" %
          (generation0, generation1, worldRange,
           100.0 * fraction, sceneIndexName))
