#
# A nested RigExecJoint chain must frame and draw its child-derived link in a
# fresh usdview. This covers the two host-facing contracts a synthetic
# scene-index test cannot: ancestor BBoxCache propagation and actual guide
# pixels in Storm.
#
import os

from pxr import Gf, UsdGeom
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
    model = appController._dataModel
    settings = model.viewSettings
    settings.showHUD = False

    # The RigExec usdview integration opts diagnostic guides in when it finds
    # a RigExecRoot. A stock host that does not load that UI integration must
    # make the equivalent guide-purpose choice itself.
    if not settings.displayGuide:
        raise AssertionError("RigExec activation did not enable guide purpose")

    roots = [prim for prim in model.stage.Traverse()
             if prim.GetTypeName() == "RigExecRoot"]
    if len(roots) != 1:
        raise AssertionError("expected one RigExecRoot, found %d" % len(roots))
    root = roots[0]
    if not UsdGeom.Imageable(root):
        raise AssertionError("RigExecRoot is not Imageable")
    joints = [prim for prim in model.stage.Traverse()
              if (prim.GetTypeName() == "RigExecJoint" and
                  prim.GetPath().HasPrefix(root.GetPath()))]
    if len(joints) != 2:
        raise AssertionError("expected two joints under %s, found %d" %
                             (root.GetPath(), len(joints)))
    if joints[1].GetPath().HasPrefix(joints[0].GetPath()):
        joint, child = joints
    elif joints[0].GetPath().HasPrefix(joints[1].GetPath()):
        child, joint = joints
    else:
        raise AssertionError("the two spider joints are not nested")
    sphere = joint.GetPath().AppendChild("rigGuideSphere_0")
    cone = joint.GetPath().AppendChild("rigGuideCone_0")
    childSphere = child.GetPath().AppendChild("rigGuideSphere_0")
    cache = UsdGeom.BBoxCache(
        model.currentFrame,
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide])
    worldRange = cache.ComputeWorldBound(root).ComputeAlignedRange()
    if worldRange.IsEmpty():
        raise AssertionError("fallback joint bound did not reach /World")
    if (not Gf.IsClose(
                worldRange.GetMin(),
                Gf.Vec3d(-1, 0.8908437164808554, -1), 1e-6) or
            not Gf.IsClose(
                worldRange.GetMax(),
                Gf.Vec3d(5.476721406958012, 5.545887511986089, 1),
                1e-6)):
        raise AssertionError("unexpected spider joint bound: %s" % worldRange)

    observer, sceneIndexName = _Observer()
    primType, dataSource = observer.GetPrim(sphere)
    if not dataSource:
        raise AssertionError("fallback sphere is absent from terminal index")
    # The terminal implicit-surface filter converts the synthesized sphere to
    # the mesh Storm consumes. Reaching this point proves the whole SI chain.
    if str(primType) != "mesh":
        raise AssertionError("terminal fallback sphere is %s, expected mesh"
                             % primType)
    coneType, coneDataSource = observer.GetPrim(cone)
    if not coneDataSource or str(coneType) != "mesh":
        raise AssertionError(
            "terminal child-link cone is %s, expected mesh" % coneType)
    childType, childDataSource = observer.GetPrim(childSphere)
    if not childDataSource or str(childType) != "mesh":
        raise AssertionError(
            "terminal child sphere is %s, expected mesh" % childType)

    visibleImage, visiblePixels = _Capture(appController)
    settings.displayGuide = False
    appController._processEvents()
    _, hiddenPixels = _Capture(appController)
    settings.displayGuide = True
    appController._processEvents()

    changed = sum(1 for shown, hidden in zip(visiblePixels, hiddenPixels)
                  if shown != hidden)
    fraction = float(changed) / max(len(visiblePixels), 1)
    if fraction < 0.02:
        raise AssertionError(
            "spider joint link did not reach the framebuffer: only %.3f%% of "
            "sampled pixels changed with guide purpose" % (100.0 * fraction))

    shot = os.environ.get("RIGEXEC_JOINT_SCREENSHOT")
    if shot:
        visibleImage.save(shot)
        print("RIGEXEC_JOINT_SCREENSHOT %s" % shot)

    print("RIGEXEC_SPIDER_JOINT_LINK_OK bound=%s changed=%.2f%% si=%s"
          % (worldRange, 100.0 * fraction, sceneIndexName))
