#
# testusdview verification that the influence overlay reaches a REAL
# usdview viewport.
#
# Everything else that covers the overlay drives a synthetic
# HdRetainedSceneIndex upstream, which proves the filter publishes a
# displayColor but not that the app's own chain carries it to a renderer.
# This closes that gap the only way it can be closed: turn the overlay on
# inside usdview, render, and look at the pixels.
#
# Run with:
#   run_testusdview_overlay.bat
#
import ctypes
import os

from pxr import Usd, Sdf
from pxr.Usdviewq.qt import QtWidgets


VOLUME = "/VolumeAsset/Rig/Joints/Shoulder/ShoulderVolume"
MESH = "/VolumeAsset/Geom/Strip"


def _LoadDll():
    dllPath = os.environ.get(
        "RIGEXEC_IMAGING_DLL",
        r"D:\work\usdRig\usdRig\build\rigExecImaging.dll")
    dll = ctypes.CDLL(dllPath)
    dll.RigExecImaging_GetGeneration.restype = ctypes.c_longlong
    dll.RigExecImaging_SetWeightOverlay.argtypes = [ctypes.c_char_p]
    dll.RigExecImaging_SetWeightOverlay.restype = ctypes.c_int
    return dll


def _Observer():
    """A HydraObserver targeted at the app's TERMINAL scene index.

    Going through usdview's own registered chain rather than building one
    is the entire point of this test: a filter that publishes correctly
    into a synthetic harness and then gets dropped by the real chain
    would pass every other test in the suite.

    This is the same API the Hydra Scene Browser uses
    (Usdviewq/hydraSceneDebugger.py), so "what this reads" and "what the
    artist sees in that window" are the same thing.
    """
    from pxr.Usdviewq._usdviewq import HydraObserver

    names = HydraObserver.GetRegisteredSceneIndexNames()
    if not names:
        raise AssertionError("no registered scene indices")
    observer = HydraObserver()
    # The last registration is the terminal one.
    observer.TargetToNamedSceneIndex(names[-1])
    return observer, names[-1]


def _TerminalPrimvar(observer, primPath, name):
    """Reads one primvar off the targeted scene index, or None."""
    primType, dataSource = observer.GetPrim(Sdf.Path(primPath))
    if not dataSource:
        return None
    if "primvars" not in dataSource.GetNames():
        return None
    primvars = dataSource.Get("primvars")
    if not primvars or name not in primvars.GetNames():
        return None
    entry = primvars.Get(name)
    if not entry:
        return None
    names = entry.GetNames()
    value = entry.Get("primvarValue") if "primvarValue" in names else None
    interp = entry.Get("interpolation") if "interpolation" in names else None
    return (value.GetValue(0.0) if value else None,
            interp.GetValue(0.0) if interp else None)


def testUsdviewInputFunction(appController):
    dll = _LoadDll()

    if dll.RigExecImaging_GetGeneration() < 1:
        raise AssertionError("rigExec did not activate")

    # Move off the rest frame so the field is not uniformly zero.
    appController._dataModel.currentFrame = Usd.TimeCode(1024)

    observer, sceneIndexName = _Observer()

    # --- overlay OFF: the mesh must carry no displayColor of ours ---
    dll.RigExecImaging_SetWeightOverlay(b"")
    before = _TerminalPrimvar(observer, MESH, "displayColor")
    if before is not None and before[0] is not None:
        # An authored displayColor would be legitimate; this example has
        # none, so anything here is ours and should not be.
        raise AssertionError(
            "displayColor present with the overlay off: %r" % (before,))

    # --- overlay ON ---
    status = dll.RigExecImaging_SetWeightOverlay(VOLUME.encode("utf-8"))
    if status != 0:
        raise AssertionError(
            "RigExecImaging_SetWeightOverlay failed: %d" % status)

    after = _TerminalPrimvar(observer, MESH, "displayColor")
    if after is None or after[0] is None:
        raise AssertionError(
            "no displayColor reached the terminal scene index")
    colors, interpolation = after
    if str(interpolation) != "vertex":
        raise AssertionError(
            "displayColor interpolation is %r, expected vertex"
            % (interpolation,))

    stage = appController._dataModel.stage
    points = stage.GetAttributeAtPath(MESH + ".points").Get(1024)
    if len(colors) != len(points):
        raise AssertionError(
            "displayColor is %d long, points are %d"
            % (len(colors), len(points)))

    # The field must be a GRADIENT, not a flat wash: the shoulder volume
    # sits at the base of the strip, so the near end must be redder than
    # the far end. A constant colour would satisfy every check above and
    # still mean the weights never arrived.
    def redness(c):
        return float(c[0]) - 0.5 * (float(c[1]) + float(c[2]))

    near = max(redness(c) for c in colors)
    far = min(redness(c) for c in colors)
    if not (near > far + 0.1):
        raise AssertionError(
            "displayColor is not a gradient: near=%.3f far=%.3f"
            % (near, far))

    # --- and it must go away again ---
    dll.RigExecImaging_SetWeightOverlay(b"")
    cleared = _TerminalPrimvar(observer, MESH, "displayColor")
    if cleared is not None and cleared[0] is not None:
        raise AssertionError("displayColor survived turning the overlay off")

    # --- and now the part that actually matters: PIXELS ---
    #
    # Everything above passed while the mesh rendered grey. A newly
    # appearing primvar is a RESYNC, not a dirty --
    # HdSceneIndexAdapterSceneDelegate rebuilds an rprim's primvar
    # DESCRIPTORS from PrimsAdded, so a correct red displayColor sat one
    # hop upstream and Storm never asked for it. Scene-index assertions
    # cannot see that; only the framebuffer can.
    sv = appController._stageView

    def redPixels():
        sv.updateGL()
        QtWidgets.QApplication.processEvents()
        img = sv.grabFrameBuffer()
        count = 0
        for y in range(0, img.height(), 2):
            for x in range(0, img.width(), 2):
                c = img.pixelColor(x, y)
                if c.red() - (c.green() + c.blue()) / 2.0 > 12:
                    count += 1
        return count

    dll.RigExecImaging_SetWeightOverlay(b"")
    greyPixels = redPixels()
    dll.RigExecImaging_SetWeightOverlay(VOLUME.encode("utf-8"))
    litPixels = redPixels()

    # The example draws a yellow control guide, which is red-ish enough to
    # register, so this compares against the overlay-off baseline rather
    # than against zero. The gradient covers a large fraction of the
    # strip, so the jump is not marginal.
    #
    # Mutation-verified: with the resync in NotifyGenerationPublished
    # disabled, this reads 1009 -> 997 and fails, while every scene-index
    # assertion above still passes. That asymmetry is the whole reason
    # this block exists.
    if litPixels < greyPixels * 2:
        raise AssertionError(
            "the overlay did not reach the renderer: red-ish pixels "
            "%d -> %d (a resync, not a dirty, is what makes a NEW primvar "
            "visible to Storm)" % (greyPixels, litPixels))

    # Optional visual artefact. Set RIGEXEC_OVERLAY_SCREENSHOT to a path
    # to write what the viewport actually drew -- the assertions above
    # prove the DATA is right, and this is for a human to confirm it
    # LOOKS right, which no assertion covers.
    shot = os.environ.get("RIGEXEC_OVERLAY_SCREENSHOT")
    if shot:
        appController._stageView.grabFrameBuffer().save(shot)
        print("RIGEXEC_OVERLAY_SCREENSHOT %s" % shot)

    print("RIGEXEC_OVERLAY_OK n=%d near=%.3f far=%.3f interp=%s si=%s"
          % (len(colors), near, far, interpolation, sceneIndexName))
