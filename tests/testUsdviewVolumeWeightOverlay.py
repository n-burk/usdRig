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
CURVE_VOLUME = "/VolumeAsset/Rig/Weights/TipTube"
CURVE_CHILD = CURVE_VOLUME + "/rigGuideVol_0"

# How much redder a pixel must get before it counts as changed by the overlay,
# and how much of the frame must change. See the pixel comparison at the end of
# testUsdviewInputFunction for how both were calibrated.
_REDDER_BY = 20.0
_MIN_CHANGED_FRACTION = 0.005
_MIN_CURVE_CHANGED_FRACTION = 0.001


def _Redness(color):
    """How red a pixel is relative to its other channels.

    The overlay tints the strip red over a grey mesh, so the red channel alone
    would also count anything merely bright.
    """
    return color.red() - 0.5 * (color.green() + color.blue())


def _LoadDll():
    # The plugin owns the platform naming (.dll/.dylib/.so) and the
    # installed-vs-build search order; asking it keeps this script working on
    # every platform without repeating either rule.
    from rigExecUsdview import ImagingLibraryPath

    dll = ctypes.CDLL(ImagingLibraryPath())
    dll.RigExecImaging_GetGeneration.restype = ctypes.c_longlong
    dll.RigExecImaging_SetTime.argtypes = [ctypes.c_double]
    dll.RigExecImaging_SetTime.restype = ctypes.c_int
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


def _CheckCurveGeometry(appController, dll, observer, sceneIndexName):
    """Proves CurveWeight geometry mode reaches both the terminal SI and Storm."""
    model = appController._dataModel
    stage = model.stage
    drawMode = stage.GetPrimAtPath(CURVE_VOLUME).GetAttribute("guide:drawMode")
    if not drawMode:
        raise AssertionError("CurveWeight has no guide:drawMode")

    def setMode(value):
        # Session-only: the checked-in example remains the ordinary wire case.
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            drawMode.Set(value)
        if dll.RigExecImaging_SetTime(1024.0) != 0:
            raise AssertionError("RigExecImaging_SetTime failed")
        appController._processEvents()
        QtWidgets.QApplication.processEvents()

    def capture():
        appController._stageView.updateGL()
        QtWidgets.QApplication.processEvents()
        image = appController._stageView.grabFrameBuffer()
        return [image.pixelColor(x, y).rgba()
                for y in range(0, image.height(), 2)
                for x in range(0, image.width(), 2)]

    # `none` gives a same-camera baseline containing every other guide and the
    # asset. Geometry must add exactly this CurveWeight's solid tube.
    setMode("none")
    _, hiddenSource = observer.GetPrim(Sdf.Path(CURVE_CHILD))
    if hiddenSource:
        raise AssertionError("CurveWeight child survived drawMode=none")
    hidden = capture()

    setMode("geometry")
    primType, source = observer.GetPrim(Sdf.Path(CURVE_CHILD))
    if not source:
        raise AssertionError("CurveWeight geometry child is absent")
    if str(primType) != "mesh":
        raise AssertionError(
            "CurveWeight geometry is %s, expected mesh" % primType)
    names = source.GetNames()
    if "mesh" not in names or "primvars" not in names:
        raise AssertionError(
            "CurveWeight terminal mesh is incomplete: %s" % names)
    primvars = source.Get("primvars")
    if "normals" not in primvars.GetNames():
        raise AssertionError("CurveWeight terminal mesh has no normals")
    normals = primvars.Get("normals")
    interpolation = normals.Get("interpolation").GetValue(0.0)
    if str(interpolation) != "faceVarying":
        raise AssertionError(
            "CurveWeight normals are %s, expected faceVarying"
            % interpolation)

    visible = capture()
    changed = sum(1 for shown, hiddenPixel in zip(visible, hidden)
                  if shown != hiddenPixel)
    fraction = float(changed) / max(len(visible), 1)
    if fraction < _MIN_CURVE_CHANGED_FRACTION:
        raise AssertionError(
            "solid CurveWeight did not reach Storm: only %.3f%% of sampled "
            "pixels changed" % (100.0 * fraction))

    # The overlay comparison below expects guide geometry to remain identical
    # between its two captures, so restore the example's authored wire mode.
    setMode("wire")
    print("RIGEXEC_CURVE_GEOMETRY_OK changed=%.3f%% si=%s"
          % (100.0 * fraction, sceneIndexName))


def testUsdviewInputFunction(appController):
    dll = _LoadDll()

    if dll.RigExecImaging_GetGeneration() < 1:
        raise AssertionError("rigExec did not activate")

    # Move off the rest frame so the field is not uniformly zero.
    appController._dataModel.currentFrame = Usd.TimeCode(1024)

    observer, sceneIndexName = _Observer()

    # Before testing the color overlay, exercise CurveWeight's distinct solid
    # draw path in the real usdview scene-index chain and framebuffer.
    appController._dataModel.viewSettings.showHUD = False
    _CheckCurveGeometry(appController, dll, observer, sceneIndexName)

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

    # The HUD reports frame timings, so its text differs between two captures
    # of an otherwise identical scene. Turning it off leaves only geometry.
    appController._dataModel.viewSettings.showHUD = False

    def capture():
        sv.updateGL()
        QtWidgets.QApplication.processEvents()
        img = sv.grabFrameBuffer()
        return [[_Redness(img.pixelColor(x, y))
                 for x in range(0, img.width(), 2)]
                for y in range(0, img.height(), 2)]

    dll.RigExecImaging_SetWeightOverlay(b"")
    grey = capture()
    dll.RigExecImaging_SetWeightOverlay(VOLUME.encode("utf-8"))
    lit = capture()

    # Compare the two frames pixel for pixel rather than counting red in each.
    #
    # An absolute count cannot work here: RigExec draws its own guide geometry
    # -- the volume rings and bars are red, and they are far larger on screen
    # than the strip -- so most of the red in either frame was never the
    # overlay's. That baseline also moves whenever guide drawing changes,
    # which is exactly how the earlier `lit > grey * 2` threshold silently
    # stopped being satisfiable.
    #
    # The guides are identical in both captures, so differencing cancels them
    # and leaves only what the overlay changed. Measured as a FRACTION of the
    # frame so the result does not depend on window size or on whether the
    # display is HiDPI.
    sampled = sum(len(row) for row in grey)
    redder = sum(1
                 for rowLit, rowGrey in zip(lit, grey)
                 for a, b in zip(rowLit, rowGrey)
                 if a - b > _REDDER_BY)
    fraction = float(redder) / max(sampled, 1)

    # Signal and noise are three orders of magnitude apart: the strip turning
    # red moves ~2.8% of the frame, while an unchanged scene moves ~0.03%
    # (antialiasing on the guide edges). 0.5% sits well clear of both.
    #
    # Mutation-verified: with the resync in NotifyGenerationPublished
    # disabled, nothing the overlay produces reaches the framebuffer, this
    # falls to the antialiasing floor and fails, while every scene-index
    # assertion above still passes. That asymmetry is the whole reason this
    # block exists.
    if fraction < _MIN_CHANGED_FRACTION:
        raise AssertionError(
            "the overlay did not reach the renderer: only %d of %d sampled "
            "pixels (%.3f%%) got redder when it was turned on, expected at "
            "least %.1f%% (a resync, not a dirty, is what makes a NEW "
            "primvar visible to Storm)"
            % (redder, sampled, 100.0 * fraction,
               100.0 * _MIN_CHANGED_FRACTION))

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
