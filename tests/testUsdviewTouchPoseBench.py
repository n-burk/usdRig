#
# TOUCHPOSE LATENCY BENCHMARK, in a real usdview.
#
# Not a pass/fail test: it prints the numbers the TouchPose rewrite is
# judged by, measured the same way for any implementation of the
# controller. It touches only the controller surface both the overlay-mesh
# version and the Storm-shader version share -- `Hover`, `RegionAt`,
# `SyncSelection`, `SyncPose`, `SetActive`, `model.regions` -- plus real
# Qt events, so a before/after pair is the same script run on two builds.
#
# WHAT IS MEASURED, and why each is split in two:
#
#   * CPU  -- the call itself returning (the event handler an animator's
#     mouse move runs);
#   * FRAME -- the call plus ONE forced synchronous redraw of the viewport
#     (`grabFrameBuffer`), which is where Hydra pays for whatever the call
#     changed: a stage edit resyncs rprims here, a primvar dirty uploads a
#     buffer here. The idle frame (nothing changed) is measured too and
#     reported, so the highlight's own share is the difference.
#
#     click and select-ext pump the Qt event loop before the grab, because
#     usdview's selection handlers run there; that pump can paint a frame
#     of its own, so their FRAME column is an upper bound (for either
#     implementation alike) and their CPU column is the one to compare.
#
#   hover-same   the cursor moves inside one region (nothing should change)
#   hover-cross  the cursor alternates between two regions (the highlight
#                moves every sample)
#   click        a real press+release through the event filter, selecting
#                a region's control, then the redraw
#   select-ext   the selection is changed from OUTSIDE TouchPose (as the
#                Control Picker or outliner would) and the highlight follows
#   pose         a rig control is moved with TouchPose on and a region
#                selected, and the viewport redraws (playback / drag cost)
#
# Run:  bin/run_testusdview_touchpose_bench.sh [stage]
# Prints one line per measurement and RIGEXEC_TOUCHPOSE_BENCH at the end.
#
import os
import statistics
import sys
import time

from pxr import Sdf, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

HIPS = "/Biped/Rig/Controls/hips_ctl"
REPEATS = int(os.environ.get("TOUCHPOSE_BENCH_REPEATS", "30"))

_PLUGIN = os.environ.get("TOUCHPOSE_PLUGIN_DIR")
if _PLUGIN and _PLUGIN not in sys.path:
    sys.path.insert(0, _PLUGIN)


def _Frame(view):
    """A synchronous redraw: the frame the change actually lands in."""
    view.grabFrameBuffer()


def _Stats(samples):
    samples = sorted(samples)
    return (statistics.median(samples), samples[int(0.9 * (len(samples) - 1))],
            statistics.mean(samples))


def _Report(name, cpu, frame, loop=None):
    c = _Stats(cpu)
    f = _Stats(frame)
    l = _Stats(loop) if loop else None
    print("TOUCHPOSE_BENCH %-12s cpu median %7.3f ms p90 %7.3f | "
          "cpu+frame median %7.3f ms p90 %7.3f%s"
          % (name, c[0], c[1], f[0], f[1],
             " | cpu+eventloop median %7.3f ms p90 %7.3f" % (l[0], l[1])
             if l else ""))
    return c[0], f[0]


def _HoverLoop(appController, view, controller, spots):
    """The same samples twice: once with one synchronous frame after the
    call, once with the Qt event loop pumped instead -- which is where
    usdview reacts to a stage edit and where a scheduled repaint lands, so
    it is the end-to-end number an animator feels."""
    cpu, frame, loop = [], [], []
    for i in range(REPEATS):
        spot = spots[i % len(spots)]
        t = time.perf_counter()
        controller.Hover(spot[0], spot[1])
        c = time.perf_counter()
        _Frame(view)
        e = time.perf_counter()
        cpu.append(1000.0 * (c - t))
        frame.append(1000.0 * (e - t))
    for i in range(REPEATS):
        spot = spots[i % len(spots)]
        t = time.perf_counter()
        controller.Hover(spot[0], spot[1])
        appController._processEvents()
        loop.append(1000.0 * (time.perf_counter() - t))
    return cpu, frame, loop


def _Click(view, x, y, ratio):
    local = QtCore.QPointF(x / ratio, y / ratio)
    globalPos = QtCore.QPointF(view.mapToGlobal(local.toPoint()))
    for kind in (QtCore.QEvent.MouseButtonPress,
                 QtCore.QEvent.MouseButtonRelease):
        event = QtGui.QMouseEvent(
            kind, local, globalPos, QtCore.Qt.LeftButton,
            QtCore.Qt.LeftButton if kind == QtCore.QEvent.MouseButtonPress
            else QtCore.Qt.NoButton, QtCore.Qt.NoModifier)
        QtWidgets.QApplication.sendEvent(view, event)


def testUsdviewInputFunction(appController):
    import touchPoseUI

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    view = appController._stageView
    dataModel = appController._dataModel
    dataModel.viewSettings.showHUD = False
    selection = dataModel.selection

    panel = touchPoseUI.OpenTouchPosePanel(api)
    controller = panel.controller
    model = controller.model
    assert model is not None, "touch regions loaded"
    t = time.perf_counter()
    controller.SetActive(True)
    activateMs = 1000.0 * (time.perf_counter() - t)
    appController._processEvents()
    t = time.perf_counter()
    _Frame(view)
    firstFrameMs = 1000.0 * (time.perf_counter() - t)
    print("TOUCHPOSE_BENCH activate %.2f ms, first frame after %.2f ms"
          % (activateMs, firstFrameMs))

    try:
        ratio = float(view.devicePixelRatioF())
    except AttributeError:
        ratio = 1.0
    viewport = view.computeWindowViewport()
    width, height = int(viewport[2]), int(viewport[3])

    # The screen, scanned once: which region is under which pixel.
    step = max(8, width // 48)
    hits = {}
    t = time.perf_counter()
    scanned = 0
    for sy in range(step // 2, height, step):
        for sx in range(step // 2, width, step):
            region, face = controller.RegionAt(float(sx), float(sy))
            scanned += 1
            if region is not None:
                hits.setdefault(region.index, []).append((float(sx), float(sy)))
    scanMs = 1000.0 * (time.perf_counter() - t)
    print("TOUCHPOSE_BENCH pick scan %d rays in %.1f ms (%.4f ms/ray)"
          % (scanned, scanMs, scanMs / max(scanned, 1)))
    ranked = sorted(hits, key=lambda i: -len(hits[i]))
    assert len(ranked) >= 2, "two regions visible"
    a, b = ranked[0], ranked[1]
    pa = hits[a][len(hits[a]) // 2]
    pb = hits[b][len(hits[b]) // 2]
    inside = hits[a][:8]

    # Idle frame: nothing changed at all.
    for _ in range(5):
        _Frame(view)
    idle = []
    for _ in range(REPEATS):
        t = time.perf_counter()
        _Frame(view)
        idle.append(1000.0 * (time.perf_counter() - t))
    idleMedian = _Stats(idle)[0]
    print("TOUCHPOSE_BENCH idle-frame   median %7.3f ms" % idleMedian)

    results = {}

    # hover inside one region
    controller.Hover(*inside[0])
    appController._processEvents()
    _Frame(view)
    results["hover-same"] = _Report(
        "hover-same", *_HoverLoop(appController, view, controller, inside))

    # hover crossing between two regions
    results["hover-cross"] = _Report(
        "hover-cross", *_HoverLoop(appController, view, controller, [pa, pb]))
    controller.Hover(1.0, 1.0)

    # click, through the event filter, alternating two regions
    cpu, frame = [], []
    for i in range(REPEATS):
        spot = pa if i % 2 else pb
        t = time.perf_counter()
        _Click(view, spot[0], spot[1], ratio)
        appController._processEvents()
        c = time.perf_counter()
        _Frame(view)
        e = time.perf_counter()
        cpu.append(1000.0 * (c - t))
        frame.append(1000.0 * (e - t))
    results["click"] = _Report("click", cpu, frame)
    want = model.regions[a].control
    got = [str(p.GetPath()) for p in selection.getPrims()]
    assert want in got or model.regions[b].control in got, \
        "the click selected a region's control: %s" % got

    # selection changed from outside TouchPose
    prims = [stage.GetPrimAtPath(Sdf.Path(model.regions[i].control))
             for i in (a, b)]
    cpu, frame = [], []
    for i in range(REPEATS):
        t = time.perf_counter()
        selection.setPrim(prims[i % 2])
        appController._processEvents()
        c = time.perf_counter()
        _Frame(view)
        e = time.perf_counter()
        cpu.append(1000.0 * (c - t))
        frame.append(1000.0 * (e - t))
    results["select-ext"] = _Report("select-ext", cpu, frame)

    # pose change with a region lit: what playback / a drag pays per frame
    hips = stage.GetPrimAtPath(Sdf.Path(HIPS))
    if hips and hips.IsValid():
        attr = hips.GetAttribute("avars:ty")
        if not attr or not attr.IsValid():
            attr = hips.CreateAttribute("avars:ty", Sdf.ValueTypeNames.Double)
        base = float(attr.Get() or 0.0)
        selection.setPrim(prims[0])
        appController._processEvents()
        _Frame(view)

        def _PoseLoop(label):
            cpu, frame = [], []
            for i in range(REPEATS):
                attr.Set(base + (6.0 if i % 2 else 0.0))
                t = time.perf_counter()
                appController._processEvents()
                controller.SyncPose()
                c = time.perf_counter()
                _Frame(view)
                e = time.perf_counter()
                cpu.append(1000.0 * (c - t))
                frame.append(1000.0 * (e - t))
            return _Report(label, cpu, frame)

        results["pose-on"] = _PoseLoop("pose-on")
        controller.SetActive(False)
        appController._processEvents()
        _Frame(view)
        results["pose-off"] = _PoseLoop("pose-off")
        attr.Set(base)
        appController._processEvents()

    # The same external selection change with TouchPose OFF: usdview's own
    # selection handling, so click / select-ext can be attributed.
    controller.SetActive(False)
    appController._processEvents()
    cpu, frame = [], []
    for i in range(REPEATS):
        t = time.perf_counter()
        selection.setPrim(prims[i % 2])
        appController._processEvents()
        c = time.perf_counter()
        _Frame(view)
        e = time.perf_counter()
        cpu.append(1000.0 * (c - t))
        frame.append(1000.0 * (e - t))
    results["select-off"] = _Report("select-off", cpu, frame)

    print("RIGEXEC_TOUCHPOSE_BENCH idle %.3f %s" % (
        idleMedian, " ".join("%s=%.3f/%.3f" % (k, v[0], v[1])
                             for k, v in sorted(results.items()))))
