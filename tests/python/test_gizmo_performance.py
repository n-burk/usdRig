#!/usr/bin/env python
"""Headless checks for drag-sample/viewport-paint telemetry pairing."""
import pathlib
import sys


_PLUGIN = pathlib.Path(__file__).resolve().parents[2] / "plugin" / "rigExecUsdview"
sys.path.insert(0, str(_PLUGIN))

import gizmoPerformance  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Near(actual, expected, tolerance=1e-6):
    return actual is not None and abs(actual - expected) <= tolerance


def TestPairsNewestSampleAndCountsCoalescing():
    stats = gizmoPerformance.ManipulationPerformance()
    stats.BeginDrag(True)

    first = stats.BeginSample(0.000)
    stats.RecordPreview(first, 0.002, 0.008)
    stats.EndSample(first, 0.010)
    second = stats.BeginSample(0.012)
    stats.RecordPreview(second, 0.014, 0.024)
    stats.EndSample(second, 0.027)

    ticket = stats.BeginPaint()
    _Check(ticket is not None, "a new sample arms one viewport paint")
    stats.EndPaint(ticket, 0.008, 0.038)
    snapshot = stats.Snapshot()
    _Check(snapshot["coalesced"] == 1,
           "the superseded first sample was counted")
    _Check(snapshot["paints"] == 1, "one new-sample paint was counted")
    _Check(_Near(snapshot["inputMs"][0], 26.0),
           "latency starts at the newest input: %r" %
           (snapshot["inputMs"],))
    _Check(stats.BeginPaint() is None,
           "a progressive paint cannot inflate interaction FPS")


def TestReportsRollingLatencyComponentsAndVisualRate():
    stats = gizmoPerformance.ManipulationPerformance()
    stats.BeginDrag(True)

    first = stats.BeginSample(1.000)
    stats.RecordPreview(first, 1.002, 1.008)  # 6 ms preview
    stats.EndSample(first, 1.011)             # 5 ms other gizmo CPU
    paint = stats.BeginPaint()
    stats.EndPaint(paint, 0.007, 1.019)

    second = stats.BeginSample(1.030)
    stats.RecordPreview(second, 1.032, 1.042) # 10 ms preview
    stats.EndSample(second, 1.048)            # 8 ms other gizmo CPU
    paint = stats.BeginPaint()
    stats.EndPaint(paint, 0.009, 1.059)

    snapshot = stats.Snapshot()
    _Check(_Near(snapshot["interactionHz"], 25.0),
           "40 ms between visible updates is 25 Hz: %r" % snapshot)
    _Check(_Near(snapshot["visualP95Ms"], 40.0),
           "visual frame interval is reported in ms")
    _Check(_Near(snapshot["previewMs"][0], 8.0) and
           _Near(snapshot["previewMs"][1], 9.8),
           "preview p50/p95 preserve the slow tail: %r" %
           (snapshot["previewMs"],))
    _Check(_Near(snapshot["gizmoCpuMs"][0], 6.5) and
           _Near(snapshot["gizmoCpuMs"][1], 7.85),
           "gizmo CPU excludes preview time: %r" %
           (snapshot["gizmoCpuMs"],))
    _Check(_Near(snapshot["renderMs"][0], 8.0) and
           _Near(snapshot["renderMs"][1], 8.9),
           "Hydra paint comes from usdview's completed paint timer")
    lines = stats.Lines()
    _Check(any("Rig + publish" in line for line in lines),
           "rig drags name the expensive preview lane")


def TestEndHidesAndInvalidatesPostedPaint():
    stats = gizmoPerformance.ManipulationPerformance()
    stats.BeginDrag(False)
    sample = stats.BeginSample(2.0)
    stats.EndSample(sample, 2.001)
    ticket = stats.BeginPaint()
    stats.EndDrag()
    _Check(not stats.EndPaint(ticket, 0.001, 2.003),
           "a callback posted by an ended drag is ignored")
    _Check(stats.Lines() == (), "the overlay is hidden outside a drag")


def main():
    for name, fn in (
            ("paint pairing", TestPairsNewestSampleAndCountsCoalescing),
            ("rolling metrics", TestReportsRollingLatencyComponentsAndVisualRate),
            ("end invalidates", TestEndHidesAndInvalidatesPostedPaint)):
        fn()
        print("  ok: %s" % name)
    print("GIZMO_PERFORMANCE_OK (3 groups)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
