#
# Rolling, Qt-free manipulation telemetry for the usdview viewport gizmo.
#
# usdview's Render HUD times StageView.paintGL().  A rig drag evaluates and
# publishes synchronously before that paint begins, so its FPS value cannot
# describe interaction latency.  This module pairs each processed drag sample
# with the first subsequent viewport paint and retains the two timings:
#
#   sample -> preview update -> repaint request
#   first paint carrying that sample -> paint completed
#
# Keeping the accumulator Qt-free makes the pairing and percentile rules
# directly testable.  gizmoUI owns the clock boundaries and the drawing.
#
import math
import time
from collections import deque


def _Percentile(values, fraction):
    """Linearly interpolated percentile, or None for an empty sequence."""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * fraction
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _Milliseconds(values, fraction):
    value = _Percentile(values, fraction)
    return None if value is None else value * 1000.0


class _Sample(object):
    def __init__(self, generation, serial, started):
        self.generation = generation
        self.serial = serial
        self.started = started
        self.previewSeconds = 0.0


class ManipulationPerformance(object):
    """
    Rolling timings for one live drag.

    A paint ticket names the newest sample available when the StageView paint
    begins.  Several samples may have arrived since the preceding paint; only
    the newest can be visible, and the older ones are counted as coalesced.
    Progressive renderer paints with no new sample are deliberately ignored.
    """

    def __init__(self, window=120, clock=None):
        self._window = max(2, int(window))
        self._clock = clock or time.perf_counter
        self._generation = 0
        self._active = False
        self._isRig = False
        self._ResetValues()

    def _Series(self):
        return deque(maxlen=self._window)

    def _ResetValues(self):
        self._serial = 0
        self._paintedSerial = 0
        self._latestStarted = None
        self._sampleCount = 0
        self._paintCount = 0
        self._coalesced = 0
        self._lastPaintFinished = None
        self._visualIntervals = self._Series()
        self._inputToPaint = self._Series()
        self._preview = self._Series()
        self._gizmoCpu = self._Series()
        self._render = self._Series()

    def Now(self):
        return self._clock()

    def BeginDrag(self, isRig):
        self._generation += 1
        self._active = True
        self._isRig = bool(isRig)
        self._ResetValues()

    def EndDrag(self):
        # Invalidate any zero-length finish callback posted by the last paint.
        self._generation += 1
        self._active = False

    def BeginSample(self, now=None):
        if not self._active:
            return None
        started = self.Now() if now is None else float(now)
        self._serial += 1
        self._sampleCount += 1
        self._latestStarted = started
        return _Sample(self._generation, self._serial, started)

    def RecordPreview(self, sample, started, finished=None):
        if sample is None or sample.generation != self._generation:
            return
        end = self.Now() if finished is None else float(finished)
        sample.previewSeconds += max(0.0, end - float(started))

    def EndSample(self, sample, now=None):
        if sample is None or sample.generation != self._generation:
            return
        finished = self.Now() if now is None else float(now)
        total = max(0.0, finished - sample.started)
        preview = min(total, max(0.0, sample.previewSeconds))
        self._preview.append(preview)
        self._gizmoCpu.append(max(0.0, total - preview))

    def BeginPaint(self):
        """Return a ticket when this paint can carry a new drag sample."""
        if not self._active or self._serial <= self._paintedSerial:
            return None
        skipped = self._serial - self._paintedSerial - 1
        self._coalesced += max(0, skipped)
        self._paintedSerial = self._serial
        # The latest sample is the one this paint can show.  Older samples in
        # the same batch were superseded and are represented by `coalesced`.
        return (self._generation, self._serial, self._latestStarted)

    def EndPaint(self, ticket, renderSeconds=0.0, now=None):
        if ticket is None or ticket[0] != self._generation or not self._active:
            return False
        finished = self.Now() if now is None else float(now)
        inputStarted = ticket[2]
        if inputStarted is not None:
            self._inputToPaint.append(max(0.0, finished - inputStarted))
        if self._lastPaintFinished is not None:
            self._visualIntervals.append(
                max(0.0, finished - self._lastPaintFinished))
        self._lastPaintFinished = finished
        render = max(0.0, float(renderSeconds or 0.0))
        if render:
            self._render.append(render)
        self._paintCount += 1
        return True

    @staticmethod
    def _Pair(values):
        return (_Milliseconds(values, 0.50),
                _Milliseconds(values, 0.95))

    def Snapshot(self):
        intervals = list(self._visualIntervals)
        elapsed = sum(intervals)
        rate = (len(intervals) / elapsed
                if intervals and elapsed > 0.0 else None)
        return {
            "active": self._active,
            "isRig": self._isRig,
            "samples": self._sampleCount,
            "paints": self._paintCount,
            "coalesced": self._coalesced,
            "interactionHz": rate,
            "visualP95Ms": _Milliseconds(intervals, 0.95),
            "inputMs": self._Pair(self._inputToPaint),
            "previewMs": self._Pair(self._preview),
            "gizmoCpuMs": self._Pair(self._gizmoCpu),
            "renderMs": self._Pair(self._render),
        }

    @staticmethod
    def _Number(value, width=5):
        return ("%*.1f" % (width, value)) if value is not None else ("-" * width)

    def Lines(self):
        """Compact text for the in-viewport overlay; empty outside a drag."""
        stats = self.Snapshot()
        if not stats["active"]:
            return ()
        hz = self._Number(stats["interactionHz"], 5)
        visual = self._Number(stats["visualP95Ms"], 5)
        lines = [
            "Interaction %s Hz | visual p95 %s ms | coalesced %d/%d" %
            (hz, visual, stats["coalesced"], stats["samples"])
        ]
        labels = [
            ("Input -> paint", stats["inputMs"]),
            ("Rig + publish" if stats["isRig"] else "Hydra xform push",
             stats["previewMs"]),
            ("Gizmo CPU", stats["gizmoCpuMs"]),
            ("Hydra paint", stats["renderMs"]),
        ]
        for label, pair in labels:
            lines.append("%-16s p50 %s | p95 %s ms" %
                         (label, self._Number(pair[0]),
                          self._Number(pair[1])))
        return tuple(lines)
