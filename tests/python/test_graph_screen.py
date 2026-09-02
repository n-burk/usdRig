#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/graphScreen.py.

Every case uses an 800x600 canvas with the default margins, so the plot
rectangle is (48, 24, 728, 548), and a time range of 0..10 against a
value range of 0..5.  Those numbers make the expected pixels exact: one
frame is 72.8 px, one value unit is 109.6 px, and the curve origin
(0, 0) lands on (48, 572) because pixel y grows DOWNWARD while value
grows upward.  Usage: test_graph_screen.py [ignored]
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Ts  # noqa: E402

import graphScreen as gscr  # noqa: E402

WIDTH, HEIGHT = 800, 600
PLOT = (48.0, 24.0, 728.0, 548.0)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _ClosePoint(a, b, tol=1e-6):
    return _Close(a[0], b[0], tol) and _Close(a[1], b[1], tol)


def _Transform(timeRange=(0.0, 10.0), valueRange=(0.0, 5.0)):
    transform = gscr.ViewTransform(WIDTH, HEIGHT)
    transform.timeRange = timeRange
    transform.valueRange = valueRange
    return transform


def _Spline(points, interp=None, tangents=None):
    """A double spline from [(time, value)], all knots InterpCurve."""
    interp = Ts.InterpCurve if interp is None else interp
    spline = Ts.Spline("double")
    for time, value in points:
        knot = Ts.Knot(typeName="double", time=time, value=value,
                       nextInterp=interp)
        if tangents is not None:
            slope, width = tangents
            knot.SetPreTanSlope(slope)
            knot.SetPreTanWidth(width)
            knot.SetPostTanSlope(slope)
            knot.SetPostTanWidth(width)
            knot.SetPreTanAlgorithm(Ts.TangentAlgorithmCustom)
            knot.SetPostTanAlgorithm(Ts.TangentAlgorithmCustom)
        spline.SetKnot(knot)
    return spline


def TestTransform():
    transform = _Transform()
    _Check(transform.PlotRect() == PLOT,
           "plot rect: %s" % (transform.PlotRect(),))
    # Value grows up, pixel y grows down: the curve origin is at the
    # BOTTOM left of the plot, not the top left.
    _Check(_ClosePoint(transform.ToPixel(0.0, 0.0), (48.0, 572.0)),
           "origin pixel: %s" % (transform.ToPixel(0.0, 0.0),))
    _Check(_ClosePoint(transform.ToPixel(10.0, 5.0), (776.0, 24.0)),
           "top right pixel: %s" % (transform.ToPixel(10.0, 5.0),))
    _Check(_ClosePoint(transform.ToPixel(5.0, 2.5), (412.0, 298.0)),
           "centre pixel: %s" % (transform.ToPixel(5.0, 2.5),))
    _Check(_Close(transform.TimeToX(1.0), 120.8)
           and _Close(transform.ValueToY(1.0), 462.4),
           "single axis mappings")
    for time, value in ((0.0, 0.0), (3.25, -2.5), (12.0, 7.5)):
        back = transform.FromPixel(*transform.ToPixel(time, value))
        _Check(_ClosePoint(back, (time, value), 1e-9),
               "round trip %s -> %s" % ((time, value), back))
    _Check(_Close(transform.XToTime(transform.TimeToX(4.0)), 4.0)
           and _Close(transform.YToValue(transform.ValueToY(4.0)), 4.0),
           "axis round trips")
    # A degenerate range must not divide by zero.
    flat = _Transform(timeRange=(3.0, 3.0), valueRange=(1.0, 1.0))
    pixel = flat.ToPixel(3.0, 1.0)
    _Check(pixel[0] == pixel[0] and pixel[1] == pixel[1],
           "degenerate ranges stay finite: %s" % (pixel,))


def TestFrame():
    transform = _Transform()
    transform.Frame(0.0, 10.0, -1.0, 1.0)
    _Check(_ClosePoint(transform.timeRange, (-1.0, 11.0)),
           "framed time range: %s" % (transform.timeRange,))
    _Check(_ClosePoint(transform.valueRange, (-1.2, 1.2)),
           "framed value range: %s" % (transform.valueRange,))
    transform.Frame(0.0, 10.0, -1.0, 1.0, padding=0.0)
    _Check(_ClosePoint(transform.timeRange, (0.0, 10.0)),
           "zero padding: %s" % (transform.timeRange,))
    # A single key has no extent; Maya frames a unit window round it
    # rather than collapsing the view.
    transform.Frame(5.0, 5.0, 2.0, 2.0)
    _Check(_ClosePoint(transform.timeRange, (4.0, 6.0))
           and _ClosePoint(transform.valueRange, (1.0, 3.0)),
           "degenerate frame: %s %s"
           % (transform.timeRange, transform.valueRange))


def TestPanZoom():
    transform = _Transform()
    # Panning moves the CONTENT: a curve point under the cursor travels
    # with it, so its pixel shifts by exactly the pan delta.
    before = transform.ToPixel(5.0, 2.5)
    transform.Pan(72.8, -109.6)
    after = transform.ToPixel(5.0, 2.5)
    _Check(_ClosePoint(after, (before[0] + 72.8, before[1] - 109.6), 1e-6),
           "pan moves content: %s -> %s" % (before, after))
    _Check(_ClosePoint(transform.timeRange, (-1.0, 9.0), 1e-9)
           and _ClosePoint(transform.valueRange, (-1.0, 4.0), 1e-9),
           "pan ranges: %s %s" % (transform.timeRange, transform.valueRange))

    transform = _Transform()
    anchor = (300.0, 400.0)
    curveAnchor = transform.FromPixel(*anchor)
    transform.ZoomAbout(anchor[0], anchor[1], 2.0, 2.0)
    _Check(_ClosePoint(transform.FromPixel(*anchor), curveAnchor, 1e-9),
           "zoom keeps the anchor under the cursor")
    _Check(_Close(transform.timeRange[1] - transform.timeRange[0], 5.0)
           and _Close(transform.valueRange[1] - transform.valueRange[0], 2.5),
           "a factor of 2 magnifies (halves the visible range): %s"
           % (transform.timeRange,))
    transform.ZoomAbout(anchor[0], anchor[1], 0.5, 1.0)
    _Check(_Close(transform.timeRange[1] - transform.timeRange[0], 10.0)
           and _Close(transform.valueRange[1] - transform.valueRange[0], 2.5),
           "per-axis zoom factors are independent")


def TestNiceStep():
    cases = [
        ((10.0, 728.0, 60.0), 1.0),
        ((100.0, 500.0, 50.0), 10.0),
        ((1.0, 600.0, 60.0), 0.1),
        ((3.0, 600.0, 60.0), 0.5),
        ((300.0, 600.0, 60.0), 50.0),
        ((1.2, 600.0, 60.0), 0.2),
        ((0.0, 600.0, 60.0), 1.0),
        ((10.0, 0.0, 60.0), 1.0),
    ]
    for args, expected in cases:
        got = gscr.NiceStep(*args)
        _Check(_Close(got, expected, 1e-12),
               "NiceStep%s = %s, expected %s" % (args, got, expected))
    # Every step is 1, 2 or 5 times a power of ten, whatever the range.
    for i in range(1, 200):
        step = gscr.NiceStep(i * 0.37, 640.0)
        mantissa = step
        while mantissa < 1.0 - 1e-9:
            mantissa *= 10.0
        while mantissa > 10.0 - 1e-9:
            mantissa /= 10.0
        _Check(any(_Close(mantissa, m, 1e-9) for m in (1.0, 2.0, 5.0)),
               "step %s has mantissa %s" % (step, mantissa))


def TestGridLines():
    transform = _Transform()
    timeTicks, valueTicks = gscr.GridLines(transform)
    _Check(len(timeTicks) == 11 and _Close(timeTicks[0], 0.0)
           and _Close(timeTicks[-1], 10.0),
           "time ticks: %s" % (timeTicks,))
    _Check(len(valueTicks) == 6 and _Close(valueTicks[0], 0.0)
           and _Close(valueTicks[-1], 5.0),
           "value ticks: %s" % (valueTicks,))
    # Ticks are on the step grid, not on the range ends.
    transform = _Transform(timeRange=(2.3, 12.3), valueRange=(-0.4, 4.6))
    timeTicks, valueTicks = gscr.GridLines(transform)
    _Check(all(2.3 <= t <= 12.3 for t in timeTicks),
           "time ticks inside the range: %s" % (timeTicks,))
    _Check(_Close(timeTicks[0], 3.0), "first tick snaps up to the grid: %s"
           % (timeTicks,))
    _Check(all(-0.4 <= v <= 4.6 for v in valueTicks),
           "value ticks inside the range: %s" % (valueTicks,))


def TestSampling():
    transform = _Transform()
    spline = _Spline([(0.0, 0.0), (10.0, 5.0)], tangents=(0.5, 2.0))
    polylines = gscr.SamplePolylines(spline, transform)
    _Check(len(polylines) == 1 and len(polylines[0]) >= 2,
           "one polyline: %s" % ([len(p) for p in polylines],))
    points = polylines[0]
    _Check(_ClosePoint(points[0], transform.ToPixel(0.0, 0.0), 1e-6),
           "first sample is the first knot pixel: %s" % (points[0],))
    _Check(_ClosePoint(points[-1], transform.ToPixel(10.0, 5.0), 1e-6),
           "last sample is the last knot pixel: %s" % (points[-1],))
    # Ts returns the polyline in CURVE space; the pixels are exactly the
    # transform applied to those points.  Pinning that here is what stops
    # a future reader from "fixing" the scales into the sample output.
    raw = spline.Sample(Gf.Interval(0.0, 10.0),
                        transform.PixelsPerTime(),
                        transform.PixelsPerValue(),
                        gscr.SAMPLE_TOLERANCE, withSources=False)
    expected = [[transform.ToPixel(p[0], p[1]) for p in line]
                for line in raw.polylines]
    _Check(len(expected) == len(polylines)
           and all(len(a) == len(b) for a, b in zip(expected, polylines))
           and all(_ClosePoint(a, b, 1e-9)
                   for la, lb in zip(expected, polylines)
                   for a, b in zip(la, lb)),
           "sampling equals transform-of-curve-space")
    # The scales are fed from the transform, so the same segment is
    # subdivided more finely as the curve grows on screen. The straight
    # spline above needs no subdivision at all, so this uses one whose
    # tangents actually bend it.
    curvy = _Spline([(0.0, 0.0), (10.0, 5.0)], tangents=(4.0, 3.0))
    small = gscr.ViewTransform(200, 160)
    small.timeRange, small.valueRange = (0.0, 10.0), (0.0, 5.0)
    dense = gscr.SamplePolylines(curvy, transform)
    coarse = gscr.SamplePolylines(curvy, small)
    _Check(len(dense[0]) > len(coarse[0]) > 2,
           "a larger canvas samples more finely: %d vs %d"
           % (len(dense[0]), len(coarse[0])))
    # Held segments come back as separate polylines (the step), and an
    # empty spline draws nothing at all.
    held = _Spline([(0.0, 0.0), (10.0, 5.0)], interp=Ts.InterpHeld)
    _Check(len(gscr.SamplePolylines(held, transform)) == 2,
           "a held segment is two polylines")
    _Check(gscr.SamplePolylines(Ts.Spline("double"), transform) == [],
           "an empty spline samples to nothing")
    _Check(gscr.SamplePolylines(None, transform) == [],
           "no spline samples to nothing")


def TestKeyGlyphs():
    transform = _Transform()
    curves = ["curveA", "curveB"]
    splines = [_Spline([(0.0, 0.0), (5.0, 2.5), (10.0, 5.0)]),
               _Spline([(2.0, 1.0)])]
    keys = gscr.KeyGlyphs(curves, splines, transform, {(0, 5.0)})
    _Check(len(keys) == 4, "four keys over two curves: %d" % len(keys))
    first = keys[0]
    _Check(first.curveIndex == 0 and _Close(first.time, 0.0)
           and _Close(first.value, 0.0)
           and _ClosePoint((first.x, first.y), transform.ToPixel(0.0, 0.0)),
           "first key glyph: %s" % (first,))
    selected = [k for k in keys if k.selected]
    _Check(len(selected) == 1 and selected[0].curveIndex == 0
           and _Close(selected[0].time, 5.0),
           "only the selected key is marked: %s" % (selected,))
    last = keys[-1]
    _Check(last.curveIndex == 1 and _Close(last.time, 2.0)
           and _ClosePoint((last.x, last.y), transform.ToPixel(2.0, 1.0)),
           "second curve's key: %s" % (last,))
    _Check(gscr.KeyGlyphs(curves, [None, None], transform, set()) == [],
           "curves without splines have no keys")


def TestTangentGlyphs():
    transform = _Transform()
    curves = ["curveA"]
    splines = [_Spline([(0.0, 0.0), (5.0, 2.5), (10.0, 5.0)],
                       tangents=(0.5, 2.0))]
    # The first knot has no previous segment, so only its out tangent is
    # drawn (splineViewer.py:363-380 draws the pre tangent only when the
    # previous segment was a curve).
    out = gscr.TangentGlyphs(curves, splines, transform, {(0, 0.0)})
    _Check(len(out) == 1 and out[0].side == gscr.SIDE_OUT,
           "first knot draws only the out tangent: %s" % (out,))
    _Check(_ClosePoint((out[0].x, out[0].y), transform.ToPixel(2.0, 1.0)),
           "out handle end is (t + w, v + w * slope): %s"
           % ((out[0].x, out[0].y),))
    _Check(_Close(out[0].slope, 0.5) and _Close(out[0].width, 2.0)
           and not out[0].locked,
           "out handle carries the custom slope and width: %s" % (out[0],))
    # The last knot has no following segment, so only its in tangent is
    # drawn: the handle end is (t - w, v - w * slope).
    inGlyphs = gscr.TangentGlyphs(curves, splines, transform, {(0, 10.0)})
    _Check(len(inGlyphs) == 1 and inGlyphs[0].side == gscr.SIDE_IN,
           "last knot draws only the in tangent: %s" % (inGlyphs,))
    _Check(_ClosePoint((inGlyphs[0].x, inGlyphs[0].y),
                       transform.ToPixel(8.0, 4.0)),
           "in handle end: %s" % ((inGlyphs[0].x, inGlyphs[0].y),))
    both = gscr.TangentGlyphs(curves, splines, transform, {(0, 5.0)})
    _Check(sorted(g.side for g in both) == [gscr.SIDE_IN, gscr.SIDE_OUT],
           "an interior knot draws both sides: %s" % (both,))
    _Check(all(g.curveIndex == 0 and _Close(g.time, 5.0) for g in both),
           "both sides name their key")
    # Held neighbours have no tangents to show.
    held = [_Spline([(0.0, 0.0), (5.0, 2.5), (10.0, 5.0)],
                    interp=Ts.InterpHeld, tangents=(0.5, 2.0))]
    _Check(gscr.TangentGlyphs(curves, held, transform, {(0, 5.0)}) == [],
           "held neighbours draw no tangent handles")
    # A knot whose tangents come from an algorithm is drawn locked, so
    # the panel can hollow it out (spec section 2.2).
    auto = _Spline([(0.0, 0.0), (5.0, 2.5), (10.0, 5.0)])
    for knot in auto.GetKnots().values():
        knot.SetPreTanAlgorithm(Ts.TangentAlgorithmAutoEase)
        knot.SetPostTanAlgorithm(Ts.TangentAlgorithmAutoEase)
        auto.SetKnot(knot)
    locked = gscr.TangentGlyphs(curves, [auto], transform, {(0, 5.0)})
    _Check(len(locked) == 2 and all(g.locked for g in locked),
           "algorithm-driven tangents are locked: %s" % (locked,))
    _Check(gscr.TangentGlyphs(curves, splines, transform, set()) == [],
           "no selection, no handles")


def TestHitTesting():
    transform = _Transform()
    curves = ["curveA"]
    splines = [_Spline([(0.0, 0.0), (5.0, 2.5), (10.0, 5.0)],
                       tangents=(0.5, 2.0))]
    keys = gscr.KeyGlyphs(curves, splines, transform, {(0, 5.0)})
    middle = transform.ToPixel(5.0, 2.5)
    hit = gscr.HitKey(keys, middle[0] + 3.0, middle[1] - 2.0)
    _Check(hit is not None and _Close(hit.time, 5.0),
           "a click near a key hits it: %s" % (hit,))
    _Check(gscr.HitKey(keys, middle[0] + 30.0, middle[1]) is None,
           "a click far from every key hits nothing")
    # Ties go to the nearer key, not to the first in the list.
    left = transform.ToPixel(0.0, 0.0)
    near = gscr.HitKey(keys, left[0] + 1.0, left[1] + 1.0, radius=1000.0)
    _Check(near is not None and _Close(near.time, 0.0),
           "the nearest key wins: %s" % (near,))

    tangents = gscr.TangentGlyphs(curves, splines, transform, {(0, 5.0)})
    end = transform.ToPixel(7.0, 3.5)
    tHit = gscr.HitTangent(tangents, end[0] - 2.0, end[1])
    _Check(tHit is not None and tHit.side == gscr.SIDE_OUT,
           "the out handle end is grabbable: %s" % (tHit,))
    _Check(gscr.HitTangent(tangents, end[0], end[1] + 40.0) is None,
           "a click away from a handle end hits nothing")

    # Marquee: the rectangle is (x, y, width, height), like PlotRect, and
    # a drag up-left gives negative extents that must still select.
    box = (left[0] - 5.0, middle[1] - 5.0,
           middle[0] - left[0] + 10.0, left[1] - middle[1] + 10.0)
    inside = gscr.KeysInRect(keys, box)
    _Check(sorted(k.time for k in inside) == [0.0, 5.0],
           "marquee selects the enclosed keys: %s"
           % (sorted(k.time for k in inside),))
    flipped = (box[0] + box[2], box[1] + box[3], -box[2], -box[3])
    _Check(sorted(k.time for k in gscr.KeysInRect(keys, flipped))
           == [0.0, 5.0], "a backwards marquee selects the same keys")
    _Check(gscr.KeysInRect(keys, (0.0, 0.0, 1.0, 1.0)) == [],
           "an empty marquee selects nothing")

    # Curve hit: polylines are indexed BY CURVE, so the result is the
    # curve index a double-click landed on.
    polylines = [[[(0.0, 0.0), (100.0, 0.0)]],
                 [[(0.0, 50.0), (100.0, 50.0)]]]
    _Check(gscr.HitCurve(polylines, 50.0, 2.0) == 0, "hits the first curve")
    _Check(gscr.HitCurve(polylines, 50.0, 48.0) == 1, "hits the second curve")
    _Check(gscr.HitCurve(polylines, 50.0, 25.0) is None,
           "a click between curves hits neither")
    _Check(_Close(gscr.CurveTimeAtX(transform, 412.0), 5.0),
           "CurveTimeAtX: %s" % gscr.CurveTimeAtX(transform, 412.0))


def TestKeyDrag():
    transform = _Transform()
    press = (100.0, 100.0)
    current = (172.8, 45.2)
    dt, dv = gscr.ResolveKeyDrag(transform, press, current)
    _Check(_Close(dt, 1.0, 1e-9) and _Close(dv, 0.5, 1e-9),
           "free drag: %s" % ((dt, dv),))
    dt, dv = gscr.ResolveKeyDrag(transform, press, current,
                                 constrainAxis=gscr.AXIS_TIME)
    _Check(_Close(dt, 1.0, 1e-9) and dv == 0.0,
           "time-constrained drag: %s" % ((dt, dv),))
    dt, dv = gscr.ResolveKeyDrag(transform, press, current,
                                 constrainAxis=gscr.AXIS_VALUE)
    _Check(dt == 0.0 and _Close(dv, 0.5, 1e-9),
           "value-constrained drag: %s" % ((dt, dv),))
    _Check(gscr.DominantAxis(press, current) == gscr.AXIS_TIME,
           "the longer pixel leg picks the axis")
    _Check(gscr.DominantAxis(press, (110.0, 0.0)) == gscr.AXIS_VALUE,
           "a mostly vertical drag constrains to value")


def TestTangentDrag():
    transform = _Transform()
    key = transform.ToPixel(5.0, 2.5)
    handle = transform.ToPixel(7.0, 3.5)
    slope, width = gscr.ResolveTangentDrag(transform, key, handle, True, 1.0)
    _Check(_Close(slope, 0.5, 1e-9) and _Close(width, 2.0, 1e-9),
           "weighted out drag: %s" % ((slope, width),))
    slope, width = gscr.ResolveTangentDrag(transform, key, handle, False, 1.0)
    _Check(_Close(slope, 0.5, 1e-9) and _Close(width, 1.0, 1e-9),
           "unweighted keeps the current width: %s" % ((slope, width),))
    # The in handle end is (t - w, v - w * slope), so dragging it down and
    # to the left of the key is a POSITIVE slope, matching Ts's pre-tangent.
    inHandle = transform.ToPixel(3.0, 1.5)
    slope, width = gscr.ResolveTangentDrag(transform, key, inHandle, True,
                                           1.0, side=gscr.SIDE_IN)
    _Check(_Close(slope, 0.5, 1e-9) and _Close(width, 2.0, 1e-9),
           "weighted in drag mirrors: %s" % ((slope, width),))
    # Round trip: the resolved tangent redraws its handle where the
    # cursor is.
    for side, pixel in ((gscr.SIDE_OUT, handle), (gscr.SIDE_IN, inHandle)):
        s, w = gscr.ResolveTangentDrag(transform, key, pixel, True, 1.0,
                                       side=side)
        sign = 1.0 if side == gscr.SIDE_OUT else -1.0
        back = transform.ToPixel(5.0 + sign * w, 2.5 + sign * w * s)
        _Check(_ClosePoint(back, pixel, 1e-6),
               "%s handle round trip: %s vs %s" % (side, back, pixel))
    # A handle dragged straight up has no time extent; the width floors
    # rather than dividing by zero.
    straightUp = transform.ToPixel(5.0, 4.0)
    slope, width = gscr.ResolveTangentDrag(transform, key, straightUp, True,
                                           1.0)
    _Check(_Close(width, gscr.MIN_TANGENT_TIME, 1e-12)
           and slope > 100.0, "vertical drag: %s" % ((slope, width),))
    # Dragging the out handle behind its key keeps it on the out side
    # instead of flipping the tangent into the other quadrant.
    behind = transform.ToPixel(3.0, 3.5)
    slope, width = gscr.ResolveTangentDrag(transform, key, behind, True, 1.0)
    _Check(width >= gscr.MIN_TANGENT_TIME and slope > 0.0,
           "a backwards out drag stays positive: %s" % ((slope, width),))


def main():
    groups = [
        ("transform", TestTransform),
        ("frame", TestFrame),
        ("pan and zoom", TestPanZoom),
        ("nice step", TestNiceStep),
        ("grid lines", TestGridLines),
        ("sampling", TestSampling),
        ("key glyphs", TestKeyGlyphs),
        ("tangent glyphs", TestTangentGlyphs),
        ("hit testing", TestHitTesting),
        ("key drag", TestKeyDrag),
        ("tangent drag", TestTangentDrag),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GRAPH_SCREEN_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
