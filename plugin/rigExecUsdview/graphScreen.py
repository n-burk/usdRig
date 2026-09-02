#
# RigExec graph editor: screen-space geometry for the animation curve
# canvas. Pure functions over Gf and Ts so the view transform, the key
# and tangent layout, hit-testing and the drag mappings are testable
# without Qt -- the same bargain gizmoScreen.py strikes for the viewport
# manipulator (design section 2.2-2.4).
#
# Two coordinate spaces meet here:
#
#   CURVE space is (time, value): frames along X, the attribute's value
#   along Y, growing UPWARD.
#
#   PIXEL space is the canvas widget's, in LOGICAL pixels, with y growing
#   DOWNWARD. ViewTransform is the only place the flip lives; everything
#   else goes through ToPixel / FromPixel.
#
import math

from pxr import Gf, Ts

# Plot margins as (left, top, right, bottom), in logical pixels. The
# left margin holds the value ruler's labels and the bottom margin the
# frame ruler (spec section 2.1).
DEFAULT_MARGINS = (48, 24, 24, 28)

# Glyph sizes, in logical pixels (spec section 2.2): keys are 6 px filled
# squares, tangent handle ends 5 px.
KEY_PIXELS = 6.0
TANGENT_PIXELS = 5.0

# Pick tolerances. A key and a tangent end are aimed at from their
# centre; a curve from its sampled polyline.
HIT_PIXELS = 6.0
CURVE_HIT_PIXELS = 5.0

# Maximum deviation, in pixels, between the drawn polyline and the true
# spline. Ts.Spline.Sample measures this against the time and value
# scales it is given, so feeding it the transform's pixels-per-unit is
# what makes the number mean pixels.
SAMPLE_TOLERANCE = 0.5

# Roughly how far apart grid lines should sit, in pixels, before
# NiceStep rounds the implied step to 1, 2 or 5 times a power of ten.
TARGET_TICK_PIXELS = 60.0

# Shortest tangent width a drag may produce. A handle dragged onto its
# own key has no time extent, and a zero width both divides by zero when
# the slope is recovered and leaves a handle that can never be grabbed
# again.
MIN_TANGENT_TIME = 1e-3

# Floor for a view range, so a fully collapsed range cannot divide by
# zero while the artist is mid-gesture.
_MIN_RANGE = 1e-9

SIDE_IN = "in"
SIDE_OUT = "out"

AXIS_TIME = "time"
AXIS_VALUE = "value"


class ViewTransform(object):
    """
    The mapping between curve space and canvas pixels.

    `timeRange` and `valueRange` are (min, max) pairs of floats naming
    the visible window; the plot rectangle is the widget minus the
    margins. Pixel y grows downward and value grows upward, so the
    curve-space origin sits at the BOTTOM left of the plot.
    """

    def __init__(self, width, height, margins=DEFAULT_MARGINS):
        self.width = float(width)
        self.height = float(height)
        self.margins = tuple(float(m) for m in margins)
        self.timeRange = (0.0, 24.0)
        self.valueRange = (-1.0, 1.0)

    def Resize(self, width, height):
        self.width = float(width)
        self.height = float(height)

    def PlotRect(self):
        """The plot area as (x, y, width, height), in pixels."""
        left, top, right, bottom = self.margins
        return (left, top,
                max(1.0, self.width - left - right),
                max(1.0, self.height - top - bottom))

    def TimeSpan(self):
        return max(_MIN_RANGE, self.timeRange[1] - self.timeRange[0])

    def ValueSpan(self):
        return max(_MIN_RANGE, self.valueRange[1] - self.valueRange[0])

    def PixelsPerTime(self):
        return self.PlotRect()[2] / self.TimeSpan()

    def PixelsPerValue(self):
        return self.PlotRect()[3] / self.ValueSpan()

    def TimeToX(self, time):
        x, _, w, _ = self.PlotRect()
        return x + (float(time) - self.timeRange[0]) / self.TimeSpan() * w

    def XToTime(self, x):
        px, _, w, _ = self.PlotRect()
        return self.timeRange[0] + (float(x) - px) / w * self.TimeSpan()

    def ValueToY(self, value):
        _, y, _, h = self.PlotRect()
        return y + (self.valueRange[1] - float(value)) / self.ValueSpan() * h

    def YToValue(self, y):
        _, py, _, h = self.PlotRect()
        return self.valueRange[1] - (float(y) - py) / h * self.ValueSpan()

    def ToPixel(self, time, value):
        return (self.TimeToX(time), self.ValueToY(value))

    def FromPixel(self, x, y):
        return (self.XToTime(x), self.YToValue(y))

    def Pan(self, dx, dy):
        """
        Move the CONTENT by (dx, dy) pixels, so a curve point under the
        cursor travels with it. Maya's Alt+middle drag hands the mouse
        delta straight in.
        """
        dt = float(dx) / self.PixelsPerTime()
        dv = float(dy) / self.PixelsPerValue()
        self.timeRange = (self.timeRange[0] - dt, self.timeRange[1] - dt)
        self.valueRange = (self.valueRange[0] + dv, self.valueRange[1] + dv)

    def ZoomAbout(self, x, y, factorX, factorY):
        """
        Magnify by `factorX` / `factorY` about the pixel (x, y).

        A factor above 1 zooms IN: the visible range is divided by it.
        The curve point under (x, y) does not move, which is what makes a
        wheel zoom feel anchored to the cursor (spec section 2.3).
        """
        time, value = self.FromPixel(x, y)
        fx = float(factorX) if abs(factorX) > _MIN_RANGE else 1.0
        fy = float(factorY) if abs(factorY) > _MIN_RANGE else 1.0
        self.timeRange = (time + (self.timeRange[0] - time) / fx,
                          time + (self.timeRange[1] - time) / fx)
        self.valueRange = (value + (self.valueRange[0] - value) / fy,
                           value + (self.valueRange[1] - value) / fy)

    def Frame(self, tMin, tMax, vMin, vMax, padding=0.1):
        """
        Fit the given curve-space box, padded by `padding` of its own
        extent on every side.

        A degenerate extent -- one key, or a flat curve -- is framed as a
        unit window round the value instead of collapsing the view to a
        line, which is what Maya's Frame Selected does with a single key.
        """
        self.timeRange = _PaddedRange(tMin, tMax, padding)
        self.valueRange = _PaddedRange(vMin, vMax, padding)


def _PaddedRange(low, high, padding):
    low, high = float(low), float(high)
    if high < low:
        low, high = high, low
    span = high - low
    if span <= _MIN_RANGE:
        return (low - 1.0, high + 1.0)
    pad = span * float(padding)
    return (low - pad, high + pad)


def NiceStep(rangeLength, pixels, targetPixels=TARGET_TICK_PIXELS):
    """
    A grid step of 1, 2 or 5 times a power of ten that puts ticks roughly
    `targetPixels` apart across `pixels` pixels showing `rangeLength`.

    Rounding UP to the next nice value rather than to the nearest keeps
    the labels from crowding: a step slightly too coarse is legible, a
    step slightly too fine overlaps its neighbours' text.
    """
    rangeLength = abs(float(rangeLength))
    pixels = float(pixels)
    if rangeLength <= _MIN_RANGE or pixels <= _MIN_RANGE:
        return 1.0
    raw = rangeLength * float(targetPixels) / pixels
    if raw <= _MIN_RANGE:
        return 1.0
    exponent = math.floor(math.log10(raw))
    decade = 10.0 ** exponent
    mantissa = raw / decade
    for nice in (1.0, 2.0, 5.0):
        if mantissa <= nice + 1e-9:
            return nice * decade
    return 10.0 * decade


def _Ticks(low, high, step):
    """Every multiple of `step` inside [low, high]."""
    if step <= _MIN_RANGE:
        return []
    first = math.ceil(low / step - 1e-9)
    last = math.floor(high / step + 1e-9)
    if last < first:
        return []
    # Multiply rather than accumulate: adding the step repeatedly drifts
    # visibly by the far edge of a wide view, and the labels are printed
    # from these numbers.
    return [i * step for i in range(int(first), int(last) + 1)]


def GridLines(transform):
    """
    (timeTicks, valueTicks) in CURVE space for `transform`'s window.

    The caller maps them with TimeToX / ValueToY; returning curve values
    rather than pixels is what lets the same list drive both the grid
    line and its ruler label (spec section 2.2).
    """
    _, _, w, h = transform.PlotRect()
    timeStep = NiceStep(transform.TimeSpan(), w)
    valueStep = NiceStep(transform.ValueSpan(), h)
    return (_Ticks(transform.timeRange[0], transform.timeRange[1], timeStep),
            _Ticks(transform.valueRange[0], transform.valueRange[1],
                   valueStep))


def SamplePolylines(spline, transform, interval=None):
    """
    `spline` drawn as polylines of PIXEL points, or [] when it is empty.

    Ts.Spline.Sample returns its polylines in CURVE space; the time and
    value scales only tell it how finely to subdivide, by fixing what one
    unit is worth in pixels so that `tolerance` means pixels (the working
    reference is usdviewq/splineViewer.py:186-218, which samples with the
    widget's scales and then applies its own world-to-view transform to
    the result). So the scales come from the transform and the points are
    mapped through the same transform afterwards.

    Sample returns more than one polyline where the curve is
    discontinuous -- a held segment is a step, not a line -- so the
    result stays a list of lists and each is stroked on its own.
    """
    if not spline or spline.IsEmpty():
        return []
    if interval is None:
        interval = Gf.Interval(transform.timeRange[0], transform.timeRange[1])
    samples = spline.Sample(interval,
                            transform.PixelsPerTime(),
                            transform.PixelsPerValue(),
                            SAMPLE_TOLERANCE,
                            withSources=False)
    if not samples:
        return []
    return [[transform.ToPixel(p[0], p[1]) for p in polyline]
            for polyline in samples.polylines]


class KeyGlyph(object):
    """
    One key square: its curve, its knot, and where it is on screen.

    A DUAL-VALUED knot is two squares in one column, because the curve
    jumps at that frame: `value` / `y` is the value the segment AFTER the
    knot starts from, and `preValue` / `preY` the value the segment
    before it arrives at (spec section 2.2, "Dual-valued knots draw both
    the pre-value and value squares"). Both are None on an ordinary
    knot, so the painter can draw one square without asking.
    """

    def __init__(self, curveIndex, time, value, x, y, selected,
                 preValue=None, preY=None):
        self.curveIndex = curveIndex
        self.time = time
        self.value = value
        self.x = x
        self.y = y
        self.selected = selected
        self.preValue = preValue
        self.preY = preY

    def IsDualValued(self):
        return self.preY is not None

    def __repr__(self):
        return "<KeyGlyph c%d t=%g v=%g at (%.1f, %.1f)%s%s>" % (
            self.curveIndex, self.time, self.value, self.x, self.y,
            "" if self.preY is None else " pre=%g@%.1f" % (self.preValue,
                                                           self.preY),
            " selected" if self.selected else "")


class TangentGlyph(object):
    """
    One tangent handle END, for a selected key.

    `side` is SIDE_IN or SIDE_OUT; `slope` and `width` are the knot's
    own, in curve space; `locked` says the tangent is produced by an
    algorithm rather than authored, so the panel draws it hollow and a
    drag on it switches the knot to a custom tangent (spec section 2.2).
    """

    def __init__(self, curveIndex, time, side, x, y, slope, width, locked):
        self.curveIndex = curveIndex
        self.time = time
        self.side = side
        self.x = x
        self.y = y
        self.slope = slope
        self.width = width
        self.locked = locked

    def __repr__(self):
        return "<TangentGlyph c%d t=%g %s slope=%g width=%g" \
               " at (%.1f, %.1f)%s>" % (
                   self.curveIndex, self.time, self.side, self.slope,
                   self.width, self.x, self.y,
                   " locked" if self.locked else "")


def _Knots(spline):
    """The spline's knots in time order, or [] when there are none."""
    if not spline or spline.IsEmpty():
        return []
    return list(spline.GetKnots().values())


def _IsSelected(selection, curveIndex, time):
    """
    Membership in a set of (curveIndex, time) pairs.

    The times are the knots' own floats, handed back unchanged by
    whoever built the selection from these glyphs, so an exact compare
    is safe and a tolerance would only make two adjacent keys ambiguous.
    """
    return selection is not None and (curveIndex, time) in selection


def KeyGlyphs(curves, splines, transform, selection):
    """
    A key square for every knot of every curve, in curve then time order.

    `curves` and `splines` are parallel; only the index is used here,
    because the CurveRef carries the colour and the label, which the
    painter needs and the geometry does not. `selection` is a container
    of (curveIndex, time) pairs.

    A dual-valued knot gets ONE glyph carrying BOTH squares, not two
    glyphs: it is a single knot, so selecting, dragging and deleting it
    must move both halves together.
    """
    glyphs = []
    for index in range(len(curves)):
        spline = splines[index] if index < len(splines) else None
        for knot in _Knots(spline):
            time = knot.GetTime()
            value = float(knot.GetValue())
            x, y = transform.ToPixel(time, value)
            preValue, preY = None, None
            if knot.IsDualValued():
                preValue = float(knot.GetPreValue())
                preY = transform.ValueToY(preValue)
            glyphs.append(KeyGlyph(index, time, value, x, y,
                                   _IsSelected(selection, index, time),
                                   preValue, preY))
    return glyphs


def TangentGlyphs(curves, splines, transform, selection):
    """
    The tangent handle ends of the SELECTED keys (Maya shows handles for
    the selection only, spec section 2.2).

    A side only gets a handle when the segment on that side is a curve:
    the pre tangent needs the PREVIOUS knot's interpolation to be
    Ts.InterpCurve, the post tangent this knot's own, and the last knot
    has no following segment at all. That is the rule
    usdviewq/splineViewer.py:363-397 draws by, and drawing a handle for
    a held or linear neighbour would offer the artist a control that
    changes nothing.

    The handle end is `(t + w, v + w * slope)` out and `(t - w, pv - w *
    slope)` in, so a knot's two handles point away from it along its
    tangents. Dual-valued knots hang the in handle off the PRE value.
    """
    glyphs = []
    for index in range(len(curves)):
        spline = splines[index] if index < len(splines) else None
        knots = _Knots(spline)
        for position, knot in enumerate(knots):
            time = knot.GetTime()
            if not _IsSelected(selection, index, time):
                continue
            previous = knots[position - 1] if position > 0 else None
            preCurve = (previous is not None
                        and previous.GetNextInterpolation() == Ts.InterpCurve)
            postCurve = (position < len(knots) - 1
                         and knot.GetNextInterpolation() == Ts.InterpCurve)
            if preCurve:
                slope = float(knot.GetPreTanSlope())
                width = float(knot.GetPreTanWidth())
                value = float(knot.GetPreValue())
                x, y = transform.ToPixel(time - width,
                                         value - width * slope)
                glyphs.append(TangentGlyph(
                    index, time, SIDE_IN, x, y, slope, width,
                    knot.GetPreTanAlgorithm()
                    != Ts.TangentAlgorithmCustom))
            if postCurve:
                slope = float(knot.GetPostTanSlope())
                width = float(knot.GetPostTanWidth())
                value = float(knot.GetValue())
                x, y = transform.ToPixel(time + width,
                                         value + width * slope)
                glyphs.append(TangentGlyph(
                    index, time, SIDE_OUT, x, y, slope, width,
                    knot.GetPostTanAlgorithm()
                    != Ts.TangentAlgorithmCustom))
    return glyphs


def _Nearest(glyphs, x, y, radius):
    """The glyph nearest (x, y) within `radius` pixels, or None."""
    best = None
    for glyph in glyphs:
        distance = math.hypot(x - glyph.x, y - glyph.y)
        if distance <= radius and (best is None or distance < best[0]):
            best = (distance, glyph)
    return None if best is None else best[1]


def HitKey(keys, x, y, radius=HIT_PIXELS):
    """
    The key glyph nearest (x, y) within `radius` pixels, or None.

    A dual-valued knot is aimed at by EITHER of its two squares and
    returns the same glyph, because both belong to one knot: an artist
    who grabs the pre-value square has grabbed that key.
    """
    best = None
    for glyph in keys:
        distance = math.hypot(x - glyph.x, y - glyph.y)
        if glyph.preY is not None:
            distance = min(distance, math.hypot(x - glyph.x, y - glyph.preY))
        if distance <= radius and (best is None or distance < best[0]):
            best = (distance, glyph)
    return None if best is None else best[1]


def HitTangent(tangents, x, y, radius=HIT_PIXELS):
    return _Nearest(tangents, x, y, radius)


def _Normalized(rect):
    """(x, y, width, height) with the extents made positive."""
    x, y, w, h = (float(v) for v in rect)
    if w < 0.0:
        x, w = x + w, -w
    if h < 0.0:
        y, h = y + h, -h
    return x, y, w, h


def KeysInRect(keys, rect):
    """
    The keys whose centres lie inside `rect`, an (x, y, width, height)
    tuple like PlotRect's. A marquee dragged up or to the left arrives
    with negative extents and must select the same keys, so the
    rectangle is normalised first.
    """
    x, y, w, h = _Normalized(rect)
    return [k for k in keys
            if x <= k.x <= x + w and y <= k.y <= y + h]


def DominantAxis(press, current):
    """
    AXIS_TIME or AXIS_VALUE, whichever leg of the pixel travel is
    longer. Maya picks the constraint axis this way when Shift goes down
    during a key drag (spec section 2.4).
    """
    if abs(current[0] - press[0]) >= abs(current[1] - press[1]):
        return AXIS_TIME
    return AXIS_VALUE


def ResolveKeyDrag(transform, press, current, constrainAxis=None):
    """
    The (dt, dv) a key drag from `press` to `current` asks for, in curve
    space. `constrainAxis` is AXIS_TIME, AXIS_VALUE or None.

    dv is negated because pixel y grows downward: dragging UP raises the
    value.
    """
    dt = (current[0] - press[0]) / transform.PixelsPerTime()
    dv = -(current[1] - press[1]) / transform.PixelsPerValue()
    if constrainAxis == AXIS_TIME:
        dv = 0.0
    elif constrainAxis == AXIS_VALUE:
        dt = 0.0
    return (dt, dv)


def ResolveTangentDrag(transform, keyPixel, handlePixel, weighted,
                       currentWidth, side=SIDE_OUT):
    """
    The (slope, width) a tangent drag asks for, in curve space.

    The handle vector runs from the key to the cursor. An IN handle is
    mirrored through the key first, because its end is at `(t - w, v - w
    * slope)`: dragging it left and down is Ts's POSITIVE pre-tangent
    slope, and mirroring is what makes the two sides share one sign
    convention.

    The time extent is floored at MIN_TANGENT_TIME and never allowed to
    go negative, so a handle dragged onto or behind its own key stays on
    its own side with a very steep tangent rather than flipping into the
    opposite quadrant. When `weighted` is off the drag only changes the
    angle and `currentWidth` is returned unchanged, which is what Maya's
    non-weighted tangents do (spec section 2.4).
    """
    time, value = transform.FromPixel(keyPixel[0], keyPixel[1])
    handleTime, handleValue = transform.FromPixel(handlePixel[0],
                                                  handlePixel[1])
    dt = handleTime - time
    dv = handleValue - value
    if side == SIDE_IN:
        dt, dv = -dt, -dv
    if dt < MIN_TANGENT_TIME:
        dt = MIN_TANGENT_TIME
    slope = dv / dt
    width = dt if weighted else max(MIN_TANGENT_TIME, float(currentWidth))
    return (slope, width)


def _PointSegmentDistance(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def HitCurve(polylines, x, y, radius=CURVE_HIT_PIXELS):
    """
    The index of the curve whose stroke passes within `radius` of
    (x, y), or None.

    `polylines` is indexed BY CURVE -- `polylines[i]` is the list of
    pixel polylines SamplePolylines returned for curve i -- so the
    result is the index a double-click landed on (spec section 2.4,
    "double-click on a curve inserts a key").
    """
    p = (x, y)
    best = None
    for index, curveLines in enumerate(polylines):
        for points in curveLines:
            if not points:
                continue
            if len(points) == 1:
                distance = math.hypot(p[0] - points[0][0],
                                      p[1] - points[0][1])
            else:
                distance = min(
                    _PointSegmentDistance(p, points[i], points[i + 1])
                    for i in range(len(points) - 1))
            if distance <= radius and (best is None or distance < best[0]):
                best = (distance, index)
    return None if best is None else best[1]


def CurveTimeAtX(transform, x):
    """The frame under a pixel column, for inserting a key there."""
    return transform.XToTime(x)
