#
# RigExec usdview plugin: the Maya-style Graph Editor -- a window over
# usdview holding a curve list, a QPainter canvas of the selected
# attributes' Ts splines, and the gestures that edit them.
#
# Layering, and why: everything decidable without Qt already was.
# graphModel.py owns "which attributes are curves" and every Ts.Spline
# edit (MoveKeys, InsertKey, SetTangentType, ...); graphScreen.py owns
# "where is that key on screen and what did the mouse mean"
# (ViewTransform, glyphs, hit-testing, drag resolution). This file is
# the shell: widgets, painting, event plumbing and the undo bracket.
#
# PIXELS. graphScreen works in LOGICAL pixels -- the units Qt mouse
# events and QPainter both use -- so, unlike gizmoUI._Position, nothing
# here multiplies by devicePixelRatioF. The ViewTransform is built from
# the canvas's logical width()/height() and KeyPixels() / TangentPixels()
# hand back logical pixels a test can put straight into a QMouseEvent.
#
# ONE GESTURE, ONE UNDO STEP (spec section 2.4). Every edit -- a drag, a
# button, a typed number -- goes through _Gesture: rigExecUndo
# .EditRecorder.Begin() over every attribute the gesture can touch,
# attr.SetSpline(edited) on each mouse move so the viewport follows the
# drag live, and Commit() + undoStack.Push() once at the end (or
# Abort() on Escape). The spline written each move is recomputed from
# the spline the gesture STARTED with, never from the previous move, so
# a slow drag cannot accumulate rounding or re-clamp its own clamp.
#
# HOTKEYS AND usdview. Two of the editor's keys are claimed
# application-wide by usdview itself, whichever window is active:
#   Escape -- appEventFilter.py:117 swallows every Escape KeyPress to
#             reset focus from the mouse position.
#   F      -- appEventFilter.py routes bare F (KeyboardShortcuts
#             .FramingKey) to appController.processNavKeyEvent, which
#             frames the 3D VIEWPORT (appController.py:5311).
# So the editor reads its keys from an application-level filter of its
# own (GraphHotkeyFilter), installed when the window opens and therefore
# ahead of usdview's -- Qt runs the most recently installed application
# filter first. The filter claims a key only for events belonging to the
# editor's own visible window and only when focus is not in a text field
# or spin box. usdview's other bare-letter shortcuts (I, V, J, W, C) are
# QActions on the MAIN window with Qt's default WindowShortcut context,
# so they are inert while this window is the active one; I is still
# taken by the filter for symmetry with the Insert Key button.
#
import math
import os
import sys

from pxr import Tf, Ts, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    # PySide6 keeps QAction / QActionGroup in QtGui; usdview's qt shim
    # re-exports whichever module has them as QtActionWidgets.
    from pxr.Usdviewq.qt import QtActionWidgets
except ImportError:                                       # PySide2
    QtActionWidgets = QtWidgets

try:
    import graphModel
    import graphScreen
    import rigExecUndo
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import graphModel
    import graphScreen
    import rigExecUndo


SIDE_IN = graphModel.SIDE_IN
SIDE_OUT = graphModel.SIDE_OUT
SIDE_BOTH = graphModel.SIDE_BOTH

# Maya's graph editor palette, dark enough that a saturated curve colour
# carries on its own (spec section 2.2).
COLOR_BACKGROUND = QtGui.QColor(43, 43, 43)
COLOR_RULER = QtGui.QColor(56, 56, 56)
COLOR_RULER_EDGE = QtGui.QColor(28, 28, 28)
COLOR_GRID = QtGui.QColor(62, 62, 62)
COLOR_AXIS = QtGui.QColor(96, 96, 96)
COLOR_TEXT = QtGui.QColor(178, 178, 178)
COLOR_SELECTED = QtGui.QColor(255, 214, 51)
COLOR_HOVER = QtGui.QColor(255, 246, 196)
COLOR_TANGENT = QtGui.QColor(196, 196, 196)
COLOR_PLAYHEAD = QtGui.QColor(90, 190, 255)
COLOR_MARQUEE = QtGui.QColor(210, 210, 210)
# Painted OVER the plot outside usdview's start/end frames, so the shot
# range reads without hiding the curve that leaves it.
COLOR_OUTSIDE = QtGui.QColor(0, 0, 0, 70)

CURVE_WIDTH = 1.6
GRID_WIDTH = 1.0
PLAYHEAD_WIDTH = 1.4

# One wheel notch is 120 eighths of a degree; this is the magnification
# per notch, matching the pace of usdview's own viewport dolly.
WHEEL_ZOOM = 1.15
# Pixels of Alt+right drag per doubling of the visible range.
ZOOM_DRAG_PIXELS = 220.0

# How far a press may travel and still count as a click rather than a
# marquee, in logical pixels.
CLICK_SLOP = 3.0

# Maya's Infinity menu, in menu order, as (label, graphModel name).
INFINITY_CHOICES = (
    ("Constant", "constant"),
    ("Linear", "linear"),
    ("Cycle", "cycle"),
    ("Cycle w/ Offset", "cycle_offset"),
    ("Oscillate", "oscillate"),
)

TANGENT_BUTTONS = (
    ("Auto", graphModel.TANGENT_AUTO,
     "Auto tangents (Ts AutoEase) on the selected keys."),
    ("Spline", graphModel.TANGENT_SPLINE,
     "Catmull-Rom slope through the neighbouring keys."),
    ("Linear", graphModel.TANGENT_LINEAR,
     "Straight segment after the key (before it, for In)."),
    ("Flat", graphModel.TANGENT_FLAT,
     "Zero slope, so the curve levels off at the key."),
    ("Step", graphModel.TANGENT_STEP,
     "Held segment after the key (before it, for In)."),
)

# Focus widgets that own every key they are given, so a bare A or F
# typed into a spin box never reaches the canvas.
_TEXT_WIDGETS = (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                 QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit)


def _Color(rgb, alpha=255):
    """A graphModel 0..1 curve colour as a QColor."""
    return QtGui.QColor(int(round(rgb[0] * 255)), int(round(rgb[1] * 255)),
                        int(round(rgb[2] * 255)), alpha)


def _Position(event):
    """
    The cursor in the canvas's LOGICAL pixels.

    No devicePixelRatioF here, unlike gizmoUI._Position: gizmoScreen
    works in the physical pixels computeWindowViewport reports, while
    graphScreen is handed the widget's logical size and QPainter draws
    in logical pixels too. Multiplying would put every pick at twice the
    cursor's position on a HiDPI display.
    """
    try:
        point = event.position()
        return (float(point.x()), float(point.y()))
    except AttributeError:                                # Qt 5
        return (float(event.x()), float(event.y()))


def _IsFinite(value):
    return not (math.isnan(value) or math.isinf(value))


def _FormatTick(value, step):
    """A grid label: whole numbers while the step is whole."""
    if step >= 1.0:
        return "%d" % int(round(value))
    decimals = max(0, int(math.ceil(-math.log10(step))) + 1)
    return "%.*f" % (decimals, value)


# ---------------------------------------------------------------------------
# Small widgets
# ---------------------------------------------------------------------------

class BlankableSpinBox(QtWidgets.QDoubleSpinBox):
    """
    A double spin box that can show NOTHING.

    Maya's Time / Value key stats are blank when the selected keys
    disagree, and a plain QDoubleSpinBox always shows a number -- which
    would read as "every selected key is at frame 1001" when in fact
    they are spread over the shot. Blankness is a display state only:
    the box keeps its last value so the arrows still work, and
    IsBlank() is what the panel checks before applying a typed number.
    """

    def __init__(self, parent=None):
        super(BlankableSpinBox, self).__init__(parent)
        self._blank = False
        self.setKeyboardTracking(False)
        self.lineEdit().textEdited.connect(self._onTextEdited)

    def _onTextEdited(self, text):
        del text
        self._blank = False

    def textFromValue(self, value):
        # Not just an empty line edit: QAbstractSpinBox re-renders its
        # text from the value on a show, a style change and an
        # interpretText, so a box blanked once would silently come back
        # reading 0.000 with nothing selected.
        if self._blank:
            return ""
        return super(BlankableSpinBox, self).textFromValue(value)

    def SetBlank(self):
        self._blank = True
        self.lineEdit().setText("")

    def SetNumber(self, value):
        self._blank = False
        value = float(value)
        self.setValue(value)
        # setValue does not refresh the line edit when the value is
        # unchanged, and the box may currently be showing blank.
        self.lineEdit().setText(self.textFromValue(value))

    def IsBlank(self):
        return self._blank or not self.lineEdit().text().strip()


class CurveListWidget(QtWidgets.QTreeWidget):
    """
    Maya's outliner column: one row per curve, with the colour it draws
    in, the prim it belongs to, the attribute, and a visibility check.

    Selecting rows ISOLATES those curves (Maya's behaviour): the canvas
    then draws only the selection, and `Show All` puts everything back.
    """

    def __init__(self, panel):
        super(CurveListWidget, self).__init__(panel)
        self._panel = panel
        self.setColumnCount(2)
        self.setHeaderLabels(["Curve", "Attribute"])
        self.setRootIsDecorated(False)
        self.setUniformRowHeights(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.setAlternatingRowColors(True)
        # usdview's dark palette draws the native check indicator almost
        # invisibly against the row, and the visibility tick is the one
        # control in this column: give it an outline of its own, the way
        # ViewportToolbar styles its checked tool buttons.
        self.setStyleSheet(
            "QTreeWidget::indicator { width: 12px; height: 12px;"
            " border: 1px solid #6e6e6e; border-radius: 2px; }"
            " QTreeWidget::indicator:unchecked { background: #202020; }"
            " QTreeWidget::indicator:checked { background: #5da0f0;"
            " border: 1px solid #9ccbff; }")
        self.setToolTip(
            "The animation curves in the editor. Tick to show or hide a "
            "curve; select rows to isolate them.")


# ---------------------------------------------------------------------------
# Canvas
# ---------------------------------------------------------------------------

class GraphCanvas(QtWidgets.QWidget):
    """
    The plot: grid, rulers, curves, keys, tangent handles, playhead --
    and every mouse gesture that edits them.

    Geometry is graphScreen's; this class only paints what graphScreen
    computes and turns Qt events into the panel's edit calls, so the
    hit-testing and the drag mappings stay headlessly testable.
    """

    def __init__(self, panel):
        super(GraphCanvas, self).__init__(panel)
        self._panel = panel
        self.transform = graphScreen.ViewTransform(600, 400)
        self.setMinimumSize(320, 200)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setMouseTracking(True)
        self.setAutoFillBackground(False)
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)

        self._hover = None          # (curveIndex, time) under the cursor
        self._marquee = None        # (x, y, w, h) while a marquee drags
        self._mode = None           # pan / zoom / scrub / keys / tangent
        self._press = None
        self._last = None
        self._pressRanges = None

    # -- geometry -------------------------------------------------------

    def resizeEvent(self, event):
        super(GraphCanvas, self).resizeEvent(event)
        self.transform.Resize(self.width(), self.height())

    def PlotRectF(self):
        x, y, w, h = self.transform.PlotRect()
        return QtCore.QRectF(x, y, w, h)

    def _Splines(self):
        """The spline per curve, None where the curve is hidden."""
        return self._panel.DrawSplines()

    def _Glyphs(self):
        """(key glyphs, tangent glyphs) for the visible curves."""
        curves = self._panel.Curves()
        splines = self._Splines()
        selection = self._panel.SelectionPairs()
        return (graphScreen.KeyGlyphs(curves, splines, self.transform,
                                      selection),
                graphScreen.TangentGlyphs(curves, splines, self.transform,
                                          selection))

    def _Polylines(self):
        """Pixel polylines per curve, indexed by curve; [] when hidden."""
        result = []
        for spline in self._Splines():
            if spline is None:
                result.append([])
            else:
                result.append(graphScreen.SamplePolylines(spline,
                                                          self.transform))
        return result

    def KeyPixels(self):
        """
        {curve label: [(x, y, time)]} in LOGICAL pixels.

        One entry per KNOT, at the value square; a dual-valued knot's
        pre-value square is drawn as well but is the same key, so it
        would only make the map ambiguous.
        """
        curves = self._panel.Curves()
        result = {}
        for glyph in self._Glyphs()[0]:
            label = curves[glyph.curveIndex].Label()
            result.setdefault(label, []).append(
                (glyph.x, glyph.y, glyph.time))
        return result

    def TangentPixels(self):
        """{curve label: [(x, y, time, side)]} in LOGICAL pixels."""
        curves = self._panel.Curves()
        result = {}
        for glyph in self._Glyphs()[1]:
            label = curves[glyph.curveIndex].Label()
            result.setdefault(label, []).append(
                (glyph.x, glyph.y, glyph.time, glyph.side))
        return result

    # -- framing --------------------------------------------------------

    def FrameAll(self):
        self._Frame(self._panel.KeyExtents(selectedOnly=False))

    def FrameSelected(self):
        extents = self._panel.KeyExtents(selectedOnly=True)
        if extents is None:
            extents = self._panel.KeyExtents(selectedOnly=False)
        self._Frame(extents)

    def FrameStageRange(self):
        """Home: back to usdview's own frame range, values framed."""
        start, end = self._panel.StageRange()
        extents = self._panel.KeyExtents(selectedOnly=False)
        if start is None or end is None:
            self._Frame(extents)
            return
        vMin, vMax = (extents[2], extents[3]) if extents else (-1.0, 1.0)
        self.transform.Frame(start, end, vMin, vMax)
        self.update()

    def _Frame(self, extents):
        if extents is None:
            start, end = self._panel.StageRange()
            extents = (start if start is not None else 0.0,
                       end if end is not None else 24.0, -1.0, 1.0)
        self.transform.Frame(*extents)
        self.update()

    # -- painting -------------------------------------------------------

    def paintEvent(self, event):
        del event
        painter = QtGui.QPainter(self)
        try:
            painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
            keys, tangents = self._Glyphs()
            self._DrawBackground(painter)
            painter.save()
            painter.setClipRect(self.PlotRectF())
            self._DrawGrid(painter)
            self._DrawOutsideRange(painter)
            self._DrawCurves(painter)
            self._DrawTangents(painter, keys, tangents)
            self._DrawKeys(painter, keys)
            self._DrawPlayhead(painter)
            self._DrawMarquee(painter)
            painter.restore()
            self._DrawRulers(painter)
        finally:
            painter.end()

    def _DrawBackground(self, painter):
        painter.fillRect(self.rect(), COLOR_BACKGROUND)
        plot = self.PlotRectF()
        painter.fillRect(QtCore.QRectF(0.0, plot.bottom(), self.width(),
                                       self.height() - plot.bottom()),
                         COLOR_RULER)
        painter.fillRect(QtCore.QRectF(0.0, 0.0, plot.left(), self.height()),
                         COLOR_RULER)

    def _DrawOutsideRange(self, painter):
        """usdview's start / end frames, shaded outside (spec 2.2)."""
        start, end = self._panel.StageRange()
        if start is None or end is None:
            return
        plot = self.PlotRectF()
        left = self.transform.TimeToX(start)
        right = self.transform.TimeToX(end)
        if left > plot.left():
            painter.fillRect(
                QtCore.QRectF(plot.left(), plot.top(),
                              min(left, plot.right()) - plot.left(),
                              plot.height()), COLOR_OUTSIDE)
        if right < plot.right():
            painter.fillRect(
                QtCore.QRectF(max(right, plot.left()), plot.top(),
                              plot.right() - max(right, plot.left()),
                              plot.height()), COLOR_OUTSIDE)

    def _DrawGrid(self, painter):
        plot = self.PlotRectF()
        timeTicks, valueTicks = graphScreen.GridLines(self.transform)
        pen = QtGui.QPen(COLOR_GRID)
        pen.setWidthF(GRID_WIDTH)
        axisPen = QtGui.QPen(COLOR_AXIS)
        axisPen.setWidthF(GRID_WIDTH)
        for time in timeTicks:
            x = self.transform.TimeToX(time)
            painter.setPen(axisPen if abs(time) < 1e-9 else pen)
            painter.drawLine(QtCore.QPointF(x, plot.top()),
                             QtCore.QPointF(x, plot.bottom()))
        for value in valueTicks:
            y = self.transform.ValueToY(value)
            painter.setPen(axisPen if abs(value) < 1e-9 else pen)
            painter.drawLine(QtCore.QPointF(plot.left(), y),
                             QtCore.QPointF(plot.right(), y))

    def _DrawCurves(self, painter):
        """
        Each visible curve as polylines, with the EXTRAPOLATED tails
        dashed (spec 2.2).

        The split at the first and last knot's pixel column is
        graphScreen.SplitExtrapolation, so it is covered headlessly by
        tests/python/test_graph_screen.py -- it used to live here and
        cut the PRE tail at the wrong bound, painting a solid run at the
        first key's value clean across the knot range.

        Non-finite samples are dropped first: linear extrapolation off a
        knot whose tangent algorithm is None samples as NaN in this Ts
        build, and a NaN point poisons the whole polyline.
        """
        curves = self._panel.Curves()
        splines = self._Splines()
        for index, spline in enumerate(splines):
            if spline is None or spline.IsEmpty():
                continue
            times = graphModel.KeyTimes(spline)
            if not times:
                continue
            xLow = self.transform.TimeToX(times[0])
            xHigh = self.transform.TimeToX(times[-1])
            color = _Color(curves[index].color)
            solidPen = QtGui.QPen(color)
            solidPen.setWidthF(CURVE_WIDTH)
            dashedPen = QtGui.QPen(color)
            dashedPen.setWidthF(CURVE_WIDTH)
            dashedPen.setStyle(QtCore.Qt.DashLine)
            for polyline in graphScreen.SamplePolylines(spline,
                                                        self.transform):
                for run in _FinitePolylines(polyline):
                    inside, outside = graphScreen.SplitExtrapolation(
                        run, xLow, xHigh)
                    painter.setPen(solidPen)
                    self._Strokes(painter, inside)
                    painter.setPen(dashedPen)
                    self._Strokes(painter, outside)

    @staticmethod
    def _Strokes(painter, runs):
        for points in runs:
            if len(points) > 1:
                painter.drawPolyline(QtGui.QPolygonF(
                    [QtCore.QPointF(p[0], p[1]) for p in points]))

    def _DrawKeys(self, painter, keys):
        curves = self._panel.Curves()
        half = graphScreen.KEY_PIXELS * 0.5
        painter.setPen(QtCore.Qt.NoPen)
        for glyph in keys:
            if glyph.selected:
                color = COLOR_SELECTED
            elif self._hover == (glyph.curveIndex, glyph.time):
                color = COLOR_HOVER
            else:
                color = _Color(curves[glyph.curveIndex].color)
            painter.setBrush(QtGui.QBrush(color))
            painter.drawRect(QtCore.QRectF(glyph.x - half, glyph.y - half,
                                           half * 2.0, half * 2.0))
            if glyph.IsDualValued():
                painter.drawRect(QtCore.QRectF(
                    glyph.x - half, glyph.preY - half, half * 2.0,
                    half * 2.0))

    def _DrawTangents(self, painter, keys, tangents):
        """
        The selected keys' handles: a line from the key to the end box,
        hollow when the tangent is produced by an algorithm rather than
        authored (spec 2.2).
        """
        anchors = {}
        for glyph in keys:
            anchors[(glyph.curveIndex, glyph.time)] = glyph
        half = graphScreen.TANGENT_PIXELS * 0.5
        pen = QtGui.QPen(COLOR_TANGENT)
        pen.setWidthF(1.0)
        for glyph in tangents:
            key = anchors.get((glyph.curveIndex, glyph.time))
            if key is None:
                continue
            y = key.y
            if glyph.side == SIDE_IN and key.IsDualValued():
                y = key.preY
            painter.setPen(pen)
            painter.setBrush(QtCore.Qt.NoBrush)
            painter.drawLine(QtCore.QPointF(key.x, y),
                             QtCore.QPointF(glyph.x, glyph.y))
            box = QtCore.QRectF(glyph.x - half, glyph.y - half,
                                half * 2.0, half * 2.0)
            if glyph.locked:
                painter.drawRect(box)
            else:
                painter.setPen(QtCore.Qt.NoPen)
                painter.setBrush(QtGui.QBrush(COLOR_TANGENT))
                painter.drawRect(box)

    def _DrawPlayhead(self, painter):
        plot = self.PlotRectF()
        x = self.transform.TimeToX(self._panel.CurrentFrame())
        pen = QtGui.QPen(COLOR_PLAYHEAD)
        pen.setWidthF(PLAYHEAD_WIDTH)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawLine(QtCore.QPointF(x, plot.top()),
                         QtCore.QPointF(x, plot.bottom()))
        painter.drawText(QtCore.QPointF(x + 4.0, plot.top() + 12.0),
                         "%g" % self._panel.CurrentFrame())

    def _DrawMarquee(self, painter):
        if self._marquee is None:
            return
        x, y, w, h = self._marquee
        pen = QtGui.QPen(COLOR_MARQUEE)
        pen.setStyle(QtCore.Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawRect(QtCore.QRectF(x, y, w, h).normalized())

    def _DrawRulers(self, painter):
        """The frame ruler along the bottom and the value ruler down the
        left, labelled from the same ticks the grid was drawn from."""
        plot = self.PlotRectF()
        timeTicks, valueTicks = graphScreen.GridLines(self.transform)
        _, _, w, h = self.transform.PlotRect()
        timeStep = graphScreen.NiceStep(self.transform.TimeSpan(), w)
        valueStep = graphScreen.NiceStep(self.transform.ValueSpan(), h)
        font = painter.font()
        font.setPointSizeF(max(7.0, font.pointSizeF() * 0.85))
        painter.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        painter.setPen(QtGui.QPen(COLOR_RULER_EDGE))
        painter.drawLine(QtCore.QPointF(plot.left(), plot.bottom()),
                         QtCore.QPointF(plot.right(), plot.bottom()))
        painter.drawLine(QtCore.QPointF(plot.left(), plot.top()),
                         QtCore.QPointF(plot.left(), plot.bottom()))
        painter.setPen(QtGui.QPen(COLOR_TEXT))
        for time in timeTicks:
            x = self.transform.TimeToX(time)
            text = _FormatTick(time, timeStep)
            painter.drawText(
                QtCore.QPointF(x - metrics.horizontalAdvance(text) * 0.5,
                               plot.bottom() + metrics.ascent() + 4.0), text)
        for value in valueTicks:
            y = self.transform.ValueToY(value)
            text = _FormatTick(value, valueStep)
            painter.drawText(
                QtCore.QPointF(
                    max(2.0, plot.left() - 6.0
                        - metrics.horizontalAdvance(text)),
                    y + metrics.ascent() * 0.5 - 1.0), text)

    # -- events ---------------------------------------------------------

    def _OnRuler(self, point):
        """Whether a press landed on the bottom frame ruler."""
        return point[1] >= self.PlotRectF().bottom()

    def mousePressEvent(self, event):
        point = _Position(event)
        modifiers = event.modifiers()
        button = event.button()
        self._press = point
        self._last = point
        if modifiers & QtCore.Qt.AltModifier:
            if button == QtCore.Qt.MiddleButton:
                self._mode = "pan"
                return
            if button == QtCore.Qt.RightButton:
                self._mode = "zoom"
                self._pressRanges = (self.transform.timeRange,
                                     self.transform.valueRange)
                return
            return
        if button == QtCore.Qt.LeftButton and self._OnRuler(point):
            self._mode = "scrub"
            self._panel.ScrubTo(self.transform.XToTime(point[0]))
            return
        if button == QtCore.Qt.LeftButton:
            self._PressLeft(point, modifiers)
            return
        if button == QtCore.Qt.MiddleButton:
            # Maya's "move the picked keys from anywhere": the selection
            # is not touched, so a middle drag never loses it.
            if self._panel.BeginKeyDrag(point):
                self._mode = "keys"

    def _PressLeft(self, point, modifiers):
        keys, tangents = self._Glyphs()
        tangent = graphScreen.HitTangent(tangents, point[0], point[1])
        key = graphScreen.HitKey(keys, point[0], point[1])
        # A tangent of zero width has its end box ON its own key, and
        # both pick radii are 6 px, so "handle first" would make such a
        # key ungrabbable. Whichever glyph the cursor is actually nearer
        # wins, and a tie goes to the key: an artist aiming at a handle
        # aims OUT along it, away from the key.
        if tangent is not None and (
                key is None
                or _Distance(point, tangent) < _KeyDistance(point, key)):
            anchor = self._Anchor(keys, tangent)
            if self._panel.BeginTangentDrag(tangent, anchor):
                self._mode = "tangent"
            return
        toggle = bool(modifiers & (QtCore.Qt.ShiftModifier
                                   | QtCore.Qt.ControlModifier))
        if key is not None:
            if toggle:
                self._panel.ToggleKey(key.curveIndex, key.time)
                return
            if not key.selected:
                self._panel.SetSelection([(key.curveIndex, key.time)])
            if self._panel.BeginKeyDrag(point):
                self._mode = "keys"
            return
        self._mode = "marquee"
        self._marquee = (point[0], point[1], 0.0, 0.0)
        if not toggle:
            self._panel.SetSelection([])

    @staticmethod
    def _Anchor(keys, tangent):
        """
        The pixel the tangent handle hangs off: the key's value square,
        or its PRE-value square for the in side of a dual-valued knot,
        which is where TangentGlyphs measured the handle from.
        """
        for glyph in keys:
            if (glyph.curveIndex == tangent.curveIndex
                    and glyph.time == tangent.time):
                if tangent.side == SIDE_IN and glyph.IsDualValued():
                    return (glyph.x, glyph.preY)
                return (glyph.x, glyph.y)
        return (tangent.x, tangent.y)

    def mouseMoveEvent(self, event):
        point = _Position(event)
        mode = self._mode
        if mode is None:
            self._UpdateHover(point)
            return
        if mode == "pan":
            self.transform.Pan(point[0] - self._last[0],
                               point[1] - self._last[1])
            self.update()
        elif mode == "zoom":
            self._Zoom(point)
        elif mode == "scrub":
            self._panel.ScrubTo(self.transform.XToTime(point[0]))
        elif mode == "marquee":
            self._marquee = (self._press[0], self._press[1],
                             point[0] - self._press[0],
                             point[1] - self._press[1])
            self.update()
        elif mode == "keys":
            self._panel.UpdateKeyDrag(
                point, bool(event.modifiers() & QtCore.Qt.ShiftModifier))
        elif mode == "tangent":
            self._panel.UpdateTangentDrag(point)
        self._last = point

    def _Zoom(self, point):
        """
        Maya's Alt+right drag: rightward magnifies time, upward
        magnifies value, both about the press point so the frame under
        the cursor when the drag started stays there.
        """
        self.transform.timeRange, self.transform.valueRange = \
            self._pressRanges
        factorX = 2.0 ** ((point[0] - self._press[0]) / ZOOM_DRAG_PIXELS)
        factorY = 2.0 ** ((self._press[1] - point[1]) / ZOOM_DRAG_PIXELS)
        self.transform.ZoomAbout(self._press[0], self._press[1],
                                 factorX, factorY)
        self.update()

    def mouseReleaseEvent(self, event):
        point = _Position(event)
        mode = self._mode
        self._mode = None
        if mode == "marquee":
            rect = self._marquee
            self._marquee = None
            if rect is not None and (abs(rect[2]) > CLICK_SLOP
                                     or abs(rect[3]) > CLICK_SLOP):
                keys = graphScreen.KeysInRect(self._Glyphs()[0], rect)
                add = bool(event.modifiers() & (QtCore.Qt.ShiftModifier
                                                | QtCore.Qt.ControlModifier))
                self._panel.SelectGlyphs(keys, add=add)
            self.update()
        elif mode in ("keys", "tangent"):
            self._panel.EndGesture()
        self._UpdateHover(point)

    def mouseDoubleClickEvent(self, event):
        if event.button() != QtCore.Qt.LeftButton:
            return
        point = _Position(event)
        self._mode = None
        self._marquee = None
        index = graphScreen.HitCurve(self._Polylines(), point[0], point[1])
        if index is None:
            return
        self._panel.InsertKeyOnCurve(
            index, graphScreen.CurveTimeAtX(self.transform, point[0]))

    def wheelEvent(self, event):
        try:
            delta = event.angleDelta().y()
            point = event.position()
            x, y = point.x(), point.y()
        except AttributeError:                            # Qt 5
            delta = event.delta()
            x, y = float(event.x()), float(event.y())
        if not delta:
            return
        factor = WHEEL_ZOOM ** (delta / 120.0)
        self.transform.ZoomAbout(x, y, factor, factor)
        self.update()
        event.accept()

    def leaveEvent(self, event):
        super(GraphCanvas, self).leaveEvent(event)
        self._UpdateHover(None)

    def _UpdateHover(self, point):
        name = None
        if point is not None:
            glyph = graphScreen.HitKey(self._Glyphs()[0], point[0], point[1])
            if glyph is not None:
                name = (glyph.curveIndex, glyph.time)
        if name != self._hover:
            self._hover = name
            self.update()

    def keyPressEvent(self, event):
        # The application filter normally gets here first; this is the
        # path for a key event posted straight at the canvas, which is
        # what a test does and a real keyboard never does.
        if not self._panel.HandleKey(event.key(), event.modifiers()):
            super(GraphCanvas, self).keyPressEvent(event)


def _Distance(point, glyph):
    return math.hypot(point[0] - glyph.x, point[1] - glyph.y)


def _KeyDistance(point, glyph):
    """A key is aimed at by EITHER square of a dual-valued knot."""
    distance = _Distance(point, glyph)
    if glyph.preY is not None:
        distance = min(distance,
                       math.hypot(point[0] - glyph.x, point[1] - glyph.preY))
    return distance


def _FinitePolylines(points):
    """
    `points` split at any non-finite sample, dropping it.

    Ts samples an extrapolated tail as NaN when the end knot has no
    tangent algorithm (verified in this build for ExtrapLinear off a
    TangentAlgorithmNone knot), and QPainter draws a polyline containing
    one NaN as a stray line across the whole widget.
    """
    runs = []
    current = []
    for point in points:
        if _IsFinite(point[0]) and _IsFinite(point[1]):
            current.append(point)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    return runs


# ---------------------------------------------------------------------------
# Gestures
# ---------------------------------------------------------------------------

class _Refused(Exception):
    """
    A model operation declined to change the spline.

    Raised through _Apply so the EditRecorder rolls the whole command
    back: several curves are edited in one pass, and a command that one
    of them refuses must not be half-applied to the rest.
    """

    def __init__(self, time=None):
        super(_Refused, self).__init__("refused")
        self.time = time


class _Gesture(object):
    """
    One undo step's worth of editing.

    Holds the spline every affected curve started with, the
    EditRecorder bracketing the whole gesture, and the live writes that
    make the viewport follow a drag. Every move recomputes from
    `origin`, never from the last write, so a drag cannot accumulate its
    own rounding or re-clamp an already clamped key.
    """

    def __init__(self, panel, indices, label):
        self.panel = panel
        self.indices = list(indices)
        self.label = label
        self.origin = {}
        self.press = (0.0, 0.0)
        self.times = {}
        self.tangent = None
        self.anchor = (0.0, 0.0)
        self.width = 0.0
        paths = []
        curves = panel.Curves()
        for index in self.indices:
            self.origin[index] = panel.SplineCopy(index)
            paths.append(curves[index].attrPath)
        self.recorder = rigExecUndo.EditRecorder(panel.Stage(), paths)
        self.recorder.Begin()

    def Origin(self, index):
        return Ts.Spline(self.origin[index])

    def Write(self, index, spline):
        self.panel.WriteSpline(index, spline, self.label)

    def Commit(self):
        edit = self.recorder.Commit(self.label)
        if edit is not None and self.panel.undoStack is not None:
            self.panel.undoStack.Push(edit)
        return edit

    def Abort(self):
        self.recorder.Abort()


# ---------------------------------------------------------------------------
# Hotkeys
# ---------------------------------------------------------------------------

class GraphHotkeyFilter(QtCore.QObject):
    """
    The editor's hotkeys, taken at the APPLICATION level.

    See this module's banner: usdview's AppEventFilter swallows Escape
    outright and turns a bare F into a 3D viewport frame, both from an
    application-wide filter, so a widget-level handler in this window
    would never see either key. Qt runs the most recently installed
    application filter first, and this one is installed when the editor
    opens -- after usdview's -- so it sees them first.

    Nothing is claimed unless it is acted on, and nothing at all is
    claimed for another window, for a hidden editor, or while focus is
    in a text field.
    """

    def __init__(self, panel):
        super(GraphHotkeyFilter, self).__init__(panel)
        self._panel = panel

    def eventFilter(self, obj, event):
        try:
            kind = event.type()
            if kind not in (QtCore.QEvent.KeyPress,
                            QtCore.QEvent.ShortcutOverride):
                return False
            panel = self._panel
            if not panel.isVisible() or not panel.OwnsEvent(obj):
                return False
            if _TypingFocus():
                return False
            key = event.key()
            if key not in panel.HOTKEYS:
                return False
            if event.modifiers() & (QtCore.Qt.ControlModifier
                                    | QtCore.Qt.AltModifier
                                    | QtCore.Qt.MetaModifier):
                return False
            if kind == QtCore.QEvent.ShortcutOverride:
                # Accepting the override is what stops usdview's own
                # QAction shortcuts from eating the KeyPress that
                # follows; the key is acted on there, once.
                event.accept()
                return True
            if not panel.HandleKey(key, event.modifiers()):
                return False
            event.accept()
            return True
        except Exception as error:
            # An exception escaping an application-wide filter would
            # break every key in usdview, not just ours.
            Tf.Warn("rigExecUsdview: graph editor hotkey filter failed: %s"
                    % error)
            return False


def _TypingFocus():
    focus = QtWidgets.QApplication.focusWidget()
    if focus is None:
        return False
    if isinstance(focus, _TEXT_WIDGETS):
        return True
    return (isinstance(focus, QtWidgets.QComboBox) and focus.isEditable())


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------

class GraphEditorPanel(QtWidgets.QWidget):
    """
    The Graph Editor window: key stats and tools across the top, the
    curve list on the left, the canvas in the middle, a status line
    underneath.

    One instance per usdview session, parented to the main window
    exactly like VolumeWeightPanel, so it floats over usdview and shares
    its lifetime.
    """

    __instance = None

    HOTKEYS = (QtCore.Qt.Key_A, QtCore.Qt.Key_F, QtCore.Qt.Key_I,
               QtCore.Qt.Key_Home, QtCore.Qt.Key_Delete,
               QtCore.Qt.Key_Backspace, QtCore.Qt.Key_Escape)

    @classmethod
    def GetInstance(cls, usdviewApi, undoStack):
        if cls.__instance is None:
            cls.__instance = GraphEditorPanel(usdviewApi, undoStack)
        return cls.__instance

    @classmethod
    def Instance(cls):
        return cls.__instance

    def __init__(self, usdviewApi, undoStack):
        super(GraphEditorPanel, self).__init__(
            usdviewApi.qMainWindow, QtCore.Qt.WindowType.Window)
        self.usdviewApi = usdviewApi
        self.undoStack = undoStack

        self._curves = []
        self._splines = []
        self._hidden = set()          # curve labels unticked
        self._isolated = set()        # curve labels selected in the list
        self._selection = set()       # (curveIndex, time)
        self._tangentSide = SIDE_BOTH
        self._snapFrames = True
        self._weighted = False
        self._gesture = None
        self._updating = False
        self._notice = ""
        self._framed = False
        self._noticeKey = None
        self._frame = _FrameValue(usdviewApi.frame)

        self.setWindowTitle("RigExec: Graph Editor")
        # A floor, not a size: below it a QHBoxLayout stops clipping the
        # tool rows and starts shrinking the checkboxes under their own
        # labels, which reads as a rendering fault rather than a squeeze.
        self.setMinimumWidth(940)
        self.resize(1100, 640)

        self._BuildUI()
        self._InstallUndoActions()
        self._hotkeys = GraphHotkeyFilter(self)
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.installEventFilter(self._hotkeys)

        model = usdviewApi.dataModel
        model.selection.signalPrimSelectionChanged.connect(
            self._onPrimSelectionChanged)
        model.selection.signalPropSelectionChanged.connect(
            self._onPropSelectionChanged)
        model.signalStageReplaced.connect(self._onStageReplaced)
        model.currentFrameChanged.connect(self._onFrameChanged)
        if undoStack is not None:
            undoStack.AddListener(self._onUndoStackChanged)
        self._ObserveStage(model.stage)

        self.RefreshCurves()

    # -- construction ---------------------------------------------------

    def _BuildUI(self):
        outer = QtWidgets.QVBoxLayout()
        outer.setContentsMargins(6, 6, 6, 4)
        outer.setSpacing(4)
        self.setLayout(outer)

        outer.addLayout(self._BuildKeyStatRow())
        outer.addLayout(self._BuildTangentRow())

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.curveList = CurveListWidget(self)
        self.curveList.itemChanged.connect(self._onCurveItemChanged)
        self.curveList.itemSelectionChanged.connect(self._onCurveIsolation)
        listSide = QtWidgets.QWidget()
        listLayout = QtWidgets.QVBoxLayout()
        listLayout.setContentsMargins(0, 0, 0, 0)
        listLayout.setSpacing(3)
        listLayout.addWidget(self.curveList)
        showAll = QtWidgets.QPushButton("Show All")
        showAll.setToolTip("Show every curve again and drop the isolation.")
        showAll.clicked.connect(self._onShowAll)
        listLayout.addWidget(showAll)
        listSide.setLayout(listLayout)
        splitter.addWidget(listSide)

        self.canvas = GraphCanvas(self)
        splitter.addWidget(self.canvas)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([230, 780])
        outer.addWidget(splitter, 1)

        self.statusLabel = QtWidgets.QLabel("")
        self.statusLabel.setToolTip(
            "What the editor is showing and what is selected.")
        outer.addWidget(self.statusLabel)

    def _BuildKeyStatRow(self):
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        row.addWidget(QtWidgets.QLabel("Time"))
        self.timeBox = BlankableSpinBox()
        self.timeBox.setDecimals(3)
        self.timeBox.setRange(-1.0e6, 1.0e6)
        self.timeBox.setFixedWidth(86)
        self.timeBox.setToolTip(
            "The selected keys' frame; blank when they differ. Typing "
            "moves every selected key by the same amount.")
        self.timeBox.editingFinished.connect(self._onTimeEdited)
        row.addWidget(self.timeBox)

        row.addWidget(QtWidgets.QLabel("Value"))
        self.valueBox = BlankableSpinBox()
        self.valueBox.setDecimals(4)
        self.valueBox.setRange(-1.0e9, 1.0e9)
        self.valueBox.setFixedWidth(96)
        self.valueBox.setToolTip(
            "The selected keys' value; blank when they differ. Typing "
            "sets every selected key to it.")
        self.valueBox.editingFinished.connect(self._onValueEdited)
        row.addWidget(self.valueBox)

        for text, tip, slot in (
                ("Frame All (A)", "Fit every visible curve.",
                 self._onFrameAll),
                ("Frame Selected (F)",
                 "Fit the selected keys, or everything when none are.",
                 self._onFrameSelected),
                ("Insert Key (I)",
                 "Key every visible curve at the current frame.",
                 self._onInsertKey),
                ("Delete (Del)", "Remove the selected keys.",
                 self._onDeleteKeys)):
            button = QtWidgets.QPushButton(text)
            button.setToolTip(tip)
            button.clicked.connect(slot)
            row.addWidget(button)

        # Snap Frames and Weighted ride with the key stats rather than
        # with the tangent buttons: the second row is already at the
        # window's minimum width, and a QHBoxLayout that overflows
        # shrinks its checkboxes UNDER their own text rather than
        # clipping the row.
        row.addSpacing(8)
        self.snapBox = QtWidgets.QCheckBox("Snap Frames")
        self.snapBox.setChecked(True)
        self.snapBox.setToolTip("Keep dragged keys on whole frames.")
        self.snapBox.toggled.connect(self._onSnapFrames)
        row.addWidget(self.snapBox)

        self.weightedBox = QtWidgets.QCheckBox("Weighted")
        self.weightedBox.setToolTip(
            "Let a handle drag change the tangent's LENGTH as well as "
            "its angle.")
        self.weightedBox.toggled.connect(self._onWeighted)
        row.addWidget(self.weightedBox)
        for box in (self.snapBox, self.weightedBox):
            # A QCheckBox's minimumSizeHint is narrower than its label,
            # so a full row shrinks it until the next widget overwrites
            # the text. Pinning the minimum to the text's own width plus
            # the indicator makes the WINDOW refuse to go that narrow
            # instead; the style's size hint is not reliable before the
            # widget has been polished, so it is only a floor here.
            width = box.fontMetrics().horizontalAdvance(box.text()) + 30
            box.setMinimumWidth(max(width, box.sizeHint().width()))
        row.addStretch(1)
        return row

    def _BuildTangentRow(self):
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        row.addWidget(QtWidgets.QLabel("Tangent"))
        for label, mode, tip in TANGENT_BUTTONS:
            button = QtWidgets.QPushButton(label)
            button.setToolTip(tip)
            button.clicked.connect(
                lambda checked=False, m=mode: self.SetTangentType(m))
            row.addWidget(button)

        row.addSpacing(8)
        self._sideButtons = {}
        group = QtWidgets.QButtonGroup(self)
        for label, side in (("In", SIDE_IN), ("Out", SIDE_OUT),
                            ("Both", SIDE_BOTH)):
            button = QtWidgets.QRadioButton(label)
            button.setToolTip(
                "Which side of the key the tangent buttons apply to.")
            button.setChecked(side == self._tangentSide)
            button.toggled.connect(
                lambda checked, s=side: self._onSide(checked, s))
            group.addButton(button)
            row.addWidget(button)
            self._sideButtons[side] = button

        row.addSpacing(8)
        for label, tip, slot in (
                ("Break", "Let the two tangents of a key move "
                          "independently.", self._onBreak),
                ("Unify", "Re-link the two tangents; the out slope wins.",
                 self._onUnify)):
            button = QtWidgets.QPushButton(label)
            button.setToolTip(tip)
            button.clicked.connect(slot)
            row.addWidget(button)

        row.addSpacing(8)
        row.addWidget(QtWidgets.QLabel("Infinity"))
        self.preInfinity = self._InfinityCombo(
            "What the curve does BEFORE its first key.",
            self._onPreInfinityChanged)
        row.addWidget(self.preInfinity)
        self.postInfinity = self._InfinityCombo(
            "What the curve does AFTER its last key.",
            self._onPostInfinityChanged)
        row.addWidget(self.postInfinity)
        row.addStretch(1)
        return row

    def _InfinityCombo(self, tip, slot):
        combo = QtWidgets.QComboBox()
        combo.setToolTip(tip)
        combo.setFixedWidth(132)
        for label, name in INFINITY_CHOICES:
            combo.addItem(label, name)
        combo.currentIndexChanged.connect(slot)
        return combo

    def _InstallUndoActions(self):
        """
        Ctrl+Z / Ctrl+Shift+Z while the editor has focus, on the SAME
        stack the viewport gizmos push onto.

        When the gizmo toolbar is up, its own actions are reused rather
        than duplicated: they carry the same shortcuts with
        Qt.ApplicationShortcut context, and a second action with the
        same sequence would make every Ctrl+Z ambiguous -- Qt refuses to
        fire either and prints "Ambiguous shortcut overload". Adding the
        toolbar's actions to this window is enough to reach them from
        here, and the undo stack listener repaints the canvas whichever
        one runs.
        """
        actions = self._GizmoUndoActions()
        self._ownsUndoActions = actions is None
        if actions is None:
            self.undoAction = QtActionWidgets.QAction("Undo", self)
            self.undoAction.setShortcut(QtGui.QKeySequence("Ctrl+Z"))
            self.undoAction.setShortcutContext(QtCore.Qt.WindowShortcut)
            self.undoAction.triggered.connect(
                lambda checked=False: self.Undo())
            self.redoAction = QtActionWidgets.QAction("Redo", self)
            self.redoAction.setShortcuts([
                QtGui.QKeySequence("Ctrl+Shift+Z"),
                QtGui.QKeySequence("Shift+Z"),
                QtGui.QKeySequence("Ctrl+Y")])
            self.redoAction.setShortcutContext(QtCore.Qt.WindowShortcut)
            self.redoAction.triggered.connect(
                lambda checked=False: self.Redo())
            actions = [self.undoAction, self.redoAction]
        else:
            self.undoAction, self.redoAction = actions
        for action in actions:
            self.addAction(action)

    def _GizmoUndoActions(self):
        """The toolbar's undo/redo actions when they drive OUR stack."""
        try:
            import gizmoUI
        except Exception:
            return None
        controller = gizmoUI.GetController()
        if controller is None or controller.undoStack is not self.undoStack:
            return None
        undo = getattr(controller.toolbar, "undoAction", None)
        redo = getattr(controller.toolbar, "redoAction", None)
        if undo is None or redo is None:
            return None
        return [undo, redo]

    # -- stage and curves -----------------------------------------------

    def Stage(self):
        return self.usdviewApi.stage

    def Curves(self):
        """The CurveRefs the editor is showing, in list order."""
        return list(self._curves)

    def CurrentFrame(self):
        return self._frame

    def StageRange(self):
        """usdview's (start, end) frames, or (None, None)."""
        stage = self.Stage()
        if not stage:
            return (None, None)
        try:
            return (stage.GetStartTimeCode(), stage.GetEndTimeCode())
        except Exception:
            return (None, None)

    def _Attribute(self, index):
        stage = self.Stage()
        if not stage or index >= len(self._curves):
            return None
        return stage.GetAttributeAtPath(self._curves[index].attrPath)

    def SplineCopy(self, index):
        """
        A private copy of curve `index`'s spline, always TYPED.

        graphModel.SplineFor is what makes an unanimated channel
        keyable: Usd hands back an untyped spline for an attribute that
        has none, and Ts.Knot(typeName="") raises.
        """
        spline = self._splines[index] if index < len(self._splines) else None
        if spline is None:
            attr = self._Attribute(index)
            spline = (graphModel.SplineFor(attr) if attr is not None
                      else Ts.Spline("double"))
        copy = Ts.Spline(spline)
        if not copy.GetValueTypeName():
            attr = self._Attribute(index)
            if attr is not None:
                copy = graphModel.SplineFor(attr)
        return copy

    def WriteSpline(self, index, spline, label="Graph Edit"):
        """
        Author `spline` onto curve `index`, through the model.

        graphModel.ApplySpline is the editor's ONLY writer, so the rule
        that an EMPTY spline clears the opinion rather than authoring an
        empty curve (graphModel.ClearSpline: an empty spline still wins
        value resolution and resolves to no value, which reads back as
        None) holds on every path -- deleting a curve's last key, a live
        drag, a tangent button -- without this file restating it.

        The undo stack argument is None ON PURPOSE. The gesture around
        this write is what pushes, once, at its release; ApplySpline's
        own recorder then only measures this one write and its Edit is
        dropped, which is what lets a drag author on EVERY mouse move
        without leaving a hundred entries in the history.

        What is CACHED afterwards is what the attribute now RESOLVES to,
        not the spline handed in. Since an empty spline clears the edit
        target's opinion instead of authoring an empty curve, the two
        stopped being the same thing: delete a root-animated curve's
        last key with the session layer as the edit target and the root
        layer's spline reappears, so caching the empty spline would draw
        a curve with no keys while the viewport kept animating it. The
        ObjectsChanged notice that would otherwise correct this is
        discarded, correctly, because the gesture still holds _gesture.
        """
        if index >= len(self._curves) or not self.Stage():
            return False
        attr = self._Attribute(index)
        if attr is None or not attr:
            # The attribute went out from under the editor (a stage edit
            # between the gesture starting and this write). Leave the
            # cache alone: it is the last thing that was true.
            return False
        try:
            graphModel.ApplySpline(self.Stage(), self._curves[index].attrPath,
                                   spline, None, label)
        except Exception as error:
            Tf.Warn("rigExecUsdview: graph editor could not author %s: %s"
                    % (self._curves[index].attrPath, error))
            return False
        self._splines[index] = graphModel.SplineFor(attr)
        self.canvas.update()
        self.usdviewApi.UpdateViewport()
        return True

    def RefreshCurves(self):
        """
        Rebuild the curve set from usdview's property / prim selection
        (spec 1.3), keeping the visibility and the key selection of any
        curve that survives.
        """
        stage = self.Stage()
        previous = list(self._curves)
        if not stage:
            self._curves = []
        else:
            selection = self.usdviewApi.dataModel.selection
            try:
                propPaths = selection.getPropPaths()
            except Exception:
                propPaths = []
            try:
                primPaths = selection.getPrimPaths()
            except Exception:
                primPaths = []
            self._curves = graphModel.DiscoverCurves(stage, propPaths,
                                                     primPaths)
        self._ReadSplines()
        self._RemapSelection(previous)
        self._RebuildCurveList()
        if self._curves and (not previous or not self._framed):
            self.FrameAll()
        self._Sync()

    def _ReadSplines(self):
        stage = self.Stage()
        self._splines = []
        for ref in self._curves:
            attr = stage.GetAttributeAtPath(ref.attrPath) if stage else None
            if attr is None or not attr:
                self._splines.append(None)
                continue
            try:
                self._splines.append(graphModel.SplineFor(attr))
            except Exception:
                self._splines.append(None)

    def _RemapSelection(self, previous):
        """
        Carry the key selection across a curve-set rebuild by LABEL: the
        indices are positions in the new list and mean nothing in the
        old one.
        """
        if not self._selection:
            return
        labels = {}
        for index, ref in enumerate(self._curves):
            labels[ref.Label()] = index
        remapped = set()
        for index, time in self._selection:
            if index >= len(previous):
                continue
            target = labels.get(previous[index].Label())
            if target is None:
                continue
            spline = self._splines[target]
            if spline is not None and spline.GetKnot(time) is not None:
                remapped.add((target, time))
        self._selection = remapped

    # -- curve list -----------------------------------------------------

    def _RebuildCurveList(self):
        self._updating = True
        try:
            self.curveList.clear()
            for ref in self._curves:
                item = QtWidgets.QTreeWidgetItem(
                    [ref.primPath.name, ref.attrName])
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                hidden = ref.Label() in self._hidden
                item.setCheckState(
                    0, QtCore.Qt.Unchecked if hidden else QtCore.Qt.Checked)
                # Dimmed text as well as an empty tick: one glance has to
                # say which curves are on the canvas, and a bare check
                # indicator is small against sixteen coloured rows.
                brush = QtGui.QBrush(QtGui.QColor(120, 120, 120) if hidden
                                     else _Color(ref.color))
                item.setForeground(0, brush)
                item.setForeground(1, brush)
                item.setIcon(0, _SwatchIcon(ref.color))
                item.setData(0, QtCore.Qt.UserRole, ref.Label())
                self.curveList.addTopLevelItem(item)
                item.setSelected(ref.Label() in self._isolated)
            self.curveList.resizeColumnToContents(0)
        finally:
            self._updating = False

    def _onCurveItemChanged(self, item, column):
        del column
        if self._updating:
            return
        label = item.data(0, QtCore.Qt.UserRole)
        if item.checkState(0) == QtCore.Qt.Checked:
            self._hidden.discard(label)
        else:
            self._hidden.add(label)
        self.canvas.update()
        self._Sync()

    def _onCurveIsolation(self):
        if self._updating:
            return
        self._isolated = set(
            item.data(0, QtCore.Qt.UserRole)
            for item in self.curveList.selectedItems())
        self.canvas.update()
        self._Sync()

    def _onShowAll(self):
        self._hidden = set()
        self._isolated = set()
        self._RebuildCurveList()
        self.canvas.update()
        self._Sync()

    def IsVisibleCurve(self, index):
        label = self._curves[index].Label()
        if label in self._hidden:
            return False
        return not self._isolated or label in self._isolated

    def VisibleIndices(self):
        return [i for i in range(len(self._curves))
                if self.IsVisibleCurve(i)]

    def DrawSplines(self):
        """The spline per curve, None where the curve is not drawn."""
        return [self._splines[i] if self.IsVisibleCurve(i) else None
                for i in range(len(self._curves))]

    def SetVisibleCurves(self, labels):
        """Show exactly these curves, by CurveRef.Label(), and no others."""
        wanted = set(labels)
        self._hidden = set(ref.Label() for ref in self._curves
                           if ref.Label() not in wanted)
        self._isolated = set()
        self._RebuildCurveList()
        self.canvas.update()
        self._Sync()

    # -- selection ------------------------------------------------------

    def SelectionPairs(self):
        """The raw (curveIndex, time) selection graphScreen wants."""
        return self._selection

    def SelectedKeys(self):
        """[(curve label, time)] for the selected keys, in list order."""
        return [(self._curves[index].Label(), time)
                for index, time in sorted(self._selection)]

    def SelectKeys(self, pairs):
        """Select exactly these (curve label, time) keys."""
        labels = dict((ref.Label(), index)
                      for index, ref in enumerate(self._curves))
        selection = set()
        for label, time in pairs:
            index = labels.get(label)
            if index is None:
                continue
            spline = self._splines[index]
            if spline is None:
                continue
            knot = spline.GetKnot(float(time))
            if knot is not None:
                selection.add((index, knot.GetTime()))
        self._selection = selection
        self.canvas.update()
        self._Sync()

    def SetSelection(self, pairs):
        self._selection = set(pairs)
        self.canvas.update()
        self._Sync()

    def ToggleKey(self, index, time):
        pair = (index, time)
        if pair in self._selection:
            self._selection.discard(pair)
        else:
            self._selection.add(pair)
        self.canvas.update()
        self._Sync()

    def SelectGlyphs(self, glyphs, add=False):
        pairs = set((g.curveIndex, g.time) for g in glyphs)
        self._selection = (self._selection | pairs) if add else pairs
        self.canvas.update()
        self._Sync()

    def SelectedTimes(self):
        """{curve index: [time, ...]} for the selected keys."""
        times = {}
        for index, time in sorted(self._selection):
            times.setdefault(index, []).append(time)
        return times

    def KeyExtents(self, selectedOnly=False):
        """
        (tMin, tMax, vMin, vMax) over the keys, or None when there are
        none. Values come from the knots, not from the sampled curve: a
        Bezier can overshoot its keys and Maya frames the keys.
        """
        times, values = [], []
        for index in self.VisibleIndices():
            spline = self._splines[index]
            if spline is None or spline.IsEmpty():
                continue
            for knot in spline.GetKnots().values():
                time = knot.GetTime()
                if selectedOnly and (index, time) not in self._selection:
                    continue
                times.append(time)
                values.append(float(knot.GetValue()))
                if knot.IsDualValued():
                    values.append(float(knot.GetPreValue()))
        if not times:
            return None
        return (min(times), max(times), min(values), max(values))

    # -- gestures -------------------------------------------------------

    def BeginKeyDrag(self, press):
        """Start moving the selected keys; False when none are."""
        times = self.SelectedTimes()
        if not times or self._gesture is not None or not self.Stage():
            return False
        gesture = _Gesture(self, sorted(times), "Move Keys")
        gesture.press = press
        gesture.times = times
        self._gesture = gesture
        return True

    def UpdateKeyDrag(self, current, shift=False):
        gesture = self._gesture
        if gesture is None:
            return
        axis = (graphScreen.DominantAxis(gesture.press, current)
                if shift else None)
        dt, dv = graphScreen.ResolveKeyDrag(self.canvas.transform,
                                            gesture.press, current, axis)
        selection = set()
        for index in gesture.indices:
            spline = gesture.Origin(index)
            moved = graphModel.MoveKeys(spline, gesture.times[index], dt, dv,
                                        self._snapFrames)
            gesture.Write(index, spline)
            for time in moved:
                selection.add((index, time))
        self._selection = selection
        self._Sync()

    def BeginTangentDrag(self, glyph, anchor):
        if self._gesture is not None or not self.Stage():
            return False
        gesture = _Gesture(self, [glyph.curveIndex], "Edit Tangent")
        gesture.tangent = glyph
        gesture.anchor = anchor
        gesture.width = glyph.width
        self._gesture = gesture
        return True

    def UpdateTangentDrag(self, current):
        """
        Slope from the handle's angle, width too while Weighted is on,
        mirrored to the other side while the key's tangents are unified
        (spec 2.4).
        """
        gesture = self._gesture
        if gesture is None or gesture.tangent is None:
            return
        glyph = gesture.tangent
        slope, width = graphScreen.ResolveTangentDrag(
            self.canvas.transform, gesture.anchor, current, self._weighted,
            gesture.width, side=glyph.side)
        spline = gesture.Origin(glyph.curveIndex)
        unified = graphModel.IsUnified(spline.GetKnot(glyph.time))
        graphModel.SetTangent(spline, glyph.time, glyph.side, slope,
                              width if self._weighted else None)
        if unified:
            other = SIDE_IN if glyph.side == SIDE_OUT else SIDE_OUT
            # Slope only: Maya's unified tangents share an angle, not a
            # length, so the far handle keeps the weight it had.
            graphModel.SetTangent(spline, glyph.time, other, slope)
        gesture.Write(glyph.curveIndex, spline)
        self._Sync()

    def EndGesture(self):
        """
        Close the live gesture; True when it changed the edit target.

        A drag that ends where it started, or on a curve a weaker layer
        owns, commits nothing -- the recorder compares the spec before
        and after -- and there is then no undo entry to offer.
        """
        gesture = self._gesture
        self._gesture = None
        if gesture is None:
            return False
        changed = False
        try:
            changed = gesture.Commit() is not None
        except Exception as error:
            Tf.Warn("rigExecUsdview: graph editor could not record the "
                    "edit: %s" % error)
        self._Sync()
        self.canvas.update()
        return changed

    def AbortGesture(self):
        gesture = self._gesture
        self._gesture = None
        if gesture is None:
            return False
        try:
            gesture.Abort()
        except Exception as error:
            Tf.Warn("rigExecUsdview: graph editor could not abort the "
                    "edit: %s" % error)
        self._ReadSplines()
        self._Sync()
        self.canvas.update()
        self.usdviewApi.UpdateViewport()
        return True

    def _Apply(self, indices, label, operation):
        """
        Run `operation(index, spline)` on a copy of each curve's spline
        and write them all as ONE undo step.

        This is the non-drag half of the same bracket _Gesture gives a
        drag: a tangent button pressed with three curves selected is one
        step, not three.

        An operation that RAISES aborts the whole command: the recorder
        puts every curve back, so a command that cannot be applied to
        one curve is not half-applied to the others. `_Refused` is how
        the Time field says "this would consume another key" (see
        SetKeyTime); anything else propagates.

        Returns True only when the edit target actually CHANGED -- when
        the recorder produced an Edit and it went on the undo stack.
        A command can run through cleanly and change nothing: deleting
        keys that a weaker layer owns clears an opinion the edit target
        never held. Reporting that as success would clear the selection
        and offer a Ctrl+Z for an edit that is not there.
        """
        indices = [i for i in indices if i < len(self._curves)]
        if not indices or not self.Stage() or self._gesture is not None:
            return False
        gesture = _Gesture(self, indices, label)
        # Held while the writes land so the ObjectsChanged handler does
        # not re-read the very splines this call is authoring.
        self._gesture = gesture
        try:
            for index in indices:
                spline = gesture.Origin(index)
                if operation(index, spline) is False:
                    continue
                gesture.Write(index, spline)
            changed = gesture.Commit() is not None
        except Exception:
            gesture.Abort()
            # The layer is back but the panel's cached splines are the
            # half-applied ones it wrote on the way to the failure.
            self._ReadSplines()
            self._PruneSelection()
            raise
        finally:
            self._gesture = None
        self._Sync()
        self.canvas.update()
        return changed

    # -- edit commands --------------------------------------------------

    def SetTangentType(self, mode):
        """Maya's tangent buttons, on the selected keys (spec 2.4)."""
        times = self.SelectedTimes()
        if not times:
            return False
        return self._Apply(
            sorted(times), "Tangent %s" % mode.capitalize(),
            lambda index, spline: graphModel.SetTangentType(
                spline, times[index], mode, self._tangentSide))

    def SetTangentSide(self, side):
        """Narrow the tangent buttons to the In or Out side, or Both."""
        if side not in (SIDE_IN, SIDE_OUT, SIDE_BOTH):
            raise ValueError("unknown tangent side %r" % (side,))
        self._tangentSide = side
        button = self._sideButtons.get(side)
        if button is not None and not button.isChecked():
            button.setChecked(True)
        self._Sync()

    def TangentSide(self):
        return self._tangentSide

    def Break(self):
        times = self.SelectedTimes()
        if not times:
            return False
        return self._Apply(
            sorted(times), "Break Tangents",
            lambda index, spline: graphModel.BreakTangents(
                spline, times[index]))

    def Unify(self):
        times = self.SelectedTimes()
        if not times:
            return False
        return self._Apply(
            sorted(times), "Unify Tangents",
            lambda index, spline: graphModel.UnifyTangents(
                spline, times[index]))

    def InsertKeyAtCurrentFrame(self):
        """
        Key every visible curve at the current frame (spec 2.4).

        A curve with no knots at all cannot be evaluated, so it is keyed
        from the attribute's RESOLVED value instead -- which is how an
        unanimated avar gets its first key from the graph editor.
        """
        frame = graphModel.SnapTime(self._frame)
        indices = self.VisibleIndices()
        if not indices:
            return False
        applied = self._Apply(indices, "Insert Key",
                              lambda index, spline: self._InsertOne(
                                  index, spline, frame))
        if applied:
            self._selection = set((i, frame) for i in indices)
            self._Sync()
            self.canvas.update()
        return applied

    def _InsertOne(self, index, spline, frame):
        if graphModel.InsertKey(spline, frame) is not None:
            return True
        attr = self._Attribute(index)
        value = attr.Get(Usd.TimeCode(frame)) if attr is not None else None
        graphModel.AuthorKnot(spline, frame, float(value or 0.0))
        return True

    def InsertKeyOnCurve(self, index, time):
        """A double-click on a curve: one key, on that curve, there."""
        frame = graphModel.SnapTime(time) if self._snapFrames else float(time)
        applied = self._Apply([index], "Insert Key",
                              lambda i, spline: self._InsertOne(i, spline,
                                                                frame))
        if applied:
            self._selection = set([(index, frame)])
            self._Sync()
            self.canvas.update()
        return applied

    def DeleteSelectedKeys(self):
        """
        Remove the selected keys from the EDIT TARGET; True when the
        layer changed.

        Deleting is a clear, not an erasure, so a key can survive it:
        the editor draws the curve the stage RESOLVES, and a key that
        belongs to a weaker layer than the edit target is still there
        afterwards. Two outcomes get a status line rather than a silent
        surprise.

        Nothing changed at all -- the edit target held no opinion on
        those keys -- keeps the selection, because the keys the artist
        picked are still on screen and still theirs to pick.
        """
        times = self.SelectedTimes()
        if not times:
            return False
        applied = self._Apply(
            sorted(times), "Delete Keys",
            lambda index, spline: graphModel.DeleteKeys(spline,
                                                        times[index]))
        survivors = self._SurvivingKeys(times)
        if not applied:
            self._Notify("Nothing deleted: those keys come from a weaker "
                         "layer, not the edit target.")
            return False
        self._selection = set()
        self._Sync()
        self.canvas.update()
        if survivors:
            self._Notify("Session keys cleared; file animation shows "
                         "through.")
        return True

    def _SurvivingKeys(self, times):
        """
        The curves among `times` still holding one of those keys after
        a delete, read off the RESOLVED spline the write cached.
        """
        survivors = []
        for index, group in times.items():
            spline = (self._splines[index] if index < len(self._splines)
                      else None)
            if spline is None:
                continue
            if any(spline.GetKnot(time) is not None for time in group):
                survivors.append(index)
        return survivors

    def AnimatedIndices(self):
        """
        The visible curves that actually HAVE keys.

        The editor lists a control's unanimated rig channels too, so an
        artist can key one from the graph (spec 1.3). They are not
        curves yet: extrapolation on a knotless spline means nothing,
        and authoring one would replace the channel's resolved value
        with an empty spline.
        """
        return [i for i in self.VisibleIndices()
                if self._splines[i] is not None
                and not self._splines[i].IsEmpty()]

    def SetInfinity(self, pre=None, post=None):
        """
        Maya's Infinity combos, on the visible curves that have keys.

        The visible set IS the selected set here: picking rows in the
        curve list isolates them, so what is drawn is what an artist
        would call selected. Knotless channels are skipped -- see
        AnimatedIndices.

        `pre` / `post` are independent: each combo passes only its own
        side and leaves the other None, so changing the post infinity
        cannot quietly rewrite every curve's pre infinity from whatever
        the other combo happened to be displaying.
        """
        indices = self.AnimatedIndices()
        if not indices:
            self._Notify("Infinity needs a curve with keys.")
            return False
        return self._Apply(
            indices, "Set Infinity",
            lambda index, spline: graphModel.SetExtrapolation(spline, pre,
                                                              post))

    def SetSnapFrames(self, enabled):
        self._snapFrames = bool(enabled)
        if self.snapBox.isChecked() != self._snapFrames:
            self.snapBox.setChecked(self._snapFrames)

    def SnapFrames(self):
        return self._snapFrames

    def SetWeighted(self, enabled):
        self._weighted = bool(enabled)
        if self.weightedBox.isChecked() != self._weighted:
            self.weightedBox.setChecked(self._weighted)

    def Weighted(self):
        return self._weighted

    def SetKeyTime(self, time):
        """
        Put the selection at `time`: Maya's Time field (spec 2.4).

        graphModel.SetKeyTimes does the move per spline, refusing when
        the frame is already held by a key OUTSIDE the selection. Across
        several curves the selection has to stay RIGID, so each curve is
        asked for its own target -- the earliest selected key of ALL of
        them lands on `time` and the rest keep their offsets -- and one
        curve's refusal aborts the whole command rather than shearing
        the selection apart.
        """
        times = self.SelectedTimes()
        if not times:
            return False
        target = float(time)
        earliest = min(t for group in times.values() for t in group)
        if abs(target - earliest) < 1e-12:
            return False

        def Move(index, spline):
            offset = min(times[index]) - earliest
            if not graphModel.SetKeyTimes(spline, times[index],
                                          target + offset):
                raise _Refused(target + offset)
            return True

        try:
            applied = self._Apply(sorted(times), "Move Keys", Move)
        except _Refused as refusal:
            self._Notify("Frame %g already has a key; nothing moved."
                         % refusal.time)
            return False
        if applied:
            self._SelectMovedKeys(times, target - earliest)
        return applied

    def _SelectMovedKeys(self, times, delta):
        """
        Follow the keys the Time field just moved.

        Their new times are read back off the stage rather than assumed:
        graphModel clamps a key that has no room, so the shift asked for
        is not always the shift applied.
        """
        selection = set()
        for index, group in times.items():
            spline = self._splines[index]
            if spline is None:
                continue
            for time in group:
                for candidate in (time + delta, time):
                    knot = spline.GetKnot(candidate)
                    if knot is not None:
                        selection.add((index, knot.GetTime()))
                        break
        self._selection = selection
        self._Sync()
        self.canvas.update()

    def SetKeyValue(self, value):
        """Set every selected key to `value` as one step (Maya's Value)."""
        times = self.SelectedTimes()
        if not times:
            return False
        return self._Apply(
            sorted(times), "Set Key Value",
            lambda index, spline: graphModel.SetKeyValues(
                spline, times[index], value))

    def FrameAll(self):
        self.canvas.FrameAll()
        self._framed = True

    def FrameSelected(self):
        self.canvas.FrameSelected()
        self._framed = True

    def ScrubTo(self, time):
        """
        Move usdview's playhead. appController.setFrame snaps to the
        nearest frame in the timeline and updates the slider, the frame
        field and the viewport; the data model alone would move only the
        value and leave usdview's own widgets behind.
        """
        frame = graphModel.SnapTime(time)
        start, end = self.StageRange()
        if start is not None and end is not None:
            frame = max(min(frame, end), start)
        controller = getattr(self.usdviewApi,
                             "_UsdviewApi__appController", None)
        scrubbed = False
        if controller is not None and hasattr(controller, "setFrame"):
            try:
                controller.setFrame(frame)
                scrubbed = True
            except Exception:
                scrubbed = False
        if not scrubbed:
            self.usdviewApi.dataModel.currentFrame = Usd.TimeCode(frame)
        self._frame = frame
        self.canvas.update()

    def Undo(self):
        if self.undoStack is None or not self.undoStack.Undo():
            return False
        return True

    def Redo(self):
        if self.undoStack is None or not self.undoStack.Redo():
            return False
        return True

    # -- keys -----------------------------------------------------------

    def OwnsEvent(self, receiver):
        """Whether a key event belongs to this window."""
        if isinstance(receiver, QtWidgets.QWidget):
            return receiver.window() is self
        return QtWidgets.QApplication.activeWindow() is self

    def HandleKey(self, key, modifiers=None):
        """One of the editor's hotkeys; True when it was acted on."""
        if modifiers is not None and modifiers & (
                QtCore.Qt.ControlModifier | QtCore.Qt.AltModifier
                | QtCore.Qt.MetaModifier):
            return False
        if key == QtCore.Qt.Key_Escape:
            if self.AbortGesture():
                return True
            if self._selection:
                self.SetSelection([])
            return True
        if key == QtCore.Qt.Key_A:
            self.FrameAll()
            return True
        if key == QtCore.Qt.Key_F:
            self.FrameSelected()
            return True
        if key == QtCore.Qt.Key_Home:
            self.canvas.FrameStageRange()
            self._framed = True
            return True
        if key == QtCore.Qt.Key_I:
            self.InsertKeyAtCurrentFrame()
            return True
        if key in (QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace):
            self.DeleteSelectedKeys()
            return True
        return False

    # -- slots ----------------------------------------------------------

    def _onFrameAll(self):
        self.FrameAll()

    def _onFrameSelected(self):
        self.FrameSelected()

    def _onInsertKey(self):
        self.InsertKeyAtCurrentFrame()

    def _onDeleteKeys(self):
        self.DeleteSelectedKeys()

    def _onBreak(self):
        self.Break()

    def _onUnify(self):
        self.Unify()

    def _onSide(self, checked, side):
        if checked and not self._updating:
            self._tangentSide = side

    def _onWeighted(self, checked):
        self._weighted = bool(checked)

    def _onSnapFrames(self, checked):
        self._snapFrames = bool(checked)

    def _onPreInfinityChanged(self, index):
        # ONLY the pre side. Passing both combos' values would push the
        # displayed pre mode -- which comes from ONE curve, the focus
        # one -- onto every other curve every time the post combo moved.
        del index
        if not self._updating:
            self.SetInfinity(pre=self.preInfinity.currentData())

    def _onPostInfinityChanged(self, index):
        del index
        if not self._updating:
            self.SetInfinity(post=self.postInfinity.currentData())

    def _onTimeEdited(self):
        if self._updating or self.timeBox.IsBlank():
            return
        self.SetKeyTime(self.timeBox.value())

    def _onValueEdited(self):
        if self._updating or self.valueBox.IsBlank():
            return
        self.SetKeyValue(self.valueBox.value())

    def _onPrimSelectionChanged(self, added=None, removed=None):
        del added, removed
        self.RefreshCurves()

    def _onPropSelectionChanged(self):
        self.RefreshCurves()

    def _onStageReplaced(self):
        self.AbortGesture()
        self._curves = []
        self._splines = []
        self._selection = set()
        self._hidden = set()
        self._isolated = set()
        self._framed = False
        self._frame = _FrameValue(self.usdviewApi.frame)
        self._ObserveStage(self.usdviewApi.dataModel.stage)
        self.RefreshCurves()

    def _onFrameChanged(self, frame):
        # The SIGNAL's frame, never dataModel.currentFrame: the setter
        # emits before it assigns, so re-reading the property here would
        # leave the playhead a scrub behind (rigExecUsdview._FrameValue).
        self._frame = _FrameValue(frame)
        self.canvas.update()
        self._Sync()

    def _onUndoStackChanged(self):
        # An undo restores layer specs behind our back; the panel's
        # splines are stale until they are read again.
        if self._gesture is not None:
            return
        self._ReadSplines()
        self._PruneSelection()
        self.canvas.update()
        self._Sync()
        self.usdviewApi.UpdateViewport()

    def _ObserveStage(self, stage):
        if self._noticeKey is not None:
            try:
                self._noticeKey.Revoke()
            except Exception:
                pass
            self._noticeKey = None
        if stage:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._onObjectsChanged, stage)

    def _onObjectsChanged(self, notice, stage):
        """
        Re-read a listed curve whose attribute was authored elsewhere,
        so a gizmo drag shows up in the graph live (spec 2.5).

        Not during our own gesture: the drag already holds the authored
        spline, and re-reading mid-drag would fight it.
        """
        del stage
        if self._gesture is not None or not self._curves:
            return
        paths = set(notice.GetChangedInfoOnlyPaths())
        paths.update(notice.GetResyncedPaths())
        if not paths:
            return
        for ref in self._curves:
            attrPath = ref.attrPath
            for path in paths:
                if path == attrPath or attrPath.HasPrefix(path):
                    self._ReadSplines()
                    self._PruneSelection()
                    self.canvas.update()
                    self._Sync()
                    return

    def _PruneSelection(self):
        """Drop selected keys that no longer exist on the stage."""
        kept = set()
        for index, time in self._selection:
            if index >= len(self._splines):
                continue
            spline = self._splines[index]
            if spline is not None and spline.GetKnot(time) is not None:
                kept.add((index, time))
        self._selection = kept

    # -- status ---------------------------------------------------------

    def Status(self):
        """One line about what the editor is showing (spec 2.1)."""
        if not self.Stage():
            return "No stage."
        if not self._curves:
            return ("No spline-capable attributes in the selection. "
                    "Select a rig control, or a scalar attribute in the "
                    "property browser.")
        visible = len(self.VisibleIndices())
        text = "%d curve%s (%d shown)" % (len(self._curves),
                                          "" if len(self._curves) == 1
                                          else "s", visible)
        if self._selection:
            text += ", %d key%s selected" % (
                len(self._selection), "" if len(self._selection) == 1
                else "s")
        else:
            text += ", no keys selected"
        text += "  frame %g" % self._frame
        if self._gesture is not None:
            text += "  [%s]" % self._gesture.label
        if self._notice:
            text += "  --  %s" % self._notice
        return text

    def _Notify(self, message):
        """
        Say why a command did nothing, on the status line.

        The message survives exactly until the next thing the panel
        does: a refusal an artist never sees is a refusal that reads as
        a broken button, and one that outlives its cause reads as a
        broken editor.
        """
        self._notice = message or ""
        self.statusLabel.setText(self.Status())

    def _Sync(self):
        """Mirror the model onto the key stats, the status and the undo."""
        self._updating = True
        try:
            # A refusal explains the command that has just been refused,
            # so the next thing the panel does retires it.
            self._notice = ""
            self._SyncKeyStats()
            self._SyncInfinityCombos()
            self.statusLabel.setText(self.Status())
            if self._ownsUndoActions and self.undoStack is not None:
                self.undoAction.setEnabled(self.undoStack.CanUndo())
                self.redoAction.setEnabled(self.undoStack.CanRedo())
        finally:
            self._updating = False

    def _SyncKeyStats(self):
        times, values = [], []
        for index, time in self._selection:
            times.append(time)
            spline = self._splines[index] if index < len(self._splines) \
                else None
            knot = spline.GetKnot(time) if spline is not None else None
            if knot is not None:
                values.append(float(knot.GetValue()))
        if len(set(times)) == 1:
            self.timeBox.SetNumber(times[0])
        else:
            self.timeBox.SetBlank()
        if values and max(values) - min(values) < 1e-12:
            self.valueBox.SetNumber(values[0])
        else:
            self.valueBox.SetBlank()

    def FocusCurveIndex(self):
        """
        The curve the key stats and the Infinity combos DISPLAY.

        The lowest-indexed curve holding a selected key, else the first
        visible curve with keys. A knotless channel is never the focus:
        its extrapolation is Held by construction and showing that would
        say the artist's cycle had been lost.
        """
        animated = self.AnimatedIndices()
        if not animated:
            return None
        selected = sorted(index for index, _ in self._selection
                          if index in animated)
        return selected[0] if selected else animated[0]

    def _SyncInfinityCombos(self):
        """
        Show the focus curve's infinities, WITHOUT writing.

        Setting a combo's index fires its slot, so this only ever runs
        under the `_updating` guard `_Sync` holds; otherwise merely
        looking at a second curve would author the first one's modes
        onto it.
        """
        index = self.FocusCurveIndex()
        spline = self._splines[index] if index is not None else None
        if spline is None:
            return
        for combo, getter in ((self.preInfinity, spline.GetPreExtrapolation),
                              (self.postInfinity,
                               spline.GetPostExtrapolation)):
            try:
                name = graphModel.EXTRAP_NAMES.get(getter().mode)
            except Exception:
                name = None
            if name is None:
                continue
            item = combo.findData(name)
            if item >= 0 and item != combo.currentIndex():
                combo.setCurrentIndex(item)

    # -- window ---------------------------------------------------------

    def showEvent(self, event):
        super(GraphEditorPanel, self).showEvent(event)
        self.canvas.setFocus()
        if not self._framed:
            self.FrameAll()


def _SwatchIcon(rgb, size=10):
    """A solid colour chip for the curve list's first column."""
    pixmap = QtGui.QPixmap(size, size)
    pixmap.fill(_Color(rgb))
    return QtGui.QIcon(pixmap)


def _FrameValue(frame):
    """A Usd.TimeCode or number as a plain float frame."""
    if isinstance(frame, Usd.TimeCode):
        return 0.0 if frame.IsDefault() else frame.GetValue()
    try:
        return float(frame)
    except (TypeError, ValueError):
        return 0.0


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def OpenGraphEditor(usdviewApi, undoStack=None):
    """
    Show the single Graph Editor for this session, creating it once.

    `undoStack` is the shared rigExecUndo.UndoStack the viewport gizmos
    push onto, so Ctrl+Z undoes graph and viewport edits from one
    history (spec 2.5).
    """
    if undoStack is None:
        undoStack = rigExecUndo.UndoStack()
    panel = GraphEditorPanel.GetInstance(usdviewApi, undoStack)
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel


def GetGraphEditor():
    """The open Graph Editor, or None."""
    return GraphEditorPanel.Instance()
