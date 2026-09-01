#
# RigExec usdview plugin: the viewport manipulator toolbar -- Maya-style
# Move / Rotate / Scale gizmos over the stage view, undoable through the
# shared rigExecUndo stack.
#
# Layering, and why: everything that can be decided without Qt already
# was. gizmoMath owns "what does this world delta do to the channels",
# gizmoScreen owns "where is the handle and what did the mouse mean",
# gizmoSettings owns "what are the tool's options". This file is the Qt
# shell: a toolbar, a transparent overlay that paints projected handles,
# a settings window, and the event filter that turns a drag into
# Apply* calls bracketed by an EditRecorder.
#
# OVERLAY APPROACH: a transparent, mouse-transparent CHILD WIDGET of the
# StageView, raised above it. usdview's StageView is a QOpenGLWidget,
# whose contents Qt 6 composites through the backing store, so ordinary
# child widgets do paint on top of it -- verified by the testusdview
# smoke grabs in .superpowers/sdd/2026-09-01-viewport-gizmo-toolbar/.
# The fallback of wrapping view.paintGL and drawing with a QPainter on
# the GL widget was NOT needed and is not implemented; if a future Qt or
# driver breaks the composite, that is where to go.
#
# HOTKEYS AND usdview (design spec 8.5). Two things in a usdview window
# get at a key before the stage view does: the Qt.ApplicationShortcut
# actions in mainWindowUI.ui, and the application-wide AppEventFilter.
# Neither key below is given up, but both are shared:
#   J  -- "Toggle Framed View" (actionToggle_Framed_View, connected,
#         application-wide). Maya's J is hold-to-step-snap and only
#         means anything WHILE dragging, so a live drag claims it in
#         ShortcutOverride and the rest of the time J still toggles the
#         framed view.
#   Escape -- appEventFilter.py:113 swallows every Escape KeyPress to
#         reset focus from the mouse position, so a KeyPress for it
#         never reaches the view. Same treatment as J: a live drag
#         claims it in ShortcutOverride and aborts there.
#   W  -- "Watch Window" (actionWatch_Window) is declared but is
#         disabled and connected to nothing, so its shortcut never
#         fires and W is free for the Move tool.
#   Q, E, R, D, X, Insert, +, -, =, Shift+Z, Ctrl+Y, Ctrl+Z and
#         Ctrl+Shift+Z are unclaimed (usdview uses Ctrl+R, Ctrl+D,
#         Ctrl+Q, Ctrl++, Ctrl+= and Ctrl+-, which do not collide).
# Undo / redo are Qt.ApplicationShortcut so they work wherever focus is;
# the tool keys are read from the stage view's event filter so they
# cannot shadow a text field.
#
# HOVER uses Qt.WA_Hover on the stage view rather than mouse tracking:
# see GizmoController.__init__.
#
import math
import os
import sys

from pxr import Gf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    # PySide6 keeps QAction / QActionGroup in QtGui; usdview's qt shim
    # re-exports whichever module has them as QtActionWidgets.
    from pxr.Usdviewq.qt import QtActionWidgets
except ImportError:                                       # PySide2
    QtActionWidgets = QtWidgets

try:
    import gizmoMath
    import gizmoScreen
    import gizmoSettings
    import rigExecUndo
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoMath
    import gizmoScreen
    import gizmoSettings
    import rigExecUndo


TOOL_SELECT = gizmoSettings.TOOL_SELECT
TOOL_TRANSLATE = gizmoScreen.TOOL_TRANSLATE
TOOL_ROTATE = gizmoScreen.TOOL_ROTATE
TOOL_SCALE = gizmoScreen.TOOL_SCALE

# Maya's names, not USD's: an animator reaches for "Move", not
# "Translate". The token stays gizmoScreen's so one spelling reaches the
# geometry code.
TOOL_LABELS = {
    TOOL_SELECT: "Select",
    TOOL_TRANSLATE: "Move",
    TOOL_ROTATE: "Rotate",
    TOOL_SCALE: "Scale",
}

# Undo labels ("Move HandIK"), which is what the toolbar's Undo tooltip
# shows, so they read the same as the tool buttons.
_EDIT_VERBS = {
    TOOL_TRANSLATE: "Move",
    TOOL_ROTATE: "Rotate",
    TOOL_SCALE: "Scale",
}

# Line width of every handle outline, in LOGICAL pixels (design 8.1).
LINE_WIDTH = 2.0

# Opacity of the pieces the spec dims rather than drops: a locked axis
# (pointing at the camera), a planar square's fill, the rotation pie and
# the free-rotate ball.
LOCKED_OPACITY = 0.4
PLANE_FILL_OPACITY = 0.5
PIE_OPACITY = 0.3
SPHERE_OPACITY = 0.5
# The silhouette is drawn at SPHERE_OPACITY per the spec; the interior
# gets a lighter wash, enough for the ball to read as a disc without
# hiding the three rings drawn inside it. It needs to be there at all
# because the silhouette lands exactly on the axis ring of the same
# radius and is covered by it whenever that ring faces the camera.
SPHERE_FILL_OPACITY = 0.30

# An arrowhead this many times its base radius long, which is the
# proportion Maya's move cones use.
CONE_LENGTH_RATIO = 3.0

# A ray/plane delta more than this many times the camera-plane delta for
# the same mouse travel is the intersection blowing up on a plane that
# has gone nearly edge-on since the press. gizmoScreen guards the
# obvious case up front; this catches the plane that tips over mid-drag.
_PLANE_DELTA_SANITY = 50.0

# Longest status line the toolbar shows before truncating; the whole
# text stays on the label's tooltip. A QToolBar sizes itself to its
# contents, so an unbounded label would push the tool buttons off.
_STATUS_CHARS = 72

# How near a click has to land, in LOGICAL pixels, before it counts as
# hitting a handle outline.
HIT_PIXELS = gizmoScreen.HIT_PIXELS

_CAMERA_MODIFIERS = (QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier)


def StageView(usdviewApi):
    """
    usdview's stage view widget, or None in a headless / pre-view state.

    The private-name mangling is usdview's own (UsdviewApi keeps the app
    controller as __appController); curvenetUI.SurfacePicker reaches it
    the same way.
    """
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


def _Color(rgb, opacity=1.0):
    return QtGui.QColor(int(round(rgb[0] * 255)), int(round(rgb[1] * 255)),
                        int(round(rgb[2] * 255)),
                        int(round(max(0.0, min(1.0, opacity)) * 255)))


def _Linear(matrix):
    """The rotation part of an orthonormal frame, as a Gf.Matrix3d."""
    return matrix.ExtractRotationMatrix()


def _ToFrame(frame, worldVector):
    """A world direction in `frame`'s axes (frame is orthonormal)."""
    return Gf.Vec3d(worldVector) * _Linear(frame).GetTranspose()


def _FromFrame(frame, localVector):
    return Gf.Vec3d(localVector) * _Linear(frame)


class _PlaneAxes(object):
    """
    An axis selector that means "these two axes" for Target.ApplyScale.

    Maya's planar scale handles change TWO channels at once, and
    Target.ApplyScale takes a single axis index or None for uniform: it
    recomputes all three values from the drag base on every call, so two
    successive single-axis calls cannot express it (the second undoes
    the first). Every ApplyScale implementation asks `axisIndex is None
    or axisIndex == i`, so an object whose __eq__ answers for a SET of
    indices expresses the missing case exactly, without reaching into
    the target's private base values.

    This is a workaround for a gap in the gizmoMath interface, not a
    pattern to copy: the right fix is an `axisIndices` argument on
    ApplyScale, and this class disappears when that lands.
    """

    def __init__(self, indices):
        self._indices = frozenset(indices)

    def __eq__(self, other):
        return other in self._indices

    def __ne__(self, other):
        return other not in self._indices

    def __hash__(self):
        return hash(self._indices)

    def __repr__(self):
        return "<axes %s>" % sorted(self._indices)


class _Drag(object):
    """
    One live manipulation: what was grabbed, where, and what has been
    written so far.

    `handle` is the handle AS IT WAS AT THE PRESS and stays frozen for
    the whole drag. A rotate drag must turn about the axis the artist
    grabbed even as the object (and, in Object orientation, the ring)
    turns underneath; recomputing the axis from the redrawn handles
    would make the manipulator chase itself.
    """

    def __init__(self, tool, handle, recorder, target, press, camera,
                 viewport, origin2d):
        self.tool = tool
        self.handle = handle
        self.recorder = recorder
        self.target = target
        self.press = press
        self.current = press
        self.camera = camera
        self.viewport = viewport
        self.origin2d = origin2d
        # Rotation bookkeeping: `raw` is the last wrapped angle from
        # RotationDragAngle, `angle` the accumulated (and snapped) total
        # actually applied, `startParameter` where on the ring the press
        # landed so the pie slice can start there.
        self.raw = 0.0
        self.total = 0.0
        self.angle = 0.0
        self.startParameter = 0.0
        # Free rotate composes each step into one running world rotation
        # applied from the drag base, so a curved drag rolls the ball
        # instead of snapping back to a single press-to-cursor axis.
        self.trackballLast = press
        self.trackball = Gf.Matrix4d(1.0)
        self.lastDelta = None
        # Maya's Ctrl+axis "move in the perpendicular plane", read from
        # every event rather than only the press: on macOS Qt turns a
        # Ctrl+left CLICK into a right-button press, so the reachable
        # gesture is to grab the axis first and then hold Ctrl.
        self.ctrl = False
        # True when this ring was drawn on a GIMBAL axis, which changes
        # which Target entry point the drag writes through -- see
        # GizmoController._ApplyRotate.
        self.gimbal = False


# ---------------------------------------------------------------------------
# Overlay
# ---------------------------------------------------------------------------

class GizmoOverlay(QtWidgets.QWidget):
    """
    The manipulator, painted over the stage view.

    A child widget rather than session-layer prims (design assumption
    1.7): the gizmo has to be screen-constant and unoccluded, and
    authoring geometry for it would re-enter the evaluator on every
    camera move. Mouse-transparent, because the controller's event
    filter on the stage view is what does the picking -- the overlay
    must never take a click that was meant for usdview.
    """

    def __init__(self, controller, parent):
        super(GizmoOverlay, self).__init__(parent)
        self._controller = controller
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
        self.setAutoFillBackground(False)
        self.setFocusPolicy(QtCore.Qt.NoFocus)

    # -- geometry -------------------------------------------------------

    def _Ratio(self):
        parent = self.parentWidget()
        try:
            return float(parent.devicePixelRatioF()) if parent else 1.0
        except AttributeError:
            return 1.0

    def _Point(self, point, ratio):
        """A physical-pixel handle point as a logical QPointF."""
        return QtCore.QPointF(point[0] / ratio, point[1] / ratio)

    def _Polygon(self, points, ratio):
        return QtGui.QPolygonF([self._Point(p, ratio) for p in points])

    # -- painting -------------------------------------------------------

    def paintEvent(self, event):
        handles = self._controller.Handles()
        if not handles:
            return
        ratio = self._Ratio()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        try:
            tool = self._controller.Tool()
            # The free-rotate ball first so the rings sit on top of it,
            # then rings, then everything else: the spec's draw order is
            # also the hit order, so what looks on top is what is picked.
            for handle in handles:
                if handle.kind == "sphere":
                    self._DrawSphere(painter, handle, ratio)
            for handle in handles:
                if handle.kind in ("ring", "view"):
                    self._DrawRing(painter, handle, ratio)
            self._DrawPie(painter, ratio)
            for handle in handles:
                if handle.kind == "axis":
                    if tool == TOOL_TRANSLATE:
                        self._DrawArrow(painter, handle, ratio)
                    else:
                        self._DrawScaleAxis(painter, handle, ratio)
            for handle in handles:
                if handle.kind == "plane":
                    self._DrawPlane(painter, handle, ratio)
            for handle in handles:
                if handle.kind == "center":
                    self._DrawCenter(painter, handle, ratio)
        finally:
            painter.end()

    def _Pen(self, handle, width=LINE_WIDTH):
        pen = QtGui.QPen(self._HandleColor(handle))
        pen.setWidthF(width)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        return pen

    def _HandleColor(self, handle, opacity=1.0):
        """
        Maya's three states: the handle under the cursor is the pale
        pre-selection highlight, the last-dragged handle stays yellow
        (middle-drag repeats it), and an ungrabbable one is its own
        colour dimmed so the artist can see it is there and inert.
        """
        controller = self._controller
        if not handle.grabbable:
            return _Color(handle.color, LOCKED_OPACITY * opacity)
        if controller.IsDragging() and \
                controller.SelectedHandleName() == handle.name:
            return _Color(gizmoScreen.COLOR_SELECTED, opacity)
        if controller.HoverHandleName() == handle.name:
            return _Color(gizmoScreen.COLOR_HOVER, opacity)
        if controller.SelectedHandleName() == handle.name:
            return _Color(gizmoScreen.COLOR_SELECTED, opacity)
        return _Color(handle.color, opacity)

    def _DrawArrow(self, painter, handle, ratio):
        """A move axis: a line stopping short of a filled cone tip."""
        start = self._Point(handle.points[0], ratio)
        end = self._Point(handle.points[-1], ratio)
        dx, dy = end.x() - start.x(), end.y() - start.y()
        length = math.hypot(dx, dy)
        radius = gizmoScreen.CONE_RADIUS * handle.sizePixels / ratio
        coneLength = min(radius * CONE_LENGTH_RATIO, length * 0.9)
        painter.setPen(self._Pen(handle))
        painter.setBrush(QtCore.Qt.NoBrush)
        if length < 1e-6:
            return
        ux, uy = dx / length, dy / length
        base = QtCore.QPointF(end.x() - ux * coneLength,
                              end.y() - uy * coneLength)
        painter.drawLine(start, base)
        painter.setBrush(QtGui.QBrush(self._HandleColor(handle)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawPolygon(QtGui.QPolygonF([
            end,
            QtCore.QPointF(base.x() - uy * radius, base.y() + ux * radius),
            QtCore.QPointF(base.x() + uy * radius, base.y() - ux * radius)]))

    def _DrawScaleAxis(self, painter, handle, ratio):
        """A scale axis: a line ending in a filled cube (a square)."""
        start = self._Point(handle.points[0], ratio)
        end = self._Point(handle.points[-1], ratio)
        side = gizmoScreen.CUBE_SIDE * handle.sizePixels / ratio
        painter.setPen(self._Pen(handle))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawLine(start, end)
        painter.setBrush(QtGui.QBrush(self._HandleColor(handle)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRect(QtCore.QRectF(end.x() - side * 0.5,
                                       end.y() - side * 0.5, side, side))

    def _DrawPlane(self, painter, handle, ratio):
        polygon = self._Polygon(handle.points, ratio)
        painter.setBrush(QtGui.QBrush(
            self._HandleColor(handle, PLANE_FILL_OPACITY)))
        painter.setPen(self._Pen(handle, LINE_WIDTH * 0.75))
        painter.drawPolygon(polygon)

    def _DrawCenter(self, painter, handle, ratio):
        point = self._Point(handle.points[0], ratio)
        side = gizmoScreen.CENTER_SIDE * handle.sizePixels / ratio
        painter.setBrush(QtGui.QBrush(self._HandleColor(handle)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRect(QtCore.QRectF(point.x() - side * 0.5,
                                       point.y() - side * 0.5, side, side))

    def _DrawRing(self, painter, handle, ratio):
        """
        Only frontPoints: Maya hides the half of each ring behind the
        ring centre so three overlapping circles stay tellable apart,
        and a fully visible run already repeats its first point, so one
        drawPolyline closes it without a chord across the manipulator.
        """
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(self._Pen(handle))
        for run in handle.frontPoints:
            if len(run) > 1:
                painter.drawPolyline(self._Polygon(run, ratio))

    def _DrawSphere(self, painter, handle, ratio):
        point = self._Point(handle.points[0], ratio)
        radius = handle.radiusPixels / ratio
        painter.setBrush(QtGui.QBrush(
            _Color(gizmoScreen.COLOR_SPHERE, SPHERE_FILL_OPACITY)))
        pen = QtGui.QPen(_Color(gizmoScreen.COLOR_SPHERE, SPHERE_OPACITY))
        pen.setWidthF(LINE_WIDTH)
        painter.setPen(pen)
        painter.drawEllipse(point, radius, radius)

    def _DrawPie(self, painter, ratio):
        """
        Maya's rotation-amount wedge, from where the ring was grabbed to
        where the sweep has reached.
        """
        wedge = self._controller.PieSlice()
        if not wedge:
            return
        polygon, color = wedge
        if len(polygon) < 3:
            return
        painter.setBrush(QtGui.QBrush(_Color(color, PIE_OPACITY)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawPolygon(self._Polygon(polygon, ratio))


# ---------------------------------------------------------------------------
# Toolbar
# ---------------------------------------------------------------------------

class ViewportToolbar(QtWidgets.QToolBar):
    """
    The strip above the viewport: tools, channel set, write mode, undo,
    the Tool Settings button and the status label that always says why
    there is no gizmo.
    """

    def __init__(self, controller, parent=None):
        super(ViewportToolbar, self).__init__("RigExec Viewport Tools",
                                              parent)
        self._controller = controller
        self.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        self.setMovable(False)
        self.setFloatable(False)
        # The default style barely distinguishes a checked tool button,
        # and "which tool am I in" is the one thing this bar must say.
        self.setStyleSheet(
            "QToolButton:checked { background: #4879b4; color: white;"
            " border: 1px solid #79a6dc; border-radius: 3px; }")

        self._toolActions = {}
        self._channelActions = {}
        self._writeActions = {}
        self._BuildTools()
        self.addSeparator()
        self._BuildChannels()
        self.addSeparator()
        self._BuildWrite()
        self.addSeparator()
        self._BuildUndo()
        self.addSeparator()
        self._BuildSettingsButton()

        # No expanding spacer before the label: QToolBar moves whatever
        # does not fit into an overflow menu, and a right-aligned status
        # label is the first thing to disappear on a narrow viewport --
        # which is exactly when an artist needs to read it.
        self._status = QtWidgets.QLabel("")
        self._status.setToolTip(
            "What the gizmo is editing, or why there is none.")
        # The text is truncated in Sync() rather than given a shrinking
        # size policy: a QToolBar hands an Ignored-policy widget zero
        # width, and a status line that is there but empty is worse than
        # one that is short. A very narrow viewport still moves it into
        # the toolbar's overflow chevron, which is Qt's own doing.
        self.addWidget(self._status)

    def _AddChecked(self, group, text, tooltip, checked, handler):
        action = QtActionWidgets.QAction(text, self)
        action.setCheckable(True)
        action.setChecked(checked)
        action.setToolTip(tooltip)
        group.addAction(action)
        self.addAction(action)
        action.triggered.connect(handler)
        return action

    def _BuildTools(self):
        group = QtActionWidgets.QActionGroup(self)
        group.setExclusive(True)
        tips = {
            TOOL_SELECT: "Select (Q): no manipulator; usdview picking "
                         "only.",
            TOOL_TRANSLATE: "Move (W): drag an axis, a planar square or "
                            "the light-blue centre. Ctrl+axis moves in "
                            "the perpendicular plane.",
            TOOL_ROTATE: "Rotate (E): drag a ring, the outer view ring, "
                         "or inside the ball to free-rotate.",
            TOOL_SCALE: "Scale (R): drag a cube for one axis, a planar "
                        "square for two, the centre cube for uniform.",
        }
        for tool in (TOOL_SELECT, TOOL_TRANSLATE, TOOL_ROTATE, TOOL_SCALE):
            self._toolActions[tool] = self._AddChecked(
                group, TOOL_LABELS[tool], tips[tool],
                tool == self._controller.Tool(),
                lambda checked=False, t=tool: self._onTool(t))

    def _BuildChannels(self):
        self.addWidget(QtWidgets.QLabel("Channels:"))
        group = QtActionWidgets.QActionGroup(self)
        group.setExclusive(True)
        tips = {
            gizmoMath.CHANNELS_POSE:
                "Pose: edit the animation channels (avars for a rig "
                "prim, the xformOps for a plain xform).",
            gizmoMath.CHANNELS_PIVOT:
                "Pivot (D / Insert): edit the rest offset a rig prim's "
                "avars ride on, or a plain xform's pivot.",
        }
        for channels, label in ((gizmoMath.CHANNELS_POSE, "Pose"),
                                (gizmoMath.CHANNELS_PIVOT, "Pivot")):
            self._channelActions[channels] = self._AddChecked(
                group, label, tips[channels],
                channels == self._controller.Channels(),
                lambda checked=False, c=channels: self._onChannels(c))

    def _BuildWrite(self):
        self.addWidget(QtWidgets.QLabel("Write:"))
        group = QtActionWidgets.QActionGroup(self)
        group.setExclusive(True)
        tips = {
            gizmoMath.WRITE_ANIMATION:
                "Animation: author at the current frame (a spline knot, "
                "or a time sample where one already exists).",
            gizmoMath.WRITE_DEFAULT:
                "Default: author the attribute's default value. A "
                "spline or time samples outrank it; the status label "
                "warns when that happens.",
        }
        for mode, label in ((gizmoMath.WRITE_ANIMATION, "Animation"),
                            (gizmoMath.WRITE_DEFAULT, "Default")):
            self._writeActions[mode] = self._AddChecked(
                group, label, tips[mode], mode == self._controller.WriteMode(),
                lambda checked=False, m=mode: self._onWrite(m))

    def _BuildUndo(self):
        # Qt.ApplicationShortcut so undo works with focus in the prim
        # tree or the attribute view, not only over the viewport.
        self.undoAction = QtActionWidgets.QAction("Undo", self)
        self.undoAction.setShortcut(QtGui.QKeySequence("Ctrl+Z"))
        self.undoAction.setShortcutContext(QtCore.Qt.ApplicationShortcut)
        self.undoAction.triggered.connect(
            lambda checked=False: self._controller.Undo())
        self.addAction(self.undoAction)

        self.redoAction = QtActionWidgets.QAction("Redo", self)
        # Ctrl+Shift+Z is the user's ask, Shift+Z is Maya's, Ctrl+Y is
        # what a Windows-trained hand reaches for.
        self.redoAction.setShortcuts([QtGui.QKeySequence("Ctrl+Shift+Z"),
                                      QtGui.QKeySequence("Shift+Z"),
                                      QtGui.QKeySequence("Ctrl+Y")])
        self.redoAction.setShortcutContext(QtCore.Qt.ApplicationShortcut)
        self.redoAction.triggered.connect(
            lambda checked=False: self._controller.Redo())
        self.addAction(self.redoAction)

    def _BuildSettingsButton(self):
        self.settingsAction = QtActionWidgets.QAction("Tool Settings…",
                                                      self)
        self.settingsAction.setToolTip(
            "Axis orientation, snapping and the manipulator size for "
            "the active tool.")
        self.settingsAction.triggered.connect(
            lambda checked=False: self._controller.ShowToolSettings())
        self.addAction(self.settingsAction)

    # -- slots ----------------------------------------------------------

    def _onTool(self, tool):
        self._controller.SetTool(tool)

    def _onChannels(self, channels):
        self._controller.SetChannels(channels)

    def _onWrite(self, mode):
        self._controller.SetWriteMode(mode)

    # -- refresh --------------------------------------------------------

    def Sync(self):
        """Mirror the controller's state onto the widgets."""
        controller = self._controller
        for tool, action in self._toolActions.items():
            action.setChecked(tool == controller.Tool())
        for channels, action in self._channelActions.items():
            action.setChecked(channels == controller.Channels())
        for mode, action in self._writeActions.items():
            action.setChecked(mode == controller.WriteMode())
        stack = controller.undoStack
        self.undoAction.setEnabled(stack.CanUndo())
        self.redoAction.setEnabled(stack.CanRedo())
        self.undoAction.setToolTip(
            "Undo %s (Ctrl+Z)" % stack.UndoText() if stack.CanUndo()
            else "Nothing to undo (Ctrl+Z)")
        self.redoAction.setToolTip(
            "Redo %s (Ctrl+Shift+Z, Shift+Z, Ctrl+Y)" % stack.RedoText()
            if stack.CanRedo()
            else "Nothing to redo (Ctrl+Shift+Z, Shift+Z, Ctrl+Y)")
        status = controller.Status()
        self._status.setToolTip(status)
        self._status.setText(status if len(status) <= _STATUS_CHARS
                             else status[:_STATUS_CHARS - 1] + "\u2026")

    def StatusLabel(self):
        return self._status


# ---------------------------------------------------------------------------
# Tool settings window
# ---------------------------------------------------------------------------

class ToolSettingsPanel(QtWidgets.QWidget):
    """
    Maya's Tool Settings for the active tool (design spec 8.6).

    One window per session, parented to usdview's main window exactly
    like VolumeWeightPanel, and rebuilt whenever the tool changes so it
    only ever shows rows that mean something for the tool in hand.
    """

    __instance = None

    @classmethod
    def GetInstance(cls, controller):
        if cls.__instance is None:
            cls.__instance = ToolSettingsPanel(controller)
        return cls.__instance

    def __init__(self, controller):
        super(ToolSettingsPanel, self).__init__(
            controller.usdviewApi.qMainWindow, QtCore.Qt.WindowType.Window)
        self._controller = controller
        self._tool = None
        self._updating = False
        self.setWindowTitle("RigExec: Tool Settings")
        self.setGeometry(0, 0, 340, 300)

        outer = QtWidgets.QVBoxLayout()
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)
        self.setLayout(outer)

        self._title = QtWidgets.QLabel("")
        font = self._title.font()
        font.setBold(True)
        self._title.setFont(font)
        outer.addWidget(self._title)

        self._body = QtWidgets.QWidget()
        self._form = QtWidgets.QFormLayout()
        self._form.setContentsMargins(0, 0, 0, 0)
        self._body.setLayout(self._form)
        outer.addWidget(self._body)
        outer.addStretch(1)

        self._resetButton = QtWidgets.QPushButton("Reset Tool")
        self._resetButton.setToolTip(
            "Restore this tool's Maya defaults.")
        self._resetButton.clicked.connect(self._onReset)
        outer.addWidget(self._resetButton)

        self.Rebuild()

    # -- construction ---------------------------------------------------

    def _Clear(self):
        while self._form.count():
            item = self._form.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()

    def Rebuild(self):
        """
        Build the rows for the active tool. Called on every tool change;
        cheap enough to do wholesale, and wholesale is what keeps a
        stale Rotate-only row from surviving into the Scale tool.
        """
        controller = self._controller
        tool = controller.Tool()
        self._tool = tool
        self._title.setText("%s Tool" % TOOL_LABELS.get(tool, tool))
        self._Clear()
        if tool == TOOL_SELECT:
            note = QtWidgets.QLabel(
                "The Select tool has no manipulator. Pick a prim in the "
                "viewport, then choose Move, Rotate or Scale.")
            note.setWordWrap(True)
            self._form.addRow(note)
        else:
            self._orientation = QtWidgets.QComboBox()
            self._orientation.setToolTip(
                "Which axes the handles are drawn along. The values "
                "written are always this prim's own channels.")
            for choice in gizmoSettings.OrientationChoices(tool):
                self._orientation.addItem(
                    gizmoSettings.OrientationLabel(choice), choice)
            self._orientation.currentIndexChanged.connect(
                self._onOrientation)
            self._form.addRow("Axis Orientation", self._orientation)

            self._stepSnap = QtWidgets.QCheckBox("Step Snap")
            self._stepSnap.setToolTip(
                "Quantise the drag to whole steps, relative to where it "
                "started. Hold J for the same thing during one drag.")
            self._stepSnap.toggled.connect(self._onStepSnap)
            self._form.addRow("", self._stepSnap)

            self._stepSize = QtWidgets.QDoubleSpinBox()
            self._stepSize.setRange(0.0001, 100000.0)
            self._stepSize.setDecimals(4)
            self._stepSize.setSingleStep(
                5.0 if tool == TOOL_ROTATE else 0.5)
            self._stepSize.setToolTip(
                "Degrees per step." if tool == TOOL_ROTATE
                else "Units per step.")
            self._stepSize.valueChanged.connect(self._onStepSize)
            self._form.addRow("Step Size", self._stepSize)

            if tool == TOOL_ROTATE:
                self._freeRotate = QtWidgets.QCheckBox("Free Rotate")
                self._freeRotate.setToolTip(
                    "Draw the trackball, and let a drag inside it turn "
                    "the object about any axis.")
                self._freeRotate.toggled.connect(self._onFreeRotate)
                self._form.addRow("", self._freeRotate)

            if tool == TOOL_SCALE:
                self._preventNegative = QtWidgets.QCheckBox(
                    "Prevent Negative Scale")
                self._preventNegative.setToolTip(
                    "Stop a drag through the origin from mirroring the "
                    "object; the scale is clamped just above zero.")
                self._preventNegative.toggled.connect(
                    self._onPreventNegative)
                self._form.addRow("", self._preventNegative)

            self._preserveChildren = QtWidgets.QCheckBox(
                "Preserve Children")
            self._preserveChildren.toggled.connect(self._onPreserveChildren)
            self._form.addRow("", self._preserveChildren)

        self._size = QtWidgets.QDoubleSpinBox()
        self._size.setRange(gizmoSettings.MANIPULATOR_SIZE_MIN,
                            gizmoSettings.MANIPULATOR_SIZE_MAX)
        self._size.setDecimals(0)
        self._size.setSingleStep(10.0)
        self._size.setToolTip(
            "How big the manipulator is drawn, in pixels. + and - over "
            "the viewport change it by 10%.")
        self._size.valueChanged.connect(self._onSize)
        self._form.addRow("Manipulator Size", self._size)

        self.Sync()

    # -- refresh --------------------------------------------------------

    def Sync(self):
        """
        Push the model's values onto the widgets.

        `_updating` guards the round trip: every widget writes to the
        settings, the settings notify the controller, and the controller
        syncs the panel back. Without the guard a spin box would fight
        its own clamped value.
        """
        controller = self._controller
        if controller.Tool() != self._tool:
            self.Rebuild()
            return
        settings = controller.settings.For(self._tool)
        self._updating = True
        try:
            if self._tool != TOOL_SELECT:
                index = self._orientation.findData(settings.orientation)
                if index >= 0:
                    self._orientation.setCurrentIndex(index)
                self._stepSnap.setChecked(bool(settings.stepSnap))
                self._stepSize.setValue(float(settings.stepSize))
                if self._tool == TOOL_ROTATE:
                    self._freeRotate.setChecked(bool(settings.freeRotate))
                if self._tool == TOOL_SCALE:
                    self._preventNegative.setChecked(
                        bool(settings.preventNegativeScale))
                self._SyncPreserveChildren(settings)
            self._size.setValue(float(controller.settings.manipulatorSize))
        finally:
            self._updating = False

    def _SyncPreserveChildren(self, settings):
        """
        The checkbox is disabled, and says why, for a target that cannot
        honour it -- a rig control's children are placed by the
        evaluator, so compensating them here would fight the rig.
        """
        target = self._controller.Target()
        supported = bool(target is not None
                         and target.supportsPreserveChildren)
        self._preserveChildren.setEnabled(supported)
        self._preserveChildren.setChecked(
            bool(settings.preserveChildren) and supported)
        if supported:
            skipped = self._controller._SkippedChildren()
            tip = ("Re-author the children after the drag so they keep "
                   "their world transforms.")
            if skipped:
                tip += "\n\nNot preserved:\n- " + "\n- ".join(skipped)
            self._preserveChildren.setToolTip(tip)
        elif target is None:
            self._preserveChildren.setToolTip("No editable prim selected.")
        else:
            self._preserveChildren.setToolTip(
                target.preserveChildrenReason
                or "Not available for this prim.")

    # -- slots ----------------------------------------------------------

    def _Settings(self):
        return self._controller.settings.For(self._tool)

    def _onOrientation(self, index):
        if self._updating or index < 0:
            return
        self._Settings().orientation = self._orientation.itemData(index)

    def _onStepSnap(self, checked):
        if not self._updating:
            self._Settings().stepSnap = bool(checked)

    def _onStepSize(self, value):
        if not self._updating:
            self._Settings().stepSize = float(value)

    def _onFreeRotate(self, checked):
        if not self._updating:
            self._Settings().freeRotate = bool(checked)

    def _onPreventNegative(self, checked):
        if not self._updating:
            self._Settings().preventNegativeScale = bool(checked)

    def _onPreserveChildren(self, checked):
        if not self._updating:
            self._Settings().preserveChildren = bool(checked)

    def _onSize(self, value):
        if not self._updating:
            self._controller.settings.manipulatorSize = float(value)

    def _onReset(self):
        self._controller.settings.Reset(self._tool)
        self.Sync()


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class GizmoController(QtCore.QObject):
    """
    Owns the tool state, the edit target, the drag and the event filter.

    Everything the viewport does goes through here: the toolbar and the
    settings panel only set state, and the overlay only reads it.
    """

    def __init__(self, usdviewApi, undoStack, parent=None):
        super(GizmoController, self).__init__(parent)
        self.usdviewApi = usdviewApi
        self.undoStack = undoStack
        self.settings = gizmoSettings.GizmoSettings()

        self._tool = TOOL_SELECT
        self._channels = gizmoMath.CHANNELS_POSE
        self._writeMode = gizmoMath.WRITE_ANIMATION
        self._target = None
        self._reason = "no stage"
        self._warnings = []
        self._handles = []
        self._drag = None
        self._selected = None
        self._hover = None
        self._holdSnap = False
        self._holdGrid = False
        self._ctrl = False
        self._visible = True
        self._gimbalAxes = None
        self._rebuilding = False
        self._panel = None
        self._noticeKey = None
        self._frame = usdviewApi.frame

        view = StageView(usdviewApi)
        self._view = view
        self.overlay = GizmoOverlay(self, view) if view is not None else None
        self.toolbar = ViewportToolbar(self)

        self._InstallToolbar()
        if view is not None:
            view.installEventFilter(self)
            # WA_Hover, not setMouseTracking: Maya's pre-selection
            # highlight needs mouse moves with no button down, and Qt
            # delivers those to a widget only if it tracks the mouse --
            # which for usdview's stage view would also turn on a GPU
            # pickObject per mouse move (stageView.mouseMoveEvent's
            # "none" camera mode). WA_Hover produces HoverMove events
            # instead, which nothing else in usdview listens for.
            view.setAttribute(QtCore.Qt.WA_Hover, True)
            view.signalFrustumChanged.connect(self._onFrustumChanged)
        self.settings.AddListener(self._onSettingsChanged)
        self.undoStack.AddListener(self._onUndoStackChanged)

        model = usdviewApi.dataModel
        model.selection.signalPrimSelectionChanged.connect(
            self._onSelectionChanged)
        model.signalStageReplaced.connect(self._onStageReplaced)
        model.currentFrameChanged.connect(self._onFrameChanged)
        self._ObserveStage(model.stage)

        self.RefreshTarget()
        self._SyncOverlay()

    # -- installation ---------------------------------------------------

    def _InstallToolbar(self):
        """
        Above the viewport, inside the frame that holds the stage view,
        so it scrolls and hides with the viewport rather than joining
        usdview's own menu bar.
        """
        try:
            layout = self.usdviewApi._UsdviewApi__appController._ui \
                .glFrame.layout()
        except AttributeError:
            layout = None
        if layout is None:
            return
        layout.insertWidget(0, self.toolbar)
        self.toolbar.Sync()

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

    # -- state ----------------------------------------------------------

    def Tool(self):
        return self._tool

    def SetTool(self, tool):
        if tool == self._tool:
            return
        self._AbortDrag()
        self._tool = tool
        self._selected = None
        self._hover = None
        self.RefreshTarget()
        self.toolbar.Sync()
        if self._panel is not None:
            self._panel.Rebuild()

    def Channels(self):
        return self._channels

    def SetChannels(self, channels):
        if channels == self._channels:
            return
        self._AbortDrag()
        self._channels = channels
        self.RefreshTarget()
        self.toolbar.Sync()

    def WriteMode(self):
        return self._writeMode

    def SetWriteMode(self, mode):
        if mode == self._writeMode:
            return
        self._AbortDrag()
        self._writeMode = mode
        self.RefreshTarget()
        self.toolbar.Sync()

    def Target(self):
        return self._target

    def Reason(self):
        return self._reason

    def Handles(self):
        return self._handles

    def IsDragging(self):
        return self._drag is not None

    def SelectedHandleName(self):
        return self._selected

    def HoverHandleName(self):
        return self._hover

    def DragAngle(self):
        """Degrees swept so far by a rotate drag; 0 when not rotating."""
        return self._drag.angle if self._drag is not None else 0.0

    def IsVisible(self):
        return self._visible

    def SetVisible(self, visible):
        visible = bool(visible)
        if visible == self._visible:
            return
        self._visible = visible
        self._AbortDrag()
        self.toolbar.setVisible(visible)
        if self.overlay is not None:
            self.overlay.setVisible(visible)
        self._RebuildHandles()

    # -- target and handles ---------------------------------------------

    def _Writer(self):
        return gizmoMath.Writer(self.usdviewApi.stage, self._frame,
                                self._writeMode)

    def _FocusPrim(self):
        """
        The one prim the gizmo edits (design assumption 3).

        usdview's focus prim, except that its selection model keeps the
        PSEUDO-ROOT focused when a prim is added to an already-cleared
        selection -- which is exactly what UsdviewApi.ClearPrimSelection
        followed by AddPrimToSelection does, and therefore what every
        script driving usdview does. Falling back to the last real prim
        in the selection makes the documented API work; a click in the
        viewport or the prim tree sets the focus properly and never
        reaches the fallback.
        """
        api = self.usdviewApi
        prim = api.prim
        if prim and not prim.IsPseudoRoot():
            return prim
        for candidate in reversed(list(api.selectedPrims or [])):
            if candidate and not candidate.IsPseudoRoot():
                return candidate
        return None

    def RefreshTarget(self):
        """
        Re-resolve the edit target from usdview's focus prim, then
        rebuild the handles and the status line.
        """
        stage = self.usdviewApi.stage
        prim = self._FocusPrim() if stage else None
        self._warnings = []
        if not stage:
            self._target, self._reason = None, "no stage"
        else:
            self._target, self._reason = gizmoMath.MakeTarget(
                stage, prim, self._channels, self._Writer())
        self._PrimePreserveChildren()
        self._RebuildHandles()
        self.toolbar.Sync()
        if self._panel is not None:
            self._panel.Sync()

    def _PrimePreserveChildren(self):
        """
        Tell the target whether Preserve Children is on, and let it work
        out which children it would have to skip.

        AttributePaths() is what recomputes target.skippedChildren, so
        calling it here is what puts the count in the status label
        BEFORE a drag rather than only after one -- an artist who ticks
        the box wants to know then that two of the children will not
        move with the parent, not once it is too late. _BeginDrag sets
        the same state again in the order the Target requires.
        """
        target = self._target
        if target is None:
            return
        try:
            target.SetPreserveChildren(
                bool(self.settings.For(self._tool).preserveChildren)
                and target.supportsPreserveChildren)
            target.AttributePaths()
        except Exception as error:
            Tf.Warn("rigExecUsdview: preserve-children probe failed: %s"
                    % error)

    def _Orientation(self, target):
        """
        (orientation matrix, gimbal axes) for the active Axis
        Orientation. World needs no frame at all -- the identity is the
        world basis -- and Gimbal falls back to Object for a target with
        no Euler channels to decompose.
        """
        settings = self.settings.For(self._tool)
        choice = settings.orientation
        if choice == gizmoSettings.ORIENT_GIMBAL:
            state = target.RotationState()
            if state is not None:
                order, angles = state
                return (target.ObjectFrame(),
                        gizmoMath.GimbalAxes(order, angles[0], angles[1],
                                             angles[2],
                                             target.GimbalFrame()))
            choice = gizmoSettings.ORIENT_OBJECT
        if choice == gizmoSettings.ORIENT_OBJECT:
            return target.ObjectFrame(), None
        if choice == gizmoSettings.ORIENT_PARENT:
            return target.ChannelFrame(), None
        return Gf.Matrix4d(1.0), None

    def _ToolSupported(self, target):
        if self._tool == TOOL_TRANSLATE:
            return target.supportsTranslate
        if self._tool == TOOL_ROTATE:
            return target.supportsRotate
        if self._tool == TOOL_SCALE:
            return target.supportsScale
        return False

    def _RebuildHandles(self):
        # A rotate drag keeps the handles it started with. The rings
        # turn with the object in Object orientation, and a manipulator
        # that spins away under a held cursor is unusable -- Maya
        # freezes it for the same reason. Translate and scale must keep
        # following the object, so they rebuild every event.
        if self._drag is not None and self._drag.tool == TOOL_ROTATE:
            return
        # resolveCamera() can emit signalFrustumChanged, whose handler
        # lands back here; without the guard the first paint after a
        # camera move recurses until the stack runs out.
        if self._rebuilding:
            return
        self._rebuilding = True
        try:
            self._handles = self._ComputeHandles()
        finally:
            self._rebuilding = False
        if self.overlay is not None:
            self.overlay.update()

    def _ComputeHandles(self):
        target = self._target
        if (not self._visible or target is None
                or self._tool == TOOL_SELECT
                or not self._ToolSupported(target)):
            return []
        camera, viewport, ratio = self._Camera()
        if camera is None:
            return []
        try:
            matrix = target.GizmoMatrix()
            orientation, gimbal = self._Orientation(target)
        except Exception as error:
            Tf.Warn("rigExecUsdview: gizmo frame failed: %s" % error)
            return []
        settings = self.settings.For(self._tool)
        # What the rings were actually laid out on, not what the option
        # asks for: Gimbal falls back to Object for a target with no
        # Euler channels, and a drag has to write through the entry
        # point that matches the axes it can see.
        self._gimbalAxes = gimbal
        return gizmoScreen.BuildHandles(
            self._tool, matrix, camera, viewport, ratio,
            sizePixels=self.settings.manipulatorSize,
            orientation=orientation, gimbalAxes=gimbal,
            freeRotate=bool(settings.freeRotate))

    def _Camera(self):
        """
        (Gf.Camera, viewport, devicePixelRatio) or (None, None, 1.0).

        resolveCamera() is NOT free of side effects -- it conforms the
        frustum to the viewport and emits signalFrustumChanged when that
        changes anything -- so this is called only where a camera is
        really needed. Anything that just wants pixels uses _Ratio().
        """
        view = self._view
        if view is None:
            return None, None, 1.0
        try:
            camera, _ = view.resolveCamera()
            viewport = view.computeWindowViewport()
            ratio = self._Ratio()
        except Exception:
            return None, None, 1.0
        if camera is None:
            return None, None, 1.0
        return camera, viewport, ratio

    def _Ratio(self):
        """The stage view's device pixel ratio; 1.0 without a view."""
        try:
            return float(self._view.devicePixelRatioF())
        except (AttributeError, TypeError):
            return 1.0

    def HandleScreenPositions(self):
        """
        {handle name: [(x, y), ...]} in LOGICAL pixels, which is what Qt
        mouse events use -- a test can hand one of these straight to
        QTest.mousePress.

        Rings report their FRONT points only (the back half is neither
        drawn nor pickable), planes their four corners, and the centre
        and the free-rotate ball a single point.
        """
        ratio = self._Ratio()
        positions = {}
        for handle in self._handles:
            if handle.kind in ("ring", "view"):
                points = [p for run in handle.frontPoints for p in run]
            else:
                points = list(handle.points)
            positions[handle.name] = [(p[0] / ratio, p[1] / ratio)
                                      for p in points]
        return positions

    def PieSlice(self):
        """
        (polygon in physical pixels, rgb) for the rotation-amount wedge,
        or None. Drawn from the PRESS-TIME handle, which is the ring the
        cursor is actually on -- see _Drag.
        """
        drag = self._drag
        if drag is None or drag.tool != TOOL_ROTATE:
            return None
        if drag.handle.kind not in ("ring", "view"):
            return None
        if abs(drag.angle) < 1e-6:
            return None
        polygon = gizmoScreen.PiePolygon(drag.handle, drag.startParameter,
                                         drag.angle)
        return polygon, drag.handle.color

    # -- status ---------------------------------------------------------

    def Status(self):
        if not self._visible:
            return "Viewport tools hidden"
        if self._tool == TOOL_SELECT:
            return "Select"
        target = self._target
        if target is None:
            return self._reason
        verb = _EDIT_VERBS.get(self._tool, self._tool)
        if not self._ToolSupported(target):
            return "%s is unavailable for %s (%s)" % (
                verb, target.label, target.kind)
        if self._drag is not None and self._drag.tool == TOOL_ROTATE:
            return "%s %s  %.1f deg" % (verb, target.label,
                                        self._drag.angle)
        text = "%s %s  [%s / %s]" % (
            verb, target.label,
            "Pivot" if self._channels == gizmoMath.CHANNELS_PIVOT
            else "Pose",
            "Default" if self._writeMode == gizmoMath.WRITE_DEFAULT
            else "Animation")
        skipped = self._SkippedChildren()
        if skipped:
            text += "  (%d child%s not preserved)" % (
                len(skipped), "" if len(skipped) == 1 else "ren")
        if self._warnings:
            text += "  warning: " + "; ".join(self._warnings)
        return text

    def _SkippedChildren(self):
        """
        The target's explanations for children a Preserve Children drag
        will NOT hold still (a rig-placed child, an incompatible op
        stack, a non-zero pivot, a mirrored transform). Empty unless the
        option is on.
        """
        target = self._target
        if target is None:
            return []
        return list(getattr(target, "skippedChildren", []))

    # -- undo -----------------------------------------------------------

    def Undo(self):
        if not self.undoStack.Undo():
            return False
        self._AfterUndoRedo()
        return True

    def Redo(self):
        if not self.undoStack.Redo():
            return False
        self._AfterUndoRedo()
        return True

    def _AfterUndoRedo(self):
        # Restoring layer specs does not schedule a repaint by itself:
        # the evaluator republishes on the ObjectsChanged notice, but
        # nothing tells the viewport to draw the new frame.
        if self._target is not None:
            try:
                self._target.Refresh()
            except Exception:
                self.RefreshTarget()
        self._RebuildHandles()
        self.toolbar.Sync()
        self.usdviewApi.UpdateViewport()

    # -- settings window ------------------------------------------------

    def ShowToolSettings(self):
        if self._panel is None:
            self._panel = ToolSettingsPanel.GetInstance(self)
        self._panel.Rebuild()
        self._panel.show()
        self._panel.raise_()
        self._panel.activateWindow()
        return self._panel

    # -- signals --------------------------------------------------------

    def _onSettingsChanged(self):
        self._PrimePreserveChildren()
        self._RebuildHandles()
        if self._panel is not None:
            self._panel.Sync()

    def _onUndoStackChanged(self):
        self.toolbar.Sync()

    def _onFrustumChanged(self):
        self._RebuildHandles()

    def _onSelectionChanged(self, added=None, removed=None):
        self._AbortDrag()
        self._selected = None
        self._hover = None
        self.RefreshTarget()

    def _onFrameChanged(self, frame):
        # The SIGNAL's frame, never dataModel.currentFrame: the setter
        # emits before it assigns, so re-reading the property here would
        # author one frame behind (rigExecUsdview._FrameValue).
        self._frame = frame if isinstance(frame, Usd.TimeCode) \
            else Usd.TimeCode(float(frame))
        self._AbortDrag()
        self.RefreshTarget()

    def _onStageReplaced(self):
        self._AbortDrag()
        self.undoStack.Clear()
        self._frame = self.usdviewApi.frame
        self._ObserveStage(self.usdviewApi.dataModel.stage)
        self.RefreshTarget()

    def _onObjectsChanged(self, notice, stage):
        # Inside a drag the target is already being refreshed by the
        # drag itself, and re-resolving it here would throw away the
        # base values every Apply* is computed from.
        if self._drag is not None:
            return
        if self._target is None:
            self.RefreshTarget()
            return
        try:
            self._target.Refresh()
        except Exception:
            self.RefreshTarget()
            return
        self._RebuildHandles()

    # -- event filter ---------------------------------------------------

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind in (QtCore.QEvent.Resize, QtCore.QEvent.Show):
            self._SyncOverlay()
            self._RebuildHandles()
            return False
        if kind == QtCore.QEvent.Paint:
            # Geometry only. Storm repaints on a 5 ms timer until the
            # render converges, so rebuilding the handles here would
            # reproject the whole manipulator two hundred times a
            # second for nothing; every reason they could be stale --
            # the camera (signalFrustumChanged, which the view's own
            # paint emits), the viewport (Resize), the values
            # (ObjectsChanged), the frame, the selection, the settings
            # -- has its own hook.
            self._SyncOverlay(restack=False)
            return False
        if kind == QtCore.QEvent.ShortcutOverride:
            return self._OnShortcutOverride(event)
        if kind == QtCore.QEvent.KeyPress:
            return self._OnKeyPress(event)
        if kind == QtCore.QEvent.KeyRelease:
            return self._OnKeyRelease(event)
        if kind == QtCore.QEvent.HoverMove:
            self._UpdateHover(self._Position(event))
            return False
        if kind in (QtCore.QEvent.Leave, QtCore.QEvent.HoverLeave):
            self._UpdateHover(None)
            return False
        if kind == QtCore.QEvent.MouseButtonPress:
            return self._OnPress(event)
        if kind == QtCore.QEvent.MouseMove:
            return self._OnMove(event)
        if kind == QtCore.QEvent.MouseButtonRelease:
            return self._OnRelease(event)
        return False

    def _SyncOverlay(self, restack=True):
        """
        Keep the overlay exactly over the stage view.

        Restacking is skipped on a plain repaint: raise_() schedules
        work of its own, and doing it inside the view's paint would put
        the widget in a loop with itself.
        """
        overlay = self.overlay
        view = self._view
        if overlay is None or view is None:
            return
        if overlay.geometry() != view.rect():
            overlay.setGeometry(view.rect())
        overlay.setVisible(self._visible)
        if restack:
            overlay.raise_()

    def _Repaint(self):
        if self.overlay is not None:
            self.overlay.update()

    def _Position(self, event):
        """
        The cursor in the PHYSICAL pixels gizmoScreen works in.

        Qt reports widget-local LOGICAL pixels and computeWindowViewport
        is physical; usdview's own mousePressEvent bridges the two with
        devicePixelRatioF and anything picking by hand has to as well.
        Skipping it is invisible on a 1.0-ratio display and puts every
        pick at half the cursor's position on a HiDPI one.
        """
        try:
            position = event.position()
            x, y = position.x(), position.y()
        except AttributeError:
            x, y = event.x(), event.y()
        ratio = self._Ratio()
        return (x * ratio, y * ratio)

    def _HitTest(self, point):
        ratio = self._Ratio()
        return gizmoScreen.HitTest(self._handles, point[0], point[1],
                                   HIT_PIXELS * ratio)

    def _Handle(self, name):
        for handle in self._handles:
            if handle.name == name:
                return handle
        return None

    def _OnPress(self, event):
        if event.modifiers() & _CAMERA_MODIFIERS:
            return False              # usdview's camera drags win
        if not self._visible or self._tool == TOOL_SELECT or \
                not self._handles:
            return False
        point = self._Position(event)
        self._ctrl = bool(event.modifiers() & QtCore.Qt.ControlModifier)
        if event.button() == QtCore.Qt.LeftButton:
            handle = self._HitTest(point)
            if handle is None:
                return False          # let usdview pick the prim
            if not self._BeginDrag(handle, point):
                return False
            self._selected = handle.name
            self._Repaint()
            return True
        if event.button() == QtCore.Qt.MiddleButton:
            # Maya's "middle-drag anywhere repeats the selected handle":
            # the artist does not have to hit the handle again.
            handle = self._Handle(self._selected)
            if handle is None or not handle.grabbable:
                return False
            return self._BeginDrag(handle, point)
        return False

    def _OnMove(self, event):
        if self._drag is not None:
            self._drag.ctrl = bool(
                event.modifiers() & QtCore.Qt.ControlModifier)
            self._UpdateDrag(self._Position(event))
            return True
        if not self._visible or self._tool == TOOL_SELECT:
            return False
        if event.buttons():
            return False              # a camera or picking drag
        self._UpdateHover(self._Position(event))
        return False

    def _UpdateHover(self, point):
        """Maya's pre-selection highlight; `point` None clears it."""
        if point is None or not self._visible or self._tool == TOOL_SELECT:
            name = None
        else:
            handle = self._HitTest(point)
            name = handle.name if handle is not None else None
        if name != self._hover:
            self._hover = name
            self._Repaint()

    def _OnRelease(self, event):
        if self._drag is None:
            return False
        if event.button() not in (QtCore.Qt.LeftButton,
                                  QtCore.Qt.MiddleButton):
            return False
        self._EndDrag()
        return True

    # -- keys -----------------------------------------------------------

    # The keys a live drag owns outright. A KeyPress that matches ANY
    # shortcut never reaches the focus widget -- Qt consumes it in the
    # shortcut map -- so a drag has to claim these in ShortcutOverride
    # or it will never see them. Escape is on the list because
    # something in usdview's window claims it: without this, an aborted
    # drag simply kept going.
    _DRAG_KEYS = (QtCore.Qt.Key_Escape, QtCore.Qt.Key_J, QtCore.Qt.Key_X)

    def _OnShortcutOverride(self, event):
        """
        Act on Escape, J and X here rather than waiting for the
        KeyPress, but only while a drag is live.

        A key that matches ANY shortcut is consumed by Qt's shortcut map
        and never reaches the focus widget, and both Escape and J match
        one in a usdview window (J is Toggle Framed View). Accepting the
        override is the documented way to reclaim such a key -- but it
        does not always produce a KeyPress either, because a synthetic
        key from QTest stops at the override it sees accepted. Doing the
        work in the override itself is the one path that holds for a
        real keyboard and for a test, and outside a drag none of this
        runs, so usdview keeps its own bindings.
        """
        if self._drag is None or event.key() not in self._DRAG_KEYS:
            return False
        key = event.key()
        if key == QtCore.Qt.Key_Escape:
            self._AbortDrag()
        elif key == QtCore.Qt.Key_J and not self._holdSnap:
            self._holdSnap = True
            self._ReapplyDrag()
        elif key == QtCore.Qt.Key_X and not self._holdGrid:
            self._holdGrid = True
            self._ReapplyDrag()
        event.accept()
        return True

    def _OnKeyPress(self, event):
        if not self._visible:
            return False
        key = event.key()
        if key == QtCore.Qt.Key_Escape and self._drag is not None:
            self._AbortDrag()
            return True
        if key == QtCore.Qt.Key_J:
            if not self._holdSnap:
                self._holdSnap = True
                self._ReapplyDrag()
            return self._drag is not None
        if key == QtCore.Qt.Key_X:
            if not self._holdGrid:
                self._holdGrid = True
                self._ReapplyDrag()
            return self._drag is not None
        if event.modifiers() & (QtCore.Qt.ControlModifier
                                | QtCore.Qt.AltModifier
                                | QtCore.Qt.MetaModifier):
            return False              # leave usdview's Ctrl+... alone
        tools = {QtCore.Qt.Key_Q: TOOL_SELECT,
                 QtCore.Qt.Key_W: TOOL_TRANSLATE,
                 QtCore.Qt.Key_E: TOOL_ROTATE,
                 QtCore.Qt.Key_R: TOOL_SCALE}
        if key in tools:
            self.SetTool(tools[key])
            return True
        if key in (QtCore.Qt.Key_Plus, QtCore.Qt.Key_Equal):
            self.settings.ScaleManipulator(1.1)
            return True
        if key == QtCore.Qt.Key_Minus:
            self.settings.ScaleManipulator(1.0 / 1.1)
            return True
        if key in (QtCore.Qt.Key_D, QtCore.Qt.Key_Insert):
            self.SetChannels(
                gizmoMath.CHANNELS_POSE
                if self._channels == gizmoMath.CHANNELS_PIVOT
                else gizmoMath.CHANNELS_PIVOT)
            return True
        return False

    def _OnKeyRelease(self, event):
        if event.isAutoRepeat():
            return False
        if event.key() == QtCore.Qt.Key_J and self._holdSnap:
            self._holdSnap = False
            self._ReapplyDrag()
            return self._drag is not None
        if event.key() == QtCore.Qt.Key_X and self._holdGrid:
            self._holdGrid = False
            self._ReapplyDrag()
            return self._drag is not None
        return False

    def _ReapplyDrag(self):
        """Re-run the live drag so a held modifier takes effect at once."""
        if self._drag is not None:
            self._UpdateDrag(self._drag.current)

    # -- drag -----------------------------------------------------------

    def _SnapActive(self):
        return bool(self.settings.For(self._tool).stepSnap) or self._holdSnap

    def _StepSize(self):
        return float(self.settings.For(self._tool).stepSize)

    def _BeginDrag(self, handle, point):
        target = self._target
        if target is None or not handle.grabbable:
            return False
        camera, viewport, _ = self._Camera()
        if camera is None:
            return False
        settings = self.settings.For(self._tool)
        # BEFORE AttributePaths() and BeginDrag(): the compensated
        # children's channels only join the undo set while the option
        # is on.
        target.SetPreserveChildren(bool(settings.preserveChildren)
                                   and target.supportsPreserveChildren)
        try:
            recorder = rigExecUndo.EditRecorder(self.usdviewApi.stage,
                                                target.AttributePaths())
            recorder.Begin()
            target.BeginDrag()
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not start the gizmo drag: %s"
                    % error)
            return False
        drag = _Drag(self._tool, handle, recorder, target, point, camera,
                     viewport, handle.center)
        drag.ctrl = self._ctrl
        drag.gimbal = self._gimbalAxes is not None
        if handle.kind in ("ring", "view"):
            drag.startParameter = gizmoScreen.RingParameter(handle, point)
        self._drag = drag
        self.toolbar.Sync()
        self._Repaint()
        return True

    def _UpdateDrag(self, point):
        drag = self._drag
        if drag is None:
            return
        drag.current = point
        try:
            if drag.tool == TOOL_TRANSLATE:
                self._ApplyTranslate(drag)
            elif drag.tool == TOOL_ROTATE:
                self._ApplyRotate(drag)
            elif drag.tool == TOOL_SCALE:
                self._ApplyScale(drag)
            drag.target.Refresh()
        except Exception as error:
            Tf.Warn("rigExecUsdview: gizmo drag failed: %s" % error)
        self._RebuildHandles()
        self._Repaint()
        self.toolbar.Sync()
        self.usdviewApi.UpdateViewport()

    def _EndDrag(self):
        drag = self._drag
        self._drag = None
        if drag is None:
            return
        label = "%s %s" % (_EDIT_VERBS.get(drag.tool, drag.tool),
                           drag.target.label)
        try:
            edit = drag.recorder.Commit(label)
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not record the gizmo edit: %s"
                    % error)
            edit = None
        if edit is not None:
            self.undoStack.Push(edit)
        self._warnings = drag.target.writer.Warnings()
        self._RebuildHandles()
        self.toolbar.Sync()
        self._Repaint()

    def _AbortDrag(self):
        drag = self._drag
        self._drag = None
        if drag is None:
            return
        if not drag.target.prim.IsValid():
            # The stage went away under the drag (a replacement, or
            # usdview quitting). There is no layer left to restore into
            # and no prim to re-read; dropping the drag IS the abort.
            self._RebuildHandles()
            return
        try:
            drag.recorder.Abort()
            drag.target.Refresh()
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not abort the gizmo drag: %s"
                    % error)
        self._RebuildHandles()
        self.toolbar.Sync()
        self._Repaint()
        self.usdviewApi.UpdateViewport()

    # -- drag mathematics -----------------------------------------------

    def _PlaneDelta(self, drag, origin, normal):
        """
        A ray/plane drag delta, with a sanity net.

        The intersection is what keeps the grabbed point under the
        cursor as the plane recedes, but a plane that tips towards
        edge-on mid-drag sends it towards infinity. The camera-plane
        delta for the same travel is always bounded, so a result wildly
        larger than that one is a miss and the last good delta stands.
        """
        delta = gizmoScreen.RayPlaneDragDelta(
            drag.camera, drag.viewport, origin, normal, drag.press,
            drag.current)
        reference = gizmoScreen.PlaneDragDelta(
            drag.camera, drag.viewport, origin, drag.press, drag.current)
        limit = max(reference.GetLength(), 1e-9) * _PLANE_DELTA_SANITY
        if drag.lastDelta is not None and delta.GetLength() > limit:
            return drag.lastDelta
        drag.lastDelta = delta
        return delta

    def _ApplyTranslate(self, drag):
        handle = drag.handle
        origin = Gf.Vec3d(handle.worldCenter)
        if handle.kind == "axis":
            if drag.ctrl:
                delta = self._PlaneDelta(drag, origin, handle.worldAxis)
            else:
                parameter = gizmoScreen.AxisDragParameter(
                    handle, drag.press, drag.current)
                delta = Gf.Vec3d(handle.worldAxis) * (parameter
                                                      * handle.worldLength)
        elif handle.kind == "plane":
            delta = self._PlaneDelta(drag, origin, handle.worldNormal)
        else:
            delta = gizmoScreen.PlaneDragDelta(
                drag.camera, drag.viewport, origin, drag.press, drag.current)
        delta = self._SnapTranslate(drag, delta)
        drag.target.ApplyTranslate(delta)

    def _SnapTranslate(self, drag, delta):
        """
        Step Snap quantises the DELTA in the channel frame, so an object
        that started off the grid moves in whole steps without jumping
        onto it. Holding X instead lands the object ON a grid of the
        same step, which is what Maya's grid snap does; the grid is the
        active orientation's axes anchored at the world origin.
        """
        step = self._StepSize()
        if step <= 0.0:
            return delta
        target = drag.target
        if self._SnapActive():
            frame = target.ChannelFrame()
            local = gizmoScreen.SnapRelative(_ToFrame(frame, delta), step)
            delta = _FromFrame(frame, local)
        if self._holdGrid:
            frame = self._Orientation(target)[0]
            origin = Gf.Vec3d(drag.handle.worldCenter)
            position = origin + delta
            snapped = gizmoScreen.SnapAbsolute(_ToFrame(frame, position),
                                               step)
            delta = _FromFrame(frame, snapped) - origin
        return delta

    def _ApplyRotate(self, drag):
        handle = drag.handle
        if handle.kind == "sphere":
            step = gizmoScreen.TrackballRotation(
                drag.camera, drag.trackballLast, drag.current,
                handle.radiusPixels)
            drag.trackballLast = drag.current
            if step is not None:
                axis, degrees = step
                matrix = Gf.Matrix4d(1.0)
                matrix.SetRotate(Gf.Rotation(axis, degrees))
                # Row-vector composition: the running rotation first,
                # then this step, so a curved drag rolls the ball.
                drag.trackball = drag.trackball * matrix
            rotation = drag.trackball.ExtractRotation()
            drag.angle = rotation.GetAngle()
            drag.target.ApplyRotate(rotation.GetAxis(), drag.angle)
            return
        raw = gizmoScreen.RotationDragAngle(
            handle.center, drag.press, drag.current,
            gizmoScreen.AxisFacesCamera(drag.camera, handle.worldAxis))
        drag.total = gizmoScreen.AccumulateAngle(drag.total, drag.raw, raw)
        drag.raw = raw
        angle = drag.total
        if self._SnapActive():
            angle = gizmoScreen.SnapRelative(angle, self._StepSize())
        drag.angle = angle
        if drag.gimbal and handle.kind == "ring":
            # Maya Gimbal: the ring IS one Euler channel, so the angle
            # goes straight onto that channel. ApplyRotate would take
            # the world-axis route, which under a sheared channel frame
            # (a non-uniform scale anywhere above) reaches the same
            # drawn rotation by moving all three channels -- correct
            # geometry, but not what a gimbal ring promises.
            drag.target.ApplyRotateChannel(handle.axisIndex, angle)
        else:
            drag.target.ApplyRotate(handle.worldAxis, angle)

    def _ApplyScale(self, drag):
        handle = drag.handle
        settings = self.settings.For(TOOL_SCALE)
        factor = gizmoScreen.MayaScaleFactor(
            handle, drag.origin2d, drag.press, drag.current,
            not settings.preventNegativeScale)
        if self._SnapActive():
            # The ratio is quantised, not the resulting channel value:
            # the base scale a drag started from is private to the
            # target. For the ordinary unit-scale prim the two are the
            # same thing.
            factor = gizmoScreen.SnapRelative(factor, self._StepSize())
            if settings.preventNegativeScale:
                factor = max(gizmoScreen.MIN_SCALE_FACTOR, factor)
        if handle.kind == "center":
            drag.target.ApplyScale(None, factor)
        elif handle.kind == "plane":
            axes = [i for i in range(3) if i != handle.axisIndex]
            drag.target.ApplyScale(_PlaneAxes(axes), factor)
        else:
            drag.target.ApplyScale(handle.axisIndex, factor)


# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------

_controller = None

# usdview builds its stage view LONG after it loads plugins
# (appController.py configures plugins at ~432 and constructs the
# StageView at ~1863), so the first stage-replaced callback a container
# gets has no viewport to attach to. Rather than make every caller guess
# at that ordering, an install that arrives too early re-tries itself
# once the event loop turns. Bounded, so a genuinely headless session
# stops asking instead of posting timers forever.
_INSTALL_RETRIES = 20
_installPending = False


def InstallViewportTools(usdviewApi, undoStack, retries=_INSTALL_RETRIES):
    """
    Put the toolbar and the overlay on usdview's stage view once.

    Returns the controller, or None when there is no stage view yet (in
    which case an install is queued) or none at all.
    """
    global _controller, _installPending
    if _controller is not None:
        return _controller
    if StageView(usdviewApi) is None:
        if retries > 0 and not _installPending:
            _installPending = True

            def _Retry():
                global _installPending
                _installPending = False
                InstallViewportTools(usdviewApi, undoStack, retries - 1)

            QtCore.QTimer.singleShot(0, _Retry)
        return None
    _controller = GizmoController(usdviewApi, undoStack)
    return _controller


def GetController():
    """The installed controller, or None."""
    return _controller
