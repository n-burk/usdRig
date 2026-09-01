#
# RigExec usdview plugin: the viewport manipulator toolbar -- Maya-style
# Move / Rotate / Scale gizmos over the stage view, undoable through the
# shared rigExecUndo stack.
#
# Layering, and why: everything that can be decided without Qt already
# was. gizmoMath owns "what does this world delta do to the channels",
# gizmoScreen owns "where is the handle and what did the mouse mean",
# gizmoDrag owns "what does this drag do" (the whole of spec 8.2-8.4's
# manipulation half, headlessly tested), gizmoSettings owns "what are
# the tool's options". This file is the Qt shell: a toolbar, a
# transparent overlay that paints projected handles, a settings window,
# and the event plumbing that turns mouse and key events into
# gizmoDrag.ApplyDrag() calls bracketed by an EditRecorder.
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
# HOTKEYS AND usdview (design spec 8.5). The viewport keys are read from
# an APPLICATION-level event filter (ViewportHotkeyFilter), not from the
# stage view: usdview's stage view has no focus policy and its
# application-wide AppEventFilter refocuses the main window on every
# mouse move (appEventFilter.py SetFocusFromMousePos), so a key never
# reaches the view and a widget-level filter never runs. The filter is
# gated so it cannot steal anything: the event must belong to usdview's
# main window, the focus widget must not be a text field or spin box,
# and the tool / size / pivot keys additionally need the cursor over the
# viewport. Escape and the J / X holds are claimed only while a drag is
# live, wherever the cursor is.
#
# Three things in a usdview window get at a key first: the
# Qt.ApplicationShortcut actions in mainWindowUI.ui, the shortcut map,
# and AppEventFilter. Neither key below is given up, but both are
# shared:
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
    import gizmoDrag
    import gizmoMath
    import gizmoScreen
    import gizmoSettings
    import rigExecUndo
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoDrag
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
        # WA_NoSystemBackground plus no auto-fill is what leaves the
        # stage view showing through; WA_TranslucentBackground is a
        # top-level-window attribute and would do nothing on a child.
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
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
        painter.setPen(self._Pen(handle))
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

class ViewportStatusBar(QtWidgets.QLabel):
    """
    The gizmo's status line, on its own full-width row under the tools.

    A row of its own because QToolBar moves whatever does not fit into
    an overflow chevron, and at usdview's DEFAULT viewport width the
    status label and the Tool Settings button were already behind it --
    exactly the width at which "why is there no gizmo" and the live
    rotation angle matter most. Elided rather than wrapped so the row
    keeps one constant height and the viewport never jumps as the text
    changes; the whole string stays on the tooltip.
    """

    def __init__(self, parent=None):
        super(ViewportStatusBar, self).__init__(parent)
        self._full = ""
        self.setContentsMargins(6, 1, 6, 1)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                           QtWidgets.QSizePolicy.Fixed)
        self.setToolTip("What the gizmo is editing, or why there is none.")

    def SetStatus(self, text):
        text = text or ""
        if text != self._full:
            self._full = text
            self.setToolTip(text or "No viewport gizmo.")
        self._Elide()

    def FullText(self):
        """The untruncated status; what a test should assert on."""
        return self._full

    def resizeEvent(self, event):
        super(ViewportStatusBar, self).resizeEvent(event)
        self._Elide()

    def _Elide(self):
        metrics = QtGui.QFontMetrics(self.font())
        width = max(0, self.width() - 12)
        QtWidgets.QLabel.setText(
            self, metrics.elidedText(self._full, QtCore.Qt.ElideRight,
                                     width))


class ViewportToolbar(QtWidgets.QToolBar):
    """
    The strip above the viewport: tools, channel set, write mode, undo,
    the Tool Settings button and the status label that always says why
    there is no gizmo.
    """

    def __init__(self, controller, statusBar, parent=None):
        super(ViewportToolbar, self).__init__("RigExec Viewport Tools",
                                              parent)
        self._controller = controller
        self._status = statusBar
        self.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        self.setMovable(False)
        self.setFloatable(False)
        # Two things this stylesheet buys. The checked state, because
        # the default style barely distinguishes it and "which tool am I
        # in" is the one thing this bar must say. And tight padding,
        # because the row has to FIT: at usdview's default split the
        # viewport is around 600 logical px and the default padding put
        # the bar over that, which sends the trailing items into
        # QToolBar's overflow chevron where nobody finds them.
        self.setStyleSheet(
            "QToolBar { padding: 0px; spacing: 1px; }"
            " QToolButton { padding: 1px 5px; margin: 0px; }"
            " QToolButton:checked { background: #4879b4; color: white;"
            " border: 1px solid #79a6dc; border-radius: 3px; }")
        self.setContentsMargins(0, 0, 0, 0)

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
        self.addWidget(QtWidgets.QLabel(" Channels "))
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
        self.addWidget(QtWidgets.QLabel(" Write "))
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
        self.settingsAction = QtActionWidgets.QAction("Settings…", self)
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
        # Greyed out during a drag as well as when empty: the actions
        # are application shortcuts and would otherwise fire with a
        # mouse button down (see GizmoController.Undo).
        dragging = controller.IsDragging()
        self.undoAction.setEnabled(stack.CanUndo() and not dragging)
        self.redoAction.setEnabled(stack.CanRedo() and not dragging)
        self.undoAction.setToolTip(
            "Undo %s (Ctrl+Z)" % stack.UndoText() if stack.CanUndo()
            else "Nothing to undo (Ctrl+Z)")
        self.redoAction.setToolTip(
            "Redo %s (Ctrl+Shift+Z, Shift+Z, Ctrl+Y)" % stack.RedoText()
            if stack.CanRedo()
            else "Nothing to redo (Ctrl+Shift+Z, Shift+Z, Ctrl+Y)")
        self._status.SetStatus(controller.Status())

    def StatusLabel(self):
        """The status row widget (a ViewportStatusBar, not in the bar)."""
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
        # A floor, not a fixed size: adjustSize() after each Rebuild
        # fits the height to the tool's rows, and without the floor it
        # also shrinks the width until "Prevent Negative Scale" clips.
        self.setMinimumWidth(360)
        self.resize(360, 300)

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
        # The row count changes with the tool; without this the window
        # keeps its first tool's height and shows an empty band.
        self.adjustSize()
        self.resize(max(self.width(), self.minimumWidth()), self.height())

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
            skipped = self._controller.SkippedChildren()
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


# Focus widgets that own every key they are given. The viewport hotkeys
# are bare letters, so typing a prim name into usdview's search box must
# never switch the tool.
_TEXT_WIDGETS = (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                 QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit)


class ViewportHotkeyFilter(QtCore.QObject):
    """
    The viewport hotkeys, taken at the APPLICATION level.

    A widget-level filter cannot work here: usdview's stage view sets no
    focus policy (it reports NoFocus), and usdview's own application
    filter calls SetFocusFromMousePos on every mouse move
    (appEventFilter.py), which walks up from the stage view and hands
    focus to the main window. So a key pressed over the viewport is
    delivered to the main window, never to the view, and Q / W / E / R,
    the size keys, D / Insert and the Escape abort were all dead in a
    real session even though QTest keys sent straight at the view worked.

    Qt runs application filters most-recently-installed first, and this
    plugin loads after usdview installed its own, so this one sees the
    key first -- which is also what lets a live drag take Escape ahead
    of appEventFilter.py's unconditional focus reset.

    Everything this filter claims, GizmoController.HandleHotkey decides;
    nothing is claimed unless it is acted on.
    """

    def __init__(self, controller):
        super(ViewportHotkeyFilter, self).__init__(controller)
        self._controller = controller

    def eventFilter(self, obj, event):
        try:
            kind = event.type()
            if kind not in (QtCore.QEvent.KeyPress,
                            QtCore.QEvent.KeyRelease,
                            QtCore.QEvent.ShortcutOverride):
                return False
            return self._controller.HandleHotkey(obj, event, kind)
        except Exception as error:
            # An exception escaping an application-wide filter would
            # break every key in usdview, not just ours.
            Tf.Warn("rigExecUsdview: gizmo hotkey filter failed: %s"
                    % error)
            return False


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
        self._claimedKey = None
        self._hotkeys = None
        self._visible = True
        self._gimbalAxes = None
        self._rebuilding = False
        self._panel = None
        self._noticeKey = None
        self._solverPosed = gizmoMath.SolverPosedCache()
        self._frame = usdviewApi.frame

        view = StageView(usdviewApi)
        self._view = view
        self.overlay = GizmoOverlay(self, view) if view is not None else None
        self.statusBar = ViewportStatusBar()
        self.toolbar = ViewportToolbar(self, self.statusBar)

        self._InstallToolbar()
        self._InstallUndoShortcuts()
        self._hotkeys = ViewportHotkeyFilter(self)
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.installEventFilter(self._hotkeys)
        if view is not None:
            # The stage view keeps the MOUSE filter; keys go through the
            # application filter above (see ViewportHotkeyFilter).
            view.installEventFilter(self)
            view.destroyed.connect(self._onViewDestroyed)
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
        layout.insertWidget(1, self.statusBar)
        self.toolbar.Sync()

    def _UndoActions(self):
        return [a for a in (getattr(self.toolbar, "undoAction", None),
                            getattr(self.toolbar, "redoAction", None))
                if a is not None]

    def _InstallUndoShortcuts(self):
        """
        Give usdview's main window its own handle on the undo and redo
        actions.

        Their shortcuts are Qt.ApplicationShortcut, but Qt will not fire
        a shortcut whose owning widget is hidden -- and RigExec ->
        Viewport Tools hides the toolbar while leaving every edit on the
        undo stack, so Ctrl+Z died on a stack full of drags. Adding the
        SAME actions to the main window gives them a second home that is
        visible for as long as usdview is. They stay parented to the
        toolbar, so they are still destroyed with it, and Detach takes
        them back off for the case where the toolbar outlives its view.
        """
        window = self._MainWindow()
        if window is None:
            return
        for action in self._UndoActions():
            window.addAction(action)

    def Detach(self):
        """
        Take the application-wide key filter, and the main window's copy
        of the undo actions, back off.

        Both outlive the widgets they serve, so leaving them installed
        after usdview has destroyed the stage view would keep answering
        keys on behalf of a gizmo that no longer has a viewport.
        """
        if self._hotkeys is None:
            return
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.removeEventFilter(self._hotkeys)
        self._hotkeys = None
        window = self._MainWindow()
        if window is None:
            return
        for action in self._UndoActions():
            try:
                window.removeAction(action)
            except RuntimeError:
                # Detach also runs from the stage view's destroyed
                # signal at shutdown, by which point the main window's
                # C++ object can be gone while the wrapper survives.
                # Qt has already dropped the action with it.
                return

    def _onViewDestroyed(self, *args):
        self._view = None
        self.overlay = None
        self.Detach()

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
        self.statusBar.setVisible(visible)
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
                stage, prim, self._channels, self._Writer(),
                self._solverPosed)
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
            return ("Select: usdview picking. Choose Move, Rotate or "
                    "Scale for a manipulator.")
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
        skipped = self.SkippedChildren()
        if skipped:
            text += "  (%d child%s not preserved)" % (
                len(skipped), "" if len(skipped) == 1 else "ren")
        if self._warnings:
            text += "  warning: " + "; ".join(self._warnings)
        return text

    def SkippedChildren(self):
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
        # Maya ignores undo while a manipulator is held. Running it here
        # would restore an earlier edit that the drag's next event then
        # overwrites from its own base, and the release would push over
        # the redo branch -- the earlier edit lost from history with its
        # effect half applied.
        if self._drag is not None or not self.undoStack.Undo():
            return False
        self._AfterUndoRedo()
        return True

    def Redo(self):
        if self._drag is not None or not self.undoStack.Redo():
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

    def StatusBar(self):
        """The status row widget under the tool buttons."""
        return self.statusBar

    def ShowToolSettings(self):
        if self._panel is None:
            self._panel = ToolSettingsPanel.GetInstance(self)
        # Sync() rebuilds by itself when the tool has changed; rebuilding
        # unconditionally would throw away and recreate every row widget
        # on each click of the button.
        self._panel.Sync()
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
        # _selected survives: Maya keeps the active handle across a
        # selection change, so middle-drag still repeats it on the prim
        # you just picked. The hover is stale by definition.
        self._AbortDrag()
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
        self._solverPosed.Clear()
        self._frame = self.usdviewApi.frame
        self._ObserveStage(self.usdviewApi.dataModel.stage)
        self.RefreshTarget()

    def _onObjectsChanged(self, notice, stage):
        """
        Refresh the target, but only for a notice that can have moved it.

        Notices arrive from the volume weight panel, the curvenet panel,
        a timeline scrub and anything else authoring in the session, not
        just from the gizmo. Refreshing unconditionally bought each of
        them a full rig walk plus a resolveCamera() -- which conforms
        the frustum and can emit signalFrustumChanged, see _Camera --
        plus a reprojection of the whole manipulator, so the filter has
        to come BEFORE any of that, not inside it.
        """
        # Inside a drag the target is already being refreshed by the
        # drag itself, and re-resolving it here would throw away the
        # base values every Apply* is computed from.
        if self._drag is not None:
            return
        resynced = notice.GetResyncedPaths()
        if resynced:
            # Only a resync can add or remove a rigExec:joints target.
            self._solverPosed.InvalidateResynced(resynced)
        target = self._target
        # A missing target may be exactly what this notice creates, and a
        # focus prim that no longer matches needs the full re-resolve.
        if target is None or self._FocusPrim() != target.prim:
            self.RefreshTarget()
            return
        if not gizmoMath.NoticeAffectsTarget(
                resynced, notice.GetChangedInfoOnlyPaths(),
                target.prim.GetPath(), target.RigRootPath()):
            return
        try:
            target.Refresh()
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
        # Key events are NOT handled here: usdview never lets the stage
        # view hold focus, so they arrive through the application-level
        # ViewportHotkeyFilter instead.
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

    # The keys a live drag owns outright, wherever the cursor is.
    _DRAG_KEYS = (QtCore.Qt.Key_Escape, QtCore.Qt.Key_J, QtCore.Qt.Key_X)

    # Everything else, which needs the cursor over the viewport.
    _TOOL_KEYS = {QtCore.Qt.Key_Q: TOOL_SELECT,
                  QtCore.Qt.Key_W: TOOL_TRANSLATE,
                  QtCore.Qt.Key_E: TOOL_ROTATE,
                  QtCore.Qt.Key_R: TOOL_SCALE}
    _SIZE_KEYS = (QtCore.Qt.Key_Plus, QtCore.Qt.Key_Equal,
                  QtCore.Qt.Key_Minus)
    _PIVOT_KEYS = (QtCore.Qt.Key_D, QtCore.Qt.Key_Insert)

    def _MainWindow(self):
        """usdview's main window, or None (a headless probe has none)."""
        try:
            return self.usdviewApi.qMainWindow
        except Exception:
            return None

    def _InMainWindow(self, receiver):
        """Whether the event belongs to usdview's own main window."""
        main = self._MainWindow()
        if main is None:
            return False
        if isinstance(receiver, QtWidgets.QWidget):
            return receiver.window() is main
        # A QWindow or another non-widget receiver: a real key press
        # implies the window is active anyway.
        return QtWidgets.QApplication.activeWindow() is main

    @staticmethod
    def _TypingFocus():
        focus = QtWidgets.QApplication.focusWidget()
        if focus is None:
            return False
        if isinstance(focus, _TEXT_WIDGETS):
            return True
        return (isinstance(focus, QtWidgets.QComboBox)
                and focus.isEditable())

    def _CursorOverView(self):
        view = self._view
        if view is None:
            return False
        if view.underMouse():
            return True
        try:
            return view.rect().contains(
                view.mapFromGlobal(QtGui.QCursor.pos()))
        except Exception:
            return False

    def HandleHotkey(self, receiver, event, kind):
        """
        One key event from the application filter. True consumes it.

        The gates, in order: the viewport tools must be visible, the
        event must belong to usdview's main window, and the focus widget
        must not be a text field. A live drag then owns Escape / J / X
        wherever the cursor is; the tool, size and pivot keys need the
        cursor over the viewport, or the event delivered straight at the
        view (which is what QTest does and a real keyboard never does).
        """
        if not self._visible or self._view is None:
            return False
        if not self._InMainWindow(receiver):
            return False
        key = event.key()
        # Release the latch BEFORE the typing gate: a key pressed over
        # the viewport and released after the focus moved into a text
        # field would otherwise stay latched and stop working.
        if (kind == QtCore.QEvent.KeyRelease and key == self._claimedKey
                and not event.isAutoRepeat()):
            self._claimedKey = None
        if self._TypingFocus():
            return False
        if kind == QtCore.QEvent.KeyRelease:
            return self._Claim(event, self._ReleaseHold(event, key))
        if self._drag is not None and key in self._DRAG_KEYS:
            return self._Claim(event, self._Act(kind, key, self._DragKey))
        if event.modifiers() & (QtCore.Qt.ControlModifier
                                | QtCore.Qt.AltModifier
                                | QtCore.Qt.MetaModifier):
            return False              # leave usdview's Ctrl+... alone
        if not (self._CursorOverView() or receiver is self._view):
            return False
        return self._Claim(event, self._Act(kind, key, self._ToolKey))

    @staticmethod
    def _Claim(event, handled):
        """
        Mark a key we acted on as accepted, not merely filtered.

        Returning True from an event filter stops the delivery, but Qt's
        shortcut map looks at the ACCEPTED flag on a ShortcutOverride to
        decide whether to run the matching shortcut. Without this, J
        both snapped the drag and fired usdview's Toggle Framed View,
        which moved the camera out from under the manipulator.
        """
        if handled:
            event.accept()
        return handled

    def _Act(self, kind, key, handler):
        """
        Run `handler` exactly ONCE per physical key press, whichever of
        the several deliveries Qt makes arrives first.

        One press reaches an application filter many times. Qt sends a
        ShortcutOverride and then a KeyPress, and it delivers each of
        them to the focus widget and then, while they stay unaccepted,
        to every ancestor up to the window -- five hops in usdview. It
        is also unknowable which of the two Qt will let through: a key
        matching any shortcut is swallowed by the shortcut map before
        the KeyPress, while QTest stops at an override it sees accepted.
        So the rule is: act on the first delivery, remember the key, and
        swallow every later one until its KeyRelease clears the latch.
        Acting only in the override loses a real keyboard, acting only
        in the KeyPress loses J and Escape, and acting on each delivery
        grew the manipulator by 10% five times per keypress.
        """
        del kind                      # every delivery is treated alike
        if key == self._claimedKey:
            return True               # already acted on this press
        if not handler(key):
            return False
        self._claimedKey = key
        return True

    def _DragKey(self, key):
        if key == QtCore.Qt.Key_Escape:
            self._AbortDrag()
            return True
        if key == QtCore.Qt.Key_J:
            if not self._holdSnap:
                self._holdSnap = True
                self._ReapplyDrag()
            return True
        if key == QtCore.Qt.Key_X:
            if not self._holdGrid:
                self._holdGrid = True
                self._ReapplyDrag()
            return True
        return False

    def _ToolKey(self, key):
        if key in self._TOOL_KEYS:
            self.SetTool(self._TOOL_KEYS[key])
            return True
        if key in self._SIZE_KEYS:
            self.settings.ScaleManipulator(
                1.0 / 1.1 if key == QtCore.Qt.Key_Minus else 1.1)
            return True
        if key in self._PIVOT_KEYS:
            self.SetChannels(
                gizmoMath.CHANNELS_POSE
                if self._channels == gizmoMath.CHANNELS_PIVOT
                else gizmoMath.CHANNELS_PIVOT)
            return True
        return False

    def _ReleaseHold(self, event, key):
        if event.isAutoRepeat():
            return False
        if key == QtCore.Qt.Key_J and self._holdSnap:
            self._holdSnap = False
            self._ReapplyDrag()
            return self._drag is not None
        if key == QtCore.Qt.Key_X and self._holdGrid:
            self._holdGrid = False
            self._ReapplyDrag()
            return self._drag is not None
        return False

    def _ReapplyDrag(self):
        """Re-run the live drag so a held modifier takes effect at once."""
        if self._drag is not None:
            self._UpdateDrag(self._drag.current)

    # -- drag -----------------------------------------------------------

    def _BeginDrag(self, handle, point):
        """
        Bracket a drag: snapshot the attributes, record the base values,
        and build the Qt-free DragState the maths runs on.
        """
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
        drag = gizmoDrag.DragState(
            self._tool, handle, target, point, camera, viewport,
            origin2d=handle.center,
            gimbal=self._gimbalAxes is not None, recorder=recorder)
        drag.ctrl = self._ctrl
        self._drag = drag
        self.toolbar.Sync()
        self._Repaint()
        return True

    def _UpdateDrag(self, point):
        drag = self._drag
        if drag is None:
            return
        try:
            gizmoDrag.ApplyDrag(
                drag, point, self.settings.For(drag.tool),
                holdSnap=self._holdSnap, holdGrid=self._holdGrid,
                ctrl=drag.ctrl)
            drag.target.Refresh()
        except Exception as error:
            Tf.Warn("rigExecUsdview: gizmo drag failed: %s" % error)
        self._RebuildHandles()
        self._Repaint()
        self.toolbar.Sync()
        self.usdviewApi.UpdateViewport()

    def _ClearHolds(self):
        """
        Drop the J / X holds and the key latch at the end of a drag.

        A KeyRelease can land on a widget this filter never sees (the
        focus moves under the cursor while a key is down), and a hold
        that survived its drag would silently snap the next one.
        """
        self._holdSnap = False
        self._holdGrid = False
        self._claimedKey = None

    def _EndDrag(self):
        drag = self._drag
        self._drag = None
        self._ClearHolds()
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
        self._ClearHolds()
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
