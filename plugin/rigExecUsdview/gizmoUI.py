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
#   C/V -- usdview's Auto Compute Clipping Planes and Show USD
#         Validation (ordinary window shortcuts, application-wide).
#         Same sharing rule as J: a live drag claims them in
#         ShortcutOverride wherever the cursor is, and outside a drag
#         only the Move tool with a target arms them; everywhere else
#         they stay usdview's (toolbar spec 8.5).
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

from pxr import Gf, Tf, Usd, UsdGeom
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
    import gizmoSnap
    import gizmoPreview
    import rigExecUndo
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoDrag
    import gizmoMath
    import gizmoScreen
    import gizmoSettings
    import gizmoSnap
    import gizmoPreview
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

# The snap candidate marker: orange reads against the axis primaries,
# the selection yellow and the pale hover highlight alike, so the
# landing preview never hides inside the manipulator's own colours.
COLOR_SNAP = (1.0, 0.55, 0.1)
# Opacity of the leader from the grabbed handle to the candidate.
LEADER_OPACITY = 0.35

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
            self._DrawSnap(painter, ratio)
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

    def _DrawSnap(self, painter, ratio):
        """
        The snap landing preview: a diamond plus a crosshair, with the
        kind-specific extra SnapMarker() computed (spec 4.5).
        """
        marker = self._controller.SnapMarker()
        if marker is None:
            return
        center, extras, leader = marker
        pen = QtGui.QPen(_Color(COLOR_SNAP))
        pen.setWidthF(LINE_WIDTH)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        size, arm = 6.0, 11.0
        painter.drawPolygon(QtGui.QPolygonF([
            QtCore.QPointF(center.x(), center.y() - size),
            QtCore.QPointF(center.x() + size, center.y()),
            QtCore.QPointF(center.x(), center.y() + size),
            QtCore.QPointF(center.x() - size, center.y())]))
        painter.drawLine(QtCore.QPointF(center.x() - arm, center.y()),
                         QtCore.QPointF(center.x() + arm, center.y()))
        painter.drawLine(QtCore.QPointF(center.x(), center.y() - arm),
                         QtCore.QPointF(center.x(), center.y() + arm))
        for start, end in extras:
            painter.drawLine(start, end)
        if leader is not None:
            faint = QtGui.QPen(_Color(COLOR_SNAP, LEADER_OPACITY))
            faint.setWidthF(1.0)
            painter.setPen(faint)
            painter.drawLine(leader, center)

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
        self._BuildSnap()
        # No separator of its own: Snap shares this one with Undo,
        # saving 7 px the default-width row cannot spare.
        self._BuildUndo()
        self.addSeparator()
        self._BuildSettingsButton()
        self._BuildGraphButton()

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
        # No section label: the row already overflows at usdview's
        # default viewport width (glFrame 598 logical px), and the
        # Pose/Pivot tooltips below name the group already. The two
        # labels cost ~78 px that pushed Undo/Redo into the chevron.
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
        # No section label either (see _BuildChannels): the
        # Animation/Default tooltips carry the meaning, and the
        # separators still delimit the groups.
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

    def _BuildSnap(self):
        # One button, not four toggles: the row already overflows at
        # usdview's default viewport width, and no test would catch
        # that (the end-to-end test resizes wide first). The menu holds
        # the mutually exclusive modes; the button text names the one
        # actually in force, holds included.
        self._snapButton = QtWidgets.QToolButton(self)
        self._snapButton.setPopupMode(
            QtWidgets.QToolButton.InstantPopup)
        self._snapButton.setToolTip(
            "Where the Move pivot lands: hold X (grid), C (edge) or "
            "V (point) for one drag, or keep a mode on here. Rotate "
            "offers Grid on a Gimbal ring only.")
        self._snapMenu = QtWidgets.QMenu(self._snapButton)
        group = QtActionWidgets.QActionGroup(self._snapMenu)
        group.setExclusive(True)
        self._snapActions = {}
        for mode in (gizmoSettings.SNAP_OFF, gizmoSettings.SNAP_GRID,
                     gizmoSettings.SNAP_POINT, gizmoSettings.SNAP_EDGE,
                     gizmoSettings.SNAP_SURFACE):
            action = QtActionWidgets.QAction(
                gizmoSettings.SnapLabel(mode), self._snapMenu)
            action.setCheckable(True)
            action.triggered.connect(
                lambda checked=False, m=mode: self._onSnap(m))
            group.addAction(action)
            self._snapMenu.addAction(action)
            self._snapActions[mode] = action
        self._snapButton.setMenu(self._snapMenu)
        self.addWidget(self._snapButton)
        # Pin the width to the widest label ("Snap: Surface"): the
        # button text shortens to bare "Snap" when off (see
        # _SyncSnap), and without this the row reflows and Redo drops
        # into the chevron the moment the artist picks Surface. Sized
        # from the button's own sizeHint so style padding and the
        # popup arrow are counted exactly.
        self._snapButton.setText(
            "Snap: %s" % gizmoSettings.SnapLabel(
                gizmoSettings.SNAP_SURFACE))
        self._snapButton.setMinimumWidth(
            self._snapButton.sizeHint().width())

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

    def _BuildGraphButton(self):
        self.graphAction = QtActionWidgets.QAction("Graph…", self)
        self.graphAction.setToolTip(
            "Open the Graph Editor: the selected attributes' animation "
            "curves, their keys and their tangents.")
        self.graphAction.triggered.connect(
            lambda checked=False: self._controller.OpenGraphEditor())
        self.addAction(self.graphAction)

    # -- slots ----------------------------------------------------------

    def _onTool(self, tool):
        self._controller.SetTool(tool)

    def _onChannels(self, channels):
        self._controller.SetChannels(channels)

    def _onWrite(self, mode):
        self._controller.SetWriteMode(mode)

    def _onSnap(self, mode):
        self._controller.settings.For(
            self._controller.Tool()).snapMode = mode

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
        self._SyncSnap()
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

    def _SyncSnap(self):
        """Mirror the sticky snap mode onto the Snap: button."""
        controller = self._controller
        mode = controller.SnapMode()
        # Bare "Snap" when off saves 19 px the default-width row
        # needs; the width stays pinned to "Snap: Surface" (see
        # _BuildSnap) so arming a mode never reflows the row.
        self._snapButton.setText(
            "Snap" if mode == gizmoSettings.SNAP_OFF
            else "Snap: %s" % gizmoSettings.SnapLabel(mode))
        choices = gizmoSettings.SnapChoices(controller.Tool())
        self._snapButton.setEnabled(bool(choices))
        sticky = controller.settings.For(controller.Tool()).snapMode
        for snap, action in self._snapActions.items():
            action.setVisible(snap in choices)
            action.setChecked(snap == sticky)

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

            snapChoices = gizmoSettings.SnapChoices(tool)
            if snapChoices:
                self._snapMode = QtWidgets.QComboBox()
                self._snapMode.setToolTip(
                    "Where the pivot lands. Hold X (grid), C (edge) "
                    "or V (point) for the same thing during one "
                    "drag.")
                for choice in snapChoices:
                    self._snapMode.addItem(
                        gizmoSettings.SnapLabel(choice), choice)
                self._snapMode.currentIndexChanged.connect(
                    self._onSnapMode)
                self._form.addRow("Snap To", self._snapMode)
                if gizmoSettings.SNAP_GRID in snapChoices:
                    self._gridSize = QtWidgets.QDoubleSpinBox()
                    self._gridSize.setRange(
                        gizmoSettings.GRID_SIZE_MIN,
                        gizmoSettings.GRID_SIZE_MAX)
                    self._gridSize.setDecimals(4)
                    self._gridSize.setSingleStep(0.5)
                    self._gridSize.setToolTip(
                        "World units per grid line for Move grid "
                        "snapping. Rotate's grid is degrees and uses "
                        "Step Size instead.")
                    self._gridSize.valueChanged.connect(
                        self._onGridSize)
                    self._form.addRow("Grid Size", self._gridSize)
            if tool == TOOL_ROTATE and snapChoices:
                note = QtWidgets.QLabel(
                    "Point, edge and surface snapping are Move-only. "
                    "Grid lands the dragged Gimbal ring's channel on "
                    "a multiple of Step Size.")
                note.setWordWrap(True)
                self._form.addRow(note)

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
                snapChoices = gizmoSettings.SnapChoices(self._tool)
                if snapChoices:
                    index = self._snapMode.findData(
                        settings.snapMode)
                    if index >= 0:
                        self._snapMode.setCurrentIndex(index)
                    if gizmoSettings.SNAP_GRID in snapChoices:
                        self._gridSize.setValue(
                            float(controller.settings.gridSize))
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

    def _onSnapMode(self, index):
        if self._updating or index < 0:
            return
        self._Settings().snapMode = self._snapMode.itemData(index)

    def _onGridSize(self, value):
        if not self._updating:
            self._controller.settings.gridSize = float(value)

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
            if kind == QtCore.QEvent.WindowDeactivate:
                # Cmd-Tab with a hold down: usdview never sees the
                # release at all, so drop the holds (and the latch)
                # here. _ClearHolds is idempotent; the Sync keeps the
                # Snap: button from naming a hold that is gone.
                self._controller._ClearHolds()
                self._controller.toolbar.Sync()
                return False
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

    def __init__(self, usdviewApi, undoStack, parent=None,
                 openGraphEditor=None):
        super(GizmoController, self).__init__(parent)
        self.usdviewApi = usdviewApi
        self.undoStack = undoStack
        # The container's opener, so this module never imports
        # graphEditorUI at module scope and stays loadable in the
        # headless contexts the C++ tests use. None falls back to a
        # lazy import inside OpenGraphEditor.
        self._openGraphEditor = openGraphEditor
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
        self._holdEdge = False
        self._holdPoint = False
        self._holdPreserve = False
        self._snapMode = gizmoSettings.SNAP_OFF
        self._snapReason = ""
        self._hoverPoint = None
        self._hoverSnap = None
        self._hoverSnapKind = None
        self._hoverPickPoint = None
        self._hoverNote = ""
        self._lastSnapNote = ""
        self._hoverRigStage = None
        self._hoverRigRoot = None
        self._hoverRigWritten = frozenset()
        self._hoverRigWrittenByTarget = frozenset()
        self._hoverRigAllWritten = False
        self._hoverRigAllWrittenByTarget = False
        self._snapGeom = {}
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
        # The hover preview resolved against the old target: the next
        # hover move re-picks. Cleared before toolbar.Sync() below, so
        # the status never names a stale candidate.
        self._hoverSnap = None
        self._hoverSnapKind = None
        self._hoverPickPoint = None
        self._hoverNote = ""
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
                (bool(self.settings.For(self._tool).preserveChildren)
                 or self._holdPreserve)
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

    def SnapCandidate(self):
        """
        The live snap candidate, or None (spec 4.5).

        The drag's while dragging, the hover preview otherwise. Never
        published through Handles(): the Select tool asserts
        HandleScreenPositions() == {}.
        """
        if self._drag is not None:
            return self._drag.snap
        return self._hoverSnap

    def SnapMarker(self):
        """
        (center, extras, leader) for the overlay's snap marker, in
        LOGICAL pixels, or None when there is no candidate to draw.

        Re-projected from the stored WORLD point on every paint, not
        from the pixel captured at pick time: a tumble or dolly with
        no mouse move must carry the marker with the geometry, and
        the pick-time pixel is stale the moment the camera moves.
        """
        candidate = self.SnapCandidate()
        if candidate is None:
            return None
        ratio = self._Ratio()
        center = self._ProjectLogical(candidate.point, ratio)
        if center is None:
            return None
        screen = (center.x() * ratio, center.y() * ratio)
        return (center,
                self._SnapMarkerExtras(candidate, center, ratio),
                self._SnapMarkerLeader(candidate, ratio, screen))

    def _SnapMarkerExtras(self, candidate, center, ratio):
        """The kind-specific lines of the snap marker (spec 4.5)."""
        kind = getattr(candidate, "kind", None)
        if kind == gizmoSettings.SNAP_SURFACE:
            normal = Gf.Vec3d(candidate.normal)
            if normal.GetLength() < 1e-12:
                return []
            end = self._ProjectLength(candidate.point, normal,
                                      18.0, ratio)
            return [(center, end)] if end is not None else []
        if kind == gizmoSettings.SNAP_EDGE:
            lines = []
            for world in getattr(candidate, "edgeEnds", None) or ():
                end = self._ProjectLogical(world, ratio)
                if end is not None:
                    lines.append((center, end))
            return lines
        if kind == gizmoSettings.SNAP_GRID:
            lines = []
            for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0),
                         Gf.Vec3d(0, 0, 1)):
                end = self._ProjectLength(candidate.point, axis,
                                          16.0, ratio)
                if end is not None:
                    lines.append((center, end))
            return lines
        return []

    def _SnapMarkerLeader(self, candidate, ratio, screen):
        """
        The grabbed (or hovered) handle's centre, or None when there
        is none or it already coincides with the candidate: the faint
        line between the two while they differ (spec 4.5).

        `screen` is the candidate's freshly re-projected PHYSICAL
        pixel from SnapMarker, not the pick-time one it carries.
        """
        start = None
        if self._drag is not None and self._drag.handle is not None:
            # The press-time handle's centre is frozen (gizmoDrag.py:
            # 63-68) while _RebuildHandles reprojects the live pivot
            # every Translate move, so read the LIVE handle -- the
            # same lookup the hover branch below already does.
            live = self._Handle(self._drag.handle.name) \
                or self._drag.handle
            start = getattr(live, "center", None)
        elif self._hover is not None:
            handle = self._Handle(self._hover)
            if handle is not None:
                start = handle.center
        if start is None:
            return None
        if math.hypot(screen[0] - start[0],
                       screen[1] - start[1]) < 2.0 * ratio:
            return None
        return QtCore.QPointF(start[0] / ratio, start[1] / ratio)

    def _ProjectLogical(self, worldPoint, ratio, camera=None,
                          viewport=None):
        """A world point as a logical QPointF, or None."""
        if camera is None or viewport is None:
            camera, viewport, _ = self._Camera()
            if camera is None:
                return None
        try:
            screen = gizmoScreen.ProjectPoint(
                gizmoScreen.ViewProjection(camera), viewport,
                worldPoint)
        except Exception:
            return None
        if screen is None:
            return None
        return QtCore.QPointF(screen[0] / ratio, screen[1] / ratio)

    def _ProjectLength(self, worldPoint, direction, pixels, ratio):
        """
        worldPoint pushed along direction by `pixels` LOGICAL screen
        pixels, as a logical QPointF, or None. What sizes the surface
        normal tick and the grid axis ticks in screen space.
        """
        camera, viewport, _ = self._Camera()
        if camera is None:
            return None
        try:
            perPixel = gizmoScreen.WorldPerPixel(
                camera, viewport, worldPoint)
        except Exception:
            return None
        if perPixel is None:
            return None
        unit = Gf.Vec3d(direction)
        if unit.GetLength() < 1e-12:
            return None
        # WorldPerPixel is per PHYSICAL pixel
        # (gizmoScreen.py:189-200: computeWindowViewport is
        # physical, stageView.py:1586-1587), so the logical-pixel
        # constant crosses the device ratio on the way in -- the
        # file's own convention (gizmoSnap.SNAP_PIXELS * ratio,
        # 2.0 * ratio).
        end = Gf.Vec3d(worldPoint) + \
            unit.GetNormalized() * (perPixel * pixels * ratio)
        return self._ProjectLogical(end, ratio, camera, viewport)

    def _ActiveSnap(self):
        """
        (mode, reason) in force with the live holds: ActiveSnapMode
        over the current tool, so a mid-drag V outranks the sticky
        mode and a Rotate V reports itself Move-only instead of
        snapping (gizmoDrag.py:170-171, always a tuple).
        """
        settings = self.settings.For(self._tool)
        return gizmoDrag.ActiveSnapMode(
            settings, self._tool, self._holdGrid, self._holdPoint,
            self._holdEdge)

    def SnapMode(self):
        """The snap mode actually in force -- a bare SNAP_* string."""
        return self._ActiveSnap()[0]

    def SnapReason(self):
        """Why a requested mode was downgraded, or the empty string."""
        return self._ActiveSnap()[1]

    def _SnapClause(self):
        """
        "  snap: ..." / "  grid: ...", or "" when no snap is active
        or armed (spec 4.5). Appended, never substituted: the rotate
        branch must keep "deg", the tool branch "outranked" and
        "unavailable". The rotate grid clause is degrees on the
        dragged gimbal channel ("grid: 15 deg"), never the Move
        world-grid wording.
        """
        drag = self._drag
        if drag is not None:
            mode, reason = self._snapMode, self._snapReason
            detail = getattr(drag, "snapReason", "") or ""
            candidate = drag.snap
            note = self._lastSnapNote
        else:
            mode, reason = self._ActiveSnap()
            detail = ""
            candidate = self._hoverSnap
            note = self._hoverNote
        if mode == gizmoSettings.SNAP_OFF:
            reason = detail or reason
            return "  snap: %s" % reason if reason else ""
        if mode == gizmoSettings.SNAP_GRID:
            if self._tool == TOOL_ROTATE:
                # Rotate's grid is an ABSOLUTE degree grid on the
                # dragged gimbal channel and ignores gridSize (spec
                # 1.8, 4.3), so neither Move wording is true here.
                # `detail` on this tool is only ever _Rotate's
                # downgrade reason, which spec 4.3 words with the
                # "snap: " prefix.
                if detail:
                    return "  snap: %s" % detail
                if drag is None and self._gimbalAxes is None:
                    return "  snap: Grid needs a Gimbal ring"
                return "  grid: %g deg" % (
                    self.settings.For(TOOL_ROTATE).stepSize,)
            return "  %s" % (detail or "grid: world")
        if candidate is not None and \
                getattr(candidate, "kind", None) == mode:
            return "  snap: %s" % gizmoSettings.SnapLabel(mode)
        if note:
            return "  snap: %s" % note
        return "  snap: no target"

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
            return "%s %s  %.1f deg%s" % (verb, target.label,
                                          self._drag.angle,
                                          self._SnapClause())
        text = "%s %s  [%s / %s]%s" % (
            verb, target.label,
            "Pivot" if self._channels == gizmoMath.CHANNELS_PIVOT
            else "Pose",
            "Default" if self._writeMode == gizmoMath.WRITE_DEFAULT
            else "Animation",
            self._SnapClause())
        skipped = self.SkippedChildren()
        if skipped:
            text += "  (%d child%s not preserved)" % (
                len(skipped), "" if len(skipped) == 1 else "ren")
        # Post-drag warnings from the Writer, plus the target's own
        # standing advisory about a drag that authors correctly and
        # moves nothing (a pivot on a joint whose solver has authored
        # bone lengths). One "warning:" prefix carries both.
        notes = list(self._warnings)
        advisory = target.Advisory()
        if advisory:
            notes.append(advisory)
        if notes:
            text += "  warning: " + "; ".join(notes)
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

    def OpenGraphEditor(self):
        """
        The Graph… button: the animation curves of whatever is selected,
        editable on THIS controller's undo stack.

        The container hands its own opener in at install time so it owns
        the panel; the lazy import is the fallback for an install that
        did not, and keeps this module importable without Qt's graph
        editor present.
        """
        if self._openGraphEditor is not None:
            return self._openGraphEditor()
        try:
            try:
                import graphEditorUI
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import graphEditorUI
            return graphEditorUI.OpenGraphEditor(self.usdviewApi,
                                                 self.undoStack)
        except Exception as error:
            Tf.Warn("rigExecUsdview: graph editor unavailable: %s" % error)
            return None

    # -- signals --------------------------------------------------------

    def _onSettingsChanged(self):
        self._PrimePreserveChildren()
        self._RebuildHandles()
        # Re-pick, never redraw stale: switching the Snap: dropdown
        # without moving the cursor must move the marker (spec 4.5).
        self._RefreshHoverSnap()
        # The button text and the status clause both read the sticky
        # mode, so a dropdown or panel write that leaves them showing
        # the old one lies until the next hover move. _RefreshHoverSnap
        # above only syncs when the candidate changed.
        self.toolbar.Sync()
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
        # Points are cached per frame: a scrub moves every one.
        self._snapGeom.clear()
        self._AbortDrag()
        self.RefreshTarget()

    def _onStageReplaced(self):
        self._AbortDrag()
        self.undoStack.Clear()
        self._solverPosed.Clear()
        # New stage, new paths: both snap caches die (the hover one
        # would trip on stage identity anyway).
        self._snapGeom.clear()
        self._hoverRigStage = None
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
        # The hover preview's rig-written sets are only as fresh as
        # the last authoring notice: the curvenet and volume-weight
        # panels author rigExec:moves / rigExec:weightTarget in this
        # session, and a stale set makes the marker offer a vertex
        # the drag (which rebuilds its sets in _BeginDrag) then
        # refuses as rig-deformed. Likewise the snap geometry cache
        # may only keep prims no notice path touches. Both go ABOVE
        # the in-drag guard: a notice arriving mid-drag must not be
        # dropped, and the drag path is unaffected (it reads
        # drag.rigWritten, and its own cache entries survive unless
        # the notice names their prim).
        self._hoverRigStage = None
        if self._snapGeom:
            self._DropSnapGeom(notice.GetResyncedPaths(),
                               notice.GetChangedInfoOnlyPaths())
        # Inside a drag the target is already being refreshed by the
        # drag itself, and re-resolving it here would throw away the
        # base values every Apply* is computed from.
        if self._drag is not None:
            return
        resynced = notice.GetResyncedPaths()
        changed = notice.GetChangedInfoOnlyPaths()
        # Both kinds: a new or cleared joints relationship resyncs, but
        # RE-targeting an existing one is info-only (SolverPosedCache).
        if resynced:
            self._solverPosed.InvalidateResynced(resynced)
        if changed:
            self._solverPosed.InvalidateChanged(changed)
        target = self._target
        # A missing target may be exactly what this notice creates, and a
        # focus prim that no longer matches needs the full re-resolve.
        if target is None or self._FocusPrim() != target.prim:
            self.RefreshTarget()
            return
        if not gizmoMath.NoticeAffectsTarget(
                resynced, changed,
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
        self._hoverPoint = point
        if point is None or not self._visible or self._tool == TOOL_SELECT:
            name = None
        else:
            handle = self._HitTest(point)
            name = handle.name if handle is not None else None
        if name != self._hover:
            self._hover = name
            self._Repaint()
        self._RefreshHoverSnap()

    def _HoverSnapKey(self):
        """What the hover preview shows, for change detection."""
        candidate = self._hoverSnap
        if candidate is None:
            return (None, self._hoverNote)
        return (getattr(candidate, "kind", None),
                str(candidate.point), str(candidate.primPath),
                candidate.index, self._hoverNote)

    def _RefreshHoverSnap(self):
        """
        The armed-mode hover preview (spec 4.5): "hold V and see what
        lights up". The same mode-keyed throttle as the drag path, so
        switching the Snap: dropdown without moving the cursor
        re-picks instead of redrawing a stale marker. toolbar.Sync()
        only when the candidate actually changed.
        """
        if self._drag is not None:
            return
        mode = self.SnapMode()
        if mode in (gizmoSettings.SNAP_OFF, gizmoSettings.SNAP_GRID) \
                or self._hoverPoint is None or self._target is None:
            if self._hoverSnap is not None or self._hoverNote:
                self._hoverSnap = None
                self._hoverSnapKind = None
                self._hoverPickPoint = None
                self._hoverNote = ""
                self._Repaint()
                self.toolbar.Sync()
            return
        point = self._hoverPoint
        if self._hoverSnapKind == mode \
                and self._hoverPickPoint is not None:
            travel = math.hypot(
                point[0] - self._hoverPickPoint[0],
                point[1] - self._hoverPickPoint[1])
            if travel <= gizmoSnap.PICK_MOVE_PIXELS:
                return
        before = self._HoverSnapKey()
        self._hoverNote = ""
        try:
            candidate = self._ResolveSnap(point[0], point[1], mode)
        except Exception:
            candidate = None
        self._hoverSnap = candidate
        self._hoverSnapKind = mode
        self._hoverPickPoint = (point[0], point[1])
        if self._HoverSnapKey() != before:
            self._Repaint()
            self.toolbar.Sync()

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
    _DRAG_KEYS = (QtCore.Qt.Key_Escape, QtCore.Qt.Key_J, QtCore.Qt.Key_X,
                  QtCore.Qt.Key_C, QtCore.Qt.Key_V, QtCore.Qt.Key_B)

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
        must not be a text field. A live drag then owns Escape / J / X /
        C / V wherever the cursor is; the tool, size, pivot and the
        outside-drag C / V keys need the cursor over the viewport, or
        the event delivered straight at the view (which is what QTest
        does and a real keyboard never does).
        """
        if not self._visible or self._view is None:
            return False
        if not self._InMainWindow(receiver):
            return False
        key = event.key()
        # Release the latch AND the holds BEFORE the typing gate: a
        # key pressed over the viewport and released after the focus
        # moved into a text field would otherwise stay latched, and
        # -- since _ToolKey arms C / V outside a drag, where no
        # _ClearHolds ever runs -- stay HELD, point-snapping the next
        # drag into a silent no-op. The claim still happens below the
        # gate: a KeyRelease delivered to a line edit is never ours
        # to consume.
        if kind == QtCore.QEvent.KeyRelease and not event.isAutoRepeat():
            if key == self._claimedKey:
                self._claimedKey = None
            handled = self._ReleaseHold(event, key)
            if self._TypingFocus():
                return False
            return self._Claim(event, handled)
        if self._TypingFocus():
            return False
        if kind == QtCore.QEvent.KeyRelease:
            # Auto-repeat only: the block above took the real one,
            # and _ReleaseHold ignores repeats anyway.
            return False
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
        if key == QtCore.Qt.Key_C:
            if not self._holdEdge:
                self._holdEdge = True
                self._ReapplyDrag()
            return True
        if key == QtCore.Qt.Key_V:
            if not self._holdPoint:
                self._holdPoint = True
                self._ReapplyDrag()
            return True
        if key == QtCore.Qt.Key_B:
            if not self._holdPreserve:
                self._holdPreserve = True
                # No _ReapplyDrag: arming mid-drag would leave the
                # children outside the undo record the recorder has
                # already begun (Target.AttributePaths says why). Hold B
                # BEFORE the drag; pressed during one it arms the next.
                self._PrimePreserveChildren()
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
        if key in (QtCore.Qt.Key_C, QtCore.Qt.Key_V):
            # usdview binds both -- C is Auto Compute Clipping
            # Planes (mainWindowUI.py:1598, appController.py:833),
            # V is Show USD Validation (mainWindowUI.py:1481,
            # appController.py:824) -- so they follow the J sharing
            # rule, NOT the tool-key rule above, which carries no
            # tool condition and would swallow both keys under every
            # tool. Only the Move tool with a target arms them;
            # everywhere else the key stays usdview's (spec 8.5).
            want = gizmoSettings.SNAP_EDGE \
                if key == QtCore.Qt.Key_C \
                else gizmoSettings.SNAP_POINT
            if self._target is None or want not in \
                    gizmoSettings.SnapChoices(self._tool):
                return False
            if key == QtCore.Qt.Key_C:
                self._holdEdge = True
            else:
                self._holdPoint = True
            self._RefreshHoverSnap()
            self.toolbar.Sync()
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
        if key == QtCore.Qt.Key_C and self._holdEdge:
            self._holdEdge = False
            self._ReapplyDrag()
            self._RefreshHoverSnap()
            self.toolbar.Sync()
            return self._drag is not None
        if key == QtCore.Qt.Key_B and self._holdPreserve:
            self._holdPreserve = False
            self._PrimePreserveChildren()
            return True
        if key == QtCore.Qt.Key_V and self._holdPoint:
            self._holdPoint = False
            self._ReapplyDrag()
            self._RefreshHoverSnap()
            self.toolbar.Sync()
            return self._drag is not None
        return False

    def _ReapplyDrag(self):
        """Re-run the live drag so a held modifier takes effect at once."""
        if self._drag is not None:
            self._UpdateDrag(self._drag.current)

    # -- snap pick and resolver -----------------------------------------

    def _SnapPick(self, x, y):
        """
        (primPath, point, normal) under the cursor, guides excluded,
        or None.

        Duplicates curvenetUI.SurfacePicker.Pick
        (curvenetUI.py:530-553) rather than importing it, which would
        pull a whole Qt panel into this module's import graph -- but
        bypasses view.pick(): the plugin forces displayGuide on
        (rigExecUsdview.py:583-588), so pick() returns a RigExec
        guide for every hit (snapping design section 3). The fresh
        RenderParams carries the view settings with showGuides =
        False; view._renderParams is never mutated, since pick()
        reuses it for the artist's own picks. showGuides=False drops
        joints and solvers but NOT controls, whose schema purpose is
        already "default": a control still wins this pick, and
        _ResolveSnap rejects it by prim type instead. Any failure
        degrades to None rather than killing the drag.
        """
        view = self._view
        if view is None:
            return None
        try:
            # Function-level like stageView.py's own pick(), which
            # keeps the GL import off the create-first-image path.
            from pxr import UsdImagingGL
            from OpenGL import GL
        except ImportError as error:
            Tf.Warn("rigExecUsdview: snap pick unavailable: %s"
                    % error)
            return None
        try:
            stage = self.usdviewApi.stage
            if stage is None:
                return None
            inBounds, frustum = view.computePickFrustum(x, y)
            if not inBounds:
                return None
            dataModel = self.usdviewApi.dataModel
            viewSettings = dataModel.viewSettings
            params = UsdImagingGL.RenderParams()
            try:
                params.frame = dataModel.currentFrame
            except AttributeError:
                params.frame = self._frame
            params.complexity = viewSettings.complexity.value
            try:
                params.drawMode = view._renderModeDict[
                    viewSettings.renderMode]
            except (AttributeError, KeyError):
                # Read-only fallback: pick() keeps the artist's own
                # draw mode here, and reading never disturbs it.
                params.drawMode = view._renderParams.drawMode
            params.showGuides = False
            params.showProxy = viewSettings.displayProxy
            params.showRender = viewSettings.displayRender
            params.enableSceneMaterials = \
                viewSettings.enableSceneMaterials
            params.enableSceneLights = viewSettings.enableSceneLights
            renderer = view._getRenderer()
            if renderer is None:
                return None
            view.makeCurrent()
            GL.glDepthMask(GL.GL_TRUE)
            pickParams = UsdImagingGL.Engine.PickParams()
            pickParams.resolveMode = "resolveNearestToCenter"
            hits = renderer.TestIntersection(
                pickParams, frustum.ComputeViewMatrix(),
                frustum.ComputeProjectionMatrix(),
                stage.GetPseudoRoot(), params)
        except Exception as error:
            Tf.Warn("rigExecUsdview: snap pick failed: %s" % error)
            return None
        if not hits:
            return None
        hit = hits[0]
        path = getattr(hit, "hitPrimPath", None)
        point = getattr(hit, "hitPoint", None)
        if path is None or point is None:
            return None
        normal = getattr(hit, "hitNormal", None)
        if normal is None:
            normal = Gf.Vec3d(0, 0, 1)
        else:
            normal = Gf.Vec3d(normal[0], normal[1], normal[2])
        return (path, Gf.Vec3d(point[0], point[1], point[2]),
                normal)

    @staticmethod
    def _RigRoot(target):
        """The target's rig root, or None when it has none."""
        try:
            return target.RigRootPath() if target is not None \
                else None
        except Exception:
            return None

    def _RigWrittenSets(self, stage, rigRoot):
        """
        (rigWritten, rigWrittenByTarget, allWritten,
        allWrittenByTarget): prim paths whose points a mover writes
        (spec 4.4 step 3a), plus whether EVERY visible PointBased prim
        is in each set.

        Collected over the whole stage the way
        rigExecUsdview._FindRigPaths does
        (rigExecUsdview.py:438-445), not through the hit prim's
        ancestry: the deformed Geom prims are siblings of the rig, so
        an ancestry walk would miss them. rigWrittenByTarget is the
        subset whose writing prim sits at or under rigRoot, empty when
        that is None. The flags are what _NoTargetNote reads: the
        "(rig-driven geometry excluded)" hint is only true when there
        is no other geometry to land on.
        """
        written, byTarget = set(), set()
        if stage is None:
            return frozenset(), frozenset(), False, False
        # PrimRange is path order, so a writing prim (/Mover) sorts
        # AFTER the geometry it writes (/Mesh): membership is tested
        # after the traversal, not inside it.
        visiblePaths = []
        try:
            invisible = UsdGeom.Tokens.invisible
            for prim in Usd.PrimRange(stage.GetPseudoRoot()):
                primPath = prim.GetPath()
                for name in ("rigExec:moves",
                             "rigExec:weightTarget"):
                    relationship = prim.GetRelationship(name)
                    if relationship is None:
                        continue
                    for target in relationship.GetTargets():
                        try:
                            writtenPath = target.GetPrimPath()
                        except Exception:
                            continue
                        written.add(writtenPath)
                        if rigRoot is not None and (
                                primPath == rigRoot
                                or primPath.HasPrefix(rigRoot)):
                            byTarget.add(writtenPath)
                if prim.IsA(UsdGeom.PointBased) and \
                        UsdGeom.Imageable(prim).ComputeVisibility() \
                        != invisible:
                    visiblePaths.append(primPath)
        except Exception as error:
            Tf.Warn("rigExecUsdview: rig-written scan failed: %s"
                    % error)
        written, byTarget = frozenset(written), frozenset(byTarget)
        return (written, byTarget,
                bool(visiblePaths) and all(
                    p in written for p in visiblePaths),
                bool(visiblePaths) and all(
                    p in byTarget for p in visiblePaths))

    def _HoverRigSets(self):
        """
        The rig-written sets for the hover preview, cached per
        (stage, rig root): the traversal is stage-wide, so redoing it
        on every hover move would crawl on a production asset. The
        drag path rebuilds its own sets in _BeginDrag instead, where
        one traversal per press is affordable and always fresh.
        """
        stage = self.usdviewApi.stage
        rigRoot = self._RigRoot(self._target)
        if stage is not self._hoverRigStage \
                or rigRoot != self._hoverRigRoot:
            written, byTarget, allWritten, allByTarget = \
                self._RigWrittenSets(stage, rigRoot)
            self._hoverRigWritten = written
            self._hoverRigWrittenByTarget = byTarget
            self._hoverRigAllWritten = allWritten
            self._hoverRigAllWrittenByTarget = allByTarget
            self._hoverRigStage = stage
            self._hoverRigRoot = rigRoot
        return self._hoverRigWritten, \
            self._hoverRigWrittenByTarget, \
            self._hoverRigAllWritten, \
            self._hoverRigAllWrittenByTarget

    def _ResolveSnap(self, x, y, mode):
        """
        The DragState.snapResolver injection (spec 4.1, 4.4): a pick
        into a gizmoSnap.SnapCandidate, or None.

        The mode travels on every call, never bound at the press: the
        holds change mid-drag, so a captured mode would make a mid-drag
        V or C inert. Rejections: the dragged prim and its ancestors
        and descendants (the stage renders as already written, so the
        object would glue itself to the cursor ray); anything at or
        under the target's rig root, but only when that root is not
        None -- xform targets have none, and Sdf.Path.HasPrefix(None)
        raises, so this guards the way
        gizmoMath.NoticeAffectsTarget already does
        (gizmoMath.py:342); RigExec-typed prims (the guides
        showGuides=False cannot drop -- controls keep purpose
        "default"); rig-deformed prims for Point/Edge, and
        own-rig-deformed ones for Surface. WHY a miss happened lands
        on _lastSnapNote (drag) or _hoverNote (hover) for the status
        clause.
        """
        stage = self.usdviewApi.stage
        if self._drag is not None:
            target = self._drag.target
            rigWritten = self._drag.rigWritten
            rigWrittenByTarget = self._drag.rigWrittenByTarget
            allWritten = self._drag.allRigWritten
            allByTarget = self._drag.allRigWrittenByTarget
            camera = self._drag.camera
            viewport = self._drag.viewport
            noteSink = "_lastSnapNote"
        else:
            target = self._target
            rigWritten, rigWrittenByTarget, allWritten, \
                allByTarget = self._HoverRigSets()
            camera, viewport, _ = self._Camera()
            noteSink = "_hoverNote"
            if camera is None:
                return None
        # Surface is rejected only by its own set, so its misses
        # read the Surface flag; every other miss reads the Point /
        # Edge one (spec 4.4 step 3a).
        noTarget = self._NoTargetNote(
            allByTarget if mode == gizmoSettings.SNAP_SURFACE
            else allWritten)
        setattr(self, noteSink, "")
        if target is None:
            return None
        hit = self._SnapPick(x, y)
        if hit is None:
            setattr(self, noteSink, noTarget)
            return None
        hitPath, hitPoint, hitNormal = hit
        try:
            primPath = target.prim.GetPath()
        except Exception:
            return None
        if hitPath == primPath or hitPath.HasPrefix(primPath) \
                or primPath.HasPrefix(hitPath):
            setattr(self, noteSink, noTarget)
            return None
        rigRoot = self._RigRoot(target)
        if rigRoot is not None and (hitPath == rigRoot
                                    or hitPath.HasPrefix(rigRoot)):
            setattr(self, noteSink,
                    "%s is on the dragged rig" % hitPath)
            return None
        # RigExec's guides are synthesized Hydra widgets, never stage
        # geometry. showGuides=False drops joints and solvers, whose
        # schema fallback is purpose "guide", but NOT controls, whose
        # fallback is the stock UsdGeomImageable "default"
        # (libs/rigExecSchema/schema.usda:121-196,
        # libs/rigExecImaging/sceneIndices.cpp:939-944). Point and
        # Edge already refuse them for not being PointBased; Surface
        # would otherwise land the pivot on a drawn control circle.
        hitPrim = stage.GetPrimAtPath(hitPath) if stage else None
        if hitPrim is not None and hitPrim.IsValid() and \
                str(hitPrim.GetTypeName()).startswith("RigExec"):
            setattr(self, noteSink, "%s is a rig guide" % hitPath)
            return None
        if mode in (gizmoSettings.SNAP_POINT, gizmoSettings.SNAP_EDGE) \
                and hitPath in rigWritten:
            setattr(self, noteSink,
                    "%s is rig-deformed" % hitPath)
            return None
        if mode == gizmoSettings.SNAP_SURFACE \
                and hitPath in rigWrittenByTarget:
            setattr(self, noteSink,
                    "%s is rig-deformed" % hitPath)
            return None
        if mode == gizmoSettings.SNAP_SURFACE:
            return self._SurfaceCandidate(
                hitPath, hitPoint, hitNormal, camera, viewport,
                noteSink, allByTarget)
        if mode == gizmoSettings.SNAP_POINT:
            return self._PointCandidate(
                hitPath, x, y, camera, viewport, noteSink,
                allWritten)
        if mode == gizmoSettings.SNAP_EDGE:
            return self._EdgeCandidate(
                hitPath, x, y, camera, viewport, noteSink,
                allWritten)
        return None

    @staticmethod
    def _NoTargetNote(allRigWritten):
        # State the consequence rather than hiding it, but only when
        # it is true: the parenthetical claims EVERY visible PointBased
        # prim is rig-written, so it reads the flag, not the set (spec
        # 4.4 step 3a).
        if allRigWritten:
            return "no target (rig-driven geometry excluded)"
        return "no target"

    def _PrimWorldPoints(self, hitPath):
        """
        The hit prim's authored points in world, or None.

        XformCache, not ComputeRigFrames: the latter composes frames
        from rest/avars only and drops the prim's own xformOps, and it
        takes a path SET, not the cache (gizmoMath.py:445, 465).
        Rig-deformed prims never reach here -- the resolver rejects
        them first, since their authored array is not what Hydra
        draws.
        """
        try:
            stage = self.usdviewApi.stage
            prim = stage.GetPrimAtPath(hitPath) if stage else None
            if prim is None or not prim.IsValid() \
                    or not prim.IsA(UsdGeom.PointBased):
                return None
            points = UsdGeom.PointBased(prim).GetPointsAttr().Get(
                self._frame)
            if not points:
                return None
            matrix = UsdGeom.XformCache(
                self._frame).GetLocalToWorldTransform(prim)
            return [matrix.Transform(Gf.Vec3d(p)) for p in points]
        except Exception:
            return None

    def _SnapPairs(self, hitPath, pointCount):
        """
        (i, j) index pairs for the hit prim's snap segments, or None.

        Mesh edges from the face counts/indices, curve segments from
        consecutive CVs -- the topology half _EdgeCandidate used to
        rebuild on every move. Cached beside the world points in
        _SnapGeom (gizmoSnap.MeshEdges / CurveSegments).
        """
        try:
            stage = self.usdviewApi.stage
            prim = stage.GetPrimAtPath(hitPath) if stage else None
            if prim is None or not prim.IsValid():
                return None
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                pairs = gizmoSnap.MeshEdges(
                    mesh.GetFaceVertexCountsAttr().Get(),
                    mesh.GetFaceVertexIndicesAttr().Get())
            elif prim.IsA(UsdGeom.BasisCurves):
                pairs = gizmoSnap.CurveSegments(
                    UsdGeom.BasisCurves(prim).
                    GetCurveVertexCountsAttr().Get(), pointCount)
            else:
                return None
        except Exception:
            return None
        return pairs or None

    def _SnapGeom(self, hitPath, camera, viewport, ratio,
                  wantPairs):
        """
        The cached geometry the candidates resolve against, or None.

        WHY a cache at all: the per-prim world points, the mesh edge
        list and the segment list are constant for a whole drag but
        were rebuilt from scratch on every move -- 21 ms (Point) and
        58 ms (Edge) per resolve on a 4225-point mesh against the
        spec's 4 ms budget, and the same code ran on plain hover with
        a mode merely armed. The world half is keyed (prim, frame);
        the screen projection alongside it rekeys on the camera view
        matrix plus the viewport, so a hover camera move rebuilds
        only the projection while a drag (fixed camera) builds once
        per press. Queries gather the 3x3 cells around the cursor, so
        only that subset reaches NearestPoint / NearestSegment; the
        returned index maps back to the prim's own numbering.
        Invalidated in _onObjectsChanged (the notice's own paths),
        _onFrameChanged and _onStageReplaced: skipping that is a
        stale-list IndexError.
        """
        key = str(hitPath)
        entry = self._snapGeom.get(key)
        if entry is None or entry.get("frame") != self._frame:
            world = self._PrimWorldPoints(hitPath)
            if not world:
                self._snapGeom.pop(key, None)
                return None
            if len(self._snapGeom) >= 64:
                self._snapGeom.clear()
            entry = {"frame": self._frame, "world": world,
                     "pairs": None, "segments": None,
                     "viewKey": None, "proj": None,
                     "cells": None, "segCells": None}
            self._snapGeom[key] = entry
        world = entry["world"]
        if wantPairs and entry["pairs"] is None:
            pairs = self._SnapPairs(hitPath, len(world))
            if not pairs:
                return None
            entry["pairs"] = pairs
            entry["segments"] = [(world[i], world[j])
                                 for (i, j) in pairs]
            # The projection below may already be built -- a Point
            # resolve on this prim ran first -- and it buckets only
            # the segments it knew. Force it to run again so the new
            # segments join segCells.
            entry["viewKey"] = None
        viewProj = gizmoScreen.ViewProjection(camera)
        viewKey = (str(viewProj), viewport[0], viewport[1],
                   viewport[2], viewport[3], ratio)
        if entry.get("viewKey") != viewKey:
            cell = gizmoSnap.SNAP_PIXELS * ratio
            proj = []
            cells = {}
            for index, point in enumerate(world):
                try:
                    screen, _ = gizmoScreen.ProjectPointWithW(
                        viewProj, viewport, point)
                except Exception:
                    screen = None
                proj.append(screen)
                if screen is not None:
                    cells.setdefault(
                        (math.floor(screen[0] / cell),
                         math.floor(screen[1] / cell)),
                        []).append(index)
            segCells = {}
            if entry["segments"] is not None:
                for segIndex, (i, j) in enumerate(
                        entry["pairs"]):
                    screenA, screenB = proj[i], proj[j]
                    if screenA is None or screenB is None:
                        continue
                    x0, x1 = sorted((screenA[0], screenB[0]))
                    y0, y1 = sorted((screenA[1], screenB[1]))
                    for cx in range(math.floor(x0 / cell),
                                    math.floor(x1 / cell) + 1):
                        for cy in range(math.floor(y0 / cell),
                                        math.floor(y1 / cell) + 1):
                            segCells.setdefault(
                                (cx, cy), []).append(segIndex)
            entry["viewKey"] = viewKey
            entry["proj"] = proj
            entry["cells"] = cells
            entry["segCells"] = segCells
        return entry

    @staticmethod
    def _SnapCells(cells, x, y, cell):
        """
        The pooled 3x3 cells around the cursor, deduped, in prim
        order: NearestPoint / NearestSegment keep the first winner
        on a tie (gizmoSnap.py:232, 292), so ascending order is what
        keeps the subset answer identical to the full scan.
        """
        found = []
        cx, cy = math.floor(x / cell), math.floor(y / cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                found.extend(cells.get((cx + dx, cy + dy), ()))
        return sorted(dict.fromkeys(found))

    def _DropSnapGeom(self, resynced, changed):
        """
        Forget the cached geometry a notice may have moved.

        A resync above a prim rebuilds its subtree and an info-only
        change to one of its properties (including the points array
        itself, which arrives as a property path) moves its points,
        so either drops the entry; every untouched prim keeps its
        one build per (prim, frame, camera). String prefixes, not
        Sdf paths: the notice paths may be property paths, where a
        "/Target.points" change must drop "/Target".
        """
        if not self._snapGeom:
            return
        touched = set(str(p) for p in list(resynced)
                      + list(changed))
        # A notice path may be a property path ("/Target.points"),
        # whose prim prefix is what the cache is keyed on.
        for path in list(touched):
            touched.add(path.split(".")[0])
        for key in list(self._snapGeom):
            for path in touched:
                if path == key or key.startswith(path + "/") \
                        or path.startswith(key + "/"):
                    del self._snapGeom[key]
                    break

    def _PointCandidate(self, hitPath, x, y, camera, viewport,
                        noteSink, allRigWritten):
        ratio = self._Ratio()
        entry = self._SnapGeom(hitPath, camera, viewport, ratio,
                               False)
        if entry is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        cell = gizmoSnap.SNAP_PIXELS * ratio
        nearby = self._SnapCells(entry["cells"], x, y, cell)
        if not nearby:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        try:
            found = gizmoSnap.NearestPoint(
                [entry["world"][i] for i in nearby], (x, y),
                gizmoScreen.ViewProjection(camera), viewport,
                gizmoSnap.SNAP_PIXELS * ratio)
        except Exception:
            found = None
        if found is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        index, point, screen = found
        return gizmoSnap.SnapCandidate(
            point, Gf.Vec3d(0, 0, 0), gizmoSettings.SNAP_POINT,
            hitPath, nearby[index], screen)

    def _EdgeCandidate(self, hitPath, x, y, camera, viewport,
                       noteSink, allRigWritten):
        ratio = self._Ratio()
        entry = self._SnapGeom(hitPath, camera, viewport, ratio,
                               True)
        if entry is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        cell = gizmoSnap.SNAP_PIXELS * ratio
        nearby = self._SnapCells(entry["segCells"], x, y, cell)
        if not nearby:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        segments = entry["segments"]
        try:
            found = gizmoSnap.NearestSegment(
                [segments[k] for k in nearby], (x, y),
                gizmoScreen.ViewProjection(camera), viewport,
                gizmoSnap.SNAP_PIXELS * ratio)
        except Exception:
            found = None
        if found is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        segIndex, point, screen = found
        original = nearby[segIndex]
        candidate = gizmoSnap.SnapCandidate(
            point, Gf.Vec3d(0, 0, 0), gizmoSettings.SNAP_EDGE,
            hitPath, original, screen)
        # The picked segment's world ends, for the marker's two
        # adjacent halves (spec 4.5).
        candidate.edgeEnds = segments[original]
        return candidate

    def _SurfaceCandidate(self, hitPath, hitPoint, hitNormal,
                          camera, viewport, noteSink,
                          allRigWritten):
        try:
            screen = gizmoScreen.ProjectPoint(
                gizmoScreen.ViewProjection(camera), viewport,
                hitPoint)
        except Exception:
            screen = None
        if screen is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        return gizmoSnap.SnapCandidate(
            Gf.Vec3d(hitPoint), Gf.Vec3d(hitNormal),
            gizmoSettings.SNAP_SURFACE, hitPath, -1, screen)

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
        written, byTarget, allWritten, allByTarget = \
            self._RigWrittenSets(
                self.usdviewApi.stage, self._RigRoot(target))
        drag.rigWritten = written
        drag.rigWrittenByTarget = byTarget
        drag.allRigWritten = allWritten
        drag.allRigWrittenByTarget = allByTarget
        # The bound method itself, never a lambda capturing the mode:
        # the holds change mid-drag, so a captured mode would make a
        # mid-drag V or C inert (spec item 13).
        drag.snapResolver = self._ResolveSnap
        mode, reason = gizmoDrag.ActiveSnapMode(
            settings, self._tool, self._holdGrid,
            self._holdPoint, self._holdEdge)
        self._snapMode, self._snapReason = mode, reason
        self._lastSnapNote = ""
        self._drag = drag
        self.toolbar.Sync()
        self._Repaint()
        return True

    def _UpdateDrag(self, point):
        drag = self._drag
        if drag is None:
            return
        try:
            settings = self.settings.For(drag.tool)
            mode, reason = gizmoDrag.ActiveSnapMode(
                settings, drag.tool, self._holdGrid,
                self._holdPoint, self._holdEdge)
            self._snapMode, self._snapReason = mode, reason
            gizmoDrag.ApplyDrag(
                drag, point, settings,
                holdSnap=self._holdSnap, holdGrid=self._holdGrid,
                ctrl=drag.ctrl, snapMode=mode,
                gridSize=self.settings.gridSize)
            # The sample goes to Hydra, not to the stage. BEFORE Refresh():
            # Refresh re-reads the target's frames, and what it has to read is
            # the previewed values, or the handles would be drawn at the
            # pre-drag pose while the geometry moved (gizmoPreview.Push sets
            # both in the right order).
            gizmoPreview.Push(drag.target.writer.Pending())
            drag.target.Refresh()
        except Exception as error:
            Tf.Warn("rigExecUsdview: gizmo drag failed: %s" % error)
        self._RebuildHandles()
        self._Repaint()
        self.toolbar.Sync()
        self.usdviewApi.UpdateViewport()

    def _ClearHolds(self):
        """
        Drop the snap holds (J / X / C / V) and the key latch at the
        end of a drag.

        A KeyRelease can land on a widget this filter never sees (the
        focus moves under the cursor while a key is down), and a hold
        that survived its drag would silently snap the next one.
        """
        self._holdSnap = False
        self._holdGrid = False
        self._holdEdge = False
        self._holdPoint = False
        self._holdPreserve = False
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
            # Author BEFORE the recorder commits: Begin() captured the layer as
            # it was, Commit() captures it as it now is, and the difference
            # between the two IS the undo entry -- so the values have to be on
            # the stage by now. The whole drag authors here, once.
            drag.target.writer.CommitToStage()
            edit = drag.recorder.Commit(label)
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not record the gizmo edit: %s"
                    % error)
            edit = None
        # AFTER authoring: the generation this republishes is the committed
        # one, so the artist sees the value they released on rather than a
        # frame of the pre-drag rig between the two.
        gizmoPreview.End()
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
        # First, and unconditionally: an abandoned preview would keep drawing a
        # pose nobody is holding any more, and this is the one path that can be
        # reached with the stage already gone.
        gizmoPreview.End()
        if drag is None:
            return
        if drag.target is not None and drag.target.writer is not None:
            # Nothing was authored, so there is nothing to take back; dropping
            # the collected values is the whole of it. recorder.Abort() below
            # still runs, because creating an xformOp IS authored structure
            # even when no value ever was.
            drag.target.writer.Clear()
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


def InstallViewportTools(usdviewApi, undoStack, retries=_INSTALL_RETRIES,
                         openGraphEditor=None):
    """
    Put the toolbar and the overlay on usdview's stage view once.

    `openGraphEditor` is the container's callable for the Graph… button;
    passing it in rather than importing graphEditorUI here is what keeps
    this module free of a Qt graph-editor import.

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
                InstallViewportTools(usdviewApi, undoStack, retries - 1,
                                     openGraphEditor)

            QtCore.QTimer.singleShot(0, _Retry)
        return None
    _controller = GizmoController(usdviewApi, undoStack,
                                  openGraphEditor=openGraphEditor)
    return _controller


def GetController():
    """The installed controller, or None."""
    return _controller
