#
# RigExec usdview plugin: a Maya-style view cube in the viewport -- a
# small labelled cube in the top-right corner that shows the camera
# orientation, whose faces, edges and corners orbit the free camera to
# the 26 canonical views when clicked, with hover highlight,
# drag-to-tumble and a home glyph.
#
# Layering, and why: everything that can be decided without Qt already
# was. viewCubeMath owns every number (regions, angles, projection,
# the ray-cast hit test); this file is the Qt shell: a mouse-accepting
# child widget on the stage view, a controller that orbits the free
# camera, and the install helper the container drives from stage
# replacement.
#
# OVERLAY APPROACH: an ordinary (mouse-accepting) CHILD WIDGET of the
# StageView, painted with QPainter. usdview's StageView is a
# QOpenGLWidget, whose contents Qt 6 composites through the backing
# store, so ordinary child widgets do paint on top of it -- the gizmo
# overlay's technique (gizmoUI.py:16-23), verified by its testusdview
# smoke grabs. Unlike the gizmo's overlay this widget is NOT
# mouse-transparent: clicks on the cube belong to the cube, and only
# events it ignore()s (Alt/Meta presses, unclaimed moves and releases)
# propagate to usdview's own navigation. Only the cube and the home
# glyph take presses; the empty widget background falls through to
# usdview's own prim picking.
#
import math
import os
import sys

from pxr import Tf, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import viewCubeMath
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import viewCubeMath


# Orbit animation length; 0 means an immediate jump, which the tests
# use. An attribute on the controller, not a constant, for that reason.
ANIMATION_MS = 250
# A press must travel this far (logical px) before it becomes a tumble,
# so an ordinary click never nudges the camera.
DRAG_THRESHOLD = 4.0
# Degrees per PHYSICAL px (stageView.py:2131-2135 tumbles by
# consecutive physical-pixel deltas).
TUMBLE_RATE = 0.25
# The cube rests dimmed and comes up to full opacity under the cursor.
IDLE_OPACITY = 0.6

# The face label is laid out in a LABEL_BOX-unit square with a bold
# LABEL_PIXELS font, and an extra 1/LABEL_BOX painter scale maps that
# box onto the unit square the face quad expects (spec section 3): Qt
# resolves a QFont to an integer pixel size BEFORE the painter
# transform applies, so sizing the font through the transform would
# paint nothing (0 px) or one glyph blown over the face (1 px floor).
LABEL_BOX = 100.0
LABEL_PIXELS = 20
# Faces flatter than this (square px) sit too edge-on for upright text.
_LABEL_MIN_AREA = 200.0

_FACE_FILL = "#c9c9c9"
_HOVER_FILL = "#a9d3ff"
_EDGE_COLOR = "#6a6a6a"
_LABEL_COLOR = "#303030"

# usdview's camera-navigation claim (stageView.py:2099): Alt or Meta on
# a press arms its own tumble/truck/zoom.
_CAMERA_MODIFIERS = (QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier)


def StageView(usdviewApi):
    # Same 6 lines as gizmoUI.StageView: the private-name mangling is
    # usdview's own (UsdviewApi keeps the app controller as
    # __appController).
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


def _EventPoint(event):
    """A mouse event's widget-local position as an (x, y) tuple."""
    try:
        point = event.position()
    except AttributeError:                            # PySide2
        point = event.pos()
    return (point.x(), point.y())


def _QPolygon(points):
    """Widget-logical points as a QPolygonF for the painter."""
    return QtGui.QPolygonF([QtCore.QPointF(p[0], p[1])
                            for p in points])


def _FaceArea(points):
    """Unsigned area of a projected polygon, in square pixels."""
    total = 0.0
    for index in range(len(points)):
        here, there = points[index], points[(index + 1) % len(points)]
        total += here[0] * there[1] - there[0] * here[1]
    return abs(total) * 0.5


# ---------------------------------------------------------------------------
# Widget
# ---------------------------------------------------------------------------

class ViewCubeWidget(QtWidgets.QWidget):
    """
    The cube itself, painted over the stage view.

    A mouse-accepting child widget (NOT WA_TransparentForMouseEvents,
    and deliberately NOT WA_NoMousePropagation): presses it handles
    are accept()ed so the cube keeps them, and everything it ignore()s
    propagates to the stage view, which is what lets usdview's own
    Alt-drag keep working over the cube. Qt makes the child under the
    cursor the implicit mouse grabber on the initial press even when
    the child ignore()s it, so every following button-down move and
    the release reach the cube first and propagate only if the cube
    ignores them too.
    """

    def __init__(self, controller, parent):
        super(ViewCubeWidget, self).__init__(parent)
        self._controller = controller
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        # WA_NoSystemBackground plus no auto-fill is what leaves the
        # stage view showing through (gizmoUI.py:183-187);
        # WA_TranslucentBackground would do nothing on a child.
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        self.setAutoFillBackground(False)
        self.setFixedSize(int(viewCubeMath.WIDGET_SIZE),
                          int(viewCubeMath.WIDGET_SIZE))
        self._hover = None
        self._lastPoint = None
        self._opacity = IDLE_OPACITY
        # Gesture state: _pressPoint is None whenever no press is ours.
        self._pressPoint = None
        self._lastDragPoint = None
        self._dragging = False
        self._dragCam = None

    def Hover(self):
        """The hovered region name, "home", or None."""
        return self._hover

    def _ClearPress(self):
        self._pressPoint = None
        self._lastDragPoint = None
        self._dragging = False
        self._dragCam = None

    # -- mouse --------------------------------------------------------

    def mousePressEvent(self, event):
        self._lastPoint = _EventPoint(event)
        if event.button() != QtCore.Qt.LeftButton or \
                event.modifiers() & _CAMERA_MODIFIERS:
            # usdview's claim (stageView.py:2099); the propagated press
            # arms usdview's own tumble/truck/zoom (:2079-2115).
            event.ignore()
            return
        if self._controller is None:
            event.ignore()
            return
        point = _EventPoint(event)
        if self._controller.RegionAt(point) is None and \
                not self._controller.HomeRect().contains(
                    QtCore.QPointF(point[0], point[1])):
            # Empty widget background: not ours. ignore() so the
            # press propagates to StageView.mousePressEvent
            # (stageView.py:2089, :2109-2110) and usdview's prim
            # pick still works in the corner the cube sits in.
            event.ignore()
            return
        self._controller.CancelAnimation()
        self._pressPoint = point
        self._lastDragPoint = point
        self._dragging = False
        self._dragCam = None
        event.accept()

    def mouseMoveEvent(self, event):
        point = _EventPoint(event)
        self._lastPoint = point
        if event.buttons() != QtCore.Qt.NoButton and \
                self._pressPoint is None:
            # The press was not ours: ignore() so usdview's
            # mouseMoveEvent (stageView.py:2121-2166) keeps navigating.
            # A handler that merely returns leaves the event accepted
            # and would swallow usdview's Alt-drag over the cube.
            event.ignore()
            return
        if self._pressPoint is not None and \
                event.buttons() & QtCore.Qt.LeftButton:
            self._MoveDragged(point)
            event.accept()
            return
        if event.buttons() == QtCore.Qt.NoButton:
            # Accept, so the stage view never sees a button-less move:
            # its "none" camera mode would run a GPU pickObject per
            # move (stageView.py:2121-2166).
            self._UpdateHover(point)
            event.accept()
            return
        event.ignore()

    def _MoveDragged(self, point):
        """One button-down move of our own gesture, in order (no elif).
        """
        press = self._pressPoint
        if not self._dragging and \
                math.hypot(point[0] - press[0],
                           point[1] - press[1]) < DRAG_THRESHOLD:
            # Inside the dead zone: accept without touching
            # _lastDragPoint, so the move that crosses the threshold
            # tumbles by its full offset from the press point.
            return
        controller = self._controller
        if controller is None or controller._view is None:
            self._ClearPress()
            return
        if not self._dragging:
            self._dragging = True
            self._dragCam = controller._FreeCamera()
            if self._dragCam is None:
                self._ClearPress()
                return
        last = self._lastDragPoint
        self._lastDragPoint = point
        ratio = controller._ViewRatio()
        self._dragCam.Tumble(TUMBLE_RATE * (point[0] - last[0]) *
                             ratio,
                             TUMBLE_RATE * (point[1] - last[1]) *
                             ratio)
        # Tumble emits signalFrustumChanged, which the view settings
        # model forwards (viewSettingsDataModel.py:832-833, :393-398)
        # and the StageView repaints on (stageView.py:855-856,
        # :2184-2188); the explicit updateGL keeps the behaviour from
        # depending on that wiring.
        controller._view.updateGL()
        controller._RefreshBasis()

    def mouseReleaseEvent(self, event):
        self._lastPoint = _EventPoint(event)
        if self._pressPoint is None:
            # Not our gesture: ignore() so usdview's mouseReleaseEvent
            # (stageView.py:2117-2119) clears its _cameraMode and
            # _dragActive instead of leaving the view armed.
            event.ignore()
            return
        press = self._pressPoint
        dragging = self._dragging
        self._ClearPress()
        controller = self._controller
        if controller is None:
            event.accept()
            return
        if not dragging and event.button() == QtCore.Qt.LeftButton:
            # A release that never crossed the threshold is a click on
            # the region (or the home glyph) under the PRESS point.
            if controller.HomeRect().contains(
                    QtCore.QPointF(press[0], press[1])):
                controller.Home()
            else:
                region = controller.RegionAt(press)
                if region is not None:
                    controller.Orbit(region.name)
        event.accept()

    def _UpdateHover(self, point):
        controller = self._controller
        if controller is None:
            hover = None
        elif controller.HomeRect().contains(
                QtCore.QPointF(point[0], point[1])):
            hover = "home"
        else:
            region = controller.RegionAt(point)
            hover = region.name if region is not None else None
        if hover != self._hover:
            self._hover = hover
            if hover is None:
                self.setCursor(QtCore.Qt.ArrowCursor)
            else:
                self.setCursor(QtCore.Qt.PointingHandCursor)
            self.update()

    def RefreshHover(self):
        """Recompute the hover after the camera moved under a still
        cursor (a click's orbit, a tumble): the highlight and the
        pointing hand must follow the pixel, not the region."""
        # A hidden cube must not keep a highlight armed for its
        # return: an orbit while hidden would paint it on show.
        if not self.isVisible():
            return
        if self._lastPoint is not None:
            self._UpdateHover(self._lastPoint)

    def enterEvent(self, event):
        self._opacity = 1.0
        # The cursor's position, never the event's: Qt
        # static_casts an Enter to QEnterEvent before
        # dispatch, so reading position() off a plain
        # QEvent(Enter) segfaults rather than raising.
        # Qt sends no synthetic move on enter or show, so
        # without this the cube would come back with no
        # highlight under a resting cursor.
        point = self.mapFromGlobal(QtGui.QCursor.pos())
        if self.rect().contains(point):
            self._lastPoint = (point.x(), point.y())
            self._UpdateHover(self._lastPoint)
        self.update()

    def leaveEvent(self, event):
        self._hover = None
        self._lastPoint = None
        self._opacity = IDLE_OPACITY
        self.setCursor(QtCore.Qt.ArrowCursor)
        self.update()

    def hideEvent(self, event):
        # Every hide path clears here, not in SetVisible:
        # widget.hide() (what the stage view or window does)
        # never reaches the controller, and a stale hover
        # would repaint on show. Opacity resets too, or a
        # cube hidden under the cursor comes back bright
        # with nothing under it until the next Leave.
        self._hover = None
        self._lastPoint = None
        self._opacity = IDLE_OPACITY
        self.setCursor(QtCore.Qt.ArrowCursor)

    # -- painting -----------------------------------------------------

    def paintEvent(self, event):
        controller = self._controller
        if controller is None:
            return
        basis = controller.Basis()
        isZUp = controller.IsZUp()
        radius = controller.CubeRadius()
        centre = controller.CubeCentre()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        try:
            painter.setOpacity(self._opacity)
            visible = [face for face in viewCubeMath.ProjectCube(
                basis, isZUp, radius, centre) if face.visible]
            for face in visible:
                painter.setBrush(QtGui.QBrush(
                    QtGui.QColor(_FACE_FILL)))
                painter.setPen(QtCore.Qt.NoPen)
                painter.drawPolygon(_QPolygon(face.polygon))
            if self._hover is not None and self._hover != "home":
                region = viewCubeMath.REGIONS.get(self._hover)
                if region is not None:
                    polygons = viewCubeMath.RegionPolygons(
                        region, basis, isZUp, radius, centre)
                    painter.setBrush(QtGui.QBrush(
                        QtGui.QColor(_HOVER_FILL)))
                    painter.setPen(QtCore.Qt.NoPen)
                    for polygon in polygons:
                        painter.drawPolygon(_QPolygon(polygon))
            pen = QtGui.QPen(QtGui.QColor(_EDGE_COLOR))
            pen.setWidthF(1.0)
            painter.setPen(pen)
            painter.setBrush(QtCore.Qt.NoBrush)
            for face in visible:
                painter.drawPolygon(_QPolygon(face.polygon))
            for face in visible:
                self._DrawLabel(painter, face)
            self._DrawHome(painter)
        finally:
            painter.end()

    def _DrawLabel(self, painter, face):
        """The face name on its face, upright in its canonical view."""
        if _FaceArea(face.polygon) < _LABEL_MIN_AREA:
            # Too edge-on for upright text; the cube still reads
            # through the faces that face the camera.
            return
        target = _QPolygon(face.polygon)
        quad = QtGui.QTransform.squareToQuad(target)
        if quad is None:
            return
        painter.save()
        try:
            painter.setTransform(quad, True)
            painter.scale(1.0 / LABEL_BOX, 1.0 / LABEL_BOX)
            font = QtGui.QFont()
            font.setBold(True)
            font.setPixelSize(LABEL_PIXELS)
            painter.setFont(font)
            painter.setPen(QtGui.QColor(_LABEL_COLOR))
            painter.drawText(QtCore.QRectF(0.0, 0.0, LABEL_BOX,
                                           LABEL_BOX),
                             QtCore.Qt.AlignCenter,
                             viewCubeMath.FACE_LABELS[face.name])
        finally:
            painter.restore()

    def _DrawHome(self, painter):
        """A house outline in the widget's top-left corner."""
        rect = self._controller.HomeRect()
        house = QtGui.QPolygonF([
            QtCore.QPointF(rect.left() + 2.0, rect.bottom() - 1.0),
            QtCore.QPointF(rect.left() + 2.0, rect.top() + 6.0),
            QtCore.QPointF(rect.center().x(), rect.top() + 1.0),
            QtCore.QPointF(rect.right() - 2.0, rect.top() + 6.0),
            QtCore.QPointF(rect.right() - 2.0,
                            rect.bottom() - 1.0)])
        if self._hover == "home":
            painter.setBrush(QtGui.QBrush(
                QtGui.QColor(_HOVER_FILL)))
        else:
            painter.setBrush(QtCore.Qt.NoBrush)
        pen = QtGui.QPen(QtGui.QColor(_LABEL_COLOR))
        pen.setWidthF(1.5)
        pen.setJoinStyle(QtCore.Qt.MiterJoin)
        painter.setPen(pen)
        painter.drawPolygon(house)


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class ViewCubeController(QtCore.QObject):
    """
    The camera behind the cube: its basis, its orbits, its placement.

    Everything the viewport does goes through here: the widget only
    reads state and forwards gestures, and the container only installs
    and toggles.
    """

    def __init__(self, usdviewApi, parent=None):
        super(ViewCubeController, self).__init__(parent)
        self.usdviewApi = usdviewApi
        self.animationMs = ANIMATION_MS
        self._isZUp = False
        self._basis = viewCubeMath.IDENTITY_BASIS
        self._view = None
        self._widget = None
        self._animTimer = None
        self._animClock = None
        self._animStart = None
        self._animTarget = None
        view = StageView(usdviewApi)
        self._view = view
        if view is None:
            return
        widget = ViewCubeWidget(self, view)
        self._widget = widget
        widget.resize(int(viewCubeMath.WIDGET_SIZE),
                      int(viewCubeMath.WIDGET_SIZE))
        # Resize/Show re-place the widget (and re-raise it on a
        # zero-length timer so it ends above the gizmo overlay, which
        # raises itself synchronously in its own Resize handler).
        view.installEventFilter(self)
        view.signalFrustumChanged.connect(self._onFrustumChanged)
        view.destroyed.connect(self._onViewDestroyed)
        model = usdviewApi.dataModel
        model.signalStageReplaced.connect(self._onStageReplaced)
        model.viewSettings.signalSettingChanged.connect(
            self._onSettingChanged)
        self._isZUp = self._ReadUpAxis()
        # Harmless while the view is still hidden: a child is shown
        # with its parent.
        self._Place()
        widget.show()
        self._RefreshBasis()

    # -- state ----------------------------------------------------------

    def Widget(self):
        """The cube widget, or None once detached."""
        return self._widget

    def IsVisible(self):
        widget = self._widget
        return widget is not None and widget.isVisible()

    def SetVisible(self, visible):
        # Hover clearing lives in hideEvent, so every hide
        # path disarms it, not just this one.
        if self._widget is not None:
            self._widget.setVisible(bool(visible))

    def IsZUp(self):
        """Whether the stage is Z-up; False with no stage."""
        return self._isZUp

    def Basis(self):
        """The world-space camera frame the cube is oriented by."""
        return self._basis

    def CubeCentre(self):
        """The cube centre in widget-local logical px."""
        half = viewCubeMath.WIDGET_SIZE / 2.0
        return (half, half)

    def CubeRadius(self):
        """The face half-size in logical px at view-space z = 0."""
        return viewCubeMath.RADIUS_FRACTION * \
            viewCubeMath.WIDGET_SIZE

    def RegionAt(self, point):
        """The Region under a widget-local point, or None on a miss."""
        if self._view is None:
            return None
        return viewCubeMath.HitTest(self._basis, self._isZUp,
                                    self.CubeRadius(),
                                    self.CubeCentre(), point)

    def HoverRegion(self):
        """The hovered region name, "home", or None."""
        widget = self._widget
        if widget is None:
            return None
        return widget.Hover()

    def ScreenPointForRegion(self, name):
        """A widget-local point inside the region, or None when none
        of its faces is visible."""
        if self._view is None:
            return None
        region = viewCubeMath.REGIONS.get(name)
        if region is None:
            return None
        return viewCubeMath.RegionPoint(region, self._basis,
                                        self._isZUp,
                                        self.CubeRadius(),
                                        self.CubeCentre())

    def HomeRect(self):
        """The home glyph rect in widget-local logical px."""
        size = viewCubeMath.HOME_GLYPH_SIZE
        return QtCore.QRectF(4.0, 4.0, size, size)

    def HomePoint(self):
        """A widget-local point on the home glyph."""
        centre = self.HomeRect().center()
        return (centre.x(), centre.y())

    def CurrentAngles(self):
        """The free camera's (rotTheta, rotPhi), or None."""
        if self._view is None:
            return None
        cam = self.usdviewApi.dataModel.viewSettings.freeCamera
        if cam is None:
            return None
        return (cam.rotTheta, cam.rotPhi)

    def IsAnimating(self):
        return self._animTimer is not None and \
            self._animTimer.isActive()

    def _ViewRatio(self):
        view = self._view
        if view is None:
            return 1.0
        try:
            return float(view.devicePixelRatioF())
        except AttributeError:
            return 1.0

    # -- camera ---------------------------------------------------------

    def _ReadUpAxis(self):
        # The data model emits signalStageReplaced with stage None from
        # appController._closeStage() (window close,
        # appController.py:2812; File > Reopen, :3001) and
        # UsdGeom.GetStageUpAxis(None) raises Tf.ErrorException, which
        # PySide6 would print and abandon the slot on. Never call
        # GetStageUpAxis on an unchecked stage.
        stage = self.usdviewApi.dataModel.stage
        return bool(stage) and \
            UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z

    def _RefreshBasis(self):
        view = self._view
        if view is None:
            return
        try:
            camera, _aspect = view.resolveCamera()
        except Exception:
            # No renderer yet: keep the last basis, which before any
            # camera has resolved is the identity -- so the cube reads
            # as FRONT until the first frustum change refreshes it.
            return
        self._basis = viewCubeMath.BasisFromFrustum(camera.frustum)
        if self._widget is not None:
            self._widget.RefreshHover()
            self._widget.update()

    def _FreeCamera(self):
        """The free camera, switching to it first; None with a warn."""
        view = self._view
        if view is None:
            return None
        # A no-op while the free camera is already active; converts a
        # prim camera into an equivalent free one otherwise.
        view.switchToFreeCamera()
        cam = self.usdviewApi.dataModel.viewSettings.freeCamera
        if cam is None:
            Tf.Warn("rigExecUsdview: view cube orbit dropped: "
                    "no free camera")
        return cam

    def _ApplyAngles(self, theta, phi):
        view = self._view
        if view is None:
            return
        cam = self.usdviewApi.dataModel.viewSettings.freeCamera
        if cam is None:
            return
        cam.rotTheta = theta
        cam.rotPhi = phi
        # FreeCamera has no public roll setter (freeCamera.py:106-131);
        # a rolled free camera only arises from a rolled camera prim
        # through FromGfCamera, so clear it under a hasattr guard.
        if hasattr(cam, "_rotPsi") and cam._rotPsi:
            cam._rotPsi = 0.0
            cam._cameraTransformDirty = True
        # Assigning the angles already repaints the viewport: the view
        # settings model forwards the free camera's
        # signalFrustumChanged as signalFreeCameraSettingChanged
        # (viewSettingsDataModel.py:832-833, :393-398) and the
        # StageView repaints on it (stageView.py:855-856, :2184-2188).
        # The explicit updateGL keeps the behaviour from depending on
        # that wiring.
        view.updateGL()
        self._RefreshBasis()

    def OrbitTo(self, theta, phi):
        """Orbit to (theta, phi): animated, or immediate when
        animationMs <= 0."""
        self.CancelAnimation()
        if self._FreeCamera() is None:
            return          # _FreeCamera() already warned
        start = self.CurrentAngles()
        if start is None:
            return          # unreachable once _FreeCamera() returned
                            # a camera; kept as a guard
        target = (viewCubeMath.Unwrap(theta, start[0]), phi)
        if self.animationMs <= 0:
            self._ApplyAngles(*target)
            return
        self._animStart = start
        self._animTarget = target
        self._animClock = QtCore.QElapsedTimer()
        self._animClock.start()
        self._animTimer = QtCore.QTimer(self)
        self._animTimer.setInterval(16)
        self._animTimer.timeout.connect(self._onAnimationTick)
        self._animTimer.start()

    def Orbit(self, regionName):
        """Orbit to a region by name (spec section 1: centre and
        distance kept, roll cleared)."""
        region = viewCubeMath.REGIONS[regionName]
        # Switch first (spec 1.3, 4.1): with a camera prim active the
        # free camera is still the pre-prim object, and the TOP/BOTTOM
        # heading snap must read the heading actually being rendered.
        if self._FreeCamera() is None:
            return          # _FreeCamera() already warned
        start = self.CurrentAngles()
        if start is None:
            return
        angles = viewCubeMath.AnglesForDirection(region.direction,
                                                 start[0])
        self.OrbitTo(*angles)

    def Home(self):
        """Frame the selection, then orbit to the home corner view."""
        if self._FreeCamera() is None:
            return          # _FreeCamera() already warned
        view = self._view
        if view is None:
            return
        # What usdview's Frame Selection does
        # (appController.py:2475-2479).
        view.updateView(resetCam=True, forceComputeBBox=True)
        self.Orbit(viewCubeMath.HOME_REGION)

    def CancelAnimation(self):
        timer, self._animTimer = self._animTimer, None
        self._animClock = None
        self._animStart = None
        self._animTarget = None
        if timer is not None:
            timer.stop()
            timer.deleteLater()   # parented to self; stop() alone leaks it

    def _onAnimationTick(self):
        if self._animTimer is None or self._animClock is None:
            return
        span = max(1, int(self.animationMs))
        ratio = min(1.0, self._animClock.elapsed() / float(span))
        eased = viewCubeMath.Smoothstep(ratio)
        if ratio >= 1.0:
            # The last tick writes the exact target; take it before
            # CancelAnimation clears it.
            target = self._animTarget
            self.CancelAnimation()
            self._ApplyAngles(*target)
        else:
            self._ApplyAngles(*viewCubeMath.LerpAngles(
                self._animStart, self._animTarget, eased))

    # -- placement and lifetime -----------------------------------------

    def _Place(self):
        view, widget = self._view, self._widget
        if view is None or widget is None:
            return
        widget.move(int(view.width() - viewCubeMath.WIDGET_SIZE -
                        viewCubeMath.MARGIN),
                    int(viewCubeMath.MARGIN))

    def _RaiseWidget(self):
        # Never raise_() synchronously inside a paint; the Resize/Show
        # filter below reaches this on a zero-length timer instead.
        if self._widget is not None:
            self._widget.raise_()

    def eventFilter(self, watched, event):
        if watched is self._view and self._widget is not None:
            kind = event.type()
            if kind == QtCore.QEvent.Resize or \
                    kind == QtCore.QEvent.Show:
                self._Place()
                QtCore.QTimer.singleShot(0, self._RaiseWidget)
        return super(ViewCubeController, self).eventFilter(watched,
                                                           event)

    def _onFrustumChanged(self):
        self._RefreshBasis()

    def _onSettingChanged(self):
        # A camera-prim switch is a view setting
        # (viewSettingsDataModel.py:842-848) and emits no frustum
        # signal of its own until the next paint; refreshing the basis
        # on it keeps the cube honest without waiting for Storm.
        self._RefreshBasis()

    def _onStageReplaced(self):
        self.CancelAnimation()
        self._isZUp = self._ReadUpAxis()
        self._RefreshBasis()

    def _onViewDestroyed(self, _obj=None):
        # The view's C++ object is already gone: disconnecting signals
        # from it would print libpyside warnings, and its child widget
        # dies with it, so just drop the references. Every later call
        # is a no-op through the None guards.
        self.CancelAnimation()
        self._view = None
        widget, self._widget = self._widget, None
        if widget is not None:
            widget._controller = None

    def Detach(self):
        """Drop the widget and the timer; every later call is a no-op.
        """
        self.CancelAnimation()
        view, self._view = self._view, None
        if view is not None:
            try:
                view.removeEventFilter(self)
            except Exception:
                pass
            try:
                view.signalFrustumChanged.disconnect(
                    self._onFrustumChanged)
            except Exception:
                pass
            try:
                view.destroyed.disconnect(self._onViewDestroyed)
            except Exception:
                pass
        widget, self._widget = self._widget, None
        if widget is not None:
            widget._controller = None
            try:
                widget.deleteLater()
            except Exception:
                pass


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


def InstallViewCube(usdviewApi, retries=_INSTALL_RETRIES):
    """
    Put the view cube on usdview's stage view once.

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
                InstallViewCube(usdviewApi, retries - 1)

            QtCore.QTimer.singleShot(0, _Retry)
        return None
    _controller = ViewCubeController(usdviewApi)
    return _controller


def GetController():
    """The installed controller, or None."""
    return _controller
