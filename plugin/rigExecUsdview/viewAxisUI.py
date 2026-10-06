#
# RigExec usdview plugin: a navigation axis gizmo in the viewport's
# bottom-left corner: the world X, Y and Z
# axes as coloured balls around a circle. Click a ball to look down that
# axis; drag inside the circle to tumble the view. Toggled from
# RigExec -> Viewport -> View Axis.
#
# The numbers live in viewAxisMath (no Qt). The camera side -- the free
# camera, the animated orbit, tumbling, following the frustum, install
# timing -- is the view cube's controller, reused through its widget
# factory, so the two gizmos move the camera identically.
#
import math
import os
import sys

from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import viewAxisMath
    import viewCubeMath
    import viewCubeUI
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import viewAxisMath
    import viewCubeMath
    import viewCubeUI


def _Color(rgba):
    return QtGui.QColor.fromRgbF(*[max(0.0, min(1.0, c)) for c in rgba])


def _LinearToDisplay(value):
    # usdview's clear colours are linear and drawn through sRGB, so the
    # grey on screen is the encoded value; the gizmo fades toward that.
    if value <= 0.0031308:
        return 12.92 * value
    return 1.055 * math.pow(value, 1.0 / 2.4) - 0.055


def _EventPoint(event):
    return viewCubeUI._EventPoint(event)


class ViewAxisWidget(QtWidgets.QWidget):
    """The gizmo, painted over the stage view.

    Takes presses on the circle and lets everything else (outside the
    circle, and Alt/Meta camera gestures) through to usdview, the same
    way the view cube does.
    """

    def __init__(self, controller, parent):
        super(ViewAxisWidget, self).__init__(parent)
        self._controller = controller
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        self.setAutoFillBackground(False)
        size = int(viewAxisMath.WIDGET_SIZE)
        self.setFixedSize(size, size)
        self._hover = None
        self._lastPoint = None
        self._pressPoint = None
        self._lastDragPoint = None
        self._dragging = False
        self._dragCam = None

    def Centre(self):
        half = viewAxisMath.WIDGET_SIZE / 2.0
        return (half, half)

    def Hover(self):
        """The hovered handle name ("+x", ...), "background", or None."""
        return self._hover

    def HitTest(self, point):
        controller = self._controller
        if controller is None:
            return None
        return viewAxisMath.HitTest(controller.Basis(), self.Centre(), point)

    def _ClearPress(self):
        self._pressPoint = None
        self._lastDragPoint = None
        self._dragging = False
        self._dragCam = None

    # -- mouse --------------------------------------------------------

    def mousePressEvent(self, event):
        point = _EventPoint(event)
        self._lastPoint = point
        if event.button() != QtCore.Qt.LeftButton or \
                event.modifiers() & viewCubeUI._CAMERA_MODIFIERS or \
                self._controller is None or self.HitTest(point) is None:
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
            event.ignore()
            return
        if self._pressPoint is not None and \
                event.buttons() & QtCore.Qt.LeftButton:
            self._MoveDragged(point)
            event.accept()
            return
        if event.buttons() == QtCore.Qt.NoButton:
            self._UpdateHover(point)
            if self._hover is None:
                event.ignore()
            else:
                event.accept()
            return
        event.ignore()

    def _MoveDragged(self, point):
        press = self._pressPoint
        if not self._dragging and math.hypot(
                point[0] - press[0],
                point[1] - press[1]) < viewCubeUI.DRAG_THRESHOLD:
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
            self._hover = "background"
        last = self._lastDragPoint
        self._lastDragPoint = point
        ratio = controller._ViewRatio()
        self._dragCam.Tumble(
            viewCubeUI.TUMBLE_RATE * (point[0] - last[0]) * ratio,
            viewCubeUI.TUMBLE_RATE * (point[1] - last[1]) * ratio)
        controller._view.updateGL()
        controller._RefreshBasis()

    def mouseReleaseEvent(self, event):
        self._lastPoint = _EventPoint(event)
        if self._pressPoint is None:
            event.ignore()
            return
        press, dragging = self._pressPoint, self._dragging
        self._ClearPress()
        controller = self._controller
        if controller is not None and not dragging and \
                event.button() == QtCore.Qt.LeftButton:
            hit = self.HitTest(press)
            if isinstance(hit, viewAxisMath.Handle):
                controller.LookDown(hit)
        self._UpdateHover(self._lastPoint)
        event.accept()

    def _UpdateHover(self, point):
        hit = self.HitTest(point) if point is not None else None
        hover = hit.name if isinstance(hit, viewAxisMath.Handle) else hit
        if hover != self._hover:
            self._hover = hover
            self.setCursor(QtCore.Qt.PointingHandCursor
                           if hover not in (None, "background")
                           else QtCore.Qt.ArrowCursor)
            self.update()

    def RefreshHover(self):
        """Keep the highlight under a still cursor after the camera moved."""
        if self.isVisible() and self._lastPoint is not None and \
                self._pressPoint is None:
            self._UpdateHover(self._lastPoint)

    def leaveEvent(self, event):
        self._hover = None
        self._lastPoint = None
        self.setCursor(QtCore.Qt.ArrowCursor)
        self.update()

    def hideEvent(self, event):
        self._hover = None
        self._lastPoint = None
        self.setCursor(QtCore.Qt.ArrowCursor)

    # -- painting -----------------------------------------------------

    def _Background(self):
        controller = self._controller
        try:
            linear = controller.usdviewApi.dataModel.viewSettings.clearColor
            return tuple(_LinearToDisplay(c) for c in linear[:3]) + (1.0,)
        except Exception:
            return (0.3, 0.3, 0.3, 1.0)

    def paintEvent(self, event):
        controller = self._controller
        if controller is None:
            return
        centre = self.Centre()
        handles = viewAxisMath.Handles(controller.Basis(), centre)
        aligned = viewAxisMath.AlignedAxis(handles)
        background = self._Background()
        active = self._hover is not None
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setRenderHint(QtGui.QPainter.TextAntialiasing, True)
        try:
            if active:
                painter.setPen(QtCore.Qt.NoPen)
                painter.setBrush(_Color(viewAxisMath.HIGHLIGHT_COLOR))
                painter.drawEllipse(QtCore.QPointF(*centre),
                                    viewAxisMath.RADIUS,
                                    viewAxisMath.RADIUS)
            font = QtGui.QFont()
            font.setBold(True)
            font.setPixelSize(max(1, int(round(viewAxisMath.TEXT_SIZE))))
            painter.setFont(font)
            for handle in handles:
                self._DrawHandle(painter, handle, aligned, background,
                                 centre, active)
        finally:
            painter.end()

    def _DrawHandle(self, painter, handle, aligned, background, centre,
                    active):
        style = viewAxisMath.HandleStyle(handle, aligned, background)
        end = QtCore.QPointF(*handle.screen)
        if style["drawLine"]:
            # From just behind the centre to the ball's inner edge.
            lead = viewAxisMath.LINE_WIDTH * 0.66
            reach = viewAxisMath.RADIUS * (1.0 - viewAxisMath.HANDLE_SIZE)
            dx, dy = handle.x, -handle.y
            start = QtCore.QPointF(centre[0] - dx * lead,
                                   centre[1] - dy * lead)
            stop = QtCore.QPointF(
                centre[0] + dx * reach * (1.0 - viewAxisMath.HANDLE_SIZE),
                centre[1] + dy * reach * (1.0 - viewAxisMath.HANDLE_SIZE))
            gradient = QtGui.QLinearGradient(start, stop)
            gradient.setColorAt(0.0, _Color(style["lineStart"]))
            gradient.setColorAt(1.0, _Color(style["lineEnd"]))
            pen = QtGui.QPen(QtGui.QBrush(gradient),
                             viewAxisMath.LINE_WIDTH)
            pen.setCapStyle(QtCore.Qt.RoundCap)
            painter.setPen(pen)
            painter.drawLine(start, stop)
        if style["drawBall"]:
            pen = QtGui.QPen(_Color(style["outline"]),
                             viewAxisMath.RING_WIDTH)
            painter.setPen(pen)
            painter.setBrush(_Color(style["inner"]))
            radius = style["ballRadius"] - viewAxisMath.RING_WIDTH / 2.0
            painter.drawEllipse(end, radius, radius)
        highlighted = self._hover == handle.name
        if style["drawBall"] and (style["drawLabel"] or highlighted):
            color = (1.0, 1.0, 1.0, 1.0) if highlighted else \
                (0.0, 0.0, 0.0, 1.0 if active else 0.9)
            painter.setPen(_Color(color))
            box = style["ballRadius"] * 2.0
            painter.drawText(
                QtCore.QRectF(end.x() - box, end.y() - box / 2.0,
                              box * 2.0, box),
                QtCore.Qt.AlignCenter, handle.label)


class ViewAxisController(viewCubeUI.ViewCubeController):
    """The view cube's camera controller, driving the axis gizmo."""

    def _MakeWidget(self, view):
        return ViewAxisWidget(self, view)

    def _Place(self):
        view, widget = self._view, self._widget
        if view is None or widget is None:
            return
        widget.move(int(viewAxisMath.MARGIN),
                    int(view.height() - viewAxisMath.WIDGET_SIZE -
                        viewAxisMath.BOTTOM_MARGIN))

    def HandlePoint(self, name):
        """A widget-local point on a handle, or None when hidden."""
        widget = self._widget
        if widget is None:
            return None
        for handle in viewAxisMath.Handles(self.Basis(), widget.Centre()):
            if handle.name == name:
                return handle.screen
        return None

    def LookDown(self, handle):
        """Orbit so the camera sits on `handle`'s axis, looking back."""
        if self._FreeCamera() is None:
            return
        start = self.CurrentAngles()
        if start is None:
            return
        world = viewAxisMath.ViewDirectionForHandle(handle)
        up = viewCubeMath.WorldToUpSpace(self.IsZUp()).TransformDir(world)
        self.OrbitTo(*viewCubeMath.AnglesForDirection(up, start[0]))

    def LookDownAxis(self, name):
        """LookDown by handle name ("+x", "-z", ...)."""
        for handle in viewAxisMath.Handles(self.Basis(), (0.0, 0.0)):
            if handle.name == name:
                self.LookDown(handle)
                return


_controller = None
_installPending = False


def InstallViewAxis(usdviewApi, retries=viewCubeUI._INSTALL_RETRIES):
    """Put the view axis on usdview's stage view once; see InstallViewCube.
    """
    global _controller, _installPending
    if _controller is not None:
        return _controller
    if viewCubeUI.StageView(usdviewApi) is None:
        if retries > 0 and not _installPending:
            _installPending = True

            def _Retry():
                global _installPending
                _installPending = False
                InstallViewAxis(usdviewApi, retries - 1)

            QtCore.QTimer.singleShot(0, _Retry)
        return None
    _controller = ViewAxisController(usdviewApi)
    return _controller


def GetController():
    """The installed controller, or None."""
    return _controller
