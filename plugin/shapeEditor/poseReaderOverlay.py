#
# The pose-reader visualization, painted over usdview's stage view.
#
# The Qt half of poseReaderModel. Everything that decides WHAT is drawn --
# the cones, fans and spheres, their projection, their silhouettes and
# their colours -- happens there and is tested headlessly; this file only
# turns its DrawOps into QPainter calls.
#
# OVERLAY APPROACH: the same as the manipulator's (gizmoUI.GizmoOverlay):
# a transparent, mouse-transparent CHILD of the stage view, kept exactly
# over it. Drawing in the overlay rather than authoring guide prims keeps
# the view non-authoring and off the evaluator's notice path, and means a
# camera orbit costs a repaint, never a re-evaluation. The price is that
# the shapes are not depth-tested against the body; they draw on top,
# which is what a debugging view of something inside the mesh wants.
#
try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError:
    from PySide2 import QtCore, QtGui, QtWidgets

from pxr import Gf

import poseReaderModel as viz


def StageView(usdviewApi):
    """usdview's stage view widget, or None (headless, or no view yet).

    The private-name mangling is usdview's own; the manipulator reaches
    the view the same way.
    """
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


def _Color(rgb, alpha):
    return QtGui.QColor(int(round(rgb[0] * 255)), int(round(rgb[1] * 255)),
                        int(round(rgb[2] * 255)),
                        int(round(max(0.0, min(1.0, alpha)) * 255)))


class PoseReaderOverlay(QtWidgets.QWidget):
    """Paints the controller's DrawOps; never takes a click."""

    def __init__(self, controller, parent):
        super(PoseReaderOverlay, self).__init__(parent)
        self._controller = controller
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        # No system background and no auto-fill is what lets the stage
        # view show through; WA_TranslucentBackground is for top-levels.
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        self.setAutoFillBackground(False)
        self.setFocusPolicy(QtCore.Qt.NoFocus)

    def _Ratio(self):
        parent = self.parentWidget()
        try:
            return float(parent.devicePixelRatioF()) if parent else 1.0
        except AttributeError:
            return 1.0

    def paintEvent(self, event):
        ops = self._controller.Ops()
        if not ops:
            return
        ratio = self._Ratio()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        try:
            for op in ops:
                self._Paint(painter, op, ratio)
        finally:
            painter.end()

    def _Pen(self, op):
        if op.color is None or op.alpha <= 0.0:
            return QtGui.QPen(QtCore.Qt.NoPen)
        pen = QtGui.QPen(_Color(op.color, op.alpha))
        pen.setWidthF(op.width)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        if op.dashed:
            pen.setStyle(QtCore.Qt.DashLine)
        return pen

    def _Brush(self, op):
        if op.fill is None or op.fillAlpha <= 0.0:
            return QtGui.QBrush(QtCore.Qt.NoBrush)
        return QtGui.QBrush(_Color(op.fill, op.fillAlpha))

    def _Paint(self, painter, op, ratio):
        def point(p):
            return QtCore.QPointF(p[0] / ratio, p[1] / ratio)

        if op.kind in ("polygon", "polyline"):
            if len(op.points) < 2:
                return
            poly = QtGui.QPolygonF([point(p) for p in op.points])
            painter.setPen(self._Pen(op))
            if op.kind == "polygon":
                painter.setBrush(self._Brush(op))
                painter.drawPolygon(poly)
            else:
                painter.setBrush(QtCore.Qt.NoBrush)
                painter.drawPolyline(poly)
        elif op.kind == "circle":
            painter.setPen(self._Pen(op))
            painter.setBrush(self._Brush(op))
            r = op.radius / ratio
            painter.drawEllipse(point(op.centre), r, r)
        elif op.kind == "dot":
            painter.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20, 200), 1.0))
            painter.setBrush(QtGui.QBrush(_Color(op.color, op.alpha)))
            r = op.radius
            painter.drawEllipse(point(op.centre), r, r)
        elif op.kind == "text" and op.text:
            at = point(op.centre) + QtCore.QPointF(6.0, -6.0)
            # A dark halo under the text so it reads on light and dark
            # surfaces alike, without a box hiding the shape under it.
            painter.setPen(QtGui.QColor(0, 0, 0, int(200 * op.alpha)))
            for dx, dy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
                painter.drawText(at + QtCore.QPointF(dx, dy), op.text)
            painter.setPen(_Color(op.color, op.alpha))
            painter.drawText(at, op.text)


class PoseReaderView(QtCore.QObject):
    """Keeps the overlay over the stage view and hands it fresh ops.

    The readers come from the panel (rebuilt when the rig's generation
    moves); the ops are re-projected here whenever the camera or the
    viewport changes, which is cheap -- a camera orbit never touches the
    rig.
    """

    def __init__(self, usdviewApi, parent=None):
        super(PoseReaderView, self).__init__(parent)
        self._api = usdviewApi
        self._readers = []
        self._settings = viz.Settings()
        self._ops = []
        self._view = None
        self.overlay = None
        self._Attach()

    # -- attachment ------------------------------------------------------

    def _Attach(self):
        view = StageView(self._api)
        if view is self._view:
            return
        self._Detach()
        self._view = view
        if view is None:
            return
        self.overlay = PoseReaderOverlay(self, view)
        view.installEventFilter(self)
        view.destroyed.connect(self._OnViewDestroyed)
        try:
            view.signalFrustumChanged.connect(self.Reproject)
        except (AttributeError, RuntimeError):
            pass
        self._Sync()

    def _Detach(self):
        view = self._view
        if view is not None:
            try:
                view.removeEventFilter(self)
                view.signalFrustumChanged.disconnect(self.Reproject)
            except (RuntimeError, TypeError, AttributeError):
                pass
        if self.overlay is not None:
            try:
                self.overlay.hide()
                self.overlay.deleteLater()
            except RuntimeError:
                pass
        self.overlay = None
        self._view = None

    def _OnViewDestroyed(self, *args):
        self.overlay = None
        self._view = None

    def Close(self):
        self._Detach()

    def eventFilter(self, obj, event):
        if event.type() in (QtCore.QEvent.Resize, QtCore.QEvent.Show):
            self._Sync()
            self.Reproject()
        elif event.type() == QtCore.QEvent.Paint:
            # Geometry only: Storm repaints on a short timer while a frame
            # converges, and every reason the ops could be stale has its
            # own hook (the frustum signal, Resize, the panel's refresh).
            self._Sync(restack=False)
        return False

    def _Sync(self, restack=True):
        overlay, view = self.overlay, self._view
        if overlay is None or view is None:
            return
        if overlay.geometry() != view.rect():
            overlay.setGeometry(view.rect())
        overlay.setVisible(bool(self._settings.enabled))
        if restack:
            overlay.raise_()

    # -- data ------------------------------------------------------------

    def SetSettings(self, settings):
        self._settings = settings
        self._Attach()
        self._Sync()
        self.Reproject()

    def SetReaders(self, readers):
        self._readers = list(readers)
        self.Reproject()

    def Ops(self):
        return self._ops

    def _Camera(self):
        """(viewProj, viewport, cameraRight) or None."""
        view = self._view
        if view is None:
            return None
        try:
            camera, _ = view.resolveCamera()
            viewport = view.computeWindowViewport()
        except Exception:
            return None
        if camera is None:
            return None
        frustum = camera.frustum
        viewProj = (frustum.ComputeViewMatrix() *
                    frustum.ComputeProjectionMatrix())
        viewDir = Gf.Vec3d(frustum.ComputeViewDirection()).GetNormalized()
        up = Gf.Vec3d(frustum.ComputeUpVector()).GetNormalized()
        right = Gf.Cross(viewDir, up).GetNormalized()
        return viewProj, viewport, right

    def Reproject(self, *args):
        """Rebuild the ops from the readers for the current camera."""
        self._ops = []
        if self._settings.enabled and self._readers:
            camera = self._Camera()
            if camera is not None:
                viewProj, viewport, right = camera
                projector = viz.Projector(viewProj, viewport, right)
                try:
                    self._ops = viz.ScreenOps(self._readers, self._settings,
                                              projector)
                except Exception:
                    self._ops = []
        if self.overlay is not None:
            self.overlay.update()
