#
# RigExec -> General Editors -> Cache Strip: per-frame cached / warming /
# dirty / uncached over the stage range, with clear-cache and warm-range
# actions.
#
# The panel face of the headless `cacheStripPanel` model beside it: one
# poll/skip/action logic behind both the dialog and the ctest suite, so
# the states you see here are the states the tests assert. Colors come
# from cacheStripModel's once-defined Maya palette, never a second copy.
#
# Updates are poll-based, never pushed: the dialog repaints on the
# recurring warming-driver tick (the container forwards each tick with
# its enqueue knowledge) and on SetTime (via currentFrameChanged, which
# the container's own handler runs first -- it connected at plugin load
# and this panel connects when opened). A tick whose completions counter
# is still, with an unchanged playhead and range, skips the repaint.
#
import os
import sys

from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import cacheStripModel
    import cacheStripPanel
except ImportError:  # pragma: no cover - plugin path, not test path
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import cacheStripModel
    import cacheStripPanel


def _FrameFloat(frame):
    # currentFrameChanged hands a plain number, but be liberal: a TimeCode
    # answers GetValue, Default reads as 0.
    try:
        return float(frame)
    except (TypeError, ValueError):
        pass
    try:
        if frame.IsDefault():
            return 0.0
        return float(frame.GetValue())
    except Exception:
        return None


class _StripWidget(QtWidgets.QWidget):
    """One cell per frame, painted in the shared Maya palette."""

    def __init__(self, panel, parent=None):
        super(_StripWidget, self).__init__(parent)
        self._panel = panel
        self.setMinimumHeight(44)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Fixed)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        rect = self.rect()
        model = self._panel._model
        frames = model.frames
        states = model.states
        if not frames or states is None or len(states) != len(frames):
            painter.drawText(rect, QtCore.Qt.AlignCenter,
                             "No warmed range -- open a rig.")
            return
        count = len(frames)
        width = float(rect.width()) / count
        for index, state in enumerate(states):
            color = QtGui.QColor(
                cacheStripModel.ColorForState(state))
            left = int(index * width)
            right = int((index + 1) * width)
            painter.fillRect(left, 0, right - left, rect.height(),
                             color)
        fraction = cacheStripModel.PlayheadFraction(
            frames, model.playhead)
        if fraction is not None:
            x = int(fraction * rect.width())
            painter.setPen(QtGui.QPen(QtCore.Qt.white, 2))
            painter.drawLine(x, 0, x, rect.height())


class CacheStripPanel(QtWidgets.QDialog):

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi, controller=None):
        if cls._instance is None:
            cls._instance = cls(usdviewApi, controller)
        else:
            cls._instance._api = usdviewApi
            if controller is not None:
                cls._instance._controller = controller
        return cls._instance

    def __init__(self, usdviewApi, controller=None, parent=None):
        super(CacheStripPanel, self).__init__(
            parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        self._controller = controller
        self._model = cacheStripPanel.CacheStripPanelModel()
        self.setWindowTitle("RigExec Cache Strip")
        self.resize(760, 220)

        layout = QtWidgets.QVBoxLayout(self)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Rig root:"))
        self._rootBox = QtWidgets.QComboBox()
        self._rootBox.setMinimumWidth(220)
        self._rootBox.currentIndexChanged.connect(
            lambda _i: self._OnRigChanged())
        controls.addWidget(self._rootBox)

        self._clearButton = QtWidgets.QPushButton("Clear cache")
        self._clearButton.clicked.connect(self.ClearCache)
        controls.addWidget(self._clearButton)

        self._warmButton = QtWidgets.QPushButton("Warm range")
        self._warmButton.clicked.connect(self.WarmRange)
        controls.addWidget(self._warmButton)

        controls.addStretch(1)
        layout.addLayout(controls)

        self._strip = _StripWidget(self)
        layout.addWidget(self._strip)

        legend = QtWidgets.QHBoxLayout()
        for role in ("cached", "warming", "dirty", "uncached"):
            swatch = QtWidgets.QLabel()
            swatch.setFixedSize(12, 12)
            swatch.setStyleSheet(
                "background: %s;"
                % cacheStripModel.PALETTE[role])
            legend.addWidget(swatch)
            legend.addWidget(QtWidgets.QLabel(role))
            legend.addSpacing(12)
        legend.addStretch(1)
        layout.addLayout(legend)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        # 3.2 tracking: the playhead and stage replacement. The
        # container's own currentFrameChanged handler runs first (it
        # connected at plugin load), so by the time this fires SetTime
        # and its idle sweep have already landed.
        try:
            dataModel = self._api.dataModel
            dataModel.currentFrameChanged.connect(self._OnFrameChanged)
            dataModel.signalStageReplaced.connect(self._OnStageReplaced)
        except Exception:
            pass

        self.Rebuild()

    # -- data ------------------------------------------------------------

    def _Stage(self):
        model = getattr(self._api, "dataModel", None)
        return model.stage if model else None

    def _RigRoots(self, stage):
        if self._controller is not None:
            try:
                rigs = self._controller.StripRigs()
            except Exception:
                rigs = None
            if rigs:
                return [str(rig) for rig in rigs]
        if stage is None:
            return []
        return sorted(str(prim.GetPath()) for prim in stage.Traverse()
                      if prim.GetTypeName() == "RigExecRoot")

    def _Library(self):
        if self._controller is None:
            return None
        try:
            return self._controller.StripLibrary()
        except Exception:
            return None

    # -- population ------------------------------------------------------

    def Rebuild(self):
        """Refill the rig chooser and range from the open stage."""
        stage = self._Stage()
        self._rootBox.blockSignals(True)
        self._rootBox.clear()
        roots = self._RigRoots(stage)
        for root in roots:
            self._rootBox.addItem(root)
        self._rootBox.blockSignals(False)
        if roots:
            self._model.SetRig(self._rootBox.currentText())
        else:
            self._model.SetRig("")
        if stage is not None:
            try:
                self._model.SetRange(stage.GetStartTimeCode(),
                                     stage.GetEndTimeCode())
            except Exception:
                self._model.SetRange(None, None)
        else:
            self._model.SetRange(None, None)
        try:
            current = self._api.dataModel.currentFrame
        except Exception:
            current = None
        value = _FrameFloat(current)
        if value is not None:
            self._model.SetPlayhead(value)
        self._Poll(cacheStripPanel.REASON_RANGE, True)
        self._RefreshButtons()

    def _OnRigChanged(self):
        if self._model.SetRig(self._rootBox.currentText()):
            self._Poll(cacheStripPanel.REASON_RANGE, True)

    def _OnFrameChanged(self, frame):
        # The SIGNAL's frame, never dataModel.currentFrame: the setter
        # emits before it assigns, so the property still holds the frame
        # the artist just left (see rigExecUsdview._FrameValue).
        value = _FrameFloat(frame)
        if value is not None:
            self._model.SetPlayhead(value)
        self._Poll(cacheStripPanel.REASON_SETTIME, True)

    def _OnStageReplaced(self):
        self.Rebuild()

    # -- polling ---------------------------------------------------------

    def PollTick(self, mayHaveEnqueued=True):
        """One recurring-driver tick: repaint when the poll says so.

        Called by the container with the tick's own enqueue knowledge;
        a hidden panel skips the poll entirely and rebuilds on show.
        """
        if not self.isVisible():
            return
        self._Poll(cacheStripPanel.REASON_TICK, mayHaveEnqueued)

    def _Poll(self, reason, mayHaveEnqueued):
        lib = self._Library()
        if lib is None or not self._model.rigPath:
            self._RefreshStatus(unavailable=True)
            self._strip.update()
            return
        if self._model.Poll(lib, reason, mayHaveEnqueued):
            self._strip.update()
            self._RefreshStatus()

    # -- actions ---------------------------------------------------------

    def ClearCache(self):
        lib = self._Library()
        if lib is None or not self._model.rigPath:
            return
        if self._model.Clear(lib):
            self._WakeDriver()
            self._Poll(cacheStripPanel.REASON_CLEAR, True)
        else:
            self._status.setText("Clear unavailable -- the loaded "
                                 "rigExecImaging has no ClearFrameCache.")

    def WarmRange(self):
        lib = self._Library()
        if lib is None or not self._model.rigPath:
            return
        if self._model.WarmRange(lib):
            self._WakeDriver()
            self._Poll(cacheStripPanel.REASON_WARM, True)
        else:
            self._status.setText("Warm unavailable -- the loaded "
                                 "rigExecImaging has no WarmRange.")

    def _WakeDriver(self):
        if self._controller is None:
            return
        try:
            self._controller.StripWakeDriver()
        except Exception:
            pass

    # -- status ----------------------------------------------------------

    def _RefreshButtons(self):
        enabled = (self._Library() is not None
                   and bool(self._model.rigPath))
        self._clearButton.setEnabled(enabled)
        self._warmButton.setEnabled(enabled)

    def _RefreshStatus(self, unavailable=False):
        if unavailable:
            self._status.setText("No rig active -- open a RigExec stage.")
            return
        frames = self._model.frames
        counts = self._model.Counts()
        completed = self._model.completedCount
        if not frames:
            text = "Empty range."
        else:
            parts = ["%d %s" % (counts.get(role, 0), role)
                     for role in ("cached", "warming", "dirty", "uncached")]
            text = "Frames %d-%d: %s" % (int(frames[0]), int(frames[-1]),
                                         ", ".join(parts))
        if completed is not None:
            text += "  (completed %d)" % completed
        self._status.setText(text)


def OpenCacheStripPanel(usdviewApi, controller=None):
    panel = CacheStripPanel.GetInstance(usdviewApi, controller)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
