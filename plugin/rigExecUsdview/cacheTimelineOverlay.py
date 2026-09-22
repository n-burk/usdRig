#
# Timeline overlay: per-frame cache states painted over usdview's own
# timeline slider -- a thin Maya-palette band along the slider's bottom
# edge that fills in green as background warming completes frames.
#
# The slider is found, never subclassed, restyled, or repainted: the
# overlay is a transparent child widget with mouse events disabled, so
# scrubbing, tooltips, and the stock style behave exactly as without
# it. An event filter keeps the overlay fitted to the slider's rect,
# and the slider's own rangeChanged repaints when the stage range
# moves. States arrive fed from the container's recurring driver tick
# (no C call of its own); colors and state ints come from
# cacheStripModel, shared with the Cache Strip dialog.
#
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

import os
import sys

try:
    import cacheStripModel
except ImportError:  # pragma: no cover - plugin path, not test path
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import cacheStripModel


def TimelineSlider(qMainWindow):
    """usdview's timeline slider, or None when not found.

    Found by type, not by object name or layout position: the stock
    timeline is the window's FrameSlider. Anything else -- no window,
    an older usdview without the class -- answers None and the
    caller degrades to no overlay rather than touching a widget it
    cannot prove is the timeline.
    """
    try:
        from pxr.Usdviewq import frameSlider
    except Exception:
        return None
    try:
        sliders = qMainWindow.findChildren(frameSlider.FrameSlider)
    except Exception:
        return None
    if not sliders:
        return None
    return sliders[0]


class TimelineOverlay(QtWidgets.QWidget):
    """A mouse-transparent cache band over the timeline slider."""

    _BAND_PX = 5

    def __init__(self, slider):
        super(TimelineOverlay, self).__init__(slider)
        self._states = []
        self._frames = []
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        self.setStyleSheet("background-color: transparent;")
        slider.installEventFilter(self)
        try:
            slider.rangeChanged.connect(self._OnSliderRangeChanged)
        except Exception:
            pass
        self._FitToSlider()

    def _Slider(self):
        return self.parentWidget()

    def _FitToSlider(self):
        slider = self._Slider()
        if slider is None:
            return
        size = slider.size()
        self.setGeometry(0, 0, size.width(), size.height())
        self.raise_()

    def eventFilter(self, watched, event):
        if watched is self._Slider() and event is not None and \
                event.type() == QtCore.QEvent.Resize:
            self._FitToSlider()
        return False

    def _OnSliderRangeChanged(self, _minimum, _maximum):
        self.update()

    def Update(self, states, frames):
        """Repaint for the tick's states; skips when nothing moved.

        Returns True when the widget repainted. A state list whose
        length disagrees with the frames is not paintable -- the C
        query answers both in one call, so a skew means a torn read
        and the last good paint stands.
        """
        states = list(states)
        frames = list(frames)
        if states == self._states and frames == self._frames:
            return False
        if len(states) != len(frames):
            return False
        self._states = states
        self._frames = frames
        self.update()
        if self.isHidden():
            self.show()
        return True

    def Clear(self):
        """Forgets the last paint and hides until the next Update."""
        self._states = []
        self._frames = []
        self.hide()

    def paintEvent(self, event):
        slider = self._Slider()
        states = self._states
        if slider is None or not states or len(states) != len(self._frames):
            return
        try:
            low = int(slider.minimum())
            high = int(slider.maximum())
        except Exception:
            return
        slots = high - low + 1
        if slots <= 0:
            return
        rect = self.rect()
        if rect.width() <= 0 or rect.height() <= 0:
            return
        painter = QtGui.QPainter(self)
        bandTop = max(0, rect.height() - self._BAND_PX)
        bandHeight = rect.height() - bandTop
        for frame, state in zip(self._frames, states):
            if state == cacheStripModel.UNCACHED:
                continue
            try:
                pos = int(round(float(frame))) - low
            except (TypeError, ValueError):
                continue
            if pos < 0 or pos >= slots:
                continue
            left = int(pos * rect.width() / slots)
            right = int((pos + 1) * rect.width() / slots)
            painter.fillRect(left, bandTop, right - left, bandHeight,
                             QtGui.QColor(
                                 cacheStripModel.ColorForState(state)))


def InstallTimelineOverlay(qMainWindow):
    """Overlays the window's timeline slider, once per window.

    Returns the overlay, or None when the slider is not found. A
    second call for the same window returns the installed overlay
    instead of stacking a second band over the first.
    """
    slider = TimelineSlider(qMainWindow)
    if slider is None:
        return None
    try:
        existing = getattr(qMainWindow, "_rigExecTimelineOverlay", None)
    except Exception:
        existing = None
    if existing is not None:
        try:
            existing.show()
        except Exception:
            pass
        return existing
    try:
        overlay = TimelineOverlay(slider)
    except Exception:
        return None
    try:
        qMainWindow._rigExecTimelineOverlay = overlay
    except Exception:
        pass
    return overlay
