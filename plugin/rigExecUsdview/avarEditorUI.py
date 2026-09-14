#
# RigExec usdview plugin: the Avar Editor.
#
# Select a control (or a joint, or anything carrying `avars:*`) in the
# viewport or the prim tree and its avar channels appear here as
# sliders and number fields. Drag one and the rig re-evaluates and the
# viewport follows; let go and the drag is one Ctrl+Z on the shared
# undo stack the viewport gizmo and the graph editor use.
#
# Every rule -- which attributes are channels, what unit they are in,
# how a value is keyed or defaulted, what reset means for an animated
# channel -- lives in avarEditorModel and is tested without Qt. This
# file is the widgets, the selection and frame wiring, and the
# refresh-on-notice loop.
#
# WHY IT AUTHORS PER SLIDER SAMPLE. The gizmo previews a drag through
# Hydra and authors once on release, because a mouse drag produces
# hundreds of samples and the preview lane is only wired for the
# channels the C side knows. A slider sample is rarer than a mouse
# sample, and a custom avar like the biped's `ikfk` has no preview
# lane, so the editor writes each sample to the stage and lets the
# evaluator's own stage notice republish. The undo bracket spans the
# whole drag, so the cost is time, never a polluted undo stack.
#
import os
import sys

from pxr import Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import avarEditorModel as model
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import avarEditorModel as model


# Slider resolution: a full-width drag is this many steps. A thousand
# gives 0.36 degrees or 2 mm per step on the default ranges, which is
# what a slider is for; the spin box beside it is for exact numbers.
SLIDER_STEPS = 1000

_WRITE_MODES = (
    (model.WRITE_ANIMATION, "Animation (key at current frame)"),
    (model.WRITE_DEFAULT, "Default (the rest value)"),
)


def _FrameOf(value):
    if isinstance(value, Usd.TimeCode):
        return value
    try:
        return Usd.TimeCode(float(value))
    except (TypeError, ValueError):
        return Usd.TimeCode.Default()


class ChannelRow(QtCore.QObject):
    """
    One channel's widgets: a name, an editor, a key badge and a reset.

    The editor depends on the value family: a spin box AND a slider for
    numbers, a combo for a token with allowed values, a check box for a
    bool, a line edit for a string. All of them feed the same
    `_Write` so a test can drive any one and hit the panel's real path.
    """

    def __init__(self, panel, channel):
        super(ChannelRow, self).__init__(panel)
        self.panel = panel
        self.channel = channel
        self.name = channel.name
        self._updating = False
        self._scope = None

        self.label = QtWidgets.QLabel(channel.shortName)
        self.label.setToolTip("%s  (%s)" % (
            channel.attr.GetPath(), channel.attr.GetTypeName()))
        if channel.custom:
            font = self.label.font()
            font.setItalic(True)
            self.label.setFont(font)
            self.label.setToolTip(self.label.toolTip() + "\ncustom avar")

        self.spin = None
        self.slider = None
        self.combo = None
        self.check = None
        self.edit = None
        family = channel.family
        if family in (model.VALUE_FLOAT, model.VALUE_INT):
            self._BuildNumeric()
        elif family == model.VALUE_TOKEN and channel.allowedTokens:
            self.combo = QtWidgets.QComboBox()
            for token in channel.allowedTokens:
                self.combo.addItem(token)
            self.combo.currentIndexChanged.connect(self._OnCombo)
        elif family == model.VALUE_BOOL:
            self.check = QtWidgets.QCheckBox()
            self.check.toggled.connect(self._OnCheck)
        else:
            self.edit = QtWidgets.QLineEdit()
            self.edit.editingFinished.connect(self._OnEdit)

        self.unit = QtWidgets.QLabel(channel.unit)
        self.unit.setMinimumWidth(28)

        self.badge = QtWidgets.QLabel("")
        self.badge.setMinimumWidth(34)
        self.badge.setAlignment(QtCore.Qt.AlignCenter)

        self.resetButton = QtWidgets.QToolButton()
        self.resetButton.setText("Reset")
        self.resetButton.setToolTip(
            "Back to the rest value (attr.Clear(); a keyed channel gets "
            "its rest value keyed at this frame in Animation mode)")
        self.resetButton.clicked.connect(self.Reset)

    # -- building --------------------------------------------------------

    def _BuildNumeric(self):
        channel = self.channel
        if channel.family == model.VALUE_FLOAT:
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(channel.Decimals())
            spin.setRange(-1e9, 1e9)
        else:
            spin = QtWidgets.QSpinBox()
            spin.setRange(-2000000000, 2000000000)
        spin.setSingleStep(channel.SpinStep())
        # Only Enter, focus-out and the arrows commit: with keyboard
        # tracking on, typing "30" would author 3 and then 30.
        spin.setKeyboardTracking(False)
        spin.setMinimumWidth(96)
        spin.valueChanged.connect(self._OnSpin)
        self.spin = spin

        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(0, SLIDER_STEPS)
        slider.setMinimumWidth(140)
        slider.sliderPressed.connect(self._OnSliderPressed)
        slider.sliderReleased.connect(self._OnSliderReleased)
        slider.valueChanged.connect(self._OnSlider)
        self.slider = slider
        self._sliderLow, self._sliderHigh = channel.SliderRange(
            self.panel.Stage(), None)

    # -- value mapping ---------------------------------------------------

    def _SliderToValue(self, position):
        low, high = self._sliderLow, self._sliderHigh
        value = low + (high - low) * (float(position) / SLIDER_STEPS)
        if self.channel.family == model.VALUE_INT:
            return int(round(value))
        return value

    def _ValueToSlider(self, value):
        low, high = self._sliderLow, self._sliderHigh
        if high <= low:
            return 0
        t = (float(value) - low) / (high - low)
        return int(round(max(0.0, min(1.0, t)) * SLIDER_STEPS))

    def SliderRange(self):
        return (self._sliderLow, self._sliderHigh)

    # -- refresh ---------------------------------------------------------

    def Refresh(self, time):
        """Show the channel's value at `time` without writing anything."""
        channel = self.channel
        value = channel.Value(time)
        self._updating = True
        try:
            if self.spin is not None:
                if value is not None:
                    span = channel.SliderRange(self.panel.Stage(), value)
                    if span != (self._sliderLow, self._sliderHigh):
                        self._sliderLow, self._sliderHigh = span
                    self.spin.setValue(value)
                    self.slider.setValue(self._ValueToSlider(value))
            elif self.combo is not None:
                text = "" if value is None else str(value)
                index = self.combo.findText(text)
                if index < 0 and text:
                    self.combo.addItem(text)
                    index = self.combo.findText(text)
                self.combo.setCurrentIndex(max(index, 0))
            elif self.check is not None:
                self.check.setChecked(bool(value))
            elif self.edit is not None:
                self.edit.setText("" if value is None else str(value))
        finally:
            self._updating = False
        self._RefreshBadge(time)

    def _RefreshBadge(self, time):
        attr = self.channel.attr
        animated = self.channel.Animated()
        keyedHere = False
        if animated and not time.IsDefault():
            try:
                if attr.GetNumTimeSamples() > 0:
                    keyedHere = time.GetValue() in attr.GetTimeSamples()
                elif attr.HasSpline():
                    # A KnotMap iterates as knot TIMES, not knots.
                    keyedHere = time.GetValue() in list(
                        attr.GetSpline().GetKnots())
            except Exception:
                keyedHere = False
        if keyedHere:
            self.badge.setText("key")
            self.badge.setToolTip("keyed at this frame")
        elif animated:
            self.badge.setText("anim")
            self.badge.setToolTip("animated: keys outrank the default")
        else:
            self.badge.setText("")
            self.badge.setToolTip("")
        self.badge.setStyleSheet(
            "color: #d9534f; font-weight: bold;" if keyedHere else
            "color: #e0a800;" if animated else "")

    # -- writing ---------------------------------------------------------

    def _Write(self, value, bracket=True):
        """
        The one path every widget goes through. `bracket` False means a
        slider drag already opened the undo scope.
        """
        if self._updating:
            return
        if bracket:
            with self.panel.EditScope([self.channel],
                                      "Set %s" % self.channel.shortName):
                warning = self.panel.WriteChannel(self.channel, value)
        else:
            warning = self.panel.WriteChannel(self.channel, value)
        self.panel.ReportWarning(warning)
        self.Refresh(self.panel.Frame())

    def SetValue(self, value):
        """Programmatic edit through the same path a typed value takes."""
        self._Write(value)

    def Reset(self):
        if self._updating:
            return
        with self.panel.EditScope([self.channel],
                                  "Reset %s" % self.channel.shortName):
            warning = self.panel.ResetChannel(self.channel)
        self.panel.ReportWarning(warning)
        self.Refresh(self.panel.Frame())

    def _OnSpin(self, value):
        if self._updating:
            return
        self._Write(value, bracket=self._scope is None)

    def _OnSliderPressed(self):
        if self._updating or self._scope is not None:
            return
        self._scope = self.panel.EditScope(
            [self.channel], "Drag %s" % self.channel.shortName)
        self._scope.__enter__()

    def _OnSlider(self, position):
        if self._updating:
            return
        value = self._SliderToValue(position)
        # Inside a drag the scope is open and the write is bare; a
        # keyboard or wheel change on an undragged slider is its own edit.
        self._Write(value, bracket=self._scope is None)

    def _OnSliderReleased(self):
        scope = self._scope
        self._scope = None
        if scope is not None:
            scope.__exit__(None, None, None)

    def _OnCombo(self, index):
        if self._updating:
            return
        self._Write(self.combo.itemText(index))

    def _OnCheck(self, checked):
        if self._updating:
            return
        self._Write(bool(checked))

    def _OnEdit(self):
        if self._updating:
            return
        self._Write(self.edit.text())


class AvarEditorPanel(QtWidgets.QDialog):
    """The Avar Editor window. One per usdview session."""

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi, undoStack):
        if cls._instance is None:
            cls._instance = cls(usdviewApi, undoStack)
        else:
            cls._instance._api = usdviewApi
            if undoStack is not None:
                cls._instance._undo = undoStack
        return cls._instance

    def __init__(self, usdviewApi, undoStack, parent=None):
        super(AvarEditorPanel, self).__init__(
            parent or getattr(usdviewApi, "qMainWindow", None))
        self._api = usdviewApi
        self._undo = undoStack
        self._rows = []
        self._prim = None
        self._others = 0
        self._writing = False
        self._noticeKey = None
        self._frame = _FrameOf(getattr(usdviewApi, "frame",
                                       Usd.TimeCode.Default()))
        self._mode = model.WRITE_ANIMATION
        self._warning = ""

        self.setWindowTitle("Avar Editor")
        # Tall enough that a control's Custom group (the biped's ikfk
        # dial) is in view under the ten schema channels without a scroll.
        self.resize(600, 660)

        layout = QtWidgets.QVBoxLayout(self)

        self._header = QtWidgets.QLabel("")
        self._header.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse)
        font = self._header.font()
        font.setBold(True)
        self._header.setFont(font)
        layout.addWidget(self._header)

        self._note = QtWidgets.QLabel("")
        self._note.setWordWrap(True)
        layout.addWidget(self._note)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Write:"))
        self._modeBox = QtWidgets.QComboBox()
        for token, label in _WRITE_MODES:
            self._modeBox.addItem(label, token)
        self._modeBox.currentIndexChanged.connect(self._OnModeChanged)
        self._modeBox.setToolTip(
            "Animation keys the current frame (the gizmo's rule: a spline "
            "knot, or a time sample on a channel that has them). Default "
            "writes the rest value; a keyed channel outranks it and the "
            "status line says so.")
        controls.addWidget(self._modeBox)
        controls.addStretch(1)
        self._resetAll = QtWidgets.QPushButton("Reset All")
        self._resetAll.clicked.connect(self.ResetAll)
        controls.addWidget(self._resetAll)
        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self.Rebuild)
        controls.addWidget(refresh)
        layout.addLayout(controls)

        self._scroll = QtWidgets.QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._body = QtWidgets.QWidget()
        self._grid = QtWidgets.QGridLayout(self._body)
        self._grid.setColumnStretch(2, 1)
        self._scroll.setWidget(self._body)
        layout.addWidget(self._scroll, 1)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._Connect()
        self.Rebuild()

    # -- wiring ------------------------------------------------------

    def _Connect(self):
        dataModel = getattr(self._api, "dataModel", None)
        if dataModel is None:
            return
        # signalPrimSelectionChanged emits (added, removed); Rebuild takes
        # *args so it can be the slot directly.
        dataModel.selection.signalPrimSelectionChanged.connect(self.Rebuild)
        dataModel.currentFrameChanged.connect(self._OnFrameChanged)
        dataModel.signalStageReplaced.connect(self._OnStageReplaced)
        self._ObserveStage(dataModel.stage)

    def _ObserveStage(self, stage):
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        if stage:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._OnObjectsChanged, stage)

    def _OnObjectsChanged(self, notice, sender):
        # Our own writes refresh the row that wrote; everything else --
        # a gizmo drag, an undo, a script -- refreshes every row.
        if self._writing:
            return
        if self._prim is not None and not self._prim.IsValid():
            self.Rebuild()
            return
        self.RefreshValues()

    def _OnFrameChanged(self, frame):
        # The SIGNAL's frame, never dataModel.currentFrame: the setter
        # emits before it assigns (rigExecUsdview._FrameValue).
        self._frame = _FrameOf(frame)
        self.RefreshValues()

    def _OnStageReplaced(self):
        self._ObserveStage(self.Stage())
        self._frame = _FrameOf(getattr(self._api, "frame",
                                       Usd.TimeCode.Default()))
        self.Rebuild()

    def closeEvent(self, event):
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        AvarEditorPanel._instance = None
        super(AvarEditorPanel, self).closeEvent(event)

    # -- state -------------------------------------------------------

    def Stage(self):
        dataModel = getattr(self._api, "dataModel", None)
        return dataModel.stage if dataModel else None

    def Frame(self):
        return self._frame

    def Prim(self):
        return self._prim

    def WriteMode(self):
        return self._mode

    def SetWriteMode(self, mode):
        for i in range(self._modeBox.count()):
            if self._modeBox.itemData(i) == mode:
                self._modeBox.setCurrentIndex(i)
                return
        raise ValueError("unknown write mode %r" % mode)

    def _OnModeChanged(self, index):
        self._mode = self._modeBox.itemData(index) or model.WRITE_ANIMATION
        self.RefreshValues()

    def Rows(self):
        return list(self._rows)

    def Row(self, name):
        """The row for `name` ('avars:rz' or just 'rz'), or None."""
        if not name.startswith(model.AVAR_PREFIX):
            name = model.AVAR_PREFIX + name
        for row in self._rows:
            if row.name == name:
                return row
        return None

    # -- build -------------------------------------------------------

    def _Clear(self):
        for row in self._rows:
            row.setParent(None)
        self._rows = []
        while self._grid.count():
            item = self._grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()

    def Rebuild(self, *args):
        self._Clear()
        stage = self.Stage()
        prim, others = (None, 0)
        if stage:
            prim, others = model.FocusPrim(self._api)
        self._prim = prim
        self._others = others
        self._warning = ""
        if prim is None:
            self._header.setText("No prim selected.")
            self._note.setText(
                "Select a control or joint in the viewport or the prim "
                "tree to edit its avars.")
            self._resetAll.setEnabled(False)
            self._SetStatus()
            return
        self._header.setText(str(prim.GetPath()))
        channels, hidden = model.DiscoverChannels(prim, stage)
        notes = []
        if others:
            notes.append("%d more selected; editing the focus prim only."
                         % others)
        if not channels:
            notes.append("%s carries no avars:* channels. Pick a "
                         "RigExecControl or RigExecJoint." % prim.GetName())
        elif hidden:
            notes.append("not shown: %s" % ", ".join(hidden))
        self._note.setText("  ".join(notes))
        self._resetAll.setEnabled(bool(channels))

        rowIndex = 0
        kind = None
        for channel in channels:
            if channel.kind != kind:
                kind = channel.kind
                heading = QtWidgets.QLabel(model.KIND_LABELS[kind])
                font = heading.font()
                font.setBold(True)
                heading.setFont(font)
                self._grid.addWidget(heading, rowIndex, 0, 1, 5)
                rowIndex += 1
            row = ChannelRow(self, channel)
            self._rows.append(row)
            self._grid.addWidget(row.label, rowIndex, 0)
            if row.spin is not None:
                self._grid.addWidget(row.spin, rowIndex, 1)
                self._grid.addWidget(row.slider, rowIndex, 2)
            else:
                editor = row.combo or row.check or row.edit
                self._grid.addWidget(editor, rowIndex, 1, 1, 2)
            self._grid.addWidget(row.unit, rowIndex, 3)
            self._grid.addWidget(row.badge, rowIndex, 4)
            self._grid.addWidget(row.resetButton, rowIndex, 5)
            rowIndex += 1
        self._grid.setRowStretch(rowIndex, 1)
        self.RefreshValues()

    def RefreshValues(self):
        if self._prim is not None and not self._prim.IsValid():
            self.Rebuild()
            return
        for row in self._rows:
            row.Refresh(self._frame)
        self._SetStatus()

    def _SetStatus(self):
        bits = []
        if self._prim is not None:
            frame = ("default time" if self._frame.IsDefault()
                     else "frame %g" % self._frame.GetValue())
            bits.append("%d channels at %s, writing %s"
                        % (len(self._rows), frame,
                           "keys" if self._mode == model.WRITE_ANIMATION
                           else "defaults"))
        if self._warning:
            bits.append("WARNING: %s" % self._warning)
        self._status.setText("  ".join(bits))

    # -- editing (the rows call these) ---------------------------------

    def EditScope(self, channels, label):
        return model.EditScope(self.Stage(), channels, self._undo, label)

    def WriteChannel(self, channel, value):
        self._writing = True
        try:
            warning = model.WriteValue(channel, value, self._frame,
                                       self._mode)
        finally:
            self._writing = False
        self._Redraw()
        return warning

    def ResetChannel(self, channel):
        self._writing = True
        try:
            warning = model.ResetValue(channel, self._frame, self._mode)
        finally:
            self._writing = False
        self._Redraw()
        return warning

    def _Redraw(self):
        update = getattr(self._api, "UpdateViewport", None)
        if update is not None:
            try:
                update()
            except Exception:
                pass

    def ReportWarning(self, warning):
        self._warning = warning or ""
        self._SetStatus()

    def SetValue(self, name, value):
        """Edit one channel by name through its row's own path."""
        row = self.Row(name)
        if row is None:
            raise KeyError("no channel %r on %s" % (name, self._prim))
        row.SetValue(value)

    def Reset(self, name):
        row = self.Row(name)
        if row is None:
            raise KeyError("no channel %r on %s" % (name, self._prim))
        row.Reset()

    def ResetAll(self):
        """Every channel back to rest, as ONE undo entry."""
        if not self._rows:
            return
        warnings = []
        channels = [row.channel for row in self._rows]
        # No Sdf.ChangeBlock here: ResetValue READS the composed channel
        # (to promote a weaker layer's keys) and a read inside a block
        # holding earlier writes is the hazard gizmoMath documents. The
        # EditScope already makes the whole reset one undo entry.
        with self.EditScope(channels, "Reset %s" % self._prim.GetName()):
            self._writing = True
            try:
                for channel in channels:
                    warning = model.ResetValue(channel, self._frame,
                                               self._mode)
                    if warning:
                        warnings.append(warning)
            finally:
                self._writing = False
        self._Redraw()
        self.ReportWarning("; ".join(warnings[:3]) if warnings else None)
        self.RefreshValues()


def OpenAvarEditorPanel(usdviewApi, undoStack=None):
    panel = AvarEditorPanel.GetInstance(usdviewApi, undoStack)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
