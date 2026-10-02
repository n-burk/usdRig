























import os
import sys

from pxr import Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import avarEditorModel as model
    import avarWidgets
    import sessionRegistry
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import avarEditorModel as model
    import avarWidgets
    import sessionRegistry


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
        self._dragging = False
        self._dragValue = None
        self._previewing = False
        # True while the field shows a manipulator's uncommitted value
        # rather than the stage's; the panel refreshes exactly these rows
        # when the drag ends (ShowPending / EndPending).
        self._pendingShown = False

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
        self.field = None
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
        """One typed field. No slider, no spin arrows -- a channel box.

        An animator types 30, or -12.5. A drag that lands on 29.8 is a
        wrong answer that looks right, and the slider it came from cost
        140 px of every row. `SliderRange` survives on the model because
        it is still the channel's sensible span, and the field uses it to
        decide how many decimals to show -- it just does not draw it.
        """
        channel = self.channel
        self.field = avarWidgets.ValueField()
        self.field.setToolTip(
            "%s  (%s)\nType a value, or drag across the field to scrub "
            "it.\nCtrl+drag is ten times finer, Ctrl+Shift+drag ten times "
            "coarser." % (channel.attr.GetPath(), channel.attr.GetTypeName()))
        self.field.committed.connect(self._OnFieldCommitted)
        self.field.scrubbed.connect(self._OnScrubbed)
        self.field.scrubFinished.connect(self._OnScrubFinished)
        # What one pixel of drag is worth. The channel's own spin step,
        # which the model already derives per kind -- a rotation moves in
        # degrees and a 0..1 dial in hundredths, and a single constant
        # here would make one of them useless.
        self.field.SetScrubStep(channel.SpinStep())
        self._sliderLow, self._sliderHigh = channel.SliderRange(
            self.panel.Stage(), None)

    def _FormatValue(self, value):
        if value is None:
            return ""
        if self.channel.family == model.VALUE_INT:
            return str(int(value))
        # Trailing zeros dropped: a column of "0.000" reads as noise, and
        # the channel's own decimals are the ceiling, not the floor.
        text = "%.*f" % (self.channel.Decimals(), float(value))
        if "." in text:
            text = text.rstrip("0").rstrip(".")
        return text or "0"

    def _OnScrubbed(self, value, first):
        """One sample of a drag across the field.

        The whole drag is ONE undo entry: the first sample opens the
        scope, every later one writes inside it, and the release closes
        it. That is the slider's old contract, kept -- a drag must not
        leave a hundred entries on the stack.

        The preview channel is used where the gizmo has one, so a drag
        goes to Hydra rather than to the stage and the rig is not
        recompiled per mouse sample.
        """
        if self._updating:
            return
        if self.channel.family == model.VALUE_INT:
            value = int(round(value))
        self._dragging = True
        self._dragValue = value
        if self._Preview(value):
            self._previewing = True
            self._ShowValue(value)
            return
        if first and self._scope is None:
            self._scope = self.panel.EditScope(
                [self.channel], "Drag %s" % self.channel.shortName)
            self._scope.__enter__()
        # Show it WHILE dragging, not on release. The preview path above
        # already does; this one wrote the value and left the field
        # reading whatever it said before the drag began, so a scrub with
        # no gizmo preview -- which is most channels, and every one of the
        # eye's shape dials -- gave no number until the mouse came up.
        self._ShowValue(value)
        self._Write(value, bracket=False)

    def _OnScrubFinished(self):
        """Release: commit once, close the scope, drop the preview."""
        self._OnSliderReleased()

    def _OnFieldCommitted(self, text):
        if self._updating:
            return
        text = (text or "").strip()
        if not text:
            self.Refresh(self.panel.Frame())
            return
        try:
            value = (int(round(float(text)))
                     if self.channel.family == model.VALUE_INT
                     else float(text))
        except (TypeError, ValueError):
            # Not a number: put the committed value back rather than
            # authoring something the channel cannot hold.
            self.panel.ReportWarning("%s: %r is not a number"
                                     % (self.channel.shortName, text))
            self.Refresh(self.panel.Frame())
            return
        self._Write(value)

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
        # Value() prefers a manipulator's uncommitted value, so a stage
        # notice or a frame change landing mid-drag keeps showing the
        # number being dragged rather than flashing the authored one.
        value = channel.Value(time)
        self._pendingShown = channel.PendingValue() is not None
        self._updating = True
        try:
            if self.field is not None:
                if value is not None:
                    span = channel.SliderRange(self.panel.Stage(), value)
                    if span != (self._sliderLow, self._sliderHigh):
                        self._sliderLow, self._sliderHigh = span
                self.field.SetText(self._FormatValue(value))
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
        if self.field is not None:
            self.field.SetAnimated(animated)

    # -- writing ---------------------------------------------------------

    def _Write(self, value, bracket=True, verb="Set"):
        """
        The one path every widget goes through. `bracket` False means a
        slider drag already opened the undo scope; `verb` names the undo
        entry ("Drag" for a slider drag committed on release).
        """
        if self._updating:
            return
        if bracket:
            with self.panel.EditScope([self.channel],
                                      "%s %s" % (verb,
                                                 self.channel.shortName)):
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

    # -- slider drags: Hydra while dragging, the stage once on release -----
    #
    # Authoring to the stage is the expensive half of an edit: a layer spec
    # write, a change notice every observer processes, and a rig that has to
    # hear about it. None of that belongs in a drag. While the slider is held
    # the value goes to the viewport through the same preview channel the
    # viewport manipulators use (gizmoPreview: uncommitted values placed on
    # the evaluator and published to Hydra, nothing authored), and the stage
    # is written exactly once, when the slider is let go, as one undo entry.
    # A session with no preview channel (headless, or RigExec inactive) falls
    # back to authoring each value, which is what it always did.

    def _OnSliderPressed(self):
        if self._updating or self._dragging:
            return
        self._dragging = True
        self._dragValue = None
        self._previewing = False

    def _OnSlider(self, position):
        if self._updating:
            return
        value = self._SliderToValue(position)
        if not self._dragging:
            # A keyboard or wheel change on an undragged slider is its own
            # edit.
            self._Write(value, bracket=self._scope is None)
            return
        self._dragValue = value
        if self._Preview(value):
            self._previewing = True
            self._ShowValue(value)
            return
        # No preview channel: author the value as before, inside one scope
        # for the whole drag.
        if self._scope is None:
            self._scope = self.panel.EditScope(
                [self.channel], "Drag %s" % self.channel.shortName)
            self._scope.__enter__()
        self._Write(value, bracket=False)

    def _OnSliderReleased(self):
        dragging, self._dragging = self._dragging, False
        scope = self._scope
        self._scope = None
        if scope is not None:
            scope.__exit__(None, None, None)
            return
        if not dragging or not self._previewing:
            return
        self._previewing = False
        value = self._dragValue
        # Commit, THEN drop the preview, as the manipulators do: the other
        # order shows the pre-drag pose for a frame before the authored one.
        if value is not None:
            self._Write(value, verb="Drag")
        _EndPreview(self._Api(), self._PreviewStage())

    def _Api(self):
        """The usdview api of the session this row's panel belongs to."""
        return getattr(self.panel, "_api", None)

    def _PreviewStage(self):
        attr = self.channel.attr
        try:
            return attr.GetStage() if attr else None
        except Exception:
            return None

    def _Preview(self, value):
        """Place `value` on the viewport without authoring. True if taken.

        EVERY selected prim, not just the focus one. `WriteChannel`
        already writes the channel AND `Peers(channel)` -- the same avar
        on every other selected prim -- so a preview that pushed one path
        made the drag look like it drove a single control and then made
        the rest jump to the value on release. Same peer list, resolved
        the same way, so the two cannot disagree about who is being
        edited.
        """
        preview = _PreviewModule()
        api = self._Api()
        if preview is None:
            return False
        attr = self.channel.attr
        if not attr:
            return False
        path = attr.GetPath()
        pushes = {path: value}
        for peer in self.panel.Peers(self.channel):
            peerAttr = getattr(peer, "attr", None)
            if peerAttr:
                pushes[peerAttr.GetPath()] = value
        gizmoMath = _GizmoMathModule()
        previewStage = self._PreviewStage()
        if gizmoMath is not None:
            # The manipulators read uncommitted values from here, so a
            # control's gizmo follows the slider too -- and every peer's
            # gizmo with it.
            #
            # Into THIS STAGE's bucket, which is the one the release
            # clears (gizmoPreview.End -> SetPreviewValues({}, stage=...)).
            # Written without a stage it landed in the keyless bucket
            # instead, so the release popped an empty one and the dragged
            # number stayed in the map: Channel.Value prefers a pending
            # value, so every later refresh of every row on that channel
            # went on showing it.
            values = dict(gizmoMath.PreviewValues(previewStage))
            values.update(pushes)
            gizmoMath.SetPreviewValues(values, stage=previewStage)
        # The SINK may refuse -- headless, no rigExecImaging, or a sink
        # that could not begin -- and the viewport follow is the only part
        # that depends on it. The uncommitted values above are placed
        # whatever it answers, because the manipulators and this panel both
        # read them; so previewing is what was PLACED, not what Hydra took.
        # Reporting the sink's answer instead left the values in the map
        # with nobody to clear them: the release took the no-preview branch,
        # never called _EndPreview, and every later refresh went on showing
        # the dragged number -- a typed value landed on the stage and the
        # field snapped back to the drag.
        taken = bool(preview.Push(pushes))
        if taken:
            _FollowPreviewInViewport(api, path.GetPrimPath())
        return gizmoMath is not None or taken

    def _ShowValue(self, value):
        """Show `value` in the field without writing it.

        Only numeric rows have one; a token, bool or string row is never
        dragged. SetText is a no-op when the text is already right, so
        calling this per mouse sample costs nothing for a field whose
        number did not change at this many decimals.
        """
        if self.field is None:
            return
        self._updating = True
        try:
            self.field.SetText(self._FormatValue(value))
        finally:
            self._updating = False

    # -- a manipulator's drag: shown here, authored nowhere yet ----------
    #
    # The viewport gizmo collects each sample in gizmoMath's Writer and
    # authors once on release (gizmoUI._UpdateDrag / _EndDrag). The panel
    # hears every sample through gizmoPreview's listeners and hands each
    # row its own value here; the row does not re-evaluate anything, it
    # formats one number.

    def ShowPending(self, value):
        """Show a manipulator's uncommitted `value`. True if this row can."""
        if self.field is None:
            return False
        try:
            self._ShowValue(value)
        except (TypeError, ValueError):
            # Not a scalar (an xformOp vector landed on an avar path
            # somehow): leave the field saying what the stage says.
            return False
        self._pendingShown = True
        return True

    def EndPending(self):
        """The drag is over: was this row showing its value?"""
        shown, self._pendingShown = self._pendingShown, False
        return shown

    def Rebind(self, channel):
        """
        Point this row at `channel`, a channel of the SAME layout (name,
        value family, kind, allowed tokens) on another prim, keeping every
        widget. The panel does this on a selection change when the new
        prim's channels line up with the old ones, which is nearly every
        switch between controls: rebuilding the widgets instead cost about
        100 ms per selection on the biped.
        """
        self.AbortDrag()
        if self._scope is not None:
            scope, self._scope = self._scope, None
            scope.__exit__(None, None, None)
        self.channel = channel
        self.name = channel.name
        tip = "%s  (%s)" % (channel.attr.GetPath(), channel.attr.GetTypeName())
        if channel.custom:
            tip += "\ncustom avar"
        self.label.setToolTip(tip)
        self.unit.setText(channel.unit)
        if self.spin is not None:
            self._sliderLow, self._sliderHigh = channel.SliderRange(
                self.panel.Stage(), None)

    def AbortDrag(self):
        """Drop an uncommitted slider drag without authoring it."""
        if self._previewing:
            self._previewing = False
            _EndPreview(self._Api(), self._PreviewStage())
        self._dragging = False
        self._dragValue = None

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


def _PreviewModule():
    try:
        import gizmoPreview
        return gizmoPreview
    except ImportError:
        return None


def _GizmoMathModule():
    try:
        import gizmoMath
        return gizmoMath
    except ImportError:
        return None


def _FollowPreviewInViewport(api, primPath=None):
    """
    Redraw `api`'s session's viewport manipulators against the values being
    previewed.

    The gizmo refreshes itself on stage edits, and a preview is not one: it
    places values on the evaluator and Hydra without touching the stage, so
    without this the handles would sit at the pre-drag pose until release.
    """
    try:
        import gizmoUI
    except ImportError:
        gizmoUI = None
    controller = (gizmoUI.GetController(api)
                  if gizmoUI is not None and api is not None else None)
    if controller is not None:
        controller.FollowExternalPreview(primPath)
        return
    # No viewport tools at all: the preview still moved the rig, so ask the
    # viewport to repaint.
    update = getattr(api, "UpdateViewport", None)
    if update is not None:
        try:
            update()
        except Exception:
            pass


def _EndPreview(api, stage=None):
    preview = _PreviewModule()
    if preview is not None:
        preview.End(session=api, stage=stage)
    _FollowPreviewInViewport(api)


class AvarEditorPanel(QtWidgets.QDialog,
                      metaclass=sessionRegistry.PerSessionInstanceMeta(
                          QtWidgets.QDialog)):
    """The Avar Editor window. One per usdview session.

    Filed under the session's main window: several usdview sessions can
    share this module in one process. `AvarEditorPanel._instance` reads
    the current session's panel (see sessionRegistry.PerSessionInstanceMeta).
    """

    _sessions = sessionRegistry.SessionRegistry("avar editors")

    @classmethod
    def GetInstance(cls, usdviewApi, undoStack):
        panel = cls._sessions.Get(usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(usdviewApi, cls(usdviewApi, undoStack))
        else:
            panel._api = usdviewApi
            if undoStack is not None:
                panel._undo = undoStack
        return panel

    def __init__(self, usdviewApi, undoStack, parent=None):
        super(AvarEditorPanel, self).__init__(
            parent or getattr(usdviewApi, "qMainWindow", None))
        self._api = usdviewApi
        self._undo = undoStack
        self._rows = []
        # {Sdf.Path of the channel attribute: ChannelRow}, so a drag
        # sample -- a handful of paths, per mouse move -- finds its rows
        # without walking them.
        self._rowsByPath = {}
        self._prim = None
        self._others = 0
        self._writing = False
        self._noticeKey = None
        self._previewListener = None
        self._frame = _FrameOf(getattr(usdviewApi, "frame",
                                       Usd.TimeCode.Default()))
        self._mode = model.WRITE_ANIMATION
        self._warning = ""
        # Which kind groups are folded, by kind token. Survives a
        # rebuild so a selection change does not re-open what the user
        # closed; it is a view preference, not rig data, so it is never
        # written to the stage.
        self._folded = {}
        self._sections = {}
        self._headings = []

        self.setWindowTitle("Avar Editor")
        # Tall enough that a control's Custom group (the biped's ikfk
        # dial) is in view under the ten schema channels without a scroll.
        self.resize(600, 660)

        layout = QtWidgets.QVBoxLayout(self)

        self._header = QtWidgets.QLabel("")
        # Never let the header widen the window; long text is elided by
        # the name-only form above, and the path lives in the tooltip.
        self._header.setMinimumWidth(1)
        self._header.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                                   QtWidgets.QSizePolicy.Preferred)
        self._header.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse)
        font = self._header.font()
        font.setBold(True)
        self._header.setFont(font)
        layout.addWidget(self._header)

        self._note = QtWidgets.QLabel("")
        self._note.setWordWrap(True)
        layout.addWidget(self._note)

        self._search = QtWidgets.QLineEdit()
        self._search.setPlaceholderText("find a channel...")
        self._search.setClearButtonEnabled(True)
        self._search.setToolTip(
            "Filter the channels below. Matches the channel's short name "
            "and its full attribute name, case-insensitively; "
            "space-separated words all have to match, in any order, so "
            "\"r x\" finds rx and nothing else.")
        self._search.textChanged.connect(self._OnSearchChanged)
        layout.addWidget(self._search)

        # Two rows, because docked the panel is 296 px and one row of
        # "Write: [Animation] Reset All Refresh Undock" clips every label
        # to an unreadable stub. _ApplyCompactLayout folds the buttons
        # onto the second row when there is no space for them beside the
        # mode box, and back up when the panel is floating and wide.
        controlRows = QtWidgets.QVBoxLayout()
        controlRows.setContentsMargins(0, 0, 0, 0)
        controlRows.setSpacing(4)
        controls = QtWidgets.QHBoxLayout()
        self._writeLabel = QtWidgets.QLabel("Write:")
        controls.addWidget(self._writeLabel)
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
        self._dockButton = QtWidgets.QPushButton("Dock")
        self._dockButton.setToolTip(
            "Pin the editor to the right of the viewport, where it folds "
            "away to a strip. The same editor either way -- it is moved, "
            "not copied, so nothing is lost and nothing refreshes twice.")
        self._dockButton.clicked.connect(self.ToggleDock)
        controls.addWidget(self._dockButton)
        self._controlsRow = controls
        self._buttonRow = QtWidgets.QHBoxLayout()
        self._buttonRow.setContentsMargins(0, 0, 0, 0)
        self._refreshButton = refresh
        controlRows.addLayout(controls)
        controlRows.addLayout(self._buttonRow)
        layout.addLayout(controlRows)

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
        self._ListenToPreview()
        self.Rebuild()

    # -- wiring ------------------------------------------------------

    def _ListenToPreview(self):
        """Hear every sample of a manipulator drag, and its end.

        The gizmo authors nothing until release (gizmoMath.Writer), so no
        stage notice arrives while it is dragged and the fields would sit
        on the pre-drag numbers. gizmoPreview.Push is the one call every
        sample makes on its way to Hydra, and its listener list is the
        smallest hook that reaches it without the panel knowing the
        controller. Kept as an attribute so RemoveListener can find the
        same bound method again.
        """
        preview = _PreviewModule()
        if preview is None or self._previewListener is not None:
            return
        self._previewListener = self._OnPreview
        preview.AddListener(self._previewListener)

    def _OnPreview(self, pending):
        """One drag sample ({Sdf.Path: value}), or None when it ends.

        Per sample: only the rows whose attribute is in `pending`, and
        each of those sets one line of text, which is a no-op when the
        number has not changed at the shown decimals. Nothing is
        evaluated and nothing is rebuilt; the rig was already evaluated
        by the drag itself. On the end: the rows that showed a pending
        value re-read the stage -- after a release that is the authored
        value the commit just made, after an abort the pre-drag one.
        """
        if pending is None:
            for row in self._rows:
                if row.EndPending():
                    row.Refresh(self._frame)
            return
        rows = self._rowsByPath
        if not rows:
            return
        for path, value in pending.items():
            row = rows.get(path)
            if row is not None:
                row.ShowPending(value)

    def _IndexRows(self):
        self._rowsByPath = {row.channel.attr.GetPath(): row
                            for row in self._rows if row.channel.attr}

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
        model.ForgetDrivenProperties()
        if stage:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._OnObjectsChanged, stage)

    def _OnObjectsChanged(self, notice, sender):
        # Our own writes refresh the row that wrote; everything else --
        # a gizmo drag, an undo, a script -- refreshes every row.
        if self._writing:
            return
        if notice.GetResyncedPaths():
            # Prims came or went, so a mover may have too, and which
            # channels the rig recomputes is no longer known.
            model.ForgetDrivenProperties(sender)
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
        if self._previewListener is not None:
            preview = _PreviewModule()
            if preview is not None:
                preview.RemoveListener(self._previewListener)
            self._previewListener = None
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
            row.AbortDrag()
            row.setParent(None)
        self._rows = []
        self._rowsByPath = {}
        while self._grid.count():
            item = self._grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()

    @staticmethod
    def _LayoutOf(channels):
        """What decides the row widgets: a rebind is only safe when equal."""
        return [(c.name, c.family, c.kind, tuple(c.allowedTokens or ()),
                 c.custom) for c in channels]

    def Rebuild(self, *args):
        stage = self.Stage()
        prim, others = (None, 0)
        if stage:
            prim, others = model.FocusPrim(self._api)
        if (prim is not None and self._rows
                and self._TryRebind(prim, others, stage)):
            return
        self._Clear()
        self._layout = None
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
        # The NAME, with the parent above it, and the full path as the
        # tooltip. The whole path of a deeply nested control (a brow under
        # the skull, the face and its pivots) is over 1200 px of unwrapped
        # text, and a label that long sets the window's minimum width.
        path = prim.GetPath()
        parent = path.GetParentPath()
        self._header.setText(
            "%s   (in %s)" % (path.name, parent.name)
            if parent and parent.name else path.name)
        self._header.setToolTip(str(path))
        channels, hidden = model.DiscoverChannels(prim, stage)
        notes = []
        if others:
            notes.append("%d more selected; shared channels edit together."
                         % others)
        if not channels:
            notes.append("%s carries no avars:* channels. Pick a "
                         "RigExecControl or RigExecJoint." % prim.GetName())
        elif hidden:
            notes.append("not shown: %s" % ", ".join(hidden))
        self._note.setText("  ".join(notes))
        self._resetAll.setEnabled(bool(channels))

        # One collapsible Section per kind, in KIND_ORDER. Folded state
        # is remembered by kind across a rebuild: a selection change must
        # not re-open the groups somebody has just closed.
        self._headings = []
        self._sections = {}
        kind = None
        section = None
        rowIndex = 0
        for channel in channels:
            if channel.kind != kind or section is None:
                kind = channel.kind
                section = avarWidgets.Section(model.KIND_LABELS[kind])
                section.SetExpanded(self._folded.get(kind, True))
                section.toggled.connect(
                    lambda expanded, k=kind: self._folded.__setitem__(
                        k, expanded))
                self._grid.addWidget(section, rowIndex, 0, 1, 2)
                self._sections[kind] = section
                self._headings.append((section, []))
                rowIndex += 1
            row = ChannelRow(self, channel)
            self._rows.append(row)
            self._headings[-1][1].append(row)
            grid = section.Grid()
            line = grid.rowCount()
            grid.addWidget(row.label, line, 0)
            editor = (row.field or row.combo or row.check or row.edit)
            grid.addWidget(editor, line, 1)
            grid.addWidget(row.unit, line, 2)
            grid.addWidget(row.resetButton, line, 3)
        self._grid.setRowStretch(rowIndex, 1)
        self._layout = self._LayoutOf(channels)
        self._IndexRows()
        self.RefreshValues()
        self._RefreshDockTabs()

    def _TryRebind(self, prim, others, stage):
        """
        Reuse the current rows for `prim` when its channels have the same
        layout. Returns False (and changes nothing) when they do not.
        """
        channels, hidden = model.DiscoverChannels(prim, stage)
        if not channels or self._LayoutOf(channels) != getattr(
                self, "_layout", None):
            return False
        self._prim = prim
        self._others = others
        self._warning = ""
        path = prim.GetPath()
        parent = path.GetParentPath()
        self._header.setText(
            "%s   (in %s)" % (path.name, parent.name)
            if parent and parent.name else path.name)
        self._header.setToolTip(str(path))
        notes = []
        if others:
            notes.append("%d more selected; editing the focus prim only."
                         % others)
        if hidden:
            notes.append("not shown: %s" % ", ".join(hidden))
        self._note.setText("  ".join(notes))
        self._resetAll.setEnabled(True)
        for row, channel in zip(self._rows, channels):
            row.Rebind(channel)
        self._IndexRows()
        self.RefreshValues()
        return True

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
            if self._others:
                # What an edit will actually reach, so nobody discovers
                # the multi-prim write by undoing it.
                bits.append(
                    "editing %d prims together (channels they share)"
                    % (self._others + 1))
        if self._warning:
            bits.append("WARNING: %s" % self._warning)
        self._status.setText("  ".join(bits))

    # -- editing (the rows call these) ---------------------------------

    def _RowWidgets(self, row):
        """Every widget one channel row owns, for showing and hiding."""
        return [w for w in (row.label, row.field, row.combo, row.check,
                            row.edit, row.unit, row.resetButton)
                if w is not None]

    def _OnSearchChanged(self, _text):
        """Show only the channels matching the box.

        Widgets are HIDDEN rather than the grid rebuilt: a rebuild would
        drop the focus and the caret out of whatever the user is typing
        in, and a control's channel count is tens, not thousands. A
        heading whose whole group is hidden goes with it, so the filter
        does not leave empty section titles behind.
        """
        query = self._search.text()
        for row in self._rows:
            visible = model.Matches(row.channel.shortName, query) or                 model.Matches(row.channel.name, query)
            for widget in self._RowWidgets(row):
                widget.setVisible(visible)
        for section, rows in getattr(self, "_headings", []):
            # A group whose every row is filtered out goes with them,
            # rather than leaving an empty title behind.
            section.setVisible(any(
                any(w.isVisible() for w in self._RowWidgets(r))
                for r in rows) if rows else True)

    def Peers(self, channel):
        """The same channel on the other selected prims.

        Resolved per edit rather than cached with the rows: the selection
        can change between two edits without the panel rebuilding (the
        shown prim did not change), and a stale peer list would write to
        a prim nobody has selected any more. It is a handful of
        `GetAttribute` calls on a handful of prims.
        """
        if not self._others:
            return []
        prims = [p for p in (self._api.selectedPrims or [])
                 if p and p.IsValid() and not p.IsPseudoRoot()]
        return model.PeerChannels(channel, prims)

    def EditScope(self, channels, label):
        """One undo entry covering the edit AND the prims it reaches."""
        channels = list(channels)
        for channel in list(channels):
            channels.extend(self.Peers(channel))
        return model.EditScope(self.Stage(), channels, self._undo, label)

    def WriteChannel(self, channel, value):
        """Write one channel, and the same avar on every selected peer.

        All of it inside the caller's EditScope, so the whole multi-prim
        edit is ONE undo entry and the rig re-evaluates once. Writing the
        peers in their own scopes would cost a full re-evaluation each --
        the trap the picker's zero-pose records at ~190 ms per attribute
        on the biped.
        """
        self._writing = True
        try:
            warning = model.WriteValue(channel, value, self._frame,
                                       self._mode)
            for peer in self.Peers(channel):
                peerWarning = model.WriteValue(peer, value, self._frame,
                                               self._mode)
                warning = warning or peerWarning
        finally:
            self._writing = False
        self._Redraw()
        return warning

    def ResetChannel(self, channel):
        self._writing = True
        try:
            warning = model.ResetValue(channel, self._frame, self._mode)
            for peer in self.Peers(channel):
                peerWarning = model.ResetValue(peer, self._frame, self._mode)
                warning = warning or peerWarning
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

    def _RefreshDockTabs(self):
        """Tell the dock its category strip is stale.

        Which sections exist follows the selected control, so the strip
        has to be rebuilt whenever the channel list is.
        """
        try:
            controller = _Dock(self._api, self._undo)
        except Exception:
            return
        dock = getattr(controller, "_dock", None) if controller else None
        refresh = getattr(dock, "RefreshTabs", None) if dock else None
        if refresh is not None:
            try:
                refresh()
            except Exception:
                pass

    def SectionTitles(self):
        """The group names currently on screen, in display order.

        What the folded dock writes up its strip. Read from the live
        sections rather than from KIND_ORDER, so a control with no scale
        channels does not advertise a Scale tab that opens on nothing.
        """
        titles = []
        for kind in model.KIND_ORDER:
            section = self._sections.get(kind)
            if section is not None:
                titles.append(model.KIND_LABELS[kind])
        return titles

    def FocusSection(self, title):
        """Open the group called `title` and scroll it into view."""
        for kind, section in self._sections.items():
            if model.KIND_LABELS.get(kind) != title:
                continue
            if not section.IsExpanded():
                section.SetExpanded(True)
                self._folded[kind] = True
            self._scroll.ensureWidgetVisible(section)
            return True
        return False

    def IsDocked(self):
        controller = _Dock(self._api, self._undo)
        return bool(controller is not None and controller.IsDocked())

    def ToggleDock(self):
        """Move the editor between the viewport and its own window.

        MOVED, not copied: the dock reparents this very widget, so the
        selection it is showing, the groups that are folded and any
        half-typed value all survive the trip.
        """
        controller = _Dock(self._api, self._undo)
        if controller is None:
            self.ReportWarning("no viewport to dock into")
            return False
        if controller.IsDocked():
            controller.Undock()
            docked = False
        else:
            controller.Dock(self)
            docked = True
        # An explicit press is a preference: reopening must not drag the
        # panel back into the viewport somebody just pulled out of it.
        self.setProperty("rigExecAvarDockChosen", True)
        self.OnDocked(docked)
        return docked

    def OnDocked(self, docked):
        """Label and layout for the position the panel is now in."""
        self._dockButton.setText("Undock" if docked else "Dock")
        self._ApplyCompactLayout(docked)

    def _ApplyCompactLayout(self, compact):
        """Buttons on their own row when the panel is dock-narrow.

        Moved between two real layouts rather than shrunk: elided button
        text ("eset A", "efresh") is what the single row degrades to, and
        a button nobody can read is not a button.
        """
        buttons = (self._resetAll, self._refreshButton, self._dockButton)
        source = self._controlsRow if compact else self._buttonRow
        target = self._buttonRow if compact else self._controlsRow
        for button in buttons:
            source.removeWidget(button)
            target.addWidget(button)
        # "Write:" is what the mode box already says in its tooltip, and
        # the two words are a third of the narrow row.
        self._writeLabel.setVisible(not compact)
        for button in buttons:
            button.show()

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


def _Dock(usdviewApi, undoStack=None):
    """The viewport dock controller, installed on first use.

    Imported lazily and behind a guard: the dock needs usdview's stage
    view, and the editor has to keep opening in a session that has none
    (a headless test, or a usdview whose view has not been built yet).
    """
    try:
        import avarDock
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        try:
            import avarDock
        except ImportError:
            return None
    try:
        return avarDock.InstallAvarDock(usdviewApi, undoStack)
    except Exception:
        return None


def OpenAvarEditorPanel(usdviewApi, undoStack=None):
    """Open the editor, DOCKED into the viewport.

    Docked is the working position: an animator picks a control in the
    viewport and types a number, and a floating window over the thing
    being posed is in the way for both halves of that. A session that has
    already undocked keeps its choice -- IsDocked answers for the live
    panel, so this only places one that is not placed yet -- and the Dock
    button still moves it back out.
    """
    panel = AvarEditorPanel.GetInstance(usdviewApi, undoStack)
    panel.Rebuild()
    placed = False
    if not panel.IsDocked() and not panel.property("rigExecAvarDockChosen"):
        controller = _Dock(usdviewApi, undoStack)
        if controller is not None and controller.Dock(panel) is not None:
            panel.OnDocked(True)
            placed = True
    if not placed:
        panel.show()
        panel.raise_()
        panel.activateWindow()
    return panel
