#
# RigExec usdview plugin: the Avar Editor's widgets.
#
# Two of them, both plain Qt and both ignorant of avars, channels and
# USD, so the panel can be restyled without touching what an edit MEANS
# (that is avarEditorModel's job) and so these can be reused by any other
# panel that wants the same shape.
#
#   ValueField  one channel's number, typed straight in. No slider, no
#               spin arrows -- a channel box, where the value IS the
#               widget and you click it and type.
#   Section     a collapsible group with a disclosure triangle, so a
#               control's Translate / Rotate / Scale / Custom groups fold
#               away and a long rig stays readable in a narrow dock.
#
# THE LOOK IS A VIEWPORT SIDEBAR, THE CODE IS OURS. What is borrowed is the
# shape of the thing -- flat rows with the value inside the field, tight
# vertical rhythm, sections that fold, a panel that lives in the viewport
# rather than in a window of its own. No third-party UI code is used,
# copied or paraphrased: that code is GPL and this repository is not, so the
# behaviour was written from what the UI does, not from how it does it.
#
# WHY NO SLIDER. The previous row was a label, a spin box, a slider, a
# unit, a badge and a reset button: six widgets and about 420 px of width
# for one number. A slider is the wrong instrument for a rig channel
# anyway -- an animator types 30, or -12.5, and a drag that lands on
# 29.8 is a wrong answer that looks right.
#
import os
import sys

from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets


# The vertical rhythm. A row is deliberately tighter than Qt's default:
# a control with ten channels plus a Custom group has to fit a dock
# without scrolling, and Qt's stock QLineEdit height (~24 px here) puts
# that over the edge on a 1080-tall screen with the timeline showing.
ROW_HEIGHT = 20
ROW_SPACING = 2
LABEL_WIDTH = 74

# How wide the disclosure triangle's column is, so every section header
# and every row beneath it share one left margin.
INDENT = 14


def _EventPoint(event):
    """A mouse event's widget-local position as an (x, y) tuple.

    Qt6 renamed `pos()` to `position()` and made it a QPointF. The same
    two-line shim viewCubeUI carries, repeated rather than imported: this
    module is meant to be usable without the view cube on the path.
    """
    try:
        point = event.position()
    except AttributeError:                            # PySide2
        point = event.pos()
    return (point.x(), point.y())


# Scrubbing. A horizontal drag across the field changes the value; the
# modifiers change how much per pixel, which is the only way to get both
# "nudge it a hair" and "swing it fifty" out of one widget.
#
#   plain        1.0 x the channel's own step
#   Ctrl         0.1 x  -- slow, for the last decimal
#   Ctrl+Shift  10.0 x  -- fast, for crossing a range
#
# Shift alone is left alone: Qt gives it to text selection inside a line
# edit, and taking it would break click-and-drag-to-select.
DRAG_SLOW = 0.1
DRAG_FAST = 10.0
# Pixels of travel before a press counts as a scrub rather than a click.
# Below this the press still places the caret and selects, so a field is
# a text box first and a dial second.
DRAG_SLOP = 3


class ValueField(QtWidgets.QLineEdit):
    """One channel's value, typed directly. The channel box rule.

    Commits on Enter and on focus-out, reverts on Escape, and selects the
    whole text when it takes focus so typing replaces rather than
    appends -- which is what every channel box does and what makes a
    column of them fast to fly down with Tab.

    It is a QLineEdit and not a spin box on purpose. A spin box owns its
    own parsing, its own clamping and two arrows nobody clicks, and it
    fights a channel whose sensible range depends on the stage's units.
    Here the panel hands the field a formatter and a parser and keeps
    both rules in the model where they are tested.

    `committed(str)` carries the raw text; the panel parses it, because
    what counts as a legal value for a channel is the model's business
    and a bool, a token and a double all arrive through this one widget.
    """

    committed = QtCore.Signal(str)
    # A scrub: (value, isFirstSample). The panel brackets the whole drag
    # into one undo entry on the first sample and closes it on release.
    scrubbed = QtCore.Signal(float, bool)
    scrubFinished = QtCore.Signal()

    def __init__(self, parent=None):
        super(ValueField, self).__init__(parent)
        self.setFixedHeight(ROW_HEIGHT)
        self.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.setFrame(False)
        self._committedText = ""
        self._animated = False
        # Scrubbing state. `_step` is what one pixel is worth before the
        # modifiers scale it; the panel sets it from the channel, because
        # a rotation in degrees and a 0..1 dial cannot share a number.
        self._step = 0.1
        self._scrubOrigin = None
        self._scrubStart = 0.0
        self._scrubbing = False
        self._dragCursor = False
        self._scrubFirst = True
        self.setMouseTracking(True)
        self.editingFinished.connect(self._OnEditingFinished)
        self._ApplyStyle()

    def SetScrubStep(self, step):
        """What one pixel of drag is worth, before the modifiers."""
        try:
            step = float(step)
        except (TypeError, ValueError):
            return
        if step > 0.0:
            self._step = step

    def IsScrubbing(self):
        return self._scrubbing

    # -- state -----------------------------------------------------------

    def SetText(self, text):
        """Show `text` without emitting anything.

        Used by the refresh loop, which runs on every frame change and
        every rig re-evaluation: a setter that emitted would author the
        value it had just read back onto the stage.
        """
        self._committedText = text
        if self.text() != text:
            blocked = self.blockSignals(True)
            self.setText(text)
            self.blockSignals(blocked)

    def SetAnimated(self, animated):
        """Tint the field when the channel carries keys.

        The old row said this in a separate badge column. Folding it into
        the field itself is what buys the width back, and it is also how
        a channel box says it -- the value's own colour is the state.
        """
        animated = bool(animated)
        if animated != self._animated:
            self._animated = animated
            self._ApplyStyle()

    def _ApplyStyle(self):
        # Colours rather than a stylesheet on each instance would be
        # cheaper still, but a stylesheet is the only way to reach the
        # focus and hover states without subclassing the style.
        base = "#c8cdd6" if not self._animated else "#e2b857"
        self.setStyleSheet(
            "QLineEdit {"
            " background: rgba(255,255,255,18);"
            " border: 1px solid rgba(255,255,255,20);"
            " border-radius: 3px;"
            " padding: 0px 5px 0px 5px;"
            " color: %s; }"
            "QLineEdit:hover { background: rgba(255,255,255,30); }"
            "QLineEdit:focus {"
            " background: rgba(0,0,0,90);"
            " border: 1px solid #5680c2; }"
            "QLineEdit:read-only { background: transparent;"
            " border: 1px solid transparent; }" % base)

    # -- editing ---------------------------------------------------------

    # -- scrubbing ---------------------------------------------------

    def _ScrubScale(self, modifiers):
        ctrl = bool(modifiers & QtCore.Qt.ControlModifier)
        shift = bool(modifiers & QtCore.Qt.ShiftModifier)
        if ctrl and shift:
            return DRAG_FAST
        if ctrl:
            return DRAG_SLOW
        return 1.0

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and not self.isReadOnly():
            # Armed, not started: the press still reaches QLineEdit so a
            # plain click places the caret. The scrub only takes over once
            # the cursor has actually travelled.
            self._scrubOrigin = _EventPoint(event)
            try:
                self._scrubStart = float(self.text() or 0.0)
            except (TypeError, ValueError):
                self._scrubOrigin = None
        super(ValueField, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):
        # A hint that the field is draggable, before anybody presses.
        if not (event.buttons() & QtCore.Qt.LeftButton):
            if not self._dragCursor and not self.isReadOnly():
                self.setCursor(QtCore.Qt.SizeHorCursor)
                self._dragCursor = True
            super(ValueField, self).mouseMoveEvent(event)
            return
        if self._scrubOrigin is None:
            super(ValueField, self).mouseMoveEvent(event)
            return
        x, _y = _EventPoint(event)
        dx = x - self._scrubOrigin[0]
        if not self._scrubbing:
            if abs(dx) < DRAG_SLOP:
                super(ValueField, self).mouseMoveEvent(event)
                return
            self._scrubbing = True
            self._scrubFirst = True
            # Drop the selection the press made: a scrub is not a text
            # drag, and leaving it highlighted reads as though it were.
            self.deselect()
        value = self._scrubStart + dx * self._step * self._ScrubScale(
            event.modifiers())
        first = self._scrubbing and self._scrubFirst
        self._scrubFirst = False
        self.scrubbed.emit(value, first)
        event.accept()

    def mouseReleaseEvent(self, event):
        if self._scrubbing:
            self._scrubbing = False
            self._scrubOrigin = None
            self.scrubFinished.emit()
            event.accept()
            return
        self._scrubOrigin = None
        super(ValueField, self).mouseReleaseEvent(event)

    def leaveEvent(self, event):
        if self._dragCursor:
            self.unsetCursor()
            self._dragCursor = False
        super(ValueField, self).leaveEvent(event)

    def focusInEvent(self, event):
        super(ValueField, self).focusInEvent(event)
        # Queued: selecting inside focusIn is undone by the click that
        # caused it, which positions the caret afterwards.
        QtCore.QTimer.singleShot(0, self.selectAll)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            # Put the committed value back and hand focus away, so Escape
            # in a field cannot leave a half-typed number looking live.
            self.SetText(self._committedText)
            self.clearFocus()
            event.accept()
            return
        super(ValueField, self).keyPressEvent(event)

    def _OnEditingFinished(self):
        text = self.text()
        if text == self._committedText:
            return
        self._committedText = text
        self.committed.emit(text)


class Section(QtWidgets.QWidget):
    """A collapsible group: a header you click, and a body that folds.

    Folded state is kept on the widget and readable, so the panel can
    remember which groups were open across a rebuild -- a selection
    change must not re-open the four groups somebody just closed.

    The triangle is drawn as text rather than as an icon: this plugin
    ships no icons of its own for it, the two glyphs are in every font
    usdview runs with, and a QStyle arrow would take the palette of
    whatever theme is loaded rather than this panel's.
    """

    toggled = QtCore.Signal(bool)

    def __init__(self, title, parent=None):
        super(Section, self).__init__(parent)
        self._title = title
        self._expanded = True

        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self._header = QtWidgets.QToolButton()
        self._header.setText(self._HeaderText())
        self._header.setCheckable(True)
        self._header.setChecked(True)
        self._header.setCursor(QtCore.Qt.PointingHandCursor)
        self._header.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        self._header.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Fixed)
        self._header.setFixedHeight(ROW_HEIGHT)
        self._header.setStyleSheet(
            "QToolButton {"
            " background: rgba(255,255,255,14);"
            " border: none; border-radius: 3px;"
            " color: #d8dce4; font-weight: bold;"
            " padding-left: 4px; text-align: left; }"
            "QToolButton:hover { background: rgba(255,255,255,26); }")
        self._header.clicked.connect(self._OnClicked)
        outer.addWidget(self._header)

        self._body = QtWidgets.QWidget()
        self._grid = QtWidgets.QGridLayout(self._body)
        self._grid.setContentsMargins(INDENT, 2, 0, 4)
        self._grid.setHorizontalSpacing(4)
        self._grid.setVerticalSpacing(ROW_SPACING)
        self._grid.setColumnStretch(1, 1)
        outer.addWidget(self._body)

    # -- the body the panel fills ----------------------------------------

    def Grid(self):
        """The layout rows are added to."""
        return self._grid

    def Body(self):
        return self._body

    # -- folding ----------------------------------------------------------

    def _HeaderText(self):
        return "%s  %s" % ("▾" if self._expanded else "▸",
                           self._title)

    def IsExpanded(self):
        return self._expanded

    def SetExpanded(self, expanded):
        expanded = bool(expanded)
        if expanded == self._expanded:
            return
        self._expanded = expanded
        self._header.setChecked(expanded)
        self._header.setText(self._HeaderText())
        self._body.setVisible(expanded)
        self.toggled.emit(expanded)

    def _OnClicked(self):
        self.SetExpanded(not self._expanded)


class RowLabel(QtWidgets.QLabel):
    """A channel's name, sized so every row's field starts at one x.

    Elided rather than wrapped or allowed to widen: a custom avar with a
    long name must not push the value column out of line, and the full
    name is on the tooltip anyway.
    """

    def __init__(self, text, parent=None):
        super(RowLabel, self).__init__(parent)
        self._full = text
        self.setFixedWidth(LABEL_WIDTH)
        self.setFixedHeight(ROW_HEIGHT)
        self.setStyleSheet("QLabel { color: #a8aeb8; }")
        self._Elide()

    def _Elide(self):
        metrics = QtGui.QFontMetrics(self.font())
        self.setText(metrics.elidedText(self._full, QtCore.Qt.ElideRight,
                                        self.width()))

    def resizeEvent(self, event):
        super(RowLabel, self).resizeEvent(event)
        self._Elide()
