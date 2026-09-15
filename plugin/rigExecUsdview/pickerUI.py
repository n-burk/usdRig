"""The control picker panel: the studio's picker, driving usdview.

Clicking a button calls `dataModel.selection.setPrim` (or `addPrim` with
Shift/Ctrl), which is the whole trick -- selecting through usdview's own
data model means the Avar Editor and the viewport gizmo follow for free,
so this panel never has to know what an avar is.

Drawing rules live in `pickerModel`, which has no Qt and is unit tested.
This file is the scene, the painting and the mouse.

Buttons whose control this rig does not have are drawn DIMMED rather than
hidden: on the biped that is most of the face panel and the bend controls,
and an animator is better served by seeing the rig is incomplete there than
by a button that silently does nothing.
"""
import os
import sys

from pxr import Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

if __name__ != "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pickerModel
import pickerScene


_DIM = 0.22


def _path_for(button):
    """A QPainterPath in the button's own local box."""
    path = QtGui.QPainterPath()
    w, h = button.w, button.h
    shape = button.shape

    if shape in ("circle", "ellipse"):
        path.addEllipse(QtCore.QRectF(0, 0, w, h))
    elif shape == "polygon" and button.polygon:
        _polygon(path, button.polygon, button.roundness)
    elif shape == "bezier" and button.bezier:
        points = button.bezier
        path.moveTo(*points[0][0])
        for i in range(len(points)):
            a = points[i]
            b = points[(i + 1) % len(points)]
            c1 = a[2] if len(a) > 2 else a[0]
            c2 = b[1] if len(b) > 1 else b[0]
            path.cubicTo(c1[0], c1[1], c2[0], c2[1], b[0][0], b[0][1])
        path.closeSubpath()
    elif shape == "trapezoid":
        ls, rs = button.left_slope, button.right_slope
        if button.direction in ("top", "bottom"):
            path.moveTo(ls, 0)
            path.lineTo(w - rs, 0)
            path.lineTo(w, h)
            path.lineTo(0, h)
        else:
            path.moveTo(0, ls)
            path.lineTo(w, 0)
            path.lineTo(w, h)
            path.lineTo(0, h - rs)
        path.closeSubpath()
    elif shape == "hexagon":
        q = w * 0.25
        path.moveTo(q, 0)
        path.lineTo(w - q, 0)
        path.lineTo(w, h * 0.5)
        path.lineTo(w - q, h)
        path.lineTo(q, h)
        path.lineTo(0, h * 0.5)
        path.closeSubpath()
    elif shape == "triangle":
        if button.direction == "right":
            path.moveTo(0, 0)
            path.lineTo(w, h * 0.5)
            path.lineTo(0, h)
        elif button.direction == "left":
            path.moveTo(w, 0)
            path.lineTo(0, h * 0.5)
            path.lineTo(w, h)
        else:
            path.moveTo(w * 0.5, 0)
            path.lineTo(w, h)
            path.lineTo(0, h)
        path.closeSubpath()
    else:
        r = max(button.roundness, 0.0)
        if shape == "roundedRectangle":
            r = max(r, min(w, h) * 0.35)
        if r > 0:
            path.addRoundedRect(QtCore.QRectF(0, 0, w, h), r, r)
        else:
            path.addRect(QtCore.QRectF(0, 0, w, h))
    return path


def _polygon(path, points, roundness):
    n = len(points)
    if roundness <= 0 or n < 3:
        path.moveTo(*points[0])
        for p in points[1:]:
            path.lineTo(*p)
        path.closeSubpath()
        return

    def trim(a, b, r):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = (dx * dx + dy * dy) ** 0.5 or 1.0
        t = min(r, length * 0.5) / length
        return (a[0] + dx * t, a[1] + dy * t)

    path.moveTo(*trim(points[0], points[1], roundness))
    for i in range(n):
        cur = points[(i + 1) % n]
        nxt = points[(i + 2) % n]
        path.lineTo(*trim(cur, points[i], roundness))
        after = trim(cur, nxt, roundness)
        path.quadTo(cur[0], cur[1], after[0], after[1])
    path.closeSubpath()


class PickerView(QtWidgets.QWidget):
    """One panel, drawn and clickable."""

    # (buttons, mode) where mode is "replace" | "toggle" | "remove".
    # One signal for a click and a marquee: a click is a marquee of one,
    # and having two paths was how the two ended up with different
    # modifier rules in every tool that has both.
    picked = QtCore.Signal(object, str)

    def __init__(self, picker, panel, parent=None):
        super(PickerView, self).__init__(parent)
        self._press = None
        self._mode = "replace"
        self._picker = picker
        self._panel = panel
        self._modes = {}
        self._edit = False
        self._band = None       # marquee in panel space, or None
        self._selected = set()
        self._hover = None
        self.setMouseTracking(True)
        self.setMinimumSize(int(panel.w), int(panel.h))
        self._font = _load_font()

    # -- painting --------------------------------------------------------

    def _scale(self):
        return min(self.width() / self._panel.w,
                   self.height() / self._panel.h) or 1.0

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor(42, 42, 42))
        scale = self._scale()
        painter.scale(scale, scale)
        painter.setBrush(QtGui.QBrush(QtGui.QColor(*self._panel.fill)))
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 200)))
        painter.drawRect(QtCore.QRectF(0, 0, self._panel.w, self._panel.h))

        for button in self._picker.visible(
                self._panel.id,
                modes=None if self._edit else self._modes,
                edit=self._edit):
            self._draw(painter, button)
        if self._band is not None:
            x0, y0, x1, y1 = self._band
            rect = QtCore.QRectF(min(x0, x1), min(y0, y1),
                                 abs(x1 - x0), abs(y1 - y0))
            painter.setBrush(QtGui.QBrush(QtGui.QColor(120, 200, 255, 40)))
            pen = QtGui.QPen(QtGui.QColor(150, 215, 255))
            pen.setWidthF(1.0)
            painter.setPen(pen)
            painter.drawRect(rect)
        painter.end()

    def _draw(self, painter, button):
        painter.save()
        painter.translate(button.x, button.y)
        if button.rotation:
            painter.translate(button.w * 0.5, button.h * 0.5)
            painter.rotate(button.rotation)
            painter.translate(-button.w * 0.5, -button.h * 0.5)

        fill = QtGui.QColor(*button.fill)
        if not button.decoration and not button.live:
            fill.setAlpha(int(fill.alpha() * _DIM))
        if button.id == self._hover and button.live:
            fill = fill.lighter(125)

        path = _path_for(button)
        painter.setBrush(QtGui.QBrush(fill))
        selected = bool(button.targets) and self._selected.issuperset(
            button.targets)
        if selected:
            pen = QtGui.QPen(QtGui.QColor(255, 255, 255))
            pen.setWidthF(2.0)
        else:
            pen = QtGui.QPen(QtGui.QColor(*button.stroke))
            pen.setWidthF(max(button.stroke_width, 0.5))
        painter.setPen(pen)
        painter.drawPath(path)

        if button.checkbox:
            side = min(button.h - 4.0, 11.0)
            top = (button.h - side) * 0.5
            left = 3.0 if button.h_align != "right" else button.w - side - 3.0
            painter.setBrush(QtGui.QBrush(QtGui.QColor(58, 58, 58)))
            painter.setPen(QtGui.QPen(QtGui.QColor(18, 18, 18)))
            painter.drawRect(QtCore.QRectF(left, top, side, side))
            if button.checked:
                tick = QtGui.QPen(QtGui.QColor(120, 200, 255))
                tick.setWidthF(1.8)
                painter.setPen(tick)
                painter.drawPolyline([
                    QtCore.QPointF(left + side * 0.20, top + side * 0.52),
                    QtCore.QPointF(left + side * 0.44, top + side * 0.76),
                    QtCore.QPointF(left + side * 0.82, top + side * 0.24)])

        if button.text:
            font = QtGui.QFont(self._font or "Sans")
            font.setPixelSize(max(int(round(button.font_size)), 5))
            font.setBold(button.bold)
            box = QtCore.QRectF(2, 0, button.w - 4, button.h)
            for _ in range(6):
                if (QtGui.QFontMetricsF(font).horizontalAdvance(button.text)
                        <= box.width() or font.pixelSize() <= 5):
                    break
                font.setPixelSize(font.pixelSize() - 1)
            painter.setFont(font)
            painter.setPen(QtGui.QColor(*button.text_color))
            align = {"left": QtCore.Qt.AlignLeft,
                     "right": QtCore.Qt.AlignRight}.get(
                         button.h_align, QtCore.Qt.AlignHCenter)
            if button.checkbox:
                box.setLeft(box.left() + min(button.h - 4.0, 11.0) + 5.0)
                align = QtCore.Qt.AlignLeft
            painter.drawText(box, align | QtCore.Qt.AlignVCenter, button.text)
            if button.value:
                painter.setPen(QtGui.QColor(*button.value_color))
                if align == QtCore.Qt.AlignRight:
                    value_box = QtCore.QRectF(box.left() + 12, box.top(),
                                              box.width() - 12, box.height())
                    value_align = QtCore.Qt.AlignLeft
                    caret_x = box.left() + 5
                else:
                    value_box = QtCore.QRectF(box.left(), box.top(),
                                              box.width() - 12, box.height())
                    value_align = QtCore.Qt.AlignRight
                    caret_x = box.right() - 7
                painter.drawText(value_box,
                                 value_align | QtCore.Qt.AlignVCenter,
                                 button.value)
                mid = box.center().y()
                caret = QtGui.QPainterPath()
                caret.moveTo(caret_x - 3.2, mid - 1.6)
                caret.lineTo(caret_x + 3.2, mid - 1.6)
                caret.lineTo(caret_x, mid + 2.2)
                caret.closeSubpath()
                painter.setBrush(QtGui.QBrush(
                    QtGui.QColor(*button.value_color)))
                painter.setPen(QtCore.Qt.NoPen)
                painter.drawPath(caret)
        painter.restore()

    # -- mouse -----------------------------------------------------------

    def _at(self, event):
        scale = self._scale()
        pos = event.position() if hasattr(event, "position") else event.pos()
        return self._picker.hits(self._panel.id, pos.x() / scale,
                                 pos.y() / scale, self._modes, self._edit)

    @staticmethod
    def ModeFor(modifiers):
        """The selection mode a set of modifiers asks for.

        Shift TOGGLES -- the same gesture adds a control that is not
        selected and removes one that is, which is what makes it usable
        for building a selection up and trimming it back down. Ctrl always
        REMOVES, whether it is a click or a marquee, so there is one
        gesture that can only ever subtract. Nothing replaces.
        """
        if modifiers & QtCore.Qt.ControlModifier:
            return "remove"
        if modifiers & QtCore.Qt.ShiftModifier:
            return "toggle"
        return "replace"

    def _PanelPos(self, event):
        scale = self._scale()
        pos = event.position() if hasattr(event, "position") else event.pos()
        return pos.x() / scale, pos.y() / scale

    def mousePressEvent(self, event):
        if event.button() != QtCore.Qt.LeftButton:
            event.ignore()
            return
        self._press = self._PanelPos(event)
        self._band = None
        self._mode = self.ModeFor(event.modifiers())

    def mouseReleaseEvent(self, event):
        if event.button() != QtCore.Qt.LeftButton or self._press is None:
            return
        start, self._press = self._press, None
        band, self._band = self._band, None
        self.update()
        if band is not None:
            hits = self._picker.within(self._panel.id, band[0], band[1],
                                       band[2], band[3], self._modes,
                                       self._edit)
            if hits:
                self.picked.emit(hits, self._mode)
            return
        # A click: an unavailable button is inert, and with nothing live
        # under the cursor an empty marquee still means "clear", which is
        # what every other picker does and what an animator expects.
        live = [b for b in self._picker.hits(self._panel.id, start[0],
                                             start[1], self._modes,
                                             self._edit) if b.live or self._edit]
        self.picked.emit(live[:1], self._mode)

    def mouseMoveEvent(self, event):
        if self._press is not None:
            x, y = self._PanelPos(event)
            # A few pixels of slop, or every click becomes a one-pixel
            # marquee and the click path never runs.
            if (abs(x - self._press[0]) > 3.0
                    or abs(y - self._press[1]) > 3.0):
                self._band = (self._press[0], self._press[1], x, y)
                self.update()
                return
        under = self._at(event)
        live = [b for b in under if b.live]
        hover = live[0].id if live else None
        if hover != self._hover:
            self._hover = hover
            if live:
                self.setToolTip("\n".join(
                    t.rsplit("/", 1)[-1] for t in live[0].targets))
                self.setCursor(QtCore.Qt.PointingHandCursor)
            elif under:
                # Name what it WOULD drive, so an animator can tell a gap
                # in the rig from a broken button.
                self.setToolTip(
                    "not in this rig yet: %s"
                    % ", ".join(under[0].objects[:4]))
                self.setCursor(QtCore.Qt.ForbiddenCursor)
            else:
                self.setToolTip("")
                self.unsetCursor()
            self.update()

    def leaveEvent(self, event):
        self._hover = None
        self.unsetCursor()
        self.update()

    def set_edit(self, edit):
        if bool(edit) != self._edit:
            self._edit = bool(edit)
            self.update()

    def set_modes(self, modes):
        """Which IK/FK mode each limb is in, keyed by its dial path."""
        if modes != self._modes:
            self._modes = dict(modes)
            self.update()

    def set_selected(self, paths):
        paths = set(str(p) for p in paths)
        if paths != self._selected:
            self._selected = paths
            self.update()


def _load_font():
    """Qt's offscreen platform ships no fonts; load one if the db is bare."""
    try:
        if QtGui.QFontDatabase.families():
            return None
    except Exception:
        return None
    for candidate in ("tahoma.ttf", "segoeui.ttf", "arial.ttf"):
        full = os.path.join(os.environ.get("WINDIR", "C:/Windows"),
                            "Fonts", candidate)
        if os.path.exists(full):
            loaded = QtGui.QFontDatabase.addApplicationFont(full)
            names = QtGui.QFontDatabase.applicationFontFamilies(loaded)
            if names:
                return names[0]
    return None


class PickerPanel(QtWidgets.QDialog):
    """Dockable-ish dialog holding one tab per picker panel."""

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi):
        if cls._instance is None:
            cls._instance = cls(usdviewApi)
        else:
            cls._instance._api = usdviewApi
        return cls._instance

    def __init__(self, usdviewApi, parent=None):
        super(PickerPanel, self).__init__(parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        # Every picker on the stage, one outer tab each, and the one
        # whose tab is showing. `_picker` stays the ACTIVE picker so the
        # single-character code below reads the same as it always did.
        self._pickers = []
        self._picker = None
        self._views = []
        self._noticeKey = None
        # The dial attributes `_RefreshModes` last read, kept so the
        # ObjectsChanged handler can ask "did a dial move?" with four
        # `AffectedObject` calls instead of walking the notice's path
        # list: the rig republishes its joints on every edit, so that
        # list is thousands of paths long and this fires on each one.
        self._dials = []
        # The prim paths the pickers live under. An edit inside one of
        # them is an edit to the picker, and the panel rebuilds; an edit
        # anywhere else on the stage is the rig moving and is ignored.
        self._pickerRoots = []
        self.setWindowTitle("Control Picker")
        self.resize(460, 720)

        layout = QtWidgets.QVBoxLayout(self)

        # EDIT MODE. Off, the picker is an animation tool: only buttons
        # whose control exists respond to hover or click, and each limb
        # shows just the half that is currently driving it. On, every
        # button becomes reachable -- including the ones whose control this
        # rig does not have yet -- so a mapping can be authored or painted
        # against them. The distinction is the user's: "they would be
        # editable to paint them, if need be, but not active for hover and
        # picking unless the control is there".
        # NO EDIT MODE. It existed so unported buttons could be reached
        # for authoring, and it owned the E key -- which is the viewport's
        # Rotate. The picker is an animation tool: a button whose control
        # this rig does not have is simply not shown. `Picker.visible`
        # still takes an `edit` flag and is still tested with it, so the
        # capability is intact for a future authoring surface; the panel
        # just never turns it on.
        self._edit = False

        self._tabs = QtWidgets.QTabWidget()
        layout.addWidget(self._tabs, 1)
        # Hotkeys, on the panel so they work whenever it has focus and
        # never fight usdview's own bindings elsewhere. The modifier rules
        # live in PickerView.ModeFor and are documented there; these are
        # the keyboard equivalents of the gestures.
        for keys, slot, tip in (
                ("Ctrl+A", self._SelectAll, "select every live control"),
                ("Ctrl+Shift+A", self._SelectNone, "clear the selection"),
                ("Ctrl+I", self._InvertSelection, "invert the selection"),
                ("F", self._FrameSelection, "frame the selection"),
        ):
            # QAction lives in QtGui on Qt6 and QtWidgets on Qt5,
            # and usdview is built against either.
            factory = getattr(QtGui, "QAction", None) or QtWidgets.QAction
            action = factory(tip, self)
            action.setShortcut(QtGui.QKeySequence(keys))
            action.setShortcutContext(QtCore.Qt.WidgetWithChildrenShortcut)
            action.triggered.connect(slot)
            self.addAction(action)

        # Q/W/E/R GO TO THE VIEWPORT, not to the picker. They are the
        # manipulator keys, and an animator who presses W expects to be
        # in Move whichever window has focus -- a picker that swallowed
        # them would be a second place where those keys mean something
        # else. Forwarded rather than reimplemented, so the gizmo stays
        # the single owner of what a tool IS.
        for keys, tool in (("Q", "select"), ("W", "translate"),
                           ("E", "rotate"), ("R", "scale")):
            factory = getattr(QtGui, "QAction", None) or QtWidgets.QAction
            action = factory("viewport %s" % tool, self)
            action.setShortcut(QtGui.QKeySequence(keys))
            action.setShortcutContext(QtCore.Qt.WidgetWithChildrenShortcut)
            action.triggered.connect(
                lambda checked=False, t=tool: self._SetViewportTool(t))
            self.addAction(action)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self.Reload()
        self._Subscribe()

    # -- wiring ----------------------------------------------------------

    def _stage(self):
        return getattr(self._api, "stage", None)

    def _Subscribe(self):
        model = getattr(self._api, "dataModel", None)
        selection = getattr(model, "selection", None)
        signal = getattr(selection, "signalPrimSelectionChanged", None)
        if signal is not None:
            signal.connect(self._OnSelectionChanged)

        # THE PICKER FOLLOWS THE RIG, not only its own buttons. The
        # IK/FK half it draws was refreshed on a switch click and nowhere
        # else, so a dial moved from anywhere ELSE -- the Avar Editor, a
        # gizmo on `avars:ikfk`, an undo, a scrub onto a keyed frame --
        # left the dead half drawn. Measured in usdview on Biped_all:
        # after setting arm_l `avars:ikfk` back to 0 (FK) from outside,
        # the panel still drew L_ArmIK and L_ArmPV and still hid L_UpArm,
        # L_LoArm and L_Hand, and only reopening it corrected them.
        for name, slot in (("currentFrameChanged", self._OnFrameChanged),
                           ("signalStageReplaced", self._OnStageReplaced)):
            emitter = getattr(model, name, None)
            if emitter is not None:
                emitter.connect(slot)

    def _ObserveStage(self, stage):
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        if stage:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._OnObjectsChanged, stage)

    def _OnObjectsChanged(self, notice, sender):
        # Cheap first: four `AffectedObject` calls, each a prefix lookup
        # in the notice's own table, and nothing else happens unless a
        # dial is among them. This runs on every edit to the stage, which
        # on this rig means every gizmo drag frame.
        for attr in self._dials:
            if notice.AffectedObject(attr):
                self._RefreshModes()
                return

        # THE PICKER ITSELF WAS EDITED. Somebody deactivated a button,
        # moved one, renamed one, retargeted one, or sublayered a whole
        # override file in. The picker is scene data, so the panel has to
        # follow an edit to it exactly as the viewport follows an edit to
        # the rig -- otherwise the override only appears on reopen and
        # authoring one is guesswork.
        #
        # Resync paths (a rename, a reparent, activation) as well as
        # changed-info paths (an attribute), because deactivating a prim
        # shows up as a resync and that is the headline case.
        if not self._pickerRoots:
            return
        for path in list(notice.GetResyncedPaths()) + list(
                notice.GetChangedInfoOnlyPaths()):
            for root in self._pickerRoots:
                if path.HasPrefix(root) or root.HasPrefix(path):
                    self._ReloadPreservingTabs()
                    return

    def _ReloadPreservingTabs(self):
        """Rebuild, and land the animator back where they were.

        A rebuild that silently jumps to the Body tab of the first
        character every time an attribute is nudged makes overriding a
        button by hand unusable."""
        outer = self._tabs.currentIndex()
        inner = self._tabs.currentWidget()
        inner = inner.currentIndex() if isinstance(
            inner, QtWidgets.QTabWidget) else 0
        self.Reload()
        if 0 <= outer < self._tabs.count():
            self._tabs.setCurrentIndex(outer)
            widget = self._tabs.currentWidget()
            if isinstance(widget, QtWidgets.QTabWidget) and (
                    0 <= inner < widget.count()):
                widget.setCurrentIndex(inner)

    def _OnFrameChanged(self, frame):
        # An `avars:ikfk` with keys on it changes the limb's mode as the
        # animator scrubs, so the drawn half has to follow the frame too.
        # The SIGNAL's frame, never `self._api.frame` -- see `_Frame`.
        self._RefreshModes(frame)

    def _OnStageReplaced(self, *args):
        self.Reload()

    def showEvent(self, event):
        # "when we open the picker it syncs to the rig": the panel is a
        # singleton that survives being closed, so re-showing it is
        # exactly when the rig has moved on without it. Four attribute
        # reads, on a gesture that happens by hand.
        super(PickerPanel, self).showEvent(event)
        self._ObserveStage(self._stage())
        self._RefreshModes()

    def closeEvent(self, event):
        # Revoke before Qt deletes the C++ side, or the notice fires into
        # a dead widget. `_instance` is deliberately kept: reopening goes
        # through `OpenPickerPanel` -> `Reload`, which re-observes.
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        super(PickerPanel, self).closeEvent(event)

    def Reload(self):
        self._tabs.clear()
        self._views = []
        # The stage may be a different one than we were watching (the
        # panel outlives a File > Open), and the cached dials belong to
        # whichever stage `_RefreshModes` last read.
        self._dials = []
        self._ObserveStage(self._stage())
        stage = self._stage()
        try:
            # SCENE DATA, and nothing else. A picker is RigExecPicker
            # prims in the stage's own layer stack, found by type, so a
            # stage with three characters gets three tabs and nobody
            # configures anything. There is no sidecar file and no
            # fallback to one: a stage either carries its picker or it
            # does not have one.
            self._pickers = pickerScene.load_all(stage)
        except Exception as exc:
            self._pickers = []
            self._picker = None
            self._status.setText("Could not read the picker: %s" % exc)
            return
        if not self._pickers:
            self._status.setText(
                "No picker on this stage. A picker is a RigExecPicker "
                "prim; add the rig's picker layer to the stage's "
                "sublayers and reopen.")
            self._picker = None
            return

        for picker in self._pickers:
            inner = QtWidgets.QTabWidget()
            for panel in picker.panels:
                # ONLY THE BODY. The facial panel's controls are not
                # ported, so every one of its buttons is dead; a tab of
                # things that cannot be clicked is worse than no tab. It
                # comes back when the face rig does, and the layout is
                # still in the layer either way.
                if (panel.label or panel.id).rsplit("/", 1)[-1] in (
                        "Facial", "Face", "panel4"):
                    continue
                view = PickerView(picker, panel)
                view.picked.connect(self._OnPicked)
                holder = QtWidgets.QScrollArea()
                holder.setWidget(view)
                holder.setWidgetResizable(True)
                inner.addTab(holder, panel.label or panel.id)
                self._views.append(view)
            # A character with one panel needs no second row of tabs.
            if inner.count() == 1:
                inner.tabBar().hide()
            self._tabs.addTab(inner, picker.name)

        self._picker = self._pickers[0]
        self._pickerRoots = [Sdf.Path(p.path) for p in self._pickers
                             if getattr(p, "path", None)]
        self._tabs.currentChanged.connect(self._OnCharacterChanged)

        self._RefreshModes()
        self._Status()
        self._OnSelectionChanged()

    def _OnCharacterChanged(self, index):
        """Follow the visible character tab.

        Everything that reports or edits one picker reads `_picker`, and
        with more than one on the stage that has to mean the one the
        animator is looking at."""
        if 0 <= index < len(self._pickers):
            self._picker = self._pickers[index]
            self._Status()

    def _LiveButtons(self):
        out = []
        for view in self._views:
            out.extend(b for b in view._picker.visible(
                view._panel.id, modes=None if view._edit else view._modes,
                edit=view._edit)
                if b.targets and (b.live or view._edit))
        return out

    def _SelectAll(self):
        self._OnPicked(self._LiveButtons(), "replace")

    def _SelectNone(self):
        model = getattr(self._api, "dataModel", None)
        if model is not None:
            model.selection.clearPrims()
            self._Status()

    def _InvertSelection(self):
        model = getattr(self._api, "dataModel", None)
        if model is None:
            return
        have = set(str(p.GetPath()) for p in model.selection.getPrims()
                   if p and p.IsValid())
        keep = [b for b in self._LiveButtons()
                if not have.issuperset(b.targets)]
        self._OnPicked(keep, "replace")

    def _FrameSelection(self):
        # usdview already knows how; borrow it rather than reimplement a
        # camera fit that would drift from the viewport's own.
        api = self._api
        for name in ("frameSelection", "FrameSelection"):
            fn = getattr(api, name, None)
            if callable(fn):
                fn()
                return

    # Every avar a control or joint can carry a pose in. Clearing these
    # IS the zero pose: `avars:t*`/`r*`/`rspin` fall back to 0 and
    # `avars:s*` to 1, which is the schema's own definition of rest.
    _POSE_AVARS = ("avars:tx", "avars:ty", "avars:tz",
                   "avars:rx", "avars:ry", "avars:rz", "avars:rspin",
                   "avars:sx", "avars:sy", "avars:sz")

    def _ZeroControls(self):
        """Zero the selected controls, or the whole rig if none are.

        CLEAR, not Set(0). Clearing the opinion returns the attribute to
        its schema fallback -- 0 for translate and rotate, 1 for scale --
        which is the same zero pose, leaves no authored junk behind, and
        shrinks the layer instead of growing it.

        ONE `Sdf.ChangeBlock` around the whole thing, and that is the
        difference between fast and unusable: the rig re-evaluates once
        per *evaluate after any edit*, not once per edit, and that
        evaluate costs ~190 ms on this rig. Zeroing 180 controls one
        attribute at a time would pay it 1800 times.
        """
        stage = self._stage()
        model = getattr(self._api, "dataModel", None)
        if stage is None:
            return

        prims = []
        if model is not None:
            prims = [p for p in model.selection.getPrims()
                     if p and p.IsValid() and not p.IsPseudoRoot()
                     and str(p.GetTypeName()) in ("RigExecControl",
                                                  "RigExecJoint")]
        whole = not prims
        if whole:
            prims = [p for p in stage.Traverse()
                     if str(p.GetTypeName()) == "RigExecControl"]

        cleared = 0
        scope = self._UndoScope("Zero controls")
        try:
            with Sdf.ChangeBlock():
                for prim in prims:
                    for name in self._POSE_AVARS:
                        attr = prim.GetAttribute(name)
                        if (attr and attr.IsValid()
                                and attr.HasAuthoredValue()):
                            attr.Clear()
                            cleared += 1
        finally:
            if scope is not None:
                scope.__exit__(None, None, None)

        self._RefreshModes()
        for view in self._views:
            view.update()
        self._status.setText(
            "Zeroed %d control%s (%d authored avars cleared)%s"
            % (len(prims), "" if len(prims) == 1 else "s", cleared,
               " -- nothing was selected, so the whole rig" if whole
               else ""))

    def _UndoScope(self, label):
        """The shared undo scope, when the container provides one."""
        maker = getattr(self, "EditScope", None)
        if maker is None:
            return None
        try:
            scope = maker([], label)
            scope.__enter__()
            return scope
        except Exception:
            return None

    def _Status(self):
        if not self._pickers:
            return
        live = dead = 0
        for picker in self._pickers:
            counts = picker.coverage()
            live, dead = live + counts[0], dead + counts[1]
        who = ("" if len(self._pickers) == 1
               else " across %d characters" % len(self._pickers))
        text = "%d buttons pickable%s." % (live, who)
        if dead:
            text += (" %d more are in the layout but their control does "
                     "not exist on this rig yet." % dead)
        self._status.setText(text)

    def _OnPicked(self, buttons, mode):
        """Apply one pick -- a click or a marquee -- to the selection.

        `mode` is "replace", "toggle" or "remove"; the view decides it
        from the modifiers so a click and a marquee cannot drift apart.
        """
        stage = self._stage()
        model = getattr(self._api, "dataModel", None)
        if stage is None or model is None:
            return

        # A command or an attribute button acts rather than selects, and
        # only on a plain click of exactly one: a marquee that swept
        # across a limb should not silently flip its IK/FK state or zero
        # the character.
        if len(buttons) == 1 and mode == "replace":
            if buttons[0].command == "zero_ctrls":
                self._ZeroControls()
                return
            if buttons[0].attr_target:
                self._Switch(buttons[0])
                return

        wanted = []
        for button in buttons:
            for target in button.targets:
                prim = stage.GetPrimAtPath(Sdf.Path(target))
                if prim and prim.IsValid() and prim not in wanted:
                    wanted.append(prim)

        selection = model.selection
        current = [p for p in selection.getPrims() if p and p.IsValid()]
        have = set(str(p.GetPath()) for p in current)

        if mode == "replace":
            keep = wanted
        elif mode == "remove":
            drop = set(str(p.GetPath()) for p in wanted)
            keep = [p for p in current if str(p.GetPath()) not in drop]
        else:                                   # toggle
            keep = [p for p in current]
            for prim in wanted:
                path = str(prim.GetPath())
                if path in have:
                    keep = [p for p in keep if str(p.GetPath()) != path]
                else:
                    keep.append(prim)

        with getattr(selection, "batchPrimChanges", _NullContext()):
            selection.clearPrims()
            for prim in keep:
                selection.addPrim(prim)
        self._Status()

    def _SetViewportTool(self, tool):
        """Hand Q/W/E/R to the viewport manipulator.

        Read-only through gizmoUI's public surface, and any failure is
        silent: a session with no viewport tools should behave as it
        always did rather than raise out of a keypress.
        """
        try:
            import gizmoUI
            controller = gizmoUI.GetController()
            if controller is None:
                return
            controller.SetTool({
                "select": gizmoUI.TOOL_SELECT,
                "translate": gizmoUI.TOOL_TRANSLATE,
                "rotate": gizmoUI.TOOL_ROTATE,
                "scale": gizmoUI.TOOL_SCALE,
            }[tool])
        except Exception:
            pass


    def _Frame(self, frame=None):
        """The frame the dials are read at, as a TimeCode.

        \\p frame is the one the change signal carried, and it has to be
        passed rather than re-read: RootDataModel emits
        currentFrameChanged(value) BEFORE it assigns `_currentFrame`, so
        `self._api.frame` inside the handler is the frame the animator
        just left. Measured with a keyed arm_r `avars:ikfk` (0 at frame
        1, 1 at frame 10): re-reading the api gave FK at frame 10 and IK
        back at frame 1 -- every scrub one step behind.
        """
        if frame is None:
            frame = getattr(self._api, "frame", None)
        if isinstance(frame, Usd.TimeCode):
            return frame
        try:
            return Usd.TimeCode(float(frame))
        except (TypeError, ValueError):
            return Usd.TimeCode.Default()

    def _RefreshModes(self, frame=None):
        """Read each limb's IK/FK dial off the RIG and tell the views.

        A limb in IK draws its IK controls only, and vice versa -- the same
        thing the fade wiring does to the viewport gizmos, but here the
        button is removed rather than ghosted, because a picker crowded
        with controls that are not driving anything is worse than a smaller
        one.

        This is the whole sync: the stage is the authority for both the
        half that is drawn and the label on the switch, and every way in
        (opening the panel, the frame, an ObjectsChanged on a dial, a
        click on the switch) comes through here. On the biped that is 8
        dial paths, 4 of which resolve.
        """
        stage = self._stage()
        if stage is None or not self._pickers:
            return
        frame = self._Frame(frame)
        modes = {}
        dials = []
        paths = []
        for picker in self._pickers:
            paths.extend(p for p in picker.dials() if p not in paths)
        for path in paths:
            # `dials()` also hands back the switch buttons' PRIM paths
            # (`.../arm_l_params`), which are not attributes and simply
            # do not resolve -- skipped here, not an error.
            attr = stage.GetAttributeAtPath(Sdf.Path(path))
            if not attr or not attr.IsValid():
                continue
            dials.append(attr)
            value = attr.Get(frame)
            if value is None:
                continue
            # The dial IS the blend weight: 0 = FK, 1 = IK. Anything
            # in between is a hand-over, and both sets stay visible so the
            # animator can still grab either.
            if float(value) >= 0.999:
                modes[path] = "ik"
            elif float(value) <= 0.001:
                modes[path] = "fk"
        self._dials = dials

        # ...and the switch's own label, which is BAKED from the conventional tool and so
        # reads back whatever state the rig happened to be exported in
        # until the animator clicks it once. Same source of truth as the
        # half that is drawn, or the panel contradicts itself.
        relabelled = False
        buttons = [b for picker in self._pickers for b in picker.buttons]
        for button in buttons:
            target = button.attr_target
            if not target:
                continue
            prim = stage.GetPrimAtPath(Sdf.Path(target["path"]))
            attr = prim.GetAttribute(target["attr"]) if prim else None
            if not attr or not attr.IsValid():
                continue
            label = button.label_for(attr.Get(frame))
            if label and label != button.value:
                button.value = label
                relabelled = True

        for view in self._views:
            view.set_modes(modes)
            if relabelled:
                view.update()
        return modes

    def _Switch(self, button):
        """Cycle an attribute button and write it."""
        stage = self._stage()
        target = button.attr_target
        prim = stage.GetPrimAtPath(Sdf.Path(target["path"]))
        if not prim or not prim.IsValid():
            return
        attr = prim.GetAttribute(target["attr"])
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute(target["attr"],
                                        Sdf.ValueTypeNames.Float)
        value, label = button.next_value(attr.Get())
        if value is None:
            return
        attr.Set(float(value))
        button.value = label
        # The limb just changed mode, so the other half of its controls
        # should appear and this half disappear.
        self._RefreshModes()
        # Reflect it on every view: the same limb's switch may be drawn on
        # both panels, and the label is what the animator reads back.
        for view in self._views:
            view.update()
        self._status.setText("%s -> %s (%s = %g)"
                             % (target.get("source") or button.id, label,
                                target["attr"], value))

    def _OnSelectionChanged(self, *args):
        model = getattr(self._api, "dataModel", None)
        if model is None:
            return
        paths = [str(p.GetPath()) for p in model.selection.getPrims()
                 if p and p.IsValid()]
        for view in self._views:
            view.set_selected(paths)


class _NullContext(object):
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def OpenPickerPanel(usdviewApi, undoStack=None):
    panel = PickerPanel.GetInstance(usdviewApi)
    panel.Reload()
    panel.show()
    panel.raise_()
    return panel
