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

from pxr import Sdf, Tf, Usd, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

if __name__ != "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import avarEditorModel
import gizmoMath
import ikfkMatch
import pickerModel
import pickerScene
import rigExecUndo
import spaceMatch

try:
    import sessionRegistry
except ImportError:                    # loader that did not add our dir
    import os as _os
    import sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
    import sessionRegistry


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
        # THE KNOT IS (anchor, OUT, IN), not (anchor, in, out).
        #
        # Read the other way round the segment leaving a knot took that
        # knot's INCOMING handle and the next knot's OUTGOING one, which
        # is a curve tied to the wrong ends: every face button came out
        # self-intersecting -- the cheeks as folded ribbons with a pinch
        # at the seam, which is what "no curved shapes, weird triangles"
        # looks like from the outside. Rendered side by side the swap is
        # not a matter of taste: the same six buttons go from twisted to
        # clean arcs.
        points = button.bezier
        path.moveTo(*points[0][0])
        for i in range(len(points)):
            a = points[i]
            b = points[(i + 1) % len(points)]
            c1 = a[1] if len(a) > 1 else a[0]
            c2 = b[2] if len(b) > 2 else b[0]
            path.cubicTo(c1[0], c1[1], c2[0], c2[1], b[0][0], b[0][1])
        path.closeSubpath()
    elif shape == "trapezoid":
        # A SLOPE INSETS AN EDGE; IT NEVER FLARES PAST THE BOX.
        #
        # The slopes arrive in +/- pairs -- L_InLid is +3/+3 and L_UpLid2
        # is -3/-3 -- and the old form applied both to the TOP edge, so a
        # negative one ran the top from -3 to w+3 and the shape spilled
        # outside its own bounds on both sides. Around an eye that is
        # dozens of lid and socket segments each three pixels too wide,
        # colliding with their neighbours: the chaotic wedges.
        #
        # Positive insets the top, negative insets the bottom. The pair
        # then mirrors vertically, which is what an upper-lid and a
        # lower-lid segment need in order to tile.
        ls, rs = button.left_slope, button.right_slope
        if button.direction in ("top", "bottom"):
            top_l = ls if ls > 0 else 0.0
            bot_l = -ls if ls < 0 else 0.0
            top_r = rs if rs > 0 else 0.0
            bot_r = -rs if rs < 0 else 0.0
            pts = [(top_l, 0.0), (w - top_r, 0.0),
                   (w - bot_r, h), (bot_l, h)]
        else:
            left_t = ls if ls > 0 else 0.0
            left_b = -ls if ls < 0 else 0.0
            right_t = rs if rs > 0 else 0.0
            right_b = -rs if rs < 0 else 0.0
            pts = [(0.0, left_t), (w, right_t),
                   (w, h - right_b), (0.0, h - left_b)]
        # ROUNDNESS IS NOT DECORATION ON A RING. The eye tiles carry
        # roundness 2 in the source and this branch dropped it, so every
        # tile kept square corners -- and square corners on a circle are
        # exactly where neighbouring tiles bite into each other. The
        # polygon helper already rounds a corner list; the trapezoid is
        # a corner list.
        _polygon(path, pts, max(button.roundness, 0.0))
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


def DrawButton(painter, button, fontFamily, hover, selectedPaths):
    """Paint one button in its panel's coordinates.

    Shared by the docked panel and the hover picker, so a button looks
    the same wherever it is drawn. `hover` is the hovered button's id
    and `selectedPaths` the selected prim paths, as strings.
    """
    painter.save()
    painter.translate(button.x, button.y)
    if button.rotation:
        painter.translate(button.w * 0.5, button.h * 0.5)
        painter.rotate(button.rotation)
        painter.translate(-button.w * 0.5, -button.h * 0.5)

    fill = QtGui.QColor(*button.fill)
    if not button.decoration and not button.live:
        fill.setAlpha(int(fill.alpha() * _DIM))
    if button.id == hover and button.live:
        fill = fill.lighter(125)

    path = _path_for(button)
    painter.setBrush(QtGui.QBrush(fill))
    selected = bool(button.targets) and selectedPaths.issuperset(
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

    # A SWITCH WITH NO STATIC LABEL STILL HAS A VALUE. The value used
    # to be drawn inside `if button.text:`, so the six switches the
    # studio left unlabelled -- head, neck, both FK arms, both arm
    # IKs -- painted as empty boxes: they carry no `ui:text`, and the
    # live space never got a chance to draw. The gate is now "is
    # there anything to say", and the label and the value are placed
    # separately so either can stand alone.
    if button.text or button.value:
        font = QtGui.QFont(fontFamily or "Sans")
        font.setPixelSize(max(int(round(button.font_size)), 5))
        font.setBold(button.bold)
        box = QtCore.QRectF(2, 0, button.w - 4, button.h)
        # Fit the PAIR, not just the label. Shrinking to fit "Foot"
        # and then drawing "world" beside it is how "worldFoot" and
        # "fooPV" happened.
        both = " ".join(x for x in (button.text, button.value) if x)
        for _ in range(6):
            if (QtGui.QFontMetricsF(font).horizontalAdvance(both)
                    <= box.width() - (14.0 if button.value else 0.0)
                    or font.pixelSize() <= 5):
                break
            font.setPixelSize(font.pixelSize() - 1)
        painter.setFont(font)
        align = {"left": QtCore.Qt.AlignLeft,
                 "right": QtCore.Qt.AlignRight}.get(
                     button.h_align, QtCore.Qt.AlignHCenter)
        if button.checkbox:
            box.setLeft(box.left() + min(button.h - 4.0, 11.0) + 5.0)
            align = QtCore.Qt.AlignLeft

        # THE TWO HALVES DO NOT OVERLAP. Both used to be drawn into
        # the same rect with opposite alignments, which reads fine
        # only while they happen not to meet in the middle -- and on
        # this rig they met on every leg.
        label_box, value_box, value_align, caret_x = box, None, None, 0.0
        if button.value:
            metrics = QtGui.QFontMetricsF(font)
            want = min(metrics.horizontalAdvance(button.value) + 14.0,
                       box.width())
            if not button.text:
                want = box.width()          # nothing to share with
                label_box = None
            if align == QtCore.Qt.AlignRight:
                value_box = QtCore.QRectF(box.left() + 12, box.top(),
                                          want - 12, box.height())
                value_align = QtCore.Qt.AlignLeft
                caret_x = box.left() + 5
                if label_box is not None:
                    label_box = QtCore.QRectF(box)
                    label_box.setLeft(box.left() + want)
            else:
                value_box = QtCore.QRectF(box.right() - want, box.top(),
                                          want - 12, box.height())
                value_align = QtCore.Qt.AlignRight
                caret_x = box.right() - 7
                if label_box is not None:
                    label_box = QtCore.QRectF(box)
                    label_box.setRight(box.right() - want)

        if button.text and label_box is not None:
            painter.setPen(QtGui.QColor(*button.text_color))
            painter.drawText(label_box, align | QtCore.Qt.AlignVCenter,
                             button.text)
        if button.value:
            painter.setPen(QtGui.QColor(*button.value_color))
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
        (self._ox, self._oy), (self._cw, self._ch) =             pickerModel.content_box(picker, panel)
        self.setMinimumSize(int(self._cw), int(self._ch))
        self._font = _load_font()
        self._backgroundImage = QtGui.QPixmap()
        if panel.backgroundImage:
            self._backgroundImage.loadFromData(panel.backgroundImage)

    # -- painting --------------------------------------------------------

    def _scale(self):
        return min(self.width() / self._cw,
                   self.height() / self._ch) or 1.0

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor(42, 42, 42))
        scale = self._scale()
        painter.scale(scale, scale)
        # Into the panel's own coordinates: the buttons carry absolute
        # positions from the canvas they were authored on, so the view
        # slides that canvas until this panel's content starts at the
        # corner. Every screen-to-panel mapping below undoes the same
        # shift -- if one of them forgot, clicks would land on whichever
        # button happened to be under the offset instead.
        painter.translate(-self._ox, -self._oy)
        painter.setBrush(QtGui.QBrush(QtGui.QColor(*self._panel.fill)))
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 200)))
        painter.drawRect(QtCore.QRectF(self._ox, self._oy,
                                       self._cw, self._ch))
        if not self._backgroundImage.isNull():
            painter.drawPixmap(QtCore.QRectF(self._panel.x, self._panel.y,
                                              self._panel.w, self._panel.h),
                               self._backgroundImage,
                               QtCore.QRectF(self._backgroundImage.rect()))

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
        DrawButton(painter, button, self._font, self._hover, self._selected)

    # -- mouse -----------------------------------------------------------

    def _at(self, event):
        scale = self._scale()
        pos = event.position() if hasattr(event, "position") else event.pos()
        return self._picker.hits(self._panel.id,
                                 pos.x() / scale + self._ox,
                                 pos.y() / scale + self._oy,
                                 self._modes, self._edit)

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

    @staticmethod
    def WantsMirror(modifiers):
        """Whether this gesture should bring the opposite side along.

        Alt, and Alt alone was free: Ctrl removes, Shift toggles, and
        nothing else was reading it. It composes with both rather than
        replacing them, so Alt+Shift adds a pair to a selection and
        Alt+Ctrl takes a pair out of one.
        """
        return bool(modifiers & QtCore.Qt.AltModifier)

    def _PanelPos(self, event):
        scale = self._scale()
        pos = event.position() if hasattr(event, "position") else event.pos()
        return pos.x() / scale + self._ox, pos.y() / scale + self._oy

    def _WithMirrors(self, buttons):
        """The buttons, plus their opposite numbers when Alt is held.

        The pairing is the studio picker's own <mirror> field, so this
        reaches the controls whose names carry no side at all -- which
        deriving L_/R_ from a name never would. A button with no mirror
        simply comes through alone, and nothing is added twice.
        """
        if not getattr(self, "_wantMirror", False):
            return buttons
        byName = {}
        for b in self._picker.visible(self._panel.id, edit=self._edit):
            byName[b.id] = b
        out, seen = [], set()
        for b in buttons:
            for candidate in (b, byName.get((b.mirror or "").rsplit("/", 1)[-1])):
                if candidate is not None and id(candidate) not in seen:
                    seen.add(id(candidate))
                    out.append(candidate)
        return out

    def mousePressEvent(self, event):
        if event.button() != QtCore.Qt.LeftButton:
            event.ignore()
            return
        self._press = self._PanelPos(event)
        self._band = None
        self._mode = self.ModeFor(event.modifiers())
        self._wantMirror = self.WantsMirror(event.modifiers())

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
                self.picked.emit(self._WithMirrors(hits), self._mode)
            return
        # A click: an unavailable button is inert, and with nothing live
        # under the cursor an empty marquee still means "clear", which is
        # what every other picker does and what an animator expects.
        live = [b for b in self._picker.hits(self._panel.id, start[0],
                                             start[1], self._modes,
                                             self._edit) if b.live or self._edit]
        self.picked.emit(self._WithMirrors(live[:1]), self._mode)

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

    # One panel per usdview session, filed under its main window: several
    # sessions can share this module in one process.
    _sessions = sessionRegistry.SessionRegistry("picker panels")

    @classmethod
    def GetInstance(cls, usdviewApi, undoStack=None):
        panel = cls._sessions.Get(usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(
                usdviewApi, cls(usdviewApi, undoStack=undoStack))
        else:
            panel._api = usdviewApi
            if undoStack is not None:
                panel._undo = undoStack
        return panel

    def __init__(self, usdviewApi, parent=None, undoStack=None):
        super(PickerPanel, self).__init__(parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        # The shared stack, so Ctrl+Z reaches a picker edit the same as
        # it reaches a gizmo drag. None in a test that built the panel
        # alone, and every edit below still works -- just unrecorded.
        self._undo = undoStack
        # A switch shown through the preview channel and not yet authored:
        # (stage, gizmoMath.Writer, undo label). See _PreviewThenCommit.
        self._pendingCommit = None
        # Every picker on the stage, one outer tab each, and the one
        # whose tab is showing. `_picker` stays the ACTIVE picker so the
        # single-character code below reads the same as it always did.
        self._pickers = []
        self._visAttrs = []
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
        # Callables told when what the buttons show changes: "reload"
        # after the pickers are re-read, "update" for anything else (a
        # limb's IK/FK half, a switch label, a toggle's tick, the
        # selection). The hover picker draws the same buttons and follows
        # through these.
        self._listeners = []
        self._modes = {}
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
        ) + tuple(
                # PANEL HOTKEYS. 1 and 2 swap between the two panels, and
                # the digit matches the panel number an animator counts
                # from -- pressing 1 for the first panel rather than 0 is
                # what everyone tries first. Extended through 9 so a rig
                # with more panels needs no new binding. Digits are free
                # here -- the picker's own gestures are modifier-based
                # and Q/W/E/R are forwarded to the viewport.
                (str(d + 1), (lambda i: lambda: self._ShowPanel(i))(d),
                 "show panel %d" % (d + 1))
                for d in range(9)):
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

    def AddListener(self, listener):
        if listener not in self._listeners:
            self._listeners.append(listener)

    def RemoveListener(self, listener):
        if listener in self._listeners:
            self._listeners.remove(listener)

    def _Notify(self, what):
        for listener in list(self._listeners):
            try:
                listener(what)
            except Exception:
                pass

    def Pickers(self):
        """Every picker on the stage, as the panel read them."""
        return list(self._pickers)

    def Modes(self):
        """Each limb's showing IK/FK half, keyed by its dial path."""
        return dict(self._modes)

    def Pick(self, buttons, mode):
        """Apply a pick made somewhere other than the panel's own views:
        the same selection, command and switch rules as a click here."""
        self._OnPicked(buttons, mode)

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
        # left the dead half drawn. Measured in usdview on Biped_stack:
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
        # The controls hidden or shown from outside the picker (usdview's
        # own hide, an override layer): keep the toggle's tick honest.
        for attr in self._visAttrs:
            if notice.AffectedObject(attr):
                self._SyncToggles()
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
        # a dead widget. The session's panel is deliberately kept:
        # reopening goes through `OpenPickerPanel` -> `Reload`, which
        # re-observes.
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








                live = sum(1 for button in picker.buttons
                           if button.parent == panel.id
                           and button.live and not button.decoration)
                if not live:
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
        self._SyncToggles()
        self._Status()
        self._OnSelectionChanged()
        self._Notify("reload")

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


    def _ControlsPrim(self, picker=None):
        """The prim holding the rig's controls: the `Controls` scope under
        `Aux` in the rig the picker names (the whole stage when it names
        none), or any `Controls` scope when no `Aux` parents one."""
        stage = self._stage()
        picker = picker or self._picker
        if stage is None or picker is None:
            return None
        root = stage.GetPrimAtPath(getattr(picker, "rig_path", "") or "/")
        if not root or not root.IsValid():
            root = stage.GetPseudoRoot()
        fallback = None
        for prim in Usd.PrimRange(root):
            if prim.GetName() != "Controls":
                continue
            if prim.GetParent().GetName() == "Aux":
                return prim
            fallback = fallback or prim
        return fallback

    def _ToggleControlsVisibility(self):
        """Hide or show the rig's controls, as usdview's own hide does:
        a visibility opinion in the session layer, so the rig's files are
        never touched and reopening the stage shows them again."""
        stage = self._stage()
        prim = self._ControlsPrim()
        if stage is None or prim is None:
            self._status.setText("No Controls prim under Aux on this rig.")
            return
        imageable = UsdGeom.Imageable(prim)
        hide = imageable.ComputeVisibility() != UsdGeom.Tokens.invisible
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            attr = imageable.GetVisibilityAttr()
            if hide:
                attr.Set(UsdGeom.Tokens.invisible)
            else:
                attr.Clear()
                # Still hidden: the rig itself, or an ancestor, says so.
                if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible:
                    attr.Set(UsdGeom.Tokens.inherited)
        self._SyncToggles()
        self._status.setText("Controls %s." % ("hidden" if hide else "shown"))

    def _SyncToggles(self):
        """Tick each Ctrl Vis button when its rig's controls are visible."""
        self._visAttrs = []
        for picker in self._pickers:
            prim = self._ControlsPrim(picker)
            shown = prim is not None and UsdGeom.Imageable(
                prim).ComputeVisibility() != UsdGeom.Tokens.invisible
            if prim is not None:
                self._visAttrs.append(UsdGeom.Imageable(prim).GetVisibilityAttr())
            for button in picker.buttons:
                if button.command == "ctrl_vis":
                    button.checked = shown
        for view in self._views:
            view.update()
        self._Notify("update")

    def _ZeroControls(self):
        """Zero the selected controls, or the whole rig if none are.

        CLEAR, not Set(0), and that is what makes this right for the
        channels that are not zero at zero. Clearing an opinion returns
        the attribute to what the RIG says, which for `avars:t*`/`r*` is
        the schema's 0 and for `avars:s*` its 1, but for a custom channel
        is whatever default the rig authored -- `spaces:active` of 2 on a
        knee's pole vector, `avars:space` of 1 on the eye target. Setting
        zero would be wrong for all three; clearing is right for all
        three, leaves no authored junk behind, and shrinks the layer
        instead of growing it.

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

        # The exact attributes about to be cleared, gathered BEFORE the
        # change block so the undo scope can snapshot them. Only the ones
        # with an authored value: an attribute nobody has posed has
        # nothing to clear and nothing to restore.
        #
        # Every POSE channel, not just the nine transform avars. A rig
        # zeroed with a limb still in IK, a foot still rolled, a knee's
        # pole vector switched out of its default space or an eye still
        # locked to the look target is not zeroed, and each of those
        # lives in a namespace of its own. avarEditorModel.PoseChannels
        # draws the line, and draws it once for both panels.
        doomed = []
        for prim in prims:
            for attr in avarEditorModel.PoseChannels(prim, stage):
                if attr.HasAuthoredValue():
                    doomed.append(attr.GetPath())

        cleared = 0
        scope = self._UndoScope("Zero controls", doomed)
        try:
            with Sdf.ChangeBlock():
                for path in doomed:
                    stage.GetAttributeAtPath(path).Clear()
                    cleared += 1
        finally:
            if scope is not None:
                scope.__exit__(None, None, None)

        self._RefreshModes()
        for view in self._views:
            view.update()
        self._Notify("update")
        self._status.setText(
            "Zeroed %d control%s (%d authored avars cleared)%s"
            % (len(prims), "" if len(prims) == 1 else "s", cleared,
               " -- nothing was selected, so the whole rig" if whole
               else ""))

    def _UndoScope(self, label, paths):
        """One undo entry covering `paths`, when there is a stack.

        `paths` is not optional and was the whole bug: this used to open
        a scope over an EMPTY list, which snapshots nothing, records no
        entry and leaves the stack untouched while the stage changes --
        so Zero Controls, which zeroes the WHOLE RIG when nothing is
        selected, could not be undone at all. A scope has to be told
        what it is covering.
        """
        if self._undo is None or not paths:
            return None
        try:
            scope = rigExecUndo.SpecScope(
                self._stage(), paths, self._undo, label)
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
            if buttons[0].command == "ctrl_vis":
                self._ToggleControlsVisibility()
                return
            if buttons[0].attr_target:
                # No position passed: _OnPicked has no event, and the
                # cursor is still where the click landed, which is where
                # a menu belongs anyway.
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
            controller = gizmoUI.GetController(self._api)
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

        # ...and the switch's own label, which is BAKED at export and so
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

        self._modes = dict(modes)
        for view in self._views:
            view.set_modes(modes)
            if relabelled:
                view.update()
        self._Notify("update")
        return modes

    def _ShowPanel(self, index):
        """Show the index'th panel of the picker that is showing.

        The panels are an inner QTabWidget inside the per-picker outer
        one, so this moves the inner selection and leaves which PICKER is
        up alone -- pressing 1 should not jump you to another character's
        Body panel.
        """
        inner = self._tabs.currentWidget()
        if not isinstance(inner, QtWidgets.QTabWidget):
            return
        if 0 <= index < inner.count():
            inner.setCurrentIndex(index)
            self._status.setText("panel: %s" % inner.tabText(index))

    def _Switch(self, button, at=None):
        """Act on an attribute button: toggle two values, offer a menu of
        more.

        A space switch names three to five places -- world, chest, head,
        hips -- and cycling through them meant an animator hunting for
        `hips` clicked until it came round, reading the label each time.
        So anything with more than two choices opens a MENU at the
        cursor and jumps straight to the one picked. IK/FK has exactly
        two and stays a toggle, which is the whole interaction for it.

        `at` is where to pop the menu; without it the menu appears at the
        button, which is what a keyboard or scripted call should get.
        """
        stage = self._stage()
        target = button.attr_target
        labels = (target or {}).get("enum") or []
        if target and len(labels) > 2:
            chosen = self._ChooseValue(button, labels, at)
            if chosen is None:
                return
            self._WriteSwitch(button, chosen)
            return
        limb = self._LimbForSwitch(target)
        if limb is not None and self._MatchedSwitch(button, target, limb):
            return
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
        self._ApplySwitch(button, target, attr, value, label)

    # -- IK/FK: match, then switch ----------------------------------------

    def _LimbForSwitch(self, target):
        """The limb whose IK/FK switch this button drives, or None."""
        stage = self._stage()
        if stage is None or not target:
            return None
        try:
            path = Sdf.Path(target["path"]).AppendProperty(target["attr"])
        except Exception:
            return None
        if getattr(self, "_limbStage", None) is not stage:
            self._limbStage = stage
            self._limbs = ikfkMatch.FindLimbs(stage)
            self._limbRest = {}
        for limb in self._limbs:
            if limb.switchPath == path:
                return limb
        return None

    def _MatchedSwitch(self, button, target, limb):
        """Switch a limb to its other half without moving it: the half
        taking over is matched to the joints first (ikfkMatch.Plan), and
        both land as one edit and one undo entry. False when the match
        could not be made, so the caller falls back to a plain toggle."""
        stage = self._stage()
        try:
            time = self._Frame()
            values, done = ikfkMatch.PlanSwitch(stage, [limb], time,
                                                self._limbRest)
            if values:
                self._PreviewThenCommit(values, ikfkMatch.SwitchLabel(done),
                                        time, self._WriteMode())
        except Exception as error:
            self._status.setText("IK/FK match failed (%s); switched "
                                 "without matching." % error)
            return False
        if not done:
            return False
        _, toIk, channels = done[0]
        value = limb.ikValue if toIk else limb.fkValue
        labels = target.get("enum") or []
        if labels:
            index = int(round(value))
            if target.get("invert"):
                index = len(labels) - 1 - index
            if 0 <= index < len(labels):
                button.value = labels[index]
        self._RefreshModes()
        for view in self._views:
            view.update()
        self._Notify("update")
        self._status.setText("%s -> %s, matched (%d channels)" % (
            limb.switchControl.name, "IK" if toIk else "FK", channels))
        return True

    def _WriteMode(self):
        """Where a switch lands: the gizmo's write mode (keys in animation
        mode, defaults otherwise), so a switch and a drag author alike."""
        try:
            import gizmoUI
            controller = gizmoUI.GetController(self._api)
            if controller is not None:
                return controller.WriteMode()
        except Exception:
            pass
        return gizmoMath.WRITE_ANIMATION

    def _ChooseValue(self, button, labels, at):
        """A menu of the named spaces, returning the label picked."""
        menu = QtWidgets.QMenu()
        current = button.value
        for name in labels:
            action = menu.addAction(str(name))
            action.setCheckable(True)
            action.setChecked(name == current)
        where = at if at is not None else QtGui.QCursor.pos()
        picked = menu.exec(where)
        return picked.text() if picked is not None else None

    def _WriteSwitch(self, button, label):
        """Write the value whose label the animator picked."""
        stage = self._stage()
        target = button.attr_target
        prim = stage.GetPrimAtPath(Sdf.Path(target["path"]))
        if not prim or not prim.IsValid():
            return
        attr = prim.GetAttribute(target["attr"])
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute(target["attr"],
                                        Sdf.ValueTypeNames.Float)
        labels = target.get("enum") or []
        if label not in labels:
            return
        index = labels.index(label)
        value = index
        if target.get("invert"):
            value = len(labels) - 1 - index
        self._ApplySwitch(button, target, attr, value, label)

    def _ApplySwitch(self, button, target, attr, value, label):
        """Write one switch value and let every view catch up."""
        # An IK/FK switch is a pose change like any other -- it decides
        # which half of a limb drives the joints -- so it belongs on the
        # same stack as the drag that follows it.
        stage = self._stage()
        time = self._Frame()
        mode = self._WriteMode()
        matched = self._MatchSpace(stage, attr, value, label, time, mode)
        if matched is None:
            self._PreviewThenCommit({attr.GetPath(): float(value)},
                                    "Switch %s" % target["attr"], time, mode)
        button.value = label
        # The limb just changed mode, so the other half of its controls
        # should appear and this half disappear.
        self._RefreshModes()
        # Reflect it on every view: the same limb's switch may be drawn on
        # both panels, and the label is what the animator reads back.
        for view in self._views:
            view.update()
        self._Notify("update")
        self._status.setText("%s -> %s (%s = %g)%s"
                             % (target.get("source") or button.id, label,
                                target["attr"], value,
                                ", matched (%d channels)" % matched
                                if matched else ""))

    def _MatchSpace(self, stage, attr, value, label, time, mode):
        """Switch a control's space without moving it, as one edit and one
        undo entry. The number of channels matched, or None when no space
        switch reads `attr` or the match could not be made -- the caller
        then writes the value alone."""
        control = spaceMatch.SwitchTarget(stage, attr.GetPath())
        if control is None:
            return None
        try:
            rig = ikfkMatch._RigRoot(stage.GetPrimAtPath(control))
            evaluate = ikfkMatch.Evaluator(stage, rig)
            plan = spaceMatch.Plan(stage, control, attr.GetPath(), value,
                                   time, evaluate)
            self._PreviewThenCommit(plan, "Switch %s to %s"
                                    % (control.name, label), time, mode)
        except Exception as error:
            self._status.setText("Space match failed (%s); switched "
                                 "without matching." % error)
            return None
        return len(plan) - 1

    def _PreviewThenCommit(self, values, label, time, mode):
        """Show `values` ({attribute path: value}) now, author them next.

        The interactive half goes to Hydra through the preview channel, the
        way a drag does: no stage edit happens on the click, so nothing
        re-reads, re-digests or rebuilds while the animator waits. The stage
        write follows on the next turn of the event loop, once the viewport
        has drawn the new pose, as one edit and one undo entry; the preview
        is withdrawn first without a republish, so the commit's own notice
        is the one evaluation (gizmoUI._EndDrag ends a drag the same way).
        """
        stage = self._stage()
        if stage is None:
            return
        # Never two in flight: an earlier switch lands before this one is
        # planned against it.
        self._CommitPending()
        writer = gizmoMath.Writer(stage, time, mode)
        for path, value in values.items():
            attr = stage.GetAttributeAtPath(Sdf.Path(str(path)))
            if attr:
                writer.Set(attr, value)
        if not writer.HasPending():
            return
        try:
            import gizmoPreview
            gizmoPreview.Push(writer.Pending(), session=self._api,
                              stage=stage)
            import avarEditorUI
            avarEditorUI._FollowPreviewInViewport(self._api)
        except Exception:
            pass
        self._pendingCommit = (stage, writer, label)
        QtCore.QTimer.singleShot(0, self._CommitPending)

    def _CommitPending(self):
        """Author the switch _PreviewThenCommit showed, if one is waiting."""
        pending, self._pendingCommit = self._pendingCommit, None
        if pending is None:
            return
        stage, writer, label = pending
        try:
            import gizmoPreview
        except ImportError:
            gizmoPreview = None
        if stage is not self._stage():
            # The stage was replaced under the switch: nothing to author it
            # onto, and nothing left to preview.
            writer.Clear()
            if gizmoPreview is not None:
                gizmoPreview.End(session=self._api, stage=stage)
            return
        if gizmoPreview is not None:
            gizmoPreview.End(session=self._api, stage=stage, publish=False)
        scope = None
        if self._undo is not None:
            scope = rigExecUndo.SpecScope(stage, list(writer.Pending()),
                                          self._undo, label)
            scope.__enter__()
        authored = []
        try:
            authored = writer.CommitToStage()
        except Exception as error:
            if scope is not None:
                scope.__exit__(type(error), error, None)
                scope = None
            raise
        finally:
            if scope is not None:
                scope.__exit__(None, None, None)
        if not authored and gizmoPreview is not None:
            # Expected to publish through its notice and authored nothing:
            # put the authored rig back on screen.
            gizmoPreview.Republish(session=self._api)

    def _OnSelectionChanged(self, *args):
        model = getattr(self._api, "dataModel", None)
        if model is None:
            return
        paths = [str(p.GetPath()) for p in model.selection.getPrims()
                 if p and p.IsValid()]
        for view in self._views:
            view.set_selected(paths)

    # -- keys -----------------------------------------------------------

    def _FrameSelection(self):
        """Frame the selection in the viewport. True if it was asked for.

        Through usdview's OWN menu action rather than appController.
        _frameSelection(): the action is a named child of the main window
        and therefore public, it is the same entry point the Camera menu
        and the viewport's F both end at, and it carries its own enabled
        state -- so a picker press with nothing selected does nothing
        rather than framing an empty bound.
        """
        main = getattr(self._api, "qMainWindow", None)
        if main is None:
            return False
        actionType = getattr(QtGui, "QAction", None) or QtWidgets.QAction
        action = main.findChild(actionType, "actionFrame_Selected")
        if action is None or not action.isEnabled():
            return False
        action.trigger()
        return True

    def keyPressEvent(self, event):
        """F frames the selection, as it does over the viewport.

        It does NOT reach here on its own. usdview routes F through one
        application-wide AppEventFilter, and that filter hands the key to
        appController.processNavKeyEvent only for widgets in the main
        window; the picker is a QDialog -- its own top-level window -- so
        the key arrived and nothing framed. Selecting in the picker and
        pressing F is the same gesture as selecting in the viewport and
        pressing F, so it does the same thing.

        Bare F only. Any modifier is somebody else's shortcut, and a key
        we did not act on goes back to Qt rather than being swallowed --
        a QLineEdit inside the panel still receives its own F, because
        the focused widget sees the key before the dialog does.
        """
        if (event.key() == QtCore.Qt.Key_F
                and event.modifiers() == QtCore.Qt.NoModifier
                and self._FrameSelection()):
            event.accept()
            return
        super(PickerPanel, self).keyPressEvent(event)


class _NullContext(object):
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def OpenPickerPanel(usdviewApi, undoStack=None):
    panel = PickerPanel.GetInstance(usdviewApi, undoStack=undoStack)
    panel.Reload()
    panel.show()
    panel.raise_()
    return panel
