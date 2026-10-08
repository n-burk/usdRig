"""The hover picker: every picker tab drawn into the viewport, in 2D.

Each tab is a small round handle with its buttons hanging off it, painted
straight over the viewport with no window, frame or background around
them. Everything that is not a button, a handle or the opacity knob stays
the viewport's: a click there selects, box-selects, drags a gizmo or moves
the camera exactly as it would without the picker.

Gestures:

  * handle, drag          -- move the tab; its buttons follow
  * handle, Shift+drag    -- scale the tab about its handle
  * handle, double-click  -- collapse the tab to its handle, or expand it
  * button, click         -- select, with the docked picker's modifiers
                             (Shift toggles, Ctrl removes, Alt mirrors)
  * button, drag          -- marquee-select within that tab
  * button or handle, middle-drag left/right -- the tab's opacity
  * knob (bottom right), drag left/right -- every tab's opacity at once;
                             double-click puts it back to full
  * P                     -- show or hide the hover picker

Hovering a button lights it as the docked picker does, and hovering a
handle names its tab.

Drawn the way the gizmo is (gizmoUI.GizmoOverlay): a mouse-transparent
child of the stage view, with the stage view's own event filter deciding
what is ours. Painting into the stage view's paintGL instead would make
every hover highlight re-render the scene.

Only buttons are drawn: the docked picker's backdrops (silhouettes and
panels of colour) stay in the docked picker, since over the viewport they
would only hide the scene.

The buttons, their state and what a click does belong to the docked
picker (pickerUI.PickerPanel), hidden or not: this module only places,
paints and routes, so the two pickers cannot disagree. The layout is a
per-user viewer preference (pickerHoverModel), never written to the stage.
"""
import math
import os
import sys

from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

if __name__ != "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pickerHoverModel as hoverModel
import pickerUI
import sessionRegistry

try:
    from pxr import Tf
except ImportError:                                   # pragma: no cover
    Tf = None

# usdview's camera-navigation claim (stageView.py): Alt or Meta on a press
# arms its own tumble/truck/zoom, so a press carrying either is never ours.
_CAMERA_MODIFIERS = QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier
# A press that travels further than this is a drag, not a click.
_DRAG_SLOP = 3.0

_SETTINGS_ORG = "RigExec"
_SETTINGS_APP = "HoverPicker"
_SETTINGS_KEY = "layout"

_HANDLE_FILL = QtGui.QColor(225, 225, 225)
_HANDLE_COLLAPSED_FILL = QtGui.QColor(70, 70, 70)
_HANDLE_HOVER_FILL = QtGui.QColor(169, 211, 255)
_HANDLE_EDGE = QtGui.QColor(20, 20, 20)
_LABEL_COLOR = QtGui.QColor(235, 235, 235)
_LABEL_SHADOW = QtGui.QColor(0, 0, 0, 170)


def StageView(usdviewApi):
    # usdview keeps its app controller name-mangled on the api object.
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


def _EventPoint(event):
    try:
        point = event.position()
    except AttributeError:                            # PySide2
        point = event.pos()
    return (point.x(), point.y())


def LoadLayout():
    settings = QtCore.QSettings(_SETTINGS_ORG, _SETTINGS_APP)
    return hoverModel.Layout.FromJson(settings.value(_SETTINGS_KEY, ""))


def SaveLayout(layout):
    settings = QtCore.QSettings(_SETTINGS_ORG, _SETTINGS_APP)
    settings.setValue(_SETTINGS_KEY, layout.ToJson())


class _Tab(object):
    """One picker panel as the hover picker shows it."""

    def __init__(self, picker, panel):
        self.picker = picker
        self.panel = panel
        self.key = hoverModel.TabKey(getattr(picker, "name", ""), panel.id)
        self.label = panel.label or panel.id
        # The group's corner is the box around the buttons it can show,
        # in either half of every IK/FK switch: buttons this rig lacks and
        # backdrops are never drawn here, so they must not push the
        # buttons away from the handle.
        shown = Shown(picker, panel.id)
        if shown:
            self.ox = min(b.x for b in shown)
            self.oy = min(b.y for b in shown)
            self.cw = max(b.x + b.w for b in shown) - self.ox
            self.ch = max(b.y + b.h for b in shown) - self.oy
        else:
            self.ox = self.oy = 0.0
            self.cw = self.ch = 1.0


def Shown(picker, panelId, modes=None):
    """The buttons the hover picker draws for a tab: the live ones the
    docked picker would show, without its backdrops. Over the viewport a
    backdrop would only hide the scene."""
    return [b for b in picker.visible(panelId, modes=modes)
            if not b.decoration]


class HoverPickerOverlay(QtWidgets.QWidget):
    """The paint surface: transparent to the mouse and to the eye except
    where the controller draws."""

    def __init__(self, controller, parent):
        super(HoverPickerOverlay, self).__init__(parent)
        self._controller = controller
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        # WA_NoSystemBackground plus no auto-fill leaves the stage view
        # showing through; WA_TranslucentBackground does nothing on a
        # child.
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        self.setAutoFillBackground(False)
        self.setFocusPolicy(QtCore.Qt.NoFocus)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        try:
            self._controller.Paint(painter, self.width(), self.height())
        finally:
            painter.end()


class HoverPickerController(QtCore.QObject):
    """Places, paints and routes the hover picker for one usdview."""

    def __init__(self, usdviewApi, undoStack=None, parent=None):
        super(HoverPickerController, self).__init__(parent)
        self._api = usdviewApi
        self._view = StageView(usdviewApi)
        self._layout = LoadLayout()
        self._tabs = []
        self._selected = set()
        # (tab key, button id) under the cursor, (tab key, None) for a
        # handle, ("knob", None) for the knob, or None.
        self._hover = None
        self._gesture = None
        self._cursorSet = False
        self._font = pickerUI._load_font()
        self._panel = pickerUI.PickerPanel.GetInstance(usdviewApi,
                                                       undoStack)
        self._panel.AddListener(self._OnPanel)
        self._overlay = None
        if self._view is None:
            return
        self._overlay = HoverPickerOverlay(self, self._view)
        self._overlay.resize(self._view.size())
        # Hover arrives as HoverMove (WA_Hover), as the gizmo's does: the
        # stage view tracks no button-less moves, and turning tracking on
        # would make it run a GPU pick on every one.
        self._view.setAttribute(QtCore.Qt.WA_Hover, True)
        self._view.installEventFilter(self)
        selection = getattr(getattr(usdviewApi, "dataModel", None),
                            "selection", None)
        signal = getattr(selection, "signalPrimSelectionChanged", None)
        if signal is not None:
            signal.connect(self._OnSelectionChanged)
        self._Rebuild()
        self._OnSelectionChanged()
        self._overlay.setVisible(self._layout.enabled)
        if self._layout.enabled:
            QtCore.QTimer.singleShot(0, self._Raise)

    # -- state ------------------------------------------------------------

    def Layout(self):
        return self._layout

    def Tabs(self):
        return list(self._tabs)

    def IsVisible(self):
        return self._overlay is not None and self._overlay.isVisible()

    def SetVisible(self, visible):
        self._layout.enabled = bool(visible)
        SaveLayout(self._layout)
        if self._overlay is None:
            return
        if visible:
            # The panel may have been built for an earlier stage.
            if not self._tabs:
                self._panel.Reload()
            self._overlay.resize(self._view.size())
        self._overlay.setVisible(bool(visible))
        if visible:
            QtCore.QTimer.singleShot(0, self._Raise)
        else:
            self._SetHover(None)
            self._gesture = None

    def _Raise(self):
        # Above the gizmo overlay, which raises itself on resize.
        if self._overlay is not None:
            self._overlay.raise_()

    def _Update(self):
        if self._overlay is not None and self._overlay.isVisible():
            self._overlay.update()

    def _Rebuild(self):
        tabs = []
        for picker in self._panel.Pickers():
            for panel in picker.panels:
                live = any(b.parent == panel.id and b.live
                           and not b.decoration for b in picker.buttons)
                if live:
                    tabs.append(_Tab(picker, panel))
        self._tabs = tabs
        height = float(self._view.height()) if self._view else 0.0
        fits = {t.key: hoverModel.FitScale(t.ch, height) for t in tabs}
        before = len(self._layout.tabs)
        self._layout.Ensure([t.key for t in tabs], fits)
        if len(self._layout.tabs) != before:
            SaveLayout(self._layout)
        self._Update()

    def _OnPanel(self, what):
        if what == "reload":
            self._Rebuild()
        else:
            self._Update()

    def _OnSelectionChanged(self, *args):
        model = getattr(self._api, "dataModel", None)
        if model is None:
            return
        self._selected = set(str(p.GetPath())
                             for p in model.selection.getPrims()
                             if p and p.IsValid())
        self._Update()

    # -- hit testing ------------------------------------------------------

    def Hit(self, point):
        """What is under a viewport point: ("knob", None, None),
        ("handle", tab, None), ("button", tab, button), or None.
        Topmost first: the knob, then the tabs in reverse paint order."""
        if self._view is None:
            return None
        if hoverModel.HitsKnob(self._view.width(), self._view.height(),
                               point):
            return ("knob", None, None)
        modes = self._panel.Modes()
        for tab in reversed(self._tabs):
            layout = self._layout.Tab(tab.key)
            if layout is None:
                continue
            if layout.HitsHandle(point):
                return ("handle", tab, None)
            if layout.collapsed:
                continue
            x, y = layout.ToPanel((tab.ox, tab.oy), point)
            live = [b for b in tab.picker.hits(tab.panel.id, x, y, modes)
                    if b.live]
            if live:
                return ("button", tab, live[0])
        return None

    # -- painting ---------------------------------------------------------

    def Paint(self, painter, width, height):
        modes = self._panel.Modes()
        gesture = self._gesture or {}
        for tab in self._tabs:
            layout = self._layout.Tab(tab.key)
            if layout is None:
                continue
            painter.save()
            painter.setOpacity(self._layout.TabOpacity(tab.key))
            if not layout.collapsed:
                painter.save()
                cx, cy = layout.ButtonsCorner()
                painter.translate(cx, cy)
                painter.scale(layout.scale, layout.scale)
                painter.translate(-tab.ox, -tab.oy)
                hover = (self._hover[1] if self._hover
                         and self._hover[0] == tab.key else None)
                for button in Shown(tab.picker, tab.panel.id, modes):
                    pickerUI.DrawButton(painter, button, self._font, hover,
                                        self._selected)
                painter.restore()
            handleHovered = self._hover == (tab.key, None)
            active = gesture.get("tab") is tab
            self._PaintHandle(painter, layout, handleHovered or active)
            if handleHovered or active:
                self._PaintLabel(painter, layout, tab, gesture
                                 if active else None)
            painter.restore()
        band = gesture.get("band")
        if band is not None:
            x0, y0, x1, y1 = band
            painter.setBrush(QtGui.QBrush(QtGui.QColor(120, 200, 255, 40)))
            pen = QtGui.QPen(QtGui.QColor(150, 215, 255))
            pen.setWidthF(1.0)
            painter.setPen(pen)
            painter.drawRect(QtCore.QRectF(min(x0, x1), min(y0, y1),
                                           abs(x1 - x0), abs(y1 - y0)))
        self._PaintKnob(painter, width, height,
                        self._hover == ("knob", None)
                        or gesture.get("kind") == "knob")

    def _PaintHandle(self, painter, layout, hovered):
        r = hoverModel.HANDLE_RADIUS
        fill = (_HANDLE_HOVER_FILL if hovered else
                _HANDLE_COLLAPSED_FILL if layout.collapsed else
                _HANDLE_FILL)
        painter.setBrush(QtGui.QBrush(fill))
        pen = QtGui.QPen(_HANDLE_EDGE)
        pen.setWidthF(1.5)
        painter.setPen(pen)
        painter.drawEllipse(QtCore.QPointF(layout.x, layout.y), r, r)
        if layout.collapsed:
            # A collapsed tab reads as a ring with a dot in it.
            painter.setBrush(QtGui.QBrush(_HANDLE_FILL))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPointF(layout.x, layout.y),
                                r * 0.35, r * 0.35)

    def _PaintLabel(self, painter, layout, tab, gesture):
        text = tab.label
        if len(self._panel.Pickers()) > 1:
            text = "%s %s" % (getattr(tab.picker, "name", ""), text)
        kind = (gesture or {}).get("kind")
        if kind == "scale":
            text += "  %d%%" % round(layout.scale * 100.0)
        elif kind == "fade":
            text += "  opacity %d%%" % round(layout.opacity * 100.0)
        self._PaintText(painter, layout.x + hoverModel.HANDLE_RADIUS + 6.0,
                        layout.y, text)

    def _PaintText(self, painter, x, y, text, alignRight=False):
        font = QtGui.QFont(self._font or "Sans")
        font.setPixelSize(11)
        painter.setFont(font)
        metrics = QtGui.QFontMetricsF(font)
        width = metrics.horizontalAdvance(text)
        left = x - width if alignRight else x
        base = y + metrics.ascent() * 0.5 - 1.0
        painter.setPen(_LABEL_SHADOW)
        painter.drawText(QtCore.QPointF(left + 1.0, base + 1.0), text)
        painter.setPen(_LABEL_COLOR)
        painter.drawText(QtCore.QPointF(left, base), text)

    def _PaintKnob(self, painter, width, height, hovered):
        cx, cy = hoverModel.KnobCentre(width, height)
        r = hoverModel.KNOB_RADIUS
        centre = QtCore.QPointF(cx, cy)
        painter.setOpacity(1.0)
        painter.setBrush(QtGui.QBrush(QtGui.QColor(40, 40, 40, 200)))
        pen = QtGui.QPen(_HANDLE_HOVER_FILL if hovered else _HANDLE_FILL)
        pen.setWidthF(1.5)
        painter.setPen(pen)
        painter.drawEllipse(centre, r, r)
        # The overall opacity as a filled wedge, full at 100%.
        painter.setBrush(QtGui.QBrush(_HANDLE_HOVER_FILL if hovered
                                      else _HANDLE_FILL))
        painter.setPen(QtCore.Qt.NoPen)
        box = QtCore.QRectF(cx - r + 2.0, cy - r + 2.0,
                            2.0 * r - 4.0, 2.0 * r - 4.0)
        span = int(round(-360.0 * 16.0 * self._layout.opacity))
        painter.drawPie(box, 90 * 16, span)
        if hovered:
            self._PaintText(painter, cx - r - 6.0, cy,
                            "picker opacity %d%%"
                            % round(self._layout.opacity * 100.0),
                            alignRight=True)

    # -- the mouse --------------------------------------------------------

    def eventFilter(self, watched, event):
        if watched is not self._view or self._overlay is None:
            return False
        kind = event.type()
        if kind in (QtCore.QEvent.Resize, QtCore.QEvent.Show):
            self._overlay.resize(self._view.size())
            QtCore.QTimer.singleShot(0, self._Raise)
            return False
        if not self._overlay.isVisible():
            return False
        if kind in (QtCore.QEvent.MouseButtonPress,
                    QtCore.QEvent.MouseButtonDblClick):
            return self._Press(event, kind == QtCore.QEvent.MouseButtonDblClick)
        if kind == QtCore.QEvent.MouseMove:
            return self._Move(event)
        if kind == QtCore.QEvent.MouseButtonRelease:
            return self._Release(event)
        if kind == QtCore.QEvent.HoverMove:
            if self._gesture is not None:
                return True
            # Claimed over a button or handle, so the gizmo does not also
            # pre-highlight a handle hidden beneath it.
            return self._HoverAt(_EventPoint(event))
        if kind in (QtCore.QEvent.Leave, QtCore.QEvent.HoverLeave):
            self._SetHover(None)
        return False

    def _HoverAt(self, point):
        hit = self.Hit(point)
        if hit is None:
            self._SetHover(None)
            return False
        what, tab, button = hit
        self._SetHover(("knob", None) if what == "knob" else
                       (tab.key, None) if what == "handle" else
                       (tab.key, button.id))
        return True

    def _Press(self, event, double):
        if self._gesture is not None:
            return True
        if event.modifiers() & _CAMERA_MODIFIERS and \
                event.button() != QtCore.Qt.LeftButton:
            return False
        point = _EventPoint(event)
        hit = self.Hit(point)
        if hit is None:
            return False
        what, tab, button = hit
        mouse = event.button()
        # Alt is the camera's everywhere except on a picker button, where
        # it is the docked picker's mirror modifier.
        if event.modifiers() & _CAMERA_MODIFIERS and what != "button":
            return False
        if mouse == QtCore.Qt.LeftButton and double:
            if what == "handle":
                layout = self._layout.Tab(tab.key)
                layout.collapsed = not layout.collapsed
                SaveLayout(self._layout)
                self._Update()
                return True
            if what == "knob":
                self._layout.opacity = 1.0
                SaveLayout(self._layout)
                self._Update()
                return True
        if mouse == QtCore.Qt.LeftButton:
            if what == "knob":
                self._gesture = {"kind": "knob", "start": point,
                                 "opacity": self._layout.opacity}
            elif what == "handle":
                layout = self._layout.Tab(tab.key)
                if event.modifiers() & QtCore.Qt.ShiftModifier:
                    self._gesture = {"kind": "scale", "tab": tab,
                                     "start": point,
                                     "scale": layout.scale}
                else:
                    self._gesture = {"kind": "move", "tab": tab,
                                     "last": point}
            else:
                layout = self._layout.Tab(tab.key)
                self._gesture = {
                    "kind": "pick", "tab": tab, "start": point,
                    "panelStart": layout.ToPanel((tab.ox, tab.oy), point),
                    "mode": pickerUI.PickerView.ModeFor(event.modifiers()),
                    "mirror": pickerUI.PickerView.WantsMirror(
                        event.modifiers()),
                    "band": None}
            self._Update()
            return True
        if mouse == QtCore.Qt.MiddleButton:
            if what == "knob":
                self._gesture = {"kind": "knob", "start": point,
                                 "opacity": self._layout.opacity,
                                 "button": QtCore.Qt.MiddleButton}
            else:
                layout = self._layout.Tab(tab.key)
                self._gesture = {"kind": "fade", "tab": tab,
                                 "start": point,
                                 "opacity": layout.opacity,
                                 "button": QtCore.Qt.MiddleButton}
            self._Update()
            return True
        # Right-click and anything else stay the viewport's.
        return False

    def _Move(self, event):
        point = _EventPoint(event)
        gesture = self._gesture
        if gesture is None:
            if event.buttons() != QtCore.Qt.NoButton:
                return False
            # Ours over a button: the stage view would otherwise run a GPU
            # pick for the hover on every move.
            return self._HoverAt(point)
        kind = gesture["kind"]
        if kind == "knob":
            self._layout.Faded(gesture["opacity"],
                               point[0] - gesture["start"][0])
        elif kind == "move":
            last = gesture["last"]
            self._layout.Tab(gesture["tab"].key).Moved(point[0] - last[0],
                                                       point[1] - last[1])
            gesture["last"] = point
        elif kind == "scale":
            start = gesture["start"]
            self._layout.Tab(gesture["tab"].key).Scaled(
                gesture["scale"], point[0] - start[0], point[1] - start[1])
        elif kind == "fade":
            self._layout.Tab(gesture["tab"].key).Faded(
                gesture["opacity"], point[0] - gesture["start"][0])
        elif kind == "pick":
            start = gesture["start"]
            if gesture["band"] is not None or math.hypot(
                    point[0] - start[0], point[1] - start[1]) > _DRAG_SLOP:
                gesture["band"] = (start[0], start[1], point[0], point[1])
        self._Update()
        return True

    def _Release(self, event):
        gesture = self._gesture
        if gesture is None:
            return False
        wanted = gesture.get("button", QtCore.Qt.LeftButton)
        if event.button() != wanted:
            return True
        self._gesture = None
        kind = gesture["kind"]
        if kind == "pick":
            self._FinishPick(gesture)
        else:
            SaveLayout(self._layout)
        self._Update()
        return True

    def _FinishPick(self, gesture):
        tab = gesture["tab"]
        layout = self._layout.Tab(tab.key)
        modes = self._panel.Modes()
        band = gesture["band"]
        if band is not None:
            x0, y0 = layout.ToPanel((tab.ox, tab.oy), band[:2])
            x1, y1 = layout.ToPanel((tab.ox, tab.oy), band[2:])
            buttons = tab.picker.within(tab.panel.id, x0, y0, x1, y1, modes)
            if not buttons:
                return
        else:
            x, y = gesture["panelStart"]
            buttons = [b for b in tab.picker.hits(tab.panel.id, x, y, modes)
                       if b.live][:1]
        if gesture["mirror"]:
            buttons = self._WithMirrors(tab, buttons)
        self._panel.Pick(buttons, gesture["mode"])

    def _WithMirrors(self, tab, buttons):
        """The buttons plus their mirrored partners, as the docked picker
        brings them along with Alt."""
        byName = {b.id: b for b in tab.picker.visible(tab.panel.id)}
        out, seen = [], set()
        for b in buttons:
            partner = byName.get((b.mirror or "").rsplit("/", 1)[-1])
            for candidate in (b, partner):
                if candidate is not None and id(candidate) not in seen:
                    seen.add(id(candidate))
                    out.append(candidate)
        return out

    def _SetHover(self, hover):
        if hover == self._hover:
            return
        self._hover = hover
        if self._view is not None:
            if hover is not None and not self._cursorSet:
                self._view.setCursor(QtCore.Qt.PointingHandCursor)
                self._cursorSet = True
            elif hover is None and self._cursorSet:
                self._view.unsetCursor()
                self._cursorSet = False
        self._Update()


# Text inputs the P key must reach untouched.
_TEXT_WIDGETS = (QtWidgets.QLineEdit, QtWidgets.QTextEdit,
                 QtWidgets.QPlainTextEdit, QtWidgets.QAbstractSpinBox)


class HoverPickerHotkey(QtCore.QObject):
    """P shows or hides the hover picker, from anywhere in the session's
    main window that is not a text field.

    An APPLICATION filter, like the gizmo's ViewportHotkeyFilter, because
    usdview never lets the stage view hold focus: a key pressed over the
    viewport is delivered to the main window. Installed after the gizmo's,
    so it sees P first.
    """

    def __init__(self, usdviewApi, undoStack=None, parent=None):
        super(HoverPickerHotkey, self).__init__(parent)
        self._api = usdviewApi
        self._undo = undoStack
        # One physical press reaches an application filter several times
        # (a ShortcutOverride and a KeyPress, to each ancestor in turn):
        # act on the first and swallow the rest until the release.
        self._latched = False

    def _MainWindow(self):
        return getattr(self._api, "qMainWindow", None)

    def eventFilter(self, obj, event):
        try:
            kind = event.type()
            if kind not in (QtCore.QEvent.KeyPress,
                            QtCore.QEvent.KeyRelease,
                            QtCore.QEvent.ShortcutOverride):
                return False
            if event.key() != QtCore.Qt.Key_P:
                return False
            if not sessionRegistry.SessionOfEvent(obj, self._MainWindow()):
                return False
            if kind == QtCore.QEvent.KeyRelease:
                if event.isAutoRepeat():
                    return self._latched
                was, self._latched = self._latched, False
                return was
            if self._latched:
                event.accept()
                return True
            if event.modifiers() & (QtCore.Qt.ShiftModifier
                                    | QtCore.Qt.ControlModifier
                                    | QtCore.Qt.AltModifier
                                    | QtCore.Qt.MetaModifier):
                return False
            if event.isAutoRepeat():
                return False
            focus = QtWidgets.QApplication.focusWidget()
            if isinstance(focus, _TEXT_WIDGETS) or (
                    isinstance(focus, QtWidgets.QComboBox)
                    and focus.isEditable()):
                return False
            self._latched = True
            ToggleHoverPicker(self._api, self._undo)
            event.accept()
            return True
        except Exception as error:
            # An exception escaping an application-wide filter would break
            # every key in usdview, not just this one.
            if Tf is not None:
                Tf.Warn("rigExecUsdview: hover picker hotkey failed: %s"
                        % error)
            return False


_hotkeys = sessionRegistry.SessionRegistry("hover picker hotkeys")


def InstallHotkey(usdviewApi, undoStack=None):
    """The session's P hotkey, installed once."""
    hotkey = _hotkeys.Get(usdviewApi)
    if hotkey is None:
        application = QtWidgets.QApplication.instance()
        if application is None:
            return None
        hotkey = _hotkeys.Set(
            usdviewApi, HoverPickerHotkey(usdviewApi, undoStack,
                                          parent=application))
        application.installEventFilter(hotkey)
    return hotkey


_controllers = sessionRegistry.SessionRegistry("hover pickers")


def GetController(usdviewApi):
    return _controllers.Get(usdviewApi)


def InstallHoverPicker(usdviewApi, undoStack=None):
    """The session's hover picker, built on first use."""
    controller = _controllers.Get(usdviewApi)
    if controller is None:
        controller = _controllers.Set(
            usdviewApi, HoverPickerController(usdviewApi, undoStack))
    return controller


def ToggleHoverPicker(usdviewApi, undoStack=None):
    controller = InstallHoverPicker(usdviewApi, undoStack)
    controller.SetVisible(not controller.IsVisible())
    return controller


def WasEnabled():
    """Whether hover mode was on when the user last left it."""
    return LoadLayout().enabled
