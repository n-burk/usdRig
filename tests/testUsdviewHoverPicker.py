#
# THE HOVER PICKER, end to end, in a real usdview.
#
# The picker's tabs drawn straight into the viewport: a handle per tab
# with the tab's buttons hanging off it. Real mouse events go to the
# stage view and this checks what they do:
#
#   * RigExec -> Animation Editors -> Hover Picker turns it on, with the
#     handles side by side along the top;
#   * a click on a button selects its control in usdview's own selection,
#     Shift adds, and a click anywhere else is left to the viewport;
#   * dragging a handle moves its buttons with it, Shift-dragging scales
#     them about it, double-clicking collapses and expands the tab;
#   * middle-dragging a button fades that tab only, and the corner knob
#     fades every tab;
#   * hovering a button lights it and hovering a handle names its tab;
#   * P shows and hides it;
#   * the layout is saved per user and comes back as it was left.
#
# The user's own saved layout is put back afterwards.
#
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RigExecMenu(appController):
    for child in appController._mainWindow.menuBar().children():
        if isinstance(child, QtWidgets.QMenu) and \
                str(child.title()).replace("&", "") == "RigExec":
            return child
    return None


def _Submenu(menu, title):
    for action in menu.actions():
        if action.menu() is not None and action.text() == title:
            return action.menu()
    return None


def _Event(view, kind, point, button=None, buttons=None, modifiers=None):
    pos = QtCore.QPointF(float(point[0]), float(point[1]))
    glob = view.mapToGlobal(QtCore.QPoint(int(point[0]), int(point[1])))
    if button is None:
        button = QtCore.Qt.LeftButton
    if buttons is None:
        buttons = button
    if modifiers is None:
        modifiers = QtCore.Qt.NoModifier
    return QtGui.QMouseEvent(kind, pos,
                             QtCore.QPointF(float(glob.x()),
                                            float(glob.y())),
                             button, buttons, modifiers)


class _Mouse(object):
    def __init__(self, appController, view):
        self._app = appController
        self._view = view

    def Send(self, kind, point, **kwargs):
        event = _Event(self._view, kind, point, **kwargs)
        QtWidgets.QApplication.sendEvent(self._view, event)
        self._app._processEvents()
        return event

    def Click(self, point, modifiers=None):
        self.Send(QtCore.QEvent.MouseButtonPress, point,
                  modifiers=modifiers)
        self.Send(QtCore.QEvent.MouseButtonRelease, point,
                  buttons=QtCore.Qt.NoButton, modifiers=modifiers)

    def Hover(self, point):
        pos = QtCore.QPointF(float(point[0]), float(point[1]))
        glob = self._view.mapToGlobal(QtCore.QPoint(int(point[0]),
                                                    int(point[1])))
        try:
            event = QtGui.QHoverEvent(QtCore.QEvent.HoverMove, pos,
                                      QtCore.QPointF(glob), pos)
        except TypeError:                             # PySide2
            event = QtGui.QHoverEvent(QtCore.QEvent.HoverMove, pos, pos)
        QtWidgets.QApplication.sendEvent(self._view, event)
        self._app._processEvents()

    def Key(self, key, modifiers=None):
        try:
            from PySide6 import QtTest
        except ImportError:
            from PySide2 import QtTest
        self._view.setFocus()
        QtTest.QTest.keyClick(self._view, key,
                              modifiers or QtCore.Qt.NoModifier)
        self._app._processEvents()

    def DoubleClick(self, point):
        self.Click(point)
        self.Send(QtCore.QEvent.MouseButtonDblClick, point)
        self.Send(QtCore.QEvent.MouseButtonRelease, point,
                  buttons=QtCore.Qt.NoButton)

    def Drag(self, start, end, button=None, modifiers=None):
        button = button or QtCore.Qt.LeftButton
        self.Send(QtCore.QEvent.MouseButtonPress, start, button=button,
                  modifiers=modifiers)
        for i in (1, 2, 3, 4):
            t = i / 4.0
            self.Send(QtCore.QEvent.MouseMove,
                      (start[0] + (end[0] - start[0]) * t,
                       start[1] + (end[1] - start[1]) * t),
                      button=QtCore.Qt.NoButton, buttons=button,
                      modifiers=modifiers)
        self.Send(QtCore.QEvent.MouseButtonRelease, end, button=button,
                  buttons=QtCore.Qt.NoButton, modifiers=modifiers)


def _Centre(controller, tab, button):
    layout = controller.Layout().Tab(tab.key)
    return layout.ToScreen((tab.ox, tab.oy),
                           (button.x + button.w * 0.5,
                            button.y + button.h * 0.5))


def _SelectableButtons(controller, tab):
    """Live buttons that select (no switches or commands), each with the
    screen point that reaches it."""
    import pickerHoverUI
    modes = controller._panel.Modes()
    out = []
    for button in pickerHoverUI.Shown(tab.picker, tab.panel.id, modes):
        if not button.live or not button.targets or button.attr_target \
                or button.command:
            continue
        point = _Centre(controller, tab, button)
        hit = controller.Hit(point)
        if hit is not None and hit[0] == "button" and hit[2] is button:
            out.append((button, point))
    return out


def _Selected(api):
    return set(str(p.GetPath())
               for p in api.dataModel.selection.getPrims())


def testUsdviewInputFunction(appController):
    import pickerHoverModel as hm
    import pickerHoverUI

    appController._processEvents()
    api = appController._usdviewApi
    settings = QtCore.QSettings(pickerHoverUI._SETTINGS_ORG,
                                pickerHoverUI._SETTINGS_APP)
    saved = settings.value(pickerHoverUI._SETTINGS_KEY, None)
    try:
        _Run(appController, api, hm, pickerHoverUI)
    finally:
        if saved is None:
            settings.remove(pickerHoverUI._SETTINGS_KEY)
        else:
            settings.setValue(pickerHoverUI._SETTINGS_KEY, saved)


def _Run(appController, api, hm, pickerHoverUI):
    # --- 1. the menu turns it on, handles side by side ------------------
    rigMenu = _RigExecMenu(appController)
    _Check(rigMenu is not None, "there is a RigExec menu")
    editors = _Submenu(rigMenu, "Animation Editors")
    _Check(editors is not None and "Hover Picker" in
           [a.text() for a in editors.actions()],
           "RigExec -> Animation Editors carries Hover Picker")

    controller = pickerHoverUI.InstallHoverPicker(api)
    if controller.IsVisible():
        controller.SetVisible(False)
    # A fresh layout, whatever this user had saved.
    controller._layout = hm.Layout()
    controller._Rebuild()
    for action in editors.actions():
        if action.text() == "Hover Picker":
            action.trigger()
    appController._processEvents()
    _Check(controller.IsVisible(), "the menu item turned it on")
    view = pickerHoverUI.StageView(api)
    shot = __import__("os").environ.get("RIGEXEC_HOVER_SHOT")
    if shot:
        appController._mainWindow.grab().save(
            shot.replace(".png", "_fresh.png"))
    mouse = _Mouse(appController, view)

    tabs = controller.Tabs()
    _Check(len(tabs) >= 2, "one group per picker tab: %d" % len(tabs))
    layouts = [controller.Layout().Tab(t.key) for t in tabs]
    _Check(len(set(round(l.y, 6) for l in layouts)) == 1,
           "the handles start in one row")
    xs = [l.x for l in layouts]
    _Check(xs == sorted(xs) and len(set(xs)) == len(xs),
           "side by side: %r" % xs)
    _Check(not layouts[0].collapsed and
           all(l.collapsed for l in layouts[1:]),
           "only the first tab opens, so the groups do not overlap")
    first = tabs[0]
    firstLayout = layouts[0]

    # --- 2. a click selects, Shift adds, empty space is the viewport's --
    buttons = _SelectableButtons(controller, first)
    _Check(len(buttons) >= 2, "the first tab has buttons to click: %d"
           % len(buttons))
    (a, aPoint), (b, bPoint) = buttons[0], buttons[-1]
    api.dataModel.selection.clearPrims()
    mouse.Click(aPoint)
    _Check(_Selected(api) == set(a.targets),
           "a click selected %s: %s" % (a.targets, sorted(_Selected(api))))
    mouse.Click(bPoint, modifiers=QtCore.Qt.ShiftModifier)
    _Check(_Selected(api) == set(a.targets) | set(b.targets),
           "Shift added %s" % b.targets)

    empty = (view.width() * 0.5, view.height() * 0.5)
    while controller.Hit(empty) is not None:
        empty = (empty[0] + 37.0, empty[1] + 23.0)
    event = _Event(view, QtCore.QEvent.MouseButtonPress, empty)
    _Check(not controller.eventFilter(view, event),
           "a press on empty viewport is left to the viewport")
    press = _Event(view, QtCore.QEvent.MouseButtonPress, aPoint,
                   modifiers=QtCore.Qt.AltModifier,
                   button=QtCore.Qt.MiddleButton)
    _Check(not controller.eventFilter(view, press),
           "an Alt camera move over a button is left to the camera")

    # --- hover: buttons light up, handles name their tab -----------------
    mouse.Hover(aPoint)
    _Check(controller._hover == (first.key, a.id),
           "hovering a button lights it: %r" % (controller._hover,))
    mouse.Hover((firstLayout.x, firstLayout.y))
    _Check(controller._hover == (first.key, None),
           "hovering a handle names its tab: %r" % (controller._hover,))
    mouse.Hover(empty)
    _Check(controller._hover is None, "and nothing is lit over the scene")

    # --- 3. move, scale, collapse ----------------------------------------
    handle = (firstLayout.x, firstLayout.y)
    mouse.Drag(handle, (handle[0] + 40.0, handle[1] + 30.0))
    _Check(abs(firstLayout.x - handle[0] - 40.0) < 1e-6 and
           abs(firstLayout.y - handle[1] - 30.0) < 1e-6,
           "dragging the handle moved it: %r"
           % ((firstLayout.x, firstLayout.y),))
    moved = _Centre(controller, first, a)
    _Check(abs(moved[0] - aPoint[0] - 40.0) < 1e-6 and
           abs(moved[1] - aPoint[1] - 30.0) < 1e-6,
           "and its buttons followed")

    handle = (firstLayout.x, firstLayout.y)
    scale = firstLayout.scale
    mouse.Drag(handle, (handle[0] + 60.0, handle[1]),
               modifiers=QtCore.Qt.ShiftModifier)
    _Check(firstLayout.scale > scale * 1.2,
           "Shift-drag right scaled it up: %.3f -> %.3f"
           % (scale, firstLayout.scale))
    _Check((firstLayout.x, firstLayout.y) == handle,
           "about its handle, which stayed put")

    point = _Centre(controller, first, a)
    mouse.DoubleClick((firstLayout.x, firstLayout.y))
    _Check(firstLayout.collapsed, "a double-click collapsed it")
    _Check(controller.Hit(point) is None,
           "a collapsed tab's buttons are the viewport's again")
    mouse.DoubleClick((firstLayout.x, firstLayout.y))
    _Check(not firstLayout.collapsed, "and another expanded it")

    # --- 4. opacity: one tab by middle-drag, every tab by the knob -------
    point = _Centre(controller, first, a)
    mouse.Drag(point, (point[0] - 100.0, point[1]),
               button=QtCore.Qt.MiddleButton)
    _Check(abs(firstLayout.opacity - 0.5) < 1e-6,
           "middle-dragging a button faded its tab: %.3f"
           % firstLayout.opacity)
    _Check(all(abs(l.opacity - 1.0) < 1e-9 for l in layouts[1:]),
           "and only its tab")
    knob = hm.KnobCentre(view.width(), view.height())
    mouse.Drag(knob, (knob[0] - 100.0, knob[1]))
    _Check(abs(controller.Layout().opacity - 0.5) < 1e-6,
           "the knob faded everything: %.3f" % controller.Layout().opacity)
    _Check(abs(controller.Layout().TabOpacity(first.key) - 0.25) < 1e-6,
           "a tab draws at its own times the overall")

    if shot:
        appController._mainWindow.grab().save(shot)

    mouse.DoubleClick(knob)
    _Check(abs(controller.Layout().opacity - 1.0) < 1e-9,
           "double-clicking the knob put it back to full")

    # --- 5. saved per user, and off means off ----------------------------
    _Check(pickerHoverUI.LoadLayout().ToJson() ==
           controller.Layout().ToJson(),
           "the layout on disk is the layout on screen")
    mouse.Key(QtCore.Qt.Key_P)
    _Check(not controller.IsVisible(), "P hides it")
    mouse.Key(QtCore.Qt.Key_P)
    _Check(controller.IsVisible(), "and P shows it again")
    pickerHoverUI.ToggleHoverPicker(api)
    appController._processEvents()
    _Check(not controller.IsVisible(), "the menu item turns it off again")
    point = _Centre(controller, first, a)
    event = _Event(view, QtCore.QEvent.MouseButtonPress, point)
    _Check(not controller.eventFilter(view, event),
           "and a hidden picker takes no clicks")
    _Check(not pickerHoverUI.LoadLayout().enabled, "off is saved too")
    print("HOVER_PICKER_OK")
