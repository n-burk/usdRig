#
# RigExec usdview plugin: the Avar Editor's viewport dock.
#
# The editor can live in two places and it is the SAME editor in both: a
# window of its own, as it always has, or a panel pinned to the right
# edge of the viewport that folds down to a title strip. Blender's
# sidebar is the shape being copied -- a panel inside the viewport rather
# than beside it, a keystroke away from gone -- and none of its code is,
# for the licence reason set out in avarWidgets.py.
#
# ONE PANEL, TWO HOMES. The dock does not build a second editor; it
# REPARENTS the existing AvarEditorPanel into itself. A QDialog is a
# QWidget, and clearing its window flags turns it into an ordinary child
# that lays out like any other. That matters for more than tidiness:
# a second instance would be a second set of notice listeners, a second
# refresh on every rig evaluation, and two panels that could disagree
# about what is selected. Undocking hands the same widget back its
# window flags and its place on the main window.
#
# The overlay technique is viewCubeUI's, deliberately: a plain child
# widget of usdview's stage view, placed by an event filter on the
# view's resize, with the install retried on a zero-length timer because
# plugins load before the stage view exists.
#
import os
import sys

from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import avarWidgets
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import avarWidgets


# How far the dock sits from the viewport's edges, and how wide it opens.
MARGIN = 8
# The editor's own width, PLUS the category strip that now sits beside
# it -- otherwise adding the strip would have quietly stolen 22px from
# the channel fields.
DOCK_WIDTH = 296 + 25
# The strip left behind when it folds. Wide enough for the chevron and
# the title turned on its side.
COLLAPSED_WIDTH = 22

# Retries for an install that arrives before usdview has a stage view.
_INSTALL_RETRIES = 40

_controller = None
_installPending = False


# Clear of usdview's orientation cube, which lives in the same corner.
TOP_INSET = 96


def StageView(usdviewApi):
    """usdview's stage view widget, or None headless."""
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


class VerticalTab(QtWidgets.QAbstractButton):
    """A section name written up the side of the folded dock.

    Blender's collapsed side panel: the strip still says what is in
    there, and clicking a name opens the panel at that group instead of
    opening it at whatever was last on screen. Painted rather than a
    styled QToolButton because the text has to run vertically, and a
    22 px strip has room for nothing else.
    """

    WIDTH = 22

    def __init__(self, title, parent=None):
        super(VerticalTab, self).__init__(parent)
        self.setText(title)
        self._active = False
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setToolTip("Open the editor at %s" % title)
        metrics = self.fontMetrics()
        self._length = metrics.horizontalAdvance(title) + 18
        self.setFixedWidth(self.WIDTH)
        self.setFixedHeight(self._length)

    def sizeHint(self):
        return QtCore.QSize(self.WIDTH, self._length)

    def SetActive(self, active):
        active = bool(active)
        if active != self._active:
            self._active = active
            self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        hovered = self.underMouse()
        if self._active:
            back, ink = QtGui.QColor(88, 96, 112), QtGui.QColor(255, 255, 255)
        elif hovered:
            back, ink = QtGui.QColor(70, 76, 88), QtGui.QColor(232, 236, 244)
        else:
            back, ink = QtGui.QColor(52, 56, 64), QtGui.QColor(196, 202, 212)
        painter.fillRect(self.rect(), back)
        painter.setPen(ink)
        painter.translate(self.width(), self.height())
        painter.rotate(-90.0)
        painter.drawText(QtCore.QRect(0, 0, self.height(), self.width()),
                         QtCore.Qt.AlignCenter, self.text())
        painter.end()


class AvarDock(QtWidgets.QFrame):
    """The frame on the viewport: a title bar, and the editor under it.

    Draws its own background because it sits over a GL widget: a
    transparent panel over a rendered character is unreadable whatever
    the palette says.
    """

    def __init__(self, controller, parent=None):
        super(AvarDock, self).__init__(parent)
        self._controller = controller
        self._expanded = True
        self._body = None

        self.setObjectName("avarDock")
        self.setStyleSheet(
            "#avarDock {"
            " background: rgba(38,40,45,232);"
            " border: 1px solid rgba(0,0,0,120);"
            " border-radius: 4px; }")

        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(3)

        bar = QtWidgets.QHBoxLayout()
        bar.setContentsMargins(0, 0, 0, 0)
        bar.setSpacing(2)

        self._fold = QtWidgets.QToolButton()
        self._fold.setCursor(QtCore.Qt.PointingHandCursor)
        self._fold.setFixedSize(16, 16)
        self._fold.setStyleSheet(
            "QToolButton { border: none; color: #cfd4dc; }"
            "QToolButton:hover { color: #ffffff; }")
        self._fold.clicked.connect(self.Toggle)
        bar.addWidget(self._fold)

        self._title = QtWidgets.QLabel("Avars")
        font = self._title.font()
        font.setBold(True)
        self._title.setFont(font)
        self._title.setStyleSheet("QLabel { color: #d8dce4; }")
        bar.addWidget(self._title, 1)

        self._pop = QtWidgets.QToolButton()
        self._pop.setText("↗")
        self._pop.setToolTip("Put the editor back in its own window")
        self._pop.setCursor(QtCore.Qt.PointingHandCursor)
        self._pop.setFixedSize(16, 16)
        self._pop.setStyleSheet(
            "QToolButton { border: none; color: #cfd4dc; }"
            "QToolButton:hover { color: #ffffff; }")
        self._pop.clicked.connect(self._controller.Undock)
        bar.addWidget(self._pop)
        outer.addLayout(bar)

        # The category strip and the editor sit SIDE BY SIDE, which is
        # what makes this read like Blender's sidebar: the tabs are a
        # permanent column you click to change category, not something
        # that only appears once the panel is folded away. Before, the
        # strip was stacked above the editor and hidden while expanded,
        # so an open panel showed nothing but the fold arrow.
        row = QtWidgets.QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(3)

        self._tabs = QtWidgets.QWidget()
        tabsLayout = QtWidgets.QVBoxLayout(self._tabs)
        tabsLayout.setContentsMargins(0, 2, 0, 2)
        tabsLayout.setSpacing(3)
        tabsLayout.addStretch(1)
        self._tabsLayout = tabsLayout
        row.addWidget(self._tabs, 0, QtCore.Qt.AlignTop)

        self._host = QtWidgets.QWidget()
        hostLayout = QtWidgets.QVBoxLayout(self._host)
        hostLayout.setContentsMargins(0, 0, 0, 0)
        hostLayout.setSpacing(0)
        self._hostLayout = hostLayout
        row.addWidget(self._host, 1)
        outer.addLayout(row, 1)

        # Which category the strip is showing as current.
        self._active = None

        self._SyncFold()

    # -- the editor it hosts ---------------------------------------------

    def Adopt(self, panel):
        """Take `panel` in as an ordinary child widget.

        Clearing the window flags is what turns a QDialog into something
        a layout can place. `setParent` alone leaves it a window and Qt
        draws it floating over the viewport with a title bar.
        """
        if panel is None or self._body is panel:
            return
        self._body = panel
        # Build the strip NOW. _SyncFold only fires when the fold state
        # CHANGES, so a dock that opens expanded -- the normal case --
        # never built its tabs and showed only the fold arrow.
        self._BuildTabs()
        panel.setParent(self._host)
        panel.setWindowFlags(QtCore.Qt.Widget)
        self._hostLayout.addWidget(panel)
        panel.show()

    def Release(self):
        """Give the editor back, still a live widget."""
        panel, self._body = self._body, None
        if panel is not None:
            self._hostLayout.removeWidget(panel)
            panel.setParent(None)
        return panel

    def Body(self):
        return self._body

    # -- folding ----------------------------------------------------------

    def IsExpanded(self):
        return self._expanded

    def SetExpanded(self, expanded):
        expanded = bool(expanded)
        if expanded == self._expanded:
            return
        self._expanded = expanded
        self._SyncFold()
        self._controller.Place()

    def Toggle(self):
        self.SetExpanded(not self._expanded)

    def _SyncFold(self):
        self._fold.setText("▸" if self._expanded else "◂")
        self._fold.setToolTip("Fold the editor away"
                              if self._expanded else "Open the editor")
        self._host.setVisible(self._expanded)
        self._title.setVisible(self._expanded)
        self._pop.setVisible(self._expanded)
        self._tabs.setVisible(True)
        self._BuildTabs()

    def _BuildTabs(self):
        """One tab per group the hosted editor is currently showing."""
        while self._tabsLayout.count():
            item = self._tabsLayout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
        panel = self._body
        names = []
        if panel is not None:
            getter = getattr(panel, "SectionTitles", None)
            if getter is not None:
                try:
                    names = list(getter())
                except Exception:
                    names = []
        if self._active not in names:
            self._active = names[0] if names else None
        for title in names:
            tab = VerticalTab(title, self._tabs)
            tab.SetActive(title == self._active)
            tab.clicked.connect(
                lambda checked=False, t=title: self._OpenAt(t))
            self._tabsLayout.addWidget(tab, 0, QtCore.Qt.AlignHCenter)
        self._tabsLayout.addStretch(1)

    def RefreshTabs(self):
        """Rebuild the category strip.

        The hosted editor calls this after it repopulates its sections,
        because which categories exist depends on the selected control:
        a control with no scale channels must not advertise a Scale tab.
        """
        self._BuildTabs()

    def _OpenAt(self, title):
        """Unfold, mark the tab current, and put that group in view."""
        self._active = title
        for i in range(self._tabsLayout.count()):
            w = self._tabsLayout.itemAt(i).widget()
            if isinstance(w, VerticalTab):
                w.SetActive(w.text() == title)
        self.SetExpanded(True)
        panel = self._body
        focus = getattr(panel, "FocusSection", None) if panel else None
        if focus is not None:
            try:
                focus(title)
            except Exception:
                pass


class AvarDockController(QtCore.QObject):
    """Installs the dock on the stage view and keeps it placed.

    Holds no rig state of its own: the editor it hosts is the one the
    menu opens, so docking and undocking cannot change what the panel is
    showing or lose an edit in flight.
    """

    def __init__(self, usdviewApi, undoStack=None, parent=None):
        super(AvarDockController, self).__init__(parent)
        self.usdviewApi = usdviewApi
        self._undo = undoStack
        self._view = StageView(usdviewApi)
        self._dock = None
        if self._view is None:
            return
        self._dock = AvarDock(self, self._view)
        self._view.installEventFilter(self)
        self._view.destroyed.connect(self._OnViewDestroyed)
        self._dock.hide()

    # -- placement ---------------------------------------------------------

    def Place(self):
        """Right edge, full height minus the margins.

        Starts below TOP_INSET, not at the margin: usdview draws its
        orientation cube in the viewport's top-right corner, which is
        the same corner this dock occupies, and the two overlapped --
        the cube sat on top of the panel's header so the control being
        edited could not be read.
        """
        dock, view = self._dock, self._view
        if dock is None or view is None:
            return
        width = (DOCK_WIDTH if dock.IsExpanded() else COLLAPSED_WIDTH)
        top = MARGIN + TOP_INSET
        height = (max(120, view.height() - top - MARGIN)
                  if dock.IsExpanded() else 28)
        dock.resize(width, height)
        dock.move(max(0, view.width() - width - MARGIN), top)
        dock.raise_()

    def eventFilter(self, obj, event):
        if obj is self._view and event.type() in (
                QtCore.QEvent.Resize, QtCore.QEvent.Show):
            self.Place()
            # Re-raised on a zero-length timer for the same reason the
            # view cube does it: the gizmo overlay raises itself
            # synchronously from its own resize handler and would
            # otherwise end up on top.
            QtCore.QTimer.singleShot(0, self.Place)
        return False

    def _OnViewDestroyed(self, *args):
        self._view = None
        self._dock = None

    # -- docking -----------------------------------------------------------

    def Dock(self, panel):
        """Bring `panel` into the viewport."""
        if self._dock is None or panel is None:
            return None
        self._dock.Adopt(panel)
        self._dock.show()
        self.Place()
        return self._dock

    def Undock(self):
        """Hand the editor back to its own window."""
        if self._dock is None:
            return None
        panel = self._dock.Release()
        self._dock.hide()
        if panel is not None:
            main = getattr(self.usdviewApi, "qMainWindow", None)
            panel.setParent(main)
            panel.setWindowFlags(QtCore.Qt.Dialog)
            panel.show()
            panel.raise_()
            panel.activateWindow()
        return panel

    def IsDocked(self):
        return self._dock is not None and self._dock.Body() is not None

    def Widget(self):
        return self._dock


def InstallAvarDock(usdviewApi, undoStack=None, retries=_INSTALL_RETRIES):
    """Put the dock on usdview's stage view once.

    Returns the controller, or None while there is no stage view yet (in
    which case an install is queued), exactly as InstallViewCube does --
    plugins are constructed before the view exists.
    """
    global _controller, _installPending
    if _controller is not None:
        return _controller
    if StageView(usdviewApi) is None:
        if retries > 0 and not _installPending:
            _installPending = True

            def _Retry():
                global _installPending
                _installPending = False
                InstallAvarDock(usdviewApi, undoStack, retries - 1)

            QtCore.QTimer.singleShot(0, _Retry)
        return None
    _controller = AvarDockController(usdviewApi, undoStack)
    return _controller


def GetController():
    """The installed controller, or None."""
    return _controller
