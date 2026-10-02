#
# RigExec usdview plugin: the Layer Stack panel.
#
# The open stage's sublayer tree with a checkbox on every layer: untick
# one to mute it, and the stage recomposes without that layer and
# everything under it. A muted branch stays listed, greyed, so it can be
# ticked back on. Muting writes nothing to any file.
#
# The rules live in layerStackModel, which imports no Qt.
#
from pxr import Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

import layerStackModel as model


_NODE_ROLE = QtCore.Qt.UserRole + 1


class LayerStackPanel(QtWidgets.QDialog):
    """The panel window. One per usdview session."""

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi, muteLayers=None, undoStack=None):
        if cls._instance is None:
            cls._instance = LayerStackPanel(usdviewApi, muteLayers,
                                            undoStack=undoStack)
        elif undoStack is not None:
            cls._instance._undo = undoStack
        return cls._instance

    def __init__(self, usdviewApi, muteLayers=None, parent=None,
                 undoStack=None):
        super(LayerStackPanel, self).__init__(parent)
        self._api = usdviewApi
        # How a mute is applied: the RigExec container's MuteLayers, which
        # releases the rig's evaluation around it, or a plain call.
        self._muteLayers = muteLayers or (lambda change: change(
            self._api.stage))
        # The shared stack, so a mute undoes with the same key as a drag.
        self._undo = undoStack
        self._rebuilding = False
        self._rebuildQueued = False
        self._noticeKey = None
        self._noticeStage = None

        self.setWindowTitle("Layer Stack")
        self.resize(460, 520)

        layout = QtWidgets.QVBoxLayout(self)
        self._tree = QtWidgets.QTreeWidget()
        self._tree.setColumnCount(1)
        self._tree.setHeaderLabels(["Layer (untick to mute)"])
        self._tree.setRootIsDecorated(True)
        self._tree.setUniformRowHeights(True)
        self._tree.itemChanged.connect(self._OnItemChanged)
        layout.addWidget(self._tree, 1)

        self._status = QtWidgets.QLabel("")
        layout.addWidget(self._status)

        buttons = QtWidgets.QHBoxLayout()
        unmute = QtWidgets.QPushButton("Unmute All")
        unmute.clicked.connect(self._OnUnmuteAll)
        buttons.addWidget(unmute)
        buttons.addStretch(1)
        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self.Rebuild)
        buttons.addWidget(refresh)
        layout.addLayout(buttons)

        self._api.dataModel.signalStageReplaced.connect(self.Rebuild)
        self.Rebuild()

    # -- wiring ------------------------------------------------------

    def _Listen(self, stage):
        if stage is self._noticeStage:
            return
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        self._noticeStage = stage
        if stage is not None:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.LayerMutingChanged, self._OnMutingChanged, stage)

    def _OnMutingChanged(self, notice, sender):
        self._QueueRebuild()

    def _QueueRebuild(self):
        """Rebuild once the event loop turns, never inline.

        A mute starts from a checkbox, inside QTreeWidgetItem::setData.
        Clearing the tree there deletes the item Qt is still writing to,
        which crashes a few toggles in. One queued rebuild also folds the
        muting notice and the click into a single refresh.
        """
        if self._rebuildQueued:
            return
        self._rebuildQueued = True
        QtCore.QTimer.singleShot(0, self._QueuedRebuild)

    def _QueuedRebuild(self):
        self._rebuildQueued = False
        if LayerStackPanel._instance is self:
            self.Rebuild()

    def closeEvent(self, event):
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        self._noticeStage = None
        try:
            self._api.dataModel.signalStageReplaced.disconnect(self.Rebuild)
        except (RuntimeError, TypeError):
            pass
        LayerStackPanel._instance = None
        super(LayerStackPanel, self).closeEvent(event)

    # -- build -------------------------------------------------------

    def Rebuild(self, *args):
        stage = self._api.stage
        self._Listen(stage)
        self._rebuilding = True
        try:
            self._tree.clear()
            root = model.BuildTree(stage)
            if root is None:
                self._status.setText("No stage.")
                return
            item = self._AddNode(self._tree.invisibleRootItem(), root)
            self._tree.expandAll()
            del item
            muted = len(stage.GetMutedLayers())
            self._status.setText(
                "%d muted layer%s" % (muted, "" if muted == 1 else "s")
                if muted else "Every layer is on.")
        finally:
            self._rebuilding = False

    def _AddNode(self, parent, node):
        item = QtWidgets.QTreeWidgetItem(parent, [node.displayName])
        item.setData(0, _NODE_ROLE, node.identifier)
        item.setToolTip(0, node.identifier)
        flags = QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable
        if node.canMute and not node.hidden:
            flags |= QtCore.Qt.ItemIsUserCheckable
        item.setFlags(flags)
        if node.canMute:
            item.setCheckState(0, QtCore.Qt.Unchecked if node.muted
                               else QtCore.Qt.Checked)
        if not node.active or node.missing:
            item.setForeground(0, QtGui.QBrush(QtGui.QColor(128, 128, 128)))
        if node.missing:
            item.setText(0, "%s  [missing]" % node.displayName)
        for child in node.children:
            self._AddNode(item, child)
        return item

    # -- edits -------------------------------------------------------

    def _OnItemChanged(self, item, column):
        if self._rebuilding or column != 0:
            return
        identifier = item.data(0, _NODE_ROLE)
        muted = item.checkState(0) != QtCore.Qt.Checked
        # Not from inside this signal: see _QueueRebuild.
        QtCore.QTimer.singleShot(
            0, lambda: self._ApplyMute(identifier, muted))

    def _ApplyMute(self, identifier, muted):
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            self._Recorded(
                lambda s: model.SetMuted(s, identifier, muted),
                "%s %s" % ("Mute" if muted else "Unmute",
                           identifier.rsplit("/", 1)[-1]))
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self._QueueRebuild()

    def _OnUnmuteAll(self):
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            self._Recorded(model.UnmuteAll, "Unmute all layers")
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self._QueueRebuild()

    def _Recorded(self, change, label):
        """Run a mute through the container, and put it on the stack.

        A mute is not scene description -- nothing in any layer changes,
        so rigExecUndo has no spec to snapshot -- but it is very much an
        edit to what the artist is looking at, and unmuting a squetch
        layer by accident should take one Ctrl+Z to put back like
        anything else. model.MuteEdit carries the two sets of muted
        identifiers and replays them through this same callable.
        """
        before = model.MutedSet(self._api.stage)
        result = self._muteLayers(change)
        if self._undo is None:
            return result
        after = model.MutedSet(self._api.stage)
        if before != after:
            self._undo.Push(
                model.MuteEdit(self._muteLayers, before, after, label))
        return result


def OpenLayerStackPanel(usdviewApi, muteLayers=None, undoStack=None):
    panel = LayerStackPanel.GetInstance(usdviewApi, muteLayers, undoStack)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
