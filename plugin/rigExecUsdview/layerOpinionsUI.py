#
# RigExec usdview plugin: the Layer Opinions panel.
#
# Shows every opinion the selected prim has, grouped by the layer that
# holds it, strongest layer first -- and lets you retype or remove one.
#
# All of the rules live in layerOpinionsModel, which imports no Qt and
# is tested headlessly (tests/python/test_layer_opinions_model.py). This
# file is a thin Qt driver over it: build a tree, dispatch edits, push
# the returned Edit onto the shared undo stack, rebuild.
#
from pxr import Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

import layerOpinionsModel as model


# Column layout. VALUE is the only editable one.
COL_NAME = 0
COL_VALUE = 1
COL_KIND = 2
_COLUMNS = ("Opinion", "Value", "")

# Roles carrying the model objects on their tree items.
_ROW_ROLE = QtCore.Qt.UserRole + 1
_GROUP_ROLE = QtCore.Qt.UserRole + 2


def _Muted(stage, layer):
    return stage.IsLayerMuted(layer.identifier)


def _GroupLabel(group, isEditTarget):
    """
    The layer header text: display name plus the badges that explain why
    a row might not be editable, or why an edit is landing elsewhere.
    """
    badges = []
    if isEditTarget:
        badges.append("edit target")
    if not group.editable:
        badges.append("read-only")
    name = group.displayName or group.layer.identifier
    return "%s  [%s]" % (name, ", ".join(badges)) if badges else name


class _ValueDelegate(QtWidgets.QStyledItemDelegate):
    """
    A plain line edit over the value column.

    Plain on purpose: every type is entered as the usda text the model
    round-trips, so there is one editor rather than a widget per Sdf
    type, and a typo is reported as a parse error instead of being
    coerced into something that silently differs.
    """

    def createEditor(self, parent, option, index):
        row = index.data(_ROW_ROLE)
        if row is None or not row.editable:
            return None
        return QtWidgets.QLineEdit(parent)

    def setEditorData(self, editor, index):
        editor.setText(index.data(QtCore.Qt.DisplayRole) or "")

    def setModelData(self, editor, itemModel, index):
        # Written by the panel, not here: applying the edit needs the
        # undo stack and a rebuild, neither of which a delegate owns.
        itemModel.setData(index, editor.text(), QtCore.Qt.EditRole)


class LayerOpinionsPanel(QtWidgets.QDialog):
    """The panel window. One per usdview session."""

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi, undoStack):
        if cls._instance is None:
            cls._instance = LayerOpinionsPanel(usdviewApi, undoStack)
        return cls._instance

    def __init__(self, usdviewApi, undoStack, parent=None):
        super(LayerOpinionsPanel, self).__init__(parent)
        self._api = usdviewApi
        self._undo = undoStack
        self._groups = []
        self._expanded = set()
        self._seeded = set()
        self._rebuilding = False
        self._noticeKey = None

        self.setWindowTitle("Layer Opinions")
        self.resize(720, 520)

        layout = QtWidgets.QVBoxLayout(self)
        self._header = QtWidgets.QLabel("")
        self._header.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self._header)

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setColumnCount(len(_COLUMNS))
        self._tree.setHeaderLabels(_COLUMNS)
        self._tree.setRootIsDecorated(True)
        self._tree.setAlternatingRowColors(True)
        self._tree.setSelectionMode(
            QtWidgets.QAbstractItemView.SingleSelection)
        self._tree.setEditTriggers(
            QtWidgets.QAbstractItemView.DoubleClicked
            | QtWidgets.QAbstractItemView.EditKeyPressed)
        self._tree.setItemDelegateForColumn(COL_VALUE, _ValueDelegate(self))
        self._tree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self._tree.customContextMenuRequested.connect(self._OnContextMenu)
        self._tree.itemChanged.connect(self._OnItemChanged)
        self._tree.itemExpanded.connect(self._OnExpansionChanged)
        self._tree.itemCollapsed.connect(self._OnExpansionChanged)
        self._tree.header().setSectionResizeMode(
            COL_VALUE, QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self._tree, 1)

        self._status = QtWidgets.QLabel("")
        layout.addWidget(self._status)

        buttons = QtWidgets.QHBoxLayout()
        self._deleteButton = QtWidgets.QPushButton("Delete Opinion")
        self._deleteButton.clicked.connect(self._OnDeleteSelected)
        buttons.addWidget(self._deleteButton)
        buttons.addStretch(1)
        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self.Rebuild)
        buttons.addWidget(refresh)
        layout.addLayout(buttons)

        self._tree.itemSelectionChanged.connect(self._SyncButtons)

        # Two independent reasons to rebuild: the selection moved, and
        # the stage changed underneath us (including by our own edits,
        # and by an undo from anywhere else in the plugin).
        self._Connect()
        self.Rebuild()

    # -- wiring ------------------------------------------------------

    def _Connect(self):
        # signalPrimSelectionChanged emits (added, removed); Rebuild
        # takes *args so it can serve as the slot directly.
        self._api.dataModel.selection.signalPrimSelectionChanged.connect(
            self.Rebuild)
        self._noticeKey = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._OnObjectsChanged,
            self._api.stage)

    def _OnObjectsChanged(self, notice, sender):
        if not self._rebuilding:
            self.Rebuild()

    def closeEvent(self, event):
        # Revoke before Qt deletes the C++ side, or the notice fires
        # into a dead widget.
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        LayerOpinionsPanel._instance = None
        super(LayerOpinionsPanel, self).closeEvent(event)

    # -- state -------------------------------------------------------

    def _SelectedPrim(self):
        """
        The prim to show, or None.

        usdviewApi.prim is the FOCUS prim, which is getPrimPaths()[0] --
        and clearing the selection leaves the pseudo-root in that list
        rather than emptying it. The pseudo-root carries no opinions and
        is never what someone meant to inspect, so it reads as nothing
        selected instead of as an empty tree.
        """
        prim = self._api.prim
        if not prim or not prim.IsValid() or prim.IsPseudoRoot():
            return None
        return prim

    def _EditTargetLayer(self):
        return self._api.stage.GetEditTarget().GetLayer()

    # -- build -------------------------------------------------------

    def Rebuild(self, *args):
        self._rebuilding = True
        try:
            self._tree.clear()
            prim = self._SelectedPrim()
            if prim is None:
                self._header.setText("No prim selected.")
                self._groups = []
                self._SyncButtons()
                return
            self._header.setText(str(prim.GetPath()))
            self._groups = model.OpinionGroups(prim)
            editTarget = self._EditTargetLayer()
            stage = self._api.stage
            for group in self._groups:
                group.isEditTarget = (group.layer == editTarget)
            self._SeedExpansion()
            for group in self._groups:
                self._AddGroup(group, stage)
            self._tree.resizeColumnToContents(COL_NAME)
        finally:
            self._rebuilding = False
            self._SyncButtons()

    def _SeedExpansion(self):
        """
        Decide the fold state for groups this panel has not shown before.

        The edit target opens, so an edit you are about to make is in
        view, and every other layer starts folded so a deep stack stays
        readable. But the edit target usually holds NO opinion for the
        prim -- usdview edits the session layer, which starts empty --
        and then no group is the edit target at all. Opening only that
        one would leave the panel looking empty on the most ordinary
        stage there is, so the strongest group opens instead.

        Only unseen layers are seeded: after the first time, the user's
        own folding is what decides.
        """
        fresh = [g for g in self._groups
                 if g.layer.identifier not in self._seeded]
        for group in fresh:
            self._seeded.add(group.layer.identifier)
        if not fresh:
            return
        opened = [g for g in fresh if g.isEditTarget]
        if not opened and not self._expanded:
            opened = self._groups[:1]
        for group in opened:
            self._expanded.add(group.layer.identifier)

    def _AddGroup(self, group, stage):
        item = QtWidgets.QTreeWidgetItem(
            self._tree, [_GroupLabel(group, group.isEditTarget), "", ""])
        item.setData(0, _GROUP_ROLE, group)
        font = item.font(COL_NAME)
        font.setBold(True)
        item.setFont(COL_NAME, font)
        if _Muted(stage, group.layer):
            item.setForeground(COL_NAME, QtGui.QBrush(QtCore.Qt.gray))
        item.setToolTip(COL_NAME, group.layer.identifier)

        for row in group.rows:
            self._AddRow(item, row)

        item.setExpanded(group.layer.identifier in self._expanded)

    def _AddRow(self, parent, row):
        kindText = {"info": "metadata",
                    "attribute": "attr",
                    "relationship": "rel"}.get(row.kind, row.kind)
        item = QtWidgets.QTreeWidgetItem(
            parent, [row.key, row.valueText, kindText])
        item.setData(COL_VALUE, _ROW_ROLE, row)
        item.setData(COL_NAME, _ROW_ROLE, row)
        flags = item.flags()
        if row.editable:
            flags |= QtCore.Qt.ItemIsEditable
        else:
            flags &= ~QtCore.Qt.ItemIsEditable
        item.setFlags(flags)
        if not row.winning:
            # Shadowed by a stronger layer: struck through, so "I edited
            # it and nothing happened" is answered by looking at it.
            font = item.font(COL_NAME)
            font.setStrikeOut(True)
            item.setFont(COL_NAME, font)
            item.setToolTip(
                COL_NAME,
                "overridden by a stronger layer above")
        if not row.editable:
            item.setForeground(COL_VALUE, QtGui.QBrush(QtCore.Qt.gray))

    def _OnContextMenu(self, point):
        item = self._tree.itemAt(point)
        if item is None:
            return
        menu = QtWidgets.QMenu(self)
        row = item.data(COL_NAME, _ROW_ROLE)
        group = item.data(0, _GROUP_ROLE)
        if row is not None:
            owner = self._GroupFor(row.layer)
            action = menu.addAction("Delete Opinion")
            action.setEnabled(bool(owner and owner.editable))
            action.triggered.connect(
                lambda: self._Apply(lambda: model.DeleteRow(row),
                                    "Delete %s" % row.key))
            if row.editable:
                edit = menu.addAction("Edit Value")
                edit.triggered.connect(
                    lambda: self._tree.editItem(item, COL_VALUE))
        elif group is not None:
            action = menu.addAction("Delete All Opinions In This Layer")
            action.setEnabled(group.editable)
            action.triggered.connect(lambda: self._DeletePrimSpec(group))
            copyId = menu.addAction("Copy Layer Path")
            copyId.triggered.connect(
                lambda: QtWidgets.QApplication.clipboard().setText(
                    group.layer.identifier))
        menu.exec_(self._tree.viewport().mapToGlobal(point))

    def _OnExpansionChanged(self, item):
        group = item.data(0, _GROUP_ROLE)
        if group is None:
            return
        key = group.layer.identifier
        if item.isExpanded():
            self._expanded.add(key)
        else:
            self._expanded.discard(key)

    # -- edits -------------------------------------------------------

    def _OnItemChanged(self, item, column):
        if self._rebuilding or column != COL_VALUE:
            return
        row = item.data(COL_VALUE, _ROW_ROLE)
        if row is None:
            return
        text = item.text(COL_VALUE)
        if text == row.valueText:
            return
        self._Apply(lambda: model.SetRowValue(row, text),
                    "Set %s" % row.key)

    def _OnDeleteSelected(self):
        items = self._tree.selectedItems()
        if not items:
            return
        item = items[0]
        row = item.data(COL_NAME, _ROW_ROLE)
        if row is not None:
            self._Apply(lambda: model.DeleteRow(row), "Delete %s" % row.key)
            return
        group = item.data(0, _GROUP_ROLE)
        if group is not None:
            self._DeletePrimSpec(group)

    def _DeletePrimSpec(self, group):
        answer = QtWidgets.QMessageBox.question(
            self, "Delete all opinions",
            "Remove every opinion for this prim from\n%s?\n\n"
            "This removes the prim's whole spec in that layer, including "
            "any descendants it defines there." % group.layer.identifier,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self._Apply(lambda: model.DeletePrimSpec(group),
                    "Delete %s opinions" % group.displayName)

    def _Apply(self, operation, label):
        """
        Run one model operation, push its Edit, rebuild.

        A parse error is reported in the status line and leaves the
        stage untouched -- the model raises before it authors anything.
        """
        try:
            edit = operation()
        except model.ValueParseError as error:
            self._status.setText(str(error))
            self.Rebuild()
            return
        except Tf.ErrorException as error:
            self._status.setText(str(error))
            self.Rebuild()
            return
        if edit is not None and self._undo is not None:
            self._undo.Push(edit)
        self._status.setText(label)
        self.Rebuild()

    def _SyncButtons(self):
        items = self._tree.selectedItems()
        enabled = False
        text = "Delete Opinion"
        if items:
            row = items[0].data(COL_NAME, _ROW_ROLE)
            group = items[0].data(0, _GROUP_ROLE)
            if row is not None:
                enabled = bool(
                    self._GroupFor(row.layer)
                    and self._GroupFor(row.layer).editable)
            elif group is not None:
                enabled = group.editable
                text = "Delete All In Layer"
        self._deleteButton.setEnabled(enabled)
        self._deleteButton.setText(text)

    def _GroupFor(self, layer):
        for group in self._groups:
            if group.layer == layer:
                return group
        return None


def OpenLayerOpinionsPanel(usdviewApi, undoStack=None):
    panel = LayerOpinionsPanel.GetInstance(usdviewApi, undoStack)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
