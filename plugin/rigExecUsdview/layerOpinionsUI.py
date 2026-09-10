#
# RigExec usdview plugin: the Layer Opinions panel.
#
# Shows every opinion the selected prim has, grouped by the layer that
# holds it, strongest layer first -- and lets you retype or remove one.
#
# A composition arc is shown as the list it is. `references` is a
# heading with one row per reference underneath it, and each of those
# rows can be retyped in place, removed on its own, moved against the
# arcs beside it, or reopened in the guided flow that would have added
# it. The same goes for the two arcs that belong to the LAYER rather
# than to the prim -- its sublayers and its relocates -- which are
# listed at the top of their layer's group.
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
# The string an expandable item's fold state is remembered under, so a
# rebuild -- which happens on every stage notice -- does not refold the
# tree under the mouse.
_EXPAND_ROLE = QtCore.Qt.UserRole + 3

# What the third column says a row is.
_KIND_TEXT = {"info": "metadata",
              "attribute": "attr",
              "relationship": "rel",
              "arcItem": "arc",
              "sublayer": "arc",
              "relocate": "arc",
              "variantSelection": "variant",
              "layerInfo": "layer"}


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
        item.setData(0, _EXPAND_ROLE, group.layer.identifier)
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
        item = QtWidgets.QTreeWidgetItem(
            parent, [row.key, row.valueText,
                     _KIND_TEXT.get(row.kind, row.kind)])
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
        for child in row.children:
            self._AddRow(item, child)
        if row.children:
            self._SeedRowExpansion(item, row)

    def _SeedRowExpansion(self, item, row):
        """
        Open an arc field the first time it is seen, and keep the fold
        the user leaves it at afterwards.

        Opened rather than folded, because the heading on its own
        ("3 items") answers nothing: the arcs underneath it are the
        reason these rows exist. The key is per layer AND per field, so
        folding one layer's `references` does not fold another's.
        """
        key = "%s|%s" % (row.layer.identifier, row.key)
        item.setData(COL_NAME, _EXPAND_ROLE, key)
        if key not in self._seeded:
            self._seeded.add(key)
            self._expanded.add(key)
        item.setExpanded(key in self._expanded)

    def _OnContextMenu(self, point):
        item = self._tree.itemAt(point)
        menu = QtWidgets.QMenu(self)
        row = item.data(COL_NAME, _ROW_ROLE) if item is not None else None
        group = item.data(0, _GROUP_ROLE) if item is not None else None
        if row is not None:
            self._AddRowActions(menu, item, row)
        elif group is not None:
            action = menu.addAction("Delete All Opinions In This Layer")
            action.setEnabled(group.editable)
            action.triggered.connect(lambda: self._DeletePrimSpec(group))
            copyId = menu.addAction("Copy Layer Path")
            copyId.triggered.connect(
                lambda: QtWidgets.QApplication.clipboard().setText(
                    group.layer.identifier))

        # Authoring a new arc is offered everywhere in the tree,
        # including the empty space below the rows -- a prim with no
        # opinions yet has nothing to right-click, and that is exactly
        # when you want to give it a reference.
        self._AddArcMenu(menu, group)
        try:
            menu.exec_(self._tree.viewport().mapToGlobal(point))
        finally:
            # Parented to the panel, so a menu per right-click would
            # otherwise pile up for the session -- each holding its
            # actions, their triggers, and the prim those captured.
            menu.deleteLater()

    def _AddRowActions(self, menu, item, row):
        """
        The actions for one opinion row.

        An arc row gets three the others do not: its own guided flow,
        reopened on the arc it already holds; and the two moves, which
        are the only way to change an arc's strength against the arcs
        beside it short of deleting and re-adding it.
        """
        if row.kind == "layerInfo":
            # The heading over a layer's own arcs. It is not an opinion,
            # so there is nothing to edit or delete on it -- the entries
            # underneath carry all of that.
            return
        arc = self._ArcFor(row)
        if arc is not None:
            action = menu.addAction(
                "Edit %s..." % arc.title.replace("Add ", ""))
            action.setToolTip(arc.summary)
            action.triggered.connect(lambda: self._EditArc(row))
        if row.editable:
            inline = menu.addAction("Edit Value")
            inline.triggered.connect(
                lambda: self._tree.editItem(item, COL_VALUE))
        delete = menu.addAction(_DeleteLabel(row))
        delete.setEnabled(model.CanDeleteRow(row, self._GroupFor(row.layer)))
        delete.triggered.connect(
            lambda: self._Apply(lambda: model.DeleteRow(row),
                                "Delete %s" % row.key))
        moves = (("Move Stronger", -1), ("Move Weaker", 1))
        if any(model.CanMove(row, delta) for _, delta in moves):
            menu.addSeparator()
            for label, delta in moves:
                move = menu.addAction(label)
                move.setEnabled(model.CanMove(row, delta))
                move.triggered.connect(
                    _MoveTrigger(self._Apply, row, delta))

    def _ArcFor(self, row):
        """
        The guided flow that can reopen `row`, or None.

        Imported where it is used for the same reason _AddArcMenu does
        it: the panel reads a stage perfectly well without the arc
        modules, and an ImportError must not take the context menu with
        it.
        """
        if not row.editable:
            return None
        try:
            import compositionArcsModel
        except ImportError:
            return None
        return compositionArcsModel.ArcForRow(row)

    def _EditArc(self, row):
        try:
            import compositionArcsUI
        except ImportError as error:
            self._status.setText("composition arcs unavailable: %s" % error)
            return
        edit, warnings = compositionArcsUI.RunArcEditFlow(
            self._api, row, self._SelectedPrim(), self)
        if edit is not None:
            self._OnArcAuthored(edit, warnings, edit.label)

    def _AddArcMenu(self, menu, group):
        """
        Hang the "Add Composition Arc" submenu off `menu`.

        Imported here rather than at module scope so a panel that is
        only being read still opens when the arc modules are missing or
        fail to import -- the flows are an addition to this panel, not a
        prerequisite for it.
        """
        try:
            import compositionArcsUI
        except ImportError as error:
            self._status.setText("composition arcs unavailable: %s" % error)
            return
        if not menu.isEmpty():
            menu.addSeparator()
        # Constructed with the parent menu as its QObject parent, not
        # via menu.addMenu(title): that convenience does NOT transfer
        # ownership, so the submenu is collected the moment this
        # function returns and the items open onto a deleted C++ object.
        submenu = QtWidgets.QMenu("Add Composition Arc", menu)
        menu.addMenu(submenu)
        compositionArcsUI.PopulateArcMenu(
            submenu, self._api, self._SelectedPrim(),
            group.layer if group is not None else None,
            self._OnArcAuthored, self)

    def _OnArcAuthored(self, edit, warnings, label):
        """
        A flow committed: push its Edit and rebuild, exactly as an
        inline edit does. Warnings are reported afterwards because the
        dialog already showed them and the artist chose to go ahead --
        the status line is a record, not a second prompt.
        """
        if edit is not None and self._undo is not None:
            self._undo.Push(edit)
        self.Rebuild()
        self._status.setText(
            "%s  (%s)" % (label, "; ".join(warnings)) if warnings else label)

    def _OnExpansionChanged(self, item):
        # Keyed off the role rather than off the item's kind, so a layer
        # group and an arc field remember their fold the same way.
        key = item.data(0, _EXPAND_ROLE)
        if key is None:
            return
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
                enabled = model.CanDeleteRow(row, self._GroupFor(row.layer))
                text = _DeleteLabel(row)
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


def _DeleteLabel(row):
    """
    What removing this row is called.

    An entry of a list is not "the opinion". Deleting one reference off
    a prim that has three is not the same act as deleting the layer's
    whole say about references, and a menu that calls both "Delete
    Opinion" invites the wrong one.
    """
    if row.kind in model.ENTRY_KINDS:
        return "Remove This Entry"
    return "Delete Opinion"


class _MoveTrigger(object):
    """
    A callable holding one move action's row and direction.

    A class rather than a lambda in the loop, for the reason
    compositionArcsUI._ArcTrigger is one: a lambda would capture the
    loop variable by reference and both items would move the same way.
    """

    def __init__(self, apply, row, delta):
        self._apply = apply
        self._row = row
        self._delta = delta

    def __call__(self, *args):
        row, delta = self._row, self._delta
        self._apply(lambda: model.MoveRow(row, delta), "Move %s" % row.key)


def OpenLayerOpinionsPanel(usdviewApi, undoStack=None):
    panel = LayerOpinionsPanel.GetInstance(usdviewApi, undoStack)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
