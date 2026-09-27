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
import panelIcons
import pathSpelling
import sessionRegistry


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
              "connection": "conn",
              "arcItem": "arc",
              "sublayer": "arc",
              "relocate": "arc",
              "variantSelection": "variant",
              "layerInfo": "layer"}


# The rows whose value is a list of paths, and so get the target picker.
_PATH_LIST_KINDS = ("relationship", "connection")

# The panel's own look. Scoped by object name so none of it leaks into
# usdview's chrome, and written for usdview's dark palette -- which is
# the only one it ships.
_STYLE = """
#opinionsHeader { background: #2b3038; border: 1px solid #3a404a;
                  border-radius: 8px; }
#opinionsTitle { font-size: 15px; font-weight: 600; color: #eef1f5; }
#opinionsSubtitle { color: #8e98a6; }
#opinionsFilter { background: #1f2329; border: 1px solid #3a404a;
                  border-radius: 12px; padding: 3px 10px; color: #dfe4ea; }
#opinionsFilter:focus { border-color: #5b8fd0; }
#opinionsTree { background: #1f2329; alternate-background-color: #23282f;
                border: 1px solid #3a404a; border-radius: 8px;
                color: #dfe4ea; }
#opinionsTree::item { padding: 3px 2px; }
#opinionsTree::item:selected { background: #34537a; color: #ffffff; }
#opinionsTree QHeaderView::section { background: #2b3038; color: #8e98a6;
                border: none; border-bottom: 1px solid #3a404a;
                padding: 4px 8px; font-weight: 600; }
#opinionsStatus { border-radius: 10px; padding: 3px 10px; color: #cfd6de; }
#opinionsStatus[state="ok"] { background: #1f3a2c; color: #9be0b5; }
#opinionsStatus[state="error"] { background: #45232a; color: #ffb4bd; }
#opinionsHint { color: #6f7988; }
QToolButton#opinionsAction { background: #2b3038; border: 1px solid #3a404a;
                border-radius: 6px; padding: 4px 10px; color: #dfe4ea; }
QToolButton#opinionsAction:hover { background: #34537a;
                border-color: #5b8fd0; }
QToolButton#opinionsAction:disabled { color: #5d6572; }
"""

# Row tints: an edit-target group, any other group, and the ink of a
# value that is overridden by a stronger layer.
_GROUP_BG = QtGui.QColor(43, 48, 56)
_TARGET_BG = QtGui.QColor(38, 61, 92)
_SHADOWED_INK = QtGui.QColor(118, 126, 138)
_MUTED_INK = QtGui.QColor(142, 152, 166)


def _MonoFont():
    font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
    return font


def _Muted(stage, layer):
    return stage.IsLayerMuted(layer.identifier)


def _GroupLabel(group, isEditTarget):
    """
    The layer header text: display name plus the badges that explain why
    a row might not be editable, or why an edit is landing elsewhere.
    """
    badges = []
    if isEditTarget:
        badges.append("\u25cf edit target")
    if not group.editable:
        badges.append("read-only")
    if not group.local:
        badges.append("via arc")
    name = group.displayName or group.layer.identifier
    return "%s     %s" % (name, "   \u00b7   ".join(badges)) if badges \
        else name


class _PathLineEdit(QtWidgets.QLineEdit):
    """
    The line edit inside a _PathListEditor. It answers Return and Escape
    itself: the delegate's own key handling watches the COMPOSITE editor,
    which never sees them, and QLineEdit would otherwise pass Return up
    past it to the dialog.
    """

    submitted = QtCore.Signal()
    cancelled = QtCore.Signal()

    def keyPressEvent(self, event):
        if event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
            self.submitted.emit()
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_Escape:
            self.cancelled.emit()
            event.accept()
            return
        super(_PathLineEdit, self).keyPressEvent(event)


class _PathListEditor(QtWidgets.QWidget):
    """
    The editor for a relationship's targets or an attribute's
    connections: the usda text, plus a picker of the paths nearby.

    The picker offers what model.TargetCandidates ranks -- siblings and
    children first, each written RELATIVE to the owning prim (`<../Wrist>`,
    `<Child.out>`) -- and choosing one adds it to the list rather than
    replacing it. The text stays the source of truth, so anything the
    picker does not list can still be typed.
    """

    submitted = QtCore.Signal()
    cancelled = QtCore.Signal()

    def __init__(self, parent, candidates, connection):
        super(_PathListEditor, self).__init__(parent)
        self.setAutoFillBackground(True)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        self._edit = _PathLineEdit(self)
        self._edit.setFont(_MonoFont())
        self._edit.setPlaceholderText(
            "[ <../Other.out> ]" if connection else "[ <../Sibling> ]")
        self._edit.submitted.connect(self.submitted)
        self._edit.cancelled.connect(self.cancelled)
        layout.addWidget(self._edit, 1)

        pick = QtWidgets.QToolButton(self)
        pick.setIcon(panelIcons.Icon("target", QtGui.QColor(120, 200, 255)))
        pick.setToolTip(
            "Add a %s -- nearby paths first, written relative to this prim"
            % ("connection source" if connection else "target"))
        pick.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        pick.setFocusPolicy(QtCore.Qt.NoFocus)
        menu = QtWidgets.QMenu(pick)
        if candidates:
            for candidate in candidates:
                action = menu.addAction(candidate)
                action.triggered.connect(
                    lambda checked=False, c=candidate: self._Add(c))
        else:
            empty = menu.addAction("Nothing nearby to offer")
            empty.setEnabled(False)
        pick.setMenu(menu)
        layout.addWidget(pick)
        self.setFocusProxy(self._edit)

    def text(self):
        return self._edit.text()

    def setText(self, text):
        self._edit.setText(text)
        self._edit.selectAll()

    def _Add(self, candidate):
        body = self._edit.text().strip()
        if body.startswith("[") and body.endswith("]"):
            body = body[1:-1]
        items = [item.strip() for item in body.split(",") if item.strip()]
        if candidate not in items:
            items.append(candidate)
        self._edit.setText("[ %s ]" % ", ".join(items))
        self._edit.setFocus()


class _ValueDelegate(QtWidgets.QStyledItemDelegate):
    """
    A plain line edit over the value column.

    Plain on purpose: every type is entered as the usda text the model
    round-trips (or the shorthand it accepts for it), so there is one
    editor rather than a widget per Sdf type, and a typo is reported as
    a parse error instead of being coerced into something that silently
    differs.
    """

    def __init__(self, panel):
        super(_ValueDelegate, self).__init__(panel)
        self._panel = panel

    def createEditor(self, parent, option, index):
        row = index.data(_ROW_ROLE)
        if row is None or not row.editable:
            return None
        if row.kind in _PATH_LIST_KINDS:
            editor = _PathListEditor(
                parent, self._panel.TargetCandidates(row),
                row.kind == "connection")
            editor.submitted.connect(lambda: self._Finish(editor, True))
            editor.cancelled.connect(lambda: self._Finish(editor, False))
            return editor
        editor = QtWidgets.QLineEdit(parent)
        editor.setFont(_MonoFont())
        return editor

    def _Finish(self, editor, commit):
        if commit:
            self.commitData.emit(editor)
        hint = (QtWidgets.QAbstractItemDelegate.NoHint if commit
                else QtWidgets.QAbstractItemDelegate.RevertModelCache)
        self.closeEditor.emit(editor, hint)

    def setEditorData(self, editor, index):
        # A refused edit reopens on the text that was refused, so fixing
        # a typo does not mean retyping the whole value.
        retry = self._panel.TakeRetryText(index.data(_ROW_ROLE))
        editor.setText(retry if retry is not None
                       else index.data(QtCore.Qt.DisplayRole) or "")

    def setModelData(self, editor, itemModel, index):
        # Written by the panel, not here: applying the edit needs the
        # undo stack and a rebuild, neither of which a delegate owns.
        itemModel.setData(index, editor.text(), QtCore.Qt.EditRole)


class LayerOpinionsPanel(QtWidgets.QDialog,
                         metaclass=sessionRegistry.PerSessionInstanceMeta(
                             QtWidgets.QDialog)):
    """The panel window. One per usdview session.

    Filed under the session's main window: several usdview sessions can
    share this module in one process. `LayerOpinionsPanel._instance` is
    the current session's panel (see
    sessionRegistry.PerSessionInstanceMeta); assigning None forgets it.
    """

    _sessions = sessionRegistry.SessionRegistry("layer opinion panels")

    @classmethod
    def GetInstance(cls, usdviewApi, undoStack):
        panel = cls._sessions.Get(usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(
                usdviewApi, LayerOpinionsPanel(usdviewApi, undoStack))
        return panel

    def __init__(self, usdviewApi, undoStack, parent=None):
        super(LayerOpinionsPanel, self).__init__(parent)
        self._api = usdviewApi
        self._undo = undoStack
        self._groups = []
        self._expanded = set()
        self._seeded = set()
        self._rebuilding = False
        self._noticeKey = None
        # A stage change that arrived while an editor was open. The
        # rebuild waits for the editor to close: rebuilding under it
        # deletes the item being typed into and throws the text away.
        self._staleWhileEditing = False
        # (row identity, text) of an edit the model just refused.
        self._retry = None
        # Set while an itemChanged is being handled: a rebuild then is
        # queued instead of run, see _RequestRebuild.
        self._inItemChanged = False
        self._rebuildQueued = False
        # The prim the tree is showing, for filtering stage notices.
        self._shownPath = None
        # (row, text) to reopen an editor on once the tree is rebuilt.
        self._reopen = None

        self.setWindowTitle("Layer Opinions")
        self.setWindowIcon(panelIcons.Icon("layer"))
        self.resize(760, 560)
        self.setStyleSheet(_STYLE)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)
        layout.addWidget(self._BuildHeader())

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setObjectName("opinionsTree")
        self._tree.setIconSize(QtCore.QSize(16, 16))
        self._tree.setUniformRowHeights(False)
        self._tree.setColumnCount(len(_COLUMNS))
        self._tree.setHeaderLabels(_COLUMNS)
        self._tree.setRootIsDecorated(True)
        self._tree.setAlternatingRowColors(True)
        self._tree.setSelectionMode(
            QtWidgets.QAbstractItemView.SingleSelection)
        self._tree.setEditTriggers(
            QtWidgets.QAbstractItemView.DoubleClicked
            | QtWidgets.QAbstractItemView.EditKeyPressed)
        delegate = _ValueDelegate(self)
        delegate.closeEditor.connect(self._OnEditorClosed)
        self._tree.setItemDelegateForColumn(COL_VALUE, delegate)
        self._tree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self._tree.customContextMenuRequested.connect(self._OnContextMenu)
        self._tree.itemChanged.connect(self._OnItemChanged)
        self._tree.itemExpanded.connect(self._OnExpansionChanged)
        self._tree.itemCollapsed.connect(self._OnExpansionChanged)
        self._tree.header().setSectionResizeMode(
            COL_VALUE, QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self._tree, 1)

        hint = QtWidgets.QLabel(
            "Double-click a value to edit. Shorthand is fine: ik, 4 5 6, "
            "bar.png, ../Wrist -- paths stay as typed, relative or "
            "absolute, and Save Layer writes them that way.")
        hint.setObjectName("opinionsHint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        buttons = QtWidgets.QHBoxLayout()
        buttons.setSpacing(6)
        self._arcButton = self._ActionButton(
            "Add Arc", "arcAdd", QtGui.QColor(200, 140, 240))
        self._arcButton.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        self._arcButton.setToolTip(
            "Add a composition arc to the selected prim")
        # Filled when it opens, so it always reflects the current prim
        # and the group under the selection.
        self._arcMenu = QtWidgets.QMenu(self._arcButton)
        self._arcMenu.aboutToShow.connect(self._FillArcButtonMenu)
        self._arcButton.setMenu(self._arcMenu)
        buttons.addWidget(self._arcButton)
        self._deleteButton = self._ActionButton(
            "Delete Opinion", "delete", QtGui.QColor(240, 130, 140))
        self._deleteButton.clicked.connect(self._OnDeleteSelected)
        buttons.addWidget(self._deleteButton)
        buttons.addStretch(1)
        self._status = QtWidgets.QLabel("")
        self._status.setObjectName("opinionsStatus")
        buttons.addWidget(self._status)
        self._saveButton = self._ActionButton("Save Layer", "layer")
        self._saveButton.setToolTip(
            "Save the selected layer, every path spelled as it was "
            "typed -- relative or absolute")
        self._saveButton.clicked.connect(self._OnSaveSelected)
        buttons.addWidget(self._saveButton)
        refresh = self._ActionButton("Refresh", "refresh")
        refresh.clicked.connect(self.Rebuild)
        buttons.addWidget(refresh)
        layout.addLayout(buttons)

        self._tree.itemSelectionChanged.connect(self._SyncButtons)

        # Two independent reasons to rebuild: the selection moved, and
        # the stage changed underneath us (including by our own edits,
        # and by an undo from anywhere else in the plugin).
        self._Connect()
        self.Rebuild()

    def _BuildHeader(self):
        """The card over the tree: which prim, what it is, and a filter."""
        card = QtWidgets.QFrame()
        card.setObjectName("opinionsHeader")
        row = QtWidgets.QHBoxLayout(card)
        row.setContentsMargins(12, 8, 12, 8)
        row.setSpacing(10)
        self._headerIcon = QtWidgets.QLabel()
        self._headerIcon.setPixmap(panelIcons.Icon(
            "layer", QtGui.QColor(120, 170, 240)).pixmap(28, 28))
        row.addWidget(self._headerIcon)
        text = QtWidgets.QVBoxLayout()
        text.setSpacing(0)
        # The name reads large; `_header` keeps the full path, which is
        # what the usdview tests (and a copy-paste) want from it.
        self._title = QtWidgets.QLabel("")
        self._title.setObjectName("opinionsTitle")
        self._header = QtWidgets.QLabel("")
        self._header.setObjectName("opinionsSubtitle")
        for label in (self._title, self._header):
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            text.addWidget(label)
        row.addLayout(text, 1)
        self._filter = QtWidgets.QLineEdit()
        self._filter.setObjectName("opinionsFilter")
        self._filter.setPlaceholderText("Filter opinions...")
        self._filter.setClearButtonEnabled(True)
        self._filter.setFixedWidth(200)
        self._filter.textChanged.connect(lambda *_: self._ApplyFilter())
        row.addWidget(self._filter)
        return card

    def _ActionButton(self, text, glyph, color=None):
        button = QtWidgets.QToolButton()
        button.setObjectName("opinionsAction")
        button.setText(text)
        button.setIcon(panelIcons.Icon(glyph, color))
        button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        button.setAutoRaise(False)
        return button

    def _SetStatus(self, text, state="", detail=""):
        """The status pill: `state` is "ok", "error" or "" (neutral)."""
        self._status.setText(text)
        self._status.setToolTip(detail)
        self._status.setProperty("state", state)
        # A dynamic property does not restyle by itself.
        self._status.style().unpolish(self._status)
        self._status.style().polish(self._status)

    def TargetCandidates(self, row):
        """
        The picker's paths for a relationship or connection row, found
        around the prim as the STAGE has it -- the one on screen.

        That is right even for a row whose spec lives in a referenced
        layer or inside a variant, where the spec path differs from the
        stage path: the candidates are offered RELATIVE, and a relative
        path means the same thing in the asset's namespace as in the
        stage's, because the reference maps the whole subtree together.
        """
        prim = self._SelectedPrim()
        if prim is None:
            return []
        try:
            return model.TargetCandidates(
                self._api.stage, prim.GetPath(),
                properties=(row.kind == "connection"), limit=60)
        except Exception as error:
            # Said, not swallowed: an empty picker with no reason is
            # indistinguishable from a prim with nothing near it.
            self._SetStatus("target picker failed: %s" % error, "error")
            return []

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
        if self._rebuilding or not self._Concerns(notice):
            return
        if self._Editing():
            self._staleWhileEditing = True
            return
        # Queued, not run: a drag or a script authors in bursts, and one
        # rebuild at the end of the burst shows the same thing as fifty.
        self._QueueRebuild()

    def _Concerns(self, notice):
        """
        Whether `notice` can change what the panel shows.

        Only the shown prim's own opinions are listed, so a change to any
        OTHER prim -- a gizmo drag on a control, a script writing a
        different mesh -- cannot. What can: the prim or one of its
        properties, a resync of it or an ancestor (which includes the
        pseudo-root, where sublayer and relocate edits land).
        """
        shown = self._shownPath
        if shown is None:
            return False
        for path in (list(notice.GetResyncedPaths())
                     + list(notice.GetChangedInfoOnlyPaths())):
            if shown.HasPrefix(path) or path.GetPrimPath() == shown:
                return True
        return False

    def _RequestRebuild(self):
        """
        Rebuild now -- or, inside an itemChanged, on the next turn of the
        event loop. itemChanged is emitted from inside the item's own
        setData, and a rebuild clears the tree, deleting that item while
        its setData is still on the stack. The EDIT is still applied
        synchronously; only the tree waits.
        """
        if not self._inItemChanged:
            self.Rebuild()
            return
        self._QueueRebuild()

    def _QueueRebuild(self):
        """Rebuild on the next turn of the event loop, once however
        many times this is asked for before then."""
        if not self._rebuildQueued:
            self._rebuildQueued = True
            QtCore.QTimer.singleShot(0, self._RunQueuedRebuild)

    def _RunQueuedRebuild(self):
        if self._rebuildQueued:
            self.Rebuild()

    def _Editing(self):
        return (self._tree.state()
                == QtWidgets.QAbstractItemView.EditingState)

    def _OnEditorClosed(self, *args):
        if self._staleWhileEditing:
            self._staleWhileEditing = False
            # Deferred: the view is still inside closeEditor, and the
            # rebuild deletes the item it is closing.
            QtCore.QTimer.singleShot(0, self.Rebuild)

    def closeEvent(self, event):
        # Revoke before Qt deletes the C++ side, or the notice fires
        # into a dead widget.
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        if LayerOpinionsPanel._sessions.Get(self._api) is self:
            LayerOpinionsPanel._sessions.Pop(self._api)
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
        self._rebuildQueued = False
        self._rebuilding = True
        try:
            self._tree.clear()
            prim = self._SelectedPrim()
            self._shownPath = prim.GetPath() if prim is not None else None
            if prim is None:
                self._title.setText("No prim selected")
                self._header.setText(
                    "Select a prim in the hierarchy or the viewport.")
                self._groups = []
                self._SyncButtons()
                return
            self._title.setText("%s   <span style='color:#8e98a6; "
                                "font-size:11px; font-weight:400'>%s</span>"
                                % (prim.GetName(),
                                   prim.GetTypeName() or "untyped"))
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
            self._ApplyFilter()
        finally:
            self._rebuilding = False
            self._SyncButtons()
        self._ReopenRefused()

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
        item.setIcon(COL_NAME, panelIcons.Icon(
            "target" if group.isEditTarget else "layer",
            QtGui.QColor(120, 200, 255) if group.isEditTarget else None))
        band = QtGui.QBrush(_TARGET_BG if group.isEditTarget else _GROUP_BG)
        for column in range(len(_COLUMNS)):
            item.setBackground(column, band)
        if _Muted(stage, group.layer):
            item.setForeground(COL_NAME, QtGui.QBrush(QtCore.Qt.gray))
        item.setToolTip(COL_NAME, group.layer.identifier)

        for row in group.rows:
            self._AddRow(item, row)

        item.setExpanded(group.layer.identifier in self._expanded)
        # Spanned so the layer name and its badges are never cut off by
        # the name column's width.
        item.setFirstColumnSpanned(True)

    def _AddRow(self, parent, row):
        item = QtWidgets.QTreeWidgetItem(
            parent, [row.key, row.valueText,
                     _KIND_TEXT.get(row.kind, row.kind)])
        item.setData(COL_VALUE, _ROW_ROLE, row)
        item.setData(COL_NAME, _ROW_ROLE, row)
        item.setIcon(COL_NAME, panelIcons.KindIcon(row.kind))
        item.setFont(COL_VALUE, _MonoFont())
        item.setForeground(COL_KIND, QtGui.QBrush(_MUTED_INK))
        if row.summarized:
            # A summary, not the value: say so, and where the value is.
            font = _MonoFont()
            font.setItalic(True)
            item.setFont(COL_VALUE, font)
            item.setToolTip(
                COL_VALUE, "Too large to show or edit inline. "
                "Right-click > Copy Full Value for all of it.")
        elif row.editable:
            item.setToolTip(COL_VALUE, "Double-click to edit")
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
            item.setForeground(COL_NAME, QtGui.QBrush(_SHADOWED_INK))
            item.setForeground(COL_VALUE, QtGui.QBrush(_SHADOWED_INK))
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
            save = menu.addAction(panelIcons.Icon("layer"), "Save Layer")
            save.setEnabled(_CanSave(group.layer))
            save.triggered.connect(lambda: self._SaveLayer(group.layer))
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
                panelIcons.Icon("arc", panelIcons.KIND_COLORS["arcItem"]),
                "Edit %s..." % arc.title.replace("Add ", ""))
            action.setToolTip(arc.summary)
            action.triggered.connect(lambda: self._EditArc(row))
        if row.summarized:
            copy = menu.addAction("Copy Full Value")
            copy.triggered.connect(
                lambda: QtWidgets.QApplication.clipboard().setText(
                    model.FullValueText(row)))
        if row.editable:
            inline = menu.addAction("Edit Value")
            inline.triggered.connect(
                lambda: self._tree.editItem(item, COL_VALUE))
        delete = menu.addAction(
            panelIcons.Icon("delete", QtGui.QColor(240, 130, 140)),
            _DeleteLabel(row))
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
        submenu.setIcon(panelIcons.Icon(
            "arcAdd", panelIcons.KIND_COLORS["arcItem"]))
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
        self._SetStatus(
            "%s  (%s)" % (label, "; ".join(warnings)) if warnings else label,
            "ok")

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
        self._inItemChanged = True
        try:
            self._Apply(lambda: model.SetRowValue(row, text),
                        "Set %s" % row.key, retry=(row, text))
        finally:
            self._inItemChanged = False

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

    def _SelectedLayer(self):
        """The layer of the selected row or layer header, or None."""
        items = self._tree.selectedItems()
        if not items:
            return None
        row = items[0].data(COL_NAME, _ROW_ROLE)
        if row is not None:
            return row.layer
        group = items[0].data(0, _GROUP_ROLE)
        return group.layer if group is not None else None

    def _OnSaveSelected(self):
        layer = self._SelectedLayer()
        if layer is not None:
            self._SaveLayer(layer)

    def _SaveLayer(self, layer):
        """
        Save `layer` through pathSpelling, so every target, connection,
        inherit and specialize is written the way the tree shows it.
        """
        try:
            report = pathSpelling.SaveLayer(layer)
        except Exception as error:
            self._SetStatus("save failed: %s" % _FirstLine(str(error)),
                            "error", str(error))
            return
        self._SetStatus(str(report), "error" if report.note else "ok",
                        report.note)
        self._RequestRebuild()

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

    def _Apply(self, operation, label, retry=None):
        """
        Run one model operation, push its Edit, rebuild.

        A parse error is reported in the status line and leaves the
        stage untouched -- the model raises before it authors anything.
        When `retry` is (row, text), a refused edit reopens that row's
        editor on the refused text, so the fix is one keystroke away
        rather than a retype.
        """
        try:
            edit = operation()
        except model.ValueParseError as error:
            self._Refused(str(error), error.detail, retry)
            return
        except Tf.ErrorException as error:
            detail = str(error).strip()
            self._Refused(_FirstLine(detail), detail, retry)
            return
        if edit is not None and self._undo is not None:
            self._undo.Push(edit)
        self._SetStatus(label, "ok")
        self._RequestRebuild()

    def _Refused(self, message, detail, retry):
        self._SetStatus(message, "error", detail)
        self._reopen = retry
        self._RequestRebuild()

    def _ReopenRefused(self):
        """After a rebuild, reopen the editor a refused edit came from."""
        if self._reopen is None:
            return
        row, text = self._reopen
        self._reopen = None
        item = self._ItemFor(row)
        if item is None:
            return
        self._retry = (_RowIdentity(row), text)
        self._tree.setCurrentItem(item)
        self._tree.editItem(item, COL_VALUE)

    def TakeRetryText(self, row):
        """The refused text to reopen `row`'s editor on, once, or None."""
        if row is None or self._retry is None:
            return None
        identity, text = self._retry
        self._retry = None
        return text if identity == _RowIdentity(row) else None

    def _ItemFor(self, row):
        """The rebuilt tree's item for the row `row` was, or None."""
        identity = _RowIdentity(row)
        iterator = QtWidgets.QTreeWidgetItemIterator(self._tree)
        while iterator.value():
            item = iterator.value()
            candidate = item.data(COL_VALUE, _ROW_ROLE)
            if (candidate is not None
                    and _RowIdentity(candidate) == identity):
                return item
            iterator += 1
        return None

    def _ApplyFilter(self):
        """
        Hide the rows whose name and value both miss the filter text.
        A parent stays visible while any child matches, so a match inside
        `references` still shows under its heading and its layer.
        """
        needle = self._filter.text().strip().lower()

        def _Visit(item):
            own = (not needle
                   or needle in item.text(COL_NAME).lower()
                   or needle in item.text(COL_VALUE).lower())
            anyChild = False
            for i in range(item.childCount()):
                anyChild = _Visit(item.child(i)) or anyChild
            isGroup = item.data(0, _GROUP_ROLE) is not None
            visible = anyChild or (own and not (isGroup and needle))
            item.setHidden(not visible)
            if needle and anyChild:
                item.setExpanded(True)
            return visible

        for i in range(self._tree.topLevelItemCount()):
            _Visit(self._tree.topLevelItem(i))

    def _FillArcButtonMenu(self):
        self._arcMenu.clear()
        try:
            import compositionArcsUI
        except ImportError as error:
            self._SetStatus("composition arcs unavailable: %s" % error,
                            "error")
            return
        group = None
        items = self._tree.selectedItems()
        if items:
            row = items[0].data(COL_NAME, _ROW_ROLE)
            group = (self._GroupFor(row.layer) if row is not None
                     else items[0].data(0, _GROUP_ROLE))
        compositionArcsUI.PopulateArcMenu(
            self._arcMenu, self._api, self._SelectedPrim(),
            group.layer if group is not None else None,
            self._OnArcAuthored, self)

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
        layer = self._SelectedLayer()
        self._saveButton.setEnabled(layer is not None and _CanSave(layer))

    def _GroupFor(self, layer):
        for group in self._groups:
            if group.layer == layer:
                return group
        return None


def _CanSave(layer):
    """Whether `layer` has a file of its own it may be saved to."""
    return (not layer.anonymous and bool(layer.permissionToSave)
            and not layer.expired)


def _RowIdentity(row):
    """
    What makes a row the same row across a rebuild. The OpinionRow
    objects are rebuilt on every stage notice, so identity is by value.
    """
    return (row.layer.identifier, str(row.specPath), row.kind, row.key)


def _FirstLine(text):
    """The first non-empty line of a multi-line diagnostic."""
    for line in text.splitlines():
        if line.strip():
            return line.strip()
    return text


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
