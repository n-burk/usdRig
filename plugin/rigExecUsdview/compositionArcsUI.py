#
# RigExec usdview plugin: the guided composition-arc dialogs.
#
# One dialog class serves every arc kind, and serves both adding an arc
# and reopening one the panel already lists. It builds its form from the
# arc's Field descriptors in compositionArcsModel, so adding a field --
# or a whole arc -- is a change to that module and nothing here. All of
# the rules (what is legal, what merely warns, what usda comes out, how
# to undo it) live there and are tested headlessly
# (tests/python/test_composition_arcs_model.py); this file is a Qt
# driver over them, matching how layerOpinionsUI sits over
# layerOpinionsModel.
#
# EDITING IS THE SAME DIALOG. Whether the flow is adding or changing is
# carried by the context, and everything the dialog does -- which fields
# to show, what to prefill, what to refuse, what to preview -- it asks
# the arc for rather than deciding here. So a second form cannot drift
# from the first, and an arc kind that grows a field grows it in both.
#
# WHAT MAKES IT "GUIDED" rather than a form. Three things, and each one
# exists because composition is the part of USD people get wrong:
#
#   * every field carries the sentence explaining what it does, under
#     the field, not in a manual;
#   * the arc's own paragraph says what the arc IS and when to reach
#     for it, because "reference vs inherit vs specialize" is the real
#     question and no field label answers it;
#   * the usda that will be authored is shown, live, before committing
#     -- rendered by running the authoring code against a scratch layer,
#     so it cannot drift from what actually lands.
#
# Refusals disable the commit button and say why; warnings are shown and
# do not block, because "this asset does not exist yet" is a normal
# thing to author and a dialog that forbids it is wrong.
#
import difflib
import os

from pxr import Tf
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

import compositionArcsModel as model


def _Wrapped(label):
    """
    Let a word-wrapped QLabel actually compress to its container.

    A QLabel with wordWrap still reports its UNWRAPPED width as its
    minimum size hint inside a layout, so a long sentence silently
    widens the whole form and the ends of the explanations fall off the
    right edge. Asking for heightForWidth and dropping the minimum width
    is what makes the wrap real.
    """
    policy = label.sizePolicy()
    policy.setHeightForWidth(True)
    policy.setHorizontalPolicy(QtWidgets.QSizePolicy.Ignored)
    label.setSizePolicy(policy)
    label.setMinimumWidth(1)
    return label


def _Narrow(combo):
    """
    Stop a combo from sizing itself to its longest item.

    Both combos here hold long strings -- layer identifiers, and every
    prim path on the stage -- and a QComboBox's default size hint is the
    width of the widest one. Left alone, the prim-path combo alone makes
    the form wider than the dialog, and with horizontal scrolling off
    that clips the explanations rather than wrapping them.
    """
    combo.setSizeAdjustPolicy(
        QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
    combo.setMinimumContentsLength(16)
    combo.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                        combo.sizePolicy().verticalPolicy())
    return combo


def _SmallLabel(text):
    """The grey explanatory line that sits under a field."""
    label = _Wrapped(QtWidgets.QLabel(text))
    label.setWordWrap(True)
    font = label.font()
    # Point sizes come back as -1 when a font is set in pixels, and
    # setPointSize(-1 * 0.9) silently produces an unreadable widget.
    if font.pointSize() > 0:
        font.setPointSize(max(7, int(font.pointSize() * 0.9)))
    label.setFont(font)
    label.setForegroundRole(QtGui.QPalette.Mid)
    palette = label.palette()
    palette.setColor(QtGui.QPalette.WindowText,
                     palette.color(QtGui.QPalette.Mid))
    label.setPalette(palette)
    return label


class _AssetField(QtWidgets.QWidget):
    """
    A line edit with a Browse button, for asset paths.

    `anchorLayer` returns the layer being authored into. A browsed file
    is shown relative to it -- the path the arc will actually carry, see
    model.AnchoredAssetPath -- and the browser opens beside it.
    """

    def __init__(self, onChanged, anchorLayer, parent=None):
        super(_AssetField, self).__init__(parent)
        self._anchorLayer = anchorLayer
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._edit = QtWidgets.QLineEdit()
        # An asset path is long, but the field must still be allowed to
        # shrink: its default minimum is wide enough to push the whole
        # form past the dialog width on its own.
        self._edit.setMinimumWidth(80)
        self._edit.textChanged.connect(onChanged)
        layout.addWidget(self._edit, 1)
        browse = QtWidgets.QPushButton("Browse...")
        browse.setAutoDefault(False)
        browse.clicked.connect(self._Browse)
        layout.addWidget(browse)

    def _Browse(self):
        layer = self._anchorLayer()
        start = ""
        if layer is not None and not layer.anonymous and layer.realPath:
            start = os.path.dirname(layer.realPath)
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Choose a layer", start,
            "USD layers (*.usd *.usda *.usdc *.usdz);;All files (*)")
        if path:
            self._edit.setText(model.AnchoredAssetPath(layer, path))

    def text(self):
        return self._edit.text()

    def setText(self, text):
        self._edit.setText(text)


class _ChoiceField(QtWidgets.QWidget):
    """Radio buttons for a CHOICE field."""

    def __init__(self, choices, onChanged, parent=None):
        super(_ChoiceField, self).__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        self._group = QtWidgets.QButtonGroup(self)
        self._values = []
        for index, (value, label) in enumerate(choices):
            button = QtWidgets.QRadioButton(label)
            self._group.addButton(button, index)
            self._values.append(value)
            layout.addWidget(button)
        # Qt 5 and Qt 6 spell this signal differently; usdview ships
        # both depending on the build, so bind whichever exists rather
        # than picking one and breaking on the other.
        signal = getattr(self._group, "idToggled", None)
        if signal is None:
            signal = self._group.buttonToggled
        signal.connect(lambda *args: onChanged())

    def value(self):
        index = self._group.checkedId()
        return self._values[index] if index >= 0 else ""

    def setValue(self, value):
        if value in self._values:
            self._group.button(self._values.index(value)).setChecked(True)


class CompositionArcDialog(QtWidgets.QDialog):
    """
    One guided flow. Modal: the form previews against the live stage,
    and letting the selection move underneath it would make the preview
    describe a prim the dialog is no longer authoring on.
    """

    def __init__(self, arc, context, clickedLayer=None, parent=None):
        super(CompositionArcDialog, self).__init__(parent)
        self._arc = arc
        self._context = context
        self._values = dict(arc.Defaults(context))
        self._widgets = {}
        self._layers = model.AuthoringLayers(context.stage)
        self._building = False
        self._edit = None
        self._warnings = []

        self._editing = context.editing

        self.setWindowTitle(arc.Title(context))
        self.setModal(True)
        self.resize(780, 760)

        outer = QtWidgets.QVBoxLayout(self)

        # Layer-scoped arcs say so plainly: "Add Sublayer" on a panel
        # that is otherwise entirely about the selected prim is exactly
        # the confusion worth spending a line of the dialog on.
        if arc.scope == "prim":
            target = QtWidgets.QLabel(
                "On <b>%s</b>" % _Escape(context.primPath.name))
            target.setToolTip(str(context.primPath))
        else:
            target = QtWidgets.QLabel(
                "On the <b>layer</b> -- not on the selected prim")
        outer.addWidget(target)

        # An edit has no "Author into" combo, so the layer has to be
        # stated somewhere: the whole point of the panel is that an
        # opinion belongs to a layer, and a dialog that does not say
        # which one it is changing is asking to be used on the wrong one.
        if self._editing:
            row = context.editRow
            layer = row.layer
            where = _Wrapped(QtWidgets.QLabel(
                "Changing <b>%s</b> in <b>%s</b>. Its position among the "
                "other arcs is kept; use the panel to move it."
                % (_Escape(row.key),
                   _Escape(layer.GetDisplayName() or layer.identifier))))
            where.setWordWrap(True)
            where.setToolTip(layer.identifier)
            outer.addWidget(where)

        # The arc's own paragraph, before the fields: the choice between
        # arcs is the decision that needs help, and it is made before
        # any field is filled in.
        summary = _Wrapped(QtWidgets.QLabel(arc.summary))
        summary.setWordWrap(True)
        summary.setFrameShape(QtWidgets.QFrame.StyledPanel)
        summary.setMargin(8)
        outer.addWidget(summary)

        # Scrolled: the reference form with every explanation shown is
        # taller than a small laptop screen, and a dialog whose Author
        # button is off-screen cannot be used at all.
        self._formHost = QtWidgets.QWidget()
        self._formLayout = QtWidgets.QFormLayout(self._formHost)
        self._formLayout.setFieldGrowthPolicy(
            QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        # Horizontal scrolling off, or the word-wrapped help lines are
        # free to take their full unwrapped width and the form scrolls
        # sideways instead of wrapping -- which hides the ends of the
        # very sentences that make this a guided flow.
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setWidget(self._formHost)
        outer.addWidget(scroll, 1)

        outer.addWidget(QtWidgets.QLabel(
            "Will change to" if self._editing else "Will author"))
        self._preview = QtWidgets.QPlainTextEdit()
        self._preview.setReadOnly(True)
        self._preview.setFont(QtGui.QFontDatabase.systemFont(
            QtGui.QFontDatabase.FixedFont))
        self._preview.setMinimumHeight(120)
        outer.addWidget(self._preview, 1)

        self._message = _Wrapped(QtWidgets.QLabel(""))
        self._message.setWordWrap(True)
        outer.addWidget(self._message)

        self._buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Cancel)
        self._authorButton = self._buttons.addButton(
            "Apply" if self._editing else "Author",
            QtWidgets.QDialogButtonBox.AcceptRole)
        self._buttons.rejected.connect(self.reject)
        self._authorButton.clicked.connect(self._OnAuthor)
        outer.addWidget(self._buttons)

        self._SeedLayerDefault(clickedLayer)
        self._BuildForm()
        self._Refresh()

    # -- form --------------------------------------------------------

    def _SeedLayerDefault(self, clickedLayer):
        # An edit already knows its layer -- the one the arc is in --
        # and must not be re-aimed at the clicked group or the edit
        # target, which would write a copy somewhere else and leave the
        # original where it was.
        if self._editing:
            return
        self._values["layer"] = model.DefaultAuthoringLayer(
            self._context.stage, clickedLayer)

    def _BuildForm(self):
        """
        (Re)build the visible fields.

        Rebuilt rather than shown/hidden because a showIf field that is
        hidden must not keep contributing its value: a stale asset path
        left behind an "internal" choice would be authored.
        """
        self._building = True
        try:
            while self._formLayout.count():
                item = self._formLayout.takeAt(0)
                widget = item.widget()
                if widget is None:
                    continue
                # Signals blocked and deleteLater(), not a bare
                # setParent(None): this runs FROM a field's own signal
                # handler -- choosing "internal" rebuilds the form --
                # and unparenting drops the last reference to the widget
                # that is mid-emit, deleting its C++ object underneath
                # the emission. deleteLater keeps it alive until control
                # is back in the event loop.
                widget.blockSignals(True)
                widget.setParent(None)
                widget.deleteLater()
            self._widgets = {}
            visible = model.VisibleFields(
                self._arc, self._values, self._context)
            # A hidden field's value is DISCARDED, not merely unshown.
            # Typing an asset path and then choosing "internal" would
            # otherwise leave that path in the request, and validation
            # would refuse it ("an internal arc takes no asset path")
            # over a field the artist can no longer see or clear.
            keep = {f.key for f in visible}
            for field in self._arc.FieldsFor(self._context):
                if field.key not in keep and field.showIf is not None:
                    self._values[field.key] = field.default
            for field in visible:
                self._AddField(field)
        finally:
            self._building = False

    def _AddField(self, field):
        widget = self._MakeWidget(field)
        self._widgets[field.key] = widget
        label = QtWidgets.QLabel(field.label)
        label.setToolTip(field.help)
        widget.setToolTip(field.help)
        self._formLayout.addRow(label, widget)
        if field.help:
            # Spanning both columns (the one-argument addRow), not
            # parked in the field column: indented under the widget it
            # reads as a second, empty input rather than as a caption.
            self._formLayout.addRow(_SmallLabel(field.help))

    def _MakeWidget(self, field):
        onChanged = self._OnFieldChanged
        if field.kind == model.LAYER:
            combo = _Narrow(QtWidgets.QComboBox())
            editTarget = self._context.stage.GetEditTarget().GetLayer()
            for layer in self._layers:
                name = layer.GetDisplayName() or layer.identifier
                if layer == editTarget:
                    name += "   (usdview's edit target)"
                combo.addItem(name, layer.identifier)
                combo.setItemData(combo.count() - 1, layer.identifier,
                                  QtCore.Qt.ToolTipRole)
            current = self._values.get("layer")
            if current in self._layers:
                combo.setCurrentIndex(self._layers.index(current))
            combo.currentIndexChanged.connect(lambda *_: onChanged())
            return combo
        if field.kind == model.ASSET:
            widget = _AssetField(lambda *_: onChanged(),
                                 lambda: self._values.get("layer"))
            widget.setText(self._values.get(field.key) or "")
            return widget
        if field.kind == model.CHOICE:
            widget = _ChoiceField(field.choices, onChanged)
            widget.setValue(self._values.get(field.key) or field.default)
            return widget
        if field.kind == model.PRIM_PATH:
            combo = _Narrow(QtWidgets.QComboBox())
            combo.setEditable(True)
            # Which paths those are is the arc's decision, not this
            # file's: the target of an external reference lives in the
            # ASSET, while an inherit names a prim on this stage. See
            # _Arc.PathChoices.
            combo.addItems(self._PathChoices(field))
            combo.setCurrentText(self._values.get(field.key) or "")
            combo.currentTextChanged.connect(lambda *_: onChanged())
            return combo
        if field.kind == model.STRING_LIST:
            edit = QtWidgets.QPlainTextEdit()
            edit.setPlainText(self._values.get(field.key) or "")
            edit.setMaximumHeight(90)
            edit.textChanged.connect(onChanged)
            return edit
        edit = QtWidgets.QLineEdit()
        edit.setText(self._values.get(field.key) or "")
        edit.textChanged.connect(lambda *_: onChanged())
        return edit

    def _PathChoices(self, field):
        """
        The combo's items for one PRIM_PATH field: the arc's own
        candidates, behind the blank that means "the asset's
        defaultPrim".
        """
        return [""] + list(
            self._arc.PathChoices(self._context, self._values, field))

    def _ReseedPathCombos(self):
        """
        Refill the prim-path combos from the current values.

        They cannot be filled once when the form is built: the target
        prims of an external reference come from the asset, and which
        asset that is changes on every keystroke in the field above.

        Refilled in place rather than by rebuilding the form, because a
        rebuild would take the keyboard focus out of the field being
        typed into -- and with signals blocked, because addItems() emits
        currentTextChanged, which lands back in _Refresh.
        """
        for field in model.VisibleFields(self._arc, self._values,
                                         self._context):
            if field.kind != model.PRIM_PATH:
                continue
            combo = self._widgets.get(field.key)
            if combo is None:
                continue
            choices = self._PathChoices(field)
            if [combo.itemText(i) for i in range(combo.count())] == choices:
                continue
            typed = combo.currentText()
            blocked = combo.blockSignals(True)
            try:
                combo.clear()
                combo.addItems(choices)
                # clear() empties the line edit of an editable combo, so
                # what was typed has to be put back or the field wipes
                # itself the moment the asset above it changes.
                combo.setCurrentText(typed)
            finally:
                combo.blockSignals(blocked)

    def _ReadWidgets(self):
        for key, widget in self._widgets.items():
            if isinstance(widget, _AssetField):
                self._values[key] = widget.text()
            elif isinstance(widget, _ChoiceField):
                self._values[key] = widget.value()
            elif isinstance(widget, QtWidgets.QComboBox):
                if widget.isEditable():
                    self._values[key] = widget.currentText()
                else:
                    index = widget.currentIndex()
                    self._values[key] = (self._layers[index]
                                         if 0 <= index < len(self._layers)
                                         else None)
            elif isinstance(widget, QtWidgets.QPlainTextEdit):
                self._values[key] = widget.toPlainText()
            else:
                self._values[key] = widget.text()

    def _OnFieldChanged(self, *args):
        if self._building:
            return
        self._ReadWidgets()
        visible = [f.key for f in
                   model.VisibleFields(self._arc, self._values,
                                       self._context)]
        if visible != list(self._widgets.keys()):
            self._BuildForm()
        self._Refresh()

    # -- validation and preview --------------------------------------

    def _Refresh(self):
        """
        Re-run the arc's own checks and preview against the live values.

        Every failure path lands here rather than at Author time, so the
        Author button is only ever enabled on a request the model has
        already said it will accept.
        """
        self._ReseedPathCombos()
        self._warnings = []
        error = None
        try:
            self._warnings = self._arc.Validate(
                self._context, self._values)
        except model.ArcError as exc:
            error = str(exc)
        except Tf.ErrorException as exc:
            error = str(exc)

        if error is None:
            try:
                self._SetPreview(
                    self._arc.PreviewBase(self._context, self._values),
                    self._arc.Preview(self._context, self._values))
            except (model.ArcError, Tf.ErrorException) as exc:
                error = str(exc)
        if error is not None:
            self._preview.setPlainText("")
            self._preview.setExtraSelections([])

        self._authorButton.setEnabled(error is None)
        self._ShowMessage(error, self._warnings)

    def _SetPreview(self, base, authored):
        """
        Show the resulting usda with the lines the arc CHANGES
        highlighted, and scrolled into view.

        The exact text is kept -- the promise is that this is what lands
        -- but on a rig prim carrying dozens of opinions the one line
        that moved is otherwise invisible. The highlight is computed by
        diffing against the same spec without the arc applied, so it
        needs no cooperation from the authoring code, and it serves an
        edit as well as an add: an edit that changes nothing highlights
        nothing, which is the honest answer.
        """
        self._preview.setPlainText(authored)
        added = _AddedLines(base.split("\n"), authored.split("\n"))
        if not added:
            return

        colour = QtGui.QColor(_ADDED_LINE_COLOUR)
        selections = []
        document = self._preview.document()
        for index in added:
            block = document.findBlockByNumber(index)
            if not block.isValid():
                continue
            selection = QtWidgets.QTextEdit.ExtraSelection()
            selection.format.setBackground(colour)
            # FullWidthSelection paints the whole row, so a short line
            # still reads as a highlighted row rather than a stub.
            selection.format.setProperty(
                QtGui.QTextFormat.FullWidthSelection, True)
            selection.cursor = QtGui.QTextCursor(block)
            selection.cursor.clearSelection()
            selections.append(selection)
        self._preview.setExtraSelections(selections)

        cursor = QtGui.QTextCursor(document.findBlockByNumber(added[0]))
        self._preview.setTextCursor(cursor)
        self._preview.ensureCursorVisible()

    def _ShowMessage(self, error, warnings):
        if error:
            self._message.setText(
                "<span style='color:#c0392b'>%s</span>" % _Escape(error))
            return
        if warnings:
            self._message.setText(
                "<span style='color:#b9770e'>%s</span>"
                % "<br>".join(_Escape(w) for w in warnings))
            return
        self._message.setText("")

    # -- commit ------------------------------------------------------

    def _OnAuthor(self):
        self._ReadWidgets()
        try:
            self._edit, self._warnings = self._arc.Author(
                self._context, self._values)
        except model.ArcError as exc:
            self._ShowMessage(str(exc), [])
            self._authorButton.setEnabled(False)
            return
        except Tf.ErrorException as exc:
            self._ShowMessage(str(exc), [])
            return
        self.accept()

    def Result(self):
        """(Edit, warnings) after an accepted dialog; (None, []) if not."""
        return self._edit, list(self._warnings)


# Translucent so it tints whatever the usdview stylesheet paints behind
# it, rather than replacing the text background with a colour that is
# unreadable in one of the two themes.
_ADDED_LINE_COLOUR = QtGui.QColor(90, 160, 90, 90)


def _AddedLines(base, authored):
    """
    Indices, in `authored`, of the lines `base` does not have.

    difflib rather than a hand-rolled LCS: this runs on every keystroke
    in the dialog, and a full LCS table over an exported prim spec is
    quadratic in both time and memory -- a few thousand lines is already
    tens of MiB and enough to freeze the form. SequenceMatcher is linear
    on the near-identical inputs this always gets.
    """
    matcher = difflib.SequenceMatcher(a=base, b=authored, autojunk=False)
    added = []
    for tag, _, _, j1, j2 in matcher.get_opcodes():
        if tag in ("insert", "replace"):
            added.extend(range(j1, j2))
    return added


def _Escape(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def RunArcEditFlow(usdviewApi, row, prim, parent=None):
    """
    Reopen the arc `row` holds in its own guided flow, prefilled.
    Returns (Edit, warnings), or (None, []) if cancelled or unavailable.

    The panel decides which rows to offer this on; this function still
    checks, because "can this arc be reopened" is a rule and a menu is a
    convenience.
    """
    arc = model.ArcForRow(row)
    stage = usdviewApi.stage
    if arc is None or stage is None:
        return None, []
    if row.layer not in model.AuthoringLayers(stage):
        QtWidgets.QMessageBox.warning(
            parent, arc.title.replace("Add ", "Edit ", 1),
            "%s is not a layer of this stage's local layer stack, or "
            "cannot be edited, so its composition is not this stage's to "
            "change." % (row.layer.GetDisplayName() or row.layer.identifier))
        return None, []
    context = model.ArcContext(
        stage,
        prim if prim is not None and prim.IsValid()
        else stage.GetPseudoRoot(),
        row)
    dialog = CompositionArcDialog(arc, context, None, parent)
    try:
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return None, []
        return dialog.Result()
    finally:
        dialog.deleteLater()


def RunArcFlow(arc, usdviewApi, prim, clickedLayer=None, parent=None):
    """
    Open `arc`'s guided flow. Returns (Edit, warnings), or (None, [])
    if it was cancelled or could not be opened.

    The caller pushes the Edit onto the shared undo stack and rebuilds;
    this function deliberately does neither, so the panel stays the one
    place that knows about the stack.
    """
    stage = usdviewApi.stage
    if stage is None:
        return None, []
    if not model.AuthoringLayers(stage):
        QtWidgets.QMessageBox.warning(
            parent, arc.title,
            "No layer of this stage can be edited, so there is nowhere to "
            "author an arc.")
        return None, []
    if arc.scope == "prim" and (prim is None or not prim.IsValid()):
        QtWidgets.QMessageBox.warning(
            parent, arc.title,
            "Select a prim first: %s is authored on a prim." % arc.title)
        return None, []

    # The layer-scoped arcs still want a prim for context (a relocate
    # prefills from the selection), and the pseudo-root serves when
    # there is no selection at all.
    context = model.ArcContext(
        stage, prim if prim is not None and prim.IsValid()
        else stage.GetPseudoRoot())
    dialog = CompositionArcDialog(arc, context, clickedLayer, parent)
    try:
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return None, []
        return dialog.Result()
    finally:
        # Parented to the panel, so without this every flow ever opened
        # is retained for the session -- along with its captured stage,
        # prim and, on a successful edit, the whole snapshot.
        dialog.deleteLater()


def PopulateArcMenu(menu, usdviewApi, prim, clickedLayer, onAuthored,
                    parent=None):
    """
    Fill `menu` with one item per arc kind.

    `onAuthored(edit, warnings, label)` is called after a flow commits,
    so the panel can push the Edit and rebuild. Split out of the panel
    so the same submenu can be raised from anywhere in the tree --
    including the empty space below the rows, where there is no item to
    hang a menu off but the selected prim is still what you mean.
    """
    # Qt hides action tooltips in menus unless this is set, so without
    # it the per-arc hover help written below is never seen.
    menu.setToolTipsVisible(True)
    hasPrim = prim is not None and prim.IsValid() and not prim.IsPseudoRoot()
    for arc in model.ARC_KINDS:
        action = menu.addAction(arc.label)
        action.setToolTip(arc.summary)
        action.setEnabled(hasPrim or arc.scope == "layer")
        action.triggered.connect(
            _ArcTrigger(arc, usdviewApi, prim, clickedLayer, onAuthored,
                        parent))
    return menu


class _ArcTrigger(object):
    """
    A callable holding one menu item's arc.

    A class rather than a lambda in the loop: a lambda would capture the
    loop variable by reference and every item would open the last arc.
    """

    def __init__(self, arc, usdviewApi, prim, clickedLayer, onAuthored,
                 parent):
        self._arc = arc
        self._api = usdviewApi
        self._prim = prim
        self._layer = clickedLayer
        self._onAuthored = onAuthored
        self._parent = parent

    def __call__(self, *args):
        edit, warnings = RunArcFlow(
            self._arc, self._api, self._prim, self._layer, self._parent)
        if edit is not None:
            self._onAuthored(edit, warnings, edit.label)
