#
# THE GUIDED COMPOSITION-ARC FLOWS, end to end.
#
# tests/python/test_composition_arcs_model.py covers every authoring
# rule against in-memory layers and never touches a widget. What it
# cannot see is the half this script exists for: that the Layer
# Opinions panel's right-click menu actually offers the arcs, that the
# dialog builds a form from the arc's fields and re-previews as they
# change, that a refusal disables the Author button instead of throwing
# on commit, and that authoring through the dialog reaches the stage and
# lands on the panel's shared undo stack.
#
# The second half of the script is the same for EDITING an arc that is
# already there: that the panel lists a reference as a row of its own
# under the field, that its menu offers the flow, the removal and the
# two moves, and that reopening it prefills the dialog with the arc it
# holds -- with no layer or position field, because neither is a
# question once the arc exists.
#
# Opened on examples/ArmRig.usda.
#
# Set RIGEXEC_ARCS_SHOT=/path.png to save a grab of the reference flow.
#
import os

from pxr import Sdf

IK = "/ArmAsset/Rig/Solvers/IK"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _WalkItems(item):
    """Every item under `item`, at any depth."""
    for i in range(item.childCount()):
        child = item.child(i)
        yield child
        for deeper in _WalkItems(child):
            yield deeper


def _PanelRow(panel, layer, key):
    """
    The (item, row) the panel is showing for one opinion of one layer.

    Keyed by layer as well as by name: an arc row is called
    "prepend references [0]" in every layer that has one, and picking
    whichever came first would silently test the wrong layer.
    """
    import layerOpinionsUI
    tree = panel._tree
    for i in range(tree.topLevelItemCount()):
        for item in _WalkItems(tree.topLevelItem(i)):
            row = item.data(layerOpinionsUI.COL_NAME,
                            layerOpinionsUI._ROW_ROLE)
            if row is not None and row.layer == layer and row.key == key:
                return item, row
    return None, None


def _MenuLabels(menu):
    return [str(a.text()) for a in menu.actions() if not a.isSeparator()]


def _FindAction(menu, text):
    for action in menu.actions():
        if str(action.text()) == text:
            return action
    return None


def _FindSubmenu(menu, title):
    from pxr.Usdviewq.qt import QtWidgets
    for action in menu.actions():
        sub = action.menu()
        if sub is not None and str(action.text()) == title:
            return sub
    return None


def _AssetLayer(tmpdir):
    """
    A tiny asset on DISK, not an anonymous layer.

    The dialog resolves the asset path relative to the layer being
    authored into, and an anonymous identifier would exercise a code
    path no artist ever takes.
    """
    path = os.path.join(tmpdir, "arcTestAsset.usda")
    layer = Sdf.Layer.CreateNew(path)
    layer.ImportFromString('''#usda 1.0
(
    defaultPrim = "Grip"
)

def Xform "Grip"
{
    double gripAmount = 0.75

    def Xform "Pad"
    {
    }
}
''')
    layer.Save()
    return path


def testUsdviewInputFunction(appController):
    import compositionArcsModel as model
    import compositionArcsUI
    import layerOpinionsUI
    from pxr.Usdviewq.qt import QtWidgets

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    tmpdir = os.path.dirname(stage.GetRootLayer().realPath)

    prim = stage.GetPrimAtPath(IK)
    _Check(bool(prim), "ArmRig has %s" % IK)
    appController._dataModel.selection.setPrim(prim)
    appController._processEvents()

    panel = layerOpinionsUI.OpenLayerOpinionsPanel(api, None)
    appController._processEvents()

    # --- 1. the right-click menu offers every arc -----------------------
    menu = QtWidgets.QMenu(panel)
    panel._AddArcMenu(menu, None)
    submenu = _FindSubmenu(menu, "Add Composition Arc")
    _Check(submenu is not None,
           "the context menu has an Add Composition Arc submenu: %s"
           % [str(a.text()) for a in menu.actions()])
    labels = [str(a.text()) for a in submenu.actions()]
    for arc in model.ARC_KINDS:
        _Check(arc.label in labels,
               "the submenu offers %s: %s" % (arc.label, labels))
    _Check(all(a.isEnabled() for a in submenu.actions()),
           "with a prim selected, every arc is reachable")
    for action in submenu.actions():
        _Check(str(action.toolTip()),
               "%s explains itself on hover" % str(action.text()))

    # --- 2. the dialog builds itself from the arc's fields --------------
    context = model.ArcContext(stage, prim)
    root = stage.GetRootLayer()
    dialog = compositionArcsUI.CompositionArcDialog(
        model.ReferenceArc, context, root, panel)
    appController._processEvents()

    _Check(set(dialog._widgets.keys())
           == {f.key for f in model.VisibleFields(
               model.ReferenceArc, dialog._values)},
           "the form has exactly the visible fields: %s"
           % sorted(dialog._widgets.keys()))
    _Check(dialog._values["layer"] == root,
           "it opened on the layer whose group was right-clicked")

    # --- 3. showIf: the internal choice drops the asset field -----------
    _Check("assetPath" in dialog._widgets,
           "an external reference asks for a file")
    dialog._widgets["source"].setValue("internal")
    appController._processEvents()
    _Check("assetPath" not in dialog._widgets,
           "choosing internal rebuilt the form without it: %s"
           % sorted(dialog._widgets.keys()))
    _Check("primPath" in dialog._widgets,
           "but still asks which prim")

    # --- 4. a refusal disables Author and says why ----------------------
    dialog._widgets["primPath"].setCurrentText(IK)
    appController._processEvents()
    _Check(not dialog._authorButton.isEnabled(),
           "a prim referencing itself cannot be authored")
    _Check("itself" in str(dialog._message.text()),
           "and the dialog says why: %r" % str(dialog._message.text()))
    _Check(not str(dialog._preview.toPlainText()),
           "with no preview standing next to a refusal")

    # --- 4b. the target prims come from the ASSET ----------------------
    # Seeded from the stage, this combo offers the one list that cannot
    # contain the answer -- and on a shot that has not been assembled it
    # is a single root prim. It has to refill as the asset is typed,
    # which no other field in the dialog does.
    dialog._widgets["source"].setValue("external")
    appController._processEvents()
    combo = dialog._widgets["primPath"]
    stagePaths = [str(combo.itemText(i)) for i in range(combo.count())]
    _Check(IK in stagePaths,
           "with no asset named, the stage's prims are the best guess: %s"
           % stagePaths[:6])

    assetPath = _AssetLayer(tmpdir)
    dialog._widgets["assetPath"].setText(assetPath)
    appController._processEvents()
    combo = dialog._widgets["primPath"]
    assetPaths = [str(combo.itemText(i)) for i in range(combo.count())]
    _Check("/Grip" in assetPaths and "/Grip/Pad" in assetPaths,
           "naming an asset refills the combo from it: %s" % assetPaths)
    _Check(IK not in assetPaths,
           "and the stage's own prims are gone: %s" % assetPaths)
    _Check(assetPaths[:2] == ["", "/Grip"],
           "with the blank first and then the defaultPrim, which is what "
           "the blank means: %s" % assetPaths)

    # Refilled in place: a rebuild would take the focus out of the field
    # being typed into, and clear() empties an editable combo's line
    # edit, so what was typed has to survive the refill.
    combo.setCurrentText("/Grip/Pad")
    appController._processEvents()
    dialog._widgets["assetPath"].setText(assetPath + "x")
    appController._processEvents()
    _Check(str(dialog._widgets["primPath"].currentText()) == "/Grip/Pad",
           "a typed target survives the asset changing under it: %r"
           % str(dialog._widgets["primPath"].currentText()))
    _Check(dialog._widgets["primPath"].count() == 1,
           "an asset that does not resolve offers nothing rather than "
           "pretending the stage's prims are its contents")

    # --- 5. a legal request previews and re-enables ---------------------
    dialog._widgets["assetPath"].setText(assetPath)
    dialog._widgets["primPath"].setCurrentText("")
    appController._processEvents()
    _Check(dialog._authorButton.isEnabled(),
           "a resolvable asset with a defaultPrim is authorable: %r"
           % str(dialog._message.text()))
    preview = str(dialog._preview.toPlainText())
    _Check("prepend references" in preview,
           "and the preview shows the arc:\n%s" % preview)
    _Check("rigExec:preferredBendRadians" in preview,
           "seeded from the prim's real spec in that layer:\n%s" % preview)

    shot = os.environ.get("RIGEXEC_ARCS_SHOT")
    if shot:
        # Shown before the grab: an unshown dialog has not laid out, so
        # resize() alone leaves it at its size hint and the picture does
        # not show what an artist sees.
        dialog.show()
        dialog.resize(760, 820)
        appController._processEvents()
        dialog.grab().save(shot)
        dialog.hide()

    # --- 6. the preview updates as a field changes ----------------------
    dialog._widgets["position"].setValue("append")
    appController._processEvents()
    _Check("append references" in str(dialog._preview.toPlainText()),
           "changing the position re-previewed:\n%s"
           % str(dialog._preview.toPlainText()))
    dialog._widgets["position"].setValue("prepend")
    appController._processEvents()

    # --- 7. authoring reaches the stage, and undo takes it back ---------
    _Check(not stage.GetPrimAtPath(IK + "/Pad"),
           "the asset is not composed yet")
    dialog._OnAuthor()
    appController._processEvents()
    edit, warnings = dialog.Result()
    _Check(edit is not None, "the dialog produced an undoable Edit")
    _Check(not warnings, "on a clean request: %s" % warnings)
    _Check(bool(stage.GetPrimAtPath(IK + "/Pad")),
           "the reference composed the asset in")
    _Check(root.GetPrimAtPath(IK).HasInfo("references"),
           "and landed in the layer the dialog named")

    edit.Undo()
    appController._processEvents()
    _Check(not stage.GetPrimAtPath(IK + "/Pad"), "undo removed it")
    _Check(bool(stage.GetPrimAtPath(IK).GetAttribute(
        "rigExec:preferredBendRadians")),
        "and left the prim's own opinions alone")

    # --- 8. the panel's own path: push onto the shared stack, rebuild ---
    # Everything above drove the dialog directly. This is what the menu
    # item does when a flow commits, and it is the only place the undo
    # stack and the rebuild are wired together.
    import rigExecUndo
    stack = rigExecUndo.UndoStack()
    panel._undo = stack
    edit, _ = model.ReferenceArc.Author(context, {
        "layer": root, "source": "external", "assetPath": assetPath,
        "primPath": "", "offset": "0", "scale": "1", "position": "prepend"})
    panel._OnArcAuthored(edit, [], edit.label)
    appController._processEvents()
    _Check(stack.CanUndo() and stack.UndoText() == "Add reference",
           "the flow's Edit is on the panel's stack: %r" % stack.UndoText())
    _Check("Add reference" in str(panel._status.text()),
           "and the status line names it: %r" % str(panel._status.text()))

    # The arc now shows in the panel as an opinion row of its own,
    # because layerOpinionsModel reads arcs as ordinary info fields.
    rows = []
    tree = panel._tree
    for i in range(tree.topLevelItemCount()):
        groupItem = tree.topLevelItem(i)
        for j in range(groupItem.childCount()):
            row = groupItem.child(j).data(layerOpinionsUI.COL_NAME,
                                          layerOpinionsUI._ROW_ROLE)
            if row is not None:
                rows.append(row.key)
    _Check("references" in rows,
           "the rebuilt panel lists the new arc: %s" % sorted(set(rows)))

    _Check(stack.Undo(), "the stack undoes it")
    appController._processEvents()
    _Check(not stage.GetPrimAtPath(IK + "/Pad"), "and the stage follows")

    # --- 9. warnings are reported without blocking ----------------------
    dialog = compositionArcsUI.CompositionArcDialog(
        model.ReferenceArc, context, root, panel)
    dialog._widgets["assetPath"].setText("./nothing-here.usda")
    appController._processEvents()
    _Check(dialog._authorButton.isEnabled(),
           "an unresolvable asset is still authorable -- it may not exist "
           "yet: %r" % str(dialog._message.text()))
    _Check("resolve" in str(dialog._message.text()),
           "but the dialog warns: %r" % str(dialog._message.text()))

    # --- 10. the MENU path, which is how it is actually reached ---------
    # Everything above constructed dialogs directly. This is the route an
    # artist takes: the action's callback routes through _ArcTrigger, so
    # a lambda capturing the loop variable -- every item opening the last
    # arc -- shows up here and nowhere else. RunArcFlow is stubbed
    # because the real one is modal and would block the test.
    opened = []
    realRunArcFlow = compositionArcsUI.RunArcFlow
    compositionArcsUI.RunArcFlow = (
        lambda arc, api, p, layer=None, parent=None:
        (opened.append((arc, layer)), (None, []))[1])
    try:
        menu = QtWidgets.QMenu(panel)
        panel._AddArcMenu(menu, None)
        submenu = _FindSubmenu(menu, "Add Composition Arc")
        for action in submenu.actions():
            action.trigger()
        appController._processEvents()
    finally:
        compositionArcsUI.RunArcFlow = realRunArcFlow
    _Check([arc for arc, _ in opened] == list(model.ARC_KINDS),
           "each item opened its OWN arc, in menu order: %s"
           % [arc.key for arc, _ in opened])

    # And a right-click on a layer group aims the flow at that layer.
    opened[:] = []
    compositionArcsUI.RunArcFlow = (
        lambda arc, api, p, layer=None, parent=None:
        (opened.append((arc, layer)), (None, []))[1])
    try:
        menu = QtWidgets.QMenu(panel)
        panel._AddArcMenu(menu, panel._groups[0])
        submenu = _FindSubmenu(menu, "Add Composition Arc")
        submenu.actions()[0].trigger()
        appController._processEvents()
    finally:
        compositionArcsUI.RunArcFlow = realRunArcFlow
    _Check(opened and opened[0][1] == panel._groups[0].layer,
           "the clicked layer group is handed to the flow: %s" % opened)

    # --- 11. a layer-scoped arc says it is not about the prim -----------
    dialog = compositionArcsUI.CompositionArcDialog(
        model.SublayerArc, context, root, panel)
    appController._processEvents()
    _Check("layer" in dialog._values,
           "the sublayer flow asks which layer")
    _Check("primPath" not in dialog._widgets,
           "and does not ask about the prim: %s"
           % sorted(dialog._widgets.keys()))

    # --- 12. an arc already there is a row of its own -------------------
    # Everything above added arcs. From here the subject is the arcs the
    # panel is SHOWING: one row per reference under the field, which is
    # what makes editing, removing and moving one of several possible at
    # all.
    edit, _ = model.ReferenceArc.Author(context, {
        "layer": root, "source": "external", "assetPath": assetPath,
        "primPath": "", "offset": "5", "scale": "2", "position": "prepend"})
    panel.Rebuild()
    appController._processEvents()

    item, arcRow = _PanelRow(panel, root, "prepend references [0]")
    _Check(arcRow is not None,
           "the reference is listed as an arc row of its own")
    _Check(arcRow.valueText
           == "@%s@ (offset = 5; scale = 2)" % assetPath,
           "shown as usda writes it, offset included: %r"
           % arcRow.valueText)
    _Check(str(item.text(layerOpinionsUI.COL_KIND)) == "arc",
           "and the kind column says arc, not metadata")
    parentItem, parentRow = _PanelRow(panel, root, "references")
    _Check(parentRow is not None and parentRow.children,
           "the field itself is the heading above it")
    _Check(parentItem.isExpanded(),
           "which opens on its own -- the arcs under it are the point")

    # --- 13. the row's menu offers the flow, the removal and the moves --
    menu = QtWidgets.QMenu(panel)
    panel._AddRowActions(menu, item, arcRow)
    labels = _MenuLabels(menu)
    _Check("Edit Reference..." in labels,
           "the arc's own flow is offered by name: %s" % labels)
    _Check("Remove This Entry" in labels,
           "removal names what it removes -- one arc, not the field: %s"
           % labels)
    _Check("Move Stronger" not in labels,
           "with one arc there is nowhere to move it: %s" % labels)
    _Check(_FindAction(menu, "Remove This Entry").isEnabled(),
           "and removal is available in a local layer")

    # A second arc makes the moves meaningful.
    second, _ = model.ReferenceArc.Author(context, {
        "layer": root, "source": "external", "assetPath": assetPath,
        "primPath": "/Grip/Pad", "offset": "0", "scale": "1",
        "position": "prepend"})
    panel.Rebuild()
    appController._processEvents()
    item, arcRow = _PanelRow(panel, root, "prepend references [0]")
    menu = QtWidgets.QMenu(panel)
    panel._AddRowActions(menu, item, arcRow)
    _Check("Move Weaker" in _MenuLabels(menu),
           "now the stronger arc can be demoted: %s" % _MenuLabels(menu))
    _Check(not _FindAction(menu, "Move Stronger").isEnabled(),
           "but the strongest one still cannot be promoted")
    _FindAction(menu, "Move Weaker").trigger()
    appController._processEvents()
    _Check(list(root.GetPrimAtPath(IK).referenceList.prependedItems)[0]
           .primPath == Sdf.Path(),
           "the moved arc swapped with the one below it: %s"
           % list(root.GetPrimAtPath(IK).referenceList.prependedItems))
    second.Undo()
    panel.Rebuild()
    appController._processEvents()

    # --- 14. reopening the arc prefills the dialog ----------------------
    item, arcRow = _PanelRow(panel, root, "prepend references [0]")
    editContext = model.ArcContext(stage, prim, arcRow)
    dialog = compositionArcsUI.CompositionArcDialog(
        model.ReferenceArc, editContext, None, panel)
    appController._processEvents()

    _Check(str(dialog.windowTitle()) == "Edit Reference",
           "the dialog says it is editing: %r" % str(dialog.windowTitle()))
    _Check(str(dialog._authorButton.text()) == "Apply",
           "and its button says Apply, not Author")
    _Check("layer" not in dialog._widgets,
           "no layer choice: the arc is in the layer it is in: %s"
           % sorted(dialog._widgets.keys()))
    _Check("position" not in dialog._widgets,
           "and no position choice: retyping must not re-rank it")
    _Check(str(dialog._widgets["assetPath"].text()) == assetPath,
           "the asset is prefilled: %r"
           % str(dialog._widgets["assetPath"].text()))
    _Check(str(dialog._widgets["offset"].text()) == "5"
           and str(dialog._widgets["scale"].text()) == "2",
           "and so is the layer offset")
    _Check(dialog._authorButton.isEnabled(),
           "reopening an arc that is already legal is legal: %r"
           % str(dialog._message.text()))

    # --- 15. applying changes the arc where it sits ---------------------
    dialog._widgets["primPath"].setCurrentText("/Grip/Pad")
    appController._processEvents()
    _Check("/Grip/Pad" in str(dialog._preview.toPlainText()),
           "the preview follows the edit:\n%s"
           % str(dialog._preview.toPlainText()))
    dialog._OnAuthor()
    appController._processEvents()
    applied, warnings = dialog.Result()
    _Check(applied is not None, "the edit produced an undoable Edit")
    _Check(applied.label == "Edit reference",
           "labelled as an edit: %r" % applied.label)

    references = list(root.GetPrimAtPath(IK).referenceList.prependedItems)
    _Check(len(references) == 1,
           "the arc was replaced, not added beside itself: %s" % references)
    _Check(references[0].primPath == Sdf.Path("/Grip/Pad"),
           "with the new target: %s" % references[0])
    _Check(references[0].layerOffset == Sdf.LayerOffset(5, 2),
           "and the offset that was not touched")
    applied.Undo()
    appController._processEvents()
    _Check(list(root.GetPrimAtPath(IK).referenceList.prependedItems)[0]
           .primPath == Sdf.Path(),
           "undo restores the arc as it was")

    # --- 16. the panel's route into the edit flow -----------------------
    # As with the add menu, the real flow is modal, so it is stubbed and
    # what is checked is that the row reaches it.
    reopened = []
    realEditFlow = compositionArcsUI.RunArcEditFlow
    compositionArcsUI.RunArcEditFlow = (
        lambda api, r, p, parent=None:
        (reopened.append(r), (None, []))[1])
    try:
        panel.Rebuild()
        appController._processEvents()
        item, arcRow = _PanelRow(panel, root, "prepend references [0]")
        menu = QtWidgets.QMenu(panel)
        panel._AddRowActions(menu, item, arcRow)
        _FindAction(menu, "Edit Reference...").trigger()
        appController._processEvents()
    finally:
        compositionArcsUI.RunArcEditFlow = realEditFlow
    _Check(reopened and reopened[0] is arcRow,
           "the clicked arc row is what the flow is opened on: %s"
           % reopened)

    # --- 17. an arc can also be retyped in the tree ---------------------
    # The inline editor is the fast path, and goes through the same
    # writes as the dialog: the row is addressed by its arm and index
    # either way.
    item, arcRow = _PanelRow(panel, root, "prepend references [0]")
    _Check(arcRow.editable, "an arc in a local layer is editable inline")
    item.setText(layerOpinionsUI.COL_VALUE,
                 "@%s@</Grip> (offset = 1)" % assetPath)
    appController._processEvents()
    references = list(root.GetPrimAtPath(IK).referenceList.prependedItems)
    _Check(references[0].primPath == Sdf.Path("/Grip")
           and references[0].layerOffset == Sdf.LayerOffset(1),
           "the typed usda was parsed and authored: %s" % references)

    panel.Rebuild()
    appController._processEvents()
    item, arcRow = _PanelRow(panel, root, "prepend references [0]")
    item.setText(layerOpinionsUI.COL_VALUE, "not a reference")
    appController._processEvents()
    _Check("not a" in str(panel._status.text()),
           "a typo is a parse error in the status line: %r"
           % str(panel._status.text()))
    _Check(list(root.GetPrimAtPath(IK).referenceList.prependedItems)[0]
           .primPath == Sdf.Path("/Grip"),
           "and the arc is left exactly as it was")

    # --- 18. a layer's own sublayers are listed under its group ---------
    # They are the LAYER's composition rather than the prim's, and this
    # panel is the only place they can be seen or changed.
    _Check(_PanelRow(panel, root, "subLayers")[1] is None,
           "a layer with no sublayers grows no heading for them")
    # An UNRESOLVABLE path on purpose. The point here is the row, not
    # the composition, and a sublayer that really composes recomposes
    # the pseudo-root -- which this USD build asserts on from inside
    # esfUsd, taking the test with it for a reason that has nothing to
    # do with the panel. The "could not load sublayer" warning on the
    # console is this line, and is expected.
    root.subLayerPaths.append("./nothing-here.usda")
    panel.Rebuild()
    appController._processEvents()
    subItem, subRow = _PanelRow(panel, root, "subLayer [0]")
    _Check(subRow is not None,
           "adding one puts it under the layer that carries it")
    _Check(subRow.valueText == "@./nothing-here.usda@",
           "shown as usda writes it: %r" % subRow.valueText)
    menu = QtWidgets.QMenu(panel)
    panel._AddRowActions(menu, subItem, subRow)
    _Check("Edit Sublayer..." in _MenuLabels(menu),
           "with the sublayer flow on its menu: %s" % _MenuLabels(menu))
    _FindAction(menu, "Remove This Entry").trigger()
    appController._processEvents()
    _Check(not list(root.subLayerPaths),
           "and removal takes out that entry: %s" % list(root.subLayerPaths))

    print("RIGEXEC_COMPOSITION_ARCS_OK menu, form, showIf, refusal, "
          "target prims from the asset, preview, author, undo, panel "
          "wiring, warning, menu routing, layer scope, arc rows, row "
          "menu, move, edit prefill, edit apply, edit routing, inline "
          "retype, sublayer rows")
