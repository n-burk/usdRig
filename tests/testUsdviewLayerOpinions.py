#
# THE LAYER OPINIONS PANEL, end to end.
#
# tests/python/test_layer_opinions_model.py covers every rule in
# layerOpinionsModel against in-memory layers and never touches a
# widget. What it cannot see is the half this script exists for: that
# the container actually registers the menu item, that the panel builds
# a tree from usdview's own selection, and that a value typed into a
# row lands on the layer that row names.
#
# Opened on examples/ArmRig.usda, whose IK carries opinions in the file
# layer while the session layer stays empty -- so the panel has both a
# real layer group and (after one edit) the edit target's own.
#
# Set RIGEXEC_OPINIONS_SHOT=/path.png to save a window grab.
#
import os

from pxr import Sdf

IK = "/ArmAsset/Rig/Solvers/IK"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RowItems(panel):
    """
    Every (layerIdentifier, key, valueText, row) the tree is showing.

    Reads the panel's own role constants rather than literals: Qt's
    UserRole is 256, so a hand-written number is both wrong and silently
    wrong -- itemData returns None and the tree looks empty.
    """
    import layerOpinionsUI
    out = []
    tree = panel._tree
    for i in range(tree.topLevelItemCount()):
        groupItem = tree.topLevelItem(i)
        for j in range(groupItem.childCount()):
            child = groupItem.child(j)
            row = child.data(layerOpinionsUI.COL_NAME,
                             layerOpinionsUI._ROW_ROLE)
            if row is not None:
                out.append((row.layer.identifier, row.key,
                            child.text(layerOpinionsUI.COL_VALUE), row))
    return out


def testUsdviewInputFunction(appController):
    import layerOpinionsUI
    import layerOpinionsModel as model

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage

    # --- 1. the container registered the menu item ----------------------
    reg = appController._plugRegistry
    _Check(reg.getCommandPlugin(
        "RigExecUsdviewContainer.layerOpinions") is not None,
        "RigExec -> Layer Opinions command is not registered")
    from pxr.Usdviewq.qt import QtWidgets
    menus = [c for c in appController._mainWindow.menuBar().children()
             if isinstance(c, QtWidgets.QMenu)
             and str(c.title()).replace("&", "") == "RigExec"]
    menus = [a.menu() for a in (menus[0].actions() if len(menus) == 1
                                else [])
             if a.menu() is not None and a.text() == "General Editors"]
    _Check(len(menus) == 1 and "Layer Opinions" in
           [a.text() for a in menus[0].actions()],
           "RigExec -> General Editors has no Layer Opinions item")

    # --- 2. it builds from usdview's selection --------------------------
    prim = stage.GetPrimAtPath(IK)
    _Check(prim, "ArmRig has %s" % IK)
    # setPrim, not ClearPrimSelection + AddPrimToSelection: clearing
    # leaves the pseudo-root in the path list, so the FOCUS prim (which
    # is getPrimPaths()[0], and is what usdviewApi.prim returns) would
    # stay '/' and the panel would correctly show nothing.
    appController._dataModel.selection.setPrim(prim)
    appController._processEvents()

    panel = layerOpinionsUI.OpenLayerOpinionsPanel(api, None)
    panel.resize(900, 600)
    appController._processEvents()

    _Check(panel._header.text() == IK,
           "the panel follows the selection: %r" % panel._header.text())
    _Check(panel._groups, "the IK has opinions to show")

    rows = _RowItems(panel)
    keys = {key for (_, key, _, _) in rows}
    for expected in ("rigExec:preferredBendRadians", "rigExec:joints",
                     "inputs:softness", "specifier", "typeName"):
        _Check(expected in keys,
               "the tree shows %r: %s" % (expected, sorted(keys)))

    # SOMETHING must be unfolded. usdview's edit target is the session
    # layer, which holds no opinion for this prim and so contributes no
    # group -- if only the edit target's group opened, the panel would
    # come up looking empty on the most ordinary stage there is.
    top = panel._tree.topLevelItem(0)
    _Check(top is not None, "there is a layer group")
    _Check(top.isExpanded(),
           "the strongest group is unfolded when the edit target has no "
           "opinions of its own")

    # The IK's own layer is the ArmRig file, not the session layer.
    fileLayer = stage.GetRootLayer()
    _Check(any(ident == fileLayer.identifier for (ident, _, _, _) in rows),
           "the file layer's opinions are listed")

    # --- 3. the selection driving a rebuild -----------------------------
    other = stage.GetPrimAtPath("/ArmAsset/Rig/Solvers/IKFKBlend")
    _Check(other, "ArmRig has the blend")
    appController._dataModel.selection.setPrim(other)
    appController._processEvents()
    _Check(panel._header.text() == "/ArmAsset/Rig/Solvers/IKFKBlend",
           "changing the selection rebuilt the panel: %r"
           % panel._header.text())

    appController._dataModel.selection.setPrim(prim)
    appController._processEvents()

    # --- 4. an edit lands on the row's own layer ------------------------
    rows = _RowItems(panel)
    bend = [r for (_, key, _, r) in rows
            if key == "rigExec:preferredBendRadians"]
    _Check(bend, "the IK has a preferredBendRadians opinion")
    row = bend[0]
    _Check(row.layer == fileLayer,
           "that opinion is in the root layer, not the session layer")

    before = fileLayer.GetAttributeAtPath(
        IK + ".rigExec:preferredBendRadians").default
    edit = model.SetRowValue(row, "-0.5")
    appController._processEvents()
    after = fileLayer.GetAttributeAtPath(
        IK + ".rigExec:preferredBendRadians").default
    _Check(abs(after - (-0.5)) < 1e-9,
           "the edit reached the row's layer: %s" % after)
    _Check(abs(prim.GetAttribute("rigExec:preferredBendRadians").Get()
               - (-0.5)) < 1e-9,
           "and the composed value followed")

    edit.Undo()
    appController._processEvents()
    _Check(abs(fileLayer.GetAttributeAtPath(
        IK + ".rigExec:preferredBendRadians").default - before) < 1e-9,
        "undo put the original value back")

    # --- 5. a bad value is refused without touching the stage -----------
    raised = False
    try:
        model.SetRowValue(row, "wildly not a double")
    except model.ValueParseError:
        raised = True
    _Check(raised, "a malformed value raises ValueParseError")
    _Check(abs(fileLayer.GetAttributeAtPath(
        IK + ".rigExec:preferredBendRadians").default - before) < 1e-9,
        "and left the stage untouched")

    # --- 6. delete + undo through the panel's own path ------------------
    panel.Rebuild()
    appController._processEvents()
    rows = _RowItems(panel)
    soft = [r for (_, key, _, r) in rows if key == "inputs:softness"]
    _Check(soft, "the IK has an inputs:softness opinion")
    deleted = model.DeleteRow(soft[0])
    appController._processEvents()
    _Check(fileLayer.GetAttributeAtPath(IK + ".inputs:softness") is None,
           "the opinion is gone from the layer")
    deleted.Undo()
    appController._processEvents()
    _Check(fileLayer.GetAttributeAtPath(IK + ".inputs:softness") is not None,
           "undo restored the deleted opinion")

    # --- 7. the MENU path, which is how it is actually opened -----------
    # Everything above drove OpenLayerOpinionsPanel directly. This is the
    # route an artist takes: the action's callback goes through the
    # container's lazy sibling import, so a broken import or a wrong
    # command name shows up here and nowhere else.
    layerOpinionsUI.LayerOpinionsPanel._instance = None
    action = [a for a in menus[0].actions()
              if a.text() == "Layer Opinions"][0]
    action.trigger()
    appController._processEvents()
    opened = layerOpinionsUI.LayerOpinionsPanel._instance
    _Check(opened is not None,
           "triggering the menu item opened the panel")
    _Check(opened.isVisible(), "the panel it opened is visible")
    _Check(opened._header.text() == IK,
           "and it built on the current selection: %r"
           % opened._header.text())
    _Check(opened._undo is not None,
           "the menu path hands it the container's shared undo stack")
    panel = opened

    shot = os.environ.get("RIGEXEC_OPINIONS_SHOT")
    if shot:
        panel.Rebuild()
        appController._processEvents()
        panel.grab().save(shot)

    print("RIGEXEC_LAYER_OPINIONS_OK menu, selection, edit, parse "
          "refusal, delete, undo")
