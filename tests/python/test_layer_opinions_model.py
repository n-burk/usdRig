#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/layerOpinionsModel.py: the
per-layer opinion enumeration for one prim, the usda value round trip
the inline editor parses through, the edit and delete operations, and
the snapshots that make both undoable.

Everything here runs on in-memory layers -- no Qt, no usdview.

Usage: test_layer_opinions_model.py [schema resource dir]
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Sdf, Usd  # noqa: E402

import layerOpinionsModel as lom  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Stage():
    """
    A three-layer stage. /Rig/IK carries an opinion in each layer, and
    `strong` shadows `weak` on the same attribute so the winning flag
    has something to distinguish.
    """
    weak = Sdf.Layer.CreateAnonymous("weak.usda")
    weak.ImportFromString('''#usda 1.0
def Scope "Rig"
{
    def Xform "IK" (
        kind = "component"
    )
    {
        double softness = 0.15
        double shadowed = 1.0
        rel joints = [ </Rig/A>, </Rig/B> ]
    }
    def Scope "A" { }
    def Scope "B" { }
}
''')
    strong = Sdf.Layer.CreateAnonymous("strong.usda")
    strong.ImportFromString('''#usda 1.0
over "Rig"
{
    over "IK"
    {
        double shadowed = 2.0
    }
}
''')
    root = Sdf.Layer.CreateAnonymous("root.usda")
    root.subLayerPaths.append(strong.identifier)
    root.subLayerPaths.append(weak.identifier)
    stage = Usd.Stage.Open(root)
    return stage, root, strong, weak


def TestEnumeration():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    groups = lom.OpinionGroups(prim)

    # One group per layer that actually has a spec, strongest first.
    # root.usda carries no spec for this prim and must not appear.
    _Check([g.layer for g in groups] == [strong, weak],
           "groups are the specced layers, strongest first: %s"
           % [g.layer.identifier for g in groups])

    weakRows = {(r.kind, r.key): r for r in groups[1].rows}
    _Check(("attribute", "softness") in weakRows,
           "attribute rows are enumerated: %s" % sorted(weakRows))
    _Check(("relationship", "joints") in weakRows,
           "relationship rows are enumerated: %s" % sorted(weakRows))
    _Check(("info", "kind") in weakRows,
           "prim metadata rows are enumerated: %s" % sorted(weakRows))
    _Check(("info", "specifier") in weakRows,
           "specifier is a metadata row: %s" % sorted(weakRows))

    # The stronger opinion wins; the weaker same-named one is marked.
    strongRows = {(r.kind, r.key): r for r in groups[0].rows}
    _Check(strongRows[("attribute", "shadowed")].winning,
           "the strongest opinion on an attribute wins")
    _Check(not weakRows[("attribute", "shadowed")].winning,
           "a shadowed opinion is not marked winning")
    _Check(weakRows[("attribute", "softness")].winning,
           "an unshadowed opinion wins even in a weak layer")


def TestArcsAreEnumeratedAsInfoRows():
    layer = Sdf.Layer.CreateAnonymous("arcs.usda")
    layer.ImportFromString('''#usda 1.0
def Xform "A" (
    prepend references = @other.usda@</B>
    prepend variantSets = "v"
    variants = { string v = "x" }
)
{
    variantSet "v" = { "x" { } }
}
''')
    stage = Usd.Stage.Open(layer)
    groups = lom.OpinionGroups(stage.GetPrimAtPath("/A"))
    keys = {(r.kind, r.key) for r in groups[0].rows}
    for key in ("references", "variantSetNames", "variantSelection"):
        _Check(("info", key) in keys,
               "composition arc %r is an info row: %s" % (key, sorted(keys)))


def TestSpecPathUnderAReference():
    """
    A spec reached through a reference lives at the REFERENCED path, not
    the prim's. An edit that used the prim path would author a brand new
    override in the wrong place instead of changing the opinion shown.
    """
    src = Sdf.Layer.CreateAnonymous("src.usda")
    src.ImportFromString('''#usda 1.0
def Xform "Base"
{
    double softness = 0.5
}
''')
    root = Sdf.Layer.CreateAnonymous("ref.usda")
    root.ImportFromString('''#usda 1.0
def Xform "A" (
    prepend references = @%s@</Base>
)
{
}
''' % src.identifier)
    stage = Usd.Stage.Open(root)
    groups = lom.OpinionGroups(stage.GetPrimAtPath("/A"))
    byLayer = {g.layer: g for g in groups}
    _Check(src in byLayer, "the referenced layer contributes a group")
    row = {r.key: r for r in byLayer[src].rows}["softness"]
    _Check(row.specPath == Sdf.Path("/Base.softness"),
           "the row keeps its own spec path: %s" % row.specPath)


def TestValueRoundTrip():
    cases = [
        (Sdf.ValueTypeNames.Double, 3.5),
        (Sdf.ValueTypeNames.Int, -7),
        (Sdf.ValueTypeNames.Bool, True),
        (Sdf.ValueTypeNames.String, "hello world"),
        (Sdf.ValueTypeNames.Token, "uniformSegments"),
        (Sdf.ValueTypeNames.Asset, Sdf.AssetPath("./tex.png")),
        (Sdf.ValueTypeNames.Float3, Gf.Vec3f(1, 2, 3)),
        (Sdf.ValueTypeNames.Double3, Gf.Vec3d(1.5, -2.5, 0)),
        (Sdf.ValueTypeNames.Matrix4d, Gf.Matrix4d(1.0).SetTranslate(
            Gf.Vec3d(4, 5, 6))),
        (Sdf.ValueTypeNames.IntArray, [1, 2, 3]),
        (Sdf.ValueTypeNames.DoubleArray, [0.25, 0.5]),
    ]
    for typeName, value in cases:
        text = lom.FormatValue(value)
        parsed = lom.ParseValue(typeName, text)
        _Check(parsed == value,
               "%s round trips through %r: got %r, want %r"
               % (typeName, text, parsed, value))


def TestFloatUsesTheSpecsOwnType():
    """
    A float attribute must format as a float. Inferring the type from
    the python value promotes it to double and shows 0.15 as
    0.15000000596046448 -- and editing that row would then author the
    double-rounded text back onto a float.
    """
    layer = Sdf.Layer.CreateAnonymous("float.usda")
    layer.ImportFromString('''#usda 1.0
def Xform "A"
{
    float softness = 0.15
}
''')
    stage = Usd.Stage.Open(layer)
    groups = lom.OpinionGroups(stage.GetPrimAtPath("/A"))
    row = {r.key: r for r in groups[0].rows}["softness"]
    _Check(row.valueText == "0.15",
           "a float row shows its own precision: %r" % row.valueText)


def TestParseValueRejectsGarbage():
    try:
        lom.ParseValue(Sdf.ValueTypeNames.Double, "not a number")
    except lom.ValueParseError:
        return
    raise AssertionError("a malformed value must raise ValueParseError")


def TestParseValueDoesNotEvaluateCode():
    """
    The editor parses whatever the user typed. It must go through usda
    syntax, never through eval -- a value field is not a code prompt.
    """
    try:
        lom.ParseValue(Sdf.ValueTypeNames.Double,
                       "__import__('os').getcwd()")
    except lom.ValueParseError:
        return
    raise AssertionError("a python expression must not be evaluated")


def TestEditAttributeAndUndo():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    row = lom.FindRow(lom.OpinionGroups(prim), weak, "attribute", "softness")

    edit = lom.SetRowValue(row, "0.75")
    _Check(weak.GetAttributeAtPath("/Rig/IK.softness").default == 0.75,
           "the edit reaches the row's own layer")
    edit.Undo()
    _Check(weak.GetAttributeAtPath("/Rig/IK.softness").default == 0.15,
           "undo restores the previous value")
    edit.Redo()
    _Check(weak.GetAttributeAtPath("/Rig/IK.softness").default == 0.75,
           "redo reapplies the edit")


def TestEditInfoAndUndo():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    row = lom.FindRow(lom.OpinionGroups(prim), weak, "info", "kind")

    edit = lom.SetRowValue(row, '"assembly"')
    _Check(weak.GetPrimAtPath("/Rig/IK").GetInfo("kind") == "assembly",
           "metadata edits go through the same path")
    edit.Undo()
    _Check(weak.GetPrimAtPath("/Rig/IK").GetInfo("kind") == "component",
           "undo restores the previous metadata")


def TestEditRelationshipTargetsAndUndo():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    row = lom.FindRow(lom.OpinionGroups(prim), weak, "relationship", "joints")
    _Check(row.editable, "a relationship target list is editable")

    edit = lom.SetRowValue(row, "[ </Rig/B> ]")
    spec = weak.GetRelationshipAtPath("/Rig/IK.joints")
    _Check(list(spec.targetPathList.explicitItems) == [Sdf.Path("/Rig/B")],
           "the targets are replaced: %s" % spec.targetPathList)
    edit.Undo()
    spec = weak.GetRelationshipAtPath("/Rig/IK.joints")
    _Check(list(spec.targetPathList.explicitItems)
           == [Sdf.Path("/Rig/A"), Sdf.Path("/Rig/B")],
           "undo restores the original targets: %s" % spec.targetPathList)


def TestEditRelationshipRejectsNonPaths():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    row = lom.FindRow(lom.OpinionGroups(prim), weak, "relationship", "joints")
    try:
        lom.SetRowValue(row, "[ not_a_path ]")
    except lom.ValueParseError:
        return
    raise AssertionError("a malformed target list must raise ValueParseError")


def TestDeleteAttributeAndUndo():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    row = lom.FindRow(lom.OpinionGroups(prim), weak, "attribute", "softness")

    edit = lom.DeleteRow(row)
    _Check(weak.GetAttributeAtPath("/Rig/IK.softness") is None,
           "the property spec is gone from that layer")
    edit.Undo()
    spec = weak.GetAttributeAtPath("/Rig/IK.softness")
    _Check(spec is not None and spec.default == 0.15,
           "undo restores the deleted property spec and its value")


def TestDeleteRelationshipAndUndo():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    row = lom.FindRow(lom.OpinionGroups(prim), weak, "relationship", "joints")

    edit = lom.DeleteRow(row)
    _Check(weak.GetRelationshipAtPath("/Rig/IK.joints") is None,
           "the relationship spec is gone")
    edit.Undo()
    spec = weak.GetRelationshipAtPath("/Rig/IK.joints")
    _Check(spec is not None, "undo restores the relationship spec")
    _Check(list(spec.targetPathList.explicitItems)
           == [Sdf.Path("/Rig/A"), Sdf.Path("/Rig/B")],
           "undo restores its targets: %s" % spec.targetPathList)


def TestDeleteInfoAndUndo():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    row = lom.FindRow(lom.OpinionGroups(prim), weak, "info", "kind")

    edit = lom.DeleteRow(row)
    _Check(not weak.GetPrimAtPath("/Rig/IK").HasInfo("kind"),
           "the metadata opinion is cleared")
    edit.Undo()
    _Check(weak.GetPrimAtPath("/Rig/IK").GetInfo("kind") == "component",
           "undo restores the metadata opinion")


def TestDeletePrimSpecAndUndo():
    stage, root, strong, weak = _Stage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    group = {g.layer: g for g in lom.OpinionGroups(prim)}[weak]

    edit = lom.DeletePrimSpec(group)
    _Check(weak.GetPrimAtPath("/Rig/IK") is None,
           "the whole prim spec is gone from that layer")
    edit.Undo()
    spec = weak.GetPrimAtPath("/Rig/IK")
    _Check(spec is not None, "undo restores the prim spec")
    _Check(spec.GetInfo("kind") == "component",
           "undo restores its metadata")
    _Check(weak.GetAttributeAtPath("/Rig/IK.softness").default == 0.15,
           "undo restores its properties")


def TestReadOnlyLayerIsNotEditable():
    stage, root, strong, weak = _Stage()
    weak.SetPermissionToEdit(False)
    try:
        prim = stage.GetPrimAtPath("/Rig/IK")
        group = {g.layer: g for g in lom.OpinionGroups(prim)}[weak]
        _Check(not group.editable,
               "a layer that refuses edits is reported as not editable")
        _Check(all(not r.editable for r in group.rows),
               "its rows are not editable either")
    finally:
        weak.SetPermissionToEdit(True)


# --------------------------------------------------------------------
# Composition arcs: the rows a list-op field expands into, and the
# operations that work on one entry of one.
# --------------------------------------------------------------------


ARCS_ROOT = '''#usda 1.0
(
    relocates = {
        </Rig/Old>: </Rig/New>,
        </Rig/Second>: </Rig/Third>
    }
)

def Scope "Rig"
{
    def Xform "IK" (
        prepend references = [@./a.usda@</X> (offset = 5), @./b.usda@]
        append references = @./c.usda@
        prepend inherits = </_class_C>
        prepend variantSets = "lod"
        variants = {
            string lod = "hi"
        }
    )
    {
    }
}
'''


def _ArcStage():
    """
    A stage whose root layer carries one of every arc shape: a list op
    with two arms, a single-item list op, a variant selection, two
    sublayers with an offset on the second, and two relocates.
    """
    sub = Sdf.Layer.CreateAnonymous("sub.usda")
    sub.ImportFromString('#usda 1.0\n')
    root = Sdf.Layer.CreateAnonymous("root.usda")
    root.ImportFromString(ARCS_ROOT)
    root.subLayerPaths.append(sub.identifier)
    root.subLayerPaths.append("./retimed.usda")
    root.subLayerOffsets[1] = Sdf.LayerOffset(12, 2)
    stage = Usd.Stage.Open(root)
    return stage, root, sub


def _ArcRows(stage, path="/Rig/IK"):
    """Every row on the prim, deepest included, keyed by its name."""
    prim = stage.GetPrimAtPath(path)
    rows = {}
    for group in lom.OpinionGroups(prim):
        for row in lom.WalkRows(group.rows):
            rows[row.key] = row
    return rows


def TestArcFieldExpandsIntoOneRowPerArc():
    """
    An arc field is a LIST of arcs. Shown as one row it can only be
    deleted whole, so it is a heading with a row per item -- each
    carrying the arm and the index that address it.
    """
    stage, root, sub = _ArcStage()
    rows = _ArcRows(stage)

    parent = rows["references"]
    _Check(parent.kind == "info", "the field itself stays a metadata row")
    _Check(parent.valueText == "3 items",
           "the heading counts its arcs, got %r" % parent.valueText)
    _Check(len(parent.children) == 3,
           "all three arcs get a row: %s" % parent.children)

    first = rows["prepend references [0]"]
    _Check(first.kind == "arcItem", "an arc row says it is an arc")
    _Check((first.arm, first.index) == ("prepended", 0),
           "it carries the arm and index that address it: %s"
           % ((first.arm, first.index),))
    _Check(first.valueText == "@./a.usda@</X> (offset = 5)",
           "shown as usda writes it, got %r" % first.valueText)
    _Check(rows["append references [0]"].arm == "appended",
           "the appended arm is its own list, numbered from zero")

    # The keyword is USD's, not the info key: `inheritPaths` is written
    # `inherits`, and a row labelled otherwise would not match the file.
    _Check("prepend inherits [0]" in rows,
           "inheritPaths is labelled `inherits`: %s" % sorted(rows))
    _Check("prepend variantSets [0]" in rows,
           "variantSetNames is labelled `variantSets`: %s" % sorted(rows))


def TestEditingOneArcLeavesTheOthersAlone():
    stage, root, sub = _ArcStage()
    row = _ArcRows(stage)["prepend references [0]"]
    edit = lom.SetRowValue(row, "@./new.usda@</Y> (offset = 1; scale = 2)")

    spec = root.GetPrimAtPath("/Rig/IK")
    prepended = list(spec.referenceList.prependedItems)
    _Check(prepended == [Sdf.Reference("./new.usda", "/Y",
                                       Sdf.LayerOffset(1, 2)),
                         Sdf.Reference("./b.usda")],
           "the arc is replaced where it sat: %s" % prepended)
    _Check(list(spec.referenceList.appendedItems)
           == [Sdf.Reference("./c.usda")],
           "the other arm is untouched")
    edit.Undo()
    _Check(list(spec.referenceList.prependedItems)[0]
           == Sdf.Reference("./a.usda", "/X", Sdf.LayerOffset(5)),
           "undo puts the original arc back, offset and all")


def TestRemovingTheLastArcClearsTheField():
    """
    Emptying a list op is not the absence of one. An explicit list left
    empty reads `references = None`, which BLOCKS every weaker
    reference, and the other arms leave a field that exports as nothing
    but still answers HasInfo -- so the panel would list a heading over
    no arcs. Removing the last one removes the field.
    """
    stage, root, sub = _ArcStage()
    for _ in range(3):
        rows = _ArcRows(stage)
        row = next(r for r in rows.values() if r.kind == "arcItem"
                   and r.infoKey == "references")
        edit = lom.DeleteRow(row)
    spec = root.GetPrimAtPath("/Rig/IK")
    _Check(not spec.HasInfo("references"),
           "the field is gone, not left empty: %s" % root.ExportToString())
    _Check("references" not in root.ExportToString(),
           "and nothing about references survives in the file")
    edit.Undo()
    _Check(spec.HasInfo("references"), "undo brings the last one back")


def TestMovingAnArcReordersItWithinItsArm():
    stage, root, sub = _ArcStage()
    rows = _ArcRows(stage)
    edit = lom.MoveRow(rows["prepend references [0]"], 1)
    spec = root.GetPrimAtPath("/Rig/IK")
    _Check(list(spec.referenceList.prependedItems)[0]
           == Sdf.Reference("./b.usda"),
           "the weaker arc is now the stronger one")
    _Check(len(spec.referenceList.appendedItems) == 1,
           "the appended arm did not gain or lose anything")
    edit.Undo()
    _Check(list(spec.referenceList.prependedItems)[0].assetPath
           == "./a.usda", "undo puts the order back")

    # An arm's ends are ends. Moving off one is not a wrap, and is not
    # a move into the next arm either -- those are different opinions.
    rows = _ArcRows(stage)
    _Check(not lom.CanMove(rows["prepend references [0]"], -1),
           "the strongest arc cannot go up")
    _Check(not lom.CanMove(rows["append references [0]"], 1),
           "an arm of one has nowhere to go")
    _Check(lom.MoveRow(rows["append references [0]"], 1) is None,
           "and asking anyway does nothing rather than reaching the "
           "prepended arm")


def TestVariantSelectionIsEditedPerSet():
    stage, root, sub = _ArcStage()
    row = _ArcRows(stage)["lod"]
    _Check(row.kind == "variantSelection", "the selection gets its own row")
    _Check(row.valueText == '"hi"', "shown quoted, got %r" % row.valueText)

    edit = lom.SetRowValue(row, "lo")
    spec = root.GetPrimAtPath("/Rig/IK")
    _Check(dict(spec.variantSelections) == {"lod": "lo"},
           "the selection is switched")
    edit.Undo()
    _Check(dict(spec.variantSelections) == {"lod": "hi"}, "and switched back")

    edit = lom.DeleteRow(_ArcRows(stage)["lod"])
    _Check(not spec.HasInfo("variantSelection"),
           "deleting the only selection clears the field")
    edit.Undo()
    _Check(dict(spec.variantSelections) == {"lod": "hi"},
           "undo restores it")


def TestLayerArcsAreListedUnderTheirLayer():
    """
    Sublayers and relocates are the LAYER's composition, not the prim's,
    and this panel is the only place they can be seen. They are listed
    under the group of the layer that holds them.
    """
    stage, root, sub = _ArcStage()
    rows = _ArcRows(stage)

    _Check(rows["subLayers"].kind == "layerInfo",
           "the heading says it is layer metadata")
    _Check(rows["subLayer [1]"].valueText
           == "@./retimed.usda@ (offset = 12; scale = 2)",
           "a sublayer's offset is shown with it, got %r"
           % rows["subLayer [1]"].valueText)
    _Check(rows["relocate [0]"].valueText == "</Rig/Old>: </Rig/New>",
           "a relocate reads as usda writes it, got %r"
           % rows["relocate [0]"].valueText)

    edit = lom.SetRowValue(rows["subLayer [1]"],
                           "@./other.usda@ (offset = 3)")
    _Check(list(root.subLayerPaths)[1] == "./other.usda",
           "the sublayer is retargeted")
    _Check(root.subLayerOffsets[1] == Sdf.LayerOffset(3),
           "with its new offset")
    _Check(root.subLayerOffsets[0] == Sdf.LayerOffset(),
           "and the other sublayer's offset is not reset by the write")
    edit.Undo()
    _Check(root.subLayerOffsets[1] == Sdf.LayerOffset(12, 2),
           "undo restores the offset, not only the path")

    edit = lom.SetRowValue(_ArcRows(stage)["relocate [0]"],
                           "</Rig/Old>: </Rig/Renamed>")
    _Check(list(root.relocates)[0] == (Sdf.Path("/Rig/Old"),
                                       Sdf.Path("/Rig/Renamed")),
           "the relocate is retargeted: %s" % root.relocates)
    _Check(len(root.relocates) == 2, "and the other one is still there")
    edit.Undo()
    _Check(list(root.relocates)[0][1] == Sdf.Path("/Rig/New"),
           "undo restores it")


def TestLayerArcsAreNotShownForAReferencedLayer():
    """
    A referenced asset's own sublayers are not this stage's composition.
    Listing them under the prim they arrive at would invite editing the
    asset for every shot that uses it.
    """
    inner = Sdf.Layer.CreateAnonymous("inner.usda")
    inner.ImportFromString('#usda 1.0\ndef Xform "Deep"\n{\n}\n')
    asset = Sdf.Layer.CreateAnonymous("asset.usda")
    asset.ImportFromString('''#usda 1.0
(
    defaultPrim = "Hand"
)

def Xform "Hand" (
    prepend references = @./inner.usda@
)
{
}
''')
    asset.subLayerPaths.append(inner.identifier)
    root = Sdf.Layer.CreateAnonymous("root.usda")
    root.ImportFromString('#usda 1.0\ndef Xform "Shot"\n{\n}\n')
    root.GetPrimAtPath("/Shot").referenceList.Prepend(
        Sdf.Reference(asset.identifier))
    stage = Usd.Stage.Open(root)

    groups = {g.layer: g for g in
              lom.OpinionGroups(stage.GetPrimAtPath("/Shot"))}
    assetGroup = groups[asset]
    _Check(not assetGroup.local,
           "a layer reached through a reference is not in the local stack")
    _Check(not any(r.kind == "layerInfo" for r in assetGroup.rows),
           "so its sublayers are not listed: %s"
           % [r.key for r in assetGroup.rows])
    inside = [r for r in lom.WalkRows(assetGroup.rows)
              if r.kind == "arcItem"]
    _Check(inside, "the asset's own reference is still SHOWN")
    _Check(all(not r.editable for r in inside),
           "but not editable -- changing it would change the asset "
           "everywhere it is used")
    _Check(not lom.CanDeleteRow(inside[0], assetGroup),
           "and not removable either")


def TestAComposingListOpIsNotMarkedOverridden():
    """
    A weaker layer's `prepend references` is not shadowed by a stronger
    layer's -- both contribute, in that order. Striking the weaker row
    through would claim an override that is not happening. Only an
    EXPLICIT list replaces what is underneath.
    """
    weak = Sdf.Layer.CreateAnonymous("weak.usda")
    weak.ImportFromString('''#usda 1.0

def Xform "A" (
    prepend references = @./w.usda@
    kind = "component"
)
{
}
''')
    strong = Sdf.Layer.CreateAnonymous("strong.usda")
    strong.ImportFromString('''#usda 1.0

over "A" (
    prepend references = @./s.usda@
    kind = "assembly"
)
{
}
''')
    root = Sdf.Layer.CreateAnonymous("root.usda")
    root.subLayerPaths.append(strong.identifier)
    root.subLayerPaths.append(weak.identifier)
    stage = Usd.Stage.Open(root)
    groups = {g.layer: g for g in
              lom.OpinionGroups(stage.GetPrimAtPath("/A"))}

    weakRefs = lom.FindRow([groups[weak]], weak, "info", "references")
    _Check(weakRefs.winning,
           "the weaker prepend still composes, so it is not struck out")
    weakKind = lom.FindRow([groups[weak]], weak, "info", "kind")
    _Check(not weakKind.winning,
           "an ordinary metadatum IS shadowed by the stronger layer")

    # An explicit list is the case that really does replace.
    strong.GetPrimAtPath("/A").referenceList.explicitItems = [
        Sdf.Reference("./s.usda")]
    groups = {g.layer: g for g in
              lom.OpinionGroups(stage.GetPrimAtPath("/A"))}
    weakRefs = lom.FindRow([groups[weak]], weak, "info", "references")
    _Check(not weakRefs.winning,
           "an explicit stronger list does shadow the weaker one")


def TestArcTextIsParsedNotEvaluated():
    """
    The value column takes whatever is typed. Same rule as ParseValue:
    a python expression must be a parse error, never something that
    runs, and two arcs typed into a one-arc row must be refused rather
    than silently losing one.
    """
    stage, root, sub = _ArcStage()
    for text in ('__import__("os").getcwd()',
                 "[@./a.usda@, @./b.usda@]",
                 "not a reference at all"):
        row = _ArcRows(stage)["prepend references [0]"]
        try:
            lom.SetRowValue(row, text)
            raise AssertionError("%r was accepted" % text)
        except lom.ValueParseError:
            pass
    _Check(list(root.GetPrimAtPath("/Rig/IK").referenceList.prependedItems)[0]
           == Sdf.Reference("./a.usda", "/X", Sdf.LayerOffset(5)),
           "and nothing was authored on the way")


def TestDeletingAWholeLayerGroupIgnoresItsLayerArcs():
    """
    The layer's own rows sit at the front of the group and their spec
    path is the pseudo-root. Taking rows[0] blindly would hand that to
    the remover and empty the layer instead of removing one prim.
    """
    stage, root, sub = _ArcStage()
    group = {g.layer: g for g in
             lom.OpinionGroups(stage.GetPrimAtPath("/Rig/IK"))}[root]
    _Check(group.rows[0].kind == "layerInfo",
           "the fixture really does put a layer row first")

    lom.DeletePrimSpec(group)
    _Check(root.GetPrimAtPath("/Rig/IK") is None, "the prim spec is gone")
    _Check(root.GetPrimAtPath("/Rig") is not None,
           "its parent is not")
    _Check(len(root.subLayerPaths) == 2,
           "and the layer still has its sublayers: %s"
           % list(root.subLayerPaths))


def main():
    groups = [
        ("enumeration", TestEnumeration),
        ("arcs as info rows", TestArcsAreEnumeratedAsInfoRows),
        ("spec path under a reference", TestSpecPathUnderAReference),
        ("value round trip", TestValueRoundTrip),
        ("float keeps its own type", TestFloatUsesTheSpecsOwnType),
        ("parse rejects garbage", TestParseValueRejectsGarbage),
        ("parse does not eval", TestParseValueDoesNotEvaluateCode),
        ("edit attribute", TestEditAttributeAndUndo),
        ("edit info", TestEditInfoAndUndo),
        ("edit relationship", TestEditRelationshipTargetsAndUndo),
        ("relationship rejects non-paths", TestEditRelationshipRejectsNonPaths),
        ("delete attribute", TestDeleteAttributeAndUndo),
        ("delete relationship", TestDeleteRelationshipAndUndo),
        ("delete info", TestDeleteInfoAndUndo),
        ("delete prim spec", TestDeletePrimSpecAndUndo),
        ("read-only layer", TestReadOnlyLayerIsNotEditable),
        ("an arc field expands", TestArcFieldExpandsIntoOneRowPerArc),
        ("edit one arc", TestEditingOneArcLeavesTheOthersAlone),
        ("removing the last arc clears the field",
         TestRemovingTheLastArcClearsTheField),
        ("move an arc", TestMovingAnArcReordersItWithinItsArm),
        ("variant selection", TestVariantSelectionIsEditedPerSet),
        ("layer arcs", TestLayerArcsAreListedUnderTheirLayer),
        ("no layer arcs for a referenced layer",
         TestLayerArcsAreNotShownForAReferencedLayer),
        ("a composing list op is not overridden",
         TestAComposingListOpIsNotMarkedOverridden),
        ("arc text is parsed, not evaluated",
         TestArcTextIsParsedNotEvaluated),
        ("deleting a group ignores its layer arcs",
         TestDeletingAWholeLayerGroupIgnoresItsLayerArcs),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("LAYER_OPINIONS_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
