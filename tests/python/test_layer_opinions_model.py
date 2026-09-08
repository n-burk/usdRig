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
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("LAYER_OPINIONS_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
