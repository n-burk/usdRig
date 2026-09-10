#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/compositionArcsModel.py: the
composition arcs the Layer Opinions panel can author, what each one
refuses, the usda preview shown before committing, and the undo that
takes each back out.

Everything here runs on in-memory layers -- no Qt, no usdview.

Usage: test_composition_arcs_model.py
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402

import compositionArcsModel as arcs  # noqa: E402
import layerOpinionsModel  # noqa: E402


BUILD_MOVERS = '''#usda 1.0

def Xform "Movers"
{
    def Xform "Finish"
    {
    }

    def Xform "Middle"
    {
    }

    def Xform "Start"
    {
    }
}
'''

BUILD_INTERNAL_REF = '''#usda 1.0

class Xform "_class_Ctrl"
{
    def Xform "Handle"
    {
    }
}

def Xform "Rig"
{
    def Xform "IK" (
        prepend references = </_class_Ctrl>
    )
    {
    }
}
'''


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Raises(fn, message):
    try:
        fn()
    except arcs.ArcError:
        return
    raise AssertionError(message)


def _Asset():
    """A referenceable asset layer with a defaultPrim."""
    layer = Sdf.Layer.CreateAnonymous("asset.usda")
    layer.ImportFromString('''#usda 1.0
(
    defaultPrim = "Hand"
)

def Xform "Hand"
{
    double grip = 0.5

    def Xform "Thumb"
    {
    }
}
''')
    return layer


def _Stage():
    """
    A two-layer stage: a root plus a sublayer, so there is more than one
    legal authoring target and the strength order of the combo has
    something to be wrong about. The sublayer is WEAKER than the root
    that pulls it in, which is the ordering the combo must reproduce.
    """
    sub = Sdf.Layer.CreateAnonymous("sub.usda")
    sub.ImportFromString('#usda 1.0\n')
    root = Sdf.Layer.CreateAnonymous("root.usda")
    root.ImportFromString('''#usda 1.0

class Xform "_class_Ctrl"
{
    double gain = 2.0
}

def Xform "Rig"
{
    def Xform "Base"
    {
    }

    def Xform "IK"
    {
        double softness = 0.15
    }
}
''')
    root.subLayerPaths.append(sub.identifier)
    stage = Usd.Stage.Open(root)
    return stage, root, sub


def _Context(stage, path="/Rig/IK"):
    prim = stage.GetPrimAtPath(path)
    _Check(bool(prim), "the fixture has %s" % path)
    return arcs.ArcContext(stage, prim)


# --------------------------------------------------------------------


def TestAuthoringLayersAreTheLocalStackOnly():
    """
    Only the local layer stack may be authored into.

    A referenced asset's layer contributes specs to the prim stack --
    that is what the Layer Opinions panel shows -- but authoring an arc
    there would edit the asset for every place it is used. The layer
    choice must not offer it, however visible it is in the panel.
    """
    stage, root, sub = _Stage()
    asset = _Asset()
    stage.GetPrimAtPath("/Rig/IK").GetReferences().AddReference(
        asset.identifier)

    prim = stage.GetPrimAtPath("/Rig/IK")
    stackLayers = {spec.layer for spec in prim.GetPrimStack()}
    _Check(asset in stackLayers,
           "the asset layer is in the prim stack, as the panel shows it")

    allowed = arcs.AuthoringLayers(stage)
    _Check(asset not in allowed,
           "but it is not offered as an authoring target")
    _Check(root in allowed and sub in allowed,
           "the local layer stack is: %s"
           % [l.GetDisplayName() for l in allowed])
    _Check(stage.GetSessionLayer() in allowed,
           "including the session layer, which is where usdview edits")

    # Strength order, so the combo and the panel's grouping agree.
    _Check(allowed.index(root) < allowed.index(sub),
           "in strength order -- a layer beats the sublayers it pulls in: "
           "%s" % [l.GetDisplayName() for l in allowed])

    # And Author() re-checks it rather than trusting the caller.
    context = _Context(stage)
    _Raises(lambda: arcs.ReferenceArc.Author(context, {
        "layer": asset, "source": "external",
        "assetPath": "./x.usda", "primPath": "", "position": "prepend"}),
        "authoring into a referenced layer is refused")


def TestDefaultAuthoringLayer():
    stage, root, sub = _Stage()
    _Check(arcs.DefaultAuthoringLayer(stage, root) == root,
           "the clicked layer wins")
    # An unclickable choice falls through to the edit target, which on a
    # freshly opened stage is the root layer.
    asset = _Asset()
    _Check(arcs.DefaultAuthoringLayer(stage, asset)
           == stage.GetEditTarget().GetLayer(),
           "a layer that cannot be authored into falls back")
    stage.SetEditTarget(Usd.EditTarget(sub))
    _Check(arcs.DefaultAuthoringLayer(stage, None) == sub,
           "with nothing clicked, usdview's own edit target is used")


def TestReferenceAuthorAndUndo():
    stage, root, sub = _Stage()
    context = _Context(stage)
    asset = _Asset()
    values = {"layer": root, "source": "external",
              "assetPath": asset.identifier, "primPath": "",
              "offset": "0", "scale": "1", "position": "prepend"}

    _Check(not stage.GetPrimAtPath("/Rig/IK/Thumb"),
           "the asset is not composed yet")
    edit, warnings = arcs.ReferenceArc.Author(context, values)
    _Check(not warnings, "a resolvable asset with a defaultPrim is clean: "
           "%s" % warnings)
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Thumb")),
           "the reference composed the asset in")
    _Check(abs(stage.GetPrimAtPath("/Rig/IK").GetAttribute("grip").Get()
               - 0.5) < 1e-9,
           "and brought its opinions")

    edit.Undo()
    _Check(not stage.GetPrimAtPath("/Rig/IK/Thumb"),
           "undo removed the reference")
    _Check(abs(stage.GetPrimAtPath("/Rig/IK")
               .GetAttribute("softness").Get() - 0.15) < 1e-9,
           "and left the prim's own opinions alone")
    edit.Redo()
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Thumb")), "redo put it back")


def TestReferenceIntoALayerWithNoSpecUndoesToNoSpec():
    """
    The prim has no spec in the session layer. Adding a reference there
    creates an `over`, and undo must remove the whole spec again --
    leaving an empty over behind would make a "no opinions" layer show
    up in the panel forever.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    session = stage.GetSessionLayer()
    _Check(session.GetPrimAtPath("/Rig/IK") is None,
           "the session layer starts with no spec for the prim")

    # Held in a local: an anonymous layer nothing references is collected,
    # and the arc would then resolve to nothing partway through the test.
    asset = _Asset()
    edit, _ = arcs.ReferenceArc.Author(context, {
        "layer": session, "source": "external",
        "assetPath": asset.identifier, "primPath": "",
        "offset": "0", "scale": "1", "position": "prepend"})
    spec = session.GetPrimAtPath("/Rig/IK")
    _Check(spec is not None, "the arc created a spec")
    _Check(spec.specifier == Sdf.SpecifierOver,
           "as an over, not a def: %s" % spec.specifier)

    edit.Undo()
    _Check(session.GetPrimAtPath("/Rig/IK") is None,
           "undo removed the spec it created, not just the arc")


def TestReferencePositionAndLayerOffset():
    stage, root, sub = _Stage()
    context = _Context(stage)
    asset = _Asset()
    common = {"layer": root, "source": "external",
              "assetPath": asset.identifier, "primPath": "/Hand"}
    arcs.ReferenceArc.Author(context, dict(
        common, offset="0", scale="1", position="prepend"))
    arcs.ReferenceArc.Author(context, dict(
        common, offset="12", scale="2", position="append"))

    listOp = root.GetPrimAtPath("/Rig/IK").referenceList
    _Check(len(listOp.prependedItems) == 1 and len(listOp.appendedItems) == 1,
           "prepend and append reached different arms of the list op")
    _Check(listOp.prependedItems[0].layerOffset == Sdf.LayerOffset(),
           "an untouched offset stays identity")
    _Check(listOp.appendedItems[0].layerOffset == Sdf.LayerOffset(12, 2),
           "the offset and scale reached the Sdf.Reference: %s"
           % listOp.appendedItems[0].layerOffset)


def TestReferenceRefusals():
    stage, root, sub = _Stage()
    context = _Context(stage)
    base = {"layer": root, "offset": "0", "scale": "1", "position": "prepend"}

    _Raises(lambda: arcs.ReferenceArc.Author(context, dict(
        base, source="external", assetPath="", primPath="")),
        "an external reference with no asset path is refused")
    _Raises(lambda: arcs.ReferenceArc.Author(context, dict(
        base, source="internal", assetPath="", primPath="/Rig/IK")),
        "a prim referencing itself is refused")
    _Raises(lambda: arcs.ReferenceArc.Author(context, dict(
        base, source="internal", assetPath="", primPath="/Rig")),
        "a prim referencing its own ancestor is refused")
    _Raises(lambda: arcs.ReferenceArc.Author(context, dict(
        base, source="internal", assetPath="", primPath="Rig/IK")),
        "a relative target path is refused")
    _Raises(lambda: arcs.ReferenceArc.Author(context, dict(
        base, source="external", assetPath="./a.usda", primPath="",
        scale="0")),
        "a zero time scale is refused")
    _Raises(lambda: arcs.ReferenceArc.Author(context, dict(
        base, source="external", assetPath="./a.usda", primPath="",
        offset="soon")),
        "a non-numeric offset is refused")

    _Check(root.GetPrimAtPath("/Rig/IK").referenceList.isExplicit is False
           and not root.GetPrimAtPath("/Rig/IK").HasInfo("references"),
           "and none of them authored anything")


def TestReferenceWarnsWithoutBlocking():
    stage, root, sub = _Stage()
    context = _Context(stage)
    noDefault = Sdf.Layer.CreateAnonymous("nodefault.usda")
    noDefault.ImportFromString('#usda 1.0\n\ndef Xform "A"\n{\n}\n')

    edit, warnings = arcs.ReferenceArc.Author(context, {
        "layer": root, "source": "external",
        "assetPath": noDefault.identifier, "primPath": "",
        "offset": "0", "scale": "1", "position": "prepend"})
    _Check(warnings and "defaultPrim" in warnings[0],
           "no defaultPrim and no target prim warns: %s" % warnings)
    _Check(root.GetPrimAtPath("/Rig/IK").HasInfo("references"),
           "but the arc was still authored -- a warning does not block")
    edit.Undo()

    _, warnings = arcs.ReferenceArc.Author(context, {
        "layer": root, "source": "external",
        "assetPath": "./not-here-yet.usda", "primPath": "/A",
        "offset": "0", "scale": "1", "position": "prepend"})
    _Check(warnings and "resolve" in warnings[0],
           "an unresolvable asset warns rather than refusing: %s" % warnings)


def TestInternalReference():
    stage, root, sub = _Stage()
    context = _Context(stage)
    edit, warnings = arcs.ReferenceArc.Author(context, {
        "layer": root, "source": "internal", "assetPath": "",
        "primPath": "/_class_Ctrl", "offset": "0", "scale": "1",
        "position": "prepend"})
    _Check(not warnings, "an existing internal target is clean: %s" % warnings)
    item = root.GetPrimAtPath("/Rig/IK").referenceList.prependedItems[0]
    _Check(item.assetPath == "", "an internal reference has no asset path")
    _Check(item.primPath == Sdf.Path("/_class_Ctrl"), "and names the prim")
    _Check(abs(stage.GetPrimAtPath("/Rig/IK").GetAttribute("gain").Get()
               - 2.0) < 1e-9,
           "and the class's opinion composed in")
    edit.Undo()


def TestPayload():
    stage, root, sub = _Stage()
    context = _Context(stage)
    asset = _Asset()
    edit, _ = arcs.PayloadArc.Author(context, {
        "layer": root, "source": "external", "assetPath": asset.identifier,
        "primPath": "", "offset": "0", "scale": "1", "position": "prepend"})
    _Check(root.GetPrimAtPath("/Rig/IK").HasInfo("payload"),
           "the payload landed in the `payload` field, not `references`")
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Thumb")),
           "and composed, because the stage loads payloads by default")
    edit.Undo()
    _Check(not root.GetPrimAtPath("/Rig/IK").HasInfo("payload"),
           "undo cleared it")


def TestInheritAndSpecialize():
    stage, root, sub = _Stage()
    context = _Context(stage)

    edit, warnings = arcs.InheritArc.Author(context, {
        "layer": root, "primPath": "/_class_Ctrl", "position": "prepend"})
    _Check(not warnings, "a class target is the clean case: %s" % warnings)
    _Check(root.GetPrimAtPath("/Rig/IK").inheritPathList.prependedItems[0]
           == Sdf.Path("/_class_Ctrl"),
           "the inherit reached the inheritPaths list op")
    _Check(abs(stage.GetPrimAtPath("/Rig/IK").GetAttribute("gain").Get()
               - 2.0) < 1e-9, "and composed")
    edit.Undo()
    _Check(not stage.GetPrimAtPath("/Rig/IK").GetAttribute("gain"),
           "undo removed it")

    edit, warnings = arcs.SpecializeArc.Author(context, {
        "layer": root, "primPath": "/Rig/Base", "position": "prepend"})
    _Check(warnings and "class" in warnings[0].lower(),
           "specializing a non-class prim warns: %s" % warnings)
    _Check(root.GetPrimAtPath("/Rig/IK").specializesList.prependedItems[0]
           == Sdf.Path("/Rig/Base"),
           "and the specialize reached its own field")
    edit.Undo()

    _Raises(lambda: arcs.InheritArc.Author(context, {
        "layer": root, "primPath": "/Rig/IK", "position": "prepend"}),
        "inheriting from itself is refused")


def TestVariantSet():
    stage, root, sub = _Stage()
    context = _Context(stage)
    edit, warnings = arcs.VariantSetArc.Author(context, {
        "layer": root, "name": "lod", "variants": "high\nlow\n",
        "selection": "high", "position": "prepend"})
    _Check(not warnings, "a set with variants and a selection is clean: %s"
           % warnings)

    prim = stage.GetPrimAtPath("/Rig/IK")
    _Check(prim.GetVariantSets().GetNames() == ["lod"],
           "the set is declared: %s" % prim.GetVariantSets().GetNames())
    vset = prim.GetVariantSet("lod")
    _Check(sorted(vset.GetVariantNames()) == ["high", "low"],
           "both variants exist: %s" % vset.GetVariantNames())
    _Check(vset.GetVariantSelection() == "high", "and the selection stuck")

    # The variant branches must be real edit targets, not just names.
    with vset.GetVariantEditContext():
        prim.CreateAttribute("inside", Sdf.ValueTypeNames.Double).Set(7.0)
    _Check(abs(prim.GetAttribute("inside").Get() - 7.0) < 1e-9,
           "authoring inside the selected variant composes")
    _Check(root.GetAttributeAtPath("/Rig/IK{lod=high}.inside")
           is not None, "and landed on the variant's own spec")

    edit.Undo()
    _Check(not stage.GetPrimAtPath("/Rig/IK").GetVariantSets().GetNames(),
           "undo removed the whole set")
    _Check(abs(stage.GetPrimAtPath("/Rig/IK")
               .GetAttribute("softness").Get() - 0.15) < 1e-9,
           "without taking the prim's other opinions with it")
    edit.Redo()
    _Check(stage.GetPrimAtPath("/Rig/IK").GetVariantSets().GetNames()
           == ["lod"], "redo restored the set")
    # Redo restores the spec as it was WHEN THE ARC WAS AUTHORED, so the
    # attribute written into the variant afterwards does not come back.
    # That is the snapshot contract, not a defect: the panel pushes every
    # edit onto one shared stack, where undo is last-in-first-out and the
    # later authoring would be undone first.
    _Check(root.GetAttributeAtPath("/Rig/IK{lod=high}.inside") is None,
           "and only that -- later edits are the next entry's business")


def TestVariantSetRefusals():
    stage, root, sub = _Stage()
    context = _Context(stage)
    base = {"layer": root, "position": "prepend"}
    _Raises(lambda: arcs.VariantSetArc.Author(context, dict(
        base, name="", variants="a", selection="")),
        "an unnamed variant set is refused")
    _Raises(lambda: arcs.VariantSetArc.Author(context, dict(
        base, name="not a name", variants="a", selection="")),
        "a non-identifier set name is refused")
    _Raises(lambda: arcs.VariantSetArc.Author(context, dict(
        base, name="lod", variants="high\nhigh", selection="")),
        "duplicate variant names are refused")
    _Raises(lambda: arcs.VariantSetArc.Author(context, dict(
        base, name="lod", variants="high", selection="mid")),
        "selecting a variant that is not being created is refused")

    _, warnings = arcs.VariantSetArc.Author(context, dict(
        base, name="lod", variants="high\nlow", selection=""))
    _Check(warnings and "selection" in warnings[0].lower(),
           "authoring no selection warns: %s" % warnings)


def TestVariantSetAddsToAnExistingSet():
    stage, root, sub = _Stage()
    context = _Context(stage)
    arcs.VariantSetArc.Author(context, {
        "layer": root, "name": "lod", "variants": "high",
        "selection": "high", "position": "prepend"})
    _, warnings = arcs.VariantSetArc.Author(context, {
        "layer": root, "name": "lod", "variants": "low",
        "selection": "", "position": "prepend"})
    _Check(any("already" in w for w in warnings),
           "a second flow on the same set says so: %s" % warnings)
    vset = stage.GetPrimAtPath("/Rig/IK").GetVariantSet("lod")
    _Check(sorted(vset.GetVariantNames()) == ["high", "low"],
           "and added to it rather than replacing it: %s"
           % vset.GetVariantNames())
    names = root.GetPrimAtPath("/Rig/IK").variantSetNameList.prependedItems
    _Check(list(names) == ["lod"],
           "without listing the set name twice: %s" % list(names))


def TestSublayer():
    """
    A sublayer composes into the layer that pulls it in, WEAKER than
    that layer's own opinions -- so the strength this flow controls is
    the order among the sublayers, not against the host. The fixture
    puts the contest between two sublayers for that reason.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    sub.ImportFromString('''#usda 1.0

over "Rig"
{
    over "IK"
    {
        double tint = 1.0
    }
}
''')
    extra = Sdf.Layer.CreateAnonymous("extra.usda")
    extra.ImportFromString('''#usda 1.0

over "Rig"
{
    over "IK"
    {
        double tint = 9.0
    }
}
''')
    # No stage.Reload() here: reloading an ANONYMOUS layer resets it to
    # empty, so it would wipe the fixture rather than refresh it.
    # ImportFromString already notifies, and the stage has recomposed.
    _Check(abs(stage.GetPrimAtPath("/Rig/IK").GetAttribute("tint").Get()
               - 1.0) < 1e-9,
           "the existing sublayer's opinion is the one composing")

    before = list(root.subLayerPaths)
    edit, _ = arcs.SublayerArc.Author(context, {
        "layer": root, "assetPath": extra.identifier,
        "offset": "0", "scale": "1", "position": "prepend"})
    _Check(list(root.subLayerPaths)[0] == extra.identifier,
           "prepend put it first: %s" % list(root.subLayerPaths))
    _Check(abs(stage.GetPrimAtPath("/Rig/IK").GetAttribute("tint").Get()
               - 9.0) < 1e-9,
           "and first among sublayers is strongest, so its opinion won")

    edit.Undo()
    _Check(list(root.subLayerPaths) == before, "undo restored the list")
    _Check(abs(stage.GetPrimAtPath("/Rig/IK").GetAttribute("tint").Get()
               - 1.0) < 1e-9, "and the composed value with it")

    # Appending puts it behind the sublayer that is already there, so
    # the same layer loses the same contest.
    edit, _ = arcs.SublayerArc.Author(context, {
        "layer": root, "assetPath": extra.identifier,
        "offset": "0", "scale": "1", "position": "append"})
    _Check(abs(stage.GetPrimAtPath("/Rig/IK").GetAttribute("tint").Get()
               - 1.0) < 1e-9,
           "appended, it is weaker than the sublayer already listed")

    _Raises(lambda: arcs.SublayerArc.Author(context, {
        "layer": root, "assetPath": extra.identifier, "offset": "0",
        "scale": "1", "position": "prepend"}),
        "sublayering the same path twice is refused")
    _Raises(lambda: arcs.SublayerArc.Author(context, {
        "layer": root, "assetPath": "", "offset": "0", "scale": "1",
        "position": "prepend"}),
        "an empty sublayer path is refused")
    _Raises(lambda: arcs.SublayerArc.Author(context, {
        "layer": root, "assetPath": root.identifier, "offset": "0",
        "scale": "1", "position": "prepend"}),
        "a layer sublayering itself is refused")


def TestSublayerOffsetSurvivesUndo():
    """
    Assigning subLayerPaths resets every offset to identity, so an undo
    that restored only the paths would silently un-retime a sublayer
    that was already there.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    root.subLayerOffsets[0] = Sdf.LayerOffset(24.0, 1.0)
    _Check(root.subLayerOffsets[0] == Sdf.LayerOffset(24, 1), "retimed")

    extra = Sdf.Layer.CreateAnonymous("extra.usda")
    extra.ImportFromString('#usda 1.0\n')
    edit, _ = arcs.SublayerArc.Author(context, {
        "layer": root, "assetPath": extra.identifier,
        "offset": "5", "scale": "2", "position": "prepend"})
    _Check(root.subLayerOffsets[0] == Sdf.LayerOffset(5, 2),
           "the new sublayer got its offset: %s" % root.subLayerOffsets[0])
    _Check(root.subLayerOffsets[1] == Sdf.LayerOffset(24, 1),
           "and the existing one kept its own")

    edit.Undo()
    _Check(list(root.subLayerOffsets) == [Sdf.LayerOffset(24, 1)],
           "undo restored paths AND offsets: %s" % list(root.subLayerOffsets))


def TestRelocate():
    stage, root, sub = _Stage()
    asset = _Asset()
    stage.GetPrimAtPath("/Rig/IK").GetReferences().AddReference(
        asset.identifier)
    context = _Context(stage)
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Thumb")),
           "the referenced child is there to relocate")

    edit, warnings = arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK/Thumb", "target": "/Rig/IK/Pollex"})
    _Check(not warnings, "relocating a referenced prim is the clean case: %s"
           % warnings)
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Pollex")),
           "the prim moved")
    _Check(not stage.GetPrimAtPath("/Rig/IK/Thumb"),
           "and is gone from its old path")
    _Check(list(root.relocates) == [(Sdf.Path("/Rig/IK/Thumb"),
                                     Sdf.Path("/Rig/IK/Pollex"))],
           "authored as LAYER metadata: %s" % list(root.relocates))

    edit.Undo()
    _Check(not root.relocates, "undo cleared the relocates map")
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Thumb")), "and the prim is back")


def TestRelocateRefusals():
    """
    What a relocate may and may not do, checked against what USD 26.08
    actually accepts rather than against a plausible-sounding rule.
    Reparenting IS allowed; relocating a root prim, or a prim nothing
    referenced in, is not.
    """
    stage, root, sub = _Stage()
    asset = _Asset()
    stage.GetPrimAtPath("/Rig/IK").GetReferences().AddReference(
        asset.identifier)
    context = _Context(stage)

    _Raises(lambda: arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK", "target": "/Rig/IK"}),
        "relocating a path onto itself is refused")
    _Raises(lambda: arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK", "target": "/Rig/IK/Inner"}),
        "relocating a prim into itself is refused")
    _Raises(lambda: arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig", "target": "/Other"}),
        "Pcp ignores a root-prim relocate, so it is refused up front")
    _Raises(lambda: arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK/Thumb", "target": "/Rig/Base"}),
        "relocating onto a path that already exists is refused")

    # The dangerous one: a prim with only local opinions has no arc to
    # relocate, so USD removes it from the old path, composes nothing at
    # the new one, and reports no error at all. The prim just vanishes.
    _Raises(lambda: arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/Base", "target": "/Rig/Renamed"}),
        "relocating a purely local prim is refused, because USD would "
        "delete it rather than move it")
    _Check(bool(stage.GetPrimAtPath("/Rig/Base")),
           "and it is still there")


def TestRelocateMayReparent():
    """
    A relocate is not restricted to renaming in place: USD 26.08 moves
    the prim under a different parent quite happily. The flow allows it
    and only remarks on it, because refusing it would forbid something
    legal.
    """
    stage, root, sub = _Stage()
    asset = _Asset()
    stage.GetPrimAtPath("/Rig/IK").GetReferences().AddReference(
        asset.identifier)
    context = _Context(stage)
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Thumb")), "the child is there")

    edit, warnings = arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK/Thumb",
        "target": "/Rig/Base/Thumb"})
    _Check(any("rather than renaming" in w for w in warnings),
           "reparenting is remarked on, not refused: %s" % warnings)
    _Check(bool(stage.GetPrimAtPath("/Rig/Base/Thumb")),
           "and it really moved under the new parent")
    _Check(not stage.GetPrimAtPath("/Rig/IK/Thumb"),
           "leaving nothing at the old path")
    _Check(not stage.GetCompositionErrors(),
           "with no composition errors: %s"
           % [str(e) for e in stage.GetCompositionErrors()])
    edit.Undo()
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Thumb")), "undo put it back")


def TestMutedLayerIsNotOffered():
    """
    Muting drops a layer from the local layer stack entirely, so it is
    not an authoring target while muted. Refusing it is the honest
    answer: an arc authored into a muted layer composes nothing and
    reads as a silent failure.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    _Check(sub in arcs.AuthoringLayers(stage), "offered while unmuted")
    stage.MuteLayer(sub.identifier)
    try:
        _Check(sub not in arcs.AuthoringLayers(stage),
               "and not while muted: %s"
               % [l.GetDisplayName() for l in arcs.AuthoringLayers(stage)])
        _Raises(lambda: arcs.InheritArc.Author(context, {
            "layer": sub, "primPath": "/_class_Ctrl",
            "position": "prepend"}),
            "authoring into a muted layer is refused")
        _Check(sub.GetPrimAtPath("/Rig/IK") is None,
               "and nothing was authored")
    finally:
        stage.UnmuteLayer(sub.identifier)


def TestEveryArcValidatesThroughOneEntryPoint():
    """
    Validate() is what the dialog calls to decide whether Author is
    enabled, and what Author calls before touching anything. If an arc
    could pass one and fail the other, the dialog would offer a button
    that then refuses.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    for arc in arcs.ARC_KINDS:
        _Raises(lambda arc=arc: arc.Validate(context, {"layer": None}),
                "%s refuses a missing layer" % arc.key)
        _Raises(lambda arc=arc: arc.Validate(context, {"layer": _Asset()}),
                "%s refuses a layer outside the local stack" % arc.key)


def TestLayerScopedArcsDeclareTheirOwnSnapshot():
    """
    Undo for a layer-scoped arc cannot use the prim-spec snapshot: there
    is no spec involved. Each declares its own rather than Author
    branching on the arc's name, so an arc added later cannot silently
    inherit the wrong one and undo something else.
    """
    stage, root, sub = _Stage()
    expected = {
        arcs.SublayerArc: arcs.SublayerSnapshot,
        arcs.RelocateArc: arcs.RelocatesSnapshot,
    }
    for arc in arcs.ARC_KINDS:
        snapshot = arc._Snapshot(root, Sdf.Path("/Rig/IK"))
        if arc.scope == "layer":
            _Check(arc in expected,
                   "%s is layer-scoped and must name its snapshot"
                   % arc.key)
            _Check(isinstance(snapshot, expected[arc]),
                   "%s snapshots with %s, not %s"
                   % (arc.key, expected[arc].__name__,
                      type(snapshot).__name__))
        else:
            _Check(isinstance(snapshot,
                              layerOpinionsModel.SpecCopySnapshot),
                   "%s snapshots the prim spec, got %s"
                   % (arc.key, type(snapshot).__name__))
        _Check(hasattr(snapshot, "Restore"),
               "%s's snapshot satisfies the Edit contract" % arc.key)


def TestListOpKeepsAnExplicitListIntact():
    """
    Writing prependedItems on an EXPLICIT list op throws the explicit
    entries away: `references = [@a@, @b@]` becomes
    `prepend references = @new@` and the two existing arcs are gone.
    Prepend()/Append() are the operations that preserve the form.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    asset = _Asset()
    spec = root.GetPrimAtPath("/Rig/IK")
    spec.referenceList.explicitItems = [
        Sdf.Reference("./one.usda"), Sdf.Reference("./two.usda")]

    arcs.ReferenceArc.Author(context, {
        "layer": root, "source": "external", "assetPath": asset.identifier,
        "primPath": "/Hand", "offset": "0", "scale": "1",
        "position": "prepend"})

    items = list(root.GetPrimAtPath("/Rig/IK").referenceList.explicitItems)
    paths = [r.assetPath for r in items]
    _Check(len(items) == 3,
           "the explicit list still has the arcs it had: %s" % paths)
    _Check(paths[1:] == ["./one.usda", "./two.usda"],
           "unchanged and in order: %s" % paths)
    _Check(paths[0] == asset.identifier,
           "with the new one prepended ahead of them: %s" % paths)


def TestPrependGoesAheadOfEarlierPrepends():
    """
    "Prepend (stronger than existing arcs)" has to mean it. Appending to
    the prependedItems arm puts each new arc BEHIND the ones already
    prepended, which is the opposite of what the dialog promises.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    common = {"layer": root, "source": "external", "primPath": "/Hand",
              "offset": "0", "scale": "1", "position": "prepend"}
    arcs.ReferenceArc.Author(context, dict(common, assetPath="./first.usda"))
    arcs.ReferenceArc.Author(context, dict(common, assetPath="./second.usda"))

    listOp = root.GetPrimAtPath("/Rig/IK").referenceList
    paths = [r.assetPath for r in listOp.prependedItems]
    _Check(paths == ["./second.usda", "./first.usda"],
           "the later prepend is the stronger one: %s" % paths)


def TestUndoKeepsSiblingOrder():
    """
    Sdf.CopySpec appends, so restoring a spec into the middle of its
    parent would land it at the end. In RigExec that is not cosmetic:
    mover evaluation order IS sibling order, so an undo would silently
    change which mover runs first.
    """
    stage, root, sub = _Stage()
    root.ImportFromString(BUILD_MOVERS)
    context = _Context(stage, "/Movers/Middle")
    before = list(root.GetPrimAtPath("/Movers").nameChildren.keys())
    _Check(before == ["Finish", "Middle", "Start"],
           "fixture order: %s" % before)

    edit, _ = arcs.InheritArc.Author(context, {
        "layer": root, "primPath": "/Anything", "position": "prepend"})
    edit.Undo()
    after = list(root.GetPrimAtPath("/Movers").nameChildren.keys())
    _Check(after == before,
           "undo put the spec back in its own place: %s" % after)


def TestUndoRemovesTheAncestorsItCreated():
    """
    Authoring into a layer holding nothing for the prim creates the whole
    ancestor chain as overs. Undo has to take those with it, or the layer
    keeps an empty `over Rig { over IK {} }` and the panel lists it
    forever as a layer with no opinions.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    session = stage.GetSessionLayer()
    asset = _Asset()
    _Check(session.GetPrimAtPath("/Rig") is None, "session starts empty")

    edit, _ = arcs.ReferenceArc.Author(context, {
        "layer": session, "source": "external",
        "assetPath": asset.identifier, "primPath": "", "offset": "0",
        "scale": "1", "position": "prepend"})
    _Check(session.GetPrimAtPath("/Rig") is not None,
           "the arc created the ancestor chain")

    edit.Undo()
    _Check(session.GetPrimAtPath("/Rig") is None,
           "undo removed the ancestors it created, not just the leaf:\n%s"
           % session.ExportToString())


def TestNonFiniteOffsetsAreRefused():
    """
    nan and inf parse as floats and reach Sdf.LayerOffset, where they
    become an invalid-offset composition error at stage open. "1e309"
    overflows to inf the same way, so the check is on the result.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    base = {"layer": root, "source": "external", "assetPath": "./a.usda",
            "primPath": "/A", "position": "prepend"}
    for text in ("nan", "inf", "-inf", "1e309"):
        _Raises(lambda t=text: arcs.ReferenceArc.Author(
            context, dict(base, offset=t, scale="1")),
            "a %s offset is refused" % text)
        _Raises(lambda t=text: arcs.ReferenceArc.Author(
            context, dict(base, offset="0", scale=t)),
            "a %s scale is refused" % text)
    _Check(not root.GetPrimAtPath("/Rig/IK").HasInfo("references"),
           "and none of them authored anything")


def TestVariantNamesNeedNotBeIdentifiers():
    """
    USD 26.08 accepts variant names an identifier rule would reject --
    "8k", "foo-bar" and ".abc" are all legal and in use. Only the SET
    name is an identifier.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    edit, _ = arcs.VariantSetArc.Author(context, {
        "layer": root, "name": "lod", "variants": "8k\nfoo-bar\n.abc",
        "selection": "8k", "position": "prepend"})
    names = stage.GetPrimAtPath("/Rig/IK").GetVariantSet(
        "lod").GetVariantNames()
    _Check(sorted(names) == sorted([".abc", "8k", "foo-bar"]),
           "all three were authored: %s" % names)
    edit.Undo()

    # Not "": a blank line in the variants box is skipped, not an error.
    for bad in ("has space", "a/b", "{x}"):
        _Raises(lambda b=bad: arcs.VariantSetArc.Author(context, {
            "layer": root, "name": "lod", "variants": b,
            "selection": "", "position": "prepend"}),
            "%r is still refused as a variant name" % bad)
    _Raises(lambda: arcs.VariantSetArc.Author(context, {
        "layer": root, "name": "not an identifier", "variants": "a",
        "selection": "", "position": "prepend"}),
        "but the SET name must still be an identifier")


def TestSelectingAnExistingVariantIsLegal():
    """
    Adding one variant to a set while selecting another that is already
    there is ordinary authoring, and was being refused.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    arcs.VariantSetArc.Author(context, {
        "layer": root, "name": "lod", "variants": "high",
        "selection": "high", "position": "prepend"})
    arcs.VariantSetArc.Author(context, {
        "layer": root, "name": "lod", "variants": "low",
        "selection": "high", "position": "prepend"})
    _Check(stage.GetPrimAtPath("/Rig/IK").GetVariantSet(
        "lod").GetVariantSelection() == "high",
        "the existing variant could be selected")
    _Raises(lambda: arcs.VariantSetArc.Author(context, {
        "layer": root, "name": "lod", "variants": "mid",
        "selection": "nonesuch", "position": "prepend"}),
        "a selection that is in neither set is still refused")


def TestDescendantTargetIsACycle():
    """
    /A referencing /A/B composes /A/B -- and so /A -- underneath /A. Pcp
    calls that ErrorArcCycle and drops the arc, exactly as it does for a
    self or ancestor target.
    """
    stage, root, sub = _Stage()
    context = _Context(stage, "/Rig")
    cases = (
        (arcs.ReferenceArc, {"source": "internal", "assetPath": "",
                             "primPath": "/Rig/IK", "offset": "0",
                             "scale": "1"}),
        (arcs.InheritArc, {"primPath": "/Rig/IK"}),
        (arcs.SpecializeArc, {"primPath": "/Rig/IK"}),
    )
    for arc, values in cases:
        _Raises(lambda a=arc, v=values: a.Author(
            context, dict(v, layer=root, position="prepend")),
            "%s onto a descendant is refused" % arc.key)


def TestSublayerCycleThroughAnotherLayer():
    """
    Only refusing direct self-sublayering misses the ordinary loop: if
    Root already sublayers Sub, adding Root beneath Sub closes it just as
    surely, and Pcp reports ErrorSublayerCycle.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    _Check(not list(sub.subLayerPaths), "sub is clean")
    _Raises(lambda: arcs.SublayerArc.Author(context, {
        "layer": sub, "assetPath": root.identifier, "offset": "0",
        "scale": "1", "position": "prepend"}),
        "adding the host layer under its own sublayer is refused")
    _Check(not list(sub.subLayerPaths), "and nothing was authored")


def TestInternalReferenceIsRelocatable():
    """
    An internal reference targets a prim in this same layer stack, so
    every spec involved is in a local layer -- while the prim is very
    much arriving through an arc. A layer-based "is it local" test calls
    that local-only and refuses a relocate that works.
    """
    stage, root, sub = _Stage()
    root.ImportFromString(BUILD_INTERNAL_REF)
    context = _Context(stage)
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Handle")),
           "the internal reference composed its child in")
    edit, warnings = arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK/Handle",
        "target": "/Rig/IK/Grip"})
    _Check(bool(stage.GetPrimAtPath("/Rig/IK/Grip")),
           "and it relocated: %s" % warnings)
    edit.Undo()


def TestRelocateChainConflictsAreRefused():
    """
    USD does not follow a chain of relocates. Authoring /D/One -> /D/X
    next to an existing /D/X -> /D/Y is a conflict, and Pcp then ignores
    BOTH -- so the relocate that was already working stops working.
    """
    stage, root, sub = _Stage()
    asset = _Asset()
    stage.GetPrimAtPath("/Rig/IK").GetReferences().AddReference(
        asset.identifier)
    context = _Context(stage)

    arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK/Thumb", "target": "/Rig/IK/X"})
    _Raises(lambda: arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK/X", "target": "/Rig/IK/Y"}),
        "chaining onto an existing target is refused")
    _Raises(lambda: arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK/Thumb", "target": "/Rig/IK/Z"}),
        "relocating the same source twice is refused")
    _Check(len(list(root.relocates)) == 1,
           "and the working relocate is untouched: %s"
           % list(root.relocates))


def TestRelocateNeedsAnExistingTargetParent():
    stage, root, sub = _Stage()
    asset = _Asset()
    stage.GetPrimAtPath("/Rig/IK").GetReferences().AddReference(
        asset.identifier)
    context = _Context(stage)
    _Raises(lambda: arcs.RelocateArc.Author(context, {
        "layer": root, "source": "/Rig/IK/Thumb",
        "target": "/Nowhere/Thumb"}),
        "a target whose parent does not exist is refused")


def TestPreviewDoesNotMisreportExistingAncestors():
    """
    Seeding only the target spec makes the preview claim an existing
    `def "Rig"` is a new `over`, and the added-line diff then highlights
    ancestors that are not changing at all.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    values = {"layer": root, "source": "external", "assetPath": "./a.usda",
              "primPath": "/A", "offset": "0", "scale": "1",
              "position": "prepend"}
    preview = arcs.ReferenceArc.Preview(context, values)
    _Check('def Xform "Rig"' in preview,
           "the ancestor keeps the specifier the layer really gives it:"
           "\n%s" % preview)
    base = arcs.ReferenceArc.PreviewBase(context, values)
    _Check('def Xform "Rig"' in base,
           "and the un-authored seed says the same, so the diff does not "
           "mark it as added:\n%s" % base)


def TestPreviewMatchesWhatIsAuthored():
    """
    The preview is not a description of the authoring, it is the same
    code run against a scratch layer. This asserts the property that
    makes it worth showing: the spec the preview draws and the spec that
    lands are the same text.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    asset = _Asset()
    values = {"layer": root, "source": "external",
              "assetPath": asset.identifier, "primPath": "/Hand",
              "offset": "3", "scale": "1", "position": "prepend"}

    preview = arcs.ReferenceArc.Preview(context, values)
    _Check("prepend references" in preview,
           "the preview shows the arc it will author:\n%s" % preview)
    _Check("softness" in preview,
           "seeded from the real spec, so existing opinions show:\n%s"
           % preview)

    edit, _ = arcs.ReferenceArc.Author(context, values)
    scratch = Sdf.Layer.CreateAnonymous("check.usda")
    Sdf.CreatePrimInLayer(scratch, "/Rig")
    Sdf.CopySpec(root, "/Rig/IK", scratch, "/Rig/IK")
    authored = scratch.ExportToString()
    _Check(_SpecBody(preview) == _SpecBody(authored),
           "preview and authored spec agree:\n--- preview ---\n%s\n"
           "--- authored ---\n%s" % (preview, authored))
    edit.Undo()


def TestPreviewDoesNotTouchTheStage():
    stage, root, sub = _Stage()
    context = _Context(stage)
    before = root.ExportToString()
    arcs.ReferenceArc.Preview(context, {
        "layer": root, "source": "external", "assetPath": "./a.usda",
        "primPath": "/A", "offset": "0", "scale": "1", "position": "prepend"})
    arcs.SublayerArc.Preview(context, {
        "layer": root, "assetPath": "./b.usda", "offset": "0", "scale": "1",
        "position": "prepend"})
    arcs.RelocateArc.Preview(context, {
        "layer": root, "source": "/Rig/IK", "target": "/Rig/Other"})
    _Check(root.ExportToString() == before,
           "previewing authored nothing into the real layer")


def TestLayerScopedPreviewsShowLayerMetadata():
    stage, root, sub = _Stage()
    context = _Context(stage)
    preview = arcs.SublayerArc.Preview(context, {
        "layer": root, "assetPath": "./over.usda", "offset": "0",
        "scale": "1", "position": "prepend"})
    _Check("subLayers" in preview and "./over.usda" in preview,
           "the sublayer preview shows the layer's own metadata:\n%s"
           % preview)
    preview = arcs.RelocateArc.Preview(context, {
        "layer": root, "source": "/Rig/IK", "target": "/Rig/Other"})
    _Check("relocates" in preview and "/Rig/Other" in preview,
           "and the relocate preview shows the relocates map:\n%s" % preview)


def TestVisibleFieldsFollowTheSourceChoice():
    values = dict(arcs.ReferenceArc.Defaults(None))
    values["source"] = "external"
    keys = [f.key for f in arcs.VisibleFields(arcs.ReferenceArc, values)]
    _Check("assetPath" in keys, "an external reference asks for a file")
    values["source"] = "internal"
    keys = [f.key for f in arcs.VisibleFields(arcs.ReferenceArc, values)]
    _Check("assetPath" not in keys,
           "an internal one does not: %s" % keys)
    _Check("primPath" in keys, "but still asks which prim: %s" % keys)


def TestEveryArcKindIsWellFormed():
    """
    Each arc must be reachable by key, carry the text the dialog shows,
    and name a layer field -- a menu entry that opens a dialog with no
    explanation and no layer choice is a bug the UI cannot catch.
    """
    for arc in arcs.ARC_KINDS:
        _Check(arcs.ArcByKey(arc.key) is arc, "%s is reachable" % arc.key)
        for attr in ("label", "title", "summary"):
            _Check(getattr(arc, attr), "%s has a %s" % (arc.key, attr))
        keys = [f.key for f in arc.fields]
        _Check("layer" in keys, "%s asks which layer: %s" % (arc.key, keys))
        _Check(len(set(keys)) == len(keys),
               "%s has no duplicate field keys: %s" % (arc.key, keys))
        for field in arc.fields:
            _Check(field.help, "%s.%s explains itself" % (arc.key, field.key))
            if field.kind == arcs.CHOICE:
                _Check(field.choices, "%s.%s offers choices"
                       % (arc.key, field.key))
                _Check(field.default in [v for v, _ in field.choices],
                       "%s.%s defaults to one of them" % (arc.key, field.key))


def _SpecBody(usda):
    """
    The `def/over "IK"` block of an exported layer, normalised.

    Compared instead of the whole export because the scratch layers
    differ in their anonymous identifiers and in whether an ancestor is
    a def or an over -- neither of which is what the preview promises.
    """
    lines = []
    depth = 0
    started = False
    for line in usda.split("\n"):
        if not started and '"IK"' in line:
            started = True
        if not started:
            continue
        lines.append(line.rstrip())
        depth += line.count("{") - line.count("}")
        if started and depth == 0 and "}" in line:
            break
    return "\n".join(lines)


# --------------------------------------------------------------------
# Reopening an arc that is already there. Same dialog, same rules; the
# difference is one attribute on the context.
# --------------------------------------------------------------------


def _EditStage():
    """
    The two-layer fixture with one of every editable arc already
    authored into the root layer, plus a sublayer with an offset.
    """
    stage, root, sub = _Stage()
    spec = root.GetPrimAtPath("/Rig/IK")
    spec.referenceList.Prepend(
        Sdf.Reference("./hand.usda", "/Hand", Sdf.LayerOffset(5, 2)))
    spec.referenceList.Prepend(Sdf.Reference("./glove.usda"))
    spec.payloadList.Prepend(Sdf.Payload("./heavy.usda", "/Heavy"))
    spec.inheritPathList.Prepend(Sdf.Path("/_class_Ctrl"))
    spec.specializesList.Prepend(Sdf.Path("/_class_Ctrl"))
    root.subLayerOffsets[0] = Sdf.LayerOffset(9, 3)
    root.relocates = [(Sdf.Path("/Rig/Base"), Sdf.Path("/Rig/Renamed"))]
    return stage, root, sub


def _EditRow(stage, key, path="/Rig/IK"):
    prim = stage.GetPrimAtPath(path)
    for group in layerOpinionsModel.OpinionGroups(prim):
        for row in layerOpinionsModel.WalkRows(group.rows):
            if row.key == key:
                return row
    raise AssertionError("no row %r" % key)


def _EditContext(stage, row, path="/Rig/IK"):
    return arcs.ArcContext(stage, stage.GetPrimAtPath(path), row)


def TestEveryEditableArcRoundTripsThroughItsFlow():
    """
    Reopening an arc and pressing Apply without touching anything must
    author exactly what was already there. That is the property that
    makes the prefill trustworthy: if _ValuesFromRow and _ApplyEdit
    disagreed anywhere, an untouched edit would change the file.
    """
    keys = {
        "prepend references [0]": arcs.ReferenceArc,
        "prepend references [1]": arcs.ReferenceArc,
        "prepend payload [0]": arcs.PayloadArc,
        "prepend inherits [0]": arcs.InheritArc,
        "prepend specializes [0]": arcs.SpecializeArc,
        "subLayer [0]": arcs.SublayerArc,
        "relocate [0]": arcs.RelocateArc,
    }
    for key, expected in keys.items():
        stage, root, sub = _EditStage()
        row = _EditRow(stage, key)
        arc = arcs.ArcForRow(row)
        _Check(arc is expected,
               "%s is a %s row, got %s" % (key, expected.key, arc))
        context = _EditContext(stage, row)
        values = arc.Defaults(context)
        _Check(values["layer"] is row.layer,
               "%s edits the layer the arc is in" % key)
        _Check(arc.Preview(context, values)
               == arc.PreviewBase(context, values),
               "%s reopened and applied unchanged leaves the layer "
               "alone:\n%s" % (key, arc.Preview(context, values)))

        before = root.ExportToString()
        arc.Author(context, values)
        _Check(root.ExportToString() == before,
               "%s authored unchanged really does change nothing" % key)


def TestEditDropsTheLayerAndPositionFields():
    """
    Neither is a question when reopening. "Author into" would put a COPY
    somewhere else and leave the original where it was, and Position is
    the arm and index the arc already occupies -- retyping an asset path
    must not silently re-rank it.
    """
    stage, root, sub = _EditStage()
    row = _EditRow(stage, "prepend references [0]")
    context = _EditContext(stage, row)
    values = arcs.ReferenceArc.Defaults(context)

    keys = [f.key for f in arcs.ReferenceArc.FieldsFor(context)]
    _Check("layer" not in keys, "no layer choice when editing: %s" % keys)
    _Check("position" not in keys, "no position choice either: %s" % keys)
    _Check("assetPath" in keys, "the arc's own fields are still asked")

    visible = [f.key for f in
               arcs.VisibleFields(arcs.ReferenceArc, values, context)]
    _Check("layer" not in visible and "position" not in visible,
           "and the dialog is told the same: %s" % visible)

    adding = _Context(stage)
    _Check("layer" in [f.key for f in arcs.ReferenceArc.FieldsFor(adding)],
           "adding still asks which layer")
    _Check("layer" in [f.key for f in arcs.ReferenceArc.FieldsFor(None)],
           "and so does asking with no flow in hand")


def TestEditKeepsTheArcsPosition():
    stage, root, sub = _EditStage()
    row = _EditRow(stage, "prepend references [1]")
    context = _EditContext(stage, row)
    values = dict(arcs.ReferenceArc.Defaults(context))
    _Check(values["assetPath"] == "./hand.usda",
           "the weaker of the two prepends is the one addressed: %s"
           % values)

    values["assetPath"] = "./mitten.usda"
    values["primPath"] = "/Mitten"
    values["offset"] = "0"
    values["scale"] = "1"
    edit, warnings = arcs.ReferenceArc.Author(context, values)

    spec = root.GetPrimAtPath("/Rig/IK")
    _Check(list(spec.referenceList.prependedItems)
           == [Sdf.Reference("./glove.usda"),
               Sdf.Reference("./mitten.usda", "/Mitten")],
           "the arc is replaced in place, ahead of nothing and behind "
           "the glove: %s" % list(spec.referenceList.prependedItems))
    _Check(edit.label == "Edit reference",
           "the undo entry says it was an edit, got %r" % edit.label)
    edit.Undo()
    _Check(list(spec.referenceList.prependedItems)[1]
           == Sdf.Reference("./hand.usda", "/Hand", Sdf.LayerOffset(5, 2)),
           "undo restores the arc, target and offset together")
    edit.Redo()
    _Check(list(spec.referenceList.prependedItems)[1].assetPath
           == "./mitten.usda", "and redo puts the edit back")


def TestEditReadsAnInternalArcAsInternal():
    """
    An internal reference is simply one with no asset path, which is how
    Sdf stores it. Reopening one must land on the "internal" choice, or
    the form asks for a file the arc does not have.
    """
    stage, root, sub = _EditStage()
    spec = root.GetPrimAtPath("/Rig/IK")
    spec.referenceList.Prepend(Sdf.Reference("", "/_class_Ctrl"))
    row = _EditRow(stage, "prepend references [0]")
    values = arcs.ReferenceArc.Defaults(_EditContext(stage, row))
    _Check(values["source"] == "internal",
           "an arc with no asset reads as internal: %s" % values)
    _Check(values["primPath"] == "/_class_Ctrl", "with its target prim")


def TestEditingASublayerIsNotADuplicateOfItself():
    """
    Adding a sublayer a layer already has is refused. Reopening THAT
    entry to change only its offset must not be refused for the same
    reason, or the offset can never be corrected.
    """
    stage, root, sub = _EditStage()
    row = _EditRow(stage, "subLayer [0]")
    context = _EditContext(stage, row)
    values = dict(arcs.SublayerArc.Defaults(context))
    _Check(arcs.SublayerArc.Validate(context, values) == [],
           "the entry is not a duplicate of itself")

    values["offset"] = "20"
    edit, _ = arcs.SublayerArc.Author(context, values)
    _Check(root.subLayerOffsets[0] == Sdf.LayerOffset(20, 3),
           "the offset is changed and the scale kept: %s"
           % root.subLayerOffsets[0])
    _Check(list(root.subLayerPaths) == [sub.identifier],
           "and the layer is not sublayered twice")
    edit.Undo()
    _Check(root.subLayerOffsets[0] == Sdf.LayerOffset(9, 3),
           "undo restores the original offset")

    # A DIFFERENT entry still is a duplicate.
    root.subLayerPaths.append("./other.usda")
    row = _EditRow(stage, "subLayer [1]")
    context = _EditContext(stage, row)
    values = dict(arcs.SublayerArc.Defaults(context))
    values["assetPath"] = sub.identifier
    _Raises(lambda: arcs.SublayerArc.Validate(context, values),
            "retargeting one sublayer onto another is still refused")


def TestEditingARelocateExcusesItsOwnEffect():
    """
    The stage already REFLECTS the relocate being edited: the prim is
    not at its source path any more, and something DOES exist at its
    target. Asked of the entry being edited, "the target already exists"
    and "nothing composes at the source" would refuse every edit,
    including one that changes nothing.
    """
    asset = _Asset()
    root = Sdf.Layer.CreateAnonymous("root.usda")
    root.ImportFromString(
        '#usda 1.0\ndef Xform "D"\n{\n    def Xform "Taken"\n'
        '    {\n    }\n}\n')
    root.GetPrimAtPath("/D").referenceList.Prepend(
        Sdf.Reference(asset.identifier))
    root.relocates = [(Sdf.Path("/D/Thumb"), Sdf.Path("/D/Renamed"))]
    stage = Usd.Stage.Open(root)
    _Check(bool(stage.GetPrimAtPath("/D/Renamed")),
           "the fixture's relocate is in effect")

    row = _EditRow(stage, "relocate [0]", "/D")
    context = _EditContext(stage, row, "/D")
    values = dict(arcs.RelocateArc.Defaults(context))
    _Check(values == {"layer": root, "source": "/D/Thumb",
                      "target": "/D/Renamed"},
           "the entry is read back as it stands: %s" % values)
    _Check(arcs.RelocateArc.Validate(context, values) == [],
           "and reopening it neither refuses nor warns")

    values["target"] = "/D/Better"
    edit, _ = arcs.RelocateArc.Author(context, values)
    _Check(bool(stage.GetPrimAtPath("/D/Better")),
           "the prim moves to the new name")
    _Check(not stage.GetPrimAtPath("/D/Renamed"),
           "and is no longer at the old one")
    edit.Undo()
    _Check(bool(stage.GetPrimAtPath("/D/Renamed")), "undo moves it back")

    # The refusals that are still refusals.
    row = _EditRow(stage, "relocate [0]", "/D")
    context = _EditContext(stage, row, "/D")
    values = dict(arcs.RelocateArc.Defaults(context))
    _Raises(lambda: arcs.RelocateArc.Validate(
        context, dict(values, target="/D/Taken")),
        "a target that exists for some OTHER reason is still refused")
    _Raises(lambda: arcs.RelocateArc.Validate(
        context, dict(values, source="/D")),
        "a root prim as the source is still refused")


def TestARelocateDoesNotChainWithTheOthersWhenEdited():
    """
    A relocates map has to stay internally consistent, and the entry
    being edited must be excluded from that check while the OTHERS are
    not -- USD ignores both halves of a chain, so an edit that made one
    would break a relocate that was working.
    """
    asset = _Asset()
    root = Sdf.Layer.CreateAnonymous("root.usda")
    root.ImportFromString('#usda 1.0\ndef Xform "D"\n{\n}\n')
    root.GetPrimAtPath("/D").referenceList.Prepend(
        Sdf.Reference(asset.identifier))
    root.relocates = [(Sdf.Path("/D/Thumb"), Sdf.Path("/D/One")),
                      (Sdf.Path("/D/Thumb2"), Sdf.Path("/D/Two"))]
    stage = Usd.Stage.Open(root)

    row = _EditRow(stage, "relocate [0]", "/D")
    context = _EditContext(stage, row, "/D")
    values = dict(arcs.RelocateArc.Defaults(context))
    _Raises(lambda: arcs.RelocateArc.Validate(
        context, dict(values, target="/D/Two")),
        "retargeting onto another entry's target is refused")
    _Raises(lambda: arcs.RelocateArc.Validate(
        context, dict(values, source="/D/Two")),
        "and so is chaining off one")


def TestAnArcThisFlowOnlyAddsIsNotOffered():
    """
    A row of `variantSetNames` is the NAME of a set, not the set.
    Retyping it here would leave the variants behind under the old name
    while declaring one that has none, so the flow declines rather than
    offering an edit that half-works.
    """
    stage, root, sub = _Stage()
    spec = root.GetPrimAtPath("/Rig/IK")
    spec.variantSetNameList.Prepend("lod")
    row = _EditRow(stage, "prepend variantSets [0]")
    _Check(arcs.ArcForRow(row) is None,
           "no flow is offered for a variant set name")
    _Check(not arcs.CanEditRow(row), "and the panel is told so")
    _Check(not arcs.VariantSetArc.editable,
           "which is declared on the arc, not decided here")


def TestOnlyOneArcRowIsOfferedAFlow():
    """
    The heading over a list is not an arc. A flow opened on `references`
    would not know WHICH reference it meant.
    """
    stage, root, sub = _EditStage()
    prim = stage.GetPrimAtPath("/Rig/IK")
    offered = {}
    for group in layerOpinionsModel.OpinionGroups(prim):
        for row in layerOpinionsModel.WalkRows(group.rows):
            offered[row.key] = arcs.ArcForRow(row)
    _Check(offered.get("references") is None,
           "the references heading gets no flow")
    _Check(offered.get("subLayers") is None,
           "nor the subLayers heading")
    _Check(offered.get("prepend references [0]") is arcs.ReferenceArc,
           "but the arcs under them do")
    _Check(offered.get("softness") is None, "an attribute is not an arc")


def TestEveryEditableArcCanReadItsOwnRows():
    """
    An arc that says it is editable must implement both halves of it.
    Declared and checked here rather than discovered by a NotImplemented
    in front of an artist.
    """
    for arc in arcs.ARC_KINDS:
        if not arc.editable:
            continue
        for name in ("_ValuesFromRow", "_ApplyEdit"):
            _Check(getattr(arc, name).__func__
                   is not getattr(arcs._Arc, name).__func__,
                   "%s is editable, so it must define %s"
                   % (arc.key, name))
        _Check(arc in arcs._ARC_BY_FIELD.values(),
               "%s is editable, so a row must be able to find it"
               % arc.key)


# --------------------------------------------------------------------
# Which paths the Target prim combo offers. For an external arc they
# come from the ASSET; everywhere else, from this stage.
# --------------------------------------------------------------------


def _PrimPathField(arc=None):
    arc = arc or arcs.ReferenceArc
    return next(f for f in arc.fields if f.key == "primPath")


def TestTargetPrimCandidatesComeFromTheAsset():
    """
    "Which prim inside the target to compose" is a question about the
    layer being referenced. The stage being edited does not contain
    those prims -- that is the point of the arc -- so seeding the combo
    from the stage offers the one list that cannot hold the answer. On a
    shot file that has not been assembled yet, that list is the single
    root prim and nothing else.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    field = _PrimPathField()
    asset = _Asset()

    for arc in (arcs.ReferenceArc, arcs.PayloadArc):
        choices = arc.PathChoices(context, {
            "layer": root, "source": "external",
            "assetPath": asset.identifier}, field)
        _Check(choices == ["/Hand", "/Hand/Thumb"],
               "%s offers the asset's prims: %s" % (arc.key, choices))
        _Check("/Rig/IK" not in choices,
               "%s does not offer the stage's: %s" % (arc.key, choices))
    _Check(arcs.ReferenceArc.PathChoices(context, {
        "layer": root, "source": "external",
        "assetPath": asset.identifier}, field)[0] == "/Hand",
        "the defaultPrim is offered first -- it is what an empty target "
        "means")


def TestTargetPrimFallsBackToTheStage():
    """
    Three cases where the stage IS the right list, and an empty combo
    would be a worse answer than the wrong one.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    field = _PrimPathField()

    internal = arcs.ReferenceArc.PathChoices(context, {
        "layer": root, "source": "internal", "assetPath": ""}, field)
    _Check("/_class_Ctrl" in internal,
           "an internal arc targets a prim on this stage: %s" % internal)
    _Check(internal[0] == "/_class_Ctrl",
           "with the class prims first, which is what it usually wants")

    blank = arcs.ReferenceArc.PathChoices(context, {
        "layer": root, "source": "external", "assetPath": ""}, field)
    _Check("/Rig/IK" in blank,
           "no asset typed yet is not the same as an asset with no "
           "prims: %s" % blank)

    # An unresolvable path IS empty, deliberately: the asset may not
    # exist yet, and offering this stage's prims as though they were its
    # contents would be a lie rather than a fallback.
    missing = arcs.ReferenceArc.PathChoices(context, {
        "layer": root, "source": "external",
        "assetPath": "./not-here.usda"}, field)
    _Check(missing == [],
           "an asset that does not resolve offers nothing: %s" % missing)


def TestOtherPathFieldsStillNameStagePrims():
    """
    Only the target of an external arc lives elsewhere. An inherit, a
    specialize and a relocate all name a prim on the stage being edited,
    and must not be dragged along by the reference flow's rule.
    """
    stage, root, sub = _Stage()
    context = _Context(stage)
    for arc in (arcs.InheritArc, arcs.SpecializeArc):
        choices = arc.PathChoices(context, {"layer": root},
                                  _PrimPathField(arc))
        _Check(choices[0] == "/_class_Ctrl",
               "%s offers this stage's class prims first: %s"
               % (arc.key, choices))
        _Check("/Rig/IK" in choices,
               "%s offers its ordinary prims too: %s" % (arc.key, choices))

    source = next(f for f in arcs.RelocateArc.fields if f.key == "source")
    choices = arcs.RelocateArc.PathChoices(context, {"layer": root}, source)
    _Check("/Rig/IK" in choices,
           "a relocate names prims on this stage: %s" % choices)


def TestLayerPrimPathsReadsTheLayerNotAStage():
    """
    Read off the layer's own prim specs. An asset layer need not compose
    on its own -- one made of `over`s composes nothing at all -- and a
    combo that could not offer its prims would be empty for exactly the
    assets that need naming.

    nameChildren only: an arc's target must be a plain prim path, and
    `/A{lod=hi}B` is not one.
    """
    layer = Sdf.Layer.CreateAnonymous("overs.usda")
    layer.ImportFromString('''#usda 1.0

over "Asset"
{
    over "Inner"
    {
    }
}

def Xform "Switched"
{
    variantSet "lod" = {
        "hi" {
            def Xform "HiOnly"
            {
            }
        }
    }
}
''')
    paths = arcs.LayerPrimPaths(layer)
    _Check("/Asset" in paths and "/Asset/Inner" in paths,
           "a layer of overs still offers its prims: %s" % paths)
    _Check(not any("{" in path for path in paths),
           "and no variant path is offered: %s" % paths)
    _Check("/Switched/HiOnly" not in paths,
           "including the prims inside a variant, which no arc can "
           "target: %s" % paths)

    _Check(arcs.LayerPrimPaths(layer, limit=2) == paths[:2],
           "the list is bounded -- a production asset has more prims "
           "than a combo is worth")


def TestDefaultPrimIsReadAsAPath():
    """
    `defaultPrim` is spelled as a NAME in a layer's metadata and as a
    path everywhere it is used, and USD now also accepts a nested path
    there. Both have to arrive as a path, or the entry never matches the
    list it is meant to be promoted within.
    """
    layer = _Asset()
    _Check(arcs.DefaultPrimPath(layer) == "/Hand",
           "a bare name becomes a path: %r" % arcs.DefaultPrimPath(layer))
    bare = Sdf.Layer.CreateAnonymous("none.usda")
    bare.ImportFromString('#usda 1.0\ndef Xform "A"\n{\n}\n')
    _Check(arcs.DefaultPrimPath(bare) == "",
           "and no defaultPrim is the empty string, not '/'")
    _Check(arcs.LayerPrimPaths(bare) == ["/A"],
           "a layer with no defaultPrim still lists its prims")


def main():
    groups = [
        ("authoring layers are the local stack",
         TestAuthoringLayersAreTheLocalStackOnly),
        ("default authoring layer", TestDefaultAuthoringLayer),
        ("reference author and undo", TestReferenceAuthorAndUndo),
        ("reference into an empty layer",
         TestReferenceIntoALayerWithNoSpecUndoesToNoSpec),
        ("reference position and offset", TestReferencePositionAndLayerOffset),
        ("reference refusals", TestReferenceRefusals),
        ("reference warnings do not block", TestReferenceWarnsWithoutBlocking),
        ("internal reference", TestInternalReference),
        ("payload", TestPayload),
        ("inherit and specialize", TestInheritAndSpecialize),
        ("variant set", TestVariantSet),
        ("variant set refusals", TestVariantSetRefusals),
        ("variant set adds to an existing set",
         TestVariantSetAddsToAnExistingSet),
        ("sublayer", TestSublayer),
        ("sublayer offsets survive undo", TestSublayerOffsetSurvivesUndo),
        ("relocate", TestRelocate),
        ("relocate refusals", TestRelocateRefusals),
        ("a relocate may reparent", TestRelocateMayReparent),
        ("a muted layer is not offered", TestMutedLayerIsNotOffered),
        ("one validation entry point",
         TestEveryArcValidatesThroughOneEntryPoint),
        ("layer-scoped arcs declare their snapshot",
         TestLayerScopedArcsDeclareTheirOwnSnapshot),
        ("explicit list ops survive", TestListOpKeepsAnExplicitListIntact),
        ("prepend really prepends", TestPrependGoesAheadOfEarlierPrepends),
        ("undo keeps sibling order", TestUndoKeepsSiblingOrder),
        ("undo removes created ancestors",
         TestUndoRemovesTheAncestorsItCreated),
        ("non-finite offsets refused", TestNonFiniteOffsetsAreRefused),
        ("variant names need not be identifiers",
         TestVariantNamesNeedNotBeIdentifiers),
        ("selecting an existing variant",
         TestSelectingAnExistingVariantIsLegal),
        ("a descendant target is a cycle", TestDescendantTargetIsACycle),
        ("sublayer cycle through another layer",
         TestSublayerCycleThroughAnotherLayer),
        ("an internal reference is relocatable",
         TestInternalReferenceIsRelocatable),
        ("relocate chain conflicts", TestRelocateChainConflictsAreRefused),
        ("relocate needs a target parent",
         TestRelocateNeedsAnExistingTargetParent),
        ("preview does not misreport ancestors",
         TestPreviewDoesNotMisreportExistingAncestors),
        ("preview matches what is authored", TestPreviewMatchesWhatIsAuthored),
        ("preview touches nothing", TestPreviewDoesNotTouchTheStage),
        ("layer-scoped previews", TestLayerScopedPreviewsShowLayerMetadata),
        ("visible fields follow the source choice",
         TestVisibleFieldsFollowTheSourceChoice),
        ("every arc kind is well formed", TestEveryArcKindIsWellFormed),
        ("reopening an arc round trips",
         TestEveryEditableArcRoundTripsThroughItsFlow),
        ("an edit drops layer and position",
         TestEditDropsTheLayerAndPositionFields),
        ("an edit keeps the arc's position", TestEditKeepsTheArcsPosition),
        ("an internal arc reads as internal",
         TestEditReadsAnInternalArcAsInternal),
        ("a sublayer is not a duplicate of itself",
         TestEditingASublayerIsNotADuplicateOfItself),
        ("a relocate excuses its own effect",
         TestEditingARelocateExcusesItsOwnEffect),
        ("an edited relocate still cannot chain",
         TestARelocateDoesNotChainWithTheOthersWhenEdited),
        ("an add-only arc is not offered",
         TestAnArcThisFlowOnlyAddsIsNotOffered),
        ("only an arc row gets a flow", TestOnlyOneArcRowIsOfferedAFlow),
        ("editable arcs implement both halves",
         TestEveryEditableArcCanReadItsOwnRows),
        ("target prims come from the asset",
         TestTargetPrimCandidatesComeFromTheAsset),
        ("target prims fall back to the stage",
         TestTargetPrimFallsBackToTheStage),
        ("other path fields name stage prims",
         TestOtherPathFieldsStillNameStagePrims),
        ("layer prim paths read the layer",
         TestLayerPrimPathsReadsTheLayerNotAStage),
        ("defaultPrim is read as a path", TestDefaultPrimIsReadAsAPath),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("COMPOSITION_ARCS_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
