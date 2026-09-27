#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/pathSpelling.py: paths are kept
the way they were typed -- relative stays relative, absolute stays
absolute -- in the Layer Opinions panel's rows and in the .usda a save
writes, although USD itself anchors every relative path it holds.

Layers here live in a temp directory: what is under test is the file.

Usage: test_path_spelling.py
"""
import io
import os
import shutil
import sys
import tempfile

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402

import compositionArcsModel as arcs  # noqa: E402
import layerOpinionsModel as lom  # noqa: E402
import pathSpelling  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


SCENE = '''#usda 1.0

def "World"
{
    def "Body"
    {
        rel material:binding = </Material/Old>
        float in
        float in.connect = </Material/Old.out>
    }

    def "Leg" (
        inherits = </_class_Leg>
    )
    {
    }
}

def "Material"
{
    def "Cloth"
    {
        float out
    }

    def "Old"
    {
        float out
    }
}
'''


class _Scene(object):
    """SCENE saved to a fresh temp file, opened on a stage."""

    def __init__(self, text=SCENE, name="scene.usda"):
        self.dir = tempfile.mkdtemp(prefix="pathSpelling")
        self.path = os.path.join(self.dir, name)
        # Exported, then opened: importing text into a .usdc layer leaves
        # it without the crate data its own Save() insists on.
        source = Sdf.Layer.CreateAnonymous("source.usda")
        source.ImportFromString(text)
        source.Export(self.path)
        self.layer = Sdf.Layer.FindOrOpen(self.path)
        self.stage = Usd.Stage.Open(self.layer)

    def Rows(self, primPath="/World/Body"):
        prim = self.stage.GetPrimAtPath(primPath)
        return {row.key: row for group in lom.OpinionGroups(prim)
                if group.layer == self.layer
                for row in lom.WalkRows(group.rows)}

    def File(self):
        with io.open(self.path, encoding="utf-8") as stream:
            return stream.read()

    def Close(self):
        self.stage = None
        shutil.rmtree(self.dir, ignore_errors=True)


def _NewSession():
    """Forget what the panel typed, as a fresh usdview would."""
    pathSpelling._typed.clear()
    pathSpelling._fromFile.clear()


def TestTypedRelativeIsSavedRelative():
    '''
    The reported case: `../../Material/Cloth` typed on /World/Body. The
    stage targets /Material/Cloth -- USD anchors it -- but the row goes
    on reading what was typed, and the saved file says it too.
    '''
    scene = _Scene()
    try:
        lom.SetRowValue(scene.Rows()["material:binding"],
                        "../../Material/Cloth")
        rel = scene.stage.GetPrimAtPath("/World/Body").GetRelationship(
            "material:binding")
        _Check(rel.GetTargets() == [Sdf.Path("/Material/Cloth")],
               "the stage composes the anchored target: %s"
               % rel.GetTargets())
        _Check(scene.Rows()["material:binding"].valueText
               == "[ <../../Material/Cloth> ]",
               "the row keeps the spelling: %r"
               % scene.Rows()["material:binding"].valueText)

        report = pathSpelling.SaveLayer(scene.layer)
        text = scene.File()
        _Check("rel material:binding = <../../Material/Cloth>" in text,
               "the file keeps it relative:\n%s" % text)
        _Check(report.relative == 1 and not report.note,
               "and the report says so: %s" % report)
        _Check(not scene.layer.dirty, "the layer is saved")

        check = Sdf.Layer.CreateAnonymous("check.usda")
        check.ImportFromString(text)
        _Check(list(check.GetRelationshipAtPath(
            "/World/Body.material:binding").targetPathList.explicitItems)
            == [Sdf.Path("/Material/Cloth")],
            "and reads back to the same target")
    finally:
        scene.Close()


def TestTypedAbsoluteIsSavedAbsolute():
    '''
    No attempt the other way either: a path typed absolute is saved
    absolute -- including over one the file spelled relative.
    '''
    scene = _Scene()
    try:
        lom.SetRowValue(scene.Rows()["material:binding"],
                        "../../Material/Cloth")
        pathSpelling.SaveLayer(scene.layer)
        lom.SetRowValue(scene.Rows()["material:binding"], "/Material/Cloth")
        _Check(scene.Rows()["material:binding"].valueText
               == "[ </Material/Cloth> ]",
               "typed absolute, it reads absolute: %r"
               % scene.Rows()["material:binding"].valueText)
        report = pathSpelling.SaveLayer(scene.layer)
        text = scene.File()
        _Check("rel material:binding = </Material/Cloth>" in text
               and "<../../Material/Cloth>" not in text,
               "and the file says it absolute:\n%s" % text)
        _Check(report.relative == 0, "nothing was kept relative: %s"
               % report)
    finally:
        scene.Close()


def TestSpellingSurvivesANewSession():
    '''
    The spelling lives in the file, not only in this session: reopened
    fresh, the row reads it back out of the file, and saving an
    unrelated edit keeps it.
    '''
    scene = _Scene()
    try:
        lom.SetRowValue(scene.Rows()["material:binding"],
                        "../../Material/Cloth")
        pathSpelling.SaveLayer(scene.layer)
        _NewSession()
        _Check(scene.Rows()["material:binding"].valueText
               == "[ <../../Material/Cloth> ]",
               "read back from the file: %r"
               % scene.Rows()["material:binding"].valueText)

        scene.layer.GetPrimAtPath("/World/Body").SetInfo("kind", "x")
        pathSpelling.SaveLayer(scene.layer)
        _Check("rel material:binding = <../../Material/Cloth>"
               in scene.File(),
               "an unrelated save keeps it:\n%s" % scene.File())
    finally:
        scene.Close()


def TestConnectionsAndInheritsKeepTheirSpelling():
    scene = _Scene()
    try:
        lom.SetRowValue(scene.Rows()["in.connect"], "../../Material/Cloth.out")
        _Check(scene.Rows()["in.connect"].valueText
               == "[ <../../Material/Cloth.out> ]",
               "a connection reads as typed: %r"
               % scene.Rows()["in.connect"].valueText)
        lom.SetRowValue(scene.Rows("/World/Leg")["inherits [0]"],
                        "../_class_Leg")
        _Check(scene.Rows("/World/Leg")["inherits [0]"].valueText
               == "<../_class_Leg>",
               "an inherit reads as typed: %r"
               % scene.Rows("/World/Leg")["inherits [0]"].valueText)
        _Check(list(scene.layer.GetPrimAtPath(
            "/World/Leg").inheritPathList.explicitItems)
            == [Sdf.Path("/World/_class_Leg")],
            "anchored at /World/Leg")

        report = pathSpelling.SaveLayer(scene.layer)
        text = scene.File()
        _Check("float in.connect = <../../Material/Cloth.out>" in text,
               "the connection is saved as typed:\n%s" % text)
        _Check("inherits = <../_class_Leg>" in text,
               "the inherit is saved as typed:\n%s" % text)
        _Check(report.relative == 2, "both kept: %s" % report)
    finally:
        scene.Close()


VARIANT_SCENE = '''#usda 1.0

def "Asset" (
    variants = {
        string look = "a"
    }
    prepend variantSets = "look"
)
{
    variantSet "look" = {
        "a" {
            def "Geo"
            {
                rel binding = </Asset/Material>
            }
        }
    }

    def "Material"
    {
    }

    def "Other"
    {
    }

    def "Rig" (
        references = </Asset/Other>
    )
    {
        rel self = </Asset/Material>
    }
}
'''


def TestWhereUsdaCannotKeepARelativePath():
    '''
    Three places the usda reader will not take a relative path back:
    inside a variant, a reference's prim path, and a target that is the
    prim itself (`<.>` is no usda). The row shows the absolute path
    there -- what the file will say -- and the save writes that.
    '''
    scene = _Scene(VARIANT_SCENE)
    try:
        lom.SetRowValue(scene.Rows("/Asset/Geo")["binding"], "../Material")
        _Check(scene.Rows("/Asset/Geo")["binding"].valueText
               == "[ </Asset/Material> ]",
               "inside a variant: absolute: %r"
               % scene.Rows("/Asset/Geo")["binding"].valueText)
        lom.SetRowValue(scene.Rows("/Asset/Rig")["references [0]"],
                        "../Other")
        _Check(scene.Rows("/Asset/Rig")["references [0]"].valueText
               == "</Asset/Other>",
               "a reference: absolute: %r"
               % scene.Rows("/Asset/Rig")["references [0]"].valueText)
        lom.SetRowValue(scene.Rows("/Asset/Rig")["self"], "<.>")
        _Check(scene.Rows("/Asset/Rig")["self"].valueText
               == "[ </Asset/Rig> ]",
               "the prim itself: absolute: %r"
               % scene.Rows("/Asset/Rig")["self"].valueText)

        report = pathSpelling.SaveLayer(scene.layer)
        text = scene.File()
        _Check(report.relative == 0 and "<.." not in text
               and "<.>" not in text,
               "and none of them is saved relative (%s):\n%s"
               % (report, text))
        _Check(not scene.layer.dirty, "the layer is still saved")
    finally:
        scene.Close()


def TestAFileWithNothingRelativeIsAPlainSave():
    '''
    A layer nothing was typed relative into is saved exactly as USD
    saves it: no rewrite, no churn in files the panel only looked at.
    '''
    scene = _Scene()
    try:
        scene.layer.GetPrimAtPath("/World/Body").SetInfo("kind", "x")
        report = pathSpelling.SaveLayer(scene.layer)
        _Check(report.relative == 0 and not report.note,
               "nothing to keep: %s" % report)
        _Check(scene.File() == scene.layer.ExportToString(),
               "the file is Sdf's own text")
    finally:
        scene.Close()


def TestReadingSkipsStringsAndComments():
    '''
    Reading spellings out of a hand-written file: only a path in a
    target, connection, inherit or specialize is one. A `<../x>` in a
    comment or a doc string, and a comment's apostrophe, are not.
    '''
    text = '''#usda 1.0
(
    doc = """keep <../NotAPath> as text, don't touch"""
)

# a note that says <../AlsoNot>, and isn't closed properly '
def "World"
{
    def "Body" (
        doc = "see <../StillNot>"
    )
    {
        rel r = [<../../Material/Cloth>, </Material/Old>]
    }
}
'''
    spellings = pathSpelling.SpellingsInText(text)
    _Check(spellings == {("/World/Body.r", "targetPaths", "/Material/Cloth"):
                         "../../Material/Cloth"},
           "only the relative target is a spelling: %s" % spellings)


def TestASpellingThatWouldNotReadBackIsNotWritten():
    '''
    The save reads its own text back before writing it. Force a
    spelling the reader refuses -- a relative path inside a variant --
    past the rule that normally stops it, and the layer is saved
    absolute with the reason, rather than written unreadable.
    '''
    scene = _Scene(VARIANT_SCENE)
    original = pathSpelling.CanSpellRelative
    try:
        pathSpelling.CanSpellRelative = lambda *args: True
        pathSpelling._typed.setdefault(scene.layer.identifier, {})[
            ("/Asset{look=a}Geo.binding", "targetPaths",
             "/Asset/Material")] = "../Material"
        scene.layer.GetPrimAtPath("/Asset/Rig").SetInfo("kind", "x")
        report = pathSpelling.SaveLayer(scene.layer)
        _Check(report.note and report.relative == 0,
               "the report says why: %s" % report)
        _Check("<../Material>" not in scene.File(),
               "the file is not written unreadable")
        _Check(not scene.layer.dirty, "but the layer is saved")
        check = Sdf.Layer.CreateAnonymous("check.usda")
        _Check(check.ImportFromString(scene.File()), "and reads back")
    finally:
        pathSpelling.CanSpellRelative = original
        scene.Close()


def TestABinaryLayerIsSavedAsIs():
    scene = _Scene(name="scene.usdc")
    try:
        lom.SetRowValue(scene.Rows()["material:binding"],
                        "../../Material/Cloth")
        report = pathSpelling.SaveLayer(scene.layer)
        _Check(report.relative == 0 and not scene.layer.dirty,
               "a .usdc has no text to spell: %s" % report)
    finally:
        scene.Close()


def TestTheInheritFlowPreviewsAndKeepsTheSpelling():
    '''
    The guided flow is the same: the preview shows the inherit as
    typed, which is what a save will write, and authoring it keeps the
    spelling for the panel's row and the save.
    '''
    scene = _Scene()
    try:
        prim = scene.stage.GetPrimAtPath("/World/Body")
        context = arcs.ArcContext(scene.stage, prim)
        values = {"layer": scene.layer, "primPath": "../_class_Body",
                  "position": "prepend"}
        preview = arcs.InheritArc.Preview(context, values)
        _Check("inherits = <../_class_Body>" in preview,
               "the preview shows it as typed:\n%s" % preview)
        arcs.InheritArc.Author(context, values)
        _Check(scene.Rows()["prepend inherits [0]"].valueText
               == "<../_class_Body>",
               "the row shows it as typed: %r"
               % scene.Rows()["prepend inherits [0]"].valueText)
        pathSpelling.SaveLayer(scene.layer)
        _Check("prepend inherits = <../_class_Body>" in scene.File(),
               "and the file says it:\n%s" % scene.File())
        editing = arcs.ArcContext(scene.stage, prim,
                                  scene.Rows()["prepend inherits [0]"])
        _Check(arcs.InheritArc.Defaults(editing)["primPath"]
               == "../_class_Body",
               "reopening it shows the spelling: %s"
               % arcs.InheritArc.Defaults(editing))
    finally:
        scene.Close()


def main():
    groups = [
        ("typed relative is saved relative",
         TestTypedRelativeIsSavedRelative),
        ("typed absolute is saved absolute",
         TestTypedAbsoluteIsSavedAbsolute),
        ("the spelling survives a new session",
         TestSpellingSurvivesANewSession),
        ("connections and inherits keep their spelling",
         TestConnectionsAndInheritsKeepTheirSpelling),
        ("where usda cannot keep a relative path",
         TestWhereUsdaCannotKeepARelativePath),
        ("nothing relative is a plain save",
         TestAFileWithNothingRelativeIsAPlainSave),
        ("reading skips strings and comments",
         TestReadingSkipsStringsAndComments),
        ("an unreadable spelling is not written",
         TestASpellingThatWouldNotReadBackIsNotWritten),
        ("a binary layer is saved as is", TestABinaryLayerIsSavedAsIs),
        ("the inherit flow keeps the spelling",
         TestTheInheritFlowPreviewsAndKeepsTheSpelling),
    ]
    for name, fn in groups:
        _NewSession()
        fn()
        print("  ok: %s" % name)
    print("PATH_SPELLING_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
