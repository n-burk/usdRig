#!/usr/bin/env python
"""
`uniform bool rigExec:baked` on the RigExecRoot, from Python: the declaration
and the path an interactive host takes to it.

Two things, because the attribute is two things. It is a SCHEMA property, so
the registry has to answer for its type, its variability and its fallback --
the checked-in generatedSchema.usda is what a host reads, and a schema edit
that never reached it would leave an authored attribute looking custom and
typeless. And it is the WEAKEST author of the evaluation mode, so a rig that
carries it must come up baked through the same binding usdview's panels use,
with nothing in that path setting the mode itself.

usdview reaches the evaluator through rigexec.Rig (python/_rigexec.cpp), and
that is what is exercised here: constructing one and compiling it is all the
bridge does, so if Rig.evaluation_mode comes back 'baked' with a source of
'attribute', the interactive path honours the attribute.

Usage: test_rigexec_baked_attribute.py [<generated schema resources dir>]
Requires the native _rigexec binding (build-python/python on PYTHONPATH).
"""
import sys

from test_rigexec_python import _setup_environment
_setup_environment()

from pxr import Sdf, Usd  # noqa: E402
import rigexec  # noqa: E402


_RIG = "/Asset/Rig"
_BAKED = "rigExec:baked"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _MakeRig(baked):
    """A control, a joint and a skinned slab -- one pose and one geometry
    domain, which is the smallest rig that actually bakes.

    `baked` is None to author nothing, or a bool to author it.
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset", "Scope")
    rig = stage.DefinePrim(_RIG, "RigExecRoot")
    if baked is not None:
        attribute = rig.GetAttribute(_BAKED)
        _Check(bool(attribute),
               "the schema declares no %s on RigExecRoot" % _BAKED)
        attribute.Set(baked)
    control = stage.DefinePrim("/Asset/Rig/Root", "RigExecControl")
    control.GetAttribute("avars:tx").Set(3.0)
    joint = stage.DefinePrim("/Asset/Rig/Bone", "RigExecJoint")
    joint.GetAttribute("avars:ty").Set(2.0)

    stage.DefinePrim("/Asset/Geom", "Scope")
    mesh = stage.DefinePrim("/Asset/Geom/Slab", "Mesh")
    mesh.GetAttribute("points").Set([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    stage.DefinePrim("/Asset/Rig/Movers", "Scope")
    skin = stage.DefinePrim("/Asset/Rig/Movers/Skin", "RigExecSkinMover")
    skin.ApplyAPI("RigExecMoverAPI")
    skin.GetRelationship("rigExec:moves").SetTargets(
        ["/Asset/Geom/Slab.points"])
    skin.CreateRelationship("rigExec:influences").SetTargets(
        ["/Asset/Rig/Bone"])
    skin.CreateAttribute("rigExec:elementSize", Sdf.ValueTypeNames.Int).Set(1)
    skin.CreateAttribute(
        "rigExec:jointIndices", Sdf.ValueTypeNames.IntArray).Set([0, 0, 0])
    skin.CreateAttribute(
        "rigExec:jointWeights", Sdf.ValueTypeNames.FloatArray).Set(
            [1.0, 1.0, 1.0])
    return stage


def TestTheSchemaDeclaresIt():
    """The declaration a host reads: RigExecRoot.rigExec:baked, uniform bool,
    falling back to false.

    Asked of the SchemaRegistry rather than of a prim, because that is what
    reads the checked-in generatedSchema.usda -- the file a schema.usda edit
    has to be regenerated into, and the one thing a hand edit to schema.usda
    alone would leave stale.
    """
    definition = Usd.SchemaRegistry().FindConcretePrimDefinition("RigExecRoot")
    _Check(definition, "RigExecRoot is not a registered concrete schema")
    _Check(_BAKED in definition.GetPropertyNames(),
           "RigExecRoot declares no %s" % _BAKED)
    spec = definition.GetSchemaAttributeSpec(_BAKED)
    _Check(spec, "%s has no attribute spec" % _BAKED)
    _Check(spec.typeName == Sdf.ValueTypeNames.Bool,
           "%s is %s, expected bool" % (_BAKED, spec.typeName))
    # Uniform, because it decides which path evaluates the whole epoch and
    # is not a channel anybody keys.
    _Check(spec.variability == Sdf.VariabilityUniform,
           "%s is not uniform" % _BAKED)
    _Check(spec.default is False,
           "%s falls back to %r, expected False" % (_BAKED, spec.default))


def TestAnAuthoredTrueOpensThroughTheProgram():
    """The interactive path: construct, compile, and the rig is baked."""
    stage = _MakeRig(True)
    rig = rigexec.Rig(stage, _RIG)
    # Nothing has asked yet; the attribute is read at compile.
    _Check(rig.evaluation_mode == "dynamic",
           "a fresh evaluator is in %s" % rig.evaluation_mode)
    _Check(rig.evaluation_mode_source == "default",
           "a fresh evaluator's source is %s" % rig.evaluation_mode_source)
    rig.compile()
    _Check(rig.evaluation_mode == "baked",
           "an authored rigExec:baked left the rig in %s" %
           rig.evaluation_mode)
    _Check(rig.evaluation_mode_source == "attribute",
           "the mode came from %s, not the attribute" %
           rig.evaluation_mode_source)
    _Check(rig.is_bakeable(),
           "the fixture does not bake: %s" % rig.bakeability_reasons())

    pose = rig.evaluate(1.0)
    _Check(pose.valid, "the baked generation is not valid")
    _Check(pose.baked_parity_mismatches == 0,
           "%d parity mismatch(es)" % pose.baked_parity_mismatches)

    # And it is the same answer a rig that asked for nothing publishes. Not
    # a proof of parity -- testRigExecBakedMode and the parity entries are
    # that, on every shipped rig -- but the one generation this path
    # produces has to agree with the one it replaces.
    reference = rigexec.Rig(_MakeRig(None), _RIG)
    reference.compile()
    _Check(reference.evaluation_mode == "dynamic",
           "the reference is in %s" % reference.evaluation_mode)
    # Asked for nothing, so it defaults to dynamic; compared as the oracle.
    reference.evaluation_mode = "reference"
    referencePose = reference.evaluate(1.0)
    _Check(referencePose.valid, "the reference generation is not valid")
    _Check(pose.joint_paths() == referencePose.joint_paths(),
           "the two paths publish %r and %r" %
           (pose.joint_paths(), referencePose.joint_paths()))
    _Check(referencePose.joint_paths(), "the fixture publishes no joints")
    for path in referencePose.joint_paths():
        a = referencePose.joint_frame(path, True)
        b = pose.joint_frame(path, True)
        _Check(a.valid and b.valid, "%s has no valid frame" % path)
        _Check(tuple(a.origin) == tuple(b.origin),
               "%s: %s dynamic, %s baked" % (path, a.origin, b.origin))


def TestNothingAuthoredStaysDynamic():
    """The negative half: without the attribute nothing changed."""
    rig = rigexec.Rig(_MakeRig(None), _RIG)
    rig.compile()
    _Check(rig.evaluation_mode == "dynamic",
           "a rig authoring nothing compiled into %s" % rig.evaluation_mode)
    _Check(rig.evaluation_mode_source == "default",
           "its source is %s" % rig.evaluation_mode_source)
    _Check(rig.evaluate(1.0).valid, "the dynamic generation is not valid")


def TestSettingTheModeOutranksTheAttribute():
    """A host that chooses deliberately keeps the choice."""
    rig = rigexec.Rig(_MakeRig(True), _RIG)
    rig.evaluation_mode = "dynamic"
    _Check(rig.evaluation_mode_source == "explicit",
           "setting the mode left the source at %s" %
           rig.evaluation_mode_source)
    rig.compile()
    _Check(rig.evaluation_mode == "dynamic",
           "the attribute overrode an explicit mode (%s)" %
           rig.evaluation_mode)
    _Check(rig.evaluation_mode_source == "explicit",
           "the source moved to %s" % rig.evaluation_mode_source)
    _Check(rig.evaluate(1.0).valid, "the dynamic generation is not valid")


def main():
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)
    groups = [
        ("the schema declares it", TestTheSchemaDeclaresIt),
        ("an authored true opens through the program",
         TestAnAuthoredTrueOpensThroughTheProgram),
        ("nothing authored stays dynamic", TestNothingAuthoredStaysDynamic),
        ("an explicit mode outranks it",
         TestSettingTheModeOutranksTheAttribute),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_BAKED_ATTRIBUTE_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
