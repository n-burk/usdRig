#!/usr/bin/env python
"""
Migrating an absolute-rest asset preserves the rig it describes.

Rest semantics changed from absolute to parent-relative, so authored rest
transforms have to be rebaked. The rebake is correct exactly when the
evaluated rig is unchanged, which is what this asserts two ways:

  * every provider's WORLD REST frame is identical before and after,
    computed by two independent routines -- the per-prim absolute read and
    the chained parent-relative product;
  * the evaluator's posed joint origins match values captured from the
    examples under the OLD code, recorded below as goldens.

Usage: test_rest_migration.py [<generated schema resources dir>]
Requires the native _rigexec binding (build-python/python on PYTHONPATH).
"""
import os
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "tools"))
import migrateRestToLocal  # noqa: E402

_EXAMPLES = (
    "examples/01_FkChainTail.usda",
    "examples/02_TwoBoneIkLeg.usda",
    "examples/03_IkFkBlendClamp.usda",
    "examples/05_TwistRibbonSpine.usda",
    "examples/ArmRig.usda",
    "examples/components/spider_leg.usd",
    "examples/components/spider_leg_ik.usd",
)

# Posed joint origins measured on the unmigrated assets with the ABSOLUTE
# rest semantics, before computeRestFrame gained its ancestor input. The
# migration exists to leave exactly these unchanged.
_GOLDEN = {
    "examples/01_FkChainTail.usda": (
        ("/TailAsset/Rig/Joints/Seg1", (0.0, 5.0, 0.0)),
        ("/TailAsset/Rig/Joints/Seg1/Seg2", (2.0, 5.0, 0.0)),
        ("/TailAsset/Rig/Joints/Seg1/Seg2/Seg3", (4.0, 5.0, 0.0)),
        ("/TailAsset/Rig/Joints/Seg1/Seg2/Seg3/Seg4", (6.0, 5.0, 0.0)),
    ),
    "examples/ArmRig.usda": (
        ("/ArmAsset/Rig/Joints/Shoulder", (0.0, 10.0, 0.0)),
        ("/ArmAsset/Rig/Joints/Shoulder/Elbow", (4.0, 10.0, 0.0)),
        ("/ArmAsset/Rig/Joints/Shoulder/Elbow/Wrist", (8.0, 10.0, 0.0)),
    ),
    "examples/components/spider_leg_ik.usd": (
        ("/RigRoot/Joints/Shoulder", (0.0, 4.5459, 0.0)),
        ("/RigRoot/Joints/Shoulder/ankle", (3.8891, 1.9766, 0.0)),
        ("/RigRoot/Joints/Shoulder/ankle/foot", (6.8811, 0.0, 0.0)),
    ),
}


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecJoint"),
           "RigExecJoint schema is not registered")


def _RigPath(stage):
    roots = [p.GetPath() for p in stage.Traverse()
             if p.GetTypeName() == "RigExecRoot"]
    _Check(len(roots) == 1, "expected exactly one RigExecRoot, got %d"
           % len(roots))
    return str(roots[0])


def _Open(relative):
    stage = Usd.Stage.Open(os.path.join(_REPO, relative))
    _Check(stage is not None, "could not open %s" % relative)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    return stage


def _PosedOrigins(stage):
    import _rigexec
    rig = _rigexec.Rig(stage, _RigPath(stage))
    rig.compile()
    time = (stage.GetStartTimeCode()
            if stage.HasAuthoredTimeCodeRange() else 0.0)
    pose = rig.evaluate(time)
    return {j: tuple(pose.joint_frame(j, True).to_matrix4()[12:15])
            for j in pose.joint_paths()}


def _Fixture(absolutePositions):
    """
    A chain of nested joints authored the OLD way: each joint's rest is
    its absolute asset-space position, not an offset from its parent.
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Rig", "RigExecRoot")
    path = "/Rig/Joints"
    stage.DefinePrim(path, "Scope")
    for index, position in enumerate(absolutePositions):
        path = "%s/J%d" % (path, index)
        prim = stage.DefinePrim(path, "RigExecJoint")
        space = Gf.Matrix4d(1.0)
        space.SetTranslate(Gf.Vec3d(*position))
        prim.CreateAttribute("rest:space", Sdf.ValueTypeNames.Matrix4d)\
            .Set(space)
    return stage


_ABSOLUTE = ((0.0, 5.0, 0.0), (2.0, 5.0, 0.0), (4.0, 5.0, 0.0),
             (6.0, 5.0, 0.0))


def TestMigrationPreservesWorldRests():
    """The rebake holds every provider's world rest frame fixed."""
    stage = _Fixture(_ABSOLUTE)
    before = migrateRestToLocal.AbsoluteWorldRests(stage)
    rewritten = migrateRestToLocal.MigrateStage(stage)
    _Check(rewritten == 3, "rebased %d provider(s), expected 3" % rewritten)
    after = migrateRestToLocal.LocalWorldRests(stage)
    _Check(set(after) == set(before), "the provider set changed")
    for provider, frame in after.items():
        for row in range(4):
            for col in range(4):
                _Check(abs(frame[row][col]
                           - before[provider][row][col]) < 1e-9,
                       "%s rest [%d][%d] moved %.12f -> %.12f"
                       % (provider, row, col,
                          before[provider][row][col], frame[row][col]))


def TestMigrationWritesParentRelativeOffsets():
    """A chain of 2-unit steps becomes a root plus three (2,0,0) offsets."""
    stage = _Fixture(_ABSOLUTE)
    migrateRestToLocal.MigrateStage(stage)
    expected = ((0.0, 5.0, 0.0), (2.0, 0.0, 0.0), (2.0, 0.0, 0.0),
                (2.0, 0.0, 0.0))
    path = "/Rig/Joints"
    for index, want in enumerate(expected):
        path = "%s/J%d" % (path, index)
        local = migrateRestToLocal.ComposedRest(
            stage.GetPrimAtPath(path)).ExtractTranslation()
        for axis in range(3):
            _Check(abs(local[axis] - want[axis]) < 1e-9,
                   "%s local rest is %s, expected %s" % (path, local, want))


def TestMigratedExamplesMatchGoldens():
    """The shipped assets evaluate to what the old absolute semantics gave."""
    for relative, expected in sorted(_GOLDEN.items()):
        stage = _Open(relative)
        _Check(migrateRestToLocal.IsMigrated(stage),
               "%s is not stamped; run tools/migrateRestToLocal.py"
               % relative)
        origins = _PosedOrigins(stage)
        for joint, want in expected:
            _Check(joint in origins, "%s: %s has no posed frame"
                   % (relative, joint))
            got = origins[joint]
            for axis in range(3):
                _Check(abs(got[axis] - want[axis]) < 1e-3,
                       "%s: %s axis %d is %.6f, the old semantics gave %.6f"
                       % (relative, joint, axis, got[axis], want[axis]))


def TestEveryShippedExampleIsStamped():
    """No shipped rig is left on the old semantics."""
    for relative in _EXAMPLES:
        _Check(migrateRestToLocal.IsMigrated(_Open(relative)),
               "%s carries no rigExec:restFrameVersion stamp" % relative)


def TestMigrationIsIdempotent():
    """The version stamp makes a second run a no-op."""
    stage = _Fixture(_ABSOLUTE)
    _Check(migrateRestToLocal.MigrateStage(stage) == 3,
           "first migration did not rebase the chain")
    frames = migrateRestToLocal.LocalWorldRests(stage)
    second = migrateRestToLocal.MigrateStage(stage)
    _Check(second == 0, "second migration rewrote %d provider(s); the "
                        "version stamp did not suppress it" % second)
    again = migrateRestToLocal.LocalWorldRests(stage)
    for provider, frame in again.items():
        moved = (Gf.Vec3d(frame.ExtractTranslation())
                 - Gf.Vec3d(frames[provider].ExtractTranslation()))
        _Check(moved.GetLength() < 1e-12,
               "a second migration moved %s by %s" % (provider, moved))


def TestTopLevelProviderKeepsItsRest():
    """A provider with no frame-provider ancestor is left alone."""
    stage = _Fixture(_ABSOLUTE)
    root = stage.GetPrimAtPath("/Rig/Joints/J0")
    _Check(migrateRestToLocal.ParentProvider(root) is None,
           "J0 unexpectedly has a frame-provider ancestor")
    before = migrateRestToLocal.ComposedRest(root)
    migrateRestToLocal.MigrateStage(stage)
    after = migrateRestToLocal.ComposedRest(root)
    for row in range(4):
        for col in range(4):
            _Check(abs(after[row][col] - before[row][col]) < 1e-12,
                   "the top-level J0 rest was rewritten")


if __name__ == "__main__":
    _RegisterSchema()
    TestMigrationPreservesWorldRests()
    TestMigrationWritesParentRelativeOffsets()
    TestMigratedExamplesMatchGoldens()
    TestEveryShippedExampleIsStamped()
    TestMigrationIsIdempotent()
    TestTopLevelProviderKeepsItsRest()
    print("test_rest_migration: OK")
