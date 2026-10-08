#!/usr/bin/env python
"""
Headless regression test: editing a stage that carries a compiled RigExec
evaluator must not post USD errors, and must not leave the evaluator
permanently broken.

Removing a prim on a stage with a compiled evaluator used to raise
"Applying predicate to invalid prim" out of stage.RemovePrim(): OpenExec's
EsfUsdStageData::_UpdateForResync applied UsdPrimDefaultPredicate to the
prim at the resynced path without first checking that the prim still
existed (pxr/exec/esfUsd/stageData.cpp). Every script or panel that
removed a prim in usdview with the RigExec plugin active hit it.

Usage: test_rigexec_stage_edits.py [<generated schema resources dir>]
Requires the native _rigexec binding (build-python/python on PYTHONPATH).
"""
import os
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Plug, Sdf, Tf, Usd  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXAMPLE = os.path.normpath(os.path.join(
    _HERE, "..", "..", "examples", "ArmShotAnim.usda"))


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered")


def TestRemovePrimWithCompiledEvaluator():
    import _rigexec
    stage = Usd.Stage.Open(_EXAMPLE)
    _Check(stage, "example stage opens")
    rig = _rigexec.Rig(stage, "/Shot/HeroArm/Rig")
    rig.compile()
    rig.evaluate(1001.0)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    child = stage.DefinePrim("/Shot/HeroArm/StageEditsChild", "Xform")
    _Check(child, "child defined")
    try:
        stage.RemovePrim(child.GetPath())
    except Tf.ErrorException as error:
        raise AssertionError(
            "RemovePrim posted an error with a compiled evaluator "
            "attached: %s" % str(error).strip().splitlines()[-1])
    _Check(not stage.GetPrimAtPath("/Shot/HeroArm/StageEditsChild"),
           "child removed")
    # The evaluator still works afterwards.
    rig.evaluate(1002.0)


def TestDeactivateReactivateRecovers():
    """An unavailable required joint refuses the pose and later recovers.

    The authored solver and frame reads require this Shoulder hierarchy.
    Refusal publishes no stale pose; reactivation restores each joint frame,
    including at a later scrub time.
    """
    import _rigexec
    stage = Usd.Stage.Open(_EXAMPLE)
    _Check(stage, "example stage opens")
    rig = _rigexec.Rig(stage, "/Shot/HeroArm/Rig")
    rig.compile()
    before = rig.evaluate(1001.0)
    _Check(before.valid and len(before.joint_paths()) == 3,
           "baseline evaluate: valid=%s joints=%d"
           % (before.valid, len(before.joint_paths())))

    expected = {1001.0: before, 1024.0: rig.evaluate(1024.0)}
    _Check(expected[1024.0].valid, "later baseline evaluate is valid")
    control = "/Shot/HeroArm/Rig/Controls/HandIK"
    _Check(control in before.control_paths(), "unaffected control is published")

    joint = stage.GetPrimAtPath("/Shot/HeroArm/Rig/Joints/Shoulder")
    _Check(joint, "shoulder joint exists")
    joint.SetActive(False)
    down = rig.evaluate(1001.0)
    _Check(not down.valid and not down.joint_paths(),
           "an unavailable required joint refuses the pose")
    _Check(not down.control_paths() and not down.provider_paths() and
           not down.moved_properties(),
           "required-provider refusal publishes no stale controls, providers, or geometry")
    _Check(any("failed to prepare pose provider inputs" in message
               for message in down.diagnostics),
           "required-provider refusal is reported")
    diagnostics = list(down.diagnostics)
    _Check(
        "Mover /Shot/HeroArm/Rig/Movers/Pose/WristAim targets missing prim "
        "/Shot/HeroArm/Rig/Joints/Shoulder/Elbow/Wrist" in diagnostics and
        "RigExecTwoBoneIk /Shot/HeroArm/Rig/Solvers/IK rigExec:joints "
        "targets missing prim /Shot/HeroArm/Rig/Joints/Shoulder/Elbow"
        in diagnostics and
        "/Shot/HeroArm/Rig/Solvers/IKFKBlend reads skipped operation "
        "/Shot/HeroArm/Rig/Solvers/IK" in diagnostics,
        "unavailable mover and solver inputs, and their dependent skip, "
        "are identified: %r" % diagnostics)

    joint.SetActive(True)
    for time in (1001.0, 1024.0):
        after = rig.evaluate(time)
        _Check(after.valid and len(after.joint_paths()) == 3,
               "evaluate at %g after reactivating: valid=%s joints=%d"
               % (time, after.valid, len(after.joint_paths())))
        _Check(after.joint_paths() == expected[time].joint_paths() and
               all(after.joint_frame(path).to_matrix4() ==
                   expected[time].joint_frame(path).to_matrix4()
                   for path in after.joint_paths()),
               "reactivated joint frames exactly match the pre-edit pose at %g"
               % time)


def TestBrokenStageIsNotRecompiledEveryFrame():
    """A stage left uncompilable fails from memory until something changes.

    A structural edit with no operation to skip -- here authored mesh normals
    on a Points prim -- used to recompile on every frame of the
    scrub that followed: a full compile per frame for the same answer. The
    failure is remembered against the stage-edit serial, so later frames
    report the same diagnostics without compiling. Independent reference
    checking does not change compilation; a repairing stage edit does.
    """
    import _rigexec
    stage = Usd.Stage.Open(_EXAMPLE)
    _Check(stage, "example stage opens")
    rig = _rigexec.Rig(stage, "/Shot/HeroArm/Rig")
    rig.cpu_reference = True
    rig.compile()
    rig.profiling_enabled = True
    _Check(rig.evaluate(1001.0).valid, "baseline evaluate is valid")

    def Frame(time):
        rig.clear_profile()
        pose = rig.evaluate(time)
        counts = {}
        for row in rig.profile_summary():
            counts[row["name"]] = counts.get(row["name"], 0) + row["count"]
        return (pose, counts.get("Compile", 0),
                counts.get("Settle.KnownBroken", 0))

    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    geometry = stage.GetPrimAtPath("/Shot/HeroArm/Geom/ArmBody")
    _Check(geometry, "deformed mesh exists")
    original_type = geometry.GetTypeName()
    geometry.SetTypeName("Points")

    broken, compiles, _ = Frame(1002.0)
    _Check(not broken.valid, "the broken stage evaluates invalid")
    _Check(compiles == 1,
           "the first broken frame compiles once (got %d)" % compiles)
    diagnostics = list(broken.diagnostics)
    _Check(any("authored normals on non-mesh points target" in message
               for message in diagnostics),
           "the broken frame identifies the invalid geometry: %r" % diagnostics)
    for time in (1003.0, 1004.0):
        pose, compiles, remembered = Frame(time)
        _Check(not pose.valid and compiles == 0 and remembered == 1,
               "frame %g on the unchanged broken stage is answered without "
               "compiling: valid=%s compiles=%d" % (time, pose.valid,
                                                      compiles))
        _Check(list(pose.diagnostics) == diagnostics,
               "frame %g repeats the compile's diagnostics: %r"
               % (time, list(pose.diagnostics)))

    # Reference checking is an observer option, not a compilation mode.
    rig.cpu_reference = False
    pose, compiles, remembered = Frame(1005.0)
    _Check(not pose.valid and compiles == 0 and remembered == 1 and
           list(pose.diagnostics) == diagnostics,
           "changing the reference observer preserves the compile failure: "
           "valid=%s compiles=%d" % (pose.valid, compiles))
    rig.cpu_reference = False
    pose, compiles, remembered = Frame(1006.0)
    _Check(not pose.valid and compiles == 0 and remembered == 1 and
           list(pose.diagnostics) == diagnostics,
           "repeating the observer option also preserves the failure: "
           "compiles=%d" % compiles)

    # The repair is a stage edit: the memo no longer answers for it.
    geometry.SetTypeName(original_type)
    for time in (1007.0, 1008.0):
        pose, compiles, remembered = Frame(time)
        _Check(pose.valid and len(pose.joint_paths()) == 3 and
               remembered == 0,
               "frame %g after the repair evaluates: valid=%s joints=%d"
               % (time, pose.valid, len(pose.joint_paths())))


def TestEditsTheDigestNeverReadSkipIt():
    """An edit the structure digest cannot see never pays for it.

    The digest records the prims it read outside the rig -- mover target
    meshes, weight targets, the ancestors its chains walked -- and a notice
    that touches none of them, and nothing in or above the rig, leaves the
    epoch alone without recomputing it. An edit on a prim it did read still
    recomputes it, and recompiles when the digest moved.
    """
    import _rigexec
    stage = Usd.Stage.Open(_EXAMPLE)
    _Check(stage, "example stage opens")
    rig = _rigexec.Rig(stage, "/Shot/HeroArm/Rig")
    rig.cpu_reference = True
    rig.compile()
    rig.profiling_enabled = True
    _Check(rig.evaluate(1001.0).valid, "baseline evaluate is valid")

    def Frame(time):
        rig.clear_profile()
        pose = rig.evaluate(time)
        counts = {}
        for row in rig.profile_summary():
            counts[row["name"]] = counts.get(row["name"], 0) + row["count"]
        digests = sum(count for name, count in counts.items()
                      if name.startswith("Digest."))
        return pose, digests, counts.get("Compile", 0)

    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    # Beside the asset: nothing the digest read is on, under or above it.
    bystander = stage.DefinePrim("/Shot/StageEditsBystander", "Xform")
    bystander.CreateAttribute("custom:value", Sdf.ValueTypeNames.Float).Set(
        1.0)
    pose, digests, compiles = Frame(1002.0)
    _Check(pose.valid and digests == 0 and compiles == 0,
           "an edit beside the asset skips the digest: valid=%s digests=%d "
           "compiles=%d" % (pose.valid, digests, compiles))
    # Beside a mesh a mover writes, under the asset root every chain walks
    # through: an ancestor it only walked is not a prim it read below.
    sibling = stage.DefinePrim("/Shot/HeroArm/Geom/StageEditsSibling", "Xform")
    sibling.CreateAttribute("custom:value", Sdf.ValueTypeNames.Float).Set(
        2.0)
    pose, digests, compiles = Frame(1003.0)
    _Check(pose.valid and digests == 0 and compiles == 0,
           "an edit beside a read mesh skips the digest: valid=%s "
           "digests=%d compiles=%d" % (pose.valid, digests, compiles))

    # On the mesh the movers write: a first-time widths spec changes what
    # the compiler would synthesize for it, so the digest runs and moves.
    body = stage.GetPrimAtPath("/Shot/HeroArm/Geom/ArmBody")
    _Check(body, "ArmBody exists")
    body.CreateAttribute("widths", Sdf.ValueTypeNames.FloatArray).Set(
        [0.1, 0.1, 0.1, 0.1])
    pose, digests, compiles = Frame(1004.0)
    _Check(pose.valid and digests > 0 and compiles == 1,
           "an edit on a read mesh recompiles: valid=%s digests=%d "
           "compiles=%d" % (pose.valid, digests, compiles))
    _Check("structural edit: epoch rebuilt" in list(pose.diagnostics),
           "the read mesh's edit says why: %r" % list(pose.diagnostics))
    pose, digests, compiles = Frame(1005.0)
    _Check(pose.valid and digests == 0 and compiles == 0,
           "and the frame after it settles for free: digests=%d compiles=%d"
           % (digests, compiles))


def main():
    _RegisterSchema()
    groups = [
        ("remove prim with compiled evaluator",
         TestRemovePrimWithCompiledEvaluator),
        ("deactivate and reactivate recovers",
         TestDeactivateReactivateRecovers),
        ("broken stage is not recompiled every frame",
         TestBrokenStageIsNotRecompiledEveryFrame),
        ("edits the digest never read skip it",
         TestEditsTheDigestNeverReadSkipIt),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_STAGE_EDITS_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
