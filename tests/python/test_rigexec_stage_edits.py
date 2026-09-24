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
    """A request invalidated by a structural edit is rebuilt, not abandoned.

    Deactivating a prim the compiled exec requests read expires those
    requests. Reactivating it restores the composed stage, but an evaluator
    that only rebuilds a request when it thinks it is unprepared never
    notices, and every later evaluate returns an empty pose for the rest of
    the session. RigExecTapSet::Evaluate therefore rebuilds on expiry as
    well, and this asserts the recovery -- including at a later time code,
    which is what a scrub after the edit looks like.
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

    joint = stage.GetPrimAtPath("/Shot/HeroArm/Rig/Joints/Shoulder")
    _Check(joint, "shoulder joint exists")
    joint.SetActive(False)
    # Deactivated, the rig cannot pose: that is the expected, reported state.
    down = rig.evaluate(1001.0)
    _Check(not down.valid and not down.joint_paths(),
           "evaluate with the joint deactivated reports an empty pose")

    joint.SetActive(True)
    for time in (1001.0, 1024.0):
        after = rig.evaluate(time)
        _Check(after.valid and len(after.joint_paths()) == 3,
               "evaluate at %g after reactivating: valid=%s joints=%d"
               % (time, after.valid, len(after.joint_paths())))


def TestBrokenStageIsNotRecompiledEveryFrame():
    """A stage left uncompilable fails from memory until something changes.

    A structural edit the rig cannot compile -- here a joint target that
    does not exist -- used to recompile, and fail, on every frame of the
    scrub that followed: a full compile per frame for the same answer. The
    failure is now remembered against the stage-edit serial and the mode,
    so every later frame reports the same diagnostics without compiling,
    and the two things that can change the answer both get a real compile:
    switching the evaluation mode, and a stage edit (here, the repair).
    """
    import _rigexec
    stage = Usd.Stage.Open(_EXAMPLE)
    _Check(stage, "example stage opens")
    rig = _rigexec.Rig(stage, "/Shot/HeroArm/Rig")
    rig.evaluation_mode = "baked"
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
    rel = stage.GetPrimAtPath(
        "/Shot/HeroArm/Rig/Solvers/IK").GetRelationship("rigExec:joints")
    _Check(rel, "IK joints relationship exists")
    original = rel.GetTargets()
    rel.SetTargets(original + [Sdf.Path("/Shot/HeroArm/StageEditsNope")])

    broken, compiles, _ = Frame(1002.0)
    _Check(not broken.valid, "the broken stage evaluates invalid")
    _Check(compiles == 1,
           "the first broken frame compiles once (got %d)" % compiles)
    diagnostics = list(broken.diagnostics)
    _Check(diagnostics and
           diagnostics[-1] == "structural recompilation failed",
           "the broken frame says why: %r" % diagnostics)
    for time in (1003.0, 1004.0):
        pose, compiles, remembered = Frame(time)
        _Check(not pose.valid and compiles == 0 and remembered == 1,
               "frame %g on the unchanged broken stage is answered without "
               "compiling: valid=%s compiles=%d" % (time, pose.valid,
                                                      compiles))
        _Check(list(pose.diagnostics) == diagnostics,
               "frame %g repeats the compile's diagnostics: %r"
               % (time, list(pose.diagnostics)))

    # The mode decides what a compile prepares, and so what can fail; it
    # moves with no stage notice, so switching it has to compile again.
    rig.evaluation_mode = "dynamic"
    pose, compiles, remembered = Frame(1005.0)
    _Check(not pose.valid and compiles == 1 and remembered == 0,
           "a mode switch while broken compiles again: valid=%s "
           "compiles=%d" % (pose.valid, compiles))
    pose, compiles, remembered = Frame(1006.0)
    _Check(not pose.valid and compiles == 0 and remembered == 1,
           "and the frame after it is remembered: compiles=%d" % compiles)

    # The repair is a stage edit: the memo no longer answers for it.
    rel.SetTargets(original)
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
    rig.evaluation_mode = "baked"
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
