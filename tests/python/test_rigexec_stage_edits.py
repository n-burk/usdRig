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

from pxr import Plug, Tf, Usd  # noqa: E402

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


def main():
    _RegisterSchema()
    groups = [
        ("remove prim with compiled evaluator",
         TestRemovePrimWithCompiledEvaluator),
        ("deactivate and reactivate recovers",
         TestDeactivateReactivateRecovers),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_STAGE_EDITS_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
