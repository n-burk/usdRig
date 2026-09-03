#!/usr/bin/env python
"""
Headless regression test: editing a stage that carries a compiled RigExec
evaluator must not post USD errors.

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


def main():
    _RegisterSchema()
    groups = [
        ("remove prim with compiled evaluator",
         TestRemovePrimWithCompiledEvaluator),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_STAGE_EDITS_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
