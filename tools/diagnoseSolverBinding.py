#!/usr/bin/env python
"""
Why did a solver stop driving its joints?

Run this against the stage that reproduces the problem -- either from
usdview's interpreter (the stage is already open) or standalone on a saved
layer. It reports, per RigExec solver:

  * every relationship target, flagging the two shapes that break binding:
    a PROPERTY path (`/Rig/Joints/Hip.rest:tx` instead of `/Rig/Joints/Hip`)
    and a target whose prim does not resolve;
  * the joint order, because a TwoBoneIk reads rigExec:joints positionally
    as (root, mid, end) -- rebinding one joint appends it to the END;
  * the compile result, including the diagnostics the viewport discards.

Usage:
    # standalone
    diagnoseSolverBinding.py <stage.usd> [rigPath]

    # inside usdview's interpreter
    exec(open(".../tools/diagnoseSolverBinding.py").read())
    Diagnose(usdviewApi.stage)
"""
import os
import sys

_SOLVER_RELS = ("rigExec:joints", "rigExec:effectorControl",
                "rigExec:poleControl", "rigExec:rootControl",
                "rigExec:controls", "rigExec:moves")


def _Bootstrap():
    """Make pxr and _rigexec importable when run outside usdview."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    sys.path.insert(0, os.path.join(repo, "tests", "python"))
    import rigexec_test_env
    rigexec_test_env.SetupPluginTest()
    sys.path.insert(0, os.path.join(repo, "build-python", "python"))
    schema = os.path.join(repo, "build", "usd", "rigExecSchema", "resources")
    from pxr import Plug
    if os.path.isdir(schema):
        Plug.Registry().RegisterPlugins(schema)


def Diagnose(stage, rigPath=None):
    """Report solver binding health for every rig on `stage`."""
    from pxr import Usd

    problems = 0
    rigs = [p for p in stage.Traverse() if p.GetTypeName() == "RigExecRoot"]
    if rigPath:
        rigs = [r for r in rigs if str(r.GetPath()) == str(rigPath)]
    if not rigs:
        print("no RigExecRoot found")
        return 1

    for rig in rigs:
        print("rig %s" % rig.GetPath())
        for prim in Usd.PrimRange(rig):
            authored = [r for r in prim.GetRelationships()
                        if r.GetName() in _SOLVER_RELS
                        and r.HasAuthoredTargets()]
            if not authored:
                continue
            print("  %s <%s>" % (prim.GetPath(), prim.GetTypeName()))
            for rel in authored:
                targets = list(rel.GetTargets())
                print("    %s  (%d target(s))" % (rel.GetName(), len(targets)))
                for i, target in enumerate(targets):
                    notes = []
                    if target.IsPrimPropertyPath():
                        notes.append(
                            "PROPERTY PATH -- binding needs the bare prim "
                            "path; this fails to compile")
                    elif not target.IsPrimPath():
                        notes.append("not a prim path")
                    if not stage.GetPrimAtPath(target.GetPrimPath()):
                        notes.append("prim does not resolve")
                    role = ""
                    if rel.GetName() == "rigExec:joints" and len(targets) == 3:
                        role = " [%s]" % ("root", "mid", "end")[i]
                    print("      %d: %s%s%s" % (
                        i, target, role,
                        "".join("\n         !! " + n for n in notes)))
                    problems += len(notes)

        try:
            import _rigexec
        except ImportError:
            print("  (_rigexec not importable; skipping the compile check)")
            continue
        try:
            evaluator = _rigexec.Rig(stage, str(rig.GetPath()))
            evaluator.compile()
        except Exception as error:      # noqa: BLE001 - report anything
            print("  COMPILE FAILED: %s" % error)
            print("  The viewport discards this message: the imaging "
                  "registry's change handler calls SetTime() with no error "
                  "sink, then clears the published generation -- so the rig "
                  "silently stops driving and everything falls back to its "
                  "authored rest pose.")
            return problems + 1
        pose = evaluator.evaluate(
            stage.GetStartTimeCode() if stage.HasAuthoredTimeCodeRange() else 0.0)
        print("  compiles; evaluate diagnostics:")
        for line in pose.diagnostics:
            print("    %s" % line)
            if "binds fewer than three elements" in line:
                problems += 1

    print("\n%d problem(s) found" % problems)
    return problems


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    _Bootstrap()
    from pxr import Usd
    _stage = Usd.Stage.Open(sys.argv[1])
    if not _stage:
        print("could not open %s" % sys.argv[1])
        sys.exit(2)
    sys.exit(1 if Diagnose(_stage, sys.argv[2] if len(sys.argv) > 2 else None) else 0)
