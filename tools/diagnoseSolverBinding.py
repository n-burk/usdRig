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
  * every joint MORE THAN ONE solver writes, because rigExec:joints is an
    ordered write rather than an exclusive claim: all of those solvers run
    and the last one in the stack supplies the joint's frame, so "my solver
    stopped driving this joint" can mean "another solver writes it after
    mine" rather than anything being broken;
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


_SOLVER_TYPES = ("RigExecFkChain", "RigExecTwoBoneIk",
                 "RigExecBlendPointFrames", "RigExecTwistDistribution",
                 "RigExecRibbon", "RigExecSplineIk")


def _Stacks(rig):
    """Joint path -> the solver paths that WRITE it, for joints with >1.

    Discovery order only, NOT stack order: the compiler decides that from
    the solver DAG first and the reverse composed pre-order second, and
    restating that rule here is how a tool starts disagreeing with the
    engine. The compile note is what names the order.

    The one compiler rule this DOES restate, because without it the report
    is actively wrong on every IK/FK arm in the tree: a solver whose
    aggregate another solver reads does not write the joints that consumer
    writes. Its rigExec:joints is a REST reference there -- which is why an
    IK feeding a blend is one writer and not two.
    """
    from pxr import Usd

    solvers = [p for p in Usd.PrimRange(rig)
               if p.GetTypeName() in _SOLVER_TYPES]
    paths = {str(p.GetPath()) for p in solvers}
    consumed = set()
    for solver in solvers:
        for rel in solver.GetRelationships():
            if rel.GetName() == "rigExec:joints":
                continue
            for target in rel.GetTargets():
                prim = str(target.GetPrimPath())
                if prim != str(solver.GetPath()) and prim in paths:
                    consumed.add(prim)

    named = {str(p.GetPath()): [str(t) for t in
                                (p.GetRelationship("rigExec:joints")
                                 .GetTargets()
                                 if p.GetRelationship("rigExec:joints")
                                 else [])]
             for p in solvers}
    byUnconsumed = set()
    for solver, joints in named.items():
        if solver not in consumed:
            byUnconsumed.update(joints)

    writers = {}
    for solver in solvers:
        path = str(solver.GetPath())
        for joint in named[path]:
            if path in consumed and joint in byUnconsumed:
                continue  # a rest reference, not a write
            writers.setdefault(joint, []).append(path)
    return {joint: who for joint, who in writers.items() if len(who) > 1}


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

        stacks = _Stacks(rig)
        if stacks:
            print("  joints written by more than one solver (the LAST writer "
                  "supplies the frame; the compile note names the order):")
            for joint in sorted(stacks):
                print("    %s  <- %s" % (joint, ", ".join(stacks[joint])))
            print("    A solver whose aggregate ANOTHER solver reads "
                  "does not write the joints that consumer writes, so the "
                  "IK/FK idiom is one writer and is not listed above.")

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
