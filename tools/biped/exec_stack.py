#!/usr/bin/env python
"""List a RigExec rig's execution stack, with filtering.

Answers "what runs, in what order, and what does it write" -- the question
that actually matters when a rig misbehaves. Ordering bugs here are silent
and expensive: a skin mover authored after the constraints runs BEFORE them
(chains apply in reverse add order), so it skins with pre-pose joint
transforms and the mesh is quietly wrong by centimetres while every joint
reads correct.

Two sections, because RigExec has two evaluation stages:

  SOLVERS  pose phase. Aggregate solvers (FK chains, two-bone IK, blends,
           twist distributions, ribbons) claim joints through
           `rigExec:joints` and publish their frames. They are scheduled by
           dependency, not by a linear order, so they are listed grouped by
           type with what each claims -- there is no meaningful "step 3 of
           7" for them.
  MOVERS   `Rig.mover_order`, which IS a strict order: the composed movers
           in execution order (reverse-sibling post-order, spec 4.2). This
           is the list to read when something deforms wrongly.

Usage:
    exec_stack.py <rig.usda> [--filter FILTER] [--type SUBSTR]
                             [--target SUBSTR] [--rig-root /Biped/Rig]

    --filter  all | movers | constraints | solvers | deformers | weights
              | skin | ik | fk
    --type    keep only entries whose schema type contains this (case
              insensitive), e.g. --type aim
    --target  keep only entries writing a path containing this, e.g.
              --target body_geo, or --target elbow
"""
import argparse
import os
import sys

from pxr import Usd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rigexec

# Classification by schema type. A type can belong to more than one group --
# a single-chain IK is both a constraint and IK, which is exactly the sort
# of thing you want to be able to filter either way.
GROUPS = {
    "constraints": ("Constraint",),
    "deformers": ("MatrixMover", "SkinMover", "CurveMover", "LatticeMover",
                  "SurfaceMover", "SmoothMover", "BlendShapeMover",
                  "VolumeCorrectMover", "CurvenetMover",
                  "CurvenetAdjusterMover"),
    "weights": ("Weight",),
    "skin": ("SkinMover", "MatrixMover"),
    "ik": ("TwoBoneIk", "SingleChainIk"),
    "fk": ("FkChain",),
    "solvers": ("FkChain", "TwoBoneIk", "BlendPointFrames",
                "TwistDistribution", "Ribbon"),
}

SOLVER_TYPES = ("RigExecFkChain", "RigExecTwoBoneIk",
                "RigExecBlendPointFrames", "RigExecTwistDistribution",
                "RigExecRibbon")


def matches(entry_type, group):
    if group in (None, "all"):
        return True
    if group == "movers":
        # Everything in mover_order is a mover; the filter is a no-op there
        # but excludes the solver section.
        return True
    for token in GROUPS.get(group, ()):
        if token.lower() in entry_type.lower():
            return True
    return False


def short(path, keep=2):
    """Trim a long joint path to its tail, which is what identifies it."""
    parts = str(path).split("/")
    return "/".join(parts[-keep:]) if len(parts) > keep else str(path)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rig")
    ap.add_argument("--filter", default="all",
                    choices=("all", "movers", "constraints", "solvers",
                             "deformers", "weights", "skin", "ik", "fk"))
    ap.add_argument("--type", default=None,
                    help="keep entries whose schema type contains this")
    ap.add_argument("--target", default=None,
                    help="keep entries writing a path containing this")
    ap.add_argument("--rig-root", default="/Biped/Rig")
    ap.add_argument("--full-paths", action="store_true",
                    help="print untrimmed target paths")
    args = ap.parse_args(argv)

    rigexec.load_schema_plugin()
    stage = Usd.Stage.Open(args.rig)
    rig = rigexec.Rig(stage, args.rig_root)
    rig.compile()

    keep_type = (args.type or "").lower()
    keep_target = (args.target or "").lower()

    def wanted(etype, targets):
        if keep_type and keep_type not in etype.lower():
            return False
        if keep_target and not any(keep_target in str(t).lower()
                                   for t in targets):
            return False
        return True

    # --- pose phase: solvers, grouped by type ---
    if args.filter in ("all", "solvers", "ik", "fk"):
        solvers = []
        for prim in stage.Traverse():
            t = prim.GetTypeName()
            if t not in SOLVER_TYPES:
                continue
            rel = prim.GetRelationship("rigExec:joints")
            claims = [str(x) for x in rel.GetTargets()] if rel else []
            if not matches(t, args.filter if args.filter != "all" else None):
                continue
            if not wanted(t, claims):
                continue
            solvers.append((t, prim.GetName(), claims))

        print("=== POSE PHASE: solvers (scheduled by dependency, not "
              "linearly) ===")
        if not solvers:
            print("  (none matching)")
        for t, name, claims in sorted(solvers):
            shown = claims if args.full_paths else [short(c, 1)
                                                    for c in claims]
            print("  %-26s %-22s claims %d: %s"
                  % (t.replace("RigExec", ""), name, len(claims),
                     ", ".join(shown[:6]) + (" ..." if len(shown) > 6
                                             else "")))
        print()

    # --- the ordered part ---
    order = rig.mover_order
    if callable(order):
        order = order()

    rows = []
    for e in order:
        etype, targets = e["type"], e.get("targets") or []
        if not matches(etype, args.filter if args.filter not in
                       ("all", "movers") else None):
            continue
        if not wanted(etype, targets):
            continue
        rows.append(e)

    print("=== EXECUTION ORDER: %d of %d movers ===" % (len(rows),
                                                        len(order)))
    print("  %-4s %-26s %-30s %s" % ("#", "type", "name", "writes"))
    for e in rows:
        targets = e.get("targets") or []
        shown = ([str(t) for t in targets] if args.full_paths
                 else [short(t, 1) for t in targets])
        print("  %-4s %-26s %-30s %s"
              % (e["ordinal"], e["type"].replace("RigExec", ""),
                 e["path"].rsplit("/", 1)[-1],
                 ", ".join(shown[:3]) + (" ..." if len(shown) > 3 else "")))

    # The ordering rule that bites: a mover reading FINAL joint transforms
    # must come after every constraint that writes a joint. Flag the shape
    # of the stack so a regression is visible at a glance.
    deform = [e for e in order
              if any(tok.lower() in e["type"].lower()
                     for tok in ("SkinMover", "MatrixMover"))]
    constraints = [e for e in order if "Constraint" in e["type"]]
    if deform and constraints:
        first_deform = min(e["ordinal"] for e in deform)
        last_constraint = max(e["ordinal"] for e in constraints)
        print()
        if first_deform > last_constraint:
            print("  ok: every deformer (first at %d) runs after every "
                  "constraint (last at %d), so a deformer reading final "
                  "joint transforms sees them posed."
                  % (first_deform, last_constraint))
        else:
            print("  WARNING: a deformer runs at ordinal %d, before a "
                  "constraint at %d. A deformer reading FINAL joint "
                  "transforms will skin with pre-pose frames."
                  % (first_deform, last_constraint))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
