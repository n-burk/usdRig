#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#
"""What a rig's evaluation schedule looks like, and where its time goes.

Headless: no Qt, so the CLI (`tools/rigexec_schedule.py`) and the usdview
panel (`profilerUI.py`) share one measurement and cannot drift into two
different answers.

A baseline you can run before and after a change and diff. It answers four
questions that are otherwise guesswork:

  1. WHAT IS THE SHAPE of the schedule -- how deep is the pose walk, how
     wide is each level, and which solvers sit where. Depth is the thing
     that cannot be parallelised: a level is a barrier.
  2. WHICH REGIONS ARE ALLOWED TO RUN IN PARALLEL, and for the ones that
     are not, WHY not. RigExec's geometry chain levels carry an explicit
     `parallel` flag and it is usually false for a reason worth knowing.
  3. WHICH ONES ACTUALLY DID. The profiler records the index of the thread
     that ran every scope, so "ran on four threads" is measured rather
     than assumed -- within the limit the docstring on `_threads` states.
  4. WHAT IT COSTS, split by the three workloads that behave completely
     differently:

       compile  the epoch: digest, bake, tap preparation
       replay   evaluate again with nothing changed
       drag     interactive overrides, which is what a manipulator does
       author   set an avar on the stage and evaluate

     `author` is usually dominated by RE-COMPILATION rather than by posing,
     because an authored edit invalidates the compiled program. Timing a rig
     that way and calling the number "evaluation" is the most common way to
     measure this wrong, so it is reported separately and labelled.
"""
import json
import os
import platform
import sys
import time

from pxr import Sdf, Usd

import rigexec


# Scopes worth naming in the cost table even when they are cheap, because
# their presence or absence is the finding.
HEADLINE = ("Evaluate@0", "Compile.Bake", "Compile.DigestJoin",
            "Digest.Solvers", "Digest.Movers", "Digest.OutputSets",
            "Baked", "BakedPrologue", "BakedRegion", "BakedEpilogue",
            "BakedChainBase", "ChainLevel", "PropertyChains",
            "RestFramesRefresh", "Parity")


def _name(path):
    return str(path).rsplit("/", 1)[-1]


def _percentiles(values):
    if not values:
        return {}
    ordered = sorted(values)
    def at(q):
        return ordered[min(len(ordered) - 1, int(q * len(ordered)))]
    return {"best": ordered[0], "p50": at(0.5), "p90": at(0.9),
            "mean": sum(ordered) / len(ordered), "worst": ordered[-1]}


# ---------------------------------------------------------------------------
# Structure
# ---------------------------------------------------------------------------

def schedule_shape(stage, rig, rig_root):
    """Levels, widths and depth: the part no thread count can change."""
    levels = rig.solver_batch_levels()
    by_level = {}
    for path, level in levels.items():
        by_level.setdefault(level, []).append(_name(path))

    movers_path = Sdf.Path(rig_root).AppendChild("Movers")
    movers = stage.GetPrimAtPath(movers_path)
    constraints, chains, math_movers = 0, 0, 0
    if movers:
        for prim in Usd.PrimRange(movers):
            t = prim.GetTypeName()
            if t.endswith("Constraint"):
                constraints += 1
            elif t == "RigExecFloatMathMover":
                math_movers += 1
            elif prim.GetParent() == movers:
                chains += 1

    return {
        "solver_count": len(levels),
        "deepest_solver_level": max(levels.values()) if levels else 0,
        "solver_levels": {str(k): sorted(v)
                          for k, v in sorted(by_level.items())},
        "widest_solver_level": max((len(v) for v in by_level.values()),
                                   default=0),
        "frame_constraints": constraints,
        "mover_chains": chains,
        "float_math_movers": math_movers,
    }


def chain_shape(rig):
    """Geometry chain levels and whether each may run its chains in parallel.

    A level is refused parallelism for a reason, and the common one on a
    character is simply that it has fewer than three chains: one deformed
    mesh is one chain, and dispatching one task to wait on it costs more
    than it saves. Said out loud here so that "parallel: false" does not
    read as a defect.
    """
    out = []
    for level in rig.chain_levels():
        targets = list(level["targets"])
        why = ""
        if not level["parallel"]:
            why = ("fewer than 3 chains in the level"
                   if len(targets) < 3
                   else "chains in this level share a weight object or "
                        "are otherwise not independent")
        out.append({"targets": targets, "parallel": bool(level["parallel"]),
                    "why_not": why})
    return out


# ---------------------------------------------------------------------------
# Cost
# ---------------------------------------------------------------------------

def _drive(stage, rig, control_path, avar):
    attr = stage.GetAttributeAtPath(Sdf.Path(control_path + "." + avar))
    if not attr or not attr.IsValid():
        prim = stage.GetPrimAtPath(Sdf.Path(control_path))
        attr = prim.CreateAttribute(avar, Sdf.ValueTypeNames.Double)
    return attr


def measure(stage, rig, control_path, avar, samples):
    """The three workloads, each timed and profiled on its own."""
    attr = _drive(stage, rig, control_path, avar)
    original = attr.Get()

    def run(label, step, warmup=5):
        for i in range(warmup):
            step(i)
        rig.profiling_enabled = True
        rig.clear_profile()
        times = []
        for i in range(samples):
            t = time.perf_counter()
            step(i)
            times.append((time.perf_counter() - t) * 1000.0)
        events = rig.profile_events()
        rows = rig.profile_summary()
        rig.profiling_enabled = False
        return {
            "ms": _percentiles(times),
            "scopes": {r["name"]: {"count": r["count"],
                                   "ms_per_call": r["total_us"] / 1000.0
                                   / max(1, r["count"]),
                                   "ms_total": r["total_us"] / 1000.0}
                       for r in rows},
            "threads": _threads(events),
        }

    out = {}
    out["replay"] = run("replay", lambda i: rig.evaluate(0.0))

    def drag(i):
        rig.set_interactive_overrides(
            [(control_path, avar, float(i) * 0.1)])
        rig.evaluate(0.0)
    out["drag"] = run("drag", drag)
    rig.clear_interactive_overrides()

    def author(i):
        attr.Set(float(i) * 0.1)
        rig.evaluate(0.0)
    out["author"] = run("author", author)

    if original is None:
        attr.Clear()
    else:
        attr.Set(original)
    rig.evaluate(0.0)
    return out


def _threads(events):
    """Which threads ran PROFILED SCOPES, and how much.

    READ THIS BEFORE BELIEVING THE NUMBER. It counts the threads that
    recorded a scope, and the per-point geometry kernels -- skinning,
    blend shapes, the envelope blend -- split their point range with
    WorkParallelForN INSIDE a single scope, with no scope of their own.
    Those worker threads therefore record nothing, so a skin kernel that
    is spreading 26,000 points across the machine still reports as one
    thread here.

    What the number does tell you is whether the regions that ARE scoped
    ran concurrently: the chain-level dispatch, the compile-time digest
    and the exec warm-up. One thread across the whole of `drag` means no
    scoped region ran in parallel -- which is the true and interesting
    statement about the pose walk.
    """
    per_thread = {}
    per_category = {}
    for event in events:
        tid = int(event["thread"])
        us = int(event["duration_us"])
        per_thread[tid] = per_thread.get(tid, 0) + us
        cat = per_category.setdefault(event["category"], {})
        cat[tid] = cat.get(tid, 0) + us
    return {
        "distinct_threads": len(per_thread),
        "ms_per_thread": {str(k): v / 1000.0
                          for k, v in sorted(per_thread.items())},
        "threads_per_category": {c: sorted(t) for c, t in
                                 sorted(per_category.items())},
    }


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def render(report, top):
    L = []
    add = L.append
    s = report["stage"]
    add("")
    add("RigExec schedule report")
    add("=" * 62)
    add("  stage            : %s" % s["path"])
    add("  rig root         : %s" % s["rig_root"])
    add("  evaluation mode  : %s (%s)" % (s["mode"], s["mode_source"]))
    add("  bakeable         : %s%s" % (
        s["bakeable"], "" if s["bakeable"] else
        "   refusals: %s" % (s["bakeability_reasons"][:2],)))
    add("  parallel eval    : %s   logical cores: %s"
        % (s["parallel_enabled"], s["cores"]))
    if s["thread_limit"]:
        add("  PXR_WORK_THREAD_LIMIT = %s" % s["thread_limit"])

    shape = report["shape"]
    add("")
    add("Schedule shape -- depth is the part threads cannot fix")
    add("-" * 62)
    add("  frame constraints: %d" % shape["frame_constraints"])
    add("  mover chains     : %d" % shape["mover_chains"])
    add("  float math movers: %d" % shape["float_math_movers"])
    add("  solvers          : %d, deepest level %d, widest level %d"
        % (shape["solver_count"], shape["deepest_solver_level"],
           shape["widest_solver_level"]))
    wide = [(int(k), v) for k, v in shape["solver_levels"].items()
            if len(v) > 1]
    if wide:
        add("  levels with more than one solver (these can share a level):")
        for level, names in sorted(wide):
            add("    %3d  %s" % (level, ", ".join(names)))
    alone = sorted(int(k) for k, v in shape["solver_levels"].items()
                   if len(v) == 1)
    if alone:
        add("  %d solvers sit alone on their own level%s"
            % (len(alone), ": %s" % alone[:12] if len(alone) <= 12
               else " (levels %d..%d)" % (alone[0], alone[-1])))

    add("")
    add("Geometry chain levels")
    add("-" * 62)
    for level in report["chains"]:
        add("  %-46s parallel=%s%s"
            % (", ".join(_name(t) for t in level["targets"])[:46],
               level["parallel"],
               "" if not level["why_not"] else "  (%s)" % level["why_not"]))
    if not report["chains"]:
        add("  none -- this rig deforms no geometry")

    add("")
    add("Cost per workload")
    add("-" * 62)
    add("  %-8s %9s %9s %9s %9s   %s"
        % ("", "best", "p50", "p90", "mean", "threads"))
    for which in ("replay", "drag", "author"):
        m = report["cost"][which]
        add("  %-8s %8.2f%s %8.2f%s %8.2f%s %8.2f%s   %d"
            % (which, m["ms"]["best"], "", m["ms"]["p50"], "",
               m["ms"]["p90"], "", m["ms"]["mean"], "",
               m["threads"]["distinct_threads"]))
    add("  (milliseconds. `author` includes recompiling the epoch, which is")
    add("   why it is usually an order of magnitude above `drag`.)")

    for which in ("drag", "author"):
        scopes = report["cost"][which]["scopes"]
        rows = [(n, v) for n, v in scopes.items() if n in HEADLINE]
        rows.sort(key=lambda kv: -kv[1]["ms_total"])
        if not rows:
            continue
        add("")
        add("  %s -- where the time went (ms per call)" % which)
        for name, v in rows[:top]:
            add("    %-24s %8.2f   x%d" % (name, v["ms_per_call"],
                                           v["count"]))

    add("")
    add("Threads that ran profiled scopes")
    add("-" * 62)
    add("  Scoped regions only. The per-point geometry kernels split their")
    add("  point range inside one scope, so their workers record nothing --")
    add("  one thread here does NOT mean the skinning ran serially. It does")
    add("  mean no SCOPED region ran in parallel.")
    for which in ("drag", "author"):
        t = report["cost"][which]["threads"]
        add("  %-8s %d distinct; ms per thread %s"
            % (which, t["distinct_threads"],
               dict(list(t["ms_per_thread"].items())[:6])))
        for cat, threads in t["threads_per_category"].items():
            if len(threads) > 1:
                add("      %-12s ran on %d threads %s"
                    % (cat, len(threads), threads[:8]))
    add("")
    return "\n".join(L)


# ---------------------------------------------------------------------------
# The one entry point both callers use
# ---------------------------------------------------------------------------

def build_report(stage, rig_root="/Biped/Rig", samples=40, control=None,
                 avar="avars:ry", rig=None):
    """Compile, measure, and return the whole report as plain data.

    `rig` lets a caller pass one it already holds -- the usdview panel
    does, because opening a second evaluator over the same stage would
    compile the rig twice and report the second compile's warm cache as
    if it were the first.
    """
    own = rig is None
    if own:
        rig = rigexec.Rig(stage, rig_root)

    rig.profiling_enabled = True
    rig.clear_profile()
    t = time.perf_counter()
    rig.compile()
    compile_ms = (time.perf_counter() - t) * 1000.0
    compile_scopes = {r["name"]: r["total_us"] / 1000.0
                      for r in rig.profile_summary()}
    compile_threads = _threads(rig.profile_events())
    rig.profiling_enabled = False

    first = rig.evaluate(0.0)
    controls = {_name(p): str(p) for p in first.control_paths()}
    if not controls:
        raise ValueError("no controls under %s" % rig_root)
    if control is None:
        control = "hips_ctl" if "hips_ctl" in controls else sorted(controls)[0]
    if control not in controls:
        raise ValueError("no control %s; have e.g. %s"
                         % (control, sorted(controls)[:8]))

    report = {
        "stage": {
            "path": stage.GetRootLayer().identifier,
            "rig_root": rig_root,
            "mode": rig.evaluation_mode,
            "mode_source": rig.evaluation_mode_source,
            "bakeable": rig.is_bakeable(),
            "bakeability_reasons": list(rig.bakeability_reasons()),
            "parallel_enabled": os.environ.get(
                "RIGEXEC_ENABLE_PARALLEL_EVAL", "true (default)"),
            "thread_limit": os.environ.get("PXR_WORK_THREAD_LIMIT"),
            "cores": os.cpu_count(),
            "platform": platform.platform(),
        },
        "compile": {"ms": compile_ms, "scopes": compile_scopes,
                    "threads": compile_threads},
        "shape": schedule_shape(stage, rig, rig_root),
        "chains": chain_shape(rig),
        "driven": {"control": controls[control], "avar": avar},
    }
    report["cost"] = measure(stage, rig, controls[control], avar, samples)
    return report, rig


def rig_roots(stage):
    """Every RigExecRoot on the stage, as paths -- what a chooser lists."""
    return [str(p.GetPath()) for p in stage.Traverse()
            if p.GetTypeName() == "RigExecRoot"]


def write_trace(rig, control_path, avar, path):
    """One dragged evaluation, as Chrome Trace Event JSON.

    One evaluation and not the whole measurement: a trace of forty poses
    is forty copies of the same spans, and what anyone opens a trace to
    see is the shape of a single frame.
    """
    rig.profiling_enabled = True
    rig.clear_profile()
    rig.set_interactive_overrides([(control_path, avar, 12.0)])
    rig.evaluate(0.0)
    rig.clear_interactive_overrides()
    rig.write_profile_trace(path)
    rig.profiling_enabled = False
    return path
