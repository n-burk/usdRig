# Copyright (c) 2026 Nick Burkard
# SPDX-License-Identifier: MIT
"""What a rig's evaluation schedule looks like, and where its time goes.

Headless: no Qt, so the CLI (`tools/rigexec_schedule.py`) and the usdview
panel (`profilerUI.py`) share one measurement and cannot drift into two
different answers.

The report describes the compiled operation graph and measures compile,
unchanged replay, interactive drag, and authored-edit workloads. Operations
become ready after their predecessors finish; dependency depth is a graph
property, not a dispatch barrier. Thread counts cover recorded scopes only.
"""
import json
import os
import platform
import sys
import time

from pxr import Sdf

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


# Structure

def schedule_shape(stage, rig, rig_root):
    """Actual operation count, longest dependency path and cluster count."""
    nodes = rig.op_graph()
    by_id = {node["step"]: node for node in nodes}
    if len(by_id) != len(nodes):
        raise ValueError("duplicate operation ids")
    remaining = {i: set(n["preds"]) for i, n in by_id.items()}
    successors = {i: [] for i in by_id}
    for i, preds in remaining.items():
        for predecessor in preds:
            if predecessor not in by_id:
                raise ValueError("missing predecessor %s" % predecessor)
            successors[predecessor].append(i)
    ready = sorted(i for i, preds in remaining.items() if not preds)
    depths = {i: 1 for i in ready}
    completed = 0
    while ready:
        i = ready.pop()
        completed += 1
        for successor in successors[i]:
            depths[successor] = max(depths.get(successor, 1), depths[i] + 1)
            remaining[successor].remove(i)
            if not remaining[successor]:
                ready.append(successor)
    if completed != len(nodes):
        raise ValueError("compiled operation graph contains a cycle")
    domains = {}
    for node in nodes:
        domains[node["domain"]] = domains.get(node["domain"], 0) + 1
    return {"op_count": len(nodes),
            "longest_path": max(depths.values(), default=0),
            "cluster_count": len({n["cluster"] for n in nodes if n["cluster"] >= 0}),
            "domains": dict(sorted(domains.items()))}


# Cost

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
    ran concurrently: ready operation dispatch, the compile-time digest
    and the exec warm-up. One thread across the whole of `drag` means no
    scoped region ran in parallel -- which is the true and interesting
    statement about the recorded scopes.
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


# Report

def render(report, top):
    L = []
    add = L.append
    s = report["stage"]
    add("")
    add("RigExec schedule report")
    add("=" * 62)
    add("  stage            : %s" % s["path"])
    add("  rig root         : %s" % s["rig_root"])
    add("  parallel eval    : %s   logical cores: %s"
        % (s["parallel_enabled"], s["cores"]))
    if s["thread_limit"]:
        add("  PXR_WORK_THREAD_LIMIT = %s" % s["thread_limit"])

    shape = report["shape"]
    add("")
    add("Schedule shape -- depth is the part threads cannot fix")
    add("-" * 62)
    add("  operations       : %d" % shape["op_count"])
    add("  longest path     : %d operations" % shape["longest_path"])
    add("  clusters         : %d" % shape["cluster_count"])
    add("  domains          : %s" % shape["domains"])
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


# The one entry point both callers use

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
        control = "M_Body" if "M_Body" in controls else sorted(controls)[0]
    if control not in controls:
        raise ValueError("no control %s; have e.g. %s"
                         % (control, sorted(controls)[:8]))

    report = {
        "stage": {
            "path": stage.GetRootLayer().identifier,
            "rig_root": rig_root,
            "parallel_enabled": os.environ.get(
                "RIGEXEC_ENABLE_PARALLEL_EVAL", "true (default)"),
            "thread_limit": os.environ.get("PXR_WORK_THREAD_LIMIT"),
            "cores": os.cpu_count(),
            "platform": platform.platform(),
        },
        "compile": {"ms": compile_ms, "scopes": compile_scopes,
                    "threads": compile_threads},
        "shape": schedule_shape(stage, rig, rig_root),
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
