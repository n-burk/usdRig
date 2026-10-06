#!/usr/bin/env python
"""
The baked op trace and op graph from Python, on the arm example.

Rig.last_op_trace() lists the steps the last generation executed with their
1-based completion order; Rig.op_graph() lists every step with its edges.
The trace must be a valid completion order over the graph, and both must be
empty for a generation the program did not answer.

Usage: test_rigexec_op_trace.py [<generated schema resources dir>]
"""
import os
import sys

from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

from pxr import Plug, Usd  # noqa: E402

import rigexec  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_ARM = os.path.join(_REPO, "examples", "ArmRig.usda")
_RIG = "/ArmAsset/Rig"


def _RequireEngine():
    missing = [n for n in ("last_op_trace", "op_graph")
               if not hasattr(rigexec.Rig, n)]
    if missing:
        raise AssertionError(
            "this build's _rigexec is older than this test: Rig is missing "
            "%s. Rebuild and re-run; the module in use is %s."
            % (", ".join(missing), getattr(rigexec, "__file__", "?")))


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Open(mode):
    stage = Usd.Stage.Open(_ARM)
    rig = rigexec.Rig(stage, _RIG)
    rig.compile()
    rig.evaluation_mode = mode
    return stage, rig


def _CheckTrace(trace, graph):
    """Every executed pred finished before its executed succ; seqs 1..N."""
    seqs = sorted(entry["seq"] for entry in trace)
    assert seqs == list(range(1, len(trace) + 1)), seqs
    steps = [entry["step"] for entry in trace]
    assert len(set(steps)) == len(steps), "a step is traced twice"
    seq_of = {entry["step"]: entry["seq"] for entry in trace}
    for node in graph:
        if node["step"] not in seq_of:
            continue
        for pred in node["preds"]:
            if pred in seq_of:
                assert seq_of[pred] < seq_of[node["step"]], (
                    "%s finished after its successor %s"
                    % (graph[pred]["label"], node["label"]))


def TestTheTraceRespectsTheGraph():
    _, rig = _Open("baked")
    rig.evaluate(1001.0)
    graph = rig.op_graph()
    trace = rig.last_op_trace()
    assert graph, "the arm is expected to bake"
    assert trace, "the first baked generation executed nothing"
    head_kinds = {"PropertyRevision", "RestCompose", "LadderCompose", "SkinTopology"}
    seen_region = False
    assert any(node["domain"] == "head" for node in graph)
    for index, node in enumerate(graph):
        assert node["step"] == index
        assert node["domain"] in ("head", "pose", "weight", "geometry"), node
        is_head = node["domain"] == "head"
        assert is_head == (node["kind"] in head_kinds), node
        assert not (is_head and seen_region), "heads must form a prefix"
        seen_region = seen_region or not is_head
        if is_head:
            assert all(graph[p]["domain"] == "head" for p in node["preds"])
        assert set(node) >= {"kind", "label", "preds", "succs", "cluster",
                             "level", "reads", "writes"}
        for pred in node["preds"]:
            assert pred < index and index in graph[pred]["succs"]
        for domain, first, last in node["reads"] + node["writes"]:
            assert isinstance(domain, str) and first <= last
    for entry in trace:
        assert set(entry) == {"step", "kind", "domain", "label", "seq",
                              "cluster"}, entry
        assert entry["kind"] == graph[entry["step"]]["kind"]
        assert entry["domain"] == graph[entry["step"]]["domain"]
    _CheckTrace(trace, graph)
    rig.evaluate(1024.0)
    _CheckTrace(rig.last_op_trace(), graph)


def TestAWalkGenerationHasNoTrace():
    _, rig = _Open("reference")
    rig.evaluate(1001.0)
    assert rig.last_op_trace() == []
    assert rig.op_graph() == []


def TestARebuiltProgramHasNoTraceUntilItRuns():
    # Switching a clean epoch to the parity mode rebuilds the program; the
    # rebuilt one has answered no generation yet.
    _, rig = _Open("baked")
    rig.evaluate(1001.0)
    assert rig.last_op_trace() and rig.op_graph()
    rig.evaluation_mode = "parity"
    assert rig.last_op_trace() == []
    assert rig.op_graph() == []
    rig.evaluate(1024.0)
    assert rig.last_op_trace() and rig.op_graph()


def main():
    _RegisterSchema()
    _RequireEngine()
    groups = [
        ("the trace respects the graph", TestTheTraceRespectsTheGraph),
        ("a walk generation has no trace", TestAWalkGenerationHasNoTrace),
        ("a rebuilt program has no trace until it runs",
         TestARebuiltProgramHasNoTraceUntilItRuns),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_OP_TRACE_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
