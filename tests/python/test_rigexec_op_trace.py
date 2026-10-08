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
    rig.cpu_reference = True
    return stage, rig


def _CheckTrace(trace, graph):
    """Every executed pred finished before its executed succ; seqs 1..N."""
    seqs = sorted(entry["seq"] for entry in trace)
    assert len(set(seqs)) == len(seqs) and all(seq > 0 for seq in seqs), seqs
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
    _, rig = _Open("graph")
    rig.evaluate(1001.0)
    graph = rig.op_graph()
    trace = rig.last_op_trace()
    assert graph, "the arm is expected to bake"
    assert trace, "the first baked generation executed nothing"
    by_id={node["step"]:node for node in graph}
    assert len(by_id)==len(graph)
    for node in graph:
        assert set(node) >= {"kind","domain","label","preds","succs","cluster","reads","writes"}
        assert "level" not in node
        for pred in node["preds"]:
            assert pred in by_id and node["step"] in by_id[pred]["succs"]
        for domain,first,last in node["reads"]+node["writes"]:
            assert isinstance(domain,str) and first<=last
    for entry in trace:
        assert set(entry) == {"step", "kind", "domain", "label", "seq",
                              "cluster", "start_us", "duration_us", "thread"}, entry
        assert entry["kind"] == by_id[entry["step"]]["kind"]
        assert entry["domain"] == by_id[entry["step"]]["domain"]
    _CheckTrace(trace, graph)
    assert all(e['start_us'] == 0 and not e['thread'] for e in trace)
    rig.evaluate(1024.0)
    _CheckTrace(rig.last_op_trace(), graph)
    _, timed_rig = _Open("graph")
    timed_rig.op_timing_enabled = True
    timed_rig.evaluate(1001.0)
    timed = timed_rig.last_op_trace()
    _CheckTrace(timed, timed_rig.op_graph())
    assert timed and all(e['start_us'] > 0 and e['duration_us'] >= 0 and e['thread'] for e in timed)


def TestReferenceKeepsTheProductionGraph():
    _,rig=_Open("graph")
    rig.cpu_reference=False
    rig.evaluate(1001.0)
    graph=rig.op_graph()
    assert rig.last_op_trace() and graph
    rig.cpu_reference=True
    pose=rig.evaluate(1024.0)
    assert pose.reference_agreements>0 and pose.reference_mismatches==0
    assert rig.op_graph()==graph
    _CheckTrace(rig.last_op_trace(),graph)


def main():
    _RegisterSchema()
    _RequireEngine()
    groups = [
        ("the trace respects the graph", TestTheTraceRespectsTheGraph),
        ("reference keeps the production graph", TestReferenceKeepsTheProductionGraph),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_OP_TRACE_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
