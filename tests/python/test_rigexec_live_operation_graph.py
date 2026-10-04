#!/usr/bin/env python
"""
The live operation graph describes the compiled epoch without touching it.

Example 16 reads one revised dial at every phase: Base and BaseCard at the
default (base), Gain and GainCard at the checkpoint after the Gain revision,
Final and FinalCard at the declared final. The contract:

  * node ids are unique and every edge names two nodes of the graph;
  * domains and edge kinds are the documented ones;
  * the dial's property chain lists each phased reader with the revision it
    reads, and the readers' own operations point at the chain;
  * asking for the graph, or for the baked step graph, changes no value;
  * each node's profile scope is a scope the profiler actually records.

Usage: test_rigexec_live_operation_graph.py [<schema resources dir>]
"""
import os
import sys

from test_rigexec_python import _setup_environment
_setup_environment()

from pxr import Plug, Usd  # noqa: E402

import rigexec  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STAGE = os.path.join(HERE, "..", "..", "examples",
                     "16_ConnectionReadPhases.usda")
RIG = "/PhaseConnectAsset/Rig"
DIAL = "/PhaseConnectAsset/Rig/Channels/Dial.rigExec:amount"
FRAME = 1024.0

DOMAINS = {"batch", "solver", "constraint", "chain", "revision", "propchain",
           "proprev", "tap", "provider", "switch", "interp", "skipped"}
EDGE_KINDS = {"member", "order", "dep", "read", "write", "provides", "phase",
              "rides"}


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Rig(mode="dynamic"):
    stage = Usd.Stage.Open(STAGE)
    rig = rigexec.Rig(stage, RIG)
    rig.compile()
    rig.evaluation_mode = mode
    return rig


def _Values(rig, frame):
    pose = rig.evaluate(frame)
    _Check(pose.valid, "pose at %g is invalid: %s" % (frame, pose.diagnostics))
    frames = dict((path, list(pose.control_frame(path).to_matrix4()))
                  for path in pose.control_paths())
    return repr(sorted(pose.moved_properties().items())), repr(sorted(
        frames.items()))


def TestGraphIsWellFormed():
    graph = _Rig().operation_graph()
    ids = [node["id"] for node in graph["nodes"]]
    _Check(ids, "a compiled rig has operations")
    _Check(len(ids) == len(set(ids)), "node ids are unique")
    known = set(ids)
    for edge in graph["edges"]:
        _Check(edge["src"] in known and edge["dst"] in known,
               "edge names a missing node: %s" % edge)
        _Check(edge["kind"] in EDGE_KINDS, "undocumented edge %s" % edge)
    for node in graph["nodes"]:
        _Check(node["domain"] in DOMAINS,
               "undocumented domain %s" % node["domain"])


def TestPhasedReadersAreListed():
    graph = _Rig().operation_graph()
    nodes = dict((node["id"], node) for node in graph["nodes"])
    chain = nodes.get("prop:" + DIAL)
    _Check(chain is not None, "the dial has a property chain node")
    readers = chain["lists"].get("phased_readers", [])

    def reads(prim, phase):
        return any(line.split(".")[0].endswith("/" + prim) and
                   line.endswith(": " + phase) for line in readers)

    for prim in ("Base", "BaseCard"):
        _Check(reads(prim, "base"), "%s reads base: %s" % (prim, readers))
    for prim in ("Gain", "GainCard"):
        _Check(reads(prim, "after 1 of 2"),
               "%s reads after the Gain revision: %s" % (prim, readers))
    _Check(not any("/Final" in line and line.endswith(": base")
                   for line in readers),
           "a declared final never reads base: %s" % readers)
    phased = [edge for edge in graph["edges"]
              if edge["kind"] == "phase" and edge["dst"] == "prop:" + DIAL]
    sources = set(nodes[edge["src"]]["domain"] for edge in phased)
    _Check("proprev" in sources,
           "a property mover reading the dial points at its chain")
    _Check("revision" in sources,
           "a geometry revision reading the dial points at its chain")


def TestDescribingChangesNothing():
    for mode in ("dynamic", "baked"):
        rig = _Rig(mode)
        before = _Values(rig, FRAME)
        rig.operation_graph()
        rig.baked_steps()
        rig.baked_clusters()
        _Check(_Values(rig, FRAME) == before,
               "%s: describing the epoch changed a value" % mode)
        if mode == "baked":
            _Check(rig.baked_steps() and rig.baked_clusters(),
                   "a baked rig describes its program")
            _Check(rig.baked_generation_count > 0,
                   "the baked program answered")


def TestProfileScopesExist():
    rig = _Rig()
    rig.profiling_enabled = True
    rig.clear_profile()
    rig.evaluate(FRAME)
    recorded = set(event["name"] for event in rig.profile_events())
    graph = rig.operation_graph()
    for node in graph["nodes"]:
        if node["domain"] in ("propchain", "revision") and node["profile"]:
            _Check(node["profile"] in recorded,
                   "%s names scope %r, which never fired: %s"
                   % (node["id"], node["profile"], sorted(recorded)))


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    if plugin_dir:
        Plug.Registry().RegisterPlugins(plugin_dir)
    TestGraphIsWellFormed()
    TestPhasedReadersAreListed()
    TestDescribingChangesNothing()
    TestProfileScopesExist()
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
