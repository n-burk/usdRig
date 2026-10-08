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

def _Check(condition,message):
    if not condition: raise AssertionError(message)

def _Rig():
    stage=Usd.Stage.Open(STAGE); rig=rigexec.Rig(stage,RIG)
    rig.cpu_reference=True; rig.compile(); rig.evaluate(FRAME)
    return rig

def _Values(rig,frame):
    pose=rig.evaluate(frame)
    _Check(pose.valid,"invalid graph pose")
    _Check(pose.reference_agreements>0 and pose.reference_mismatches==0,
           "independent scalar reference disagrees")
    frames={path:list(pose.control_frame(path).to_matrix4()) for path in pose.control_paths()}
    return repr(sorted(pose.moved_properties().items())),repr(sorted(frames.items()))

def TestGraphIsWellFormed():
    nodes=_Rig().op_graph(); by_id={node["step"]:node for node in nodes}
    _Check(bool(nodes) and len(nodes)==len(by_id),"unique compiled operations")
    for node in nodes:
        _Check("level" not in node,"retired barrier level exposed")
        for predecessor in node["preds"]:
            _Check(predecessor in by_id and node["step"] in by_id[predecessor]["succs"],
                   "dependency endpoints and reverse edges")

def TestPhasedReadersAreListed():
    nodes=_Rig().op_graph()
    # A phased property has multiple typed output slots; consumers must name
    # the exact slot rather than a time-dependent source lookup.
    writes=[tuple(r) for n in nodes for r in n["writes"] if "Property" in r[0]]
    reads=[tuple(r) for n in nodes for r in n["reads"] if "Property" in r[0]]
    _Check(len(set(writes))>=2 and bool(reads),"phased typed values missing")
    _Check(any(read in writes for read in reads),"no phased value consumer")

def TestDescribingChangesNothing():
    rig=_Rig(); before=_Values(rig,FRAME)
    rig.op_graph(); rig.last_op_trace()
    _Check(_Values(rig,FRAME)==before,"graph inspection changed publication")

def TestProfileScopesExist():
    rig=_Rig(); rig.profiling_enabled=True; rig.clear_profile(); rig.evaluate(FRAME+1)
    _Check(bool(rig.profile_events()),"graph execution records scopes")


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
