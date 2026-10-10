// RigExec live operation graph: the evaluator's in-memory operation
// structures as one diagnostic graph.
// The compiled epoch is scattered across a dozen members -- solver
// batches, pose steps, frame constraints, geometry revision chains with
// their live VdfNetworks, property chains, tap sets -- and each answers
// only its own question. This is the union: every operation as a node,
// every dependency the compile derived as an edge, so a visualizer (or
// a test holding the compile to account) reads the whole epoch at once.
// Nodes carry the profiler scope that fires them, so a generation's
// profile events attribute straight onto the graph.
// Built only on request (RigExecRigEvaluator::DescribeLiveOperations);
// compile and evaluation never build or read it.
// Ids are stable within the epoch and read as paths where they can be:
//   batch:<index>              one solver batch (the exec request unit)
//   solver:<path>              one aggregate solver
//   constraint:<mover path>    one compiled frame constraint
//   chain:<target>             one geometry revision chain
//   rev:<target>@<i>           its i-th revision, in execution order
//   derived:<target>           the derived chain (extent and the like)
//                              that runs after the target's points chain
//   drev:<target>@<i>          its i-th revision, in execution order
//   prop:<target>              one property chain
//   proprev:<target>@<i>       its i-th revision, in execution order
//   tap:<set>:<id>             one exec tap (sets: main, rest, first,
//                              guide, batch<k>, conn:<path>)
//   provider:<path>            one transform provider value
//   switch:<path>              one space switch
//   interp:<path>              one pose interpolator
//   skipped:<path>             an operation the compile set aside
// Edge kinds, always producer/earlier -> consumer/later except reads:
//   member   part -> whole (solver -> batch, revision -> chain, ...)
//   order    predecessor -> successor in walk order
//   dep      user -> used (solver -> solver it reads)
//   read     consumer -> tap or provider it reads
//   write    operation -> provider or value it revises
//   provides tap -> the provider value it computes
//   phase    consumer -> chain it reads at a phase; the chain node's
//            phased_readers list names the revision each one reads
//   rides    follower batch -> the leader batch whose request it shares
#ifndef RIGEXEC_LIVE_OPERATION_GRAPH_H
#define RIGEXEC_LIVE_OPERATION_GRAPH_H

#include <map>
#include <string>
#include <vector>

namespace rigExec {

/// One operation, value, or exec tap of the compiled epoch.
struct RigExecLiveOpNode {
    std::string id;
    /// batch, solver, constraint, chain, revision, propchain, proprev,
    /// tap, provider, switch, interp or skipped.
    std::string domain;
    /// The schema type, revision op, computation, or role within the domain.
    std::string kind;
    /// One human line, usually the path.
    std::string label;
    /// The profiler scope a generation records when this fires, "" when
    /// none does. Dynamic fires are profiled visits: the walk reaches
    /// batches, constraints, and revisions every generation, so those
    /// light in walk order with varying durations, while cached ops
    /// (property chains) light only when they re-evaluate.
    std::string profile;
    /// The batch, chain, or band level where the domain has one, else -1.
    int level = -1;
    /// The walk position where the domain has one, else -1.
    int order = -1;
    /// Scalar inspector facts.
    std::map<std::string, std::string> details;
    /// Listed inspector facts.
    std::map<std::string, std::vector<std::string>> lists;
};

/// One dependency between two nodes of the same graph.
struct RigExecLiveOpEdge {
    std::string src;
    std::string dst;
    std::string kind;
};

/// The whole compiled epoch, as nodes plus edges. Node ids are unique;
/// edges name only nodes of the same graph.
struct RigExecLiveOperationGraph {
    std::vector<RigExecLiveOpNode> nodes;
    std::vector<RigExecLiveOpEdge> edges;
};

}  // namespace rigExec

#endif  // RIGEXEC_LIVE_OPERATION_GRAPH_H
