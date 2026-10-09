#ifndef RIGEXEC_GRAPH_OP_GRAPH_H
#define RIGEXEC_GRAPH_OP_GRAPH_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rigExec {

using RigExecValueId = uint64_t;

/// Backend-independent operations. Values already identify a domain/slot/version.
/// Stable keys include owner, kind and revision; no source iteration orders them.
struct RigExecOpDescriptor {
    std::string key;
    uint32_t kind = 0;
    std::vector<RigExecValueId> reads, writes;
    std::vector<uint32_t> predecessors;
    bool volatileInput = false;
};

struct RigExecCompiledOp {
    RigExecOpDescriptor descriptor;
    uint32_t originalIndex = 0;
    std::vector<uint32_t> predecessors, successors;
};

struct RigExecOpCluster {
    std::vector<uint32_t> members, predecessors, successors;
};

struct RigExecCompiledGraph {
    std::vector<RigExecCompiledOp> ops;
    std::vector<int32_t> canonicalIndex;
    std::unordered_map<RigExecValueId, std::vector<uint32_t>> readers;
    /// One actual edge-following loop per cyclic strongly connected component.
    std::vector<std::vector<std::string>> cycles;
    /// Complete SCC members, sorted by stable key and aligned with cycles.
    /// Downstream consumers are not members, even when they depend on a loop.
    /// Compile reporting only; execution and wire ownership are unchanged.
    std::vector<std::vector<std::string>> cycleMembers;
    size_t longestPath = 0;
    std::vector<RigExecOpCluster> clusters;
    std::vector<uint32_t> opClusters;
    /// Canonical ops whose descriptor has volatileInput, ascending. Filled
    /// by the compiler; a graph rebuilt from another source copies it.
    std::vector<uint32_t> volatileOps;
};

enum class RigExecCyclePolicy { Reject, SetAside };

/// Retained compile-time SCC authority when writer bindings are reconstructed.
/// Supplied authority must come from the original canonical compiler, not
/// fabricated reports. Keys retain bodies; removedKeys prove normalized writers.
struct RigExecOpExclusionProof {
    std::vector<std::string> keys, removedKeys;
    std::vector<std::vector<std::string>> cycles, cycleMembers;
};

/// One producer-based compiler and canonical Kahn order for every domain.
/// SetAside omits only cycle members; consumers retain explicit fallback reads.
bool RigExecCompileOpGraph(const std::vector<RigExecOpDescriptor> &descriptors,
    const std::vector<RigExecValueId> &leaves, RigExecCyclePolicy cyclePolicy,
    RigExecCompiledGraph *graph, std::string *error = nullptr);
bool RigExecCompileOpGraph(const std::vector<RigExecOpDescriptor> &descriptors,
    const std::vector<RigExecValueId> &leaves, RigExecCyclePolicy cyclePolicy,
    const RigExecOpExclusionProof *retainedExclusions,
    RigExecCompiledGraph *graph, std::string *error);

/// Coarsen only linear chains whose intermediate values have no outside
/// readers and whose later members have no extra predecessors. Readiness at
/// a cluster boundary therefore introduces no cross-branch dependency.
/// Costs are in canonical op order; empty uses one microsecond per op.
/// Zero grain gives singleton clusters. Lower once, serialize unchanged.
bool RigExecLowerOpClusters(RigExecCompiledGraph *graph,
    const std::vector<double> &opCostsUs, double grainUs,
    std::string *error = nullptr);

/// Validate serialized membership and safe coarsening against canonical ops.
bool RigExecValidateOpClusters(const RigExecCompiledGraph &graph,
    std::string *error = nullptr);

struct RigExecOpCallbacks {
    /// false cancels bodies not yet started; tasks already active may finish.
    /// Nonfatal invalid values resolve normally with run returning true.
    std::function<bool(uint32_t)> run;
    std::function<void(uint32_t)> skip;
    /// Optional ascending list of the ops whose skip changes state. When set,
    /// a non-candidate op outside it is not passed to skip; a candidate that
    /// does not run always is. Null passes every skipped op.
    const std::vector<uint32_t> *skipEffects = nullptr;
    /// Current-generation change state, including validity/count/error changes.
    std::function<bool(RigExecValueId)> changed;
    /// Optional reader-specific policy, for an input shadowed at this consumer.
    std::function<bool(uint32_t, RigExecValueId)> inputChanged;
    /// Optional complete input-change predicate, evaluated after predecessors.
    /// Selects the effective input when a produced value shadows raw fallbacks.
    /// Mandatory seeds still run; the predicate also updates retained input state.
    std::function<bool(uint32_t)> inputsChanged;
    /// Optional asynchronous dispatch. wait joins every recursively submitted op.
    std::function<void(std::function<void()>)> dispatch;
    std::function<void()> wait;
};

struct RigExecOpExecution {
    std::vector<char> candidates, ran;
    /// Finish order from 1; zero for non-candidates. A cluster numbers its
    /// candidate members consecutively, in member order, after its last body
    /// and before any successor is released, so every predecessor finishes
    /// first. A serial run numbers ops in the order their bodies ran.
    std::vector<uint64_t> completion;
    /// Derived from ran after the join: skipped is every op that did not run.
    size_t executed = 0, skipped = 0;
};

struct RigExecOpPendingCount {
    std::atomic<uint32_t> value{0};
    RigExecOpPendingCount() = default;
    RigExecOpPendingCount(const RigExecOpPendingCount &other)
        : value(other.value.load(std::memory_order_relaxed)) {}
    RigExecOpPendingCount &operator=(const RigExecOpPendingCount &other) {
        value.store(other.value.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }
};

/// Retained by each backend store; clone only while no execution is active.
struct RigExecOpWorkspace {
    std::vector<char> seeded, clusterCandidates;
    std::vector<uint32_t> pending, ready;
    std::vector<RigExecOpPendingCount> unresolved;
};

/// The same readiness/change-propagation loop for serial and parallel backends.
/// Callers reset current-generation change flags before publishing sampled leaves.
/// A clean retained output stops the wave; no domain or source barriers exist.
/// candidateOps schedules potential readers without forcing their bodies to run.
bool RigExecExecuteOpGraph(const RigExecCompiledGraph &graph,
    const std::vector<RigExecValueId> &changedLeaves,
    const std::vector<uint32_t> &seedOps, bool force,
    const RigExecOpCallbacks &callbacks, RigExecOpExecution *execution,
    std::string *error = nullptr, RigExecOpWorkspace *workspace = nullptr,
    const std::vector<uint32_t> *candidateOps = nullptr);

} // namespace rigExec
#endif
