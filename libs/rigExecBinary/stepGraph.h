// The step and cluster graph a .rigexec carries, checked once per load.
// Playback walks the steps in index order (source steps first, in index
// order) and dirties whole clusters, so a file is refused unless that walk
// is a topological order of its step graph, every slot a step reads has
// been written by then, and its cluster graph is an acyclic quotient of
// the step graph. The file validator (RigExecFormatValidate, which every
// Open runs) calls RigExecStepGraphError, so the bake's self-check and the
// runtime refuse a bad graph in the same words. USD-free and header-only.
// On a valid file it allocates the cluster sort's two count-sized vectors,
// the walk order, and one map node per run of written slots.
#ifndef RIGEXEC_BINARY_STEP_GRAPH_H
#define RIGEXEC_BINARY_STEP_GRAPH_H

#include "rigExecBinary/format.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

namespace rigExec {

/// A slot range as the file's SlotRange holds it; the domain is an
/// fb::SlotDomain value.
struct RigExecStepGraphRange {
    uint8_t domain = 0;
    uint32_t begin = 0;
    uint32_t end = 0;
};

/// Whether a domain's slots hold a value before any step of a run writes
/// them, so a step may read them with no producer: the sources the
/// prologue fills (avars, property-chain results, chain bases and solver
/// points; RigExecBakedIsSourceDomain), and the snapshot store, whose read
/// names every earlier step whether or not it recorded.
inline bool
RigExecStepGraphDomainHoldsValue(uint8_t domain)
{
    switch (fb::SlotDomain(domain)) {
    case fb::SlotDomain::Avars:
    case fb::SlotDomain::PropertyResult:
    case fb::SlotDomain::ChainBase:
    case fb::SlotDomain::SolverPoints:
    case fb::SlotDomain::Snapshots:
        return true;
    default:
        return false;
    }
}

namespace rigExecStepGraphDetail {

inline std::string
Text(int64_t value)
{
    return std::to_string(value);
}

inline std::string
DomainName(uint8_t domain)
{
    switch (fb::SlotDomain(domain)) {
    case fb::SlotDomain::Avars: return "Avars";
    case fb::SlotDomain::PoseBase: return "PoseBase";
    case fb::SlotDomain::PoseFin: return "PoseFin";
    case fb::SlotDomain::PosedM: return "PosedM";
    case fb::SlotDomain::FinalMatrix: return "FinalMatrix";
    case fb::SlotDomain::BaseMatrix: return "BaseMatrix";
    case fb::SlotDomain::Aggregate: return "Aggregate";
    case fb::SlotDomain::SolverPoints: return "SolverPoints";
    case fb::SlotDomain::Candidates: return "Candidates";
    case fb::SlotDomain::CommitTable: return "CommitTable";
    case fb::SlotDomain::CommitDelta: return "CommitDelta";
    case fb::SlotDomain::CommitStaging: return "CommitStaging";
    case fb::SlotDomain::ConstraintDelta: return "ConstraintDelta";
    case fb::SlotDomain::PropertyResult: return "PropertyResult";
    case fb::SlotDomain::ChainBase: return "ChainBase";
    case fb::SlotDomain::RevisionPacket: return "RevisionPacket";
    case fb::SlotDomain::RevisionTransforms:
        return "RevisionTransforms";
    case fb::SlotDomain::RevisionOut: return "RevisionOut";
    case fb::SlotDomain::RevisionDone: return "RevisionDone";
    case fb::SlotDomain::ChainDirty: return "ChainDirty";
    case fb::SlotDomain::ChainPoints: return "ChainPoints";
    case fb::SlotDomain::DerivedOut: return "DerivedOut";
    case fb::SlotDomain::WeightPacket: return "WeightPacket";
    case fb::SlotDomain::WeightFrames: return "WeightFrames";
    case fb::SlotDomain::PoseWeight: return "PoseWeight";
    case fb::SlotDomain::Snapshots: return "Snapshots";
    }
    return "domain " + Text(domain);
}

/// The written slots of every domain as disjoint, non-touching runs,
/// keyed by domain and first slot.
class WrittenRuns {
public:
    /// Whether [range.begin, range.end) lies inside one run.
    bool Covers(const RigExecStepGraphRange &range) const
    {
        auto at = _runs.upper_bound(_Key(range.domain, range.begin));
        if (at == _runs.begin()) {
            return false;
        }
        --at;
        return _Domain(at->first) == range.domain && at->second >= range.end;
    }

    /// Adds [range.begin, range.end), merging the runs it overlaps or
    /// touches.
    void Add(const RigExecStepGraphRange &range)
    {
        if (range.begin >= range.end) {
            return;
        }
        uint64_t first = _Key(range.domain, range.begin);
        uint32_t end = range.end;
        auto at = _runs.upper_bound(first);
        if (at != _runs.begin()) {
            const auto before = std::prev(at);
            if (_Domain(before->first) == range.domain &&
                before->second >= range.begin) {
                first = before->first;
                end = std::max(end, before->second);
                at = _runs.erase(before);
            }
        }
        while (at != _runs.end() && _Domain(at->first) == range.domain &&
               uint32_t(at->first) <= end) {
            end = std::max(end, at->second);
            at = _runs.erase(at);
        }
        _runs.emplace_hint(at, first, end);
    }

private:
    static uint64_t _Key(uint8_t domain, uint32_t begin)
    {
        return (uint64_t(domain) << 32) | begin;
    }
    static uint8_t _Domain(uint64_t key) { return uint8_t(key >> 32); }

    /// First slot (under the domain) -> one past the last slot.
    std::map<uint64_t, uint32_t> _runs;
};

/// Whether the increasing \p list holds \p value.
template <class List>
bool
Contains(const List &list, int64_t value)
{
    return std::binary_search(
        list.begin(), list.end(), value,
        [](int64_t a, int64_t b) { return a < b; });
}

/// Which side of their owner an edge list's entries must lie on.
enum class Side { Earlier, Later, Either };

/// Every entry of \p list is an index below \p count other than \p self,
/// lies on \p side of it, and is greater than the entry before it. Empty
/// when so; otherwise the reason, naming \p owner.
template <class List>
std::string
EdgeListError(const List &list, size_t count, int64_t self, Side side,
              const std::string &owner, const char *edge,
              const char *thing)
{
    for (size_t k = 0; k < list.size(); ++k) {
        const int64_t at = int64_t(list[k]);
        if (at < 0 || uint64_t(at) >= uint64_t(count)) {
            return owner + " names " + edge + " " + Text(at) +
                   ", which is no " + thing;
        }
        if (at == self) {
            return owner + " depends on itself";
        }
        if (side == Side::Earlier && at > self) {
            return owner + " depends on later " + thing + " " + Text(at);
        }
        if (side == Side::Later && at < self) {
            return owner + " names earlier " + thing + " " + Text(at) +
                   " as a " + edge;
        }
        if (k > 0 && at <= int64_t(list[k - 1])) {
            return owner + " lists its " + edge + "s out of order or twice";
        }
    }
    return std::string();
}

/// Every entry of each node's \p forward list names a node whose
/// \p backward list names it back. The lists are increasing
/// (EdgeListError), so each lookup is a binary search.
template <class Nodes, class Forward, class Backward>
std::string
InverseError(const Nodes &nodes, Forward forward, Backward backward,
             const char *thing, const char *forwardEdge,
             const char *backwardEdge)
{
    for (size_t at = 0; at < nodes.size(); ++at) {
        for (const auto other : forward(nodes[at])) {
            if (!Contains(backward(nodes[size_t(other)]), int64_t(at))) {
                return std::string(thing) + " " + Text(int64_t(at)) +
                       " names " + forwardEdge + " " + Text(int64_t(other)) +
                       ", which does not name it as a " + backwardEdge;
            }
        }
    }
    return std::string();
}

}  // namespace rigExecStepGraphDetail

/// Empty when \p steps and \p clustering form a graph playback may walk;
/// otherwise the first violation found. Every index is range-checked
/// before it is followed. \p rangeOf turns a step's read or write range
/// into a RigExecStepGraphRange. Checked, in order:
///  - each step's preds are increasing, unique and earlier than it, its
///    succs increasing, unique and later, and the two are inverses;
///  - a step that runs in the source pass depends only on source steps;
///  - every slot a step reads is written by a step playback runs before
///    it, unless its domain holds a value before the run
///    (RigExecStepGraphDomainHoldsValue);
///  - clusterOf names a cluster for every step and agrees with each
///    step's own cluster;
///  - each cluster's members are increasing, belong to it, and together
///    the clusters list every step;
///  - cluster preds and succs are increasing, unique, free of self-edges
///    and inverses, and every step edge across clusters is a cluster edge;
///  - a Kahn sort over the cluster preds orders every cluster.
template <class Step, class Clustering, class RangeOf>
std::string
RigExecStepGraphError(const std::vector<Step> &steps,
                      const Clustering &clustering, RangeOf rangeOf)
{
    namespace detail = rigExecStepGraphDetail;
    using detail::Contains;
    using detail::Side;
    using detail::Text;
    const size_t stepCount = steps.size();
    const auto stepName = [](size_t s) { return "step " + Text(int64_t(s)); };

    // Each step's own lists first, so that an edge pointing the wrong way
    // is named as such rather than as a missing inverse on the step at
    // its other end.
    for (size_t s = 0; s < stepCount; ++s) {
        std::string why = detail::EdgeListError(
            steps[s].preds, stepCount, int64_t(s), Side::Earlier,
            stepName(s), "predecessor", "step");
        if (why.empty()) {
            why = detail::EdgeListError(steps[s].succs, stepCount,
                                        int64_t(s), Side::Later, stepName(s),
                                        "successor", "step");
        }
        if (!why.empty()) {
            return why;
        }
    }
    {
        const auto preds = [](const Step &step) -> const auto & {
            return step.preds;
        };
        const auto succs = [](const Step &step) -> const auto & {
            return step.succs;
        };
        std::string why = detail::InverseError(steps, preds, succs, "step",
                                               "predecessor", "successor");
        if (why.empty()) {
            why = detail::InverseError(steps, succs, preds, "step",
                                       "successor", "predecessor");
        }
        if (!why.empty()) {
            return why;
        }
    }
    // The source pass runs ahead of every other step, so a source step's
    // predecessors must run in it too.
    for (size_t s = 0; s < stepCount; ++s) {
        if (!steps[s].isSource) {
            continue;
        }
        for (const auto p : steps[s].preds) {
            if (!steps[size_t(p)].isSource) {
                return "source " + stepName(s) + " depends on step " +
                       Text(int64_t(p)) + ", which is not a source";
            }
        }
    }

    // Producers, in playback order: the source pass in index order, then
    // every other step in index order. A step's reads see the writes of
    // the steps before it, not its own.
    {
        std::vector<size_t> order;
        order.reserve(stepCount);
        for (const bool sourcePass : {true, false}) {
            for (size_t s = 0; s < stepCount; ++s) {
                if (bool(steps[s].isSource) == sourcePass) {
                    order.push_back(s);
                }
            }
        }
        detail::WrittenRuns written;
        for (const size_t s : order) {
            for (const auto &read : steps[s].reads) {
                const RigExecStepGraphRange range = rangeOf(read);
                if (range.begin < range.end &&
                    !RigExecStepGraphDomainHoldsValue(range.domain) &&
                    !written.Covers(range)) {
                    return stepName(s) + " reads " +
                           detail::DomainName(range.domain) + " slots [" +
                           Text(range.begin) + ", " + Text(range.end) +
                           "), which no earlier step writes";
                }
            }
            for (const auto &write : steps[s].writes) {
                written.Add(rangeOf(write));
            }
        }
    }

    // Cluster membership.
    const auto &clusters = clustering.clusters;
    const auto &clusterOf = clustering.clusterOf;
    const size_t clusterCount = clusters.size();
    const auto clusterName = [](size_t c) {
        return "cluster " + Text(int64_t(c));
    };
    if (clusterOf.size() != stepCount) {
        return "the clustering places " + Text(int64_t(clusterOf.size())) +
               " steps; the file has " + Text(int64_t(stepCount));
    }
    for (size_t s = 0; s < stepCount; ++s) {
        const int64_t c = int64_t(steps[s].cluster);
        if (c < 0 || uint64_t(c) >= uint64_t(clusterCount)) {
            return stepName(s) + " names cluster " + Text(c) +
                   ", which is no cluster";
        }
        if (int64_t(clusterOf[s]) != c) {
            return stepName(s) + " names cluster " + Text(c) +
                   ", but the clustering places it in cluster " +
                   Text(int64_t(clusterOf[s]));
        }
    }
    for (size_t c = 0; c < clusterCount; ++c) {
        const auto &members = clusters[c].members;
        for (size_t k = 0; k < members.size(); ++k) {
            const int64_t s = int64_t(members[k]);
            if (s < 0 || uint64_t(s) >= uint64_t(stepCount)) {
                return clusterName(c) + " names member " + Text(s) +
                       ", which is no step";
            }
            if (int64_t(clusterOf[size_t(s)]) != int64_t(c)) {
                return clusterName(c) + " names member " + Text(s) +
                       ", which the clustering places in cluster " +
                       Text(int64_t(clusterOf[size_t(s)]));
            }
            if (k > 0 && s <= int64_t(members[k - 1])) {
                return clusterName(c) +
                       " lists its members out of order or twice";
            }
        }
    }
    // Every member is placed in the cluster listing it and listed once
    // there, so the clusters partition the steps once each step is listed.
    for (size_t s = 0; s < stepCount; ++s) {
        if (!Contains(clusters[size_t(clusterOf[s])].members, int64_t(s))) {
            return stepName(s) + " is not a member of cluster " +
                   Text(int64_t(clusterOf[s]));
        }
    }

    // Cluster edges.
    for (size_t c = 0; c < clusterCount; ++c) {
        std::string why = detail::EdgeListError(
            clusters[c].preds, clusterCount, int64_t(c), Side::Either,
            clusterName(c), "predecessor", "cluster");
        if (why.empty()) {
            why = detail::EdgeListError(clusters[c].succs, clusterCount,
                                        int64_t(c), Side::Either,
                                        clusterName(c), "successor",
                                        "cluster");
        }
        if (!why.empty()) {
            return why;
        }
    }
    {
        using Cluster = typename std::decay<decltype(clusters[0])>::type;
        const auto preds = [](const Cluster &cluster) -> const auto & {
            return cluster.preds;
        };
        const auto succs = [](const Cluster &cluster) -> const auto & {
            return cluster.succs;
        };
        std::string why = detail::InverseError(
            clusters, preds, succs, "cluster", "predecessor", "successor");
        if (why.empty()) {
            why = detail::InverseError(clusters, succs, preds, "cluster",
                                       "successor", "predecessor");
        }
        if (!why.empty()) {
            return why;
        }
    }
    for (size_t s = 0; s < stepCount; ++s) {
        const int64_t to = int64_t(clusterOf[s]);
        const auto &clusterPreds = clusters[size_t(to)].preds;
        for (const auto p : steps[s].preds) {
            const int64_t from = int64_t(clusterOf[size_t(p)]);
            if (from != to && !Contains(clusterPreds, from)) {
                return stepName(s) + " in cluster " + Text(to) +
                       " depends on step " + Text(int64_t(p)) +
                       " in cluster " + Text(from) + ", which cluster " +
                       Text(to) + " does not depend on";
            }
        }
    }

    // Kahn over the cluster preds. A cluster left waiting has a waiting
    // predecessor, so walking from the lowest waiting cluster to its first
    // waiting predecessor, and on, must revisit a cluster, and the cluster
    // it revisits is on a cycle.
    std::vector<int64_t> waiting(clusterCount, 0);
    std::vector<size_t> ready;
    ready.reserve(clusterCount);
    for (size_t c = 0; c < clusterCount; ++c) {
        waiting[c] = int64_t(clusters[c].preds.size());
        if (waiting[c] == 0) {
            ready.push_back(c);
        }
    }
    for (size_t head = 0; head < ready.size(); ++head) {
        for (const auto succ : clusters[ready[head]].succs) {
            if (--waiting[size_t(succ)] == 0) {
                ready.push_back(size_t(succ));
            }
        }
    }
    if (ready.size() != clusterCount) {
        size_t at = 0;
        while (waiting[at] == 0) {
            ++at;
        }
        constexpr int64_t visited = -1;
        while (waiting[at] != visited) {
            waiting[at] = visited;
            for (const auto pred : clusters[at].preds) {
                if (waiting[size_t(pred)] != 0) {
                    at = size_t(pred);
                    break;
                }
            }
        }
        return "the cluster graph has a cycle through " + clusterName(at);
    }
    return std::string();
}

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_STEP_GRAPH_H
