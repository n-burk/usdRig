// The step and cluster graph a .rigexec carries, checked once per load.
// Playback walks the steps in index order (source steps first, in index
// order) and dirties whole clusters, so a file is refused unless that walk
// is a topological order of its step graph and its cluster graph is an
// acyclic quotient of the step graph. Whether each slot a step reads has
// an earlier writer is not checked here. The current wire reader and the
// FlatBuffer validator both call RigExecStepGraphError, whose step and
// cluster types share their field names, so the two refuse a bad graph in
// the same words. USD-free and header-only. On a valid file it allocates
// only the cluster sort's two count-sized vectors.
#ifndef RIGEXEC_BINARY_STEP_GRAPH_H
#define RIGEXEC_BINARY_STEP_GRAPH_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

namespace rigExec {

namespace rigExecStepGraphDetail {

inline std::string
Text(int64_t value)
{
    return std::to_string(value);
}

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
/// before it is followed. Checked, in order:
///  - each step's preds are increasing, unique and earlier than it, its
///    succs increasing, unique and later, and the two are inverses;
///  - a step that runs in the source pass depends only on source steps;
///  - clusterOf names a cluster for every step and agrees with each
///    step's own cluster;
///  - each cluster's members are increasing, belong to it, and together
///    the clusters list every step;
///  - cluster preds and succs are increasing, unique, free of self-edges
///    and inverses, and every step edge across clusters is a cluster edge;
///  - a Kahn sort over the cluster preds orders every cluster.
template <class Step, class Clustering>
std::string
RigExecStepGraphError(const std::vector<Step> &steps,
                      const Clustering &clustering)
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
