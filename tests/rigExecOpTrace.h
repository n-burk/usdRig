// Assertions over the baked program's op trace (rigExec/bakedTrace.h).
// A trace lists the steps one run executed with their 1-based completion
// sequence numbers; a graph lists every step with its predecessors and
// successors. These helpers answer "did X finish before Y" and "does the
// order the run produced respect every edge of the graph", returning
// violations as text so a suite keeps its own failure counter.
#ifndef RIGEXEC_TESTS_OP_TRACE_H
#define RIGEXEC_TESTS_OP_TRACE_H

#include "rigExec/bakedTrace.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace rigExecTest {

/// The entries whose kind matches exactly and whose domain/label contain the given substrings;
/// an empty substring matches everything.
inline std::vector<rigExec::RigExecOpTraceEntry>
FindTraceEntries(const std::vector<rigExec::RigExecOpTraceEntry> &trace,
                 const std::string &kind, const std::string &domain = "",
                 const std::string &label = "")
{
    std::vector<rigExec::RigExecOpTraceEntry> out;
    for (const rigExec::RigExecOpTraceEntry &entry : trace) {
        if ((kind.empty() || entry.kind == kind) &&
            entry.domain.find(domain) != std::string::npos &&
            entry.label.find(label) != std::string::npos) {
            out.push_back(entry);
        }
    }
    return out;
}

/// How many entries belong to \p domain ("pose", "weight", "geometry").
inline size_t
CountTraceDomain(const std::vector<rigExec::RigExecOpTraceEntry> &trace,
                 const std::string &domain)
{
    size_t count = 0;
    for (const rigExec::RigExecOpTraceEntry &entry : trace) {
        count += entry.domain == domain ? 1 : 0;
    }
    return count;
}

/// Whether program step \p a finished before program step \p b. False when
/// either did not run.
inline bool
Precedes(const std::vector<rigExec::RigExecOpTraceEntry> &trace, size_t a,
         size_t b)
{
    uint32_t seqA = 0, seqB = 0;
    for (const rigExec::RigExecOpTraceEntry &entry : trace) {
        if (entry.step == a) seqA = entry.seq;
        if (entry.step == b) seqB = entry.seq;
    }
    return seqA != 0 && seqB != 0 && seqA < seqB;
}

/// The 1-based completion sequence of program step \p step in \p trace, 0
/// when it did not run.
inline uint32_t
TraceSeqOf(const std::vector<rigExec::RigExecOpTraceEntry> &trace,
           size_t step)
{
    for (const rigExec::RigExecOpTraceEntry &entry : trace) {
        if (entry.step == step) {
            return entry.seq;
        }
    }
    return 0;
}

/// The program indices of graph nodes whose kind matches exactly and label contains the
/// given substrings, in program order; an empty substring matches everything.
inline std::vector<size_t>
FindOpGraphSteps(const std::vector<rigExec::RigExecOpGraphNode> &graph,
                 const std::string &kind, const std::string &label = "")
{
    std::vector<size_t> out;
    for (const rigExec::RigExecOpGraphNode &node : graph) {
        if ((kind.empty() || node.kind == kind) &&
            node.label.find(label) != std::string::npos) {
            out.push_back(node.step);
        }
    }
    return out;
}

/// Every step reachable from \p seeds along succ edges, the seeds included.
inline std::vector<char>
OpGraphForwardCone(const std::vector<rigExec::RigExecOpGraphNode> &graph,
                   const std::vector<size_t> &seeds)
{
    std::vector<char> reached(graph.size(), 0);
    std::vector<size_t> stack;
    for (const size_t seed : seeds) {
        if (seed < graph.size() && !reached[seed]) {
            reached[seed] = 1;
            stack.push_back(seed);
        }
    }
    while (!stack.empty()) {
        const size_t index = stack.back();
        stack.pop_back();
        for (const size_t succ : graph[index].succs) {
            if (succ < graph.size() && !reached[succ]) {
                reached[succ] = 1;
                stack.push_back(succ);
            }
        }
    }
    return reached;
}

/// Every step that reaches one of \p seeds along succ edges, the seeds
/// included: the steps a seed waits on.
inline std::vector<char>
OpGraphBackwardCone(const std::vector<rigExec::RigExecOpGraphNode> &graph,
                    const std::vector<size_t> &seeds)
{
    std::vector<char> reached(graph.size(), 0);
    std::vector<size_t> stack;
    for (const size_t seed : seeds) {
        if (seed < graph.size() && !reached[seed]) {
            reached[seed] = 1;
            stack.push_back(seed);
        }
    }
    while (!stack.empty()) {
        const size_t index = stack.back();
        stack.pop_back();
        for (const size_t pred : graph[index].preds) {
            if (pred < graph.size() && !reached[pred]) {
                reached[pred] = 1;
                stack.push_back(pred);
            }
        }
    }
    return reached;
}

/// The cluster-grain form of OpGraphForwardCone: every step a cluster
/// executor makes wait on the seeds, the seeds included. A cluster runs its
/// members back to back in program order once all of its predecessors are
/// done, so
///   - a cluster entered through an edge from another cluster waits as a
///     whole: every member shares its readiness;
///   - within a seed's own cluster only the members after the seed wait;
///     earlier members share the seed's readiness and run before it.
/// Steps flagged in \p sources (RigExecBakedStep::isSource, one entry per
/// step, or empty for none) run in the source pass ahead of every cluster
/// and never wait. Unclustered steps (-1) stand alone.
inline std::vector<char>
OpGraphClusterForwardCone(const std::vector<rigExec::RigExecOpGraphNode> &graph,
                          const std::vector<size_t> &seeds,
                          const std::vector<char> &sources = {})
{
    // Cluster ids are non-negative; an unclustered step gets its own key.
    const auto key = [&graph](size_t step) -> long long {
        return graph[step].cluster >= 0 ? graph[step].cluster
                                        : -1 - static_cast<long long>(step);
    };
    const auto isSource = [&sources](size_t step) {
        return step < sources.size() && sources[step];
    };
    // Members in program order, which is the order a cluster runs them.
    std::map<long long, std::vector<size_t>> members;
    std::vector<size_t> position(graph.size(), 0);
    for (size_t index = 0; index < graph.size(); ++index) {
        std::vector<size_t> &list = members[key(index)];
        position[index] = list.size();
        list.push_back(index);
    }
    // Per cluster, the first member position already marked as waiting.
    std::map<long long, size_t> waitingFrom;
    std::vector<char> reached(graph.size(), 0);
    std::vector<size_t> stack;
    const auto reach = [&](long long cluster, size_t from) {
        const std::vector<size_t> &list = members[cluster];
        const auto it = waitingFrom.find(cluster);
        const size_t end = it == waitingFrom.end() ? list.size() : it->second;
        if (from >= end) {
            return;
        }
        waitingFrom[cluster] = from;
        for (size_t p = from; p < end; ++p) {
            if (!isSource(list[p])) {
                reached[list[p]] = 1;
                stack.push_back(list[p]);
            }
        }
    };
    for (const size_t seed : seeds) {
        if (seed < graph.size() && !isSource(seed)) {
            reach(key(seed), position[seed]);
        }
    }
    while (!stack.empty()) {
        const size_t index = stack.back();
        stack.pop_back();
        for (const size_t succ : graph[index].succs) {
            // A successor in the same cluster follows `index` in member
            // order and is marked already.
            if (succ < graph.size() && key(succ) != key(index)) {
                reach(key(succ), 0);
            }
        }
    }
    return reached;
}

/// Every way \p trace fails to be a valid completion order over \p graph:
/// a step listed twice or outside the graph, nonpositive, repeated or
/// out-of-graph completion numbers, and an executed predecessor that did not finish before its
/// executed successor. Empty when the trace respects the graph.
inline std::vector<std::string>
CheckTraceRespectsEdges(const std::vector<rigExec::RigExecOpTraceEntry> &trace,
                        const std::vector<rigExec::RigExecOpGraphNode> &graph)
{
    std::vector<std::string> violations;
    std::vector<uint32_t> seqOf(graph.size(), 0);
    // Skipped candidates complete too, so executed rows may contain gaps.
    std::vector<char> seqSeen(graph.size() + 1, 0);
    for (const rigExec::RigExecOpTraceEntry &entry : trace) {
        const std::string what =
            "step " + std::to_string(entry.step) + " (" + entry.label + ")";
        if (entry.step >= graph.size()) {
            violations.push_back(what + " is not in the graph");
            continue;
        }
        if (seqOf[entry.step] != 0) {
            violations.push_back(what + " appears more than once");
            continue;
        }
        seqOf[entry.step] = entry.seq;
        if (entry.seq == 0 || entry.seq > graph.size()) {
            violations.push_back(what + " has seq " +
                                 std::to_string(entry.seq) + " outside 1.." +
                                 std::to_string(graph.size()));
        } else if (seqSeen[entry.seq]) {
            violations.push_back(what + " repeats seq " +
                                 std::to_string(entry.seq));
        } else {
            seqSeen[entry.seq] = 1;
        }
    }
    for (const rigExec::RigExecOpGraphNode &node : graph) {
        if (node.step >= seqOf.size() || seqOf[node.step] == 0) {
            continue;
        }
        for (const size_t pred : node.preds) {
            if (pred >= seqOf.size() || seqOf[pred] == 0) {
                continue;
            }
            if (seqOf[pred] >= seqOf[node.step]) {
                violations.push_back(
                    "edge " + std::to_string(pred) + " -> " +
                    std::to_string(node.step) + " (" + graph[pred].label +
                    " -> " + node.label + ") finished out of order: seq " +
                    std::to_string(seqOf[pred]) + " >= " +
                    std::to_string(seqOf[node.step]));
            }
        }
    }
    return violations;
}

/// Every way \p graph fails to be a DAG whose preds and succs mirror each
/// other. Empty when it is one.
inline std::vector<std::string>
CheckOpGraphIsAcyclic(const std::vector<rigExec::RigExecOpGraphNode> &graph)
{
    std::vector<std::string> violations;
    std::map<std::pair<size_t, size_t>, int> edges;  // +1 pred, +2 succ
    for (size_t index = 0; index < graph.size(); ++index) {
        const rigExec::RigExecOpGraphNode &node = graph[index];
        if (node.step != index) {
            violations.push_back("node " + std::to_string(index) +
                                 " names step " + std::to_string(node.step));
        }
        for (const size_t pred : node.preds) {
            if (pred >= graph.size()) {
                violations.push_back("step " + std::to_string(index) +
                                     " names missing pred " +
                                     std::to_string(pred));
                continue;
            }
            edges[{pred, index}] |= 1;
        }
        for (const size_t succ : node.succs) {
            if (succ >= graph.size()) {
                violations.push_back("step " + std::to_string(index) +
                                     " names missing succ " +
                                     std::to_string(succ));
                continue;
            }
            edges[{index, succ}] |= 2;
        }
    }
    for (const auto &[edge, sides] : edges) {
        if (sides != 3) {
            violations.push_back(
                "edge " + std::to_string(edge.first) + " -> " +
                std::to_string(edge.second) + " is listed only as a " +
                (sides == 1 ? "pred" : "succ"));
        }
    }
    // Kahn over the pred lists.
    std::vector<size_t> remaining(graph.size(), 0);
    std::vector<std::vector<size_t>> out(graph.size());
    for (const auto &[edge, sides] : edges) {
        if (sides & 1) {
            ++remaining[edge.second];
            out[edge.first].push_back(edge.second);
        }
    }
    std::vector<size_t> ready;
    for (size_t index = 0; index < graph.size(); ++index) {
        if (remaining[index] == 0) {
            ready.push_back(index);
        }
    }
    size_t visited = 0;
    while (!ready.empty()) {
        const size_t index = ready.back();
        ready.pop_back();
        ++visited;
        for (const size_t succ : out[index]) {
            if (--remaining[succ] == 0) {
                ready.push_back(succ);
            }
        }
    }
    if (visited != graph.size()) {
        violations.push_back(std::to_string(graph.size() - visited) +
                             " step(s) lie on or behind a cycle");
    }
    return violations;
}

}  // namespace rigExecTest

#endif  // RIGEXEC_TESTS_OP_TRACE_H
