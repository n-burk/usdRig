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

/// The entries whose kind, domain and label contain the given substrings;
/// an empty substring matches everything.
inline std::vector<rigExec::RigExecOpTraceEntry>
FindTraceEntries(const std::vector<rigExec::RigExecOpTraceEntry> &trace,
                 const std::string &kind, const std::string &domain = "",
                 const std::string &label = "")
{
    std::vector<rigExec::RigExecOpTraceEntry> out;
    for (const rigExec::RigExecOpTraceEntry &entry : trace) {
        if (entry.kind.find(kind) != std::string::npos &&
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

/// Every way \p trace fails to be a valid completion order over \p graph:
/// a step listed twice or outside the graph, sequence numbers that are not
/// exactly 1..N, and an executed predecessor that did not finish before its
/// executed successor. Empty when the trace respects the graph.
inline std::vector<std::string>
CheckTraceRespectsEdges(const std::vector<rigExec::RigExecOpTraceEntry> &trace,
                        const std::vector<rigExec::RigExecOpGraphNode> &graph)
{
    std::vector<std::string> violations;
    std::vector<uint32_t> seqOf(graph.size(), 0);
    std::vector<char> seqSeen(trace.size() + 1, 0);
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
        if (entry.seq == 0 || entry.seq > trace.size()) {
            violations.push_back(what + " has seq " +
                                 std::to_string(entry.seq) + " outside 1.." +
                                 std::to_string(trace.size()));
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
