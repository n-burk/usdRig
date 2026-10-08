#include "opGraph.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <sstream>

namespace rigExec {
namespace {
void Unique(std::vector<uint32_t> *values)
{
    std::sort(values->begin(), values->end());
    values->erase(std::unique(values->begin(), values->end()), values->end());
}
std::vector<std::vector<uint32_t>> Components(
    const std::vector<std::vector<uint32_t>> &successors,
    const std::vector<std::vector<uint32_t>> &predecessors)
{
    const size_t count = successors.size();
    std::vector<char> visited(count, 0);
    std::vector<uint32_t> finish;
    for (uint32_t start = 0; start < count; ++start) {
        if (visited[start]) continue;
        std::vector<std::pair<uint32_t, size_t>> stack{{start, 0}};
        visited[start] = 1;
        while (!stack.empty()) {
            auto &top = stack.back();
            if (top.second == successors[top.first].size()) {
                finish.push_back(top.first); stack.pop_back(); continue;
            }
            const uint32_t next = successors[top.first][top.second++];
            if (!visited[next]) { visited[next] = 1; stack.emplace_back(next, 0); }
        }
    }
    std::fill(visited.begin(), visited.end(), 0);
    std::vector<std::vector<uint32_t>> result;
    for (auto it = finish.rbegin(); it != finish.rend(); ++it) {
        if (visited[*it]) continue;
        std::vector<uint32_t> component, pending{*it}; visited[*it] = 1;
        while (!pending.empty()) {
            const uint32_t at = pending.back(); pending.pop_back(); component.push_back(at);
            for (uint32_t next : predecessors[at]) if (!visited[next]) {
                visited[next] = 1; pending.push_back(next);
            }
        }
        if (component.size() > 1 ||
            std::find(successors[component[0]].begin(), successors[component[0]].end(),
                      component[0]) != successors[component[0]].end())
            result.push_back(std::move(component));
    }
    return result;
}
std::vector<std::string> Loop(const std::vector<uint32_t> &component,
    const std::vector<std::vector<uint32_t>> &successors,
    const std::vector<RigExecOpDescriptor> &descriptors)
{
    std::set<uint32_t> members(component.begin(), component.end());
    std::vector<uint32_t> starts(component);
    std::sort(starts.begin(), starts.end(), [&](uint32_t a, uint32_t b) {
        return descriptors[a].key < descriptors[b].key;
    });
    auto edges = successors;
    for (uint32_t member : component)
        std::sort(edges[member].begin(), edges[member].end(), [&](uint32_t a, uint32_t b) {
            return descriptors[a].key < descriptors[b].key;
        });
    std::vector<char> state(descriptors.size(), 0);
    std::vector<int32_t> position(descriptors.size(), -1);
    std::vector<std::pair<uint32_t, size_t>> stack{{starts.front(), 0}};
    state[starts.front()] = 1; position[starts.front()] = 0;
    while (!stack.empty()) {
        auto &top = stack.back();
        if (top.second == edges[top.first].size()) {
            state[top.first] = 2; position[top.first] = -1; stack.pop_back(); continue;
        }
        const uint32_t next = edges[top.first][top.second++];
        if (!members.count(next)) continue;
        if (state[next] == 1) {
            std::vector<std::string> loop;
            for (size_t i = size_t(position[next]); i < stack.size(); ++i)
                loop.push_back(descriptors[stack[i].first].key);
            loop.push_back(descriptors[next].key); return loop;
        }
        if (!state[next]) {
            state[next] = 1; position[next] = int32_t(stack.size()); stack.emplace_back(next, 0);
        }
    }
    return {};
}
} // namespace

bool RigExecCompileOpGraph(const std::vector<RigExecOpDescriptor> &descriptors,
    const std::vector<RigExecValueId> &leaves, RigExecCyclePolicy policy,
    RigExecCompiledGraph *out, std::string *error)
{
    return RigExecCompileOpGraph(descriptors,leaves,policy,nullptr,out,error);
}

bool RigExecCompileOpGraph(const std::vector<RigExecOpDescriptor> &descriptors,
    const std::vector<RigExecValueId> &leaves, RigExecCyclePolicy policy,
    const RigExecOpExclusionProof *retainedExclusions,
    RigExecCompiledGraph *out, std::string *error)
{
    const auto fail = [&](const std::string &message) { if (error) *error = message; return false; };
    if (!out) return fail("null graph output");
    if (descriptors.size() > UINT32_MAX) return fail("too many operations");
    const size_t count = descriptors.size();
    std::set<RigExecValueId> leafSet(leaves.begin(), leaves.end());
    std::map<RigExecValueId, uint32_t> producers;
    std::set<std::string> keys;
    for (uint32_t i = 0; i < count; ++i) {
        if (descriptors[i].key.empty() || !keys.insert(descriptors[i].key).second)
            return fail("empty or duplicate operation key: " + descriptors[i].key);
        for (RigExecValueId value : descriptors[i].writes) {
            auto added = producers.emplace(value, i);
            if (!added.second && added.first->second != i)
                return fail("multiple producers for value " + std::to_string(value) + ": " +
                    descriptors[added.first->second].key + " and " + descriptors[i].key);
            if (leafSet.count(value)) return fail("operation writes a sampled leaf: " + descriptors[i].key);
        }
    }
    std::vector<std::vector<uint32_t>> preds(count), succs(count);
    for (uint32_t i = 0; i < count; ++i) {
        preds[i] = descriptors[i].predecessors;
        for (uint32_t predecessor : preds[i])
            if (predecessor >= count) return fail("predecessor out of range: " + descriptors[i].key);
        for (RigExecValueId value : descriptors[i].reads) {
            const auto producer = producers.find(value);
            if (producer != producers.end()) preds[i].push_back(producer->second);
            else if (!leafSet.count(value)) return fail("read has no producer or sampled leaf: " + descriptors[i].key);
        }
        Unique(&preds[i]);
        for (uint32_t predecessor : preds[i]) succs[predecessor].push_back(i);
    }
    for (auto &successors : succs) Unique(&successors);
    RigExecCompiledGraph graph;
    std::vector<char> excluded(count, 0);
    // Keep the reporting pair together when ordering SCCs. The same component
    // result remains the sole authority for exclusions and full membership.
    using CycleReport = std::pair<std::vector<std::string>, std::vector<std::string>>;
    std::vector<CycleReport> reports;
    if (retainedExclusions) {
        if (retainedExclusions->cycles.size()!=retainedExclusions->cycleMembers.size())
            return fail("retained exclusion report cardinality mismatch");
        std::set<std::string> members;
        for (size_t i=0;i<retainedExclusions->cycles.size();++i) {
            const auto &loop=retainedExclusions->cycles[i];
            const auto &component=retainedExclusions->cycleMembers[i];
            if(loop.size()<2 || loop.front()!=loop.back() || component.empty() ||
               !std::is_sorted(component.begin(),component.end()) ||
               std::adjacent_find(component.begin(),component.end())!=component.end())
                return fail("invalid retained exclusion SCC report");
            for(const auto &key:component) {
                if(key.empty())return fail("empty retained exclusion member");
                members.insert(key);
            }
            for(const auto &key:loop)if(!std::binary_search(component.begin(),component.end(),key))
                return fail("retained exclusion loop is outside SCC membership");
            reports.emplace_back(loop,component);
        }
        std::set<std::string> retainedKeys;
        for(const auto &key:retainedExclusions->keys) {
            if(!retainedKeys.insert(key).second || !members.count(key))
                return fail("duplicate or unproved retained exclusion key: "+key);
            const auto found=std::find_if(descriptors.begin(),descriptors.end(),
                [&](const auto &descriptor){return descriptor.key==key;});
            if(found==descriptors.end())return fail("retained exclusion has no descriptor: "+key);
            excluded[size_t(found-descriptors.begin())]=1;
        }
        std::set<std::string> removedKeys;
        for(const auto &key:retainedExclusions->removedKeys) {
            if(!removedKeys.insert(key).second || retainedKeys.count(key) || !members.count(key))
                return fail("duplicate, overlapping or unproved removed exclusion key: "+key);
            if(keys.count(key))return fail("removed exclusion still has a descriptor: "+key);
        }
        for(const auto &key:members)if(!retainedKeys.count(key) && !removedKeys.count(key))
            return fail("retained SCC member has no exclusion mapping: "+key);
    }
    // Existing SCC discovery sees only the remaining active subgraph. Retained
    // bodies stay unavailable; no artificial edges resurrect a former cycle.
    if(retainedExclusions && !retainedExclusions->keys.empty())
    for(uint32_t i=0;i<count;++i) {
        if(excluded[i]) {preds[i].clear();succs[i].clear();continue;}
        auto removeExcluded=[&](auto &edges){edges.erase(std::remove_if(edges.begin(),edges.end(),
            [&](uint32_t at){return excluded[at]!=0;}),edges.end());};
        removeExcluded(preds[i]);removeExcluded(succs[i]);
    }
    for (const auto &component : Components(succs, preds)) {
        std::vector<std::string> members;
        members.reserve(component.size());
        for (uint32_t i : component) {
            excluded[i] = 1;
            members.push_back(descriptors[i].key);
        }
        std::sort(members.begin(), members.end());
        reports.emplace_back(Loop(component, succs, descriptors), std::move(members));
    }
    std::sort(reports.begin(), reports.end());
    reports.erase(std::unique(reports.begin(),reports.end()),reports.end());
    for (auto &report : reports) {
        graph.cycles.push_back(std::move(report.first));
        graph.cycleMembers.push_back(std::move(report.second));
    }
    if (!graph.cycles.empty() && policy == RigExecCyclePolicy::Reject) {
        std::string message = "operation cycle";
        for (const auto &loop : graph.cycles) {
            message += "\n";
            for (size_t i = 0; i < loop.size(); ++i) {
                if (i) message += " -> "; message += loop[i];
            }
        }
        return fail(message);
    }
    const auto after = [&](uint32_t a, uint32_t b) { return descriptors[a].key > descriptors[b].key; };
    std::priority_queue<uint32_t, std::vector<uint32_t>, decltype(after)> ready(after);
    std::vector<uint32_t> unresolved(count, 0), order;
    for (uint32_t i = 0; i < count; ++i) if (!excluded[i]) {
        for (uint32_t predecessor : preds[i]) if (!excluded[predecessor]) ++unresolved[i];
        if (!unresolved[i]) ready.push(i);
    }
    while (!ready.empty()) {
        const uint32_t at = ready.top(); ready.pop(); order.push_back(at);
        for (uint32_t next : succs[at]) if (!excluded[next] && --unresolved[next] == 0) ready.push(next);
    }
    graph.canonicalIndex.assign(count, -1);
    for (uint32_t i = 0; i < order.size(); ++i) graph.canonicalIndex[order[i]] = int32_t(i);
    graph.ops.resize(order.size());
    std::vector<size_t> distance(order.size(), 1);
    for (uint32_t i = 0; i < order.size(); ++i) {
        const uint32_t old = order[i]; auto &op = graph.ops[i];
        op.descriptor = descriptors[old]; op.descriptor.predecessors.clear(); op.originalIndex = old;
        for (uint32_t predecessor : preds[old]) if (!excluded[predecessor]) {
            const uint32_t canonical = uint32_t(graph.canonicalIndex[predecessor]);
            op.predecessors.push_back(canonical);
            distance[i] = std::max(distance[i], distance[canonical] + 1);
        }
        for (uint32_t next : succs[old]) if (!excluded[next])
            op.successors.push_back(uint32_t(graph.canonicalIndex[next]));
        Unique(&op.predecessors); Unique(&op.successors);
        for (RigExecValueId value : op.descriptor.reads) graph.readers[value].push_back(i);
        graph.longestPath = std::max(graph.longestPath, distance[i]);
    }
    for (auto &reader : graph.readers) Unique(&reader.second);
    if (!RigExecLowerOpClusters(&graph, {}, 0, error)) return false;
    *out = std::move(graph); return true;
}

bool RigExecLowerOpClusters(RigExecCompiledGraph *graph,
    const std::vector<double> &costs, double grain, std::string *error)
{
    const auto fail = [&](const std::string &message) { if (error) *error = message; return false; };
    if (!graph || !std::isfinite(grain) || grain < 0)
        return fail("invalid cluster lowering grain");
    const size_t count = graph->ops.size();
    if (count > UINT32_MAX || (!costs.empty() && costs.size() != count))
        return fail("cluster costs do not match canonical operations");
    for (double cost : costs)
        if (!std::isfinite(cost) || cost < 0) return fail("invalid operation cost");
    const auto cost = [&](uint32_t op) { return costs.empty() ? 1.0 : costs[op]; };
    std::vector<uint32_t> membership(count, UINT32_MAX);
    std::vector<RigExecOpCluster> clusters;
    for (uint32_t start = 0; start < count; ++start) {
        if (membership[start] != UINT32_MAX) continue;
        const uint32_t cluster = uint32_t(clusters.size());
        RigExecOpCluster group;
        uint32_t current = start;
        double total = 0;
        while (true) {
            group.members.push_back(current); membership[current] = cluster;
            total += cost(current);
            const auto &op = graph->ops[current];
            if (grain == 0 || group.members.size() == 64 || op.successors.size() != 1) break;
            const uint32_t next = op.successors.front();
            if (next >= count || next <= current) return fail("non-topological operation graph");
            const auto &successor = graph->ops[next];
            if (membership[next] != UINT32_MAX || successor.predecessors.size() != 1 ||
                successor.predecessors.front() != current || total + cost(next) > grain) break;
            current = next;
        }
        clusters.push_back(std::move(group));
    }
    for (uint32_t op = 0; op < count; ++op) {
        for (uint32_t predecessor : graph->ops[op].predecessors) {
            if (predecessor >= op) return fail("non-topological operation predecessor");
            const uint32_t before = membership[predecessor], after = membership[op];
            if (before != after) {
                clusters[after].predecessors.push_back(before);
                clusters[before].successors.push_back(after);
            }
        }
    }
    for (auto &cluster : clusters) { Unique(&cluster.predecessors); Unique(&cluster.successors); }
    graph->clusters = std::move(clusters); graph->opClusters = std::move(membership);
    return true;
}

bool RigExecValidateOpClusters(const RigExecCompiledGraph &graph, std::string *error)
{
    const auto fail = [&](const std::string &message) { if (error) *error = message; return false; };
    const size_t count = graph.ops.size(), groups = graph.clusters.size();
    if (graph.opClusters.size() != count || (!count && groups))
        return fail("cluster membership cardinality differs from operations");
    std::vector<char> seen(count, 0);
    uint32_t previousStart = 0;
    for (uint32_t group = 0; group < groups; ++group) {
        const auto &cluster = graph.clusters[group];
        if (cluster.members.empty() || cluster.members.size() > 64)
            return fail("empty or oversized operation cluster");
        if (group && cluster.members.front() <= previousStart)
            return fail("cluster order is not canonical");
        previousStart = cluster.members.front();
        for (size_t part = 0; part < cluster.members.size(); ++part) {
            const uint32_t op = cluster.members[part];
            if (op >= count || seen[op] || graph.opClusters[op] != group)
                return fail("duplicate, missing or invalid operation cluster member");
            seen[op] = 1;
            if (part) {
                const uint32_t before = cluster.members[part - 1];
                if (before >= op || graph.ops[before].successors != std::vector<uint32_t>{op} ||
                    graph.ops[op].predecessors != std::vector<uint32_t>{before})
                    return fail("cluster introduces an extra causal dependency");
            }
        }
    }
    if (std::find(seen.begin(), seen.end(), char(0)) != seen.end())
        return fail("operation missing from cluster lowering");
    std::vector<std::vector<uint32_t>> predecessors(groups), successors(groups);
    for (uint32_t op = 0; op < count; ++op) {
        for (uint32_t before : graph.ops[op].predecessors) {
            if (before >= op) return fail("operation predecessor is not topological");
            const uint32_t a = graph.opClusters[before], b = graph.opClusters[op];
            if (a != b) {
                if (a >= b) return fail("cluster predecessor is not topological");
                predecessors[b].push_back(a); successors[a].push_back(b);
            }
        }
    }
    for (uint32_t group = 0; group < groups; ++group) {
        Unique(&predecessors[group]); Unique(&successors[group]);
        if (graph.clusters[group].predecessors != predecessors[group] ||
            graph.clusters[group].successors != successors[group])
            return fail("cluster edges differ from operation dependencies");
    }
    return true;
}

bool RigExecExecuteOpGraph(const RigExecCompiledGraph &graph,
    const std::vector<RigExecValueId> &changedLeaves,
    const std::vector<uint32_t> &seedOps, bool force,
    const RigExecOpCallbacks &callbacks, RigExecOpExecution *out, std::string *error,
    RigExecOpWorkspace *suppliedWorkspace, const std::vector<uint32_t> *candidateOps)
{
    const auto fail = [&](const std::string &message) { if (error) *error = message; return false; };
    if (!out || !callbacks.run || !callbacks.skip || !callbacks.changed)
        return fail("incomplete operation execution callbacks");
    if (bool(callbacks.dispatch) != bool(callbacks.wait)) return fail("dispatch requires a join");
    const size_t count = graph.ops.size(), clusterCount = graph.clusters.size();
    if (graph.opClusters.size() != count || (count && !clusterCount))
        return fail("operation graph has no cluster lowering");
    for (uint32_t seed : seedOps)
        if (seed >= count) return fail("operation seed out of range");
    if (candidateOps) for (uint32_t candidate : *candidateOps)
        if (candidate >= count) return fail("operation candidate out of range");
    RigExecOpExecution &result = *out;
    RigExecOpWorkspace localWorkspace;
    RigExecOpWorkspace &workspace = suppliedWorkspace ? *suppliedWorkspace : localWorkspace;
    result.candidates.assign(count, 0); result.ran.assign(count, 0); result.completion.assign(count, 0);
    auto &seeded = workspace.seeded;
    seeded.assign(count, force ? 1 : 0);
    auto &pending = workspace.pending;
    pending.clear(); pending.reserve(count);
    const auto seed = [&](uint32_t i, bool mandatory) {
        if (mandatory) seeded[i] = 1;
        if (!result.candidates[i]) { result.candidates[i] = 1; pending.push_back(i); }
    };
    for (uint32_t i : seedOps) seed(i, true);
    if (candidateOps) for (uint32_t i : *candidateOps) seed(i, false);
    for (RigExecValueId value : changedLeaves) {
        const auto readers = graph.readers.find(value);
        if (readers != graph.readers.end()) for (uint32_t i : readers->second) seed(i, false);
    }
    for (uint32_t i = 0; i < count; ++i)
        if (force || graph.ops[i].descriptor.volatileInput) seed(i, true);
    for (size_t at = 0; at < pending.size(); ++at)
        for (uint32_t next : graph.ops[pending[at]].successors)
            if (!result.candidates[next]) { result.candidates[next] = 1; pending.push_back(next); }
    auto &clusterCandidates = workspace.clusterCandidates;
    clusterCandidates.assign(clusterCount, 0);
    for (uint32_t op : pending) clusterCandidates[graph.opClusters[op]] = 1;
    auto &unresolved = workspace.unresolved;
    unresolved.resize(clusterCount);
    auto &ready = workspace.ready;
    ready.clear(); ready.reserve(clusterCount);
    for (uint32_t i = 0; i < count; ++i) if (!result.candidates[i]) callbacks.skip(i);
    for (uint32_t i = 0; i < clusterCount; ++i) {
        uint32_t dependencies = 0;
        if (clusterCandidates[i]) {
            for (uint32_t predecessor : graph.clusters[i].predecessors)
                if (clusterCandidates[predecessor]) ++dependencies;
            if (!dependencies) ready.push_back(i);
        }
        unresolved[i].value.store(dependencies, std::memory_order_relaxed);
    }
    std::atomic<uint64_t> sequence{0};
    std::atomic<size_t> executed{0}, skipped{count - pending.size()};
    std::atomic<bool> succeeded{true};
    const auto body = [&](uint32_t i) {
        if (!succeeded.load(std::memory_order_acquire)) {
            callbacks.skip(i); skipped.fetch_add(1, std::memory_order_relaxed);
            result.completion[i] = sequence.fetch_add(1, std::memory_order_relaxed) + 1;
            return;
        }
        bool needed = seeded[i] != 0;
        if (callbacks.inputsChanged) needed = callbacks.inputsChanged(i) || needed;
        else for (RigExecValueId input : graph.ops[i].descriptor.reads)
            needed = (callbacks.inputChanged ? callbacks.inputChanged(i, input)
                                             : callbacks.changed(input)) || needed;
        if (needed) {
            result.ran[i] = 1;
            if (!callbacks.run(i)) succeeded.store(false, std::memory_order_release);
            executed.fetch_add(1, std::memory_order_relaxed);
        } else { callbacks.skip(i); skipped.fetch_add(1, std::memory_order_relaxed); }
        result.completion[i] = sequence.fetch_add(1, std::memory_order_relaxed) + 1;
    };
    const auto clusterBody = [&](uint32_t cluster) {
        for (uint32_t op : graph.clusters[cluster].members)
            if (result.candidates[op]) body(op);
    };
    if (!callbacks.dispatch) {
        for (size_t at = 0; at < ready.size(); ++at) {
            const uint32_t cluster = ready[at]; clusterBody(cluster);
            for (uint32_t next : graph.clusters[cluster].successors)
                if (clusterCandidates[next] && unresolved[next].value.fetch_sub(1, std::memory_order_acq_rel) == 1)
                    ready.push_back(next);
        }
    } else {
        std::function<void(uint32_t)> submit;
        submit = [&](uint32_t cluster) {
            callbacks.dispatch([&, cluster] {
                clusterBody(cluster);
                for (uint32_t next : graph.clusters[cluster].successors)
                    if (clusterCandidates[next] && unresolved[next].value.fetch_sub(1, std::memory_order_acq_rel) == 1)
                        submit(next);
            });
        };
        for (uint32_t cluster : ready) submit(cluster);
        callbacks.wait();
    }
    result.executed = executed.load(std::memory_order_relaxed);
    result.skipped = skipped.load(std::memory_order_relaxed);
    const bool valid = succeeded.load(std::memory_order_acquire);
    if (!valid && error) *error = "an operation failed";
    return valid;
}
} // namespace rigExec
