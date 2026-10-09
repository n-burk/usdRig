#ifndef RIGEXEC_GRAPH_OP_VALUES_H
#define RIGEXEC_GRAPH_OP_VALUES_H

#include "opGraph.h"
#include "providerRecords.h"
#include <algorithm>
#include <array>
#include <map>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace rigExec {

/// The adapter retains one exact key for each produced version, including
/// presence, element count and error state. Scratch capacity survives runs.
struct RigExecOpValueState {
    uint32_t domain = 0, slot = 0;
    std::string key, scratch;
    bool initialized = false;
    char changed = 0;
    uint64_t revision = 0;
};

struct RigExecOpAdapterState {
    std::vector<RigExecOpValueState> values;
    std::vector<RigExecValueId> leaves, changedLeaves;
    /// Every value whose changed flag the last run set: its changed leaves
    /// and the changed writes of its ops that ran. Gathered by the owner
    /// after the join; the next run clears exactly these flags.
    std::vector<RigExecValueId> changedValues;
    std::vector<RigExecValueId> excludedValues;
    /// RigExecOpCallbacks::skipEffects, filled by the backend at compile.
    std::vector<uint32_t> skipEffects;
    std::vector<uint32_t> seeds, candidateOps;
    std::vector<uint64_t> inputRevisions;
    std::vector<std::string> inputKeys, inputScratch;
    std::vector<std::string> sourceKeys, sourceScratch;
    std::vector<std::vector<uint32_t>> coveredPropertyInputs;
    std::vector<std::vector<std::pair<uint32_t,uint32_t>>> coveredTypedInputs;
    std::vector<char> inputExact;
    /// Native retained pure bodies are candidates on their replacement epoch first run.
    std::vector<char> retainedFirst;
    /// Per op, set at compile: its source key reads no state that can change,
    /// so a run that rebuilds no source keys treats it as an equal compare.
    std::vector<char> constantSource;
    /// Ascending ops such a run still visits: keyed ones and those that always run.
    std::vector<uint32_t> sourceVisits;
    bool compiled = false, everRan = false;
    bool parallel = false, measuring = false;
    /// Native: rebuild every constant source key each run and verify it.
    bool verifyConstantSources = false;
    /// Native (RIGEXEC_VERIFY_LEAF_VERSIONS): beside each key that carries
    /// path-leaf content versions, the same key over the leaves' contents,
    /// and per op whether the two told a different change this run. Each op
    /// writes only its own entries; the owner reports after the join.
    bool verifyLeafVersions = false;
    std::vector<std::string> contentSourceKeys, contentInputKeys;
    std::vector<char> leafVersionMismatch;
};

inline RigExecValueId RigExecOpAddValue(RigExecOpAdapterState *, uint32_t, uint32_t);

/// Preindex typed producers before binding any reads. Backends supply
/// their domain descriptors; the compiler alone owns canonical ordering.
template <class Step, class Domain, class Begin, class End, class Key, class Sampled>
inline bool RigExecOpCompileAdapter(const std::vector<Step> &steps,
    Domain domain, Begin begin, End end, Key key, Sampled sampled,
    RigExecCompiledGraph *graph, RigExecOpAdapterState *state,
    std::string *error, RigExecCyclePolicy cyclePolicy=RigExecCyclePolicy::Reject,
    const std::function<const std::vector<std::string> &(uint32_t)> &semanticPredecessors = {},
    const RigExecOpExclusionProof *retainedExclusions = nullptr)
{
    *state = RigExecOpAdapterState();
    std::map<std::pair<uint32_t, uint32_t>, RigExecValueId> values;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> writers;
    std::vector<RigExecOpDescriptor> descriptors(steps.size());
    for (uint32_t i = 0; i < steps.size(); ++i) {
        const auto &step = steps[i]; auto &op = descriptors[i];
        op.key = key(step); op.kind = uint32_t(step.kind);
        op.volatileInput = step.externalReads;
        for (const auto &range : step.writes) {
            for (uint32_t slot = begin(range); slot < end(range); ++slot) {
                const auto typed = std::make_pair(domain(range), slot);
                const auto found = writers.emplace(typed,i);
                if (!found.second && found.first->second != i) {
                    if (error) *error = "multiple typed producers for domain " +
                        std::to_string(typed.first) + " slot " + std::to_string(slot) +
                        ": " + key(steps[found.first->second]) + " and " + key(step);
                    return false;
                }
                auto value = values.find(typed);
                if (value == values.end()) value = values.emplace(typed,
                    RigExecOpAddValue(state,typed.first,slot)).first;
                op.writes.push_back(value->second);
            }
        }
    }
    // Authored semantic prerequisites are distinct from generated graph
    // predecessors. Backends opt in with stable descriptor identities only.
    // Never import a historical scheduling list from Step.preds.
    if (semanticPredecessors) {
        std::map<std::string, uint32_t> byKey;
        for (uint32_t i = 0; i < descriptors.size(); ++i)
            byKey.emplace(descriptors[i].key, i);
        for (uint32_t i = 0; i < descriptors.size(); ++i)
            for (const auto &required : semanticPredecessors(i)) {
                const auto found = byKey.find(required);
                if (found == byKey.end()) {
                    if (error) *error = "semantic predecessor has no descriptor: " +
                        descriptors[i].key + " requires " + required;
                    return false;
                }
                descriptors[i].predecessors.push_back(found->second);
            }
    }
    for (uint32_t i = 0; i < steps.size(); ++i) {
        auto &op = descriptors[i];
        for (const auto &range : steps[i].reads) {
            for (uint32_t slot = begin(range); slot < end(range); ++slot) {
                const auto typed = std::make_pair(domain(range),slot);
                auto value = values.find(typed);
                if (value == values.end()) {
                    if (!sampled(typed.first,slot)) {
                        if(error) *error="read has no producer or declared sampled source: "+
                            key(steps[i])+" domain "+std::to_string(typed.first)+
                            " slot "+std::to_string(slot);
                        return false;
                    }
                    value = values.emplace(typed,RigExecOpAddValue(state,typed.first,slot)).first;
                    state->leaves.push_back(value->second);
                }
                op.reads.push_back(value->second);
            }
        }
    }
    state->compiled = RigExecCompileOpGraph(descriptors, state->leaves,
        cyclePolicy, retainedExclusions, graph, error);
    if (state->compiled) {
        for(uint32_t i=0;i<descriptors.size();++i) if(graph->canonicalIndex[i]<0)
            for(auto id:descriptors[i].writes) state->excludedValues.push_back(id);
        state->leaves.insert(state->leaves.end(),state->excludedValues.begin(),state->excludedValues.end());
        std::vector<RigExecValueId> order(state->values.size()), remap(state->values.size());
        for (RigExecValueId i=0;i<order.size();++i) order[size_t(i)]=i;
        std::sort(order.begin(),order.end(),[&](RigExecValueId a,RigExecValueId b) {
            const auto &x=state->values[size_t(a)], &y=state->values[size_t(b)];
            return std::make_pair(x.domain,x.slot)<std::make_pair(y.domain,y.slot);
        });
        std::vector<RigExecOpValueState> canonical; canonical.reserve(order.size());
        for (RigExecValueId i=0;i<order.size();++i) {
            remap[size_t(order[size_t(i)])]=i;
            canonical.push_back(std::move(state->values[size_t(order[size_t(i)])]));
        }
        state->values=std::move(canonical);
        for (auto &id:state->leaves) id=remap[size_t(id)];
        for (auto &id:state->excludedValues) id=remap[size_t(id)];
        std::sort(state->leaves.begin(),state->leaves.end());
        state->leaves.erase(std::unique(state->leaves.begin(),state->leaves.end()),state->leaves.end());
        graph->readers.clear();
        for (uint32_t i=0;i<graph->ops.size();++i) {
            auto &op=graph->ops[i];
            for (auto &id:op.descriptor.reads) { id=remap[size_t(id)]; graph->readers[id].push_back(i); }
            for (auto &id:op.descriptor.writes) id=remap[size_t(id)];
            std::sort(op.descriptor.reads.begin(),op.descriptor.reads.end());
            std::sort(op.descriptor.writes.begin(),op.descriptor.writes.end());
            op.descriptor.reads.erase(std::unique(op.descriptor.reads.begin(),op.descriptor.reads.end()),op.descriptor.reads.end());
            op.descriptor.writes.erase(std::unique(op.descriptor.writes.begin(),op.descriptor.writes.end()),op.descriptor.writes.end());
        }
        for (auto &entry:graph->readers) entry.second.erase(std::unique(entry.second.begin(),entry.second.end()),entry.second.end());
    }
    return state->compiled;
}

inline RigExecValueId RigExecOpAddValue(RigExecOpAdapterState *state,
    uint32_t domain, uint32_t slot)
{
    const RigExecValueId id = state->values.size();
    state->values.push_back({domain, slot});
    return id;
}

/// Serialize fields explicitly: object padding and container addresses are
/// never part of a value's identity.
template <class T> inline void RigExecOpKeyAppend(std::string *key, const T &v)
{
    key->append(reinterpret_cast<const char *>(&v), sizeof(T));
}
inline void RigExecOpKeyAppend(std::string *key, const std::string &v)
{
    RigExecOpKeyAppend(key, v.size()); key->append(v);
}
/// Element types whose key bytes are exactly their object bytes: padding-free
/// and trivially copyable, so a contiguous run of them keys with one append.
/// Integers, float, double and std::array of them qualify here. A backend
/// opts in its fixed-size vector and matrix types, asserting their size, in
/// a header that every user of their keys includes.
template <class T> struct RigExecOpKeyBulkElement : std::integral_constant<bool,
    std::is_integral<T>::value || std::is_same<T, float>::value ||
    std::is_same<T, double>::value> {};
template <class T, size_t N> struct RigExecOpKeyBulkElement<std::array<T, N>>
    : RigExecOpKeyBulkElement<T> {
    static_assert(sizeof(std::array<T, N>) == N * sizeof(T),
                  "key runs need padding-free elements");
};
/// Containers whose elements are one contiguous array (std::vector<bool> is not).
template <class C, class = void> struct RigExecOpKeyContiguous : std::false_type {};
template <class C> struct RigExecOpKeyContiguous<C,
    std::void_t<decltype(std::declval<const C &>().data())>>
    : std::is_same<decltype(std::declval<const C &>().data()),
                   const typename C::value_type *> {};
/// Appends \p count elements as one run: the same bytes as appending each
/// element's fields in memory order, signed zeros and NaN payloads included.
template <class T> inline void RigExecOpKeyAppendRun(std::string *key,
    const T *data, size_t count)
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "key runs copy object bytes");
    if (count) key->append(reinterpret_cast<const char *>(data), count * sizeof(T));
}
template <class T> inline void RigExecOpKeyArray(std::string *key, const T &v)
{
    using Element = typename T::value_type;
    RigExecOpKeyAppend(key, v.size());
    if constexpr (RigExecOpKeyBulkElement<Element>::value &&
                  RigExecOpKeyContiguous<T>::value)
        RigExecOpKeyAppendRun(key, v.data(), v.size());
    else
        for (const auto &item : v) RigExecOpKeyAppend(key, static_cast<const Element &>(item));
}
template <class T> struct RigExecOpKeyIsVector : std::false_type {};
template <class T, class A> struct RigExecOpKeyIsVector<std::vector<T, A>> : std::true_type {};
/// A provider value's payload. Vector alternatives key by count and
/// contents, strings element by element; the variant index is the caller's.
inline void RigExecOpKeyPlainValue(std::string *key, const RigExecProviderPlainValue &value)
{
    std::visit([&](const auto &v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same<T, RigExecProviderPlainFrame>::value) {
            RigExecOpKeyAppend(key, v.points); RigExecOpKeyAppend(key, v.flags);
        } else if constexpr (RigExecOpKeyIsVector<T>::value) RigExecOpKeyArray(key, v);
        else if constexpr (!std::is_same<T, std::monostate>::value) RigExecOpKeyAppend(key, v);
    }, value);
}

template <class Sample> inline void RigExecOpPublishValue(
    RigExecOpValueState *value, Sample sample)
{
    value->scratch.clear(); sample(value->domain, value->slot, &value->scratch);
    value->changed = !value->initialized || value->scratch != value->key;
    if (value->changed) ++value->revision;
    value->key.swap(value->scratch); value->initialized = true;
}

/// Clears every change flag at the start of a run. Only publication sets a
/// flag, and the last run gathered each one it set into changedValues.
inline void RigExecOpClearChanges(RigExecOpAdapterState *state)
{
    for (RigExecValueId id : state->changedValues) state->values[size_t(id)].changed = 0;
    state->changedValues.clear();
}

/// Owner, after the join and on failure too: gathers the flags this run set,
/// from its changed leaves and the writes of the ops that ran.
inline void RigExecOpGatherChanges(RigExecOpAdapterState *state,
    const RigExecCompiledGraph &graph, const std::vector<char> &ran)
{
    state->changedValues.assign(state->changedLeaves.begin(), state->changedLeaves.end());
    for (size_t c = 0; c < graph.ops.size() && c < ran.size(); ++c) if (ran[c])
        for (RigExecValueId id : graph.ops[c].descriptor.writes)
            if (state->values[size_t(id)].changed) state->changedValues.push_back(id);
}

} // namespace rigExec
#endif
