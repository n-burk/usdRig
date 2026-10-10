// Compiler and readiness-loop contracts shared by native and binary playback.
#include "rigExecGraph/opGraph.h"
#include "rigExecGraph/opValues.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/threadLimits.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace {
int failures = 0;
#define CHECK(value) do { if (!(value)) { ++failures; \
    std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value); } } while (0)

RigExecOpDescriptor Op(const char *key,
    std::vector<RigExecValueId> reads, std::vector<RigExecValueId> writes)
{
    RigExecOpDescriptor op;
    op.key = key; op.reads = std::move(reads); op.writes = std::move(writes);
    return op;
}

std::vector<std::string> Keys(const RigExecCompiledGraph &graph)
{
    std::vector<std::string> result;
    for (const auto &op : graph.ops) result.push_back(op.descriptor.key);
    return result;
}

void TestCanonicalOrder()
{
    const std::vector<RigExecOpDescriptor> source{
        Op("D.join", {2, 3}, {4}), Op("B.left", {1}, {2}),
        Op("C.right", {1}, {3}), Op("A.source", {0}, {1})};
    std::vector<unsigned> permutation{0, 1, 2, 3};
    size_t permutations = 0;
    do {
        std::vector<RigExecOpDescriptor> descriptors;
        for (unsigned i : permutation) descriptors.push_back(source[i]);
        RigExecCompiledGraph graph;
        CHECK(RigExecCompileOpGraph(descriptors, {0},
            RigExecCyclePolicy::Reject, &graph));
        CHECK(Keys(graph) == std::vector<std::string>(
            {"A.source", "B.left", "C.right", "D.join"}));
        CHECK(graph.longestPath == 3);
        CHECK(graph.ops[3].predecessors == std::vector<uint32_t>({1, 2}));
        for (size_t i = 0; i < graph.ops.size(); ++i) {
            CHECK(descriptors[graph.ops[i].originalIndex].key ==
                  graph.ops[i].descriptor.key);
            CHECK(graph.canonicalIndex[graph.ops[i].originalIndex] == int32_t(i));
        }
        ++permutations;
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    CHECK(permutations == 24);
}

void TestCompilerRefusals()
{
    RigExecCompiledGraph graph;
    std::string error;
    CHECK(!RigExecCompileOpGraph({Op("missing", {77}, {1})}, {},
        RigExecCyclePolicy::Reject, &graph, &error));
    CHECK(error.find("missing") != std::string::npos);
    CHECK(!RigExecCompileOpGraph({Op("first", {}, {1}), Op("second", {}, {1})},
        {}, RigExecCyclePolicy::Reject, &graph, &error));
    CHECK(error.find("first") != std::string::npos &&
          error.find("second") != std::string::npos);
    CHECK(!RigExecCompileOpGraph({Op("leafWriter", {}, {1})}, {1},
        RigExecCyclePolicy::Reject, &graph, &error));
    CHECK(!RigExecCompileOpGraph({Op("same", {}, {}), Op("same", {}, {})}, {},
        RigExecCyclePolicy::Reject, &graph, &error));
    auto bad = Op("badPredecessor", {}, {});
    bad.predecessors = {1};
    CHECK(!RigExecCompileOpGraph({bad}, {}, RigExecCyclePolicy::Reject,
        &graph, &error));
}

void TestCyclesAndSetAside()
{
    // A/B/C form one SCC; the reported loop must follow actual edges.
    // D is its downstream consumer and E is unrelated: neither is cyclic.
    const std::vector<RigExecOpDescriptor> source{
        Op("A", {3}, {1}), Op("B", {1}, {2}), Op("C", {2}, {3}),
        Op("D.consumer", {2}, {4}), Op("E.unrelated", {0}, {5})};
    RigExecCompiledGraph graph;
    std::string error;
    CHECK(!RigExecCompileOpGraph(source, {0}, RigExecCyclePolicy::Reject,
        &graph, &error));
    CHECK(error.find("A -> B -> C -> A") != std::string::npos);
    CHECK(RigExecCompileOpGraph(source, {0}, RigExecCyclePolicy::SetAside,
        &graph, &error));
    CHECK(Keys(graph) == std::vector<std::string>({"D.consumer", "E.unrelated"}));
    CHECK(graph.canonicalIndex == std::vector<int32_t>({-1, -1, -1, 0, 1}));
    CHECK(graph.cycles == std::vector<std::vector<std::string>>({{"A", "B", "C", "A"}}));
    CHECK(graph.cycleMembers == std::vector<std::vector<std::string>>({{"A", "B", "C"}}));
    CHECK(graph.ops[0].descriptor.reads == std::vector<RigExecValueId>({2}));
    CHECK(graph.ops[0].predecessors.empty());
    CHECK(RigExecCompileOpGraph({Op("self", {1}, {1})}, {},
        RigExecCyclePolicy::SetAside, &graph));
    CHECK(graph.ops.empty());
    CHECK(graph.cycles == std::vector<std::vector<std::string>>({{"self", "self"}}));
    CHECK(graph.cycleMembers == std::vector<std::vector<std::string>>({{"self"}}));

    // A short reported loop need not visit the whole SCC. C is still a cyclic
    // member; its downstream reader is not. A second SCC tests aligned sorting.
    const std::vector<RigExecOpDescriptor> branched{
        Op("Z", {6}, {6}), Op("C", {1}, {3}), Op("B", {1}, {2}),
        Op("A", {2, 3}, {1}), Op("tail", {3}, {4}), Op("independent", {0}, {5})};
    CHECK(RigExecCompileOpGraph(branched, {0}, RigExecCyclePolicy::SetAside,
        &graph));
    CHECK(graph.cycles == std::vector<std::vector<std::string>>(
        {{"A", "B", "A"}, {"Z", "Z"}}));
    CHECK(graph.cycleMembers == std::vector<std::vector<std::string>>(
        {{"A", "B", "C"}, {"Z"}}));
    CHECK(Keys(graph) == std::vector<std::string>({"independent", "tail"}));
    CHECK(graph.canonicalIndex == std::vector<int32_t>({-1, -1, -1, -1, 1, 0}));
    auto reversed = branched;
    std::reverse(reversed.begin(), reversed.end());
    RigExecCompiledGraph reordered;
    CHECK(RigExecCompileOpGraph(reversed, {0}, RigExecCyclePolicy::SetAside,
        &reordered));
    CHECK(reordered.cycles == graph.cycles);
    CHECK(reordered.cycleMembers == graph.cycleMembers);
    CHECK(Keys(reordered) == Keys(graph));

    // Real initial SCC authority survives removal of its commit writer.
    // Both the solver and a non-solver body must remain unavailable even
    // though the rebuilt descriptors would otherwise be acyclic.
    RigExecCompiledGraph original;
    CHECK(RigExecCompileOpGraph({Op("Solve",{2},{1}),Op("Field",{3},{2}),
        Op("Commit",{1},{3}),Op("Consumer",{2,0},{4})},{0},
        RigExecCyclePolicy::SetAside,&original,&error));
    RigExecOpExclusionProof proof;
    proof.keys={"Solve","Field"};proof.removedKeys={"Commit"};
    proof.cycles=original.cycles;proof.cycleMembers=original.cycleMembers;
    const std::vector<RigExecOpDescriptor> rebuilt{
        Op("Solve",{0},{1}),Op("Field",{1},{2}),Op("Consumer",{2,0},{4})};
    RigExecCompiledGraph normalized;
    CHECK(!RigExecCompileOpGraph(rebuilt,{0},RigExecCyclePolicy::SetAside,nullptr,nullptr));
    CHECK(RigExecCompileOpGraph(rebuilt,{0},RigExecCyclePolicy::SetAside,
        &proof,&normalized,&error));
    CHECK(Keys(normalized)==std::vector<std::string>({"Consumer"}));
    CHECK(normalized.cycles==original.cycles);
    CHECK(normalized.cycleMembers==original.cycleMembers);
    CHECK(normalized.canonicalIndex[0]<0 && normalized.canonicalIndex[1]<0);
    unsigned consumerRuns=0,consumerSkips=0;int published=0;
    std::vector<int> values(5,0);values[0]=17;
    std::vector<char> available(5,0);available[0]=1;
    RigExecOpCallbacks callbacks;
    callbacks.run=[&](uint32_t i){
        CHECK(normalized.ops[i].descriptor.key=="Consumer");
        ++consumerRuns;published=available[2]?values[2]:values[0];return true;
    };
    callbacks.skip=[&](uint32_t){++consumerSkips;};
    callbacks.changed=[](RigExecValueId){return false;};
    RigExecOpExecution execution;
    CHECK(RigExecExecuteOpGraph(normalized,{0},{},true,callbacks,&execution,&error));
    CHECK(consumerRuns==1 && published==17 && !available[1] && !available[2]);
    CHECK(consumerSkips==0 && execution.skipped==0 && execution.executed==1);
    const auto unchangedKeys=Keys(normalized);
    const auto unchangedMembers=normalized.cycleMembers;
    const auto refuse=[&](const auto &descriptors,const RigExecOpExclusionProof &bad) {
        error.clear();
        CHECK(!RigExecCompileOpGraph(descriptors,{0},RigExecCyclePolicy::SetAside,
            &bad,&normalized,&error));
        CHECK(!error.empty());
        CHECK(Keys(normalized)==unchangedKeys && normalized.cycleMembers==unchangedMembers);
    };
    auto bad=proof;bad.keys.push_back("Solve");refuse(rebuilt,bad);
    bad=proof;bad.keys.pop_back();refuse(rebuilt,bad); // Unmapped original body.
    bad=proof;bad.cycles.pop_back();refuse(rebuilt,bad);
    bad=proof;bad.cycles[0][1]="NotAnOriginalMember";refuse(rebuilt,bad);
    bad=proof;bad.removedKeys.push_back("Field");refuse(rebuilt,bad);
    bad=proof;bad.cycleMembers[0].push_back("Field");refuse(rebuilt,bad);
    auto missing=rebuilt;missing.erase(missing.begin()+1);missing.back().reads={0};
    refuse(missing,proof); // Exact persistent key missing after reconstruction.
    auto ambiguous=rebuilt;ambiguous.push_back(Op("Solve",{0},{5}));
    refuse(ambiguous,proof); // Duplicate stable identity cannot pick a winner.
    auto stillPresent=rebuilt;stillPresent.push_back(Op("Commit",{0},{3}));
    refuse(stillPresent,proof); // A claimed removed writer must actually be gone.
}

void TestDeclaredSemanticPredecessors()
{
    struct Range { uint32_t domain, begin, end; };
    struct Step {
        std::string key;
        uint32_t kind = 0;
        bool externalReads = false;
        std::vector<Range> reads, writes;
        std::vector<std::string> required;
        std::vector<uint32_t> preds; // An obsolete schedule is not authority.
    };
    std::vector<Step> steps(3);
    steps[0].key="A";steps[0].writes={{0,0,1}};steps[0].required={"B"};
    steps[1].key="B";steps[1].writes={{0,1,2}};steps[1].required={"A"};
    steps[2].key="unrelated";steps[2].writes={{0,2,3}};steps[2].preds={UINT32_MAX};
    RigExecCompiledGraph graph;RigExecOpAdapterState state;std::string error;
    const auto compile=[&](bool declared) {
        std::function<const std::vector<std::string> &(uint32_t)> requirements;
        if(declared)requirements=[&](uint32_t i)->const std::vector<std::string>&{return steps[i].required;};
        return RigExecOpCompileAdapter(steps,
            [](const Range &r){return r.domain;},[](const Range &r){return r.begin;},
            [](const Range &r){return r.end;},[](const Step &s){return s.key;},
            [](uint32_t,uint32_t){return false;},&graph,&state,&error,
            RigExecCyclePolicy::SetAside,requirements);
    };
    CHECK(compile(false));
    CHECK(Keys(graph)==std::vector<std::string>({"A","B","unrelated"}));
    CHECK(graph.cycles.empty());
    CHECK(compile(true));
    CHECK(Keys(graph)==std::vector<std::string>({"unrelated"}));
    CHECK(graph.cycleMembers==std::vector<std::vector<std::string>>({{"A","B"}}));
    CHECK(state.excludedValues.size()==2);
    // No numeric dependency was fabricated for a structural relationship.
    CHECK(graph.ops[0].descriptor.reads.empty());
    steps[0].required={"missing"};
    CHECK(!compile(true));
    CHECK(error.find("A requires missing")!=std::string::npos);
}

void TestCutoffAndRetention()
{
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph({Op("A", {0}, {1}), Op("B", {1}, {2}),
        Op("C", {2}, {3}), Op("U", {4}, {5})}, {0, 4},
        RigExecCyclePolicy::Reject, &graph));
    std::vector<char> changed(6, 0), ran(4, 0), skipped(4, 0);
    changed[0] = 1;
    RigExecOpCallbacks callbacks;
    callbacks.run = [&](uint32_t i) { ++ran[i]; return true; };
    callbacks.skip = [&](uint32_t i) { ++skipped[i]; };
    callbacks.changed = [&](RigExecValueId value) { return changed[value] != 0; };
    RigExecOpWorkspace workspace;
    RigExecOpExecution execution;
    CHECK(RigExecExecuteOpGraph(graph, {0}, {}, false, callbacks, &execution,
        nullptr, &workspace));
    CHECK(execution.executed == 1 && execution.skipped == 3);
    CHECK(execution.candidates == std::vector<char>({1, 1, 1, 0}));
    CHECK(ran == std::vector<char>({1, 0, 0, 0}));
    CHECK(skipped == std::vector<char>({0, 1, 1, 1}));
    CHECK(execution.completion[0] < execution.completion[1] &&
          execution.completion[1] < execution.completion[2]);
    CHECK(execution.completion[3] == 0);
    const auto *pendingStorage = workspace.pending.data();
    const auto *readyStorage = workspace.ready.data();
    const auto *unresolvedStorage = workspace.unresolved.data();
    const auto *candidateStorage = execution.candidates.data();
    changed.assign(6, 0); ran.assign(4, 0); skipped.assign(4, 0);
    callbacks.run = [&](uint32_t i) {
        ++ran[i]; for (auto value : graph.ops[i].descriptor.writes) changed[value] = 1;
        return true;
    };
    CHECK(RigExecExecuteOpGraph(graph, {}, {0}, false, callbacks, &execution,
        nullptr, &workspace));
    CHECK(execution.executed == 3 && execution.skipped == 1);
    CHECK(ran == std::vector<char>({1, 1, 1, 0}));
    CHECK(workspace.pending.data() == pendingStorage);
    CHECK(workspace.ready.data() == readyStorage);
    CHECK(workspace.unresolved.data() == unresolvedStorage);
    CHECK(execution.candidates.data() == candidateStorage);
    changed.assign(6, 0);
    CHECK(RigExecExecuteOpGraph(graph, {}, {}, false, callbacks, &execution,
        nullptr, &workspace));
    // Caller-reset change flags are required even on an idle generation.
    CHECK(execution.executed == 0 && execution.skipped == 4);
    CHECK(RigExecExecuteOpGraph(graph, {}, {}, true, callbacks, &execution,
        nullptr, &workspace));
    CHECK(execution.executed == 4 && execution.skipped == 0);
}

void TestReaderSpecificShadow()
{
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph({Op("A.shadowed", {0}, {1}),
        Op("B.shadowedChild", {1}, {2}), Op("C.visible", {0}, {3}),
        Op("D.visibleChild", {3}, {4})}, {0},
        RigExecCyclePolicy::Reject, &graph));
    std::vector<char> changed(5, 0), calls(4, 0), skips(4, 0);
    changed[0] = 1;
    RigExecOpCallbacks callbacks;
    callbacks.changed = [&](RigExecValueId value) { return changed[value] != 0; };
    callbacks.inputChanged = [&](uint32_t reader, RigExecValueId value) {
        return !(reader == 0 && value == 0) && changed[value] != 0;
    };
    callbacks.run = [&](uint32_t i) {
        ++calls[i];
        for (auto value : graph.ops[i].descriptor.writes) changed[value] = 1;
        return true;
    };
    callbacks.skip = [&](uint32_t i) { ++skips[i]; };
    RigExecOpExecution execution;
    CHECK(RigExecExecuteOpGraph(graph, {0}, {}, false, callbacks, &execution));
    CHECK(execution.candidates == std::vector<char>({1, 1, 1, 1}));
    CHECK(execution.ran == std::vector<char>({0, 0, 1, 1}));
    CHECK(calls == std::vector<char>({0, 0, 1, 1}));
    CHECK(skips == std::vector<char>({1, 1, 0, 0}));
    CHECK(execution.executed == 2 && execution.skipped == 2);
}

void TestCandidateEffectiveInputs()
{
    for (double grain : {0.0, 100.0}) for (bool parallel : {false, true}) {
        RigExecCompiledGraph graph;
        CHECK(RigExecCompileOpGraph({Op("A.clamp", {0}, {1}),
            Op("B.consumer", {1, 2}, {3}), Op("C.child", {3}, {4}),
            Op("U.unrelated", {5}, {6})}, {0, 2, 5},
            RigExecCyclePolicy::Reject, &graph));
        CHECK(RigExecLowerOpClusters(&graph, {}, grain));
        std::vector<int> values{11, 7, 22, 8, 9, 0, 1};
        std::vector<char> changed(7, 0);
        std::atomic<bool> producerDone{false}, readiness{true};
        int selectedMemo=7, produced=7;
        RigExecOpCallbacks callbacks;
        callbacks.changed=[&](RigExecValueId id) { return changed[id]!=0; };
        callbacks.inputsChanged=[&](uint32_t i) {
            if (i==1) {
                if (!producerDone.load(std::memory_order_acquire)) readiness.store(false);
                const bool different=selectedMemo!=values[1];
                selectedMemo=values[1];
                return different;
            }
            for (auto id:graph.ops[i].descriptor.reads) if (changed[id]) return true;
            return false;
        };
        callbacks.skip=[](uint32_t) {};
        callbacks.run=[&](uint32_t i) {
            if (i==0) {
                changed[1]=values[1]!=produced; values[1]=produced;
                producerDone.store(true,std::memory_order_release);
            } else {
                const auto input=graph.ops[i].descriptor.reads.front();
                const auto output=graph.ops[i].descriptor.writes.front();
                const int next=values[input]+1;
                changed[output]=values[output]!=next; values[output]=next;
            }
            return true;
        };
        WorkDispatcher dispatcher;
        if (parallel) {
            callbacks.dispatch=[&](std::function<void()> task) {dispatcher.Run(std::move(task));};
            callbacks.wait=[&] {dispatcher.Wait();};
        }
        RigExecOpExecution execution; RigExecOpWorkspace workspace;
        const std::vector<uint32_t> candidates{1};
        // Raw fallback 2 changes while the selected producer clamps to 7.
        changed[0]=changed[2]=1;
        CHECK(RigExecExecuteOpGraph(graph,{0,2},{},false,callbacks,&execution,
            nullptr,&workspace,&candidates));
        CHECK(readiness.load());
        CHECK(execution.ran==std::vector<char>({1,0,0,0}));
        CHECK(execution.executed==1 && values[3]==8 && values[4]==9);
        // A real selected value change reaches the consumer and its child.
        changed.assign(7,0); changed[0]=1; produced=8; producerDone.store(false);
        CHECK(RigExecExecuteOpGraph(graph,{0},{},false,callbacks,&execution,
            nullptr,&workspace,&candidates));
        CHECK(readiness.load());
        CHECK(execution.ran==std::vector<char>({1,1,1,0}));
        CHECK(values[3]==9 && values[4]==10);
        // Mandatory execution survives an unchanged effective-input predicate.
        changed.assign(7,0); producerDone.store(true);
        CHECK(RigExecExecuteOpGraph(graph,{}, {1},false,callbacks,&execution,
            nullptr,&workspace,&candidates));
        CHECK(execution.ran==std::vector<char>({0,1,0,0}));
        const std::vector<uint32_t> invalid{uint32_t(graph.ops.size())};
        std::string error;
        CHECK(!RigExecExecuteOpGraph(graph,{}, {},false,callbacks,&execution,
            &error,&workspace,&invalid));
        CHECK(error=="operation candidate out of range");
    }
}

struct RunResult {
    RigExecOpExecution execution;
    std::vector<int> values;
    bool valid = false, readiness = false;
};

RunResult EvaluateDiamond(const RigExecCompiledGraph &graph, bool parallel)
{
    std::atomic<bool> changed[5], done[4];
    for (auto &v : changed) v.store(false);
    for (auto &v : done) v.store(false);
    changed[0].store(true);
    RunResult result; result.values.assign(5, 0); result.values[0] = 7;
    std::atomic<bool> readiness{true};
    RigExecOpCallbacks callbacks;
    callbacks.run = [&](uint32_t i) {
        for (auto predecessor : graph.ops[i].predecessors)
            if (!done[predecessor].load(std::memory_order_acquire)) readiness.store(false);
        int value = 1;
        for (auto read : graph.ops[i].descriptor.reads) value += result.values[read];
        for (auto write : graph.ops[i].descriptor.writes) {
            result.values[write] = value;
            changed[write].store(true, std::memory_order_release);
        }
        done[i].store(true, std::memory_order_release); return true;
    };
    callbacks.skip = [](uint32_t) {};
    callbacks.changed = [&](RigExecValueId value) {
        return changed[value].load(std::memory_order_acquire);
    };
    WorkDispatcher dispatcher;
    if (parallel) {
        // Execute submits all roots before wait; descendants submit only
        // inside those tasks, as required by WorkDispatcher::Run/Wait.
        callbacks.dispatch = [&](std::function<void()> task) { dispatcher.Run(std::move(task)); };
        callbacks.wait = [&] { dispatcher.Wait(); };
    }
    RigExecOpWorkspace workspace;
    result.valid = RigExecExecuteOpGraph(graph, {0}, {}, false, callbacks,
        &result.execution, nullptr, &workspace);
    result.readiness = readiness.load();
    return result;
}

void TestSerialParallelReadiness()
{
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph({Op("A", {0}, {1}), Op("B", {1}, {2}),
        Op("C", {1}, {3}), Op("D", {2, 3}, {4})}, {0},
        RigExecCyclePolicy::Reject, &graph));
    const auto serial = EvaluateDiamond(graph, false);
    const auto parallel = EvaluateDiamond(graph, true);
    CHECK(serial.valid && parallel.valid && serial.readiness && parallel.readiness);
    CHECK(serial.values == std::vector<int>({7, 8, 9, 9, 19}));
    CHECK(parallel.values == serial.values);
    CHECK(parallel.execution.candidates == serial.execution.candidates);
    CHECK(parallel.execution.ran == serial.execution.ran);
    CHECK(parallel.execution.executed == 4 && parallel.execution.skipped == 0);
    for (uint32_t i = 0; i < graph.ops.size(); ++i)
        for (uint32_t predecessor : graph.ops[i].predecessors)
            CHECK(parallel.execution.completion[predecessor] < parallel.execution.completion[i]);
}

void TestUnrelatedBranchFinishesBeforeGate()
{
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph({Op("A.slow", {0}, {1}),
        Op("B.slowChild", {1}, {2}), Op("C.fast", {3}, {4}),
        Op("D.fastChild", {4}, {5}), Op("E.fastObserver", {5}, {6})}, {0, 3},
        RigExecCyclePolicy::Reject, &graph));
    std::atomic<bool> slowStarted{false}, releaseSlow{false}, fastFinished{false};
    std::atomic<bool> gateTimedOut{false};
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    const auto await=[&](const auto &ready) {
        while(!ready()) {
            if(std::chrono::steady_clock::now()>=deadline)return false;
            std::this_thread::yield();
        }
        return true;
    };
    std::atomic<bool> changed[7];
    for (auto &value : changed) value.store(false);
    changed[0].store(true); changed[3].store(true);
    RigExecOpExecution execution;
    bool valid = false;
    std::thread runner([&] {
        WorkDispatcher dispatcher;
        RigExecOpCallbacks callbacks;
        callbacks.run = [&](uint32_t i) {
            if (graph.ops[i].descriptor.key == "A.slow") {
                slowStarted.store(true,std::memory_order_release);
                if(!await([&] {return releaseSlow.load(std::memory_order_acquire);}))
                    gateTimedOut.store(true,std::memory_order_release);
            }
            for (auto value : graph.ops[i].descriptor.writes)
                changed[value].store(true, std::memory_order_release);
            if (graph.ops[i].descriptor.key == "E.fastObserver")
                fastFinished.store(true,std::memory_order_release);
            return true;
        };
        callbacks.skip = [](uint32_t) {};
        callbacks.changed = [&](RigExecValueId value) {
            return changed[value].load(std::memory_order_acquire);
        };
        callbacks.dispatch = [&](std::function<void()> task) { dispatcher.Run(std::move(task)); };
        callbacks.wait = [&] { dispatcher.Wait(); };
        valid = RigExecExecuteOpGraph(graph, {0, 3}, {}, false, callbacks, &execution);
    });
    const bool completedBeforeRelease=await([&] {
        return slowStarted.load(std::memory_order_acquire) &&
            fastFinished.load(std::memory_order_acquire);
    });
    releaseSlow.store(true,std::memory_order_release);
    runner.join();
    CHECK(completedBeforeRelease && !gateTimedOut.load());
    CHECK(valid && execution.executed == 5 && execution.skipped == 0);
    // This is an actual dependency crossing, not merely an independent root.
    CHECK(execution.completion[2] < execution.completion[3]);
    CHECK(execution.completion[0] < execution.completion[1]);
    CHECK(execution.completion[3] < execution.completion[0]);
}
void TestFatalCancellationJoinsActiveCallbacks()
{
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph({Op("A.active",{0},{1}),
        Op("B.fatal",{0},{2}),Op("C.afterActive",{1},{3}),
        Op("D.afterFatal",{2},{4}),Op("E.join",{3,4},{5})}, {0},
        RigExecCyclePolicy::Reject,&graph));
    std::atomic<bool> activeStarted{false}, releaseActive{false}, activeFinished{false};
    std::atomic<bool> gateTimedOut{false}, unexpectedBody{false};
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    const auto await=[&](const auto &ready) {
        while(!ready()) {
            if(std::chrono::steady_clock::now()>=deadline)return false;
            std::this_thread::yield();
        }
        return true;
    };
    const auto owner=std::this_thread::get_id();
    // Each queue has one writer. The owner reads the active callback's queue
    // only after join establishes completion of all its submissions.
    std::vector<std::function<void()>> queued, activeQueued;
    std::thread active; bool firstDispatch=true;
    RigExecOpCallbacks callbacks;
    std::atomic<int> bodies{0}, skipped{0};
    callbacks.run=[&](uint32_t i) {
        ++bodies;
        if(graph.ops[i].descriptor.key=="A.active") {
            activeStarted.store(true,std::memory_order_release);
            if(!await([&] {return releaseActive.load(std::memory_order_acquire);}))
                gateTimedOut.store(true,std::memory_order_release);
            activeFinished.store(true,std::memory_order_release);
            return true;
        }
        if(graph.ops[i].descriptor.key!="B.fatal")unexpectedBody.store(true);
        if(!await([&] {return activeStarted.load(std::memory_order_acquire);}))
            gateTimedOut.store(true,std::memory_order_release);
        return false;
    };
    callbacks.skip=[&](uint32_t) { ++skipped; };
    callbacks.changed=[](RigExecValueId) { return true; };
    callbacks.dispatch=[&](std::function<void()> task) {
        if(std::this_thread::get_id()!=owner)activeQueued.push_back(std::move(task));
        else if(firstDispatch) { firstDispatch=false; active=std::thread(std::move(task)); }
        else queued.push_back(std::move(task));
    };
    const auto drain=[&] {
        for(size_t next=0;next<queued.size();++next) {
            auto task=std::move(queued[next]);
            task();
        }
        queued.clear();
    };
    callbacks.wait=[&] {
        // The fatal wrapper has returned, so cancellation is published before
        // the already active body is released. Its descendants enqueue later.
        drain();
        releaseActive.store(true,std::memory_order_release);
        active.join();
        for(auto &task:activeQueued)queued.push_back(std::move(task));
        activeQueued.clear();
        drain();
    };
    RigExecOpExecution execution; std::string error;
    CHECK(!RigExecExecuteOpGraph(graph,{0},{},false,callbacks,&execution,&error));
    CHECK(activeFinished.load() && !gateTimedOut.load() && !unexpectedBody.load());
    CHECK(bodies.load()==2 && skipped.load()==3);
    CHECK(execution.executed==2 && execution.skipped==3 && !error.empty());
    for(uint32_t i=0;i<graph.ops.size();++i) {
        CHECK(execution.completion[i]!=0);
        for(auto predecessor:graph.ops[i].predecessors)
            CHECK(execution.completion[predecessor]<execution.completion[i]);
    }
}

/// Multi-member clusters number their candidate members consecutively after
/// the cluster's last body; counts come from ran. Serial numbering is the
/// order the bodies ran, which binary playback's run trace relies on.
void TestClusterCompletionAndCounts()
{
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph({Op("A0",{0},{1}),Op("A1",{1},{2}),Op("A2",{2},{3}),
        Op("B0",{10},{11}),Op("B1",{11},{12}),Op("J",{3,12},{20}),Op("U",{30},{31})},
        {0,10,30},RigExecCyclePolicy::Reject,&graph));
    CHECK(RigExecLowerOpClusters(&graph,{},100.0));
    CHECK(Keys(graph)==std::vector<std::string>({"A0","A1","A2","B0","B1","J","U"}));
    CHECK(graph.clusters.size()==4);
    CHECK(graph.opClusters==std::vector<uint32_t>({0,0,0,1,1,2,3}));
    for (bool parallel : {false, true}) {
        std::vector<std::atomic<bool>> changed(32);
        for (auto &value:changed) value.store(false);
        std::vector<uint32_t> order; // Serial runs only.
        std::atomic<int> skips{0};
        RigExecOpCallbacks callbacks;
        bool republish=true;
        callbacks.run=[&](uint32_t i) {
            if (!parallel) order.push_back(i);
            if (republish) for (auto write:graph.ops[i].descriptor.writes)
                changed[write].store(true,std::memory_order_release);
            return true;
        };
        callbacks.skip=[&](uint32_t) { ++skips; };
        callbacks.changed=[&](RigExecValueId id) { return changed[id].load(std::memory_order_acquire); };
        WorkDispatcher dispatcher;
        if (parallel) {
            callbacks.dispatch=[&](std::function<void()> task) {dispatcher.Run(std::move(task));};
            callbacks.wait=[&] {dispatcher.Wait();};
        }
        RigExecOpExecution execution; RigExecOpWorkspace workspace;
        changed[0].store(true); changed[10].store(true);
        CHECK(RigExecExecuteOpGraph(graph,{0,10},{},false,callbacks,&execution,nullptr,&workspace));
        CHECK(execution.executed==6 && execution.skipped==1 && skips.load()==1);
        CHECK(execution.ran==std::vector<char>({1,1,1,1,1,1,0}));
        CHECK(execution.completion[6]==0);
        // Members of one cluster are consecutive; every edge points forward.
        CHECK(execution.completion[1]==execution.completion[0]+1 &&
              execution.completion[2]==execution.completion[1]+1);
        CHECK(execution.completion[4]==execution.completion[3]+1);
        for (uint32_t i=0;i<graph.ops.size();++i)
            for (uint32_t predecessor:graph.ops[i].predecessors)
                if (execution.candidates[i])
                    CHECK(execution.completion[predecessor]<execution.completion[i]);
        std::vector<uint64_t> numbers;
        for (uint32_t i=0;i<graph.ops.size();++i)
            if (execution.candidates[i]) numbers.push_back(execution.completion[i]);
        std::sort(numbers.begin(),numbers.end());
        CHECK(numbers==std::vector<uint64_t>({1,2,3,4,5,6}));
        if (!parallel) {
            CHECK(order==std::vector<uint32_t>({0,1,2,3,4,5}));
            for (size_t k=0;k<order.size();++k) CHECK(execution.completion[order[k]]==k+1);
        }
        // B1 is a candidate whose input did not change: it and J skip in
        // their bodies and are still numbered; A and U are not candidates.
        for (auto &value:changed) value.store(false);
        changed[10].store(true); republish=false; skips=0; order.clear();
        CHECK(RigExecExecuteOpGraph(graph,{10},{},false,callbacks,&execution,nullptr,&workspace));
        CHECK(execution.candidates==std::vector<char>({0,0,0,1,1,1,0}));
        CHECK(execution.ran==std::vector<char>({0,0,0,1,0,0,0}));
        CHECK(execution.executed==1 && execution.skipped==6 && skips.load()==6);
        CHECK(execution.completion==std::vector<uint64_t>({0,0,0,1,2,3,0}));
    }
}

/// Volatile ops are compiled into a list and seeded without a changed input;
/// a skip-effect list limits skip to the listed non-candidates while every
/// candidate that does not run is still skipped.
void TestSkipEffectsAndVolatileSeeds()
{
    auto volatileOp=Op("V",{40},{41}); volatileOp.volatileInput=true;
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph({Op("A",{0},{1}),Op("B",{1},{2}),volatileOp,
        Op("W",{41},{42}),Op("U",{50},{51})},{0,40,50},RigExecCyclePolicy::Reject,&graph));
    CHECK(Keys(graph)==std::vector<std::string>({"A","B","U","V","W"}));
    CHECK(graph.volatileOps==std::vector<uint32_t>({3}));
    std::vector<char> changed(64,0);
    std::vector<uint32_t> ran, skipped;
    RigExecOpCallbacks callbacks;
    callbacks.run=[&](uint32_t i) {
        ran.push_back(i);
        if (graph.ops[i].descriptor.key!="A")
            for (auto write:graph.ops[i].descriptor.writes) changed[write]=1;
        return true;
    };
    callbacks.skip=[&](uint32_t i) { skipped.push_back(i); };
    callbacks.changed=[&](RigExecValueId id) { return changed[id]!=0; };
    RigExecOpExecution execution;
    CHECK(RigExecExecuteOpGraph(graph,{},{},false,callbacks,&execution));
    CHECK(ran==std::vector<uint32_t>({3,4}));
    CHECK(skipped==std::vector<uint32_t>({0,1,2}));
    const std::vector<uint32_t> effects{1,2};
    callbacks.skipEffects=&effects;
    ran.clear(); skipped.clear(); changed.assign(64,0);
    CHECK(RigExecExecuteOpGraph(graph,{},{},false,callbacks,&execution));
    CHECK(ran==std::vector<uint32_t>({3,4}));
    CHECK(skipped==std::vector<uint32_t>({1,2}));
    CHECK(execution.executed==2 && execution.skipped==3);
    // A and B are candidates; A republishes nothing, so B skips in its body
    // although the list does not name it.
    const std::vector<uint32_t> onlyU{2};
    callbacks.skipEffects=&onlyU;
    ran.clear(); skipped.clear(); changed.assign(64,0); changed[0]=1;
    CHECK(RigExecExecuteOpGraph(graph,{0},{},false,callbacks,&execution));
    CHECK(ran==std::vector<uint32_t>({0,3,4}));
    CHECK(skipped==std::vector<uint32_t>({2,1}));
    // Forced runs seed every op, volatile or not, in the serial FIFO order.
    ran.clear(); skipped.clear(); changed.assign(64,0);
    CHECK(RigExecExecuteOpGraph(graph,{},{},true,callbacks,&execution));
    CHECK(ran==std::vector<uint32_t>({0,2,3,1,4}) && skipped.empty());
    const std::vector<uint32_t> invalid{uint32_t(graph.ops.size())};
    callbacks.skipEffects=&invalid;
    std::string error;
    CHECK(!RigExecExecuteOpGraph(graph,{},{},false,callbacks,&execution,&error));
    CHECK(error=="skip effect operation out of range");
}

/// The flags one run sets are exactly the ones the next run clears.
void TestChangeFlagsClearedFromGatheredList()
{
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph({Op("A",{0},{1}),Op("B",{1},{2}),Op("C",{3},{4}),
        Op("D",{5},{6})},{0,3,5},RigExecCyclePolicy::Reject,&graph));
    RigExecOpAdapterState state;
    for (uint32_t slot=0;slot<7;++slot) RigExecOpAddValue(&state,0,slot);
    state.leaves={0,3,5};
    std::vector<std::string> source(7,"x");
    const auto sample=[&](uint32_t,uint32_t slot,std::string *key) { key->append(source[slot]); };
    RigExecOpCallbacks callbacks;
    callbacks.run=[&](uint32_t i) {
        for (auto id:graph.ops[i].descriptor.writes) RigExecOpPublishValue(&state.values[id],sample);
        return true;
    };
    callbacks.skip=[](uint32_t) {};
    callbacks.changed=[&](RigExecValueId id) { return state.values[id].changed!=0; };
    RigExecOpExecution execution;
    const auto frame=[&] {
        state.changedLeaves.clear();
        RigExecOpClearChanges(&state);
        for (auto &value:state.values) CHECK(!value.changed);
        for (auto id:state.leaves) {
            RigExecOpPublishValue(&state.values[id],sample);
            if (state.values[id].changed) state.changedLeaves.push_back(id);
        }
        CHECK(RigExecExecuteOpGraph(graph,state.changedLeaves,{},false,callbacks,&execution));
        RigExecOpGatherChanges(&state,graph,execution.ran);
        for (RigExecValueId id=0;id<state.values.size();++id)
            CHECK(!state.values[id].changed ||
                  std::find(state.changedValues.begin(),state.changedValues.end(),id)!=
                      state.changedValues.end());
        // Visiting only the executor's record gathers the same list.
        const std::vector<RigExecValueId> swept=state.changedValues;
        CHECK(execution.touchedValid);
        RigExecOpGatherChanges(&state,graph,execution.ran,&execution.touched);
        CHECK(state.changedValues==swept);
    };
    frame(); // First publication: every value is new.
    CHECK(state.changedValues.size()==7);
    source[0]="y"; source[1]="y";
    frame(); // A and B rerun; A's leaf and output move, B's output does not.
    CHECK(execution.ran==std::vector<char>({1,1,0,0}));
    CHECK(state.changedValues==std::vector<RigExecValueId>({0,1}));
    frame(); // Idle: the last frame's flags are gone.
    CHECK(state.changedValues.empty());
}

/// Forty ops in four chains, C joining A and B, lowered three clusters a
/// chain. One execution and workspace, reset through their own records,
/// give a fresh pair's answer after every run, across whole-struct
/// restores; an execution without a valid record is reset whole; a
/// list-driven reset leaves a byte no record names, and the reset judge
/// fails the run on it.
void TestSparseResetMatchesFreshWorkspace()
{
    std::vector<RigExecOpDescriptor> descriptors;
    const char chains[] = "ABCD";
    for (int k = 0; k < 4; ++k)
        for (int i = 0; i < 10; ++i) {
            const RigExecValueId base = RigExecValueId(100 * (k + 1));
            std::vector<RigExecValueId> reads{base + RigExecValueId(i)};
            if (k == 2 && i == 0) reads = {RigExecValueId(110), RigExecValueId(210)};
            const std::string key = std::string(1, chains[k]) + char('0' + i);
            descriptors.push_back(Op(key.c_str(), reads, {base + RigExecValueId(i + 1)}));
        }
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph(descriptors, {100, 200, 400},
        RigExecCyclePolicy::Reject, &graph));
    CHECK(RigExecLowerOpClusters(&graph, {}, 4.0));
    // Canonical order A0..A9, B0..B9, C0..C9, D0..D9; clusters of 4, 4, 2.
    CHECK(graph.ops.size() == 40 && graph.clusters.size() == 12);
    if (graph.ops.size() != 40 || graph.clusters.size() != 12) return;
    CHECK(graph.ops[20].descriptor.key == "C0" && graph.opClusters[17] == 4 &&
          graph.opClusters[30] == 9 && graph.opClusters[39] == 11);
    for (bool parallel : {false, true}) {
        std::vector<std::atomic<bool>> changed(512);
        RigExecOpCallbacks callbacks;
        // Every seventh op from the fourth keeps a clean output, which
        // stops the wave there.
        callbacks.run = [&](uint32_t i) {
            if (i % 7 != 3)
                for (auto write : graph.ops[i].descriptor.writes)
                    changed[size_t(write)].store(true, std::memory_order_release);
            return true;
        };
        callbacks.skip = [](uint32_t) {};
        callbacks.changed = [&](RigExecValueId id) {
            return changed[size_t(id)].load(std::memory_order_acquire);
        };
        WorkDispatcher dispatcher;
        if (parallel) {
            callbacks.dispatch = [&](std::function<void()> task) { dispatcher.Run(std::move(task)); };
            callbacks.wait = [&] { dispatcher.Wait(); };
        }
        std::string error;
        const auto execute = [&](RigExecOpExecution *execution,
                                 RigExecOpWorkspace *workspace,
                                 const std::vector<uint32_t> &seeds, bool force) {
            for (auto &value : changed) value.store(false);
            error.clear();
            return RigExecExecuteOpGraph(graph, {}, seeds, force, callbacks,
                                         execution, &error, workspace);
        };
        // \p kept against a fresh execution and workspace given the same run.
        const auto same = [&](const RigExecOpExecution &kept,
                              const std::vector<uint32_t> &seeds, bool force,
                              const char *what) {
            RigExecOpExecution fresh;
            RigExecOpWorkspace freshWorkspace;
            CHECK(execute(&fresh, &freshWorkspace, seeds, force));
            std::vector<uint32_t> ascending;
            for (uint32_t i = 0; i < fresh.candidates.size(); ++i)
                if (fresh.candidates[i]) ascending.push_back(i);
            bool equal = kept.candidates == fresh.candidates &&
                kept.ran == fresh.ran && kept.executed == fresh.executed &&
                kept.skipped == fresh.skipped && kept.touchedValid &&
                kept.touched == ascending && fresh.touched == ascending &&
                kept.completion.size() == fresh.completion.size();
            // A parallel run numbers in finish order; only its support is fixed.
            for (size_t i = 0; equal && i < kept.completion.size(); ++i)
                equal = (kept.completion[i] != 0) == (kept.candidates[i] != 0);
            if (!parallel) equal = equal && kept.completion == fresh.completion;
            if (!equal)
                std::printf("    %s run %s: the reused pair differs from a fresh one\n",
                            parallel ? "parallel" : "serial", what);
            CHECK(equal);
        };
        const std::vector<uint32_t> seedsA{0}, seedsB{17, 30}, seedsD{5}, none;
        RigExecOpExecution execution, afterB;
        RigExecOpWorkspace workspace;
        CHECK(execute(&execution, &workspace, seedsA, false));
        CHECK(execution.executed == 4 && execution.touched.size() == 20);
        same(execution, seedsA, false, "A");
        CHECK(execute(&execution, &workspace, seedsB, false));
        same(execution, seedsB, false, "B");
        afterB = execution;
        CHECK(execute(&execution, &workspace, none, true));
        CHECK(execution.executed == 40);
        same(execution, none, true, "C");
        // Put back as the verify restores do: the record comes with the bytes.
        execution = afterB;
        CHECK(execute(&execution, &workspace, seedsD, false));
        CHECK(execution.executed == 10 && execution.touched.size() == 15);
        same(execution, seedsD, false, "D");
        // A copy taken after A, put back after B: only the copy's own record
        // names A0..A4, which D does not reach.
        RigExecOpExecution second, afterA;
        RigExecOpWorkspace secondWorkspace;
        CHECK(execute(&second, &secondWorkspace, seedsA, false));
        afterA = second;
        CHECK(execute(&second, &secondWorkspace, seedsB, false));
        second = afterA;
        CHECK(execute(&second, &secondWorkspace, seedsD, false));
        same(second, seedsD, false, "D after A put back");
        // Another execution on the same workspace, every byte set and no
        // valid record: reset whole.
        RigExecOpExecution poisoned;
        poisoned.candidates.assign(40, 1);
        poisoned.ran.assign(40, 1);
        poisoned.completion.assign(40, 9);
        poisoned.touched = {0};
        poisoned.touchedValid = false;
        CHECK(execute(&poisoned, &workspace, seedsD, false));
        same(poisoned, seedsD, false, "D on a poisoned execution");
        // The reset follows the record: a byte it does not name stays.
        CHECK(execute(&execution, &workspace, seedsD, false));
        execution.ran[0] = 1;
        CHECK(execute(&execution, &workspace, seedsD, false));
        CHECK(execution.ran[0] == 1 && execution.executed == 10);
        // The judge finds such a byte in the execution or the workspace,
        // zeroes it and fails the run.
        workspace.verifyReset = true;
        CHECK(!execute(&execution, &workspace, seedsD, false));
        CHECK(error == "operation execution reset missed an entry");
        CHECK(execution.ran[0] == 0);
        CHECK(execute(&execution, &workspace, seedsD, false));
        same(execution, seedsD, false, "D after the judge");
        workspace.clusterCandidates[11] = 1;
        CHECK(!execute(&execution, &workspace, seedsD, false));
        CHECK(error == "operation execution reset missed an entry");
        workspace.seeded[35] = 1;
        CHECK(!execute(&execution, &workspace, seedsD, false));
        CHECK(error == "operation execution reset missed an entry");
        CHECK(execute(&execution, &workspace, seedsD, false));
        same(execution, seedsD, false, "D after the judged workspace");
    }
}

/// The sorted and the scanned forms give one ascending, unique sequence.
void TestAscendingListsAgree()
{
    const size_t n = 1000;
    std::vector<char> flags(n, 0);
    const auto scan = [&](size_t size) {
        std::vector<uint32_t> result;
        for (uint32_t i = 0; i < size && i < flags.size(); ++i)
            if (flags[i]) result.push_back(i);
        return result;
    };
    // Ten entries, two repeated: 10 * 16 < 1000 sorts.
    const std::vector<uint32_t> sparse{913, 4, 77, 4, 500, 999, 0, 77, 250, 13};
    for (uint32_t i : sparse) flags[i] = 1;
    std::vector<uint32_t> out{42, 7};
    RigExecOpAscending(sparse, flags, n, &out);
    CHECK(out == scan(n));
    CHECK(out == std::vector<uint32_t>({0, 4, 13, 77, 250, 500, 913, 999}));
    // 450 entries, each twice and scrambled: 900 * 16 >= 1000 scans.
    std::fill(flags.begin(), flags.end(), char(0));
    std::vector<uint32_t> dense;
    for (uint32_t k = 0; k < 450; ++k) {
        const uint32_t i = (k * 919u) % 1000u;
        dense.push_back(i); dense.push_back(i);
        flags[i] = 1;
    }
    const std::vector<uint32_t> expected = scan(n);
    CHECK(dense.size() == 900 && expected.size() == 450);
    RigExecOpAscending(dense, flags, n, &out);
    CHECK(out == expected);
    // The same list against a larger n sorts: 900 * 16 < 20000.
    flags.resize(20000, char(0));
    RigExecOpAscending(dense, flags, flags.size(), &out);
    CHECK(out == expected);
}

} // namespace

int main()
{
    WorkSetConcurrencyLimit(4);
    TestCanonicalOrder(); TestCompilerRefusals(); TestCyclesAndSetAside();
    TestDeclaredSemanticPredecessors();
    TestCutoffAndRetention(); TestReaderSpecificShadow(); TestCandidateEffectiveInputs(); TestSerialParallelReadiness();
    TestUnrelatedBranchFinishesBeforeGate(); TestFatalCancellationJoinsActiveCallbacks();
    TestClusterCompletionAndCounts(); TestSkipEffectsAndVolatileSeeds();
    TestChangeFlagsClearedFromGatheredList();
    TestSparseResetMatchesFreshWorkspace(); TestAscendingListsAgree();
    std::printf("OpGraph: %d failures\n", failures);
    return failures ? 1 : 0;
}
