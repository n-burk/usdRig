//
// RigExec sparse cross-frame reuse, Stream D: the output-affected index, the
// task-list cache, retained-frame planning, candidate lookup, capture-index
// epoch invalidation, and executed-cluster counts.
//
// Style follows testRigExecFrameCache.cpp: plain C++ with main + CHECK over
// synthetic programs and in-memory stages. The synthetic programs below are
// hand-built clusterings closed by the REAL RigExecBakedBuildCones -- the
// closures under test are never hand-drawn -- and the capture-index test
// builds a REAL baked program over a REAL in-memory rig and routes REAL
// stage notices through it.
//

#include "rigExec/frameCacheSparsity.h"
#include "rigExec/bakedSchedule.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/base/tf/weakPtr.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

// A pose whose bytes do not move with the tag, as in testRigExecFrameCache.
RigExecRigPose
MakePose(double tag, size_t joints)
{
    RigExecRigPose pose;
    pose.valid = true;
    for (size_t i = 0; i < joints; ++i) {
        RigExecPointFrame frame;
        frame.points[0] = GfVec3d(tag, double(i), 0.0);
        pose.jointFramesFinal[SdfPath("/Joint" + std::to_string(i))] = frame;
    }
    return pose;
}

RigExecFrameInputs
MakeInputs(UsdTimeCode time,
           std::initializer_list<std::pair<const char *, double>> samples)
{
    RigExecFrameInputs inputs;
    inputs.time = time;
    for (const auto &s : samples) {
        inputs.Add(SdfPath(s.first), VtValue(s.second));
    }
    return inputs;
}

// Diamond plus tail plus isolate, closed by the real BuildCones:
//
//   0 -> 1 -> 3 -> 4
//   0 -> 2 -> 3
//   5 (no edges)
//
// cone[0]={0,1,2,3,4} cone[1]={1,3,4} cone[2]={2,3,4} cone[3]={3,4}
// cone[4]={4} cone[5]={5}
//
// With \p alwaysStep, one step reading outside the graph sits in cluster 4,
// so the always-dirty set is {4}.
RigExecBakedProgramImpl
MakeDiamond(bool alwaysStep)
{
    RigExecBakedProgramImpl program{};
    program.clustering.clusters.resize(6);
    const auto edge = [&program](int from, int to) {
        program.clustering.clusters[size_t(from)].succs.push_back(to);
        program.clustering.clusters[size_t(to)].preds.push_back(from);
    };
    edge(0, 1);
    edge(0, 2);
    edge(1, 3);
    edge(2, 3);
    edge(3, 4);
    if (alwaysStep) {
        // Derived is inert in BuildCones' table lookups (empty reads hit no
        // table, and its kind names no table) and is the kind that reads
        // outside the graph unconditionally -- the loop resets every other
        // kind's authored externalReads.
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::Derived;
        step.object = -1;
        step.cluster = 4;
        program.steps.push_back(step);
    }
    RigExecBakedBuildCones(&program);
    return program;
}

bool
SetIs(const RigExecBakedClusterSet &set, std::initializer_list<int> members)
{
    // Membership, not words: an empty index answers a wordless set, which
    // is still the empty set.
    if (set.Count() != members.size()) {
        return false;
    }
    for (int c : members) {
        const size_t word = size_t(c) >> 6;
        if (c < 0 || word >= set.words.size() ||
            ((set.words[word] >> (size_t(c) & 63)) & 1) == 0) {
            return false;
        }
    }
    return true;
}

RigExecRetainedFrameState
MakeRetained(uint64_t epoch, size_t clusters, UsdTimeCode time,
             std::initializer_list<std::pair<const char *, double>> samples)
{
    RigExecRetainedFrameState state;
    state.inputs = MakeInputs(time, samples);
    state.epochDigest = epoch;
    state.clusterCount = clusters;
    return state;
}

}  // namespace

// The D2 decision answers from the Stream 0 numbers: GO at ~80 full biped
// frames under the default cap, no-go when the cap cannot hold a useful
// neighborhood, and never GO without a measurement.
void
TestGoNoGo()
{
    const RigExecSparsityDecision stream0 =
        RigExecStream0SparsityDecision();
    CHECK(stream0.go);
    CHECK(stream0.fullFramesAtCap >= 80);
    CHECK(!stream0.memo.empty());

    // The 9mesh numbers go too, with room to spare.
    const RigExecSparsityDecision mesh9 = RigExecDecideSparsity(
        kRigExecSparsityStream0Mesh9ArenaBytes,
        kRigExecSparsityStream0Mesh9PoseBytes,
        kRigExecFrameCacheDefaultByteCap);
    CHECK(mesh9.go);
    CHECK(mesh9.fullFramesAtCap >= 100);

    // A cap that cannot hold the neighborhood is whole-pose memo only.
    const RigExecSparsityDecision small = RigExecDecideSparsity(
        kRigExecSparsityStream0BipedArenaBytes,
        kRigExecSparsityStream0BipedPoseBytes, 1024 * 1024);
    CHECK(!small.go);
    CHECK(small.fullFramesAtCap == 0);
    CHECK(!small.memo.empty());

    // No measurement, no GO.
    CHECK(!RigExecDecideSparsity(0, 0, 1024).go);
    CHECK(!RigExecDecideSparsity(100, 100, 0).go);
}

// The index closes over the baked cones without re-deriving them: the
// closures it answers are the ones BuildCones computed.
void
TestOutputAffectedIndex()
{
    const RigExecBakedProgramImpl program = MakeDiamond(false);
    RigExecOutputAffectedIndex index;
    CHECK(index.Empty());
    index.Build(program, 7);
    CHECK(!index.Empty());
    CHECK(index.ClusterCount() == 6);
    CHECK(index.EpochDigest() == 7);
    CHECK(SetIs(index.ConeOf(0), {0, 1, 2, 3, 4}));
    CHECK(SetIs(index.ConeOf(1), {1, 3, 4}));
    CHECK(SetIs(index.ConeOf(2), {2, 3, 4}));
    CHECK(SetIs(index.ConeOf(3), {3, 4}));
    CHECK(SetIs(index.ConeOf(4), {4}));
    CHECK(SetIs(index.ConeOf(5), {5}));
    CHECK(index.ConeOf(6).Count() == 0);
    CHECK(index.Always().Count() == 0);
    CHECK(index.AllClusters().Count() == 6);

    index.MapControl("/Ctl/A", {1});
    index.MapControl("/Ctl/B", {2});
    CHECK(index.SeedsForControl("/Ctl/A") == std::vector<int>{1});
    CHECK(index.SeedsForControl("/Ctl/Nope").empty());

    const size_t walks = index.Walks();
    CHECK(SetIs(index.AffectedByControls({"/Ctl/A"}), {1, 3, 4}));
    CHECK(SetIs(index.AffectedByControls({"/Ctl/A", "/Ctl/B"}),
                {1, 2, 3, 4}));
    // An unknown control reaches everything: the conservative answer.
    CHECK(SetIs(index.AffectedByControls({"/Ctl/Nope"}),
                {0, 1, 2, 3, 4, 5}));
    CHECK(SetIs(index.AffectedByControls({}), {}));
    CHECK(index.Walks() == walks + 4);

    CHECK(SetIs(index.AffectedClusters(std::vector<int>{3}), {3, 4}));
    // Out-of-range seeds are skipped, not trusted.
    CHECK(SetIs(index.AffectedClusters(std::vector<int>{4, 600}), {4}));

    RigExecBakedClusterSet seeds;
    seeds.Resize(6);
    seeds.Set(2);
    CHECK(SetIs(index.AffectedClusters(seeds), {2, 3, 4}));
    // A shorter set reads its missing words as zero.
    RigExecBakedClusterSet shortSeeds;
    CHECK(SetIs(index.AffectedClusters(shortSeeds), {}));

    // dirty ∩ affecting(requested), as bit arrays.
    RigExecBakedClusterSet dirty, affecting;
    dirty.Resize(6);
    affecting.Resize(6);
    dirty.Set(1);
    dirty.Set(3);
    dirty.Set(4);
    affecting.Set(3);
    affecting.Set(4);
    affecting.Set(5);
    CHECK(SetIs(RigExecIntersectClusterSets(dirty, affecting), {3, 4}));
    CHECK(!RigExecIntersectClusterSets(dirty, shortSeeds).Any());

    // Out-of-range mappings are dropped; Build clears explicit ones.
    index.MapControl("/Ctl/A", {1, 99, -2});
    CHECK(index.SeedsForControl("/Ctl/A") == std::vector<int>{1});
    index.Build(program, 9);
    CHECK(index.EpochDigest() == 9);
    CHECK(index.SeedsForControl("/Ctl/A").empty());
    CHECK(index.Walks() == 0);
    index.Clear();
    CHECK(index.Empty());
    CHECK(SetIs(index.AffectedByControls({"/Ctl/A"}), {}));

    // A program whose cones never built is refused, not trusted empty.
    RigExecBakedProgramImpl unconed{};
    unconed.clustering.clusters.resize(3);
    RigExecOutputAffectedIndex refused;
    refused.Build(unconed, 7);
    CHECK(refused.Empty());
}

// The task list memoizes one closed set per (control, epoch): repeats hit,
// epochs are isolated, and invalidation drops exactly one epoch.
void
TestTaskListCache()
{
    RigExecTaskListCache memo;
    RigExecBakedClusterSet set;
    set.Resize(6);
    set.Set(1);
    set.Set(3);

    RigExecBakedClusterSet out;
    CHECK(!memo.Lookup(7, "/Ctl/A", &out));
    CHECK(!memo.Lookup(7, "/Ctl/A", nullptr));
    memo.Store(7, "/Ctl/A", set);
    CHECK(memo.Lookup(7, "/Ctl/A", &out));
    CHECK(SetIs(out, {1, 3}));
    // Another epoch is another table.
    CHECK(!memo.Lookup(8, "/Ctl/A", &out));
    memo.Store(8, "/Ctl/A", set);
    CHECK(memo.Size() == 2);

    RigExecTaskListStats stats = memo.Stats();
    CHECK(stats.hits == 1);
    CHECK(stats.misses == 3);
    CHECK(stats.stores == 2);
    CHECK(stats.entries == 2);

    CHECK(memo.InvalidateEpoch(7) == 1);
    CHECK(memo.Size() == 1);
    CHECK(!memo.Lookup(7, "/Ctl/A", &out));
    CHECK(memo.Lookup(8, "/Ctl/A", &out));
    CHECK(memo.InvalidateEpoch(7) == 0);
    memo.Clear();
    CHECK(memo.Size() == 0);
    stats = memo.Stats();
    CHECK(stats.hits == 0 && stats.misses == 0 && stats.stores == 0);
}

// Changed controls are a by-value diff: moves, adds, drops, hasValue flips,
// and override changes all name their control; identical inputs name none.
void
TestChangedControls()
{
    const RigExecRetainedFrameState cached =
        MakeRetained(7, 6, UsdTimeCode(1.0), {{"/Ctl/A", 1.0}, {"/Ctl/B", 2.0}});

    const std::vector<RigExecValueOverride> noOverrides;
    CHECK(RigExecChangedControls(cached, cached.inputs, noOverrides).empty());

    CHECK(RigExecChangedControls(
              cached,
              MakeInputs(UsdTimeCode(1.0),
                         {{"/Ctl/A", 1.5}, {"/Ctl/B", 2.0}}),
              noOverrides) == std::vector<RigExecControlId>{"/Ctl/A"});
    // Time is not a control: the plan reads it separately.
    CHECK(RigExecChangedControls(
              cached,
              MakeInputs(UsdTimeCode(2.0),
                         {{"/Ctl/A", 1.0}, {"/Ctl/B", 2.0}}),
              noOverrides).empty());
    // Added and dropped paths are changes.
    CHECK(RigExecChangedControls(
              cached,
              MakeInputs(UsdTimeCode(1.0), {{"/Ctl/A", 1.0}}),
              noOverrides) == std::vector<RigExecControlId>{"/Ctl/B"});
    CHECK(RigExecChangedControls(
              cached,
              MakeInputs(UsdTimeCode(1.0),
                         {{"/Ctl/A", 1.0},
                          {"/Ctl/B", 2.0},
                          {"/Ctl/C", 3.0}}),
              noOverrides) == std::vector<RigExecControlId>{"/Ctl/C"});

    // A value that stopped reading is a change; two valueless samples agree.
    RigExecFrameInputs dropped = MakeInputs(UsdTimeCode(1.0), {});
    dropped.Add(SdfPath("/Ctl/A"), VtValue(), false);
    dropped.Add(SdfPath("/Ctl/B"), VtValue(2.0));
    CHECK(RigExecChangedControls(cached, dropped, noOverrides) ==
          std::vector<RigExecControlId>{"/Ctl/A"});

    // +0.0 and -0.0 compare equal here, as they do in §7's own `!=` on the
    // avar table: an in-frame cone would skip that change, so the
    // cross-frame one does too. (The key digest stays bitwise-strict, so
    // exact-key lookup still misses across the sign.)
    CHECK(RigExecChangedControls(
              cached,
              MakeInputs(UsdTimeCode(1.0),
                         {{"/Ctl/A", 1.0}, {"/Ctl/B", 2.0}}),
              noOverrides).empty());
    RigExecFrameInputs negZero = MakeInputs(UsdTimeCode(1.0), {});
    negZero.Add(SdfPath("/Zero"), VtValue(0.0));
    RigExecRetainedFrameState cachedZero = MakeRetained(7, 6,
        UsdTimeCode(1.0), {});
    cachedZero.inputs = negZero;
    RigExecFrameInputs negZeroReq = MakeInputs(UsdTimeCode(1.0), {});
    negZeroReq.Add(SdfPath("/Zero"), VtValue(-0.0));
    CHECK(RigExecChangedControls(cachedZero, negZeroReq, noOverrides)
              .empty());

    // A NaN is unequal to everything including itself: always a change,
    // never a skip.
    RigExecFrameInputs nanReq = MakeInputs(UsdTimeCode(1.0), {});
    nanReq.Add(SdfPath("/Ctl/A"),
               VtValue(std::numeric_limits<double>::quiet_NaN()));
    nanReq.Add(SdfPath("/Ctl/B"), VtValue(2.0));
    CHECK(RigExecChangedControls(cached, nanReq, noOverrides) ==
          std::vector<RigExecControlId>{"/Ctl/A"});

    // Overrides key by (prim, computation, attribute): a changed value, an
    // added override, and a lifted one each name their control.
    RigExecValueOverride before, moved, added;
    before.prim = SdfPath("/Ctl/A");
    before.computation = TfToken("computePointFrame");
    before.value = VtValue(1.0);
    moved = before;
    moved.value = VtValue(9.0);
    added.prim = SdfPath("/Ctl/Z");
    added.computation = TfToken("computePointFrame");
    added.value = VtValue(0.0);
    RigExecRetainedFrameState cachedOvr = cached;
    cachedOvr.overrides = {before};
    CHECK(RigExecChangedControls(cachedOvr, cached.inputs, {before})
              .empty());
    const std::string movedId = "/Ctl/A|computePointFrame|";
    CHECK(RigExecChangedControls(cachedOvr, cached.inputs, {moved}) ==
          std::vector<RigExecControlId>{movedId});
    CHECK(RigExecChangedControls(cachedOvr, cached.inputs, {before, added}) ==
          std::vector<RigExecControlId>{"/Ctl/Z|computePointFrame|"});
    CHECK(RigExecChangedControls(cachedOvr, cached.inputs, {}) ==
          std::vector<RigExecControlId>{movedId});
}

// Planning: an empty cone is a hit with zero work, one moved control
// re-runs a strict subset, repeats reuse the memoized selection without
// rewalking, and epoch or topology drift is a miss.
void
TestPlanSparseReuse()
{
    const RigExecBakedProgramImpl program = MakeDiamond(false);
    RigExecOutputAffectedIndex index;
    index.Build(program, 7);
    index.MapControl("/Ctl/A", {1});
    index.MapControl("/Ctl/B", {2});
    RigExecTaskListCache memo;

    const RigExecRetainedFrameState cached =
        MakeRetained(7, 6, UsdTimeCode(1.0), {{"/Ctl/A", 1.0}, {"/Ctl/B", 2.0}});
    const std::vector<RigExecValueOverride> noOverrides;

    // Identical sources: a hit, and the suite's zero-work case.
    RigExecSparsePlan hit = RigExecPlanSparseReuse(
        index, &memo, cached, 7, cached.inputs, noOverrides);
    CHECK(hit.verdict == RigExecSparseVerdict::Hit);
    CHECK(hit.ClustersToRun() == 0);
    CHECK(hit.ClustersSkipped(6) == 6);
    CHECK(!hit.memoUsed);
    CHECK(index.Walks() == 0);

    // One moved control: the strict subset downstream of it.
    RigExecFrameInputs movedA = MakeInputs(UsdTimeCode(1.0), {});
    movedA.Add(SdfPath("/Ctl/A"), VtValue(1.5));
    movedA.Add(SdfPath("/Ctl/B"), VtValue(2.0));
    RigExecSparsePlan partial = RigExecPlanSparseReuse(
        index, &memo, cached, 7, movedA, noOverrides);
    CHECK(partial.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(partial.clusters, {1, 3, 4}));
    CHECK(partial.ClustersToRun() == 3);
    CHECK(partial.ClustersSkipped(6) == 3);
    CHECK(!partial.memoUsed);
    CHECK(index.Walks() == 1);

    // The repeat edit reuses the memoized selection: same set, no rewalk.
    RigExecSparsePlan repeat = RigExecPlanSparseReuse(
        index, &memo, cached, 7, movedA, noOverrides);
    CHECK(repeat.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(repeat.clusters, {1, 3, 4}));
    CHECK(repeat.memoUsed);
    CHECK(index.Walks() == 1);
    CHECK(memo.Stats().hits == 1);

    // Two moved controls union their memoized sets.
    RigExecFrameInputs movedBoth = MakeInputs(UsdTimeCode(1.0), {});
    movedBoth.Add(SdfPath("/Ctl/A"), VtValue(1.5));
    movedBoth.Add(SdfPath("/Ctl/B"), VtValue(2.5));
    RigExecSparsePlan both = RigExecPlanSparseReuse(
        index, &memo, cached, 7, movedBoth, noOverrides);
    CHECK(both.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(both.clusters, {1, 2, 3, 4}));

    // dirty ∩ affecting(requested): a move that reaches nothing requested
    // is still a hit.
    RigExecBakedClusterSet affecting;
    affecting.Resize(6);
    affecting.Set(5);
    RigExecSparsePlan unrequested = RigExecPlanSparseReuse(
        index, &memo, cached, 7, movedA, noOverrides, &affecting);
    CHECK(unrequested.verdict == RigExecSparseVerdict::Hit);
    affecting.Set(3);
    RigExecSparsePlan narrowed = RigExecPlanSparseReuse(
        index, &memo, cached, 7, movedA, noOverrides, &affecting);
    CHECK(narrowed.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(narrowed.clusters, {3}));

    // An unknown control re-runs everything: conservative, never wrong.
    RigExecFrameInputs movedUnknown = MakeInputs(UsdTimeCode(1.0), {});
    movedUnknown.Add(SdfPath("/Ctl/A"), VtValue(1.0));
    movedUnknown.Add(SdfPath("/Ctl/B"), VtValue(2.0));
    movedUnknown.Add(SdfPath("/Ctl/Z"), VtValue(0.0));
    RigExecSparsePlan unknown = RigExecPlanSparseReuse(
        index, &memo, cached, 7, movedUnknown, noOverrides);
    CHECK(unknown.verdict == RigExecSparseVerdict::Partial);
    CHECK(unknown.ClustersToRun() == 6);

    // Epoch drift either side of the plan is a miss.
    CHECK(RigExecPlanSparseReuse(index, &memo, cached, 8, movedA,
                                 noOverrides).verdict ==
          RigExecSparseVerdict::Miss);
    RigExecRetainedFrameState otherEpoch = cached;
    otherEpoch.epochDigest = 8;
    CHECK(RigExecPlanSparseReuse(index, &memo, otherEpoch, 7, movedA,
                                 noOverrides).verdict ==
          RigExecSparseVerdict::Miss);
    RigExecOutputAffectedIndex otherIndex;
    otherIndex.Build(program, 8);
    CHECK(RigExecPlanSparseReuse(otherIndex, &memo, cached, 7, movedA,
                                 noOverrides).verdict ==
          RigExecSparseVerdict::Miss);
    RigExecOutputAffectedIndex emptyIndex;
    CHECK(RigExecPlanSparseReuse(emptyIndex, &memo, cached, 7, movedA,
                                 noOverrides).verdict ==
          RigExecSparseVerdict::Miss);

    // A cluster count that moved under a standing epoch is a topology
    // change: miss, flagged, and the epoch's memo dropped.
    CHECK(memo.Size() > 0);
    RigExecRetainedFrameState reshaped = cached;
    reshaped.clusterCount = 5;
    RigExecSparsePlan topology = RigExecPlanSparseReuse(
        index, &memo, reshaped, 7, movedA, noOverrides);
    CHECK(topology.verdict == RigExecSparseVerdict::Miss);
    CHECK(topology.topologyChanged);
    CHECK(memo.Size() == 0);

    // Without a memo the plan still draws, walking every time.
    const size_t walks = index.Walks();
    RigExecSparsePlan unmemoized = RigExecPlanSparseReuse(
        index, nullptr, cached, 7, movedA, noOverrides);
    CHECK(unmemoized.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(unmemoized.clusters, {1, 3, 4}));
    CHECK(!unmemoized.memoUsed);
    CHECK(index.Walks() == walks + 1);
}

// At a moved time the always-dirty steps re-run even when every compared
// source agrees (§7's time rule, cross-frame); at a standing time the same
// request is a hit.
void
TestPlanTimeRule()
{
    const RigExecBakedProgramImpl program = MakeDiamond(true);
    RigExecOutputAffectedIndex index;
    index.Build(program, 7);
    CHECK(SetIs(index.Always(), {4}));

    const RigExecRetainedFrameState cached =
        MakeRetained(7, 6, UsdTimeCode(1.0), {{"/Ctl/A", 1.0}});
    const std::vector<RigExecValueOverride> noOverrides;

    RigExecSparsePlan standing = RigExecPlanSparseReuse(
        index, nullptr, cached, 7, cached.inputs, noOverrides);
    CHECK(standing.verdict == RigExecSparseVerdict::Hit);

    const RigExecFrameInputs later =
        MakeInputs(UsdTimeCode(2.0), {{"/Ctl/A", 1.0}});
    RigExecSparsePlan moved = RigExecPlanSparseReuse(
        index, nullptr, cached, 7, later, noOverrides);
    CHECK(moved.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(moved.clusters, {4}));
}

// Execution runs the plan's clusters in the program's topological order and
// counts them: hits run zero, partials run their strict subset, misses run
// nothing, and a runner that hands the generation back stops the run.
void
TestRunSparsePlan()
{
    const RigExecBakedProgramImpl program = MakeDiamond(false);
    const std::vector<int> &order = program.clustering.topologicalOrder;
    RigExecOutputAffectedIndex index;
    index.Build(program, 7);
    index.MapControl("/Ctl/A", {1});

    const RigExecRetainedFrameState cached =
        MakeRetained(7, 6, UsdTimeCode(1.0), {{"/Ctl/A", 1.0}});
    const std::vector<RigExecValueOverride> noOverrides;
    const RigExecFrameInputs movedA =
        MakeInputs(UsdTimeCode(1.0), {{"/Ctl/A", 2.0}});

    const RigExecSparsePlan partial = RigExecPlanSparseReuse(
        index, nullptr, cached, 7, movedA, noOverrides);
    CHECK(partial.verdict == RigExecSparseVerdict::Partial);
    std::vector<int> ran;
    const RigExecSparseExecution done = RigExecRunSparsePlan(
        partial, order, [&ran](int cluster) {
            ran.push_back(cluster);
            return true;
        });
    CHECK(done.completed);
    CHECK(done.executed == 3);
    CHECK(ran == std::vector<int>({1, 3, 4}));
    CHECK(done.executedClusters == ran);

    const RigExecSparsePlan hit = RigExecPlanSparseReuse(
        index, nullptr, cached, 7, cached.inputs, noOverrides);
    ran.clear();
    const RigExecSparseExecution zero = RigExecRunSparsePlan(
        hit, order, [&ran](int cluster) {
            ran.push_back(cluster);
            return true;
        });
    CHECK(zero.completed);
    CHECK(zero.executed == 0);
    CHECK(ran.empty());

    const RigExecSparsePlan miss = RigExecPlanSparseReuse(
        index, nullptr, cached, 8, movedA, noOverrides);
    ran.clear();
    const RigExecSparseExecution none = RigExecRunSparsePlan(
        miss, order, [&ran](int cluster) {
            ran.push_back(cluster);
            return true;
        });
    CHECK(!none.completed);
    CHECK(none.executed == 0);
    CHECK(ran.empty());
    CHECK(RigExecRunSparsePlan(partial, order, nullptr).executed == 0);

    // The runner hands the generation back at cluster 3: cluster 1 ran and
    // the run reports incomplete, so the caller live-evals.
    ran.clear();
    const RigExecSparseExecution aborted = RigExecRunSparsePlan(
        partial, order, [&ran](int cluster) {
            if (cluster == 3) {
                return false;
            }
            ran.push_back(cluster);
            return true;
        });
    CHECK(!aborted.completed);
    CHECK(aborted.executed == 1);
    CHECK(ran == std::vector<int>({1}));

    // An order that leaves out a planned cluster runs nothing: running a
    // subset of the plan is never correct, so the caller live-evals.
    ran.clear();
    const RigExecSparseExecution uncovered = RigExecRunSparsePlan(
        partial, std::vector<int>({1, 4}), [&ran](int cluster) {
            ran.push_back(cluster);
            return true;
        });
    CHECK(!uncovered.completed);
    CHECK(uncovered.executed == 0);
    CHECK(ran.empty());
}

// Cluster ids number level-packing bins, not dependencies: a cluster can
// have a higher-numbered predecessor. Here 3 -> 0 -> 1 and 2 -> 1, so a walk
// in increasing id would run 0 before 3 and 1 before 2 -- each against a
// predecessor's slots from the retained frame, not this one. The plan runs
// in the stored topological order instead, and the runner checks that every
// planned predecessor of a cluster ran before it.
//
//   3 -> 0 -> 1
//        2 -> 1
//   4 (no edges)
void
TestRunSparsePlanRunsPredecessorsFirst()
{
    RigExecBakedProgramImpl program{};
    program.clustering.clusters.resize(5);
    const auto edge = [&program](int from, int to) {
        program.clustering.clusters[size_t(from)].succs.push_back(to);
        program.clustering.clusters[size_t(to)].preds.push_back(from);
    };
    edge(3, 0);
    edge(0, 1);
    edge(2, 1);
    RigExecBakedBuildCones(&program);

    // Build caches the order: every cluster once, each after its preds.
    const std::vector<int> &order = program.clustering.topologicalOrder;
    CHECK(order.size() == 5);
    std::vector<int> position(5, -1);
    for (size_t k = 0; k < order.size(); ++k) {
        CHECK(order[k] >= 0 && order[k] < 5);
        if (order[k] >= 0 && order[k] < 5) {
            CHECK(position[size_t(order[k])] == -1);
            position[size_t(order[k])] = int(k);
        }
    }
    for (int c = 0; c < 5; ++c) {
        for (const int pred : program.clustering.clusters[size_t(c)].preds) {
            CHECK(position[size_t(pred)] < position[size_t(c)]);
        }
    }
    CHECK(RigExecBakedClusterTopologicalOrder(program.clustering) == order);
    CHECK(SetIs(program.cones.cone[3], {3, 0, 1}));

    RigExecOutputAffectedIndex index;
    index.Build(program, 7);
    index.MapControl("/Ctl/A", {3});
    index.MapControl("/Ctl/B", {2});
    const RigExecRetainedFrameState cached = MakeRetained(
        7, 5, UsdTimeCode(1.0), {{"/Ctl/A", 1.0}, {"/Ctl/B", 1.0}});
    const std::vector<RigExecValueOverride> noOverrides;

    // A fake runner: a cluster may run only once every predecessor the plan
    // names has run. It counts the clusters that ran too early.
    const auto runInOrder = [&program](const RigExecSparsePlan &plan,
                                       const std::vector<int> &walk,
                                       std::vector<int> *ran,
                                       int *early) {
        return RigExecRunSparsePlan(
            plan, walk, [&program, &plan, ran, early](int cluster) {
                for (const int pred :
                     program.clustering.clusters[size_t(cluster)].preds) {
                    if (plan.clusters.Test(pred) &&
                        std::find(ran->begin(), ran->end(), pred) ==
                            ran->end()) {
                        ++*early;
                    }
                }
                ran->push_back(cluster);
                return true;
            });
    };

    const RigExecSparsePlan movedA = RigExecPlanSparseReuse(
        index, nullptr, cached, 7,
        MakeInputs(UsdTimeCode(1.0), {{"/Ctl/A", 2.0}, {"/Ctl/B", 1.0}}),
        noOverrides);
    CHECK(movedA.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(movedA.clusters, {0, 1, 3}));
    std::vector<int> ran;
    int early = 0;
    const RigExecSparseExecution a = runInOrder(movedA, order, &ran, &early);
    CHECK(a.completed);
    CHECK(a.executed == 3);
    CHECK(early == 0);
    CHECK(ran == std::vector<int>({3, 0, 1}));
    CHECK(a.executedClusters == ran);

    // The runner does catch the old walk: increasing id runs 0 before 3.
    ran.clear();
    early = 0;
    runInOrder(movedA, std::vector<int>({0, 1, 2, 3, 4}), &ran, &early);
    CHECK(early > 0);

    const RigExecSparsePlan movedBoth = RigExecPlanSparseReuse(
        index, nullptr, cached, 7,
        MakeInputs(UsdTimeCode(1.0), {{"/Ctl/A", 2.0}, {"/Ctl/B", 2.0}}),
        noOverrides);
    CHECK(SetIs(movedBoth.clusters, {0, 1, 2, 3}));
    ran.clear();
    early = 0;
    const RigExecSparseExecution both =
        runInOrder(movedBoth, order, &ran, &early);
    CHECK(both.completed);
    CHECK(both.executed == 4);
    CHECK(early == 0);
    CHECK(ran.size() == 4 && ran.back() == 1);
}

// The cache's sparsity hooks: retained handles publish and serve beside the
// pose with accounted bytes, pose-only entries serve a null handle, and
// EvictEpoch drops exactly one epoch.
void
TestRetainedPublishAndEpochEviction()
{
    RigExecFrameCache cache;
    const RigExecRigPose pose = MakePose(1.0, 4);
    const size_t poseBytes = RigExecFrameCachePoseBytes(pose);
    CHECK(poseBytes > 0);

    auto retained = std::make_shared<RigExecRetainedFrameState>(
        MakeRetained(7, 6, UsdTimeCode(1.0), {{"/Ctl/A", 1.0}}));
    retained->slotBytes = 4096;
    const size_t retainedBytes = retained->RetainedBytes();
    CHECK(retainedBytes > 4096);

    const RigExecFrameCacheKey key{7, 11};
    CHECK(cache.Publish(key, UsdTimeCode(1.0), pose, retainedBytes,
                        retained));
    CHECK(cache.Stats().bytes == poseBytes + retainedBytes);

    RigExecRigPose served;
    std::shared_ptr<const void> handle;
    CHECK(cache.Lookup(key, &served, &handle));
    CHECK(served.jointFramesFinal == pose.jointFramesFinal);
    CHECK(handle == retained);

    // The plain Lookup still serves the pose alone.
    RigExecRigPose plain;
    CHECK(cache.Lookup(key, &plain));
    CHECK(plain.jointFramesFinal == pose.jointFramesFinal);

    // A pose-only entry serves a null handle.
    const RigExecFrameCacheKey poseOnly{7, 12};
    CHECK(cache.Publish(poseOnly, UsdTimeCode(1.0), pose));
    std::shared_ptr<const void> noHandle =
        std::make_shared<int>(1);
    CHECK(cache.Lookup(poseOnly, &served, &noHandle));
    CHECK(!noHandle);

    // Bytes without a handle are declined, like an invalid pose.
    CHECK(!cache.Publish({7, 13}, UsdTimeCode(1.0), pose, 64, nullptr));
    RigExecRigPose invalid;
    CHECK(!cache.Publish({7, 13}, UsdTimeCode(1.0), invalid, 64, retained));
    // Oversized retained entries are dropped, never partially stored.
    CHECK(!cache.Publish({7, 13}, UsdTimeCode(1.0), pose,
                         cache.GetByteCap(), retained));

    // Retained bytes count toward the cap: one full entry fits, two evict.
    cache.Clear();
    CHECK(cache.Publish(key, UsdTimeCode(1.0), pose, retainedBytes,
                        retained));
    cache.SetByteCap(poseBytes + retainedBytes);
    CHECK(cache.Stats().entryCount == 1);
    CHECK(cache.Publish(poseOnly, UsdTimeCode(2.0), pose));
    CHECK(cache.Stats().entryCount == 1);
    CHECK(!cache.Lookup(key, &served, &handle));

    // EvictEpoch drops exactly the named epoch.
    cache.Clear();
    cache.SetByteCap(kRigExecFrameCacheDefaultByteCap);
    CHECK(cache.Publish({7, 1}, UsdTimeCode(1.0), pose));
    CHECK(cache.Publish({7, 2}, UsdTimeCode(2.0), pose, retainedBytes,
                        retained));
    CHECK(cache.Publish({8, 1}, UsdTimeCode(1.0), pose));
    CHECK(cache.EvictEpoch(7) == 2);
    CHECK(!cache.Lookup({7, 1}, &served));
    CHECK(!cache.Lookup({7, 2}, &served, &handle));
    CHECK(cache.Lookup({8, 1}, &served));
    CHECK(cache.EvictEpoch(7) == 0);
    CHECK(cache.Stats().entryCount == 1);
}

// The candidate sidecar maps (epoch, time) to the key a re-warm reuses.
void
TestCandidateIndex()
{
    RigExecSparseCandidateIndex candidates;
    CHECK(candidates.Size() == 0);
    const RigExecFrameCacheKey key{7, 11};
    candidates.NotePublished(7, UsdTimeCode(1.0), key);
    candidates.NotePublished(7, UsdTimeCode(2.0), {7, 12});
    CHECK(candidates.Size() == 2);

    RigExecFrameCacheKey found{0, 0};
    CHECK(candidates.Find(7, UsdTimeCode(1.0), &found));
    CHECK(found == key);
    CHECK(!candidates.Find(7, UsdTimeCode(3.0), &found));
    CHECK(!candidates.Find(8, UsdTimeCode(1.0), &found));
    CHECK(!candidates.Find(7, UsdTimeCode(1.0), nullptr));

    CHECK(candidates.Drop(7, UsdTimeCode(1.0)));
    CHECK(!candidates.Drop(7, UsdTimeCode(1.0)));
    CHECK(candidates.Size() == 1);
    candidates.Clear();
    CHECK(candidates.Size() == 0);
}

namespace {

// One skinned mesh over two shared controls, after testRigExecChainLevels'
// MakeMultiMeshRig: small (8 points), valid envelope, no time samples -- so
// the controls' avars are captured constants and a value edit on one hits
// the bake's capture index.
UsdStageRefPtr
MakeSingleMeshRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim alongX = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongX"), TfToken("RigExecControl"));
    alongX.GetAttribute(TfToken("avars:tx")).Set(10.0);
    const UsdPrim alongY = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongY"), TfToken("RigExecControl"));
    alongY.GetAttribute(TfToken("avars:ty")).Set(20.0);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    constexpr size_t points = 8;
    const UsdPrim prim =
        stage->DefinePrim(SdfPath("/Asset/Geom/Mesh_0"), TfToken("Mesh"));
    VtVec3fArray base(points);
    for (size_t i = 0; i < points; ++i) {
        base[i] = GfVec3f(float(i) * 0.5f, float(i) * -0.25f,
                          -float(i) * 0.125f);
    }
    prim.GetAttribute(TfToken("points")).Set(base);

    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin_0"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/Mesh_0.points")});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({alongX.GetPath(), alongY.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(2);
    VtIntArray indices(points * 2);
    for (size_t i = 0; i < points; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
    }
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(indices);
    VtFloatArray weights(points * 2);
    for (size_t i = 0; i < points; ++i) {
        weights[i * 2] = 0.1f;
        weights[i * 2 + 1] = 0.9f;
    }
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray).Set(weights);
    return stage;
}

// Routes each notice through the capture index inside the callback, where
// the notice object is alive: no copy of it is ever needed.
struct CaptureListener : public TfWeakBase {
    RigExecFrameCache *cache = nullptr;
    RigExecTaskListCache *memo = nullptr;
    uint64_t epoch = 0;
    const RigExecBakedProgram *program = nullptr;
    // Per edit under test: what the oracle said and what the routing did.
    bool oracleHit = false;
    bool routedHit = false;
    size_t notices = 0;

    void Handle(const UsdNotice::ObjectsChanged &notice)
    {
        ++notices;
        oracleHit = program->IsInvalidatedBy(notice);
        routedHit = RigExecNoteCaptureIndex(*cache, *memo, epoch, *program,
                                            notice);
    }
};

}  // namespace

// Union tolerates mismatched widths: a narrower other reads as zero past
// its end, a wider one grows the set, and `grew` still answers whether any
// member was added (the fixpoint test). Reachable from planning when a
// memoized set is narrower than the dirty set.
void
TestUnionToleratesMismatchedWidths()
{
    // Wide union narrow: members kept, nothing spurious, growth honest.
    RigExecBakedClusterSet wide;
    wide.Resize(130);
    wide.Set(0);
    wide.Set(129);
    RigExecBakedClusterSet narrow;
    narrow.Resize(64);
    narrow.Set(1);
    CHECK(wide.Union(narrow));
    CHECK(wide.Test(0) && wide.Test(1) && wide.Test(129));
    CHECK(wide.Count() == 3);
    CHECK(!wide.Union(narrow));

    // Narrow union wide: grows to the wider width with every member.
    RigExecBakedClusterSet grown;
    grown.Resize(64);
    grown.Set(1);
    CHECK(grown.Union(wide));
    CHECK(grown.words.size() == wide.words.size());
    CHECK(grown.Test(0) && grown.Test(1) && grown.Test(129));
    CHECK(grown.Count() == 3);

    // Equal widths behave exactly as before.
    RigExecBakedClusterSet a;
    a.Resize(70);
    a.Set(3);
    RigExecBakedClusterSet b;
    b.Resize(70);
    b.Set(69);
    CHECK(a.Union(b));
    CHECK(a.Test(3) && a.Test(69));
    CHECK(a.Count() == 2);
    CHECK(!a.Union(b));

    // An empty other adds nothing and reports no growth, whatever the width.
    RigExecBakedClusterSet empty;
    empty.Resize(200);
    CHECK(!a.Union(empty));
    CHECK(a.Count() == 2);
    RigExecBakedClusterSet emptyNarrow;
    CHECK(!a.Union(emptyNarrow));
    CHECK(a.Count() == 2);
}

// Capture-index epoch invalidation, end to end on a real baked program: a
// value edit on a captured constant hits the index and drops the epoch's
// frames and memo; an edit the bake never read misses and leaves every
// cached frame standing.
void
TestCaptureIndexEpochInvalidation()
{
    UsdStageRefPtr stage = MakeSingleMeshRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &error : errors) {
            std::printf("  compile: %s\n", error.c_str());
        }
        CHECK(false);
        return;
    }
    std::vector<std::string> reasons;
    std::unique_ptr<RigExecBakedProgram> program =
        RigExecBakedProgram::Build(&evaluator, &reasons);
    if (!program) {
        for (const std::string &reason : reasons) {
            std::printf("  bake refused: %s\n", reason.c_str());
        }
        CHECK(false);
        return;
    }
    const uint64_t epoch =
        uint64_t(evaluator.GetBindingEpochDigest());

    RigExecFrameCache cache;
    RigExecTaskListCache memo;
    const RigExecRigPose pose = MakePose(3.0, 2);
    CHECK(cache.Publish({epoch, 1}, UsdTimeCode(1.0), pose));
    CHECK(cache.Publish({epoch, 2}, UsdTimeCode(2.0), pose));
    CHECK(cache.Publish({epoch + 1, 1}, UsdTimeCode(1.0), pose));
    RigExecBakedClusterSet selected;
    selected.Resize(6);
    selected.Set(1);
    memo.Store(epoch, "/Ctl/A", selected);
    memo.Store(epoch + 1, "/Ctl/A", selected);

    CaptureListener listener;
    listener.cache = &cache;
    listener.memo = &memo;
    listener.epoch = epoch;
    listener.program = program.get();
    TfNotice::Key key = TfNotice::Register(TfCreateWeakPtr(&listener),
                                              &CaptureListener::Handle);

    // An edit the bake never read: prims and properties under a path the
    // index does not name. Misses, and keeps every entry.
    listener.notices = 0;
    stage->DefinePrim(SdfPath("/Asset/Unrelated"), TfToken("Scope"));
    stage->GetPrimAtPath(SdfPath("/Asset/Unrelated"))
        .CreateAttribute(TfToken("note"), SdfValueTypeNames->String)
        .Set(std::string("unrelated"));
    CHECK(listener.notices > 0);
    CHECK(!listener.oracleHit);
    CHECK(!listener.routedHit);
    RigExecRigPose served;
    CHECK(cache.Lookup({epoch, 1}, &served));
    CHECK(cache.Lookup({epoch, 2}, &served));
    CHECK(cache.Stats().entryCount == 3);
    CHECK(memo.Size() == 2);

    // A value edit on a captured constant: the control's default avar has
    // no samples and no connections, so the bake captured it and the edit
    // hits the rebuild index. Drops the epoch, keeps the other.
    listener.notices = 0;
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongX"))
        .GetAttribute(TfToken("avars:tx"))
        .Set(11.0);
    CHECK(listener.notices > 0);
    CHECK(listener.oracleHit);
    CHECK(listener.routedHit);
    CHECK(!cache.Lookup({epoch, 1}, &served));
    CHECK(!cache.Lookup({epoch, 2}, &served));
    CHECK(cache.Lookup({epoch + 1, 1}, &served));
    CHECK(cache.Stats().entryCount == 1);
    CHECK(memo.Size() == 1);
    RigExecBakedClusterSet out;
    CHECK(!memo.Lookup(epoch, "/Ctl/A", &out));
    CHECK(memo.Lookup(epoch + 1, "/Ctl/A", &out));

    TfNotice::Revoke(key);
}

// Known-empty is distinct from unknown: a universe member with no seeds
// retires nothing (and plans a zero-work Hit when it alone moved), while
// a foreign ID retires everything. The three-way branch, plus the
// universe that backs it.
void
TestKnownEmptyVsUnknown()
{
    const RigExecBakedProgramImpl program = MakeDiamond(false);
    RigExecOutputAffectedIndex index;
    index.Build(program, 7);
    index.MapControl("/Ctl/A", {1});
    index.MapControl("/Ctl/Empty", {});
    CHECK(index.IsKnownControl("/Ctl/A"));
    CHECK(index.IsKnownControl("/Ctl/Empty"));
    CHECK(!index.IsKnownControl("/Ctl/Nope"));
    CHECK(!index.IsKnownControl(""));
    const std::vector<RigExecControlId> universe = index.ControlUniverse();
    const auto has = [&](const char *id) {
        return std::find(universe.begin(), universe.end(),
                         RigExecControlId(id)) != universe.end();
    };
    CHECK(has("/Ctl/A"));
    CHECK(has("/Ctl/Empty"));
    CHECK(!has("/Ctl/Nope"));

    CHECK(index.SeedsForControl("/Ctl/Empty").empty());
    CHECK(SetIs(index.AffectedByControls({"/Ctl/Empty"}), {}));
    CHECK(SetIs(index.AffectedByControls({"/Ctl/Nope"}),
                {0, 1, 2, 3, 4, 5}));
    // A known-empty member contributes nothing to a union.
    CHECK(SetIs(index.AffectedByControls({"/Ctl/Empty", "/Ctl/A"}),
                {1, 3, 4}));
    // But it does not dilute a foreign member's conservatism.
    CHECK(SetIs(index.AffectedByControls({"/Ctl/Empty", "/Ctl/Nope"}),
                {0, 1, 2, 3, 4, 5}));

    // Planning: a moved known-empty control is a Hit with zero work.
    const RigExecRetainedFrameState cached = MakeRetained(
        7, 6, UsdTimeCode(1.0), {{"/Ctl/Empty", 1.0}, {"/Ctl/A", 1.0}});
    const std::vector<RigExecValueOverride> noOverrides;
    const RigExecSparsePlan hit = RigExecPlanSparseReuse(
        index, nullptr, cached, 7,
        MakeInputs(UsdTimeCode(1.0),
                   {{"/Ctl/Empty", 2.0}, {"/Ctl/A", 1.0}}),
        noOverrides);
    CHECK(hit.verdict == RigExecSparseVerdict::Hit);
    CHECK(hit.ClustersToRun() == 0);
    CHECK(hit.ClustersSkipped(6) == 6);
}

// The notice adapter over explicit path lists: property paths contribute
// both granularities, shadowed info-only ancestor prims contribute
// nothing, and everything else stays conservative.
void
TestNoticeAdapterPaths()
{
    const std::vector<SdfPath> none;
    using Ids = std::vector<RigExecControlId>;
    // A property contributes itself plus its prim; the ancestors it
    // shadows contribute nothing.
    CHECK(RigExecNoticeControlIdsFromPaths(
              none,
              {SdfPath("/A/B.attr"), SdfPath("/A/B"), SdfPath("/A")},
              none) == Ids({"/A/B", "/A/B.attr"}));
    // A lone prim path is an unspecified change under it: kept.
    CHECK(RigExecNoticeControlIdsFromPaths(none, {SdfPath("/X")}, none) ==
          Ids({"/X"}));
    // Twins never shadow each other into silence.
    CHECK(RigExecNoticeControlIdsFromPaths(
              none, {SdfPath("/P"), SdfPath("/P")}, none) == Ids({"/P"}));
    // Empty paths normalize to nothing.
    CHECK(RigExecNoticeControlIdsFromPaths(
              none, {SdfPath::EmptyPath()}, none)
              .empty());
    // Resynced paths are never filtered, even with info beneath them.
    CHECK(RigExecNoticeControlIdsFromPaths(
              {SdfPath("/R")}, {SdfPath("/R.a")}, none) ==
          Ids({"/R", "/R.a"}));
    // A resync at the prim names the change its info twin only echoes.
    CHECK(RigExecNoticeControlIdsFromPaths(
              {SdfPath("/R")}, {SdfPath("/R")}, none) == Ids({"/R"}));
    // Resolved-asset resyncs contribute like any resync.
    CHECK(RigExecNoticeControlIdsFromPaths(none, none,
                                           {SdfPath("/Ext/File")}) ==
          Ids({"/Ext/File"}));
}

// The adapter against a real notice: a value edit's packaging carries
// ancestor prims, and the adapter keeps the property plus its prim while
// the ancestors shadow out.
void
TestNoticeAdapterRealNotice()
{
    UsdStageRefPtr stage = MakeSingleMeshRig();
    struct AdapterListener : public TfWeakBase {
        std::vector<RigExecControlId> lastIds;
        size_t notices = 0;
        void Handle(const UsdNotice::ObjectsChanged &notice)
        {
            ++notices;
            lastIds = RigExecNoticeControlIds(notice);
        }
    };
    AdapterListener listener;
    TfNotice::Key key = TfNotice::Register(TfCreateWeakPtr(&listener),
                                           &AdapterListener::Handle);
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongX"))
        .GetAttribute(TfToken("avars:tx"))
        .Set(11.0);
    CHECK(listener.notices > 0);
    const auto has = [&](const char *id) {
        return std::find(listener.lastIds.begin(), listener.lastIds.end(),
                         RigExecControlId(id)) != listener.lastIds.end();
    };
    CHECK(has("/Asset/Rig/AlongX.avars:tx"));
    CHECK(has("/Asset/Rig/AlongX"));
    CHECK(!has("/Asset/Rig"));
    CHECK(!has("/Asset"));
    TfNotice::Revoke(key);
}

// Override seeds mirror SetOverrides placement exactly: the attribute
// override on prim-plus-attribute reaches its inputs' clusters, and
// anything unplaceable (computation, unknown, folded, exec-typed, or
// merely routed) reaches nothing -- left foreign, never admitted
// seedless.
void
TestOverrideSeeds()
{
    RigExecBakedProgramImpl program{};
    program.clustering.clusters.resize(3);
    program.steps.resize(3);
    program.steps[0].overrideInputs = {0};
    program.steps[0].cluster = 0;
    program.steps[1].overrideInputs = {1};
    program.steps[1].cluster = 1;
    program.steps[2].cluster = 2;
    program.overridableInputs[SdfPath("/P.a")] = {0, 1};
    program.folded.insert(SdfPath("/F.b"));
    program.overridableInputs[SdfPath("/F.b")] = {2};
    program.execTypedArrayInputs.insert(SdfPath("/E.c"));
    program.overridableInputs[SdfPath("/E.c")] = {2};
    program.resolvedRoutedPrims.insert(SdfPath("/R"));

    RigExecValueOverride place;
    place.prim = SdfPath("/P");
    place.attribute = TfToken("a");
    CHECK(RigExecOverrideSeeds(program, place) ==
          std::vector<int>({0, 1}));

    RigExecValueOverride computation;
    computation.prim = SdfPath("/P");
    computation.computation = TfToken("computeX");
    CHECK(RigExecOverrideSeeds(program, computation).empty());

    RigExecValueOverride unknown;
    unknown.prim = SdfPath("/Q");
    unknown.attribute = TfToken("z");
    CHECK(RigExecOverrideSeeds(program, unknown).empty());

    RigExecValueOverride folded;
    folded.prim = SdfPath("/F");
    folded.attribute = TfToken("b");
    CHECK(RigExecOverrideSeeds(program, folded).empty());

    RigExecValueOverride execTyped;
    execTyped.prim = SdfPath("/E");
    execTyped.attribute = TfToken("c");
    CHECK(RigExecOverrideSeeds(program, execTyped).empty());

    RigExecValueOverride routed;
    routed.prim = SdfPath("/R");
    routed.attribute = TfToken("anything");
    CHECK(RigExecOverrideSeeds(program, routed).empty());

    // The avar half: an override on a bound avar reaches its provider's
    // compose cluster through the value comparison, even though no step
    // carries its index.
    RigExecBakedProgramImpl avars{};
    avars.clustering.clusters.resize(2);
    avars.cones.avarCluster = {0, 1};
    avars.overridableInputs[SdfPath("/A.v")] = {5};
    RigExecBakedProgramImpl::AvarBinding bound;
    bound.slot = 11;
    bound.input.overrideIndex = 5;
    avars.avarBindings.push_back(bound);
    RigExecValueOverride avar;
    avar.prim = SdfPath("/A");
    avar.attribute = TfToken("v");
    CHECK(RigExecOverrideSeeds(avars, avar) == std::vector<int>({1}));
}

// Planner/executor parity (plan 2.1): at a moved time the plan re-runs
// the always-dirty set plus the varying closure (not just Always), and
// standing overrides dirty the override closure even when no value moved.
void
TestPlannerExecutorParity()
{
    // The diamond, plus a varying step in cluster 1 and an override step
    // in cluster 2. Solve with empty reads is inert in BuildCones' table
    // lookups and reads nothing outside the graph, so Always stays {4}
    // while the two closures land exactly on their cones.
    RigExecBakedProgramImpl program{};
    program.clustering.clusters.resize(6);
    const auto edge = [&program](int from, int to) {
        program.clustering.clusters[size_t(from)].succs.push_back(to);
        program.clustering.clusters[size_t(to)].preds.push_back(from);
    };
    edge(0, 1);
    edge(0, 2);
    edge(1, 3);
    edge(2, 3);
    edge(3, 4);
    RigExecBakedStep derived;
    derived.kind = RigExecBakedStepKind::Derived;
    derived.object = -1;
    derived.cluster = 4;
    program.steps.push_back(derived);
    RigExecBakedStep varying;
    varying.kind = RigExecBakedStepKind::Solve;
    varying.object = -1;
    varying.cluster = 1;
    varying.varyingInputs = true;
    program.steps.push_back(varying);
    RigExecBakedStep overriden;
    overriden.kind = RigExecBakedStepKind::Solve;
    overriden.object = -1;
    overriden.cluster = 2;
    overriden.overrideInputs = {0};
    program.steps.push_back(overriden);
    program.clustering.clusterOf = {4, 1, 2};
    RigExecBakedBuildCones(&program);

    RigExecOutputAffectedIndex index;
    index.Build(program, 7);
    CHECK(!index.Empty());
    CHECK(SetIs(index.Always(), {4}));
    CHECK(SetIs(index.Varying(), {1, 3, 4}));
    CHECK(SetIs(index.Override(), {2, 3, 4}));

    const RigExecRetainedFrameState cached = MakeRetained(
        7, 6, UsdTimeCode(1.0), {{"/Ctl/A", 1.0}});
    const std::vector<RigExecValueOverride> noOverrides;
    // A moved time with agreeing sources: Always plus the varying
    // closure, matching the executor's time rule.
    const RigExecSparsePlan moved = RigExecPlanSparseReuse(
        index, nullptr, cached, 7,
        MakeInputs(UsdTimeCode(2.0), {{"/Ctl/A", 1.0}}), noOverrides);
    CHECK(moved.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(moved.clusters, {1, 3, 4}));
    // Standing overrides with agreeing values: the override closure,
    // matching the executor's override rule.
    RigExecValueOverride standing;
    standing.prim = SdfPath("/Ctl/A");
    standing.computation = TfToken("computePointFrame");
    standing.value = VtValue(1.0);
    RigExecRetainedFrameState cachedOvr = cached;
    cachedOvr.overrides = {standing};
    const RigExecSparsePlan held = RigExecPlanSparseReuse(
        index, nullptr, cachedOvr, 7, cachedOvr.inputs, {standing});
    CHECK(held.verdict == RigExecSparseVerdict::Partial);
    CHECK(SetIs(held.clusters, {2, 3, 4}));
}

int
main()
{
    // The codeless schema has to be loadable before a RigExecControl means
    // anything to exec; ctest runs this with no plugin path set. The define
    // comes from CMake's test registration, as in testRigExecChainLevels.
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);

    TestGoNoGo();
    TestOutputAffectedIndex();
    TestTaskListCache();
    TestChangedControls();
    TestPlanSparseReuse();
    TestPlanTimeRule();
    TestRunSparsePlan();
    TestRunSparsePlanRunsPredecessorsFirst();
    TestRetainedPublishAndEpochEviction();
    TestCandidateIndex();
    TestUnionToleratesMismatchedWidths();
    TestCaptureIndexEpochInvalidation();
    TestKnownEmptyVsUnknown();
    TestNoticeAdapterPaths();
    TestNoticeAdapterRealNotice();
    TestOverrideSeeds();
    TestPlannerExecutorParity();
    if (failures == 0) {
        std::printf("PASS testRigExecFrameCacheSparsity\n");
    } else {
        std::printf("FAIL testRigExecFrameCacheSparsity: %d failure(s)\n",
                    failures);
    }
    return failures == 0 ? 0 : 1;
}
