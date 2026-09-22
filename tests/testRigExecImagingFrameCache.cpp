//
// testRigExecImagingFrameCache (Stream E): the imaging integration of the
// per-frame cache -- scrub-from-cache, live fallback, edit fencing, and
// cache-only background completions.
//
// The contract, one rule per test group in the testRigExecStaticInputCache
// style:
//
//   * SCRUB. A scripted scrub across a warmed range performs zero evaluator
//     pulls, and every served generation is bit-identical to the live
//     evaluation of the same frame.
//   * COLD. Scrub into cold frames evaluates live with behavior equal to
//     the cache-off path (same generations, same pull counts).
//   * BYPASS. Cache-off, empty, unsampleable, and playback sessions never
//     consult the frame cache: a rigExec:asset rig warms nothing and its
//     stats stay zero.
//   * EDIT. A rig-affecting edit bumps the warming generation and cancels
//     in-flight jobs; work sampled under the old token is dropped, and a
//     re-evaluation at the playhead is live.
//   * FENCE. Background completions publish into the cache only, behind
//     the generation fence: the snapshot store still holds the previous
//     generation afterwards, and stale or invalid completions drop.
//   * TRIGGER. OnEditCommitted enqueues neighbors plus sweep and never the
//     playhead; OnIdle enqueues the sweep only; a closed fill gate
//     enqueues nothing.
//   * BURST. The standing burst serves a second consecutive trigger with
//     no rebuild (one warmBurstRebuild instant, usable stays set); an
//     over-slice prep parks unusable and latches (plain per-frame route,
//     same burst shape, no repaid rebuild), and restoring the slice
//     unlatches the overrun.
//   * SCOPES. The 1.2 profiler scopes land with non-zero totals:
//     memoize sampling+digest and burst prep on the bridge profiler,
//     per-sweep-time factory cost on the scheduler profiler.
//   * INDEX. The warm-frame index, driven directly: completions under the
//     query generation+epoch read cached; older ones read dirty;
//     evictions retire only their own key; queue transitions drive
//     warming; reset clears.
//   * EVICT. The cache reports every drop with the key and time the index
//     retires on.
//   * STATES. The scripted SetTime/warm/drain sequence reads through the
//     per-frame API (C++ and C): memoized/warmed cached, edited dirty,
//     re-warmed cached, cleared uncached; an eviction retires to
//     uncached while the fresh playhead memo stands.
//   * BUDGET. The default sampling budget shapes triggers (idle: a
//     16-frame sweep slice; commit: 16 neighbors); explicit budgets bind
//     per trigger; a zero millisecond stop samples nothing.
//   * CURSOR. A set warm range replaces the default sweep: closest-first
//     from the playhead (ties prefer the future), visited frames skipped
//     without sampling; a refusal rig never counts visited.
//   * STREAKS. Non-publish streaks toward un-warmable-after-3: declines
//     and skips count; publishes, generation pushes, and evictions
//     clear; generation fences never count.
//   * PRODUCTION. The C-API trigger path (no injected runner) enqueues the
//     same bursts through the production runner, and the generation fence
//     drops stale production work.
//   * SERVED. A background completion plus its enqueue-time proof serves
//     without evaluating: the warmed pose reaches the viewport with zero
//     evaluator pulls.
//   * FAST. A servable index completion serves without sampling: revisits
//     hit with zero pulls, zero lookup-sample runs, and bit-identical
//     generations.
//   * INTERACTIVE. A drag at the playhead bypasses the frame cache
//     entirely: every tick carries unique overrides, so a lookup would
//     always miss after paying a full sample+digest, and memoization would
//     store single-use entries no scrub can reach. While overrides stand,
//     evaluations publish live with zero cache reads and zero writes.
//   * LANES. The frameCache profiler lane records production lookups: a
//     cold frame (no store consultation) records nothing, a hit records
//     cacheHit, and a proven lookup that misses the store records
//     cacheMiss.
//   * EPOCH. A notice that hits the baked capture index drops the epoch's
//     frames eagerly on the notice -- while the epoch half still names
//     them -- instead of leaving them for LRU; a miss leaves every entry
//     standing for D1 reachability to decide.
//   * BRANCHES. One case per 2.1 evaluator branch (patched / stamp-bumped
//     / stale): the recorded disposition plus the scoped-or-global
//     retirement each drives.
//   * CARRY. A constant patch re-keys provenance-clean entries (new key,
//     old evicted, proof re-pointed) with zero recompute while full
//     siblings retire; the carried frame serves with zero pulls.
//   * SCOPED-CANCEL. A gated old job for a purged time drops on fence-token
//     mismatch (never overwriting the requeued result) while untouched
//     times publish under the same generation.
//   * RETIRE. One control retires exactly the affected frames; the next
//     commit re-warms them first; re-warmed frames are bit-identical.
//   * PROOFS. Proofs carry their sampled-path set: constant patches retire
//     none, varying edits retire all.
//   * DRAG. Warming under a drag admits override identities with exact
//     seeds; release re-warms what the commit retired.
//   * FENCED-CLEAR. Clear with jobs queued and running, then drain: late
//     completions fence (no repopulation) and the 1.4 index resets.
//   * FENCE-RACE. A job paused between the fence-check and the cache
//     insert (holding the fence) while the main thread clears: the clear
//     serializes after the insert and the cache stays empty --
//     check-and-insert atomicity.
//   * OVERLAY-RACE. Setting the overlay mid-warming cancels before
//     clearing: old-flag jobs drop at the fence and no stale-overlay pose
//     is served afterwards.
//
// The rig is the frozen-context test's tiny in-memory rig (one skinned mesh
// over two animated controls), which bakes, evaluates, and samples. The
// tests set RIGEXEC_FRAME_CACHE in-process and also pass under the
// validation plan's outer combos (cache off, parallel eval off, verify on)
// by reading the live switches and expecting the combo's own counts.
//

#include "rigExecImaging/bridge.h"
#include "rigExecImaging/registry.h"
#include "rigExecBake/bake.h"
#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/frozenContext.h"
#include "rigExec/generation.h"
#include "rigExec/outputAffectedIndex.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/primRange.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                               \
    } while (0)

namespace {

void
SetEnv(const char *name, const char *value)
{
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

constexpr size_t kTinyPointCount = 8;

// One skinned mesh over two animated controls: frames genuinely differ
// instead of hashing alike (see testRigExecFrozenContext.cpp).
// \p staticY authors AlongY's ty as a default value only (no time samples),
// so the bake treats it as an epoch constant rather than a varying input.
UsdStageRefPtr
MakeTinyRig(bool staticY = false)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim alongX = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongX"), TfToken("RigExecControl"));
    const UsdPrim alongY = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongY"), TfToken("RigExecControl"));
    UsdAttribute tx = alongX.GetAttribute(TfToken("avars:tx"));
    UsdAttribute ty = alongY.GetAttribute(TfToken("avars:ty"));
    for (int t = 1; t <= 4; ++t) {
        tx.Set(10.0 + 0.1 * double(t), UsdTimeCode(double(t)));
        if (!staticY) {
            ty.Set(20.0 - 0.05 * double(t), UsdTimeCode(double(t)));
        }
    }
    if (staticY) {
        ty.Set(20.0);
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    const SdfPath meshPath("/Asset/Geom/Mesh_0");
    const UsdPrim prim = stage->DefinePrim(meshPath, TfToken("Mesh"));
    VtVec3fArray points(kTinyPointCount);
    for (size_t i = 0; i < kTinyPointCount; ++i) {
        points[i] = GfVec3f(float(i) * 0.5f, float(i) * -0.25f,
                            -float(i) * 0.125f);
    }
    prim.GetAttribute(TfToken("points")).Set(points);

    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin_0"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({meshPath.AppendProperty(TfToken("points"))});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({alongX.GetPath(), alongY.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(2);
    VtIntArray indices(kTinyPointCount * 2);
    for (size_t i = 0; i < kTinyPointCount; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
    }
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(indices);
    VtFloatArray weights(kTinyPointCount * 2);
    for (size_t i = 0; i < kTinyPointCount; ++i) {
        weights[i * 2] = 0.1f;
        weights[i * 2 + 1] = 0.9f;
    }
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray).Set(weights);
    return stage;
}

// The geometry leaves of the current generation, for cross-pass equality.
struct _GenerationGeometry {
    std::map<SdfPath, VtVec3fArray> points;
    std::map<SdfPath, VtVec3fArray> normals;
    std::map<SdfPath, float> movedFloats;
};

_GenerationGeometry
_CaptureGeometry(const RigExecImagingSnapshotConstPtr &snapshot)
{
    _GenerationGeometry out;
    if (!snapshot) {
        return out;
    }
    for (const auto &[path, prim] : snapshot->prims) {
        if (prim.hasPoints) {
            out.points[path] = prim.points;
        }
        if (prim.hasNormals) {
            out.normals[path] = prim.normals;
        }
    }
    out.movedFloats = snapshot->movedFloats;
    return out;
}

bool
_SameArrays(const VtVec3fArray &a, const VtVec3fArray &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

bool
_SameGeometry(const _GenerationGeometry &a, const _GenerationGeometry &b)
{
    if (a.points.size() != b.points.size() ||
        a.normals.size() != b.normals.size() ||
        a.movedFloats != b.movedFloats) {
        return false;
    }
    for (const auto &[path, points] : a.points) {
        const auto found = b.points.find(path);
        if (found == b.points.end() || !_SameArrays(points, found->second)) {
            return false;
        }
    }
    for (const auto &[path, normals] : a.normals) {
        const auto found = b.normals.find(path);
        if (found == b.normals.end() || !_SameArrays(normals, found->second)) {
            return false;
        }
    }
    return true;
}

// A serial kernel for warming tests: valid, recognizable (its time in the
// diagnostics), and render-neutral (nothing a guide fill would draw).
bool
_TestKernel(const RigExecFrozenEvalContext &context,
            const RigExecFrameInputs &inputs, RigExecFrozenArena &arena,
            RigExecRigPose *pose)
{
    if (!pose) {
        return false;
    }
    (void)context;
    (void)arena;
    pose->valid = true;
    pose->diagnostics.push_back(
        "warm:" + std::to_string(inputs.time.GetValue()));
    return true;
}

// A gate a warming kernel blocks in until the test releases it: lets a
// test hold jobs running on pool workers while the main thread clears,
// overlays, or otherwise races them. No early returns between gating and
// releasing -- blocked workers hang every later WaitUntilBackgroundIdle.
struct _PublishGate {
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<int> entered{0};
    std::atomic<bool> released{false};
};

// _TestKernel behind a gate: counts entry, blocks until released, then
// answers the same valid marker pose.
RigExecFrozenStepRunner
_GatedKernel(_PublishGate *gate)
{
    return [gate](const RigExecFrozenEvalContext &context,
                  const RigExecFrameInputs &inputs, RigExecFrozenArena &arena,
                  RigExecRigPose *pose) {
        (void)context;
        (void)arena;
        gate->entered.fetch_add(1);
        std::unique_lock<std::mutex> lock(gate->mutex);
        gate->cv.wait(lock, [gate] { return gate->released.load(); });
        if (!pose) {
            return false;
        }
        pose->valid = true;
        pose->diagnostics.push_back(
            "warm:" + std::to_string(inputs.time.GetValue()));
        return true;
    };
}

void
_ReleaseGate(_PublishGate *gate)
{
    {
        std::lock_guard<std::mutex> lock(gate->mutex);
        gate->released.store(true);
    }
    gate->cv.notify_all();
}

// Spins until \p ready or \p timeoutMs passes; answers what \p ready says
// at the end. A missed rendezvous fails loudly instead of hanging.
template <typename Ready>
bool
_WaitFor(Ready ready, int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (ready()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return ready();
}

// SCRUB + COLD. Cold frames evaluate live; a second pass over the warmed
// range performs zero evaluator pulls and serves identical generations.
void
TestScrubWarmsThenHits()
{
    std::printf("progress: TestScrubWarmsThenHits\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    const bool verify = RigExecFrameCacheVerifyRequested();
    const bool readsOn =
        RigExecFrameCacheModeFromEnvironment() != RigExecFrameCacheMode::Off;

    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    CHECK(registry.GetSessionEvaluationCount(rig) == 1);

    // Cold pass: every new frame evaluates live and memoizes.
    std::map<double, _GenerationGeometry> firstPass;
    firstPass[1.0] = _CaptureGeometry(registry.GetStore()->Get());
    CHECK(!firstPass[1.0].points.empty());
    for (double frame : {2.0, 3.0, 4.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        firstPass[frame] = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!firstPass[frame].points.empty());
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == 4);
    const RigExecFrameCacheStats coldStats =
        registry.GetFrameCacheStats(rig);
    if (readsOn) {
        CHECK(coldStats.published == 4);
        CHECK(coldStats.hits == 0);
    } else {
        CHECK(coldStats.published == 0);
    }

    // Warm pass: hits serve without evaluating (under verify, each hit is
    // shadow-proven against a live evaluation and counts as a pull, but
    // still serves the cached pose).
    const size_t revisits = 6;
    for (double frame : {3.0, 2.0, 1.0, 2.0, 3.0, 4.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        const _GenerationGeometry now =
            _CaptureGeometry(registry.GetStore()->Get());
        if (!_SameGeometry(firstPass[frame], now)) {
            std::printf("generation differs at frame %g\n", frame);
            CHECK(false);
        }
    }
    const size_t pulls = registry.GetSessionEvaluationCount(rig);
    const RigExecFrameCacheStats warmStats =
        registry.GetFrameCacheStats(rig);
    if (!readsOn) {
        CHECK(pulls == 4 + revisits);
    } else if (verify) {
        CHECK(pulls == 4 + revisits);
        CHECK(warmStats.hits == revisits);
    } else {
        CHECK(pulls == 4);
        CHECK(warmStats.hits == revisits);
        // No store miss was ever counted: the lookup runs only under a
        // freshness proof, and every proven lookup hit (a miss would mean
        // an entry was proven but evicted).
        CHECK(warmStats.misses == 0);
    }

    // Shadow pass: with verification on, every hit is proven against a
    // live evaluation -- pulls grow, hits still count, generations match.
    if (readsOn && !verify) {
        SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "1");
        CHECK(RigExecFrameCacheVerifyRequested());
        const size_t shadowPullsBefore =
            registry.GetSessionEvaluationCount(rig);
        const size_t shadowHitsBefore =
            registry.GetFrameCacheStats(rig).hits;
        for (double frame : {1.0, 2.0, 3.0, 4.0}) {
            CHECK(registry.SetTime(UsdTimeCode(frame)));
            const _GenerationGeometry now =
                _CaptureGeometry(registry.GetStore()->Get());
            if (!_SameGeometry(firstPass[frame], now)) {
                std::printf("shadow generation differs at frame %g\n", frame);
                CHECK(false);
            }
        }
        CHECK(registry.GetSessionEvaluationCount(rig) ==
              shadowPullsBefore + 4);
        CHECK(registry.GetFrameCacheStats(rig).hits == shadowHitsBefore + 4);
        SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    }
    registry.Deactivate();
}

// COLD. Cache-off scrubs evaluate every frame and produce the same
// generations the cache-on path serves.
void
TestCacheOffMatchesCacheOn()
{
    std::printf("progress: TestCacheOffMatchesCacheOn\n");
    std::fflush(stdout);
    std::map<double, _GenerationGeometry> offGenerations;
    {
        SetEnv("RIGEXEC_FRAME_CACHE", "off");
        UsdStageRefPtr stage = MakeTinyRig();
        const SdfPath rig("/Asset/Rig");
        RigExecImagingRegistry &registry =
            RigExecImagingRegistry::GetInstance();
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        offGenerations[1.0] = _CaptureGeometry(registry.GetStore()->Get());
        for (double frame : {2.0, 3.0, 4.0, 3.0, 2.0, 1.0}) {
            CHECK(registry.SetTime(UsdTimeCode(frame)));
            offGenerations[frame] = _CaptureGeometry(registry.GetStore()->Get());
        }
        CHECK(registry.GetSessionEvaluationCount(rig) == 7);
        const RigExecFrameCacheStats stats =
            registry.GetFrameCacheStats(rig);
        CHECK(stats.published == 0 && stats.hits == 0 && stats.misses == 0);
        registry.Deactivate();
    }
    {
        SetEnv("RIGEXEC_FRAME_CACHE", "on");
        SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
        UsdStageRefPtr stage = MakeTinyRig();
        const SdfPath rig("/Asset/Rig");
        RigExecImagingRegistry &registry =
            RigExecImagingRegistry::GetInstance();
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        for (double frame : {2.0, 3.0, 4.0, 3.0, 2.0, 1.0}) {
            CHECK(registry.SetTime(UsdTimeCode(frame)));
        }
        // Four pulls (one per distinct frame), and every generation the
        // cache-off path drew is drawn identically here.
        if (RigExecFrameCacheModeFromEnvironment() !=
            RigExecFrameCacheMode::Off) {
            CHECK(registry.GetSessionEvaluationCount(rig) == 4);
        }
        for (double frame : {1.0, 2.0, 3.0, 4.0}) {
            CHECK(registry.SetTime(UsdTimeCode(frame)));
            const _GenerationGeometry now =
                _CaptureGeometry(registry.GetStore()->Get());
            if (!_SameGeometry(offGenerations[frame], now)) {
                std::printf("cache-on differs from cache-off at %g\n", frame);
                CHECK(false);
            }
        }
        registry.Deactivate();
    }
}

// D1: the digest must cover every source value the frame reads. A control
// authored as a default value only is an epoch constant to the bake, not a
// varying input; editing it must still move every cached frame out of
// reach. The cache-off session is the reference.
void
TestStaticControlEditInvalidatesCachedFrames()
{
    std::printf("progress: TestStaticControlEditInvalidatesCachedFrames\n");
    std::fflush(stdout);
    const SdfPath rig("/Asset/Rig");
    const SdfPath yPath("/Asset/Rig/AlongY");
    auto drive = [&](UsdStageRefPtr stage, _GenerationGeometry *afterEdit) {
        RigExecImagingRegistry &registry =
            RigExecImagingRegistry::GetInstance();
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        // The edit lands while the playhead sits at 2; frame 1 is revisited
        // afterwards and must reflect it.
        stage->GetPrimAtPath(yPath).GetAttribute(TfToken("avars:ty"))
            .Set(5.0);
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        *afterEdit = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!afterEdit->points.empty());
        registry.Deactivate();
    };

    _GenerationGeometry reference;
    SetEnv("RIGEXEC_FRAME_CACHE", "off");
    drive(MakeTinyRig(/*staticY=*/true), &reference);

    _GenerationGeometry cached;
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    drive(MakeTinyRig(/*staticY=*/true), &cached);

    if (!_SameGeometry(reference, cached)) {
        std::printf("cache-on served a pre-edit pose at frame 1 after a "
                    "static control edit\n");
        CHECK(false);
    }
}

// EPOCH. A stage notice that hits the baked capture index drops the epoch's
// cached frames eagerly on the notice -- while the epoch half still names
// them -- instead of leaving them for LRU; a notice that misses the index
// leaves every entry standing for D1 reachability to decide.
void
TestCaptureIndexHitDropsEpochEagerly()
{
    std::printf("progress: TestCaptureIndexHitDropsEpochEagerly\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    const SdfPath yPath("/Asset/Rig/AlongY");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();

    // Hit: a default-value edit on a captured avar. The value patch takes
    // the scoped path (no epoch move, no eviction): the old frames stand
    // unreachable under the new digest plus the fresh playhead memo the
    // notice path re-evaluated.
    {
        std::vector<std::string> errors;
        UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.GetFrameCacheStats(rig).entryCount == 2);
        stage->GetPrimAtPath(yPath).GetAttribute(TfToken("avars:ty"))
            .Set(5.0);
        CHECK(registry.GetFrameCacheStats(rig).entryCount == 3);
        const size_t pulls = registry.GetSessionEvaluationCount(rig);
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        CHECK(registry.GetSessionEvaluationCount(rig) == pulls + 1);
        registry.Deactivate();
    }

    // Miss: an edit the bake never read leaves every entry standing, and
    // the revisit still hits.
    {
        std::vector<std::string> errors;
        UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.GetFrameCacheStats(rig).entryCount == 2);
        stage->DefinePrim(SdfPath("/Asset/Unrelated"), TfToken("Scope"));
        stage->GetPrimAtPath(SdfPath("/Asset/Unrelated"))
            .CreateAttribute(TfToken("note"), SdfValueTypeNames->String)
            .Set(std::string("unrelated"));
        CHECK(registry.GetFrameCacheStats(rig).entryCount == 2);
        const size_t pulls = registry.GetSessionEvaluationCount(rig);
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        CHECK(registry.GetSessionEvaluationCount(rig) == pulls);
        registry.Deactivate();
    }
}

// BYPASS. An overlay change retires cached poses (weightFields content
// moves without the digest moving), so the next publication is live.
void
TestOverlayChangeClearsCache()
{
    std::printf("progress: TestOverlayChangeClearsCache\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    for (double frame : {2.0, 3.0, 4.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
    }
    const bool readsOn =
        RigExecFrameCacheModeFromEnvironment() != RigExecFrameCacheMode::Off;
    if (readsOn) {
        CHECK(registry.GetFrameCacheStats(rig).published == 4);
    }
    const size_t pullsBefore = registry.GetSessionEvaluationCount(rig);
    // Any absolute prim path selects (the tiny rig owns no weight object,
    // so nothing draws -- the assertion is the retirement, not the paint).
    // The republish retires the old entries and memoizes the new flag's
    // first generation, so exactly one entry stands afterwards.
    CHECK(registry.SetWeightOverlay("/Asset/Rig/Movers/Skin_0"));
    const RigExecFrameCacheStats cleared =
        registry.GetFrameCacheStats(rig);
    if (readsOn) {
        CHECK(cleared.published == 1 && cleared.entryCount == 1);
        CHECK(cleared.hits == 0 && cleared.misses == 0);
    } else {
        CHECK(cleared.published == 0 && cleared.entryCount == 0);
    }
    // Only the playhead frame was re-memoized: a scrub elsewhere evaluates
    // live. (The same frame would be free through the redundant-SetTime
    // guard, which is not what this asserts.)
    CHECK(registry.SetTime(UsdTimeCode(3.0)));
    CHECK(registry.GetSessionEvaluationCount(rig) >= pullsBefore + 2);
    CHECK(registry.SetWeightOverlay(""));
    registry.Deactivate();
}

// BYPASS. A playback rig never consults the frame cache and never warms.
void
TestPlaybackBypassesCache(
    const std::filesystem::path &scratch)
{
    std::printf("progress: TestPlaybackBypassesCache\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = {1.0, 2.0, 3.0, 4.0};
    RigExecBakeResult baked;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &baked, &error));
    if (baked.bytes.empty()) {
        std::printf("bake diagnostic: %s\n", error.c_str());
        return;
    }
    const std::string binary =
        (scratch / "imaging-framecache.rigexec").string();
    {
        std::ofstream stream(binary, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(baked.bytes.data()),
                     std::streamsize(baked.bytes.size()));
    }
    UsdPrim rigPrim = stage->GetPrimAtPath(rig);
    const UsdAttribute asset = rigPrim.CreateAttribute(
        TfToken("rigExec:asset"), SdfValueTypeNames->Asset);
    CHECK(asset);
    asset.Set(SdfAssetPath(binary));

    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    for (double frame : {2.0, 3.0, 4.0, 3.0, 2.0, 1.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
    }
    // Every visit pulled (playback owns no cache lane), and the frame
    // cache stayed empty throughout.
    CHECK(registry.GetSessionEvaluationCount(rig) == 7);
    const RigExecFrameCacheStats stats = registry.GetFrameCacheStats(rig);
    CHECK(stats.published == 0 && stats.hits == 0 && stats.misses == 0 &&
          stats.entryCount == 0);
    CHECK(!registry.BuildWarmWork(
        rig, UsdTimeCode(2.0), registry.CurrentFrameGeneration(rig),
        _TestKernel).work);
    CHECK(registry.OnEditCommitted(_TestKernel) == 0);
    CHECK(registry.OnIdle(_TestKernel) == 0);
    // The production path is excluded the same way: no injected runner,
    // still nothing for a playback rig.
    CHECK(!registry.BuildWarmWork(
        rig, UsdTimeCode(2.0), registry.CurrentFrameGeneration(rig),
        RigExecFrozenStepRunner()).work);
    CHECK(registry.OnEditCommitted() == 0);
    CHECK(registry.OnIdle() == 0);
    registry.Deactivate();
}

// EDIT. An edit bumps the generation; work sampled under the old token is
// dropped, and the commit-time frame re-evaluates live.
void
TestEditBumpsGenerationAndDropsStaleWork()
{
    std::printf("progress: TestEditBumpsGenerationAndDropsStaleWork\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    const RigExecFrameGeneration before =
        registry.CurrentFrameGeneration(rig);

    // Sampled under the old token, run after the edit: dropped.
    RigExecWarmFactoryResult stale = registry.BuildWarmWork(
        rig, UsdTimeCode(3.0), before, _TestKernel);
    CHECK(static_cast<bool>(stale.work));

    // An explicit generation bump (what a Stale structural edit drives
    // through the notice path; avar value edits take the scoped path
    // with no bump). The fence assertions below are about the bump,
    // not about which edit caused it.
    registry.CancelFrameGeneration(rig);
    CHECK(registry.CurrentFrameGeneration(rig) == before + 1);

    const size_t publishedBefore =
        registry.GetFrameCacheStats(rig).published;
    RigExecImagingSnapshotConstPtr snapshotBefore =
        registry.GetStore()->Get();
    RigExecWarmRequest request;
    request.rig = rig;
    request.time = UsdTimeCode(3.0);
    request.priority = RigExecWarmPriority::Neighbor;
    request.generation = before;
    // Guarded: calling an empty work closure throws, and the CHECK above
    // already records the failure without aborting the suite.
    if (stale.work) {
        CHECK(stale.work(request) == RigExecWarmOutcome::DeclinedGeneration);
    }
    CHECK(registry.GetFrameCacheStats(rig).published == publishedBefore);
    CHECK(registry.GetStore()->Get() == snapshotBefore);

    // Fresh work under the new token completes into the cache only.
    RigExecWarmFactoryResult fresh = registry.BuildWarmWork(
        rig, UsdTimeCode(3.0), before + 1, _TestKernel);
    CHECK(static_cast<bool>(fresh.work));
    request.generation = before + 1;
    RigExecWarmOutcome freshOutcome = RigExecWarmOutcome::DeclinedInvalid;
    if (fresh.work) {
        freshOutcome = fresh.work(request);
    }
    if (RigExecFrameCacheModeFromEnvironment() !=
        RigExecFrameCacheMode::Off) {
        CHECK(freshOutcome == RigExecWarmOutcome::Published);
        CHECK(registry.GetFrameCacheStats(rig).published ==
              publishedBefore + 1);
    }
    CHECK(registry.GetStore()->Get() == snapshotBefore);
    // The injected kernel's poses are markers, not evaluations: retire
    // them so no later scrub can serve one.
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// FENCE. Completion semantics on an isolated scheduler and cache: current
// publishes, stale drops, invalid drops, and a null fence publishes.
void
TestBackgroundCompletionFence()
{
    std::printf("progress: TestBackgroundCompletionFence\n");
    std::fflush(stdout);
    RigExecBackgroundScheduler scheduler(0);
    const SdfPath rig("/Asset/Rig");
    auto cache = std::make_shared<RigExecFrameCache>();
    const RigExecFrameGeneration generation =
        scheduler.CurrentGeneration(rig);

    RigExecRigPose pose;
    pose.valid = true;
    pose.diagnostics.push_back("completion");
    const RigExecFrameCacheKey key{7, 11};
    CHECK(RigExecPublishBackgroundCompletion(
        cache, key, UsdTimeCode(3.0), pose, generation, &scheduler, rig));
    RigExecRigPose served;
    CHECK(cache->Lookup(key, &served));
    CHECK(served.valid);

    // An edit lands: the same token no longer publishes.
    scheduler.CancelGeneration(rig);
    const RigExecFrameCacheKey stale{7, 12};
    CHECK(!RigExecPublishBackgroundCompletion(
        cache, stale, UsdTimeCode(4.0), pose, generation, &scheduler, rig));
    CHECK(!cache->Lookup(stale, &served));

    // The new token publishes again.
    const RigExecFrameGeneration fresh = scheduler.CurrentGeneration(rig);
    CHECK(fresh == generation + 1);
    CHECK(RigExecPublishBackgroundCompletion(
        cache, stale, UsdTimeCode(4.0), pose, fresh, &scheduler, rig));
    CHECK(cache->Lookup(stale, &served));

    // Invalid poses and null caches never publish, with or without a fence.
    RigExecRigPose invalid;
    CHECK(!invalid.valid);
    const RigExecFrameCacheKey bad{7, 13};
    CHECK(!RigExecPublishBackgroundCompletion(
        cache, bad, UsdTimeCode(5.0), invalid, fresh, &scheduler, rig));
    CHECK(!cache->Lookup(bad, &served));
    CHECK(!RigExecPublishBackgroundCompletion(
        std::shared_ptr<RigExecFrameCache>(), bad, UsdTimeCode(5.0), pose,
        fresh, &scheduler, rig));
    const RigExecFrameCacheKey unfenced{7, 14};
    CHECK(RigExecPublishBackgroundCompletion(
        cache, unfenced, UsdTimeCode(6.0), pose, fresh,
        nullptr, rig));
    CHECK(cache->Lookup(unfenced, &served));
}

// TRIGGER. Commit enqueues neighbors plus sweep and never the playhead;
// idle enqueues the sweep only; a closed gate or null runner enqueues
// nothing. Runs last: the injected kernel's marker poses are retired
// before returning so no later scrub can serve one.
void
TestTriggersEnqueueNeighborsAndSweep()
{
    std::printf("progress: TestTriggersEnqueueNeighborsAndSweep\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));

    if (!RigExecBackgroundWarmingEnabled()) {
        // Fill off (warm-off, or parallel eval off): reads still serve,
        // triggers stay silent, production or injected alike.
        CHECK(registry.OnEditCommitted() == 0);
        CHECK(registry.OnIdle() == 0);
        CHECK(registry.OnEditCommitted(_TestKernel) == 0);
        CHECK(registry.OnIdle(_TestKernel) == 0);
        registry.Deactivate();
        return;
    }

    // No injected runner: the production runner warms the same burst shape
    // the injected kernel does below (a budgeted sweep slice on idle,
    // neighbors first on commit, never the playhead). Drained before the
    // kernel burst so the counts below start from an empty queue. The
    // default budget (16 invocations) binds both: idle takes 16 sweep
    // frames, commit takes 16 neighbors and defers the sweep to idle
    // ticks.
    CHECK(registry.OnIdle() == 16);
    registry.WaitUntilBackgroundIdle();
    CHECK(registry.OnEditCommitted() == 16);
    registry.WaitUntilBackgroundIdle();
    const RigExecBackgroundSchedulerStats before =
        registry.GetBackgroundStats();
    CHECK(before.queuedDepth == 0 && before.running == 0);

    // Idle first: the sweep slice only, 16 frames, no neighbors. Drained
    // before the commit so the second burst is also counted against an
    // empty queue: workers popping mid-trigger would otherwise let sweep
    // times re-enqueue as new jobs beside their running twins.
    CHECK(registry.OnIdle(_TestKernel) == 16);
    registry.WaitUntilBackgroundIdle();
    // Then commit: 16 neighbors (the budget defers the sweep to idle
    // ticks), and nothing for the playhead.
    CHECK(registry.OnEditCommitted(_TestKernel) == 16);
    registry.WaitUntilBackgroundIdle();
    const RigExecBackgroundSchedulerStats after =
        registry.GetBackgroundStats();
    CHECK(after.completed - before.completed == 32);
    CHECK(after.queuedDepth == 0 && after.running == 0);
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// TRIGGER. A drag at the playhead warms around it: commit from a preview
// excludes the playhead frame and samples the standing overrides.
void
TestCommitDuringPreviewExcludesPlayhead()
{
    std::printf("progress: TestCommitDuringPreviewExcludesPlayhead\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    if (!RigExecBackgroundWarmingEnabled()) {
        registry.Deactivate();
        return;
    }
    // Declare a one-scalar preview lane and push one sample: the bridge
    // now stands overrides on top of the authored frame.
    const int arity = registry.BeginPreview("/Asset/Rig/AlongX.avars:tx");
    CHECK(arity == 1);
    const double sample = 11.0;
    CHECK(registry.UpdatePreview(&sample, 1));
    // A commit still enqueues the budgeted burst around the playhead --
    // 16 neighbors -- and still nothing FOR it -- with the standing
    // overrides in the key.
    CHECK(registry.OnEditCommitted(_TestKernel) == 16);
    CHECK(registry.EndPreview());
    registry.WaitUntilBackgroundIdle();
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// PRODUCTION. The C-API trigger path -- no injected runner -- enqueues real
// jobs with sampled vectors through the production runner, which executes
// them against the session's frozen snapshot; the generation fence drops
// stale production work. This proves the poses, not just the pipeline:
// fresh production work publishes, the warmed frame serves with zero
// evaluator pulls, and the served generation matches a live evaluation.
void
TestProductionTriggerPathEnqueuesAndFences()
{
    std::printf("progress: TestProductionTriggerPathEnqueuesAndFences\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    if (!RigExecBackgroundWarmingEnabled()) {
        CHECK(registry.OnEditCommitted() == 0);
        CHECK(registry.OnIdle() == 0);
        registry.Deactivate();
        return;
    }

    // Unit level: production work builds (the chain-free rig samples fresh)
    // under the current generation and refuses a stale one outright.
    const RigExecFrameGeneration gen0 =
        registry.CurrentFrameGeneration(rig);
    RigExecWarmFactoryResult work0 = registry.BuildWarmWork(
        rig, UsdTimeCode(3.0), gen0, RigExecFrozenStepRunner());
    CHECK(static_cast<bool>(work0.work));
    registry.CancelFrameGeneration(rig);
    CHECK(registry.CurrentFrameGeneration(rig) == gen0 + 1);
    CHECK(!registry.BuildWarmWork(
        rig, UsdTimeCode(3.0), gen0, RigExecFrozenStepRunner()).work);

    // Run level: work sampled under the old token drops without publishing.
    const size_t publishedBefore =
        registry.GetFrameCacheStats(rig).published;
    RigExecImagingSnapshotConstPtr snapshotBefore =
        registry.GetStore()->Get();
    RigExecWarmRequest request;
    request.rig = rig;
    request.time = UsdTimeCode(3.0);
    request.priority = RigExecWarmPriority::Neighbor;
    request.generation = gen0;
    if (work0.work) {
        CHECK(work0.work(request) ==
              RigExecWarmOutcome::DeclinedGeneration);
    }
    CHECK(registry.GetFrameCacheStats(rig).published == publishedBefore);
    CHECK(registry.GetStore()->Get() == snapshotBefore);

    // Fresh production work builds under the new token, executes against
    // the session's snapshot, and publishes into the cache only -- the
    // live snapshot still stands.
    RigExecWarmFactoryResult work1 = registry.BuildWarmWork(
        rig, UsdTimeCode(3.0), gen0 + 1, RigExecFrozenStepRunner());
    CHECK(static_cast<bool>(work1.work));
    request.generation = gen0 + 1;
    if (work1.work) {
        CHECK(work1.work(request) == RigExecWarmOutcome::Published);
    }
    CHECK(registry.GetFrameCacheStats(rig).published == publishedBefore + 1);
    CHECK(registry.GetStore()->Get() == snapshotBefore);

    // The scrub to the warmed frame serves from cache with zero evaluator
    // pulls, and the served generation matches a live evaluation of the
    // same frame (forced live past an empty cache, from a different
    // history -- values agree regardless of which history ran them).
    {
        const size_t pullsBefore = registry.GetSessionEvaluationCount(rig);
        const size_t hitsBefore =
            registry.GetFrameCacheStats(rig).hits;
        CHECK(registry.SetTime(UsdTimeCode(3.0)));
        CHECK(registry.GetSessionEvaluationCount(rig) == pullsBefore);
        CHECK(registry.GetFrameCacheStats(rig).hits == hitsBefore + 1);
        const _GenerationGeometry warmed =
            _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!warmed.points.empty());
        registry.ClearFrameCache(rig);
        CHECK(registry.SetTime(UsdTimeCode(4.0)));
        CHECK(registry.SetTime(UsdTimeCode(3.0)));
        const _GenerationGeometry live =
            _CaptureGeometry(registry.GetStore()->Get());
        if (!_SameGeometry(warmed, live)) {
            std::printf("warmed generation differs from live at frame 3\n");
            CHECK(false);
        }
    }

    // Trigger level: the budgeted commit burst enqueues and drains through
    // the pool -- 16 neighbors, nothing for the playhead -- every job
    // completes, and every completion publishes.
    const RigExecBackgroundSchedulerStats before =
        registry.GetBackgroundStats();
    const size_t publishedBeforeBurst =
        registry.GetFrameCacheStats(rig).published;
    CHECK(registry.OnEditCommitted() == 16);
    registry.WaitUntilBackgroundIdle();
    const RigExecBackgroundSchedulerStats after =
        registry.GetBackgroundStats();
    CHECK(after.completed - before.completed == 16);
    CHECK(after.queuedDepth == 0 && after.running == 0);
    CHECK(registry.GetFrameCacheStats(rig).published ==
          publishedBeforeBurst + 16);
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// A rig the program cannot express: one provider's posed:space is
// CONNECTED, so its value is whatever an arbitrary exec computation says
// from the middle of the pose walk, and there is no epoch-constant summary
// of that to compile (see testRigExecBakedMode.cpp, whose in-memory fixture
// this is, animated). The joint's connection reads the control's posed:space
// attribute -- not its computed frame, and providers read no xformOp -- so
// the animated posed:space is what makes frames genuinely differ, and
// per-time memos cannot cross-serve undetected.
UsdStageRefPtr
MakeRefusalRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim root = stage->DefinePrim(SdfPath("/Asset/Rig/Root"),
                                           TfToken("RigExecControl"));
    root.GetAttribute(TfToken("avars:tx")).Set(3.0);
    UsdAttribute rootSpace = root.GetAttribute(TfToken("posed:space"));
    if (!rootSpace) {
        rootSpace = root.CreateAttribute(TfToken("posed:space"),
                                         SdfValueTypeNames->Matrix4d);
    }
    for (int t = 1; t <= 4; ++t) {
        GfMatrix4d space(1.0);
        space.SetTranslate(GfVec3d(10.0 * double(t), 0, 0));
        rootSpace.Set(space, UsdTimeCode(double(t)));
    }
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Bone"),
                                            TfToken("RigExecJoint"));
    joint.GetAttribute(TfToken("avars:ty")).Set(2.0);

    // The refusal. The connection is to the control's own posed:space, which
    // is the identity the joint would have composed anyway -- so the rig
    // still evaluates to a comparable pose, and the only thing the
    // connection changes is that a value which was an epoch constant is now
    // an exec answer.
    joint.CreateAttribute(TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
        .AddConnection(root.GetPath().AppendProperty(TfToken("posed:space")));

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Mesh"));
    VtVec3fArray points{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0)};
    mesh.GetAttribute(TfToken("points")).Set(points);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({joint.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(1);
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(VtIntArray{0, 0, 0});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 1.0f, 1.0f});
    return stage;
}

// D7 end to end: a bake-refusal rig memoizes the UI thread's own live
// evaluations and serves repeats with zero evaluator pulls, while no
// background job is ever created for it -- and a value edit retires the
// time-keyed memos, so the next visit evaluates live and serves the edited
// pose rather than a stale one.
void
TestRefusalRigMemoizesUiThreadResults()
{
    std::printf("progress: TestRefusalRigMemoizesUiThreadResults\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    const bool verify = RigExecFrameCacheVerifyRequested();
    const bool readsOn =
        RigExecFrameCacheModeFromEnvironment() != RigExecFrameCacheMode::Off;

    // The fixture still refuses the bake: without this, the "zero jobs"
    // assertions below would pass on a rig that warms.
    {
        UsdStageRefPtr probeStage = MakeRefusalRig();
        RigExecRigEvaluator probe(probeStage, SdfPath("/Asset/Rig"));
        probe.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::vector<std::string> errors;
        CHECK(probe.Compile(&errors));
        std::vector<std::string> reasons;
        CHECK(!probe.IsBakeable(&reasons));
        CHECK(!reasons.empty());
        bool named = false;
        for (const std::string &reason : reasons) {
            if (reason.find("connected posed:space") != std::string::npos) {
                named = true;
            }
        }
        CHECK(named);
    }

    UsdStageRefPtr stage = MakeRefusalRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    CHECK(registry.GetSessionEvaluationCount(rig) == 1);

    // No background job for a refusal rig, by every route: direct builds
    // (injected and production runners) decline, and both triggers enqueue
    // nothing while the pool sits idle.
    const RigExecFrameGeneration gen = registry.CurrentFrameGeneration(rig);
    const RigExecWarmFactoryResult refusedInjected =
        registry.BuildWarmWork(rig, UsdTimeCode(2.0), gen, _TestKernel);
    CHECK(!refusedInjected.work);
    CHECK(refusedInjected.skip == RigExecWarmSkipReason::D7Exempt);
    const RigExecWarmFactoryResult refusedProduction =
        registry.BuildWarmWork(
            rig, UsdTimeCode(2.0), gen, RigExecFrozenStepRunner());
    CHECK(!refusedProduction.work);
    CHECK(refusedProduction.skip == RigExecWarmSkipReason::D7Exempt);
    CHECK(!refusedProduction.detail.empty());
    CHECK(registry.GetLastWarmSkipReason(rig) ==
          RigExecWarmSkipReason::D7Exempt);
    CHECK(registry.GetLastWarmSkipDetail(rig) == refusedProduction.detail);
    const RigExecBackgroundSchedulerStats bgBefore =
        registry.GetBackgroundStats();
    CHECK(registry.OnEditCommitted() == 0);
    CHECK(registry.OnIdle() == 0);
    CHECK(registry.OnEditCommitted(_TestKernel) == 0);
    CHECK(registry.OnIdle(_TestKernel) == 0);
    const RigExecBackgroundSchedulerStats bgAfter =
        registry.GetBackgroundStats();
    CHECK(bgAfter.completed == bgBefore.completed);
    CHECK(bgAfter.queuedDepth == 0 && bgAfter.running == 0);

    // Cold pass: every new frame evaluates live and memoizes.
    std::map<double, _GenerationGeometry> firstPass;
    firstPass[1.0] = _CaptureGeometry(registry.GetStore()->Get());
    CHECK(!firstPass[1.0].points.empty());
    for (double frame : {2.0, 3.0, 4.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        firstPass[frame] = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!firstPass[frame].points.empty());
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == 4);
    // Sensitivity: frames genuinely differ, so a memo served at the wrong
    // frame would be caught by the geometry comparisons below.
    if (_SameGeometry(firstPass[1.0], firstPass[2.0])) {
        std::printf("refusal frames do not differ: the rig is static\n");
        CHECK(false);
    }
    const RigExecFrameCacheStats coldStats =
        registry.GetFrameCacheStats(rig);
    if (readsOn) {
        CHECK(coldStats.published == 4);
        CHECK(coldStats.hits == 0);
    } else {
        CHECK(coldStats.published == 0);
    }

    // Warm pass: revisits serve the memoized poses with zero pulls (under
    // verify, each hit is shadow-proven and counts a pull, but still hits).
    const size_t revisits = 6;
    for (double frame : {3.0, 2.0, 1.0, 2.0, 3.0, 4.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        const _GenerationGeometry now =
            _CaptureGeometry(registry.GetStore()->Get());
        if (!_SameGeometry(firstPass[frame], now)) {
            std::printf("refusal generation differs at frame %g\n", frame);
            CHECK(false);
        }
    }
    const size_t pulls = registry.GetSessionEvaluationCount(rig);
    const RigExecFrameCacheStats warmStats =
        registry.GetFrameCacheStats(rig);
    if (!readsOn) {
        CHECK(pulls == 4 + revisits);
    } else if (verify) {
        CHECK(pulls == 4 + revisits);
        CHECK(warmStats.hits == revisits);
    } else {
        CHECK(pulls == 4);
        CHECK(warmStats.hits == revisits);
        // No store miss was ever counted: the lookup runs only under a
        // freshness proof, and every proven lookup hit.
        CHECK(warmStats.misses == 0);
    }

    // An edit retires the refusal memos: the stage-edit serial folded into
    // the digest moves at every frame, so the next visit evaluates live,
    // serves the edited pose, and memoizes it again.
    if (readsOn && !verify) {
        const RigExecFrameGeneration genBefore =
            registry.CurrentFrameGeneration(rig);
        const size_t pullsBefore = registry.GetSessionEvaluationCount(rig);
        UsdAttribute rootSpace = stage->GetAttributeAtPath(
            SdfPath("/Asset/Rig/Root.posed:space"));
        CHECK(rootSpace);
        GfMatrix4d editedSpace(1.0);
        editedSpace.SetTranslate(GfVec3d(999.0, 0, 0));
        rootSpace.Set(editedSpace, UsdTimeCode(2.0));
        CHECK(registry.CurrentFrameGeneration(rig) == genBefore + 1);
        // The edit re-evaluated live at the playhead (frame 4).
        CHECK(registry.GetSessionEvaluationCount(rig) == pullsBefore + 1);
        // A previously-visited frame evaluates live -- not a stale memo --
        // and serves the edited pose.
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.GetSessionEvaluationCount(rig) == pullsBefore + 2);
        const _GenerationGeometry edited =
            _CaptureGeometry(registry.GetStore()->Get());
        if (_SameGeometry(firstPass[2.0], edited)) {
            std::printf("refusal edit did not move the frame-2 pose\n");
            CHECK(false);
        }
        // And the edited pose memoizes: a scrub away and back is free.
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        const size_t pullsAt1 = registry.GetSessionEvaluationCount(rig);
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.GetSessionEvaluationCount(rig) == pullsAt1);
        const _GenerationGeometry again =
            _CaptureGeometry(registry.GetStore()->Get());
        if (!_SameGeometry(edited, again)) {
            std::printf("refusal re-memoized pose differs at frame 2\n");
            CHECK(false);
        }
    }
    registry.Deactivate();
}

// SERVED. A background completion plus its enqueue-time proof serves without
// evaluating: the warmed pose reaches the viewport with zero evaluator pulls.
// Fails without the proof (the lookup misses and evaluates live); the
// injected kernel stands in for the frozen executor's pose here, so the
// assertion is the pipeline -- proof plus entry plus hit -- not the pixels.
void
TestWarmedCompletionServesWithoutEvaluating()
{
    std::printf("progress: TestWarmedCompletionServesWithoutEvaluating\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    const bool verify = RigExecFrameCacheVerifyRequested();
    const bool readsOn =
        RigExecFrameCacheModeFromEnvironment() != RigExecFrameCacheMode::Off;
    if (!readsOn) {
        return;
    }
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    CHECK(registry.GetSessionEvaluationCount(rig) == 1);

    // Build (recording the proof) and run (publishing the completion) one
    // warming job for an unvisited frame, exactly as a worker would.
    RigExecWarmFactoryResult work = registry.BuildWarmWork(
        rig, UsdTimeCode(3.0), registry.CurrentFrameGeneration(rig),
        _TestKernel);
    CHECK(static_cast<bool>(work.work));
    RigExecWarmRequest request;
    request.rig = rig;
    request.time = UsdTimeCode(3.0);
    request.priority = RigExecWarmPriority::Neighbor;
    request.generation = registry.CurrentFrameGeneration(rig);
    const size_t publishedBefore =
        registry.GetFrameCacheStats(rig).published;
    if (work.work) {
        CHECK(work.work(request) == RigExecWarmOutcome::Published);
    }
    CHECK(registry.GetFrameCacheStats(rig).published == publishedBefore + 1);

    // The scrub to the warmed frame serves from cache: under verify the hit
    // is shadow-proven (one pull either way -- the marker mismatches live
    // and the entry repairs) and otherwise it is free.
    const size_t pullsBefore = registry.GetSessionEvaluationCount(rig);
    const size_t hitsBefore = registry.GetFrameCacheStats(rig).hits;
    CHECK(registry.SetTime(UsdTimeCode(3.0)));
    CHECK(registry.GetFrameCacheStats(rig).hits == hitsBefore + 1);
    if (verify) {
        CHECK(registry.GetSessionEvaluationCount(rig) == pullsBefore + 1);
    } else {
        CHECK(registry.GetSessionEvaluationCount(rig) == pullsBefore);
    }
    // The injected kernel's marker served above: retire it.
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// INTERACTIVE. A drag at the playhead bypasses the frame cache: every tick
// carries unique overrides, so a lookup would always miss after paying a
// full sample+digest, and memoization would store single-use entries no
// scrub can reach -- pure overhead on the hottest path (D5: the playhead is
// always live). While overrides stand, evaluations publish live with zero
// cache reads AND zero writes; releasing the drag resumes memoization.
void
TestInteractiveDragBypassesCache()
{
    std::printf("progress: TestInteractiveDragBypassesCache\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    UsdStageRefPtr stage = MakeTinyRig();
    RigExecImagingBridge bridge(stage, SdfPath("/Asset/Rig"));
    CHECK(bridge.Compile());

    // The authored frame memoizes: the control case.
    const RigExecImagingBridge::PublishResult first =
        bridge.EvaluateAndPublishResult(UsdTimeCode(2.0));
    CHECK(first.ok && !first.cacheHit);
    CHECK(bridge.GetFrameCacheStats().published == 1);

    // A drag tick at the same playhead: live, uncached, and honoring the
    // override (the dragged pose differs from the authored one).
    const SdfPath alongX("/Asset/Rig/AlongX");
    bridge.SetInteractiveOverrides({RigExecValueOverride{
        alongX, TfToken(), TfToken("avars:tx"), VtValue(999.0)}});
    const _GenerationGeometry authored =
        _CaptureGeometry(bridge.GetStore()->Get());
    const RigExecImagingBridge::PublishResult drag1 =
        bridge.EvaluateAndPublishResult(UsdTimeCode(2.0));
    CHECK(drag1.ok && !drag1.cacheHit);
    const _GenerationGeometry dragged =
        _CaptureGeometry(bridge.GetStore()->Get());
    CHECK(!_SameGeometry(authored, dragged));

    // A second tick with a new value: still live, still uncached.
    bridge.SetInteractiveOverrides({RigExecValueOverride{
        alongX, TfToken(), TfToken("avars:tx"), VtValue(-999.0)}});
    const RigExecImagingBridge::PublishResult drag2 =
        bridge.EvaluateAndPublishResult(UsdTimeCode(2.0));
    CHECK(drag2.ok && !drag2.cacheHit);

    // Zero cache activity across both drag ticks: no reads, no writes.
    const RigExecFrameCacheStats during = bridge.GetFrameCacheStats();
    CHECK(during.published == 1);
    CHECK(during.hits == 0 && during.misses == 0);

    // Releasing the drag resumes memoization: the authored frame hits the
    // entry the first publication stored.
    bridge.ClearInteractiveOverrides();
    const RigExecImagingBridge::PublishResult released =
        bridge.EvaluateAndPublishResult(UsdTimeCode(2.0));
    CHECK(released.ok && released.cacheHit);
    const _GenerationGeometry served =
        _CaptureGeometry(bridge.GetStore()->Get());
    CHECK(_SameGeometry(authored, served));
}

// LANES. The frameCache profiler lane records production lookups: a cold
// frame (no store consultation) records nothing, a hit records cacheHit,
// and a proven lookup that misses the store records cacheMiss.
void
TestProfilerFrameCacheLane()
{
    std::printf("progress: TestProfilerFrameCacheLane\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    UsdStageRefPtr stage = MakeTinyRig();
    RigExecImagingBridge bridge(stage, SdfPath("/Asset/Rig"));
    CHECK(bridge.Compile());
    bridge.MutableProfiler()->SetEnabled(true);

    // Cold: the lookup never reaches the store (no proof), so the lane
    // stays empty.
    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(2.0)).ok);
    // Warm: one store consultation, one hit.
    const RigExecImagingBridge::PublishResult warm =
        bridge.EvaluateAndPublishResult(UsdTimeCode(2.0));
    CHECK(warm.ok && warm.cacheHit);
    // Evicted under a standing proof: the consultation misses.
    bridge.GetFrameCache()->SetByteCap(0);
    const RigExecImagingBridge::PublishResult missed =
        bridge.EvaluateAndPublishResult(UsdTimeCode(2.0));
    CHECK(missed.ok && !missed.cacheHit);

    std::vector<std::pair<std::string, double>> lane;
    for (const RigExecProfileEvent &event :
         bridge.GetProfiler().GetEvents()) {
        if (event.kind == RigExecProfileEventKind::Instant &&
            event.category == kRigExecProfileCategoryFrameCache) {
            lane.push_back(
                {event.name, std::stod(event.args.at("frame"))});
        }
    }
    CHECK(lane.size() == 2);
    if (lane.size() == 2) {
        CHECK(lane[0].first == "cacheHit" && lane[0].second == 2.0);
        CHECK(lane[1].first == "cacheMiss" && lane[1].second == 2.0);
    }
}

// BURST. The standing burst serves a second consecutive trigger with no
// rebuild: one warmBurstRebuild instant across both, usable stays set.
void
TestStandingBurstSurvivesSecondIdle()
{
    std::printf("progress: TestStandingBurstSurvivesSecondIdle\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));

    if (!RigExecBackgroundWarmingEnabled()) {
        CHECK(registry.OnIdle() == 0);
        CHECK(registry.OnIdle() == 0);
        registry.Deactivate();
        return;
    }
    RigExecProfiler *profiler = registry.MutableBridgeProfiler(rig);
    CHECK(profiler != nullptr);
    if (profiler) {
        profiler->SetEnabled(true);
    }
    registry.OnIdle();
    registry.WaitUntilBackgroundIdle();
    CHECK(registry.GetWarmBurstUsable(rig));
    registry.OnIdle();
    registry.WaitUntilBackgroundIdle();
    CHECK(registry.GetWarmBurstUsable(rig));
    size_t rebuilds = 0;
    if (profiler) {
        for (const RigExecProfileEvent &event : profiler->GetEvents()) {
            if (event.kind == RigExecProfileEventKind::Instant &&
                event.name == "warmBurstRebuild") {
                ++rebuilds;
                CHECK(event.args.at("usable") == "1");
            }
        }
    }
    CHECK(rebuilds == 1);
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// BURST. An over-slice prep parks the burst unusable and latches: the
// tick takes the plain per-frame route (same burst shape as the cached
// route), the doomed rebuild is not repaid while the pins stand, and
// restoring the slice unlatches the overrun.
void
TestOverSlicePrepTakesPlainRoute()
{
    std::printf("progress: TestOverSlicePrepTakesPlainRoute\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));

    if (!RigExecBackgroundWarmingEnabled()) {
        CHECK(registry.OnIdle(_TestKernel) == 0);
        registry.Deactivate();
        return;
    }
    RigExecProfiler *profiler = registry.MutableBridgeProfiler(rig);
    CHECK(profiler != nullptr);
    if (profiler) {
        profiler->SetEnabled(true);
    }
    const double savedSlice = registry.GetWarmBurstPrepSliceMs();
    registry.SetWarmBurstPrepSliceMs(-1.0);
    // Plain route, same budgeted shape the cached route enqueues: 16.
    CHECK(registry.OnIdle(_TestKernel) == 16);
    registry.WaitUntilBackgroundIdle();
    CHECK(!registry.GetWarmBurstUsable(rig));
    // Second tick under the same pins: the latched overrun re-serves, no
    // second rebuild.
    registry.OnIdle(_TestKernel);
    registry.WaitUntilBackgroundIdle();
    CHECK(!registry.GetWarmBurstUsable(rig));
    size_t rebuilds = 0;
    if (profiler) {
        for (const RigExecProfileEvent &event : profiler->GetEvents()) {
            if (event.kind == RigExecProfileEventKind::Instant &&
                event.name == "warmBurstRebuild") {
                ++rebuilds;
                CHECK(event.args.at("usable") == "0");
                CHECK(event.args.at("overrun") == "1");
            }
        }
    }
    CHECK(rebuilds == 1);
    // Pins unchanged, slice restored: the overrun unlatches, the next
    // trigger rebuilds, and the burst is usable again.
    registry.SetWarmBurstPrepSliceMs(savedSlice);
    registry.OnIdle(_TestKernel);
    registry.WaitUntilBackgroundIdle();
    CHECK(registry.GetWarmBurstUsable(rig));
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// SCOPES. The 1.2 profiler scopes land with non-zero totals: memoize
// sampling+digest and burst prep on the bridge profiler, per-sweep-time
// factory cost on the scheduler profiler.
void
TestProfilerWarmScopes()
{
    std::printf("progress: TestProfilerWarmScopes\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    RigExecProfiler *bridgeProfiler = registry.MutableBridgeProfiler(rig);
    RigExecProfiler *schedulerProfiler =
        registry.MutableSchedulerProfiler();
    CHECK(bridgeProfiler != nullptr);
    CHECK(schedulerProfiler != nullptr);
    if (bridgeProfiler) {
        bridgeProfiler->SetEnabled(true);
    }
    if (schedulerProfiler) {
        schedulerProfiler->SetEnabled(true);
    }

    // Cold SetTime evaluates live and memoizes; the idle trigger prepares
    // the burst and samples the sweep through the factory.
    CHECK(registry.SetTime(UsdTimeCode(1.0)));
    const bool warming = RigExecBackgroundWarmingEnabled();
    if (warming) {
        CHECK(registry.OnIdle(_TestKernel) == 16);
        registry.WaitUntilBackgroundIdle();
    }
    const auto findRow = [](const RigExecProfiler &profiler,
                            const char *name) {
        for (const RigExecProfileSummaryRow &row : profiler.Summarize()) {
            if (row.name == name) {
                return row;
            }
        }
        return RigExecProfileSummaryRow();
    };
    const bool cacheOn = RigExecFrameCacheModeFromEnvironment() !=
        RigExecFrameCacheMode::Off;
    if (bridgeProfiler) {
        const RigExecProfileSummaryRow memoize =
            findRow(*bridgeProfiler, "Imaging.MemoizeSampleDigest");
        if (cacheOn) {
            CHECK(memoize.count > 0 && memoize.totalUs > 0);
        }
        const RigExecProfileSummaryRow prep =
            findRow(*bridgeProfiler, "Imaging.PrepareWarmBurst");
        if (warming) {
            CHECK(prep.count > 0 && prep.totalUs > 0);
        }
    }
    if (schedulerProfiler && warming) {
        const RigExecProfileSummaryRow factory =
            findRow(*schedulerProfiler, "Scheduler.WarmFactorySample");
        CHECK(factory.count > 0 && factory.totalUs > 0);
    }
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// FAST. A servable index completion serves without sampling: revisits over
// memoized frames hit with zero pulls, zero lookup-sample runs, and
// bit-identical generations.
void
TestCachedServeSkipsLookupSample()
{
    std::printf("progress: TestCachedServeSkipsLookupSample\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    std::map<double, _GenerationGeometry> firstPass;
    firstPass[1.0] = _CaptureGeometry(registry.GetStore()->Get());
    CHECK(registry.SetTime(UsdTimeCode(2.0)));
    firstPass[2.0] = _CaptureGeometry(registry.GetStore()->Get());
    // The fast path's precondition, explicit: both frames read Cached,
    // so a lookup sample below would be pure overhead, not a fallback.
    {
        const std::vector<RigExecWarmFrameState> states =
            registry.GetFrameStates(rig, {1.0, 2.0});
        CHECK(states.size() == 2);
        if (states.size() == 2) {
            CHECK(states[0] == RigExecWarmFrameState::Cached);
            CHECK(states[1] == RigExecWarmFrameState::Cached);
        }
    }
    RigExecProfiler *profiler = registry.MutableBridgeProfiler(rig);
    CHECK(profiler != nullptr);
    if (profiler) {
        profiler->SetEnabled(true);
    }
    const auto scopeCount = [](const RigExecProfiler &profiler,
                               const char *name) {
        for (const RigExecProfileSummaryRow &row : profiler.Summarize()) {
            if (row.name == name) {
                return row.count;
            }
        }
        return size_t(0);
    };
    const size_t pulls = registry.GetSessionEvaluationCount(rig);
    const size_t hits = registry.GetFrameCacheStats(rig).hits;
    const size_t samples =
        profiler ? scopeCount(*profiler, "Imaging.LookupSampleDigest") : 0;
    const size_t memoizes =
        profiler ? scopeCount(*profiler, "Imaging.MemoizeSampleDigest") : 0;
    CHECK(registry.SetTime(UsdTimeCode(1.0)));
    if (!_SameGeometry(firstPass[1.0],
                        _CaptureGeometry(registry.GetStore()->Get()))) {
        std::printf("fast-path generation differs at frame 1\n");
        CHECK(false);
    }
    CHECK(registry.SetTime(UsdTimeCode(2.0)));
    if (!_SameGeometry(firstPass[2.0],
                        _CaptureGeometry(registry.GetStore()->Get()))) {
        std::printf("fast-path generation differs at frame 2\n");
        CHECK(false);
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == pulls);
    CHECK(registry.GetFrameCacheStats(rig).hits == hits + 2);
    if (profiler) {
        CHECK(scopeCount(*profiler, "Imaging.LookupSampleDigest") == samples);
        CHECK(scopeCount(*profiler, "Imaging.MemoizeSampleDigest") ==
              memoizes);
    }
    registry.Deactivate();
}

// INDEX. The warm-frame index, driven directly: completions under the
// query generation+epoch read cached; older generations or epochs read
// dirty; evictions retire only their own key; queue transitions drive
// warming; reset clears.
void
TestWarmFrameIndexStates()
{
    std::printf("progress: TestWarmFrameIndexStates\n");
    std::fflush(stdout);
    RigExecWarmFrameIndex index;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameCacheKey key{7, 9};
    CHECK(index.State(rig, 2.0, 0, 7) == RigExecWarmFrameState::Uncached);
    index.NoteGeneration(rig, 3);
    CHECK(index.CurrentGeneration(rig) == 3);
    index.NoteCompleted(rig, 2.0, key, 3);
    CHECK(index.State(rig, 2.0, 3, 7) == RigExecWarmFrameState::Cached);
    CHECK(index.State(rig, 2.0, 2, 7) == RigExecWarmFrameState::Dirty);
    CHECK(index.State(rig, 2.0, 3, 8) == RigExecWarmFrameState::Dirty);
    // A drained old key retires nothing; the recorded key retires.
    index.NoteEvicted(rig, RigExecFrameCacheKey{7, 10}, 2.0);
    CHECK(index.State(rig, 2.0, 3, 7) == RigExecWarmFrameState::Cached);
    index.NoteEvicted(rig, key, 2.0);
    CHECK(index.State(rig, 2.0, 3, 7) == RigExecWarmFrameState::Uncached);
    // Republish, then queue visibility beats cached.
    index.NoteCompleted(rig, 2.0, key, 3);
    RigExecWarmTransition queued;
    queued.rig = rig;
    queued.timeValue = 2.0;
    queued.kind = RigExecWarmTransitionKind::Queued;
    index.NoteTransition(queued);
    CHECK(index.State(rig, 2.0, 3, 7) == RigExecWarmFrameState::Warming);
    RigExecWarmTransition running = queued;
    running.kind = RigExecWarmTransitionKind::Running;
    index.NoteTransition(running);
    CHECK(index.State(rig, 2.0, 3, 7) == RigExecWarmFrameState::Warming);
    RigExecWarmTransition finished = queued;
    finished.kind = RigExecWarmTransitionKind::Finished;
    finished.outcome = RigExecWarmOutcome::Published;
    index.NoteTransition(finished);
    CHECK(index.State(rig, 2.0, 3, 7) == RigExecWarmFrameState::Cached);
    // Cancel clears queued; the completion stands.
    index.NoteTransition(queued);
    RigExecWarmTransition canceled = queued;
    canceled.kind = RigExecWarmTransitionKind::Canceled;
    index.NoteTransition(canceled);
    CHECK(index.State(rig, 2.0, 3, 7) == RigExecWarmFrameState::Cached);
    // Shed, shutdown, and stale drops clear queue-only records.
    RigExecWarmTransition shed;
    shed.rig = rig;
    shed.timeValue = 4.0;
    shed.kind = RigExecWarmTransitionKind::Queued;
    index.NoteTransition(shed);
    CHECK(index.State(rig, 4.0, 3, 7) == RigExecWarmFrameState::Warming);
    shed.kind = RigExecWarmTransitionKind::Shed;
    index.NoteTransition(shed);
    CHECK(index.State(rig, 4.0, 3, 7) == RigExecWarmFrameState::Uncached);
    RigExecWarmTransition dropped = shed;
    dropped.timeValue = 5.0;
    dropped.kind = RigExecWarmTransitionKind::Queued;
    index.NoteTransition(dropped);
    dropped.kind = RigExecWarmTransitionKind::DroppedAtShutdown;
    index.NoteTransition(dropped);
    CHECK(index.State(rig, 5.0, 3, 7) == RigExecWarmFrameState::Uncached);
    RigExecWarmTransition stale = shed;
    stale.timeValue = 6.0;
    stale.kind = RigExecWarmTransitionKind::Queued;
    index.NoteTransition(stale);
    stale.kind = RigExecWarmTransitionKind::Running;
    index.NoteTransition(stale);
    stale.kind = RigExecWarmTransitionKind::DroppedStale;
    index.NoteTransition(stale);
    CHECK(index.State(rig, 6.0, 3, 7) == RigExecWarmFrameState::Uncached);
    // Batch query keeps order.
    index.NoteCompleted(rig, 1.0, key, 3);
    const std::vector<RigExecWarmFrameState> states =
        index.States(rig, {1.0, 2.0, 3.0}, 3, 7);
    CHECK(states.size() == 3);
    if (states.size() == 3) {
        CHECK(states[0] == RigExecWarmFrameState::Cached);
        CHECK(states[1] == RigExecWarmFrameState::Cached);
        CHECK(states[2] == RigExecWarmFrameState::Uncached);
    }
    index.ResetRig(rig);
    CHECK(index.State(rig, 1.0, 3, 7) == RigExecWarmFrameState::Uncached);
    CHECK(index.State(rig, 2.0, 3, 7) == RigExecWarmFrameState::Uncached);
}

// EVICT. The cache reports every drop -- single, epoch, cap, clear -- with
// the key and time the index retires on.
void
TestFrameCacheEvictionCallbackReportsDrops()
{
    std::printf("progress: TestFrameCacheEvictionCallbackReportsDrops\n");
    std::fflush(stdout);
    RigExecFrameCache cache;
    std::vector<RigExecFrameCacheEviction> reported;
    cache.SetEvictionCallback(
        [&reported](const std::vector<RigExecFrameCacheEviction> &evicted) {
            reported.insert(reported.end(), evicted.begin(), evicted.end());
        });
    const auto publish = [&cache](uint64_t epoch, uint64_t control,
                                  double time) {
        RigExecRigPose pose;
        pose.valid = true;
        pose.time = UsdTimeCode(time);
        const RigExecFrameCacheKey key{epoch, control};
        CHECK(cache.Publish(key, UsdTimeCode(time), pose));
    };
    publish(7, 1, 1.0);
    publish(7, 2, 2.0);
    publish(8, 3, 3.0);
    CHECK(reported.empty());
    CHECK(cache.Evict((RigExecFrameCacheKey{7, 1})));
    CHECK(reported.size() == 1);
    if (!reported.empty()) {
        CHECK(reported[0].key == (RigExecFrameCacheKey{7, 1}));
        CHECK(reported[0].time == UsdTimeCode(1.0));
    }
    CHECK(cache.EvictEpoch(7) == 1);
    CHECK(reported.size() == 2);
    if (reported.size() == 2) {
        CHECK(reported[1].key == (RigExecFrameCacheKey{7, 2}));
        CHECK(reported[1].time == UsdTimeCode(2.0));
    }
    // A miss reports nothing.
    CHECK(!cache.Evict((RigExecFrameCacheKey{7, 1})));
    CHECK(cache.EvictEpoch(9) == 0);
    CHECK(reported.size() == 2);
    // The cap pass and Clear report the rest.
    cache.SetByteCap(0);
    CHECK(reported.size() == 3);
    if (reported.size() == 3) {
        CHECK(reported[2].key == (RigExecFrameCacheKey{8, 3}));
    }
    cache.SetByteCap(kRigExecFrameCacheDefaultByteCap);
    publish(8, 4, 4.0);
    cache.Clear();
    CHECK(reported.size() == 4);
    if (reported.size() == 4) {
        CHECK(reported[3].key == (RigExecFrameCacheKey{8, 4}));
        CHECK(reported[3].time == UsdTimeCode(4.0));
    }
    // Uninstalled: drops stay silent.
    cache.SetEvictionCallback(RigExecFrameCacheEvictionCallback());
    publish(8, 5, 5.0);
    CHECK(cache.Evict((RigExecFrameCacheKey{8, 5})));
    CHECK(reported.size() == 4);
}

// STATES. The scripted SetTime/warm/drain sequence reads through the
// per-frame API: memoized frames cached, warmed frames cached, edited
// frames dirty, re-warmed cached, cleared uncached. The C API agrees.
void
TestFrameStatesScriptedSequence()
{
    std::printf("progress: TestFrameStatesScriptedSequence\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    const std::vector<double> frames{1.0, 2.0, 3.0, 4.0};
    const auto expect = [&](RigExecWarmFrameState a, RigExecWarmFrameState b,
                            RigExecWarmFrameState c, RigExecWarmFrameState d) {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, frames);
        return got.size() == 4 && got[0] == a && got[1] == b && got[2] == c &&
            got[3] == d;
    };
    const RigExecWarmFrameState uncached = RigExecWarmFrameState::Uncached;
    const RigExecWarmFrameState cached = RigExecWarmFrameState::Cached;
    const RigExecWarmFrameState dirty = RigExecWarmFrameState::Dirty;
    // Activation evaluates the playhead, memoizing frame 2 when reads run.
    const bool readsOn = RigExecFrameCacheModeFromEnvironment() !=
        RigExecFrameCacheMode::Off;
    if (readsOn) {
        CHECK(expect(uncached, cached, uncached, uncached));
    } else {
        CHECK(expect(uncached, uncached, uncached, uncached));
        registry.Deactivate();
        return;
    }
    if (!RigExecBackgroundWarmingEnabled()) {
        registry.Deactivate();
        return;
    }
    CHECK(registry.OnEditCommitted(_TestKernel) == 16);
    registry.WaitUntilBackgroundIdle();
    CHECK(expect(cached, cached, cached, cached));
    // The C API agrees, batched; bad arguments answer -1.
    {
        int out[4] = {-1, -1, -1, -1};
        CHECK(RigExecImaging_GetFrameStates("/Asset/Rig", frames.data(), out,
                                            4) == 4);
        CHECK(out[0] == 2 && out[1] == 2 && out[2] == 2 && out[3] == 2);
        CHECK(RigExecImaging_GetFrameStates(nullptr, frames.data(), out, 4) ==
              -1);
        CHECK(RigExecImaging_GetFrameStates("/Asset/Rig", nullptr, out, 4) ==
              -1);
        CHECK(RigExecImaging_GetFrameStates("/Asset/Rig", frames.data(),
                                            nullptr, 4) == -1);
        CHECK(RigExecImaging_GetFrameStates("/Asset/Rig", frames.data(), out,
                                            -1) == -1);
        CHECK(RigExecImaging_GetFrameStates("not a path", frames.data(), out,
                                            4) == -1);
        CHECK(RigExecImaging_GetFrameStates("/Nope", frames.data(), out, 4) ==
              -1);
    }
    // A control edit: the generation moves and the playhead re-memoizes
    // under it while the other frames go dirty.
    UsdAttribute tx = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/AlongX.avars:tx"));
    CHECK(tx);
    tx.Set(99.0, UsdTimeCode(2.0));
    CHECK(expect(dirty, cached, dirty, dirty));
    // Re-warm under the new generation: cached again.
    CHECK(registry.OnEditCommitted(_TestKernel) == 16);
    registry.WaitUntilBackgroundIdle();
    CHECK(expect(cached, cached, cached, cached));
    // Clear retires everything.
    CHECK(RigExecImaging_ClearFrameCache("/Asset/Rig") == 0);
    CHECK(expect(uncached, uncached, uncached, uncached));
    CHECK(RigExecImaging_ClearFrameCache(nullptr) == -1);
    CHECK(RigExecImaging_ClearFrameCache("/Nope") == 0);
    registry.Deactivate();
}

// STATES. An eviction retires the index record: after the capture index
// drops the old epoch, the retired frame reads uncached (not dirty) while
// the fresh playhead memo stands cached.
void
TestFrameStatesEvictionRetires()
{
    std::printf("progress: TestFrameStatesEvictionRetires\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    const SdfPath yPath("/Asset/Rig/AlongY");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    CHECK(registry.SetTime(UsdTimeCode(2.0)));
    const std::vector<double> frames{1.0, 2.0};
    const auto states = [&]() {
        return registry.GetFrameStates(rig, frames);
    };
    {
        const std::vector<RigExecWarmFrameState> got = states();
        CHECK(got.size() == 2);
        if (got.size() == 2) {
            CHECK(got[0] == RigExecWarmFrameState::Cached);
            CHECK(got[1] == RigExecWarmFrameState::Cached);
        }
    }
    // A default-value edit on a captured avar: the scoped notice dirties
    // the old frames and re-memoizes the playhead.
    stage->GetPrimAtPath(yPath).GetAttribute(TfToken("avars:ty")).Set(5.0);
    {
        const std::vector<RigExecWarmFrameState> got = states();
        CHECK(got.size() == 2);
        if (got.size() == 2) {
            CHECK(got[0] == RigExecWarmFrameState::Dirty);
            CHECK(got[1] == RigExecWarmFrameState::Cached);
        }
    }
    registry.Deactivate();
}

// BUDGET. The default sampling budget shapes triggers (idle: a 16-frame
// sweep slice; commit: 16 neighbors, the sweep deferred to idle ticks);
// explicit budgets bind per trigger; a zero millisecond stop samples
// nothing.
void
TestSamplingBudgetShapesTriggers()
{
    std::printf("progress: TestSamplingBudgetShapesTriggers\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    if (!RigExecBackgroundWarmingEnabled()) {
        registry.Deactivate();
        return;
    }
    CHECK(registry.GetWarmSamplingMaxInvocations() == 16);
    CHECK(registry.OnIdle(_TestKernel) == 16);
    registry.WaitUntilBackgroundIdle();
    CHECK(registry.OnEditCommitted(_TestKernel) == 16);
    registry.WaitUntilBackgroundIdle();
    // Explicit: four invocations, then the trigger stops.
    const size_t savedInvocations = registry.GetWarmSamplingMaxInvocations();
    const double savedMs = registry.GetWarmSamplingMaxMs();
    registry.SetWarmSamplingBudget(4, savedMs);
    CHECK(registry.OnIdle(_TestKernel) == 4);
    registry.WaitUntilBackgroundIdle();
    // A zero millisecond stop samples nothing (deterministic).
    registry.SetWarmSamplingBudget(savedInvocations, 0.0);
    const size_t invocationsBefore =
        registry.GetBackgroundStats().factoryInvocations;
    CHECK(registry.OnIdle(_TestKernel) == 0);
    CHECK(registry.GetBackgroundStats().factoryInvocations ==
          invocationsBefore);
    registry.SetWarmSamplingBudget(savedInvocations, savedMs);
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// CURSOR. A set warm range replaces the default sweep: the cursor visits
// closest-first from the playhead (ties prefer the future), and visited
// frames are skipped without sampling -- no factory invocations for a
// fully-visited range.
void
TestWarmRangeCursorSkipsVisited()
{
    std::printf("progress: TestWarmRangeCursorSkipsVisited\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    const bool readsOn = RigExecFrameCacheModeFromEnvironment() !=
        RigExecFrameCacheMode::Off;
    if (!readsOn || !RigExecBackgroundWarmingEnabled()) {
        registry.Deactivate();
        return;
    }
    CHECK(registry.SetWarmRange(rig, {1.0, 2.0, 3.0, 4.0}));
    // Frames 1, 3, 4 (the playhead never queues); 2 memoized at activation.
    CHECK(registry.OnIdle(_TestKernel) == 3);
    registry.WaitUntilBackgroundIdle();
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0, 4.0});
        CHECK(got.size() == 4);
        if (got.size() == 4) {
            CHECK(got[0] == RigExecWarmFrameState::Cached);
            CHECK(got[1] == RigExecWarmFrameState::Cached);
            CHECK(got[2] == RigExecWarmFrameState::Cached);
            CHECK(got[3] == RigExecWarmFrameState::Cached);
        }
    }
    // All visited now: the cursor finds nothing, sampling nothing.
    const size_t invocationsBefore =
        registry.GetBackgroundStats().factoryInvocations;
    CHECK(registry.OnIdle(_TestKernel) == 0);
    CHECK(registry.GetBackgroundStats().factoryInvocations ==
          invocationsBefore);
    // Closest-first with future-first ties: budget 1 over a cleared cache
    // warms 3.0 (|3-2| == |1-2|, future wins) and nothing else.
    registry.ClearFrameCache(rig);
    CHECK(registry.SetWarmRange(rig, {1.0, 2.0, 3.0, 4.0}));
    const size_t savedInvocations = registry.GetWarmSamplingMaxInvocations();
    const double savedMs = registry.GetWarmSamplingMaxMs();
    registry.SetWarmSamplingBudget(1, savedMs);
    CHECK(registry.OnIdle(_TestKernel) == 1);
    registry.WaitUntilBackgroundIdle();
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0, 4.0});
        CHECK(got.size() == 4);
        if (got.size() == 4) {
            CHECK(got[0] == RigExecWarmFrameState::Uncached);
            CHECK(got[1] == RigExecWarmFrameState::Uncached);
            CHECK(got[2] == RigExecWarmFrameState::Cached);
            CHECK(got[3] == RigExecWarmFrameState::Uncached);
        }
    }
    registry.SetWarmSamplingBudget(savedInvocations, savedMs);
    // Unknown and playback rigs refuse a range; the C API agrees.
    CHECK(!registry.SetWarmRange(SdfPath("/Nope"), {1.0}));
    CHECK(RigExecImaging_WarmRange("/Asset/Rig", nullptr, 0) == 0);
    CHECK(RigExecImaging_WarmRange(nullptr, nullptr, 0) == -1);
    CHECK(RigExecImaging_WarmRange("/Nope", nullptr, 0) == -1);
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// CURSOR. A refusal (D7) rig never counts visited: no factory, no
// completions, no invocations -- across repeated triggers over a set range.
// (Frame 2.0 reads Cached: Activate's initial evaluation memoized it
// through the live path, a genuine cached frame for the strip. The null
// factory takes no per-frame verdicts, so no streaks advance and the
// cursor keeps proposing; each trigger instead records the session-level
// D7Exempt skip, naming the cause without warming anything.)
void
TestRefusalRigNeverCountsVisited()
{
    std::printf("progress: TestRefusalRigNeverCountsVisited\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    UsdStageRefPtr stage = MakeRefusalRig();
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    if (!RigExecBackgroundWarmingEnabled()) {
        registry.Deactivate();
        return;
    }
    CHECK(registry.SetWarmRange(rig, {1.0, 2.0, 3.0, 4.0}));
    const size_t invocationsBefore =
        registry.GetBackgroundStats().factoryInvocations;
    for (int i = 0; i < 5; ++i) {
        CHECK(registry.OnIdle(_TestKernel) == 0);
        registry.WaitUntilBackgroundIdle();
    }
    CHECK(registry.GetBackgroundStats().factoryInvocations ==
          invocationsBefore);
    // The session-level verdict: a factory-less trigger names its cause.
    CHECK(registry.GetLastWarmSkipReason(rig) ==
          RigExecWarmSkipReason::D7Exempt);
    CHECK(!registry.GetLastWarmSkipDetail(rig).empty());
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0, 4.0});
        CHECK(got.size() == 4);
        CHECK(got[0] == RigExecWarmFrameState::Uncached);
        CHECK(got[1] == RigExecWarmFrameState::Cached);
        CHECK(got[2] == RigExecWarmFrameState::Uncached);
        CHECK(got[3] == RigExecWarmFrameState::Uncached);
    }
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// INDEX. Non-publish streaks: declined-invalid finishes and reasoned skips
// count toward un-warmable-after-3; publishes, generation pushes, and
// evictions clear; generation fences never count.
void
TestWarmFrameStreaks()
{
    std::printf("progress: TestWarmFrameStreaks\n");
    std::fflush(stdout);
    CHECK(kRigExecWarmUnwarmableAfter == 3);
    RigExecWarmFrameIndex index;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameCacheKey key{7, 9};
    CHECK(!index.IsUnwarmable(rig, 2.0));
    index.NoteSkipped(rig, 2.0);
    index.NoteSkipped(rig, 2.0);
    CHECK(!index.IsUnwarmable(rig, 2.0));
    CHECK(index.IsVisitable(rig, 2.0, 3, 7));
    RigExecWarmTransition finished;
    finished.rig = rig;
    finished.timeValue = 2.0;
    finished.kind = RigExecWarmTransitionKind::Finished;
    finished.outcome = RigExecWarmOutcome::DeclinedInvalid;
    index.NoteTransition(finished);
    CHECK(index.IsUnwarmable(rig, 2.0));
    CHECK(!index.IsVisitable(rig, 2.0, 3, 7));
    // A generation fence is transient: the streak stands.
    finished.outcome = RigExecWarmOutcome::DeclinedGeneration;
    index.NoteTransition(finished);
    CHECK(index.IsUnwarmable(rig, 2.0));
    // A publish clears.
    finished.outcome = RigExecWarmOutcome::Published;
    index.NoteTransition(finished);
    CHECK(!index.IsUnwarmable(rig, 2.0));
    CHECK(index.IsVisitable(rig, 2.0, 3, 7));
    // Revisited on edit: a generation push clears.
    index.NoteSkipped(rig, 2.0);
    index.NoteSkipped(rig, 2.0);
    index.NoteSkipped(rig, 2.0);
    CHECK(index.IsUnwarmable(rig, 2.0));
    index.NoteGeneration(rig, 4);
    CHECK(!index.IsUnwarmable(rig, 2.0));
    // A completion clears; a matching eviction clears too.
    index.NoteCompleted(rig, 2.0, key, 4);
    index.NoteSkipped(rig, 2.0);
    index.NoteSkipped(rig, 2.0);
    CHECK(!index.IsUnwarmable(rig, 2.0));
    index.NoteEvicted(rig, key, 2.0);
    index.NoteSkipped(rig, 2.0);
    index.NoteSkipped(rig, 2.0);
    CHECK(!index.IsUnwarmable(rig, 2.0));
    CHECK(index.IsVisitable(rig, 2.0, 4, 7));
}

// STACK. The full-range cursor over the layered stack anim: a registry-level
// idle driver (the headless shape of the plugin's recurring driver) warms
// every in-range frame -- published grows once per warmed frame, no trigger
// exceeds the sampling budget, the stack declines nothing, warmed frames
// serve a full scrub without evaluating, and warmed poses are bit-identical
// to live.
void
TestStackFullRangeCursorWarmsEveryFrame(const std::string &examplesDir)
{
    std::printf("progress: TestStackFullRangeCursorWarmsEveryFrame\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (examplesDir.empty()) {
        std::printf("SKIP TestStackFullRangeCursorWarmsEveryFrame: "
                    "no examples dir\n");
        return;
    }
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestStackFullRangeCursorWarmsEveryFrame: "
                    "warming unavailable in this process\n");
        return;
    }
    const std::string stagePath = examplesDir + "/biped/Biped_stack_anim.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(static_cast<bool>(stage));
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    const double start = stage->GetStartTimeCode();
    const double end = stage->GetEndTimeCode();
    CHECK(end > start);
    std::vector<double> frames;
    for (double t = start; t <= end; t += 1.0) {
        frames.push_back(t);
    }
    const size_t budget = registry.GetWarmSamplingMaxInvocations();
    // A cursor test needs more frames than one budgeted trigger can warm.
    CHECK(frames.size() > budget);
    if (frames.size() <= budget) {
        return;
    }
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(start), &errors));
    if (!registry.SetWarmRange(rig, frames)) {
        CHECK(false);
        registry.Deactivate();
        return;
    }
    // The playhead memo from Activate is already cached, not warmed: every
    // other frame must publish exactly once below.
    size_t alreadyCached = 0;
    {
        const std::vector<RigExecWarmFrameState> states =
            registry.GetFrameStates(rig, frames);
        for (const RigExecWarmFrameState state : states) {
            alreadyCached += state == RigExecWarmFrameState::Cached ? 1 : 0;
        }
    }
    const RigExecBackgroundSchedulerStats before =
        registry.GetBackgroundStats();
    // The recurring-driver shape: tick, drain, re-query, until every frame
    // is cached or the bound trips (a stuck frame fails loudly below).
    const size_t maxTicks = frames.size() + 8;
    size_t ticks = 0;
    for (; ticks < maxTicks; ++ticks) {
        const std::vector<RigExecWarmFrameState> states =
            registry.GetFrameStates(rig, frames);
        CHECK(states.size() == frames.size());
        bool allCached = states.size() == frames.size();
        for (const RigExecWarmFrameState state : states) {
            allCached = allCached && state == RigExecWarmFrameState::Cached;
        }
        if (allCached) {
            break;
        }
        const size_t invocationsBefore =
            registry.GetBackgroundStats().factoryInvocations;
        const int enqueued = registry.OnIdle();
        CHECK(enqueued >= 0);
        CHECK(enqueued >= 0 &&
              static_cast<size_t>(enqueued) <= budget);
        CHECK(registry.GetBackgroundStats().factoryInvocations -
                  invocationsBefore <=
              budget);
        registry.WaitUntilBackgroundIdle();
    }
    CHECK(ticks < maxTicks);
    {
        const std::vector<RigExecWarmFrameState> states =
            registry.GetFrameStates(rig, frames);
        CHECK(states.size() == frames.size());
        for (size_t i = 0; i < states.size() && i < frames.size(); ++i) {
            if (states[i] != RigExecWarmFrameState::Cached) {
                std::printf("frame %g never cached (state %d)\n", frames[i],
                            int(states[i]));
                CHECK(false);
            }
        }
    }
    const RigExecBackgroundSchedulerStats stats =
        registry.GetBackgroundStats();
    CHECK(stats.published - before.published ==
          frames.size() - alreadyCached);
    CHECK(stats.declinedInvalid - before.declinedInvalid == 0);
    CHECK(stats.declinedGeneration - before.declinedGeneration == 0);
    CHECK(stats.completed - before.completed ==
          stats.published - before.published);
    CHECK(stats.queuedDepth == 0 && stats.running == 0);
    // A full scrub serves without evaluating: every SetTime hits (the first
    // may be a same-frame no-op that reads nothing).
    const size_t evalsBefore = registry.GetSessionEvaluationCount(rig);
    const size_t hitsBefore = registry.GetFrameCacheStats(rig).hits;
    for (const double frame : frames) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == evalsBefore);
    const size_t hits = registry.GetFrameCacheStats(rig).hits - hitsBefore;
    CHECK(hits == frames.size() || hits + 1 == frames.size());
    // Warmed-vs-live bit identity on three spread frames: capture warmed,
    // clear, re-evaluate live, compare exactly.
    const std::vector<double> spots = {
        frames[frames.size() / 4], frames[frames.size() / 2],
        frames[3 * frames.size() / 4]};
    std::map<double, _GenerationGeometry> warmed;
    for (const double frame : spots) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        warmed[frame] = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!warmed[frame].points.empty());
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == evalsBefore);
    registry.ClearFrameCache(rig);
    for (const double frame : spots) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        const _GenerationGeometry live =
            _CaptureGeometry(registry.GetStore()->Get());
        if (!_SameGeometry(warmed[frame], live)) {
            std::printf("warmed-vs-live differs at frame %g\n", frame);
            CHECK(false);
        }
    }
    CHECK(registry.GetSessionEvaluationCount(rig) ==
          evalsBefore + spots.size());
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// BRANCHES. One case per 2.1 evaluator branch: a default-value patch on a
// captured avar, a keyframe nudge on a varying input, and a structural
// removal. Each pins the recorded disposition plus the registry behavior
// the branch drives: scoped (no generation bump, affected frames dirty,
// playhead re-memoized) for patch/stamp, global (bump plus epoch eviction)
// for stale.
void
TestEvaluatorBranchesDriveRetirement()
{
    std::printf("progress: TestEvaluatorBranchesDriveRetirement\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    const RigExecWarmFrameState dirty = RigExecWarmFrameState::Dirty;
    const RigExecWarmFrameState cached = RigExecWarmFrameState::Cached;
    const RigExecWarmFrameState uncached = RigExecWarmFrameState::Uncached;

    // Patched: exact patched-avar clusters retire; the generation stands.
    {
        std::vector<std::string> errors;
        UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.GetFrameCacheStats(rig).entryCount == 2);
        const RigExecFrameGeneration gen =
            registry.CurrentFrameGeneration(rig);
        const size_t pulls = registry.GetSessionEvaluationCount(rig);
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongY"))
            .GetAttribute(TfToken("avars:ty"))
            .Set(5.0);
        RigExecImagingBridge *bridge = registry.GetBridge(rig);
        CHECK(bridge != nullptr);
        if (bridge) {
            CHECK(bridge->GetEvaluator().GetLastNoticeDisposition() ==
                  RigExecNoticeDisposition::Patched);
            const std::vector<SdfPath> &patched =
                bridge->GetEvaluator().GetLastNoticePatchedPaths();
            CHECK(patched.size() == 1 &&
                  patched[0] == SdfPath("/Asset/Rig/AlongY.avars:ty"));
        }
        CHECK(registry.CurrentFrameGeneration(rig) == gen);
        // Old frames strand under the old namespace plus the fresh playhead
        // memo the notice path re-evaluated.
        CHECK(registry.GetFrameCacheStats(rig).entryCount == 3);
        CHECK(registry.GetSessionEvaluationCount(rig) == pulls + 1);
        {
            const std::vector<RigExecWarmFrameState> got =
                registry.GetFrameStates(rig, {1.0, 2.0});
            CHECK(got.size() == 2);
            if (got.size() == 2) {
                CHECK(got[0] == dirty);
                CHECK(got[1] == cached);
            }
        }
        // Frame 1 re-evaluates live once (the constant namespace moved),
        // then hits.
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        CHECK(registry.GetSessionEvaluationCount(rig) == pulls + 2);
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        CHECK(registry.GetSessionEvaluationCount(rig) == pulls + 2);
        registry.Deactivate();
    }

    // Stamp-bumped: affected retire plus re-resolve; the generation stands.
    {
        std::vector<std::string> errors;
        UsdStageRefPtr stage = MakeTinyRig();
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        // Frame 1's pre-edit content: the nudge below lands at frame 2,
        // so frame 1's values never move.
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        const _GenerationGeometry frame1Before =
            _CaptureGeometry(registry.GetStore()->Get());
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        const RigExecFrameGeneration gen =
            registry.CurrentFrameGeneration(rig);
        const size_t pulls = registry.GetSessionEvaluationCount(rig);
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongX"))
            .GetAttribute(TfToken("avars:tx"))
            .Set(11.0, UsdTimeCode(2.0));
        RigExecImagingBridge *bridge = registry.GetBridge(rig);
        CHECK(bridge != nullptr);
        if (bridge) {
            CHECK(bridge->GetEvaluator().GetLastNoticeDisposition() ==
                  RigExecNoticeDisposition::StampBumped);
        }
        CHECK(registry.CurrentFrameGeneration(rig) == gen);
        {
            const std::vector<RigExecWarmFrameState> got =
                registry.GetFrameStates(rig, {1.0, 2.0});
            CHECK(got.size() == 2);
            if (got.size() == 2) {
                CHECK(got[0] == dirty);
                CHECK(got[1] == cached);
            }
        }
        // Frame 1's values never moved, so the sparse plan heals its
        // retired proof and serves with zero pulls, bit-identical.
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        CHECK(registry.GetSessionEvaluationCount(rig) == pulls + 1);
        if (!_SameGeometry(frame1Before,
                            _CaptureGeometry(registry.GetStore()->Get()))) {
            std::printf("healed frame 1 differs from pre-edit content\n");
            CHECK(false);
        }
        registry.Deactivate();
    }

    // Stale: the program rebuilds, the epoch moves, and epoch-half eviction
    // plus generation cancel stand.
    {
        std::vector<std::string> errors;
        UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        const RigExecFrameGeneration gen =
            registry.CurrentFrameGeneration(rig);
        stage->RemovePrim(SdfPath("/Asset/Rig/Movers/Skin_0"));
        RigExecImagingBridge *bridge = registry.GetBridge(rig);
        CHECK(bridge != nullptr);
        if (bridge) {
            CHECK(bridge->GetEvaluator().GetLastNoticeDisposition() ==
                  RigExecNoticeDisposition::Stale);
        }
        CHECK(registry.CurrentFrameGeneration(rig) == gen + 1);
        // Evicted, not merely dirty: only an eviction clears the row to
        // uncached (a generation mismatch alone would read dirty).
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0});
        CHECK(got.size() == 1);
        if (got.size() == 1) {
            CHECK(got[0] == uncached);
        }
        registry.Deactivate();
    }
}

// CARRY. A constant patch re-keys entries whose provenance touches nothing
// dirty (lane b: re-publish under the new key, evict the old, re-point the
// proof) with zero recompute, while full-evaluation siblings retire. The
// clean provenance below is SYNTHETIC (hand-emptied): it pins the carry
// machinery -- re-key math, eviction, re-point, zero-pull serve of the
// claimed-untouched content -- under an oracle the test constructs.
// Production oracles are proven separately: full evaluations claim every
// cluster (never carry on a real patch), and the cone rule is the
// executor's own dirtiness test.
void
TestCarryOverRekeysCleanEntries()
{
    std::printf("progress: TestCarryOverRekeysCleanEntries\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    CHECK(registry.SetTime(UsdTimeCode(2.0)));
    CHECK(registry.SetTime(UsdTimeCode(3.0)));
    RigExecImagingBridge *bridge = registry.GetBridge(rig);
    CHECK(bridge != nullptr);
    if (!bridge) {
        registry.Deactivate();
        return;
    }
    const std::shared_ptr<RigExecFrameCache> cache =
        bridge->GetFrameCache();
    // Frame 1's key, pose, retained handle, and honest provenance.
    RigExecFrameCacheKey oldKey{0, 0};
    bool foundKey = false;
    for (const auto &entry : registry.GetCompletedKeys(rig)) {
        if (entry.first == UsdTimeCode(1.0)) {
            oldKey = entry.second;
            foundKey = true;
        }
    }
    CHECK(foundKey);
    RigExecRigPose pose;
    std::shared_ptr<const void> retained;
    size_t bytes = 0;
    CHECK(cache->Lookup(oldKey, &pose, &retained, &bytes));
    CHECK(pose.valid && retained);
    RigExecEntryProvenance honest;
    CHECK(cache->LookupProvenance(oldKey, &honest));
    CHECK(!honest.clusters.empty());
    CHECK(honest.unfoldedControlDigest != 0);
    const uint64_t unfolded = honest.unfoldedControlDigest;
    // The synthetic clean oracle: same content, empty dependency sets.
    RigExecEntryProvenance clean = honest;
    clean.clusters.clear();
    clean.weightReads.clear();
    clean.constantRegions.clear();
    CHECK(cache->Publish(oldKey, UsdTimeCode(1.0), pose, bytes, retained,
                         &clean));
    // The pre-edit content frame 1 must serve bit-identically: capture it
    // with a hit (zero pulls).
    const size_t pullsBeforeVisit = registry.GetSessionEvaluationCount(rig);
    CHECK(registry.SetTime(UsdTimeCode(1.0)));
    CHECK(registry.GetSessionEvaluationCount(rig) == pullsBeforeVisit);
    const _GenerationGeometry claimed =
        _CaptureGeometry(registry.GetStore()->Get());
    CHECK(!claimed.points.empty());
    CHECK(registry.SetTime(UsdTimeCode(3.0)));

    // The patch: ty's cone is real dirt, so full siblings retire while the
    // clean entry carries.
    const RigExecFrameGeneration gen = registry.CurrentFrameGeneration(rig);
    const size_t pulls = registry.GetSessionEvaluationCount(rig);
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongY"))
        .GetAttribute(TfToken("avars:ty"))
        .Set(5.0);
    CHECK(bridge->GetEvaluator().GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Patched);
    CHECK(registry.CurrentFrameGeneration(rig) == gen);
    // Carried: new key under the same epoch, old key evicted, proof
    // re-pointed at the new digest.
    RigExecFrameCacheKey newKey{0, 0};
    foundKey = false;
    for (const auto &entry : registry.GetCompletedKeys(rig)) {
        if (entry.first == UsdTimeCode(1.0)) {
            newKey = entry.second;
            foundKey = true;
        }
    }
    CHECK(foundKey);
    CHECK(newKey.epochDigest == oldKey.epochDigest);
    CHECK(newKey.controlDigest != oldKey.controlDigest);
    RigExecRigPose evicted;
    CHECK(!cache->Lookup(oldKey, &evicted));
    RigExecFreshProof proof;
    CHECK(bridge->GetFreshProof(UsdTimeCode(1.0), &proof));
    if (bridge->GetFreshProof(UsdTimeCode(1.0), &proof)) {
        CHECK(proof.digest == newKey.controlDigest);
        CHECK(proof.unfolded == unfolded);
    }
    // The strip: carried cached, full sibling dirty, re-memoized playhead
    // cached. Entries: carried re-keyed (net zero), sibling stranded, fresh
    // playhead memo.
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0});
        CHECK(got.size() == 3);
        if (got.size() == 3) {
            CHECK(got[0] == RigExecWarmFrameState::Cached);
            CHECK(got[1] == RigExecWarmFrameState::Dirty);
            CHECK(got[2] == RigExecWarmFrameState::Cached);
        }
    }
    CHECK(registry.GetFrameCacheStats(rig).entryCount == 4);
    // The carried frame serves with zero pulls, bit-identical to the
    // claimed content.
    CHECK(registry.SetTime(UsdTimeCode(1.0)));
    CHECK(registry.GetSessionEvaluationCount(rig) == pulls + 1);
    if (!_SameGeometry(claimed, _CaptureGeometry(registry.GetStore()->Get()))) {
        std::printf("carried frame differs from claimed content\n");
        CHECK(false);
    }
    // The retired sibling re-evaluates live exactly once, then hits.
    CHECK(registry.SetTime(UsdTimeCode(2.0)));
    CHECK(registry.GetSessionEvaluationCount(rig) == pulls + 2);
    CHECK(registry.SetTime(UsdTimeCode(1.0)));
    CHECK(registry.SetTime(UsdTimeCode(2.0)));
    CHECK(registry.GetSessionEvaluationCount(rig) == pulls + 2);
    registry.Deactivate();
}

// SCOPED-CANCEL. An edit affecting some times leaves other times' jobs to
// publish under the same generation; a gated old job for an affected time
// drops on fence-token mismatch and never overwrites the requeued result.
void
TestScopedCancelDropsOldJobsOnTokenMismatch()
{
    std::printf("progress: TestScopedCancelDropsOldJobsOnTokenMismatch\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    CHECK(registry.SetTime(UsdTimeCode(3.0)));
    const RigExecFrameGeneration gen = registry.CurrentFrameGeneration(rig);

    // Two jobs sampled under the standing token: one for a completed
    // frame the edit will purge, one for a frame it will not.
    RigExecWarmFactoryResult gated = registry.BuildWarmWork(
        rig, UsdTimeCode(3.0), gen, _TestKernel);
    CHECK(static_cast<bool>(gated.work));
    RigExecWarmFactoryResult untouched = registry.BuildWarmWork(
        rig, UsdTimeCode(5.0), gen, _TestKernel);
    CHECK(static_cast<bool>(untouched.work));

    // The patch purges completed times without bumping the generation.
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongY"))
        .GetAttribute(TfToken("avars:ty"))
        .Set(5.0);
    CHECK(registry.CurrentFrameGeneration(rig) == gen);
    const size_t publishedAfterEdit =
        registry.GetFrameCacheStats(rig).published;
    const size_t entriesAfterEdit =
        registry.GetFrameCacheStats(rig).entryCount;
    RigExecImagingSnapshotConstPtr snapshotBefore =
        registry.GetStore()->Get();

    // The gated job drops: its token predates the purge.
    RigExecWarmRequest oldRequest;
    oldRequest.rig = rig;
    oldRequest.time = UsdTimeCode(3.0);
    oldRequest.priority = RigExecWarmPriority::Neighbor;
    oldRequest.generation = gen;
    oldRequest.fenceToken = 0;
    if (gated.work) {
        CHECK(gated.work(oldRequest) == RigExecWarmOutcome::DeclinedGeneration);
    }
    CHECK(registry.GetFrameCacheStats(rig).published == publishedAfterEdit);
    CHECK(registry.GetFrameCacheStats(rig).entryCount == entriesAfterEdit);
    CHECK(registry.GetStore()->Get() == snapshotBefore);

    // The untouched time's job publishes normally under the same token.
    RigExecWarmRequest liveRequest = oldRequest;
    liveRequest.time = UsdTimeCode(5.0);
    if (untouched.work) {
        CHECK(untouched.work(liveRequest) == RigExecWarmOutcome::Published);
    }
    CHECK(registry.GetFrameCacheStats(rig).published ==
          publishedAfterEdit + 1);

    // The requeued frame publishes fresh; the gated job run after it still
    // drops, so the new result stands (a same-key replace would count a
    // publish, and a new key would grow the entry count -- neither moves).
    RigExecWarmFactoryResult fresh = registry.BuildWarmWork(
        rig, UsdTimeCode(3.0), gen, _TestKernel);
    CHECK(static_cast<bool>(fresh.work));
    if (fresh.work) {
        // The scheduler stamps the current token at enqueue; the purge
        // moved frame 3's. Resolve it the way the scheduler does: the
        // match publishes while the gated job's mismatch drops.
        const RigExecWarmFenceToken live =
            registry.CurrentFenceToken(rig, UsdTimeCode(3.0));
        CHECK(live != 0);
        CHECK(registry.CurrentFenceToken(rig, UsdTimeCode(5.0)) == 0);
        RigExecWarmRequest freshRequest = oldRequest;
        freshRequest.fenceToken = live;
        CHECK(fresh.work(freshRequest) == RigExecWarmOutcome::Published);
        const size_t publishedAfterFresh =
            registry.GetFrameCacheStats(rig).published;
        const size_t entriesAfterFresh =
            registry.GetFrameCacheStats(rig).entryCount;
        if (gated.work) {
            CHECK(gated.work(oldRequest) ==
                  RigExecWarmOutcome::DeclinedGeneration);
        }
        CHECK(registry.GetFrameCacheStats(rig).published ==
              publishedAfterFresh);
        CHECK(registry.GetFrameCacheStats(rig).entryCount ==
              entriesAfterFresh);
    }
    CHECK(registry.GetStore()->Get() == snapshotBefore);
    // The injected kernel's poses are markers, not evaluations: retire
    // them so no later scrub can serve one.
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// RETIRE. Editing one control retires exactly the affected frames: the
// strip dirties the retired set while the playhead re-memoizes, the next
// commit re-warms retired frames first (a tight budget proves the order),
// and the re-warmed frames are bit-identical to live evaluation.
void
TestEditOneControlRetiresAndRewarms()
{
    std::printf("progress: TestEditOneControlRetiresAndRewarms\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    for (double frame : {2.0, 3.0, 4.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
    }
    // One control moves: every full-evaluation frame is affected (each
    // claims every cluster), so all retire while the playhead re-memoizes
    // under the new namespace.
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongY"))
        .GetAttribute(TfToken("avars:ty"))
        .Set(5.0);
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0, 4.0});
        CHECK(got.size() == 4);
        if (got.size() == 4) {
            CHECK(got[0] == RigExecWarmFrameState::Dirty);
            CHECK(got[1] == RigExecWarmFrameState::Dirty);
            CHECK(got[2] == RigExecWarmFrameState::Dirty);
            CHECK(got[3] == RigExecWarmFrameState::Cached);
        }
    }
    if (!RigExecBackgroundWarmingEnabled()) {
        registry.Deactivate();
        return;
    }
    // A distant playhead keeps the retired set out of the neighbor band,
    // so the tight budget below isolates the pending-first order: 16
    // neighbors plus the closest retired frame.
    CHECK(registry.SetTime(UsdTimeCode(100.0)));
    const size_t savedInvocations = registry.GetWarmSamplingMaxInvocations();
    const double savedMs = registry.GetWarmSamplingMaxMs();
    registry.SetWarmSamplingBudget(17, std::numeric_limits<double>::infinity());
    CHECK(registry.OnEditCommitted() == 17);
    registry.WaitUntilBackgroundIdle();
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0});
        CHECK(got.size() == 3);
        if (got.size() == 3) {
            // Closest-first among the retired set: frame 3 re-warms while
            // 1 and 2 wait.
            CHECK(got[0] == RigExecWarmFrameState::Dirty);
            CHECK(got[1] == RigExecWarmFrameState::Dirty);
            CHECK(got[2] == RigExecWarmFrameState::Cached);
        }
    }
    registry.SetWarmSamplingBudget(savedInvocations, savedMs);
    // Drain the rest through the production path (no injected kernel, so
    // the re-warmed poses are real evaluations).
    for (int i = 0; i < 8; ++i) {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0, 4.0});
        bool allCached = got.size() == 4;
        for (const RigExecWarmFrameState state : got) {
            allCached =
                allCached && state == RigExecWarmFrameState::Cached;
        }
        if (allCached) {
            break;
        }
        registry.OnIdle();
        registry.WaitUntilBackgroundIdle();
    }
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0, 4.0});
        CHECK(got.size() == 4);
        for (size_t i = 0; i < got.size(); ++i) {
            if (got[i] != RigExecWarmFrameState::Cached) {
                std::printf("frame %g never re-warmed\n", double(i + 1));
                CHECK(false);
            }
        }
    }
    // Re-warmed frames serve without evaluating, bit-identical to live.
    const size_t pullsBefore = registry.GetSessionEvaluationCount(rig);
    std::map<double, _GenerationGeometry> rewarmed;
    for (double frame : {1.0, 2.0, 3.0, 4.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        rewarmed[frame] = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!rewarmed[frame].points.empty());
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == pullsBefore);
    registry.ClearFrameCache(rig);
    for (double frame : {1.0, 2.0, 3.0, 4.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        const _GenerationGeometry live =
            _CaptureGeometry(registry.GetStore()->Get());
        if (!_SameGeometry(rewarmed[frame], live)) {
            std::printf("re-warmed frame %g differs from live\n", frame);
            CHECK(false);
        }
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == pullsBefore + 4);
    registry.Deactivate();
}

// PROOFS. Freshness proofs carry their sampled-path dependency set: a
// constant patch (constants are never sampled) retires no proof, while a
// varying edit retires every proof naming it.
void
TestProofScopingRetiresOnlyIntersecting()
{
    std::printf("progress: TestProofScopingRetiresOnlyIntersecting\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();

    // Constant patch: proofs survive (their sets name sampled paths only).
    {
        std::vector<std::string> errors;
        UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        RigExecImagingBridge *bridge = registry.GetBridge(rig);
        CHECK(bridge != nullptr);
        if (!bridge) {
            registry.Deactivate();
            return;
        }
        CHECK(bridge->GetFreshProofCount() == 2);
        RigExecFreshProof proof;
        CHECK(bridge->GetFreshProof(UsdTimeCode(1.0), &proof));
        // The dependency set is recorded: non-empty, sorted, unique.
        CHECK(!proof.paths.empty());
        CHECK(std::is_sorted(proof.paths.begin(), proof.paths.end()));
        CHECK(std::adjacent_find(proof.paths.begin(), proof.paths.end()) ==
              proof.paths.end());
        CHECK(proof.digest != 0 && proof.unfolded != 0);
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongY"))
            .GetAttribute(TfToken("avars:ty"))
            .Set(5.0);
        CHECK(bridge->GetFreshProofCount() == 2);
        registry.Deactivate();
    }

    // Varying edit: every proof names the moved sample, so all retire.
    {
        std::vector<std::string> errors;
        UsdStageRefPtr stage = MakeTinyRig();
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        RigExecImagingBridge *bridge = registry.GetBridge(rig);
        CHECK(bridge != nullptr);
        if (!bridge) {
            registry.Deactivate();
            return;
        }
        CHECK(bridge->GetFreshProofCount() == 2);
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongX"))
            .GetAttribute(TfToken("avars:tx"))
            .Set(11.0, UsdTimeCode(2.0));
        // Both proofs retired; the trailing playhead re-memoize records
        // one fresh proof, so exactly the playhead proves.
        CHECK(bridge->GetFreshProofCount() == 1);
        RigExecFreshProof retired;
        CHECK(!bridge->GetFreshProof(UsdTimeCode(1.0), &retired));
        CHECK(bridge->GetFreshProof(UsdTimeCode(2.0), &retired));
        registry.Deactivate();
    }
}

// DRAG. A drag keeps the interactive bypass; building warm work under its
// standing overrides admits the override identities to the epoch index
// (the MapControl production caller); on release the commit burst
// re-warms what the committed values retired.
void
TestDragReleaseRewarmsAffected()
{
    std::printf("progress: TestDragReleaseRewarmsAffected\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    const SdfPath alongX("/Asset/Rig/AlongX");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    UsdStageRefPtr stage = MakeTinyRig();
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    CHECK(registry.SetTime(UsdTimeCode(1.0)));
    CHECK(registry.SetTime(UsdTimeCode(3.0)));
    RigExecImagingBridge *bridge = registry.GetBridge(rig);
    CHECK(bridge != nullptr);
    if (!bridge) {
        registry.Deactivate();
        return;
    }
    const size_t publishedBeforeDrag =
        registry.GetFrameCacheStats(rig).published;

    // The drag: standing overrides bypass the cache (no reads, no writes).
    bridge->SetInteractiveOverrides({RigExecValueOverride{
        alongX, TfToken(), TfToken("avars:tx"), VtValue(999.0)}});
    CHECK(registry.SetTime(UsdTimeCode(2.0)));
    CHECK(registry.GetFrameCacheStats(rig).published == publishedBeforeDrag);

    // Warming under the drag admits the override identity with its exact
    // seeds (mirroring SetOverrides placement), instead of leaving it
    // foreign.
    const RigExecFrameGeneration gen = registry.CurrentFrameGeneration(rig);
    RigExecWarmFactoryResult dragged = registry.BuildWarmWork(
        rig, UsdTimeCode(4.0), gen, _TestKernel);
    CHECK(static_cast<bool>(dragged.work));
    const RigExecBakedProgram *program =
        bridge->GetEvaluator().GetBakedProgram();
    CHECK(program != nullptr);
    const RigExecOutputAffectedIndex *index = bridge->GetAffectedIndex();
    CHECK(index != nullptr && !index->Empty());
    if (program && index && !index->Empty()) {
        const RigExecValueOverride o{alongX, TfToken(), TfToken("avars:tx"),
                                     VtValue(999.0)};
        const RigExecControlId id = RigExecControlIdForOverride(o);
        CHECK(index->IsKnownControl(id));
        const std::vector<int> seeds = RigExecOverrideSeeds(
            program->GetStepGraph(), o);
        CHECK(!seeds.empty());
        CHECK(index->SeedsForControl(id) == seeds);
    }

    // The release: overrides lift, committed values author, and the
    // notice retires the affected frames without bumping the generation.
    bridge->ClearInteractiveOverrides();
    stage->GetPrimAtPath(alongX).GetAttribute(TfToken("avars:tx")).Set(
        10.5, UsdTimeCode(2.0));
    CHECK(bridge->GetEvaluator().GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::StampBumped);
    CHECK(registry.CurrentFrameGeneration(rig) == gen);
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0});
        CHECK(got.size() == 3);
        if (got.size() == 3) {
            CHECK(got[0] == RigExecWarmFrameState::Dirty);
            CHECK(got[1] == RigExecWarmFrameState::Cached);
            CHECK(got[2] == RigExecWarmFrameState::Dirty);
        }
    }
    if (!RigExecBackgroundWarmingEnabled()) {
        registry.Deactivate();
        return;
    }
    // The commit burst carries the retired frames back to cached.
    for (int i = 0; i < 8; ++i) {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0});
        bool allCached = got.size() == 3;
        for (const RigExecWarmFrameState state : got) {
            allCached =
                allCached && state == RigExecWarmFrameState::Cached;
        }
        if (allCached) {
            break;
        }
        registry.OnEditCommitted();
        registry.WaitUntilBackgroundIdle();
    }
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0});
        CHECK(got.size() == 3);
        for (size_t i = 0; i < got.size(); ++i) {
            if (got[i] != RigExecWarmFrameState::Cached) {
                std::printf("frame %g never re-warmed after release\n",
                            double(i + 1));
                CHECK(false);
            }
        }
    }
    registry.Deactivate();
}

// FENCED-CLEAR. Clear with jobs queued and running, then drain: the clear
// bumps the generation and purges the queue, every late completion drops
// at the publish fence (no repopulation), and the 1.4 index resets.
void
TestClearWhileWarmingKeepsCacheEmpty()
{
    std::printf("progress: TestClearWhileWarmingKeepsCacheEmpty\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    if (!RigExecBackgroundWarmingEnabled()) {
        registry.Deactivate();
        return;
    }
    // Warmed neighbors plus the playhead memo: the clear below retires
    // real entries, not an empty cache.
    CHECK(registry.OnEditCommitted(_TestKernel) == 16);
    registry.WaitUntilBackgroundIdle();
    CHECK(registry.GetFrameCacheStats(rig).entryCount > 0);
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1.0, 2.0, 3.0, 4.0});
        CHECK(got.size() == 4);
        for (const RigExecWarmFrameState state : got) {
            CHECK(state == RigExecWarmFrameState::Cached);
        }
    }
    const RigExecFrameGeneration gen = registry.CurrentFrameGeneration(rig);

    // A gated burst: both pool workers block running while the rest of the
    // burst queues behind them.
    _PublishGate gate;
    const size_t enqueued = registry.OnEditCommitted(_GatedKernel(&gate));
    CHECK(enqueued > 2);
    CHECK(_WaitFor([&] { return gate.entered.load() >= 2; }, 10000));

    // Clear across queued AND running jobs.
    registry.ClearFrameCache(rig);
    CHECK(registry.CurrentFrameGeneration(rig) == gen + 1);
    CHECK(registry.GetFrameCacheStats(rig).entryCount == 0);
    CHECK(registry.GetFrameCacheStats(rig).published == 0);
    _ReleaseGate(&gate);
    registry.WaitUntilBackgroundIdle();

    // Every gated job dropped at the fence: the cache stays empty and no
    // frame reads cached.
    const RigExecFrameCacheStats stats = registry.GetFrameCacheStats(rig);
    CHECK(stats.entryCount == 0);
    CHECK(stats.published == 0);
    const std::vector<RigExecWarmFrameState> got =
        registry.GetFrameStates(rig, {1.0, 2.0, 3.0, 4.0});
    CHECK(got.size() == 4);
    for (const RigExecWarmFrameState state : got) {
        CHECK(state == RigExecWarmFrameState::Uncached);
    }
    registry.Deactivate();
}

// FENCE-RACE. A job paused between the fence-check and the cache insert --
// holding the fence -- while the main thread clears: the clear cannot land
// mid-pause, so it serializes after the insert and the cache stays empty,
// proving check-and-insert atomicity. Without the shared mutex the clear
// would slip between the check and the store and the resumed insert would
// repopulate.
void
TestFencedClearSerializesAfterPausedInsert()
{
    std::printf("progress: TestFencedClearSerializesAfterPausedInsert\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    const RigExecFrameGeneration gen = registry.CurrentFrameGeneration(rig);

    // One production-closure job: it captures the session cache and the
    // scheduler whose fence the clear below takes.
    RigExecWarmFactoryResult gated = registry.BuildWarmWork(
        rig, UsdTimeCode(5.0), gen, _TestKernel);
    CHECK(static_cast<bool>(gated.work));

    struct Probe {
        std::mutex mutex;
        std::condition_variable cv;
        std::atomic<bool> inside{false};
        std::atomic<bool> release{false};
    };
    Probe probe;
    RigExecSetPublishFenceProbeForTesting([&] {
        probe.inside.store(true);
        std::unique_lock<std::mutex> lock(probe.mutex);
        probe.cv.wait(lock, [&] { return probe.release.load(); });
    });

    RigExecWarmRequest request;
    request.rig = rig;
    request.time = UsdTimeCode(5.0);
    request.priority = RigExecWarmPriority::Neighbor;
    request.generation = gen;
    request.fenceToken = registry.CurrentFenceToken(rig, UsdTimeCode(5.0));
    std::atomic<RigExecWarmOutcome> outcome{
        RigExecWarmOutcome::DeclinedInvalid};
    std::thread worker([&] {
        if (gated.work) {
            outcome.store(gated.work(request));
        }
    });
    CHECK(_WaitFor([&] { return probe.inside.load(); }, 10000));
    std::atomic<bool> attempted{false};
    std::thread clearer([&] {
        attempted.store(true);
        registry.ClearFrameCache(rig);
    });
    CHECK(_WaitFor([&] { return attempted.load(); }, 10000));
    // Let the clear reach the fence and block: it cannot pass until the
    // probe releases, because the worker holds the fence across its check
    // and its insert.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
        std::lock_guard<std::mutex> lock(probe.mutex);
        probe.release.store(true);
    }
    probe.cv.notify_all();
    worker.join();
    clearer.join();
    RigExecSetPublishFenceProbeForTesting(nullptr);

    // The job ran through the race and published; the serialized clear
    // wiped it afterwards. The cache stays empty either way.
    CHECK(outcome.load() == RigExecWarmOutcome::Published);
    CHECK(registry.CurrentFrameGeneration(rig) == gen + 1);
    const RigExecFrameCacheStats stats = registry.GetFrameCacheStats(rig);
    CHECK(stats.entryCount == 0);
    CHECK(stats.published == 0);
    // The job's completion record is stamped pre-clear: it either landed
    // before the reset (wiped) or after it (older generation: dirty) --
    // never cached.
    const std::vector<RigExecWarmFrameState> got =
        registry.GetFrameStates(rig, {5.0});
    CHECK(got.size() == 1);
    if (!got.empty()) {
        CHECK(got[0] != RigExecWarmFrameState::Cached);
    }
    registry.Deactivate();
}

// OVERLAY-RACE. Setting the overlay mid-warming cancels before clearing:
// jobs sampled under the old flag drop at the publish fence, and no
// stale-overlay pose is served afterwards. (The overlay moves pose content
// without moving the key, so a clear-before-cancel order would let a stale
// pose land in the cleared cache and serve under a fresh proof.)
void
TestOverlayMidWarmingServesNoStalePose()
{
    std::printf("progress: TestOverlayMidWarmingServesNoStalePose\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    const bool readsOn =
        RigExecFrameCacheModeFromEnvironment() != RigExecFrameCacheMode::Off;
    const RigExecFrameGeneration gen = registry.CurrentFrameGeneration(rig);

    // Old-flag jobs in flight while the overlay moves (when the fill gate
    // runs; otherwise the order holds trivially over an empty queue).
    _PublishGate gate;
    const size_t enqueued = registry.OnEditCommitted(_GatedKernel(&gate));
    if (enqueued > 0) {
        CHECK(_WaitFor([&] { return gate.entered.load() >= 1; }, 10000));
    }
    CHECK(registry.SetWeightOverlay("/Asset/Rig/Movers/Skin_0"));
    CHECK(registry.CurrentFrameGeneration(rig) == gen + 1);
    _ReleaseGate(&gate);
    registry.WaitUntilBackgroundIdle();

    // Only the overlay republish's playhead memo stands: every old-flag
    // job dropped instead of publishing a stale-overlay pose.
    const RigExecFrameCacheStats stats = registry.GetFrameCacheStats(rig);
    if (readsOn) {
        CHECK(stats.published == 1 && stats.entryCount == 1);
        CHECK(stats.hits == 0 && stats.misses == 0);
    } else {
        CHECK(stats.published == 0 && stats.entryCount == 0);
    }
    // A neighbor the burst targeted serves nothing stale: it evaluates
    // live, with no hit counted.
    const size_t pullsBefore = registry.GetSessionEvaluationCount(rig);
    const size_t hitsBefore = registry.GetFrameCacheStats(rig).hits;
    CHECK(registry.SetTime(UsdTimeCode(3.0)));
    CHECK(registry.GetSessionEvaluationCount(rig) == pullsBefore + 1);
    CHECK(registry.GetFrameCacheStats(rig).hits == hitsBefore);
    CHECK(registry.SetWeightOverlay(""));
    registry.Deactivate();
}

}  // namespace

int
main(int argc, char **argv)
{
    const std::string examplesDir = argc > 1 ? argv[1] : "";
    // Baked unless the outer environment chose a mode: the bridge's
    // evaluators inherit the session mode, and only a baked program can be
    // sampled for cache keys. Read once per process by the evaluator, so
    // this must precede the first construction.
#ifdef _WIN32
    char *outerMode = nullptr;
    size_t outerModeSize = 0;
    const bool outerChoseMode =
        _dupenv_s(&outerMode, &outerModeSize, "RIGEXEC_EVALUATION_MODE") == 0 &&
        outerMode && *outerMode;
    free(outerMode);
#else
    const char *outerMode = getenv("RIGEXEC_EVALUATION_MODE");
    const bool outerChoseMode = outerMode && *outerMode;
#endif
    if (!outerChoseMode) {
        SetEnv("RIGEXEC_EVALUATION_MODE", "baked");
    }
    const auto scratch =
        std::filesystem::temp_directory_path() /
        ("rigexec-imaging-framecache-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    if (!std::filesystem::create_directory(scratch)) {
        return 1;
    }
    TestScrubWarmsThenHits();
    TestCacheOffMatchesCacheOn();
    TestStaticControlEditInvalidatesCachedFrames();
    TestCaptureIndexHitDropsEpochEagerly();
    TestOverlayChangeClearsCache();
    TestPlaybackBypassesCache(scratch);
    TestEditBumpsGenerationAndDropsStaleWork();
    TestBackgroundCompletionFence();
    TestTriggersEnqueueNeighborsAndSweep();
    TestStandingBurstSurvivesSecondIdle();
    TestOverSlicePrepTakesPlainRoute();
    TestProfilerWarmScopes();
    TestWarmFrameIndexStates();
    TestFrameCacheEvictionCallbackReportsDrops();
    TestFrameStatesScriptedSequence();
    TestFrameStatesEvictionRetires();
    TestSamplingBudgetShapesTriggers();
    TestWarmRangeCursorSkipsVisited();
    TestRefusalRigNeverCountsVisited();
    TestWarmFrameStreaks();
    TestCommitDuringPreviewExcludesPlayhead();
    TestProductionTriggerPathEnqueuesAndFences();
    TestWarmedCompletionServesWithoutEvaluating();
    TestRefusalRigMemoizesUiThreadResults();
    TestInteractiveDragBypassesCache();
    TestProfilerFrameCacheLane();
    TestCachedServeSkipsLookupSample();
    TestStackFullRangeCursorWarmsEveryFrame(examplesDir);
    TestEvaluatorBranchesDriveRetirement();
    TestCarryOverRekeysCleanEntries();
    TestScopedCancelDropsOldJobsOnTokenMismatch();
    TestEditOneControlRetiresAndRewarms();
    TestProofScopingRetiresOnlyIntersecting();
    TestDragReleaseRewarmsAffected();
    TestClearWhileWarmingKeepsCacheEmpty();
    TestFencedClearSerializesAfterPausedInsert();
    TestOverlayMidWarmingServesNoStalePose();
    if (failures == 0) {
        std::printf("testRigExecImagingFrameCache: all tests passed\n");
    } else {
        std::printf("testRigExecImagingFrameCache: %d failures\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
