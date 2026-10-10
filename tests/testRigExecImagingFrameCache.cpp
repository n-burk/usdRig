// testRigExecImagingFrameCache (Stream E): the imaging integration of the
// per-frame cache -- scrub-from-cache, live fallback, edit fencing, and
// cache-only background completions.
// The contract, one rule per test group in the testRigExecStaticInputCache
// style:
//   * SCRUB. A scripted scrub across a warmed range performs zero evaluator
//     pulls, and every served generation is bit-identical to the live
//     evaluation of the same frame.
//   * COLD. Scrub into cold frames evaluates live with behavior equal to
//     the cache-off path (same generations, same pull counts).
//   * BYPASS. Cache-off, empty, and playback sessions never
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
//     none, varying edits retire all. The set is the vector's shared digest
//     order, and retiring by control id matches the path-text rule.
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
// The rig is the frozen-context test's tiny in-memory rig (one skinned mesh
// over two animated controls), which bakes, evaluates, and samples. The
// tests set RIGEXEC_FRAME_CACHE in-process and also pass under the
// validation plan's outer combos (cache off, parallel eval off, verify on)
// by reading the live switches and expecting the combo's own counts.
#include "rigExecImaging/bridge.h"
#include "rigExecImaging/registry.h"
#include "rigExecBake/bake.h"
#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/goldenPose.h"
#include "rigExec/frameCache.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/frozenContext.h"
#include "rigExec/generation.h"
#include "rigExec/outputAffectedIndex.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/spline.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/usd/notice.h"
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
#include <set>
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

// A two-joint rig whose root rest:tx is animated, each joint under an
// authored non-identity default:space, skinning one mesh: an edit of the
// root rest moves both rests and neither ladder.
UsdStageRefPtr
MakeRestRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Joints"), TfToken("Scope"));
    const UsdPrim root = stage->DefinePrim(
        SdfPath("/Asset/Rig/Joints/Root"), TfToken("RigExecJoint"));
    const UsdPrim child = stage->DefinePrim(
        SdfPath("/Asset/Rig/Joints/Root/Child"), TfToken("RigExecJoint"));
    const UsdAttribute restTx =
        root.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double);
    restTx.Set(1.0, UsdTimeCode(1.0));
    restTx.Set(2.0, UsdTimeCode(2.0));
    restTx.Set(4.0, UsdTimeCode(3.0));
    child.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
        .Set(1.0);
    GfMatrix4d rootSpace(1.0), childSpace(1.0);
    rootSpace.SetTranslateOnly(GfVec3d(0.0, 1.0, 0.0));
    childSpace.SetTranslateOnly(GfVec3d(0.0, 0.0, 1.0));
    root.CreateAttribute(TfToken("default:space"),
                         SdfValueTypeNames->Matrix4d).Set(rootSpace);
    child.CreateAttribute(TfToken("default:space"),
                          SdfValueTypeNames->Matrix4d).Set(childSpace);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const SdfPath meshPath("/Asset/Geom/Mesh_0");
    const UsdPrim prim = stage->DefinePrim(meshPath, TfToken("Mesh"));
    VtVec3fArray points(kTinyPointCount);
    for (size_t i = 0; i < kTinyPointCount; ++i) {
        points[i] = GfVec3f(float(i) * 0.5f, float(i) * -0.25f, 0.125f);
    }
    prim.GetAttribute(TfToken("points")).Set(points);
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin_0"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({meshPath.AppendProperty(TfToken("points"))});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({root.GetPath(), child.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(2);
    VtIntArray indices(kTinyPointCount * 2);
    VtFloatArray weights(kTinyPointCount * 2);
    for (size_t i = 0; i < kTinyPointCount; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
        weights[i * 2] = 0.5f;
        weights[i * 2 + 1] = 0.5f;
    }
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(indices);
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray).Set(weights);
    return stage;
}

// A rest edit is seeded through the head tier (the rest and ladder ops'
// closure), so a frame it moved is not served from the cache: revisited
// after the edit, frame 1 equals the cache-off session's, and differs from
// the frame before the edit.
void
TestARestEditIsNotServedStale()
{
    std::printf("progress: TestARestEditIsNotServedStale\n");
    std::fflush(stdout);
    const SdfPath rig("/Asset/Rig");
    auto drive = [&](UsdStageRefPtr stage, _GenerationGeometry *before,
                     _GenerationGeometry *after) {
        RigExecImagingRegistry &registry =
            RigExecImagingRegistry::GetInstance();
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.SetTime(UsdTimeCode(3.0)));
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        *before = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Joints/Root"))
            .GetAttribute(TfToken("rest:tx"))
            .Set(1.75, UsdTimeCode(1.0));
        CHECK(registry.SetTime(UsdTimeCode(1.0)));
        *after = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!after->points.empty());
        registry.Deactivate();
    };

    _GenerationGeometry referenceBefore, reference;
    SetEnv("RIGEXEC_FRAME_CACHE", "off");
    drive(MakeRestRig(), &referenceBefore, &reference);
    CHECK(!_SameGeometry(referenceBefore, reference));

    _GenerationGeometry cachedBefore, cached;
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    drive(MakeRestRig(), &cachedBefore, &cached);
    CHECK(_SameGeometry(referenceBefore, cachedBefore));
    if (!_SameGeometry(reference, cached)) {
        std::printf("cache-on served a pre-edit pose at frame 1 after a "
                    "rest edit\n");
        CHECK(false);
    }
}

// WARM over a recomposing ladder. MakeRestRig's root rest:tx is animated,
// so its provider ladder recomposes every frame; the rig freezes, and the
// production worker warms frames 2 and 3 by running the rest and ladder ops
// from each job's sampled ladder leaves. A scrub over the range then serves
// every frame from the cache with no evaluation, and each served frame
// equals the live evaluation of it after the cache is cleared.
void
TestAnAnimatedRestRigServesWarmedHits()
{
    std::printf("progress: TestAnAnimatedRestRigServesWarmedHits\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
            RigExecFrameCacheMode::Off ||
        RigExecFrameCacheVerifyRequested()) {
        return;
    }
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestAnAnimatedRestRigServesWarmedHits: warming "
                    "unavailable in this process\n");
        return;
    }
    const UsdStageRefPtr stage = MakeRestRig();
    const SdfPath rig("/Asset/Rig");
    const std::vector<double> frames{1.0, 2.0, 3.0};
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    RigExecImagingBridge *bridge = registry.GetBridge(rig);
    CHECK(bridge != nullptr);
    if (!bridge) {
        registry.Deactivate();
        return;
    }
    const RigExecBakedProgram *program =
        bridge->GetEvaluator().GetBakedProgram();
    CHECK(program != nullptr);
    // Not vacuous: the ladder is read per frame, which the freeze refused
    // before the worker ran the rest tier.
    CHECK(program && program->GetStepGraph().ladderVarying);
    std::string why;
    CHECK(RigExecCanFreezeProgram(bridge->GetEvaluator(), &why));
    if (!why.empty()) {
        std::printf("    freeze refused: %s\n", why.c_str());
    }
    CHECK(registry.SetWarmRange(rig, frames));
    const RigExecBackgroundSchedulerStats before =
        registry.GetBackgroundStats();
    for (size_t tick = 0; tick < frames.size() + 4; ++tick) {
        bool allCached = true;
        for (const RigExecWarmFrameState state :
             registry.GetFrameStates(rig, frames)) {
            allCached = allCached && state == RigExecWarmFrameState::Cached;
        }
        if (allCached) {
            break;
        }
        CHECK(registry.OnIdle() >= 0);
        registry.WaitUntilBackgroundIdle();
    }
    for (const RigExecWarmFrameState state :
         registry.GetFrameStates(rig, frames)) {
        CHECK(state == RigExecWarmFrameState::Cached);
    }
    const RigExecBackgroundSchedulerStats stats =
        registry.GetBackgroundStats();
    // Frame 1 is the Activate memo; 2 and 3 are warmed by frozen jobs.
    CHECK(stats.published - before.published == frames.size() - 1);
    CHECK(stats.declinedInvalid - before.declinedInvalid == 0);
    CHECK(stats.declinedFreezeRefused - before.declinedFreezeRefused == 0);
    CHECK(stats.declinedUnsampleable - before.declinedUnsampleable == 0);

    const size_t evalsBefore = registry.GetSessionEvaluationCount(rig);
    const size_t hitsBefore = registry.GetFrameCacheStats(rig).hits;
    std::map<double, _GenerationGeometry> warmed;
    for (const double frame : {3.0, 2.0, 1.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        warmed[frame] = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!warmed[frame].points.empty());
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == evalsBefore);
    CHECK(registry.GetFrameCacheStats(rig).hits - hitsBefore ==
          frames.size());
    // The rests moved the mesh between the warmed frames.
    CHECK(!_SameGeometry(warmed[2.0], warmed[3.0]));
    registry.ClearFrameCache(rig);
    for (const double frame : {2.0, 3.0, 1.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        const _GenerationGeometry live =
            _CaptureGeometry(registry.GetStore()->Get());
        if (!_SameGeometry(warmed[frame], live)) {
            std::printf("warmed-vs-live differs at frame %g on the animated "
                        "rest rig\n", frame);
            CHECK(false);
        }
    }
    CHECK(registry.GetSessionEvaluationCount(rig) ==
          evalsBefore + frames.size());

    // A value edit on the child's static rest, and the range warmed again:
    // each frame served after it equals its live evaluation, so nothing
    // keyed before the edit is served after it.
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Joints/Root/Child"))
        .GetAttribute(TfToken("rest:tx"))
        .Set(2.5);
    for (size_t tick = 0; tick < frames.size() + 4; ++tick) {
        bool allCached = true;
        for (const RigExecWarmFrameState state :
             registry.GetFrameStates(rig, frames)) {
            allCached = allCached && state == RigExecWarmFrameState::Cached;
        }
        if (allCached) {
            break;
        }
        CHECK(registry.OnIdle() >= 0);
        registry.WaitUntilBackgroundIdle();
    }
    std::map<double, _GenerationGeometry> edited;
    for (const double frame : {3.0, 2.0, 1.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        edited[frame] = _CaptureGeometry(registry.GetStore()->Get());
    }
    CHECK(!_SameGeometry(edited[2.0], warmed[2.0]));
    registry.ClearFrameCache(rig);
    for (const double frame : {2.0, 3.0, 1.0}) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        if (!_SameGeometry(edited[frame],
                           _CaptureGeometry(registry.GetStore()->Get()))) {
            std::printf("served-vs-live differs at frame %g after a static "
                        "rest edit\n", frame);
            CHECK(false);
        }
    }
    registry.ClearFrameCache(rig);
    registry.Deactivate();
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

    RigExecBakeOpts opts;
    opts.time = 1.0;
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
    RigExecWarmFrameIndex index;
    const RigExecFrameGeneration generation =
        scheduler.CurrentGeneration(rig);

    RigExecRigPose pose;
    pose.valid = true;
    pose.diagnostics.push_back("completion");
    const RigExecFrameCacheKey key{7, 11};
    CHECK(RigExecPublishBackgroundCompletion(
        cache, key, UsdTimeCode(3.0), pose, generation, &scheduler, rig,
        RigExecWarmFenceToken(0), nullptr, 0, nullptr, &index));
    RigExecRigPose served;
    CHECK(cache->Lookup(key, &served));
    CHECK(served.valid);
    RigExecFrameCacheKey indexed;
    CHECK(index.FindCachedKey(rig, 3.0, generation, key.epochDigest, &indexed));
    CHECK(indexed == key);

    // An edit lands: the same token no longer publishes.
    scheduler.CancelGeneration(rig);
    const RigExecFrameCacheKey stale{7, 12};
    CHECK(!RigExecPublishBackgroundCompletion(
        cache, stale, UsdTimeCode(4.0), pose, generation, &scheduler, rig,
        RigExecWarmFenceToken(0), nullptr, 0, nullptr, &index));
    CHECK(!cache->Lookup(stale, &served));
    CHECK(!index.FindKey(rig, 4.0, &indexed));

    // The new token publishes again.
    const RigExecFrameGeneration fresh = scheduler.CurrentGeneration(rig);
    CHECK(fresh == generation + 1);
    CHECK(RigExecPublishBackgroundCompletion(
        cache, stale, UsdTimeCode(4.0), pose, fresh, &scheduler, rig,
        RigExecWarmFenceToken(0), nullptr, 0, nullptr, &index));
    CHECK(cache->Lookup(stale, &served));
    CHECK(index.FindCachedKey(rig, 4.0, fresh, stale.epochDigest, &indexed));
    CHECK(indexed == stale);

    // Invalid poses and null caches never publish, with or without a fence.
    RigExecRigPose invalid;
    CHECK(!invalid.valid);
    const RigExecFrameCacheKey bad{7, 13};
    CHECK(!RigExecPublishBackgroundCompletion(
        cache, bad, UsdTimeCode(5.0), invalid, fresh, &scheduler, rig,
        RigExecWarmFenceToken(0), nullptr, 0, nullptr, &index));
    CHECK(!cache->Lookup(bad, &served));
    CHECK(!index.FindKey(rig, 5.0, &indexed));
    CHECK(!RigExecPublishBackgroundCompletion(
        std::shared_ptr<RigExecFrameCache>(), bad, UsdTimeCode(5.0), pose,
        fresh, &scheduler, rig));
    const RigExecFrameCacheKey unfenced{7, 14};
    CHECK(RigExecPublishBackgroundCompletion(
        cache, unfenced, UsdTimeCode(6.0), pose, fresh,
        nullptr, rig, RigExecWarmFenceToken(0), nullptr, 0, nullptr, &index));
    CHECK(cache->Lookup(unfenced, &served));
    CHECK(index.FindCachedKey(rig, 6.0, fresh, unfenced.epochDigest, &indexed));
    CHECK(indexed == unfenced);

    // A rejected insertion must not make the index advertise a cached frame.
    cache->SetByteCap(0);
    CHECK(!RigExecPublishBackgroundCompletion(
        cache, key, UsdTimeCode(7.0), pose, fresh, &scheduler, rig,
        RigExecWarmFenceToken(0), nullptr, 0, nullptr, &index));
    CHECK(!index.FindKey(rig, 7.0, &indexed));
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

// SETTLE. A release that commits exactly the previewed pose evaluates
// nothing -- the published generation is already the stage's answer -- and
// a commit of anything else evaluates as before.
void
TestCommitOfPreviewedPoseSkipsEvaluation()
{
    std::printf("progress: TestCommitOfPreviewedPoseSkipsEvaluation\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    UsdAttribute tx =
        stage->GetAttributeAtPath(SdfPath("/Asset/Rig/AlongX.avars:tx"));

    // The committed value is the previewed one: no evaluation, and the
    // kept generation matches a fresh evaluation of the edited stage.
    CHECK(registry.BeginPreview("/Asset/Rig/AlongX.avars:tx") == 1);
    const double sample = 11.0;
    CHECK(registry.UpdatePreview(&sample, 1));
    const _GenerationGeometry previewed =
        _CaptureGeometry(registry.GetStore()->Get());
    const size_t pulls = registry.GetSessionEvaluationCount(rig);
    CHECK(registry.EndPreview(/* publish = */ false));
    tx.Set(sample, UsdTimeCode(2.0));
    CHECK(registry.GetSessionEvaluationCount(rig) == pulls);
    CHECK(_SameGeometry(previewed,
                        _CaptureGeometry(registry.GetStore()->Get())));
    {
        RigExecImagingBridge fresh(stage, rig);
        CHECK(fresh.Compile());
        CHECK(fresh.EvaluateAndPublishResult(UsdTimeCode(2.0)).ok);
        CHECK(_SameGeometry(previewed,
                            _CaptureGeometry(fresh.GetStore()->Get())));
    }

    // A different committed value evaluates, and shows the stage's value.
    CHECK(registry.BeginPreview("/Asset/Rig/AlongX.avars:tx") == 1);
    const double dragged = 12.5;
    CHECK(registry.UpdatePreview(&dragged, 1));
    const _GenerationGeometry draggedPose =
        _CaptureGeometry(registry.GetStore()->Get());
    const size_t before = registry.GetSessionEvaluationCount(rig);
    CHECK(registry.EndPreview(/* publish = */ false));
    tx.Set(12.0, UsdTimeCode(2.0));
    CHECK(registry.GetSessionEvaluationCount(rig) == before + 1);
    CHECK(!_SameGeometry(draggedPose,
                         _CaptureGeometry(registry.GetStore()->Get())));

    // A withdrawn preview nobody commits still goes back to the stage.
    const _GenerationGeometry authored =
        _CaptureGeometry(registry.GetStore()->Get());
    CHECK(registry.BeginPreview("/Asset/Rig/AlongX.avars:tx") == 1);
    CHECK(registry.UpdatePreview(&dragged, 1));
    CHECK(registry.EndPreview(/* publish = */ false));
    CHECK(registry.SetTime(UsdTimeCode(2.0)));
    CHECK(_SameGeometry(authored,
                        _CaptureGeometry(registry.GetStore()->Get())));
    registry.ClearFrameCache(rig);
    registry.Deactivate();
}

// First-session ancestor overs do not invalidate an exact preview commit.
void
TestFirstSessionCommitOfPreviewedPose()
{
    std::printf("progress: TestFirstSessionCommitOfPreviewedPose\n");
    SetEnv("RIGEXEC_FRAME_CACHE", "off");
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    for (int variant = 0; variant != 7; ++variant) {
        UsdStageRefPtr stage = MakeTinyRig();
        const UsdAttribute shape = stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongX"))
            .CreateAttribute(TfToken("guide:shape"), SdfValueTypeNames->Token);
        shape.Set(TfToken("cube"));
        stage->SetEditTarget(stage->GetSessionLayer());
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
        const UsdAttribute tx = stage->GetAttributeAtPath(
            SdfPath("/Asset/Rig/AlongX.avars:tx"));
        CHECK(registry.BeginPreview("/Asset/Rig/AlongX.avars:tx") == 1);
        const double sample = 11.0;
        CHECK(registry.UpdatePreview(&sample, 1));
        const _GenerationGeometry previewed =
            _CaptureGeometry(registry.GetStore()->Get());
        const size_t pulls = registry.GetSessionEvaluationCount(rig);
        CHECK(registry.EndPreview(/*publish=*/false));
        {
            SdfChangeBlock changes;
            TsSpline spline(tx.GetTypeName().GetType());
            TsKnot knot(tx.GetTypeName().GetType());
            knot.SetTime(2.0);
            knot.SetValue(variant == 4 ? 12.0 : sample);
            spline.SetKnot(knot);
            CHECK(tx.SetSpline(spline));
            if (variant == 1) {
                stage->GetPrimAtPath(rig).SetMetadata(TfToken("kind"),
                                                     TfToken("assembly"));
            } else if (variant == 2) {
                tx.SetCustom(true);
            } else if (variant == 3) {
                stage->GetAttributeAtPath(SdfPath("/Asset/Rig/AlongY.avars:ty"))
                    .Set(25.0, UsdTimeCode(2.0));
            } else if (variant == 5) {
                shape.Set(TfToken("sphere"));
            } else if (variant == 6) {
                const SdfPrimSpecHandle other = SdfCreatePrimInLayer(
                    stage->GetSessionLayer(), SdfPath("/Asset/Rig/Other"));
                other->SetSpecifier(SdfSpecifierDef);
                other->SetTypeName("Scope");
            }
        }
        const size_t committedPulls = registry.GetSessionEvaluationCount(rig);
        if (committedPulls != pulls + (variant == 0 ? 0 : 1)) {
            std::fprintf(stderr, "settlement variant %d: pulls %zu -> %zu\n",
                         variant, pulls, committedPulls);
        }
        CHECK(committedPulls == pulls + (variant == 0 ? 0 : 1));
        double authoredValue = 0.0;
        CHECK(tx.Get(&authoredValue, UsdTimeCode(2.0)));
        CHECK(authoredValue == (variant == 4 ? 12.0 : sample));
        const _GenerationGeometry committed =
            _CaptureGeometry(registry.GetStore()->Get());
        RigExecImagingBridge fresh(stage, rig);
        CHECK(fresh.Compile());
        CHECK(fresh.EvaluateAndPublishResult(UsdTimeCode(2.0)).ok);
        const _GenerationGeometry expected = _CaptureGeometry(fresh.GetStore()->Get());
        if (!_SameGeometry(committed, expected)) {
            std::fprintf(stderr, "settlement geometry variant %d\n", variant);
            for (const auto &[path, points] : committed.points) {
                const auto &reference = expected.points.at(path);
                std::fprintf(stderr, "point0 live %.9g %.9g %.9g fresh %.9g %.9g %.9g\n",
                    points[0][0], points[0][1], points[0][2],
                    reference[0][0], reference[0][1], reference[0][2]);
            }
        }
        CHECK(_SameGeometry(committed, expected));
        if (variant == 0) {
            CHECK(_SameGeometry(previewed, committed));
            // Settlement is consumed once. An ordinary authored edit after
            // it must not be mistaken for another released preview.
            const size_t ordinaryPulls = registry.GetSessionEvaluationCount(rig);
            tx.Set(12.0, UsdTimeCode(2.0));
            CHECK(registry.GetSessionEvaluationCount(rig) == ordinaryPulls + 1);
        } else if (variant == 3 || variant == 4) {
            CHECK(!_SameGeometry(previewed, committed));
        }
        if (variant == 4) {
            // A rebuilt baked program can be used again. A subsequent
            // exact release still settles, and removing the session opinions
            // (the undo of a first write) restores the original authored pose.
            CHECK(registry.BeginPreview("/Asset/Rig/AlongX.avars:tx") == 1);
            const double next = 13.0;
            CHECK(registry.UpdatePreview(&next, 1));
            const size_t nextPulls = registry.GetSessionEvaluationCount(rig);
            CHECK(registry.EndPreview(/*publish=*/false));
            TsSpline spline(tx.GetTypeName().GetType());
            TsKnot knot(tx.GetTypeName().GetType());
            knot.SetTime(2.0);
            knot.SetValue(next);
            spline.SetKnot(knot);
            CHECK(tx.SetSpline(spline));
            CHECK(registry.GetSessionEvaluationCount(rig) == nextPulls);
            stage->GetSessionLayer()->Clear();
            CHECK(registry.GetSessionEvaluationCount(rig) == nextPulls + 1);
            CHECK(tx.Get(&authoredValue, UsdTimeCode(2.0)));
            CHECK(authoredValue == 10.2);
            RigExecImagingBridge restored(stage, rig);
            CHECK(restored.Compile());
            CHECK(restored.EvaluateAndPublishResult(UsdTimeCode(2.0)).ok);
            CHECK(_SameGeometry(_CaptureGeometry(registry.GetStore()->Get()),
                                _CaptureGeometry(restored.GetStore()->Get())));
        }
        registry.ClearFrameCache(rig);
        registry.Deactivate();
    }
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
}

// A chain revises the preview and authored value as its base in both cases.
void
TestCommitOfAChainTargetSettles()
{
    std::printf("progress: TestCommitOfAChainTargetSettles\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    UsdStageRefPtr stage = MakeTinyRig();
    const UsdPrim gain = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/TxGain"), TfToken("RigExecFloatMathMover"));
    gain.ApplyAPI(TfToken("RigExecMoverAPI"));
    gain.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Rig/AlongX.avars:tx")});
    gain.CreateAttribute(TfToken("rigExec:operation"), SdfValueTypeNames->Token)
        .Set(TfToken("multiply"));
    gain.CreateAttribute(TfToken("inputs:defaultWeight"),
                         SdfValueTypeNames->Float).Set(1.0f);
    gain.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .Set(2.0f);
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(2.0), &errors));
    UsdAttribute tx =
        stage->GetAttributeAtPath(SdfPath("/Asset/Rig/AlongX.avars:tx"));

    CHECK(registry.BeginPreview("/Asset/Rig/AlongX.avars:tx") == 1);
    const double sample = 11.0;
    CHECK(registry.UpdatePreview(&sample, 1));
    const _GenerationGeometry previewed =
        _CaptureGeometry(registry.GetStore()->Get());
    const size_t pulls = registry.GetSessionEvaluationCount(rig);
    CHECK(registry.EndPreview(/* publish = */ false));
    tx.Set(sample, UsdTimeCode(2.0));
    CHECK(registry.GetSessionEvaluationCount(rig) == pulls);
    CHECK(_SameGeometry(previewed,
                        _CaptureGeometry(registry.GetStore()->Get())));
    {
        RigExecImagingBridge fresh(stage, rig);
        CHECK(fresh.Compile());
        CHECK(fresh.EvaluateAndPublishResult(UsdTimeCode(2.0)).ok);
        CHECK(_SameGeometry(previewed,
                            _CaptureGeometry(fresh.GetStore()->Get())));
    }
    registry.ClearFrameCache(rig);
    registry.Deactivate();
    // The chain revised the drag: the previewed pose is the doubled sample
    // authored on a rig without the chain.
    tx.Set(2.0 * sample, UsdTimeCode(2.0));
    stage->RemovePrim(gain.GetPath());
    RigExecImagingBridge unrevised(stage, rig);
    CHECK(unrevised.Compile());
    CHECK(unrevised.EvaluateAndPublishResult(UsdTimeCode(2.0)).ok);
    // The points only: the chain's own result is a moved value only the
    // chained rig publishes.
    _GenerationGeometry previewedPoints = previewed;
    _GenerationGeometry unrevisedPoints =
        _CaptureGeometry(unrevised.GetStore()->Get());
    previewedPoints.movedFloats.clear();
    unrevisedPoints.movedFloats.clear();
    CHECK(!previewedPoints.points.empty());
    CHECK(_SameGeometry(previewedPoints, unrevisedPoints));
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
MakeConnectedSpaceRig()
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

    // The connection is to the control's own posed:space, which
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

// The program bakes, but its weighted property chain reads a live weight
// oracle and therefore cannot freeze its complete inputs.
UsdStageRefPtr
MakePropertyChainRig()
{
    UsdStageRefPtr stage = MakeTinyRig();
    const UsdPrim gain = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/TxGain"), TfToken("RigExecFloatMathMover"));
    gain.ApplyAPI(TfToken("RigExecMoverAPI"));
    gain.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Rig/AlongX.avars:tx")});
    gain.CreateAttribute(TfToken("rigExec:operation"), SdfValueTypeNames->Token)
        .Set(TfToken("multiply"));
    const UsdAttribute value = gain.CreateAttribute(
        TfToken("inputs:value"), SdfValueTypeNames->Float);
    for (int t = 1; t <= 4; ++t) {
        value.Set(0.5f * float(t), UsdTimeCode(double(t)));
    }
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Rig/AlongX.avars:tx")});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token).Set(TfToken("constant"));
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float).Set(0.5f);
    gain.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({weight.GetPath()});
    return stage;
}

void
TestPropertyChainRigMemoizesCompletePoses()
{
    std::printf("progress: TestPropertyChainRigMemoizesCompletePoses\n");
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() == RigExecFrameCacheMode::Off) {
        return;
    }
    const UsdStageRefPtr stage = MakePropertyChainRig();
    RigExecImagingBridge bridge(stage, SdfPath("/Asset/Rig"));
    CHECK(bridge.Compile());
    std::map<double, _GenerationGeometry> live;
    for (double time : {1.0, 2.0, 3.0, 4.0}) {
        const auto result = bridge.EvaluateAndPublishResult(UsdTimeCode(time));
        CHECK(result.ok && !result.cacheHit);
        live[time] = _CaptureGeometry(bridge.GetStore()->Get());
        RigExecFreshProof proof;
        CHECK(bridge.GetFreshProof(UsdTimeCode(time), &proof));
        CHECK(proof.sampledInputs);
    }
    CHECK(bridge.GetEvaluator().GetBakedProgram() != nullptr);
    CHECK(RigExecCanFreezeProgram(bridge.GetEvaluator()));
    CHECK(!_SameGeometry(live[1.0], live[2.0]));
    for (double time : {3.0, 1.0, 4.0, 2.0}) {
        const auto result = bridge.EvaluateAndPublishResult(UsdTimeCode(time));
        CHECK(result.ok && result.cacheHit);
        CHECK(_SameGeometry(live[time], _CaptureGeometry(bridge.GetStore()->Get())));
    }
    // A value omitted by the frozen contract still invalidates every pose
    // through the stage serial, and the edited result memoizes again.
    stage->GetAttributeAtPath(SdfPath("/Asset/Rig/Movers/TxGain.inputs:value"))
        .Set(8.0f, UsdTimeCode(2.0));
    const auto changed = bridge.EvaluateAndPublishResult(UsdTimeCode(2.0));
    CHECK(changed.ok && !changed.cacheHit);
    CHECK(!_SameGeometry(live[2.0], _CaptureGeometry(bridge.GetStore()->Get())));
    const auto repeated = bridge.EvaluateAndPublishResult(UsdTimeCode(2.0));
    CHECK(repeated.ok && repeated.cacheHit);
}

void
TestSupportedSourcesWarmCompletePoses()
{
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (!RigExecBackgroundWarmingEnabled()) return;
    const SdfPath rig("/Asset/Rig");
    const std::vector<double> range{1.0, 2.0, 3.0, 4.0};
    auto &registry = RigExecImagingRegistry::GetInstance();
    for (bool propertyChain : {false, true}) {
        const auto stage = propertyChain ? MakePropertyChainRig() : MakeConnectedSpaceRig();
        std::string sourceBefore;
        stage->GetRootLayer()->ExportToString(&sourceBefore);
        RigExecImagingBridge reference(stage, rig);
        CHECK(reference.Compile());
        std::map<double, _GenerationGeometry> expected;
        for (double time : range) {
            CHECK(reference.EvaluateAndPublishResult(UsdTimeCode(time)).ok);
            expected[time] = _CaptureGeometry(reference.GetStore()->Get());
        }
        CHECK(RigExecCanFreezeProgram(reference.GetEvaluator()));
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetWarmRange(rig, range));
        const auto snapshot = registry.GetStore()->Get();
        for (int tick = 0; tick != 8; ++tick) {
            registry.OnIdle();
            registry.WaitUntilBackgroundIdle();
        }
        CHECK(registry.GetStore()->Get() == snapshot);
        for (auto state : registry.GetFrameStates(rig, range)) {
            CHECK(state == RigExecWarmFrameState::Cached);
        }
        for (double time : range) {
            CHECK(registry.SetTime(UsdTimeCode(time)));
            CHECK(_SameGeometry(expected[time], _CaptureGeometry(registry.GetStore()->Get())));
        }
        std::string sourceAfter;
        stage->GetRootLayer()->ExportToString(&sourceAfter);
        CHECK(sourceBefore == sourceAfter);
        registry.Deactivate();
    }
}

// Connected source-space values participate in cache keys and edit invalidation.
void
TestConnectedSpaceRigMemoizesResults()
{
    std::printf("progress: TestConnectedSpaceRigMemoizesResults\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    const bool verify = RigExecFrameCacheVerifyRequested();
    const bool readsOn =
        RigExecFrameCacheModeFromEnvironment() != RigExecFrameCacheMode::Off;

    UsdStageRefPtr stage = MakeConnectedSpaceRig();
    const SdfPath rig("/Asset/Rig");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    CHECK(registry.GetSessionEvaluationCount(rig) == 1);

    const auto work = registry.BuildWarmWork(
        rig, UsdTimeCode(2.0), registry.CurrentFrameGeneration(rig),
        RigExecFrozenStepRunner());
    CHECK(work.work);

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
        std::printf("connected-space frames do not differ: the rig is static\n");
        CHECK(false);
    }
    const RigExecFrameCacheStats coldStats =
        registry.GetFrameCacheStats(rig);
    if (readsOn) {
        CHECK(coldStats.published == 4);
        CHECK(coldStats.hits == 0);
        // The explicitly built but unrun frame2 job installed its fresh
        // input proof. The cold proven-key consultation misses once.
        CHECK(coldStats.misses == 1);
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
            std::printf("connected-space generation differs at frame %g\n", frame);
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
        // Warm revisits add no misses to the earlier cold consultation.
        CHECK(warmStats.misses == coldStats.misses);
    }

    // An edit retires the connected-space memos: the stage-edit serial folded into
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
            std::printf("connected-space edit did not move the frame-2 pose\n");
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
            std::printf("connected-space re-memoized pose differs at frame 2\n");
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

// Connected source-space rigs warm through the production graph runner.
void
TestConnectedSpaceRigWarmsRange()
{
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (!RigExecBackgroundWarmingEnabled()) return;
    const SdfPath rig("/Asset/Rig");
    const std::vector<double> range{1.0, 2.0, 3.0, 4.0};
    auto &registry = RigExecImagingRegistry::GetInstance();
    for (bool propertyChain : {false}) {
        const auto stage = propertyChain ? MakePropertyChainRig() : MakeConnectedSpaceRig();
        std::string sourceBefore;
        stage->GetRootLayer()->ExportToString(&sourceBefore);
        RigExecImagingBridge reference(stage, rig);
        CHECK(reference.Compile());
        std::map<double, _GenerationGeometry> expected;
        for (double time : range) {
            CHECK(reference.EvaluateAndPublishResult(UsdTimeCode(time)).ok);
            expected[time] = _CaptureGeometry(reference.GetStore()->Get());
        }
        CHECK(RigExecCanFreezeProgram(reference.GetEvaluator()));
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetWarmRange(rig, range));
        const auto snapshot = registry.GetStore()->Get();
        for (int tick = 0; tick != 8; ++tick) {
            registry.OnIdle();
            registry.WaitUntilBackgroundIdle();
        }
        CHECK(registry.GetStore()->Get() == snapshot);
        for (auto state : registry.GetFrameStates(rig, range)) {
            CHECK(state == RigExecWarmFrameState::Cached);
        }
        for (double time : range) {
            CHECK(registry.SetTime(UsdTimeCode(time)));
            CHECK(_SameGeometry(expected[time], _CaptureGeometry(registry.GetStore()->Get())));
        }
        std::string sourceAfter;
        stage->GetRootLayer()->ExportToString(&sourceAfter);
        CHECK(sourceBefore == sourceAfter);
        registry.Deactivate();
    }
}

// A Copy Frame provider poses the joint that a Layered Skin Mover (a core
// external revision) skins with a per-point mask. The animated source makes
// every frame distinct.
UsdStageRefPtr
MakeAffineLayeredSkinRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim source = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Source"), TfToken("RigExecControl"));
    const UsdAttribute tx = source.GetAttribute(TfToken("avars:tx"));
    for (int t = 1; t <= 4; ++t) {
        CHECK(tx.Set(2.0 + 0.5 * double(t), UsdTimeCode(double(t))));
    }
    const UsdPrim copy = stage->DefinePrim(
        SdfPath("/Asset/Rig/Computations/Copy"), TfToken("RigExecCopyFrame"));
    CHECK(copy.GetRelationship(TfToken("rigExec:source"))
              .SetTargets({source.GetPath()}));
    CHECK(copy.GetRelationship(TfToken("rigExec:poseInputs"))
              .SetTargets({source.GetPath()}));
    const UsdPrim joint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Joints/Influence"), TfToken("RigExecJoint"));
    CHECK(joint.CreateAttribute(TfToken("posed:space"),
                                SdfValueTypeNames->Matrix4d)
              .SetConnections({copy.GetPath().AppendProperty(
                  TfToken("outputs:matrix"))}));
    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const SdfPath cloud("/Asset/Geom/Cloud");
    const UsdPrim points = stage->DefinePrim(cloud, TfToken("Points"));
    CHECK(points.GetAttribute(TfToken("points"))
              .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0, 1, 0)}));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"),
        TfToken("RigExecLayeredSkinMover"));
    CHECK(skin.ApplyAPI(TfToken("RigExecMoverAPI")));
    CHECK(skin.GetRelationship(TfToken("rigExec:moves"))
              .SetTargets({cloud.AppendProperty(TfToken("points"))}));
    CHECK(skin.GetRelationship(TfToken("rigExec:influences"))
              .SetTargets({joint.GetPath()}));
    CHECK(skin.GetAttribute(TfToken("rigExec:jointIndices"))
              .Set(VtIntArray{0, 0}));
    CHECK(skin.GetAttribute(TfToken("rigExec:jointWeights"))
              .Set(VtFloatArray{1.0f, 1.0f}));
    CHECK(skin.GetAttribute(TfToken("inputs:mask"))
              .Set(VtFloatArray{0.25f, 0.75f}));
    return stage;
}

// Affine frame providers and core external movers warm on the frame-cache
// workers like every other operation: the range fills without an
// owner-thread pull and serves the reference poses bit for bit.
void
TestAffineProvidersAndExternalMoversWarmFrozen()
{
    std::printf("progress: TestAffineProvidersAndExternalMoversWarmFrozen\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (!RigExecBackgroundWarmingEnabled()) return;
    const SdfPath rig("/Asset/Rig");
    const std::vector<double> range{1.0, 2.0, 3.0, 4.0};
    auto &registry = RigExecImagingRegistry::GetInstance();
    const UsdStageRefPtr stage = MakeAffineLayeredSkinRig();
    std::string sourceBefore;
    stage->GetRootLayer()->ExportToString(&sourceBefore);
    RigExecImagingBridge reference(stage, rig);
    CHECK(reference.Compile());
    std::map<double, _GenerationGeometry> expected;
    for (double time : range) {
        CHECK(reference.EvaluateAndPublishResult(UsdTimeCode(time)).ok);
        expected[time] = _CaptureGeometry(reference.GetStore()->Get());
    }
    CHECK(!expected[1.0].points.empty());
    CHECK(!_SameGeometry(expected[1.0], expected[2.0]));
    std::string freezeError;
    const bool freezes =
        RigExecCanFreezeProgram(reference.GetEvaluator(), &freezeError);
    if (!freezes) {
        std::printf("affine/layered skin rig cannot freeze: %s\n",
                    freezeError.c_str());
    }
    CHECK(freezes);
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    CHECK(registry.SetWarmRange(rig, range));
    const auto snapshot = registry.GetStore()->Get();
    const size_t pulls = registry.GetSessionEvaluationCount(rig);
    for (int tick = 0; tick != 8; ++tick) {
        registry.OnIdle();
        registry.WaitUntilBackgroundIdle();
    }
    CHECK(registry.GetStore()->Get() == snapshot);
    for (auto state : registry.GetFrameStates(rig, range)) {
        CHECK(state == RigExecWarmFrameState::Cached);
    }
    for (double time : range) {
        CHECK(registry.SetTime(UsdTimeCode(time)));
        CHECK(_SameGeometry(expected[time],
                            _CaptureGeometry(registry.GetStore()->Get())));
    }
    // Neither the idle owner-thread fill nor the visits pulled the live
    // evaluator: the workers produced every frame.
    if (!RigExecFrameCacheVerifyRequested()) {
        CHECK(registry.GetSessionEvaluationCount(rig) == pulls);
    }
    std::string sourceAfter;
    stage->GetRootLayer()->ExportToString(&sourceAfter);
    CHECK(sourceBefore == sourceAfter);
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

// Progress is bounded to a request and never grants a cache hit. Eviction,
// failure, late request tokens, epoch changes and cancellation are distinct.
void TestWarmRangeProgress()
{
    RigExecWarmFrameIndex index;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameCacheKey key{7, 9};
    index.NoteGeneration(rig, 3);
    index.SetRequest(rig, {1.0, 2.0}, 3, 7);
    const uint64_t first = index.RequestToken(rig);
    index.NoteCompleted(rig, 1.0, key, 3);
    index.NoteRequestPublished(rig, 1.0, key, 3, first);
    index.NoteEvicted(rig, key, 1.0);
    CHECK(index.State(rig, 1.0, 3, 7) == RigExecWarmFrameState::Uncached);
    RigExecFrameCacheKey found;
    CHECK(!index.FindCachedKey(rig, 1.0, 3, 7, &found));
    CHECK(index.IsVisitable(rig, 1.0, 3, 7)); // explicit foreground miss
    CHECK(!index.IsCursorVisitable(rig, 1.0, 3, 7)); // finite range progressed
    CHECK(index.IsCursorVisitable(rig, 2.0, 3, 7)); // never published
    CHECK(index.IsCursorVisitable(rig, 1.0, 3, 8)); // different epoch
    index.InvalidateRequest(rig);
    index.NoteRequestPublished(rig, 1.0, key, 3, first); // late fenced job
    CHECK(index.IsCursorVisitable(rig, 1.0, 3, 7));
    const uint64_t second = index.RequestToken(rig);
    index.NoteRequestPublished(rig, 1.0, key, 2, second); // stale generation
    CHECK(index.IsCursorVisitable(rig, 1.0, 3, 7));
    index.NoteRequestPublished(rig, 1.0, key, 3, second);
    CHECK(!index.IsCursorVisitable(rig, 1.0, 3, 7));
    index.NoteDirtied(rig, 1.0); // source change after eviction still matters
    CHECK(index.IsCursorVisitable(rig, 1.0, 3, 7));
    index.SetRequest(rig, {2.0}, 3, 7);
    index.NoteRequestPublished(rig, 1.0, key, 3, index.RequestToken(rig));
    CHECK(index.IsCursorVisitable(rig, 1.0, 3, 7)); // outside request ignored
    RigExecWarmTransition failed;
    failed.rig = rig; failed.timeValue = 2.0;
    failed.kind = RigExecWarmTransitionKind::Finished;
    failed.outcome = RigExecWarmOutcome::DeclinedInvalid;
    index.NoteTransition(failed);
    CHECK(index.IsCursorVisitable(rig, 2.0, 3, 7)); // failure is not progress
    index.NoteGeneration(rig, 4);
    index.NoteRequestPublished(rig, 2.0, key, 3, index.RequestToken(rig));
    CHECK(index.IsCursorVisitable(rig, 2.0, 4, 7));
    index.NoteRequestPublished(rig, 2.0, key, 4, index.RequestToken(rig));
    CHECK(!index.IsCursorVisitable(rig, 2.0, 4, 7)); // recovery
    index.ResetRig(rig);
    CHECK(index.RequestToken(rig) == 0);
    CHECK(index.IsCursorVisitable(rig, 2.0, 4, 7));
}

// STACK. The full-range cursor over the layered stack anim: a registry-level
// idle driver (the headless shape of the plugin's recurring driver) warms
// every in-range frame -- published grows once per warmed frame, no trigger
// exceeds the sampling budget, the stack declines nothing, warmed frames
// each published job result reads back exactly, retained entries serve
// without evaluating, and spread background poses are bit-identical to live.
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
    // Every requested frame must actually publish and its full job result
    // must survive cache transport exactly before any later eviction.
    // Only compact keys/counts survive the callback, not 200 full poses.
    struct PublishedFrame {
        RigExecFrameCacheKey key{0, 0};
        size_t count = 0;
    };
    const std::vector<double> spots = {
        frames[frames.size() / 4], frames[frames.size() / 2],
        frames[3 * frames.size() / 4]};
    std::map<double, uint64_t> backgroundDigests;
    const auto poseDigest = [](const RigExecRigPose &pose, uint64_t *digest) {
        std::vector<RigExecGoldenValue> values;
        if (!RigExecEncodeGoldenPose(pose, &values)) return false;
        *digest = RigExecGoldenDigest(RigExecGoldenVisit(
            "background-native", 0, pose, values, true));
        return true;
    };
    struct PublicationCell {
        std::atomic<uint8_t> state{0}; // one claim; release-ready after payload
        std::atomic<size_t> count{0};
        RigExecFrameCacheKey key{0, 0};
        uint64_t digest = 0;
        bool exact = false;
        bool encoded = false;
        bool spot = false;
    };
    static_assert(std::atomic<uint8_t>::is_always_lock_free &&
                  std::atomic<size_t>::is_always_lock_free,
                  "publication observations must not introduce hidden locks");
    std::unique_ptr<PublicationCell[]> publicationCells(new PublicationCell[frames.size()]);
    std::map<double, PublishedFrame> publications; // owner only
    std::atomic<bool> publicationFailure{false};
    static_assert(std::atomic<bool>::is_always_lock_free,
                  "publication failure flag must be lock-free");
    const auto mergePublications = [&]() {
        for (size_t i = 0; i < frames.size(); ++i) {
            const auto &cell = publicationCells[i];
            if (cell.state.load(std::memory_order_acquire) != 2) continue;
            if (cell.spot && cell.exact && cell.encoded)
                backgroundDigests[frames[i]] = cell.digest;
            if (cell.exact) {
                auto &row = publications[frames[i]];
                row.key = cell.key;
                row.count = cell.count.load(std::memory_order_relaxed);
            }
        }
    };
    const RigExecFrameGeneration generation = registry.CurrentFrameGeneration(rig);
    registry.SetWarmPublishObserverForTesting(
        [&](const SdfPath &publishedRig, UsdTimeCode time,
            const RigExecFrameCacheKey &key, RigExecFrameGeneration token,
            const RigExecRigPose &result,
            const std::shared_ptr<RigExecFrameCache> &cache) {
            if (!time.IsNumeric()) {
                publicationFailure.store(true, std::memory_order_relaxed);
                return;
            }
            const auto position = std::lower_bound(frames.begin(), frames.end(), time.GetValue());
            if (position == frames.end() || *position != time.GetValue()) {
                publicationFailure.store(true, std::memory_order_relaxed);
                return;
            }
            auto &cell = publicationCells[size_t(position - frames.begin())];
            cell.count.fetch_add(1, std::memory_order_relaxed);
            uint8_t empty = 0;
            if (!cell.state.compare_exchange_strong(empty, uint8_t(1),
                    std::memory_order_relaxed, std::memory_order_relaxed)) {
                // Duplicate observation never blocks or overwrites payload.
                publicationFailure.store(true, std::memory_order_relaxed);
                return;
            }
            RigExecRigPose served, comparison;
            const bool found = cache && cache->Lookup(key, &served);
            if (found) RigExecComparePoses(result, served, &comparison);
            const bool exact = publishedRig == rig && time.IsNumeric() &&
                token == generation && result.valid && found && served.valid &&
                result.time == time && served.time == time &&
                comparison.comparisonMismatches == 0 &&
                std::binary_search(frames.begin(), frames.end(), time.GetValue());
            uint64_t digest = 0;
            const bool spot = std::find(spots.begin(), spots.end(),
                time.GetValue()) != spots.end();
            const bool encoded = !spot || poseDigest(result, &digest);
            cell.key = key;
            cell.digest = digest;
            cell.spot = spot;
            cell.exact = exact;
            cell.encoded = encoded;
            if (!encoded || !exact) publicationFailure.store(true, std::memory_order_relaxed);
            cell.state.store(2, std::memory_order_release);
        });
    // Activate published the playhead natively. Run one genuine default
    // closure for that same input/time to verify its cache transport too;
    // the range cursor correctly never queues the playhead itself.
    RigExecWarmFactoryResult playheadWork = registry.BuildWarmWork(
        rig, UsdTimeCode(start), generation, RigExecFrozenStepRunner());
    CHECK(static_cast<bool>(playheadWork.work));
    RigExecWarmRequest playheadRequest;
    playheadRequest.rig = rig;
    playheadRequest.time = UsdTimeCode(start);
    playheadRequest.generation = generation;
    playheadRequest.fenceToken = registry.CurrentFenceToken(rig, UsdTimeCode(start));
    if (playheadWork.work)
        CHECK(playheadWork.work(playheadRequest) == RigExecWarmOutcome::Published);
    // The 1 GiB logical cap deliberately cannot retain all 200 stack frames.
    if (!registry.SetWarmRange(rig, frames)) {
        CHECK(false);
        registry.SetWarmPublishObserverForTesting({});
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
    // Drive the existing scheduler until all unique requested publications
    // have been verified, rather than reheating already completed evictions.
    const size_t maxTicks = frames.size() + 8;
    size_t ticks = 0;
    for (; ticks < maxTicks; ++ticks) {
        mergePublications();
        if (publications.size() == frames.size()) break;
        const size_t invocationsBefore =
            registry.GetBackgroundStats().factoryInvocations;
        const int enqueued = registry.OnIdle();
        CHECK(enqueued >= 0);
        CHECK(enqueued >= 0 && static_cast<size_t>(enqueued) <= budget);
        CHECK(registry.GetBackgroundStats().factoryInvocations -
                  invocationsBefore <= budget);
        registry.WaitUntilBackgroundIdle();
    }
    CHECK(ticks < maxTicks);
    registry.SetWarmPublishObserverForTesting({}); // all jobs drained
    mergePublications();
    CHECK(!publicationFailure.load(std::memory_order_relaxed));
    CHECK(publications.size() == frames.size());
    for (const double time : frames) {
        const auto found = publications.find(time);
        CHECK(found != publications.end());
        if (found != publications.end()) CHECK(found->second.count == 1);
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
    const RigExecFrameCacheStats cacheStats = registry.GetFrameCacheStats(rig);
    CHECK(cacheStats.bytes <= kRigExecFrameCacheDefaultByteCap);
    CHECK(cacheStats.evictions > 0);
    CHECK(cacheStats.entryCount < frames.size());
    // Still-resident publications must serve without an evaluator pull.
    // Progress alone is never considered cached: evicted frames are absent.
    const auto retained = registry.GetCompletedKeys(rig);
    CHECK(retained.size() == cacheStats.entryCount);
    const size_t evalsBefore = registry.GetSessionEvaluationCount(rig);
    const size_t hitsBefore = registry.GetFrameCacheStats(rig).hits;
    for (const auto &[time, key] : retained) {
        const auto found = publications.find(time.GetValue());
        CHECK(found != publications.end());
        if (found != publications.end()) CHECK(found->second.key == key);
        CHECK(registry.SetTime(time));
    }
    CHECK(registry.GetSessionEvaluationCount(rig) == evalsBefore);
    const size_t hits = registry.GetFrameCacheStats(rig).hits - hitsBefore;
    CHECK(hits == retained.size() || hits + 1 == retained.size());
    // Warmed-vs-live bit identity on three spread frames: capture warmed,
    // clear, re-evaluate live, compare exactly.
    std::map<double, _GenerationGeometry> warmed;
    size_t spotMisses = 0;
    for (const double frame : spots) {
        const auto state = registry.GetFrameStates(rig, {frame});
        CHECK(state.size() == 1);
        if (state.size() == 1 && state[0] != RigExecWarmFrameState::Cached)
            ++spotMisses;
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        warmed[frame] = _CaptureGeometry(registry.GetStore()->Get());
        CHECK(!warmed[frame].points.empty());
    }
    // A foreground visit to an evicted completed frame must evaluate;
    // a resident one still hits. The following three exact live comparisons
    // retain the original independent numeric/sensitivity witness.
    const size_t evalsAfterWarmSpots = registry.GetSessionEvaluationCount(rig);
    CHECK(evalsAfterWarmSpots == evalsBefore + spotMisses);
    registry.ClearFrameCache(rig);
    for (const double frame : spots) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        const _GenerationGeometry live =
            _CaptureGeometry(registry.GetStore()->Get());
        // The native generation just evaluated above is held here; its
        // publication maps are independently compared to the original
        // default-worker result, even when that cache entry was evicted.
        RigExecImagingBridge *bridge = registry.GetBridge(rig);
        CHECK(bridge != nullptr);
        uint64_t liveDigest = 0;
        CHECK(backgroundDigests.count(frame) == 1);
        if (bridge) {
            auto &evaluator = const_cast<RigExecRigEvaluator &>(bridge->GetEvaluator());
            CHECK(poseDigest(evaluator.Evaluate(UsdTimeCode(frame)),
                &liveDigest));
            CHECK(backgroundDigests[frame] == liveDigest);
        }
        if (!_SameGeometry(warmed[frame], live)) {
            std::printf("warmed-vs-live differs at frame %g\n", frame);
            CHECK(false);
        }
    }
    CHECK(registry.GetSessionEvaluationCount(rig) ==
          evalsAfterWarmSpots + spots.size());
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

    // Routed (a time sample on a keyed avar is a per-frame input's value,
    // edited in place): affected retire plus re-resolve; the generation
    // stands.
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
                  RigExecNoticeDisposition::Edited);
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

    // Routed to nothing: a value on a prim inside the rig that nothing the
    // program reads. No completed frame retires and the generation stands,
    // where a stamp bump named the property as a foreign control and
    // retired them all.
    {
        std::vector<std::string> errors;
        UsdStageRefPtr stage = MakeTinyRig();
        const UsdAttribute note =
            stage->DefinePrim(SdfPath("/Asset/Rig/Notes"), TfToken("Scope"))
                .CreateAttribute(TfToken("note:weight"),
                                 SdfValueTypeNames->Float);
        CHECK(note.Set(0.0f));
        CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        const RigExecFrameGeneration gen =
            registry.CurrentFrameGeneration(rig);
        CHECK(note.Set(1.0f));
        RigExecImagingBridge *bridge = registry.GetBridge(rig);
        CHECK(bridge != nullptr);
        if (bridge) {
            CHECK(bridge->GetEvaluator().GetLastNoticeDisposition() ==
                  RigExecNoticeDisposition::Edited);
            CHECK(bridge->GetEvaluator().GetLastNoticePatchedPaths().empty());
        }
        CHECK(registry.CurrentFrameGeneration(rig) == gen);
        {
            const std::vector<RigExecWarmFrameState> got =
                registry.GetFrameStates(rig, {1.0, 2.0});
            CHECK(got.size() == 2);
            if (got.size() == 2) {
                CHECK(got[0] == cached);
                CHECK(got[1] == cached);
            }
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

// A first-fill worker must be retired by a value edit even after it has
// left the queue and before its first cache entry has been published.
void
TestRunningFirstFillEditDropsOldResult()
{
    std::printf("progress: TestRunningFirstFillEditDropsOldResult\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (!RigExecBackgroundWarmingEnabled()) {
        return;
    }
    const UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    const UsdTimeCode firstFill(2.0);
    auto &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
    CHECK(registry.SetWarmRange(rig, {1.0, 2.0}));
    const size_t savedInvocations = registry.GetWarmSamplingMaxInvocations();
    const double savedMs = registry.GetWarmSamplingMaxMs();
    registry.SetWarmSamplingBudget(1, std::numeric_limits<double>::infinity());
    const auto initial = registry.GetFrameStates(rig, {2.0});
    CHECK(initial.size() == 1 && initial[0] == RigExecWarmFrameState::Uncached);
    const RigExecFrameGeneration generation = registry.CurrentFrameGeneration(rig);
    const RigExecWarmFenceToken token = registry.CurrentFenceToken(rig, firstFill);

    // The worker has popped this first fill, so no queued job or completed
    // cache entry names it when the stage edit arrives.
    _PublishGate gate;
    CHECK(registry.OnIdle(_GatedKernel(&gate)) == 1);
    CHECK(_WaitFor([&gate] { return gate.entered.load() == 1; }, 5000));
    const auto running = registry.GetBackgroundStats();
    CHECK(running.running == 1 && running.queuedDepth == 0);
    CHECK(stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Movers/Skin_0.inputs:defaultWeight")).Set(0.0f));
    CHECK(registry.CurrentFrameGeneration(rig) == generation);
    CHECK(registry.CurrentFenceToken(rig, firstFill) != token);
    const auto editedPlayhead = registry.GetStore()->Get();
    const size_t publishedAfterEdit = registry.GetFrameCacheStats(rig).published;
    _ReleaseGate(&gate);
    registry.WaitUntilBackgroundIdle();
    const auto dropped = registry.GetBackgroundStats();
    CHECK(dropped.declinedGeneration == running.declinedGeneration + 1);
    CHECK(registry.GetFrameCacheStats(rig).published == publishedAfterEdit);
    CHECK(registry.GetStore()->Get() == editedPlayhead);
    const auto retired = registry.GetFrameStates(rig, {2.0});
    CHECK(retired.size() == 1 && retired[0] != RigExecWarmFrameState::Cached);

    // The next normal idle trigger retries the missing frame under its new
    // token, and the result agrees with a fresh evaluation of the edited rig.
    CHECK(registry.OnIdle() == 1);
    registry.WaitUntilBackgroundIdle();
    CHECK(registry.GetStore()->Get() == editedPlayhead);
    const auto rewarmed = registry.GetFrameStates(rig, {2.0});
    CHECK(rewarmed.size() == 1 && rewarmed[0] == RigExecWarmFrameState::Cached);
    const size_t pulls = registry.GetSessionEvaluationCount(rig);
    CHECK(registry.SetTime(firstFill));
    if (!RigExecFrameCacheVerifyRequested()) {
        CHECK(registry.GetSessionEvaluationCount(rig) == pulls);
    }
    RigExecImagingBridge reference(stage, rig);
    CHECK(reference.Compile());
    CHECK(reference.EvaluateAndPublishResult(firstFill).ok);
    CHECK(_SameGeometry(_CaptureGeometry(registry.GetStore()->Get()),
                        _CaptureGeometry(reference.GetStore()->Get())));
    registry.SetWarmSamplingBudget(savedInvocations, savedMs);
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
        // The dependency set is recorded: the shared digest order or path
        // text, the text sorted and unique.
        CHECK(proof.order || !proof.paths.empty());
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

// PROOFS BY ORDER. A proof holds its sampled vector's recorded digest order
// instead of every sample path's text: memoized frames of one path sequence
// share one order, and retiring by control id retires exactly the proofs
// the text-set rule retires -- a canonical path id by path, anything else
// (override ids, non-paths, other spellings of a path) by text only.
void
TestProofsShareTheDigestOrder()
{
    std::printf("progress: TestProofsShareTheDigestOrder\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    const UsdStageRefPtr stage = MakeTinyRig();
    RigExecImagingBridge bridge(stage, SdfPath("/Asset/Rig"));
    CHECK(bridge.Compile());
    const std::vector<RigExecValueOverride> none;

    // Warming proofs recorded beside the memoized frames: an ordered vector
    // with a repeated, an empty and a variant-selection path; an ordered
    // vector under a standing override; and a vector with no order, which
    // keeps the path text.
    const SdfPath variant = SdfPath("/V")
                                .AppendVariantSelection("v", "a")
                                .AppendProperty(TfToken("x"));
    RigExecFrameInputs repeated;
    repeated.Add(SdfPath("/P/A.x"), VtValue(1.0));
    repeated.Add(SdfPath("/P/B.y"), VtValue(2.0));
    repeated.Add(SdfPath("/P/A.x"), VtValue(3.0));
    repeated.Add(SdfPath(), VtValue(4.0));
    repeated.Add(variant, VtValue(5.0));
    repeated.digestOrder = RigExecRecordFrameDigestOrder(repeated.values);
    RigExecFrameInputs overridden;
    overridden.Add(SdfPath("/P/A.x"), VtValue(6.0));
    overridden.Add(SdfPath("/R.w"), VtValue(7.0));
    overridden.digestOrder = RigExecRecordFrameDigestOrder(overridden.values);
    RigExecFrameInputs unordered;
    unordered.Add(SdfPath("/P/B.y"), VtValue(8.0));
    unordered.Add(SdfPath("/Q.z"), VtValue(9.0));
    const std::vector<RigExecValueOverride> standing{RigExecValueOverride{
        SdfPath("/Asset/Rig/AlongX"), TfToken(), TfToken("avars:tx"),
        VtValue(3.0)}};
    const std::vector<double> memoized{1.0, 2.0, 3.0};
    const std::vector<double> warmed{11.0, 12.0, 13.0};

    // Drops every proof, then records the memoized and the warming ones.
    const auto record = [&]() {
        bridge.ClearFrameCache();
        for (double time : memoized) {
            const auto result =
                bridge.EvaluateAndPublishResult(UsdTimeCode(time));
            CHECK(result.ok && !result.cacheHit);
        }
        bridge.NoteWarmingEnqueued(UsdTimeCode(11.0), 11, 11, repeated);
        bridge.NoteWarmingEnqueued(UsdTimeCode(13.0), 13, 13, unordered);
        bridge.SetInteractiveOverrides(standing);
        bridge.NoteWarmingEnqueued(UsdTimeCode(12.0), 12, 12, overridden);
        bridge.ClearInteractiveOverrides();
        CHECK(bridge.GetFreshProofCount() ==
              memoized.size() + warmed.size());
    };

    // Today's rule per proof, from the vector and overrides it was recorded
    // from: every non-empty sample path's text plus the override ids. The
    // memoized vectors are re-sampled here and pinned to their proofs by
    // the unfolded digest.
    std::map<double, std::set<std::string>> reference;
    const auto addReference =
        [&reference](double time, const RigExecFrameInputs &inputs,
                     const std::vector<RigExecValueOverride> &overrides) {
            std::set<std::string> &ids = reference[time];
            for (const RigExecSampledInput &sample : inputs.values) {
                if (!sample.path.IsEmpty()) {
                    ids.insert(sample.path.GetString());
                }
            }
            for (const RigExecValueOverride &o : overrides) {
                ids.insert(RigExecControlIdForOverride(o));
            }
        };
    record();
    std::map<double, RigExecFreshProof> proofs;
    for (double time : memoized) {
        CHECK(bridge.GetFreshProof(UsdTimeCode(time), &proofs[time]));
        RigExecFrameInputs sampled;
        CHECK(RigExecSampleFrameInputs(bridge.GetEvaluator(),
                                       UsdTimeCode(time), none, &sampled));
        CHECK(!sampled.values.empty());
        CHECK(RigExecControlStateDigest(sampled, none) ==
              proofs[time].unfolded);
        addReference(time, sampled, none);
    }
    // The varying edit test's control: every memoized proof names it.
    CHECK(reference[1.0].count("/Asset/Rig/AlongX.avars:tx") == 1);
    addReference(11.0, repeated, none);
    addReference(12.0, overridden, standing);
    addReference(13.0, unordered, none);
    for (double time : warmed) {
        CHECK(bridge.GetFreshProof(UsdTimeCode(time), &proofs[time]));
    }

    // One order for every memoized frame, and no sample text beside it.
    CHECK(proofs[1.0].order != nullptr);
    CHECK(proofs[1.0].order == proofs[2.0].order);
    CHECK(proofs[2.0].order == proofs[3.0].order);
    for (double time : memoized) {
        CHECK(proofs[time].sampledInputs);
        CHECK(proofs[time].paths.empty());
    }
    CHECK(proofs[11.0].order == repeated.digestOrder);
    CHECK(proofs[11.0].paths.empty());
    CHECK(proofs[12.0].order == overridden.digestOrder);
    CHECK(proofs[12.0].paths ==
          std::vector<std::string>{RigExecControlIdForOverride(standing[0])});
    CHECK(proofs[13.0].order == nullptr);
    CHECK(proofs[13.0].paths ==
          std::vector<std::string>(reference[13.0].begin(),
                                   reference[13.0].end()));

    // Every id any proof names, plus ids that name none: other spellings of
    // a named path (the second is a valid, non-canonical one), a relative
    // path, a prim path, a non-path and the empty id.
    std::set<std::string> candidates;
    for (const auto &entry : reference) {
        candidates.insert(entry.second.begin(), entry.second.end());
    }
    candidates.insert({"/P/../P/A.x", "/V{ v = a }.x", "P/A.x", "/P",
                       "not a path", ""});
    std::vector<double> times = memoized;
    times.insert(times.end(), warmed.begin(), warmed.end());
    const auto retireAndCompare =
        [&](const std::vector<std::string> &controls) {
            record();
            std::set<double> expected;
            for (const auto &[time, ids] : reference) {
                for (const std::string &id : controls) {
                    if (ids.count(id)) {
                        expected.insert(time);
                    }
                }
            }
            const size_t retired = bridge.RetireProofsForControls(controls);
            if (retired != expected.size()) {
                std::printf("retiring <%s>: %zu proofs, expected %zu\n",
                            controls.front().c_str(), retired,
                            expected.size());
                CHECK(false);
            }
            for (double time : times) {
                RigExecFreshProof held;
                if (bridge.GetFreshProof(UsdTimeCode(time), &held) ==
                    (expected.count(time) != 0)) {
                    std::printf("retiring <%s>: proof %g wrongly %s\n",
                                controls.front().c_str(), time,
                                expected.count(time) ? "kept" : "retired");
                    CHECK(false);
                }
            }
        };
    for (const std::string &id : candidates) {
        retireAndCompare({id});
    }
    // A sampled path, an override id and a non-path in one call.
    retireAndCompare({"/Q.z", RigExecControlIdForOverride(standing[0]),
                      "not a path"});
}

// ROUTED THROUGH A CONNECTION. A property-chain mover's input connected to a
// value on a prim the bake never recorded: the chain reads it through the
// resolved inputs, so the edit is routed (Edited) and names that value as a
// read path. A path the affected index does not know is a foreign control,
// so the completed frames the edit moved retire -- where a routing that
// reported nothing would have kept them cached with the pre-edit result.
void
TestConnectedChainSourceRetiresFrames(const std::string &examplesDir)
{
    std::printf("progress: TestConnectedChainSourceRetiresFrames\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() ==
        RigExecFrameCacheMode::Off) {
        return;
    }
    if (examplesDir.empty()) {
        std::printf("SKIP TestConnectedChainSourceRetiresFrames: "
                    "no examples dir\n");
        return;
    }
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/09_PropertyMathMovers.usda");
    CHECK(static_cast<bool>(stage));
    if (!stage) {
        return;
    }
    const SdfPath rig("/PropMathAsset/Rig");
    const UsdAttribute gain =
        stage->DefinePrim(SdfPath("/PropMathAsset/Rig/Settings"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("maxGain"), SdfValueTypeNames->Float);
    CHECK(gain.Set(1.0f));
    CHECK(stage->GetPrimAtPath(SdfPath("/PropMathAsset/Rig/Movers/ClampGain"))
              .GetAttribute(TfToken("inputs:max"))
              .SetConnections({gain.GetPath()}));
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rig, UsdTimeCode(1001.0), &errors));
    CHECK(registry.SetTime(UsdTimeCode(1002.0)));
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1001.0, 1002.0});
        CHECK(got.size() == 2);
        if (got.size() == 2) {
            CHECK(got[0] == RigExecWarmFrameState::Cached);
            CHECK(got[1] == RigExecWarmFrameState::Cached);
        }
    }
    CHECK(gain.Set(0.25f));
    RigExecImagingBridge *bridge = registry.GetBridge(rig);
    CHECK(bridge != nullptr);
    if (bridge) {
        CHECK(bridge->GetEvaluator().GetLastNoticeDisposition() ==
              RigExecNoticeDisposition::Edited);
        CHECK(bridge->GetEvaluator().GetLastNoticePatchedPaths() ==
              std::vector<SdfPath>{gain.GetPath()});
    }
    {
        const std::vector<RigExecWarmFrameState> got =
            registry.GetFrameStates(rig, {1001.0});
        CHECK(got.size() == 1);
        if (got.size() == 1 && got[0] != RigExecWarmFrameState::Dirty) {
            std::printf("frame 1001 stayed cached across a connected "
                        "source edit\n");
            CHECK(false);
        }
    }
    registry.Deactivate();
}

// Geometry-mover parameters are live inputs even when they start as schema
// defaults. Editing them must retire warmed frames before a cache read, and
// idle warming must publish the revised result without moving the playhead.
void
TestWrinkleParameterEditsRetireAndRewarm()
{
    std::printf("progress: TestWrinkleParameterEditsRetireAndRewarm\n");
    std::fflush(stdout);
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (RigExecFrameCacheModeFromEnvironment() == RigExecFrameCacheMode::Off) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    const SdfPath meshPath("/Asset/Geom/Mesh");
    const SdfPath moverPath("/Asset/Rig/Movers/Wrinkle");
    const std::vector<double> range{1.0, 2.0, 3.0};
    auto &registry = RigExecImagingRegistry::GetInstance();
    const auto warmRange = [&]() {
        const auto playhead = registry.GetStore()->Get();
        for (int tick = 0; tick != 8; ++tick) {
            const auto states = registry.GetFrameStates(rig, range);
            if (states.size() == range.size() &&
                std::all_of(states.begin(), states.end(), [](const auto state) {
                    return state == RigExecWarmFrameState::Cached;
                })) {
                break;
            }
            registry.OnIdle();
            registry.WaitUntilBackgroundIdle();
            CHECK(registry.GetStore()->Get() == playhead);
        }
        for (const auto state : registry.GetFrameStates(rig, range)) {
            CHECK(state == RigExecWarmFrameState::Cached);
        }
    };
    VtVec3fArray rest;
    VtIntArray counts, indices;
    constexpr int nx = 13, ny = 7;
    for (int y = 0; y != ny; ++y) {
        for (int x = 0; x != nx; ++x) {
            rest.push_back(GfVec3f(0.125f * x, 0.2f * y, 0.0f));
        }
    }
    for (int y = 0; y + 1 < ny; ++y) {
        for (int x = 0; x + 1 < nx; ++x) {
            const int a = y * nx + x;
            counts.push_back(4);
            for (int index : {a, a + 1, a + 1 + nx, a + nx}) {
                indices.push_back(index);
            }
        }
    }
    struct Parameter {
        const char *name;
        VtValue initial;
        VtValue revised;
        bool varying = true;
        bool mustChange = false;
    };
    VtVec3fArray revisedRest = rest;
    for (GfVec3f &point : revisedRest) {
        point[0] *= 0.9f;
    }
    const Parameter parameters[] = {
        {"inputs:restPoints", VtValue(rest), VtValue(revisedRest), false},
        {"inputs:iterations", VtValue(80), VtValue(0), true, true},
        {"inputs:topology", VtValue(TfToken("cloth")),
            VtValue(TfToken("surfaceStruts")), false},
        {"inputs:neighborDistance", VtValue(2), VtValue(3)},
        {"inputs:restLengthScale", VtValue(1.0f), VtValue(0.9f)},
        {"inputs:stretchStiffness", VtValue(1.0f), VtValue(0.5f)},
        {"inputs:compressionStiffness", VtValue(1.0f), VtValue(0.5f)},
        {"inputs:bendStiffness", VtValue(0.1f), VtValue(0.05f)},
        {"inputs:maxDisplacement", VtValue(0.2f), VtValue(0.0f), true, true},
        {"inputs:pinBorders", VtValue(true), VtValue(false)},
        {"inputs:pinPoints", VtValue(VtIntArray()), VtValue(VtIntArray{45}), false},
        {"inputs:tangentPlaneCollisions", VtValue(true), VtValue(false)},
        {"inputs:tangentPlaneInset", VtValue(0.0f), VtValue(0.03f)},
        {"inputs:wrinkleScale", VtValue(1.0f), VtValue(0.0f), true, true},
        {"inputs:smoothingIterations", VtValue(0), VtValue(2)},
        {"inputs:enabled", VtValue(true), VtValue(false), true, true},
        {"inputs:defaultWeight", VtValue(1.0f), VtValue(0.0f), true, true}};
    { // All authored parameter cases use the same compiled graph.
        for (const Parameter &parameter : parameters) {
            for (int authored = 0; authored != 5; ++authored) {
                const bool late = authored >= 3;
                if (late && std::string(parameter.name) != "inputs:wrinkleScale") {
                    continue;
                }
                if (authored == 2 && !parameter.varying) {
                    continue;
                }
                std::printf("  %s, %s, %s\n", parameter.name,
                            "graph",
                            authored == 0 ? "schema fallback" :
                            authored == 1 ? "default" :
                            authored == 2 ? "time samples" :
                            authored == 3 ? "late mover default" : "late mover spline");
                std::fflush(stdout);
                const UsdStageRefPtr stage = late ? MakeTinyRig() : UsdStage::CreateInMemory();
                stage->SetStartTimeCode(1.0);
                stage->SetEndTimeCode(3.0);
                stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
                const UsdPrim root = stage->DefinePrim(rig, TfToken("RigExecRoot"));
                const UsdPrim mesh = stage->DefinePrim(meshPath, TfToken("Mesh"));
                CHECK(mesh.GetAttribute(TfToken("faceVertexCounts")).Set(counts));
                CHECK(mesh.GetAttribute(TfToken("faceVertexIndices")).Set(indices));
                for (double frame : range) {
                    VtVec3fArray points = rest;
                    for (GfVec3f &point : points) {
                        point[0] *= 0.6f + 0.05f * float(frame);
                    }
                    CHECK(mesh.GetAttribute(TfToken("points")).Set(
                        points, UsdTimeCode(frame)));
                }
                std::vector<std::string> errors;
                if (late) {
                    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
                    CHECK(registry.SetTime(UsdTimeCode(2.0)));
                    CHECK(registry.SetTime(UsdTimeCode(3.0)));
                    for (const auto state : registry.GetFrameStates(rig, range)) {
                        CHECK(state == RigExecWarmFrameState::Cached);
                    }
                }
                const UsdPrim mover = stage->DefinePrim(
                    moverPath, TfToken("RigExecWrinkleMover"));
                CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
                CHECK(mover.GetRelationship(TfToken("rigExec:moves")).SetTargets(
                    {meshPath.AppendProperty(TfToken("points"))}));
                if (std::string(parameter.name) != "inputs:restPoints") {
                    CHECK(mover.GetAttribute(TfToken("inputs:restPoints")).Set(rest));
                }
                if (std::string(parameter.name) == "inputs:neighborDistance") {
                    CHECK(mover.GetAttribute(TfToken("inputs:topology")).Set(
                        TfToken("surfaceStruts")));
                }
                // Mutable for SetSpline with USD 26.05 headers.
                UsdAttribute attribute = mover.GetAttribute(TfToken(parameter.name));
                CHECK(attribute);
                if (authored == 1 || late) {
                    CHECK(attribute.Set(parameter.initial));
                } else if (authored == 2) {
                    CHECK(attribute.Set(parameter.initial, UsdTimeCode(1.0)));
                    CHECK(attribute.Set(parameter.initial, UsdTimeCode(3.0)));
                } else {
                    CHECK(!attribute.HasAuthoredValueOpinion());
                }
                if (!late) {
                    CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
                } else {
                    CHECK(registry.SetTime(UsdTimeCode(1.0)));
                }
                CHECK(registry.SetWarmRange(rig, range));
                // Establish a standing frozen sampling burst before the edit:
                // live-only memoization does not exercise its retained inputs.
                if (RigExecBackgroundWarmingEnabled()) {
                    warmRange();
                }
                CHECK(registry.SetTime(UsdTimeCode(2.0)));
                const _GenerationGeometry before = _CaptureGeometry(registry.GetStore()->Get());
                CHECK(!before.points.empty());
                CHECK(registry.SetTime(UsdTimeCode(3.0)));
                for (const auto state : registry.GetFrameStates(rig, range)) {
                    CHECK(state == RigExecWarmFrameState::Cached);
                }
                // Direct USD authoring covers node-editor edits as well as
                // adding the first opinion to a schema-provided input.
                if (authored == 4) {
                    TsSpline spline(attribute.GetTypeName().GetType());
                    TsKnot knot(attribute.GetTypeName().GetType());
                    knot.SetTime(2.0);
                    knot.SetValue(0.0f);
                    knot.SetNextInterpolation(TsInterpCurve);
                    spline.SetKnot(knot);
                    CHECK(attribute.SetSpline(spline));
                } else {
                    CHECK(attribute.Set(parameter.revised, authored == 2
                        ? UsdTimeCode(2.0) : UsdTimeCode::Default()));
                }
                const auto states = registry.GetFrameStates(rig, {1.0, 2.0});
                CHECK(states.size() == 2);
                for (const auto state : states) {
                    // First opinions on uniform inputs may rebuild the
                    // epoch and clear its entries instead of retaining rows.
                    CHECK(state == RigExecWarmFrameState::Dirty ||
                          (authored == 0 && !parameter.varying &&
                           state == RigExecWarmFrameState::Uncached));
                }
                if (RigExecBackgroundWarmingEnabled()) {
                    warmRange();
                }
                const size_t pulls = registry.GetSessionEvaluationCount(rig);
                CHECK(registry.SetTime(UsdTimeCode(2.0)));
                if (RigExecBackgroundWarmingEnabled() &&
                    !RigExecFrameCacheVerifyRequested()) {
                    CHECK(registry.GetSessionEvaluationCount(rig) == pulls);
                }
                const _GenerationGeometry after = _CaptureGeometry(registry.GetStore()->Get());
                if (parameter.mustChange) {
                    CHECK(!_SameGeometry(before, after));
                }
                RigExecImagingBridge reference(stage, rig);
                CHECK(reference.Compile());
                const auto expected = reference.EvaluateAndPublishResult(UsdTimeCode(2.0));
                CHECK(expected.ok && !expected.cacheHit);
                CHECK(_SameGeometry(after, _CaptureGeometry(reference.GetStore()->Get())));
                registry.Deactivate();
            }
        }
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
          RigExecNoticeDisposition::Edited);
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

// MakeTinyRig over frames 1-3 with every control static (AlongX's tx held
// at a default too): no sample in `values` moves between frames.
UsdStageRefPtr
MakeStaticTinyRig()
{
    UsdStageRefPtr stage = MakeTinyRig(/*staticY=*/true);
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    UsdAttribute tx =
        stage->GetAttributeAtPath(SdfPath("/Asset/Rig/AlongX.avars:tx"));
    tx.Clear();
    tx.Set(10.0);
    return stage;
}

// Every point's weights on AlongX and AlongY.
VtFloatArray
TinyWeights(float alongX, float alongY)
{
    VtFloatArray out(kTinyPointCount * 2);
    for (size_t i = 0; i < kTinyPointCount; ++i) {
        out[i * 2] = alongX;
        out[i * 2 + 1] = alongY;
    }
    return out;
}

// rigExec:jointWeights time samples at 1, 2 and 3 that put the points of
// each frame somewhere else.
void
AuthorWeightSamples(const UsdStageRefPtr &stage)
{
    const UsdAttribute layout = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Movers/Skin_0.rigExec:jointWeights"));
    layout.Set(TinyWeights(1.0f, 0.0f), UsdTimeCode(1.0));
    layout.Set(TinyWeights(0.0f, 1.0f), UsdTimeCode(2.0));
    layout.Set(TinyWeights(0.5f, 0.5f), UsdTimeCode(3.0));
}

// MakeStaticTinyRig with rigExec:jointWeights time-sampled (and a default,
// at which compile validates the layout): the layout is not epoch state,
// and the weights are all that moves.
UsdStageRefPtr
MakeAnimatedWeightsRig()
{
    UsdStageRefPtr stage = MakeStaticTinyRig();
    stage->GetAttributeAtPath(
             SdfPath("/Asset/Rig/Movers/Skin_0.rigExec:jointWeights"))
        .Set(TinyWeights(1.0f, 0.0f));
    AuthorWeightSamples(stage);
    return stage;
}

// D1 for a time-varying skin layout. With every control static, no sample
// in `values` moves between frames; the weights do, so the control digest
// must fold them or every frame shares one key and a revisit serves the
// points the last frame published under it. Visiting 1, 2, 1, 3, 2, each
// visit equals the cache-off session's, and the two revisits hit. The
// plain and burst digests key the three frames apart and agree.
void
TestTimeSampledSkinWeightsKeyFramesApart()
{
    std::printf("progress: TestTimeSampledSkinWeightsKeyFramesApart\n");
    std::fflush(stdout);
    const SdfPath rig("/Asset/Rig");
    const std::vector<double> visits = {1.0, 2.0, 1.0, 3.0, 2.0};
    const auto drive = [&](std::vector<_GenerationGeometry> *out,
                           size_t *pulls, RigExecFrameCacheStats *stats) {
        UsdStageRefPtr stage = MakeAnimatedWeightsRig();
        RigExecImagingRegistry &registry =
            RigExecImagingRegistry::GetInstance();
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, rig, UsdTimeCode(visits[0]), &errors));
        out->push_back(_CaptureGeometry(registry.GetStore()->Get()));
        for (size_t i = 1; i < visits.size(); ++i) {
            CHECK(registry.SetTime(UsdTimeCode(visits[i])));
            out->push_back(_CaptureGeometry(registry.GetStore()->Get()));
        }
        *pulls = registry.GetSessionEvaluationCount(rig);
        *stats = registry.GetFrameCacheStats(rig);
        registry.Deactivate();
    };

    SetEnv("RIGEXEC_FRAME_CACHE", "off");
    std::vector<_GenerationGeometry> reference;
    size_t pulls = 0;
    RigExecFrameCacheStats stats;
    drive(&reference, &pulls, &stats);
    CHECK(reference.size() == visits.size());
    if (reference.size() != visits.size()) {
        return;
    }
    // The weights move the points: frames 1, 2 and 3 all differ.
    CHECK(!reference[0].points.empty());
    CHECK(!_SameGeometry(reference[0], reference[1]));
    CHECK(!_SameGeometry(reference[0], reference[3]));
    CHECK(!_SameGeometry(reference[1], reference[3]));

    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    std::vector<_GenerationGeometry> cached;
    drive(&cached, &pulls, &stats);
    CHECK(cached.size() == visits.size());
    for (size_t i = 0; i < visits.size() && i < cached.size(); ++i) {
        if (!_SameGeometry(reference[i], cached[i])) {
            std::printf("visit %zu: cache-on served another frame's points "
                        "at %g\n", i, visits[i]);
            CHECK(false);
        }
    }
    if (RigExecFrameCacheModeFromEnvironment() !=
            RigExecFrameCacheMode::Off &&
        !RigExecFrameCacheVerifyRequested()) {
        // Three distinct frames evaluate; both revisits are served.
        CHECK(pulls == 3);
        CHECK(stats.hits == 2);
    }

    // The keys themselves, on the plain and the burst routes.
    UsdStageRefPtr stage = MakeAnimatedWeightsRig();
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.GetBakedProgram() != nullptr);
    if (!evaluator.GetBakedProgram()) {
        return;
    }
    const std::vector<RigExecValueOverride> none;
    RigExecChainSampleBindings pinned;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned, &error));
    RigExecBurstSampleCache burst;
    CHECK(RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, none,
        RigExecFrameCacheEpochDigest(evaluator), &burst, &error));
    std::vector<uint64_t> digests;
    for (double frame : {1.0, 2.0, 3.0}) {
        RigExecFrameInputs plain, burstInputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame), none,
                                       &plain, &error));
        CHECK(RigExecSampleFrameInputsWithBurstCache(
            evaluator, UsdTimeCode(frame), none, &burst, &burstInputs,
            &error));
        CHECK(RigExecControlStateDigestible(plain, none));
        const uint64_t digest = RigExecControlStateDigest(plain, none);
        CHECK(RigExecControlStateDigestWithBurstCache(burstInputs, none,
                                                      &burst) == digest);
        digests.push_back(digest);
        // The one skin's row is listed and is key material.
        CHECK(plain.varyingLayoutRows == std::vector<uint32_t>{0});
        CHECK(plain.varyingRevisionLeaves.empty());
        CHECK(burstInputs.varyingLayoutRows == plain.varyingLayoutRows);
    }
    CHECK(digests.size() == 3 && digests[0] != digests[1] &&
          digests[0] != digests[2] && digests[1] != digests[2]);

    // A fixed layout lists nothing and keys exactly as before: its row is
    // not key material (it reads one value at every time).
    const auto perturbedLayoutDigest = [&](RigExecFrameInputs inputs) {
        for (std::vector<VtValue> &row : inputs.layoutLeaves) {
            for (VtValue &value : row) {
                if (value.IsHolding<VtFloatArray>()) {
                    VtFloatArray weights = value.UncheckedGet<VtFloatArray>();
                    if (!weights.empty()) {
                        weights[0] += 0.25f;
                    }
                    value = VtValue(weights);
                }
            }
        }
        return RigExecControlStateDigest(inputs, none);
    };
    RigExecFrameInputs animatedInputs;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0), none,
                                   &animatedInputs, &error));
    CHECK(perturbedLayoutDigest(animatedInputs) !=
          RigExecControlStateDigest(animatedInputs, none));
    UsdStageRefPtr fixedStage = MakeTinyRig(/*staticY=*/true);
    RigExecRigEvaluator fixed(fixedStage, rig);
    CHECK(fixed.Compile(&errors));
    CHECK(fixed.Evaluate(UsdTimeCode(1.0)).valid);
    RigExecFrameInputs fixedInputs;
    CHECK(RigExecSampleFrameInputs(fixed, UsdTimeCode(2.0), none,
                                   &fixedInputs, &error));
    CHECK(fixedInputs.layoutLeaves.size() == 1 &&
          !fixedInputs.layoutLeaves[0].empty());
    CHECK(fixedInputs.varyingLayoutRows.empty());
    CHECK(fixedInputs.varyingRevisionLeaves.empty());
    CHECK(perturbedLayoutDigest(fixedInputs) ==
          RigExecControlStateDigest(fixedInputs, none));

    // A notice the program cannot route (prim metadata on the skin) moves
    // the program stamp; fixedness, asked again before the next run, still
    // holds, so the row stays unlisted and the key stands.
    fixedStage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Skin_0"))
        .SetDocumentation("weights painted once");
    CHECK(fixed.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::StampBumped);
    RigExecFrameInputs bumpedInputs;
    CHECK(RigExecSampleFrameInputs(fixed, UsdTimeCode(2.0), none,
                                   &bumpedInputs, &error));
    CHECK(bumpedInputs.varyingLayoutRows.empty());
    CHECK(RigExecControlStateDigest(bumpedInputs, none) ==
          RigExecControlStateDigest(fixedInputs, none));
}

// D1 across an in-session edit that makes a fixed layout time-varying.
// Under static controls frames 1-3 share one key, and the program asks
// layout fixedness again only on its next run: every lookup or warm job
// sampled before that run must already key the layout as varying, or the
// notice path's playhead re-evaluation hits the pre-edit entry and so does
// every later visit. Routed (the samples alone: Edited, which marks the
// layout's reads) and stamp-bumped (the same samples and prim metadata in
// one change block: StampBumped, which marks nothing), every capture after
// the edit equals the cache-off session's.
void
TestInSessionWeightSamplesKeyFramesApart()
{
    std::printf("progress: TestInSessionWeightSamplesKeyFramesApart\n");
    std::fflush(stdout);
    const SdfPath rig("/Asset/Rig");
    const std::vector<double> visits = {1.0, 2.0, 3.0, 1.0, 2.0};
    for (const bool bumped : {false, true}) {
        const auto drive = [&](std::vector<_GenerationGeometry> *out) {
            UsdStageRefPtr stage = MakeStaticTinyRig();
            RigExecImagingRegistry &registry =
                RigExecImagingRegistry::GetInstance();
            std::vector<std::string> errors;
            CHECK(registry.Activate(stage, rig, UsdTimeCode(1.0), &errors));
            CHECK(registry.SetTime(UsdTimeCode(2.0)));
            CHECK(registry.SetTime(UsdTimeCode(3.0)));
            {
                SdfChangeBlock changes;
                AuthorWeightSamples(stage);
                if (bumped) {
                    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Skin_0"))
                        .SetDocumentation("weights painted per frame");
                }
            }
            RigExecImagingBridge *bridge = registry.GetBridge(rig);
            CHECK(bridge != nullptr);
            if (bridge) {
                CHECK(bridge->GetEvaluator().GetLastNoticeDisposition() ==
                      (bumped ? RigExecNoticeDisposition::StampBumped
                              : RigExecNoticeDisposition::Edited));
            }
            // The playhead the notice path re-evaluated, then each visit.
            out->push_back(_CaptureGeometry(registry.GetStore()->Get()));
            for (double time : visits) {
                CHECK(registry.SetTime(UsdTimeCode(time)));
                out->push_back(_CaptureGeometry(registry.GetStore()->Get()));
            }
            registry.Deactivate();
        };

        SetEnv("RIGEXEC_FRAME_CACHE", "off");
        std::vector<_GenerationGeometry> reference;
        drive(&reference);
        CHECK(reference.size() == visits.size() + 1);
        if (reference.size() != visits.size() + 1) {
            return;
        }
        // The samples move the points: the playhead (3), 1 and 2 differ.
        CHECK(!reference[0].points.empty());
        CHECK(!_SameGeometry(reference[0], reference[1]));
        CHECK(!_SameGeometry(reference[0], reference[2]));
        CHECK(!_SameGeometry(reference[1], reference[2]));

        SetEnv("RIGEXEC_FRAME_CACHE", "on");
        SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
        std::vector<_GenerationGeometry> cached;
        drive(&cached);
        CHECK(cached.size() == reference.size());
        size_t stale = 0;
        for (size_t i = 0; i < reference.size() && i < cached.size(); ++i) {
            if (!_SameGeometry(reference[i], cached[i])) {
                ++stale;
            }
        }
        if (stale != 0) {
            std::printf("%s edit: %zu of %zu captures served another "
                        "frame's points\n", bumped ? "stamp-bumped" : "routed",
                        stale, reference.size());
            CHECK(false);
        }
    }
}

}  // namespace


// The live sampler's chain bindings are bound once per binding epoch and kept
// current by stage notices rather than re-verified on every sample (which on
// the full biped stack cost 60 ms per call, twice per viewport release). The
// contract that replaces the check: a chain mover's constant edited
// mid-epoch -- which moves no epoch digest -- rebinds them; an edit to a
// control that no chain reads does not; and the trusted sample always
// digests exactly as the self-binding sampler's. Since the frozen worker runs
// the chains itself, the bindings carry no value into a sample: bindings
// taken BEFORE the edit digest the same once it has landed.
struct _ChainNoticeForward : public TfWeakBase {
    RigExecImagingBridge *bridge = nullptr;
    void OnChanged(const UsdNotice::ObjectsChanged &notice,
                   const UsdStageWeakPtr &)
    {
        bridge->NoteChainEdits(notice);
    }
};

void
TestLiveChainBindingsFollowNotices()
{
    UsdStageRefPtr stage = MakeTinyRig();
    const UsdPrim gain = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/TxGain"), TfToken("RigExecFloatMathMover"));
    gain.ApplyAPI(TfToken("RigExecMoverAPI"));
    gain.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Rig/AlongX.avars:tx")});
    gain.CreateAttribute(TfToken("rigExec:operation"), SdfValueTypeNames->Token)
        .Set(TfToken("multiply"));
    gain.CreateAttribute(TfToken("inputs:defaultWeight"),
                         SdfValueTypeNames->Float).Set(1.0f);
    UsdAttribute value = gain.CreateAttribute(TfToken("inputs:value"),
                                              SdfValueTypeNames->Float);
    for (int t = 1; t <= 4; ++t) {
        value.Set(0.5f * float(t), UsdTimeCode(double(t)));
    }

    RigExecImagingBridge bridge(stage, SdfPath("/Asset/Rig"));
    CHECK(bridge.Compile());
    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(2.0)).ok);
    _ChainNoticeForward forward;
    forward.bridge = &bridge;
    TfNotice::Key key = TfNotice::Register(
        TfCreateWeakPtr(&forward), &_ChainNoticeForward::OnChanged, stage);

    const RigExecRigEvaluator &evaluator = bridge.GetEvaluator();
    const std::vector<RigExecValueOverride> none;
    const auto digestsAgree = [&](const RigExecChainSampleBindings &bound) {
        RigExecFrameInputs trusted, plain;
        if (!RigExecSampleFrameInputsWithTrustedChainBindings(
                evaluator, UsdTimeCode(3.0), none, bound, &trusted) ||
            !RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), none,
                                      &plain)) {
            return false;
        }
        return RigExecControlStateDigest(trusted, none) ==
               RigExecControlStateDigest(plain, none);
    };

    uint64_t first = 0;
    const RigExecChainSampleBindings *bound =
        bridge.AcquireChainBindings(&first);
    CHECK(bound != nullptr && bound->chains.size() == 1);
    CHECK(bound && digestsAgree(*bound));
    const RigExecChainSampleBindings before = *bound;

    // An avar no chain reads: the bindings stand. Authoring it for the first
    // time creates its spec, which USD reports as a PROPERTY resync; that
    // must not drop them either.
    uint64_t now = 0;
    stage->GetAttributeAtPath(SdfPath("/Asset/Rig/AlongY.avars:tz"))
        .Set(0.25);
    bridge.AcquireChainBindings(&now);
    CHECK(now == first);

    // The chain mover's folded constant, edited mid-epoch: same binding
    // epoch, new bindings, and the trusted sample is still exact.
    const size_t epoch = evaluator.GetBindingEpochDigest();
    stage->GetAttributeAtPath(
            SdfPath("/Asset/Rig/Movers/TxGain.inputs:defaultWeight"))
        .Set(0.5f);
    CHECK(evaluator.GetBindingEpochDigest() == epoch);
    bound = bridge.AcquireChainBindings(&now);
    CHECK(bound != nullptr && now != first);
    CHECK(bound && digestsAgree(*bound));
    // The pre-edit bindings sample the same vector: they are the epoch's
    // currency check only, and the sampled values (head leaves included)
    // are read off the stage at the job's time, never through them.
    CHECK(digestsAgree(before));

    TfNotice::Revoke(key);
}

int
main(int argc, char **argv)
{
    const std::string examplesDir = argc > 1 ? argv[1] : "";
    // A legacy custom attribute cannot choose a different evaluator.
    {
        auto stage=MakeTinyRig(); const SdfPath rig("/Asset/Rig");
        RigExecImagingBridge bridge(stage,rig); CHECK(bridge.Compile());
        CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(1)).ok);
        CHECK(bridge.GetEvaluator().GetBakedProgram()!=nullptr);
        const auto legacy=stage->GetPrimAtPath(rig).CreateAttribute(
            TfToken("rigExec:baked"),SdfValueTypeNames->Bool,true);
        CHECK(legacy.Set(false));
        CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(2)).ok);
        CHECK(bridge.GetEvaluator().GetBakedProgram()!=nullptr);
        CHECK(legacy.Set(true));
        CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(3)).ok);
        CHECK(bridge.GetEvaluator().GetBakedProgram()!=nullptr);
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
    TestARestEditIsNotServedStale();
    TestAnAnimatedRestRigServesWarmedHits();
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
    TestConnectedSpaceRigWarmsRange();
    TestAffineProvidersAndExternalMoversWarmFrozen();
    TestWarmFrameStreaks();
    TestWarmRangeProgress();
    TestCommitDuringPreviewExcludesPlayhead();
    TestCommitOfPreviewedPoseSkipsEvaluation();
    TestFirstSessionCommitOfPreviewedPose();
    TestCommitOfAChainTargetSettles();
    TestProductionTriggerPathEnqueuesAndFences();
    TestWarmedCompletionServesWithoutEvaluating();
    TestConnectedSpaceRigMemoizesResults();
    TestPropertyChainRigMemoizesCompletePoses();
    TestSupportedSourcesWarmCompletePoses();
    TestInteractiveDragBypassesCache();
    TestProfilerFrameCacheLane();
    TestCachedServeSkipsLookupSample();
    TestStackFullRangeCursorWarmsEveryFrame(examplesDir);
    TestEvaluatorBranchesDriveRetirement();
    TestConnectedChainSourceRetiresFrames(examplesDir);
    TestWrinkleParameterEditsRetireAndRewarm();
    TestCarryOverRekeysCleanEntries();
    TestScopedCancelDropsOldJobsOnTokenMismatch();
    TestRunningFirstFillEditDropsOldResult();
    TestEditOneControlRetiresAndRewarms();
    TestProofScopingRetiresOnlyIntersecting();
    TestDragReleaseRewarmsAffected();
    TestClearWhileWarmingKeepsCacheEmpty();
    TestFencedClearSerializesAfterPausedInsert();
    TestOverlayMidWarmingServesNoStalePose();
    TestTimeSampledSkinWeightsKeyFramesApart();
    TestInSessionWeightSamplesKeyFramesApart();
    TestLiveChainBindingsFollowNotices();
    TestProofsShareTheDigestOrder();
    if (failures == 0) {
        std::printf("testRigExecImagingFrameCache: all tests passed\n");
    } else {
        std::printf("testRigExecImagingFrameCache: %d failures\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
