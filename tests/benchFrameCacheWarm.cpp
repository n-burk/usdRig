//
// benchFrameCacheWarm -- Stream F warming benches for the per-frame cache.
//
// Where benchFrameCache measures the numbers the defaults are made from,
// this bench measures what warming DOES with them: cold-vs-warm scrub
// throughput, edit-to-affected-frames recompute cost, UI-eval latency with
// warming on/off (median and p95), memory under the cap, an N-second
// UI-vs-warming stress for TSAN runs, and a Chrome-trace lane demo. Prints
// human-readable numbers; asserts nothing, so it is built but deliberately
// NOT registered with ctest (like benchFrameCache): wall-clock comparisons
// flake on shared CI runners, so the benches report and the gate stays
// deterministic.
//
//   benchFrameCacheWarm [examplesDir] [mode] [seconds] [stage] [frames]
//
//   mode: all (default), scrub, edit, latency, memory, stress, trace.
//   seconds: the stress duration (default 5).
//   stage: the scrub rig under examplesDir (default
//   "biped/Biped_anim.usda"); "biped/Biped_stack_anim.usda" is the heavy
//   lane. frames: the scrub's comma-separated frame list (default
//   "1,2,3,4,5,6,7,8").
//
// The primary rig is the frozen-context test's tiny in-memory rig (one
// skinned mesh over two animated controls), which bakes, evaluates, and --
// importantly -- samples, so every key below is a real control-state digest
// over real sampled inputs rather than a synthetic stand-in. When
// examplesDir names the checkout's examples/, the scrub bench also attempts
// the biped; the sampler may decline it (the chain-caveat on
// RigExecSampleFrameInputs), in which case the bench says so and reports
// the evaluator-only cold reference instead of a ratio it cannot stand
// behind.
//
// Environment, read and reported, never assumed: RIGEXEC_FRAME_CACHE (unset
// is set to "on" in-process for the run, so a bare bench measures warming;
// an explicit off/warm-off is honored and the fill modes say they skipped),
// RIGEXEC_ENABLE_PARALLEL_EVAL (the process default: workers always run the
// serial executor whatever it says), and the background-fill gate the two
// imply. The busy-work latency load is synthetic by label: a calibrated
// ~3.1 ms spin standing in for one biped serial frame (report section 2),
// so the on-vs-off p95 answers the contention question on any rig.
//

#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameCache.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/frozenContext.h"
#include "rigExec/parallel.h"
#include "rigExec/profiler.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

const char *
GetEnv(const char *name)
{
#ifdef _WIN32
    static char buffer[256];
    size_t needed = 0;
    if (getenv_s(&needed, buffer, sizeof(buffer), name) != 0 ||
        needed == 0) {
        return nullptr;
    }
    return buffer;
#else
    return getenv(name);
#endif
}

void
SetEnv(const char *name, const char *value)
{
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

double
NowUs()
{
    return double(std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count());
}

double
Percentile(std::vector<double> samples, double q)
{
    if (samples.empty()) {
        return 0.0;
    }
    std::sort(samples.begin(), samples.end());
    const size_t rank = size_t(std::ceil(q * double(samples.size())));
    return samples[std::min(rank, samples.size()) - 1];
}

double
Mean(const std::vector<double> &samples)
{
    if (samples.empty()) {
        return 0.0;
    }
    double total = 0.0;
    for (double v : samples) {
        total += v;
    }
    return total / double(samples.size());
}

void
PrintDistribution(const char *label, const std::vector<double> &samples,
                  const char *unit)
{
    std::printf("  %-22s %10.1f %s  (n=%zu, mean %.1f, max %.1f)\n", label,
                Percentile(samples, 0.5), unit, samples.size(),
                Mean(samples), Percentile(samples, 1.0));
    std::printf("  %-22s %10.1f %s\n", "p95", Percentile(samples, 0.95),
                unit);
}

// One skinned mesh over two animated controls, after the frozen-context
// test's rig: time samples at every frame in [t0, t1] so consecutive frames
// genuinely re-evaluate instead of cone-skipping a static rig.
UsdStageRefPtr
MakeTinyRig(size_t pointCount, int t0, int t1)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim alongX = stage->DefinePrim(SdfPath("/Asset/Rig/AlongX"),
                                             TfToken("RigExecControl"));
    const UsdPrim alongY = stage->DefinePrim(SdfPath("/Asset/Rig/AlongY"),
                                             TfToken("RigExecControl"));
    UsdAttribute tx = alongX.GetAttribute(TfToken("avars:tx"));
    UsdAttribute ty = alongY.GetAttribute(TfToken("avars:ty"));
    for (int t = t0; t <= t1; ++t) {
        tx.Set(10.0 + 0.1 * double(t), UsdTimeCode(double(t)));
        ty.Set(20.0 - 0.05 * double(t), UsdTimeCode(double(t)));
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    const SdfPath meshPath("/Asset/Geom/Mesh_0");
    const UsdPrim prim = stage->DefinePrim(meshPath, TfToken("Mesh"));
    VtVec3fArray points(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        points[i] = GfVec3f(float(i) * 0.5f, float(i) * -0.25f,
                            -float(i) * 0.125f);
    }
    prim.GetAttribute(TfToken("points")).Set(points);

    const UsdPrim skin = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Skin_0"),
                                           TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({meshPath.AppendProperty(TfToken("points"))});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({alongX.GetPath(), alongY.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int)
        .Set(2);
    VtIntArray indices(pointCount * 2);
    for (size_t i = 0; i < pointCount; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
    }
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(indices);
    VtFloatArray weights(pointCount * 2);
    for (size_t i = 0; i < pointCount; ++i) {
        weights[i * 2] = 0.1f;
        weights[i * 2 + 1] = 0.9f;
    }
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(weights);
    return stage;
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// One compiled, baked, sample-probed rig. Sampling is probed, not assumed:
// sampleable false carries the sampler's reason, and the benches that need
// keys skip with that reason instead of timing a fallback they mislabel.
struct BenchRig {
    UsdStageRefPtr stage;
    SdfPath rig;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    uint64_t epoch = 0;
    size_t clusters = 0;
    bool sampleable = false;
    std::string sampleError;
};

bool
PrepareRig(const char *name, const UsdStageRefPtr &stage, BenchRig *out)
{
    out->stage = stage;
    out->rig = FindRig(stage);
    if (out->rig.IsEmpty()) {
        std::printf("FATAL: %s has no RigExecRoot\n", name);
        return false;
    }
    out->evaluator.reset(new RigExecRigEvaluator(stage, out->rig));
    std::vector<std::string> errors;
    if (!out->evaluator->Compile(&errors)) {
        std::printf("FATAL: %s did not compile\n", name);
        for (const std::string &error : errors) {
            std::printf("  %s\n", error.c_str());
        }
        return false;
    }
    std::vector<std::string> reasons;
    if (!RigExecBakedProgram::IsBakeable(*out->evaluator, &reasons)) {
        std::printf("FATAL: %s does not bake\n", name);
        for (const std::string &reason : reasons) {
            std::printf("  %s\n", reason.c_str());
        }
        return false;
    }
    out->evaluator->SetEvaluationMode(RigExecEvaluationMode::Baked);
    const RigExecBakedProgram *program = out->evaluator->GetBakedProgram();
    if (!program) {
        std::printf("FATAL: %s built no program\n", name);
        return false;
    }
    out->epoch = uint64_t(out->evaluator->GetBindingEpochDigest());
    out->clusters = program->GetClusterCount();
    const std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameInputs probe;
    std::string error;
    out->sampleable = RigExecSampleFrameInputs(
        *out->evaluator, UsdTimeCode(1.0), noOverrides, &probe, &error);
    out->sampleError = error;
    std::printf("[%s] providers=%zu bound=%zu varying=%zu clusters=%zu "
                "sampleable=%s%s%s\n",
                name, program->GetProviderCount(),
                program->GetBoundInputCount(),
                program->GetVaryingInputCount(), out->clusters,
                out->sampleable ? "yes" : "NO",
                out->sampleable ? "" : " (",
                out->sampleable ? "" : (error + ")").c_str());
    return true;
}

RigExecFrameCacheKey
KeyFor(uint64_t epoch, const RigExecFrameInputs &inputs,
       const std::vector<RigExecValueOverride> &overrides)
{
    RigExecFrameCacheKey key;
    key.epochDigest = epoch;
    key.controlDigest = RigExecControlStateDigest(inputs, overrides);
    return key;
}

// ---------------------------------------------------------------------------
// Scrub throughput: cold (live eval + publish) vs warm (digest + lookup).
// ---------------------------------------------------------------------------

bool
BenchScrub(BenchRig *rig, const char *name,
           const std::vector<double> &frames)
{
    std::printf("[%s] scrub over %zu frames:\n", name, frames.size());
    if (!rig->sampleable) {
        // No keys, no ratio: report the evaluator-only cold reference the
        // sampler's decline left standing, and say the warm half is why.
        std::printf("  sampler declines %s (%s): no warm pass\n", name,
                    rig->sampleError.c_str());
        std::vector<double> evalOnly;
        for (size_t i = 0; i < 40; ++i) {
            const double t = frames[i % frames.size()];
            const double start = NowUs();
            const RigExecRigPose pose =
                rig->evaluator->Evaluate(UsdTimeCode(t));
            evalOnly.push_back(NowUs() - start);
            if (!pose.valid) {
                std::printf("  frame %g invalid; declining the rig\n", t);
                return false;
            }
        }
        PrintDistribution("cold eval-only", evalOnly, "us/frame");
        return true;
    }

    const std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameCache cache;

    // The evaluator alone, min-of-5 pass means, so the cold pass below can
    // be read as eval cost plus cache overhead rather than one number.
    std::vector<double> passMeans;
    for (size_t pass = 0; pass < 5; ++pass) {
        const double start = NowUs();
        for (size_t i = 0; i < 40; ++i) {
            const RigExecRigPose pose = rig->evaluator->Evaluate(
                UsdTimeCode(frames[i % frames.size()]));
            if (!pose.valid) {
                std::printf("  FATAL: frame invalid during eval reference\n");
                return false;
            }
        }
        passMeans.push_back((NowUs() - start) / 40.0);
    }
    std::printf("  %-22s %10.1f us/frame (min-of-5 pass mean)\n",
                "live eval reference",
                *std::min_element(passMeans.begin(), passMeans.end()));

    // Cold: every frame evaluates live and publishes, the first scrub over
    // a cold range.
    std::vector<double> cold;
    size_t published = 0;
    for (double t : frames) {
        const double start = NowUs();
        const RigExecRigPose pose =
            rig->evaluator->Evaluate(UsdTimeCode(t));
        RigExecFrameInputs inputs;
        std::string error;
        const bool sampled = RigExecSampleFrameInputs(
            *rig->evaluator, UsdTimeCode(t), noOverrides, &inputs, &error);
        if (sampled && pose.valid &&
            RigExecControlStateDigestible(inputs)) {
            published +=
                cache.Publish(KeyFor(rig->epoch, inputs, noOverrides),
                              UsdTimeCode(t), pose)
                    ? 1
                    : 0;
        }
        cold.push_back(NowUs() - start);
        if (!pose.valid || !sampled) {
            std::printf("  frame %g %s; declining the rig\n", t,
                        !pose.valid ? "invalid" : error.c_str());
            return false;
        }
    }
    PrintDistribution("cold eval+publish", cold, "us/frame");
    std::printf("  %-22s %10zu of %zu\n", "cold published", published,
                frames.size());

    // Warm: five reversed passes over the published range. Every lookup is
    // expected to hit; a miss is printed as data, not asserted.
    std::vector<double> warm;
    size_t misses = 0;
    size_t invalid = 0;
    for (size_t pass = 0; pass < 5; ++pass) {
        for (size_t i = frames.size(); i-- > 0;) {
            const double t = frames[i];
            const double start = NowUs();
            RigExecFrameInputs inputs;
            std::string error;
            RigExecFrameCacheKey key;
            bool ok = RigExecSampleFrameInputs(*rig->evaluator,
                                               UsdTimeCode(t), noOverrides,
                                               &inputs, &error);
            RigExecRigPose served;
            if (ok) {
                key = KeyFor(rig->epoch, inputs, noOverrides);
                ok = cache.Lookup(key, &served);
            }
            warm.push_back(NowUs() - start);
            misses += (!ok && error.empty()) ? 1 : 0;
            invalid += (ok && !served.valid) ? 1 : 0;
            if (!ok && !error.empty()) {
                std::printf("  warm sample failed at %g (%s)\n", t,
                            error.c_str());
                return false;
            }
        }
    }
    PrintDistribution("warm digest+lookup", warm, "us/frame");

    // Warm-path anatomy, 40 repetitions each: sampling alone, then
    // digest+lookup against an already-sampled vector, so the report can
    // say which half of the warm cost is the re-read and which is the
    // re-hash plus the served copy.
    {
        std::vector<double> sampleOnly;
        std::vector<double> digestLookup;
        for (size_t rep = 0; rep < 40; ++rep) {
            const double t = frames[rep % frames.size()];
            RigExecFrameInputs inputs;
            std::string error;
            const double sampleStart = NowUs();
            const bool sampled = RigExecSampleFrameInputs(
                *rig->evaluator, UsdTimeCode(t), noOverrides, &inputs,
                &error);
            sampleOnly.push_back(NowUs() - sampleStart);
            if (!sampled) {
                std::printf("  warm anatomy sample failed at %g\n", t);
                return false;
            }
            const double digestStart = NowUs();
            RigExecRigPose served;
            const bool hit =
                cache.Lookup(KeyFor(rig->epoch, inputs, noOverrides),
                             &served);
            digestLookup.push_back(NowUs() - digestStart);
            if (!hit || !served.valid) {
                std::printf("  warm anatomy lookup missed at %g\n", t);
                return false;
            }
        }
        std::printf("  %-22s %10.1f us/frame (median)\n", "warm of which: sample",
                    Percentile(sampleOnly, 0.5));
        std::printf("  %-22s %10.1f us/frame (median)\n",
                    "warm of which: digest+lookup",
                    Percentile(digestLookup, 0.5));
    }
    const double ratio =
        Percentile(warm, 0.5) > 0.0
            ? Percentile(cold, 0.5) / Percentile(warm, 0.5)
            : 0.0;
    std::printf("  %-22s %10.1fx (cold median / warm median)\n",
                "scrub speedup", ratio);
    std::printf("  (Stream F target: warmed scrub >=10x cold on the biped; "
                "this run reports actuals)\n");
    const RigExecFrameCacheStats stats = cache.Stats();
    std::printf("  hits=%zu misses=%zu published=%zu entries=%zu "
                "bytes=%zu (bench misses=%zu invalid=%zu)\n",
                stats.hits, stats.misses, stats.published,
                stats.entryCount, stats.bytes, misses, invalid);
    return true;
}

const char *
VerdictName(RigExecSparseVerdict verdict)
{
    switch (verdict) {
    case RigExecSparseVerdict::Hit:
        return "Hit";
    case RigExecSparseVerdict::Partial:
        return "Partial";
    case RigExecSparseVerdict::Miss:
        return "Miss";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Edit cost: one control-sample edit across an N-frame range, then what the
// sparse planner does with the affected frames.
// ---------------------------------------------------------------------------

bool
BenchEdit(BenchRig *rig, const std::vector<double> &frames)
{
    std::printf("[tiny] edit: one control sample across %zu frames:\n",
                frames.size());
    if (!rig->sampleable) {
        std::printf("  skipped: %s\n", rig->sampleError.c_str());
        return true;
    }
    UsdAttribute tx = rig->stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/AlongX.avars:tx"));
    if (!tx.IsValid()) {
        std::printf("  skipped: no authored tx to edit\n");
        return true;
    }
    const std::vector<RigExecValueOverride> noOverrides;

    // Warm: sample every frame's inputs and digests, publish every pose.
    RigExecFrameCache cache;
    std::vector<RigExecFrameInputs> warmed(frames.size());
    std::vector<uint64_t> digests(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        const RigExecRigPose pose =
            rig->evaluator->Evaluate(UsdTimeCode(frames[i]));
        std::string error;
        if (!pose.valid ||
            !RigExecSampleFrameInputs(*rig->evaluator,
                                      UsdTimeCode(frames[i]), noOverrides,
                                      &warmed[i], &error)) {
            std::printf("  FATAL: warm pass failed at %g (%s)\n",
                        frames[i], error.c_str());
            return false;
        }
        digests[i] = RigExecControlStateDigest(warmed[i], noOverrides);
        cache.Publish(KeyFor(rig->epoch, warmed[i], noOverrides),
                      UsdTimeCode(frames[i]), pose);
    }

    // The affected-frame computation itself: re-sample and re-digest the
    // range after the edit and count what moved. Timed as one burst,
    // reported total and per frame.
    const double editedFrame = frames[frames.size() / 2];
    tx.Set(999.0, UsdTimeCode(editedFrame));
    std::vector<RigExecFrameInputs> edited(frames.size());
    std::vector<uint64_t> editedDigests(frames.size());
    const double affectedStart = NowUs();
    for (size_t i = 0; i < frames.size(); ++i) {
        std::string error;
        if (!RigExecSampleFrameInputs(*rig->evaluator,
                                      UsdTimeCode(frames[i]), noOverrides,
                                      &edited[i], &error)) {
            std::printf("  FATAL: re-sample failed at %g (%s)\n",
                        frames[i], error.c_str());
            return false;
        }
        editedDigests[i] =
            RigExecControlStateDigest(edited[i], noOverrides);
    }
    const double affectedUs = NowUs() - affectedStart;
    size_t affected = 0;
    size_t affectedIndex = 0;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (editedDigests[i] != digests[i]) {
            affected++;
            affectedIndex = i;
        }
    }
    std::printf("  %-22s %10.1f us total, %.1f us/frame (%zu of %zu "
                "affected)\n",
                "affected-frame scan", affectedUs,
                affectedUs / double(frames.size()), affected,
                frames.size());

    // Sparse planning for the affected frame: the retained pre-edit state
    // against the edited request. Planning is sub-microsecond scale, so it
    // is timed over 200 repetitions and reported as a mean.
    const double indexStart = NowUs();
    RigExecOutputAffectedIndex index;
    index.Build(rig->evaluator->GetBakedProgram()->GetStepGraph(),
                rig->epoch);
    std::printf("  %-22s %10.1f us (one-time per epoch, %zu clusters)\n",
                "affected-index build", NowUs() - indexStart,
                index.ClusterCount());
    RigExecRetainedFrameState retained;
    retained.inputs = warmed[affectedIndex];
    retained.epochDigest = rig->epoch;
    retained.clusterCount = rig->clusters;
    RigExecTaskListCache memo;
    const size_t walksBefore = index.Walks();
    RigExecSparsePlan plan;
    const double planStart = NowUs();
    for (int rep = 0; rep < 200; ++rep) {
        plan = RigExecPlanSparseReuse(index, &memo, retained, rig->epoch,
                                      edited[affectedIndex], noOverrides);
    }
    const double planUs = (NowUs() - planStart) / 200.0;
    std::printf("  %-22s %10.1f us/plan (%s: run %zu of %zu clusters, "
                "memo %s, walks +%zu)\n",
                "sparse plan", planUs, VerdictName(plan.verdict),
                plan.ClustersToRun(), rig->clusters,
                plan.memoUsed ? "used" : "cold",
                index.Walks() - walksBefore);
    const RigExecTaskListStats memoStats = memo.Stats();
    std::printf("  memo stats: hits=%zu misses=%zu stores=%zu entries=%zu\n",
                memoStats.hits, memoStats.misses, memoStats.stores,
                memoStats.entries);

    // A second edit of the same control re-runs the memoized selection
    // without rewalking: the walk count must not move.
    tx.Set(1001.0, UsdTimeCode(editedFrame));
    RigExecFrameInputs editedAgain;
    std::string error;
    if (!RigExecSampleFrameInputs(*rig->evaluator,
                                  UsdTimeCode(editedFrame), noOverrides,
                                  &editedAgain, &error)) {
        std::printf("  FATAL: repeat sample failed (%s)\n", error.c_str());
        return false;
    }
    const size_t walksBeforeRepeat = index.Walks();
    const RigExecSparsePlan repeat =
        RigExecPlanSparseReuse(index, &memo, retained, rig->epoch,
                               editedAgain, noOverrides);
    std::printf("  %-22s %s (memo %s, walks +%zu)\n", "repeat-edit plan",
                VerdictName(repeat.verdict),
                repeat.memoUsed ? "used" : "cold",
                index.Walks() - walksBeforeRepeat);

    // An override drag touches every frame's digest: the affected scan
    // below is the all-frames-moved endpoint, and the plan is the
    // conservative one (an override names no provider seed, so the index
    // answers every cluster rather than risk skipping one that moved).
    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/AlongX");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.25);
    const std::vector<RigExecValueOverride> dragged = {drag};
    size_t dragAffected = 0;
    RigExecFrameInputs dragInputs;
    for (size_t i = 0; i < frames.size(); ++i) {
        RigExecFrameInputs at;
        if (!RigExecSampleFrameInputs(*rig->evaluator,
                                      UsdTimeCode(frames[i]), dragged, &at,
                                      &error)) {
            std::printf("  FATAL: drag sample failed at %g (%s)\n",
                        frames[i], error.c_str());
            return false;
        }
        if (RigExecControlStateDigest(at, dragged) != digests[i]) {
            dragAffected++;
        }
        if (i == affectedIndex) {
            dragInputs = at;
        }
    }
    const RigExecSparsePlan dragPlan =
        RigExecPlanSparseReuse(index, &memo, retained, rig->epoch,
                               dragInputs, dragged);
    std::printf("  override drag: %zu of %zu frames affected; plan %s, "
                "run %zu of %zu clusters (conservative: unmapped control)\n",
                dragAffected, frames.size(),
                VerdictName(dragPlan.verdict), dragPlan.ClustersToRun(),
                rig->clusters);
    return true;
}

// Calibrates a busy-spin to \p targetUs: times a probe loop, derives
// iterations per microsecond, and scales. The spin stands in for one biped
// serial frame on a warming worker; it is synthetic load by label, so the
// on-vs-off p95 answers the contention question on any rig.
uint64_t
CalibrateSpinIters(double targetUs)
{
    volatile double sink = 0.0;
    const uint64_t probe = 1000000;
    std::vector<double> rates;
    for (int round = 0; round < 3; ++round) {
        const double start = NowUs();
        for (uint64_t i = 0; i < probe; ++i) {
            sink += double(i) * 0.5;
        }
        const double us = NowUs() - start;
        if (us > 0.0) {
            rates.push_back(double(probe) / us);
        }
    }
    (void)sink;
    if (rates.empty()) {
        return probe;
    }
    return uint64_t(Percentile(rates, 0.5) * targetUs);
}

void
SpinIters(uint64_t iters)
{
    volatile double sink = 0.0;
    for (uint64_t i = 0; i < iters; ++i) {
        sink += double(i) * 0.5;
    }
    (void)sink;
}

std::vector<double>
TimeUiEvals(BenchRig *rig, const std::vector<double> &frames, size_t samples)
{
    std::vector<double> out;
    out.reserve(samples);
    for (size_t i = 0; i < samples; ++i) {
        const double t = frames[i % frames.size()];
        const double start = NowUs();
        const RigExecRigPose pose =
            rig->evaluator->Evaluate(UsdTimeCode(t));
        out.push_back(NowUs() - start);
        if (!pose.valid) {
            std::printf("  UI eval invalid at %g; stopping the series\n",
                        t);
            break;
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// UI-eval latency with warming on/off: the calling thread's live-eval cost
// alone, then under a draining pool (plumbing first, then synthetic serial
// load), median and p95 each. A bench, not a gate: wall-clock assertions
// flake on shared runners.
// ---------------------------------------------------------------------------

bool
BenchLatency(BenchRig *rig, const std::vector<double> &frames)
{
    std::printf("[tiny] UI-eval latency, warming off vs on:\n");
    constexpr size_t kSamples = 200;

    // Warmup first: the first series a process times pays frequency
    // scaling and allocator cold-start, which would otherwise masquerade
    // as a warming effect. The trailing off-repeat exposes any drift the
    // warmup missed (A/B/A).
    TimeUiEvals(rig, frames, kSamples);
    const std::vector<double> off =
        TimeUiEvals(rig, frames, kSamples);
    if (off.size() != kSamples) {
        return false;
    }
    PrintDistribution("warming off", off, "us/eval");

    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("  fill gate closed by the environment "
                    "(RIGEXEC_FRAME_CACHE=%s, parallel=%s): on-modes "
                    "skipped, not failed\n",
                    GetEnv("RIGEXEC_FRAME_CACHE")
                        ? GetEnv("RIGEXEC_FRAME_CACHE")
                        : "(unset)",
                    RigExecParallelEvaluationEnabled() ? "on" : "off");
        return true;
    }

    // Plumbing: a real pool, real queue, real fences, trivial work. Any
    // gap to off here is scheduling overhead, not contention.
    {
        RigExecBackgroundScheduler sched(
            kRigExecBackgroundSchedulerDefaultWorkers);
        const RigExecWarmWork trivial =
            [](const RigExecWarmRequest &) {
                return RigExecWarmOutcome::Published;
            };
        const RigExecWarmJobFactory factory =
            [&trivial](UsdTimeCode) { return trivial; };
        std::vector<UsdTimeCode> sweep;
        for (int t = 100; t < 164; ++t) {
            sweep.push_back(UsdTimeCode(double(t)));
        }
        sched.OnEditCommitted(rig->rig, UsdTimeCode(frames[0]),
                              sched.CurrentGeneration(rig->rig), sweep,
                              factory);
        const std::vector<double> on =
            TimeUiEvals(rig, frames, kSamples);
        sched.WaitUntilIdle();
        const RigExecBackgroundSchedulerStats stats = sched.Stats();
        PrintDistribution("warming on (plumbing)", on, "us/eval");
        std::printf("  pool: completed=%zu canceled=%zu declined=%zu "
                    "(workers=%d)\n",
                    stats.completed, stats.canceled, stats.declined,
                    sched.GetWorkerCount());
    }

    // Contention: the same pool with each job spinning one synthetic
    // biped serial frame (~3.1 ms, report section 2) at below-normal
    // priority while the UI thread evaluates live.
    {
        const uint64_t spin = CalibrateSpinIters(3100.0);
        const double spinStart = NowUs();
        SpinIters(spin);
        const double spinMs = (NowUs() - spinStart) / 1000.0;
        RigExecBackgroundScheduler sched(
            kRigExecBackgroundSchedulerDefaultWorkers);
        const RigExecWarmWork loaded = [spin](const RigExecWarmRequest &) {
            SpinIters(spin);
            return RigExecWarmOutcome::Published;
        };
        const RigExecWarmJobFactory factory =
            [&loaded](UsdTimeCode) { return loaded; };
        // Sixteen jobs at ~3.1 ms over two workers: the pool is still
        // draining when the 200 UI evals finish, so every sample is taken
        // under load.
        std::vector<UsdTimeCode> sweep;
        for (int t = 200; t < 216; ++t) {
            sweep.push_back(UsdTimeCode(double(t)));
        }
        sched.OnEditCommitted(rig->rig, UsdTimeCode(frames[0]),
                              sched.CurrentGeneration(rig->rig), sweep,
                              factory);
        const std::vector<double> on =
            TimeUiEvals(rig, frames, kSamples);
        sched.WaitUntilIdle();
        const RigExecBackgroundSchedulerStats stats = sched.Stats();
        PrintDistribution("warming on (loaded)", on, "us/eval");
        std::printf("  pool: completed=%zu canceled=%zu declined=%zu; "
                    "synthetic job=%.1f ms spin (measured)\n",
                    stats.completed, stats.canceled, stats.declined,
                    spinMs);
    }

    const std::vector<double> offAgain =
        TimeUiEvals(rig, frames, kSamples);
    PrintDistribution("warming off (repeat)", offAgain, "us/eval");
    return true;
}

// ---------------------------------------------------------------------------
// Memory under the cap: measured per-frame bytes, eviction behavior, and the
// Stream 0 capacity arithmetic the defaults stand on.
// ---------------------------------------------------------------------------

bool
BenchMemory(BenchRig *rig, const std::vector<double> &frames)
{
    std::printf("[tiny] memory under the cap:\n");
    if (!rig->sampleable) {
        std::printf("  skipped: %s\n", rig->sampleError.c_str());
        return true;
    }
    RigExecFrameCache cache;
    std::printf("  %-22s %10zu bytes (%zu MiB)\n", "default byte cap",
                cache.GetByteCap(),
                cache.GetByteCap() / (1024 * 1024));

    // 64 distinct frames: 16 times x 4 drag values, all sampled live.
    const std::vector<RigExecValueOverride> noOverrides;
    size_t poseBytes = 0;
    for (int variant = 0; variant < 4; ++variant) {
        RigExecValueOverride drag;
        drag.prim = SdfPath("/Asset/Rig/AlongX");
        drag.attribute = TfToken("avars:tx");
        drag.value = VtValue(100.0 + double(variant));
        const std::vector<RigExecValueOverride> overrides = {drag};
        for (double t : frames) {
            const RigExecRigPose pose =
                rig->evaluator->Evaluate(UsdTimeCode(t));
            RigExecFrameInputs inputs;
            std::string error;
            if (!pose.valid ||
                !RigExecSampleFrameInputs(*rig->evaluator,
                                          UsdTimeCode(t), overrides,
                                          &inputs, &error)) {
                std::printf("  FATAL: fill failed (%s)\n", error.c_str());
                return false;
            }
            poseBytes = RigExecFrameCachePoseBytes(pose);
            cache.Publish(KeyFor(rig->epoch, inputs, overrides),
                          UsdTimeCode(t), pose);
        }
    }
    RigExecFrameCacheStats stats = cache.Stats();
    std::printf("  %-22s %10zu bytes/frame (measured pose maps)\n",
                "tiny frame", poseBytes);
    std::printf("  %-22s entries=%zu bytes=%zu evictions=%zu\n",
                "64 frames at default", stats.entryCount, stats.bytes,
                stats.evictions);

    // Squeeze the cap and keep publishing: held bytes must stay under it
    // and evictions must count the pressure. 64 KiB holds ~47 tiny frames,
    // so the second 64-frame burst overflows it by construction.
    cache.SetByteCap(64 * 1024);
    for (int variant = 4; variant < 8; ++variant) {
        RigExecValueOverride drag;
        drag.prim = SdfPath("/Asset/Rig/AlongX");
        drag.attribute = TfToken("avars:tx");
        drag.value = VtValue(100.0 + double(variant));
        const std::vector<RigExecValueOverride> overrides = {drag};
        for (double t : frames) {
            const RigExecRigPose pose =
                rig->evaluator->Evaluate(UsdTimeCode(t));
            RigExecFrameInputs inputs;
            std::string error;
            if (!pose.valid ||
                !RigExecSampleFrameInputs(*rig->evaluator,
                                          UsdTimeCode(t), overrides,
                                          &inputs, &error)) {
                std::printf("  FATAL: squeeze fill failed (%s)\n",
                            error.c_str());
                return false;
            }
            cache.Publish(KeyFor(rig->epoch, inputs, overrides),
                          UsdTimeCode(t), pose);
        }
    }
    stats = cache.Stats();
    std::printf("  64 KiB cap: entries=%zu bytes=%zu evictions=%zu "
                "(bytes %s cap)\n",
                stats.entryCount, stats.bytes, stats.evictions,
                stats.bytes <= 64 * 1024 ? "under" : "OVER");

    // The capacity arithmetic, from the Stream 0 constants in code: the
    // report's numbers must match these, and this line is the check.
    const RigExecSparsityDecision stream0 =
        RigExecStream0SparsityDecision();
    std::printf("  Stream 0 decision in code: %s, %zu full frames at cap\n",
                stream0.go ? "GO" : "NO-GO", stream0.fullFramesAtCap);
    std::printf("  %s\n", stream0.memo.c_str());
    std::printf("  code defaults: cap=%zu workers=%d radius=+/-%d\n",
                kRigExecFrameCacheDefaultByteCap,
                kRigExecBackgroundSchedulerDefaultWorkers,
                kRigExecFrameCacheDefaultNeighborRadius);
    return true;
}

// ---------------------------------------------------------------------------
// Stress: N seconds of UI-vs-warming churn for TSAN runs (Linux). The UI
// thread evaluates live, publishes, commits, and cancels while a second
// thread hammers the fence; a TSAN-instrumented build reports any race.
// Prints counts, asserts nothing.
// ---------------------------------------------------------------------------

bool
BenchStress(BenchRig *rig, const std::vector<double> &frames, double seconds)
{
    std::printf("[tiny] UI-vs-warming stress for %.0f s:\n", seconds);
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("  skipped: fill gate closed by the environment\n");
        return true;
    }
    RigExecBackgroundScheduler sched(
        kRigExecBackgroundSchedulerDefaultWorkers);
    RigExecFrameCache cache;
    const std::vector<RigExecValueOverride> noOverrides;
    const RigExecWarmWork trivial = [](const RigExecWarmRequest &) {
        return RigExecWarmOutcome::Published;
    };
    const RigExecWarmJobFactory factory =
        [&trivial](UsdTimeCode) { return trivial; };
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> uiEvals{0};
    std::atomic<uint64_t> uiPublishes{0};
    std::atomic<uint64_t> uiCommits{0};
    std::atomic<uint64_t> churnEnqueues{0};

    std::thread churn([&]() {
        uint64_t n = 0;
        while (!stop.load()) {
            const double t = 1000.0 + double(n % 64);
            sched.Enqueue(rig->rig, UsdTimeCode(t),
                          RigExecWarmPriority::Neighbor,
                          sched.CurrentGeneration(rig->rig), trivial);
            if (++n % 16 == 0) {
                sched.CancelGeneration(rig->rig);
            }
            ++churnEnqueues;
        }
    });

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(seconds);
    uint64_t n = 0;
    size_t invalid = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const double t = frames[n % frames.size()];
        const RigExecRigPose pose =
            rig->evaluator->Evaluate(UsdTimeCode(t));
        ++uiEvals;
        if (!pose.valid) {
            ++invalid;
        } else if (n % 8 == 0) {
            RigExecFrameInputs inputs;
            if (RigExecSampleFrameInputs(*rig->evaluator,
                                         UsdTimeCode(t), noOverrides,
                                         &inputs, nullptr) &&
                RigExecControlStateDigestible(inputs)) {
                cache.Publish(KeyFor(rig->epoch, inputs, noOverrides),
                              UsdTimeCode(t), pose);
                ++uiPublishes;
            }
        }
        if (++n % 64 == 0) {
            sched.CancelGeneration(rig->rig);
            std::vector<UsdTimeCode> sweep;
            for (int s = 0; s < 8; ++s) {
                sweep.push_back(UsdTimeCode(t + 20.0 + double(s)));
            }
            sched.OnEditCommitted(rig->rig, UsdTimeCode(t),
                                  sched.CurrentGeneration(rig->rig),
                                  sweep, factory);
            ++uiCommits;
        }
        if (n % 512 == 0) {
            sched.NotePlaybackAdvanced(rig->rig, UsdTimeCode(t));
        }
    }
    stop.store(true);
    churn.join();
    sched.WaitUntilIdle();
    const RigExecBackgroundSchedulerStats schedStats = sched.Stats();
    const RigExecFrameCacheStats cacheStats = cache.Stats();
    std::printf("  UI: evals=%llu publishes=%llu commits=%llu invalid=%zu\n",
                (unsigned long long)uiEvals.load(),
                (unsigned long long)uiPublishes.load(),
                (unsigned long long)uiCommits.load(), invalid);
    std::printf("  churn: enqueues=%llu\n",
                (unsigned long long)churnEnqueues.load());
    std::printf("  scheduler: queued=%zu running=%zu completed=%zu "
                "droppedStale=%zu canceled=%zu shed=%zu coalesced=%zu "
                "upgraded=%zu declined=%zu shutdown=%zu\n",
                schedStats.queuedDepth, schedStats.running,
                schedStats.completed, schedStats.droppedStale,
                schedStats.canceled, schedStats.shed,
                schedStats.coalesced, schedStats.upgraded,
                schedStats.declined, schedStats.droppedAtShutdown);
    std::printf("  cache: hits=%zu misses=%zu published=%zu evictions=%zu "
                "entries=%zu bytes=%zu\n",
                cacheStats.hits, cacheStats.misses, cacheStats.published,
                cacheStats.evictions, cacheStats.entryCount,
                cacheStats.bytes);
    std::printf("  (run a TSAN-instrumented Linux build of this bench to "
                "race-check the UI/worker surface)\n");
    return true;
}

// ---------------------------------------------------------------------------
// Trace: a short scripted run that exercises the profiler's cache and
// scheduler lanes end to end, then writes the Chrome trace.
// ---------------------------------------------------------------------------

bool
BenchTrace(BenchRig *rig, const std::vector<double> &frames)
{
    std::printf("[tiny] profiler lane demo:\n");
    if (!rig->sampleable) {
        std::printf("  skipped: %s\n", rig->sampleError.c_str());
        return true;
    }
    const std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameCache cache;
    RigExecProfiler profiler;
    profiler.SetEnabled(true);

    {
        RIGEXEC_PROFILE_SCOPE_CAT(profiler, "benchCold", "bench");
        for (size_t i = 0; i < 4 && i < frames.size(); ++i) {
            const RigExecRigPose pose = rig->evaluator->Evaluate(
                UsdTimeCode(frames[i]));
            RigExecFrameInputs inputs;
            if (pose.valid &&
                RigExecSampleFrameInputs(*rig->evaluator,
                                         UsdTimeCode(frames[i]),
                                         noOverrides, &inputs, nullptr)) {
                cache.Publish(KeyFor(rig->epoch, inputs, noOverrides),
                              UsdTimeCode(frames[i]), pose);
            }
        }
    }
    {
        RIGEXEC_PROFILE_SCOPE_CAT(profiler, "benchWarm", "bench");
        for (size_t i = 4; i-- > 0 && i < frames.size();) {
            RigExecFrameInputs inputs;
            RigExecRigPose served;
            const bool sampled = RigExecSampleFrameInputs(
                *rig->evaluator, UsdTimeCode(frames[i]), noOverrides,
                &inputs, nullptr);
            const bool hit =
                sampled &&
                cache.Lookup(KeyFor(rig->epoch, inputs, noOverrides),
                             &served) &&
                served.valid;
            profiler.RecordCacheLookup(hit, frames[i]);
            if (i == 0) {
                break;
            }
        }
    }

    // Scheduler lane on the manual (zero-worker) pool: deterministic
    // queue depth and cancel counts, no thread timing involved.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(profiler, "benchCommit", "bench");
        RigExecBackgroundScheduler sched(0);
        const RigExecWarmWork trivial =
            [](const RigExecWarmRequest &) {
                return RigExecWarmOutcome::Published;
            };
        const RigExecWarmJobFactory factory =
            [&trivial](UsdTimeCode) { return trivial; };
        std::vector<UsdTimeCode> sweep;
        for (int t = 40; t < 48; ++t) {
            sweep.push_back(UsdTimeCode(double(t)));
        }
        const RigExecFrameGeneration gen =
            sched.CurrentGeneration(rig->rig);
        const size_t enqueued = sched.OnEditCommitted(
            rig->rig, UsdTimeCode(frames[0]), gen, sweep, factory);
        const RigExecBackgroundSchedulerStats queued = sched.Stats();
        profiler.RecordSchedulerQueue(queued.queuedDepth, queued.running,
                                      queued.canceled);
        sched.CancelGeneration(rig->rig);
        const RigExecBackgroundSchedulerStats canceled = sched.Stats();
        profiler.RecordSchedulerCancel(
            canceled.canceled - queued.canceled, "edit");
        // Draining after the purge runs nothing: the cancel is the story.
        sched.DrainQueueForTesting();
        const RigExecBackgroundSchedulerStats drained = sched.Stats();
        profiler.RecordSchedulerQueue(drained.queuedDepth,
                                      drained.running, drained.canceled);
        std::printf("  commit: enqueued=%zu queued=%zu canceled=%zu\n",
                    enqueued, queued.queuedDepth, canceled.canceled);
    }

    const std::string path = "frame-cache-warm.trace";
    std::string error;
    if (!profiler.WriteChromeTrace(path, &error)) {
        std::printf("  FATAL: cannot write %s (%s)\n", path.c_str(),
                    error.c_str());
        return false;
    }
    std::printf("  wrote %s (%zu events; open in Perfetto or "
                "chrome://tracing)\n",
                path.c_str(), profiler.GetEventCount());
    return true;
}

}  // namespace

std::string
SchemaResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return std::string();
#endif
}

int
main(int argc, char **argv)
{
    const std::string examplesDir = argc > 1 ? argv[1] : std::string();
    const std::string mode = argc > 2 ? argv[2] : "all";
    const double stressSeconds = argc > 3 ? std::atof(argv[3]) : 5.0;
    const std::string stageFile =
        argc > 4 ? argv[4] : std::string("biped/Biped_anim.usda");
    const std::string framesArg =
        argc > 5 ? argv[5] : std::string("1,2,3,4,5,6,7,8");

    const std::string resources = SchemaResourceDir();
    if (!resources.empty() &&
        PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("warning: no schema plugin found at %s (tiny-rig "
                    "benches continue; file rigs may not)\n",
                    resources.c_str());
    }

    // A bare bench measures warming: default the live switch on unless the
    // caller chose. An explicit off/warm-off is honored, and the fill
    // modes skip aloud rather than timing a gate they cannot see.
    if (!GetEnv("RIGEXEC_FRAME_CACHE")) {
        SetEnv("RIGEXEC_FRAME_CACHE", "on");
        std::printf("RIGEXEC_FRAME_CACHE was unset; set to on for this run\n");
    }
    std::printf("frame-cache mode: %s\n",
                GetEnv("RIGEXEC_FRAME_CACHE") ? GetEnv("RIGEXEC_FRAME_CACHE")
                                              : "(unset)");
    std::printf("parallel eval: %s\n",
                RigExecParallelEvaluationEnabled() ? "ON" : "OFF (serial)");
    std::printf("background fill: %s\n",
                RigExecBackgroundWarmingEnabled() ? "OPEN" : "CLOSED");
    std::printf("priority backend: %s\n",
                RigExecBackgroundPriorityBackend());
    std::printf("sizeof(RigExecPointFrame)=%zu sizeof(GfMatrix4d)=%zu "
                "sizeof(SdfPath)=%zu sizeof(VtValue)=%zu\n",
                sizeof(RigExecPointFrame), sizeof(GfMatrix4d),
                sizeof(SdfPath), sizeof(VtValue));

    const bool wantScrub =
        mode == "all" || mode == "scrub";
    const bool wantEdit = mode == "all" || mode == "edit";
    const bool wantLatency = mode == "all" || mode == "latency";
    const bool wantMemory = mode == "all" || mode == "memory";
    const bool wantStress = mode == "all" || mode == "stress";
    const bool wantTrace = mode == "all" || mode == "trace";
    if (!wantScrub && !wantEdit && !wantLatency && !wantMemory &&
        !wantStress && !wantTrace) {
        std::printf("unknown mode '%s' (want all|scrub|edit|latency|memory|"
                    "stress|trace)\n",
                    mode.c_str());
        return 2;
    }

    std::vector<double> tinyFrames;
    for (int t = 1; t <= 16; ++t) {
        tinyFrames.push_back(double(t));
    }
    BenchRig tiny;
    if (!PrepareRig("tiny", MakeTinyRig(64, 1, 16), &tiny)) {
        return 2;
    }

    bool ok = true;
    if (wantScrub) {
        ok = BenchScrub(&tiny, "tiny", tinyFrames) && ok;
        if (!examplesDir.empty()) {
            const std::string stagePath = examplesDir + "/" + stageFile;
            UsdStageRefPtr stage = UsdStage::Open(stagePath);
            const std::string::size_type slash = stageFile.rfind('/');
            const std::string label = slash == std::string::npos
                                          ? stageFile
                                          : stageFile.substr(slash + 1);
            std::vector<double> scrubFrames;
            for (const std::string &piece : TfStringSplit(framesArg, ",")) {
                if (!piece.empty()) {
                    scrubFrames.push_back(TfStringToDouble(piece));
                }
            }
            if (!stage) {
                std::printf("[%s] cannot open %s; skipped\n", label.c_str(),
                            stagePath.c_str());
            } else if (scrubFrames.empty()) {
                std::printf("[%s] no scrub frames parsed from '%s'; "
                            "skipped\n",
                            label.c_str(), framesArg.c_str());
            } else {
                BenchRig biped;
                if (PrepareRig(label.c_str(), stage, &biped)) {
                    ok = BenchScrub(&biped, label.c_str(), scrubFrames) && ok;
                } else {
                    ok = false;
                }
            }
        }
    }
    if (wantEdit) {
        // The edit bench mutates its stage (control samples move), so it
        // runs on a fresh rig rather than the scrub rig.
        BenchRig edited;
        if (!PrepareRig("tiny", MakeTinyRig(64, 1, 16), &edited)) {
            return 2;
        }
        ok = BenchEdit(&edited, tinyFrames) && ok;
    }
    if (wantLatency) {
        ok = BenchLatency(&tiny, tinyFrames) && ok;
    }
    if (wantMemory) {
        ok = BenchMemory(&tiny, tinyFrames) && ok;
    }
    if (wantStress) {
        ok = BenchStress(&tiny, tinyFrames,
                         stressSeconds > 0.0 ? stressSeconds : 5.0) &&
             ok;
    }
    if (wantTrace) {
        ok = BenchTrace(&tiny, tinyFrames) && ok;
    }
    return ok ? 0 : 2;
}
