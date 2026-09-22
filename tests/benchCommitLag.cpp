//
// benchCommitLag -- what a manipulation release (or undo/redo) costs at a
// held playhead with warming on.
//
// Simulates exactly what the usdview plugin does on a gizmo release, an
// undo/redo, or a panel commit: one stage edit (synchronous notices through
// the native handler plus the SetTime re-evaluation) followed by
// OnEditCommitted (the release flush's warming burst). Reports the two
// halves separately, plus a standalone breakdown of the per-frame
// BuildWarmWork ingredients (sampling, epoch digest, control digest) and
// the rig's scale, so a lag report can be attributed rather than guessed
// at. Prints human-readable numbers; asserts nothing, so it is built but
// deliberately NOT registered with ctest (like benchFrameCacheWarm).
//
//   benchCommitLag [examplesDir]
//
// Environment, read and reported, never assumed: RIGEXEC_FRAME_CACHE (unset
// is set to "on" in-process for the run, so a bare bench measures warming;
// an explicit off is honored and the commit half says it enqueued nothing)
// and RIGEXEC_ENABLE_PARALLEL_EVAL (the process default).
//

#include "rigExecImaging/registry.h"
#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
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
Median(std::vector<double> samples)
{
    if (samples.empty()) {
        return 0.0;
    }
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

// Payload bytes of one sampled value, by the same counting the digest's
// own byte helper uses: array payloads and scalar sizes; anything else
// counts the VtValue shell and lands in the "other" bucket.
size_t
SampledBytes(const VtValue &value, bool *isArray)
{
    if (isArray) {
        *isArray = false;
    }
    if (value.IsEmpty()) {
        return 0;
    }
#define _BENCH_ARRAY(type, element)                                  \
    if (value.IsHolding<type>()) {                                   \
        if (isArray) {                                               \
            *isArray = true;                                         \
        }                                                            \
        return value.UncheckedGet<type>().size() * sizeof(element);  \
    }
    _BENCH_ARRAY(VtVec3fArray, GfVec3f)
    _BENCH_ARRAY(VtVec3dArray, GfVec3d)
    _BENCH_ARRAY(VtVec2fArray, GfVec2f)
    _BENCH_ARRAY(VtVec4fArray, GfVec4f)
    _BENCH_ARRAY(VtFloatArray, float)
    _BENCH_ARRAY(VtDoubleArray, double)
    _BENCH_ARRAY(VtIntArray, int)
    _BENCH_ARRAY(VtBoolArray, bool)
    _BENCH_ARRAY(VtStringArray, std::string)
#undef _BENCH_ARRAY
#define _BENCH_SCALAR(type)        \
    if (value.IsHolding<type>()) { \
        return sizeof(type);       \
    }
    _BENCH_SCALAR(float)
    _BENCH_SCALAR(double)
    _BENCH_SCALAR(int)
    _BENCH_SCALAR(bool)
    _BENCH_SCALAR(GfVec2f)
    _BENCH_SCALAR(GfVec3f)
    _BENCH_SCALAR(GfVec3d)
    _BENCH_SCALAR(GfVec4f)
    _BENCH_SCALAR(GfMatrix4d)
    _BENCH_SCALAR(TfToken)
    _BENCH_SCALAR(SdfPath)
#undef _BENCH_SCALAR
    if (value.IsHolding<std::string>()) {
        return value.UncheckedGet<std::string>().size();
    }
    return sizeof(VtValue);
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

// The first double-typed avars:* attribute under the rig: the stand-in for
// the control a gizmo release (or an undo) rewrites.
UsdAttribute
FindAvarDouble(const UsdStageRefPtr &stage, const SdfPath &rig)
{
    const UsdPrim root = stage->GetPrimAtPath(rig);
    for (const UsdPrim &prim : UsdPrimRange(root)) {
        for (const UsdAttribute &attr : prim.GetAttributes()) {
            const std::string name = attr.GetName().GetString();
            if (name.compare(0, 6, "avars:") == 0 &&
                attr.GetTypeName() == SdfValueTypeNames->Double) {
                return attr;
            }
        }
    }
    return UsdAttribute();
}

}  // namespace

int
main(int argc, char **argv)
{
    const std::string examplesDir =
        argc > 1 ? argv[1] : std::string("examples");
    if (!GetEnv("RIGEXEC_FRAME_CACHE")) {
        SetEnv("RIGEXEC_FRAME_CACHE", "on");
    }
    std::printf("RIGEXEC_FRAME_CACHE=%s\n",
                GetEnv("RIGEXEC_FRAME_CACHE"));
    std::printf("RIGEXEC_ENABLE_PARALLEL_EVAL=%s\n",
                GetEnv("RIGEXEC_ENABLE_PARALLEL_EVAL") ?
                    GetEnv("RIGEXEC_ENABLE_PARALLEL_EVAL") : "(unset)");
    std::printf("background warming enabled: %s\n",
                RigExecBackgroundWarmingEnabled() ? "yes" : "NO");

    const std::string stagePath = examplesDir + "/biped/Biped_anim.usda";
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        std::printf("FATAL: cannot open %s\n", stagePath.c_str());
        return 1;
    }
    const SdfPath rig = FindRig(stage);
    if (rig.IsEmpty()) {
        std::printf("FATAL: no RigExecRoot on %s\n", stagePath.c_str());
        return 1;
    }
    double playheadValue = 1.0;
    const double stageStart = stage->GetStartTimeCode();
    if (std::isfinite(stageStart)) {
        playheadValue = stageStart;
    }
    const UsdTimeCode playhead(playheadValue);
    std::printf("[biped] %s rig=%s playhead=%g\n", stagePath.c_str(),
                rig.GetString().c_str(), playheadValue);

    // Standalone breakdown first: the per-frame ingredients outside the
    // registry, on a second evaluator over the same stage.
    RigExecRigEvaluator evaluator(stage, rig);
    {
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            std::printf("FATAL: the biped did not compile\n");
            for (const std::string &error : errors) {
                std::printf("  %s\n", error.c_str());
            }
            return 1;
        }
    }
    {
        std::vector<std::string> reasons;
        if (!RigExecBakedProgram::IsBakeable(evaluator, &reasons)) {
            std::printf("FATAL: the biped does not bake\n");
            for (const std::string &reason : reasons) {
                std::printf("  %s\n", reason.c_str());
            }
            return 1;
        }
    }
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        std::printf("FATAL: the biped built no program\n");
        return 1;
    }
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    std::printf("[biped] providers=%zu bound=%zu varying=%zu "
                "avarConstants=%zu avarBindings=%zu chains=%zu\n",
                program->GetProviderCount(), program->GetBoundInputCount(),
                program->GetVaryingInputCount(), impl.avarConstants.size(),
                impl.avarConstantBindings.size(), impl.chains.size());
    const std::vector<RigExecValueOverride> noOverrides;
    {
        RigExecFrameInputs probe;
        std::string error;
        const bool sampleable = RigExecSampleFrameInputs(
            evaluator, playhead, noOverrides, &probe, &error);
        std::printf("[biped] sampleable=%s%s%s\n",
                    sampleable ? "yes" : "NO",
                    sampleable ? "" : " (",
                    sampleable ? "" : (error + ")").c_str());
    }
    {
        std::vector<double> epoch;
        for (size_t i = 0; i < 200; ++i) {
            const double start = NowUs();
            volatile uint64_t digest =
                RigExecFrameCacheEpochDigest(evaluator);
            (void)digest;
            epoch.push_back(NowUs() - start);
        }
        std::printf("  %-28s %10.1f us (median of %zu)\n", "epoch digest",
                    Median(epoch), epoch.size());
    }
    RigExecFrameInputs sampled;
    {
        std::vector<double> samples;
        for (int step = 0; step < 8; ++step) {
            RigExecFrameInputs inputs;
            std::string error;
            const double start = NowUs();
            const bool ok = RigExecSampleFrameInputs(
                evaluator, UsdTimeCode(playheadValue + double(step)),
                noOverrides, &inputs, &error);
            samples.push_back(NowUs() - start);
            if (!ok) {
                std::printf("  sample at %g declined: %s\n",
                            playheadValue + double(step), error.c_str());
                break;
            }
            if (step == 0) {
                sampled = inputs;
            }
        }
        std::printf("  %-28s %10.1f us (median of %zu)\n",
                    "sample one frame", Median(samples), samples.size());
    }
    if (!sampled.values.empty()) {
        std::vector<double> digests;
        for (size_t i = 0; i < 200; ++i) {
            const double start = NowUs();
            volatile uint64_t digest =
                RigExecControlStateDigest(sampled, noOverrides);
            (void)digest;
            digests.push_back(NowUs() - start);
        }
        std::printf("  %-28s %10.1f us (median of %zu)\n",
                    "control digest", Median(digests), digests.size());
    }
    {
        // Chain-bind cost apart from the sample: production pins the
        // bindings and currency-checks them per frame instead.
        std::vector<double> binds;
        for (size_t i = 0; i < 20; ++i) {
            RigExecChainSampleBindings fresh;
            std::string error;
            const double start = NowUs();
            volatile bool ok =
                RigExecBindChainSampleInputs(evaluator, &fresh, &error);
            (void)ok;
            binds.push_back(NowUs() - start);
        }
        std::printf("  %-28s %10.1f us (median of %zu)\n",
                    "chain bind", Median(binds), binds.size());
    }
    {
        // Pinned-route anatomy: one bind, then per-frame currency, hook,
        // and full pinned samples -- what production's BuildWarmWork pays
        // per frame (three currency checks: refresh pins, the explicit
        // check, and the sampler's own entry check).
        RigExecChainSampleBindings pinned;
        std::string bindError;
        if (!RigExecBindChainSampleInputs(evaluator, &pinned,
                                          &bindError)) {
            std::printf("  pinned bind failed: %s\n", bindError.c_str());
        } else {
            std::vector<double> current;
            for (size_t i = 0; i < 200; ++i) {
                const double start = NowUs();
                volatile bool ok =
                    RigExecChainSampleBindingsStillCurrent(pinned,
                                                           evaluator);
                (void)ok;
                current.push_back(NowUs() - start);
            }
            std::printf("  %-28s %10.1f us (median of %zu, x3/frame)\n",
                        "chain currency", Median(current),
                        current.size());
            std::vector<double> hooks;
            for (int step = 0; step < 8; ++step) {
                RigExecResolvedInputs hooked;
                std::map<SdfPath, VtValue> results;
                std::vector<std::string> diagnostics;
                std::string error;
                const double start = NowUs();
                volatile bool ok = RigExecEvaluateChainsForTime(
                    pinned, UsdTimeCode(playheadValue + double(step)),
                    &hooked, &results, &diagnostics, &error);
                (void)ok;
                hooks.push_back(NowUs() - start);
            }
            std::printf("  %-28s %10.1f us (median of %zu)\n",
                        "chain hook", Median(hooks), hooks.size());
            std::vector<double> pinnedSamples;
            for (int step = 0; step < 8; ++step) {
                RigExecFrameInputs inputs;
                std::string error;
                const double start = NowUs();
                volatile bool ok =
                    RigExecSampleFrameInputsWithChainBindings(
                        evaluator,
                        UsdTimeCode(playheadValue + double(step)),
                        noOverrides, pinned, &inputs, &error);
                (void)ok;
                pinnedSamples.push_back(NowUs() - start);
            }
            std::printf("  %-28s %10.1f us (median of %zu)\n",
                        "pinned sample", Median(pinnedSamples),
                        pinnedSamples.size());
        }
    }
    if (!sampled.values.empty()) {
        // Composition of one sampled vector: what the per-frame sample
        // reads and the digest folds. The static bucket (attributes that
        // cannot vary with time) sizes a burst-scoped sample cache: reads
        // a first-frame sample could serve to every later frame instead.
        size_t arrays = 0, arrayBytes = 0, scalars = 0, other = 0;
        size_t valueless = 0, totalBytes = 0;
        size_t staticCount = 0, staticBytes = 0;
        size_t varyingCount = 0, varyingBytes = 0;
        size_t unresolvedCount = 0, unresolvedBytes = 0;
        for (const RigExecSampledInput &s : sampled.values) {
            if (!s.hasValue) {
                ++valueless;
                continue;
            }
            bool isArray = false;
            const size_t bytes = SampledBytes(s.value, &isArray);
            totalBytes += bytes;
            if (isArray) {
                ++arrays;
                arrayBytes += bytes;
            } else if (bytes <= 128) {
                ++scalars;
            } else {
                ++other;
            }
            const UsdAttribute attr = stage->GetAttributeAtPath(s.path);
            if (!attr.IsValid()) {
                ++unresolvedCount;
                unresolvedBytes += bytes;
            } else if (attr.ValueMightBeTimeVarying()) {
                ++varyingCount;
                varyingBytes += bytes;
            } else {
                ++staticCount;
                staticBytes += bytes;
            }
        }
        std::printf("  samples: %zu values (%zu arrays %zu bytes, "
                    "%zu scalars, %zu other, %zu valueless, %zu bytes "
                    "total)\n",
                    sampled.values.size(), arrays, arrayBytes, scalars,
                    other, valueless, totalBytes);
        std::printf("  static: %zu values %zu bytes; varying: %zu values "
                    "%zu bytes; unresolved: %zu values %zu bytes\n",
                    staticCount, staticBytes, varyingCount, varyingBytes,
                    unresolvedCount, unresolvedBytes);
        std::printf("  chainResults: %zu revisionPackets: %zu\n",
                    sampled.chainResults.size(),
                    sampled.revisionPackets.size());
    }

    // The mouse-up simulation: an edit plus the release flush's commit, at
    // a held playhead, with the background drained before each round so
    // every commit samples the full burst (the fresh-release case).
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    {
        std::vector<std::string> errors;
        if (!registry.Activate(stage, rig, playhead, &errors)) {
            std::printf("FATAL: activation failed\n");
            for (const std::string &error : errors) {
                std::printf("  %s\n", error.c_str());
            }
            return 1;
        }
    }
    const UsdAttribute avar = FindAvarDouble(stage, rig);
    if (!avar.IsValid()) {
        std::printf("FATAL: no double avars:* attribute under %s\n",
                    rig.GetString().c_str());
        return 1;
    }
    double v0 = 0.0;
    if (!avar.Get(&v0, playhead)) {
        std::printf("FATAL: cannot read %s at the playhead\n",
                    avar.GetPath().GetString().c_str());
        return 1;
    }
    const double v1 = v0 + 1.0;
    std::printf("[biped] release edit rewrites %s (%g <-> %g at %g)\n",
                avar.GetPath().GetString().c_str(), v0, v1, playheadValue);
    std::vector<double> edits;
    std::vector<double> commits;
    std::vector<size_t> enqueuedCounts;
    for (size_t round = 0; round < 5; ++round) {
        registry.WaitUntilBackgroundIdle();
        const double value = (round % 2 == 0) ? v1 : v0;
        const double editStart = NowUs();
        if (!avar.Set(value, playhead)) {
            std::printf("FATAL: round %zu could not author the edit\n",
                        round);
            return 1;
        }
        edits.push_back(NowUs() - editStart);
        const double commitStart = NowUs();
        const size_t enqueued = registry.OnEditCommitted();
        commits.push_back(NowUs() - commitStart);
        enqueuedCounts.push_back(enqueued);
    }
    std::printf("  %-28s %10.1f us (median of %zu)\n",
                "edit: Set + notices + re-eval", Median(edits),
                edits.size());
    std::printf("  %-28s %10.1f us (median of %zu)\n",
                "commit: OnEditCommitted", Median(commits),
                commits.size());
    std::printf("  enqueued per commit:");
    for (size_t count : enqueuedCounts) {
        std::printf(" %zu", count);
    }
    std::printf("\n");
    {
        // The already-warming case: a second commit with no new edit pays
        // only the already-queued skips, without sampling.
        const double start = NowUs();
        const size_t enqueued = registry.OnEditCommitted();
        std::printf("  %-28s %10.1f us (enqueued %zu)\n",
                    "commit with no new edit", NowUs() - start, enqueued);
    }
    {
        const RigExecBackgroundSchedulerStats stats =
            registry.GetBackgroundStats();
        std::printf("  scheduler: queued=%zu running=%zu completed=%zu "
                    "droppedStale=%zu canceled=%zu coalesced=%zu "
                    "declined=%zu\n",
                    stats.queuedDepth, stats.running, stats.completed,
                    stats.droppedStale, stats.canceled, stats.coalesced,
                    stats.declined);
    }
    registry.WaitUntilBackgroundIdle();
    registry.Deactivate();
    return 0;
}
