//
// RigExec frozen contexts, Stream B: the sampled input vector, the
// epoch-pinned context, the UI-thread sampler, and the fail-closed frozen
// worker entry.
//
// FrameInputs is the type the UI thread samples at enqueue time and the
// worker reads from, so its container semantics are asserted here: values
// are found by path, a missing path is null rather than a default, an
// explicitly valueless source is still a recorded source, and Clear empties
// the vector and resets the time.
//
// The sampler (RigExecSampleFrameInputs) walks the baked program's varying
// bindings through the same route the frame path reads, so its fidelity is
// asserted against two independent oracles: the authored time samples the
// test set itself, and direct stage reads at the sampled time. The digest
// (RigExecFrozenControlDigest) must move with every control edit and stand
// still across re-samples, because the frame cache keys on it.
//
// The frozen worker entry (RigExecEvaluateFrozen with a runner) is asserted
// through an injected serial kernel: bit-identical across runs, equivalent
// to the same kernel run directly, refusing every inconsistent request
// (wrong epoch, truncated vector, stale generation, refusal flag), and safe
// under concurrent runs. What the runner CAN be is bounded too: the
// context's "no live pointers" rule is held structurally -- the type is
// trivially copyable, which a member holding a stage, an evaluator, or a
// USD handle cannot be -- and the purity audit names every unit the frozen
// path was checked against.
//

#include "rigExec/frozenContext.h"
#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/frameCache.h"
#include "rigExec/generation.h"
#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecRigging/rigBuilder.h"

#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

// Values are found by path, and a path never sampled finds nothing -- not a
// default, not a neighbor's value.
void
TestValuesAreFoundByPath()
{
    RigExecFrameInputs inputs;
    inputs.time = UsdTimeCode(4.0);
    inputs.Add(SdfPath("/Rig/Ctl.avars:tx"), VtValue(1.5));
    inputs.Add(SdfPath("/Rig/Ctl.avars:ty"), VtValue(2.5));

    const VtValue *tx = inputs.Find(SdfPath("/Rig/Ctl.avars:tx"));
    CHECK(tx != nullptr);
    CHECK(tx->IsHolding<double>());
    CHECK(tx->Get<double>() == 1.5);
    CHECK(inputs.Find(SdfPath("/Rig/Ctl.avars:tz")) == nullptr);
    CHECK(inputs.Contains(SdfPath("/Rig/Ctl.avars:ty")));
    CHECK(!inputs.Contains(SdfPath("/Rig/Ctl.avars:tz")));
}

// A source that held no value at the sampled time is recorded as valueless,
// which the worker must reproduce exactly: Find answers the (empty) slot,
// and Contains still sees the path.
void
TestAValuelessSourceIsStillRecorded()
{
    RigExecFrameInputs inputs;
    inputs.Add(SdfPath("/Rig/Ctl.avars:tx"), VtValue(), /*hasValue=*/false);
    CHECK(inputs.Contains(SdfPath("/Rig/Ctl.avars:tx")));
    const VtValue *found = inputs.Find(SdfPath("/Rig/Ctl.avars:tx"));
    CHECK(found != nullptr);
    CHECK(found->IsEmpty());
}

// The first sample at a path wins: a resampled path keeps its enqueue-time
// value, never a later one.
void
TestTheFirstSampleAtAPathWins()
{
    RigExecFrameInputs inputs;
    inputs.Add(SdfPath("/Rig/Ctl.avars:tx"), VtValue(1.0));
    inputs.Add(SdfPath("/Rig/Ctl.avars:tx"), VtValue(9.0));
    const VtValue *found = inputs.Find(SdfPath("/Rig/Ctl.avars:tx"));
    CHECK(found != nullptr);
    CHECK(found->Get<double>() == 1.0);
}

// Clear empties the vector and resets the time, so a reused struct cannot
// serve one frame's inputs at another frame's time.
void
TestClearEmptiesTheVectorAndResetsTime()
{
    RigExecFrameInputs inputs;
    inputs.time = UsdTimeCode(6.0);
    inputs.Add(SdfPath("/Rig/Ctl.avars:tx"), VtValue(1.0));
    inputs.chainDiagnostics.push_back("diag");
    inputs.chainResults[SdfPath("/Rig/Ctl.avars:tx")] = VtValue(1.0);
    inputs.stageSeeds.xformBase.push_back(GfMatrix4d(1.0));
    inputs.stageSeeds.nativeOk.push_back(1);
    inputs.stageSeeds.deltaBase.push_back(GfMatrix4d(1.0));
    inputs.Clear();
    CHECK(inputs.time == UsdTimeCode::Default());
    CHECK(inputs.values.empty());
    CHECK(inputs.Find(SdfPath("/Rig/Ctl.avars:tx")) == nullptr);
    CHECK(inputs.chainDiagnostics.empty());
    CHECK(inputs.chainResults.empty());
    CHECK(inputs.stageSeeds.xformBase.empty());
    CHECK(inputs.stageSeeds.xformFrames.empty());
    CHECK(inputs.stageSeeds.nativeOk.empty());
    CHECK(inputs.stageSeeds.nativeFrames.empty());
    CHECK(inputs.stageSeeds.deltaOk.empty());
    CHECK(inputs.stageSeeds.deltaBase.empty());
}

// The context pins an epoch and a generation and sizes an arena, and nothing
// else: trivially copyable, so no member can hold the stage, the evaluator,
// or a USD handle.
void
TestTheContextPinsAnEpochAndSizesAnArena()
{
    static_assert(std::is_trivially_copyable<RigExecFrozenEvalContext>::value,
                  "a frozen context must be plain digests and counts");
    RigExecFrozenEvalContext context;
    CHECK(context.epochDigest == 0);
    CHECK(context.generation == 0);
    CHECK(context.slotCount == 0);
    CHECK(context.programDigest == 0);
    CHECK(context.varyingInputCount == 0);
    CHECK(context.flags == 0);
    context.epochDigest = 0x1234;
    context.generation = 7;
    context.slotCount = 128;
    context.programDigest = 0xabcd;
    context.varyingInputCount = 11;
    context.flags = kRigExecFrozenPublishWeightFields;
    const RigExecFrozenEvalContext copy = context;
    CHECK(copy.epochDigest == 0x1234);
    CHECK(copy.generation == 7);
    CHECK(copy.slotCount == 128);
    CHECK(copy.programDigest == 0xabcd);
    CHECK(copy.varyingInputCount == 11);
    CHECK(copy.flags == kRigExecFrozenPublishWeightFields);
}

// The stub evaluator: every request answers invalid -- the fail-closed
// answer, which sends the caller down the live path -- while still carrying
// the requested time for the fallback's own logging.
void
TestTheStubEvaluatorAnswersInvalidAtTheRequestedTime()
{
    RigExecFrozenEvalContext context;
    context.epochDigest = 42;
    RigExecFrameInputs inputs;
    inputs.time = UsdTimeCode(9.0);
    inputs.Add(SdfPath("/Rig/Ctl.avars:tx"), VtValue(1.0));
    const RigExecRigPose pose = RigExecEvaluateFrozen(context, inputs);
    CHECK(!pose.valid);
    CHECK(pose.time == UsdTimeCode(9.0));
}

// ---------------------------------------------------------------------------
// Stream B: the sampler and its oracles.
// ---------------------------------------------------------------------------

constexpr size_t kTinyPointCount = 8;

double
TinyTx(double t)
{
    return 10.0 + 0.1 * t;
}

double
TinyTy(double t)
{
    return 20.0 - 0.05 * t;
}

// One skinned mesh over two animated controls: the bench's
// MakeAnimatedMultiMeshRig at one mesh and eight points. The two control
// avars carry time samples at 1..4, so consecutive frames genuinely differ
// instead of hashing alike. Everything else is epoch-constant, so the
// sampled vector is exactly the two varying avars plus the one chain base.
struct _TinyRigOptions {
    // With a static tx, the avar binds as a patchable constant instead of
    // a varying input: the copy-on-write test's edit target.
    bool staticTx = false;
};

UsdStageRefPtr
MakeTinyRigWith(const _TinyRigOptions &options)
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
    if (options.staticTx) {
        tx.Set(TinyTx(2.0));
    } else {
        for (int t = 1; t <= 4; ++t) {
            tx.Set(TinyTx(double(t)), UsdTimeCode(double(t)));
        }
    }
    for (int t = 1; t <= 4; ++t) {
        ty.Set(TinyTy(double(t)), UsdTimeCode(double(t)));
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

UsdStageRefPtr
MakeTinyRig()
{
    return MakeTinyRigWith(_TinyRigOptions());
}

// The sampler reproduces what the frame path reads: each sampled avar equals
// both the authored time sample and a direct stage read at the sampled
// time, and the vector holds exactly the two varying bindings plus the one
// chain base plus the one authored mover scalar -- the bench's 9-mesh
// breakdown (2 query reads, N base reads) at one mesh, with the packet
// assembly's per-frame reads sampled beside them.
void
TestSamplerMatchesLiveReads()
{
    UsdStageRefPtr stage = MakeTinyRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(1.0));
    CHECK(live.valid);
    CHECK(evaluator.GetBakedProgram() != nullptr);
    CHECK(evaluator.GetBakedGenerationCount() == 1);

    RigExecFrameInputs inputs;
    std::string error;
    std::vector<RigExecValueOverride> noOverrides;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(1.0), noOverrides,
                                   &inputs, &error));
    CHECK(error.empty());
    CHECK(inputs.time == UsdTimeCode(1.0));
    CHECK(inputs.values.size() == 6);

    const SdfPath txPath("/Asset/Rig/AlongX.avars:tx");
    const SdfPath tyPath("/Asset/Rig/AlongY.avars:ty");
    const VtValue *tx = inputs.Find(txPath);
    const VtValue *ty = inputs.Find(tyPath);
    CHECK(tx != nullptr);
    CHECK(ty != nullptr);
    CHECK(tx->IsHolding<double>());
    CHECK(ty->IsHolding<double>());
    // Oracle one: the authored time samples.
    CHECK(tx->Get<double>() == TinyTx(1.0));
    CHECK(ty->Get<double>() == TinyTy(1.0));
    // Oracle two: direct stage reads at the sampled time.
    double stageTx = 0.0, stageTy = 0.0;
    CHECK(stage->GetAttributeAtPath(txPath).Get(&stageTx,
                                               UsdTimeCode(1.0)));
    CHECK(stage->GetAttributeAtPath(tyPath).Get(&stageTy,
                                               UsdTimeCode(1.0)));
    CHECK(tx->Get<double>() == stageTx);
    CHECK(ty->Get<double>() == stageTy);
    // The chain base travels with the vector: the worker cannot read it.
    CHECK(inputs.Contains(SdfPath("/Asset/Geom/Mesh_0.points")));
    // So do the mover's per-frame scalars (the packet assembly reads them
    // per frame): the authored defaultWeight plus the schema fallbacks for
    // the unauthored enabled and skinningMethod, each matching a direct
    // stage read the way the avars do above.
    const VtValue *moverWeight = inputs.Find(
        SdfPath("/Asset/Rig/Movers/Skin_0.inputs:defaultWeight"));
    const VtValue *moverEnabled = inputs.Find(
        SdfPath("/Asset/Rig/Movers/Skin_0.inputs:enabled"));
    const VtValue *moverMethod = inputs.Find(
        SdfPath("/Asset/Rig/Movers/Skin_0.rigExec:skinningMethod"));
    CHECK(moverWeight != nullptr);
    CHECK(moverEnabled != nullptr);
    CHECK(moverMethod != nullptr);
    CHECK(moverWeight->IsHolding<float>());
    CHECK(moverEnabled->IsHolding<bool>());
    CHECK(moverMethod->IsHolding<TfToken>());
    CHECK(moverWeight->Get<float>() == 1.0f);
    float stageWeight = 0.0f;
    bool stageEnabled = false;
    TfToken stageMethod;
    CHECK(stage->GetAttributeAtPath(
                  SdfPath("/Asset/Rig/Movers/Skin_0.inputs:defaultWeight"))
              .Get(&stageWeight, UsdTimeCode(1.0)));
    CHECK(stage->GetAttributeAtPath(
                  SdfPath("/Asset/Rig/Movers/Skin_0.inputs:enabled"))
              .Get(&stageEnabled, UsdTimeCode(1.0)));
    CHECK(stage->GetAttributeAtPath(
                  SdfPath("/Asset/Rig/Movers/Skin_0.rigExec:skinningMethod"))
              .Get(&stageMethod, UsdTimeCode(1.0)));
    CHECK(moverWeight->Get<float>() == stageWeight);
    CHECK(moverEnabled->Get<bool>() == stageEnabled);
    CHECK(moverMethod->Get<TfToken>() == stageMethod);
    // And the assembled skin packet travels beside the values (one chain
    // revision), excluded from the digest as derived.
    CHECK(inputs.revisionPackets.size() == 1);
    CHECK(inputs.revisionPackets[0].valid);
    CHECK(inputs.revisionPackets[0].kind == TfToken("skin"));
    CHECK(inputs.chainDiagnostics.empty());
    CHECK(inputs.overrides.empty());
}

// The digest moves with every control edit and stands still across
// re-samples: a scrub between identical states hits, a drag never serves a
// pre-drag pose, and the standing overrides hash with the authored values.
void
TestDigestMovesWithControls()
{
    UsdStageRefPtr stage = MakeTinyRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameInputs at1, at1Again, at2;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(1.0), noOverrides,
                                   &at1));
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(1.0), noOverrides,
                                   &at1Again));
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0), noOverrides,
                                   &at2));
    bool exact = false;
    const uint64_t digest1 = RigExecFrozenControlDigest(at1, &exact);
    CHECK(exact);
    const uint64_t digest1Again = RigExecFrozenControlDigest(at1Again, &exact);
    CHECK(exact);
    const uint64_t digest2 = RigExecFrozenControlDigest(at2, &exact);
    CHECK(exact);
    CHECK(digest1 == digest1Again);
    CHECK(digest1 != digest2);

    // A held drag changes the digest at the same frame: the override is
    // sampled from the caller's list because it never reaches the stage.
    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/AlongX");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.25);
    RigExecFrameInputs dragged;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(1.0), {drag},
                                   &dragged));
    const uint64_t digestDragged = RigExecFrozenControlDigest(dragged);
    CHECK(digestDragged != digest1);
    CHECK(dragged.values.size() == at1.values.size() + 1);

    // A valueless source hashes apart from an unrecorded one.
    RigExecFrameInputs withGap = at1;
    withGap.Add(SdfPath("/Asset/Rig/AlongX.avars:tz"), VtValue(),
                /*hasValue=*/false);
    CHECK(RigExecFrozenControlDigest(withGap) != digest1);
}

// ---------------------------------------------------------------------------
// Increment B: the chain-sampling hook. The tiny rig plus one float math
// mover revising the tx avar by a time-varying factor -- the biped's foot
// chains in miniature (compare examples/09_PropertyMathMovers.usda): the
// chain's output at the sampled time exists nowhere until the hook runs,
// so a sampler that read the standing state would warm frame 3 with
// frame 2's chain values.
// ---------------------------------------------------------------------------

UsdStageRefPtr
MakeChainedRig()
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
    return stage;
}

// ---------------------------------------------------------------------------
// The production frozen executor: a warming job evaluates bit-identically
// to live evaluation of the same inputs.
// ---------------------------------------------------------------------------

// Runs one warming job through the real worker entry point: freeze, sample,
// evaluate under the production runner with a live generation fence.
static RigExecRigPose
RunWarmingJob(RigExecRigEvaluator *evaluator, const SdfPath &rig,
              const std::shared_ptr<const RigExecFrozenProgram> &frozen,
              const RigExecFrameInputs &inputs,
              RigExecBackgroundScheduler *scheduler, bool *ran)
{
    RigExecFrozenEvalContext context;
    context.epochDigest = evaluator->GetBindingEpochDigest();
    context.generation = scheduler->CurrentGeneration(rig);
    context.slotCount = evaluator->GetBakedProgram()->GetProviderCount();
    // programDigest is unchecked by the runner (it has no standing program
    // to compare against); the generation fence is what retires a job whose
    // epoch moved under it.
    context.programDigest = 0;
    context.varyingInputCount = inputs.values.size();
    context.flags = 0;
    if (evaluator->GetPublishWeightFields()) {
        context.flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator->GetSolverGuidesEnabled()) {
        context.flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.frozen = frozen.get();
    RigExecRigPose pose = RigExecEvaluateFrozen(
        context, inputs, RigExecMakeProductionStepRunner(), scheduler, rig);
    if (ran) {
        *ran = pose.valid;
    }
    return pose;
}

static void
CheckPosesBitIdentical(const char *what, const RigExecRigPose &live,
                        const RigExecRigPose &warmed)
{
    CHECK(warmed.valid);
    CHECK(warmed.time == live.time);
    RigExecRigPose diff;
    RigExecComparePoses(live, warmed, &diff);
    if (diff.bakedParityMismatches != 0) {
        std::printf("FAIL %s: %zu parity mismatch(es):\n", what,
                    diff.bakedParityMismatches);
        for (const std::string &diagnostic : diff.diagnostics) {
            std::printf("    %s\n", diagnostic.c_str());
        }
    }
    CHECK(diff.bakedParityMismatches == 0);
    CHECK(diff.diagnostics.empty());
}

// A warming job's pose is bit-identical to live evaluation of the same
// inputs: freeze after frame 2, warm frame 3 (an unrun time, with genuinely
// different inputs), and diff against live at 3 from the same history. Then
// again at frame 4, and again at 3 with a held drag standing.
void
TestProductionRunnerIsBitIdenticalToLive()
{
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;

    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), noOverrides,
                                   &at3, &error));
    CHECK(!at3.HasChainResolvedInputs());
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckPosesBitIdentical("warmed frame 3", live3, warmed3);

    // Frame 4 from live's frame-3 history: re-freeze (the snapshot pins its
    // history) and warm again.
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs at4;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(4.0), noOverrides,
                                   &at4, &error));
    const RigExecRigPose warmed4 =
        RunWarmingJob(&evaluator, rig, frozen, at4, &scheduler, nullptr);
    const RigExecRigPose live4 = evaluator.Evaluate(UsdTimeCode(4.0));
    CHECK(live4.valid);
    CheckPosesBitIdentical("warmed frame 4", live4, warmed4);

    // A held drag: the override is sampled from the list, the job's flags
    // place it, and the warmed pose matches live under the same drag.
    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/AlongX");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.25);
    evaluator.SetInteractiveOverrides({drag});
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs dragged3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {drag},
                                   &dragged3, &error));
    const RigExecRigPose warmedDragged =
        RunWarmingJob(&evaluator, rig, frozen, dragged3, &scheduler, nullptr);
    const RigExecRigPose liveDragged = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(liveDragged.valid);
    CheckPosesBitIdentical("warmed frame 3 under a drag", liveDragged,
                           warmedDragged);
    evaluator.SetInteractiveOverrides({});
}

// The chained rig warms bit-identically: the hook recomputes the chain for
// the job's time (never the evaluator's last-run outputs), the sampled tx
// carries the revised value with no stale mark, the per-target results
// travel with the vector, and the warmed poses match live with zero
// parity mismatches -- including under a drag on the chain's target (the
// post-chain replacement) and on the mover's own factor input (a pre-chain
// read).
void
TestChainedRigWarmsBitIdentical()
{
    UsdStageRefPtr stage = MakeChainedRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    RigExecChainSampleBindings bound;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &bound, &error));
    CHECK(bound.chains.size() == 1);
    CHECK(bound.chains[0].targetPath ==
          SdfPath("/Asset/Rig/AlongX.avars:tx"));
    CHECK(RigExecChainSampleBindingsStillCurrent(bound, evaluator));

    // The unlock: a chained rig freezes now that the hook reproduces the
    // chains for the job's time.
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;

    // Frame 3, which live has not run: the hook's values, not frame 2's.
    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), noOverrides,
                                   &at3, &error));
    CHECK(!at3.HasChainResolvedInputs());
    CHECK(at3.chainResults.size() == 1);
    // Sensitivity: the chain's output moves every frame, so a sampler that
    // read the standing (frame-2) state would be caught below -- and the
    // tx binding consumed the revised value, not the authored base.
    {
        RigExecFrameInputs at2;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0),
                                       noOverrides, &at2, &error));
        const SdfPath txPath("/Asset/Rig/AlongX.avars:tx");
        const auto r3 = at3.chainResults.find(txPath);
        const auto r2 = at2.chainResults.find(txPath);
        CHECK(r3 != at3.chainResults.end());
        CHECK(r2 != at2.chainResults.end());
        CHECK(r3->second != r2->second);
        const VtValue *tx = at3.Find(txPath);
        CHECK(tx != nullptr && tx->IsHolding<double>());
        CHECK(tx->Get<double>() == r3->second.Get<double>());
        double authored = 0.0;
        CHECK(stage->GetAttributeAtPath(txPath).Get(&authored,
                                                   UsdTimeCode(3.0)));
        CHECK(tx->Get<double>() != authored);
    }
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckPosesBitIdentical("warmed chained frame 3", live3, warmed3);
    // The transported result is the revised value live published: the
    // chain's output, not its authored base.
    {
        const auto published = live3.movedProperties.find(
            SdfPath("/Asset/Rig/AlongX.avars:tx"));
        CHECK(published != live3.movedProperties.end());
        const auto carried = at3.chainResults.find(
            SdfPath("/Asset/Rig/AlongX.avars:tx"));
        CHECK(carried != at3.chainResults.end());
        CHECK(carried->second == published->second);
    }

    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs at4;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(4.0), noOverrides,
                                   &at4, &error));
    CHECK(!at4.HasChainResolvedInputs());
    const RigExecRigPose warmed4 =
        RunWarmingJob(&evaluator, rig, frozen, at4, &scheduler, nullptr);
    const RigExecRigPose live4 = evaluator.Evaluate(UsdTimeCode(4.0));
    CHECK(live4.valid);
    CheckPosesBitIdentical("warmed chained frame 4", live4, warmed4);

    // A drag on the chain's target replaces the chain's result; a drag on
    // the mover's factor input is read pre-chain. Both warm bit-identical.
    for (int arm = 0; arm < 2; ++arm) {
        RigExecValueOverride drag;
        if (arm == 0) {
            drag.prim = SdfPath("/Asset/Rig/AlongX");
            drag.attribute = TfToken("avars:tx");
            drag.value = VtValue(3.25);
        } else {
            drag.prim = SdfPath("/Asset/Rig/Movers/TxGain");
            drag.attribute = TfToken("inputs:value");
            drag.value = VtValue(7.0f);
        }
        evaluator.SetInteractiveOverrides({drag});
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        RigExecFrameInputs dragged;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {drag},
                                       &dragged, &error));
        CHECK(!dragged.HasChainResolvedInputs());
        const RigExecRigPose warmedDragged = RunWarmingJob(
            &evaluator, rig, frozen, dragged, &scheduler, nullptr);
        const RigExecRigPose liveDragged =
            evaluator.Evaluate(UsdTimeCode(3.0));
        CHECK(liveDragged.valid);
        CheckPosesBitIdentical(arm == 0 ? "chained drag on the target"
                                        : "chained drag on the factor",
                               liveDragged, warmedDragged);
        evaluator.SetInteractiveOverrides({});
    }
}

// A chain binding a weight object declines at every layer: the hook names
// the chain but refuses to evaluate it (the envelope resolves through the
// evaluator's live oracle), the sampler falls back to the standing state
// and marks viaChain, the freeze refuses the rig, and the runner declines
// the marked vector. Live still evaluates -- decline is never failure.
void
TestChainHookDeclinesWeightObjects()
{
    UsdStageRefPtr stage = MakeChainedRig();
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Rig/AlongX.avars:tx")});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float)
        .Set(0.5f);
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/TxGain"))
        .CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({weight.GetPath()});

    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecChainSampleBindings bound;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &bound, &error));
    CHECK(bound.chains.size() == 1);
    CHECK(bound.chains[0].revisions.size() == 1);
    CHECK(bound.chains[0].revisions[0].weightObjects.size() == 1);

    RigExecResolvedInputs resolved;
    CHECK(!RigExecEvaluateChainsForTime(bound, UsdTimeCode(2.0), &resolved,
                                        nullptr, nullptr, &error));
    CHECK(!error.empty());

    std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameInputs at2;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0), noOverrides,
                                   &at2, &error));
    CHECK(at2.HasChainResolvedInputs());
    CHECK(at2.chainResults.empty());

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(!RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen == nullptr);
    CHECK(error.find("weight object") != std::string::npos);

    // The marked vector declines at the runner even against a snapshot
    // taken before the weight object was bound.
    CHECK(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/TxGain"))
              .GetRelationship(TfToken("rigExec:weightObject"))
              .SetTargets({}));
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    RigExecBackgroundScheduler scheduler;
    bool ran = true;
    const RigExecRigPose pose =
        RunWarmingJob(&evaluator, rig, frozen, at2, &scheduler, &ran);
    CHECK(!pose.valid);
    CHECK(!ran);
}

// The session-pinned route samples exactly what a fresh bind samples: same
// values, same stale marks, same transported results and lines, same
// digest -- at an unrun frame and under a drag alike.
void
TestSessionBindingsMatchFreshBind()
{
    UsdStageRefPtr stage = MakeChainedRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    RigExecChainSampleBindings pinned;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned));
    CHECK(!pinned.chains.empty());

    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/AlongX");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.25);
    for (int arm = 0; arm < 2; ++arm) {
        const std::vector<RigExecValueOverride> overrides =
            arm == 0 ? std::vector<RigExecValueOverride>{}
                     : std::vector<RigExecValueOverride>{drag};
        RigExecFrameInputs fresh, session;
        std::string error;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), overrides,
                                       &fresh, &error));
        CHECK(RigExecSampleFrameInputsWithChainBindings(
            evaluator, UsdTimeCode(3.0), overrides, pinned, &session,
            &error));
        CHECK(fresh.values.size() == session.values.size());
        for (size_t i = 0; i < fresh.values.size(); ++i) {
            CHECK(fresh.values[i].path == session.values[i].path);
            CHECK(fresh.values[i].hasValue == session.values[i].hasValue);
            CHECK(fresh.values[i].viaChain == session.values[i].viaChain);
            CHECK(fresh.values[i].value == session.values[i].value);
        }
        CHECK(fresh.chainResults == session.chainResults);
        CHECK(fresh.chainDiagnostics == session.chainDiagnostics);
        CHECK(RigExecFrozenControlDigest(fresh) ==
              RigExecFrozenControlDigest(session));
    }

    // Stale pins fail the pinned route instead of sampling through them.
    stage->GetAttributeAtPath(
            SdfPath("/Asset/Rig/Movers/TxGain.inputs:defaultWeight"))
        .Set(0.5f);
    CHECK(!RigExecChainSampleBindingsStillCurrent(pinned, evaluator));
    RigExecFrameInputs refused;
    std::string error;
    CHECK(!RigExecSampleFrameInputsWithChainBindings(
        evaluator, UsdTimeCode(3.0), {}, pinned, &refused, &error));
    CHECK(!error.empty());
}

// The burst cache samples elementwise-identical vectors to the plain
// pinned sampler: the same paths in the same order, the same values and
// flags, the same chain results and diagnostics -- plus the route marks
// the cached digest reads. One cache serves a four-frame burst per arm
// (no overrides, a held drag), on a chainless rig (mover and topology
// reads cache through the resolved route) and a chained one (chain bases
// cache through the stage route; refreshed reads stay fresh). Digests
// agree three ways: the frozen oracle, the plain control digest, and the
// cached control digest. A misclassified (varying-as-static) sample would
// serve frame 1's value at frame 3 and fail the elementwise check.
void
TestBurstCacheMatchesPinnedSampling()
{
    for (int rigArm = 0; rigArm < 2; ++rigArm) {
        UsdStageRefPtr stage =
            rigArm == 0 ? MakeTinyRig() : MakeChainedRig();
        if (rigArm == 0) {
            // A varying resolved read: animate the skin mover's default
            // weight, so the resolved memo path serves (and the
            // misclassification check below guards) genuinely differing
            // values. The other mover scalars and the topology stay
            // static and memoize.
            UsdAttribute weight = stage->GetAttributeAtPath(SdfPath(
                "/Asset/Rig/Movers/Skin_0.inputs:defaultWeight"));
            CHECK(weight.IsValid());
            for (int t = 1; t <= 4; ++t) {
                CHECK(weight.Set(0.5f + 0.1f * float(t),
                                 UsdTimeCode(double(t))));
            }
        } else {
            // A varying stage read: animate the mesh points the chain
            // base samples, so a varying-as-static misclassification
            // serves frame 1's base at frame 3 and fails below.
            UsdAttribute points = stage->GetAttributeAtPath(
                SdfPath("/Asset/Geom/Mesh_0.points"));
            CHECK(points.IsValid());
            for (int t = 1; t <= 4; ++t) {
                VtVec3fArray animated(kTinyPointCount);
                for (size_t i = 0; i < kTinyPointCount; ++i) {
                    animated[i] = GfVec3f(
                        float(i) * 0.5f + float(t), float(i) * -0.25f,
                        -float(i) * 0.125f);
                }
                CHECK(points.Set(animated, UsdTimeCode(double(t))));
            }
            // A second, static mesh and chain beside it: the stage memo
            // path still engages (and the routing assertion below still
            // holds) while the first chain's base varies.
            const SdfPath meshPath("/Asset/Geom/Mesh_1");
            const UsdPrim mesh =
                stage->DefinePrim(meshPath, TfToken("Mesh"));
            VtVec3fArray still(kTinyPointCount);
            for (size_t i = 0; i < kTinyPointCount; ++i) {
                still[i] = GfVec3f(float(i) * 0.5f, float(i) * -0.25f,
                                   -float(i) * 0.125f);
            }
            mesh.GetAttribute(TfToken("points")).Set(still);
            const UsdPrim skin = stage->DefinePrim(
                SdfPath("/Asset/Rig/Movers/Skin_1"),
                TfToken("RigExecSkinMover"));
            skin.ApplyAPI(TfToken("RigExecMoverAPI"));
            skin.GetRelationship(TfToken("rigExec:moves"))
                .SetTargets({meshPath.AppendProperty(TfToken("points"))});
            skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
            skin.CreateRelationship(TfToken("rigExec:influences"))
                .SetTargets({SdfPath("/Asset/Rig/AlongX"),
                             SdfPath("/Asset/Rig/AlongY")});
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
        }
        const SdfPath rig("/Asset/Rig");
        RigExecRigEvaluator evaluator(stage, rig);
        CHECK(evaluator.Compile());
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

        RigExecChainSampleBindings pinned;
        CHECK(RigExecBindChainSampleInputs(evaluator, &pinned));

        RigExecValueOverride drag;
        drag.prim = SdfPath("/Asset/Rig/AlongX");
        drag.attribute = TfToken("avars:tx");
        drag.value = VtValue(3.25);
        for (int arm = 0; arm < 2; ++arm) {
            const std::vector<RigExecValueOverride> overrides =
                arm == 0 ? std::vector<RigExecValueOverride>{}
                         : std::vector<RigExecValueOverride>{drag};
            RigExecBurstSampleCache cache;
            std::string error;
            CHECK(RigExecBuildBurstSampleCache(
                *evaluator.GetBakedProgram(), pinned, overrides,
                RigExecFrameCacheEpochDigest(evaluator), &cache, &error));
            CHECK(cache.usable);
            for (int frame = 1; frame <= 4; ++frame) {
                const UsdTimeCode time = UsdTimeCode(double(frame));
                RigExecFrameInputs plain, burst;
                CHECK(RigExecSampleFrameInputsWithChainBindings(
                    evaluator, time, overrides, pinned, &plain, &error));
                CHECK(RigExecSampleFrameInputsWithBurstCache(
                    evaluator, time, overrides, &cache, &burst, &error));
                CHECK(plain.values.size() == burst.values.size());
                for (size_t i = 0; i < plain.values.size(); ++i) {
                    CHECK(plain.values[i].path == burst.values[i].path);
                    CHECK(plain.values[i].hasValue ==
                          burst.values[i].hasValue);
                    CHECK(plain.values[i].viaChain ==
                          burst.values[i].viaChain);
                    CHECK(plain.values[i].value == burst.values[i].value);
                }
                CHECK(plain.chainResults == burst.chainResults);
                CHECK(plain.chainDiagnostics == burst.chainDiagnostics);
                CHECK(RigExecFrozenControlDigest(plain) ==
                      RigExecFrozenControlDigest(burst));
                CHECK(RigExecControlStateDigest(plain, overrides) ==
                      RigExecControlStateDigestWithBurstCache(
                          burst, overrides, &cache));
            }
            // The routing rules, pinned: the chainless rig caches mover
            // and topology reads through the resolved route; the chained
            // rig caches chain bases through the stage route and never
            // memoizes refreshed reads under chains.
            if (rigArm == 0) {
                CHECK(!cache.staticResolved.empty());
            } else {
                CHECK(!cache.staticStage.empty());
                CHECK(cache.staticResolved.empty());
            }
        }
    }
}

// The burst route stays elementwise identical to the plain route on a rig
// with several visiting weight objects (examples/11_VolumeWeights.usda):
// per object, bindings then arrays, in table order on both routes. A
// bindings loop followed by an arrays loop emits the same SET in a
// different ORDER, which the tiny rig (one visiting object) cannot see.
void
TestVolumeWeightsBurstMatchesPlain(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/11_VolumeWeights.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1001.0)).valid);
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();
    CHECK(B.weightObjects.size() >= 2);

    RigExecChainSampleBindings pinned;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned, &error));
    RigExecBurstSampleCache cache;
    std::vector<RigExecValueOverride> noOverrides;
    CHECK(RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, noOverrides,
        RigExecFrameCacheEpochDigest(evaluator), &cache, &error));
    CHECK(cache.usable);
    for (double frame : {1012.0, 1048.0}) {
        const UsdTimeCode time(frame);
        RigExecFrameInputs plain, burst;
        CHECK(RigExecSampleFrameInputs(evaluator, time, noOverrides, &plain,
                                       &error));
        CHECK(RigExecSampleFrameInputsWithBurstCache(
            evaluator, time, noOverrides, &cache, &burst, &error));
        CHECK(plain.values.size() == burst.values.size());
        for (size_t i = 0; i < plain.values.size(); ++i) {
            CHECK(plain.values[i].path == burst.values[i].path);
            CHECK(plain.values[i].hasValue == burst.values[i].hasValue);
            CHECK(plain.values[i].viaChain == burst.values[i].viaChain);
            CHECK(plain.values[i].value == burst.values[i].value);
        }
        CHECK(plain.chainResults == burst.chainResults);
        CHECK(plain.chainDiagnostics == burst.chainDiagnostics);
        CHECK(plain.stageSeeds == burst.stageSeeds);
        CHECK(RigExecFrozenControlDigest(plain) ==
              RigExecFrozenControlDigest(burst));
        CHECK(RigExecControlStateDigest(plain, noOverrides) ==
              RigExecControlStateDigestWithBurstCache(burst, noOverrides,
                                                      &cache));
    }
}

// Poses warmed from burst-cached vectors are bit-identical to live: freeze
// after frame 2, warm frames 3 and 4 through one cache, and diff against
// live -- then again at 3 with a held drag standing, through a cache built
// for the drag.
void
TestBurstCacheWarmsBitIdentical()
{
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    RigExecChainSampleBindings pinned;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned));
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;

    RigExecBurstSampleCache cache;
    CHECK(RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, noOverrides,
        RigExecFrameCacheEpochDigest(evaluator), &cache, &error));
    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputsWithBurstCache(
        evaluator, UsdTimeCode(3.0), noOverrides, &cache, &at3, &error));
    CHECK(!at3.HasChainResolvedInputs());
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckPosesBitIdentical("cached warmed frame 3", live3, warmed3);

    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs at4;
    CHECK(RigExecSampleFrameInputsWithBurstCache(
        evaluator, UsdTimeCode(4.0), noOverrides, &cache, &at4, &error));
    const RigExecRigPose warmed4 =
        RunWarmingJob(&evaluator, rig, frozen, at4, &scheduler, nullptr);
    const RigExecRigPose live4 = evaluator.Evaluate(UsdTimeCode(4.0));
    CHECK(live4.valid);
    CheckPosesBitIdentical("cached warmed frame 4", live4, warmed4);

    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/AlongX");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.25);
    evaluator.SetInteractiveOverrides({drag});
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecBurstSampleCache dragged;
    CHECK(RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, {drag},
        RigExecFrameCacheEpochDigest(evaluator), &dragged, &error));
    RigExecFrameInputs dragged3;
    CHECK(RigExecSampleFrameInputsWithBurstCache(
        evaluator, UsdTimeCode(3.0), {drag}, &dragged, &dragged3, &error));
    const RigExecRigPose warmedDragged =
        RunWarmingJob(&evaluator, rig, frozen, dragged3, &scheduler,
                      nullptr);
    const RigExecRigPose liveDragged = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(liveDragged.valid);
    CheckPosesBitIdentical("cached warmed frame 3 under a drag", liveDragged,
                           warmedDragged);
    evaluator.SetInteractiveOverrides({});
}

// A cache built for another program -- or never built -- fails loud and
// leaves the vector untouched, instead of sampling through prepared state
// that does not describe the call.
void
TestBurstCacheRejectsForeignProgram()
{
    UsdStageRefPtr stageA = MakeTinyRig();
    RigExecRigEvaluator evaluatorA(stageA, SdfPath("/Asset/Rig"));
    CHECK(evaluatorA.Compile());
    evaluatorA.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluatorA.Evaluate(UsdTimeCode(1.0)).valid);
    UsdStageRefPtr stageB = MakeChainedRig();
    RigExecRigEvaluator evaluatorB(stageB, SdfPath("/Asset/Rig"));
    CHECK(evaluatorB.Compile());
    evaluatorB.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluatorB.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecChainSampleBindings pinned;
    CHECK(RigExecBindChainSampleInputs(evaluatorA, &pinned));
    RigExecBurstSampleCache cache;
    std::string error;
    CHECK(RigExecBuildBurstSampleCache(
        *evaluatorA.GetBakedProgram(), pinned, {},
        RigExecFrameCacheEpochDigest(evaluatorA), &cache, &error));
    CHECK(cache.usable);

    RigExecFrameInputs out;
    out.Add(SdfPath("/Marker"), VtValue(1.0), true);
    CHECK(!RigExecSampleFrameInputsWithBurstCache(
        evaluatorB, UsdTimeCode(1.0), {}, &cache, &out, &error));
    CHECK(!error.empty());
    CHECK(out.values.size() == 1);
    CHECK(out.values[0].path == SdfPath("/Marker"));

    RigExecBurstSampleCache unbuilt;
    CHECK(!unbuilt.usable);
    CHECK(!RigExecSampleFrameInputsWithBurstCache(
        evaluatorA, UsdTimeCode(1.0), {}, &unbuilt, &out, &error));
    CHECK(!error.empty());
    CHECK(out.values.size() == 1);
}

// Sampling into no vector declines on both routes and says so.
void
TestSamplerNullOutDeclines()
{
    UsdStageRefPtr stage = MakeTinyRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    std::string error;
    CHECK(!RigExecSampleFrameInputs(evaluator, UsdTimeCode(1.0), {}, nullptr,
                                    &error));
    CHECK(error.find("no input vector") != std::string::npos);

    RigExecChainSampleBindings pinned;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned, &error));
    RigExecBurstSampleCache cache;
    CHECK(RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, {},
        RigExecFrameCacheEpochDigest(evaluator), &cache, &error));
    CHECK(cache.usable);
    CHECK(!RigExecSampleFrameInputsWithBurstCache(
        evaluator, UsdTimeCode(1.0), {}, &cache, nullptr, &error));
    CHECK(error.find("no input vector") != std::string::npos);
}

// Binding into no bindings declines and says so.
void
TestBindIntoNullDeclines()
{
    UsdStageRefPtr stage = MakeChainedRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    std::string error;
    CHECK(!RigExecBindChainSampleInputs(evaluator, nullptr, &error));
    CHECK(error.find("no bindings to bind into") != std::string::npos);
}

// The pinned route trusts nothing: bindings that no longer name the
// evaluator's chains fail the sample, and only a rebind feeds it. An
// empty pin on a chained rig is stale by definition; a fresh bind on the
// same rig samples.
void
TestStaleChainBindingsDeclineSampling()
{
    UsdStageRefPtr stage = MakeChainedRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    RigExecChainSampleBindings empty;
    RigExecFrameInputs inputs;
    std::string error;
    CHECK(!RigExecSampleFrameInputsWithChainBindings(
        evaluator, UsdTimeCode(1.0), {}, empty, &inputs, &error));
    CHECK(error.find("rebind and retry") != std::string::npos);
    CHECK(inputs.values.empty());

    RigExecChainSampleBindings fresh;
    CHECK(RigExecBindChainSampleInputs(evaluator, &fresh, &error));
    CHECK(RigExecSampleFrameInputsWithChainBindings(
        evaluator, UsdTimeCode(1.0), {}, fresh, &inputs, &error));
    CHECK(!inputs.values.empty());
}

// Building into no cache declines and says so.
void
TestBurstBuildIntoNullDeclines()
{
    UsdStageRefPtr stage = MakeTinyRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    RigExecChainSampleBindings pinned;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned, &error));
    CHECK(!RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, {},
        RigExecFrameCacheEpochDigest(evaluator), nullptr, &error));
    CHECK(error.find("no burst cache") != std::string::npos);
}

// A constraint target deleted after the build declines sampling and names
// the target: the epoch settles lazily, so the stale program still stands
// and the seed hook -- which the sampler only forwards -- reports the
// same unresolvable target live would give the generation back for.
void
TestUnresolvableTargetDeclinesSampling(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/aimtest.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    RigExecFrameInputs before;
    std::string error;
    std::vector<RigExecValueOverride> noOverrides;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(25.0), noOverrides,
                                   &before, &error));
    CHECK(!before.stageSeeds.xformBase.empty());

    CHECK(stage->RemovePrim(SdfPath("/World/Geom/Sphere")));
    RigExecStageFrameSeeds seeds;
    CHECK(!evaluator.GetBakedProgram()->SampleStageFrameSeeds(
        UsdTimeCode(25.0), &seeds, &error));
    CHECK(error.find("could not resolve constraint target") !=
          std::string::npos);
    CHECK(error.find("/World/Geom/Sphere") != std::string::npos);
    RigExecFrameInputs after;
    CHECK(!RigExecSampleFrameInputs(evaluator, UsdTimeCode(25.0), noOverrides,
                                    &after, &error));
    CHECK(error.find("could not resolve constraint target") !=
          std::string::npos);
    CHECK(error.find("/World/Geom/Sphere") != std::string::npos);
}

// Overrides select placement, samples, and digest together: sampling
// through a cache built for another list fails loud in both directions.
void
TestBurstCacheRejectsChangedOverrides()
{
    UsdStageRefPtr stage = MakeChainedRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecChainSampleBindings pinned;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned));
    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/AlongX");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.25);
    const uint64_t epoch = RigExecFrameCacheEpochDigest(evaluator);
    std::string error;

    RigExecBurstSampleCache clean;
    CHECK(RigExecBuildBurstSampleCache(*evaluator.GetBakedProgram(), pinned,
                                       {}, epoch, &clean, &error));
    RigExecFrameInputs out;
    CHECK(!RigExecSampleFrameInputsWithBurstCache(
        evaluator, UsdTimeCode(3.0), {drag}, &clean, &out, &error));
    CHECK(!error.empty());
    CHECK(out.values.empty());

    RigExecBurstSampleCache dragged;
    CHECK(RigExecBuildBurstSampleCache(*evaluator.GetBakedProgram(), pinned,
                                       {drag}, epoch, &dragged, &error));
    CHECK(!RigExecSampleFrameInputsWithBurstCache(
        evaluator, UsdTimeCode(3.0), {}, &dragged, &out, &error));
    CHECK(!error.empty());
    CHECK(out.values.empty());
}

// The cached digest falls back to the plain digest -- same answer, full
// cost -- for a vector shaped unlike the recorded order's and for foreign
// overrides, and the fallback leaves the recorded order intact.
void
TestBurstDigestOrderFallback()
{
    UsdStageRefPtr stage = MakeChainedRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    RigExecChainSampleBindings pinned;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned));
    RigExecBurstSampleCache cache;
    std::string error;
    CHECK(RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, {},
        RigExecFrameCacheEpochDigest(evaluator), &cache, &error));
    RigExecFrameInputs at1;
    CHECK(RigExecSampleFrameInputsWithBurstCache(
        evaluator, UsdTimeCode(1.0), {}, &cache, &at1, &error));
    const uint64_t cached1 =
        RigExecControlStateDigestWithBurstCache(at1, {}, &cache);
    CHECK(cached1 == RigExecControlStateDigest(at1, {}));
    CHECK(!cache.sortedOrder.empty());

    RigExecFrameInputs reshaped = at1;
    reshaped.values.pop_back();
    CHECK(RigExecControlStateDigestWithBurstCache(reshaped, {}, &cache) ==
          RigExecControlStateDigest(reshaped, {}));
    CHECK(RigExecControlStateDigestWithBurstCache(at1, {}, &cache) ==
          cached1);

    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/AlongX");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.25);
    CHECK(RigExecControlStateDigestWithBurstCache(at1, {drag}, &cache) ==
          RigExecControlStateDigest(at1, {drag}));
}

// Unplaceable overrides decline the build, with an error and an unusable
// cache -- and the plain sampler declines the same overrides, so neither
// route samples what the other refuses.
void
TestBurstBuildDeclinesUnplaceable()
{
    UsdStageRefPtr stage = MakeChainedRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecChainSampleBindings pinned;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned));
    RigExecValueOverride computation;
    computation.prim = SdfPath("/Asset/Rig/AlongX");
    computation.computation = TfToken("computePointFrame");
    computation.value = VtValue(1.0);
    RigExecBurstSampleCache cache;
    std::string error;
    CHECK(!RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, {computation},
        RigExecFrameCacheEpochDigest(evaluator), &cache, &error));
    CHECK(!cache.usable);
    CHECK(!error.empty());
    RigExecFrameInputs out;
    CHECK(!RigExecSampleFrameInputsWithChainBindings(
        evaluator, UsdTimeCode(3.0), {computation}, pinned, &out, &error));
    CHECK(!error.empty());
}

// A constant edited mid-epoch moves no epoch digest -- the program object
// and epoch stand -- but the pins go stale by value and rebind to current,
// and the rebound pins warm bit-identically.
void
TestStillCurrentDetectsConstantEdit()
{
    UsdStageRefPtr stage = MakeChainedRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecChainSampleBindings bound;
    CHECK(RigExecBindChainSampleInputs(evaluator, &bound));
    CHECK(RigExecChainSampleBindingsStillCurrent(bound, evaluator));
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    const uint64_t epoch = evaluator.GetBindingEpochDigest();

    stage->GetAttributeAtPath(
            SdfPath("/Asset/Rig/Movers/TxGain.inputs:defaultWeight"))
        .Set(0.5f);
    CHECK(evaluator.GetBakedProgram() == program);
    CHECK(evaluator.GetBindingEpochDigest() == epoch);
    CHECK(!RigExecChainSampleBindingsStillCurrent(bound, evaluator));
    CHECK(RigExecBindChainSampleInputs(evaluator, &bound));
    CHECK(RigExecChainSampleBindingsStillCurrent(bound, evaluator));

    std::string error;
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameInputs at2;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0), noOverrides,
                                   &at2, &error));
    CHECK(!at2.HasChainResolvedInputs());
    const RigExecRigPose warmed2 =
        RunWarmingJob(&evaluator, rig, frozen, at2, &scheduler, nullptr);
    const RigExecRigPose live2 = evaluator.Evaluate(UsdTimeCode(2.0));
    CHECK(live2.valid);
    CheckPosesBitIdentical("warmed frame 2 under edited pins", live2,
                           warmed2);
}

// Increment C at the API level: an avar default edit patches the live
// program in place (same program object, same epoch, moved region digest),
// and carrying the region onto a copy of the snapshot warms
// bit-identically -- while the unpatched snapshot provably does not, which
// is what makes the copy load-bearing rather than ceremonial.
void
TestPatchFrozenAvarConstants()
{
    _TinyRigOptions options;
    options.staticTx = true;
    UsdStageRefPtr stage = MakeTinyRigWith(options);
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program != nullptr);
    const uint64_t regionBefore = RigExecFrozenAvarRegionDigest(*program);
    std::string error;
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);

    // The edit: a new tx default, patched in place, not rebuilt.
    stage->GetAttributeAtPath(SdfPath("/Asset/Rig/AlongX.avars:tx"))
        .Set(TinyTx(2.0) + 1.5);
    CHECK(evaluator.GetBakedProgram() == program);
    const RigExecBakedProgram *liveProgram = evaluator.GetBakedProgram();
    CHECK(liveProgram != nullptr);
    CHECK(RigExecFrozenAvarRegionDigest(*liveProgram) != regionBefore);

    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), noOverrides,
                                   &at3, &error));
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);

    // The stale snapshot warms the old constant: mismatches, as it must.
    {
        const RigExecRigPose stale =
            RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
        CHECK(stale.valid);
        RigExecRigPose diff;
        RigExecComparePoses(live3, stale, &diff);
        CHECK(diff.bakedParityMismatches != 0);
    }

    // The patched copy warms bit-identically, and the base is untouched
    // (jobs already holding it run on).
    std::shared_ptr<const RigExecFrozenProgram> patched;
    CHECK(RigExecPatchFrozenAvarConstants(*frozen, *liveProgram, &patched,
                                          &error));
    CHECK(patched != nullptr);
    CHECK(patched.get() != frozen.get());
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, patched, at3, &scheduler, nullptr);
    CheckPosesBitIdentical("warmed frame 3 under patched constants", live3,
                           warmed3);
}

// The runner still declines where it cannot prove bit-identity: no frozen
// program, a refusal flag, a stale chain sample, a mistyped holding, and a
// snapshot/vector mismatch each hand the generation back.
void
TestProductionRunnerDeclinesWithoutProof()
{
    UsdStageRefPtr stage = MakeTinyRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(evaluator, &frozen));
    std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameInputs at2;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0), noOverrides,
                                   &at2));
    RigExecBackgroundScheduler scheduler;

    // No program: decline.
    {
        RigExecFrozenEvalContext context;
        context.epochDigest = evaluator.GetBindingEpochDigest();
        context.generation = scheduler.CurrentGeneration(rig);
        context.slotCount =
            evaluator.GetBakedProgram()->GetProviderCount();
        context.varyingInputCount = at2.values.size();
        context.frozen = nullptr;
        const RigExecRigPose pose = RigExecEvaluateFrozen(
            context, at2, RigExecMakeProductionStepRunner(), &scheduler, rig);
        CHECK(!pose.valid);
    }
    // Refusal flag: decline even with a program.
    {
        RigExecFrozenEvalContext context;
        context.epochDigest = evaluator.GetBindingEpochDigest();
        context.generation = scheduler.CurrentGeneration(rig);
        context.slotCount =
            evaluator.GetBakedProgram()->GetProviderCount();
        context.varyingInputCount = at2.values.size();
        context.flags = kRigExecFrozenBakeRefused;
        context.frozen = frozen.get();
        const RigExecRigPose pose = RigExecEvaluateFrozen(
            context, at2, RigExecMakeProductionStepRunner(), &scheduler, rig);
        CHECK(!pose.valid);
    }
    // A stale chain mark: decline.
    {
        RigExecFrameInputs stale = at2;
        CHECK(!stale.values.empty());
        stale.values[0].viaChain = true;
        CHECK(stale.HasChainResolvedInputs());
        bool ran = true;
        const RigExecRigPose pose =
            RunWarmingJob(&evaluator, rig, frozen, stale, &scheduler, &ran);
        CHECK(!pose.valid);
        CHECK(!ran);
    }
    // A mistyped holding: decline.
    {
        RigExecFrameInputs mistyped = at2;
        CHECK(!mistyped.values.empty());
        mistyped.values[0].value = VtValue(TfToken("not-a-number"));
        mistyped.values[0].hasValue = true;
        const RigExecRigPose pose = RunWarmingJob(&evaluator, rig, frozen,
                                                  mistyped, &scheduler,
                                                  nullptr);
        CHECK(!pose.valid);
    }
}

// ---------------------------------------------------------------------------
// Stream B: the frozen worker entry through an injected serial kernel.
// ---------------------------------------------------------------------------

// A deterministic serial kernel standing in for the baked serial executor:
// reads every sampled double, works the private arena, and publishes one
// array. Bit-sensitive on purpose (division and square roots), so any
// aliasing, truncation, or cross-run leak changes the bytes.
bool
TestKernel(const RigExecFrozenEvalContext &context,
           const RigExecFrameInputs &inputs, RigExecFrozenArena &arena,
           RigExecRigPose *pose)
{
    if (!pose || arena.Size() < 4) {
        return false;
    }
    double acc = double(context.slotCount);
    for (const RigExecSampledInput &sampled : inputs.values) {
        if (sampled.hasValue && sampled.value.IsHolding<double>()) {
            acc += sampled.value.UncheckedGet<double>();
        }
    }
    arena.Data()[0] = acc;
    for (size_t i = 1; i < 4; ++i) {
        arena.Data()[i] = arena.Data()[i - 1] / 1.25 + 0.5;
    }
    VtDoubleArray published(4);
    for (size_t i = 0; i < 4; ++i) {
        published[i] = arena.Data()[i] * arena.Data()[i] + double(i);
        published[i] = published[i] / (published[i] + 1.0);
    }
    pose->movedProperties[SdfPath("/FrozenTest.out")] = VtValue(published);
    pose->solverEvaluations = inputs.values.size();
    pose->valid = true;
    return true;
}

RigExecFrameInputs
TestInputs()
{
    RigExecFrameInputs inputs;
    inputs.time = UsdTimeCode(3.0);
    inputs.Add(SdfPath("/Rig/A.avars:tx"), VtValue(1.5));
    inputs.Add(SdfPath("/Rig/A.avars:ty"), VtValue(-2.25));
    inputs.Add(SdfPath("/Rig/B.avars:tx"), VtValue(), /*hasValue=*/false);
    return inputs;
}

RigExecFrozenEvalContext
TestContext(const RigExecFrameInputs &inputs)
{
    RigExecFrozenEvalContext context;
    context.epochDigest = 0x1234;
    context.generation = 0;
    context.slotCount = 8;
    context.programDigest = 0xabcd;
    context.varyingInputCount = inputs.values.size();
    return context;
}

const VtDoubleArray *
TestOutput(const RigExecRigPose &pose)
{
    const auto found =
        pose.movedProperties.find(SdfPath("/FrozenTest.out"));
    if (found == pose.movedProperties.end() ||
        !found->second.IsHolding<VtDoubleArray>()) {
        return nullptr;
    }
    return &found->second.UncheckedGet<VtDoubleArray>();
}

// The frozen entry is bit-identical across runs and equivalent to the same
// kernel run directly: same bytes, same counts, same time. Mutating the
// input vector after a run moves no published byte (no aliasing).
void
TestFrozenRunIsBitIdentical()
{
    const RigExecFrameInputs inputs = TestInputs();
    const RigExecFrozenEvalContext context = TestContext(inputs);

    const RigExecRigPose first =
        RigExecEvaluateFrozen(context, inputs, TestKernel);
    const RigExecRigPose second =
        RigExecEvaluateFrozen(context, inputs, TestKernel);
    CHECK(first.valid);
    CHECK(second.valid);
    CHECK(first.time == UsdTimeCode(3.0));
    CHECK(first.solverEvaluations == inputs.values.size());
    const VtDoubleArray *a = TestOutput(first);
    const VtDoubleArray *b = TestOutput(second);
    CHECK(a != nullptr);
    CHECK(b != nullptr);
    CHECK(a->size() == b->size());
    CHECK(std::memcmp(a->cdata(), b->cdata(),
                      a->size() * sizeof(double)) == 0);

    // The same kernel outside the frozen entry publishes the same bytes:
    // the machinery (arena, serial scope, checks) preserves bit-identity.
    RigExecFrozenArena directArena(context.slotCount);
    RigExecRigPose direct;
    direct.time = inputs.time;
    CHECK(TestKernel(context, inputs, directArena, &direct));
    const VtDoubleArray *c = TestOutput(direct);
    CHECK(c != nullptr);
    CHECK(c->size() == a->size());
    CHECK(std::memcmp(c->cdata(), a->cdata(),
                      c->size() * sizeof(double)) == 0);

    // No aliasing: the pose owns its bytes, not a view of the inputs.
    RigExecFrameInputs mutated = inputs;
    const RigExecRigPose before =
        RigExecEvaluateFrozen(context, mutated, TestKernel);
    const VtDoubleArray beforeBytes = *TestOutput(before);
    mutated.values[0].value = VtValue(999.0);
    const VtDoubleArray *after = TestOutput(before);
    CHECK(after != nullptr);
    CHECK(after->size() == beforeBytes.size());
    CHECK(std::memcmp(after->cdata(), beforeBytes.cdata(),
                      after->size() * sizeof(double)) == 0);
    // ... while a genuinely different input vector publishes genuinely
    // different bytes: the equality above is isolation, not insensitivity.
    RigExecFrozenEvalContext mutatedContext = TestContext(mutated);
    const RigExecRigPose rerun =
        RigExecEvaluateFrozen(mutatedContext, mutated, TestKernel);
    CHECK(rerun.valid);
    const VtDoubleArray *rerunOut = TestOutput(rerun);
    CHECK(rerunOut != nullptr);
    CHECK(std::memcmp(rerunOut->cdata(), beforeBytes.cdata(),
                      rerunOut->size() * sizeof(double)) != 0);
}

// Every inconsistency declines: a wrong epoch, a truncated vector, a null
// or refusing runner, and a runner that leaves valid down. All decline at
// the requested time, sending the caller down the live path.
void
TestInconsistentRequestsDecline()
{
    const RigExecFrameInputs inputs = TestInputs();

    RigExecFrozenEvalContext wrongEpoch = TestContext(inputs);
    wrongEpoch.epochDigest = 0xdead;
    // The epoch pin is checked by the caller against the standing epoch;
    // here the mismatch is simulated by a runner that refuses it, which
    // must still decline rather than run.
    auto epochChecked = [](const RigExecFrozenEvalContext &context,
                           const RigExecFrameInputs &in,
                           RigExecFrozenArena &arena,
                           RigExecRigPose *pose) {
        if (context.epochDigest != 0x1234) {
            return false;
        }
        return TestKernel(context, in, arena, pose);
    };
    CHECK(!RigExecEvaluateFrozen(wrongEpoch, inputs, epochChecked).valid);

    RigExecFrozenEvalContext truncated = TestContext(inputs);
    truncated.varyingInputCount = inputs.values.size() + 1;
    const RigExecRigPose short_ =
        RigExecEvaluateFrozen(truncated, inputs, TestKernel);
    CHECK(!short_.valid);
    CHECK(short_.time == UsdTimeCode(3.0));

    RigExecFrozenEvalContext context = TestContext(inputs);
    CHECK(!RigExecEvaluateFrozen(context, inputs,
                                 RigExecFrozenStepRunner())
               .valid);
    auto refusing = [](const RigExecFrozenEvalContext &,
                       const RigExecFrameInputs &, RigExecFrozenArena &,
                       RigExecRigPose *) { return false; };
    CHECK(!RigExecEvaluateFrozen(context, inputs, refusing).valid);
    auto silent = [](const RigExecFrozenEvalContext &,
                     const RigExecFrameInputs &, RigExecFrozenArena &,
                     RigExecRigPose *) { return true; };
    CHECK(!RigExecEvaluateFrozen(context, inputs, silent).valid);
}

// The generation fence: stale at start never runs, an edit mid-run drops
// the result before publish, and a fresh token under the new generation
// runs. The mid-run edit is deterministic -- the runner itself bumps -- so
// no wall clock and no sleep.
void
TestGenerationFenceDropsStaleJobs()
{
    const SdfPath rig("/Asset/Rig");
    RigExecBackgroundScheduler scheduler;
    const RigExecFrameInputs inputs = TestInputs();

    RigExecFrozenEvalContext context = TestContext(inputs);
    context.generation = scheduler.CurrentGeneration(rig);
    CHECK(RigExecEvaluateFrozen(context, inputs, TestKernel, &scheduler, rig)
              .valid);

    scheduler.CancelGeneration(rig);
    CHECK(!RigExecEvaluateFrozen(context, inputs, TestKernel, &scheduler,
                                 rig)
               .valid);

    RigExecFrozenEvalContext fresh = TestContext(inputs);
    fresh.generation = scheduler.CurrentGeneration(rig);
    bool runnerRan = false;
    auto bumping = [&](const RigExecFrozenEvalContext &ctx,
                       const RigExecFrameInputs &in,
                       RigExecFrozenArena &arena,
                       RigExecRigPose *pose) {
        runnerRan = true;
        scheduler.CancelGeneration(rig);
        return TestKernel(ctx, in, arena, pose);
    };
    CHECK(!RigExecEvaluateFrozen(fresh, inputs, bumping, &scheduler, rig)
               .valid);
    CHECK(runnerRan);

    RigExecFrozenEvalContext newest = TestContext(inputs);
    newest.generation = scheduler.CurrentGeneration(rig);
    CHECK(RigExecEvaluateFrozen(newest, inputs, TestKernel, &scheduler, rig)
              .valid);
}

// D7: refusal rigs memoize UI-thread results but never create a background
// job. The policy predicate, the sampler, and the worker entry all agree:
// the predicate declines, the sampler reports no program, and a context
// carrying the refusal flag declines even with a willing runner.
void
TestRefusalRigsNeverEnqueue()
{
    CHECK(!RigExecShouldEnqueueBackgroundJob(/*bakeRefused=*/true));
    CHECK(RigExecShouldEnqueueBackgroundJob(/*bakeRefused=*/false));
    CHECK(RigExecShouldMemoizeUiThreadResult(/*bakeRefused=*/true));
    CHECK(RigExecShouldMemoizeUiThreadResult(/*bakeRefused=*/false));

    UsdStageRefPtr stage = MakeTinyRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.GetBakedProgram() == nullptr);

    RigExecFrameInputs inputs;
    std::string error;
    std::vector<RigExecValueOverride> noOverrides;
    CHECK(!RigExecSampleFrameInputs(evaluator, UsdTimeCode(1.0), noOverrides,
                                    &inputs, &error));
    CHECK(!error.empty());

    const RigExecFrameInputs runnable = TestInputs();
    RigExecFrozenEvalContext refused = TestContext(runnable);
    refused.flags |= kRigExecFrozenBakeRefused;
    const RigExecRigPose pose =
        RigExecEvaluateFrozen(refused, runnable, TestKernel);
    CHECK(!pose.valid);
    CHECK(pose.time == UsdTimeCode(3.0));
}

// The serial scope constrains the thread that entered it and no other:
// active inside (including inside the runner), inactive outside, nesting
// correctly, and invisible across threads.
void
TestSerialScopeIsThreadLocal()
{
    CHECK(!RigExecFrozenSerialActive());
    {
        RigExecFrozenSerialScope outer;
        CHECK(RigExecFrozenSerialActive());
        {
            RigExecFrozenSerialScope inner;
            CHECK(RigExecFrozenSerialActive());
        }
        CHECK(RigExecFrozenSerialActive());
    }
    CHECK(!RigExecFrozenSerialActive());

    bool runnerSawSerial = false;
    auto observing = [&](const RigExecFrozenEvalContext &ctx,
                         const RigExecFrameInputs &in,
                         RigExecFrozenArena &arena,
                         RigExecRigPose *pose) {
        runnerSawSerial = RigExecFrozenSerialActive();
        return TestKernel(ctx, in, arena, pose);
    };
    const RigExecFrameInputs inputs = TestInputs();
    CHECK(RigExecEvaluateFrozen(TestContext(inputs), inputs, observing)
              .valid);
    CHECK(runnerSawSerial);
    CHECK(!RigExecFrozenSerialActive());

    RigExecFrozenSerialScope held;
    CHECK(RigExecFrozenSerialActive());
    bool otherThreadSawSerial = true;
    std::thread other([&otherThreadSawSerial]() {
        otherThreadSawSerial = RigExecFrozenSerialActive();
    });
    other.join();
    CHECK(!otherThreadSawSerial);
}

// The arena is one job's private working state: zeroed, distinct per
// instance, movable but never copyable, cleared on demand, and capped
// against a corrupt context.
void
TestArenaIsolation()
{
    static_assert(!std::is_copy_constructible<RigExecFrozenArena>::value,
                  "a shared arena would be shared mutable worker state");
    static_assert(std::is_move_constructible<RigExecFrozenArena>::value,
                  "a job must be able to take its arena with it");

    RigExecFrozenArena a(8), b(8);
    CHECK(a.Size() == 8);
    CHECK(a.Bytes() == 8 * sizeof(double));
    CHECK(a.Data() != b.Data());
    for (size_t i = 0; i < 8; ++i) {
        CHECK(a.Data()[i] == 0.0);
    }
    a.Data()[0] = 1.0;
    CHECK(b.Data()[0] == 0.0);
    a.Clear();
    CHECK(a.Data()[0] == 0.0);

    RigExecFrozenEvalContext context;
    context.slotCount = 16;
    RigExecFrozenArena sized;
    CHECK(sized.ResizeFor(context));
    CHECK(sized.Size() == 16);
    context.slotCount = size_t(1) << 40;
    CHECK(!sized.ResizeFor(context));
    CHECK(sized.Size() == 16);

    RigExecFrozenArena moved(std::move(a));
    CHECK(moved.Size() == 8);
}

// Concurrent frozen runs share the context and the inputs and nothing else:
// each carries its own arena, and every run publishes bytes identical to a
// main-thread reference.
void
TestConcurrentFrozenRunsAgree()
{
    const RigExecFrameInputs inputs = TestInputs();
    const RigExecFrozenEvalContext context = TestContext(inputs);
    const RigExecRigPose reference =
        RigExecEvaluateFrozen(context, inputs, TestKernel);
    CHECK(reference.valid);
    const VtDoubleArray expected = *TestOutput(reference);

    constexpr int kThreads = 4;
    constexpr int kIters = 25;
    std::atomic<int> validCount(0);
    std::atomic<int> mismatchCount(0);
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&]() {
            for (int i = 0; i < kIters; ++i) {
                const RigExecRigPose pose = RigExecEvaluateFrozen(
                    context, inputs, TestKernel);
                if (!pose.valid) {
                    continue;
                }
                validCount.fetch_add(1);
                const VtDoubleArray *out = TestOutput(pose);
                if (!out || out->size() != expected.size() ||
                    std::memcmp(out->cdata(), expected.cdata(),
                                expected.size() * sizeof(double)) != 0) {
                    mismatchCount.fetch_add(1);
                }
            }
        });
    }
    for (std::thread &worker : workers) {
        worker.join();
    }
    CHECK(validCount.load() == kThreads * kIters);
    CHECK(mismatchCount.load() == 0);
}

// The purity audit is data, so its shape is asserted: every row names a
// unit and its note, every verdict class is represented, and the rows the
// plan's Constraints exist for -- the shared memos workers bypass, the live
// state they never reach -- are present with non-pure verdicts.
void
TestPurityAuditNamesEveryUnit()
{
    const std::vector<RigExecPurityFinding> &audit =
        RigExecFrozenPurityAudit();
    CHECK(!audit.empty());
    size_t pure = 0, pinned = 0, liveOnly = 0;
    bool sawCurvenetMemo = false, sawWireMemo = false;
    bool sawStage = false, sawEvaluator = false, sawDynamic = false;
    bool sawSolverKernels = false, sawMoverKernels = false;
    for (const RigExecPurityFinding &finding : audit) {
        CHECK(finding.unit != nullptr);
        CHECK(finding.note != nullptr);
        CHECK(finding.unit[0] != '\0');
        CHECK(finding.note[0] != '\0');
        switch (finding.verdict) {
        case RigExecFrozenPurity::Pure: ++pure; break;
        case RigExecFrozenPurity::EpochPinned: ++pinned; break;
        case RigExecFrozenPurity::LiveOnly: ++liveOnly; break;
        }
        const bool isLive = finding.verdict == RigExecFrozenPurity::LiveOnly;
        if (std::strstr(finding.unit, "curvenet binding LRU")) {
            sawCurvenetMemo = isLive;
        }
        if (std::strstr(finding.unit, "wire-basis memo")) {
            sawWireMemo = isLive;
        }
        if (std::strstr(finding.unit, "UsdStage")) {
            sawStage = isLive;
        }
        if (std::strstr(finding.unit, "RigExecRigEvaluator")) {
            sawEvaluator = isLive;
        }
        if (std::strstr(finding.unit, "dynamic path")) {
            sawDynamic = isLive;
        }
        if (std::strstr(finding.unit, "solverKernels")) {
            sawSolverKernels =
                finding.verdict == RigExecFrozenPurity::Pure;
        }
        if (std::strstr(finding.unit, "moverKernels")) {
            sawMoverKernels =
                finding.verdict == RigExecFrozenPurity::Pure;
        }
    }
    CHECK(pure > 0);
    CHECK(pinned > 0);
    CHECK(liveOnly > 0);
    CHECK(sawCurvenetMemo);
    CHECK(sawWireMemo);
    CHECK(sawStage);
    CHECK(sawEvaluator);
    CHECK(sawDynamic);
    CHECK(sawSolverKernels);
    CHECK(sawMoverKernels);
}

}  // namespace

// The biped: twelve float chain-driven inputs over the foot movers, and
// the rig the hook unlocks. Warmed frames match live with zero parity
// mismatches -- the same bar as the fixture, on a production rig.
void
TestBipedWarmsBitIdentical(const std::string &examplesDir,
                           const std::string &stageFile = "Biped_anim.usda",
                           size_t expectChains = 12)
{
    const std::string stagePath = examplesDir + "/biped/" + stageFile;
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim :
         stage->TraverseAll()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    RigExecChainSampleBindings bound;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &bound, &error));
    std::printf("%s chains bound: %zu\n", stageFile.c_str(),
                bound.chains.size());
    CHECK(bound.chains.size() == expectChains);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("%s freeze refused: %s\n", stageFile.c_str(),
                    error.c_str());
    }
    CHECK(frozen != nullptr);
    if (!frozen) {
        return;
    }
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    const std::string what3 = "warmed " + stageFile + " frame 3";
    const std::string what4 = "warmed " + stageFile + " frame 4";

    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), noOverrides,
                                   &at3, &error));
    CHECK(!at3.HasChainResolvedInputs());
    CHECK(!at3.chainResults.empty());
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckPosesBitIdentical(what3.c_str(), live3, warmed3);

    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs at4;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(4.0), noOverrides,
                                   &at4, &error));
    CHECK(!at4.HasChainResolvedInputs());
    const RigExecRigPose warmed4 =
        RunWarmingJob(&evaluator, rig, frozen, at4, &scheduler, nullptr);
    const RigExecRigPose live4 = evaluator.Evaluate(UsdTimeCode(4.0));
    CHECK(live4.valid);
    CheckPosesBitIdentical(what4.c_str(), live4, warmed4);
}

// The stack (examples/biped/Biped_stack_anim.usda) warms bit-identically:
// the layered full-body rig with its pose-interpolator steps, which no
// flat rig builds, warmed from the same two-frame history.
void
TestStackAnimWarmsBitIdentical(const std::string &examplesDir)
{
    TestBipedWarmsBitIdentical(examplesDir, "Biped_stack_anim.usda",
                               /*expectChains=*/250);
}

// The blend face (examples/04_BlendShapeFace.usda) warms bit-identically:
// two animated dense channels over a bound weight object, warmed at three
// frames that span the weight spline -- a full-target hit (1024), an
// in-between lerp (1040), and the implicit-zero endpoint (1048). Each
// warmed frame diffs against live from the same history, and the warmed
// frames must differ from each other, or the weights never moved and the
// test proved nothing.
void
TestBlendFaceWarmsBitIdentical(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/04_BlendShapeFace.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    evaluator.SetPublishWeightFields(true);
    CHECK(evaluator.Evaluate(UsdTimeCode(1001.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(1016.0)).valid);

    RigExecChainSampleBindings bound;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &bound, &error));
    CHECK(bound.chains.empty());

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("blend face freeze refused: %s\n", error.c_str());
    }
    CHECK(frozen != nullptr);
    if (!frozen) {
        return;
    }
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    const SdfPath faceTarget("/FaceAsset/Geom/FaceCard.points");

    const auto facePoints = [&](const RigExecRigPose &pose) {
        const auto found = pose.movedProperties.find(faceTarget);
        CHECK(found != pose.movedProperties.end());
        if (found == pose.movedProperties.end()) {
            return VtVec3fArray();
        }
        CHECK(found->second.IsHolding<VtVec3fArray>());
        return found->second.UncheckedGet<VtVec3fArray>();
    };

    VtVec3fArray previousPoints;
    for (const double frame : {1024.0, 1040.0, 1048.0}) {
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &inputs, &error));
        CHECK(!inputs.HasChainResolvedInputs());
        const RigExecRigPose warmed = RunWarmingJob(
            &evaluator, rig, frozen, inputs, &scheduler, nullptr);
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid);
        CheckPosesBitIdentical(
            TfStringPrintf("warmed blend face frame %g", frame).c_str(), live,
            warmed);
        if (!previousPoints.empty()) {
            CHECK(facePoints(warmed) != previousPoints);
        }
        previousPoints = facePoints(warmed);
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        CHECK(frozen != nullptr);
        if (!frozen) {
            return;
        }
    }
}

// Warms the given frames of an example rig and diffs each against live
// from the same history: run the history live, freeze, then per frame
// sample, warm, compare, and re-freeze so the snapshot tracks live. The
// warmed target points must differ across consecutive frames, or the rig
// never moved and the test proved nothing.
static void
CheckExampleWarmsBitIdentical(const std::string &stagePath,
                              const std::vector<double> &history,
                              const std::vector<double> &frames,
                              const SdfPath &target,
                              bool expectMotion = true)
{
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    for (double frame : history) {
        CHECK(evaluator.Evaluate(UsdTimeCode(frame)).valid);
    }
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("freeze refused for %s: %s\n", stagePath.c_str(),
                    error.c_str());
    }
    CHECK(frozen != nullptr);
    if (!frozen) {
        return;
    }
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    VtVec3fArray previousPoints;
    for (double frame : frames) {
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &inputs, &error));
        CHECK(!inputs.HasChainResolvedInputs());
        const RigExecRigPose warmed = RunWarmingJob(
            &evaluator, rig, frozen, inputs, &scheduler, nullptr);
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid);
        CheckPosesBitIdentical(
            TfStringPrintf("warmed %s frame %g", stagePath.c_str(), frame)
                .c_str(),
            live, warmed);
        const auto found = warmed.movedProperties.find(target);
        CHECK(found != warmed.movedProperties.end());
        if (found != warmed.movedProperties.end()) {
            CHECK(found->second.IsHolding<VtVec3fArray>());
            const VtVec3fArray points =
                found->second.UncheckedGet<VtVec3fArray>();
            CHECK(!points.empty());
            if (expectMotion && !previousPoints.empty()) {
                CHECK(points != previousPoints);
            }
            previousPoints = points;
        }
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        CHECK(frozen != nullptr);
        if (!frozen) {
            return;
        }
    }
}

// The lattice stack (examples/06_LatticeBulge.usda) warms bit-identically:
// a Lattice cage bulge through a Smooth pass into a VolumeCorrect hold,
// with the cage keyed at 1001/1024/1048.
void
TestLatticeStackWarmsBitIdentical(const std::string &examplesDir)
{
    CheckExampleWarmsBitIdentical(
        examplesDir + "/06_LatticeBulge.usda", {1001.0},
        {1012.0, 1024.0, 1036.0, 1048.0},
        SdfPath("/LatticeAsset/Geom/Slab.points"));
}

// The drape (examples/07_SurfaceDrape.usda) warms bit-identically: two
// SurfaceProject movers over a ground mesh keyed at 1001/1024/1048.
void
TestSurfaceDrapeWarmsBitIdentical(const std::string &examplesDir)
{
    CheckExampleWarmsBitIdentical(
        examplesDir + "/07_SurfaceDrape.usda", {1001.0},
        {1012.0, 1024.0, 1036.0, 1048.0},
        SdfPath("/DrapeAsset/Geom/ProjectSticker.points"));
}

// The ribbon spine (examples/05_TwistRibbonSpine.usda) warms
// bit-identically: a Ribbon revision and an EmitGuidePoints revision over
// one solver's driver frames, plus a matrix fin.
void
TestRibbonSpineWarmsBitIdentical(const std::string &examplesDir)
{
    CheckExampleWarmsBitIdentical(
        examplesDir + "/05_TwistRibbonSpine.usda", {1001.0},
        {1012.0, 1024.0, 1036.0, 1048.0},
        SdfPath("/SpineAsset/Geom/SpineStrip.points"));
}

// The arm (examples/ArmRig.usda) warms bit-identically: VolumeCorrect,
// Ribbon and EmitGuidePoints revisions stacked with matrix skins and a
// blendshape in-between over one mesh. A static rig, so consecutive
// frames agree by construction and only parity is asserted.
void
TestArmRigWarmsBitIdentical(const std::string &examplesDir)
{
    CheckExampleWarmsBitIdentical(examplesDir + "/ArmRig.usda", {1001.0},
                                  {1024.0, 1048.0},
                                  SdfPath("/ArmAsset/Geom/ArmBody.points"),
                                  /*expectMotion=*/false);
}

// The curvenet profile (examples/12_CurvenetProfile.usda) warms
// bit-identically: a Curvenet revision over a net chain's posed result,
// with the bind transported from the sample.
void
TestCurvenetProfileWarmsBitIdentical(const std::string &examplesDir)
{
    CheckExampleWarmsBitIdentical(
        examplesDir + "/12_CurvenetProfile.usda", {1001.0},
        {1012.0, 1024.0, 1036.0, 1048.0},
        SdfPath("/CurvenetAsset/Geom/Tube.points"));
}

// The read phases (examples/13_ReadPhases.usda) warm bit-identically: a
// lattice reading its cage at `final` through the run's snapshot store,
// over two matrix movers that pose the cage first.
void
TestReadPhasesWarmBitIdentical(const std::string &examplesDir)
{
    CheckExampleWarmsBitIdentical(
        examplesDir + "/13_ReadPhases.usda", {1001.0},
        {1012.0, 1024.0, 1036.0, 1048.0},
        SdfPath("/ReadPhaseAsset/Geom/Slab.points"));
}

// A constraint stage warms bit-identically: freeze after the history,
// sample each probe frame through both routes, and diff the warmed pose
// against live. The program-shape assertions are the vacuity guard -- the
// seeds are only exercised when the program carries the slots -- and the
// motion assertions keep the comparison honest: consecutive frames must
// digest apart and pose apart, and where the target animates the seeds
// themselves must move, which is what proves the worker patches moving
// seeds rather than replaying freeze-time bases.
void
CheckConstraintStageWarmsBitIdentical(
    const std::string &stagePath, const std::vector<double> &history,
    const std::vector<double> &frames, size_t expectXform,
    size_t expectNative, size_t expectDelta, bool expectSeedMotion)
{
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    for (double frame : history) {
        CHECK(evaluator.Evaluate(UsdTimeCode(frame)).valid);
    }
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();
    CHECK(B.xformSlots.size() == expectXform);
    CHECK(B.nativeSources.size() == expectNative);
    CHECK(B.deltaBasePaths.size() == expectDelta);

    RigExecChainSampleBindings pinned;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &pinned, &error));
    RigExecBurstSampleCache cache;
    std::vector<RigExecValueOverride> noOverrides;
    CHECK(RigExecBuildBurstSampleCache(
        *evaluator.GetBakedProgram(), pinned, noOverrides,
        RigExecFrameCacheEpochDigest(evaluator), &cache, &error));
    CHECK(cache.usable);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("freeze refused for %s: %s\n", stagePath.c_str(),
                    error.c_str());
    }
    CHECK(frozen != nullptr);
    if (!frozen) {
        return;
    }
    RigExecBackgroundScheduler scheduler;
    RigExecStageFrameSeeds previousSeeds;
    RigExecRigPose previousLive;
    bool havePreviousLive = false;
    uint64_t previousDigest = 0;
    for (double frame : frames) {
        RigExecFrameInputs plain, burst;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &plain, &error));
        CHECK(RigExecSampleFrameInputsWithBurstCache(
            evaluator, UsdTimeCode(frame), noOverrides, &cache, &burst,
            &error));
        CHECK(plain.stageSeeds.xformBase.size() == expectXform);
        CHECK(plain.stageSeeds.nativeOk.size() == expectNative);
        CHECK(plain.stageSeeds.deltaOk.size() == expectDelta);
        CHECK(plain.stageSeeds == burst.stageSeeds);
        CHECK(RigExecFrozenControlDigest(plain) ==
              RigExecFrozenControlDigest(burst));
        CHECK(RigExecControlStateDigestWithBurstCache(burst, {}, &cache) ==
              RigExecControlStateDigest(burst, {}));
        const RigExecRigPose warmed = RunWarmingJob(
            &evaluator, rig, frozen, plain, &scheduler, nullptr);
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid);
        CheckPosesBitIdentical(
            TfStringPrintf("warmed %s frame %g", stagePath.c_str(), frame)
                .c_str(),
            live, warmed);
        const uint64_t digest = RigExecControlStateDigest(plain);
        if (havePreviousLive) {
            CHECK(digest != previousDigest);
            RigExecRigPose motion;
            RigExecComparePoses(previousLive, live, &motion);
            CHECK(motion.bakedParityMismatches != 0);
            if (expectSeedMotion) {
                CHECK(previousSeeds != plain.stageSeeds);
            }
        }
        previousSeeds = plain.stageSeeds;
        previousLive = live;
        havePreviousLive = true;
        previousDigest = digest;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        CHECK(frozen != nullptr);
        if (!frozen) {
            return;
        }
    }
}

// The turret (examples/10_AimXformTurret.usda) warms bit-identically: one
// xform slot seeding the barrel's revised transform. The target is static,
// so the seeds do not move -- shape plus bit-identity only.
void
TestAimXformTurretWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/10_AimXformTurret.usda", {1001.0},
        {1012.0, 1024.0, 1036.0, 1048.0}, /*expectXform=*/1,
        /*expectNative=*/0, /*expectDelta=*/0, /*expectSeedMotion=*/false);
}

// The aim diagnostic (examples/aimtest.usda) warms bit-identically: one
// xform slot plus one native source, both moving with the pivot.
void
TestAimtestWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/aimtest.usda", {1.0}, {25.0, 50.0, 75.0, 100.0},
        /*expectXform=*/1, /*expectNative=*/1, /*expectDelta=*/0,
        /*expectSeedMotion=*/true);
}

// The points-domain aim (examples/aimtest_points.usda) warms
// bit-identically: xform slot, native source, and the geometry-delta base
// the aim measures against, with the delta handed to the points revision
// through the shared fold.
void
TestAimtestPointsWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/aimtest_points.usda", {1.0}, {25.0, 50.0, 75.0, 100.0},
        /*expectXform=*/1, /*expectNative=*/1, /*expectDelta=*/1,
        /*expectSeedMotion=*/true);
}

// The rotation diagnostic (examples/rotateConstraint.usda) warms
// bit-identically: one xform slot plus one native source.
void
TestRotateConstraintWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/rotateConstraint.usda", {1.0}, {25.0, 50.0, 75.0, 100.0},
        /*expectXform=*/1, /*expectNative=*/1, /*expectDelta=*/0,
        /*expectSeedMotion=*/true);
}

// The flattened aim (examples/rigexec_flat.usda) warms bit-identically:
// one xform slot plus one native source.
void
TestRigexecFlatWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/rigexec_flat.usda", {0.0}, {25.0, 50.0, 75.0, 100.0},
        /*expectXform=*/1, /*expectNative=*/1, /*expectDelta=*/0,
        /*expectSeedMotion=*/true);
}

// The parent/rotate/aim stack (examples/par_rot_aim.usd) warms
// bit-identically: one xform slot plus three native sources.
void
TestParRotAimWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/par_rot_aim.usd", {0.0}, {25.0, 50.0, 75.0, 100.0},
        /*expectXform=*/1, /*expectNative=*/3, /*expectDelta=*/0,
        /*expectSeedMotion=*/true);
}

// The reordered stack (examples/par_rot_aim_redorder.usd) warms
// bit-identically: one xform slot plus three native sources.
void
TestParRotAimReorderWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/par_rot_aim_redorder.usd", {0.0}, {25.0, 50.0, 75.0, 100.0},
        /*expectXform=*/1, /*expectNative=*/3, /*expectDelta=*/0,
        /*expectSeedMotion=*/true);
}

// The rotation/parent combo (examples/rot_par_combo.usd) warms
// bit-identically: one xform slot plus two native sources.
void
TestRotParComboWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/rot_par_combo.usd", {0.0}, {25.0, 50.0, 75.0, 100.0},
        /*expectXform=*/1, /*expectNative=*/2, /*expectDelta=*/0,
        /*expectSeedMotion=*/true);
}

// The flattened aim/parent combo (examples/aim_par_combo_flattened.usd)
// warms bit-identically: one xform slot plus two native sources.
void
TestAimParComboFlattenedWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/aim_par_combo_flattened.usd", {0.0},
        {25.0, 50.0, 75.0, 100.0}, /*expectXform=*/1, /*expectNative=*/2,
        /*expectDelta=*/0, /*expectSeedMotion=*/true);
}

// The volume-constrained sweep
// (examples/14_VolumeConstrainedSweep.usda) warms bit-identically: xform
// slot, native source, and the geometry-delta base, with the volume
// weight object bound on the geometry-domain constraint -- which never
// resolves through the oracle (weight stays 1.0; the revision packet
// scales per point), so it freezes where a pose-domain weight object
// refuses.
void
TestVolumeConstrainedSweepWarmsBitIdentical(const std::string &examplesDir)
{
    CheckConstraintStageWarmsBitIdentical(
        examplesDir + "/14_VolumeConstrainedSweep.usda", {1001.0},
        {1012.0, 1024.0, 1036.0, 1048.0}, /*expectXform=*/1,
        /*expectNative=*/1, /*expectDelta=*/1, /*expectSeedMotion=*/true);
}

// Freezing into no snapshot refuses and says so.
void
TestFreezeIntoNullSnapshotRefuses()
{
    UsdStageRefPtr stage = MakeTinyRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    std::string error;
    CHECK(!RigExecFreezeProgram(evaluator, nullptr, &error));
    CHECK(error.find("no snapshot") != std::string::npos);
}

// CPU parity mode runs the dynamic path, which no snapshot can reproduce,
// so freeze refuses there and only there: the same rig freezes with the
// flag down.
void
TestCpuParityModeRefusesFreeze()
{
    UsdStageRefPtr stage = MakeTinyRig();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    evaluator.cpuParityMode = true;
    frozen.reset();
    CHECK(!RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen == nullptr);
    CHECK(error.find("CPU parity mode") != std::string::npos);
}

// A pose-domain constraint with a bound weight object refuses freeze and
// names it: the step resolves one envelope element through the live
// oracle per frame. The static weight is bakeable, so the refusal is the
// freeze gate's, not the bake's -- the message proves which.
void
TestPoseConstraintWeightObjectRefusesFreeze()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    auto builder = RigExecRigBuilder::Create(stage, SdfPath("/Asset/Rig"));
    auto control = builder.AddControl("Ctl");
    auto joint = builder.AddJoint("Jnt");
    auto weight =
        builder.AddStaticWeight("W", joint.GetPath(), {}, {}, 0.5f);
    auto chain = builder.NewMoverChain("Pose");
    auto aim = chain.AddAimConstraint("Aim", joint.GetPath());
    aim.SetSources({control.GetPath()});
    aim.SetWeightObject(weight.GetPath());
    RigExecRigEvaluator evaluator(stage, builder.GetRootPath());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    std::vector<std::string> reasons;
    CHECK(evaluator.IsBakeable(&reasons));
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(!RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen == nullptr);
    if (error.find("weight object") == std::string::npos) {
        ++failures;
        std::printf("FAIL weight-object refusal names it: %s\n",
                    error.c_str());
    }
}

// CurvenetAdjuster is the one revision op no shipped baked example
// authors, so it keeps a named refusal: freeze must decline a rig
// carrying one, and the message must name the op.
void
TestCurvenetAdjusterRefusesFreeze()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    auto builder = RigExecRigBuilder::Create(stage, SdfPath("/Asset/Rig"));
    const std::vector<GfVec3f> rest{
        {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
    auto net = builder.AddCurvenet("Net", rest);
    net.AddSpline(0, 1, 2, 3);
    auto knot = builder.AddCurvenetAdjustment("Knot", net.GetPath(), 0);
    knot.SetAvarTranslation(1, 0, 0);
    const SdfPath target = net.GetPath().AppendProperty(TfToken("points"));
    auto chain = builder.NewMoverChain("Shape", target);
    chain.AddCurvenetAdjusterMover("Adjust", {knot.GetPath()});
    RigExecRigEvaluator evaluator(stage, builder.GetRootPath());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(!RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen == nullptr);
    if (error.find("curvenetAdjuster") == std::string::npos) {
        ++failures;
        std::printf("FAIL adjuster refusal names the op: %s\n",
                    error.c_str());
    }
}

// The Stream 0 9-mesh rig (reports/frame-cache-measurements.md §1): the
// MakeMultiMeshRig construction -- 9 skinned meshes over two shared
// controls -- animated (time samples at 1..40, so consecutive frames
// genuinely re-evaluate instead of cone-skipping a static rig) with valid
// envelopes throughout, so the cost compared is the skin kernels'.
static constexpr size_t k9MeshCount = 9;
static constexpr size_t k9MeshPointCount =
    RigExecGeometryParallelThreshold + 37;

UsdStageRefPtr
MakeAnimated9MeshRig()
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
    for (int t = 1; t <= 40; ++t) {
        tx.Set(10.0 + 0.1 * double(t), UsdTimeCode(double(t)));
        ty.Set(20.0 - 0.05 * double(t), UsdTimeCode(double(t)));
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    VtIntArray indices(k9MeshPointCount * 2);
    for (size_t i = 0; i < k9MeshPointCount; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
    }
    for (size_t mesh = 0; mesh < k9MeshCount; ++mesh) {
        const SdfPath meshPath(
            TfStringPrintf("/Asset/Geom/Mesh_%zu", mesh));
        const UsdPrim prim = stage->DefinePrim(meshPath, TfToken("Mesh"));
        VtVec3fArray points(k9MeshPointCount);
        for (size_t i = 0; i < k9MeshPointCount; ++i) {
            points[i] = GfVec3f(float(i) * 0.5f + float(mesh),
                                float(i) * -0.25f,
                                float(mesh) * 3.0f - float(i) * 0.125f);
        }
        prim.GetAttribute(TfToken("points")).Set(points);

        const UsdPrim skin = stage->DefinePrim(
            SdfPath(TfStringPrintf("/Asset/Rig/Movers/Skin_%zu", mesh)),
            TfToken("RigExecSkinMover"));
        skin.ApplyAPI(TfToken("RigExecMoverAPI"));
        skin.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({meshPath.AppendProperty(TfToken("points"))});
        skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
        skin.CreateRelationship(TfToken("rigExec:influences"))
            .SetTargets({alongX.GetPath(), alongY.GetPath()});
        skin.CreateAttribute(TfToken("rigExec:elementSize"),
                             SdfValueTypeNames->Int).Set(2);
        skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                             SdfValueTypeNames->IntArray).Set(indices);
        VtFloatArray weights(k9MeshPointCount * 2);
        const float xWeight = 0.1f + 0.05f * float(mesh);
        const float yWeight = 0.9f - 0.05f * float(mesh);
        for (size_t i = 0; i < k9MeshPointCount; ++i) {
            weights[i * 2] = xWeight;
            weights[i * 2 + 1] = yWeight;
        }
        skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray).Set(weights);
    }
    return stage;
}

// Every mesh's moved points are present and full: bit-identity over dropped
// or empty chains would pass while proving nothing.
static void
CheckAll9MeshesPublished(const char *what, const RigExecRigPose &pose)
{
    for (size_t mesh = 0; mesh < k9MeshCount; ++mesh) {
        const SdfPath target(TfStringPrintf("/Asset/Geom/Mesh_%zu.points",
                                            mesh));
        const auto found = pose.movedProperties.find(target);
        CHECK(found != pose.movedProperties.end());
        if (found == pose.movedProperties.end()) {
            continue;
        }
        CHECK(found->second.IsHolding<VtVec3fArray>());
        if (found->second.IsHolding<VtVec3fArray>()) {
            CHECK(found->second.UncheckedGet<VtVec3fArray>().size() ==
                  k9MeshPointCount);
        }
    }
    if (pose.movedProperties.size() < k9MeshCount) {
        std::printf("FAIL %s: %zu moved properties, want at least %zu\n",
                    what, pose.movedProperties.size(), k9MeshCount);
        ++failures;
    }
}

// The 9mesh rig warms bit-identically: freeze after frame 2, warm the unrun
// frames 3 and 4 (plus frame 3 under a held drag), and diff each against
// live from the same history -- the biped bar, on the multi-chain rig.
void
Test9MeshWarmsBitIdentical()
{
    UsdStageRefPtr stage = MakeAnimated9MeshRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    // The rig bakes: a refusal here would make a freeze decline expected
    // and this test vacuous.
    std::vector<std::string> reasons;
    if (!evaluator.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL: the 9mesh rig is not bakeable\n");
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("9mesh freeze refused: %s\n", error.c_str());
    }
    CHECK(frozen != nullptr);
    if (!frozen) {
        return;
    }
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;

    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), noOverrides,
                                   &at3, &error));
    CHECK(!at3.HasChainResolvedInputs());
    std::printf("9mesh sampled inputs: %zu values, %zu revision packets\n",
                at3.values.size(), at3.revisionPackets.size());
    CHECK(at3.revisionPackets.size() == k9MeshCount);
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckAll9MeshesPublished("warmed 9mesh frame 3", live3);
    CheckPosesBitIdentical("warmed 9mesh frame 3", live3, warmed3);

    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs at4;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(4.0), noOverrides,
                                   &at4, &error));
    CHECK(!at4.HasChainResolvedInputs());
    // Sensitivity: the animated controls move every frame, so frames 3 and
    // 4 must digest apart -- identical digests would compare one frame with
    // itself.
    CHECK(RigExecFrozenControlDigest(at3) != RigExecFrozenControlDigest(at4));
    const RigExecRigPose warmed4 =
        RunWarmingJob(&evaluator, rig, frozen, at4, &scheduler, nullptr);
    const RigExecRigPose live4 = evaluator.Evaluate(UsdTimeCode(4.0));
    CHECK(live4.valid);
    CheckAll9MeshesPublished("warmed 9mesh frame 4", live4);
    CheckPosesBitIdentical("warmed 9mesh frame 4", live4, warmed4);

    // A held drag: the override places across all nine chains, and the
    // warmed pose matches live under the same drag.
    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/AlongX");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.25);
    evaluator.SetInteractiveOverrides({drag});
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs dragged3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {drag},
                                   &dragged3, &error));
    CHECK(!dragged3.HasChainResolvedInputs());
    const RigExecRigPose warmedDragged = RunWarmingJob(
        &evaluator, rig, frozen, dragged3, &scheduler, nullptr);
    const RigExecRigPose liveDragged = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(liveDragged.valid);
    CheckAll9MeshesPublished("warmed 9mesh frame 3 under a drag",
                             liveDragged);
    CheckPosesBitIdentical("warmed 9mesh frame 3 under a drag", liveDragged,
                           warmedDragged);
    evaluator.SetInteractiveOverrides({});
}

// Bit-identity at sweep distance: freeze once after frame 2, warm a far
// frame from that freeze, and diff against live from the same history.
// Production sweeps to +-40, and the frozen run branches its cone-skip
// decisions from freeze-time history, so distance is the thing under test.
// Each probe frame gets a fresh evaluator so the live comparison always
// runs from the frozen history -- the existing tests' arrangement, which
// re-freezes for the same reason.
void
Check9MeshWarmsBitIdenticalAt(double frame)
{
    UsdStageRefPtr stage = MakeAnimated9MeshRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    std::vector<std::string> reasons;
    if (!evaluator.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL: the 9mesh rig is not bakeable\n");
        return;
    }
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    if (!frozen) {
        return;
    }
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;

    RigExecFrameInputs probed;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame), noOverrides,
                                   &probed, &error));
    CHECK(!probed.HasChainResolvedInputs());
    CHECK(probed.revisionPackets.size() == k9MeshCount);
    const RigExecRigPose warmed = RunWarmingJob(
        &evaluator, rig, frozen, probed, &scheduler, nullptr);
    const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
    CHECK(live.valid);
    const std::string what =
        "warmed 9mesh frame " + std::to_string(int(frame));
    CheckAll9MeshesPublished(what.c_str(), live);
    CheckPosesBitIdentical(what.c_str(), live, warmed);
}

void
Test9MeshWarmsBitIdenticalAtSweepDistance()
{
    Check9MeshWarmsBitIdenticalAt(39.0);
    Check9MeshWarmsBitIdenticalAt(40.0);
}

// The biped at sweep distance: freeze after frame 2, warm 32 frames past
// the authored range's end, as a production sweep from a late playhead
// would -- and diff against live from the same history.
void
TestBipedWarmsBitIdenticalAtSweepDistance(
    const std::string &examplesDir,
    const std::string &stageFile = "Biped_anim.usda")
{
    const std::string stagePath = examplesDir + "/biped/" + stageFile;
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    if (!frozen) {
        return;
    }
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;

    const double sweepFrame = stage->GetEndTimeCode() + 32.0;
    RigExecFrameInputs atSweep;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(sweepFrame),
                                   noOverrides, &atSweep, &error));
    CHECK(!atSweep.HasChainResolvedInputs());
    const RigExecRigPose warmedSweep = RunWarmingJob(
        &evaluator, rig, frozen, atSweep, &scheduler, nullptr);
    const RigExecRigPose liveSweep =
        evaluator.Evaluate(UsdTimeCode(sweepFrame));
    CHECK(liveSweep.valid);
    const std::string what = "warmed " + stageFile + " frame " +
                           std::to_string(int(sweepFrame));
    CheckPosesBitIdentical(what.c_str(), liveSweep, warmedSweep);
}

// The stack at sweep distance: one frame past the end of its range,
// warmed against live from the same history.
void
TestStackAnimWarmsBitIdenticalAtSweepDistance(const std::string &examplesDir)
{
    TestBipedWarmsBitIdenticalAtSweepDistance(examplesDir,
                                              "Biped_stack_anim.usda");
}

int
main(int argc, char **argv)
{
    TestValuesAreFoundByPath();
    TestAValuelessSourceIsStillRecorded();
    TestTheFirstSampleAtAPathWins();
    TestClearEmptiesTheVectorAndResetsTime();
    TestTheContextPinsAnEpochAndSizesAnArena();
    TestTheStubEvaluatorAnswersInvalidAtTheRequestedTime();
    TestSamplerMatchesLiveReads();
    TestDigestMovesWithControls();
    TestProductionRunnerIsBitIdenticalToLive();
    TestChainedRigWarmsBitIdentical();
    TestChainHookDeclinesWeightObjects();
    TestSessionBindingsMatchFreshBind();
    TestBurstCacheMatchesPinnedSampling();
    TestBurstCacheWarmsBitIdentical();
    TestBurstCacheRejectsForeignProgram();
    TestBurstCacheRejectsChangedOverrides();
    TestBurstDigestOrderFallback();
    TestBurstBuildDeclinesUnplaceable();
    TestStillCurrentDetectsConstantEdit();
    TestPatchFrozenAvarConstants();
    if (argc > 1) {
        TestBipedWarmsBitIdentical(argv[1]);
        TestBipedWarmsBitIdenticalAtSweepDistance(argv[1]);
        TestStackAnimWarmsBitIdentical(argv[1]);
        TestStackAnimWarmsBitIdenticalAtSweepDistance(argv[1]);
        TestBlendFaceWarmsBitIdentical(argv[1]);
        TestLatticeStackWarmsBitIdentical(argv[1]);
        TestSurfaceDrapeWarmsBitIdentical(argv[1]);
        TestRibbonSpineWarmsBitIdentical(argv[1]);
        TestArmRigWarmsBitIdentical(argv[1]);
        TestCurvenetProfileWarmsBitIdentical(argv[1]);
        TestReadPhasesWarmBitIdentical(argv[1]);
        TestAimXformTurretWarmsBitIdentical(argv[1]);
        TestAimtestWarmsBitIdentical(argv[1]);
        TestAimtestPointsWarmsBitIdentical(argv[1]);
        TestRotateConstraintWarmsBitIdentical(argv[1]);
        TestRigexecFlatWarmsBitIdentical(argv[1]);
        TestParRotAimWarmsBitIdentical(argv[1]);
        TestParRotAimReorderWarmsBitIdentical(argv[1]);
        TestRotParComboWarmsBitIdentical(argv[1]);
        TestAimParComboFlattenedWarmsBitIdentical(argv[1]);
        TestVolumeConstrainedSweepWarmsBitIdentical(argv[1]);
        TestVolumeWeightsBurstMatchesPlain(argv[1]);
        TestUnresolvableTargetDeclinesSampling(argv[1]);
    } else {
        std::printf("skipping the biped (no examples directory given)\n");
    }
    Test9MeshWarmsBitIdentical();
    Test9MeshWarmsBitIdenticalAtSweepDistance();
    TestProductionRunnerDeclinesWithoutProof();
    TestFrozenRunIsBitIdentical();
    TestInconsistentRequestsDecline();
    TestGenerationFenceDropsStaleJobs();
    TestRefusalRigsNeverEnqueue();
    TestSerialScopeIsThreadLocal();
    TestArenaIsolation();
    TestConcurrentFrozenRunsAgree();
    TestPurityAuditNamesEveryUnit();
    TestCurvenetAdjusterRefusesFreeze();
    TestFreezeIntoNullSnapshotRefuses();
    TestCpuParityModeRefusesFreeze();
    TestPoseConstraintWeightObjectRefusesFreeze();
    TestSamplerNullOutDeclines();
    TestBindIntoNullDeclines();
    TestStaleChainBindingsDeclineSampling();
    TestBurstBuildIntoNullDeclines();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecFrozenContext: all tests passed\n");
    return 0;
}
