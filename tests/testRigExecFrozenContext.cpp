// RigExec frozen contexts, Stream B: the sampled input vector, the
// epoch-pinned context, the UI-thread sampler, and the fail-closed frozen
// worker entry.
// FrameInputs is the type the UI thread samples at enqueue time and the
// worker reads from, so its container semantics are asserted here: values
// are found by path, a missing path is null rather than a default, an
// explicitly valueless source is still a recorded source, and Clear empties
// the vector and resets the time.
// The sampler (RigExecSampleFrameInputs) walks the baked program's varying
// bindings through the same route the frame path reads, so its fidelity is
// asserted against two independent oracles: the authored time samples the
// test set itself, and direct stage reads at the sampled time. The digest
// (RigExecFrozenControlDigest) must move with every control edit and stand
// still across re-samples, because the frame cache keys on it.
// The frozen worker entry (RigExecEvaluateFrozen with a runner) is asserted
// through an injected serial kernel: bit-identical across runs, equivalent
// to the same kernel run directly, refusing every inconsistent request
// (wrong epoch, truncated vector, stale generation, refusal flag), and safe
// under concurrent runs. What the runner CAN be is bounded too: the
// context's "no live pointers" rule is held structurally -- the type is
// trivially copyable, which a member holding a stage, an evaluator, or a
// USD handle cannot be -- and the purity audit names every unit the frozen
// path was checked against.
#include "rigExec/frozenContext.h"
#include "rigExec/frozenContextInternal.h"
#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/frameCache.h"
#include "rigExec/generation.h"
#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecRigging/rigBuilder.h"

#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editTarget.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
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
    inputs.stageSeeds.xformBase.push_back(GfMatrix4d(1.0));
    inputs.stageSeeds.nativeOk.push_back(1);
    inputs.stageSeeds.deltaBase.push_back(GfMatrix4d(1.0));
    inputs.Clear();
    CHECK(inputs.time == UsdTimeCode::Default());
    CHECK(inputs.values.empty());
    CHECK(inputs.Find(SdfPath("/Rig/Ctl.avars:tx")) == nullptr);
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

// Stream B: the sampler and its oracles.

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

// Chained rigs. The tiny rig plus one float math mover revising the tx avar
// by a time-varying factor -- the biped's foot chains in miniature (compare
// examples/09_PropertyMathMovers.usda): the chain's output at the sampled
// time exists nowhere until the worker's head tier runs, so a job that read
// the standing state would warm frame 3 with frame 2's chain values.

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

// The production frozen executor: a warming job evaluates bit-identically
// to live evaluation of the same inputs.

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

// The job was accepted, not served by a live fallback: no sample came from
// standing chain outputs, and the worker returned a pose.
static void
CheckJobAccepted(const char *what, const RigExecFrameInputs &inputs,
                 const RigExecRigPose &warmed)
{
    bool viaChain = false;
    for (const RigExecSampledInput &sample : inputs.values) {
        viaChain = viaChain || sample.viaChain;
    }
    if (viaChain || !warmed.valid) {
        std::printf("FAIL %s: the job was declined\n", what);
    }
    CHECK(!viaChain);
    CHECK(warmed.valid);
}

// Whether every reader of \p a reads what it reads in \p b: each moved
// property (a float or double by its bits), control frame and joint frame,
// and the diagnostics. Unlike CheckPosesBitIdentical it leaves the work
// counters out, which differ between a held drag and its release.
static void
CheckSameReadings(const char *what, const RigExecRigPose &a,
                  const RigExecRigPose &b)
{
    const auto sameValue = [](const VtValue &x, const VtValue &y) {
        if (x.IsHolding<float>() && y.IsHolding<float>()) {
            const float p = x.UncheckedGet<float>();
            const float q = y.UncheckedGet<float>();
            return std::memcmp(&p, &q, sizeof(p)) == 0;
        }
        if (x.IsHolding<double>() && y.IsHolding<double>()) {
            const double p = x.UncheckedGet<double>();
            const double q = y.UncheckedGet<double>();
            return std::memcmp(&p, &q, sizeof(p)) == 0;
        }
        return x == y;
    };
    std::string why;
    if (!a.valid || !b.valid) {
        why = "an invalid pose";
    } else if (a.movedProperties.size() != b.movedProperties.size()) {
        why = "moved property count";
    } else if (a.controlFrames != b.controlFrames) {
        why = "control frames";
    } else if (a.jointFramesFinal != b.jointFramesFinal) {
        why = "joint frames";
    } else {
        // Less the mover graph's work line, a work counter too.
        const auto lines = [](const RigExecRigPose &pose) {
            std::vector<std::string> out;
            for (const std::string &line : pose.diagnostics) {
                if (line.rfind("mover graph: ", 0) != 0) {
                    out.push_back(line);
                }
            }
            return out;
        };
        if (lines(a) != lines(b)) {
            why = "diagnostics";
        }
    }
    for (auto i = a.movedProperties.begin(), j = b.movedProperties.begin();
         why.empty() && i != a.movedProperties.end(); ++i, ++j) {
        if (i->first != j->first || !sameValue(i->second, j->second)) {
            why = "moved property " + i->first.GetString();
        }
    }
    if (!why.empty()) {
        std::printf("FAIL %s: the readings differ (%s)\n", what, why.c_str());
    }
    CHECK(why.empty());
}

// Space switches nested under switched controls: S (in Other's space) sits
// under P, and P is switched into C's space with C under S, so S resolves
// first and reads P BEFORE P's switch. The baked program binds that read to
// P's pre-switch version; a warming job runs the same steps from the frozen
// snapshot, so it must match live baked and the dynamic walk bit for bit --
// at an unrun frame, and under a drag of an index.
UsdStageRefPtr
MakeNestedSpaceSwitchRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&stage](const char *path, double x) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        GfMatrix4d rest(1.0);
        rest.SetTranslateOnly(GfVec3d(x, 100.0, 0.0));
        prim.GetAttribute(TfToken("rest:space")).Set(rest);
        return prim;
    };
    const UsdPrim other = control("/Asset/Rig/Other", 50.0);
    const UsdPrim p = control("/Asset/Rig/P", 0.0);
    const UsdPrim s = control("/Asset/Rig/P/S", 10.0);
    const UsdPrim c = control("/Asset/Rig/P/S/C", 15.0);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const double pActive[4] = {0.0, 1.0, 0.5, 0.0};
    const double sActive[4] = {0.5, 0.0, 1.0, 0.5};
    const auto spaces = [&stage](const char *name, const UsdPrim &target,
                                 const UsdPrim &source,
                                 const double (&keys)[4]) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers").AppendChild(TfToken(name)),
            TfToken("RigExecSpaceSwitch"));
        prim.CreateRelationship(TfToken("rigExec:target"))
            .SetTargets({target.GetPath()});
        // The rig root is not a provider: it is spelled "world".
        prim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({source.GetPath(), SdfPath("/Asset/Rig")});
        UsdAttribute active = prim.CreateAttribute(
            TfToken("inputs:activeSpace"), SdfValueTypeNames->Double);
        for (int t = 0; t < 4; ++t) {
            active.Set(keys[t], UsdTimeCode(double(t + 1)));
        }
    };
    spaces("sSpaces", s, other, sActive);
    spaces("pSpaces", p, c, pActive);
    UsdAttribute tx = other.GetAttribute(TfToken("avars:tx"));
    UsdAttribute rz = p.GetAttribute(TfToken("avars:rz"));
    for (int t = 1; t <= 4; ++t) {
        tx.Set(10.0 * double(t), UsdTimeCode(double(t)));
        rz.Set(7.5 * double(t) - 12.0, UsdTimeCode(double(t)));
    }
    return stage;
}

void
TestNestedSpaceSwitchesWarmBitIdentical()
{
    UsdStageRefPtr stage = MakeNestedSpaceSwitchRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecRigEvaluator walk(stage, rig);
    CHECK(walk.Compile(&errors));
    walk.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
    CHECK(evaluator.GetBakedProgram() != nullptr);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("nested space switches freeze refused: %s\n",
                    error.c_str());
        return;
    }
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), noOverrides,
                                   &at3, &error));
    bool ran = false;
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, &ran);
    CHECK(ran);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckPosesBitIdentical("nested switches warmed frame 3", live3, warmed3);
    CheckPosesBitIdentical("nested switches walk frame 3",
                           walk.Evaluate(UsdTimeCode(3.0)), warmed3);

    // A held drag of P's index: the warmed pose, live baked and the walk
    // agree under it.
    RigExecValueOverride drag;
    drag.prim = SdfPath("/Asset/Rig/Movers/pSpaces");
    drag.attribute = TfToken("inputs:activeSpace");
    drag.value = VtValue(0.25);
    evaluator.SetInteractiveOverrides({drag});
    walk.SetInteractiveOverrides({drag});
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs dragged;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(4.0), {drag},
                                   &dragged, &error));
    const RigExecRigPose warmedDragged =
        RunWarmingJob(&evaluator, rig, frozen, dragged, &scheduler, &ran);
    CHECK(ran);
    const RigExecRigPose liveDragged = evaluator.Evaluate(UsdTimeCode(4.0));
    CHECK(liveDragged.valid);
    CheckPosesBitIdentical("nested switches warmed under a drag",
                           liveDragged, warmedDragged);
    CheckPosesBitIdentical("nested switches walk under a drag",
                           walk.Evaluate(UsdTimeCode(4.0)), warmedDragged);
    evaluator.SetInteractiveOverrides({});
    walk.SetInteractiveOverrides({});
}

// A job at live's last time under a different drag value than live's last
// run: the clone carries live's time and both override flags, so nothing in
// them says the value moved, and the worker re-samples every leaf anyway.
// The warmed pose is the job's drag's, as the walk and live baked pose it.
void
TestAHeldFrameJobServesItsOwnDrag()
{
    UsdStageRefPtr stage = MakeNestedSpaceSwitchRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecRigEvaluator walk(stage, rig);
    CHECK(walk.Compile(&errors));
    walk.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    const auto dragTo = [](double value) {
        RigExecValueOverride drag;
        drag.prim = SdfPath("/Asset/Rig/Movers/pSpaces");
        drag.attribute = TfToken("inputs:activeSpace");
        drag.value = VtValue(value);
        return drag;
    };
    const UsdTimeCode held(3.0);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
    evaluator.SetInteractiveOverrides({dragTo(0.25)});
    CHECK(evaluator.Evaluate(held).valid);
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("held-frame drag freeze refused: %s\n", error.c_str());
        return;
    }
    const RigExecValueOverride other = dragTo(0.75);
    RigExecFrameInputs inputs;
    CHECK(RigExecSampleFrameInputs(evaluator, held, {other}, &inputs,
                                   &error));
    RigExecBackgroundScheduler scheduler;
    bool ran = false;
    const RigExecRigPose warmed =
        RunWarmingJob(&evaluator, rig, frozen, inputs, &scheduler, &ran);
    CHECK(ran);
    walk.SetInteractiveOverrides({other});
    CheckPosesBitIdentical("held-frame job against the walk",
                           walk.Evaluate(held), warmed);
    evaluator.SetInteractiveOverrides({other});
    const RigExecRigPose live = evaluator.Evaluate(held);
    CheckPosesBitIdentical("held-frame job against live", live, warmed);
    // Not vacuous: the two drag values pose the rig differently.
    walk.SetInteractiveOverrides({dragTo(0.25)});
    const RigExecRigPose first = walk.Evaluate(held);
    CHECK(first.controlFrames != live.controlFrames);
    evaluator.SetInteractiveOverrides({});
    walk.SetInteractiveOverrides({});
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

// The chained rig warms bit-identically: the worker recomputes the chain
// for the job's time from its head-leaf samples (never the evaluator's
// last-run outputs) and resolves the tx binding through its reader walk,
// no sample is marked stale, and the warmed poses match live with zero
// parity mismatches -- including under a drag on the chain's target (the
// chain's base) and on the mover's own factor input (a pre-chain read).
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

    // A chained rig freezes: the worker runs the chains for the job's time
    // and resolves the bindings that read them.
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;

    // Frame 3, which live has not run: the job's head leaves, not frame 2's
    // state.
    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), noOverrides,
                                   &at3, &error));
    CHECK(!at3.HasChainResolvedInputs());
    const SdfPath txPath("/Asset/Rig/AlongX.avars:tx");
    double authored = 0.0;
    CHECK(stage->GetAttributeAtPath(txPath).Get(&authored,
                                               UsdTimeCode(3.0)));
    // The tx binding reads the chain's result, so it samples nothing: the
    // worker resolves its reader walk. The chain's own inputs travel as
    // head leaves under their own keys: the target's authored value, which
    // moves from frame 2, and the gain's factor at frame 3.
    {
        RigExecFrameInputs at2;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0),
                                       noOverrides, &at2, &error));
        CHECK(at3.Find(txPath) == nullptr);
        const SdfPath ownKey("/Asset/Rig/AlongX.frozenChainOwn:avars:tx");
        const VtValue *own2 = at2.Find(ownKey);
        CHECK(own2 != nullptr && own2->IsHolding<double>() &&
              own2->Get<double>() != authored);
        const VtValue *own = at3.Find(
            SdfPath("/Asset/Rig/AlongX.frozenChainOwn:avars:tx"));
        CHECK(own != nullptr && own->IsHolding<double>() &&
              own->Get<double>() == authored);
        const VtValue *factor = at3.Find(SdfPath(
            "/Asset/Rig/Movers/TxGain.frozenChainHop:inputs:value"));
        CHECK(factor != nullptr && factor->IsHolding<float>() &&
              factor->Get<float>() == 1.5f);
    }
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckPosesBitIdentical("warmed chained frame 3", live3, warmed3);
    CheckJobAccepted("warmed chained frame 3", at3, warmed3);
    // The worker's head tier published the revised value live published:
    // the chain's output, not its base.
    {
        const auto published = live3.movedProperties.find(txPath);
        const auto computed = warmed3.movedProperties.find(txPath);
        CHECK(published != live3.movedProperties.end());
        CHECK(computed != warmed3.movedProperties.end());
        CHECK(published != live3.movedProperties.end() &&
              computed != warmed3.movedProperties.end() &&
              computed->second == published->second &&
              computed->second != VtValue(authored));
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
    CheckJobAccepted("warmed chained frame 4", at4, warmed4);

    // A drag on the chain's target is the chain's base; a drag on the
    // mover's factor input is read pre-chain. Both warm bit-identical.
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
        CheckJobAccepted("chained drag", dragged, warmedDragged);
        evaluator.SetInteractiveOverrides({});
    }
}

// The read phase of a connection, warmed: the chained rig plus readout
// movers that add what they read of the revised tx avar to float channels
// of their own -- undeclared (the base: tx as authored), `final` (no record:
// tx's published value answers it), `base` on a hop, `final` through that
// hop (a record of every revision) and a checkpoint at the last revision.
// The worker publishes each record on its reader as live does, the warmed
// poses match live bit for bit, and so they do under each drag rule: on a
// reader's own input, on the hop, and on the target, whose drag is the
// chain's base -- the base readers read it, the checkpoint and the final
// readers the chain revised from it -- and reads, warmed and live, what
// the drag authored at that frame reads.
UsdStageRefPtr
MakePhasedChainedRig()
{
    UsdStageRefPtr stage = MakeChainedRig();
    const UsdPrim channels = stage->DefinePrim(
        SdfPath("/Asset/Rig/Channels/Readouts"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Readouts"), TfToken("Scope"));
    const auto readout = [&](const char *name, const SdfPath &source,
                             const char *phase) {
        const UsdAttribute target = channels.CreateAttribute(
            TfToken(std::string("rigExec:") + name), SdfValueTypeNames->Float);
        target.Set(0.0f);
        const UsdPrim mover = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/Readouts/") + name),
            TfToken("RigExecFloatMathMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.CreateAttribute(TfToken("rigExec:operation"),
                              SdfValueTypeNames->Token)
            .Set(TfToken("add"));
        mover.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({target.GetPath()});
        const UsdAttribute value = mover.CreateAttribute(
            TfToken("inputs:value"), SdfValueTypeNames->Float);
        value.SetConnections({source});
        if (phase) {
            value.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));
        }
    };
    const SdfPath tx("/Asset/Rig/AlongX.avars:tx");
    readout("base", tx, nullptr);
    readout("final", tx, "final");
    readout("hop", tx, "base");
    readout("finalViaHop",
            SdfPath("/Asset/Rig/Movers/Readouts/hop.inputs:value"), "final");
    readout("lastCheckpoint", tx, "/Asset/Rig/Movers/TxGain");
    return stage;
}

static float
_ReadoutOf(const RigExecRigPose &pose, const char *name)
{
    const auto it = pose.movedProperties.find(SdfPath(
        std::string("/Asset/Rig/Channels/Readouts.rigExec:") + name));
    return it != pose.movedProperties.end() && it->second.IsHolding<float>()
               ? it->second.UncheckedGet<float>()
               : -1.0f;
}

void
TestPhasedReadsWarmBitIdentical()
{
    UsdStageRefPtr stage = MakePhasedChainedRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
    // base, hop, finalViaHop and lastCheckpoint; `final` passes no
    // recorded hop.
    CHECK(evaluator.GetPhasedConnections().size() == 4);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    RigExecBackgroundScheduler scheduler;

    const SdfPath txPath("/Asset/Rig/AlongX.avars:tx");
    double authored = 0.0;
    CHECK(stage->GetAttributeAtPath(txPath).Get(&authored, UsdTimeCode(3.0)));
    RigExecFrameInputs at3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {}, &at3,
                                   &error));
    CHECK(!at3.HasChainResolvedInputs());
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckPosesBitIdentical("phased reads, frame 3", live3, warmed3);
    CheckJobAccepted("phased reads, frame 3", at3, warmed3);
    // The worker's head tier publishes each record on its reader, as live
    // does: the base reader reads tx as authored.
    CHECK(_ReadoutOf(warmed3, "base") == float(authored));
    const auto revised = live3.movedProperties.find(txPath);
    CHECK(revised != live3.movedProperties.end() &&
          revised->second.IsHolding<double>());
    const float final = revised != live3.movedProperties.end()
                            ? float(revised->second.Get<double>())
                            : 0.0f;
    CHECK(final != float(authored));
    CHECK(_ReadoutOf(live3, "base") == float(authored));
    CHECK(_ReadoutOf(live3, "hop") == float(authored));
    CHECK(_ReadoutOf(live3, "final") == final);
    CHECK(_ReadoutOf(live3, "finalViaHop") == final);
    CHECK(_ReadoutOf(live3, "lastCheckpoint") == final);

    struct Drag {
        const char *what;
        RigExecValueOverride override;
        float base, hop, final, finalViaHop, lastCheckpoint;
    };
    const auto at = [](const char *prim, const char *attribute,
                       const VtValue &value) {
        return RigExecValueOverride{SdfPath(prim), TfToken(),
                                    TfToken(attribute), value};
    };
    // TxGain multiplies by 1.5 at frame 3: 3.25 revises to 4.875 exactly.
    const float draggedFinal = 4.875f;
    const Drag drags[] = {
        {"drag on a reader's input",
         at("/Asset/Rig/Movers/Readouts/base", "inputs:value",
            VtValue(0.5f)),
         0.5f, float(authored), final, final, final},
        {"drag on the hop",
         at("/Asset/Rig/Movers/Readouts/hop", "inputs:value",
            VtValue(0.25f)),
         float(authored), 0.25f, final, 0.25f, final},
        {"drag on the target",
         at("/Asset/Rig/AlongX", "avars:tx", VtValue(3.25)),
         3.25f, 3.25f, draggedFinal, draggedFinal, draggedFinal},
    };
    RigExecRigPose liveOnTarget;
    for (const Drag &drag : drags) {
        evaluator.SetInteractiveOverrides({drag.override});
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        RigExecFrameInputs dragged;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0),
                                       {drag.override}, &dragged, &error));
        const RigExecRigPose warmed = RunWarmingJob(
            &evaluator, rig, frozen, dragged, &scheduler, nullptr);
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(3.0));
        CHECK(live.valid);
        CheckPosesBitIdentical(drag.what, live, warmed);
        CheckJobAccepted(drag.what, dragged, warmed);
        const bool read = _ReadoutOf(live, "base") == drag.base &&
                          _ReadoutOf(live, "hop") == drag.hop &&
                          _ReadoutOf(live, "final") == drag.final &&
                          _ReadoutOf(live, "finalViaHop") ==
                              drag.finalViaHop &&
                          _ReadoutOf(live, "lastCheckpoint") ==
                              drag.lastCheckpoint;
        if (!read) {
            std::printf("FAIL %s: read %.9g %.9g %.9g %.9g %.9g\n",
                        drag.what, double(_ReadoutOf(live, "base")),
                        double(_ReadoutOf(live, "hop")),
                        double(_ReadoutOf(live, "final")),
                        double(_ReadoutOf(live, "finalViaHop")),
                        double(_ReadoutOf(live, "lastCheckpoint")));
        }
        CHECK(read);
        if (drag.override.prim == SdfPath("/Asset/Rig/AlongX")) {
            liveOnTarget = live;
        }
        evaluator.SetInteractiveOverrides({});
    }

    // Released: 3.25 authored at frame 3 reads, live and warmed (frozen
    // after frame 2, as above), what the drag on the target read.
    stage->GetAttributeAtPath(txPath).Set(3.25, UsdTimeCode(3.0));
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs released3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {},
                                   &released3, &error));
    const RigExecRigPose warmedReleased = RunWarmingJob(
        &evaluator, rig, frozen, released3, &scheduler, nullptr);
    const RigExecRigPose liveReleased = evaluator.Evaluate(UsdTimeCode(3.0));
    CheckPosesBitIdentical("released target drag", liveReleased,
                           warmedReleased);
    CheckJobAccepted("released target drag", released3, warmedReleased);
    CheckSameReadings("released target drag against the drag", liveOnTarget,
                      liveReleased);
    stage->GetAttributeAtPath(txPath).Set(authored, UsdTimeCode(3.0));

    // A rewire that keeps every record's consumer, type and position but
    // lengthens a walk -- finalViaHop now reads through a relay outside
    // the rig, which reads nothing at a phase itself, before the hop --
    // leaves bindings that would stand aside by the old hops: no longer
    // current, so the caller rebinds.
    RigExecChainSampleBindings bound;
    CHECK(RigExecBindChainSampleInputs(evaluator, &bound, &error));
    CHECK(RigExecChainSampleBindingsStillCurrent(bound, evaluator));
    const SdfPath finalViaHop(
        "/Asset/Rig/Movers/Readouts/finalViaHop.inputs:value");
    const UsdAttribute relay =
        stage->DefinePrim(SdfPath("/Asset/Relay"), TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:relay"),
                             SdfValueTypeNames->Float);
    relay.SetConnections(
        {SdfPath("/Asset/Rig/Movers/Readouts/hop.inputs:value")});
    stage->GetAttributeAtPath(finalViaHop).SetConnections({relay.GetPath()});
    CHECK(evaluator.Evaluate(UsdTimeCode(3.0)).valid);
    size_t hops = 0;
    for (const RigExecPhasedConnection &connection :
         evaluator.GetPhasedConnections()) {
        if (connection.consumer == finalViaHop) {
            hops = connection.hops.size();
        }
    }
    CHECK(hops == 3);
    CHECK(evaluator.GetPhasedConnections().size() == 4);
    CHECK(!RigExecChainSampleBindingsStillCurrent(bound, evaluator));
}

struct _BlinkRigOptions {
    // The channel authored at 0.2; false leaves it without a value.
    bool authored = true;
    // The lid control reading it.
    bool lid = true;
    // A Gain doubling the channel before the clamp, read at its checkpoint
    // by a third readout chain and, with the lid, by the lid's tz.
    bool gain = false;
};

// The blink on the tiny rig: a float channel authored at 0.2 and clamped to
// [0, 1], read undeclared and at `final` by readout chains and by a lid
// control's avars (tx at the base, ty at `final`).
UsdStageRefPtr
MakeBlinkRig(const _BlinkRigOptions &options = _BlinkRigOptions())
{
    UsdStageRefPtr stage = MakeTinyRig();
    const UsdAttribute blink =
        stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Face"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:blink"),
                             SdfValueTypeNames->Float);
    if (options.authored) {
        blink.Set(0.2f);
    }
    const auto mover = [&](const std::string &path, const char *operation,
                           const SdfPath &target) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath("/Asset/Rig/Movers/" + path),
                              TfToken("RigExecFloatMathMover"));
        prim.ApplyAPI(TfToken("RigExecMoverAPI"));
        prim.CreateAttribute(TfToken("rigExec:operation"),
                             SdfValueTypeNames->Token)
            .Set(TfToken(operation));
        prim.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
        return prim;
    };
    const UsdPrim clamp = mover("ClampBlink", "clamp", blink.GetPath());
    clamp.CreateAttribute(TfToken("inputs:min"), SdfValueTypeNames->Float)
        .Set(0.0f);
    clamp.CreateAttribute(TfToken("inputs:max"), SdfValueTypeNames->Float)
        .Set(1.0f);
    // A nested mover revises before its parent: Gain, then the clamp.
    const char *const gainPath = "/Asset/Rig/Movers/ClampBlink/Gain";
    if (options.gain) {
        mover("ClampBlink/Gain", "multiply", blink.GetPath())
            .CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
            .Set(2.0f);
    }
    const UsdPrim channels = stage->DefinePrim(
        SdfPath("/Asset/Rig/Channels/Readouts"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Readouts"), TfToken("Scope"));
    std::vector<std::pair<const char *, const char *>> readouts = {
        {"blinkBase", nullptr}, {"blinkFinal", "final"}};
    if (options.gain) {
        readouts.emplace_back("blinkCheckpoint", gainPath);
    }
    for (const auto &[name, phase] : readouts) {
        const UsdAttribute target = channels.CreateAttribute(
            TfToken(std::string("rigExec:") + name), SdfValueTypeNames->Float);
        target.Set(0.0f);
        const UsdAttribute value =
            mover(std::string("Readouts/") + name, "add", target.GetPath())
                .CreateAttribute(TfToken("inputs:value"),
                                 SdfValueTypeNames->Float);
        value.SetConnections({blink.GetPath()});
        if (phase) {
            value.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));
        }
    }
    if (options.lid) {
        const UsdPrim lid = stage->DefinePrim(SdfPath("/Asset/Rig/Lid"),
                                              TfToken("RigExecControl"));
        lid.GetAttribute(TfToken("avars:tx"))
            .SetConnections({blink.GetPath()});
        const UsdAttribute ty = lid.GetAttribute(TfToken("avars:ty"));
        ty.SetConnections({blink.GetPath()});
        ty.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
        if (options.gain) {
            const UsdAttribute tz = lid.GetAttribute(TfToken("avars:tz"));
            tz.SetConnections({blink.GetPath()});
            tz.SetMetadata(TfToken("rigExecReadPhase"), std::string(gainPath));
        }
    }
    return stage;
}

// The blink dragged to 1.4 warms bit-identically to live, with the base
// readers on 1.4 and the final readers on 1.0; authored at 1.4 and
// released, it warms bit-identically again and reads, live, exactly what
// the drag read.
void
TestBlinkDragWarmsAsReleased()
{
    UsdStageRefPtr stage = MakeBlinkRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.GetSkippedOperations().empty());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
    RigExecBackgroundScheduler scheduler;
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;

    const SdfPath blink("/Asset/Rig/Channels/Face.rigExec:blink");
    const RigExecValueOverride drag{blink.GetPrimPath(), TfToken(),
                                    blink.GetNameToken(), VtValue(1.4f)};
    evaluator.SetInteractiveOverrides({drag});
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs dragged3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {drag},
                                   &dragged3, &error));
    CHECK(!dragged3.HasChainResolvedInputs());
    const RigExecRigPose warmed = RunWarmingJob(&evaluator, rig, frozen,
                                                dragged3, &scheduler, nullptr);
    const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(3.0));
    CheckPosesBitIdentical("blink drag", live, warmed);
    CheckJobAccepted("blink drag", dragged3, warmed);
    CHECK(_ReadoutOf(live, "blinkBase") == 1.4f);
    CHECK(_ReadoutOf(live, "blinkFinal") == 1.0f);
    const auto lid = live.controlFrames.find(SdfPath("/Asset/Rig/Lid"));
    CHECK(lid != live.controlFrames.end() &&
          lid->second.points[0] == GfVec3d(double(1.4f), 1.0, 0.0));
    evaluator.SetInteractiveOverrides({});

    // Frozen after frame 2 again, so the warmed and the live frame 3 share
    // a history.
    stage->GetAttributeAtPath(blink).Set(1.4f);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs released3;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {},
                                   &released3, &error));
    const RigExecRigPose warmedReleased = RunWarmingJob(
        &evaluator, rig, frozen, released3, &scheduler, nullptr);
    const RigExecRigPose liveReleased = evaluator.Evaluate(UsdTimeCode(3.0));
    CheckPosesBitIdentical("blink released", liveReleased, warmedReleased);
    CheckJobAccepted("blink released", released3, warmedReleased);
    CheckSameReadings("blink released against the drag", live, liveReleased);
}

// The frozen sampler's own copy of the base rule, at its edges, each warmed
// bit-identically to live at frame 3 (frozen after frame 2): with nothing
// authored a drag of 0.5 is the base, so the chain runs and its result
// travels with the vector; a NaN drag skips the chain with the line an
// authored NaN prints, and nothing travels; with a Gain before the clamp,
// a drag of 0.75 reads 0.75 at the base, 1.5 at Gain's checkpoint and 1.0
// at `final`; and a double drag of 1.4 reads what the float 1.4 reads.
void
TestBlinkDragEdgesWarmBitIdentical()
{
    const SdfPath rig("/Asset/Rig");
    const SdfPath blink("/Asset/Rig/Channels/Face.rigExec:blink");
    struct Warmed {
        RigExecRigPose live;
        RigExecRigPose warmed;
        RigExecFrameInputs inputs;
    };
    const auto warm = [&](const char *what, const _BlinkRigOptions &options,
                          const VtValue &value) {
        Warmed out;
        UsdStageRefPtr stage = MakeBlinkRig(options);
        RigExecRigEvaluator evaluator(stage, rig);
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.GetSkippedOperations().empty());
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
        RigExecBackgroundScheduler scheduler;
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        std::string error;
        const RigExecValueOverride drag{blink.GetPrimPath(), TfToken(),
                                        blink.GetNameToken(), value};
        evaluator.SetInteractiveOverrides({drag});
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {drag},
                                       &out.inputs, &error));
        CHECK(!out.inputs.HasChainResolvedInputs());
        out.warmed = RunWarmingJob(&evaluator, rig, frozen, out.inputs,
                                   &scheduler, nullptr);
        out.live = evaluator.Evaluate(UsdTimeCode(3.0));
        CHECK(out.live.valid);
        CheckPosesBitIdentical(what, out.live, out.warmed);
        CheckJobAccepted(what, out.inputs, out.warmed);
        return out;
    };
    const auto lidAt = [](const RigExecRigPose &pose) {
        const auto lid = pose.controlFrames.find(SdfPath("/Asset/Rig/Lid"));
        return lid == pose.controlFrames.end()
                   ? GfVec3d(-1.0)
                   : GfVec3d(lid->second.points[0]);
    };

    _BlinkRigOptions unauthored;
    unauthored.authored = false;
    const Warmed drawn = warm("blink drag, nothing authored", unauthored,
                              VtValue(0.5f));
    CHECK(_ReadoutOf(drawn.live, "blinkBase") == 0.5f);
    CHECK(_ReadoutOf(drawn.live, "blinkFinal") == 0.5f);
    CHECK(lidAt(drawn.live) == GfVec3d(0.5, 0.5, 0.0));
    const auto computed = drawn.warmed.movedProperties.find(blink);
    CHECK(computed != drawn.warmed.movedProperties.end() &&
          computed->second == VtValue(0.5f));

    _BlinkRigOptions noLid;
    noLid.lid = false;
    const Warmed skipped = warm("blink NaN drag", noLid,
                                VtValue(std::nanf("")));
    CHECK(skipped.live.movedProperties.count(blink) == 0);
    CHECK(skipped.warmed.movedProperties.count(blink) == 0);
    for (const RigExecRigPose *pose : {&skipped.live, &skipped.warmed}) {
        CHECK(std::count(pose->diagnostics.begin(), pose->diagnostics.end(),
                         "property chain " + blink.GetString() +
                             ": authored base is not finite; chain "
                             "skipped") == 1);
    }

    _BlinkRigOptions gained;
    gained.gain = true;
    const Warmed checkpoint = warm("blink drag, Gain then clamp", gained,
                                   VtValue(0.75f));
    CHECK(_ReadoutOf(checkpoint.live, "blinkBase") == 0.75f);
    CHECK(_ReadoutOf(checkpoint.live, "blinkCheckpoint") == 1.5f);
    CHECK(_ReadoutOf(checkpoint.live, "blinkFinal") == 1.0f);
    CHECK(lidAt(checkpoint.live) == GfVec3d(0.75, 1.0, 1.5));

    const Warmed asFloat =
        warm("blink float drag", _BlinkRigOptions(), VtValue(1.4f));
    const Warmed asDouble =
        warm("blink double drag", _BlinkRigOptions(), VtValue(1.4));
    CheckSameReadings("blink double drag against the float drag",
                      asFloat.live, asDouble.live);
}

// A chain binding a weight object refuses the freeze: its envelope is
// resolved by the oracle off the live stage, which no frozen job has. The
// sampler still samples the rig and marks nothing stale, live still
// evaluates, and once the weight object is unbound the rig freezes and a
// warmed frame is accepted and equals live.
void
TestAPropertyEnvelopeRefusesTheFreeze()
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

    std::vector<RigExecValueOverride> noOverrides;
    RigExecFrameInputs at2;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0), noOverrides,
                                   &at2, &error));
    CHECK(!at2.HasChainResolvedInputs());

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(!RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen == nullptr);
    CHECK(error.find("a property chain envelope resolves weights from the "
                     "live stage") != std::string::npos);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);

    CHECK(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/TxGain"))
              .GetRelationship(TfToken("rigExec:weightObject"))
              .SetTargets({}));
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    RigExecFrameInputs unbound;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(2.0), noOverrides,
                                   &unbound, &error));
    RigExecBackgroundScheduler scheduler;
    const RigExecRigPose warmed = RunWarmingJob(&evaluator, rig, frozen,
                                                unbound, &scheduler, nullptr);
    CheckJobAccepted("unbound envelope warmed frame 2", unbound, warmed);
    const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(2.0));
    CheckPosesBitIdentical("unbound envelope frame 2", live, warmed);
}

// testRigExecArm's phase rig (_PhaseRig there): one dial at 0.45, revised
// by Gain (x2) and then Limit (clamp to 0.6), read by readout movers at
// every phase rule, in kFrozenPhaseReadouts order:
//   Undeclared       no metadata                        0.45
//   Final            `final`                            0.6
//   Hop              `base`                             0.45
//   FinalViaHop      `final`, through Hop's input       0.6
//   FinalHop         `final`                            0.6
//   BaseViaFinalHop  no metadata, through FinalHop's    0.45
//   Checkpoint       Gain's path                        0.9
//   LastCheckpoint   Limit's path, the last revision    0.6
const char *const kFrozenPhaseReadouts[] = {
    "Undeclared", "Final",   "Hop",       "FinalViaHop",
    "FinalHop",   "BaseViaFinalHop", "Checkpoint", "LastCheckpoint"};

UsdStageRefPtr
MakeFrozenPhaseRig()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Readouts"),
                      TfToken("Scope"));
    const UsdAttribute dial =
        stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Dial"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:amount"),
                             SdfValueTypeNames->Float);
    dial.Set(0.45f);
    const auto mover = [&](const std::string &path, const char *operation,
                           const SdfPath &target) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/" + path),
            TfToken("RigExecFloatMathMover"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateAttribute(TfToken("rigExec:operation"),
                             SdfValueTypeNames->Token)
            .Set(TfToken(operation));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({target});
        return prim;
    };
    const UsdPrim limit = mover("Limit", "clamp", dial.GetPath());
    limit.CreateAttribute(TfToken("inputs:min"), SdfValueTypeNames->Float)
        .Set(0.0f);
    limit.CreateAttribute(TfToken("inputs:max"), SdfValueTypeNames->Float)
        .Set(0.6f);
    mover("Limit/Gain", "multiply", dial.GetPath())
        .CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .Set(2.0f);
    const UsdPrim readouts = stage->DefinePrim(
        SdfPath("/Asset/Rig/Channels/Readouts"), TfToken("Scope"));
    const auto readout = [&](const char *name, const SdfPath &source,
                             const char *phase) {
        const UsdAttribute channel = readouts.CreateAttribute(
            TfToken(std::string("rigExec:") + name),
            SdfValueTypeNames->Float);
        channel.Set(0.0f);
        const UsdAttribute value =
            mover(std::string("Readouts/") + name, "add", channel.GetPath())
                .CreateAttribute(TfToken("inputs:value"),
                                 SdfValueTypeNames->Float);
        value.SetConnections({source});
        if (phase) {
            value.SetMetadata(TfToken("rigExecReadPhase"),
                              std::string(phase));
        }
    };
    const SdfPath hop("/Asset/Rig/Movers/Readouts/Hop.inputs:value");
    const SdfPath finalHop(
        "/Asset/Rig/Movers/Readouts/FinalHop.inputs:value");
    readout("Undeclared", dial.GetPath(), nullptr);
    readout("Final", dial.GetPath(), "final");
    readout("Hop", dial.GetPath(), "base");
    readout("FinalViaHop", hop, "final");
    readout("FinalHop", dial.GetPath(), "final");
    readout("BaseViaFinalHop", finalHop, nullptr);
    readout("Checkpoint", dial.GetPath(), "/Asset/Rig/Movers/Limit/Gain");
    readout("LastCheckpoint", dial.GetPath(), "/Asset/Rig/Movers/Limit");
    return stage;
}

bool
FrozenPhaseReadoutsAre(const char *what, const RigExecRigPose &pose,
                       const std::vector<float> &expected)
{
    bool same = pose.valid;
    for (size_t i = 0; i < expected.size(); ++i) {
        const auto it = pose.movedProperties.find(SdfPath(
            std::string("/Asset/Rig/Channels/Readouts.rigExec:") +
            kFrozenPhaseReadouts[i]));
        const float got =
            it != pose.movedProperties.end() && it->second.IsHolding<float>()
                ? it->second.UncheckedGet<float>()
                : -1.0f;
        if (std::abs(got - expected[i]) > 1e-6f) {
            std::printf("  %s: %s reads %.9g, expected %.9g\n", what,
                        kFrozenPhaseReadouts[i], double(got),
                        double(expected[i]));
            same = false;
        }
    }
    return same;
}

// testRigExecArm's TestPhasedReadDragRules, through the worker: each drag
// rule warms bit-identically to live, the job is accepted (no stale chain
// sample, a pose returned), and the warmed readouts are the rule's. Each
// case is frozen from live's run under the case before it, so the worker's
// head tier starts from another drag's versions and re-runs what moved.
void
TestPhasedReadDragRulesFrozen()
{
    const auto drag = [](const char *prim, const char *attribute,
                         float value) {
        return RigExecValueOverride{SdfPath(prim), TfToken(),
                                    TfToken(attribute), VtValue(value)};
    };
    const std::vector<float> undragged = {0.45f, 0.6f,  0.45f, 0.6f,
                                          0.6f,  0.45f, 0.9f,  0.6f};
    struct Case {
        const char *what;
        std::vector<RigExecValueOverride> overrides;
        std::vector<float> readouts;
    };
    const Case cases[] = {
        {"undragged", {}, undragged},
        // A recorded reader's own input, which FinalViaHop's walk passes.
        {"drag on Hop",
         {drag("/Asset/Rig/Movers/Readouts/Hop", "inputs:value", 0.125f)},
         {0.45f, 0.6f, 0.125f, 0.125f, 0.6f, 0.45f, 0.9f, 0.6f}},
        // An unrecorded hop: the base reader through it stands aside.
        {"drag on FinalHop",
         {drag("/Asset/Rig/Movers/Readouts/FinalHop", "inputs:value",
               0.375f)},
         {0.45f, 0.6f, 0.45f, 0.6f, 0.375f, 0.375f, 0.9f, 0.6f}},
        // The target: the drag is the base.
        {"drag on the dial",
         {drag("/Asset/Rig/Channels/Dial", "rigExec:amount", 0.25f)},
         {0.25f, 0.5f, 0.25f, 0.5f, 0.5f, 0.25f, 0.5f, 0.5f}},
        {"lifted", {}, undragged},
    };
    const UsdStageRefPtr stage = MakeFrozenPhaseRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.GetSkippedOperations().empty());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    RigExecBackgroundScheduler scheduler;
    const UsdTimeCode time(2.0);
    for (const Case &c : cases) {
        evaluator.SetInteractiveOverrides(c.overrides);
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        std::string error;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("FAIL %s: freeze refused: %s\n", c.what,
                        error.c_str());
            continue;
        }
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, time, c.overrides, &inputs,
                                       &error));
        const RigExecRigPose warmed = RunWarmingJob(
            &evaluator, rig, frozen, inputs, &scheduler, nullptr);
        const RigExecRigPose live = evaluator.Evaluate(time);
        CHECK(live.valid);
        CheckPosesBitIdentical(c.what, live, warmed);
        CheckJobAccepted(c.what, inputs, warmed);
        CHECK(FrozenPhaseReadoutsAre(c.what, warmed, c.readouts));
    }
    evaluator.SetInteractiveOverrides({});
}

// A control whose tx (authored \p tx, static) a clamp revises to at most
// 0.5 after a gain multiplies it by \p factor, or by 0.5 x t at frames 1-4
// when \p animatedFactor: tx is a chain target and the head of tx's own
// avar binding, which reads the chain's result.
UsdStageRefPtr
MakeClampedChainRig(double tx, bool animatedFactor)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/AlongX"), TfToken("RigExecControl"))
        .GetAttribute(TfToken("avars:tx"))
        .Set(tx);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const SdfPath target("/Asset/Rig/AlongX.avars:tx");
    const auto mover = [&](const std::string &path, const char *operation) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/" + path),
            TfToken("RigExecFloatMathMover"));
        prim.ApplyAPI(TfToken("RigExecMoverAPI"));
        prim.CreateAttribute(TfToken("rigExec:operation"),
                             SdfValueTypeNames->Token)
            .Set(TfToken(operation));
        prim.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
        return prim;
    };
    const UsdPrim limit = mover("Limit", "clamp");
    limit.CreateAttribute(TfToken("inputs:min"), SdfValueTypeNames->Float)
        .Set(0.0f);
    limit.CreateAttribute(TfToken("inputs:max"), SdfValueTypeNames->Float)
        .Set(0.5f);
    const UsdAttribute factor =
        mover("Limit/Gain", "multiply")
            .CreateAttribute(TfToken("inputs:value"),
                             SdfValueTypeNames->Float);
    if (animatedFactor) {
        for (int t = 1; t <= 4; ++t) {
            factor.Set(0.5f * float(t), UsdTimeCode(double(t)));
        }
    } else {
        factor.Set(2.0f);
    }
    return stage;
}

// \p inputs less the head-leaf samples: the vector a sampler without them
// would produce, whose digest is what the frame cache keyed on before.
RigExecFrameInputs
WithoutHeadLeaves(const RigExecFrameInputs &inputs)
{
    RigExecFrameInputs out = inputs;
    out.values.clear();
    for (const RigExecSampledInput &sample : inputs.values) {
        if (sample.path.GetName().rfind("frozenChain", 0) != 0) {
            out.values.push_back(sample);
        }
    }
    return out;
}

// A chain target's own value samples at its synthetic key, as authored,
// and a reader whose head is that target (tx's avar binding) samples
// nothing: the worker resolves it to the chain's result. Two jobs whose
// targets differ only in the authored value, which the clamp folds to one
// result, have different digests -- the own value moves them -- and each
// serves a pose equal to live. Without the head leaves the two would share
// a key (equal poses, so a collision that served nothing wrong).
void
TestAChainTargetOwnValueKeyIsDistinct()
{
    const SdfPath rig("/Asset/Rig");
    const SdfPath txPath("/Asset/Rig/AlongX.avars:tx");
    const SdfPath ownKey("/Asset/Rig/AlongX.frozenChainOwn:avars:tx");
    struct Job {
        RigExecFrameInputs inputs;
        RigExecRigPose live, warmed;
    };
    const auto warm = [&](double tx) {
        Job job;
        const UsdStageRefPtr stage = MakeClampedChainRig(tx, false);
        RigExecRigEvaluator evaluator(stage, rig);
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        std::string error;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("FAIL own-value key: freeze refused: %s\n",
                        error.c_str());
            return job;
        }
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {},
                                       &job.inputs, &error));
        RigExecBackgroundScheduler scheduler;
        job.warmed = RunWarmingJob(&evaluator, rig, frozen, job.inputs,
                                   &scheduler, nullptr);
        job.live = evaluator.Evaluate(UsdTimeCode(3.0));
        CHECK(job.live.valid);
        CheckPosesBitIdentical("own-value key", job.live, job.warmed);
        CheckJobAccepted("own-value key", job.inputs, job.warmed);
        return job;
    };
    const Job three = warm(3.0);
    const Job four = warm(4.0);
    for (const Job *job : {&three, &four}) {
        CHECK(job->inputs.Find(txPath) == nullptr);
        const auto reader = job->warmed.movedProperties.find(txPath);
        CHECK(reader != job->warmed.movedProperties.end() &&
              reader->second.IsHolding<double>() &&
              reader->second.UncheckedGet<double>() == 0.5);
    }
    const VtValue *own3 = three.inputs.Find(ownKey);
    const VtValue *own4 = four.inputs.Find(ownKey);
    CHECK(own3 != nullptr && own3->IsHolding<double>() &&
          own3->Get<double>() == 3.0);
    CHECK(own4 != nullptr && own4->IsHolding<double>() &&
          own4->Get<double>() == 4.0);
    CHECK(RigExecFrozenControlDigest(three.inputs) !=
          RigExecFrozenControlDigest(four.inputs));
    CHECK(RigExecControlStateDigest(three.inputs, {}) !=
          RigExecControlStateDigest(four.inputs, {}));
    CheckSameReadings("own-value key: equal results", three.live, four.live);
    // The pre-existing gap the head leaves close: without them the two
    // vectors are the same control state.
    CHECK(RigExecControlStateDigest(WithoutHeadLeaves(three.inputs), {}) ==
          RigExecControlStateDigest(WithoutHeadLeaves(four.inputs), {}));
}

// Two frames that differ only in a chain mover's own animated inputs:value,
// whose effect the clamp folds away, have different control digests: the
// factor is a head-leaf sample. Without the head leaves they collide (the
// pre-existing gap, harmless here because the poses are equal), which the
// last check pins so a sampler that dropped the leaves is caught.
void
TestAChainMoverInputMovesTheDigest()
{
    const SdfPath rig("/Asset/Rig");
    const UsdStageRefPtr stage = MakeClampedChainRig(3.0, true);
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("FAIL chain mover input digest: freeze refused: %s\n",
                    error.c_str());
        return;
    }
    RigExecBackgroundScheduler scheduler;
    RigExecFrameInputs at3, at4;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0), {}, &at3,
                                   &error));
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(4.0), {}, &at4,
                                   &error));
    const SdfPath factorKey(
        "/Asset/Rig/Movers/Limit/Gain.frozenChainHop:inputs:value");
    const VtValue *factor3 = at3.Find(factorKey);
    const VtValue *factor4 = at4.Find(factorKey);
    CHECK(factor3 != nullptr && factor3->IsHolding<float>() &&
          factor3->Get<float>() == 1.5f);
    CHECK(factor4 != nullptr && factor4->IsHolding<float>() &&
          factor4->Get<float>() == 2.0f);
    CHECK(RigExecControlStateDigest(at3, {}) !=
          RigExecControlStateDigest(at4, {}));
    CHECK(RigExecFrozenControlDigest(at3) != RigExecFrozenControlDigest(at4));
    CHECK(RigExecControlStateDigest(WithoutHeadLeaves(at3), {}) ==
          RigExecControlStateDigest(WithoutHeadLeaves(at4), {}));
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CheckPosesBitIdentical("chain mover input, frame 3", live3, warmed3);
    CheckJobAccepted("chain mover input, frame 3", at3, warmed3);
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    const RigExecRigPose warmed4 =
        RunWarmingJob(&evaluator, rig, frozen, at4, &scheduler, nullptr);
    const RigExecRigPose live4 = evaluator.Evaluate(UsdTimeCode(4.0));
    CheckPosesBitIdentical("chain mover input, frame 4", live4, warmed4);
    CheckJobAccepted("chain mover input, frame 4", at4, warmed4);
    CheckSameReadings("chain mover input: equal results", live3, live4);
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
        CHECK(plain.stageSeeds == burst.stageSeeds);
        CHECK(RigExecFrozenControlDigest(plain) ==
              RigExecFrozenControlDigest(burst));
        CHECK(RigExecControlStateDigest(plain, noOverrides) ==
              RigExecControlStateDigestWithBurstCache(burst, noOverrides,
                                                      &cache));
    }
}

// The paths of every volume slot the walk places (placedVolumes) of \p B.
static std::set<SdfPath>
PlacedVolumeSlots(const RigExecBakedProgramImpl &B)
{
    std::set<SdfPath> paths;
    for (size_t i = 0; i < B.placedVolumes.size() && i < B.paths.size();
         ++i) {
        if (B.placedVolumes[i]) {
            paths.insert(B.paths[i]);
        }
    }
    return paths;
}

static std::set<SdfPath>
KeysOf(const std::map<SdfPath, GfMatrix4d> &frames)
{
    std::set<SdfPath> keys;
    for (const auto &[path, frame] : frames) {
        keys.insert(path);
    }
    return keys;
}

// The clusters of every VolumePlacements step of \p B (one per volume
// slot), in program order.
static std::vector<int>
VolumePlacementsClusters(const RigExecBakedProgramImpl &B)
{
    std::vector<int> clusters;
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::VolumePlacements) {
            clusters.push_back(step.cluster);
        }
    }
    return clusters;
}

// Frozen weightFrames must equal live's, keys and bits.
static void
CheckWeightFramesEqual(const char *what, const char *path,
                       const std::map<SdfPath, GfMatrix4d> &live,
                       const std::map<SdfPath, GfMatrix4d> &frozen)
{
    if (frozen == live) {
        return;
    }
    std::printf("FAIL %s: %s publishes %zu weight frame(s), live %zu%s\n",
                what, path, frozen.size(), live.size(),
                KeysOf(frozen) == KeysOf(live) ? " (values differ)" : "");
    ++failures;
}

// Every frozen path publishes live's weightFrames at \p time: an empty-plan
// cone restored from live slots, a cone that runs every cluster (every
// VolumePlacements step included), an empty-plan cone chained from that
// cone's slots, and a whole job. Returns the slots the all-cluster cone
// left.
static std::shared_ptr<const void>
CheckFrozenWeightFramesMatchLive(RigExecRigEvaluator *evaluator,
                                 const SdfPath &rig, UsdTimeCode time,
                                 const char *what)
{
    const RigExecRigPose live = evaluator->Evaluate(time);
    CHECK(live.valid);
    CHECK(!live.weightFrames.empty());
    const RigExecBakedProgramImpl &B =
        evaluator->GetBakedProgram()->GetStepGraph();
    CHECK(KeysOf(live.weightFrames) == PlacedVolumeSlots(B));
    std::shared_ptr<const void> slots;
    size_t slotBytes = 0;
    CHECK(RigExecCapturePartialSlots(B, &slots, &slotBytes));
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(*evaluator, &frozen, &error));
    RigExecFrameInputs inputs;
    CHECK(RigExecSampleFrameInputs(*evaluator, time, {}, &inputs, &error));
    if (!frozen || !slots) {
        std::printf("FAIL %s: no snapshot (%s)\n", what, error.c_str());
        ++failures;
        return {};
    }
    const size_t clusters = B.clustering.clusters.size();
    const std::vector<int> placements = VolumePlacementsClusters(B);
    CHECK(!placements.empty());
    for (const int placement : placements) {
        CHECK(placement >= 0);
    }

    RigExecBakedClusterSet none;
    none.Resize(clusters);
    const RigExecPartialRunResult skipped =
        RigExecRunPartialCone(*frozen, inputs, slots, live, none, 0);
    CHECK(skipped.completed);
    CHECK(skipped.executedClusters.empty());
    CheckWeightFramesEqual(what, "an empty-plan cone from live slots",
                           live.weightFrames, skipped.pose.weightFrames);

    RigExecBakedClusterSet all;
    all.SetAll(clusters);
    const RigExecPartialRunResult ran =
        RigExecRunPartialCone(*frozen, inputs, slots, live, all, 0);
    CHECK(ran.completed);
    for (const int placement : placements) {
        CHECK(std::find(ran.executedClusters.begin(),
                        ran.executedClusters.end(),
                        placement) != ran.executedClusters.end());
    }
    CheckWeightFramesEqual(what, "a cone that ran VolumePlacements",
                           live.weightFrames, ran.pose.weightFrames);
    const RigExecPartialRunResult chained =
        RigExecRunPartialCone(*frozen, inputs, ran.slots, ran.pose, none, 0);
    CHECK(chained.completed);
    CHECK(chained.executedClusters.empty());
    CheckWeightFramesEqual(what, "an empty-plan cone from frozen slots",
                           live.weightFrames, chained.pose.weightFrames);

    // On these rigs every VolumePlacements step shares its cluster with
    // steps that read outside the program, so a whole job runs each of
    // them. TestFrozenWholeRunSkippingVolumePlacementsPublishesLive covers
    // the job that skips them.
    for (const int placement : placements) {
        CHECK(placement >= 0 && B.cones.always.Test(placement));
    }
    RigExecBackgroundScheduler scheduler;
    bool wholeRan = false;
    const RigExecRigPose whole =
        RunWarmingJob(evaluator, rig, frozen, inputs, &scheduler, &wholeRan);
    CHECK(wholeRan);
    CheckWeightFramesEqual(what, "a whole job", live.weightFrames,
                           whole.weightFrames);
    return ran.slots;
}

// Frozen publishes exactly live's weightFrames on every path. On
// 14_VolumeConstrainedSweep with a volume outside the rig that only a
// constraint names, that volume has a slot and a placement step, which the
// frozen cone runs, and neither live nor frozen publishes it.
void
TestFrozenWeightFramesMatchLive(const std::string &examplesDir)
{
    const auto open = [](const std::string &path, SdfPath *rig) {
        UsdStageRefPtr stage = UsdStage::Open(path);
        CHECK(stage);
        if (stage) {
            for (const UsdPrim &prim : stage->TraverseAll()) {
                if (prim.GetTypeName() == "RigExecRoot") {
                    *rig = prim.GetPath();
                    break;
                }
            }
        }
        CHECK(!rig->IsEmpty());
        return stage;
    };
    {
        SdfPath rig;
        UsdStageRefPtr stage =
            open(examplesDir + "/11_VolumeWeights.usda", &rig);
        if (!stage || rig.IsEmpty()) {
            return;
        }
        RigExecRigEvaluator evaluator(stage, rig);
        CHECK(evaluator.Compile());
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(evaluator.Evaluate(UsdTimeCode(1001.0)).valid);
        CheckFrozenWeightFramesMatchLive(&evaluator, rig, UsdTimeCode(1012.0),
                                         "11_VolumeWeights");
    }
    SdfPath rig;
    UsdStageRefPtr stage =
        open(examplesDir + "/14_VolumeConstrainedSweep.usda", &rig);
    if (!stage || rig.IsEmpty()) {
        return;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const SdfPath outsidePath("/SweepAsset/Outside");
    const UsdPrim outside =
        stage->DefinePrim(outsidePath, TfToken("RigExecSphereWeight"));
    const char *const avars[3] = {"avars:tx", "avars:ty", "avars:tz"};
    const GfVec3d from(0, 5, 0), to(6, 5, 4);
    for (int k = 0; k < 3; ++k) {
        const UsdAttribute avar = outside.CreateAttribute(
            TfToken(avars[k]), SdfValueTypeNames->Double);
        avar.Set(from[k], UsdTimeCode(1001));
        avar.Set(to[k], UsdTimeCode(1048));
    }
    stage->GetPrimAtPath(SdfPath("/SweepAsset/Rig/Movers/Sweep"))
        .GetRelationship(TfToken("rigExec:sources"))
        .SetTargets({outsidePath});
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1001.0)).valid);
    const std::shared_ptr<const void> ranSlots =
        CheckFrozenWeightFramesMatchLive(&evaluator, rig, UsdTimeCode(1024.0),
                                         "untapped volume source");
    // Not vacuous: the untapped volume is a volume slot, and the cone that
    // ran its placement step placed it (ty = 5) without publishing it.
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();
    const auto slot = B.index.find(outsidePath);
    CHECK(slot != B.index.end());
    if (slot == B.index.end() || !ranSlots) {
        return;
    }
    const size_t i = size_t(slot->second);
    CHECK(i < B.noScaleAvars.size() && B.noScaleAvars[i] == 1);
    CHECK(i < B.placedVolumes.size() && B.placedVolumes[i] == 0);
    // RigExecPartialSlots holds the volume slots only, in slot order.
    size_t compact = 0;
    for (size_t k = 0; k < i && k < B.noScaleAvars.size(); ++k) {
        compact += B.noScaleAvars[k] ? 1 : 0;
    }
    const RigExecPartialSlots &held =
        *std::static_pointer_cast<const RigExecPartialSlots>(ranSlots);
    CHECK(compact < held.volumePlacement.size());
    if (compact < held.volumePlacement.size()) {
        CHECK(held.volumePlacement[compact].ExtractTranslation()[1] == 5.0);
    }
}

// A whole frozen job whose closure skips VolumePlacements publishes live's
// weightFrames from the placements its clone kept. A frozen job runs whole
// clusters, so the step is skipped only when its cluster is clean. Here
// nothing in the rig reads outside the program (no mover, so no
// RevisionStatic), and the only volume is a static guide bound to nothing,
// so its placement cluster is not always run. A job at 3, cloned from live
// at 2, has no dirty input (rz is held after 2). The skip is shown by
// poisoning the guide's placement in a second snapshot: the job at 3
// publishes the poison, and the job at 1 (rz changes) overwrites it.
void
TestFrozenWholeRunSkippingVolumePlacementsPublishesLive()
{
    const char *label = "whole frozen run skips VolumePlacements";
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const bool imported = stage->GetRootLayer()->ImportFromString(R"(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 10
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def RigExecControl "Root"
        {
            double avars:rz = 0
            double avars:rz.timeSamples = {1: 0, 2: 45}
        }
        def RigExecJoint "Joint"
        {
            matrix4d rest:space = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1))
            double avars:rz.connect = </Asset/Rig/Root.avars:rz>
        }
        def RigExecSphereWeight "Guide"
        {
            double avars:ty = 3
            float inputs:falloffMin = 0
            float inputs:falloffMax = 1
        }
    }
}
)");
    CHECK(imported);
    const SdfPath rig("/Asset/Rig"), guide("/Asset/Rig/Guide");
    RigExecRigEvaluator evaluator(stage, rig);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    evaluator.SetProfilingEnabled(true);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("FAIL %s: compile error: %s\n", label, e.c_str());
        }
        ++failures;
        return;
    }
    CHECK(evaluator.Evaluate(UsdTimeCode(1)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2)).valid);
    CHECK(evaluator.GetBakedGenerationCount() == 2);
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program != nullptr);
    if (!program) {
        return;
    }
    // The setup: the program has one VolumePlacements step, whose cluster is
    // not always run, and no step reads outside the program.
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const std::vector<int> placements = VolumePlacementsClusters(B);
    CHECK(placements.size() == 1);
    for (const int placement : placements) {
        CHECK(placement >= 0 && !B.cones.always.Test(placement));
    }
    for (const RigExecBakedStep &step : B.steps) {
        CHECK(!step.externalReads);
    }
    const auto guideSlot = B.index.find(guide);
    CHECK(guideSlot != B.index.end());
    if (guideSlot == B.index.end()) {
        return;
    }

    std::shared_ptr<const RigExecFrozenProgram> frozen, poisoned;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(RigExecFreezeProgram(evaluator, &poisoned, &error));
    RigExecFrameInputs held, moved;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3), {}, &held,
                                   &error));
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(1), {}, &moved,
                                   &error));
    if (!frozen || !poisoned) {
        std::printf("FAIL %s: no snapshot (%s)\n", label, error.c_str());
        ++failures;
        return;
    }
    // RigExecFreezeProgram allocates the snapshot non-const, and no job is
    // running on it.
    GfMatrix4d poison(1.0);
    poison.SetTranslate(GfVec3d(0, 100, 0));
    const_cast<RigExecFrozenProgram &>(*poisoned)
        .program.volumePlacement[size_t(guideSlot->second)] = poison;

    RigExecBackgroundScheduler scheduler;
    bool ran = false;
    const RigExecRigPose poisonedHeld =
        RunWarmingJob(&evaluator, rig, poisoned, held, &scheduler, &ran);
    CHECK(ran);
    const auto poisonAt3 = poisonedHeld.weightFrames.find(guide);
    if (poisonAt3 == poisonedHeld.weightFrames.end() ||
        poisonAt3->second != poison) {
        std::printf("FAIL %s: the job at 3 does not publish the clone's "
                    "placement (it ran VolumePlacements or omits the guide)\n",
                    label);
        ++failures;
    }
    ran = false;
    const RigExecRigPose poisonedMoved =
        RunWarmingJob(&evaluator, rig, poisoned, moved, &scheduler, &ran);
    CHECK(ran);
    const auto poisonAt1 = poisonedMoved.weightFrames.find(guide);
    CHECK(poisonAt1 != poisonedMoved.weightFrames.end() &&
          poisonAt1->second.ExtractTranslation()[1] == 3.0);

    ran = false;
    const RigExecRigPose skipped =
        RunWarmingJob(&evaluator, rig, frozen, held, &scheduler, &ran);
    CHECK(ran);
    std::shared_ptr<const void> skippedSlots;
    size_t skippedSlotBytes = 0;
    CHECK(RigExecTakeLastFrozenSlots(&skippedSlots, &skippedSlotBytes));
    ran = false;
    const RigExecRigPose placed =
        RunWarmingJob(&evaluator, rig, frozen, moved, &scheduler, &ran);
    CHECK(ran);

    evaluator.ClearProfile();
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3));
    CHECK(live3.valid);
    for (const RigExecOpTraceEntry &entry : evaluator.GetLastOpTrace()) {
        CHECK(entry.kind != "VolumePlacements");
    }
    const auto guideLive = live3.weightFrames.find(guide);
    CHECK(live3.weightFrames.size() == 1);
    CHECK(guideLive != live3.weightFrames.end() &&
          guideLive->second.ExtractTranslation()[1] == 3.0);
    CheckWeightFramesEqual(label, "the job at 3", live3.weightFrames,
                           skipped.weightFrames);
    const RigExecRigPose live1 = evaluator.Evaluate(UsdTimeCode(1));
    CHECK(live1.valid);
    CheckWeightFramesEqual(label, "the job at 1", live1.weightFrames,
                           placed.weightFrames);

    if (!skippedSlots) {
        return;
    }
    RigExecBakedClusterSet none;
    none.Resize(B.clustering.clusters.size());
    const RigExecPartialRunResult chained =
        RigExecRunPartialCone(*frozen, held, skippedSlots, skipped, none, 0);
    CHECK(chained.completed);
    CHECK(chained.executedClusters.empty());
    CheckWeightFramesEqual(label, "an empty-plan cone from the job at 3",
                           live3.weightFrames, chained.pose.weightFrames);
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

// Stream B: the frozen worker entry through an injected serial kernel.

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
    bool sawWireMemo = false;
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
    CHECK(sawWireMemo);
    CHECK(sawStage);
    CHECK(sawEvaluator);
    CHECK(sawDynamic);
    CHECK(sawSolverKernels);
    CHECK(sawMoverKernels);
}

}  // namespace

// The biped: sixteen float chain-driven inputs over the foot movers, and
// the rig the worker's head tier unlocks. Warmed frames match live with zero
// parity mismatches -- the same bar as the fixture, on a production rig.
//
// The counts are the DELIVERED rig's. They are asserted rather than
// printed because a chain that stops binding is a chain the warming job
// silently stops sampling, and the parity check below would then compare
// two runs that agree about the wrong thing.
void
TestBipedWarmsBitIdentical(const std::string &examplesDir,
                           const std::string &stageFile = "Biped_anim.usda",
                           size_t expectChains = 38)
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
    // The chains' inputs travel as head leaves, under their own keys.
    size_t headSamples = 0;
    for (const RigExecSampledInput &sample : at3.values) {
        headSamples +=
            sample.path.GetName().rfind("frozenChain", 0) == 0 ? 1 : 0;
    }
    CHECK(headSamples > 0);
    const RigExecRigPose warmed3 =
        RunWarmingJob(&evaluator, rig, frozen, at3, &scheduler, nullptr);
    const RigExecRigPose live3 = evaluator.Evaluate(UsdTimeCode(3.0));
    CHECK(live3.valid);
    CheckPosesBitIdentical(what3.c_str(), live3, warmed3);
    CheckJobAccepted(what3.c_str(), at3, warmed3);

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

// Per-frame operator inputs that move nothing else: a space switch's index,
// a numeric pose driver's dial, a TwoBoneIk rigExec:spaceMatrix, and the
// controls behind a radial cluster and a posed wire. Each is keyed to differ
// between frames 3 and 4 in the session layer. Live runs with the parity
// check, so a baked step that does not declare one of them (and is skipped
// on the time change) disagrees with the dynamic path; a warm that replays a
// stale constant -- an input the sampler never visits -- disagrees with live.
void
TestKeyedOperatorInputsWarmBitIdentical(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/biped/Biped_stack.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const auto keyNumber = [](const UsdAttribute &a, double v3, double v4) {
        if (!a) return false;
        if (a.GetTypeName() == SdfValueTypeNames->Double) {
            a.Set(v3, UsdTimeCode(3.0));
            a.Set(v4, UsdTimeCode(4.0));
            return true;
        }
        if (a.GetTypeName() == SdfValueTypeNames->Float) {
            a.Set(float(v3), UsdTimeCode(3.0));
            a.Set(float(v4), UsdTimeCode(4.0));
            return true;
        }
        return false;
    };
    const auto keyAvar = [&](const SdfPath &path, const char *avar,
                             double v3, double v4) {
        const UsdPrim prim = stage->GetPrimAtPath(path);
        if (!prim || prim.GetTypeName() != "RigExecControl") return false;
        UsdAttribute a = prim.GetAttribute(TfToken(avar));
        if (!a) {
            a = prim.CreateAttribute(TfToken(avar),
                                     SdfValueTypeNames->Double);
        }
        return keyNumber(a, v3, v4);
    };
    SdfPath rig;
    bool switchKeyed = false, dialKeyed = false, ikKeyed = false;
    bool radialKeyed = false, wireKeyed = false;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        const TfToken type = prim.GetTypeName();
        SdfPathVector targets;
        if (type == "RigExecRoot" && rig.IsEmpty()) {
            rig = prim.GetPath();
        } else if (type == "RigExecSpaceSwitch" && !switchKeyed) {
            prim.GetRelationship(TfToken("rigExec:activeSpaceAttribute"))
                .GetTargets(&targets);
            switchKeyed = keyNumber(
                targets.empty()
                    ? prim.GetAttribute(TfToken("inputs:activeSpace"))
                    : stage->GetAttributeAtPath(targets[0]),
                0.0, 1.0);
        } else if (type == "RigExecPoseInterpolator" && !dialKeyed) {
            prim.GetRelationship(TfToken("rigExec:driverAttributes"))
                .GetTargets(&targets);
            dialKeyed = !targets.empty() &&
                        keyNumber(stage->GetAttributeAtPath(targets[0]), 0.0,
                                  2.0);
        } else if (type == "RigExecTwoBoneIk" && !ikKeyed) {
            GfMatrix4d scaled(1.0);
            scaled.SetScale(1.1);
            UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:spaceMatrix"));
            if (a) {
                a.Set(GfMatrix4d(1.0), UsdTimeCode(3.0));
                a.Set(scaled, UsdTimeCode(4.0));
                ikKeyed = true;
            }
        } else if (type == "RigExecMatrixMover" && !radialKeyed) {
            TfToken blend;
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:weightBlend"))) {
                a.Get(&blend);
            }
            prim.GetRelationship(TfToken("rigExec:transform"))
                .GetTargets(&targets);
            radialKeyed = blend == "radial" && !targets.empty() &&
                          keyAvar(targets[0], "avars:rx", 0.0, 20.0);
        } else if (type == "RigExecCurveMover" && !wireKeyed) {
            TfToken delta;
            if (UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:driverDeltaFrame"))) {
                a.Get(&delta);
            }
            prim.GetRelationship(TfToken("rigExec:driverTransforms"))
                .GetTargets(&targets);
            wireKeyed = delta == "posed" && !targets.empty() &&
                        keyAvar(targets[0], "avars:ty", 0.0, 1.0);
        }
    }
    CHECK(!rig.IsEmpty());
    CHECK(switchKeyed);
    CHECK(dialKeyed);
    CHECK(ikKeyed);
    CHECK(radialKeyed);
    CHECK(wireKeyed);
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(2.0)).valid);
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("keyed operator inputs: freeze refused: %s\n",
                    error.c_str());
        return;
    }
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    for (const double frame : {3.0, 4.0}) {
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &inputs, &error));
        const RigExecRigPose warmed = RunWarmingJob(
            &evaluator, rig, frozen, inputs, &scheduler, nullptr);
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid);
        if (live.bakedParityMismatches != 0) {
            for (const std::string &diagnostic : live.diagnostics) {
                std::printf("keyed operator inputs live frame %g: %s\n",
                            frame, diagnostic.c_str());
            }
        }
        CHECK(live.bakedParityMismatches == 0);
        const std::string what =
            "keyed operator inputs warmed frame " + std::to_string(frame);
        CheckPosesBitIdentical(what.c_str(), live, warmed);
    }
}

// The stack (examples/biped/Biped_stack_anim.usda) warms bit-identically:
// the layered full-body rig with its pose-interpolator steps, which no
// flat rig builds, warmed from the same two-frame history.
void
TestStackAnimWarmsBitIdentical(const std::string &examplesDir)
{
    TestBipedWarmsBitIdentical(examplesDir, "Biped_stack_anim.usda",
                               /*expectChains=*/335);
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

void
TestIterativeMoversWarmBitIdentical()
{
    for (bool wrinkle : {false, true}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const SdfPath rig("/Rig"), target("/Rig/Mesh.points");
        auto builder = RigExecRigBuilder::Create(stage, rig);
        const UsdPrim mesh = stage->DefinePrim(target.GetPrimPath(), TfToken("Mesh"));
        VtVec3fArray rest, posed;
        VtIntArray counts, indices;
        for (int y = 0; y < 5; ++y) {
            for (int x = 0; x < 5; ++x) {
                rest.push_back(GfVec3f(0.25f * x, 0.25f * y, 0.0f));
                posed.push_back(GfVec3f(0.15f * x, 0.25f * y,
                                      (x == 2 && y == 2) ? 0.2f : 0.0f));
            }
        }
        for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 4; ++x) {
                const int a = y * 5 + x;
                counts.push_back(4);
                for (int index : {a, a + 1, a + 6, a + 5}) {
                    indices.push_back(index);
                }
            }
        }
        CHECK(mesh.CreateAttribute(TfToken("points"),
            SdfValueTypeNames->Point3fArray).Set(posed));
        CHECK(mesh.CreateAttribute(TfToken("faceVertexCounts"),
            SdfValueTypeNames->IntArray).Set(counts));
        CHECK(mesh.CreateAttribute(TfToken("faceVertexIndices"),
            SdfValueTypeNames->IntArray).Set(indices));
        auto chain = builder.NewMoverChain("Deform", target);
        const UsdPrim mover = wrinkle
            ? chain.AddWrinkleMover("Mover").GetPrim()
            : chain.AddDeltaMushMover("Mover").GetPrim();
        CHECK(mover.GetAttribute(TfToken("inputs:restPoints")).Set(rest));
        CHECK(mover.GetAttribute(TfToken("inputs:pinBorders")).Set(false));
        const UsdAttribute iterations = mover.GetAttribute(TfToken("inputs:iterations"));
        CHECK(iterations.Set(4, UsdTimeCode(1.0)));
        CHECK(iterations.Set(12, UsdTimeCode(3.0)));
        const TfToken amplitudeName(wrinkle ? "inputs:wrinkleScale"
                                            : "inputs:displacement");
        const UsdAttribute amplitude = mover.GetAttribute(amplitudeName);
        // A connected scalar exercises the same resolved read as live.
        const UsdAttribute driver = mesh.CreateAttribute(TfToken("deformAmount"),
            SdfValueTypeNames->Float);
        CHECK(driver.Set(0.0f, UsdTimeCode(1.0)));
        CHECK(driver.Set(1.0f, UsdTimeCode(3.0)));
        CHECK(amplitude.SetConnections({driver.GetPath()}));
        if (wrinkle) {
            CHECK(mover.GetAttribute(TfToken("inputs:topology"))
                .Set(TfToken("surfaceStruts")));
            CHECK(mover.GetAttribute(TfToken("inputs:pinPoints"))
                .Set(VtIntArray{0, 4}));
        }
        RigExecRigEvaluator evaluator(stage, rig);
        CHECK(evaluator.Compile());
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        std::string error;
        CHECK(RigExecCanFreezeProgram(evaluator, &error));
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("iterative mover freeze refused: %s\n", error.c_str());
            continue;
        }
        RigExecChainSampleBindings bindings;
        CHECK(RigExecBindChainSampleInputs(evaluator, &bindings, &error));
        RigExecBurstSampleCache cache;
        CHECK(RigExecBuildBurstSampleCache(*evaluator.GetBakedProgram(),
            bindings, {}, RigExecFrameCacheEpochDigest(evaluator), &cache, &error));
        RigExecBackgroundScheduler scheduler;
        uint64_t previousDigest = 0;
        VtVec3fArray previousPoints;
        for (double frame : {3.0, 2.0, 1.0, 3.0}) {
            RigExecFrameInputs plain, burst;
            CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                {}, &plain, &error));
            CHECK(RigExecSampleFrameInputsWithBurstCache(evaluator,
                UsdTimeCode(frame), {}, &cache, &burst, &error));
            const uint64_t digest = RigExecControlStateDigest(plain);
            CHECK(digest == RigExecControlStateDigestWithBurstCache(burst, {}, &cache));
            CHECK(digest != previousDigest);
            previousDigest = digest;
            const auto warmed = RunWarmingJob(&evaluator, rig, frozen,
                burst, &scheduler, nullptr);
            const auto live = evaluator.Evaluate(UsdTimeCode(frame));
            CheckPosesBitIdentical(wrinkle ? "wrinkle" : "delta mush", live, warmed);
            const auto found = warmed.movedProperties.find(target);
            CHECK(found != warmed.movedProperties.end());
            if (found != warmed.movedProperties.end() &&
                found->second.IsHolding<VtVec3fArray>()) {
                const auto &points = found->second.UncheckedGet<VtVec3fArray>();
                CHECK(points != previousPoints);
                previousPoints = points;
            }
            CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        }
        // A standing override at the same time must also change the digest
        // and match a live evaluation of that override.
        RigExecValueOverride override;
        override.prim = mover.GetPath();
        override.attribute = amplitudeName;
        override.value = VtValue(0.25f);
        evaluator.SetInteractiveOverrides({override});
        RigExecFrameInputs overridden;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(3.0),
            {override}, &overridden, &error));
        CHECK(RigExecControlStateDigest(overridden) != previousDigest);
        const auto warmed = RunWarmingJob(&evaluator, rig, frozen,
            overridden, &scheduler, nullptr);
        const auto live = evaluator.Evaluate(UsdTimeCode(3.0));
        CheckPosesBitIdentical("iterative mover override", live, warmed);
    }
}

// Frozen equals live, bit for bit, on the rigs whose lattices (a phased cage
// among them), wires, deltaMush movers, projector targets and derived
// normals the worker assembles from the job's leaves: each job carries a
// leaf value for every key of those revisions and derived targets, and
// serves live's pose at the stage's first frame and four frames on. (The
// wrinkle, which no example ships, warms in
// TestIterativeMoversWarmBitIdentical.)
void
TestFrozenAssemblesFromLeaves(const std::string &examplesDir)
{
    const std::string fixtures = examplesDir + "/../tests/fixtures/";
    std::set<RigExecRevisionOp> warmed;
    for (const std::string &stagePath :
         {examplesDir + "/06_LatticeBulge.usda",
          examplesDir + "/13_ReadPhases.usda",
          examplesDir + "/2d/bust/bust_anim.usda",
          fixtures + "projector_spaces.usda",
          fixtures + "computed_path_reads.usda",
          examplesDir + "/biped/Biped_stack_anim.usda"}) {
        UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        SdfPath rig;
        for (const UsdPrim &prim : stage->TraverseAll()) {
            if (prim.GetTypeName() == "RigExecRoot") {
                rig = prim.GetPath();
                break;
            }
        }
        CHECK(!rig.IsEmpty());
        RigExecRigEvaluator evaluator(stage, rig);
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        const double start = stage->GetStartTimeCode();
        CHECK(evaluator.Evaluate(UsdTimeCode(start)).valid);
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        std::string error;
        if (!RigExecFreezeProgram(evaluator, &frozen, &error)) {
            std::printf("frozen leaves: %s does not freeze (%s)\n",
                        stagePath.c_str(), error.c_str());
            continue;
        }
        RigExecBackgroundScheduler scheduler;
        for (const double frame : {start, start + 4.0}) {
            RigExecFrameInputs inputs;
            CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame), {},
                                           &inputs, &error));
            const RigExecBakedProgramImpl &B =
                evaluator.GetBakedProgram()->GetStepGraph();
            const auto carried = [&](const std::vector<VtValue> &values,
                                     const RigExecBakedProgramImpl::
                                         GeomRevision &revision) {
                const bool held =
                    revision.leaves.decl.assembles &&
                    values.size() == revision.leaves.decl.keys.size();
                if (!held) {
                    std::printf("FAIL frozen leaves %s: %s travels without "
                                "its leaves\n",
                                stagePath.c_str(),
                                revision.moverPath.GetText());
                }
                CHECK(held);
                warmed.insert(revision.op);
            };
            CHECK(inputs.revisionLeaves.size() == B.revisionIndex.size());
            for (size_t r = 0; r < B.revisionIndex.size() &&
                               r < inputs.revisionLeaves.size();
                 ++r) {
                const auto &[c, i] = B.revisionIndex[r];
                const auto &revision = B.chains[size_t(c)].revisions[size_t(i)];
                if (revision.op == RigExecRevisionOp::Lattice ||
                    revision.op == RigExecRevisionOp::Wire ||
                    revision.op == RigExecRevisionOp::DeltaMush ||
                    revision.op == RigExecRevisionOp::Wrinkle) {
                    carried(inputs.revisionLeaves[r], revision);
                }
            }
            CHECK(inputs.derivedLeaves.size() == B.derivedIndex.size());
            for (size_t d = 0; d < B.derivedIndex.size() &&
                               d < inputs.derivedLeaves.size();
                 ++d) {
                const auto &[c, i] = B.derivedIndex[d];
                carried(inputs.derivedLeaves[d],
                        B.chains[size_t(c)].derived[size_t(i)].revision);
            }
            const RigExecRigPose job = RunWarmingJob(&evaluator, rig, frozen,
                                                     inputs, &scheduler,
                                                     nullptr);
            const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
            CHECK(live.valid);
            CheckPosesBitIdentical(TfStringPrintf("frozen leaves %s frame %g",
                                                  stagePath.c_str(), frame)
                                       .c_str(),
                                   live, job);
            CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        }
        std::printf("frozen leaves: %s warmed\n", stagePath.c_str());
    }
    for (const RigExecRevisionOp op :
         {RigExecRevisionOp::Lattice, RigExecRevisionOp::Wire,
          RigExecRevisionOp::DeltaMush, RigExecRevisionOp::SurfaceProjector,
          RigExecRevisionOp::ShaderDials, RigExecRevisionOp::RecomputeNormals}) {
        if (!warmed.count(op)) {
            std::printf("FAIL frozen leaves: no rig warmed op %d\n", int(op));
        }
        CHECK(warmed.count(op) == 1);
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

// Read phases on connections (examples/16_ConnectionReadPhases.usda) warm
// bit-identically: the frame-cache sampler publishes each phased reader's
// value from its own pass over the dial's chain, as the live path does.
void
TestConnectionReadPhasesWarmBitIdentical(const std::string &examplesDir)
{
    CheckExampleWarmsBitIdentical(
        examplesDir + "/16_ConnectionReadPhases.usda", {1001.0},
        {1012.0, 1024.0, 1036.0, 1048.0},
        SdfPath("/PhaseConnectAsset/Geom/GainCard.points"));
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

// A value edit the live program routed but has not yet run travels into the
// snapshot. The edit here is a time sample of a constraint's per-frame input
// at the held time, which only its edited bit can route: the time does not
// move. The generation right after it is answered by the dynamic path (an
// override the program cannot place stands for one frame), so the program
// never consumes the bit, and a snapshot taken then has to carry it -- its
// first run is at its own lastTime, where nothing else dirties the
// constraint, and without the bit it would republish the pre-edit pose.
void
TestAFrozenRewarmAfterAFallbackCarriesTheEdit(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/aimtest.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig("/World/RigRoot");
    const SdfPath aim("/World/RigRoot/Movers/RigExecAimConstraint1");
    const UsdAttribute weight = stage->GetPrimAtPath(aim).CreateAttribute(
        TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float);
    CHECK(weight.Set(1.0f, UsdTimeCode(1.0)));
    CHECK(weight.Set(0.5f, UsdTimeCode(100.0)));
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    const UsdTimeCode held(50.0);
    CHECK(evaluator.Evaluate(held).valid);
    const RigExecRigPose before = evaluator.Evaluate(held);
    CHECK(before.valid);
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program != nullptr);
    if (!program) {
        return;
    }
    // An override on a folded value, restating the value it already has:
    // the program cannot place it, so the generation falls back, and the
    // dynamic path it falls back to answers the same pose.
    RigExecValueOverride unplaceable;
    bool found = false;
    for (const SdfPath &path : program->GetStepGraph().folded) {
        const UsdAttribute a = stage->GetAttributeAtPath(path);
        VtValue value;
        if (a && a.Get(&value) && !value.IsEmpty()) {
            unplaceable.prim = path.GetPrimPath();
            unplaceable.attribute = path.GetNameToken();
            unplaceable.value = value;
            found = true;
            break;
        }
    }
    CHECK(found);
    if (!found) {
        return;
    }

    CHECK(weight.Set(0.25f, UsdTimeCode(100.0)));
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Edited);
    const size_t generations = evaluator.GetBakedGenerationCount();
    evaluator.SetInteractiveOverrides({unplaceable});
    CHECK(evaluator.Evaluate(held).valid);
    CHECK(evaluator.GetBakedGenerationCount() == generations);
    evaluator.SetInteractiveOverrides({});
    CHECK(evaluator.GetBakedProgram()->GetStepGraph().anyEdited);

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    CHECK(frozen != nullptr);
    if (!frozen) {
        std::printf("freeze refused: %s\n", error.c_str());
        return;
    }
    CHECK(frozen->program.anyEdited);
    CHECK(frozen->program.lastTime == held);
    RigExecFrameInputs at;
    CHECK(RigExecSampleFrameInputs(evaluator, held, {}, &at, &error));
    RigExecBackgroundScheduler scheduler;
    bool ran = false;
    const RigExecRigPose warmed =
        RunWarmingJob(&evaluator, rig, frozen, at, &scheduler, &ran);
    CHECK(ran);
    const RigExecRigPose live = evaluator.Evaluate(held);
    CHECK(live.valid);
    CheckPosesBitIdentical("rewarm at the snapshot's lastTime after a "
                           "fallback",
                           live, warmed);
    // Sensitivity: the edit moved the pose, so a snapshot that skipped the
    // constraint would have been caught above.
    RigExecRigPose moved;
    RigExecComparePoses(before, live, &moved);
    CHECK(moved.bakedParityMismatches != 0);
}

// A snapshot frozen BEFORE a routed value edit, which the live program has
// since run and consumed: the snapshot's own history never saw the edit, and
// the live program's pending flags are gone, so only the per-index edit
// counts can say which steps a job cloned from it owes. The registry keeps
// such a snapshot while the avar region is unchanged and asks
// RigExecFrozenSnapshotOwesLiveEdits; the patch it then takes re-runs the
// constraint in the job, at the snapshot's own lastTime where nothing else
// would dirty it.
void
TestAStandingSnapshotPatchedAfterALiveRunCarriesTheEdit(
    const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/aimtest.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig("/World/RigRoot");
    const SdfPath aim("/World/RigRoot/Movers/RigExecAimConstraint1");
    const UsdAttribute weight = stage->GetPrimAtPath(aim).CreateAttribute(
        TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float);
    CHECK(weight.Set(1.0f, UsdTimeCode(1.0)));
    CHECK(weight.Set(0.5f, UsdTimeCode(100.0)));
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    const UsdTimeCode held(50.0);
    CHECK(evaluator.Evaluate(held).valid);
    CHECK(evaluator.Evaluate(held).valid);
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program != nullptr);
    if (!program) {
        return;
    }
    std::shared_ptr<const RigExecFrozenProgram> standing;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &standing, &error));
    if (!standing) {
        std::printf("freeze refused: %s\n", error.c_str());
        return;
    }
    CHECK(!RigExecFrozenSnapshotOwesLiveEdits(*standing, *program));
    const uint64_t avarDigest = RigExecFrozenAvarRegionDigest(*program);

    CHECK(weight.Set(0.25f, UsdTimeCode(100.0)));
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Edited);
    const RigExecRigPose live = evaluator.Evaluate(held);
    CHECK(live.valid);
    CHECK(!program->GetStepGraph().anyEdited);
    // The edit moved no avar: the digest alone would keep the snapshot.
    CHECK(RigExecFrozenAvarRegionDigest(*program) == avarDigest);
    CHECK(RigExecFrozenSnapshotOwesLiveEdits(*standing, *program));

    RigExecFrameInputs at;
    CHECK(RigExecSampleFrameInputs(evaluator, held, {}, &at, &error));
    RigExecBackgroundScheduler scheduler;
    // Sensitivity: the unpatched snapshot skips the constraint and warms
    // the pre-edit pose.
    bool ran = false;
    const RigExecRigPose unpatched =
        RunWarmingJob(&evaluator, rig, standing, at, &scheduler, &ran);
    CHECK(ran);
    RigExecRigPose stale;
    RigExecComparePoses(live, unpatched, &stale);
    CHECK(stale.bakedParityMismatches != 0);

    std::shared_ptr<const RigExecFrozenProgram> patched;
    CHECK(RigExecPatchFrozenAvarConstants(*standing, *program, &patched,
                                          &error));
    CHECK(patched != nullptr);
    if (!patched) {
        return;
    }
    CHECK(patched->program.anyEdited);
    CHECK(!RigExecFrozenSnapshotOwesLiveEdits(*patched, *program));
    ran = false;
    const RigExecRigPose warmed =
        RunWarmingJob(&evaluator, rig, patched, at, &scheduler, &ran);
    CHECK(ran);
    CheckPosesBitIdentical("patched standing snapshot after a live run",
                           evaluator.Evaluate(held), warmed);
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

// Surface projectors whose providers are constrained controls
// (tests/fixtures/projector_spaces.usda). The projector frames are built
// from base and final of rigExec:space, rigExec:sources and
// rigExec:sourceSpace, so the baked program must publish both matrices for
// them although nothing else reads them. The bake must run at every frame,
// live baked and a warming job must match the dynamic walk exactly, and both
// primvars must move. Both primvars must also differ from a walk with the
// constraints disabled, so a final matrix replaced by its base cannot match.
void
TestProjectorSpacesMatchDynamic(const std::string &examplesDir)
{
    const std::string stagePath =
        examplesDir + "/../tests/fixtures/projector_spaces.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig("/ProjectorAsset/Rig");
    const SdfPath targets[2] = {
        SdfPath("/ProjectorAsset/Geom/Ball.primvars:inSpace"),
        SdfPath("/ProjectorAsset/Geom/Ball.primvars:fromSource")};
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecRigEvaluator walk(stage, rig);
    CHECK(walk.Compile(&errors));
    walk.SetEvaluationMode(RigExecEvaluationMode::Dynamic);

    // The same rig with every provider's final equal to its base.
    UsdStageRefPtr unconstrainedStage = UsdStage::Open(stagePath);
    CHECK(unconstrainedStage);
    if (!unconstrainedStage) {
        return;
    }
    unconstrainedStage->SetEditTarget(unconstrainedStage->GetSessionLayer());
    for (const char *name : {"SpaceToDriver", "SourceToDriver",
                             "SourceSpaceToDriver"}) {
        const UsdPrim constraint = unconstrainedStage->GetPrimAtPath(
            rig.AppendChild(TfToken("Constraints"))
                .AppendChild(TfToken(name)));
        CHECK(constraint);
        if (!constraint) {
            return;
        }
        CHECK(constraint.GetAttribute(TfToken("inputs:enabled"))
                  .Set(false));
    }
    RigExecRigEvaluator unconstrained(unconstrainedStage, rig);
    CHECK(unconstrained.Compile(&errors));
    unconstrained.SetEvaluationMode(RigExecEvaluationMode::Dynamic);

    GfMatrix4d previous[2];
    // Live baked against the walk; true when the bake ran rather than
    // falling back to the walk it is compared with.
    const auto checkLive = [&](double frame, const RigExecRigPose &warmed) {
        const size_t before = evaluator.GetBakedGenerationCount();
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid);
        const bool ran = evaluator.GetBakedGenerationCount() == before + 1;
        if (!ran) {
            std::printf("FAIL projector spaces frame %g: the bake did not "
                        "run\n", frame);
        }
        CHECK(ran);
        const RigExecRigPose dynamic = walk.Evaluate(UsdTimeCode(frame));
        CHECK(dynamic.valid);
        CheckPosesBitIdentical(
            TfStringPrintf("projector spaces baked frame %g", frame).c_str(),
            dynamic, live);
        if (warmed.valid) {
            CheckPosesBitIdentical(
                TfStringPrintf("projector spaces warmed frame %g", frame)
                    .c_str(),
                dynamic, warmed);
        }
        const RigExecRigPose baseOnly =
            unconstrained.Evaluate(UsdTimeCode(frame));
        CHECK(baseOnly.valid);
        for (int k = 0; k < 2; ++k) {
            const auto found = dynamic.movedProperties.find(targets[k]);
            const auto foundBase = baseOnly.movedProperties.find(targets[k]);
            if (found == dynamic.movedProperties.end() ||
                !found->second.IsHolding<GfMatrix4d>() ||
                foundBase == baseOnly.movedProperties.end() ||
                !foundBase->second.IsHolding<GfMatrix4d>()) {
                CHECK(false);
                continue;
            }
            const GfMatrix4d matrix = found->second.UncheckedGet<GfMatrix4d>();
            if (frame > 1.0) {
                CHECK(matrix != previous[k]);
            }
            previous[k] = matrix;
            const bool revised =
                matrix != foundBase->second.UncheckedGet<GfMatrix4d>();
            if (!revised) {
                std::printf("FAIL projector spaces frame %g: %s ignores its "
                            "providers' final matrices\n",
                            frame, targets[k].GetText());
            }
            CHECK(revised);
        }
    };
    checkLive(1.0, RigExecRigPose());
    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    std::string error;
    for (double frame : {4.0, 7.0, 10.0}) {
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("FAIL projector spaces freeze refused: %s\n",
                        error.c_str());
            CHECK(false);
            return;
        }
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &inputs, &error));
        bool ran = false;
        const RigExecRigPose warmed =
            RunWarmingJob(&evaluator, rig, frozen, inputs, &scheduler, &ran);
        CHECK(ran);
        checkLive(frame, warmed);
    }
}

// tests/fixtures/frame_record_fallbacks.usda, frozen: AtPrim read phases on
// rigExec:transform resolve through the worker's own FrameMatrix records. A
// warming job at frames 4 (C2 disabled, so its record is X as C1 left it)
// and 8 matches live baked and the dynamic walk bit for bit, and the frozen
// program carries the live one's records.
void
TestFrameRecordFallbacksFreeze(const std::string &examplesDir)
{
    const char *label = "frame record fallbacks";
    const std::string stagePath =
        examplesDir + "/../tests/fixtures/frame_record_fallbacks.usda";
    const SdfPath rig("/RecordAsset/Rig");
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    std::vector<std::string> errors;
    std::string error;
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecRigEvaluator walk(stage, rig);
    CHECK(walk.Compile(&errors));
    walk.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(walk.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    for (const double frame : {4.0, 8.0}) {
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("FAIL %s: freeze refused: %s\n", label,
                        error.c_str());
            ++failures;
            return;
        }
        CHECK(frozen->program.frameRecords.size() == 3);
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &inputs, &error));
        bool ran = false;
        const RigExecRigPose warmed =
            RunWarmingJob(&evaluator, rig, frozen, inputs, &scheduler, &ran);
        CHECK(ran);
        const size_t before = evaluator.GetBakedGenerationCount();
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(evaluator.GetBakedGenerationCount() == before + 1);
        const RigExecRigPose dynamic = walk.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid && dynamic.valid);
        const std::string where = TfStringPrintf("%s frame %g", label, frame);
        CheckPosesBitIdentical((where + " warmed").c_str(), dynamic, warmed);
        CheckPosesBitIdentical((where + " baked").c_str(), dynamic, live);
    }
}

// tests/fixtures/solver_checkpoint.usda, frozen: AtPrim read phases that name
// solver checkpoints resolve through the worker's own FrameMatrix records,
// LegBlend's skipped one included. Warming jobs at frames 5 and 9 match live
// baked and the dynamic walk bit for bit, and the frozen program carries the
// live one's three records.
void
TestSolverCheckpointFreezes(const std::string &examplesDir)
{
    const char *label = "solver checkpoint";
    const std::string stagePath =
        examplesDir + "/../tests/fixtures/solver_checkpoint.usda";
    const SdfPath rig("/CheckpointAsset/Rig");
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    std::vector<std::string> errors;
    std::string error;
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecRigEvaluator walk(stage, rig);
    CHECK(walk.Compile(&errors));
    walk.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(walk.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    for (const double frame : {5.0, 9.0}) {
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("FAIL %s: freeze refused: %s\n", label,
                        error.c_str());
            ++failures;
            return;
        }
        CHECK(frozen->program.frameRecords.size() == 3);
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &inputs, &error));
        bool ran = false;
        const RigExecRigPose warmed =
            RunWarmingJob(&evaluator, rig, frozen, inputs, &scheduler, &ran);
        CHECK(ran);
        const size_t before = evaluator.GetBakedGenerationCount();
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(evaluator.GetBakedGenerationCount() == before + 1);
        const RigExecRigPose dynamic = walk.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid && dynamic.valid);
        const std::string where = TfStringPrintf("%s frame %g", label, frame);
        CheckPosesBitIdentical((where + " warmed").c_str(), dynamic, warmed);
        CheckPosesBitIdentical((where + " baked").c_str(), dynamic, live);
    }
}

// tests/fixtures/preceding_own_chain.usda, frozen: Echo reads its own
// chain at `preceding`, bound to the version entering it (2, after Lift and
// Settle) in the frozen program as in the live one. Warming jobs at frames 5
// and 9 match live baked and the dynamic walk bit for bit. So does a
// partial cone that runs Echo and the box's status sweep but not Settle's
// fuse: frozen with LiftCtl raised, it re-runs Echo for a drag on Echo's
// own weight, reading the version Settle left in the snapshot.
void
TestPrecedingOwnChainFreezes(const std::string &examplesDir)
{
    const char *label = "preceding own chain";
    const std::string stagePath =
        examplesDir + "/../tests/fixtures/preceding_own_chain.usda";
    const SdfPath rig("/PrecedingAsset/Rig");
    const SdfPath box("/PrecedingAsset/Geom/Box.points");
    const SdfPath echo("/PrecedingAsset/Rig/Movers/Echo");
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    std::vector<std::string> errors;
    std::string error;
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecRigEvaluator walk(stage, rig);
    CHECK(walk.Compile(&errors));
    walk.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(walk.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    for (const double frame : {5.0, 9.0}) {
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("FAIL %s: freeze refused: %s\n", label,
                        error.c_str());
            ++failures;
            return;
        }
        int bound = 0;
        for (size_t c = 0; c < frozen->program.chains.size(); ++c) {
            const auto &chain = frozen->program.chains[c];
            if (chain.target != box) {
                continue;
            }
            for (const auto &revision : chain.revisions) {
                if (revision.moverPath != echo) {
                    continue;
                }
                for (const RigExecBakedPointsBinding &binding :
                         revision.pointBindings) {
                    CHECK(binding.input == box);
                    CHECK(binding.candidates.size() == 1);
                    if (binding.candidates.size() == 1) {
                        CHECK(binding.candidates[0].chain == int(c));
                        CHECK(binding.candidates[0].version == 2);
                    }
                    ++bound;
                }
            }
        }
        CHECK(bound == 1);
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &inputs, &error));
        bool ran = false;
        const RigExecRigPose warmed =
            RunWarmingJob(&evaluator, rig, frozen, inputs, &scheduler, &ran);
        CHECK(ran);
        const size_t before = evaluator.GetBakedGenerationCount();
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(evaluator.GetBakedGenerationCount() == before + 1);
        const RigExecRigPose dynamic = walk.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid && dynamic.valid);
        const std::string where = TfStringPrintf("%s frame %g", label, frame);
        CheckPosesBitIdentical((where + " warmed").c_str(), dynamic, warmed);
        CheckPosesBitIdentical((where + " baked").c_str(), dynamic, live);
    }

    // The box's points, compared by their bits.
    const auto sameBox = [&](const char *what, const RigExecRigPose &expected,
                             const RigExecRigPose &got) {
        const auto e = expected.movedProperties.find(box);
        const auto g = got.movedProperties.find(box);
        const bool same =
            got.valid && e != expected.movedProperties.end() &&
            g != got.movedProperties.end() &&
            e->second.IsHolding<VtVec3fArray>() &&
            g->second.IsHolding<VtVec3fArray>() &&
            e->second.UncheckedGet<VtVec3fArray>().size() ==
                g->second.UncheckedGet<VtVec3fArray>().size() &&
            std::memcmp(e->second.UncheckedGet<VtVec3fArray>().cdata(),
                        g->second.UncheckedGet<VtVec3fArray>().cdata(),
                        e->second.UncheckedGet<VtVec3fArray>().size() *
                            sizeof(GfVec3f)) == 0;
        if (!same) {
            std::printf("FAIL %s: %s publishes another box than live\n",
                        label, what);
            ++failures;
        }
    };
    // A partial cone declines a vector with chain-resolved samples, which
    // LiftCtl's spline gives; here it is a constant, raised, in the session
    // layer.
    const SdfPath settle("/PrecedingAsset/Rig/Movers/Settle");
    UsdStageRefPtr raised = UsdStage::Open(stagePath);
    CHECK(raised);
    if (!raised) {
        return;
    }
    raised->SetEditTarget(raised->GetSessionLayer());
    CHECK(raised->GetAttributeAtPath(
                    SdfPath("/PrecedingAsset/Rig/Controls/LiftCtl.avars:ty"))
              .Set(0.75));
    RigExecRigEvaluator coned(raised, rig);
    CHECK(coned.Compile(&errors));
    coned.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecRigEvaluator coneWalk(raised, rig);
    CHECK(coneWalk.Compile(&errors));
    coneWalk.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    RigExecValueOverride weight;
    weight.prim = echo;
    weight.attribute = TfToken("inputs:defaultWeight");
    weight.value = VtValue(0.5f);
    const UsdTimeCode at9(9.0);
    const RigExecRigPose lifted = coned.Evaluate(at9);
    CheckPosesBitIdentical("preceding own chain lifted baked",
                           coneWalk.Evaluate(at9), lifted);
    if (!coned.GetBakedProgram()) {
        std::printf("FAIL %s: the raised rig does not bake\n", label);
        ++failures;
        return;
    }
    const RigExecBakedProgramImpl &B =
        coned.GetBakedProgram()->GetStepGraph();
    std::shared_ptr<const void> liftedSlots;
    size_t slotBytes = 0;
    CHECK(RigExecCapturePartialSlots(B, &liftedSlots, &slotBytes));
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(coned, &frozen, &error));
    RigExecFrameInputs liftedInputs, weightedInputs;
    CHECK(RigExecSampleFrameInputs(coned, at9, noOverrides, &liftedInputs,
                                   &error));
    CHECK(RigExecSampleFrameInputs(coned, at9, {weight}, &weightedInputs,
                                   &error));
    CHECK(!liftedInputs.HasChainResolvedInputs());
    CHECK(!weightedInputs.HasChainResolvedInputs());
    if (!frozen || !liftedSlots) {
        std::printf("FAIL %s: no snapshot (%s)\n", label, error.c_str());
        ++failures;
        return;
    }
    const size_t clusters = B.clustering.clusters.size();
    RigExecBakedClusterSet all;
    all.SetAll(clusters);
    const RigExecPartialRunResult full = RigExecRunPartialCone(
        *frozen, liftedInputs, liftedSlots, lifted, all, 0);
    CHECK(full.completed);
    sameBox("an every-cluster cone under the lift", lifted, full.pose);

    // Echo's steps and the box's status sweep, and where Settle's fuse is.
    RigExecBakedClusterSet plan;
    plan.Resize(clusters);
    std::vector<int> echoClusters;
    int settleFuse = -1;
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::ChainStatus) {
            if (B.chains[size_t(step.object)].target == box) {
                CHECK(step.cluster >= 0);
                plan.Set(step.cluster);
            }
            continue;
        }
        if (step.kind != RigExecBakedStepKind::InfluenceFold &&
            step.kind != RigExecBakedStepKind::RevisionStatic &&
            step.kind != RigExecBakedStepKind::RevisionChunk &&
            step.kind != RigExecBakedStepKind::RevisionFuse) {
            continue;
        }
        const auto &[c, r] = B.revisionIndex[size_t(step.object)];
        const SdfPath &mover = B.chains[size_t(c)].revisions[size_t(r)].moverPath;
        if (mover == echo) {
            CHECK(step.cluster >= 0);
            plan.Set(step.cluster);
            echoClusters.push_back(step.cluster);
        } else if (mover == settle &&
                   step.kind == RigExecBakedStepKind::RevisionFuse) {
            settleFuse = step.cluster;
        }
    }
    CHECK(!echoClusters.empty());
    if (settleFuse < 0 || plan.Test(settleFuse)) {
        std::printf("FAIL %s: Settle's fuse (cluster %d) is not outside the "
                    "cone\n", label, settleFuse);
        ++failures;
        return;
    }
    coned.SetInteractiveOverrides({weight});
    coneWalk.SetInteractiveOverrides({weight});
    const RigExecRigPose weighted = coned.Evaluate(at9);
    CheckPosesBitIdentical("preceding own chain weighted baked",
                           coneWalk.Evaluate(at9), weighted);
    const RigExecPartialRunResult cone = RigExecRunPartialCone(
        *frozen, weightedInputs, full.slots, full.pose, plan, 0);
    CHECK(cone.completed);
    for (const int cluster : echoClusters) {
        CHECK(std::find(cone.executedClusters.begin(),
                        cone.executedClusters.end(),
                        cluster) != cone.executedClusters.end());
    }
    CHECK(std::find(cone.executedClusters.begin(),
                    cone.executedClusters.end(),
                    settleFuse) == cone.executedClusters.end());
    sameBox("a cone over Echo alone", weighted, cone.pose);
    // The weight moved the box, so the cone ran Echo rather than keeping
    // what the snapshot held.
    CHECK(weighted.movedProperties.count(box) &&
          lifted.movedProperties.count(box) &&
          weighted.movedProperties.at(box) != lifted.movedProperties.at(box));
}

// tests/fixtures/volume_placements.usda, frozen. As shipped, StripSmooth's
// field is current-phase (SphereA reads its weightTarget at `preceding`),
// which the freeze refuses by name. With SphereA's read phase set to `base`
// the rig freezes, and a warming job matches live baked and the dynamic walk
// bit for bit, including both volumes' weightFrames: SphereB's is the final
// placement BAim leaves, which differs from its base. Every frozen path
// publishes live's weightFrames, two placement steps and both phases.
void
TestVolumePlacementsFixtureFreezes(const std::string &examplesDir)
{
    const char *label = "volume placements";
    const std::string stagePath =
        examplesDir + "/../tests/fixtures/volume_placements.usda";
    const SdfPath rig("/PlacementAsset/Rig");
    const SdfPath sphereA("/PlacementAsset/Rig/Joints/A/SphereA");
    const SdfPath sphereB("/PlacementAsset/Rig/Joints/B/SphereB");
    std::vector<std::string> errors;
    std::string error;
    {
        UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage);
        if (!stage) {
            return;
        }
        RigExecRigEvaluator evaluator(stage, rig);
        CHECK(evaluator.Compile(&errors));
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        CHECK(evaluator.GetBakedGenerationCount() == 1);
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        CHECK(!RigExecFreezeProgram(evaluator, &frozen, &error));
        CHECK(!frozen);
        const std::string expected =
            "revision /PlacementAsset/Rig/GeometryMovers/StripSmooth "
            "measures its weight field against the current phase, which "
            "resolves through the live oracle";
        if (error.find(expected) == std::string::npos) {
            std::printf("FAIL %s: the freeze refusal is \"%s\"\n", label,
                        error.c_str());
            ++failures;
        }
    }

    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdPrim sphere = stage->GetPrimAtPath(sphereA);
    CHECK(sphere);
    if (!sphere) {
        return;
    }
    CHECK(sphere.GetRelationship(TfToken("rigExec:weightTarget"))
              .SetMetadata(TfToken("rigExecReadPhase"), std::string("base")));
    RigExecRigEvaluator evaluator(stage, rig);
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecRigEvaluator walk(stage, rig);
    CHECK(walk.Compile(&errors));
    walk.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    // Both from frame 1, so their work counters compare at 5 and 10.
    CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(walk.Evaluate(UsdTimeCode(1.0)).valid);

    RigExecBackgroundScheduler scheduler;
    std::vector<RigExecValueOverride> noOverrides;
    for (const double frame : {5.0, 10.0}) {
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("FAIL %s: freeze refused: %s\n", label,
                        error.c_str());
            ++failures;
            return;
        }
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       noOverrides, &inputs, &error));
        bool ran = false;
        const RigExecRigPose warmed =
            RunWarmingJob(&evaluator, rig, frozen, inputs, &scheduler, &ran);
        CHECK(ran);
        const size_t before = evaluator.GetBakedGenerationCount();
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(evaluator.GetBakedGenerationCount() == before + 1);
        const RigExecRigPose dynamic = walk.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid && dynamic.valid);
        const std::string where = TfStringPrintf("%s frame %g", label, frame);
        CheckPosesBitIdentical((where + " warmed").c_str(), live, warmed);
        CheckPosesBitIdentical((where + " baked").c_str(), dynamic, live);
        CHECK(warmed.weightFrames.size() == 2);
        CHECK(warmed.weightFrames.count(sphereA) == 1);
        CHECK(warmed.weightFrames.count(sphereB) == 1);
    }
    for (const double frame : {3.0, 7.0}) {
        const std::string where =
            TfStringPrintf("%s, base phase, frame %g", label, frame);
        CheckFrozenWeightFramesMatchLive(&evaluator, rig, UsdTimeCode(frame),
                                         where.c_str());
        // Not vacuous: SphereB publishes its final placement, which is not
        // the placement of its base frame.
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        const RigExecBakedProgramImpl &B =
            evaluator.GetBakedProgram()->GetStepGraph();
        const auto slot = B.index.find(sphereB);
        const auto published = live.weightFrames.find(sphereB);
        CHECK(slot != B.index.end() && published != live.weightFrames.end());
        if (slot == B.index.end() || published == live.weightFrames.end() ||
            size_t(slot->second) >= B.baseLast.size()) {
            continue;
        }
        const GfMatrix4d basePlacement = RigExecVolumePlacement(
            B.base[size_t(B.baseLast[size_t(slot->second)])]);
        CHECK(published->second != basePlacement);
    }
}

// The frozen VolumePlacements step places through the shared gate
// (RigExecVolumePlacement): a final frame flagged valid and non-degenerate
// but holding a NaN point places at the identity, and frozen publishes what
// the live step publishes for the same frame. The step bodies run on clones
// of 11_VolumeWeights' program whose fin entry for one volume is poisoned
// directly, so the step reads exactly a frame flagged valid and
// non-degenerate yet non-finite, independent of what upstream compose and
// commit steps would make of a non-finite authored value.
void
TestFrozenVolumePlacementUsesTheSharedGate(const std::string &examplesDir)
{
    const char *label = "frozen volume placement gate";
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
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(1001.0)).valid);
    const UsdTimeCode time(1012.0);
    const RigExecRigPose live = evaluator.Evaluate(time);
    CHECK(live.valid);
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();

    // The first published volume and its VolumePlacements step.
    size_t slot = B.placedVolumes.size();
    for (size_t i = 0; i < B.placedVolumes.size(); ++i) {
        if (B.placedVolumes[i]) {
            slot = i;
            break;
        }
    }
    size_t stepIndex = B.steps.size();
    for (size_t k = 0; k < B.steps.size(); ++k) {
        if (B.steps[k].kind == RigExecBakedStepKind::VolumePlacements &&
            B.steps[k].object == int(slot)) {
            stepIndex = k;
        }
    }
    CHECK(slot < B.placedVolumes.size() && stepIndex < B.steps.size() &&
          slot < B.finLast.size());
    if (slot >= B.placedVolumes.size() || stepIndex >= B.steps.size() ||
        slot >= B.finLast.size()) {
        return;
    }
    const SdfPath volume = B.paths[slot];
    const size_t finIndex = size_t(B.finLast[slot]);
    // Not vacuous: live places this volume away from the identity.
    const auto published = live.weightFrames.find(volume);
    CHECK(published != live.weightFrames.end() &&
          published->second != GfMatrix4d(1.0));

    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    RigExecFrameInputs inputs;
    CHECK(RigExecSampleFrameInputs(evaluator, time, {}, &inputs, &error));
    if (!frozen) {
        std::printf("FAIL %s: freeze refused: %s\n", label, error.c_str());
        ++failures;
        return;
    }

    // One coordinate made non-finite, once as a NaN and once as +inf, so
    // both halves of the finite check are exercised.
    struct Poison {
        double value;
        const char *what;
    };
    const Poison poisons[] = {
        {std::numeric_limits<double>::quiet_NaN(), "a NaN final frame"},
        {std::numeric_limits<double>::infinity(), "an infinite final frame"},
    };
    for (const Poison &poison : poisons) {
        // The frozen worker, as a job clones it, with the poisoned final
        // frame.
        auto worker = std::make_unique<frozenDetail::_FrozenWorker>();
        frozenDetail::_CloneImpl(frozen->program, &worker->B);
        CHECK(finIndex < worker->B.fin.size() &&
              stepIndex < worker->B.steps.size());
        if (finIndex >= worker->B.fin.size() ||
            stepIndex >= worker->B.steps.size()) {
            return;
        }
        RigExecPointFrame poisoned = worker->B.fin[finIndex];
        poisoned.points[1][0] = poison.value;
        // The flags pass a validity-only gate, which would place at a
        // non-finite matrix.
        CHECK(poisoned.IsValid() && !poisoned.IsDegenerate());
        worker->B.fin[finIndex] = poisoned;
        const std::map<SdfPath, size_t> index;
        CHECK(frozenDetail::_FrozenStepBody(worker.get(),
                                            &worker->B.steps[stepIndex],
                                            index, inputs, time));
        std::map<SdfPath, GfMatrix4d> frozenFrames;
        RigExecBakedPublishVolumePlacements(worker->B, &frozenFrames);
        const auto frozenEntry = frozenFrames.find(volume);
        CHECK(frozenEntry != frozenFrames.end());
        if (frozenEntry != frozenFrames.end() &&
            frozenEntry->second != GfMatrix4d(1.0)) {
            std::printf("FAIL %s: %s publishes a non-identity placement for "
                        "%s\n",
                        label, volume.GetText(), poison.what);
            ++failures;
        }

        // Live's step body and publication over the same frame.
        auto liveCopy = std::make_unique<RigExecBakedProgramImpl>();
        frozenDetail::_CloneImpl(B, liveCopy.get());
        liveCopy->fin[finIndex] = poisoned;
        RigExecBakedRunWeightStep(liveCopy.get(),
                                  &liveCopy->steps[stepIndex], time);
        std::map<SdfPath, GfMatrix4d> liveFrames;
        RigExecBakedPublishVolumePlacements(*liveCopy, &liveFrames);
        CHECK(liveFrames.count(volume) == 1 &&
              liveFrames.at(volume) == GfMatrix4d(1.0));
        CheckWeightFramesEqual(label, poison.what, liveFrames, frozenFrames);
    }
}

// A rig whose provider ladder recomposes, for the frozen ladder cases below:
// the stage, a session-layer edit authored before Compile, the frames its
// rests move over, and a ladder channel to drag at the last of them.
struct _RecomposingLadderCase {
    const char *label = "";
    std::string stagePath;
    SdfPath rig;
    std::function<void(const UsdStageRefPtr &)> edit;
    std::vector<double> frames;
    SdfPath dragPrim;
    TfToken dragAttribute;
    double dragValues[2] = {0.0, 0.0};
    // Whether a ladder channel is read per frame (Build's ladderVarying):
    // the rigs the freeze refused before the worker ran the rest tier.
    bool varying = true;
    // A joint whose final frame the moved rest moves.
    SdfPath watched;
};

// A frozen job over a ladder that recomposes equals live, bit for bit: at
// each frame, frozen from the live run before it (so the job's ladder leaves
// differ from the clone's and its rest and ladder ops re-run), then under
// two values of a drag on a ladder channel at a held frame, and after its
// release. Each job is accepted, not declined.
void
CheckRecomposingLadderFreezes(const _RecomposingLadderCase &c)
{
    UsdStageRefPtr stage = UsdStage::Open(c.stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    if (c.edit) {
        stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
        c.edit(stage);
    }
    std::vector<std::string> errors;
    std::string error;
    RigExecRigEvaluator evaluator(stage, c.rig);
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(c.frames.front())).valid);
    CHECK(evaluator.GetBakedProgram() != nullptr);
    if (!evaluator.GetBakedProgram()) {
        return;
    }
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();
    CHECK(B.ladderVarying == c.varying);
    CHECK(RigExecCanFreezeProgram(evaluator, &error));

    RigExecBackgroundScheduler scheduler;
    const auto warm = [&](const std::string &where, double frame,
                          const std::vector<RigExecValueOverride> &overrides)
        -> RigExecRigPose {
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        if (!frozen) {
            std::printf("FAIL %s: freeze refused: %s\n", where.c_str(),
                        error.c_str());
            ++failures;
            return RigExecRigPose();
        }
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                       overrides, &inputs, &error));
        const RigExecRigPose warmed = RunWarmingJob(
            &evaluator, c.rig, frozen, inputs, &scheduler, nullptr);
        CheckJobAccepted(where.c_str(), inputs, warmed);
        evaluator.SetInteractiveOverrides(overrides);
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid);
        CheckPosesBitIdentical((where + " warmed").c_str(), live, warmed);
        return warmed;
    };
    // A rest move shows in the joint's final frame or, where its default
    // space carries the move, in its rest-to-pose matrix.
    const auto watchedFrame = [&c](const RigExecRigPose &pose) {
        const auto found = pose.jointFramesFinal.find(c.watched);
        CHECK(found != pose.jointFramesFinal.end());
        return found == pose.jointFramesFinal.end() ? RigExecPointFrame()
                                                    : found->second;
    };
    const auto watchedMatrix = [&c](const RigExecRigPose &pose) {
        const auto found = pose.jointMatricesFinal.find(c.watched);
        CHECK(found != pose.jointMatricesFinal.end());
        return found == pose.jointMatricesFinal.end() ? GfMatrix4d(1.0)
                                                      : found->second;
    };
    const auto moved = [&](const std::string &what, const RigExecRigPose &a,
                           const RigExecRigPose &b) {
        const RigExecPointFrame x = watchedFrame(a), y = watchedFrame(b);
        if (x.flags == y.flags && x.points == y.points &&
            watchedMatrix(a) == watchedMatrix(b)) {
            std::printf("FAIL %s %s: %s did not move\n", c.label,
                        what.c_str(), c.watched.GetText());
            ++failures;
        }
    };

    // Forward over the frames, then back to the first.
    std::vector<double> frames = c.frames;
    frames.push_back(c.frames.front());
    std::vector<RigExecRigPose> warmed;
    for (const double frame : frames) {
        warmed.push_back(warm(TfStringPrintf("%s frame %g", c.label, frame),
                              frame, {}));
    }
    if (c.varying) {
        moved("over the frames", warmed.front(), warmed[warmed.size() - 2]);
    }

    // A drag on a ladder channel at the held last frame, moved, released.
    const double held = c.frames.back();
    const auto drag = [&c](double value) {
        return std::vector<RigExecValueOverride>{RigExecValueOverride{
            c.dragPrim, TfToken(), c.dragAttribute, VtValue(value)}};
    };
    const RigExecRigPose settled = warmed[warmed.size() - 2];
    const RigExecRigPose first = warm(
        TfStringPrintf("%s dragged to %g", c.label, c.dragValues[0]), held,
        drag(c.dragValues[0]));
    moved("under the drag", settled, first);
    warm(TfStringPrintf("%s dragged to %g", c.label, c.dragValues[1]), held,
         drag(c.dragValues[1]));
    const RigExecRigPose released =
        warm(TfStringPrintf("%s released", c.label), held, {});
    evaluator.SetInteractiveOverrides({});
    CheckSameReadings(TfStringPrintf("%s released vs settled", c.label)
                          .c_str(),
                      settled, released);

    // One snapshot serving every job, as the warmer's does: frozen from the
    // live run at the first frame, and again from the run under the first
    // drag, so a job's leaves and ladder flags differ from the clone's by
    // more than one run. Values only: the work a job does follows from what
    // its snapshot last ran, not from live's history.
    struct _Job {
        std::string what;
        double frame;
        std::vector<RigExecValueOverride> overrides;
    };
    std::vector<_Job> jobs;
    for (const double frame : frames) {
        jobs.push_back({TfStringPrintf("frame %g", frame), frame, {}});
    }
    jobs.push_back({TfStringPrintf("dragged to %g", c.dragValues[1]), held,
                    drag(c.dragValues[1])});
    jobs.push_back({"released", held, {}});
    const auto sameValues = [&c](const std::string &what,
                                 const RigExecRigPose &live,
                                 const RigExecRigPose &warmed) {
        CheckSameReadings(what.c_str(), live, warmed);
        if (live.jointMatricesFinal != warmed.jointMatricesFinal ||
            live.providerXforms != warmed.providerXforms ||
            live.providerBaseXforms != warmed.providerBaseXforms) {
            std::printf("FAIL %s: joint matrices or provider transforms "
                        "differ\n", what.c_str());
            ++failures;
        }
    };
    const auto fromOneSnapshot =
        [&](const std::string &where, double frame,
            const std::vector<RigExecValueOverride> &overrides) {
            evaluator.SetInteractiveOverrides(overrides);
            CHECK(evaluator.Evaluate(UsdTimeCode(frame)).valid);
            std::shared_ptr<const RigExecFrozenProgram> frozen;
            CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
            if (!frozen) {
                return;
            }
            for (const _Job &job : jobs) {
                const std::string what = TfStringPrintf(
                    "%s, one snapshot %s: %s", c.label, where.c_str(),
                    job.what.c_str());
                RigExecFrameInputs inputs;
                CHECK(RigExecSampleFrameInputs(evaluator,
                                               UsdTimeCode(job.frame),
                                               job.overrides, &inputs,
                                               &error));
                const RigExecRigPose warmed = RunWarmingJob(
                    &evaluator, c.rig, frozen, inputs, &scheduler, nullptr);
                CheckJobAccepted(what.c_str(), inputs, warmed);
                evaluator.SetInteractiveOverrides(job.overrides);
                const RigExecRigPose live =
                    evaluator.Evaluate(UsdTimeCode(job.frame));
                CHECK(live.valid);
                sameValues(what, live, warmed);
            }
        };
    fromOneSnapshot("at the first frame", c.frames.front(), {});
    fromOneSnapshot("under the drag", held, drag(c.dragValues[0]));
    evaluator.SetInteractiveOverrides({});
}

// The tail (examples/01_FkChainTail.usda) with testRigExecEpochRests'
// animated rests on Seg2 -- a time-sampled rest:tx, a rest:space connected
// to an animated driver, a property chain writing rest:space -- plus the
// unedited tail, whose static ladder recomposes only under the drag. Each
// freezes, and every job equals live.
void
TestRecomposingLaddersFreeze(const std::string &examplesDir)
{
    const SdfPath rig("/TailAsset/Rig");
    const SdfPath joint("/TailAsset/Rig/Joints/Seg1/Seg2");
    const SdfPath child = joint.AppendChild(TfToken("Seg3"));
    const std::vector<double> frames{1001.0, 1005.0, 1009.0};
    const auto translation = [](double x) {
        GfMatrix4d m(1.0);
        m.SetTranslateOnly(GfVec3d(x, 0.0, 0.0));
        return m;
    };
    _RecomposingLadderCase base;
    base.stagePath = examplesDir + "/01_FkChainTail.usda";
    base.rig = rig;
    base.frames = frames;
    base.dragPrim = joint;
    base.dragAttribute = TfToken("rest:tx");
    base.dragValues[0] = 1.5;
    base.dragValues[1] = 4.0;
    base.watched = child;

    _RecomposingLadderCase sampled = base;
    sampled.label = "a time-sampled rest";
    sampled.edit = [&](const UsdStageRefPtr &stage) {
        UsdAttribute restTx = stage->GetPrimAtPath(joint).CreateAttribute(
            TfToken("rest:tx"), SdfValueTypeNames->Double);
        restTx.Set(0.0, UsdTimeCode(frames[0]));
        restTx.Set(3.0, UsdTimeCode(frames[1]));
        restTx.Set(6.0, UsdTimeCode(frames[2]));
    };
    CheckRecomposingLadderFreezes(sampled);

    _RecomposingLadderCase connected = base;
    connected.label = "a connected rest space";
    connected.edit = [&](const UsdStageRefPtr &stage) {
        const UsdPrim prim = stage->GetPrimAtPath(joint);
        const UsdAttribute driver = prim.CreateAttribute(
            TfToken("inputs:restSpaceDriver"), SdfValueTypeNames->Matrix4d);
        driver.Set(translation(0.0), UsdTimeCode(frames[0]));
        driver.Set(translation(2.5), UsdTimeCode(frames[1]));
        driver.Set(translation(5.0), UsdTimeCode(frames[2]));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .SetConnections({driver.GetPath()});
    };
    CheckRecomposingLadderFreezes(connected);

    _RecomposingLadderCase chained = base;
    chained.label = "a property chain on a rest";
    chained.edit = [&](const UsdStageRefPtr &stage) {
        const UsdPrim prim = stage->DefinePrim(
            rig.AppendChild(TfToken("Movers"))
                .AppendChild(TfToken("RestOffset")),
            TfToken("RigExecMatrixMathMover"));
        prim.ApplyAPI(TfToken("RigExecMoverAPI"));
        prim.CreateAttribute(TfToken("rigExec:operation"),
                             SdfValueTypeNames->Token)
            .Set(TfToken("multiply"));
        const UsdAttribute value = prim.CreateAttribute(
            TfToken("inputs:value"), SdfValueTypeNames->Matrix4d);
        value.Set(translation(1.0), UsdTimeCode(frames[0]));
        value.Set(translation(2.0), UsdTimeCode(frames[1]));
        value.Set(translation(3.0), UsdTimeCode(frames[2]));
        prim.CreateAttribute(TfToken("inputs:defaultWeight"),
                             SdfValueTypeNames->Float)
            .Set(1.0f);
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({joint.AppendProperty(TfToken("rest:space"))});
    };
    CheckRecomposingLadderFreezes(chained);

    _RecomposingLadderCase still = base;
    still.label = "a static ladder under a drag";
    still.varying = false;
    CheckRecomposingLadderFreezes(still);
}

// tests/fixtures/computed_ik_space.usda with Master's rest keyed (the
// space rest testRigExecSolverBake's TestASpaceRestMoveReachesTheSolve
// moves): the TwoBoneIk arm and the SplineIk tail measure their space from
// Master's rest, so a job whose Solve did not refresh its space rest from
// the rests its own rest ops composed would differ from live at 5 and 10.
void
TestARecomposedSpaceRestFreezes(const std::string &examplesDir)
{
    _RecomposingLadderCase c;
    c.label = "a keyed space rest";
    c.stagePath = examplesDir + "/../tests/fixtures/computed_ik_space.usda";
    c.rig = SdfPath("/IkSpaceAsset/Rig");
    const SdfPath master("/IkSpaceAsset/Rig/Controls/Master");
    c.edit = [&master](const UsdStageRefPtr &stage) {
        const UsdPrim prim = stage->GetPrimAtPath(master);
        const auto key = [&prim](const char *name, double last) {
            UsdAttribute a = prim.CreateAttribute(TfToken(name),
                                                  SdfValueTypeNames->Double);
            a.Set(0.0, UsdTimeCode(1.0));
            a.Set(last, UsdTimeCode(10.0));
        };
        key("rest:ry", 20.0);
        key("rest:tx", 3.0);
    };
    c.frames = {1.0, 5.0, 10.0};
    c.dragPrim = master;
    c.dragAttribute = TfToken("rest:ry");
    c.dragValues[0] = 15.0;
    c.dragValues[1] = 25.0;
    c.watched = SdfPath("/IkSpaceAsset/Rig/Joints/Shoulder/Elbow/Wrist");
    CheckRecomposingLadderFreezes(c);

    // Unkeyed, the ladder recomposes only under the drag, so the Solve
    // refreshes on the drag and once after its release (ladderDisturbed),
    // including in a job frozen while the drag stood.
    _RecomposingLadderCase still = c;
    still.label = "a static space rest under a drag";
    still.edit = nullptr;
    still.varying = false;
    CheckRecomposingLadderFreezes(still);
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
    TestNestedSpaceSwitchesWarmBitIdentical();
    TestAHeldFrameJobServesItsOwnDrag();
    TestChainedRigWarmsBitIdentical();
    TestPhasedReadsWarmBitIdentical();
    TestBlinkDragWarmsAsReleased();
    TestBlinkDragEdgesWarmBitIdentical();
    TestAPropertyEnvelopeRefusesTheFreeze();
    TestPhasedReadDragRulesFrozen();
    TestAChainTargetOwnValueKeyIsDistinct();
    TestAChainMoverInputMovesTheDigest();
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
        TestKeyedOperatorInputsWarmBitIdentical(argv[1]);
        TestStackAnimWarmsBitIdenticalAtSweepDistance(argv[1]);
        TestBlendFaceWarmsBitIdentical(argv[1]);
        TestLatticeStackWarmsBitIdentical(argv[1]);
        TestFrozenAssemblesFromLeaves(argv[1]);
        TestSurfaceDrapeWarmsBitIdentical(argv[1]);
        TestRibbonSpineWarmsBitIdentical(argv[1]);
        TestArmRigWarmsBitIdentical(argv[1]);
        TestReadPhasesWarmBitIdentical(argv[1]);
        TestConnectionReadPhasesWarmBitIdentical(argv[1]);
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
        TestAFrozenRewarmAfterAFallbackCarriesTheEdit(argv[1]);
        TestAStandingSnapshotPatchedAfterALiveRunCarriesTheEdit(argv[1]);
        TestVolumeWeightsBurstMatchesPlain(argv[1]);
        TestFrozenWeightFramesMatchLive(argv[1]);
        TestUnresolvableTargetDeclinesSampling(argv[1]);
        TestProjectorSpacesMatchDynamic(argv[1]);
        TestVolumePlacementsFixtureFreezes(argv[1]);
        TestFrozenVolumePlacementUsesTheSharedGate(argv[1]);
        TestFrameRecordFallbacksFreeze(argv[1]);
        TestSolverCheckpointFreezes(argv[1]);
        TestPrecedingOwnChainFreezes(argv[1]);
        TestRecomposingLaddersFreeze(argv[1]);
        TestARecomposedSpaceRestFreezes(argv[1]);
    } else {
        std::printf("skipping the biped (no examples directory given)\n");
    }
    Test9MeshWarmsBitIdentical();
    TestIterativeMoversWarmBitIdentical();
    TestFrozenWholeRunSkippingVolumePlacementsPublishesLive();
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
