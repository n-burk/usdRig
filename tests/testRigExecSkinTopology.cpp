// The skin layout is resolved once per binding epoch and shared, so these
// are the cases where "once per epoch" must not mean "once, ever":
//  - a weight-paint edit is a VALUE edit. It does not change the structure
//    digest, so no new epoch begins and nothing recompiles; the only thing
//    standing between the artist's new weights and a stale deformation is
//    the change notice dropping the cached layout.
//  - a layout that is time-varying, connected, or written by a property
//    chain is not epoch state at all. Compile has to refuse to cache it,
//    and the rig has to keep deforming correctly frame by frame.
//  - an interactive override is a value the static reads must prefer over
//    the stage, and the layout is read through exactly that route.
// With --baked <examples>, the baked program's own layout handles instead.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecInputActionsAdapters.h"

#include "pxr/base/arch/env.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;

static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

namespace {

constexpr size_t kPointCount = 4;
const SdfPath kTarget("/Asset/Geom/Mesh.points");
const SdfPath kSkin("/Asset/Rig/Movers/Skin");

VtVec3fArray
BasePoints()
{
    VtVec3fArray points(kPointCount);
    for (size_t i = 0; i < kPointCount; ++i) {
        points[i] = GfVec3f(float(i), float(i) * 2.0f, float(i) * 3.0f);
    }
    return points;
}

// One mesh, two controls that translate along x and y, and a skin mover with
// two influence slots per point. With indices {0, 1} everywhere the kernel
// reduces to p + w0 * (10, 0, 0) + w1 * (0, 20, 0), which is checkable by
// hand at every point.
UsdStageRefPtr
MakeSkinnedRig()
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

    const UsdPrim mesh =
        stage->DefinePrim(kTarget.GetPrimPath(), TfToken("Mesh"));
    mesh.GetAttribute(TfToken("points")).Set(BasePoints());

    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(kSkin, TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves")).SetTargets({kTarget});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({alongX.GetPath(), alongY.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(2);
    VtIntArray indices(kPointCount * 2);
    for (size_t i = 0; i < kPointCount; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
    }
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(indices);
    return stage;
}

VtFloatArray
Weights(float alongX, float alongY)
{
    VtFloatArray weights(kPointCount * 2);
    for (size_t i = 0; i < kPointCount; ++i) {
        weights[i * 2] = alongX;
        weights[i * 2 + 1] = alongY;
    }
    return weights;
}

// The displacement the layout above produces, checked against every point.
void
CheckSkinned(const RigExecRigPose &pose, float alongX, float alongY,
             const char *what)
{
    CHECK(pose.valid);
    const auto found = pose.movedProperties.find(kTarget);
    CHECK(found != pose.movedProperties.end());
    if (found == pose.movedProperties.end()) {
        std::printf("  (%s: no moved points)\n", what);
        return;
    }
    const VtVec3fArray points = found->second.Get<VtVec3fArray>();
    CHECK(points.size() == kPointCount);
    if (points.size() != kPointCount) {
        return;
    }
    const VtVec3fArray base = BasePoints();
    for (size_t i = 0; i < kPointCount; ++i) {
        const GfVec3f expected =
            base[i] + GfVec3f(alongX * 10.0f, alongY * 20.0f, 0.0f);
        if ((GfVec3d(points[i]) - GfVec3d(expected)).GetLength() >= 1e-4) {
            ++failures;
            std::printf("FAIL %s: point %zu is (%g %g %g), expected "
                        "(%g %g %g)\n", what, i, points[i][0], points[i][1],
                        points[i][2], expected[0], expected[1], expected[2]);
        }
    }
}

void
CompileOrReport(RigExecRigEvaluator *evaluator)
{
    std::vector<std::string> errors;
    if (!evaluator->Compile(&errors)) {
        ++failures;
        for (const std::string &error : errors) {
            std::printf("FAIL compile: %s\n", error.c_str());
        }
    }
}

// A weight-paint edit between two evaluations. The epoch does not change --
// asserted, because that is exactly what makes this the interesting case --
// so only the notice can invalidate the cached layout.
void
TestWeightPaintEditAfterFirstEvaluate()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdAttribute weights =
        stage->GetPrimAtPath(kSkin)
            .CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    const size_t epoch = evaluator.GetBindingEpochDigest();
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 1.0f, 0.0f,
                 "painted weights before the edit");

    weights.Set(Weights(0.0f, 1.0f));
    const RigExecRigPose repainted = evaluator.Evaluate(UsdTimeCode::Default());
    CheckSkinned(repainted, 0.0f, 1.0f, "painted weights after the edit");
    CHECK(evaluator.GetBindingEpochDigest() == epoch);

    // A partial repaint, to catch a cache that happens to hold the right
    // answer for the two extremes.
    weights.Set(Weights(0.25f, 0.5f));
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 0.25f, 0.5f,
                 "partially painted weights");

    // The indices are topology: an edit rebuilds the program rather than
    // reaching it as a value, and swapping the index of every second slot
    // swaps which control drives it.
    VtIntArray swapped(kPointCount * 2);
    for (size_t i = 0; i < kPointCount; ++i) {
        swapped[i * 2] = 1;
        swapped[i * 2 + 1] = 0;
    }
    stage->GetPrimAtPath(kSkin)
        .GetAttribute(TfToken("rigExec:jointIndices")).Set(swapped);
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 0.5f, 0.25f,
                 "swapped joint indices");
}

// A time-sampled layout is not epoch state, so Compile must refuse to cache
// it and every frame must read its own samples.
void
TestTimeSampledWeightsRefuseTheCache()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdAttribute weights =
        stage->GetPrimAtPath(kSkin)
            .CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray);
    // A default as well, because compile validates the layout at Default and
    // an export with samples only would fail there for an unrelated reason.
    weights.Set(Weights(1.0f, 0.0f));
    weights.Set(Weights(1.0f, 0.0f), 1.0);
    weights.Set(Weights(0.0f, 1.0f), 2.0);
    weights.Set(Weights(0.5f, 0.5f), 3.0);
    CHECK(weights.ValueMightBeTimeVarying());

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    // Frame order matters: the first evaluation is the one that would fill a
    // cache, and the later frames are the ones a cache would answer wrongly.
    CheckSkinned(evaluator.Evaluate(1.0), 1.0f, 0.0f, "time sample 1");
    CheckSkinned(evaluator.Evaluate(2.0), 0.0f, 1.0f, "time sample 2");
    CheckSkinned(evaluator.Evaluate(3.0), 0.5f, 0.5f, "time sample 3");
    // And backwards, so the answer cannot be coming from a one-frame memo.
    CheckSkinned(evaluator.Evaluate(1.0), 1.0f, 0.0f, "time sample 1 again");
}

// An interactive override replaces the layout without touching the stage,
// so no notice fires and the override path itself has to drop the cache.
void
TestInteractiveOverrideOnTheLayout()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdAttribute weights =
        stage->GetPrimAtPath(kSkin)
            .CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 1.0f, 0.0f,
                 "authored weights");

    RigExecValueOverride override;
    override.prim = kSkin;
    override.attribute = TfToken("rigExec:jointWeights");
    override.value = VtValue(Weights(0.0f, 1.0f));
    // The layout really was dropped, rather than the override reaching the
    // deformation by some other route: the cache is emptied by an override
    // that names a layout attribute, which is the YES branch of the
    // predicate SetInteractiveOverrides narrowed the clear to.
    CHECK(evaluator.GetSkinTopologyCacheSize() > 0);
    evaluator.SetInteractiveOverrides({override});
    CHECK(evaluator.GetSkinTopologyCacheSize() == 0);
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 0.0f, 1.0f,
                 "overridden weights");

    evaluator.ClearInteractiveOverrides();
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 1.0f, 0.0f,
                 "authored weights after the override is dropped");
}

// A connected layout attribute is refused too: the connection is what the
// value comes from, and following it is not this cache's business.
void
TestConnectedWeightsStillEvaluate()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdPrim skin = stage->GetPrimAtPath(kSkin);
    const UsdAttribute weights = skin.CreateAttribute(
        TfToken("rigExec:jointWeights"), SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));
    const UsdAttribute source = skin.CreateAttribute(
        TfToken("inputs:paintedWeights"), SdfValueTypeNames->FloatArray);
    source.Set(Weights(1.0f, 0.0f));
    CHECK(weights.AddConnection(source.GetPath()));
    CHECK(weights.HasAuthoredConnections());

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 1.0f, 0.0f,
                 "connected weights");
    weights.Set(Weights(0.0f, 1.0f));
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 0.0f, 1.0f,
                 "connected weights after an edit");
}

// The base points are cached on the same terms: a value edit to them must
// still reach the deformation, and a time-sampled base must be re-read.
void
TestBasePointEdits()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    stage->GetPrimAtPath(kSkin)
        .CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(Weights(1.0f, 0.0f));
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 1.0f, 0.0f,
                 "authored base");

    const UsdAttribute points = stage->GetAttributeAtPath(kTarget);
    VtVec3fArray moved = BasePoints();
    for (GfVec3f &point : moved) {
        point += GfVec3f(0, 0, 100);
    }
    points.Set(moved);
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    const auto published = pose.movedProperties.find(kTarget);
    CHECK(published != pose.movedProperties.end());
    if (published == pose.movedProperties.end()) {
        return;
    }
    const VtVec3fArray result = published->second.Get<VtVec3fArray>();
    CHECK(result.size() == kPointCount);
    for (size_t i = 0; i < result.size(); ++i) {
        CHECK(std::abs(result[i][2] - (moved[i][2])) < 1e-4f);
        CHECK(std::abs(result[i][0] - (moved[i][0] + 10.0f)) < 1e-4f);
    }
}

// The one check the epoch cannot make: whether the points that arrive are
// the points the layout describes. A cached layout is validated once, so the
// per-frame guard is reduced to a length comparison -- and a base that
// changes cardinality between frames is what that guard is for. Compile
// validates the layout against the base at Default, so this has to be a
// time-varying base to get past it and reach the kernel.
void
TestPointCountChangeFailsAtomically()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    stage->GetPrimAtPath(kSkin)
        .CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(Weights(1.0f, 0.0f));
    VtVec3fArray longer = BasePoints();
    longer.push_back(GfVec3f(-1, -2, -3));
    const UsdAttribute points = stage->GetAttributeAtPath(kTarget);
    points.Set(BasePoints(), 1.0);
    points.Set(longer, 2.0);

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    CheckSkinned(evaluator.Evaluate(1.0), 1.0f, 0.0f, "matched layout");

    const RigExecRigPose pose = evaluator.Evaluate(2.0);
    const auto found = pose.movedProperties.find(kTarget);
    CHECK(found != pose.movedProperties.end());
    if (found != pose.movedProperties.end()) {
        const VtVec3fArray result = found->second.Get<VtVec3fArray>();
        CHECK(result.size() == longer.size());
        // Passed through unchanged, not partially skinned.
        for (size_t i = 0; i < result.size() && i < longer.size(); ++i) {
            CHECK(result[i] == longer[i]);
        }
    }
    bool reported = false;
    for (const std::string &diagnostic : pose.diagnostics) {
        reported = reported ||
            diagnostic.find("MoverFailed") != std::string::npos;
    }
    CHECK(reported);

    // And it recovers at a time where the layout matches again.
    CheckSkinned(evaluator.Evaluate(1.0), 1.0f, 0.0f, "layout matched again");
}


// A partial constant envelope, which is the case both geometry loops' "the
// blend is the identity, skip it" fast path must NOT take.
// Every shipped stage and every other fixture here carries a full-strength
// constant envelope, where skipping and blending give the same answer -- so
// without this the predicate is only ever exercised where it cannot be
// wrong, and a version of it that answered "full strength" for 0.5 would
// double the deformation in silence.
void
TestAPartialConstantEnvelopeIsApplied()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    stage->GetPrimAtPath(kSkin)
        .CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(Weights(1.0f, 0.0f));
    stage->GetPrimAtPath(kSkin)
        .GetAttribute(TfToken("inputs:defaultWeight")).Set(0.5f);

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    // Half the displacement, because the envelope is applied once against
    // the preceding revision.
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 0.5f, 0.0f,
                 "a half-strength constant envelope");
    // And zero strength is a pass-through, not a skip.
    stage->GetPrimAtPath(kSkin)
        .GetAttribute(TfToken("inputs:defaultWeight")).Set(0.0f);
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 0.0f, 0.0f,
                 "a zero-strength constant envelope");
}

// The same refusal, taken AFTER the epoch was compiled.
// Authoring the first time sample on a layout attribute -- or connecting it --
// moves no structure digest, so nothing recompiles and Compile's decision is
// never re-taken. It does send a notice, and a notice drops the cache, so the
// re-read is the one place that sees the stage as it now is: the decision has
// to be re-taken there or the layout freezes at whatever frame happened to
// come first after the edit.
void
TestWeightsBecomeTimeSampledAfterCompile()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdAttribute weights =
        stage->GetPrimAtPath(kSkin)
            .CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    const size_t epoch = evaluator.GetBindingEpochDigest();
    // The frame that fills the cache, before the layout is animated at all.
    CheckSkinned(evaluator.Evaluate(1.0), 1.0f, 0.0f, "constant weights");

    weights.Set(Weights(1.0f, 0.0f), 1.0);
    weights.Set(Weights(0.0f, 1.0f), 2.0);
    weights.Set(Weights(0.5f, 0.5f), 3.0);
    CHECK(weights.ValueMightBeTimeVarying());
    // The digest is blind to values and to their time codes, so nothing
    // about this edit recompiles -- which is exactly why the decision has to
    // be re-taken where the cache is refilled.
    CheckSkinned(evaluator.Evaluate(2.0), 0.0f, 1.0f, "sample 2 after compile");
    CheckSkinned(evaluator.Evaluate(3.0), 0.5f, 0.5f, "sample 3 after compile");
    CheckSkinned(evaluator.Evaluate(1.0), 1.0f, 0.0f, "sample 1 after compile");
    CHECK(evaluator.GetBindingEpochDigest() == epoch);
}

// And the connected half of the same after-Compile case: an attribute that
// gains a connection after the epoch was compiled.
// The layout read itself does not FOLLOW the connection (see _Array in
// moverGraph.cpp: the attribute's own value is what a layout is), so the
// values below are the attribute's throughout. What is being tested is that
// gaining a connection after Compile takes the packet off the shared-layout
// path and leaves the rig evaluating -- the same refusal
// TestConnectedWeightsStillEvaluate makes before Compile.
void
TestWeightsBecomeConnectedAfterCompile()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdPrim skin = stage->GetPrimAtPath(kSkin);
    const UsdAttribute weights = skin.CreateAttribute(
        TfToken("rigExec:jointWeights"), SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));
    const UsdAttribute source = skin.CreateAttribute(
        TfToken("inputs:paintedWeights"), SdfValueTypeNames->FloatArray);
    source.Set(Weights(0.0f, 1.0f));

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    CheckSkinned(evaluator.Evaluate(1.0), 1.0f, 0.0f, "unconnected weights");

    CHECK(weights.AddConnection(source.GetPath()));
    CHECK(weights.HasAuthoredConnections());
    CheckSkinned(evaluator.Evaluate(1.0), 1.0f, 0.0f,
                 "weights connected after compile");
    weights.Set(Weights(0.25f, 0.5f));
    CheckSkinned(evaluator.Evaluate(2.0), 0.25f, 0.5f,
                 "connected weights repainted after compile");
}

// An edit that touched nothing the rig reads still clears the layout cache --
// every notice does, because the cache cannot tell which properties a notice
// named. Re-reading is cheap; handing back a NEW layout pointer is not,
// because the mover's packet compares layouts by identity and an unequal
// packet re-runs the whole per-point kernel for a binding that did not move.
void
TestAnUnrelatedEditDoesNotRerunTheKernel()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    stage->GetPrimAtPath(kSkin)
        .CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(Weights(1.0f, 0.0f));

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    CHECK(evaluator.GetBakedProgram());
    if (!evaluator.GetBakedProgram()) return;
    const size_t epoch = evaluator.GetBindingEpochDigest();
    // The first generation builds the graph and runs the kernel; the second,
    // identical, must run nothing. That is the baseline the case below is
    // measured against.
    CHECK(evaluator.Evaluate(1.0).executedOpCount > 0);
    CHECK(evaluator.Evaluate(1.0).executedOpCount == 0);

    // A prim the rig has never heard of, between two identical frames.
    // Neutral routing preserves the stamp as well as the binding epoch.
    const size_t stamp=evaluator.GetBakedProgram()->GetStepGraph().programStamp;
    stage->DefinePrim(SdfPath("/Scratch"), TfToken("Scope"));
    CHECK(evaluator.GetLastNoticeDisposition()==RigExecNoticeDisposition::Edited);
    CHECK(evaluator.GetBakedProgram()->GetStepGraph().programStamp==stamp);
    const RigExecRigPose after = evaluator.Evaluate(1.0);
    CheckSkinned(after, 1.0f, 0.0f, "after an unrelated edit");
    CHECK(evaluator.GetBindingEpochDigest() == epoch);
    CHECK(after.executedOpCount == 0);

    // And the cache is still a cache, not a freeze: a real weight edit after
    // the unrelated one still lands.
    stage->GetPrimAtPath(kSkin)
        .GetAttribute(TfToken("rigExec:jointWeights")).Set(Weights(0.0f, 1.0f));
    const RigExecRigPose repainted = evaluator.Evaluate(1.0);
    CheckSkinned(repainted, 0.0f, 1.0f, "repainted after an unrelated edit");
    CHECK(repainted.executedOpCount > 0);
}

// An override one hop UPSTREAM of a layout attribute drops the layouts, and
// reaches the deformation.
// The hazard the narrow clear has to survive. An override is compared against
// the properties that can REACH a layout, and "reach" has to mean the same
// walk the value is read through: RigExecResolvedInputs::GetAttribute follows
// one authored connection per hop and consults the overrides at every hop, so
// a value placed on the SOURCE of a connected rigExec:skinningMethod is the
// value the mover assembles with. Compare an override against the mover's own
// four attributes alone and that one is called unrelated, the layouts are
// kept, and a mover deforms against a binding nobody has any more.
// Two halves, and they prove different things. The deformation changing is
// the read walk: skinningMethod is assembled per frame, so it would follow
// the override whatever this cache did. The cache EMPTYING is the predicate:
// nothing else in a generation drops it, and the mover's layout is cached
// here (only the three array attributes decide that, and none of them is
// connected on this rig).
void
TestAnOverrideUpstreamOfTheLayoutDropsTheCache()
{
    const auto build = [](const TfToken &method) {
        UsdStageRefPtr stage = MakeSkinnedRig();
        const UsdPrim skin = stage->GetPrimAtPath(kSkin);
        skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray)
            .Set(Weights(0.5f, 0.5f));
        // A rotating influence, so that the two skinning methods answer
        // differently: dual-quaternion and linear blending agree exactly
        // while every influence is a pure translation.
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/AlongY"))
            .GetAttribute(TfToken("avars:rz")).Set(90.0);
        const UsdAttribute source = skin.CreateAttribute(
            TfToken("inputs:method"), SdfValueTypeNames->Token);
        source.Set(method);
        const UsdAttribute declared = skin.CreateAttribute(
            TfToken("rigExec:skinningMethod"), SdfValueTypeNames->Token);
        declared.Set(TfToken("classicLinear"));
        declared.AddConnection(source.GetPath());
        return stage;
    };
    const auto deformed = [](RigExecRigEvaluator *evaluator) {
        const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode::Default());
        const auto found = pose.movedProperties.find(kTarget);
        return found == pose.movedProperties.end()
                   ? VtVec3fArray()
                   : found->second.Get<VtVec3fArray>();
    };

    // What the stage says when the connection's source is authored either
    // way, with no override anywhere: the two answers this rig can give.
    UsdStageRefPtr linearStage = build(TfToken("classicLinear"));
    UsdStageRefPtr dualStage = build(TfToken("dualQuaternion"));
    RigExecRigEvaluator linear(linearStage, SdfPath("/Asset/Rig"));
    RigExecRigEvaluator dual(dualStage, SdfPath("/Asset/Rig"));
    CompileOrReport(&linear);
    CompileOrReport(&dual);
    const VtVec3fArray linearPoints = deformed(&linear);
    const VtVec3fArray dualPoints = deformed(&dual);
    CHECK(linearPoints.size() == kPointCount);
    // The fixture is only worth anything while the two methods disagree.
    CHECK(linearPoints != dualPoints);

    // Now the same difference asked for with an override, one hop upstream
    // of the attribute the mover declares.
    UsdStageRefPtr stage = build(TfToken("classicLinear"));
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    CHECK(deformed(&evaluator) == linearPoints);
    CHECK(evaluator.GetSkinTopologyCacheSize() > 0);

    RigExecValueOverride override;
    override.prim = kSkin;
    override.attribute = TfToken("inputs:method");
    override.value = VtValue(TfToken("dualQuaternion"));
    evaluator.SetInteractiveOverrides({override});
    CHECK(evaluator.GetSkinTopologyCacheSize() == 0);
    CHECK(deformed(&evaluator) == dualPoints);

    evaluator.ClearInteractiveOverrides();
    CHECK(evaluator.GetSkinTopologyCacheSize() == 0);
    CHECK(deformed(&evaluator) == linearPoints);
}

// An override on a control keeps them.
// The whole point of narrowing the clear: a drag names a control avar, and
// re-reading and re-comparing every skinned mesh's layout for it costs about
// a third of a baked drag frame on a biped (~400us of ~1.1ms) for arrays no
// manipulator can touch. The deformation still follows the drag -- the
// layout is not what a control moves.
void
TestAnUnrelatedOverrideKeepsTheCache()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    stage->GetPrimAtPath(kSkin)
        .CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(Weights(1.0f, 0.0f));

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&evaluator);
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 1.0f, 0.0f,
                 "before the drag");
    const size_t cached = evaluator.GetSkinTopologyCacheSize();
    CHECK(cached > 0);

    RigExecValueOverride override;
    override.prim = SdfPath("/Asset/Rig/AlongX");
    override.attribute = TfToken("avars:tx");
    override.value = VtValue(20.0);
    evaluator.SetInteractiveOverrides({override});
    CHECK(evaluator.GetSkinTopologyCacheSize() == cached);
    // tx = 20 against a rest of 10 is two units of the influence's
    // displacement, which is what CheckSkinned's first argument scales.
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 2.0f, 0.0f,
                 "during the drag");
    CHECK(evaluator.GetSkinTopologyCacheSize() == cached);
    evaluator.ClearInteractiveOverrides();
    CHECK(evaluator.GetSkinTopologyCacheSize() == cached);
    CheckSkinned(evaluator.Evaluate(UsdTimeCode::Default()), 1.0f, 0.0f,
                 "after the drag");
}

// The baked program's layouts (--baked). The program keeps one layout handle
// per fixed skin revision and adopts it where the geometry prologue resolves
// a topology; these cases hold it to the stage after authored edits, drags,
// time samples and a bailed generation, and hold the exported layout fields
// to where they were set.

using Revision = RigExecBakedProgramImpl::GeomRevision;

bool
ParityMode()
{
    return TfGetenv("RIGEXEC_EVALUATION_MODE") == "parity";
}

bool
ReferenceChecksRequested()
{
    return ParityMode() ? true
                        : false;
}

std::unique_ptr<RigExecRigEvaluator>
CompiledIn(const UsdStageRefPtr &stage, const SdfPath &rig,
           bool referenceChecks)
{
    auto evaluator = std::make_unique<RigExecRigEvaluator>(stage, rig);
    CompileOrReport(evaluator.get());
    evaluator->cpuReference = referenceChecks;
    return evaluator;
}

// The revision of \p program whose mover is \p mover, chain or derived.
const Revision *
RevisionOf(const RigExecBakedProgramImpl &program, const SdfPath &mover)
{
    for (const auto &chain : program.chains) {
        for (const Revision &revision : chain.revisions) {
            if (revision.moverPath == mover) {
                return &revision;
            }
        }
        for (const auto &derived : chain.derived) {
            if (derived.revision.moverPath == mover) {
                return &derived.revision;
            }
        }
    }
    return nullptr;
}

const Revision *
SkinRevision(const RigExecRigEvaluator &evaluator)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    return program ? RevisionOf(program->GetStepGraph(), kSkin) : nullptr;
}

// What the layout handle adopted for the skin holds: null when there is no
// program, no revision, or a refused layout.
std::shared_ptr<const RigExecSkinTopology>
Adopted(const RigExecRigEvaluator &evaluator)
{
    const Revision *revision = SkinRevision(evaluator);
    return revision ? revision->topology : nullptr;
}

// \p pose against a program built fresh on the stage as it stands, under the
// same overrides, at \p time: equal in every value a pose publishes. The
// mover-graph counters count what a generation built, which a fresh
// program's first generation always does, so they are left out.
void
CheckAgreesWithFresh(const std::string &what, const UsdStageRefPtr &stage,
                     const std::vector<RigExecValueOverride> &overrides,
                     UsdTimeCode time, RigExecRigPose pose)
{
    auto fresh = CompiledIn(stage, SdfPath("/Asset/Rig"), ReferenceChecksRequested());
    fresh->SetInteractiveOverrides(overrides);
    RigExecRigPose expected = fresh->Evaluate(time);
    CHECK(pose.valid);
    CHECK(expected.valid);
    CHECK(pose.referenceMismatches == 0);
    const auto keepPosed = [](std::vector<std::string> *lines) {
        std::vector<std::string> kept;
        for (std::string &line : *lines) {
            if (line.rfind("mover graph:", 0) != 0) {
                kept.push_back(std::move(line));
            }
        }
        *lines = std::move(kept);
    };
    keepPosed(&expected.diagnostics);
    keepPosed(&pose.diagnostics);
    RigExecRigPose diff;
    RigExecComparePoses(expected, pose, &diff);
    if (diff.comparisonMismatches != 0) {
        ++failures;
        std::printf("FAIL %s: %zu difference(s) from a fresh program\n",
                    what.c_str(), diff.comparisonMismatches);
        for (const std::string &line : diff.diagnostics) {
            std::printf("    %s\n", line.c_str());
        }
    }
}

// \p pose against the exec reference on the stage as it stands at \p time,
// in every value; the counters and lines are what the reference built.
void
CheckAgreesWithReference(const std::string &what,
                         const UsdStageRefPtr &stage, UsdTimeCode time,
                         RigExecRigPose pose)
{
    auto reference = CompiledIn(stage, SdfPath("/Asset/Rig"),
                                true);
    const RigExecRigPose expected = reference->Evaluate(time);
    CHECK(pose.valid);
    CHECK(expected.valid);
    pose.diagnostics = expected.diagnostics;
    RigExecRigPose diff;
    RigExecComparePoses(expected, pose, &diff);
    if (diff.comparisonMismatches != 0) {
        ++failures;
        std::printf("FAIL %s: %zu difference(s) from the reference\n",
                    what.c_str(), diff.comparisonMismatches);
        for (const std::string &line : diff.diagnostics) {
            std::printf("    %s\n", line.c_str());
        }
    }
}

RigExecValueOverride
WeightsOverride(const VtFloatArray &weights)
{
    RigExecValueOverride override;
    override.prim = kSkin;
    override.attribute = TfToken("rigExec:jointWeights");
    override.value = VtValue(weights);
    return override;
}

// Calls ApplyValueEdits on a program run outside an evaluator, and bumps its
// stamp for a notice it cannot route, as the evaluator's notice handler does.
class ValueEditRouter : public TfWeakBase
{
public:
    ValueEditRouter(const UsdStageRefPtr &stage, RigExecBakedProgram *program)
        : _program(program)
    {
        RigExecInputReplayHeldProgram::Router(program, true);
        _key = TfNotice::Register(TfCreateWeakPtr(this),
                                  &ValueEditRouter::_OnChanged, stage);
    }
    ~ValueEditRouter() { TfNotice::Revoke(_key); RigExecInputReplayHeldProgram::Router(_program, false); }

private:
    void _OnChanged(const UsdNotice::ObjectsChanged &notice,
                    const UsdStageWeakPtr &)
    {
        if (!_program->ApplyValueEdits(notice)) {
            _program->BumpProgramStamp();
        }
    }
    RigExecBakedProgram *_program;
    TfNotice::Key _key;
};

// An authored edit of each layout array, and a drag on the weights and its
// lift: each equal to a program built fresh after it. The indices are
// topology, so their edit rebuilds the program once; the weights are values,
// and nothing after that rebuilds. Then time samples authored on the weights
// and removed again, run on a program directly and held to the exec
// reference.
// Direct Run publishes into caller-owned pose storage; initialize the same
// fresh output and requested-time metadata that Evaluate initializes before Run.
bool
RunSkinProgramAtTime(RigExecBakedProgram &program, RigExecRigEvaluator &owner,
                     UsdTimeCode time, RigExecRigPose *pose)
{
    if (pose) {
        *pose = RigExecRigPose();
        pose->time = time;
    }
    return RigExecInputActionsRunProgram(program, owner, time, pose);
}

void
TestBakedLayoutFollowsEdits()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdPrim skin = stage->GetPrimAtPath(kSkin);
    const UsdAttribute weights = skin.CreateAttribute(
        TfToken("rigExec:jointWeights"), SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));
    auto evaluator = CompiledIn(stage, SdfPath("/Asset/Rig"), ReferenceChecksRequested());
    const UsdTimeCode t1(1.0), t2(2.0);
    const std::vector<RigExecValueOverride> none;

    RigExecRigPose pose = evaluator->Evaluate(t1);
    CheckSkinned(pose, 1.0f, 0.0f, "baked, authored layout");
    CheckAgreesWithFresh("baked, authored layout", stage, none, t1, pose);
    const size_t builds = evaluator->GetBakedProgramBuildCount();
    CHECK(builds > 0);
    CHECK(Adopted(*evaluator) != nullptr);

    // Swapping the indices swaps which control drives each slot.
    VtIntArray swapped(kPointCount * 2);
    for (size_t i = 0; i < kPointCount; ++i) {
        swapped[i * 2] = 1;
        swapped[i * 2 + 1] = 0;
    }
    skin.GetAttribute(TfToken("rigExec:jointIndices")).Set(swapped);
    CHECK(evaluator->GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Stale);
    pose = evaluator->Evaluate(t1);
    CheckSkinned(pose, 0.0f, 1.0f, "baked, jointIndices edited");
    CheckAgreesWithFresh("baked, jointIndices edited", stage, none, t1, pose);
    CHECK(evaluator->GetBakedProgramBuildCount() == builds + 1);
    const size_t rebuilt = evaluator->GetBakedProgramBuildCount();

    weights.Set(Weights(0.25f, 0.5f));
    CHECK(evaluator->GetLastNoticeDisposition() !=
          RigExecNoticeDisposition::Stale);
    pose = evaluator->Evaluate(t1);
    CheckSkinned(pose, 0.5f, 0.25f, "baked, jointWeights edited");
    CheckAgreesWithFresh("baked, jointWeights edited", stage, none, t1, pose);

    const std::vector<RigExecValueOverride> drag = {
        WeightsOverride(Weights(0.0f, 1.0f))};
    evaluator->SetInteractiveOverrides(drag);
    pose = evaluator->Evaluate(t1);
    CheckSkinned(pose, 1.0f, 0.0f, "baked, jointWeights dragged");
    CheckAgreesWithFresh("baked, jointWeights dragged", stage, drag, t1, pose);
    evaluator->ClearInteractiveOverrides();
    pose = evaluator->Evaluate(t1);
    CheckSkinned(pose, 0.5f, 0.25f, "baked, drag lifted");
    CheckAgreesWithFresh("baked, drag lifted", stage, none, t1, pose);
    CHECK(Adopted(*evaluator) != nullptr);

    CHECK(evaluator->GetBakedProgramBuildCount() == rebuilt);

    // Two time samples on the weights: the layout is no longer epoch state.
    // Retain a program across the new animation. Its declared SkinTopology
    // producer adopts every sampled layout and preserves equal held values.
    RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&rig);
    const std::unique_ptr<RigExecBakedProgram> program =
        RigExecBakedProgram::Build(&rig, nullptr);
    CHECK(program);
    if (!program) {
        return;
    }
    CHECK(RunSkinProgramAtTime(*program, rig, t1, &pose));
    CheckSkinned(pose, 0.5f, 0.25f, "direct, before the samples");
    const Revision *revision = RevisionOf(program->GetStepGraph(), kSkin);
    CHECK(revision != nullptr);
    if (!revision) {
        return;
    }
    const std::shared_ptr<const RigExecSkinTopology> held =
        revision->topology;
    CHECK(held != nullptr);
    ValueEditRouter router(stage, program.get());
    weights.Set(Weights(1.0f, 0.0f), 1.0);
    weights.Set(Weights(0.0f, 1.0f), 2.0);
    CHECK(weights.ValueMightBeTimeVarying());
    CHECK(RunSkinProgramAtTime(*program, rig, t1, &pose));
    CheckSkinned(pose, 0.0f, 1.0f, "direct, time-sampled weights at 1");
    CheckAgreesWithReference("direct, time-sampled weights at 1", stage, t1,
                             pose);
    CHECK(revision->topologyResolved && revision->topology != nullptr);
    CHECK(revision->layoutHandle == revision->topology && !revision->layoutFixed);
    const auto firstAnimatedLayout = revision->topology;
    CHECK(RunSkinProgramAtTime(*program, rig, t2, &pose));
    CheckSkinned(pose, 1.0f, 0.0f, "direct, time-sampled weights at 2");
    CheckAgreesWithReference("direct, time-sampled weights at 2", stage, t2,
                             pose);
    CHECK(revision->topology != nullptr && revision->topology != firstAnimatedLayout);
    CHECK(revision->layoutHandle == revision->topology);
    weights.ClearAtTime(1.0);
    weights.ClearAtTime(2.0);
    CHECK(!weights.ValueMightBeTimeVarying());
    CHECK(RunSkinProgramAtTime(*program, rig, t1, &pose));
    CheckSkinned(pose, 0.5f, 0.25f, "direct, time samples removed");
    CheckAgreesWithReference("direct, time samples removed", stage, t1, pose);
    CHECK(revision->topology != nullptr && *revision->topology == *held);
    CHECK(revision->layoutHandle == revision->topology);
    const auto restoredLayout = revision->topology;
    CHECK(RunSkinProgramAtTime(*program, rig, t1,&pose));
    CheckSkinned(pose, 0.5f, 0.25f, "held restored layout");
    CHECK(revision->topology == restoredLayout);
}

// Original retained StageFrames refusal preserves prepared layout work,
// but withholds geometry adoption/publication until target restoration.
// Run the retained program directly to exercise sampled refusal state.
void
TestATargetRefusalKeepsIndependentLayoutWork()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdPrim skin = stage->GetPrimAtPath(kSkin);
    const UsdAttribute weights = skin.CreateAttribute(
        TfToken("rigExec:jointWeights"), SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));
    const SdfPath probe("/Asset/Geom/Probe");
    const UsdPrim probePrim = stage->DefinePrim(probe, TfToken("Xform"));
    const SdfPath probeB("/Asset/Geom/ProbeB");
    const UsdPrim probeBPrim = stage->DefinePrim(probeB, TfToken("Xform"));
    const UsdPrim aim = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/ProbeAim"),
        TfToken("RigExecAimConstraint"));
    CHECK(aim.CreateAttribute(TfToken("rigExec:aimAxis"),
                              SdfValueTypeNames->Token, false,
                              SdfVariabilityUniform)
              .Set(TfToken("z")));
    CHECK(aim.CreateRelationship(TfToken("rigExec:aimTarget"))
              .SetTargets({SdfPath("/Asset/Rig/AlongX")}));
    CHECK(aim.CreateRelationship(TfToken("rigExec:moves")).SetTargets({probe}));

    const UsdPrim aimB = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/ProbeAimB"),
        TfToken("RigExecAimConstraint"));
    CHECK(aimB.CreateAttribute(TfToken("rigExec:aimAxis"),
                              SdfValueTypeNames->Token, false,
                              SdfVariabilityUniform)
              .Set(TfToken("z")));
    CHECK(aimB.CreateRelationship(TfToken("rigExec:aimTarget"))
              .SetTargets({SdfPath("/Asset/Rig/AlongX")}));
    CHECK(aimB.CreateRelationship(TfToken("rigExec:moves")).SetTargets({probeB}));

    RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
    CompileOrReport(&rig);
    std::vector<std::string> reasons;
    const auto program = RigExecInputReplayHeldProgram::Build(&rig, &reasons);
    CHECK(bool(*program));
    if (!*program) {
        for (const std::string &reason : reasons) {
            std::printf("    refused: %s\n", reason.c_str());
        }
        return;
    }
    const auto &compiled = (*program)->GetStepGraph();
    const auto hasRequiredTarget = [&](const SdfPath &path) {
        return std::any_of(compiled.xformSlots.begin(), compiled.xformSlots.end(),
            [&](int slot) { return compiled.paths[size_t(slot)] == path; });
    };
    CHECK(hasRequiredTarget(probe));
    CHECK(hasRequiredTarget(probeB));
    RigExecRigPose first;
    CHECK(program->Run(UsdTimeCode(1.0), &first, true));
    CheckSkinned(first, 1.0f, 0.0f, "before the bail");
    const Revision *revision = RevisionOf((*program)->GetStepGraph(), kSkin);
    CHECK(revision != nullptr);
    if (!revision) {
        return;
    }
    const std::shared_ptr<const RigExecSkinTopology> held =
        revision->topology;
    CHECK(held != nullptr);

    // Duplicate admission input must be rejected, not counted as one read.
    auto &mutableGraph = const_cast<RigExecBakedProgramImpl &>((*program)->GetStepGraph());
    auto gate = std::find_if(mutableGraph.steps.begin(), mutableGraph.steps.end(), [](const auto &step) {
        return std::any_of(step.reads.begin(), step.reads.end(), [](const auto &read) {
            return read.domain == RigExecBakedSlotDomain::RequiredStageFramesAdmission;
        });
    });
    CHECK(gate != mutableGraph.steps.end());
    std::string graphError;
    CHECK(RigExecBakedValidateStepGraph(mutableGraph, &graphError));
    if (gate != mutableGraph.steps.end()) {
        const auto savedReads = gate->reads;
        const auto admissionRead = *std::find_if(savedReads.begin(), savedReads.end(), [](const auto &read) {
            return read.domain == RigExecBakedSlotDomain::RequiredStageFramesAdmission;
        });
        gate->reads.push_back(admissionRead);
        CHECK(!RigExecBakedValidateStepGraph(mutableGraph, &graphError));
        CHECK(graphError.find("required stage-frame admission role") != std::string::npos);
        gate->reads = savedReads;
        CHECK(RigExecBakedValidateStepGraph(mutableGraph, &graphError));
    }

    ValueEditRouter router(stage, program->get());
    CHECK(probeBPrim.SetTypeName(TfToken("Scope")));
    CHECK(probePrim.SetTypeName(TfToken("Scope")));
    weights.Set(Weights(0.0f, 1.0f));
    RigExecRigPose given;
    CHECK(!program->Run(UsdTimeCode(2.0), &given, true));
    CHECK((*program)->GetLastBail() == RigExecBakedBail::StageFrames);
    CHECK(!given.valid);
    const auto &graph = (*program)->GetStepGraph();
    const auto &admission = graph.requiredStageFramesAdmission;
    CHECK(!admission.admitted && admission.firstBadTarget >= 0);
    int firstBad = -1;
    for (size_t i = 0; i < graph.xformSlots.size(); ++i) {
        const SdfPath &path = graph.paths[size_t(graph.xformSlots[i])];
        if (path == probe || path == probeB) { firstBad = int(i); break; }
    }
    CHECK(firstBad >= 0 && admission.firstBadTarget == firstBad);
    const SdfPath firstBadPath = firstBad >= 0
        ? graph.paths[size_t(graph.xformSlots[size_t(firstBad)])] : SdfPath();
    CHECK(given.providerXforms.empty());
    CHECK(given.providerBaseXforms.empty());
    CHECK(given.jointMatricesFinal.empty());
    CHECK(given.movedProperties.count(kTarget) == 0);
    CHECK(!given.diagnostics.empty() && given.diagnostics.back() ==
        "could not resolve constraint target " + firstBadPath.GetString() +
        " relative to the asset root");
    // Preparation survives, but the geometry adoption boundary does not run.
    CHECK(revision->topology == held);
    CHECK(revision->layoutHandle != nullptr && revision->layoutHandle != held);
    if (revision->layoutHandle) {
        const VtFloatArray painted = Weights(0.0f, 1.0f);
        CHECK(revision->layoutHandle->weights ==
              std::vector<float>(painted.begin(), painted.end()));
    }
    const std::shared_ptr<const RigExecSkinTopology> repainted =
        revision->layoutHandle;

    RigExecRigPose heldRefusal;
    CHECK(!program->Run(UsdTimeCode(2.0), &heldRefusal, true));
    CHECK((*program)->GetLastBail() == RigExecBakedBail::StageFrames);
    CHECK(!heldRefusal.valid && heldRefusal.movedProperties.count(kTarget) == 0);
    CHECK(heldRefusal.diagnostics == given.diagnostics);
    CHECK(revision->topology == held);
    CHECK(revision->layoutHandle == repainted);

    CHECK(probePrim.SetTypeName(TfToken("Xform")));
    CHECK(probeBPrim.SetTypeName(TfToken("Xform")));
    RigExecRigPose next;
    CHECK(program->Run(UsdTimeCode(3.0), &next, true));
    CHECK((*program)->GetStepGraph().requiredStageFramesAdmission.admitted);
    CHECK((*program)->GetStepGraph().requiredStageFramesAdmission.firstBadTarget == -1);
    CheckSkinned(next, 0.0f, 1.0f, "the generation after the bail");
    CHECK(revision->topology != held);
    CHECK(revision->topology == repainted);

    CheckAgreesWithReference("the generation after the bail", stage,
                             UsdTimeCode(3.0), next);

    // Successfully reading a degenerate transform is not a missing source.
    // Downstream numerical refusal remains separate from StageFrames admission.
    GfMatrix4d degenerate(1.0); degenerate.SetScale(GfVec3d(0.0));
    CHECK(probePrim.CreateAttribute(TfToken("xformOp:transform"),
        SdfValueTypeNames->Matrix4d).Set(degenerate));
    CHECK(probePrim.CreateAttribute(TfToken("xformOpOrder"),
        SdfValueTypeNames->TokenArray).Set(VtTokenArray{TfToken("xformOp:transform")}));
    RigExecRigPose degeneratePose;
    program->Run(UsdTimeCode(4.0), &degeneratePose, true);
    CHECK((*program)->GetStepGraph().requiredStageFramesAdmission.admitted);
    CHECK((*program)->GetStepGraph().requiredStageFramesAdmission.firstBadTarget == -1);
    CHECK((*program)->GetLastBail() != RigExecBakedBail::StageFrames);
}

// FNV-1a over bytes, for a signature that reads the same in every process.
struct Fnv {
    uint64_t h = 1469598103934665603ull;
    void Bytes(const void *data, size_t n)
    {
        const unsigned char *p = static_cast<const unsigned char *>(data);
        for (size_t i = 0; i < n; ++i) {
            h = (h ^ p[i]) * 1099511628211ull;
        }
    }
    template <class T>
    void Pod(const T &value) { Bytes(&value, sizeof(value)); }
    template <class T>
    void Vector(const std::vector<T> &values)
    {
        Pod(values.size());
        if (!values.empty()) {
            Bytes(values.data(), values.size() * sizeof(T));
        }
    }
    void Str(const std::string &s)
    {
        Pod(s.size());
        Bytes(s.data(), s.size());
    }
};

void
FoldTopology(Fnv *f, const RigExecSkinTopology *topology)
{
    f->Pod(topology != nullptr);
    if (!topology) {
        return;
    }
    f->Vector(topology->indices);
    f->Vector(topology->weights);
    f->Pod(topology->elementSize);
    f->Pod(topology->pointCount);
    f->Pod(topology->influenceCount);
    f->Pod(topology->validated);
}

// The fields the exporter reads off one revision: whether a layout was
// resolved, its content, the partition's identity relative to it, and the
// partition itself.
void
FoldLayoutFields(Fnv *f, const Revision &revision)
{
    f->Str(revision.moverPath.GetString());
    f->Pod(revision.skinTopologyFixed);
    f->Pod(revision.topologyResolved);
    FoldTopology(f, revision.topology.get());
    const int partition =
        !revision.partitionTopology ? 0
        : revision.partitionTopology == revision.topology ? 1 : 2;
    f->Pod(partition);
    if (partition == 2) {
        FoldTopology(f, revision.partitionTopology.get());
    }
    f->Pod(revision.chunked);
    f->Pod(revision.partitionElementSize);
    f->Pod(revision.partitionIndexCount);
    f->Pod(revision.partitionPointCount);
    f->Pod(revision.chunks.size());
    for (const auto &chunk : revision.chunks) {
        f->Pod(chunk.begin);
        f->Pod(chunk.end);
        f->Vector(chunk.key);
    }
}

// What a fresh evaluator layout cache answers for \p revision at \p time:
// the independent scalar reference's layout, which the adopted one must equal.
std::shared_ptr<const RigExecSkinTopology>
CacheReference(const RigExecBakedProgramImpl &B, const Revision &revision,
               UsdTimeCode time)
{
    RigExecSkinTopologyCache cache;
    return RigExecResolveSkinTopology(revision.moverPrim,
                                      revision.influenceSlots.size(), time,
                                      B.resolvedInputs, &cache);
}

bool
SameLayout(const std::shared_ptr<const RigExecSkinTopology> &a,
           const std::shared_ptr<const RigExecSkinTopology> &b)
{
    return (!a && !b) || (a && b && *a == *b);
}

struct LayoutCase {
    std::string name;
    std::string path;
};

// Every revision's layout fields after Build, after baked runs at two
// frames, and after a bake's forced run, each checked against what the
// fields were set from and folded into one signature per moment. The
// signatures are printed so that two builds of this test can be compared
// line for line.
void
CheckLayoutFields(const std::string &what, const UsdStageRefPtr &stage,
                  const SdfPath &rigPath)
{
    RigExecRigEvaluator rig(stage, rigPath);
    CompileOrReport(&rig);
    std::vector<std::string> reasons;
    const std::unique_ptr<RigExecBakedProgram> program =
        RigExecBakedProgram::Build(&rig, &reasons);
    CHECK(program);
    if (!program) {
        std::printf("FAIL %s: no program\n", what.c_str());
        for (const std::string &reason : reasons) {
            std::printf("    refused: %s\n", reason.c_str());
        }
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    struct Built {
        std::vector<std::vector<int>> keys;
        size_t indexCount = 0, pointCount = 0;
        int elementSize = 0;
    };
    std::map<SdfPath, Built> built;
    std::map<SdfPath, bool> resolved;
    size_t fixed = 0, chunked = 0, layouts = 0;
    const auto forEach = [&B](const auto &fn) {
        for (const auto &chain : B.chains) {
            for (const Revision &revision : chain.revisions) {
                fn(revision, chain.haveBase);
            }
            for (const auto &derived : chain.derived) {
                fn(derived.revision,
                   derived.haveBase && !derived.matrixTarget);
            }
        }
    };
    forEach([&](const Revision &revision, bool) {
        Built &b = built[revision.moverPath];
        for (const auto &chunk : revision.chunks) {
            b.keys.push_back(chunk.key);
        }
        b.indexCount = revision.partitionIndexCount;
        b.pointCount = revision.partitionPointCount;
        b.elementSize = revision.partitionElementSize;
        fixed += revision.skinTopologyFixed ? 1 : 0;
        chunked += revision.chunked ? 1 : 0;
    });
    const auto moment = [&](const std::string &when, UsdTimeCode time,
                            bool ran) {
        Fnv f;
        size_t violations = 0;
        forEach([&](const Revision &revision, bool haveBase) {
            FoldLayoutFields(&f, revision);
            const std::string at =
                what + " " + when + " " + revision.moverPath.GetString();
            // Set when an owning body adopts a sampled SkinTopology handle.
            bool &expected = resolved[revision.moverPath];
            expected = expected ||
                       (ran && revision.op == rigExec::RigExecRevisionOp::Skin && haveBase);
            if (revision.topologyResolved != expected) {
                ++violations;
                std::printf("FAIL %s: topologyResolved %d, expected %d\n",
                            at.c_str(), int(revision.topologyResolved),
                            int(expected));
            }
            if (!revision.topologyResolved) {
                if (revision.topology) {
                    ++violations;
                    std::printf("FAIL %s: a layout nothing resolved\n",
                                at.c_str());
                }
                return;
            }
            if (ran && haveBase &&
                !SameLayout(revision.topology,
                            CacheReference(B, revision, time))) {
                ++violations;
                std::printf("FAIL %s: the layout differs from the cache's\n",
                            at.c_str());
            }
            layouts += ran && revision.topology ? 1 : 0;
            if (!revision.chunked || !revision.topology) {
                return;
            }
            const Built &b = built[revision.moverPath];
            std::vector<std::vector<int>> keys;
            for (const auto &chunk : revision.chunks) {
                keys.push_back(chunk.key);
            }
            if (revision.partitionTopology != revision.topology ||
                keys != b.keys ||
                revision.partitionIndexCount != b.indexCount ||
                revision.partitionPointCount != b.pointCount ||
                revision.partitionElementSize != b.elementSize) {
                ++violations;
                std::printf("FAIL %s: the partition is not Build's cut of "
                            "the adopted layout\n", at.c_str());
            }
        });
        failures += int(violations);
        std::printf("layout fields %s %s: %016llx\n", what.c_str(),
                    when.c_str(), static_cast<unsigned long long>(f.h));
    };
    const double start = stage->GetStartTimeCode();
    moment("build", UsdTimeCode(start), false);
    RigExecRigPose pose;
    CHECK(RunSkinProgramAtTime(*program, rig, UsdTimeCode(start), &pose));
    moment("run1", UsdTimeCode(start), true);
    CHECK(RunSkinProgramAtTime(*program, rig, UsdTimeCode(start + 2.0), &pose));
    moment("run3", UsdTimeCode(start + 2.0), true);
    program->RequestFullRun();
    CHECK(RunSkinProgramAtTime(*program, rig, UsdTimeCode(start + 2.0), &pose));
    moment("bake", UsdTimeCode(start + 2.0), true);
    std::printf("layout fields %s: %zu fixed, %zu chunked, %zu adopted\n",
                what.c_str(), fixed, chunked, layouts);
}

SdfPath
RigOf(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// The skinned example and fixture rigs, cut as Build cuts them and cut into
// chunks wherever a layout can be, plus a skin whose target has no points
// at any frame: its layout is never resolved, before the bake or after.
void
TestTheLayoutFieldsAreSetWhereTheyWere(const std::string &examples)
{
    const std::string fixtures = examples + "/../tests/fixtures/";
    const std::vector<LayoutCase> cases = {
        {"biped", examples + "/biped/Biped_anim.usda"},
        {"bust", examples + "/2d/bust/bust_anim.usda"},
        {"bust_dd_a", examples + "/2d/bust_dd_a/bust_dd_a_anim.usda"},
        {"rubberhose", examples + "/2d/rubberhose/rubberhose_anim.usda"},
        {"oneloop_cross_domain", fixtures + "oneloop_cross_domain.usda"},
        {"oneloop_two_limbs", fixtures + "oneloop_two_limbs.usda"},
        {"raw_skin_layouts", fixtures + "raw_skin_layouts.usda"},
        {"volume_placements", fixtures + "volume_placements.usda"},
    };
    for (const bool always : {false, true}) {
        if (always) {
            ArchSetEnv("RIGEXEC_BAKED_CHUNK_ALWAYS", "1", true);
            ArchSetEnv("RIGEXEC_BAKED_CHUNK_VERTS", "2", true);
        }
        for (const LayoutCase &c : cases) {
            const UsdStageRefPtr stage = UsdStage::Open(c.path);
            CHECK(stage);
            if (!stage) {
                std::printf("FAIL cannot open %s\n", c.path.c_str());
                continue;
            }
            if (c.name == "volume_placements") {
                // This parity history requests the original final-placement
                // field explicitly; unannotated fields retain the base default.
                const auto weight = stage->GetPrimAtPath(
                    SdfPath("/PlacementAsset/Rig/GeometryMovers/StripSmooth"))
                    .GetRelationship(TfToken("rigExec:weightObject"));
                CHECK(weight);
                if (weight) {
                    const UsdEditContext edit(stage, UsdEditTarget(stage->GetSessionLayer()));
                    CHECK(weight.SetMetadata(TfToken(RigExecReadPhaseMetadataName),
                                             std::string("final")));
                }
            }
            CheckLayoutFields(c.name + (always ? " chunked" : ""), stage,
                              RigOf(stage));
        }
        if (always) {
            ArchRemoveEnv("RIGEXEC_BAKED_CHUNK_ALWAYS");
            ArchRemoveEnv("RIGEXEC_BAKED_CHUNK_VERTS");
        }
    }
    // A skin whose target's points are blocked: the chain reads no base,
    // so the prologue resolves no layout for it, and the bake exports none.
    UsdStageRefPtr stage = MakeSkinnedRig();
    stage->GetPrimAtPath(kSkin)
        .CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(Weights(1.0f, 0.0f));
    stage->GetAttributeAtPath(kTarget).Block();
    CheckLayoutFields("baseless", stage, SdfPath("/Asset/Rig"));
}

// Edits the program reads nothing of, and one value it reads that is not a
// layout input: the layout handle stays the same object, the skin revision's
// packet compares equal, and no kernel re-runs.
void
TestTheLayoutHandleSurvivesUnrelatedNotices()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdPrim skin = stage->GetPrimAtPath(kSkin);
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(Weights(1.0f, 0.0f));
    auto evaluator = CompiledIn(stage, SdfPath("/Asset/Rig"), ReferenceChecksRequested());
    CHECK(evaluator->Evaluate(UsdTimeCode(1.0)).executedOpCount >
          0);
    CHECK(evaluator->Evaluate(UsdTimeCode(1.0)).executedOpCount ==
          0);
    const std::shared_ptr<const RigExecSkinTopology> held =
        Adopted(*evaluator);
    CHECK(held != nullptr);
    const auto unrelated = [&](const std::string &what, bool rebuilt=false) {
        const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(1.0));
        CheckSkinned(pose, 1.0f, 0.0f, what.c_str());
        if (!rebuilt && pose.executedOpCount != 0) {
            std::printf("held-layout notice %s executed %zu op(s)\n",
                        what.c_str(), size_t(pose.executedOpCount));
        }
        if (!rebuilt) {
            CHECK(pose.executedOpCount == 0);
        } else {
            // ORIGINAL requires no retained geometry kernels. The new control
            // still initializes through the common graph in this generation.
            CHECK(pose.valid && pose.executedOpCount > 0);
            const auto &B=evaluator->GetBakedProgram()->GetStepGraph();
            size_t geometryBodies=0;
            for(size_t i=0;i<B.steps.size();++i)
                if(i<B.opExecution.ran.size() && B.opExecution.ran[i] &&
                   (B.steps[i].kind==RigExecBakedStepKind::RevisionChunk ||
                    B.steps[i].kind==RigExecBakedStepKind::RevisionFuse))++geometryBodies;
            CHECK(geometryBodies == 0);
            CHECK(pose.controlFrames.count(SdfPath("/Asset/Rig/Extra")) == 1);
            const auto extra=B.index.find(SdfPath("/Asset/Rig/Extra"));
            CHECK(extra!=B.index.end());
            bool initialized=false;
            if(extra!=B.index.end())for(size_t i=0;i<B.steps.size();++i)
                if(B.steps[i].kind==RigExecBakedStepKind::AvarInputs &&
                   B.steps[i].object==extra->second && i<B.opExecution.ran.size() &&
                   B.opExecution.ran[i])initialized=true;
            CHECK(initialized);
        }
        CHECK(Adopted(*evaluator) == held);
        const Revision *revision = SkinRevision(*evaluator);
        CHECK(revision && !revision->staticDirty);
        CHECK(revision && revision->layoutHandle == held);
    };
    stage->DefinePrim(SdfPath("/Scratch"), TfToken("Scope"));
    unrelated("after a prim nothing reads");
    stage->GetPrimAtPath(kTarget.GetPrimPath())
        .CreateAttribute(TfToken("foo"), SdfValueTypeNames->Float)
        .Set(1.0f);
    unrelated("after an attribute on the mesh");
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    unrelated("after the mover's envelope, set to its value");

    // A new control is a new epoch: the program is rebuilt over the standing
    // one's geometry state, and its first build of the same layout hands
    // back the same object, so the kept packet still compares equal.
    const size_t builds = evaluator->GetBakedProgramBuildCount();
    const int oldAlongX=evaluator->GetBakedProgram()->GetStepGraph().index.at(SdfPath("/Asset/Rig/AlongX"));
    const auto reordered=stage->DefinePrim(SdfPath("/Asset/Rig/A0Extra"),TfToken("RigExecControl"));
    CHECK(reordered.GetAttribute(TfToken("avars:tx")).Set(3.0));
    stage->DefinePrim(SdfPath("/Asset/Rig/Extra"), TfToken("RigExecControl"));
    unrelated("after a rebuild", true);
    CHECK(evaluator->GetBakedProgram()->GetStepGraph().index.at(SdfPath("/Asset/Rig/AlongX"))!=oldAlongX);
    const auto reorderedPose=evaluator->Evaluate(UsdTimeCode(1.0));
    CHECK(reorderedPose.controlFrames.count(reordered.GetPath())==1);
    if(reorderedPose.controlFrames.count(reordered.GetPath()))
        CHECK(reorderedPose.controlFrames.at(reordered.GetPath()).Origin()==GfVec3d(3,0,0));
    unrelated("held after a rebuild");
    CHECK(evaluator->GetBakedProgramBuildCount() > builds);

    // A drag on a control reaches none of the layout's reads: no
    // SkinTopology op runs and the handle stands, on both edges. A drag on
    // the weights builds a new layout, and so does its lift.
    const auto layoutOpsRun = [&evaluator]() {
        const RigExecBakedProgram *program = evaluator->GetBakedProgram();
        if (!program) return size_t(0);
        const auto &B = program->GetStepGraph();
        size_t count = 0;
        for (size_t i = 0; i < B.steps.size() && i < B.opExecution.ran.size(); ++i)
            if (B.steps[i].kind == RigExecBakedStepKind::SkinTopology && B.opExecution.ran[i])
                ++count;
        return count;
    };
    RigExecValueOverride control;
    control.prim = SdfPath("/Asset/Rig/AlongX");
    control.attribute = TfToken("avars:tx");
    control.value = VtValue(20.0);
    evaluator->SetInteractiveOverrides({control});
    CheckSkinned(evaluator->Evaluate(UsdTimeCode(1.0)), 2.0f, 0.0f,
                 "during a control drag");
    CHECK(Adopted(*evaluator) == held);
    CHECK(layoutOpsRun() == 0);
    evaluator->ClearInteractiveOverrides();
    CheckSkinned(evaluator->Evaluate(UsdTimeCode(1.0)), 1.0f, 0.0f,
                 "after a control drag");
    CHECK(Adopted(*evaluator) == held);
    CHECK(layoutOpsRun() == 0);
    evaluator->SetInteractiveOverrides({WeightsOverride(Weights(0.0f, 1.0f))});
    CheckSkinned(evaluator->Evaluate(UsdTimeCode(1.0)), 0.0f, 1.0f,
                 "during a weights drag");
    const std::shared_ptr<const RigExecSkinTopology> dragged =
        Adopted(*evaluator);
    CHECK(dragged != nullptr && dragged != held);
    CHECK(layoutOpsRun() == 1);
    evaluator->ClearInteractiveOverrides();
    CheckSkinned(evaluator->Evaluate(UsdTimeCode(1.0)), 1.0f, 0.0f,
                 "after a weights drag");
    CHECK(Adopted(*evaluator) != dragged);
    CHECK(SameLayout(Adopted(*evaluator), held));
    CHECK(layoutOpsRun() == 1);
}

void TestRetainedSkinRebuildInvalidation()
{
    for(int mode=0;mode<2;++mode) {
        const auto stage=MakeSkinnedRig();const auto skin=stage->GetPrimAtPath(kSkin);
        CHECK(skin.CreateAttribute(TfToken("rigExec:jointWeights"),SdfValueTypeNames->FloatArray)
            .Set(Weights(1.0f,0.0f)));
        auto evaluator=CompiledIn(stage,SdfPath("/Asset/Rig"),ReferenceChecksRequested());
        CheckSkinned(evaluator->Evaluate(UsdTimeCode(1)),1.0f,0.0f,"before retained input change");
        CHECK(evaluator->Evaluate(UsdTimeCode(1)).executedOpCount==0);
        const size_t builds=evaluator->GetBakedProgramBuildCount();
        if(mode==0)CHECK(skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.5f));
        else CHECK(skin.GetRelationship(TfToken("rigExec:influences")).SetTargets(
            {SdfPath("/Asset/Rig/AlongY"),SdfPath("/Asset/Rig/AlongX")}));
        stage->DefinePrim(SdfPath("/Asset/Rig/A0Changed"),TfToken("RigExecControl"));
        const auto changed=evaluator->Evaluate(UsdTimeCode(1));
        CheckSkinned(changed,mode==0?0.5f:0.0f,mode==0?0.0f:1.0f,"changed retained inputs");
        CHECK(changed.diagnostics==(std::vector<std::string>{"structural edit: epoch rebuilt"}));
        CHECK(evaluator->GetBakedProgramBuildCount()>builds);
        const auto &B=evaluator->GetBakedProgram()->GetStepGraph();size_t kernels=0;
        for(size_t i=0;i<B.steps.size();++i)
            if(B.steps[i].kind==RigExecBakedStepKind::RevisionChunk && i<B.opExecution.ran.size() &&
               B.opExecution.ran[i])++kernels;
        CHECK(kernels>0);
        const auto held=evaluator->Evaluate(UsdTimeCode(1));
        CheckSkinned(held,mode==0?0.5f:0.0f,mode==0?0.0f:1.0f,"held changed inputs");
        CHECK(held.executedOpCount==0);
        CHECK(held.diagnostics.empty());
        CheckAgreesWithFresh("held changed retained inputs",stage,{},UsdTimeCode(1),held);
    }
}

// Topology is epoch state: an authored edit of the skin's indices or element
// size is not a value the standing program re-reads but a rebuild, decided
// by the program's own index (Stale) before the next generation, and the
// rebuilt program agrees with a fresh one -- for a layout of another count
// too. The weights stay values: their edit reaches the standing program.
void
TestATopologyEditRebuildsTheProgram()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdPrim skin = stage->GetPrimAtPath(kSkin);
    const UsdAttribute weights = skin.CreateAttribute(
        TfToken("rigExec:jointWeights"), SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));
    auto evaluator = CompiledIn(stage, SdfPath("/Asset/Rig"),
                                ReferenceChecksRequested());
    const UsdTimeCode t1(1.0);
    const std::vector<RigExecValueOverride> none;
    CheckSkinned(evaluator->Evaluate(t1), 1.0f, 0.0f, "topology, before");
    {
        const Revision *revision = SkinRevision(*evaluator);
        CHECK(revision && revision->layoutTopologyFixed &&
              revision->layoutFixed);
        // The topology reads are in the program's rebuild index; the
        // weights are not.
        const RigExecBakedProgram *program = evaluator->GetBakedProgram();
        CHECK(program != nullptr);
        if (!program) {
            return;
        }
        const RigExecBakedProgramImpl &B = program->GetStepGraph();
        CHECK(B.rebuild.count(
                  kSkin.AppendProperty(TfToken("rigExec:jointIndices"))) == 1);
        CHECK(B.rebuild.count(
                  kSkin.AppendProperty(TfToken("rigExec:elementSize"))) == 1);
        CHECK(B.rebuild.count(
                  kSkin.AppendProperty(TfToken("rigExec:jointWeights"))) == 0);
    }
    size_t builds = evaluator->GetBakedProgramBuildCount();

    // Every point on the other control: the same count, other indices.
    VtIntArray swapped(kPointCount * 2);
    for (size_t i = 0; i < kPointCount; ++i) {
        swapped[i * 2] = 1;
        swapped[i * 2 + 1] = 0;
    }
    skin.GetAttribute(TfToken("rigExec:jointIndices")).Set(swapped);
    CHECK(evaluator->GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Stale);
    RigExecRigPose pose = evaluator->Evaluate(t1);
    CHECK(evaluator->GetBakedProgramBuildCount() == builds + 1);
    CheckSkinned(pose, 0.0f, 1.0f, "topology, indices swapped");
    CheckAgreesWithFresh("topology, indices swapped", stage, none, t1, pose);

    // A layout of another count: one influence per point, every point on
    // the first control. Held to a fresh program at the generation after
    // the rebuild, whatever the rebuild itself reported.
    builds = evaluator->GetBakedProgramBuildCount();
    {
        SdfChangeBlock block;
        skin.GetAttribute(TfToken("rigExec:elementSize")).Set(1);
        skin.GetAttribute(TfToken("rigExec:jointIndices"))
            .Set(VtIntArray(kPointCount, 0));
        weights.Set(VtFloatArray(kPointCount, 1.0f));
    }
    CHECK(evaluator->GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Stale);
    CheckSkinned(evaluator->Evaluate(t1), 1.0f, 0.0f,
                 "topology, one influence per point");
    CHECK(evaluator->GetBakedProgramBuildCount() > builds);
    pose = evaluator->Evaluate(t1);
    CheckSkinned(pose, 1.0f, 0.0f, "topology, one influence per point, held");
    CheckAgreesWithFresh("topology, one influence per point", stage, none, t1,
                         pose);
    {
        const Revision *revision = SkinRevision(*evaluator);
        CHECK(revision && revision->layoutHandle &&
              revision->layoutHandle->elementSize == 1);
    }

    // The weights are values: a repaint reaches the standing program.
    builds = evaluator->GetBakedProgramBuildCount();
    weights.Set(VtFloatArray(kPointCount, 0.5f));
    CHECK(evaluator->GetLastNoticeDisposition() !=
          RigExecNoticeDisposition::Stale);
    pose = evaluator->Evaluate(t1);
    CheckSkinned(pose, 0.5f, 0.0f, "topology, weights repainted");
    CheckAgreesWithFresh("topology, weights repainted", stage, none, t1,
                         pose);
    CHECK(evaluator->GetBakedProgramBuildCount() == builds);
}

// Joint weights stay per-frame values over fixed indices: keyed weights are
// re-read at every frame by the standing program, which nothing rebuilds,
// while the indices and the element size, whose reads cannot vary, are
// topology the samples leave alone.
void
TestAnimatedWeightsOverFixedIndices()
{
    UsdStageRefPtr stage = MakeSkinnedRig();
    const UsdAttribute weights =
        stage->GetPrimAtPath(kSkin)
            .CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray);
    weights.Set(Weights(1.0f, 0.0f));
    weights.Set(Weights(1.0f, 0.0f), 1.0);
    weights.Set(Weights(0.0f, 1.0f), 2.0);
    weights.Set(Weights(0.5f, 0.5f), 3.0);
    auto evaluator = CompiledIn(stage, SdfPath("/Asset/Rig"),
                                ReferenceChecksRequested());
    CheckSkinned(evaluator->Evaluate(1.0), 1.0f, 0.0f,
                 "animated weights at 1");
    const size_t builds = evaluator->GetBakedProgramBuildCount();
    const Revision *revision = SkinRevision(*evaluator);
    CHECK(revision != nullptr);
    if (!revision) {
        return;
    }
    // The layout moves with the time; its topology half does not.
    CHECK(revision->layoutTopologyFixed);
    CHECK(!revision->layoutFixed);
    const RigExecBakedPathLeaves &leaves = revision->layoutLeaves;
    const int indices = leaves.decl.Role(RigExecRevisionLeafRole::JointIndices);
    const int size = leaves.decl.Role(RigExecRevisionLeafRole::ElementSize);
    const int painted =
        leaves.decl.Role(RigExecRevisionLeafRole::JointWeights);
    const auto flag = [](const std::vector<char> &flags, int k) {
        return k >= 0 && size_t(k) < flags.size() && flags[size_t(k)] != 0;
    };
    CHECK(flag(leaves.epoch, indices) && flag(leaves.epoch, size));
    CHECK(!flag(leaves.epoch, painted));
    CHECK(flag(leaves.varying, painted));
    CHECK(!flag(leaves.varying, indices) && !flag(leaves.varying, size));
    const std::vector<std::pair<double, std::pair<float, float>>> frames = {
        {2.0, {0.0f, 1.0f}}, {3.0, {0.5f, 0.5f}}, {1.0, {1.0f, 0.0f}}};
    for (const auto &[frame, expected] : frames) {
        const std::string what =
            "animated weights at " + std::to_string(int(frame));
        const RigExecRigPose pose = evaluator->Evaluate(frame);
        CheckSkinned(pose, expected.first, expected.second, what.c_str());
        CheckAgreesWithFresh(what, stage, {}, UsdTimeCode(frame), pose);
        // The frame's sample moved the weights and left the topology.
        CHECK(flag(leaves.changed, painted));
        CHECK(!flag(leaves.changed, indices) && !flag(leaves.changed, size));
    }
    CHECK(evaluator->GetBakedProgramBuildCount() == builds);
}

}  // namespace

int
main(int argc, char **argv)
{
    // The codeless schema has to be loadable before a RigExecControl means
    // anything to exec; ctest runs this with no plugin path set.
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);

    if (argc > 2 && std::string(argv[1]) == "--baked") {
        TestBakedLayoutFollowsEdits();
        TestATargetRefusalKeepsIndependentLayoutWork();
        TestTheLayoutFieldsAreSetWhereTheyWere(argv[2]);
        TestTheLayoutHandleSurvivesUnrelatedNotices();
        TestRetainedSkinRebuildInvalidation();
        TestATopologyEditRebuildsTheProgram();
        TestAnimatedWeightsOverFixedIndices();
        if (failures) {
            std::printf("%d FAILURE(S)\n", failures);
            return 1;
        }
        std::printf("testRigExecSkinTopology --baked: all tests passed\n");
        return 0;
    }
    TestWeightPaintEditAfterFirstEvaluate();
    TestTimeSampledWeightsRefuseTheCache();
    TestInteractiveOverrideOnTheLayout();
    TestConnectedWeightsStillEvaluate();
    TestBasePointEdits();
    TestPointCountChangeFailsAtomically();
    TestAPartialConstantEnvelopeIsApplied();
    TestWeightsBecomeTimeSampledAfterCompile();
    TestWeightsBecomeConnectedAfterCompile();
    TestAnUnrelatedEditDoesNotRerunTheKernel();
    TestAnOverrideUpstreamOfTheLayoutDropsTheCache();
    TestAnUnrelatedOverrideKeepsTheCache();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecSkinTopology: all tests passed\n");
    return 0;
}
