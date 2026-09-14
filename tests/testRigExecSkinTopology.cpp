//
// The skin layout is resolved once per binding epoch and shared, so these
// are the cases where "once per epoch" must not mean "once, ever":
//
//  - a weight-paint edit is a VALUE edit. It does not change the structure
//    digest, so no new epoch begins and nothing recompiles; the only thing
//    standing between the artist's new weights and a stale deformation is
//    the change notice dropping the cached layout.
//  - a layout that is time-varying, connected, or written by a property
//    chain is not epoch state at all. Compile has to refuse to cache it,
//    and the rig has to keep deforming correctly frame by frame.
//  - an interactive override is a value the static reads must prefer over
//    the stage, and the layout is read through exactly that route.
//
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/relationship.h"

#include <cmath>
#include <cstdio>
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

    // The other two layout attributes are equally value edits: swapping the
    // index of every second slot swaps which control drives it.
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
    evaluator.SetInteractiveOverrides({override});
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
//
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
//
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
//
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
    const size_t epoch = evaluator.GetBindingEpochDigest();
    // The first generation builds the graph and runs the kernel; the second,
    // identical, must run nothing. That is the baseline the case below is
    // measured against.
    CHECK(evaluator.Evaluate(1.0).moverGraphRevisionsExecuted > 0);
    CHECK(evaluator.Evaluate(1.0).moverGraphRevisionsExecuted == 0);

    // A prim the rig has never heard of, between two identical frames.
    stage->DefinePrim(SdfPath("/Scratch"), TfToken("Scope"));
    const RigExecRigPose after = evaluator.Evaluate(1.0);
    CheckSkinned(after, 1.0f, 0.0f, "after an unrelated edit");
    CHECK(evaluator.GetBindingEpochDigest() == epoch);
    CHECK(after.moverGraphRevisionsExecuted == 0);
    CHECK(after.moverGraphRevisionsCreated == 0);

    // And the cache is still a cache, not a freeze: a real weight edit after
    // the unrelated one still lands.
    stage->GetPrimAtPath(kSkin)
        .GetAttribute(TfToken("rigExec:jointWeights")).Set(Weights(0.0f, 1.0f));
    const RigExecRigPose repainted = evaluator.Evaluate(1.0);
    CheckSkinned(repainted, 0.0f, 1.0f, "repainted after an unrelated edit");
    CHECK(repainted.moverGraphRevisionsExecuted > 0);
}

}  // namespace

int
main()
{
    // The codeless schema has to be loadable before a RigExecControl means
    // anything to exec; ctest runs this with no plugin path set.
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);

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

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecSkinTopology: all tests passed\n");
    return 0;
}
