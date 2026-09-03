//
// Tests for the compiled mover graph (spec §7.2).
//
// Three halves, really: the revision ops themselves (every operation,
// composition, weighting, cardinality guards, pass-through paths); the binding
// resolution that replaces compiler-authored rigExec:resolved* wiring with
// build-time path choices; and packet assembly from provider values, which is
// what lets a revision run with no derived stage in existence.
//
#include "rigExec/moverGraph.h"
#include "rigExec/types.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/vt/array.h"
#include "pxr/exec/exec/typeRegistry.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>

PXR_NAMESPACE_USING_DIRECTIVE

using rigExec::RigExecMoverGraph;
using rigExec::RigExecMoverParameters;
using rigExec::RigExecMoverStatus;
using rigExec::RigExecRevisionBinding;
using rigExec::RigExecResolveRevisionBinding;
using rigExec::RigExecRevisionOp;
using rigExec::RigExecRevisionOpForSchema;
using rigExec::RigExecAssembleMatrixParameters;
using rigExec::RigExecStatusForParameters;
using rigExec::RigExecWeightPacket;
using rigExec::RigExecAssembleParameters;

static int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAILED: %s (line %d)\n", #cond, __LINE__);            \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

static bool
Near(const GfVec3f &a, const GfVec3f &b, double eps = 1e-5)
{
    return (GfVec3d(a) - GfVec3d(b)).GetLength() < eps;
}

static VtVec3fArray
MakePoints()
{
    VtVec3fArray points(4);
    points[0] = GfVec3f(0, 0, 0);
    points[1] = GfVec3f(1, 0, 0);
    points[2] = GfVec3f(0, 1, 0);
    points[3] = GfVec3f(0, 0, 1);
    return points;
}

static RigExecMoverParameters
MakeMatrixParams(const GfVec3d &translate, float weight)
{
    RigExecMoverParameters params;
    params.valid = true;
    params.kind = TfToken("matrix");
    params.transform = GfMatrix4d(1.0);
    params.transform.SetTranslate(translate);
    params.weights.representation = TfToken("constant");
    params.weights.defaultWeight = weight;
    params.weights.valid = true;
    return params;
}

static RigExecMoverStatus
MakeOkStatus()
{
    RigExecMoverStatus status;
    status.state = TfToken("ok");
    return status;
}

// One revision: a full-weight translate must move every point by the offset.
static void
TestSingleRevision()
{
    RigExecMoverGraph graph;
    const SdfPath target("/Asset/Geom/M.points");

    const VdfMaskedOutput base = graph.AddPointSource(target, MakePoints());
    const VdfMaskedOutput head = graph.AddRevision(
        RigExecRevisionOp::Matrix, base,
        MakeMatrixParams(GfVec3d(0, 2, 0), 1.0f), MakeOkStatus());

    CHECK(graph.GetRevisionCount() == 1);

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 2, 0)));
        CHECK(Near(out[1], GfVec3f(1, 2, 0)));
        CHECK(Near(out[2], GfVec3f(0, 3, 0)));
        CHECK(Near(out[3], GfVec3f(0, 2, 1)));
    }
}

// Two revisions on one target compose in chain order, which is the whole point
// of the namespace-ordered write-set model.
static void
TestChainedRevisions()
{
    RigExecMoverGraph graph;
    const SdfPath target("/Asset/Geom/M.points");

    VdfMaskedOutput head = graph.AddPointSource(target, MakePoints());
    head = graph.AddRevision(
        RigExecRevisionOp::Matrix, head,
        MakeMatrixParams(GfVec3d(0, 2, 0), 1.0f), MakeOkStatus());
    head = graph.AddRevision(
        RigExecRevisionOp::Matrix, head,
        MakeMatrixParams(GfVec3d(3, 0, 0), 1.0f), MakeOkStatus());

    CHECK(graph.GetRevisionCount() == 2);

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(3, 2, 0)));
        CHECK(Near(out[3], GfVec3f(3, 2, 1)));
    }
}

// Half weight interpolates toward the transformed position (spec §7.4).
static void
TestWeightedRevision()
{
    RigExecMoverGraph graph;
    const VdfMaskedOutput base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());
    const VdfMaskedOutput head = graph.AddRevision(
        RigExecRevisionOp::Matrix, base,
        MakeMatrixParams(GfVec3d(0, 4, 0), 0.5f), MakeOkStatus());

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 2, 0)));
    }
}

// A failed/disabled mover returns its preceding revision unchanged, with no
// partial write (spec §6.6).
static void
TestStatusPassThrough()
{
    RigExecMoverGraph graph;
    const VdfMaskedOutput base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    RigExecMoverStatus failed;
    failed.state = TfToken("failed");

    const VdfMaskedOutput head = graph.AddRevision(
        RigExecRevisionOp::Matrix, base,
        MakeMatrixParams(GfVec3d(0, 9, 0), 1.0f), failed);

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 0, 0)));
        CHECK(Near(out[1], GfVec3f(1, 0, 0)));
    }
}

// An invalid parameter packet passes through rather than computing garbage.
static void
TestInvalidParamsPassThrough()
{
    RigExecMoverGraph graph;
    const VdfMaskedOutput base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    RigExecMoverParameters invalid = MakeMatrixParams(GfVec3d(0, 9, 0), 1.0f);
    invalid.valid = false;

    const VdfMaskedOutput head = graph.AddRevision(
        RigExecRevisionOp::Matrix, base, invalid, MakeOkStatus());

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 0, 0)));
    }
}

// Blend deltas are added to the preceding revision, masked per element.
static void
TestBlendShapeRevision()
{
    RigExecMoverGraph graph;
    const VdfMaskedOutput base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    RigExecMoverParameters params;
    params.valid = true;
    params.kind = TfToken("blendShape");
    params.weights = RigExecWeightPacket::Constant(1.0f);
    params.blendDeltas = {GfVec3f(0, 1, 0), GfVec3f(0, 1, 0),
                          GfVec3f(0, 1, 0), GfVec3f(0, 1, 0)};

    const VdfMaskedOutput head = graph.AddRevision(
        RigExecRevisionOp::BlendShape, base, params, MakeOkStatus());

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 1, 0)));
        CHECK(Near(out[2], GfVec3f(0, 2, 0)));
    }
}

// A delta array that disagrees with the target cardinality fails the
// application atomically rather than writing part of it (spec §7.4).
static void
TestBlendCardinalityMismatchPassesThrough()
{
    RigExecMoverGraph graph;
    const VdfMaskedOutput base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    RigExecMoverParameters params;
    params.valid = true;
    params.kind = TfToken("blendShape");
    params.weights = RigExecWeightPacket::Constant(1.0f);
    params.blendDeltas = {GfVec3f(0, 1, 0)};  // 1 delta for 4 points

    const VdfMaskedOutput head = graph.AddRevision(
        RigExecRevisionOp::BlendShape, base, params, MakeOkStatus());

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 0, 0)));
        CHECK(Near(out[2], GfVec3f(0, 1, 0)));
    }
}

// Each op only accepts its own packet kind; anything else passes through,
// so a mis-bound revision can never silently compute the wrong operation.
static void
TestKindMismatchPassesThrough()
{
    RigExecMoverGraph graph;
    const VdfMaskedOutput base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    // A matrix packet handed to a smooth revision.
    const VdfMaskedOutput head = graph.AddRevision(
        RigExecRevisionOp::Smooth, base,
        MakeMatrixParams(GfVec3d(0, 5, 0), 1.0f), MakeOkStatus());

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 0, 0)));
        CHECK(Near(out[3], GfVec3f(0, 0, 1)));
    }
}

// Mixed ops compose in chain order on one target.
static void
TestMixedOpChain()
{
    RigExecMoverGraph graph;
    VdfMaskedOutput head =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    head = graph.AddRevision(
        RigExecRevisionOp::Matrix, head,
        MakeMatrixParams(GfVec3d(0, 2, 0), 1.0f), MakeOkStatus());

    RigExecMoverParameters blend;
    blend.valid = true;
    blend.kind = TfToken("blendShape");
    blend.weights = RigExecWeightPacket::Constant(1.0f);
    blend.blendDeltas = {GfVec3f(1, 0, 0), GfVec3f(1, 0, 0),
                         GfVec3f(1, 0, 0), GfVec3f(1, 0, 0)};
    head = graph.AddRevision(
        RigExecRevisionOp::BlendShape, head, blend, MakeOkStatus());

    CHECK(graph.GetRevisionCount() == 2);

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(1, 2, 0)));
    }
}

// The bindings the compiler used to author as rigExec:resolved* relationships
// are pure path resolution. These assert the resolver reproduces each one, so
// the graph can build the edge without anything being authored.
static void
TestRevisionBindingResolution()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath target("/Asset/Geom/M.points");
    stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));

    // Matrix: base phase binds the authored provider.
    {
        UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecMatrixMover"));
        mover.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({SdfPath("/Asset/Rig/Joints/J")});
        mover.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({SdfPath("/Asset/Rig/Weights/W")});

        const RigExecRevisionBinding b =
            RigExecResolveRevisionBinding(mover, target, {});
        CHECK(b.transform == SdfPath("/Asset/Rig/Joints/J"));
        CHECK(b.weightObject == SdfPath("/Asset/Rig/Weights/W"));

        // "final" swaps in the provider's frame-chain head instead.
        mover.CreateAttribute(TfToken("rigExec:transformReadPhase"),
                              SdfValueTypeNames->Token)
            .Set(TfToken("final"));
        const std::map<SdfPath, SdfPath> heads = {
            {SdfPath("/Asset/Rig/Joints/J"), SdfPath("/Asset/Gen/Head")}};
        const RigExecRevisionBinding f =
            RigExecResolveRevisionBinding(mover, target, heads);
        CHECK(f.transform == SdfPath("/Asset/Gen/Head"));
    }

    // Smooth: fixed adjacency comes from the destination's own topology.
    {
        UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Relax"), TfToken("RigExecSmoothMover"));
        const RigExecRevisionBinding b =
            RigExecResolveRevisionBinding(mover, target, {});
        CHECK(b.topologyCounts ==
              SdfPath("/Asset/Geom/M.faceVertexCounts"));
        CHECK(b.topologyIndices ==
              SdfPath("/Asset/Geom/M.faceVertexIndices"));
    }

    // Lattice: a cage prim binds to its .points by the standard rule.
    {
        UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Bulge"), TfToken("RigExecLatticeMover"));
        mover.CreateRelationship(TfToken("rigExec:cage"))
            .SetTargets({SdfPath("/Asset/Rig/Cage")});
        const RigExecRevisionBinding b =
            RigExecResolveRevisionBinding(mover, target, {});
        CHECK(b.cagePoints == SdfPath("/Asset/Rig/Cage.points"));
        CHECK(b.base == target);
    }

    // Surface: driver points and topology from the surface prim.
    {
        UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Drape"), TfToken("RigExecSurfaceMover"));
        mover.CreateRelationship(TfToken("rigExec:surface"))
            .SetTargets({SdfPath("/Asset/Geom/S")});
        const RigExecRevisionBinding b =
            RigExecResolveRevisionBinding(mover, target, {});
        CHECK(b.surfacePoints == SdfPath("/Asset/Geom/S.points"));
        CHECK(b.topologyCounts == SdfPath("/Asset/Geom/S.faceVertexCounts"));
    }

    // Blend: channels are sorted (target-list order is non-semantic).
    {
        UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Face"),
            TfToken("RigExecBlendShapeMover"));
        mover.CreateRelationship(TfToken("rigExec:blendInputs"))
            .SetTargets({SdfPath("/Asset/Rig/B/Smile"),
                         SdfPath("/Asset/Rig/B/Brow")});
        const RigExecRevisionBinding b =
            RigExecResolveRevisionBinding(mover, target, {});
        CHECK(b.blendInputs.size() == 2);
        if (b.blendInputs.size() == 2) {
            CHECK(b.blendInputs[0] == SdfPath("/Asset/Rig/B/Brow"));
            CHECK(b.blendInputs[1] == SdfPath("/Asset/Rig/B/Smile"));
        }
        CHECK(b.base == target);
    }
}

// Schema type selects the frozen operation; the curve mover branches on mode.
static void
TestRevisionOpForSchema()
{
    const TfToken none;
    CHECK(RigExecRevisionOpForSchema(TfToken("RigExecMatrixMover"), none) ==
          RigExecRevisionOp::Matrix);
    CHECK(RigExecRevisionOpForSchema(TfToken("RigExecSmoothMover"), none) ==
          RigExecRevisionOp::Smooth);
    CHECK(RigExecRevisionOpForSchema(TfToken("RigExecCurveMover"),
                                     TfToken("ribbon")) ==
          RigExecRevisionOp::Ribbon);
    CHECK(RigExecRevisionOpForSchema(TfToken("RigExecCurveMover"),
                                     TfToken("emitGuidePoints")) ==
          RigExecRevisionOp::EmitGuidePoints);
    // A non-mover type has no revision op rather than a defaulted one.
    CHECK(!RigExecRevisionOpForSchema(TfToken("RigExecJoint"), none));
}

// End to end with no derived stage: resolve the binding off the authored
// stage, assemble the packet from provider values, build the graph, evaluate.
// This is the whole replacement path for one revision.
static void
TestAssembleAndEvaluateWithoutDerivedStage()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
    const SdfPath target("/Asset/Geom/M.points");

    UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecMatrixMover"));
    mover.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({SdfPath("/Asset/Rig/Joints/J")});

    const RigExecRevisionBinding binding =
        RigExecResolveRevisionBinding(mover, target, {});
    CHECK(binding.transform == SdfPath("/Asset/Rig/Joints/J"));

    // Stands in for the tapped computeMatrix / computeWeightPacket results.
    GfMatrix4d transform(1.0);
    transform.SetTranslate(GfVec3d(0, 2, 0));
    RigExecWeightPacket weights;
    weights.representation = TfToken("constant");
    weights.defaultWeight = 1.0f;
    weights.valid = true;

    const RigExecMoverParameters params =
        RigExecAssembleMatrixParameters(mover, &transform, &weights);
    CHECK(params.valid);
    CHECK(params.enabled);
    CHECK(params.kind == TfToken("matrix"));

    const RigExecMoverStatus status =
        RigExecStatusForParameters(params, binding.moverPath);
    CHECK(status.AllowsApply());

    RigExecMoverGraph graph;
    const VdfMaskedOutput base = graph.AddPointSource(target, MakePoints());
    const VdfMaskedOutput head =
        graph.AddRevision(RigExecRevisionOp::Matrix, base, params, status);

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 2, 0)));
        CHECK(Near(out[2], GfVec3f(0, 3, 0)));
    }

    // Nothing was authored to express any of it.
    CHECK(!stage->GetPrimAtPath(SdfPath("/Asset/Rig/__RigExecGenerated")));
    CHECK(!mover.GetRelationship(TfToken("rigExec:resolvedTransform")));
    CHECK(!stage->GetRootLayer()->IsDirty() ||
          stage->GetRootLayer()->GetNumSubLayerPaths() == 0);
}

// A disabled mover is an ordinary pass-through, not a failure (spec §6.6).
static void
TestAssembleDisabledAndFailed()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecMatrixMover"));
    mover.CreateAttribute(TfToken("inputs:enabled"), SdfValueTypeNames->Bool)
        .Set(false);

    GfMatrix4d transform(1.0);
    RigExecWeightPacket weights;
    weights.valid = true;

    const RigExecMoverParameters disabled =
        RigExecAssembleMatrixParameters(mover, &transform, &weights);
    CHECK(disabled.valid);
    CHECK(!disabled.enabled);
    CHECK(RigExecStatusForParameters(disabled, mover.GetPath()).state ==
          TfToken("disabled"));

    // A missing provider value fails the application and records the address.
    UsdPrim enabledMover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Other"), TfToken("RigExecMatrixMover"));
    const RigExecMoverParameters failed =
        RigExecAssembleMatrixParameters(enabledMover, nullptr, &weights);
    CHECK(!failed.valid);
    const RigExecMoverStatus status =
        RigExecStatusForParameters(failed, enabledMover.GetPath());
    CHECK(status.state == TfToken("moverFailed"));
    CHECK(status.firstBadAddress == "/Asset/Rig/Movers/Other");
    CHECK(!status.AllowsApply());

    // A non-affine transform is rejected rather than applied (spec §7.4).
    GfMatrix4d skewed(1.0);
    skewed[0][3] = 0.5;
    const RigExecMoverParameters nonAffine =
        RigExecAssembleMatrixParameters(enabledMover, &skewed, &weights);
    CHECK(!nonAffine.valid);
}

// Non-matrix packets assemble from static stage reads plus provider values,
// with no derived stage and no rigExec:resolved* relationship involved.
static void
TestAssembleNonMatrixParameters()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdPrim mesh = stage->DefinePrim(SdfPath("/A/Geom/M"), TfToken("Mesh"));
    mesh.CreateAttribute(TfToken("faceVertexCounts"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray({3}));
    mesh.CreateAttribute(TfToken("faceVertexIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray({0, 1, 2}));
    const SdfPath target("/A/Geom/M.points");

    // Smooth: common envelope plus the destination's own topology.
    {
        UsdPrim mover = stage->DefinePrim(SdfPath("/A/Rig/Movers/Relax"),
                                          TfToken("RigExecSmoothMover"));
        mover.CreateAttribute(TfToken("inputs:defaultWeight"),
                              SdfValueTypeNames->Float)
            .Set(0.25f);
        const RigExecRevisionBinding b =
            RigExecResolveRevisionBinding(mover, target, {});
        const RigExecMoverParameters p = RigExecAssembleParameters(
            mover, RigExecRevisionOp::Smooth, b, {});
        CHECK(p.valid);
        CHECK(p.kind == TfToken("smooth"));
        CHECK(p.strength == 1.0f);
        CHECK(p.weights.valid);
        CHECK(p.weights.representation == TfToken("constant"));
        CHECK(p.weights.defaultWeight == 0.25f);
        CHECK(p.topologyCounts.size() == 1);
        CHECK(p.topologyIndices.size() == 3);
    }

    // Volume correct: the reference volume comes from the authored base.
    {
        UsdPrim mover = stage->DefinePrim(SdfPath("/A/Rig/Movers/Vol"),
                                          TfToken("RigExecVolumeCorrectMover"));
        const RigExecRevisionBinding b =
            RigExecResolveRevisionBinding(mover, target, {});

        rigExec::RigExecProviderValues values;
        values.basePoints = {GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                             GfVec3f(0, 1, 0), GfVec3f(0, 0, 1)};
        const RigExecMoverParameters p = RigExecAssembleParameters(
            mover, RigExecRevisionOp::VolumeCorrect, b, values);
        CHECK(p.valid);
        CHECK(p.referenceVolume > 0.0);

        // Without a base there is nothing to correct toward: MoverFailed.
        const RigExecMoverParameters none = RigExecAssembleParameters(
            mover, RigExecRevisionOp::VolumeCorrect, b, {});
        CHECK(!none.valid);
    }

    // Disabled short-circuits every op before any provider is consulted.
    {
        UsdPrim mover = stage->DefinePrim(SdfPath("/A/Rig/Movers/Off"),
                                          TfToken("RigExecSmoothMover"));
        mover.CreateAttribute(TfToken("inputs:enabled"),
                              SdfValueTypeNames->Bool)
            .Set(false);
        const RigExecMoverParameters p = RigExecAssembleParameters(
            mover, RigExecRevisionOp::Smooth,
            RigExecResolveRevisionBinding(mover, target, {}), {});
        CHECK(p.valid);
        CHECK(!p.enabled);
        CHECK(RigExecStatusForParameters(p, mover.GetPath()).state ==
              TfToken("disabled"));
    }
}

int
main(int argc, char **argv)
{
    // Force the exec type registry to run its registry functions. The other
    // suites get this for free by constructing an ExecUsdSystem; this one talks
    // to VDF directly, and without it RigExecMoverParameters/RigExecMoverStatus
    // are never handed to VdfExecutionTypeRegistry::Define and the executor
    // cannot tell the two apart.
    ExecTypeRegistry::GetInstance();

    TestSingleRevision();
    TestChainedRevisions();
    TestWeightedRevision();
    TestStatusPassThrough();
    TestInvalidParamsPassThrough();
    TestBlendShapeRevision();
    TestBlendCardinalityMismatchPassesThrough();
    TestKindMismatchPassesThrough();
    TestMixedOpChain();
    TestRevisionBindingResolution();
    TestRevisionOpForSchema();
    TestAssembleAndEvaluateWithoutDerivedStage();
    TestAssembleDisabledAndFailed();
    TestAssembleNonMatrixParameters();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecMoverGraph: all tests passed\n");
    return 0;
}
