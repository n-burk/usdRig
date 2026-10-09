// Tests for the compiled mover graph (spec §7.2).
// Three halves, really: the revision ops themselves (every operation,
// composition, weighting, cardinality guards, pass-through paths); the binding
// resolution that replaces compiler-authored rigExec:resolved* wiring with
// build-time path choices; and packet assembly from provider values, which is
// what lets a revision run with no derived stage in existence.
#include "rigExec/moverGraph.h"
#include "rigExecRevisionProgramTest.h"
#include "rigExec/types.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/latticeKernel.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

using rigExecTest::RevisionProgram;
using rigExecTest::RevisionValue;
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

static RigExecMoverParameters
MakeSkinParams(const std::vector<GfMatrix4d> &transforms,
               const std::vector<int> &indices,
               const std::vector<float> &weights, int elementSize,
               float envelope = 1.0f)
{
    RigExecMoverParameters params;
    params.valid = true;
    params.kind = TfToken("skin");
    params.skinTransforms = transforms;
    params.skinIndices = indices;
    params.skinWeights = weights;
    params.skinElementSize = elementSize;
    params.skinningMethod = TfToken("classicLinear");
    params.weights = RigExecWeightPacket::Constant(envelope);
    return params;
}

static RigExecMoverStatus
MakeOkStatus()
{
    RigExecMoverStatus status;
    status.state = TfToken("ok");
    return status;
}

// The multi-influence skin revision (RigExecSkinMover, classicLinear):
// p' = (1 - sum w) p + sum_i w_i T_i p per point, from the incoming revision.
static void
TestSkinRevision()
{
    const SdfPath target("/M.points");
    const VtVec3fArray base = MakePoints();
    GfMatrix4d t1(1.0), t2(1.0);
    t1.SetTranslate(GfVec3d(10, 0, 0));
    t2.SetTranslate(GfVec3d(0, 10, 0));
    const GfMatrix4d rt = GfMatrix4d(GfRotation(GfVec3d(0, 0, 1), 90.0),
                                     GfVec3d(1, 2, 3));
    const std::vector<GfMatrix4d> transforms = {t1, t2, rt};

    // Single influence at weight 1: identical to the sequential matrix
    // mover on the same transform, point for point.
    {
        rigExecTest::RevisionProgram graph;
        const auto source = graph.AddPointSource(target, base);
        const auto skin = graph.AddRevision(
            RigExecRevisionOp::Skin, source,
            MakeSkinParams(transforms, {2, 2, 2, 2}, {1, 1, 1, 1}, 1),
            MakeOkStatus());
        const VtVec3fArray skinned = graph.Evaluate(skin);
        CHECK(graph.GetRevisionStatus(skin).state == "ok");

        rigExecTest::RevisionProgram sequential;
        const auto seqSource = sequential.AddPointSource(target, base);
        RigExecMoverParameters matrix = MakeMatrixParams(GfVec3d(0), 1.0f);
        matrix.transform = rt;
        const VtVec3fArray moved = sequential.Evaluate(
            sequential.AddRevision(RigExecRevisionOp::Matrix, seqSource,
                                   matrix, MakeOkStatus()));
        CHECK(skinned.size() == 4 && moved.size() == 4);
        for (size_t i = 0; i < skinned.size() && i < moved.size(); ++i) {
            CHECK(Near(skinned[i], moved[i]));
            CHECK(Near(skinned[i], GfVec3f(rt.TransformAffine(GfVec3d(base[i])))));
        }
    }
    // Two translations at 0.5 / 0.5 on every point: the analytic midpoint
    // of T1 p and T2 p.
    {
        rigExecTest::RevisionProgram graph;
        const auto source = graph.AddPointSource(target, base);
        const auto skin = graph.AddRevision(
            RigExecRevisionOp::Skin, source,
            MakeSkinParams(transforms, {0, 1, 0, 1, 0, 1, 0, 1},
                           {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f}, 2),
            MakeOkStatus());
        const VtVec3fArray out = graph.Evaluate(skin);
        CHECK(out.size() == 4);
        for (size_t i = 0; i < out.size(); ++i) {
            CHECK(Near(out[i], base[i] + GfVec3f(5, 5, 0)));
        }
    }
    // Weights that do not sum to one: the shortfall keeps the rest point
    // (0.25 / 0.25 retains half of p), and all-zero weights leave a point
    // exactly where it was. Per-point layouts differ to prove the gather
    // indexes per point, not per mesh.
    {
        rigExecTest::RevisionProgram graph;
        const auto source = graph.AddPointSource(target, base);
        const auto skin = graph.AddRevision(
            RigExecRevisionOp::Skin, source,
            MakeSkinParams(transforms, {0, 1, 0, 1, 1, 0, 2, 2},
                           {0.25f, 0.25f, 0.0f, 0.0f, 1.0f, 0.0f, 0.5f, 0.5f}, 2),
            MakeOkStatus());
        const VtVec3fArray out = graph.Evaluate(skin);
        CHECK(out.size() == 4);
        if (out.size() == 4) {
            CHECK(Near(out[0], base[0] + GfVec3f(2.5f, 2.5f, 0)));
            CHECK(out[1] == base[1]);
            CHECK(Near(out[2], base[2] + GfVec3f(0, 10, 0)));
            CHECK(Near(out[3], GfVec3f(rt.TransformAffine(GfVec3d(base[3])))));
        }
    }
    // The common MoverAPI envelope mixes the skinned candidate back over
    // the incoming revision, as for every other point mover.
    {
        rigExecTest::RevisionProgram graph;
        const auto source = graph.AddPointSource(target, base);
        const auto skin = graph.AddRevision(
            RigExecRevisionOp::Skin, source,
            MakeSkinParams(transforms, {0, 0, 0, 0}, {1, 1, 1, 1}, 1, 0.5f),
            MakeOkStatus());
        const VtVec3fArray out = graph.Evaluate(skin);
        CHECK(out.size() == 4);
        for (size_t i = 0; i < out.size(); ++i) {
            CHECK(Near(out[i], base[i] + GfVec3f(5, 0, 0)));
        }
    }
    // The skin reads the INCOMING revision: a blend shape ahead of it is
    // skinned, as a skinCluster skins its input geometry.
    {
        rigExecTest::RevisionProgram graph;
        const auto source = graph.AddPointSource(target, base);
        RigExecMoverParameters blend;
        blend.valid = true;
        blend.kind = TfToken("blendShape");
        blend.weights = RigExecWeightPacket::Constant(1.0f);
        blend.blendDeltas.assign(4, GfVec3f(0, 0, 1));
        const auto blended = graph.AddRevision(
            RigExecRevisionOp::BlendShape, source, blend, MakeOkStatus());
        const auto skin = graph.AddRevision(
            RigExecRevisionOp::Skin, blended,
            MakeSkinParams(transforms, {0, 0, 0, 0}, {1, 1, 1, 1}, 1),
            MakeOkStatus());
        const VtVec3fArray out = graph.Evaluate(skin);
        CHECK(out.size() == 4);
        for (size_t i = 0; i < out.size(); ++i) {
            CHECK(Near(out[i], base[i] + GfVec3f(10, 0, 1)));
        }
    }
    // dualQuaternion on a single rotation + translation influence at
    // weight 1 is that transform, as the linear kernel is (the two methods
    // only part with two or more rotating influences).
    {
        rigExecTest::RevisionProgram graph;
        const auto source = graph.AddPointSource(target, base);
        RigExecMoverParameters dq =
            MakeSkinParams(transforms, {2, 2, 2, 2}, {1, 1, 1, 1}, 1);
        dq.skinningMethod = TfToken("dualQuaternion");
        const auto skin = graph.AddRevision(
            RigExecRevisionOp::Skin, source, dq, MakeOkStatus());
        const VtVec3fArray out = graph.Evaluate(skin);
        CHECK(graph.GetRevisionStatus(skin).state == "ok");
        CHECK(out.size() == 4);
        for (size_t i = 0; i < out.size(); ++i) {
            CHECK(Near(out[i], GfVec3f(rt.TransformAffine(GfVec3d(base[i])))));
        }
    }
    // Cardinality and range failures pass the incoming revision through
    // atomically and report moverFailed; so does a method token neither
    // kernel owns.
    {
        auto failing = [&](const RigExecMoverParameters &params) {
            rigExecTest::RevisionProgram graph;
            const auto source = graph.AddPointSource(target, base);
            const auto skin = graph.AddRevision(
                RigExecRevisionOp::Skin, source, params, MakeOkStatus());
            CHECK(graph.Evaluate(skin) == base);
            CHECK(graph.GetRevisionStatus(skin).state == "moverFailed");
        };
        failing(MakeSkinParams(transforms, {0, 0, 0}, {1, 1, 1}, 1));
        failing(MakeSkinParams(transforms, {0, 0, 0, 3}, {1, 1, 1, 1}, 1));
        failing(MakeSkinParams(transforms, {0, 0, 0, 0}, {1, 1, 1, -1}, 1));
        failing(MakeSkinParams(transforms, {0, 0, 0, 0}, {1, 1, 1, 1}, 0));
        RigExecMoverParameters unknown =
            MakeSkinParams(transforms, {0, 0, 0, 0}, {1, 1, 1, 1}, 1);
        unknown.skinningMethod = TfToken("bogus");
        failing(unknown);
    }
}

// A RigExecSkinMover prim resolves to the Skin op, binds its ordered
// influences, and assembles a packet from provider matrices with nothing
// authored beyond the mover itself.
static void
TestSkinBindingAndAssembly()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
    const SdfPath target("/Asset/Geom/M.points");
    UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    mover.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({SdfPath("/Asset/Rig/Joints/B"),
                     SdfPath("/Asset/Rig/Joints/A")});
    mover.CreateAttribute(TfToken("rigExec:elementSize"),
                          SdfValueTypeNames->Int).Set(2);
    mover.CreateAttribute(TfToken("rigExec:jointIndices"),
                          SdfValueTypeNames->IntArray)
        .Set(VtIntArray({0, 1, 0, 1, 0, 1, 0, 1}));
    mover.CreateAttribute(TfToken("rigExec:jointWeights"),
                          SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray({0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f}));

    CHECK(RigExecRevisionOpForSchema(TfToken("RigExecSkinMover"), TfToken()) ==
          RigExecRevisionOp::Skin);
    const RigExecRevisionBinding binding =
        RigExecResolveRevisionBinding(mover, target, {});
    CHECK(binding.influences.size() == 2);
    CHECK(binding.influences.size() == 2 &&
          binding.influences[0] == SdfPath("/Asset/Rig/Joints/B") &&
          binding.influences[1] == SdfPath("/Asset/Rig/Joints/A"));
    CHECK(binding.transform.IsEmpty());

    // "final" rebinding applies to every influence, as it does for the
    // matrix mover's one.
    CHECK(mover.GetRelationship(TfToken("rigExec:influences"))
              .SetMetadata(TfToken(rigExec::RigExecReadPhaseMetadataName),
                           std::string("final")));
    const RigExecRevisionBinding finalBinding = RigExecResolveRevisionBinding(
        mover, target,
        {{SdfPath("/Asset/Rig/Joints/A"), SdfPath("/Asset/Rig/Heads/A")}});
    CHECK(finalBinding.influences.size() == 2 &&
          finalBinding.influences[1] == SdfPath("/Asset/Rig/Heads/A"));

    GfMatrix4d tb(1.0), ta(1.0);
    tb.SetTranslate(GfVec3d(10, 0, 0));
    ta.SetTranslate(GfVec3d(0, 10, 0));
    const std::vector<GfMatrix4d> matrices = {tb, ta};
    rigExec::RigExecProviderValues values;
    values.influenceTransforms = &matrices;
    const RigExecMoverParameters params = RigExecAssembleParameters(
        mover, RigExecRevisionOp::Skin, binding, values);
    CHECK(params.valid);
    CHECK(params.kind == TfToken("skin"));
    CHECK(params.skinningMethod == TfToken("classicLinear"));
    CHECK(params.skinElementSize == 2);
    CHECK(params.skinTransforms.size() == 2);
    CHECK(RigExecStatusForParameters(params, binding.moverPath).AllowsApply());

    rigExecTest::RevisionProgram graph;
    const auto source = graph.AddPointSource(target, MakePoints());
    const VtVec3fArray out = graph.Evaluate(graph.AddRevision(
        RigExecRevisionOp::Skin, source, params,
        RigExecStatusForParameters(params, binding.moverPath)));
    CHECK(out.size() == 4);
    for (size_t i = 0; i < out.size(); ++i) {
        CHECK(Near(out[i], MakePoints()[i] + GfVec3f(5, 5, 0)));
    }

    // No providers, a layout that does not divide by elementSize, and an
    // index past the influence table each fail assembly.
    rigExec::RigExecProviderValues none;
    CHECK(!RigExecAssembleParameters(
        mover, RigExecRevisionOp::Skin, binding, none).valid);
    mover.GetAttribute(TfToken("rigExec:elementSize")).Set(3);
    CHECK(!RigExecAssembleParameters(
        mover, RigExecRevisionOp::Skin, binding, values).valid);
    mover.GetAttribute(TfToken("rigExec:elementSize")).Set(2);
    mover.GetAttribute(TfToken("rigExec:jointIndices"))
        .Set(VtIntArray({0, 1, 0, 1, 0, 1, 0, 2}));
    CHECK(!RigExecAssembleParameters(
        mover, RigExecRevisionOp::Skin, binding, values).valid);
    mover.GetAttribute(TfToken("rigExec:jointIndices"))
        .Set(VtIntArray({0, 1, 0, 1, 0, 1, 0, 1}));

    // dualQuaternion assembles (the packet carries the author's intent) and
    // the kernel honours it: two pure translations at 0.5 / 0.5 blend to
    // the same midpoint under either method.
    mover.CreateAttribute(TfToken("rigExec:skinningMethod"),
                          SdfValueTypeNames->Token)
        .Set(TfToken("dualQuaternion"));
    const RigExecMoverParameters dq = RigExecAssembleParameters(
        mover, RigExecRevisionOp::Skin, binding, values);
    CHECK(dq.valid);
    CHECK(dq.skinningMethod == TfToken("dualQuaternion"));
    rigExecTest::RevisionProgram dqGraph;
    const auto dqSource = dqGraph.AddPointSource(target, MakePoints());
    const auto dqHead = dqGraph.AddRevision(
        RigExecRevisionOp::Skin, dqSource, dq,
        RigExecStatusForParameters(dq, binding.moverPath));
    const VtVec3fArray dqOut = dqGraph.Evaluate(dqHead);
    CHECK(dqGraph.GetRevisionStatus(dqHead).state != "moverFailed");
    CHECK(dqOut.size() == 4);
    for (size_t i = 0; i < dqOut.size(); ++i) {
        CHECK(Near(dqOut[i], MakePoints()[i] + GfVec3f(5, 5, 0)));
    }

    // Nothing was authored to express any of it.
    CHECK(!stage->GetPrimAtPath(SdfPath("/Asset/Rig/__RigExecGenerated")));
    CHECK(!mover.GetRelationship(TfToken("rigExec:resolvedInfluences")));
}

// One revision: a full-weight translate must move every point by the offset.
static void
TestSingleRevision()
{
    rigExecTest::RevisionProgram graph;
    const SdfPath target("/Asset/Geom/M.points");

    const rigExecTest::RevisionValue base = graph.AddPointSource(target, MakePoints());
    const rigExecTest::RevisionValue head = graph.AddRevision(
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
    rigExecTest::RevisionProgram graph;
    const SdfPath target("/Asset/Geom/M.points");

    rigExecTest::RevisionValue head = graph.AddPointSource(target, MakePoints());
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
    rigExecTest::RevisionProgram graph;
    const rigExecTest::RevisionValue base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());
    const rigExecTest::RevisionValue head = graph.AddRevision(
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
    rigExecTest::RevisionProgram graph;
    const rigExecTest::RevisionValue base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    RigExecMoverStatus failed;
    failed.state = TfToken("failed");

    const rigExecTest::RevisionValue head = graph.AddRevision(
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
    rigExecTest::RevisionProgram graph;
    const rigExecTest::RevisionValue base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    RigExecMoverParameters invalid = MakeMatrixParams(GfVec3d(0, 9, 0), 1.0f);
    invalid.valid = false;

    const rigExecTest::RevisionValue head = graph.AddRevision(
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
    rigExecTest::RevisionProgram graph;
    const rigExecTest::RevisionValue base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    RigExecMoverParameters params;
    params.valid = true;
    params.kind = TfToken("blendShape");
    params.weights = RigExecWeightPacket::Constant(1.0f);
    params.blendDeltas = {GfVec3f(0, 1, 0), GfVec3f(0, 1, 0),
                          GfVec3f(0, 1, 0), GfVec3f(0, 1, 0)};

    const rigExecTest::RevisionValue head = graph.AddRevision(
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
    rigExecTest::RevisionProgram graph;
    const rigExecTest::RevisionValue base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    RigExecMoverParameters params;
    params.valid = true;
    params.kind = TfToken("blendShape");
    params.weights = RigExecWeightPacket::Constant(1.0f);
    params.blendDeltas = {GfVec3f(0, 1, 0)};  // 1 delta for 4 points

    const rigExecTest::RevisionValue head = graph.AddRevision(
        RigExecRevisionOp::BlendShape, base, params, MakeOkStatus());

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(graph.GetRevisionStatus(head).state == "moverFailed");
    CHECK(out.size() == 4);
    if (out.size() == 4) {
        CHECK(Near(out[0], GfVec3f(0, 0, 0)));
        CHECK(Near(out[2], GfVec3f(0, 1, 0)));
    }
    params.blendDeltas.assign(4, GfVec3f(0, 1, 0));
    CHECK(graph.UpdateRevision(head, params, MakeOkStatus()));
    CHECK(graph.Evaluate(head)[0] == GfVec3f(0, 1, 0));
    CHECK(graph.GetRevisionStatus(head).state == "ok");
}

// Each op only accepts its own packet kind; anything else passes through,
// so a mis-bound revision can never silently compute the wrong operation.
static void
TestKindMismatchPassesThrough()
{
    rigExecTest::RevisionProgram graph;
    const rigExecTest::RevisionValue base =
        graph.AddPointSource(SdfPath("/M.points"), MakePoints());

    // A matrix packet handed to a smooth revision.
    const rigExecTest::RevisionValue head = graph.AddRevision(
        RigExecRevisionOp::Smooth, base,
        MakeMatrixParams(GfVec3d(0, 5, 0), 1.0f), MakeOkStatus());

    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(graph.GetRevisionStatus(head).state == "moverFailed");
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
    rigExecTest::RevisionProgram graph;
    rigExecTest::RevisionValue head =
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
        CHECK(mover.GetRelationship(TfToken("rigExec:transform"))
                  .SetMetadata(TfToken(rigExec::RigExecReadPhaseMetadataName),
                               std::string("final")));
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

    rigExecTest::RevisionProgram graph;
    const rigExecTest::RevisionValue base = graph.AddPointSource(target, MakePoints());
    const rigExecTest::RevisionValue head =
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

// Cache intermediate revisions so edits resume at their dependency boundary,
// including status-only transitions and an unrelated chain in the same graph.
static void
TestIncrementalRevisionUpdates()
{
    rigExecTest::RevisionProgram graph;
    const VtVec3fArray initial = MakePoints();
    const rigExecTest::RevisionValue source =
        graph.AddPointSource(SdfPath("/M.points"), initial);
    const RigExecMoverParameters firstParams =
        MakeMatrixParams(GfVec3d(0, 2, 0), 1.0f);
    const RigExecMoverParameters secondParams =
        MakeMatrixParams(GfVec3d(0, 3, 0), 1.0f);
    const rigExecTest::RevisionValue first = graph.AddRevision(
        RigExecRevisionOp::Matrix, source, firstParams, MakeOkStatus());
    const rigExecTest::RevisionValue second = graph.AddRevision(
        RigExecRevisionOp::Matrix, first, secondParams, MakeOkStatus());
    const rigExecTest::RevisionValue otherSource =
        graph.AddPointSource(SdfPath("/Other.points"), initial);
    const rigExecTest::RevisionValue other = graph.AddRevision(
        RigExecRevisionOp::Matrix, otherSource, firstParams, MakeOkStatus());

    auto checkFirst = [&](const rigExecTest::RevisionValue &output, const GfVec3f &expected) {
        const VtVec3fArray result = graph.Evaluate(output);
        CHECK(result.size() == initial.size());
        if (!result.empty()) {
            CHECK(Near(result[0], expected));
        }
    };
    checkFirst(second, GfVec3f(0, 5, 0));
    CHECK(graph.GetRevisionExecutionCount() == 3);
    CHECK(graph.GetScheduleBuildCount() == 1);
    checkFirst(first, GfVec3f(0, 2, 0));
    checkFirst(second, GfVec3f(0, 5, 0));
    CHECK(graph.GetRevisionExecutionCount() == 3);

    CHECK(graph.UpdatePointSource(source, initial));
    CHECK(graph.UpdateRevision(second, secondParams, MakeOkStatus()));
    checkFirst(second, GfVec3f(0, 5, 0));
    CHECK(graph.GetRevisionExecutionCount() == 3);

    RigExecMoverParameters changed =
        MakeMatrixParams(GfVec3d(0, 6, 0), 1.0f);
    CHECK(graph.UpdateRevision(second, changed, MakeOkStatus()));
    checkFirst(second, GfVec3f(0, 8, 0));
    CHECK(graph.GetRevisionExecutionCount() == 4);

    RigExecMoverStatus disabled;
    disabled.state = TfToken("disabled");
    CHECK(graph.UpdateRevision(second, changed, disabled));
    checkFirst(second, GfVec3f(0, 2, 0));
    CHECK(graph.GetRevisionExecutionCount() == 5);
    CHECK(graph.UpdateRevision(second, changed, MakeOkStatus()));
    checkFirst(second, GfVec3f(0, 8, 0));
    CHECK(graph.GetRevisionExecutionCount() == 6);

    checkFirst(other, GfVec3f(0, 2, 0));
    CHECK(graph.GetRevisionExecutionCount() == 6);
    VtVec3fArray edited = initial;
    edited[0] = GfVec3f(1, 0, 0);
    CHECK(graph.UpdatePointSource(source, edited));
    checkFirst(second, GfVec3f(1, 8, 0));
    checkFirst(other, GfVec3f(0, 2, 0));
    CHECK(graph.GetRevisionExecutionCount() == 8);
    CHECK(graph.GetScheduleBuildCount() == 1);
    CHECK(graph.GetRevisionCount() == 3);

    CHECK(!graph.UpdatePointSource(source, VtVec3fArray(1)));
    CHECK(!graph.UpdatePointSource(second, initial));
    CHECK(!graph.UpdateRevision(source, changed, MakeOkStatus()));
    CHECK(graph.Evaluate(rigExecTest::RevisionValue()).empty());
    checkFirst(second, GfVec3f(1, 8, 0));
    CHECK(graph.GetRevisionExecutionCount() == 8);
}

static void
TestLongChainDirtySuffix()
{
    rigExecTest::RevisionProgram graph;
    const VtVec3fArray initial({GfVec3f(0, 0, 0)});
    const rigExecTest::RevisionValue source =
        graph.AddPointSource(SdfPath("/M.points"), initial);
    rigExecTest::RevisionValue head = source;
    std::vector<rigExecTest::RevisionValue> revisions;
    const size_t count = 2048;
    const RigExecMoverParameters step =
        MakeMatrixParams(GfVec3d(0, 1, 0), 1.0f);
    for (size_t i = 0; i < count; ++i) {
        head = graph.AddRevision(
            RigExecRevisionOp::Matrix, head, step, MakeOkStatus());
        revisions.push_back(head);
    }
    const VtVec3fArray first = graph.Evaluate(head);
    CHECK(first.size() == 1);
    if (!first.empty()) {
        CHECK(Near(first[0], GfVec3f(0, float(count), 0)));
    }
    CHECK(graph.GetRevisionExecutionCount() == count);
    CHECK(graph.GetScheduleBuildCount() == 1);
    CHECK(graph.UpdateRevision(revisions[count - 2],
        MakeMatrixParams(GfVec3d(0, 2, 0), 1.0f), MakeOkStatus()));
    const VtVec3fArray changed = graph.Evaluate(head);
    CHECK(changed.size() == 1);
    if (!changed.empty()) {
        CHECK(Near(changed[0], GfVec3f(0, float(count + 1), 0)));
    }
    CHECK(graph.GetRevisionExecutionCount() == count + 2);
    CHECK(graph.GetScheduleBuildCount() == 1);
    CHECK(graph.GetRevisionCount() == count);
}

// Extending topology should invalidate the schedule while preserving already
// computed prefixes. Updates made before the first evaluation are supported.
static void
TestAppendAfterEvaluation()
{
    rigExecTest::RevisionProgram graph;
    const rigExecTest::RevisionValue source = graph.AddPointSource(
        SdfPath("/M.points"), VtVec3fArray({GfVec3f(0, 0, 0)}));
    CHECK(graph.UpdatePointSource(source, VtVec3fArray({GfVec3f(1, 0, 0)})));
    const auto step = MakeMatrixParams(GfVec3d(0, 1, 0), 1.0f);
    const rigExecTest::RevisionValue first = graph.AddRevision(
        RigExecRevisionOp::Matrix, source, step, MakeOkStatus());
    CHECK(graph.Evaluate(first).size() == 1);
    const rigExecTest::RevisionValue second = graph.AddRevision(
        RigExecRevisionOp::Matrix, first, step, MakeOkStatus());
    CHECK(graph.UpdateRevision(second,
        MakeMatrixParams(GfVec3d(0, 2, 0), 1.0f), MakeOkStatus()));
    const VtVec3fArray result = graph.Evaluate(second);
    CHECK(result.size() == 1);
    if (!result.empty()) {
        CHECK(Near(result[0], GfVec3f(1, 3, 0)));
    }
    CHECK(graph.GetRevisionExecutionCount() == 2);
    CHECK(graph.GetScheduleBuildCount() == 2);
}

// Every operation consumes mutable packets. Compare an edited persistent
// graph with a freshly built graph, and require the edit to visibly change
// its result so a pass-through or accidentally unused input cannot pass.
static void
TestEveryOperationUpdatesInteractively()
{
    auto check = [&](RigExecRevisionOp op, const VtVec3fArray &base,
                     const RigExecMoverParameters &before,
                     const RigExecMoverParameters &after) {
        rigExecTest::RevisionProgram graph;
        const auto source = graph.AddPointSource(SdfPath("/M.points"), base);
        const auto head = graph.AddRevision(op, source, before, MakeOkStatus());
        const VtVec3fArray initial = graph.Evaluate(head);
        CHECK(graph.UpdateRevision(head, after, MakeOkStatus()));
        const VtVec3fArray updated = graph.Evaluate(head);
        rigExecTest::RevisionProgram fresh;
        const auto freshSource = fresh.AddPointSource(SdfPath("/M.points"), base);
        const VtVec3fArray expected = fresh.Evaluate(
            fresh.AddRevision(op, freshSource, after, MakeOkStatus()));
        CHECK(updated.size() == base.size());
        CHECK(updated == expected);
        CHECK(updated != initial);
        CHECK(graph.GetRevisionExecutionCount() == 2);
        CHECK(graph.GetScheduleBuildCount() == 1);
        CHECK(graph.UpdateRevision(head, after, MakeOkStatus()));
        CHECK(graph.Evaluate(head) == updated);
        CHECK(graph.GetRevisionExecutionCount() == 2);
    };
    auto packet = [](const char *kind) {
        RigExecMoverParameters p;
        p.kind = TfToken(kind);
        p.valid = true;
        p.strength = 1.0f;
        p.weights = RigExecWeightPacket::Constant(1.0f);
        return p;
    };
    const VtVec3fArray tetra = MakePoints();
    {
        auto a = MakeSkinParams(
            {GfMatrix4d(1.0), GfMatrix4d(1.0).SetTranslate(GfVec3d(0, 4, 0))},
            {0, 1, 0, 1, 0, 1, 0, 1},
            {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f}, 2);
        auto b = a;
        b.skinWeights[0] = 0.0f;
        b.skinWeights[1] = 1.0f;
        check(RigExecRevisionOp::Skin, tetra, a, b);
    }
    {
        auto a = packet("blendShape");
        a.blendDeltas.assign(4, GfVec3f(0, 0, 0));
        auto b = a;
        b.blendDeltas[0] = GfVec3f(0, 3, 0);
        check(RigExecRevisionOp::BlendShape, tetra, a, b);
    }
    {
        auto a = packet("volumeCorrect");
        a.referenceVolume = 1;
        auto b = a;
        b.referenceVolume = 8;
        check(RigExecRevisionOp::VolumeCorrect, tetra, a, b);
    }
    {
        auto a = packet("smooth");
        a.topologyCounts = {3};
        a.topologyIndices = {0, 1, 2};
        auto b = a;
        b.topologyIndices = {1, 2, 3};
        check(RigExecRevisionOp::Smooth, tetra, a, b);
    }
    {
        auto a = packet("lattice");
        a.divisions = GfVec3i(2, 2, 2);
        a.restPoints.assign(tetra.begin(), tetra.end());
        for (int z = 0; z < 2; ++z) {
            for (int y = 0; y < 2; ++y) {
                for (int x = 0; x < 2; ++x) {
                    a.auxPoints.emplace_back(float(x), float(y), float(z));
                }
            }
        }
        a.auxPointsB = a.auxPoints;
        a.auxPointsB[0] += GfVec3f(0, 0, 2);
        auto b = a;
        // Moving a rest point changes its cage bind coordinates immediately.
        b.restPoints[0] = GfVec3f(0.5f, 0.5f, 0.5f);
        check(RigExecRevisionOp::Lattice, tetra, a, b);
    }
    {
        auto a = packet("surfaceProject");
        a.auxPoints = {GfVec3f(0, 0, 0), GfVec3f(5, 0, 0), GfVec3f(0, 5, 0)};
        a.topologyCounts = {3};
        a.topologyIndices = {0, 1, 2};
        auto b = a;
        for (auto &point : b.auxPoints) point[2] += 2;
        check(RigExecRevisionOp::SurfaceProject, tetra, a, b);
    }
    {
        auto a = packet("emitGuidePoints");
        a.frames.frames.resize(4);
        a.frames.rests.resize(4, a.frames.frames[0].points);
        auto b = a;
        for (auto &point : b.frames.frames[0].points) point[1] += 3;
        check(RigExecRevisionOp::EmitGuidePoints, tetra, a, b);
        a.kind = b.kind = TfToken("ribbon");
        a.bindCoords.assign(4, GfVec2f(0, 0));
        b = a;
        // A rest-frame edit changes the rest-relative driver transformation.
        for (auto &point : b.frames.rests[0]) point[1] += 3;
        check(RigExecRevisionOp::Ribbon, tetra, a, b);
    }
    {
        auto a = packet("recomputeNormals");
        a.auxPoints.assign(tetra.begin(), tetra.end());
        a.topologyCounts = {3};
        a.topologyIndices = {0, 1, 2};
        auto b = a;
        b.auxPoints[2] = GfVec3f(0, 0, 1);
        check(RigExecRevisionOp::RecomputeNormals, tetra, a, b);
        a.kind = TfToken("recomputeExtent");
        b = a;
        b.widths = {2.0f};
        check(RigExecRevisionOp::RecomputeExtent,
              VtVec3fArray({GfVec3f(0), GfVec3f(0)}), a, b);
    }
}

// A revision's decision, made from its packet before any point is written,
// is the answer its kernel gives: Refuses where RigExecRunRevisionKernel
// fails and Applies where it succeeds, for each separable operation class
// over a valid packet and each validation its kernel runs first. Every case
// runs with no wire basis cache, then twice through one (a build, then a
// memo hit), so a memoized basis answers as the build did. Deferred cases
// only pin the classification.
static void
TestRevisionAcceptanceMatchesKernel()
{
    using rigExec::RigExecRevisionAcceptance;
    const VtVec3fArray tetra = MakePoints();
    const std::vector<GfVec3f> points(tetra.begin(), tetra.end());
    size_t compared = 0;
    const auto check = [&](const char *what, RigExecRevisionOp op,
                           const RigExecMoverParameters &p,
                           const std::vector<GfVec3f> &base,
                           RigExecRevisionAcceptance expected) {
        const RigExecRevisionAcceptance decided =
            rigExec::RigExecRevisionKernelAcceptance(op, p, base.size());
        if (decided != expected) {
            std::printf("  %s: decided %d, expected %d\n", what, int(decided),
                        int(expected));
        }
        CHECK(decided == expected);
        rigExec::RigExecWireBasisCache cache;
        for (int run = 0;
             run < 3 && decided != RigExecRevisionAcceptance::Deferred;
             ++run) {
            std::vector<GfVec3f> pts = base;
            const bool applied = rigExec::RigExecRunRevisionKernel(
                op, p, &pts, false, run == 0 ? nullptr : &cache);
            if (applied != (decided == RigExecRevisionAcceptance::Applies)) {
                std::printf("  %s, run %d: the kernel %s, the decision %d\n",
                            what, run, applied ? "applied" : "refused",
                            int(decided));
                CHECK(false);
            }
        }
        // The envelope's own decision is its resolve's.
        std::vector<float> resolved;
        CHECK(p.weights.ResolvesAll(base.size()) ==
              p.weights.ResolveAll(base.size(), &resolved));
        ++compared;
    };
    const auto envelope = [](const char *representation,
                            std::vector<float> values,
                            std::vector<int> indices, float defaultWeight) {
        RigExecWeightPacket packet;
        packet.representation = TfToken(representation);
        packet.rangePolicy = TfToken("strict");
        packet.values = std::move(values);
        packet.indices = std::move(indices);
        packet.defaultWeight = defaultWeight;
        packet.valid = true;
        return packet;
    };
    const RigExecRevisionAcceptance applies =
        RigExecRevisionAcceptance::Applies;
    const RigExecRevisionAcceptance refuses =
        RigExecRevisionAcceptance::Refuses;
    const RigExecRevisionAcceptance deferred =
        RigExecRevisionAcceptance::Deferred;

    // Matrix: full strength, a resolved constant, a dense field, a sparse
    // walk, and a sparse field with a default (resolved densely).
    const RigExecMoverParameters matrix =
        MakeMatrixParams(GfVec3d(0, 2, 0), 1.0f);
    check("matrix", RigExecRevisionOp::Matrix, matrix, points, applies);
    check("matrix at half", RigExecRevisionOp::Matrix,
          MakeMatrixParams(GfVec3d(0, 2, 0), 0.5f), points, applies);
    const RigExecWeightPacket dense =
        envelope("dense", {1.0f, 0.5f, 0.25f, 0.0f}, {}, 0.0f);
    const RigExecWeightPacket shortDense =
        envelope("dense", {1.0f, 0.5f, 0.25f}, {}, 0.0f);
    const RigExecWeightPacket sparse =
        envelope("sparse", {0.5f, 1.0f}, {1, 3}, 0.0f);
    {
        auto p = matrix;
        p.weights = dense;
        check("matrix, dense", RigExecRevisionOp::Matrix, p, points, applies);
        p.weights = shortDense;
        check("matrix, dense one weight short", RigExecRevisionOp::Matrix, p,
              points, refuses);
        p.weights = dense;
        p.weights.values[2] = 1.5f;
        check("matrix, dense weight above 1", RigExecRevisionOp::Matrix, p,
              points, refuses);
        p.weights = sparse;
        check("matrix, sparse walk", RigExecRevisionOp::Matrix, p, points,
              applies);
        p.weights.indices = {1, 9};
        check("matrix, sparse walk past the points",
              RigExecRevisionOp::Matrix, p, points, refuses);
        p.weights.indices = {3, 1};
        check("matrix, sparse walk descending", RigExecRevisionOp::Matrix, p,
              points, refuses);
        p.weights = sparse;
        p.weights.values[0] = std::numeric_limits<float>::quiet_NaN();
        check("matrix, sparse walk NaN weight", RigExecRevisionOp::Matrix, p,
              points, refuses);
        p.weights = sparse;
        p.weights.defaultWeight = 0.25f;
        check("matrix, sparse with a default", RigExecRevisionOp::Matrix, p,
              points, applies);
        p = matrix;
        p.valid = false;
        check("matrix, invalid packet", RigExecRevisionOp::Matrix, p, points,
              refuses);
        check("matrix packet on a blend shape", RigExecRevisionOp::BlendShape,
              matrix, points, refuses);
    }

    // Blend shape: the deltas and the envelope; a surface-frame transport
    // reads the entering points, so only its kernel answers.
    {
        RigExecMoverParameters blend;
        blend.valid = true;
        blend.kind = TfToken("blendShape");
        blend.weights = RigExecWeightPacket::Constant(1.0f);
        blend.blendDeltas.assign(4, GfVec3f(0, 1, 0));
        check("blend shape", RigExecRevisionOp::BlendShape, blend, points,
              applies);
        auto p = blend;
        p.weights = RigExecWeightPacket::Constant(0.5f);
        check("blend shape at half", RigExecRevisionOp::BlendShape, p, points,
              applies);
        p.weights = dense;
        check("blend shape, dense", RigExecRevisionOp::BlendShape, p, points,
              applies);
        p.weights = shortDense;
        check("blend shape, envelope one weight short",
              RigExecRevisionOp::BlendShape, p, points, refuses);
        p = blend;
        p.blendDeltas.pop_back();
        check("blend shape, one delta short", RigExecRevisionOp::BlendShape,
              p, points, refuses);
        p = blend;
        p.blendSurfaceFrame = true;
        check("blend shape on the surface frame",
              RigExecRevisionOp::BlendShape, p, points, deferred);
    }

    // Wire: a dense walk (one bind coordinate per point, its envelope
    // blended after the kernel) and a sparse walk (one bind coordinate per
    // weighted point, or per mesh point).
    {
        RigExecMoverParameters wire;
        wire.valid = true;
        wire.kind = TfToken("wire");
        wire.weights = RigExecWeightPacket::Constant(1.0f);
        wire.curveOrder = 2;
        wire.curveKnots = {0.0, 0.0, 1.0, 1.0};
        wire.restPoints = {GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)};
        wire.auxPoints = {GfVec3f(0, 0, 0), GfVec3f(1, 1, 0)};
        wire.wireBindCoords =
            VtArray<GfVec2f>{GfVec2f(0.25f, 0.0f), GfVec2f(0.5f, 0.0f),
                              GfVec2f(0.75f, 0.0f), GfVec2f(1.0f, 0.0f)};
        check("wire", RigExecRevisionOp::Wire, wire, points, applies);
        auto p = wire;
        p.weights = RigExecWeightPacket::Constant(0.5f);
        check("wire at half", RigExecRevisionOp::Wire, p, points, applies);
        p.weights = shortDense;
        check("wire, separate envelope one weight short",
              RigExecRevisionOp::Wire, p, points, refuses);
        p = wire;
        p.wireBindCoords = VtArray<GfVec2f>{GfVec2f(0.25f, 0.0f)};
        check("wire, one bind coordinate", RigExecRevisionOp::Wire, p, points,
              refuses);
        p = wire;
        p.auxPoints.push_back(GfVec3f(2, 0, 0));
        check("wire, polygons of unlike size", RigExecRevisionOp::Wire, p,
              points, refuses);
        p = wire;
        p.curveKnots[0] = std::numeric_limits<double>::quiet_NaN();
        check("wire, NaN knot", RigExecRevisionOp::Wire, p, points, refuses);
        p = wire;
        p.curveOrder = 1;
        check("wire, order 1", RigExecRevisionOp::Wire, p, points, refuses);
        p = wire;
        p.wireBindCoords = VtArray<GfVec2f>();
        check("wire over no points", RigExecRevisionOp::Wire, p, {},
              deferred);

        auto walk = wire;
        walk.weights = envelope("sparse", {1.0f, 0.5f}, {0, 2}, 0.0f);
        walk.wireBindCoords =
            VtArray<GfVec2f>{GfVec2f(0.25f, 0.0f), GfVec2f(0.75f, 0.0f)};
        check("wire, sparse walk", RigExecRevisionOp::Wire, walk, points,
              applies);
        p = walk;
        p.wireBindCoords = wire.wireBindCoords;
        check("wire, sparse walk over the mesh's binds",
              RigExecRevisionOp::Wire, p, points, applies);
        p.wireBindCoords =
            VtArray<GfVec2f>{GfVec2f(0.25f, 0.0f), GfVec2f(0.5f, 0.0f),
                              GfVec2f(0.75f, 0.0f)};
        check("wire, sparse walk with three binds",
              RigExecRevisionOp::Wire, p, points, refuses);
        p = walk;
        p.weights.indices = {0, 9};
        check("wire, sparse walk past the points", RigExecRevisionOp::Wire,
              p, points, refuses);
    }

    // Lattice: the kernel refuses only a rest-point count other than the
    // entering one, and then its separate envelope; an invalid cage passes
    // every point through, which applies.
    {
        RigExecMoverParameters lattice;
        lattice.valid = true;
        lattice.kind = TfToken("lattice");
        lattice.weights = RigExecWeightPacket::Constant(1.0f);
        lattice.divisions = GfVec3i(2, 2, 2);
        lattice.restPoints = points;
        for (int z = 0; z < 2; ++z) {
            for (int y = 0; y < 2; ++y) {
                for (int x = 0; x < 2; ++x) {
                    lattice.auxPoints.emplace_back(float(x), float(y),
                                                   float(z));
                }
            }
        }
        lattice.auxPointsB = lattice.auxPoints;
        lattice.auxPointsB[7] += GfVec3f(0, 0, 1);
        check("lattice", RigExecRevisionOp::Lattice, lattice, points,
              applies);
        auto p = lattice;
        p.weights = RigExecWeightPacket::Constant(0.5f);
        check("lattice at half", RigExecRevisionOp::Lattice, p, points,
              applies);
        p.weights = dense;
        check("lattice, dense envelope", RigExecRevisionOp::Lattice, p,
              points, applies);
        p.weights = shortDense;
        check("lattice, envelope one weight short",
              RigExecRevisionOp::Lattice, p, points, refuses);
        // RevisionStatic resolves only a wire's envelope before it asks, so
        // a lattice's envelope is validated whatever the caller says.
        const bool resolves = true;
        CHECK(rigExec::RigExecRevisionKernelAcceptance(
                  RigExecRevisionOp::Lattice, p, points.size(), &resolves) ==
              refuses);
        p = lattice;
        p.restPoints.pop_back();
        check("lattice, one rest point short", RigExecRevisionOp::Lattice, p,
              points, refuses);
        p = lattice;
        p.auxPointsB.pop_back();
        check("lattice, posed cage one point short",
              RigExecRevisionOp::Lattice, p, points, applies);
        p = lattice;
        p.divisions = GfVec3i(1, 2, 2);
        check("lattice, one division", RigExecRevisionOp::Lattice, p, points,
              applies);
        p = lattice;
        p.valid = false;
        check("lattice, invalid packet", RigExecRevisionOp::Lattice, p,
              points, refuses);
        check("lattice packet on a wire", RigExecRevisionOp::Wire, lattice,
              points, refuses);
    }

    // A skin's decision is the baked program's, from the halves its steps
    // hold; this one only checks the packet.
    {
        const auto skin = MakeSkinParams(
            {GfMatrix4d(1.0), GfMatrix4d(1.0).SetTranslate(GfVec3d(0, 4, 0))},
            {0, 1, 0, 1, 0, 1, 0, 1},
            {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f}, 2);
        check("skin", RigExecRevisionOp::Skin, skin, points, deferred);
        auto p = skin;
        p.valid = false;
        check("skin, invalid packet", RigExecRevisionOp::Skin, p, points,
              refuses);
    }
    std::printf("revision acceptance: %zu packet(s) decided as their kernels "
                "answer\n", compared);
}

static float
NanWithPayload(uint32_t payload)
{
    const uint32_t bits = 0x7fc00000u | (payload & 0x3fffffu);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

static bool
SameBits(const std::vector<GfVec3f> &a, const std::vector<GfVec3f> &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            !std::memcmp(a.data(), b.data(), a.size() * sizeof(GfVec3f)));
}

// Whether every point of [begin, end) holds \p fill's bits.
static bool
HoldsBits(const std::vector<GfVec3f> &points, size_t begin, size_t end,
          const GfVec3f &fill)
{
    for (size_t i = begin; i < end; ++i) {
        if (std::memcmp(&points[i], &fill, sizeof(GfVec3f))) {
            return false;
        }
    }
    return true;
}

// Entering points with NaN payloads, signed zeros and infinities at and
// beside the bounds the range tests cut at, and inside the ranges. A point
// holds one NaN payload and nothing else non-finite, so no addition meets
// two different NaNs, whose result is the compiler's operand order.
static std::vector<GfVec3f>
MakeRangeEnteringPoints(size_t count)
{
    std::vector<GfVec3f> points(count);
    for (size_t i = 0; i < count; ++i) {
        const float t = float(i);
        points[i] = GfVec3f(3.0f * std::sin(0.37f * t),
                            2.0f * std::cos(0.11f * t) - 1.0f,
                            0.01f * t - 4.0f);
    }
    const float inf = std::numeric_limits<float>::infinity();
    points[0] = GfVec3f(-0.0f, 0.0f, -0.0f);
    points[1] = GfVec3f(NanWithPayload(0x1234), -0.0f, 1.0f);
    points[3] = GfVec3f(inf, -inf, 0.5f);
    points[332] = GfVec3f(-inf, 0.5f, -0.0f);
    points[333] = GfVec3f(-0.0f, NanWithPayload(0x2), 0.25f);
    points[340] = GfVec3f(0.0f, -0.0f, inf);
    points[699] = GfVec3f(NanWithPayload(0x3ffff), 2.0f, -0.0f);
    points[700] = GfVec3f(-0.0f, -0.0f, -0.0f);
    points[1000] = GfVec3f(inf, 1.0f, 0.0f);
    return points;
}

// The packets the range tests run, over 1001 points.
struct RangeTestPackets {
    static constexpr size_t count = 1001;
    std::vector<int> sparseIndices;
    RigExecWeightPacket sparse, denseMatrix, denseWire;
    RigExecMoverParameters matrix, wire, sparseWire, lattice;

    RangeTestPackets()
    {
        // Indices in [0, 1), [1, 333) and [334, 700), none in [333, 334)
        // or [700, 1001).
        sparseIndices.push_back(0);
        for (int i = 3; i < 333; i += 7) sparseIndices.push_back(i);
        for (int i = 340; i < 700; i += 12) sparseIndices.push_back(i);
        sparse.representation = TfToken("sparse");
        sparse.rangePolicy = TfToken("strict");
        sparse.indices = sparseIndices;
        for (size_t k = 0; k < sparseIndices.size(); ++k) {
            sparse.values.push_back(float(k % 6) / 5.0f);
        }
        sparse.defaultWeight = 0.0f;
        sparse.valid = true;
        denseMatrix.representation = TfToken("dense");
        denseMatrix.rangePolicy = TfToken("strict");
        denseWire = denseMatrix;
        for (size_t i = 0; i < count; ++i) {
            // Runs of one weight, as a painted falloff has.
            denseMatrix.values.push_back(float((i / 3) % 11) / 10.0f);
            denseWire.values.push_back(float(i % 9) / 8.0f);
        }
        denseMatrix.valid = denseWire.valid = true;

        matrix = MakeMatrixParams(GfVec3d(0.0), 1.0f);
        matrix.transform.SetTransform(GfRotation(GfVec3d(1, 2, 3), 37.0),
                                      GfVec3d(0.5, -1.0, 2.0));

        wire.valid = true;
        wire.kind = TfToken("wire");
        wire.weights = RigExecWeightPacket::Constant(1.0f);
        wire.curveOrder = 3;
        wire.curveKnots = {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0};
        wire.restPoints = {GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                           GfVec3f(2, 0, 0), GfVec3f(3, 0, 0),
                           GfVec3f(4, 0, 0)};
        // The first control point at rest moves nothing.
        wire.auxPoints = {GfVec3f(0, 0, 0), GfVec3f(1, 0.5f, 0),
                          GfVec3f(2, -0.25f, 0.3f), GfVec3f(3, 0.75f, -0.1f),
                          GfVec3f(4, 0, 0.4f)};
        wire.dropoffDistance = 2.0;
        // Some binds lie past the dropoff and are skipped.
        VtArray<GfVec2f> binds(count);
        for (size_t i = 0; i < count; ++i) {
            binds[i] = GfVec2f(3.0f * float(i % 97) / 96.0f,
                               0.2f * float(i % 13));
        }
        wire.wireBindCoords = binds;
        sparseWire = wire;
        sparseWire.weights = sparse;
        VtArray<GfVec2f> sparseBinds(sparseIndices.size());
        for (size_t k = 0; k < sparseIndices.size(); ++k) {
            sparseBinds[k] =
                GfVec2f(3.0f * float(k) / float(sparseIndices.size()),
                        0.25f * float(k % 11));
        }
        sparseWire.wireBindCoords = sparseBinds;

        lattice.valid = true;
        lattice.kind = TfToken("lattice");
        lattice.weights = RigExecWeightPacket::Constant(1.0f);
        lattice.divisions = GfVec3i(3, 4, 2);
        for (int c = 0; c < 2; ++c) {
            for (int b = 0; b < 4; ++b) {
                for (int a = 0; a < 3; ++a) {
                    lattice.auxPoints.emplace_back(float(a), 1.5f * float(b),
                                                   0.5f * float(c));
                }
            }
        }
        lattice.auxPointsB = lattice.auxPoints;
        for (size_t k = 0; k < lattice.auxPointsB.size(); ++k) {
            lattice.auxPointsB[k] += GfVec3f(0.1f * float(k % 5),
                                             -0.05f * float(k % 3), 0.3f);
        }
        // Rest points inside and outside the cage's bound, so the nonzero
        // factor spans differ from point to point.
        lattice.restPoints.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const float t = float(i);
            lattice.restPoints[i] =
                GfVec3f(1.0f + 1.6f * std::sin(0.7f * t),
                        2.25f + 3.0f * std::cos(0.3f * t),
                        0.25f + 0.4f * std::sin(1.3f * t));
        }
        lattice.restPoints[500][1] = NanWithPayload(0x5);
    }
};

// A range-pipelined revision's ranges against its whole kernel. For each
// range op and envelope shape, RigExecRunRevisionRange over an uneven
// partition (one-point ranges at the start and in the middle) and over one
// range writes, concatenated, exactly the bits RigExecRunRevisionKernel
// writes over the whole array; a range touches no point outside it; and it
// reports untouched exactly where the whole kernel cannot write. The
// prepared inputs are the memos the whole kernel's path reads.
static void
TestRevisionRangesMatchTheWholeKernel()
{
    using rigExec::RigExecRevisionAcceptance;
    using rigExec::RigExecRevisionRangeInputs;
    using Cache = rigExec::RigExecSurfaceKernelCache<GfVec3f, GfVec3d>;
    const RangeTestPackets packets;
    const size_t count = RangeTestPackets::count;
    const std::vector<GfVec3f> entering = MakeRangeEnteringPoints(count);
    const std::vector<std::vector<size_t>> partitions = {
        {0, 1, 333, 334, 700, 1001}, {0, 1001}};
    const GfVec3f poison(NanWithPayload(0x5a5a5), -0.0f,
                         NanWithPayload(0x123));
    size_t compared = 0;
    // The points every case enters with, unless a case says otherwise.
    const std::vector<GfVec3f> *source = &entering;
    const auto check = [&](const char *what, RigExecRevisionOp op,
                           const RigExecMoverParameters &p, bool useSimd,
                           bool cached,
                           const std::function<bool(size_t, size_t)> &skips,
                           bool moves) {
        const std::vector<GfVec3f> &in = *source;
        rigExec::RigExecWireBasisCache wholeWire, rangeWire;
        Cache wholeSurface, rangeSurface;
        std::vector<GfVec3f> whole = in;
        bool ok = rigExec::RigExecRunRevisionKernel(
            op, p, &whole, useSimd, cached ? &wholeWire : nullptr,
            cached ? &wholeSurface : nullptr);
        ok = ok && SameBits(whole, in) != moves;
        RigExecRevisionRangeInputs prepared;
        const RigExecRevisionAcceptance acceptance =
            rigExec::RigExecPrepareRevisionRanges(
                op, p, count, cached ? &rangeWire : nullptr,
                cached ? &rangeSurface : nullptr, &prepared);
        ok = ok && acceptance == RigExecRevisionAcceptance::Applies &&
             acceptance ==
                 rigExec::RigExecRevisionKernelAcceptance(op, p, count);
        // What the prepared inputs hold for this shape.
        const bool sparseShape =
            rigExec::RigExecWireTakesSparseEnvelope(p.weights);
        const bool fullStrength =
            rigExec::RigExecEnvelopeIsFullStrength(p.weights);
        if (op == RigExecRevisionOp::Matrix) {
            ok = ok && prepared.matrixWeights.size() ==
                           (fullStrength || sparseShape ? size_t(0) : count);
        } else if (op == RigExecRevisionOp::Wire) {
            ok = ok && bool(prepared.wireBasis) == sparseShape &&
                 (prepared.wireRestEvaluations != nullptr) ==
                     (cached && !sparseShape);
        } else {
            ok = ok && (prepared.latticeBasis != nullptr) == cached &&
                 (!cached || (prepared.latticeBind &&
                              &prepared.latticeBind->value ==
                                  prepared.latticeBasis));
        }
        std::vector<float> envelope;
        if (rigExec::RigExecRevisionTakesSeparateBlend(op, p.weights) &&
            !fullStrength) {
            ok = p.weights.ResolveAll(count, &envelope) && ok;
        }
        const float *separate = envelope.empty() ? nullptr : envelope.data();
        for (const std::vector<size_t> &bounds : partitions) {
            std::vector<GfVec3f> reported(count, poison);
            std::vector<GfVec3f> written(count, poison);
            for (size_t k = 0; k + 1 < bounds.size(); ++k) {
                const size_t b = bounds[k], e = bounds[k + 1];
                bool untouched = !skips(b, e);
                ok = rigExec::RigExecRunRevisionRange(
                         op, p, prepared, in.data(), count, b, e,
                         separate, &reported, useSimd, &untouched) &&
                     ok;
                ok = ok && untouched == skips(b, e);
                if (untouched) {
                    // Nothing written, and the whole kernel left these
                    // points as they entered; the caller passes them on.
                    ok = ok && HoldsBits(reported, b, e, poison) &&
                         !std::memcmp(whole.data() + b, in.data() + b,
                                      (e - b) * sizeof(GfVec3f));
                    std::copy(in.begin() + b, in.begin() + e,
                              reported.begin() + b);
                }
                // Asked for the result in `out`, a range writes it there and
                // writes nothing else.
                std::vector<GfVec3f> alone(count, poison);
                ok = rigExec::RigExecRunRevisionRange(
                         op, p, prepared, in.data(), count, b, e,
                         separate, &alone, useSimd) &&
                     ok;
                ok = ok && HoldsBits(alone, 0, b, poison) &&
                     HoldsBits(alone, e, count, poison);
                std::copy(alone.begin() + b, alone.begin() + e,
                          written.begin() + b);
            }
            ok = ok && SameBits(reported, whole) && SameBits(written, whole);
        }
        if (!ok) {
            std::printf("  %s (simd %d, cached %d): the ranges are not the "
                        "whole kernel's\n", what, int(useSimd), int(cached));
        }
        CHECK(ok);
        ++compared;
    };
    const auto never = [](size_t, size_t) { return false; };
    const auto always = [](size_t, size_t) { return true; };
    const auto noIndexIn = [&](size_t b, size_t e) {
        return std::none_of(packets.sparseIndices.begin(),
                            packets.sparseIndices.end(), [&](int index) {
                                return size_t(index) >= b &&
                                       size_t(index) < e;
                            });
    };
    using Op = RigExecRevisionOp;

    // Matrix: full strength, a dense envelope and a sparse walk, each with
    // the linear and the radial blend; the identity's sparse walk writes
    // nothing at all.
    for (const bool radial : {false, true}) {
        for (const bool simd : {false, true}) {
            auto p = packets.matrix;
            p.radialWeight = radial;
            check(radial ? "matrix full strength, radial"
                         : "matrix full strength",
                  Op::Matrix, p, simd, false, never, true);
            p.weights = packets.denseMatrix;
            check(radial ? "matrix dense, radial" : "matrix dense",
                  Op::Matrix, p, simd, false, never, true);
        }
        auto p = packets.matrix;
        p.radialWeight = radial;
        p.weights = packets.sparse;
        check(radial ? "matrix sparse walk, radial" : "matrix sparse walk",
              Op::Matrix, p, false, false, noIndexIn, true);
        p.transform = GfMatrix4d(1.0);
        check("matrix sparse walk at identity", Op::Matrix, p, false, false,
              always, false);
    }

    // Wire: the sparse basis (memoized and built per call), the dense walk
    // with and without rest evaluations, and the dense walk blended with a
    // separate envelope.
    for (const bool cached : {false, true}) {
        check("wire sparse basis", Op::Wire, packets.sparseWire, false,
              cached, noIndexIn, true);
        check("wire dense", Op::Wire, packets.wire, false, cached, never,
              true);
        auto p = packets.wire;
        p.weights = packets.denseWire;
        check("wire dense, separate envelope", Op::Wire, p, false, cached,
              never, true);
    }

    // Lattice: through a retained basis (cached) and per point (not), at
    // full strength and blended, and with non-finite cage deltas, which
    // visit every term. Those deltas make NaNs of their own, so that case
    // enters infinities where the others enter NaN payloads.
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<GfVec3f> noPayloads = entering;
    for (GfVec3f &point : noPayloads) {
        for (int axis = 0; axis < 3; ++axis) {
            if (std::isnan(point[axis])) {
                point[axis] = inf;
            }
        }
    }
    for (const bool cached : {false, true}) {
        check("lattice", Op::Lattice, packets.lattice, false, cached, never,
              true);
        auto p = packets.lattice;
        p.weights = RigExecWeightPacket::Constant(0.5f);
        check("lattice at half", Op::Lattice, p, false, cached, never, true);
        p = packets.lattice;
        p.auxPointsB[3][0] = inf;
        p.auxPointsB[5][1] = -inf;
        source = &noPayloads;
        check("lattice, non-finite cage deltas", Op::Lattice, p, false,
              cached, never, true);
        source = &entering;
    }

    // The retained bind outlives its cache's slot: handed an equal bind
    // between runs (RigExecLatticeBindSharing), the cache drops its own,
    // which the prepared inputs still hold, and the ranges still answer the
    // whole kernel's bits through it.
    {
        Cache first, second;
        RigExecRevisionRangeInputs prepared;
        CHECK(rigExec::RigExecPrepareRevisionRanges(
                  Op::Lattice, packets.lattice, count, nullptr, &first,
                  &prepared) == RigExecRevisionAcceptance::Applies);
        std::vector<GfVec3f> whole = entering;
        CHECK(rigExec::RigExecRunRevisionKernel(Op::Lattice, packets.lattice,
                                                &whole, false, nullptr,
                                                &second));
        const auto held = first.RetainedLatticeBind();
        CHECK(held && prepared.latticeBind == held &&
              second.RetainedLatticeBind() &&
              second.RetainedLatticeBind() != held);
        {
            rigExec::RigExecLatticeBindSharing<GfVec3f> sharing;
            sharing.Offer(&second);
            sharing.Offer(&first);
        }
        CHECK(first.RetainedLatticeBind() == second.RetainedLatticeBind());
        // Only this test's copy and the prepared inputs hold it now.
        CHECK(held.use_count() == 2 &&
              &held->value == prepared.latticeBasis);
        std::vector<GfVec3f> out(count, poison);
        for (size_t k = 0; k + 1 < partitions[0].size(); ++k) {
            CHECK(rigExec::RigExecRunRevisionRange(
                Op::Lattice, packets.lattice, prepared, entering.data(),
                count, partitions[0][k], partitions[0][k + 1], nullptr, &out,
                false));
        }
        CHECK(SameBits(out, whole));
    }
    std::printf("revision ranges: %zu revision(s) matched their whole "
                "kernels\n", compared);
}

// RigExecPrepareRevisionRanges answers RigExecRevisionKernelAcceptance for
// every packet, and leaves no input from an earlier packet behind where the
// answer is not Applies for a range op. RigExecRevisionIsRangeOp names
// exactly Matrix, Wire and Lattice.
static void
TestPrepareRevisionRangesDecidesAsTheKernel()
{
    using rigExec::RigExecRevisionAcceptance;
    using rigExec::RigExecRevisionRangeInputs;
    using Op = RigExecRevisionOp;
    const RangeTestPackets packets;
    const size_t count = RangeTestPackets::count;
    const std::vector<GfVec3f> staleTable(3);
    const rigExec::RigExecLatticeBasis staleBasis{};
    const auto check = [&](const char *what, Op op,
                           const RigExecMoverParameters &p, size_t n,
                           RigExecRevisionAcceptance expected) {
        rigExec::RigExecWireBasisCache wire;
        rigExec::RigExecSurfaceKernelCache<GfVec3f, GfVec3d> surface;
        RigExecRevisionRangeInputs prepared;
        prepared.matrixWeights.assign(3, 0.5f);
        prepared.wireBasis = std::make_shared<rigExec::RigExecWireBasis>();
        prepared.wireRestEvaluations = &staleTable;
        prepared.latticeBasis = &staleBasis;
        const RigExecRevisionAcceptance decided =
            rigExec::RigExecPrepareRevisionRanges(op, p, n, &wire, &surface,
                                                  &prepared);
        bool ok = decided == expected &&
                  decided == rigExec::RigExecRevisionKernelAcceptance(op, p, n);
        if (decided != RigExecRevisionAcceptance::Applies ||
            !rigExec::RigExecRevisionIsRangeOp(op)) {
            ok = ok && prepared.matrixWeights.empty() && !prepared.wireBasis &&
                 !prepared.wireRestEvaluations && !prepared.latticeBasis &&
                 !prepared.latticeBind;
        }
        // A packet the kernel refuses is one RigExecRunRevisionKernel fails.
        if (decided == RigExecRevisionAcceptance::Refuses) {
            std::vector<GfVec3f> pts(n, GfVec3f(1.0f, 2.0f, 3.0f));
            ok = ok && !rigExec::RigExecRunRevisionKernel(op, p, &pts, false,
                                                          nullptr);
        }
        if (!ok) {
            std::printf("  %s: prepared %d, expected %d\n", what,
                        int(decided), int(expected));
        }
        CHECK(ok);
    };
    const RigExecRevisionAcceptance applies =
        RigExecRevisionAcceptance::Applies;
    const RigExecRevisionAcceptance refuses =
        RigExecRevisionAcceptance::Refuses;

    auto p = packets.matrix;
    p.weights = packets.denseMatrix;
    check("matrix dense", Op::Matrix, p, count, applies);
    check("matrix dense, one point more", Op::Matrix, p, count + 1, refuses);
    p.weights.values[7] = 1.5f;
    check("matrix dense weight above 1", Op::Matrix, p, count, refuses);
    p.weights = packets.sparse;
    std::swap(p.weights.indices[1], p.weights.indices[2]);
    check("matrix sparse walk descending", Op::Matrix, p, count, refuses);
    p = packets.matrix;
    p.valid = false;
    check("matrix, invalid packet", Op::Matrix, p, count, refuses);

    check("wire dense", Op::Wire, packets.wire, count, applies);
    check("wire dense, one point more", Op::Wire, packets.wire, count + 1,
          refuses);
    p = packets.wire;
    p.weights = packets.denseWire;
    p.weights.values.pop_back();
    check("wire dense, envelope one weight short", Op::Wire, p, count,
          refuses);
    p = packets.sparseWire;
    std::swap(p.weights.indices[1], p.weights.indices[2]);
    check("wire sparse walk descending", Op::Wire, p, count, refuses);
    check("wire sparse basis", Op::Wire, packets.sparseWire, count, applies);

    check("lattice", Op::Lattice, packets.lattice, count, applies);
    check("lattice, one point more", Op::Lattice, packets.lattice, count + 1,
          refuses);
    p = packets.lattice;
    p.weights = packets.denseWire;
    p.weights.values[3] = std::numeric_limits<float>::quiet_NaN();
    check("lattice, NaN envelope weight", Op::Lattice, p, count, refuses);

    // Not range ops: the acceptance alone, nothing prepared.
    RigExecMoverParameters blend;
    blend.valid = true;
    blend.kind = TfToken("blendShape");
    blend.weights = RigExecWeightPacket::Constant(1.0f);
    blend.blendDeltas.assign(count, GfVec3f(0, 1, 0));
    check("blend shape", Op::BlendShape, blend, count, applies);
    blend.blendSurfaceFrame = true;
    check("blend shape on the surface frame", Op::BlendShape, blend, count,
          RigExecRevisionAcceptance::Deferred);

    const Op ops[] = {Op::Matrix,          Op::Skin,
                      Op::BlendShape,      Op::VolumeCorrect,
                      Op::Smooth,          Op::Lattice,
                      Op::SurfaceProject,  Op::Ribbon,
                      Op::Wire,            Op::EmitGuidePoints,
                      Op::RecomputeNormals, Op::RecomputeExtent,
                      Op::DeltaMush,       Op::Wrinkle,
                      Op::External,        Op::SurfaceProjector,
                      Op::ShaderDials};
    for (const Op op : ops) {
        CHECK(rigExec::RigExecRevisionIsRangeOp(op) ==
              (op == Op::Matrix || op == Op::Wire || op == Op::Lattice));
    }
}

static void
TestLongResolvedInputConnections()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const UsdPrim prim = stage->DefinePrim(SdfPath("/Inputs"));
    std::vector<UsdAttribute> chain;
    const size_t count = 2048;
    for (size_t i = 0; i < count; ++i) {
        chain.push_back(prim.CreateAttribute(
            TfToken("value" + std::to_string(i)), SdfValueTypeNames->Float));
        if (i > 0) chain[i - 1].SetConnections({chain[i].GetPath()});
    }
    chain.front().Set(1.0f);
    chain.back().Set(2.0f);
    rigExec::RigExecResolvedInputs resolved;
    float value = 0;
    CHECK(resolved.GetAttribute(chain.front(), UsdTimeCode::Default(), &value));
    CHECK(value == 2.0f);
    chain.back().Set(3.0f);
    CHECK(resolved.GetAttribute(chain.front(), UsdTimeCode::Default(), &value));
    CHECK(value == 3.0f);
    resolved.SetProperty(chain[count / 2].GetPath(), VtValue(4.0f));
    CHECK(resolved.GetAttribute(chain.front(), UsdTimeCode::Default(), &value));
    CHECK(value == 4.0f);
    resolved.Clear();
    chain.back().Clear();
    CHECK(resolved.GetAttribute(chain.front(), UsdTimeCode::Default(), &value));
    CHECK(value == 1.0f);
    // Malformed cycles terminate and preserve the established fallback rule.
    chain.back().SetConnections({chain.front().GetPath()});
    CHECK(resolved.GetAttribute(chain.front(), UsdTimeCode::Default(), &value));
    CHECK(value == 1.0f);
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestSingleRevision();
    TestSkinRevision();
    TestSkinBindingAndAssembly();
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
    TestIncrementalRevisionUpdates();
    TestLongChainDirtySuffix();
    TestAppendAfterEvaluation();
    TestEveryOperationUpdatesInteractively();
    TestRevisionAcceptanceMatchesKernel();
    TestRevisionRangesMatchTheWholeKernel();
    TestPrepareRevisionRangesDecidesAsTheKernel();
    TestLongResolvedInputConnections();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecMoverGraph: all tests passed\n");
    return 0;
}
