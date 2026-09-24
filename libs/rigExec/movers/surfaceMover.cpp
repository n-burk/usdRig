//
// RigExecSurfaceMover: everything about the surface mover (spec §4.1).
//
// A surface mover projects points onto a driver surface. This TU owns
// its exec-side computeMoverParameters registration and builder, its
// revision binder, and its parity-oracle branch, and registers the row
// that points at them. Compile validation is the generic points-target
// rules.
//

#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "rigExecMath/geometryKernels.h"

#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"

using rigExec::RigExecMoverParameters;
using rigExec::RigExecMoverExecTokens;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

RigExecMoverParameters
_BuildSurfaceMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("surfaceProject");
    const bool *enabled =
        ctx.GetInputValuePtr<bool>(RigExecMoverExecTokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    if (!rigExec::RigExecMoverSetCommonEnvelope(ctx, &params)) {
        return params;
    }
    params.strength = 1.0f;  // v0.1 attach/project maps fully
    params.auxPoints = rigExec::RigExecMoverCollect<GfVec3f>(
        ctx, RigExecMoverExecTokens->surfacePoints);
    params.topologyCounts = rigExec::RigExecMoverCollect<int>(
        ctx, RigExecMoverExecTokens->topologyCounts);
    params.topologyIndices = rigExec::RigExecMoverCollect<int>(
        ctx, RigExecMoverExecTokens->topologyIndices);
    params.valid = !params.auxPoints.empty() &&
                   !params.topologyCounts.empty();
    return params;
}

void
_BindSurfaceMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    const UsdPrim &moverPrim = ctx.moverPrim;
    const SdfPathVector surfaces = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:surface");
    if (!surfaces.empty()) {
        const SdfPath surfacePrim = surfaces[0].GetPrimPath();
        binding.surfacePoints =
            surfacePrim.AppendProperty(TfToken("points"));
        const rigExec::RigExecReadPhase phase =
            rigExec::RigExecPhaseForInput(
                moverPrim, "rigExec:surface", "rigExec:surfaceReadPhase");
        if (!phase.IsBase()) {
            binding.phases[binding.surfacePoints] = phase;
        }
        binding.topologyCounts =
            surfacePrim.AppendProperty(TfToken("faceVertexCounts"));
        binding.topologyIndices =
            surfacePrim.AppendProperty(TfToken("faceVertexIndices"));
    }
}

rigExec::RigExecOracleResult
_OracleSurfaceMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const UsdStageRefPtr &stage = ctx.stage;
    const UsdPrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    const UsdTimeCode time = ctx.time;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    SdfPathVector surfaces;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:surface"))) {
        rel.GetTargets(&surfaces);
    }
    if (surfaces.empty()) {
        return RigExecOracleResult::PassThrough;
    }
    const SdfPath surfacePrim = surfaces[0].GetPrimPath();
    VtVec3fArray surfacePoints;
    VtIntArray counts, indices;
    rigExec::RigExecReadPhasedPoints(
        stage, ctx.snapshots, time, prim, "rigExec:surface",
        "rigExec:surfaceReadPhase",
        surfacePrim.AppendProperty(TfToken("points")), moverPath,
        &surfacePoints);
    if (const UsdPrim s = stage->GetPrimAtPath(surfacePrim)) {
        s.GetAttribute(TfToken("faceVertexCounts")).Get(&counts, time);
        s.GetAttribute(TfToken("faceVertexIndices"))
            .Get(&indices, time);
    }
    if (surfacePoints.empty() || counts.empty()) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": surface has no points/topology");
        return RigExecOracleResult::PassThrough;
    }
    std::vector<GfVec3f> scratch(points.begin(), points.end());
    // v0.1 attach/project both map fully; the mode token selects no
    // numeric difference yet (_BuildSurfaceMoverParameters pins 1.0).
    rigExec::RigExecApplySurfaceProject(
        &scratch,
        std::vector<GfVec3f>(surfacePoints.begin(),
                             surfacePoints.end()),
        std::vector<int>(counts.begin(), counts.end()),
        std::vector<int>(indices.begin(), indices.end()), 1.0f);
    std::copy(scratch.begin(), scratch.end(), points.begin());
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecSurfaceMover",
        &rigExec::RigExecFixedMoverOp<
            rigExec::RigExecRevisionOp::SurfaceProject>,
        rigExec::RigExecMoverDomain::Points);
    handler.bind = &_BindSurfaceMover;
    handler.oracle = &_OracleSurfaceMover;
    return handler;
}

}  // namespace

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so the registration block stays at global scope.
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecSurfaceMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildSurfaceMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            Relationship(RigExecMoverExecTokens->resolvedSurfacePoints)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->surfacePoints),
            Relationship(RigExecMoverExecTokens->resolvedTopologyCounts)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->topologyCounts),
            Relationship(RigExecMoverExecTokens->resolvedTopologyIndices)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->topologyIndices));

    self.PrimComputation(RigExecMoverExecTokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(
            &rigExec::RigExecMoverBuildStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                RigExecMoverExecTokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(RigExecMoverExecTokens->moverPath));
}

RIGEXEC_REGISTER_MOVER(_MakeHandler());
