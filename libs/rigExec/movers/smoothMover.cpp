//
// RigExecSmoothMover: everything about the smooth mover (spec §4.1).
//
// A smooth mover applies fixed-adjacency Laplacian smoothing over the
// destination's standard topology. This TU owns its exec-side
// computeMoverParameters registration and builder, its revision binder,
// and its parity-oracle branch, and registers the row that points at
// them. Compile validation is the generic points-target + single-target
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

// RigExecSmoothMover parameters: fixed-adjacency Laplacian smoothing of
// the destination's standard topology (spec §7.6 revised).
RigExecMoverParameters
_BuildSmoothMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("smooth");
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
    params.strength = 1.0f;
    params.topologyCounts = rigExec::RigExecMoverCollect<int>(
        ctx, RigExecMoverExecTokens->topologyCounts);
    params.topologyIndices = rigExec::RigExecMoverCollect<int>(
        ctx, RigExecMoverExecTokens->topologyIndices);
    params.valid = !params.topologyCounts.empty();
    return params;
}

void
_BindSmoothMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    const SdfPath &ownerPath = ctx.ownerPath;
    binding.topologyCounts =
        ownerPath.AppendProperty(TfToken("faceVertexCounts"));
    binding.topologyIndices =
        ownerPath.AppendProperty(TfToken("faceVertexIndices"));
}

rigExec::RigExecOracleResult
_OracleSmoothMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const UsdStageRefPtr &stage = ctx.stage;
    const SdfPath &target = ctx.target;
    const UsdTimeCode time = ctx.time;
    VtVec3fArray &points = *ctx.points;
    std::vector<GfVec3f> scratch(points.begin(), points.end());
    const UsdPrim owner =
        stage->GetPrimAtPath(target.GetPrimPath());
    VtIntArray counts, indices;
    if (owner) {
        owner.GetAttribute(TfToken("faceVertexCounts"))
            .Get(&counts, time);
        owner.GetAttribute(TfToken("faceVertexIndices"))
            .Get(&indices, time);
    }
    rigExec::RigExecApplyLaplacianSmooth(
        &scratch,
        std::vector<int>(counts.begin(), counts.end()),
        std::vector<int>(indices.begin(), indices.end()),
        1.0f);
    std::copy(scratch.begin(), scratch.end(), points.begin());
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecSmoothMover",
        &rigExec::RigExecFixedMoverOp<rigExec::RigExecRevisionOp::Smooth>,
        rigExec::RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.legacyEnvelopeAttribute = "inputs:strength";
    handler.bind = &_BindSmoothMover;
    handler.oracle = &_OracleSmoothMover;
    return handler;
}

}  // namespace

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so the registration block stays at global scope.
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecSmoothMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildSmoothMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
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
