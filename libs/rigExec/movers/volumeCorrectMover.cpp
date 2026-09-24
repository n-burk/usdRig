//
// RigExecVolumeCorrectMover: everything about the volume-correct mover
// (spec §4.1).
//
// A volume-correct mover preserves the authored base bound volume
// against the deformation the chain has applied so far. This TU owns
// its exec-side computeMoverParameters registration and builder, its
// revision binder, and its parity-oracle branch, and registers the row
// that points at them. Compile validation is the generic points-target
// + single-target rules.
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

// RigExecVolumeCorrectMover parameters: the correction reference is the
// authored base bound volume (spec §7.6 revised).
RigExecMoverParameters
_BuildVolumeCorrectMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("volumeCorrect");
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
    const std::vector<GfVec3f> base =
        rigExec::RigExecMoverCollect<GfVec3f>(
            ctx, RigExecMoverExecTokens->basePoints);
    if (base.empty()) {
        return params;
    }
    params.referenceVolume =
        rigExec::RigExecBoundVolume(base.data(), base.size());
    params.valid = true;
    return params;
}

void
_BindVolumeCorrectMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    binding.base = ctx.target;
}

rigExec::RigExecOracleResult
_OracleVolumeCorrectMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const UsdStageRefPtr &stage = ctx.stage;
    const SdfPath &target = ctx.target;
    const UsdTimeCode time = ctx.time;
    VtVec3fArray &points = *ctx.points;
    std::vector<GfVec3f> scratch(points.begin(), points.end());
    VtVec3fArray base;
    if (UsdAttribute a = stage->GetAttributeAtPath(target)) {
        a.Get(&base, time);
    }
    const double reference = rigExec::RigExecBoundVolume(
        base.cdata(), base.size());
    rigExec::RigExecApplyVolumeCorrect(&scratch, reference, 1.0f);
    std::copy(scratch.begin(), scratch.end(), points.begin());
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecVolumeCorrectMover",
        &rigExec::RigExecFixedMoverOp<
            rigExec::RigExecRevisionOp::VolumeCorrect>,
        rigExec::RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.legacyEnvelopeAttribute = "inputs:strength";
    handler.bind = &_BindVolumeCorrectMover;
    handler.oracle = &_OracleVolumeCorrectMover;
    return handler;
}

}  // namespace

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so the registration block stays at global scope.
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecVolumeCorrectMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(
            &_BuildVolumeCorrectMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            Relationship(RigExecMoverExecTokens->resolvedBase)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->basePoints));

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
