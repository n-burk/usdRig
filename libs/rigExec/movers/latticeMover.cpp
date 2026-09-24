//
// RigExecLatticeMover: everything about the lattice mover (spec §4.1).
//
// A lattice mover deforms points through a posed lattice cage measured
// against its rest cage. This TU owns its exec-side
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

// Synthesized derived-maintenance parameters (spec §7.6 revised): the
// hosts are compiler-authored with no authored mover and no enable; the
// rig-level derived policy gates synthesis at compile time.
RigExecMoverParameters
_BuildLatticeMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("lattice");
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
    const GfVec3i *divisions = ctx.GetInputValuePtr<GfVec3i>(
        RigExecMoverExecTokens->divisionsAttr);
    params.divisions = divisions ? *divisions : GfVec3i(0);
    params.restPoints = rigExec::RigExecMoverCollect<GfVec3f>(
        ctx, RigExecMoverExecTokens->basePoints);
    params.auxPoints = rigExec::RigExecMoverCollect<GfVec3f>(
        ctx, RigExecMoverExecTokens->restCagePointsAttr);
    params.auxPointsB = rigExec::RigExecMoverCollect<GfVec3f>(
        ctx, RigExecMoverExecTokens->cagePoints);
    const size_t cageCount = size_t(params.divisions[0]) *
                             size_t(params.divisions[1]) *
                             size_t(params.divisions[2]);
    params.valid = params.divisions[0] >= 2 && params.divisions[1] >= 2 &&
                   params.divisions[2] >= 2 &&
                   params.auxPoints.size() == cageCount &&
                   params.auxPointsB.size() == cageCount &&
                   !params.restPoints.empty();
    return params;
}

void
_BindLatticeMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    const UsdPrim &moverPrim = ctx.moverPrim;
    binding.base = ctx.target;
    const SdfPathVector cages = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:cage");
    if (!cages.empty()) {
        binding.cagePoints = rigExec::RigExecPointsOf(cages[0]);
        const rigExec::RigExecReadPhase phase =
            rigExec::RigExecPhaseForInput(
                moverPrim, "rigExec:cage", "rigExec:cageReadPhase");
        if (!phase.IsBase()) {
            binding.phases[binding.cagePoints] = phase;
        }
    }
}

rigExec::RigExecOracleResult
_OracleLatticeMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const UsdStageRefPtr &stage = ctx.stage;
    const UsdPrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    const SdfPath &target = ctx.target;
    const UsdTimeCode time = ctx.time;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    // Independent of RigExecAssembleParameters on purpose: this is the
    // parity oracle, so it resolves its own inputs off the stage. A
    // reference that called the assembler would have agreed with the
    // SurfaceProject strength bug instead of catching it.
    //
    // The rest cage is the cage at Default time. That is exactly
    // what the deleted compiler captured into
    // rigExec:restCagePoints -- a Default-time read and nothing
    // more, which is why the authored capture was removable.
    SdfPathVector cages;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:cage"))) {
        rel.GetTargets(&cages);
    }
    if (cages.empty()) {
        return RigExecOracleResult::PassThrough;
    }
    SdfPath cagePoints = cages[0];
    if (cagePoints.IsPrimPath()) {
        cagePoints = cagePoints.AppendProperty(TfToken("points"));
    }
    VtVec3fArray restCage, posedCage, base;
    // Rest is the BIND pose and always the authored value; only the
    // live cage carries a phase.
    if (UsdAttribute a = stage->GetAttributeAtPath(cagePoints)) {
        a.Get(&restCage, UsdTimeCode::Default());
    }
    rigExec::RigExecReadPhasedPoints(
        stage, ctx.snapshots, time, prim, "rigExec:cage",
        "rigExec:cageReadPhase", cagePoints, moverPath, &posedCage);
    if (UsdAttribute a = stage->GetAttributeAtPath(target)) {
        a.Get(&base, time);
    }
    GfVec3i divisions(0);
    if (UsdAttribute a =
            prim.GetAttribute(TfToken("rigExec:divisions"))) {
        a.Get(&divisions, time);
    }
    const size_t cageCount = size_t(divisions[0]) *
                             size_t(divisions[1]) *
                             size_t(divisions[2]);
    if (divisions[0] < 2 || divisions[1] < 2 || divisions[2] < 2 ||
        restCage.size() != cageCount ||
        posedCage.size() != cageCount || base.size() != points.size()) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": lattice cage/divisions mismatch");
        return RigExecOracleResult::PassThrough;
    }
    std::vector<GfVec3f> scratch(points.begin(), points.end());
    rigExec::RigExecApplyLattice(
        &scratch, std::vector<GfVec3f>(base.begin(), base.end()),
        std::vector<GfVec3f>(restCage.begin(), restCage.end()),
        std::vector<GfVec3f>(posedCage.begin(), posedCage.end()),
        divisions);
    std::copy(scratch.begin(), scratch.end(), points.begin());
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecLatticeMover",
        &rigExec::RigExecFixedMoverOp<rigExec::RigExecRevisionOp::Lattice>,
        rigExec::RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.bind = &_BindLatticeMover;
    handler.oracle = &_OracleLatticeMover;
    return handler;
}

}  // namespace

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so the registration block stays at global scope.
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecLatticeMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildLatticeMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            AttributeValue<GfVec3i>(RigExecMoverExecTokens->divisionsAttr),
            AttributeValue<GfVec3f>(
                RigExecMoverExecTokens->restCagePointsAttr),
            Relationship(RigExecMoverExecTokens->resolvedBase)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->basePoints),
            Relationship(RigExecMoverExecTokens->resolvedCagePoints)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->cagePoints));

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
