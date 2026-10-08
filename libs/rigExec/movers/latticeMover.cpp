// RigExecLatticeMover: everything about the lattice mover (spec §4.1).
// A lattice mover deforms points through a posed lattice cage measured
// against its rest cage. This TU owns its exec-side
// computeMoverParameters registration and builder, its revision binder,
// and its parity-oracle branch, and registers the row that points at
// them. Compile validation is the generic points-target + single-target
// rules.
#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "rigExecMath/geometryKernels.h"

#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include <type_traits>

using rigExec::RigExecMoverParameters;
using rigExec::RigExecMoverExecTokens;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

template<class Read>
bool Configure(RigExecMoverParameters *p,Read read,const std::vector<GfMatrix4d> &frames) {
    using namespace rigExec;
    const auto evaluation=read("rigExec:evaluation",TfToken("legacy"));
    if(evaluation!=TfToken("legacy") && evaluation!=TfToken("regularGrid"))return false;
    p->latticeSettings.regularGrid=evaluation==TfToken("regularGrid");
    if(!p->latticeSettings.regularGrid)return true;
    const char *names[]={"rigExec:interpolationU","rigExec:interpolationV","rigExec:interpolationW"};
    for(int a=0;a<3;++a)if(!RigExecLatticeInterpolationFromString(read(names[a],TfToken("bspline")).GetString(),&p->latticeSettings.interpolation[a]))return false;
    const auto origin=read("rigExec:origin",GfVec3f(-.5f)),spacing=read("rigExec:spacing",GfVec3f(1));
    for(int a=0;a<3;++a) {p->latticeSettings.origin[a]=origin[a];p->latticeSettings.spacing[a]=spacing[a];}
    const auto mask=read("rigExec:mask",VtFloatArray());p->latticeSettings.mask.assign(mask.begin(),mask.end());
    p->latticeSettings.strength=read("rigExec:strength",1.f);
    auto cage=read("rigExec:cageMatrix",GfMatrix4d(1)),target=read("rigExec:targetMatrix",GfMatrix4d(1));
    if(!RigExecSurfaceSnapValidMatrix(cage) || !RigExecSurfaceSnapValidMatrix(target))return false;
    if(!frames.empty()) {
        if(frames.size()!=2)return false;cage*=frames[0];target*=frames[1];
        if(!RigExecSurfaceSnapCanonicalComputedMatrix(&cage) || !RigExecSurfaceSnapCanonicalComputedMatrix(&target))return false;
    }
    return RigExecLatticeCoordinateMaps(read("rigExec:pointSpace",TfToken("local")).GetString(),cage,target,
        &p->targetToLattice,&p->latticeToTarget,&p->cageToLattice);
}

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
    const auto read=[&](const char *name,auto fallback) {
        using T=decltype(fallback);
        if constexpr(std::is_same_v<T,VtFloatArray>) {
            const auto a=rigExec::RigExecMoverCollect<float>(ctx,TfToken(name));return VtFloatArray(a.begin(),a.end());
        } else {const T *v=ctx.GetInputValuePtr<T>(TfToken(name));return v?*v:fallback;}
    };
    if(!Configure(&params,read,rigExec::RigExecMoverCollect<GfMatrix4d>(ctx,TfToken("latticeFrames"))))return params;
    const size_t cageCount = size_t(params.divisions[0]) *
                             size_t(params.divisions[1]) *
                             size_t(params.divisions[2]);
    const int minimum=params.latticeSettings.regularGrid?1:2;
    params.valid = params.divisions[0] >= minimum && params.divisions[1] >= minimum &&
                   params.divisions[2] >= minimum &&
                   (params.latticeSettings.regularGrid || params.auxPoints.size() == cageCount) &&
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
            rigExec::RigExecPhaseForInput(moverPrim, "rigExec:cage");
        if (!phase.IsBase()) {
            binding.phases[binding.cagePoints] = phase;
        }
    }
    binding.influences=rigExec::RigExecRelationshipTargets(moverPrim,"rigExec:frames");
    binding.transformPhase=rigExec::RigExecPhaseForInput(moverPrim,"rigExec:frames");
    if(binding.transformPhase.kind==rigExec::RigExecReadPhaseKind::Final)
        for(auto &provider:binding.influences) {
            const auto head=ctx.frameChainHeads.find(provider);if(head!=ctx.frameChainHeads.end())provider=head->second;
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
        stage, ctx.snapshots, time, prim, "rigExec:cage", cagePoints,
        moverPath, &posedCage);
    if (UsdAttribute a = stage->GetAttributeAtPath(target)) {
        a.Get(&base, time);
    }
    GfVec3i divisions(0);
    if (UsdAttribute a =
            prim.GetAttribute(TfToken("rigExec:divisions"))) {
        a.Get(&divisions, time);
    }
    const auto read=[&](const char *name,auto fallback) {
        const auto a=prim.GetAttribute(TfToken(name));if(a)ctx.resolved.GetAttribute(a,time,&fallback);return fallback;
    };
    std::vector<GfMatrix4d> matrices;
    const auto frames=rigExec::RigExecRelationshipTargets(prim,"rigExec:frames");
    const auto &table=rigExec::RigExecPhaseForInput(prim,"rigExec:frames").kind==rigExec::RigExecReadPhaseKind::Final?
        ctx.finalProviderMatrices:ctx.baseProviderMatrices;
    for(const auto &path:frames) {const auto it=table.find(path);if(it==table.end())return RigExecOracleResult::PassThrough;matrices.push_back(it->second);}
    RigExecMoverParameters params;if(!Configure(&params,read,matrices))return RigExecOracleResult::PassThrough;
    if(params.latticeSettings.regularGrid) {
        std::vector<GfVec3f> scratch(points.begin(),points.end());
        if(!rigExec::RigExecApplyLatticeGridKernel<GfVec3f,GfVec3d>(&scratch,{posedCage.begin(),posedCage.end()},divisions,
            params.latticeSettings,params.targetToLattice,params.latticeToTarget,params.cageToLattice))return RigExecOracleResult::PassThrough;
        std::copy(scratch.begin(),scratch.end(),points.begin());return RigExecOracleResult::Blend;
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

bool ValidateLattice(const rigExec::RigExecMoverValidateContext &ctx,std::string *error) {
    const auto &prim=ctx.prim;
    const auto fail=[&]() {*error="invalid lattice evaluation, interpolation, grid, mask or coordinate frame";return false;};
    const auto read=[&](const char *name,auto fallback) {const auto a=prim.GetAttribute(TfToken(name));if(a)a.Get(&fallback);return fallback;};
    RigExecMoverParameters p;if(!Configure(&p,read,{}))return fail();
    const auto cages=rigExec::RigExecRelationshipTargets(prim,"rigExec:cage");
    const auto frames=rigExec::RigExecRelationshipTargets(prim,"rigExec:frames");
    if(cages.size()!=1 || (!frames.empty() && frames.size()!=2))return fail();
    if(!ctx.stage->GetAttributeAtPath(rigExec::RigExecPointsOf(cages[0])))return fail();
    if(!frames.empty() && rigExec::RigExecPhaseForInput(prim,"rigExec:frames").kind==rigExec::RigExecReadPhaseKind::AtPrim) {
        *error="lattice frame providers require base or final read phase";return false;
    }
    if(!p.latticeSettings.regularGrid) {
        const char *names[]={"rigExec:interpolationU","rigExec:interpolationV","rigExec:interpolationW","rigExec:origin","rigExec:spacing","rigExec:strength","rigExec:mask","rigExec:pointSpace","rigExec:cageMatrix","rigExec:targetMatrix"};
        for(const char *name:names) {const auto a=prim.GetAttribute(TfToken(name));if(a && (a.HasAuthoredValueOpinion() || a.HasAuthoredConnections())) {
            *error=std::string(name)+" requires rigExec:evaluation=regularGrid";return false;
        }}
        if(!frames.empty()) {*error="lattice frames require rigExec:evaluation=regularGrid";return false;}
    }
    const auto divisions=read("rigExec:divisions",GfVec3i(2));
    const int minimum=p.latticeSettings.regularGrid?1:2;
    for(int a=0;a<3;++a)if(divisions[a]<minimum)return fail();
    if(p.latticeSettings.regularGrid) {
        const auto &grid=p.latticeSettings;
        if(!std::isfinite(grid.strength))return fail();
        for(int a=0;a<3;++a)if(!std::isfinite(grid.origin[a]) || !std::isfinite(grid.spacing[a]) || (divisions[a]>1 && grid.spacing[a]==0))return fail();
        for(float w:grid.mask)if(!std::isfinite(w) || w<0 || w>1)return fail();
        VtVec3fArray points;if(!grid.mask.empty() && ctx.stage->GetAttributeAtPath(ctx.targets[0]).Get(&points) && grid.mask.size()!=points.size())return fail();
    }
    return true;
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
    handler.validate = &ValidateLattice;
    handler.frameRelationships={"rigExec:frames"};handler.transformRelationship="rigExec:frames";
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
            AttributeValue<TfToken>(TfToken("rigExec:evaluation")),
            AttributeValue<TfToken>(TfToken("rigExec:interpolationU")),
            AttributeValue<TfToken>(TfToken("rigExec:interpolationV")),
            AttributeValue<TfToken>(TfToken("rigExec:interpolationW")),
            AttributeValue<GfVec3f>(TfToken("rigExec:origin")),
            AttributeValue<GfVec3f>(TfToken("rigExec:spacing")),
            AttributeValue<float>(TfToken("rigExec:strength")),
            AttributeValue<float>(TfToken("rigExec:mask")),
            AttributeValue<TfToken>(TfToken("rigExec:pointSpace")),
            AttributeValue<GfMatrix4d>(TfToken("rigExec:cageMatrix")),
            AttributeValue<GfMatrix4d>(TfToken("rigExec:targetMatrix")),
            Relationship(TfToken("rigExec:frames")).TargetedObjects<GfMatrix4d>(RigExecMoverExecTokens->computeMatrix)
                .InputName(TfToken("latticeFrames")),
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
