// RigExecSurfaceMover: everything about the surface mover (spec §4.1).
// A surface mover projects points onto a driver surface. This TU owns
// its exec-side computeMoverParameters registration and builder, its
// revision binder, and its parity-oracle branch, and registers the row
// that points at them. Compile validation is the generic points-target
// rules.
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

bool SetSettings(RigExecMoverParameters *p, const TfToken &snap, const TfToken &space,
                 float offset, const std::vector<float> &mask, const std::vector<int> &triangles,
                 const GfMatrix4d &surface, const GfMatrix4d &target)
{
    using namespace rigExec;
    if(snap==TfToken("onSurface"))p->surfaceSettings.mode=RigExecSurfaceSnapMode::OnSurface;
    else if(snap==TfToken("inside"))p->surfaceSettings.mode=RigExecSurfaceSnapMode::Inside;
    else if(snap==TfToken("outside"))p->surfaceSettings.mode=RigExecSurfaceSnapMode::Outside;
    else if(snap==TfToken("outsideSurface"))p->surfaceSettings.mode=RigExecSurfaceSnapMode::OutsideSurface;
    else return false;
    p->surfaceSettings.offset=offset;p->surfaceSettings.mask=mask;p->surfaceSettings.triangles=triangles;
    if(!std::isfinite(offset) || !RigExecSurfaceSnapValidMatrix(surface) || !RigExecSurfaceSnapValidMatrix(target))return false;
    if(space==TfToken("local")) {
        p->targetToSurface=target*rigExec::RigExecSurfaceSnapAffineInverse(surface);p->surfaceToTarget=surface*rigExec::RigExecSurfaceSnapAffineInverse(target);
    } else if(space==TfToken("common")) {
        p->targetToSurface=rigExec::RigExecSurfaceSnapAffineInverse(surface);p->surfaceToMetric=rigExec::RigExecSurfaceSnapAffineInverse(surface);p->surfaceToTarget=surface;
    } else return false;
    return true;
}

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
    const auto read=[&](const char *name,auto fallback) {
        using T=decltype(fallback);const T *value=ctx.GetInputValuePtr<T>(TfToken(name));return value?*value:fallback;
    };
    auto surface=read("rigExec:surfaceMatrix",GfMatrix4d(1.0)),target=read("rigExec:targetMatrix",GfMatrix4d(1.0));
    if(!rigExec::RigExecSurfaceSnapValidMatrix(surface) || !rigExec::RigExecSurfaceSnapValidMatrix(target))return params;
    const auto frames=rigExec::RigExecMoverCollect<GfMatrix4d>(ctx,TfToken("surfaceFrames"));
    if(!frames.empty()) {
        if(frames.size()!=2)return params;surface*=frames[0];target*=frames[1];
        if(!rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&surface) || !rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&target))return params;
    }
    const auto mask=rigExec::RigExecMoverCollect<float>(ctx,TfToken("rigExec:mask"));
    const auto triangles=rigExec::RigExecMoverCollect<int>(ctx,TfToken("rigExec:triangles"));
    params.valid = !params.auxPoints.empty() &&
                   (!params.topologyCounts.empty() || !triangles.empty()) &&
        SetSettings(&params,read("rigExec:snapMode",TfToken("onSurface")),read("rigExec:pointSpace",TfToken("local")),
                    read("rigExec:offset",0.f),mask,triangles,surface,target);
    return params;
}

void
_BindSurfaceMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    binding.base = ctx.target;
    const UsdPrim &moverPrim = ctx.moverPrim;
    const SdfPathVector surfaces = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:surface");
    if (!surfaces.empty()) {
        const SdfPath surfacePrim = surfaces[0].GetPrimPath();
        binding.surfacePoints =
            surfacePrim.AppendProperty(TfToken("points"));
        const rigExec::RigExecReadPhase phase =
            rigExec::RigExecPhaseForInput(moverPrim, "rigExec:surface");
        if (!phase.IsBase()) {
            binding.phases[binding.surfacePoints] = phase;
        }
        binding.topologyCounts =
            surfacePrim.AppendProperty(TfToken("faceVertexCounts"));
        binding.topologyIndices =
            surfacePrim.AppendProperty(TfToken("faceVertexIndices"));
    }
    binding.influences=rigExec::RigExecRelationshipTargets(moverPrim,"rigExec:frames");
    binding.transformPhase=rigExec::RigExecPhaseForInput(moverPrim,"rigExec:frames");
    if(binding.transformPhase.kind==rigExec::RigExecReadPhaseKind::Final)
        for(auto &provider:binding.influences) {
            const auto head=ctx.frameChainHeads.find(provider);if(head!=ctx.frameChainHeads.end())provider=head->second;
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
        surfacePrim.AppendProperty(TfToken("points")), moverPath,
        &surfacePoints);
    if (const UsdPrim s = stage->GetPrimAtPath(surfacePrim)) {
        s.GetAttribute(TfToken("faceVertexCounts")).Get(&counts, time);
        s.GetAttribute(TfToken("faceVertexIndices"))
            .Get(&indices, time);
    }
    VtIntArray explicitTriangles;
    prim.GetAttribute(TfToken("rigExec:triangles")).Get(&explicitTriangles,time);
    if (surfacePoints.empty() || (counts.empty() && explicitTriangles.empty())) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": surface has no points/topology");
        return RigExecOracleResult::PassThrough;
    }
    std::vector<GfVec3f> scratch(points.begin(), points.end());
    const auto read=[&](const char *name,auto fallback) {
        const auto a=prim.GetAttribute(TfToken(name));if(a)ctx.resolved.GetAttribute(a,time,&fallback);return fallback;
    };
    auto surface=read("rigExec:surfaceMatrix",GfMatrix4d(1.0)),target=read("rigExec:targetMatrix",GfMatrix4d(1.0));
    if(!rigExec::RigExecSurfaceSnapValidMatrix(surface) || !rigExec::RigExecSurfaceSnapValidMatrix(target))return RigExecOracleResult::PassThrough;
    const auto frames=rigExec::RigExecRelationshipTargets(prim,"rigExec:frames");
    if(!frames.empty()) {
        if(frames.size()!=2)return RigExecOracleResult::PassThrough;
        const auto &table=rigExec::RigExecPhaseForInput(prim,"rigExec:frames").kind==rigExec::RigExecReadPhaseKind::Final?
            ctx.finalProviderMatrices:ctx.baseProviderMatrices;
        const auto s=table.find(frames[0]),t=table.find(frames[1]);
        if(s==table.end() || t==table.end())return RigExecOracleResult::PassThrough;
        surface*=s->second;target*=t->second;
        if(!rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&surface) || !rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&target))return RigExecOracleResult::PassThrough;
    }
    const auto mask=read("rigExec:mask",VtFloatArray());const auto triangles=read("rigExec:triangles",VtIntArray());
    RigExecMoverParameters params;
    if(!SetSettings(&params,read("rigExec:snapMode",TfToken("onSurface")),read("rigExec:pointSpace",TfToken("local")),
                    read("rigExec:offset",0.f),{mask.begin(),mask.end()},{triangles.begin(),triangles.end()},surface,target))
        return RigExecOracleResult::PassThrough;
    if(rigExec::RigExecSurfaceSnapIsLegacy(params.surfaceSettings,params.targetToSurface,params.surfaceToTarget,params.surfaceToMetric)) {
        rigExec::RigExecApplySurfaceProject(&scratch,{surfacePoints.begin(),surfacePoints.end()},
            {counts.begin(),counts.end()},{indices.begin(),indices.end()},1.0);
    } else if(!rigExec::RigExecApplySurfaceSnapKernel<GfVec3f,GfVec3d>(
        &scratch,
        std::vector<GfVec3f>(surfacePoints.begin(),
                             surfacePoints.end()),
        std::vector<int>(counts.begin(), counts.end()),
        std::vector<int>(indices.begin(), indices.end()),params.surfaceSettings,
        params.targetToSurface,params.surfaceToTarget,params.surfaceToMetric))return RigExecOracleResult::PassThrough;
    std::copy(scratch.begin(), scratch.end(), points.begin());
    return RigExecOracleResult::Blend;
}

bool ValidateSurface(const rigExec::RigExecMoverValidateContext &ctx,std::string *error)
{
    const auto &prim=ctx.prim;
    const auto frames=rigExec::RigExecRelationshipTargets(prim,"rigExec:frames");
    const auto surfaces=rigExec::RigExecRelationshipTargets(prim,"rigExec:surface");
    const auto fail=[&]() {*error="invalid surface snap mode, topology, mask or coordinate frame";return false;};
    if(surfaces.size()!=1 || (!frames.empty() && frames.size()!=2))return fail();
    if(!frames.empty() && rigExec::RigExecPhaseForInput(prim,"rigExec:frames").kind==rigExec::RigExecReadPhaseKind::AtPrim) {
        *error="surface frame providers require base or final read phase";return false;
    }
    const auto read=[&](const char *name,auto fallback) {
        const auto a=prim.GetAttribute(TfToken(name));if(a)a.Get(&fallback);return fallback;
    };
    const auto mask=read("rigExec:mask",VtFloatArray());const auto triangles=read("rigExec:triangles",VtIntArray());
    RigExecMoverParameters params;
    if(!SetSettings(&params,read("rigExec:snapMode",TfToken("onSurface")),read("rigExec:pointSpace",TfToken("local")),
                    read("rigExec:offset",0.f),{mask.begin(),mask.end()},{triangles.begin(),triangles.end()},
                    read("rigExec:surfaceMatrix",GfMatrix4d(1.0)),read("rigExec:targetMatrix",GfMatrix4d(1.0))))return fail();
    const auto surface=ctx.stage->GetPrimAtPath(surfaces[0].GetPrimPath());
    if(!surface || !surface.GetAttribute(TfToken("points")))return fail();
    for(float w:mask)if(!std::isfinite(w) || w<0 || w>1)return fail();
    if(triangles.size()%3)return fail();
    for(int i:triangles)if(i<0)return fail();
    // Shape-dependent checks belong to the evaluated frame. Default points
    // may be absent or degenerate in an otherwise valid animated surface.
    for(const auto &target:ctx.targets) {
        VtVec3fArray points;
        if(!mask.empty() && ctx.stage->GetAttributeAtPath(target).Get(&points) && mask.size()!=points.size())return fail();
    }
    return true;
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
    handler.validate = &ValidateSurface;
    handler.frameRelationships={"rigExec:frames"};handler.transformRelationship="rigExec:frames";
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
            AttributeValue<TfToken>(TfToken("rigExec:snapMode")),
            AttributeValue<TfToken>(TfToken("rigExec:pointSpace")),
            AttributeValue<float>(TfToken("rigExec:offset")),
            AttributeValue<float>(TfToken("rigExec:mask")),
            AttributeValue<int>(TfToken("rigExec:triangles")),
            AttributeValue<GfMatrix4d>(TfToken("rigExec:surfaceMatrix")),
            AttributeValue<GfMatrix4d>(TfToken("rigExec:targetMatrix")),
            Relationship(TfToken("rigExec:frames")).TargetedObjects<GfMatrix4d>(RigExecMoverExecTokens->computeMatrix)
                .InputName(TfToken("surfaceFrames")),
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
