#include "moverRegistry.h"
#include "moverExecCommon.h"
#include "rigExecMath/geometryKernels.h"
#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/usd/usdGeom/mesh.h"

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
TF_DEFINE_PRIVATE_TOKENS(_mush,
    ((rest,"inputs:restPoints")) ((iterations,"inputs:iterations"))
    ((step,"inputs:step")) ((pin,"inputs:pinBorders"))
    ((distance,"inputs:distanceWeight")) ((detail,"inputs:displacement"))
    ((smoothing,"inputs:smoothing")) ((transport,"inputs:frameTransport"))
    ((smoothWeights,"inputs:smoothWeights")) ((edges,"inputs:edges"))
    ((onlySmooth,"inputs:onlySmooth")) ((computationToTarget,"inputs:computationToTarget"))
    ((frame,"rigExec:frame")) (frameTransforms));
namespace {
template<class T> T Input(const VdfContext &ctx,const TfToken &name,T fallback) {
    const auto *value=ctx.GetInputValuePtr<T>(name);return value ? *value : fallback;
}
RigExecMoverParameters Parameters(const VdfContext &ctx) {
    RigExecMoverParameters p;p.kind=TfToken("deltaMush");
    p.enabled=Input(ctx,RigExecMoverExecTokens->enabled,true);
    if(!p.enabled) {p.valid=true;return p;}
    if(!RigExecMoverSetCommonEnvelope(ctx,&p))return p;
    p.restPoints=RigExecMoverCollect<GfVec3f>(ctx,_mush->rest);
    p.mushComputationToTarget=Input(ctx,_mush->computationToTarget,GfMatrix4d(1));
    const auto frames=RigExecMoverCollect<GfMatrix4d>(ctx,_mush->frameTransforms);
    if(frames.size()>1)return p;
    const bool needsRest=p.mushComputationToTarget!=GfMatrix4d(1)||!frames.empty();
    if(!frames.empty())p.mushComputationToTarget*=frames.front();
    p.mushSettings.onlySmooth=Input(ctx,_mush->onlySmooth,false);
    if(p.restPoints.empty()) {
        if(needsRest&&!p.mushSettings.onlySmooth)return p;
        p.restPoints=RigExecMoverCollect<GfVec3f>(ctx,RigExecMoverExecTokens->basePoints);
    }
    if(!RigExecParseDeltaMushSmoothing(Input(ctx,_mush->smoothing,TfToken("rest")).GetString(),&p.mushSettings.smoothing)||
       !RigExecParseDeltaMushFrameTransport(Input(ctx,_mush->transport,TfToken("vertex")).GetString(),&p.mushSettings.frameTransport))return p;
    p.mushSettings.smoothWeights=RigExecMoverCollect<float>(ctx,_mush->smoothWeights);
    p.mushSettings.edges=RigExecMoverCollect<int>(ctx,_mush->edges);
    p.topologyCounts=RigExecMoverCollect<int>(ctx,RigExecMoverExecTokens->topologyCounts);
    p.topologyIndices=RigExecMoverCollect<int>(ctx,RigExecMoverExecTokens->topologyIndices);
    p.mushIterations=Input(ctx,_mush->iterations,10);p.mushStep=Input(ctx,_mush->step,0.5f);
    p.mushPinBorders=Input(ctx,_mush->pin,true);p.mushDistanceWeight=Input(ctx,_mush->distance,0.0f);
    p.mushDisplacement=Input(ctx,_mush->detail,1.0f);
    p.valid=!p.restPoints.empty() && !p.topologyCounts.empty();return p;
}
void Bind(const RigExecMoverBindContext &ctx) {
    ctx.binding->base=ctx.target;
    ctx.binding->topologyCounts=ctx.ownerPath.AppendProperty(TfToken("faceVertexCounts"));
    ctx.binding->topologyIndices=ctx.ownerPath.AppendProperty(TfToken("faceVertexIndices"));
    ctx.binding->influences=RigExecRelationshipTargets(ctx.moverPrim,"rigExec:frame");
    ctx.binding->transformPhase=RigExecPhaseForInput(ctx.moverPrim,"rigExec:frame");
    if(ctx.binding->transformPhase.kind==RigExecReadPhaseKind::Final)
        for(auto &path:ctx.binding->influences) {
            const auto it=ctx.frameChainHeads.find(path);if(it!=ctx.frameChainHeads.end())path=it->second;
        }
}
bool Validate(const RigExecMoverValidateContext &ctx,std::string *error) {
    if(!UsdGeomMesh(ctx.stage->GetPrimAtPath(ctx.targets.front().GetPrimPath()))) {
        *error="DeltaMush requires polygon mesh topology";return false;
    }
    auto attr=ctx.prim.GetAttribute(_mush->rest);
    if(attr.GetNumTimeSamples()!=0 || attr.HasAuthoredConnections()) {
        *error="DeltaMush restPoints must be static";return false;
    }
    const auto frames=RigExecRelationshipTargets(ctx.prim,"rigExec:frame");
    if(frames.size()>1){*error="DeltaMush frame takes at most one provider";return false;}
    if(RigExecPhaseForInput(ctx.prim,"rigExec:frame").kind==RigExecReadPhaseKind::AtPrim) {
        *error="DeltaMush frame read phase must be base or final";return false;
    }
    for(const auto &name:{_mush->smoothing,_mush->transport,_mush->edges}) {
        const auto a=ctx.prim.GetAttribute(name);
        if(a.GetNumTimeSamples()!=0||a.HasAuthoredConnections()){*error="DeltaMush topology and mode inputs must be static";return false;}
    }
    VtVec3fArray rest;attr.Get(&rest);GfMatrix4d matrix(1);ctx.prim.GetAttribute(_mush->computationToTarget).Get(&matrix);
    bool only=false;ctx.prim.GetAttribute(_mush->onlySmooth).Get(&only);
    if(rest.empty()&&(!frames.empty()||matrix!=GfMatrix4d(1))&&!only) {
        *error="DeltaMush computation transform requires explicit restPoints";return false;
    }
    return true;
}
RigExecOracleResult Oracle(const RigExecMoverOracleContext &ctx) {
    VtVec3fArray rest;ctx.prim.GetAttribute(_mush->rest).Get(&rest);

    VtIntArray counts,indices;auto owner=ctx.stage->GetPrimAtPath(ctx.target.GetPrimPath());
    owner.GetAttribute(TfToken("faceVertexCounts")).Get(&counts,ctx.time);
    owner.GetAttribute(TfToken("faceVertexIndices")).Get(&indices,ctx.time);
    int iterations=10;float step=0.5f,distance=0,detail=1;bool pin=true;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->iterations),ctx.time,&iterations);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->step),ctx.time,&step);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->pin),ctx.time,&pin);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->distance),ctx.time,&distance);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->detail),ctx.time,&detail);
    RigExecDeltaMushSettings settings;TfToken smoothing("rest"),transport("vertex");
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->smoothing),UsdTimeCode::Default(),&smoothing);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->transport),UsdTimeCode::Default(),&transport);
    if(!RigExecParseDeltaMushSmoothing(smoothing.GetString(),&settings.smoothing)||
       !RigExecParseDeltaMushFrameTransport(transport.GetString(),&settings.frameTransport))return RigExecOracleResult::PassThrough;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->onlySmooth),ctx.time,&settings.onlySmooth);
    VtFloatArray weights;VtIntArray edges;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->smoothWeights),ctx.time,&weights);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->edges),UsdTimeCode::Default(),&edges);
    settings.smoothWeights.assign(weights.begin(),weights.end());settings.edges.assign(edges.begin(),edges.end());
    GfMatrix4d computationToTarget(1);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->computationToTarget),ctx.time,&computationToTarget);
    const auto frames=RigExecRelationshipTargets(ctx.prim,"rigExec:frame");
    if(rest.empty()&&(!frames.empty()||computationToTarget!=GfMatrix4d(1))&&!settings.onlySmooth)return RigExecOracleResult::PassThrough;
    if(rest.empty())rest=ctx.basePoints;
    if(frames.size()>1)return RigExecOracleResult::PassThrough;
    if(!frames.empty()) {
        const bool final=RigExecPhaseForInput(ctx.prim,"rigExec:frame").kind==RigExecReadPhaseKind::Final;
        const auto &matrices=final?ctx.finalProviderMatrices:ctx.baseProviderMatrices;
        const auto it=matrices.find(frames.front());if(it==matrices.end())return RigExecOracleResult::PassThrough;
        computationToTarget*=it->second;
    }
    std::vector<GfVec3f> points(ctx.points->begin(),ctx.points->end());
    if(!RigExecApplyDeltaMush(&points,{rest.begin(),rest.end()},{counts.begin(),counts.end()},
        {indices.begin(),indices.end()},iterations,step,pin,distance,detail,settings,computationToTarget)) {
        ctx.diagnostics->push_back("MoverFailed "+ctx.moverPath.GetString()+": invalid delta mush inputs");
        return RigExecOracleResult::PassThrough;
    }
    std::copy(points.begin(),points.end(),ctx.points->begin());return RigExecOracleResult::Blend;
}
RigExecMoverHandler Handler() {
    RigExecMoverHandler h("RigExecDeltaMushMover",&RigExecFixedMoverOp<RigExecRevisionOp::DeltaMush>,RigExecMoverDomain::Points);
    h.frameRelationships={"rigExec:frame"};h.transformRelationship="rigExec:frame";
    h.singleTarget=true;h.bind=&Bind;h.validate=&Validate;h.oracle=&Oracle;return h;
}
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecDeltaMushMover) {
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters).Callback<RigExecMoverParameters>(&Parameters).Inputs(
        RIGEXEC_MOVER_COMMON_INPUTS,
        AttributeValue<GfVec3f>(_mush->rest),AttributeValue<int>(_mush->iterations),
        AttributeValue<float>(_mush->step),AttributeValue<bool>(_mush->pin),
        AttributeValue<float>(_mush->distance),AttributeValue<float>(_mush->detail),
        AttributeValue<TfToken>(_mush->smoothing),AttributeValue<TfToken>(_mush->transport),
        AttributeValue<float>(_mush->smoothWeights),AttributeValue<int>(_mush->edges),
        AttributeValue<bool>(_mush->onlySmooth),AttributeValue<GfMatrix4d>(_mush->computationToTarget),
        Relationship(_mush->frame).TargetedObjects<GfMatrix4d>(RigExecMoverExecTokens->computeMatrix).InputName(_mush->frameTransforms),
        Relationship(RigExecMoverExecTokens->resolvedBase).TargetedObjects<GfVec3f>(ExecBuiltinComputations->computeValue).InputName(RigExecMoverExecTokens->basePoints),
        Relationship(RigExecMoverExecTokens->resolvedTopologyCounts).TargetedObjects<int>(ExecBuiltinComputations->computeValue).InputName(RigExecMoverExecTokens->topologyCounts),
        Relationship(RigExecMoverExecTokens->resolvedTopologyIndices).TargetedObjects<int>(ExecBuiltinComputations->computeValue).InputName(RigExecMoverExecTokens->topologyIndices));
    self.PrimComputation(RigExecMoverExecTokens->computeMoverStatus).Callback<RigExecMoverStatus>(&RigExecMoverBuildStatus).Inputs(
        Computation<RigExecMoverParameters>(RigExecMoverExecTokens->computeMoverParameters).Required(),
        Computation<SdfPath>(ExecBuiltinComputations->computePath).InputName(RigExecMoverExecTokens->moverPath));
}
RIGEXEC_REGISTER_MOVER(Handler());
