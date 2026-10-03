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
    ((distance,"inputs:distanceWeight")) ((detail,"inputs:displacement")));
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
    if(p.restPoints.empty())p.restPoints=RigExecMoverCollect<GfVec3f>(ctx,RigExecMoverExecTokens->basePoints);
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
}
bool Validate(const RigExecMoverValidateContext &ctx,std::string *error) {
    if(!UsdGeomMesh(ctx.stage->GetPrimAtPath(ctx.targets.front().GetPrimPath()))) {
        *error="DeltaMush requires polygon mesh topology";return false;
    }
    auto attr=ctx.prim.GetAttribute(_mush->rest);
    if(attr.GetNumTimeSamples()!=0 || attr.HasAuthoredConnections()) {
        *error="DeltaMush restPoints must be static";return false;
    }
    return true;
}
RigExecOracleResult Oracle(const RigExecMoverOracleContext &ctx) {
    VtVec3fArray rest;ctx.prim.GetAttribute(_mush->rest).Get(&rest);
    if(rest.empty())rest=ctx.basePoints;
    VtIntArray counts,indices;auto owner=ctx.stage->GetPrimAtPath(ctx.target.GetPrimPath());
    owner.GetAttribute(TfToken("faceVertexCounts")).Get(&counts,ctx.time);
    owner.GetAttribute(TfToken("faceVertexIndices")).Get(&indices,ctx.time);
    int iterations=10;float step=0.5f,distance=0,detail=1;bool pin=true;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->iterations),ctx.time,&iterations);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->step),ctx.time,&step);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->pin),ctx.time,&pin);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->distance),ctx.time,&distance);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->detail),ctx.time,&detail);
    std::vector<GfVec3f> points(ctx.points->begin(),ctx.points->end());
    if(!RigExecApplyDeltaMush(&points,{rest.begin(),rest.end()},{counts.begin(),counts.end()},
        {indices.begin(),indices.end()},iterations,step,pin,distance,detail)) {
        ctx.diagnostics->push_back("MoverFailed "+ctx.moverPath.GetString()+": invalid delta mush inputs");
        return RigExecOracleResult::PassThrough;
    }
    std::copy(points.begin(),points.end(),ctx.points->begin());return RigExecOracleResult::Blend;
}
RigExecMoverHandler Handler() {
    RigExecMoverHandler h("RigExecDeltaMushMover",&RigExecFixedMoverOp<RigExecRevisionOp::DeltaMush>,RigExecMoverDomain::Points);
    h.singleTarget=true;h.bind=&Bind;h.validate=&Validate;h.oracle=&Oracle;return h;
}
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecDeltaMushMover) {
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters).Callback<RigExecMoverParameters>(&Parameters).Inputs(
        RIGEXEC_MOVER_COMMON_INPUTS,
        AttributeValue<GfVec3f>(_mush->rest),AttributeValue<int>(_mush->iterations),
        AttributeValue<float>(_mush->step),AttributeValue<bool>(_mush->pin),
        AttributeValue<float>(_mush->distance),AttributeValue<float>(_mush->detail),
        Relationship(RigExecMoverExecTokens->resolvedBase).TargetedObjects<GfVec3f>(ExecBuiltinComputations->computeValue).InputName(RigExecMoverExecTokens->basePoints),
        Relationship(RigExecMoverExecTokens->resolvedTopologyCounts).TargetedObjects<int>(ExecBuiltinComputations->computeValue).InputName(RigExecMoverExecTokens->topologyCounts),
        Relationship(RigExecMoverExecTokens->resolvedTopologyIndices).TargetedObjects<int>(ExecBuiltinComputations->computeValue).InputName(RigExecMoverExecTokens->topologyIndices));
    self.PrimComputation(RigExecMoverExecTokens->computeMoverStatus).Callback<RigExecMoverStatus>(&RigExecMoverBuildStatus).Inputs(
        Computation<RigExecMoverParameters>(RigExecMoverExecTokens->computeMoverParameters).Required(),
        Computation<SdfPath>(ExecBuiltinComputations->computePath).InputName(RigExecMoverExecTokens->moverPath));
}
RIGEXEC_REGISTER_MOVER(Handler());
