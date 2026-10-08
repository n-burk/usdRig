#include "moverRegistry.h"
#include "moverExecCommon.h"
#include "rigExecMath/geometryKernels.h"
#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/usd/usdGeom/mesh.h"

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

TF_DEFINE_PRIVATE_TOKENS(_wrinkle,
    ((rest, "inputs:restPoints"))
    ((iterations, "inputs:iterations"))
    ((topology, "inputs:topology"))
    ((neighborDistance, "inputs:neighborDistance"))
    ((restLengthScale, "inputs:restLengthScale"))
    ((stretchStiffness, "inputs:stretchStiffness"))
    ((compressionStiffness, "inputs:compressionStiffness"))
    ((bendStiffness, "inputs:bendStiffness"))
    ((maxDisplacement, "inputs:maxDisplacement"))
    ((pinBorders, "inputs:pinBorders"))
    ((pinPoints, "inputs:pinPoints"))
    ((tangentPlaneCollisions, "inputs:tangentPlaneCollisions"))
    ((tangentPlaneInset, "inputs:tangentPlaneInset"))
    ((wrinkleScale, "inputs:wrinkleScale"))
    ((smoothingIterations, "inputs:smoothingIterations"))
    (cloth) (surfaceStruts));

namespace {
const TfToken _oracleToken0("faceVertexCounts");
const TfToken _oracleToken1("faceVertexIndices");


template<class T>
T Input(const VdfContext &ctx, const TfToken &name, T fallback)
{
    const auto *value = ctx.GetInputValuePtr<T>(name);
    return value ? *value : fallback;
}

RigExecMoverParameters Parameters(const VdfContext &ctx)
{
    RigExecMoverParameters p;
    p.kind = TfToken("wrinkle");
    p.enabled = Input(ctx, RigExecMoverExecTokens->inputsEnabled, true);
    if (!p.enabled) {
        p.valid = true;
        return p;
    }
    if (!RigExecMoverSetCommonEnvelope(ctx, &p)) return p;
    p.restPoints = RigExecMoverCollect<GfVec3f>(ctx, _wrinkle->rest);
    if (p.restPoints.empty()) {
        p.restPoints = RigExecMoverCollect<GfVec3f>(
            ctx, RigExecMoverExecTokens->basePoints);
    }
    p.topologyCounts = RigExecMoverCollect<int>(
        ctx, RigExecMoverExecTokens->topologyCounts);
    p.topologyIndices = RigExecMoverCollect<int>(
        ctx, RigExecMoverExecTokens->topologyIndices);
    auto &s = p.wrinkleSettings;
    const TfToken topology = Input(ctx, _wrinkle->topology, _wrinkle->cloth);
    if (topology != _wrinkle->cloth && topology != _wrinkle->surfaceStruts) return p;
    s.topology = topology == _wrinkle->cloth
        ? RigExecWrinkleTopology::Cloth : RigExecWrinkleTopology::SurfaceStruts;
    s.pinPoints = RigExecMoverCollect<int>(ctx, _wrinkle->pinPoints);
    s.iterations = Input(ctx, _wrinkle->iterations, s.iterations);
    s.neighborDistance = Input(ctx, _wrinkle->neighborDistance, s.neighborDistance);
    s.restLengthScale = Input(ctx, _wrinkle->restLengthScale, s.restLengthScale);
    s.stretchStiffness = Input(ctx, _wrinkle->stretchStiffness, s.stretchStiffness);
    s.compressionStiffness = Input(ctx, _wrinkle->compressionStiffness, s.compressionStiffness);
    s.bendStiffness = Input(ctx, _wrinkle->bendStiffness, s.bendStiffness);
    s.maxDisplacement = Input(ctx, _wrinkle->maxDisplacement, s.maxDisplacement);
    s.pinBorders = Input(ctx, _wrinkle->pinBorders, s.pinBorders);
    s.tangentPlaneCollisions = Input(ctx, _wrinkle->tangentPlaneCollisions, s.tangentPlaneCollisions);
    s.tangentPlaneInset = Input(ctx, _wrinkle->tangentPlaneInset, s.tangentPlaneInset);
    s.wrinkleScale = Input(ctx, _wrinkle->wrinkleScale, s.wrinkleScale);
    s.smoothingIterations = Input(ctx, _wrinkle->smoothingIterations, s.smoothingIterations);
    p.valid = !p.restPoints.empty() && !p.topologyCounts.empty();
    return p;
}

void Bind(const RigExecMoverBindContext &ctx)
{
    ctx.binding->base = ctx.target;
    ctx.binding->topologyCounts =
        ctx.ownerPath.AppendProperty(TfToken("faceVertexCounts"));
    ctx.binding->topologyIndices =
        ctx.ownerPath.AppendProperty(TfToken("faceVertexIndices"));
}

bool Validate(const RigExecMoverValidateContext &ctx, std::string *error)
{
    if (!UsdGeomMesh(ctx.stage->GetPrimAtPath(ctx.targets.front().GetPrimPath()))) {
        *error = "Wrinkle requires polygon mesh topology";
        return false;
    }
    for (const TfToken &name : {_wrinkle->rest, _wrinkle->topology, _wrinkle->pinPoints}) {
        const UsdAttribute attr = ctx.prim.GetAttribute(name);
        if (attr.GetNumTimeSamples() != 0 || attr.HasAuthoredConnections()) {
            *error = "Wrinkle " + name.GetString() + " must be static and unconnected";
            return false;
        }
    }
    TfToken topology;
    ctx.prim.GetAttribute(_wrinkle->topology).Get(&topology);
    if (topology != _wrinkle->cloth && topology != _wrinkle->surfaceStruts) {
        *error = "Wrinkle topology must be cloth or surfaceStruts";
        return false;
    }
    return true;
}

RigExecOracleResult Oracle(const RigExecMoverOracleContext &ctx)
{
    VtVec3fArray rest;
    ctx.prim.GetAttribute(_wrinkle->rest).Get(&rest);
    if (rest.empty()) rest = ctx.basePoints;
    const RigExecOraclePrim owner = ctx.stage->GetPrimAtPath(ctx.target.GetPrimPath());
    VtIntArray counts, indices;
    owner.GetAttribute(_oracleToken0).Get(&counts, ctx.time);
    owner.GetAttribute(_oracleToken1).Get(&indices, ctx.time);
    RigExecWrinkleSettings s;
    TfToken topology;
    ctx.prim.GetAttribute(_wrinkle->topology).Get(&topology);
    s.topology = topology == _wrinkle->cloth
        ? RigExecWrinkleTopology::Cloth : RigExecWrinkleTopology::SurfaceStruts;
    VtIntArray pins;
    ctx.prim.GetAttribute(_wrinkle->pinPoints).Get(&pins);
    s.pinPoints.assign(pins.begin(), pins.end());
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->iterations), ctx.time, &s.iterations);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->neighborDistance), ctx.time, &s.neighborDistance);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->restLengthScale), ctx.time, &s.restLengthScale);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->stretchStiffness), ctx.time, &s.stretchStiffness);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->compressionStiffness), ctx.time, &s.compressionStiffness);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->bendStiffness), ctx.time, &s.bendStiffness);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->maxDisplacement), ctx.time, &s.maxDisplacement);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->pinBorders), ctx.time, &s.pinBorders);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->tangentPlaneCollisions), ctx.time, &s.tangentPlaneCollisions);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->tangentPlaneInset), ctx.time, &s.tangentPlaneInset);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->wrinkleScale), ctx.time, &s.wrinkleScale);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->smoothingIterations), ctx.time, &s.smoothingIterations);
    std::vector<GfVec3f> points(ctx.points->begin(), ctx.points->end());
    if (!RigExecApplyWrinkle(&points, {rest.begin(), rest.end()},
            {counts.begin(), counts.end()}, {indices.begin(), indices.end()},
            s)) {
        ctx.diagnostics->push_back("MoverFailed " + ctx.moverPath.GetString() +
            ": invalid wrinkle inputs");
        return RigExecOracleResult::PassThrough;
    }
    std::copy(points.begin(), points.end(), ctx.points->begin());
    return RigExecOracleResult::Blend;
}

RigExecMoverHandler Handler()
{
    RigExecMoverHandler h("RigExecWrinkleMover",
        &RigExecFixedMoverOp<RigExecRevisionOp::Wrinkle>, RigExecMoverDomain::Points);
    h.singleTarget = true;
    h.bind = &Bind;
    h.validate = &Validate;
    h.oracle = &Oracle;
    return h;
}

} // namespace

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecWrinkleMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&Parameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            AttributeValue<GfVec3f>(_wrinkle->rest),
            AttributeValue<int>(_wrinkle->iterations),
            AttributeValue<TfToken>(_wrinkle->topology),
            AttributeValue<int>(_wrinkle->neighborDistance),
            AttributeValue<float>(_wrinkle->restLengthScale),
            AttributeValue<float>(_wrinkle->stretchStiffness),
            AttributeValue<float>(_wrinkle->compressionStiffness),
            AttributeValue<float>(_wrinkle->bendStiffness),
            AttributeValue<float>(_wrinkle->maxDisplacement),
            AttributeValue<bool>(_wrinkle->pinBorders),
            AttributeValue<int>(_wrinkle->pinPoints),
            AttributeValue<bool>(_wrinkle->tangentPlaneCollisions),
            AttributeValue<float>(_wrinkle->tangentPlaneInset),
            AttributeValue<float>(_wrinkle->wrinkleScale),
            AttributeValue<int>(_wrinkle->smoothingIterations),
            Relationship(RigExecMoverExecTokens->resolvedBase)
                .TargetedObjects<GfVec3f>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->basePoints),
            Relationship(RigExecMoverExecTokens->resolvedTopologyCounts)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->topologyCounts),
            Relationship(RigExecMoverExecTokens->resolvedTopologyIndices)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->topologyIndices));
    self.PrimComputation(RigExecMoverExecTokens->computeMoverStatus)
        .Callback<RigExecMoverStatus>(&RigExecMoverBuildStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                RigExecMoverExecTokens->computeMoverParameters).Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(RigExecMoverExecTokens->moverPath));
}

RIGEXEC_REGISTER_MOVER(Handler());
