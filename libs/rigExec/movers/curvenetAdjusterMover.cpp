//
// RigExecCurvenetAdjusterMover: everything about the curvenet-adjuster
// mover (spec §4.1).
//
// An adjuster mover poses the knots of a RigExecCurvenet through
// RigExecCurvenetAdjustment prims. The adjuster has no exec-side
// computation: its packet is assembled directly by
// RigExecAssembleCurvenetAdjusterParameters below. This TU owns that
// assembler, its revision binder, its compile validator, and its
// parity-oracle branch, and registers the row that points at them.
//

#include "moverRegistry.h"
#include "rigExecMath/avarScale.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"

#include <cmath>
#include <map>
#include <set>

namespace rigExec {
namespace {
template<class T> T _Read(const UsdPrim &prim, const std::string &name,
    T fallback, UsdTimeCode time, const RigExecResolvedInputs *resolved,
    RigExecBakeReadRecorder *bakeRecorder=nullptr) {
    const auto attr=prim.GetAttribute(TfToken(name));
    if (!bakeRecorder && resolved) bakeRecorder=resolved->bakeRecorder;
    if (attr) {
        if (resolved && resolved->GetAttribute(attr,time,&fallback))
            RigExecRecordStageRead(resolved,bakeRecorder,attr.GetPath(),attr,
                time,VtValue(fallback),/*forceFrame=*/true);
        else {
            if (!resolved) attr.Get(&fallback,time);
            RigExecRecordStageRead(nullptr,bakeRecorder,attr.GetPath(),attr,
                time,VtValue(fallback),/*forceFrame=*/false);
        }
    } else
        RigExecRecordStageRead(nullptr,bakeRecorder,
            prim.GetPath().AppendProperty(TfToken(name)),attr,
            time,VtValue(fallback),/*forceFrame=*/false);
    return fallback;
}
GfMatrix4d _Space(const UsdPrim &prim,const char *name,const GfMatrix4d &fallback,
    UsdTimeCode time,const RigExecResolvedInputs *resolved) {
    const auto attr=prim.GetAttribute(TfToken(name));
    if (!attr) return fallback;
    SdfPathVector connections; attr.GetConnections(&connections);
    const auto value=_Read(prim,name,GfMatrix4d(1),time,resolved);
    return !connections.empty() || value!=GfMatrix4d(1) ? value : fallback;
}
GfMatrix4d _Channels(const UsdPrim &prim,const std::string &prefix,
    UsdTimeCode time,const RigExecResolvedInputs *resolved) {
    const auto scalar=[&](const char *name,double fallback=0.0) {
        return _Read(prim,prefix+name,fallback,time,resolved);
    };
    GfMatrix4d result(1);
    if (prefix=="avars:") result.SetScale(GfVec3d(
        RigExecNormalizeAvarScale(scalar("sx",1)),
        RigExecNormalizeAvarScale(scalar("sy",1)),
        RigExecNormalizeAvarScale(scalar("sz",1))));
    const double angles[]{scalar("rx"),scalar("ry"),scalar("rz")};
    const GfVec3d axes[]{GfVec3d(1,0,0),GfVec3d(0,1,0),GfVec3d(0,0,1)};
    const TfToken order=prefix=="avars:"
        ? _Read(prim,"avars:rotationOrder",TfToken("XYZ"),time,resolved) : TfToken("XYZ");
    for (const char c:order.GetString()) {
        const int axis=c=='X'?0:c=='Y'?1:2;
        result=result*GfMatrix4d(GfRotation(axes[axis],angles[axis]),GfVec3d(0));
    }
    if (prefix=="avars:") result=result*GfMatrix4d(
        GfRotation(axes[0],scalar("rspin")),GfVec3d(0));
    const double units=prefix=="avars:" ? scalar("unitScaleFactor",1) : 1;
    GfMatrix4d translation(1);
    translation.SetTranslate(GfVec3d(scalar("tx"),scalar("ty"),scalar("tz"))*units);
    return result*translation;
}
GfMatrix4d _Local(const UsdPrim &prim,UsdTimeCode time,
    const RigExecResolvedInputs *resolved) {
    const auto rest=_Channels(prim,"rest:",time,resolved)*
        _Read(prim,"rest:space",GfMatrix4d(1),time,resolved);
    auto defaults=_Space(prim,"default:space",
        _Channels(prim,"default:",time,resolved)*rest,time,resolved);
    defaults=_Space(prim,"avars:defaultSpace",defaults,time,resolved);
    defaults=_Space(prim,"posed:defaultSpace",defaults,time,resolved);
    const auto parentDefault=_Space(prim,"parent:defaultSpace",GfMatrix4d(1),time,resolved);
    const auto parent=_Space(prim,"parent:space",GfMatrix4d(1),time,resolved);
    return _Space(prim,"posed:space",_Channels(prim,"avars:",time,resolved)*
        defaults*parentDefault.GetInverse()*parent,time,resolved);
}
} // namespace

std::vector<SdfPath> RigExecCurvenetAdjustmentPaths(const UsdPrim &mover) {
    SdfPathVector roots;
    mover.GetRelationship(TfToken("rigExec:adjustments")).GetTargets(&roots);
    std::vector<SdfPath> result;
    std::set<SdfPath> visited;
    for (const auto &path:roots) {
        const auto prim=mover.GetStage()->GetPrimAtPath(path);
        if (!prim) { result.push_back(path); continue; }
        for (const auto &child:UsdPrimRange(prim)) {
            if (child==prim || child.GetTypeName()=="RigExecCurvenetAdjustment") {
                if (visited.insert(child.GetPath()).second) result.push_back(child.GetPath());
            }
        }
    }
    return result;
}

RigExecMoverParameters RigExecAssembleCurvenetAdjusterParameters(
    const UsdPrim &mover,const SdfPath &target,const RigExecWeightPacket *weights,
    UsdTimeCode time,const RigExecResolvedInputs *resolved) {
    RigExecMoverParameters p;
    p.kind=TfToken("curvenetAdjuster");
    p.enabled=_Read(mover,"inputs:enabled",true,time,resolved);
    p.weights=weights ? *weights : RigExecWeightPacket::Constant(
        _Read(mover,"inputs:defaultWeight",1.0f,time,resolved));
    if (!p.enabled) { p.valid=true; return p; }
    const auto net=mover.GetStage()->GetPrimAtPath(target.GetPrimPath());
    if (!net || net.GetTypeName()!="RigExecCurvenet" || !p.weights.valid) return p;
    const auto rest=_Read(net,"points",VtVec3fArray(),UsdTimeCode::Default(),nullptr,
        resolved ? resolved->bakeRecorder : nullptr);
    p.restPoints.assign(rest.begin(),rest.end());
    const auto indices=_Read(net,"rigExec:splineIndices",VtIntArray(),time,resolved);
    p.topologyIndices.assign(indices.begin(),indices.end());
    const auto basis=_Read(net,"rigExec:basis",TfToken("bezier"),time,resolved);
    if (basis!="bezier" && basis!="catmullRom") return p;
    p.curvenetAdjustmentBasis=basis=="bezier" ? RigExecCurvenetBasis::Bezier : RigExecCurvenetBasis::CatmullRom;
    const auto paths=RigExecCurvenetAdjustmentPaths(mover);
    if (paths.empty()) return p;
    std::map<SdfPath,int> commandIndex;
    for (const auto &path:paths) {
        const auto adjustment=mover.GetStage()->GetPrimAtPath(path);
        if (!adjustment || adjustment.GetTypeName()!="RigExecCurvenetAdjustment") return p;
        SdfPathVector nets;
        adjustment.GetRelationship(TfToken("rigExec:curvenet")).GetTargets(&nets);
        if (nets.size()!=1 || nets[0]!=net.GetPath()) return p;
        RigExecCurvenetAdjustmentCommand command;
        command.pointIndex=_Read(adjustment,"rigExec:knotIndex",-1,time,resolved);
        command.includeTangents=_Read(adjustment,"rigExec:includeTangents",true,time,resolved);
        const auto kind=_Read(adjustment,"rigExec:pointKind",TfToken("knot"),time,resolved);
        if (kind=="tangent") {
            const auto parent=commandIndex.find(path.GetParentPath());
            if (parent==commandIndex.end()) return p;
            command.parentCommand=parent->second;
        } else if (kind!="knot") return p;
        command.localTransform=_Local(adjustment,time,resolved);
        commandIndex[path]=int(p.curvenetAdjustments.size());
        p.curvenetAdjustments.push_back(command);
    }
    p.valid=!p.restPoints.empty() && !p.topologyIndices.empty();
    return p;
}

namespace {
bool _ValidateCurvenetAdjusterBinding(const UsdPrim &mover,
    const SdfPath &target,std::string *error) {
    auto p=RigExecAssembleCurvenetAdjusterParameters(mover,target,nullptr,UsdTimeCode::Default(),nullptr);
    // Disabled controls still need a valid topology and binding at compile.
    if (!p.enabled) {
        const auto attr=mover.GetAttribute(TfToken("inputs:enabled"));
        RigExecResolvedInputs inputs;
        inputs.SetProperty(attr.GetPath(),VtValue(true));
        p=RigExecAssembleCurvenetAdjusterParameters(mover,target,nullptr,UsdTimeCode::Default(),&inputs);
    }
    if (!p.valid) {
        if (error) *error="adjuster requires a native curvenet target and valid adjustment bindings";
        return false;
    }
    auto points=p.restPoints;
    return RigExecApplyCurvenetAdjustments(&points,p.restPoints,p.topologyIndices,
        p.curvenetAdjustmentBasis,p.curvenetAdjustments,nullptr,error);
}
}  // namespace

namespace {

void
_BindCurvenetAdjusterMover(const RigExecMoverBindContext &ctx)
{
    RigExecRevisionBinding &binding = *ctx.binding;
    const SdfPath &target = ctx.target;
    binding.base = target;
    binding.curvenet = target.GetPrimPath();
}

bool
_ValidateCurvenetAdjusterMover(
    const RigExecMoverValidateContext &ctx, std::string *error)
{
    // The single-target generic rule runs first, so targets[0] is the
    // one canonical target.
    std::string inner;
    if (!_ValidateCurvenetAdjusterBinding(
            ctx.prim, ctx.targets[0], &inner)) {
        *error = ctx.prim.GetPath().GetString() + ": " + inner;
        return false;
    }
    return true;
}

RigExecOracleResult
_OracleCurvenetAdjusterMover(const RigExecMoverOracleContext &ctx)
{
    const UsdPrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    const SdfPath &target = ctx.target;
    const UsdTimeCode time = ctx.time;
    const RigExecResolvedInputs &resolved = ctx.resolved;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    const auto p = RigExecAssembleCurvenetAdjusterParameters(
        prim, target, nullptr, time, &resolved);
    std::vector<GfVec3f> scratch(points.begin(), points.end());
    std::string applyError;
    if (!p.valid || !RigExecApplyCurvenetAdjustments(&scratch,
            p.restPoints, p.topologyIndices, p.curvenetAdjustmentBasis,
            p.curvenetAdjustments, nullptr, &applyError)) {
        if (diagnostics) {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() + ": " + applyError);
        }
        return RigExecOracleResult::PassThrough;
    }
    std::copy(scratch.begin(), scratch.end(), points.begin());
    return RigExecOracleResult::Blend;
}

RigExecMoverHandler
_MakeHandler()
{
    RigExecMoverHandler handler(
        "RigExecCurvenetAdjusterMover",
        &RigExecFixedMoverOp<RigExecRevisionOp::CurvenetAdjuster>,
        RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.bind = &_BindCurvenetAdjusterMover;
    handler.validate = &_ValidateCurvenetAdjusterMover;
    handler.oracle = &_OracleCurvenetAdjusterMover;
    return handler;
}

}  // namespace

RIGEXEC_REGISTER_MOVER(_MakeHandler());
} // namespace rigExec
