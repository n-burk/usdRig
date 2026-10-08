#ifndef RIGEXEC_GRAPH_SOLVER_PROGRAM_H
#define RIGEXEC_GRAPH_SOLVER_PROGRAM_H
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"
#include "rigExec/types.h"
namespace rigExec {
enum class RigExecSolverKind { FkChain, TwoBoneIk, BlendPointFrames, TwistDistribution, Ribbon, SplineIk };
/// Schema relationships that declare solver input prerequisites. A target
/// solver remains a structural dependency even when its numerical frame/curve
/// type is unavailable; scheduling comes only from the common op compiler.
inline std::vector<std::string> RigExecSolverRelationshipPorts(RigExecSolverKind kind)
{
    switch (kind) {
    case RigExecSolverKind::FkChain: return {"rigExec:controls", "rigExec:startFrame"};
    case RigExecSolverKind::TwoBoneIk:
        return {"rigExec:rootControl", "rigExec:effectorControl", "rigExec:poleControl", "rigExec:space"};
    case RigExecSolverKind::BlendPointFrames: return {"rigExec:inputA", "rigExec:inputB"};
    case RigExecSolverKind::TwistDistribution: return {"rigExec:start", "rigExec:end"};
    case RigExecSolverKind::Ribbon:
        return {"rigExec:driverCurve", "rigExec:startFrame", "rigExec:endFrame", "rigExec:twistFrames"};
    case RigExecSolverKind::SplineIk:
        return {"rigExec:rootControl", "rigExec:midControl", "rigExec:endControl", "rigExec:space"};
    }
    return {};
}
using RigExecSolverRest = std::array<GfVec3d,4>;
/// Detached structural/refresh data. Source bindings belong to the compiler,
/// never this record. All rest data is in rig-common row-vector space.
struct RigExecSolverRecord {
    RigExecSolverKind kind = RigExecSolverKind::FkChain;
    bool degenerate=false, parentRelative=false, hasStart=false;
    std::vector<RigExecSolverRest> controlRests, jointRests;
    std::vector<bool> restIsLive;
    RigExecSolverRest startRest{};
    std::array<RigExecSolverRest,3> ikRests{};
    RigExecTwoBoneIkParams ikParams;
    RigExecScaleBlend scaleMode=RigExecScaleBlend::Log;
    bool blendRotationRejected=false;
    RigExecSolverRest twistStartRest{},twistEndRest{};
    std::vector<double> twistWeights;
    std::vector<GfVec3f> ribbonRestPoints;
    RigExecSplineIkRest splineRest;
    RigExecSplineIkParams splineParams;
    std::vector<RigExecPointFrame> splineRestFrames;
    RigExecPointFrame splineRootRest,splineMidRest,splineEndRest;
    std::vector<double> splineRestWeights;
    RigExecSplineIkRestLength splineRestMode=RigExecSplineIkRestLength::Curve;
    std::vector<RigExecSolverRest> splineJointRests;
    size_t splineCount=0;
};
/// Current graph values, delivered after typed read-phase resolution. Null
/// aggregate pointers preserve BlendPointFrames unwired-input pass-through.
struct RigExecSolverInputs {
    std::vector<RigExecPointFrame> controls;
    RigExecPointFrame start,root,mid,end,pole;
    const RigExecPointFrameArray *blendA=nullptr,*blendB=nullptr;
    double blendWeight=0,twistTurns=0;
    std::vector<GfVec3f> ribbonPoints;
    int ribbonSampleCount=0;
    bool refreshIkParams=false;
    double bend=0,upperOffset=0,lowerOffset=0;
    float stretch=1,softness=0;
    GfMatrix4d ikSpace=GfMatrix4d(1.0);
    bool hasSpace=false;
    RigExecSolverRest spaceRest{};
    RigExecPointFrame space;
    bool refreshSplineParams=false;
    double preserveVolume=0,midFollowWeight=0,rollDegrees=0,twistDegrees=0,minLengthRatio=0;
};
struct RigExecSolverWorkspace {
    std::vector<RigExecFkChainElement> fkElements;
    std::vector<RigExecPointFrame> spacedFrames;
    RigExecSplineIkRest spacedRest;
    RigExecSplineIkResult splineResult;
};
/// One production numerical body for native and direct SceneDb graphs.
/// Clears outputs even for malformed/degenerate bindings.
bool RigExecRunSolver(const RigExecSolverRecord &,const RigExecSolverInputs &,
    RigExecSolverWorkspace *,RigExecPointFrameArray *,std::string *error=nullptr);
}
#endif
