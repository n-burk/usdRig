#ifndef RIGEXEC_GRAPH_POSE_PROGRAM_H
#define RIGEXEC_GRAPH_POSE_PROGRAM_H
#include "rigExecGraph/poseArithmetic.h"
#include "rigExecMath/rbf.h"
#include "rigExecMath/pointFrame.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/token.h"
namespace rigExec {
using RigExecSpaceSwitchRecord=RigExecSpaceSwitchRecordT<GfVec3d>;
using RigExecSpaceSwitchInputs=RigExecSpaceSwitchInputsT<GfMatrix4d>;
using RigExecSpaceCheckpointInput=RigExecSpaceCheckpointInputT<GfMatrix4d>;
GfMatrix4d RigExecRunSpaceCheckpoint(const GfMatrix4d &,
    const std::vector<RigExecSpaceCheckpointInput> &);
using RigExecPoseInterpolatorRecord=RigExecPoseInterpolatorRecordT<RigExecRbfSolver>;
using RigExecPoseInterpolatorInputs=RigExecPoseInterpolatorInputsT<RigExecPointFrame>;
bool RigExecPreparePoseDelta(const RigExecPointFrame &before,const RigExecPointFrame &after,GfMatrix4d *delta);
RigExecPosePropagateOutcome RigExecPropagatePoseFrame(const RigExecPointFrame &current,
    const RigExecPointFrame &before,const RigExecPointFrame &after,const GfMatrix4d &delta,
    bool deltaOk,bool solverOutput,bool parentBlocked,bool candidatePresent,RigExecPointFrame *output);
bool RigExecPrepareRestDerivedIkChain(const std::vector<RigExecPointFrame> &current,
    const std::vector<RigExecPointFrame> &rest,std::vector<RigExecPointFrame> *prepared);
GfMatrix4d RigExecComposePoseAvars(double tx,double ty,double tz,double sx,double sy,double sz,
    double rx,double ry,double rz,double spin,const TfToken &order);
GfMatrix4d RigExecPoseRoundTrip(const GfMatrix4d &);
bool RigExecRunSpaceSwitch(const RigExecSpaceSwitchRecord &,const RigExecSpaceSwitchInputs &,
    RigExecPointFrame *);
RigExecPoseInterpolatorStatus RigExecRunPoseInterpolator(RigExecPoseInterpolatorRecord &,
    const RigExecPoseInterpolatorInputs &,std::vector<double> *);
}
#endif
