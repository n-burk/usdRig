#include "poseProgram.h"
#include "constraintProgram.h"
#include "providerArithmetic.h"
#include "pxr/base/gf/rotation.h"
#include <cmath>
#include <limits>
#include "rigExec/solverKernels.h"
#include "rigExec/frameExtraction.h"
namespace rigExec {
namespace {
struct PoseMath {
    using Matrix=GfMatrix4d;using Vector=GfVec3d;using Quaternion=GfQuatd;using Rotation=GfRotation;
    static bool Usable(const RigExecPointFrame &frame) {return RigExecConstraintFrameUsable(frame);}
    static RigExecPointFrame Carry(const RigExecPointFrame &frame,const Matrix &delta) {
        return RigExecMatrixToPoints(frame.points,delta);
    }
    static Matrix RoundTrip(const Matrix &matrix) {
        Matrix result(1.0);
        RigExecPointsToMatrix(RigExecIdentityLandmarks(),RigExecFrameFromMatrix(matrix).points,&result);
        return result;
    }
    static Matrix RoundTripCheckpoint(const Matrix &matrix) {
        const auto frame=RigExecFrameFromMatrix(matrix);
        Matrix result(1.0);
        if(!frame.IsValid() || frame.IsDegenerate() ||
           !RigExecPointsToMatrix(RigExecIdentityLandmarks(),frame.points,&result)) {
            result=Matrix(1.0);
            result[3][0]=std::numeric_limits<double>::quiet_NaN();
        }
        return result;
    }
    static Matrix Filter(const Matrix &matrix,const Vector &axis,int filter) {
        return RigExecFilterSpaceRotation(matrix,axis,RigExecRotationFilter(filter));
    }
    static Matrix Blend(const Matrix &a,const Matrix &b,double weight) {
        return RigExecBlendTransforms(a,b,weight);
    }
    static Matrix Mask(const Matrix &matrix,const bool *t,const bool *r,const bool *s) {
        return RigExecMaskTransform(matrix,t,r,s);
    }
    static bool FrameRotation(const RigExecPointFrame &frame,Quaternion *out) {
        return RigExecFrameRotation(frame,out);
    }
    static bool FrameTranslation(const RigExecPointFrame &frame,const RigExecPointFrame &rest,
        const RigExecPointFrame *parent,const RigExecPointFrame *parentRest,Vector *out) {
        return RigExecFrameTranslation(frame,rest,parent,parentRest,out);
    }
    static Vector EulerFromQuaternion(const Quaternion &q) {return RigExecRbfEulerFromQuaternion(q);}
};
}
bool RigExecPreparePoseDelta(const RigExecPointFrame &before,const RigExecPointFrame &after,GfMatrix4d *delta) {
    return RigExecPointsToMatrix(before.points,after.points,delta);
}
RigExecPosePropagateOutcome RigExecPropagatePoseFrame(const RigExecPointFrame &current,
    const RigExecPointFrame &before,const RigExecPointFrame &after,const GfMatrix4d &delta,
    bool deltaOk,bool solverOutput,bool parentBlocked,bool candidatePresent,RigExecPointFrame *output) {
    return RigExecPropagatePoseArithmetic<PoseMath>(current,before,after,delta,deltaOk,
        solverOutput,parentBlocked,candidatePresent,output);
}
GfMatrix4d RigExecRunSpaceCheckpoint(const GfMatrix4d &ancestor,
    const std::vector<RigExecSpaceCheckpointInput> &inputs) {
    return RigExecRunSpaceCheckpointArithmetic<PoseMath>(ancestor,inputs);
}
GfMatrix4d RigExecComposePoseAvars(double tx,double ty,double tz,double sx,double sy,double sz,
    double rx,double ry,double rz,double spin,const TfToken &order) {
    // The token's text as a field read: GetString() on an empty token reads a
    // function-local static.
    return RigExecProviderComposeAvars<PoseMath>(tx,ty,tz,sx,sy,sz,rx,ry,rz,spin,
        std::string_view(order.GetText(),order.size()));
}
GfMatrix4d RigExecPoseRoundTrip(const GfMatrix4d &matrix) {return PoseMath::RoundTrip(matrix);}
bool RigExecRunSpaceSwitch(const RigExecSpaceSwitchRecord &record,
    const RigExecSpaceSwitchInputs &input,RigExecPointFrame *output) {
    GfMatrix4d matrix(1.0);
    if(!output || !RigExecRunSpaceSwitchArithmetic<PoseMath>(record,input,&matrix))return false;
    *output=RigExecFrameFromMatrix(matrix);return true;
}
RigExecPoseInterpolatorStatus RigExecRunPoseInterpolator(RigExecPoseInterpolatorRecord &record,
    const RigExecPoseInterpolatorInputs &input,std::vector<double> *output) {
    return RigExecRunPoseInterpolatorArithmetic<PoseMath>(record,input,output);
}
// `neverTS` retains current rotations/root placement while rebuilding
// child placement and handle lengths from rest frames. A joint without
// any authored rest transform has the schema's identity fallback, which
// is not an actual chain rest layout; use its current static layout in
// that case. The public math solver can therefore keep measuring its
// input chain; evaluator-side preparation decides whether those
// measurements are rest- or animation-derived.
bool
RigExecPrepareRestDerivedIkChain(
    const std::vector<RigExecPointFrame> &current,
    const std::vector<RigExecPointFrame> &rest,
    std::vector<RigExecPointFrame> *prepared)
{
    if (current.size() != rest.size() || current.empty()) {
        return false;
    }
    bool usableRestLayout = true;
    for (size_t i = 1; i < rest.size(); ++i) {
        const double segmentLength =
            (rest[i].Origin() - rest[i - 1].Origin()).GetLength();
        if (!std::isfinite(segmentLength) || segmentLength <= 0.0) {
            usableRestLayout = false;
            break;
        }
    }
    const std::vector<RigExecPointFrame> &lengthReference =
        usableRestLayout ? rest : current;
    prepared->clear();
    prepared->reserve(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
        if (!RigExecConstraintFrameUsable(current[i]) ||
            !RigExecConstraintFrameUsable(rest[i])) {
            return false;
        }
        GfVec3d origin = current[i].Origin();
        if (i > 0) {
            GfMatrix4d parentRest(1.0), parentPrepared(1.0);
            if (!RigExecPointsToMatrix(
                    RigExecIdentityLandmarks(),
                    lengthReference[i - 1].points,
                    &parentRest) ||
                !RigExecPointsToMatrix(
                    RigExecIdentityLandmarks(), prepared->back().points,
                    &parentPrepared)) {
                return false;
            }
            origin = parentPrepared.TransformAffine(
                parentRest.GetInverse().TransformAffine(
                    lengthReference[i].Origin()));
        }

        RigExecPointFrame frame = current[i];
        frame.points[0] = origin;
        for (size_t axis = 1; axis < frame.points.size(); ++axis) {
            GfVec3d direction =
                current[i].points[axis] - current[i].Origin();
            const double directionLength = direction.GetLength();
            const double length =
                (lengthReference[i].points[axis] -
                 lengthReference[i].Origin()).GetLength();
            if (!std::isfinite(length) || length <= 0.0 ||
                !std::isfinite(directionLength) ||
                directionLength <= 0.0) {
                return false;
            }
            direction /= directionLength;
            frame.points[axis] = origin + direction * length;
        }
        if (!RigExecConstraintFrameUsable(frame)) {
            return false;
        }
        prepared->push_back(frame);
    }
    return true;
}

}
