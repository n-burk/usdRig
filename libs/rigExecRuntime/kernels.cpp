// rigExecRuntime shared kernels (M2 framework).
// Bit-identical ports of the point-frame kernels every family reads,
// from libs/rigExecMath/pointFrame.cpp, libs/rigExec/frameExtraction.h
// and libs/rigExec/solverKernels.cpp. Gf -> Rr, algorithm untouched.
#include "rigExecRuntime/store.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

namespace {

RrMat3d
_RrRowBasis(const std::array<RrVec3d, 4> &pts)
{
    return RrMat3d(
        pts[1][0] - pts[0][0], pts[1][1] - pts[0][1], pts[1][2] - pts[0][2],
        pts[2][0] - pts[0][0], pts[2][1] - pts[0][1], pts[2][2] - pts[0][2],
        pts[3][0] - pts[0][0], pts[3][1] - pts[0][1], pts[3][2] - pts[0][2]);
}

RrVec3d
_RrTransformDir(const RrVec3d &v, const RrMat3d &m)
{
    return v * m;
}

}  // namespace

RrPointFrame
RrFrameFromMatrix(const RrMat4d &m)
{
    RrPointFrame frame;
    const std::array<RrVec3d, 4> &identity = RrIdentityLandmarks();
    for (size_t i = 0; i < 4; ++i) {
        frame.points[i] = m.TransformAffine(identity[i]);
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(frame.points[i][a])) {
                frame.flags = RrPointFrameDegenerate;
                return frame;
            }
        }
    }
    frame.flags = RrPointFrameValid;
    return frame;
}

double
RrFrameEpsilon(const std::array<RrVec3d, 4> &restPoints)
{
    const double lx = (restPoints[1] - restPoints[0]).GetLength();
    const double ly = (restPoints[2] - restPoints[0]).GetLength();
    const double lz = (restPoints[3] - restPoints[0]).GetLength();
    return 1e-10 * std::max({1.0, lx, ly, lz});
}

bool
RrPointsToMatrix(const std::array<RrVec3d, 4> &restPoints,
                 const std::array<RrVec3d, 4> &posePoints,
                 RrMat4d *matrix)
{
    const RrMat3d mq = _RrRowBasis(restPoints);
    const RrMat3d mp = _RrRowBasis(posePoints);

    const double det = mq.GetDeterminant();
    const double eps = RrFrameEpsilon(restPoints);
    if (std::abs(det) < eps * eps * eps) {
        if (matrix) {
            matrix->SetIdentity();
        }
        return false;
    }

    const RrMat3d m3 = mq.GetInverse() * mp;
    const RrVec3d t = posePoints[0] - _RrTransformDir(restPoints[0], m3);

    if (matrix) {
        *matrix = RrMat4d(
            m3[0][0], m3[0][1], m3[0][2], 0.0, m3[1][0], m3[1][1], m3[1][2],
            0.0, m3[2][0], m3[2][1], m3[2][2], 0.0, t[0], t[1], t[2], 1.0);
    }
    return true;
}

bool
RrPointsToMatrix(const std::array<RrVec3d, 4> &restPoints,
                 const RrPointFrame &frame, RrMat4d *matrix)
{
    return RrPointsToMatrix(restPoints, frame.points, matrix);
}

RrPointFrame
RrMatrixToPoints(const std::array<RrVec3d, 4> &restPoints,
                 const RrMat4d &matrix)
{
    RrPointFrame frame;
    for (size_t i = 0; i < 4; ++i) {
        frame.points[i] = matrix.TransformAffine(restPoints[i]);
    }
    frame.flags = RrPointFrameValid;

    const RrMat3d posed = _RrRowBasis(frame.points);
    if (posed.GetDeterminant() < 0) {
        frame.flags |= RrPointFrameReflected;
    }
    const double eps = RrFrameEpsilon(restPoints);
    if (std::abs(posed.GetDeterminant()) <= eps * eps * eps) {
        frame.flags |= RrPointFrameDegenerate;
    }
    return frame;
}

bool
RrFrameRotation(const RrPointFrame &frame, RrQuatd *out)
{
    RrMat4d matrix;
    matrix.SetIdentity();
    if (!frame.IsValid() || frame.IsDegenerate() ||
        !RrPointsToMatrix(RrIdentityLandmarks(), frame.points, &matrix)) {
        return false;
    }
    matrix.SetTranslateOnly(RrVec3d(0.0));
    *out = matrix.GetOrthonormalized(false).ExtractRotationQuat();
    return true;
}

RrMat4d
RrElementOutSpace(const RrPointFrameArray *source, size_t index)
{
    RrMat4d m;
    m.SetIdentity();
    if (!source || index >= source->frames.size() ||
        !source->frames[index].IsValid()) {
        return m;
    }
    const std::array<RrVec3d, 4> &posed = source->frames[index].points;
    const RrVec3d origin = posed[0];
    const bool hasRest = index < source->rests.size();
    RrMat4d rm;
    rm.SetIdentity();
    for (int a = 0; a < 3; ++a) {
        RrVec3d direction = posed[size_t(a) + 1] - origin;
        const double posedLength = direction.GetLength();
        if (posedLength < 1e-12) {
            return m;
        }
        direction /= posedLength;
        double ratio = 1.0;
        if (hasRest) {
            const double restLength =
                (source->rests[index][size_t(a) + 1] - source->rests[index][0])
                    .GetLength();
            if (restLength > 1e-12) {
                ratio = posedLength / restLength;
            }
        }
        rm.SetRow(a, RrVec4d(direction[0] * ratio, direction[1] * ratio,
                             direction[2] * ratio, 0));
    }
    rm.SetRow(3, RrVec4d(origin[0], origin[1], origin[2], 1));
    return rm;
}

RrPointFrame
RrExtractElementFrame(const RrPointFrameArray *source, size_t index)
{
    if (source && index < source->frames.size()) {
        const RrPointFrame &f = source->frames[index];
        if (f.IsValid() && !f.IsDegenerate()) {
            return RrFrameFromMatrix(RrElementOutSpace(source, index));
        }
    }
    RrPointFrame degenerate;
    degenerate.flags = RrPointFrameDegenerate;
    return degenerate;
}

RrMat4d
RrRoundTrip(const RrMat4d &m)
{
    RrMat4d out;
    out.SetIdentity();
    RrPointsToMatrix(RrIdentityLandmarks(), RrFrameFromMatrix(m).points,
                     &out);
    return out;
}

namespace {

bool
_RrFrameMatrix(const RrPointFrame &frame, RrMat4d *out)
{
    return frame.IsValid() && !frame.IsDegenerate() &&
           RrPointsToMatrix(RrIdentityLandmarks(), frame.points, out);
}

// Translation, rotation and scale of a transform, taken the way
// GfTransform takes them so that the decomposition and the recomposition
// below are each other's inverse by construction -- and so that the
// runtime's answer is the program's answer to the last bit.
struct _RrDecomposed {
    RrVec3d translation = RrVec3d(0.0);
    RrQuatd rotation = RrQuatd::GetIdentity();
    RrVec3d scale = RrVec3d(1.0);
};

_RrDecomposed
_RrDecompose(const RrMat4d &m)
{
    // GfTransform::SetMatrix with a zero pivot: Factor, then the rotation
    // out of the rotation factor. The scale orientation GfTransform would
    // keep is dropped here exactly as _Recompose drops it, because
    // _Recompose sets only scale, rotation and translation.
    RrMat4d shearRot, rotMat, proj;
    _RrDecomposed out;
    m.Factor(&shearRot, &out.scale, &rotMat, &out.translation, &proj);
    out.rotation = rotMat.ExtractRotation().GetQuat();
    return out;
}

RrMat4d
_RrRecompose(const _RrDecomposed &d)
{
    // GfTransform::GetMatrix with no pivot and no scale orientation:
    // scale, then rotate, then translate, each skipped when it is the
    // identity -- which is what keeps an unscaled, unrotated transform
    // bit-identical to its own translation.
    const RrRotation rotation(d.rotation);
    const bool doScale = d.scale != RrVec3d(1.0, 1.0, 1.0);
    const bool doRotation = rotation.GetAngle() != 0.0;
    const bool doTranslation = d.translation != RrVec3d(0.0);
    bool anySet = false;
    RrMat4d mtx;
    RrMat4d tmp;
    if (doScale) {
        mtx.SetScale(d.scale);
        anySet = true;
    }
    if (doRotation) {
        if (anySet) {
            tmp.SetRotate(rotation);
            mtx = mtx * tmp;
        } else {
            mtx.SetRotate(rotation);
            anySet = true;
        }
    }
    if (doTranslation) {
        if (anySet) {
            tmp.SetTranslate(d.translation);
            mtx = mtx * tmp;
        } else {
            mtx.SetTranslate(d.translation);
            anySet = true;
        }
    }
    if (!anySet) {
        mtx.SetIdentity();
    }
    return mtx;
}

}  // namespace

bool
RrFrameTranslation(const RrPointFrame &driverFinal,
                   const RrPointFrame &driverRest,
                   const RrPointFrame *parentFinal,
                   const RrPointFrame *parentRest, RrVec3d *out)
{
    RrMat4d world(1.0), restWorld(1.0);
    if (!_RrFrameMatrix(driverFinal, &world) ||
        !_RrFrameMatrix(driverRest, &restWorld)) {
        return false;
    }
    RrMat4d local = world, restLocal = restWorld;
    double det = 0.0;
    if (parentFinal && parentRest) {
        RrMat4d parent(1.0), restParent(1.0);
        if (!_RrFrameMatrix(*parentFinal, &parent) ||
            !_RrFrameMatrix(*parentRest, &restParent)) {
            return false;
        }
        const RrMat4d parentInverse = parent.GetInverse(&det);
        if (det == 0.0) {
            return false;
        }
        const RrMat4d restParentInverse = restParent.GetInverse(&det);
        if (det == 0.0) {
            return false;
        }
        local = world * parentInverse;
        restLocal = restWorld * restParentInverse;
    }
    const RrMat4d restLocalInverse = restLocal.GetInverse(&det);
    if (det == 0.0) {
        return false;
    }
    *out = restLocalInverse.Transform(local.ExtractTranslation());
    return true;
}

RrMat4d
RrBlendTransforms(const RrMat4d &a, const RrMat4d &b, double weight)
{
    if (!(weight > 0.0)) {
        return a;
    }
    if (weight >= 1.0) {
        return b;
    }
    const _RrDecomposed da = _RrDecompose(a), db = _RrDecompose(b);
    _RrDecomposed out;
    // GfLerp's own expression, not the algebraically equal
    // a + w * (b - a): the parity mode compares these bit for bit.
    out.translation = (1 - weight) * da.translation + weight * db.translation;
    out.scale = (1 - weight) * da.scale + weight * db.scale;
    RrQuatd target = db.rotation;
    const double cosArc = da.rotation.GetReal() * target.GetReal() +
                          RrDot(da.rotation.GetImaginary(),
                                target.GetImaginary());
    if (cosArc < 0.0) {
        target = RrQuatd(-target.GetReal(), -target.GetImaginary());
    }
    out.rotation = RrSlerp(weight, da.rotation, target).GetNormalized();
    return _RrRecompose(out);
}

RrMat4d
RrOrientSpaceDelta(const RrMat4d &delta, const RrMat4d &local,
                   const RrMat4d &localInverse, const RrMat4d &unswitched)
{
    RrMat4d frame = delta * local;
    frame[3][0] = unswitched[3][0];
    frame[3][1] = unswitched[3][1];
    frame[3][2] = unswitched[3][2];
    return frame * localInverse;
}

RrMat4d
RrFilterSpaceRotation(const RrMat4d &m, const RrVec3d &axis,
                      RrRotationFilter filter)
{
    if (filter == RrRotationFilter::All ||
        filter == RrRotationFilter::Orient) {
        return m;
    }
    const double length = axis.GetLength();
    if (length < 1e-12) {
        return m;
    }
    const RrVec3d a = axis / length;
    _RrDecomposed d = _RrDecompose(m);
    const RrQuatd q = d.rotation.GetNormalized();
    const double along = RrDot(q.GetImaginary(), a);
    RrQuatd twist(q.GetReal(), a * along);
    const double norm = std::sqrt(twist.GetReal() * twist.GetReal() +
                                  twist.GetImaginary().GetLengthSq());
    if (norm < 1e-12) {
        twist = RrQuatd::GetIdentity();
    } else {
        twist = RrQuatd(twist.GetReal() / norm,
                        twist.GetImaginary() / norm);
    }
    d.rotation = filter == RrRotationFilter::Twist
                     ? twist
                     : (q * twist.GetInverse()).GetNormalized();
    return _RrRecompose(d);
}

RrMat4d
RrMaskTransform(const RrMat4d &m, const bool translation[3],
                const bool rotation[3], const bool scale[3])
{
    const bool all = translation[0] && translation[1] && translation[2] &&
                     rotation[0] && rotation[1] && rotation[2] &&
                     scale[0] && scale[1] && scale[2];
    if (all) {
        return m;
    }
    _RrDecomposed d = _RrDecompose(m);
    for (int axis = 0; axis < 3; ++axis) {
        if (!translation[axis]) d.translation[axis] = 0.0;
        if (!scale[axis]) d.scale[axis] = 1.0;
    }
    if (!rotation[0] || !rotation[1] || !rotation[2]) {
        // The same XYZ row-vector extraction the program does, so a mask
        // agrees with the avar compose at every angle and not merely at
        // small ones.
        const RrMat4d r(RrRotation(d.rotation), RrVec3d(0.0));
        double x = 0.0, y = 0.0, z = 0.0;
        const double sinY = RrClamp(r._mtx[0][2], -1.0, 1.0);
        y = std::asin(sinY);
        if (std::abs(sinY) < 1.0 - 1e-9) {
            x = std::atan2(r._mtx[1][2], r._mtx[2][2]);
            z = std::atan2(r._mtx[0][1], r._mtx[0][0]);
        } else {
            x = std::atan2(-r._mtx[2][1], r._mtx[1][1]);
            z = 0.0;
        }
        static const RrVec3d axes[3] = {RrVec3d(1, 0, 0), RrVec3d(0, 1, 0),
                                        RrVec3d(0, 0, 1)};
        const double angles[3] = {RrRadiansToDegrees(x),
                                  RrRadiansToDegrees(y),
                                  RrRadiansToDegrees(z)};
        RrMat4d composed(1.0);
        for (int axis = 0; axis < 3; ++axis) {
            if (!rotation[axis] || angles[axis] == 0.0) continue;
            composed = composed *
                RrMat4d(RrRotation(axes[axis], angles[axis]), RrVec3d(0.0));
        }
        d.rotation = composed.ExtractRotationQuat();
    }
    return _RrRecompose(d);
}

const RigExecWireInput &
RrProgram::LadderInput(size_t slot, int field) const
{
    const RigExecWireLadder &ladder = poses->ladders[slot];
    switch (field) {
    case RrLadderRestSpace:
        return ladder.restSpace;
    case RrLadderDefaultSpace:
        return ladder.defaultSpace;
    case RrLadderPosedSpace:
        return ladder.posedSpace;
    default:
        break;
    }
    if (field >= RrLadderRestAvar0 && field < RrLadderRestAvar0 + 6) {
        return ladder.restAvars[field - RrLadderRestAvar0];
    }
    if (field >= RrLadderDefaultAvar0 && field < RrLadderDefaultAvar0 + 6) {
        return ladder.defaultAvars[field - RrLadderDefaultAvar0];
    }
    return ladder.rotationOrder;
}

const RigExecWireInput &
RrProgram::SolverInput(size_t solver, int field) const
{
    const RigExecWireSolver &s = poses->solvers[solver];
    switch (field) {
    case RrSolverBend:
        return s.bend;
    case RrSolverUpperOffset:
        return s.upperOffset;
    case RrSolverLowerOffset:
        return s.lowerOffset;
    case RrSolverStretch:
        return s.stretch;
    case RrSolverSoftness:
        return s.softness;
    case RrSolverBlendWeight:
        return s.blendWeight;
    case RrSolverPreserveVolume:
        return s.preserveVolume;
    case RrSolverMidFollowWeight:
        return s.midFollowWeight;
    case RrSolverRoll:
        return s.roll;
    case RrSolverTwist:
        return s.twist;
    case RrSolverMinLengthRatio:
        return s.minLengthRatio;
    case RrSolverTwistTurns:
        return s.twistTurns;
    default:
        break;
    }
    return s.ribbonSampleCount;
}

const RigExecWireInput &
RrProgram::ConstraintInput(size_t constraint, int field) const
{
    const RigExecWireConstraint &c = poses->constraints[constraint];
    switch (field) {
    case RrConstraintEnabled:
        return c.enabled;
    case RrConstraintDefaultWeight:
        return c.defaultWeight;
    case RrConstraintOffset:
        return c.offset;
    case RrConstraintAffectX:
        return c.affectX;
    case RrConstraintAffectY:
        return c.affectY;
    case RrConstraintAffectZ:
        return c.affectZ;
    case RrConstraintTX:
        return c.tX;
    case RrConstraintTY:
        return c.tY;
    case RrConstraintTZ:
        return c.tZ;
    case RrConstraintRX:
        return c.rX;
    case RrConstraintRY:
        return c.rY;
    case RrConstraintRZ:
        return c.rZ;
    case RrConstraintSX:
        return c.sX;
    case RrConstraintSY:
        return c.sY;
    case RrConstraintSZ:
        return c.sZ;
    case RrConstraintAimVector:
        return c.aimVector;
    case RrConstraintUpVector:
        return c.upVector;
    case RrConstraintRotationOffset:
        return c.rotationOffset;
    case RrConstraintWorldUpVector:
        return c.worldUpVector;
    case RrConstraintPoleVector:
        return c.poleVector;
    default:
        break;
    }
    return c.twistDegrees;
}

const RigExecWireInput &
RrProgram::WeightInput(size_t object, int field) const
{
    const RigExecWireWeightObject &w = geometry->weightObjects[object];
    switch (field) {
    case RrWeightDefaultWeight:
        return w.defaultWeight;
    case RrWeightDriver:
        return w.driver;
    case RrWeightScale:
        return w.scale;
    case RrWeightBias:
        return w.bias;
    case RrWeightStrength:
        return w.strength;
    case RrWeightInvert:
        return w.invert;
    case RrWeightFalloffMin:
        return w.falloffMin;
    case RrWeightFalloffMax:
        return w.falloffMax;
    case RrWeightScaleXPos:
        return w.scaleXPos;
    case RrWeightScaleYPos:
        return w.scaleYPos;
    case RrWeightScaleZPos:
        return w.scaleZPos;
    case RrWeightScaleXNeg:
        return w.scaleXNeg;
    case RrWeightScaleYNeg:
        return w.scaleYNeg;
    case RrWeightScaleZNeg:
        return w.scaleZNeg;
    case RrWeightScaleX:
        return w.scaleX;
    case RrWeightScaleY:
        return w.scaleY;
    case RrWeightScaleZ:
        return w.scaleZ;
    case RrWeightExtentU:
        return w.extentU;
    case RrWeightExtentV:
        return w.extentV;
    case RrWeightCurvenetSamples:
        return w.curvenetSamples;
    default:
        break;
    }
    return w.curvenetUnreached;
}

}  // namespace rigExec
