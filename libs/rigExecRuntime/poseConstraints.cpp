// Runtime constraints, source resolution, and single-chain IK.

#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// FBX constraint kernels (solvers.cpp).

enum class _RrEulerOrder : uint8_t {
    XYZ = 0,
    XZY,
    YXZ,
    YZX,
    ZXY,
    ZYX,
};

struct _RrConstraintAxisMask {
    bool x = true;
    bool y = true;
    bool z = true;
};

struct _RrConstraintSource {
    RrPointFrame frame;
    double normalizedWeight = 0.0;
    RrVec3d translationOffset{0.0};
    RrVec3d rotationOffsetDegrees{0.0};
};

struct _RrPositionConstraintParams {
    double weight = 1.0;
    RrVec3d offset{0.0};
    _RrConstraintAxisMask affect;
};

struct _RrRotationConstraintParams {
    double weight = 1.0;
    RrVec3d offsetDegrees{0.0};
    _RrConstraintAxisMask affect;
    _RrEulerOrder rotationOrder = _RrEulerOrder::XYZ;
};

struct _RrScaleConstraintParams {
    double weight = 1.0;
    RrVec3d offset{1.0, 1.0, 1.0};
    _RrConstraintAxisMask affect;
};

struct _RrParentConstraintParams {
    double weight = 1.0;
    _RrConstraintAxisMask translationAxes;
    _RrConstraintAxisMask rotationAxes;
    _RrConstraintAxisMask scaleAxes;
    _RrEulerOrder rotationOrder = _RrEulerOrder::XYZ;
};

struct _RrAimConstraintParams {
    double weight = 1.0;
    RrVec3d localAimVector{1.0, 0.0, 0.0};
    RrVec3d localUpVector{0.0, 1.0, 0.0};
    RrVec3d rotationOffsetDegrees{0.0};
    _RrConstraintAxisMask affectRotation;
    _RrEulerOrder rotationOrder = _RrEulerOrder::XYZ;
    const RrVec3d *worldUpDirection = nullptr;
    bool preserveInputUp = false;
};

bool
_RrConstraintFrameUsable(const RrPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const RrVec3d &point : frame.points) {
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2])) {
            return false;
        }
    }
    return true;
}

bool
_RrConstraintVecFinite(const RrVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}

RrPointFrame
_RrConstraintFailure(const RrPointFrame &input)
{
    RrPointFrame failed = input;
    failed.flags |= RrPointFrameDegenerate;
    return failed;
}

bool _RrHasFinitePoints(const RrPointFrame &frame);

bool
_RrDecomposeConstraintFrame(const RrPointFrame &frame,
                            _RrTransformParams *params)
{
    if (!_RrConstraintFrameUsable(frame) ||
        !_RrPointsToParams(RrIdentityLandmarks(), frame.points, 2,
                           params)) {
        return false;
    }
    return _RrConstraintVecFinite(params->translation) &&
           _RrConstraintVecFinite(params->scale) &&
           _RrConstraintVecFinite(params->shear) &&
           std::isfinite(params->rotation.GetReal()) &&
           _RrConstraintVecFinite(params->rotation.GetImaginary());
}

RrPointFrame
_RrFrameFromConstraintParams(const RrPointFrame &input,
                             const _RrTransformParams &params)
{
    RrPointFrame result = RrMatrixToPoints(
        RrIdentityLandmarks(), _RrParamsToMatrix(params));
    if (!_RrHasFinitePoints(result)) {
        return _RrConstraintFailure(input);
    }
    // Affine is descriptive rather than required for reconstruction, but an
    // input explicitly classified affine remains classified affine when its
    // shear is carried through the constraint.
    result.flags |= input.flags & RrPointFrameAffine;
    return result;
}

bool
_RrValidateGlobalWeight(double weight, double *clamped)
{
    if (!std::isfinite(weight)) {
        return false;
    }
    *clamped = std::min(std::max(weight, 0.0), 1.0);
    return true;
}

bool
_RrValidateSourceWeight(const _RrConstraintSource &source)
{
    return std::isfinite(source.normalizedWeight) &&
           source.normalizedWeight >= 0.0;
}

bool
_RrAffects(const _RrConstraintAxisMask &mask, int axis)
{
    return axis == 0 ? mask.x : axis == 1 ? mask.y : mask.z;
}

bool
_RrIsValidOrder(_RrEulerOrder order)
{
    const int raw = int(order);
    return raw >= 0 && raw <= 5;
}

std::array<int, 3>
_RrOrderIndices(_RrEulerOrder order)
{
    switch (order) {
    case _RrEulerOrder::XYZ:
        return {0, 1, 2};
    case _RrEulerOrder::XZY:
        return {0, 2, 1};
    case _RrEulerOrder::YXZ:
        return {1, 0, 2};
    case _RrEulerOrder::YZX:
        return {1, 2, 0};
    case _RrEulerOrder::ZXY:
        return {2, 0, 1};
    case _RrEulerOrder::ZYX:
        return {2, 1, 0};
    }
    return {0, 1, 2};
}

RrQuatd
_RrQuatFromEulerDegrees(const RrVec3d &degrees, _RrEulerOrder order)
{
    static const RrVec3d axes[3] = {
        RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
    RrMat4d matrix = _RrIdentity();
    const std::array<int, 3> indices = _RrOrderIndices(order);
    for (const int axis : indices) {
        matrix = matrix *
                 RrMat4d(RrRotation(axes[axis], degrees[axis]),
                         RrVec3d(0.0));
    }
    return matrix.ExtractRotation().GetQuat().GetNormalized();
}

RrVec3d
_RrEulerDegreesFromQuat(const RrQuatd &rotation, _RrEulerOrder order)
{
    static const RrVec3d axes[3] = {
        RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
    const std::array<int, 3> indices = _RrOrderIndices(order);
    // GfRotation::Decompose(a,b,c) describes row-matrix factors in the
    // reverse order Rc * Rb * Ra.  Pass the authored application sequence
    // reversed so returned components invert _QuatFromEulerDegrees exactly.
    const RrVec3d ordered = RrRotation(rotation).Decompose(
        axes[indices[2]], axes[indices[1]], axes[indices[0]]);
    RrVec3d result(0.0);
    result[indices[2]] = ordered[0];
    result[indices[1]] = ordered[1];
    result[indices[0]] = ordered[2];
    return result;
}

double
_RrShortestDegrees(double degrees)
{
    double wrapped = std::fmod(degrees + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    wrapped -= 180.0;
    // Resolve the exact half-turn tie without depending on fmod's sign.
    return (wrapped == -180.0 && degrees > 0.0) ? 180.0 : wrapped;
}

RrVec3d
_RrApplyEulerDelta(const RrVec3d &inputEuler,
                   const RrVec3d &targetEuler,
                   const _RrConstraintAxisMask &affect, double weight)
{
    RrVec3d output = inputEuler;
    for (int axis = 0; axis < 3; ++axis) {
        if (_RrAffects(affect, axis)) {
            // The target Euler representation is the full-strength
            // candidate.  Select it directly at the endpoint: reconstructing
            // the equivalent input + shortestDelta representation can differ
            // bit-for-bit (for example, 190 versus -170 degrees).
            if (weight >= 1.0) {
                output[axis] = targetEuler[axis];
            } else {
                output[axis] += weight * _RrShortestDegrees(
                    targetEuler[axis] - inputEuler[axis]);
            }
        }
    }
    return output;
}

bool
_RrHasFinitePoints(const RrPointFrame &frame)
{
    for (const RrVec3d &point : frame.points) {
        if (!_RrConstraintVecFinite(point)) {
            return false;
        }
    }
    return true;
}

bool
_RrNormalizeDirection(RrVec3d *direction, double eps = 1e-12)
{
    if (!_RrConstraintVecFinite(*direction)) {
        return false;
    }
    const double largest = std::max(
        {std::abs((*direction)[0]), std::abs((*direction)[1]),
         std::abs((*direction)[2])});
    if (largest < eps) {
        return false;
    }
    *direction /= largest;
    const double length = direction->GetLength();
    if (!std::isfinite(length) || length < eps) {
        return false;
    }
    *direction /= length;
    return _RrConstraintVecFinite(*direction);
}

// GfRotation::RotateOntoProjected (rotation.cpp): the twist about
// `axisParam` taking v1's projection onto v2's.
RrRotation
_RrRotateOntoProjected(const RrVec3d &v1, const RrVec3d &v2,
                       const RrVec3d &axisParam)
{
    RrVec3d axis = axisParam.GetNormalized();

    RrVec3d v1Proj = v1 - RrDot(v1, axis) * axis;
    RrVec3d v2Proj = v2 - RrDot(v2, axis) * axis;
    v1Proj.Normalize();
    v2Proj.Normalize();
    RrVec3d crossAxis = RrCross(v1Proj, v2Proj);
    double sinTheta = RrDot(crossAxis, axis);
    double cosTheta = RrDot(v1Proj, v2Proj);
    double theta = 0;
    if (!(std::fabs(sinTheta) < 1e-6 && std::fabs(cosTheta) < 1e-6)) {
        theta = std::atan2(sinTheta, cosTheta);
    }

    const double toDeg = (180.0) / std::acos(-1.0);
    return RrRotation(axis, theta * toDeg);
}

RrPointFrame
_RrApplyPositionConstraint(
    const RrPointFrame &input,
    const std::vector<_RrConstraintSource> &sources,
    const _RrPositionConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_RrConstraintFrameUsable(input) ||
        !_RrConstraintVecFinite(params.offset)) {
        return _RrConstraintFailure(input);
    }

    RrVec3d target(0.0);
    double totalWeight = 0.0;
    for (const _RrConstraintSource &source : sources) {
        if (!_RrValidateSourceWeight(source)) {
            return _RrConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!_RrConstraintFrameUsable(source.frame)) {
            return _RrConstraintFailure(input);
        }
        target += source.frame.points[0] * source.normalizedWeight;
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input
                                  : _RrConstraintFailure(input);
    }
    target = target / totalWeight + params.offset;
    if (!_RrConstraintVecFinite(target)) {
        return _RrConstraintFailure(input);
    }

    RrVec3d constrainedOrigin = input.points[0];
    for (int axis = 0; axis < 3; ++axis) {
        if (_RrAffects(params.affect, axis)) {
            constrainedOrigin[axis] = _RrBlendEnvelope(
                input.points[0][axis], target[axis], globalWeight);
        }
    }
    if (!_RrConstraintVecFinite(constrainedOrigin)) {
        return _RrConstraintFailure(input);
    }
    RrPointFrame output = input;
    if (globalWeight >= 1.0) {
        for (size_t point = 1; point < output.points.size(); ++point) {
            output.points[point] =
                constrainedOrigin +
                (input.points[point] - input.points[0]);
        }
        output.points[0] = constrainedOrigin;
    } else {
        const RrVec3d translation =
            constrainedOrigin - input.points[0];
        for (RrVec3d &point : output.points) {
            point += translation;
        }
    }
    return output;
}

RrPointFrame
_RrApplyRotationConstraint(
    const RrPointFrame &input,
    const std::vector<_RrConstraintSource> &sources,
    const _RrRotationConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_RrIsValidOrder(params.rotationOrder) ||
        !_RrConstraintVecFinite(params.offsetDegrees)) {
        return _RrConstraintFailure(input);
    }

    _RrTransformParams inputParams;
    if (!_RrDecomposeConstraintFrame(input, &inputParams)) {
        return _RrConstraintFailure(input);
    }
    const RrVec3d inputEuler =
        _RrEulerDegreesFromQuat(inputParams.rotation,
                                params.rotationOrder);
    if (!_RrConstraintVecFinite(inputEuler)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d sourceAnchor(0.0), weightedDelta(0.0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const _RrConstraintSource &source : sources) {
        if (!_RrValidateSourceWeight(source)) {
            return _RrConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        _RrTransformParams sourceParams;
        if (!_RrDecomposeConstraintFrame(source.frame, &sourceParams)) {
            return _RrConstraintFailure(input);
        }
        const RrVec3d sourceEuler = _RrEulerDegreesFromQuat(
            sourceParams.rotation, params.rotationOrder);
        if (!_RrConstraintVecFinite(sourceEuler)) {
            return _RrConstraintFailure(input);
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedDelta[axis] += source.normalizedWeight *
                _RrShortestDegrees(sourceEuler[axis] -
                                   sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input
                                  : _RrConstraintFailure(input);
    }
    const RrVec3d targetEuler =
        sourceAnchor + weightedDelta / totalWeight + params.offsetDegrees;
    if (!_RrConstraintVecFinite(weightedDelta) ||
        !_RrConstraintVecFinite(targetEuler)) {
        return _RrConstraintFailure(input);
    }
    const RrVec3d outputEuler = _RrApplyEulerDelta(
        inputEuler, targetEuler, params.affect, globalWeight);
    inputParams.rotation =
        _RrQuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _RrFrameFromConstraintParams(input, inputParams);
}

RrPointFrame
_RrApplyScaleConstraint(
    const RrPointFrame &input,
    const std::vector<_RrConstraintSource> &sources,
    const _RrScaleConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_RrConstraintVecFinite(params.offset)) {
        return _RrConstraintFailure(input);
    }

    _RrTransformParams inputParams;
    if (!_RrDecomposeConstraintFrame(input, &inputParams)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d targetScale(0.0);
    double totalWeight = 0.0;
    for (const _RrConstraintSource &source : sources) {
        if (!_RrValidateSourceWeight(source)) {
            return _RrConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        _RrTransformParams sourceParams;
        if (!_RrDecomposeConstraintFrame(source.frame, &sourceParams)) {
            return _RrConstraintFailure(input);
        }
        targetScale += sourceParams.scale * source.normalizedWeight;
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input
                                  : _RrConstraintFailure(input);
    }
    targetScale = targetScale / totalWeight + params.offset;
    if (!_RrConstraintVecFinite(targetScale)) {
        return _RrConstraintFailure(input);
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (_RrAffects(params.affect, axis)) {
            inputParams.scale[axis] = _RrBlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    return _RrFrameFromConstraintParams(input, inputParams);
}

RrPointFrame
_RrApplyParentConstraint(
    const RrPointFrame &input,
    const std::vector<_RrConstraintSource> &sources,
    const _RrParentConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_RrIsValidOrder(params.rotationOrder)) {
        return _RrConstraintFailure(input);
    }

    _RrTransformParams inputParams;
    if (!_RrDecomposeConstraintFrame(input, &inputParams)) {
        return _RrConstraintFailure(input);
    }
    const RrVec3d inputEuler =
        _RrEulerDegreesFromQuat(inputParams.rotation,
                                params.rotationOrder);
    if (!_RrConstraintVecFinite(inputEuler)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d targetTranslation(0.0), targetScale(0.0);
    RrVec3d sourceAnchor(0.0), weightedRotationDelta(0.0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const _RrConstraintSource &source : sources) {
        if (!_RrValidateSourceWeight(source)) {
            return _RrConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!_RrConstraintVecFinite(source.translationOffset) ||
            !_RrConstraintVecFinite(source.rotationOffsetDegrees)) {
            return _RrConstraintFailure(input);
        }
        if (!_RrConstraintFrameUsable(source.frame)) {
            return _RrConstraintFailure(input);
        }
        RrMat4d sourceMatrix = _RrIdentity();
        if (!RrPointsToMatrix(RrIdentityLandmarks(), source.frame.points,
                              &sourceMatrix)) {
            return _RrConstraintFailure(input);
        }

        _RrTransformParams offsetParams;
        offsetParams.translation = source.translationOffset;
        offsetParams.rotation = _RrQuatFromEulerDegrees(
            source.rotationOffsetDegrees, params.rotationOrder);
        const RrMat4d targetMatrix =
            _RrParamsToMatrix(offsetParams) * sourceMatrix;
        const RrPointFrame targetFrame = RrMatrixToPoints(
            RrIdentityLandmarks(), targetMatrix);
        _RrTransformParams targetParams;
        if (!_RrDecomposeConstraintFrame(targetFrame, &targetParams)) {
            return _RrConstraintFailure(input);
        }
        targetTranslation +=
            targetParams.translation * source.normalizedWeight;
        targetScale += targetParams.scale * source.normalizedWeight;

        const RrVec3d sourceEuler = _RrEulerDegreesFromQuat(
            targetParams.rotation, params.rotationOrder);
        if (!_RrConstraintVecFinite(sourceEuler)) {
            return _RrConstraintFailure(input);
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedRotationDelta[axis] += source.normalizedWeight *
                _RrShortestDegrees(sourceEuler[axis] -
                                   sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input
                                  : _RrConstraintFailure(input);
    }
    targetTranslation /= totalWeight;
    targetScale /= totalWeight;
    const RrVec3d targetEuler =
        sourceAnchor + weightedRotationDelta / totalWeight;
    if (!_RrConstraintVecFinite(targetTranslation) ||
        !_RrConstraintVecFinite(targetScale) ||
        !_RrConstraintVecFinite(weightedRotationDelta) ||
        !_RrConstraintVecFinite(targetEuler)) {
        return _RrConstraintFailure(input);
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (_RrAffects(params.translationAxes, axis)) {
            inputParams.translation[axis] = _RrBlendEnvelope(
                inputParams.translation[axis], targetTranslation[axis],
                globalWeight);
        }
        if (_RrAffects(params.scaleAxes, axis)) {
            inputParams.scale[axis] = _RrBlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    const RrVec3d outputEuler = _RrApplyEulerDelta(
        inputEuler, targetEuler, params.rotationAxes, globalWeight);
    inputParams.rotation =
        _RrQuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _RrFrameFromConstraintParams(input, inputParams);
}

RrPointFrame
_RrApplyAimConstraint(const RrPointFrame &input,
                      const RrVec3d &targetPoint,
                      const _RrAimConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    const bool needsRollCorrection =
        params.worldUpDirection != nullptr || params.preserveInputUp;
    if (!_RrIsValidOrder(params.rotationOrder) ||
        !_RrConstraintVecFinite(targetPoint) ||
        !_RrConstraintVecFinite(params.localAimVector) ||
        (needsRollCorrection &&
         !_RrConstraintVecFinite(params.localUpVector)) ||
        !_RrConstraintVecFinite(params.rotationOffsetDegrees) ||
        (params.worldUpDirection &&
         !_RrConstraintVecFinite(*params.worldUpDirection))) {
        return _RrConstraintFailure(input);
    }

    _RrTransformParams inputParams;
    if (!_RrDecomposeConstraintFrame(input, &inputParams)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d localAim = params.localAimVector;
    if (!_RrNormalizeDirection(&localAim)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d targetAim = targetPoint - inputParams.translation;
    if (!_RrNormalizeDirection(&targetAim)) {
        return _RrConstraintFailure(input);
    }

    const RrQuatd inputRotation = inputParams.rotation.GetNormalized();
    RrVec3d currentAim = inputRotation.Transform(localAim);
    if (!_RrNormalizeDirection(&currentAim)) {
        return _RrConstraintFailure(input);
    }
    const RrRotation swing(currentAim, targetAim);
    RrRotation twist(RrVec3d(1, 0, 0), 0.0);
    if (needsRollCorrection) {
        RrVec3d localUp = params.localUpVector;
        if (!_RrNormalizeDirection(&localUp)) {
            return _RrConstraintFailure(input);
        }
        localUp -= localAim * RrDot(localAim, localUp);
        if (!_RrNormalizeDirection(&localUp)) {
            return _RrConstraintFailure(input);
        }

        RrVec3d currentUp = inputRotation.Transform(localUp);
        if (!_RrNormalizeDirection(&currentUp)) {
            return _RrConstraintFailure(input);
        }
        RrVec3d swungUp = swing.TransformDir(currentUp);
        swungUp -= targetAim * RrDot(targetAim, swungUp);
        if (!_RrNormalizeDirection(&swungUp)) {
            return _RrConstraintFailure(input);
        }

        RrVec3d desiredUp;
        if (params.worldUpDirection) {
            desiredUp = *params.worldUpDirection;
            if (!_RrNormalizeDirection(&desiredUp)) {
                return _RrConstraintFailure(input);
            }
            desiredUp -= targetAim * RrDot(targetAim, desiredUp);
            if (!_RrNormalizeDirection(&desiredUp)) {
                return _RrConstraintFailure(input);
            }
        } else {
            desiredUp =
                currentUp - targetAim * RrDot(targetAim, currentUp);
            if (!_RrNormalizeDirection(&desiredUp)) {
                desiredUp = swungUp;
            }
        }
        twist = _RrRotateOntoProjected(swungUp, desiredUp, targetAim);
    }

    RrMat4d aimedMatrix = _RrIdentity();
    aimedMatrix.SetRotateOnly(inputRotation);
    RrMat4d swingMatrix = _RrIdentity(), twistMatrix = _RrIdentity();
    swingMatrix.SetRotate(swing);
    twistMatrix.SetRotate(twist);
    aimedMatrix = aimedMatrix * swingMatrix * twistMatrix;
    const RrQuatd aimedRotation =
        aimedMatrix.ExtractRotation().GetQuat().GetNormalized();

    const RrVec3d inputEuler =
        _RrEulerDegreesFromQuat(inputRotation, params.rotationOrder);
    const RrVec3d targetEuler =
        _RrEulerDegreesFromQuat(aimedRotation, params.rotationOrder) +
        params.rotationOffsetDegrees;
    if (!_RrConstraintVecFinite(inputEuler) ||
        !_RrConstraintVecFinite(targetEuler)) {
        return _RrConstraintFailure(input);
    }
    const RrVec3d outputEuler = _RrApplyEulerDelta(
        inputEuler, targetEuler, params.affectRotation, globalWeight);
    inputParams.rotation =
        _RrQuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _RrFrameFromConstraintParams(input, inputParams);
}

RrPointFrame
_RrApplyAimLandmarks(const RrPointFrame &input,
                     const RrVec3d &targetOrigin, double weight,
                     int aimLandmarkIndex)
{
    if (!std::isfinite(weight)) {
        RrPointFrame flagged = input;
        flagged.flags |= RrPointFrameDegenerate;
        return flagged;
    }
    weight = std::min(std::max(weight, 0.0), 1.0);

    const int aimIdx = std::min(std::max(aimLandmarkIndex, 1), 3);
    const int upIdx = (aimIdx % 3) + 1;
    const int sideIdx = (upIdx % 3) + 1;

    const RrVec3d origin = input.points[0];
    const RrVec3d currentAim = input.points[size_t(aimIdx)] - origin;
    const double aimLen = currentAim.GetLength();
    const RrVec3d toTarget = targetOrigin - origin;
    if (aimLen < 1e-12 || toTarget.GetLength() < 1e-12 ||
        weight <= 0.0) {
        return input;
    }
    const RrVec3d a0 = currentAim / aimLen;
    const RrVec3d a1 = toTarget.GetNormalized();

    const RrVec3d inUp = input.points[size_t(upIdx)] - origin;
    const RrVec3d inSide = input.points[size_t(sideIdx)] - origin;
    const double handedness =
        RrDot(RrCross(currentAim, inUp), inSide) < 0 ? -1.0 : 1.0;

    const RrRotation full(a0, a1);
    const RrRotation partial(full.GetAxis(),
                             full.GetAngle() * weight);
    const RrVec3d newAim = weight >= 1.0
        ? a1
        : partial.TransformDir(a0).GetNormalized();

    const double upLen = inUp.GetLength();
    const double sideLen = inSide.GetLength();
    RrVec3d up = inUp;
    up -= newAim * RrDot(newAim, up);
    if (up.GetLength() < 1e-12) {
        up = weight >= 1.0 ? full.TransformDir(inUp)
                           : partial.TransformDir(inUp);
        up -= newAim * RrDot(newAim, up);
    }
    if (up.GetLength() < 1e-12) {
        RrPointFrame flagged = input;
        flagged.flags |= RrPointFrameDegenerate;
        return flagged;
    }
    up.Normalize();
    const RrVec3d side =
        handedness * RrCross(newAim, up).GetNormalized();

    RrPointFrame out = input;
    out.points[size_t(aimIdx)] = origin + newAim * aimLen;
    out.points[size_t(upIdx)] = origin + up * upLen;
    out.points[size_t(sideIdx)] = origin + side * sideLen;
    return out;
}

// Single-chain IK (singleChainIk.cpp).

enum : int {
    _RrFabrikIterations = 128,
};

enum class _RrSingleChainIkMode : uint8_t {
    RotatePlane = 0,
    SingleChain = 1,
};

struct _RrSingleChainIkParams {
    _RrSingleChainIkMode mode = _RrSingleChainIkMode::RotatePlane;
    RrVec3d pole{0.0, 1.0, 0.0};
    double twistDegrees = 0.0;
    double weight = 1.0;
};

bool
_RrIkVecFinite(const RrVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}

bool
_RrIkFrameFinite(const RrPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const RrVec3d &point : frame.points) {
        if (!_RrIkVecFinite(point)) {
            return false;
        }
    }
    return true;
}

std::vector<RrPointFrame>
_RrIkDegenerate(const std::vector<RrPointFrame> &frames)
{
    std::vector<RrPointFrame> failed = frames;
    for (RrPointFrame &frame : failed) {
        frame.flags |= RrPointFrameDegenerate;
    }
    return failed;
}

RrVec3d
_RrIkProjectPerpendicular(const RrVec3d &v, const RrVec3d &axis)
{
    return v - axis * RrDot(v, axis);
}

RrVec3d
_RrIkWorldAxisLeastParallel(const RrVec3d &axis)
{
    const RrVec3d world[3] = {
        RrVec3d(1.0, 0.0, 0.0),
        RrVec3d(0.0, 1.0, 0.0),
        RrVec3d(0.0, 0.0, 1.0),
    };
    int best = 0;
    double bestAlignment = std::abs(RrDot(world[0], axis));
    for (int i = 1; i < 3; ++i) {
        const double alignment = std::abs(RrDot(world[i], axis));
        if (alignment < bestAlignment) {
            best = i;
            bestAlignment = alignment;
        }
    }
    return world[best];
}

RrVec3d
_RrIkStablePerpendicular(const RrVec3d &axis, const RrVec3d &first,
                          const RrVec3d &second, double epsilon)
{
    auto normalized = [](const RrVec3d &candidate) {
        const double length = candidate.GetLength();
        return std::isfinite(length) && length > 0.0
            ? candidate / length
            : RrVec3d(0.0);
    };
    RrVec3d result = _RrIkProjectPerpendicular(normalized(first), axis);
    if (result.GetLength() <= epsilon) {
        result = _RrIkProjectPerpendicular(normalized(second), axis);
    }
    if (result.GetLength() <= epsilon) {
        result = _RrIkProjectPerpendicular(
            _RrIkWorldAxisLeastParallel(axis), axis);
    }
    return result.GetNormalized();
}

RrVec3d
_RrIkRotate(const RrVec3d &v, const RrVec3d &unitAxis, double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return v * c + RrCross(unitAxis, v) * s +
           unitAxis * RrDot(unitAxis, v) * (1.0 - c);
}

RrVec3d
_RrIkRotateToward(const RrVec3d &v, const RrVec3d &fromAxis,
                  const RrVec3d &toAxis, double weight,
                  const RrVec3d &fallback, double epsilon)
{
    const RrVec3d from = fromAxis.GetNormalized();
    const RrVec3d to = toAxis.GetNormalized();
    const double dot =
        std::min(std::max(RrDot(from, to), -1.0), 1.0);
    RrVec3d rotationAxis = RrCross(from, to);
    const double sinAngle = rotationAxis.GetLength();
    double angle = 0.0;
    if (sinAngle > epsilon) {
        rotationAxis /= sinAngle;
        angle = std::atan2(sinAngle, dot);
    } else if (dot < 0.0) {
        rotationAxis = _RrIkStablePerpendicular(
            from, fallback, RrVec3d(0.0), epsilon);
        angle = _RrPi;
    } else {
        return v;
    }
    return _RrIkRotate(v, rotationAxis, angle * weight);
}

RrVec3d
_RrIkBlendDirection(const RrVec3d &from, const RrVec3d &to,
                    double weight, const RrVec3d &fallback,
                    double epsilon)
{
    RrVec3d result =
        _RrIkRotateToward(from, from, to, weight, fallback, epsilon);
    const double length = result.GetLength();
    if (length <= epsilon || !_RrIkVecFinite(result)) {
        return from.GetNormalized();
    }
    return result / length;
}

struct _RrIkFrameBasis {
    RrVec3d x;
    RrVec3d up;
    double xLength = 0.0;
    double yLength = 0.0;
    double zLength = 0.0;
    double handedness = 1.0;
};

bool
_RrIkExtractBasis(const RrPointFrame &frame, double epsilon,
                  _RrIkFrameBasis *basis)
{
    const RrVec3d x = frame.points[1] - frame.points[0];
    const RrVec3d y = frame.points[2] - frame.points[0];
    const RrVec3d z = frame.points[3] - frame.points[0];
    const double xLength = x.GetLength();
    const double yLength = y.GetLength();
    const double zLength = z.GetLength();
    const double frameScale = std::max({xLength, yLength, zLength});
    if (!std::isfinite(frameScale) || frameScale <= 0.0 ||
        xLength <= frameScale * epsilon ||
        yLength <= frameScale * epsilon ||
        zLength <= frameScale * epsilon) {
        return false;
    }

    const RrVec3d unitX = x / xLength;
    RrVec3d up = _RrIkProjectPerpendicular(y / yLength, unitX);
    if (up.GetLength() <= epsilon) {
        return false;
    }
    up.Normalize();

    const double determinant = RrDot(
        RrCross(x / xLength, y / yLength), z / zLength);
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-10) {
        return false;
    }

    basis->x = unitX;
    basis->up = up;
    basis->xLength = xLength;
    basis->yLength = yLength;
    basis->zLength = zLength;
    basis->handedness = determinant < 0.0 ? -1.0 : 1.0;
    return true;
}

RrPointFrame
_RrIkFrameFromBasis(const _RrIkFrameBasis &basis, const RrVec3d &origin,
                    const RrVec3d &unitX, const RrVec3d &unitUp)
{
    const RrVec3d side =
        basis.handedness * RrCross(unitX, unitUp).GetNormalized();
    RrPointFrame frame;
    frame.points[0] = origin;
    frame.points[1] = origin + unitX * basis.xLength;
    frame.points[2] = origin + unitUp * basis.yLength;
    frame.points[3] = origin + side * basis.zLength;
    frame.flags = RrPointFrameValid;
    if (basis.handedness < 0.0) {
        frame.flags |= RrPointFrameReflected;
    }
    return frame;
}

_RrIkFrameBasis
_RrIkAimedBasis(const _RrIkFrameBasis &basis, const RrVec3d &unitAim,
                double epsilon)
{
    RrVec3d up = _RrIkRotateToward(basis.up, basis.x, unitAim, 1.0,
                                   basis.up, epsilon);
    up = _RrIkProjectPerpendicular(up, unitAim);
    if (up.GetLength() <= epsilon) {
        up = _RrIkStablePerpendicular(unitAim, basis.up, basis.x,
                                       epsilon);
    } else {
        up.Normalize();
    }
    _RrIkFrameBasis aimed = basis;
    aimed.x = unitAim;
    aimed.up = up;
    return aimed;
}

_RrIkFrameBasis
_RrIkTransportedBasis(const _RrIkFrameBasis &basis,
                      const RrVec3d &fromDirection,
                      const RrVec3d &toDirection, double epsilon)
{
    _RrIkFrameBasis transported = basis;
    transported.x = _RrIkRotateToward(basis.x, fromDirection,
                                      toDirection, 1.0, basis.up,
                                      epsilon);
    transported.x.Normalize();
    transported.up = _RrIkRotateToward(basis.up, fromDirection,
                                       toDirection, 1.0, basis.up,
                                       epsilon);
    transported.up = _RrIkProjectPerpendicular(transported.up,
                                                transported.x);
    if (transported.up.GetLength() <= epsilon) {
        transported.up = _RrIkStablePerpendicular(
            transported.x, basis.up, basis.x, epsilon);
    } else {
        transported.up.Normalize();
    }
    return transported;
}

double
_RrIkSignedAngle(const RrVec3d &from, const RrVec3d &to,
                 const RrVec3d &unitAxis)
{
    return std::atan2(
        RrDot(unitAxis, RrCross(from, to)),
        std::min(std::max(RrDot(from, to), -1.0), 1.0));
}

RrPointFrame
_RrIkBlendEndFrame(const _RrIkFrameBasis &input,
                   const _RrIkFrameBasis &effector,
                   const RrVec3d &origin, double weight,
                   double epsilon)
{
    const RrVec3d x = _RrIkBlendDirection(
        input.x, effector.x, weight, input.up, epsilon);

    RrVec3d carriedAtTarget = _RrIkRotateToward(
        input.up, input.x, effector.x, 1.0, input.up, epsilon);
    carriedAtTarget =
        _RrIkProjectPerpendicular(carriedAtTarget, effector.x);
    if (carriedAtTarget.GetLength() <= epsilon) {
        carriedAtTarget = _RrIkStablePerpendicular(
            effector.x, input.up, effector.up, epsilon);
    } else {
        carriedAtTarget.Normalize();
    }
    const double roll =
        _RrIkSignedAngle(carriedAtTarget, effector.up, effector.x);

    RrVec3d up = _RrIkRotateToward(input.up, input.x, effector.x,
                                   weight, input.up, epsilon);
    up = _RrIkProjectPerpendicular(up, x);
    if (up.GetLength() <= epsilon) {
        up = _RrIkStablePerpendicular(x, input.up, effector.up,
                                       epsilon);
    } else {
        up.Normalize();
    }
    up = _RrIkRotate(up, x, roll * weight).GetNormalized();
    return _RrIkFrameFromBasis(input, origin, x, up);
}

RrVec3d
_RrIkDirectionOrFallback(const RrVec3d &direction,
                         const RrVec3d &fallback,
                         double positionEpsilon, double angularEpsilon)
{
    const double directionLength = direction.GetLength();
    if (directionLength > positionEpsilon) {
        return direction / directionLength;
    }
    const double fallbackLength = fallback.GetLength();
    if (fallbackLength > angularEpsilon) {
        return fallback / fallbackLength;
    }
    return RrVec3d(1.0, 0.0, 0.0);
}

std::vector<RrVec3d>
_RrIkSolvePositions(
    const std::vector<RrVec3d> &original,
    const std::vector<double> &lengths,
    const std::vector<_RrIkFrameBasis> &bases,
    const _RrIkFrameBasis &effectorBasis, const RrVec3d &goal,
    const _RrSingleChainIkParams &params, double positionEpsilon,
    double angularEpsilon)
{
    const size_t count = original.size();
    const RrVec3d root = original.front();
    const double totalLength = [&]() {
        double total = 0.0;
        for (double length : lengths)
            total += length;
        return total;
    }();

    RrVec3d rootToGoal = goal - root;
    const double goalDistance = rootToGoal.GetLength();
    RrVec3d solveAxis;
    if (goalDistance > positionEpsilon) {
        solveAxis = rootToGoal / goalDistance;
    } else {
        solveAxis = original.back() - root;
        if (solveAxis.GetLength() <= positionEpsilon) {
            solveAxis = original[1] - root;
        }
        solveAxis = _RrIkDirectionOrFallback(
            solveAxis, bases.front().x, positionEpsilon, angularEpsilon);
    }

    if (goalDistance >= totalLength - positionEpsilon &&
        goalDistance > positionEpsilon) {
        std::vector<RrVec3d> extended(count);
        extended[0] = root;
        for (size_t i = 0; i < lengths.size(); ++i) {
            extended[i + 1] = extended[i] + solveAxis * lengths[i];
        }
        return extended;
    }

    std::vector<RrVec3d> positions = original;
    RrVec3d planeUp;
    RrVec3d planeNormal;
    if (params.mode == _RrSingleChainIkMode::RotatePlane) {
        RrVec3d poleOffset = params.pole - root;
        if (poleOffset.GetLength() <= positionEpsilon) {
            poleOffset = RrVec3d(0.0);
        }
        planeUp = _RrIkStablePerpendicular(
            solveAxis, poleOffset, bases.front().up, angularEpsilon);
        planeUp = _RrIkRotate(
            planeUp, solveAxis, params.twistDegrees * (_RrPi / 180.0));
        planeUp.Normalize();
    } else {
        planeUp = _RrIkStablePerpendicular(
            solveAxis, effectorBasis.up, effectorBasis.x,
            angularEpsilon);
    }
    planeNormal = RrCross(solveAxis, planeUp).GetNormalized();

    for (size_t i = 1; i + 1 < count; ++i) {
        const RrVec3d relative = positions[i] - root;
        const double along = RrDot(relative, solveAxis);
        const double towardPlane = std::abs(RrDot(relative, planeUp));
        positions[i] =
            root + solveAxis * along + planeUp * towardPlane;
    }

    double maxOffAxis = 0.0;
    for (size_t i = 1; i + 1 < count; ++i) {
        maxOffAxis = std::max(
            maxOffAxis,
            _RrIkProjectPerpendicular(positions[i] - root, solveAxis)
                .GetLength());
    }
    if (count > 2 && maxOffAxis <= positionEpsilon * 8.0 &&
        goalDistance < totalLength - positionEpsilon) {
        const RrVec3d bend = planeUp;
        double distance = 0.0;
        const double amplitude = std::max(
            totalLength * 0.05, positionEpsilon * 16.0);
        for (size_t i = 1; i + 1 < count; ++i) {
            distance += lengths[i - 1];
            positions[i] += bend *
                (amplitude *
                 std::sin(_RrPi * distance / totalLength));
        }
    }

    std::vector<RrVec3d> fallback(lengths.size());
    for (size_t i = 0; i < lengths.size(); ++i) {
        RrVec3d direction = positions[i + 1] - positions[i];
        direction -= planeNormal * RrDot(direction, planeNormal);
        fallback[i] = _RrIkDirectionOrFallback(
            direction, solveAxis, positionEpsilon, angularEpsilon);
    }

    std::vector<RrVec3d> best = positions;
    double bestError = std::numeric_limits<double>::infinity();
    const double tolerance = std::max(
        positionEpsilon * 8.0, totalLength * 1e-12);
    for (int iteration = 0; iteration < _RrFabrikIterations;
         ++iteration) {
        positions.back() = goal;

        for (size_t i = count - 1; i-- > 0;) {
            const RrVec3d direction = _RrIkDirectionOrFallback(
                positions[i] - positions[i + 1], -fallback[i],
                positionEpsilon, angularEpsilon);
            positions[i] = positions[i + 1] + direction * lengths[i];
        }

        positions[0] = root;
        for (size_t i = 0; i < lengths.size(); ++i) {
            const RrVec3d direction = _RrIkDirectionOrFallback(
                positions[i + 1] - positions[i], fallback[i],
                positionEpsilon, angularEpsilon);
            positions[i + 1] = positions[i] + direction * lengths[i];
            fallback[i] = direction;
        }

        const double error = (positions.back() - goal).GetLength();
        if (error < bestError) {
            bestError = error;
            best = positions;
        }
        if (error <= tolerance) {
            break;
        }
    }
    return best;
}

std::vector<RrPointFrame>
_RrSolveSingleChainIk(const std::vector<RrPointFrame> &currentFrames,
                      const RrPointFrame &effectorFrame,
                      const _RrSingleChainIkParams &params)
{
    if (!std::isfinite(params.weight)) {
        return _RrIkDegenerate(currentFrames);
    }
    const double weight =
        std::min(std::max(params.weight, 0.0), 1.0);
    if (weight <= 0.0) {
        return currentFrames;
    }
    if (currentFrames.size() < 2) {
        return _RrIkDegenerate(currentFrames);
    }
    if (params.mode != _RrSingleChainIkMode::RotatePlane &&
        params.mode != _RrSingleChainIkMode::SingleChain) {
        return _RrIkDegenerate(currentFrames);
    }
    if (!_RrIkVecFinite(effectorFrame.points[0])) {
        return _RrIkDegenerate(currentFrames);
    }
    if (params.mode == _RrSingleChainIkMode::RotatePlane &&
        (!_RrIkVecFinite(params.pole) ||
         !std::isfinite(params.twistDegrees))) {
        return _RrIkDegenerate(currentFrames);
    }
    for (const RrPointFrame &frame : currentFrames) {
        if (!_RrIkFrameFinite(frame)) {
            return _RrIkDegenerate(currentFrames);
        }
    }

    std::vector<RrVec3d> original(currentFrames.size());
    std::vector<double> lengths(currentFrames.size() - 1);
    double segmentScale = 0.0;
    for (size_t i = 0; i < currentFrames.size(); ++i) {
        original[i] = currentFrames[i].points[0];
        if (i + 1 < currentFrames.size()) {
            lengths[i] =
                (currentFrames[i + 1].points[0] - original[i])
                    .GetLength();
            segmentScale = std::max(segmentScale, lengths[i]);
        }
    }
    if (!std::isfinite(segmentScale) || segmentScale <= 0.0) {
        return _RrIkDegenerate(currentFrames);
    }
    const double positionEpsilon = segmentScale * 1e-10;
    constexpr double angularEpsilon = 1e-12;

    std::vector<_RrIkFrameBasis> bases(currentFrames.size());
    for (size_t i = 0; i < currentFrames.size(); ++i) {
        if (!_RrIkExtractBasis(currentFrames[i], angularEpsilon,
                               &bases[i])) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    _RrIkFrameBasis effectorBasis = bases.back();
    if (params.mode == _RrSingleChainIkMode::SingleChain) {
        if (!_RrIkFrameFinite(effectorFrame) ||
            !_RrIkExtractBasis(effectorFrame, angularEpsilon,
                               &effectorBasis)) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    for (double length : lengths) {
        if (!std::isfinite(length) || length <= positionEpsilon) {
            return _RrIkDegenerate(currentFrames);
        }
    }

    const std::vector<RrVec3d> solved = _RrIkSolvePositions(
        original, lengths, bases, effectorBasis,
        effectorFrame.points[0], params, positionEpsilon,
        angularEpsilon);
    if (solved.size() != currentFrames.size()) {
        return _RrIkDegenerate(currentFrames);
    }

    std::vector<_RrIkFrameBasis> solvedBases = bases;
    for (size_t i = 0; i + 1 < solvedBases.size(); ++i) {
        const RrVec3d solvedSegment = solved[i + 1] - solved[i];
        const RrVec3d aim = solvedSegment / solvedSegment.GetLength();
        solvedBases[i] = _RrIkAimedBasis(bases[i], aim, angularEpsilon);
    }
    if (params.mode == _RrSingleChainIkMode::SingleChain) {
        solvedBases.back() = effectorBasis;
    } else {
        const RrVec3d currentTerminalSegment =
            original.back() - original[original.size() - 2];
        const RrVec3d currentTerminal =
            currentTerminalSegment / currentTerminalSegment.GetLength();
        const RrVec3d solvedTerminalSegment =
            solved.back() - solved[solved.size() - 2];
        const RrVec3d solvedTerminal =
            solvedTerminalSegment / solvedTerminalSegment.GetLength();
        solvedBases.back() = _RrIkTransportedBasis(
            bases.back(), currentTerminal, solvedTerminal,
            angularEpsilon);
    }

    std::vector<RrPointFrame> result(currentFrames.size());
    if (weight >= 1.0) {
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = _RrIkFrameFromBasis(
                bases[i], solved[i], solvedBases[i].x, solvedBases[i].up);
        }
    } else {
        std::vector<RrVec3d> blended(currentFrames.size());
        blended[0] = original[0];
        for (size_t i = 0; i < lengths.size(); ++i) {
            const RrVec3d currentDirection =
                (original[i + 1] - original[i]) / lengths[i];
            const RrVec3d solvedSegment = solved[i + 1] - solved[i];
            const RrVec3d solvedDirection =
                solvedSegment / solvedSegment.GetLength();
            const RrVec3d direction = _RrIkBlendDirection(
                currentDirection, solvedDirection, weight, bases[i].up,
                angularEpsilon);
            blended[i + 1] = blended[i] + direction * lengths[i];
        }
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = _RrIkBlendEndFrame(
                bases[i], solvedBases[i], blended[i], weight,
                angularEpsilon);
        }
    }

    for (const RrPointFrame &frame : result) {
        if (!_RrIkFrameFinite(frame)) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    return result;
}

// RigExecPrepareRestDerivedIkChain (rigEvaluator.cpp): rest segment
// lengths laid along the current chain's directions.
bool
_RrPrepareRestDerivedIkChain(
    const std::vector<RrPointFrame> &current,
    const std::vector<RrPointFrame> &rest,
    std::vector<RrPointFrame> *prepared)
{
    if (current.size() != rest.size() || current.empty()) {
        return false;
    }
    bool usableRestLayout = true;
    for (size_t i = 1; i < rest.size(); ++i) {
        const double segmentLength =
            (rest[i].points[0] - rest[i - 1].points[0]).GetLength();
        if (!std::isfinite(segmentLength) || segmentLength <= 0.0) {
            usableRestLayout = false;
            break;
        }
    }
    const std::vector<RrPointFrame> &lengthReference =
        usableRestLayout ? rest : current;
    prepared->clear();
    prepared->reserve(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
        if (!_RrConstraintFrameUsable(current[i]) ||
            !_RrConstraintFrameUsable(rest[i])) {
            return false;
        }
        RrVec3d origin = current[i].points[0];
        if (i > 0) {
            RrMat4d parentRest = _RrIdentity();
            RrMat4d parentPrepared = _RrIdentity();
            if (!RrPointsToMatrix(RrIdentityLandmarks(),
                                  lengthReference[i - 1].points,
                                  &parentRest) ||
                !RrPointsToMatrix(RrIdentityLandmarks(),
                                  prepared->back().points,
                                  &parentPrepared)) {
                return false;
            }
            origin = parentPrepared.TransformAffine(
                parentRest.GetInverse().TransformAffine(
                    lengthReference[i].points[0]));
        }

        RrPointFrame frame = current[i];
        frame.points[0] = origin;
        for (size_t axis = 1; axis < frame.points.size(); ++axis) {
            RrVec3d direction =
                current[i].points[axis] - current[i].points[0];
            const double directionLength = direction.GetLength();
            const double length =
                (lengthReference[i].points[axis] -
                 lengthReference[i].points[0]).GetLength();
            if (!std::isfinite(length) || length <= 0.0 ||
                !std::isfinite(directionLength) ||
                directionLength <= 0.0) {
                return false;
            }
            direction /= directionLength;
            frame.points[axis] = origin + direction * length;
        }
        if (!_RrConstraintFrameUsable(frame)) {
            return false;
        }
        prepared->push_back(frame);
    }
    return true;
}

// The Constraint step (bakedPose.cpp).

// RigExecWeightPacket::ResolveAll (types.cpp), over an RrWeightPacket.
bool
_RrResolveWeightPacketAll(const RrProgram *program,
                          const RrWeightPacket &packet, size_t count,
                          std::vector<float> *resolved)
{
    if (!resolved || !packet.valid) {
        return false;
    }
    if (packet.rangePolicy != 0 &&
        !program->TokenEquals(packet.rangePolicy, "strict") &&
        !program->TokenEquals(packet.rangePolicy, "clamp")) {
        return false;
    }
    const bool isConstant =
        program->TokenEquals(packet.representation, "constant");
    const bool isDense =
        program->TokenEquals(packet.representation, "dense");
    const bool isSparse =
        program->TokenEquals(packet.representation, "sparse");
    if (isConstant) {
        if (!packet.values.empty() || !packet.indices.empty()) {
            return false;
        }
    } else if (isDense) {
        if (!packet.indices.empty() || packet.values.size() != count) {
            return false;
        }
    } else if (isSparse) {
        if (packet.indices.size() != packet.values.size()) {
            return false;
        }
        for (size_t i = 0; i < packet.indices.size(); ++i) {
            if (packet.indices[i] < 0 ||
                size_t(packet.indices[i]) >= count ||
                (i > 0 &&
                 packet.indices[i] <= packet.indices[i - 1])) {
                return false;
            }
        }
    } else {
        return false;
    }

    const auto usable = [](float v) {
        return std::isfinite(v) && v >= 0.0f && v <= 1.0f;
    };

    if (isConstant) {
        if (!usable(packet.defaultWeight)) {
            return false;
        }
        resolved->assign(count, packet.defaultWeight);
        return true;
    }

    if (isDense) {
        for (size_t i = 0; i < count; ++i) {
            if (!usable(packet.values[i])) {
                return false;
            }
        }
        resolved->assign(packet.values.begin(),
                         packet.values.begin() + count);
        return true;
    }

    if (packet.indices.size() < count && !usable(packet.defaultWeight)) {
        return false;
    }
    for (const float value : packet.values) {
        if (!usable(value)) {
            return false;
        }
    }
    std::vector<float> valuesOut(count, packet.defaultWeight);
    for (size_t i = 0; i < packet.indices.size(); ++i) {
        valuesOut[size_t(packet.indices[i])] = packet.values[i];
    }
    resolved->swap(valuesOut);
    return true;
}

// Whether the wire weight object names a type the oracle understands
// (_IsWeightObjectType: static, dynamic, curvenet, combine, and the
// three volumetric kinds).
bool
_RrIsWeightObjectType(const RrProgram *program, uint32_t type)
{
    return program->TokenEquals(type, "RigExecStaticWeight") ||
           program->TokenEquals(type, "RigExecDynamicWeight") ||
           program->TokenEquals(type, "RigExecCurvenetWeight") ||
           program->TokenEquals(type, "RigExecCombineWeight") ||
           program->TokenEquals(type, "RigExecSphereWeight") ||
           program->TokenEquals(type, "RigExecPlaneWeight") ||
           program->TokenEquals(type, "RigExecCurveWeight");
}

// RigExecApplyRevisedAncestorDelta (rigEvaluator.cpp): the native
// source rides the delta of the deepest revised ancestor above it.
// Provider paths are never the root, where slash-counting and
// SdfPath's element count agree everywhere else.
bool
_RrApplyRevisedAncestorDelta(
    const RrProgram *program, const std::string &xformPath,
    const std::vector<RigExecWireAncestorRead> &ancestors,
    RrPointFrame *frame)
{
    const RrStore &store = program->store;
    const auto elementCount = [](const std::string &path) {
        return size_t(std::count(path.begin(), path.end(), '/'));
    };
    std::string closest;
    RrPointFrame closestBase, closestCurrent;
    for (const RigExecWireAncestorRead &a : ancestors) {
        if (a.slot < 0 ||
            size_t(a.slot) >= program->slotMeta->paths.size() ||
            size_t(a.base) >= store.base.size() ||
            size_t(a.fin) >= store.fin.size()) {
            return false;
        }
        const std::string provider =
            program->TextOrEmpty(program->slotMeta->paths[size_t(a.slot)]);
        const RrPointFrame &base = store.base[size_t(a.base)];
        const RrPointFrame &current = store.fin[size_t(a.fin)];
        if (provider == xformPath ||
            xformPath.size() <= provider.size() ||
            xformPath.compare(0, provider.size(), provider) != 0 ||
            xformPath[provider.size()] != '/' ||
            current.points == base.points) {
            continue;
        }
        if (closest.empty() ||
            elementCount(provider) > elementCount(closest)) {
            closest = provider;
            closestBase = base;
            closestCurrent = current;
        }
    }
    if (!closest.empty()) {
        RrMat4d delta = _RrIdentity();
        if (!RrPointsToMatrix(closestBase.points, closestCurrent.points,
                              &delta)) {
            return false;
        }
        *frame = RrMatrixToPoints(frame->points, delta);
    }
    return frame->IsValid();
}

} // namespace

namespace runtimePoseDetail {

bool _RrRunConstraintStep(RrProgram *program, size_t step,
                          std::string *error);

bool
_RrRunConstraintStep(RrProgram *program, size_t step,
                     std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->commits.size() ||
        size_t(wire.object) >= store.commits.size() ||
        size_t(wire.object) >= program->poses->walkSteps.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    const RigExecWireCommit &wireCommit =
        program->poses->commits[size_t(wire.object)];
    RrCommitScratch &commit = store.commits[size_t(wire.object)];
    const int walkIndex =
        program->poses->walkSteps[size_t(wire.object)].index;
    if (walkIndex < 0 ||
        size_t(walkIndex) >= program->poses->constraints.size() ||
        size_t(walkIndex) >= scratch->weightScratch.size() ||
        size_t(walkIndex) >= scratch->weightError.size() ||
        size_t(walkIndex) >= scratch->constraintWeights.size() ||
        size_t(walkIndex) >= scratch->constraintHaveWeight.size()) {
        if (error) {
            *error = _RrStepHead(program, step) +
                     " names no constraint";
        }
        return false;
    }
    const size_t ci = size_t(walkIndex);
    const RigExecWireConstraint &c = program->poses->constraints[ci];
    const std::string cpath = program->TextOrEmpty(c.path);
    RrStepOutput &output = store.stepOutputs[step];
    commit.abandoned = true;
    std::fill(commit.present.begin(), commit.present.end(), 0);

    const auto finish = [&]() {
        if (wireCommit.split) {
            return true;
        }
        if (!commit.abandoned) {
            if (!_RrComputeCommitDeltas(program, step, wireCommit,
                                        &commit, error) ||
                !_RrStageCommitPairs(program, step, wireCommit, &commit,
                                     0, wireCommit.propagate.size(),
                                     error)) {
                return false;
            }
        }
        return _RrFinishCommit(program, step, size_t(wire.object),
                               error);
    };

    const int deltaBase = c.deltaBase;
    if (size_t(wire.object) >= scratch->recordAfter.size() ||
        size_t(wire.object) >= scratch->recordEveryTarget.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    scratch->recordAfter[size_t(wire.object)] = 1;
    scratch->recordEveryTarget[size_t(wire.object)] = 1;
    if (deltaBase >= 0) {
        if (size_t(deltaBase) >= scratch->deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaPresent.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no delta path";
            }
            return false;
        }
        scratch->deltaPresent[size_t(deltaBase)] = 0;
        // Published for the geometry fold (the store pair mirrors the
        // baked B.deltaValues/deltaPresent the fold consumes).
        store.deltaPresent[size_t(deltaBase)] = 0;
    }
    if (c.target < 0) {
        return finish();
    }
    if (!program->ReadConstraint(ci, RrConstraintEnabled).boolean) {
        return finish();
    }
    // The envelope, in the dynamic path's THREE exclusive arms. The
    // oracle call resolves into this constraint's own scratch; the
    // runtime answers it from the weight packets the weights family
    // resolved, falling back to the captured envelope when the
    // weights family is masked out of the run.
    double weight = 1.0;
    if (c.weightObject != 0 && c.pointsTarget == 0) {
        std::vector<float> &envelope = scratch->weightScratch[ci];
        std::string &envelopeError = scratch->weightError[ci];
        envelope.clear();
        envelopeError.clear();
        envelope.assign(1, 1.0f);
        bool resolved = false;
        const auto found = scratch->weightIndex.find(c.weightObject);
        if (found != scratch->weightIndex.end() &&
            found->second < store.weightPackets.size() &&
            !store.weightPackets.empty()) {
            envelope.clear();
            resolved = _RrResolveWeightPacketAll(
                program, store.weightPackets[found->second], 1,
                &envelope);
        }
        if (!resolved && scratch->constraintHaveWeight[ci]) {
            envelope.assign(
                1, scratch->constraintWeights[ci]);
            resolved = true;
        }
        if (!resolved || envelope.size() != 1) {
            const std::string wpath =
                program->TextOrEmpty(c.weightObject);
            if (found == scratch->weightIndex.end()) {
                envelopeError = "missing weight object " + wpath;
            } else {
                const uint32_t type =
                    program->geometry
                        ->weightObjects[found->second]
                        .type;
                if (!_RrIsWeightObjectType(program, type)) {
                    envelopeError = "unknown weight object type " +
                                    program->TextOrEmpty(type) + " on " +
                                    wpath;
                } else {
                    envelopeError =
                        "could not resolve weights on " + wpath;
                }
            }
            output.diagnostics.push_back(
                cpath + ": " + envelopeError +
                "; constraint passed through");
            return finish();
        }
        weight = envelope[0];
    } else if (c.weightObject == 0) {
        weight = double(program->ReadConstraint(ci,
                                                RrConstraintDefaultWeight)
                            .f32);
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
            output.diagnostics.push_back(
                cpath +
                " has inputs:defaultWeight outside finite [0, 1]; "
                "constraint passed through");
            return finish();
        }
    }
    if (weight <= 0.0 && (c.pointsTarget == 0 || c.weightObject == 0)) {
        if (deltaBase >= 0) {
            if (size_t(deltaBase) >= scratch->deltaValues.size() ||
                size_t(deltaBase) >= store.deltaValues.size() ||
                size_t(deltaBase) >= store.deltaPresent.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no delta path";
                }
                return false;
            }
            scratch->deltaValues[size_t(deltaBase)] = _RrIdentity();
            scratch->deltaPresent[size_t(deltaBase)] = 1;
            store.deltaValues[size_t(deltaBase)] = _RrIdentity();
            store.deltaPresent[size_t(deltaBase)] = 1;
        }
        return finish();
    }

    const auto resolveSource =
        [&](int slot, uint32_t finRead, int native,
            const std::vector<RigExecWireAncestorRead> &ancestors,
            RrPointFrame *out) {
            if (slot >= 0) {
                if (size_t(finRead) >= store.fin.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no fin version";
                    }
                    return false;
                }
                *out = store.fin[size_t(finRead)];
                return out->IsValid();
            }
            if (native < 0 ||
                size_t(native) >= store.nativeFrameOk.size() ||
                size_t(native) >= store.nativeFrames.size() ||
                size_t(native) >= program->poses->nativeSources.size() ||
                !store.nativeFrameOk[size_t(native)]) {
                return false;
            }
            *out = store.nativeFrames[size_t(native)];
            if (!out->IsValid()) {
                return false;
            }
            return _RrApplyRevisedAncestorDelta(
                program,
                program->TextOrEmpty(
                    program->poses->nativeSources[size_t(native)].path),
                ancestors, out);
        };

    if (size_t(c.arrays) >= store.arrays.size()) {
        if (error) {
            *error = _RrStepHead(program, step) +
                     " names no constraint arrays";
        }
        return false;
    }
    const RrConstraintArraysLive &arrays = store.arrays[size_t(c.arrays)];

    if (c.singleChainIk) {
        if (wireCommit.targetReads.size() != c.targetSlots.size() ||
            commit.ikChain.size() != c.targetSlots.size() ||
            commit.ikRest.size() != c.targetSlots.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no chain target";
            }
            return false;
        }
        bool inputsValid = true;
        for (size_t k = 0; k < c.targetSlots.size(); ++k) {
            if (size_t(wireCommit.targetReads[k]) >= store.fin.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no fin version";
                }
                return false;
            }
            commit.ikChain[k] =
                store.fin[size_t(wireCommit.targetReads[k])];
        }
        RrPointFrame effector;
        if (!resolveSource(c.effector, wireCommit.effectorRead,
                           c.effectorNative,
                           wireCommit.effectorAncestors, &effector)) {
            output.diagnostics.push_back(
                cpath + " could not resolve its effector; constraint "
                "passed through");
            inputsValid = false;
        }
        _RrSingleChainIkParams params;
        params.mode = _RrSingleChainIkMode(c.ikMode);
        params.weight = weight;
        if (c.ikMode == 0) {
            params.pole = program->ReadConstraint(ci,
                                                   RrConstraintPoleVector)
                              .vec;
            params.twistDegrees =
                program->ReadConstraint(ci,
                                        RrConstraintTwistDegrees)
                    .f64;
        }
        if (inputsValid && c.ikMode == 0 && c.poleModeObject) {
            if (c.poleObjects.empty()) {
                output.diagnostics.push_back(
                    cpath + " uses object pole mode with no pole-vector "
                    "objects; constraint passed through");
                inputsValid = false;
            }
            if (inputsValid && !arrays.poleOk) {
                inputsValid = false;
            }
            RrVec3d polePoint(0.0);
            double total = 0;
            for (size_t k = 0;
                 inputsValid && k < c.poleObjects.size(); ++k) {
                if (k >= wireCommit.poleReads.size() ||
                    k >= wireCommit.poleAncestors.size() ||
                    k >= c.poleObjectNatives.size() ||
                    k >= arrays.poleWeights.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no pole object";
                    }
                    return false;
                }
                RrPointFrame poleFrame;
                if (!resolveSource(
                        c.poleObjects[k], wireCommit.poleReads[k],
                        c.poleObjectNatives[k],
                        wireCommit.poleAncestors[k], &poleFrame) ||
                    !std::isfinite(arrays.poleWeights[k]) ||
                    arrays.poleWeights[k] < 0) {
                    output.diagnostics.push_back(
                        cpath + " has an invalid pole-vector source or "
                        "weight; constraint passed through");
                    inputsValid = false;
                    break;
                }
                polePoint +=
                    poleFrame.points[0] * arrays.poleWeights[k];
                total += arrays.poleWeights[k];
            }
            if (inputsValid && total <= 0) {
                output.diagnostics.push_back(
                    cpath + " has zero total pole-vector weight; "
                    "constraint passed through");
                inputsValid = false;
            }
            if (inputsValid) {
                params.pole = polePoint / total;
            }
        }
        const std::vector<RrPointFrame> *solveChain = &commit.ikChain;
        if (inputsValid && !c.useAnimatedTs) {
            for (size_t k = 0; k < c.targetSlots.size(); ++k) {
                if (c.targetSlots[k] < 0 ||
                    size_t(c.targetSlots[k]) >=
                        scratch->restFrames.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no rest slot";
                    }
                    return false;
                }
                commit.ikRest[k] =
                    (k < c.ikRestLive.size() && c.ikRestLive[k] &&
                     k < commit.ikChain.size())
                        ? commit.ikChain[k]
                        : scratch->restFrames[size_t(c.targetSlots[k])];
            }
            if (!_RrPrepareRestDerivedIkChain(commit.ikChain,
                                              commit.ikRest,
                                              &commit.ikPrepared)) {
                output.diagnostics.push_back(
                    cpath + " could not prepare rest-derived IK inputs; "
                    "constraint passed through");
                inputsValid = false;
            } else {
                solveChain = &commit.ikPrepared;
            }
        }
        commit.ikSolved.clear();
        if (inputsValid) {
            commit.ikSolved =
                _RrSolveSingleChainIk(*solveChain, effector, params);
        }
        if (inputsValid &&
            (commit.ikSolved.size() != c.targetSlots.size() ||
             std::any_of(commit.ikSolved.begin(), commit.ikSolved.end(),
                         [](const RrPointFrame &frame) {
                             return !RrFrameUsable(frame);
                         }))) {
            output.diagnostics.push_back(
                cpath + " failed to solve its joint chain; constraint "
                "passed through atomically");
            inputsValid = false;
        }
        if (inputsValid) {
            for (size_t k = 0; k < c.targetSlots.size(); ++k) {
                const auto found = std::lower_bound(
                    wireCommit.slots.begin(), wireCommit.slots.end(),
                    c.targetSlots[k]);
                if (found == wireCommit.slots.end() ||
                    *found != c.targetSlots[k]) {
                    continue;
                }
                const size_t pos = size_t(found - wireCommit.slots.begin());
                if (pos >= commit.frames.size() ||
                    pos >= commit.present.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no candidate slot";
                    }
                    return false;
                }
                commit.frames[pos] = commit.ikSolved[k];
                commit.present[pos] = 1;
            }
            commit.abandoned = false;
        }
        return finish();
    }

    scratch->recordEveryTarget[size_t(wire.object)] = 0;

    bool sourcesReady = arrays.ok;
    for (size_t k = 0; sourcesReady && k < c.sources.size(); ++k) {
        if (k >= wireCommit.sourceReads.size() ||
            k >= wireCommit.sourceAncestors.size() ||
            k >= c.sourceNatives.size() ||
            k >= c.sourcePaths.size() ||
            k >= commit.sources.size() ||
            k >= arrays.weights.size() ||
            k >= arrays.translationOffsets.size() ||
            k >= arrays.rotationOffsets.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no constraint source";
            }
            return false;
        }
        RrPointFrame frame;
        if (!resolveSource(c.sources[k], wireCommit.sourceReads[k],
                           c.sourceNatives[k],
                           wireCommit.sourceAncestors[k], &frame)) {
            output.diagnostics.push_back(
                cpath + " could not resolve source " +
                program->TextOrEmpty(c.sourcePaths[k]));
            sourcesReady = false;
            break;
        }
        commit.sources[k].frame = frame;
        commit.sources[k].normalizedWeight = arrays.weights[k];
        commit.sources[k].translationOffset =
            arrays.translationOffsets[k];
        commit.sources[k].rotationOffsetDegrees =
            arrays.rotationOffsets[k];
    }
    if (!sourcesReady) {
        output.diagnostics.push_back(
            cpath + " has unusable constraint inputs; constraint passed "
            "through");
        return finish();
    }
    scratch->recordAfter[size_t(wire.object)] = deltaBase < 0 ? 1 : 0;
    const double solveWeight = deltaBase < 0 ? weight : 1.0;
    if (size_t(wireCommit.targetRead) >= store.fin.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no fin version";
        }
        return false;
    }
    const std::vector<RrConstraintSource> &scratchSources =
        commit.sources;
    std::vector<_RrConstraintSource> sources;
    sources.reserve(scratchSources.size());
    for (const RrConstraintSource &s : scratchSources) {
        _RrConstraintSource out;
        out.frame = s.frame;
        out.normalizedWeight = s.normalizedWeight;
        out.translationOffset = s.translationOffset;
        out.rotationOffsetDegrees = s.rotationOffsetDegrees;
        sources.push_back(out);
    }
    const RrPointFrame input =
        store.fin[size_t(wireCommit.targetRead)];
    RrPointFrame candidate = input;
    bool candidateReady = true;
    _RrConstraintAxisMask affect;
    affect.x = program->ReadConstraint(ci, RrConstraintAffectX).boolean;
    affect.y = program->ReadConstraint(ci, RrConstraintAffectY).boolean;
    affect.z = program->ReadConstraint(ci, RrConstraintAffectZ).boolean;
    const std::string ctype = program->TextOrEmpty(c.type);
    const _RrEulerOrder order = _RrEulerOrder(c.order);
    if (ctype == "RigExecPositionConstraint") {
        _RrPositionConstraintParams params;
        params.offset =
            program->ReadConstraint(ci, RrConstraintOffset).vec;
        params.affect = affect;
        params.weight = solveWeight;
        candidate = _RrApplyPositionConstraint(input, sources, params);
    } else if (ctype == "RigExecRotationConstraint") {
        _RrRotationConstraintParams params;
        params.offsetDegrees =
            program->ReadConstraint(ci, RrConstraintOffset).vec;
        params.affect = affect;
        params.rotationOrder = order;
        params.weight = solveWeight;
        candidate = _RrApplyRotationConstraint(input, sources, params);
    } else if (ctype == "RigExecScaleConstraint") {
        _RrScaleConstraintParams params;
        params.offset =
            program->ReadConstraint(ci, RrConstraintOffset).vec;
        params.affect = affect;
        params.weight = solveWeight;
        candidate = _RrApplyScaleConstraint(input, sources, params);
    } else if (ctype == "RigExecParentConstraint") {
        _RrParentConstraintParams params;
        params.translationAxes.x =
            program->ReadConstraint(ci, RrConstraintTX).boolean;
        params.translationAxes.y =
            program->ReadConstraint(ci, RrConstraintTY).boolean;
        params.translationAxes.z =
            program->ReadConstraint(ci, RrConstraintTZ).boolean;
        params.rotationAxes.x =
            program->ReadConstraint(ci, RrConstraintRX).boolean;
        params.rotationAxes.y =
            program->ReadConstraint(ci, RrConstraintRY).boolean;
        params.rotationAxes.z =
            program->ReadConstraint(ci, RrConstraintRZ).boolean;
        params.scaleAxes.x =
            program->ReadConstraint(ci, RrConstraintSX).boolean;
        params.scaleAxes.y =
            program->ReadConstraint(ci, RrConstraintSY).boolean;
        params.scaleAxes.z =
            program->ReadConstraint(ci, RrConstraintSZ).boolean;
        params.rotationOrder = order;
        params.weight = solveWeight;
        candidate = _RrApplyParentConstraint(input, sources, params);
    } else {
        RrVec3d target(0.0);
        double total = 0;
        for (const _RrConstraintSource &source : sources) {
            if (!std::isfinite(source.normalizedWeight) ||
                source.normalizedWeight < 0) {
                output.diagnostics.push_back(
                    cpath + " has an invalid source weight; constraint "
                    "passed through");
                candidateReady = false;
                break;
            }
            target += source.frame.points[0] * source.normalizedWeight;
            total += source.normalizedWeight;
        }
        if (candidateReady && total > 0) {
            target /= total;
            _RrAimConstraintParams params;
            params.localAimVector =
                c.aimVectorAuthored
                    ? program->ReadConstraint(ci,
                                              RrConstraintAimVector).vec
                    : RrVec3d(c.aimAxisFallback[0], c.aimAxisFallback[1],
                              c.aimAxisFallback[2]);
            params.localUpVector =
                program->ReadConstraint(ci, RrConstraintUpVector).vec;
            params.rotationOffsetDegrees =
                program->ReadConstraint(ci,
                                        RrConstraintRotationOffset).vec;
            params.affectRotation = affect;
            params.rotationOrder = order;
            params.weight = solveWeight;
            params.preserveInputUp = c.preserveInputUp;
            const RrVec3d authoredWorldUp =
                program->ReadConstraint(ci,
                                        RrConstraintWorldUpVector).vec;
            const RrVec3d sceneUp(c.sceneUp[0], c.sceneUp[1],
                                  c.sceneUp[2]);
            RrVec3d worldUpStorage(0.0);
            bool haveWorldUp = false;
            const std::string worldUpType =
                program->TextOrEmpty(c.worldUpType);
            if (worldUpType == "sceneUp") {
                worldUpStorage = sceneUp;
                haveWorldUp = true;
            } else if (worldUpType == "vector") {
                worldUpStorage = authoredWorldUp;
                haveWorldUp = true;
            } else if (worldUpType == "objectUp") {
                RrPointFrame upObject;
                if (!c.worldUpObjectNamed) {
                    worldUpStorage = -input.points[0];
                    haveWorldUp = true;
                } else if (!resolveSource(c.worldUpObject,
                                          wireCommit.worldUpRead,
                                          c.worldUpNative,
                                          wireCommit.worldUpAncestors,
                                          &upObject)) {
                    output.diagnostics.push_back(
                        cpath + " could not resolve its world-up object; "
                        "constraint passed through");
                    candidateReady = false;
                } else {
                    worldUpStorage =
                        upObject.points[0] - input.points[0];
                    haveWorldUp = true;
                }
            } else if (worldUpType == "objectRotationUp") {
                if (!c.worldUpObjectNamed) {
                    worldUpStorage = authoredWorldUp;
                    haveWorldUp = true;
                } else {
                    RrPointFrame upObject;
                    if (!resolveSource(c.worldUpObject,
                                       wireCommit.worldUpRead,
                                       c.worldUpNative,
                                       wireCommit.worldUpAncestors,
                                       &upObject)) {
                        output.diagnostics.push_back(
                            cpath +
                            " could not resolve its world-up object; "
                            "constraint passed through");
                        candidateReady = false;
                    }
                    RrMat4d up = _RrIdentity();
                    if (candidateReady &&
                        !RrPointsToMatrix(RrIdentityLandmarks(),
                                          upObject.points, &up)) {
                        output.diagnostics.push_back(
                            cpath + " has a degenerate world-up object");
                        candidateReady = false;
                    } else if (candidateReady) {
                        worldUpStorage = up.ExtractRotation().TransformDir(
                            authoredWorldUp);
                        haveWorldUp = true;
                    }
                }
            }
            if (haveWorldUp) {
                params.worldUpDirection = &worldUpStorage;
            }
            if (candidateReady) {
                candidate =
                    _RrApplyAimConstraint(input, target, params);
            }
        }
    }
    if (candidateReady && deltaBase >= 0) {
        if (size_t(deltaBase) >= scratch->deltaValues.size() ||
            size_t(deltaBase) >= scratch->deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaValues.size() ||
            size_t(deltaBase) >= store.deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaBaseMatrix.size() ||
            size_t(deltaBase) >= store.deltaBaseOk.size() ||
            size_t(deltaBase) >=
                program->geometry->deltaBasePaths.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no delta path";
            }
            return false;
        }
        const RrMat4d &baseMatrix =
            store.deltaBaseMatrix[size_t(deltaBase)];
        RrMat4d solvedMatrix = _RrIdentity();
        if (RrFrameUsable(candidate) &&
            store.deltaBaseOk[size_t(deltaBase)] &&
            std::isfinite(baseMatrix.GetDeterminant()) &&
            baseMatrix.GetDeterminant() != 0.0 &&
            RrPointsToMatrix(RrIdentityLandmarks(), candidate.points,
                             &solvedMatrix)) {
            scratch->deltaValues[size_t(deltaBase)] =
                solvedMatrix * baseMatrix.GetInverse();
            scratch->deltaPresent[size_t(deltaBase)] = 1;
            store.deltaValues[size_t(deltaBase)] =
                solvedMatrix * baseMatrix.GetInverse();
            store.deltaPresent[size_t(deltaBase)] = 1;
        } else {
            output.diagnostics.push_back(
                cpath + " could not measure its delta against " +
                program->TextOrEmpty(
                    program->geometry->deltaBasePaths[size_t(deltaBase)]) +
                "; constraint passed through");
        }
    } else if (candidateReady) {
        if (!RrFrameUsable(candidate)) {
            if (c.target < 0 ||
                size_t(c.target) >= program->slotMeta->paths.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no target slot";
                }
                return false;
            }
            output.diagnostics.push_back(
                cpath + " produced an invalid or degenerate frame for " +
                program->TextOrEmpty(
                    program->slotMeta->paths[size_t(c.target)]) +
                "; constraint passed through");
        } else {
            if (commit.frames.empty() || commit.present.empty()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no candidate slot";
                }
                return false;
            }
            commit.frames[0] = candidate;
            commit.present[0] = 1;
            commit.abandoned = false;
        }
    }
    return finish();
}

} // namespace runtimePoseDetail

} // namespace rigExec
