//
// RigExec arbitrary-length single-chain IK kernel.
//
#include "singleChainIk.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rigExec {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr int kFabrikIterations = 128;

bool
_IsFinite(const GfVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}

bool
_IsFinite(const RigExecPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const GfVec3d &point : frame.points) {
        if (!_IsFinite(point)) {
            return false;
        }
    }
    return true;
}

std::vector<RigExecPointFrame>
_Degenerate(const std::vector<RigExecPointFrame> &frames)
{
    std::vector<RigExecPointFrame> failed = frames;
    for (RigExecPointFrame &frame : failed) {
        frame.flags |= RigExecPointFrameDegenerate;
    }
    return failed;
}

GfVec3d
_ProjectPerpendicular(const GfVec3d &v, const GfVec3d &axis)
{
    return v - axis * GfDot(v, axis);
}

GfVec3d
_WorldAxisLeastParallel(const GfVec3d &axis)
{
    const GfVec3d world[3] = {
        GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(0.0, 1.0, 0.0),
        GfVec3d(0.0, 0.0, 1.0),
    };
    int best = 0;
    double bestAlignment = std::abs(GfDot(world[0], axis));
    for (int i = 1; i < 3; ++i) {
        const double alignment = std::abs(GfDot(world[i], axis));
        if (alignment < bestAlignment) {
            best = i;
            bestAlignment = alignment;
        }
    }
    return world[best];
}

GfVec3d
_StablePerpendicular(
    const GfVec3d &axis, const GfVec3d &first, const GfVec3d &second,
    double epsilon)
{
    auto normalized = [](const GfVec3d &candidate) {
        const double length = candidate.GetLength();
        return std::isfinite(length) && length > 0.0
            ? candidate / length
            : GfVec3d(0.0);
    };
    GfVec3d result = _ProjectPerpendicular(normalized(first), axis);
    if (result.GetLength() <= epsilon) {
        result = _ProjectPerpendicular(normalized(second), axis);
    }
    if (result.GetLength() <= epsilon) {
        result = _ProjectPerpendicular(_WorldAxisLeastParallel(axis), axis);
    }
    return result.GetNormalized();
}

GfVec3d
_Rotate(const GfVec3d &v, const GfVec3d &unitAxis, double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return v * c + GfCross(unitAxis, v) * s +
           unitAxis * GfDot(unitAxis, v) * (1.0 - c);
}

/// Applies the deterministic shortest rotation from fromAxis to toAxis to v.
/// The fallback supplies the otherwise-underdetermined 180-degree axis.
GfVec3d
_RotateToward(
    const GfVec3d &v, const GfVec3d &fromAxis, const GfVec3d &toAxis,
    double weight, const GfVec3d &fallback, double epsilon)
{
    const GfVec3d from = fromAxis.GetNormalized();
    const GfVec3d to = toAxis.GetNormalized();
    const double dot = std::min(std::max(GfDot(from, to), -1.0), 1.0);
    GfVec3d rotationAxis = GfCross(from, to);
    const double sinAngle = rotationAxis.GetLength();
    double angle = 0.0;
    if (sinAngle > epsilon) {
        rotationAxis /= sinAngle;
        angle = std::atan2(sinAngle, dot);
    } else if (dot < 0.0) {
        rotationAxis = _StablePerpendicular(
            from, fallback, GfVec3d(0.0), epsilon);
        angle = kPi;
    } else {
        return v;
    }
    return _Rotate(v, rotationAxis, angle * weight);
}

GfVec3d
_BlendDirection(
    const GfVec3d &from, const GfVec3d &to, double weight,
    const GfVec3d &fallback, double epsilon)
{
    GfVec3d result = _RotateToward(
        from, from, to, weight, fallback, epsilon);
    const double length = result.GetLength();
    if (length <= epsilon || !_IsFinite(result)) {
        return from.GetNormalized();
    }
    return result / length;
}

struct _FrameBasis {
    GfVec3d x;
    GfVec3d up;
    double xLength = 0.0;
    double yLength = 0.0;
    double zLength = 0.0;
    double handedness = 1.0;
};

bool
_ExtractBasis(
    const RigExecPointFrame &frame, double epsilon, _FrameBasis *basis)
{
    const GfVec3d x = frame.X() - frame.Origin();
    const GfVec3d y = frame.Y() - frame.Origin();
    const GfVec3d z = frame.Z() - frame.Origin();
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

    const GfVec3d unitX = x / xLength;
    GfVec3d up = _ProjectPerpendicular(y / yLength, unitX);
    if (up.GetLength() <= epsilon) {
        return false;
    }
    up.Normalize();

    // Use normalized input axes so this test is scale independent.
    const double determinant = GfDot(
        GfCross(x / xLength, y / yLength), z / zLength);
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

RigExecPointFrame
_FrameFromBasis(
    const _FrameBasis &basis, const GfVec3d &origin,
    const GfVec3d &unitX, const GfVec3d &unitUp)
{
    const GfVec3d side =
        basis.handedness * GfCross(unitX, unitUp).GetNormalized();
    RigExecPointFrame frame;
    frame.points[0] = origin;
    frame.points[1] = origin + unitX * basis.xLength;
    frame.points[2] = origin + unitUp * basis.yLength;
    frame.points[3] = origin + side * basis.zLength;
    frame.flags = RigExecPointFrameValid;
    if (basis.handedness < 0.0) {
        frame.flags |= RigExecPointFrameReflected;
    }
    return frame;
}

_FrameBasis
_AimedBasis(
    const _FrameBasis &basis, const GfVec3d &unitAim, double epsilon)
{
    GfVec3d up = _RotateToward(
        basis.up, basis.x, unitAim, 1.0, basis.up, epsilon);
    up = _ProjectPerpendicular(up, unitAim);
    if (up.GetLength() <= epsilon) {
        up = _StablePerpendicular(
            unitAim, basis.up, basis.x, epsilon);
    } else {
        up.Normalize();
    }
    _FrameBasis aimed = basis;
    aimed.x = unitAim;
    aimed.up = up;
    return aimed;
}

_FrameBasis
_TransportedBasis(
    const _FrameBasis &basis, const GfVec3d &fromDirection,
    const GfVec3d &toDirection, double epsilon)
{
    _FrameBasis transported = basis;
    transported.x = _RotateToward(
        basis.x, fromDirection, toDirection, 1.0, basis.up, epsilon);
    transported.x.Normalize();
    transported.up = _RotateToward(
        basis.up, fromDirection, toDirection, 1.0, basis.up, epsilon);
    transported.up = _ProjectPerpendicular(
        transported.up, transported.x);
    if (transported.up.GetLength() <= epsilon) {
        transported.up = _StablePerpendicular(
            transported.x, basis.up, basis.x, epsilon);
    } else {
        transported.up.Normalize();
    }
    return transported;
}

double
_SignedAngle(const GfVec3d &from, const GfVec3d &to,
             const GfVec3d &unitAxis)
{
    return std::atan2(
        GfDot(unitAxis, GfCross(from, to)),
        std::min(std::max(GfDot(from, to), -1.0), 1.0));
}

RigExecPointFrame
_BlendEndFrame(
    const _FrameBasis &input, const _FrameBasis &effector,
    const GfVec3d &origin, double weight, double epsilon)
{
    const GfVec3d x = _BlendDirection(
        input.x, effector.x, weight, input.up, epsilon);

    GfVec3d carriedAtTarget = _RotateToward(
        input.up, input.x, effector.x, 1.0, input.up, epsilon);
    carriedAtTarget = _ProjectPerpendicular(carriedAtTarget, effector.x);
    if (carriedAtTarget.GetLength() <= epsilon) {
        carriedAtTarget = _StablePerpendicular(
            effector.x, input.up, effector.up, epsilon);
    } else {
        carriedAtTarget.Normalize();
    }
    const double roll =
        _SignedAngle(carriedAtTarget, effector.up, effector.x);

    GfVec3d up = _RotateToward(
        input.up, input.x, effector.x, weight, input.up, epsilon);
    up = _ProjectPerpendicular(up, x);
    if (up.GetLength() <= epsilon) {
        up = _StablePerpendicular(x, input.up, effector.up, epsilon);
    } else {
        up.Normalize();
    }
    up = _Rotate(up, x, roll * weight).GetNormalized();
    return _FrameFromBasis(input, origin, x, up);
}

GfVec3d
_DirectionOrFallback(
    const GfVec3d &direction, const GfVec3d &fallback,
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
    return GfVec3d(1.0, 0.0, 0.0);
}

std::vector<GfVec3d>
_SolvePositions(
    const std::vector<GfVec3d> &original,
    const std::vector<double> &lengths,
    const std::vector<_FrameBasis> &bases,
    const _FrameBasis &effectorBasis,
    const GfVec3d &goal,
    const RigExecSingleChainIkParams &params,
    double positionEpsilon, double angularEpsilon)
{
    const size_t count = original.size();
    const GfVec3d root = original.front();
    const double totalLength = [&]() {
        double total = 0.0;
        for (double length : lengths) total += length;
        return total;
    }();

    GfVec3d rootToGoal = goal - root;
    const double goalDistance = rootToGoal.GetLength();
    GfVec3d solveAxis;
    if (goalDistance > positionEpsilon) {
        solveAxis = rootToGoal / goalDistance;
    } else {
        solveAxis = original.back() - root;
        if (solveAxis.GetLength() <= positionEpsilon) {
            solveAxis = original[1] - root;
        }
        solveAxis = _DirectionOrFallback(
            solveAxis, bases.front().x, positionEpsilon, angularEpsilon);
    }

    // A far-unreachable target has one unique closest solution: full
    // extension down the root-to-goal ray.
    if (goalDistance >= totalLength - positionEpsilon &&
        goalDistance > positionEpsilon) {
        std::vector<GfVec3d> extended(count);
        extended[0] = root;
        for (size_t i = 0; i < lengths.size(); ++i) {
            extended[i + 1] = extended[i] + solveAxis * lengths[i];
        }
        return extended;
    }

    std::vector<GfVec3d> positions = original;
    GfVec3d planeUp;
    GfVec3d planeNormal;
    if (params.mode == RigExecSingleChainIkMode::RotatePlane) {
        GfVec3d poleOffset = params.pole - root;
        if (poleOffset.GetLength() <= positionEpsilon) {
            poleOffset = GfVec3d(0.0);
        }
        planeUp = _StablePerpendicular(
            solveAxis, poleOffset, bases.front().up, angularEpsilon);
        planeUp = _Rotate(
            planeUp, solveAxis, params.twistDegrees * (kPi / 180.0));
        planeUp.Normalize();
    } else {
        // FBX SingleChain uses effector rotation, rather than a pole vector,
        // to orient the chain. The effector up axis fixes that solve plane.
        planeUp = _StablePerpendicular(
            solveAxis, effectorBasis.up, effectorBasis.x,
            angularEpsilon);
    }
    planeNormal = GfCross(solveAxis, planeUp).GetNormalized();

    // Project the initial bend into the selected plane and choose its
    // positive half-plane. FABRIK remains planar because every seed,
    // fallback, and anchor is planar.
    for (size_t i = 1; i + 1 < count; ++i) {
        const GfVec3d relative = positions[i] - root;
        const double along = GfDot(relative, solveAxis);
        const double towardPlane = std::abs(GfDot(relative, planeUp));
        positions[i] =
            root + solveAxis * along + planeUp * towardPlane;
    }

    // FABRIK cannot leave an exactly straight singular seed when the target
    // lies on the same ray but inside maximum reach. Introduce a deterministic
    // scale-relative arc using pole-up for RotatePlane and transported input
    // up for SingleChain.
    double maxOffAxis = 0.0;
    for (size_t i = 1; i + 1 < count; ++i) {
        maxOffAxis = std::max(
            maxOffAxis,
            _ProjectPerpendicular(positions[i] - root, solveAxis).GetLength());
    }
    if (count > 2 && maxOffAxis <= positionEpsilon * 8.0 &&
        goalDistance < totalLength - positionEpsilon) {
        const GfVec3d bend = planeUp;
        double distance = 0.0;
        const double amplitude = std::max(
            totalLength * 0.05, positionEpsilon * 16.0);
        for (size_t i = 1; i + 1 < count; ++i) {
            distance += lengths[i - 1];
            positions[i] +=
                bend * (amplitude * std::sin(kPi * distance / totalLength));
        }
    }

    std::vector<GfVec3d> fallback(lengths.size());
    for (size_t i = 0; i < lengths.size(); ++i) {
        GfVec3d direction = positions[i + 1] - positions[i];
        direction -= planeNormal * GfDot(direction, planeNormal);
        fallback[i] = _DirectionOrFallback(
            direction, solveAxis, positionEpsilon, angularEpsilon);
    }

    std::vector<GfVec3d> best = positions;
    double bestError = std::numeric_limits<double>::infinity();
    const double tolerance = std::max(
        positionEpsilon * 8.0, totalLength * 1e-12);
    for (int iteration = 0; iteration < kFabrikIterations; ++iteration) {
        positions.back() = goal;

        // Backward reaching: place each parent at its authored segment
        // distance from the already-positioned child.
        for (size_t i = count - 1; i-- > 0;) {
            const GfVec3d direction = _DirectionOrFallback(
                positions[i] - positions[i + 1], -fallback[i],
                positionEpsilon, angularEpsilon);
            positions[i] = positions[i + 1] + direction * lengths[i];
        }

        // Forward reaching: restore the fixed root and authored lengths.
        positions[0] = root;
        for (size_t i = 0; i < lengths.size(); ++i) {
            const GfVec3d direction = _DirectionOrFallback(
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

}  // namespace

std::vector<RigExecPointFrame>
RigExecSolveSingleChainIk(
    const std::vector<RigExecPointFrame> &currentFrames,
    const RigExecPointFrame &effectorFrame,
    const RigExecSingleChainIkParams &params)
{
    if (!std::isfinite(params.weight)) {
        return _Degenerate(currentFrames);
    }
    const double weight =
        std::min(std::max(params.weight, 0.0), 1.0);
    // A zero-weight constraint is dormant. Match the other constraint
    // kernels by not inspecting disconnected or malformed solve inputs.
    if (weight <= 0.0) {
        return currentFrames;
    }
    if (currentFrames.size() < 2) {
        return _Degenerate(currentFrames);
    }
    if (params.mode != RigExecSingleChainIkMode::RotatePlane &&
        params.mode != RigExecSingleChainIkMode::SingleChain) {
        return _Degenerate(currentFrames);
    }
    if (!_IsFinite(effectorFrame.Origin())) {
        return _Degenerate(currentFrames);
    }
    // SingleChain deliberately does not validate pole or twist: FBX semantics
    // say this mode ignores both. RotatePlane consumes both but only consumes
    // the effector origin, never its orientation.
    if (params.mode == RigExecSingleChainIkMode::RotatePlane &&
        (!_IsFinite(params.pole) || !std::isfinite(params.twistDegrees))) {
        return _Degenerate(currentFrames);
    }
    for (const RigExecPointFrame &frame : currentFrames) {
        if (!_IsFinite(frame)) {
            return _Degenerate(currentFrames);
        }
    }

    std::vector<GfVec3d> original(currentFrames.size());
    std::vector<double> lengths(currentFrames.size() - 1);
    double segmentScale = 0.0;
    for (size_t i = 0; i < currentFrames.size(); ++i) {
        original[i] = currentFrames[i].Origin();
        if (i + 1 < currentFrames.size()) {
            lengths[i] =
                (currentFrames[i + 1].Origin() - original[i]).GetLength();
            segmentScale = std::max(segmentScale, lengths[i]);
        }
    }
    if (!std::isfinite(segmentScale) || segmentScale <= 0.0) {
        return _Degenerate(currentFrames);
    }
    const double positionEpsilon = segmentScale * 1e-10;
    constexpr double angularEpsilon = 1e-12;

    std::vector<_FrameBasis> bases(currentFrames.size());
    for (size_t i = 0; i < currentFrames.size(); ++i) {
        if (!_ExtractBasis(currentFrames[i], angularEpsilon, &bases[i])) {
            return _Degenerate(currentFrames);
        }
    }
    _FrameBasis effectorBasis = bases.back();
    if (params.mode == RigExecSingleChainIkMode::SingleChain) {
        if (!_IsFinite(effectorFrame) ||
            !_ExtractBasis(
                effectorFrame, angularEpsilon, &effectorBasis)) {
            return _Degenerate(currentFrames);
        }
    }
    for (double length : lengths) {
        if (!std::isfinite(length) || length <= positionEpsilon) {
            return _Degenerate(currentFrames);
        }
    }

    const std::vector<GfVec3d> solved = _SolvePositions(
        original, lengths, bases, effectorBasis, effectorFrame.Origin(),
        params, positionEpsilon, angularEpsilon);
    if (solved.size() != currentFrames.size()) {
        return _Degenerate(currentFrames);
    }

    // Construct the fully solved orientations first, then blend from the
    // authored frame orientations. This makes the limit as weight approaches
    // zero continuous even when a frame's local X is not its child direction.
    std::vector<_FrameBasis> solvedBases = bases;
    for (size_t i = 0; i + 1 < solvedBases.size(); ++i) {
        const GfVec3d solvedSegment = solved[i + 1] - solved[i];
        const GfVec3d aim = solvedSegment / solvedSegment.GetLength();
        solvedBases[i] = _AimedBasis(bases[i], aim, angularEpsilon);
    }
    if (params.mode == RigExecSingleChainIkMode::SingleChain) {
        solvedBases.back() = effectorBasis;
    } else {
        const GfVec3d currentTerminalSegment =
            original.back() - original[original.size() - 2];
        const GfVec3d currentTerminal =
            currentTerminalSegment / currentTerminalSegment.GetLength();
        const GfVec3d solvedTerminalSegment =
            solved.back() - solved[solved.size() - 2];
        const GfVec3d solvedTerminal =
            solvedTerminalSegment / solvedTerminalSegment.GetLength();
        solvedBases.back() = _TransportedBasis(
            bases.back(), currentTerminal, solvedTerminal, angularEpsilon);
    }

    std::vector<RigExecPointFrame> result(currentFrames.size());
    if (weight >= 1.0) {
        // `solved` plus `solvedBases` is the full-strength candidate.  Return
        // it directly: rotating toward it again with weight 1 and
        // re-accumulating normalized segment directions is mathematically
        // equivalent but is not bit-exact at the endpoint.
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = _FrameFromBasis(
                bases[i], solved[i], solvedBases[i].x, solvedBases[i].up);
        }
    } else {
        // Blend directions, not joint positions. Re-accumulating the unchanged
        // segment lengths is what makes partial global weights remain a valid
        // rigid chain rather than stretching between two world-space poses.
        std::vector<GfVec3d> blended(currentFrames.size());
        blended[0] = original[0];
        for (size_t i = 0; i < lengths.size(); ++i) {
            const GfVec3d currentDirection =
                (original[i + 1] - original[i]) / lengths[i];
            const GfVec3d solvedSegment = solved[i + 1] - solved[i];
            const GfVec3d solvedDirection =
                solvedSegment / solvedSegment.GetLength();
            const GfVec3d direction = _BlendDirection(
                currentDirection, solvedDirection, weight, bases[i].up,
                angularEpsilon);
            blended[i + 1] = blended[i] + direction * lengths[i];
        }
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = _BlendEndFrame(
                bases[i], solvedBases[i], blended[i], weight,
                angularEpsilon);
        }
    }

    for (const RigExecPointFrame &frame : result) {
        if (!_IsFinite(frame)) {
            return _Degenerate(currentFrames);
        }
    }
    return result;
}

}  // namespace rigExec
