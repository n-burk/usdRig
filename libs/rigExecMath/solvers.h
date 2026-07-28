//
// RigExec solver kernels (spec §4.1 solver schemas, §5 math).
//
// Pure, deterministic, stateless functions consumed by the OpenExec
// computation callbacks. All frames are in rig-common space.
//
#ifndef RIGEXEC_MATH_SOLVERS_H
#define RIGEXEC_MATH_SOLVERS_H

#include "pointFrame.h"

#include <vector>

namespace rigExec {

/// One FK chain element: a control's rest/pose landmark sets and the
/// index of its parent within the chain (-1 for the chain root).
struct RigExecFkChainElement {
    std::array<GfVec3d, 4> restPoints;
    std::array<GfVec3d, 4> posePoints;
    int parentIndex = -1;
};

/// Applies control frames to a rest hierarchy (RigExecFkChain).
///
/// Each control's own rest->pose affine map A_i composes with its parent's
/// accumulated map: W_i = W_parent ∘ A_i (child motion is carried into the
/// parent's moved space). The returned frame i is W_i applied to the
/// control's rest landmarks. Elements must be topologically ordered
/// (parentIndex < i).
std::vector<RigExecPointFrame> RigExecSolveFkChain(
    const std::vector<RigExecFkChainElement> &elements);

/// Two-bone IK parameters (RigExecTwoBoneIk, spec §4.5 example).
struct RigExecTwoBoneIkParams {
    double upperLength = 1.0;
    double lowerLength = 1.0;
    double stretch = 1.0;        ///< inputs:stretch in [0,1]: amount of
                                 ///< stretching permitted beyond full reach
    double softness = 0.0;       ///< inputs:softness: soft-reach distance
    double preferredBendRadians = 0.0;  ///< fallback bend plane when the
                                        ///< pole degenerates
    // stretchPolicy: v0.1 implements "uniformSegments".
    // unreachablePolicy: v0.1 implements "clampWithSoftness".
};

/// Analytic two-bone IK with pole vector (RigExecTwoBoneIk).
///
/// rootFrame supplies the chain root origin and reference orientation;
/// effectorFrame supplies the goal position/orientation; poleFrame's origin
/// defines the bend plane. rest[0..2] are the rest landmark sets of the
/// root/mid/end results. Returns three frames: root, mid, end. End copies
/// the effector orientation at the solved end position.
std::array<RigExecPointFrame, 3> RigExecSolveTwoBoneIk(
    const RigExecPointFrame &rootFrame,
    const RigExecPointFrame &effectorFrame,
    const RigExecPointFrame &poleFrame,
    const std::array<std::array<GfVec3d, 4>, 3> &restPoints,
    const RigExecTwoBoneIkParams &params);

/// Rotation blend mode for RigExecBlendPointFrames.
enum class RigExecRotationBlend {
    ShortestArc,  ///< quaternion slerp with hemisphere alignment
};

/// Scale blend mode for RigExecBlendPointFrames.
enum class RigExecScaleBlend {
    Log,     ///< per-axis blend of log(scale)
    Linear,  ///< per-axis linear blend
};

/// Blends two frames sharing one rest landmark set (RigExecBlendPointFrames).
/// weight 0 returns a; weight 1 returns b. Translation lerps; rotation
/// slerps shortest-arc; scale blends per the mode; shear lerps linearly.
RigExecPointFrame RigExecBlendFrames(
    const RigExecPointFrame &a,
    const RigExecPointFrame &b,
    const std::array<GfVec3d, 4> &restPoints,
    double weight,
    RigExecRotationBlend rotationBlend = RigExecRotationBlend::ShortestArc,
    RigExecScaleBlend scaleBlend = RigExecScaleBlend::Log);

/// Distributes twist between two frames over N samples
/// (RigExecTwistDistribution). weights[k] in [0,1] positions sample k
/// between start (0) and end (1): origins lerp, swing slerps, and the
/// aim-axis twist angle interpolates linearly after swing/twist
/// decomposition (deterministic minimum-energy distribution).
///
/// v0.1 limitation: the twist angle is the principal angle in (-pi, pi]
/// derived from the endpoint orientations. Multi-revolution ("unwrapped")
/// twist is not derivable from two frames alone and requires an explicit
/// wrap-count input, which the current schema does not author.
std::vector<RigExecPointFrame> RigExecDistributeTwist(
    const RigExecPointFrame &start,
    const RigExecPointFrame &end,
    const std::array<GfVec3d, 4> &startRest,
    const std::array<GfVec3d, 4> &endRest,
    const std::vector<double> &weights);

/// Weighted matrix movement kernel (RigExecMatrixMover, spec §7.4):
///   p' = q + w (T q - q), 0 <= w <= 1,
/// applied per point of an exact native points value.
GfVec3d RigExecApplyWeightedMatrix(
    const GfVec3d &point, const GfMatrix4d &transform, double weight);

/// Aim-constraint kernel (RigExecAimConstraint): blends the authored aim
/// landmark's direction toward the target while preserving the origin,
/// handle lengths, and the input frame's handedness; the input up
/// direction is re-projected against the new aim (preserveInputUp).
/// The weight is validated finite and clamped to [0, 1].
/// aimLandmarkIndex is 1..3 per the constraint's authored rigExec:aimAxis.
RigExecPointFrame RigExecApplyAimConstraint(
    const RigExecPointFrame &input, const GfVec3d &targetOrigin,
    double weight, int aimLandmarkIndex);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SOLVERS_H
