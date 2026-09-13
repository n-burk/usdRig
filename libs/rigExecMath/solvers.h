//
// RigExec solver kernels (spec §4.1 solver schemas, §5 math).
//
// Pure, deterministic, stateless functions consumed by the OpenExec
// computation callbacks. All frames are in rig-common space.
//
#ifndef RIGEXEC_MATH_SOLVERS_H
#define RIGEXEC_MATH_SOLVERS_H

#include "dualQuat.h"
#include "pointFrame.h"

#include <optional>
#include <string>
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
/// twistTurns adds signed revolutions to the principal endpoint twist. It
/// carries winding information that endpoint frames alone cannot represent;
/// fractional values support continuous animation through complete turns.
std::vector<RigExecPointFrame> RigExecDistributeTwist(
    const RigExecPointFrame &start,
    const RigExecPointFrame &end,
    const std::array<GfVec3d, 4> &startRest,
    const std::array<GfVec3d, 4> &endRest,
    const std::vector<double> &weights,
    double twistTurns = 0.0);

/// Weighted matrix movement kernel (RigExecMatrixMover, spec §7.4):
///   p' = q + w (T q - q), 0 <= w <= 1,
/// applied per point of an exact native points value.
GfVec3d RigExecApplyWeightedMatrix(
    const GfVec3d &point, const GfMatrix4d &transform, double weight);

/// Skinning influence layout (RigExecSkinMover): UsdSkel's jointIndices /
/// jointWeights arrangement over a table of influence matrices, elementSize
/// slots per point in point order.
///
/// Every skinning method starts from the same gather -- for point i and
/// slot k, Transform(i, k) and Weight(i, k) applied to that point's rest
/// position -- and differs only in how the gathered terms accumulate. The
/// layout is therefore shared, and each method is one loop over it:
/// RigExecApplyLinearBlendSkin below, and the dual-quaternion path that
/// libs/rigExecMath/dualQuat.h is to supply.
///
/// A view, not an owner: the pointers must outlive the call. Validate()
/// checks the shape once so the per-point accessors can index unchecked.
struct RigExecSkinLayout {
    const GfMatrix4d *transforms = nullptr;  ///< one per influence
    size_t transformCount = 0;
    const int *indices = nullptr;            ///< pointCount * elementSize
    const float *weights = nullptr;          ///< parallel to indices
    size_t indexCount = 0;                   ///< length of indices/weights
    size_t elementSize = 0;                  ///< influence slots per point
    size_t pointCount = 0;

    const GfMatrix4d &Transform(size_t point, size_t slot) const {
        return transforms[indices[point * elementSize + slot]];
    }
    float Weight(size_t point, size_t slot) const {
        return weights[point * elementSize + slot];
    }

    /// Shape and range check: at least one influence, elementSize >= 1,
    /// indexCount == pointCount * elementSize, every index in
    /// [0, transformCount), every weight finite and non-negative, and every
    /// transform finite and affine. Fills \p error on failure.
    bool Validate(std::string *error = nullptr) const;
};

/// Linear blend skinning of point \p i (scalar reference):
///   p' = (1 - sum_k w_k) p + sum_k w_k (T_k p).
/// With weights summing to one this is exactly sum_k w_k T_k p. The
/// complement of the weight sum stays with the rest point, so a partially
/// weighted point is held in place in proportion to its shortfall rather
/// than pulled toward the origin (which is what the bare sum does), and an
/// all-zero-weight point is left where it was. A sum above one is used as
/// authored: the point is over-driven, not renormalized.
GfVec3d RigExecApplyLinearBlendSkin(
    const GfVec3d &point, const RigExecSkinLayout &layout, size_t i);

/// Whole-array form of the above over \p layout.pointCount points. The
/// caller has validated the layout. in/out may alias.
void RigExecApplyLinearBlendSkin(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout);

/// Dual-quaternion skinning over the same layout (scale-aware DQS from
/// dualQuat.h; scalar, there is no SIMD path for it yet):
///   p' = (p * S_blend) rotated and translated by normalise(sum_k w_k dq_k)
/// with S_blend = sum_k w_k S_k, every influence split once per evaluation
/// into a pre-rotation stretch S_k and a unit dual quaternion dq_k by
/// RigExecScaledDualQuatFromMatrix, so non-uniform scale (squash and
/// stretch) survives and the rotational part is length-preserving.
///
/// Weight shortfall: the complement 1 - sum_k w_k enters the blend as an
/// extra IDENTITY influence, the exact analogue of the rest-retaining rule
/// of RigExecApplyLinearBlendSkin. With weights summing to one the extra
/// influence has weight zero and is skipped, so a fully weighted point is
/// the plain DQS blend; a partially weighted point is pulled only part of
/// the way from rest along the shortest arc (a lone influence at weight w
/// rotates the point by w of its angle, not to w of its chord); an
/// all-zero-weight point stays exactly where it was; and for pure
/// translations the result is identical to the linear rule, complement
/// included. A sum above one is used as authored, as for the linear
/// kernel: the identity enters with a negative weight, which extrapolates
/// past the influences rather than renormalising.
///
/// Sign-correction reference: the blend's reference is its first non-zero
/// input, and the gather puts the LARGEST-weight slot first (ties to the
/// earliest slot), which is UsdSkel's pivot choice; the remaining slots
/// follow in authored order and the complement last. For one or two
/// influences this is bit-neutral, so only points with three or more
/// influences spanning more than 90 degrees of quaternion space can tell
/// the two reference conventions apart.
///
/// Returns false, leaving \p out unspecified, if any point's blend is
/// degenerate (RigExecBlendScaledDualQuats' rules; with non-negative
/// weights summing to at most one the sign-corrected sum is always
/// constructive, so this is only reachable through an over-driven sum
/// that cancels the identity). in/out may alias.
bool RigExecApplyDualQuatSkin(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout);

/// The hoisted half of RigExecApplyDualQuatSkin: one RigExecScaledDualQuat
/// per influence in layout order, then one trailing identity entry at
/// index layout.transformCount that carries the weight complement. Built
/// once per evaluation, never per point.
std::vector<RigExecScaledDualQuat> RigExecSkinDualQuatPalette(
    const RigExecSkinLayout &layout);

/// Array form against a palette the CALLER built with
/// RigExecSkinDualQuatPalette, for a caller that splits the influences once
/// per evaluation and then skins several ranges of one array against them --
/// which is what a chunked skin step does. \p palette must hold
/// \p paletteSize entries built from the same influence table \p layout
/// names; the three-argument form above is this one with a palette built on
/// the spot. in/out may alias.
bool RigExecApplyDualQuatSkin(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout,
    const RigExecScaledDualQuat *palette, size_t paletteSize);

/// Single-point form of RigExecApplyDualQuatSkin against a palette from
/// RigExecSkinDualQuatPalette (scalar reference; allocates, so the array
/// form does not call it per point). Returns false on a degenerate blend.
bool RigExecApplyDualQuatSkin(
    const GfVec3d &point, const RigExecScaledDualQuat *palette,
    size_t paletteSize, const RigExecSkinLayout &layout, size_t i,
    GfVec3d *out);

/// Euler application/decomposition order used by the FBX-equivalent
/// constraints.  The named axes are applied from left to right in the same
/// row-vector convention as UsdGeom's rotate-order xform ops.
enum class RigExecEulerOrder { XYZ, XZY, YXZ, YZX, ZXY, ZYX };

/// Independent component selection.  Disabled components are copied from
/// the input frame; constraint enable/disable remains evaluator-side.
struct RigExecConstraintAxisMask {
    bool x = true;
    bool y = true;
    bool z = true;
};

/// One FBX-equivalent constraint source in asset space.  Sources use the
/// identity landmark frame.  normalizedWeight is a non-negative relative
/// source weight; kernels normalize the usable set independently of the
/// constraint's global weight.  Parent constraints apply the per-source
/// translation/rotation offsets; the other kernels use their global offsets.
struct RigExecConstraintSource {
    RigExecPointFrame frame;
    double normalizedWeight = 1.0;
    GfVec3d translationOffset{0, 0, 0};
    GfVec3d rotationOffsetDegrees{0, 0, 0};
};

struct RigExecPositionConstraintParams {
    GfVec3d offset{0, 0, 0};
    RigExecConstraintAxisMask affect;
    double weight = 1.0;
};

struct RigExecRotationConstraintParams {
    GfVec3d offsetDegrees{0, 0, 0};
    RigExecConstraintAxisMask affect;
    RigExecEulerOrder rotationOrder = RigExecEulerOrder::XYZ;
    double weight = 1.0;
};

struct RigExecScaleConstraintParams {
    /// FBX scale-constraint offset is additive and defaults to zero.
    GfVec3d offset{0, 0, 0};
    RigExecConstraintAxisMask affect;
    double weight = 1.0;
};

struct RigExecParentConstraintParams {
    RigExecConstraintAxisMask translationAxes;
    RigExecConstraintAxisMask rotationAxes;
    /// FBX Parent constraints do not affect scale unless explicitly enabled.
    RigExecConstraintAxisMask scaleAxes{false, false, false};
    RigExecEulerOrder rotationOrder = RigExecEulerOrder::XYZ;
    double weight = 1.0;
};

/// Deterministic multi-source FBX-equivalent constraints over asset-space
/// frames whose reference landmarks are [O, X, Y, Z].  A zero global weight
/// or zero total source weight is an exact pass-through.  Non-finite inputs,
/// invalid Euler orders, and singular source frames fail atomically by
/// returning the input with RigExecPointFrameDegenerate set.  Rotation uses
/// shortest per-Euler-component deltas from the first positive-weight source,
/// so the fully constrained result depends only on the sources while retaining
/// deterministic FBX source order. Input shear is retained by every kernel.
RigExecPointFrame RigExecApplyPositionConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecPositionConstraintParams &params);

RigExecPointFrame RigExecApplyRotationConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecRotationConstraintParams &params);

RigExecPointFrame RigExecApplyScaleConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecScaleConstraintParams &params);

RigExecPointFrame RigExecApplyParentConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecParentConstraintParams &params);

/// General FBX-equivalent aim parameters.  localAimVector/localUpVector are
/// arbitrary non-collinear directions in the constrained object's local
/// orientation.  When worldUpDirection is present it pins roll by projection
/// around the solved aim direction.  Without a direction, preserveInputUp
/// selects the legacy roll-preserving behavior; false implements FBX
/// worldUpType=None as the minimum swing with no roll correction.  The final
/// orientation offset and masked/global-weight blend use rotationOrder.
struct RigExecAimConstraintParams {
    GfVec3d localAimVector{1, 0, 0};
    GfVec3d localUpVector{0, 1, 0};
    std::optional<GfVec3d> worldUpDirection;
    bool preserveInputUp = true;
    GfVec3d rotationOffsetDegrees{0, 0, 0};
    RigExecConstraintAxisMask affectRotation;
    RigExecEulerOrder rotationOrder = RigExecEulerOrder::XYZ;
    double weight = 1.0;
};

RigExecPointFrame RigExecApplyAimConstraint(
    const RigExecPointFrame &input, const GfVec3d &targetPoint,
    const RigExecAimConstraintParams &params);

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
