//
// RigExec arbitrary-length single-chain IK kernel.
//
// This is the pure-math counterpart of FBX's FbxConstraintSingleChainIK.
// It has no USD, Exec, hierarchy, or schema dependencies: callers resolve the
// ordered first-joint-through-end-joint chain and all object relationships
// before invoking it.
//
#ifndef RIGEXEC_MATH_SINGLE_CHAIN_IK_H
#define RIGEXEC_MATH_SINGLE_CHAIN_IK_H

#include "pointFrame.h"

#include <vector>

namespace rigExec {

/// FBX-compatible solver modes.
enum class RigExecSingleChainIkMode {
    RotatePlane,  ///< constrain the solved chain to the pole-defined plane
    SingleChain,  ///< orient the solve plane from the effector; ignore pole/twist
};

/// Parameters for RigExecSolveSingleChainIk.
struct RigExecSingleChainIkParams {
    RigExecSingleChainIkMode mode = RigExecSingleChainIkMode::RotatePlane;

    /// Absolute asset-space pole point. FBX vector mode's XYZ value is a
    /// point in space; object mode is resolved to an object origin by callers.
    GfVec3d pole{0.0, 1.0, 0.0};

    /// RotatePlane-only rotation of the bend plane about the root-to-effector
    /// axis. SingleChain ignores this value, including non-finite values.
    double twistDegrees = 0.0;

    /// Global normalized constraint weight. Finite values are clamped to
    /// [0, 1]. Segment directions are blended on the unit sphere and then
    /// accumulated, so authored segment lengths remain exact at every weight.
    double weight = 1.0;
};

/// Solves an ordered joint chain [firstJoint, ..., endJoint].
///
/// The first joint origin remains fixed. Every distance between consecutive
/// joint origins is measured from currentFrames and preserved. A reachable
/// effector origin is reached by deterministic FABRIK; a goal beyond full
/// extension returns the chain aimed at the goal at its maximum reach.
/// RotatePlane solves in the plane spanned by the root-to-goal direction and
/// absolute pole point, rotated by twist; it does not consume effector
/// orientation. SingleChain derives its plane from effector orientation and
/// ignores pole and twist. Each fully weighted non-end frame aims its X axis
/// at the next solved joint while retaining its input handle lengths,
/// handedness, and a transported stable up direction. The SingleChain end
/// takes effector orientation; the RotatePlane end transports its input
/// orientation with the terminal segment.
///
/// Invalid, non-finite, flagged-degenerate, or zero-length input frames and
/// chain segments fail atomically: the input vector is returned with every
/// frame marked RigExecPointFrameDegenerate. A one-frame input is likewise
/// flagged; an empty input necessarily returns empty. As with the other
/// constraint kernels, finite weight <= 0 is an exact early pass-through and
/// does not inspect dormant chain, effector, pole, or twist inputs.
std::vector<RigExecPointFrame> RigExecSolveSingleChainIk(
    const std::vector<RigExecPointFrame> &currentFrames,
    const RigExecPointFrame &effectorFrame,
    const RigExecSingleChainIkParams &params = {});

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SINGLE_CHAIN_IK_H
