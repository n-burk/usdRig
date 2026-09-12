//
// RigExec spline-IK spine kernel.
//
// A control-driven counterpart of Maya's ikSplineSolver as the squarebit
// biped uses it for the spine and neck (core/maya/rig/spline.py driven by
// rig_bits.nxt:/spine and /neck). Three control frames (root, mid, end)
// shape an open degree-2 B-spline with four CVs; the joint chain is laid
// out along that curve by arc length; each joint aims +X along the curve
// with an up vector carried from its rest frame by the minimal rotation; a
// twist that is linear in arc position is added; and a linear
// volume-preservation scale thins (stretch) or thickens (squash) the
// off-axis handles. The whole thing is pure math on plain types: no stage,
// no schema, no hierarchy. Callers resolve the frames and hand them in.
//
// Why not RigExecRibbon or the single-chain IK: the ribbon reads its driver
// curve as native scene data, so a curve reshaped by control-driven movers
// is invisible to it; a single-chain IK solves to an effector with no
// shaping, twist distribution, or arc-length stretch. This kernel builds
// the curve itself from control frames and therefore lives entirely in the
// pose phase.
//
// Conventions (state them, then trust them):
//   * Row-vector matrices, p' = p * M, the GfMatrix4d layout used by every
//     other RigExec kernel. The only matrices used here are the rest->pose
//     affine maps of the control frames from RigExecPointsToMatrix.
//   * Frames are RigExecPointFrame landmarks [O, X, Y, Z]. The aim axis is
//     X (down the chain toward the next joint), the up axis is Y, Z closes
//     the basis with the handedness of the rest frame. Rest handle lengths
//     |X-O|, |Y-O|, |Z-O| are preserved and the volume scale multiplies the
//     Y and Z handles. Every input and output frame is in one common space
//     (in practice the chain parent's space; see the rigid-motion note in
//     RigExecSolveSplineIk).
//   * The curve is the open degree-2 B-spline with four CVs P0..P3 and Maya
//     knots [0,0,1,2,2] (full clamped vector [0,0,0,1,2,2,2]), parameter
//     u in [0,2], two spans. Inserting the knot u=1 to multiplicity two
//     adds the point (P1+P2)/2 (alpha = (1-0)/(2-0) = 1/2), so each span is
//     a quadratic Bezier:
//         span 0, u in [0,1]: (P0, P1, (P1+P2)/2)
//         span 1, u in [1,2]: ((P1+P2)/2, P2, P3)
//     The curve interpolates P0 and P3 only; it does NOT pass through P1 or
//     P2, which is why the interior joints of a chain whose rest CVs are
//     joint positions do not land exactly on their rest origins (the
//     rest-residual the tests measure; a caller absorbs it with a
//     maintained offset the way Maya's mo=1 constraints do).
//   * Arc length is integrated with 8-point Gauss-Legendre quadrature on 32
//     sub-intervals per span (the speed |C'(u)| of a quadratic is smooth,
//     so this is accurate to roughly 1e-14 relative); the inverse (param at
//     arc distance) is a table lookup plus safeguarded Newton.
//   * Angles are radians; a positive twist is a right-handed rotation about
//     the joint's +X aim axis.
//
// The Maya spec, as implemented:
//   Curve.  Rest CVs are the rest origins of joints [0], [1], [N-2], [N-1].
//     cv0 and cv1 are carried by the root control's rest->pose map, cv2 and
//     cv3 by the end control's; cv1 and cv2 additionally receive the mid
//     control's translation offset relative to its follow point, which is
//     the mid control's rest origin carried by the root and end controls
//     and blended with RigExecSplineIkParams::midFollowWeight (a Maya
//     parentConstraint with maintainOffset on both parents). At rest the
//     offset is zero.
//   Placement.  ratio = arcLength / restArcLength. Joint 0 sits at the
//     curve start; joint i sits at arc distance ratio * sum(segment lengths
//     before i). No clamp: a joint past the curve end continues straight
//     along the end tangent. With restArcLength equal to the chain length
//     the chain spans the whole curve exactly; with restArcLength equal to
//     the rest curve length the ratio is exactly one at rest. Those two
//     differ whenever the rest chain is not itself the curve (the real
//     spine: chain 34.504 cm, curve 34.386 cm), so the choice is an
//     explicit input (RigExecSplineIkRestLength) and never guessed.
//   Frames.  +X aims at the next joint (the last joint aims along the curve
//     tangent at its position). The up vector is the rest Y direction
//     carried by the minimal rotation taking the rest X direction to the
//     posed aim, re-orthogonalised. Then Y and Z are rotated about +X by
//     roll + twist * t_i, with t_i = arcDistance_i / arcLength.
//   Twist.  roll is the root control's twist about the rest chain axis
//     (cv3 - cv0 at rest) relative to its rest frame, by swing-twist
//     decomposition of the rest->pose rotation; twist is the end control's
//     twist about the same axis minus roll. Maya's linear twistType.
//   Squash.  s_x = 1, s_y = s_z = 1 - w_i * preserveVolume * (ratio - 1).
//     Linear thinning, exactly as specified, no clamp: ratio < 1 thickens.
//     Reference weights: spine [0.1429, 0.2857, 0.4286, 0.5, 0.3571,
//     0.2143, 0.0714], neck [0.16, 0.32, 0.4, 0.24, 0.08].
//
#ifndef RIGEXEC_MATH_SPLINE_IK_H
#define RIGEXEC_MATH_SPLINE_IK_H

#include "pointFrame.h"

#include "pxr/pxr.h"
#include "pxr/base/gf/vec3d.h"

#include <array>
#include <cstddef>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Length tolerance below which a curve, chain, segment, or handle is
/// considered collapsed. Inputs are character-scale (centimetres), so an
/// absolute tolerance is appropriate, matching RigExecFrameEpsilon's scale.
constexpr double RigExecSplineIkEpsilon = 1e-10;

/// The open degree-2, four-CV B-spline with its arc-length table. Built
/// once per solve (or once per test) from the posed CVs.
class RigExecSplineIkCurve {
public:
    RigExecSplineIkCurve();
    explicit RigExecSplineIkCurve(const std::array<GfVec3d, 4> &cvs);

    const std::array<GfVec3d, 4> &Cvs() const { return _cvs; }

    /// Total arc length over u in [0, 2].
    double ArcLength() const { return _length; }

    /// True when the whole curve is shorter than RigExecSplineIkEpsilon
    /// (all four CVs coincident): there is no usable tangent anywhere.
    bool IsDegenerate() const { return _length <= RigExecSplineIkEpsilon; }

    /// Position and derivative dC/du at parameter u, clamped to [0, 2].
    /// Either output may be null.
    void Evaluate(double u, GfVec3d *position, GfVec3d *derivative) const;

    /// Parameter u at arc distance \p distance from the start, clamped to
    /// [0, ArcLength()]. Accurate to about 1e-13 of the arc length.
    double ParamAtArcLength(double distance) const;

    /// Position and unit tangent at arc distance \p distance. A distance
    /// past the end extrapolates straight along the end tangent; a
    /// negative distance extrapolates backward along the start tangent.
    /// Returns false (position = start CV, tangent = zero) when the curve
    /// is degenerate. When the derivative vanishes at the requested point
    /// (coincident CVs at one end) the tangent falls back to the chord to
    /// the nearest distinct point on the curve.
    bool PointAtArcLength(
        double distance, GfVec3d *position, GfVec3d *tangent) const;

private:
    std::array<GfVec3d, 4> _cvs;
    std::vector<double> _u;          ///< table parameters (sub-interval edges)
    std::vector<double> _cumulative; ///< arc length at each table parameter
    double _length = 0.0;
};

/// Which length the stretch ratio is measured against (see the file
/// comment: they differ whenever the rest chain is not itself the curve).
enum class RigExecSplineIkRestLength {
    Curve,  ///< the rest curve's arc length: ratio == 1 at rest
    Chain,  ///< the sum of rest segment lengths: chain spans the curve exactly
};

/// Everything measured at rest, all in the common space.
struct RigExecSplineIkRest {
    /// Rest curve CVs. RigExecSplineIkMakeRest fills them from joints
    /// [0], [1], [N-2], [N-1].
    std::array<GfVec3d, 4> cvs{};

    /// Rest control frames. Their rest->pose maps carry the CVs.
    RigExecPointFrame rootControl;
    RigExecPointFrame midControl;
    RigExecPointFrame endControl;

    /// Rest joint frames, ordered root to tip. X should aim down the chain
    /// and Y carries the up direction; handle lengths are preserved.
    std::vector<RigExecPointFrame> joints;

    /// segmentLengths[k] is the rest arc spacing between joints k and k+1
    /// (Maya's rest tx of joint k+1); size joints.size() - 1.
    std::vector<double> segmentLengths;

    /// The reference length for ratio = arcLength / restArcLength. Must be
    /// positive; see RigExecSplineIkRestLength.
    double restArcLength = 0.0;

    /// Per-joint volume weights w_i, size joints.size(); empty means zero
    /// everywhere (no thinning).
    std::vector<double> volumeWeights;
};

/// The posed control frames, in the common space.
struct RigExecSplineIkControls {
    RigExecPointFrame root;
    RigExecPointFrame mid;
    RigExecPointFrame end;
};

/// Solver parameters.
struct RigExecSplineIkParams {
    /// 0..1 strength of the linear squash/stretch thinning. Not clamped.
    double preserveVolume = 1.0;

    /// Blend of the mid control's follow point between its rest origin
    /// carried by the root control (0) and by the end control (1).
    double midFollowWeight = 0.5;

    /// Additive roll and twist (radians) on top of what the controls
    /// contribute: the Maya ikHandle roll / twist attributes.
    double roll = 0.0;
    double twist = 0.0;

    /// Length floor as a fraction of the rest root->end chord; 0 (the
    /// default) is off. When the posed end CV (cv3) projects onto the
    /// root control's posed chain axis -- the rest chord direction carried
    /// by the root's rest->pose map -- less than this fraction of the rest
    /// chord ahead of cv0, cv2 and cv3 are both moved along that axis
    /// until it does. The end then overshoots its control, the curve can
    /// neither shrink past the floor nor fold back through the root (the
    /// two tangent legs |cv1 - cv0| and |cv3 - cv2| stay clear of each
    /// other once the floor exceeds their sum over the chord), and the
    /// mid control's follow point, the twist and the volume ratio are
    /// unaffected because they read the control frames, not the CVs.
    /// Maya's ikSpline has no floor at all; this is the requested
    /// departure. Not clamped to [0, 1].
    double minLengthRatio = 0.0;

    /// Aim the root tangent: cv1 is turned about cv0 by the minimal
    /// rotation taking the root control's posed chain axis to the
    /// direction cv0 -> cv3 (after the length floor), so the curve leaves
    /// the root pointing at the end. Maya's neck does exactly this with
    /// cluster[1] under a joint at the neck control aimed at the head
    /// (rig_bits.nxt /neck/head_pivot_connect); done here it uses the
    /// floored end and adds no twist (the rotation axis is perpendicular
    /// to the chain). The mid offset is applied on top. Off by default.
    bool aimRootTangent = false;
};

/// One posed joint.
struct RigExecSplineIkJoint {
    /// Posed frame. The Y and Z handles already carry \p scale.
    RigExecPointFrame frame;
    /// (1, s, s) with s = 1 - w_i * preserveVolume * (ratio - 1).
    GfVec3d scale{1.0, 1.0, 1.0};
    /// Arc distance from the curve start, ratio * (rest spacing before i).
    double arcDistance = 0.0;
    /// Normalised arc position t_i = arcDistance / arcLength (0 when the
    /// curve is degenerate). Not clamped: a joint extrapolated past the
    /// end has t_i > 1 and the twist gradient continues linearly.
    double arcParam = 0.0;
    /// Applied twist angle roll + twist * t_i, radians.
    double twist = 0.0;
};

/// The solve output.
struct RigExecSplineIkResult {
    std::array<GfVec3d, 4> cvs{};   ///< posed CVs the joints were laid on
    double arcLength = 0.0;         ///< posed curve arc length
    double ratio = 1.0;             ///< arcLength / restArcLength
    double roll = 0.0;              ///< root twist about the chain axis + params.roll
    double twist = 0.0;             ///< end twist - root twist + params.twist
    std::vector<RigExecSplineIkJoint> joints;
};

/// Builds the rest description from rest joint frames and rest control
/// frames: cvs from joints [0], [1], [N-2], [N-1]; segmentLengths from
/// consecutive origins; restArcLength per \p restLength. \p volumeWeights
/// is copied through (may be empty). A chain shorter than two joints uses
/// the available origins for every CV (one joint: all four CVs equal).
RigExecSplineIkRest RigExecSplineIkMakeRest(
    const std::vector<RigExecPointFrame> &joints,
    const RigExecPointFrame &rootControl,
    const RigExecPointFrame &midControl,
    const RigExecPointFrame &endControl,
    const std::vector<double> &volumeWeights = {},
    RigExecSplineIkRestLength restLength = RigExecSplineIkRestLength::Curve);

/// Carries the rest CVs by the control frames as described in the file
/// comment. Returns false (and rest CVs) if a rest control frame is
/// singular.
bool RigExecSplineIkPoseCvs(
    const RigExecSplineIkRest &rest,
    const RigExecSplineIkControls &controls,
    const RigExecSplineIkParams &params,
    std::array<GfVec3d, 4> *cvs);

/// Twist of the rest->pose rotation of a frame about a unit \p axis, in
/// radians in [-pi, pi], by swing-twist decomposition: with the rotation
/// quaternion q = (w, v), sign-corrected to w >= 0, the twist about the
/// axis is 2 * atan2(dot(v, axis), w). Any scale in the frames is removed
/// by orthonormalising the rest->pose map first; a reflection is ignored
/// (its proper rotation is used). Returns 0 for a singular rest frame, a
/// zero axis, or a non-finite input.
double RigExecSplineIkTwistAboutAxis(
    const RigExecPointFrame &restFrame,
    const RigExecPointFrame &posedFrame,
    const GfVec3d &axis);

/// Solves the chain. See the file comment for the model.
///
/// Returns true when every joint was fully determined. Degenerate cases
/// return false with a best-effort, always finite result:
///   * no joints, or segmentLengths / volumeWeights of the wrong size:
///     result cleared (no joints);
///   * restArcLength <= epsilon, or a singular rest control frame: every
///     joint is its rest frame flagged RigExecPointFrameDegenerate;
///   * a degenerate posed curve (all CVs coincident): every joint sits at
///     the curve point with its rest orientation, flagged degenerate;
///   * a rest joint frame with a collapsed X or Y handle: that joint is
///     returned as its rest frame flagged degenerate, the others solve.
/// A single-joint chain is not degenerate: the joint sits at the curve
/// start aimed along the start tangent (its restArcLength must then come
/// from the curve, since the chain length is zero).
///
/// Rigid-motion note: because roll is measured about the REST chain axis,
/// a rigid motion of all three controls reproduces the rigidly moved rest
/// frames exactly for a chain whose rest X axes lie along the chain axis
/// (a straight rest chain); a curved rest chain reproduces it only up to
/// the swing/twist ambiguity inherent in the linear-twist model, which is
/// why callers should hand in frames in the chain parent's space rather
/// than world space.
bool RigExecSolveSplineIk(
    const RigExecSplineIkRest &rest,
    const RigExecSplineIkControls &controls,
    const RigExecSplineIkParams &params,
    RigExecSplineIkResult *result);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SPLINE_IK_H
