//
// RigExec — point-native transform system (spec §5).
//
// The canonical pose value is a point frame: four points [O, X, Y, Z]
// representing an affine origin and the transformed X, Y, and Z basis
// endpoints. It round-trips exactly to a nonsingular affine matrix,
// including scale and shear.
//
#ifndef RIGEXEC_MATH_POINT_FRAME_H
#define RIGEXEC_MATH_POINT_FRAME_H

#include "pxr/pxr.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/tf/token.h"

#include <array>
#include <cstdint>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Flags describing the state of a RigExecPointFrame (spec §5.1).
enum RigExecPointFrameFlags : uint32_t {
    RigExecPointFrameValid      = 1u << 0,  ///< frame carries usable points
    RigExecPointFrameDegenerate = 1u << 1,  ///< posed frame is singular/collapsed
    RigExecPointFrameReflected  = 1u << 2,  ///< negative-determinant linear part
    RigExecPointFrameAffine     = 1u << 3,  ///< carries shear/nonuniform scale
};

/// The runtime scalar transform value (spec §5.1).
///
/// points[0] = O (origin), points[1] = X/tip, points[2] = Y/up,
/// points[3] = Z/side.
struct RigExecPointFrame {
    std::array<GfVec3d, 4> points{ GfVec3d(0), GfVec3d(1, 0, 0),
                                   GfVec3d(0, 1, 0), GfVec3d(0, 0, 1) };
    uint32_t flags = RigExecPointFrameValid;

    bool operator==(const RigExecPointFrame &o) const {
        return points == o.points && flags == o.flags;
    }
    bool operator!=(const RigExecPointFrame &o) const { return !(*this == o); }

    const GfVec3d &Origin() const { return points[0]; }
    const GfVec3d &X() const { return points[1]; }
    const GfVec3d &Y() const { return points[2]; }
    const GfVec3d &Z() const { return points[3]; }

    bool IsValid() const { return flags & RigExecPointFrameValid; }
    bool IsDegenerate() const { return flags & RigExecPointFrameDegenerate; }
};

/// Frame reconstruction policies (spec §5.2).
enum class RigExecFramePolicy {
    Affine,      ///< use all four points directly; keeps scale/shear/reflection
    Orthogonal,  ///< aim/up orthonormalization; removes shear; 3 scales
    Axial,       ///< orthogonal with scale only along the aim axis
    Rigid,       ///< orthogonal, unit scale
};

/// Parses a policy token ("affine", "orthogonal", "axial", "rigid").
/// Unknown tokens return the default policy (Orthogonal).
RigExecFramePolicy RigExecParseFramePolicy(const TfToken &token);

/// Axis selector used for aim/up/reflection policies.
enum class RigExecAxis { X, Y, Z };
RigExecAxis RigExecParseAxis(const TfToken &token, RigExecAxis fallback);

/// The affine map from reference (rest) points Q to posed points P
/// (spec §5.1):
///   B_Q = [qx-qo qy-qo qz-qo], B_P = [px-po py-po pz-po] (column matrices)
///   L = B_P B_Q^-1, t = po - L qo, f(q) = L q + t.
///
/// The returned GfMatrix4d satisfies M.TransformAffine(q_i) ~= p_i for
/// each landmark (row-vector convention internally; conformance is by
/// transforming points, never by raw element comparison).
///
/// Returns false (identity matrix) when the *reference* frame is singular.
/// A singular *posed* frame still produces the forward map.
bool RigExecPointsToMatrix(
    const std::array<GfVec3d, 4> &restPoints,
    const std::array<GfVec3d, 4> &posePoints,
    GfMatrix4d *matrix);

/// Convenience overload evaluating a frame against its rest landmarks.
bool RigExecPointsToMatrix(
    const std::array<GfVec3d, 4> &restPoints,
    const RigExecPointFrame &frame,
    GfMatrix4d *matrix);

/// Applies matrix to the four rest landmarks to produce posed points
/// (spec §5.4 MatrixToPoints).
RigExecPointFrame RigExecMatrixToPoints(
    const std::array<GfVec3d, 4> &restPoints,
    const GfMatrix4d &matrix);

/// Parameters controlling deterministic frame reconstruction.
struct RigExecFrameReconstructionArgs {
    RigExecFramePolicy policy = RigExecFramePolicy::Orthogonal;
    RigExecAxis aimAxis = RigExecAxis::X;  ///< which landmark carries aim
    RigExecAxis upAxis = RigExecAxis::Y;   ///< which landmark carries up/twist
    double twist = 0.0;                    ///< unwrapped twist (radians)
    RigExecAxis reflectionAxis = RigExecAxis::Z;  ///< pinned reflection axis
    /// Handedness assigned when the side landmark lies in the aim/up plane
    /// (zero side determinant): the authored reflection policy supplies the
    /// sign deterministically (spec §5.2 "zero follows the authored
    /// reflection policy").
    double zeroSideSign = 1.0;
    /// Deterministic degeneracy fallbacks (spec §5.3), in order:
    /// parent computed up axis, authored rest up axis, world axis least
    /// parallel to the aim direction. Optional parent up direction:
    bool hasParentUp = false;
    GfVec3d parentUp{0, 1, 0};
};

/// Reconstructs a frame from posed landmarks under a policy (spec §5.2).
///
/// For the orthogonal family:
///   a = px - po, ex = a/|a|
///   u = (py - po) - ex (ex . (py - po)), ey = u/|u|, ez = ex x ey, ey = ez x ex
///   twist theta rotates ey/ez about ex.
///   sx = |a|/lx, sy = |u|/ly, dz = (pz - po) . ez, sz = |dz|/lz,
///   sigma_z = sign(dz).
/// Reconstructed landmarks: po, po + lx sx ex, po + ly sy ey_theta,
/// po + lz sigma_z sz ez_theta. Rigid forces all scales to one; axial
/// forces sy = sz = 1.
///
/// Degeneracy (spec §5.3): eps = 1e-10 * max(1, lx, ly, lz). |a| < eps
/// invalidates orientation (frame returned with Degenerate flag and raw
/// points). |u| < eps means twist underdetermined: fall back to parent up,
/// then rest up, then the world basis axis least parallel to ex.
RigExecPointFrame RigExecReconstructFrame(
    const std::array<GfVec3d, 4> &restPoints,
    const std::array<GfVec3d, 4> &posePoints,
    const RigExecFrameReconstructionArgs &args);

/// SRT parameter tuple (spec §5.4). Decomposition is via SVD:
/// L = U S V^T, delta = det(U V^T); reflection selected on the pinned
/// reflectionAxis; R = U D V^T proper rotation; H = R^T L with
/// diag(H) = scale and normalized off-diagonals = shear.
struct RigExecTransformParams {
    GfVec3d translation{0, 0, 0};
    GfQuatd rotation{1, 0, 0, 0};   ///< unit quaternion derived from R
    GfVec3d scale{1, 1, 1};         ///< diag(H); may carry one negative axis
    GfVec3d shear{0, 0, 0};         ///< (h_xy, h_xz, h_yz) normalized
    RigExecAxis reflectionAxis = RigExecAxis::Z;
};

/// Decomposes the affine map defined by rest->pose points into SRT
/// parameters. Returns false for a singular reference or posed frame.
bool RigExecPointsToParams(
    const std::array<GfVec3d, 4> &restPoints,
    const std::array<GfVec3d, 4> &posePoints,
    RigExecAxis reflectionAxis,
    RigExecTransformParams *params);

/// Rebuilds the affine matrix L = R(q) H(s,h), t and returns the posed
/// landmarks obtained by applying it to restPoints.
GfMatrix4d RigExecParamsToMatrix(const RigExecTransformParams &params);

/// Character-scale-relative degeneracy tolerance (spec §5.3):
/// eps = 1e-10 * max(1, lx, ly, lz).
double RigExecFrameEpsilon(const std::array<GfVec3d, 4> &restPoints);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_POINT_FRAME_H
