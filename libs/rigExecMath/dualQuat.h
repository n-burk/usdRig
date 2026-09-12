//
// RigExec dual-quaternion skinning primitives.
//
// A unit dual quaternion q^ = qr + eps*qd encodes a rigid motion: the real
// part qr is a unit quaternion carrying the rotation, and the dual part
// qd = 1/2 * t * qr carries the translation t (as a pure quaternion, on the
// LEFT of qr, so t is a world-space offset applied after the rotation).
// These are the building blocks for dual-quaternion skinning (DQS, Kavan
// et al., "Geometric Skinning with Approximate Dual Quaternion Blending",
// ACM TOG 27(4), 2008): blend N weighted joint motions, normalise once,
// apply to a point.
//
// Conventions (state them, then trust them):
//   * Matrices follow the GfMatrix4d row-vector convention used throughout
//     RigExec: points are row vectors, p' = p * M, the upper 3x3 rows are the
//     images of the basis vectors, and the translation lives in row 3. The
//     conversions here read and write exactly that layout; an orthonormal
//     rotation part M3 corresponds to the quaternion q with
//     p * M3 == q.Transform(p), which is the GfQuatd / GfMatrix4d::SetRotate
//     pairing already relied on by pointFrame.cpp.
//   * Quaternions are GfQuatd (real, imaginary), Hamilton product.
//   * A dual quaternion and its negation encode the same rigid motion. The
//     blend performs shortest-arc sign correction so that this double cover
//     never averages a motion against itself.
//
// Scale and shear policy: a dual quaternion can only represent a proper
// rigid motion. If the input matrix carries scale, shear, or a reflection,
// RigExecDualQuatFromMatrix keeps the proper rotation of the polar
// decomposition (the closest rotation in the Frobenius sense, computed by
// the library's standard RigExecPointsToParams SVD path) and the
// translation, and DROPS the stretch and any reflection. This is the same
// limitation Maya's dual-quaternion skinning has (joint scale is ignored by
// the DQ branch of skinCluster); dropping it with an explicit report is more
// useful to a rig than failing, because the alternative for a kernel is to
// fall back to linear blending for those joints, which needs to be a caller
// decision. The optional isRigid output tells the caller whether anything
// was dropped.
//
#ifndef RIGEXEC_MATH_DUAL_QUAT_H
#define RIGEXEC_MATH_DUAL_QUAT_H

#include "pxr/pxr.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quatd.h"

#include <cstddef>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// A dual quaternion real + eps*dual. Default-constructed as the identity
/// motion. Every function below that says "unit" assumes |real| == 1 and
/// dot(real, dual) == 0; RigExecDualQuatFromMatrix, the blends, and
/// RigExecDualQuatNormalize all return values satisfying that.
struct RigExecDualQuat {
    GfQuatd real{1.0};  ///< rotation (unit quaternion)
    GfQuatd dual{0.0};  ///< 1/2 * translation * real

    RigExecDualQuat() = default;
    RigExecDualQuat(const GfQuatd &r, const GfQuatd &d) : real(r), dual(d) {}
};

/// Tolerance used for orthonormality checks and for declaring a blended
/// real part too short to normalise.
constexpr double RigExecDualQuatEpsilon = 1e-9;

/// Builds a unit dual quaternion from a unit rotation and a world-space
/// translation applied after the rotation (p' = rotate(p) + translation).
/// The rotation is normalised defensively.
RigExecDualQuat RigExecDualQuatFromRotationTranslation(
    const GfQuatd &rotation, const GfVec3d &translation);

/// Converts a row-vector GfMatrix4d rigid transform to a unit dual
/// quaternion. See the scale/shear policy in the file comment: non-rigid
/// input keeps its polar rotation and translation and drops the rest; a
/// singular linear part keeps only the translation. \p isRigid, when
/// supplied, receives true iff the 3x3 part was orthonormal with positive
/// determinant within RigExecDualQuatEpsilon and nothing was dropped.
RigExecDualQuat RigExecDualQuatFromMatrix(
    const GfMatrix4d &matrix, bool *isRigid = nullptr);

/// Converts a unit dual quaternion back to a row-vector GfMatrix4d with the
/// rotation in the upper 3x3 and the translation in row 3.
GfMatrix4d RigExecDualQuatToMatrix(const RigExecDualQuat &dq);

/// Extracts the world-space translation t = 2 * dual * conj(real) of a unit
/// dual quaternion.
GfVec3d RigExecDualQuatTranslation(const RigExecDualQuat &dq);

/// True iff |real| == 1 and dot(real, dual) == 0 within \p tolerance.
bool RigExecDualQuatIsUnit(
    const RigExecDualQuat &dq, double tolerance = RigExecDualQuatEpsilon);

/// Normalises in place: divides both parts by |real| and removes the
/// component of the dual part along the real part, so the result is a unit
/// dual quaternion. Returns false and resets \p dq to the identity if |real|
/// is below RigExecDualQuatEpsilon or non-finite.
bool RigExecDualQuatNormalize(RigExecDualQuat *dq);

/// Weighted DQS blend of \p count dual quaternions.
///
/// The reference is the first input with a non-zero weight; every later
/// input whose real part has a negative dot product with the reference's is
/// negated (shortest-arc sign correction). The corrected inputs are
/// accumulated as sum(w_i * dq_i) and the sum is normalised ONCE at the end.
/// Weights need not sum to one (the normalisation absorbs the scale) and may
/// be negative; a single non-zero weight reproduces that input exactly.
///
/// Degenerate cases return false and write the identity to \p result:
/// count == 0, every weight zero, a non-finite weight, or an accumulated
/// real part with norm below RigExecDualQuatEpsilon * max(1, sum|w_i|)
/// (which is only reachable through cancelling weights, since sign
/// correction makes same-signed weights constructive).
bool RigExecBlendDualQuats(
    const RigExecDualQuat *dqs, const double *weights, size_t count,
    RigExecDualQuat *result);

/// UsdSkel-shaped variant of RigExecBlendDualQuats: influences are
/// (jointIndex, weight) pairs into a palette of joint dual quaternions.
/// An index outside [0, paletteSize) is a degenerate case (false, identity)
/// unless its weight is exactly zero, in which case it is ignored.
bool RigExecBlendDualQuats(
    const RigExecDualQuat *palette, size_t paletteSize,
    const int *jointIndices, const float *jointWeights, size_t count,
    RigExecDualQuat *result);

/// Rotates a direction by the real part of a unit dual quaternion:
/// v' = v + 2 * (w * (u x v) + u x (u x v)) with real = (w, u). No
/// translation. This is the per-point half of the transform for callers
/// that apply one dual quaternion to many points and hoist the translation
/// out of the loop (see the cost note below).
GfVec3d RigExecDualQuatRotateVector(
    const RigExecDualQuat &dq, const GfVec3d &vector);

/// Applies a unit dual quaternion directly to a point without building a
/// matrix: RigExecDualQuatRotateVector(dq, p) + RigExecDualQuatTranslation(dq).
///
/// Cost note, measured by testRigExecDualQuat (MSVC 19.38 /O2, 200k points,
/// best of three; the test prints the live numbers each run):
///   one blended dual quaternion per point, i.e. the skinning pattern:
///     RigExecDualQuatToMatrix + TransformAffine   ~14.5-18.5 ns/point
///     RigExecDualQuatTransformPoint               ~10.6-10.7 ns/point
///   eight points per dual quaternion (rigid clusters):
///     build matrix once + TransformAffine         ~4.5-5.9 ns/point
///     RigExecDualQuatTransformPoint per point     ~12.2-13.6 ns/point
///     RotateVector + hoisted Translation          ~9.3 ns/point
/// So for a per-point skinning kernel the direct path IS cheaper (1.4-1.7x):
/// the quaternion-to-matrix conversion dominates and is wasted on a single
/// point. Once several points share a dual quaternion the matrix wins,
/// because a 3x3 row-vector multiply (9 mul + 9 add) is cheaper than the
/// two cross products of the Rodrigues form; convert once and reuse it.
/// RotateVector with a hoisted translation sits in between and is the
/// choice when a caller wants to stay in quaternion form for a small cluster.
GfVec3d RigExecDualQuatTransformPoint(
    const RigExecDualQuat &dq, const GfVec3d &point);

// ---------------------------------------------------------------------------
// Scale-aware DQS
//
// A rig that scales a joint non-uniformly (the squash-and-stretch spine
// carries s_y = s_z = 1 - w * preserveVolume * (ratio - 1)) must not lose
// that scale on the DQS path. The rigid-only API above drops it by policy;
// this API carries it. Each influence is split by polar decomposition into a
// symmetric stretch and a rigid motion, in the row-vector convention
//
//     p' = p * S_i * R_i + t_i        S_i symmetric (scale + shear), R_i rotation
//
// where S_i acts FIRST, i.e. in the joint's pre-rotation frame. (Column
// form: L = R * H with H = R^T L symmetric, the right polar factor; transpose
// to rows and H * R_row falls out. RigExecPointsToParams computes exactly
// this H.) The two parts are then blended with the method appropriate to
// each:
//
//     S_blend = sum(w_i * S_i) / sum(w_i)     linear: correct for stretch
//     (R, t)_blend = RigExecBlendDualQuats     shortest-arc DQS: rigid stays rigid
//     p' = p * S_blend * R_blend + t_blend
//
// So the rotational part keeps DQS's length preservation (the reason DQS
// exists) and scale/shear is interpolated linearly in the pre-rotation
// frame, which is the interpolation that makes uniform scale s on every
// influence come out as exactly s and anisotropic scales come out as their
// weighted mean, independent of how much the influences rotate relative to
// each other.
//
// DIFFERENCE FROM MAYA: Maya's skinCluster in dual-quaternion mode ignores
// joint scale and shear entirely (only the rigid motion of each joint
// reaches the skin). This path does NOT ignore them. A non-uniformly scaled
// joint therefore deforms the skin here and does not in Maya; that is the
// intended behaviour, not a bug.
//
// Reflection and singular input: a reflected influence comes back from the
// library decomposition with one negative scale on the pinned Z axis, so
// its stretch is symmetric but not positive-definite, and a blend across it
// passes through zero volume linearly. A singular linear part has no
// rotation to keep: the rotation is identity and the stretch is the raw
// linear part, so a lone singular influence still reproduces its transform
// exactly. Both report isRigid == false.
// ---------------------------------------------------------------------------

/// One influence split into rigid motion plus pre-rotation stretch.
/// A rigid input has stretch == exactly the identity and isRigid == true,
/// which is what makes rigid inputs through the scale-aware path
/// bit-identical to the rigid-only path.
struct RigExecScaledDualQuat {
    RigExecDualQuat rigid;    ///< R_i and t_i
    GfMatrix3d stretch{1.0};  ///< S_i, row-vector 3x3 applied before rigid
    bool isRigid = true;      ///< stretch is the identity (nothing dropped)
};

/// Splits a row-vector GfMatrix4d into RigExecScaledDualQuat. The rigid
/// part is exactly what RigExecDualQuatFromMatrix returns for the same
/// input; the stretch is L * R^T (symmetrised) for non-rigid input and the
/// exact identity for rigid input. The orthonormality check is the same
/// cheap test the rigid path uses, so rigid joints never pay for the SVD.
RigExecScaledDualQuat RigExecScaledDualQuatFromMatrix(const GfMatrix4d &matrix);

/// Rebuilds the row-vector matrix [S * R | t].
GfMatrix4d RigExecScaledDualQuatToMatrix(const RigExecScaledDualQuat &sdq);

/// Scale-aware blend: rigid parts through RigExecBlendDualQuats (same
/// reference, sign correction, single normalisation, and degenerate rules),
/// stretches as sum(w_i * S_i) / sum(w_i). The result's isRigid is true iff
/// every non-zero-weight input was rigid, in which case its stretch is the
/// exact identity. Returns false and writes the identity when the rigid
/// blend is degenerate or when |sum(w_i)| <= RigExecDualQuatEpsilon *
/// max(1, sum|w_i|), since the stretch mean is then undefined (weights that
/// cancel are legal for the rigid-only blend but not here).
bool RigExecBlendScaledDualQuats(
    const RigExecScaledDualQuat *sdqs, const double *weights, size_t count,
    RigExecScaledDualQuat *result);

/// Palette-indexed variant of the scale-aware blend, the shape a skinning
/// kernel wants: influences are (paletteIndex, weight) pairs into a table
/// of decomposed joints, so the per-influence polar decomposition is done
/// once per joint and never copied per point. Same reference, sign
/// correction, single normalisation, stretch mean and degenerate rules as
/// the array form; the two agree bit for bit on the same inputs. Weights
/// are double so a caller can feed an exactly computed complement. An index
/// outside [0, paletteSize) is a degenerate case (false, identity) unless
/// its weight is exactly zero, in which case it is ignored.
bool RigExecBlendScaledDualQuats(
    const RigExecScaledDualQuat *palette, size_t paletteSize,
    const int *indices, const double *weights, size_t count,
    RigExecScaledDualQuat *result);

/// Applies p' = (p * S) rotated and translated by the rigid part. For a
/// rigid input (stretch identity) this is bit-identical to
/// RigExecDualQuatTransformPoint on the rigid part.
GfVec3d RigExecScaledDualQuatTransformPoint(
    const RigExecScaledDualQuat &sdq, const GfVec3d &point);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_DUAL_QUAT_H
