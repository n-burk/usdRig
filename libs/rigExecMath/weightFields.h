//
// RigExec volumetric weight-field kernels (spec §4.1 weight objects,
// volumetric extension).
//
// Pure, deterministic CPU kernels that turn a point set plus a placed
// volume into a scalar weight field. Everything here is expressed in the
// VOLUME's local space: callers hand in the world-to-volume matrix and the
// kernel transforms as it goes, so a non-uniform volume transform gives
// ellipsoidal (sphere) or sheared (plane, curve) iso-surfaces for free
// rather than needing a separate radius-per-axis authoring surface.
//
// No token-string dispatch and no USD dependency beyond gf/vt: the schema
// token -> enum mapping belongs to the rigExec layer, exactly as it does
// for the geometry mover kernels.
//
#ifndef RIGEXEC_MATH_WEIGHT_FIELDS_H
#define RIGEXEC_MATH_WEIGHT_FIELDS_H

#include "pointFrame.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"

#include <cstddef>
#include <vector>

namespace rigExec {

/// Named analytic falloff profiles, baked to a lookup table so the hot
/// loop has exactly one remap path whether the author picked a preset or
/// drew a curve (spec §4.1 volumetric extension).
///
/// Every profile maps the ramp parameter r in [0, 1] -- 1 at falloffMin,
/// 0 at falloffMax -- to a weight in [0, 1], with f(0) = 0 and f(1) = 1.
enum class RigExecFalloffProfile {
    Linear,    ///< f(r) = r
    Smooth,    ///< smoothstep: f(r) = r^2 (3 - 2r); zero slope at both ends
    EaseIn,    ///< f(r) = r^2; slow departure from zero
    EaseOut,   ///< f(r) = 1 - (1 - r)^2; slow approach to one
    Constant,  ///< hard step: f(r) = r > 0 ? 1 : 0
};

/// Resolution of the baked falloff lookup table. A power of two plus one
/// so both endpoints land exactly on a sample and f(0)/f(1) are exact.
constexpr size_t RigExecFalloffLutSize = 257;

/// Bakes \p profile into a \p count-entry lookup table over r in [0, 1].
/// Fewer than two entries yields an empty table, which every consumer
/// reads as "linear".
std::vector<float> RigExecBuildFalloffLut(
    RigExecFalloffProfile profile, size_t count = RigExecFalloffLutSize);

/// The distance-to-weight remap shared by every volumetric weight object.
///
///   u = clamp01((d - falloffMin) / (falloffMax - falloffMin))
///   u = lerp(u, 1 - u, invert)
///   w = strength * Curve(1 - u)
///
/// so the field is fully ON at falloffMin and fully OFF at falloffMax,
/// and \p invert exchanges the two ends continuously (it is a float, not
/// a bool, so it can be animated or driven like any other avar).
///
/// falloffMax < falloffMin is legal and simply flips the band; only the
/// exactly-degenerate band is special-cased, to a hard step at that
/// distance.
struct RigExecFalloffParams {
    float falloffMin = 0.0f;
    float falloffMax = 1.0f;
    float invert = 0.0f;
    float strength = 1.0f;
    /// Uniformly spaced samples of Curve over r in [0, 1]. Empty means
    /// linear. Built by RigExecBuildFalloffLut or resampled from an
    /// authored spline by the caller -- this layer never knows about Ts.
    std::vector<float> curve;
};

/// Evaluates the remap above for one raw distance. The result is NOT
/// clamped after the strength multiply: the weight object's rangePolicy
/// (strict or clamp) is the authority on out-of-range weights, and
/// silently clamping here would hide a strict-policy violation.
float RigExecEvaluateFalloff(float distance, const RigExecFalloffParams &p);

/// Samples \p curve (a uniform [0, 1] lookup table) at \p r with linear
/// interpolation, clamping r into range. An empty or single-entry table
/// is the identity.
float RigExecSampleFalloffLut(const std::vector<float> &curve, float r);

// ---------------------------------------------------------------------------
// Distance functions. Each takes a point already in the volume's local
// space; the field builders below do the transform.
// ---------------------------------------------------------------------------

/// Radial distance from the local origin.
float RigExecSphereDistance(const GfVec3f &p);

/// SIGNED distance along one local axis (0 = x, 1 = y, 2 = z). Signed
/// rather than absolute so a falloffMin/falloffMax pair straddling zero
/// authors a gradient ACROSS the plane -- the useful planar-falloff
/// behaviour -- instead of a band mirrored on both sides of it. An author
/// who wants the mirrored band sets falloffMin = 0 on a plane placed at
/// the band's centre, or uses two planes composed with min.
float RigExecPlaneDistance(const GfVec3f &p, int axis);

/// Half-extents of the rectangle a BOUNDED plane weight's field lives
/// inside, measured in the two in-plane axes U = (axis + 1) % 3 and
/// V = (axis + 2) % 3.
///
/// Deliberately not derived from the falloff band. The band is a distance
/// ALONG the axis and decides where the gradient runs; the extents are
/// the size ACROSS it and decide how far the sheet reaches. Sizing one
/// from the other means scrubbing the falloff silently resizes the plane.
struct RigExecPlaneBounds {
    float extentU = 1.0f;
    float extentV = 1.0f;
};

/// True when \p p (already in the volume's local space) is inside the
/// rectangle |u| <= extentU, |v| <= extentV about \p axis.
///
/// Containment is HARD: a point one epsilon outside gets zero, not a
/// ramped-down weight. That is the requested behaviour and it is the only
/// one that makes the bounded region exactly the drawn rectangle. An
/// author who wants a soft border composes the bounded plane with a
/// sphere (or a second plane) and inherits that volume's falloff -- which
/// is the same answer composition gives to every other "but softer"
/// question in this file.
bool RigExecPlaneWithinBounds(
    const GfVec3f &p, int axis, const RigExecPlaneBounds &bounds);

/// Distance from \p p to the nearest point on the polyline through
/// \p curvePoints. Zero points gives infinity (an empty field); one point
/// degenerates to the sphere distance about it.
float RigExecCurveDistance(
    const GfVec3f &p, const GfVec3f *curvePoints, size_t count);

/// Distance from \p p to the nearest point on the segment [a, b].
float RigExecSegmentDistance(
    const GfVec3f &p, const GfVec3f &a, const GfVec3f &b);

// ---------------------------------------------------------------------------
// Field builders. \p worldToLocal maps a point from the space \p points
// are expressed in into the volume's local space; pass identity when they
// already share a space. \p weights is resized to points.size().
// ---------------------------------------------------------------------------

void RigExecSphereWeightField(
    const std::vector<GfVec3f> &points,
    const GfMatrix4d &worldToLocal,
    const RigExecFalloffParams &params,
    std::vector<float> *weights);

/// \p bounds is null for the infinite plane and non-null for a bounded
/// one, whose field is exactly zero outside the rectangle. A pointer
/// rather than a flag-plus-struct so "unbounded" has one spelling and a
/// caller cannot pass extents that are quietly ignored; the caller is
/// responsible for having rejected non-positive or non-finite extents,
/// exactly as it is for the per-axis scales.
void RigExecPlaneWeightField(
    const std::vector<GfVec3f> &points,
    const GfMatrix4d &worldToLocal,
    int axis,
    const RigExecFalloffParams &params,
    std::vector<float> *weights,
    const RigExecPlaneBounds *bounds = nullptr);

/// \p curvePoints are taken in the SAME space as \p points (they are a
/// native geometry property read off the stage, not a property of the
/// volume prim), so both are transformed by \p worldToLocal together.
void RigExecCurveWeightField(
    const std::vector<GfVec3f> &points,
    const std::vector<GfVec3f> &curvePoints,
    const GfMatrix4d &worldToLocal,
    const RigExecFalloffParams &params,
    std::vector<float> *weights);

// ---------------------------------------------------------------------------
// Composition (spec §4.1 volumetric extension: weight objects compose).
// ---------------------------------------------------------------------------

/// How one weight field folds into the accumulated result.
///
/// Multiply, Add, Max, Min, and Average are order independent. Subtract
/// and Overlay are NOT: they consume the authored relationship target
/// order, which is the one place the volumetric extension departs from
/// the spec §7.2 rule that target-list permutation cannot change a
/// result. Authors who need a permutation-proof composition use the
/// commutative modes or nest single-input combines.
enum class RigExecWeightCombine {
    Multiply,  ///< identity 1
    Add,       ///< identity 0
    Subtract,  ///< first input minus each subsequent one
    Max,       ///< identity 0
    Min,       ///< identity 1
    Average,   ///< arithmetic mean over the inputs
    Overlay,   ///< a < 0.5 ? 2ab : 1 - 2(1-a)(1-b), per element
};

/// The seed value a fold of zero inputs produces under \p mode.
float RigExecWeightCombineIdentity(RigExecWeightCombine mode);

/// Folds one value into an accumulator. Average is NOT expressible
/// pairwise and is handled only by RigExecCombineWeightFields.
float RigExecFoldWeight(RigExecWeightCombine mode, float acc, float value);

/// Ordered fold of \p inputs (each the same length) into \p out.
///
/// Inputs of differing lengths are a structural error: \p out is cleared
/// and false is returned rather than folding a truncated field, because a
/// short input would silently mean "identity" for the missing tail.
/// Zero inputs fills \p out with the mode's identity at length
/// \p elementCount.
bool RigExecCombineWeightFields(
    RigExecWeightCombine mode,
    const std::vector<std::vector<float>> &inputs,
    size_t elementCount,
    std::vector<float> *out);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_WEIGHT_FIELDS_H
