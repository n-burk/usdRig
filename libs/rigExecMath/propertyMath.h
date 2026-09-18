//
// RigExec property-domain math kernels (spec §4.1 math movers).
//
// The point-domain movers revise a point3f[] array; these revise a single
// scalar, vector, or matrix property. Same chain model -- base value in,
// revised value out, one revision per mover in composed post-order -- so the
// only thing that differs is the value type, and that is exactly what the
// three statically typed mover schemas encode.
//
// Pure and stateless, like every other kernel in this library: the evaluator
// reads the authored inputs and hands them over as values.
//
#ifndef RIGEXEC_MATH_PROPERTY_MATH_H
#define RIGEXEC_MATH_PROPERTY_MATH_H

#include "pxr/pxr.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The operation a property mover performs (schema `rigExec:operation`).
enum class RigExecPropertyOp { Add, Multiply, Clamp, Remap, Blend, Curve };

/// Parses a `rigExec:operation` token. Returns false for an unknown token
/// rather than substituting a default: the operation selects the compiled
/// kernel, and silently computing the wrong one is worse than refusing.
bool RigExecParsePropertyOp(const TfToken &token, RigExecPropertyOp *op);

/// The authored inputs a float or vec3f property mover carries.
///
/// `min`/`max` are only read by clamp and remap, `value` only by add,
/// multiply, and blend; a mover authors all of them and the operation
/// chooses. Per-component for the vec3f variant, which is why the schema
/// declares float3 rather than float bounds.
template <class T>
struct RigExecPropertyMathParams {
    RigExecPropertyOp op = RigExecPropertyOp::Add;
    T value{};
    T min{};
    T max{};
    /// Resolved common MoverAPI envelope (spec §5): either the bound
    /// rigExec:weightObject's one-element field or inputs:defaultWeight. The
    /// operation's result is mixed back toward the incoming value, so weight
    /// 0 is a no-op and weight 1 applies the operation outright. Same rule the
    /// point-domain matrix mover follows (p' = q + w*(T q - q)).
    float weight = 1.0f;
    /// The curve operation's keys as (input, output) pairs sorted by input,
    /// borrowed from the caller for the duration of one apply. Only curve
    /// reads them, and only the float mover defines curve.
    const GfVec2f *keys = nullptr;
    size_t keyCount = 0;
    /// Optional (in slope, out slope) per key, parallel to keys. With them
    /// the curve is a cubic Hermite through the keys and extrapolates along
    /// the first key's in slope and the last key's out slope, which is how a
    /// driven key with fixed tangents and linear infinity evaluates.
    const GfVec2f *tangents = nullptr;
    size_t tangentCount = 0;
};

/// Piecewise-linear evaluation of sorted (input, output) keys with linear
/// extrapolation past both ends along the first and last segments, the way
/// a driven key with linear tangents and linear pre/post infinity behaves.
/// One key is a constant; no keys return x unchanged.
float RigExecEvaluateLinearKeys(const GfVec2f *keys, size_t keyCount, float x);

/// Cubic Hermite evaluation of sorted keys with per-key (in, out) slopes,
/// extrapolated linearly along the end slopes. tangents may be null, which
/// is RigExecEvaluateLinearKeys.
float RigExecEvaluateHermiteKeys(const GfVec2f *keys, const GfVec2f *tangents,
                                 size_t keyCount, float x);

/// True when the keys are finite and strictly increasing in input.
bool RigExecValidateLinearKeys(const GfVec2f *keys, size_t keyCount);

/// Applies one float revision: r = op(base), then base + weight*(r - base).
///
/// - add:      r = base + value
/// - multiply: r = base * value
/// - clamp:    r = min(max(base, lo), hi)
/// - remap:    r = (base - lo) / (hi - lo), UNCLAMPED (a degenerate
///             lo == hi yields 0). Composing a clamp mover after a remap is
///             what bounds it -- baking the bound in would make the two
///             operations indistinguishable and cost the author the
///             out-of-range signal.
/// - blend:    r = value (so the weighted result is lerp(base, value, w))
/// - curve:    r = keys(base), piecewise linear with linear extrapolation
float RigExecApplyFloatMath(
    float base, const RigExecPropertyMathParams<float> &params);

/// The vec3f peer, component-wise in every operation including the bounds.
GfVec3f RigExecApplyVec3fMath(
    const GfVec3f &base, const RigExecPropertyMathParams<GfVec3f> &params);

/// The matrix4d peer. Only multiply and blend are defined for a matrix:
///
/// - multiply: r = base * value, i.e. `value` applied AFTER `base` in
///             GfMatrix4d's row-vector convention (a post-multiply, which is
///             what an authored local offset means).
/// - blend:    r = value
///
/// The weighted mix is component-wise, matching the point-domain matrix
/// mover's p' = q + w*(T q - q): exact at both endpoints, and a linear path
/// between them. A rotation blended this way is not a great intermediate
/// matrix, which is why RigExecBlendPointFrames exists for frames; a matrix
/// PROPERTY has no rest set to decompose against, so there is nothing better
/// available here and the endpoints are what authors actually use.
///
/// Returns false for an unsupported operation, leaving \p result untouched.
bool RigExecApplyMatrixMath(
    const GfMatrix4d &base, RigExecPropertyOp op, const GfMatrix4d &value,
    float weight, GfMatrix4d *result);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_PROPERTY_MATH_H
