// RigExec property-domain math kernels, USD-free.
// One copy of the arithmetic for every adapter: propertyMath.cpp
// instantiates these templates with Gf types for the evaluators, and the
// zero-USD runtime instantiates them with its own vector and matrix types.
// Key, V3 and M4 only need element access (key[0], v[i], m[r][c]) and, for
// M4, the row-vector product `base * value`; the operations, their order and
// the float/double types of every intermediate are fixed here, so each
// instantiation computes the same bits.
#ifndef RIGEXEC_MATH_PROPERTY_MATH_KERNEL_H
#define RIGEXEC_MATH_PROPERTY_MATH_KERNEL_H

#include "rigExecMath/envelope.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rigExec {

/// The operation a property mover performs (schema `rigExec:operation`).
enum class RigExecPropertyOp { Add, Multiply, Clamp, Remap, Blend, Curve };

/// The authored inputs a float or vec3f property mover carries, with the
/// curve keys as \p Key pairs (input, output).
///
/// `min`/`max` are only read by clamp and remap, `value` only by add,
/// multiply, and blend; a mover authors all of them and the operation
/// chooses. Per-component for the vec3f variant, which is why the schema
/// declares float3 rather than float bounds.
template <class T, class Key>
struct RigExecPropertyMathKernelParams {
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
    const Key *keys = nullptr;
    size_t keyCount = 0;
    /// Optional (in slope, out slope) per key, parallel to keys. With them
    /// the curve is a cubic Hermite through the keys and extrapolates along
    /// the first key's in slope and the last key's out slope, which is how a
    /// driven key with fixed tangents and linear infinity evaluates.
    const Key *tangents = nullptr;
    size_t tangentCount = 0;
};

/// Piecewise-linear keys with linear extrapolation along the end segments.
/// The segment search is a linear scan up to eight keys and a binary
/// search beyond; both find the same segment.
template <class Key>
float
RigExecEvaluateLinearKeysKernel(const Key *keys, size_t keyCount, float x)
{
    if (!keys || keyCount == 0) {
        return x;
    }
    if (keyCount == 1) {
        return keys[0][1];
    }
    // The segment whose end is the first key past x, clamped to the first and
    // last segments so values outside the keys extrapolate along them.
    size_t hi = 1;
    if (keyCount <= 8) {
        while (hi < keyCount - 1 && keys[hi][0] < x) {
            ++hi;
        }
    } else {
        const Key *it = std::lower_bound(
            keys + 1, keys + keyCount - 1, x,
            [](const Key &key, float value) { return key[0] < value; });
        hi = size_t(it - keys);
    }
    const Key &a = keys[hi - 1];
    const Key &b = keys[hi];
    const float span = b[0] - a[0];
    if (span == 0.0f) {
        return a[1];
    }
    return a[1] + (x - a[0]) * (b[1] - a[1]) / span;
}

/// Cubic Hermite keys with per-key (in, out) slopes, extrapolated along the
/// end slopes; the basis is evaluated in double and the result cast to
/// float. Null tangents are the linear keys.
template <class Key>
float
RigExecEvaluateHermiteKeysKernel(const Key *keys, const Key *tangents,
                                 size_t keyCount, float x)
{
    if (!tangents) {
        return RigExecEvaluateLinearKeysKernel(keys, keyCount, x);
    }
    if (!keys || keyCount == 0) {
        return x;
    }
    const size_t last = keyCount - 1;
    if (x <= keys[0][0]) {
        return keys[0][1] + (x - keys[0][0]) * tangents[0][0];
    }
    // A singleton has no interior segment, including unordered x.
    if (keyCount == 1 || x >= keys[last][0]) {
        return keys[last][1] + (x - keys[last][0]) * tangents[last][1];
    }
    size_t hi = 1;
    if (keyCount <= 8) {
        while (hi < last && keys[hi][0] < x) {
            ++hi;
        }
    } else {
        const Key *it = std::lower_bound(
            keys + 1, keys + last, x,
            [](const Key &key, float value) { return key[0] < value; });
        hi = size_t(it - keys);
    }
    const Key &a = keys[hi - 1];
    const Key &b = keys[hi];
    const double h = double(b[0]) - double(a[0]);
    if (h <= 0.0) {
        return a[1];
    }
    const double t = (double(x) - double(a[0])) / h;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    return float(h00 * a[1] + h10 * h * tangents[hi - 1][1] +
                 h01 * b[1] + h11 * h * tangents[hi][0]);
}

/// True when the keys are finite and strictly increasing in input.
template <class Key>
bool
RigExecValidateLinearKeysKernel(const Key *keys, size_t keyCount)
{
    for (size_t i = 0; i < keyCount; ++i) {
        if (!std::isfinite(keys[i][0]) || !std::isfinite(keys[i][1])) {
            return false;
        }
        if (i > 0 && !(keys[i - 1][0] < keys[i][0])) {
            return false;
        }
    }
    return true;
}

/// One float revision: r = op(base), then RigExecBlendEnvelope(base, r,
/// weight) with a float weight.
template <class Key>
float
RigExecApplyFloatMathKernel(
    float base, const RigExecPropertyMathKernelParams<float, Key> &params)
{
    float result = base;
    switch (params.op) {
    case RigExecPropertyOp::Add:
        result = base + params.value;
        break;
    case RigExecPropertyOp::Multiply:
        result = base * params.value;
        break;
    case RigExecPropertyOp::Clamp:
        // Authored bounds that cross are not silently reordered: a rig with
        // max < min is a mistake, and min(max(v, lo), hi) pinning the result
        // to hi is at least a legible one.
        result = std::min(std::max(base, params.min), params.max);
        break;
    case RigExecPropertyOp::Remap: {
        const float span = params.max - params.min;
        result = span == 0.0f ? 0.0f : (base - params.min) / span;
        break;
    }
    case RigExecPropertyOp::Blend:
        result = params.value;
        break;
    case RigExecPropertyOp::Curve:
        result = RigExecEvaluateHermiteKeysKernel(
            params.keys,
            params.tangentCount == params.keyCount ? params.tangents : nullptr,
            params.keyCount, base);
        break;
    }
    return RigExecBlendEnvelope(base, result, params.weight);
}

/// The vec3f peer, component-wise in every operation including the bounds;
/// curve leaves the base (compile refuses curve on a vec3f mover).
///
/// noinline: the Gf entry point and a plain-type instantiation must return
/// the same bits, including NaN signs. Inlining lets a caller fold this
/// arithmetic with the argument expressions, and that rewrite does not
/// preserve a NaN's sign.
template <class V3, class Key>
#if defined(_MSC_VER)
__declspec(noinline)
#else
__attribute__((noinline))
#endif
V3
RigExecApplyVec3fMathKernel(
    const V3 &base, const RigExecPropertyMathKernelParams<V3, Key> &params)
{
    V3 result = base;
    for (size_t i = 0; i < 3; ++i) {
        switch (params.op) {
        case RigExecPropertyOp::Add:
            result[i] = base[i] + params.value[i];
            break;
        case RigExecPropertyOp::Multiply:
            result[i] = base[i] * params.value[i];
            break;
        case RigExecPropertyOp::Clamp:
            result[i] =
                std::min(std::max(base[i], params.min[i]), params.max[i]);
            break;
        case RigExecPropertyOp::Remap: {
            const float span = params.max[i] - params.min[i];
            result[i] = span == 0.0f ? 0.0f : (base[i] - params.min[i]) / span;
            break;
        }
        case RigExecPropertyOp::Blend:
            result[i] = params.value[i];
            break;
        case RigExecPropertyOp::Curve:
            break;
        }
    }
    // Every component is overwritten; the copy only avoids requiring a
    // default constructor of V3.
    V3 mixed = base;
    for (size_t i = 0; i < 3; ++i) {
        mixed[i] = RigExecBlendEnvelope(base[i], result[i], params.weight);
    }
    return mixed;
}

/// The matrix4d peer: multiply is `base * value` (value applied after base
/// in the row-vector convention), blend is `value`, and the mix is
/// per element with the weight widened to double. False for any other
/// operation, leaving \p result untouched.
template <class M4>
bool
RigExecApplyMatrixMathKernel(const M4 &base, RigExecPropertyOp op,
                             const M4 &value, float weight, M4 *result)
{
    if (!result) {
        return false;
    }
    if (op != RigExecPropertyOp::Multiply && op != RigExecPropertyOp::Blend) {
        // add / clamp / remap have no matrix meaning. The schema's
        // allowedTokens already say so, but allowedTokens is advisory in
        // USD, so the evaluator validates against this return.
        return false;
    }
    const M4 operated =
        op == RigExecPropertyOp::Multiply ? M4(base * value) : value;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            (*result)[r][c] = RigExecBlendEnvelope(
                base[r][c], operated[r][c], double(weight));
        }
    }
    return true;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_PROPERTY_MATH_KERNEL_H
