// Shared, USD-free float property math. The evaluator's float and double
// property movers (propertyMath.cpp) and the binary runtime's property
// chains both call these, so the two produce bit-identical results by
// construction. A key type only needs `[0]` (input) and `[1]` (output, or
// for tangents the in and out slopes) returning float.
#ifndef RIGEXEC_MATH_PROPERTY_MATH_KERNEL_H
#define RIGEXEC_MATH_PROPERTY_MATH_KERNEL_H

#include "rigExecMath/envelope.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rigExec {

/// The operation a property mover performs (schema `rigExec:operation`).
enum class RigExecPropertyOp { Add, Multiply, Clamp, Remap, Blend, Curve };

namespace propertyMathKernel {

/// Piecewise-linear evaluation of sorted (input, output) keys with linear
/// extrapolation past both ends along the first and last segments. One key
/// is a constant; no keys return x unchanged.
template <class Key>
float
EvaluateLinearKeys(const Key *keys, size_t keyCount, float x)
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

/// Cubic Hermite evaluation of sorted keys with per-key (in, out) slopes,
/// extrapolated linearly along the end slopes. tangents may be null, which
/// is EvaluateLinearKeys.
template <class Key>
float
EvaluateHermiteKeys(const Key *keys, const Key *tangents, size_t keyCount,
                    float x)
{
    if (!tangents) {
        return EvaluateLinearKeys(keys, keyCount, x);
    }
    if (!keys || keyCount == 0) {
        return x;
    }
    const size_t last = keyCount - 1;
    if (x <= keys[0][0]) {
        return keys[0][1] + (x - keys[0][0]) * tangents[0][0];
    }
    if (x >= keys[last][0]) {
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
ValidateLinearKeys(const Key *keys, size_t keyCount)
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

/// One float revision: r = op(base), then base + weight*(r - base).
/// `tangents` is used only when `tangentCount == keyCount`.
template <class Key>
float
ApplyFloatMath(float base, RigExecPropertyOp op, float value, float min,
               float max, float weight, const Key *keys, size_t keyCount,
               const Key *tangents, size_t tangentCount)
{
    float result = base;
    switch (op) {
    case RigExecPropertyOp::Add:
        result = base + value;
        break;
    case RigExecPropertyOp::Multiply:
        result = base * value;
        break;
    case RigExecPropertyOp::Clamp:
        // Authored bounds that cross are not silently reordered: a rig with
        // max < min is a mistake, and min(max(v, lo), hi) pinning the result
        // to hi is at least a legible one.
        result = std::min(std::max(base, min), max);
        break;
    case RigExecPropertyOp::Remap: {
        const float span = max - min;
        result = span == 0.0f ? 0.0f : (base - min) / span;
        break;
    }
    case RigExecPropertyOp::Blend:
        result = value;
        break;
    case RigExecPropertyOp::Curve:
        result = EvaluateHermiteKeys(
            keys, tangentCount == keyCount ? tangents : nullptr, keyCount,
            base);
        break;
    }
    return RigExecBlendEnvelope(base, result, weight);
}

}  // namespace propertyMathKernel
}  // namespace rigExec

#endif  // RIGEXEC_MATH_PROPERTY_MATH_KERNEL_H
