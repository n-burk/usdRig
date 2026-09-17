//
// RigExec property-domain math kernels.
//
#include "propertyMath.h"
#include "envelope.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

bool
RigExecParsePropertyOp(const TfToken &token, RigExecPropertyOp *op)
{
    if (!op) {
        return false;
    }
    if (token == "add") {
        *op = RigExecPropertyOp::Add;
    } else if (token == "multiply") {
        *op = RigExecPropertyOp::Multiply;
    } else if (token == "clamp") {
        *op = RigExecPropertyOp::Clamp;
    } else if (token == "remap") {
        *op = RigExecPropertyOp::Remap;
    } else if (token == "blend") {
        *op = RigExecPropertyOp::Blend;
    } else if (token == "curve") {
        *op = RigExecPropertyOp::Curve;
    } else {
        return false;
    }
    return true;
}

float
RigExecEvaluateLinearKeys(const GfVec2f *keys, size_t keyCount, float x)
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
        const GfVec2f *it = std::lower_bound(
            keys + 1, keys + keyCount - 1, x,
            [](const GfVec2f &key, float value) { return key[0] < value; });
        hi = size_t(it - keys);
    }
    const GfVec2f &a = keys[hi - 1];
    const GfVec2f &b = keys[hi];
    const float span = b[0] - a[0];
    if (span == 0.0f) {
        return a[1];
    }
    return a[1] + (x - a[0]) * (b[1] - a[1]) / span;
}

float
RigExecEvaluateHermiteKeys(const GfVec2f *keys, const GfVec2f *tangents,
                           size_t keyCount, float x)
{
    if (!tangents) {
        return RigExecEvaluateLinearKeys(keys, keyCount, x);
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
        const GfVec2f *it = std::lower_bound(
            keys + 1, keys + last, x,
            [](const GfVec2f &key, float value) { return key[0] < value; });
        hi = size_t(it - keys);
    }
    const GfVec2f &a = keys[hi - 1];
    const GfVec2f &b = keys[hi];
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

bool
RigExecValidateLinearKeys(const GfVec2f *keys, size_t keyCount)
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

float
RigExecApplyFloatMath(
    float base, const RigExecPropertyMathParams<float> &params)
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
        result = RigExecEvaluateHermiteKeys(
            params.keys,
            params.tangentCount == params.keyCount ? params.tangents : nullptr,
            params.keyCount, base);
        break;
    }
    return RigExecBlendEnvelope(base, result, params.weight);
}

GfVec3f
RigExecApplyVec3fMath(
    const GfVec3f &base, const RigExecPropertyMathParams<GfVec3f> &params)
{
    GfVec3f result = base;
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
            // Float only; compile refuses curve on a vec3f mover.
            break;
        }
    }
    GfVec3f mixed;
    for (size_t i = 0; i < 3; ++i) {
        mixed[i] = RigExecBlendEnvelope(base[i], result[i], params.weight);
    }
    return mixed;
}

bool
RigExecApplyMatrixMath(
    const GfMatrix4d &base, RigExecPropertyOp op, const GfMatrix4d &value,
    float weight, GfMatrix4d *result)
{
    if (!result) {
        return false;
    }
    GfMatrix4d operated;
    switch (op) {
    case RigExecPropertyOp::Multiply:
        // Row-vector convention: base first, then value.
        operated = base * value;
        break;
    case RigExecPropertyOp::Blend:
        operated = value;
        break;
    default:
        // add / clamp / remap have no matrix meaning. The schema's
        // allowedTokens already say so, but allowedTokens is advisory in
        // USD, so the evaluator validates against this return.
        return false;
    }
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            (*result)[r][c] = RigExecBlendEnvelope(
                base[r][c], operated[r][c], double(weight));
        }
    }
    return true;
}

}  // namespace rigExec
