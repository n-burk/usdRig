//
// RigExec property-domain math kernels.
//
#include "propertyMath.h"

#include <algorithm>

namespace rigExec {

namespace {

// Non-finite inputs are the one case where "compute anyway" is worse than
// passing through: a NaN authored on a dial propagates into every consumer of
// the overridden attribute, and exec has no way to report that back. The
// evaluator fails such a mover with a diagnostic; these helpers keep the
// kernels themselves total.
template <class T>
T
_Lerp(const T &a, const T &b, float w)
{
    return a + (b - a) * double(w);
}

float
_LerpF(float a, float b, float w)
{
    return a + (b - a) * w;
}

}  // namespace

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
    } else {
        return false;
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
    }
    return _LerpF(base, result, params.weight);
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
        }
    }
    GfVec3f mixed;
    for (size_t i = 0; i < 3; ++i) {
        mixed[i] = _LerpF(base[i], result[i], params.weight);
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
            (*result)[r][c] =
                _Lerp(base[r][c], operated[r][c], weight);
        }
    }
    return true;
}

}  // namespace rigExec
