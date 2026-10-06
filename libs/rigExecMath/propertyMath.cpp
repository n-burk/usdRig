// RigExec property-domain math kernels.
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
    return propertyMathKernel::EvaluateLinearKeys(keys, keyCount, x);
}

float
RigExecEvaluateHermiteKeys(const GfVec2f *keys, const GfVec2f *tangents,
                           size_t keyCount, float x)
{
    return propertyMathKernel::EvaluateHermiteKeys(keys, tangents, keyCount,
                                                   x);
}

bool
RigExecValidateLinearKeys(const GfVec2f *keys, size_t keyCount)
{
    return propertyMathKernel::ValidateLinearKeys(keys, keyCount);
}

float
RigExecApplyFloatMath(
    float base, const RigExecPropertyMathParams<float> &params)
{
    return propertyMathKernel::ApplyFloatMath(
        base, params.op, params.value, params.min, params.max, params.weight,
        params.keys, params.keyCount, params.tangents, params.tangentCount);
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
