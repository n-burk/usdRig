// RigExec property-domain math kernels: the Gf instantiations of
// propertyMathKernel.h, which holds the arithmetic.
#include "propertyMath.h"

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
    return RigExecEvaluateLinearKeysKernel(keys, keyCount, x);
}

float
RigExecEvaluateHermiteKeys(const GfVec2f *keys, const GfVec2f *tangents,
                           size_t keyCount, float x)
{
    return RigExecEvaluateHermiteKeysKernel(keys, tangents, keyCount, x);
}

bool
RigExecValidateLinearKeys(const GfVec2f *keys, size_t keyCount)
{
    return RigExecValidateLinearKeysKernel(keys, keyCount);
}

float
RigExecApplyFloatMath(
    float base, const RigExecPropertyMathParams<float> &params)
{
    return RigExecApplyFloatMathKernel(base, params);
}

GfVec3f
RigExecApplyVec3fMath(
    const GfVec3f &base, const RigExecPropertyMathParams<GfVec3f> &params)
{
    return RigExecApplyVec3fMathKernel(base, params);
}

bool
RigExecApplyMatrixMath(
    const GfMatrix4d &base, RigExecPropertyOp op, const GfMatrix4d &value,
    float weight, GfMatrix4d *result)
{
    return RigExecApplyMatrixMathKernel(base, op, value, weight, result);
}

}  // namespace rigExec
