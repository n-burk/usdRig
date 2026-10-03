#include "rigExec/movers/moverRegistry.h"

#include <cmath>
#include <limits>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {

bool Assemble(const UsdPrim &prim, const RigExecRevisionBinding &,
              const RigExecProviderValues &values, UsdTimeCode time,
              VtValue *data)
{
    if (values.basePoints.empty()) return false;
    float gain = 1.0f;
    const UsdAttribute attribute = prim.GetAttribute(TfToken("inputs:gain"));
    if (values.resolved) {
        values.resolved->GetAttribute(attribute, time, &gain);
    } else {
        attribute.Get(&gain, time);
    }
    if (!std::isfinite(gain)) return false;
    *data = VtValue(gain);
    return true;
}

bool Apply(const VtValue &data, std::vector<GfVec3f> *points)
{
    if (!data.IsHolding<float>()) return false;
    const float gain = data.UncheckedGet<float>();
    if (gain == -2.0f) {
        points->push_back(GfVec3f(0));
        return true;
    }
    if (gain == -3.0f && !points->empty()) {
        (*points)[0][2] = std::numeric_limits<float>::quiet_NaN();
        return true;
    }
    for (GfVec3f &point : *points) {
        point[2] += gain * point[0] * point[0];
    }
    // Exercise the host's atomic failure path after modifying the candidate.
    return gain >= 0.0f;
}

RigExecOracleResult Oracle(const RigExecMoverOracleContext &ctx)
{
    float gain = 1.0f;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(TfToken("inputs:gain")),
                              ctx.time, &gain);
    if (!std::isfinite(gain) || gain < 0.0f) {
        return RigExecOracleResult::PassThrough;
    }
    for (GfVec3f &point : *ctx.points) {
        point[2] += gain * point[0] * point[0];
    }
    return RigExecOracleResult::Blend;
}

RigExecMoverHandler Handler()
{
    RigExecMoverHandler handler("ExternalQuadraticMover",
        &RigExecFixedMoverOp<RigExecRevisionOp::External>,
        RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.assembleExternal = &Assemble;
    handler.applyExternal = &Apply;
    handler.oracle = &Oracle;
    return handler;
}

}  // namespace

RIGEXEC_REGISTER_MOVER(Handler());
