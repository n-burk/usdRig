//
// RigExec exec type registrations (spec §12.1).
//
#include "types.h"

#include "pxr/exec/exec/typeRegistry.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

void RigExecLoadComputations() {}

float
RigExecWeightPacket::Resolve(size_t i, size_t count) const
{
    if (representation == "dense") {
        if (values.size() != count || i >= count) {
            return -1.0f;
        }
        return values[i];
    }
    if (representation == "sparse") {
        if (indices.size() != values.size()) {
            return -1.0f;
        }
        const auto it = std::lower_bound(
            indices.begin(), indices.end(), static_cast<int>(i));
        if (it != indices.end() && *it == static_cast<int>(i)) {
            return values[it - indices.begin()];
        }
        return defaultWeight;
    }
    // constant
    return defaultWeight;
}

RigExecWeightPacket
RigExecWeightPacket::Constant(float weight)
{
    RigExecWeightPacket packet;
    packet.representation = TfToken("constant");
    packet.rangePolicy = TfToken("strict");
    packet.defaultWeight = weight;
    packet.valid = std::isfinite(weight) && weight >= 0.0f && weight <= 1.0f;
    return packet;
}

bool
RigExecWeightPacket::ResolveAll(
    size_t count, std::vector<float> *resolved) const
{
    if (!resolved || !valid) {
        return false;
    }
    if (!rangePolicy.IsEmpty() &&
        rangePolicy != "strict" && rangePolicy != "clamp") {
        return false;
    }
    if (representation == "constant") {
        if (!values.empty() || !indices.empty()) {
            return false;
        }
    } else if (representation == "dense") {
        if (!indices.empty() || values.size() != count) {
            return false;
        }
    } else if (representation == "sparse") {
        if (indices.size() != values.size()) {
            return false;
        }
        for (size_t i = 0; i < indices.size(); ++i) {
            if (indices[i] < 0 || static_cast<size_t>(indices[i]) >= count ||
                (i > 0 && indices[i] <= indices[i - 1])) {
                return false;
            }
        }
    } else {
        return false;
    }
    std::vector<float> valuesOut(count);
    for (size_t i = 0; i < count; ++i) {
        const float value = Resolve(i, count);
        if (!std::isfinite(value) || value < 0.0f || value > 1.0f) {
            return false;
        }
        valuesOut[i] = value;
    }
    resolved->swap(valuesOut);
    return true;
}

}  // namespace rigExec

PXR_NAMESPACE_OPEN_SCOPE

TF_REGISTRY_FUNCTION(ExecTypeRegistry)
{
    ExecTypeRegistry::RegisterType(rigExec::RigExecPointFrame{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecPointFrameArray{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecPointsPacket{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecWeightPacket{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecFalloffLut{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecBlendSampleData{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecBlendChannel{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecMoverParameters{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecMoverStatus{});
}

PXR_NAMESPACE_CLOSE_SCOPE
