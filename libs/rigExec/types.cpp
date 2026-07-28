//
// RigExec exec type registrations (spec §12.1).
//
#include "types.h"

#include "pxr/exec/exec/typeRegistry.h"

#include <algorithm>

namespace rigExec {

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

}  // namespace rigExec

PXR_NAMESPACE_OPEN_SCOPE

TF_REGISTRY_FUNCTION(ExecTypeRegistry)
{
    ExecTypeRegistry::RegisterType(rigExec::RigExecPointFrame{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecPointFrameArray{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecPointsPacket{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecWeightPacket{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecBlendSampleData{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecBlendChannel{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecMoverParameters{});
    ExecTypeRegistry::RegisterType(rigExec::RigExecMoverStatus{});
}

PXR_NAMESPACE_CLOSE_SCOPE
