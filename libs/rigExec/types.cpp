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
    // PER REPRESENTATION, not per element. The shape was settled by the
    // checks above and cannot change inside the loop, so asking Resolve()
    // which representation this is once per point re-decides a question
    // already answered -- 26,276 times for one body mesh, every generation,
    // and for `sparse` it was worse than redundant: Resolve() binary-searches
    // the index array per point, making a whole-array resolve O(n log m)
    // where a scatter is O(n + m).
    //
    // Same values, same failure conditions, and the same atomicity: nothing
    // is written into *resolved until the whole array has passed, because an
    // in-place caller cannot roll back a partial write.
    const auto usable = [](float v) {
        return std::isfinite(v) && v >= 0.0f && v <= 1.0f;
    };

    if (representation == "constant") {
        if (!usable(defaultWeight)) {
            return false;
        }
        resolved->assign(count, defaultWeight);
        return true;
    }

    if (representation == "dense") {
        for (size_t i = 0; i < count; ++i) {
            if (!usable(values[i])) {
                return false;
            }
        }
        resolved->assign(values.begin(), values.begin() + count);
        return true;
    }

    // sparse: the default everywhere, then the authored entries scattered
    // over it. The indices were range-checked and proved strictly ascending
    // above, so each one lands exactly once and in bounds.
    //
    // The default is only checked when some point can actually read it. A
    // sparse packet that happens to name every point never resolves to its
    // default, and the per-point loop this replaces would never have seen
    // it -- so rejecting an unusable one here would fail a packet that used
    // to succeed.
    if (indices.size() < count && !usable(defaultWeight)) {
        return false;
    }
    for (const float value : values) {
        if (!usable(value)) {
            return false;
        }
    }
    std::vector<float> valuesOut(count, defaultWeight);
    for (size_t i = 0; i < indices.size(); ++i) {
        valuesOut[static_cast<size_t>(indices[i])] = values[i];
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
