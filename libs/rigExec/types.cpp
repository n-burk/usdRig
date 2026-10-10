// RigExec exec type registrations (spec §12.1).
#include "types.h"

#include "pxr/exec/exec/typeRegistry.h"

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace rigExec {

namespace {
const TfToken constantWeightRepresentation("constant"), strictWeightRange("strict");
}

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
    packet.representation = constantWeightRepresentation;
    packet.rangePolicy = strictWeightRange;
    packet.defaultWeight = weight;
    packet.valid = std::isfinite(weight) && weight >= 0.0f && weight <= 1.0f;
    return packet;
}

// The validation behind both ResolveAll overloads and ResolvesAll: one
// definition, so a caller that decides ahead of the resolve (a revision's
// apply-or-fail decision) and the resolve itself cannot disagree.
static bool
_WeightPacketResolvesAll(const RigExecWeightPacket &packet, size_t count)
{
    if (!packet.valid) {
        return false;
    }
    if (!packet.rangePolicy.IsEmpty() &&
        packet.rangePolicy != "strict" && packet.rangePolicy != "clamp") {
        return false;
    }
    if (packet.representation == "constant") {
        if (!packet.values.empty() || !packet.indices.empty()) {
            return false;
        }
    } else if (packet.representation == "dense") {
        if (!packet.indices.empty() || packet.values.size() != count) {
            return false;
        }
    } else if (packet.representation == "sparse") {
        if (packet.indices.size() != packet.values.size()) {
            return false;
        }
        for (size_t i = 0; i < packet.indices.size(); ++i) {
            if (packet.indices[i] < 0 ||
                static_cast<size_t>(packet.indices[i]) >= count ||
                (i > 0 && packet.indices[i] <= packet.indices[i - 1])) {
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
    const auto usable = [](float v) {
        return std::isfinite(v) && v >= 0.0f && v <= 1.0f;
    };

    if (packet.representation == "constant") {
        return usable(packet.defaultWeight);
    }

    if (packet.representation == "dense") {
        for (size_t i = 0; i < count; ++i) {
            if (!usable(packet.values[i])) {
                return false;
            }
        }
        return true;
    }

    // sparse: the default is only checked when some point can actually read
    // it. A sparse packet that happens to name every point never resolves to
    // its default, and the per-point loop this replaces would never have
    // seen it -- so rejecting an unusable one here would fail a packet that
    // used to succeed.
    if (packet.indices.size() < count && !usable(packet.defaultWeight)) {
        return false;
    }
    for (const float value : packet.values) {
        if (!usable(value)) {
            return false;
        }
    }
    return true;
}

// One definition behind both ResolveAll overloads: identical
// validation, values, failure conditions, and atomicity for either
// output array. Every success arm assigns before writing, so resolving
// into a shared VtFloatArray drops (never detaches-copies) the old
// buffer. Only the vector instantiation can alias the packet's own
// weights, so only it keeps the self-resolve temporary.
template <typename FloatArray>
static bool
_ResolveWeightPacketAll(
    const RigExecWeightPacket &packet, size_t count,
    FloatArray *resolved)
{
    // Same values, same failure conditions, and the same atomicity: nothing
    // is written into *resolved until the whole array has passed, because an
    // in-place caller cannot roll back a partial write.
    if (!resolved || !_WeightPacketResolvesAll(packet, count)) {
        return false;
    }

    if (packet.representation == "constant") {
        resolved->assign(count, packet.defaultWeight);
        return true;
    }

    if (packet.representation == "dense") {
        resolved->assign(
            packet.values.begin(), packet.values.begin() + count);
        return true;
    }

    // sparse: the default everywhere, then the authored entries scattered
    // over it. The indices were range-checked and proved strictly ascending
    // above, so each one lands exactly once and in bounds. The self-resolve
    // keeps the temporary below: scattering into the packet's own weights
    // would clobber entries not yet read.
    if constexpr (std::is_same<FloatArray, std::vector<float>>::value) {
        if (resolved == &packet.values) {
            std::vector<float> valuesOut(count, packet.defaultWeight);
            for (size_t i = 0; i < packet.indices.size(); ++i) {
                valuesOut[static_cast<size_t>(packet.indices[i])] =
                    packet.values[i];
            }
            resolved->swap(valuesOut);
            return true;
        }
    }
    resolved->assign(count, packet.defaultWeight);
    for (size_t i = 0; i < packet.indices.size(); ++i) {
        (*resolved)[static_cast<size_t>(packet.indices[i])] =
            packet.values[i];
    }
    return true;
}

bool
RigExecWeightPacket::ResolveAll(
    size_t count, std::vector<float> *resolved) const
{
    return _ResolveWeightPacketAll(*this, count, resolved);
}

bool
RigExecWeightPacket::ResolveAll(
    size_t count, VtFloatArray *resolved) const
{
    return _ResolveWeightPacketAll(*this, count, resolved);
}

bool
RigExecWeightPacket::ResolvesAll(size_t count) const
{
    return _WeightPacketResolvesAll(*this, count);
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
