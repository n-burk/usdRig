// Internal stage-sampling transport; no authored access to private slots.
#ifndef RIGEXEC_RUNTIME_STAGE_ARRAY_INPUTS_H
#define RIGEXEC_RUNTIME_STAGE_ARRAY_INPUTS_H
#include "values.h"
#include "rigExecBinary/format.h"
#include <cstddef>
#include <string>
#include <set>
#include <vector>
#include <unordered_map>
namespace rigExec {
class RigExecRuntimeReader;
// Raw canonical weightTarget fallback, independent of resolved path rows.
inline std::vector<int32_t>
RigExecStageOracleFallbackSlots(const fb::RigExecWireFile &file)
{
    if (!file.geometry) return {};
    std::unordered_map<uint32_t, int32_t> slots;
    for (size_t i = 0; i < file.inputs.size(); ++i) {
        if (file.inputs[i].type() == fb::InputTag::Vec3fArray)
            slots.emplace(file.inputs[i].name(), int32_t(i));
    }
    std::vector<int32_t> result(file.geometry->weightObjects.size(), -1);
    for (size_t i = 0; i < result.size(); ++i) {
        const auto &object = file.geometry->weightObjects[i];
        if (object.samplesInFlight || object.oracleSamplesSlot < 0 ||
            object.targetPoints.size() != 1 || object.targetValid.size() != 1)
            continue;
        const auto found = slots.find(object.targetPoints[0]);
        if (found != slots.end()) result[i] = found->second;
    }
    return result;
}

inline std::vector<size_t>
RigExecStageArraySlots(const fb::RigExecWireFile &file)
{
    if (!file.geometry) return {};
    std::set<size_t> timeSlots;
    const auto field = [&](int32_t id) {
        if (id >= 0) timeSlots.insert(size_t(id));
    };
    const auto read = [&](const RigExecWireInput *input) {
        if (input && RigExecFormatIsArrayTag(input->tag)) {
            timeSlots.insert(input->walk.begin(), input->walk.end());
        }
    };
    for (const auto &row : file.geometry->pathReads) {
        if (!row.rest) read(row.read.get());
    }
    const auto revision = [&](const auto &rev) {
        field(rev.jointIndicesSlot);
        field(rev.jointWeightsSlot);
        for (const auto &channel : rev.blendChannels) {
            for (const auto &sample : channel.samples) read(sample.pointsRead.get());
        }
    };
    for (const auto &chain : file.geometry->chains) {
        field(chain.baseSlot);
        for (const auto &rev : chain.revisions) revision(rev);
        for (const auto &derived : chain.derived) {
            field(derived.baseSlot);
            if (derived.revision) revision(*derived.revision);
        }
    }
    if(file.pose) for(const auto &arrays:file.pose->constraintArrays)
        for(int slot:arrays.rawSlots) if(slot>=0) field(slot);
    if(file.providerProgram) for(const auto &leaf:file.providerProgram->sampled)
        if(leaf.inputSlot>=0 && RigExecFormatIsArrayTag(file.inputs[size_t(leaf.inputSlot)].type()))
            field(leaf.inputSlot);
    for (const auto &object : file.geometry->weightObjects) {
        field(object.oracleSamplesSlot);
        field(object.oracleCurveSlot);
    }
    for (int32_t slot : RigExecStageOracleFallbackSlots(file)) field(slot);
    std::vector<size_t> result;
    for (const size_t id : timeSlots) {
        if (id < file.inputs.size() &&
            RigExecFormatIsArrayTag(file.inputs[id].type()) &&
            (file.inputs[id].flags() & uint8_t(RigExecWireInputSlotFlags::Animated))) {
            result.push_back(id);
        }
    }
    return result;
}
inline std::vector<size_t>
RigExecStageTokenSlots(const fb::RigExecWireFile &file)
{
    std::set<size_t> slots;
    if (file.geometry) {
        for (const auto &object : file.geometry->weightObjects) {
            for (int32_t slot : {object.oraclePlaneAxisSlot, object.oraclePlaneBoundsSlot}) {
                if (slot >= 0 && size_t(slot) < file.inputs.size() &&
                    file.inputs[size_t(slot)].type() == fb::InputTag::Token &&
                    (file.inputs[size_t(slot)].flags() & uint8_t(RigExecWireInputSlotFlags::Animated)))
                    slots.insert(size_t(slot));
            }
        }
    }
    return {slots.begin(), slots.end()};
}
struct RigExecStageArrayInputInfo {
    size_t slot = 0;
    std::string name;
    RrInputTag tag = RrInputTag::FloatArray;
    bool animated = true; // Existing array/token rows are temporal by admission.
};
class RigExecRuntimeStageArrayInputs {
public:
    static bool CanSample(const RigExecRuntimeReader &reader, size_t slot);
    static bool CanSampleProviderValue(const RigExecRuntimeReader &reader,size_t slot);
    static bool SetSampleBlocked(RigExecRuntimeReader &reader,size_t slot,bool blocked,std::string *error);
    static std::vector<RigExecStageArrayInputInfo> EnumerateProviderValues(const RigExecRuntimeReader &reader);
    static bool SetScalarSample(RigExecRuntimeReader &,size_t,const RrInputValue &,std::string *);
    static bool ClearScalarSample(RigExecRuntimeReader &,size_t,std::string *);
    static bool CanSampleToken(const RigExecRuntimeReader &reader, size_t slot);
    static bool SetTokenArraySample(RigExecRuntimeReader &reader, size_t slot,
        const std::vector<std::string> &values, std::string *error);
    static bool SetTokenSample(RigExecRuntimeReader &reader, size_t slot,
                               const std::string &text, std::string *error);
    static bool ClearTokenSample(RigExecRuntimeReader &reader, size_t slot,
                                 std::string *error);
    static std::vector<RigExecStageArrayInputInfo> EnumerateTokens(
        const RigExecRuntimeReader &reader);
    static std::vector<RigExecStageArrayInputInfo> Enumerate(
        const RigExecRuntimeReader &reader);
    static bool SetSample(RigExecRuntimeReader &reader, size_t slot,
                          const RigExecRuntimeArray &value, std::string *error);
    static bool ClearSample(RigExecRuntimeReader &reader, size_t slot,
                            std::string *error);
};
}
#endif
