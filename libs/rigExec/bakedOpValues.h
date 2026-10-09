#ifndef RIGEXEC_BAKED_OP_VALUES_H
#define RIGEXEC_BAKED_OP_VALUES_H
#include <cstdint>
#include "pxr/base/vt/value.h"
#include <string>
#include <vector>
#include <utility>
namespace rigExec {
/// Unsupported boxed types compare unequal conservatively.
bool RigExecExactSourceValueEqual(const PXR_NS::VtValue &, const PXR_NS::VtValue &);
struct RigExecBakedProgramImpl;
struct RigExecBakedStep;
enum class RigExecBakedSlotDomain : uint8_t;
/// Checked source/value identity mapping from an outgoing epoch to its replacement.
/// Used only to re-encode exact retained Skin memos; values are never rewritten.
struct RigExecBakedOpIdentityRemap {
    std::vector<int> bindingLeaves, bindingRefIndices, headLeaves, overrides, readerWalks;
    std::vector<int> values;
};
/// Exact in-process value bytes, appended into caller-retained storage.
/// Opaque plugin payloads cannot be encoded; their packet outputs must use
/// typed equality or conservative change propagation in the graph adapter.
void RigExecBakedOpValueKey(const RigExecBakedProgramImpl &,
    RigExecBakedSlotDomain, uint32_t slot, std::string *key);
/// False means an unsupported boxed type or invalid declaration: seed conservatively.
bool RigExecBakedOpInputKey(const RigExecBakedProgramImpl &,
    const RigExecBakedStep &, std::string *key,
    const RigExecBakedOpIdentityRemap *remap=nullptr);
/// True when RigExecBakedOpInputKey reads nothing sampled, overridable or
/// published for \p step: its bytes and exactness are then fixed by the
/// compiled step, so the executor builds that key once per program.
bool RigExecBakedOpInputKeyIsConstant(const RigExecBakedProgramImpl &,
    const RigExecBakedStep &);
bool RigExecBakedOpEffectiveInputKey(const RigExecBakedProgramImpl &,
    const RigExecBakedStep &,std::string *key,std::vector<uint32_t> *coveredPropertyVersions,
    std::vector<std::pair<uint32_t,uint32_t>> *coveredTyped=nullptr,
    const RigExecBakedOpIdentityRemap *remap=nullptr);
bool RigExecBakedOpValueKeyIsExact(const RigExecBakedProgramImpl &,
    RigExecBakedSlotDomain, uint32_t slot);
}
#endif
