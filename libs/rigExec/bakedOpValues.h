#ifndef RIGEXEC_BAKED_OP_VALUES_H
#define RIGEXEC_BAKED_OP_VALUES_H
#include <cstdint>
#include "pxr/base/vt/value.h"
#include <string>
#include <vector>
#include <utility>
PXR_NAMESPACE_OPEN_SCOPE
class GfVec3f;
PXR_NAMESPACE_CLOSE_SCOPE
namespace rigExec {
/// Unsupported boxed types compare unequal conservatively.
bool RigExecExactSourceValueEqual(const PXR_NS::VtValue &, const PXR_NS::VtValue &);
struct RigExecBakedProgramImpl;
struct RigExecBakedStep;
struct RigExecOpValueState;
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
/// Publishes a fixed-size value -- pose frames, matrices, avars, staged
/// frames, unboxed space values -- by building RigExecBakedOpValueKey's
/// bytes on the stack and comparing them in place with the stored key:
/// the flag, revision and key the string publication leaves, for values
/// RigExecBakedOpValueKeyIsExact calls exact. False, with \p value
/// untouched, for any other value; the caller publishes it through its key.
bool RigExecBakedPublishSmallValue(const RigExecBakedProgramImpl &,
    RigExecOpValueState *value);
/// RigExecBakedOpValueKey of provider leaf \p slot, returning what
/// RigExecBakedOpValueKeyIsExact answers for it, from one overlay lookup.
bool RigExecBakedSpaceLeafKey(const RigExecBakedProgramImpl &, uint32_t slot,
    std::string *key);
/// False means an unsupported boxed type or invalid declaration: seed conservatively.
/// A geometry path leaf keys as its content version (RigExecBakedSetPathLeaf),
/// which only this program's keys compare against; \p contentLeaves, or a
/// \p remap, keys its value instead, as a key read against another program must.
bool RigExecBakedOpInputKey(const RigExecBakedProgramImpl &,
    const RigExecBakedStep &, std::string *key,
    const RigExecBakedOpIdentityRemap *remap=nullptr, bool contentLeaves=false);
/// True when RigExecBakedOpInputKey reads nothing sampled, overridable or
/// published for \p step: its bytes and exactness are then fixed by the
/// compiled step, so the executor builds that key once per program.
bool RigExecBakedOpInputKeyIsConstant(const RigExecBakedProgramImpl &,
    const RigExecBakedStep &);
bool RigExecBakedOpEffectiveInputKey(const RigExecBakedProgramImpl &,
    const RigExecBakedStep &,std::string *key,std::vector<uint32_t> *coveredPropertyVersions,
    std::vector<std::pair<uint32_t,uint32_t>> *coveredTyped=nullptr,
    const RigExecBakedOpIdentityRemap *remap=nullptr, bool contentLeaves=false);
bool RigExecBakedOpValueKeyIsExact(const RigExecBakedProgramImpl &,
    RigExecBakedSlotDomain, uint32_t slot);
/// Whether two point arrays hold the same bytes: the equality the point
/// content versions (RevisionDone, ChainDirty, ChainPoints, ChainBase,
/// ChainInput, DerivedOut) are bumped by, signed zeros and NaN payloads
/// included.
bool RigExecBakedSamePoints(const PXR_NS::GfVec3f *a, size_t aCount,
    const PXR_NS::GfVec3f *b, size_t bCount);
/// The key those six domains carried before they were keyed by content
/// version: the same fields with the points' bytes in place of the version.
/// False, and an empty key, for any other domain. Owner thread; for the
/// RIGEXEC_VERIFY_CHAIN_VERSIONS check and tests.
bool RigExecBakedChainContentKey(const RigExecBakedProgramImpl &,
    RigExecBakedSlotDomain, uint32_t slot, std::string *key);
/// Replaces \p field with \p scratch's floats and bumps \p version, unless
/// the two hold the same bytes (signed zeros and NaN payloads included), in
/// which case nothing moves: how the RevisionPacket's `envelope` and
/// `publishedWeightValues` versions move. \p scratch is left unspecified.
void RigExecBakedNoteFloats(std::vector<float> *field,
    std::vector<float> *scratch, uint64_t *version);
/// The RevisionPacket key of revision \p slot as it was before its two
/// float arrays were keyed by content version: the same fields with the
/// arrays' bytes in place of the versions. False, and an empty key, for a
/// missing revision. Owner thread; for RIGEXEC_VERIFY_PACKET_VERSIONS and
/// tests.
bool RigExecBakedPacketContentKey(const RigExecBakedProgramImpl &,
    uint32_t slot, std::string *key);
/// Whether \p key, the value's stored key, still describes the state it is
/// keyed from: RigExecBakedOpValueKey's answer now, or, for the RevisionOut
/// of an unchunked revision whose fuse swapped staging into `output`, the
/// same key over `output`. What a retained op (an adopted skin body, a
/// frozen clone) is checked by.
bool RigExecBakedOpValueKeyStands(const RigExecBakedProgramImpl &,
    RigExecBakedSlotDomain, uint32_t slot, const std::string &key);
}
#endif
