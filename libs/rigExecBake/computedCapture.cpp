// .rigexec input collection.
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/arrayReads.h"
#include "rigExecBake/pathTable.h"
#include "rigExecBake/staticCapture.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frozenContextInternal.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/rigEvaluatorInternal.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/resolveInfo.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cstring>
#include <functional>
#include <iterator>
#include <map>
#include <numeric>
#include <set>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

using InputTag = fb::InputTag;

uint8_t
_Bit(fb::InputReadFlags flag)
{
    return uint8_t(flag);
}

uint8_t
_Bit(fb::InputSlotFlags flag)
{
    return uint8_t(flag);
}

/// The slot type of \p attribute's value type: scalars, tokens,
/// 4x4 double matrices and 3-vectors of either precision, any role. False
/// for every other type, arrays included.
bool
_SlotTag(const UsdAttribute &attribute, InputTag *tag)
{
    const SdfValueTypeName typeName = attribute.GetTypeName();
    if (!typeName || typeName.IsArray()) {
        return false;
    }
    const TfType type = typeName.GetType();
    if (type == TfType::Find<double>()) {
        *tag = InputTag::Double;
    } else if (type == TfType::Find<float>()) {
        *tag = InputTag::Float;
    } else if (type == TfType::Find<bool>()) {
        *tag = InputTag::Bool;
    } else if (type == TfType::Find<int>()) {
        *tag = InputTag::Int;
    } else if (type == TfType::Find<TfToken>()) {
        *tag = InputTag::Token;
    } else if (type == TfType::Find<GfMatrix4d>()) {
        *tag = InputTag::Matrix4d;
    } else if (type == TfType::Find<GfVec3d>()) {
        *tag = InputTag::Vec3d;
    } else if (type == TfType::Find<GfVec3i>()) {
        *tag=InputTag::Vec3i;
    } else if (type == TfType::Find<GfVec3f>()) {
        *tag = InputTag::Vec3f;
    } else {
        return false;
    }
    return true;
}

/// A value of \p tag holding zero: the member the tag names is allocated
/// and zeroed, every other one stays null, as the pool stores it.
fb::RigExecWireValue
_Zero(InputTag tag)
{
    fb::RigExecWireValue value;
    value.tag = tag;
    switch (tag) {
    case InputTag::Matrix4d:
        value.matrix = std::make_unique<RigExecWireMatrix4d>();
        break;
    case InputTag::Vec3d:
        value.vec3d = std::make_unique<RigExecWireVec3d>();
        break;
    case InputTag::Vec3i:
        value.vec3i=std::make_unique<RigExecWireVec3i>();
        break;
    case InputTag::Vec3f:
        value.vec3f = std::make_unique<RigExecWireVec3f>();
        break;
    default:
        break;
    }
    return value;
}

fb::RigExecWireValue
_Double(double d)
{
    fb::RigExecWireValue value = _Zero(InputTag::Double);
    std::memcpy(&value.bits, &d, sizeof(d));
    return value;
}

fb::RigExecWireValue
_Float(float f)
{
    fb::RigExecWireValue value = _Zero(InputTag::Float);
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(f));
    value.bits = bits;
    return value;
}

fb::RigExecWireValue
_Bool(bool b)
{
    fb::RigExecWireValue value = _Zero(InputTag::Bool);
    value.bits = b ? 1 : 0;
    return value;
}

fb::RigExecWireValue
_Identity()
{
    fb::RigExecWireValue value = _Zero(InputTag::Matrix4d);
    for (size_t i = 0; i < 4; ++i) {
        (*value.matrix)[i * 4 + i] = 1.0;
    }
    return value;
}

/// The empty array of array tag \p tag: pool entry 0 of its pool.
fb::RigExecWireValue
_EmptyArray(InputTag tag)
{
    fb::RigExecWireValue value;
    value.tag = tag;
    value.arraySource = fb::ArraySource::Pool;
    value.array = 0;
    return value;
}

fb::PropertyValueType
_PropertyType(RigExecBakedPropertyChainDesc::ValueType type)
{
    using T = RigExecBakedPropertyChainDesc::ValueType;
    switch (type) {
    case T::Double:
        return fb::PropertyValueType::Double;
    case T::Matrix4d:
        return fb::PropertyValueType::Matrix4d;
    case T::Vec3f:
        return fb::PropertyValueType::Vec3f;
    default:
        return fb::PropertyValueType::Float;
    }
}

fb::PropertyOp
_PropertyOp(bool valid, RigExecPropertyOp op)
{
    if (!valid) {
        return fb::PropertyOp::Invalid;
    }
    switch (op) {
    case RigExecPropertyOp::Add:
        return fb::PropertyOp::Add;
    case RigExecPropertyOp::Multiply:
        return fb::PropertyOp::Multiply;
    case RigExecPropertyOp::Clamp:
        return fb::PropertyOp::Clamp;
    case RigExecPropertyOp::Remap:
        return fb::PropertyOp::Remap;
    case RigExecPropertyOp::Blend:
        return fb::PropertyOp::Blend;
    case RigExecPropertyOp::Curve:
        return fb::PropertyOp::Curve;
    }
    return fb::PropertyOp::Invalid;
}

std::vector<RigExecWireVec2f>
_ToVec2fs(const std::vector<GfVec2f> &keys)
{
    std::vector<RigExecWireVec2f> out;
    out.reserve(keys.size());
    for (const GfVec2f &k : keys) {
        out.push_back(RigExecWireVec2f{k[0], k[1]});
    }
    return out;
}

/// Every read of a property revision.
template <class Visit>
void
_ForEachRevisionRead(fb::RigExecWirePropertyRevision &revision,
                     const Visit &visit)
{
    for (fb::RigExecWireInput *input :
         {revision.enabled.get(), revision.defaultWeight.get(),
          revision.value.get(), revision.min.get(), revision.max.get()}) {
        visit(*input);
    }
}

/// A property revision with its five reads allocated.
fb::RigExecWirePropertyRevision
_NewPropertyRevision()
{
    fb::RigExecWirePropertyRevision revision;
    for (std::unique_ptr<fb::RigExecWireInput> *input :
         {&revision.enabled, &revision.defaultWeight, &revision.value,
          &revision.min, &revision.max}) {
        *input = std::make_unique<fb::RigExecWireInput>();
    }
    return revision;
}

std::vector<RigExecWireVec3f>
_ToVec3fs(const std::vector<GfVec3f> &points)
{
    std::vector<RigExecWireVec3f> out;
    out.reserve(points.size());
    for (const GfVec3f &p : points) {
        out.push_back(RigExecWireVec3f{p[0], p[1], p[2]});
    }
    return out;
}

void
_ToAttributes(const std::vector<UsdAttribute> &attrs,
              RigExecBakePathTable *interner, std::vector<uint32_t> *paths,
              std::vector<uint8_t> *valid)
{
    paths->reserve(attrs.size());
    valid->reserve(attrs.size());
    for (const UsdAttribute &attr : attrs) {
        paths->push_back(attr ? interner->Path(attr.GetPath()) : 0);
        valid->push_back(attr ? uint8_t(1) : uint8_t(0));
    }
}

/// Every read slot of a weight object, in field order, so an allocation
/// or a walk rewrite cannot miss one.
template <class Visit>
void
_ForEachReadSlot(fb::RigExecWireWeightObject &object, const Visit &visit)
{
    for (std::unique_ptr<fb::RigExecWireInput> *input :
         {&object.defaultWeight, &object.driver, &object.scale, &object.bias,
          &object.strength, &object.invert, &object.falloffMin,
          &object.falloffMax, &object.scaleXPos, &object.scaleYPos,
          &object.scaleZPos, &object.scaleXNeg, &object.scaleYNeg,
          &object.scaleZNeg, &object.scaleX, &object.scaleY, &object.scaleZ,
          &object.extentU, &object.extentV}) {
        visit(*input);
    }
}

/// A weight object with its nineteen reads allocated.
fb::RigExecWireWeightObject
_NewWeightObject()
{
    fb::RigExecWireWeightObject object;
    _ForEachReadSlot(object, [](std::unique_ptr<fb::RigExecWireInput> &input) {
        input = std::make_unique<fb::RigExecWireInput>();
    });
    return object;
}

}  // namespace

struct RigExecBakeComputedCapture::_State {
    RigExecBakePathTable *interner = nullptr;
    UsdStageRefPtr stage;
    const RigExecBakedProgramImpl *program = nullptr;
    const std::set<SdfPath> *chainTargets = nullptr;
    UsdTimeCode bakeTime;
    RigExecBakeInputs computed;
    std::map<std::string, uint32_t> valueIds;
    /// Slots in first-reference order until Finish sorts them.
    std::map<SdfPath, uint32_t> slotIds;
    std::set<SdfPath> privateTokenPaths;
    std::vector<UsdAttribute> slots;
    std::vector<InputTag> slotTags;
    /// Paths each override number was registered under (the inverse of
    /// RigExecBakedProgramImpl::overridableInputs).
    std::vector<std::set<SdfPath>> registered;
    std::vector<RigExecBakeTimeVaryingFact> timeVaryingFacts;
    std::vector<std::string> listedNames;

    uint32_t Intern(const fb::RigExecWireValue &value)
    {
        const auto found = valueIds.emplace(RigExecBakeValueKey(value),
                                            uint32_t(computed.values.size()));
        if (found.second) {
            computed.values.push_back(value);
        }
        return found.first->second;
    }

    /// The raw typed read of a slot attribute at \p time: no walk, no
    /// overlay. A failed read holds the type's zero and says so.
    uint32_t ReadSlot(const UsdAttribute &a, InputTag tag, UsdTimeCode time,
                      bool *has)
    {
        fb::RigExecWireValue value = _Zero(tag);
        switch (tag) {
        case InputTag::Double: {
            double v = 0.0;
            if ((*has = a.Get(&v, time))) {
                value = _Double(v);
            }
            break;
        }
        case InputTag::Float: {
            float v = 0.0f;
            if ((*has = a.Get(&v, time))) {
                value = _Float(v);
            }
            break;
        }
        case InputTag::Bool: {
            bool v = false;
            if ((*has = a.Get(&v, time))) {
                value.bits = v ? 1 : 0;
            }
            break;
        }
        case InputTag::Int: {
            int v = 0;
            if ((*has = a.Get(&v, time))) {
                value.bits = uint32_t(v);
            }
            break;
        }
        case InputTag::Token: {
            TfToken v;
            if ((*has = a.Get(&v, time))) {
                value.bits = interner->Token(v);
            }
            break;
        }
        case InputTag::Matrix4d: {
            GfMatrix4d v(1.0);
            if ((*has = a.Get(&v, time))) {
                for (int r = 0; r < 4; ++r) {
                    for (int c = 0; c < 4; ++c) {
                        (*value.matrix)[size_t(r * 4 + c)] = v[r][c];
                    }
                }
            }
            break;
        }
        case InputTag::Vec3d: {
            GfVec3d v(0.0);
            if ((*has = a.Get(&v, time))) {
                *value.vec3d = RigExecWireVec3d{v[0], v[1], v[2]};
            }
            break;
        }
        case InputTag::Vec3i: {
            GfVec3i v(0);
            if((*has=a.Get(&v,time))) *value.vec3i=RigExecWireVec3i{v[0],v[1],v[2]};
            break;
        }
        case InputTag::Vec3f: {
            GfVec3f v(0.0f);
            if ((*has = a.Get(&v, time))) {
                *value.vec3f = RigExecWireVec3f{v[0], v[1], v[2]};
            }
            break;
        }
        default:
            // An array slot's default waits for the bake's run.
            *has = false;
            break;
        }
        return Intern(value);
    }

    /// The slot of the attribute at \p path (provisional id until the
    /// slots are sorted), added on first use. \p reader names who reads it
    /// in the refusal.
    bool Slot(const SdfPath &path, const std::string &reader, uint32_t *id,
              std::string *error)
    {
        auto found = slotIds.find(path);
        if (found == slotIds.end()) {
            const UsdAttribute a = stage->GetAttributeAtPath(path);
            InputTag tag = InputTag::Double;
            if (a && a.GetTypeName().IsArray()) {
                const auto type=a.GetTypeName().GetType();
                if(type==TfType::Find<VtVec3fArray>()) return ArraySlot(path,InputTag::Vec3fArray,id,error);
                if(type==TfType::Find<VtVec2fArray>()) return ArraySlot(path,InputTag::Vec2fArray,id,error);
                if(type==TfType::Find<VtFloatArray>()) return ArraySlot(path,InputTag::FloatArray,id,error);
                if(type==TfType::Find<VtDoubleArray>()) return ArraySlot(path,InputTag::DoubleArray,id,error);
                if(type==TfType::Find<VtIntArray>()) return ArraySlot(path,InputTag::IntArray,id,error);
                if(type==TfType::Find<VtVec3dArray>()) return ArraySlot(path,InputTag::Vec3dArray,id,error);
                if(type==TfType::Find<VtMatrix4dArray>()) return ArraySlot(path,InputTag::Matrix4dArray,id,error);
                if(type==TfType::Find<VtTokenArray>()) return ArraySlot(path,InputTag::TokenArray,id,error);
                if(type==TfType::Find<VtBoolArray>()) return ArraySlot(path,InputTag::BoolArray,id,error);
            }
            if (!a || !_SlotTag(a, &tag)) {
                *error = reader + " reads through " + path.GetString() +
                         ", whose value type no input slot can hold";
                return false;
            }
            found = slotIds.emplace(path, uint32_t(slots.size())).first;
            slots.push_back(a);
            slotTags.push_back(tag);
        }
        *id = found->second;
        return true;
    }

    /// The slot of array attribute \p path, of \p tag (provisional id until
    /// the slots are sorted), added on first use; false when another read
    /// takes it as another type.
    bool ArraySlot(const SdfPath &path, InputTag tag, uint32_t *id,
                   std::string *error)
    {
        auto found = slotIds.find(path);
        if (found == slotIds.end()) {
            found = slotIds.emplace(path, uint32_t(slots.size())).first;
            slots.push_back(stage->GetAttributeAtPath(path));
            slotTags.push_back(tag);
        } else if (slotTags[found->second] != tag) {
            *error = "the program reads " + path.GetString() +
                     " as two types";
            return false;
        }
        *id = found->second;
        return true;
    }

    /// The walk GetAttribute takes from \p head, as slot ids (provisional
    /// until the slots are sorted), with what the classifier saw on it.
    bool Walk(const UsdAttribute &head, fb::RigExecWireInput *out,
              SdfPathVector *paths, std::string *error)
    {
        bool viaChain = false, varying = false;
        UsdAttribute selected;
        RigExecBakedClassifyInput<float>(head, bakeTime, *chainTargets,
                                         &viaChain, &varying, &selected,
                                         paths);
        out->walk.clear();
        out->walk.reserve(paths->size());
        const std::string reader = "input " + head.GetPath().GetString();
        for (const SdfPath &path : *paths) {
            uint32_t id = 0;
            if (!Slot(path, reader, &id, error)) {
                return false;
            }
            out->walk.push_back(id);
        }
        if (viaChain) {
            out->flags |= _Bit(fb::InputReadFlags::ViaChain);
        }
        if (varying && out->mode != fb::ReadMode::Baked) {
            out->flags |= _Bit(fb::InputReadFlags::Varying);
        }
        return true;
    }

    /// A registered input's constant, tagged with the input's type.
    fb::RigExecWireValue Value(float v) { return _Float(v); }
    fb::RigExecWireValue Value(double v) { return _Double(v); }
    fb::RigExecWireValue Value(bool v) { return _Bool(v); }
    fb::RigExecWireValue Value(int v)
    {
        fb::RigExecWireValue value = _Zero(InputTag::Int);
        value.bits = uint32_t(v);
        return value;
    }
    fb::RigExecWireValue Value(const TfToken &v)
    {
        fb::RigExecWireValue value = _Zero(InputTag::Token);
        value.bits = interner->Token(v);
        return value;
    }
    fb::RigExecWireValue Value(const GfMatrix4d &v)
    {
        fb::RigExecWireValue value = _Zero(InputTag::Matrix4d);
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                (*value.matrix)[size_t(r * 4 + c)] = v[r][c];
            }
        }
        return value;
    }
    fb::RigExecWireValue Value(const GfVec3d &v)
    {
        fb::RigExecWireValue value = _Zero(InputTag::Vec3d);
        *value.vec3d = RigExecWireVec3d{v[0], v[1], v[2]};
        return value;
    }

    bool Candidates(const RigExecBakedWalk &walk, fb::RigExecWireInput *out,
                    std::string *error, int rawLeaf = -1)
    {
        const auto capture = [&](const auto &hops, auto *result) {
            for (const auto &hop : hops) {
                RigExecWirePropertyInputCandidate candidate;
                if (!Slot(hop.path, "property input", &candidate.slot, error))
                    return false;
                candidate.raw = hop.leaf >= 0;
                candidate.crossDomain = hop.crossDomain;
                if (hop.chain >= 0) {
                    const auto &chain = program->propertyChains[size_t(hop.chain)];
                    candidate.kind = uint8_t(fb::PropertyCandidateKind::ChainFinal);
                    candidate.version = int32_t(chain.versionBase + chain.revisions.size());
                } else if (hop.record >= 0) {
                    candidate.kind = uint8_t(fb::PropertyCandidateKind::PhasedRecord);
                    candidate.version = int32_t(program->propertyRecords[size_t(hop.record)].id);
                }
                result->push_back(candidate);
            }
            return true;
        };
        if (!capture(walk.hops, &out->propertyCandidates) ||
            !capture(walk.doubleHops, &out->doubleCandidates)) return false;
        if (rawLeaf >= 0) {
            uint32_t slot = 0;
            if (!Slot(program->headLeaves[size_t(rawLeaf)].path,
                      "property input fallback", &slot, error)) return false;
            out->rawFallbackSlot = int32_t(slot);
        }
        return true;
    }

    bool OracleRead(const RigExecBakedReaderWalk &read,
                    const std::vector<int> &available, float fallback,
                    fb::RigExecWireInput *out, std::string *error)
    {
        *out = fb::RigExecWireInput();
        out->tag = InputTag::Float;
        out->mode = fb::ReadMode::Resolved;
        out->constant = Intern(_Float(fallback));
        const auto capture = [&](const auto &hops, auto *candidates) {
            for (const auto &hop : hops) {
                RigExecWirePropertyInputCandidate candidate;
                if (!Slot(hop.path, "weight field oracle", &candidate.slot, error)) return false;
                candidate.raw = hop.leaf >= 0;
                candidate.poseWeight = hop.poseWeight;
                // Actual publication precedence in the field's context.
                // Runtime chooses the last VALID writer, so all available
                // chains are retained independently in the descriptor.
                for (int c : available) {
                    const auto &chain = program->propertyChains[size_t(c)];
                    if (chain.target == hop.path) {
                        candidate.kind = uint8_t(fb::PropertyCandidateKind::ChainFinal);
                        candidate.version = int32_t(chain.versionBase + chain.revisions.size());
                    }
                    for (uint32_t r : chain.records) {
                        const auto &record = program->propertyRecords[r];
                        if (record.consumer == hop.path) {
                            candidate.kind = uint8_t(fb::PropertyCandidateKind::PhasedRecord);
                            candidate.version = int32_t(record.id);
                        }
                    }
                }
                candidates->push_back(candidate);
                if (std::find(out->walk.begin(), out->walk.end(), candidate.slot) == out->walk.end())
                    out->walk.push_back(candidate.slot);
            }
            return true;
        };
        if (!capture(read.walk.hops, &out->propertyCandidates) ||
            !capture(read.walk.doubleHops, &out->doubleCandidates)) return false;
        if (read.rawLeaf >= 0) {
            uint32_t slot = 0;
            if (!Slot(program->headLeaves[size_t(read.rawLeaf)].path,
                      "weight field fallback", &slot, error)) return false;
            out->rawFallbackSlot = int32_t(slot);
        }
        return true;
    }

    // Oracle/path reads use GetAttribute's published overlay, not a
    // patchable binding's phased walk. Freeze that traversal independently;
    // runtime gates its version candidates by this generation's publication.
    bool ResolvedCandidates(const UsdAttribute &head, fb::RigExecWireInput *out,
                            std::string *error)
    {
        if (!head || !program) return true;
        const auto hop = [&](const UsdAttribute &attribute, InputTag type,
                             bool raw, auto *result) {
            RigExecWirePropertyInputCandidate candidate;
            if (!Slot(attribute.GetPath(), "resolved input", &candidate.slot, error))
                return false;
            candidate.raw = raw;
            for (const auto &chain : program->propertyChains) {
                if (chain.target != attribute.GetPath()) continue;
                candidate.kind = uint8_t(fb::PropertyCandidateKind::ChainFinal);
                candidate.version = int32_t(chain.versionBase + chain.revisions.size());
                break;
            }
            if (candidate.version < 0) {
                for (const auto &record : program->propertyRecords) {
                    if (record.consumer != attribute.GetPath()) continue;
                    candidate.kind = uint8_t(fb::PropertyCandidateKind::PhasedRecord);
                    candidate.version = int32_t(record.id);
                    break;
                }
            }
            result->push_back(candidate);
            return true;
        };
        const auto next = [&](const UsdAttribute &attribute) {
            SdfPathVector connections;
            if (attribute.HasAuthoredConnections()) attribute.GetConnections(&connections);
            return connections.size() == 1 ? stage->GetAttributeAtPath(connections[0])
                                           : UsdAttribute();
        };
        const auto doubleTail = [&](const UsdAttribute &from) {
            std::set<SdfPath> visiting;
            for (UsdAttribute a = from; a && visiting.insert(a.GetPath()).second; a = next(a))
                if (!hop(a, InputTag::Double, true, &out->doubleCandidates)) return false;
            return true;
        };
        if (out->tag == InputTag::Float && head.GetTypeName() == SdfValueTypeNames->Double)
            return doubleTail(head);
        std::set<SdfPath> visiting;
        for (UsdAttribute a = head; a && visiting.insert(a.GetPath()).second; a = next(a)) {
            if (!hop(a, out->tag, true, &out->propertyCandidates)) return false;
            if (out->tag == InputTag::Float && a.GetTypeName() == SdfValueTypeNames->Double) {
                for (auto &candidate : out->propertyCandidates) candidate.raw = false;
                return doubleTail(a);
            }
        }
        return true;
    }

    /// A registered program read, as RigExecBakedRead resolves it.
    template <class T>
    bool Baked(const RigExecBakedInput<T> &input, fb::RigExecWireInput *out,
               std::string *error)
    {
        *out = fb::RigExecWireInput();
        const fb::RigExecWireValue constant =
            Value(input.sourceBacked ? input.sourceFallback : input.constant);
        out->tag = constant.tag;
        out->mode = fb::ReadMode::Baked;
        out->overrideIndex = int32_t(input.overrideIndex);
        out->constant = Intern(constant);
        if (input.varying) {
            out->flags |= _Bit(fb::InputReadFlags::Varying);
        }
        if (input.resolvedAttr) {
            out->flags |= _Bit(fb::InputReadFlags::LongWay);
        }
        if (!input.head) {
            if (input.sourceBacked) {
                *error = "a source-backed input has no head";
                return false;
            }
            if (input.overrideIndex >= 0) {
                *error = "a registered input has no head";
                return false;
            }
            return true;
        }
        const std::string head = input.head.GetPath().GetString();
        SdfPathVector paths;
        if (!Walk(input.head, out, &paths, error)) {
            return false;
        }
        if (input.sourceBacked) {
            if (input.varying || input.resolvedAttr || input.walk >= 0 ||
                paths.size() != 1 || input.head.HasAuthoredConnections() ||
                out->flags != 0 ||
                (out->tag != InputTag::Double && out->tag != InputTag::Vec3d) ||
                slotTags[out->walk[0]] != out->tag) {
                *error = "input " + head + " is not a direct static source-backed read";
                return false;
            }
            out->flags |= _Bit(fb::InputReadFlags::SourceBacked);
            out->selected = 0;
        }
        if (input.varying && !input.resolvedAttr && input.query.IsValid()) {
            const SdfPath pinned = input.query.GetAttribute().GetPath();
            const auto at = std::find(paths.begin(), paths.end(), pinned);
            if (at == paths.end() || paths.size() > 32767) {
                *error = "input " + head + " is pinned to " +
                         pinned.GetString() + ", which is not on its walk";
                return false;
            }
            out->selected = int16_t(at - paths.begin());
        }
        // The walk must be the one Build registered the override number
        // under (RigExecBakedRecordBind), or the runtime would read
        // different attributes than the program does.
        if (input.overrideIndex >= 0) {
            const size_t number = size_t(input.overrideIndex);
            const std::set<SdfPath> mine(paths.begin(), paths.end());
            if (number >= registered.size() || mine != registered[number]) {
                *error = "the walk of input " + head +
                         " is not the one its override number was "
                         "registered under";
                return false;
            }
        }
        if (input.walk >= 0) {
            const auto &reader = program->readerWalks[size_t(input.walk)];
            return Candidates(reader.walk, out, error, reader.rawLeaf);
        }
        return true;
    }

    /// RigExecResolvedInputs::GetAttribute from \p head, typed and falling
    /// back as \p fallback: the oracle's _ResolvedRead, and every assembly
    /// read that resolves through the generation's inputs.
    bool Resolved(const UsdAttribute &head,
                  const fb::RigExecWireValue &fallback,
                  fb::RigExecWireInput *out, std::string *error)
    {
        *out = fb::RigExecWireInput();
        out->tag = fallback.tag;
        out->mode = fb::ReadMode::Resolved;
        out->constant = Intern(fallback);
        if (!head) {
            return true;
        }
        SdfPathVector paths;
        return Walk(head, out, &paths, error) &&
               ResolvedCandidates(head, out, error);
    }

    /// An envelope read, as the oracle's _ResolvedRead resolves it.
    bool Resolved(const UsdAttribute &head, float fallback,
                  fb::RigExecWireInput *out, std::string *error)
    {
        return Resolved(head, _Float(fallback), out, error);
    }

    /// A property-mover input, as _PinnedRead resolves it: a connected
    /// attribute through GetAttribute's walk (Resolved), an unconnected one
    /// from its own path (Pinned over the attribute alone), a missing one
    /// as \p fallback (an empty Pinned walk).
    bool Property(const UsdAttribute &head, const fb::RigExecWireValue &fallback,
                  fb::RigExecWireInput *out, std::string *error)
    {
        *out = fb::RigExecWireInput();
        out->tag = fallback.tag;
        out->mode = fb::ReadMode::Pinned;
        out->constant = Intern(fallback);
        if (!head) {
            return true;
        }
        if (head.HasAuthoredConnections()) {
            out->mode = fb::ReadMode::Resolved;
        }
        SdfPathVector paths;
        if (!Walk(head, out, &paths, error)) {
            return false;
        }
        if (out->mode == fb::ReadMode::Pinned && out->walk.size() != 1) {
            *error = "input " + head.GetPath().GetString() +
                     " has no connection but its walk leaves it";
            return false;
        }
        return true;
    }
};

RigExecBakeComputedCapture::RigExecBakeComputedCapture(
    RigExecRigEvaluator &evaluator, double time,
    RigExecBakePathTable *interner, std::string *error)
    : _state(std::make_unique<_State>())
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
    };
    const RigExecBakedProgram *baked = evaluator.GetBakedProgram();
    if (!baked || !interner) {
        Fail("no baked program standing to capture from");
        return;
    }
    const RigExecBakedProgramImpl &B = baked->GetStepGraph();
    _State &S = *_state;
    S.interner = interner;
    S.stage = B.stage;
    S.program = &B;
    S.chainTargets = &B.chainTargets;
    S.bakeTime = UsdTimeCode(time);
    RigExecBakeInputs &C = S.computed;
    S.Intern(_Double(0.0));
    S.registered.resize(B.overridden.size());
    for (const auto &[path, numbers] : B.overridableInputs) {
        for (int number : numbers) {
            if (number >= 0 && size_t(number) < S.registered.size()) {
                S.registered[size_t(number)].insert(path);
            }
        }
    }

    std::vector<RigExecBakedEnvelopeObject> envelopes;
    std::map<SdfPath, int> envelopeIndex;
    std::string why;
    if (!RigExecBakedComposeEnvelopeObjects(evaluator, B, &envelopes,
                                            &envelopeIndex, &why)) {
        Fail("cannot compose a weight envelope: " + why);
        return;
    }

    // The step-backed objects, in the program's order, read as the program
    // reads them.
    const auto token = [&](const TfToken &t) {
        return interner->Token(t);
    };
    C.weightObjects.reserve(B.weightObjects.size() + envelopes.size());
    for (const RigExecBakedProgramImpl::WeightObject &object :
         B.weightObjects) {
        fb::RigExecWireWeightObject wire = _NewWeightObject();
        wire.path = interner->Path(object.path);
        wire.type = token(object.type);
        wire.representation = token(object.representation);
        wire.rangePolicy = token(object.rangePolicy);
        wire.base = int32_t(object.base);
        wire.inputs.assign(object.inputs.begin(), object.inputs.end());
        wire.combineMode = token(object.combineMode);
        _ToAttributes(object.combineTargetPoints, interner,
                      &wire.combineTargetPoints, &wire.combineTargetValid);
        wire.costElements = uint64_t(object.costElements);
        wire.providerSlot = int32_t(object.providerSlot);
        wire.planeAxis = token(object.planeAxis);
        wire.planeBounds = token(object.planeBounds);
        _ToAttributes(object.targetPoints, interner, &wire.targetPoints,
                      &wire.targetValid);
        _ToAttributes(object.samplePoints, interner, &wire.samplePoints,
                      &wire.sampleValid);
        _ToAttributes(object.curvePoints, interner, &wire.curvePoints,
                      &wire.curveValid);
        wire.falloffCurve = object.falloffCurve;
        const std::pair<const RigExecBakedInput<float> *,
                        fb::RigExecWireInput *>
            reads[] = {
                {&object.defaultWeight, wire.defaultWeight.get()},
                {&object.driver, wire.driver.get()},
                {&object.scale, wire.scale.get()},
                {&object.bias, wire.bias.get()},
                {&object.strength, wire.strength.get()},
                {&object.invert, wire.invert.get()},
                {&object.falloffMin, wire.falloffMin.get()},
                {&object.falloffMax, wire.falloffMax.get()},
                {&object.scaleXPos, wire.scaleXPos.get()},
                {&object.scaleYPos, wire.scaleYPos.get()},
                {&object.scaleZPos, wire.scaleZPos.get()},
                {&object.scaleXNeg, wire.scaleXNeg.get()},
                {&object.scaleYNeg, wire.scaleYNeg.get()},
                {&object.scaleZNeg, wire.scaleZNeg.get()},
                {&object.scaleX, wire.scaleX.get()},
                {&object.scaleY, wire.scaleY.get()},
                {&object.scaleZ, wire.scaleZ.get()},
                {&object.extentU, wire.extentU.get()},
                {&object.extentV, wire.extentV.get()},
            };
        for (const auto &read : reads) {
            if (!S.Baked(*read.first, read.second, &why)) {
                Fail(why);
                return;
            }
        }
        C.weightObjects.push_back(std::move(wire));
    }

    // The envelope-only objects, read as the oracle reads them. A scalar
    // object reads none of the volume fields; they carry the oracle's
    // fallbacks over an empty walk so every entry has one shape.
    for (const RigExecBakedEnvelopeObject &object : envelopes) {
        fb::RigExecWireWeightObject wire = _NewWeightObject();
        wire.envelopeOnly = true;
        wire.path = interner->Path(object.path);
        wire.type = token(object.type);
        wire.representation = token(object.representation);
        wire.rangePolicy = token(object.rangePolicy);
        wire.base = int32_t(object.base);
        wire.inputs.assign(object.inputs.begin(), object.inputs.end());
        wire.combineMode = token(object.combineMode);
        wire.costElements = uint64_t(std::max<size_t>(object.values.size(), 1));
        const std::pair<const RigExecBakedEnvelopeObject::Read *,
                        fb::RigExecWireInput *>
            reads[] = {
                {&object.defaultWeight, wire.defaultWeight.get()},
                {&object.driver, wire.driver.get()},
                {&object.scale, wire.scale.get()},
                {&object.bias, wire.bias.get()},
                {&object.strength, wire.strength.get()},
                {&object.invert, wire.invert.get()},
            };
        for (const auto &read : reads) {
            if (!S.Resolved(read.first->head, read.first->fallback,
                            read.second, &why)) {
                Fail(why);
                return;
            }
        }
        const std::pair<fb::RigExecWireInput *, float> unread[] = {
            {wire.falloffMin.get(), 0.0f}, {wire.falloffMax.get(), 1.0f},
            {wire.scaleXPos.get(), 1.0f},  {wire.scaleYPos.get(), 1.0f},
            {wire.scaleZPos.get(), 1.0f},  {wire.scaleXNeg.get(), 1.0f},
            {wire.scaleYNeg.get(), 1.0f},  {wire.scaleZNeg.get(), 1.0f},
            {wire.scaleX.get(), 1.0f},     {wire.scaleY.get(), 1.0f},
            {wire.scaleZ.get(), 1.0f},     {wire.extentU.get(), 1.0f},
            {wire.extentV.get(), 1.0f},
        };
        for (const auto &read : unread) {
            S.Resolved(UsdAttribute(), read.second, read.first, &why);
        }
        C.weightObjects.push_back(std::move(wire));
    }

    // The oracle facts, for every entry; the points it samples are input
    // slots, listed with the other array reads below.
    std::vector<SdfPath> objectPaths(C.weightObjects.size());
    std::vector<TfToken> objectTypes(C.weightObjects.size());
    std::vector<SdfPath> timeVarying(C.weightObjects.size());
    std::vector<RigExecWeightOracleFacts> oracleFacts(C.weightObjects.size());
    for (size_t i = 0; i < C.weightObjects.size(); ++i) {
        const bool stepBacked = i < B.weightObjects.size();
        const SdfPath &path =
            stepBacked ? B.weightObjects[i].path
                       : envelopes[i - B.weightObjects.size()].path;
        const TfToken &type =
            stepBacked ? B.weightObjects[i].type
                       : envelopes[i - B.weightObjects.size()].type;
        RigExecWeightOracleFacts &facts = oracleFacts[i];
        RigExecBakedDescribeWeightOracle(evaluator, path, S.bakeTime, &facts);
        fb::RigExecWireWeightObject &wire = C.weightObjects[i];
        wire.samplesInFlight = facts.samplesInFlight;
        if (type == TfToken("RigExecPlaneWeight")) {
            wire.oraclePlaneAxis = token(facts.planeAxis);
            wire.oraclePlaneBounds = token(facts.planeBounds);
            const auto prim = B.stage->GetPrimAtPath(path);
            for (const auto &entry : {std::make_pair("rigExec:planeAxis", &wire.oraclePlaneAxisSlot),
                                     std::make_pair("rigExec:planeBounds", &wire.oraclePlaneBoundsSlot)}) {
                const auto attr = prim.GetAttribute(TfToken(entry.first));
                if (attr) {
                    uint32_t slot = 0;
                    if (!S.Slot(attr.GetPath(), "oracle token", &slot, &why)) {
                        Fail(why); return;
                    }
                    *entry.second = int32_t(slot);
                    S.privateTokenPaths.insert(attr.GetPath());
                }
            }
        }
        wire.oraclePhaseError = facts.phaseError;
        wire.oracleStaticError = facts.staticError;
        objectPaths[i] = path;
        objectTypes[i] = type;
        timeVarying[i] = facts.timeVarying;
    }

    // The envelope arm's own condition (bakedPose.cpp, the constraint
    // step): a weight object and no points target.
    C.constraintWeightObjectIndex.reserve(B.constraints.size());
    for (const RigExecBakedProgramImpl::Constraint &c : B.constraints) {
        int32_t index = -1;
        if (!c.weightObject.IsEmpty() && c.pointsTarget.IsEmpty()) {
            const auto baked = B.weightIndex.find(c.weightObject);
            const auto composed = envelopeIndex.find(c.weightObject);
            if (baked != B.weightIndex.end() && baked->second >= 0) {
                index = int32_t(baked->second);
            } else if (composed != envelopeIndex.end() &&
                       composed->second >= 0) {
                index = int32_t(composed->second);
            } else {
                Fail("constraint " + c.path.GetString() +
                     " has no envelope entry for " +
                     c.weightObject.GetString());
                return;
            }
        }
        C.constraintWeightObjectIndex.push_back(index);
    }

    // The property chains (rigEvaluatorProperties.cpp runChain): each
    // chain's target slot, each revision's reads as _PinnedRead resolves
    // them and its envelope's entry, then the phased consumers' slots.
    std::vector<RigExecBakedPropertyChainDesc> chainDescs;
    using Desc = RigExecBakedPropertyChainDesc;
    const auto descType = [](const SdfValueTypeName &type) {
        return type == SdfValueTypeNames->Float ? Desc::ValueType::Float
            : type == SdfValueTypeNames->Double ? Desc::ValueType::Double
            : type == SdfValueTypeNames->Matrix4d ? Desc::ValueType::Matrix4d
            : Desc::ValueType::Vec3f;
    };
    const auto headOf = [&](const RigExecBakedWalk &walk) {
        const auto &hops = walk.hops.empty() ? walk.doubleHops : walk.hops;
        return hops.empty() ? UsdAttribute()
                           : B.stage->GetAttributeAtPath(hops.front().path);
    };
    for (const auto &bound : B.propertyChains) {
        Desc desc;
        desc.target = bound.target;
        desc.targetAttr = B.stage->GetAttributeAtPath(bound.target);
        desc.valueType = descType(bound.valueType);
        for (const auto &r : bound.revisions) {
            Desc::Revision revision;
            revision.mover = r.mover;
            revision.opValid = r.opValid;
            revision.op = r.op;
            revision.weightObject = r.weightObject;
            revision.enabled = headOf(r.enabled);
            revision.defaultWeight = headOf(r.defaultWeight);
            revision.value = headOf(r.value);
            revision.minimum = headOf(r.minimum);
            revision.maximum = headOf(r.maximum);
            if (r.opValid && r.op == RigExecPropertyOp::Curve) {
                VtVec2fArray values;
                for (const auto &a : {headOf(r.keys), headOf(r.tangents)}) {
                    if (RigExecBakedAnimatedOrConnected(a)) {
                        Fail("property mover " + r.mover.GetString() + " reads " +
                             a.GetPath().GetString() + ", which is animated or connected; "
                             "a baked curve holds its keys at one time only");
                        return;
                    }
                }
                if (headOf(r.keys).Get(&values))
                    revision.keys.assign(values.begin(), values.end());
                values.clear();
                if (headOf(r.tangents).Get(&values))
                    revision.tangents.assign(values.begin(), values.end());
                revision.hasTangentsAttr = r.hasTangents;
            }
            desc.revisions.push_back(std::move(revision));
        }
        for (const auto index : bound.records) {
            const auto &record = B.propertyRecords[index];
            Desc::Phased phased;
            phased.consumer = record.consumer;
            phased.consumerType = descType(record.consumerType);
            phased.applied = record.applied;
            for (const int slot : record.hopSlots) {
                for (const auto &[path, id] : B.headOverrideSlots)
                    if (int(id) == slot) { phased.hops.push_back(path); break; }
            }
            desc.phased.push_back(std::move(phased));
        }
        chainDescs.push_back(std::move(desc));
    }
    for (const auto &field : B.weightFields) {
        fb::RigExecWireWeightField out;
        out.form = fb::WeightFieldForm(field.form);
        out.object = field.object;
        out.consumer = field.consumer;
        out.part = field.part;
        out.placementPhase = fb::WeightFieldPlacementPhase(field.placementPhase);
        out.volumes.assign(field.volumes.begin(), field.volumes.end());
        out.availableChains.assign(field.availableChains.begin(), field.availableChains.end());
        out.scalarObjects.assign(field.scalarObjects.begin(), field.scalarObjects.end());
        for (int member : field.scalarMembers)
            out.scalarMembers.push_back(fb::WeightFieldScalarMember(member));
        for (size_t r = 0; r < field.scalarReads.size(); ++r) {
            const int member = field.scalarMembers[r];
            const float fallback = member == 0 || member == 3 || member == 5 || member == 6 ? 0.0f : 1.0f;
            fb::RigExecWireInput read;
            if (!S.OracleRead(field.scalarReads[r], field.availableChains, fallback, &read, &why)) {
                Fail(why); return;
            }
            out.scalarReads.push_back(std::move(read));
        }
        C.weightFields.push_back(std::move(out));
    }
    C.propertyChains.reserve(chainDescs.size());
    for (const RigExecBakedPropertyChainDesc &desc : chainDescs) {
        using ValueType = RigExecBakedPropertyChainDesc::ValueType;
        fb::RigExecWirePropertyChain chain;
        const auto &boundChain = B.propertyChains[C.propertyChains.size()];
        chain.versionBase = boundChain.versionBase;
        chain.valueType = _PropertyType(desc.valueType);
        if (!S.Slot(desc.target, "property chain " + desc.target.GetString(),
                    &chain.target, &why)) {
            Fail(why);
            return;
        }
        // value, min and max as the chain's arm reads them: float for a
        // double chain too, which is computed in float.
        fb::RigExecWireValue operand = _Float(0.0f);
        if (desc.valueType == ValueType::Vec3f) {
            operand = _Zero(InputTag::Vec3f);
        } else if (desc.valueType == ValueType::Matrix4d) {
            operand = _Identity();
        }
        for (const RigExecBakedPropertyChainDesc::Revision &r :
             desc.revisions) {
            fb::RigExecWirePropertyRevision revision =
                _NewPropertyRevision();
            revision.mover = interner->Path(r.mover);
            revision.op = _PropertyOp(r.opValid, r.op);
            const bool matrix = desc.valueType == ValueType::Matrix4d;
            if (!S.Property(r.enabled, _Bool(true), revision.enabled.get(),
                            &why) ||
                !S.Property(r.defaultWeight, _Float(1.0f),
                            revision.defaultWeight.get(), &why) ||
                !S.Property(r.value, operand, revision.value.get(), &why) ||
                !S.Property(matrix ? UsdAttribute() : r.minimum, operand,
                            revision.min.get(), &why) ||
                !S.Property(matrix ? UsdAttribute() : r.maximum, operand,
                            revision.max.get(), &why)) {
                Fail(why);
                return;
            }
            const auto &bound = boundChain.revisions[chain.revisions.size()];
            revision.weightField = bound.weightField;
            const bool boundOk =
                S.Candidates(bound.enabled, revision.enabled.get(), &why) &&
                S.Candidates(bound.defaultWeight, revision.defaultWeight.get(), &why) &&
                S.Candidates(bound.value, revision.value.get(), &why) &&
                S.Candidates(bound.minimum, revision.min.get(), &why) &&
                S.Candidates(bound.maximum, revision.max.get(), &why);
            if (!boundOk) { Fail(why); return; }
            if (!r.weightObject.IsEmpty()) {
                const auto baked = B.weightIndex.find(r.weightObject);
                const auto composed = envelopeIndex.find(r.weightObject);
                if (baked != B.weightIndex.end() && baked->second >= 0) {
                    revision.envelope = int32_t(baked->second);
                } else if (composed != envelopeIndex.end() &&
                           composed->second >= 0) {
                    revision.envelope = int32_t(composed->second);
                } else {
                    Fail("property mover " + r.mover.GetString() +
                         " has no envelope entry for " +
                         r.weightObject.GetString());
                    return;
                }
            }
            revision.keys = _ToVec2fs(r.keys);
            revision.hasTangentsAttr = r.hasTangentsAttr;
            revision.tangents = _ToVec2fs(r.tangents);
            chain.revisions.push_back(std::move(revision));
        }
        C.propertyChains.push_back(std::move(chain));
    }
    // The connected readers' records (RigExecPhasedConnection: the base
    // unless a reader declares another phase). Compile admits a reader only
    // of the chain's value type, or float and double, so every consumer is
    // a float, double, matrix4d or 3-float slot, as every chain target is.
    for (size_t c = 0; c < chainDescs.size(); ++c) {
        for (const RigExecBakedPropertyChainDesc::Phased &p :
             chainDescs[c].phased) {
            fb::RigExecWirePhasedConsumer phased;
            phased.chain = uint32_t(c);
            phased.version = B.propertyRecords[C.phasedConsumers.size()].id;
            phased.consumerType = _PropertyType(p.consumerType);
            phased.applied = uint32_t(p.applied);
            if (!S.Slot(p.consumer,
                        "phased consumer " + p.consumer.GetString(),
                        &phased.consumer, &why)) {
                Fail(why);
                return;
            }
            // The attributes an override stands the reader aside on.
            phased.hops.reserve(p.hops.size());
            for (const SdfPath &hop : p.hops) {
                uint32_t id = 0;
                if (!S.Slot(hop, "phased consumer " + p.consumer.GetString(),
                            &id, &why)) {
                    Fail(why);
                    return;
                }
                phased.hops.push_back(id);
            }
            C.phasedConsumers.push_back(phased);
        }
    }

    // A registered read that crosses a chain target reads the long way, from
    // its head: the walk the file stores starts there.
    bool readsOk = true;
    frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
        if (!readsOk || !input.resolvedAttr) {
            return;
        }
        bool viaChain = false, varying = false;
        UsdAttribute selected;
        RigExecBakedClassifyInput<float>(input.resolvedAttr, S.bakeTime,
                                         *S.chainTargets, &viaChain,
                                         &varying, &selected);
        if (!viaChain) {
            return;
        }
        if (!input.head ||
            input.head.GetPath() != input.resolvedAttr.GetPath()) {
            why = "input " + input.resolvedAttr.GetPath().GetString() +
                  " reads the long way from another attribute than its head";
            readsOk = false;
        }
    });
    if (!readsOk) {
        Fail(why);
        return;
    }

    // Every registered read, in the frozen context's walk over the
    // program's tables, so the list cannot drift from the program. The row
    // and field each sits at in the file's tables come from the
    // enumeration below, in the tables' field order; the two must name the
    // same inputs.
    {
        using Family = RigExecBakeReadFamily;
        struct _Where {
            Family family;
            uint32_t object;
            uint32_t field;
        };
        std::map<const void *, _Where> where;
        const auto place = [&](const void *input, Family family,
                               size_t object, size_t field) {
            where.emplace(input,
                          _Where{family, uint32_t(object), uint32_t(field)});
        };
        for (size_t i = 0; i < B.avarBindings.size(); ++i) {
            place(&B.avarBindings[i].input, Family::AvarBinding, i, 0);
        }
        for (size_t i = 0; i < B.avarConstantBindings.size(); ++i) {
            place(&B.avarConstantBindings[i].input,
                  Family::AvarConstantBinding, i, 0);
        }
        for (size_t i = 0; i < B.ladders.size(); ++i) {
            const RigExecBakedProgramImpl::Ladder &ladder = B.ladders[i];
            place(&ladder.restSpace, Family::Ladder, i, 0);
            place(&ladder.defaultSpace, Family::Ladder, i, 1);
            place(&ladder.posedSpace, Family::Ladder, i, 2);
            for (size_t k = 0; k < 6; ++k) {
                place(&ladder.restAvars[k], Family::Ladder, i, 3 + k);
                place(&ladder.defaultAvars[k], Family::Ladder, i, 9 + k);
            }
            place(&ladder.rotationOrder, Family::Ladder, i, 15);
            place(&ladder.parentSpace, Family::Ladder, i, 16);
            place(&ladder.parentDefaultSpace, Family::Ladder, i, 17);
            place(&ladder.avarDefaultSpace, Family::Ladder, i, 18);
            place(&ladder.posedDefaultSpace, Family::Ladder, i, 19);
            place(&ladder.rotationSign, Family::Ladder, i, 20);
            place(&ladder.interveningSpace, Family::Ladder, i, 21);
        }
        for(size_t i=0;i<B.autoClavicles.size();++i)for(size_t k=0;k<B.autoClavicles[i].scalars.size();++k) {
            const auto &read=B.autoClavicles[i].scalars[k];
            place(read.isFloat?static_cast<const void *>(&read.narrow):static_cast<const void *>(&read.wide),Family::AutoClavicle,i,k);
        }
        for (size_t i = 0; i < B.spaceSwitches.size(); ++i) {
            if(B.spaceSwitches[i].tokenIndex)
                place(&B.spaceSwitches[i].activeTokenInput, Family::SpaceSwitch, i, 0);
            else place(&B.spaceSwitches[i].activeInput, Family::SpaceSwitch, i, 0);
        }
        for (size_t i = 0; i < B.poseInterpolators.size(); ++i) {
            const auto &interp = B.poseInterpolators[i];
            place(&interp.enabled, Family::Interpolator, i, 0);
            for (size_t k = 0; k < interp.valueInputs.size(); ++k) {
                place(&interp.valueInputs[k], Family::Interpolator, i, 1 + k);
            }
        }
        for (size_t i = 0; i < B.solvers.size(); ++i) {
            const RigExecBakedProgramImpl::Solver &s = B.solvers[i];
            const void *fields[] = {
                &s.bend,           &s.upperOffset,    &s.lowerOffset,
                &s.stretch,        &s.softness,       &s.blendWeight,
                &s.preserveVolume, &s.midFollowWeight, &s.roll,
                &s.twist,          &s.minLengthRatio, &s.twistTurns,
                &s.ribbonSampleCount, &s.ikSpace, &s.pin, &s.upperScale, &s.lowerScale, &s.softDistance, &s.limbTwist};
            for (size_t f = 0; f < std::size(fields); ++f) {
                place(fields[f], Family::Solver, i, f);
            }
        }
        for (size_t i = 0; i < B.constraints.size(); ++i) {
            const RigExecBakedProgramImpl::Constraint &c = B.constraints[i];
            const void *fields[] = {
                &c.enabled,   &c.defaultWeight, &c.offset,
                &c.affectX,   &c.affectY,       &c.affectZ,
                &c.tX,        &c.tY,            &c.tZ,
                &c.rX,        &c.rY,            &c.rZ,
                &c.sX,        &c.sY,            &c.sZ,
                &c.aimVector, &c.upVector,      &c.rotationOffset,
                &c.worldUpVector, &c.poleVector, &c.twistDegrees, &c.stretch};
            for (size_t f = 0; f < std::size(fields); ++f) {
                place(fields[f], Family::Constraint, i, f);
            }
        }
        for (size_t i = 0; i < B.weightObjects.size(); ++i) {
            const RigExecBakedProgramImpl::WeightObject &w =
                B.weightObjects[i];
            const void *fields[] = {
                &w.defaultWeight, &w.driver,    &w.scale,     &w.bias,
                &w.strength,      &w.invert,    &w.falloffMin,
                &w.falloffMax,    &w.scaleXPos, &w.scaleYPos, &w.scaleZPos,
                &w.scaleXNeg,     &w.scaleYNeg, &w.scaleZNeg, &w.scaleX,
                &w.scaleY,        &w.scaleZ,    &w.extentU,   &w.extentV};
            for (size_t f = 0; f < std::size(fields); ++f) {
                place(fields[f], Family::WeightObject, i, f);
            }
        }
        size_t visited = 0;
        frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
            if (!readsOk) {
                return;
            }
            ++visited;
            const auto found = where.find(&input);
            if (found == where.end()) {
                why = "a registered input " +
                      (input.head ? input.head.GetPath().GetString()
                                  : std::string("with no head")) +
                      " sits in no table the runtime reads";
                readsOk = false;
                return;
            }
            RigExecBakeRegisteredRead entry;
            entry.family = found->second.family;
            entry.object = found->second.object;
            entry.field = found->second.field;
            readsOk = S.Baked(input, &entry.read, &why);
            if (readsOk) {
                C.registeredReads.push_back(std::move(entry));
            }
        });
        if (readsOk && visited != where.size()) {
            why = "the registered inputs and the runtime's tables disagree (" +
                  std::to_string(visited) + " visited, " +
                  std::to_string(where.size()) + " in the tables)";
            readsOk = false;
        }
        if (!readsOk) {
            Fail(why);
            return;
        }
    }

    // The geometry assembly's own reads: every blend channel's weight (a
    // pose-driven channel reads it only when a value is published at the
    // weight itself) and its samples' activations, a revision's
    // inputs:defaultWeight (RevisionStatic), and the scalar mover inputs the
    // assemblers read through their connections, keyed by the head's path.
    {
        const TfToken defaultWeightName("inputs:defaultWeight");
        const auto blendReads =
            [&](const RigExecBakedProgramImpl::GeomRevision &revision,
                size_t chain, size_t index, bool derived) {
                for (size_t ch = 0; ch < revision.blendChannels.size();
                     ++ch) {
                    const auto &bound = revision.blendChannels[ch];
                    RigExecBakeBlendRead entry;
                    entry.chain = uint32_t(chain);
                    entry.revision = uint32_t(index);
                    entry.derived = derived;
                    entry.channel = uint32_t(ch);
                    if (!S.Resolved(bound.weight, 0.0f, &entry.read, &why)) {
                        return false;
                    }
                    // Each sample's activation, which the gather reads
                    // through the same resolved inputs; an absent one keeps
                    // RigExecBlendSampleData's 1.
                    entry.activations.resize(bound.samples.size());
                    for (size_t s = 0; s < bound.samples.size(); ++s) {
                        if (!S.Resolved(bound.samples[s].activation, 1.0f,
                                        &entry.activations[s], &why)) {
                            return false;
                        }
                    }
                    C.blendWeightReads.push_back(std::move(entry));
                }
                return true;
            };
        // The file keys these reads by the head's path alone, so one
        // attribute read as two types, or as a float one site widens to
        // double and another does not, is refused.
        std::map<SdfPath, std::pair<fb::InputTag, bool>> pathSeen;
        const auto pathRead = [&](const UsdAttribute &a,
                                  const fb::RigExecWireValue &fallback,
                                  bool widen) {
            if (!a) {
                return true;
            }
            const auto seen = pathSeen.emplace(
                a.GetPath(), std::make_pair(fallback.tag, widen));
            if (!seen.second) {
                if (seen.first->second != std::make_pair(fallback.tag, widen)) {
                    why = "the geometry assembly reads " +
                          a.GetPath().GetString() + " as two types";
                    return false;
                }
                return true;
            }
            RigExecBakePathScalarRead entry;
            entry.path = interner->Path(a.GetPath());
            // Every site below reads like moverGraph.cpp's _Read: the
            // resolved walk, then the head's own value, then the fallback.
            entry.headFallback = true;
            if (!S.Resolved(a, fallback, &entry.read, &why)) {
                return false;
            }
            C.pathScalarReads.push_back(std::move(entry));
            return true;
        };
        // The connection-following sites of moverGraph.cpp: _Enabled on
        // every authored mover that assembles parameters, the skin's
        // elementSize and skinningMethod, the iterative movers' scalars
        // (_RecordedInput), and a surface projector's dials and ray
        // settings.
        const auto moverReads =
            [&](const RigExecBakedProgramImpl::GeomRevision &revision) {
                const UsdPrim &prim = revision.moverPrim;
                if (!prim) {
                    return true;
                }
                const RigExecRevisionOp op = revision.op;
                const auto attr = [&](const char *name) {
                    return prim.GetAttribute(TfToken(name));
                };
                // The synthesized derived ops have no authored mover, and a
                // projector's matrix targets never assemble parameters.
                if (op != RigExecRevisionOp::RecomputeNormals &&
                    op != RigExecRevisionOp::RecomputeExtent &&
                    !RigExecIsDerivedMatrixOp(op) &&
                    !pathRead(attr("inputs:enabled"), _Bool(true), false)) {
                    return false;
                }
                if (op == RigExecRevisionOp::Skin &&
                    (!pathRead(attr("rigExec:elementSize"), S.Value(1),
                               false) ||
                     !pathRead(attr("rigExec:skinningMethod"),
                               S.Value(TfToken("classicLinear")), false))) {
                    return false;
                }
                if (op == RigExecRevisionOp::DeltaMush ||
                    op == RigExecRevisionOp::Wrinkle) {
                    RigExecMoverParameters parameters;
                    bool ok = true;
                    frozenDetail::_VisitIterativeMoverScalars(
                        op, parameters,
                        [&](const char *name, auto fallback, auto &) {
                            ok = ok && pathRead(attr(name),
                                                S.Value(fallback), false);
                        });
                    if (!ok) {
                        return false;
                    }
                }
                // The extended deformer settings (format 21) read at the
                // time; the Default-time tokens and the arrays are path
                // reads and leaf sites of their own.
                const auto token = [&](const char *text) {
                    return S.Value(TfToken(text));
                };
                if (op == RigExecRevisionOp::DeltaMush &&
                    (!pathRead(attr("inputs:onlySmooth"), _Bool(false),
                               false) ||
                     !pathRead(attr("inputs:computationToTarget"),
                               _Identity(), false))) {
                    return false;
                }
                if (op == RigExecRevisionOp::Lattice) {
                    const auto vec3f = [](float v) {
                        fb::RigExecWireValue value = _Zero(InputTag::Vec3f);
                        *value.vec3f = RigExecWireVec3f{v, v, v};
                        return value;
                    };
                    if (!pathRead(attr("rigExec:evaluation"), token("legacy"),
                                  false) ||
                        !pathRead(attr("rigExec:interpolationU"),
                                  token("bspline"), false) ||
                        !pathRead(attr("rigExec:interpolationV"),
                                  token("bspline"), false) ||
                        !pathRead(attr("rigExec:interpolationW"),
                                  token("bspline"), false) ||
                        !pathRead(attr("rigExec:origin"), vec3f(-0.5f),
                                  false) ||
                        !pathRead(attr("rigExec:spacing"), vec3f(1.0f),
                                  false) ||
                        !pathRead(attr("rigExec:strength"), _Float(1.0f),
                                  false) ||
                        !pathRead(attr("rigExec:cageMatrix"), _Identity(),
                                  false) ||
                        !pathRead(attr("rigExec:targetMatrix"), _Identity(),
                                  false) ||
                        !pathRead(attr("rigExec:pointSpace"), token("local"),
                                  false)) {
                        return false;
                    }
                }
                if (op == RigExecRevisionOp::SurfaceProject &&
                    (!pathRead(attr("rigExec:snapMode"), token("onSurface"),
                               false) ||
                     !pathRead(attr("rigExec:offset"), _Float(0.0f), false) ||
                     !pathRead(attr("rigExec:surfaceMatrix"), _Identity(),
                               false) ||
                     !pathRead(attr("rigExec:targetMatrix"), _Identity(),
                               false) ||
                     !pathRead(attr("rigExec:pointSpace"), token("local"),
                               false))) {
                    return false;
                }
                if (op == RigExecRevisionOp::ShaderDials) {
                    for (const SdfPath &dial : revision.binding.shaderDials) {
                        const UsdAttribute a = S.stage->GetAttributeAtPath(dial);
                        const bool asFloat =
                            a && a.GetTypeName() == SdfValueTypeNames->Float;
                        if (!pathRead(a,
                                      asFloat ? _Float(0.0f) : _Double(0.0),
                                      asFloat)) {
                            return false;
                        }
                    }
                } else if (RigExecIsDerivedMatrixOp(op)) {
                    if (!pathRead(attr("rigExec:rayOrigin"),
                                  S.Value(GfVec3d(0.0, 0.0, 0.0)), false) ||
                        !pathRead(attr("rigExec:rayDirection"),
                                  S.Value(GfVec3d(0.0, 0.0, 1.0)), false) ||
                        !pathRead(attr("rigExec:rayUp"),
                                  S.Value(GfVec3d(0.0, 1.0, 0.0)), false) ||
                        !pathRead(attr("rigExec:shaderOffset"), _Identity(),
                                  false)) {
                        return false;
                    }
                }
                return true;
            };
        bool ok = true;
        for (size_t c = 0; ok && c < B.chains.size(); ++c) {
            const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
            for (size_t r = 0; ok && r < chain.revisions.size(); ++r) {
                const RigExecBakedProgramImpl::GeomRevision &revision =
                    chain.revisions[r];
                RigExecBakeDefaultWeightRead entry;
                entry.chain = uint32_t(c);
                entry.revision = uint32_t(r);
                ok = blendReads(revision, c, r, false) &&
                     S.Resolved(revision.moverPrim
                                    ? revision.moverPrim.GetAttribute(
                                          defaultWeightName)
                                    : UsdAttribute(),
                                1.0f, &entry.read, &why) &&
                     moverReads(revision);
                if (ok) {
                    C.defaultWeightReads.push_back(std::move(entry));
                }
            }
            for (size_t d = 0; ok && d < chain.derived.size(); ++d) {
                ok = blendReads(chain.derived[d].revision, c, d, true) &&
                     moverReads(chain.derived[d].revision);
            }
        }
        if (!ok) {
            Fail(why);
            return;
        }
    }

    // The facts hold one time's answer, so an object the runtime resolves
    // -- a constraint or property-mover envelope, a current-phase field,
    // and every object either composes -- that reads an animated one holds
    // it at the bake time only. Object identities are allocated before their
    // dependencies. Close the
    // set by identity rather than assuming a discovery order; SCC back-edges
    // terminate because each identity enters the work list once.
    std::vector<char> resolved(C.weightObjects.size(), 0);
    for (int32_t index : C.constraintWeightObjectIndex) {
        if (index >= 0) {
            resolved[size_t(index)] = 1;
        }
    }
    for (const fb::RigExecWirePropertyChain &chain : C.propertyChains) {
        for (const fb::RigExecWirePropertyRevision &revision :
             chain.revisions) {
            if (revision.envelope >= 0) {
                resolved[size_t(revision.envelope)] = 1;
            }
        }
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (revision.weightCurrentPhase && revision.weightObject >= 0 &&
                size_t(revision.weightObject) < resolved.size()) {
                resolved[size_t(revision.weightObject)] = 1;
            }
        }
    }
    std::vector<size_t> pending;
    for(size_t i=0;i<resolved.size();++i) if(resolved[i]) pending.push_back(i);
    for (size_t cursor=0;cursor<pending.size();++cursor) {
        const size_t i=pending[cursor];
        if (!timeVarying[i].IsEmpty()) {
            S.timeVaryingFacts.push_back(
                {objectPaths[i].GetString(), timeVarying[i].GetString()});
        }
        const fb::RigExecWireWeightObject &wire = C.weightObjects[i];
        const auto include=[&](int32_t input) {
            if(input>=0 && !resolved[size_t(input)]) {
                resolved[size_t(input)]=1;
                pending.push_back(size_t(input));
            }
        };
        if (wire.base >= 0) {
            include(wire.base);
        }
        for (int32_t input : wire.inputs) {
            include(input);
        }
    }

    // The array reads, as RigExecBakeListArrayReads chooses them: a slot
    // per attribute they reach, and the site each binds. An entry the
    // oracle resolves names the points it samples unless its read phases
    // fail it first.
    std::vector<RigExecBakeArrayObject> arrayObjects(C.weightObjects.size());
    for (size_t i = 0; i < C.weightObjects.size(); ++i) {
        RigExecBakeArrayObject &object = arrayObjects[i];
        object.path = objectPaths[i];
        object.type = objectTypes[i];
        object.oracle = resolved[i] && oracleFacts[i].phaseError.empty();
        object.samplesInFlight = oracleFacts[i].samplesInFlight;
        if (object.oracle && !object.samplesInFlight &&
            (object.type == TfToken("RigExecSphereWeight") ||
             object.type == TfToken("RigExecPlaneWeight") ||
             object.type == TfToken("RigExecCurveWeight"))) {
            // Preserve full raw relationship cardinality. Prim targets are
            // oracle-only: packets ignore them, as their native gather does.
            SdfPathVector targets;
            const auto prim = B.stage->GetPrimAtPath(object.path);
            if (const auto rel = prim.GetRelationship(TfToken("rigExec:weightTarget")))
                rel.GetTargets(&targets);
            auto &wire = C.weightObjects[i];
            wire.targetPoints.clear();
            wire.targetValid.clear();
            for (const auto &target : targets) {
                if (target.IsPropertyPath()) {
                    const auto attr = B.stage->GetAttributeAtPath(target);
                    wire.targetPoints.push_back(attr ? interner->Path(target) : 0);
                    wire.targetValid.push_back(attr ? 1 : 0);
                } else {
                    wire.targetPoints.push_back(interner->Path(
                        evaluatorDetail::_ResolveGeometryInput(B.stage, target)));
                    wire.targetValid.push_back(0);
                }
            }
        }
    }
    std::vector<RigExecBakeArrayRead> arrayReads;
    if (!RigExecBakeListArrayReads(B, arrayObjects, time, &arrayReads,
                                   &why)) {
        Fail(why);
        return;
    }
    // Per array slot (provisional id), how its default is taken, and
    // whether any read takes it at the time.
    std::map<uint32_t, RigExecBakeArraySlot> arraySlots;
    std::set<uint32_t> arrayLive;
    std::map<std::pair<uint32_t, bool>, size_t> rowAt;
    std::map<std::pair<uint32_t, uint32_t>, size_t> layoutAt;
    C.chainBaseSlots.assign(B.chains.size(), -1);
    std::vector<int32_t> oracleFallbackSlots(C.weightObjects.size(), -1);
    for (const RigExecBakeArrayRead &read : arrayReads) {
        fb::RigExecWireInput input;
        input.tag = read.tag;
        input.walk.reserve(read.hops.size());
        for (const SdfPath &hop : read.hops) {
            uint32_t id = 0;
            if (!S.ArraySlot(hop, read.tag, &id, &why)) {
                Fail(why);
                return;
            }
            input.walk.push_back(id);
            arraySlots[id].tag = read.tag;
            if (!read.rest) {
                arrayLive.insert(id);
            }
        }
        input.mode = input.walk.size() == 1 ? fb::ReadMode::Raw
                                            : fb::ReadMode::Resolved;
        input.constant = S.Intern(_EmptyArray(read.tag));
        const uint32_t head = input.walk.front();
        fb::RigExecWireWeightObject *object =
            read.object < C.weightObjects.size()
                ? &C.weightObjects[read.object]
                : nullptr;
        switch (read.consumer) {
        case RigExecBakeArrayConsumer::Declared:
            // The owning API4 leaf descriptor is captured below. Shared
            // typed slots and time/default metadata were registered above.
            break;
        case RigExecBakeArrayConsumer::Row: {
            const auto key =
                std::make_pair(interner->Path(read.hops.front()), read.rest);
            const auto at = rowAt.emplace(key, C.arrayRows.size());
            if (at.second) {
                RigExecBakeArrayRow row;
                row.path = key.first;
                row.rest = key.second;
                row.gather = read.gather;
                row.read = std::move(input);
                C.arrayRows.push_back(std::move(row));
                break;
            }
            RigExecBakeArrayRow &row = C.arrayRows[at.first->second];
            if (row.read.walk != input.walk) {
                Fail("the geometry assembly reads " +
                     read.hops.front().GetString() + " two ways");
                return;
            }
            row.gather = row.gather || read.gather;
            break;
        }
        case RigExecBakeArrayConsumer::BlendPoints: {
            RigExecBakeBlendPointsRead entry;
            entry.chain = read.chain;
            entry.revision = read.revision;
            entry.derived = read.derived;
            entry.channel = read.channel;
            entry.sample = read.sample;
            entry.read = std::move(input);
            C.blendPoints.push_back(std::move(entry));
            break;
        }
        case RigExecBakeArrayConsumer::Layout: {
            const auto at = layoutAt.emplace(
                std::make_pair(read.chain, read.revision),
                C.layoutSlots.size());
            if (at.second) {
                RigExecBakeLayoutSlots slots;
                slots.chain = read.chain;
                slots.revision = read.revision;
                C.layoutSlots.push_back(slots);
            }
            RigExecBakeLayoutSlots &slots = C.layoutSlots[at.first->second];
            (read.indices ? slots.indices : slots.weights) = head;
            RigExecBakeArraySlot &slot = arraySlots[head];
            slot.layoutChain = int32_t(read.chain);
            slot.layoutRevision = int32_t(read.revision);
            slot.layoutIndices = read.indices;
            break;
        }
        case RigExecBakeArrayConsumer::ChainBase:
            C.chainBaseSlots[read.chain] = int32_t(head);
            arraySlots[head].chain = int32_t(read.chain);
            break;
        case RigExecBakeArrayConsumer::Painted:
            if (object) {
                (read.indices ? object->indicesSlot : object->valuesSlot) =
                    int32_t(head);
            }
            break;
        case RigExecBakeArrayConsumer::OracleSamples:
            if (object) {
                object->oracleSamplesSlot = int32_t(head);
            }
            break;
        case RigExecBakeArrayConsumer::OracleFallback:
            if (object) oracleFallbackSlots[read.object] = int32_t(head);
            break;
        case RigExecBakeArrayConsumer::OracleCurve:
            if (object) {
                object->oracleCurveSlot = int32_t(head);
            }
            break;
        }
    }
    const auto captureDeclaration=[&](const RigExecRevisionLeafKey &key,
                                     fb::RigExecWireExternalDeclaredInput *output)->bool {
            auto &row=*output;
            row.path=S.interner->Path(key.path);
            row.time=fb::ExternalInputTime(key.time);
            row.flavour=fb::ExternalInputFlavour(key.flavour);
            row.fallbackHasValue=!key.fallback.IsEmpty();
            row.read=std::make_unique<fb::RigExecWireInput>();
            row.read->sampleTime=uint8_t(key.time);
            InputTag tag=InputTag::Float;
            using Type=RigExecRevisionLeafType;
            switch(key.type) {
            case Type::Bool:tag=InputTag::Bool;break;
            case Type::Int:tag=InputTag::Int;break;
            case Type::Float:tag=InputTag::Float;break;
            case Type::Double:case Type::Dial:tag=InputTag::Double;break;
            case Type::Token:tag=InputTag::Token;break;
            case Type::Matrix4d:tag=InputTag::Matrix4d;break;
            case Type::Vec3d:tag=InputTag::Vec3d;break;
            case Type::Vec3f:tag=InputTag::Vec3f;break;
            case Type::Vec3i:tag=InputTag::Vec3i;break;
            case Type::IntArray:tag=InputTag::IntArray;break;
            case Type::FloatArray:tag=InputTag::FloatArray;break;
            case Type::DoubleArray:tag=InputTag::DoubleArray;break;
            case Type::Vec2fArray:tag=InputTag::Vec2fArray;break;
            case Type::Vec3fArray:tag=InputTag::Vec3fArray;break;
            default:Fail("declaration has no exact transport type");return false;
            }
            fb::RigExecWireValue fallback=_Zero(tag);
            const auto &v=key.fallback;
            if(v.IsHolding<float>()) fallback=S.Value(v.UncheckedGet<float>());
            else if(v.IsHolding<double>()) fallback=S.Value(v.UncheckedGet<double>());
            else if(v.IsHolding<bool>()) fallback=S.Value(v.UncheckedGet<bool>());
            else if(v.IsHolding<int>()) fallback=S.Value(v.UncheckedGet<int>());
            else if(v.IsHolding<TfToken>()) fallback=S.Value(v.UncheckedGet<TfToken>());
            else if(v.IsHolding<GfMatrix4d>()) fallback=S.Value(v.UncheckedGet<GfMatrix4d>());
            else if(v.IsHolding<GfVec3d>()) fallback=S.Value(v.UncheckedGet<GfVec3d>());
            else if(v.IsHolding<GfVec3i>()) {
                const auto &point=v.UncheckedGet<GfVec3i>();
                fallback.vec3i=std::make_unique<RigExecWireVec3i>(RigExecWireVec3i{point[0],point[1],point[2]});
            }
            else if(v.IsHolding<GfVec3f>()) {
                const auto &point=v.UncheckedGet<GfVec3f>();
                fallback.vec3f=std::make_unique<RigExecWireVec3f>(RigExecWireVec3f{point[0],point[1],point[2]});
            }
            const auto attribute=B.stage->GetAttributeAtPath(key.path);
            // Dial reads follow the source's actual scalar precision, then
            // the projector packet widens Float to Double after selection.
            if(key.type==Type::Dial && attribute &&
               attribute.GetTypeName().GetType()==SdfValueTypeNames->Float.GetType()) {
                tag=InputTag::Float;
                fallback=S.Value(v.IsHolding<double>() ? float(v.UncheckedGet<double>()) :
                    v.IsHolding<float>() ? v.UncheckedGet<float>() : 0.0f);
            }
            // Static structural int3 values remain literals. Only actual
            // current sampled sources require the private Vec3i slot transport.
            if(key.type==Type::Vec3i && key.flavour==RigExecRevisionLeafFlavour::Raw &&
               (!attribute || key.time==RigExecRevisionLeafTime::AtDefault ||
                !attribute.ValueMightBeTimeVarying())) {
                GfVec3i value;
                if(attribute && attribute.Get(&value,key.time==RigExecRevisionLeafTime::AtDefault
                    ? UsdTimeCode::Default() : S.bakeTime)) {
                    fallback.vec3i=std::make_unique<RigExecWireVec3i>(RigExecWireVec3i{value[0],value[1],value[2]});
                    row.fallbackHasValue=true;
                }
                row.read->tag=tag; row.read->mode=fb::ReadMode::Pinned;
                row.read->constant=S.Intern(fallback);
                return true;
            }
            std::string why;
            if(key.flavour==RigExecRevisionLeafFlavour::Raw ||
               key.flavour==RigExecRevisionLeafFlavour::OverlayThenRaw ||
               key.flavour==RigExecRevisionLeafFlavour::Present) {
                row.read->tag=tag; row.read->mode=fb::ReadMode::Raw;
                row.read->constant=S.Intern(fallback);
                if(attribute) {
                    uint32_t slot=0;
                    if(!S.Slot(key.path,"external declaration",&slot,&why)) {Fail(why);return false;}
                    row.read->walk.push_back(slot);
                } else row.read->mode=RigExecFormatIsArrayTag(tag)
                    ? fb::ReadMode::Resolved : fb::ReadMode::Pinned;
            } else if(!S.Resolved(attribute,fallback,row.read.get(),&why)) {Fail(why);return false;}
            row.read->sampleTime=uint8_t(key.time);
            row.allowFloatToDouble=key.type==Type::Dial && row.read->tag==InputTag::Double;
        return true;
    };
    const auto captureLeafSite = [&](const RigExecBakedPathLeaves &leaves, size_t k,
                                     fb::RigExecWireExternalDeclaredInput *row) -> bool {
        if (!captureDeclaration(leaves.decl.keys[k], row)) return false;
        row->exactVersion = k < leaves.exactVersions.size() ? leaves.exactVersions[k] : -1;
        row->exactRecord = k < leaves.exactRecordIndices.size() ? leaves.exactRecordIndices[k] : -1;
        row->exactValueType = k < leaves.exactValueTypes.size() ? leaves.exactValueTypes[k] : -1;
        // Sampling retains only raw connection traversal. Computed producers are
        // consumed through the exact owning route, never through publication lookup.
        for (auto *segment : {&row->read->propertyCandidates, &row->read->doubleCandidates})
            for (auto &hop : *segment) {
                hop.kind = uint8_t(fb::PropertyCandidateKind::SlotOnly);
                hop.version = -1; hop.crossDomain = -1; hop.poseWeight = -1;
            }
        if ((row->exactVersion < 0 || leaves.decl.keys[k].type==RigExecRevisionLeafType::Double) &&
            k < leaves.walks.size() && leaves.walks[k] >= 0) {
            const auto &reader = B.readerWalks[size_t(leaves.walks[k])];
            row->bodyWalk = std::make_unique<fb::RigExecWireInput>(*row->read);
            row->bodyWalk->propertyCandidates.clear(); row->bodyWalk->doubleCandidates.clear();
            std::string why;
            if (!S.Candidates(reader.walk, row->bodyWalk.get(), &why, reader.rawLeaf)) {
                Fail(why); return false;
            }
        }
        return true;
    };
    const auto captureOwner = [&](const RigExecBakedPathLeaves &leaves,
                                  uint32_t chain, uint32_t revision, bool derived, bool layout) {
        auto &owner = C.leafSites[{chain, revision, derived, layout}];
        for (size_t k = 0; k < leaves.decl.keys.size(); ++k) {
            fb::RigExecWireExternalDeclaredInput row;
            if (!captureLeafSite(leaves, k, &row)) return false;
            owner.reads.push_back(std::move(row));
            owner.fallbacks.push_back(leaves.decl.keys[k].fallback);
        }
        return true;
    };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        for (size_t r = 0; r < B.chains[c].revisions.size(); ++r) {
            const auto &rev = B.chains[c].revisions[r];
            if (!captureOwner(rev.leaves, uint32_t(c), uint32_t(r), false, false) ||
                !captureOwner(rev.layoutLeaves, uint32_t(c), uint32_t(r), false, true)) return;
        }
        for (size_t r = 0; r < B.chains[c].derived.size(); ++r) {
            const auto &rev = B.chains[c].derived[r].revision;
            if (!captureOwner(rev.leaves, uint32_t(c), uint32_t(r), true, false) ||
                !captureOwner(rev.layoutLeaves, uint32_t(c), uint32_t(r), true, true)) return;
        }
    }
    for(size_t c=0;c<B.chains.size();++c) for(size_t r=0;r<B.chains[c].revisions.size();++r) {
        const auto &revision=B.chains[c].revisions[r];
        if(revision.binding.externalInputs.empty()) continue;
        auto &captured=C.externalInputs[{uint32_t(c),uint32_t(r)}];
        for(size_t i=0;i<revision.binding.externalInputs.size();++i) {
            const auto &key=revision.binding.externalInputs[i];
            fb::RigExecWireExternalDeclaredInput row;
            const int begin=revision.leaves.decl.externalBegin;
            if(begin<0 || size_t(begin)+i>=revision.leaves.decl.keys.size()) {
                Fail("external declaration has no owning native leaf site"); return;
            }
            if(!captureLeafSite(revision.leaves,size_t(begin)+i,&row)) return;
            captured.reads.push_back(std::move(row));
            captured.fallbacks.push_back(key.fallback);
        }
    }
    C.derivedBaseSlots.assign(B.derivedIndex.size(), -1);
    for(size_t k=0;k<B.derivedIndex.size();++k) {
        const auto &index=B.derivedIndex[k];
        const auto &derived=B.chains[size_t(index.first)].derived[size_t(index.second)];
        if(derived.matrixTarget || !derived.baseQuery.IsValid()) continue;
        const auto attribute=derived.baseQuery.GetAttribute();
        if(!attribute || attribute.GetTypeName().GetType()!=TfType::Find<VtVec3fArray>()) continue;
        const SdfPath path=attribute.GetPath();
        uint32_t slot=0; std::string why;
        if(!S.ArraySlot(path,InputTag::Vec3fArray,&slot,&why)) { Fail(why); return; }
        arraySlots[slot].tag=InputTag::Vec3fArray;
        arrayLive.insert(slot);
        C.derivedBaseSlots[k]=int32_t(slot);
    }
    C.crossDomainRawSlots.assign(B.crossDomainReads.size(), -1);
    for (size_t k=0;k<B.crossDomainReads.size();++k) {
        const int leaf=B.crossDomainReads[k].rawLeaf;
        if (leaf<0) continue;
        const auto &path=B.headLeaves[size_t(leaf)].path;
        uint32_t slot=0; std::string why;
        if (!S.Slot(path,"cross-domain raw fallback",&slot,&why)) {
            Fail(why); return;
        }
        C.crossDomainRawSlots[k]=int32_t(slot);
        if (RigExecFormatIsArrayTag(S.slotTags[slot])) {
            arraySlots[slot].tag = S.slotTags[slot];
            arrayLive.insert(slot);
        }
    }
    C.constraintRawSlots.assign(B.constraintArrays.size(), std::array<int32_t,4>{-1,-1,-1,-1});
    const char *constraintNames[]={"inputs:sourceWeights","inputs:translationOffsets","inputs:rotationOffsets","inputs:poleVectorWeights"};
    for(size_t k=0;k<B.constraintArrays.size();++k) {
        const auto &arrays=B.constraintArrays[k];
        for(size_t channel=0;channel<4;++channel) {
            const auto attribute=arrays.prim.GetAttribute(TfToken(constraintNames[channel]));
            if(!attribute) continue;
            uint32_t slot=0; std::string why;
            if(!S.Slot(attribute.GetPath(),"constraint raw array",&slot,&why)) { Fail(why); return; }
            C.constraintRawSlots[k][channel]=int32_t(slot);
            // Constraint arrays are typed raw AtTime reads. S.Slot only
            // declares their identity; the array census owns their payload.
            if (RigExecFormatIsArrayTag(S.slotTags[slot])) {
                arraySlots[slot].tag = S.slotTags[slot];
                arrayLive.insert(slot);
            }
        }
    }
    C.providerLeafSlots.assign(B.providerProgram.sampled.size(), -1);
    for (size_t k = 0; k < B.providerProgram.sampled.size(); ++k) {
        const auto &leaf = B.providerProgram.sampled[k];
        const auto attribute = B.stage->GetAttributeAtPath(leaf.attribute);
        if (!attribute) continue;
        uint32_t slot = 0;
        std::string why;
        if (!S.Slot(leaf.attribute, "provider raw leaf", &slot, &why)) {
            Fail(why); return;
        }
        C.providerLeafSlots[k] = int32_t(slot);
        if (RigExecFormatIsArrayTag(S.slotTags[slot])) {
            arraySlots[slot].tag = S.slotTags[slot];
            arrayLive.insert(slot);
        }
    }
    // Keep the original typed visitor order, including duplicate conversions:
    // Baked may allocate slots before first-emplace discards a duplicate read.
    using MemoReads = std::map<uint32_t, fb::RigExecWireInput>;
    std::map<uint32_t, std::vector<size_t>> memoOccurrences;
    std::vector<std::function<void(MemoReads &)>> memoConvert;
    frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        if (input.leaf < 0) return;
        const uint32_t leaf = B.leaves.Of<T>().id[size_t(input.leaf)];
        memoOccurrences[leaf].push_back(memoConvert.size());
        memoConvert.emplace_back([&, input, leaf](MemoReads &bindingReads) {
            fb::RigExecWireInput read;
            if (!S.Baked(input, &read, &why)) { readsOk = false; return; }
            bindingReads.emplace(leaf, std::move(read));
        });
    });
    // The original scan stops at the first path for an ID, even when that
    // path is unavailable or an array. Preserve that exact alias policy.
    std::map<uint32_t, SdfPath> firstOverridePath;
    for (const auto &[path, overrideId] : B.headOverrideSlots)
        firstOverridePath.emplace(overrideId, path);
    C.headInputSlots.resize(B.steps.size());
    C.headInputReads.resize(B.steps.size());
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const auto &step = B.steps[index];
        // Raw/binding memo declarations belong to every common op.
        auto &slots = C.headInputSlots[index];
        // Painted storage is private, but its current payload still schedules
        // the packet and diagnostic field that read it. Folded native arrays
        // rebuild the epoch; detached input slots must wake the same bodies.
        const auto paintedSlots = [&](int object) {
            if (object < 0 || size_t(object) >= C.weightObjects.size()) return;
            const auto &weight = C.weightObjects[size_t(object)];
            for (int32_t slot : {weight.valuesSlot, weight.indicesSlot})
                if (slot >= 0) slots.push_back(uint32_t(slot));
        };
        if (step.kind == RigExecBakedStepKind::WeightPacket)
            paintedSlots(step.object);
        else if (step.kind == RigExecBakedStepKind::WeightField)
            for (int object : B.weightFields[size_t(step.object)].objects)
                paintedSlots(object);
        for (const auto leafId : step.leaves) {
            const auto &leaf = B.headLeaves[leafId];
            // Curve keys/tangents are immutable Default literals in the
            // property revision payload; mismatched typed leaves never read.
            if (!leaf.typeMatches ||
                leaf.type == RigExecBakedHeadValueType::Vec2fArray) continue;
            uint32_t slot = 0;
            if (!S.Slot(leaf.path, "head memo", &slot, &why)) {
                Fail(why); return;
            }
            if (RigExecFormatIsArrayTag(S.slotTags[slot])) {
                arraySlots[slot].tag = S.slotTags[slot];
                arrayLive.insert(slot);
            }
            slots.push_back(slot);
        }
        for (const auto id : step.overrideSlots) {
            const auto found = firstOverridePath.find(id);
            if (found == firstOverridePath.end()) continue;
            const auto &path = found->second;
            const auto attr = B.stage->GetAttributeAtPath(path);
            if (!attr || attr.GetTypeName().IsArray()) continue;
            uint32_t slot = 0;
            if (!S.Slot(path, "head override", &slot, &why)) {
                Fail(why); return;
            }
            slots.push_back(slot);
        }
        MemoReads bindingReads;
        std::vector<size_t> selectedOccurrences;
        for (const auto leaf : step.bindingLeaves) {
            const auto found = memoOccurrences.find(leaf);
            if (found != memoOccurrences.end())
                selectedOccurrences.insert(selectedOccurrences.end(),
                                           found->second.begin(), found->second.end());
        }
        std::sort(selectedOccurrences.begin(), selectedOccurrences.end());
        selectedOccurrences.erase(
            std::unique(selectedOccurrences.begin(), selectedOccurrences.end()),
            selectedOccurrences.end());
        for (const auto ordinal : selectedOccurrences)
            memoConvert[ordinal](bindingReads);
        for (const auto leaf : step.bindingLeaves) {
            // Topology heads already capture their exact raw layout slots
            // below. Native path IDs must not add a second read form.
            if (step.kind == RigExecBakedStepKind::SkinTopology) continue;
            if (leaf >= B.leafRefs.size()) {
                const size_t pathLeaf = size_t(leaf) - B.leafRefs.size();
                if (pathLeaf >= B.pathLeafRefs.size()) {
                    Fail("head memo path binding is absent"); return;
                }
                const auto &ref = B.pathLeafRefs[pathLeaf];
                const auto *leaves=RigExecBakedPathLeavesOf(B,ref);
                if (!leaves || ref.key>=leaves->decl.keys.size()) {
                    Fail("memo path binding has no actual owner"); return;
                }
                const auto &key=leaves->decl.keys[ref.key];
                fb::RigExecWireExternalDeclaredInput sample;
                if (!captureLeafSite(*leaves,ref.key,&sample)) return;
                // Raw oracle arrays/tokens retain direct slot sampling. Ordinary
                // leaves retain their complete raw connection traversal instead.
                if (ref.owner==RigExecBakedPathLeafOwner::WeightOracle) {
                    for (uint32_t slot:sample.read->walk) slots.push_back(slot);
                } else {
                    // The sampling descriptor strips computed producers. A memo
                    // read belongs to the body and retains its actual bound walk.
                    auto memo=std::move(*sample.read);
                    if (ref.key<leaves->walks.size() && leaves->walks[ref.key]>=0) {
                        const auto &reader=B.readerWalks[size_t(leaves->walks[ref.key])];
                        memo.propertyCandidates.clear();memo.doubleCandidates.clear();
                        if (!S.Candidates(reader.walk,&memo,&why,reader.rawLeaf)) {
                            Fail(why); return;
                        }
                    }
                    C.headInputReads[index].push_back(std::move(memo));
                }
                continue;
            }
            const auto found = bindingReads.find(leaf);
            if (found == bindingReads.end()) { Fail("head memo binding is absent"); return; }
            C.headInputReads[index].push_back(found->second);
        }
        if (!readsOk) { Fail(why); return; }
        if (step.kind == RigExecBakedStepKind::SkinTopology) {
            const auto *revision = RigExecBakedLayoutRevision(B, size_t(step.object));
            if (revision) {
                for (const auto &key : revision->layoutLeaves.decl.keys) {
                    const auto found = S.slotIds.find(key.path);
                    if (found != S.slotIds.end()) slots.push_back(found->second);
                }
            }
        }
        std::sort(slots.begin(), slots.end());
        slots.erase(std::unique(slots.begin(), slots.end()), slots.end());
    }
    {
        // The painted arrays are stored once, in their inputs, whose
        // defaults (read at Default) are what Build folded; the oracle's
        // points are its inputs' values at the time, as the facts hold them.
        const auto painted = [&](int32_t slot, const auto &folded) {
            using Element =
                typename std::decay_t<decltype(folded)>::value_type;
            VtArray<Element> held;
            if (slot >= 0) {
                S.slots[size_t(slot)].Get(&held, UsdTimeCode::Default());
            }
            return held.size() == folded.size() &&
                   (held.empty() ||
                    std::memcmp(held.cdata(), folded.data(),
                                sizeof(Element) * folded.size()) == 0);
        };
        const auto points = [&](int32_t slot,
                                const std::vector<GfVec3f> &want) {
            VtVec3fArray held;
            return slot >= 0 &&
                   S.slots[size_t(slot)].Get(&held, S.bakeTime) &&
                   held.size() == want.size() &&
                   (want.empty() ||
                    std::memcmp(held.cdata(), want.data(),
                                sizeof(GfVec3f) * want.size()) == 0);
        };
        const std::string support = "dynamic/base sparse support mismatch on ";
        for (size_t i = 0; i < C.weightObjects.size(); ++i) {
            fb::RigExecWireWeightObject &wire = C.weightObjects[i];
            const bool stepBacked = i < B.weightObjects.size();
            const std::vector<float> &values =
                stepBacked ? B.weightObjects[i].values
                           : envelopes[i - B.weightObjects.size()].values;
            const std::vector<int> &indices =
                stepBacked ? B.weightObjects[i].indices
                           : envelopes[i - B.weightObjects.size()].indices;
            const std::string path = objectPaths[i].GetString();
            if (!painted(wire.valuesSlot, values) ||
                !painted(wire.indicesSlot, indices)) {
                Fail("the painted weights of " + path +
                     " are not its arrays at Default");
                return;
            }
            const RigExecWeightOracleFacts &facts = oracleFacts[i];
            const bool samples = arrayObjects[i].oracle &&
                                 !facts.samplesInFlight && facts.haveSamples;
            const auto present = [&](int32_t slot) {
                VtVec3fArray held;
                return slot >= 0 && S.slots[size_t(slot)].Get(&held, S.bakeTime);
            };
            int32_t sampleSlot = wire.oracleSamplesSlot;
            if (!present(sampleSlot)) sampleSlot = oracleFallbackSlots[i];
            if (samples != present(sampleSlot) ||
                (samples && !points(sampleSlot, facts.samples))) {
                Fail("the oracle's sample points of " + path +
                     " are not its input's");
                return;
            }
            const bool curve = arrayObjects[i].oracle && facts.haveCurve;
            VtVec3fArray curveValue;
            const bool haveCurve = wire.oracleCurveSlot >= 0 &&
                S.slots[size_t(wire.oracleCurveSlot)].Get(&curveValue, S.bakeTime) &&
                !curveValue.empty();
            if (curve != haveCurve ||
                (curve && !points(wire.oracleCurveSlot, facts.curve))) {
                Fail("the oracle's curve points of " + path +
                     " are not its input's");
                return;
            }
            // A dynamic weight's sparse support, checked against its
            // base's on every run over their indices inputs: the facts'
            // error stands only where those inputs at their defaults would
            // not report it.
            if (wire.oracleStaticError == support + path &&
                wire.indicesSlot >= 0 && wire.base >= 0 &&
                size_t(wire.base) < C.weightObjects.size()) {
                const int32_t baseSlot =
                    C.weightObjects[size_t(wire.base)].indicesSlot;
                VtIntArray mine, theirs;
                S.slots[size_t(wire.indicesSlot)].Get(
                    &mine, UsdTimeCode::Default());
                if (baseSlot >= 0) {
                    S.slots[size_t(baseSlot)].Get(&theirs,
                                                  UsdTimeCode::Default());
                }
                if (!mine.empty() &&
                    std::set<int>(mine.begin(), mine.end()) !=
                        std::set<int>(theirs.begin(), theirs.end())) {
                    wire.oracleStaticError.clear();
                }
            }
        }
    }

    // Public arrays follow the evaluator's admission set. Other array
    // slots retain static consumer defaults without exposing input APIs.
    std::set<SdfPath> publicArrays;
    for (const RigExecUpstreamArrayRow &row :
         RigExecBakedUpstreamAdmissibleArrays(evaluator)) {
        InputTag tag;
        if (row.type == TfType::Find<VtIntArray>()) {
            tag = InputTag::IntArray;
        } else if (row.type == TfType::Find<VtFloatArray>()) {
            tag = InputTag::FloatArray;
        } else if (row.type == TfType::Find<VtDoubleArray>()) {
            tag = InputTag::DoubleArray;
        } else if (row.type == TfType::Find<VtVec2fArray>()) {
            tag = InputTag::Vec2fArray;
        } else if (row.type == TfType::Find<VtVec3fArray>()) {
            tag = InputTag::Vec3fArray;
        } else {
            Fail("no array input tag holds " + row.path.GetString());
            return;
        }
        uint32_t id = 0;
        if (!S.ArraySlot(row.path, tag, &id, &why)) {
            Fail(why);
            return;
        }
        publicArrays.insert(row.path);
        arraySlots[id].tag = tag;
        if (row.time == RigExecUpstreamArrayRow::Time::AtDefault) {
            arrayLive.erase(id);
        } else {
            arrayLive.insert(id);
        }
    }
    const auto &publicScalars = baked->GetUpstreamAdmissible();
    // The reads an export build's Range skins and group gates rest on are
    // the file's constants: private slots holding their bake-time value,
    // which no integration can set. Empty for a Live program.
    const std::set<SdfPath> &pinned = baked->GetExportPinnedPaths();
    const auto listedSlot = [&](uint32_t id) {
        const UsdAttribute &attribute = S.slots[id];
        if (pinned.count(attribute.GetPath()) != 0)
            return false;
        if (RigExecFormatIsArrayTag(S.slotTags[id]))
            return publicArrays.count(attribute.GetPath()) != 0;
        const auto found = publicScalars.find(attribute.GetPath());
        return found != publicScalars.end() &&
               found->second == attribute.GetTypeName().GetType();
    };
    uint32_t listedCount = 0;
    for (size_t id = 0; id < S.slots.size(); ++id)
        listedCount += listedSlot(uint32_t(id)) ? 1 : 0;
    // The public prefix and private suffix are each ordered by path text.

    std::vector<std::string> names;
    names.reserve(S.slots.size());
    for (const UsdAttribute &a : S.slots) {
        names.push_back(a.GetPath().GetString());
    }
    std::vector<uint32_t> order(S.slots.size());
    std::iota(order.begin(), order.end(), 0u);
    std::stable_sort(order.begin(), order.end(),
                     [&](uint32_t a, uint32_t b) {
                         const bool x = listedSlot(a), y = listedSlot(b);
                         return x != y ? x > y : names[a] < names[b];
                     });
    std::vector<uint32_t> remap(S.slots.size());
    std::vector<UsdAttribute> slots(S.slots.size());
    std::vector<InputTag> tags(S.slots.size());
    for (size_t at = 0; at < order.size(); ++at) {
        remap[order[at]] = uint32_t(at);
        slots[at] = S.slots[order[at]];
        tags[at] = S.slotTags[order[at]];
    }
    S.slots = std::move(slots);
    S.slotTags = std::move(tags);
    for (auto &entry : S.slotIds) {
        entry.second = remap[entry.second];
    }
    const auto remapWalk = [&](fb::RigExecWireInput &input) {
        for (uint32_t &slot : input.walk) slot = remap[slot];
        for (auto &candidate : input.propertyCandidates)
            candidate.slot = remap[candidate.slot];
        for (auto &candidate : input.doubleCandidates)
            candidate.slot = remap[candidate.slot];
        if (input.rawFallbackSlot >= 0)
            input.rawFallbackSlot = int32_t(remap[size_t(input.rawFallbackSlot)]);
    };
    for (auto &slots : C.headInputSlots) {
        for (auto &slot : slots) slot = remap[slot];
        std::sort(slots.begin(), slots.end());
    }
    for (auto &field : C.weightFields)
        for (auto &read : field.scalarReads) remapWalk(read);
    for (auto &reads : C.headInputReads)
        for (auto &read : reads) remapWalk(read);
    for (auto &entry : C.externalInputs)
        for (auto &row : entry.second.reads) {
            remapWalk(*row.read); if (row.bodyWalk) remapWalk(*row.bodyWalk);
        }
    for (auto &entry : C.leafSites)
        for (auto &row : entry.second.reads) {
            remapWalk(*row.read); if (row.bodyWalk) remapWalk(*row.bodyWalk);
        }
    for (auto &slot : C.derivedBaseSlots)
        if (slot >= 0) slot = int32_t(remap[size_t(slot)]);
    for (auto &slot : C.crossDomainRawSlots)
        if (slot >= 0) slot = int32_t(remap[size_t(slot)]);
    for (auto &row : C.constraintRawSlots)
        for(auto &slot:row) if(slot>=0) slot=int32_t(remap[size_t(slot)]);
    for (auto &slot : C.providerLeafSlots)
        if (slot >= 0) slot = int32_t(remap[size_t(slot)]);
    for (fb::RigExecWireWeightObject &object : C.weightObjects) {
        _ForEachReadSlot(object,
                         [&](std::unique_ptr<fb::RigExecWireInput> &input) {
                             remapWalk(*input);
                         });
    }
    for (fb::RigExecWirePropertyChain &chain : C.propertyChains) {
        chain.target = remap[chain.target];
        for (fb::RigExecWirePropertyRevision &revision : chain.revisions) {
            _ForEachRevisionRead(revision, remapWalk);
        }
    }
    for (fb::RigExecWirePhasedConsumer &phased : C.phasedConsumers) {
        phased.consumer = remap[phased.consumer];
        for (uint32_t &hop : phased.hops) {
            hop = remap[hop];
        }
    }
    for (RigExecBakeRegisteredRead &entry : C.registeredReads) {
        remapWalk(entry.read);
    }
    for (RigExecBakeBlendRead &entry : C.blendWeightReads) {
        remapWalk(entry.read);
        for (fb::RigExecWireInput &activation : entry.activations) {
            remapWalk(activation);
        }
    }
    for (RigExecBakeDefaultWeightRead &entry : C.defaultWeightReads) {
        remapWalk(entry.read);
    }
    for (RigExecBakePathScalarRead &entry : C.pathScalarReads) {
        remapWalk(entry.read);
    }
    for (RigExecBakeArrayRow &row : C.arrayRows) {
        remapWalk(row.read);
    }
    for (RigExecBakeBlendPointsRead &entry : C.blendPoints) {
        remapWalk(entry.read);
    }
    for (RigExecBakeLayoutSlots &slots : C.layoutSlots) {
        slots.indices = remap[slots.indices];
        slots.weights = remap[slots.weights];
    }
    const auto remapField = [&](int32_t *slot) {
        if (*slot >= 0) {
            *slot = int32_t(remap[size_t(*slot)]);
        }
    };
    for (int32_t &slot : C.chainBaseSlots) {
        remapField(&slot);
    }
    for (fb::RigExecWireWeightObject &object : C.weightObjects) {
        remapField(&object.valuesSlot);
        remapField(&object.indicesSlot);
        remapField(&object.oracleSamplesSlot);
        remapField(&object.oracleCurveSlot);
        remapField(&object.oraclePlaneAxisSlot);
        remapField(&object.oraclePlaneBoundsSlot);
    }
    for (const auto &[id, slot] : arraySlots) {
        RigExecBakeArraySlot entry = slot;
        entry.slot = remap[id];
        entry.atDefault = arrayLive.count(id) == 0;
        C.arraySlots.push_back(entry);
    }
    std::sort(C.arraySlots.begin(), C.arraySlots.end(),
              [](const RigExecBakeArraySlot &a,
                 const RigExecBakeArraySlot &b) { return a.slot < b.slot; });
    // A slot's fields until its chain and phased consumer are known.
    struct _SlotRow {
        uint32_t name = 0;
        uint32_t value = 0;
        int32_t chain = -1;
        int32_t phased = -1;
        InputTag type = InputTag::Double;
        uint8_t flags = 0;
    };
    std::vector<_SlotRow> rows;
    rows.reserve(S.slots.size());
    for (size_t s = 0; s < S.slots.size(); ++s) {
        const UsdAttribute &a = S.slots[s];
        _SlotRow slot;
        slot.name = interner->Path(a.GetPath());
        slot.type = S.slotTags[s];
        bool has = false;
        slot.value = S.ReadSlot(a, slot.type, S.bakeTime, &has);
        slot.flags = s < listedCount ? _Bit(fb::InputSlotFlags::Listed) : 0;
        if (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0) {
            slot.flags |= _Bit(fb::InputSlotFlags::Animated);
        }
        if (has) {
            slot.flags |= _Bit(fb::InputSlotFlags::HasValue);
        }
        if(a.GetResolveInfo(S.bakeTime).ValueIsBlocked())
            slot.flags |= _Bit(fb::InputSlotFlags::SourceBlocked);
        rows.push_back(slot);
    }
    C.listedInputs = listedCount;
    S.listedNames.reserve(order.size());
    for (size_t s = 0; s < listedCount; ++s) {
        S.listedNames.push_back(names[order[s]]);
    }
    // Each slot names the chain it is the target of and the phased consumer
    // publishing at it; one of each at most.
    for (size_t c = 0; c < C.propertyChains.size(); ++c) {
        _SlotRow &slot = rows[C.propertyChains[c].target];
        if (slot.chain >= 0) {
            Fail("two property chains share the target " +
                 names[order[C.propertyChains[c].target]]);
            return;
        }
        slot.chain = int32_t(c);
    }
    for (size_t k = 0; k < C.phasedConsumers.size(); ++k) {
        _SlotRow &slot = rows[C.phasedConsumers[k].consumer];
        if (slot.phased >= 0) {
            Fail("two phased connections publish at " +
                 names[order[C.phasedConsumers[k].consumer]]);
            return;
        }
        slot.phased = int32_t(k);
    }
    C.inputs.reserve(rows.size());
    for (const _SlotRow &slot : rows) {
        C.inputs.emplace_back(slot.name, slot.value, slot.chain, slot.phased,
                              slot.type, slot.flags);
    }
    _valid = true;
}

RigExecBakeComputedCapture::~RigExecBakeComputedCapture() = default;

const RigExecBakeInputs &
RigExecBakeComputedCapture::GetInputs() const
{
    return _state->computed;
}

const std::vector<RigExecBakeTimeVaryingFact> &
RigExecBakeComputedCapture::GetTimeVaryingFacts() const
{
    return _state->timeVaryingFacts;
}

const std::vector<std::string> &
RigExecBakeComputedCapture::GetListedInputNames() const
{
    return _state->listedNames;
}

}  // namespace rigExec
