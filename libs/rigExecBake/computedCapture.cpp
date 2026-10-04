// .rigexec Computed section producer.
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/capture.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frozenContextInternal.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <map>
#include <numeric>
#include <set>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

using v4::InputTag;

uint8_t
_Bit(v4::InputReadFlags flag)
{
    return uint8_t(flag);
}

uint8_t
_Bit(v4::InputSlotFlags flag)
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
    } else if (type == TfType::Find<GfVec3f>()) {
        *tag = InputTag::Vec3f;
    } else {
        return false;
    }
    return true;
}

v4::RigExecWireValue
_Zero(InputTag tag)
{
    v4::RigExecWireValue value;
    value.tag = tag;
    return value;
}

v4::RigExecWireValue
_Double(double d)
{
    v4::RigExecWireValue value = _Zero(InputTag::Double);
    std::memcpy(&value.bits, &d, sizeof(d));
    return value;
}

v4::RigExecWireValue
_Float(float f)
{
    v4::RigExecWireValue value = _Zero(InputTag::Float);
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(f));
    value.bits = bits;
    return value;
}

v4::RigExecWireValue
_Bool(bool b)
{
    v4::RigExecWireValue value = _Zero(InputTag::Bool);
    value.bits = b ? 1 : 0;
    return value;
}

v4::RigExecWireValue
_Identity()
{
    v4::RigExecWireValue value = _Zero(InputTag::Matrix4d);
    for (size_t i = 0; i < 4; ++i) {
        value.matrix[i * 4 + i] = 1.0;
    }
    return value;
}

v4::PropertyValueType
_PropertyType(RigExecBakedPropertyChainDesc::ValueType type)
{
    using T = RigExecBakedPropertyChainDesc::ValueType;
    switch (type) {
    case T::Double:
        return v4::PropertyValueType::Double;
    case T::Matrix4d:
        return v4::PropertyValueType::Matrix4d;
    case T::Vec3f:
        return v4::PropertyValueType::Vec3f;
    default:
        return v4::PropertyValueType::Float;
    }
}

v4::PropertyOp
_PropertyOp(bool valid, RigExecPropertyOp op)
{
    if (!valid) {
        return v4::PropertyOp::Invalid;
    }
    switch (op) {
    case RigExecPropertyOp::Add:
        return v4::PropertyOp::Add;
    case RigExecPropertyOp::Multiply:
        return v4::PropertyOp::Multiply;
    case RigExecPropertyOp::Clamp:
        return v4::PropertyOp::Clamp;
    case RigExecPropertyOp::Remap:
        return v4::PropertyOp::Remap;
    case RigExecPropertyOp::Blend:
        return v4::PropertyOp::Blend;
    case RigExecPropertyOp::Curve:
        return v4::PropertyOp::Curve;
    }
    return v4::PropertyOp::Invalid;
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
_ForEachRevisionRead(v4::PropertyRevision &revision, const Visit &visit)
{
    for (v4::RigExecWireInput *input :
         {&revision.enabled, &revision.defaultWeight, &revision.value,
          &revision.min, &revision.max}) {
        visit(*input);
    }
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
              RigExecBinaryWriter *writer, std::vector<uint32_t> *paths,
              std::vector<uint8_t> *valid)
{
    paths->reserve(attrs.size());
    valid->reserve(attrs.size());
    for (const UsdAttribute &attr : attrs) {
        paths->push_back(attr ? writer->AddString(attr.GetPath().GetString())
                              : 0);
        valid->push_back(attr ? uint8_t(1) : uint8_t(0));
    }
}

/// Every read of a v4 weight object, so a walk rewrite cannot miss one.
template <class Visit>
void
_ForEachRead(v4::RigExecWireWeightObject &object, const Visit &visit)
{
    for (v4::RigExecWireInput *input :
         {&object.defaultWeight, &object.driver, &object.scale, &object.bias,
          &object.strength, &object.invert, &object.falloffMin,
          &object.falloffMax, &object.scaleXPos, &object.scaleYPos,
          &object.scaleZPos, &object.scaleXNeg, &object.scaleYNeg,
          &object.scaleZNeg, &object.scaleX, &object.scaleY, &object.scaleZ,
          &object.extentU, &object.extentV}) {
        visit(*input);
    }
}

}  // namespace

struct RigExecBakeComputedCapture::_State {
    RigExecBinaryWriter *writer = nullptr;
    UsdStageRefPtr stage;
    const std::set<SdfPath> *chainTargets = nullptr;
    UsdTimeCode bakeTime;
    RigExecWireComputed computed;
    std::map<std::string, uint32_t> valueIds;
    std::map<std::string, int32_t> arrayIds;
    /// Slots in first-reference order until Finish sorts them.
    std::map<SdfPath, uint32_t> slotIds;
    std::vector<UsdAttribute> slots;
    std::vector<InputTag> slotTags;
    /// Paths each override number was registered under (the inverse of
    /// RigExecBakedProgramImpl::overridableInputs).
    std::vector<std::set<SdfPath>> registered;

    uint32_t Intern(const v4::RigExecWireValue &value)
    {
        std::string key(1, char(value.tag));
        key.append(reinterpret_cast<const char *>(&value.bits),
                   sizeof(value.bits));
        switch (value.tag) {
        case InputTag::Matrix4d:
            key.append(reinterpret_cast<const char *>(value.matrix.data()),
                       sizeof(double) * value.matrix.size());
            break;
        case InputTag::Vec3d:
            key.append(reinterpret_cast<const char *>(value.vec3d.data()),
                       sizeof(double) * value.vec3d.size());
            break;
        case InputTag::Vec3f:
            key.append(reinterpret_cast<const char *>(value.vec3f.data()),
                       sizeof(float) * value.vec3f.size());
            break;
        default:
            break;
        }
        const auto found =
            valueIds.emplace(key, uint32_t(computed.values.size()));
        if (found.second) {
            computed.values.push_back(value);
        }
        return found.first->second;
    }

    int32_t InternPoints(const std::vector<GfVec3f> &points)
    {
        std::string key(reinterpret_cast<const char *>(points.data()),
                        sizeof(GfVec3f) * points.size());
        const auto found =
            arrayIds.emplace(key, int32_t(computed.vec3fArrays.size()));
        if (found.second) {
            v4::RigExecWireVec3fArray array;
            array.v = _ToVec3fs(points);
            computed.vec3fArrays.push_back(std::move(array));
        }
        return found.first->second;
    }

    /// The raw typed read of a slot attribute at \p time: no walk, no
    /// overlay. A failed read holds the type's zero and says so.
    uint32_t ReadSlot(const UsdAttribute &a, InputTag tag, UsdTimeCode time,
                      bool *has)
    {
        v4::RigExecWireValue value = _Zero(tag);
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
                value.bits = writer->AddString(v.GetString());
            }
            break;
        }
        case InputTag::Matrix4d: {
            GfMatrix4d v(1.0);
            if ((*has = a.Get(&v, time))) {
                for (int r = 0; r < 4; ++r) {
                    for (int c = 0; c < 4; ++c) {
                        value.matrix[size_t(r * 4 + c)] = v[r][c];
                    }
                }
            }
            break;
        }
        case InputTag::Vec3d: {
            GfVec3d v(0.0);
            if ((*has = a.Get(&v, time))) {
                value.vec3d = RigExecWireVec3d{v[0], v[1], v[2]};
            }
            break;
        }
        case InputTag::Vec3f: {
            GfVec3f v(0.0f);
            if ((*has = a.Get(&v, time))) {
                value.vec3f = RigExecWireVec3f{v[0], v[1], v[2]};
            }
            break;
        }
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

    /// The walk GetAttribute takes from \p head, as slot ids (provisional
    /// until the slots are sorted), with what the classifier saw on it.
    bool Walk(const UsdAttribute &head, v4::RigExecWireInput *out,
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
            out->flags |= _Bit(v4::InputReadFlags::ViaChain);
        }
        if (varying && out->mode != v4::ReadMode::Baked) {
            out->flags |= _Bit(v4::InputReadFlags::Varying);
        }
        return true;
    }

    /// A registered input's constant, tagged with the input's type.
    v4::RigExecWireValue Value(float v) { return _Float(v); }
    v4::RigExecWireValue Value(double v) { return _Double(v); }
    v4::RigExecWireValue Value(bool v) { return _Bool(v); }
    v4::RigExecWireValue Value(int v)
    {
        v4::RigExecWireValue value = _Zero(InputTag::Int);
        value.bits = uint32_t(v);
        return value;
    }
    v4::RigExecWireValue Value(const TfToken &v)
    {
        v4::RigExecWireValue value = _Zero(InputTag::Token);
        value.bits = writer->AddString(v.GetString());
        return value;
    }
    v4::RigExecWireValue Value(const GfMatrix4d &v)
    {
        v4::RigExecWireValue value = _Zero(InputTag::Matrix4d);
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                value.matrix[size_t(r * 4 + c)] = v[r][c];
            }
        }
        return value;
    }
    v4::RigExecWireValue Value(const GfVec3d &v)
    {
        v4::RigExecWireValue value = _Zero(InputTag::Vec3d);
        value.vec3d = RigExecWireVec3d{v[0], v[1], v[2]};
        return value;
    }

    /// A registered program read, as RigExecBakedRead resolves it.
    template <class T>
    bool Baked(const RigExecBakedInput<T> &input, v4::RigExecWireInput *out,
               std::string *error)
    {
        *out = v4::RigExecWireInput();
        const v4::RigExecWireValue constant = Value(input.constant);
        out->tag = constant.tag;
        out->mode = v4::ReadMode::Baked;
        out->overrideIndex = int32_t(input.overrideIndex);
        out->constant = Intern(constant);
        if (input.varying) {
            out->flags |= _Bit(v4::InputReadFlags::Varying);
        }
        if (input.resolvedAttr) {
            out->flags |= _Bit(v4::InputReadFlags::LongWay);
        }
        if (!input.head) {
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
        return true;
    }

    /// RigExecResolvedInputs::GetAttribute from \p head, typed and falling
    /// back as \p fallback: the oracle's _ResolvedRead, and every assembly
    /// read that resolves through the generation's inputs.
    bool Resolved(const UsdAttribute &head,
                  const v4::RigExecWireValue &fallback,
                  v4::RigExecWireInput *out, std::string *error)
    {
        *out = v4::RigExecWireInput();
        out->tag = fallback.tag;
        out->mode = v4::ReadMode::Resolved;
        out->constant = Intern(fallback);
        if (!head) {
            return true;
        }
        SdfPathVector paths;
        return Walk(head, out, &paths, error);
    }

    /// An envelope read, as the oracle's _ResolvedRead resolves it.
    bool Resolved(const UsdAttribute &head, float fallback,
                  v4::RigExecWireInput *out, std::string *error)
    {
        return Resolved(head, _Float(fallback), out, error);
    }

    /// A property-mover input, as _PinnedRead resolves it: a connected
    /// attribute through GetAttribute's walk (Resolved), an unconnected one
    /// from its own path (Pinned over the attribute alone), a missing one
    /// as \p fallback (an empty Pinned walk).
    bool Property(const UsdAttribute &head, const v4::RigExecWireValue &fallback,
                  v4::RigExecWireInput *out, std::string *error)
    {
        *out = v4::RigExecWireInput();
        out->tag = fallback.tag;
        out->mode = v4::ReadMode::Pinned;
        out->constant = Intern(fallback);
        if (!head) {
            return true;
        }
        if (head.HasAuthoredConnections()) {
            out->mode = v4::ReadMode::Resolved;
        }
        SdfPathVector paths;
        if (!Walk(head, out, &paths, error)) {
            return false;
        }
        if (out->mode == v4::ReadMode::Pinned && out->walk.size() != 1) {
            *error = "input " + head.GetPath().GetString() +
                     " has no connection but its walk leaves it";
            return false;
        }
        return true;
    }
};

RigExecBakeComputedCapture::RigExecBakeComputedCapture(
    RigExecRigEvaluator &evaluator, const RigExecBakeCapture &records,
    RigExecBinaryWriter *writer, std::string *error)
    : _state(std::make_unique<_State>())
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
    };
    const RigExecBakedProgram *baked = evaluator.GetBakedProgram();
    if (!baked || !writer) {
        Fail("no baked program standing to capture from");
        return;
    }
    const RigExecBakedProgramImpl &B = baked->GetStepGraph();
    _State &S = *_state;
    S.writer = writer;
    S.stage = B.stage;
    S.chainTargets = &B.chainTargets;
    S.bakeTime = RigExecBakedProbeTime(B.stage);
    RigExecWireComputed &C = S.computed;
    C.bakeTime = S.bakeTime.GetValue();
    S.Intern(_Double(0.0));
    C.vec3fArrays.emplace_back();
    S.arrayIds.emplace(std::string(), 0);
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

    // The step-backed objects, in the geometry section's order, read as
    // the program reads them.
    const auto token = [&](const TfToken &t) {
        return writer->AddString(t.GetString());
    };
    C.weightObjects.reserve(B.weightObjects.size() + envelopes.size());
    for (const RigExecBakedProgramImpl::WeightObject &object :
         B.weightObjects) {
        v4::RigExecWireWeightObject wire;
        wire.path = writer->AddString(object.path.GetString());
        wire.type = token(object.type);
        wire.representation = token(object.representation);
        wire.rangePolicy = token(object.rangePolicy);
        wire.values = object.values;
        wire.indices.assign(object.indices.begin(), object.indices.end());
        wire.base = int32_t(object.base);
        wire.inputs.assign(object.inputs.begin(), object.inputs.end());
        wire.combineMode = token(object.combineMode);
        _ToAttributes(object.combineTargetPoints, writer,
                      &wire.combineTargetPoints, &wire.combineTargetValid);
        wire.costElements = uint64_t(object.costElements);
        wire.providerSlot = int32_t(object.providerSlot);
        wire.planeAxis = token(object.planeAxis);
        wire.planeBounds = token(object.planeBounds);
        _ToAttributes(object.targetPoints, writer, &wire.targetPoints,
                      &wire.targetValid);
        _ToAttributes(object.samplePoints, writer, &wire.samplePoints,
                      &wire.sampleValid);
        _ToAttributes(object.curvePoints, writer, &wire.curvePoints,
                      &wire.curveValid);
        wire.falloffCurve = object.falloffCurve;
        const std::pair<const RigExecBakedInput<float> *,
                        v4::RigExecWireInput *>
            reads[] = {
                {&object.defaultWeight, &wire.defaultWeight},
                {&object.driver, &wire.driver},
                {&object.scale, &wire.scale},
                {&object.bias, &wire.bias},
                {&object.strength, &wire.strength},
                {&object.invert, &wire.invert},
                {&object.falloffMin, &wire.falloffMin},
                {&object.falloffMax, &wire.falloffMax},
                {&object.scaleXPos, &wire.scaleXPos},
                {&object.scaleYPos, &wire.scaleYPos},
                {&object.scaleZPos, &wire.scaleZPos},
                {&object.scaleXNeg, &wire.scaleXNeg},
                {&object.scaleYNeg, &wire.scaleYNeg},
                {&object.scaleZNeg, &wire.scaleZNeg},
                {&object.scaleX, &wire.scaleX},
                {&object.scaleY, &wire.scaleY},
                {&object.scaleZ, &wire.scaleZ},
                {&object.extentU, &wire.extentU},
                {&object.extentV, &wire.extentV},
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
        v4::RigExecWireWeightObject wire;
        wire.envelopeOnly = true;
        wire.path = writer->AddString(object.path.GetString());
        wire.type = token(object.type);
        wire.representation = token(object.representation);
        wire.rangePolicy = token(object.rangePolicy);
        wire.values = object.values;
        wire.indices.assign(object.indices.begin(), object.indices.end());
        wire.base = int32_t(object.base);
        wire.inputs.assign(object.inputs.begin(), object.inputs.end());
        wire.combineMode = token(object.combineMode);
        wire.costElements = uint64_t(std::max<size_t>(object.values.size(), 1));
        const std::pair<const RigExecBakedEnvelopeObject::Read *,
                        v4::RigExecWireInput *>
            reads[] = {
                {&object.defaultWeight, &wire.defaultWeight},
                {&object.driver, &wire.driver},
                {&object.scale, &wire.scale},
                {&object.bias, &wire.bias},
                {&object.strength, &wire.strength},
                {&object.invert, &wire.invert},
            };
        for (const auto &read : reads) {
            if (!S.Resolved(read.first->head, read.first->fallback,
                            read.second, &why)) {
                Fail(why);
                return;
            }
        }
        const std::pair<v4::RigExecWireInput *, float> unread[] = {
            {&wire.falloffMin, 0.0f}, {&wire.falloffMax, 1.0f},
            {&wire.scaleXPos, 1.0f},  {&wire.scaleYPos, 1.0f},
            {&wire.scaleZPos, 1.0f},  {&wire.scaleXNeg, 1.0f},
            {&wire.scaleYNeg, 1.0f},  {&wire.scaleZNeg, 1.0f},
            {&wire.scaleX, 1.0f},     {&wire.scaleY, 1.0f},
            {&wire.scaleZ, 1.0f},     {&wire.extentU, 1.0f},
            {&wire.extentV, 1.0f},
        };
        for (const auto &read : unread) {
            S.Resolved(UsdAttribute(), read.second, read.first, &why);
        }
        C.weightObjects.push_back(std::move(wire));
    }

    // The oracle facts, for every entry.
    std::vector<SdfPath> objectPaths(C.weightObjects.size());
    std::vector<SdfPath> timeVarying(C.weightObjects.size());
    for (size_t i = 0; i < C.weightObjects.size(); ++i) {
        const bool stepBacked = i < B.weightObjects.size();
        const SdfPath &path =
            stepBacked ? B.weightObjects[i].path
                       : envelopes[i - B.weightObjects.size()].path;
        const TfToken &type =
            stepBacked ? B.weightObjects[i].type
                       : envelopes[i - B.weightObjects.size()].type;
        RigExecWeightOracleFacts facts;
        RigExecBakedDescribeWeightOracle(evaluator, path, S.bakeTime, &facts);
        v4::RigExecWireWeightObject &wire = C.weightObjects[i];
        wire.samplesInFlight = facts.samplesInFlight;
        wire.oracleSamples =
            facts.haveSamples ? S.InternPoints(facts.samples) : -1;
        wire.oracleCurve = facts.haveCurve ? S.InternPoints(facts.curve) : -1;
        if (type == TfToken("RigExecPlaneWeight")) {
            wire.oraclePlaneAxis = token(facts.planeAxis);
            wire.oraclePlaneBounds = token(facts.planeBounds);
        }
        wire.oraclePhaseError = facts.phaseError;
        wire.oracleStaticError = facts.staticError;
        objectPaths[i] = path;
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
    if (!RigExecBakedDescribePropertyChains(evaluator, &chainDescs, &why)) {
        Fail(why);
        return;
    }
    C.propertyChains.reserve(chainDescs.size());
    for (const RigExecBakedPropertyChainDesc &desc : chainDescs) {
        using ValueType = RigExecBakedPropertyChainDesc::ValueType;
        v4::PropertyChain chain;
        chain.valueType = _PropertyType(desc.valueType);
        if (!S.Slot(desc.target, "property chain " + desc.target.GetString(),
                    &chain.target, &why)) {
            Fail(why);
            return;
        }
        // value, min and max as the chain's arm reads them: float for a
        // double chain too, which is computed in float.
        v4::RigExecWireValue operand = _Float(0.0f);
        if (desc.valueType == ValueType::Vec3f) {
            operand = _Zero(InputTag::Vec3f);
        } else if (desc.valueType == ValueType::Matrix4d) {
            operand = _Identity();
        }
        for (const RigExecBakedPropertyChainDesc::Revision &r :
             desc.revisions) {
            v4::PropertyRevision revision;
            revision.mover = writer->AddString(r.mover.GetString());
            revision.op = _PropertyOp(r.opValid, r.op);
            const bool matrix = desc.valueType == ValueType::Matrix4d;
            if (!S.Property(r.enabled, _Bool(true), &revision.enabled,
                            &why) ||
                !S.Property(r.defaultWeight, _Float(1.0f),
                            &revision.defaultWeight, &why) ||
                !S.Property(r.value, operand, &revision.value, &why) ||
                !S.Property(matrix ? UsdAttribute() : r.minimum, operand,
                            &revision.min, &why) ||
                !S.Property(matrix ? UsdAttribute() : r.maximum, operand,
                            &revision.max, &why)) {
                Fail(why);
                return;
            }
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
            v4::PhasedConsumer phased;
            phased.chain = uint32_t(c);
            phased.consumerType = _PropertyType(p.consumerType);
            phased.applied = uint32_t(p.applied);
            if (!S.Slot(p.consumer,
                        "phased consumer " + p.consumer.GetString(),
                        &phased.consumer, &why)) {
                Fail(why);
                return;
            }
            C.phasedConsumers.push_back(phased);
        }
    }

    // The program's registered reads that cross a chain target: each reads
    // the long way, through the chain results, and the frame records hold
    // what it read under the input's uid.
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
        const std::string head = input.resolvedAttr.GetPath().GetString();
        if (!input.head ||
            input.head.GetPath() != input.resolvedAttr.GetPath()) {
            why = "input " + head + " reads the long way from another "
                  "attribute than its head";
            readsOk = false;
            return;
        }
        const int64_t uid = records.FindUid(&input);
        if (uid < 0) {
            why = "input " + head + " reads through a property chain, but "
                  "the frame records give it no uid";
            readsOk = false;
            return;
        }
        RigExecWireChainRead entry;
        entry.uid = uint32_t(uid);
        readsOk = S.Baked(input, &entry.read, &why);
        if (readsOk) {
            C.chainReads.push_back(std::move(entry));
        }
    });
    if (!readsOk) {
        Fail(why);
        return;
    }
    std::sort(C.chainReads.begin(), C.chainReads.end(),
              [](const RigExecWireChainRead &a,
                 const RigExecWireChainRead &b) { return a.uid < b.uid; });

    // Every registered read, in the frozen context's walk over the
    // program's tables, so the list cannot drift from the program. The row
    // and field each sits at in the runtime's tables come from the
    // enumeration below, in the capture's field order; the two must name
    // the same inputs.
    {
        using Family = RigExecWireRegisteredFamily;
        struct _Where {
            Family family;
            uint32_t object;
            uint32_t field;
            int32_t avar;
        };
        std::map<const void *, _Where> where;
        const auto place = [&](const void *input, Family family,
                               size_t object, size_t field, int64_t avar) {
            where.emplace(input, _Where{family, uint32_t(object),
                                        uint32_t(field), int32_t(avar)});
        };
        for (size_t i = 0; i < B.avarBindings.size(); ++i) {
            place(&B.avarBindings[i].input, Family::AvarBinding, i, 0,
                  int64_t(B.avarBindings[i].slot));
        }
        for (size_t i = 0; i < B.avarConstantBindings.size(); ++i) {
            place(&B.avarConstantBindings[i].input,
                  Family::AvarConstantBinding, i, 0,
                  int64_t(B.avarConstantBindings[i].slot));
        }
        for (size_t i = 0; i < B.ladders.size(); ++i) {
            const RigExecBakedProgramImpl::Ladder &ladder = B.ladders[i];
            place(&ladder.restSpace, Family::Ladder, i, 0, -1);
            place(&ladder.defaultSpace, Family::Ladder, i, 1, -1);
            place(&ladder.posedSpace, Family::Ladder, i, 2, -1);
            for (size_t k = 0; k < 6; ++k) {
                place(&ladder.restAvars[k], Family::Ladder, i, 3 + k, -1);
                place(&ladder.defaultAvars[k], Family::Ladder, i, 9 + k, -1);
            }
            place(&ladder.rotationOrder, Family::Ladder, i, 15, -1);
        }
        for (size_t i = 0; i < B.spaceSwitches.size(); ++i) {
            place(&B.spaceSwitches[i].activeInput, Family::SpaceSwitch, i, 0,
                  -1);
        }
        for (size_t i = 0; i < B.poseInterpolators.size(); ++i) {
            const auto &interp = B.poseInterpolators[i];
            place(&interp.enabled, Family::Interpolator, i, 0, -1);
            for (size_t k = 0; k < interp.valueInputs.size(); ++k) {
                place(&interp.valueInputs[k], Family::Interpolator, i, 1 + k,
                      -1);
            }
        }
        for (size_t i = 0; i < B.solvers.size(); ++i) {
            const RigExecBakedProgramImpl::Solver &s = B.solvers[i];
            const void *fields[] = {
                &s.bend,           &s.upperOffset,    &s.lowerOffset,
                &s.stretch,        &s.softness,       &s.blendWeight,
                &s.preserveVolume, &s.midFollowWeight, &s.roll,
                &s.twist,          &s.minLengthRatio, &s.twistTurns,
                &s.ribbonSampleCount, &s.ikSpace};
            for (size_t f = 0; f < std::size(fields); ++f) {
                place(fields[f], Family::Solver, i, f, -1);
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
                &c.worldUpVector, &c.poleVector, &c.twistDegrees};
            for (size_t f = 0; f < std::size(fields); ++f) {
                place(fields[f], Family::Constraint, i, f, -1);
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
                place(fields[f], Family::WeightObject, i, f, -1);
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
            RigExecWireRegisteredRead entry;
            entry.family = found->second.family;
            entry.object = found->second.object;
            entry.field = found->second.field;
            entry.avar = found->second.avar;
            entry.uid = int32_t(records.FindUid(&input));
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
    // weight itself), a revision's inputs:defaultWeight (RevisionStatic),
    // and the scalar mover inputs the assemblers read through their
    // connections and record under the head's path.
    {
        const TfToken defaultWeightName("inputs:defaultWeight");
        const auto blendReads =
            [&](const RigExecBakedProgramImpl::GeomRevision &revision,
                size_t chain, size_t index, bool derived) {
                for (size_t ch = 0; ch < revision.blendChannels.size();
                     ++ch) {
                    const auto &bound = revision.blendChannels[ch];
                    RigExecWireBlendWeightRead entry;
                    entry.chain = uint32_t(chain);
                    entry.revision = uint32_t(index);
                    entry.derived = derived;
                    entry.channel = uint32_t(ch);
                    if (!S.Resolved(bound.weight, 0.0f, &entry.read, &why)) {
                        return false;
                    }
                    C.blendWeightReads.push_back(std::move(entry));
                }
                return true;
            };
        // The record keys these reads by the head's path alone, so one
        // attribute read as two types cannot be compared and is refused.
        std::map<SdfPath, std::pair<v4::InputTag, bool>> pathSeen;
        const auto pathRead = [&](const UsdAttribute &a,
                                  const v4::RigExecWireValue &fallback,
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
            RigExecWirePathScalarRead entry;
            entry.path = writer->AddString(a.GetPath().GetString());
            entry.widen = widen;
            // Every site below reads like moverGraph.cpp's _Read: the
            // resolved walk, then the head's own value, then the fallback.
            entry.headFallback = true;
            if (!S.Resolved(a, fallback, &entry.read, &why)) {
                return false;
            }
            C.pathScalarReads.push_back(std::move(entry));
            return true;
        };
        // The sites moverGraph.cpp records with forceFrame: _Enabled on every
        // authored mover that assembles parameters, the skin's elementSize
        // and skinningMethod, the
        // iterative movers' scalars (_RecordedInput), and a surface
        // projector's dials and ray settings.
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
                RigExecWireDefaultWeightRead entry;
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
    // and every object either composes -- must not read an animated one.
    // Composition points below the entry, so one descending pass closes the
    // set.
    std::vector<char> resolved(C.weightObjects.size(), 0);
    for (int32_t index : C.constraintWeightObjectIndex) {
        if (index >= 0) {
            resolved[size_t(index)] = 1;
        }
    }
    for (const v4::PropertyChain &chain : C.propertyChains) {
        for (const v4::PropertyRevision &revision : chain.revisions) {
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
    for (size_t i = C.weightObjects.size(); i-- > 0;) {
        if (!resolved[i]) {
            continue;
        }
        if (!timeVarying[i].IsEmpty()) {
            Fail("weight object " + objectPaths[i].GetString() + " reads " +
                 timeVarying[i].GetString() +
                 " at every frame, which is animated; the computed section "
                 "holds its value at one time only");
            return;
        }
        const v4::RigExecWireWeightObject &wire = C.weightObjects[i];
        if (wire.base >= 0) {
            resolved[size_t(wire.base)] = 1;
        }
        for (int32_t input : wire.inputs) {
            resolved[size_t(input)] = 1;
        }
    }

    // Every slot is listed; listed slots are ordered by path text.
    std::vector<std::string> names;
    names.reserve(S.slots.size());
    for (const UsdAttribute &a : S.slots) {
        names.push_back(a.GetPath().GetString());
    }
    std::vector<uint32_t> order(S.slots.size());
    std::iota(order.begin(), order.end(), 0u);
    std::stable_sort(order.begin(), order.end(),
                     [&](uint32_t a, uint32_t b) {
                         return names[a] < names[b];
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
    const auto remapWalk = [&](v4::RigExecWireInput &input) {
        for (uint32_t &slot : input.walk) {
            slot = remap[slot];
        }
    };
    for (v4::RigExecWireWeightObject &object : C.weightObjects) {
        _ForEachRead(object, remapWalk);
    }
    for (v4::PropertyChain &chain : C.propertyChains) {
        chain.target = remap[chain.target];
        for (v4::PropertyRevision &revision : chain.revisions) {
            _ForEachRevisionRead(revision, remapWalk);
        }
    }
    for (v4::PhasedConsumer &phased : C.phasedConsumers) {
        phased.consumer = remap[phased.consumer];
    }
    for (RigExecWireChainRead &entry : C.chainReads) {
        remapWalk(entry.read);
    }
    for (RigExecWireRegisteredRead &entry : C.registeredReads) {
        remapWalk(entry.read);
    }
    for (RigExecWireBlendWeightRead &entry : C.blendWeightReads) {
        remapWalk(entry.read);
    }
    for (RigExecWireDefaultWeightRead &entry : C.defaultWeightReads) {
        remapWalk(entry.read);
    }
    for (RigExecWirePathScalarRead &entry : C.pathScalarReads) {
        remapWalk(entry.read);
    }
    C.inputs.reserve(S.slots.size());
    for (size_t s = 0; s < S.slots.size(); ++s) {
        const UsdAttribute &a = S.slots[s];
        v4::InputSlot slot;
        slot.name = writer->AddString(names[order[s]]);
        slot.type = S.slotTags[s];
        bool has = false;
        slot.value = S.ReadSlot(a, slot.type, S.bakeTime, &has);
        slot.flags = _Bit(v4::InputSlotFlags::Listed);
        if (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0) {
            slot.flags |= _Bit(v4::InputSlotFlags::Animated);
        }
        if (has) {
            slot.flags |= _Bit(v4::InputSlotFlags::HasValue);
        }
        C.inputs.push_back(slot);
    }
    C.listedInputs = uint32_t(C.inputs.size());
    // Each slot names the chain it is the target of and the phased consumer
    // publishing at it; one of each at most.
    for (size_t c = 0; c < C.propertyChains.size(); ++c) {
        v4::InputSlot &slot = C.inputs[C.propertyChains[c].target];
        if (slot.chain >= 0) {
            Fail("two property chains share the target " +
                 names[order[C.propertyChains[c].target]]);
            return;
        }
        slot.chain = int32_t(c);
    }
    for (size_t k = 0; k < C.phasedConsumers.size(); ++k) {
        v4::InputSlot &slot = C.inputs[C.phasedConsumers[k].consumer];
        if (slot.phased >= 0) {
            Fail("two phased connections publish at " +
                 names[order[C.phasedConsumers[k].consumer]]);
            return;
        }
        slot.phased = int32_t(k);
    }
    _valid = true;
}

RigExecBakeComputedCapture::~RigExecBakeComputedCapture() = default;

bool
RigExecBakeComputedCapture::RecordFrame(double frame, std::string *error)
{
    if (!_valid) {
        if (error) {
            *error = "the computed capture is not armed";
        }
        return false;
    }
    _State &S = *_state;
    RigExecWireComputedFrame record;
    record.frame = frame;
    record.values.reserve(S.slots.size());
    record.hasValue.reserve(S.slots.size());
    const UsdTimeCode time(frame);
    for (size_t s = 0; s < S.slots.size(); ++s) {
        bool has = false;
        record.values.push_back(
            S.ReadSlot(S.slots[s], S.slotTags[s], time, &has));
        record.hasValue.push_back(has ? uint8_t(1) : uint8_t(0));
    }
    S.computed.frames.push_back(std::move(record));
    return true;
}

const RigExecWireComputed &
RigExecBakeComputedCapture::GetComputed() const
{
    return _state->computed;
}

}  // namespace rigExec
