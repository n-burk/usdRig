// .rigexec Computed section (tag 18): wire encoding. Temporary; see
// computed.h.
#include "rigExecBinary/computed.h"

#include <algorithm>
#include <tuple>

namespace rigExec {
namespace {

/// Layout number of this hand codec, first in the payload, so a skew
/// between bake and runtime builds fails rather than misreads.
constexpr uint32_t _ComputedLayout = 6;

bool
_Fail(std::string *error, const std::string &what)
{
    if (error) {
        *error = "computed section: " + what;
    }
    return false;
}

// Writing.

void
_PutU32s(std::vector<uint8_t> *out, const std::vector<uint32_t> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (uint32_t value : values) {
        RigExecWirePutU32(out, value);
    }
}

void
_PutI32s(std::vector<uint8_t> *out, const std::vector<int32_t> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (int32_t value : values) {
        RigExecWirePutI32(out, value);
    }
}

void
_PutU8s(std::vector<uint8_t> *out, const std::vector<uint8_t> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    out->insert(out->end(), values.begin(), values.end());
}

void
_PutF32s(std::vector<uint8_t> *out, const std::vector<float> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (float value : values) {
        RigExecWirePutF32(out, value);
    }
}

void
_PutString(std::vector<uint8_t> *out, const std::string &text)
{
    RigExecWirePutU32(out, uint32_t(text.size()));
    out->insert(out->end(), text.begin(), text.end());
}

void
_PutValue(std::vector<uint8_t> *out, const v4::RigExecWireValue &value)
{
    RigExecWirePutU8(out, uint8_t(value.tag));
    RigExecWirePutU64(out, value.bits);
    switch (value.tag) {
    case v4::InputTag::Matrix4d:
        RigExecWirePutMatrix4d(out, value.matrix);
        break;
    case v4::InputTag::Vec3d:
        RigExecWirePutVec3d(out, value.vec3d);
        break;
    case v4::InputTag::Vec3f:
        RigExecWirePutVec3f(out, value.vec3f);
        break;
    default:
        break;
    }
}

void
_PutInput(std::vector<uint8_t> *out, const v4::RigExecWireInput &input)
{
    RigExecWirePutU8(out, uint8_t(input.tag));
    RigExecWirePutU8(out, uint8_t(input.mode));
    RigExecWirePutU8(out, input.flags);
    RigExecWirePutI32(out, input.overrideIndex);
    RigExecWirePutU32(out, input.constant);
    _PutU32s(out, input.walk);
    RigExecWirePutI32(out, int32_t(input.selected));
}

/// Every read of a weight object, in encoding order.
template <class Object, class Visit>
void
_ForEachInput(Object &object, const Visit &visit)
{
    for (auto *input :
         {&object.defaultWeight, &object.driver, &object.scale, &object.bias,
          &object.strength, &object.invert, &object.falloffMin,
          &object.falloffMax, &object.scaleXPos, &object.scaleYPos,
          &object.scaleZPos, &object.scaleXNeg, &object.scaleYNeg,
          &object.scaleZNeg, &object.scaleX, &object.scaleY, &object.scaleZ,
          &object.extentU, &object.extentV}) {
        visit(*input);
    }
}

void
_PutWeightObject(std::vector<uint8_t> *out,
                 const v4::RigExecWireWeightObject &object)
{
    RigExecWirePutU32(out, object.path);
    RigExecWirePutU32(out, object.type);
    RigExecWirePutU32(out, object.representation);
    RigExecWirePutU32(out, object.rangePolicy);
    _PutF32s(out, object.values);
    _PutI32s(out, object.indices);
    RigExecWirePutI32(out, object.base);
    _PutI32s(out, object.inputs);
    RigExecWirePutU32(out, object.combineMode);
    _PutU32s(out, object.combineTargetPoints);
    _PutU8s(out, object.combineTargetValid);
    RigExecWirePutU64(out, object.costElements);
    RigExecWirePutI32(out, object.providerSlot);
    RigExecWirePutU32(out, object.planeAxis);
    RigExecWirePutU32(out, object.planeBounds);
    _PutU32s(out, object.targetPoints);
    _PutU8s(out, object.targetValid);
    _PutU32s(out, object.samplePoints);
    _PutU8s(out, object.sampleValid);
    _PutU32s(out, object.curvePoints);
    _PutU8s(out, object.curveValid);
    _PutF32s(out, object.falloffCurve);
    _ForEachInput(object, [&](const v4::RigExecWireInput &input) {
        _PutInput(out, input);
    });
    RigExecWirePutU8(out, object.envelopeOnly ? 1 : 0);
    RigExecWirePutU8(out, object.samplesInFlight ? 1 : 0);
    RigExecWirePutI32(out, object.oracleSamples);
    RigExecWirePutI32(out, object.oracleCurve);
    RigExecWirePutU32(out, object.oraclePlaneAxis);
    RigExecWirePutU32(out, object.oraclePlaneBounds);
    _PutString(out, object.oraclePhaseError);
    _PutString(out, object.oracleStaticError);
}

void
_PutVec2fs(std::vector<uint8_t> *out,
           const std::vector<RigExecWireVec2f> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const RigExecWireVec2f &value : values) {
        RigExecWirePutVec2f(out, value);
    }
}

/// Every read of a property revision, in encoding order.
template <class Revision, class Visit>
void
_ForEachRevisionInput(Revision &revision, const Visit &visit)
{
    for (auto *input : {&revision.enabled, &revision.defaultWeight,
                        &revision.value, &revision.min, &revision.max}) {
        visit(*input);
    }
}

void
_PutPropertyChain(std::vector<uint8_t> *out, const v4::PropertyChain &chain)
{
    RigExecWirePutU32(out, chain.target);
    RigExecWirePutU8(out, uint8_t(chain.valueType));
    RigExecWirePutU32(out, uint32_t(chain.revisions.size()));
    for (const v4::PropertyRevision &revision : chain.revisions) {
        RigExecWirePutU32(out, revision.mover);
        RigExecWirePutU8(out, uint8_t(revision.op));
        RigExecWirePutI32(out, revision.envelope);
        _ForEachRevisionInput(revision, [&](const v4::RigExecWireInput &input) {
            _PutInput(out, input);
        });
        _PutVec2fs(out, revision.keys);
        RigExecWirePutU8(out, revision.hasTangentsAttr ? 1 : 0);
        _PutVec2fs(out, revision.tangents);
    }
}

// Reading. Every count is checked against the bytes left before anything
// is sized, at the element's smallest encoded size.

bool
_ReadCount(RigExecWireReader *reader, size_t elementBytes, uint32_t *count)
{
    return reader->ReadU32(count) &&
           size_t(*count) <= reader->Remaining() / elementBytes;
}

bool
_ReadU32s(RigExecWireReader *reader, std::vector<uint32_t> *values)
{
    uint32_t count = 0;
    if (!_ReadCount(reader, 4, &count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t &value : *values) {
        if (!reader->ReadU32(&value)) {
            return false;
        }
    }
    return true;
}

bool
_ReadI32s(RigExecWireReader *reader, std::vector<int32_t> *values)
{
    uint32_t count = 0;
    if (!_ReadCount(reader, 4, &count)) {
        return false;
    }
    values->resize(count);
    for (int32_t &value : *values) {
        if (!reader->ReadI32(&value)) {
            return false;
        }
    }
    return true;
}

bool
_ReadU8s(RigExecWireReader *reader, std::vector<uint8_t> *values)
{
    uint32_t count = 0;
    if (!_ReadCount(reader, 1, &count)) {
        return false;
    }
    values->clear();
    return reader->ReadBytes(count, values);
}

bool
_ReadF32s(RigExecWireReader *reader, std::vector<float> *values)
{
    uint32_t count = 0;
    if (!_ReadCount(reader, 4, &count)) {
        return false;
    }
    values->resize(count);
    for (float &value : *values) {
        if (!reader->ReadF32(&value)) {
            return false;
        }
    }
    return true;
}

bool
_ReadString(RigExecWireReader *reader, std::string *text)
{
    uint32_t count = 0;
    if (!_ReadCount(reader, 1, &count)) {
        return false;
    }
    std::vector<uint8_t> bytes;
    if (!reader->ReadBytes(count, &bytes)) {
        return false;
    }
    text->assign(bytes.begin(), bytes.end());
    return true;
}

bool
_ReadTag(RigExecWireReader *reader, v4::InputTag *tag)
{
    uint8_t raw = 0;
    if (!reader->ReadU8(&raw) || raw > v4::InputTagLast) {
        return false;
    }
    *tag = v4::InputTag(raw);
    return true;
}

bool
_ReadValue(RigExecWireReader *reader, v4::RigExecWireValue *value)
{
    if (!_ReadTag(reader, &value->tag) || !reader->ReadU64(&value->bits)) {
        return false;
    }
    switch (value->tag) {
    case v4::InputTag::Matrix4d:
        return RigExecWireReadMatrix4d(reader, &value->matrix);
    case v4::InputTag::Vec3d:
        return RigExecWireReadVec3d(reader, &value->vec3d);
    case v4::InputTag::Vec3f:
        return RigExecWireReadVec3f(reader, &value->vec3f);
    default:
        return true;
    }
}

bool
_ReadInput(RigExecWireReader *reader, v4::RigExecWireInput *input)
{
    uint8_t mode = 0;
    int32_t selected = -1;
    if (!_ReadTag(reader, &input->tag) || !reader->ReadU8(&mode) ||
        mode > v4::ReadModeLast || !reader->ReadU8(&input->flags) ||
        !reader->ReadI32(&input->overrideIndex) ||
        !reader->ReadU32(&input->constant) ||
        !_ReadU32s(reader, &input->walk) || !reader->ReadI32(&selected) ||
        selected < -1 || selected > 32767) {
        return false;
    }
    input->mode = v4::ReadMode(mode);
    input->selected = int16_t(selected);
    return true;
}

bool
_ReadWeightObject(RigExecWireReader *reader,
                  v4::RigExecWireWeightObject *object)
{
    uint8_t envelopeOnly = 0, samplesInFlight = 0;
    if (!reader->ReadU32(&object->path) || !reader->ReadU32(&object->type) ||
        !reader->ReadU32(&object->representation) ||
        !reader->ReadU32(&object->rangePolicy) ||
        !_ReadF32s(reader, &object->values) ||
        !_ReadI32s(reader, &object->indices) ||
        !reader->ReadI32(&object->base) ||
        !_ReadI32s(reader, &object->inputs) ||
        !reader->ReadU32(&object->combineMode) ||
        !_ReadU32s(reader, &object->combineTargetPoints) ||
        !_ReadU8s(reader, &object->combineTargetValid) ||
        !reader->ReadU64(&object->costElements) ||
        !reader->ReadI32(&object->providerSlot) ||
        !reader->ReadU32(&object->planeAxis) ||
        !reader->ReadU32(&object->planeBounds) ||
        !_ReadU32s(reader, &object->targetPoints) ||
        !_ReadU8s(reader, &object->targetValid) ||
        !_ReadU32s(reader, &object->samplePoints) ||
        !_ReadU8s(reader, &object->sampleValid) ||
        !_ReadU32s(reader, &object->curvePoints) ||
        !_ReadU8s(reader, &object->curveValid) ||
        !_ReadF32s(reader, &object->falloffCurve)) {
        return false;
    }
    bool ok = true;
    _ForEachInput(*object, [&](v4::RigExecWireInput &input) {
        ok = ok && _ReadInput(reader, &input);
    });
    if (!ok || !reader->ReadU8(&envelopeOnly) || envelopeOnly > 1 ||
        !reader->ReadU8(&samplesInFlight) || samplesInFlight > 1 ||
        !reader->ReadI32(&object->oracleSamples) ||
        !reader->ReadI32(&object->oracleCurve) ||
        !reader->ReadU32(&object->oraclePlaneAxis) ||
        !reader->ReadU32(&object->oraclePlaneBounds) ||
        !_ReadString(reader, &object->oraclePhaseError) ||
        !_ReadString(reader, &object->oracleStaticError)) {
        return false;
    }
    object->envelopeOnly = envelopeOnly != 0;
    object->samplesInFlight = samplesInFlight != 0;
    return true;
}

bool
_ReadVec2fs(RigExecWireReader *reader, std::vector<RigExecWireVec2f> *values)
{
    uint32_t count = 0;
    if (!_ReadCount(reader, 8, &count)) {
        return false;
    }
    values->resize(count);
    for (RigExecWireVec2f &value : *values) {
        if (!RigExecWireReadVec2f(reader, &value)) {
            return false;
        }
    }
    return true;
}

// The smallest encoded read: tag, mode, flags, override, constant, an empty
// walk and selected.
constexpr size_t _InputMinBytes = 1 + 1 + 1 + 4 + 4 + 4 + 4;

bool
_ReadPropertyChain(RigExecWireReader *reader, v4::PropertyChain *chain)
{
    uint8_t valueType = 0;
    uint32_t count = 0;
    // Mover, op, envelope, five reads, two empty arrays and the flag.
    constexpr size_t revisionMinBytes = 4 + 1 + 4 + 5 * _InputMinBytes + 9;
    if (!reader->ReadU32(&chain->target) || !reader->ReadU8(&valueType) ||
        valueType > v4::PropertyValueTypeLast ||
        !_ReadCount(reader, revisionMinBytes, &count)) {
        return false;
    }
    chain->valueType = v4::PropertyValueType(valueType);
    chain->revisions.resize(count);
    for (v4::PropertyRevision &revision : chain->revisions) {
        uint8_t op = 0, hasTangentsAttr = 0;
        if (!reader->ReadU32(&revision.mover) || !reader->ReadU8(&op) ||
            op > v4::PropertyOpLast || !reader->ReadI32(&revision.envelope)) {
            return false;
        }
        revision.op = v4::PropertyOp(op);
        bool ok = true;
        _ForEachRevisionInput(revision, [&](v4::RigExecWireInput &input) {
            ok = ok && _ReadInput(reader, &input);
        });
        if (!ok || !_ReadVec2fs(reader, &revision.keys) ||
            !reader->ReadU8(&hasTangentsAttr) || hasTangentsAttr > 1 ||
            !_ReadVec2fs(reader, &revision.tangents)) {
            return false;
        }
        revision.hasTangentsAttr = hasTangentsAttr != 0;
    }
    return true;
}

// The section's own consistency, shared by both directions.

bool
_CheckValue(const v4::RigExecWireValue &value)
{
    if (uint8_t(value.tag) > v4::InputTagLast) {
        return false;
    }
    switch (value.tag) {
    case v4::InputTag::Bool:
        return value.bits <= 1;
    case v4::InputTag::Float:
    case v4::InputTag::Int:
    case v4::InputTag::Token:
        return (value.bits >> 32) == 0;
    case v4::InputTag::Matrix4d:
    case v4::InputTag::Vec3d:
    case v4::InputTag::Vec3f:
        return value.bits == 0;
    default:
        return true;
    }
}

bool
_CheckInput(const RigExecWireComputed &computed,
            const v4::RigExecWireInput &input)
{
    if (uint8_t(input.tag) > v4::InputTagLast ||
        uint8_t(input.mode) > v4::ReadModeLast ||
        (input.flags & ~uint8_t(v4::InputReadFlags::ANY)) != 0 ||
        input.overrideIndex < -1 ||
        input.constant >= computed.values.size() ||
        computed.values[input.constant].tag != input.tag ||
        input.walk.size() > 32767 || input.selected < -1 ||
        (input.selected >= 0 && size_t(input.selected) >= input.walk.size())) {
        return false;
    }
    if (input.mode != v4::ReadMode::Baked &&
        (input.overrideIndex != -1 || input.selected != -1 ||
         (input.flags & uint8_t(v4::InputReadFlags::LongWay)) != 0)) {
        return false;
    }
    for (uint32_t slot : input.walk) {
        if (slot >= computed.inputs.size()) {
            return false;
        }
    }
    return true;
}

/// The tag a revision's value, min and max reads carry in a chain of
/// \p type: a double chain is computed in float.
v4::InputTag
_ChainReadTag(v4::PropertyValueType type)
{
    switch (type) {
    case v4::PropertyValueType::Matrix4d:
        return v4::InputTag::Matrix4d;
    case v4::PropertyValueType::Vec3f:
        return v4::InputTag::Vec3f;
    default:
        return v4::InputTag::Float;
    }
}

bool
_IsScalar(v4::PropertyValueType type)
{
    return type == v4::PropertyValueType::Float ||
           type == v4::PropertyValueType::Double;
}

/// A revision read: Pinned over at most its own attribute, or Resolved
/// over a connection walk, carrying \p tag.
bool
_CheckRevisionInput(const RigExecWireComputed &computed,
                    const v4::RigExecWireInput &input, v4::InputTag tag)
{
    return _CheckInput(computed, input) && input.tag == tag &&
           ((input.mode == v4::ReadMode::Pinned && input.walk.size() <= 1) ||
            input.mode == v4::ReadMode::Resolved);
}

bool
_CheckPropertyChains(const RigExecWireComputed &computed, std::string *error)
{
    const size_t slots = computed.inputs.size();
    for (size_t i = 0; i < slots; ++i) {
        const v4::InputSlot &slot = computed.inputs[i];
        if ((slot.chain >= 0 &&
             (size_t(slot.chain) >= computed.propertyChains.size() ||
              computed.propertyChains[size_t(slot.chain)].target != i)) ||
            (slot.phased >= 0 &&
             (size_t(slot.phased) >= computed.phasedConsumers.size() ||
              computed.phasedConsumers[size_t(slot.phased)].consumer != i))) {
            return _Fail(error, "input slot " + std::to_string(i) +
                                    " names a chain or phased consumer that "
                                    "does not name it");
        }
    }
    for (size_t c = 0; c < computed.propertyChains.size(); ++c) {
        const v4::PropertyChain &chain = computed.propertyChains[c];
        const std::string where = "property chain " + std::to_string(c);
        if (uint8_t(chain.valueType) > v4::PropertyValueTypeLast ||
            chain.target >= slots ||
            computed.inputs[chain.target].chain != int32_t(c)) {
            return _Fail(error, where + " has a malformed target");
        }
        const v4::InputTag tag = _ChainReadTag(chain.valueType);
        const bool matrix =
            chain.valueType == v4::PropertyValueType::Matrix4d;
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const v4::PropertyRevision &revision = chain.revisions[r];
            const std::string at = where + " revision " + std::to_string(r);
            if (uint8_t(revision.op) > v4::PropertyOpLast ||
                revision.envelope < -1 ||
                (revision.envelope >= 0 &&
                 size_t(revision.envelope) >= computed.weightObjects.size())) {
                return _Fail(error, at + " has a malformed operation or "
                                         "envelope");
            }
            if (!_CheckRevisionInput(computed, revision.enabled,
                                     v4::InputTag::Bool) ||
                !_CheckRevisionInput(computed, revision.defaultWeight,
                                     v4::InputTag::Float) ||
                !_CheckRevisionInput(computed, revision.value, tag) ||
                !_CheckRevisionInput(computed, revision.min, tag) ||
                !_CheckRevisionInput(computed, revision.max, tag) ||
                (matrix &&
                 (!revision.min.walk.empty() || !revision.max.walk.empty()))) {
                return _Fail(error, at + " has a malformed read");
            }
        }
    }
    for (size_t k = 0; k < computed.phasedConsumers.size(); ++k) {
        const v4::PhasedConsumer &phased = computed.phasedConsumers[k];
        const std::string where = "phased consumer " + std::to_string(k);
        if (phased.chain >= computed.propertyChains.size() ||
            (k > 0 && phased.chain < computed.phasedConsumers[k - 1].chain) ||
            phased.consumer >= slots ||
            computed.inputs[phased.consumer].phased != int32_t(k) ||
            uint8_t(phased.consumerType) > v4::PropertyValueTypeLast) {
            return _Fail(error, where + " is malformed");
        }
        const v4::PropertyChain &chain =
            computed.propertyChains[phased.chain];
        if ((phased.consumerType != chain.valueType &&
             !(_IsScalar(phased.consumerType) &&
               _IsScalar(chain.valueType))) ||
            phased.applied > chain.revisions.size()) {
            return _Fail(error, where + " does not fit its chain");
        }
    }
    return true;
}

/// A registered read crossing a chain target: Baked, read per run the long
/// way, a chain target on its walk, of a type the frame records hold (their
/// tags end at Vec3d), in ascending uid.
bool
_CheckChainReads(const RigExecWireComputed &computed, std::string *error)
{
    const uint8_t required = uint8_t(v4::InputReadFlags::Varying) |
                             uint8_t(v4::InputReadFlags::LongWay) |
                             uint8_t(v4::InputReadFlags::ViaChain);
    for (size_t k = 0; k < computed.chainReads.size(); ++k) {
        const RigExecWireChainRead &entry = computed.chainReads[k];
        const v4::RigExecWireInput &read = entry.read;
        const std::string where = "chain read " + std::to_string(k);
        if (k > 0 && entry.uid <= computed.chainReads[k - 1].uid) {
            return _Fail(error, where + " is out of uid order");
        }
        bool ok = _CheckInput(computed, read) &&
                  read.mode == v4::ReadMode::Baked &&
                  (read.flags & required) == required && read.selected == -1 &&
                  uint8_t(read.tag) <= uint8_t(v4::InputTag::Vec3d);
        bool crossesChain = false;
        if (ok) {
            for (uint32_t slot : read.walk) {
                crossesChain =
                    crossesChain || computed.inputs[slot].chain >= 0;
            }
        }
        if (!ok || !crossesChain) {
            return _Fail(error, where + " is malformed");
        }
    }
    return true;
}

bool
_IsAvarFamily(RigExecWireRegisteredFamily family)
{
    return family == RigExecWireRegisteredFamily::AvarBinding ||
           family == RigExecWireRegisteredFamily::AvarConstantBinding;
}

/// The registered reads: Baked, an avar index exactly on the avar
/// families; and the geometry reads: Resolved, a blend or default weight a
/// Float, one blend read per channel and one default weight read per
/// revision, a path read headed by the attribute it is keyed by and unique
/// by that path (a widened read is a Float).
bool
_CheckReadTables(const RigExecWireComputed &computed, std::string *error)
{
    for (size_t k = 0; k < computed.registeredReads.size(); ++k) {
        const RigExecWireRegisteredRead &entry = computed.registeredReads[k];
        if (uint8_t(entry.family) > RigExecWireRegisteredFamilyLast ||
            !_CheckInput(computed, entry.read) ||
            entry.read.mode != v4::ReadMode::Baked || entry.uid < -1 ||
            entry.avar < -1 ||
            (entry.avar >= 0) != _IsAvarFamily(entry.family) ||
            (_IsAvarFamily(entry.family) && entry.field != 0)) {
            return _Fail(error, "registered read " + std::to_string(k) +
                                    " is malformed");
        }
    }
    const auto resolvedFloat = [&](const v4::RigExecWireInput &read) {
        return _CheckInput(computed, read) &&
               read.mode == v4::ReadMode::Resolved &&
               read.tag == v4::InputTag::Float;
    };
    for (size_t k = 0; k < computed.blendWeightReads.size(); ++k) {
        if (!resolvedFloat(computed.blendWeightReads[k].read)) {
            return _Fail(error, "blend weight read " + std::to_string(k) +
                                    " is malformed");
        }
    }
    for (size_t k = 0; k < computed.defaultWeightReads.size(); ++k) {
        if (!resolvedFloat(computed.defaultWeightReads[k].read)) {
            return _Fail(error, "default weight read " + std::to_string(k) +
                                    " is malformed");
        }
    }
    for (size_t k = 0; k < computed.pathScalarReads.size(); ++k) {
        const RigExecWirePathScalarRead &entry = computed.pathScalarReads[k];
        if (entry.path == 0 || !_CheckInput(computed, entry.read) ||
            entry.read.mode != v4::ReadMode::Resolved ||
            (entry.widen && entry.read.tag != v4::InputTag::Float) ||
            entry.read.walk.empty() ||
            computed.inputs[entry.read.walk[0]].name != entry.path) {
            return _Fail(error, "path read " + std::to_string(k) +
                                    " is malformed");
        }
    }
    // The runtime keys blend and default weight reads by their revision
    // and channel, and path reads by their head, so a second entry for
    // the same key would be either dropped or compared against the wrong
    // read.
    std::vector<std::tuple<uint32_t, uint32_t, bool, uint32_t>> channels;
    channels.reserve(computed.blendWeightReads.size());
    for (const RigExecWireBlendWeightRead &entry : computed.blendWeightReads) {
        channels.emplace_back(entry.chain, entry.revision, entry.derived,
                              entry.channel);
    }
    std::sort(channels.begin(), channels.end());
    if (std::adjacent_find(channels.begin(), channels.end()) !=
        channels.end()) {
        return _Fail(error, "two blend weight reads name one channel");
    }
    std::vector<std::pair<uint32_t, uint32_t>> revisions;
    revisions.reserve(computed.defaultWeightReads.size());
    for (const RigExecWireDefaultWeightRead &entry :
         computed.defaultWeightReads) {
        revisions.emplace_back(entry.chain, entry.revision);
    }
    std::sort(revisions.begin(), revisions.end());
    if (std::adjacent_find(revisions.begin(), revisions.end()) !=
        revisions.end()) {
        return _Fail(error, "two default weight reads name one revision");
    }
    std::vector<uint32_t> paths;
    paths.reserve(computed.pathScalarReads.size());
    for (const RigExecWirePathScalarRead &entry : computed.pathScalarReads) {
        paths.push_back(entry.path);
    }
    std::sort(paths.begin(), paths.end());
    if (std::adjacent_find(paths.begin(), paths.end()) != paths.end()) {
        return _Fail(error, "two path reads name one attribute");
    }
    return true;
}

bool
_Check(const RigExecWireComputed &computed, std::string *error)
{
    if (computed.values.empty() ||
        computed.values[0].tag != v4::InputTag::Double ||
        computed.values[0].bits != 0) {
        return _Fail(error, "values[0] is not Double +0.0");
    }
    for (const v4::RigExecWireValue &value : computed.values) {
        if (!_CheckValue(value)) {
            return _Fail(error, "a value carries bits its tag does not use");
        }
    }
    if (computed.vec3fArrays.empty() || !computed.vec3fArrays[0].v.empty()) {
        return _Fail(error, "vec3fArrays[0] is not the empty array");
    }
    if (computed.listedInputs > computed.inputs.size()) {
        return _Fail(error, "more listed inputs than inputs");
    }
    for (size_t i = 0; i < computed.inputs.size(); ++i) {
        const v4::InputSlot &slot = computed.inputs[i];
        const bool listed =
            (slot.flags & uint8_t(v4::InputSlotFlags::Listed)) != 0;
        if (uint8_t(slot.type) > v4::InputTagLast ||
            (slot.flags & ~uint8_t(v4::InputSlotFlags::ANY)) != 0 ||
            slot.value >= computed.values.size() ||
            computed.values[slot.value].tag != slot.type ||
            slot.chain < -1 || slot.phased < -1 ||
            listed != (i < computed.listedInputs)) {
            return _Fail(error, "input slot " + std::to_string(i) +
                                    " is malformed");
        }
    }
    bool envelopeSeen = false;
    for (size_t i = 0; i < computed.weightObjects.size(); ++i) {
        const v4::RigExecWireWeightObject &object = computed.weightObjects[i];
        const std::string where = "weight object " + std::to_string(i);
        if (envelopeSeen && !object.envelopeOnly) {
            return _Fail(error, where + " is step-backed after an "
                                        "envelope-only entry");
        }
        envelopeSeen = envelopeSeen || object.envelopeOnly;
        if (object.base < -1 || (object.base >= 0 && size_t(object.base) >= i)) {
            return _Fail(error, where + " has a base out of dependency order");
        }
        for (int32_t input : object.inputs) {
            if (input < 0 || size_t(input) >= i) {
                return _Fail(error, where +
                                        " has an input out of dependency "
                                        "order");
            }
        }
        if (object.combineTargetValid.size() !=
                object.combineTargetPoints.size() ||
            object.targetValid.size() != object.targetPoints.size() ||
            object.sampleValid.size() != object.samplePoints.size() ||
            object.curveValid.size() != object.curvePoints.size()) {
            return _Fail(error, where + " has a validity mask of the wrong "
                                        "size");
        }
        if (object.oracleSamples < -1 ||
            (object.oracleSamples >= 0 &&
             size_t(object.oracleSamples) >= computed.vec3fArrays.size()) ||
            object.oracleCurve < -1 ||
            (object.oracleCurve >= 0 &&
             size_t(object.oracleCurve) >= computed.vec3fArrays.size())) {
            return _Fail(error, where + " names a points array out of range");
        }
        bool inputsOk = true;
        _ForEachInput(object, [&](const v4::RigExecWireInput &input) {
            inputsOk = inputsOk && _CheckInput(computed, input);
        });
        if (!inputsOk) {
            return _Fail(error, where + " has a malformed read");
        }
    }
    for (int32_t index : computed.constraintWeightObjectIndex) {
        if (index < -1 ||
            (index >= 0 && size_t(index) >= computed.weightObjects.size())) {
            return _Fail(error, "a constraint envelope index is out of range");
        }
    }
    if (!_CheckPropertyChains(computed, error) ||
        !_CheckChainReads(computed, error) ||
        !_CheckReadTables(computed, error)) {
        return false;
    }
    for (size_t f = 0; f < computed.frames.size(); ++f) {
        const RigExecWireComputedFrame &frame = computed.frames[f];
        if (frame.values.size() != computed.inputs.size() ||
            frame.hasValue.size() != computed.inputs.size()) {
            return _Fail(error, "frame " + std::to_string(f) +
                                    " does not hold one value per slot");
        }
        for (size_t s = 0; s < frame.values.size(); ++s) {
            if (frame.values[s] >= computed.values.size() ||
                computed.values[frame.values[s]].tag !=
                    computed.inputs[s].type ||
                frame.hasValue[s] > 1) {
                return _Fail(error, "frame " + std::to_string(f) +
                                        " holds a malformed value for slot " +
                                        std::to_string(s));
            }
        }
    }
    return true;
}

}  // namespace

bool
RigExecWireEncodeComputed(const RigExecWireComputed &computed,
                          std::vector<uint8_t> *out, std::string *error)
{
    if (!out || !_Check(computed, error)) {
        return false;
    }
    RigExecWirePutU32(out, _ComputedLayout);
    RigExecWirePutF64(out, computed.bakeTime);
    RigExecWirePutU32(out, uint32_t(computed.values.size()));
    for (const v4::RigExecWireValue &value : computed.values) {
        _PutValue(out, value);
    }
    RigExecWirePutU32(out, uint32_t(computed.vec3fArrays.size()));
    for (const v4::RigExecWireVec3fArray &array : computed.vec3fArrays) {
        RigExecWirePutU32(out, uint32_t(array.v.size()));
        for (const RigExecWireVec3f &point : array.v) {
            RigExecWirePutVec3f(out, point);
        }
    }
    RigExecWirePutU32(out, uint32_t(computed.inputs.size()));
    for (const v4::InputSlot &slot : computed.inputs) {
        RigExecWirePutU32(out, slot.name);
        RigExecWirePutU32(out, slot.value);
        RigExecWirePutI32(out, slot.chain);
        RigExecWirePutI32(out, slot.phased);
        RigExecWirePutU8(out, uint8_t(slot.type));
        RigExecWirePutU8(out, slot.flags);
    }
    RigExecWirePutU32(out, computed.listedInputs);
    RigExecWirePutU32(out, uint32_t(computed.weightObjects.size()));
    for (const v4::RigExecWireWeightObject &object : computed.weightObjects) {
        _PutWeightObject(out, object);
    }
    _PutI32s(out, computed.constraintWeightObjectIndex);
    RigExecWirePutU32(out, uint32_t(computed.propertyChains.size()));
    for (const v4::PropertyChain &chain : computed.propertyChains) {
        _PutPropertyChain(out, chain);
    }
    RigExecWirePutU32(out, uint32_t(computed.phasedConsumers.size()));
    for (const v4::PhasedConsumer &phased : computed.phasedConsumers) {
        RigExecWirePutU32(out, phased.chain);
        RigExecWirePutU32(out, phased.consumer);
        RigExecWirePutU8(out, uint8_t(phased.consumerType));
        RigExecWirePutU32(out, phased.applied);
    }
    RigExecWirePutU32(out, uint32_t(computed.chainReads.size()));
    for (const RigExecWireChainRead &entry : computed.chainReads) {
        RigExecWirePutU32(out, entry.uid);
        _PutInput(out, entry.read);
    }
    RigExecWirePutU32(out, uint32_t(computed.registeredReads.size()));
    for (const RigExecWireRegisteredRead &entry : computed.registeredReads) {
        RigExecWirePutU8(out, uint8_t(entry.family));
        RigExecWirePutU32(out, entry.object);
        RigExecWirePutU32(out, entry.field);
        RigExecWirePutI32(out, entry.uid);
        RigExecWirePutI32(out, entry.avar);
        _PutInput(out, entry.read);
    }
    RigExecWirePutU32(out, uint32_t(computed.blendWeightReads.size()));
    for (const RigExecWireBlendWeightRead &entry : computed.blendWeightReads) {
        RigExecWirePutU32(out, entry.chain);
        RigExecWirePutU32(out, entry.revision);
        RigExecWirePutU8(out, entry.derived ? 1 : 0);
        RigExecWirePutU32(out, entry.channel);
        _PutInput(out, entry.read);
    }
    RigExecWirePutU32(out, uint32_t(computed.defaultWeightReads.size()));
    for (const RigExecWireDefaultWeightRead &entry :
         computed.defaultWeightReads) {
        RigExecWirePutU32(out, entry.chain);
        RigExecWirePutU32(out, entry.revision);
        _PutInput(out, entry.read);
    }
    RigExecWirePutU32(out, uint32_t(computed.pathScalarReads.size()));
    for (const RigExecWirePathScalarRead &entry : computed.pathScalarReads) {
        RigExecWirePutU32(out, entry.path);
        RigExecWirePutU8(out, entry.widen ? 1 : 0);
        RigExecWirePutU8(out, entry.headFallback ? 1 : 0);
        _PutInput(out, entry.read);
    }
    RigExecWirePutU32(out, uint32_t(computed.frames.size()));
    for (const RigExecWireComputedFrame &frame : computed.frames) {
        RigExecWirePutF64(out, frame.frame);
        for (uint32_t value : frame.values) {
            RigExecWirePutU32(out, value);
        }
        out->insert(out->end(), frame.hasValue.begin(), frame.hasValue.end());
    }
    return true;
}

bool
RigExecWireDecodeComputed(RigExecWireReader *reader,
                          RigExecWireComputed *computed, std::string *error)
{
    if (!reader || !computed) {
        return _Fail(error, "nothing to decode into");
    }
    *computed = RigExecWireComputed();
    uint32_t layout = 0;
    if (!reader->ReadU32(&layout) || layout != _ComputedLayout) {
        return _Fail(error, "unknown layout");
    }
    if (!reader->ReadF64(&computed->bakeTime)) {
        return _Fail(error, "truncated");
    }
    uint32_t count = 0;
    // 9 bytes: a tag and its bits.
    if (!_ReadCount(reader, 9, &count)) {
        return _Fail(error, "truncated value table");
    }
    computed->values.resize(count);
    for (v4::RigExecWireValue &value : computed->values) {
        if (!_ReadValue(reader, &value)) {
            return _Fail(error, "truncated or malformed value");
        }
    }
    if (!_ReadCount(reader, 4, &count)) {
        return _Fail(error, "truncated points pool");
    }
    computed->vec3fArrays.resize(count);
    for (v4::RigExecWireVec3fArray &array : computed->vec3fArrays) {
        uint32_t points = 0;
        if (!_ReadCount(reader, 12, &points)) {
            return _Fail(error, "truncated points array");
        }
        array.v.resize(points);
        for (RigExecWireVec3f &point : array.v) {
            if (!RigExecWireReadVec3f(reader, &point)) {
                return _Fail(error, "truncated points array");
            }
        }
    }
    // 18 bytes per slot.
    if (!_ReadCount(reader, 18, &count)) {
        return _Fail(error, "truncated input list");
    }
    computed->inputs.resize(count);
    for (v4::InputSlot &slot : computed->inputs) {
        if (!reader->ReadU32(&slot.name) || !reader->ReadU32(&slot.value) ||
            !reader->ReadI32(&slot.chain) || !reader->ReadI32(&slot.phased) ||
            !_ReadTag(reader, &slot.type) || !reader->ReadU8(&slot.flags)) {
            return _Fail(error, "truncated or malformed input slot");
        }
    }
    if (!reader->ReadU32(&computed->listedInputs)) {
        return _Fail(error, "truncated");
    }
    // A weight object's fixed fields alone are well over 64 bytes.
    if (!_ReadCount(reader, 64, &count)) {
        return _Fail(error, "truncated weight objects");
    }
    computed->weightObjects.resize(count);
    for (v4::RigExecWireWeightObject &object : computed->weightObjects) {
        if (!_ReadWeightObject(reader, &object)) {
            return _Fail(error, "truncated or malformed weight object");
        }
    }
    if (!_ReadI32s(reader, &computed->constraintWeightObjectIndex)) {
        return _Fail(error, "truncated constraint envelope indices");
    }
    // Target, value type and a revision count.
    if (!_ReadCount(reader, 9, &count)) {
        return _Fail(error, "truncated property chains");
    }
    computed->propertyChains.resize(count);
    for (v4::PropertyChain &chain : computed->propertyChains) {
        if (!_ReadPropertyChain(reader, &chain)) {
            return _Fail(error, "truncated or malformed property chain");
        }
    }
    // Chain, consumer, consumer type and applied count.
    if (!_ReadCount(reader, 13, &count)) {
        return _Fail(error, "truncated phased consumers");
    }
    computed->phasedConsumers.resize(count);
    for (v4::PhasedConsumer &phased : computed->phasedConsumers) {
        uint8_t consumerType = 0;
        if (!reader->ReadU32(&phased.chain) ||
            !reader->ReadU32(&phased.consumer) ||
            !reader->ReadU8(&consumerType) ||
            consumerType > v4::PropertyValueTypeLast ||
            !reader->ReadU32(&phased.applied)) {
            return _Fail(error, "truncated or malformed phased consumer");
        }
        phased.consumerType = v4::PropertyValueType(consumerType);
    }
    // A uid and a read.
    if (!_ReadCount(reader, 4 + _InputMinBytes, &count)) {
        return _Fail(error, "truncated chain reads");
    }
    computed->chainReads.resize(count);
    for (RigExecWireChainRead &entry : computed->chainReads) {
        if (!reader->ReadU32(&entry.uid) || !_ReadInput(reader, &entry.read)) {
            return _Fail(error, "truncated or malformed chain read");
        }
    }
    // Family, object, field, uid, avar and a read.
    if (!_ReadCount(reader, 17 + _InputMinBytes, &count)) {
        return _Fail(error, "truncated registered reads");
    }
    computed->registeredReads.resize(count);
    for (RigExecWireRegisteredRead &entry : computed->registeredReads) {
        uint8_t family = 0;
        if (!reader->ReadU8(&family) ||
            family > RigExecWireRegisteredFamilyLast ||
            !reader->ReadU32(&entry.object) ||
            !reader->ReadU32(&entry.field) || !reader->ReadI32(&entry.uid) ||
            !reader->ReadI32(&entry.avar) ||
            !_ReadInput(reader, &entry.read)) {
            return _Fail(error, "truncated or malformed registered read");
        }
        entry.family = RigExecWireRegisteredFamily(family);
    }
    // Chain, revision, derived flag, channel and a read.
    if (!_ReadCount(reader, 13 + _InputMinBytes, &count)) {
        return _Fail(error, "truncated blend weight reads");
    }
    computed->blendWeightReads.resize(count);
    for (RigExecWireBlendWeightRead &entry : computed->blendWeightReads) {
        uint8_t derived = 0;
        if (!reader->ReadU32(&entry.chain) ||
            !reader->ReadU32(&entry.revision) || !reader->ReadU8(&derived) ||
            derived > 1 || !reader->ReadU32(&entry.channel) ||
            !_ReadInput(reader, &entry.read)) {
            return _Fail(error, "truncated or malformed blend weight read");
        }
        entry.derived = derived != 0;
    }
    // Chain, revision and a read.
    if (!_ReadCount(reader, 8 + _InputMinBytes, &count)) {
        return _Fail(error, "truncated default weight reads");
    }
    computed->defaultWeightReads.resize(count);
    for (RigExecWireDefaultWeightRead &entry : computed->defaultWeightReads) {
        if (!reader->ReadU32(&entry.chain) ||
            !reader->ReadU32(&entry.revision) ||
            !_ReadInput(reader, &entry.read)) {
            return _Fail(error, "truncated or malformed default weight read");
        }
    }
    // Path, widen and head-fallback flags and a read.
    if (!_ReadCount(reader, 6 + _InputMinBytes, &count)) {
        return _Fail(error, "truncated path reads");
    }
    computed->pathScalarReads.resize(count);
    for (RigExecWirePathScalarRead &entry : computed->pathScalarReads) {
        uint8_t widen = 0;
        uint8_t headFallback = 0;
        if (!reader->ReadU32(&entry.path) || !reader->ReadU8(&widen) ||
            widen > 1 || !reader->ReadU8(&headFallback) ||
            headFallback > 1 || !_ReadInput(reader, &entry.read)) {
            return _Fail(error, "truncated or malformed path read");
        }
        entry.widen = widen != 0;
        entry.headFallback = headFallback != 0;
    }
    const size_t slots = computed->inputs.size();
    // 8 bytes of frame time plus 5 per slot.
    if (!_ReadCount(reader, 8 + 5 * slots, &count)) {
        return _Fail(error, "truncated frames");
    }
    computed->frames.resize(count);
    for (RigExecWireComputedFrame &frame : computed->frames) {
        if (!reader->ReadF64(&frame.frame)) {
            return _Fail(error, "truncated frame");
        }
        frame.values.resize(slots);
        for (uint32_t &value : frame.values) {
            if (!reader->ReadU32(&value)) {
                return _Fail(error, "truncated frame");
            }
        }
        if (!reader->ReadBytes(slots, &frame.hasValue)) {
            return _Fail(error, "truncated frame");
        }
    }
    if (!reader->Exhausted()) {
        return _Fail(error, "trailing bytes");
    }
    return _Check(*computed, error);
}

bool
RigExecWireApplyComputed(const RigExecWireComputed &computed,
                         const RigExecWireDomainGeometry &geometry,
                         const RigExecWireInputTable &inputs,
                         RigExecWireDomainPose *pose, std::string *error)
{
    if (!pose) {
        return _Fail(error, "no pose tables to apply to");
    }
    size_t stepBacked = 0;
    while (stepBacked < computed.weightObjects.size() &&
           !computed.weightObjects[stepBacked].envelopeOnly) {
        ++stepBacked;
    }
    if (stepBacked != geometry.weightObjects.size()) {
        return _Fail(error, "step-backed weight objects do not match the "
                            "geometry section");
    }
    for (size_t i = 0; i < stepBacked; ++i) {
        if (computed.weightObjects[i].path != geometry.weightObjects[i].path ||
            computed.weightObjects[i].type != geometry.weightObjects[i].type) {
            return _Fail(error, "weight object " + std::to_string(i) +
                                    " differs from the geometry section");
        }
    }
    if (computed.constraintWeightObjectIndex.size() !=
        pose->constraints.size()) {
        return _Fail(error, "one envelope index per constraint expected");
    }
    if (computed.frames.size() != inputs.frames.size()) {
        return _Fail(error, "one slot record per input-table frame expected");
    }
    if (pose->hasPropertyChains == computed.propertyChains.empty()) {
        return _Fail(error, "property chains do not match the pose tables");
    }
    for (size_t f = 0; f < computed.frames.size(); ++f) {
        if (!(computed.frames[f].frame == inputs.frames[f].frame)) {
            return _Fail(error, "frame " + std::to_string(f) +
                                    " is not the input table's");
        }
    }
    // Each chain-crossing read names the directory entry of the same input:
    // its tag (the old tags are the v4 numbers up to Vec3d), override number
    // and head path (both sections intern into one string table).
    for (const RigExecWireChainRead &entry : computed.chainReads) {
        const std::string where = "chain read of uid " +
                                  std::to_string(entry.uid);
        if (entry.uid >= inputs.directory.size()) {
            return _Fail(error, where + " names no input directory entry");
        }
        const RigExecWireInputDirectoryEntry &directory =
            inputs.directory[entry.uid];
        if (uint8_t(directory.tag) != uint8_t(entry.read.tag) ||
            directory.overrideIndex != entry.read.overrideIndex ||
            entry.read.walk.empty() ||
            directory.head != computed.inputs[entry.read.walk[0]].name) {
            return _Fail(error, where + " differs from its input directory "
                                        "entry");
        }
    }
    // Each registered read names a row of its table, and a uid names the
    // directory entry of the same input. The avar bindings have no table
    // here and the space switches travel in their own section; the
    // runtime checks those rows when it binds the reads.
    for (size_t k = 0; k < computed.registeredReads.size(); ++k) {
        const RigExecWireRegisteredRead &entry = computed.registeredReads[k];
        const std::string where = "registered read " + std::to_string(k);
        size_t rows = 0;
        switch (entry.family) {
        case RigExecWireRegisteredFamily::AvarBinding:
        case RigExecWireRegisteredFamily::AvarConstantBinding:
        case RigExecWireRegisteredFamily::SpaceSwitch:
            rows = size_t(entry.object) + 1;
            break;
        case RigExecWireRegisteredFamily::Ladder:
            rows = pose->ladders.size();
            break;
        case RigExecWireRegisteredFamily::Interpolator:
            rows = pose->poseInterpolators.size();
            break;
        case RigExecWireRegisteredFamily::Solver:
            rows = pose->solvers.size();
            break;
        case RigExecWireRegisteredFamily::Constraint:
            rows = pose->constraints.size();
            break;
        case RigExecWireRegisteredFamily::WeightObject:
            rows = geometry.weightObjects.size();
            break;
        }
        if (entry.object >= rows) {
            return _Fail(error, where + " names no table row");
        }
        if (entry.uid < 0) {
            continue;
        }
        if (size_t(entry.uid) >= inputs.directory.size()) {
            return _Fail(error, where + " names no input directory entry");
        }
        const RigExecWireInputDirectoryEntry &directory =
            inputs.directory[size_t(entry.uid)];
        const uint32_t head =
            entry.read.walk.empty()
                ? 0
                : computed.inputs[entry.read.walk[0]].name;
        if (uint8_t(directory.tag) != uint8_t(entry.read.tag) ||
            directory.overrideIndex != entry.read.overrideIndex ||
            directory.head != head) {
            return _Fail(error, where + " differs from its input directory "
                                        "entry");
        }
    }
    // Each geometry read names a revision the geometry section holds, and a
    // blend weight one of its channels.
    const auto revisionOf =
        [&](uint32_t chain, uint32_t revision,
            bool derived) -> const RigExecWireRevision * {
        if (chain >= geometry.chains.size()) {
            return nullptr;
        }
        const RigExecWireChain &wire = geometry.chains[chain];
        if (derived) {
            return revision < wire.derived.size()
                       ? &wire.derived[revision].revision
                       : nullptr;
        }
        return revision < wire.revisions.size() ? &wire.revisions[revision]
                                                : nullptr;
    };
    for (size_t k = 0; k < computed.blendWeightReads.size(); ++k) {
        const RigExecWireBlendWeightRead &entry = computed.blendWeightReads[k];
        const RigExecWireRevision *revision =
            revisionOf(entry.chain, entry.revision, entry.derived);
        if (!revision || entry.channel >= revision->blendChannels.size()) {
            return _Fail(error, "blend weight read " + std::to_string(k) +
                                    " names no blend channel");
        }
    }
    for (size_t k = 0; k < computed.defaultWeightReads.size(); ++k) {
        const RigExecWireDefaultWeightRead &entry =
            computed.defaultWeightReads[k];
        if (!revisionOf(entry.chain, entry.revision, false)) {
            return _Fail(error, "default weight read " + std::to_string(k) +
                                    " names no revision");
        }
    }
    for (size_t k = 0; k < pose->constraints.size(); ++k) {
        pose->constraints[k].weightObjectIndex =
            computed.constraintWeightObjectIndex[k];
    }
    return true;
}

}  // namespace rigExec
