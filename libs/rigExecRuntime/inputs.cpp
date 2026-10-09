#include "poseInternal.h"
#include "spaces.h"
// rigExecRuntime input slot model and read evaluation: a line port of
// RigExecResolvedInputs::GetAttribute (moverGraph.h), RigExecBakedRead
// (bakedProgramImpl.h) and _PinnedRead (rigEvaluatorProperties.cpp) over
// the slot values, plus the input API's writes to them. GetAttribute's
// overlay holds the generation's property-chain results
// (RrStore::propertyResults), keyed by the same attribute path ids as the
// slot names.
#include "rigExecRuntime/inputs.h"
#include "rigExecRuntime/store.h"
#include "rigExecRuntime/spaces.h"

#include <algorithm>
#include <set>
#include "stageArrayInputs.h"
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <utility>

namespace rigExec {

namespace {

bool
_RrFail(std::string *error, const std::string &why)
{
    if (error) {
        *error = why;
    }
    return false;
}

double
_RrWireDouble(const RrWireValue &value)
{
    double d = 0.0;
    std::memcpy(&d, &value.bits, sizeof(d));
    return d;
}

RrWireValue
_RrFloatValue(float f)
{
    RrWireValue value;
    value.tag = RigExecWireInputTag::Float;
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    value.bits = bits;
    return value;
}

// The overlay's value at \p path: the chain result published there, when
// it is exactly \p tag.
bool
_RrPropertyWire(const RrPropertyValue &held, RigExecWireInputTag tag,
                RrWireValue *out)
{
    using HeldTag = RrPropertyValue::Tag;
    RrWireValue value;
    value.tag = tag;
    switch (tag) {
    case RigExecWireInputTag::Float:
        if (held.tag != HeldTag::Float) {
            return false;
        }
        value = _RrFloatValue(held.f32);
        break;
    case RigExecWireInputTag::Double:
        if (held.tag != HeldTag::Double) {
            return false;
        }
        std::memcpy(&value.bits, &held.f64, sizeof(held.f64));
        break;
    case RigExecWireInputTag::Matrix4d:
        if (held.tag != HeldTag::Matrix4d) {
            return false;
        }
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                value.matrix[r * 4 + c] = held.matrix[r][c];
            }
        }
        break;
    case RigExecWireInputTag::Vec3f:
        if (held.tag != HeldTag::Vec3f) {
            return false;
        }
        value.vec3f = RigExecWireVec3f{held.vec[0], held.vec[1], held.vec[2]};
        break;
    default:
        return false;
    }
    *out = value;
    return true;
}

bool
_RrSlotOverlay(const RrProgram &program, uint32_t slot, RigExecWireInputTag tag,
               RrWireValue *out)
{
    const int32_t version = program.store.propertyPublishedVersions[slot];
    if (version >= 0 && program.store.propertyVersionValid[size_t(version)] &&
        _RrPropertyWire(program.store.propertyVersions[size_t(version)], tag, out)) return true;
    return false;
}

bool
_RrOverlay(const RrProgram &program, uint32_t path, RigExecWireInputTag tag,
           RrWireValue *out)
{
    const auto found = program.store.propertyPathSlots.find(path);
    return found != program.store.propertyPathSlots.end() &&
           _RrSlotOverlay(program, found->second, tag, out);
}

bool
_RrCandidateOverlay(const RrProgram &program,
                    const RigExecWirePropertyInputCandidate &candidate,
                    RigExecWireInputTag tag, RrWireValue *out, bool publishedOnly = false,
                    const RrOracleReadContext *context = nullptr)
{
    const auto &store = program.store;
    const size_t slot = candidate.slot;
    if (context) {
        if (candidate.poseWeight >= 0 && tag == RigExecWireInputTag::Float) {
            *out = _RrFloatValue(store.poseWeights[size_t(candidate.poseWeight)]);
            return true;
        }
        const auto found = context->versionsBySlot.find(uint32_t(slot));
        if (found == context->versionsBySlot.end()) return false;
        for (auto it = found->second.rbegin(); it != found->second.rend(); ++it) {
            if (store.propertyVersionValid[size_t(*it)])
                return _RrPropertyWire(store.propertyVersions[size_t(*it)], tag, out);
        }
        return false;
    }
    if (publishedOnly) {
        const int32_t version = store.propertyPublishedVersions[slot];
        if (version >= 0 && store.propertyVersionValid[size_t(version)] &&
            _RrPropertyWire(store.propertyVersions[size_t(version)], tag, out)) return true;
        return false;
    }
    const auto kind = fb::PropertyCandidateKind(candidate.kind);
    if (kind == fb::PropertyCandidateKind::ChainFinal) {
        const size_t v = size_t(candidate.version);
        if (store.propertyVersionValid[v])
            return _RrPropertyWire(store.propertyVersions[v], tag, out);
        return false;
    }
    if (kind == fb::PropertyCandidateKind::PhasedRecord) {
        const size_t v = size_t(candidate.version);
        if (store.propertyVersionValid[v])
            return _RrPropertyWire(store.propertyVersions[v], tag, out);
    }
    return false;
}

bool
_RrCandidates(const RrProgram &program, const RigExecWireInput &input,
              RrWireValue *out, bool publishedOnly = false,
              const RrOracleReadContext *context = nullptr, bool defaultTime = false)
{
    const auto &state = program.inputState;
    const auto present=[&](size_t slot) { return defaultTime && !state.slotAuthored[slot]
        ? state.slotDefaultHasValue[slot]!=0 : state.slotHasValue[slot]!=0; };
    const auto value=[&](size_t slot)->const RrWireValue & { return defaultTime && !state.slotAuthored[slot]
        ? state.values[state.file->inputs[slot].value()] : state.slotCurrent[slot]; };
    const auto overlay = [&](const auto &list, RigExecWireInputTag tag) {
        for (const auto &candidate : list)
            if (_RrCandidateOverlay(program, candidate, tag, out, publishedOnly, context)) return true;
        return false;
    };
    const auto raw = [&](const auto &list, RigExecWireInputTag tag) {
        for (auto it = list.rbegin(); it != list.rend(); ++it) {
            if (it->raw && present(it->slot) &&
                state.file->inputs[it->slot].type() == tag) {
                *out = value(it->slot); return true;
            }
        }
        return false;
    };
    if (input.mode == RigExecWireReadMode::Pinned &&
        !input.propertyCandidates.empty()) {
        const auto &candidate = input.propertyCandidates.front();
        if (_RrCandidateOverlay(program, candidate, input.tag, out, publishedOnly, context)) return true;
        if (candidate.raw && present(candidate.slot) &&
            state.file->inputs[candidate.slot].type() == input.tag) {
            *out = value(candidate.slot); return true;
        }
        return false;
    }
    if (overlay(input.propertyCandidates, input.tag)) return true;
    // All overlay hops precede any typed cross-domain source, matching
    // the native retained walk's two passes.
    for(const auto &candidate:input.propertyCandidates) {
        if(candidate.crossDomain>=0 && RrReadCrossDomain(&program,candidate.crossDomain,out) &&
           out->tag==input.tag) return true;
    }
    if (!input.doubleCandidates.empty()) {
        if (input.tag == RigExecWireInputTag::Float &&
            (overlay(input.doubleCandidates, RigExecWireInputTag::Double) ||
             raw(input.doubleCandidates, RigExecWireInputTag::Double))) {
            *out = _RrFloatValue(float(_RrWireDouble(*out))); return true;
        }
        // A Double tail prevents raw Float upstream traversal, but does not
        // discard an explicitly declared consumer fallback.
    } else if (raw(input.propertyCandidates, input.tag)) return true;
    if (input.rawFallbackSlot >= 0) {
        const size_t slot = size_t(input.rawFallbackSlot);
        if (present(slot) && state.file->inputs[slot].type() == input.tag) {
            *out = value(slot); return true;
        }
    }
    return false;
}

// Slot \p slot's value this run: the one place a read meets the slots.
const RrWireValue &
_RrSlotValue(const RrInputState &state, uint32_t slot)
{
    return state.slotCurrent[slot];
}

// What answered a walk: a slot's value, an overlay hit, or a double hop
// cast to float.
struct _RrWalkAnswer {
    RrWireValue value;
};

// GetAttribute<T> from walk[from], hop for hop. At each hop the overlay
// answers first; a float read that meets a double hop reads a double walk
// from that hop and casts it (_CoerceFromDouble), and a failed double walk
// fails the read. GetAttribute<float> tests its own head for a double
// before consulting the overlay there. Otherwise the most upstream hop
// whose typed read succeeds answers, which is the walk's reverse scan.
bool
_RrLongWay(const RrProgram &program, const std::vector<uint32_t> &walk,
           size_t from, RigExecWireInputTag tag, _RrWalkAnswer *answer,
           bool defaultTime=false)
{
    const RrInputState &state = program.inputState;
    const std::vector<RigExecWireInputSlot> &slots = state.file->inputs;
    const auto narrowFrom = [&](size_t k) {
        _RrWalkAnswer wide;
        if (!_RrLongWay(program, walk, k, RigExecWireInputTag::Double,
                        &wide,defaultTime)) {
            return false;
        }
        const double d = _RrWireDouble(wide.value);
        answer->value = _RrFloatValue(static_cast<float>(d));
        return true;
    };
    if (tag == RigExecWireInputTag::Float && from < walk.size() &&
        slots[walk[from]].type() == RigExecWireInputTag::Double) {
        return narrowFrom(from);
    }
    for (size_t k = from; k < walk.size(); ++k) {
        const RigExecWireInputSlot &slot = slots[walk[k]];
        if (_RrSlotOverlay(program, walk[k], tag, &answer->value)) {
            return true;
        }
        if (tag == RigExecWireInputTag::Float &&
            slot.type() == RigExecWireInputTag::Double) {
            return narrowFrom(k);
        }
    }
    for (size_t k = walk.size(); k-- > from;) {
        const uint32_t slot = walk[k];
        const bool authored=state.slotAuthored[slot]!=0;
        const bool have=defaultTime && !authored ? state.slotDefaultHasValue[slot]!=0 : state.slotHasValue[slot]!=0;
        if (have && slots[slot].type() == tag) {
            answer->value = defaultTime && !authored ? state.values[slots[slot].value()] : _RrSlotValue(state, slot);
            return true;
        }
    }
    return false;
}

RrWireValue
_RrWalkValue(const RrProgram &program, const RigExecWireInput &input)
{
    const RrInputState &state = program.inputState;
    _RrWalkAnswer answer;
    const bool bound = !input.propertyCandidates.empty() || !input.doubleCandidates.empty();
    if (!(bound ? _RrCandidates(program, input, &answer.value)
                : _RrLongWay(program, input.walk, 0, input.tag, &answer))) {
        return state.values[input.constant];
    }
    return answer.value;
}

// The walk's first slot read raw: its own typed value when that read
// succeeded and has the read's type, else the read's constant.
RrWireValue
_RrHeadValue(const RrInputState &state, const RigExecWireInput &input)
{
    const RrWireValue &fallback = state.values[input.constant];
    if (input.walk.empty()) {
        return fallback;
    }
    const uint32_t slot = input.walk[0];
    if (state.slotHasValue[slot] &&
        state.file->inputs[slot].type() == input.tag) {
        return _RrSlotValue(state, slot);
    }
    return fallback;
}

// Bitwise equality of two values of one tag, on the member the tag names.
bool
_RrSameValueBits(const RrWireValue &a, const RrWireValue &b)
{
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case RigExecWireInputTag::Matrix4d:
        return std::memcmp(a.matrix.data(), b.matrix.data(),
                           sizeof(a.matrix)) == 0;
    case RigExecWireInputTag::Vec3d:
        return std::memcmp(a.vec3d.data(), b.vec3d.data(),
                           sizeof(a.vec3d)) == 0;
    case RigExecWireInputTag::Vec3i:
        return a.vec3i==b.vec3i;
    case RigExecWireInputTag::Vec3f:
        return std::memcmp(a.vec3f.data(), b.vec3f.data(),
                           sizeof(a.vec3f)) == 0;
    default:
        return a.bits == b.bits;
    }
}

// Whether slot \p slot holds other than its bake-time default: another
// HasValue, or other bits.
bool
_RrSlotDiffersFromDefault(const RrInputState &state, uint32_t slot)
{
    return state.slotHasValue[slot] != state.slotDefaultHasValue[slot] ||
           !_RrSameValueBits(state.slotCurrent[slot],
                             state.values[state.file->inputs[slot].value()]);
}

// The bytes of one element of array tag \p tag.
size_t
_RrElementSize(RigExecWireInputTag tag)
{
    switch (tag) {
    case RigExecWireInputTag::IntArray:
        return sizeof(int32_t);
    case RigExecWireInputTag::FloatArray:
        return sizeof(float);
    case RigExecWireInputTag::DoubleArray:
        return sizeof(double);
    case RigExecWireInputTag::Vec2fArray:
        return sizeof(RrVec2f);
    case RigExecWireInputTag::Vec3fArray:
        return sizeof(RrVec3f);
    case RigExecWireInputTag::Vec3dArray: return sizeof(RigExecWireVec3d);
    case RigExecWireInputTag::Matrix4dArray: return sizeof(RigExecWireMatrix4d);
    case RigExecWireInputTag::TokenArray: return sizeof(uint32_t);
    case RigExecWireInputTag::BoolArray: return sizeof(uint8_t);
    default:
        return 0;
    }
}

// The elements of \p vector, a const std::vector of \p tag's element type.
struct _RrElements {
    const void *data = nullptr;
    size_t count = 0;
};

template <class T>
_RrElements
_RrElementsOf(const void *vector)
{
    const std::vector<T> &v = *static_cast<const std::vector<T> *>(vector);
    return {v.data(), v.size()};
}

_RrElements
_RrVectorElements(RigExecWireInputTag tag, const void *vector)
{
    switch (tag) {
    case RigExecWireInputTag::IntArray:
        return _RrElementsOf<int32_t>(vector);
    case RigExecWireInputTag::FloatArray:
        return _RrElementsOf<float>(vector);
    case RigExecWireInputTag::DoubleArray:
        return _RrElementsOf<double>(vector);
    case RigExecWireInputTag::Vec2fArray:
        return _RrElementsOf<RrVec2f>(vector);
    case RigExecWireInputTag::Vec3fArray:
        return _RrElementsOf<RrVec3f>(vector);
    case RigExecWireInputTag::Vec3dArray: return _RrElementsOf<RigExecWireVec3d>(vector);
    case RigExecWireInputTag::Matrix4dArray: return _RrElementsOf<RigExecWireMatrix4d>(vector);
    case RigExecWireInputTag::TokenArray: return _RrElementsOf<uint32_t>(vector);
    case RigExecWireInputTag::BoolArray: return _RrElementsOf<uint8_t>(vector);
    default:
        return {};
    }
}

// \p buffer's vector of \p tag's element type.
const void *
_RrBufferVector(RigExecWireInputTag tag, const RrArrayBuffer &buffer)
{
    switch (tag) {
    case RigExecWireInputTag::IntArray:
        return &buffer.ints;
    case RigExecWireInputTag::FloatArray:
        return &buffer.floats;
    case RigExecWireInputTag::DoubleArray:
        return &buffer.doubles;
    case RigExecWireInputTag::Vec2fArray:
        return &buffer.vec2s;
    case RigExecWireInputTag::Vec3fArray:
        return &buffer.vec3s;
    case RigExecWireInputTag::Vec3dArray: return &buffer.vec3ds;
    case RigExecWireInputTag::Matrix4dArray: return &buffer.matrices;
    case RigExecWireInputTag::TokenArray: return &buffer.tokens;
    case RigExecWireInputTag::BoolArray: return &buffer.bools;
    default:
        return nullptr;
    }
}

// \p out becomes a copy of the \p count elements at \p data, which may point
// into \p out itself.
template <class T>
void
_RrCopyInto(std::vector<T> *out, const void *data, size_t count)
{
    if (!count) { out->clear(); return; }
    const T *first = static_cast<const T *>(data);
    std::vector<T>(first, first + count).swap(*out);
}

// \p buffer's vector of \p tag's element type becomes a copy of the
// \p count elements at \p data.
void
_RrAssign(RigExecWireInputTag tag, RrArrayBuffer *buffer, const void *data,
          size_t count)
{
    switch (tag) {
    case RigExecWireInputTag::IntArray:
        _RrCopyInto(&buffer->ints, data, count);
        break;
    case RigExecWireInputTag::FloatArray:
        _RrCopyInto(&buffer->floats, data, count);
        break;
    case RigExecWireInputTag::DoubleArray:
        _RrCopyInto(&buffer->doubles, data, count);
        break;
    case RigExecWireInputTag::Vec2fArray:
        _RrCopyInto(&buffer->vec2s, data, count);
        break;
    case RigExecWireInputTag::Vec3fArray:
        _RrCopyInto(&buffer->vec3s, data, count);
        break;
    case RigExecWireInputTag::Vec3dArray: _RrCopyInto(&buffer->vec3ds,data,count); break;
    case RigExecWireInputTag::Matrix4dArray: _RrCopyInto(&buffer->matrices,data,count); break;
    case RigExecWireInputTag::TokenArray: _RrCopyInto(&buffer->tokens,data,count); break;
    case RigExecWireInputTag::BoolArray: _RrCopyInto(&buffer->bools,data,count); break;
    default:
        break;
    }
}

// Whether \p vector holds exactly \p count elements equal, byte for byte,
// to the ones at \p data.
bool
_RrSameElements(RigExecWireInputTag tag, const void *vector, const void *data,
                size_t count)
{
    const _RrElements held = _RrVectorElements(tag, vector);
    return held.count == count &&
           (count == 0 ||
            std::memcmp(held.data, data, count * _RrElementSize(tag)) == 0);
}

// Array slot \p a's elements this run: its held set, else its default.
const void *
_RrArrayView(const RrArraySlot &a)
{
    return a.holdsSet ? _RrBufferVector(a.tag, a.held) : a.defaultVector;
}

// Pool entry \p id of \p tag's pool, as a const std::vector of its element
// type: the file's own for int, float and double, the one Open converted
// for float2 and float3.
const void *
_RrPoolVector(const RrInputState &state, RigExecWireInputTag tag, uint32_t id)
{
    const RigExecWireFile &file = *state.file;
    switch (tag) {
    case RigExecWireInputTag::IntArray:
        return id < file.intArrays.size() ? &file.intArrays[id].v : nullptr;
    case RigExecWireInputTag::FloatArray:
        return id < file.floatArrays.size() ? &file.floatArrays[id].v
                                            : nullptr;
    case RigExecWireInputTag::DoubleArray:
        return id < file.doubleArrays.size() ? &file.doubleArrays[id].v
                                             : nullptr;
    case RigExecWireInputTag::Vec3dArray:
        return id<file.vec3dArrays.size()?&file.vec3dArrays[id].v:nullptr;
    case RigExecWireInputTag::Matrix4dArray:
        return id<file.matrix4dArrays.size()?&file.matrix4dArrays[id].v:nullptr;
    case RigExecWireInputTag::TokenArray: {
        const auto found=state.tokenArrayPool.find(id);
        return found==state.tokenArrayPool.end()?nullptr:&found->second;
    }
    case RigExecWireInputTag::BoolArray:
        return id<file.boolArrays.size()?&file.boolArrays[id].v:nullptr;
    case RigExecWireInputTag::Vec2fArray: {
        const auto found = state.vec2Pool.find(id);
        return found == state.vec2Pool.end() ? nullptr : &found->second;
    }
    case RigExecWireInputTag::Vec3fArray: {
        const auto found = state.vec3Pool.find(id);
        return found == state.vec3Pool.end() ? nullptr : &found->second;
    }
    default:
        return nullptr;
    }
}

// A path value's array of \p tag, or null when it holds another.
const void *
_RrPathValueArray(const RrPathValue &value, RigExecWireInputTag tag)
{
    using Tag = RrPathValue::Tag;
    switch (tag) {
    case RigExecWireInputTag::IntArray:
        return value.tag == Tag::IntArray ? &value.ints : nullptr;
    case RigExecWireInputTag::FloatArray:
        return value.tag == Tag::FloatArray ? &value.floats : nullptr;
    case RigExecWireInputTag::DoubleArray:
        return value.tag == Tag::DoubleArray ? &value.doubles : nullptr;
    case RigExecWireInputTag::Vec2fArray:
        return value.tag == Tag::Vec2fArray ? &value.vec2s : nullptr;
    case RigExecWireInputTag::Vec3fArray:
        return value.tag == Tag::Vec3fArray ? &value.vec3s : nullptr;
    default:
        return nullptr;
    }
}

// Every read a weight object carries, in field order (RrWeightField).
template <class Visit>
void
_RrForEachWeightRead(const RigExecWireWeightObject &object,
                     const Visit &visit)
{
    for (const std::unique_ptr<RigExecWireInput> *input :
         {&object.defaultWeight, &object.driver, &object.scale, &object.bias,
          &object.strength, &object.invert, &object.falloffMin,
          &object.falloffMax, &object.scaleXPos, &object.scaleYPos,
          &object.scaleZPos, &object.scaleXNeg, &object.scaleYNeg,
          &object.scaleZNeg, &object.scaleX, &object.scaleY, &object.scaleZ,
          &object.extentU, &object.extentV}) {
        visit(input->get());
    }
}

}  // namespace

float
RrWireValueFloat(const RrWireValue &value)
{
    const uint32_t bits = uint32_t(value.bits);
    float f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

RrWireValue
RrWireValueOf(const fb::RigExecWireValue &value)
{
    RrWireValue out;
    out.tag = value.tag;
    out.bits = value.bits;
    if (value.matrix) {
        out.matrix = *value.matrix;
    }
    if (value.vec3d) {
        out.vec3d = *value.vec3d;
    }
    if(value.vec3i) out.vec3i=*value.vec3i;
    if (value.vec3f) {
        out.vec3f = *value.vec3f;
    }
    return out;
}

bool
RrInputsOpen(RrProgram *program, const RigExecWireFile *file,
             std::string *error)
{
    if (!program || !program->poses || !program->geometry || !file) {
        return _RrFail(error, "the input slots need the pose and geometry "
                              "tables");
    }
    RrInputState &state = program->inputState;
    state = RrInputState();
    state.file = file;
    state.stageArraySlots = RigExecStageArraySlots(*file);
    state.stageTokenSlots = RigExecStageTokenSlots(*file);
    state.values.reserve(file->values.size());
    for (const fb::RigExecWireValue &value : file->values) {
        state.values.push_back(RrWireValueOf(value));
    }
    state.extraTokenBase = uint32_t(file->paths.size());
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    const std::vector<RigExecWireWeightObject> &objects =
        geometry.weightObjects;
    // The constraint step's envelope arm (a weight object and no points
    // target) resolves through the oracle, and nothing else does.
    for (const RigExecWireConstraint &c : poses.constraints) {
        const bool envelope = c.weightObject != 0 && c.pointsTarget == 0;
        if (envelope && c.weightObjectIndex < 0) {
            return _RrFail(error, "constraint " +
                                      program->TextOrEmpty(c.path) +
                                      " resolves an envelope the file does "
                                      "not carry");
        }
        if (!envelope && c.weightObjectIndex >= 0) {
            return _RrFail(error, "the file gives constraint " +
                                      program->TextOrEmpty(c.path) +
                                      " an envelope it does not resolve");
        }
        // The entry must be the constraint's own weight object.
        if (envelope &&
            (size_t(c.weightObjectIndex) >= objects.size() ||
             objects[size_t(c.weightObjectIndex)].path != c.weightObject)) {
            const std::string other =
                size_t(c.weightObjectIndex) < objects.size()
                    ? program->TextOrEmpty(
                          objects[size_t(c.weightObjectIndex)].path)
                    : std::string("no entry");
            return _RrFail(error, "the file gives constraint " +
                                      program->TextOrEmpty(c.path) +
                                      " the envelope of " + other +
                                      ", not of its weight object " +
                                      program->TextOrEmpty(c.weightObject));
        }
    }
    for (const RigExecWireChain &chain : geometry.chains) {
        for (const RigExecWireRevision &revision : chain.revisions) {
            if (!revision.weightCurrentPhase || revision.weightObject < 0) {
                continue;
            }
            if (size_t(revision.weightObject) >= objects.size()) {
                return _RrFail(error,
                               program->TextOrEmpty(revision.moverPath) +
                                   " binds a weight object the file does "
                                   "not carry");
            }
        }
    }
    // The oracle reads every scalar as a float (_ResolvedRead<float>).
    for (const RigExecWireWeightObject &object : objects) {
        bool floats = true;
        _RrForEachWeightRead(object, [&](const RigExecWireInput *input) {
            floats = floats && input &&
                     input->tag == RigExecWireInputTag::Float;
        });
        if (!floats) {
            return _RrFail(error, "weight object " +
                                      program->TextOrEmpty(object.path) +
                                      " carries a read that is not a float");
        }
    }
    const size_t slots = file->inputs.size();
    state.slotCurrent.resize(slots);
    state.slotHasValue.resize(slots);
    state.slotBlocked.resize(slots);
    state.slotProviderSource.assign(slots,0);
    if(file->providerProgram) for(const auto &leaf:file->providerProgram->sampled)
        if(leaf.inputSlot>=0) state.slotProviderSource[size_t(leaf.inputSlot)]=1;
    // Declared external AtTime numeric sources can live outside the public
    // input prefix. Keep their stage transport private and type-exact.
    const auto externalSource = [&](uint32_t slot) {
        const auto &input = file->inputs[slot];
        if (!(input.flags() & uint8_t(RigExecWireInputSlotFlags::Animated)) ||
            RigExecFormatIsArrayTag(input.type()) ||
            input.type() == RigExecWireInputTag::Token) return;
        state.slotProviderSource[slot] |= 4;
    };
    const auto externalWalk = [&](const RigExecWireInput &read) {
        for (uint32_t slot : read.walk) externalSource(slot);
        if (read.rawFallbackSlot >= 0) externalSource(uint32_t(read.rawFallbackSlot));
    };
    for (const auto &external : file->externalMovers)
        for (const auto &input : external.declaredInputs) {
            if (input.time != fb::ExternalInputTime::AtTime ||
                input.flavour == fb::ExternalInputFlavour::Present) continue;
            if (input.flavour == fb::ExternalInputFlavour::Raw) {
                if (!input.read->walk.empty()) externalSource(input.read->walk.front());
            } else {
                externalWalk(*input.read);
                if (input.bodyWalk) externalWalk(*input.bodyWalk);
            }
        }
    state.slotAuthored.assign(slots, 0);
    state.slotRanAuthored.assign(slots, 0);
    state.slotDefaultHasValue.resize(slots);
    for (size_t s = 0; s < slots; ++s) {
        const RigExecWireInputSlot &slot = file->inputs[s];
        state.slotCurrent[s] = state.values[slot.value()];
        state.slotBlocked[s]=(slot.flags() & uint8_t(RigExecWireInputSlotFlags::SourceBlocked))!=0;
        state.slotDefaultHasValue[s] =
            (slot.flags() & uint8_t(RigExecWireInputSlotFlags::HasValue)) !=
                    0
                ? 1
                : 0;
        state.slotHasValue[s] = state.slotDefaultHasValue[s];
    }
    state.slotRan = state.slotCurrent;
    state.slotRanHasValue = state.slotHasValue;
    state.touchedFlag.assign(slots, 0);
    state.writtenFlag.assign(slots, 0);
    state.written.clear();
    state.slotChangedSinceRun.assign(slots, 0);
    // The float2 and float3 pool entries an array value names, converted
    // once; the other pools are read in place.
    for (const fb::RigExecWireValue &value : file->values) {
        if (value.arraySource != RigExecWireArraySource::Pool) {
            continue;
        }
        const uint32_t id = value.array;
        if (value.tag == RigExecWireInputTag::Vec2fArray &&
            !state.vec2Pool.count(id)) {
            std::vector<RrVec2f> &out = state.vec2Pool[id];
            for (const RigExecWireVec2f &p : file->vec2fArrays[id].v) {
                out.push_back(RrVec2f(p[0], p[1]));
            }
        }
        if (value.tag == RigExecWireInputTag::Vec3fArray &&
            !state.vec3Pool.count(id)) {
            std::vector<RrVec3f> &out = state.vec3Pool[id];
            for (const RigExecWireVec3f &p : file->vec3fArrays[id].v) {
                out.push_back(RrVec3f(p[0], p[1], p[2]));
            }
        }
    }
    // The array slots, each on its default: a pool entry here, a stored
    // layout's expansion once the geometry family built it.
    state.extraTokenBase=uint32_t(file->paths.size());
    for(size_t pool=0;pool<file->tokenArrays.size();++pool) {
        auto &ids=state.tokenArrayPool[uint32_t(pool)];
        for(const auto &text:file->tokenArrays[pool].v) {
            auto found=state.tokenIds.find(text);
            if(found==state.tokenIds.end()) {
                const uint32_t id=state.extraTokenBase+uint32_t(state.extraTokens.size());
                state.extraTokens.push_back(text); found=state.tokenIds.emplace(text,id).first;
            }
            ids.push_back(found->second);
        }
    }
    state.arrayOf.assign(slots, -1);
    for (size_t s = 0; s < slots; ++s) {
        const RigExecWireInputSlot &slot = file->inputs[s];
        if (!RigExecFormatIsArrayTag(slot.type())) {
            continue;
        }
        RrArraySlot entry;
        entry.tag = slot.type();
        const fb::RigExecWireValue &value = file->values[slot.value()];
        if (value.arraySource == RigExecWireArraySource::Pool) {
            entry.defaultVector = _RrPoolVector(state, entry.tag, value.array);
        }
        state.arrayOf[s] = int32_t(state.arrays.size());
        state.arrays.push_back(std::move(entry));
    }
    // The token text the file holds, for SetInputToken: the empty token,
    // which the file writes as id 0, then each Token node's text, the
    // first node of a text winning.
    state.tokenIds.emplace(std::string(), 0u);
    for (size_t id = 0; id < file->paths.size(); ++id) {
        if (file->paths[id].kind() == RigExecWirePathKind::Token &&
            id < program->nodeText.size()) {
            state.tokenIds.emplace(program->nodeText[id], uint32_t(id));
        }
    }
    return true;
}

namespace {
void _RrEffectiveWireMemo(const RrProgram *program,const RrWireValue &v,std::string *key)
{
    const auto append=[&](const auto &x) { key->append(reinterpret_cast<const char *>(&x),sizeof(x)); };
    append(uint8_t(v.tag));
    switch(v.tag) {
    case RigExecWireInputTag::Matrix4d: for(double x:v.matrix) append(x); break;
    case RigExecWireInputTag::Vec3d: for(double x:v.vec3d) append(x); break;
    case RigExecWireInputTag::Vec3f: for(float x:v.vec3f) append(x); break;
    case RigExecWireInputTag::Vec3i: for(int32_t x:v.vec3i) append(x); break;
    case RigExecWireInputTag::Token: {
        const auto &text=program->TextOrEmpty(uint32_t(v.bits)); append(uint64_t(text.size())); key->append(text); break;
    }
    default: append(v.bits); break;
    }
}
void _RrEffectiveArrayMemo(const RrProgram *program,const void *array,RigExecWireInputTag tag,std::string *key)
{
    const auto append=[&](const auto &x) { key->append(reinterpret_cast<const char *>(&x),sizeof(x)); };
    append(uint8_t(tag)); append(bool(array)); if(!array) return;
    const auto bytes=[&](const auto *v) { append(uint64_t(v->size()));
        if(!v->empty()) key->append(reinterpret_cast<const char *>(v->data()),v->size()*sizeof((*v)[0])); };
    switch(tag) {
    case RigExecWireInputTag::IntArray: bytes(static_cast<const std::vector<int32_t> *>(array)); break;
    case RigExecWireInputTag::FloatArray: bytes(static_cast<const std::vector<float> *>(array)); break;
    case RigExecWireInputTag::DoubleArray: bytes(static_cast<const std::vector<double> *>(array)); break;
    case RigExecWireInputTag::Vec2fArray: bytes(static_cast<const std::vector<RrVec2f> *>(array)); break;
    case RigExecWireInputTag::Vec3fArray: bytes(static_cast<const std::vector<RrVec3f> *>(array)); break;
    case RigExecWireInputTag::Vec3dArray: bytes(static_cast<const std::vector<RigExecWireVec3d> *>(array)); break;
    case RigExecWireInputTag::Matrix4dArray: bytes(static_cast<const std::vector<RigExecWireMatrix4d> *>(array)); break;
    case RigExecWireInputTag::BoolArray: bytes(static_cast<const std::vector<uint8_t> *>(array)); break;
    case RigExecWireInputTag::TokenArray: {
        const auto &tokens=*static_cast<const std::vector<uint32_t> *>(array); append(uint64_t(tokens.size()));
        for(uint32_t token:tokens) { const auto &text=program->TextOrEmpty(token); append(uint64_t(text.size())); key->append(text); } break;
    }
    default: break;
    }
}
void _RrCoveredRead(const RigExecWireInput &read,std::vector<uint32_t> *covered)
{
    for(const auto *segment:{&read.propertyCandidates,&read.doubleCandidates})
        for(const auto &hop:*segment) if(hop.version>=0) covered->push_back(uint32_t(hop.version));
}
}

void RrEffectiveReadMemo(const RrProgram *program,const RigExecWireInput &read,std::string *key)
{
    if(RigExecFormatIsArrayTag(read.tag))
        _RrEffectiveArrayMemo(program,RrArrayReadValue(program,read,read.tag),read.tag,key);
    else _RrEffectiveWireMemo(program,RrReadInput(program,read),key);
}

bool RrEffectiveInputMemo(const RrProgram *program,uint32_t stepIndex,std::string *key,
                         std::vector<uint32_t> *covered,
                         std::vector<std::pair<uint32_t,uint32_t>> *coveredTyped)
{
    if(stepIndex>=program->steps->size()) return false;
    const auto &step=(*program->steps)[stepIndex];
    if (step.kind == RigExecWireStepKind::ComposeSubtree) {
        const auto *pose = static_cast<const RrPoseScratch *>(program->pose.get());
        if (!pose) return false;
        const auto append = [&](const auto &value) {
            key->append(reinterpret_cast<const char *>(&value), sizeof(value));
        };
        // Preserve complete Ladder semantics for other consumers. Compose
        // observes this projection: restRoundTrip is not read by its body.
        for (const auto &range : step.reads) {
            if (range.domain() != RigExecWireSlotDomain::Ladder) continue;
            if (range.begin() > range.end()) return false;
            for (uint32_t slot = range.begin(); slot < range.end(); ++slot) {
                if (slot >= pose->selfD.size() || slot >= pose->parentDinv.size() ||
                    slot >= pose->defaultRoundTrip.size() || slot >= pose->rotOrder.size() ||
                    slot >= pose->posedAuthored.size() || slot >= pose->posedAuthoredM.size() ||
                    slot >= pose->posedD.size() || slot >= pose->parentSpaceM.size() ||
                    slot >= pose->parentSpaceAuthored.size() || slot >= pose->rotationSign.size())
                    return false;
                append(uint32_t(RigExecWireSlotDomain::Ladder)); append(slot);
                append(pose->selfD[slot]); append(pose->parentDinv[slot]);
                append(pose->defaultRoundTrip[slot]); append(pose->rotOrder[slot]);
                append(pose->posedAuthored[slot]); append(pose->posedAuthoredM[slot]);
                append(pose->posedD[slot]); append(pose->parentSpaceM[slot]);
                append(pose->parentSpaceAuthored[slot]); append(pose->rotationSign[slot]);
                if (coveredTyped)
                    coveredTyped->push_back({uint32_t(RigExecWireSlotDomain::Ladder), slot});
            }
        }
        // Continue through ordinary active-switch input fingerprints below.
        // All other declared typed inputs retain independent revision reasons.
    }
    if(step.kind==RigExecWireStepKind::WeightField)
        return RrWeightFieldEffectiveInputMemo(program,uint32_t(step.object),key,covered);
    if(step.kind==RigExecWireStepKind::SpaceExpression)
        return RrProviderEffectiveInputMemo(program,step,key,covered);
    if(step.kind==RigExecWireStepKind::ProviderRefresh) {
        const auto &refresh=program->poses->providerRefreshes[size_t(step.object)];
        for(const auto &carry:refresh.carries) for(int slot:carry.blockingSlots) {
            const auto &input=program->RegisteredInput(program->ladderRead[size_t(slot)][RrLadderParentSpace]);
            _RrCoveredRead(input,covered);
            _RrEffectiveWireMemo(program,RrReadProviderRefreshBlocker(program,slot),key);
        }
        // The adapter independently appends all entering frame, provider frame
        // and CommitTable semantic revisions, including did-commit guards.
        return true;
    }
    const bool crossCoverage=coveredTyped && (step.kind==RigExecWireStepKind::PropertyRevision ||
        step.kind==RigExecWireStepKind::AvarInputs);
    const auto read=[&](const RigExecWireInput &input) {
        _RrCoveredRead(input,covered); RrEffectiveReadMemo(program,input,key);
        if(!crossCoverage) return;
        const auto add=[&](RigExecWireSlotDomain domain,uint32_t slot) {
            coveredTyped->push_back({uint32_t(domain),slot});
        };
        for(const auto *segment:{&input.propertyCandidates,&input.doubleCandidates})
            for(const auto &hop:*segment) if(hop.crossDomain>=0) {
                const auto &cross=program->file->crossDomainReads[size_t(hop.crossDomain)];
                if(cross.kind==fb::CrossDomainReadKind::SpaceValue) add(RigExecWireSlotDomain::SpaceValue,uint32_t(cross.spaceValue));
                else if(cross.kind==fb::CrossDomainReadKind::PropertyResult) add(RigExecWireSlotDomain::PropertyResult,cross.propertyVersion);
                else if(cross.kind==fb::CrossDomainReadKind::PoseFrame)
                    for(uint32_t version:cross.frames) add(cross.baseFrame?RigExecWireSlotDomain::PoseBase:RigExecWireSlotDomain::PoseFin,version);
                else for(const auto &point:cross.points) {
                    if(cross.finalPoints) add(RigExecWireSlotDomain::ChainPoints,uint32_t(point.first));
                    else if(point.second==0) add(RigExecWireSlotDomain::ChainBase,uint32_t(point.first));
                    else {
                        const uint32_t revision=uint32_t(program->geometry->chainRevisionBegin[size_t(point.first)])+uint32_t(point.second)-1;
                        add(RigExecWireSlotDomain::RevisionDone,revision); add(RigExecWireSlotDomain::ChainDirty,revision);
                    }
                }
            }
    };
    const auto sites=[&](const auto &rows) {
        for(const auto &site:rows) {
            if(site.exactVersion>=0) covered->push_back(uint32_t(site.exactVersion));
            if(site.bodyWalk) _RrCoveredRead(*site.bodyWalk,covered);
            if(RigExecFormatIsArrayTag(site.read->tag)) {
                // Authored raw Default values stand on the canonical rest
                // snapshot even when their bytes equal the numeric bake sample.
                if(site.time==fb::ExternalInputTime::AtDefault &&
                   site.flavour==fb::ExternalInputFlavour::Raw && !site.read->walk.empty()) {
                    const bool authored=RrInputArrayAuthored(program,site.read->walk.front());
                    key->append(reinterpret_cast<const char *>(&authored),sizeof(authored));
                }
                _RrEffectiveArrayMemo(program,RrReadExternalArray(program,site),site.read->tag,key);
            } else { RrWireValue value; const bool have=RrReadExternalScalar(program,site,&value);
                key->append(reinterpret_cast<const char *>(&have),sizeof(have));
                if(have) _RrEffectiveWireMemo(program,value,key); }
        }
    };
    if(step.kind==RigExecWireStepKind::PropertyRevision && step.object>=0) {
        const auto &chain=program->file->propertyChains[size_t(step.object)];
        if(step.part==0) {
            RigExecWireInput raw; raw.tag=program->file->inputs[chain.target].type();
            raw.mode=RigExecWireReadMode::Raw; raw.walk={chain.target};
            raw.constant=program->file->inputs[chain.target].value();
            RrSourceReadMemo(program,raw,key);
        } else {
            const auto &revision=chain.revisions[size_t(step.part)-1];
            for(const auto *input:{revision.enabled.get(),revision.defaultWeight.get(),revision.value.get(),revision.min.get(),revision.max.get()})
                if(input) read(*input);
        }
        return true;
    }
    if(step.kind==RigExecWireStepKind::AvarInputs) {
        const auto &state=program->inputState;
        const auto range=RrAvarReadRange(state,step.object);
        for(uint32_t i=range.first;i<range.second;++i)
            read(*program->registeredReads[state.avarReads[i]].read);
        return true;
    }
    const RigExecWireRevision *owner=nullptr;
    bool layout=false;
    if(step.kind==RigExecWireStepKind::RevisionStatic && step.object>=0) {
        const auto &id=program->geometry->revisionIndex[size_t(step.object)];
        owner=&program->geometry->chains[size_t(id.first)].revisions[size_t(id.second)];
    } else if(step.kind==RigExecWireStepKind::Derived && step.object>=0) {
        const auto &id=program->geometry->derivedIndex[size_t(step.object)];
        owner=program->geometry->chains[size_t(id.first)].derived[size_t(id.second)].revision.get();
    } else if(step.kind==RigExecWireStepKind::SkinTopology && step.object>=0) {
        layout=true; const size_t index=size_t(step.object);
        if(index<program->geometry->revisionIndex.size()) {
            const auto &id=program->geometry->revisionIndex[index];
            owner=&program->geometry->chains[size_t(id.first)].revisions[size_t(id.second)];
        } else {
            const auto &id=program->geometry->derivedIndex[index-program->geometry->revisionIndex.size()];
            owner=program->geometry->chains[size_t(id.first)].derived[size_t(id.second)].revision.get();
        }
    }
    if(owner) { sites(layout?owner->layoutLeafSites:owner->leafSites); return true; }
    if(!step.headInputReads.empty()) {
        for(const auto &input:step.headInputReads) read(input);
        return step.headInputSlots.empty();
    }
    if (!step.headInputSlots.empty() || !step.overrideInputs.empty()) return false;
    // These bodies consume only their declared typed values and immutable epoch facts.
    // The adapter independently compares every typed read revision.
    switch (step.kind) {
    case RigExecWireStepKind::ComposeSubtree:
    case RigExecWireStepKind::FrameMatrix:
    case RigExecWireStepKind::ProviderMatrix:
    case RigExecWireStepKind::SolverCommit:
    case RigExecWireStepKind::CommitDelta:
    case RigExecWireStepKind::PropagateChunk:
    case RigExecWireStepKind::CommitApply:
    case RigExecWireStepKind::InfluenceFold:
    case RigExecWireStepKind::RevisionChunk:
    case RigExecWireStepKind::RevisionFuse:
    case RigExecWireStepKind::ChainStatus:
    case RigExecWireStepKind::ChainInputs:
    case RigExecWireStepKind::VolumePlacements:
    case RigExecWireStepKind::SpaceCheckpoint:
    case RigExecWireStepKind::ProviderRefresh:
        return true;
    default:
        return false;
    }
}

void RrSourceReadMemo(const RrProgram *program,const RigExecWireInput &read,std::string *key,
                      bool versioned)
{
    const auto &state=program->inputState;
    const auto append=[&](const auto &v) { key->append(reinterpret_cast<const char *>(&v),sizeof(v)); };
    const auto text=[&](const std::string &v) { append(uint64_t(v.size())); key->append(v); };
    const auto scalar=[&](const RrWireValue &v) {
        append(uint8_t(v.tag)); append(v.bits);
        for(double x:v.matrix) append(x);
        for(double x:v.vec3d) append(x);
        for(float x:v.vec3f) append(x);
        for(int32_t x:v.vec3i) append(x);
        if(v.tag==RigExecWireInputTag::Token) text(program->TextOrEmpty(uint32_t(v.bits)));
    };
    const auto array=[&](const void *data,RigExecWireInputTag tag) {
        const auto bytes=[&](const auto *v) {
            append(uint64_t(v->size()));
            if(!v->empty()) key->append(reinterpret_cast<const char *>(v->data()),v->size()*sizeof((*v)[0]));
        };
        switch(tag) {
        case RigExecWireInputTag::IntArray: bytes(static_cast<const std::vector<int32_t> *>(data)); break;
        case RigExecWireInputTag::FloatArray: bytes(static_cast<const std::vector<float> *>(data)); break;
        case RigExecWireInputTag::DoubleArray: bytes(static_cast<const std::vector<double> *>(data)); break;
        case RigExecWireInputTag::Vec2fArray: bytes(static_cast<const std::vector<RrVec2f> *>(data)); break;
        case RigExecWireInputTag::Vec3fArray: bytes(static_cast<const std::vector<RrVec3f> *>(data)); break;
        case RigExecWireInputTag::Vec3dArray: bytes(static_cast<const std::vector<RigExecWireVec3d> *>(data)); break;
        case RigExecWireInputTag::Matrix4dArray: bytes(static_cast<const std::vector<RigExecWireMatrix4d> *>(data)); break;
        case RigExecWireInputTag::BoolArray: bytes(static_cast<const std::vector<uint8_t> *>(data)); break;
        case RigExecWireInputTag::TokenArray:
            for(uint32_t token:*static_cast<const std::vector<uint32_t> *>(data)) text(program->TextOrEmpty(token));
            append(uint64_t(static_cast<const std::vector<uint32_t> *>(data)->size())); break;
        default: break;
        }
    };
    append(read.sampleTime); append(uint8_t(read.tag));
    scalar(state.values[read.constant]);
    std::vector<uint32_t> slots=read.walk;
    if(read.rawFallbackSlot>=0) slots.push_back(uint32_t(read.rawFallbackSlot));
    for(const auto *segment:{&read.propertyCandidates,&read.doubleCandidates})
        for(const auto &hop:*segment) if(hop.raw) slots.push_back(hop.slot);
    std::sort(slots.begin(),slots.end()); slots.erase(std::unique(slots.begin(),slots.end()),slots.end());
    for(uint32_t slot:slots) {
        const bool authored=RigExecFormatIsArrayTag(state.file->inputs[slot].type())
            ? RrInputArrayAuthored(program,slot) : state.slotAuthored[slot]!=0;
        const bool defaultRead=read.sampleTime==1 && !authored;
        const bool have=defaultRead?state.slotDefaultHasValue[slot]!=0:state.slotHasValue[slot]!=0;
        append(slot); append(have); append(authored);
        append(uint8_t(defaultRead?0:state.slotBlocked[slot]));
        const auto tag=state.file->inputs[slot].type(); append(uint8_t(tag));
        if(!have) continue;
        if(RigExecFormatIsArrayTag(tag)) {
            const void *value=defaultRead?RrInputArrayDefault(program,slot,tag):RrInputArrayValue(program,slot,tag);
            append(bool(value));
            // Token arrays key their text, which ids need not spell uniquely.
            if(value && versioned && tag!=RigExecWireInputTag::TokenArray) {
                if(!defaultRead) append(RrInputArrayVersion(program,slot));
            } else if(value) array(value,tag);
        } else scalar(defaultRead?state.values[state.file->inputs[slot].value()]:state.slotCurrent[slot]);
    }
}

bool
RrReadExternalScalar(const RrProgram *program,
                     const RigExecWireExternalDeclaredInput &declaration,
                     RrWireValue *out)
{
    const auto &read=*declaration.read;
    const auto &state=program->inputState;
    if (declaration.exactVersion >= 0 && declaration.flavour != fb::ExternalInputFlavour::Raw) {
        const size_t version=size_t(declaration.exactVersion);
        const bool published=program->store.propertyVersionValid[version] &&
            (declaration.exactRecord < 0 ||
             !program->store.propertyRecordStoodAside[size_t(declaration.exactRecord)]);
        bool typed=false;
        RrWireValue produced;
        if (published) {
            const auto &value=program->store.propertyVersions[version];
            typed=_RrPropertyWire(value,read.tag,&produced);
            if (!typed && declaration.allowFloatToDouble && read.tag==RigExecWireInputTag::Double && value.tag==RrPropertyValue::Tag::Float) {
                produced.tag=read.tag; const double widened=double(value.f32);
                std::memcpy(&produced.bits,&widened,sizeof(widened)); typed=true;
            } else if (!typed && read.tag==RigExecWireInputTag::Float && value.tag==RrPropertyValue::Tag::Double) {
                produced=_RrFloatValue(float(value.f64)); typed=true;
            }
        }
        if (declaration.flavour == fb::ExternalInputFlavour::Present) {
            // Runtime SetInput is an authored raw layer, not the native
            // interactive publication overlay observed by Present.
            out->tag=RigExecWireInputTag::Bool; out->bits=uint64_t(published);
            return true;
        }
        if (typed) { *out=produced; return true; }
        if (published && read.tag==RigExecWireInputTag::Double && !declaration.allowFloatToDouble &&
            declaration.bodyWalk && _RrCandidates(*program,*declaration.bodyWalk,out,false,nullptr,
                declaration.time==fb::ExternalInputTime::AtDefault)) return true;
        if (declaration.flavour == fb::ExternalInputFlavour::ResolvedOnly) {
            if (declaration.fallbackHasValue) { *out=state.values[read.constant]; return true; }
            return false;
        }
        // Invalid producers retain the current raw sample fallback, never a
        // previous generation's producer or a different publication at this path.
    } else if (declaration.bodyWalk && declaration.flavour != fb::ExternalInputFlavour::Raw && declaration.flavour != fb::ExternalInputFlavour::Present) {
        // Declared candidates identify current graph values; publication
        // slots are populated only after all consuming operations complete.
        if (_RrCandidates(*program,*declaration.bodyWalk,out,false,nullptr,declaration.time == fb::ExternalInputTime::AtDefault)) return true;
        if (declaration.flavour == fb::ExternalInputFlavour::ResolvedOnly) {
            if (declaration.fallbackHasValue) { *out=state.values[read.constant]; return true; }
            return false;
        }
    }
    bool have=false;
    // Raw/default reads never consult computed publication. A sampled Default
    // value remains the captured default unless an authored raw layer replaces it.
    if (declaration.time == fb::ExternalInputTime::AtDefault && declaration.flavour != fb::ExternalInputFlavour::Raw && declaration.flavour != fb::ExternalInputFlavour::Present &&
        (!read.propertyCandidates.empty() || !read.doubleCandidates.empty())) {
        have=_RrCandidates(*program,read,out,false,nullptr,true);
    } else if (declaration.flavour == fb::ExternalInputFlavour::Raw) {
        if (!read.walk.empty()) {
            const uint32_t slot=read.walk.front();
            const bool authored=state.slotAuthored[slot]!=0;
            const bool present=declaration.time == fb::ExternalInputTime::AtDefault && !authored
                ? state.slotDefaultHasValue[slot]!=0
                : state.slotHasValue[slot]!=0;
            if (present && state.file->inputs[slot].type()==read.tag) {
                *out=declaration.time == fb::ExternalInputTime::AtDefault && !authored ? state.values[state.file->inputs[slot].value()] : state.slotCurrent[slot];
                have=true;
            }
        }
    } else if (declaration.flavour == fb::ExternalInputFlavour::Present) {
        // No exact producer means no declared current publication. Raw
        // authored values do not manufacture an interactive overlay.
        out->tag=RigExecWireInputTag::Bool; out->bits=0; have=true;
    } else {
        const bool candidates=!read.propertyCandidates.empty() || !read.doubleCandidates.empty();
        if (candidates) have=_RrCandidates(*program,read,out);
        else {
            _RrWalkAnswer answer;
            have=_RrLongWay(*program,read.walk,0,read.tag,&answer,declaration.time == fb::ExternalInputTime::AtDefault);
            if (have) *out=answer.value;
        }

    }
    if (!have && (declaration.flavour == fb::ExternalInputFlavour::Resolved ||
                  declaration.flavour == fb::ExternalInputFlavour::OverlayThenRaw) &&
        !read.walk.empty()) {
        // Native Resolved retries Get on the consuming attribute when
        // the connected typed walk fails. This is its own exact raw head,
        // not another Float hop in an unsuccessful Double tail.
        const uint32_t slot=read.walk.front();
        const bool atDefault=declaration.time == fb::ExternalInputTime::AtDefault &&
                             !state.slotAuthored[slot];
        const bool present=atDefault ? state.slotDefaultHasValue[slot]!=0
                                     : state.slotHasValue[slot]!=0;
        if (present && state.file->inputs[slot].type()==read.tag) {
            *out=atDefault ? state.values[state.file->inputs[slot].value()]
                           : state.slotCurrent[slot];
            have=true;
        }
    }
    if (!have && declaration.fallbackHasValue) {
        *out=state.values[read.constant]; have=true;
    }
    return have;
}

RrWireValue
RrReadInput(const RrProgram *program, const RigExecWireInput &input)
{
    const RrInputState &state = program->inputState;
    const RrWireValue &fallback = state.values[input.constant];
    switch (input.mode) {
    case RigExecWireReadMode::Baked: {
        // RigExecBakedRead: an override reads the long way; a fixed input
        // is its folded constant unless explicitly source-backed; a long-way input walks; otherwise the
        // query pinned at Build reads its own attribute, or the constant
        // when that read fails.
        const std::vector<char> &overridden = program->store.overridden;
        if (input.overrideIndex >= 0 &&
            size_t(input.overrideIndex) < overridden.size() &&
            overridden[size_t(input.overrideIndex)]) {
            return _RrWalkValue(*program, input);
        }
        if ((input.flags & (uint8_t(RigExecWireInputReadFlags::Varying) |
                            uint8_t(RigExecWireInputReadFlags::SourceBacked))) == 0) {
            return fallback;
        }
        if ((input.flags & uint8_t(RigExecWireInputReadFlags::LongWay)) !=
            0) {
            return _RrWalkValue(*program, input);
        }
        if (input.selected >= 0) {
            const uint32_t slot = input.walk[size_t(input.selected)];
            if (state.slotHasValue[slot] && !state.slotBlocked[slot] &&
                state.file->inputs[slot].type() == input.tag) {
                return _RrSlotValue(state, slot);
            }
        }
        return fallback;
    }
    case RigExecWireReadMode::Resolved:
        return _RrWalkValue(*program, input);
    case RigExecWireReadMode::Pinned: {
        if (!input.propertyCandidates.empty() || !input.doubleCandidates.empty()) {
            RrWireValue answer;
            return _RrCandidates(*program, input, &answer) ? answer : fallback;
        }
        // _PinnedRead: a connected input reads the long way; otherwise an
        // overlay at the attribute's own path holding exactly the read's
        // type answers before the attribute's own value.
        if (input.walk.size() > 1) {
            return _RrWalkValue(*program, input);
        }
        RrWireValue overlay;
        if (!input.walk.empty() &&
            _RrSlotOverlay(*program, input.walk[0],
                       input.tag, &overlay)) {
            return overlay;
        }
        return _RrHeadValue(state, input);
    }
    case RigExecWireReadMode::Raw:
        return _RrHeadValue(state, input);
    }
    return fallback;
}

RrWireValue RrReadProviderRefreshBlocker(const RrProgram *program,int provider)
{
    const auto &input=program->RegisteredInput(program->ladderRead[size_t(provider)][RrLadderParentSpace]);
    const auto &state=program->inputState;
    const int slot=input.rawFallbackSlot>=0 ? input.rawFallbackSlot :
        input.walk.empty() ? -1 : int(input.walk.front());
    if(slot>=0 && state.slotHasValue[size_t(slot)] && !state.slotBlocked[size_t(slot)] &&
       state.file->inputs[size_t(slot)].type()==RigExecWireInputTag::Matrix4d)
        return _RrSlotValue(state,uint32_t(slot));
    RrWireValue identity; identity.tag=RigExecWireInputTag::Matrix4d;
    identity.matrix.fill(0.0);
    for(size_t axis=0;axis<4;++axis)identity.matrix[axis*4+axis]=1.0;
    return identity;
}

RrWireValue
RrReadPathScalar(const RrProgram *program, const RigExecWirePathRead &row)
{
    const RrInputState &state = program->inputState;
    const RigExecWireInput &read = *row.read;
    if (!row.headFallback || read.mode != RigExecWireReadMode::Resolved) {
        return RrReadInput(program, read);
    }
    // moverGraph.cpp's _Read: `if (!GetAttribute) attr.Get`, so a walk
    // that yields nothing still reads the head before the fallback.
    _RrWalkAnswer answer;
    const bool bound = !read.propertyCandidates.empty() || !read.doubleCandidates.empty();
    if (bound ? _RrCandidates(*program, read, &answer.value)
              : _RrLongWay(*program, read.walk, 0, read.tag, &answer)) {
        return answer.value;
    }
    return _RrHeadValue(state, read);
}

float
RrReadResolvedFloat(const RrProgram *program, const RigExecWireInput &input,
                    float fallback)
{
    _RrWalkAnswer answer;
    const bool bound = !input.propertyCandidates.empty() || !input.doubleCandidates.empty();
    if (!(bound ? _RrCandidates(*program, input, &answer.value, true)
                : _RrLongWay(*program, input.walk, 0, RigExecWireInputTag::Float,
                             &answer))) {
        return fallback;
    }
    return RrWireValueFloat(answer.value);
}

float
RrReadOracleFloat(const RrProgram *program, const RigExecWireInput &input,
                  const RrOracleReadContext &context, float fallback)
{
    RrWireValue value;
    return _RrCandidates(*program, input, &value, false, &context)
               ? RrWireValueFloat(value) : fallback;
}

namespace {

using _RrFamily = RrReadFamily;

// The Input of ladder field \p field (RrLadderField).
const RigExecWireInput *
_RrLadderField(const RigExecWireLadder &ladder, int field)
{
    switch (field) {
    case RrLadderRestSpace:
        return ladder.restSpace.get();
    case RrLadderDefaultSpace:
        return ladder.defaultSpace.get();
    case RrLadderPosedSpace:
        return ladder.posedSpace.get();
    case RrLadderRotationOrder:
        return ladder.rotationOrder.get();
    case RrLadderParentSpace: return ladder.parentSpace.get();
    case RrLadderParentDefaultSpace: return ladder.parentDefaultSpace.get();
    case RrLadderAvarDefaultSpace: return ladder.avarDefaultSpace.get();
    case RrLadderPosedDefaultSpace: return ladder.posedDefaultSpace.get();
    case RrLadderRotationSign: return ladder.rotationSign.get();
    case RrLadderInterveningSpace: return ladder.interveningSpace.get();
    default:
        break;
    }
    if (field >= RrLadderRestAvar0 && field < RrLadderRestAvar0 + 6) {
        return &ladder.restAvars[size_t(field - RrLadderRestAvar0)];
    }
    return &ladder.defaultAvars[size_t(field - RrLadderDefaultAvar0)];
}

// The Input of solver field \p field (RrSolverField).
const RigExecWireInput *
_RrSolverField(const RigExecWireSolver &s, int field)
{
    switch (field) {
    case RrSolverBend:
        return s.bend.get();
    case RrSolverUpperOffset:
        return s.upperOffset.get();
    case RrSolverLowerOffset:
        return s.lowerOffset.get();
    case RrSolverStretch:
        return s.stretch.get();
    case RrSolverSoftness:
        return s.softness.get();
    case RrSolverBlendWeight:
        return s.blendWeight.get();
    case RrSolverPreserveVolume:
        return s.preserveVolume.get();
    case RrSolverMidFollowWeight:
        return s.midFollowWeight.get();
    case RrSolverRoll:
        return s.roll.get();
    case RrSolverTwist:
        return s.twist.get();
    case RrSolverMinLengthRatio:
        return s.minLengthRatio.get();
    case RrSolverTwistTurns:
        return s.twistTurns.get();
    case RrSolverRibbonSampleCount:
        return s.ribbonSampleCount.get();
    case RrSolverPin:return s.pin.get();
    case RrSolverUpperScale:return s.upperScale.get();
    case RrSolverLowerScale:return s.lowerScale.get();
    case RrSolverSoftDistance:return s.softDistance.get();
    case RrSolverLimbTwist:return s.limbTwist.get();
    default:return s.ikSpace.get();
    }
}

// The Input of constraint field \p field (RrConstraintField).
const RigExecWireInput *
_RrConstraintField(const RigExecWireConstraint &c, int field)
{
    switch (field) {
    case RrConstraintEnabled:
        return c.enabled.get();
    case RrConstraintDefaultWeight:
        return c.defaultWeight.get();
    case RrConstraintOffset:
        return c.offset.get();
    case RrConstraintAffectX:
        return c.affectX.get();
    case RrConstraintAffectY:
        return c.affectY.get();
    case RrConstraintAffectZ:
        return c.affectZ.get();
    case RrConstraintTX:
        return c.tX.get();
    case RrConstraintTY:
        return c.tY.get();
    case RrConstraintTZ:
        return c.tZ.get();
    case RrConstraintRX:
        return c.rX.get();
    case RrConstraintRY:
        return c.rY.get();
    case RrConstraintRZ:
        return c.rZ.get();
    case RrConstraintSX:
        return c.sX.get();
    case RrConstraintSY:
        return c.sY.get();
    case RrConstraintSZ:
        return c.sZ.get();
    case RrConstraintAimVector:
        return c.aimVector.get();
    case RrConstraintUpVector:
        return c.upVector.get();
    case RrConstraintRotationOffset:
        return c.rotationOffset.get();
    case RrConstraintWorldUpVector:
        return c.worldUpVector.get();
    case RrConstraintPoleVector:
        return c.poleVector.get();
    case RrConstraintStretch:return c.stretch.get();
    default:
        return c.twistDegrees.get();
    }
}

}  // namespace

bool
RrInputsBindReads(RrProgram *program, std::string *error)
{
    RrInputState &state = program->inputState;
    const RigExecWireFile &file = *state.file;
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const size_t slots = file.inputs.size();
    const size_t avars = program->constants->avarConstants.size();
    state.avarReadBegin.clear();
    state.avarReads.clear();
    state.ladderOverrides.clear();
    program->registeredReads.clear();
    // Per slot: the avar binding the slot heads, or -1. A slot heads one.
    std::vector<int32_t> slotAvar(slots, -1);
    std::array<int32_t, RrLadderFieldCount> ladderNone;
    ladderNone.fill(-1);
    program->ladderRead.assign(poses.ladders.size(), ladderNone);
    program->spaceSwitchRead.assign(poses.spaceSwitches.size(), -1);
    program->interpRead.assign(poses.poseInterpolators.size(), -1);
    program->interpValueRead.assign(poses.poseInterpolators.size(),
                                    {-1, -1, -1});
    std::array<int32_t, RrSolverFieldCount> solverNone;
    solverNone.fill(-1);
    program->solverRead.assign(poses.solvers.size(), solverNone);
    std::array<int32_t, RrConstraintFieldCount> constraintNone;
    constraintNone.fill(-1);
    program->constraintRead.assign(poses.constraints.size(), constraintNone);
    std::array<int32_t, RrWeightFieldCount> weightNone;
    weightNone.fill(-1);
    const size_t stepObjects =
        std::min(program->stepWeightObjects, geometry.weightObjects.size());
    program->weightRead.assign(stepObjects, weightNone);

    // (slot, override number) for every read whose walk passes the slot.
    std::vector<std::pair<uint32_t, int32_t>> reach;
    // Registers one table field's read; false when the file left it out.
    const auto bind = [&](const RigExecWireInput *read, _RrFamily family,
                          size_t object, size_t field, int32_t *at) {
        if (!read) {
            return false;
        }
        RrRegisteredRead entry;
        entry.read = read;
        entry.family = family;
        entry.object = uint32_t(object);
        entry.field = uint32_t(field);
        if (at) {
            *at = int32_t(program->registeredReads.size());
        }
        if (read->overrideIndex >= 0) {
            for (uint32_t slot : read->walk) {
                reach.emplace_back(slot, read->overrideIndex);
            }
        }
        program->registeredReads.push_back(entry);
        return true;
    };
    const auto unbound = [&](const char *table, size_t row) {
        return _RrFail(error, "the file binds no read to a field of " +
                                  std::string(table) + "[" +
                                  std::to_string(row) + "]");
    };
    // The avar table, varying bindings first (the program's avarBindings),
    // then the constant ones (avarConstantBindings), each in file order.
    std::vector<uint32_t> avarOrder;
    for (const bool varying : {true, false}) {
        for (size_t i = 0; i < poses.avarBindings.size(); ++i) {
            const RigExecWireAvarBinding &binding = poses.avarBindings[i];
            const RigExecWireInput *read = binding.read.get();
            if (!read) {
                return unbound("pose.avar_bindings", i);
            }
            const bool flagged =
                (read->flags & uint8_t(RigExecWireInputReadFlags::Varying)) !=
                0;
            if (flagged != varying) {
                continue;
            }
            const std::string field =
                "pose.avar_bindings[" + std::to_string(i) + "]";
            if (size_t(binding.flat) >= avars ||
                read->tag != RigExecWireInputTag::Double) {
                return _RrFail(error, field + " names no avar");
            }
            if (!read->walk.empty()) {
                if (read->walk[0] >= slots) {
                    return _RrFail(error, field + " walks no slot");
                }
                int32_t &head = slotAvar[read->walk[0]];
                if (head >= 0 && head != int32_t(binding.flat)) {
                    return _RrFail(error, field + " heads two avars");
                }
                head = int32_t(binding.flat);
            }
            avarOrder.push_back(uint32_t(program->registeredReads.size()));
            bind(read,
                 varying ? _RrFamily::AvarBinding
                         : _RrFamily::AvarConstantBinding,
                 i, 0, nullptr);
            program->registeredReads.back().avar = int32_t(binding.flat);
        }
    }
    // Bucketed by provider with a stable counting sort, so a provider's
    // reads keep the walk order above whatever order the file lists them.
    {
        const size_t providers = (avars + 10) / 11;
        const auto providerOf = [&](uint32_t id) {
            return program->registeredReads[id].avar;
        };
        state.avarReadBegin.assign(providers + 1, 0);
        for (const uint32_t id : avarOrder) {
            if (providerOf(id) >= 0) {
                ++state.avarReadBegin[size_t(providerOf(id)) / 11 + 1];
            }
        }
        for (size_t p = 0; p < providers; ++p) {
            state.avarReadBegin[p + 1] += state.avarReadBegin[p];
        }
        state.avarReads.resize(state.avarReadBegin[providers]);
        std::vector<uint32_t> next(state.avarReadBegin.begin(),
                                   state.avarReadBegin.end() - 1);
        for (const uint32_t id : avarOrder) {
            if (providerOf(id) >= 0) {
                state.avarReads[next[size_t(providerOf(id)) / 11]++] = id;
            }
        }
    }
    for (size_t i = 0; i < poses.ladders.size(); ++i) {
        for (int field = 0; field < RrLadderFieldCount; ++field) {
            const RigExecWireInput *read =
                _RrLadderField(poses.ladders[i], field);
            if (!bind(read, _RrFamily::Ladder, i, size_t(field),
                      &program->ladderRead[i][size_t(field)])) {
                return unbound("pose.ladders", i);
            }
            // RigExecBakedProgramImpl::ladderOverrides: the provider
            // slots' ladders a drag recomposes.
            if (read->overrideIndex >= 0 && i < meta.slotKind.size() &&
                meta.slotKind[i] == RigExecWireSlotKind::FirstFramePose) {
                state.ladderOverrides.push_back(read->overrideIndex);
            }
        }
    }
    program->autoClavicleRead.resize(poses.autoClavicles.size());
    for(size_t i=0;i<poses.autoClavicles.size();++i) {
        const auto &ac=poses.autoClavicles[i];program->autoClavicleRead[i].assign(ac.scalars.size(),-1);
        for(size_t k=0;k<ac.scalars.size();++k)if(!bind(&ac.scalars[k],_RrFamily::AutoClavicle,i,k,&program->autoClavicleRead[i][k]))
            return unbound("pose.auto_clavicles",i);
    }
    for (size_t i = 0; i < poses.spaceSwitches.size(); ++i) {
        if (!bind(poses.spaceSwitches[i].active.get(), _RrFamily::SpaceSwitch,
                  i, 0, &program->spaceSwitchRead[i])) {
            return unbound("pose.space_switches", i);
        }
    }
    for (size_t i = 0; i < poses.poseInterpolators.size(); ++i) {
        const RigExecWirePoseInterpolator &interp =
            poses.poseInterpolators[i];
        if (!bind(interp.enabled.get(), _RrFamily::Interpolator, i, 0,
                  &program->interpRead[i])) {
            return unbound("pose.pose_interpolators", i);
        }
        if (interp.valueInputs.size() > 3) {
            return unbound("pose.pose_interpolators", i);
        }
        for (size_t v = 0; v < interp.valueInputs.size(); ++v) {
            bind(&interp.valueInputs[v], _RrFamily::Interpolator, i, 1 + v,
                 &program->interpValueRead[i][v]);
        }
    }
    for (size_t i = 0; i < poses.solvers.size(); ++i) {
        for (int field = 0; field < RrSolverFieldCount; ++field) {
            if(field>=14 && !_RrSolverField(poses.solvers[i],field))continue;
            if (!bind(_RrSolverField(poses.solvers[i], field),
                      _RrFamily::Solver, i, size_t(field),
                      &program->solverRead[i][size_t(field)])) {
                return unbound("pose.solvers", i);
            }
        }
    }
    for (size_t i = 0; i < poses.constraints.size(); ++i) {
        for (int field = 0; field < RrConstraintFieldCount; ++field) {
            if(field==RrConstraintStretch && !poses.constraints[i].stretch)continue;
            if (!bind(_RrConstraintField(poses.constraints[i], field),
                      _RrFamily::Constraint, i, size_t(field),
                      &program->constraintRead[i][size_t(field)])) {
                return unbound("pose.constraints", i);
            }
        }
    }
    for (size_t i = 0; i < stepObjects; ++i) {
        int field = 0;
        bool bound = true;
        _RrForEachWeightRead(
            geometry.weightObjects[i], [&](const RigExecWireInput *read) {
                bound = bind(read, _RrFamily::WeightObject, i, size_t(field),
                             &program->weightRead[i][size_t(field)]) &&
                        bound;
                ++field;
            });
        if (!bound) {
            return unbound("geometry.weight_objects", i);
        }
    }
    std::sort(state.ladderOverrides.begin(), state.ladderOverrides.end());
    state.ladderOverrides.erase(std::unique(state.ladderOverrides.begin(),
                                            state.ladderOverrides.end()),
                                state.ladderOverrides.end());

    // Where an override on each slot reaches, as an index by slot.
    std::sort(reach.begin(), reach.end());
    reach.erase(std::unique(reach.begin(), reach.end()), reach.end());
    state.slotOverrideBegin.assign(slots + 1, 0);
    state.slotOverrideNumbers.clear();
    state.slotOverrideNumbers.reserve(reach.size());
    for (const auto &[slot, number] : reach) {
        ++state.slotOverrideBegin[slot + 1];
        state.slotOverrideNumbers.push_back(number);
    }
    for (size_t s = 0; s < slots; ++s) {
        state.slotOverrideBegin[s + 1] += state.slotOverrideBegin[s];
    }
    // And by override number, the walk slots a set value is compared on.
    {
        size_t numbers = 0;
        for (const auto &[slot, number] : reach) {
            numbers = std::max(numbers, size_t(number) + 1);
        }
        std::vector<std::pair<int32_t, uint32_t>> byNumber;
        byNumber.reserve(reach.size());
        for (const auto &[slot, number] : reach) {
            byNumber.emplace_back(number, slot);
        }
        std::sort(byNumber.begin(), byNumber.end());
        state.overrideSlotBegin.assign(numbers + 1, 0);
        state.overrideSlotList.clear();
        state.overrideSlotList.reserve(byNumber.size());
        for (const auto &[number, slot] : byNumber) {
            ++state.overrideSlotBegin[size_t(number) + 1];
            state.overrideSlotList.push_back(slot);
        }
        for (size_t n = 0; n < numbers; ++n) {
            state.overrideSlotBegin[n + 1] += state.overrideSlotBegin[n];
        }
        const uint8_t animated =
            uint8_t(RigExecWireInputSlotFlags::Animated);
        state.overrideWalkMoves.assign(numbers, 0);
        for (size_t n = 0; n < numbers; ++n) {
            for (uint32_t j = state.overrideSlotBegin[n];
                 j < state.overrideSlotBegin[n + 1]; ++j) {
                const RigExecWireInputSlot &slot =
                    file.inputs[state.overrideSlotList[j]];
                if ((slot.flags() & animated) != 0 || slot.chain() >= 0 ||
                    slot.phased() >= 0) {
                    state.overrideWalkMoves[n] = 1;
                    break;
                }
            }
        }
    }
    state.nameIndex.clear();
    state.nameIndex.reserve(slots);
    for (size_t s = 0; s < slots; ++s) {
        state.nameIndex.emplace(program->TextOrEmpty(file.inputs[s].name()),
                                uint32_t(s));
    }
    // Bit 1 retains provider-sampled membership; bit 2 marks only the
    // strictly validated source-backed registered direct numeric sites.
    // Bit 4 marks animated numeric sources of exact External AtTime declarations.
    // These are raw sampling channels, never additional public controls.
    for (const auto &registered : program->registeredReads) {
        const auto &read = *registered.read;
        if ((read.flags & uint8_t(RigExecWireInputReadFlags::SourceBacked)) != 0)
            state.slotProviderSource[read.walk[size_t(read.selected)]] |= 2;
    }
    // The listed inputs, as the input API reports them.
    const size_t listed = std::min<size_t>(file.listedInputs, slots);
    state.inputInfo.clear();
    state.inputInfo.reserve(listed);
    state.inputValues.clear();
    state.inputValues.reserve(listed);
    for (size_t s = 0; s < listed; ++s) {
        const RigExecWireInputSlot &slot = file.inputs[s];
        RigExecRuntimeInputInfo info;
        info.name = program->TextOrEmpty(slot.name());
        info.type = RrInputTag(uint8_t(slot.type()));
        info.animated = (slot.flags() &
                         uint8_t(RigExecWireInputSlotFlags::Animated)) != 0;
        info.defaultValue = RrValueFromWire(state.values[slot.value()]);
        state.inputValues.push_back(RrValueFromWire(state.slotCurrent[s]));
        state.inputInfo.push_back(std::move(info));
    }
    return true;
}

namespace {

// A static value of path_reads decoded out of the pools into the
// assemblers' form; only the member the tag names is set.
RrPathValue
_RrPathValueOf(const RigExecWireFile &file, const RigExecWirePathValue &wire)
{
    using Tag = RrPathValue::Tag;
    RrPathValue out;
    out.tag = Tag(uint8_t(wire.tag));
    switch (wire.tag) {
    case RigExecWirePathTag::Bool:
        out.boolean = wire.bits != 0;
        break;
    case RigExecWirePathTag::Int:
        out.i32 = int32_t(uint32_t(wire.bits));
        break;
    case RigExecWirePathTag::Float: {
        const uint32_t bits = uint32_t(wire.bits);
        std::memcpy(&out.f32, &bits, sizeof(out.f32));
        break;
    }
    case RigExecWirePathTag::Double:
        std::memcpy(&out.f64, &wire.bits, sizeof(out.f64));
        break;
    case RigExecWirePathTag::Token:
        out.token = uint32_t(wire.bits);
        break;
    case RigExecWirePathTag::Matrix4d:
        if (wire.matrix) {
            out.matrix = *wire.matrix;
        }
        break;
    case RigExecWirePathTag::Vec3d:
        if (wire.vec3d) {
            out.vec = *wire.vec3d;
        }
        break;
    case RigExecWirePathTag::Vec3i:
        if (wire.vec3i) {
            out.vec3i = *wire.vec3i;
        }
        break;
    case RigExecWirePathTag::IntArray:
        out.ints = file.intArrays[wire.array].v;
        break;
    case RigExecWirePathTag::FloatArray:
        out.floats = file.floatArrays[wire.array].v;
        break;
    case RigExecWirePathTag::Vec2fArray:
        out.vec2s.reserve(file.vec2fArrays[wire.array].v.size());
        for (const RigExecWireVec2f &p : file.vec2fArrays[wire.array].v) {
            out.vec2s.push_back(RrVec2f(p[0], p[1]));
        }
        break;
    case RigExecWirePathTag::Vec3fArray:
        out.vec3s.reserve(file.vec3fArrays[wire.array].v.size());
        for (const RigExecWireVec3f &p : file.vec3fArrays[wire.array].v) {
            out.vec3s.push_back(RrVec3f(p[0], p[1], p[2]));
        }
        break;
    case RigExecWirePathTag::DoubleArray:
        out.doubles = file.doubleArrays[wire.array].v;
        break;
    default:
        break;
    }
    return out;
}

bool
_RrPathKeyLess(const RrPathRead &a, const RrPathRead &b)
{
    return a.path != b.path ? a.path < b.path : a.rest < b.rest;
}

}  // namespace

int32_t
RrFindPathReadRow(const std::vector<RrPathRead> &table, uint32_t path,
                  bool rest)
{
    RrPathRead key;
    key.path = path;
    key.rest = rest;
    const auto found =
        std::lower_bound(table.begin(), table.end(), key, _RrPathKeyLess);
    if (found == table.end() || found->path != path || found->rest != rest) {
        return -1;
    }
    return int32_t(found - table.begin());
}

bool
RrInputsBuildPathReads(RrProgram *program, std::string *error)
{
    const RigExecWireFile &file = *program->inputState.file;
    const std::vector<RigExecWirePathRead> &rows =
        program->geometry->pathReads;
    std::vector<RrPathRead> &table = program->pathReads;
    table.clear();
    table.reserve(rows.size());
    program->pathReadRows.clear();
    // The validator holds the rows sorted strictly by (path, rest), each a
    // value, a scalar read, or an array read (with its Default-time value
    // at rest).
    for (const RigExecWirePathRead &row : rows) {
        RrPathRead entry;
        entry.path = row.path;
        entry.rest = row.rest;
        if (row.read && RigExecFormatIsArrayTag(row.read->tag)) {
            entry.read = &row;
            if (row.value) {
                entry.value = _RrPathValueOf(file, *row.value);
            }
        } else if (row.read) {
            RrPathValue probe;
            if (!RrPathValueFromRead(row, RrWireValue(), &probe)) {
                return _RrFail(error, "a connection-following path read has "
                                      "a type no assembler site reads");
            }
            entry.read = &row;
            program->pathReadRows.push_back(uint32_t(table.size()));
        } else if (row.value) {
            entry.value = _RrPathValueOf(file, *row.value);
        }
        table.push_back(std::move(entry));
    }
    return true;
}

namespace {

const char *
_RrTagName(RrInputTag tag)
{
    switch (tag) {
    case RrInputTag::Double:
        return "double";
    case RrInputTag::Float:
        return "float";
    case RrInputTag::Bool:
        return "bool";
    case RrInputTag::Int:
        return "int";
    case RrInputTag::Matrix4d:
        return "matrix4d";
    case RrInputTag::Token:
        return "token";
    case RrInputTag::Vec3d:
        return "vec3d";
    case RrInputTag::Vec3f:
        return "vec3f";
    case RrInputTag::IntArray:
        return "int[]";
    case RrInputTag::FloatArray:
        return "float[]";
    case RrInputTag::DoubleArray:
        return "double[]";
    case RrInputTag::Vec2fArray:
        return "float2[]";
    case RrInputTag::Vec3fArray:
        return "float3[]";
    }
    return "unknown";
}

// Marks array slot \p slot for the next run, once; the first mark since
// the last run keeps what that run read for the comparison.
void
_RrTouchArray(RrInputState &state, uint32_t slot, RrArraySlot &a)
{
    RrInputsMarkWritten(state, slot);
    if (state.touchedFlag[slot]) {
        return;
    }
    a.ranHeld = a.holdsSet;
    if (a.holdsSet) {
        a.ran = std::move(a.held);
        a.held = RrArrayBuffer();
    }
    state.touchedFlag[slot] = 1;
    state.touched.push_back(slot);
}

// Whether array slot \p slot holds other than its default.
bool
_RrArrayDiffersFromDefault(const RrInputState &state, uint32_t slot)
{
    const RrArraySlot &a = state.arrays[size_t(state.arrayOf[slot])];
    return a.holdsSet ||
           state.slotHasValue[slot] != state.slotDefaultHasValue[slot];
}

bool
_RrIsArraySlot(const RrInputState &state, size_t slot)
{
    return slot < state.arrayOf.size() && state.arrayOf[slot] >= 0;
}

// Slot \p slot holds \p value (with \p has), the input cache follows, and
// the slot is marked for the next run, once.
void
_RrStoreSlot(RrInputState &state, uint32_t slot, const RrWireValue &value,
             bool has)
{
    RrInputsMarkWritten(state, slot);
    state.slotCurrent[slot] = value;
    state.slotHasValue[slot] = has ? 1 : 0;
    state.slotBlocked[slot]=0;
    if (slot < state.inputValues.size()) {
        state.inputValues[slot] = RrValueFromWire(value);
    }
    if (!state.touchedFlag[slot]) {
        state.touchedFlag[slot] = 1;
        state.touched.push_back(slot);
    }
}

}  // namespace

bool
RrStageScalarSet(RrProgram *program,size_t slot,const RrInputValue &value,std::string *error)
{
    auto &state=program->inputState;
    if(slot>=state.slotProviderSource.size() || !state.slotProviderSource[slot])
        return _RrFail(error,"not a declared provider raw source");
    const auto tag=state.file->inputs[slot].type();
    if(RigExecFormatIsArrayTag(tag) || tag==RigExecWireInputTag::Token ||
       uint8_t(value.tag)!=uint8_t(tag)) return _RrFail(error,"wrong provider scalar type");
    RrWireValue wire; wire.tag=tag;
    switch(tag) {
    case RigExecWireInputTag::Double: std::memcpy(&wire.bits,&value.f64,sizeof(double)); break;
    case RigExecWireInputTag::Float: {uint32_t bits; std::memcpy(&bits,&value.f32,sizeof(float));wire.bits=bits;break;}
    case RigExecWireInputTag::Bool: wire.bits=value.boolean?1:0;break;
    case RigExecWireInputTag::Int: wire.bits=uint32_t(value.i32);break;
    case RigExecWireInputTag::Vec3d: for(size_t k=0;k<3;++k) wire.vec3d[k]=value.vec[k];break;
    case RigExecWireInputTag::Vec3f: for(size_t k=0;k<3;++k) wire.vec3f[k]=value.vec3f[k];break;
    case RigExecWireInputTag::Vec3i: for(size_t k=0;k<3;++k) wire.vec3i[k]=value.vec3i[k];break;
    case RigExecWireInputTag::Matrix4d:
        for(size_t k=0;k<16;++k) wire.matrix[k]=value.matrix[k/4][k%4];break;
    default: return _RrFail(error,"unsupported provider scalar type");
    }
    state.slotAuthored[slot]=0; _RrStoreSlot(state,uint32_t(slot),wire,true);return true;
}
bool
RrStageScalarClear(RrProgram *program,size_t slot,std::string *error)
{
    auto &state=program->inputState;
    if(slot>=state.slotProviderSource.size() || !state.slotProviderSource[slot] ||
       RigExecFormatIsArrayTag(state.file->inputs[slot].type()))
        return _RrFail(error,"not a declared provider scalar source");
    state.slotAuthored[slot]=0;
    _RrStoreSlot(state,uint32_t(slot),state.values[state.file->inputs[slot].value()],false);
    return true;
}

bool
RrInputsSet(RrProgram *program, size_t index, const RrInputValue &value,
            std::string *error, bool acceptNonFinite)
{
    RrInputState &state = program->inputState;
    if (index >= state.inputInfo.size()) {
        return _RrFail(error, "no input at index " + std::to_string(index) +
                                  "; the file lists " +
                                  std::to_string(state.inputInfo.size()));
    }
    const RigExecRuntimeInputInfo &info = state.inputInfo[index];
    const RrInputTag type = info.type;
    if (RrInputTagIsArray(type)) {
        return _RrFail(error, info.name + " is a " + _RrTagName(type) +
                                  " input: set it with SetInputArray");
    }
    // GetAttribute<float> narrows a double with static_cast<float>.
    const bool narrow =
        type == RrInputTag::Float && value.tag == RrInputTag::Double;
    if (value.tag != type && !narrow) {
        return _RrFail(error, info.name + " is a " + _RrTagName(type) +
                                  " input, not a " + _RrTagName(value.tag));
    }
    const auto notFinite = [&] {
        return _RrFail(error, info.name + " takes finite values only");
    };
    const auto isFinite = [acceptNonFinite](double component) {
        return acceptNonFinite || std::isfinite(component);
    };
    RrWireValue wire;
    wire.tag = RigExecWireInputTag(uint8_t(type));
    switch (type) {
    case RrInputTag::Double:
        if (!isFinite(value.f64)) {
            return notFinite();
        }
        std::memcpy(&wire.bits, &value.f64, sizeof(value.f64));
        break;
    case RrInputTag::Float: {
        if (narrow && !isFinite(value.f64)) {
            return notFinite();
        }
        const float f = narrow ? static_cast<float>(value.f64) : value.f32;
        if (!isFinite(double(f))) {
            return notFinite();
        }
        wire = _RrFloatValue(f);
        break;
    }
    case RrInputTag::Bool:
        wire.bits = value.boolean ? 1 : 0;
        break;
    case RrInputTag::Int:
        wire.bits = uint32_t(value.i32);
        break;
    case RrInputTag::Matrix4d:
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                if (!isFinite(value.matrix[r][c])) {
                    return notFinite();
                }
                wire.matrix[r * 4 + c] = value.matrix[r][c];
            }
        }
        break;
    case RrInputTag::Token: {
        std::string text;
        if (!program->GetText(value.token, &text)) {
            return _RrFail(error, info.name + ": token id " +
                                      std::to_string(value.token) +
                                      " names no text");
        }
        wire.bits = value.token;
        break;
    }
    case RrInputTag::Vec3d:
        for (size_t i = 0; i < 3; ++i) {
            if (!isFinite(value.vec[i])) {
                return notFinite();
            }
            wire.vec3d[i] = value.vec[i];
        }
        break;
    case RrInputTag::Vec3f:
        for (size_t i = 0; i < 3; ++i) {
            if (!isFinite(double(value.vec3f[i]))) {
                return notFinite();
            }
            wire.vec3f[i] = value.vec3f[i];
        }
        break;
    default:
        break;
    }
    state.slotAuthored[index] = acceptNonFinite ? 0 : 1;
    _RrStoreSlot(state, uint32_t(index), wire, true);
    return true;
}

bool
_RrSetArraySlot(RrProgram *program, size_t index,
                 const RigExecRuntimeArray &value, bool authored,
                 std::string *error)
{
    RrInputState &state = program->inputState;
    if (index >= state.arrayOf.size()) {
        return _RrFail(error, "no input at index " + std::to_string(index) +
                                  "; the file lists " +
                                  std::to_string(state.arrayOf.size()));
    }
    RigExecRuntimeInputInfo info;
    info.name = program->TextOrEmpty(state.file->inputs[index].name());
    info.type = RrInputTag(uint8_t(state.file->inputs[index].type()));
    if (_RrIsArraySlot(state, index)) {
        const auto &a = state.arrays[size_t(state.arrayOf[index])];
        info.defaultCount = _RrVectorElements(a.tag, a.defaultVector).count;
    }
    if (!RrInputTagIsArray(info.type)) {
        return _RrFail(error, info.name + " is a " + _RrTagName(info.type) +
                                  " input, not an array");
    }
    if (value.tag != info.type) {
        return _RrFail(error, info.name + " is a " + _RrTagName(info.type) +
                                  " input, not a " + _RrTagName(value.tag));
    }
    if (value.count > 0 && !value.data) {
        return _RrFail(error, info.name + ": no elements to copy");
    }
    if (authored && value.count != info.defaultCount) {
        return _RrFail(error, info.name + " holds " +
                                  std::to_string(info.defaultCount) +
                                  " elements; an array set keeps that "
                                  "count, not " +
                                  std::to_string(value.count));
    }
    const uint32_t slot = uint32_t(index);
    RrArraySlot &a = state.arrays[size_t(state.arrayOf[slot])];
    const RigExecWireInputTag tag = a.tag;
    // Even a repeat writes the set's kind.
    RrInputsMarkWritten(state, slot);
    // Compared bit for bit: a repeat only takes the set's kind.
    if (state.slotHasValue[slot] &&
        _RrSameElements(tag, _RrArrayView(a), value.data, value.count)) {
        a.authored = authored;
        return true;
    }
    _RrTouchArray(state, slot, a);
    if (state.slotDefaultHasValue[slot] &&
        _RrSameElements(tag, a.defaultVector, value.data, value.count)) {
        a.holdsSet = false;
        a.held = RrArrayBuffer();
    } else {
        _RrAssign(tag, &a.held, value.data, value.count);
        a.holdsSet = true;
    }
    state.slotHasValue[slot] = 1;
    a.authored = authored;
    return true;
}

bool
RrInputsSetArray(RrProgram *program, size_t index,
                 const RigExecRuntimeArray &value, bool authored,
                 std::string *error)
{
    if (index >= program->inputState.inputInfo.size()) {
        return _RrFail(error, "no input at index " + std::to_string(index) +
                                  "; the file lists " +
                                  std::to_string(program->inputState.inputInfo.size()));
    }
    return _RrSetArraySlot(program, index, value, authored, error);
}

std::vector<size_t>
RrStageArraySlots(const RrProgram *program)
{
    return program->inputState.stageArraySlots;
}

bool
RrStageArraySet(RrProgram *program, size_t slot,
                const RigExecRuntimeArray &value, std::string *error)
{
    const auto &slots = program->inputState.stageArraySlots;
    if (!std::binary_search(slots.begin(), slots.end(), slot)) {
        return _RrFail(error, "no sampled array at slot " + std::to_string(slot));
    }
    return _RrSetArraySlot(program, slot, value, false, error);
}

bool RrStageTokenArraySet(RrProgram *program,size_t slot,
    const std::vector<std::string> &texts,std::string *error)
{
    auto &state=program->inputState;
    if(slot>=state.file->inputs.size() || state.file->inputs[slot].type()!=RigExecWireInputTag::TokenArray)
        return _RrFail(error,"sampled token array has incompatible slot type");
    std::vector<uint32_t> ids; ids.reserve(texts.size());
    for(const auto &text:texts) {
        auto found=state.tokenIds.find(text);
        if(found==state.tokenIds.end()) {
            const uint32_t id=state.extraTokenBase+uint32_t(state.extraTokens.size());
            state.extraTokens.push_back(text); found=state.tokenIds.emplace(text,id).first;
        }
        ids.push_back(found->second);
    }
    RigExecRuntimeArray value;
    value.tag=RrInputTag::TokenArray; value.data=ids.data(); value.count=ids.size();
    return RrStageArraySet(program,slot,value,error);
}

bool
RrStageArrayClear(RrProgram *program, size_t slot, std::string *error)
{
    const auto &slots = program->inputState.stageArraySlots;
    if (!std::binary_search(slots.begin(), slots.end(), slot)) {
        return _RrFail(error, "no sampled array at slot " + std::to_string(slot));
    }
    auto &state = program->inputState;
    RrInputsMarkWritten(state, slot);
    if (state.slotHasValue[slot]) {
        auto &array = state.arrays[size_t(state.arrayOf[slot])];
        _RrTouchArray(state, uint32_t(slot), array);
        array.holdsSet = false;
        array.held = RrArrayBuffer();
        array.authored = false;
        state.slotHasValue[slot] = 0;
    }
    return true;
}

bool
RrInputsGetArray(const RrProgram *program, size_t index,
                 RigExecRuntimeArray *out)
{
    const RrInputState &state = program->inputState;
    if (!out || index >= state.inputInfo.size() ||
        !_RrIsArraySlot(state, index)) {
        return false;
    }
    const RrArraySlot &a = state.arrays[size_t(state.arrayOf[index])];
    const _RrElements elements = _RrVectorElements(a.tag, _RrArrayView(a));
    out->tag = RrInputTag(uint8_t(a.tag));
    out->data = elements.data;
    out->count = elements.count;
    return true;
}

void
RrInputsBindArrayDefault(RrProgram *program, uint32_t slot,
                         const void *vector)
{
    RrInputState &state = program->inputState;
    if (_RrIsArraySlot(state, slot)) {
        state.arrays[size_t(state.arrayOf[slot])].defaultVector = vector;
    }
}

bool
RrInputsFinishArrays(RrProgram *program, std::string *error)
{
    RrInputState &state = program->inputState;
    for (size_t s = 0; s < state.arrayOf.size(); ++s) {
        if (state.arrayOf[s] < 0) {
            continue;
        }
        const RrArraySlot &a = state.arrays[size_t(state.arrayOf[s])];
        if (!a.defaultVector) {
            return _RrFail(error, "array input " +
                                      program->TextOrEmpty(
                                          state.file->inputs[s].name()) +
                                      " has no default");
        }
        if (s < state.inputInfo.size()) {
            state.inputInfo[s].defaultCount =
                _RrVectorElements(a.tag, a.defaultVector).count;
        }
    }
    return true;
}

const void *
RrInputArrayValue(const RrProgram *program, uint32_t slot,
                  RigExecWireInputTag tag)
{
    const RrInputState &state = program->inputState;
    if (!_RrIsArraySlot(state, slot) || !state.slotHasValue[slot]) {
        return nullptr;
    }
    const RrArraySlot &a = state.arrays[size_t(state.arrayOf[slot])];
    return a.tag == tag ? _RrArrayView(a) : nullptr;
}

const void *
RrInputArrayDefault(const RrProgram *program, uint32_t slot,
                    RigExecWireInputTag tag)
{
    const RrInputState &state = program->inputState;
    if (!_RrIsArraySlot(state, slot)) {
        return nullptr;
    }
    const RrArraySlot &a = state.arrays[size_t(state.arrayOf[slot])];
    return a.tag == tag ? a.defaultVector : nullptr;
}

bool
RrInputArrayAuthored(const RrProgram *program, uint32_t slot)
{
    const RrInputState &state = program->inputState;
    return _RrIsArraySlot(state, slot) &&
           state.arrays[size_t(state.arrayOf[slot])].authored;
}

uint64_t
RrInputArrayVersion(const RrProgram *program, uint32_t slot)
{
    const RrInputState &state = program->inputState;
    return _RrIsArraySlot(state, slot)
               ? state.arrays[size_t(state.arrayOf[slot])].version
               : 0;
}

bool
RrStageInputBlockedSet(RrProgram *program,size_t slot,bool blocked,std::string *error)
{
    auto &state=program->inputState;
    if(slot>=state.slotProviderSource.size() || !state.slotProviderSource[slot])
        return _RrFail(error,"not a declared provider raw source");
    RrInputsMarkWritten(state,slot);
    state.slotBlocked[slot]=blocked?1:0;
    return true;
}

bool
RrInputHasValue(const RrProgram *program, uint32_t slot)
{
    const RrInputState &state = program->inputState;
    return slot < state.slotHasValue.size() && state.slotHasValue[slot] != 0;
}

bool
RrInputDiffersFromDefault(const RrProgram *program, uint32_t slot)
{
    const RrInputState &state = program->inputState;
    if (slot >= state.slotHasValue.size()) {
        return false;
    }
    return _RrIsArraySlot(state, slot)
               ? _RrArrayDiffersFromDefault(state, slot)
               : _RrSlotDiffersFromDefault(state, slot);
}

bool
RrInputChangedSinceRun(const RrProgram *program, uint32_t slot)
{
    const RrInputState &state = program->inputState;
    return slot < state.slotChangedSinceRun.size() &&
           state.slotChangedSinceRun[slot] != 0;
}

int32_t
RrArrayReadSlot(const RrProgram *program, const RigExecWireInput &read)
{
    const RrInputState &state = program->inputState;
    if (read.mode == RigExecWireReadMode::Raw) {
        return !read.walk.empty() && state.slotHasValue[read.walk[0]]
                   ? int32_t(read.walk[0])
                   : -1;
    }
    for (size_t k = read.walk.size(); k-- > 0;) {
        if (state.slotHasValue[read.walk[k]]) {
            return int32_t(read.walk[k]);
        }
    }
    return -1;
}

const void *
RrReadExternalArray(const RrProgram *program,
                    const RigExecWireExternalDeclaredInput &declaration)
{
    const auto &read=*declaration.read;
    const auto &state=program->inputState;
    std::set<uint32_t> unavailableProducerPaths;
    const auto slotValue=[&](uint32_t slot)->const void * {
        if(unavailableProducerPaths.count(state.file->inputs[slot].name())) return nullptr;
        if(state.file->inputs[slot].type()!=read.tag) return nullptr;
        const bool authored=RrInputArrayAuthored(program,slot);
        if(declaration.time == fb::ExternalInputTime::AtDefault && !authored)
            return state.slotDefaultHasValue[slot] ? RrInputArrayDefault(program,slot,read.tag) : nullptr;
        return state.slotHasValue[slot] ? RrInputArrayValue(program,slot,read.tag) : nullptr;
    };
    if (declaration.bodyWalk && read.tag==RigExecWireInputTag::Vec3fArray &&
        declaration.flavour != fb::ExternalInputFlavour::Raw && declaration.flavour != fb::ExternalInputFlavour::Present) {
        bool computedCandidate=false;
        for (const auto &hop:declaration.bodyWalk->propertyCandidates) {
            if (hop.crossDomain < 0) continue;
            const auto &cross=program->file->crossDomainReads[size_t(hop.crossDomain)];
            if (cross.kind!=fb::CrossDomainReadKind::Points) continue;
            computedCandidate=true;
            if (const auto *points=RrReadCrossDomainPoints(program,cross)) return points;
            if(!cross.points.empty()) unavailableProducerPaths.insert(cross.source);
        }
        if (computedCandidate && declaration.flavour == fb::ExternalInputFlavour::ResolvedOnly)
            return declaration.fallbackHasValue ? _RrPoolVector(state,read.tag,state.file->values[read.constant].array) : nullptr;
    }
    const void *answer=nullptr;
    if(declaration.flavour != fb::ExternalInputFlavour::Present) {
        if(declaration.flavour == fb::ExternalInputFlavour::Raw) {
            if(!read.walk.empty()) answer=slotValue(read.walk.front());
        } else {
            for(size_t k=read.walk.size();k-- >0;) {
                answer=slotValue(read.walk[k]); if(answer) break;
            }
        }
    }
    if(!answer && declaration.fallbackHasValue)
        answer=_RrPoolVector(state,read.tag,state.file->values[read.constant].array);
    return answer;
}

const void *
RrArrayReadValue(const RrProgram *program, const RigExecWireInput &read,
                 RigExecWireInputTag tag)
{
    const RrInputState &state = program->inputState;
    if (read.tag != tag) {
        return nullptr;
    }
    const int32_t slot = RrArrayReadSlot(program, read);
    if (slot >= 0) {
        return RrInputArrayValue(program, uint32_t(slot), tag);
    }
    return read.walk.empty()
        ? _RrPoolVector(state, tag, state.file->values[read.constant].array)
        : nullptr;
}

const void *
RrPathArrayValue(const RrProgram *program, const RrPathRead &row,
                 RigExecWireInputTag tag)
{
    const RigExecWireInput *read = row.read ? row.read->read.get() : nullptr;
    if (!read || !RigExecFormatIsArrayTag(read->tag)) {
        return _RrPathValueArray(row.value, tag);
    }
    if (read->tag != tag) {
        return nullptr;
    }
    // A Default-time read takes an authored value only; a sampled one
    // stands at its own time.
    if (row.rest) {
        const uint32_t head = read->walk[0];
        return RrInputArrayAuthored(program, head)
                   ? RrInputArrayValue(program, head, tag)
                   : _RrPathValueArray(row.value, tag);
    }
    return RrArrayReadValue(program, *read, tag);
}

bool
RrInputsSetToken(RrProgram *program, size_t index, const std::string &text,
                 std::string *error)
{
    RrInputState &state = program->inputState;
    if (index >= state.inputInfo.size()) {
        return _RrFail(error, "no input at index " + std::to_string(index) +
                                  "; the file lists " +
                                  std::to_string(state.inputInfo.size()));
    }
    const RigExecRuntimeInputInfo &info = state.inputInfo[index];
    if (info.type != RrInputTag::Token) {
        return _RrFail(error, info.name + " is a " + _RrTagName(info.type) +
                                  " input, not a token");
    }
    uint32_t id = 0;
    const auto found = state.tokenIds.find(text);
    if (found != state.tokenIds.end()) {
        id = found->second;
    } else {
        id = state.extraTokenBase + uint32_t(state.extraTokens.size());
        state.extraTokens.push_back(text);
        state.tokenIds.emplace(text, id);
    }
    RrWireValue wire;
    wire.tag = RigExecWireInputTag::Token;
    wire.bits = id;
    state.slotAuthored[index] = 1;
    _RrStoreSlot(state, uint32_t(index), wire, true);
    return true;
}

bool
RrStageTokenSet(RrProgram *program, size_t slot, const std::string &text,
                bool hasValue, std::string *error)
{
    auto &state = program->inputState;
    if (!std::binary_search(state.stageTokenSlots.begin(),
                            state.stageTokenSlots.end(), slot))
        return _RrFail(error, "no sampled token at slot " + std::to_string(slot));
    uint32_t id = 0;
    if (hasValue) {
        const auto found = state.tokenIds.find(text);
        if (found != state.tokenIds.end()) id = found->second;
        else {
            id = state.extraTokenBase + uint32_t(state.extraTokens.size());
            state.extraTokens.push_back(text);
            state.tokenIds.emplace(text, id);
        }
    }
    RrWireValue value;
    value.tag = RigExecWireInputTag::Token;
    value.bits = id;
    state.slotAuthored[slot] = 0;
    _RrStoreSlot(state, uint32_t(slot), value, hasValue);
    return true;
}

void
RrInputsReset(RrProgram *program, size_t index)
{
    RrInputState &state = program->inputState;
    RrInputsMarkWritten(state, index);
    if (index < state.slotAuthored.size()) state.slotAuthored[index] = 0;
    if (index >= state.inputInfo.size()) {
        return;
    }
    if (_RrIsArraySlot(state, index)) {
        const uint32_t slot = uint32_t(index);
        RrArraySlot &a = state.arrays[size_t(state.arrayOf[slot])];
        if (_RrArrayDiffersFromDefault(state, slot)) {
            _RrTouchArray(state, slot, a);
            a.holdsSet = false;
            a.held = RrArrayBuffer();
            state.slotHasValue[slot] = state.slotDefaultHasValue[slot];
        }
        a.authored = false;
        return;
    }
    const RigExecWireInputSlot &slot = state.file->inputs[index];
    if ((state.slotProviderSource[index] & 2) != 0)
        state.slotBlocked[index] =
            (slot.flags() & uint8_t(RigExecWireInputSlotFlags::SourceBlocked)) != 0;
    _RrStoreSlot(state, uint32_t(index), state.values[slot.value()],
                 state.slotDefaultHasValue[index] != 0);
}

bool
RrInputsClear(RrProgram *program, size_t index, std::string *error)
{
    RrInputState &state = program->inputState;
    if (index >= state.inputInfo.size()) {
        return _RrFail(error, "no input at index " + std::to_string(index) +
                                  "; the file lists " +
                                  std::to_string(state.inputInfo.size()));
    }
    if (_RrIsArraySlot(state, index)) {
        return _RrFail(error, state.inputInfo[index].name +
                                  " is an array input; ResetInput restores "
                                  "it");
    }
    const RigExecWireInputSlot &slot = state.file->inputs[index];
    _RrStoreSlot(state, uint32_t(index), state.values[slot.value()], false);
    state.slotAuthored[index] = 0;
    return true;
}

void
RrInputsApplyTouched(RrProgram *program)
{
    RrInputState &state = program->inputState;
    RrStore &store = program->store;
    const std::vector<RigExecWireInputSlot> &slots = state.file->inputs;
    const uint8_t animated = uint8_t(RigExecWireInputSlotFlags::Animated);
    for (const uint32_t s : state.changedSlots) {
        state.slotChangedSinceRun[s] = 0;
    }
    state.changedSlots.clear();
    const auto record = [&state](uint32_t s) {
        state.slotChangedSinceRun[s] = 1;
        state.changedSlots.push_back(s);
    };
    for (const uint32_t s : state.touched) {
        state.touchedFlag[s] = 0;
        // An array changes when its elements or HasValue moved against
        // what the last run read. Its readers run every Execute or compare
        // by value, so it raises no override number and no time change.
        if (_RrIsArraySlot(state, s)) {
            RrArraySlot &a = state.arrays[size_t(state.arrayOf[s])];
            const void *ran =
                a.ranHeld ? _RrBufferVector(a.tag, a.ran) : a.defaultVector;
            const _RrElements before = _RrVectorElements(a.tag, ran);
            const bool moved = !_RrSameElements(a.tag, _RrArrayView(a),
                                                before.data, before.count);
            if (moved) {
                ++a.version;
            }
            const bool changed =
                state.slotHasValue[s] != state.slotRanHasValue[s] || moved;
            state.slotRanHasValue[s] = state.slotHasValue[s];
            a.ran = RrArrayBuffer();
            a.ranHeld = false;
            if (changed) {
                record(s);
            }
            continue;
        }
        const bool kindChanged = state.slotAuthored[s] != state.slotRanAuthored[s];
        state.slotRanAuthored[s] = state.slotAuthored[s];
        if ((slots[s].flags() & animated) != 0) {
            store.animatedTouched = true;
        }
        // Compared with what the last run read, not with the default: a
        // set that repeats the value moves nothing, and a reset after a
        // drag is a change like any other.
        const bool changed =
            kindChanged || state.slotHasValue[s] != state.slotRanHasValue[s] ||
            !_RrSameValueBits(state.slotCurrent[s], state.slotRan[s]);
        if (changed) {
            state.slotRan[s] = state.slotCurrent[s];
            state.slotRanHasValue[s] = state.slotHasValue[s];
            record(s);
        }
        for (uint32_t k = state.slotOverrideBegin[s];
             k < state.slotOverrideBegin[s + 1]; ++k) {
            const size_t number = size_t(state.slotOverrideNumbers[k]);
            if (changed && number < store.changedSinceRun.size()) {
                store.changedSinceRun[number] = 1;
                store.anyChangedSinceRun = true;
            }
            if (number >= state.valueOverridden.size() ||
                number + 1 >= state.overrideSlotBegin.size()) {
                continue;
            }
            bool differs = false;
            for (uint32_t j = state.overrideSlotBegin[number];
                 j < state.overrideSlotBegin[number + 1] && !differs; ++j) {
                const uint32_t t = state.overrideSlotList[j];
                // Sampled values also enable a static bound walk when
                // they differ from its captured raw default. Explicit
                // authored presence retains equal-default upstream routing.
                differs = (state.slotAuthored[t] && state.slotHasValue[t]) ||
                          _RrSlotDiffersFromDefault(state, t);
            }
            state.valueOverridden[number] = differs ? 1 : 0;
        }
    }
    state.touched.clear();
    if (store.overridden.size() == state.valueOverridden.size()) {
        store.overridden = state.valueOverridden;
    }
    store.anyOverridden =
        std::find(store.overridden.begin(), store.overridden.end(),
                  char(1)) != store.overridden.end();
}

bool
RrInputsPublished(const RrProgram *program, uint32_t path)
{
    return !program->store.propertyResults.empty() &&
           program->store.propertyResults.count(path) != 0;
}

bool
RrInputsOverlay(const RrProgram *program, uint32_t path,
                RigExecWireInputTag tag, RrWireValue *out)
{
    return _RrOverlay(*program, path, tag, out);
}

bool
RrChainBase(const RrProgram *program, size_t chain, RigExecWireInputTag tag,
            RrWireValue *base)
{
    const RrInputState &state = program->inputState;
    const RigExecWireFile &file = *state.file;
    const uint32_t target = file.propertyChains[chain].target;
    if (!state.slotHasValue[target] || file.inputs[target].type() != tag) {
        return false;
    }
    *base = _RrSlotValue(state, target);
    return true;
}

bool
RrPathValueFromWire(RigExecWireInputTag tag, const RrWireValue &value,
                    RrPathValue *out)
{
    using Tag = RrPathValue::Tag;
    switch (tag) {
    case RigExecWireInputTag::Bool:
        out->tag = Tag::Bool;
        out->boolean = value.bits != 0;
        return true;
    case RigExecWireInputTag::Int:
        out->tag = Tag::Int;
        out->i32 = int32_t(uint32_t(value.bits));
        return true;
    case RigExecWireInputTag::Float:
        out->tag = Tag::Float;
        out->f32 = RrWireValueFloat(value);
        return true;
    case RigExecWireInputTag::Double:
        out->tag = Tag::Double;
        out->f64 = _RrWireDouble(value);
        return true;
    case RigExecWireInputTag::Token:
        out->tag = Tag::Token;
        out->token = uint32_t(value.bits);
        return true;
    case RigExecWireInputTag::Matrix4d:
        out->tag = Tag::Matrix4d;
        out->matrix = value.matrix;
        return true;
    case RigExecWireInputTag::Vec3i:
        out->tag=Tag::Vec3i; out->vec3i=value.vec3i; return true;
    case RigExecWireInputTag::Vec3d:
        out->tag = Tag::Vec3d;
        out->vec = value.vec3d;
        return true;
    default:
        return false;
    }
}

bool RrPathValueFromRead(const RigExecWirePathRead &row,const RrWireValue &value,RrPathValue *out)
{
    return row.read && RrPathValueFromWire(row.read->tag,value,out);
}

namespace {

// Revision \p revision of chain \p chain, or derived target \p revision of
// it when \p derived; null when the file holds none.
const RigExecWireRevision *
_RrRevisionAt(const RrProgram *program, size_t chain, size_t revision,
              bool derived)
{
    const std::vector<RigExecWireChain> &chains = program->geometry->chains;
    if (chain >= chains.size()) {
        return nullptr;
    }
    if (derived) {
        return revision < chains[chain].derived.size()
                   ? chains[chain].derived[revision].revision.get()
                   : nullptr;
    }
    return revision < chains[chain].revisions.size()
               ? &chains[chain].revisions[revision]
               : nullptr;
}

}  // namespace

float
RrReadBlendWeight(const RrProgram *program, size_t chain, size_t revision,
                  bool derived, size_t channel)
{
    const RigExecWireRevision *wire =
        _RrRevisionAt(program, chain, revision, derived);
    if (!wire || channel >= wire->blendChannels.size() ||
        !wire->blendChannels[channel].weightRead) {
        return 0.0f;
    }
    const RigExecWireBlendChannel &bound = wire->blendChannels[channel];
    // The owning weight's exact producer takes precedence over the pose
    // channel. Publication path maps are output views, not body-read authority.
    bool held = false;
    for (const auto &candidate : bound.weightRead->propertyCandidates) {
        if (candidate.version < 0 ||
            program->file->inputs[candidate.slot].name() != bound.weightPath) continue;
        const size_t version = size_t(candidate.version);
        if (version >= program->store.propertyVersionValid.size() ||
            !program->store.propertyVersionValid[version]) continue;
        RrWireValue value;
        if (_RrPropertyWire(program->store.propertyVersions[version],
                            RigExecWireInputTag::Float, &value)) {
            held = true;
            break;
        }
    }
    if (bound.poseWeight >= 0 && !held) {
        const std::vector<float> &weights = program->store.poseWeights;
        return size_t(bound.poseWeight) < weights.size()
                   ? weights[size_t(bound.poseWeight)]
                   : 0.0f;
    }
    return RrWireValueFloat(RrReadInput(program, *bound.weightRead));
}

float
RrReadBlendActivation(const RrProgram *program, size_t chain,
                      size_t revision, bool derived, size_t channel,
                      size_t sample)
{
    const RigExecWireRevision *wire =
        _RrRevisionAt(program, chain, revision, derived);
    if (!wire || channel >= wire->blendChannels.size() ||
        sample >= wire->blendChannels[channel].samples.size() ||
        !wire->blendChannels[channel].samples[sample].activationRead) {
        return 1.0f;
    }
    return RrWireValueFloat(RrReadInput(
        program,
        *wire->blendChannels[channel].samples[sample].activationRead));
}

float
RrReadDefaultWeight(const RrProgram *program, size_t chain, size_t revision)
{
    const RigExecWireRevision *wire =
        _RrRevisionAt(program, chain, revision, false);
    if (!wire || !wire->defaultWeight) {
        return 1.0f;
    }
    return RrWireValueFloat(RrReadInput(program, *wire->defaultWeight));
}

}  // namespace rigExec
