// rigExecRuntime input slot model and read evaluation: a line port of
// RigExecResolvedInputs::GetAttribute (moverGraph.h), RigExecBakedRead
// (bakedProgramImpl.h) and _PinnedRead (rigEvaluatorProperties.cpp) over
// the Computed section's slot values. GetAttribute's overlay, the
// generation's property-chain results, is RrStore::propertyResults, keyed
// by the same attribute path ids as the slot names.
#include "rigExecRuntime/inputs.h"
#include "rigExecRuntime/store.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
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
_RrWireDouble(const v4::RigExecWireValue &value)
{
    double d = 0.0;
    std::memcpy(&d, &value.bits, sizeof(d));
    return d;
}

v4::RigExecWireValue
_RrFloatValue(float f)
{
    v4::RigExecWireValue value;
    value.tag = v4::InputTag::Float;
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    value.bits = bits;
    return value;
}

// RigExecResolvedInputs::Get<T>: the property-chain result published at
// \p path this run, when it holds exactly \p tag. \p out is written only
// on a hit.
bool
_RrOverlay(const RrStore &store, uint32_t path, v4::InputTag tag,
           v4::RigExecWireValue *out)
{
    if (store.propertyResults.empty()) {
        return false;
    }
    const auto found = store.propertyResults.find(path);
    if (found == store.propertyResults.end()) {
        return false;
    }
    const RrPropertyValue &held = found->second;
    using HeldTag = RrPropertyValue::Tag;
    v4::RigExecWireValue value;
    value.tag = tag;
    switch (tag) {
    case v4::InputTag::Float:
        if (held.tag != HeldTag::Float) {
            return false;
        }
        value = _RrFloatValue(held.f32);
        break;
    case v4::InputTag::Double:
        if (held.tag != HeldTag::Double) {
            return false;
        }
        std::memcpy(&value.bits, &held.f64, sizeof(held.f64));
        break;
    case v4::InputTag::Matrix4d:
        if (held.tag != HeldTag::Matrix4d) {
            return false;
        }
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                value.matrix[r * 4 + c] = held.matrix[r][c];
            }
        }
        break;
    case v4::InputTag::Vec3f:
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

// What answered a walk: the values[] id of the slot read, or a value held
// outside the pool (an overlay hit, or a double hop cast to float).
struct _RrWalkAnswer {
    uint32_t value = 0;
    bool direct = false;
    v4::RigExecWireValue wire;
};

// GetAttribute<T> from walk[from], hop for hop. At each hop the overlay
// answers first; a float read that meets a double hop reads a double walk
// from that hop and casts it (_CoerceFromDouble), and a failed double walk
// fails the read. GetAttribute<float> tests its own head for a double
// before consulting the overlay there. Otherwise the most upstream hop
// whose typed read succeeds answers, which is the walk's reverse scan.
bool
_RrLongWay(const RrProgram &program, const std::vector<uint32_t> &walk,
           size_t from, v4::InputTag tag, _RrWalkAnswer *answer)
{
    const RrInputState &state = program.inputState;
    const std::vector<v4::InputSlot> &slots = state.computed->inputs;
    const auto narrowFrom = [&](size_t k) {
        _RrWalkAnswer wide;
        if (!_RrLongWay(program, walk, k, v4::InputTag::Double, &wide)) {
            return false;
        }
        const double d =
            _RrWireDouble(wide.direct ? wide.wire
                                      : state.computed->values[wide.value]);
        answer->direct = true;
        answer->wire = _RrFloatValue(static_cast<float>(d));
        return true;
    };
    if (tag == v4::InputTag::Float && from < walk.size() &&
        slots[walk[from]].type == v4::InputTag::Double) {
        return narrowFrom(from);
    }
    for (size_t k = from; k < walk.size(); ++k) {
        const v4::InputSlot &slot = slots[walk[k]];
        if (_RrOverlay(program.store, slot.name, tag, &answer->wire)) {
            answer->direct = true;
            return true;
        }
        if (tag == v4::InputTag::Float &&
            slot.type == v4::InputTag::Double) {
            return narrowFrom(k);
        }
    }
    for (size_t k = walk.size(); k-- > from;) {
        const uint32_t slot = walk[k];
        if (state.slotHasValue[slot] && slots[slot].type == tag) {
            answer->value = state.slotValue[slot];
            return true;
        }
    }
    return false;
}

v4::RigExecWireValue
_RrWalkValue(const RrProgram &program, const v4::RigExecWireInput &input)
{
    const RrInputState &state = program.inputState;
    _RrWalkAnswer answer;
    if (!_RrLongWay(program, input.walk, 0, input.tag, &answer)) {
        return state.computed->values[input.constant];
    }
    return answer.direct ? answer.wire : state.computed->values[answer.value];
}

// The walk's first slot read raw: its own typed value when that read
// succeeded and has the read's type, else the read's constant.
v4::RigExecWireValue
_RrHeadValue(const RrInputState &state, const v4::RigExecWireInput &input)
{
    const v4::RigExecWireValue &fallback =
        state.computed->values[input.constant];
    if (input.walk.empty()) {
        return fallback;
    }
    const uint32_t slot = input.walk[0];
    if (state.slotHasValue[slot] &&
        state.computed->inputs[slot].type == input.tag) {
        return state.computed->values[state.slotValue[slot]];
    }
    return fallback;
}

// Every read a v4 weight object carries.
template <class Visit>
void
_RrForEachWeightRead(const v4::RigExecWireWeightObject &object,
                     const Visit &visit)
{
    for (const v4::RigExecWireInput *input :
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

float
RrWireValueFloat(const v4::RigExecWireValue &value)
{
    const uint32_t bits = uint32_t(value.bits);
    float f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

bool
RrSameFloatBits(float a, float b)
{
    return std::memcmp(&a, &b, sizeof(float)) == 0;
}

std::string
RrCrossCheckMismatch(const std::string &field, float computed, float recorded)
{
    uint32_t a = 0, b = 0;
    std::memcpy(&a, &computed, sizeof(a));
    std::memcpy(&b, &recorded, sizeof(b));
    char text[128];
    std::snprintf(text, sizeof(text),
                  ": computed %.9g (0x%08x), recorded %.9g (0x%08x)",
                  double(computed), unsigned(a), double(recorded),
                  unsigned(b));
    return "cross-check mismatch at " + field + text;
}

std::string
RrCrossCheckMismatch(const std::string &field, double computed,
                     double recorded)
{
    uint64_t a = 0, b = 0;
    std::memcpy(&a, &computed, sizeof(a));
    std::memcpy(&b, &recorded, sizeof(b));
    char text[160];
    std::snprintf(text, sizeof(text),
                  ": computed %.17g (0x%016llx), recorded %.17g (0x%016llx)",
                  computed, static_cast<unsigned long long>(a), recorded,
                  static_cast<unsigned long long>(b));
    return "cross-check mismatch at " + field + text;
}

std::string
RrCrossCheckMismatch(const std::string &field, const std::string &computed,
                     const std::string &recorded)
{
    return "cross-check mismatch at " + field + ": computed '" + computed +
           "', recorded '" + recorded + "'";
}

bool
RrInputsOpen(RrProgram *program, const RigExecWireComputed *computed,
             std::string *error)
{
    if (!program || !program->poses || !program->geometry) {
        return _RrFail(error, "the input slots need the pose and geometry "
                              "tables");
    }
    RrInputState &state = program->inputState;
    state = RrInputState();
    state.computed = computed;
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    // The constraint step's envelope arm (a weight object and no points
    // target) resolves through the oracle, and nothing else does.
    for (const RigExecWireConstraint &c : poses.constraints) {
        const bool envelope = c.weightObject != 0 && c.pointsTarget == 0;
        if (envelope && c.weightObjectIndex < 0) {
            return _RrFail(
                error,
                computed ? "constraint " + program->TextOrEmpty(c.path) +
                               " resolves an envelope the computed section "
                               "does not carry"
                         : "constraint " + program->TextOrEmpty(c.path) +
                               " resolves an envelope, which needs the "
                               "computed section this file does not carry; "
                               "rebake it");
        }
        if (!envelope && c.weightObjectIndex >= 0) {
            return _RrFail(error, "the computed section gives constraint " +
                                      program->TextOrEmpty(c.path) +
                                      " an envelope it does not resolve");
        }
        // The entry must be the constraint's own weight object (both are
        // ids into the one string table).
        if (envelope && computed &&
            (size_t(c.weightObjectIndex) >= computed->weightObjects.size() ||
             computed->weightObjects[size_t(c.weightObjectIndex)].path !=
                 c.weightObject)) {
            const std::string other =
                size_t(c.weightObjectIndex) < computed->weightObjects.size()
                    ? program->TextOrEmpty(
                          computed->weightObjects[size_t(c.weightObjectIndex)]
                              .path)
                    : std::string("no entry");
            return _RrFail(error, "the computed section gives constraint " +
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
            if (!computed) {
                return _RrFail(error,
                               program->TextOrEmpty(revision.moverPath) +
                                   " measures a current-phase weight, which "
                                   "needs the computed section this file "
                                   "does not carry; rebake it");
            }
            if (size_t(revision.weightObject) >=
                computed->weightObjects.size()) {
                return _RrFail(error,
                               program->TextOrEmpty(revision.moverPath) +
                                   " binds a weight object the computed "
                                   "section does not carry");
            }
        }
    }
    if (!computed) {
        return true;
    }
    // The oracle reads every scalar as a float (_ResolvedRead<float>).
    for (const v4::RigExecWireWeightObject &object : computed->weightObjects) {
        bool floats = true;
        _RrForEachWeightRead(object, [&](const v4::RigExecWireInput &input) {
            floats = floats && input.tag == v4::InputTag::Float;
        });
        if (!floats) {
            return _RrFail(error, "weight object " +
                                      program->TextOrEmpty(object.path) +
                                      " carries a read that is not a float");
        }
    }
    const size_t slots = computed->inputs.size();
    state.slotValue.resize(slots);
    state.slotHasValue.resize(slots);
    for (size_t s = 0; s < slots; ++s) {
        const v4::InputSlot &slot = computed->inputs[s];
        state.slotValue[s] = slot.value;
        state.slotHasValue[s] =
            (slot.flags & uint8_t(v4::InputSlotFlags::HasValue)) != 0 ? 1
                                                                       : 0;
    }
    return true;
}

bool
RrInputsSelectFrame(RrProgram *program, size_t index, std::string *error)
{
    RrInputState &state = program->inputState;
    if (!state.computed) {
        return true;
    }
    if (index >= state.computed->frames.size()) {
        return _RrFail(error, "the computed section holds no slot values "
                              "for the selected frame");
    }
    const RigExecWireComputedFrame &frame = state.computed->frames[index];
    state.slotValue.assign(frame.values.begin(), frame.values.end());
    state.slotHasValue.assign(frame.hasValue.begin(), frame.hasValue.end());
    return true;
}

v4::RigExecWireValue
RrReadInput(const RrProgram *program, const v4::RigExecWireInput &input)
{
    const RrInputState &state = program->inputState;
    if (!state.computed) {
        return v4::RigExecWireValue();
    }
    const v4::RigExecWireValue &fallback =
        state.computed->values[input.constant];
    switch (input.mode) {
    case v4::ReadMode::Baked: {
        // RigExecBakedRead: an override reads the long way; a fixed input
        // is its folded constant; a long-way input walks; otherwise the
        // query pinned at Build reads its own attribute, or the constant
        // when that read fails.
        const std::vector<char> &overridden = program->store.overridden;
        if (input.overrideIndex >= 0 &&
            size_t(input.overrideIndex) < overridden.size() &&
            overridden[size_t(input.overrideIndex)]) {
            return _RrWalkValue(*program, input);
        }
        if ((input.flags & uint8_t(v4::InputReadFlags::Varying)) == 0) {
            return fallback;
        }
        if ((input.flags & uint8_t(v4::InputReadFlags::LongWay)) != 0) {
            return _RrWalkValue(*program, input);
        }
        if (input.selected >= 0) {
            const uint32_t slot = input.walk[size_t(input.selected)];
            if (state.slotHasValue[slot] &&
                state.computed->inputs[slot].type == input.tag) {
                return state.computed->values[state.slotValue[slot]];
            }
        }
        return fallback;
    }
    case v4::ReadMode::Resolved:
        return _RrWalkValue(*program, input);
    case v4::ReadMode::Pinned: {
        // _PinnedRead: a connected input reads the long way; otherwise an
        // overlay at the attribute's own path holding exactly the read's
        // type answers before the attribute's own value.
        if (input.walk.size() > 1) {
            return _RrWalkValue(*program, input);
        }
        v4::RigExecWireValue overlay;
        if (!input.walk.empty() &&
            _RrOverlay(program->store,
                       state.computed->inputs[input.walk[0]].name, input.tag,
                       &overlay)) {
            return overlay;
        }
        return _RrHeadValue(state, input);
    }
    case v4::ReadMode::Raw:
        return _RrHeadValue(state, input);
    }
    return fallback;
}

v4::RigExecWireValue
RrReadPathScalar(const RrProgram *program,
                 const RigExecWirePathScalarRead &entry)
{
    const RrInputState &state = program->inputState;
    if (!state.computed) {
        return v4::RigExecWireValue();
    }
    if (!entry.headFallback || entry.read.mode != v4::ReadMode::Resolved) {
        return RrReadInput(program, entry.read);
    }
    // moverGraph.cpp's _Read: `if (!GetAttribute) attr.Get`, so a walk
    // that yields nothing still reads the head before the fallback.
    _RrWalkAnswer answer;
    if (_RrLongWay(*program, entry.read.walk, 0, entry.read.tag, &answer)) {
        return answer.direct ? answer.wire
                             : state.computed->values[answer.value];
    }
    return _RrHeadValue(state, entry.read);
}

float
RrReadResolvedFloat(const RrProgram *program,
                    const v4::RigExecWireInput &input, float fallback)
{
    const RrInputState &state = program->inputState;
    if (!state.computed) {
        return fallback;
    }
    _RrWalkAnswer answer;
    if (!_RrLongWay(*program, input.walk, 0, v4::InputTag::Float, &answer)) {
        return fallback;
    }
    return RrWireValueFloat(answer.direct
                                ? answer.wire
                                : state.computed->values[answer.value]);
}

namespace {

using _RrFamily = RigExecWireRegisteredFamily;

// The record field names a registered read's table and field go by.
const char *const _RrFamilyNames[] = {
    "avarBindings", "avarConstantBindings", "ladders", "spaceSwitches",
    "poseInterpolators", "solvers", "constraints", "weightObjects"};
const char *const _RrSolverFieldNames[RrSolverFieldCount] = {
    "bend",          "upperOffset",     "lowerOffset",    "stretch",
    "softness",      "blendWeight",     "preserveVolume", "midFollowWeight",
    "roll",          "twist",           "minLengthRatio", "twistTurns",
    "ribbonSampleCount", "ikSpace"};
const char *const _RrConstraintFieldNames[RrConstraintFieldCount] = {
    "enabled",   "defaultWeight",  "offset",        "affectX",
    "affectY",   "affectZ",        "tX",            "tY",
    "tZ",        "rX",             "rY",            "rZ",
    "sX",        "sY",             "sZ",            "aimVector",
    "upVector",  "rotationOffset", "worldUpVector", "poleVector",
    "twistDegrees"};
const char *const _RrWeightFieldNames[RrWeightFieldCount] = {
    "defaultWeight", "driver",     "scale",      "bias",      "strength",
    "invert",        "falloffMin", "falloffMax", "scaleXPos", "scaleYPos",
    "scaleZPos",     "scaleXNeg",  "scaleYNeg",  "scaleZNeg", "scaleX",
    "scaleY",        "scaleZ",     "extentU",    "extentV"};

// "solvers[2].ikSpace (uid 7, /Rig/Solvers/Ik.rigExec:spaceMatrix)".
std::string
_RrRegisteredField(const RrProgram &program,
                   const RigExecWireRegisteredRead &entry)
{
    std::string field = std::string(_RrFamilyNames[size_t(entry.family)]) +
                        "[" + std::to_string(entry.object) + "]";
    const uint32_t f = entry.field;
    switch (entry.family) {
    case _RrFamily::AvarBinding:
    case _RrFamily::AvarConstantBinding:
        field += ".input";
        break;
    case _RrFamily::Ladder:
        if (f == RrLadderRestSpace) {
            field += ".restSpace";
        } else if (f == RrLadderDefaultSpace) {
            field += ".defaultSpace";
        } else if (f == RrLadderPosedSpace) {
            field += ".posedSpace";
        } else if (f >= RrLadderRestAvar0 && f < RrLadderRestAvar0 + 6) {
            field += ".restAvars[" + std::to_string(f - RrLadderRestAvar0) +
                     "]";
        } else if (f >= RrLadderDefaultAvar0 &&
                   f < RrLadderDefaultAvar0 + 6) {
            field += ".defaultAvars[" +
                     std::to_string(f - RrLadderDefaultAvar0) + "]";
        } else {
            field += ".rotationOrder";
        }
        break;
    case _RrFamily::SpaceSwitch:
        field += ".active";
        break;
    case _RrFamily::Interpolator:
        field += f == 0 ? std::string(".enabled")
                        : ".valueInputs[" + std::to_string(f - 1) + "]";
        break;
    case _RrFamily::Solver:
        field += std::string(".") + _RrSolverFieldNames[f];
        break;
    case _RrFamily::Constraint:
        field += std::string(".") + _RrConstraintFieldNames[f];
        break;
    case _RrFamily::WeightObject:
        field += std::string(".") + _RrWeightFieldNames[f];
        break;
    }
    const RigExecWireComputed &computed = *program.inputState.computed;
    const std::string head =
        entry.read.walk.empty()
            ? std::string("no head")
            : program.TextOrEmpty(computed.inputs[entry.read.walk[0]].name);
    return field + " (uid " + std::to_string(entry.uid) + ", " + head + ")";
}

bool
_RrSameDouble(double a, double b)
{
    return std::memcmp(&a, &b, sizeof(double)) == 0;
}

// Bitwise equality of a computed read and the value a record-driven step
// consumes, on the member the tag names; false with the mismatch text.
// \p field() names the value and is called only on a mismatch.
template <class Field>
bool
_RrSameAsUsed(const RrProgram &program, const v4::RigExecWireValue &computed,
              const RrInputValue &used, const Field &field, std::string *why)
{
    if (uint8_t(computed.tag) != uint8_t(used.tag)) {
        *why = RrCrossCheckMismatch(field() + ".tag",
                                    std::to_string(int(computed.tag)),
                                    std::to_string(int(used.tag)));
        return false;
    }
    switch (computed.tag) {
    case v4::InputTag::Double: {
        const double value = _RrWireDouble(computed);
        if (!_RrSameDouble(value, used.f64)) {
            *why = RrCrossCheckMismatch(field(), value, used.f64);
            return false;
        }
        return true;
    }
    case v4::InputTag::Float: {
        const float value = RrWireValueFloat(computed);
        if (!RrSameFloatBits(value, used.f32)) {
            *why = RrCrossCheckMismatch(field(), value, used.f32);
            return false;
        }
        return true;
    }
    case v4::InputTag::Bool:
        if ((computed.bits != 0) != used.boolean) {
            *why = RrCrossCheckMismatch(
                field(), std::string(computed.bits != 0 ? "true" : "false"),
                std::string(used.boolean ? "true" : "false"));
            return false;
        }
        return true;
    case v4::InputTag::Int: {
        const int32_t value = int32_t(uint32_t(computed.bits));
        if (value != used.i32) {
            *why = RrCrossCheckMismatch(field(), std::to_string(value),
                                        std::to_string(used.i32));
            return false;
        }
        return true;
    }
    case v4::InputTag::Matrix4d:
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                if (!_RrSameDouble(computed.matrix[r * 4 + c],
                                   used.matrix[r][c])) {
                    *why = RrCrossCheckMismatch(
                        field() + ".matrix[" + std::to_string(r) + "][" +
                            std::to_string(c) + "]",
                        computed.matrix[r * 4 + c], used.matrix[r][c]);
                    return false;
                }
            }
        }
        return true;
    case v4::InputTag::Token:
        if (uint32_t(computed.bits) != used.token) {
            *why = RrCrossCheckMismatch(
                field(), program.TextOrEmpty(uint32_t(computed.bits)),
                program.TextOrEmpty(used.token));
            return false;
        }
        return true;
    case v4::InputTag::Vec3d:
        for (size_t i = 0; i < 3; ++i) {
            if (!_RrSameDouble(computed.vec3d[i], used.vec[i])) {
                *why = RrCrossCheckMismatch(
                    field() + ".vec[" + std::to_string(i) + "]",
                    computed.vec3d[i], used.vec[i]);
                return false;
            }
        }
        return true;
    case v4::InputTag::Vec3f:
        break;
    }
    *why = RrCrossCheckMismatch(field() + ".tag", std::string("vec3f"),
                                std::string("no record form"));
    return false;
}

// The record tag a scalar path read is stored under: its own type, or a
// double for a site that widens a float read.
bool
_RrPathTag(const RigExecWirePathScalarRead &entry,
           RigExecWirePathValue::Tag *tag)
{
    using Tag = RigExecWirePathValue::Tag;
    if (entry.widen) {
        *tag = Tag::Double;
        return true;
    }
    switch (entry.read.tag) {
    case v4::InputTag::Bool:
        *tag = Tag::Bool;
        return true;
    case v4::InputTag::Int:
        *tag = Tag::Int;
        return true;
    case v4::InputTag::Float:
        *tag = Tag::Float;
        return true;
    case v4::InputTag::Double:
        *tag = Tag::Double;
        return true;
    case v4::InputTag::Token:
        *tag = Tag::Token;
        return true;
    case v4::InputTag::Matrix4d:
        *tag = Tag::Matrix4d;
        return true;
    case v4::InputTag::Vec3d:
        *tag = Tag::Vec3d;
        return true;
    default:
        return false;
    }
}

// Bitwise equality of a computed path read and the recorded stage value;
// \p field() as for _RrSameAsUsed.
template <class Field>
bool
_RrSameAsRecorded(const RrProgram &program,
                  const RigExecWirePathScalarRead &entry,
                  const v4::RigExecWireValue &computed,
                  const RigExecWirePathValue &recorded, const Field &field,
                  std::string *why)
{
    using Tag = RigExecWirePathValue::Tag;
    Tag tag = Tag::Absent;
    if (!_RrPathTag(entry, &tag) || recorded.tag != tag) {
        *why = RrCrossCheckMismatch(field() + ".tag",
                                    std::to_string(int(tag)),
                                    std::to_string(int(recorded.tag)));
        return false;
    }
    if (entry.widen) {
        const double value = double(RrWireValueFloat(computed));
        if (!_RrSameDouble(value, recorded.f64)) {
            *why = RrCrossCheckMismatch(field(), value, recorded.f64);
            return false;
        }
        return true;
    }
    RrInputValue as;
    switch (tag) {
    case Tag::Bool:
        as.tag = RigExecWireInput::Tag::Bool;
        as.boolean = recorded.boolean;
        break;
    case Tag::Int:
        as.tag = RigExecWireInput::Tag::Int;
        as.i32 = recorded.i32;
        break;
    case Tag::Float:
        as.tag = RigExecWireInput::Tag::Float;
        as.f32 = recorded.f32;
        break;
    case Tag::Double:
        as.tag = RigExecWireInput::Tag::Double;
        as.f64 = recorded.f64;
        break;
    case Tag::Token:
        as.tag = RigExecWireInput::Tag::Token;
        as.token = recorded.token;
        break;
    case Tag::Matrix4d:
        as.tag = RigExecWireInput::Tag::Matrix4d;
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                as.matrix[r][c] = recorded.matrix[r * 4 + c];
            }
        }
        break;
    case Tag::Vec3d:
        as.tag = RigExecWireInput::Tag::Vec3d;
        as.vec = RrVec3d(recorded.vec[0], recorded.vec[1], recorded.vec[2]);
        break;
    default:
        break;
    }
    return _RrSameAsUsed(program, computed, as, field, why);
}

}  // namespace

bool
RrInputsBindReads(RrProgram *program, std::string *error)
{
    RrInputState &state = program->inputState;
    state.registered.clear();
    state.defaultWeightRead.clear();
    state.blendReads.clear();
    state.derivedBlendReads.clear();
    state.pathReadIndex.clear();
    const RigExecWireComputed *computed = state.computed;
    if (!computed) {
        return true;
    }
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    const RrStore &store = program->store;
    std::vector<char> chainUid(store.inputHolders.size(), 0);
    for (const RigExecWireChainRead &entry : computed->chainReads) {
        if (entry.uid < chainUid.size()) {
            chainUid[entry.uid] = 1;
        }
    }
    state.registered.reserve(computed->registeredReads.size());
    for (size_t k = 0; k < computed->registeredReads.size(); ++k) {
        const RigExecWireRegisteredRead &entry = computed->registeredReads[k];
        const auto fail = [&](const std::string &why) {
            return _RrFail(error, "the computed section's registered read " +
                                      std::to_string(k) + " " + why);
        };
        RrInputState::BoundRead bound;
        bound.uid = entry.uid;
        bound.avar = entry.avar;
        const uint8_t flags = entry.read.flags;
        bound.live =
            (flags & uint8_t(v4::InputReadFlags::Varying)) != 0 &&
            ((flags & uint8_t(v4::InputReadFlags::LongWay)) != 0 ||
             entry.read.selected >= 0);
        if (entry.uid >= 0 &&
            size_t(entry.uid) >= store.inputHolders.size()) {
            return fail("names no input holder");
        }
        const size_t object = entry.object;
        const uint32_t field = entry.field;
        int32_t routed = -1;
        switch (entry.family) {
        case _RrFamily::AvarBinding:
        case _RrFamily::AvarConstantBinding:
            if (entry.avar < 0 || size_t(entry.avar) >= store.avars.size()) {
                return fail("names no avar");
            }
            if (entry.uid >= 0 &&
                (size_t(entry.uid) >= program->avarUidTarget.size() ||
                 program->avarUidTarget[size_t(entry.uid)] != entry.avar)) {
                return fail("does not route to its avar");
            }
            break;
        case _RrFamily::Ladder:
            if (object >= poses.ladders.size() ||
                field >= uint32_t(RrLadderFieldCount)) {
                return fail("names no ladder field");
            }
            bound.wire = &program->LadderInput(object, int(field));
            routed = program->ladderUid[object][field];
            break;
        case _RrFamily::SpaceSwitch:
            if (object >= poses.spaceSwitches.size() || field != 0) {
                return fail("names no space switch");
            }
            bound.wire = &poses.spaceSwitches[object].active;
            routed = program->spaceSwitchUid[object];
            break;
        case _RrFamily::Interpolator:
            if (object >= poses.poseInterpolators.size() ||
                field > poses.poseInterpolators[object].valueInputs.size() ||
                field > 3) {
                return fail("names no interpolator input");
            }
            if (field == 0) {
                bound.wire = &poses.poseInterpolators[object].enabled;
                routed = program->interpUid[object];
            } else {
                bound.wire =
                    &poses.poseInterpolators[object].valueInputs[field - 1];
                routed = program->interpValueUid[object][field - 1];
            }
            break;
        case _RrFamily::Solver:
            if (object >= poses.solvers.size() ||
                field >= uint32_t(RrSolverFieldCount)) {
                return fail("names no solver field");
            }
            bound.wire = &program->SolverInput(object, int(field));
            routed = program->solverUid[object][field];
            break;
        case _RrFamily::Constraint:
            if (object >= poses.constraints.size() ||
                field >= uint32_t(RrConstraintFieldCount)) {
                return fail("names no constraint field");
            }
            bound.wire = &program->ConstraintInput(object, int(field));
            routed = program->constraintUid[object][field];
            break;
        case _RrFamily::WeightObject:
            if (object >= geometry.weightObjects.size() ||
                field >= uint32_t(RrWeightFieldCount)) {
                return fail("names no weight object field");
            }
            bound.wire = &program->WeightInput(object, int(field));
            routed = program->weightUid[object][field];
            break;
        }
        if (bound.wire) {
            if (routed != entry.uid) {
                return fail("carries uid " + std::to_string(entry.uid) +
                            " where the uid replay routes " +
                            std::to_string(routed));
            }
            if (uint8_t(bound.wire->tag) != uint8_t(entry.read.tag) ||
                bound.wire->overrideIndex != entry.read.overrideIndex ||
                (bound.wire->varying && bound.wire->bound) != bound.live) {
                return fail("differs from the table input it names");
            }
        }
        bound.chainRead = entry.uid >= 0 && chainUid[size_t(entry.uid)] != 0;
        state.registered.push_back(bound);
    }
    state.defaultWeightRead.resize(geometry.chains.size());
    state.blendReads.resize(geometry.chains.size());
    state.derivedBlendReads.resize(geometry.chains.size());
    for (size_t c = 0; c < geometry.chains.size(); ++c) {
        state.defaultWeightRead[c].assign(geometry.chains[c].revisions.size(),
                                          -1);
        state.blendReads[c].resize(geometry.chains[c].revisions.size());
        state.derivedBlendReads[c].resize(geometry.chains[c].derived.size());
    }
    // RigExecWireApplyComputed checked that each geometry read names a
    // revision the geometry section holds.
    for (size_t k = 0; k < computed->defaultWeightReads.size(); ++k) {
        const RigExecWireDefaultWeightRead &entry =
            computed->defaultWeightReads[k];
        state.defaultWeightRead[entry.chain][entry.revision] = int32_t(k);
    }
    for (size_t k = 0; k < computed->blendWeightReads.size(); ++k) {
        const RigExecWireBlendWeightRead &entry =
            computed->blendWeightReads[k];
        (entry.derived ? state.derivedBlendReads
                       : state.blendReads)[entry.chain][entry.revision]
            .push_back(uint32_t(k));
    }
    // The decoder refused two path reads at one path.
    state.pathReadIndex.reserve(computed->pathScalarReads.size());
    for (size_t k = 0; k < computed->pathScalarReads.size(); ++k) {
        state.pathReadIndex.emplace_back(computed->pathScalarReads[k].path,
                                         uint32_t(k));
    }
    std::sort(state.pathReadIndex.begin(), state.pathReadIndex.end());
    return true;
}

bool
RrCrossCheckReads(const RrProgram *program,
                  const RigExecWireFrameInputs &record,
                  RrCrossCheckCounts *counts, std::string *error)
{
    const RrInputState &state = program->inputState;
    const RigExecWireComputed *computed = state.computed;
    if (!computed) {
        return true;
    }
    const RrStore &store = program->store;
    uint64_t registered = 0;
    for (size_t k = 0; k < state.registered.size(); ++k) {
        const RrInputState::BoundRead &bound = state.registered[k];
        if (bound.chainRead) {
            continue;
        }
        // A read made per run is in the record only at the frames a step
        // read it, which are the frames its holder holds this frame's
        // value.
        if (bound.live && !std::binary_search(record.uids.begin(),
                                              record.uids.end(),
                                              uint32_t(bound.uid))) {
            continue;
        }
        const RigExecWireRegisteredRead &entry = computed->registeredReads[k];
        const v4::RigExecWireValue value = RrReadInput(program, entry.read);
        RrInputValue used;
        if (bound.wire) {
            used = program->ReadUid(*bound.wire, bound.uid);
        } else {
            used.tag = RigExecWireInput::Tag::Double;
            used.f64 = bound.uid >= 0
                           ? store.inputHolders[size_t(bound.uid)].f64
                           : program->constants
                                 ->avarConstants[size_t(bound.avar)];
        }
        std::string why;
        if (!_RrSameAsUsed(
                *program, value, used,
                [&] { return _RrRegisteredField(*program, entry); }, &why)) {
            return _RrFail(error, "registered reads: " + why);
        }
        ++registered;
    }
    // This frame's forced live reads (the record holds one entry per
    // (path, wasDefault)), each against the path read at its head.
    uint64_t paths = 0;
    const std::vector<std::pair<uint32_t, uint32_t>> &index =
        state.pathReadIndex;
    for (const RigExecWirePathRead &read : record.pathReads) {
        if (index.empty() || read.wasDefault != 0 || read.forceFrame == 0) {
            continue;
        }
        const auto found = std::lower_bound(
            index.begin(), index.end(), std::make_pair(read.path, 0u));
        if (found == index.end() || found->first != read.path) {
            continue;
        }
        const RigExecWirePathScalarRead &entry =
            computed->pathScalarReads[found->second];
        const v4::RigExecWireValue value = RrReadPathScalar(program, entry);
        std::string why;
        if (!_RrSameAsRecorded(
                *program, entry, value, read.value,
                [&] {
                    return "pathReads[" + program->TextOrEmpty(entry.path) +
                           "]";
                },
                &why)) {
            return _RrFail(error, "path reads: " + why);
        }
        ++paths;
    }
    (*counts)[RrCrossCheckRegisteredRead] += registered;
    (*counts)[RrCrossCheckPathRead] += paths;
    return true;
}

bool
RrCrossCheckRevisionReads(const RrProgram *program,
                          const RigExecWireFrameInputs *record, size_t chain,
                          size_t revision, bool derived, RrStepOutput *output,
                          std::string *error)
{
    const RrInputState &state = program->inputState;
    const RigExecWireComputed *computed = state.computed;
    if (!computed || !record) {
        return true;
    }
    // "[c][r]" and the field names are built only to report a failure.
    const auto at = [&] {
        return "[" + std::to_string(chain) + "][" + std::to_string(revision) +
               "]";
    };
    const std::vector<std::vector<std::vector<uint32_t>>> &blends =
        derived ? state.derivedBlendReads : state.blendReads;
    if (chain < blends.size() && revision < blends[chain].size() &&
        !blends[chain][revision].empty()) {
        const auto &weights =
            derived ? record->derivedBlendWeights : record->blendWeights;
        // RigExecWireApplyComputed checked each read's revision and channel.
        const RigExecWireChain &wireChain = program->geometry->chains[chain];
        const RigExecWireRevision &wire =
            derived ? wireChain.derived[revision].revision
                    : wireChain.revisions[revision];
        const RrStore &store = program->store;
        for (uint32_t k : blends[chain][revision]) {
            const RigExecWireBlendWeightRead &entry =
                computed->blendWeightReads[k];
            const auto field = [&] {
                return std::string(derived ? "derivedBlendWeights"
                                           : "blendWeights") +
                       at() + "[" + std::to_string(entry.channel) + "]";
            };
            if (chain >= weights.size() ||
                revision >= weights[chain].size() ||
                entry.channel >= weights[chain][revision].size()) {
                return _RrFail(error, "the frame record holds no " + field());
            }
            // The assembly's channel weight: a pose-driven channel reads its
            // pose weight slot unless a value is published at the weight
            // itself (bakedGeometry.cpp's `!R.Find(weightPath)`).
            const RigExecWireBlendChannel &bound =
                wire.blendChannels[entry.channel];
            float value = 0.0f;
            if (bound.poseWeight >= 0 &&
                store.propertyResults.find(bound.weightPath) ==
                    store.propertyResults.end()) {
                if (size_t(bound.poseWeight) >= store.poseWeights.size()) {
                    return _RrFail(error, field() + " names no pose weight");
                }
                value = store.poseWeights[size_t(bound.poseWeight)];
            } else {
                value = RrWireValueFloat(RrReadInput(program, entry.read));
            }
            const float recorded = weights[chain][revision][entry.channel];
            if (!RrSameFloatBits(value, recorded)) {
                return _RrFail(error,
                               RrCrossCheckMismatch(field(), value, recorded));
            }
            ++output->crossChecked[RrCrossCheckBlendWeight];
        }
    }
    if (!derived && chain < state.defaultWeightRead.size() &&
        revision < state.defaultWeightRead[chain].size() &&
        state.defaultWeightRead[chain][revision] >= 0) {
        const RigExecWireDefaultWeightRead &entry =
            computed->defaultWeightReads[size_t(
                state.defaultWeightRead[chain][revision])];
        const auto field = [&] { return "revisionDefaultWeights" + at(); };
        if (chain >= record->revisionDefaultWeights.size() ||
            revision >= record->revisionDefaultWeights[chain].size()) {
            return _RrFail(error, "the frame record holds no " + field());
        }
        const float value = RrWireValueFloat(RrReadInput(program, entry.read));
        const float recorded = record->revisionDefaultWeights[chain][revision];
        if (!RrSameFloatBits(value, recorded)) {
            return _RrFail(error,
                           RrCrossCheckMismatch(field(), value, recorded));
        }
        ++output->crossChecked[RrCrossCheckDefaultWeight];
    }
    return true;
}

}  // namespace rigExec
