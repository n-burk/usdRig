// rigExecRuntime input slot model and read evaluation: a line port of
// RigExecResolvedInputs::GetAttribute (moverGraph.h), RigExecBakedRead
// (bakedProgramImpl.h) and _PinnedRead (rigEvaluatorProperties.cpp) over
// the slot values, plus the input API's writes to them. GetAttribute's
// overlay holds the generation's property-chain results
// (RrStore::propertyResults), keyed by the same attribute path ids as the
// slot names.
#include "rigExecRuntime/inputs.h"
#include "rigExecRuntime/store.h"

#include <algorithm>
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

// The overlay's value at \p path: the chain result published there, when
// it is exactly \p tag.
bool
_RrOverlay(const RrProgram &program, uint32_t path, v4::InputTag tag,
           v4::RigExecWireValue *out)
{
    const RrStore &store = program.store;
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

// Slot \p slot's value this run: the one place a read meets the slots.
const v4::RigExecWireValue &
_RrSlotValue(const RrInputState &state, uint32_t slot)
{
    return state.slotCurrent[slot];
}

// What answered a walk: a slot's value, an overlay hit, or a double hop
// cast to float.
struct _RrWalkAnswer {
    v4::RigExecWireValue value;
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
        const double d = _RrWireDouble(wide.value);
        answer->value = _RrFloatValue(static_cast<float>(d));
        return true;
    };
    if (tag == v4::InputTag::Float && from < walk.size() &&
        slots[walk[from]].type == v4::InputTag::Double) {
        return narrowFrom(from);
    }
    for (size_t k = from; k < walk.size(); ++k) {
        const v4::InputSlot &slot = slots[walk[k]];
        if (_RrOverlay(program, slot.name, tag, &answer->value)) {
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
            answer->value = _RrSlotValue(state, slot);
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
    return answer.value;
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
        return _RrSlotValue(state, slot);
    }
    return fallback;
}

// Bitwise equality of two values of one tag, on the member the tag names.
bool
_RrSameValueBits(const v4::RigExecWireValue &a, const v4::RigExecWireValue &b)
{
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case v4::InputTag::Matrix4d:
        return std::memcmp(a.matrix.data(), b.matrix.data(),
                           sizeof(a.matrix)) == 0;
    case v4::InputTag::Vec3d:
        return std::memcmp(a.vec3d.data(), b.vec3d.data(),
                           sizeof(a.vec3d)) == 0;
    case v4::InputTag::Vec3f:
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
           !_RrSameValueBits(
               state.slotCurrent[slot],
               state.computed->values[state.computed->inputs[slot].value]);
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
RrInputsOpen(RrProgram *program, const RigExecWireComputed *computed,
             std::string *error)
{
    if (!program || !program->poses || !program->geometry) {
        return _RrFail(error, "the input slots need the pose and geometry "
                              "tables");
    }
    RrInputState &state = program->inputState;
    state = RrInputState();
    // Every input a step reads is evaluated over the section's slots.
    if (!computed) {
        return _RrFail(error, "the file carries no computed section, which "
                              "every input read needs; rebake it");
    }
    state.computed = computed;
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    // The constraint step's envelope arm (a weight object and no points
    // target) resolves through the oracle, and nothing else does.
    for (const RigExecWireConstraint &c : poses.constraints) {
        const bool envelope = c.weightObject != 0 && c.pointsTarget == 0;
        if (envelope && c.weightObjectIndex < 0) {
            return _RrFail(error, "constraint " +
                                      program->TextOrEmpty(c.path) +
                                      " resolves an envelope the computed "
                                      "section does not carry");
        }
        if (!envelope && c.weightObjectIndex >= 0) {
            return _RrFail(error, "the computed section gives constraint " +
                                      program->TextOrEmpty(c.path) +
                                      " an envelope it does not resolve");
        }
        // The entry must be the constraint's own weight object (both are
        // ids into the one string table).
        if (envelope &&
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
            if (size_t(revision.weightObject) >=
                computed->weightObjects.size()) {
                return _RrFail(error,
                               program->TextOrEmpty(revision.moverPath) +
                                   " binds a weight object the computed "
                                   "section does not carry");
            }
        }
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
    state.slotCurrent.resize(slots);
    state.slotHasValue.resize(slots);
    state.slotDefaultHasValue.resize(slots);
    for (size_t s = 0; s < slots; ++s) {
        const v4::InputSlot &slot = computed->inputs[s];
        state.slotCurrent[s] = computed->values[slot.value];
        state.slotDefaultHasValue[s] =
            (slot.flags & uint8_t(v4::InputSlotFlags::HasValue)) != 0 ? 1
                                                                       : 0;
        state.slotHasValue[s] = state.slotDefaultHasValue[s];
    }
    state.touchedFlag.assign(slots, 0);
    return true;
}

v4::RigExecWireValue
RrReadInput(const RrProgram *program, const v4::RigExecWireInput &input)
{
    const RrInputState &state = program->inputState;
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
                return _RrSlotValue(state, slot);
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
            _RrOverlay(*program, state.computed->inputs[input.walk[0]].name,
                       input.tag, &overlay)) {
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
    if (!entry.headFallback || entry.read.mode != v4::ReadMode::Resolved) {
        return RrReadInput(program, entry.read);
    }
    // moverGraph.cpp's _Read: `if (!GetAttribute) attr.Get`, so a walk
    // that yields nothing still reads the head before the fallback.
    _RrWalkAnswer answer;
    if (_RrLongWay(*program, entry.read.walk, 0, entry.read.tag, &answer)) {
        return answer.value;
    }
    return _RrHeadValue(state, entry.read);
}

float
RrReadResolvedFloat(const RrProgram *program,
                    const v4::RigExecWireInput &input, float fallback)
{
    _RrWalkAnswer answer;
    if (!_RrLongWay(*program, input.walk, 0, v4::InputTag::Float, &answer)) {
        return fallback;
    }
    return RrWireValueFloat(answer.value);
}

namespace {

using _RrFamily = RigExecWireRegisteredFamily;

// Whether two reads resolve alike: every field of the read.
bool
_RrSameRead(const v4::RigExecWireInput &a, const v4::RigExecWireInput &b)
{
    return a.tag == b.tag && a.mode == b.mode && a.flags == b.flags &&
           a.overrideIndex == b.overrideIndex && a.constant == b.constant &&
           a.walk == b.walk && a.selected == b.selected;
}

}  // namespace

bool
RrInputsBindReads(RrProgram *program, std::string *error)
{
    RrInputState &state = program->inputState;
    const RigExecWireComputed *computed = state.computed;
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const size_t slots = computed->inputs.size();
    const size_t avars = program->constants->avarConstants.size();
    state.avarBindingReads.clear();
    state.avarConstantReads.clear();
    state.ladderOverrides.clear();
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
    program->weightRead.assign(geometry.weightObjects.size(), weightNone);

    // (slot, override number) for every read whose walk passes the slot.
    std::vector<std::pair<uint32_t, int32_t>> reach;
    std::unordered_map<int32_t, size_t> byUid;
    for (size_t k = 0; k < computed->registeredReads.size(); ++k) {
        const RigExecWireRegisteredRead &entry = computed->registeredReads[k];
        const auto fail = [&](const std::string &why) {
            return _RrFail(error, "the computed section's registered read " +
                                      std::to_string(k) + " " + why);
        };
        // The table input the read replaces, null for an avar binding, and
        // whether the read is made per run.
        const RigExecWireInput *wire = nullptr;
        const uint8_t flags = entry.read.flags;
        const bool live =
            (flags & uint8_t(v4::InputReadFlags::Varying)) != 0 &&
            ((flags & uint8_t(v4::InputReadFlags::LongWay)) != 0 ||
             entry.read.selected >= 0);
        const size_t object = entry.object;
        const uint32_t field = entry.field;
        int32_t *at = nullptr;
        switch (entry.family) {
        case _RrFamily::AvarBinding:
        case _RrFamily::AvarConstantBinding:
            if (entry.avar < 0 || size_t(entry.avar) >= avars || field != 0 ||
                entry.read.tag != v4::InputTag::Double) {
                return fail("names no avar");
            }
            if (!entry.read.walk.empty()) {
                int32_t &head = slotAvar[entry.read.walk[0]];
                if (head >= 0 && head != entry.avar) {
                    return fail("heads two avars");
                }
                head = entry.avar;
            }
            (entry.family == _RrFamily::AvarBinding ? state.avarBindingReads
                                                    : state.avarConstantReads)
                .push_back(uint32_t(k));
            break;
        case _RrFamily::Ladder:
            if (object >= poses.ladders.size() ||
                field >= uint32_t(RrLadderFieldCount)) {
                return fail("names no ladder field");
            }
            wire = &program->LadderInput(object, int(field));
            at = &program->ladderRead[object][field];
            // RigExecBakedProgramImpl::ladderOverrides: the provider
            // slots' ladders a drag recomposes.
            if (entry.read.overrideIndex >= 0 &&
                object < meta.slotKind.size() &&
                meta.slotKind[object] == RigExecWireSlotKind::FirstFramePose) {
                state.ladderOverrides.push_back(entry.read.overrideIndex);
            }
            break;
        case _RrFamily::SpaceSwitch:
            if (object >= poses.spaceSwitches.size() || field != 0) {
                return fail("names no space switch");
            }
            wire = &poses.spaceSwitches[object].active;
            at = &program->spaceSwitchRead[object];
            break;
        case _RrFamily::Interpolator:
            if (object >= poses.poseInterpolators.size() ||
                field > poses.poseInterpolators[object].valueInputs.size() ||
                field > 3) {
                return fail("names no interpolator input");
            }
            if (field == 0) {
                wire = &poses.poseInterpolators[object].enabled;
                at = &program->interpRead[object];
            } else {
                wire =
                    &poses.poseInterpolators[object].valueInputs[field - 1];
                at = &program->interpValueRead[object][field - 1];
            }
            break;
        case _RrFamily::Solver:
            if (object >= poses.solvers.size() ||
                field >= uint32_t(RrSolverFieldCount)) {
                return fail("names no solver field");
            }
            wire = &program->SolverInput(object, int(field));
            at = &program->solverRead[object][field];
            break;
        case _RrFamily::Constraint:
            if (object >= poses.constraints.size() ||
                field >= uint32_t(RrConstraintFieldCount)) {
                return fail("names no constraint field");
            }
            wire = &program->ConstraintInput(object, int(field));
            at = &program->constraintRead[object][field];
            break;
        case _RrFamily::WeightObject:
            if (object >= geometry.weightObjects.size() ||
                field >= uint32_t(RrWeightFieldCount)) {
                return fail("names no weight object field");
            }
            wire = &program->WeightInput(object, int(field));
            at = &program->weightRead[object][field];
            break;
        }
        if (at) {
            if (*at >= 0) {
                return fail("binds a field another read binds");
            }
            *at = int32_t(k);
        }
        // The table input it replaces carries the same type, override
        // number and per-run liveness.
        if (wire &&
            (uint8_t(wire->tag) != uint8_t(entry.read.tag) ||
             wire->overrideIndex != entry.read.overrideIndex ||
             (wire->varying && wire->bound) != live)) {
            return fail("differs from the table input it names");
        }
        if (entry.uid >= 0 && !byUid.emplace(entry.uid, k).second) {
            return fail("repeats uid " + std::to_string(entry.uid));
        }
        if (entry.read.overrideIndex >= 0) {
            for (uint32_t slot : entry.read.walk) {
                reach.emplace_back(slot, entry.read.overrideIndex);
            }
        }
    }
    // Every field of every table row is read, or a step would read none.
    const auto unbound = [&](const char *table, size_t row) {
        return _RrFail(error, "the computed section binds no read to a field "
                              "of " + std::string(table) + "[" +
                                  std::to_string(row) + "]");
    };
    for (size_t i = 0; i < program->ladderRead.size(); ++i) {
        for (int32_t read : program->ladderRead[i]) {
            if (read < 0) {
                return unbound("ladders", i);
            }
        }
    }
    for (size_t i = 0; i < program->spaceSwitchRead.size(); ++i) {
        if (program->spaceSwitchRead[i] < 0) {
            return unbound("spaceSwitches", i);
        }
    }
    for (size_t i = 0; i < program->interpRead.size(); ++i) {
        const size_t dials = poses.poseInterpolators[i].valueInputs.size();
        if (program->interpRead[i] < 0) {
            return unbound("poseInterpolators", i);
        }
        for (size_t v = 0; v < dials && v < 3; ++v) {
            if (program->interpValueRead[i][v] < 0) {
                return unbound("poseInterpolators", i);
            }
        }
    }
    for (size_t i = 0; i < program->solverRead.size(); ++i) {
        for (int32_t read : program->solverRead[i]) {
            if (read < 0) {
                return unbound("solvers", i);
            }
        }
    }
    for (size_t i = 0; i < program->constraintRead.size(); ++i) {
        for (int32_t read : program->constraintRead[i]) {
            if (read < 0) {
                return unbound("constraints", i);
            }
        }
    }
    for (size_t i = 0; i < program->weightRead.size(); ++i) {
        for (int32_t read : program->weightRead[i]) {
            if (read < 0) {
                return unbound("weightObjects", i);
            }
        }
    }
    std::sort(state.ladderOverrides.begin(), state.ladderOverrides.end());
    state.ladderOverrides.erase(std::unique(state.ladderOverrides.begin(),
                                            state.ladderOverrides.end()),
                                state.ladderOverrides.end());

    // A chain read restates the registered read of its uid, field for
    // field.
    for (const RigExecWireChainRead &entry : computed->chainReads) {
        const auto found = byUid.find(int32_t(entry.uid));
        if (found == byUid.end() ||
            !_RrSameRead(computed->registeredReads[found->second].read,
                         entry.read)) {
            return _RrFail(error, "the computed section's chain read of uid " +
                                      std::to_string(entry.uid) +
                                      " restates no registered read");
        }
    }

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
    }
    // That reach is the program's own (overridableInputs): the same
    // override numbers on the same attributes, or a set input would flag
    // other reads than an authored edit flags in the program.
    if (program->inputs) {
        const RigExecWireInputTable &table = *program->inputs;
        std::unordered_map<uint32_t, uint32_t> slotOf;
        slotOf.reserve(slots);
        for (size_t s = 0; s < slots; ++s) {
            slotOf.emplace(computed->inputs[s].name, uint32_t(s));
        }
        size_t placed = 0;
        for (size_t i = 0; i < table.overridablePaths.size() &&
                           i < table.overridableIndices.size();
             ++i) {
            std::vector<int32_t> numbers = table.overridableIndices[i];
            std::sort(numbers.begin(), numbers.end());
            numbers.erase(std::unique(numbers.begin(), numbers.end()),
                          numbers.end());
            const auto found = slotOf.find(table.overridablePaths[i]);
            const bool same =
                found != slotOf.end() &&
                std::equal(numbers.begin(), numbers.end(),
                           state.slotOverrideNumbers.begin() +
                               state.slotOverrideBegin[found->second],
                           state.slotOverrideNumbers.begin() +
                               state.slotOverrideBegin[found->second + 1]);
            if (!same) {
                return _RrFail(error,
                               "the computed section's reads reach other "
                               "override numbers at " +
                                   program->TextOrEmpty(
                                       table.overridablePaths[i]) +
                                   " than the program registered there");
            }
            ++placed;
        }
        size_t reached = 0;
        for (size_t s = 0; s < slots; ++s) {
            reached += state.slotOverrideBegin[s + 1] !=
                               state.slotOverrideBegin[s]
                           ? 1
                           : 0;
        }
        if (reached != placed) {
            return _RrFail(error, "the computed section's reads reach "
                                  "attributes the program registered no "
                                  "override on");
        }
    }
    state.nameIndex.clear();
    state.nameIndex.reserve(slots);
    for (size_t s = 0; s < slots; ++s) {
        state.nameIndex.emplace(program->TextOrEmpty(computed->inputs[s].name),
                                uint32_t(s));
    }
    // The listed inputs, as the input API reports them.
    const size_t listed = std::min<size_t>(computed->listedInputs, slots);
    state.inputInfo.clear();
    state.inputInfo.reserve(listed);
    state.inputValues.clear();
    state.inputValues.reserve(listed);
    for (size_t s = 0; s < listed; ++s) {
        const v4::InputSlot &slot = computed->inputs[s];
        RigExecRuntimeInputInfo info;
        info.name = program->TextOrEmpty(slot.name);
        info.type = RrInputTag(uint8_t(slot.type));
        info.animated =
            (slot.flags & uint8_t(v4::InputSlotFlags::Animated)) != 0;
        info.defaultValue = RrValueFromWire(computed->values[slot.value]);
        state.inputValues.push_back(RrValueFromWire(state.slotCurrent[s]));
        state.inputInfo.push_back(std::move(info));
    }

    // The geometry reads by revision and channel. RigExecWireApplyComputed
    // checked that each names a revision, and a channel, the geometry
    // section holds.
    state.defaultWeightRead.assign(geometry.chains.size(), {});
    state.blendReads.assign(geometry.chains.size(), {});
    state.derivedBlendReads.assign(geometry.chains.size(), {});
    for (size_t c = 0; c < geometry.chains.size(); ++c) {
        const RigExecWireChain &chain = geometry.chains[c];
        state.defaultWeightRead[c].assign(chain.revisions.size(), -1);
        state.blendReads[c].resize(chain.revisions.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            state.blendReads[c][r].assign(
                chain.revisions[r].blendChannels.size(), -1);
        }
        state.derivedBlendReads[c].resize(chain.derived.size());
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            state.derivedBlendReads[c][d].assign(
                chain.derived[d].revision.blendChannels.size(), -1);
        }
    }
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
                                          [entry.channel] = int32_t(k);
    }
    // The decoder refused two path reads at one path.
    state.pathReadIndex.clear();
    state.pathReadIndex.reserve(computed->pathScalarReads.size());
    for (size_t k = 0; k < computed->pathScalarReads.size(); ++k) {
        state.pathReadIndex.emplace_back(computed->pathScalarReads[k].path,
                                         uint32_t(k));
    }
    std::sort(state.pathReadIndex.begin(), state.pathReadIndex.end());
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
    }
    return "unknown";
}

// Slot \p slot holds \p value (with \p has), the input cache follows, and
// the slot is marked for the next run, once.
void
_RrStoreSlot(RrInputState &state, uint32_t slot,
             const v4::RigExecWireValue &value, bool has)
{
    state.slotCurrent[slot] = value;
    state.slotHasValue[slot] = has ? 1 : 0;
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
    v4::RigExecWireValue wire;
    wire.tag = v4::InputTag(uint8_t(type));
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
    }
    _RrStoreSlot(state, uint32_t(index), wire, true);
    return true;
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
    // Built here rather than at Open: only a reader that sets a token pays
    // for it, and a reader is owned by one thread.
    if (!state.tokenIdsBuilt) {
        for (uint32_t id = 0;; ++id) {
            std::string entry;
            if (!program->strings || !program->strings->GetString(id, &entry)) {
                break;
            }
            state.tokenIds.emplace(std::move(entry), id);
        }
        state.tokenIdsBuilt = true;
    }
    uint32_t id = 0;
    const auto found = state.tokenIds.find(text);
    if (found != state.tokenIds.end()) {
        id = found->second;
    } else {
        id = RrExtraTokenBase + uint32_t(state.extraTokens.size());
        state.extraTokens.push_back(text);
        state.tokenIds.emplace(text, id);
    }
    v4::RigExecWireValue wire;
    wire.tag = v4::InputTag::Token;
    wire.bits = id;
    _RrStoreSlot(state, uint32_t(index), wire, true);
    return true;
}

void
RrInputsReset(RrProgram *program, size_t index)
{
    RrInputState &state = program->inputState;
    if (index >= state.inputInfo.size()) {
        return;
    }
    const v4::InputSlot &slot = state.computed->inputs[index];
    _RrStoreSlot(state, uint32_t(index), state.computed->values[slot.value],
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
    const v4::InputSlot &slot = state.computed->inputs[index];
    _RrStoreSlot(state, uint32_t(index), state.computed->values[slot.value],
                 false);
    return true;
}

void
RrInputsApplyTouched(RrProgram *program)
{
    RrInputState &state = program->inputState;
    RrStore &store = program->store;
    const std::vector<v4::InputSlot> &slots = state.computed->inputs;
    const uint8_t animated = uint8_t(v4::InputSlotFlags::Animated);
    for (const uint32_t s : state.touched) {
        state.touchedFlag[s] = 0;
        // An animated input is what a time change moves: it never flags a
        // read, or setting it would dirty what only an edit dirties.
        if ((slots[s].flags & animated) != 0) {
            store.animatedTouched = true;
            continue;
        }
        for (uint32_t k = state.slotOverrideBegin[s];
             k < state.slotOverrideBegin[s + 1]; ++k) {
            const size_t number = size_t(state.slotOverrideNumbers[k]);
            if (number >= state.valueOverridden.size() ||
                number + 1 >= state.overrideSlotBegin.size()) {
                continue;
            }
            bool differs = false;
            for (uint32_t j = state.overrideSlotBegin[number];
                 j < state.overrideSlotBegin[number + 1] && !differs; ++j) {
                const uint32_t t = state.overrideSlotList[j];
                differs = (slots[t].flags & animated) == 0 &&
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
RrInputsOverlay(const RrProgram *program, uint32_t path, v4::InputTag tag,
                v4::RigExecWireValue *out)
{
    return _RrOverlay(*program, path, tag, out);
}

bool
RrChainBase(const RrProgram *program, size_t chain, v4::InputTag tag,
            v4::RigExecWireValue *base)
{
    const RrInputState &state = program->inputState;
    const RigExecWireComputed &computed = *state.computed;
    const uint32_t target = computed.propertyChains[chain].target;
    if (!state.slotHasValue[target] || computed.inputs[target].type != tag) {
        return false;
    }
    *base = _RrSlotValue(state, target);
    return true;
}

bool
RrPathValueFromRead(const RigExecWirePathScalarRead &entry,
                    const v4::RigExecWireValue &value,
                    RigExecWirePathValue *out)
{
    using Tag = RigExecWirePathValue::Tag;
    if (entry.widen) {
        out->tag = Tag::Double;
        out->f64 = double(RrWireValueFloat(value));
        return true;
    }
    switch (entry.read.tag) {
    case v4::InputTag::Bool:
        out->tag = Tag::Bool;
        out->boolean = value.bits != 0;
        return true;
    case v4::InputTag::Int:
        out->tag = Tag::Int;
        out->i32 = int32_t(uint32_t(value.bits));
        return true;
    case v4::InputTag::Float:
        out->tag = Tag::Float;
        out->f32 = RrWireValueFloat(value);
        return true;
    case v4::InputTag::Double:
        out->tag = Tag::Double;
        out->f64 = _RrWireDouble(value);
        return true;
    case v4::InputTag::Token:
        out->tag = Tag::Token;
        out->token = uint32_t(value.bits);
        return true;
    case v4::InputTag::Matrix4d:
        out->tag = Tag::Matrix4d;
        out->matrix = value.matrix;
        return true;
    case v4::InputTag::Vec3d:
        out->tag = Tag::Vec3d;
        out->vec = value.vec3d;
        return true;
    default:
        return false;
    }
}

float
RrReadBlendWeight(const RrProgram *program, size_t chain, size_t revision,
                  bool derived, size_t channel)
{
    const RrInputState &state = program->inputState;
    const std::vector<std::vector<std::vector<int32_t>>> &reads =
        derived ? state.derivedBlendReads : state.blendReads;
    if (chain >= reads.size() || revision >= reads[chain].size() ||
        channel >= reads[chain][revision].size() ||
        reads[chain][revision][channel] < 0) {
        return 0.0f;
    }
    const RigExecWireChain &wireChain = program->geometry->chains[chain];
    const RigExecWireRevision &wire =
        derived ? wireChain.derived[revision].revision
                : wireChain.revisions[revision];
    const RigExecWireBlendChannel &bound = wire.blendChannels[channel];
    if (bound.poseWeight >= 0 &&
        !RrInputsPublished(program, bound.weightPath)) {
        const std::vector<float> &weights = program->store.poseWeights;
        return size_t(bound.poseWeight) < weights.size()
                   ? weights[size_t(bound.poseWeight)]
                   : 0.0f;
    }
    const RigExecWireBlendWeightRead &entry =
        state.computed
            ->blendWeightReads[size_t(reads[chain][revision][channel])];
    return RrWireValueFloat(RrReadInput(program, entry.read));
}

float
RrReadBlendActivation(const RrProgram *program, size_t chain,
                      size_t revision, bool derived, size_t channel,
                      size_t sample)
{
    const RrInputState &state = program->inputState;
    const std::vector<std::vector<std::vector<int32_t>>> &reads =
        derived ? state.derivedBlendReads : state.blendReads;
    if (chain >= reads.size() || revision >= reads[chain].size() ||
        channel >= reads[chain][revision].size() ||
        reads[chain][revision][channel] < 0) {
        return 1.0f;
    }
    const RigExecWireBlendWeightRead &entry =
        state.computed
            ->blendWeightReads[size_t(reads[chain][revision][channel])];
    if (sample >= entry.activations.size()) {
        return 1.0f;
    }
    return RrWireValueFloat(RrReadInput(program, entry.activations[sample]));
}

float
RrReadDefaultWeight(const RrProgram *program, size_t chain, size_t revision)
{
    const RrInputState &state = program->inputState;
    if (chain >= state.defaultWeightRead.size() ||
        revision >= state.defaultWeightRead[chain].size() ||
        state.defaultWeightRead[chain][revision] < 0) {
        return 1.0f;
    }
    const RigExecWireDefaultWeightRead &entry =
        state.computed->defaultWeightReads[size_t(
            state.defaultWeightRead[chain][revision])];
    return RrWireValueFloat(RrReadInput(program, entry.read));
}

}  // namespace rigExec
