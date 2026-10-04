// rigExecRuntime property chains: a line port of runChain and its three
// appliers (RigExecRigEvaluator::_EvaluatePropertyChains,
// rigEvaluatorProperties.cpp) over the Computed section. The chains run
// before every prologue, as the baked prologue runs them, and publish into
// RrStore::propertyResults, which every later read of a chain target sees;
// the program's registered reads that cross a target are evaluated right
// after and handed to the holders their consumers read.
// The arithmetic is propertyMathKernel.h instantiated with the runtime's
// own types, so each revision computes the evaluator's bits. USD-free.
#include "rigExecRuntime/store.h"
#include "rigExecMath/propertyMathKernel.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

namespace rigExec {

namespace {

using _RrTag = RrPropertyValue::Tag;

// What Open settled about the chains: the diagnostic texts, the parsed
// operations, the curve keys as the kernel reads them, each chain's range
// of phased consumers, and where each result is published.
struct RrPropertyScratch {
    struct Revision {
        std::string mover;  ///< prim path text, for diagnostics
        bool opValid = false;
        RigExecPropertyOp op = RigExecPropertyOp::Add;
        std::vector<RrVec2f> keys;
        std::vector<RrVec2f> tangents;
        /// RigExecValidateLinearKeys over the keys (static data).
        bool keysValid = false;
    };
    struct Chain {
        std::string target;  ///< target path text, for diagnostics
        /// The slot type the chain's typed base read needs.
        v4::InputTag baseTag = v4::InputTag::Float;
        /// propertyValues entry of the target.
        size_t publish = 0;
        /// [phasedBegin, phasedEnd) in the section's phasedConsumers.
        size_t phasedBegin = 0;
        size_t phasedEnd = 0;
        std::vector<Revision> revisions;
    };
    std::vector<Chain> chains;
    /// propertyValues entry per phased consumer.
    std::vector<size_t> phasedPublish;
    /// Per propertyValues entry: the attribute path id it is published at.
    std::vector<uint32_t> publishNames;
    /// Per chain read: the frame-record field it is compared with, named
    /// for a mismatch.
    std::vector<std::string> chainReadFields;
    // Per-run scratch, reused across Executes.
    std::vector<RrPropertyValue> history;
    std::vector<float> weights;
};

bool
_RrFail(std::string *error, const std::string &why)
{
    if (error) {
        *error = why;
    }
    return false;
}

// Every member set, so two values compare equal exactly when their tagged
// member does (RrPropertyValue::operator== compares all of them).
RrPropertyValue
_RrPropertyZero()
{
    RrPropertyValue value;
    value.tag = _RrTag::Float;
    value.f32 = 0.0f;
    value.f64 = 0.0;
    value.matrix.SetDiagonal(0.0);
    value.vec = RrVec3f(0.0f);
    return value;
}

RrPropertyValue
_RrHold(float v)
{
    RrPropertyValue value = _RrPropertyZero();
    value.f32 = v;
    return value;
}

RrPropertyValue
_RrHold(double v)
{
    RrPropertyValue value = _RrPropertyZero();
    value.tag = _RrTag::Double;
    value.f64 = v;
    return value;
}

RrPropertyValue
_RrHold(const RrMat4d &m)
{
    RrPropertyValue value = _RrPropertyZero();
    value.tag = _RrTag::Matrix4d;
    value.matrix = m;
    return value;
}

RrPropertyValue
_RrHold(const RrVec3f &v)
{
    RrPropertyValue value = _RrPropertyZero();
    value.tag = _RrTag::Vec3f;
    value.vec = v;
    return value;
}

// _IsFinite (rigEvaluatorProperties.cpp). It has no double overload, so a
// double is tested as the float it converts to.
bool
_RrFinite(float v)
{
    return std::isfinite(v);
}

bool
_RrFinite(double v)
{
    return std::isfinite(float(v));
}

bool
_RrFinite(const RrVec3f &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
_RrFinite(const RrMat4d &m)
{
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            if (!std::isfinite(m[r][c])) {
                return false;
            }
        }
    }
    return true;
}

RrMat4d
_RrMatrix(const RigExecWireMatrix4d &m)
{
    RrMat4d out;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            out[r][c] = m[r * 4 + c];
        }
    }
    return out;
}

RrVec3f
_RrVec3f(const v4::RigExecWireValue &value)
{
    return RrVec3f(value.vec3f[0], value.vec3f[1], value.vec3f[2]);
}

void
_RrPublish(RrProgram *program, const RrPropertyScratch &scratch, size_t entry,
           const RrPropertyValue &value)
{
    RrStore &store = program->store;
    store.propertyValues[entry] = value;
    store.propertyPublished[entry] = 1;
    store.propertyResults[scratch.publishNames[entry]] = value;
}

// The float applier: the operation and its three inputs, all read and
// finite-checked whatever the operation, then the curve's keys and
// tangents, then the kernel.
bool
_RrApplyFloat(const RrProgram *program, const v4::PropertyRevision &revision,
              const RrPropertyScratch::Revision &bound, float in,
              float envelope, float *out)
{
    if (!bound.opValid) {
        return false;
    }
    RigExecPropertyMathKernelParams<float, RrVec2f> params;
    params.op = bound.op;
    params.value = RrWireValueFloat(RrReadInput(program, revision.value));
    params.min = RrWireValueFloat(RrReadInput(program, revision.min));
    params.max = RrWireValueFloat(RrReadInput(program, revision.max));
    if (!_RrFinite(params.value) || !_RrFinite(params.min) ||
        !_RrFinite(params.max)) {
        return false;
    }
    if (params.op == RigExecPropertyOp::Curve) {
        if (bound.keys.empty() || !bound.keysValid) {
            return false;
        }
        params.keys = bound.keys.data();
        params.keyCount = bound.keys.size();
        if (!bound.tangents.empty()) {
            if (bound.tangents.size() != bound.keys.size()) {
                return false;
            }
            params.tangents = bound.tangents.data();
            params.tangentCount = bound.tangents.size();
        }
    }
    params.weight = envelope;
    *out = RigExecApplyFloatMathKernel(in, params);
    return true;
}

// A double chain is computed in float: the incoming value is narrowed, the
// float applier runs, and its result is widened back.
bool
_RrApplyDouble(const RrProgram *program, const v4::PropertyRevision &revision,
               const RrPropertyScratch::Revision &bound, double in,
               float envelope, double *out)
{
    float result = 0.0f;
    if (!_RrApplyFloat(program, revision, bound, float(in), envelope,
                       &result)) {
        return false;
    }
    *out = double(result);
    return true;
}

bool
_RrApplyMatrix(const RrProgram *program, const v4::PropertyRevision &revision,
               const RrPropertyScratch::Revision &bound, const RrMat4d &in,
               float envelope, RrMat4d *out)
{
    if (!bound.opValid) {
        return false;
    }
    const RrMat4d opValue =
        _RrMatrix(RrReadInput(program, revision.value).matrix);
    if (!_RrFinite(opValue)) {
        return false;
    }
    return RigExecApplyMatrixMathKernel(in, bound.op, opValue, envelope, out);
}

bool
_RrApplyVec3f(const RrProgram *program, const v4::PropertyRevision &revision,
              const RrPropertyScratch::Revision &bound, const RrVec3f &in,
              float envelope, RrVec3f *out)
{
    if (!bound.opValid) {
        return false;
    }
    RigExecPropertyMathKernelParams<RrVec3f, RrVec2f> params;
    params.op = bound.op;
    params.value = _RrVec3f(RrReadInput(program, revision.value));
    params.min = _RrVec3f(RrReadInput(program, revision.min));
    params.max = _RrVec3f(RrReadInput(program, revision.max));
    if (!_RrFinite(params.value) || !_RrFinite(params.min) ||
        !_RrFinite(params.max)) {
        return false;
    }
    params.weight = envelope;
    *out = RigExecApplyVec3fMathKernel(in, params);
    return true;
}

// runChain: the base, then each revision in order (enabled, envelope,
// apply, finite result), then the published result and the phased
// consumers' values. A skipped chain publishes nothing.
template <class T, class Apply>
void
_RrRunChain(RrProgram *program, RrPropertyScratch *scratch, size_t c,
            T value, const Apply &apply,
            std::vector<std::string> *diagnostics)
{
    const RigExecWireComputed &computed = *program->inputState.computed;
    const v4::PropertyChain &chain = computed.propertyChains[c];
    const RrPropertyScratch::Chain &plan = scratch->chains[c];
    if (!_RrFinite(value)) {
        diagnostics->push_back("property chain " + plan.target +
                               ": authored base is not finite; chain "
                               "skipped");
        return;
    }
    // The value after each revision, base first, so a phased consumer can
    // read the chain where it asked to. A revision that passes through
    // still takes its place.
    const bool phased = plan.phasedBegin != plan.phasedEnd;
    std::vector<RrPropertyValue> &history = scratch->history;
    history.clear();
    for (size_t r = 0; r < chain.revisions.size(); ++r) {
        const v4::PropertyRevision &revision = chain.revisions[r];
        const RrPropertyScratch::Revision &bound = plan.revisions[r];
        if (phased) {
            history.push_back(_RrHold(value));
        }
        if (RrReadInput(program, revision.enabled).bits == 0) {
            diagnostics->push_back("diag " + bound.mover +
                                   ": disabled; revision passed through");
            continue;
        }
        float envelope = 1.0f;
        if (revision.envelope >= 0) {
            std::string error;
            if (!RrResolveWeightOracle(program, size_t(revision.envelope), 1,
                                       nullptr, &scratch->weights, &error) ||
                scratch->weights.size() != 1) {
                diagnostics->push_back("diag " + bound.mover + ": " + error +
                                       "; revision passed through");
                continue;
            }
            envelope = scratch->weights[0];
        } else {
            envelope =
                RrWireValueFloat(RrReadInput(program, revision.defaultWeight));
            if (!std::isfinite(envelope) || envelope < 0.0f ||
                envelope > 1.0f) {
                diagnostics->push_back(
                    "diag " + bound.mover +
                    ": inputs:defaultWeight must be finite and in [0, 1]; "
                    "revision passed through");
                continue;
            }
        }
        T next = value;
        if (!apply(program, revision, bound, value, envelope, &next)) {
            diagnostics->push_back("diag " + bound.mover +
                                   ": inputs unusable; revision passed "
                                   "through");
            continue;
        }
        if (!_RrFinite(next)) {
            diagnostics->push_back("diag " + bound.mover +
                                   ": produced a non-finite value; revision "
                                   "passed through");
            continue;
        }
        value = next;
    }
    // Published at once: a later chain that reads this target sees the
    // revised value.
    _RrPublish(program, *scratch, plan.publish, _RrHold(value));
    if (!phased) {
        return;
    }
    history.push_back(_RrHold(value));
    for (size_t k = plan.phasedBegin; k < plan.phasedEnd; ++k) {
        const v4::PhasedConsumer &consumer = computed.phasedConsumers[k];
        RrPropertyValue held =
            history[std::min(size_t(consumer.applied), history.size() - 1)];
        // RigExecPhasedConsumerValue: a float/double pair converts, every
        // other pair passes the value as it is.
        if (consumer.consumerType == v4::PropertyValueType::Double &&
            held.tag == _RrTag::Float) {
            held = _RrHold(double(held.f32));
        } else if (consumer.consumerType == v4::PropertyValueType::Float &&
                   held.tag == _RrTag::Double) {
            held = _RrHold(float(held.f64));
        }
        _RrPublish(program, *scratch, scratch->phasedPublish[k], held);
    }
}

const char *
_RrTagName(_RrTag tag)
{
    switch (tag) {
    case _RrTag::Float:
        return "float";
    case _RrTag::Double:
        return "double";
    case _RrTag::Matrix4d:
        return "matrix4d";
    case _RrTag::Vec3f:
        return "vec3f";
    }
    return "unknown";
}

const char *
_RrTagName(RigExecWirePropertyValue::Tag tag)
{
    return _RrTagName(_RrTag(uint8_t(tag)));
}

bool
_RrSameDoubleBits(double a, double b)
{
    return std::memcmp(&a, &b, sizeof(double)) == 0;
}

// A read's value in the frame record's form: the old input tags are the v4
// numbers up to Vec3d, and every member the tag does not use stays zero, as
// the record leaves it.
RigExecWireValue
_RrRecordForm(const v4::RigExecWireValue &value)
{
    RigExecWireValue out;
    out.tag = RigExecWireInput::Tag(uint8_t(value.tag));
    switch (value.tag) {
    case v4::InputTag::Double:
        std::memcpy(&out.f64, &value.bits, sizeof(out.f64));
        break;
    case v4::InputTag::Float:
        out.f32 = RrWireValueFloat(value);
        break;
    case v4::InputTag::Bool:
        out.boolean = value.bits != 0;
        break;
    case v4::InputTag::Int:
        out.i32 = int32_t(uint32_t(value.bits));
        break;
    case v4::InputTag::Matrix4d:
        out.matrix = value.matrix;
        break;
    case v4::InputTag::Token:
        out.token = uint32_t(value.bits);
        break;
    case v4::InputTag::Vec3d:
        out.vec = value.vec3d;
        break;
    default:
        break;
    }
    return out;
}

const char *
_RrInputTagName(RigExecWireInput::Tag tag)
{
    switch (tag) {
    case RigExecWireInput::Tag::Double:
        return "double";
    case RigExecWireInput::Tag::Float:
        return "float";
    case RigExecWireInput::Tag::Bool:
        return "bool";
    case RigExecWireInput::Tag::Int:
        return "int";
    case RigExecWireInput::Tag::Matrix4d:
        return "matrix4d";
    case RigExecWireInput::Tag::Token:
        return "token";
    case RigExecWireInput::Tag::Vec3d:
        return "vec3d";
    }
    return "unknown";
}

// Bitwise equality of a computed read and the record's value, on the
// member the tag names; false with the mismatch text naming \p field.
bool
_RrSameRecordBits(const RrProgram *program, const RigExecWireValue &computed,
                  const RigExecWireValue &recorded, const std::string &field,
                  std::string *why)
{
    if (computed.tag != recorded.tag) {
        *why = RrCrossCheckMismatch(field + ".tag",
                                    _RrInputTagName(computed.tag),
                                    _RrInputTagName(recorded.tag));
        return false;
    }
    switch (computed.tag) {
    case RigExecWireInput::Tag::Double:
        if (!_RrSameDoubleBits(computed.f64, recorded.f64)) {
            *why = RrCrossCheckMismatch(field, computed.f64, recorded.f64);
            return false;
        }
        return true;
    case RigExecWireInput::Tag::Float:
        if (!RrSameFloatBits(computed.f32, recorded.f32)) {
            *why = RrCrossCheckMismatch(field, computed.f32, recorded.f32);
            return false;
        }
        return true;
    case RigExecWireInput::Tag::Bool:
        if (computed.boolean != recorded.boolean) {
            *why = RrCrossCheckMismatch(
                field, std::string(computed.boolean ? "true" : "false"),
                std::string(recorded.boolean ? "true" : "false"));
            return false;
        }
        return true;
    case RigExecWireInput::Tag::Int:
        if (computed.i32 != recorded.i32) {
            *why = RrCrossCheckMismatch(field, std::to_string(computed.i32),
                                        std::to_string(recorded.i32));
            return false;
        }
        return true;
    case RigExecWireInput::Tag::Matrix4d:
        for (size_t i = 0; i < 16; ++i) {
            if (!_RrSameDoubleBits(computed.matrix[i], recorded.matrix[i])) {
                *why = RrCrossCheckMismatch(
                    field + ".matrix[" + std::to_string(i / 4) + "][" +
                        std::to_string(i % 4) + "]",
                    computed.matrix[i], recorded.matrix[i]);
                return false;
            }
        }
        return true;
    case RigExecWireInput::Tag::Token:
        if (computed.token != recorded.token) {
            *why = RrCrossCheckMismatch(field,
                                        program->TextOrEmpty(computed.token),
                                        program->TextOrEmpty(recorded.token));
            return false;
        }
        return true;
    case RigExecWireInput::Tag::Vec3d:
        for (size_t i = 0; i < 3; ++i) {
            if (!_RrSameDoubleBits(computed.vec[i], recorded.vec[i])) {
                *why = RrCrossCheckMismatch(
                    field + ".vec[" + std::to_string(i) + "]",
                    computed.vec[i], recorded.vec[i]);
                return false;
            }
        }
        return true;
    }
    *why = RrCrossCheckMismatch(field + ".tag", std::string("unknown"),
                                std::string("unknown"));
    return false;
}

}  // namespace

bool
RrPropertySizeScratch(RrProgram *program, std::string *error)
{
    auto scratch = std::make_shared<RrPropertyScratch>();
    RrStore &store = program->store;
    const RigExecWireComputed *computed = program->inputState.computed;
    if (!computed) {
        if (program->poses && program->poses->hasPropertyChains) {
            return _RrFail(error, "the rig runs property chains, which need "
                                  "the computed section this file does not "
                                  "carry; rebake it");
        }
    } else {
        const size_t slots = computed->inputs.size();
        std::vector<int64_t> slotEntry(slots, -1);
        const auto entryOf = [&](uint32_t slot) {
            if (slotEntry[slot] < 0) {
                slotEntry[slot] = int64_t(scratch->publishNames.size());
                scratch->publishNames.push_back(computed->inputs[slot].name);
            }
            return size_t(slotEntry[slot]);
        };
        scratch->chains.resize(computed->propertyChains.size());
        for (size_t c = 0; c < computed->propertyChains.size(); ++c) {
            const v4::PropertyChain &chain = computed->propertyChains[c];
            RrPropertyScratch::Chain &plan = scratch->chains[c];
            if (chain.target >= slots) {
                return _RrFail(error, "a property chain targets no input "
                                      "slot");
            }
            plan.target = program->TextOrEmpty(computed->inputs[chain.target]
                                                   .name);
            switch (chain.valueType) {
            case v4::PropertyValueType::Float:
                plan.baseTag = v4::InputTag::Float;
                break;
            case v4::PropertyValueType::Double:
                plan.baseTag = v4::InputTag::Double;
                break;
            case v4::PropertyValueType::Matrix4d:
                plan.baseTag = v4::InputTag::Matrix4d;
                break;
            case v4::PropertyValueType::Vec3f:
                plan.baseTag = v4::InputTag::Vec3f;
                break;
            default:
                return _RrFail(error, "property chain " + plan.target +
                                          " has an unknown value type");
            }
            plan.publish = entryOf(chain.target);
            plan.revisions.resize(chain.revisions.size());
            for (size_t r = 0; r < chain.revisions.size(); ++r) {
                const v4::PropertyRevision &revision = chain.revisions[r];
                RrPropertyScratch::Revision &bound = plan.revisions[r];
                bound.mover = program->TextOrEmpty(revision.mover);
                if (revision.envelope >= 0 &&
                    size_t(revision.envelope) >=
                        computed->weightObjects.size()) {
                    return _RrFail(error, bound.mover +
                                              " binds a weight object the "
                                              "computed section does not "
                                              "carry");
                }
                // RigExecPropertyOp's enumerators in the section's order.
                bound.opValid = revision.op != v4::PropertyOp::Invalid &&
                                uint8_t(revision.op) < v4::PropertyOpLast;
                if (bound.opValid) {
                    bound.op = RigExecPropertyOp(int(revision.op));
                }
                bound.keys.reserve(revision.keys.size());
                for (const RigExecWireVec2f &key : revision.keys) {
                    bound.keys.emplace_back(key[0], key[1]);
                }
                bound.tangents.reserve(revision.tangents.size());
                for (const RigExecWireVec2f &tangent : revision.tangents) {
                    bound.tangents.emplace_back(tangent[0], tangent[1]);
                }
                bound.keysValid = RigExecValidateLinearKeysKernel(
                    bound.keys.data(), bound.keys.size());
            }
        }
        // Grouped by chain, so each chain's consumers are one range.
        scratch->phasedPublish.resize(computed->phasedConsumers.size());
        for (size_t k = 0; k < computed->phasedConsumers.size(); ++k) {
            const v4::PhasedConsumer &consumer = computed->phasedConsumers[k];
            if (consumer.chain >= scratch->chains.size() ||
                consumer.consumer >= slots) {
                return _RrFail(error, "a phased consumer names no chain or "
                                      "no input slot");
            }
            RrPropertyScratch::Chain &plan = scratch->chains[consumer.chain];
            if (plan.phasedBegin == plan.phasedEnd) {
                plan.phasedBegin = k;
            } else if (plan.phasedEnd != k) {
                return _RrFail(error, "the phased consumers of property "
                                      "chain " + plan.target +
                                          " are not contiguous");
            }
            plan.phasedEnd = k + 1;
            scratch->phasedPublish[k] = entryOf(consumer.consumer);
        }
        // A chain read's result goes to the holder of its uid, which the
        // uid replay must route to an input that reads the long way, or to
        // an avar binding.
        std::vector<const RigExecWireInput *> routed(store.inputHolders.size(),
                                                     nullptr);
        const auto route = [&](int32_t uid, const RigExecWireInput &input) {
            if (uid >= 0 && size_t(uid) < routed.size()) {
                routed[size_t(uid)] = &input;
            }
        };
        for (size_t i = 0; i < program->ladderUid.size(); ++i) {
            for (int f = 0; f < RrLadderFieldCount; ++f) {
                route(program->ladderUid[i][size_t(f)],
                      program->LadderInput(i, f));
            }
        }
        for (size_t i = 0; i < program->spaceSwitchUid.size(); ++i) {
            route(program->spaceSwitchUid[i],
                  program->poses->spaceSwitches[i].active);
        }
        for (size_t i = 0; i < program->interpUid.size(); ++i) {
            const RigExecWirePoseInterpolator &interp =
                program->poses->poseInterpolators[i];
            route(program->interpUid[i], interp.enabled);
            for (size_t v = 0; v < interp.valueInputs.size() && v < 3; ++v) {
                route(program->interpValueUid[i][v], interp.valueInputs[v]);
            }
        }
        for (size_t i = 0; i < program->solverUid.size(); ++i) {
            for (int f = 0; f < RrSolverFieldCount; ++f) {
                route(program->solverUid[i][size_t(f)],
                      program->SolverInput(i, f));
            }
        }
        for (size_t i = 0; i < program->constraintUid.size(); ++i) {
            for (int f = 0; f < RrConstraintFieldCount; ++f) {
                route(program->constraintUid[i][size_t(f)],
                      program->ConstraintInput(i, f));
            }
        }
        for (size_t i = 0; i < program->weightUid.size(); ++i) {
            for (int f = 0; f < RrWeightFieldCount; ++f) {
                route(program->weightUid[i][size_t(f)],
                      program->WeightInput(i, f));
            }
        }
        scratch->chainReadFields.reserve(computed->chainReads.size());
        for (const RigExecWireChainRead &entry : computed->chainReads) {
            const std::string head =
                entry.read.walk.empty()
                    ? std::string()
                    : program->TextOrEmpty(
                          computed->inputs[entry.read.walk[0]].name);
            const size_t uid = entry.uid;
            const bool longWay =
                uid < routed.size() &&
                (routed[uid] ? routed[uid]->viaResolved
                             : uid < program->avarUidTarget.size() &&
                                   program->avarUidTarget[uid] >= 0);
            if (!longWay) {
                return _RrFail(error, "the computed section reads " + head +
                                          " through a property chain, but "
                                          "uid " + std::to_string(uid) +
                                          " routes to no input that reads "
                                          "the long way");
            }
            scratch->chainReadFields.push_back(
                "values[uid " + std::to_string(uid) + ", " + head + "]");
        }
    }
    const size_t entries = scratch->publishNames.size();
    store.propertyResults.clear();
    store.propertyValues.assign(entries, _RrPropertyZero());
    store.lastPropertyValues.assign(entries, _RrPropertyZero());
    store.propertyPublished.assign(entries, 0);
    store.lastPropertyPublished.assign(entries, 0);
    program->properties = scratch;
    return true;
}

bool
RrRunPropertyChains(RrProgram *program,
                    std::vector<std::string> *poseDiagnostics)
{
    RrStore &store = program->store;
    store.propertyResults.clear();
    std::fill(store.propertyValues.begin(), store.propertyValues.end(),
              _RrPropertyZero());
    std::fill(store.propertyPublished.begin(), store.propertyPublished.end(),
              char(0));
    const RigExecWireComputed *computed = program->inputState.computed;
    if (!computed || computed->propertyChains.empty()) {
        return true;
    }
    RrPropertyScratch *scratch =
        static_cast<RrPropertyScratch *>(program->properties.get());
    if (!scratch || scratch->chains.size() != computed->propertyChains.size()) {
        return false;
    }
    const RrInputState &state = program->inputState;
    for (size_t c = 0; c < computed->propertyChains.size(); ++c) {
        const v4::PropertyChain &chain = computed->propertyChains[c];
        const RrPropertyScratch::Chain &plan = scratch->chains[c];
        // The base is the target's own typed value this frame; a missing
        // value, or one of another type, fails that read.
        const uint32_t target = chain.target;
        if (!state.slotHasValue[target] ||
            computed->inputs[target].type != plan.baseTag) {
            poseDiagnostics->push_back("property chain " + plan.target +
                                       ": target has no authored value; "
                                       "chain skipped");
            continue;
        }
        const v4::RigExecWireValue &base =
            computed->values[state.slotValue[target]];
        switch (chain.valueType) {
        case v4::PropertyValueType::Float:
            _RrRunChain(program, scratch, c, RrWireValueFloat(base),
                        _RrApplyFloat, poseDiagnostics);
            break;
        case v4::PropertyValueType::Double: {
            double value = 0.0;
            std::memcpy(&value, &base.bits, sizeof(value));
            _RrRunChain(program, scratch, c, value, _RrApplyDouble,
                        poseDiagnostics);
            break;
        }
        case v4::PropertyValueType::Matrix4d:
            _RrRunChain(program, scratch, c, _RrMatrix(base.matrix),
                        _RrApplyMatrix, poseDiagnostics);
            break;
        case v4::PropertyValueType::Vec3f:
            _RrRunChain(program, scratch, c, _RrVec3f(base), _RrApplyVec3f,
                        poseDiagnostics);
            break;
        }
    }
    return true;
}

bool
RrCrossCheckPropertyResults(const RrProgram *program,
                            const RigExecWireFrameInputs &record,
                            uint64_t *compared, std::string *error)
{
    const auto mismatch = [error](const std::string &why) {
        return _RrFail(error, "property chains: " + why);
    };
    const auto fieldOf = [program](uint32_t path) {
        return "propertyValues[" + program->TextOrEmpty(path) + "]";
    };
    if (record.propertyPaths.size() != record.propertyValues.size()) {
        return _RrFail(error,
                       "frame record pairs no values with its properties");
    }
    const std::map<uint32_t, RrPropertyValue> &results =
        program->store.propertyResults;
    uint64_t checked = 0;
    for (size_t i = 0; i < record.propertyPaths.size(); ++i) {
        const RigExecWirePropertyValue &recorded = record.propertyValues[i];
        const uint32_t path = record.propertyPaths[i];
        const auto found = results.find(path);
        if (found == results.end()) {
            return mismatch(RrCrossCheckMismatch(
                fieldOf(path), std::string("nothing"),
                _RrTagName(recorded.tag)));
        }
        const RrPropertyValue &value = found->second;
        if (uint8_t(value.tag) != uint8_t(recorded.tag)) {
            return mismatch(RrCrossCheckMismatch(fieldOf(path) + ".tag",
                                                 _RrTagName(value.tag),
                                                 _RrTagName(recorded.tag)));
        }
        switch (value.tag) {
        case _RrTag::Float:
            if (!RrSameFloatBits(value.f32, recorded.f32)) {
                return mismatch(
                    RrCrossCheckMismatch(fieldOf(path), value.f32,
                                         recorded.f32));
            }
            break;
        case _RrTag::Double:
            if (!_RrSameDoubleBits(value.f64, recorded.f64)) {
                return mismatch(
                    RrCrossCheckMismatch(fieldOf(path), value.f64,
                                         recorded.f64));
            }
            break;
        case _RrTag::Matrix4d:
            for (size_t r = 0; r < 4; ++r) {
                for (size_t c = 0; c < 4; ++c) {
                    const double want = recorded.matrix[r * 4 + c];
                    if (!_RrSameDoubleBits(value.matrix[r][c], want)) {
                        return mismatch(RrCrossCheckMismatch(
                            fieldOf(path) + ".matrix[" + std::to_string(r) +
                                "][" + std::to_string(c) + "]",
                            value.matrix[r][c], want));
                    }
                }
            }
            break;
        case _RrTag::Vec3f:
            for (size_t k = 0; k < 3; ++k) {
                if (!RrSameFloatBits(value.vec[k], recorded.vec[k])) {
                    return mismatch(RrCrossCheckMismatch(
                        fieldOf(path) + ".vec[" + std::to_string(k) + "]",
                        value.vec[k], recorded.vec[k]));
                }
            }
            break;
        }
        ++checked;
    }
    // Every recorded path is published (above) and the record names each
    // path once, so a larger map publishes a path the record lacks.
    if (results.size() != record.propertyPaths.size()) {
        for (const auto &[path, value] : results) {
            if (std::find(record.propertyPaths.begin(),
                          record.propertyPaths.end(),
                          path) == record.propertyPaths.end()) {
                return mismatch(RrCrossCheckMismatch(fieldOf(path),
                                                     _RrTagName(value.tag),
                                                     std::string("nothing")));
            }
        }
        return mismatch("the frame record names a property path twice");
    }
    if (compared) {
        *compared += checked;
    }
    return true;
}

bool
RrRunChainReads(RrProgram *program, const RigExecWireFrameInputs &record,
                uint64_t *compared, std::string *error)
{
    const RigExecWireComputed *computed = program->inputState.computed;
    if (!computed || computed->chainReads.empty()) {
        return true;
    }
    const RrPropertyScratch *scratch =
        static_cast<const RrPropertyScratch *>(program->properties.get());
    if (!scratch ||
        scratch->chainReadFields.size() != computed->chainReads.size()) {
        return _RrFail(error, "the chain reads disagree with the computed "
                              "section");
    }
    RrStore &store = program->store;
    uint64_t checked = 0;
    for (size_t k = 0; k < computed->chainReads.size(); ++k) {
        const RigExecWireChainRead &entry = computed->chainReads[k];
        const RigExecWireValue value =
            _RrRecordForm(RrReadInput(program, entry.read));
        // The record holds a value for the uid only at the frames whose
        // step read it (uids ascending, paired with values); those are
        // compared.
        if (program->crossCheck) {
            const auto at = std::lower_bound(record.uids.begin(),
                                             record.uids.end(), entry.uid);
            const size_t index = size_t(at - record.uids.begin());
            if (at != record.uids.end() && *at == entry.uid &&
                index < record.values.size()) {
                std::string why;
                if (!_RrSameRecordBits(program, value, record.values[index],
                                       scratch->chainReadFields[k], &why)) {
                    return _RrFail(error, "chain reads: " + why);
                }
                ++checked;
            }
        }
        store.inputHolders[entry.uid] = RrWireFrameValue(value);
    }
    if (compared) {
        *compared += checked;
    }
    return true;
}

}  // namespace rigExec
