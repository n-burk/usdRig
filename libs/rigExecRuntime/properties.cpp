// rigExecRuntime property chains: a line port of runChain and its three
// appliers (RigExecRigEvaluator::_EvaluatePropertyChains,
// rigEvaluatorProperties.cpp) over the Computed section. The chains run
// before every prologue, as the baked prologue runs them, and publish into
// RrStore::propertyResults, which every later read of a chain target sees.
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

}  // namespace

bool
RrPropertySizeScratch(RrProgram *program, std::string *error)
{
    auto scratch = std::make_shared<RrPropertyScratch>();
    RrStore &store = program->store;
    const RigExecWireComputed *computed = program->inputState.computed;
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
    if (computed->propertyChains.empty()) {
        return true;
    }
    RrPropertyScratch *scratch =
        static_cast<RrPropertyScratch *>(program->properties.get());
    if (!scratch || scratch->chains.size() != computed->propertyChains.size()) {
        return false;
    }
    for (size_t c = 0; c < computed->propertyChains.size(); ++c) {
        const v4::PropertyChain &chain = computed->propertyChains[c];
        const RrPropertyScratch::Chain &plan = scratch->chains[c];
        // The base: the target's own value, which an input set there
        // authors; a missing value, or one of another type, fails that
        // read.
        v4::RigExecWireValue base;
        if (!RrChainBase(program, c, plan.baseTag, &base)) {
            poseDiagnostics->push_back("property chain " + plan.target +
                                       ": target has no authored value; "
                                       "chain skipped");
            continue;
        }
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

}  // namespace rigExec
