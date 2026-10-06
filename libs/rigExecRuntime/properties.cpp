// rigExecRuntime property chains: a line port of runChain and its three
// appliers (RigExecRigEvaluator::_EvaluatePropertyChains,
// rigEvaluatorProperties.cpp) over the file's chain tables. The chains run
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
        RigExecWireInputTag baseTag = RigExecWireInputTag::Float;
        /// propertyValues entry of the target.
        size_t publish = 0;
        /// [phasedBegin, phasedEnd) in the file's phasedConsumers.
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
_RrVec3f(const RrWireValue &value)
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
_RrApplyFloat(const RrProgram *program, const RigExecWirePropertyRevision &revision,
              const RrPropertyScratch::Revision &bound, float in,
              float envelope, float *out)
{
    if (!bound.opValid) {
        return false;
    }
    RigExecPropertyMathKernelParams<float, RrVec2f> params;
    params.op = bound.op;
    params.value = RrWireValueFloat(RrReadInput(program, *revision.value));
    params.min = RrWireValueFloat(RrReadInput(program, *revision.min));
    params.max = RrWireValueFloat(RrReadInput(program, *revision.max));
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
_RrApplyDouble(const RrProgram *program, const RigExecWirePropertyRevision &revision,
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
_RrApplyMatrix(const RrProgram *program, const RigExecWirePropertyRevision &revision,
               const RrPropertyScratch::Revision &bound, const RrMat4d &in,
               float envelope, RrMat4d *out)
{
    if (!bound.opValid) {
        return false;
    }
    const RrMat4d opValue =
        _RrMatrix(RrReadInput(program, *revision.value).matrix);
    if (!_RrFinite(opValue)) {
        return false;
    }
    return RigExecApplyMatrixMathKernel(in, bound.op, opValue, envelope, out);
}

bool
_RrApplyVec3f(const RrProgram *program, const RigExecWirePropertyRevision &revision,
              const RrPropertyScratch::Revision &bound, const RrVec3f &in,
              float envelope, RrVec3f *out)
{
    if (!bound.opValid) {
        return false;
    }
    RigExecPropertyMathKernelParams<RrVec3f, RrVec2f> params;
    params.op = bound.op;
    params.value = _RrVec3f(RrReadInput(program, *revision.value));
    params.min = _RrVec3f(RrReadInput(program, *revision.min));
    params.max = _RrVec3f(RrReadInput(program, *revision.max));
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
_RrRunPart(RrProgram *program, RrPropertyScratch *scratch, size_t c, size_t part,
           T value, const Apply &apply, std::vector<std::string> *diagnostics)
{
    const auto &file = *program->inputState.file;
    const auto &chain = file.propertyChains[c];
    const auto &plan = scratch->chains[c];
    auto &store = program->store;
    if (part == 0) {
        store.propertyChainValid[c] = _RrFinite(value) ? 1 : 0;
        if (!store.propertyChainValid[c])
            diagnostics->push_back("property chain " + plan.target +
                ": authored base is not finite; chain skipped");
    } else if (store.propertyChainValid[c]) {
        const size_t r = part - 1;
        const auto &revision = chain.revisions[r];
        const auto &bound = plan.revisions[r];
        do {
        if (RrReadInput(program, *revision.enabled).bits == 0) {
            diagnostics->push_back("diag " + bound.mover +
                                   ": disabled; revision passed through");
            break;
        }
        float envelope = 1.0f;
        if (revision.envelope >= 0) {
            std::string error;
            if (!RrResolveWeightOracle(program, size_t(revision.envelope), 1,
                                       nullptr, &scratch->weights, &error) ||
                scratch->weights.size() != 1) {
                diagnostics->push_back("diag " + bound.mover + ": " + error +
                                       "; revision passed through");
                break;
            }
            envelope = scratch->weights[0];
        } else {
            envelope =
                RrWireValueFloat(RrReadInput(program, *revision.defaultWeight));
            if (!std::isfinite(envelope) || envelope < 0.0f ||
                envelope > 1.0f) {
                diagnostics->push_back(
                    "diag " + bound.mover +
                    ": inputs:defaultWeight must be finite and in [0, 1]; "
                    "revision passed through");
                break;
            }
        }
        T next = value;
        if (!apply(program, revision, bound, value, envelope, &next)) {
            diagnostics->push_back("diag " + bound.mover +
                                   ": inputs unusable; revision passed "
                                   "through");
            break;
        }
        if (!_RrFinite(next)) {
            diagnostics->push_back("diag " + bound.mover +
                                   ": produced a non-finite value; revision "
                                   "passed through");
            break;
        }
        value = next;
        } while (false);
    }
    const size_t version = size_t(chain.versionBase) + part;
    store.propertyVersions[version] = _RrHold(value);
    store.propertyVersionValid[version] = store.propertyChainValid[c];
    for (size_t k = plan.phasedBegin; k < plan.phasedEnd; ++k) {
        const auto &consumer = file.phasedConsumers[k];
        if (size_t(consumer.applied) != part) continue;
        auto held = _RrHold(value);
        if (consumer.consumerType == RigExecWirePropertyValueType::Double &&
            held.tag == _RrTag::Float) held = _RrHold(double(held.f32));
        else if (consumer.consumerType == RigExecWirePropertyValueType::Float &&
                 held.tag == _RrTag::Double) held = _RrHold(float(held.f64));
        store.propertyVersions[consumer.version] = held;
        store.propertyVersionValid[consumer.version] =
            store.propertyChainValid[c] && !store.propertyRecordStoodAside[k];
    }
}

}  // namespace

bool
RrPropertySizeScratch(RrProgram *program, std::string *error)
{
    auto scratch = std::make_shared<RrPropertyScratch>();
    RrStore &store = program->store;
    const RigExecWireFile *file = program->inputState.file;
    const size_t slots = file->inputs.size();
    std::vector<int64_t> slotEntry(slots, -1);
    const auto entryOf = [&](uint32_t slot) {
        if (slotEntry[slot] < 0) {
            slotEntry[slot] = int64_t(scratch->publishNames.size());
            scratch->publishNames.push_back(file->inputs[slot].name());
        }
        return size_t(slotEntry[slot]);
    };
    scratch->chains.resize(file->propertyChains.size());
    for (size_t c = 0; c < file->propertyChains.size(); ++c) {
        const RigExecWirePropertyChain &chain = file->propertyChains[c];
        RrPropertyScratch::Chain &plan = scratch->chains[c];
        if (chain.target >= slots) {
            return _RrFail(error, "a property chain targets no input "
                                  "slot");
        }
        plan.target =
            program->TextOrEmpty(file->inputs[chain.target].name());
        switch (chain.valueType) {
        case RigExecWirePropertyValueType::Float:
            plan.baseTag = RigExecWireInputTag::Float;
            break;
        case RigExecWirePropertyValueType::Double:
            plan.baseTag = RigExecWireInputTag::Double;
            break;
        case RigExecWirePropertyValueType::Matrix4d:
            plan.baseTag = RigExecWireInputTag::Matrix4d;
            break;
        case RigExecWirePropertyValueType::Vec3f:
            plan.baseTag = RigExecWireInputTag::Vec3f;
            break;
        default:
            return _RrFail(error, "property chain " + plan.target +
                                      " has an unknown value type");
        }
        plan.publish = entryOf(chain.target);
        plan.revisions.resize(chain.revisions.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecWirePropertyRevision &revision = chain.revisions[r];
            RrPropertyScratch::Revision &bound = plan.revisions[r];
            bound.mover = program->TextOrEmpty(revision.mover);
            if (revision.envelope >= 0 &&
                size_t(revision.envelope) >=
                    program->geometry->weightObjects.size()) {
                return _RrFail(error, bound.mover +
                                          " binds a weight object the "
                                          "file does not carry");
            }
            // RigExecPropertyOp's enumerators in the file's order.
            bound.opValid =
                revision.op != RigExecWirePropertyOp::Invalid &&
                uint8_t(revision.op) < uint8_t(RigExecWirePropertyOp::MAX);
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
    scratch->phasedPublish.resize(file->phasedConsumers.size());
    for (size_t k = 0; k < file->phasedConsumers.size(); ++k) {
        const RigExecWirePhasedConsumer &consumer = file->phasedConsumers[k];
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
    store.propertyPublished.assign(entries, 0);
    size_t versions = 0;
    for (const auto &chain : file->propertyChains)
        versions = std::max(versions, size_t(chain.versionBase) + chain.revisions.size() + 1);
    for (const auto &record : file->phasedConsumers)
        versions = std::max(versions, size_t(record.version) + 1);
    store.propertyVersions.assign(versions, _RrPropertyZero());
    store.propertyVersionValid.assign(versions, 0);
    store.propertyVersionChanged.assign(versions, 0);
    store.propertyChainValid.assign(file->propertyChains.size(), 0);
    store.propertyRecordStoodAside.assign(file->phasedConsumers.size(), 0);
    store.propertyPublishedVersions.assign(slots, -1);
    store.propertyPathSlots.clear();
    for (size_t slot = 0; slot < slots; ++slot)
        store.propertyPathSlots.emplace(file->inputs[slot].name(), uint32_t(slot));
    program->properties = scratch;
    return true;
}

void
RrPropertyBegin(RrProgram *program)
{
    auto &store = program->store;
    store.propertyResults.clear();
    std::fill(store.propertyPublishedVersions.begin(), store.propertyPublishedVersions.end(), -1);
    std::fill(store.propertyPublished.begin(), store.propertyPublished.end(), char(0));
    std::fill(store.propertyVersionChanged.begin(), store.propertyVersionChanged.end(), char(0));
    const auto &records = program->inputState.file->phasedConsumers;
    for (size_t k = 0; k < records.size(); ++k) {
        // The runtime authored API changes the raw upstream layer. It
        // supplies no interactive overlay, so it never stands a record aside.
        const bool standing = false;
        store.propertyRecordStoodAside[k] = standing ? 1 : 0;
        // The producer owns the value; publication validity follows this
        // generation's override kind even while that producer stays clean.
        const char valid = store.propertyChainValid[records[k].chain] && !standing;
        if (store.propertyVersionValid[records[k].version] != valid)
            store.propertyVersionChanged[records[k].version] = 1;
        store.propertyVersionValid[records[k].version] = valid;
    }
}

void
RrPropertyPublishFinished(RrProgram *program, const std::vector<char> &finished)
{
    auto &store = program->store;
    const auto &file = *program->inputState.file;
    const auto *scratch = static_cast<RrPropertyScratch *>(program->properties.get());
    if (!scratch) return;
    for (size_t c = 0; c < file.propertyChains.size(); ++c) {
        if (!finished[c] || !store.propertyChainValid[c]) continue;
        const auto &chain = file.propertyChains[c];
        const size_t version = size_t(chain.versionBase) + chain.revisions.size();
        store.propertyPublishedVersions[chain.target] = int32_t(version);
        _RrPublish(program, *scratch, scratch->chains[c].publish,
                   store.propertyVersions[version]);
        for (size_t k = scratch->chains[c].phasedBegin;
             k < scratch->chains[c].phasedEnd; ++k) {
            const auto &record = file.phasedConsumers[k];
            if (!store.propertyVersionValid[record.version]) continue;
            store.propertyPublishedVersions[record.consumer] = int32_t(record.version);
            _RrPublish(program, *scratch, scratch->phasedPublish[k],
                       store.propertyVersions[record.version]);
        }
    }
}

void
RrPropertyPublish(RrProgram *program)
{
    const std::vector<char> finished(program->inputState.file->propertyChains.size(), 1);
    RrPropertyPublishFinished(program, finished);
}

bool
RrRunPropertyPart(RrProgram *program, size_t c, size_t part,
                  std::vector<std::string> *diagnostics)
{
    auto &store = program->store;
    const auto &file = *program->inputState.file;
    auto *scratch = static_cast<RrPropertyScratch *>(program->properties.get());
    if (!scratch || c >= file.propertyChains.size() ||
        part > file.propertyChains[c].revisions.size()) return false;
    const auto &chain = file.propertyChains[c];
    RrWireValue base;
    if (part == 0 && !RrChainBase(program, c, scratch->chains[c].baseTag, &base)) {
        store.propertyChainValid[c] = 0;
        store.propertyVersionValid[chain.versionBase] = 0;
        for (const auto &record : file.phasedConsumers)
            if (record.chain == c && record.applied == 0)
                store.propertyVersionValid[record.version] = 0;
        diagnostics->push_back("property chain " + scratch->chains[c].target +
                              ": target has no authored value; chain skipped");
        return true;
    }
    const auto &previous = store.propertyVersions[
        size_t(chain.versionBase) + (part ? part - 1 : 0)];
    switch (chain.valueType) {
    case RigExecWirePropertyValueType::Float:
        _RrRunPart(program, scratch, c, part,
                   part ? previous.f32 : RrWireValueFloat(base), _RrApplyFloat, diagnostics);
        break;
    case RigExecWirePropertyValueType::Double: {
        double value = previous.f64;
        if (!part) std::memcpy(&value, &base.bits, sizeof(value));
        _RrRunPart(program, scratch, c, part, value, _RrApplyDouble, diagnostics);
        break;
    }
    case RigExecWirePropertyValueType::Matrix4d:
        _RrRunPart(program, scratch, c, part,
                   part ? previous.matrix : _RrMatrix(base.matrix), _RrApplyMatrix, diagnostics);
        break;
    case RigExecWirePropertyValueType::Vec3f:
        _RrRunPart(program, scratch, c, part,
                   part ? previous.vec : _RrVec3f(base), _RrApplyVec3f, diagnostics);
        break;
    }
    return true;
}

}  // namespace rigExec
