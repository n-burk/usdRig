// Stage-side property-chain discovery, binding, and evaluation.

#include "frozenContextInternal.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/propertyMath.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace {

// NOTE: no double overload, exactly as on the live path: a double base is
// checked through the float conversion, so a finite double past float range
// skips the chain on both paths alike.

bool
_IsChainMathMover(const TfToken &schemaType)
{
    return RigExecIsPropertyMover(schemaType);
}

// Mirror of _PinnedRead, arm for arm: no attribute answers the fallback; a
// connected attribute walks the resolved inputs (the return ignored, so a
// failed walk keeps the fallback); an in-memory value of the read type
// wins; a folded constant answers; otherwise the pinned query resolves.
template <class T>
T
_ChainPinnedRead(const RigExecResolvedInputs &resolved,
                 const RigExecChainSampleInput &input, T fallback,
                 UsdTimeCode time)
{
    T value = fallback;
    if (!input.attribute) {
        return value;
    }
    if (input.connected) {
        resolved.GetAttribute(input.attribute, time, &value);
        return value;
    }
    if (const VtValue *const standing = resolved.Find(input.path)) {
        if (standing->IsHolding<T>()) {
            return standing->UncheckedGet<T>();
        }
    }
    if (input.constant && input.constantValue.IsHolding<T>()) {
        return input.constantValue.UncheckedGet<T>();
    }
    T resolvedValue;
    if (input.query.Get(&resolvedValue, time)) {
        value = resolvedValue;
    }
    return value;
}

// Mirror of _ReadOperation: the operation names the arithmetic, not a
// value, so it is read at Default with no resolved inputs consulted.
void
_ChainReadOperation(const RigExecChainSampleInput &input, TfToken *operation)
{
    if (input.constant) {
        if (input.constantValue.IsHolding<TfToken>()) {
            *operation = input.constantValue.UncheckedGet<TfToken>();
        }
        return;
    }
    input.query.Get(operation);
}

// Mirror of _ReadPinnedPropertyMathParams: the mover's whole authored
// state, read through the pinned inputs whether or not the operation uses
// every field.
template <class T>
bool
_ChainReadMathParams(const RigExecResolvedInputs &resolved,
                     const RigExecChainSampleRevision &mover, UsdTimeCode time,
                     RigExecPropertyMathParams<T> *params)
{
    TfToken operation;
    if (mover.operation) {
        _ChainReadOperation(mover.operation, &operation);
    }
    if (!RigExecParsePropertyOp(operation, &params->op)) {
        return false;
    }
    params->value =
        _ChainPinnedRead(resolved, mover.value, params->value, time);
    params->min = _ChainPinnedRead(resolved, mover.minimum, params->min, time);
    params->max = _ChainPinnedRead(resolved, mover.maximum, params->max, time);
    return true;
}

// The eight inputs a revision pins, by attribute name, for the currency
// check below.
const char *const _kChainInputNames[8] = {
    "inputs:enabled", "inputs:defaultWeight", "rigExec:operation",
    "inputs:value", "inputs:min", "inputs:max", "inputs:keys",
    "inputs:tangents"};

const RigExecChainSampleInput *
_ChainBoundInput(const RigExecChainSampleRevision &revision, size_t i)
{
    switch (i) {
    case 0: return &revision.enabled;
    case 1: return &revision.defaultWeight;
    case 2: return &revision.operation;
    case 3: return &revision.value;
    case 4: return &revision.minimum;
    case 5: return &revision.maximum;
    case 6: return &revision.keys;
    default: return &revision.tangents;
    }
}

} // namespace

namespace frozenDetail {

bool
_ChainIsFinite(const GfVec3f &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
_ChainIsFinite(const GfMatrix4d &m)
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

// Mirror of the compile pass that fills the evaluator's property chains:
// every math mover in mover order, each over its one exact property
// target. Validation guarantees the shape for a compiled rig; anything
// else fails the bind, never the evaluation.
bool
_DiscoverChainMovers(const RigExecRigEvaluator &evaluator,
                     std::vector<_ChainMoverDesc> *out, std::string *error)
{
    for (const RigExecMoverRecord &record : evaluator.GetMoverOrder()) {
        if (!_IsChainMathMover(record.schemaType)) {
            continue;
        }
        if (record.targets.size() != 1 ||
            !record.targets[0].IsPropertyPath()) {
            if (error) {
                *error = record.schemaType.GetString() + " " +
                         record.moverPath.GetString() +
                         " has no single exact property target";
            }
            return false;
        }
        _ChainMoverDesc desc;
        desc.moverPath = record.moverPath;
        desc.schemaType = record.schemaType;
        desc.target = record.targets[0];
        out->push_back(desc);
    }
    return true;
}

// Mirror of the compile pass that orders the chains: producers first, over
// the same dependency edges (every mover attribute's connection walk, plus
// the weight prims behind rigExec:weightObject). Same walk, same sets,
// same depth-first visit -- so the same order, deterministically.
bool
_OrderDiscoveredChains(
    const UsdStageRefPtr &stage,
    const std::map<SdfPath, std::vector<_ChainMoverDesc>> &chains,
    std::vector<SdfPath> *order, std::string *error)
{
    std::map<SdfPath, std::set<SdfPath>> dependsOn;
    for (const auto &[target, _] : chains) {
        dependsOn[target];
    }
    auto addAttributeDependency = [&](const SdfPath &consumer,
                                      const UsdAttribute &attribute) {
        std::set<SdfPath> visited;
        std::function<void(const UsdAttribute &)> walk =
            [&](const UsdAttribute &a) {
            if (!a || !visited.insert(a.GetPath()).second) {
                return;
            }
            if (chains.count(a.GetPath())) {
                dependsOn[consumer].insert(a.GetPath());
            }
            SdfPathVector connections;
            if (a.HasAuthoredConnections()) {
                a.GetConnections(&connections);
            }
            for (const SdfPath &sourcePath : connections) {
                walk(stage->GetAttributeAtPath(sourcePath));
            }
        };
        walk(attribute);
    };
    auto addPrimDependencies = [&](const SdfPath &consumer,
                                   const UsdPrim &prim) {
        if (!prim) {
            return;
        }
        for (const UsdAttribute &attribute : prim.GetAttributes()) {
            addAttributeDependency(consumer, attribute);
        }
    };
    std::function<void(const SdfPath &, const SdfPath &, std::set<SdfPath> *)>
        addWeightDependencies =
            [&](const SdfPath &consumer, const SdfPath &weightPath,
                std::set<SdfPath> *visited) {
            if (!visited->insert(weightPath).second) {
                return;
            }
            const UsdPrim weight = stage->GetPrimAtPath(weightPath);
            addPrimDependencies(consumer, weight);
            for (const char *relationship :
                 {"rigExec:inputWeights", "rigExec:baseWeight"}) {
                SdfPathVector inputs;
                if (const UsdRelationship rel =
                        weight.GetRelationship(TfToken(relationship))) {
                    rel.GetTargets(&inputs);
                }
                for (const SdfPath &input : inputs) {
                    addWeightDependencies(consumer, input, visited);
                }
            }
        };

    for (const auto &[target, revisions] : chains) {
        for (const _ChainMoverDesc &revision : revisions) {
            const UsdPrim mover = stage->GetPrimAtPath(revision.moverPath);
            addPrimDependencies(target, mover);
            SdfPathVector weights;
            if (const UsdRelationship rel = mover.GetRelationship(
                    TfToken("rigExec:weightObject"))) {
                rel.GetTargets(&weights);
            }
            std::set<SdfPath> visitedWeights;
            for (const SdfPath &weight : weights) {
                addWeightDependencies(target, weight, &visitedWeights);
            }
        }
    }

    std::map<SdfPath, int> colour;
    std::vector<SdfPath> stack;
    std::function<bool(const SdfPath &)> visit = [&](const SdfPath &target) {
        colour[target] = 1;
        stack.push_back(target);
        for (const SdfPath &producer : dependsOn[target]) {
            if (colour[producer] == 1) {
                if (error) {
                    std::string cycle;
                    for (const SdfPath &path : stack) {
                        cycle += path.GetString() + " -> ";
                    }
                    cycle += producer.GetString();
                    *error = "property input dependency cycle: " + cycle;
                }
                return false;
            }
            if (colour[producer] == 0 && !visit(producer)) {
                return false;
            }
        }
        stack.pop_back();
        colour[target] = 2;
        order->push_back(target);
        return true;
    };
    for (const auto &[target, _] : dependsOn) {
        if (colour[target] == 0 && !visit(target)) {
            return false;
        }
    }
    return true;
}

// Mirror of _BindInput: one chain input pinned, or empty when the mover
// authors none. Constants fold at Default rather than at the live path's
// first-run time; a constant reads identically at every time code, so the
// pins agree whatever frame ran first.
RigExecChainSampleInput
_BindChainInput(const UsdPrim &prim, const char *name)
{
    RigExecChainSampleInput input;
    if (!prim) {
        return input;
    }
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        input.attribute = a;
        input.path = a.GetPath();
        input.connected = a.HasAuthoredConnections();
        input.query = UsdAttributeQuery(a);
        input.constant = !input.connected && !a.ValueMightBeTimeVarying() &&
                         a.GetNumTimeSamples() == 0;
        if (input.constant) {
            a.Get(&input.constantValue, UsdTimeCode::Default());
        }
    }
    return input;
}

// Within-process mixing (never persisted, never compared across runs, never
// baked into tests): a word-at-a-time splitmix64 finalizer, the same mixer
// the frame-cache digest uses in frameCache.cpp -- identical logical
// stream, one mix per 8 bytes instead of one FNV round per byte, at roughly
// a third of the multiplies.
uint64_t
_MixWord(uint64_t hash, uint64_t word)
{
    // splitmix64 finalizer for the word, FNV-style chaining for the state.
    // The multiply matters: an xor-only fold is commutative, so duplicate
    // words cancel anywhere in the stream ([W,W] folds to the seed whatever
    // W is) and permutations collide -- both serve stale poses.
    word += 0x9E3779B97F4A7C15ull;
    word = (word ^ (word >> 30)) * 0xBF58476D1CE4E5B9ull;
    word = (word ^ (word >> 27)) * 0x94D049BB133111EBull;
    word ^= word >> 31;
    hash ^= word;
    hash *= 1099511628211ull;
    return hash;
}

} // namespace frozenDetail

bool
RigExecChainSampleBindingsStillCurrent(
    const RigExecChainSampleBindings &bindings,
    const RigExecRigEvaluator &evaluator)
{
    const UsdStageRefPtr stage = evaluator.GetEvaluationStage();
    if (!stage) {
        return false;
    }
    // The discovery signature: the same math movers over the same targets.
    // Mover membership is epoch-digest-covered, so a mismatch also means a
    // new epoch -- but the check is cheap and the failure mode of trusting
    // it is a silently misbound chain.
    std::vector<_ChainMoverDesc> movers;
    std::string ignored;
    if (!_DiscoverChainMovers(evaluator, &movers, &ignored)) {
        return false;
    }
    std::map<SdfPath, std::vector<_ChainMoverDesc>> grouped;
    for (const _ChainMoverDesc &mover : movers) {
        grouped[mover.target].push_back(mover);
    }
    if (grouped.size() != bindings.chains.size()) {
        return false;
    }
    // The same phased reads: a phase edit re-epochs the evaluator, which
    // resolves them afresh.
    size_t phasedBound = 0;
    for (const RigExecChainSampleChain &chain : bindings.chains) {
        phasedBound += chain.phased.size();
    }
    const std::vector<RigExecPhasedConnection> &phasedNow =
        evaluator.GetPhasedConnections();
    if (phasedBound != phasedNow.size()) {
        return false;
    }
    for (const RigExecChainSampleChain &chain : bindings.chains) {
        size_t k = 0;
        for (const RigExecPhasedConnection &connection : phasedNow) {
            if (connection.target != chain.targetPath) {
                continue;
            }
            if (k >= chain.phased.size() ||
                chain.phased[k].consumer != connection.consumer ||
                chain.phased[k].consumerType != connection.consumerType ||
                chain.phased[k].applied != connection.applied) {
                return false;
            }
            ++k;
        }
        if (k != chain.phased.size()) {
            return false;
        }
    }
    for (const RigExecChainSampleChain &chain : bindings.chains) {
        const auto found = grouped.find(chain.targetPath);
        if (found == grouped.end() ||
            found->second.size() != chain.revisions.size()) {
            return false;
        }
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            if (found->second[i].moverPath !=
                    chain.revisions[i].moverPath ||
                found->second[i].schemaType !=
                    chain.revisions[i].moverPrim.GetTypeName()) {
                return false;
            }
        }
        // The target resolves as it did, to the same type: the evaluation
        // dispatches on the target's type, and a mid-epoch retype moves no
        // epoch digest.
        const UsdAttribute freshTarget =
            stage->GetAttributeAtPath(chain.targetPath);
        if (bool(freshTarget) != bool(chain.target)) {
            return false;
        }
        if (freshTarget && freshTarget.GetTypeName() != chain.valueType) {
            return false;
        }
        for (const RigExecChainSampleRevision &revision : chain.revisions) {
            const UsdPrim mover = stage->GetPrimAtPath(revision.moverPath);
            if (bool(mover) != bool(revision.moverPrim)) {
                return false;
            }
            SdfPathVector weights;
            if (const UsdRelationship rel = mover.GetRelationship(
                    TfToken("rigExec:weightObject"))) {
                rel.GetTargets(&weights);
            }
            if (weights != revision.weightObjects) {
                return false;
            }
            // Every pin: still present iff pinned, still classified as
            // pinned, and a folded constant still reading its pinned value.
            // Value edits move no epoch digest, so without this a constant
            // edited mid-epoch would read stale until the next recompile.
            for (size_t i = 0; i < 8; ++i) {
                const RigExecChainSampleInput *pinned =
                    _ChainBoundInput(revision, i);
                const UsdAttribute fresh = mover.GetAttribute(
                    TfToken(_kChainInputNames[i]));
                if (bool(fresh) != bool(*pinned)) {
                    return false;
                }
                if (!fresh) {
                    continue;
                }
                const bool connected = fresh.HasAuthoredConnections();
                const bool constant =
                    !connected && !fresh.ValueMightBeTimeVarying() &&
                    fresh.GetNumTimeSamples() == 0;
                if (connected != pinned->connected ||
                    constant != pinned->constant) {
                    return false;
                }
                if (constant) {
                    VtValue now;
                    fresh.Get(&now, UsdTimeCode::Default());
                    if (now != pinned->constantValue) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

bool
RigExecEvaluateChainsForTime(
    const RigExecChainSampleBindings &bindings, UsdTimeCode time,
    RigExecResolvedInputs *resolved, std::map<SdfPath, VtValue> *results,
    std::vector<std::string> *diagnostics, std::string *error)
{
    if (!resolved) {
        if (error) {
            *error = "no resolved inputs to evaluate the chains into";
        }
        return false;
    }
    auto diag = [diagnostics](const std::string &message) {
        if (diagnostics) {
            diagnostics->push_back(message);
        }
    };

    for (const RigExecChainSampleChain &chain : bindings.chains) {
        const SdfPath &target = chain.targetPath;
        if (!chain.target) {
            diag("property chain " + target.GetString() +
                 ": target attribute disappeared; chain skipped");
            continue;
        }
        resolved->ClearProperty(target);
        for (const RigExecPhasedConnection &phased : chain.phased) {
            resolved->ClearProperty(phased.consumer);
        }
        const SdfValueTypeName &valueType = chain.valueType;

        // One shared revision loop over the three value domains, as on the
        // live path: each iteration reads the mover's own authored state
        // and applies it to the preceding revision, from the target's
        // AUTHORED base value.
        auto runChain = [&](auto value, auto apply) {
            using ValueT = decltype(value);
            if (!chain.targetQuery.Get(&value, time)) {
                diag("property chain " + target.GetString() +
                     ": target has no authored value; chain skipped");
                return true;
            }
            if (!_ChainIsFinite(value)) {
                diag("property chain " + target.GetString() +
                     ": authored base is not finite; chain skipped");
                return true;
            }
            // The value after each revision, base first, for the phased
            // consumers, as on the live path.
            std::vector<ValueT> history;
            if (!chain.phased.empty()) {
                history.reserve(chain.revisions.size() + 1);
            }
            for (const RigExecChainSampleRevision &revision :
                 chain.revisions) {
                if (!chain.phased.empty()) {
                    history.push_back(value);
                }
                const UsdPrim &moverPrim = revision.moverPrim;
                if (!moverPrim) {
                    continue;
                }
                const SdfPath moverPath = moverPrim.GetPath();
                const bool enabled = _ChainPinnedRead(
                    *resolved, revision.enabled, true, time);
                if (!enabled) {
                    diag("diag " + moverPath.GetString() +
                         ": disabled; revision passed through");
                    continue;
                }
                float envelope = 1.0f;
                if (!revision.weightObjects.empty()) {
                    if (error) {
                        *error =
                            "diag " + moverPath.GetString() +
                            " binds a weight object, whose envelope resolves "
                            "through the evaluator's live oracle";
                    }
                    return false;
                }
                envelope = _ChainPinnedRead(
                    *resolved, revision.defaultWeight, 1.0f, time);
                if (!std::isfinite(envelope) || envelope < 0.0f ||
                    envelope > 1.0f) {
                    diag("diag " + moverPath.GetString() +
                         ": inputs:defaultWeight must be finite and in "
                         "[0, 1]; revision passed through");
                    continue;
                }
                ValueT next = value;
                if (!apply(revision, value, envelope, &next)) {
                    diag("diag " + moverPath.GetString() +
                         ": inputs unusable; revision passed through");
                    continue;
                }
                if (!_ChainIsFinite(next)) {
                    diag("diag " + moverPath.GetString() +
                         ": produced a non-finite value; revision passed "
                         "through");
                    continue;
                }
                value = next;
            }
            if (results) {
                (*results)[target] = VtValue(value);
            }
            resolved->SetProperty(target, VtValue(value));
            if (!chain.phased.empty()) {
                history.push_back(value);
                for (const RigExecPhasedConnection &phased : chain.phased) {
                    const VtValue read = RigExecPhasedConsumerValue(
                        VtValue(history[std::min(phased.applied,
                                                 history.size() - 1)]),
                        phased.consumerType);
                    if (results) {
                        (*results)[phased.consumer] = read;
                    }
                    resolved->SetProperty(phased.consumer, read);
                }
            }
            return true;
        };

        const auto applyFloat = [&](const RigExecChainSampleRevision &mover,
                                    float in, float envelope, float *out) {
            RigExecPropertyMathParams<float> params;
            if (!_ChainReadMathParams(*resolved, mover, time, &params) ||
                !_ChainIsFinite(params.value) || !_ChainIsFinite(params.min) ||
                !_ChainIsFinite(params.max)) {
                return false;
            }
            VtArray<GfVec2f> keys;
            VtArray<GfVec2f> tangents;
            if (params.op == RigExecPropertyOp::Curve) {
                keys = _ChainPinnedRead(*resolved, mover.keys, keys, time);
                if (keys.empty() || !RigExecValidateLinearKeys(
                                        keys.cdata(), keys.size())) {
                    return false;
                }
                params.keys = keys.cdata();
                params.keyCount = keys.size();
                if (mover.tangents) {
                    tangents = _ChainPinnedRead(
                        *resolved, mover.tangents, tangents, time);
                }
                if (!tangents.empty()) {
                    if (tangents.size() != keys.size()) {
                        return false;
                    }
                    params.tangents = tangents.cdata();
                    params.tangentCount = tangents.size();
                }
            }
            params.weight = envelope;
            *out = RigExecApplyFloatMath(in, params);
            return true;
        };
        bool chainOk = true;
        if (valueType == SdfValueTypeNames->Float) {
            chainOk = runChain(float(0), applyFloat);
        } else if (valueType == SdfValueTypeNames->Double) {
            chainOk = runChain(
                double(0),
                [&](const RigExecChainSampleRevision &mover, double in,
                    float envelope, double *out) {
                    float result = 0.0f;
                    if (!applyFloat(mover, float(in), envelope, &result)) {
                        return false;
                    }
                    *out = double(result);
                    return true;
                });
        } else if (valueType == SdfValueTypeNames->Matrix4d) {
            chainOk = runChain(
                GfMatrix4d(1.0),
                [&](const RigExecChainSampleRevision &mover,
                    const GfMatrix4d &in, float envelope, GfMatrix4d *out) {
                    TfToken operation;
                    if (mover.operation) {
                        _ChainReadOperation(mover.operation, &operation);
                    }
                    RigExecPropertyOp op;
                    if (!RigExecParsePropertyOp(operation, &op)) {
                        return false;
                    }
                    const GfMatrix4d opValue = _ChainPinnedRead(
                        *resolved, mover.value, GfMatrix4d(1.0), time);
                    if (!_ChainIsFinite(opValue)) {
                        return false;
                    }
                    return RigExecApplyMatrixMath(
                        in, op, opValue, envelope, out);
                });
        } else {
            // Every remaining type the compiler admits is GfVec3f-backed.
            chainOk = runChain(
                GfVec3f(0),
                [&](const RigExecChainSampleRevision &mover, const GfVec3f &in,
                    float envelope, GfVec3f *out) {
                    RigExecPropertyMathParams<GfVec3f> params;
                    if (!_ChainReadMathParams(
                            *resolved, mover, time, &params) ||
                        !_ChainIsFinite(params.value) ||
                        !_ChainIsFinite(params.min) ||
                        !_ChainIsFinite(params.max)) {
                        return false;
                    }
                    params.weight = envelope;
                    *out = RigExecApplyVec3fMath(in, params);
                    return true;
                });
        }
        if (!chainOk) {
            return false;
        }
    }
    return true;
}

bool
RigExecSampleFrameInputs(const RigExecRigEvaluator &evaluator,
                         UsdTimeCode time,
                         const std::vector<RigExecValueOverride> &overrides,
                         RigExecFrameInputs *out, std::string *error)
{
    // Bind fresh, then sample through the pinned route: the two routes
    // share every line below the bind, and the equivalence test holds them
    // to account sample by sample. A bind failure names no chains at all,
    // so no vector is built; an evaluation decline (a weight object) still
    // builds a marked vector that declines downstream.
    RigExecChainSampleBindings fresh;
    if (!RigExecBindChainSampleInputs(evaluator, &fresh, error)) {
        return false;
    }
    return RigExecSampleFrameInputsWithChainBindings(
        evaluator, time, overrides, fresh, out, error);
}

} // namespace rigExec
