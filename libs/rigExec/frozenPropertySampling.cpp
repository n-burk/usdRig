// Stage-side property-chain discovery and binding, for the freeze gate and
// the samplers' epoch currency check.

#include "frozenContextInternal.h"
#include "movers/moverRegistry.h"
#include <algorithm>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace {

bool
_IsChainMathMover(const TfToken &schemaType)
{
    return RigExecIsPropertyMover(schemaType);
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
    // The same phased reads, each over the same hops: a phase edit or a
    // rewire re-epochs the evaluator, which resolves them afresh, and the
    // hops decide who stands aside for a drag.
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
                chain.phased[k].applied != connection.applied ||
                chain.phased[k].final != connection.final ||
                chain.phased[k].hops != connection.hops) {
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
RigExecSampleFrameInputs(const RigExecRigEvaluator &evaluator,
                         UsdTimeCode time,
                         const std::vector<RigExecValueOverride> &overrides,
                         RigExecFrameInputs *out, std::string *error)
{
    // Bind fresh, then sample through the pinned route: the two routes
    // share every line below the bind, and the equivalence test holds them
    // to account sample by sample. A bind failure names no chains at all,
    // so no vector is built.
    return RigExecSampleFrameInputs(evaluator, time, overrides, {}, out,
                                    error);
}

bool
RigExecSampleFrameInputs(const RigExecRigEvaluator &evaluator,
                         UsdTimeCode time,
                         const std::vector<RigExecValueOverride> &overrides,
                         const std::vector<RigExecUpstreamValue> &upstream,
                         RigExecFrameInputs *out, std::string *error)
{
    RigExecChainSampleBindings fresh;
    if (!RigExecBindChainSampleInputs(evaluator, &fresh, error)) {
        return false;
    }
    return RigExecSampleFrameInputsWithChainBindings(
        evaluator, time, overrides, upstream, fresh, out, error);
}

} // namespace rigExec
