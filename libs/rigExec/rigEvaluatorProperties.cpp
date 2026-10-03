// Property-chain binding and evaluation.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorPropertyBindings.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/propertyMath.h"

#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

bool
_IsFinite(float v)
{
    return std::isfinite(v);
}

bool
_IsFinite(const GfVec3f &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
_IsFinite(const GfMatrix4d &m)
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

/// Binds one input of \p prim, or leaves the binding empty when there is
/// none -- which is what `prim.GetAttribute(...)` returning invalid used to
/// mean at the call site.
RigExecPropertyChainBindings::Input
_BindInput(const UsdPrim &prim, const char *name, UsdTimeCode time)
{
    RigExecPropertyChainBindings::Input input;
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
            // At THIS time, which is what the static cache fills an entry
            // with on its first read. The attribute does not vary, so the
            // time chooses nothing; saying which one was used is what makes
            // that claim checkable.
            a.Get(&input.constantValue, time);
        }
    }
    return input;
}

/// _ResolvedRead through a pinned query.
///
/// Equivalent to it by construction, arm for arm:
///
///  * no attribute -> the fallback, as `prim.GetAttribute()` returning
///    invalid gives;
///  * a connected attribute -> handed to RigExecResolvedInputs::GetAttribute,
///    which is the only code that follows a connection chain;
///  * an in-memory value of the right type for this exact property -> that
///    value, which is the `Get(a.GetPath(), out)` at the head of the walk
///    (and a value of the WRONG type falls through to the stage there too);
///  * otherwise the attribute's own value, which is what the tail of the
///    walk reads and what the query resolves.
///
/// The static-input cache is not consulted on this path. It only ever
/// answers for an attribute with no connections, no time samples and no
/// time-varying opinion, so the value it would hand back is the value the
/// query resolves; and its hit/refusal counters reach no pose.
template <class T>
T
_PinnedRead(const RigExecResolvedInputs &resolved,
            const RigExecPropertyChainBindings::Input &input, T fallback,
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

/// rigExec:operation as `a.Get(&operation)` read it: at Default, off the
/// stage, with no resolved input consulted -- the operation names the
/// arithmetic and not a value.
void
_ReadOperation(const RigExecPropertyChainBindings::Input &input,
               TfToken *operation)
{
    if (input.constant) {
        if (input.constantValue.IsHolding<TfToken>()) {
            *operation = input.constantValue.UncheckedGet<TfToken>();
        }
        return;
    }
    input.query.Get(operation);
}

// Reads the authored inputs of one float/vec3f math mover at \p time.
// Every field is read even though the operation uses only some of them: the
// packet is the mover's whole authored state, and branching on the operation
// while reading would put the same switch in two places.
// rigExec:operation is read at Default with no resolved inputs consulted,
// which is what the unpinned form did: the operation names the arithmetic,
// not a value, and a chain whose arithmetic an override could change is not
// a chain this routine is allowed to be wrong about quietly.
template <class T>
bool
_ReadPinnedPropertyMathParams(
    const RigExecResolvedInputs &resolved,
    const RigExecPropertyChainBindings::Revision &bound, UsdTimeCode time,
    RigExecPropertyMathParams<T> *params)
{
    TfToken operation;
    if (bound.operation) {
        _ReadOperation(bound.operation, &operation);
    }
    if (!RigExecParsePropertyOp(operation, &params->op)) {
        return false;
    }
    params->value =
        _PinnedRead(resolved, bound.value, params->value, time);
    params->min = _PinnedRead(resolved, bound.minimum, params->min, time);
    params->max = _PinnedRead(resolved, bound.maximum, params->max, time);
    return true;
}

} // namespace

void
RigExecRigEvaluator::_EvaluatePropertyChains(
    UsdTimeCode time,
    std::map<SdfPath, VtValue> *results,
    std::vector<RigExecValueOverride> *overrides,
    std::vector<std::string> *diagnostics)
{
    auto diag = [diagnostics](const std::string &message) {
        if (diagnostics) {
            diagnostics->push_back(message);
        }
    };

    // The first frame of an epoch pays for the bindings; every frame after it
    // reads through them. Built here rather than in Compile because a chain
    // is also rebound after a notice that recompiles nothing, and "whatever
    // the stage now says" is the only state this has to describe.
    const auto bind = [this, time](
                          const SdfPath &targetPath,
                          const std::vector<_PropertyRevision> &chainRevisions) {
        RigExecPropertyChainBindings::Chain bound;
        bound.targetPath = targetPath;
        bound.target = _stage->GetAttributeAtPath(bound.targetPath);
        if (bound.target) {
            bound.targetQuery = UsdAttributeQuery(bound.target);
            bound.valueType = bound.target.GetTypeName();
        }
        for (const _PropertyRevision &revision : chainRevisions) {
            RigExecPropertyChainBindings::Revision boundRevision;
            bound.moverPaths.push_back(revision.moverPath);
            boundRevision.moverPrim =
                _stage->GetPrimAtPath(revision.moverPath);
            const UsdPrim &mover = boundRevision.moverPrim;
            if (mover) {
                if (const UsdRelationship rel = mover.GetRelationship(
                        TfToken("rigExec:weightObject"))) {
                    rel.GetTargets(&boundRevision.weightObjects);
                }
            }
            boundRevision.enabled =
                _BindInput(mover, "inputs:enabled", time);
            boundRevision.defaultWeight =
                _BindInput(mover, "inputs:defaultWeight", time);
            boundRevision.operation =
                _BindInput(mover, "rigExec:operation", time);
            boundRevision.value = _BindInput(mover, "inputs:value", time);
            boundRevision.minimum = _BindInput(mover, "inputs:min", time);
            boundRevision.maximum = _BindInput(mover, "inputs:max", time);
            boundRevision.keys = _BindInput(mover, "inputs:keys", time);
            boundRevision.tangents =
                _BindInput(mover, "inputs:tangents", time);
            bound.alwaysDirty =
                bound.alwaysDirty || !boundRevision.weightObjects.empty();
            for (const RigExecPropertyChainBindings::Input *input :
                     {&boundRevision.enabled,
                      &boundRevision.defaultWeight,
                      &boundRevision.operation, &boundRevision.value,
                      &boundRevision.minimum, &boundRevision.maximum,
                      &boundRevision.keys, &boundRevision.tangents}) {
                if (!input->attribute) {
                    continue;
                }
                bound.watch.push_back(input->path);
                if (input->constant) {
                    continue;
                }
                if (!input->connected) {
                    bound.varying = true;  // animated in place
                    continue;
                }
                // Follow the connection chain the read will walk.
                UsdAttribute a = input->attribute;
                std::set<SdfPath> seen;
                while (a && seen.insert(a.GetPath()).second) {
                    bound.watch.push_back(a.GetPath());
                    if (a.ValueMightBeTimeVarying() ||
                        a.GetNumTimeSamples() > 0) {
                        bound.varying = true;
                    }
                    SdfPathVector sources;
                    a.GetConnections(&sources);
                    if (sources.size() != 1) {
                        break;
                    }
                    a = _stage->GetAttributeAtPath(sources[0]);
                    if (!a) {
                        bound.missingSources.push_back(sources[0]);
                    }
                }
            }
            bound.revisions.push_back(std::move(boundRevision));
        }
        if (bound.target && (bound.target.ValueMightBeTimeVarying() ||
                             bound.target.GetNumTimeSamples() > 0)) {
            bound.varying = true;
        }
        return bound;
    };
    // Which chains' targets each chain watches. Recomputed whole whenever a
    // chain is (re)bound, because a rebound chain's walk may reach another
    // target than before; the indices themselves are stable, since chains
    // keep _propertyChainOrder's order.
    const auto link = [](std::vector<RigExecPropertyChainBindings::Chain>
                             *chains) {
        std::unordered_map<SdfPath, size_t, SdfPath::Hash> chainOf;
        for (size_t i = 0; i < chains->size(); ++i) {
            chainOf[(*chains)[i].targetPath] = i;
        }
        for (size_t i = 0; i < chains->size(); ++i) {
            RigExecPropertyChainBindings::Chain &chain = (*chains)[i];
            chain.upstream.clear();
            for (const SdfPath &path : chain.watch) {
                const auto it = chainOf.find(path);
                if (it != chainOf.end() && it->second != i) {
                    chain.upstream.push_back(it->second);
                }
            }
        }
    };
    if (!_propertyChainBindings) {
        _propertyChainBindings =
            std::make_unique<RigExecPropertyChainBindings>();
        for (const SdfPath &orderedTarget : _propertyChainOrder) {
            const auto chainIt = _propertyChains.find(orderedTarget);
            if (chainIt == _propertyChains.end()) {
                continue;
            }
            _propertyChainBindings->chains.push_back(
                bind(chainIt->first, chainIt->second));
        }
        link(&_propertyChainBindings->chains);
    } else if (_propertyChainBindings->anyStale) {
        // Only what a notice reached, rebound as the first frame bound it.
        // A rebound chain has run nothing, so it runs this frame and reports
        // itself changed to every chain downstream of it; every other chain
        // keeps its last answer for as long as nothing it reads moves.
        for (RigExecPropertyChainBindings::Chain &chain :
                 _propertyChainBindings->chains) {
            if (chain.stale) {
                chain = bind(chain.targetPath,
                             _propertyChains.at(chain.targetPath));
            }
        }
        _propertyChainBindings->anyStale = false;
        link(&_propertyChainBindings->chains);
    }

    // What moved since the last run: the time, and each interactive override
    // placed, lifted or changed in value.
    RigExecPropertyChainBindings &bindings = *_propertyChainBindings;
    const bool timeMoved = !bindings.haveLast || time != bindings.lastTime;
    std::unordered_map<SdfPath, VtValue, SdfPath::Hash> nowOverrides;
    for (const RigExecValueOverride &o : _interactiveOverrides) {
        if (!o.attribute.IsEmpty()) {
            nowOverrides[o.prim.AppendProperty(o.attribute)] = o.value;
        }
    }
    std::unordered_set<SdfPath, SdfPath::Hash> overrideMoved;
    for (const auto &[path, value] : nowOverrides) {
        const auto it = bindings.lastOverrides.find(path);
        if (it == bindings.lastOverrides.end() || it->second != value) {
            overrideMoved.insert(path);
        }
    }
    for (const auto &[path, value] : bindings.lastOverrides) {
        if (!nowOverrides.count(path)) {
            overrideMoved.insert(path);
        }
    }
    bindings.haveLast = true;
    bindings.lastTime = time;
    bindings.lastOverrides = std::move(nowOverrides);

    for (RigExecPropertyChainBindings::Chain &chain :
             _propertyChainBindings->chains) {
        const SdfPath &target = chain.targetPath;
        const std::vector<RigExecPropertyChainBindings::Revision> &revisions =
            chain.revisions;
        if (!chain.target) {
            diag("property chain " + target.GetString() +
                 ": target attribute disappeared; chain skipped");
            continue;
        }
        // Clean: nothing the chain reads moved, so the last answer stands.
        // Published and reported exactly as a run would publish and report
        // it, so no consumer can tell the difference.
        bool dirty = !chain.cached || chain.alwaysDirty ||
                     (chain.varying && timeMoved);
        for (size_t k = 0; !dirty && k < chain.watch.size(); ++k) {
            dirty = overrideMoved.count(chain.watch[k]) != 0;
        }
        for (size_t k = 0; !dirty && k < chain.upstream.size(); ++k) {
            dirty = bindings.chains[chain.upstream[k]].changedThisRun;
        }
        if (!dirty) {
            chain.changedThisRun = false;
            for (const std::string &line : chain.lastDiagnostics) {
                diag(line);
            }
            if (chain.published) {
                if (results) {
                    (*results)[target] = chain.lastValue;
                }
                _resolvedInputs.SetProperty(target, chain.lastValue);
                if (overrides) {
                    overrides->push_back(RigExecValueOverride{
                        target.GetPrimPath(), TfToken(),
                        target.GetNameToken(), chain.lastValue});
                }
            }
            continue;
        }
        const size_t diagnosticsBefore =
            diagnostics ? diagnostics->size() : 0;
        _resolvedInputs.ClearProperty(target);
        struct _Remember {
            RigExecPropertyChainBindings::Chain &chain;
            RigExecResolvedInputs &resolved;
            std::vector<std::string> *diagnostics;
            size_t before;
            ~_Remember() {
                const VtValue *now = resolved.Find(chain.targetPath);
                const bool published = now != nullptr;
                chain.changedThisRun =
                    !chain.cached || published != chain.published ||
                    (published && *now != chain.lastValue);
                chain.published = published;
                chain.lastValue = published ? *now : VtValue();
                chain.lastDiagnostics.clear();
                if (diagnostics) {
                    chain.lastDiagnostics.assign(
                        diagnostics->begin() + long(before),
                        diagnostics->end());
                }
                chain.cached = true;
            }
        } remember{chain, _resolvedInputs, diagnostics, diagnosticsBefore};
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler, "PropertyChain " + target.GetString(), "property");
        const SdfValueTypeName &valueType = chain.valueType;

        // One shared revision loop over the three value domains. Each
        // iteration reads the mover's own authored state and applies it to
        // the preceding revision -- the base being the target's AUTHORED
        // value, exactly as a point chain's base is the target's authored
        // points.
        auto runChain = [&](auto value, auto apply) {
            using ValueT = decltype(value);
            if (!chain.targetQuery.Get(&value, time)) {
                diag("property chain " + target.GetString() +
                     ": target has no authored value; chain skipped");
                return false;
            }
            if (!_IsFinite(value)) {
                diag("property chain " + target.GetString() +
                     ": authored base is not finite; chain skipped");
                return false;
            }
            for (const RigExecPropertyChainBindings::Revision &revision :
                     revisions) {
                const UsdPrim &moverPrim = revision.moverPrim;
                if (!moverPrim) {
                    continue;
                }
                const SdfPath moverPath = moverPrim.GetPath();
                const bool enabled = _PinnedRead(
                    _resolvedInputs, revision.enabled, true, time);
                if (!enabled) {
                    diag("diag " + moverPath.GetString() +
                         ": disabled; revision passed through");
                    continue;  // ordinary pass-through (spec §6.6)
                }
                float envelope = 1.0f;
                const SdfPathVector &weightObjects = revision.weightObjects;
                if (!weightObjects.empty()) {
                    std::vector<float> weights;
                    std::string error;
                    if (!_ResolveWeights(weightObjects[0], 1, time,
                                         &weights, &error) ||
                        weights.size() != 1) {
                        diag("diag " + moverPath.GetString() +
                             ": " + error + "; revision passed through");
                        continue;
                    }
                    envelope = weights[0];
                } else {
                    envelope = _PinnedRead(
                        _resolvedInputs, revision.defaultWeight, 1.0f, time);
                    if (!std::isfinite(envelope) || envelope < 0.0f ||
                        envelope > 1.0f) {
                        diag("diag " + moverPath.GetString() +
                             ": inputs:defaultWeight must be finite and in "
                             "[0, 1]; revision passed through");
                        continue;
                    }
                }
                ValueT next = value;
                if (!apply(revision, value, envelope, &next)) {
                    diag("diag " + moverPath.GetString() +
                         ": inputs unusable; revision passed through");
                    continue;
                }
                if (!_IsFinite(next)) {
                    // A NaN reaching an exec override propagates into every
                    // consumer of the attribute with no way to report it
                    // back, so the mover fails and passes through instead
                    // (spec §6.6).
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
            // Publish immediately, not after every chain has run. Dependency
            // ordering guarantees that any later chain which consumes this
            // property reads the revised value.
            _resolvedInputs.SetProperty(target, VtValue(value));
            if (overrides) {
                overrides->push_back(RigExecValueOverride{
                    target.GetPrimPath(), TfToken(), target.GetNameToken(),
                    VtValue(value)});
            }
            return true;
        };

        using BoundRevision = RigExecPropertyChainBindings::Revision;
        const auto applyFloat = [&](const BoundRevision &mover, float in,
                                    float envelope, float *out) {
                RigExecPropertyMathParams<float> params;
                if (!_ReadPinnedPropertyMathParams(
                        _resolvedInputs, mover, time, &params) ||
                    !_IsFinite(params.value) || !_IsFinite(params.min) ||
                    !_IsFinite(params.max)) {
                    return false;
                }
                // Held here so the borrowed key pointer outlives the apply.
                VtArray<GfVec2f> keys;
                VtArray<GfVec2f> tangents;
                if (params.op == RigExecPropertyOp::Curve) {
                    keys = _PinnedRead(
                        _resolvedInputs, mover.keys, keys, time);
                    if (keys.empty() || !RigExecValidateLinearKeys(
                                            keys.cdata(), keys.size())) {
                        return false;
                    }
                    params.keys = keys.cdata();
                    params.keyCount = keys.size();
                    if (mover.tangents) {
                        tangents = _PinnedRead(
                            _resolvedInputs, mover.tangents, tangents, time);
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
        if (valueType == SdfValueTypeNames->Float) {
            runChain(float(0), applyFloat);
        } else if (valueType == SdfValueTypeNames->Double) {
            runChain(double(0), [&](const BoundRevision &mover, double in,
                                    float envelope, double *out) {
                float result = 0.0f;
                if (!applyFloat(mover, float(in), envelope, &result)) {
                    return false;
                }
                *out = double(result);
                return true;
            });
        } else if (valueType == SdfValueTypeNames->Matrix4d) {
            runChain(GfMatrix4d(1.0), [&](const BoundRevision &mover,
                                          const GfMatrix4d &in,
                                          float envelope,
                                          GfMatrix4d *out) {
                TfToken operation;
                if (mover.operation) {
                    _ReadOperation(mover.operation, &operation);
                }
                RigExecPropertyOp op;
                if (!RigExecParsePropertyOp(operation, &op)) {
                    return false;
                }
                const GfMatrix4d opValue = _PinnedRead(
                    _resolvedInputs, mover.value, GfMatrix4d(1.0), time);
                if (!_IsFinite(opValue)) {
                    return false;
                }
                return RigExecApplyMatrixMath(
                    in, op, opValue, envelope, out);
            });
        } else {
            // Every remaining type the compiler admits is GfVec3f-backed.
            runChain(GfVec3f(0), [&](const BoundRevision &mover,
                                     const GfVec3f &in, float envelope,
                                     GfVec3f *out) {
                RigExecPropertyMathParams<GfVec3f> params;
                if (!_ReadPinnedPropertyMathParams(
                        _resolvedInputs, mover, time, &params) ||
                    !_IsFinite(params.value) || !_IsFinite(params.min) ||
                    !_IsFinite(params.max)) {
                    return false;
                }
                params.weight = envelope;
                *out = RigExecApplyVec3fMath(in, params);
                return true;
            });
        }
    }
}

bool
RigExecRigEvaluator::_CompilePropertyChains(
    const std::vector<RigExecMoverRecord> &newMovers,
    std::map<SdfPath, std::vector<_PropertyRevision>> &newPropertyChains,
    std::vector<SdfPath> &newPropertyChainOrder, _CompileFailure *failure) const
{
    const auto fail = [failure](const std::string &message,
                                SdfPathVector operations = {}) {
        *failure = {message, std::move(operations)};
        return false;
    };
    for (const RigExecMoverRecord &mover : newMovers) {
        if (!RigExecIsPropertyMover(mover.schemaType)) {
            continue;
        }
        // Validation above guarantees exactly one exact property target of
        // the matching type.
        for (const SdfPath &target : mover.targets) {
            newPropertyChains[target].push_back(
                _PropertyRevision{mover.moverPath, mover.schemaType});
        }
    }

    // A property chain can revise an input consumed by another property
    // chain. Resolve those producers first; namespace/map order is unrelated
    // to dataflow and made `/Consumer` read its authored envelope before a
    // lexically later `/Driver` had produced the revised one.
    {
        std::map<SdfPath, std::set<SdfPath>> dependsOn;
        for (const auto &[target, _] : newPropertyChains) {
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
                if (newPropertyChains.count(a.GetPath())) {
                    dependsOn[consumer].insert(a.GetPath());
                }
                const SdfPathVector connections = _AuthoredConnections(a);
                for (const SdfPath &sourcePath : connections) {
                    walk(_stage->GetAttributeAtPath(sourcePath));
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
        std::function<void(const SdfPath &, const SdfPath &,
                           std::set<SdfPath> *)>
            addWeightDependencies =
                [&](const SdfPath &consumer, const SdfPath &weightPath,
                    std::set<SdfPath> *visited) {
                if (!visited->insert(weightPath).second) {
                    return;
                }
                const UsdPrim weight = _stage->GetPrimAtPath(weightPath);
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

        for (const auto &[target, revisions] : newPropertyChains) {
            for (const _PropertyRevision &revision : revisions) {
                const UsdPrim mover =
                    _stage->GetPrimAtPath(revision.moverPath);
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
        std::function<bool(const SdfPath &)> visit =
            [&](const SdfPath &target) {
            colour[target] = 1;
            stack.push_back(target);
            for (const SdfPath &producer : dependsOn[target]) {
                if (colour[producer] == 1) {
                    std::string cycle;
                    for (const SdfPath &path : stack) {
                        cycle += path.GetString() + " -> ";
                    }
                    cycle += producer.GetString();
                    SdfPathVector operations;
                    const auto first = std::find(stack.begin(), stack.end(), producer);
                    for (auto it = first; it != stack.end(); ++it) {
                        for (const auto &revision : newPropertyChains.at(*it)) {
                            operations.push_back(revision.moverPath);
                        }
                    }
                    return fail("property input dependency cycle: " + cycle,
                                std::move(operations));
                }
                if (colour[producer] == 0 && !visit(producer)) {
                    return false;
                }
            }
            stack.pop_back();
            colour[target] = 2;
            newPropertyChainOrder.push_back(target);
            return true;
        };
        for (const auto &[target, _] : dependsOn) {
            if (colour[target] == 0 && !visit(target)) {
                return false;
            }
        }
    }

    return true;
}

} // namespace rigExec
