// Property-chain binding and evaluation.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorPropertyBindings.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/propertyMath.h"

#include "pxr/base/tf/type.h"
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

/// Whether an input of \p consumer type can read a chain on a \p target of
/// that type at a phase: the same value type -- a color3f input reads a
/// float3 chain -- or float and double either way round, which
/// RigExecPhasedConsumerValue converts.
bool
_PhasedTypesAgree(const SdfValueTypeName &consumer,
                  const SdfValueTypeName &target)
{
    if (!consumer || !target) {
        return false;
    }
    const TfType consumerValue = consumer.GetType();
    const TfType targetValue = target.GetType();
    if (consumerValue == targetValue) {
        return true;
    }
    const auto scalar = [](const TfType &type) {
        return type == TfType::Find<float>() || type == TfType::Find<double>();
    };
    return scalar(consumerValue) && scalar(targetValue);
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
        for (const RigExecPhasedConnection &connection : _phasedConnections) {
            if (connection.target == targetPath) {
                bound.phased.push_back(
                    RigExecPropertyChainBindings::Chain::Phased{
                        connection.consumer, connection.consumerType,
                        connection.applied, connection.hops,
                        connection.final});
            }
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
    const auto &overridden = bindings.lastOverrides;
    // Whether a phased reader stands aside this run: an interactive override
    // on its consumer or on a hop before the target is what the overlay walk
    // meets first, so publishing the chain there would hide it.
    const auto readerOverridden =
        [&overridden](const RigExecPropertyChainBindings::Chain::Phased &p) {
            for (const SdfPath &hop : p.hops) {
                if (overridden.count(hop)) {
                    return true;
                }
            }
            return false;
        };

    // One value onto one property, through the three routes a chain result
    // takes: the published results, the generation's resolved inputs, and
    // the exec overrides.
    const auto publish = [&](const SdfPath &path, const VtValue &value) {
        if (results) {
            (*results)[path] = value;
        }
        _resolvedInputs.SetProperty(path, value);
        if (overrides) {
            overrides->push_back(RigExecValueOverride{
                path.GetPrimPath(), TfToken(), path.GetNameToken(), value});
        }
    };

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
        // What the chain publishes also turns on overrides its inputs do
        // not reach: one on the target, which stands for the final value,
        // and one on a phased reader's hops, which stands it aside.
        if (!dirty) {
            dirty = overrideMoved.count(target) != 0;
            for (size_t k = 0; !dirty && k < chain.phased.size(); ++k) {
                for (const SdfPath &hop : chain.phased[k].hops) {
                    if (overrideMoved.count(hop)) {
                        dirty = true;
                        break;
                    }
                }
            }
        }
        if (!dirty) {
            chain.changedThisRun = false;
            for (const std::string &line : chain.lastDiagnostics) {
                diag(line);
            }
            if (chain.published) {
                publish(target, chain.lastValue);
            }
            for (size_t k = 0; k < chain.lastPhased.size(); ++k) {
                if (!chain.lastPhased[k].IsEmpty()) {
                    publish(chain.phased[k].consumer, chain.lastPhased[k]);
                }
            }
            continue;
        }
        const size_t diagnosticsBefore =
            diagnostics ? diagnostics->size() : 0;
        _resolvedInputs.ClearProperty(target);
        // What each phased reader publishes this run, empty where it stands
        // aside; a reader standing aside keeps its override in place.
        std::vector<VtValue> phasedPublished(chain.phased.size());
        for (const auto &phased : chain.phased) {
            if (!readerOverridden(phased)) {
                _resolvedInputs.ClearProperty(phased.consumer);
            }
        }
        struct _Remember {
            RigExecPropertyChainBindings::Chain &chain;
            RigExecResolvedInputs &resolved;
            const std::vector<VtValue> &phasedPublished;
            std::vector<std::string> *diagnostics;
            size_t before;
            ~_Remember() {
                const VtValue *now = resolved.Find(chain.targetPath);
                const bool published = now != nullptr;
                // A phased consumer can move while the final value does not
                // (a clamp at the end of the chain), and a chain downstream
                // of it has to hear about that too.
                bool phasedMoved = false;
                chain.lastPhased.resize(chain.phased.size());
                for (size_t k = 0; k < chain.phased.size(); ++k) {
                    const VtValue &current = phasedPublished[k];
                    phasedMoved = phasedMoved ||
                                  current != chain.lastPhased[k];
                    chain.lastPhased[k] = current;
                }
                chain.changedThisRun =
                    !chain.cached || published != chain.published ||
                    (published && *now != chain.lastValue) || phasedMoved;
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
        } remember{chain, _resolvedInputs, phasedPublished, diagnostics,
                   diagnosticsBefore};
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
            // The value after each revision, base first, so a phased
            // consumer can read the chain where it asked to. A revision that
            // passes through still takes its place.
            std::vector<ValueT> history;
            if (!chain.phased.empty()) {
                history.reserve(revisions.size() + 1);
            }
            for (const RigExecPropertyChainBindings::Revision &revision :
                     revisions) {
                if (!chain.phased.empty()) {
                    history.push_back(value);
                }
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
            // Publish immediately, not after every chain has run. Dependency
            // ordering guarantees that any later chain which consumes this
            // property reads the revised value. An interactive override on
            // the target replaces that final value for every reader of it,
            // here as after the chains; the revisions still run from the
            // authored base, whose history the base and checkpoint readers
            // read.
            const auto targetOverride = overridden.find(target);
            const VtValue finalValue = targetOverride != overridden.end()
                                           ? targetOverride->second
                                           : VtValue(value);
            publish(target, finalValue);
            if (!chain.phased.empty()) {
                history.push_back(value);
                for (size_t k = 0; k < chain.phased.size(); ++k) {
                    const auto &phased = chain.phased[k];
                    if (readerOverridden(phased)) {
                        continue;
                    }
                    const VtValue read =
                        phased.final
                            ? finalValue
                            : VtValue(history[std::min(phased.applied,
                                                       revisions.size())]);
                    phasedPublished[k] =
                        RigExecPhasedConsumerValue(read, phased.consumerType);
                    publish(phased.consumer, phasedPublished[k]);
                }
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
    std::vector<SdfPath> &newPropertyChainOrder,
    std::vector<RigExecPhasedConnection> &newPhasedConnections,
    const std::vector<UsdPrim> &solvers, const SdfPathVector &inertMovers,
    _CompileFailure *failure) const
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

    // Read phases on connections. Every connected operator input
    // (_ForEachConnectedInput) whose single-source walk -- the walk
    // RigExecResolvedInputs::GetAttribute takes -- reaches a property-chain
    // target reads that chain at its own rigExecReadPhase: `base`, the
    // target's authored value, when it declares none; `final`, after every
    // revision; or a prim path, as the last revision of the target at or
    // beneath that prim left it. The record publishes on the consumer
    // itself, which the overlay walk meets before any hop, so the reading
    // input's phase decides whatever a hop declares. An undeclared input
    // never fails here: one that math movers revise themselves reads its
    // own chain's result, and one of another value type never read the
    // chain, so both keep reading as they did.
    const TfToken phaseField(RigExecReadPhaseMetadataName);
    const TfToken interpolatorType("RigExecPoseInterpolator");
    const TfToken poseType("RigExecPose");
    const auto targetsOf = [](const UsdPrim &prim, const TfToken &name) {
        SdfPathVector targets;
        if (const UsdRelationship rel = prim.GetRelationship(name)) {
            rel.GetTargets(&targets);
        }
        return targets;
    };
    // Who reads an input, and so answers for a bad declaration on it: the
    // operation the retry sets aside. A mover reads its own inputs and,
    // transitively, the weight objects and blend inputs it names; a solver
    // its own; a pose interpolator its own, its poses' and its numeric
    // driver attributes. The first compiled operation in that order
    // answers. An input that only movers set aside or inert movers read is
    // not compiled, as they are not. An input no operation reads -- on a
    // control, a joint, a channel, a space switch, an expression or a
    // relay -- fails the rig when its declaration is bad. Worked out only
    // when asked: an operation set aside, an inert mover, or a refusal.
    struct _Readers {
        SdfPath compiled;       // the first compiled operation reading it
        bool notCompiled = false;  // an operation set aside or inert reads it
    };
    std::unordered_map<SdfPath, _Readers, SdfPath::Hash> readersOf;
    bool readersKnown = false;
    const auto learnReaders = [&]() {
        readersKnown = true;
        const auto claim = [&](const SdfPath &path, const SdfPath &operation,
                               bool compiled) {
            _Readers &readers = readersOf[path];
            if (!compiled) {
                readers.notCompiled = true;
            } else if (readers.compiled.IsEmpty()) {
                readers.compiled = operation;
            }
        };
        const auto claimMover = [&](const SdfPath &mover, bool compiled) {
            claim(mover, mover, compiled);
            const UsdPrim prim = _stage->GetPrimAtPath(mover);
            if (!prim) {
                return;
            }
            std::unordered_set<SdfPath, SdfPath::Hash> weightsSeen;
            SdfPathVector pending = targetsOf(prim, _weightObjectRel);
            while (!pending.empty()) {
                const SdfPath weightPath = pending.back().GetPrimPath();
                pending.pop_back();
                if (!weightsSeen.insert(weightPath).second) {
                    continue;
                }
                claim(weightPath, mover, compiled);
                if (const UsdPrim weight = _stage->GetPrimAtPath(weightPath)) {
                    for (const TfToken *rel :
                         {&_inputWeightsRel, &_baseWeightRel}) {
                        const SdfPathVector next = targetsOf(weight, *rel);
                        pending.insert(pending.end(), next.begin(),
                                       next.end());
                    }
                }
            }
            for (const SdfPath &input : targetsOf(prim, _blendInputsRel)) {
                claim(input.GetPrimPath(), mover, compiled);
            }
        };
        for (const RigExecMoverRecord &mover : newMovers) {
            claimMover(mover.moverPath, true);
        }
        for (const UsdPrim &solver : solvers) {
            claim(solver.GetPath(), solver.GetPath(), true);
        }
        for (const SdfPath &mover : inertMovers) {
            claimMover(mover, false);
        }
        for (const auto &[operation, reason] : _skippedOperations) {
            claimMover(operation, false);
        }
        for (const SdfPath &interpolator :
             _DiscoverPoseInterpolators(_stage, _rigPath)) {
            const bool compiled = !_IsSkippedOperation(interpolator);
            if (const UsdPrim prim = _stage->GetPrimAtPath(interpolator)) {
                for (const SdfPath &driver :
                     targetsOf(prim, _driverAttributesRel)) {
                    claim(driver, interpolator, compiled);
                }
            }
        }
    };
    // The operation answering for \p input, and whether no compiled
    // operation reads it although one set aside or inert does.
    const auto readerOf = [&](const UsdAttribute &input,
                              bool *notCompiled) -> SdfPath {
        if (!readersKnown) {
            learnReaders();
        }
        *notCompiled = false;
        const UsdPrim prim = input.GetPrim();
        for (const SdfPath &path : {input.GetPath(), prim.GetPath()}) {
            if (const auto it = readersOf.find(path); it != readersOf.end()) {
                if (!it->second.compiled.IsEmpty()) {
                    return it->second.compiled;
                }
                *notCompiled = *notCompiled || it->second.notCompiled;
            }
        }
        if (*notCompiled) {
            return SdfPath();
        }
        if (prim.GetTypeName() == interpolatorType) {
            *notCompiled = _IsSkippedOperation(prim.GetPath());
            return prim.GetPath();
        }
        if (prim.GetTypeName() == poseType) {
            const UsdPrim parent = prim.GetParent();
            if (parent && parent.GetTypeName() == interpolatorType) {
                *notCompiled = _IsSkippedOperation(parent.GetPath());
                return parent.GetPath();
            }
        }
        return SdfPath();
    };
    const bool anyNotCompiled =
        !_skippedOperations.empty() || !inertMovers.empty();

    // A record per reading input, `final` ones decided below.
    struct _Candidate {
        RigExecPhasedConnection connection;
        // A double reader of a float chain: the overlay walk does not widen
        // what it meets (RigExecResolvedInputs::GetAttribute only narrows a
        // double to a float), so only a record converts the value.
        bool widens = false;
    };
    std::vector<_Candidate> candidates;
    bool failed = false;
    const auto consider = [&](const UsdAttribute &input) {
        if (failed) {
            return;
        }
        const SdfPath &path = input.GetPath();
        // An operation set aside or inert is not compiled, and neither is
        // what only such operations read.
        if (anyNotCompiled) {
            bool notCompiled = false;
            readerOf(input, &notCompiled);
            if (notCompiled || _IsSkippedOperation(path.GetPrimPath())) {
                return;
            }
        }
        // An unconnected input reads its own value, so a phase there has
        // nothing to choose. A connection list authored empty is none.
        SdfPathVector hop = _AuthoredConnections(input);
        if (hop.empty()) {
            return;
        }
        const auto refuse = [&](const std::string &message) {
            failed = true;
            bool notCompiled = false;
            const SdfPath owner = readerOf(input, &notCompiled);
            fail(message,
                 owner.IsEmpty() ? SdfPathVector() : SdfPathVector{owner});
        };
        const std::string who =
            path.GetString() + ": rigExecReadPhase";
        const bool declared = input.HasAuthoredMetadata(phaseField);
        RigExecReadPhase phase;  // base
        if (declared) {
            std::string phaseError;
            if (!RigExecResolveReadPhase(input, &phase, &phaseError)) {
                return refuse(phaseError);
            }
            if (phase.kind == RigExecReadPhaseKind::Preceding) {
                return refuse(who + " 'preceding' names no position for a "
                                    "connection; use base, final or a prim "
                                    "path");
            }
        }
        if (newPropertyChains.count(path)) {
            if (declared) {
                refuse(who + " on an input math movers also revise is "
                             "ambiguous");
            }
            return;
        }
        // The first chain target along the single-source walk, and every
        // hop before it.
        SdfPath target;
        SdfPathVector hops{path};
        while (hop.size() == 1 &&
               std::find(hops.begin(), hops.end(), hop[0]) == hops.end()) {
            if (newPropertyChains.count(hop[0])) {
                target = hop[0];
                break;
            }
            const UsdAttribute next = _stage->GetAttributeAtPath(hop[0]);
            if (!next) {
                break;
            }
            hops.push_back(hop[0]);
            hop = _AuthoredConnections(next);
        }
        if (target.IsEmpty()) {
            // No math mover writes what the connection reads: base and
            // final are the same value there, and a checkpoint names a
            // revision that does not exist.
            if (phase.kind == RigExecReadPhaseKind::AtPrim) {
                refuse(who + " names " + phase.prim.GetString() +
                       ", but the connection reaches no property a math "
                       "mover writes");
            }
            return;
        }
        const UsdAttribute targetAttr = _stage->GetAttributeAtPath(target);
        const SdfValueTypeName consumerType = input.GetTypeName();
        const SdfValueTypeName targetType =
            targetAttr ? targetAttr.GetTypeName() : SdfValueTypeName();
        if (!_PhasedTypesAgree(consumerType, targetType)) {
            if (declared) {
                refuse(who + " reads " + target.GetString() + " (" +
                       targetType.GetAsToken().GetString() + ") into a " +
                       consumerType.GetAsToken().GetString() +
                       " input; a phased read needs the same value type, "
                       "or float and double");
            }
            return;
        }
        const std::vector<_PropertyRevision> &revisions =
            newPropertyChains.at(target);
        _Candidate candidate;
        candidate.connection.consumer = path;
        candidate.connection.consumerType = consumerType;
        candidate.connection.target = target;
        candidate.connection.hops = std::move(hops);
        candidate.widens =
            consumerType.GetType() == TfType::Find<double>() &&
            targetType.GetType() == TfType::Find<float>();
        if (phase.kind == RigExecReadPhaseKind::Final) {
            candidate.connection.final = true;
            candidate.connection.applied = revisions.size();
        } else if (phase.kind == RigExecReadPhaseKind::AtPrim) {
            // The last revision of the target at or beneath the prim.
            size_t applied = 0;
            for (size_t i = 0; i < revisions.size(); ++i) {
                if (revisions[i].moverPath.HasPrefix(phase.prim)) {
                    applied = i + 1;
                }
            }
            if (applied == 0) {
                return refuse(who + " names " + phase.prim.GetString() +
                              ", which revises nothing on " +
                              target.GetString());
            }
            candidate.connection.applied = applied;
        }
        candidates.push_back(std::move(candidate));
    };
    _ForEachConnectedInput(_stage->GetPrimAtPath(_rigPath), consider,
                           [](const SdfPath &) {});
    if (failed) {
        return false;
    }

    // A `final` needs a record only where the overlay walk would answer
    // otherwise: where it passes another record's consumer, whose value it
    // would meet first, or where it cannot read the target as the
    // consumer's type (`widens`). Elsewhere the walk reaches the target,
    // whose published value is the final one, and a record would publish
    // the same value again. A hop's walk is a suffix of the reader's, so
    // deciding the shorter walks first has decided every hop before the
    // walk that passes it.
    std::unordered_set<SdfPath, SdfPath::Hash> recorded;
    std::vector<size_t> finals;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].connection.final) {
            finals.push_back(i);
        } else {
            recorded.insert(candidates[i].connection.consumer);
        }
    }
    std::stable_sort(finals.begin(), finals.end(),
                     [&candidates](size_t a, size_t b) {
                         return candidates[a].connection.hops.size() <
                                candidates[b].connection.hops.size();
                     });
    std::vector<char> keep(candidates.size(), 1);
    for (const size_t i : finals) {
        const SdfPathVector &hops = candidates[i].connection.hops;
        keep[i] = candidates[i].widens ||
                  std::any_of(hops.begin() + 1, hops.end(),
                              [&recorded](const SdfPath &hop) {
                                  return recorded.count(hop) != 0;
                              });
        if (keep[i]) {
            recorded.insert(candidates[i].connection.consumer);
        }
    }
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (keep[i]) {
            newPhasedConnections.push_back(
                std::move(candidates[i].connection));
        }
    }

    return true;
}

} // namespace rigExec
