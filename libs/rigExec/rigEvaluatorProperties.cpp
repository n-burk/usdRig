// Property-chain binding and evaluation.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorPropertyBindings.h"
#include "crossDomainInputs.h"
#include "moverGraph.h"
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

    // Descriptor discovery order never schedules a domain in isolation.
    // Cross-domain producer references are linked by the common compiler.
    for(const auto &entry:newPropertyChains) newPropertyChainOrder.push_back(entry.first);

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
            if (phase.kind == RigExecReadPhaseKind::Preceding &&
                !input.HasAuthoredMetadata(TfToken(RigExecInputElementMetadataName)) &&
                input.GetTypeName() != SdfValueTypeNames->Matrix4d) {
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
        bool crossDomain = false;
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
            crossDomain = crossDomain ||
                (input.HasAuthoredMetadata(TfToken(RigExecInputElementMetadataName)) &&
                 next.GetTypeName().GetType() == TfType::Find<VtVec3fArray>()) ||
                (input.GetTypeName() == SdfValueTypeNames->Matrix4d &&
                 next.GetTypeName() == SdfValueTypeNames->Matrix4d &&
                 next.GetName() == TfToken("posed:space"));
            hops.push_back(hop[0]);
            hop = _AuthoredConnections(next);
        }
        if (target.IsEmpty()) {
            if (phase.kind == RigExecReadPhaseKind::Preceding && !crossDomain) {
                return refuse(who + " 'preceding' names no position for a "
                                    "connection; use base, final or a prim path");
            }
            // No math mover writes what the connection reads: base and
            // final are the same value there, and a checkpoint names a
            // revision that does not exist.
            if (phase.kind == RigExecReadPhaseKind::AtPrim && !crossDomain) {
                refuse(who + " names " + phase.prim.GetString() +
                       ", but the connection reaches no property a math "
                       "mover writes");
            }
            return;
        }
        const UsdAttribute targetAttr = _stage->GetAttributeAtPath(target);
        if (phase.kind == RigExecReadPhaseKind::Preceding) {
            return refuse(who + " 'preceding' names no position for a "
                                "connection; use base, final or a prim path");
        }
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
