// Input reads, interactive overrides, and value-cache invalidation.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorPropertyBindings.h"
#include "movers/moverRegistry.h"

#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"

#include <algorithm>
#include <set>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

const TfToken _restPointsAttr("rigExec:restPoints");

// Whether any override in \p overrides could reach a cached blend sample
// shape.
// A shape is a function of exactly `offsets` and `pointIndices` on the
// UsdSkelBlendShape a sparse sample names, so an avar drag -- which is what
// every interactive override is -- cannot move one, and dropping 169
// resolved correctives per mouse sample would re-read 26,276-point arrays
// for nothing. A computation override names a computation this cannot
// inspect, so it counts, which is the same conservative reading the skin
// layout predicate makes.
bool
_OverridesReachBlendShapes(const std::vector<RigExecValueOverride> &overrides)
{
    static const TfToken offsets("offsets"), pointIndices("pointIndices");
    for (const RigExecValueOverride &o : overrides) {
        if (o.attribute.IsEmpty() || o.attribute == offsets ||
            o.attribute == pointIndices) {
            return true;
        }
    }
    return false;
}

// What one notice reaches, in the terms the value caches ask it.
// A notice names a composed change at every stage path that depends on the
// edited spec -- through references, inherits and every other arc -- so a
// cache keyed by the stage path it read is reached exactly where one of those
// paths covers its key: the property itself for a changed-info or a property
// resync, and everything at or under a prim for a prim resync or a prim's own
// changed-info (a clip or a layer offset can move every value beneath it).
// Three kinds of notice reach everything instead:
//   * one at the absolute root, which is how a sublayer, a mute or layer
//     metadata arrives;
//   * a resolved-asset resync, which re-reads asset contents under paths
//     that need not name what moved;
//   * anything inside a prototype, because what the caches read through an
//     instance proxy is keyed by the proxy's path and the notice names the
//     prototype's.
struct _NoticeReach
{
    bool everything = false;
    std::vector<SdfPath> paths;

    explicit _NoticeReach(const UsdNotice::ObjectsChanged &notice)
    {
        everything = !notice.GetResolvedAssetPathsResyncedPaths().empty();
        const auto note = [this](const SdfPath &path) {
            if (path.IsAbsoluteRootPath() ||
                UsdPrim::IsPathInPrototype(path.GetPrimPath())) {
                everything = true;
            }
            paths.push_back(path);
        };
        for (const SdfPath &path : notice.GetResyncedPaths()) {
            note(path);
        }
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            note(path);
        }
    }

    // Whether the notice moved \p path: named it, or named a prim above it.
    bool Reaches(const SdfPath &path) const
    {
        if (everything) {
            return true;
        }
        for (const SdfPath &p : paths) {
            if (path.HasPrefix(p)) {
                return true;
            }
        }
        return false;
    }

    // Whether the notice moved anything of the prim \p prim: the prim, any
    // property or prim under it, or a prim above it.
    bool ReachesPrim(const SdfPath &prim) const
    {
        if (everything) {
            return true;
        }
        for (const SdfPath &p : paths) {
            if (p.HasPrefix(prim) || prim.HasPrefix(p)) {
                return true;
            }
        }
        return false;
    }
};

} // namespace

namespace evaluatorDetail {

// Composed connections for an attribute, skipping the composition entirely
// when the attribute carries no authored connection opinion.
// UsdAttribute::GetConnections builds a Pcp property index and then a target
// index for the attribute on every call, and PcpBuildTargetIndex derives its
// targets from authored ConnectionPaths opinions alone -- so on an attribute
// that has no such opinion anywhere in its composition the whole index build
// can only ever come back empty. Compile and evaluate ask nearly every
// attribute on the rig for its connections while only a small minority are
// connected at all, so the cheap authored-metadata test comes first.
SdfPathVector
_AuthoredConnections(const UsdAttribute &attribute)
{
    SdfPathVector sources;
    if (attribute && attribute.HasAuthoredConnections()) {
        attribute.GetConnections(&sources);
    }
    return sources;
}

// usdview presents a prim's composed children from top to bottom. Movers use
// that namespace as a stack, so the bottom branch executes first, each mover
// parent executes after all of its descendants, and the top branch executes
// last. This is therefore post-order within a branch and REVERSE composed
// child order between sibling branches.
// Keep this as the one traversal primitive for both the structural digest and
// compilation. If those walks ever disagree, an order edit can retain the old
// epoch digest while executing a different chain.
std::vector<UsdPrim>
_GetPoseStackOrder(const UsdPrim &root)
{
    std::vector<UsdPrim> ordered;
    if (root) {
        // Reversing an ordinary composed pre-order yields exactly the stack
        // walk: reversed sibling branches, recursively, with each parent
        // after its descendants. Using UsdPrimRange here also preserves its
        // standard traversal predicate and instance behavior.
        for (const UsdPrim &prim : UsdPrimRange(root)) {
            ordered.push_back(prim);
        }
        std::reverse(ordered.begin(), ordered.end());
    }
    return ordered;
}

/// The same walk over the whole RIG ROOT, which is what numbers the mover
/// stack. A mover is any prim beneath the rig that carries rigExec:moves --
/// what it IS, not where it sits: `Movers` is the conventional scope for
/// them, exactly as `Solvers` is for solvers (spec §4.1, no membership
/// lists), and a mover under `Ops`, under a control, or straight under the
/// rig root is discovered the same way. The identical walk numbers the
/// UNIFIED POSE STACK -- joint-writing aggregate solvers and pose-domain
/// frame constraints in one order (spec §4.2) -- and the mover order is a
/// restriction of it, so a scope's position among its siblings orders the
/// movers inside it against every other scope's.
std::vector<UsdPrim>
_GetMoverExecutionOrder(const UsdPrim &rig)
{
    return _GetPoseStackOrder(rig);
}

namespace {

// Type names the predicates below compare against, interned once at load:
// they are asked per weight object per frame, and token comparison is a
// pointer comparison while literal comparison is not.
const TfToken _kSphereWeightType("RigExecSphereWeight");
const TfToken _kPlaneWeightType("RigExecPlaneWeight");
const TfToken _kCurveWeightType("RigExecCurveWeight");
const TfToken _kJointType("RigExecJoint");
const TfToken _kControlType("RigExecControl");
const TfToken _kStaticWeightType("RigExecStaticWeight");
const TfToken _kDynamicWeightType("RigExecDynamicWeight");
const TfToken _kCombineWeightType("RigExecCombineWeight");

} // namespace

/// True for the schema types that GENERATE a weight field from a placed
/// volume, as opposed to storing or modulating one.
bool
_IsVolumeWeightType(const TfToken &typeName)
{
    return typeName == _kSphereWeightType || typeName == _kPlaneWeightType ||
           typeName == _kCurveWeightType;
}

// Types whose frames participate in namespace-based pose dependencies.
bool
_IsFrameProviderType(const TfToken &type)
{
    return type == _kJointType || type == _kControlType ||
           _IsVolumeWeightType(type);
}

bool
_IsFrameProvider(const UsdPrim &prim)
{
    return prim && _IsFrameProviderType(prim.GetTypeName());
}

/// True for every schema that publishes computeWeightPacket.
///
/// The volumetric types are NOT RigExecWeightObject subclasses -- a typed
/// schema gets exactly one base and they spend it on RigExecXformable, to
/// be placeable (see the RigExecVolumeWeight schema doc) -- so weight-object
/// identity is a type-name question here rather than an IsA one. That is
/// what the rest of this file already does for RigExecDynamicWeight.
bool
_IsWeightObjectType(const TfToken &typeName)
{
    return typeName == _kStaticWeightType ||
           typeName == _kDynamicWeightType ||
           typeName == _kCombineWeightType || _IsVolumeWeightType(typeName);
}

std::vector<TfToken>
_MakeRestInputNames()
{
    return {TfToken("rest:space"), TfToken("rest:tx"), TfToken("rest:ty"),
            TfToken("rest:tz"),    TfToken("rest:rx"), TfToken("rest:ry"),
            TfToken("rest:rz")};
}

bool
_IsRestInputName(const std::vector<TfToken> &names, const TfToken &name)
{
    return std::find(names.begin(), names.end(), name) != names.end();
}

// Whether any rest channel of \p provider can move within an epoch.
// Three ways it can, and the epoch-constant rest path is refused for all
// three: an authored connection (which can reach anything, including an
// animated avar, so it counts without being followed); time samples anywhere
// in the composition -- BOTH tests, because ValueMightBeTimeVarying() is
// false for exactly one sample of a non-composable type while Default and a
// numeric read still disagree about it; and a property chain writing the
// attribute, which recomputes it every generation (the same guard the skin
// layout already applies to its own three attributes).
bool
_ProviderRestMightVary(const UsdStageRefPtr &stage,
                       const std::vector<TfToken> &restInputNames,
                       const SdfPath &provider,
                       const std::set<SdfPath> &chainTargets)
{
    const UsdPrim prim = stage ? stage->GetPrimAtPath(provider) : UsdPrim();
    if (!prim) {
        return true;
    }
    for (const TfToken &name : restInputNames) {
        if (chainTargets.count(provider.AppendProperty(name))) {
            return true;
        }
        const UsdAttribute attribute = prim.GetAttribute(name);
        if (!attribute) {
            continue;
        }
        if (attribute.HasAuthoredConnections() ||
            attribute.ValueMightBeTimeVarying() ||
            attribute.GetNumTimeSamples() > 0) {
            return true;
        }
    }
    return false;
}

// Appends the interactive overrides to \p overrides, replacing any entry
// already standing on the same key, and mirrors the attribute ones into
// _resolvedInputs.
// Replacing rather than appending is not a tidiness preference: exec is given
// a vector of key/value pairs and which of two entries on one key wins is not
// a promise anything here should rely on. Removing the loser makes the answer
// a property of this function.
void
_ApplyInteractiveOverrides(
    const std::vector<RigExecValueOverride> &interactive,
    std::vector<RigExecValueOverride> *overrides,
    RigExecResolvedInputs *resolved)
{
    for (const RigExecValueOverride &o : interactive) {
        if (overrides) {
            overrides->erase(
                std::remove_if(
                    overrides->begin(), overrides->end(),
                    [&o](const RigExecValueOverride &existing) {
                        return existing.prim == o.prim &&
                               existing.attribute == o.attribute &&
                               existing.computation == o.computation;
                    }),
                overrides->end());
            overrides->push_back(o);
        }
        // Only an ATTRIBUTE override has a property path to resolve; a
        // computation override names no property and the static readers never
        // look for one.
        if (resolved && !o.attribute.IsEmpty()) {
            resolved->SetProperty(o.prim.AppendProperty(o.attribute), o.value);
        }
    }
}

} // namespace evaluatorDetail

void
RigExecRigEvaluator::_ClearValueCachesWholesale(bool avarValuesOnly)
{
    // The static cache holds authored values, so it is dropped even for an
    // avar-only notice: it holds the edited avar's OLD authored value, and
    // it is the one cache below that can.
    _staticInputs.Clear();
    // Property chains can read numeric avars through scalar connections.
    // Their cached result must be dropped even for an avar-only notice.
    _propertyChainBindings.reset();
    // Skin layouts, blend samples and base geometry cannot hold avar values.
    if (avarValuesOnly) {
        return;
    }
    // Dropped as answers and kept as candidates (see the caches' Clear), so
    // a re-read that finds the same arrays keeps the same pointer.
    _skinTopologies.Clear();
    _blendSampleShapes.Clear();
    _skinLayoutInputsValid = false;
    for (auto &[target, live] : _liveGraphs) {
        if (live) live->basePointsPushed = false;
    }
}

void
RigExecRigEvaluator::_ClearValueCaches(const UsdNotice::ObjectsChanged &notice,
                                       bool avarValuesOnly)
{
    if (_wholesaleValueClears) {
        _ClearValueCachesWholesale(avarValuesOnly);
        return;
    }
    const _NoticeReach reach(notice);
    if (reach.everything) {
        _ClearValueCachesWholesale(avarValuesOnly);
        return;
    }
    // The static inputs, keyed by the attribute read: a property the notice
    // names is erased by key, and a prim it names takes everything beneath
    // it in one pass. Both re-stamp the cache's owner as Clear() does.
    {
        std::vector<SdfPath> prefixes;
        for (const SdfPath &path : reach.paths) {
            if (path.IsPropertyPath()) {
                _staticInputs.Erase(path);
            } else {
                prefixes.push_back(path);
            }
        }
        _staticInputs.ErasePrefixes(prefixes);
    }
    // A property chain answers from what it bound and from the value it
    // cached last run, so it is marked stale -- and rebound, and re-run, by
    // the next frame -- when the notice reaches anything that answer came
    // from, in ANY field: an input or an attribute along an input's
    // connection walk (a default edit on an unconnected input moves the
    // constant it captured), a connection source the walk could not find,
    // the target, and the mover prim with everything on it, the
    // weight-object relationship included. Never scoped to the connection
    // fields alone: that is how a default edit on inputs:value would replay
    // a stale value.
    if (_propertyChainBindings) {
        for (RigExecPropertyChainBindings::Chain &chain :
                 _propertyChainBindings->chains) {
            if (chain.stale) {
                continue;
            }
            bool hit = reach.Reaches(chain.targetPath);
            for (size_t k = 0; !hit && k < chain.watch.size(); ++k) {
                hit = reach.Reaches(chain.watch[k]);
            }
            for (size_t k = 0; !hit && k < chain.missingSources.size();
                 ++k) {
                hit = reach.Reaches(chain.missingSources[k]);
            }
            for (size_t k = 0; !hit && k < chain.moverPaths.size(); ++k) {
                hit = reach.ReachesPrim(chain.moverPaths[k]);
            }
            if (hit) {
                chain.stale = true;
                _propertyChainBindings->anyStale = true;
            }
        }
    }
    // Avar-only edits may invalidate a property chain above, but cannot
    // change skin layouts, blend offsets or base geometry below.
    if (avarValuesOnly) {
        return;
    }
    // The skin layouts, and which properties can reach one. A layout is read
    // from its mover's own layout attributes, and _skinLayoutInputs holds
    // those plus every attribute along the connection walk a read of one
    // follows -- so a notice that reaches a member (its value, its
    // connections, a resync of it or above it) drops both, and any other
    // notice drops neither. Never keyed on "the notice names a mesh": the
    // layout attributes live on the mover prim and are reached through
    // connections. The set is resolved here when it is not standing, off the
    // stage as the notice left it: a layout attribute itself is a member
    // whatever its connections say, and a connection edit is a notice on the
    // member that carries it.
    if (_skinLayoutInputsValid || _skinTopologies.GetSize() > 0) {
        if (!_skinLayoutInputsValid) {
            _ResolveSkinLayoutInputs();
        }
        bool hit = std::any_of(
            _skinLayoutInputs.begin(), _skinLayoutInputs.end(),
            [&reach](const SdfPath &member) { return reach.Reaches(member); });
        // And the mover prims themselves: a set resolved after the notice
        // holds nothing for a mover the notice removed, whose layout is
        // still cached under its path.
        for (size_t k = 0; !hit && k < _movers.size(); ++k) {
            const RigExecMoverHandler *handler =
                RigExecFindMoverHandler(_movers[k].schemaType);
            hit = handler && !handler->layoutAttributes.empty() &&
                  reach.Reaches(_movers[k].moverPath);
        }
        if (hit) {
            _skinTopologies.Clear();
            _skinLayoutInputsValid = false;
        }
    }
    // A blend sample shape is read from the UsdSkelBlendShape its sample
    // names and range-checked against the length of the points it is
    // applied to, so it is dropped where the notice reaches the sample prim,
    // the blend shape prim (by prefix) or those points -- one sample at a
    // time, the rest keeping their pointers.
    if (_blendSampleShapes.GetSize() > 0) {
        if (!_blendSampleSourcesValid) {
            _blendSampleSources.clear();
            for (const auto &[target, revisions] : _graphChains) {
                for (const _GraphRevision &revision : revisions) {
                    for (const auto &[input, samples] :
                             revision.binding.blendSamples) {
                        for (const RigExecBlendSampleBinding &sample :
                                 samples) {
                            if (!sample.blendShape.IsEmpty()) {
                                _blendSampleSources.push_back(
                                    {sample.sample,
                                     sample.blendShape.GetPrimPath(),
                                     target});
                            }
                        }
                    }
                }
            }
            _blendSampleSourcesValid = true;
        }
        for (const _BlendSampleSource &source : _blendSampleSources) {
            if (reach.ReachesPrim(source.sample) ||
                reach.ReachesPrim(source.blendShape) ||
                reach.Reaches(source.points)) {
                _blendSampleShapes.Erase(source.sample);
            }
        }
    }
    // The base points a live graph pushed and will not re-read while they
    // cannot vary: re-read once the notice reaches that target's points.
    for (auto &[target, live] : _liveGraphs) {
        if (live && live->basePointsPushed && reach.Reaches(target)) {
            live->basePointsPushed = false;
        }
    }
}

void
RigExecRigEvaluator::SetInteractiveOverrides(
    std::vector<RigExecValueOverride> overrides)
{
    if (_scopedClearShadow) {
        _scopedClearShadow->SetInteractiveOverrides(overrides);
    }
    // Asked of BOTH sets before either is dropped: an override being lifted
    // off a layout attribute moves the value the layout was read with just
    // as much as one being placed on it. The predicates read keys alone, so
    // the two sets also cover every entry of _valueInputs this moves (an
    // upstream value under a lifted override re-emerges at a key the old
    // set names).
    const bool touchesLayout =
        _OverridesReachSkinLayout(_interactiveOverrides) ||
        _OverridesReachSkinLayout(overrides);
    const bool touchesShapes =
        _OverridesReachBlendShapes(_interactiveOverrides) ||
        _OverridesReachBlendShapes(overrides);
    _interactiveOverrides = std::move(overrides);
    _RebuildValueInputs();
    // The static-input cache is NOT dropped here. It used to be, as defence
    // in depth, on the reasoning that this "costs one map clear per drag
    // start" -- but a manipulator calls this on EVERY MOUSE SAMPLE, not once
    // per drag, so every drag frame began with a cold cache and re-read
    // every static input from the stage. Measured on the biped, a brow drag
    // spent 0.94 ms re-reading 161 blend-sample activations that are
    // authored constants, the largest single item in its frame.
    // It was never a correctness requirement, and the three ways it could
    // matter are each closed by construction:
    //   * an overridden attribute is written into the resolved inputs before
    //     anything reads, and GetAttribute consults the resolved map FIRST,
    //     so an override never reaches the cache -- and is never put in it;
    //   * an override on a connection SOURCE cannot leave a stale reader,
    //     because the cache refuses any attribute with an authored
    //     connection and that reader goes the long way every time;
    //   * lifting an override is safe, because the cache only ever holds the
    //     authored value it read from the stage.
    // Authored edits still clear it: every notice does.
    // _propertyChainBindings is NOT dropped here, and the asymmetry with the
    // cache above is deliberate. It folds constants for the same class of
    // attribute, but _PinnedRead consults the resolved inputs FIRST -- and
    // _ApplyInteractiveOverridesToResolved writes every override into those
    // before a chain runs, on both paths -- so an overridden attribute can
    // never reach a folded constant to begin with. Dropping the bindings per
    // drag would rebind every chain input of the rig on both halves of every
    // drag, which is the whole of what binding once per compile bought. The
    // two invalidations it does have are the ones that make a binding WRONG
    // rather than outranked: a stage edit (_OnObjectsChanged) and the
    // recompile that replaces the chains the bindings describe.
    // An override is a value the static reads must prefer over the stage,
    // and the skin layout is read through exactly that route -- so a layout
    // resolved before the override set changed was resolved against a
    // different answer. But only for an override that can actually reach
    // one: dropping every layout costs ~400us of a biped drag frame (a
    // third of it) re-reading and re-comparing 105k elements that no
    // manipulator touched, and an animator's drag names a control avar.
    // The predicate follows the same connection walk the layout is read
    // through and answers yes wherever it is unsure -- see
    // _OverridesReachSkinLayout. The dynamic path shares this cache, so
    // both paths get the same answer either way: the cache hands back the
    // pointer it held for arrays that compare equal, so what is at stake is
    // the re-read and not the deformation.
    if (touchesLayout) {
        _skinTopologies.Clear();
    }
    // The same rule for the blend sample shapes, which are the other cache a
    // drag must not pay to rebuild: see _OverridesReachBlendShapes.
    if (touchesShapes) {
        _blendSampleShapes.Clear();
    }
    // Nothing to invalidate: the program is asked to PLACE these at the top
    // of every generation (Evaluate), and it runs only for a set it can place
    // exactly. An override it cannot place -- one standing on a value folded
    // into bake state, or a computation only exec can answer -- makes that
    // generation dynamic instead, which is the same answer more slowly.
}

void
RigExecRigEvaluator::ClearInteractiveOverrides()
{
    if (_scopedClearShadow) {
        _scopedClearShadow->ClearInteractiveOverrides();
    }
    const bool touchesLayout =
        _OverridesReachSkinLayout(_interactiveOverrides);
    const bool touchesShapes =
        _OverridesReachBlendShapes(_interactiveOverrides);
    _interactiveOverrides.clear();
    _RebuildValueInputs();
    // Both halves of a drag treat the caches the same way. The static-input
    // cache is kept on the way out for the same reasons it is kept on the
    // way in (see SetInteractiveOverrides): it never holds an override, so
    // there is nothing of the drag's in it to forget.
    if (touchesLayout) {
        _skinTopologies.Clear();
    }
    // The same rule for the blend sample shapes, which are the other cache a
    // drag must not pay to rebuild: see _OverridesReachBlendShapes.
    if (touchesShapes) {
        _blendSampleShapes.Clear();
    }
}

void
RigExecRigEvaluator::_ApplyInteractiveOverridesToResolved(
    RigExecResolvedInputs *resolved) const
{
    _ApplyInteractiveOverrides(_interactiveOverrides, /* overrides = */
                               nullptr, resolved);
}

void
RigExecRigEvaluator::_ApplyValueInputsToResolved(
    RigExecResolvedInputs *resolved) const
{
    _ApplyInteractiveOverrides(_valueInputs, /* overrides = */ nullptr,
                               resolved);
}

void
RigExecRigEvaluator::_RebuildValueInputs()
{
    // With no upstream value the list IS the interactive one, entry for
    // entry, so every reader sees exactly what it saw before.
    if (_upstreamAdmitted.empty()) {
        _valueInputs = _interactiveOverrides;
        return;
    }
    _valueInputs = _upstreamAdmitted;
    _ApplyInteractiveOverrides(_interactiveOverrides, &_valueInputs,
                               /* resolved = */ nullptr);
}

void
RigExecRigEvaluator::SetUpstreamInputs(
    std::vector<RigExecValueOverride> inputs)
{
    if (_scopedClearShadow) {
        _scopedClearShadow->SetUpstreamInputs(inputs);
    }
    _upstreamRequested = std::move(inputs);
    _AdmitUpstreamInputs();
}

std::vector<SdfPath>
RigExecRigEvaluator::GetUpstreamInputPaths() const
{
    std::vector<SdfPath> paths;
    paths.reserve(_upstreamAdmitted.size());
    for (const RigExecValueOverride &o : _upstreamAdmitted) {
        paths.push_back(o.prim.AppendProperty(o.attribute));
    }
    return paths;
}

void
RigExecRigEvaluator::_AdmitUpstreamInputs()
{
    std::vector<RigExecValueOverride> admitted;
    std::vector<std::string> dropLines;
    if (_upstreamRequested.empty()) {
        _SetUpstreamAdmitted(std::move(admitted), std::move(dropLines));
        return;
    }
    // Condition 3 against the program standing now. With none (an epoch
    // that cannot bake, or a mode that runs no program) conditions 1 and 2
    // decide: the walk reads every attribute through the same overlay, and
    // such an epoch has no file to agree with.
    const std::map<SdfPath, TfType> *listed =
        _bakedProgram ? &_bakedProgram->GetUpstreamAdmissible() : nullptr;
    // One entry per attribute, the last given winning, sorted by path.
    std::map<SdfPath, const RigExecValueOverride *> byPath;
    for (const RigExecValueOverride &o : _upstreamRequested) {
        const SdfPath path = o.attribute.IsEmpty() || o.prim.IsEmpty()
                                 ? SdfPath()
                                 : o.prim.AppendProperty(o.attribute);
        const auto drop = [&](const std::string &reason) {
            dropLines.push_back(
                "upstream input " +
                (path.IsEmpty() ? o.prim.GetString() : path.GetString()) +
                ": " + reason + "; ignored");
        };
        if (path.IsEmpty() || !o.computation.IsEmpty()) {
            drop("names no attribute");
            continue;
        }
        const std::string reason =
            RigExecUpstreamDropReason(_stage, listed, path, o.value);
        if (!reason.empty()) {
            drop(reason);
            continue;
        }
        byPath[path] = &o;
    }
    admitted.reserve(byPath.size());
    for (const auto &[path, o] : byPath) {
        admitted.push_back(*o);
    }
    _SetUpstreamAdmitted(std::move(admitted), std::move(dropLines));
}

void
RigExecRigEvaluator::_SetUpstreamAdmitted(
    std::vector<RigExecValueOverride> admitted,
    std::vector<std::string> dropLines)
{
    _upstreamDropLines = std::move(dropLines);
    if (admitted == _upstreamAdmitted) {
        return;
    }
    // The entries placed, moved or lifted. The skin layouts and blend
    // shapes are read through the overlay these values ride, so the reach
    // test an interactive change takes decides what they drop.
    std::vector<RigExecValueOverride> moved;
    for (const RigExecValueOverride &o : admitted) {
        if (std::find(_upstreamAdmitted.begin(), _upstreamAdmitted.end(),
                      o) == _upstreamAdmitted.end()) {
            moved.push_back(o);
        }
    }
    for (const RigExecValueOverride &o : _upstreamAdmitted) {
        if (std::find(admitted.begin(), admitted.end(), o) ==
            admitted.end()) {
            moved.push_back(o);
        }
    }
    _upstreamAdmitted = std::move(admitted);
    _RebuildValueInputs();
    if (_OverridesReachSkinLayout(moved)) {
        _skinTopologies.Clear();
    }
    if (_OverridesReachBlendShapes(moved)) {
        _blendSampleShapes.Clear();
    }
}

} // namespace rigExec
