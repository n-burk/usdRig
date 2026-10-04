// USD notice classification and epoch invalidation.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorPropertyBindings.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorConstraints.h"
#include "bakedProgramImpl.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <set>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// RIGEXEC_VERIFY_DIGEST_GATE=1: every notice the digest gate calls neutral,
// on an epoch nothing has dirtied since its last commit or settle, has the
// digest recomputed there and then, and moving it is fatal.
bool
_DigestGateVerifyRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_VERIFY_DIGEST_GATE", false);
    return requested;
}

// Whether \p notice is provably nothing but new VALUES on the numeric avar
// channels (see _OnObjectsChanged): the gate for the in-place patch path.
// Shared by the notice handler and ClassifyNoticeDisposition, so the two
// can never disagree about which branch a notice takes.
bool
_NoticeIsAvarValuesOnly(const UsdNotice::ObjectsChanged &notice)
{
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    static const TfToken kDefault("default");
    static const TfToken kTimeSamples("timeSamples");
    static const TfToken kSpline("spline");
    static const TfToken kTypeName("typeName");
    // The numeric channels only. avars:defaultSpace,
    // avars:rotationOrder and avars:rotationSign choose how a frame is
    // composed, which is structure, so they are not in this list -- the
    // baked program captures the sign once per slot and only a recompile
    // can move it.
    static const TfToken kChannels[] = {
        TfToken("avars:tx"), TfToken("avars:ty"), TfToken("avars:tz"),
        TfToken("avars:sx"), TfToken("avars:sy"), TfToken("avars:sz"),
        TfToken("avars:rx"), TfToken("avars:ry"), TfToken("avars:rz"),
        TfToken("avars:rspin"), TfToken("avars:unitScaleFactor")};
    const auto isChannel = [](const TfToken &name) {
        for (const TfToken &avar : kChannels) {
            if (name == avar) {
                return true;
            }
        }
        return false;
    };
    bool sawAvar = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath() || !isChannel(path.GetNameToken())) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kTypeName && field != kDefault &&
                field != kTimeSamples && field != kSpline) {
                return false;
            }
        }
        sawAvar = true;
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path.IsPrimPath()) {
            continue;  // ancestor info around the edit
        }
        if (!isChannel(path.GetNameToken())) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kDefault && field != kTimeSamples &&
                field != kSpline) {
                return false;
            }
        }
        sawAvar = true;
    }
    return sawAvar;
}

// Whether \p notice is nothing but new values (or a value's own spec
// appearing or going away) on avar channels: the numeric transform channels
// _NoticeIsAvarValuesOnly accepts, and the animator channels the bake reads
// live (RigExecIsLiveAvarName). The epoch digest reads none of their values
// -- a switch's index, an IK/FK blend, a dial -- so such a notice cannot
// move it, and recomputing it whole cost ~150 ms per click on the biped.
bool
_NoticeIsAvarChannelValuesOnly(const UsdNotice::ObjectsChanged &notice)
{
    static const std::string kChannels[] = {
        "avars:tx", "avars:ty", "avars:tz", "avars:sx", "avars:sy",
        "avars:sz", "avars:rx", "avars:ry", "avars:rz", "avars:rspin",
        "avars:unitScaleFactor"};
    const auto channel = [](const std::string &name) {
        if (RigExecIsLiveAvarName(name)) {
            return true;
        }
        for (const std::string &avar : kChannels) {
            if (name == avar) {
                return true;
            }
        }
        return false;
    };
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    static const TfToken kDefault("default");
    static const TfToken kTimeSamples("timeSamples");
    static const TfToken kSpline("spline");
    static const TfToken kTypeName("typeName");
    static const TfToken kCustom("custom");
    bool sawAvar = false;
    // A new spec restates the type and, for a custom attribute, that it is
    // custom; neither changes what is read.
    const auto valueFields = [&](const SdfPath &path, bool resync) {
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kDefault && field != kTimeSamples &&
                field != kSpline &&
                !(resync && (field == kTypeName || field == kCustom))) {
                return false;
            }
        }
        return true;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath() || !channel(path.GetName()) ||
            !valueFields(path, true)) {
            return false;
        }
        sawAvar = true;
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path.IsPrimPath()) {
            continue;  // ancestor info around the edit
        }
        if (!channel(path.GetName()) || !valueFields(path, false)) {
            return false;
        }
        sawAvar = true;
    }
    return sawAvar;
}

// The types whose prims the digest writes the path of for the type alone,
// wherever under the rig they sit: the discovered joints, controls, volume
// weights and pose interpolators, and every aggregate solver.
bool
_IsDigestDiscoveredType(const TfToken &type)
{
    static const TfToken kJoint("RigExecJoint");
    static const TfToken kControl("RigExecControl");
    static const TfToken kInterpolator("RigExecPoseInterpolator");
    return type == kJoint || type == kControl || type == kInterpolator ||
           _IsVolumeWeightType(type) || _IsAggregateSolverType(type);
}

} // namespace

namespace evaluatorDetail {

_CertainStructuralCounts *
_CertainStructuralVerifyCounts()
{
    static _CertainStructuralCounts *const counts =
        []() -> _CertainStructuralCounts * {
        if (!TfGetenvBool("RIGEXEC_VERIFY_CERTAIN_STRUCTURAL", false)) {
            return nullptr;
        }
        // Never freed: the exit hook reads it after statics may be gone.
        static _CertainStructuralCounts *const created =
            new _CertainStructuralCounts();
        std::atexit([]() {
            std::fprintf(stderr,
                         "RIGEXEC_VERIFY_CERTAIN_STRUCTURAL: %zu certainly "
                         "structural edit(s), %zu false positive(s); %zu "
                         "structural edit(s) found by the digest\n",
                         created->tagged.load(),
                         created->falsePositives.load(),
                         created->digestFound.load());
            std::fflush(stderr);
        });
        return created;
    }();
    return counts;
}

} // namespace evaluatorDetail

RigExecNoticeDisposition
RigExecRigEvaluator::ClassifyNoticeDisposition(
    const UsdNotice::ObjectsChanged &notice,
    std::vector<SdfPath> *patchedPaths) const
{
    if (patchedPaths) {
        patchedPaths->clear();
    }
    if (!_bakedProgram) {
        return RigExecNoticeDisposition::None;
    }
    std::vector<SdfPath> paths;
    const bool transformOnly = _NoticeIsAvarValuesOnly(notice);
    if ((transformOnly || _NoticeIsAvarChannelValuesOnly(notice)) &&
        _bakedProgram->DryRunAvarValueEdits(notice, &paths)) {
        // Transform channels patched in place; any live channel in the same
        // edit (an IK/FK blend keyed with what it switches) routes as a
        // value edit, and its paths join the patched ones so every cached
        // frame that read either is retired.
        std::vector<SdfPath> routed;
        if (transformOnly ||
            _bakedProgram->DryRunValueEdits(notice, &routed,
                                            /* skipPatchableAvars = */ true)) {
            paths.insert(paths.end(), routed.begin(), routed.end());
            if (patchedPaths) {
                *patchedPaths = std::move(paths);
            }
            return RigExecNoticeDisposition::Patched;
        }
    }
    if (_bakedProgram->IsInvalidatedBy(notice)) {
        return RigExecNoticeDisposition::Stale;
    }
    if (_bakedProgram->DryRunValueEdits(notice, &paths)) {
        if (patchedPaths) {
            *patchedPaths = std::move(paths);
        }
        return RigExecNoticeDisposition::Edited;
    }
    return RigExecNoticeDisposition::StampBumped;
}

RigExecRigEvaluator::_DigestGate
RigExecRigEvaluator::_MakeDigestGate(
    _DigestFootprint (&parts)[_DigestSegmentCount])
{
    _DigestGate gate;
    gate.valid = true;
    // Every recorded prim and every ancestor of one, walked up only until a
    // path already present: that path's own ancestors went in with it.
    const auto cover = [&gate](const SdfPath &prim) {
        for (SdfPath path = prim; !path.IsEmpty();
             path = path.GetParentPath()) {
            if (!gate.covered.insert(path).second ||
                path.IsAbsoluteRootPath()) {
                break;
            }
        }
    };
    for (size_t i = 0; i < _DigestSegmentCount; ++i) {
        const _DigestFootprint &part = parts[i];
        for (const SdfPath &prim : part.read) {
            gate.read.insert(prim);
            cover(prim);
        }
        for (const SdfPath &prim : part.ancestors) {
            cover(prim);
        }
        // Kept per segment rather than merged: each is asked by lookup, and
        // three lookups cost less than building one table out of three.
        gate.certain[i] = std::move(parts[i].certain);
    }
    return gate;
}

bool
RigExecRigEvaluator::_NoticeIsDigestSuspect(
    const UsdNotice::ObjectsChanged &notice) const
{
    if (!_digestGate.valid ||
        !notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return true;
    }
    // Judged per prim: a property path answers for its prim, which is the
    // grain the footprint was recorded at.
    const auto suspect = [this](const SdfPath &path, bool resync) {
        const SdfPath prim = path.GetPrimPath();
        // Anything in the rig: the digest reads the rig whole.
        if (prim.HasPrefix(_rigPath)) {
            return true;
        }
        // A resync at or above the rig recomposes all of it.
        if (resync && _rigPath.HasPrefix(prim)) {
            return true;
        }
        // Changed info on a prim outside the rig, rather than on one of its
        // properties. The digest reads no prim metadata there, and USD
        // reports the fields that could move it -- active, kind, specifier,
        // typeName, apiSchemas -- as resyncs. It is also what every edit
        // under a prim says about the prim itself: defining a child raises
        // it on the parent, which is an ancestor every chain walks through.
        if (!resync && path.IsPrimPath()) {
            return false;
        }
        // The digest follows instance proxies, and an edit behind one is
        // reported on the prototype, not on the proxy path it read.
        if (UsdPrim::IsPathInPrototype(prim)) {
            return true;
        }
        // On or above a prim the digest read or walked through.
        if (_digestGate.covered.count(prim)) {
            return true;
        }
        // Under a prim it read. Not under an ancestor it only walked
        // through: nothing below an ancestor is read by way of it.
        for (SdfPath above = prim.GetParentPath();
             !above.IsEmpty() && !above.IsAbsoluteRootPath();
             above = above.GetParentPath()) {
            if (_digestGate.read.count(above)) {
                return true;
            }
        }
        return false;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (suspect(path, true)) {
            return true;
        }
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (suspect(path, false)) {
            return true;
        }
    }
    return false;
}

void
RigExecRigEvaluator::_VerifyNeutralNotice(
    const UsdNotice::ObjectsChanged &notice) const
{
    // Nothing since the last commit or settle was suspect, so the committed
    // digest is the digest of the stage this notice found; recomputed now,
    // on the stage it left behind, it must not have moved.
    const size_t digest = _SettleStructureDigest();
    if (digest == _structureDigest) {
        return;
    }
    std::string paths;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        paths += " resync:" + path.GetString();
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        paths += " info:" + path.GetString();
    }
    const std::string message = TfStringPrintf(
        "RIGEXEC_VERIFY_DIGEST_GATE: a notice the digest gate called neutral "
        "moved the structure digest of <%s>; it named%s",
        _rigPath.GetText(), paths.c_str());
    // Written out first as well: a fatal error's own text was measured not
    // to reach a Windows test's stderr before the abort, and a violation
    // nobody can read names no path to fix.
    std::fputs((message + "\n").c_str(), stderr);
    std::fflush(stderr);
    TF_FATAL_ERROR("%s", message.c_str());
}

void
RigExecRigEvaluator::_NoteCertainStructuralCandidates(
    const UsdNotice::ObjectsChanged &notice)
{
    // Judged against the committed footprint, so there is nothing to note
    // without one: the settle computes the digest either way.
    if (!_digestGate.valid || _certainCandidatesOverflowed) {
        return;
    }
    // A bound on what one settle judges. An edit past it -- a layer swap, a
    // reference retargeted -- is one whose digest has to be computed to be
    // sure of anything, and it gets exactly that.
    static constexpr size_t kMaxCandidates = 256;
    const auto note = [this](const SdfPath &path) {
        if (_certainCandidates.size() == kMaxCandidates) {
            _certainCandidatesOverflowed = true;
            _certainCandidates.clear();
            return false;
        }
        _certainCandidates.push_back(path);
        return true;
    };
    // Whether the committed footprint recorded anything the property could
    // change: its targets, a solver's jointElements, or an attribute of a
    // pose-input closure prim.
    const auto recorded = [this](const SdfPath &property) {
        const _CertainFootprint::PropertyKey key{property.GetPrimPath(),
                                                 property.GetNameToken()};
        for (const _CertainFootprint &certain : _digestGate.certain) {
            if (certain.targets.count(key) ||
                certain.jointElements.count(key.prim) ||
                certain.closurePrims.count(key.prim)) {
                return true;
            }
        }
        return false;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        // A prim resync strictly inside the rig: at or above the rig root,
        // everything recomposes, and that is the digest's to judge.
        const bool candidate = path.IsPrimPath()
            ? path != _rigPath && path.HasPrefix(_rigPath)
            : path.IsPropertyPath() && recorded(path);
        if (candidate && !note(path)) {
            return;
        }
    }
    static const TfToken kTargetPaths("targetPaths");
    static const TfToken kConnectionPaths("connectionPaths");
    static const TfToken kJointElements("rigExec:jointElements");
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (!path.IsPropertyPath()) {
            continue;
        }
        bool listEdit = path.GetNameToken() == kJointElements;
        for (const TfToken &field : notice.GetChangedFields(path)) {
            listEdit = listEdit || field == kTargetPaths ||
                       field == kConnectionPaths;
        }
        if (listEdit && recorded(path) && !note(path)) {
            return;
        }
    }
}

bool
RigExecRigEvaluator::_EditIsCertainlyStructural() const
{
    if (!_digestGate.valid || _certainCandidatesOverflowed ||
        _certainCandidates.empty() || !_stage) {
        return false;
    }
    // The digest walks the rig with the default predicate, so a rig that no
    // longer passes it is a rig whose every list has changed shape, which is
    // the digest's to say.
    const UsdPrim rig = _stage->GetPrimAtPath(_rigPath);
    if (!rig || rig.IsInstanceProxy() || !UsdPrimDefaultPredicate(rig)) {
        return false;
    }
    static const TfToken kJoint("RigExecJoint");
    static const TfToken kJointElements("rigExec:jointElements");
    // A prim's state as the digest's discovery walks see it now: its type,
    // when it is in the walk and of a discovered type; empty when it is not
    // written for its type at all; nothing when that cannot be told. A joint
    // outside the walk is the case that cannot: a solver's rigExec:joints
    // still unions it into the joint set, so leaving the walk need not move
    // the digest.
    const auto stateNow = [this](const SdfPath &path)
        -> std::optional<TfToken> {
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        if (!prim) {
            return TfToken();
        }
        const TfToken type = prim.GetTypeName();
        if (!_IsDigestDiscoveredType(type)) {
            return TfToken();
        }
        bool walked = !prim.IsInstanceProxy();
        for (UsdPrim at = prim; walked && at.GetPath() != _rigPath;
             at = at.GetParent()) {
            walked = UsdPrimDefaultPredicate(at);
        }
        if (walked) {
            return type;
        }
        if (type == kJoint) {
            return std::nullopt;
        }
        return TfToken();
    };
    const auto changed = [&stateNow](const SdfPath &path,
                                     const TfToken &recorded) {
        const std::optional<TfToken> now = stateNow(path);
        return now && *now != recorded;
    };
    for (const SdfPath &path : _certainCandidates) {
        if (path.IsPrimPath()) {
            // The resynced prim and every recorded prim below it: a prim
            // resync recomposes the whole subtree, so it can remove, retype
            // or deactivate anything the last digest listed there. A prim
            // it creates below itself is not looked for.
            bool recordedHere = false;
            for (const _CertainFootprint &certain : _digestGate.certain) {
                for (auto it = certain.prims.lower_bound(path);
                     it != certain.prims.end() && it->first.HasPrefix(path);
                     ++it) {
                    recordedHere = recordedHere || it->first == path;
                    if (changed(it->first, it->second)) {
                        return true;
                    }
                }
            }
            if (!recordedHere && changed(path, TfToken())) {
                return true;
            }
            continue;
        }
        // A property: its owner must still stand for the digest to reach it
        // where it did.
        const UsdPrim owner = _stage->GetPrimAtPath(path.GetPrimPath());
        if (!owner) {
            continue;
        }
        const _CertainFootprint::PropertyKey key{path.GetPrimPath(),
                                                 path.GetNameToken()};
        for (const _CertainFootprint &certain : _digestGate.certain) {
            const auto targets = certain.targets.find(key);
            if (targets != certain.targets.end()) {
                SdfPathVector now;
                if (const UsdRelationship rel =
                        owner.GetRelationship(key.name)) {
                    rel.GetTargets(&now);
                }
                SdfPathVector before = targets->second.targets;
                if (targets->second.sorted) {
                    std::sort(now.begin(), now.end());
                    std::sort(before.begin(), before.end());
                }
                if (now != before) {
                    return true;
                }
            }
            const auto elements = certain.jointElements.find(key.prim);
            if (elements != certain.jointElements.end() &&
                key.name == kJointElements) {
                VtIntArray values;
                size_t samples = 0;
                if (const UsdAttribute attr = owner.GetAttribute(key.name)) {
                    attr.Get(&values);
                    samples = attr.GetNumTimeSamples();
                }
                if (values != elements->second.values ||
                    samples != elements->second.samples) {
                    return true;
                }
            }
            if (certain.closurePrims.count(key.prim)) {
                // Every attribute of a closure prim is written with its
                // sources, so its sources now are compared with the ones
                // recorded -- none, where it was not recorded as connected.
                // One that is gone was certainly written only if it was
                // recorded: an unconnected one may never have existed.
                const auto connected = certain.closureConnections.find(path);
                const bool wasConnected =
                    connected != certain.closureConnections.end();
                const UsdAttribute attr = owner.GetAttribute(key.name);
                if (!attr) {
                    if (wasConnected) {
                        return true;
                    }
                } else if (_AuthoredConnections(attr) !=
                           (wasConnected ? connected->second
                                         : SdfPathVector())) {
                    return true;
                }
            }
        }
    }
    return false;
}

void
RigExecRigEvaluator::_OnObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &)
{
    ++_stageEditSerial;
    // Never evaluate in a notice callback: ExecUsd must finish invalidating
    // its own caches before the next pull. External inputs can live anywhere
    // on the stage, so retain conservative structural checks after edits.
    // EXCEPT for a notice that is provably nothing but new VALUES on the
    // numeric avar channels: an Avar Editor slider tick, a typed value, a
    // key moved, a released gizmo. The structure digest never reads an
    // avar's value (only rigExec:* attributes, and the time-sample COUNT of
    // a handful of those), and the rest frames the epoch refresh on an edit
    // are rest:*, not avars -- so re-deriving both answered "nothing
    // changed" at ~115 ms per slider tick on the biped. Anything else in the
    // notice -- a resync, a non-avar property, a field other than a value --
    // keeps the conservative path.
    // A RESYNC on one of those channels still qualifies when it is on the
    // property alone. The first value a layer holds for an avar creates its
    // property spec, which USD reports as a property resync carrying only
    // typeName, and undoing it removes the spec, a resync carrying nothing.
    // Every released gizmo drag and its undo is exactly that pair, and
    // treating it as structure rebaked the program for ~250 ms per release.
    // A prim resync is never a value edit and keeps the conservative path.
    // And a notice that is not avar values reaches the digest only when it
    // could move it: _NoticeIsDigestSuspect is the prim-granular gate over
    // what the committed digest read outside the rig. An edit on a material
    // beside the rig, or on another rig, leaves the digest where it was and
    // costs the next settle nothing. The digest itself still trusts no
    // incremental invalidation: a suspect notice recomputes it whole.
    // A suspect notice also names, for the settle to judge, the paths that
    // could make it CERTAINLY structural: a prim whose path the digest
    // writes because of its type, a relationship whose targets it writes, a
    // connection inside a pose-input closure. When one of them has changed,
    // the digest has moved, and the settle compiles without computing it.
    const bool avarValuesOnly = _NoticeIsAvarValuesOnly(notice);
    // ... unless the avar the notice names is one a property chain READS.
    //
    // A chain's inputs:value may be connected to a control's avar -- that is
    // how every "distance from rest, from the control's own translate" chain
    // in this rig is built -- and a binding that was resolved before the
    // avar moved is resolved against a stale number. The bindings' own dirty
    // test watches interactive overrides and upstream chains, not authored
    // values, so a drag updated such a chain and AUTHORING the same avar did
    // not: the cheeks followed the mouth corner while it was moving and
    // stopped the moment it was let go.
    const bool chainReadsAnEditedAvar = avarValuesOnly &&
        !_propertyChainInputs.empty() && [this, &notice]() {
            for (const SdfPath &path : notice.GetResyncedPaths()) {
                if (_propertyChainInputs.count(path)) return true;
            }
            for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
                if (_propertyChainInputs.count(path)) return true;
            }
            return false;
        }();
    if (chainReadsAnEditedAvar) {
        // The one thing an avar-only notice CAN invalidate, dropped on its
        // own so the rest of the fast path is untouched: an Avar Editor tick
        // still skips the skin layouts, the blend samples and the rest.
        _propertyChainBindings.reset();
    }
    if (!avarValuesOnly && !_NoticeIsAvarChannelValuesOnly(notice)) {
        if (_NoticeIsDigestSuspect(notice)) {
            _structureDirty = true;
            _NoteCertainStructuralCandidates(notice);
        } else if (!_structureDirty && _compiled &&
                   _DigestGateVerifyRequested()) {
            _VerifyNeutralNotice(notice);
        }
    }
    // The epoch's rest frames answer to their own gate, not to the digest:
    // the digest reads no rest:* value, so a rest:rx edit is not a
    // structural edit and never recompiles, and the rests it moved are still
    // owed to the dynamic walk.
    _NoteRestEdits(notice);
    // The value caches that outlive a generation hold authored values and
    // where they come from, so an edit has to reach the very next read of
    // anything it moved -- and only that: dropping them whole on every
    // notice cost ~2.5 ms of re-binding and re-reading on the first frame
    // after any edit on the biped, for chains, layouts and shapes the edit
    // never touched.
    _ClearValueCaches(notice, avarValuesOnly);
    // The baked program captured values, and the epoch digest is deliberately
    // blind to values, so the digest cannot say whether one of them moved.
    // The program's own index of what the bake read can: a notice that hits
    // it asks for a rebuild, and one that misses it -- a value on an input
    // read per frame, anything on a prim the bake never looked at -- leaves
    // the program standing, which is what keeps an edit elsewhere in the
    // scene from degrading the rig to the dynamic path.
    if (_bakedProgram) {
        // Classified first, through the same query the registry's notice
        // adapter consumes: the branch below and the adapter's
        // retire/re-resolve decision read one verdict.
        _lastNoticeDisposition =
            ClassifyNoticeDisposition(notice, &_lastNoticePatchedPaths);
        if (_lastNoticeDisposition == RigExecNoticeDisposition::Patched) {
            // Patched in place: the program's avar table already holds the
            // new constants, and its by-value slot comparison re-runs only
            // their cone. No rebuild, and no stamp bump -- the bump would
            // mark the whole program dirty for one run to find what the
            // comparison already finds. The dry run inside the
            // classification already passed, so this applies; a refusal
            // (impossible on this thread, but fail-closed) falls back to
            // the stamp bump.
            if (!_bakedProgram->ApplyAvarValueEdits(notice)) {
                _bakedProgram->BumpProgramStamp();
            } else if (!_NoticeIsAvarValuesOnly(notice) &&
                       !_bakedProgram->ApplyValueEdits(
                           notice, /* skipPatchableAvars = */ true)) {
                // The live channels the classification routed beside the
                // patch; fail-closed as above.
                _bakedProgram->BumpProgramStamp();
            }
        } else if (_lastNoticeDisposition ==
                   RigExecNoticeDisposition::Edited) {
            // Routed like an override placed for one run and lifted: the
            // inputs the edit reached are marked, and the next run re-runs
            // the cone of the steps that read them instead of everything.
            // Whatever else the notice named is compared by value where it
            // is read, or read by nothing. A refusal (impossible after the
            // classification, but fail-closed) falls back to the stamp.
            if (!_bakedProgram->ApplyValueEdits(notice)) {
                _bakedProgram->BumpProgramStamp();
            }
        } else if (_lastNoticeDisposition ==
                   RigExecNoticeDisposition::Stale) {
            _bakedProgramStale = true;
            // The rebuild is allowed to refuse where the standing program did
            // not, and a refusal remembered from before this notice would
            // otherwise answer for a stage that has since changed.
            _bakeRefused = false;
            _bakeRefusalReasons.clear();
            _bakeBail = _BakeBailMemo();
        } else {
            // The other half of the same index, and the reason the program
            // may skip work at all. A notice that misses the index and that
            // the program could not route is a value edit on something the
            // frame path re-reads through a reader no dirty set names -- or
            // a resync, or layer metadata -- so the program is still right
            // about its structure and may be wrong about any value it cached
            // from the last frame. Saying so here is what makes the next
            // generation run everything once; the program compares its own
            // sources by value from then on.
            _bakedProgram->BumpProgramStamp();
        }
    } else {
        _lastNoticeDisposition = RigExecNoticeDisposition::None;
        _lastNoticePatchedPaths.clear();
    }
    // rigExec:baked is a VALUE on the rig root, so the epoch digest is blind
    // to it and _SettleEpoch will not recompile for it: this is the only
    // place a flip can be seen. Nothing is BUILT here -- never evaluate in a
    // notice callback -- and nothing needs to be. Dropping the program is
    // the whole of true -> false, and false -> true is built by the lazy
    // build in Evaluate, which is the same path SetEvaluationMode leaves
    // behind when it is asked on an epoch that has not settled.
    if (_NoticeNamesTheBakedAttribute(notice) &&
        _RefreshAttributeEvaluationMode() && !_ModeRunsProgram()) {
        _bakedProgram.reset();
        _bakedProgramPublished = false;
        _lastGenerationRanProgram = false;
        _bakedProgramStale = false;
    }
    // The seed, connected, and guide requests read authored values straight
    // off the stage; an edit that leaves the override tuple unchanged (a
    // rest attribute, a weight, a goal transform) still changes what they
    // compute. Any stage edit therefore retires their cached snapshots, the
    // same way it retires the affected solver batches below.
    // Retire means CLEAR, not just flagging: the seed and batch caches
    // are time-keyed LRUs, and the dirty flag only forces the FIRST
    // post-edit call to recompute. Once it clears, the other times would
    // hit pre-edit entries whose override tuple still matches -- the edit
    // changed the stage beneath an identical key.
    _firstFramePoseDirty = true;
    _firstFramePoseCache.Clear();
    _authSnapshotDirty = true;
    _authSnapTimeKeyed.clear();
    _guideDirty = true;
    _connectedPoseCache.clear();
    for (auto &[target, derived] : _derivedCache) {
        derived.cached = false;
    }
    const auto dirty = [this](const std::set<size_t> &batches) {
        for (size_t index : batches) {
            _SolverBatch &batch = _solverBatches[index];
            batch.dirty = true;
            // Beside the flag: the per-batch cache is a time-keyed LRU,
            // and the flag alone only forces the first post-edit call to
            // recompute -- the other times would hit pre-edit entries.
            batch.cache.Clear();
            // And the request it evaluates in, when that is shared: the
            // merged entries for other times hold this solver's pre-edit
            // answer too. (The walk vetoes the request's own lookup while
            // any member is dirty, so the flag needs no copy there.)
            _solverBatches[batch.leader].requestCache.Clear();
        }
    };
    if (_solverInputIndexAbsent) {
        // A deferred epoch has no index to route through until its first
        // dynamic generation builds one, so every batch is taken to have
        // been reached. Nothing is lost by it: no batch has computed, and so
        // cached, anything before that same generation.
        for (_SolverBatch &batch : _solverBatches) {
            batch.dirty = true;
            batch.cache.Clear();
            batch.requestCache.Clear();
        }
        return;
    }
    for (const SdfPath &property : notice.GetChangedInfoOnlyPaths()) {
        const auto input = _solverInputBatches.find(property.GetPrimPath());
        if (input != _solverInputBatches.end()) {
            dirty(input->second);
        }
    }
    const auto dirtySubtree = [this, &dirty](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        for (auto input = _solverInputBatches.lower_bound(primPath);
             input != _solverInputBatches.end() &&
             input->first.HasPrefix(primPath); ++input) {
            dirty(input->second);
        }
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        dirtySubtree(path);
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        dirtySubtree(path);
    }
}

} // namespace rigExec
