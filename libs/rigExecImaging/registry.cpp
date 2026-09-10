//
// RigExec imaging registry and C activation surface.
//
#include "registry.h"

#include "rigExecMath/avarScale.h"

#include "pxr/base/gf/bbox3d.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/type.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usdGeom/boundable.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/usd/usdGeom/boundableComputeExtent.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdUtils/stageCache.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace rigExec {

namespace {

uint64_t
_HashEpochPart(uint64_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= uint64_t(bytes[i]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

uint64_t
_HashEpochPart(uint64_t hash, const std::string &value)
{
    return _HashEpochPart(hash, value.data(), value.size());
}

// Read dependencies may live outside the assets whose outputs we publish.
// Follow authored relationships and attribute connections transitively, also
// inspecting ancestors because provider frames can inherit their inputs.
std::set<SdfPath>
_CollectReadRoots(const UsdStageRefPtr &stage,
                  const std::set<SdfPath> &assetRoots)
{
    std::set<SdfPath> roots = assetRoots;
    std::vector<SdfPath> pending(assetRoots.begin(), assetRoots.end());
    std::set<SdfPath> scannedPrims;
    std::set<SdfPath> scannedSubtrees;
    const auto enqueue = [&](const SdfPath &target) {
        const SdfPath path = target.GetPrimPath();
        if (!path.IsAbsolutePath() || !path.IsPrimPath()) return;
        roots.insert(path);
        for (const SdfPath &scope : scannedSubtrees) {
            if (path.HasPrefix(scope)) return;
        }
        pending.push_back(path);
    };
    const auto scan = [&](const UsdPrim &prim) {
        if (!prim || !scannedPrims.insert(prim.GetPath()).second) return;
        for (const UsdRelationship &rel : prim.GetRelationships()) {
            SdfPathVector targets;
            rel.GetTargets(&targets);
            for (const SdfPath &target : targets) enqueue(target);
        }
        for (const UsdAttribute &attr : prim.GetAttributes()) {
            SdfPathVector connections;
            attr.GetConnections(&connections);
            for (const SdfPath &target : connections) enqueue(target);
        }
    };
    while (!pending.empty()) {
        const SdfPath path = pending.back();
        pending.pop_back();
        if (!scannedSubtrees.insert(path).second) continue;
        const UsdPrim root = stage->GetPrimAtPath(path);
        if (!root) continue;
        for (const UsdPrim &prim : UsdPrimRange(root)) scan(prim);
        for (UsdPrim parent = root.GetParent(); parent && !parent.IsPseudoRoot();
             parent = parent.GetParent()) {
            scan(parent);
        }
    }
    // Keep only maximal scopes; scanning a notice then costs the number of
    // independent dependency regions, not the number of connected properties.
    std::set<SdfPath> compact;
    for (const SdfPath &path : roots) {
        bool covered = false;
        for (const SdfPath &root : compact) {
            if (path.HasPrefix(root)) {
                covered = true;
                break;
            }
        }
        if (!covered) compact.insert(path);
    }
    return compact;
}

}  // namespace

RigExecImagingRegistry &
RigExecImagingRegistry::GetInstance()
{
    static RigExecImagingRegistry instance;
    return instance;
}

RigExecImagingRegistry::RigExecImagingRegistry()
    : _store(std::make_shared<RigExecSnapshotStore>())
{
}

void
RigExecImagingRegistry::RegisterChain(
    const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
    const RigExecBindingResolvingSceneIndexRefPtr &binding,
    const RigExecResultsSceneIndexRefPtr &results)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _chains.push_back(
        {TfWeakPtr<RigExecInternalPrimPruningSceneIndex>(
             get_pointer(pruning)),
         TfWeakPtr<RigExecBindingResolvingSceneIndex>(get_pointer(binding)),
         TfWeakPtr<RigExecResultsSceneIndex>(get_pointer(results))});
    // A chain constructed after activation adopts every active rig's scope.
    if (!_generatedScopes.empty()) {
        pruning->SetOwnedScopes(_generatedScopes);
    }
}

bool
RigExecImagingRegistry::_EvaluateSessions(
    RigSessions *sessions,
    const UsdStageRefPtr &stage,
    UsdTimeCode time,
    std::shared_ptr<RigExecImagingSnapshot> *snapshot,
    RigExecBindingResolvingSceneIndex::BindingEpochConstPtr *epoch,
    std::vector<std::string> *errors)
{
    auto combined = std::make_shared<RigExecImagingSnapshot>();
    combined->stage = UsdStageWeakPtr(stage);
    combined->sampleTimeIsDefault = time.IsDefault();
    combined->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();

    auto combinedEpoch = std::make_shared<
        RigExecBindingResolvingSceneIndex::BindingEpoch>();
    uint64_t epochId = 1469598103934665603ULL;
    SdfPath commonAssetRoot;
    bool firstRoot = true;

    for (RigSession &session : *sessions) {
        RigExecImagingSnapshotConstPtr rigSnapshot = session.store->Get();
        if (session.dirty || !rigSnapshot || !rigSnapshot->Describes(stage, time)) {
            const RigExecImagingBridge::PublishResult result =
                session.bridge->EvaluateAndPublishResult(time);
            ++session.evaluationCount;
            if (!result.ok) {
                if (errors) {
                    errors->push_back(
                        "initial/evaluated generation failed for rig " +
                        session.rigPath.GetString());
                }
                return false;
            }
            if (result.epoch) session.epoch = result.epoch;
            session.dirty = false;
            rigSnapshot = session.store->Get();
        }

        if (!rigSnapshot || !rigSnapshot->Describes(stage, time)) {
            if (errors) {
                errors->push_back(
                    "rig did not publish the requested stage/time: " +
                    session.rigPath.GetString());
            }
            return false;
        }

        // A zero digest is legal.  The bridge normally supplies the epoch on
        // its first publication; construct the same payload defensively so a
        // legal zero cannot leave a stage without a binding epoch.
        if (!session.epoch) {
            auto rigEpoch = std::make_shared<
                RigExecBindingResolvingSceneIndex::BindingEpoch>();
            rigEpoch->id = session.bridge->GetBindingEpochDigest();
            for (const auto &[path, published] : rigSnapshot->prims) {
                rigEpoch->publishedPrims.insert(path);
            }
            session.epoch = std::move(rigEpoch);
        }

        epochId = _HashEpochPart(epochId, session.rigPath.GetString());
        const uint64_t rigEpochId = session.epoch->id;
        epochId = _HashEpochPart(epochId, &rigEpochId, sizeof(rigEpochId));
        combinedEpoch->publishedPrims.insert(
            session.epoch->publishedPrims.begin(),
            session.epoch->publishedPrims.end());
        combined->xformResetPaths.insert(rigSnapshot->xformResetPaths.begin(),
                                        rigSnapshot->xformResetPaths.end());

        if (firstRoot) {
            commonAssetRoot = session.assetRoot;
            firstRoot = false;
        } else if (commonAssetRoot != session.assetRoot) {
            commonAssetRoot = SdfPath();
        }

        for (const auto &[path, published] : rigSnapshot->prims) {
            if (!combined->prims.emplace(path, published).second) {
                if (errors) {
                    errors->push_back(
                        "two rigs publish the same Hydra prim " +
                        path.GetString() + "; cross-rig output overlap is "
                        "not allowed");
                }
                return false;
            }
        }
    }

    // Reserve zero for the registry's no-published-epoch state.
    combinedEpoch->id = epochId ? epochId : 1;
    combined->assetRoot = commonAssetRoot;
    *snapshot = std::move(combined);
    *epoch = std::move(combinedEpoch);
    return true;
}

RigExecImagingBridge::PublishResult
RigExecImagingRegistry::_Publish(
    std::shared_ptr<RigExecImagingSnapshot> snapshot,
    const RigExecBindingResolvingSceneIndex::BindingEpochConstPtr &epoch)
{
    RigExecImagingBridge::PublishResult result;
    snapshot->generation = ++_generation;
    result.dirtied = _store->Publish(std::move(snapshot));
    if (epoch && epoch->id != _publishedEpochId) {
        result.epoch = epoch;
        _publishedEpochId = epoch->id;
    }
    result.ok = true;
    return result;
}

bool
RigExecImagingRegistry::Activate(
    const UsdStageRefPtr &stage, const SdfPath &rigPath,
    UsdTimeCode initialTime, std::vector<std::string> *errors)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!stage) {
        if (errors) {
            errors->push_back("no stage to activate");
        }
        return false;
    }

    std::vector<SdfPath> rigPaths;
    if (!rigPath.IsEmpty()) {
        rigPaths.push_back(rigPath);
    } else {
        for (const UsdPrim &prim : stage->Traverse()) {
            if (prim.GetTypeName() == "RigExecRoot") {
                rigPaths.push_back(prim.GetPath());
            }
        }
    }
    std::sort(rigPaths.begin(), rigPaths.end());
    rigPaths.erase(std::unique(rigPaths.begin(), rigPaths.end()),
                   rigPaths.end());
    if (rigPaths.empty()) {
        if (errors) {
            errors->push_back("no RigExecRoot prim found");
        }
        return false;
    }

    // Edit-driven re-evaluation: listen on the source stage so property
    // edits under any active character asset republish at the current time.
    //
    // REGISTERED FIRST, BEFORE Compile(), AND THAT ORDER IS LOAD-BEARING.
    //
    // Tf_NoticeRegistry::_Register PREPENDS its deliverer, so listeners are
    // delivered most-recently-registered FIRST. OpenExec's own
    // ExecUsdSystem::_NoticeListener -- the thing that invalidates every
    // cached computed value when an attribute is authored -- is created by
    // the ExecUsdSystem constructor, which happens inside Compile(). So
    // registering after Compile() put us AHEAD of exec in delivery order,
    // and _OnObjectsChanged re-evaluated while exec still held the
    // pre-edit value: every generation published from an edit was one
    // edit stale. Measured on 11_VolumeWeights: with the overlay on and
    // inputs:falloffMax dragged 4.46 -> 9 -> 1.5, the stage already
    // resolved to the new value inside the callback while the published
    // field was the previous one, and a second Evaluate in the same
    // dispatch was equally stale -- only an evaluation after the dispatch
    // ended came back fresh.
    //
    // Registering first puts us at the BACK of the list, which is where a
    // re-evaluating listener belongs: every recompile builds a new
    // ExecUsdSystem that prepends ahead of us again, so this holds for the
    // life of the rig rather than only until the first structural edit.
    //
    // Registering this early also arms _OnObjectsChanged across the whole
    // compile-and-evaluate window, so _assetRoots -- what arms it -- stays
    // empty until the commit below, and a failure retains the previously
    // active stage's listener. Nothing in that window may author to \p stage
    // regardless: _OnObjectsChanged takes the same non-recursive _mutex this
    // function holds. Neither Compile() nor evaluation authors anything by
    // construction (that is what testRigExecNoAuthoring asserts), which is
    // what makes this safe.
    const std::set<SdfPath> previousAssetRoots = _assetRoots;
    _assetRoots.clear();
    TfNotice::Key candidateChangeKey = TfNotice::Register(
        TfCreateWeakPtr(this), &RigExecImagingRegistry::_OnObjectsChanged,
        stage);

    // Activation is transactional: a rig that fails to compile or evaluate
    // leaves the previously published generation, and the listener feeding
    // it, exactly as they were rather than tearing the viewport down.
    // Keep the previous key registered until commit: re-registering it on
    // rollback would put it ahead of the retained ExecUsdSystem listener,
    // causing the next edit to publish before exec invalidates its cache.
    const auto abandon = [&]() {
        TfNotice::Revoke(candidateChangeKey);
        _assetRoots = previousAssetRoots;
    };

    // Prepare a complete replacement without touching the active stage,
    // scene-index scopes, or published store.
    RigSessions candidate;
    candidate.reserve(rigPaths.size());
    for (const SdfPath &path : rigPaths) {
        const UsdPrim rig = stage->GetPrimAtPath(path);
        if (!rig || rig.GetTypeName() != "RigExecRoot") {
            if (errors) {
                errors->push_back(
                    "activation path is not a RigExecRoot: " +
                    path.GetString());
            }
            abandon();
            return false;
        }
        RigSession session;
        session.rigPath = path;
        session.assetRoot = path.GetParentPath();
        session.store = std::make_shared<RigExecSnapshotStore>();
        session.bridge = std::make_unique<RigExecImagingBridge>(
            stage, path, session.store);
        // A selection made before this rig was activated applies to it: the
        // overlay is a viewer mode, not a property of one bridge. Set before
        // the first evaluation so the initial generation already carries it.
        session.bridge->SetWeightOverlay(_weightOverlay);
        if (!session.bridge->Compile(errors)) {
            abandon();
            return false;
        }
        candidate.push_back(std::move(session));
    }

    std::shared_ptr<RigExecImagingSnapshot> initialSnapshot;
    RigExecBindingResolvingSceneIndex::BindingEpochConstPtr initialEpoch;
    if (!_EvaluateSessions(&candidate, stage, initialTime,
                           &initialSnapshot, &initialEpoch, errors)) {
        abandon();
        return false;
    }

    // Commit the fully compiled and evaluated stage in one transaction.
    // Repopulating _assetRoots is what arms _OnObjectsChanged.
    TfNotice::Revoke(_changeKey);
    _changeKey = candidateChangeKey;
    _sessions = std::move(candidate);
    _stage = stage;
    _generatedScopes.clear();
    _assetRoots.clear();
    for (const RigSession &session : _sessions) {
        _generatedScopes.insert(session.bridge->GetGeneratedScope());
        _assetRoots.insert(session.assetRoot);
    }
    _RefreshReadRoots();
    for (Chain &chain : _chains) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes(_generatedScopes);
        }
    }
    _lastTime = initialTime;
    _Broadcast(_Publish(std::move(initialSnapshot), initialEpoch));
    return true;
}

bool
RigExecImagingRegistry::SetTime(UsdTimeCode time)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_stage) {
        return false;
    }
    if (_readRootsDirty) {
        _RefreshReadRoots();
    }
    std::shared_ptr<RigExecImagingSnapshot> snapshot;
    RigExecBindingResolvingSceneIndex::BindingEpochConstPtr epoch;
    if (!_EvaluateSessions(
            &_sessions, _stage, time, &snapshot, &epoch, nullptr)) {
        // A rig that can no longer evaluate stops driving the scene.
        //
        // The per-session bridge already cleared its own store on the way
        // out, but this COMBINED generation is the one the scene indices
        // read, and returning here left the last good one current: remove a
        // mover's rigExec:moves and the recompile fails, so the mesh under
        // the driven Xform stayed where the constraint had put it through
        // every later edit and frame change.
        //
        // Cleared exactly the way Deactivate does, and broadcast for the
        // same reason: the store being right is worth nothing if the
        // observers are never told.
        _lastTime = time;
        RigExecImagingBridge::PublishResult cleared;
        cleared.ok = true;
        cleared.dirtied = _store->Publish(nullptr);
        // The next good generation must re-announce its epoch; the one the
        // binding index holds names prims this clear just removed.
        _publishedEpochId = 0;
        _Broadcast(cleared);
        return false;
    }
    _lastTime = time;
    _Broadcast(_Publish(std::move(snapshot), epoch));
    return true;
}

void
RigExecImagingRegistry::_RefreshReadRoots()
{
    _readRoots.clear();
    for (RigSession &session : _sessions) {
        if (session.readRootsDirty) {
            session.readRoots = _CollectReadRoots(_stage, {session.assetRoot});
            session.readRootsDirty = false;
        }
        _readRoots.insert(session.readRoots.begin(), session.readRoots.end());
    }
    _readRootsDirty = false;
}

size_t
RigExecImagingRegistry::GetSessionEvaluationCount(const SdfPath &rigPath)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rigPath) return session.evaluationCount;
    }
    return 0;
}

bool
RigExecImagingRegistry::SetWeightOverlay(const std::string &weightPrimPath)
{
    UsdTimeCode time;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        SdfPath resolved;
        if (!weightPrimPath.empty()) {
            // An arbitrary caller-supplied string reaches SdfPath here and
            // its constructor is loud about a malformed one. Ask first --
            // the same guard RigExecImaging_GetGuideBoundsAssetSpace uses.
            if (!SdfPath::IsValidPathString(weightPrimPath)) {
                return false;
            }
            resolved = SdfPath(weightPrimPath);
            if (!resolved.IsAbsolutePath() || !resolved.IsPrimPath()) {
                return false;
            }
        }
        _weightOverlay = resolved;
        if (_sessions.empty()) {
            // Remembered for whenever a rig is activated. Not a failure:
            // a host that sets its viewer mode before opening a stage has
            // done nothing wrong.
            return true;
        }
        // Offered to every active rig: the path selects at most one rig's
        // weight object, and a bridge that does not own it draws no overlay.
        for (RigSession &session : _sessions) {
            session.bridge->SetWeightOverlay(resolved);
            session.dirty = true;
        }
        time = _lastTime;
    }
    // Republished OUTSIDE the lock, exactly as _OnObjectsChanged does it:
    // SetTime takes the same plain (non-recursive) std::mutex, so calling
    // it while still holding _mutex is a self-deadlock. This is also what
    // makes the change visible immediately -- the fresh generation's diff
    // reports the overlay's displayColor primvar appearing or disappearing
    // as structural, which dirties the mesh universally.
    return SetTime(time);
}

void
RigExecImagingRegistry::_OnObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &sender)
{
    RigExecTapSet::PrepareStageChange(UsdStageRefPtr(sender), notice);
    std::set<SdfPath> readRoots;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || _assetRoots.empty() ||
            sender != UsdStageWeakPtr(_stage)) {
            return;
        }
        readRoots = _readRoots;
    }
    // Any edit touching the asset or its transitive reads can factor into
    // the final frame (solvers, joints, movers, controls, weights, driver geometry,
    // guide styling): re-evaluate at the current time. The evaluator's
    // epoch digest turns structural edits into recompiles; value edits
    // flow through exec invalidation on the shared layers.
    auto touchesInput = [&readRoots](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        for (const SdfPath &root : readRoots) {
            if (primPath.HasPrefix(root) || root.HasPrefix(primPath)) {
                return true;
            }
        }
        return false;
    };
    bool relevant = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (touchesInput(path)) {
            relevant = true;
            break;
        }
    }
    if (!relevant) {
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            if (touchesInput(path)) {
                relevant = true;
                break;
            }
        }
    }
    if (relevant) {
        UsdTimeCode time;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // A value-only edit keeps the cached dependency regions. Resyncs
            // and connection/relationship edits may introduce a new external
            // input even when the evaluator's binding epoch stays unchanged.
            _readRootsDirty = _readRootsDirty ||
                !notice.GetResyncedPaths().empty();
            for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field == "connectionPaths" || field == "targetPaths") {
                        _readRootsDirty = true;
                    }
                }
            }
            for (RigSession &session : _sessions) {
                const auto affects = [&session](const SdfPath &path) {
                    const SdfPath prim = path.GetPrimPath();
                    for (const SdfPath &root : session.readRoots) {
                        if (prim.HasPrefix(root) || root.HasPrefix(prim)) return true;
                    }
                    return false;
                };
                bool affected = false;
                for (const SdfPath &path : notice.GetResyncedPaths())
                    affected = affected || affects(path);
                for (const SdfPath &path : notice.GetChangedInfoOnlyPaths())
                    affected = affected || affects(path);
                if (affected) {
                    session.dirty = true;
                    session.readRootsDirty = session.readRootsDirty || _readRootsDirty;
                }
            }
            time = _lastTime;
        }
        SetTime(time);
    }
}

void
RigExecImagingRegistry::Deactivate()
{
    std::lock_guard<std::mutex> lock(_mutex);
    TfNotice::Revoke(_changeKey);
    _changeKey = TfNotice::Key();
    _assetRoots.clear();
    _readRoots.clear();
    _readRootsDirty = false;
    _generatedScopes.clear();
    _sessions.clear();
    _stage.Reset();
    _publishedEpochId = 0;
    for (Chain &chain : _chains) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes({});
        }
    }
    RigExecImagingBridge::PublishResult cleared;
    cleared.ok = true;
    cleared.dirtied = _store->Publish(nullptr);
    _Broadcast(cleared);
}

void
RigExecImagingRegistry::_Broadcast(
    const RigExecImagingBridge::PublishResult &result)
{
    if (!result.ok) {
        return;
    }
    // Prune chains whose scene index graphs were destroyed.
    _chains.erase(
        std::remove_if(_chains.begin(), _chains.end(),
                       [](const Chain &c) { return !c.results; }),
        _chains.end());
    for (Chain &chain : _chains) {
        if (result.epoch && chain.binding) {
            chain.binding->SetBindingEpoch(result.epoch);
        }
        if (chain.results) {
            chain.results->NotifyGenerationPublished(result.dirtied);
        }
    }
}

}  // namespace rigExec

// ---------------------------------------------------------------------------
// C activation surface.
// ---------------------------------------------------------------------------

using rigExec::RigExecImagingRegistry;

namespace {

// Grows \p range to cover everything one published prim draws, in ASSET
// space. Returns whether it contributed anything at all.
bool
_AccumulateGuideBounds(
    const rigExec::RigExecPublishedPrim &published, PXR_NS::GfRange3d *range)
{
    bool any = false;

    // Joint and solver guides: a sphere where requested plus a cone reaching
    // guideLength along that frame's +X axis. Joint link frames and lengths
    // are derived from evaluated parent/child origins; solver elements derive
    // them from frame landmarks. Bounding the cone by spheres at both ends is
    // conservative by design and also covers sphere-less branch links.
    for (size_t i = 0; i < published.guideFrames.size(); ++i) {
        const PXR_NS::GfMatrix4d &frame = published.guideFrames[i];
        const double radius = i < published.guideRadii.size()
            ? published.guideRadii[i] : 1.0;
        const double length = i < published.guideLengths.size()
            ? published.guideLengths[i] : 0.0;
        const PXR_NS::GfVec3d origin = frame.ExtractTranslation();
        // Row-vector convention: row 0 is the frame's X basis, and the
        // frame is orthonormalized upstream, so it is already unit length.
        const PXR_NS::GfVec3d tip = origin + length * frame.GetRow3(0);
        const PXR_NS::GfVec3d extent(radius, radius, radius);
        range->UnionWith(origin - extent);
        range->UnionWith(origin + extent);
        range->UnionWith(tip - extent);
        range->UnionWith(tip + extent);
        any = true;
    }

    // The control guide: every shape is documented as unit-sized and
    // centred on the frame origin with half-extent 1 (spec §10.3
    // extension), so the unit cube bounds all six of them. Deliberately
    // NOT the exact per-shape extent: that table lives in the scene index,
    // which this registry must not depend on, and using it would make the
    // framing distance jump around as an author retypes guide:shape
    // between a sphere and a flat circle.
    //
    // It must bound what is actually DRAWN, though, so it asks the scene
    // index's own predicate whether anything is: an unrecognized
    // shape/drawMode pair synthesizes no prim, and reporting a box for it
    // would frame a host's camera on empty space.
    if (published.hasControlGuide &&
        rigExec::RigExecControlGuideIsDrawn(published.controlGuideShape,
                                            published.controlGuideDrawMode)) {
        // Wire curves are drawn with a width, in the guide's own local
        // pre-scale units, so the drawn geometry reaches half a width
        // past the unit shape BEFORE the per-axis scale applies.
        double halfWidth = 0.0;
        if (published.controlGuideDrawMode == PXR_NS::TfToken("wire") &&
            std::isfinite(published.controlGuideWireWidth) &&
            published.controlGuideWireWidth > 0.0) {
            halfWidth = published.controlGuideWireWidth * 0.5;
        }
        const PXR_NS::GfVec3d &scale = published.controlGuideScale;
        const PXR_NS::GfVec3d half(scale[0] * (1.0 + halfWidth),
                                   scale[1] * (1.0 + halfWidth),
                                   scale[2] * (1.0 + halfWidth));
        // The frame is rigid, so aligning the transformed box is exact.
        range->UnionWith(
            PXR_NS::GfBBox3d(PXR_NS::GfRange3d(-half, half),
                             published.controlGuideFrame)
                .ComputeAlignedRange());
        any = true;
    }

    // Volume weight iso-surfaces. Their points are already the drawn
    // geometry in the element's own local space, so the exact bound is one
    // transformed range -- no per-shape table and no unit-size convention
    // to keep in step, which is what the control arm above has to settle
    // for. The implicits (a solid sphere iso-surface) carry no points and
    // are the unit sphere by construction, so ±1 is exact for them too.
    for (const rigExec::RigExecVolumeGuideElement &element :
             published.volumeGuides) {
        PXR_NS::GfRange3d local;
        for (const PXR_NS::GfVec3f &p : element.points) {
            local.UnionWith(PXR_NS::GfVec3d(p));
        }
        if (local.IsEmpty()) {
            local = PXR_NS::GfRange3d(PXR_NS::GfVec3d(-1, -1, -1),
                                      PXR_NS::GfVec3d(1, 1, 1));
        }
        // Wire curves are drawn with a width in the element's own local
        // units, so the drawn geometry reaches half a width past the
        // points -- the same correction the control arm applies.
        if (element.wireWidth > 0.0 &&
            std::isfinite(element.wireWidth)) {
            const PXR_NS::GfVec3d pad(element.wireWidth * 0.5);
            local = PXR_NS::GfRange3d(local.GetMin() - pad,
                                      local.GetMax() + pad);
        }
        range->UnionWith(
            PXR_NS::GfBBox3d(local, element.xform *
                published.volumeGuideAnchorToAsset).ComputeAlignedRange());
        any = true;
    }
    return any;
}

// The union of everything published beneath \p path that shares its
// PURPOSE, including \p path itself.
//
// A Boundable's extent is authoritative for its whole subtree:
// UsdGeomBBoxCache stops descending at one ("Boundables should always
// provide their own extent and do not require participation from
// descendants", bboxCache.cpp). RigExec nests providers as a matter of
// course -- a joint chain is joints under joints -- so an extent covering
// only its own guide silently drops every descendant from any ancestor's
// bound, and framing a rig framed its first joint.
//
// Purpose-scoped, though, because one extent carries ONE purpose: the
// cache files this box under the boundable's own resolved purpose. Folding
// a default-purpose control nested under a guide-purpose joint into that
// box would file the control's bounds under `guide`, so a viewer with
// guides off would frame around geometry it is not showing -- and a viewer
// with guides on would frame around it twice. Differing-purpose
// descendants are excluded here and warned about at compile.
bool
_AccumulateSubtreeGuideBounds(
    const rigExec::RigExecImagingSnapshot &snapshot, const PXR_NS::SdfPath &path,
    const PXR_NS::TfToken &purpose, PXR_NS::GfRange3d *range)
{
    bool any = false;
    // The published set is path-keyed and sorted, so the subtree is one
    // contiguous run beginning at the prim itself.
    for (auto it = snapshot.prims.lower_bound(path);
         it != snapshot.prims.end() && it->first.HasPrefix(path); ++it) {
        if (it->second.guidePurpose != purpose) {
            continue;
        }
        any = _AccumulateGuideBounds(it->second, range) || any;
    }
    return any;
}

// A prim's resolved render purpose -- the same value the bridge publishes
// and the same one UsdGeomBBoxCache files its extent under.
PXR_NS::TfToken
_ResolvedPurpose(const PXR_NS::UsdPrim &prim)
{
    if (const PXR_NS::UsdGeomImageable imageable =
            PXR_NS::UsdGeomImageable(prim)) {
        const PXR_NS::TfToken purpose = imageable.ComputePurpose();
        if (!purpose.IsEmpty()) {
            return purpose;
        }
    }
    return PXR_NS::UsdGeomTokens->default_;
}

// The authored REST frame of one RigExec transform provider, composed the
// same way RigExecJointRestSpace does it in computations.cpp: the rest
// avars as a local delta preceding the authored rest:space, orthonormalized
// (rest spaces always are, per the Ir contract).
//
// Duplicated rather than shared with the evaluator on purpose. This runs
// with no compiled rig and no exec system -- the whole point of the rest
// fallback is to answer for a stage nobody has evaluated -- so it can only
// read authored attributes, which is precisely what it does.
PXR_NS::GfMatrix4d
_AuthoredRestSpace(const PXR_NS::UsdPrim &prim, const PXR_NS::UsdTimeCode &time,
                   bool *rigid)
{
    auto scalar = [&prim, &time](const char *name) {
        double value = 0.0;
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return value;
    };
    static const PXR_NS::GfVec3d axes[3] = {
        PXR_NS::GfVec3d(1, 0, 0), PXR_NS::GfVec3d(0, 1, 0),
        PXR_NS::GfVec3d(0, 0, 1)};
    const double angles[3] = {scalar("rest:rx"), scalar("rest:ry"),
                              scalar("rest:rz")};
    PXR_NS::GfMatrix4d local(1.0);
    for (int index = 0; index < 3; ++index) {  // XYZ, the rest-avar order
        if (angles[index] != 0.0) {
            local = local * PXR_NS::GfMatrix4d(
                                PXR_NS::GfRotation(axes[index], angles[index]),
                                PXR_NS::GfVec3d(0));
        }
    }
    PXR_NS::GfMatrix4d translate(1.0);
    translate.SetTranslate(
        PXR_NS::GfVec3d(scalar("rest:tx"), scalar("rest:ty"),
                        scalar("rest:tz")));
    local = local * translate;

    PXR_NS::GfMatrix4d space(1.0);
    if (const PXR_NS::UsdAttribute a =
            prim.GetAttribute(PXR_NS::TfToken("rest:space"))) {
        a.Get(&space, time);
    }
    PXR_NS::GfMatrix4d rest = local * space;
    // Reported, not swallowed: the bridge refuses to publish a guide whose
    // frame cannot be orthonormalized, so an extent computed from one would
    // bound a guide the renderer declines to draw.
    *rigid = rest.Orthonormalize(/* issueWarning = */ false);
    if (!*rigid) {
        return rest;
    }
    // An authored rest is relative to the namespace frame provider, so it
    // only becomes a world frame once the ancestor chain is folded in --
    // the same multiply _JointRestSpace does (computations.cpp:230).
    // Without it a nested joint bounds at its parent-relative offset, which
    // for a chain authored as successive 2-unit steps collapses the whole
    // tail onto the origin.
    for (PXR_NS::UsdPrim parent = prim.GetParent();
         parent && !parent.IsPseudoRoot(); parent = parent.GetParent()) {
        const PXR_NS::TfToken parentType = parent.GetTypeName();
        if (parentType != "RigExecJoint" && parentType != "RigExecControl") {
            continue;
        }
        bool parentRigid = false;
        const PXR_NS::GfMatrix4d parentRest =
            _AuthoredRestSpace(parent, time, &parentRigid);
        if (parentRigid) {
            rest = rest * parentRest;
        }
        break;
    }
    return rest;
}

// The bounds a provider's guide would draw AT REST, from authored
// attributes alone.
//
// This is the answer for a stage that has never been evaluated -- opened in
// a host that has not activated RigExec, or queried before the first
// generation is published. It is deliberately the rest pose rather than
// nothing: a bounding box that collapses the moment the rig is not running
// makes framing fail exactly where a user reaches for it first.
bool
_AccumulateRestGuideBounds(
    const PXR_NS::UsdPrim &prim, const PXR_NS::UsdTimeCode &time,
    PXR_NS::GfRange3d *range)
{
    auto number = [&prim, &time](const char *name, double fallback) {
        double value = fallback;
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return value;
    };
    auto floatNumber = [&prim, &time](const char *name, double fallback) {
        float value = static_cast<float>(fallback);
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return double(value);
    };
    auto token = [&prim, &time](const char *name, const char *fallback) {
        PXR_NS::TfToken value(fallback);
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return value;
    };
    bool rigid = false;
    const PXR_NS::GfMatrix4d rest = _AuthoredRestSpace(prim, time, &rigid);
    if (!rigid) {
        return false;
    }
    const PXR_NS::TfToken type = prim.GetTypeName();

    if (type == "RigExecControl") {
        // Same predicate the scene index draws through: an unrecognized
        // shape/drawMode pair synthesizes nothing and must bound nothing.
        const PXR_NS::TfToken shape = token("guide:shape", "circle");
        const PXR_NS::TfToken drawMode = token("guide:drawMode", "wire");
        if (!rigExec::RigExecControlGuideIsDrawn(shape, drawMode)) {
            return false;
        }
        const PXR_NS::GfVec3d authoredScale(
            number("guide:scaleX", 1.0), number("guide:scaleY", 1.0),
            number("guide:scaleZ", 1.0));
        PXR_NS::GfVec3d scale(1.0);
        static const char *avarScaleNames[3] = {
            "avars:sx", "avars:sy", "avars:sz"};
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(authoredScale[i]) || authoredScale[i] <= 0.0) {
                return false;  // draws nothing, so it bounds nothing
            }
            // No evaluated frame exists on this fallback path. Resolve the
            // authored channel at the queried time with the SAME floor and
            // non-finite policy as the runtime, then size the guide by its
            // magnitude so a supported reflection remains visible.
            const double avarMagnitude = std::abs(
                rigExec::RigExecNormalizeAvarScale(
                    number(avarScaleNames[i], 1.0)));
            scale[i] = authoredScale[i] * avarMagnitude;
            if (!std::isfinite(scale[i]) || scale[i] <= 0.0) {
                return false;
            }
        }
        double halfWidth = 0.0;
        if (drawMode == PXR_NS::TfToken("wire")) {
            const double width = number("guide:wireWidth", 0.05);
            if (std::isfinite(width) && width > 0.0) {
                halfWidth = width * 0.5;
            }
        }
        // The unit shape, for the same reason the snapshot path uses it:
        // every guide shape is documented as unit-sized with half-extent 1.
        const PXR_NS::GfVec3d half(scale[0] * (1.0 + halfWidth),
                                   scale[1] * (1.0 + halfWidth),
                                   scale[2] * (1.0 + halfWidth));
        range->UnionWith(
            PXR_NS::GfBBox3d(PXR_NS::GfRange3d(-half, half), rest)
                .ComputeAlignedRange());
        return true;
    }
    if (type == "RigExecJoint") {
        const double radius = number("guide:radius", 1.0);
        if (!std::isfinite(radius) || radius <= 0.0) {
            return false;
        }
        const PXR_NS::GfVec3d origin = rest.ExtractTranslation();
        const PXR_NS::GfVec3d extent(radius, radius, radius);
        range->UnionWith(origin - extent);
        range->UnionWith(origin + extent);

        // A joint's outgoing links terminate at every descendant whose
        // nearest RigExecJoint ancestor is this prim. Intervening grouping
        // scopes therefore preserve the same hierarchy used by evaluation.
        for (const PXR_NS::UsdPrim &candidate :
             PXR_NS::UsdPrimRange(prim)) {
            if (candidate == prim ||
                candidate.GetTypeName() != "RigExecJoint") {
                continue;
            }
            PXR_NS::UsdPrim ancestor = candidate.GetParent();
            while (ancestor && ancestor != prim &&
                   ancestor.GetTypeName() != "RigExecJoint") {
                ancestor = ancestor.GetParent();
            }
            if (ancestor != prim) {
                continue;
            }
            bool childRigid = false;
            const PXR_NS::GfMatrix4d childRest =
                _AuthoredRestSpace(candidate, time, &childRigid);
            if (!childRigid) {
                continue;
            }
            const PXR_NS::GfVec3d tip = childRest.ExtractTranslation();
            if (!std::isfinite(tip[0]) || !std::isfinite(tip[1]) ||
                !std::isfinite(tip[2]) ||
                (tip - origin).GetLength() <= 1e-12) {
                continue;
            }
            range->UnionWith(tip - extent);
            range->UnionWith(tip + extent);
        }
        return true;
    }
    if (type == "RigExecSphereWeight" ||
        type == "RigExecPlaneWeight" ||
        type == "RigExecCurveWeight") {
        const PXR_NS::TfToken drawMode = token("guide:drawMode", "wire");
        if (drawMode != PXR_NS::TfToken("wire") &&
            drawMode != PXR_NS::TfToken("geometry")) {
            return false;
        }
        const bool wire = drawMode == PXR_NS::TfToken("wire");
        double halfWidth = 0.0;
        if (wire) {
            const double width = number("guide:wireWidth", 0.05);
            if (std::isfinite(width) && width > 0.0) {
                halfWidth = width * 0.5;
            }
        }
        const double falloffMin =
            floatNumber("inputs:falloffMin", 0.0);
        const double falloffMax =
            floatNumber("inputs:falloffMax", 1.0);
        if (!std::isfinite(falloffMin) || !std::isfinite(falloffMax)) {
            return false;
        }

        if (type == "RigExecPlaneWeight") {
            const PXR_NS::TfToken axisName =
                token("rigExec:planeAxis", "y");
            const int axis = axisName == PXR_NS::TfToken("x") ? 0
                : (axisName == PXR_NS::TfToken("y") ? 1
                   : (axisName == PXR_NS::TfToken("z") ? 2 : -1));
            const PXR_NS::TfToken bounds =
                token("rigExec:planeBounds", "unbounded");
            const double extentU = floatNumber("inputs:extentU", 1.0);
            const double extentV = floatNumber("inputs:extentV", 1.0);
            if (axis < 0 ||
                (bounds != PXR_NS::TfToken("bounded") &&
                 bounds != PXR_NS::TfToken("unbounded")) ||
                !std::isfinite(extentU) || !std::isfinite(extentV) ||
                extentU <= 0.0 || extentV <= 0.0) {
                return false;
            }
            // Unbounded planes add four outward ticks ending at 1.25 times
            // the rectangle half-extent. This is the exact local range of the
            // guide elements built by _AppendPlaneVolumeGuide.
            const double tickScale =
                bounds == PXR_NS::TfToken("unbounded") ? 1.25 : 1.0;
            PXR_NS::GfRange3d local;
            for (const double distance : {falloffMin, falloffMax}) {
                PXR_NS::GfVec3d min(-halfWidth), max(halfWidth);
                min[axis] += distance;
                max[axis] += distance;
                min[(axis + 1) % 3] -= extentU * tickScale;
                max[(axis + 1) % 3] += extentU * tickScale;
                min[(axis + 2) % 3] -= extentV * tickScale;
                max[(axis + 2) % 3] += extentV * tickScale;
                local.UnionWith(PXR_NS::GfRange3d(min, max));
            }
            range->UnionWith(
                PXR_NS::GfBBox3d(local, rest).ComputeAlignedRange());
            return true;
        }

        PXR_NS::GfVec3d axisScale(
            floatNumber("inputs:scaleX", 1.0),
            floatNumber("inputs:scaleY", 1.0),
            floatNumber("inputs:scaleZ", 1.0));
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(axisScale[axis]) || axisScale[axis] <= 0.0) {
                return false;
            }
        }
        if (type == "RigExecSphereWeight") {
            bool found = false;
            for (const double radius : {falloffMin, falloffMax}) {
                if (!std::isfinite(radius) || radius <= 0.0) {
                    continue;
                }
                PXR_NS::GfMatrix4d scale(1.0);
                scale.SetScale(PXR_NS::GfVec3d(
                    radius * axisScale[0], radius * axisScale[1],
                    radius * axisScale[2]));
                const PXR_NS::GfVec3d half(1.0 + halfWidth);
                range->UnionWith(
                    PXR_NS::GfBBox3d(PXR_NS::GfRange3d(-half, half),
                                     scale * rest)
                        .ComputeAlignedRange());
                found = true;
            }
            return found;
        }

        // CurveWeight: accept the same two relationship spellings as the
        // evaluator and bridge -- an exact point3f[] property or a prim whose
        // native `points` attribute supplies at least two vertices.
        PXR_NS::SdfPathVector targets;
        if (const PXR_NS::UsdRelationship rel =
                prim.GetRelationship(PXR_NS::TfToken("rigExec:curve"))) {
            rel.GetTargets(&targets);
        }
        if (targets.size() != 1) {
            return false;
        }
        const PXR_NS::SdfPath pointsPath = targets[0].IsPropertyPath()
            ? targets[0]
            : targets[0].AppendProperty(PXR_NS::TfToken("points"));
        PXR_NS::VtVec3fArray curvePoints;
        const PXR_NS::UsdAttribute points =
            prim.GetStage()->GetAttributeAtPath(pointsPath);
        if (!points || !points.Get(&curvePoints, time) ||
            curvePoints.size() < 2) {
            return false;
        }
        PXR_NS::GfMatrix4d divide(1.0);
        divide.SetScale(PXR_NS::GfVec3d(
            1.0 / axisScale[0], 1.0 / axisScale[1],
            1.0 / axisScale[2]));
        const PXR_NS::GfMatrix4d toLocal = rest.GetInverse() * divide;
        PXR_NS::GfRange3d localCenters;
        PXR_NS::GfVec3f previous(0.0f);
        size_t distinctPointCount = 0;
        for (const PXR_NS::GfVec3f &point : curvePoints) {
            const PXR_NS::GfVec3f localPoint(
                toLocal.Transform(PXR_NS::GfVec3d(point)));
            if (!std::isfinite(localPoint[0]) ||
                !std::isfinite(localPoint[1]) ||
                !std::isfinite(localPoint[2])) {
                return false;
            }
            // Match _AppendCurveVolumeGuide: consecutive coincident points
            // do not form a drawable segment and are collapsed before the
            // wire or mesh is built.
            if (distinctPointCount == 0 ||
                (localPoint - previous).GetLength() > 1e-6f) {
                localCenters.UnionWith(PXR_NS::GfVec3d(localPoint));
                previous = localPoint;
                ++distinctPointCount;
            }
        }
        if (distinctPointCount < 2) {
            return false;
        }
        PXR_NS::GfMatrix4d scale(1.0);
        scale.SetScale(axisScale);
        const PXR_NS::GfMatrix4d toAsset = scale * rest;
        bool found = false;
        for (const double radius : {falloffMin, falloffMax}) {
            if (!std::isfinite(radius) || radius <= 0.0) {
                continue;
            }
            // The bridge's rings live inside this radius-expanded box. It is
            // conservative at diagonal tangents, which is preferable to a
            // cold framing bound that clips a valid authored guide.
            const PXR_NS::GfVec3d pad(radius + halfWidth);
            const PXR_NS::GfRange3d local(
                localCenters.GetMin() - pad, localCenters.GetMax() + pad);
            range->UnionWith(
                PXR_NS::GfBBox3d(local, toAsset).ComputeAlignedRange());
            found = true;
        }
        return found;
    }
    return false;
}

// Writes \p range out as min xyz then max xyz.
void
_WriteBounds(const PXR_NS::GfRange3d &range, double outMinMax[6])
{
    const PXR_NS::GfVec3d min = range.GetMin();
    const PXR_NS::GfVec3d max = range.GetMax();
    for (int i = 0; i < 3; ++i) {
        outMinMax[i] = min[i];
        outMinMax[i + 3] = max[i];
    }
}

// UsdGeomBoundable::ComputeExtentFromPlugins entry point for every RigExec
// transform provider (spec §10.3 extension, host-durability redesign).
//
// This is what makes framing a control work in EVERY host rather than in
// the one whose Python we could reach. UsdGeomBBoxCache is what usdview,
// Solaris, and mayaUsd all consult, and it asks a Boundable for its extent;
// before this, RigExec types were not Boundable and reported nothing, so a
// rig had no bounds anywhere and framing a control moved the camera not at
// all. The usdview adapter used to monkeypatch computeWorldBound to paper
// over that -- a patch for one host, replaced by this.
//
// The extent BAKES the posed frame, because the prim carries no stage
// transform of its own: rest:space plus avars are the only transform
// authority (the Ir alignment), and they are asset-relative.
//
// "Asset-relative" is not "local", and an extent is LOCAL --
// UsdGeomBBoxCache multiplies it by the prim's own local-to-world. The two
// coincide only while nothing between the asset root and the provider
// contributes a transform. Since 2026-09-10 something may: an intervening
// Xformable is composed into the published frames
// (RigExecRigEvaluator::_ComposeInterveningXforms), so the snapshot branch
// below divides it back out. Leaving it in draws the guide in one place and
// its bounding box in another, offset by exactly that Xform.
//
// The rest fallback needs no such division: it reads authored rest
// attributes alone, which never carried the intervening transform in the
// first place. Both branches therefore return the same local quantity, and
// the box does not jump when a generation is published.
// The published bounds expressed in the prim's OWN space, by dividing out
// everything UsdGeomBBoxCache is about to re-apply between the asset root
// and this prim.
//
// That is the whole namespace chain, the prim's own xformOps included. Ops
// authored on a provider are not a transform authority for EVALUATION --
// rest:space and the avars are -- but BBoxCache applies them regardless, so
// an extent that ignored them would be wrong in the same way and for the
// same reason.
//
// Conservative rather than tight: an inverse-rotated box has to be
// re-aligned to axes to be expressed as a min/max pair, and USD re-aligns
// it again on the way out, so a rotated chain grows the box slightly.
// Bounds are allowed to be too big and are never allowed to be too small.
PXR_NS::GfRange3d
_AssetSpaceToLocal(
    const PXR_NS::UsdPrim &prim, const PXR_NS::SdfPath &assetRootPath,
    const PXR_NS::UsdTimeCode &time, const PXR_NS::GfRange3d &range)
{
    if (range.IsEmpty() || assetRootPath.IsEmpty()) {
        return range;
    }
    const PXR_NS::UsdPrim assetRoot =
        prim.GetStage()->GetPrimAtPath(assetRootPath);
    if (!assetRoot || prim == assetRoot) {
        return range;
    }
    PXR_NS::UsdGeomXformCache cache(time);
    bool resetsBelowAsset = false;
    const PXR_NS::GfMatrix4d localToAsset =
        cache.ComputeRelativeTransform(prim, assetRoot, &resetsBelowAsset);
    // A reset detaches the prim from the asset root, so there is no
    // relative transform to divide out and guessing at one would move the
    // box somewhere nobody asked for. Matches what the evaluator does with
    // the same condition.
    if (resetsBelowAsset || localToAsset == PXR_NS::GfMatrix4d(1.0)) {
        return range;
    }
    return PXR_NS::GfBBox3d(range, localToAsset.GetInverse())
        .ComputeAlignedRange();
}

bool
_ComputeRigExecGuideExtent(
    const PXR_NS::UsdGeomBoundable &boundable, const PXR_NS::UsdTimeCode &time,
    const PXR_NS::GfMatrix4d *transform, PXR_NS::VtVec3fArray *extent)
{
    const PXR_NS::UsdPrim prim = boundable.GetPrim();
    if (!prim || !extent) {
        return false;
    }
    PXR_NS::GfRange3d range;
    bool found = false;

    // The live generation wins -- but ONLY if it describes this stage at
    // this time. USD calls this with a stage and a time of its own
    // choosing, and the store is a process-global singleton, so an
    // ungated lookup by path answers a query about stage A frame 12 with
    // stage B's frame 30 pose: plausible, wrong, and undetectable
    // downstream. Identity is the stage OBJECT -- two stages commonly share
    // a root layer and differ only by session layer. Anything else falls through to the rest pose, which
    // makes this callback a pure function of (stage, time).
    if (const rigExec::RigExecImagingSnapshotConstPtr snapshot =
            RigExecImagingRegistry::GetInstance().GetStore()->Get()) {
        if (snapshot->Describes(prim.GetStage(), time)) {
            found = _AccumulateSubtreeGuideBounds(
                *snapshot, prim.GetPath(), _ResolvedPurpose(prim), &range);
            if (found) {
                range = _AssetSpaceToLocal(prim, snapshot->assetRoot, time,
                                           range);
            }
        }
    }
    // ...otherwise the rest pose, from authored attributes alone, so an
    // un-evaluated stage still frames. The subtree rule applies here too:
    // BBoxCache stops descending at a Boundable, so a provider's extent
    // has to speak for the providers nested under it.
    if (!found) {
        const PXR_NS::TfToken purpose = _ResolvedPurpose(prim);
        for (const PXR_NS::UsdPrim &descendant :
             PXR_NS::UsdPrimRange(prim)) {
            // Purpose-scoped for the same reason the snapshot path is: one
            // extent carries one purpose.
            if (_ResolvedPurpose(descendant) != purpose) {
                continue;
            }
            found = _AccumulateRestGuideBounds(descendant, time, &range) ||
                    found;
        }
    }
    if (!found || range.IsEmpty()) {
        return false;
    }
    if (transform) {
        range = PXR_NS::GfBBox3d(range, *transform).ComputeAlignedRange();
    }
    *extent = PXR_NS::VtVec3fArray{PXR_NS::GfVec3f(range.GetMin()),
                                   PXR_NS::GfVec3f(range.GetMax())};
    return true;
}

}  // namespace

PXR_NAMESPACE_OPEN_SCOPE

// Keyed by TfType rather than by the templated overload: the RigExec schema
// is CODELESS, so there is no C++ class to name as a template argument. The
// type still exists -- Plug declares it from the schema plugInfo, and its
// ancestor chain reaches UsdGeomBoundable -- which is all
// ComputeExtentFromPlugins needs to find this.
//
// Registered on the abstract base, not on each concrete type: the lookup
// walks a prim's ancestor types, so one registration answers for every
// provider that inherits it.
//
// Inside PXR_NAMESPACE_OPEN_SCOPE, like every other TF_REGISTRY_FUNCTION in
// this tree -- the macro's tag type has to resolve the way usdGeom's own
// subscription resolves it, and at global scope with a PXR_NS:: qualifier
// it registers into a registry nobody subscribes to, which fails silently.
TF_REGISTRY_FUNCTION(UsdGeomBoundable)
{
    // RigExecXformable covers joints and controls through its ancestors. The
    // volume base gets its own registration so its shape-specific authored
    // fallback can be framed cold; aggregate solvers draw guides too but
    // inherit Boundable directly, so they are named individually.
    for (const char *name : {"RigExecXformable",
                             "RigExecVolumeWeight",
                             "RigExecFkChain",
                             "RigExecTwoBoneIk",
                             "RigExecBlendPointFrames",
                             "RigExecTwistDistribution",
                             "RigExecRibbon"}) {
        const TfType type = TfType::FindByName(name);
        if (type.IsUnknown()) {
            // The schema plugin is not registered in this process, so
            // nothing can be Boundable anyway. Not worth shouting about.
            continue;
        }
        UsdGeomRegisterComputeExtentFunction(
            type, ::_ComputeRigExecGuideExtent);
    }
}

PXR_NAMESPACE_CLOSE_SCOPE

extern "C" {

int
RigExecImaging_Activate(
    long long stageCacheId, const char *rigPath, double initialFrame)
{
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(
            static_cast<long int>(stageCacheId)));
    if (!stage) {
        std::printf("rigExecImaging: no stage for cache id %lld\n",
                    stageCacheId);
        return 1;
    }

    PXR_NS::SdfPath path;
    if (rigPath && rigPath[0]) {
        if (!PXR_NS::SdfPath::IsValidPathString(rigPath)) {
            std::printf("rigExecImaging: invalid rig path '%s'\n", rigPath);
            return 2;
        }
        path = PXR_NS::SdfPath(rigPath);
    }

    std::vector<std::string> errors;
    if (!RigExecImagingRegistry::GetInstance().Activate(
            stage, path, PXR_NS::UsdTimeCode(initialFrame), &errors)) {
        for (const std::string &e : errors) {
            std::printf("rigExecImaging: %s\n", e.c_str());
        }
        return 3;
    }
    if (path.IsEmpty()) {
        std::printf("rigExecImaging: activated all RigExecRoot prims\n");
    } else {
        std::printf("rigExecImaging: activated %s\n", path.GetText());
    }
    return 0;
}

int
RigExecImaging_SetTime(double frame)
{
    return RigExecImagingRegistry::GetInstance().SetTime(
               PXR_NS::UsdTimeCode(frame))
        ? 0 : 1;
}

void
RigExecImaging_Deactivate()
{
    RigExecImagingRegistry::GetInstance().Deactivate();
}

long long
RigExecImaging_GetGeneration()
{
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    return snapshot ? static_cast<long long>(snapshot->generation) : 0;
}

int
RigExecImaging_GetControlFrameAssetSpace(
    long long stageCacheId, const char *primPath, double frame,
    int isDefault, double outMatrix[16])
{
    if (!primPath || !outMatrix || !PXR_NS::SdfPath::IsValidPathString(primPath) ||
        (!isDefault && !std::isfinite(frame))) return 0;
    const auto stage = PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(stageCacheId));
    const auto snapshot = RigExecImagingRegistry::GetInstance().GetStore()->Get();
    const auto time = isDefault ? PXR_NS::UsdTimeCode::Default() : PXR_NS::UsdTimeCode(frame);
    if (!stage || !snapshot || !snapshot->Describes(stage, time)) return 0;
    const auto it = snapshot->prims.find(PXR_NS::SdfPath(primPath));
    if (it == snapshot->prims.end() || !it->second.hasControlFrame) return 0;
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
        outMatrix[r*4+c] = it->second.controlFrame[r][c];
    return 1;
}

int
RigExecImaging_GetGuideBoundsAssetSpace(
    const char *primPath, double outMinMax[6])
{
    if (!primPath || !primPath[0] || !outMinMax) {
        return 0;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    if (!snapshot) {
        return 0;
    }
    // An arbitrary caller-supplied string reaches SdfPath here, and its
    // constructor is loud about a malformed one. Ask first.
    if (!PXR_NS::SdfPath::IsValidPathString(primPath)) {
        return 0;
    }
    const auto it = snapshot->prims.find(PXR_NS::SdfPath(primPath));
    if (it == snapshot->prims.end()) {
        return 0;
    }
    PXR_NS::GfRange3d range;
    if (!_AccumulateGuideBounds(it->second, &range) || range.IsEmpty()) {
        return 0;
    }
    _WriteBounds(range, outMinMax);
    return 1;
}

int
RigExecImaging_SetWeightOverlay(const char *weightPrimPath)
{
    return RigExecImagingRegistry::GetInstance().SetWeightOverlay(
               weightPrimPath ? std::string(weightPrimPath) : std::string())
        ? 0 : 1;
}

int
RigExecImaging_GetAllGuideBoundsAssetSpace(double outMinMax[6])
{
    if (!outMinMax) {
        return 0;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    if (!snapshot) {
        return 0;
    }
    PXR_NS::GfRange3d range;
    bool any = false;
    PXR_NS::SdfPath commonAssetRoot;
    for (const auto &[path, published] : snapshot->prims) {
        PXR_NS::GfRange3d publishedRange;
        if (!_AccumulateGuideBounds(published, &publishedRange)) {
            continue;
        }

        const PXR_NS::SdfPath &assetRoot = published.assetRoot.IsEmpty()
            ? snapshot->assetRoot
            : published.assetRoot;
        if (!any) {
            commonAssetRoot = assetRoot;
        } else if (commonAssetRoot != assetRoot) {
            // Asset-space bounds from independently placed assets cannot be
            // combined meaningfully.  Callers must query each prim and apply
            // that prim's asset-root transform instead.
            return 0;
        }

        range.UnionWith(publishedRange);
        any = true;
    }
    if (!any || range.IsEmpty()) {
        return 0;
    }
    _WriteBounds(range, outMinMax);
    return 1;
}

}  // extern "C"
