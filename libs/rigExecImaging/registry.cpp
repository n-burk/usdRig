//
// RigExec imaging registry and C activation surface.
//
#include <fstream>
#include "registry.h"
#include "rigExec/frameCacheSparsity.h"

#include "rigExecMath/avarScale.h"

#include "pxr/base/gf/bbox3d.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/tf/diagnostic.h"
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
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdUtils/stageCache.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

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

// The injected runner, or the production one when none was given. Null used
// to mean "skip every frame"; since the production runner factory exists it
// means "warm for real", which is what the C-API triggers run under.
RigExecFrozenStepRunner
_ProductionRunner(const RigExecFrozenStepRunner &runner)
{
    return runner ? runner : RigExecMakeProductionStepRunner();
}

// The Premonition-style sweep around \p playhead: closest-first, skipping
// the neighbor band the commit trigger already covers (+-1..N) and the
// playhead itself. 64 frames: one full commit burst (16 neighbors + 64
// sweep) fits the default per-rig in-flight cap with room to spare.
std::vector<UsdTimeCode>
_SweepTimes(UsdTimeCode playhead)
{
    std::vector<UsdTimeCode> times;
    if (playhead.IsDefault() || !std::isfinite(playhead.GetValue())) {
        return times;
    }
    const double center = playhead.GetValue();
    for (int i = kRigExecFrameCacheDefaultNeighborRadius + 1;
         i <= kRigExecFrameCacheDefaultNeighborRadius + 32; ++i) {
        times.push_back(UsdTimeCode(center - double(i)));
        times.push_back(UsdTimeCode(center + double(i)));
    }
    return times;
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
    , _scheduler(std::make_unique<RigExecBackgroundScheduler>())
    , _warmIndex(std::make_shared<RigExecWarmFrameIndex>())
{
    // The hook holds the index shared: transitions record into shared
    // state past whatever the sessions do next, and a shutdown purge
    // during destruction still lands (the hook keeps the index alive).
    std::shared_ptr<RigExecWarmFrameIndex> index = _warmIndex;
    _scheduler->SetTransitionHook(
        [index](const RigExecWarmTransition &transition) {
            index->NoteTransition(transition);
        });
}

void
RigExecImagingRegistry::RegisterChain(
    const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
    const RigExecBindingResolvingSceneIndexRefPtr &binding,
    const RigExecResultsSceneIndexRefPtr &results,
    const RigExecXformOverrideSceneIndexRefPtr &xforms)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _chains.push_back(
        {TfWeakPtr<RigExecInternalPrimPruningSceneIndex>(
             get_pointer(pruning)),
         TfWeakPtr<RigExecBindingResolvingSceneIndex>(get_pointer(binding)),
         TfWeakPtr<RigExecResultsSceneIndex>(get_pointer(results)),
         TfWeakPtr<RigExecXformOverrideSceneIndex>(get_pointer(xforms))});
    // A chain built mid-manipulation -- a second viewport opened during a drag
    // -- starts out holding the deltas the others already have, or it would
    // draw the un-previewed prim until the next mouse sample.
    if (xforms && !_previewXformDeltas.empty()) {
        xforms->SetWorldDeltas(_previewXformDeltas);
    }
    // A chain constructed after activation adopts every active rig's scope.
    if (!_generatedScopes.empty()) {
        pruning->SetOwnedScopes(_generatedScopes);
    }
}

void
RigExecImagingRegistry::_RefreshFrozenSnapshot(RigSession *session)
{
    if (!session || session->playback || !session->bridge) {
        return;
    }
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    const uint64_t epoch = evaluator.GetBindingEpochDigest();
    const uint64_t serial = evaluator.GetStageEditSerial();
    // The pins, rebound when only they moved. A constant edited mid-epoch
    // moves no digest, so the comparison is by value, not by key.
    const auto refreshPins = [&]() {
        if (session->chainBindingsValid &&
            RigExecChainSampleBindingsStillCurrent(session->chainBindings,
                                                   evaluator)) {
            return;
        }
        session->chainBindingsValid = false;
        RigExecChainSampleBindings rebound;
        if (RigExecBindChainSampleInputs(evaluator, &rebound)) {
            session->chainBindings = std::move(rebound);
            session->chainBindingsValid = true;
        }
    };
    if (session->frozenValid && session->frozenProgram == program &&
        session->frozenEpoch == epoch) {
        // Still serial, same program and epoch, valid pins: no notice
        // moved the bindings or the avar region since the standing
        // snapshot verified, so there is nothing to re-walk or
        // re-digest. (The burst pins, including overrides, compare
        // after the refresh regardless.) Failed pins still retry
        // every trigger -- only verified pins skip. During
        // notice-free playback this skips the refresh floor (~15 ms
        // on the stack) on every tick.
        if (session->frozenSerial == serial &&
            session->chainBindingsValid) {
            return;
        }
        refreshPins();
        if (!program) {
            session->frozen.reset();
            session->frozenAvarDigest = 0;
            session->frozenError.clear();
            session->frozenSerial = serial;
            return;
        }
        // Same program, same epoch: what may have moved is the avar region
        // (ApplyAvarValueEdits), the inputs a routed value edit marked
        // (ApplyValueEdits), and the program stamp. None re-clones the
        // epoch: all three carry onto a copy. The last two move no digest,
        // and the snapshot's history predates them either way, so they are
        // asked of the snapshot itself. A refused patch (a shape change
        // wearing a patch's clothes) falls through to a re-freeze.
        const uint64_t avarDigest =
            RigExecFrozenAvarRegionDigest(*program);
        if (avarDigest == session->frozenAvarDigest &&
            !(session->frozen &&
              RigExecFrozenSnapshotOwesLiveEdits(*session->frozen,
                                                 *program))) {
            session->frozenSerial = serial;
            return;
        }
        if (session->frozen) {
            std::shared_ptr<const RigExecFrozenProgram> patched;
            std::string ignored;
            if (RigExecPatchFrozenAvarConstants(*session->frozen, *program,
                                                &patched, &ignored)) {
                session->frozen = std::move(patched);
                session->frozenAvarDigest = avarDigest;
                session->frozenSerial = serial;
                return;
            }
        }
    }
    // A new program, a new epoch, or a patch that refused: re-freeze from
    // the live state. A refusal parks a validly empty entry -- production
    // jobs decline and the frame evaluates live when asked.
    session->frozen.reset();
    session->frozenProgram = program;
    session->frozenEpoch = epoch;
    session->frozenAvarDigest = 0;
    session->frozenValid = true;
    session->frozenSerial = serial;
    refreshPins();
    if (!program) {
        session->frozenError.clear();
        return;
    }
    session->frozenAvarDigest = RigExecFrozenAvarRegionDigest(*program);
    std::shared_ptr<const RigExecFrozenProgram> fresh;
    std::string frozenError;
    if (RigExecFreezeProgram(evaluator, &fresh, &frozenError)) {
        session->frozen = std::move(fresh);
        session->frozenError.clear();
    } else {
        session->frozenError = std::move(frozenError);
    }
}

RigExecBurstSampleCache *
RigExecImagingRegistry::_PrepareWarmBurst(RigSession *session)
{
    if (!session || session->playback || !session->bridge) {
        return nullptr;
    }
    RigExecProfileScope prepScope(session->bridge->MutableProfiler(),
                                  "Imaging.PrepareWarmBurst", "imaging");
    _RefreshFrozenSnapshot(session);
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        session->standingBurst.Clear();
        session->burstProgram = nullptr;
        session->burstEpochDigest = 0;
        session->burstOverrides.clear();
        session->burstOverrun = false;
        session->burstDeclined = false;
        // Session-level verdict: no factory means no per-frame verdicts,
        // so prep names the cause instead of spinning silently.
        session->lastWarmSkip = RigExecWarmSkipReason::D7Exempt;
        session->lastWarmSkipDetail = "no baked program (D7)";
        return &session->standingBurst;
    }
    const uint64_t epochDigest = RigExecFrameCacheEpochDigest(evaluator);
    const std::vector<RigExecValueOverride> overrides =
        session->bridge->GetInteractiveOverrides();
    // Pins current and the standing burst usable: serve it, with no
    // rebuild. The burst embeds its own pins copy, so the session's
    // re-verified bindings need no second check -- identical pins mean
    // identical inputs, which would build an identical burst.
    const bool pinsCurrent =
        session->burstProgram == program &&
        session->burstEpochDigest == epochDigest &&
        session->burstOverrides == overrides;
    if (pinsCurrent && session->standingBurst.usable) {
        return &session->standingBurst;
    }
    // The refresh above verified the pins, rebinding or invalidating
    // them; valid here means verified-current, and no notice can move
    // them before the burst's last frame samples. A burst is never
    // rebuilt onto unverified pins; the standing pins stay, so the next
    // trigger retries. The Clear parks neither latch flag, so the retry
    // is a rebuild, not a latched serve.
    if (!session->chainBindingsValid) {
        session->standingBurst.Clear();
        session->burstOverrun = false;
        session->burstDeclined = false;
        // Session-level verdict, as above (same words as the factory's).
        session->lastWarmSkip = RigExecWarmSkipReason::Unsampleable;
        session->lastWarmSkipDetail = "chain-bind";
        return &session->standingBurst;
    }
    // A pins-determined unusable outcome under the current pins:
    // re-serve it without paying for the doomed rebuild that produced
    // it. The latch sits past the transient check above, so a failed
    // bindings refresh never latches.
    if (pinsCurrent &&
        (session->burstOverrun || session->burstDeclined)) {
        return &session->standingBurst;
    }
    RigExecBurstSampleCache rebuilt;
    std::string buildError;
    const uint64_t buildStartUs = RigExecProfiler::NowUs();
    const bool built = RigExecBuildBurstSampleCache(
        *program, session->chainBindings, overrides, epochDigest, &rebuilt,
        &buildError);
    const double buildMs =
        double(RigExecProfiler::NowUs() - buildStartUs) / 1000.0;
    session->burstProgram = program;
    session->burstEpochDigest = epochDigest;
    session->burstOverrides = overrides;
    if (buildMs > _warmBurstPrepSliceMs) {
        // Past the slice: park unusable and mark the overrun, so this
        // tick and every tick until the pins move take the plain
        // per-frame route instead of paying for another doomed build.
        session->standingBurst.Clear();
        session->burstOverrun = true;
        session->burstDeclined = false;
    } else {
        session->standingBurst = std::move(rebuilt);
        session->standingBurst.usable =
            built && session->standingBurst.usable;
        session->burstOverrun = false;
        session->burstDeclined = !session->standingBurst.usable;
        if (session->burstDeclined) {
            // Session-level verdict, as above; the latched serve below
            // re-records nothing (this verdict stands while it latches).
            session->lastWarmSkip = RigExecWarmSkipReason::Unsampleable;
            session->lastWarmSkipDetail = "burst-declined: " + buildError;
        }
    }
    session->bridge->MutableProfiler()->RecordInstant(
        "warmBurstRebuild", "imaging", RigExecProfiler::NowUs(),
        {{"usable", session->standingBurst.usable ? "1" : "0"},
         {"overrun", session->burstOverrun ? "1" : "0"},
         {"ms", std::to_string(buildMs)}});
    return &session->standingBurst;
}

bool
RigExecImagingRegistry::GetWarmBurstUsable(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.standingBurst.usable;
        }
    }
    return false;
}

void
RigExecImagingRegistry::SetWarmBurstPrepSliceMs(double ms)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _warmBurstPrepSliceMs = ms;
    // A new slice re-times every verdict: an overrun latched under the
    // old slice unlatches (the next trigger retries the build), while a
    // usable burst stands -- the slice guards rebuild cost, not standing
    // bursts -- and a shape decline, which no slice forgives, stays.
    for (RigSession &session : _sessions) {
        session.burstOverrun = false;
    }
}

double
RigExecImagingRegistry::GetWarmBurstPrepSliceMs()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmBurstPrepSliceMs;
}

RigExecWarmSkipReason
RigExecImagingRegistry::GetLastWarmSkipReason(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.lastWarmSkip;
        }
    }
    return RigExecWarmSkipReason::None;
}

std::string
RigExecImagingRegistry::GetLastWarmSkipDetail(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.lastWarmSkipDetail;
        }
    }
    return std::string();
}

void
RigExecImagingRegistry::SetWarmSamplingBudget(size_t maxFactoryInvocations,
                                              double maxMs)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _warmSamplingMaxInvocations = maxFactoryInvocations;
    _warmSamplingMaxMs = maxMs;
}

size_t
RigExecImagingRegistry::GetWarmSamplingMaxInvocations()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmSamplingMaxInvocations;
}

double
RigExecImagingRegistry::GetWarmSamplingMaxMs()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmSamplingMaxMs;
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
                session.playback
                    ? session.playback->EvaluateAndPublishResult(time)
                    : session.bridge->EvaluateAndPublishResult(time);
            // A cache hit is not an evaluator pull: the count is what the
            // scrub tests hold to zero over a warmed range.
            if (!result.cacheHit) {
                ++session.evaluationCount;
            }
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
        // No frozen-snapshot refresh here: only warming reads the snapshot,
        // so BuildWarmWork refreshes it lazily at warm-build time. Refreshing
        // eagerly on this per-frame path made every SetTime -- evaluated or
        // cache-hit, drag tick or scrub -- pay a chain-currency walk plus an
        // avar-region digest for state nobody on this path consumes.

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
            rigEpoch->id =
                session.playback
                    ? session.playback->GetBindingEpochDigest()
                    : session.bridge->GetBindingEpochDigest();
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

        // Carried across for the same reason xformResetPaths is: the
        // combined snapshot is BUILT here, field by field, so anything
        // not named is silently dropped. That is how this one was lost
        // the first time -- the bridge recorded every pose weight, the
        // store published them, and the merge quietly left them behind,
        // so RigExecImaging_GetMovedFloats found 0 of 121 with no error
        // anywhere. Two rigs cannot publish the same property path (the
        // prim check below already rejects a shared prim), so a plain
        // insert is right.
        combined->movedFloats.insert(rigSnapshot->movedFloats.begin(),
                                     rigSnapshot->movedFloats.end());

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
    // unique_lock, not lock_guard: the broadcast at the end runs WITHOUT the
    // lock (see _Broadcast), because scene indices call back into the
    // registry -- the results index's time trigger evaluates through
    // IsActiveRigRoot/SetTime -- and broadcasting while holding it is a
    // self-deadlock. Same rule SetWeightOverlay documents, applied here.
    std::unique_lock<std::mutex> lock(_mutex);
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
    // A (re-)activation never inherits another stage's warming: the records
    // name the old stage's keys. Reset BEFORE the candidates evaluate, so
    // the initial evaluation's own memos survive; a failed activation below
    // leaves uncached records, which re-warm on demand.
    for (const SdfPath &path : rigPaths) {
        _warmIndex->ResetRig(path);
    }
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
        // Baked playback (M2b): a rig naming rigExec:asset plays the
        // file instead of evaluating. An unreadable or unplayable
        // file falls back to live evaluation with a warning -- the
        // schema contract -- rather than failing the activation.
        std::string assetPath;
        if (RigExecPlaybackAssetFor(rig, &assetPath)) {
            auto playback = std::make_unique<RigExecBakedPlayback>(
                stage, path, session.store);
            std::string why;
            if (playback->Open(assetPath, &why)) {
                playback->SetWeightOverlay(_weightOverlay);
                session.playback = std::move(playback);
                session.readRoots = {session.assetRoot};
                session.readRootsDirty = false;
            } else {
                TF_WARN("rigExec: %s names rigExec:asset %s (%s); "
                        "evaluating live",
                        path.GetString().c_str(), assetPath.c_str(),
                        why.c_str());
            }
        }
        if (!session.playback) {
            session.bridge = std::make_unique<RigExecImagingBridge>(
                stage, path, session.store);
            session.bridge->SetWarmFrameIndex(_warmIndex);
            if (std::shared_ptr<RigExecFrameCache> cache =
                    session.bridge->GetFrameCache()) {
                // Evictions retire the index's completions: the strip never
                // shows cached for a drained entry. The callback holds the
                // index shared and the rig by value, so a drop for a dead
                // session lands in a live index under a dead rig.
                std::shared_ptr<RigExecWarmFrameIndex> index = _warmIndex;
                const SdfPath rigPath = path;
                cache->SetEvictionCallback(
                    [index, rigPath](
                        const std::vector<RigExecFrameCacheEviction>
                            &evicted) {
                        for (const RigExecFrameCacheEviction &entry :
                             evicted) {
                            if (entry.time.IsNumeric()) {
                                index->NoteEvicted(rigPath, entry.key,
                                                   entry.time.GetValue());
                            }
                        }
                    });
            }
            // A selection made before this rig was activated applies to it: the
            // overlay is a viewer mode, not a property of one bridge. Set before
            // the first evaluation so the initial generation already carries it.
            session.bridge->SetWeightOverlay(_weightOverlay);
            if (!session.bridge->Compile(errors)) {
                abandon();
                return false;
            }
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
    // Fresh sessions retire every queued or running job for their rigs:
    // a completion holds its sampled cache, so this drops work, never
    // results, and a re-activation never inherits another stage's warming.
    for (const RigSession &session : _sessions) {
        // Fresh sessions retire in-flight jobs for their rigs -- but only
        // when any are in flight: an idle bump retires nothing and would
        // orphan the initial evaluation's memos, which stamped the
        // pre-commit generation.
        if (_scheduler->InFlightCount(session.rigPath) > 0) {
            _CancelGenerationLocked(session.rigPath);
        }
    }
    _stage = stage;
    {
        // Nested _mutex -> _notedMutex: the documented lock order.
        std::lock_guard<std::mutex> notedLock(_notedMutex);
        _notedRigRoots.clear();
    }
    _generatedScopes.clear();
    _assetRoots.clear();
    for (const RigSession &session : _sessions) {
        _generatedScopes.insert(
            session.playback ? session.playback->GetGeneratedScope()
                             : session.bridge->GetGeneratedScope());
        _assetRoots.insert(session.assetRoot);
    }
    _RefreshReadRoots();
    for (Chain &chain : _chains) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes(_generatedScopes);
        }
    }
    _lastTime = initialTime;
    RigExecImagingBridge::PublishResult published =
        _Publish(std::move(initialSnapshot), initialEpoch);
    // Unlocked: the commit above is complete, and _Broadcast sends scene
    // index notices whose observers call back into this registry.
    lock.unlock();
    _Broadcast(published);
    return true;
}

bool
RigExecImagingRegistry::EnsureActivated(
    const UsdStageRefPtr &stage, UsdTimeCode time)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_sessions.empty()) {
            // Already active: a no-op on this stage, a silent refusal on any
            // other (the automatic path never steals a live activation).
            return _stage == stage;
        }
    }
    // Outside the lock: Activate takes the same non-recursive mutex (the
    // rule SetWeightOverlay documents). Two rigs populating at once can both
    // reach here; the activations serialize inside Activate and the second
    // transactionally replaces the first, so the race costs a compile, not
    // correctness.
    std::vector<std::string> errors;
    if (!Activate(stage, SdfPath::EmptyPath(), time, &errors)) {
        // One line, not silent: a broken rig in a batch host (usdrecord)
        // otherwise renders its rest pose with no explanation. During live
        // authoring this fires at most once per root definition -- defining
        // the outputs that complete the rig does not resync the root, so the
        // adapter is not asked again; the host's explicit activation (or the
        // next resync of the root itself) retries.
        TF_WARN("RigExec: automatic activation failed for %s: %s",
                stage ? stage->GetRootLayer()->GetIdentifier().c_str()
                      : "<no stage>",
                errors.empty() ? "unknown error" : errors.front().c_str());
        return false;
    }
    return true;
}

bool
RigExecImagingRegistry::IsActiveRigRoot(const SdfPath &path)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == path) {
            return true;
        }
    }
    return false;
}

void
RigExecImagingRegistry::NoteRigRoot(const SdfPath &rigPath)
{
    if (rigPath.IsEmpty()) {
        return;
    }
    // _notedMutex, never _mutex: the adapter calls this from inside scene
    // index traversals that run while _mutex is held.
    std::lock_guard<std::mutex> lock(_notedMutex);
    _notedRigRoots.insert(rigPath);
}

bool
RigExecImagingRegistry::IsNotedRigRoot(const SdfPath &path)
{
    std::lock_guard<std::mutex> lock(_notedMutex);
    return _notedRigRoots.count(path) != 0;
}

bool
RigExecImagingRegistry::SetTime(UsdTimeCode time)
{
    // unique_lock: both broadcasts below run unlocked (see Activate), because
    // the notices they send re-enter this registry through the results
    // index's time trigger.
    std::unique_lock<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_stage) {
        return false;
    }
    // A publish still dirties every prim in the generation through
    // _Broadcast, even when it republishes content nothing changed -- so a
    // redundant SetTime cost a full Hydra redraw for nothing. In usdview the
    // generation was observed advancing by 2 per frame change while the
    // plugin's Python _OnFrameChanged fired once, which means a second
    // SetTime arrives from somewhere else (UpdatePreview/EndPreview,
    // SetWeightOverlay and _OnObjectsChanged all republish at _lastTime).
    // Rather than chase which, make the redundant one free: nothing dirty
    // plus the same stage/time already published means there is nothing to
    // say. Headless, generation advanced by exactly 1 per SetTime, so this
    // guard is what closes the gap the usdview session showed.
    if (!_readRootsDirty && _publishedEpochId != 0) {
        bool anyDirty = false;
        for (const RigSession &session : _sessions) {
            anyDirty = anyDirty || session.dirty || session.readRootsDirty;
        }
        if (!anyDirty) {
            const RigExecImagingSnapshotConstPtr current = _store->Get();
            if (current && current->Describes(UsdStageWeakPtr(_stage), time)) {
                _lastTime = time;
                return true;
            }
        }
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
        lock.unlock();
        _Broadcast(cleared);
        return false;
    }
    const UsdTimeCode previousTime = _lastTime;
    _lastTime = time;
    // During active playback the sweep jobs the playhead already passed are
    // dead weight: the live path serves those frames now. Neighbors stay --
    // the frames ahead still warm the imminent scrub path. Only a FORWARD
    // step sheds: a backward scrub is heading into exactly the frames at or
    // behind the new playhead, and purging those would strip the warming
    // the scrub is about to use.
    const bool advanced =
        previousTime.IsNumeric() && time.IsNumeric() &&
        time.GetValue() > previousTime.GetValue();
    if (advanced) {
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                _scheduler->NotePlaybackAdvanced(session.rigPath, time);
            }
        }
    }
    // Recorded into the first rig's profiler, so one summary holds the whole
    // update: the rig (Imaging.EvaluateAndPublish, per session, inside
    // _EvaluateSessions above) and then the COMBINED generation's publish
    // and the dirty notices that drive Hydra's sync.
    RigExecProfiler *profiler = nullptr;
    if (!_sessions.empty() && !_sessions.front().playback) {
        profiler = _sessions.front().bridge->MutableProfiler();
    }
    RigExecImagingBridge::PublishResult published;
    {
        RigExecProfileScope scope(profiler, "Imaging.CombinedPublish",
                                  "imaging");
        published = _Publish(std::move(snapshot), epoch);
    }
    lock.unlock();
    {
        RigExecProfileScope scope(profiler, "Imaging.Broadcast", "imaging");
        _Broadcast(published);
    }
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

RigExecWarmFactoryResult
RigExecImagingRegistry::BuildWarmWork(
    const SdfPath &rig, UsdTimeCode time,
    RigExecFrameGeneration generation, RigExecFrozenStepRunner runner,
    RigExecBurstSampleCache *burst)
{
    std::lock_guard<std::mutex> lock(_mutex);
    // Production-ness is read BEFORE the runner resolves: a null runner
    // means "warm for real", and real execution needs the session's
    // snapshot, while an injected test runner brings its own answers and
    // runs without one.
    const bool production = !runner;
    runner = _ProductionRunner(runner);
    if (!runner || time.IsDefault() || !std::isfinite(time.GetValue())) {
        return RigExecWarmFactoryResult();
    }
    // Non-const: a built job records its freshness proof on the bridge.
    RigSession *session = nullptr;
    for (RigSession &candidate : _sessions) {
        if (candidate.rigPath == rig) {
            session = &candidate;
            break;
        }
    }
    // Unknown rigs, playback rigs (a .rigexec never warms), and anything
    // without a bridge take no job: the frame evaluates live when asked.
    if (!session || session->playback || !session->bridge) {
        return RigExecWarmFactoryResult();
    }
    // A reasoned skip: records the session's last skip (the named cause a
    // test or the strip reads back) and returns it with the empty work.
    const auto skip = [&](RigExecWarmSkipReason reason,
                          const std::string &detail) {
        session->lastWarmSkip = reason;
        session->lastWarmSkipDetail = detail;
        // A reasoned skip is a non-publish signal toward un-warmable.
        _warmIndex->NoteSkipped(rig, time.GetValue());
        RigExecWarmFactoryResult result;
        result.skip = reason;
        result.detail = detail;
        return result;
    };
    if (!_scheduler->IsGenerationCurrent(rig, generation)) {
        return RigExecWarmFactoryResult();
    }
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    // The cached route, when the burst prepared one for this rig's
    // program: prepared pins, visit sets, and epoch digest, with static
    // reads and digests served from the burst maps. Anything else takes
    // the plain per-frame route below, which re-derives everything
    // exactly as before.
    const bool cached = burst && burst->usable &&
        evaluator.GetBakedProgram() == burst->program;
    const std::vector<RigExecValueOverride> overrides =
        session->bridge->GetInteractiveOverrides();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (program == nullptr) {
        // D7: a rig with no baked program evaluates dynamically against
        // the live stage, which a worker must never touch. No job, ever;
        // the rig still memoizes its own UI-thread results.
        return skip(RigExecWarmSkipReason::D7Exempt,
                    "no baked program (D7)");
    }
    if (!cached) {
        // Refreshed here -- lazily, at warm-build time -- and not on the
        // per-frame path: only warming reads the snapshot. The live state
        // a refresh pins is the last evaluated frame's either way (no
        // evaluation runs between that frame and this build), so warming
        // sees the same history it did under the eager refresh, including
        // after cache hits, which evaluate nothing on either path.
        _RefreshFrozenSnapshot(session);
        // The session's epoch-pinned chain bindings, verified at use and
        // rebound when a mid-epoch edit moved them. A rig that cannot bind
        // takes no job: the sampler would mark its chains stale and decline
        // anyway, but declining here skips the doomed sample.
        if (!session->chainBindingsValid ||
            !RigExecChainSampleBindingsStillCurrent(session->chainBindings,
                                                    evaluator)) {
            session->chainBindingsValid = false;
            RigExecChainSampleBindings rebound;
            if (!RigExecBindChainSampleInputs(evaluator, &rebound)) {
                return skip(RigExecWarmSkipReason::Unsampleable,
                            "chain-bind");
            }
            session->chainBindings = std::move(rebound);
            session->chainBindingsValid = true;
        }
    }
    // No snapshot, no job -- checked BEFORE sampling on both routes (the
    // cached route's refresh ran in burst prep), so a refusing rig skips
    // every sweep time without paying for doomed samples. Production
    // execution runs from the session's snapshot; an injected test runner
    // brings its own answers and runs without one.
    if (production && !session->frozen) {
        return skip(RigExecWarmSkipReason::FreezeRefused,
                    session->frozenError.empty()
                        ? "freeze refused"
                        : session->frozenError);
    }
    RigExecFrameInputs inputs;
    if (cached) {
        if (!RigExecSampleFrameInputsWithBurstCache(
                evaluator, time, overrides, burst, &inputs)) {
            return skip(RigExecWarmSkipReason::Unsampleable,
                        "burst-sample");
        }
    } else {
        // Sampled here, on the UI thread: the worker receives values, never
        // live state, stage handles, or queries. The chains evaluate through
        // the session's pins -- the same values a fresh bind would read, held
        // to account sample by sample by the equivalence test.
        if (!RigExecSampleFrameInputsWithChainBindings(
                evaluator, time, overrides, session->chainBindings,
                &inputs)) {
            return skip(RigExecWarmSkipReason::Unsampleable,
                        "chain-sample");
        }
    }
    // A declined hook (a chain binding a weight object) samples the
    // standing resolved state -- the chain outputs at whatever time the
    // evaluator last ran, not at this job's time -- and marks it viaChain.
    // No job is built from such a vector; the frame evaluates live when
    // asked and still memoizes through the live path.
    if (inputs.HasChainResolvedInputs()) {
        return skip(RigExecWarmSkipReason::Unsampleable, "chain-resolved");
    }
    if (!RigExecControlStateDigestible(inputs, overrides)) {
        return skip(RigExecWarmSkipReason::Unsampleable, "undigestible");
    }
    RigExecFrozenEvalContext context;
    context.epochDigest = evaluator.GetBindingEpochDigest();
    context.generation = generation;
    context.slotCount = program->GetProviderCount();
    context.programDigest =
        context.epochDigest ^
        (uint64_t(program->GetBoundInputCount()) * 1099511628211ull) ^
        (uint64_t(program->GetVaryingInputCount()) * 16777619ull);
    context.varyingInputCount = inputs.values.size();
    uint32_t flags = 0;
    if (evaluator.GetPublishWeightFields()) {
        flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator.GetSolverGuidesEnabled()) {
        flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.flags = flags;
    // The session's snapshot, bound per job; the existence check ran
    // before sampling above (the lock never dropped between, so the
    // session cannot have swapped it out from under this build).
    context.frozen = session->frozen.get();
    RigExecFrameCacheKey key;
    // The same epoch half the bridge's lookup computes (epoch structure +
    // build count), so a completion lands where the UI thread will look.
    // The context keeps the bare binding epoch: that is what the frozen
    // snapshot pins and cross-checks against. The cached route reads the
    // prepared halves; the digests agree by construction (the same level-1s
    // over the same served values). The constant digest folds beside the
    // sampled one on both routes (plan D3): the session's frozen avar
    // digest IS the epoch-constant digest, refreshed by
    // _RefreshFrozenSnapshot above, so a value patch opens a new key
    // namespace while the epoch half stands.
    const uint64_t constantDigest = session->frozenAvarDigest;
    uint64_t unfoldedDigest = 0;
    if (cached) {
        key.epochDigest = burst->epochDigest;
        unfoldedDigest = RigExecControlStateDigestWithBurstCache(
            inputs, overrides, burst);
        key.controlDigest =
            RigExecFoldConstantDigest(unfoldedDigest, constantDigest);
    } else {
        key.epochDigest = RigExecFrameCacheEpochDigest(evaluator);
        unfoldedDigest =
            RigExecControlStateDigest(inputs, overrides);
        key.controlDigest =
            RigExecFoldConstantDigest(unfoldedDigest, constantDigest);
    }
    // Recorded at enqueue, on this thread: a warmed entry with no proof is
    // unreachable (lookups serve only under one), and the worker cannot
    // record it -- the bridge is UI-thread state. Hook-refreshed by the
    // checks above (or chainless to begin with), so the digest names fresh
    // values and proves them.
    session->bridge->NoteWarmingEnqueued(time, key.controlDigest,
                                        unfoldedDigest, inputs);
    // Dynamic-override bootstrapping (plan 2.1): an override identity the
    // epoch index never learned would answer every plan as foreign (all
    // clusters). Admission is exact -- the seeds mirror SetOverrides'
    // placement -- and a no-op for identities already known.
    session->bridge->AdmitOverrideControls(overrides);
    std::shared_ptr<RigExecFrameCache> cache =
        session->bridge->GetFrameCache();
    RigExecBackgroundScheduler *scheduler = _scheduler.get();
    std::shared_ptr<RigExecWarmFrameIndex> warmIndex = _warmIndex;
    // Only sampled values cross the thread boundary, plus the snapshot the
    // context points at: the job's own shared_ptr keeps it alive past a
    // deactivation or a refresh that swaps the session's entry out. The
    // cache is shared (a completion for a dead session lands in a dead
    // cache); the scheduler outlives the pool (registry singleton).
    std::shared_ptr<const RigExecFrozenProgram> frozen = session->frozen;
    // Retained-state publish (plan 2.0): the source snapshot and the
    // dependency record ride with the pose. Built here, on the UI thread:
    // only values cross into the closure. The wider half -- the pose-domain
    // slots a partial cone re-runs against -- is taken from the worker's
    // thread-local after the run, inside the closure below.
    RigExecRetainedFrameState retainedState = RigExecCaptureRetainedState(
        inputs, overrides, key.epochDigest,
        program->GetStepGraph().clustering.clusters.size(),
        constantDigest);
    auto retained = std::make_shared<RigExecRetainedFrameState>(
        std::move(retainedState));
    const size_t retainedBytes = retained->RetainedBytes();
    RigExecEntryProvenance provenance =
        RigExecFullEvalProvenance(program->GetStepGraph(), time);
    provenance.unfoldedControlDigest = unfoldedDigest;
    RigExecWarmWork work = [inputs, context, key, cache, scheduler, rig,
                              runner, frozen, warmIndex, retained,
                              retainedBytes, provenance](
                                 const RigExecWarmRequest &request) {
        const RigExecRigPose pose = RigExecEvaluateFrozen(
            context, inputs, runner, scheduler, rig);
        // Stale at publish means an edit landed mid-run -- a generation
        // bump or a scoped purge that moved this time's fence token (the
        // frame requeues under the live generation); anything else that
        // fails to publish is a genuine decline (the frame evaluates live
        // when asked). The fence is re-read per failure -- never trusted
        // across the evaluate/publish calls -- so a purge between them
        // still lands DeclinedGeneration.
        const auto stale = [&]() {
            return scheduler && !scheduler->IsWarmRequestCurrent(
                                    rig, request.generation, request.time,
                                    request.fenceToken);
        };
        if (!pose.valid) {
            return stale() ? RigExecWarmOutcome::DeclinedGeneration
                           : RigExecWarmOutcome::DeclinedInvalid;
        }
        // The wider retained half (plan 2.0): the production run captured
        // its pose-domain slots into this worker's thread-local; take them
        // here, on the same worker, and publish a copy of the retained
        // state carrying both halves. An injected test kernel captures
        // nothing, so the take fails and the sources-only handle publishes
        // exactly as before.
        std::shared_ptr<const void> retainedHandle = retained;
        size_t publishBytes = retainedBytes;
        std::shared_ptr<RigExecRetainedFrameState> slotted;
        std::shared_ptr<const void> slots;
        size_t slotBytes = 0;
        if (RigExecTakeLastFrozenSlots(&slots, &slotBytes) && slots &&
            slotBytes > 0) {
            slotted =
                std::make_shared<RigExecRetainedFrameState>(*retained);
            slotted->slots = std::move(slots);
            slotted->slotBytes = slotBytes;
            publishBytes = slotted->RetainedBytes();
            retainedHandle = slotted;
        }
        const bool pubOk = RigExecPublishBackgroundCompletion(
            cache, key, request.time, pose, request.generation,
            scheduler, rig, request.fenceToken, retainedHandle,
            publishBytes, &provenance);
        if (!pubOk) {
            return stale() ? RigExecWarmOutcome::DeclinedGeneration
                           : RigExecWarmOutcome::DeclinedInvalid;
        }
        // Publish-confirmed: the frame is cached (and visited, for the
        // cursor) under the generation it ran in -- stamped explicit, so
        // a bump between the publish and this record cannot misfile it.
        warmIndex->NoteCompleted(rig, request.time.GetValue(), key,
                                 request.generation);
        return RigExecWarmOutcome::Published;
    };
    return RigExecWarmFactoryResult{std::move(work)};
}

RigExecProfiler *
RigExecImagingRegistry::MutableSchedulerProfiler()
{
    return _scheduler ? _scheduler->MutableProfiler() : nullptr;
}

RigExecProfiler *
RigExecImagingRegistry::MutableBridgeProfiler(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge) {
            return session.bridge->MutableProfiler();
        }
    }
    return nullptr;
}

std::vector<UsdTimeCode>
RigExecImagingRegistry::_CursorSweepTimes(const RigSession &session,
                                          UsdTimeCode playhead)
{
    if (!session.warmRangeActive || !session.bridge || !_scheduler ||
        !_warmIndex) {
        return _SweepTimes(playhead);
    }
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(session.rigPath);
    const uint64_t epoch =
        RigExecFrameCacheEpochDigest(session.bridge->GetEvaluator());
    const double center = playhead.GetValue();
    std::vector<std::pair<double, double>> scored;
    for (const double time : session.warmRange) {
        // The playhead is never queued, however it arises (the scheduler
        // enforces this too; skipping here keeps the vector honest).
        if (time == center) {
            continue;
        }
        if (!_warmIndex->IsVisitable(session.rigPath, time, generation,
                                     epoch)) {
            continue;
        }
        scored.emplace_back(std::abs(time - center), time);
    }
    // Closest first; ties prefer the future (playback warms ahead).
    std::sort(scored.begin(), scored.end(),
              [](const std::pair<double, double> &a,
                 const std::pair<double, double> &b) {
                  if (a.first != b.first) {
                      return a.first < b.first;
                  }
                  return a.second > b.second;
              });
    std::vector<UsdTimeCode> times;
    times.reserve(scored.size());
    for (const std::pair<double, double> &entry : scored) {
        times.emplace_back(entry.second);
    }
    return times;
}

size_t
RigExecImagingRegistry::OnEditCommitted(RigExecFrozenStepRunner runner)
{
    // Snapshot the rigs and the playhead under a short lock; the triggers
    // below sample outside it (their factories re-resolve each session, so
    // a Deactivate mid-trigger only skips frames, never dangles).
    std::vector<SdfPath> rigs;
    UsdTimeCode playhead;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_scheduler) {
            return 0;
        }
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                rigs.push_back(session.rigPath);
            }
        }
        playhead = _lastTime;
    }
    if (rigs.empty() || playhead.IsDefault() ||
        !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // The runner passes through unresolved: BuildWarmWork reads
    // production-ness off the null before resolving it to the production
    // runner itself.
    size_t enqueued = 0;
    for (const SdfPath &rig : rigs) {
        const RigExecFrameGeneration generation =
            _scheduler->CurrentGeneration(rig);
        // The session's standing burst: rebuilt when its pins moved,
        // served across ticks while current. Usable takes the cached
        // route; overrun takes the plain per-frame route (null burst),
        // since frames can still sample; shape-declined takes no
        // factory -- no frame could sample. The pointer names session
        // storage, captured for synchronous same-thread use inside the
        // trigger only (no session can activate or deactivate under it).
        RigExecBurstSampleCache *burst = nullptr;
        bool useCachedRoute = false;
        bool allowPlainRoute = false;
        std::vector<UsdTimeCode> sweepTimes;
        RigExecWarmSamplingBudget budget;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (RigSession &session : _sessions) {
                if (session.rigPath == rig && !session.playback &&
                    session.bridge) {
                    burst = _PrepareWarmBurst(&session);
                    useCachedRoute = burst && burst->usable;
                    allowPlainRoute =
                        burst && !burst->usable && session.burstOverrun;
                    sweepTimes = _CursorSweepTimes(session, playhead);
                    // Retired frames re-warm FIRST (plan 2.2/2.4): the
                    // affected-time set replaces the fixed sweep for the
                    // edit, so a release re-warms what it retired before
                    // the cursor wanders on.
                    _PrependPendingRewarm(&session, playhead, &sweepTimes);
                    budget.maxFactoryInvocations =
                        _warmSamplingMaxInvocations;
                    budget.maxMs = _warmSamplingMaxMs;
                    // Stamped after prep: the stop bounds factory sampling,
                    // not the burst prep above (frozen refresh + rebuild),
                    // which runs outside the factory budget under its own
                    // slice (plan 1.1). Measuring from the tick's start let
                    // a slow prep -- the stack's refresh floor is ~15 ms --
                    // trip the 8 ms stop before the first sample on every
                    // tick, so the rig never warmed.
                    budget.startUs = RigExecProfiler::NowUs();
                    break;
                }
            }
        }
        RigExecWarmJobFactory factory;
        if (useCachedRoute || allowPlainRoute) {
            RigExecBurstSampleCache *route = useCachedRoute ? burst : nullptr;
            factory = [this, rig, generation, runner, route](
                          UsdTimeCode time) {
                return BuildWarmWork(rig, time, generation, runner, route);
            };
        }
        enqueued += _scheduler->OnEditCommitted(
            rig, playhead, generation, std::move(sweepTimes),
            std::move(factory), kRigExecFrameCacheDefaultNeighborRadius,
            budget);
    }
    return enqueued;
}

size_t
RigExecImagingRegistry::OnIdle(RigExecFrozenStepRunner runner)
{
    std::vector<SdfPath> rigs;
    UsdTimeCode playhead;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_scheduler) {
            return 0;
        }
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                rigs.push_back(session.rigPath);
            }
        }
        playhead = _lastTime;
    }
    if (rigs.empty() || playhead.IsDefault() ||
        !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // The runner passes through unresolved: BuildWarmWork reads
    // production-ness off the null before resolving it to the production
    // runner itself.
    size_t enqueued = 0;
    for (const SdfPath &rig : rigs) {
        const RigExecFrameGeneration generation =
            _scheduler->CurrentGeneration(rig);
        // The standing burst, as in OnEditCommitted: cached route when
        // usable, plain per-frame route on overrun, no factory on a
        // shape decline.
        RigExecBurstSampleCache *burst = nullptr;
        bool useCachedRoute = false;
        bool allowPlainRoute = false;
        std::vector<UsdTimeCode> sweepTimes;
        RigExecWarmSamplingBudget budget;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (RigSession &session : _sessions) {
                if (session.rigPath == rig && !session.playback &&
                    session.bridge) {
                    burst = _PrepareWarmBurst(&session);
                    useCachedRoute = burst && burst->usable;
                    allowPlainRoute =
                        burst && !burst->usable && session.burstOverrun;
                    sweepTimes = _CursorSweepTimes(session, playhead);
                    // Retired frames first here too: without-Qt the idle
                    // tick is the only recurring driver.
                    _PrependPendingRewarm(&session, playhead, &sweepTimes);
                    budget.maxFactoryInvocations =
                        _warmSamplingMaxInvocations;
                    budget.maxMs = _warmSamplingMaxMs;
                    // Stamped after prep: the stop bounds factory sampling,
                    // not the burst prep above (frozen refresh + rebuild),
                    // which runs outside the factory budget under its own
                    // slice (plan 1.1). Measuring from the tick's start let
                    // a slow prep -- the stack's refresh floor is ~15 ms --
                    // trip the 8 ms stop before the first sample on every
                    // tick, so the rig never warmed.
                    budget.startUs = RigExecProfiler::NowUs();
                    break;
                }
            }
        }
        RigExecWarmJobFactory factory;
        if (useCachedRoute || allowPlainRoute) {
            RigExecBurstSampleCache *route = useCachedRoute ? burst : nullptr;
            factory = [this, rig, generation, runner, route](
                          UsdTimeCode time) {
                return BuildWarmWork(rig, time, generation, runner, route);
            };
        }
        enqueued += _scheduler->OnIdle(
            rig, playhead, generation, std::move(sweepTimes),
            std::move(factory), budget);
    }
    return enqueued;
}

RigExecFrameGeneration
RigExecImagingRegistry::CurrentFrameGeneration(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_scheduler) {
        return 0;
    }
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return _scheduler->CurrentGeneration(rig);
        }
    }
    return 0;
}

RigExecWarmFenceToken
RigExecImagingRegistry::CurrentFenceToken(const SdfPath &rig, UsdTimeCode time)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_scheduler) {
        return 0;
    }
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return _scheduler->CurrentFenceToken(rig, time);
        }
    }
    return 0;
}

void
RigExecImagingRegistry::_CancelGenerationLocked(const SdfPath &rig)
{
    if (_scheduler) {
        _scheduler->CancelGeneration(rig);
        _warmIndex->NoteGeneration(rig, _scheduler->CurrentGeneration(rig));
    }
}

void
RigExecImagingRegistry::_CancelGenerationTimesLocked(const SdfPath &rig)
{
    if (!_scheduler) {
        return;
    }
    // The unpartitioned affected set: completions the warm index holds
    // plus queued times the scheduler holds. No generation bump and no
    // index generation push: unaffected times keep their tokens and
    // their running jobs publish normally, while affected times get
    // fresh fence tokens and dirty flags. The partitioned path
    // (_RetireAffectedTimesLocked) narrows this by entry provenance;
    // this stays as its no-oracle fallback.
    std::vector<UsdTimeCode> times = _warmIndex->CompletedTimes(rig);
    const std::vector<UsdTimeCode> queued = _scheduler->QueuedTimes(rig);
    times.insert(times.end(), queued.begin(), queued.end());
    _scheduler->CancelGenerationTimes(rig, times);
    for (const UsdTimeCode &time : times) {
        _warmIndex->NoteDirtied(
            rig, time.IsDefault() ? 0.0 : time.GetValue());
    }
}

void
RigExecImagingRegistry::_NotePendingRewarm(RigSession *session,
                                           const std::vector<double> &times)
{
    if (!session) {
        return;
    }
    // De-duplicated against the standing list; capped past which the
    // overflow is dropped (dirtied frames stay visitable, so the cursor
    // sweep covers them when a range is set, and every frame still
    // evaluates live on visit when none is).
    static constexpr size_t kPendingRewarmCap = 4096;
    for (const double time : times) {
        if (!std::isfinite(time)) {
            continue;
        }
        if (session->pendingRewarm.size() >= kPendingRewarmCap) {
            break;
        }
        if (std::find(session->pendingRewarm.begin(),
                      session->pendingRewarm.end(), time) ==
            session->pendingRewarm.end()) {
            session->pendingRewarm.push_back(time);
        }
    }
}

void
RigExecImagingRegistry::_PrependPendingRewarm(
    RigSession *session, UsdTimeCode playhead,
    std::vector<UsdTimeCode> *sweepTimes)
{
    if (!session || !sweepTimes || session->pendingRewarm.empty() ||
        !session->bridge || !_scheduler || !_warmIndex) {
        return;
    }
    const double center =
        playhead.IsNumeric() ? playhead.GetValue() : 0.0;
    // One batched read, no sampling on the query path.
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(session->rigPath);
    const uint64_t epoch =
        RigExecFrameCacheEpochDigest(session->bridge->GetEvaluator());
    const std::vector<RigExecWarmFrameState> states = _warmIndex->States(
        session->rigPath, session->pendingRewarm, generation, epoch);
    std::vector<std::pair<double, double>> scored;
    std::vector<double> keep;
    for (size_t i = 0; i < session->pendingRewarm.size(); ++i) {
        const double time = session->pendingRewarm[i];
        const RigExecWarmFrameState state =
            i < states.size() ? states[i]
                              : RigExecWarmFrameState::Uncached;
        if (state == RigExecWarmFrameState::Cached) {
            continue;
        }
        keep.push_back(time);
        if (state == RigExecWarmFrameState::Warming) {
            continue;
        }
        scored.emplace_back(std::abs(time - center), time);
    }
    session->pendingRewarm.swap(keep);
    std::sort(scored.begin(), scored.end(),
              [](const std::pair<double, double> &a,
                 const std::pair<double, double> &b) {
                  if (a.first != b.first) {
                      return a.first < b.first;
                  }
                  return a.second > b.second;
              });
    std::vector<UsdTimeCode> first;
    first.reserve(scored.size());
    for (const auto &entry : scored) {
        // The playhead is never queued, however it arises (the scheduler
        // enforces this too; skipping here keeps the vector honest).
        if (playhead.IsNumeric() && entry.second == center) {
            continue;
        }
        first.emplace_back(entry.second);
    }
    sweepTimes->insert(sweepTimes->begin(), first.begin(), first.end());
}

void
RigExecImagingRegistry::_RetireAffectedTimesLocked(
    RigSession *session, const UsdNotice::ObjectsChanged &notice,
    RigExecNoticeDisposition disposition,
    const std::vector<SdfPath> &patchedPaths)
{
    if (!session || !session->bridge || session->playback || !_scheduler ||
        !_warmIndex) {
        return;
    }
    const SdfPath &rig = session->rigPath;
    RigExecImagingBridge *bridge = session->bridge.get();
    const RigExecBakedProgram *program =
        bridge->GetEvaluator().GetBakedProgram();
    const RigExecOutputAffectedIndex *index = bridge->GetAffectedIndex();
    if (!program || !index || index->Empty()) {
        // No oracle: every completed time is affected, as before. Queued
        // times purge (their provenance publishes later); proofs retire
        // by the notice's paths.
        _CancelGenerationTimesLocked(rig);
        const std::vector<UsdTimeCode> completed =
            _warmIndex->CompletedTimes(rig);
        std::vector<double> rewarm;
        for (const UsdTimeCode &time : completed) {
            rewarm.push_back(time.IsDefault() ? 0.0 : time.GetValue());
        }
        _NotePendingRewarm(session, rewarm);
        bridge->RetireProofsForControls(RigExecNoticeControlIds(notice));
        return;
    }
    // The edit's controls: the evaluator's exact patched set for a
    // patch (no notice noise -- a Patched notice carries nothing but
    // patchable values and their ancestors), the full adapter for a
    // stamp bump. A routed edit names only the properties the program
    // reads, so a property nothing reads retires nothing; each named one
    // stands for itself where the index knows it -- an overridable input,
    // seeded as an override on it is -- and brings its prim along where
    // it does not, which is the adapter's own answer for it.
    std::vector<RigExecControlId> controls;
    if (disposition == RigExecNoticeDisposition::Edited) {
        std::set<RigExecControlId> ids;
        for (const SdfPath &path : patchedPaths) {
            if (path.IsEmpty()) {
                continue;
            }
            const RigExecControlId id = RigExecControlIdForPath(path);
            ids.insert(id);
            if (path.IsPropertyPath() && !index->IsKnownControl(id)) {
                ids.insert(RigExecControlIdForPath(path.GetPrimPath()));
            }
        }
        controls.assign(ids.begin(), ids.end());
    } else if (disposition == RigExecNoticeDisposition::Patched) {
        std::set<RigExecControlId> ids;
        for (const SdfPath &path : patchedPaths) {
            if (path.IsEmpty()) {
                continue;
            }
            ids.insert(path.GetString());
            if (path.IsPropertyPath()) {
                ids.insert(path.GetPrimPath().GetString());
            }
        }
        controls.assign(ids.begin(), ids.end());
    } else {
        controls = RigExecNoticeControlIds(notice);
    }
    const RigExecBakedClusterSet dirty =
        index->AffectedByControls(controls);
    // Weight-object edits: a notice path at or under a weight object an
    // entry read (WeightPacket steps resolve through step.object, outside
    // the avar-only index, so the provenance read set is the oracle).
    std::vector<SdfPath> noticePaths;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        noticePaths.push_back(path);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        noticePaths.push_back(path);
        if (path.IsPropertyPath()) {
            noticePaths.push_back(path.GetPrimPath());
        }
    }
    for (const SdfPath &path :
         notice.GetResolvedAssetPathsResyncedPaths()) {
        noticePaths.push_back(path);
    }
    const uint64_t newConstants = RigExecEpochConstantDigest(*program);
    const std::shared_ptr<RigExecFrameCache> cache =
        bridge->GetFrameCache();
    const std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>> done =
        _warmIndex->CompletedKeys(rig);
    std::vector<UsdTimeCode> purge;
    std::vector<double> rewarm;
    for (const auto &entry : done) {
        const UsdTimeCode time = entry.first;
        const RigExecFrameCacheKey &oldKey = entry.second;
        const double stamped =
            time.IsDefault() ? 0.0 : time.GetValue();
        RigExecEntryProvenance provenance;
        bool affected = true;
        if (cache && cache->LookupProvenance(oldKey, &provenance)) {
            // The affected test is the executor's own dirtiness rule
            // (bakedSchedule §7): an entry's pose moved exactly when a
            // dirty cluster reaches its computation, or an edited weight
            // object feeds it. Constant regions are recorded in the
            // provenance but deliberately NOT consulted: the patchable
            // binding's compose-cluster cone is the exact oracle (a
            // seedless binding dirties nothing and carries), while
            // regions over-approximate by stride.
            affected = false;
            for (const int cluster : provenance.clusters) {
                // Bounded by hand: ClusterSet::Test guards negative
                // only, and provenance is publisher-controlled.
                if (cluster >= 0 &&
                    size_t(cluster) < dirty.words.size() * 64 &&
                    dirty.Test(cluster)) {
                    affected = true;
                    break;
                }
            }
            for (size_t i = 0; !affected && i < provenance.weightReads.size();
                 ++i) {
                const SdfPath weight(provenance.weightReads[i]);
                if (weight.IsEmpty()) {
                    continue;
                }
                for (const SdfPath &path : noticePaths) {
                    if (path == weight || path.HasPrefix(weight)) {
                        affected = true;
                        break;
                    }
                }
            }
        }
        if (!affected) {
            // Lane a/b: the entry's computation touches nothing dirty.
            // Across a namespace move it carries (re-keyed, re-pointed,
            // zero recompute); otherwise it stands untouched -- no purge,
            // no dirty flag, no requeue. A failed carry degrades to
            // lane c rather than stranding the frame.
            if (cache && bridge->CarryEntry(time, oldKey, newConstants)) {
                // The new key holds the row now: old-key jobs queued or
                // running for this time must drop, not resurrect it.
                purge.push_back(time);
                continue;
            }
            if (cache) {
                RigExecEntryProvenance carried;
                if (cache->LookupProvenance(oldKey, &carried) &&
                    carried.unfoldedControlDigest != 0 &&
                    RigExecFoldConstantDigest(
                        carried.unfoldedControlDigest, newConstants) !=
                        oldKey.controlDigest) {
                    // A namespace move the carry declined (evicted base,
                    // pose-only entry, over-cap re-publish): retire and
                    // re-warm rather than serve stale.
                    purge.push_back(time);
                    rewarm.push_back(stamped);
                    _warmIndex->NoteDirtied(rig, stamped);
                    continue;
                }
            }
            continue;
        }
        // Lane c: retire the time. The stale entry strands under its old
        // key (unreachable by construction -- the digest backstop -- and
        // drained by LRU); the row goes dirty and the frame requeues for
        // a full re-warm. (Cone-granular re-execution waits on slot
        // capture: sources-only retained state plans but cannot run a
        // Partial, so the planner's verdict here would only restate this
        // partition at higher cost.)
        purge.push_back(time);
        rewarm.push_back(stamped);
        _warmIndex->NoteDirtied(rig, stamped);
    }
    // Queued times are always affected: their provenance publishes with
    // their completion, so nothing partitions them yet.
    const std::vector<UsdTimeCode> queued = _scheduler->QueuedTimes(rig);
    purge.insert(purge.end(), queued.begin(), queued.end());
    _scheduler->CancelGenerationTimes(rig, purge);
    _NotePendingRewarm(session, rewarm);
    bridge->RetireProofsForControls(controls);
}

void
RigExecImagingRegistry::CancelFrameGeneration(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _CancelGenerationLocked(rig);
}

RigExecBackgroundSchedulerStats
RigExecImagingRegistry::GetBackgroundStats()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_scheduler) {
        return RigExecBackgroundSchedulerStats();
    }
    return _scheduler->Stats();
}

void
RigExecImagingRegistry::WaitUntilBackgroundIdle()
{
    RigExecBackgroundScheduler *scheduler = nullptr;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler.get();
    }
    // Outside _mutex: workers never take it, but idleness can outlast a
    // caller's patience for holding it.
    if (scheduler) {
        scheduler->WaitUntilIdle();
    }
}

RigExecFrameCacheStats
RigExecImagingRegistry::GetFrameCacheStats(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig && session.bridge) {
            return session.bridge->GetFrameCacheStats();
        }
    }
    return RigExecFrameCacheStats();
}

void
RigExecImagingRegistry::ClearFrameCache(const SdfPath &rig)
{
    // The fence is outermost: read the scheduler under a short lock, then
    // hold the fence across the whole cancel-THEN-clear below. Lock order
    // fence, then registry, then scheduler, then cache shards, then the
    // warm index -- never inverted.
    RigExecBackgroundScheduler *scheduler = nullptr;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler.get();
    }
    std::unique_lock<std::mutex> fenceLock;
    if (scheduler) {
        fenceLock = std::unique_lock<std::mutex>(scheduler->FenceMutex());
    }
    // Cancel in its own scope BEFORE locking for the clear: the registry
    // mutex is non-recursive, so cancel-under-clear-lock self-deadlocks.
    CancelFrameGeneration(rig);
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig && session.bridge) {
                session.bridge->ClearFrameCache();
            }
        }
        _warmIndex->ResetRig(rig);
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig) {
                session.warmRangeActive = false;
                session.warmRange.clear();
            }
        }
    }
}

bool
RigExecImagingRegistry::SetWarmRange(const SdfPath &rig,
                                     const std::vector<double> &frames)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge) {
            session.warmRange.clear();
            for (const double time : frames) {
                if (std::isfinite(time)) {
                    session.warmRange.push_back(time);
                }
            }
            std::sort(session.warmRange.begin(), session.warmRange.end());
            session.warmRange.erase(
                std::unique(session.warmRange.begin(), session.warmRange.end()),
                session.warmRange.end());
            session.warmRangeActive = true;
            return true;
        }
    }
    return false;
}

std::vector<RigExecWarmFrameState>
RigExecImagingRegistry::GetFrameStates(const SdfPath &rig,
                                       const std::vector<double> &frames)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (frames.empty() || frames.size() > 100000) {
        return {};
    }
    const RigSession *session = nullptr;
    for (const RigSession &candidate : _sessions) {
        if (candidate.rigPath == rig && !candidate.playback &&
            candidate.bridge) {
            session = &candidate;
            break;
        }
    }
    if (!session || !_scheduler || !_warmIndex) {
        return {};
    }
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(rig);
    const uint64_t epoch =
        RigExecFrameCacheEpochDigest(session->bridge->GetEvaluator());
    return _warmIndex->States(rig, frames, generation, epoch);
}

RigExecImagingBridge *
RigExecImagingRegistry::GetBridge(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge) {
            return session.bridge.get();
        }
    }
    return nullptr;
}

std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>>
RigExecImagingRegistry::GetCompletedKeys(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge &&
            _warmIndex) {
            return _warmIndex->CompletedKeys(rig);
        }
    }
    return {};
}

bool
RigExecImagingRegistry::SetWeightOverlay(const std::string &weightPrimPath)
{
    UsdTimeCode time;
    // Fenced cancel-THEN-clear like ClearFrameCache (plan 3.3): the fence
    // is outermost, so it is taken before the registry lock below and held
    // across every session's cancel plus clear.
    RigExecBackgroundScheduler *scheduler = nullptr;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler.get();
    }
    std::unique_lock<std::mutex> fenceLock;
    if (scheduler) {
        fenceLock = std::unique_lock<std::mutex>(scheduler->FenceMutex());
    }
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
            if (session.playback) {
                session.playback->SetWeightOverlay(resolved);
            } else {
                // Cancel BEFORE the bridge clears (plan 3.3): the overlay
                // flag rides to workers in context.flags and changes pose
                // content without moving the key, so the old clear-then-
                // cancel order let a worker publish a stale-overlay pose
                // into the cleared cache between the two. Retired jobs
                // re-enqueue fresh on the next commit.
                _CancelGenerationLocked(session.rigPath);
                session.bridge->SetWeightOverlay(resolved);
            }
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

// ---------------------------------------------------------------------------
// Manipulation preview (docs/superpowers/specs/
// 2026-09-10-hydra-preview-manipulation-design.md)
// ---------------------------------------------------------------------------

namespace {

// How many doubles a value of this type takes, and the token naming how to
// rebuild it. An unsupported type returns an empty token: a manipulation
// cannot preview a value it cannot reconstruct, and saying so at declaration
// time is better than guessing per sample.
TfToken
_PreviewValueKind(const SdfValueTypeName &type, size_t *arity)
{
    static const TfToken kDouble("double");
    static const TfToken kFloat("float");
    static const TfToken kVec3d("vec3d");
    static const TfToken kVec3f("vec3f");
    static const TfToken kMatrix4d("matrix4d");
    if (type == SdfValueTypeNames->Double) {
        *arity = 1;
        return kDouble;
    }
    if (type == SdfValueTypeNames->Float) {
        *arity = 1;
        return kFloat;
    }
    if (type == SdfValueTypeNames->Double3 ||
        type == SdfValueTypeNames->Vector3d ||
        type == SdfValueTypeNames->Point3d ||
        type == SdfValueTypeNames->Normal3d) {
        *arity = 3;
        return kVec3d;
    }
    if (type == SdfValueTypeNames->Float3 ||
        type == SdfValueTypeNames->Vector3f ||
        type == SdfValueTypeNames->Point3f ||
        type == SdfValueTypeNames->Normal3f) {
        *arity = 3;
        return kVec3f;
    }
    if (type == SdfValueTypeNames->Matrix4d) {
        *arity = 16;
        return kMatrix4d;
    }
    *arity = 0;
    return TfToken();
}

VtValue
_PreviewValue(const TfToken &kind, const double *v)
{
    if (kind == "double") {
        return VtValue(v[0]);
    }
    if (kind == "float") {
        return VtValue(float(v[0]));
    }
    if (kind == "vec3d") {
        return VtValue(GfVec3d(v[0], v[1], v[2]));
    }
    if (kind == "vec3f") {
        return VtValue(GfVec3f(float(v[0]), float(v[1]), float(v[2])));
    }
    if (kind == "matrix4d") {
        return VtValue(GfMatrix4d(
            v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
            v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15]));
    }
    return VtValue();
}

std::vector<std::string>
_SplitLines(const std::string &packed)
{
    std::vector<std::string> out;
    std::string current;
    for (const char c : packed) {
        if (c == '\n' || c == '\r') {
            if (!current.empty()) {
                out.push_back(current);
                current.clear();
            }
            continue;
        }
        current.push_back(c);
    }
    if (!current.empty()) {
        out.push_back(current);
    }
    return out;
}

}  // namespace

int
RigExecImagingRegistry::BeginPreview(const std::string &packedAttributePaths)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_stage) {
        return -1;
    }
    std::vector<PreviewSlot> slots;
    size_t total = 0;
    for (const std::string &text : _SplitLines(packedAttributePaths)) {
        // Caller-supplied strings: ask before constructing, the same guard
        // SetWeightOverlay uses.
        if (!SdfPath::IsValidPathString(text)) {
            return -1;
        }
        const SdfPath path(text);
        if (!path.IsAbsolutePath() || !path.IsPropertyPath()) {
            return -1;
        }
        const UsdAttribute attribute = _stage->GetAttributeAtPath(path);
        if (!attribute) {
            return -1;
        }
        PreviewSlot slot;
        slot.primPath = path.GetPrimPath();
        slot.attributeName = path.GetNameToken();
        slot.valueKind = _PreviewValueKind(
            attribute.GetTypeName(), &slot.arity);
        if (slot.valueKind.IsEmpty()) {
            return -1;
        }
        // Which lane. An attribute inside an active rig previews through that
        // rig's evaluator; everything else is a transform of its own.
        // A playback rig offers no evaluator lane: its inputs are baked,
        // so an attribute under one falls through to the xform lane (or
        // is refused, when it is not an xform op either).
        for (const RigSession &session : _sessions) {
            if (!session.playback &&
                slot.primPath.HasPrefix(session.rigPath)) {
                slot.rigPath = session.rigPath;
                break;
            }
        }
        if (slot.rigPath.IsEmpty() &&
            !UsdGeomXformOp::IsXformOp(slot.attributeName)) {
            // Nothing to preview: the prim has no rig to re-run and this is
            // not part of its transform. Refused rather than silently
            // ignored, because the caller would then be sending values that
            // change nothing and wondering why.
            return -1;
        }
        total += slot.arity;
        slots.push_back(std::move(slot));
    }
    _previewSlots = std::move(slots);
    _previewActive = true;
    return int(total);
}

bool
RigExecImagingRegistry::IsPreviewActive() const
{
    return _previewActive;
}

bool
RigExecImagingRegistry::_ComposeXformDelta(
    const UsdPrim &prim,
    const std::map<TfToken, VtValue> &opValues,
    UsdGeomXformCache *cache,
    GfMatrix4d *delta) const
{
    const UsdGeomXformable xformable(prim);
    if (!xformable || !cache || !delta) {
        return false;
    }
    bool resetsXformStack = false;
    const std::vector<UsdGeomXformOp> ops =
        xformable.GetOrderedXformOps(&resetsXformStack);

    // The authored local transform and the previewed one, composed the same
    // way out of the same ops, so the only difference between them is the
    // overridden values. Composing op by op rather than calling
    // GetLocalTransformation twice is what lets an override stand in for one
    // op's value without being authored anywhere.
    GfMatrix4d authored(1.0), previewed(1.0);
    const UsdTimeCode time = cache->GetTime();
    for (const UsdGeomXformOp &op : ops) {
        VtValue value;
        if (!op.GetAttr().Get(&value, time)) {
            // An op with no value at this time contributes its identity to
            // both, which is what USD itself does with it.
            continue;
        }
        authored = op.GetOpTransform(op.GetOpType(), value, op.IsInverseOp()) *
                   authored;
        const auto found = opValues.find(op.GetOpName());
        const VtValue &previewValue =
            found == opValues.end() ? value : found->second;
        previewed =
            op.GetOpTransform(op.GetOpType(), previewValue, op.IsInverseOp()) *
            previewed;
    }

    // delta = parentToWorld^-1 . local^-1 . local' . parentToWorld
    //
    // which is exactly xform^-1 . xform', the quantity
    // RigExecXformOverrideSceneIndex post-multiplies. A prim that resets the
    // xform stack has no parent contribution, and then the conjugation
    // collapses to local^-1 . local' on its own.
    const GfMatrix4d parentToWorld = resetsXformStack
        ? GfMatrix4d(1.0)
        : cache->GetParentToWorldTransform(prim);
    const GfMatrix4d parentInverse = parentToWorld.GetInverse();
    *delta = parentInverse * authored.GetInverse() * previewed * parentToWorld;
    return true;
}

bool
RigExecImagingRegistry::_ResolvePreviewSample(
    const double *values, size_t count,
    std::map<SdfPath, std::vector<RigExecValueOverride>> *byRig,
    std::map<SdfPath, GfMatrix4d> *xformDeltas) const
{
    size_t offset = 0;
    std::map<SdfPath, std::map<TfToken, VtValue>> xformOps;
    for (const PreviewSlot &slot : _previewSlots) {
        if (offset + slot.arity > count) {
            return false;
        }
        const VtValue value =
            _PreviewValue(slot.valueKind, values + offset);
        offset += slot.arity;
        if (value.IsEmpty()) {
            return false;
        }
        if (!slot.rigPath.IsEmpty()) {
            (*byRig)[slot.rigPath].push_back(RigExecValueOverride{
                slot.primPath, TfToken(), slot.attributeName, value});
        } else {
            xformOps[slot.primPath][slot.attributeName] = value;
        }
    }
    if (offset != count) {
        return false;
    }
    if (!xformOps.empty()) {
        UsdGeomXformCache cache(_lastTime);
        for (const auto &[primPath, opValues] : xformOps) {
            const UsdPrim prim = _stage->GetPrimAtPath(primPath);
            GfMatrix4d delta(1.0);
            if (!prim ||
                !_ComposeXformDelta(prim, opValues, &cache, &delta)) {
                return false;
            }
            (*xformDeltas)[primPath] = delta;
        }
    }
    return true;
}

void
RigExecImagingRegistry::_SetChainXformDeltas(
    const std::map<SdfPath, GfMatrix4d> &deltas)
{
    _previewXformDeltas = deltas;
    for (auto it = _chains.begin(); it != _chains.end();) {
        if (!it->results) {
            it = _chains.erase(it);
            continue;
        }
        if (it->xforms) {
            it->xforms->SetWorldDeltas(deltas);
        }
        ++it;
    }
}

bool
RigExecImagingRegistry::UpdatePreview(const double *values, size_t count)
{
    UsdTimeCode time;
    bool needsRigPublish = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_previewActive || !_stage || (!values && count)) {
            return false;
        }
        std::map<SdfPath, std::vector<RigExecValueOverride>> byRig;
        std::map<SdfPath, GfMatrix4d> xformDeltas;
        if (!_ResolvePreviewSample(values, count, &byRig, &xformDeltas)) {
            return false;
        }
        // The xform lane needs no evaluation at all: the delta goes straight
        // into the chains and the dirty notice it sends is the whole update.
        _SetChainXformDeltas(xformDeltas);

        // Every active rig is told, including the ones with no slots this
        // sample: a rig that was being previewed and no longer is has to stop.
        for (RigSession &session : _sessions) {
            const auto found = byRig.find(session.rigPath);
            if (found == byRig.end()) {
                if (session.bridge) {
                    session.bridge->ClearInteractiveOverrides();
                }
            } else if (session.bridge) {
                session.bridge->SetInteractiveOverrides(found->second);
                needsRigPublish = true;
            }
            // D5: any new edit bumps the generation and cancels the rig's
            // in-flight jobs. A job sampled before this tick would land
            // under an unreachable key anyway (the digest folds the
            // overrides), but it would still burn a worker on a pose no
            // lookup can reach; the commit trigger re-enqueues fresh.
            _CancelGenerationLocked(session.rigPath);
            session.dirty = true;
        }
        time = _lastTime;
    }
    // Republished OUTSIDE the lock: SetTime takes the same non-recursive mutex
    // (the rule _OnObjectsChanged and SetWeightOverlay both follow).
    //
    // Only when a rig is involved. An xform-lane drag has already dirtied what
    // it changed, and evaluating every active rig to redraw a prim no rig
    // drives would make the cheap lane pay for the expensive one.
    return needsRigPublish ? SetTime(time) : true;
}

bool
RigExecImagingRegistry::WriteProfileSummary(const std::string &path)
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "rig\tname\tcategory\tcount\ttotal_ms\tmax_ms\tmode\n";
    for (const RigSession &session : _sessions) {
        // Playback records no phases, so it contributes no rows.
        if (session.playback) {
            continue;
        }
        const RigExecRigEvaluator &evaluator = session.bridge->GetEvaluator();
        // The mode ASKED for. Whether the baked program actually answered is
        // in the rows themselves: a generation it served records `Baked`.
        const RigExecEvaluationMode requested = evaluator.GetEvaluationMode();
        const std::string mode =
            requested == RigExecEvaluationMode::Dynamic ? "dynamic"
            : requested == RigExecEvaluationMode::Baked ? "baked"
            : requested == RigExecEvaluationMode::ExecReference
                ? "reference"
                : "parity";
        for (const RigExecProfileSummaryRow &row :
             session.bridge->GetProfiler().Summarize()) {
            out << session.rigPath.GetString() << '\t' << row.name << '\t'
                << row.category << '\t' << row.count << '\t'
                << (row.totalUs / 1000.0) << '\t' << (row.maxUs / 1000.0)
                << '\t' << mode << '\n';
        }
        session.bridge->MutableProfiler()->Clear();
    }
    return true;
}

bool
RigExecImagingRegistry::EndPreview()
{
    UsdTimeCode time;
    bool hadRigOverrides = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _previewSlots.clear();
        _previewActive = false;
        _SetChainXformDeltas({});
        for (RigSession &session : _sessions) {
            if (session.bridge) {
                session.bridge->ClearInteractiveOverrides();
            }
            session.dirty = true;
            hadRigOverrides = true;
        }
        time = _lastTime;
    }
    // Republished so the authored rig is what is drawn again. The caller
    // authors its committed values separately; when it has already done so,
    // this generation is the committed one and the artist sees no flicker,
    // and when it has not -- an aborted drag -- this is the rig going back.
    return hadRigOverrides ? SetTime(time) : true;
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
    if (!relevant) {
        // Resolved-asset resyncs: the evaluator honors them (they break
        // the avar-only fast path and hit the capture index), so the
        // registry scan reads them too.
        for (const SdfPath &path :
             notice.GetResolvedAssetPathsResyncedPaths()) {
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
            //
            // A property resync that names only a value (a spec created by
            // its first authored value, or removed by the undo of it) is a
            // value edit here too: a new connection or target would arrive
            // as its own field, and a removed one leaves the cached regions
            // a superset of what is read, which is safe. The one input a
            // removal can reveal is a weaker layer's connection the removed
            // spec was hiding, so a property still connected afterwards
            // counts as structure.
            for (const SdfPath &path : notice.GetResyncedPaths()) {
                if (_readRootsDirty) {
                    break;
                }
                if (!path.IsPropertyPath()) {
                    _readRootsDirty = true;
                    break;
                }
                const UsdAttribute attribute =
                    _stage ? _stage->GetAttributeAtPath(path) : UsdAttribute();
                const UsdRelationship relationship =
                    _stage ? _stage->GetRelationshipAtPath(path)
                           : UsdRelationship();
                if ((attribute && attribute.HasAuthoredConnections()) ||
                    (relationship && relationship.HasAuthoredTargets())) {
                    _readRootsDirty = true;
                    break;
                }
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field != "typeName" && field != "default" &&
                        field != "timeSamples" && field != "spline") {
                        _readRootsDirty = true;
                        break;
                    }
                }
            }
            for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field == "connectionPaths" || field == "targetPaths") {
                        _readRootsDirty = true;
                    }
                }
            }
            // A resolved asset arriving from a new file can reveal a new
            // external input the cached regions never named.
            if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
                _readRootsDirty = true;
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
                for (const SdfPath &path :
                     notice.GetResolvedAssetPathsResyncedPaths())
                    affected = affected || affects(path);
                // A playback session never dirties: the binary is static,
                // so no stage edit changes what it publishes. Picking up
                // a new asset (or a new file behind it) needs a
                // re-activation.
                if (affected && !session.playback) {
                    session.dirty = true;
                    session.readRootsDirty = session.readRootsDirty || _readRootsDirty;
                    // The bridge caches authored guide styling (shape, scale,
                    // colour, purpose...) across generations; an edit that
                    // reaches the rig is the only thing that can move it.
                    session.bridge->InvalidateGuideCaches();
                    // Outcome-driven retirement (plan 2.1/2.2): the
                    // evaluator classifies this notice (patched plus
                    // paths, routed plus the paths it reads, stamp-bumped,
                    // or stale) through a read-only query, so the verdict
                    // holds whatever order the two notice handlers ran in.
                    // Patch/routed/stamp edits partition
                    // completions by entry provenance -- clean entries
                    // stand or carry, dirty ones retire -- and purge
                    // affected times only, without bumping the
                    // generation, so other times' queued and running jobs
                    // still publish; a stale program (or no program)
                    // keeps the global cancel plus the capture-index
                    // epoch retirement.
                    std::vector<SdfPath> patchedPaths;
                    const RigExecNoticeDisposition disposition =
                        session.bridge->GetEvaluator()
                            .ClassifyNoticeDisposition(notice,
                                                       &patchedPaths);
                    if (disposition == RigExecNoticeDisposition::Patched ||
                        disposition == RigExecNoticeDisposition::Edited ||
                        disposition ==
                            RigExecNoticeDisposition::StampBumped) {
                        _RetireAffectedTimesLocked(&session, notice,
                                                   disposition,
                                                   patchedPaths);
                    } else {
                        // A new edit bumps the warming generation and
                        // cancels in-flight jobs for the rig; the commit
                        // trigger re-enqueues fresh jobs under the new
                        // token.
                        _CancelGenerationLocked(session.rigPath);
                        // The baked capture index retires cached frames
                        // eagerly: a notice that hits it drops the epoch
                        // now, while its half still names it (the
                        // evaluator rebuilds lazily); a miss drops
                        // nothing and D1 reachability decides.
                        session.bridge->NoteCaptureIndex(notice);
                    }
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
    // unique_lock: the broadcast below runs unlocked (see Activate), because
    // the notices it sends re-enter this registry through the results
    // index's time trigger.
    std::unique_lock<std::mutex> lock(_mutex);
    TfNotice::Revoke(_changeKey);
    _changeKey = TfNotice::Key();
    _assetRoots.clear();
    _readRoots.clear();
    _readRootsDirty = false;
    _generatedScopes.clear();
    for (const RigSession &session : _sessions) {
        _warmIndex->ResetRig(session.rigPath);
        if (_scheduler) {
            _scheduler->ResetRig(session.rigPath);
        }
    }
    _sessions.clear();
    _stage.Reset();
    {
        // Nested _mutex -> _notedMutex: the documented lock order.
        std::lock_guard<std::mutex> notedLock(_notedMutex);
        _notedRigRoots.clear();
    }
    _publishedEpochId = 0;
    for (Chain &chain : _chains) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes({});
        }
    }
    RigExecImagingBridge::PublishResult cleared;
    cleared.ok = true;
    cleared.dirtied = _store->Publish(nullptr);
    lock.unlock();
    _Broadcast(cleared);
}

void
RigExecImagingRegistry::_Broadcast(
    const RigExecImagingBridge::PublishResult &result)
{
    if (!result.ok) {
        return;
    }
    // The chain list is snapshotted under a short lock; the sends go out
    // WITHOUT it. Scene indices call back into this registry -- the results
    // index's time trigger evaluates through IsActiveRigRoot/SetTime -- so
    // every caller unlocks first (see Activate), and this function never
    // assumes the lock is held. A chain whose graph dies between the snapshot
    // and its send simply fails its weak-pointer check and is pruned on the
    // next broadcast.
    std::vector<Chain> chains;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // Prune chains whose scene index graphs were destroyed.
        _chains.erase(
            std::remove_if(_chains.begin(), _chains.end(),
                           [](const Chain &c) { return !c.results; }),
            _chains.end());
        chains = _chains;
    }
    for (Chain &chain : chains) {
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
// the one whose Python we could reach. UsdGeomBBoxCache is what usdview
// and Solaris both consult, and it asks a Boundable for its extent;
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
                             "RigExecRibbon",
                             "RigExecSplineIk"}) {
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

int
RigExecImaging_OnEditCommitted()
{
    RigExecImagingRegistry &registry =
        RigExecImagingRegistry::GetInstance();
    if (!registry.IsActive()) {
        return 1;
    }
    registry.OnEditCommitted();
    return 0;
}

int
RigExecImaging_OnIdle()
{
    RigExecImagingRegistry &registry =
        RigExecImagingRegistry::GetInstance();
    if (!registry.IsActive()) {
        return 1;
    }
    registry.OnIdle();
    return 0;
}

long long
RigExecImaging_GetGeneration()
{
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    return snapshot ? static_cast<long long>(snapshot->generation) : 0;
}

int
RigExecImaging_GetMovedFloats(
    const char *packedPaths, float *out, int count)
{
    if (!packedPaths || !out || count <= 0) {
        return 0;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    if (!snapshot) {
        return 0;
    }
    // Every slot is cleared first: a caller that reads past the return
    // value gets zeros rather than whatever was in its buffer.
    for (int i = 0; i < count; ++i) {
        out[i] = 0.0f;
    }
    int found = 0;
    int index = 0;
    const char *cursor = packedPaths;
    while (*cursor && index < count) {
        const char *end = std::strchr(cursor, '\n');
        const std::string text(cursor, end ? end - cursor
                                           : std::strlen(cursor));
        // An arbitrary caller-supplied string reaches SdfPath here, whose
        // constructor is loud about a malformed one. Ask first.
        if (!text.empty() && PXR_NS::SdfPath::IsValidPathString(text)) {
            const auto it = snapshot->movedFloats.find(PXR_NS::SdfPath(text));
            if (it != snapshot->movedFloats.end()) {
                out[index] = it->second;
                ++found;
            }
        }
        ++index;
        if (!end) {
            break;
        }
        cursor = end + 1;
    }
    return found;
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

// Viewport profiling. With RIGEXEC_IMAGING_PROFILE set, every active rig's
// evaluator records its own phases and the imaging layer's publish phases;
// this writes the per-phase totals as tab-separated text and clears them,
// so a caller can bracket exactly the interaction it wants to measure.
int
RigExecImaging_WriteProfileSummary(const char *path)
{
    return RigExecImagingRegistry::GetInstance().WriteProfileSummary(
               path ? std::string(path) : std::string())
        ? 0 : 1;
}

int
RigExecImaging_SetWeightOverlay(const char *weightPrimPath)
{
    return RigExecImagingRegistry::GetInstance().SetWeightOverlay(
               weightPrimPath ? std::string(weightPrimPath) : std::string())
        ? 0 : 1;
}

int
RigExecImaging_GetFrameStates(const char *rigPath, const double *frames,
                              int *statesOut, int count)
{
    if (!rigPath || !frames || !statesOut || count < 0) {
        return -1;
    }
    if (!SdfPath::IsValidPathString(rigPath)) {
        return -1;
    }
    std::vector<double> times;
    times.reserve(size_t(count));
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(frames[i])) {
            return -1;
        }
        times.push_back(frames[i]);
    }
    const std::vector<rigExec::RigExecWarmFrameState> states =
        RigExecImagingRegistry::GetInstance().GetFrameStates(
            SdfPath(rigPath), times);
    if (states.empty() && count > 0) {
        return -1;
    }
    for (size_t i = 0; i < states.size(); ++i) {
        statesOut[i] = int(states[i]);
    }
    return int(states.size());
}

int
RigExecImaging_ClearFrameCache(const char *rigPath)
{
    if (!rigPath) {
        return -1;
    }
    if (!SdfPath::IsValidPathString(rigPath)) {
        return -1;
    }
    RigExecImagingRegistry::GetInstance().ClearFrameCache(SdfPath(rigPath));
    return 0;
}

int
RigExecImaging_WarmRange(const char *rigPath, const double *frames, int count)
{
    if (!rigPath || count < 0 || (!frames && count > 0)) {
        return -1;
    }
    if (!SdfPath::IsValidPathString(rigPath)) {
        return -1;
    }
    std::vector<double> times;
    times.reserve(size_t(count));
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(frames[i])) {
            return -1;
        }
        times.push_back(frames[i]);
    }
    return RigExecImagingRegistry::GetInstance().SetWarmRange(
               SdfPath(rigPath), times)
        ? 0
        : -1;
}

long long
RigExecImaging_GetWarmingCompletedCount()
{
    return static_cast<long long>(
        RigExecImagingRegistry::GetInstance().GetBackgroundStats().completed);
}

// Manipulation preview (docs/superpowers/specs/
// 2026-09-10-hydra-preview-manipulation-design.md). Three calls, and only the
// middle one runs per mouse sample: strings are marshalled once per drag and
// every sample after that is an array of doubles.
int
RigExecImaging_BeginPreview(const char *packedAttributePaths)
{
    return RigExecImagingRegistry::GetInstance().BeginPreview(
        packedAttributePaths ? std::string(packedAttributePaths)
                             : std::string());
}

int
RigExecImaging_UpdatePreview(const double *values, int count)
{
    if (count < 0) {
        return 1;
    }
    return RigExecImagingRegistry::GetInstance().UpdatePreview(
               values, size_t(count)) ? 0 : 1;
}

int
RigExecImaging_EndPreview()
{
    return RigExecImagingRegistry::GetInstance().EndPreview() ? 0 : 1;
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
