//
// RigExec Hydra publication bridge (spec §8.2 generation-fenced flow).
//
// Owns the evaluator-side of the imaging chain: evaluation completes
// first, then the complete immutable generation is atomically published
// to the snapshot store, and only then are precise dirtied notices sent.
// Hydra pulls never compute or wait.
//
#ifndef RIGEXEC_IMAGING_BRIDGE_H
#define RIGEXEC_IMAGING_BRIDGE_H

#include "sceneIndices.h"
#include "snapshotStore.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/generation.h"
#include "rigExec/outputAffectedIndex.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/taskListCache.h"

#include "pxr/usd/usd/notice.h"

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rigExec {

class RigExecWarmFrameIndex;

/// Publishes one background warming completion into \p cache, and ONLY into
/// the cache: never into the snapshot store the viewport reads.
///
/// The generation fence is checked first: a token that is no longer the
/// rig's current one means an edit landed while the job ran, and the result
/// is dropped. An invalid pose is likewise dropped. Either way the frame the
/// job would have warmed simply evaluates live when asked.
///
/// The fence-check and the cache insert run atomically under the
/// scheduler's fence mutex (plan 3.3): a running job can never pass its
/// check before a generation bump and insert after the fenced clear. The
/// fenced clear (generation-bump + queue-purge + cache-clear + index-reset)
/// holds the same mutex, so a clear-while-warming retires the late
/// completion instead of letting it repopulate the cache. Lock order: fence
/// outermost, then the scheduler mutex, then cache shards -- never inverted.
/// The UI-thread memoization path needs no fence (the UI thread serializes
/// with itself).
///
/// Free function, not a bridge method, so a worker closure can hold the
/// cache without holding the bridge: sessions (and their bridges) die on
/// Deactivate while pool workers outlive them, and a completion for a dead
/// session must land in a dead cache, not in freed memory.
bool RigExecPublishBackgroundCompletion(
    const std::shared_ptr<RigExecFrameCache> &cache,
    const RigExecFrameCacheKey &key, UsdTimeCode time,
    const RigExecRigPose &pose, RigExecFrameGeneration generation,
    const RigExecBackgroundScheduler *scheduler, const SdfPath &rig);

/// The fenced, retained-state completion (plan 2.0/2.1): as above, plus the
/// per-time fence token and the retained handle with its provenance. The
/// fence requires generation AND token match (see
/// RigExecBackgroundScheduler::IsWarmRequestCurrent): an old job for a
/// scoped-purged time drops here and can never overwrite the requeued
/// result. The overload above publishes pose-only under token zero, which
/// matches only a never-purged time.
bool RigExecPublishBackgroundCompletion(
    const std::shared_ptr<RigExecFrameCache> &cache,
    const RigExecFrameCacheKey &key, UsdTimeCode time,
    const RigExecRigPose &pose, RigExecFrameGeneration generation,
    const RigExecBackgroundScheduler *scheduler, const SdfPath &rig,
    RigExecWarmFenceToken fenceToken,
    std::shared_ptr<const void> retained, size_t retainedBytes,
    const RigExecEntryProvenance *provenance);

/// Test-only probe for the deterministic fence-race test: when set, the
/// fenced publish above runs it while holding the fence mutex, after a
/// passing fence-check and before the cache insert. Null in production.
/// Set only while no publish is in flight (tests set it around one driven
/// job); the probe itself must never publish or clear.
void RigExecSetPublishFenceProbeForTesting(
    std::function<void()> probe);

/// One frame's freshness proof (plan 2.3): the folded control digest a
/// lookup must reproduce, the unfolded digest it folds from, and the
/// sampled-path dependency set the digest covers -- first-win sample paths
/// plus standing override identities, sorted and unique. An edit retires
/// the proofs whose sets intersect its affected inputs instead of clearing
/// the map wholesale; a constant-namespace carry re-points a surviving
/// proof by re-folding its unfolded half. The digest compare stays the
/// correctness backstop: an over-retained proof only costs a sample and a
/// miss, never a wrong pose.
struct RigExecFreshProof {
    uint64_t digest = 0;
    uint64_t unfolded = 0;
    std::vector<std::string> paths;
    RigExecFreshProof() = default;
    RigExecFreshProof(uint64_t digestIn, uint64_t unfoldedIn,
                      const RigExecFrameInputs *inputs,
                      const std::vector<RigExecValueOverride> *overrides);
};

/// Drives one rig's evaluation into the imaging chain.
class RigExecImagingBridge {
public:
    RigExecImagingBridge(
        const UsdStageRefPtr &stage, const SdfPath &rigPath);

    /// Constructs over an externally owned store (the application may
    /// create the store before any scene index or bridge exists).
    RigExecImagingBridge(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        std::shared_ptr<RigExecSnapshotStore> store);

    /// The outcome of one evaluate-and-publish step, for callers that
    /// broadcast notices to multiple filter chains.
    struct PublishResult {
        bool ok = false;
        RigExecPublishedDirtyVector dirtied;
        /// Non-null when the binding epoch changed this generation.
        RigExecBindingResolvingSceneIndex::BindingEpochConstPtr epoch;
        /// True when the published generation came from the frame cache
        /// without running the evaluator: no evaluator pull happened. False
        /// on the live path (including a shadow-verified hit, which DID
        /// evaluate) and on every path that bypasses the cache.
        bool cacheHit = false;
    };

    /// The renderer's HdSceneIndexCreateArgsSchema.motionBlurSupport
    /// capability bit (spec §10.3.1): false permits one sample, true
    /// allows the explicit render-preflight sample set, and when absent
    /// the application's explicit render profile is authoritative.
    enum class MotionBlurSupport { Absent, False, True };

    /// Validates a render-motion profile against the renderer capability
    /// bit before any evaluation (spec §10.3.1, §10.5): a multi-sample
    /// profile fails preflight when the renderer advertises
    /// motionBlurSupport = false, and an empty offset set never renders.
    static bool PreflightMotionProfile(
        const std::vector<float> &shutterOffsets,
        MotionBlurSupport support,
        std::string *whyNot = nullptr);

    /// Evaluate-then-publish without notifying any scene index; the
    /// caller forwards the result to its chains.
    /// With RIGEXEC_FRAME_CACHE=off this is exactly the historical path:
    /// live evaluation, then publication into the snapshot store. Otherwise
    /// a frame whose inputs digest to a cached entry publishes the stored
    /// pose through the same atomic snapshot without evaluating
    /// (result.cacheHit), and a miss evaluates live as today and memoizes
    /// the result into both the snapshot and the cache. While interactive
    /// overrides stand (an active drag), the cache is bypassed entirely:
    /// live evaluation with neither lookup nor memoization. The multi-sample
    /// render path below never consults the cache.
    PublishResult EvaluateAndPublishResult(UsdTimeCode time);

    /// Render-motion publication: evaluates the rig at
    /// baseTime + offset for every explicit frame-relative shutter offset,
    /// retains points, normals, extents and driven transforms per offset,
    /// and publishes them under one complete generation fence. Diagnostic
    /// guides use the offset nearest zero. Preflight (capability check and offset
    /// validation) fails before any evaluation; an incomplete sample set
    /// never publishes.
    PublishResult EvaluateAndPublishSamples(
        UsdTimeCode baseTime, const std::vector<float> &shutterOffsets,
        MotionBlurSupport support = MotionBlurSupport::Absent);

    /// Compiles the rig; returns false with messages on failure.
    bool Compile(std::vector<std::string> *errors = nullptr);

    /// The store the results scene index reads from.
    const std::shared_ptr<RigExecSnapshotStore> &GetStore() const {
        return _store;
    }

    /// The reserved generated scope this rig owns (for pruning).
    SdfPath GetGeneratedScope() const;

    size_t GetBindingEpochDigest() const {
        return _evaluator->GetBindingEpochDigest();
    }

    /// Wires the filters that receive epoch swaps and generation notices.
    void SetSceneIndices(
        const RigExecBindingResolvingSceneIndexRefPtr &binding,
        const RigExecResultsSceneIndexRefPtr &results) {
        _binding = binding;
        _results = results;
    }

    /// Serialized evaluate-then-publish for one explicit time
    /// (spec §8.2): compute the complete generation, atomically swap the
    /// snapshot, then send coalesced dirtied notices.
    bool EvaluateAndPublish(UsdTimeCode time);

    /// Selects the weight object whose resolved field is painted onto its
    /// weighted geometry as the influence overlay; an empty path turns
    /// the overlay off.
    ///
    /// ONE at a time on purpose. Two gradients composited onto one mesh
    /// are a picture of neither field, and a rigger placing a volume is
    /// asking about exactly one of them.
    ///
    /// Takes effect on the next publication -- this only records the
    /// selection, because publishing is the caller's serialization point
    /// (the registry republishes at the current time right after).
    void SetWeightOverlay(const SdfPath &weightPrimPath) {
        if (weightPrimPath != _weightOverlay) {
            // The overlay flag changes POSE CONTENT (weightFields resolved
            // or not) without changing any sampled input, so entries stored
            // under the other flag describe poses this flag must not serve.
            ClearFrameCache();
            _weightOverlay = weightPrimPath;
        }
        // Resolving the per-point influence field is the overlay's whole
        // cost, and this bridge is its only consumer: pay it only while an
        // overlay is actually selected.
        _evaluator->SetPublishWeightFields(!_weightOverlay.IsEmpty());
    }

    const SdfPath &GetWeightOverlay() const { return _weightOverlay; }

    /// The standing uncommitted manipulation values (see
    /// SetInteractiveOverrides), for the frame-cache key: a drag must never
    /// be served a pre-drag pose, so the digest folds these explicitly.
    const std::vector<RigExecValueOverride> &GetInteractiveOverrides() const {
        return _interactiveOverrides;
    }

    /// This rig's frame cache: the (epochDigest, controlDigest) store the
    /// UI thread memoizes live results into and background completions warm
    /// behind the generation fence. Shared (not unique) so worker closures
    /// can hold it past the session that sampled them.
    const std::shared_ptr<RigExecFrameCache> &GetFrameCache() const {
        return _frameCache;
    }

    /// Drops every cached frame and every freshness proof. The next
    /// publication evaluates live. Called when the overlay selection moves
    /// (pose content changes without the digest moving) and available to
    /// tests that must isolate one warming sequence from the next. The
    /// registry's fenced entries call this under the scheduler's fence
    /// mutex -- it never takes the fence itself (non-recursive).
    void ClearFrameCache();

    /// The shared warm-frame index memoized completions record into (null
    /// for bare bridges, which warm nothing to query). Set once at
    /// activation; the registry owns the index.
    void SetWarmFrameIndex(std::shared_ptr<RigExecWarmFrameIndex> index);

    /// Records the freshness proof for a warming job built for \p time whose
    /// sampled inputs digest to \p controlDigest. Called on the UI thread at
    /// ENQUEUE time by BuildWarmWork, under the registry lock, for chain-free
    /// jobs only -- never for a vector carrying chain-resolved samples, whose
    /// digest names stale values and must prove nothing.
    ///
    /// Without this, a warmed entry is unreachable: lookups serve only under
    /// a proof, and proofs were recorded only right after a live evaluation
    /// -- which memoizes its own result anyway. Recording the enqueue-time
    /// digest as the proof is what lets a later lookup serve the warmed pose
    /// without evaluating. Safe by the same argument as the live path: the
    /// sampler is deterministic in the stage, so a lookup that samples the
    /// same values proves the same digest, and anything that moved digests
    /// apart and misses. A job that never completes leaves a proof pointing
    /// at no entry, which reads as a miss, never as a wrong pose.
    ///
    /// No-op when the frame cache is off (nothing is served, so nothing is
    /// proven). Applies the same epoch/mode scope discipline as the live
    /// memoization: a moved scope clears every proof, this one included.
    /// \p unfoldedDigest is the pre-constant-fold digest and \p inputs the
    /// sampled vector, recorded into the proof's dependency set (plan 2.3).
    void NoteWarmingEnqueued(UsdTimeCode time, uint64_t controlDigest,
                             uint64_t unfoldedDigest,
                             const RigExecFrameInputs &inputs);

    /// The epoch's output-affected index (plan 2.1), synced to the current
    /// program and epoch: control IDs to dirty-cluster closures, the
    /// notice adapter's oracle for path-scoped retirement. Null with no
    /// baked program; empty when the program's cones were never built (a
    /// caller that finds it empty retires conservatively, as before).
    /// UI thread only, like the bridge itself: no worker consults it.
    const RigExecOutputAffectedIndex *GetAffectedIndex();

    /// Admits \p overrides' identities to the epoch index (plan 2.1
    /// MapControl production caller): the dynamic-override bootstrapping
    /// path, so a changed override maps to its clusters instead of
    /// collapsing to foreign (all clusters). Seeds mirror
    /// RigExecBakedProgram::SetOverrides placement exactly; an override
    /// that places nowhere is left foreign -- conservative, never wrong.
    /// No-op with no baked program. UI thread only.
    void AdmitOverrideControls(
        const std::vector<RigExecValueOverride> &overrides);

    /// Retires every freshness proof whose dependency set intersects \p
    /// controls (plan 2.3): the path-scoped counterpart to the wholesale
    /// clears. Answers how many proofs were dropped. UI thread only.
    size_t RetireProofsForControls(
        const std::vector<RigExecControlId> &controls);

    /// Re-points the proof for \p time at \p newDigest (plan 2.2 lane-b
    /// carry-over): the entry was re-published under a new key with
    /// identical content, so the surviving proof names the new digest.
    /// Answers false when no proof is recorded for the time. The
    /// dependency set is unchanged: the inputs did not move, only the
    /// constant namespace did. UI thread only.
    bool RepointProof(UsdTimeCode time, uint64_t newDigest);

    /// Answers the recorded proof for \p time, or false when none is held.
    /// For tests pinning proof scoping; production serves through the
    /// lookup path, never through this.
    bool GetFreshProof(UsdTimeCode time, RigExecFreshProof *proof) const;
    size_t GetFreshProofCount() const { return _freshDigests.size(); }

    /// Carries the entry published for \p time under \p oldKey into the
    /// constant namespace \p newConstants (plan 2.2 lane b): re-publishes
    /// the pose with its retained handle and provenance under the re-keyed
    /// key, evicts the old key, re-points the proof, and re-records the
    /// warm-index completion -- with zero re-evaluation. The caller has
    /// already proven the entry clean (its provenance touches nothing the
    /// edit dirtied); this only proves the MECHANICS: answers false,
    /// having changed nothing observable, when the entry, its retained
    /// handle, or its unfolded digest is missing, when the re-publish
    /// declines, or when the entry already keys under the new namespace
    /// (not a carry -- the caller stands it instead). A false degrades
    /// the frame to lane c (retire + re-warm), never strands it. UI
    /// thread only.
    bool CarryEntry(UsdTimeCode time, const RigExecFrameCacheKey &oldKey,
                    uint64_t newConstants);

    /// Routes a stage notice through the baked capture index: a hit drops
    /// this rig's cached frames (and memoized task lists) for the epoch
    /// while its half still names it -- the evaluator rebuilds lazily, and
    /// between the notice and the rebuild the old epoch still keys lookups.
    /// A miss drops nothing (D1 reachability decides). With no baked
    /// program (D7) nothing is dropped: refusal entries key on the
    /// stage-edit serial, which already retires them. Answers whether the
    /// notice hit. Called from the registry's notice path, under its lock;
    /// the cache and memo serialize bare-bridge callers too.
    bool NoteCaptureIndex(const UsdNotice::ObjectsChanged &notice);

    /// Lifetime counters of this rig's frame cache (zeros when empty).
    RigExecFrameCacheStats GetFrameCacheStats() const;

    /// Uncommitted manipulation values for this rig, with nothing authored
    /// (RigExecRigEvaluator::SetInteractiveOverrides). Records only, like
    /// SetWeightOverlay: publishing is the caller's serialization point.
    void SetInteractiveOverrides(std::vector<RigExecValueOverride> overrides) {
        _interactiveOverrides = overrides;
        _evaluator->SetInteractiveOverrides(std::move(overrides));
    }

    void ClearInteractiveOverrides() {
        _interactiveOverrides.clear();
        _evaluator->ClearInteractiveOverrides();
    }

    /// The stage the rig evaluates against, for a caller that has to read the
    /// authored value an override is standing in for.
    const UsdStageRefPtr &GetEvaluationStage() const {
        return _evaluator->GetEvaluationStage();
    }

private:
    /// Publishes guide payloads (joints and aggregate solvers draw as
    /// guide geometry like OpenExec's IrJointScope) into the snapshot.
    void _FillProviderXforms(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    void _FillGuides(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    /// Publishes the single synthesized guide shape each control draws at
    /// its posed frame (spec §10.3 extension).
    /// Records the stage identity and sample time on a generation.
    /// Samples this rig's inputs at \p time and folds the cache key, or
    /// false when no faithful key exists (a bakeable-but-unsampleable rig,
    /// an undigestible held type): the caller evaluates live and memoizes
    /// nothing. A rig with no baked program (plan D7) keys on time plus the
    /// stage-edit serial plus the standing overrides instead of a sampled
    /// vector.
    bool _ComputeCacheKey(UsdTimeCode time, RigExecFrameCacheKey *key) const;

    /// Publishes \p pose -- live or cached, already stamped for \p time --
    /// as one complete immutable generation: geometry, guides, the binding
    /// epoch when it moved, and the atomic snapshot swap. Shared by the hit
    /// path and the live path so the two cannot publish differently.
    void _PublishPoseSnapshot(
        UsdTimeCode time, const RigExecRigPose &pose,
        PublishResult *result);

    /// Serves \p time from the frame cache when a proven entry exists, or
    /// false to fall through to live evaluation. A hit never runs the
    /// evaluator (result.cacheHit); under RIGEXEC_FRAME_CACHE_VERIFY=1 the
    /// hit is shadow-proven against a live evaluation and counts as one.
    bool _TryPublishCachedResult(UsdTimeCode time, PublishResult *result);

    /// Serves \p time from its same-time retained base when the sparse
    /// planner proves the retained pose is still the answer (plan 2.2):
    /// the digest missed (values moved, or the proof was retired) but
    /// nothing the entry's computation reaches did. Re-keys the entry
    /// under \p key (the fresh sample's key) and serves it with zero
    /// evaluator pulls; false falls through to live evaluation. The base
    /// is ALWAYS this time's own completion -- a cross-time base could
    /// alias one frame's pose onto another's through stale chain values.
    /// \p unfolded is the fresh sample's pre-fold digest, recorded into
    /// the re-pointed proof. UI thread only.
    bool _TrySparseServe(UsdTimeCode time, const RigExecFrameInputs &inputs,
                         const RigExecFrameCacheKey &key, uint64_t unfolded,
                         PublishResult *result);

    /// Publishes an already-servable cached \p pose -- a store hit or a
    /// sparse-serve -- as the generation for \p time: the alias record,
    /// the shadow verify under RIGEXEC_FRAME_CACHE_VERIFY=1 (a mismatch
    /// serves live and repairs the entry), and the snapshot publication
    /// with result.cacheHit set. Shared by the hit path and the sparse
    /// path so the two cannot publish differently.
    void _ServeCachedPose(UsdTimeCode time, const RigExecRigPose &pose,
                          const RigExecFrameCacheKey &key,
                          PublishResult *result);

    /// Memoizes a just-evaluated pose into the frame cache and records its
    /// digest as its time's freshness proof. Sampled after the evaluation,
    /// so the proof names fresh inputs.
    void _MemoizeLiveResult(UsdTimeCode time, const RigExecRigPose &pose);

    void _StampGeneration(
        UsdTimeCode time, RigExecImagingSnapshot *snapshot) const;

    void _FillControlGuides(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    /// Publishes the wire (or solid) falloffMin/falloffMax iso-surfaces
    /// every placed influence volume draws, so a rigger can see the shape
    /// being placed rather than infer it from the deformation.
    void _FillVolumeGuides(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    /// Samples native curvenets from their evaluated control-point pools.
    void _FillCurvenetGuides(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    /// Copies the selected weight object's resolved field onto the
    /// geometry prim it weights (spec §10.3 influence-overlay extension).
    void _FillWeightOverlay(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    UsdStageRefPtr _stage;
    SdfPath _rigPath;
    /// Weight object currently painted as the influence overlay; empty
    /// means off, which is the default and the ordinary render.
    SdfPath _weightOverlay;
public:
    /// The evaluator's profiler, which the imaging layer also records its
    /// own publish phases into, so one summary covers a whole viewport
    /// update. Recording is off unless RIGEXEC_IMAGING_PROFILE is set.
    const RigExecProfiler &GetProfiler() const
    {
        return _evaluator->GetProfiler();
    }
    RigExecProfiler *MutableProfiler() const
    {
        return const_cast<RigExecProfiler *>(&_evaluator->GetProfiler());
    }
    /// The evaluation mode actually answering this rig (baked, dynamic...),
    /// for diagnostics a viewer shows. See RigExecRigEvaluator.
    const RigExecRigEvaluator &GetEvaluator() const { return *_evaluator; }

    /// Forgets every cached guide input. Called for any stage notice that
    /// touches the rig and on every recompile: the caches hold AUTHORED
    /// styling, and only an edit can move that.
    void InvalidateGuideCaches();

private:
    /// A prim's guide styling, read once and republished every generation.
    ///
    /// Measured on the biped: rebuilding the guides for ~400 joints and
    /// controls re-read every one of these off the stage on EVERY mouse
    /// move of a drag -- 14-16 ms of each move, more than the rig's own
    /// evaluation. They are authored values that do not move while a
    /// control is dragged, so each is read once. An attribute that CAN move
    /// between two generations -- connected (the opacity an IK/FK switch
    /// drives) or time-varying -- is marked live and read exactly as before.
    struct _GuideInputs {
        UsdPrim prim;
        bool styleReady = false;
        TfToken purpose;
        UsdAttribute colorAttr;
        bool colorLive = false;
        bool hasColor = false;
        GfVec3f color;
        UsdAttribute opacityAttr;
        bool opacityLive = false;
        bool hasOpacity = false;
        float opacity = 1.0f;
        bool controlReady = false;
        bool controlLive = false;
        TfToken shape;
        TfToken drawMode;
        GfVec3d scale;
        double wireWidth = 0.05;
        GfVec3d offset = GfVec3d(0.0);
        bool radiusReady = false;
        bool radiusLive = false;
        double radius = 1.0;
    };
    _GuideInputs &_GuideInputsFor(const SdfPath &path) const;
    void _ReadGuideStyleCached(_GuideInputs &inputs,
                               const RigExecRigPose &pose,
                               RigExecPublishedPrim *published) const;
    double _GuideRadius(_GuideInputs &inputs, UsdTimeCode time) const;
    const std::map<SdfPath, std::vector<SdfPath>> &
    _JointChildren(const RigExecRigPose &pose) const;

    std::unique_ptr<RigExecRigEvaluator> _evaluator;
    std::shared_ptr<RigExecSnapshotStore> _store;
    RigExecBindingResolvingSceneIndexRefPtr _binding;
    RigExecResultsSceneIndexRefPtr _results;
    static constexpr size_t _kFreshDigestCap = 1024;
    uint64_t _generation = 0;
    size_t _publishedEpochDigest = 0;
    /// This rig's frame cache, held shared so background completions can
    /// publish into it after the session that sampled them is gone.
    std::shared_ptr<RigExecFrameCache> _frameCache;
    /// The shared warm-frame index, or null for a bare bridge.
    std::shared_ptr<RigExecWarmFrameIndex> _warmIndex;
    /// The memoized affected-set table for sparse planning (Stream D): the
    /// capture index retires its epochs here, beside the frames.
    RigExecTaskListCache _taskListMemo;
    /// The epoch half of the last memoized key. Atomic for bare bridges,
    /// which serialize on no lock (registry callers hold its mutex on both
    /// the memoizing and the notice paths). A notice that finds it drifted
    /// from the current epoch evicts the old half eagerly -- its frames are
    /// unreachable by construction (D1) -- instead of waiting for LRU.
    std::atomic<uint64_t> _lastPublishedEpoch{0};
    /// The standing overrides, as handed to SetInteractiveOverrides: the
    /// digest folds them explicitly (a drag must never hit a pre-drag pose)
    /// and warming jobs sample them at enqueue time.
    std::vector<RigExecValueOverride> _interactiveOverrides;
    /// Freshness proofs: (isDefault, timeValue) -> the proof recorded for
    /// a sample taken right after a live evaluation AT that time, when
    /// every chain-resolved input was fresh. A lookup serves a cached pose
    /// only when its pre-evaluation sample digests to the recorded proof,
    /// which is what makes a hit bit-identical to a live evaluation:
    /// chain inputs sampled for a time the evaluator has not run come from
    /// the standing (stale) resolved state, and without the proof a scrub
    /// forth and back over animated chains could alias one frame's pose
    /// onto another's. Scoped by _freshEpoch and by the evaluation mode:
    /// either moving clears the map (entries stay in the cache,
    /// unreachable, for LRU). Bounded: past _kFreshDigestCap entries the
    /// map clears wholesale, which only costs misses, never correctness.
    /// Path-scoped retirement (plan 2.3) drops intersecting proofs via
    /// RetireProofsForControls instead of clearing wholesale.
    std::map<std::pair<bool, double>, RigExecFreshProof> _freshDigests;
    /// The epoch's output-affected index (plan 2.1), built for
    /// _affectedEpoch by _SyncAffectedIndex and re-admitted per epoch.
    /// UI thread only: no worker consults it, so no lock guards it.
    std::unique_ptr<RigExecOutputAffectedIndex> _affectedIndex;
    const RigExecBakedProgram *_affectedProgram = nullptr;
    uint64_t _affectedEpoch = 0;
    bool _affectedValid = false;
    /// Rebuilds the epoch index when the program or epoch moved.
    /// Answers the synced index, or null with no baked program.
    const RigExecOutputAffectedIndex *_SyncAffectedIndex();
    uint64_t _freshEpoch = 0;
    bool _freshEpochValid = false;
    RigExecEvaluationMode _cacheMode = RigExecEvaluationMode::Baked;
    bool _cacheModeValid = false;
    mutable std::unordered_map<SdfPath, _GuideInputs, SdfPath::Hash>
        _guideInputs;
    mutable std::map<SdfPath, std::vector<SdfPath>> _jointChildren;
    /// Joint path -> whether a RigExecControl is above it (through joints): the
    /// hidden pivots nested in a control hierarchy. Stage-edit scoped like
    /// the other guide caches.
    mutable std::unordered_map<SdfPath, bool, SdfPath::Hash>
        _controlSpaceJoints;
    bool _IsControlSpaceJoint(const SdfPath &path) const;
    mutable size_t _jointChildrenKey = 0;
    mutable bool _jointChildrenValid = false;
    /// The evaluator's stage-edit serial the guide caches were filled under.
    mutable uint64_t _guideCacheSerial = 0;
    /// Drops the guide caches if any stage edit has happened since they
    /// were filled. Called at the top of every fill that reads them.
    void _SyncGuideCaches() const;
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_BRIDGE_H
