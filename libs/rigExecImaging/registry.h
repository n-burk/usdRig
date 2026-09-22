//
// RigExec imaging registry: process-global rendezvous between the
// UsdImaging scene-index plugin (which builds filter chains when engines
// construct) and the application activation (which owns the stage,
// evaluator, and time source). Order-independent: chains read the shared
// snapshot store atomically, so activation may happen before or after
// chain construction.
//
#ifndef RIGEXEC_IMAGING_REGISTRY_H
#define RIGEXEC_IMAGING_REGISTRY_H

#include "bridge.h"
#include "playback.h"
#include "sceneIndices.h"
#include "snapshotStore.h"
#include "warmIndex.h"

#include "rigExec/frozenContext.h"

#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

class RigExecImagingRegistry : public TfWeakBase {
public:
    static RigExecImagingRegistry &GetInstance();

    /// The shared store every results scene index reads from.
    const std::shared_ptr<RigExecSnapshotStore> &GetStore() const {
        return _store;
    }

    /// Called by the scene-index plugin for each constructed chain.
    void RegisterChain(
        const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
        const RigExecBindingResolvingSceneIndexRefPtr &binding,
        const RigExecResultsSceneIndexRefPtr &results,
        const RigExecXformOverrideSceneIndexRefPtr &xforms = nullptr);

    /// Activates evaluation for one rig, or every RigExecRoot when rigPath is
    /// empty.  Compilation and the first evaluation complete off to the side;
    /// the active stage and Hydra generation change only after every rig has
    /// succeeded.
    bool Activate(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        UsdTimeCode initialTime, std::vector<std::string> *errors);

    /// Serialized evaluate-then-publish, broadcast to every chain.
    bool SetTime(UsdTimeCode time);

    /// Number of actual evaluator pulls for one active session, for profiling.
    /// Cache hits are not pulls and are not counted.
    size_t GetSessionEvaluationCount(const SdfPath &rigPath);

    /// The primary warming trigger: call on drag release / value commit, on
    /// the UI thread. Enqueues the scrub neighbors (+-1..N of the playhead)
    /// plus the Premonition-style sweep of the surrounding range for every
    /// active non-playback rig, sampling each job's input vector on this
    /// thread. The playhead itself is never enqueued (the playhead always
    /// evaluates live). Returns how many jobs were enqueued across rigs.
    ///
    /// \p runner is the frozen serial step the workers execute; the default
    /// (null) builds the production runner via RigExecMakeProductionStepRunner,
    /// which is what the C-API triggers run under. Tests inject a kernel.
    size_t OnEditCommitted(
        RigExecFrozenStepRunner runner = RigExecFrozenStepRunner());

    /// The secondary warming trigger: call from the frame loop when the UI
    /// is idle. Enqueues the sweep only (neighbors belong to the commit
    /// trigger) with the same runner, gate, and generation rules.
    size_t OnIdle(
        RigExecFrozenStepRunner runner = RigExecFrozenStepRunner());

    /// Builds one frame's background work for \p rig at \p time under \p
    /// generation, sampling the input vector on the calling (UI) thread.
    /// A null \p runner builds the production runner (see OnEditCommitted).
    /// Empty work when no faithful job exists: unknown or playback rig,
    /// default or non-finite time, stale generation, refusal rig (D7: no
    /// background job, ever), unsampleable inputs, a chain the sampling
    /// hook declined (no job is ever built from stale chain values), or an
    /// undigestible held type. A built job also records its time's
    /// freshness proof, so a completion is servable without a live visit.
    /// The scheduler triggers build their factories from this; tests drive
    /// it directly for deterministic fence coverage.
    ///
    /// Skips carry reasons: D7Exempt (no baked program), FreezeRefused (no
    /// snapshot; the detail is the session's freeze error), or Unsampleable
    /// (the detail names the sampler gate). Trigger-shape skips (unknown
    /// rig, stale generation, bad time) carry none.
    ///
    /// \p burst, when usable and prepared for this rig's program, takes
    /// the cached route: no per-frame snapshot refresh, pin
    /// re-verification, or epoch-constant table walks, and static reads
    /// and digests served from the burst maps. Anything else falls back
    /// to the plain per-frame route, which re-derives everything exactly
    /// as before -- including a null cache, which is what the direct
    /// test drivers pass.
    RigExecWarmFactoryResult BuildWarmWork(
        const SdfPath &rig, UsdTimeCode time,
        RigExecFrameGeneration generation, RigExecFrozenStepRunner runner,
        RigExecBurstSampleCache *burst = nullptr);

    /// The rig's current warming generation (0 when inactive or unknown).
    RigExecFrameGeneration CurrentFrameGeneration(const SdfPath &rig);

    /// The rig's current per-time fence token for \p time (0 when inactive,
    /// unknown, or never purged): the token the scheduler stamps on jobs
    /// enqueued now, and the one the publish fence requires. For tests
    /// pinning the scoped-cancel protocol; production never reads it.
    RigExecWarmFenceToken CurrentFenceToken(const SdfPath &rig,
                                            UsdTimeCode time);

    /// Bumps the rig's warming generation and purges its queued jobs. Every
    /// rig-affecting edit already calls this internally through the stage
    /// notice; hosts call it directly only for edits the notice cannot see.
    void CancelFrameGeneration(const SdfPath &rig);

    /// Lifetime warming counters (queue depth, completions, drops...).
    RigExecBackgroundSchedulerStats GetBackgroundStats();

    /// Blocks until no warming job is queued or running.
    void WaitUntilBackgroundIdle();

    /// Lifetime frame-cache counters for one rig (zeros when the rig is
    /// unknown, a playback session, or simply cold).
    RigExecFrameCacheStats GetFrameCacheStats(const SdfPath &rig);

    /// The warming scheduler's profiler (per-sweep-time factory costs land
    /// here), for benches and tests; null when the scheduler is off.
    RigExecProfiler *MutableSchedulerProfiler();

    /// One rig's bridge profiler (memoize and burst-prep scopes land here),
    /// or null when the rig is unknown or a playback session. Valid while
    /// the session stays activated.
    RigExecProfiler *MutableBridgeProfiler(const SdfPath &rig);

    /// Whether the rig's standing warm burst is usable (false when the rig
    /// is unknown, a playback session, shape-declined, or overrun). The
    /// flag the over-slice test asserts: an aborted prep parks false and
    /// the tick takes the plain route.
    bool GetWarmBurstUsable(const SdfPath &rig);

    /// Millisecond slice for one standing-burst rebuild; a rebuild past
    /// it parks the burst unusable (overrun) instead of serving a stale
    /// one. Default 50 ms -- the stack's steady rebuild measures ~18 ms,
    /// so the slice only bites on pathology (or a test's negative).
    /// A negative slice overruns deterministically: a build always costs
    /// >= 0 ms, while a 0 slice races the clock on rigs that build in
    /// under a microsecond. Setting the
    /// slice unlatches overruns: the next trigger retries the build under
    /// the new slice. A usable burst and a shape decline stand.
    void SetWarmBurstPrepSliceMs(double ms);
    double GetWarmBurstPrepSliceMs();

    /// The rig's last reasoned factory skip (None when the rig is unknown
    /// or no reasoned skip has landed) and its detail: the freeze cause
    /// for FreezeRefused, the sampler gate for Unsampleable, the D7 note
    /// for D7Exempt. Burst prep records the session-level verdict on its
    /// null-factory paths, so a factory-less trigger names its cause.
    /// Unreasoned skips and successful builds leave it standing.
    RigExecWarmSkipReason GetLastWarmSkipReason(const SdfPath &rig);
    std::string GetLastWarmSkipDetail(const SdfPath &rig);

    /// Per-trigger sampling budget shared by both triggers (per rig): max
    /// factory invocations (default 16 -- one neighbor band: a commit warms
    /// neighbors now, the sweep follows on idle ticks) and a millisecond
    /// stop measured after burst prep (default 8 ms -- cached-route
    /// sampling runs ~0.1 ms/frame on the biped, so the count binds;
    /// the stop binds the plain route). Prep (frozen refresh + rebuild)
    /// runs outside this budget under its own slice: measuring the stop
    /// from the tick's start let a slow prep starve sampling forever.
    /// Pass (max, infinity) for unlimited.
    void SetWarmSamplingBudget(size_t maxFactoryInvocations, double maxMs);
    size_t GetWarmSamplingMaxInvocations();
    double GetWarmSamplingMaxMs();

    /// Sets the rig's persistent warm range (sorted and de-duplicated on
    /// the way in; an empty list warms nothing): the full-range cursor
    /// visits these frames closest-first from the playhead across idle
    /// triggers instead of the default playhead-relative sweep. Returns
    /// false for an unknown or playback rig.
    bool SetWarmRange(const SdfPath &rig, const std::vector<double> &frames);

    /// Fenced cancel-THEN-clear plus 1.4 index reset, in that order (plan
    /// 3.3): bumps the rig's warming generation and purges its queued jobs,
    /// then drops its cached frames and freshness proofs, then resets its
    /// warm-frame index. The whole sequence holds the scheduler's fence
    /// mutex, so a running job can never insert a late completion into the
    /// cleared cache; the cancel runs in its own scope before locking for
    /// the clear (the registry mutex is non-recursive). The C binding, the
    /// overlay path, and every caller route through here.
    void ClearFrameCache(const SdfPath &rig);

    /// Per-frame warming states over \p frames, in order -- one call per
    /// strip repaint, no sampling on the query path. Empty when the rig is
    /// unknown, a playback session, or simply unqueried (no frames), or
    /// when the list exceeds 100k frames.
    std::vector<RigExecWarmFrameState> GetFrameStates(
        const SdfPath &rig, const std::vector<double> &frames);

    /// One rig's session bridge, or null when the rig is unknown or a
    /// playback session. For tests driving plan-2.x scenarios (provenance
    /// surgery, proof inspection, evaluator queries): production hosts
    /// never need it. Valid while the session stays activated, and only
    /// under the caller's own serialization with triggers.
    RigExecImagingBridge *GetBridge(const SdfPath &rig);

    /// Every (time, key) completion the warm index holds for the rig, in
    /// index order. For tests pinning the 2.2 re-resolve lanes (which key
    /// each frame completed under, before and after a carry). Empty when
    /// the rig is unknown or holds no completions.
    std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>>
    GetCompletedKeys(const SdfPath &rig);

    /// Selects the weight object painted as the influence overlay, and
    /// republishes at the current time so the viewport updates without
    /// waiting for a frame change. An empty string turns the overlay off.
    ///
    /// The selection is remembered even with no bridge activated, so a
    /// host that sets it before opening a rig gets the overlay on the
    /// first generation rather than none at all.
    bool SetWeightOverlay(const std::string &weightPrimPath);

    /// Declares what one manipulation is going to change, and returns how
    /// many doubles UpdatePreview will expect (-1 when the declaration is
    /// rejected). \p packedAttributePaths is newline-separated absolute
    /// attribute paths.
    ///
    /// Declared once per drag so the per-sample call can be nothing but
    /// numbers. Resolving a path to a rig session, reading its value type, and
    /// deciding which lane it previews through are all done HERE, once,
    /// because the interactive cost of a manipulation is the cost of the thing
    /// that runs per mouse sample.
    ///
    /// Two lanes, chosen per attribute and invisible to the caller:
    ///
    ///   * an attribute under an active rig previews through the evaluator,
    ///     because moving a control has to re-run the rig
    ///     (RigExecRigEvaluator::SetInteractiveOverrides);
    ///   * an xformOp on a prim no rig drives previews through
    ///     RigExecXformOverrideSceneIndex, which is the same prim's transform
    ///     and nothing else.
    int BeginPreview(const std::string &packedAttributePaths);

    /// One mouse sample: the declared slots' values, flattened in declaration
    /// order -- 1 double for a scalar, 3 for a vector, 16 for a matrix.
    /// Evaluates and republishes. Authors nothing.
    bool UpdatePreview(const double *values, size_t count);

    /// Writes every active rig's recorded profile totals to \p path and
    /// clears them. See RigExecImaging_WriteProfileSummary.
    bool WriteProfileSummary(const std::string &path);

    /// Ends the manipulation: drops every override and delta and republishes
    /// the authored rig. The application authors the committed values itself,
    /// before or after this call -- the two are independent, which is why an
    /// aborted drag is this call alone.
    bool EndPreview();

    bool IsPreviewActive() const;

    /// Drops the bridge; chains remain and read the (cleared) store.
    void Deactivate();

    bool IsActive() const { return !_sessions.empty(); }

    /// Activates the stage's rigs unless this exact stage is already active
    /// (adapter-driven discovery, docs/specs/imaging-datasource-redesign.md
    /// §3.2).
    ///
    /// The stage scene index's rig adapter calls this the first time a
    /// RigExecRoot prim's data is built, so a host with no explicit
    /// activation (usdrecord) still evaluates. A no-op when already active
    /// on \p stage -- hosts that activate explicitly (the usdview plugin)
    /// are unaffected -- and otherwise exactly Activate with an empty rig
    /// path: every RigExecRoot on the stage, compiled and published at
    /// \p time.
    ///
    /// NOT a substitute for Activate: a rig authored into an already-active
    /// stage still needs an explicit re-activation, which is what the
    /// usdview plugin's root-set tracking does.
    ///
    /// Refuses (silently returns false) while active on a DIFFERENT stage:
    /// unlike an explicit Activate, the automatic path never steals a live
    /// activation. The refusal is silent because the adapter calls this on
    /// every data pull -- a second stage's rigs would otherwise warn once
    /// per pull, forever.
    bool EnsureActivated(
        const UsdStageRefPtr &stage, UsdTimeCode time);

    /// Whether \p path is an active rig root. The results index's
    /// _PrimsDirtied time trigger only fires for these, so output-prim
    /// dirties (including the universal ones from epoch swaps) never
    /// re-enter evaluation.
    bool IsActiveRigRoot(const SdfPath &path);

    /// Records a RigExecRoot sighting for eager activation. The rig adapter
    /// calls this from GetImagingSubprims, which the stage scene index runs
    /// for every prim during populate -- long before any data pull -- so the
    /// results index's _PrimsAdded can force the rig's data (and its
    /// time-varying flag, and its activation) into existence on the populate
    /// thread instead of whichever render worker pulls first.
    ///
    /// Always records, even while active: GetImagingSubprims runs inside
    /// GetChildPrimPaths walks the xform-override index makes WHILE HOLDING
    /// _mutex (a preview's _DirtyXformSubtree), so this must never take it.
    /// The set only holds distinct rig paths, so recording while active is
    /// bounded and harmless -- the activation it forces resolves to a no-op
    /// inside EnsureActivated. A rig authored into an already-active stage
    /// is still the usdview plugin's re-activation to make: the forced pull
    /// builds its data but EnsureActivated refuses to replace a live stage
    /// (see below). Cleared by Activate and Deactivate; a note that
    /// outlives a failed activation retries on the next resync, like the
    /// plugin's own retry.
    ///
    /// Paths only, no stage: the note is just the _PrimsAdded matcher, and
    /// the activation it forces runs through the pulled prim's own adapter,
    /// which always names the true stage. A path collision across two stages
    /// therefore still activates the right one.
    void NoteRigRoot(const SdfPath &rigPath);

    /// Whether \p path was noted by NoteRigRoot and is still pending.
    bool IsNotedRigRoot(const SdfPath &path);

private:
    RigExecImagingRegistry();

    // Weak references: chains are owned by their scene index graphs and
    // die with their engines; the registry prunes expired entries.
    struct Chain {
        TfWeakPtr<RigExecInternalPrimPruningSceneIndex> pruning;
        TfWeakPtr<RigExecBindingResolvingSceneIndex> binding;
        TfWeakPtr<RigExecResultsSceneIndex> results;
        TfWeakPtr<RigExecXformOverrideSceneIndex> xforms;
    };

    struct RigSession {
        SdfPath rigPath;
        SdfPath assetRoot;
        std::shared_ptr<RigExecSnapshotStore> store;
        std::unique_ptr<RigExecImagingBridge> bridge;
        /// Set when the rig plays a .rigexec instead of evaluating:
        /// exactly one of bridge/playback is ever set (see Activate).
        std::unique_ptr<RigExecBakedPlayback> playback;
        RigExecBindingResolvingSceneIndex::BindingEpochConstPtr epoch;
        std::set<SdfPath> readRoots;
        bool readRootsDirty = true;
        bool dirty = true;
        size_t evaluationCount = 0;
        // The session's single-entry frozen snapshot cache, keyed by the
        // program object, the binding epoch, and the patchable avar
        // region's digest. Refreshed once per warming burst, in
        // _PrepareWarmBurst (the live state it pins is the last evaluated
        // frame's, so the pinned history is the frame's), and bound per
        // job in BuildWarmWork, which keeps a shared_ptr alive past
        // whatever the session does next. Null with frozenValid set means
        // validly empty: no program (D7), or a rig the freeze refuses --
        // production jobs decline and live serves.
        const RigExecBakedProgram *frozenProgram = nullptr;
        uint64_t frozenEpoch = 0;
        uint64_t frozenAvarDigest = 0;
        bool frozenValid = false;
        /// The stage-edit serial the standing snapshot was verified under.
        /// A still serial means no notice landed since, so the refresh
        /// skips its chain-currency walk and avar-region digest --
        /// bindings and avars move only under notices, and the program,
        /// epoch, and override pins are compared cheaply regardless.
        /// Impossible until the first refresh verifies one.
        uint64_t frozenSerial = ~uint64_t(0);
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        /// The named cause of the standing snapshot's freeze refusal, empty
        /// when the last refresh froze (or had no program to freeze): a
        /// refusal parks a validly empty entry AND says why, so the
        /// factory's freeze-refused skips report the cause instead of
        /// swallowing it.
        std::string frozenError;
        /// The session's standing warm burst: rebuilt when its pins move --
        /// the program object, the cache-epoch digest (binding epoch +
        /// build count + avar region), and the interactive overrides it
        /// was built under -- and served across ticks while current, so a
        /// second consecutive trigger pays no rebuild. A pins-determined
        /// unusable outcome latches the same way: overrun (or a shape
        /// decline) under the current pins is re-served, not rebuilt --
        /// the build is pure in the pins, so a rebuild would be a doomed
        /// repeat. usable=false with overrun set means the last rebuild
        /// exceeded the prep slice: the tick takes the plain per-frame
        /// route. usable=false with declined set is a shape decline:
        /// the tick takes no factory at all. usable=false with neither
        /// is a transient (bindings refresh failed): the next trigger
        /// retries.
        RigExecBurstSampleCache standingBurst;
        const RigExecBakedProgram *burstProgram = nullptr;
        uint64_t burstEpochDigest = 0;
        std::vector<RigExecValueOverride> burstOverrides;
        bool burstOverrun = false;
        bool burstDeclined = false;
        /// The session's last reasoned factory skip and its detail (the
        /// freeze cause or the sampler gate), for tests and the strip.
        /// Prep records the session-level verdict on its null-factory
        /// paths (D7 / chain-bind / burst-declined). Unreasoned skips
        /// and successful builds leave it standing.
        RigExecWarmSkipReason lastWarmSkip = RigExecWarmSkipReason::None;
        std::string lastWarmSkipDetail;
        /// The persistent full-range cursor: an explicit frame list
        /// (sorted, de-duplicated) visited closest-first from the playhead
        /// across triggers, skipping visited and un-warmable frames
        /// without sampling. Unset rigs warm the default
        /// playhead-relative sweep instead.
        std::vector<double> warmRange;
        bool warmRangeActive = false;
        // The session's epoch-pinned chain bindings, refreshed with the
        // snapshot and verified per burst (a constant edited mid-epoch
        // moves no digest, so the pins are re-read, not trusted -- once,
        // at prepare, since no notice can interleave a burst's frames).
        // UI thread only: the pins hold live stage handles.
        RigExecChainSampleBindings chainBindings;
        bool chainBindingsValid = false;
        /// Frames a scoped retirement dirtied (plan 2.2 lane c), awaiting
        /// re-warm: the next commit/idle trigger enqueues these FIRST, so
        /// the affected-time set replaces the fixed sweep for the edit.
        /// A trigger drops the cached ones and keeps the rest (warming
        /// ones unprepended, so a budget cut never loses a frame); capped,
        /// past which the cursor sweep covers the overflow (dirtied
        /// frames stay visitable). Times only: the trigger skips the
        /// playhead.
        std::vector<double> pendingRewarm;
    };

    using RigSessions = std::vector<RigSession>;

    bool _EvaluateSessions(
        RigSessions *sessions,
        const UsdStageRefPtr &stage,
        UsdTimeCode time,
        std::shared_ptr<RigExecImagingSnapshot> *snapshot,
        RigExecBindingResolvingSceneIndex::BindingEpochConstPtr *epoch,
        std::vector<std::string> *errors);

    /// Brings one session's frozen snapshot cache current: re-freezes when
    /// the program object or epoch moved, copy-on-write patches the avar
    /// region when only constants drifted, rebinds the chain pins when only
    /// they did, and leaves a current entry alone. A freeze refusal (or no
    /// program) parks a validly empty entry: production jobs decline and
    /// live serves. UI thread, _mutex held.
    void _RefreshFrozenSnapshot(RigSession *session);

    /// Returns the session's standing warm burst, rebuilding it when its
    /// pins moved (program, cache-epoch digest, overrides) and serving it
    /// across ticks while current. A rebuild past the prep slice parks the
    /// burst unusable and marks the overrun, so the tick takes the plain
    /// per-frame route; a shape decline parks it unusable without the mark,
    /// so the tick takes no factory. Null only when there is no session,
    /// bridge, or playback to prepare for. The pointer names session
    /// storage: the trigger's factory captures it for synchronous,
    /// same-thread use inside the trigger only. UI thread, _mutex held.
    /// Cancels the rig's warming generation AND pushes the new token to
    /// the warm-frame index, so memoize stamps and dirty checks read the
    /// bump. Every cancel flows through here; a direct scheduler cancel
    /// would leave the index's generation behind. Call with _mutex held.
    void _CancelGenerationLocked(const SdfPath &rig);

    /// Scoped cancellation for patch/stamp-bump edits (plan 2.1): purges
    /// the rig's queued jobs for its completed/queued times and assigns
    /// fresh per-time fence tokens, WITHOUT bumping the generation, and
    /// marks those completions path-scoped dirty in the warm index. Call
    /// with _mutex held.
    void _CancelGenerationTimesLocked(const SdfPath &rig);

    /// Path-scoped retirement for one session's patch/stamp-bump edit
    /// (plan 2.1/2.2): maps the notice to dirty clusters through the
    /// session's epoch index, partitions completions by entry provenance
    /// (clean entries stand or carry, dirty ones retire), purges ONLY the
    /// affected/carried/queued times, retires intersecting proofs, and
    /// records lane-c frames for the next trigger. \p patchedPaths is the
    /// evaluator's exact patched set (Patched only; empty otherwise).
    /// Falls back to _CancelGenerationTimesLocked when the session has no
    /// program or no usable index. Call with _mutex held.
    void _RetireAffectedTimesLocked(
        RigSession *session, const UsdNotice::ObjectsChanged &notice,
        RigExecNoticeDisposition disposition,
        const std::vector<SdfPath> &patchedPaths);

    /// Records \p times (time values) for first-priority re-warm on the
    /// next trigger, de-duplicated and capped. Past the cap the overflow
    /// is dropped: dirtied frames stay visitable, so the cursor sweep
    /// covers them. Call with _mutex held.
    void _NotePendingRewarm(RigSession *session,
                            const std::vector<double> &times);

    /// Moves the session's pending re-warm times that still need work
    /// to the FRONT of \p sweepTimes (closest-first among themselves).
    /// Cached times drop out of the list (re-warmed); warming times stay
    /// listed but are not prepended (a job is already coming -- prepending
    /// would double-enqueue past the popped running job); dirty and
    /// uncached times prepend and stay listed until a later trigger sees
    /// them cached, so a budget cut never loses a frame. The playhead is
    /// never queued, however it arises. Call with _mutex held.
    void _PrependPendingRewarm(RigSession *session, UsdTimeCode playhead,
                               std::vector<UsdTimeCode> *sweepTimes);

    /// Ordered sweep candidates for the session: the warm range
    /// closest-first from the playhead (visited and un-warmable frames
    /// skipped without sampling) when set, else the default
    /// playhead-relative sweep. Call with _mutex held.
    std::vector<UsdTimeCode> _CursorSweepTimes(const RigSession &session,
                                               UsdTimeCode playhead);

    RigExecBurstSampleCache *_PrepareWarmBurst(RigSession *session);

    RigExecImagingBridge::PublishResult _Publish(
        std::shared_ptr<RigExecImagingSnapshot> snapshot,
        const RigExecBindingResolvingSceneIndex::BindingEpochConstPtr &epoch);

    /// Broadcasts a publication to every chain. Call WITHOUT _mutex held:
    /// the sends re-enter this registry (the results index's time trigger),
    /// so _Broadcast snapshots the chain list under a short lock and sends
    /// outside it.
    void _Broadcast(const RigExecImagingBridge::PublishResult &result);
    void _RefreshReadRoots();

    /// Edit-driven re-evaluation: any authored change touching the rig's
    /// asset or transitive external read dependencies re-evaluates at the
    /// last-set time and republishes, so
    /// property edits redraw exactly like timeline changes.
    void _OnObjectsChanged(
        const UsdNotice::ObjectsChanged &notice,
        const UsdStageWeakPtr &sender);

    /// One declared value of a manipulation in progress; see BeginPreview.
    struct PreviewSlot {
        SdfPath primPath;
        TfToken attributeName;
        /// The rig this attribute previews through, or empty for the xform
        /// lane. Resolved once, at declaration: a drag does not change which
        /// rig a prim belongs to.
        SdfPath rigPath;
        /// double | float | vec3d | vec3f | matrix4d -- how to turn this
        /// slot's doubles back into the attribute's own type.
        TfToken valueKind;
        size_t arity = 1;
    };

    /// Turns one sample's doubles into the per-rig override vectors and the
    /// per-prim xform deltas. Stage reads happen here, off the Hydra thread.
    bool _ResolvePreviewSample(
        const double *values, size_t count,
        std::map<SdfPath, std::vector<RigExecValueOverride>> *byRig,
        std::map<SdfPath, GfMatrix4d> *xformDeltas) const;

    /// The composed world delta for \p primPath given its overridden ops.
    bool _ComposeXformDelta(
        const UsdPrim &prim,
        const std::map<TfToken, VtValue> &opValues,
        UsdGeomXformCache *cache,
        GfMatrix4d *delta) const;

    void _SetChainXformDeltas(const std::map<SdfPath, GfMatrix4d> &deltas);

    std::mutex _mutex;

    /// Millisecond slice for one standing-burst rebuild; see
    /// SetWarmBurstPrepSliceMs. UI thread, _mutex held.
    double _warmBurstPrepSliceMs = 50.0;
    std::shared_ptr<RigExecSnapshotStore> _store;
    std::vector<Chain> _chains;
    RigSessions _sessions;
    UsdStageRefPtr _stage;
    /// Rig roots sighted by the rig adapter (see NoteRigRoot).
    ///
    /// Its own mutex, SEPARATE from _mutex on purpose: the adapter notes
    /// roots from inside scene index traversals that run while _mutex is
    /// held, and taking _mutex there is a self-deadlock (MSVC throws
    /// device_or_resource_busy). Lock order is _mutex THEN _notedMutex --
    /// Activate and Deactivate clear the set while holding _mutex -- and
    /// nothing ever takes _mutex while holding _notedMutex.
    std::mutex _notedMutex;
    std::set<SdfPath> _notedRigRoots;
    std::set<SdfPath> _generatedScopes;
    std::set<SdfPath> _assetRoots;
    std::set<SdfPath> _readRoots;
    bool _readRootsDirty = false;
    UsdTimeCode _lastTime = UsdTimeCode::Default();
    /// A manipulation in progress: the declared slots, and the xform-lane
    /// deltas currently standing (kept here as well as on the chains so a
    /// chain built mid-drag starts in step -- see RegisterChain).
    std::vector<PreviewSlot> _previewSlots;
    std::map<SdfPath, GfMatrix4d> _previewXformDeltas;
    bool _previewActive = false;
    /// The influence-overlay selection, held HERE rather than only on the
    /// bridge because it outlives one: a host may select before
    /// activation, and Deactivate/Activate must not silently drop it.
    SdfPath _weightOverlay;
    uint64_t _generation = 0;
    uint64_t _publishedEpochId = 0;
    /// The frame-warming pool: below-normal workers draining neighbor and
    /// sweep jobs sampled on the UI thread. Its mutex is a LEAF: triggers
    /// take _mutex first and scheduler state second, and workers never take
    /// _mutex at all, so no path inverts the order.
    std::unique_ptr<RigExecBackgroundScheduler> _scheduler;
    /// The shared warm-frame index: completions (worker closures,
    /// memoized publishes), evictions (cache callbacks), and queue
    /// transitions (scheduler hook) record here; the strip queries it.
    /// A leaf under both _mutex and the scheduler mutex.
    std::shared_ptr<RigExecWarmFrameIndex> _warmIndex;
    /// The triggers' sampling budget (see SetWarmSamplingBudget).
    size_t _warmSamplingMaxInvocations = 16;
    double _warmSamplingMaxMs = 8.0;
    TfNotice::Key _changeKey;
};

}  // namespace rigExec

// Visibility for the C surface below. Windows needs dllexport while building
// rigExecImaging and dllimport when a consumer includes this header -- the
// build defines RIGEXEC_IMAGING_EXPORTS to tell the two apart. Elsewhere the
// ELF/Mach-O equivalent is default visibility, which also keeps the symbols
// alive if the tree is ever built with -fvisibility=hidden.
#if defined(_WIN32)
#  if defined(RIGEXEC_IMAGING_EXPORTS)
#    define RIGEXEC_IMAGING_C_API __declspec(dllexport)
#  else
#    define RIGEXEC_IMAGING_C_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define RIGEXEC_IMAGING_C_API __attribute__((visibility("default")))
#else
#  define RIGEXEC_IMAGING_C_API
#endif

// C surface for language-neutral activation (e.g. the usdview Python
// plugin via ctypes + UsdUtilsStageCache ids). Returns 0 on success.
extern "C" {

RIGEXEC_IMAGING_C_API int RigExecImaging_Activate(
    long long stageCacheId, const char *rigPath, double initialFrame);
RIGEXEC_IMAGING_C_API int RigExecImaging_SetTime(double frame);
/// The primary warming trigger for language-neutral hosts (see
/// RigExecImagingRegistry::OnEditCommitted): call on drag release / value
/// commit. Returns 0 on success, non-zero when nothing is active.
RIGEXEC_IMAGING_C_API int RigExecImaging_OnEditCommitted();
/// The secondary warming trigger (see RigExecImagingRegistry::OnIdle):
/// call from the frame loop when the UI is idle. Returns 0 on success,
/// non-zero when nothing is active.
RIGEXEC_IMAGING_C_API int RigExecImaging_OnIdle();
/// Per-phase viewport profile totals to a text file, then cleared. Needs
/// RIGEXEC_IMAGING_PROFILE set when the rig was activated.
RIGEXEC_IMAGING_C_API int RigExecImaging_WriteProfileSummary(const char *path);
RIGEXEC_IMAGING_C_API void RigExecImaging_Deactivate();
/// Current published snapshot generation (0 before first publication).
RIGEXEC_IMAGING_C_API long long RigExecImaging_GetGeneration();

/// Reads one complete evaluated control matrix from the exact published
/// stage/time. Returns 1 on success, 0 without changing output otherwise.
/// isDefault selects UsdTimeCode::Default; ordinary frames must be finite.
/// Reads scalar properties this generation published, by property path.
///
/// \p packedPaths is newline-separated absolute property paths, the same
/// convention BeginPreview uses; \p out receives one float each and must
/// hold \p count of them. Returns how many were FOUND; a path the rig did
/// not publish leaves its slot at 0 and is not counted, so a caller can
/// tell "everything is zero" from "nothing was published".
///
/// This exists so a tool does not have to evaluate the rig a second time to
/// see numbers the viewport already computed. The Shape Editor was doing
/// exactly that -- 9-15 ms per refresh to recover pose-interpolator weights
/// sitting in the current snapshot.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetMovedFloats(
    const char *packedPaths, float *out, int count);

RIGEXEC_IMAGING_C_API int RigExecImaging_GetControlFrameAssetSpace(
    long long stageCacheId, const char *primPath, double frame,
    int isDefault, double outMatrix[16]);

/// ASSET-SPACE axis-aligned bounds of everything the current generation
/// draws for \p primPath, as min xyz then max xyz in \p outMinMax.
/// Returns 1 when the prim draws something, 0 otherwise (\p outMinMax is
/// then untouched).
///
/// This exists because a rig draws nothing a bounding box can be computed
/// from the ordinary way. Guides are synthesized inside the imaging chain
/// and never authored, and the RigExec prim types are not UsdGeomImageable,
/// so UsdGeomBBoxCache -- which is what usdview's frame-selection goes
/// through -- correctly reports an empty box for every joint, control, and
/// solver on the stage. Framing a control therefore moved the camera
/// nowhere. The snapshot is the only place the drawn extent exists, so the
/// answer has to come from here.
///
/// Asset space, not world: guide frames carry no stage placement (see
/// RigExecImagingSnapshot::assetRoot), so the caller composes the asset
/// root's own world transform. No Hydra dependency -- snapshot data only.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetGuideBoundsAssetSpace(
    const char *primPath, double outMinMax[6]);

/// The union of RigExecImaging_GetGuideBoundsAssetSpace over every prim in
/// the current generation, so framing one rig frames its whole guide set.
/// Returns 0 when guides belong to multiple asset roots because their
/// asset-space ranges cannot be combined before applying distinct root
/// transforms. Returns 1 when anything at all draws, 0 otherwise.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetAllGuideBoundsAssetSpace(
    double outMinMax[6]);

/// Paints \p weightPrimPath's resolved weight field onto the geometry it
/// weights, as a grey-to-red vertex gradient in the viewport (the R&H
/// "Voodoo" influence display). Null or empty turns the overlay off.
/// Returns 0 on success, non-zero on failure.
///
/// This exists because a weight volume is invisible and its effect is only
/// legible after the fact: a rigger placing one is otherwise reading a
/// deformation and inferring the region that caused it. The overlay shows
/// the region directly, and shows the field a mover ACTUALLY consumed
/// rather than a re-derivation of it.
///
/// Republishes at the current time before returning, so the viewport
/// updates immediately rather than at the next frame change.
RIGEXEC_IMAGING_C_API int RigExecImaging_SetWeightOverlay(
    const char *weightPrimPath);

/// Per-frame warming states over an explicit frame list, for the cache
/// strip: one call per repaint, no sampling on the query path. \p frames
/// holds \p count frame numbers; \p statesOut receives one state each
/// (0 uncached, 1 warming, 2 cached, 3 dirty) and must hold \p count
/// ints. Returns how many states were written; -1 on bad arguments or an
/// unknown/inactive rig.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetFrameStates(
    const char *rigPath, const double *frames, int *statesOut, int count);

/// Drops one rig's cached frames, freshness proofs, and warm-frame index
/// (the strip's clear action; test isolation). Unknown rigs answer 0:
/// nothing to drop is still dropped. Returns 0 on success.
RIGEXEC_IMAGING_C_API int RigExecImaging_ClearFrameCache(const char *rigPath);

/// Sets the rig's persistent warm range (see SetWarmRange): the full-range
/// cursor visits these frames closest-first from the playhead across idle
/// triggers instead of the default playhead-relative sweep. An empty list
/// warms nothing. Returns 0 on success; -1 on bad arguments or an
/// unknown rig.
RIGEXEC_IMAGING_C_API int RigExecImaging_WarmRange(
    const char *rigPath, const double *frames, int count);

/// Lifetime warming completions ("ran", see
/// RigExecBackgroundSchedulerStats::completed): the cache strip's repaint
/// gate. A poll whose counter is still, with unchanged states and playhead,
/// skips the repaint. Never negative; 0 when the scheduler is off.
RIGEXEC_IMAGING_C_API long long RigExecImaging_GetWarmingCompletedCount();

}

#endif  // RIGEXEC_IMAGING_REGISTRY_H
