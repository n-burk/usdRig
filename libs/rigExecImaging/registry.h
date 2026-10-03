// RigExec imaging registry: the rendezvous between the UsdImaging
// scene-index plugin (which builds filter chains when engines construct)
// and the application activation (which owns the stage, evaluator, and time
// source). Order-independent: chains read their context's snapshot store
// atomically, so activation may happen before or after chain construction.
// PER-STAGE CONTEXTS (docs/multistage-imaging.md). One
// RigExecImagingRegistry object is one IMAGING CONTEXT: everything that
// used to be process-global -- the snapshot store, the bound chains, the
// rig sessions and their stage, noted roots, generated scopes, asset and
// read roots, the last time, preview slots and deltas, the weight overlay,
// the generation and published epoch, the warming scheduler, warm index
// and budgets, and the stage-notice key -- belongs to exactly one UsdStage.
// Several usdview sessions in one process therefore evaluate their own
// rigs side by side, including two stages opened from the same file
// (identical prim paths).
// THE DIRECTORY maps a stage to its context (ForStage / ForStageCacheId /
// ForKey). It is keyed by stage IDENTITY -- the UsdStage object, validated
// through a UsdStageWeakPtr, so an expired stage never aliases a new one
// allocated at the same address -- and every context carries a
// monotonically increasing KEY (never the raw pointer) that is what
// anything outliving the stage holds. A context whose stage has expired is
// dropped from the directory on the next directory call; an active context
// holds its stage, so only inactive contexts ever expire. An AUTOMATIC
// activation (the rig adapter's EnsureActivated) is released by the library
// itself once the last chain bound to its context goes away (engine
// destroyed, or the stage replaced under it); an explicit activation lasts
// until its host deactivates it.
// CHAIN -> CONTEXT BINDING (the sanctioned transport; there is no stage at
// AppendSceneIndex time). The scene-index plugin registers every new chain
// with the directory UNBOUND (RegisterUnboundChain): an unbound chain reads
// an empty store and receives nothing stage-specific. The keyless rig
// adapter (rigAdapter.cpp) contributes a `rigExec/stageKey` leaf -- a
// HdRetainedTypedSampledDataSource<uint64_t> holding
// ForStage(prim.GetStage())->GetKey() -- on TOP-LEVEL prims only, merged
// into the same `rigExec` container that carries the `rigExec/time`
// trigger when a top-level prim is also a rig root. The results scene
// index pulls that leaf from its input on each _PrimsAdded batch that holds
// a top-level prim and, when the key differs from the one it is bound to
// (first population, or a stage replaced under the same engine), asks the
// directory to (re)bind it (BindChain): the chain then reads the context's
// store, pruned scopes, preview deltas and binding epoch, and only that
// context's publications reach it. Same mechanism as the time trigger: a
// data-source leaf the stage scene index already builds, no side channel.
// LEGACY SURFACE. GetInstance() and the stage-less C functions keep their
// signatures. GetInstance() is the process's LEGACY HANDLE: an unbound
// context object that owns no stage and routes every call to the CURRENT
// context -- the one most recently activated through a legacy entry point
// (RigExecImaging_Activate, GetInstance().Activate) or, when no current
// context is live, through the adapter's automatic activation. With no
// current context it answers as an inactive context (an empty store,
// SetTime false, ...). Its Activate(stage) forwards to that stage's
// context -- creating it when the stage has none -- and makes it current.
// A host that activates one stage at a time and deactivates before it
// switches (usdview's plugin, usdrecord, every pre-existing test) sees the
// old singleton's behavior. Differences a legacy caller can observe are
// listed in docs/multistage-imaging.md ("Legacy surface"): a legacy
// activation of a second stage does NOT deactivate the first (both keep
// evaluating; the first is reachable through the ...ForStage functions),
// and automatic activation evaluates every imaged stage instead of
// refusing all but one. Stage-aware callers use ForStage* and the
// ...ForStage C functions (RigExecImaging_ActivateForStage included) and
// never touch the current designation.
// LOCK ORDER (never inverted):
//   scheduler fence mutex (ClearFrameCache/SetWeightOverlay only)
//     -> context _mutex
//       -> context _notedMutex
//       -> scheduler state / warm index / frame cache shards
//       -> DIRECTORY mutex
// The directory mutex is a LEAF: it is taken under a context's _mutex
// (chain snapshots for broadcasts and preview deltas) and alone, and
// nothing is ever called into a context -- and no context is destroyed --
// while it is held. Contexts released by the directory are destroyed after
// it unlocks.
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

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

class RigExecImagingDirectory;

class RigExecImagingRegistry : public TfWeakBase {
public:
    using Ptr = std::shared_ptr<RigExecImagingRegistry>;

    // -- the directory (process level) --------------------------------------

    /// The legacy handle (see the file comment): an unbound context that
    /// routes every call to the current context. Kept for source
    /// compatibility; no library code calls it.
    static RigExecImagingRegistry &GetInstance();

    /// \p stage's context, created on demand when \p create is set. Null for
    /// a null stage, or when the stage has none and \p create is false.
    static Ptr ForStage(const UsdStageRefPtr &stage, bool create = true);

    /// The context of the stage UsdUtilsStageCache::Get() holds under \p id.
    static Ptr ForStageCacheId(long long id, bool create);

    /// The context with \p key, or null when there is none or its stage has
    /// expired (key 0 never names a context).
    static Ptr ForKey(uint64_t key);

    /// The current context the legacy surface routes to, or null.
    static Ptr Current();

    /// Contexts in the directory whose stage is still alive.
    static size_t ContextCount();

    /// Called by the scene-index plugin for each constructed chain: the
    /// chain starts UNBOUND (an empty store, no scopes, no deltas) and
    /// binds on its first population (see the file comment).
    static void RegisterUnboundChain(
        const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
        const RigExecBindingResolvingSceneIndexRefPtr &binding,
        const RigExecResultsSceneIndexRefPtr &results,
        const RigExecXformOverrideSceneIndexRefPtr &xforms = nullptr);

    /// (Re)binds the registered chain whose results index is \p results to
    /// the context with \p key (0 unbinds). A no-op for a chain the
    /// directory does not know (a hand-built chain) or one already bound to
    /// \p key. Called by the results index from _PrimsAdded, never while a
    /// context lock is held.
    static void BindChain(const RigExecResultsSceneIndex *results,
                          uint64_t key);

    /// Forgets the chain whose results index is \p results. Called by that
    /// index's destructor, never while a context lock is held. When it was
    /// the last chain bound to a context whose rig was activated
    /// AUTOMATICALLY (EnsureActivated), that context is deactivated: the
    /// engine that imaged the stage is gone, so nothing would ever release
    /// its stage and worker threads otherwise. The release does not count as
    /// a host deactivation, so a later engine on the same stage activates it
    /// again. Explicit activations are never touched. A no-op for a chain
    /// the directory does not know.
    static void ReleaseChain(const RigExecResultsSceneIndex *results);

    /// This context's key (0 for the legacy handle).
    uint64_t GetKey() const { return _key; }

    /// The stage this context belongs to, or null (the legacy handle, or a
    /// stage that has gone away).
    UsdStageRefPtr GetBoundStage() const;

    // -- the context --------------------------------------------------------

    /// The store every results scene index bound to this context reads
    /// from. On the legacy handle: the current context's store, or the
    /// handle's own (empty) one when there is no current context.
    const std::shared_ptr<RigExecSnapshotStore> &GetStore() const;

    /// Registers a chain BOUND to this context (the legacy handle registers
    /// it unbound), seeding it with this context's store, scopes, deltas and
    /// epoch. The scene-index plugin uses RegisterUnboundChain instead.
    void RegisterChain(
        const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
        const RigExecBindingResolvingSceneIndexRefPtr &binding,
        const RigExecResultsSceneIndexRefPtr &results,
        const RigExecXformOverrideSceneIndexRefPtr &xforms = nullptr);

    /// Activates evaluation for one rig, or every RigExecRoot when rigPath is
    /// empty.  Compilation and the first evaluation complete off to the side;
    /// the active stage and Hydra generation change only after every rig has
    /// succeeded.
    ///
    /// A context only ever evaluates its own stage: called with another
    /// stage, the call is forwarded to that stage's context (created when
    /// the stage has none). On the legacy handle a successful activation
    /// also makes the target the current context.
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
    /// plus the nearest-frame-first sweep of the surrounding range for every
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

    /// Warming counters over the context's lifetime (completions, drops...)
    /// plus the running scheduler's gauges (queue depth, running). The
    /// counters survive Deactivate and re-activation: each stopped
    /// scheduler's totals, as of its deactivation, are retained and summed
    /// with the running one's. Per CONTEXT -- another stage's warming never
    /// counts here. Gauges are zero while no rig is active.
    RigExecBackgroundSchedulerStats GetBackgroundStats();

    /// Blocks until no warming job is queued or running.
    void WaitUntilBackgroundIdle();

    /// Lifetime frame-cache counters for one rig (zeros when the rig is
    /// unknown, a playback session, or simply cold).
    RigExecFrameCacheStats GetFrameCacheStats(const SdfPath &rig);

    /// The warming scheduler's profiler (per-sweep-time factory costs land
    /// here), for benches and tests; null when the scheduler is off (no
    /// active rig). Per ACTIVATION: a re-activation starts a new scheduler
    /// with an empty profiler.
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
    ///
    /// With \p publish false the overrides are dropped and every session is
    /// marked dirty, but nothing is evaluated: for a COMMIT that authors
    /// right after, whose own stage notice evaluates and publishes the
    /// committed rig. Ending first and publishing would flash the pre-drag
    /// pose; ending after the commit evaluated the rig twice -- once from
    /// the notice with the overrides still standing, once here. MEASURED on
    /// the full biped stack: two EvaluateAndPublish passes per release.
    bool EndPreview(bool publish = true);

    bool IsPreviewActive() const;

    /// Drops the bridge and stops the warming scheduler (a context without
    /// an active rig owns no threads); bound chains remain and read the
    /// (cleared) store. Other contexts are untouched. On the legacy handle:
    /// deactivates the current context only. A HOST deactivation: the
    /// automatic path never revives the context afterwards (see
    /// EnsureActivated).
    void Deactivate();

    /// Whether the active rig was activated automatically (EnsureActivated)
    /// rather than by an explicit Activate. False while inactive.
    bool IsAutoActivated() const;

    bool IsActive() const;

    /// Activates the stage's rigs unless they are already active
    /// (adapter-driven discovery, docs/specs/imaging-datasource-redesign.md
    /// §3.2).
    ///
    /// The stage scene index's rig adapter calls this -- on
    /// ForStage(prim.GetStage()) -- the first time a RigExecRoot prim's data
    /// is built, so a host with no explicit activation (usdrecord) still
    /// evaluates. A no-op when this context is already active -- hosts that
    /// activate explicitly (the usdview plugin) are unaffected -- and
    /// otherwise exactly Activate with an empty rig path: every RigExecRoot
    /// on the stage, compiled and published at \p time.
    ///
    /// NOT a substitute for Activate: a rig authored into an already-active
    /// stage still needs an explicit re-activation, which is what the
    /// usdview plugin's root-set tracking does.
    ///
    /// Never refuses on account of another stage: each stage has its own
    /// context, so there is nothing to steal, and a second stage's rigs
    /// evaluate independently. (It used to refuse while the process-global
    /// registry was active on a different stage.) A successful automatic
    /// activation becomes the legacy current context only when no current
    /// context is live, so it never takes the legacy surface away from an
    /// explicit activation.
    ///
    /// Never revives a context its host explicitly Deactivate()d either --
    /// only an explicit Activate does. Otherwise any stray pull of a rig
    /// root (UsdImaging traverses the old stage while a stage scene index
    /// announces a SetStage) would re-activate a closed stage, and its
    /// context would then hold that stage alive for the life of the
    /// process: nothing replaces it the way the old singleton was replaced.
    ///
    /// An activation made HERE is automatic, and the library owns its
    /// lifetime: when the last chain bound to this context goes away (its
    /// engine is destroyed, or a SetStage rebinds it to another stage) the
    /// context deactivates itself and lets go of its stage and threads (see
    /// ReleaseChain). That release is not a host deactivation, so a later
    /// engine on the same stage activates it again. An explicit Activate on
    /// an automatically active context takes the lifetime over: from then
    /// on only the host deactivates it.
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
    /// builds its data but EnsureActivated leaves a live activation alone.
    /// Cleared by Activate and Deactivate; a note that outlives a failed
    /// activation retries on the next resync, like the plugin's own retry.
    ///
    /// Paths only: the context IS the stage (the adapter notes on
    /// ForStage(prim.GetStage())), and the chains that match notes are the
    /// ones bound to this context, so a path collision across two stages
    /// never crosses between them.
    void NoteRigRoot(const SdfPath &rigPath);

    /// Whether \p path was noted by NoteRigRoot and is still pending.
    bool IsNotedRigRoot(const SdfPath &path);

    ~RigExecImagingRegistry();

private:
    friend class RigExecImagingDirectory;

    /// A context for \p stage under \p key (the directory creates these).
    RigExecImagingRegistry(uint64_t key, const UsdStageRefPtr &stage);

    /// The legacy handle (GetInstance).
    struct _LegacyHandleTag {};
    explicit RigExecImagingRegistry(_LegacyHandleTag);

    RigExecImagingRegistry(const RigExecImagingRegistry &) = delete;
    RigExecImagingRegistry &operator=(const RigExecImagingRegistry &) = delete;

    // Weak references: chains are owned by their scene index graphs and
    // die with their engines; the directory prunes expired entries.
    struct Chain {
        TfWeakPtr<RigExecInternalPrimPruningSceneIndex> pruning;
        TfWeakPtr<RigExecBindingResolvingSceneIndex> binding;
        TfWeakPtr<RigExecResultsSceneIndex> results;
        TfWeakPtr<RigExecXformOverrideSceneIndex> xforms;
    };

    /// On the legacy handle, the context a call routes to: the current
    /// context, or null (the call then answers from the handle's own
    /// inactive state). Always null on a real context.
    Ptr _Routed() const;

    /// Activate, recording whether the activation is \p automatic
    /// (EnsureActivated) or explicit. An automatic call re-checks under
    /// _mutex and is a no-op (true) when the context became active meanwhile
    /// and refused (false) when its host deactivated it.
    bool _Activate(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        UsdTimeCode initialTime, std::vector<std::string> *errors,
        bool automatic);

    /// Deactivate. \p byHost marks a host deactivation (Deactivate): the
    /// automatic path never revives it. Otherwise it is the library's
    /// release of an automatic activation (ReleaseChain / BindChain), which
    /// only proceeds -- re-checked under _mutex -- while the activation is
    /// still automatic and no chain is bound to this context.
    void _Deactivate(bool byHost);

    /// The context with \p key releases its automatic activation if no
    /// chain is bound to it any more (see _Deactivate). Call with no lock
    /// held.
    static void _ReleaseIfUnbound(uint64_t key);

    /// Legacy activation (the handle's Activate and RigExecImaging_Activate):
    /// forwards to \p stage's context, created on demand and seeded with
    /// the legacy overlay selection, and makes it current on success.
    static bool _LegacyActivate(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        UsdTimeCode initialTime, std::vector<std::string> *errors);

    /// The chains bound to this context, live ones only (a snapshot taken
    /// under the directory's leaf lock).
    std::vector<Chain> _BoundChains() const;

    /// Seeds a chain that just bound to this context with its store,
    /// scopes, preview deltas and epoch. Takes _mutex briefly; the sends
    /// run unlocked (they re-enter through the results index's trigger).
    void _AdoptChain(const Chain &chain);

    /// The stage a preview resolves attributes on: the active stage, else
    /// the bound one (a stage with no rig still previews its Xforms).
    UsdStageRefPtr _PreviewStageLocked() const;

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
        /// The bridge's binding generation chainBindings was copied at
        /// (RigExecImagingBridge::AcquireChainBindings): the copy refreshes
        /// when it moves, and the standing burst rebuilds with it.
        uint64_t chainBindingsGeneration = 0;
        /// The overrides EndPreview(publish=false) withdrew while the
        /// published generation still showed them, for a commit about to
        /// author them. A notice whose every edit is one of these
        /// attributes, after which the stage holds every one of these
        /// values, leaves the published pose exact: the session stays
        /// clean instead of evaluating it again. Dropped by any
        /// evaluation and by any notice that does not settle it.
        std::vector<RigExecValueOverride> settleOverrides;
        uint64_t burstChainGeneration = 0;
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
    /// The session's chain pins, taken from its bridge -- the bindings the
    /// live sampler uses, kept current by the bridge's notice tracking --
    /// and copied only when the bridge has rebound since. False, pins
    /// marked invalid, when the rig's chains cannot be bound.
    bool _AdoptBridgeChainBindings(RigSession *session);
    /// Whether \p notice only authors \p session's settleOverrides and
    /// leaves the stage holding every one of them at _lastTime. _mutex held.
    bool _NoticeSettlesPreview(const RigSession &session,
                               const UsdNotice::ObjectsChanged &notice) const;

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

    /// The scheduler under _mutex, shared so a trigger or a fenced clear
    /// can keep using it after unlocking while a Deactivate stops it: the
    /// last holder destroys it (which joins its workers) outside any lock.
    std::shared_ptr<RigExecBackgroundScheduler> _SchedulerLocked() const {
        return _scheduler;
    }

    /// The directory key (0 on the legacy handle) and the stage this
    /// context belongs to (weak: a context never keeps its stage alive
    /// unless it is active, see _stage).
    const uint64_t _key = 0;
    const UsdStageWeakPtr _boundStage;
    const bool _legacyHandle = false;

    /// Mirrors !_sessions.empty(), readable without _mutex (IsActive, and
    /// the directory's live-current test, which must not lock a context).
    std::atomic<bool> _active{false};

    /// Set by an explicit Deactivate, cleared by the next successful
    /// Activate: the automatic path (EnsureActivated) never revives a stage
    /// its host turned off. _mutex held.
    bool _deactivatedByHost = false;

    /// Set by a successful automatic activation (EnsureActivated), cleared
    /// by an explicit Activate and by every deactivation: the library, not
    /// the host, releases an automatic activation once its last bound chain
    /// is gone. _mutex held.
    bool _autoActivated = false;

    /// The totals of the schedulers this context stopped, as of their
    /// deactivation (GetBackgroundStats adds the running one's). Gauges
    /// (queue depth, running) stay zero here. _mutex held.
    RigExecBackgroundSchedulerStats _retiredStats;

    mutable std::mutex _mutex;

    /// Millisecond slice for one standing-burst rebuild; see
    /// SetWarmBurstPrepSliceMs. UI thread, _mutex held.
    double _warmBurstPrepSliceMs = 50.0;
    /// Never reassigned after construction: GetStore() reads it unlocked
    /// (the compute-extent callback runs on arbitrary threads).
    const std::shared_ptr<RigExecSnapshotStore> _store;
    RigSessions _sessions;
    /// The active stage (strong while active, reset by Deactivate).
    UsdStageRefPtr _stage;
    /// The stage a preview in progress resolves on (see BeginPreview).
    /// Weak: the directory holds this context until its stage dies, so a
    /// preview abandoned mid-drag must not keep that stage alive.
    UsdStageWeakPtr _previewStage;
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
    /// On the legacy handle: the legacy selection, which a legacy
    /// activation carries into its target (the old singleton's
    /// process-wide viewer mode); _legacyOverlaySet says one was made.
    SdfPath _weightOverlay;
    bool _legacyOverlaySet = false;
    uint64_t _generation = 0;
    uint64_t _publishedEpochId = 0;
    /// The binding epoch last announced to the bound chains, for seeding
    /// a chain that binds later (_AdoptChain).
    RigExecBindingResolvingSceneIndex::BindingEpochConstPtr _publishedEpoch;
    /// The frame-warming pool: below-normal workers draining neighbor and
    /// sweep jobs sampled on the UI thread. Its mutex is a LEAF: triggers
    /// take _mutex first and scheduler state second, and workers never take
    /// _mutex at all, so no path inverts the order. Created when the
    /// context first activates and stopped by Deactivate: a context without
    /// an active rig owns no threads. Jobs are keyed by rig path WITHIN the
    /// context's own scheduler, so equal paths on two stages never meet.
    std::shared_ptr<RigExecBackgroundScheduler> _scheduler;
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

/// Warming completions ("ran", see
/// RigExecBackgroundSchedulerStats::completed) over the current context's
/// lifetime -- they survive Deactivate and re-activation -- the cache
/// strip's repaint gate. A poll whose counter is still, with unchanged
/// states and playhead, skips the repaint. Never negative; 0 before the
/// context's first activation (and with no current context).
RIGEXEC_IMAGING_C_API long long RigExecImaging_GetWarmingCompletedCount();

/// Manipulation preview (see RigExecImagingRegistry::BeginPreview /
/// UpdatePreview / EndPreview). BeginPreview returns the double count
/// UpdatePreview expects, or -1; the other two return 0 on success.
RIGEXEC_IMAGING_C_API int RigExecImaging_BeginPreview(
    const char *packedAttributePaths);
RIGEXEC_IMAGING_C_API int RigExecImaging_UpdatePreview(
    const double *values, int count);
RIGEXEC_IMAGING_C_API int RigExecImaging_EndPreview();
/// EndPreview without the republish, for a commit that authors next (see
/// RigExecImagingRegistry::EndPreview).
RIGEXEC_IMAGING_C_API int RigExecImaging_EndPreviewWithoutPublish();

// Stage-scoped surface (docs/multistage-imaging.md).
// Every per-stage entry point above has a ...ForStage twin taking the stage's
// UsdUtilsStageCache id FIRST and otherwise the same arguments and return
// convention. A twin acts on THAT stage's context only -- several stages
// evaluate side by side in one process -- and never changes the legacy
// current context. The stage-less functions above keep routing to the
// current context (the one most recently activated through
// RigExecImaging_Activate). An unknown id, or a stage without a context,
// answers exactly as an inactive registry would. SetWeightOverlayForStage
// and BeginPreviewForStage create the stage's context when it has none (a
// selection is remembered before activation; a stage with no rig still
// previews its Xforms); the others never create one.
// RigExecImaging_Activate and RigExecImaging_GetControlFrameAssetSpace
// already take the id and use that stage's context; Activate also makes that
// context current, RigExecImaging_ActivateForStage does not.

/// RigExecImaging_Activate on THAT stage's context (created when it has
/// none) without making it the legacy current context, and without the
/// legacy overlay selection (a stage's own selection is
/// SetWeightOverlayForStage). An explicit activation: it lasts until
/// DeactivateForStage. Same arguments and return codes as
/// RigExecImaging_Activate (0 success, 1 no stage, 2 bad rig path,
/// 3 activation failed).
RIGEXEC_IMAGING_C_API int RigExecImaging_ActivateForStage(
    long long stageCacheId, const char *rigPath, double initialFrame);

RIGEXEC_IMAGING_C_API int RigExecImaging_SetTimeForStage(
    long long stageCacheId, double frame);
RIGEXEC_IMAGING_C_API int RigExecImaging_OnEditCommittedForStage(
    long long stageCacheId);
RIGEXEC_IMAGING_C_API int RigExecImaging_OnIdleForStage(
    long long stageCacheId);
RIGEXEC_IMAGING_C_API int RigExecImaging_WriteProfileSummaryForStage(
    long long stageCacheId, const char *path);
RIGEXEC_IMAGING_C_API void RigExecImaging_DeactivateForStage(
    long long stageCacheId);
RIGEXEC_IMAGING_C_API long long RigExecImaging_GetGenerationForStage(
    long long stageCacheId);
RIGEXEC_IMAGING_C_API int RigExecImaging_GetMovedFloatsForStage(
    long long stageCacheId, const char *packedPaths, float *out, int count);
RIGEXEC_IMAGING_C_API int RigExecImaging_GetGuideBoundsAssetSpaceForStage(
    long long stageCacheId, const char *primPath, double outMinMax[6]);
RIGEXEC_IMAGING_C_API int RigExecImaging_GetAllGuideBoundsAssetSpaceForStage(
    long long stageCacheId, double outMinMax[6]);
RIGEXEC_IMAGING_C_API int RigExecImaging_SetWeightOverlayForStage(
    long long stageCacheId, const char *weightPrimPath);
RIGEXEC_IMAGING_C_API int RigExecImaging_GetFrameStatesForStage(
    long long stageCacheId, const char *rigPath, const double *frames,
    int *statesOut, int count);
RIGEXEC_IMAGING_C_API int RigExecImaging_ClearFrameCacheForStage(
    long long stageCacheId, const char *rigPath);
RIGEXEC_IMAGING_C_API int RigExecImaging_WarmRangeForStage(
    long long stageCacheId, const char *rigPath, const double *frames,
    int count);
RIGEXEC_IMAGING_C_API long long RigExecImaging_GetWarmingCompletedCountForStage(
    long long stageCacheId);
RIGEXEC_IMAGING_C_API int RigExecImaging_BeginPreviewForStage(
    long long stageCacheId, const char *packedAttributePaths);
RIGEXEC_IMAGING_C_API int RigExecImaging_UpdatePreviewForStage(
    long long stageCacheId, const double *values, int count);
RIGEXEC_IMAGING_C_API int RigExecImaging_EndPreviewForStage(
    long long stageCacheId);
RIGEXEC_IMAGING_C_API int RigExecImaging_EndPreviewWithoutPublishForStage(
    long long stageCacheId);

/// 1 when the stage's context has an active rig, 0 otherwise (including an
/// unknown id).
RIGEXEC_IMAGING_C_API int RigExecImaging_IsActiveForStage(
    long long stageCacheId);

/// How many imaging contexts the process holds for live stages (tests and
/// diagnostics).
RIGEXEC_IMAGING_C_API int RigExecImaging_ContextCount();

}

#endif  // RIGEXEC_IMAGING_REGISTRY_H
