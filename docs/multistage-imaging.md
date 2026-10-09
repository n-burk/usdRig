# Multi-stage RigExec imaging

Several stages can be imaged and evaluated in one process, each on its own.
The typical case is several usdview sessions in one host process (usdOrchestrate
SPEC 14). Two stages opened from the same file (identical prim paths) are the
hardest case, and they do not interfere. Before this change the imaging registry
was one process-global object: one stage, one snapshot store, one chain list,
and one preview. Only the most recent activation evaluated, and every other
viewport showed its rest pose.

Code: `libs/rigExecImaging/registry.{h,cpp}`, `sceneIndices.{h,cpp}`,
`sceneIndexPlugin.cpp`, `rigAdapter.cpp`, `touchPose*.{h,cpp}`; for the usdview
plugins, `plugin/rigExecUsdview/imagingHandle.py` and `sessionRegistry.py`
(see "usdview plugins").
Tests: `tests/testRigExecImagingMultiStage.cpp`, and the headless
`tests/python/test_imaging_handle.py` and `tests/python/test_session_state.py`.

## Contexts

One `RigExecImagingRegistry` object is one **imaging context**, and each
context belongs to exactly one `UsdStage`. The following state used to be
process-global and is now per context:

| state | notes |
|---|---|
| snapshot store | read by the context's bound chains, never swapped |
| rig sessions and the active `_stage` | the stage is held strongly while active |
| stage-notice key (`_OnObjectsChanged`) | registered on the context's own stage |
| noted rig roots, generated scopes, asset roots, read roots | |
| `_lastTime`, generation, published epoch (id and pointer) | |
| preview slots, preview stage (weak), xform deltas | only this context's chains receive the deltas |
| weight overlay selection | remembered before activation |
| background scheduler | created on the first activation, stopped by every deactivation |
| warming counters (`GetBackgroundStats`) | per context, kept across deactivations (see below) |
| warm-frame index, warm budgets, burst prep slice | |

**A context without an active rig owns no threads.** The scheduler, with two
workers by default, starts when the context commits its first activation. It
is released by `Deactivate`, and its last holder joins the workers outside
every lock. Triggers and fenced clears hold a `shared_ptr` to the scheduler
across their unlocked sections. The scheduler keys jobs by rig path within its
own context, so equal paths on two stages never share a queue, generation or
fence.

**Warming counters outlive the scheduler.** `GetBackgroundStats` and
`GetWarmingCompletedCount[ForStage]` count over the context's lifetime, not
one activation's. A deactivation adds the stopped scheduler's totals, read
when it deactivates, to a per-context accumulator, and the running
scheduler's counts are added on top. Completions of jobs that were still
running at that moment are not counted. The gauges (queue depth, running)
come from the running scheduler only and are zero while no rig is active.
The old singleton kept one scheduler for the whole process, so its counters
never went back to zero; the cache strip's "completed" label and its repaint
gate depend on that. `MutableSchedulerProfiler` is per activation: it
belongs to the scheduler, and is null while no rig is active.

## The directory

The directory is a process-level map from stage to context. It lives in
`registry.cpp` as `RigExecImagingDirectory`.

```cpp
static Ptr ForStage(const UsdStageRefPtr &stage, bool create = true);
static Ptr ForStageCacheId(long long id, bool create);  // UsdUtilsStageCache::Get()
static Ptr ForKey(uint64_t key);
static Ptr Current();                                   // legacy current context
static size_t ContextCount();
uint64_t GetKey() const;
UsdStageRefPtr GetBoundStage() const;
```

- **Keyed by stage identity.** Entries are found by the `UsdStage*` and
  validated through a `UsdStageWeakPtr`. An expired stage therefore never
  aliases a new stage allocated at the same address. Every context carries a
  monotonically increasing **key**, never the raw pointer. Anything that
  outlives a stage holds the key: chains, TouchPose highlights, and the
  `rigExec/stageKey` leaf.
- **Lifetime.** The directory holds contexts strongly until their stage
  expires. The next directory call then prunes them. An active context holds
  its stage, so only an inactive context can expire. Pruned contexts are
  destroyed after the directory unlocks. Who ends an activation depends on
  who started it; see "Activation lifetimes".
- **Contexts are created on demand.** The rig adapter creates one for every
  stage it images, and so do `RigExecTouchPose_Open`, `SetWeightOverlayForStage`,
  `BeginPreviewForStage` and a legacy activation. A context with no rig is
  cheap: a store, an index and a mutex.

## Chain to context binding

There is no stage at `AppendSceneIndex` time, so a chain binds when it is
first populated:

1. The scene-index plugin builds pruning, xforms, binding and results over an
   empty store, and registers the chain with the directory as **unbound**
   (`RegisterUnboundChain`). An unbound chain receives nothing stage-specific.
2. The keyless rig adapter (`RigExecImagingRigAdapter`) contributes a
   `rigExec/stageKey` leaf on **top-level prims only**. The leaf is a
   `HdRetainedTypedSampledDataSource<uint64_t>` holding
   `ForStage(prim.GetStage())->GetKey()`. When a top-level prim is also a
   `RigExecRoot`, the leaf is merged into the same `rigExec` container as the
   `rigExec/time` trigger. Any prim that is neither top-level nor a rig root
   still gets no subprim data.
3. `RigExecResultsSceneIndex::_PrimsAdded` pulls the leaf from its input
   (`RigExecStageKeyFromAddedEntries`) on every batch that holds a top-level
   prim. When the leaf names a context other than the bound one, the results
   index calls `BindChain(this, key)`. This happens on the first population,
   and again when a stage is replaced under the same engine. The directory
   records the key, and the context seeds the chain with its store (an atomic
   `SetStore` that announces the swap as a structural generation), its
   generated scopes, its preview deltas and its binding epoch.
4. From then on the chain resolves its context through the key:
   - the time trigger (`_PrimsDirtied` calls `ForKey(key)->SetTime`);
   - eager activation (`IsNotedRigRoot` on the bound context);
   - `_Broadcast`, `_SetChainXformDeltas` and the scope updates, which reach
     only the chains bound to the context's key.

5. A chain leaves its context in two ways. A later batch can rebind it to
   another key (a stage replaced under the engine), or its results index can
   be destroyed (the engine is gone). The destructor calls
   `ReleaseChain(this)`, and the directory drops the record by identity. In
   both cases the context the chain left is checked for release (see
   "Activation lifetimes").

The mechanism is the same as the time trigger's: a data-source leaf that the
stage scene index already builds, with no side channel. The TouchPose highlight
scene index binds through the same leaf.

## Activation lifetimes

A context is activated in one of two ways, and the way it was activated
decides who deactivates it.

- **Explicit**: `Activate`, `RigExecImaging_Activate`,
  `RigExecImaging_ActivateForStage`, or the legacy handle. The host owns the
  activation, and it lasts until the host calls `Deactivate` /
  `DeactivateForStage` (or the legacy `RigExecImaging_Deactivate` on the
  current context). An explicit activation outlives its engines. usdview's
  plugin relies on this: it activates once and keeps evaluating while
  usdview rebuilds its renderer.
- **Automatic**: the rig adapter's `EnsureActivated` activates a stage when
  its rig root is first imaged, which is how usdrecord and a plain Hydra host
  evaluate. The library owns this activation. When the last chain bound to
  the context goes away, the context deactivates itself and releases its
  stage, its compiled rigs, its edit listener and its worker threads. A chain
  goes away when its engine is destroyed, or when a `SetStage` rebinds it to
  another stage. Nobody else would release it: the host never asked for the
  activation, and an active context keeps its stage alive, so the directory
  would never prune it.

Rules:

- The release checks, under the context's `_mutex`, that the activation is
  still automatic and that no chain is bound to the context. A chain that
  binds in the meantime is either counted, or it adopts the cleared state and
  re-activates through its own population.
- A release is not a host deactivation. A later engine on the same stage
  activates the context again through `EnsureActivated`.
- An explicit activation of an automatically active context takes the
  lifetime over (`IsAutoActivated()` turns false). After that, only the host
  deactivates it.
- A context activated automatically that no chain ever binds to is not
  released. This happens with a hand-built pipeline that never registers
  with the directory. It lasts until its stage's host deactivates it.
- The results index's destructor is the hook. It runs with no context lock
  held, because contexts and the directory hold chains only weakly. A results
  index destroyed during static destruction, after the directory, does
  nothing.

## Legacy surface

`GetInstance()` and every stage-less C function keep their signatures.
`GetInstance()` returns the **legacy handle**: an unbound context object that
owns no stage and routes every call to the **current context**. The current
context is the one most recently activated through `RigExecImaging_Activate`
or `GetInstance().Activate`. If no current context is live, the rig adapter's
automatic activation (`EnsureActivated`) also claims it; this is the usdrecord
case. An automatic activation never takes the current designation from a live
explicit one. With no current context, the handle answers as an inactive
registry: an empty store, `SetTime` returns false, and so on.

- `GetInstance().Activate(stage)` and `RigExecImaging_Activate(id)` forward to
  that stage's context. If the stage has no context, one is created, seeded
  with the legacy overlay selection (the old singleton's process-wide viewer
  mode). On success the target becomes current. **Nothing is replaced:**
  other stages keep evaluating.
- `RigExecImaging_ActivateForStage(id)` activates that stage's context the
  same way, but it does not make the context current and does not apply the
  legacy overlay. A stage's own overlay is set with `SetWeightOverlayForStage`.
- `RigExecImaging_Deactivate()` deactivates the current context only.
- `EnsureActivated` never refuses on account of another stage. Each stage
  activates its own context. It also never revives a context that its host
  explicitly `Deactivate`d; only an explicit `Activate` does that. Without
  this rule, a stray rig-root pull could re-activate a closed stage, and the
  context would then pin that stage for the life of the process. One such
  pull: UsdImaging traverses the old stage while a stage scene index
  announces a `SetStage`.
- A context's own `Activate` or `EnsureActivated` called with a different
  stage forwards to that stage's context.

A host that activates one stage at a time and deactivates before switching
behaves as it did with the singleton. usdview's plugin, usdrecord and every
pre-existing test are hosts of this kind. A legacy caller can still observe
these differences:

- **Re-activation does not replace.** With the singleton,
  `RigExecImaging_Activate(B)` released A. Now A keeps evaluating, and keeps
  its stage and threads, until something deactivates it. The stage-less
  `RigExecImaging_Deactivate()` reaches only the current context (B), so a
  legacy-only host that switches stages without deactivating first must
  release A with `RigExecImaging_DeactivateForStage(idA)`. Replacing on
  re-activation is not an option, because a multi-session host activates
  every session's stage through `RigExecImaging_Activate`.
- **Automatic activation evaluates every imaged stage.** The singleton's
  `EnsureActivated` refused a second stage. Now each stage's rigs activate
  on first sight, and the activation is released with its last chain.
- **The legacy overlay applies to every legacy activation.** The legacy
  `SetWeightOverlay` selection is the old process-wide viewer mode. Every
  stage activated through the legacy path afterwards inherits it, not only
  the current one.
- **Warming counters are per context.** They survive re-activation (as the
  singleton's did), but switching the current context switches which
  counters the legacy `GetWarmingCompletedCount` reads.

## C API

Every per-stage entry point has a `...ForStage` twin. The twin takes the
stage's `UsdUtilsStageCache` id **first**, then the legacy arguments, and
returns what the legacy function returns. A twin acts on that stage's context
only and never moves the current context. An unknown id, or a stage with no
context, answers exactly as an inactive registry would.

| legacy (routes to the current context) | stage-scoped twin |
|---|---|
| `int Activate(long long id, const char *rig, double frame)` (also makes the stage current) | `int ActivateForStage(long long id, const char *rig, double frame)` (current untouched) |
| `int SetTime(double frame)` | `int SetTimeForStage(long long id, double frame)` |
| `int OnEditCommitted()` | `int OnEditCommittedForStage(long long id)` |
| `int OnIdle()` | `int OnIdleForStage(long long id)` |
| `int WriteProfileSummary(const char *path)` | `int WriteProfileSummaryForStage(long long id, const char *path)` |
| `void Deactivate()` | `void DeactivateForStage(long long id)` |
| `long long GetGeneration()` | `long long GetGenerationForStage(long long id)` |
| `int GetMovedFloats(const char *paths, float *out, int count)` | `int GetMovedFloatsForStage(long long id, const char *paths, float *out, int count)` |
| `int GetGuideBoundsAssetSpace(const char *prim, double out[6])` | `int GetGuideBoundsAssetSpaceForStage(long long id, const char *prim, double out[6])` |
| `int GetAllGuideBoundsAssetSpace(double out[6])` | `int GetAllGuideBoundsAssetSpaceForStage(long long id, double out[6])` |
| `int SetWeightOverlay(const char *prim)` | `int SetWeightOverlayForStage(long long id, const char *prim)` (creates the context) |
| `int GetFrameStates(const char *rig, const double *frames, int *states, int count)` | `int GetFrameStatesForStage(long long id, const char *rig, const double *frames, int *states, int count)` |
| `int ClearFrameCache(const char *rig)` | `int ClearFrameCacheForStage(long long id, const char *rig)` |
| `int WarmRange(const char *rig, const double *frames, int count)` | `int WarmRangeForStage(long long id, const char *rig, const double *frames, int count)` |
| `long long GetWarmingCompletedCount()` | `long long GetWarmingCompletedCountForStage(long long id)` |
| `int BeginPreview(const char *paths)` | `int BeginPreviewForStage(long long id, const char *paths)` (creates the context) |
| `int UpdatePreview(const double *values, int count)` | `int UpdatePreviewForStage(long long id, const double *values, int count)` |
| `int EndPreview()` | `int EndPreviewForStage(long long id)` |

Every name above carries the `RigExecImaging_` prefix. The following functions
already take the id and use **that** stage's context:

- `int RigExecImaging_Activate(long long id, const char *rig, double frame)`
  also makes the target the current context. Its twin
  `RigExecImaging_ActivateForStage` does not.
- `int RigExecImaging_GetControlFrameAssetSpace(long long id, const char *prim, double frame, int isDefault, double out[16])`.

Two functions exist for diagnostics and tests:

- `int RigExecImaging_IsActiveForStage(long long id)` returns 1 or 0.
- `int RigExecImaging_ContextCount()` returns the number of contexts for live
  stages.

The UsdGeomBoundable extent callback reads the queried stage's own context.
`RigExecTouchPose_SyncPose` reads the pose from the handle's stage context,
and TouchPose highlights are keyed by that context. Key 0 is unscoped: it is
used for topology-only handles and lights every chain.

**Previews.** A context whose stage has no rig can still preview. It resolves
xform-lane slots on its bound stage, so `BeginPreviewForStage` works on a plain
Xform, and the deltas reach only that stage's chains. The rig lane previews
through the context's own evaluator.

Viewport release queues authoring on the stage's owner thread, retaining the
final preview until the commit. Undo, a new drag, time or selection changes,
and saving drain the pending edit first; a replacement stage invalidates its
queued callback. A clean preview committed with exactly the same values keeps
its published generation, including the first opinion in an empty session
layer. Empty-field ancestor overs are inert; real metadata, structural edits,
connections and revised property-chain targets still require evaluation.

## usdview plugins

The Python plugins (`plugin/rigExecUsdview`, `plugin/touchPose`,
`plugin/shapeEditor`) are loaded once per process, but usdview builds their
containers once per session. A host that runs several usdview sessions in one
process (usdOrchestrate's shared host) therefore shares every module between
its windows, and the plugins keep their state per session.

- **Imaging handle** (`plugin/rigExecUsdview/imagingHandle.py`). Every call a
  plugin makes into rigExecImaging goes through an `ImagingHandle(lib, stage)`
  bound to its own session's stage, found by the stage's `UsdUtilsStageCache`
  id. The handle takes the legacy names (`SetTime`, `OnIdle`, `GetGeneration`,
  ...), and `getattr` also resolves the `RigExecImaging_<Name>` spellings
  (`cacheStripModel` looks them up that way). It calls the `...ForStage` twin
  when the library exports one. Otherwise it calls the legacy function, which
  is the old single-stage behaviour, and warns once.
- **Activation** goes through `RigExecImaging_Activate(id, ...)`, not
  `RigExecImaging_ActivateForStage`. It activates that stage's own context and
  also makes it the legacy current context. In a plain usdview the stage-less
  API therefore still reaches the one session's stage, exactly as before, for
  callers outside the plugins such as a script in usdview's interpreter or
  another plugin. In a host, the current context is the stage activated last,
  and no RigExec plugin code reads it. Deactivation is `DeactivateForStage` for
  the container's own stage only; a container never deactivates another
  session's stage.
- **Per-session state** (`plugin/rigExecUsdview/sessionRegistry.py`). A
  `SessionRegistry` files one value per session. Its key is the session's main
  window (`usdviewApi.qMainWindow`), held weakly, and the entry is dropped when
  the window is destroyed. The state kept this way: the rigExec, TouchPose and
  shape editor containers (`rigExecUsdview.ContainerFor(api)`), the gizmo and
  view cube controllers and their pending installs
  (`gizmoUI.GetController(api)`, `viewCubeUI.GetController(api)`), gizmo tool
  settings, `gizmoMath`'s published-frame readers, `gizmoPreview`'s channels,
  and every panel's `_instance`.
  A caller that passes no api gets the session that owns
  `QApplication.activeWindow()`, else the only session, else None. No session
  wins by registering last. The old module attributes (`gizmoUI._controller`,
  `viewCubeUI._controller`, `rigExecUsdview._container`) can still be read,
  and give that same answer. `sessionRegistry.ForgetSession(api)` drops a
  session from every registry at once; usdOrchestrate's host calls it when a
  session closes.
- **Application-wide event filters.** The gizmo hotkeys act only on events
  whose window belongs to their session (`sessionRegistry.SessionOfEvent`).
  The graph editor's hotkeys act only inside its own panel.
- **Published control frames.** Each session's `gizmoMath` reader answers only
  for its own stage, compared by identity, so a gizmo never reads another
  session's evaluated frame, even at an identical prim path.

## Lock order

```
scheduler fence mutex          (ClearFrameCache / SetWeightOverlay only)
  -> context _mutex
       -> context _notedMutex
       -> scheduler state -> warm index / frame cache shards
       -> DIRECTORY mutex      (leaf)
```

- The directory mutex is a **leaf**. It is taken under a context's `_mutex`,
  for chain snapshots in `Activate`, `Deactivate` and `_SetChainXformDeltas`,
  and it is taken on its own. While it is held, nothing calls into a context
  and no context is destroyed.
- `_notedMutex` stays separate from `_mutex`. The adapter notes rig roots, and
  now also resolves `ForStage` (directory leaf), from traversals that
  `_DirtyXformSubtree` makes while holding `_mutex`.
- Notices that re-enter a context are always sent unlocked. This covers
  `_Broadcast`, `_AdoptChain`, and the results index's trigger through
  `ForKey`. `_SetChainXformDeltas` sends under `_mutex`, as it always did,
  because the xform dirties never intersect `rigExec/time`. The results index
  tests the trigger locator before it looks up the context.
- Legacy routing adds one directory lookup per call and no context lock
  beyond the target's own.
- An automatic release (`ReleaseChain`, or a rebind in `BindChain`) resolves
  the context through the directory with no lock held. It then takes that
  context's `_mutex`, checks the bound chains under the directory leaf, and
  deactivates. The broadcast and the scheduler join run unlocked, exactly as
  in `Deactivate`.

## Lifetimes (summary)

- A stage's context lives while the stage lives. After the stage expires, the
  next directory call releases the context. An active context holds its
  stage, so the activation has to end first:
  - An **automatic** activation ends by itself when its last bound chain goes
    away, whether the engine is destroyed or the stage is replaced under it.
  - An **explicit** activation ends only when its host deactivates it.
  UsdImaging's own chain also holds the stage until the engine is destroyed;
  `SetStage(nullptr)` is not enough. A host-deactivated context stays
  inactive until the host activates it again, so an engine torn down after
  the deactivation cannot revive it.
- A chain's registration dies with its results index (its destructor calls
  `ReleaseChain`). A chain that outlives its context keeps the stale key,
  `ForKey` resolves that key to nothing, and the chain keeps working and can
  rebind.
- The legacy current context is held strongly until the next legacy
  activation (or, with no live current context, an automatic one) takes the
  designation. Taking the designation does not deactivate the context that
  held it (see "Legacy surface"). A current context that was released or
  deactivated answers as an inactive registry.
