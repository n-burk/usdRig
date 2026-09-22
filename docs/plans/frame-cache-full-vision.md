# Full-vision frame cache: async everywhere, partial invalidation, timeline coloring, every op

## Goal

Make the RigExec frame cache the engine the viewport runs on: it warms frames
asynchronously ahead of and behind the playhead on every rig including the
full biped stack; a rig edit invalidates and re-caches only the affected part
of the solved graph across cached frames; the timeline shows per-frame cache
state (Maya-style coloring); and the frozen executor supports every op the
baked program can express, so no shipped baked rig silently falls back to
live-only evaluation. (D7 dynamic rigs stay UI-memo by design; see
Constraints. A named, tested refusal is explicit, not silent:
refusal reasons surface in profiler/stats output — never swallowed
(the freeze error is literally `ignored` today,
`registry.cpp:252-256`, and `BuildWarmWork` returns empty with no
reason) — and empty-factory skips distinguish freeze-refused from
unsampleable from D7-exempt.)

## Success Criteria

- On `examples/biped/Biped_stack_anim.usda`, scrubbing to unvisited frames
  gets faster over a session: first-visit cost is bounded by live evaluation
  plus memoization, and steady-state scrubbing over warmed ranges serves
  cache hits with no evaluator pulls.
- Background warming completes jobs on the full stack (today: `completed=0`
  forever). Proof: `benchPlayback`-style SetTime+OnIdle runs show
  `published` growing and `enq > 0` on idle triggers.
- A control-value edit re-caches only frames whose inputs changed and only
  the affected subgraph within them; an edit to an unrelated control does
  not evict or re-warm the world. Proven by targeted tests, not by timing.
- The usdview plugin shows a per-frame cache strip: cached / warming /
  dirty / uncached states over the stage range, updating as workers
  complete.
- Every `RigExecRevisionOp` (14) and every `RigExecBakedStepKind` (18) the
  compiler can emit is either executed by the frozen executor or carries a
  named, tested refusal. Every shipped baked example warms with no
  refusal; D7 dynamic rigs are exempt by design, not by silence.

## Context And Current Facts

Architecture today (all verified against code this session):

- Viewport tick = `SetTime` + `OnIdle` (`benchPlayback.cpp:6-11`,
  `plugin/rigExecUsdview/rigExecUsdview.py:952-975`). `SetTime` evaluates
  and publishes (`registry.cpp:685`); `OnIdle` prepares a warming burst and
  enqueues sweep jobs (`registry.cpp:1042-1095`).
- Frame-cache key = epoch digest + control-state digest; lookups serve only
  under a freshness proof (`frameCache.h:100-140`, `bridge.cpp:1699-1796`).
  Memoization samples the full input vector plus chain prologue after each
  live run (`bridge.cpp:1799-1838`, `_ComputeCacheKey` at `:1655`). Both
  production publishes are pose-only (`bridge.cpp:1570,1827`); the
  retained-handle `Publish` overload exists (`frameCache.h:234-251`) but
  only `frameCacheSparsity` and its tests use it. Entries carry no
  dependency record: key (epoch + whole-vector digest), pose, time hint,
  LRU tick (`frameCache.h:57-67,276-295`).
- Warming jobs are built on the UI thread per sweep time (64 sweep times
  around the playhead, `registry.cpp:130-144`), sampled through the burst
  cache, and executed by workers against a frozen program snapshot
  (`registry.cpp:813-934`, `backgroundScheduler.cpp:399-412,648-758`).
- Invalidation today is coarse: a notice under the read roots marks the
  session dirty, cancels the warming generation, and retires the epoch
  half (`registry.cpp:1714-1730`); proofs are scoped by epoch+mode and
  cleared wholesale (`bridge.h:409-424`). No path- or subgraph-scoped
  retirement of cached entries exists.
- Partial-evaluation substrate exists but is not wired to the cache:
  per-cluster forward closures (`bakedProgramImpl.h:934-954`) and the
  output-affected index mapping control paths to dirty clusters
  (`outputAffectedIndex.cpp:28-67`). The index keys on control IDs
  (sampled path strings / override identities,
  `outputAffectedIndex.h:34-52`), has no USD-notice adapter, and treats
  unknown controls as all clusters (`:107-112`). Scheduler tests pin the
  burst shape: idle enqueues exactly 64, commit exactly 80
  (`testRigExecImagingFrameCache.cpp:759-783`).
- The C API exposed to the Python plugin covers activate/SetTime/idle/
  profile/generation/reads (`registry.h:428-444`) but exposes no
  per-frame cache state; stats are aggregate counters
  (`frameCache.h:109-121`). Stats, `GetBackgroundStats`, and
  `ClearFrameCache` are C++-only (`registry.h:116-127`); the plugin binds
  Activate/SetTime/OnIdle/OnEditCommitted/Deactivate plus read APIs
  (`rigExecUsdview.py:62-86`), and the triggers take no frame arguments
  (`registry.h:74-81`). The plugin's only timer is the single-shot commit
  flush, and `OnIdle` runs only on frame changes
  (`rigExecUsdview.py:905-975`): nothing ticks at a held playhead.

Measured baseline (probes built and run this session against `build/`):

- Stack, cache on (usdview default): SetTime ~62 ms (evaluate ~5-14,
  memoize sampling ~48, guides ~4), OnIdle ~205 ms with `enq=0` every
  tick. Flat rig for comparison: SetTime ~6 ms, OnIdle ~11 ms.
- The 205 ms is 64 sweep-time `BuildWarmWork` calls (~3 ms each) that all
  decline: `RigExecFreezeProgram` refuses on the stack, so no snapshot
  exists, and the frozen check runs after the sample
  (`registry.cpp:861-934`). `completed` stays 0.
- Freeze refuses twice over on the stack: weight objects
  (`frozenContext.cpp:2938`, from the face layers — `Biped_all.usda`
  refuses, `Biped_layered.usda` freezes fine) and the `body_geo_shapes`
  revision op, which is not Skin/RecomputeNormals/RecomputeExtent
  (`frozenContext.cpp:2970-2977`, from PSD/shapes — `Biped_psd_all.usda`
  refuses).
- Cache off: ~29 ms/scrub. Warm-off: ~80 ms/scrub. Memoized revisits hit
  (cache `published`/`entries` grow; re-scrub serves without pulls).
- Shipped-example op coverage (the implement/refuse boundary): the
  `YES` fixtures author BlendShape, Ribbon, VolumeCorrect, Smooth,
  VolumeWeights and ReadPhases behaviors (`tests/exampleFixtures.cmake:
  62-72`, `examples/05_TwistRibbonSpine.usda:94-100`), but `YES` means
  `RigExecBakedProgram::Build` accepts the rig — compile, not
  execution (`:39-44`): 04/05/12/13/ArmRig all build, yet their
  weighted/ribbon-driven halves neither RUN baked (held on the
  dynamic path by refusals owned elsewhere, which the geometry-ops
  suite unbinds in memory,
  `tests/testRigExecGeometryOpsBakedParity.cpp:4-18`,
  `CMakeLists.txt:535-542`) nor reach the FROZEN executor (Stream
  0's weight-object + driver-frame refusals). The exercised set is
  therefore frozen reachability post-unblock (0.3), not the YES set.
  The freeze
  bisects prove the biped stack needs weight objects plus blend-shape
  revisions (`frozenContext.cpp:2938,2970-2977`). Refusal is allowed
  only outside the post-unblock exercised set.
- Prior art: `docs/plans/per-frame-caching-system.md` (830 lines) covers
  scheduler/frozen/D7 design; it has no timeline-visualization section and
  no op-completeness section (verified by search). `reports/frame-cache-
  measurements.md` §9-10 records flat-rig burst/commit costs and the
  UI-thread sampling rule (workers cannot sample: USD forbids concurrent
  read-during-write).

## Constraints And Non-goals

- Keys stay within-process and bitwise-exact; the "drop, don't guess" rule
  (`generation.h:45-64`) and bit-identical hit semantics stay.
- Sampling stays on the UI thread (measurements report §10). Bursts must
  stay budgeted; no unbounded per-tick sampling growth.
- D7 dynamic/refusal rigs keep the UI-thread memo path and never enqueue
  background jobs (`generation.h:71-90`); this plan does not put them on
  workers. They are the only sanctioned live-only evaluations.
- No persisted (on-disk) cache; no cross-session cache.
- No painting into stock usdview widgets (see D2).
- No changes to evaluation semantics: cached poses must remain bit-identical
  to live (`CheckPosesBitIdentical` stays the judge).

## Key Decisions

- **D1 — Unblock order: op coverage first.** Warming completions are 0 on
  the stack because the freeze refuses. Timeline coloring with nothing
  cached, and cross-frame invalidation with no entries, are empty work.
  So: frozen-executor op coverage (weight objects, blend-shape revision
  op first) is Stream 0 and gates the rest. Rejected: timeline-first
  (nothing to color), invalidation-first (nothing cached to invalidate).
- **D2 — Timeline coloring is a custom cache strip dialog, not paint on
  the stock timeline.** usdview's timeline is `FrameSlider`, a bare
  57-line `QSlider` subclass with no overlay API (installed
  `pxr/Usdviewq/frameSlider.py`), and `usdviewApi` does not expose the
  widget (installed `pxr/Usdviewq/usdviewApi.py`). The repo pattern is
  modeless dialogs via `configureView` menu commands
  (`rigExecUsdview.py:313-329`; `ProfilerPanel` is a `QDialog`,
  `profilerUI.py:53-72,215-221`); the profiler panel is the
  integration model — a dialog, not a dock. Rejected: reaching into
  appController privates or replacing the slider (fragile across USD
  upgrades).
- **D3 — Partial invalidation keys on cluster closures, plus new entry
  provenance.** `RigExecBakedCones` + `RigExecOutputAffectedIndex`
  already map moved inputs to dirty clusters, but cached entries carry
  no record of which clusters produced them, so there is nothing to
  retire at that granularity today. Stream 2 therefore adds entry
  provenance/dependency records (2.0) and then retires and re-resolves
  *cached frames* at cluster granularity instead of epoch/generation
  wholesale. This requires rescoping the epoch digest first: avar
  constants are folded into it (`frozenContext.cpp:3133-3145`), and any
  drift evicts the old epoch half eagerly (`bridge.cpp:1630-1641`), so
  a value patch would still nuke the cache without cluster retirement
  ever mattering. Epoch becomes structure-only (binding epoch, build
  count, varying/promoted sets); avar constant *values* move into
  control-digest/proof coverage. The rescope splits, not deletes: the
  frozen-snapshot refresh compares the avar-region digest to decide
  patch-vs-keep (`registry.cpp:211-238`), so it keeps its own
  value-sensitive region digest while the epoch stops folding it.
  Constants do NOT ride the per-frame control digest: the sampler
  skips non-varying bindings by design
  (`frozenContext.cpp:101-116`) and the digest hashes
  `FrameInputs.values` only (`:2428-2446`,
  `frameCache.cpp:385-405`), so folding nothing would alias
  pre/post-patch poses. Instead a new epoch-constant sampler captures
  the constant region at epoch build and at every value patch, and its
  `constantDigest` folds into control-digest derivation — one more
  fold at the `RigExecControlStateDigest` site, level-1-chain-
  compatible (`frameCache.cpp:385-405`); the key struct stays
  (epochDigest, controlDigest) (`frameCache.h:59-67`). Retirement
  tracks (epoch, constantDigest, time) while lookup keys stay
  two-field, so a patch opens a new key namespace with two free
  soundness properties: proofs auto-mismatch across constant-only
  changes (digest compare, `bridge.cpp:1735-1741` — no proof-scope
  change; `bridge.h:417-418` epoch+mode scoping untouched), and
  digest equality implies same namespace, so the 2.2 lanes partition
  exactly. Crossing it does NOT orphan
  the cache: entries whose 2.0 provenance touches nothing dirty are
  carried over — re-published under the new key and old key evicted
  (existing `Publish`/`Evict`, `frameCache.h:225-254`), proofs
  re-pointed via 2.3 — with zero recompute; only entries touching
  dirty clusters re-run. Without the carry-over path exact lookup
  (`bridge.cpp:1699-1746`) would strand every entry at each
  namespace change. Rejected: keeping epoch-only (too
  coarse for the ask) and inventing a second dependency system beside
  the cones.
- **D4 — Keep the scheduler/worker design; fix the doomed-sample path.**
  The flat-rig behavior is good (benchPlayback: 6/11 ms ticks, warming
  completes). The stack pathology is ordering, not architecture: move the
  frozen-snapshot check before sampling in `BuildWarmWork`, add the
  missing profiler scopes (memoize sampling, burst prep), then extend
  sweeps to the full range under caps. Rejected: scheduler rewrite.
- **D5 — Per-frame states come from a new C query API that constructs
  state the engine does not track today.** Add
  `RigExecImaging_GetFrameStates(rig, frames[])`-style query returning
  cached/warming/dirty/uncached per frame, backed by a new
  completed-per-frame index — not by the proof map. Proofs are recorded
  at enqueue time, before jobs run (`bridge.cpp:1587-1611`), can
  outlive declined jobs and evicted entries, and entries are keyed by
  digest with time as a hint (`frameCache.h:57-67,98-107`). The index
  is shared-owned, never bridge-called: worker closures must not hold
  bridges (sessions die on Deactivate while pool workers outlive them;
  completions land via shared cache handles,
  `bridge.h:48-51`) — so the index is a shared object held by the
  bridge and by job closures alike, fed by publish completions
  (memoize + background paths record into shared state; background
  completions today publish straight to the cache with no bridge
  notification, `bridge.cpp:1557-1570`), and retired by eviction,
  which today reports counts only, replaces same-key entries silently
  and clears silently (`frameCache.cpp:677-689,716-739,742-760`), so
  eviction/clear gain affected-key reporting (or an observer hook) as
  part of this unit. Alias semantics: one digest may serve many frames
  (all list cached); a replace-under-held-key retires every aliasing
  frame to uncached until re-resolved; `Clear` resets the index. The `(epoch, time)->key` sidecar in
  sparsity land (`frameCacheSparsity.h:225-250`) is the starting shape,
  promoted to production with eviction wiring plus carry-over
  rewriting. The rewrite sequence per carried frame is Find-old →
  sparse-Lookup (pose + retained + bytes) → sparse-Publish under the
  new key → cache-Evict(old) → NotePublished(new), which REPLACES
  the row (`frameCacheSparsity.cpp:304-309`) — no Drop: Drop erases
  by (epoch, time) and would eat the new row. No cache enumeration
  is needed: the adapter iterates the known completed-time set from
  1.4 transition recording, Finding per time. Re-publish needs one
  new accessor — sparse Lookup returns pose + retained but not bytes
  (`frameCache.h:246-251`; bytes live in the private entry,
  `:276-283`) — so Lookup gains a bytes out-param. Interleavings
  need no cross-op atomicity: a lookup landing mid-carry-over misses
  (proofs already mismatch) and evaluates live, and a worker
  publishing the same new key concurrently holds bit-identical
  content (deterministic eval), so silent same-key replace
  (`:229-230`) is benign — the codebase's own LRU-race rule
  (`:284-287`): races cost misses, never wrong poses. Rows the
  adapter never rewrote read as Miss under the sidecar's own
  stale-key rule (`frameCacheSparsity.h:225-229`) — never as a wrong
  base — so no lost carry-over and no old-namespace key reuse. Per-frame queue
  visibility and per-frame dirtiness are likewise new tracked state:
  completions alone cannot supply them (scheduler stats are aggregate
  counters, `backgroundScheduler.h:174-207`), so the scheduler gains
  per-frame transition recording — queued/running/declined/shed/
  canceled/completed by frame, with hooks at the shed/cancel points
  that today only bump counters.
  Index lifecycle is swap, never mutate-under-readers: `Build`
  replaces the index in place (`outputAffectedIndex.h:74-79`) while
  the header's share contract covers const queries only
  (`:63-65`), so each epoch builds a NEW index the bridge swaps in
  atomically (shared_ptr); sidecar entries and transition rows are
  tagged (epoch, generation), worker consults pin the shared_ptr for
  the job lifetime, and post-cancel drains record nothing for purged
  generations (running jobs are not preempted,
  `backgroundScheduler.h:232-237`).
  The proof map stays bridge-private, and dirty stays global until 2.1
  lands path-scoped retirement.
  State colors are an assumption (see Open Questions), proposed:
  cached green, warming blue, dirty red, uncached grey.

## Recommended Approach

Four streams in dependency order. Stream 0 makes warming exist on the
stack; Stream 1 makes it cheap and observable; Stream 2 makes edits
surgical across frames; Stream 3 shows it on a timeline strip. Each
stream lands behind existing env flags where behavior changes
(`RIGEXEC_FRAME_CACHE=on/off/warm-off` keeps working as the rollback
switch), and each carries frozen/live bit-identity tests.

The guiding invariant: **a cached frame is served only under a proof
that names exactly what it depends on.** Op coverage extends *what can
be proven*, partial invalidation narrows *what each proof covers*, and
the timeline *displays proof coverage*.

## Work Plan

### Stream 0 — Frozen executor knows every op (gates all else)

- **0.1 Weight objects in the frozen executor.** The freeze nulls the
  weight callbacks (`frozenContext.cpp:2858-2865`) and refuses the
  `WeightPacket` / `VolumePlacements` / `SnapshotFinals` steps, but
  shipped fixtures pin all three behaviors (`11_VolumeWeights`,
  `13_ReadPhases`, `14_VolumeConstrainedSweep`,
  `tests/exampleFixtures.cmake:69-72`). Worker-safe strategy, following
  the revision-packet pattern: extend the job inputs with weight /
  placement snapshot fields (`RigExecFrameInputs` today carries values,
  revision packets, chain results and overrides,
  `frozenContext.h:105-147`, but no weight snapshots), sampled from
  the oracle on the UI thread at enqueue time where stage reads are
  legal; frozen packet-build / placement-refresh / phased-read kernels
  consume the snapshots over frozen slots on workers — no live stage,
  no callbacks. Digest coverage is part of the unit: the control
  digest folds sampled values plus overrides only
  (`frameCache.cpp:385-405`), so snapshot fields either stay excluded
  by proving purity from digest-covered values (the revision-packet
  precedent, `frozenContext.h:111-123`) or fold into the digest with
  the digestibility gate extended — distinct snapshots must never
  share a key. Byte accounting extends to the new fields: retained
  accounting today covers sampled values plus overrides only
  (`frameCacheSparsity.cpp:107-121`), omitting revision packets,
  chain results/diagnostics and snapshots, while retained publish
  trusts caller-supplied bytes — publish must compute (or verify)
  the full retained size. Succeeds when `Biped_all.usda` freezes and
  its warmed frames bit-match live.
- **0.2 Blend-shape revision op.** Implement the missing
  `RigExecRevisionOp` for `body_geo_shapes` in the frozen executor
  (read the live revision kernel; mirror it over frozen slots).
  Succeeds when `Biped_psd_all.usda` freezes.
- **0.3 Remaining revision ops.** Walk the 14 `RigExecRevisionOp`
  enumerators (`moverGraph.h:59-76`): implement each of the 11 beyond
  Skin/RecomputeNormals/RecomputeExtent that any shipped baked
  example exercises — exercised POST-unblock, with all Stream 0
  refusals removed, not in today's run set. The known cascade: 0.1
  lets 04/12/13 freeze, so their BlendShape / non-default-read-phase /
  curvenet ops become exercised and must be implemented; 0.4's
  removal of the curvenet/driver-frames/delta gate
  (`frozenContext.cpp:2997-3004`) lets 05/ArmRig freeze, so the ribbon
  op becomes exercised too; the audit establishes emitGuidePoints
  authorship and any further cascade members the same way. VolumeCorrect,
  Smooth, the volume-weight behaviors and the stack's weight objects
  plus blend-shape revisions are already confirmed on shipped
  examples (see Context). A named, tested refusal is allowed only
  for ops unreachable even post-unblock.
- **0.4 Remaining refusal gates.** Close or justify every
  `RigExecFreezeProgram` gate not owned by 0.1-0.3/0.6, with the same
  post-unblock exercised-set rule (implement where any shipped baked
  example needs it once Stream 0 refusals are removed; named refusal
  only where none does): CPU parity mode / no baked
  program / API misuse (`:2891-2901`, by design, tested); chain bind
  errors and chain-discovery disagreement (`:2910-2919`); chain
  revisions binding weight objects (`:2920-2930`, with 0.1);
  xform-derived seeds / native sources / delta bases (`:2932-2936`);
  constraint weight objects and geometry-domain revisions
  (`:2950-2962`); ladder-varying providers (`:2963-2965`); blend
  channels (`:2978-2983`); phased reads and snapshot steps
  (`:2984-2989`, `:2942-2949`, with 0.1/0.6); revision weight objects
  and current-phase reads (`:2991-2995`); curvenet nets, driver
  frames, geometry-delta hand-offs (`:2997-3004`); unfixed skin
  layouts (`:3006-3012`); derived-target op and phase restrictions
  (`:3014-3029`).
- **0.5 Sampler parity.** Every `RigExecSampleFrameInputs*` false-path
  and viaChain marking gets the same treatment: implement where any
  shipped baked example needs it; named refusal with a pinning test
  only where none does.
- **0.6 Per-step-kind matrix.** Tabulate all 18 `RigExecBakedStepKind`
  (`bakedProgramImpl.h:572-591`) against frozen support: the 3 refused
  kinds (`WeightPacket`, `VolumePlacements`, `SnapshotFinals`,
  refused at freeze `frozenContext.cpp:2942-2949` and at dispatch
  `:4016-4020`), 2 via dedicated branches (`RevisionStatic`,
  `Derived`, `:4021-4026`), and the remaining 13 through the shared
  geometry / pose runners (`:4027-4031`). Each row is implemented
  where any shipped baked example needs it post-unblock (0.3's rule —
  the matrix is drawn with Stream 0 refusals removed, so the
  blendShape/curvenet/ribbon/read-phase/emitGuidePoints rows the
  cascade newly reaches get implementations and tests, not
  refusals); a named, tested refusal is allowed only for rows no
  shipped baked example exercises even post-unblock. 0.1 closes the
  `WeightPacket` row for the face rig.

  Landed matrix (the enum holds 19 kinds, not 18 — `PoseInterpolator`
  arrived after this section was drafted; every row is exercised by at
  least one shipped baked example and warmed bit-identically by the
  named test; no row refuses):

  | Step kind | Frozen route | Exercised by | Warming test |
  |---|---|---|---|
  | `ComposeSubtree` | shared pose runner | all rigs | every warming test |
  | `Solve` | shared pose runner | 01, 02, 03, 05, 09, 12, 13, Arm*, spider_ik, assembly, bipeds | `TestRibbonSpineWarmsBitIdentical`, `TestArmRigWarmsBitIdentical`, `TestBipedWarmsBitIdentical` |
  | `SolverCommit` | shared pose runner | same as `Solve` | same as `Solve` |
  | `Constraint` | shared pose runner | 08, 10, 14, aims, combos, Arm*, bipeds | the 10-test constraint battery, `TestBipedWarmsBitIdentical` |
  | `CommitDelta` | shared pose runner | bipeds | `TestBipedWarmsBitIdentical`, `TestStackAnimWarmsBitIdentical` |
  | `PropagateChunk` | shared pose runner | bipeds | same as `CommitDelta` |
  | `CommitApply` | shared pose runner | bipeds | same as `CommitDelta` |
  | `ProviderMatrix` | shared pose runner | all but the pure-combo aims | many (e.g. `TestBlendFaceWarmsBitIdentical`) |
  | `SnapshotFinals` | shared pose runner | 13 | `TestReadPhasesWarmBitIdentical` |
  | `PoseInterpolator` | shared pose runner | stack_anim only | `TestStackAnimWarmsBitIdentical` (0.7) |
  | `VolumePlacements` | `_FrozenWeightStep` | 11, 14 | `TestVolumeConstrainedSweepWarmsBitIdentical` |
  | `WeightPacket` | `_FrozenWeightStep` | 01, 02, 03, 04, 05, 08, 09, 11, 12, 13, 14, Arm*, stack | `TestBlendFaceWarmsBitIdentical`, `TestVolumeConstrainedSweepWarmsBitIdentical`, `TestStackAnimWarmsBitIdentical` |
  | `InfluenceFold` | shared geometry runner | all geometry rigs | every geometry warming test |
  | `RevisionStatic` | `_FrozenRevisionStatic` | all geometry rigs | every geometry warming test |
  | `RevisionChunk` | shared geometry runner | all geometry rigs | every geometry warming test |
  | `RevisionFuse` | shared geometry runner | all geometry rigs | every geometry warming test |
  | `ChainStatus` | shared geometry runner | all geometry rigs | every geometry warming test |
  | `Derived` | `_FrozenDerived` | 01–10, 12, 13, aimtest_points, Arm*, simple*, bipeds | `TestBlendFaceWarmsBitIdentical`, `TestRibbonSpineWarmsBitIdentical`, `TestStackAnimWarmsBitIdentical` |

  Cascade ops the matrix newly reaches, each with a frozen-parity test:
  `BlendShape` (04, ArmRig, stack), `Curvenet` (12), `Ribbon` (05,
  ArmRig), read-phases (13), `EmitGuidePoints` (05, ArmRig), `Wire`
  (stack only, via the 0.7 stack test), `Skin` (bipeds). The one
  unexercised op, `CurvenetAdjuster`, keeps its named refusal with
  `TestCurvenetAdjusterRefusesFreeze`.
- **0.7 Harness covers the stack.** Parameterize the stage (and frames)
  of `benchPlayback` (hard-codes `Biped_anim.usda`,
  `tests/benchPlayback.cpp:133`; CLI gains a 4th positional,
  `benchPlayback [examplesDir] [frameCount] [paceMs] [stage]`,
  defaulting to `Biped_anim.usda`), `benchFrameCacheWarm`
  (`tests/benchFrameCacheWarm.cpp:1191-1201`, still scrubs 1-8), and
  the FrozenContext biped tests
  (`tests/testRigExecFrozenContext.cpp:1813`);
  add dedicated `Biped_stack_anim.usda` ctest entries *outside* the
  shared fixture table (a fixture row would multiply across every
  parity/binary/verify consumer). `benchPlayback` is the registry-level
  driver (triggers, scheduler, completions); `benchFrameCacheWarm`
  stays kernel-level (it drives evaluator + local cache directly with
  no imaging code linked) and takes stage/frames only.

### Stream 1 — Async across all frames, cheap and observable

- **1.1 Kill the doomed sample.** In `BuildWarmWork`
  (`registry.cpp:861-934`), check snapshot existence (and burst
  usability fast-fail) before sampling; skip sweep times that cannot
  produce jobs. Burst prep gets the same treatment: `_PrepareWarmBurst`
  (frozen refresh + full burst build, `registry.cpp:260-285`) runs
  under `_mutex` on EVERY idle/commit trigger (`:1017-1021,
  :1073-1081`), outside any factory budget — so cache the burst per
  (program, epoch, avar-digest, overrides), skipping rebuild when
  the standing burst is still current, and run rebuilds under a
  millisecond slice that aborts to unusable (plain per-frame route,
  `:845-849`) on overrun. Target: stack OnIdle with nothing
  enqueuable costs the ~16 ms refresh floor, not ~205 ms.
- **1.2 Profile the dark matter.** Add profiler scopes for memoize
  sampling+digest, burst prep (`_PrepareWarmBurst`), and per-sweep-time
  factory cost, so the next diagnosis reads the summary instead of
  building probes.
- **1.3 Full-range sweeps under caps and a per-tick sampling budget.**
  Add a persistent full-range cursor plus visited-frame state (today
  each trigger reprocesses a playhead-relative 64-frame vector with no
  memory of warmed ranges); sweep past + future closest-first with
  per-rig in-flight caps and stale-shedding on playhead moves
  (existing `NotePlaybackAdvanced` semantics). Fairness across rigs
  is by cap-occupancy: the queue stays one global priority/FIFO lane
  (`backgroundScheduler.h:344-356`), and per-rig in-flight caps
  (`:406-407`) bound how much one rig can keep ahead of another, so
  every rig under its cap progresses — no rig starves. Cursor contract: a
  frame counts visited only on publish-confirmed completion recorded
  in the 1.4 index — never on scheduler `completed`, which counts
  void closure returns even when frozen evaluation or publish
  declined inside (`backgroundScheduler.h:143`,
  `backgroundScheduler.cpp:580-611`), and an empty factory skips the
  frame without queueing (`:690-691`). The confirmation mechanism is
  outcome-reporting work: `RigExecWarmWork` returns a publish
  outcome (published / declined-invalid / declined-generation /
  threw) instead of void, the worker splits completion accounting on
  it (new published/declined counters beside `completed`), and the
  production closure returns the publish disposition instead of
  `return;` on invalid poses (`registry.cpp:965-975`). That covers
  RAN jobs; never-queued frames get a separate factory skip reason —
  the factory returns a result struct (work + skip reason) instead of
  a bare function, distinguishing freeze-refused (carrying the named
  cause; `_RefreshFrozenSnapshot` keeps the freeze error in the
  session instead of `ignored`, `registry.cpp:252-256`),
  unsampleable (which sampler gate), and D7-exempt
  (`BuildWarmWork` returns empty for all of these today,
  `registry.cpp:825-934`) — with
  per-reason counters beside aggregate `declined`
  (`backgroundScheduler.h:195-197`) and bridge-level last-skip
  exposure. Outcome accounting balances exactly, extending the
  existing rule (`backgroundScheduler.h:168-173`): queued-ever ==
  published + declinedInvalid + declinedGeneration + droppedStale +
  canceled + shed + droppedAtShutdown + queuedDepth + running;
  `completed` ("ran", kept for compat) == published +
  declinedInvalid + declinedGeneration; threw maps to
  declinedInvalid (warn preserved,
  `backgroundScheduler.cpp:583-587`); `declined` (enqueue refusals,
  per-reason skips included) never entered the queue; coalesced /
  upgraded merge without queueing, as today. Shed frames return
  to pending (shedding today only bumps a counter, `:427-449`);
  persistently declined/skipped frames become un-warmable-after-N
  (revisited on edit), never silently pending-forever nor marked
  visited; retired frames re-pend on invalidation. Bound UI-thread
  work
  with a per-trigger sampling budget — max factory invocations (and a
  millisecond stop) per tick, burst-prep slice included (1.1) — since
  the factory samples synchronously before enqueue
  (`backgroundScheduler.cpp:682-692`) and the in-flight cap does not
  bound one tick's sampling. Add a recurring idle driver:
  today nothing ticks at a held playhead (single-shot commit timer
  only, `OnIdle` on frame changes), so full-range warming needs a
  repeating trigger that runs while unwarm frames remain in range and
  sleeps otherwise, with a without-Qt degradation that warms on frame
  changes only. Update the tests that pin
  64/80 burst shapes (`testRigExecImagingFrameCache.cpp:759-783`).
  Keep the 120 ms idle timer discipline from the measurements report
  for commit bursts.
- **1.4 Per-frame state query.** Implement the D5 C API: per-frame
  cached/warming/dirty/uncached, backed by the shared-owned
  completed-per-frame index (fed by publish completions recording
  into shared state, retired by eviction via new affected-key
  reporting, reset by `Clear` — not by proof-map reads; no callbacks
  into bridges, which sessions may destroy under live jobs).
  Per-frame queue visibility (queued/running/declined/shed/canceled/
  completed by frame, via new scheduler transition hooks) and
  per-frame dirtiness are likewise new tracked state (scheduler stats
  today are aggregate counters). Batched (one call per strip repaint),
  no per-frame sampling on the query path. Dirty is global until 2.1
  lands; the API reports that coarseness honestly (all-cached frames
  flip dirty together) rather than faking precision.

### Stream 2 — Partial invalidation and recache across frames

- **2.0 Entry provenance.** Record per-entry dependency data (clusters
  whose computation the entry depends on; proof aliases where one
  entry serves several frames) and integrate retained-state publish
  into the production bridge paths (pose-only today). Retain/rebind/
  publish contract: at publish, the worker retains the slot subset a
  cone re-run reads — the frozen arena carries doubles only while
  wider state (points, matrices, packets) rides with the runner
  (`frozenContext.h:349-357`), so both halves need capture — against
  the retained handle (`frameCacheSparsity.h:104-119`); a re-run job
  rebinds the handle into a fresh worker program (cloned per job,
  `frozenContext.cpp:4158-4160`) and runs clusters through a
  production `RigExecClusterRunner` (`frameCacheSparsity.h:196-219`),
  which today receives only a cluster ID and needs the rebind
  context; partial results merge into a pose and publish through the
  same completion path as full evaluations (`bridge.cpp:1557-1570`).
  Provenance covers weight-object reads too: `WeightPacket` steps
  resolve through `step.object` (`bakedSchedule.cpp:1043-1061`) while
  the index maps paths to avar clusters only, so record which steps
  read which weight objects/packets and map weight-prim edits through
  it. Provenance also records each entry's constant-region read set
  (regions defined by the D3 epoch-constant sampler): carry-over
  across a constant-namespace change is sound only for entries that
  read no patched region. Without this there is nothing
  cluster-granular to retire or re-run.
- **2.1 Path-scoped retirement.** Rescope the epoch digest first (D3):
  without it every value patch evicts the world. Then build the
  missing notice adapter: normalize USD notice paths to
  `RigExecControlId`s (sampled path strings / override identities),
  extend index coverage to every cone-source family — chain, solver,
  revision, native-source, delta, constraint-array, varying/override
  (`bakedProgramImpl.h:948-972`), not just the provider slots `Build`
  derives — populating `MapControl` (which has no production callers
  today) so unknown controls stop collapsing to all clusters, add a
  known-empty representation distinct from unknown (empty and missing
  both expand to all clusters today,
  `outputAffectedIndex.cpp:82-95,150-155`, asserted by
  `tests/testRigExecFrameCacheSparsity.cpp:200-212`): Build records
  the control universe — every control ID the extended family walk
  yields (sampled paths across all cone-source families plus
  override identities) — with today's cluster--1 slots stored as
  seedless members (`outputAffectedIndex.cpp:53-66`) instead of left
  unmapped, and AffectedByControls branches three ways (collapsing
  two today, `:149-156`): foreign ID → all clusters; universe
  member → cone closure of its seeds, possibly empty (retires
  nothing); non-control paths never reach the index at all
  (normalize-to-nothing yields no controls). MapControl admits its
  control to the universe — the dynamic-override bootstrapping path —
  and cover
  resolved-asset resync paths, which the evaluator honors
  (`rigEvaluator.cpp:1571`) but the registry notice scan never reads
  (resynced/info-only only, `registry.cpp:1634-1708`). Extend the
  planner's time rule to match the executor: at a moved time the
  planner unions `Always()` only
  (`frameCacheSparsity.cpp:227-255`) while the executor also dirties
  varying-step and override-step clusters
  (`bakedSchedule.cpp:1444-1469`), so `Build` captures
  varying/override cluster closures as new index accessors beside
  `Always()` (`outputAffectedIndex.h:122-124`) and the planner
  unions them at moved time / standing overrides. Then map
  changed paths through the output-affected index to dirty clusters
  and retire only entries / proofs whose computation touched them —
  interposed on the evaluator's three notice outcomes, which needs a
  new evaluator outcome query consumed by the notice adapter: the
  registry never observes the branch today (no calls to
  `IsInvalidatedBy`/`ApplyAvarValueEdits`/`BumpProgramStamp` anywhere
  in `registry.cpp`), so the evaluator must report per-notice
  disposition — patched (plus patched paths), stamp-bumped, or stale
  (`rigEvaluator.cpp:1676-1700`) — for the adapter to branch on:
  default-only avar patches retire exactly the patched avars'
  clusters (the program already re-runs only their cone live, with no
  stamp bump); stamp-bump value edits retire affected clusters and
  re-resolve; `IsInvalidatedBy` overlap still rebuilds the program
  (`bakedProgram.cpp:921-975`), the epoch moves, and epoch-half
  eviction + generation cancel stand. The adapter then partitions the
  1.4 index against dirty clusters / weight objects / constant
  regions: clean entries carry over under the new key (D3, zero
  recompute), dirty entries retire for 2.2 re-runs. Cancellation is
  scoped to match: patch/stamp outcomes purge queued jobs for
  affected times only (new `CancelGenerationTimes`, same loop shape
  as `CancelGeneration` / the shed pass,
  `backgroundScheduler.cpp:244-253,427-434`, via the
  `_queuedByRigAndTime` index) without bumping the generation — and
  since old and requeued jobs would then share one generation token,
  the purge assigns affected times fresh per-time fence tokens
  (monotonic rig-local counter, held in the shared sidecar):
  `RigExecWarmRequest` carries the token
  (`backgroundScheduler.h:126-133`), and the publish fence requires
  token match beside the generation check (`bridge.cpp:1566-1569`),
  under the same atomic check-and-insert protocol as 3.3 — an old
  job for a requeued time drops on token mismatch and can never
  overwrite the new result, while unaffected times keep their tokens
  and their running jobs publish normally; global
  `CancelGeneration` stays for epoch moves only. The registry notice
  path's blanket per-notice cancel (`registry.cpp:1714-1729`) is
  replaced by this outcome-driven rule.
- **2.2 Re-resolve, don't just drop.** The whole-vector digest rekeys
  any frame whose sampled inputs moved (`frameCache.cpp:385-405`),
  and a namespace move rekeys every frame, so each frame lands in
  exactly one lane: (a) same namespace, digest equal — the entry
  stands, zero work (an edit to an unrelated control must not churn
  the cache); (b) namespace moved but 2.0 provenance clean —
  carry-over: re-publish under the new key, evict the old, re-point
  the 2.3 proof, zero recompute (a failed re-publish degrades the
  frame to lane (c) rather than stranding it); (c) dirty — re-execute
  the dirty
  cones against retained state (2.0) and publish the merged pose
  under the NEW key. Provenance selects the reuse base, the lane,
  and the dirty set; the runner produces the entry only in lane (c).
  Affected-time computation replaces the fixed sweep: intersect the
  1.4 completed-frame index with entries whose 2.0 provenance touches
  dirty clusters, and requeue exactly those times (today a commit
  cancels the whole rig and re-enqueues a fixed 64-time sweep,
  `registry.cpp:1026-1036,1714-1729`). Retired frames whose partial
  re-execution declines (observed via the 1.3 work-outcome return,
  not via `completed`) re-enter the warming queue at their prior
  priority for full re-warm; declined-generation outcomes requeue
  only under the live generation.
- **2.3 Proof scoping per frame-set.** Proofs today store only
  `(default, time) -> controlDigest` (`bridge.h:409-424`), recorded at
  enqueue or memoize time (`bridge.cpp:1587-1611,1799-1838`), with no
  record of which sampled inputs the digest covers — so affected-set
  retirement is unrepresentable. Capture the sampled-path dependency
  set with each proof at record time, reusing the existing level-1
  per-input digests the control digest already chains
  (`RigExecSampleDigest`, `frameCache.h:143-148`); then retire proofs
  intersecting the affected input set rather than clearing wholesale
  past `_kFreshDigestCap` behavior, and re-point surviving proofs
  under the new key on namespace moves (the 2.2 lane-(b)
  carry-over). Keep the bit-identity guarantee (unproven frames
  evaluate live).
- **2.4 Override/drag interplay.** Drags keep the interactive bypass
  (`bridge.cpp:1965-1973`); on release, the commit burst re-warms
  affected frames under the new values (existing trigger, cluster-
  scoped enqueue).

### Stream 3 — Timeline cache strip in usdview

- **3.1 Cache strip dialog.** New modeless dialog (profiler-panel
  pattern) rendering per-frame states over the stage range via the
  1.4 API. Updates are poll-based — repaint on the 1.3 recurring
  driver tick and on `SetTime`, skipping repaints while the
  completions counter is still (there is no worker-completion
  callback into the plugin, and the plugin today observes only stage
  replacement and frame changes). States: cached / warming
  (queued+running) / dirty / uncached.
- **3.2 Playhead/range tracking.** Strip follows `currentFrameChanged`
  and stage replacement (existing signal pattern,
  `rigExecUsdview.py:307-311`); range from stage start/end timecodes.
- **3.3 Manual controls.** Clear-cache and warm-range actions on the
  panel, so artists can force the states the strip shows. Requires new
  C bindings: `ClearFrameCache` is C++-only and the triggers take no
  frame arguments, so add e.g. `RigExecImaging_ClearFrameCache(rig)`
  and `RigExecImaging_WarmRange(rig, frames[])` (explicit frame list,
  not the playhead sweep) alongside the panel. Cancellation/fencing
  lives in the shared `ClearFrameCache` path itself — not just the
  new binding: bare `ClearFrameCache` (`registry.cpp:1159-1167`,
  `bridge.cpp:1574-1584`) leaves queued and running jobs whose
  completions would repopulate the cache at once, and the existing
  weight-overlay path clears the bridge cache BEFORE it cancels
  (`bridge.h:162-168`, then `registry.cpp:1197-1208`) — a worker can
  publish a stale-overlay pose into the cleared cache between the
  two, and the overlay moves pose content without moving the key
  (`bridge.h:164-166`). So `ClearFrameCache` becomes cancel-THEN-
  clear plus 1.4 index reset, in that order — and cancel-then-clear
  alone is NOT atomic: a running job can pass the generation check
  (`bridge.cpp:1566-1569`) before the bump and insert after the
  clear (`:1570`), repopulating retired data. Worker publish and
  cancel/clear therefore share a fence mutex: the production
  closure performs fence-check (generation + per-time fence token)
  and cache insert atomically under it, and the fenced clear holds
  it across generation-bump + queue-purge + cache-clear +
  index-reset.
  The memoized UI-thread publish path needs no fence (the UI thread
  serializes with itself). Lock discipline: fence outermost, then
  scheduler `_mutex`, then cache shards — never inverted; and
  `ClearFrameCache` holds the registry `_mutex` (`:1161`) while
  `CancelFrameGeneration` locks it too (`:1115`), so the fenced
  entry cancels in its own scope BEFORE locking for the clear —
  never cancel-under-clear-lock (self-deadlock on the non-recursive
  mutex). The C binding, the overlay path, and existing callers all
  route through the fenced entry; the strip would otherwise keep
  showing the repopulated frames' stale cached states.

## Validation Plan

- **0.x:** `ctest -R 'testRigExecFrozenContext|testRigExecFrameCache|testRigExecBakedSchedule'`; new tests freeze each shipped example and assert bit-identity warmed-vs-live (extend `CheckPosesBitIdentical` coverage); `Biped_stack_anim.usda` must freeze with no refusal error. Refusal observability is pinned: tests drive a refusing rig and assert the named cause (e.g. the weight-object op) surfaces in profiler/stats (not swallowed), factory skips report freeze-refused-with-cause vs unsampleable-gate vs D7-exempt distinctly, and per-reason counters move while aggregate `declined` still balances. The 0.6 matrix ships as a doc table plus one test per row; cascade rows are pinned explicitly: 04/12/13 freeze once 0.1 lands and 05/ArmRig once 0.4's driver gate lands, and the blendShape/curvenet/ribbon/read-phase/emitGuidePoints ops they newly reach get frozen-parity tests (no existing suite runs the frozen executor over them); the 0.7 harness entries prove the stack (not just the flat rig) in benches and frozen parity. Highest-risk validation: blend-shape + weight-object frozen parity on the face (new kernels, subtle float behavior).
- **1.1-1.2:** `benchPlayback examples 8` (flat behavior unchanged: ~6/~11 ms ticks); scrub probe over stack shows OnIdle-with-nothing-enqueuable at the ~16 ms floor; a second consecutive OnIdle pays no burst rebuild (cached burst); an over-slice prep aborts to unusable and the tick takes the plain route (assert via burst-usable flag, no tick blowout); profile summary contains the new scopes with non-zero totals.
- **1.3-1.4:** cursor coverage, sampling-budget compliance and warming publish growth on the stack are proven by an automated ctest (registry-level idle driver over `Biped_stack_anim.usda`, asserting `published` grows, every in-range frame is visited, no trigger exceeds the sampling budget, and a declined/invalid frame never counts visited (requeued or un-warmable-after-N); outcome-counter assertions extend the `ImagingFrameCache` ctests). Priority/progress under real workers gets its own scheduler ctest (workerCount>0, not the workerCount=0 priority test, `tests/testRigExecBackgroundScheduler.cpp:354-385`): closest-first visit order, neighbor-before-sweep under load, no rig starves (two-rig progress test: both rigs complete within bounded pops under cap-occupancy fairness), and the full 1.3 balance equation holds (queued-ever == published + declinedInvalid + declinedGeneration + droppedStale + canceled + shed + droppedAtShutdown + queuedDepth + running). New C API returns correct states against a scripted SetTime/warm/drain sequence, including across a constant-namespace change (carried frames still cached, retired ones warming/uncached); `verify_binary_*` suites stay green. Benches stay manual perf lanes: parameterized `benchPlayback` (0.7) for tick timings, `benchFrameCacheWarm` for kernel checks (evaluator + local cache, no imaging linked, built-not-registered, asserts nothing — its warm passes report lookup misses as data, never comparing a live reference, `tests/benchFrameCacheWarm.cpp:379-461`; accept on zero warm misses/invalid, sane cold/warm distributions and speedup ratio).
- **2.x:** new tests: entry provenance records the clusters each entry depended on; one case per 2.1 evaluator branch — patched (exact patched-avar clusters retire), stamp-bumped (affected retire + re-resolve), stale (rebuild path stands) — plus resolved-asset resync, known-empty (seedless universe member retires nothing) vs unknown (foreign ID retires conservatively), non-control paths (normalize-to-nothing yields no controls), and constant-state change (patch opens a new constantDigest namespace; no aliasing). Planner/executor parity cases: time-only change dirties varying+override cones, not just always-dirty; standing overrides dirty their cones. Carry-over case: a constant patch touching a subset of regions serves the untouched entries with zero evaluator pulls (pull counters + pose identity), while touched entries re-run cones only. Scoped-cancel case: an edit affecting some times leaves other times' queued/running jobs to publish, and only affected times requeue; a gated old job for an affected time (paused pre-publish across the purge+requeue) drops on fence-token mismatch and never overwrites the new result. Edit one control, assert only affected frames retire (proof/entry accounting), re-warmed frames bit-match live, unrelated edits churn nothing; `testRigExecBakedMode` edit tests and `Cones_*` suites stay green.
- **3.x:** headless panel-model tests (mirror `profilerModel` headless pattern); clear-while-warming test: clear with jobs queued/running, then drain — cache stays empty (late completions fenced, no repopulation) and the 1.4 index resets; deterministic race test: a test closure pauses between fence-check and insert while the main thread clears, then resumes — the insert drops and the cache stays empty, proving check-and-insert atomicity; overlay test: set overlay mid-warming, assert no stale-overlay pose is served (cancel-before-clear order); manual: `bin\launch_usdview.bat examples\biped\Biped_stack_anim.usda`, scrub and watch states flip cached→dirty→warming→cached; clear/warm actions visibly recolor.
- **E2E:** full `ctest -R '^testRigExec' -E 'Cones_|ExampleParity'` gate plus the full `example_parity_*` and `verify_binary_*` families (`ctest -R 'example_parity_|verify_binary_'`; one entry per baking fixture, `CMakeLists.txt:479-492,666-677`, DISABLED rows skip) — the flat and 0.7 stack anim entries are the named must-pass subset, not the whole run.

## Risks / Rollback

- Frozen-executor float parity (new kernels drifting from live): mitigate
  with bit-identity tests per op before enabling; rollback is the refusal
  gate (unimplemented op declines, rig evaluates live as today).
- UI-thread sampling budget: full-range sweeps multiply UI-thread
  sampling, and the in-flight cap does not bound one tick's factory
  sampling; mitigate with the 1.3 per-trigger sampling budget (max
  invocations + millisecond stop) + closest-first + the 1.1 fast-fail;
  `RIGEXEC_FRAME_CACHE=warm-off`/`off` remain instant rollbacks.
- Proof-scoping bugs aliasing frames (plausible wrong poses): mitigate
  with the existing shadow-verify mode plus new aliasing tests; any
  doubt resolves to live evaluation, never to serving. Same guard for
  the epoch rescope: avar constants leaving the epoch digest must land
  in constantDigest key coverage first (D3), or two constant-states
  share a key — the aliasing tests cover that transition explicitly.
- usdview API drift across USD upgrades: the strip uses only
  `configureView`/dataModel signals plus the repo-owned C API; no
  stock-widget surgery, so upgrades can only break the panel, never
  the timeline.

## Open Questions

- **Timeline palette and placement.** Proposed assumption: modeless
  dialog like the profiler panel, with cached green / warming blue /
  dirty red / uncached grey. Confirm or adjust — cosmetic, does not
  change the design.
- **Per-tick sampling budget number.** Policy decided (1.3): per-trigger
  factory-invocation cap plus a millisecond stop, closest-first, full
  range reachable over idle, never blocking the tick. Scale anchors
  (not the answer): the flat-rig commit burst samples 16 neighbors in
  ~1.2 ms at enqueue, with ~25 ms of background drain on two workers
  (`backgroundScheduler.h:66-69`); stack samples cost ~3 ms cached /
  ~48 ms full per the measured baseline. Proposed assumption: cap 8
  factory invocations per tick with a millisecond stop in the low
  tens, tuned by measurement on the stack. If a range wider than a few
  hundred frames must pre-warm eagerly rather than over idle, say so.
