## Goal

Build a per-frame caching system for RigExec (branch `feature/per-frame-cache`,
worktree `D:/work/usdRig-framecache`) in the spirit of the Premo brief's
Premonition + pose-cache behavior: scrub and playback read cached frames,
edits trigger sparse recalculation of dirtied nodes only, and all cache
fill and neighbor warming runs asynchronously on lower-priority background
threads that never block the UI thread.

## Success Criteria

- A frame change to an already-evaluated frame publishes without running
  the evaluator on the calling thread (cache hit path).
- A frame change to a cold frame, or an edit at the playhead, evaluates
  live on the calling thread exactly as today and is never slower than
  today: the viewport never shows a neighboring frame's pose in place of
  the requested one.
- A control/value edit recomputes only affected frames, and within an
  affected frame only the cone downstream of what changed; unaffected
  cached frames are returned untouched (measured, not assumed).
- The calling (UI) thread never waits on cache fill or warming work:
  background jobs are cancellable, run below normal priority, never
  take the registry serialization mutex, and never read the USD stage.
- Every cached pose is bit-identical to a live evaluation of the same
  inputs (the existing `dynamic == baked == reference` parity gate
  extends to `cached == live`).
- Memory is bounded: per-rig byte cap with LRU eviction; cap defaults
  follow Stream 0 measurement.
- The repo's CI gate stays green, including new `testRigExec*` cache
  tests.

## Context And Current Facts

- Engine shape: `RigExecRigEvaluator::Evaluate(UsdTimeCode)` returns one
  `RigExecRigPose` (`usdRig/libs/rigExec/rigEvaluator.h:270-271`).
  Modes are Baked (request), Dynamic (reference), BakedWithParityCheck
  (`usdRig/libs/rigExec/bakedProgram.h:43-57`).
- Per-frame sparsity already exists *within* one frame: the baked step
  graph runs only the closure of value-changed sources over clusters
  (`docs/specs/baked-step-graph.md` section 7; executors in
  `bakedSchedule.h`; `RIGEXEC_BAKED_VERIFY_CONES` shadow-verifies cones).
  There is no *cross-frame* reuse: every frame change re-evaluates.
- Per-frame inputs are read inside the baked run: varying inputs
  through retained `UsdAttributeQuery`s, and chain-resolved inputs
  through the generation's resolved inputs, the same route the dynamic
  path takes (`libs/rigExec/bakedProgramImpl.h:188-201`). Interactive
  overrides are not authored to the stage; the evaluator places them on
  the program per generation (`SetOverrides` in `Evaluate`,
  `rigEvaluator.cpp`). Both are inputs a cache key must see.
- The dynamic path runs OpenExec against the live stage. It has no
  isolated form and cannot run concurrently with stage edits.
- The imaging path is synchronous evaluate-then-publish, serialized by
  one registry mutex (`registry.h:56` `SetTime`, `registry.h:257`
  `_mutex`, lock-order rules `:264-270`). No async boundary exists.
- Hydra already reads immutable generations through an atomic current
  snapshot (`libs/rigExecImaging/snapshotStore.h:1-8`); the results
  scene index never computes or waits.
- `.rigexec` playback (`playback.h`) is the closest existing
  "serve without evaluating" path: nearest-frame mapping, stage edits
  never dirty it, and its standing rule applies here too — an
  unevaluated result is recoverable, a plausible wrong one is not
  (`playback.h:72-78`).
- `docs/plans/evaluation-engine-gaps-vs-premo-libee.md` §4.2/§5.2
  already scopes this exact project (P1: multi-frame throughput, then
  Premonition-lite) and names its two unresolved requirements:
  (a) isolated evaluator/program state for background execution, and
  (b) cancellation/generation checks. It also prescribes the store key
  `(time, control-state-digest)`, a per-rig byte cap with LRU, and
  measurement of per-frame store bytes before sizing anything.
- The Premo brief itself (local PDF, read in full 2026-09-18):
  Premonition precomputes adjacent frames "without waiting for idle
  time" (pp. 17, 33, 40-41); task-list caching for repeated edits
  avoids dirty rewalks (pp. 9, 32); external dirty / input-change /
  output-dependency lists with lazy dirty ∩ affecting(requested)
  (p. 10); multi-graph throughput is distinct from the Premonition UX
  (p. 18); frame-order policy, interruption, and memory budgeting are
  unpublished (Gaps #4, p. 26).
- Test convention is plain C++ with `main` + `CHECK` macros over
  in-memory stages (e.g. `tests/testRigExecStaticInputCache.cpp`),
  notably its THREAD rule: single-threaded state with no lock must
  refuse cross-thread reads rather than race (`:22-25`).
- CI gate (`usdRig/.github/workflows/usdrig.yml`): configure, Release
  + Ninja build with `rigExecPose` smoke, `ctest -R '^testRigExec'
  -E 'Cones_|ExampleParity'`, install check. MSVC/macOS are
  locally-verified only; per-test TIMEOUT is 120 s.

## Constraints And Non-goals

- The byte-identical parity gate is non-negotiable; cached frames join
  it, and anything that cannot prove bit-identity stays on live eval.
- USD remains the authoring representation; no new file format and no
  on-disk cache in this project (persistence is a follow-up, if ever).
- All compute stays C++ with no Python on any compute path (existing
  rule, `bakedProgram.h` lineage + GIL-release design).
- Threading stays on `pxr/Work` (TBB) for kernel-level parallelism; the
  project adds a small dedicated background pool for *frame* jobs only,
  plus environment kill switches. No new third-party threading
  dependency.
- A pose must be a pure function of its sampled inputs. Stream B
  audits every kernel and solver for state carried between
  evaluations; any history-dependent node disqualifies its rig from
  the cache (falls back to live eval) until the node is made pure.
- Out of scope: incremental epoch recompile / LibEE-2 sync rules (P3,
  benchmark-first), `HierarchyFrames` bundles, FBX/glTF ingest,
  skin-cluster cutting, profiler UI. These stay exactly where the gap
  analysis put them.
- Source note: the Premo brief was read in full after the first draft
  (local PDF, 42 pages, 2026-09-18) and confirms the gap analysis as
  proxy — checklist, Premonition behavior, and dirty/task-list caching
  all match. Its deltas are folded into D2, D4, D5, and Streams C-F;
  no stream was removed.

## Brief ↔ plan mapping

The brief sentence "Graph System manages LibEE through
interfaces and caches" is this plan's FrameCache +
BackgroundScheduler + frozen contexts: the store, the queue
in front of it, and the isolated per-job state that lets
background threads evaluate without touching live state.
The brief's "subscription" metaphor is Stream E's publish
fence: background completions publish into the cache only,
behind a generation check, and never into the snapshot the
viewport reads -- a subscription that can deliver fresh
results but never a stale frame. The frame-order policy --
the playhead always live, scrub neighbors (+-1..N) first,
then the Premonition-style sweep, triggered by edit-commit
rather than idle -- is original design (D5): the brief's
Gaps #4 (p. 26) confirms frame-order policy, interruption,
and memory budgeting are unpublished, so there is no public
policy to match.

## Key Decisions

- D1 — Cache at pose level, in `libs/rigExec`. The unit stored is a
  `RigExecRigPose` generation (plus, only if Stream 0 shows it
  affordable, the per-frame slot arena needed for sparse reuse), keyed
  by `(bindingEpochDigest, controlStateDigest)` where the digest covers
  every source value the frame actually reads, including interactive
  overrides. Time is NOT part of the key; it is stored on the entry as
  a warming hint and for stats. Because the digest is recomputed from
  the stage at lookup, an edit to a control's curve changes the digest
  at every affected frame by construction: stale entries become
  unreachable and LRU reclaims them. No frame-level invalidation
  bookkeeping is needed for correctness; the affected-frame
  computation in D2 exists only to decide what to re-warm. Rejected:
  caching published Hydra prims (multiplies variants across weight
  overlays and guide toggles) and caching inside the imaging bridge
  (wrong layer; the evaluator owns invalidation knowledge).
- D2 — Sparse cross-frame recalc reuses the baked cone machinery,
  gated on measurement. Cluster-level reuse needs a full slot arena
  retained per cached frame, not just the pose; Stream 0 measures
  arena bytes separately from pose bytes, and if arena retention does
  not fit the byte cap on the biped at a useful frame count, Stream D
  ships whole-pose memo plus fast live re-eval instead (the 0.6-2.7 ms
  baked frame makes that a credible endpoint). When retained: a
  cached frame keeps its source values; on lookup the request's
  sources are compared by value, and a frame whose cone is empty is a
  hit. A partial cone re-runs only affected clusters against
  otherwise-retained slots. New work is the cross-frame state
  retention and the output-affected (downstream) index at cluster
  granularity — the gap analysis notes no output-dependency list
  exists today. The affected-set computation is itself memoized per
  (control, epoch) — the brief's task-list cache (pp. 9, 32) — so
  repeated edits of one control re-run a cached selection without
  rewalking. Epoch-level admission still goes through the baked
  capture index: a notice that hits it invalidates the epoch, a miss
  leaves every cached frame standing (their digests decide
  reachability, per D1).
- D3 — Background jobs never touch live evaluator state, the
  registry mutex, or the USD stage. The UI thread samples every
  per-frame input a job needs (varying attribute queries,
  chain-resolved inputs, interactive overrides) for each enqueued time
  at enqueue time, and packs them into a frozen, per-job evaluation
  context (epoch-pinned immutable program + private slot arena +
  per-time input vectors). Workers evaluate from that vector only; the
  Stream B audit proves no code path from a worker reaches a
  `UsdStage`, `UsdAttribute`, or `UsdAttributeQuery`. The UI-thread
  sampling cost per frame is measured in Stream 0 and bounds the
  neighbor radius. A generation token taken at enqueue time is
  rechecked before publish, and a mismatch drops the job's result.
  Fail-closed: any inconsistency discards the entry and falls back to
  live evaluation.
- D4 — Lower priority means a dedicated pool, not arena priority.
  Frame jobs run on their own small pool (default sized by Stream 0
  measurement, 2-4 workers expected) with each worker set to
  below-normal OS thread priority through the platform thread API,
  and each worker runs the baked *serial* executor end to end.
  Workers never call `Work*`/TBB: a kernel-level parallel call would
  run its tasks on the shared arena at normal priority, leaking the
  work back past the priority boundary at exactly the moment it should
  hold. The frozen context therefore forces the serial kernel variants
  per context (the same gating `RigExecParallelEvaluationEnabled()`
  applies process-wide today). `tbb::task_arena` priority was
  rejected because it only orders tasks among TBB's own workers and
  cannot yield to the UI thread, which is not a TBB worker; OS thread
  priority can. Concurrency with the UI is bounded by the small worker
  count. This is the mechanism that reconciles the brief's "uses
  multicore aggressively" with "non-disruptive" (p. 17): below-normal
  priority runs hot when the UI is idle and yields when it is not.
  Priority inversion is designed out rather than tolerated: no lock a
  worker can hold is ever held across evaluation or allocation (see
  Stream A's lock rule).
- D5 — The playhead is always live; the pool only warms. A miss at
  the requested frame, and every evaluation during an edit at the
  playhead, runs on the calling thread exactly as today (sparse baked
  run, ~1-3 ms on the measured rigs) and its result is published into
  the cache. The pool never serves the playhead. Rejected: routing UI
  misses through the pool with "show last good meanwhile". That is a
  regression for the primary interactive case (each drag tick would
  bump the generation, cancel, re-enqueue on deprioritized workers,
  and lag the mouse), and it violates the playback rule this plan
  adopts: frame T-1's pose displayed at frame T is a plausible wrong
  answer, not a recoverable unevaluated one. Background order is then:
  scrub neighbors (±1..N of the play head); then the Premonition-style
  sweep of the surrounding range.
  Warming is triggered by edit-commit (drag release / value commit),
  not by idle detection — the brief states Premonition "doesn't wait
  for idle" (pp. 17, 33, 40-41); an idle signal remains only as a
  secondary trigger. Any new edit bumps the generation and cancels
  in-flight jobs for that rig. The frame-order policy itself is
  original design: the brief's Gaps #4 (p. 26) confirms no public
  policy exists to match.
- D6 — Switches: `RIGEXEC_FRAME_CACHE={on,off,warm-off}` (default
  on), honoring `RIGEXEC_ENABLE_PARALLEL_EVAL=0` by disabling
  background fill while still serving cache reads, and
  `RIGEXEC_FRAME_CACHE_VERIFY=1` as a `RIGEXEC_BAKED_VERIFY_CONES`
  analogue (shadow live-eval comparison of cache hits).
- D7 — Dynamic-path rigs (bake refusals) get read-only caching: the
  UI thread's own live evaluations are memoized on the same key and
  served on hit, but there is no background fill for them, because the
  dynamic path drives OpenExec against the live stage and cannot run
  on a worker under D3. Cluster reuse is baked-only, matching the
  existing refusal contract.

## Recommended Approach

Build in three phases so the high-risk threading design (gap §5.2's
unresolved requirements) lands before any imaging behavior changes:

1. Phase 1 — Contracts + isolation + store (Streams 0, A, B). Merge
   the `FrameCache` / `BackgroundScheduler` / frozen-context
   interfaces and their unit tests first; everything else codes
   against them.
2. Phase 2 — Fill + sparse reuse (Streams C, D). Background pool,
   priority queue, cancellation, neighbor warming, cross-frame cone
   reuse with shadow verification.
3. Phase 3 — Integration + hardening (Streams E, F). Warming results
   published into the imaging chain behind a generation fence,
   scrub-from-cache, CI, benches, docs.

Streams within a phase run in parallel; stream contracts below are
the merge order. All new code lives on `feature/per-frame-cache` in
`D:/work/usdRig-framecache`.

## Work Plan

### Stream 0 — Measurement + interface skeleton (solo, first; unblocks all)

- Measure per-frame stored bytes on the biped and 9-mesh rigs, pose
  maps and slot arenas REPORTED SEPARATELY, so D2's arena-retention
  decision is made on numbers; measure baked serial frame cost as the
  warming budget; measure the UI-thread cost of sampling one frame's
  input vector (D3), which bounds the neighbor radius; propose the
  default per-rig byte cap, worker count, and neighbor radius from
  numbers.
- Land skeleton headers with the contracts Streams A–E implement
  against (signatures below are the plan's; Stream 0 may adjust
  names but must keep the semantics):
  - `FrameCacheKey{epochDigest, controlDigest}` (time carried on the
    entry, not the key), `FrameCache::Lookup/Publish/Evict/Stats`
    (byte accounting).
  - `FrozenEvalContext` (epoch-pinned, no live pointers, no stage
    handles) + `FrameInputs` (one sampled input vector per time,
    built on the UI thread) + `EvaluateFrozen(ctx, inputs) ->
    RigExecRigPose`.
  - `BackgroundScheduler::Enqueue(time, priority, generation)`,
    `CancelGeneration(rig)`, generation token source.
- Files: `libs/rigExec/frameCache.{h,cpp}`,
  `libs/rigExec/frozenContext.{h,cpp}`,
  `libs/rigExec/backgroundScheduler.{h,cpp}` (skeletons + key/stats
  types), `reports/frame-cache-measurements.md`.
- Done: numbers published, headers merged, empty-impl unit tests
  compile in the existing CTest gate.

### Stream A — FrameCache store (needs Stream 0)

- Implement the `(epochDigest, controlDigest)` store: exact lookup,
  LRU eviction under a per-rig byte cap, stats (hits, misses,
  evictions, bytes), thread-safe for concurrent lookup during
  background publish (fine-grained or shard lock — never the registry
  mutex). Lock rule: a lock is held only for a pointer swap or a map
  edit, never across evaluation, allocation of an entry's payload, or
  digest computation, so a below-normal worker can never stall a
  UI-thread lookup (priority inversion designed out, D4).
- Define `controlStateDigest`: hash over the frame's source values
  actually read (authored controls, time-varying inputs, AND the
  evaluator's interactive overrides, which never reach the stage), so
  a scrub between identical control states hits across times and a
  drag never serves a pre-drag pose.
- Own the "drop, don't guess" rule: corrupt/undersized entries are
  evicted, never partially served.
- Files: `libs/rigExec/frameCache.{h,cpp}`,
  `tests/testRigExecFrameCache.cpp` (admission/typing/thread rules
  in the `testRigExecStaticInputCache.cpp` style).
- Done: unit tests for keying, digest sensitivity (control change
  misses, unrelated edit hits), LRU + cap accounting, concurrent
  lookup/publish stress.

### Stream B — Frozen evaluation contexts + generations (needs Stream 0)

- Implement the epoch-pinned, per-job evaluation context: per-time
  sampled input vectors (built on the UI thread from attribute
  queries, chain-resolved inputs and overrides) + private slot arena +
  reference to the immutable baked program, with serial kernel
  variants forced per context; prove no path from a worker reaches
  the live evaluator, the stage, or any `Usd*` handle (document the
  audit; THREAD-rule test).
- Purity audit: enumerate every kernel and solver and confirm none
  carries state across evaluations; a node that does marks its rig
  cache-ineligible until fixed (Constraints).
- Implement generation tokens: registry/evaluator bumps per
  rig-affecting edit; jobs check at start and before publish.
- Cover D7: dynamic-path rigs memoize UI-thread results through the
  same publish path; assert that no background job is ever created
  for a refusal rig.
- Files: `libs/rigExec/frozenContext.{h,cpp}`,
  evaluator/bridge hooks for generation bump,
  `tests/testRigExecFrozenContext.cpp` (equivalence with live eval
  across modes; no-aliasing/thread tests).
- Done: frozen-eval bit-equals live eval on the biped; refusal rigs
  hit only from UI-thread memo; generation mismatch demonstrably drops
  results; purity audit recorded.

### Stream C — Background scheduler, low-priority pool, warming (needs A+B)

- Dedicated pool: fixed workers, below-normal OS priority on each
  platform's thread API, baked serial executor per job,
  `RIGEXEC_FRAME_CACHE` + `RIGEXEC_ENABLE_PARALLEL_EVAL` switch
  semantics (D6/D4).
- Priority queue per rig: neighbors ±1..N > edit-commit sweep (the
  playhead is never queued, D5); coalescing of duplicate times;
  per-rig in-flight cap.
- Cancellation: generation check before run and before publish;
  dropped jobs free their arenas promptly.
- Trigger + backpressure: the primary trigger is an explicit
  `OnEditCommitted()` call from the imaging/registry layer on drag
  release / value commit (no timers inside the scheduler); an idle
  signal is secondary. During active playback advance the sweep
  deprioritizes and skips frames the playhead already passed.
- Files: `libs/rigExec/backgroundScheduler.{h,cpp}`,
  `tests/testRigExecBackgroundScheduler.cpp` (ordering,
  coalescing, cancellation, trigger semantics, priority smoke via
  observed progress-under-load rather than asserting OS priority
  values).
- Done: ordering, coalescing, cancel-on-edit, edit-commit trigger,
  and playback shedding asserted deterministically (counts and
  sequences, no wall-clock); the UI-eval latency comparison with
  warming on vs off is a Stream F bench, not a gate test.

### Stream D — Sparse cross-frame reuse (needs A+B; parallel with C)

- Go/no-go from Stream 0's arena-bytes number (D2). No-go: ship
  whole-pose memo only, keep the output-affected index for warming
  selection, and skip the rest of this bullet list.
- Go: retain per-cached-frame source values + slot/cluster state so a
  lookup can compute the dirty cone for the requested inputs; re-run
  only affected clusters (reuse `bakedSchedule` cone/edge data — no
  duplicate dependency derivation).
- Build the cluster-granularity output-affected index the gap
  analysis flags as missing; it selects which cached frames to
  re-warm after an edit (correctness needs no invalidation, D1); wire
  the baked capture index to epoch invalidation (hit → drop epoch's
  frames; miss → keep).
- Task-list cache: memoize the affected-cluster set per (control,
  epoch) so repeated edits of one control re-run the cached
  selection without rewalking; topology change invalidates the memo.
  Represent dirty ∩ affecting(requested) as bit arrays over
  clusters (the US9,135,739 embodiment, brief p. 10).
- Implement `RIGEXEC_FRAME_CACHE_VERIFY=1` shadow mode: every cache
  hit also live-evaluates and diffs via `RigExecComparePoses`,
  reporting mismatches as diagnostics (Cones-style suite; the test
  name carries the `Cones_` substring so the existing CI exclusion
  pattern catches it without edits).
- Files: `libs/rigExec/*` (cache/cone extensions),
  `tests/testRigExecFrameCacheSparsity.cpp` (+
  `testRigExecFrameCacheCones_Verify.cpp` for the shadow suite).
- Done: control edit recomputes a strict subset of clusters on
  affected frames and zero work on unaffected frames (asserted via
  executed-cluster counts); repeated edits of one control reuse the
  memoized selection; shadow suite clean on biped + 9-mesh.

### Stream E — Imaging integration: async publish + scrub-from-cache (needs C+D)

- `SetTime`/frame-change fast path: cache hit publishes the stored
  generation through the existing atomic snapshot without evaluating;
  miss evaluates live on the calling thread as today and publishes
  the result into both the snapshot and the cache (D5). Background
  completions publish only into the cache behind the generation
  fence, never into the snapshot; stale completions are dropped.
- The synchronous path remains the whole story when the cache is off,
  empty, or bypassed (playback sessions unchanged — a `rigExec:asset`
  rig never consults the frame cache).
- Forward new-edit signals to generation bump + cancel; expose
  `OnEditCommitted()` (drag release / value commit) plus a secondary
  idle signal from the usdview-side frame loop.
- Files: `libs/rigExecImaging/{registry,bridge}.{h,cpp}`,
  `plugin/rigExecUsdview` frame-change hook,
  `tests/testRigExecImagingFrameCache.cpp` (+ extend
  `testRigExecImagingPlayback.cpp` patterns for scrub reads).
- Done: scripted scrub across a warmed range performs zero evaluator
  runs; scrub into cold frames evaluates live with latency equal to
  the cache-off path; a drag at the playhead never enqueues a job for
  the playhead frame.

### Stream F — CI, benches, profiler, docs (runs throughout; final gate)

- Wire new tests into CMake/CTest following the existing
  `testRigExec*` registration; keep the default CI exclusion
  pattern (`Cones_*`, `ExampleParity`) for the shadow suites; hold
  the 120 s per-test TIMEOUT.
- Benches: scrub throughput (cold vs warm), edit→affected-frames
  recompute cost, UI-eval latency with warming on/off, memory under
  cap; extend the Chrome-trace profiler with cache/scheduler lanes
  (hit/miss, job queue depth, cancel counts).
- Benches also carry the non-deterministic checks kept out of the
  gate: UI-eval p95 with warming on vs off, and an N-second
  UI-vs-warming stress under TSAN on Linux.
- Docs: update `docs/specs/baked-step-graph.md` (cache interaction
  appendix), keep this plan current with the Stream 0 numbers, and
  add a TD-facing "what warming does" note; record final
  defaults (cap, workers, neighbor radius) with the numbers that
  chose them. Include the brief↔plan mapping paragraph: "Graph
  System manages LibEE through interfaces and caches" → FrameCache
  + BackgroundScheduler + frozen contexts; the "subscription"
  metaphor → Stream E's publish fence; D5's frame-order policy
  labeled as original (brief Gaps #4 confirms no public policy).
- Done: full `ctest -R '^testRigExec' -E 'Cones_|ExampleParity'`
  green locally and on CI; benches show warmed scrub ≥10× cold
  scrub on the biped (target; report actuals).

## Validation Plan

- Build (from `D:/work/usdRig-framecache`, helpers mirror the
  repo's `bin/` `.sh`/`.bat` pairs; CI runs configure → Release +
  Ninja → `rigExecPose` smoke):
  - Configure + Release build green; `rigExecPose` smoke passes.
- Unit/layout (Streams A–D; each new `testRigExec*` binary):
  - `ctest -R '^testRigExec(FrameCache|FrozenContext|BackgroundScheduler|FrameCacheSparsity|ImagingFrameCache)$'`
    green; each binary also passes with
    `RIGEXEC_ENABLE_PARALLEL_EVAL=0` (reads served, fill off) and
    with `RIGEXEC_FRAME_CACHE=off` (pure live fallback).
- Parity (highest-risk gate):
  - `RIGEXEC_FRAME_CACHE_VERIFY=1` shadow suite
    (`ctest -R 'FrameCache.*Cones'`) clean on biped + 9-mesh +
    one bake-refusal rig: every hit bit-equals live eval.
  - Existing `BakedWithParityCheck` suites unchanged and green.
- Concurrency:
  - Gate tests are deterministic (fixed job sequences, injected
    generation bumps, asserted counts) and fit the 120 s / 4 vCPU CI
    budget. The N-second UI-vs-warming stress (TSAN on Linux; MSVC
    manual) and the p95 latency comparison run as Stream F benches,
    reported in `reports/frame-cache-measurements.md`, not as gate
    tests, because wall-clock assertions flake on shared CI runners.
- End-to-end (manual, workstation with usdview):
  - Open biped, scrub a cold range (fills cache), scrub back
    (hits; observe zero evaluator runs via `solver_evaluations` /
    executed-cluster counters), drag a control (playhead evaluates
    live as today; affected neighbors re-warm after release), confirm
    the drag feels identical with `RIGEXEC_FRAME_CACHE=on` and `off`.
- Doc check: defaults in code match the numbers in
  `reports/frame-cache-measurements.md`.

## Risks / Rollback

- Background evaluation racing live state → mitigate with frozen
  contexts + generation fences + fail-closed drop; the THREAD-rule
  tests and the shadow-verify suite exist specifically for this.
- USD stage concurrent read from workers → the UI thread samples
  per-time input vectors at enqueue; workers hold no `Usd*` handle at
  all; the Stream B audit must prove it, or Stream B redesigns before
  C/E proceed.
- Playhead latency regression → structurally excluded by D5: the
  playhead path is the existing synchronous path plus one cache
  publish; Stream E's done criterion measures it against cache-off.
- Arena retention blowing the byte cap → D2's go/no-go on Stream 0's
  separated numbers; whole-pose memo is the fallback deliverable.
- Hidden cross-evaluation state in a kernel or solver → Stream B
  purity audit; the shadow-verify suite is the runtime backstop.
- Memory growth on long sessions → byte cap + LRU from day one;
  stats counters make over-retention visible in tests.
- Priority APIs differ per OS and CI covers Linux only → keep the
  priority call behind one function with a logged no-op fallback;
  MSVC/macOS verified manually per repo convention.
- Waking-cost and priority leak (the baked serial-default lesson) →
  background workers run the serial executor and never call `Work*`;
  any parallel re-enable is gated on per-rig measurement as today.
- Rollback: `RIGEXEC_FRAME_CACHE=off` restores exact current
  behavior at runtime; the branch reverts cleanly since all
  behavior changes sit behind the new switches and no authoring
  contracts change.

## Final defaults (Stream 0, measured 2026-09-18)

| default | value | in code | derived as |
|---------|-------|---------|------------|
| per-rig byte cap | 256 MiB | `kRigExecFrameCacheDefaultByteCap` (`libs/rigExec/frameCache.h`) | ~80 full biped frames at 3.05 MiB (430,258 B pose + 2,767,505 B arena) -- the +-8 neighborhood plus a +-32 sweep -- or ~119 full 9mesh frames (2.14 MiB), or 600+ pose-only frames (420 KiB) |
| background workers | 2 | `kRigExecBackgroundSchedulerDefaultWorkers` (`libs/rigExec/backgroundScheduler.h`) | the biped +-8 neighborhood (16 x 3.1 ms / 2) drains in ~25 ms; the 9mesh neighborhood in ~1.4 ms |
| neighbor radius | +-8 (16 frames) | `kRigExecFrameCacheDefaultNeighborRadius` (`libs/rigExec/backgroundScheduler.h`) | one 1.2 ms UI sampling burst at enqueue on the biped (74.4 us/frame), ~18 us on 9mesh |
| D2 go/no-go | GO | -- | arena retention fits the cap at a useful frame count on both rigs, so Stream D ships cross-frame cone reuse, not whole-pose memo only |

Full numbers, counting rules, and repro steps are in
`reports/frame-cache-measurements.md`; the switch semantics
these defaults sit under are D6 (`RIGEXEC_FRAME_CACHE`,
`RIGEXEC_FRAME_CACHE_VERIFY`).

## Stream F bench results (measured 2026-09-19)

`benchFrameCacheWarm` (`tests/benchFrameCacheWarm.cpp`, built but not a
ctest gate) on the section-2 workstation. Full tables, method, and repro
in `reports/frame-cache-measurements.md` §6; the verdicts:

- Scrub (cold eval+publish vs warm digest+lookup, medians): tiny 10.0 vs
  3.0 us (3.3x); biped parallel 2130 vs 583 us (**3.7x**); biped serial
  4090 vs 560 us (**7.3x**). Zero misses, zero invalid serves.
- **The Stream F done criterion (>=10x warmed scrub on the biped) is
  missed.** The warm anatomy (biped parallel: 159 us sample + 410 us
  digest+lookup) shows the miss is structural, not noise: D1 recomputes
  the digest from the stage at every lookup, and re-hashing 11,500 inputs
  costs ~70% of the warm frame. The follow-up is a digest that memoizes
  per-input hashes and re-folds only what the notice touched; it is
  recorded in the report and is not in this branch.
- Edit: one control-sample edit across 16 tiny frames affects 1 frame
  (60 us scan); sparse plan 0.8 us (Partial, memo used); repeat edit
  +0 walks; override drag affects 16/16 with the conservative
  whole-graph plan, by contract.
- Latency (200 live evals/series, A/B/A): off 6.0 us (p95 9.0),
  on-plumbing 6.0 (p95 7.0), on-loaded 5.0 (p95 6.0, 3.1 ms synthetic
  jobs draining throughout), off-repeat 5.0 (p95 6.0) -- no warming
  effect on the tiny rig; maxima in all series incl. off are machine
  noise. The biped-drag comparison stays the manual validation item.
- Memory: 64 tiny frames under the default cap (88 KiB, 0 evictions);
  a 64 KiB squeeze holds 47 entries at 64,766 bytes with 81 evictions
  counted -- bytes stay under the cap.
- Stress: `stress <seconds>` churns UI eval/publish/commit/cancel
  against a fence-hammering thread for TSAN runs (2 s: 268k evals,
  1.07M churn enqueues, scheduler drained to idle, 0 invalid). It is a
  bench on every platform, never a gate. The Linux TSAN verdict is
  recorded in the report when a TSAN build runs it.
- Profiler: `trace` mode writes `frame-cache-warm.trace` with the new
  lanes -- `cacheHit`/`cacheMiss` instants on `frameCache`,
  `warmQueue` counters and `warmCancel` instants on `scheduler`
  (`libs/rigExec/profiler.h`, covered by `testRigExecProfiler`).
- Doc check: the §6.7 table re-reads every default from its header on
  the measurement date -- 256 MiB / 2 workers / +-8, all confirmed,
  holding 83 full biped frames at the cap.

## Open Questions

- Closed. The three Stream 0 items resolved to the Final
  defaults above: 256 MiB per-rig cap, 2 background workers,
  +-8 neighbor radius, with D2 going GO on arena retention.
  The brief-access item is closed: the Premo brief was read
  in full and its deltas are folded into D2, D4, D5, and
  Streams C-F above.

## Completion record (2026-09-18, branch `feature/per-frame-cache`)

- Frozen runner landed in three increments: A (chain-free
  rigs evaluate on the worker via constant-patching +
  nulled handles), B (UI-thread chain-sampling hook --
  the biped's 12 float chains bind and warm), C (session
  snapshot cache with COW avar patching, eager refresh in
  `_EvaluateSessions`, per-job `context.frozen` binding in
  `BuildWarmWork`), plus the D4 per-context serial hook at
  the 4 `WorkParallelForN` sites.
- Bit-identity (`RigExecComparePoses`, 0 mismatches each):
  tiny rig, chained fixture (frames + drag-on-target +
  drag-on-factor arms), biped frames 3-4, 9mesh frames 3-4
  + drag arm, COW-patched snapshots, pinned-vs-fresh
  bindings; fail-closed declines proven at every layer.
- D7 gap closed: refusal-rig UI-memoization was documented
  but unimplemented; program-less rigs now key on time +
  stage-edit serial + overrides (serial advances on every
  stage notice, so edits move the digest by construction).
  `TestRefusalRigMemoizesUiThreadResults` proves hits with
  zero pulls and zero background jobs.
- Final gate `ctest -R '^testRigExec' -E 'Cones_|ExampleParity'`:
  75/81. The 6 failures are pre-existing Python-evaluator
  segfaults (`FkStartFramePython`, `PoseCyclePython`,
  `SkinLayoutOverrides`, `BakedCone`, `BakedPsd`,
  `BakedAttributePython`): they crash identically with
  `RIGEXEC_FRAME_CACHE=off`, in code this branch never
  touches. All 6 new cache binaries pass, including under
  `RIGEXEC_ENABLE_PARALLEL_EVAL=0` and
  `RIGEXEC_FRAME_CACHE=off`; `rigExecPose` biped smoke
  passes.
- Still manual / platform-gated, as the plan always had
  them: the usdview scrub/drag validation (hook:
  `plugin/rigExecUsdview/rigExecUsdview.py`) and the Linux
  TSAN stress verdict (`benchFrameCacheWarm stress`).

## Review record (2026-09-19, plan-coverage audit of the branch)

Every stream was audited against the code, and the CI gate was re-run
(`ctest -R '^testRigExec' -E 'Cones_|ExampleParity'`: 80/81, the one
failure `PoseCyclePython` reproduces with `RIGEXEC_FRAME_CACHE=off` in
code the branch never touches; the six cache binaries pass under
`RIGEXEC_ENABLE_PARALLEL_EVAL=0`, `RIGEXEC_FRAME_CACHE=off`, and
`RIGEXEC_FRAME_CACHE_VERIFY=1`). Streams 0, A, B, C, E, F are covered
as written. Fixed on the branch during the review:

- **Stale pose after a static control edit (D1 violation).** Epoch
  constants are not per-frame inputs, so the sampled control digest
  never saw them, and the binding epoch digest is blind to values: an
  avar edited as a default value (patched in place) or a captured
  value (rebuild, same epoch) left every cached frame reachable under
  the old key with a valid proof. `RigExecFrameCacheEpochDigest` now
  folds the program build count and the avar-region digest into the
  epoch half at both key sites and the proof scope.
  `TestStaticControlEditInvalidatesCachedFrames` reproduced it and
  now holds it.
- **`FrameCache::Clear` raced background publishes.** It zeroed the
  byte and entry counters unlocked after per-shard clears, so a
  publish landing in between orphaned its bytes and a later eviction
  wrapped the counter (every later publish would drain the store).
  Accounting is now per shard under the lock;
  `TestClearDuringConcurrentPublishKeepsAccountingHonest` covers it.
- **Payload teardown under the shard lock** at all five drop sites
  violated the Stream A lock rule; entries are now retired outside the
  lock.
- **Weight-overlay change did not fence in-flight jobs**: the flag
  rides to workers in `context.flags` and changes pose content without
  moving the key. `SetWeightOverlay` now cancels the rig's generation.
- **Playback shedding ran on every `SetTime`, in both directions.** A
  backward scrub purged exactly the sweep frames it was heading into.
  The registry now sheds only on a forward step of the playhead.
- **Override ticks did not bump the generation.** D5 says any new edit
  cancels in-flight jobs; a drag tick left pre-drag jobs running to a
  key no lookup can reach. The preview setter now cancels per rig.
- **Every cold frame sampled and digested twice** (once before the
  proof check, once for memoization after the live run), about 1.1 ms
  on the biped, against the Stream E criterion that a cold miss costs
  what cache-off costs. The lookup now consults the proof by time
  first and samples nothing when no proof exists.
- **`CancelGeneration` freed up to a rig's in-flight cap of sampled
  vectors under the scheduler lock**; purged closures now die after the
  lock closes. **`WaitUntilIdle` ignored `_stopping`** and could hang
  against a shutdown or a zero-worker pool; it now returns on stop.
- Spec appendix corrected (see next item) and the brief-to-plan mapping
  paragraph added to it; generated site rebuilt with the concept page.

Open against the plan, not closed by this review:

- **Stream D is a library, not wired.** `RigExecPlanSparseReuse`,
  `RigExecRunSparsePlan`, `RigExecOutputAffectedIndex`,
  `RigExecTaskListCache`, and `RigExecNoteCaptureIndex` have no caller
  outside tests and benches. The bridge serves whole-pose memo (hit =
  stored pose, miss = full live run), the commit trigger re-warms a
  fixed +-8 band plus +-32 sweep rather than the index's affected
  frames, and no running code drops an epoch's frames on a capture-index
  hit. The D2 "GO" delivered the machinery and its tests; the
  success criterion "within an affected frame only the cone downstream
  of what changed" is met only in `testRigExecFrameCacheSparsity`.
- **Shadow suite runs on synthetic poses.** `Cones_Verify` fabricates
  three-joint poses; it opens no stage and never loads the biped,
  9-mesh, or a refusal rig the Validation Plan names.
- **usdview trigger fires on the next frame change, not on release.**
  The plugin's only hook is `currentFrameChanged`, so an edit at a held
  playhead does not warm until the user scrubs; there is no periodic
  tick in usdview to carry the idle signal.
- **Profiler lanes have no production caller.** `cacheHit`/`cacheMiss`,
  `warmQueue`, `warmCancel` are emitted only by the bench and the
  profiler test.
- **No priority smoke under load** in the scheduler test (the backend
  is named and set, but progress under load is not observed); the
  bench carries it, which the plan permits.
- **The queue is one global priority map, not per rig**
  (`backgroundScheduler.h`): generations and in-flight caps are per
  rig, but one rig's neighbors preempt another rig's sweep. Harmless
  at one session; the header's "per rig" wording overstates it.
- **A hit still samples and digests the full input vector on the UI
  thread** (about 70% of a warm biped frame, per the report); the
  memoized per-input digest follow-up is the fix and is not on the
  branch.
- **`RigExecProgramAvarPatch`'s two parallel sites in `bakedProgram.cpp`
  gate on `RigExecParallelEvaluationEnabled()` alone**, without the
  frozen-serial term; UI-thread-only today, so no leak, but unguarded.
- **Drag ticks memoize under override digests no later lookup can
  reach**; a long biped drag can churn the warmed neighborhood out of
  the 256 MiB cap. D5 says an edit at the playhead publishes into the
  cache, so this is the plan's own call; skipping memoization while the
  override list is non-empty is the one-line alternative.
- **Bit-identity is proven one frame past the freeze** in every
  frozen-context test; production sweeps to +-40, where a frozen run
  branches its cone-skip decisions from freeze-time history. The
  shadow switch is the runtime backstop; a test at sweep distance is
  the missing gate.
- **`epochDigest`/`programDigest` on the frozen context are documented
  as pre-run drop checks and read nowhere**; the generation fence and
  the epoch-keyed cache cover it, so the header overstates.
- **`TfGetenv` on the per-frame path**: the cache mode is re-read on
  every enqueue and publish (about 80 per commit burst) and the verify
  flag on every hit; tests toggle them mid-run, so memoizing needs a
  reset hook.
- **Latent, in the unwired path:** `RigExecBakedClusterSet::Union`
  reads `other.words[w]` unguarded, reachable from
  `frameCacheSparsity.cpp` when a memoized set is narrower than the
  dirty set; `NotePlaybackAdvanced` rejects a default playhead but not
  a NaN one, which would shed the whole sweep; `programDigest` is set
  on the context but checked nowhere.

## Follow-up record (2026-09-19, gap-closure pass)

Closed every open item above except the three recorded as remaining at
the end. Nothing is committed.

- **Stream D, capture-index half: wired.** `RigExecImagingBridge::NoteCaptureIndex`
  routes the registry's notice path through the baked capture index: a hit
  drops the epoch eagerly while its half still names it (the evaluator
  rebuilds lazily), a miss drops nothing; epoch drift since the last
  memoization (in-place avar patch, rebuild, structural edit) evicts the
  old half, whose frames are unreachable by D1, instead of waiting for LRU;
  proofs naming evicted entries are cleared so the next visit misses
  without paying a sample. The bridge owns the `RigExecTaskListCache` the
  call needs (its first production owner) and a last-published-epoch stash
  (atomic for bare bridges). Refusal rigs are excluded by construction:
  their entries key on the stage-edit serial, which already retires them.
  Proven by `TestCaptureIndexHitDropsEpochEagerly` (hit evicts, miss
  stands and still hits).
- **Stream D, planning half: still future work, by decision.** `PlanSparseReuse` /
  `RunSparsePlan` stay unwired: partial cluster re-runs need retained slot
  arenas plus a cluster runner, a multi-day project with real bit-identity
  risk. The commit trigger keeps the fixed band deliberately: D5 specifies
  the neighbors-plus-sweep order, and the affected-frame selection D2 names
  rides the retained-slot path, not job-skipping. Digest-gated skipping was
  evaluated and rejected: it would rewrite three pinned trigger-count
  contracts (`TestTriggersEnqueueNeighborsAndSweep`,
  `TestProductionTriggerPathEnqueuesAndFences`, the injected-kernel bursts)
  to save worker CPU only -- `BuildWarmWork` samples before it could gate,
  so the UI burst cost would not move. (The "~6 ms" estimate that stood
  here was wrong: `benchCommitLag` measures the real biped burst at
  ~136 ms, ~121 ms of it the per-frame sampling -- see the lag note
  below. The rejection stands; the number did not.)
- **Shadow suite: on real rigs.** `testRigExecFrameCacheCones_Verify` now
  runs the judge on the biped (new `examples` argument, same convention as
  the frozen suite), the animated 9-mesh at 3, 4, 39 and 40, and a
  bake-refusal rig: settle live, publish a settled re-run under a faithful
  key, shadow-verify the served hit against a third live run. Every probe
  matches bit-exactly under the full judge (counters and diagnostics
  included); a one-float mutation control on each rig proves the judge is
  live on these poses. The suite takes the schema plugin path alongside the
  frozen and imaging suites.
- **Bit-identity at sweep distance: proven.** `Test9MeshWarmsBitIdenticalAtSweepDistance`
  (frames 39, 40) and `TestBipedWarmsBitIdenticalAtSweepDistance` (frame 40,
  past the authored 1..8, as a production sweep from a late playhead)
  freeze after frame 2 and diff warmed-vs-live under the full judge, in the
  existing tests' same-history arrangement. Both pass.
- **usdview release trigger: wired, then deferred off the gesture.**
  Notices schedule a coalesced flush through `pxr.Usdviewq.qt` (PySide6 in
  this environment), so an edit at a held playhead commits without waiting
  for a scrub; the frame-change path stays as the no-Qt fallback, and
  scheduling degrades to the pending flag when Qt is absent. The first
  wiring fired on the next event-loop turn (`singleShot(0)`), which put
  the synchronous 80-frame sampling burst inside every gizmo release and
  every undo/redo -- `benchCommitLag` (new, built-not-registered like the
  other benches) measures the commit half at ~136 ms on the biped against
  ~10 ms for the edit half, with ~121 ms of the burst in per-frame
  sampling, ~11 ms in control digests, and ~4 ms in per-frame overhead
  (snapshot refresh, chain-currency checks, epoch digests). The flush now
  runs from a restarting 120 ms idle timer: scheduling arms (or restarts)
  and commits nothing, so the gesture's own repaint lands first and a
  pause in editing still warms the held playhead with no scrub; a scrub
  beforehand consumes the pending flag on its tick instead. The flush
  logic (consume-once, idle no-op, inactive keeps pending, armed
  early-out) is asserted by `_AssertWarmingFlushOnRelease`, and the
  deferral (positive delay, timer shape, restart-not-recreate, zero
  commits before the flush) by `_AssertWarmingCommitDeferred` -- both
  verified headless, since this machine runs testusdview without GL.
  No repeating idle timer: the commit covers the gap, and idle remains
  scrub-driven per D5. Deliberately not hoisted at the time: the ~4 ms
  per-burst overhead was 3% of the burst, not worth the session-state
  risk; the sampling anomaly looked like the entire bill. That reading
  was wrong (see the burst-prep note below): two thirds of the burst was
  re-verification and table walks, and the follow-up pass prepares them
  once per burst.
- **Profiler lanes: production callers.** The bridge records `cacheHit` /
  `cacheMiss` on actual store consultations (proof-less lookups record
  nothing); the scheduler samples `warmQueue` on every queue mutation and
  `warmCancel` with `edit` / `playback` / `shutdown` causes on purges (the
  profiler lock is a documented leaf under the scheduler lock). Proven by
  `TestProfilerFrameCacheLane` and `TestProfilerLaneRecordsQueueAndCancels`,
  including off-by-default.
- **Warm-path digest cost: halved, not closed.** The byte FNV fold is a
  word-at-a-time splitmix64 finalizer with FNV-style chaining in both
  digest TUs; biped warm digest+lookup measures ~243 us, down from ~475 us
  (report section 8). The first attempt (xor-only chaining) served stale
  poses by cancelling duplicate words -- caught by
  `TestPatchFrozenAvarConstants`, whose constant lives in two digested
  locations -- and is now pinned by `TestDigestSeesDuplicateValues`
  (duplicates move, permutations collide never). No test hardcodes a digest
  value. The 10x target stays missed: the warm frame is now
  sample-dominated, and sampling itself measures ~10x its section 7 value
  on this workstation with untouched binaries -- recorded in section 8 as
  an unexplained pre-existing gap, so the memoized per-input digest remains
  future work, now joined by a sampling investigation.
- **`TfGetenv` on the trigger path: hoisted, not memoized.** The fill gate
  is read once per burst at trigger entry instead of once per frame; the
  per-frame path keeps its staleness checks (stopping, generation, queued
  set, cap). Mid-burst flips land on the next trigger (no test flips
  mid-trigger; all toggles are function-scope RAII). Memoizing would need
  the reset hook for zero measurable gain -- the remaining reads are one
  per UI event against millisecond-scale sampling -- so the live-read
  contract stands, documented at the call sites.
- **Drag-tick memoization: already closed before this pass.** The bridge
  bypasses lookup and memoization while interactive overrides stand
  (`TestInteractiveDragBypassesCache`); the review's open item predates it.
- **Small items: all closed.** `Union` tolerates mismatched widths
  (`TestUnionToleratesMismatchedWidths`); `NotePlaybackAdvanced` rejects
  NaN (`TestPlaybackAdvanceWithNaNShedsNothing`); the scheduler header no
  longer claims per-rig queues; the frozen-context digest members document
  fence coverage instead of worker checks; both avar parallel sites carry
  the frozen-serial term (no observable behavior change -- UI-thread-only
  today, covered by the existing parallel/serial suites).
- **Python pose-cycle failure: verified pre-existing.** Fails identically
  with the cache on and off (same file, line, assertion, `None` message);
  the traceback never enters frame-cache code.

- **Burst preparation: the post-edit pause is deleted, not moved.**
  Moving the burst off the main thread was asked and refused on
  architectural grounds: sampling reads the live USD stage, the UI thread
  can author at any moment, and USD forbids concurrent read-during-write
  -- the codebase's worker/values split exists for exactly this reason,
  and no cheap stage snapshot or safe lock placement exists. Instead the
  bench's anatomy (benchCommitLag, extended with a pinned-route breakdown
  and a static/varying composition probe) showed the ~141 ms biped burst
  was ~55% chain-currency re-verification (312 us x3 per frame), ~6%
  binding-table walks over epoch constants, ~8% static array re-reads
  (843 KB per frame, 99.99% time-invariant), ~8% control digest, ~4%
  chain hook, and the rest enqueue/proof/misc. A burst is a synchronous
  UI-thread span, so all of it except the hook, the varying reads, and
  the per-frame enqueue is burst-fixed and prepared once:
  RigExecBurstSampleCache (verified bindings copy, epoch digest,
  per-table varying/overridden site lists, route-partitioned static
  sample maps, pre-sorted digest order), built by RigExecBuildBurstSampleCache,
  consumed by RigExecSampleFrameInputsWithBurstCache and
  RigExecControlStateDigestWithBurstCache. The control digest is two-level
  now (one level-1 per first-win path, chained; memoized for statics),
  which keeps every sensitivity relation and the enqueue-order pin while
  letting the burst fold 843 KB once instead of per frame. Bit-identity is
  by construction (same emission order, same routes, digest folds the
  served vector) and by test: six new frozen tests (elementwise sampler
  agreement over two rigs x two override arms x four frames with animated
  array reads, warmed-pose identity, foreign-program/changed-override
  rejection, digest fallback, unplaceable decline), a mutation check that
  flips the static classifier and fails the agreement test, and the
  unchanged imaging suite exercising the cached trigger path end to end
  (same 80/64 counts, same fences, same serves). The registry keeps the
  plain per-frame route as the fallback (null cache, unusable build,
  moved program) and declines unusable bursts up front instead of per
  frame. Commit on the biped: ~141 ms to ~20 ms (7.1x), same 80 jobs;
  no-edit re-commit ~55 ms to ~2 ms. Residual is fundamental per-frame
  work (chain hook, digest combine, packet assembly, enqueue). The
  memoized-digest future work is now half done (burst-scoped); the
  lookup-path (epoch-keyed, notice-invalidated) half remains, and would
  speed warm scrub the same way. The sampling "anomaly" is retired as a
  mystery: per-frame reads were never the whole story.

Remaining, in addition to the partial-execution project above: the
priority smoke stays bench-carried (the plan permits it); the lookup-path
half of the memoized digest stays future work. Further pause reduction
needs fewer frames per burst (the affected-frames index) or a burst-shape
change (neighbors-on-commit plus sweep-on-tick), both contract changes
for a plan review, not this pass.

Test results:

| run | result |
|-----|--------|
| CI gate `ctest -R '^testRigExec' -E 'Cones_\|ExampleParity'` | 80 / 81 (only the pre-existing Python pose-cycle failure) |
| Six cache binaries under `PARALLEL_EVAL=0`, `FRAME_CACHE=off`, `VERIFY=1` | all pass |
| Shadow suite (`Cones_Verify`, now with stage shadows) | pass |
| Flush-logic python assert, headless | pass |
| Commit-deferral python assert (lag gate), headless | pass |
| `benchCommitLag examples` on the biped | edit ~10 ms, commit ~136 ms (sample 1.52 ms/frame, control digest 134 us, epoch digest 33 us) |
| Same bench after burst preparation | commit ~141 ms to ~20 ms (7.1x), same 80 jobs; currency 312 us x3, hook 68 us, 253 samples/843 KB (99.99% static) |
| Shadow suite (`Cones_*`: 23 tests) after burst preparation | all pass |

