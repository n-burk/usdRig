# Frame-cache full vision — remaining work

Branch `feature/per-frame-cache`. Parent plan:
[frame-cache-full-vision.md](/D:/work/usdRig-framecache/docs/plans/frame-cache-full-vision.md).
Stream 0, Stream 1 sections 1.1/1.2, and 1.3–1.4 slices 1–3 are done and
green. This doc starts at the live blocker and runs to the E2E gate.

## 0. Blocker: the stack never warms through the registry production path

`TestStackFullRangeCursorWarmsEveryFrame`
([test](/D:/work/usdRig-framecache/tests/testRigExecImagingFrameCache.cpp))
fails: 208 driver ticks, every `OnIdle()` returns 0,
`factoryInvocations` delta 0, `lastSkip == None`, `published` delta 0;
frames 2–200 stay `Uncached` (199 state failures plus knock-on scrub /
eval-count failures).

Established mechanism (not yet the root cause):

- `_PrepareWarmBurst` returns an unusable burst with `burstOverrun == false`,
  so `OnIdle` leaves the factory null — "no factory on a shape decline"
  ([registry.cpp](/D:/work/usdRig-framecache/libs/rigExecImaging/registry.cpp)).
- `RigExecBuildBurstSampleCache` itself only fails on unplaceable overrides
  ([frozenContext.cpp](/D:/work/usdRig-framecache/libs/rigExec/frozenContext.cpp)),
  so with the test's empty overrides the build should succeed. The early
  return must be one of:
  - (a) `!program` — the bridge evaluator has no baked program at registry
    level;
  - (b) `!chainBindingsValid` — `RigExecBindChainSampleInputs` fails or
    refuses on the stack.

Next steps, in order:

1. Distinguishing probe: call `registry.BuildWarmWork(rig, t, gen,
   productionRunner)` directly on the opened stack. The skip reason names
   the cause: `D7Exempt` ⇒ (a), `Unsampleable`/`chain-bind` ⇒ (b),
   `FreezeRefused` ⇒ the freeze itself fails deeper.
2. Fix the root cause; get the stack test green (all 200 frames cached,
   `published` == warmed count, per-trigger invocations ≤ 16, scrub serves
   without evaluating, warmed-vs-live bit-identity on the spot frames).
3. Remove the `TEMP` diagnostic in the stack test.
4. Fix the silent-null-factory gap this exposed: a null-factory trigger
   records no skip and no streaks, so a recurring driver would spin forever
   on such a rig — against the plan's "never silently pending-forever".
   Decide and implement: session-level skip record, un-warmable advance,
   or a driver no-progress escape.
5. Verify and correct the `TestRefusalRigNeverCountsVisited` comment: it
   claims persistent D7 skips advance the streak, but if the refusal rig
   also takes the null-factory path no skips are ever recorded. The
   assertions hold either way; the comment must describe the real
   mechanism.

## 1. Stream 1 tail: plugin recurring driver + headless model

The C++ side already exports everything; the plugin binds none of it
(`rigExecUsdview.py` binds only Activate/SetTime/OnEditCommitted/OnIdle/
Deactivate/preview/overlay — no WarmRange, GetFrameStates, or
ClearFrameCache anywhere under `plugin/`).

1. Add ctypes bindings in
   [rigExecUsdview.py](/D:/work/usdRig-framecache/plugin/rigExecUsdview/rigExecUsdview.py):
   `RigExecImaging_WarmRange`, `RigExecImaging_GetFrameStates`
   (`RigExecImaging_ClearFrameCache` rides along for §3).
2. New headless cache-strip model module (e.g.
   `plugin/rigExecUsdview/cacheStripModel.py`): batched state fetch over a
   frame list, any-unwarm check for driver sleep/wake, and the
   state→display-role mapping the Stream 3 view will reuse. Maya-style
   coloring is a standing user requirement — define the palette here, once.
3. Headless model tests mirroring the `profilerModel` pattern (pure model,
   no Qt).
4. Recurring `QTimer` driver in `rigExecUsdview.py`: tick `OnIdle` while
   unwarm frames remain in range, sleep otherwise; wake on `SetTime`, edit
   commit, and range change; `SetWarmRange` from the stage range on
   activation. Keep the 120 ms commit-burst discipline; without Qt the
   degradation is frame-change-only warming (today's behavior).
5. Validate manually: `bin\launch_usdview.bat
   examples\biped\Biped_stack_anim.usda`, hold the playhead, watch the full
   range warm; scrub re-warms.

## 2. Stream 2: partial invalidation + recache (2.0–2.4, nothing landed)

Verified absent: `CancelGenerationTimes`, per-time fence tokens, any
evaluator per-notice disposition query, retained-state publish;
`MapControl` still has no callers. Plan sections:

- **2.0 Entry provenance.** Per-entry cluster-dependency sets, proof aliases,
  weight-object/packet read sets (`WeightPacket` steps resolve through
  `step.object`, outside the avar-only index), and constant-region read
  sets. Retained-state publish in the production bridge paths: the frozen
  arena carries doubles while wider state (points, matrices, packets)
  rides with the runner — capture both halves against the retained handle;
  re-run jobs rebind into a per-job cloned program and run clusters
  through a production `RigExecClusterRunner` (needs the rebind context it
  lacks today); partials merge and publish through the normal completion
  path.
- **2.1 Path-scoped retirement.** D3 epoch-digest rescope first (else every
  value patch evicts the world). Notice adapter: USD paths →
  `RigExecControlId`s; index coverage for every cone-source family (chain,
  solver, revision, native-source, delta, constraint-array,
  varying/override), giving `MapControl` production callers; control
  universe with known-empty distinct from unknown (update the sparsity
  tests that assert today's collapse); resolved-asset resync paths;
  planner time rule matching the executor (new varying/override closure
  accessors beside `Always()`). New evaluator outcome query per notice —
  patched (+paths) / stamp-bumped / stale — driving exact-cluster retire,
  retire + re-resolve, or rebuild + epoch move. Scoped cancellation:
  `CancelGenerationTimes` for affected times only (no generation bump)
  with fresh per-time fence tokens carried on the warm request and
  required at the publish fence; global cancel stays for epoch moves.
  Replaces the blanket per-notice cancel.
- **2.2 Re-resolve lanes.** Per frame: (a) same namespace, digest equal —
  stands, zero work; (b) namespace moved, provenance clean — carry-over
  re-publish under the new key (failure degrades to (c)); (c) dirty —
  cone re-run against retained state, merged pose under the new key.
  Affected-time computation (1.4 index ∩ provenance) replaces the fixed
  sweep; declined partials requeue at prior priority; declined-generation
  requeues under the live generation only.
- **2.3 Proof scoping.** Capture each proof's sampled-path dependency set
  (existing level-1 per-input digests) at record time; retire intersecting
  proofs instead of wholesale clearing; re-point survivors on namespace
  moves. Unproven frames keep evaluating live (bit-identity).
- **2.4 Drag interplay.** Keep the interactive bypass; on release the commit
  burst re-warms affected frames via cluster-scoped enqueue.
- Validate per plan §2.x: one case per 2.1 evaluator branch, carry-over
  with zero evaluator pulls on untouched entries, scoped-cancel (old gated
  job drops on fence-token mismatch), edit-one-control retires only
  affected frames with bit-identical re-warm, `Cones_*` and
  `testRigExecBakedMode` edit suites green.

## 3. Stream 3: timeline cache strip (3.1–3.3)

Do the 3.3 fencing core before the view — Stream 2's per-time tokens share
the fence protocol, and the strip would otherwise display repopulated-stale
states. Verified: `ClearFrameCache` is still clear-only (no cancel, no
fence, no generation bump); the C bindings exist but route through it.

- **3.3 fencing.** `ClearFrameCache` becomes cancel-THEN-clear plus 1.4
  index reset, in that order. Worker publish and cancel/clear share a
  fence mutex: fence-check (generation + per-time token) and cache insert
  atomically under it; the fenced clear holds it across bump + purge +
  clear + reset. Lock order fence → scheduler → cache shards, never
  inverted; cancel in its own scope before locking for the clear (the
  registry mutex is non-recursive). Route the C binding, the overlay path
  (fix its clear-before-cancel order — a worker can publish a stale-overlay
  pose between the two), and all existing callers through the fenced entry.
- **3.1 strip dialog.** Modeless dialog on the profiler-panel pattern,
  rendering per-frame cached / warming / dirty / uncached over the stage
  range via the 1.4 API and the §1 model/palette. Poll-based repaint on
  the driver tick and on `SetTime`; skip repaints while the completions
  counter is still.
- **3.2 tracking.** Follow `currentFrameChanged` and stage replacement;
  range from stage start/end timecodes.
- **3.3 panel actions.** Clear-cache and warm-range actions driving the C
  bindings, so artists can force the states the strip shows.
- Validate per plan §3.x: headless panel-model tests; clear-while-warming
  (cache stays empty, index resets); deterministic fence race test (paused
  insert vs concurrent clear); overlay-mid-warming staleness test; manual
  scrub recolor.

## 4. E2E gate

- `ctest -R '^testRigExec' -E 'Cones_|ExampleParity'` plus the full
  `example_parity_*` and `verify_binary_*` families. The flat rig and the
  0.7 stack anim entries are the named must-pass subset, not the whole run.
- Known pre-existing failures, out of scope unless touched: 6 Python
  SEGFAULTs fail the full `ctest` today.
