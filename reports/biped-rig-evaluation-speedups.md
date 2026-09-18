# Speeding up rig evaluation in the biped example

Date: 2026-09-13. Sources: the two shared research docs (Premo/LibEE and
Presto briefs, read via `export?format=txt`), the Watt TBB paper (downloaded
from multithreadingandvfx.org, extracted via `pdftotext`, 892 lines), the
local evaluator (`libs/rigExec/rigEvaluator.h` full,
`rigEvaluator.cpp` `Evaluate` l.7188 + `_poseSteps` loop l.7878,
`moverGraph.cpp` `Evaluate` ll.1826–1862, `tapSet.cpp`), `docs/biped-rig.md`,
and — new in this revision — a **measured profile** (`reports/biped_eval.trace`,
Chrome-trace JSON, 375 events: one Compile + 3 Evaluations).

Prior revision of this file said "static analysis only". §4 is now measured.
The predicted ranking was wrong in one important way: the skin chain is *not*
the critical path — the serial pose-constraint walk is.

## 1. Sources

### 1.1 The three external sources

1. **DreamWorks Premo / LibEE brief** (Google Doc `17Gp4y…`): Premo =
   animation app + parallel rigs + **LibEE**, a character-animation-specialized
   dependency-graph engine. Key ideas: eval-only v1 (strip edit infrastructure,
   small nodes, fixed topology, cache locality); few traversed paths so **task
   lists are cached** per control edit; 3-pass eval (dirty → upstream
   task-list build → CnC/TBB parallel schedule); **chain fusion**
   (single-input node chains = one task on one core); switch-prune pass; TBB +
   Concurrent Collections with studio-wide TBB for composability; TBB allocator
   (~+20% perf / ~+20% memory); thread-safety categories + global kill switch;
   C++ only in operators (no Python/GIL); threading visualizer with
   critical-path highlight and average-concurrency metric; measured **node-only
   ~1.35×, +4.2× graph-level, ~5.7× total on 16 cores, later heroes ~7–8×,
   multi-graph ~12–15× throughput**; LibEE 2 multi-representation (authoring
   vs evaluation graph, ~100× authoring-ops claim); Hierarchy Models (joints
   as atomic bundle flowing through DG); Premonition (adjacent-frame
   precompute after edits).
2. **Pixar Presto brief** (Google Doc `1J2LX2z3…`): Presto Execution System =
   **compile → schedule → evaluate** split to amortize dependency analysis;
   Network / Schedulers / Data Managers / Executors; vectorized dataflow (many
   elements per connection); pull eval with cache truncation; invalidation
   push; strategies per-node / per-branch / per-model / **per-frame Background
   Execution** (most successful at Pixar) with **starter executor** computing
   time-invariant data once, read locklessly by frame executors; interruption
   checks per node, discard-no-join for non-topology edits; ~30k-node light
   character <42 ms toward 24 fps; jemalloc/NUMA; callbacks static, stateless,
   deps declared a priori; Python initially banned, later GIL caveats; open
   descendant OpenExec (vdf/ef/exec).
3. **Watt, Parallel evaluation of character rigs using TBB** (the
   primary-source depth behind (1), verified from the PDF this pass): 150k-node
   / 15–24 fps targets on **HP Z820 16-core Sandy Bridge 3.1 GHz** animator
   workstations; TBB + CnC with Intel collaboration; task-list caching to avoid
   dirty walks; static-dirty over-compute vs switch prune; Type/Group/Globally
   Unsafe locks (production: Threadsafe + Type/Globally Unsafe only);
   `DWA_PARALLEL_FOR` macros with timing variants; **parallel-region cost
   ≈10k cycles** rule-of-thumb and grain-size tuning; overthreading and
   threading-fatigue warnings; "⅓ threading, ⅔ cleanup" indirect-speedup
   anecdote; TBB allocator numbers; chain-on-one-core, hammocks rejected;
   performance P-states +20%, CPU affinity ~+10% plus steadier fps, 2-socket
   NUMA tolerable; API layer blocking unsafe calls; parallel unit-test wrapper;
   Intel `ww1711` static-store warning; kill switch; critical-path-first
   optimization; case studies (claw chain rewire, hair rewire, "free clothing"
   off critical path); TD rig-parallelism skill gap. Final numbers:
   **5.7× total (1.35× node + 4.2× graph) → 7–8× on later heroes; 12–15×
   multi-graph throughput** at the cost of multiple graph states in memory.

### 1.2 What was actually read locally

- `libs/rigExec/rigEvaluator.h` (full), `rigEvaluator.cpp` `Evaluate` l.7188,
  `_poseSteps` loop ll.7878–7958, geometry-chain evaluation (per-target
  `graph.Evaluate(head)`).
- `libs/rigExec/moverGraph.cpp` `Evaluate` ll.1826–1862 (`VdfScheduler::Schedule`
  once + cached, `executor.Run` per call), `libs/rigExec/tapSet.cpp`
  (`ComputeWithOverrides` l.245), `libs/rigExec/profiler.h`
  (`RIGEXEC_PROFILE_SCOPE_CAT`, Chrome-trace writer).
- `README.md` (interactive-updates / limitations), `docs/biped-rig.md`,
  `examples/biped/README.md`.
- Biped census by regex over `examples/biped/Biped.usda`: **252 `RigExecJoint`,
  74 `RigExecControl`, 44 `RigExecFloatMathMover`, 65 constraints
  (19 Parent / 17 Aim / 13 Scale / 12 Rotation / 4 Position),
  4 FK + 4 TwoBoneIK + 4 BlendPointFrames + 2 SplineIK solvers, 1 `RigExecSkinMover`**;
  body mesh **26,276 points** (`faceVertexCounts`); layered form 485 prims /
  110 movers per README.
- `grep` for threading in `libs/rigExec` and `libs/rigExecMath`: **no
  `parallel_for`/`tbb`/`omp`/`std::thread`/`std::async` in evaluator or math
  kernels** — only `std::mutex` in tap-context setup, curvenet bind cache, and
  the profiler. SIMD exists (`rigExecMath/simdKernels.cpp`) but that is
  vectorization, not multithreading.
- `reports/biped_eval.trace`: measured profile analyzed in §4.

## 2. How rigEvaluator executes operators today

One `Evaluate(time)` generation (`rigEvaluator.cpp` l.7188) is a **serial
ordered walk** through four domains (`README.md` "How it works" diagram;
`rigEvaluator.h` ll.1–10):

1. **Property chains** (`_propertyChains`, `_EvaluatePropertyChains`):
   float/vec3f/matrix math movers resolve **off the authored stage before exec
   runs**; results are delivered twice — as exec value overrides *and* through
   `_resolvedInputs` for packet assemblers and the CPU oracle so both read the
   same number.
2. **Pose domain** (`_poseSteps`, l.7878): an interleaved schedule of
   **solver batches** (one `RigExecTapSet::Evaluate(time, inputs)` per
   dependency level, sequential across levels, snapshot-cached with
   input-equality + `ConsumeDirty` checks, counted in
   `solverEvaluations`/`solverOverrideRounds`) and **frame constraints**
   (Aim/Position/Rotation/Scale/Parent/SingleChainIK, registry ll.196–229)
   solved by serial C++ kernels in `rigExecMath` with descendant propagation
   and zero-weight short-circuit. Constraint → IK-goal → later-constraint
   chaining works *because* this walk is one compiled dependency order.
3. **Geometry domain**: per exact-points-target revision chains in
   `_chainOrder`. Each target owns a persistent in-memory `VdfNetwork`
   (`RigExecMoverGraph`): `UpdatePointSource`/`UpdateRevision` splice retained
   nodes, then `graph.Evaluate(head)` runs the cached `VdfSchedule` +
   `executor.Run` (ll.1833–1847). **Targets evaluate one after another**;
   derived normals/extent run after their points chain.
4. **Parity oracle** (`cpuParityMode` only): independent scalar
   `_EvaluateChain` re-runs point chains for agreement counting. Off by
   default.

Caching already present (the good news — this repo already implements the
LibEE/Presto "amortize everything" half): persistent VDF schedules +
executors with dirty-suffix invalidation; solver snapshot reuse
(`solverEvaluations == 0` when cached); shared `ExecUsdSystem` per stage;
`RigExecResolvedInputs`; curvenet bind cache; falloff LUTs baked at Compile;
per-eval counters exposed to Python. And since the prior revision: a **scoped
phase profiler** (`RIGEXEC_PROFILE_SCOPE_CAT`, categories `compile` /
`evaluate` / `pose` / `exec` / `geometry` / `property`) with
`WriteProfileTrace` Chrome-trace output and `SetProfilingEnabled` — the
"counters only" limitation is fixed; what is still missing is the
*concurrency* half of the LibEE visualizer (timeline across threads,
critical-path highlight), because there is only one thread so far.

## 3. Comparison: external approaches vs rigEvaluator

| Concern | LibEE / Presto / Watt paper | rigEvaluator today |
|---|---|---|
| Eval phases | 3-pass (dirty → task list → parallel schedule); compile/schedule/evaluate split | Same split in spirit: Compile (bindings, batches, `_chainOrder`, `_poseSteps`) → persistent VDF schedules + tap requests → per-time Evaluate. No separate task-list structure; order is the compiled walk. |
| Graph-level parallelism | The main win (4.2× on top of 1.35× node-only). CnC tasks, chains fused on one core. Per-frame concurrency for throughput. | **Absent at the evaluator layer.** `_solverBatches` run level-by-level, constraints one-by-one, geometry targets one-by-one. Parallelism exists only *inside* each `ExecUsdRequest`/`VdfExecutor::Run` (OpenExec's own threading). No cross-batch, cross-constraint, or cross-target parallelism is expressed. |
| Node-level parallelism | Internally-threaded deformers/solvers composable with graph MT via studio-wide TBB. 10k-cycle region rule, grain-size tuning. | **Absent in `rigExecMath` kernels** (grep §1.2). Skin LBS over 26k points, vertex-normal recompute, extent, weight resolution, blend summation are serial loops. VDF executor threads *across* VDF nodes, not *within* these kernels. |
| Dirty/caching | Task-list cache; dirty ∩ affecting(requested); switch prune; pull truncation on cache hit. | Equivalent mechanisms, different shape: persistent schedules + dirty suffix, solver snapshot input-equality, phased-read snapshots only where named. Over-compute is handled by explicit `rigExecReadPhase` (base/preceding/final/AtPrim) with compile-time cycle rejection instead of a prune pass. |
| Scheduling overhead | Low-overhead scheduler for 150k nodes @ 15–24 fps; chain fusion cuts task count. | Per-target `VdfNetwork`s keep each schedule small (good), but pay one `Schedule`+`Run` per target chain per generation and one exec `Compute` per solver level. Fine at biped scale; overhead grows with target/level count. |
| Thread safety model | Declared categories + typed locks + global kill switch; API layer blocks unsafe calls; parallel stress-test wrapper. | No declared categories; safety inherited from OpenExec/VDF. No evaluator kill switch, no unsafe-node locks (nothing to lock — the walk is serial). Grows into a requirement the moment §5 items parallelize. |
| Profiling | Threading visualizer: concurrency timeline, component colors, critical-path highlight, average concurrency, before/after compare. Optimize critical path only. | Scoped phase profiler + Chrome-trace export now exist (§2); `exec_stack.py` shows *order*. Still missing: concurrency timeline, critical-path computation, average-concurrency metric. |
| Rig parallelism authoring | TDs must express limb/finger/hair independence in topology; merge repeated groups into custom nodes; clothing off critical path can be "free". | Biped already expresses side independence (center/left/right layers, references-not-inherits to avoid cross-side animation coupling — `docs/biped-rig.md` §6) and shared `limb_frames.py`. But `final`-phase reads (e.g. `body_geo_skin` reads final of `pelvis_l_bind`) pin `_chainOrder` and serialize chains; `params.py::_execute_last` exists precisely because bottom-up order feeds solvers backwards. |
| Background frames | Premonition (adjacent-frame precompute); Presto Background Execution with starter executor; multi-graph 12–15× throughput at memory cost. | None. Each `Evaluate` is one synchronous generation; `rigExecPose --frames` is the batch path but frames evaluate sequentially. No starter sharing, no neighbor warming, no scrub predictor. |
| Memory allocators / hardware | TBB allocator +20%/−20% tradeoff; jemalloc; performance P-states +20%; CPU affinity +10% + steadier; NUMA watch on 4-socket. | No allocator policy stated; `README.md` "Current limitations" notes checkpoint memory (points × revisions) with no eviction policy, and per-point dirty masks as future work. No affinity/P-state guidance. |
| Operator language | C++ only, no Python in operators (GIL/speed). | Kernels are C++ (`rigExecMath`); Python `Builder`/`schema` is *authoring*-side, not compute-side — consistent with the restriction. |

## 4. Where time goes in the biped (measured)

From `reports/biped_eval.trace`: one Compile (~490 ms, one-time) + 3
evaluations (~35 / 31 / 30 ms, avg **~32 ms/eval ≈ 31 fps serial**).
Per-eval breakdown (summed over 3 evals ÷ 3; event counts ÷ 3 confirm the §1.2
census: 19 Parent, 13 Scale, 17 Aim, 4 Position, 12 Rotation):

| Cost/eval | Source | Note |
|---|---|---|
| 7.5 ms | 19 `RigExecParentConstraint` | per-axis masks + full compose, serial loop |
| 6.7 ms | 13 `RigExecScaleConstraint` | ~0.5 ms each — highest per-op cost |
| 5.6 ms | `PoseSeed` (exec tap request) | one exec `Compute` before the walk |
| 3.0 ms | Solver batches (14 aggregates) | snapshot-cached; this is the refresh cost |
| ~2.8 ms | Geometry chain `body_geo.points` | the single skin mover + derived work |
| ~2 ms | `AuthoritativeSnapshot` exec requests | ~0.65 ms × 3 |
| ~1.1 ms | Property chains (44 float-math movers) | resolve + override plumbing |
| 0.7 ms | 17 Aim, 0.5 ms 4 Position, 0.07 ms 12 Rotation | cheap frame math |

Compile (~490 ms) is dominated by `Digest.Solvers` (300 ms) + request
preparation (67 + 60 ms); `Digest.Movers` is 2.4 ms. Compile happens once per
epoch, so it is not the interactive problem — but the solver-digest cost is
worth a look if epoch rebuilds ever happen mid-scrub.

Two corrections to the prior revision: (a) the **pose walk (~22 ms, ~70% of
the frame) is the critical path, not the skin chain** — cross-target graph
parallelism would barely move the needle (one heavy target at ~3 ms);
(b) phase timing **exists** — the "no timing" row in the old §3 is fixed by
the scoped profiler; the remaining gap is concurrency/critical-path analysis.

Watt's trap applies directly: optimizing the skin kernel first would attack
<10% of the frame. The Parent/Scale constraint loop and PoseSeed are where
the critical path lives.

## 5. Recommendations (ordered, calibrated to §4)

**Do first (cut the measured critical path):**

1. **Profile every change against the trace.** Re-capture `biped_eval.trace`
   on the same 3-frame scrub as the acceptance baseline (§6). Surface the
   per-phase table in `rigExecPose` (e.g. `--profile`) and the Execution Stack
   panel — the local threading visualizer, scoped to serial costs first. Do
   not parallelize before re-measuring.
2. **Attack the Parent/Scale constraint loop (~14 ms/eval).** Per-op costs
   (~0.4–0.5 ms × 32 ops) point at per-step overhead, not math: hoist
   loop-invariant stage reads out of the 65-constraint loop, route every
   per-eval read through `_resolvedInputs`, keep the zero-weight/enabled-off
   early-outs (this rig's switch-prune pass). Then levelize: left/right limbs
   at the same depth should share a solver batch (one exec request,
   VDF-parallel inside) rather than occupying separate `_poseSteps` levels —
   treat level count as a performance metric. `docs/biped-rig.md` §5 shows
   order bugs (`twist_aims` before `hips_follow`, `neck_to_bind` before
   `spine_to_bind`) already dominate biped debugging.
3. **Shrink PoseSeed (5.6 ms/eval).** One exec `Compute` that seeds every
   provider frame before any pose op runs; audit which providers the walk
   actually reads and seed only those, or fuse the seed request with the first
   solver batch where dependencies allow.
4. **Internally thread the skin + derived kernels (Watt playbook).** Only
   *after* (2)+(3): `parallel_for` over points for LBS/normals/extent with a
   measured grain size and a serial threshold (10k-cycle rule), timing macro
   on/off so parity mode can force serial. Expect a modest single-frame win
   here (~3 ms chain) — it matters more as mesh density grows than it does
   today. Keep `cpuParityMode` comparing against the serial oracle.

**Do next (amortize and fuse):**

5. **Cache packet assembly inputs.** Weight-field resolution and static mover
   inputs should reuse across generations unless epoch digest or time changes
   — extend the solver-snapshot input-equality pattern to revision packets so
   an unchanged mover pays no assembly (README confirms assembly still runs
   per revision per eval).
6. **Fuse small exec requests.** Where a solver level holds one small
   aggregate plus a constraint immediately consuming it, evaluate them in one
   request if dependencies allow. Track `schedules_built` — any rise per
   scrub frame is a regression signal.
7. **Skip or defer derived normals/extent when cheap.** Synthesized
   unconditionally for any gprim authoring them; a preview LOD that does not
   author normals/extent sheds the work entirely. Short of that, compute
   extent from the skin kernel's min/max pass instead of a second walk.
8. **Bound checkpoint memory + cut Compile.Solvers.** Add the missing eviction
   knob from README limitations (keep N intermediates; `_snapshotPoints`
   already limits *which* revisions snapshot, add *how many generations*
   survive). Separately, profile the 300 ms solver digest — nothing in the
   interactive loop costs that much per epoch rebuild.

**Do later (throughput, system, long bets):**

9. **Neighbor-frame warming (Premonition-lite).** After an edit commits,
   evaluate adjacent frames into the existing snapshot cache on idle cores so
   scrubbing hits cache. Start with `rigExecPose --frames` batch throughput
   (parallel frames = separate evaluator states; memory cost is the known
   tradeoff, 12–15× cited for heroes) before touching interactive scheduling.
10. **System tuning pass.** P-state/performance mode, CPU affinity for the eval
    thread pool, thread-count control, and a TBB-allocator (or jemalloc) A/B
    on a fixed scrub — each is a one-line runbook entry if it reproduces the
    paper's +10–20% — plus a kill-switch (`RIGEXEC_SINGLE_THREADED=1` forcing
    serial kernels + VDF serial executor) for the inevitable "is it
    threading?" session. Add declared thread-safety categories for new
    parallel kernels at the same time (Watt §threadsafety).
11. **Keep rig-authoring parallelism.** Preserve the reference-not-inherit side
    split, minimize `final`-phase cross-chain reads, and document the
    "bottom-up Movers stack + joint-write propagation" rule (already in
    `docs/biped-rig.md` §5) as the rig-side parallelism contract. New limbs
    default to same-level batches.
12. **Non-goals for the biped.** Cross-target graph parallelism (one ~3 ms
    heavy target), GPU executor, LibEE-2-style dual-rep rewrite (the in-memory
    compiled graph already separates authoring from evaluation), hammock
    scheduling (rejected in the paper for complexity/benefit).

## 6. Suggested acceptance checks

- Baseline: `biped_eval.trace` 3-frame scrub — avg ~32 ms/eval, pose ~70%,
  Compile ~490 ms (Solvers digest 300 ms). Any optimization re-captures this
  trace first.
- After (2)+(3): same scrub, wall-time delta + parity agreement count
  unchanged (`mover_graph_parity_mismatches == 0`); Parent/Scale per-op ms
  down, PoseSeed down.
- After (4)+(6): `solver_evaluations` and `schedules_built` per scrub frame
  unchanged or lower; joint frames bit-identical to baseline
  (`verify_layers.py` 0.000e+00 cm gate).
- Memory: peak bytes over a 48-frame scrub before/after (8).

## 7. Limitations of this report

- §4 is 3 evaluations from one trace on one machine — a baseline, not a
  benchmark. No multi-frame throughput, no hardware-counter data.
- VDF/`ExecUsdSystem` internal threading was not traced into OpenUSD source;
  "parallelism only inside each request" assumes stock OpenExec executor
  behavior — confirm by reading the pinned OpenUSD `VdfExecutor::Run` or by
  observing core occupancy during one `graph.Evaluate(head)`.
- Google Docs were read via `export?format=txt`; embedded diagrams (Figs. A–C)
  did not survive extraction, so layout claims rest on text. The Watt PDF was
  read in full via `pdftotext` (`/tmp/watt_tbb.pdf` → 892 lines).
