# Baked program as a step graph (Phase 2 specification, v2)

Status: specification for the restructure of `RigExecBakedProgram` (libs/rigExec/bakedProgram.cpp)
into a dependency graph of steps. v2 incorporates two independent design reviews (exact-parity
lens, scheduling lens) of v1; the review findings are cited as [P#] (parity) and [S#] (scheduling)
where a rule exists because of them. Line numbers refer to bakedProgram.cpp on branch
`bake-all/infra` before Phase 2 (Run at 2124-2980). The implementer commits this document as
`docs/baked-step-graph.md` and keeps it in sync with the code.

Implementation status: §9's file split has landed, and so have the graph, the cost model, the
level-pack clustering and BOTH executors. `bakedProgramImpl.h` holds the program state, the slot
domains, `RigExecBakedStep`, `RigExecBakedCluster`/`RigExecBakedClustering` and the helpers more
than one file calls; `bakedPose.cpp` and `bakedGeometry.cpp` hold each domain's bake, its step
builders and its step bodies; `bakedSchedule.{h,cpp}` holds the edge sweep, the cost model, the
clustering, the serial and parallel executors, the calibration mode, the timing replay and the
`RIGEXEC_BAKED_SCHEDULE_REPORT` dumps; and `bakedProgram.cpp` keeps the public surface,
`IsBakeable`, the `Build` skeleton, the run prologue and epilogue, and `RigExecComparePoses`.
Every published value, diagnostic and compared counter of the three rigs that bake is
byte-identical to before, in serial and in parallel, at grain 0, at the default grain and at 200
microseconds. Vertex-chunked skinning (§6) and cone re-execution (§7) are still to come, and
until the vertex partition lands a revision is one whole-range chunk, so a biped's geometry tail
is a chain of four steps and the schedule's critical-path estimate is 490us of a 530us program --
which is why `RIGEXEC_BAKED_SCHEDULE` still DEFAULTS to `serial`: parallel is correct at every
grain but not yet faster, and the default flips after the acceptance matrix of §8.

The cost table in `bakedSchedule.cpp` is the one place a machine's numbers are written down. It
was fitted by `RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1` over eight frames of `Biped_anim` and should be
re-fitted whenever the shape of a step changes -- the vertex partition will change
`RevisionChunk`'s row in particular, because the row currently measures a step that spreads itself
over the arena. Calibration is opt-in, runs the serial executor whatever the mode asks for, and
times each step into that step's own accumulator; `Build` measures nothing, so the same program
produces the same schedule on the same machine however busy it is.

Two rules of §2 and §4.3 that a serial run cannot enforce, and where they are enforced instead:
the per-frame assemblers take no token-registry lock -- every `TfToken(const char *)` on the step
path is hoisted to a `TF_DEFINE_PRIVATE_TOKENS` block in `moverGraph.cpp` and `bakedGeometry.cpp`,
while the BIND-time readers (`RigExecResolveRevisionBinding` and the read-phase metadata) keep
their inline tokens because they run once per generation and off any step; and `Snapshots` is not
a source domain -- every record in the run's store is written by a step, so a step that reads the
store declares the steps before it, which `tests/testRigExecBakedSchedule` then checks like any
other read.

Nine deviations from the sections below, each made for a stated reason:

* The slot **kind** of §3 is called a slot DOMAIN in the code (`RigExecBakedSlotDomain`), because
  `RigExecBakedSlotKind` already says what a PROVIDER slot is.
* `InfluenceFold(c, r)` runs BEFORE `RevisionStatic(c, r)` and the packet depends on it.
  §6 wants the static packet to be independent of the influence matrices, which would mean
  splitting `RigExecAssembleParameters` in two; with one whole-range chunk per revision the chunk
  needs every influence anyway, so the split buys nothing until the vertex partition lands and it
  is a parity hazard until then.
* The `executed` decision is made in `RevisionStatic` rather than in `RevisionFuse`. The predicate
  and its inputs are exactly §6's (a value comparison, never dirtiness), but the CHUNK has to know
  it: without §7's cone to skip a clean revision, a fuse-side decision would run every skin kernel
  on every frame. The fuse still owns `applied`, `resultStatus`, the sticky chain bit and the
  `currentSource` indirection.
* `graphChainsBuilt` / `graphRevisionsBuilt` are summed from per-step counter deltas rather than
  taken as program constants (§4.2 item 7). A chain -- or a derived target -- whose base attribute
  does not read at the frame's time is skipped entirely by the dynamic path and by today's `Run`,
  counters included, and a constant cannot reproduce that.
* A commit's size in the cost model is its CANDIDATES PLUS its propagation pairs, not §5.1's
  `|propagate|` alone. Both halves of the commit walk the candidate table in slot order, and a
  solver batch with forty candidates and no descendants is not free.
* A `Derived` step's size is the CHAIN's vertex count, which §5.1 does not name. `recomputeExtent`
  walks the points it is maintained from and publishes two vectors; sizing it by its own array
  made the fitted per-unit cost 50us, which is the same number saying the model was wrong.
* Every recorder of the run's phased-read store declares the whole store up to its own step,
  instead of only its own slot, on a rig where something can look a record up (`phasedReads`).
  Per-step slots order a record against its READERS, which is what §4 asks for, but the records
  reach one container through a fold the executor performs, and two folds at once is a race
  whatever the slots say. Widening a declared write is always sound (§2.5), the widening costs
  nothing on a rig with no read phase, and the alternative -- a lock around the store -- is
  forbidden.
* The schedule report is TWO reports. `RigExecBakedScheduleReport` is structural and
  deterministic, so two builds of one stage produce the same text and a test can say so; the
  per-cluster wait and run times of §8.5 are in `RigExecBakedScheduleRunReport`, which needs a
  frame to have happened. §8.5's per-skin-revision chunk statistics wait for the vertex partition
  that creates chunks to have anything to say.
* The vertex partition of §6 not having landed, `RIGEXEC_BAKED_SCHEDULE=parallel` is not the
  default yet. §5.2 makes it the default "once §8 passes", and §8's chunk-count and chunk-vertex
  rows cannot pass before chunks exist.

Two things the parallel executor found that a serial one could not, recorded so they are not
rediscovered. A `RevisionChunk` reads the chain's running value BEFORE its revision, which is two
things: the earlier revisions' buffers, and the `currentSource` indirection that says which of
them to read. It declared only the buffers, so at one cluster per step a chunk could overtake the
fuse that decides the indirection -- deterministically wrong points on
`tests/testRigExecInteractive`, and invisible in every serial order. It now declares both. And
`RigExecStaticInputCache` answers a read from a worker thread by bypassing itself (its owner-thread
rule, moverGraph.h), so a step running off the evaluator's thread resolves its inputs the long way
and gets the same value; what moves is that cache's bypass COUNTER, which
`tests/testRigExecStaticInputCache` asserts on only for its own fixture. The counter's increment is
itself unsynchronised, which is a real data race on a statistic and should be made relaxed-atomic
before parallel becomes the default.

## 1. Why

The baked program is the OpenExec replacement. Today `Run` is one straight line: inputs, compose
every provider, the interleaved solver/constraint walk in `_poseSteps` order, matrices, then every
geometry chain revision after revision, then derived maintenance. That order is the dynamic path's
order and it is correct, but it is only ONE valid order, and it serialises independent work:

* a geometry chain whose influences are all finalised after pose step k waits for every later
  pose step;
* a skin revision waits for ALL of its influences even though most vertices are moved by a handful
  of joints -- the arm vertices wait for the leg constraints;
* two constraints on disjoint subtrees run one after the other;
* a drag on one control re-runs the whole program because nothing records what depends on what.

User requirements, in intent: "interleave and break apart/parallelize constraints and mover
operation scheduling"; "break up skincluster into discrete scheduled joints/matrix operations
within the scheduler with chunks of vertices being moved and fused back together; each joint
should have its own constraint/control path dependencies so the entire rig becomes parallelizable
during scheduling and we can bake that in for bake mode".

## 2. Non-negotiables

1. **Exact parity.** Every published value, every diagnostic (in order), every compared counter of
   `RigExecRigPose` is bit-identical to the dynamic path in every execution mode, with and without
   interactive overrides. `RigExecComparePoses` is the judge (it compares the seven maps,
   solverFrames, weightFields/weightFrames after Phase 1, solverOverrideRounds, the three mover
   graph counters, and diagnostics in order; it excludes solverEvaluations). A schedule that is
   faster and differs in one ulp is wrong.
2. **No locks in step bodies.** No mutex, spin lock, or condition variable is taken inside a step.
   The scheduler's only synchronisation is one cache-line-padded `std::atomic<int>` per cluster
   [S18] plus `WorkDispatcher` task spawning inside `WorkWithScopedParallelism` [S14]. Everything
   that does take a lock today is moved to the serial prologue or epilogue: `RigExecSkinTopologyCache::Resolve`
   (mutex held across the build, moverGraph.cpp:1013) [S36], `RigExecCurvenetBindCache` (not
   thread-safe at all, moverGraph.h:508-539) [S36], and every `RIGEXEC_PROFILE_SCOPE*` (three mutex
   acquisitions per scope even when profiling is off, profiler.h:70-74, 329-335) [S16]. Steps write
   only the slots they declared; two runnable steps never share a written slot.
3. **Shared kernels.** Steps call the kernels the mover graph and the exec callbacks call
   (`RigExecApplySkinKernel`, `RigExecRunRevisionKernel`, `RigExecSolve*`, `RigExecApply*Constraint`,
   the Phase 1 hoists). A chunked skin step calls a RANGE form whose per-vertex body is the one
   definition the full-range form runs (§6). Never the CPU oracles.
4. **Serial order is the reference.** The graph is derived from today's program order. Executing
   the steps serially in that order must be byte-identical to today's `Run`; this is verified first
   and kept forever as `RIGEXEC_BAKED_SCHEDULE=serial`.
5. **Dependencies come from declared slots.** A step lists the slot ranges it reads and writes;
   edges are computed mechanically (§4). Declared writes are an UPPER BOUND: a step may write fewer
   (a disabled constraint writes nothing, :2416-2418, :2425-2438, :2452-2457, :2566) and an
   executor must never "write what was declared" [P35]. A step touching an undeclared slot is a bug
   the verifier mode (§8) catches.
6. **`pose` is untouched inside the parallel region.** Every `pose->...` push or increment in
   today's walk (:2244, :2405, :2407, :2814, ...) becomes per-step storage summed or concatenated
   in the epilogue [S36]. No `TF_WARN`/`TF_ERROR` from a step body.

## 3. Slots

`RigExecBakedSlot = uint32_t` (kind in the top 8 bits, index in the low 24) and
`RigExecBakedSlotRange {kind, begin, end}` [S6]. Provider slot order is namespace DFS pre-order,
so a subtree is a contiguous range and `buildPropagation` (:1910) already enumerates descendants
as a contiguous scan; declaring ranges collapses the commit edge count by an order of magnitude
and makes Build's edge computation an interval sweep.

| kind | index | storage | written by |
|---|---|---|---|
| Avars | provider i | `avars[i*11..+10]` | Inputs (prologue) |
| PoseBase | i | `base[i]` | ComposeSubtree, SolverCommit(+Apply) |
| PoseFin | i | `fin[i]` | ComposeSubtree, SolverCommit, Constraint, CommitApply |
| PosedM | i | `posedM[i]` | ComposeSubtree |
| FinalMatrix / BaseMatrix | i | `finalMatrix[i]` / `baseMatrix[i]` (`_Impl` members, not per-frame allocations [S34 #7]) | ProviderMatrix |
| Aggregate | solver s | `aggregates[s]` | Solve(s) |
| Candidates | Solve step / Constraint step | per-step dense `{slots[], frames[]}` scratch [S34 #2] | that step |
| Propagated / Delta | commit step | per-step staging | CommitDelta / PropagateChunk |
| PropertyResult | property-chain target t | `E._resolvedInputs` entry + `propertyResults[t]` | PropertyChains (prologue) |
| ChainBase | chain c | `chains[c].lastBase` | ChainBase (prologue) |
| RevisionPacket | (c, r) | static packet: indices, weights, topology handle, resolved envelope, enabled/status inputs | RevisionStatic(c, r) |
| RevisionTransforms | (c, r) | full influence table, validity flag, SIMD rows, DQ palette | InfluenceFold(c, r) |
| RevisionOut | (c, r, chunk k) | vertex range of `revisions[r].output` (the revision's OWN buffer) | RevisionChunk(c, r, k) |
| RevisionDone / ChainDirty | (c, r) | applied/executed/status flags, the sticky chain dirty bit [S1] | RevisionFuse(c, r) |
| DerivedOut | (c, d) | `derived[d].output` | Derived(c, d) |
| FallbackJoints | program | per-step lists, merged into one `std::set<SdfPath>` in the epilogue [P9][S1] | Solve steps |
| WeightPacket / WeightFrame / ConstraintDelta / ControlFrames | Phase 3 | | |

One slot per property-chain target [S4], not one PropertyResults slot: a step's reads are the exact
target paths its `resolvedAttr` inputs walk (Build knows the walk).

Slot storage is owned by `_Impl`, sized at Build, persists across runs, and is NEVER resized inside
the parallel region [S33]. A chain whose point count moved with time is reset for the whole chain in
the prologue (today's :2772-2787 block). A step that bails writes nothing, so its slots hold the
previous run's well-formed values at this run's size; successors may read them without size checks.

### 3.1 Multi-writer slots and versions [P20]

`PoseFin(i)` and `PoseBase(i)` have several writers per run (compose, commits, propagation). A step
reads a specific VERSION: the one live at its program point. Build records for every read
`(slot, version k, writer W_k)`. Cone re-execution (§7) must therefore also re-run W_k whenever a
dirty reader needs version k and the slot has a later writer in program order (the storage holds
the last writer's value). This "restore closure" is computed at Build (§7) and is what makes
"clean steps keep last run's values" sound.

## 4. Steps and the graph

`struct RigExecBakedStep`: `kind`, `payload`, `reads`/`writes` (sorted range lists), `preds`/`succs`,
`cluster`, `cost`, per-step preallocated scratch, `std::vector<std::string> diagnostics` (at most
`kMaxStepDiagnostics = 4` lines per run; every diagnostic site in the walk is terminal [S17]; no
`reserve()` at Build), per-step counter deltas, `bail` flag.

Program order is today's `Run` order. The prologue and epilogue are serial code, not steps.

**Prologue (serial, always runs)** [S4][P22][P36]: `E._resolvedInputs.Clear()`; `E._chainSnapshots`
handling per SI-4 (Run-local store); apply interactive overrides to resolved (pre); property
chains, appending straight into `pose->diagnostics` (they run first today too, :2149) and writing
one PropertyResult slot per target; apply overrides (post); the bound-inputs block exactly as today
(:2170-2194) including the constant-avar pass and the single `avarsDisturbed` flag update, ONCE per
run; ChainBase for every chain (base query read, compare, the point-count-moved reset,
`accountForChain` flags); `RigExecSkinTopologyCache::Resolve` for every skin revision (the only
mutex; a hit after the first frame) [S36]; curvenet binds (Phase 3) likewise; the dirty-set
computation of §7.

**Steps, in program order:**

1. `ComposeSubtree(r)` for each subtree root r of a Build-time partition of the provider forest
   into ~`total/(4P)` providers per subtree (cut at the deepest nodes whose subtree exceeds the
   budget; a biped yields ~10 cuts) [S10]. Reads `PosedM(parent(r))`, `Avars[r, r+n)`; writes
   `PoseBase/PoseFin/PosedM [r, r+n)`; executes today's compose loop body (:2196-2224) over the
   range, which is a pure forward pass with `parent[i] < i`.
2. For every `_poseSteps` entry, in order:
   * solver batch: `Solve(s)` per solver (reads `PoseFin` of its controls, `Aggregate` of its inputs
     for BlendPointFrames -- a loop-carried read of LAST run's value when no earlier writer exists,
     so the reader list is seeded from program start [P23]; writes `Aggregate(s)` and its own
     Candidates scratch and FallbackJoints list). `Solve` clears the aggregate BEFORE the type
     dispatch, and the `degenerate` guard (§10) sits after that clear [P25]. Then `SolverCommit(p)`:
     merges the per-solver candidate lists in `batchSolvers` order (last writer wins on a duplicate
     slot, as `candidates[slot] = ...` does today), then iterates in SLOT order for both the
     usability sweep and the write-back [P24]. Reads `Candidates` of the batch, `PoseFin(j)` and
     `PoseFin(closest)` for every propagate pair [S3]; writes `PoseFin` and `PoseBase` of every
     output and propagated slot (`solverOutput` writes both, :2293-2295).
   * constraint: `Constraint(c)` = compute + commit. Reads `PoseFin` of target, sources, world-up
     object, and of every propagated descendant and its closest ancestor (read-modify-write) [P27];
     writes `PoseFin` of target and propagated descendants (not PoseBase).
   * A commit with `|propagate| > 64` is split [S11]: `CommitDelta(p)` computes the delta into a
     `Delta` slot; `PropagateChunk(p, k)` over descendant index ranges writes staged frames and an
     `ok` flag into per-step scratch; `CommitApply(p)` ANDs the flags, emits the diagnostic for the
     LOWEST-indexed failing descendant (today's loop returns at the first failure in `propagate`
     order), and copies staged frames into `PoseFin`/`PoseBase`. Atomicity and diagnostic preserved.
3. `ProviderMatrix(i)` for every slot some later step or the epilogue reads a matrix of, computing
   exactly the set today's lazy `finalMatrixOf`/`baseMatrixOf` compute (final for published joints
   and final-phase influences, base for base-phase influences; both when both are used) [P34].
   `FinalMatrix(i)` reads `PoseFin(i)`; `BaseMatrix(i)` reads `PoseBase(i)` [P26]. These steps
   NEVER bail: `RigExecPointsToMatrix` leaves identity and returns false; the only `return false`
   is the joint-publication one and it stays in the epilogue [P33].
4. Geometry, per chain c and revision r: `RevisionStatic(c, r)`, `InfluenceFold(c, r)`,
   `RevisionChunk(c, r, k)` for each chunk, `RevisionFuse(c, r)` (§6); then `ChainStatus(c)` (the
   per-chain `moverFailed` sweep over persisted `resultStatus`, :2853-2859) [P8][S2]; then
   `Derived(c, d)` reading `RevisionDone(c, last)` [S7] plus the last revision's output buffer.

**Epilogue (serial)**: publish; diagnostics; counters; bail; drain. See §4.2.

### 4.1 Edges

Once at Build, in program order: for each read range add an edge from every `lastWriter` overlapping
it; for each write range add edges from overlapping `lastWriter`s and from every reader since (WAR);
then update `lastWriter` and clear the reader lists for the range. Interval sweep over sorted
ranges. Readers accumulate from program start [P23]. Deduplicate; store `preds`/`succs`. Any
topological order -- including a parallel one -- then produces the serial result, because
floating-point results depend only on operand values and per-step arithmetic is fixed. Invariant:
every edge points forward in program order, so a cluster executing its members in increasing
program index is always in topological order [S8].

Biped estimate: ~300 steps after ComposeSubtree and chunking (down from ~2 500 with per-provider
compose), a few thousand edges.

### 4.2 Epilogue, in order [P10]

1. (already emitted) settle diagnostics from the caller and property-chain diagnostics from the
   prologue.
2. Walk diagnostics: per-step lists concatenated in program (step) order -- commit lines, constraint
   lines, world-up lines.
3. Fallback joints: merge every Solve step's list into one `std::set<SdfPath>`, emit in path order,
   de-duplicated (today :2577-2590).
4. Joint publication in `_jointPaths` order: `jointFramesBase`, `jointFramesFinal`, the degenerate
   final-frame diagnostic, `jointMatricesFinal` from `FinalMatrix`, and the ONLY bail in this block:
   `!_Usable(restFrames[slot])` for a valid, non-degenerate final frame → return false [P33].
   Control frames. Solver guides: read `B.aggregates` directly under the runtime toggle
   `E._guideTaps && E._solverGuidesEnabled`, never cached in a step [P22].
5. Property-domain results into `movedProperties`.
6. Per chain in chain order: each revision's `RevisionStatic` diagnostics (the `inputs:defaultWeight`
   line, :2800, emitted whether or not the revision executed), then `ChainStatus` lines in revision
   order, then `movedProperties[target]` from the chain's double-buffered `VtVec3fArray` (§4.3),
   then each derived target's diagnostic and publication.
7. Counters: `solverOverrideRounds` = number of distinct batch levels in the program (a program
   constant, :2404-2406) [P15]; `solverEvaluations` = Σ batch sizes (baked meaning, every run)
   [P16]; `moverGraphRevisionsCreated/SchedulesBuilt` from the always-run prologue flags;
   `moverGraphRevisionsExecuted` = Σ per-revision `executed` from the fuses;
   `graphChainsBuilt`/`graphRevisionsBuilt` = program constants (Σ chains + derived, Σ revisions +
   derived) [P17][S27].
8. If any step bailed: return false NOW, before the curvenet drain -- today every `return false`
   precedes the drain and the dynamic fallback emits the pending bind lines [P14][S31]. The caller
   destroys the program (`_bakedProgram.reset()`, rigEvaluator.cpp:8001-8006), so no slot state left
   by a bailed run is ever read again [P12][S32]; a scheduler may stop seeding clusters after a bail
   but must not alter what an in-flight step writes [P13].
9. Curvenet bind-cache drain (exactly once, never replayed), then the `mover graph: ...` summary line.
10. `pose->valid = true`.

### 4.3 Allocation rules [S34]

No step allocates on a path another step allocates on concurrently; per-step scratch is sized at
Build from the declared payload: candidate lists, `sources`, FK `elements`, propagation staging,
per-chunk influence tables, the per-revision resolved envelope buffer, the per-chain running buffers.
`finalMatrix`/`baseMatrix` become `_Impl` members (today 267 KB zero-filled per frame). Each chain's
published `VtVec3fArray` is DOUBLE-BUFFERED: two persistent arrays alternate, so publication is a
refcount bump and the array a consumer may still hold from last frame is never mutated (COW aliasing
rule). Skin scratch copies (`scratch = current`, `preceding = scratch`, `revision.output = current`)
disappear because chunks write the revision's own buffer (§6). `TfToken(const char*)` constructions
inside assemblers are hoisted to `static const TfToken` -- a file-scope `TF_DEFINE_PRIVATE_TOKENS`
block in practice, which is the same thing without a guard variable per call. A commit's staging
slots are numbered across the whole program, not within one commit: per-commit ids would make the
sweep order two unrelated commits' chunks against each other.

## 5. Clustering and execution

Implemented. `RigExecBakedAssignStepCosts` fills every step's size, cost and longest-path level;
`RigExecBakedBuildClusters(program, grainUs)` is a PURE function from a program and a grain to a
partition, which is what lets a test ask one program for three schedules and compare them;
`RigExecBakedScheduleGrainUs` applies `RIGEXEC_BAKED_GRAIN_US` or computes the clamped default.
Build calls all three once and allocates one cache-line-padded counter per cluster. On a biped
(452 steps, 767 edges, 20 cores) the default grain is 6.6us and the partition is 54 clusters.


### 5.1 Clustering [S8][S9][S12][S13]

Cost model: `cost = a[kind] + b[kind] × size(step)`, with size = |propagate| for commits, |controls|
for FK, joint count for spline IK, `vertices × elementSize` for skin chunks (NOT × influences),
|influences| for InfluenceFold; constants in a comment-documented table calibrated from
`rigExecPose --profile`. Build never measures (a schedule depending on machine state breaks the
byte-identity-across-grains test); an opt-in `RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1` prints a
replacement table from a lock-free per-step timer.

Algorithm (level-pack):

```
level[v] = 1 + max(level[u] for u in preds[v], default 0)   # program order is topological
total = Σ cost; P = WorkGetConcurrencyLimit()
grain = clamp(total / (4P), 5us, 50us)                        # RIGEXEC_BAKED_GRAIN_US overrides
for each level L (steps in program order):
    k = clamp(ceil(Σcost(L) / grain), 1, P); pack L into k contiguous bins by cost
repeat until fixpoint: contract quotient edge (A,B) when |succs(A)|==1 and |preds(B)|==1
absorb: a cluster with cost < 2 × spawn cost whose preds are all one cluster C joins C
```

Theorem used: with longest-path levels no edge joins two steps of the same level, so any same-level
grouping is acyclic. Merge invariant for the absorb rule: `pred(s) ⊆ C ∧ pred(s) ≠ ∅`, evaluated on
the CURRENT quotient graph after each merge. Target: clusters ≈ 2-4 × P (16-32 on a 20-core box),
never a fixed count. Correctness never depends on clustering; byte-identity across grains is tested.

### 5.2 Execution modes

`RIGEXEC_BAKED_SCHEDULE` (read once, `TfGetenv`), also a program option for tests:

* `serial`: steps in program order on one thread. Reference. Forced when
  `RigExecParallelEvaluationEnabled()` is false (`RIGEXEC_ENABLE_PARALLEL_EVAL=0`) [S14].
* `parallel` (default once §8 passes): inside `WorkWithScopedParallelism` (the precedent is the
  chain-level walk, rigEvaluator.cpp:10380-10394; `Run` is never entered from an exec callback, but
  a client may call `Evaluate` from a TBB task, which is exactly what isolation covers) [S14][S15]:
  one padded atomic remaining-predecessor counter per cluster, reset at run start; seed every
  zero-predecessor dirty cluster; a finished cluster decrements its successors, `Run`s all but one
  of those reaching zero and executes the last one inline on the finishing thread [S13];
  `dispatcher.Wait()`; then the epilogue.
* No profiler scopes inside step bodies. Per-step start/end timestamps go to preallocated arrays
  (one store each) and the epilogue replays them into the profiler in step order, which also makes
  the trace deterministic [S16]. Scopes remain around prologue, region, epilogue.
* Static-input cache: reads from workers bypass `RigExecStaticInputCache` (owner-thread rule) and
  read the stage the long way; values are identical, the bypass counter moves, and
  `tests/testRigExecStaticInputCache` asserts on that counter only for its own fixture -- check
  before merging [P37][S36].

## 6. Skin clusters as vertex-chunked, per-joint steps

Per-vertex separability holds for LBS (scalar accumulates in double in slot order, SIMD in float one
point per iteration, no cross-vertex normalisation or bounds pass; moverGraph.cpp:604-608 already
argues that a split boundary cannot land inside a vectorised block) and for DQS point work [P4].
Everything ELSE about a skin revision is a whole-array decision [P1][P6][S20]: layout validation
(`Validate()` scans every index/weight/matrix; the fixed-topology gate compares `pointCount` to the
whole array), the influence table's finite/affine check that decides `params.valid` and the status,
`ResolveAll` of the envelope (fails atomically on any bad element, requires the full count, sparse
indices strictly increasing), DQS returning false at the first degenerate point leaving `out`
unspecified, an unknown skinning method. The design that keeps both facts:

* **Kernel range form.** `RigExecApplySkinKernelRange(packet, transformsView, begin, end, out)` holds
  the per-vertex body verbatim and performs NO validation and NO `WorkParallelForN` (unconditionally
  serial) [S21]; the full-range `RigExecApplySkinKernel` validates, then calls the range form inside
  its existing `WorkParallelForN` with the existing grain/threshold. `RigExecBlendEnvelopeRange`
  takes the PRE-RESOLVED full envelope array and absolute indices [P6]. `RigExecApplyMatrixKernel`
  gets the same range form taking a pre-resolved envelope [P7]. The DQ palette is built once per
  revision and passed by pointer [S22].
* **`RevisionStatic(c, r)`** (a source step, always runs, §7): assembles everything that does not
  depend on influence matrices -- enabled/defaultWeight/status inputs, indices/weights/topology
  handle (resolved in the prologue), `ResolveAll` of the envelope at the FULL point count into the
  per-revision persistent buffer, `RigExecEnvelopeIsFullStrength` -- validates it, compares with last
  run for dirtiness, writes `RevisionPacket(c, r)`.
* **`InfluenceFold(c, r)`**: reads `FinalMatrix`/`BaseMatrix` of EVERY influence in binding order,
  writes the full `skinTransforms` table (never partially initialised: `GfMatrix4d`'s default ctor
  does not initialise, and `Validate()`/the SIMD narrowing read every entry [P3]), the finite/affine
  validity flag, the SIMD rows table and the DQ palette, and compares the table against last run's
  for the executed decision. O(influences); ~1-2 µs. It is a real dependency of the revision on all
  its joints -- but only the FUSE waits on it, not the chunk work (below).
* **Partition at Build** [S19]: chunks are CONTIGUOUS vertex ranges of `RIGEXEC_BAKED_CHUNK_VERTS`
  (default 4096 = `RigExecGeometryParallelThreshold`), capped at `RIGEXEC_BAKED_MAX_CHUNKS` (default
  32; the range grows to meet the cap). A range's key is the UNION of its vertices' influence slots
  as a bitset over the revision's influence count; adjacent ranges merge while under the vertex cap
  and one key is a subset of the other. Vertex order is never permuted. Build cost: one pass over
  the indices. Trade-off, stated honestly: a range spanning two body regions (mirrored meshes,
  merged shells) depends on both and simply waits longer; the other chunks still start when their
  own joints land, which is strictly better than today. The schedule report (§8.5) prints per chunk
  |key| and the ready level so the claim is measurable per asset.
* **`RevisionChunk(c, r, k)`** is SPECULATIVE [S20][P2]: it reads `RevisionPacket(c, r)`, the
  `FinalMatrix`/`BaseMatrix` of its key influences only, and the overlapping chunks of revision
  r-1's buffer (or `ChainBase` for r = 0); it writes its range of `revisions[r].output` -- the
  revision's OWN buffer, disjoint from r-1's -- and an `ok` flag. It starts the instant its joints
  are final. Per-chunk scratch: a transforms table sized to the influence count, identity-filled,
  with the chunk's own influences copied from the matrix slots (LBS never reads an entry outside the
  chunk's key), and per-chunk SIMD rows narrowed for the same entries (per-matrix operation,
  identical values). DQS chunks read `RevisionTransforms` instead (the palette depends on all
  influences) and therefore depend on the fold; LBS chunks do not.
* **`RevisionFuse(c, r)`** reads `RevisionPacket.valid`, `RevisionTransforms.valid` and the table
  compare, every chunk's `ok`, `RevisionDone(c, r-1)` and `ChainDirty(c, r-1)`. It decides, in
  today's terms: `executed` = chain dirty so far OR `!ran` OR static packet changed OR transforms
  changed OR status changed (value comparison, never dirtiness -- a control dragged back to the same
  value must NOT count as executed, matching the VdfNetwork) [P18][S27]; `applied` = status allows AND
  valid AND every chunk ok AND size checks; on `!applied` the revision's `currentSource` indirection
  points at revision r-1's buffer (today's `current = revision.output` with no copy), and successful
  chunks are discarded, not kept [P2]. It sets `resultStatus`, `ChainDirty(c, r)` (sticky within the
  chain: once a revision executed, every later one counts as executed, :2789/:2849) and the
  `moverFailed` state that `ChainStatus(c)` later reports. There is no fuse copy.
* Every other geometry op is one chunk with its whole influence/driver set as key WHEN it becomes
  bakeable (Phase 3) [S38]; the durable rule for the geometry group: smooth and surfaceProject read
  neighbours or another chain's surface and are never separable; blendShape/lattice/volumeCorrect
  with per-vertex weights may use the range mechanism later.

Per-joint control/constraint paths then hold end to end for the arithmetic: control avars →
ComposeSubtree → Solve/Commit → Constraint and propagation → ProviderMatrix(i) → the chunks whose key
contains i → fuse → derived → publish. The revision-level validity fold is a separate, cheap
reduction over all influences [P3].

## 7. Cone re-execution (drags and static frames) [S24-S28][P20-P22]

Source steps are the only steps that read outside the program: Inputs, ChainBase, PropertyChains
(all prologue), RevisionStatic, and Phase 3 weight-object readers. Every other step is a pure
function of its declared reads. Sources ALWAYS run and compare their outputs by VALUE against last
run (11 doubles per provider, the base arrays, per-target property results, the static packet);
"time changed" or "overridden" is never the predicate. This is what makes an override on a
`resolvedRoutedPrims` prim (which sets no `overridden` flag, :1341-1345), a released drag
(`avarsDisturbed` pass), a cleared topology cache, and a moved keyframe all reach the graph through
the same test. A notice that does not invalidate the program (a value edit on a re-read input,
IsInvalidatedBy's documented gap) bumps a program stamp from the evaluator's notice handler and
marks everything dirty for one run; Set/ClearInteractiveOverrides do NOT bump it.

```
Build:  coneOf[c]    = forward closure of cluster c (bitset over clusters, |clusters| <= 128)
        restoreOf[c] = for each read (slot, version k) of a step in c whose slot has a later writer:
                       the writer's cluster, closed transitively over ITS reads (§3.1)
Run:    dirty = everRan && stamp == lastStamp ? OR of bit(c) for sources that wrote a different value
                                             : ALL pose clusters ∪ geometry clusters with !revision.ran
        closed = OR of coneOf[c] for dirty c;  closed |= OR of restoreOf[c] for c in closed  (fixpoint)
        remaining[c] = closed[c] ? |preds[c] ∩ closed| : skipped
```

Cost: a few hundred bitset ORs. The initial dirty set after a rebuild is all pose clusters plus the
geometry clusters whose adoption (`AdoptGeometryStateFrom`) did not keep `ran` -- never all geometry,
or every value-edit rebuild re-deforms everything and reports counters the dynamic path does not
[S28]. Skipped steps keep their slots and their stored diagnostics, which the epilogue replays (every
diagnostic is path + fixed text, none embeds time) [S31]; the two set-ordered/sticky items (fallback
joints, chain status) are recomputed from persistent state every run [P11]. The curvenet drain is
never replayed. Accounting stays exact because `executed` is a value comparison (§6) and the
structural counters are constants (§4.2).

Measure and record (§8.4): drag frame cost with and without the per-override-set
`_skinTopologies.Clear()` in `SetInteractiveOverrides` (rigEvaluator.cpp:7853); if it dominates,
§10 clears only when the override set names a skin mover property [S29].

## 8. Acceptance (all mandatory before Phase 3 starts)

1. `RIGEXEC_BAKED_SCHEDULE=serial`: pose dumps byte-identical to today's baked run and to the dynamic
   baseline for Biped, Biped_anim (frames 1-8), spider_legs; all `*BakedParity` and example parity
   ctest entries pass with `RIGEXEC_BAKE_REQUIRED=1`.
2. `parallel`: the same, for `RIGEXEC_BAKED_GRAIN_US` 0 (one step per cluster), default, and 200;
   for `RIGEXEC_BAKED_MAX_CHUNKS` 1, 8, 32; for `RIGEXEC_BAKED_CHUNK_VERTS` 512 and 4096.
3. Drags: `tests/testRigExecExampleParity` and the biped drag tests pass in both modes with
   `RIGEXEC_BAKED_VERIFY_CONES=1`, which re-runs everything after the cone run and compares every
   slot, the counters and the diagnostics [S27]. Add a test where a control is dragged and returned
   to its exact original value (executed counter must not move) and one where a constraint is
   dirty while its target's compose is clean (the restore closure).
4. Performance (informational, `rigExecPose --profile` with prologue / region / epilogue as
   separate lines [S30]): serial mode within 5% of today's `Run` on the biped frame; parallel mode
   faster than serial on the biped and on the drag benchmark. Expectation, not a target: the
   epilogue's map publication (~50-150 µs) is irreducible serial work, so the biped frame should
   land around 2-2.5× faster than serial, not 8× [S30]. A serial regression means per-frame work
   that belongs at Build.
5. `RIGEXEC_BAKED_SCHEDULE_REPORT=1` prints steps, edges, clusters, critical-path estimate, and per
   skin revision: chunk count, vertices per chunk, |key| min/mean/max, fraction of chunks covering
   ≥ 50% of influences, each chunk's ready level vs the global max level, and skin critical path vs
   skin serial cost [S23]. Include the biped's report in the commit message or docs.

## 9. Code layout [S35]

`RigExecBakedProgram::_Impl` is a private nested type and the evaluator has exactly one friend
(`friend class RigExecBakedProgram`), so free functions in other files can name neither. Therefore:

* `libs/rigExec/bakedProgramImpl.h` -- namespace-scope `struct RigExecBakedProgramImpl` (no
  reserved `_Impl` name) holding everything today's `_Impl` holds PLUS the evaluator state captured
  ONCE at Build inside the one TU that is the evaluator's friend: pointers to `_resolvedInputs`,
  `_skinTopologies`, the program-owned curvenet cache, `_profiler`, the chain snapshot store,
  `_jointSolverBinding`, `_guideTaps`/`_solverGuidesEnabled`, and whatever else `Run` reads. Also:
  slot ids/ranges/kinds, `RigExecBakedStep`, the `_Input`/`_Read` templates renamed without the
  reserved prefix, and the multi-file helpers (`_ComposeAvars`, `_Usable`, avar tables, `_OpName`)
  as `inline`. `bakedProgram.h` keeps a forward declaration and the `unique_ptr`.
* `libs/rigExec/bakedProgram.cpp` -- public API, `IsBakeable`, `Build` skeleton calling per-domain
  bake functions, `SetOverrides`, `IsInvalidatedBy`, `AdoptGeometryStateFrom`, Build-only helpers,
  and `RigExecComparePoses` (stays; it is the parity judge and touches no impl).
* `libs/rigExec/bakedSchedule.{h,cpp}` -- edge construction, clustering, serial/parallel executors,
  cones, the report, the calibration mode.
* `libs/rigExec/bakedPose.cpp` -- Compose/Solve/Commit/Constraint/ProviderMatrix builders and
  executors (Phase 3 solvers and constraints groups edit here).
* `libs/rigExec/bakedGeometry.cpp` -- chain/revision/chunk/fuse/derived builders and executors
  (Phase 3 geometry group).
* `libs/rigExec/bakedWeights.cpp` -- Phase 3 weights group creates it.

Adding a step kind = one enum value, one builder declaring reads/writes/payload/cost, one executor
case. Documented in `docs/baked-step-graph.md`.

## 10. Also in Phase 2 (bakedProgram-side shared infrastructure from the plans)

Prerequisites the operator groups share; all live in bakedProgram, so they land here to keep Phase 3
worktrees from colliding. None may change a published value OR a diagnostic on the three rigs that
bake today (diagnostics are compared) [P29]:

* slot-kind table: `paths` = ordered union of `_poseSeedFrames` and `_xformDerivedProviders` with
  `slotKind`, `parent[]` (nearest compose ancestor, used by compose) and `propParent[]` (nearest
  ancestor of any kind, used for the `closest` climb). The DESCENDANT role in `buildPropagation` is
  filtered to `slotKind == PoseSeed`, matching the dynamic `hierarchicalProviders` filter
  (rigEvaluator.cpp:8580-8585, :8674); `ownedBySolver` stays keyed on path. Land this alone and diff
  the biped dump before anything else [P28].
* `_Solver::degenerate` (guard placed after the aggregate clear [P25]); the jointElements resolution
  fix and duplicate-slot check (plan_solvers) -- diff diagnostics too, since they change
  `fallbackJoints` [P29].
* SI-3 basePoints contract: per-revision assemble receives the chain's AUTHORED base; the DERIVED
  assemble keeps receiving the chain's FINAL points (rigEvaluator.cpp:10272; recompute
  normals/extent take `auxPoints = basePoints`) [P30].
* SI-4 Run-local `RigExecChainSnapshots` + overlay recorded per step and absorbed in the epilogue,
  recording exactly today's set of matrices [P34]; SI-5 program-owned `RigExecCurvenetBindCache`,
  resolved in the prologue (the class is not thread-safe) [S36], carried by `AdoptGeometryStateFrom`,
  drained in the epilogue after the bail check [P14]; `_GeomRevision::driverFramesSolver` and
  `snapshotAfter` fields; SI-6 invalidation-index coverage note.
* `_WeightObject` table skeleton and the `WeightPacket` step kind wired to Phase 1's
  `weightPackets.h`, unused by any rig until Phase 3.
* `constraintDeltas` slot kind and its `find`-guarded consumption (mirrors
  rigEvaluator.cpp:9955-9958) [P31]. Consumed in `InfluenceFold`, not in the assemble: the delta
  IS the revision's transform, the fold is the step that declares `RevisionTransforms`, and a step
  writing a slot it declared as a read is the declaration an executor trusts being wrong.
* The comparator gained weightFields/weightFrames/solverOverridesConverged in Phase 1; §8 relies on
  that [P32].
