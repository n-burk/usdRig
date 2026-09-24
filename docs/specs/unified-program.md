# Unified program: one loop graph for baked and dynamic mode, and a lower compile floor

Status: design, 2026-09-23. Nothing here is implemented yet. This builds on
`docs/specs/baked-step-graph.md` (called "the step-graph spec" below) and on
`reports/scheduling-audit-2026-09-23.md` (called "the audit"). Line numbers refer to the working tree of
2026-09-23, which has the audit's S1a/S4/S1b/S6A/S6B/S13 changes on top of 64d6a5f.

Bracketed tags name the review that produced a rule:
* `[CF1-deps]` is the deps lens on proposal CF1.
* `[EP2-gain]` is the feasibility/gain lens on EP2.
* `[audit S8]` is a row of the audit's §5.

Every figure is a measurement on the biped (`examples/biped/Biped_anim.usda`, `/Biped/Rig`, 32 logical CPUs)
unless it is marked *est.* (estimated, unmeasured). The machine was busier than in the audit's morning runs:
the cold baked Compile median was 87.5-92.0 ms today, against 80.0 ms in the audit's §7. Every A/B pair below
was run within one session window.

---

## 1. Goals and non-goals

### 1.1 Goals (user product requirements)

1. **Lower the compile floor.** In order: the exec warmup lane and the structure-digest lane, then the
   main-thread work those two unblock, then the inline settle digest on the edit path.
2. **Dynamic mode runs the same single loop graph as baked mode.** That graph must be:
   * **Parallelizable.** It runs under the same executors (serial, parallel, frozen), and the choice between
     them is deterministic.
   * **Editable.** Interactive overrides and stage edits reach the next frame without a full recompile whenever
     the edit is not structural. Structural edits recompile as little as possible.
   * **Sparse.** When an input changes, only the affected cone runs again. That includes time changes and exec
     invalidations.

### 1.2 Non-goals

* **Truly incremental Build/Compile** (stable slot IDs, spliced step regions, local edge patches). This is
  xlarge work. It would touch every slot-indexed array, the frozen copies and the frame-cache keys, because:
  * slot numbering is a dense namespace DFS with a `parent < index` refusal (bakedProgram.cpp:1913-1973);
  * override numbering is one running counter (bakedProgramImpl.h:2564-2616);
  * edges, levels, clusters and cones are global sweeps (bakedSchedule.cpp:839-1264).
* **An exec-authoritative product Dynamic path** (HP4 "PreferExec"). §8 explains why it was dropped.
* **Parallel as the default executor for CPU-only programs.** On the biped, parallel loses about 200 us per
  untraced frame, and it loses on every drag measured (audit §2.2).
* **Concurrent exec.** `ExecUsdSystem::PrepareRequest` is not reentrant across requests of one system (rule X5),
  and ChangeTime mutates the shared system.

### 1.3 The exec-authoritative parity oracle stays, explicitly

`_EvaluateDynamic` (rigEvaluator.cpp:11307-14299) is kept, with unchanged semantics, as a separate verification
path. It becomes the explicit mode `RigExecEvaluationMode::ExecReference` (item X0). Its roles:
* It stays the reference half of `BakedWithParityCheck` (rigEvaluator.cpp:10860-10875).
* It stays the `cpuParityMode` scalar oracle (13361-13390, 14156-14295).
* It stays the fallback for unplaceable overrides (10797-10834), for program bails other than `stageFrames`
  (10842-10853), and for refusals that no lowering covers yet.

Once HP-D-flip lands it is **not** the product path of Dynamic mode. The design needs the oracle because every
exec lowering is judged against it (§6, the §2.1 amendment).

---

## 2. Architecture

### 2.1 One program type

`RigExecBakedProgram` is the only executable form. It gains a lowering parameter:

```
struct RigExecLowering {
    enum FoldPolicy   { Throughput, Editable } fold;   // Editable: HP-D1
    enum ExecLowering { RefusalsOnly }        exec;    // PreferExec dropped (§8)
};
```

| evaluation mode | runs | lowering |
|---|---|---|
| Baked | program | Throughput + RefusalsOnly |
| Dynamic (after HP-D-flip) | program | Throughput, then Editable once HP-D1 lands; RefusalsOnly |
| BakedWithParityCheck | program, then ExecReference; compared by `RigExecComparePoses` | Throughput + RefusalsOnly |
| ExecReference (new, X0) | `_EvaluateDynamic` only | none |

**Rule P1.** Dynamic mode builds and runs a program for every rig that has one. `_EvaluateDynamic` is reached
only through:
* ExecReference;
* the oracle half of BakedWithParityCheck;
* `cpuParityMode`;
* a refusal with no lowering;
* an unplaceable override;
* a bail other than `stageFrames`.

**Rule P2.** A rig gets an exec lowering only for work that CPU steps cannot express. Four kinds of work need
exec at all:
1. Exec-seeded providers whose ladder the CPU compose cannot express.
2. Connected pose providers.
3. Solver types the CPU cannot solve.
4. Snapshot taps whose consumer has no slot.

Only kinds 1 and 2 are reachable today. For the other two:
* The baked solver set equals the registered set (bakedProgram.cpp:126-131 vs rigEvaluator.cpp:1081-1088).
* The baked weight set equals the registered set (bakedProgram.cpp:153-158 vs rigEvaluator.cpp:209-216).
* Every `RigExecRevisionOp` is accepted (moverGraph.h:59-76 vs bakedProgram.cpp:626-650).
* No guide refusal exists. [HP1-gain]

**Rule P3.** A refusal that is a CPU capture gap closes with a CPU step, never with an exec step. Constraints
and mover ops are CPU kernels in both paths (`_ConstraintHandlers`, `RigExecMoverGraph`). [map dynamic-to-steps]

### 2.2 Step kinds

The CPU kinds of the step-graph spec (bakedProgramImpl.h:572-591) are unchanged. An exec-backed kind is added
**only together with the item that lowers something to it** [EX2-gain], because `kStepCosts` asserts a row per
kind (bakedSchedule.cpp:193-215).

| kind | added by | class | reads | writes |
|---|---|---|---|---|
| `ExecSeed(group)` | EX2a | source; runs in the source pass (bakedSchedule.cpp:1898-1903) | BaseOverrides, time, the request's stage inputs | the first version of PoseBase/PoseFin/PosedM over `[p, p+subtree)` of eligible providers, and their rests into the prologue rest tables |
| `ExecPull(provider, site)` | EX2b / HP1' | interior | the request inputs only: PoseBase/PoseFin versions of `_poseProviderInputs[p]` live at the site, Committed flags, BaseOverrides | a private `ExecResult{raw, current}` |
| `ConnectedCommit(provider, site)` | EX2b / HP1' | interior, CPU | ExecResult and the carried versions | the descendant delta and the commit of `p` (upper bound `[p, p+subtree)`) |

Kinds that are **not** added: `ExecSolve`, `ExecGuide`, `ExecSnapshotSubset`, `ExecRest`. Their refusals are
unreachable, or they would reintroduce a barrier. [HP1-gain][HP1-deps]

New domains:
* `ExecLane`: order-only (EX2b).
* `Committed`: one per constraint (HP1').

**Rule K1 (no PoseRest domain).** Rests stay in the prologue-owned tables (bakedProgramImpl.h:1318-1340).
ExecSeed is a source that runs serially before dispatch, and it writes into those tables for its own ranges. A
rest slot domain would add a data edge from every compose to every rest reader in all 28 in-tree programs, and
that would change their clustering and cones. [EX2-deps][EX2-gain]

### 2.3 Sources and cones

**Rule C1.** Source steps run on every run, in program order, on the calling thread, and compare their outputs
by value. Sources are the prologue sources of step-graph spec §7, RevisionStatic, the weight-object readers and
ExecSeed. A source reads nothing that a non-source step writes (bakedSchedule.cpp:1893-1896).

**Rule C2 (step closure, SU1a).** The closure is computed per STEP: a forward bitset closure over `step.succs`,
in reverse program order. Program order is topological for steps (bakedSchedule.cpp:915-925). `B.closed` becomes
"the clusters that contain a closed step".

The per-cluster tables (`avarCluster` and its siblings, `cones.cone`, `cones.always`, `varyingSteps`,
`overrideSteps`) are **kept beside** the new per-step tables. Other code reads them: outputAffectedIndex.cpp:42-200,
frameCacheSparsity.cpp:269-306 and the frozen clone (frozenContext.cpp:3863-3866). [SU1-deps]

**Rule C3.** On the live path, both places that test `step.cluster` switch to the step bit:
* the serial executor (bakedSchedule.cpp:1626);
* the skip-marking loop in `RigExecBakedRunSteps` (1904-1918).

If only the first moved, a clean step inside a closed cluster would keep last run's `revisionsExecuted` counters
and geometry deltas (bakedProgramImpl.h:765-770, bakedGeometry.cpp:1965-2015). `_FrozenRunSteps` and
`RigExecRunPartialCone` may keep testing the cluster bit, which gives a superset. [SU1-deps]

**Rule C4.** The time predicate stays: `time != B.lastTime` dirties `varyingSteps` (bakedSchedule.cpp:1492-1500).
On a frozen worker it is the only signal that dirties varying steps, for two reasons:
* `_FrozenRunSteps` shares `RigExecBakedComputeClosure` (frozenContext.cpp:6359, 3652-3656).
* The frozen worker patches varying inputs into constants (3630-3636, 4374-4420).

Replacing the predicate with a per-input value compare (SU1b) is deferred. [SU1-deps][SU1-gain]

**Rule C5.** `phasedReads` still forces a full run (bakedSchedule.cpp:1286-1288). `RigExecChainSnapshots`
appends positionally (moverGraph.cpp:1333-1347). Retaining its records across runs would change the answers of
`Preceding` and `AtPrim`. It would also open holes on the frozen path, which emit "resolved to nothing"
diagnostics (frozenContext.cpp:6072-6080). [SU1-deps]

**Rule C6.** Order-only edges (EX2b) live in a separate list. They feed only the executor's counters. They are
never used by:
* cones (bakedSchedule.cpp:1253-1262);
* the output-affected index (outputAffectedIndex.cpp:50-67);
* RAW/WAW derivation;
* the verifier's data check.

If order edges fed the cones, one dirty exec step would re-pull the whole exec spine. [map sparse-update][EX3-gain]

### 2.4 Executor contracts

| executor | where | contract |
|---|---|---|
| serial (reference, default) | bakedSchedule.cpp:1607-1674 | Program order on the calling thread. Legal for exec steps once `Evaluate` releases the GIL for its whole duration (F0b) and the prologue has run `PrepareRequest` for every request (rule X3). |
| parallel (opt-in, `RIGEXEC_BAKED_SCHEDULE=parallel`) | 1681-1873 | Counters count only closed predecessors (1836-1844), so a dispatched set must be path-convex over the counter edges (rule E1). **At most one Exec\* step per parallel frame** until EX3' serializes the dispatched exec steps (rule E2). |
| frozen serial (background, frame cache) | frozenContext.cpp:6338-6384 | Never runs an Exec\* step, sources included (rule F1). |
| sparse partial cone | frameCacheSparsity.cpp:283-310 | Iterates a stored topological cluster order (F0c). Refuses a plan that contains an exec cluster. |

**Rule E1.** The parallel executor counts predecessors on the unreduced cluster edges. The only exception is a
frame whose dispatched set is a union of cones, which is convex. Under step closure, the set of clusters that
contain a closed step is not convex in general.

Example: step edges a2→b, b→c and a1→c. A frame dirties only a1, so dispatch is {A, C}. If the edge A→C had been
removed as transitively redundant, C would be seeded beside A and race it. [EX3-deps]

**Rule E2.** No two exec steps run at once.
* Within one program, the serial executor guarantees this by construction.
* In the parallel executor, each dispatched Exec\* step must wait for the previous dispatched Exec\* step in
  program order. That wait comes from a per-frame counter edge, or from pass-through dispatch of the Exec\*
  steps in between.
* The static ExecLane chain alone is not enough. E0 and E2 can be closed while E1 is not, because
  bakedSchedule.cpp:839-945 links each write only to the last writer.
* Until EX3' lands, a frame that closes more than one Exec\* step runs serially. [EX3-deps]

**Rule E3.** Exec steps are singleton clusters. They are excluded from level packing (bakedSchedule.cpp:692-744),
fusion (746-771) and absorb (773-797). Closure is per cluster on the frozen and partial-cone paths, so a
co-clustered exec step would re-pull whenever a CPU sibling was dirty. [map executor-constraints]

**Rule F1.** Background and frozen workers never call exec, for two reasons:
* Exec drives the live system (purity rows, frozenContext.cpp:3575-3578).
* Exec's internal TBB parallelism escapes `RigExecFrozenSerialActive` (parallel.h:23-29).

Consequences:
* Any program that contains an Exec\* step, including one whose only exec step is a source ExecSeed, takes the
  `kRigExecFrozenBakeRefused` decline (frozenContext.cpp:2042, 2125) and the D7 UI-thread memo.
* bridge.cpp:1878-1886 must change. Today a rig that has a program but cannot sample stays live-only.
* `_FrozenStepBody` (frozenContext.cpp:6306-6330) must decline exec kinds explicitly. Today an unknown kind falls
  through to the pose body. [EX2-deps][EX2-gain]

### 2.5 The exec lane rule, per stage

**Rule X1 (one lane per stage, owned by a scope).** Each `RigExecTapContext` owns an **exec lane token**. There
is one context per stage, shared by every evaluator (tapSet.cpp:65-130).
* The calling thread acquires the token only after it has released the GIL.
* The token is re-entrant within the owning scope. When Compile or `_RealizeDeferredExecPrep` runs inside an
  owned `Evaluate`, it verifies ownership instead of re-locking.
* Ownership is not checked by thread id. Worker-side exec that runs under the owner's scope is covered by the
  owner's token: Compile's warmup lane (rigEvaluator.cpp:4079) and the overlapped snapshot (12309-12319).

The token is taken around **every** exec site:
* Compile's whole dispatch-to-join span;
* `_RefreshEpochRestFrames` (10057), in every mode;
* `_RealizeDeferredExecPrep` (11279-11302);
* the `_EvaluateDynamic` body;
* the prologue of any program that contains Exec\* steps.

A lease taken only by programs that have exec steps does not serialize against a baked sibling's rest refresh,
and that refresh moves the shared system to `_restTime`. [EX1-deps][EX1-gain]

**Rule X2 (GIL before token).** `TF_PY_ALLOW_THREADS_IN_SCOPE()` runs at function scope of
`RigExecRigEvaluator::Evaluate` (rigEvaluator.cpp:10750), and of Compile (already there at 3593), before the
token is acquired. Otherwise this deadlock is possible:
1. Thread A holds the GIL and blocks on the token.
2. Thread B holds the token and needs the GIL to load plugins through TfScriptModuleLoader, which happens on the
   first `PrepareRequest` in a process (rigEvaluator.cpp:11234-11243; exec/definitionRegistry.cpp:462-478).
[EX1-deps][F0-deps]

**Rule X3 (no compilation inside a step body).**
* `ExecUsdSystem::Compute` and `ComputeWithOverrides` always call `requestImpl.Compile()` and `Schedule()`
  (OpenUSD execUsd/system.cpp:112-117, 132-135).
* They recompile whenever there is a pending system-wide recompilation (exec/requestImpl.cpp:526-530,
  exec/system.cpp:219-226).
* `PrepareStageChange` resets requests only when a prim is removed (tapSet.cpp:101-110), so a request can be
  valid and still stale.

The rule: the prologue calls `system->PrepareRequest` on **every** exec request the frame can touch, not only the
invalid ones. On a clean request this is a no-op (execUsd/requestImpl.cpp:132). The prologue then calls
`ChangeTime(time)` once. A ChangeTime to the same time returns early (exec/system.cpp:48-54), so this is
byte-safe and nearly free: 28 calls measured 0.023 ms in total over 3 dynamic frames.
[EX1-deps][EX1-gain][HP4-deps]

**Rule X4 (isolated waits while the token is held).** A thread that holds the token and waits must not steal
another evaluator's task that needs the same token. Two ways to satisfy this:
* Waits made under the token use `WorkIsolatingDispatcher`.
* Or the whole dispatch-to-join span is wrapped in one `WorkWithScopedParallelism`, so `Run`, `Wait` and the
  dispatcher destructors share one isolate.

This applies to:
* Compile's dispatchers (rigEvaluator.cpp:3680, 4062, 4063, and bakedProgram.cpp:1790);
* the snapshot dispatcher (12307, waited at 13120);
* the region's `Wait`, which is already isolated at bakedSchedule.cpp:1850.

Wrapping only the `Wait` calls is illegal. Tasks spawned outside the isolate carry no tag, so a waiter inside the
isolate cannot run its own lane task, and a run with `PXR_WORK_THREAD_LIMIT=1` hangs (python/_rigexec.cpp:596;
withScopedParallelism_impl.h:26). [F0-deps]

**Rule X5 (no detach, no concurrent prepare).**
* Nothing detaches an exec lane past the end of Compile. `PrepareStageChange` resets requests synchronously on
  the editing thread, which would be a use-after-free for a detached lane [audit S8].
* Distinct requests of one system never prepare or compute concurrently. Each prepare opens a program-global
  compilation round (exec/compiler.cpp:40-139, program.h:103-109), and program.h marks its node-deletion entry
  points "not thread-safe" (258/268/285).
* Exec already parallelizes inside one request (compiler.cpp:55-103). [map compile-floor]

**Rule X6.** Exec callbacks never call into exec (execUsd/system.h:99-102). rigExec's invalidation callbacks only
set atomics (tapSet.cpp:219-231). Exec value invalidation is **never** used as a "clean" signal:
* it is deduplicated until a plain `_Compute` (exec/requestImpl.cpp:488-521);
* `ComputeWithOverrides` never renews it;
* override-only requests do not re-arm it (rigEvaluator.h:965-966).

### 2.6 The parity oracle

**Rule O1.** `RigExecComparePoses` is the only judge (bakedProgram.cpp:986-1000). It checks exact equality,
including the ordered diagnostics, and excludes `solverEvaluations`.

**Rule O2.** The reference for a CPU step is serial program order (step-graph spec §2.4). The reference for an
exec lowering is ExecReference.

**Rule O3.** The oracle keeps its own caches: `batch.cache`, `_firstFramePoseCache`, `_connectedPoseCache`,
`_authSnapTimeKeyed` and `_guide*` (rigEvaluator.h:798-801, 943-1044). It also keeps its own `ConsumeDirty`
draining (rigEvaluator.cpp:11480, 12311, 12380, 12422, 12466, 12528, 13127, 13292, 13319). A program exec step
never shares a consumable flag with the oracle; its time accumulator is owned by the program (rule S8).
[SU2-deps][HP4-deps]

**Rule O4.** Once compile no longer prepares the oracle's exec state (CF1, HP-D0), that state is prepared lazily
in `_RealizeDeferredExecPrep` (rigEvaluator.cpp:11232-11305).

---

## 3. Edit tiers

Baselines are measured in baked mode on the biped: medians of 5-10 evaluates right after the edit, profiler on.
Where the dynamic baseline differs, it is also given.

| tier | edit | today | target | items |
|---|---|---:|---:|---|
| T0 | leaf-finger override drag | 1.16-1.68 ms (72/76 clusters); dynamic 5.9 | 0.8-1.0 ms (15/634 steps; 0.73-0.86 measured at grain 0) | SU1a; dynamic via HP-D0 |
| T0 | hips drag | 1.1-1.4 ms (403/634 steps) | unchanged (no gain measured at grain 0) | none |
| T1a | avar default value | 1.46-1.62 ms (Patched) | ~0.8-1.2 ms | SU1a |
| T1b | avar timeSample on a keyed avar | 1.46 ms, but 76/76 clusters (DryRun declines, StampBumped) | cone-sized | EP3a |
| T1c | non-avar value edit in the rig (`guide:scaleX`, mover `inputs:defaultWeight`) | 32.4-33.0 ms (see breakdown below) | ~28 (EP1a), then ~3.6 (EP1c), then ~0.6-1.2 ms (EP3a+b) | EP1a, EP1c, EP3a, EP3b |
| T1d | value edit outside the rig (`/Biped/Materials/cornea_mat`) | 32.4-33.0 ms; dynamic 38.3 | settle ≈ 0 plus the cache-drop rebuild (5-15 ms *est.*), then < 1 ms after EP3b | EP1b, EP3a, EP3b |
| T2 | captured rest value (`rest:rx` on `hips_ctl`) | 54-60 ms, median 57 (settle 31.3 + Build 20.5 + first run 5.2); evicts every cached frame | ~30-37 (EP4a alone), then ~2-5 ms (with EP1c + EP3a); a rest edit on the root joint costs about a full frame | EP4a (+EP1c, EP3a) |
| T3 | folded value (`parent:space`, `jointElements`) | ~57 ms | ~25 ms (rebuild without the digest), then ~20-22 ms (EP5') | EP1c, EP5' |
| T4 | structural (add a `RigExecControl`) | 82.7-91.4 ms, median 87.6 (inline digest ~29 + warm Compile 53.75, of which Bake 20.7 and PoseInfoPrefetch 7.6) | ~58-63 ms (EP2a), then ~50-55 ms (CF3a, EP5') | EP2a, CF3a, EP5' |
| T5 | stage left uncompilable (bad `rigExec:joints` target) | 54-68 ms on **every** frame | ~0.3 ms per frame | EP2b |

T1c breakdown today: settle 29.7-30.0 ms (digest 26.2 + `_EpochRestsMightVary` ~3.3, by subtraction + rest
re-pull 0.34-0.39) plus a full run of 3.6-3.9 ms.

Dynamic mode today (edits2.py, profiled): steady 2.22 ms, time step 6.44, leaf drag 5.90, avar edit 5.89,
unrelated edit 38.3, compile 115.8. After HP-D0 these become the baked numbers above:
* untraced animated frame: ~4.1 → ~1.2 ms (audit §7: baked Run 21.2 ms over 16 frames);
* compile: 96.2 ms → the baked floor.

### 3.1 Tier rules

**Rule T-a (the digest recomputes in full, but only for a suspect notice).** The digest still never trusts
incremental invalidation (rigEvaluator.cpp:2405-2412). What changes is whether it runs at all:
`_structureDirty` (1820-1823) is set only for a notice that could change something the digest read. §4.1 defines
"suspect". `RIGEXEC_VERIFY_DIGEST_GATE` recomputes the digest on every notice the gate called neutral, and fails
if the digest moved.

**Rule T-b (edits route like overrides).** A value edit on a live `RigExecBakedInput` behaves like an override
that is placed for one run and then lifted. While the edited path is indexed, it never bumps the program stamp
(§4.2).

**Rule T-c (captured constants are patched, never promoted).** When a captured input moves (bound, length-1
walk), its `.constant` is patched in place and its consumers are marked edited for one run. It is never flipped
to varying, because flipping would:
* set `ladderVarying`, and `RigExecFreeze` refuses a `ladderVarying` program (frozenContext.cpp:4003-4006), so
  background warming would be lost;
* make every later frame recompose (bakedPose.cpp:2140-2144).

A spline edit or a sample-key edit still takes the Stale path. [EP4-deps][EP4-gain]

**Rule T-d (folded values rebuild).** A folded value shapes Build: `RecordFold` records no binding
(bakedProgramImpl.h:2724-2735), and space expressions replace the ladder (bakedProgram.cpp:2082-2091). Only a
Build moves a folded value: either a rebuild, or an Editable-policy bind under HP-D1. [EP4-deps]

**Rule T-e (structural edits compile once).**
* A notice that is **certainly** structural skips the inline digest.
* A merely suspect notice computes the digest once, inline. Compile then computes it again on its worker, off
  the critical path (DigestJoin waits 0; audit §5 S5).
* Reusing the settle digest inside Compile would save worker CPU, not wall time, so it is deferred (EP2c).

**Rule T-f (a broken state is memoized).** A failed compile is keyed by
`(_stageEditSerial, _PeekEvaluationMode(), _evaluationModeSource)`, not by the serial alone. The reasons:
* `deferExecPrep = _PeekEvaluationMode() == Baked` (rigEvaluator.cpp:3701-3702) decides whether the dynamic-only
  prepares can fail (7485, 7514-7518, 7898-7918).
* `SetEvaluationMode` (11048-11078) changes the mode without bumping the serial.
[EP2-deps][EP2-gain]

---

## 4. Sparse update rules

### 4.1 Settle path: which notices reach the digest

`_OnObjectsChanged` (rigEvaluator.cpp:1795-1995) runs for every stage notice. It is gated in three layers.

**Rest gate (EP1a).**
* At commit, build `restPaths = {p.AppendProperty(n) : p in _restTapIds, n in _RestInputNames()}`. This set is
  complete because:
  * `seedProvider` seeds every provider ancestor (7270-7280);
  * computeRestFrame reads exactly those seven names plus the namespace ancestor (computations.cpp:442-455).
* `_EpochRestsMightVary` (10036-10047) and its recompile (11209-11218) run only for one of:
  * a resync or changed-info on a path in `restPaths`;
  * a resync at or above a provider;
  * a resync of `/`.
* The same hit sets `_epochRestFramesStale`, and nothing else sets it. It cannot be keyed on `_structureDirty`:
  the digest never reads `rest:*` values (2461-2519), so a `rest:rx` value edit is not digest-suspect, the stale
  flag would never be set, and the oracle would read stale rests at 11578. [EP1-deps]
* The flag is consumed at the head of `_EvaluateDynamic`, after `_SettleEpoch` and `_RealizeDeferredExecPrep`
  (11314-11324). A failed re-pull emits "rest frame evaluation incomplete" there and leaves the flag set. When no
  program exists, the re-pull stays inline where it is today (11222).
* This is a documented behaviour change: a baked evaluate no longer fails on a failed re-pull (10765-10772).
  `_epochRestFrames` has exactly one value reader, the dynamic walk (11578). Line 10042 reads only its keys, and
  the program reads `B.restFrames` (bakedPose.cpp:2068).

**Prim-granular digest gate (EP1b).** The digest records the prims it read outside `UsdPrimRange(rig)`:
* relationship and connection targets, including missing ones;
* mover target gprims;
* weight objects;
* ancestors it walked (2660-2690).

A notice is **suspect** if any of these hold:
* it is a resolved-asset resync;
* it has a path at or under `rigPath`;
* it resyncs a prim at or above `rigPath`;
* it resyncs or changes info on, under or above a recorded outside prim;
* it has any `/__Prototype` path (the digest follows instance proxies).

Every other notice is neutral.

**Attribute-granular gate (EP1c).** Inside the rig, the digest records four sets:
* `probedPaths`: every attribute or relationship it looked up, **including lookups that failed**. Examples:
  `GetAttributeAtPath` on `owner.normals/extent/widths` and on `points` (2877-2921), missing connection sources
  (2107-2110), target prims (2125-2130).
* `valueReads`. The complete list:
  * appendToken 2056-2069;
  * appendScalar 2073-2082;
  * appendAttributeBinding 2083-2114;
  * weight objects 2181/2192/2266, with `_BakeFalloffLut` 476-520;
  * jointElements 2754-2764;
  * cardinality 2784-2834;
  * mover points and authoredness 2877-2921;
  * activation 3031;
  * `readPhase` metadata 2047-2052.
* `sampleCountReads`.
* `listedPrims`, with the attribute names and types listed for each (hopFor's `GetAttributes`, 2509).

A notice is **suspect** if any of these hold:
* It is a PROPERTY resync on a probed path. A property's first spec in a layer arrives as a property resync, not
  as a changed-info default (comment at 1812-1818).
* It is a property resync on a listed prim. **Exception**: the resync is neutral when all of these hold:
  * the name was in the recorded listing;
  * the composed attribute still exists with the same type;
  * the changed fields are a subset of {typeName, default, timeSamples, spline};
  * the path is in neither read set.

  The exception mirrors the typeName-only exception in `_NoticeIsAvarValuesOnly`. Without it, every gizmo release
  and every undo would pay the digest. [EP1-gain]
* It changes an info field outside {default, timeSamples, spline}.
* It changes a value field on a `valueReads` path.
* It changes timeSamples on a `sampleCountReads` path.

A prim changed-info notice is neutral. The digest reads no prim metadata, and USD reports active, kind,
specifier, typeName and apiSchemas changes as resyncs (stage.cpp:4582-4614).

**Rule S1.** Every digest records the footprint, whether it runs at settle or at compile. If recording costs more
than 1 ms on the compile lane after CF1, the footprint is recorded lazily at the first settle after the compile.
Recording is never allowed to lengthen the cold critical path without being measured. [EP1-gain]

### 4.2 Program path: which steps re-run

**Rule S2 (routing, EP3a).** A changed-info path whose changed fields all lie in {default, timeSamples, spline}
is mapped through `B.overridableInputs` (bakedProgramImpl.h:2685-2701). The mapping sets a pending `edited` bit
for each override index. The bits OR together across notices and are consumed only in
`RigExecBakedComputeClosure`'s override loop (bakedSchedule.cpp:1504-1512), next to `lastOverridden`.

Paths under a source that is already compared by value need nothing more:
* avars (1368-1378);
* xformBase (1385-1389);
* native frames (1437-1444);
* delta bases;
* constraint arrays (1402-1423);
* ribbon points (bakedPose.cpp:2201-2207);
* chain base and revision static (bakedGeometry.cpp:1925, 2269).

Weight-object constraints and Derived steps are already in `cones.always` (bakedSchedule.cpp:1121-1144).

**Rule S3 (fallbacks fail closed).**
* `BumpProgramStamp` applies to three cases:
  * a `named`/`prims` path that is neither overridable nor under a value-compared source;
  * pseudo-root or layer-metadata changed-info;
  * every resync, until EP3b is verified.
* Stale applies to a `connectionPaths` or `targetPaths` field, or any other non-value field, on a `named` path.
  Without this, a retargeted `resolvedAttr` walk would leave edits on its new upstream hops unindexed
  (bakedProgramImpl.h:315-318). [EP3-deps]

**Rule S4 (frozen clones).** `_CloneImpl` copies the `edited` bits along with `programStamp` and
`lastProgramStamp` (frozenContext.cpp:3873-3874), and `RigExecPatchFrozenAvarConstants` carries them. Otherwise,
when the clone is re-frozen at its `lastTime` after a dynamic-fallback frame, the edited cone is skipped.
[EP3-deps]

**Rule S5 (scoped cache clears, EP3b).**
* `_staticInputs` (moverGraph.h:150-222):
  * exact erase on changed-info;
  * prefix erase on resync;
  * full `Clear()` on resolved-asset resyncs and pseudo-root changes.

  A scoped erase re-stamps `_owner` exactly as `Clear()` does (moverGraph.h:186-192).
* `_propertyChainBindings`: a chain is dropped when a notice path hits, in **any** field, its watch set, its
  target, its mover prim or its weight-object relationship. The clear is never scoped to connection fields alone.
  `_BindInput` captures unconnected values once (rigEvaluator.cpp:9521-9545, 9448-9455), so a default edit on
  `inputs:value` would otherwise replay a stale `lastValue` (bakedSchedule.cpp:1292-1294). [EP3-deps]
* Skin topologies and `_skinLayoutInputsValid` are dropped on any of:
  * membership in `_skinLayoutInputs` (rigEvaluator.cpp:10231-10295);
  * a resync prefix;
  * any connection field of a member.

  They are never dropped just because "the notice names a mesh". Layout attributes live on the mover prim and are
  reached through connections, for example `inputs:method` (testRigExecSkinTopology.cpp:566-576).
* Blend sample shapes are dropped by sample prim and by blendShape target prefix.
* `basePointsPushed` resets when the notice names that target's points.

**Rule S6 (verification of scoped clears).** The cone verifier captures state after the prologue
(bakedProgram.cpp:3005-3030), so it cannot see a stale prologue cache. EP3b is verified instead by a debug env:
it runs a shadow evaluation with the wholesale clears and byte-compares `--pose-out`. [EP3-gain]

**Rule S7 (frame cache).**
* Full-eval provenance lists every cluster (frameCacheSparsity.cpp:313-322). An edit that reaches the program
  therefore retires every full-eval entry, and that is correct.
* EP3a's retirement gain applies only to paths the program never reads. Today those count as foreign controls
  and retire everything (outputAffectedIndex.cpp:302-306).
* To deliver that gain:
  * the overridable-input paths go into the index's control universe (outputAffectedIndex.cpp:112-201, seeds
    from `RigExecOverrideSeeds`, 417-480);
  * `ClassifyNoticeDisposition` returns a new `Edited` disposition (rigEvaluator.cpp:1770-1793). It stays const,
    because it is called from 1878 and from registry.cpp:2600.
[EP3-gain][EP3-deps]

### 4.3 Exec steps: time, edits, invalidation

An exec step is **not** a pure function of its declared slots, because exec reads stage values and time itself.

**Rule S8 (time accumulator, SU2a).** Each exec request of a program owns a sticky atomic time accumulator,
separate from `_dirty` (tapSet.h:208), which the oracle drains.
* The time callback sets it, per member for leader/follower requests, no matter who called ChangeTime: the
  oracle, `Warm`, guides, or another evaluator.
* Only the owning step's successful pull clears it.
* Time callbacks run on a `WorkDispatcher` task inside ChangeTime (exec/system.cpp:63-79). The calling thread
  harvests the accumulators after ChangeTime returns.
* The step also records `lastRunTime` and a request generation. It is dirty when:
  * its request was (re)prepared or replaced;
  * the program is on its first run;
  * `lastRunTime != time` and the generation changed.

This closes a gap: the prologue's ChangeTime is a no-op when the oracle has already moved the system to this time
(exec/system.cpp:48-54). That happens because `runBaked` is false for unplaceable overrides
(rigEvaluator.cpp:10810-10832) while the program is kept. [SU2-deps]

**Rule S9 (edits are coarse until an index is proven).** Any of the following dirties every Exec\* step:
* a notice under the rig's paths, including an avar-only Patched notice (under the program, avars reach ExecSeed
  through the request, not through slots; rigEvaluator.cpp:1879-1890, 1939-1948);
* a change to BaseOverrides;
* a move of the program stamp.

A generalized exec-input index (SU2b) is deferred. It would have to be built lazily, like S4's deferred build
(11262-11275), so that no baked compile pays 7-8 ms for it. [SU2-gain]

**Rule S10 (body guard).** An exec step carries its SSA output when all three hold:
* its full input vector equals the last run's (all of BaseOverrides, in order, duplicates included, plus the
  tail);
* no dirty bit is set;
* a stored output exists.

Otherwise it pulls. This follows the `sameOverrides` pattern (rigEvaluator.cpp:12055-12061). The verifier's
forced pass bypasses the guard. An unchanged pull is not free: about 55 us fixed, 86 us median (audit §2.3).

**Rule S11 (exec always receives the full BaseOverrides).** Exec pulls carry the whole BaseOverrides vector, as
today (rigEvaluator.cpp:11488, 12095-12096, 12398-12400). `solverPoseReads` (6868-6872, 7402-7414) misses the
solver prim's own attributes, its relationship targets and its connection closures (1350-1357, 1418-1440). If a
filtered vector went to exec, the forced verifier pass would reproduce the same wrong value and not catch it. A
drag therefore re-pulls every exec step whose inputs it reaches. [SU2-deps]

**Rule S12 (failure).** A failed exec step:
* sets its own dirty bit;
* publishes the dynamic diagnostic;
* makes `Run` return false, matching the oracle's early return (12402-12406).

A persistent failure is memoized per epoch and cause, so it does not rebuild the program every frame (rule D3).

**Rule S13 (frame-cache key).** For a program that contains a time-dependent Exec\* step, the D1 key folds in
time and the stage-edit serial (the D7 rule, bridge.cpp:1875-1890), and the step registers in `Varying`.
Otherwise two times with equal sampled controls but different exec-read values would share a key.

### 4.4 Dispatch rules for Dynamic (HP-D)

**Rule D1.** "Dynamic" comes from three sources, and each is mapped explicitly before the switch:
* the default (bakedProgram.h:73-76, rigEvaluator.h:1645);
* an authored `rigExec:baked=false` (rigEvaluator.cpp:10997-11013);
* `RIGEXEC_EVALUATION_MODE=dynamic` (1505-1545).

`RIGEXEC_EVALUATION_MODE=reference` is added. Whether an authored `false` means ExecReference is a product
question (§9).

**Rule D2.** Every Dynamic gate moves together:
* the lazy build (10792);
* `runBaked` (10812);
* `_RebuildBakedProgram`'s early return (11125-11129);
* the `deferExecPrep` predicate (3701), which becomes `!= BakedWithParityCheck && != ExecReference`;
* the baked-flag notice drop (1921-1927);
* `SetEvaluationMode` (11048 onward);
* the three announcers (10883, 10908, 10933), which stay no-ops for Dynamic.

**Rule D3 (bails).**
* Only the `stageFrames` bail (bakedProgram.cpp:3001-3007) publishes the invalid pose that the oracle returns at
  the same point (rigEvaluator.cpp:11616-11623).
* `NoCandidate` (bakedPose.cpp:2249-2255, 2412-2418) and the unusable-frame publish bail (3671-3674) keep the
  oracle re-run. At those points the dynamic walk returns a valid pose: it keeps unpublished joints on their rest
  chain (rigEvaluator.cpp:11452-11455) and re-picks the closest published candidate (11878-11916).
* A bail is memoized per epoch and cause, the way `_bakeRefused` is (10792-10795). [HP-D-deps]

**Rule D4.** `solverEvaluations` means different things on the two paths. In the program it is a Build-time
constant (bakedPose.cpp:1367). In the oracle it counts re-evaluated batches (rigEvaluator.cpp:12427, 12516).
Tests that assert the oracle meaning (testRigExecConstraints.cpp:2709-2724, 2833-2855) are pinned to
ExecReference.

---

## 5. Compile floor plan

Today's cold baked critical path, from 7 runs on the main build:

DerivedStartFrames 6.75 ms → Validate 2.7 → exec lane (warmup 49.1 + guides 5.0, ending at ~64.7 ms; main has
been waiting since 48.2) → Commit → Bake 23.6 → end at ~92 ms.

The digest lane (OutputSets 1.2 + Solvers 63.8 + Movers 5.5) ends at ~77 ms, so it has about 15 ms of slack.

| step | item | cold baked Compile (median) | what becomes critical |
|---|---|---:|---|
| today (busy box) | none | 87.5-92.0 ms (80.0 on a quiet box, audit §7) | exec lane (warmup) |
| 1 | CF1 warmup diet | 68.5 (measured, 6 runs; `--pose-out` byte-identical with `--guides`) | digest lane (DigestJoin waits 3.5-8 ms) |
| 2 | CF2a three-task digest | 62.4 (measured, 6 runs; concat+hash 0.49 ms) | co-critical: Bake ends at 60.35, Digest.Solvers at 60.59 |
| 3 | CF2b Solvers read memo | ~61 (*est.*; cold Solvers 55-67 → ~35-42, but the main thread ends at the same time) | main thread |
| 4 | CF3a/b/c | ~55-60 (*est.*, −3..8 ms; not below the end of the digest lane unless CF2b pays off) | digest lane / Bake |

The realistic cold floor is **~55 ms** (*est.*). The map's earlier "50-55" is not supported until CF2b and CF3
have both been measured. [CF2-gain][CF3-gain]

Edit-path compile work:
* Settle digest: 25-33 ms, then ~21-29 after CF2a (−3.5..4), then ~15-19 after CF2b (*est.*).
* Warm structural recompile: bound by the main thread (DiscoverValidate 11.1, SolverSchedule 16.8 including
  PoseInfoPrefetch 10.6, Bake 23.4). CF3a saves 5-8 ms there. CF1 saves little wall time on a warm recompile,
  because the warm warmup (13.1-19.4 ms) is not critical.

Rules:

**Rule CF-a.** CF1 applies only to epochs with `deferExecPrep` set. Until HP-D0 changes the predicate, that
means Baked mode only: the biped stages that author `rigExec:baked` (Biped.usda:12, Biped_layered_center.usda:19,
inherited by 6 more) and any run with `--mode baked`. Stages that default to Dynamic see no change until then.
[CF1-gain]

**Rule CF-b.** The rest pull is **not** chained into the lane task at dispatch. `newRestTaps` is filled on the
main thread at 7880-7891, after the lane dispatch. The pull stays dispatched from main (8019-8022) and joined at
8536. [CF1-deps]

**Rule CF-c.** Each digest segment gets its own string, its own `digestVisiting` set and its own
`digestRegionStart`.
* The Solvers segment is never split across tasks: the cycle marker (2593) makes its tokens depend on walk order.
* Byte identity is proven: 487215 = 50384 + 85158 + 351673.
* Segment boundaries: OutputSets 2308-2383, Solvers 2384-2841, Movers 2842-3041. Splitting at 2366 would move
  bytes from one segment to another. [CF2-gain]

**Rule CF-d.** The memo-backed connection-input collector is a copy local to the digest.
`_CollectAttributeConnectionInputs` (1117) is also used by the solver cache (1462), and it stays unchanged.
[CF2-deps]

**Rule CF-e.** A speculative bake never moves `newGuideTaps` while the lane may be reading it.
* The lambda captures a raw `RigExecTapSet*`, not `&newGuideTaps` (4040-4046).
* Only `_solverArrayTaps` is committed before Bake. `_guideTaps` and the `guidesPrepared` read stay after the
  join (8084-8088).
* If the guides fail to prepare:
  * the outgoing program is held, not retired, so that the re-bake can adopt geometry state (11145-11163);
  * the build counters that tests read (testRigExecBakedAttribute.cpp:202) either count one build, or the tests
    pin the double count.
[CF3-deps]

**Rule CF-f.** Any work added to Compile (the EP1b/c footprint recorder, the EP5' per-entity memo, anything else)
is measured on 7 or more cold runs against the milestone's baseline before it lands.

---

## 6. Amendments to `baked-step-graph.md`

These amendments land with EX2a, each next to the test that exercises it. None of them changes anything for a
program without Exec\* steps.

| § | today | amendment | why |
|---|---|---|---|
| §2.1 | The dynamic path is the parity reference. | For CPU steps, serial program order stays the reference. For an exec lowering, the reference is `ExecReference` (`_EvaluateDynamic`), judged by `RigExecComparePoses`. | Exec values are exec's own; only the oracle can judge a lowering. |
| §2.2 | No lock is taken in a step body. | **rigExec code** takes no lock in a step body. An Exec\* step may enter `ExecUsdSystem`, whose internal synchronization (the request tracker mutex, exec/system.h:88-93, and TBB) belongs to exec. It may do so only while the calling scope holds the stage's exec lane token (rule X1). Within a program, the order-only chain plus rule E2 guarantee that no two Exec\* steps overlap. | Exec steps take exec's internal locks, so forbidding those locks would forbid exec steps. The token itself lives in the prologue, which the spec already allows to lock (bakedProgram.cpp:2950-2953). [F0-deps][EX1-deps] |
| §2.4 | Serial is byte-identical to today's Run. | Unchanged. Added: before the region, a program's prologue runs `PrepareRequest` on every exec request and calls `ChangeTime` once (rule X3). | No compilation and no plugin load inside a region. [EX1-deps] |
| §2.6 | No `TF_WARN`/`TF_ERROR` from a step body. | Build validates the value type of every exec override (execUsd/system.h ~141-145), so exec cannot post a coding error from a worker. Any TF error an Exec\* step still raises is caught under a `TfErrorMark` into step-owned storage. The epilogue re-posts it from the calling thread, in step order, never into `pose.diagnostics`. Rig diagnostics the step reproduces (for example "connected ... pose input incomplete") stay per-step pose diagnostics. | `WorkDispatcher` hands task errors over at `Wait` (work/dispatcher.h:106-170), and the inline seed cluster is not trapped, so the order would differ between executors. The oracle leaves exec errors posted, so parity needs them re-posted, not swallowed. [EX1-deps] |
| §4 (no USD in a step) | Steps touch no USD. | **rigExec code** in a step reads USD only through the pinned queries it already uses (`RigExecBakedRead`, bakedProgramImpl.h:312-340). An Exec\* step reads the stage **through exec**: override resolution goes through Esf (exec/system.cpp:118-141) and attribute input nodes (attributeInputNode.cpp:75). That happens only inside `Evaluate`, on the scope that holds the token, where no edit or notice can run, and after rule X3's prologue. Override keys are pre-resolved through a per-(prim, attribute or computation) memo; the evaluator clears an entry on any resync covering its path. This replaces the per-call `GetPrimAtPath` at tapSet.cpp:301-320. | In practice "no USD" already meant "no concurrent edit". Exec's own reads cannot be removed. [HP4-deps][EX1-deps] |
| §5.2 | serial / parallel / frozen | A program with any Exec\* step never runs frozen or in the background (rule F1). The parallel executor serializes dispatched Exec\* steps (rule E2) and makes them singleton clusters (rule E3). | Exec drives the live system, and its TBB work escapes the frozen-serial gate. |
| §7 | Sources always run; non-sources are pure functions of their declared reads. | Exec steps are a third category. A step is dirty when any of these apply: it is in the cone; its program-owned time accumulator is set; its request generation changed; the coarse edit rule fires; the program stamp moved (rules S8-S9). Exec invalidation callbacks never mark a step clean. Order-only edges are excluded from cones (rule C6). Closure is per step (rule C2). | A pure-function model is unsound for exec. [map sparse-update] |
| §9 | Code layout. | Add `ExecSeed`, `ExecPull` and `ConnectedCommit`; the `ExecLane` (order-only) and `Committed` domains; the `RigExecTapSet::EvaluatePrepared` API; the exec lane token on `RigExecTapContext`. | New surface. |

---

## 7. Sequenced implementation plan

Each item below is either a survivor as written or the corrected form of a refuted proposal. Proposals refuted
outright appear only in §8.

Every milestone ends in a **parity gate**. No milestone starts until the previous gate is green.

**G-parity** passes when all of these are green:
* the build, `cmd //c "bin\build_rigexec.bat --no-test"`;
* ctest (192/192 plus the milestone's new tests);
* `tests/python`;
* `testUsdviewRigExec.py` at the memory baseline (curvenet and touchpose fail at HEAD; the three biped runners
  skip);
* `--pose-out` on all 28 RigExecRoot stages, frames 1..24, byte-identical to the milestone's starting build in
  each of these runs: baked, dynamic, reference (after X0), parity, `--guides`, and
  `RIGEXEC_ENABLE_PARALLEL_EVAL=0`;
* `BakedWithParityCheck` with 0 mismatches;
* `RIGEXEC_BAKED_VERIFY_CONES=1` under `RIGEXEC_BAKED_SCHEDULE=serial` and under `=parallel`.

**G-perf**: the milestone's bench numbers (compile: at least 7 cold runs; edits: F0a) are recorded in the audit's
§8 before the next milestone starts.

Effort: S ≤ 1 day, M 2-5 days, L 1-3 weeks, XL > 3 weeks.

### M0: Foundations (no behaviour change)

| id | change | files | gain | risk | effort | deps | verification |
|---|---|---|---|---|---|---|---|
| F0a | Edit-latency bench `tests/benchEditLatency.cpp`, a sibling of `benchCommitLag.cpp` that reuses its edit + SetTime harness (benchCommitLag.cpp:1-20). Scenarios: leaf and hips drag, avar default and timeSample, non-avar value in the rig and outside it, `rest:rx`, folded, add/remove `RigExecControl`, a broken `rigExec:joints` target. It prints medians split into Settle / Compile / Bake / Run, plus steps run. Built but **not** registered with ctest (repo convention, CMakeLists.txt:1054-1086). | tests/benchEditLatency.cpp, CMakeLists.txt | reproducible numbers for every later tier | low | S-M | none | reproduces §3's baselines within noise |
| F0b | Add `TF_PY_ALLOW_THREADS_IN_SCOPE()` at the top of `RigExecRigEvaluator::Evaluate` (10750). This is defensive. The live case is a C++ host holding the GIL, such as Hydra `Render` (usdImagingGL/wrapEngine.cpp:130 → bridge.cpp:1933/2532/2633), with exec calls at 11475/11488 before the only dynamic guard at 12304. Also delete the stray `RIGEXEC_MEASURE` block (11432-11451) and the dead `_authSnapshotCache` (rigEvaluator.h:1003, cleared at 8110); keep `_authSnapshotDirty`. | rigEvaluator.cpp/.h | removes a GIL-deadlock class; stops a stderr print | low | S | none | G-parity; a Python test that holds the GIL through Render |
| F0c | `RigExecRunSparsePlan` (frameCacheSparsity.cpp:293-306) iterates a topological cluster order stored at Build. `ClusterTopologicalOrder` (bakedSchedule.cpp:1013-1047) is exported and cached on the clustering. Fix the comment at 297-300 and update the synthetic test (testRigExecFrameCacheSparsity.cpp:562-610). The bug is latent: `RigExecRunPartialCone` has no callers (frozenContext.cpp:6776, .h:920). | frameCacheSparsity.cpp, bakedSchedule.cpp/.h, frozenContext.cpp, tests | fixes a latent ordering race | low | S | none | new case: a successor cluster with a lower id; a fake runner asserts that predecessors ran first |
| X0 | Add `RigExecEvaluationMode::ExecReference`, which always runs `_EvaluateDynamic`, with `RIGEXEC_EVALUATION_MODE=reference`, rigExecPose `--mode reference` and a Python binding. Migrate every test that uses Dynamic as its reference (list below). Add a CMake variant that runs the 29 default-mode suites under `=reference`, and pin the ChainLevels, SkinTopology and plain EpochRests runs to it. | rigEvaluator.cpp/.h, bakedProgram.h, python/_rigexec.cpp, tools/rigExecPose.cpp, tests, CMakeLists.txt | keeps the oracle testable once Dynamic stops being it | low | M | none | G-parity; reference output equals today's dynamic output byte for byte |

Tests that X0 migrates to ExecReference:
* testRigExecBakedMode.cpp:1335, 372-378, 533, 2305-2316;
* testRigExecBakedAttribute.cpp:208, 401, and, once HP-D-flip lands, the `GetBakedGenerationCount()==0` checks at
  231, 250, 332, 341, 442, 482, 500;
* testRigExecBakedSchedule.cpp:2105, 2172, 2264;
* testRigExecCurvenetAdjuster.cpp:162;
* testRigExecSolverBake.cpp:88;
* testRigExecSolverStacking.cpp:1211;
* the Python `dynamic` loops in test_float_curve_op.py, test_matrix_space.py, test_wire_mover.py,
  test_rigexec_biped_hand.py, test_rigexec_fk_start_frame.py, test_rigexec_baked_cone.py and
  test_rigexec_baked_attribute.py.

**Gate M0:** G-parity, with the F0a baseline recorded.

### M1: Compile floor

| id | change | files | gain | risk | effort | deps | verification |
|---|---|---|---|---|---|---|---|
| CF1 | Warmup diet (details below). | rigEvaluator.cpp | cold baked −19 ms (87.5 → 68.5, 6 runs, byte-identical). About +25 ms (*est.*) moves serially onto the first frame of a refused or fallback Baked epoch, so time-to-first-pose changes by 0..+6 ms net. Parity epochs are unaffected. | low-medium | S | M0 | G-parity; at least 7 cold runs at ≤ 70 ms; a scratch run of baked → `SetEvaluationMode(Parity)` records the `DeferredExecPrep` spike; the compile of a Baked rig with a connected provider (7806-7815) |
| CF2a | Three-task digest (details below). | rigEvaluator.cpp | cold −6.1 ms on top of CF1 (68.5 → 62.4); settle −3.5..4 ms on each structurally dirty evaluate | low | S | CF1 (for the cold gain) | `RIGEXEC_VERIFY_DIGEST_SPLIT=1` asserts whole == concat on the 28 stages and on the testRigExecArm cycle fixtures; at least 7 cold runs at ≤ 63 ms |
| CF2b | Solvers read memo (details below). | rigEvaluator.cpp | cold Solvers ~55 → ~35-42 ms, but Compile only −1..2 ms until CF3; settle 25-33 → ~15-19 ms (*est.*) | low (memo) / medium (prefetch) | S-M | CF2a | digest bytes with the memo on equal those with it off; settle measured with the bench equivalent of `RIGEXEC_SCRATCH_POST=settle` |
| CF3a | PoseInfoPrefetch as an SCC (details below). | rigEvaluator.cpp/.h | warm structural recompile −5..8 ms; dynamic compile −5..8; cold −0..3 until the lanes shrink. Today: 45,678 entries vs 15,727 unique; 12.4 ms cold, 10.6 warm. | medium | M | CF2b | the verifier across ctest, the 28 stages and a new mutual `default:space` cycle fixture in testRigExecDefaultSpaces: zero mismatches |
| CF3b | Early lane dispatch (details below). Lands only if F0a, after CF1 and CF2, shows a guide-join wait above 0. | rigEvaluator.cpp | −2.7..3.5 ms while the lane is critical | low: a rig that fails Validate now prepares guides (failure path only) | S | CF3a, measured | G-parity; diagnostic order on the failure path is unchanged |
| CF3c | Speculative bake beside the guide prepare (details below). | rigEvaluator.cpp/.h | 0..4 ms on its own | medium | M | CF3b, measured | a forced guide-prepare failure on a **recompile** after a published frame: IsBakeable decisions, mover-graph and created diagnostics, counters and `--pose-out` are identical |

CF1 details:
* When `deferExecPrep` is set (3701-3702), skip building and preparing `warmupTaps` (4012-4027, 4064-4070).
* The lane task prepares only the guides (4040-4048). That Prepare constructs the system on the lane, which keeps
  the registry's listener order (registry.cpp:655-686).
* The rest pull stays dispatched from main (8019-8022), after the guide join at 7932 (rule CF-b).
* The kill switch runs the reduced lane inline. Rewrite the comment at 4074-4076.
* Add Prepare sub-scopes inside the existing `DeferredExecPrep` scope (11234).
* Document that `TF_CODING_ERROR`s raised by the warmup (exec/requestImpl.cpp:357-370) no longer reach Compile's
  caller.

CF2a details:
* Add a segment mask to `_ComputeStructureDigest` (rule CF-c).
* Declare `std::string parts[3]` after `retiringBakedProgram` (3623) and before `digestDispatcher` (3680).
  Concatenate and apply `std::hash` at the join (8567-8574).
* The settle path (11192) uses `WorkWithScopedDispatcher`, which drops the GIL and isolates
  (withScopedParallelism.h:119), and honours `RigExecParallelEvaluationEnabled()`.
* No `_DigestWriter` refactor, no footprint pointer.

CF2b details:
* Add a digest-local memo per attribute, `{exists, typeName, sources}`, filled in hopFor (2495-2530) from the
  attribute that `GetAttributes` returns.
* attributeText (2461-2480) and a digest-local connection-input closure read from the memo (rule CF-d).
* An optional parallel per-prim prefetch lands only if it is at least 3 ms better on the median of 7 cold runs.
  Its per-task fills are merged serially.

CF3a details:
* Phase A: parallel per-prim reads of `{type, nsProvider, attribute sources, exists}`. The levelling covers the
  prims that **own** reachable nodes (connection sources, namespace-parent rest channels), not only the providers.
* Phase B: a serial iterative Tarjan over `(attrPath, connected)` nodes, using the successor rules of 1303-1338.
  Each SCC folds its providers and the sticky `connectedPose`.
* Missing attributes still land in `attributes` (1302-1304), and `newPoseInputInfo` holds exactly the prims that
  were reached (6704-6708).
* `attributes` is materialized eagerly in non-deferred epochs (`_BuildSolverInputIndex` at 7737), and at
  `_RealizeDeferredExecPrep` in deferred ones.
* The old walker stays available behind `RIGEXEC_VERIFY_POSEINFO`.

CF3b details:
* Hoist the rig checks, the four discoveries, the ribbon driver resolution and the lane dispatch above
  `_ValidateAdjustmentPoseConsumers` (3748).
* Move every lane-owned object and both dispatchers as one block (4001-4081).
* `_CompilePoseInterpolators` and the transform-authority warnings stay where they are.

CF3c details:
* Applies only when `deferExecPrep && parallel`, and only if the join wait measured after CF3a and CF3b is at
  least 3 ms (rule CF-e).
* The lane becomes a single task: the guides, then the rest pull, started by a main-thread signal once `RestTaps`
  is filled (7885-7892). Otherwise, state that the ~7.3 ms rest pull moves to after Bake, and re-measure.

**Gate M1:** G-parity + G-perf. Cold baked target (at least 7 runs): ≤ 62 ms after CF2a, ~55-60 ms after CF3.

### M2: Edit tiers

| id | change | files | gain | risk | effort | deps | verification |
|---|---|---|---|---|---|---|---|
| EP1a | Rest gate and lazy rest re-pull (§4.1). | rigEvaluator.cpp/.h | ~3.5-4.5 ms per edited evaluate on baked rigs; 0 on dynamic | low | S | M0 | testRigExecEpochRests; new cases: a session-layer `rest:rx` value edit, after which the parity oracle reads the new rest; a rest edit on an ancestor |
| EP1b | Prim-granular digest gate (§4.1). Today's digest records the outside-prim set; it is committed at 8567-8574 and at each settle. | rigEvaluator.cpp/.h | out-of-rig and other-rig edits: 26-36 ms of digest → 0 (the cache-drop rebuild remains until EP3b) | medium | M | EP1a | `RIGEXEC_VERIFY_DIGEST_GATE=1` over testRigExecStageEdits, EpochRests, Constraints, NoAuthoring, Interactive, test_rigexec_stage_edits.py and the usdview runners: zero violations; recording costs ≤ 1 ms on the compile lane (rule S1) |
| EP2a | Certain-structural skip (details below). | rigEvaluator.cpp/.h | T4 −24..29 ms (87.6 → ~58-63) | low | S-M | EP1b | test_rigexec_stage_edits.py, testRigExecArm (762, 3119), testRigExecVolumeWeights (543/573/588): rebuild lines identical |
| EP2b | Failed-compile memo (rule T-f; details below). | rigEvaluator.cpp/.h | T5 54-68 → ~0.3 ms per frame | low | S | M0 | F0a broken scenario; the repair frame recompiles; a mode switch while broken recompiles |
| SU1a | Step-granular closure (rules C2, C3), keeping the time predicate and the phasedReads rule. The parallel `RunFrom` skips unclosed members of a dispatched cluster; counters stay per cluster (1762-1787, 1820-1845). | bakedSchedule.cpp/.h, bakedProgramImpl.h, bakedProgram.cpp, bakedVerify.cpp, CMakeLists.txt | serial finger drag 1.1-1.5 → 0.8-1.0 ms; wide drags and scrubs gain nothing; the steady floor stays ~0.45-0.6 ms (Derived extent, prologue, publish) | medium: grain-0 verification once found an undeclared read that clustering had masked (bakedVerify.cpp:740-755) | M | M0 | new ctest variants with `RIGEXEC_BAKED_GRAIN_US=0` + `VERIFY_CONES` next to each existing one (CMakeLists.txt:386-405); at least 7 untraced interleaved runs each of finger drag, hips/neck drag, animated scrub and static scrub: no regression |
| EP3a | Path-targeted dirtying (rules S2-S4, S7): `edited` bits, fallbacks, copy into frozen clones, the `Edited` disposition, overridable paths in the affected index. | rigEvaluator.cpp/.h, bakedProgram.cpp, bakedProgramImpl.h, bakedSchedule.cpp, frozenContext.cpp, outputAffectedIndex.cpp, frameCacheSparsity.cpp, registry.cpp | T1 region run 3.6-3.9 ms → cone (~0.5-1 ms saved); T1b becomes a timeSample cone; edits to unread paths stop retiring the frame cache | medium | M | EP1b, SU1a | `RIGEXEC_BAKED_VERIFY_CONES=1` edited-vs-forced over F0a, StageEdits, EpochRests and FrameCache*; plus a frozen rewarm at the clone's `lastTime` after a dynamic-fallback frame, and a retargeted `resolvedAttr` walk followed by an upstream value edit |
| EP3b | Scoped cache clears (rules S5, S6). | rigEvaluator.cpp, moverGraph.h | BakedPrologue after a non-avar edit 2.8 → ~0.13-0.3 ms | medium | M | EP3a | a shadow evaluation with the wholesale clears, byte-compared; plus a default edit on a property-chain mover input, an `inputs:method` edit on a skin mover, and a blend sample edit |
| EP1c | Attribute-granular digest gate (§4.1), with the probe footprint, the listing-existence exception and the instance-proxy rule. | rigEvaluator.cpp/.h | T1c settle 26-36 ms → 0 for in-rig attributes the digest does not read, including the first edit in a layer | medium | M-L | EP1b, CF2a | the gate verifier plus new ctest cases, each of which must recompile: first-time `normals` authoring on an out-of-rig target mesh; a session-layer point cardinality change; the first `rigExec:activation` on an external blend sample |
| EP4a | Rest-ladder constant patch (rule T-c; details below). | bakedProgram.cpp, bakedProgramImpl.h, bakedPose.cpp, rigEvaluator.cpp, frozenContext.cpp | T2 57 → ~30-37 ms on its own; → ~2-5 ms with EP1c + EP3a | medium | M | EP3a, EP1c | a debug env patches, then forces a rebuild, and byte-compares every provider's rest channels on the 28 stages; a first-time `inputs:aimVector` authoring still rebuilds |
| EP4c | Treat an `execTypedArrayInputs` override as a placeable no-op (details below). | bakedProgram.cpp, bakedWeights.cpp, frozenContext.cpp, rigEvaluator.cpp, tests | a typed-array drag stays in the graph (5.9 → ~1 ms) | low | S | EP3a | rewrite TestAnArrayOverrideOnACurvenetWeightFallsBack (testRigExecBakedMode.cpp:1297-1345) to assert byte parity under BakedWithParityCheck |
| EP4b | Step-parameter constant patch (details below). | bakedPose.cpp, bakedProgram.cpp, frozenContext.cpp, outputAffectedIndex.cpp | per-parameter T2 edits → ~1-3 ms; cached frames outside the cone survive | medium | M-L | EP4a | patch-then-rebuild compare; testRigExecFrameCacheSparsity |
| EP5' | Notice-keyed resolve memo (details below). | bakedProgram.cpp, bakedProgramImpl.h, rigEvaluator.cpp/.h | rebuild Bake 20.7 → ~16 ms (*est.*); T3 ~25 → ~20-22; T4 −3..5; cold compile gains 0 and must not regress | medium-high | L | EP1c, EP3a | `RIGEXEC_VERIFY_BAKE_MEMO` builds without the memo and compares every `RIGEXEC_BAKED_PROGRAM_DIGEST` table (bakedProgram.cpp:1437-1628), the query/resolvedAttr paths and `RigExecComparePoses`; cases: a spline re-authored after its first release (bakedProgramImpl.h:2955-2983), a single-sample avar, a new property chain that targets an existing input's walk |

EP2a details:
* The router tags two kinds of notice as certainly structural:
  * prim-path resyncs that create, remove, retype, toggle active on, or reparent a RigExec-typed prim under the
    rig root, checked against the previous epoch's footprint;
  * targetPaths changes on relationships the digest provably hashes: the per-type named lists (2171-2250,
    2928-3001), any relationship on an aggregate solver (2730-2739), `rigExec:joints` and `jointElements`, and
    connections inside the pose-input closure (2394-2404).
* `connectionPaths` in general is not tagged: `rigExec:jointWeights` moves no digest (1858-1860).
* A tagged edit calls Compile directly.
* "structural edit: epoch rebuilt" is pushed only when the new digest differs. An edit whose digest came out equal
  is counted as a false positive by a debug env; that count must read 0 on the stage-edit tests.

EP2b details:
* The memo is keyed on `(_stageEditSerial, _PeekEvaluationMode(), _evaluationModeSource)`.
* It is cleared by `SetEvaluationMode`, `_RefreshAttributeEvaluationMode` and any public Compile.
* It caches Compile's own error vector, without "structural recompilation failed".
* Documented behaviour while broken: `TF_WARN` (3643, 3816, 6262) is emitted once instead of every frame, and the
  build/attempt counters stop moving.

EP4a details:
* Record `patchableBinds` (path → input) in `RecordBind` for walks of length 1 whose head is the property and
  that are not varying.
* Exclude every path touched by Fold, FoldShape or a direct `rebuild` insert: `inputs:aimVector`
  (bakedPose.cpp:823-830), IK-joint `posed:space` and avars (742-747), ribbon points (511), and the root
  (bakedProgram.cpp:1905).
* The dry run reuses the checks of `_DryRunAvarPatches` (1231-1330).
* Apply:
  * write `.constant`;
  * set `B.ladderDisturbed` (bakedPose.cpp:2140-2144);
  * set the one-run `edited` bit of the override index, so solvers with `restOverrides` refresh (527-560, 2658);
  * mark `_epochRestFrames` stale (EP1a).
* Frozen stage 1: fold a patch counter into the frame-cache epoch digest (frozenContext.cpp:4190-4217).

EP4c details:
* Read `inputs:weights` and `rigExec:autoSmooth` without the override, on both the live path
  (bakedWeights.cpp:455-466) and the frozen path (frozenContext.cpp:643-651).
* Make `movedProperties` match the dynamic side (11408-11411).
* Filter the evaluator's resolved-input write (10425-10445).

EP4b details:
* Covers bound scalar inputs whose step already has a live arm: TwoBoneIk bend/stretch/softness/offsets
  (bakedPose.cpp:2719-2725) and the spline params.
* Patch `.constant`, re-derive `ikParams` and `splineParams` (282-288, 404-408, 2529-2530), and mark the step
  edited.
* First enumerate the other consumers derived at Build time with a patch-then-rebuild compare. Anything not
  covered stays rebuild-class.
* Also part of this item, stage 2: cached frames survive. That means extending
  `RigExecFrozenAvarRegionDigest` and `PatchFrozenAvarConstants` (frozenContext.cpp:4162-4187, 4284-4303) and
  seeding the patched paths in the affected index.

EP5' details:
* Memoize `Bake.ladder.resolve`, `Bake.input_table.resolve` **and** the per-entity IsBakeable verdicts. Without
  the last one, the ladder saving is absorbed by the IsBakeableJoin wait (bakedProgram.cpp:1790-1797, 2128-2135).
* The key is a path footprint: the prim path, every ResolveBind walk path and named channel, chainTargets
  membership, the capture and probe times (bakedProgram.cpp:1876-1885), and the stage identity.
* Invalidation runs in `_OnObjectsChanged` **unconditionally**: before the `if (_bakedProgram)` gate at 1863, and
  for avar-only notices too.
  * A changed-info notice drops the entries that read the path.
  * A resync drops entries by prefix.
  * A change to `/`, to a layer or to chainTargets flushes everything.
* An input backed by a spline, or a varying input, is never reused.
* Walk, geometry and revision memos are dropped: they store dense slot ints and copy other slots' composed rests
  (bakedPose.cpp:192-440).

**Gate M2:** G-parity + G-perf. F0a targets: T1c < 5 ms, T1d < 1 ms, T2 ≤ 5 ms, T4 ≤ 63 ms, T5 < 1 ms.

### M3: Dynamic runs the program

| id | change | files | gain | risk | effort | deps | verification |
|---|---|---|---|---|---|---|---|
| HP-D0 | Dynamic dispatches to the existing Throughput program behind `RIGEXEC_DYNAMIC_RUNS_PROGRAM` (off by default): rules D1, D2, D4. The `deferExecPrep` predicate is redefined, so CF1 reaches Dynamic epochs. The bail re-runs stay (D3 comes in HP-D2). | rigEvaluator.cpp/.h, bakedProgram.h, tests | with the flag on: dynamic compile 96.2 → the baked floor; animated frame ~4.1 → ~1.2 ms untraced; leaf drag 5.9 → the baked cone | medium: it flips the default path for ~22 example stages, for usdview, and for their background jobs (frozenContext.cpp:3947-3950) | M | X0, M2 (EP3a) | with the flag on, byte-identical to `--mode reference` on the 28 stages, frames 1..24; F0a on at least 3 non-biped default rigs, comparing compile, value edit and steady frame against today's dynamic |
| HP-D2 | Retire the `stageFrames` bail re-run only, and memoize bails per epoch and cause (rule D3). | rigEvaluator.cpp, bakedProgram.cpp | a persistent mid-edit bail stops rebuilding the program (~19 ms) every frame | low-medium | S-M | HP-D0 | a mid-edit NoCandidate fixture: the pose is valid and the diagnostics are identical to reference; one rebuild, not one per frame |
| HP-D-flip | Turn `RIGEXEC_DYNAMIC_RUNS_PROGRAM` on by default. **Requires user confirmation**: `--mode dynamic` and the default mode stop being exec-authoritative. | rigEvaluator.cpp | meets the product requirement for all 28 in-tree stages | medium | S | HP-D0, HP-D2, and EP4a measured: on the default rigs an index-hitting value edit must cost less than a ~19 ms Bake | G-parity with the default flipped |
| HP-D1 | `FoldPolicy::Editable` (details below). | bakedProgram.cpp/.h, bakedProgramImpl.h, bakedPose.cpp, bakedWeights.cpp, frozenContext.cpp | T3 folded edits in Dynamic: ~25 ms → the cost of a patch | medium-high | L | HP-D-flip, EP4b | the Editable parity variant; steady-frame cost against Throughput (at least 7 untraced runs) |

HP-D1 details:
* A value that only carries a value is bound, not folded, so that rule T-c patches reach it. Values that decide
  ladder kind, connection presence, or varyingness driven by sample count are still folded.
* Requires (a): a parity CMake variant that forces Editable (`RIGEXEC_FOLD_POLICY=editable` × parity ×
  `VERIFY_CONES`).
* Requires (b): one of two things, because registry.cpp:204-283 freezes whenever a program exists
  (frozenContext.cpp:2047-2050):
  * the frozen sampler covers every added per-frame read, in `RigExecFrameInputs` and in the control digest; or
  * Editable programs are refused for freezing.

**Gate M3:** G-parity with Dynamic running the program by default; the reference suites are green.

### M4: CPU gap closures (independent of exec; may interleave with M5)

| id | change | files | gain | risk | effort | deps | verification |
|---|---|---|---|---|---|---|---|
| HP2'a | Solver checkpoint (details below). | bakedProgram.cpp, bakedPose.cpp | checkpoint rigs leave the dynamic fallback | medium | S-M | SU1a | flip testRigExecSolverStacking.cpp:1044-1087 to expect a program; parity |
| HP2'b | Animated or connected skin layout (details below). | bakedProgram.cpp, bakedGeometry.cpp | layout rigs leave the dynamic fallback | medium | M | SU1a | a fixture with an animated layout; parity; VERIFY_CONES |
| HP2'c | Intervening or animated Xform, behind its own flag (details below). | bakedProgram.cpp, bakedPose.cpp, bakedSchedule.cpp, frozenContext.cpp | intervening-Xform rigs leave the dynamic fallback | medium-high | M-L | SU1a | flip testRigExecBakedMode.cpp:2841-2860; tests/python/test_intervening_xform.py under BakedWithParityCheck |

HP2'a details:
* A phased read that names a solver binds to the `fin` version its Solve commit writes.
* The record is `PointsToMatrix(walk-rest landmarks, fin)`, with `recordFrame`'s skip rules (11738-11757,
  12557-12575), gated on `!_snapshotPoints.empty()`.
* Lift both refusals (bakedProgram.cpp:584-622 and 2716-2737), and close the skin `transformPhase` gap (607-611).

HP2'b details:
* Delete the refusal (bakedProgram.cpp:668-682). Route these layouts through the existing
  `skinTopologyFixed=false` whole-array path (rigEvaluator.cpp:6540-6570; bakedGeometry.cpp:757-770, 1716-1809).
* Add `rigExec:skinningMethod` to the per-frame read, and set the step's varying inputs.

HP2'c details:
* One prologue source X(P) per candidate provider, added to the frozen `stageSeeds` and to the background sampler
  (frozenContext.cpp:3477, 4581-4591).
* When any X ≠ I, a pass after compose runs over **all** providers, parents first. It applies the exact dynamic
  formula `own*anchorExec^-1*X*anchorTrue`, including its round trips (rigEvaluator.cpp:10092-10227), to rest,
  base and seeded final.
* Any change of an X, or a flip of whether any X ≠ I, dirties every compose group.
* Two rest stores:
  * `B.restPts` and `restFrames` stay uncorrected for solvers, controls and the ladder, because exec's
    `computeRestFrame` ignores X (12216-12242);
  * a corrected walk-rest store feeds skin, commit propagation and checkpoints.
* The prologue replays the `resetXformStack` diagnostic (10108-10117).

**Gate M4:** G-parity; `_ReportBakeRequired` no longer fires for the three classes.

### M5: Exec in the graph, sources

| id | change | files | gain | risk | effort | deps | verification |
|---|---|---|---|---|---|---|---|
| EX1' | Exec runtime, proven on the oracle behind `RIGEXEC_DYNAMIC_PREPARED` (details below). | tapSet.h/.cpp, rigEvaluator.cpp/.h, bakedProgram.cpp | ~0.05-0.3 ms per oracle frame (*est.*: ~400 override-key resolutions per frame); the proven API that EX2a needs | medium | M | M3 | listed below |
| EX2a | `ExecSeed` as a source step, behind `RIGEXEC_HYBRID_PROGRAM` (details below). | bakedProgram.cpp, bakedProgramImpl.h, bakedPose.cpp, bakedSchedule.cpp, frozenContext.cpp/.h, bridge.cpp, rigEvaluator.cpp, tapSet.cpp, docs/specs/baked-step-graph.md, tests/testRigExecHybridProgram.cpp | a biped-class rig with a ladder refusal: ~4.1-8.2 ms/frame dynamic → ~0.8-1.0 ms, plus ExecSeed at 0.1-0.45 ms on frames where time or overrides move (*est.*); compile adds Bake (~19 ms), and the first frame keeps a lazy seed prepare; the 28 in-tree rigs: no gain, no regression | high | L | EX1', SU1a | listed below |
| SU2a | Exec step dirtiness (rules S8-S12): a program-owned time accumulator plus generation, coarse edits, a full-vector guard, and a failure that withdraws the frame. Nothing is built when a program has no Exec\* step. | tapSet.cpp/.h, bakedSchedule.cpp, bakedProgram.cpp, bakedProgramImpl.h, bakedVerify.cpp | a refused rig pulls no exec on steady and held-key frames (100-450 us saved on each) | medium | S-M | EX2a | an "exec pulls run" counter reads 0 on steady frames and 0 on held keys; VERIFY_CONES captures and restores exec memos and forces re-pulls; BakedWithParityCheck over held-key ranges after an oracle frame at a different time; CompileTotal and notice-handler cost unchanged on the 28 stages (at least 7 runs) |

EX1' details:
* **(a) Lane token** (rules X1, X2): one per `RigExecTapContext`, re-entrant for its owner scope, taken around
  every exec site. `clients` and the lazy `GetSystem` (tapSet.cpp:89-99, 121, 136, 142) are guarded by a
  per-context mutex that is taken only after the GIL is released.
* **(b) Isolated waits** (rule X4): `WorkIsolatingDispatcher` at rigEvaluator.cpp:3680, 4062, 4063, 12307 and
  bakedProgram.cpp:1790, or one isolate spanning each dispatch-to-join span.
* **(c) `RigExecTapSet::EvaluatePrepared`**:
  * only Compute or ComputeWithOverrides: no ChangeTime, no Prepare at the tapSet level;
  * keys come from a per-(prim, attribute or computation) memo that any covering resync clears;
  * keeps the skip-on-missing rule of tapSet.cpp:302-317;
  * leaves exec errors posted.
* **(d) Oracle prologue**, after `_SettleEpoch` and `_RealizeDeferredExecPrep` (11316, 11322): `PrepareRequest`
  on every request, then one ChangeTime (rule X3). Convert every oracle exec site: 11475-11497, 11566,
  12122/12131, 12317, 12403, 12496, 13146, 13312.

EX1' verification:
* ExecReference and BakedWithParityCheck on the 28 stages, frames 1..24, flag 0 against flag 1: byte-identical,
  diagnostics included.
* testRigExecInteractive (duplicate keys).
* A stage edit that removes a prim.
* A mixed-mode two-thread test: one baked evaluator making rest-changing edits and one oracle evaluator on the
  same stage, for 100 iterations; results equal the serial run.
* A Python two-thread GIL test that covers the plugin load on first prepare.
* Compile under `PXR_WORK_THREAD_LIMIT=1` with `RIGEXEC_ENABLE_PARALLEL_EVAL=1` does not hang.

EX2a details:
* **Eligibility.** A provider P and its namespace subtree are lowered only if every provider in the subtree:
  * (a) has no `connectedPose`, meaning no `_connectedPoseTaps` entry (rigEvaluator.cpp:7805);
  * (b) has a `parent:space` that is unconnected, unauthored and unsampled, so the Build-time propagation at
    bakedPose.cpp:867-918 stays exact;
  * (c) has a FirstFramePose tap.
* **Eligible classes:**
  * authored, sampled or chain-written `parent:defaultSpace`, `avars:defaultSpace` and `posed:defaultSpace`;
  * connected ones of those that do not reach a `parent:space`;
  * a connected `default:space` that does not reach a pose;
  * a connected `rest:space` that reaches a computed space (its rests come from the same seed request).
* **The step:**
  * It owns its request.
  * Its override template is built at Build: BaseOverrides first, in dynamic order (11389-11431), duplicates kept,
    types validated.
  * It writes the first versions and the prologue rest tables for its ranges (rule K1).
  * Its outputs are compared by value per slot, like avars (bakedSchedule.cpp:1369-1378).
  * ComposeSubtree skips its ranges.
* **The prologue** calls `Warm`, a full Compute. With a cold main cache, override pulls recompute everything
  upstream (rigEvaluator.cpp:11471-11475).
* **Frozen:** rule F1, plus the bridge change.
* **Amendments:** §6.

EX2a verification:
* Fixtures, each with a constraint or solver on an ancestor: an authored `parent:defaultSpace` (static and
  sampled), a chain writing `parent:defaultSpace`, a connected `default:space`, a connected `rest:space`. Frames
  1..24 are byte-identical to ExecReference, serial and parallel.
* A connected `posed:space` whose input is a solved joint **still refuses**.
* testRigExecFrozenContext asserts the refusal.
* A two-time frame-cache test produces two keys.
* With the flag off, the 28 stages are unchanged.

**Gate M5:** G-parity with the flag off and on; the hybrid fixtures are byte-identical to ExecReference.

### M6: Exec in the graph, interior steps (connected providers)

| id | change | files | gain | risk | effort | deps | verification |
|---|---|---|---|---|---|---|---|
| EX2b | Interior exec substrate (details below). Lands together with its first user, HP1'. | bakedProgramImpl.h, bakedSchedule.cpp/.h, bakedVerify.cpp, outputAffectedIndex.cpp | enables HP1' | medium | M | EX2a, SU2a | a synthetic program with stub exec steps shows that coneOf excludes order edges; an exec-in-flight atomic stays ≤ 1 under `=parallel` |
| HP1' | Connected pose providers at unconditional demand sites only (details below). | bakedProgram.cpp, bakedPose.cpp, bakedProgramImpl.h, rigEvaluator.cpp/.h, tests | a connected-provider rig: a full dynamic frame → the program plus dirty pulls (~55-100 us each); compile roughly neutral | high | L | EX2b, EX2a (the same provider's ladder refusals lower to ExecSeed) | listed below |

EX2b details:
* The order-only edge class, kept in a **separate** list (rule C6).
* The `ExecLane` pseudo-slot.
* Singleton clusters (rule E3).
* An exec cost row: ~55 us plus a per-tap term.
* Order-edge filtering in cluster cones, step cones, the output-affected index and the verifier.
* The parallel rule E2. Until EX3' lands, a frame that closes more than one Exec\* step runs serially.

HP1' details:
* **Sites.** An `ExecPull` plus CPU `ConnectedCommit` pair is placed at each unconditional demand site:
  * constraint targets and weightObject (12588-12591);
  * batch tails (12193);
  * the final sweep (13074-13076).

  Pairs are placed for every provider in the transitive `_poseProviderInputs` closure, in dynamic DFS order
  (12070-12090).
* **Identity chain.** A step-owned identity chain replaces `connectedInputCache` (12054, 12109) and
  `_connectedPoseCache` (12111-12165). When the identity matches, the commit is an exact no-op: no descendant
  delta and no commit.
* **Committed flags** reproduce the recursion stop (12075-12076, 12036). They are read for every intermediate DFS
  node.
* **Blocking table.** A static nearest-blocking table is built at Build. Steps never call `nearestBlocking` or
  `inheritsNamespacePose` (11780-11856).
* **Failures and exclusions:**
  * A refresh failure at these sites bails to the oracle.
  * Constraints with a `resolveBinding` site that reaches a connected provider stay refused (11721, 12678, 12730,
    12985, 13005).
  * The connected-tap Prepare stays before commit (7805-7818).
* **Reads.** For the cone, declared reads narrow to P's `_PoseInputInfo.attributes` closure. The pull itself
  still carries the full BaseOverrides (rule S11).

HP1' verification:
* A connected `parent:space` provider, with and without overrides, serial and parallel.
* A connected provider that feeds a batch tail.
* A connected provider used as a constraint target.
* **A disabled constraint whose source is a connected provider, followed by an IK constraint whose chain holds the
  provider's descendant.** This is the parity trap for ungated pairs.
* A pose cycle asserts that Compile rejects it (7682).

**Gate M6:** G-parity; the hybrid fixtures are byte-identical to ExecReference, serial and parallel.

### Deferred (not scheduled; each has a stated trigger)

| id | item (corrected form) | trigger |
|---|---|---|
| CF1-s2 | Skip the warmup in a Dynamic or parity epoch when **this evaluator's** previous epoch prepared against the same system generation. The generation is a counter bumped when `GetSystem` constructs and reset by `PrepareStageChange`; never compare pointers. Saves 13..17 ms of lane time; the wall-time effect is unknown. | After HP-D0, when only parity and ExecReference compiles still warm; measure wall time first. |
| EP2c | Reuse the settle digest in Compile when a byte-level snapshot is identical before the retract and after the re-author (3454-3565). The snapshot covers the session-layer relationship specs of old ∪ new solver keys, plus whether the solver and ancestor prim specs exist. Take the hint only from `_SettleEpoch` at 11193, with `RIGEXEC_VERIFY_DIGEST_REUSE`. Saves ~34 ms of worker CPU, ~0 wall time. | Contention measurements on warm recompiles show a main-thread gain of at least 3 ms. |
| SU1b | Compare the varying inputs by value: hoist their reads into prologue buffers that the bodies consume; handle solver rests via `ladderMovedSlots` over each solver's `restSlots` chain (bakedPose.cpp:539-547, 2090-2106); add a frozen-side compare, or keep the time predicate when `_FrozenRunSteps` passes no set; keep `RigExecResolvedInputs` counter parity. | A rig with animated solver, constraint or weight parameters shows the cost. The biped has 12 time-varying reads. |
| SU2b | A lazy exec-input index over the Exec\* requests (dirty-all only while the index is absent), and a reach filter that applies to the comparison only, derived from the solver-input index and gated on 0 parity mismatches. | A measured exec-pull share of frame time on a real refused rig. |
| EX3' | Hybrid-aware parallel executor (details below). | EX2b/HP1' fixtures exist, and audit S11(0) has been re-run on this box. |
| HP2'd | Exec-seeded AND xform-derived double placement (bakedProgram.cpp:549-580). | A product decision (§9). |

EX3' details:
* Transitive reduction kept as a separate dispatch-edge list, used only on convex frames (rule E1).
* The inline successor chosen by bottom level, using recalibrated costs (audit S11(0)); `kSpawnCostUs` ~8 us.
* `RIGEXEC_BAKED_SCHEDULE=auto`:
  * serial for programs without Exec\* steps, decided by a Build-time flag with no per-frame scan;
  * parallel only when the closed exec chain and the independent closed CPU work are each at least ~150 us.
* Per-frame serialization of the dispatched exec steps (rule E2).

---

## 8. Refuted ideas

The audit's §5 (S1-S14) still stands. This table covers only today's proposals.

| id | refuted part | lens | refutation | what survived |
|---|---|---|---|---|
| F0 (6) | Wrap Compile's dispatcher waits in `WorkWithScopedParallelism` | deps | Tasks `Run` before the isolate carry no tag, so a waiter inside the isolate cannot run its own lane task; under `PXR_WORK_THREAD_LIMIT=1` it hangs (python/_rigexec.cpp:596). The proposal also treated declarations (3680, 4062-4063) as waits, and missed the destructor waits, the digest join (8572), bakedProgram.cpp:1790 and the non-isolated `WorkParallelForN` (6856, 7878). | rule X4, in EX1' |
| F0 (7) | Guard `clients`/`GetSystem` with Find's static mutex | deps | That mutex is function-local (tapSet.cpp:72). Holding a process-global mutex through `ExecUsdSystem` construction means holding it through plugin load under TfPyLock: a lock-order inversion with binding calls that hold the GIL (_rigexec.cpp:863, 1014). | a per-context mutex taken after the GIL is released, in EX1' |
| F0 (1,2) | A 10-scenario mode in rigExecPose registered with ctest; five empty verifier envs | gain | tests/CMakeLists.txt does not exist; by convention benches are not registered with ctest; empty envs are YAGNI; the `edited` bits did not exist yet. | F0a; each verifier lands with its item |
| CF2 | `_DigestWriter` refactor, footprint pointer, TfToken statics | gain/deps | None of it is needed for the split: a segment mask does it in ~40 lines. `TfToken(name)` cannot be static where the name is dynamic (2736-2737). The memo's cold gain is 1-2 ms until the main-thread cuts land. | CF2a, CF2b |
| CF3 (c) | Speculative bake on the assumption that "only the unique_ptr moves" | deps | The lambda dereferences `&newGuideTaps` on the lane (4040-4046) while the commit moves it (8086-8087): undefined behaviour. The proposal left the rest pull's sequencing unspecified (4098-4102, 7885-7892). A re-bake loses geometry adoption and counts builds twice (11124-11173). | CF3c with rule CF-e |
| EP1 | Notice rules covering only prim resyncs and value fields; the stale rest flag tied to `_structureDirty` | deps/gain | A property's first spec arrives as a PROPERTY resync, so the rules missed normals/extent/widths, cardinality and blend activation (2877-2921, 3005-3031). A `rest:rx` edit is not digest-suspect, so the stale flag was never set. The wholesale cache drops were kept (1840-1908), so "~3.6 ms" was unsupported. | EP1a, EP1b, EP1c (§4.1) |
| EP2 (1) | Certain-structural on any `targetPaths`/`connectionPaths` of a footprint prim | deps/gain | Some connections are deliberately invisible to the digest (1858-1860). The rule over-fires into ~80 ms recompiles and new "epoch rebuilt" lines. | EP2a, narrowed |
| EP2 (2) | Author only the diff in `_ApplyDerivedStartFrames` | gain | Warm DerivedStartFrames takes 0.19 ms, and a cold compile has nothing to retract. The planner would also have to simulate the retract, because "authored wins" reads composed targets after it (3488-3509). | dropped |
| EP2 (3) | Reuse the settle digest in Compile | gain | DigestJoin waits 0, so this saves worker CPU only. It never applies on the same edit as (1). | EP2c, deferred |
| EP2 (4) | Failure memo keyed on `_stageEditSerial` | deps | The compile outcome depends on the mode (3701-3702, 7485, 7514-7518), and `SetEvaluationMode` changes the mode without a notice. | EP2b |
| SU1 (2) | Replace the time predicate with value compares in the prologue | deps/gain | `_FrozenRunSteps` shares the closure and has no set of changed inputs, so background frames would publish stale solves. Rests vary through the ladder, not through USD reads. The biped has ~0 varying steps. | SU1b, deferred |
| SU1 (3) | Keep phased-read records across runs | deps | The store is positional and append-only, so `Preceding`/`AtPrim` would answer wrongly, and frozen holes would emit "resolved to nothing". | dropped (rule C5) |
| SU1 (1), "replace" | Replace the per-cluster source maps with per-step ones | deps | outputAffectedIndex.cpp, frameCacheSparsity.cpp and the frozen clone all read the per-cluster maps. | SU1a (per-step tables added beside the per-cluster ones) |
| EP3 | Scoped clears keyed on "connection/target fields" and on "names a mesh"; "an unindexed path dirties nothing"; no copy into frozen clones; "avoids the ~136 ms burst" | deps/gain | Property chains would keep a stale `constantValue`; layouts are reached through connections; retargeted walks go unindexed; `_CloneImpl` drops the bits; full-eval provenance still retires every entry. | EP3a, EP3b (rules S2-S7) |
| EP4 | Promote a captured input to a per-frame read; promote folded paths when overridden; place computation overrides as a prologue write; cap at 64 promotions and rebuild when idle | deps/gain | Promotion sets `ladderVarying`, which blocks freezing (frozenContext.cpp:4003-4006), and recomposes every frame. Non-avar varying steps are fixed at Build (bakedSchedule.cpp:1145-1146), so a same-time edit dirties nothing. FoldShape inserts into `rebuild` but not into `folded` (bakedProgram.cpp:1717-1725: `inputs:aimVector`). Folds have no binding. Computation overrides collide with the overrides the walk appends on the same keys (12095-12104, 12184-12197, 12272-12281), and no product code issues them. No idle rebuild exists. | EP4a, EP4b, EP4c; folds move via HP-D1 |
| EP5 | A memo keyed by the digest footprint; walk and geometry memos | deps/gain | The digest ignores values (2000-2005), and T3 is exactly a value edit, so the memo would return the old constant. Walk entries store dense slot ints and copy other slots' composed rests. About 5 ms can be memoized, not ~10.7. | EP5' |
| HP-D | Retire every bail re-run; Dynamic runs the program at once; migrate six tests | deps/gain | NoCandidate and unusable-frame bails return valid poses in the oracle. Dynamic is the **default** (bakedProgram.h:73-76), so the switch flips ~22 example stages and usdview. Before EP4, a value edit would pay a ~19 ms rebuild that dynamic mode does not pay today. Many more tests assert Dynamic semantics than the six listed. | X0, HP-D0, HP-D2, HP-D-flip, HP-D1 |
| EX1 | "EvaluatePrepared never prepares"; a lease only for exec programs and the oracle; a thread-id lease; ~7 ChangeTimes saved | deps/gain | Compute recompiles when a system-wide recompilation is pending (execUsd/system.cpp:112-135). Baked rest refreshes move the shared system's time outside that lease. The Compile lane runs exec on a worker. A same-time ChangeTime costs ~0.001 ms. | EX1' (rules X1-X4) |
| EX2 | Lower **every** ladder refusal to a source ExecSeed; a PoseRest domain; all seven kinds plus order edges in one item | deps/gain | A connected `posed:space` is re-pulled mid-walk with solved inputs (12062-12181). `parent:space` changes propagation blocking at runtime (11780-11858), which a Build-time table cannot express. PoseRest would change all 28 programs. A source step needs no order edges. | EX2a, EX2b |
| SU2 | Harvest time bits from "the prologue's single ChangeTime"; filter exec's inputs by reach; assume plain-attribute keys go unreported | deps/gain | A ChangeTime that is a no-op after an oracle frame produces no bits (exec/system.cpp:48-54). A filtered vector changes exec's result, and the forced verifier pass reproduces the same result, so it cannot catch the error. Attribute input nodes **are** reported on time changes (attributeInputNode.h:68, program.cpp:618-660). | SU2a; SU2b deferred |
| HP1 | ExecSolve, ExecGuide, ExecSnapshotSubset; pairs at every demand site including `resolveBinding`; "prepare only emitted requests beside Bake"; adopt tap sets on rebuild | deps/gain | The solver, guide and weight refusals are unreachable (rule P2), and 504-514 is CPU `_ResolveWeights`. `resolveBinding` sites are gated at runtime (11700-11726, 12593-12663, 12721-12738, 12979-13012) and are not idempotent. `AdoptGeometryStateFrom` moves geometry caches, not exec state. | HP1' |
| HP2 (5) | Split a stacked batch's commit | deps | Unreachable: each `_SolverBatch` holds one solver (7579-7613; bakedPose.cpp:1368-1377). Dynamic collapses stacked writes through the last-writer `candidates` map (12540-12556). | the refusal is kept as an assertion |
| HP2 (2) | A shadow avar slot for double placement | deps | Dynamic's product value is the xform-derived last writer (10699-10714, 11618-11632). The shadow slot would match exec, not the oracle. | HP2'd, a product decision |
| HP2 (1), as written | A per-slot InterveningX plus PoseRest | deps/gain | The correction applies to the whole rig, and it needs two rest stores (12192-12242, bakedPose.cpp:226-2530). | HP2'c |
| EX3 | In-place transitive reduction; a CPU-only Σcost/CP branch in the chooser; 0.3-1 ms gained from overlapSnapshot | deps/gain | Under step closure, dispatched sets are not convex (rule E1). The ExecLane chain loses links when a middle exec step is not closed (rule E2). The cost table is 3-10x off (audit S11). The snapshot that produced the gain leaves the graph. | EX3', deferred |
| HP4 | PreferExec: an exec-authoritative Dynamic built as an in-graph program | gain/deps | The user allowed the oracle to stay separate, and HP-D already meets the requirement. The baselines were stale: 24 → 7 requests after S13, and dynamic compile is 96.2 ms, not 115-140. Compile would cost at least baked plus the ExecPrepare that does not overlap. Every drag would re-pull all requests (full BaseOverrides), making it 2-3x slower than the CPU program, and it would be a third byte-identical implementation to maintain. As specified it is also unsound: it recompiles inside steps, shares a consumable dirty flag with the oracle, and relies on a generation-local memo. | dropped; revisit only on an explicit product request (§9) |

---

## 9. Open questions

1. **Product:** HP-D-flip makes Dynamic, and therefore the default mode, CPU-authoritative, with bytes checked
   against ExecReference. Is that confirmed? Should an authored `rigExec:baked=false` mean ExecReference or the
   program?
2. **Product:** while a structural edit leaves the stage uncompilable, should the rig keep publishing an invalid
   pose (today's behaviour, which EP2b keeps) or the last good epoch with a diagnostic?
3. **Product:** exec-seeded plus xform-derived double placement (HP2'd). Should "a constraint targets a volume
   weight" become a validation error, or should last-writer semantics be blessed?
4. After CF1, how large is the first-frame spike in `_RealizeDeferredExecPrep` on a Baked epoch that refuses or
   bails? (*est.* +20-30 ms)
5. Does the parallel prefetch of the Solvers read memo scale? Serial and concurrent reads already differ by
   1.7-2x (Digest.Solvers 34 vs 55-73 ms; MoverDiscovery 5.3 vs 8.8-17.3), and the cause is unknown: TfToken
   registry, Sdf path tables, or the allocator.
6. Can `ExecUsdSystem` construction (~7-9 ms cold) move earlier without breaking the registry's listener-order
   assumption (registry.cpp:655-686)?
7. How does `ComputeWithOverrides` treat duplicate keys? Interactive overrides are applied twice
   (rigEvaluator.cpp:11389-11412). ExecSeed's template reproduces the vector order either way; the answer decides
   whether the vector could ever be deduplicated.
8. Is a CPU `ComposeSubtree` under an exec-seeded parent bit-identical to exec's child frames
   (bakedProgramImpl.h:1335)? Until that is proven, ExecSeed covers the whole subtree.
9. Does usdview or rigExecImaging ever evaluate two rigs of one stage from different threads? This decides
   whether the lane token is ever contended. The registry serializes by convention (registry.cpp:491-497, 909).
10. Does the attribute-granular footprint recorder (EP1c) fit in ≤ 1 ms on the compile lane, or must it be
    recorded lazily at the first settle?
11. Does `_RestInputNames()` cover every channel the exec rest request reads, including the ancestors'
    `parentRestFrame`? The deps review found the seven names plus the namespace ancestor
    (computations.cpp:442-455). A TF_DEBUG cross-check against the compiled network would settle it.
12. The cost table was fitted on a 20-core box (audit Q11). It must be re-fitted before EX3' or any re-clustering
    is judged.
13. Re-baseline on a quiet machine: today's medians were 87.5-92 ms against the audit's 80 ms. Every compile cut
    that lands must be re-measured with at least 7 runs.
14. For the first HP-D-flip measurement on default (non-biped) rigs: are value-edit rebuilds of Throughput
    programs (~19 ms) frequent enough there that HP-D1 must land before the flip?
