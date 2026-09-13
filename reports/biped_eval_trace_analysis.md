# Why the biped compile takes 490 ms, and how to cut and parallelize compile and evaluate

Date: 2026-09-12. Source trace: `reports/biped_eval.trace` (`rigExecPose examples/biped/Biped.usda --frames 1,2,3 --profile`).
Machine: Linux aarch64, 20 CPUs (10 fast + 10 slow cores), OpenUSD 26.08, Release build. The trace
reproduces within 10% on this box (Compile 445-455 ms, Evaluate 27-33 ms per frame).

Method: every claim below was measured twice by independent agents (an investigator and a skeptic who
tried to refute it experimentally in a separate copy of the tree), and the cited code paths were re-read
by a third. Numbers are minimums over 5 or more runs. The accepted fixes were then landed together in
one tree, merged, reviewed adversarially by three more agents, fixed up, and measured again by me in a
paired run on the quiet machine (section 7). All work happened in scratch copies; the checkout was not
modified. Deliverables next to this file: `rigexec-perf.patch` and `rigexec-perf-CHANGES.md`.

## 1. Headline

| | Baseline | With `rigexec-perf.patch` | Where explained |
|---|---|---|---|
| Compile, first compile of the stage | 490 ms in the trace, 446 ms reproduced | 104 ms | sections 2-4 |
| Digest re-run on the next Evaluate after any authored stage edit | ~260 ms | ~31 ms | section 2.4 |
| Evaluate, static pose, steady state | 27 ms | 8.0 ms | section 5 |
| Evaluate, animated pose (six keyed controls), steady state | 55 ms | 9.1 ms | section 5.4, invisible in the trace |
| Whole process, 8 animated frames | 1.02 s wall, 2.61 s user | 0.33 s wall, 0.96 s user | |
| Peak RSS, same run | 128 MB | 140 MB | warm executor pages, section 5.2 |

Output is byte-identical to the baseline on three stages (`Biped.usda`, `Biped_layered.usda`, and the new
`Biped_anim.usda`), the epoch digest is unchanged, and the ctest failure set is unchanged (the two
pre-existing failures on this box, `testRigExecCurvenet` and `testUsdNoodles`, are unrelated).

Three facts drive everything else:

- **Compile is dominated by redundant USD composition queries, not by exec.** 61% of it is one
  structural-digest loop that closes over the same 115 prims 748 times. Another 400k `GetConnections`
  calls hit attributes that have no authored connection anywhere.
- **Evaluate is dominated by two things the trace does not name:** a per-descendant USD read inside
  constraint propagation (the linear cost decay down the spine), and an exec executor that is never
  warmed, so every request recomputes the whole pose network from scratch. On animated stages a third
  one dwarfs both: every exec request is rebuilt on every frame.
- **Parallelism is worth little until the algorithmic work is gone.** The exec system is single-writer
  (concurrent preparation crashes it), the pose walk's true critical path is ~2 ms once the redundant
  reads are removed, and the only large safe overlaps are two compile-time tasks worth 17-30 ms each.

## 2. Where the 490 ms compile goes

| Region (trace) | ms | What it is |
|---|---|---|
| Compile.DiscoverValidate | 57 | discovery + validation passes (`rigEvaluator.cpp` ~1998-3667) |
| Compile.StructureDigest | 304 | `_ComputeStructureDigest()`; Digest.Solvers alone is 300 |
| Compile.SolverSchedule | 60 | Kahn levelling + 14 `TapPrepare solverBatch` (29 ms) |
| Compile.PrepareRequests | 67 | poseSeed/main/guides `TapPrepare` (33 ms) + provider closures (29 ms) |
| Commit, ChainOrderValidate | 0 | |

### 2.1 Digest.Solvers: 300 ms of the same closure over and over

`_ComputeStructureDigest` (l.1277-1972) hashes the rig's structure into a string. In the solver block
(l.1605-1783) it walks, for each of the 14 aggregate solvers, every relationship, every target, and
**every ancestor path of that target up to the root**, calling `appendSolverInputConnections(ancestor)`
at each step (l.1684). That lambda runs `_CollectAttributeConnectionInputs` (l.1014) plus a BFS over
`_CollectPoseInputInfo` (l.1160), neither of which is memoized.

Counted on the biped (temporary counters, reproduced independently to the digit):

| | count |
|---|---|
| solvers entering the block | 14 |
| relationship targets | 86, average ancestor depth 8.5, so 734 ancestor steps |
| `appendSolverInputConnections` calls | 748, over **115 distinct prims** (6.5x redundant) |
| `_CollectPoseInputInfo` calls | 2,370, over the same 115 prims (20x; `hips_bind` alone 270 times) |
| `GetAttributeAtPath` + `GetConnections` | 305,109 each |
| emitted path entries | 98,686 for 4,865 distinct paths; the digest string reaches 9 MB |

Time split inside the 273 ms: pose-info closure 154 ms, the final emit loop 83 ms, attribute closure
17 ms. About half of each is USD API (`GetConnections` 58 ms + `GetAttributeAtPath` 33 ms in the closure
alone), the other half is `std::set<pair<SdfPath,bool>>` inserts and `SdfPath::AppendProperty` /
`TfToken` churn. Hoisting `GetPrimAtPath` is worthless (0.1 ms total).

**Fix (landed as C1):** three function-local caches for the duration of one digest call: per-prim
`_PoseInputInfo`, the per-prim emitted block, and the per-attribute-path text of the emit loop. The
digest stays **byte-identical** (hash 16236336116187278719 before and after, on both `Biped.usda` and
`Biped_layered.usda`), so epoch identity is untouched, and the caches die with the call, so an edit
between two digests is still seen. Digest.Solvers: 273 to 31 ms; Compile 444 to 207 ms.

### 2.2 `GetConnections` on attributes with no authored connections

Compile issues 406,237 `UsdAttribute::GetConnections` calls (and 387,492 `GetAttributeAtPath`) for a rig
with **84** authored connection targets; 99.86% return empty. Each call builds a Pcp property index and
target index (`usd/property.cpp:171-231`), ~0.3 µs, ~120 ms over the compile, plus allocator churn. Most
target attributes (`avars:*`, `parent:space`) have no property spec in any layer at all, so the
composition work is pure overhead.

**Fix (landed as C2):** one file-local helper that returns empty unless `HasAuthoredConnections()`,
used at all 11 call sites. This is sound, not just empirically identical: `PcpBuildTargetIndex` derives
connections only from authored `ConnectionPaths` opinions, so no authored opinion means an empty
composed result, verified across 19 composition shapes (list-op deletes, references, inherits,
specializes, payloads, variants, instance proxies, layer mutes). Measured alone: Compile −90 ms and
3-6 ms off every Evaluate frame; on top of C1 it is worth −24 ms and −2 ms per frame, because C1 had
already removed most of the same calls.

### 2.3 The remaining 190 ms

Sub-region timing of the three other compile phases (temporary stamps, min of 5):

| Sub-region | ms | Cause | Fix |
|---|---|---|---|
| `_ValidateAdjustmentPoseConsumers` (l.1065, called l.2025) | 29 | walks all 491 prims and 17,650 attributes, 32,801 `GetConnections`, to reject reads of `RigExecCurvenetAdjustment` frames; the biped has none | landed (C3): early-return when no Adjustment prim exists **anywhere on the stage**, instance proxies and inactive prims included, because the BFS follows connections that can leave the rig; the two Adjustment tests still take the full path |
| provider closures in SolverSchedule + PrepareRequests (`registerInput` l.4672, `poseProviderClosure` l.4505, l.4834) | ~45 | `_CollectPoseInputInfo` recomputed 2,672 times over 345 prims, `_CollectAttributeConnectionInputs` 1,151 times over 115 | landed (C4): one Compile-lifetime cache shared by those passes; worth 9 ms once C2 is in |
| transform-authority warning pass (l.2094-2177) | 15 | `ComputePurpose` recomputed per descendant of 340 providers, to emit zero warnings | landed (C5): resolve each prim's purpose once; DiscoverValidate 22 to 9 ms |
| `TapPrepare` (17 `RigExecTapSet::Prepare` calls) | 62 | exec network **compilation** 36 ms + per-request `VdfScheduler::Schedule` 27 ms + one-time `ExecUsdSystem` construction 11 ms; never a Compute | section 4 |
| `poseDependencies` constraint loop (l.4599-4633) | 8 | constraints × solvers × reads × targets | not landed |
| mover walk, joint/control discovery | 4 | | |

### 2.4 The digest is not a one-time cost

`_OnObjectsChanged` (l.1248) sets `_structureDirty` on **every** `ObjectsChanged` notice, before looking
at the changed paths, and the next `Evaluate` (l.7203) recomputes the full digest. Measured: one avar
value edit on `hips_ctl` costs the next Evaluate 260 ms; so does a `DefinePrim` under an unrelated
`/Scratch` prim or a root-layer metadata edit. The interactive-override path (`SetInteractiveOverrides`,
which the imaging bridge uses) fires no notice and does not pay this, so it is per **authored** edit
(drag release, keyframe, undo, layer mute), not per drag sample.

C1 cuts this hitch to ~31 ms. Pre-filtering notices so value edits skip the digest is **not** safe as an
obvious next step: the digest hashes attribute values too (`jointElements`, sample counts,
cardinalities), so "value edits are not structural" is false here. That needs an audit of which fields
are structural before it can be attempted.

## 3. Reducing compile overhead: what was landed and what it gave

Cumulative, in landing order, static stage, min of 5 per row (track C's copy; the last row is the merged
tree measured by me, min of 7 paired with the pristine build):

| step | Compile | DiscoverValidate | StructureDigest | SolverSchedule | PrepareRequests |
|---|---|---|---|---|---|
| pristine | 444.4 | 48.1 | 276.1 | 56.9 | 61.9 |
| C1 digest memo | 206.6 | 48.0 | 34.8 | 58.8 | 62.3 |
| C2 HasAuthoredConnections | 183.1 | 43.1 | 30.9 | 51.6 | 55.0 |
| C3 adjustment early-out | 164.2 | 22.9 | 30.8 | 51.9 | 57.1 |
| C4 compile closure memo | 154.8 | 22.2 | 29.2 | 45.5 | 54.8 |
| C5 purpose hoist + memo | 141.1 | 8.9 | 30.3 | 45.2 | 54.0 |
| C6 digest on a task | 117.2 | 9.2 | 0.0 (hidden, 33-53 on a worker) | 47.3 | 56.3 |
| C7 exec warm-up on a task | 100.2 | 9.2 | 0.0 | 41.9 | 42.7 |
| merged tree, my paired run | 103.5 | 11.7 | 0.0 | 41.2 | 45.3 |

What is left (~100 ms): SolverSchedule and PrepareRequests hold ~16 ms of exec preparation plus a
9 ms wait for the warm-up; the rest is untraced non-exec bookkeeping in the schedule and request passes
that still walks the stage more than once. Not landed, recommended next:

- **A per-Compile prim index** (type name, attribute list with authored connections, relationship
  targets, built once in ~8 ms cold) that every pass and the digest read from. Measured as −145 ms on the
  baseline; it subsumes 2.2 and most of 2.3 and is where the remaining schedule/request cost would go.
  It must include prims **outside** the rig that connections reach, or it breaks epoch sensitivity and
  the solver-batch invalidation index, which is why it was left for a proper change.
- **Lazy tap preparation** buys nothing by itself: exec compile cost just moves to the first frame.
  Only fewer, wider rounds (section 4) reduce it.

## 4. Parallelizing compile: what is legal, what pays

Rules established from the OpenUSD 26.08 source and by experiment:

| Operation | Concurrent? | Evidence |
|---|---|---|
| `UsdStage` reads (`GetPrimAtPath`, `Get`, `GetConnections`, `GetTargets`, `UsdPrimRange`) while nobody authors | yes, but scales only ~1.5-1.6x on 20 cores | the digest's read closure 480 to 300 ms under `WorkParallelForN`, saturating at 8 threads: Pcp property-index resolution serializes on per-layer spin mutexes |
| `ExecUsdSystem::PrepareRequest` / `Compute` / `ComputeWithOverrides` / `ChangeTime` on one system | **no** | zero locks in `exec/system.cpp`, `exec/requestImpl.cpp`, `execUsd/requestImpl.cpp`, `program.h`, `runtime.h`; one program, one network, one main executor; `ChangeTime` is a single global cursor. Preparing 14 requests from 2 threads produced coding errors, from 4-8 threads a core dump; concurrent Compute hung |
| `RigExecTapSet` construction/destruction | no | they mutate `RigExecTapContext::clients`, an unguarded `std::set` (`tapSet.cpp:98,104`); lazy `GetSystem()` is unguarded |
| exec's own internals | already parallel | `Exec_Compiler::Compile` is one `WorkParallelForN` region fanning out recursively; at `PXR_WORK_THREAD_LIMIT=1`, `TapPrepare poseSeed` is 3x slower and `PoseSeed` 1.6x slower |
| `UsdGeomXformCache` | no, one per thread | `xformCache.h:37` |

So the 17 tap-set preparations cannot be threaded, and threading the digest before memoizing it would
have looked like a 120 ms win while hiding the real bug. What does pay, both landed:

1. **Overlap the digest with SolverSchedule + PrepareRequests (C6).** `newDigest` is computed at l.3668
   and first read at l.4926; nothing between reads it and Compile authors nothing to the stage. The
   digest runs on a `WorkDispatcher` task joined just before the commit, with the dispatcher's waiting
   destructor covering every early return. Compile −24 ms. The digest inflates from 31 to 33-53 ms on
   the worker while exec's own threaded Prepare runs, and is still fully hidden (the join waits 0.0 ms),
   but it needs a spare core.
2. **One wide exec compilation round instead of 17 narrow ones (C7).** A throwaway request holding
   `computePointFrame` and `computeRestFrame` for every provider and the array output of every solver
   compiles the shared network once; the 14 per-batch preparations then cost 5.5 ms instead of 30, and
   poseSeed 9 instead of 22. The tap set is constructed and destroyed on the compiling thread, only its
   `Prepare()` runs on a task, and nothing else enters exec until the join. Compile −17 ms on a quiet
   box. **This gain is conditional on a free fast core:** the host has 10 fast and 10 slow cores, the
   warm-up task takes 37-41 ms when placed well and 65-120 ms when not, and on a saturated box a paired
   A/B put it ~10 ms behind the C6-only build. The fused-but-serial variant is not a fallback; it
   measured ~3 ms worse than no warm-up. It is three self-contained hunks if deterministic compile time
   matters more than the 17 ms. Fusing alone cannot remove the per-request `VdfScheduler::Schedule`
   (27 ms), which cannot be shared.

Critical path of the parallel Compile after the fixes: DiscoverValidate (~12 ms) then
SolverSchedule + PrepareRequests (~85 ms, exec-bound and necessarily single-threaded from rigExec's
side), with the digest and the warm-up hidden underneath.

## 5. Where the 30 ms frame goes

Full accounting of a static frame with every block scoped (residual 0.1 ms):

| Block | ms | Cause |
|---|---|---|
| pose walk (`_poseSteps`, l.7881-7962) | 16.8 | of which `commitConstraintFrames` 13.9 (section 5.1) |
| PoseSeed (one `ComputeWithOverrides`, 652 taps) | 5.2 | cold executor (5.2) |
| AuthoritativeSnapshot (1092 taps) | 1.9 | cold executor, same fix |
| Chain body_geo.points | 0.7 | the skin is **skipped** on frames 2-3 of the static stage; a real skin evaluation costs 0.84 ms, LBS math 0.26 |
| SolverGuides (14 taps) | 0.6 | observational output, always computed |
| post-walk provider refresh (l.8413) | 0.5 | no-op on this rig (5.3) |
| `_ComposeInterveningXforms` | 0.37 | 326 `ComputeRelativeTransform` to find nothing to compose |
| publish loops into `std::map` (seed maps, final frames, joints, controls) | 0.9 | ~1,800 red-black nodes per frame |
| PropertyChains | 0.34 | |
| parity-only provider matrix maps | 0.18 | built although `cpuParityMode` is off |

### 5.1 The spine's linear cost decay: one USD resolve per descendant per ancestor step

`commitConstraintFrames` (l.7588-7681) re-materializes a world frame for every unblocked descendant of
a written joint. For each descendant it walks up to the nearest written ancestor and, at every step,
calls `inheritsNamespacePose` (l.7569-7581): `GetPrimAtPath` + `GetAttribute("parent:space")` +
`GetConnections` + `Get(time)`, ~700 ns. Per frame: 15,273 ancestor steps, **13,719 calls over 194
distinct paths (98.6% duplicates)**, 9.9 ms. A constraint's cost is proportional to the summed depth
gaps of its descendants, which is exactly the 1.2 to 0.5 ms decay from `hips_bind` to the chest and
0.22 to 0.10 ms down the neck. On this rig the predicate is always true (`parent:space` is authored
nowhere); every block comes from `_jointSolverBinding`.

**Fix (landed as E3):** compute once per Evaluate, for each provider, its nearest pose-owning
ancestor-or-self (closing over every path element, since a provider's parent need not be a provider),
and make the `blocked` test a lookup: blocked iff that owner lies strictly below the written ancestor.
This is equivalent to the half-open ancestor walk it replaces, and it is applied at both walk sites
(l.7628-7634 and the duplicate at l.7763-7770 in `refreshPoseProvider`). Output byte-identical. Static
frame 20.1 to 8.9 ms, animated 21.9 to 9.9 ms, the largest single evaluate win. Two new tests cover
the boundary rule at both sites; without them a build with blocking deleted passed the whole suite.

The remaining eager propagation (subtree scan, delta per descendant, `propagated` map) is ~1.7 ms per
frame. Removing it needs local frames with on-demand world composition, which must preserve two
behaviours: a degenerate descendant currently aborts the whole commit so the target does not move, and
native Xform descendants are deliberately not propagated (Hydra applies the ancestor delta). That is a
rewrite, not landed.

### 5.2 PoseSeed: the exec executor is never warmed

`ExecUsdSystem::Compute()` is called zero times in a run (51 exec calls, all `ComputeWithOverrides`).
`ComputeWithOverrides` computes into a throwaway `VdfSubExecutor` (`exec/runtime.cpp:250-268`), so the
shared main executor is permanently empty and the entire namespace-chain network is recomputed on every
call. Proof: at one time code a second `ComputeWithOverrides` costs the same 7.6 ms; a plain `Compute()`
then a second `Compute()` costs 0.055 ms; after one warm `Compute()`, the override call costs 0.1-0.2 ms.

**Fix (landed as E2):** `RigExecTapSet::Warm(time)` = `ChangeTime` + `system->Compute(*_request)` with
the value view dropped, called once per Evaluate on the pose-seed request before the override pass.
Static frame: PoseSeed 6.5 to 0.4 ms, AuthoritativeSnapshot 2.1 to 1.0, the 14 solver batches 2.9 to
1.7; static Evaluate 27.8 to 20.1 ms; on the animated rig the override call drops from 15 to 9.5 ms.
Byte-identical output across the C++ and python tests, a 200-frame scrub, drags, edits, layer mutes.
Two honest caveats: this is the first code that depends on exec's main-executor invalidation being
complete (`EfMaskedSubExecutor::IsEmpty()` semantics, `maskedSubExecutor.h:83-91`), which is
upstream-tested rather than proven here; and the warm executor keeps per-time pages
(`EfPageCacheStorage`), so a scrub costs 12-20 MB more RSS. Filtering the 12 irrelevant overrides is
the same mechanism (an empty vector takes the `Compute()` branch); the gains do not add.

### 5.3 Small serial waste

- `refreshPoseProvider` is entered 544 times per frame although the biped compiles **zero** connected
  pose taps: 1.3 ms alone, 0.3-0.7 ms once E3 is in. Landed (E4) as an early return only. A narrowing
  of the post-walk sweep to the connected-tap map was **backed out** in review: a plain `Xform`
  constraint source with an attribute connected to a control's `parent:space` lands in that map but
  publishes no `computePointFrame`, so refreshing it directly turned a valid pose with one
  passed-through constraint into an invalid pose with zero joints.
- `_ComposeInterveningXforms` rediscovers per frame that no provider has an intervening Xform: 0.37 ms.
  Landed (E5): anchors and candidates recorded per epoch; the correction loop deliberately still visits
  every provider, because a descendant of a corrected anchor must be corrected even when its own
  intervening transform is the identity (`testInterveningXform` catches the wrong version).
- The pose-seed publish loop now uses end-hinted inserts (E6, 0.3-0.6 ms). Publish maps for joints and
  controls, parity-only matrices, `RigExecSkinLayout::Validate` per frame, the dense 1.0 envelope in the
  skin kernel: each 0.1-0.4 ms, listed, not landed.

### 5.4 Animated frames: every request is rebuilt every frame

The static trace hides the biggest per-frame cost. `tapSet.cpp:179-182` passes `BuildRequest` a
**time-change** callback (arity 1, per `exec/request.h:28-41`) that sets `_prepared = false`, so on any
stage with time samples every one of the 17 requests is rebuilt and re-scheduled each frame: 26 ms of a
56 ms animated frame (AuthoritativeSnapshot 9.6, PoseSeed 7.7, solver batches 5.3, guides 1.8). The
biped example authors no time samples, so the trace shows 0 for this. The effect was re-derived with a
stage keying a single control (128 vs 137 ms over five frames), so it is not an artifact of the
synthetic overlay.

**Fix (landed as E1):** drop `_prepared = false` from the time callback, and rebuild on expiry instead
(`if (!_prepared || (_request && !_request->IsValid())) Prepare();`). The second half is required: the
old line was the only path that resurrected a request whose indices expired after a
deactivate/reactivate, variant switch, payload reload, or undo of a delete (`ExecUsdRequest::IsValid()`
stays false forever; `PrepareStageChange` only fires when a resynced prim no longer resolves). It also
fixes a pre-existing bug: on the baseline, deactivating and reactivating `hips_bind` leaves every later
Evaluate invalid with zero joints; a new python test pins the recovery. Animated frame 54 to 29 ms,
joints byte-identical over 8 frames on flat and layered stages. Open hole, noted by review: a request
exec *discards* outright (all indices expired, e.g. a payload unloaded over the whole rig) reports
`IsValid()` true again after `Discard()`, so that case still does not recover; it did not recover before
either.

### 5.5 exec's parallel engine on small rounds

The 42 per-frame solver-batch computes pay ~75 µs each of parallel-dispatch overhead (6.9 ms with
threads, 3.7 ms at `PXR_WORK_THREAD_LIMIT=1`), while PoseSeed and the snapshot profit from the same
engine (1.6x). The schedules are 310-930 nodes, above VDF's 32-node "small schedule" threshold
(`vdf/scheduler.cpp:2581`), so the gate is on the hot path and picks "parallel" on purpose; they are
small in time, not in node count. Fixing that means raising the threshold inside OpenUSD or issuing
fewer rounds; flipping `VDF_ENABLE_PARALLEL_EVALUATION_ENGINE` globally gives back the PoseSeed gain and
nets zero. After E2 the warm executor makes these rounds cheaper anyway.

## 6. Scheduling and parallelizing the subsequent operations

### 6.1 The pose walk

The **compiled** schedule is a total order: Compile gives constraint *i* an explicit dependency on
constraint *i−1* (l.4604-4606), so the Kahn loop advances one constraint per level and the seven
independent L0 solver batches are followed by 65 serial steps. The **true** data DAG, derived from the
compiled read/write sets plus the actual propagated write sets, has 28 levels:

```
L00[11] arm_l/arm_r/leg_l/leg_r FK, arm_l/arm_r 2-bone IK, neck splineIK, hips parent, spine_mid_follow_pos, 2 pivots
L01[6]  arm ikfk x2, hips scale, spine_mid_follow_aim, 2 pivots
L02[5]  spine splineIK, pelvis positions x2, heel rolls
L03-L07 toes, ballRolls, leg IK/ikfk, ankle/ball/knee/thigh aims, spine_0..2 (5-7 steps each)
L08-L17 spine_2 .. chest: ONE step per level (a real kinematic chain)
L18-L19 clavicles, neck_0, arm aims, wrist params (3 + 9 steps)
L20-L27 neck_1 .. skull: ONE step per level
```

With the baseline per-step costs, level-parallel gives 17.1 to 13.2 ms (1.3x): the spine chain alone is
6 ms of serial levels and the hips constraint 1.1 ms. After the fixes in 5.1-5.3 the serial walk is
~4 ms, level-parallel 2.2 ms, a task graph 1.9 ms; with eager propagation also removed, 3.2 to 1.3 ms.
**Parallelism is worth at most ~2 ms per frame; the caching fixes were worth ~13 ms. They came first.**

When the walk is parallelized, the design is: replace the *i−1* edge with a precedence edge to the
latest earlier step whose write set meets this step's read or write set; run each level with
`WorkParallelForN`; give every step private output buffers merged at the level barrier in compiled
order so diagnostics, `recordFrame`/`_chainSnapshots` writes, `constrainedProviders` and
`connectedInputCache` stay deterministic; one `UsdGeomXformCache` per task; and **never call
`RigExecTapSet::Evaluate` inside a parallel body**, because it drives the single exec system. Solver
batches therefore stay serial; the way to speed them up is fewer, larger requests.

### 6.2 Merging same-level solver batches

Compile builds one `_SolverBatch` per ready solver (l.4655), not per level as the header says (l.370),
so L0 is seven exec rounds. Merging to one request per level was prototyped: values byte-identical, 14
rounds to 7, ExecEvaluate 6.3 to 4.1 ms over three frames, SolverSchedule 57.7 to 50.6 ms. It is **not**
landed because `_solverInputBatches` is per batch, so a merged level makes an edit to one solver's
inputs re-run all solvers in that level; four assertions in `testRigExecConstraints` (l.2818, l.2974)
fail. It needs a per-solver input fingerprint inside the merged batch first.

### 6.3 The tail: snapshot, guides, geometry

- The geometry chain cannot overlap AuthoritativeSnapshot: it reads transform, influence and weight
  taps out of that snapshot (l.8757-8823) and `finalMatrices` built from it.
- SolverGuides only reads finished frames and could run beside the snapshot, but both are exec requests
  on the one system. Giving the guides their own `ExecUsdSystem` is the only route to that 0.55 ms;
  gating them on whether anyone reads `pose.solverFrames` is cheaper and needs an API decision.
- The publish loops are pure fan-out but insert into `std::map`; they become parallel only after the
  maps become index-addressed vectors.
- Across geometry targets, `_chainOrder` is a topological sort; chains without edges could run under a
  `WorkDispatcher` per level, with `_liveGraphs` pre-created at Compile, `_chainSnapshots` respected per
  level, and `RigExecCurvenetBindCache::Resolve` locked. Worth 0 on the biped (one chain). Inside the
  skin kernel, `WorkParallelForN` over point ranges is the real intra-frame parallelism (LBS 0.26 ms,
  gather/write-back 0.12, `Validate` 0.28 if kept); switching the mover graph to
  `VdfParallelExecutorEngine` does nothing, the graph has two nodes.
- A repeated evaluation at an unchanged time could skip the snapshot and guides (3 ms), but the proposed
  guard (`ConsumeDirty` + override equality) is unsafe: it fails `testRigExecArm`,
  `testRigExecInteractive`, `testRigExecVolumeWeights`, `testRigExecWeightOverlay` with stale
  generations. A real invalidation source is needed; playback never hits this case anyway.

### 6.4 Pipelining across frames

Frame N+1's pose while frame N's tail runs would hide ~5 ms of a ~21 ms baseline frame (proportionally
less now), but two structural blockers stand: the evaluator has single-instance per-frame state
(`_liveGraphs`, `_chainSnapshots`, `_resolvedInputs`, `_volumeWeightMatrices`, batch snapshots), and
`ExecUsdSystem::ChangeTime` is one global time cursor per system, so in-flight frames need one system
each, with one compiled network each (memory unmeasured). Lowest payoff per unit of change in this list;
do it last, if at all.

### 6.5 Mechanics and kill switch

Use `pxr/base/work` (`WorkParallelForN`, `WorkDispatcher`, `WorkWithScopedParallelism`), not raw TBB, so
`PXR_WORK_THREAD_LIMIT` and malloc tags keep working; the patch adds `work` to `rigExec`'s link list.
`PXR_WORK_THREAD_LIMIT=1` already turns every `WorkParallelForN` into a serial loop and runs
`WorkDispatcher` tasks inline, and `VDF_ENABLE_PARALLEL_EVALUATION_ENGINE=0` serializes exec; a
`RIGEXEC_ENABLE_PARALLEL_EVAL` `TfEnvSetting` checked at each rigExec parallel site would let a
regression be bisected without process-wide limits (not in the patch). Never share
`std::map`/`std::set`/`std::string` as an accumulator across tasks; index pre-sized vectors and merge in
index order.

## 7. The prototype patch

`reports/rigexec-perf.patch` (1310 lines) applies to this working tree from the repo root with
`git apply reports/rigexec-perf.patch` or `patch -p1 < reports/rigexec-perf.patch`. It assumes the
uncommitted profiler work already in the tree. It is **not** applied; that is your call. Contents:

| file | lines | changes |
|---|---|---|
| `libs/rigExec/rigEvaluator.cpp` | +354 / −76 | C1-C7, E2 call, E3, E4, E5, E6 |
| `libs/rigExec/rigEvaluator.h` | +6 | per-epoch anchors and intervening-Xform candidates (E5) |
| `libs/rigExec/tapSet.cpp`, `tapSet.h` | +37 / −2 | E1 (time callback, expiry rebuild), E2 (`Warm`) |
| `CMakeLists.txt` | +1 / −1 | link `work` |
| `tests/testRigExecConstraints.cpp` | +178 / −9 | new `TestSolverOwnedJointBlocksNamespacePropagation`; `TestConnectedParentSpaceSolverInputs` extended so the second blocking site executes |
| `tests/python/test_rigexec_stage_edits.py` | +41 / −1 | `TestDeactivateReactivateRecovers` |
| `examples/biped/Biped_anim.usda`, `examples/biped/README.md` | +187 | the animated overlay the evaluate numbers rest on, now reproducible |

No environment gates or build flags; every existing profiler scope still fires; three new scopes
(`Compile.DigestJoin`, `Compile.WarmupJoin`, `TapPrepare warmup`). Note that the `--profile` summary
now lists `Digest.*` and `TapPrepare warmup` on worker threads, so they are no longer additive with the
`Compile.*` regions.

Verification: the merged tree and a fresh copy of the checkout with only the patch applied both give
byte-identical stdout, `--joints-out` layers and empty stderr against a pristine build on all three
stages; digest 16236336116187278719; ctest 47 of 49 with the same two pre-existing failures. Three
adversarial reviewers (semantics, thread-safety and exec-cache, test coverage via 18 mutants) returned
"land after fixes": one blocker (the E4 sweep narrowing) was reproduced and backed out, three must-fix
items were missing tests and are now the three tests above. Remaining should-fix items, not done:

- guard `RigExecTapContext::clients` and the lazy `_system` construction with a mutex, so the C7
  invariant ("no tap-set lifetime event while the warm-up task runs") is enforced rather than assumed;
- add a `TfEnvSetting` kill switch for the two compile tasks;
- record the E1 discard hole (above) next to the code, or treat an invalid cache view as a rebuild trigger;
- a digest-sensitivity table test, a purpose-warning test for C5, and an out-of-rig Adjustment case for C3.

Per-change contributions are in `rigexec-perf-CHANGES.md`. `run.sh`-style reproduction:

```
build/rigExecPose examples/biped/Biped.usda      --frames 1,2,3            --profile static.trace
build/rigExecPose examples/biped/Biped_anim.usda --frames 1,2,3,4,5,6,7,8  --profile anim.trace
```

## 8. Follow-ups, in order of value

1. Per-Compile prim index including out-of-rig prims (−145 ms measured on the baseline; subsumes the
   guard; the remaining ~85 ms of schedule/request bookkeeping is where it lands now).
2. The first Evaluate of a session costs 19 ms against 8 ms steady state; nobody has profiled the premium.
3. Per-solver input fingerprints, then one exec request per Kahn level (−0.6 ms per frame, −7 ms compile).
4. Local frames with on-demand world composition in the pose walk (−1.7 ms per frame), preserving
   commit atomicity and the native-Xform rule; then real precedence edges and a level-parallel walk
   (−1 to −2 ms).
5. Rest-frame taps are epoch-constant (`computeRestFrame` for 326 providers, half of the pose-seed
   request); cache per epoch.
6. Gate SolverGuides on a consumer; move the skin layout behind a `shared_ptr` so the 2 MB parameter
   packet is not compared element-wise (0.15 ms per frame).
7. Structural-vs-value audit of the digest so `_OnObjectsChanged` can skip pure value edits (the
   remaining 31 ms per authored edit).
8. Upstream: VDF's small-schedule threshold is node-count based; these 300-900-node schedules run in
   90 µs and lose to dispatch overhead.

## 9. Limitations

- All measurements are on this 20-core heterogeneous aarch64 box, mostly with other builds running;
  deltas under ~15% were re-run and interleaved, but absolute numbers will differ elsewhere, and C7's
  gain in particular depends on a free fast core.
- The animated stage is synthetic (six keyed controls); the tap-set rebuild cost was re-derived with one
  keyed control, so it is not an artifact of breadth.
- `perf` is unavailable here (`perf_event_paranoid=4`); attribution relies on scoped timers and counters,
  cross-checked by two agents per finding. USD's `TraceCollector` inflates per-call costs ~3x and was not
  trusted for magnitudes.
- Pre-existing failures on this box (`testRigExecCurvenet`, `testUsdNoodles` for a missing `GL/glu.h`)
  are unrelated and identical before and after.

## 10. Two follow-up questions, measured on the patched tree (2026-09-13)

Method as before: three investigators on copies of the patched tree, each finding re-measured by an
experimental skeptic and re-read by a code skeptic; corrections folded in. Stage for geometry and bake
numbers: `Biped_anim.usda`, steady-state frames, min of 5-25 runs.

### 10.1 Fixed topology: segment and parallelize geometry across the per-target networks?

Where the animated frame's geometry goes today (1.52-1.62 ms of a ~9 ms frame; 0.45 ms on the static
stage where the skin is skipped):

| part | ms | nature |
|---|---|---|
| re-read `rigExec:jointIndices`/`jointWeights` off the stage, `RigExecSkinLayout::Validate` over 262,760 slots **twice** (assembler and kernel), 26k base-point compare, 2 MB packet carried and compared | 0.62-0.70 | epoch-constant re-derivation |
| constant weight-1 envelope: preceding copy, `ResolveAll` (a per-element `TfToken` compare), per-point blend | 0.17-0.19 | dead work when the envelope is the identity |
| LBS kernel over 26,276 points × 10 slots | 0.26 | the only arithmetic; the "SIMD" path is SSE2-only, so it is the scalar kernel on this ARM box |
| scratch gather in, write-back, result copy-out | 0.15 | copies around the kernel |
| evaluator prologue per revision (resolved-inputs map copy, 315 KB basePoints assign, matrix gathers) | 0.17 | bookkeeping |
| VDF dispatch and buffers | ~0.06 | |

Answers, all bit-identical on every published point array:

- **Assume fixed topology first, no threads.** A per-epoch shared skin topology (indices, weights,
  element size, validated once; identity compare in the packet) plus the constant-envelope fast path
  takes geometry 1.52 to 0.72 ms. This is the largest and safest part, and it does not depend on the
  network shape. Invalidation caveat: the structure digest does **not** hash the layout arrays (only the
  `rigExec:influences` targets), so a digest-keyed cache serves a stale layout after a weight-paint edit
  (it failed `testRigExecArm` and `testRigExecMoverGraph`). The cache must be an evaluator member cleared
  from `_OnObjectsChanged`, and Compile must refuse it when the layout attributes are time-sampled or
  driven by a connection or property chain.
- **Parallelize inside the kernel, not across networks.** `WorkParallelForN` over point ranges (grain
  512-2048) makes the LBS 0.26 to 0.08 ms and the blend 0.07 to 0.03, bit-identical; geometry lands at
  0.54 ms, the frame at ~7.5 ms. The copies around the kernel stay serial.
- **Segmenting one mesh across N `VdfNetwork`s was measured and is the wrong axis.** With the same
  fixed-topology packet on both sides, 4 to 20 segment networks under a `WorkDispatcher` run in 0.131 ms
  versus 0.214 ms for one network with the in-kernel loop, and the only reason they win is that each
  segment parallelizes its own gather, write-back and copy-out. Each network costs 4.5 µs per frame of
  fixed overhead, N networks run serially are strictly slower than one (0.38 to 0.55 ms at N=32), and the
  net gain is 0.08 ms per mesh per frame for N schedules, N packets and a stitch. Making the skin node
  work in place on the Vdf buffer recovers the same 0.15 ms with no segmentation; that needs a Vdf-level
  raw accessor, which the read/write iterators do not expose.
- **Where cross-network parallelism does pay: across targets.** The biped has one skinned mesh, so it
  gets nothing (0.66 to 0.66 ms). On synthetic stages with 5 and 9 independent skinned meshes, running
  each dependency level of `_chainOrder` under a `WorkDispatcher` takes geometry from 13.7 to 3.7 ms
  (9 meshes) by itself; the marginal gain once the serial fixes above are in is 2.3 ms per frame at
  9 meshes and 0.9 ms at 5, at ~31% parallel efficiency (24-34 ms of CPU for 13.5 ms of work). It must
  be gated on level size. A flat dispatch of `_chainOrder` is wrong (fails `testRigExecArm` and
  `testRigExecInteractive`); levels are required. Even with levels, the region has unsynchronized reads
  that peers in the same level mutate (`_chainSnapshots` lookup vs record, the curvenet bind cache, a
  shared `UsdGeomXformCache`, `movedProperties`), measured overlapping in 6 of 6 runs, and
  `pose.diagnostics` order becomes nondeterministic. The design needs per-task buffers concatenated in
  `_chainOrder` order and snapshot reads resolved at level boundaries, not a mutex.

Ceiling on the biped: geometry 1.6 to 0.55 ms per frame, frame 9.3 to 7.7. Beyond that the geometry
floor is the copies and the ARM scalar kernel (a NEON path is estimated at another ~0.13 ms).

### 10.2 Encapsulate the rig: bake the graph internals as an optional mode

What is "dynamic execution and graph overhead" in a patched frame, every microsecond classified
(static frame 7.9 ms; animated 9.0 ms):

| class | static ms | share | what it is |
|---|---|---|---|
| exec round-trip plumbing | 2.4 | 30% | 17 requests per frame; sub-executor entry, one value-key resolve per override (922 per frame), override install, invalidation replay, a throwaway data vector, copy-out. The 14 solver kernels inside are 0.014 ms (158-177:1). Same requests with no overrides: 6 µs each; with one override: 30 µs; with the real vector: 1.8 ms |
| pose-walk bookkeeping | 1.9-2.0 | 25% | `std::map<SdfPath>` descendant enumeration, closest-ancestor climbs, blocking lookups, propagated-map inserts; pose math is 0.35 ms |
| USD attribute reads outside exec | 1.0 | 12% | 1,119 value resolutions per frame, 708 through `_ResolvedRead`, **none time-sampled**; 0.33 ms is unguarded `GetConnections` at `moverGraph.h:190` (a call site the patch missed) |
| publish into `RigExecRigPose` maps | 0.8 | 10% | 326 + 252×3 + 74 map inserts, two exec-feeding override vectors |
| geometry | 0.45 / 1.24 | 6-13% | see 10.1 |
| property chains | 0.34 | 4% | 0.28 of it is reads, 0.06 math |
| first frame of a session | +11.7 once | | 7.4 ms first warm compute (8,295 attribute reads), first schedules and packet assembly 1.6, first-touch allocation 1.5; 9-10 ms of it could be paid at Compile |

**A baked program was built and measured.** A flat op list over dense slots: 12 `UsdAttributeQuery`
reads (the only time-varying inputs; 17,554 of the rig's 17,650 attributes are constant), 326 provider
frames composed as one avar compose plus three 4×4 products each (8 of the 14 computations exec
registers per provider are epoch-constant on this rig, including the whole default-space ladder), the
14 solver kernels in compiled batch order (the SplineIK rest description and the two-bone lengths bake
out), the 65 constraint kernels in `_poseSteps` order with the 1,538 descendant propagations
precomputed as pairs, 389 `PointsToMatrix`, and the LBS kernel. Cost: **0.35 ms warm, 0.42 ms cold,
against 8.9 ms dynamic**, bit-exact (max error 0.0) on 252 joint frames, 252 matrices and 26,276 points,
at 30 distinct times and on all three stages. The rig solve without the skin is 0.12 ms. Per stage:

| stage | dynamic | baked |
|---|---|---|
| provider frames (PoseSeed) | 0.56 | 0.03 |
| 14 solver batches | 1.45 | 0.012 |
| 65 constraints + propagation | 2.3-2.4 | 0.07 |
| AuthoritativeSnapshot (pure re-derivation of values the evaluator already holds) | 0.93 | 0.008 |
| SolverGuides (second request over aggregates the walk already has) | 0.53 | 0 |
| skin | 1.44 (chain) | 0.23 serial, 0.04 parallel |

Fair floor for a complete frame, including map publication (0.12), derived extent (0.13), guides, the
envelope and property chains: 0.5-1.5 ms. Defensible gain: **7.4-8.4 ms per frame, roughly 8 ms**, i.e.
a 6-17x frame, not the 25x the raw prototype suggests. Bake time is ~9 ms on top of Compile. Memory:
~0.5 MB of dense pose state plus a 2.7 MB skin table, a wash against the packet the dynamic path already
materializes and compares every frame.

**What it costs, and where the prototype is not yet honest:**

- It is a second implementation of the evaluation semantics. The biped uses none of the awkward
  features: 0 connected-space providers, 0 intervening Xforms, 0 weight objects or volume weights,
  0 geometry-domain constraints, 0 SingleChainIK, 0 ribbons, curvenets, blend shapes or profile movers.
  Each of those must be written a second time or make the rig refuse to bake.
- **The epoch digest is not a sufficient invalidation key.** It excludes values by design: keying the
  IK/FK blend weight, or setting `inputs:enabled` false, leaves the digest bit-identical and changes the
  pose (measured divergences of 25.9 and a frozen hips). Ts-spline attributes report zero time samples in
  USD 26.08 while varying. Connected inputs resolve through their source. The correctness-critical piece
  is therefore a generated index from every changed property to the captured constants it feeds, and the
  safe default is "any changed-info notice on a prim that contributed a constant rebuilds the program"
  (one ~9 ms hitch). Admission for capture must be `!ValueMightBeTimeVarying() && !HasAuthoredConnections()`
  per attribute, generated by the same pass that captures.
- Property chains must be evaluated inside the program (the prototype captured their results from the
  last dynamic evaluate; with `foot:roll` keyed it diverges by 25). Interactive overrides must become
  slot writes (an override in flight diverges by 38). Both are straightforward but not done.
- Rest edits on bound joints take effect today without a recompile by design
  (`computations.cpp:1053`); baking the SplineIK rest and bone lengths changes that unless a rest edit
  rebuilds the bake.

**Toggle design.** `SetEvaluationMode(Dynamic | Baked | BakedWithParityCheck)`. `Baked` is a request:
Compile attempts to build the program, records a reason per unbakeable feature (`IsBakeable(&reasons)`),
and stays Dynamic when anything blocks. Structural notice: rebuild the program with the epoch. Value
notice on a contributing prim: rebuild (safe default) or, for bound input slots, write the slot and re-run
the whole program, which at 0.35 ms costs less than one of today's exec requests, so no finer dirty
analysis is needed. `BakedWithParityCheck` runs both paths in one generation and compares with equality,
which the prototype earns; `cpuParityMode` and the existing ctests become the gate by running the suite
twice with the mode flipped. The precedent already exists in the code: `_falloffLutOverrides` is an
epoch-constant captured at Compile and replayed until the next epoch.

**The cheaper tier that keeps exec** (measured or bounded, all inside the current architecture):
epoch-constant `computeRestFrame` taps out of the pose-seed and snapshot requests (implemented,
PoseSeed 0.46 to 0.40 ms, byte-identical); the missing `HasAuthoredConnections` guard at
`moverGraph.h:190` (~0.2 ms, free); a per-evaluator value cache for reads admitted by
`!ValueMightBeTimeVarying() && !HasAuthoredConnections()` and cleared on every notice (0.2-0.3 ms; the
process-lifetime version that measured 0.5-0.9 ms is unsafe); guides gated on a consumer (0.5 ms);
one request per solver level with per-solver fingerprints (0.4-0.6 ms, estimated); the fixed-topology
geometry work from 10.1 (0.8-1.0 ms animated); dense slots for publication (0.5-0.8 ms); paying the
first-frame premium at Compile (9-10 ms once). Together roughly 2.5-3.5 ms of the 8-9 ms frame with low
risk. The baked program is the way to remove the other 4-5 ms, and it is the only route to a frame
under 2 ms.

Recommended order: fixed-topology geometry plus the in-kernel parallel loop, then the tier that keeps
exec, then the baked pose program as the optional mode; cross-target chain parallelism only for rigs
with several independent meshes.
