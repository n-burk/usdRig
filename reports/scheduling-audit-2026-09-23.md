# Scheduling audit: one solver/constraint/mover loop, and a compile that overlaps what it legally can

Date: 2026-09-23. Commit 64d6a5f (rigexec-baked-step-graph, "exec optimization", with the staged `movers/`
refactor in the working tree). Source traces, all produced today by `build/rigExecPose.exe` on
`examples/biped/Biped_anim.usda --frames 1:8` (8 frames run twice, 16 `Evaluate` calls) with `--profile <file>`:

| trace | time | mode | executor |
|---|---|---|---|
| `reports/biped_anim.trace` | 07:24 | baked | serial (no cluster events) |
| `reports/biped_anim_baked.trace` | 07:28 | baked | serial |
| `reports/biped_anim_dynamic.trace` | 07:28 | dynamic | n/a (everything on tid 0) |
| `reports/biped_anim_baked_parallel.trace` | 07:36 | baked | `RIGEXEC_BAKED_SCHEDULE=parallel` (cluster events carry `level/steps/waitUs`) |

Supporting runs this session, all in the scratchpad
(`C:\Users\nburk\AppData\Local\Temp\claude\D--work-usdRig-usdRig\5d6a39cc-e23c-477b-8d1b-130ee0f375ad\scratchpad`):
- A `RIGEXEC_BAKED_SCHEDULE_REPORT=1` re-run with the same binary (`sched_report.txt`, `rerun_par.trace`): 634 steps,
  1115 edges, 76 clusters, 330 cluster edges; cluster levels identical to the 07:36 trace.
- 3+3 cold compiles with and without `RIGEXEC_ENABLE_PARALLEL_EVAL=0` (`par_*`, `ser_*`).
- Six further cold baked runs (`s1run1-6.trace`).
- Four back-to-back `Compile()` calls through the python binding (`cmp_recompile*.py`).
- Untraced `RIGEXEC_BAKED_STEP_TIMING` and `--repeat` A/B runs.
- A forced 512-vertex skin cut (`rep512.txt`, `q0_*/q1_*`, `ab_par_*`, `np_*`).
- C++ microbenchmarks of the constraint-table reads and the extent kernel (`bench.cpp`, `bench2.cpp`).

No file under `libs/` or `tools/` was modified.

Method. The traces were parsed into per-thread timelines: critical path, idle time, concurrency histograms and
realized cluster chains. Five area audits (traces, baked schedule, dynamic evaluate, compile, movers) produced 14
synthesized proposals. Two independent verifiers then attacked each proposal:
- a **deps** lens: data dependencies, thread safety, parity, lifetime;
- a **gain** lens: does the saving land on the critical path, measured against the traces and fresh runs.

Both corrected gain estimates are reported below. A proposal counts as "verified" only if neither lens refuted it.
Several refuted proposals have a corrected form that survives; the plan calls those out explicitly.

Caveat for every per-frame number: the profiler inflates the baked frame about 2-3x. The spec's untraced biped frame
is ~588 us on a 20-core box; untraced here it is ~1.1-1.4 ms including the skin. Relative conclusions from traces
hold; absolute microseconds do not. Untraced numbers are labelled.

## 1. Summary

1. **Compile runs on three threads out of 32.** Average parallelism is 1.74-1.84. Two workers each run one long
   single task: the structure digest (54-62 ms) and `TapPrepare warmup` (42-50 ms). The other 29 pool threads record
   no compile scope. After the digest finishes, the main thread runs alone for 43-65 ms (PrepareRequests, Commit,
   Bake) with every worker idle.
2. **The compile critical path runs through the exec warmup, and the main thread waits for it by accident.**
   `Compile.WarmupJoin` costs 9.5-15.3 ms (rigEvaluator.cpp:6937-6943). In baked mode nothing between that join and
   the guides Prepare (~7389) enters exec, because `deferExecPrep` (3521) skips every batch Prepare (7026). The
   largest compile win is to move the join and to move the guides/rest exec calls (10.1-12.6 ms) onto the exec lane:
   **~18-20 ms**. It is legal only with two corrections (Section 4.1, S1).
3. **Most of the remaining post-join compile work is dynamic-only bookkeeping that baked mode never reads.**
   `_solverInputBatches` costs 8.6-8.9 ms of serial stage walks (rigEvaluator.cpp:7033-7068), and only the dynamic
   notice handler (1796-1810) reads it. Deferring it and making `RestTimeVarying` parallel saves **~8.3-8.5 ms** (S4,
   verified).
4. **Compile.Bake runs 29-32 ms strictly on main while every worker is idle.** Three changes save **~8 ms cold**
   (S6, verified; gain lens 8, deps lens 10): a resolve/commit split with prefix-sum override indices, one edge
   sweep, and IsBakeable overlap. Asynchronous teardown adds 4-5 ms on warm recompiles.
5. **Once items 2-4 land, the next floor is the single-task digest and the exec lane**, both ending around 59-80 ms.
   Until the digest is split or its join moves, further main-thread compile cuts (PoseInfoPrefetch memoization,
   parallel MoverDiscovery) buy nothing on wall time. Today they would only lengthen `WarmupJoin` (S2, S3 refuted on
   gain).
6. **Baked evaluate already is the one loop, but the parallel executor does not pay.** Pose and geometry steps
   form one DAG under one dependency-counting executor with no level barrier (bakedSchedule.cpp:1661-1830). Traced
   region medians are a wash (1120 us parallel vs 1121/1135 serial). Untraced `--repeat` runs show parallel **losing
   ~200 us/frame** (~1380 vs ~1170 us). Three causes:
   - The quotient graph turns a pose half that is 2.9x wide at step grain into ~1.5 clusters per level; 221 of 330
     cluster edges are transitively redundant.
   - Pose step bodies run 1.65-1.9x slower on workers.
   - 46-63% of the region is a serial skin + extent tail that waits on the last neck joint by data dependency.
7. **The per-frame upper bound is small.** Even perfect step-grain scheduling with serial step times cannot take the
   baked region below ~760-816 us traced (1.27x). The reachable frame wins are serial removals, not more
   parallelism:
   - constraint-table reads in the prologue (~75 us untraced);
   - the extent copies and scalar reduce (~100 us untraced);
   - a reclustered executor that stops losing (~50-150 us).
8. **Dynamic mode runs entirely on tid 0 as at least six sequential loops** (median 8.2 ms/frame traced). Exec is a
   single serialized lane: one `ExecUsdSystem` per stage (tapSet.cpp:65-121), mutated by `ChangeTime` (298). So the
   wins are fewer requests (24 to 7 per frame) and overlapping the authoritative snapshot with the CPU constraint
   tail: **~2.0 ms/frame**. This is legal only with a stricter merge rule and an earlier join (S13).
9. **Unifying dynamic mode into the step graph (S14) is legal but worth 0 ms on every in-tree rig.** All 28 stages
   under `examples/` that have a RigExecRoot bake fully. It is a product decision, not a scheduling win.
10. **Twelve of fourteen proposals were refuted as written.** Section 5 records each refutation so nobody
    re-proposes it. S1, S13, S10 (stage A), S11, S9 and S7 have corrected forms that survive; they are in the plan.

## 2. What the four traces show

### 2.1 Compile

Compile wall: 105.16 (`biped_anim`) / 118.13 (`baked`) / 104.97 (`baked_parallel`) / 137.88 ms (`dynamic`).
Fresh cold reruns in parallel mode: 103.3 / 104.5 / 109.6 ms. With `RIGEXEC_ENABLE_PARALLEL_EVAL=0`:
195.7 / 196.0 / 197.6 ms. Warm recompiles through the binding: 133.8 (cold) / 79.1 / 72.5 / 73.5 ms.

Main-thread critical path, `biped_anim.trace` (ms from Compile start):

| phase | start-end | dur | inside | serial by |
|---|---|---:|---|---|
| DerivedStartFrames | 0.01-5.60 | 5.6 | session-layer authoring under a process mutex, `TfNotice::Block`, `SdfChangeBlock` (rigEvaluator.cpp:3229-3388; 0.19 ms warm) | data (must precede every reader) |
| DiscoverValidate | 5.62-22.82 | 17.2 | Validate 2.26, MoverDiscovery 11.26 (3831-4474), SolverJointBinding 1.71, SolverDag 1.64 (4590-5293) | accident, but off the critical path (see S3) |
| SolverSchedule (to SolverBatches) | 22.82-40.39 | 17.6 | PoseConstraints 3.79, PropertyChains 0.83, PoseDag 11.53 (PoseInfoPrefetch 10.90, 6499-6567) | mixed; off the critical path |
| **Compile.WarmupJoin** | 40.39-49.92 | **9.53** | main idle on `TapPrepare warmup` (6937-6943) | **accident in baked mode** |
| SolverBatches after join | 49.92-58.56 | 8.64 | Kahn loop, 24 single-solver TapSets, `registerInput`/`GetRelationships`/`GetTargets`/`collectAttributeInputs` (7005-7068) | accident (dynamic-only index) |
| PrepareRequests | 58.57-73.27 | 14.7 | ProviderSeed/Closure ~0.7, RestTimeVarying 1.7-1.9 (7325-7337), GuideTaps 4.72, TapPrepare restFrames 2.71, RestFrames eval 3.73 (7376-7461) | exec order is data (single lane); running it on main is accident |
| Commit + universe | 73.27-74.23 | 0.9 | DigestJoin 0 ms, provider-universe build (7846-7900) | data |
| **Compile.Bake** | 74.23-103.58 | **29.35** | IsBakeable 1.26, rest_chain_and_default-space_ladder 9.82, input_binding_table 7.04, the_walk 5.21, the_step_graph 5.54 (bakedProgram.cpp:1498-2400) | mostly accident (serial commit halves) |
| tail | 103.58-105.16 | 1.58 | compile-local destruction | accident |

Worker activity during Compile. Only these two worker threads record compile scopes:

| worker | work | window (ms) | busy (ms) | note |
|---|---|---|---:|---|
| tid 1 | Digest.OutputSets 1.06, then Digest.Solvers 51.9-62, then Digest.Movers 3.6 | 5.64-62.18 | 56.5-67.6 | one task (rigEvaluator.cpp:3500-3507; region stamps 2203/2661/2861); the join at 7496-7499 waits 0 ms |
| tid 2 | TapPrepare warmup | 8.08-49.92 | 41.8-50.5 | one `WorkDispatcher::Run` (3817-3829); ends within 0.01 ms of the WarmupJoin end in all six traces checked |

| metric | anim | baked | baked_par | dynamic |
|---|---:|---:|---:|---:|
| Compile wall (ms) | 105.16 | 118.13 | 104.97 | 137.88 |
| average thread parallelism | 1.84 | 1.84 | 1.81 | 1.74 |
| worker idle vs 31 pool threads | 97.0% | 96.9% | 97.0% | 97.2% |
| worker idle vs the 2 active workers | 53.2% | 52.5% | 54.0% | 57.2% |
| main alone with all workers idle (ms) | 43.0 | 50.4 | 46.0 | 64.5 |
| WarmupJoin wait (ms) | 9.53 | 12.79 | 11.16 | 15.32 |
| digest slack before DigestJoin (ms) | 11.09 | 16.36 | 13.72 | 62.08 |

Three internal parallel loops exist today, and their TBB children emit no trace events:
- PoseInfoPrefetch, a per-BFS-level `WorkParallelForN` (rigEvaluator.cpp:6559-6561);
- the Bake ladder resolve (bakedProgram.cpp:1761-1766);
- the input-table resolve (1946-1951).

Serial-vs-parallel reruns bound what is still serial inside them:

| phase | parallel (ms) | serial mode (ms) | what this implies |
|---|---:|---:|---|
| MoverDiscovery | 9.5-15.0 | 5.1 | none left to parallelize; parallel mode is slower from contention with the digest and warmup |
| Digest.Solvers | 51-55 | 33 | same contention effect (1.7x cold, 1.3x warm) |
| PoseInfoPrefetch | 9.4-15.1 | 31.8-33.2 | already parallel |
| Bake ladder | 9.5-9.9 | 17.2-17.5 | ~8.6-9.3 ms serial commit |
| Bake input table | 6.9-7.7 | 12.2-12.4 | ~6.5-6.9 ms serial commit |
| Bake walk | ~5.0 | 5.1 | fully serial |
| Bake step graph | ~5.5 | 5.9 | fully serial |

Dynamic compile adds 61.7 ms of main-thread exec calls in 30 scopes (rigEvaluator.cpp:7026-7030, 7348-7460), all
serialized by the single exec lane:

| scope | count | ms |
|---|---:|---:|
| `TapPrepare solverBatch` | 24 | 15.78 |
| `TapPrepare main` | 1 | 15.39 |
| `TapPrepare firstFramePose` | 1 | 13.75 |
| `WarmFirstFramePose` | 1 | 8.58 |
| guides | 1 | 4.70 |
| restFrames | 1 | 3.26 |

### 2.2 Baked frame, serial vs parallel

Medians over frames 2-16 (frame 1 in parentheses), traced:

| scope (us) | serial `biped_anim` | serial `baked` | parallel |
|---|---:|---:|---:|
| Evaluate@ frame | 1465 (4130) | 1520 (4067) | 1585 (4050) |
| BakedPrologue | 125 (2289) | ~115-159 | 119 (2223) |
| BakedRegion | 1121 (1572) | 1135 (1567) | 1120 (1564) |
| BakedEpilogue | 151 (252) | ~126-150 | 180 (250) |
| 16-frame total (ms) | 26.9 | | 31.2 |

Untraced, from six interleaved `--repeat` pairs (us/frame):

| run | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---:|---:|---:|---:|---:|---:|
| serial | 1234 | 1320 | 1077 | 1064 | 1078 | 1262 |
| parallel | 1366 | 1545 | 1358 | 1327 | 1226 | 1471 |

**Untraced, parallel loses ~200 us/frame.** The traced 4.4 ms total difference has two sources:
- One preempted frame, about 2.85 ms. In frame 12, worker tid 24 had ~2 us constraints take 124-142 us each and the
  skin take 1753 us.
- About 1 ms of extra profiler replay (`RigExecBakedReplayStepTimings`, bakedSchedule.cpp:2310-2360, called at
  bakedProgram.cpp:2699).

Region content by kind (us/frame, traced medians):

| kind | serial | parallel | note |
|---|---:|---:|---|
| Constraint (120) | 264-265 | 443-512 | 1.7-1.9x slower on workers |
| ComposeSubtree (200) | 52-55 | 92-98 | budget N/(4P) gives ~2 providers per step (bakedPose.cpp:1285-1287) |
| Solve (24) | 29-32 | 59-61 | |
| ProviderMatrix (252) | 23 | 41-43 | |
| SolverCommit (24) | 17-19 | 30 | |
| **pose sum** | **394** | **677-706** | span 394 serial vs 561 parallel |
| RevisionChunk (skin, 1 chunk) | 373-441 | 241-401 | self-parallel kernel, moverGraph.cpp:735 |
| Derived body_geo.extent | 227-254 | 250-271 | max 460; last step of every frame |
| InfluenceFold + ChainStatus | ~45 | ~35 | |
| **geometry sum** | **692-708** | **525** | 46-63% of the region |

Parallel schedule statistics. The executor is dependency-counting, not level-synchronous: `ParallelRun::RunFrom`
(bakedSchedule.cpp:1661-1747) handles successors, and seeding plus a single `Wait` sit at 1749-1830.

| statistic | value |
|---|---|
| clusters run / built | 75 / 76 (cluster 15 skipped after frame 1) |
| distinct levels | 48 (1-51; 45, 49, 50 empty) |
| steps per cluster | 1-36, median 5 |
| pose cluster duration p10/p50/p90 | 4 / 8 / 18 us; 59% under 10 us |
| max concurrent clusters | 5 |
| threads touching clusters | 15 per frame (10-17), 32 over the run |
| waitUs | mean 1.9, max 15, outlier 144; sum 143 us/frame (range 95-431) |
| concurrency histogram (frame 3, 1088 us span) | 904 us at width 1, 119 at 2, 21 at 3, 44 idle; average 1.11 |
| region time with no cluster running | 67 us (29 before the first cluster, 4 after the last) |
| main thread in region | runs ~10 of 75 clusters (80 us); parked in `dispatcher.Wait` ~1056 of 1120 us |

Bounds, all at measured or modelled costs:

| graph | work | critical path | bound | measured |
|---|---:|---:|---:|---:|
| clusters, measured durations | 1255 us | 998 us | 1.20x | 1.12x |
| clusters, pose only | 706 us | 471 us | 1.5x | 1.3x |
| clusters, modelled (report) | 586.0 us | 483.6 us | 1.21x | |
| steps, modelled (grain 0) | 586.4 us | 362.6 us | 1.62x | |
| steps, pose half modelled | 266 us | ~55 us | 4.8x | |
| steps, median serial step times | 1037 us | 816 us | 1.27x | |

Why parallel loses, in order of size:

1. **The geometry tail is serial by data.** The chain is: InfluenceFold (level 47, waits on 27 predecessor
   clusters), then one unpartitioned RevisionChunk (26,276 vertices, 137 influences, ready level 46 of 51), then
   RevisionFuse, ChainStatus, and the Derived extent (level 51). In all 32 baked frames, the first geometry step
   starts within 0-2 us of the last pose step. Every contiguous 4096-vertex range contains a vertex weighted to a
   neck, head or face joint that is final at level 44-46, so `PartitionAtBuild` reverts to one chunk
   (bakedGeometry.cpp:769-844).
2. **Level-packing creates false dependencies.** It bins unrelated same-level steps (bakedSchedule.cpp:654-699), and
   `BuildQuotient` (526-563) unions member predecessors without transitive reduction. The pose cluster critical
   path grows from ~55 us at step grain to 176.9 us at cluster grain. 221 of 330 cluster edges are redundant, which
   stops chain fusion (701-728) and absorb (730-756) from collapsing the 26-cluster spine/neck stretch (clusters
   34-59, levels 20-46).
3. **The spine/neck constraint chain carries per-cluster overhead.** The chain itself is a true read-after-write:
   each constraint propagates to all 27-53 descendants (bakedPose.cpp:1588-1616, 1655-1660). But it runs as 26
   clusters of one to three steps, with ~94 us of inter-cluster gaps and 13 thread switches along the realized
   48-cluster chain.
4. **Step bodies slow down ~1.65-1.9x on workers.** Likely causes: slot migration across cores, spinning workers,
   and per-step pinned USD reads that bypass the static-input cache on worker threads (bakedProgramImpl.h:311-340).
5. **The executor's heuristics are mis-tuned.**
   - The inline successor is simply the last ready one in cluster-id order (bakedSchedule.cpp:1719-1745), not the
     critical one.
   - `kSpawnCostUs` is 1.0 us (281), against 5-15 us of measured ready-to-start latency.
   - The cost table (192-211) was fitted on a 20-core box, and it prices serial range chunks ~11x too cheap (214).

Serial work around the region, per frame:

| block | where | us | serial by |
|---|---|---:|---|
| unscoped prologue (constraint tables, ribbon sources, clears) | bakedProgram.cpp:2515-2622 | ~100-105 traced; the microbench puts ~78 of it in the constraint-table reads | accident |
| PropertyChains / BakedInputs / BakedChainBase | 2589-2610; bakedGeometry.cpp:1758-1962 | 6 / 8-13 / 2-3 (frame 1: 807 / - / 1300) | data when warm; accident for the frame-1 cold cost |
| region lead: source steps + closure + skip loop | bakedSchedule.cpp:1832-1880 | 25-37 | partly data (WeightPacket composition) |
| BakedPublish | bakedPose.cpp:3541-3783, bakedGeometry.cpp:2719-2842 | 58-65 | data (it writes `B.resolvedInputs`, which geometry steps read) |
| epilogue replay | bakedSchedule.cpp:2310-2360 | 80 serial / 114 parallel | profiler only |

### 2.3 Dynamic frame

Median over frames 2-16 (frame 1 in parentheses): wall 8155 (15402) us, entirely on tid 0 (all 9,436 events). The
only work off tid 0 is in-kernel `WorkParallelForN` (moverGraph.cpp:471, 695, 815, 983, 2047), which is untraced.

| loop / phase | where (rigEvaluator.cpp) | us/frame | serial by |
|---|---|---:|---|
| PropertyChains | 10683 (loop 9142) | 8-14 (946 on frame 1) | accident, too small to matter |
| FirstFramePose (Warm + override pull) | 10759-10790 | 421 (459) | data: seeds every provider frame |
| FrameSeed map rebuild | 10876-10921 | 184 (204) | accident (baked ComposeSubtree does the same in 55) |
| pose walk: 24 solver batches | 11481-11650 (exec at 11591) | 2699 (3640); 24 ExecEvaluate = 2407, mean 100 | within a level: accident (one `_SolverBatch` per solver, 6968-7003); across levels: data |
| pose walk: 120 constraints | 11651-12152 | 1202 | the ordering is data; 11-14 us/step vs 2.5 baked is accident (per-frame stage reads, std::map commits) |
| AuthoritativeSnapshot | 12186-12240 (exec at 12219) | 1400 (19 on the 3 cache-hit frames) | accident: its inputs are final at the last solver commit |
| Publish (providers/joints/controls) | 12250 / 12289 / 12343 | 360 | PublishJoints must precede geometry (data) |
| GraphEvaluate (skin) | 13036 (runChain 12579) | 563 (outlier 7665 on frame 4) | data (reads snapshot taps and finalMatrices) |
| Derived extent | 13087-13183 | 478 | data, plus ~100-150 us of copy/compare overhead |
| unscoped | | 252 (649) = 3.1% | |

Walk shape: `SSSSSSSSSS C SSSSSSSSSS CCCC...CCCCC SS CCCCCCCC SS CCC...`, i.e. seven solver/constraint alternations.

Solver batches per frame: L0 x8, L1 x2, L2 x10, then L19, L20, L29 and L30 at one each. That is 24, not the 14 the
2026-09-13 report had.

The solver-batch cache never hits: there are 24 ExecEvaluate calls on every frame, including the repeat pass 9-16.
- `_SnapshotCache` capacity is 4 (rigEvaluator.h:850), against an 8-frame loop.
- The time-keyed auth cache evicts the smallest time, not the least recently used (12222), so it hits only on
  frames 14-16.

Per-request exec overhead dominates:
- `TF_DEBUG=RIGEXEC_TAP_TIMING` shows `ComputeWithOverrides` at a median of 86 us for 1-tap requests.
- The same solvers as baked Solve steps average 1.9 us.
- The cheapest request per frame on the repeat pass is 49-62 us, which caps the fixed cost per request at ~55 us.

## 3. State of unification

### 3.1 Baked mode: what is in the one step graph

These step kinds run in one DAG under one dependency-counting executor (edges swept at bakedProgram.cpp:2374 and
2391; kinds at bakedProgramImpl.h:572-591): ComposeSubtree, Solve, SolverCommit, Constraint, CommitDelta,
PropagateChunk, CommitApply, ProviderMatrix, SnapshotFinals, PoseInterpolator, VolumePlacements, WeightPacket,
InfluenceFold, RevisionStatic, RevisionChunk, RevisionFuse, ChainStatus, Derived.

IsBakeable accepts all 12 point-chain operations and both derived ones (bakedProgram.cpp:620-650). No point-mover
kind is missing from the graph.

Work that stays outside the graph and runs serially:

| work | where | per frame | could it be a step? |
|---|---|---:|---|
| interactive overrides (both sides of the chains) | bakedProgram.cpp:2584-2603 | small | no, it brackets the chains |
| property/math-mover chains | 2589-2600 (`_EvaluatePropertyChains`) | 6 us (807 on frame 1) | possible (the PropertyResult slot domain exists, bakedSchedule.cpp:1072); gain ~6 us |
| avar/ladder inputs, solver sources | bakedPose.cpp:2117-2209 | 8-13 us | partly (per subtree); small |
| stage xform frames | bakedProgram.cpp:2449-2510 | 0 on the biped (early return at 2459-2462: no plain-Xformable targets) | no: `UsdGeomXformCache` is not thread-safe |
| constraint weight/offset tables | 2515-2556 | ~78 us (microbench) | no: steps may not touch USD, and the closure needs the values first; fold or pin them instead (S10-A) |
| geometry prologue (base, topology, blend layouts, curvenet bind) | bakedGeometry.cpp:1758-1962 | 2 us (1300 on frame 1) | no: takes locks and uses the unsynchronized curvenet bind cache (spec §2.2) |
| source steps (RevisionStatic, pure WeightPacket) + closure | bakedSchedule.cpp:1846-1880 | 25-37 us | partly; WeightPacket-to-WeightPacket composition is a real order |
| pose + geometry publication | bakedProgram.cpp:2698-2777 | 58-65 us | no: publish writes `B.resolvedInputs` (bakedPose.cpp:3777-3781), which geometry steps read (bakedGeometry.cpp:1537, 2022) |

Three executors run over the same graph:
- serial (`RunStepsSerial`, bakedSchedule.cpp:1563-1629), the default (111);
- parallel (1661-1830), opt-in;
- the frozen-serial copy for background frames (frozenContext.cpp:6337-6384), serial by design because a frozen
  worker must not dispatch TBB work.

One kernel gate is missing. `RigExecSumBlendChannels`' `WorkParallelForN` (moverGraph.cpp:2047-2049) does not check
`RigExecFrozenSerialActive`, yet the frozen assembler calls it (frozenContext.cpp:5616). That contradicts the "four
gated launch sites" comment at frozenContext.cpp:3666-3671. It costs nothing on the biped (no blend shapes), but it
is a contract bug to fix regardless of scheduling.

### 3.2 Dynamic mode: separate loops

| # | loop | where (rigEvaluator.cpp) | baked step-kind equivalent |
|---|---|---|---|
| 1 | FirstFramePose exec + FrameSeed | 10759-10921 | BakedInputs + ComposeSubtree |
| 2 | pose walk (solver batches interleaved with constraints) | 11481-12152 | Solve/SolverCommit, Constraint, CommitDelta/PropagateChunk/CommitApply |
| 3 | AuthoritativeSnapshot exec | 12186-12240 | none needed (ProviderMatrix + WeightPacket + InfluenceFold replace its consumers) |
| 4 | publish loops | 12250-12353 | ProviderMatrix + BakedPublish |
| 5 | pose interpolators | 12430 (loop 3132) | PoseInterpolator |
| 6 | point-chain walk + derived | 12471-13228 | RevisionStatic/Chunk/Fuse, ChainStatus, Derived |

Only chain levels can run in parallel (13201-13228), and only when `_IsChainLevelParallelSafe` holds: at least 3
targets, no Curvenet, no phases (7950-8024). Chain levels are greedy contiguous runs of the topological order
(7672-7701), not longest-path levels. The biped has one chain, so loop 6 is serial.

Every dynamic phase already has a step-kind equivalent. Two things cannot become CPU steps:
- Exec requests for what the bake refuses (bakedProgram.cpp:240-700): connected pose providers, unsupported solver
  or mover types, animated or intervening Xforms above providers, time-varying skin inputs.
- The role of `_EvaluateDynamic` as the exec-authoritative parity oracle (`BakedWithParityCheck`, `cpuParityMode`,
  rigEvaluator.cpp:10166-10247).

No in-tree rig reaches any bake refusal (Section 5, S14).

## 4. Verified and corrected proposals: sequenced plan

Two proposals survived both lenses unchanged: S4 and S6. Five others were refuted as written, but the refuting lens
itself says a corrected form is legal. Those are marked "corrected", and the correction is a requirement. Gains are
listed per lens.

| # | action | area | deps-lens gain | gain-lens gain | planning gain | risk | effort |
|---|---|---|---:|---:|---|---|---|
| 1 | S1-corrected: move WarmupJoin, exec lane for guides/rests | compile | 18 ms | 20 ms (stage 1: 10-11) | **~18-20 ms cold baked compile** | stage 1 low, stage 2 medium | medium |
| 2 | S4: defer `_solverInputBatches`, parallel RestTimeVarying | compile | 8.5 ms | 8.3 ms | **~8.3 ms** (partly capped by the digest, 4.4) | low-medium | small |
| 3 | S6: Bake resolve/commit split, one edge sweep, overlap, async teardown | compile | 10 ms | 8 ms | **~8 ms cold, +4-5 ms warm** | tier A low, tier B medium | large |
| 4 | S13-corrected: 24 to 7 exec requests; snapshot on a worker | dynamic | 2.4 ms/frame | 2.0 ms/frame | **~2.0 ms/frame dynamic, ~7-8 ms dynamic compile** | medium | medium |
| 5 | S10-A narrowed: fold/pin constraint tables | baked eval | 0.05 ms/frame | 0.075 ms/frame | **~0.075 ms/frame (~5-6%)** | low-medium | small |
| 6 | S11: reduce, recluster, critical-path inline, width-1 gate | baked eval | 0.15 ms/frame | 0.1 ms/frame | **~0.1 ms/frame (0.05-0.15), parallel mode only** | low | medium |
| 7 | S9 narrowed: SIMD extent, drop point and topology copies | baked + dynamic eval | 0.03 ms/frame | 0.1 ms/frame | **~0.1 ms/frame, both executors** | low-medium | small |
| 8 | S7 variant: pre-warm frame-1 caches with a deferred join | first frame | 0.8 ms | 0.85 (2.0 with a deferred join) | **~0.8-2.0 ms once per compile** | medium | small |
| 9 | S5 part 1 only: digest as three tasks | compile | 2 ms | 8 ms (warm, settle digest) | **~4-5 ms once items 1-3 land; ~8-12 ms per edited evaluate** | low-medium | medium |

### 4.1 Compile: exec lane and join placement (S1-corrected)

**Stage 1 (can land alone).**
- Replace the unconditional `warmupDispatcher.Wait()` at rigEvaluator.cpp:6937-6943 with an idempotent
  `joinExecLane()` that keeps the `Compile.WarmupJoin` scope.
- Call it immediately before the ConnectedPoseTaps Prepare (~7294-7298) when `newConnectedPoseTaps` is non-empty,
  otherwise before the GuideTaps Prepare (~7384-7389).
- Do this in baked mode only (`deferExecPrep`, 3521). Dynamic mode keeps the early join for its 24 batch Prepares
  (7026).
- Rewrite the comments at 3799-3802 and 6939 to state the real rule. TapSet ctor/dtor touch only `context->clients`
  (tapSet.cpp:132-142), which Prepare never touches, and `newTaps` is already constructed on main during the warmup
  (5315).
- Add an `execCall()` wrapper with a debug assert, so a future exec call placed before the join trips.

Measured: the main-thread work between the old and new join sites is 11.4-13.4 ms, which covers the warmup tail.
Residual wait is 0-3.4 ms (8.9 in one slow outlier). Stage-1 gain: 9.5 / 11.6 / 11.2 ms on today's traces, 10.2-13.4
ms on fresh runs.

**Stage 2.**
- Construct `newGuideTaps` on main right after `warmupTaps` (solverArrayPaths is final at ~3762), and prepare it in
  the same worker task after the warmup.
- Once `RestTimeVarying` has frozen `newRestTapIds` and `newRestsMightVary`, hand the rest job to the same lane:
  Prepare, `Evaluate(restTime)`, and fill a task-private rest-frame map.
- Exec stays strictly single-lane.

The deps lens found two corrections that are **required**, not optional:

1. **Join the guide half before the commit at 7568, not after Bake.** Bake reads `E._solverArrayTaps`:
   bakedProgram.cpp:459 (IsBakeable), 2076-2080 (`guideOnlySolvers`) and 2136-2146 (solver-array publication). A
   guide Prepare failure clears it (rigEvaluator.cpp:7391-7394). With a late guide join, a failed-guide compile would
   still bake the guide solvers that today are dropped. Guides finish around t=60 ms, before Commit, so the earlier
   join costs almost nothing.
2. **Declare every lane-owned object before `warmupDispatcher` (~3818).** `newRestTaps` (7323) and the rest-frame
   map are declared after it. The early returns at 7360, 7373, 7450, 7489, 7669, 7736, 7744, 7760, 7780, 7798, 7824
   and 7838 would then destroy them before the dispatcher's destructor joins: a use-after-free. Follow the existing
   rule at 3493-3500 ("newDigest is declared before the dispatcher so it outlives it").

Also required or recommended:
- Move the DigestJoin after Bake. This is safe because `_ComputeStructureDigest` reads only `_stage` and
  `_profiler`, and it matters because after S1+S4 the digest is the next floor (4.4).
- On a late rest failure, reset `_bakedProgram`, set `_compiled=false` and return false. Accept that
  `_bakedProgramBuildAttempts` and `_bakeRefused` will already have moved.
- Every join sits inside the function-wide `TF_PY_ALLOW_THREADS_IN_SCOPE` (3413).
- Dispatching the warmup before Validate (after ~3598) is legal but worth ~0 once stage 2 lands.

Gain: WarmupJoin plus the main-thread exec calls (guides, restFrames prepare, RestFrames) total 20.7 / 25.4 / 21.3 ms
on today's traces and 20.0-35.3 ms on fresh runs. Subtract ~0-2 ms of Bake/lane contention. The lane (warmup, then
guides and rests) ends at ~60-80 ms, off main's path.

Verify, over at least 5 cold runs (compile sub-phases vary +-50%):
- `Compile.WarmupJoin` is ~0 at its old site.
- `TapPrepare guides`, `TapPrepare restFrames` and `Compile.RestFrames` run back to back on the warmup tid.
- PrepareRequests on tid 0 drops from ~14 to ~3 ms; Compile wall is ~85 ms.
- Dynamic compile is unchanged.
- Tests: testRigExecStageEdits x15 (GIL), testRigExecConstraints, testRigExecEpochRests, testRigExecBakedMode,
  testRigExecExampleParity, `rigExecPose --guides`, a new forced rest-failure test, and one
  `RIGEXEC_ENABLE_PARALLEL_EVAL=0` run.

### 4.2 Compile: stop building dynamic-only state in baked epochs (S4, verified)

1. With `deferExecPrep`, skip the `registerInput` and relationship walk (rigEvaluator.cpp:7005-7068) that fills
   `newSolverInputBatches`.
   - Its only reader is the notice handler (1796-1810). It is committed at 7567 and declared at rigEvaluator.h:936.
   - Nothing under `baked*`, `frozenContext`, `frameCache` or `backgroundScheduler` reads it.
   - `batch.dirty` and `batch.cache` are consumed only in the `_EvaluateDynamic` pose walk (11483-11604).
2. Build it in `_RealizeDeferredExecPrep` (10550-10590), which runs at the top of every dynamic, fallback and parity
   evaluate (10611; call sites 10172/10206/10220).
   - The build must come **before that function's early-return Prepare failure paths (10568-10583)**, because
     `_execPrepDeferred` is cleared first and the realization is never retried.
   - Prefetch `_CollectAttributeConnectionInputs` per ancestor with `WorkParallelForN`, then fill serially in order.
3. **Correction from the deps lens:** `registerInput` also reads `newPoseInputInfo[provider].attributes`
   (7053-7058), and `newPoseInputInfo` is not committed. The deferred build must either record those attribute sets
   at compile or recompute them with `_CollectPoseInputInfo`; otherwise dirty routing silently drops prims. The
   deferred build should use the committed `_jointSolverBinding` and `_solverDependencies`.
4. Add an "index absent" flag. Set and clear it at commit (7505-7567), and have `restorePreviousEpoch` keep it
   consistent. While it is set, the notice handler treats every batch as dirty. That is a no-op in practice, since
   no batch cache is populated before realization.
5. Make `RestTimeVarying` (7325-7337) a parallel any_of with an atomic early-out. **Do not** reuse the Bake ladder's
   `varying` flag: `newRestsMightVary` decides rest-tap placement before Bake runs (refuted by the deps lens).
6. Hoist ProviderSeed/ProviderClosure (7237-7304, excluding the ConnectedPoseTaps Prepare at ~7290) **in front of**
   the WarmupJoin wait. Placed after the wait, the hoist saves nothing. Once S1 stage 1 lands this is moot, because
   the join moves anyway.

Gain for the cold baked compile: 7-8 ms (index) + ~1.4 ms (RestTimeVarying 1.71-1.92 -> ~0.3-0.4 ms) + 0-0.7 ms
(hoist) = **~8.3-8.5 ms**. Per-frame gain is 0. The first dynamic-fallback frame after a baked compile pays 2-5 ms
more. Dynamic compile gains ~6-8 ms, not the claimed 11.4 -> 4 ms, because the ordered fill of ~23k
`std::map<SdfPath,set>` inserts stays serial.

Verify:
- Unscoped SolverBatches time drops from ~8.9 to ~1.5-2 ms; RestTimeVarying from 1.9 to ~0.3 ms.
- Tests, in both modes: testRigExecConstraints (four dirty-routing assertions), testRigExecBakedMode with a parity
  check after a solver input edit, testRigExecInteractive, testRigExecStageEdits.

### 4.3 Compile.Bake (S6, verified)

Tier A (low risk):
- **(a)** Run IsBakeable (bakedProgram.cpp:1498) as a task beside Bake.entry, dense_provider_slots and the ladder
  resolve. It must write to scratch reason vectors that are merged after the join, because `ctx.reasons = reasons`
  (1576) is shared with `refuse()` in dense_provider_slots (1668). About 0.8-1.0 of its 1.26 ms is realized.
- **(b)** Run `RigExecBakedBuildStepEdges`/`AssignStepCosts` once. Today they run at 2374-2375 and again via
  BuildSchedule (bakedSchedule.cpp:923-929). Keep the writer/reader interval tables alive between the two halves.
- **(c)** Build step labels lazily on main. They are consumed at run time by the replay (bakedSchedule.cpp:2312,
  2335) and by bakedVerify.cpp:811, so gating them on a Build-time flag is wrong.
- **(d)** Add sub-phase marks inside the_step_graph first; its 5.5 ms has no internal breakdown.
- **(e)** Incremental fusion/absorb (701-757) must keep the lowest-index-eligible merge order, because absorb
  eligibility reads the grown cluster cost. Credit this item to S11, which rewrites that code.
- **(f)** Destroy the retiring program (`outgoing`, rigEvaluator.cpp:10438) with `WorkRunDetachedTask`. First reset
  its `UsdStageRefPtr`s on main (bakedProgramImpl.h:1122, 2702), so the detached destroy cannot extend the stage's
  lifetime.

Tier B (medium risk, gated on byte identity):
- **(g-j)** Rework the commit halves of `CommitBind`/`Register` (bakedProgramImpl.h:2851-2868, 2566-2600):
  - Move their order-free part into the parallel resolve.
  - Assign `overrideIndex` by a prefix sum that starts from the counter's value at phase entry (it is a global
    running counter).
  - Fill `prims`/`named`/`rebuild`/`folded` from per-chunk vectors merged by sort-unique.
  - Build `overridableInputs` from (path, index) pairs.
  - `RigExecBakedComposeLadder` (1891) and `restChainVaries` (1868-1890) stay serial: they have a real
    parent-before-child dependency and cost ~0.5-1.5 ms.
- **(k)** Split `RigExecBakedBuildWalk` (bakedPose.cpp:51-977) into a parallel resolve and an ordered commit.
  - Pre-assign solver slots by walk position, and keep the rule that a lookup sees only earlier solvers
    (`BlendPointFrames` reads `B.solverIndex`, 289-293, 567-569).
  - Defer nativeSource dedup (583-605), deltaBasePaths (713-714), constraintArrays numbering, folds, binds and
    refusals to the in-order commit.
  - This is the highest effort for ~2 ms, so do it last.

Gain:
- Gain lens: **~8 ms cold**, with Compile.Bake going from ~30 to ~22 ms. The SdfPath-ordered set merges stay serial,
  so the ladder floor is ~4-5 ms rather than 3, and the input table ~3-3.5 ms rather than 2.
- Deps lens: 10 ms.
- Warm recompiles also gain 4.2 ms of retiring-program destruction and 1.19 ms of old-request teardown. Today's
  traces are all cold, so this part is not measurable from them.

Verify:
- A debug env check that serializes the built program and byte-compares it against a
  `RIGEXEC_ENABLE_PARALLEL_EVAL=0` build and against the pre-change build.
- For tier A, if landed before S11, the `RIGEXEC_BAKED_SCHEDULE_REPORT` output stays unchanged (634 / 1115 / 76 /
  330 / CP 483.62).
- Tests: testRigExecBakedSchedule, testRigExecBakedMode, testRigExecGeometryOpsBakedParity, testRigExecInteractive,
  testRigExecFrameCacheCones_Verify.

### 4.4 The next compile floor: digest and exec lane

After S1+S4+S6, the main thread has ~70 ms of work. Two single-threaded worker tasks then bound compile:

| lane | end today (ms) | splittable? |
|---|---:|---|
| exec: warmup 42-58, then guides + rests 10-14 | ~60-80 | no: exec is non-reentrant (tapSet.cpp:92-145); only exec itself can make it shorter |
| digest: OutputSets 1 + Solvers 52-62 + Movers 3.6 | 59-68 | yes: by segment (S5 part 1, <=5 ms) and by read prefetch (unproven; USD reads scale 1.3-1.7x worse with 3 readers) |

So after items 1-3, with DigestJoin moved after Bake, the realistic cold baked compile is **~70-80 ms, down from
105**. Further main-thread cuts (S2 memoization, S3 parallel MoverDiscovery) pay only after the digest is split.

On the warm edit path, the larger lever is the 23.5-24.9 ms inline settle digest (rigEvaluator.cpp:10510).
Splitting the shared `_ComputeStructureDigest` into three tasks plus a read prefetch saves ~8-12 ms per edited
evaluate.

### 4.5 Dynamic evaluate (S13-corrected)

**(a) One exec request per pose level (24 -> 7).** In the Kahn loop (rigEvaluator.cpp:6944-7082), build one
`RigExecTapSet` per ready level, union the tails, commit in stack-ordinal order, and keep a per-solver input
fingerprint for `_solverInputBatches` routing.

**Required correction:** split the level, or pin every named joint of every merged solver, whenever one solver's
live `computeRestFrame` override is an ancestor-or-self of a joint that another same-level solver names.
- `computeRestFrame` reads `parentRestFrame` from its namespace ancestor (computations.cpp:450-462;
  rigEvaluator.cpp:6980-6991), and solvers with no live joint push no pin (6999-7021). So a merged tail can change
  the rest another solver sees.
- Also verify that no live rest input sits under another same-level solver's written joints. Commits propagate to
  descendants (11318-11332) with no guaranteed edge, because frame inputs skip `rigExec:joints` (6468).
- The byte-identical prototype from 2026-09-13 predates `restInputs` (fad82b6, 2026-09-17), so it does not cover
  this case.
- On the biped the hole looks dormant: the finger chains share no joints, and L1 appears to be mirrored limbs
  (unconfirmed).

**(b) Snapshot on a worker.** After the last solver commit (~11640), assemble `jointOverrides` (12200-12218) on main
and run the snapshot `_taps->Evaluate` on a worker while main continues the CPU constraint tail.
- Only when `_connectedPoseTaps` is empty. `refreshPoseProvider` then returns at 11356, so the tail makes no exec
  call.
- **Required correction:** join before PublishProviders (12250), not PublishJoints, or buffer PublishProviders' side
  effects. Otherwise a snapshot failure leaks diagnostics that today's early return at 12230 never produces.
- Gate on `RigExecParallelEvaluationEnabled()` and `!RigExecFrozenSerialActive()`. SolverGuides (also exec) stays
  after the join.

Gain:
- (a) ~1.1-1.2 ms/frame. The fixed cost per request is at most ~55 us, not the prototype's 105 us.
- (b) ~1.0-1.1 ms/frame on cache-miss frames (overlap median 1.13 ms, minus dispatch); 0 on the 3 cache-hit frames.
- Together **~2.0 ms/frame**, taking the median from 8.2 to ~6.2 ms. Dynamic compile saves ~7-8 ms (24 -> 7 batch
  Prepares).
- The biped runs dynamic only when forced. The value is for refused rigs and the parity oracle.

Verify: 24 -> 7 ExecEvaluate per frame; AuthoritativeSnapshot on a worker row, overlapping the post-L30 constraints;
testRigExecConstraints; `BakedWithParityCheck` and `cpuParityMode` with 0 mismatches and identical diagnostics.

Not a scheduling change, but the largest single dynamic item: size `_SnapshotCache` (rigEvaluator.h:850, capacity
4) and the time-keyed auth cache (12222, evicts the smallest time) for looped playback. That is ~3 ms/frame on repeat
passes, at some memory risk.

### 4.6 Baked frame

**S10-A narrowed (item 5).**
- Classify the constraint tables (bakedPose.cpp:684-705) at Build. `sourceWeights` and the offsets are already
  `foldShape`d unconditionally (rebuild + named + prims, bakedProgram.cpp:1451-1458), so folding them as constants is
  edit-safe.
- Read the remaining varying tables through pinned `UsdAttributeQuery`, in a `WorkParallelForN` gated the same way
  Build gates it (bakedProgram.cpp:1766-1770). The reads stay in the prologue, because the closure needs them before
  the region (bakedSchedule.cpp:1359-1379).
- **Do not** fold `poleVectorWeights`. It is `foldShape`d only when the attribute exists (bakedPose.cpp:783-786), so
  a pole table authored later would not trigger a rebuild.
- Add profile scopes around constraintArrays/stageFrames first: the ~100 us attribution is inferred.
- Microbench: raw reads 78 us/frame, pinned queries 21, folded ~0-5. **~0.075 ms/frame untraced, both executors.**
- Drop two items from the original S10: the retained XformCache breaks the invariant at bakedProgram.cpp:2451-2457
  (`SetTime` never drops stale queries), and reading inside a step is illegal (steps may not touch USD).

**S11 (item 6).** All changes are in bakedSchedule.cpp:
- **(0)** Re-run `RIGEXEC_BAKED_SCHEDULE_CALIBRATE` on this 32-thread box, and split the RevisionChunk cost row (214)
  into whole-kernel and range rows. Modelled cluster costs are 3-10x below realized ones.
- **(1)** Transitively reduce the quotient (221 of 330 edges are redundant). Reduce a copy, not in place, because
  `SkinCosts` reads intra-revision `step.preds` (bakedGeometry.cpp:2587-2610), and re-apply after each merge.
- **(2)** Use dependency/linear clustering instead of level bins (654-699), and exclude isSource steps from cost and
  from the critical path.
- **(3)** Run the successor with the highest bottom level inline (1719-1745), and raise `kSpawnCostUs` (281) to ~8 us.
- **(4)** Record the modelled serial/CP ratio at Build, and use the width-1 walk when it is below ~1.3x.
- **(5)** Flip the default (111) only after untraced `--repeat` shows parallel no slower on every rig in the spec
  table.

Deps lens: legal, because `B.closed` is forward-closed (bakedSchedule.cpp:1210-1220, 1473-1477), so a reduced edge's
intermediate cluster is always closed.

Gain lens: the geometry tail (60-65% of the region) is out of reach. An idealised list schedule of the pose half gives
~145-225 us against ~395 serial, so the expected gain over serial is **~0.05-0.15 ms/frame**, and only after the
change first recovers today's ~200 us/frame untraced loss. Drag cones get coarser, so measure a neck drag.

This is the enabling step for "one loop, as parallel as possible". Without it the default must stay serial.

**S9 narrowed (item 7).** Changes to `Derived` (bakedGeometry.cpp:2023-2110):
- Drop the two 315 KB point copies: `basePoints.assign` at bakedGeometry.cpp:1609, and the copy-assign into
  `auxPoints` at moverGraph.cpp:2584.
- Drop the `faceVertexCounts`/`faceVertexIndices` reads and copies (moverGraph.cpp:2585-2587, ~525 KB), which
  RecomputeExtent never uses.
- Make `RigExecComputeExtent` (rigExecMath/geometryKernels.cpp:302-323) branchless or SSE. Microbench: 64-73 us ->
  20-22 us.
- **Keep the value compare.** `revisionsExecuted` is counter-parity-bearing (tests/testRigExecBakedSchedule.cpp:
  1175-1183, 1348-1350).
- **Do not** parallelize the reduce: 7 grains of ~3 us lose to fork/join.

Untraced, Derived costs 140-168 us/frame, not the 70-76 us the comment at 2044-2045 claims. Gain **~0.1 ms/frame** in
both executors (the deps lens gives 0.03 for the narrower copy-only subset).

In dynamic mode, the analogous target is the deferred-cache copies at rigEvaluator.cpp:13163-13167 (~840 KB). The Vdf
revision-node copies (moverGraph.cpp:242-265, 2825-2829) are required for buffer ownership and stay.

**S7 variant (item 8).** Frame 1 pays for the PropertyChains binding (~700 of 807 us) and BakedChainBase (1300 us).
- The binding build (rigEvaluator.cpp:9017-9114) reads only the committed `_propertyChainOrder`/`_propertyChains`
  and the stage. Dispatch it at Commit so it hides under Bake.
- For the geometry part, resolve at a numeric time such as `ctx.probe`, not Default: single-sample attributes pass
  the cache admission test (moverGraph.cpp:1706-1712) and would be mis-seeded.
- Resolve against a fresh `RigExecResolvedInputs` that holds only the current interactive overrides, not the stale
  `E._resolvedInputs` (bakedProgram.cpp:1510; see rigEvaluator.cpp:9741-9760).
- Join lazily at the first Evaluate/Settle rather than at the end of Compile. An end-of-Compile join is fully
  exposed, because `_RebuildBakedProgram` is Compile's last statement (7932).

Gain: **~0.8 ms net per load with an end-of-Compile join, ~2.0 ms with a deferred join; 0 per steady frame.**

### 4.7 Sequencing and cumulative effect

1. S1 stage 1 (small, low risk), then S4, then S1 stage 2 with both corrections and DigestJoin after Bake. Cold
   baked compile goes from ~105 to ~80 ms.
2. S6 tier A, then tier B (g-j) behind the byte-identity check, with S6 (k) last. Compile goes from ~80 to ~70-75
   ms and is then bounded by the digest and exec lanes (4.4). Warm recompiles gain another 4-5 ms from (f).
3. S5 part 1 (digest as three tasks). Then re-measure whether S2's memoization pays on wall time.
4. Frame: S10-A and S9-narrowed, which help both executors. Together ~0.17 ms/frame untraced, ~12-15% of a ~1.2 ms
   untraced frame. Independent of the compile work.
5. S11, gated on the calibration re-run and an untraced A/B. Only after that, consider S12 again (Section 5).
6. Dynamic: S13 (a) and (b) with their corrections; cache sizing separately.
7. Fix the `RigExecSumBlendChannels` frozen gate (moverGraph.cpp:2047-2049) regardless.

Methodology for every A/B:
- Compile: at least 5 cold runs. Sub-phases vary +-50% (MoverDiscovery 9.0-18.2 ms, PoseInfoPrefetch 10.5-18.5 ms).
- Frames: untraced `--repeat` with `RIGEXEC_BAKED_STEP_TIMING`. One preempted frame moved today's 16-frame total by
  2.85 ms, and the replay adds ~1 ms.
- Never compare a traced parallel run to a traced serial run on totals.

## 5. Refuted proposals (do not re-propose)

| id | proposal | refuted by | refutation | what survives |
|---|---|---|---|---|
| S1 | join warmup late; guides/rests on the exec lane; join after Bake | deps | Bake reads `_solverArrayTaps` (bakedProgram.cpp:459, 2076, 2136), which a guide failure clears (rigEvaluator.cpp:7391-7394) before the 7568 commit. Lane-owned objects declared after the dispatcher are freed on early returns | corrected form is item 1 (18-20 ms) |
| S2 | memoize `_CollectPoseInputInfo`, hoist PoseInfoPrefetch | gain | PoseInfoPrefetch finishes 10-21 ms before the warmup in all six traces, so cutting it only lengthens WarmupJoin. Warmup duration does not track prefetch duration (warmup 41.8/50.5/50.35/42.7 ms vs prefetch 10.9/10.5/18.5/16.4). `WorkWithScopedParallelism` could stop main from helping the critical warmup | memoization is sound CPU work; revisit after the digest floor moves. It saves ~20-25 ms only in kill-switch mode. Deps lens: the memo must handle connection cycles (rigEvaluator.cpp:1301-1302), not be built bottom-up |
| S3 | parallel-for MoverDiscovery beside SolverJointBinding/Dag | deps + gain | legal, but main already idles 9.5-15.3 ms at WarmupJoin, so the 7 ms is absorbed, and extra USD readers contend with the critical warmup. The proposal's error precedence is also wrong: OutputCheck (4494-4580) sits between discovery and binding | same as S2: revisit only after the warmup/digest floor moves |
| S5 | digest as 3 tasks + prefetch + reuse settle digest | gain | cold gain is 0 today (DigestJoin waits 0, 14-16 ms slack). The reuse skips the off-path worker digest, not the 24 ms inline settle digest (rigEvaluator.cpp:10510). `_ApplyDerivedStartFrames` re-authors on every biped compile (10 derived start frames), so "authored nothing" never holds, and a map-equality flag is unsound | segment split (item 9), applied to the settle digest too |
| S7 | pre-warm frame-1 caches after Build, join at end of Compile | deps + gain | no exec-lane tail exists to hide it (workers idle after ~68 ms; Bake is last). A stale `E._resolvedInputs` can seed the epoch topology cache with a released or missing override. Default-time reads mis-seed single-sample attributes | item 8 variant |
| S8 | skip the warmup when exec is live; detach the exec lane from Compile | deps + gain | a detached lane runs exec while the host writes the stage. `PrepareStageChange` resets every `_request` and `_system` synchronously on the editing thread (tapSet.cpp:101-119) while the lane holds raw pointers (224-251): use-after-free. The frozen-context purity table says exec "cannot run concurrently with stage edits" (frozenContext.cpp:3575-3578). All evaluators on a stage share its exec system (tapSet.cpp:70-88). `_SettleEpoch` reads `_epochRestFrames` and calls `_restTaps->Evaluate` on every frame (9400, 9411-9421). The digest lane ends as late as the exec lane anyway | stage A (skip the warmup when warm) is legal but unmeasured: no trace in reports/ has a warm recompile |
| S9 | extent: execution stamp, fused partials, no copies, dynamic copy removal | deps | the stamp breaks `revisionsExecuted` counter parity. The seed copy at bakedGeometry.cpp:1485 is the in-place skin kernel's input (moverGraph.cpp:632-662). The Vdf revision-node copies are required for buffer ownership. Fused partials are valid only if the skin is last, full-strength and executed. Parallel min/max is not bit-identical without per-grain seeding from `points[0]` | item 7, narrowed |
| S10 | prologue into the graph (scopes, constant tables, retained XformCache, geometry prologue and publish as steps) | deps + gain | a retained XformCache goes stale on target xform edits that only bump programStamp (rigEvaluator.cpp:1737-1746). In-step USD reads violate the step contract, and the closure needs the values first. The geometry prologue takes locks (spec §2.2). Publish writes `B.resolvedInputs` while geometry steps read it. stageFrames costs 0 on the biped | stage A items (1)-(2), narrowed, as item 5 |
| S11 | recluster + CP inline + width-1 gate | gain | wrong baseline: untraced, parallel loses ~200 us/frame. The geometry tail is out of reach. Expected gain is 0.05-0.15, not 0.2 | the whole proposal, as item 6, with recalibration first |
| S12 | permuted skin partition by influence ready level | gain (measured) | a forced contiguous 512-vertex cut (27 of 52 blocks early) made the parallel region **+420 us slower** (1095-1186 -> 1506-1626 us): early chunks delayed the critical pose chain by ~514 us and shortened the tail by only ~80 us. The true serial skin is 2.18 ms, not ~200 us, and the whole kernel already reaches ~9x. Serial mode: +2.1 ms. Same-level merging would make a ~500 us serial chunk. Deps lens also found a latent re-cut hazard (bakedGeometry.cpp:1799-1818, frozenContext.cpp:4869-4879: keys are recomputed while declared reads stay at the Build keys) | nothing, until an executor with critical-path priority exists. Check the latent re-cut race independently |
| S13 | merge same-level solver batches; snapshot on a worker | deps | merging tails leaks a `computeRestFrame` override to descendants named by another same-level solver. A join after PublishProviders changes failure-path diagnostics | item 4, with the stricter split rule and the earlier join |
| S14 | dynamic mode as the step graph with exec-backed steps | gain | 0 ms on every measured workload: all 28 RigExecRoot stages in examples/ bake (`--require-baked`), and only fixtures reach the refusals (spec:833). The parallel executor loses today. A hybrid compile would pay the exec prepares plus Bake (>138 ms). The "rest" refusals are one-rest-per-slot limits that exec steps cannot fix. Deps lens: legal only with a GIL guard around the region (Evaluate at rigEvaluator.cpp:10108 has none, and tapSet.cpp:271-278 can prepare lazily) and a spec amendment for §2.2/§2.6 | a product decision, not a scheduling task; revisit when a shipped rig refuses to bake |

The data also rules out two ideas that were never formal proposals:
- **Level parallelism in the dynamic pose walk.** The pose DAG has 44 levels, 1-6 clusters wide, with a critical path
  of ~720 of ~780 us. CPU-side level parallelism is worth at most ~60-100 us on the biped, and concurrent exec is
  impossible (single `ExecUsdSystem`).
- **A parallel extent reduce.** Seven 4096-point grains of ~3 us each lose to fork/join/wake; a serial SIMD loop is
  faster.

## 6. Open questions

1. Is `ExecUsdSystem::PrepareRequest` safe on distinct requests of one system, or serialized only by rigExec
   convention (rigEvaluator.cpp:3810-3815)? If it is reentrant, guides/rests could overlap the warmup instead of
   chaining behind it, and dynamic compile's 61.7 ms of prepares could be split.
2. How much of the cold 42.7 ms warmup is one-time plugin load and network compile, and how much is request
   scheduling? Warm, it is 14-18 ms.
   - Could the plugin-load part start at evaluator construction?
   - In baked mode, would preparing only the guide and rest requests (no throwaway warmup) compile the same network
     faster?
3. How many threads do the warmup, PoseInfoPrefetch, the Bake resolves and the skin kernel actually use? TBB children
   emit no trace events. Does main steal exec's cold-compile tasks while it waits in a non-isolated
   `WorkParallelForN`? (OpenUSD workTBB/loops_impl.h:53 isolates cancellation, not work.)
4. How exactly do the 5.5 ms of the_step_graph and the ladder/input-table commits split? Answering this needs
   sub-phase marks in bakedSchedule.cpp and bakedProgram.cpp (S6 d).
5. Why are pose step bodies 1.65-1.9x slower on workers? Candidates: slot cache migration, false sharing between
   adjacent PoseFin/PoseBase slots, spinning workers, or USD query reads on worker threads. Untraced
   `RIGEXEC_BAKED_STEP_TIMING` per body in both modes would separate them.
6. What caused the frame-12 stall on tid 24 (~135 us per 2 us step, skin 1753 us): OS preemption, or
   oversubscription (32 TBB workers on 32 logical CPUs plus exec threads)? The continuation-on-same-thread policy
   (bakedSchedule.cpp:1741-1745) keeps the whole critical chain on the stalled thread.
7. Can two same-level solvers name the same joint with different `restInputs` states, or name an ancestor/descendant
   pair (the S13 correction)? And are the `rigExec:*` tokens read per frame in dynamic constraint steps
   (solverMode, poleVectorMode, evaluationMode, rotationOrder, aimAxis, worldUpType; rigEvaluator.cpp:11770-11958)
   uniform and safe to hoist?
8. Why did dynamic frame 4 spend 7.7 ms in GraphEvaluate (normally 0.6-1.1 ms): a schedule rebuild, the topology
   cache, or thread-pool spin-up? Separately, `TAP_TIMING` logged 618 `ComputeWithOverrides` calls against ~413
   expected, and the ~205 extra are unattributed.
9. Does parity mode or any dynamic-fallback frame read `_solverInputBatches` before its first notice (S4)? The deps
   lens found no such path; a targeted test should confirm.
10. Should dynamic mode stay the independent exec-authoritative reference for `BakedWithParityCheck`, or may it be
    re-based on the step graph (S14)? This is a product decision.
11. Should the cost table be re-fitted on this 32-thread box before any cluster-shape change is judged? The grain and
    the compose budget already scale with `WorkGetConcurrencyLimit()=32`, while the costs came from a 20-core fit.

## 7. What landed the same day (2026-09-23, working tree on top of 64d6a5f)

Items 1-4 of Section 4 (S1 stages 1+2, S4, S6 tiers A and B g-j, S13-corrected) and the `RigExecSumBlendChannels`
frozen gate were implemented, each gated by a deps reviewer and a parity reviewer, full ctest (192/192) and
byte-identical `--joints-out`/`--pose-out` in baked, dynamic, parity, `--guides` and `RIGEXEC_ENABLE_PARALLEL_EVAL=0`
runs on Biped_anim (1-8), Biped and Biped_layered against a pre-session build. S6 (k) was left out as specified.

Cold compile on the biped, medians of 7 (baked) / 3 (dynamic) runs, ms:

| phase | before | after |
|---|---:|---:|
| Compile (baked) | 109.6 | 80.0 |
| Compile.SolverSchedule | 40.9 | 20.9 |
| Compile.Bake | 30.4 | 19.4 |
| Compile.PrepareRequests main-thread work | ~14 | ~0.8 (the phase now holds the guide-half join) |
| Compile.WarmupJoin | 11.7 (inside SolverSchedule) | 15.6 (at GuideTaps: warmup tail + guides; the new critical path) |
| Compile (dynamic) | 124.4 | 96.2 |
| dynamic frame, untraced `--repeat 20` | ~5.8 ms | ~4.1 ms (24 -> 7 exec requests) |
| baked Evaluate.Run (16 frames, traced) | 21.5 | 21.2 (untouched) |

Compile's critical path is now: warmup end, guides (~4 ms), Commit, Bake, with the digest joined after Bake and
never waited on. The next floor is Section 4.4: the single-task exec warmup and the single-task digest.

Follow-ups not done: S6 (k) the walk split; S5 part 1 (digest as three tasks); S10-A and S9-narrowed frame
removals; S11 recluster; a unit test that forces a solver-request split under S13's rules 1-4; a debug comparison
of the parallel and serial program digests (S6B); a runtime guard for S6A's append-only sweep invariant.

## 8. Unified program plan (2026-09-23)

The design spec is `docs/specs/unified-program.md`. It answers two product requirements:
- push the compile floor down;
- make dynamic mode the same single loop graph as baked mode, so that it is parallelizable, editable and sparse.

Every proposal went through a deps review and a feasibility/gain review. Survivors appear in the spec's §7 exactly
as written. Refuted proposals appear only in their corrected form. Proposals refuted outright are listed with their
refutations in the spec's §8, next to this report's §5.

Architecture in brief:
- One program type (`RigExecBakedProgram`) serves Baked and Dynamic mode. Dynamic runs it behind
  `RIGEXEC_DYNAMIC_RUNS_PROGRAM` until the default is flipped, which needs the user's confirmation.
- `_EvaluateDynamic` stays, unchanged, as the explicit `ExecReference` parity oracle.
- Exec steps are added only where CPU steps cannot express the work. The reachable cases are ExecSeed for ladder
  refusals, and ExecPull + ConnectedCommit for connected providers.
- Each stage has one exec lane token. It is re-entrant for its owner scope, taken after the GIL is released, and
  held around every exec site.
- Background and frozen workers never run exec.

Milestones, each ending in a byte-identical parity gate:

| milestone | items | cumulative target |
|---|---|---|
| M0 foundations | F0a bench, F0b GIL guard + dead code, F0c sparse-plan order, X0 ExecReference mode | no behaviour change |
| M1 compile floor | CF1 warmup diet, CF2a 3-task digest, CF2b Solvers memo, CF3a PoseInfo SCC, CF3b/c (gated) | cold baked 87.5 -> 68.5 -> 62.4 (measured) -> ~55-60 ms (est.) |
| M2 edit tiers | EP1a, EP1b, EP2a, EP2b, SU1a, EP3a, EP3b, EP1c, EP4a, EP4c, EP4b, EP5' | see the edit targets below |
| M3 dynamic = program | HP-D0, HP-D2, HP-D-flip (needs user confirmation), HP-D1 | dynamic compile 96 -> baked floor; frame ~4.1 -> ~1.2 ms |
| M4 CPU gap closures | HP2'a checkpoint, HP2'b skin layout, HP2'c intervening Xform | 3 refusal classes leave the fallback |
| M5 exec sources | EX1', EX2a ExecSeed, SU2a | ladder-refused rigs run the program |
| M6 exec interior | EX2b, HP1' connected providers | connected-provider rigs run the program |

M2 edit targets:
- non-avar value edit: 32-35 -> < 5 ms
- rest edit: 57 -> <= 5 ms
- structural edit: 87.6 -> <= 63 ms
- broken stage: 54-68 ms/frame -> < 1 ms

Deferred, each with a stated trigger: CF1-s2, EP2c, SU1b, SU2b, EX3', HP2'd.

Dropped:
- HP4 (PreferExec);
- EP2's author-the-diff;
- SU1's phased-read retention;
- EP4's folded promotion and computation-override placement;
- HP1's ExecSolve, ExecGuide and ExecSnapshotSubset;
- HP2's stacked-batch split.

The first three items to implement are F0b, X0 and CF1.

### M0 gate (2026-09-23)

M0 (F0a, F0b, F0c, X0) measured on the working tree after all four items, with the same build throughout.
Scratch outputs are under the workflow's `scratchpad/impl/M0_*` and `scratchpad/gate/M0/`.

G-parity:

| check | result |
|---|---|
| build (`bin\build_rigexec.bat --no-test`) | PASS |
| ctest, full, `-j 8` | PASS, 216/216. That is 192 at the start plus 24 new: X0's 23 `*Reference` entries and F0b's `test_rigexec_gil`. |
| `bin/run_python_tests.sh` | PASS, 17/17 |
| `--pose-out`/`--joints-out`: 42 stages × {baked, dynamic, parity, guides, noparallel} against the starting build | PASS: 335/335 files byte-identical, 0 missing, no rc change in summary.tsv. The outcomes match the start: 140 ok, 55 no-joints, 15 compile-failed (three stale examples). |
| `--mode reference` (X0) on all 42 stages against the starting build's `--mode dynamic` | PASS: 67/67 `.pose`/`.joints.usda` files byte-identical, and the per-stage rc matches the dynamic run. |
| run logs (`.log`, 210) | They differ only in sanctioned ways once the outdir path is normalized. F0b deleted the stderr `RIGEXEC_MEASURE` block, which removes the `RIGEXEC_MEASURE baseOverrides=…` line and its `  type … count=N` histogram from dynamic and parity runs; the interleaved fragments of those lines also split stdout lines in the old logs. The line number in the RigExecSphereWeight purpose warning moved (3816 → 3822) because source above it moved. After normalizing those, 0 of 210 logs differ. |
| `BakedWithParityCheck` (`--mode parity`) | PASS: `bakedParityMismatches=0` on all 936 frames (39 compiling stages × 24, the bipeds included) |
| `RIGEXEC_BAKED_VERIFY_CONES=1`, serial and parallel, Biped / Biped_anim / Biped_everything, frames 1..8 | PASS: rc 0, 8/8 valid frames, 0 "baked cone mismatch", `bakedParityMismatches=0` in all 6 runs |
| usdview runners (23) | Not at the memory baseline. Passing: 14 runners. At baseline: curvenet fails (the centre pick hits `/CurvenetAsset/Rig/Solvers/BendChain`), and avars, execstack and ikfk_opacity skip because their generated stages are missing. touchpose now passes. Three runners fail that the baseline counts as passing, and each fails the same way when rerun: draw ("a left-press on the model added no knot"), noodles_rename (`view.nodes` is empty) and viewcube ("entering the cube arms the highlight at once: None"). No binary of the starting build was kept, so these three failures are not yet attributed. The failures are picking and hover assertions, and none of the M0 items touches imaging picking or UsdNoodles, but F0b's GIL release in Evaluate is the one M0 change that alters Render-time threading. |

G-perf, compile (cold rigExecPose on Biped_anim, frames 1..8, medians in ms; the start column is `gate/base_bench_*.txt`,
taken at workflow start on the same machine):

| scope | baked start (7) | baked M0 (7) | dynamic start (3) | dynamic M0 (3) |
|---|---|---|---|---|
| Compile | 83.12 | 80.52 (min 75.22) | 111.11 | 107.48 |
| Compile.WarmupJoin | 16.46 | 16.56 | 14.19 | 11.62 |
| Compile.SolverSchedule | 19.40 | 19.48 | 42.38 | 39.48 |
| Compile.PrepareRequests | 17.19 | 17.39 | 42.58 | 44.32 |
| Compile.Bake | 20.17 | 19.63 | 0 | 0 |
| Compile.DiscoverValidate | 16.87 | 16.04 | 17.48 | 16.09 |
| TapPrepare warmup | 47.78 | 44.40 | 47.12 | 43.53 |
| Digest.Solvers | 58.42 | 55.54 | 57.03 | 57.65 |
| Evaluate.Run | 21.45 | 21.05 | 0 | 0 |

The M0 items make no claim about performance, and both modes are within run-to-run noise of the start.

G-perf, edits: this is the F0a baseline, `build/benchEditLatency.exe` on Biped_anim, baked, held=10, 8 rounds, each
scenario in its own process. The table gives medians in ms.

| id | wall | settle | compile | bake | run | baked |
|---|---|---|---|---|---|---|
| ref-steady | 0.52 | 0 | 0 | 0 | 0.52 | 8/8 |
| ref-time | 1.33 | 0 | 0 | 0 | 1.32 | 8/8 |
| T0-leaf | 1.53 | 0 | 0 | 0 | 1.52 | 8/8 |
| T0-hips | 1.44 | 0 | 0 | 0 | 1.42 | 8/8 |
| T1a | 1.32 | 0 | 0 | 0 | 1.31 | 8/8 |
| T1b | 1.49 | 0 | 0 | 0 | 1.49 | 8/8 |
| T1c-guide | 31.97 | 28.19 | 0 | 0 | 3.83 | 8/8 |
| T1c-mover | 32.66 | 29.12 | 0 | 0 | 3.56 | 8/8 |
| T1d | 33.69 | 29.05 | 0 | 0 | 4.05 | 8/8 |
| T2 | 55.50 | 28.83 | 0 | 19.55 | 4.85 | 8/8 |
| T3 (jointElements) | 82.00 | 77.05 | 49.76 | 19.62 | 4.58 | 8/8 |
| T3-weights | 53.48 | 28.98 | 0 | 19.94 | 4.72 | 8/8 |
| T4-add | 89.65 | 81.47 | 54.04 | 23.05 | 8.16 | 4/4 |
| T4-remove | 105.77 | 101.75 | 75.37 | 22.64 | 4.46 | 4/4 |
| T5 (broken) | 61.20 | 61.19 | 30.61 | 0 | 0 | 0/8 |
| T5-repair | 82.30 | 75.76 | 49.52 | 19.07 | 6.53 | 1/1 |

These are the numbers the M2 edit targets are measured against: T1c/T1d < 5 ms, T2 ≤ 5 ms, T4 ≤ 63 ms, T5 < 1 ms.

Verdict: M0's G-parity is green on the build, ctest, python, pose parity, reference parity, BakedWithParityCheck and
cone verification. The usdview runner check is open, because draw, noodles_rename and viewcube have not been
attributed. G-perf is recorded.

### M1 gate (2026-09-23)

M1 (CF1, CF2a, CF2b, CF3a landed; CF3b skipped because its own F0a gate was not met; CF3c not attempted) measured on
the working tree after those items, with the same build throughout. The pose comparisons are against both
`gate/base/` (the workflow's starting build) and `gate/M0/` (M1's starting build). Scratch outputs are under the
workflow's `scratchpad/impl/M1_*`, `scratchpad/impl/M1gate_*` and `scratchpad/gate/M1/`.

G-parity:

| check | result |
|---|---|
| build (`bin\build_rigexec.bat --no-test`) | PASS |
| ctest, full, `-j 8` | PASS, 216/216. The count is the same as at M0 because CF3a's mutual `default:space` cycle fixture was added inside testRigExecDefaultSpaces. |
| `bin/run_python_tests.sh` | PASS, 17/17 |
| `--pose-out`/`--joints-out`: 42 stages × {baked, dynamic, parity, guides, noparallel} | PASS: 335/335 files byte-identical to `gate/base/` and 335/335 to `gate/M0/`, 0 missing, no rc change in summary.tsv. Outcomes: 140 ok, 55 no-joints, 15 compile-failed, the same as at the start. |
| `--mode reference` on all 42 stages | PASS: 67/67 `.pose`/`.joints.usda` files byte-identical to the starting build's `--mode dynamic` and to M0's reference run; per-stage rc unchanged. |
| run logs (`.log`, 210) | Against `gate/M0/`, with the outdir path normalized, 205 are identical. The other 5 are the 14_VolumeConstrainedSweep runs, where the only change is the source line in the RigExecSphereWeight purpose warning (3822 → 4823), because M1 added code above it. Against `gate/base/`, the only other differences are the M0-sanctioned removal of `RIGEXEC_MEASURE` (F0b). No M1 item changes log text. |
| `BakedWithParityCheck` (`--mode parity`) | PASS: `bakedParityMismatches=0` on all 936 frames (39 compiling stages × 24, the three bipeds included), and no "parity leg INVALID" |
| `RIGEXEC_BAKED_VERIFY_CONES=1`, serial and parallel, Biped / Biped_anim / Biped_everything, frames 1..8 | PASS: rc 0, 8/8 valid, 8/8 baked generations, 0 "cone mismatch" and `bakedParityMismatches=0` in all 6 runs. The poses are byte-identical to M0's cone runs. |
| M1 item verifiers (`RIGEXEC_VERIFY_DIGEST_SPLIT=1 RIGEXEC_VERIFY_DIGEST_MEMO=1 RIGEXEC_VERIFY_POSEINFO=1`), baked and dynamic, on the three bipeds, 02_TwoBoneIkLeg, 05_TwistRibbonSpine and 09_PropertyMathMovers | PASS: rc 0 in all 12 runs, no fatal, and the poses are byte-identical to the gate run |
| usdview runners | Not run by this gate. They are still open from M0: draw, noodles_rename and viewcube are unattributed. |

G-perf, compile (cold rigExecPose on Biped_anim, frames 1..8, medians in ms). There are two passes of 7 baked runs in
this session. No binary of the M0 build was kept, so the M0 column is the M0 gate's own measurement from earlier in the
workflow, on the same machine.

| scope | baked M0 (7) | baked M1 pass 1 (7) | baked M1 pass 2 (7) | dynamic M0 (3) | dynamic M1 (3) | dynamic M1 (7) |
|---|---|---|---|---|---|---|
| Compile | 80.52 | 62.52 (min 59.60) | 67.20 (min 63.71) | 107.48 | 112.09 | 117.94 |
| Compile.WarmupJoin | 16.56 | 0.00 | 0.00 | 11.62 | 11.80 | 13.05 |
| Compile.SolverSchedule | 19.48 | 10.71 | 10.93 | 39.48 | 37.48 | 44.40 |
| Compile.PrepareRequests | 17.39 | 0.72 | 0.74 | 44.32 | 43.79 | 45.96 |
| Compile.Bake | 19.63 | 22.56 | 24.54 | 0 | 0 | 0 |
| Compile.DiscoverValidate | 16.04 | 19.64 | 19.04 | 16.09 | 20.35 | 18.30 |
| Compile.DigestJoin | n/a | 0.51 | 0.53 | n/a | 0.45 | 0.46 |
| TapPrepare warmup | 44.40 | 0 | 0 | 43.53 | 44.14 | 46.72 |
| Digest.Solvers | 55.54 | 48.37 | 49.43 | 57.65 | 53.29 | 52.64 |
| Evaluate.Run | 21.05 | 24.48 | 22.99 | 0 | 0 | 0 |

Across all 14 baked runs the Compile median is 65.35 ms (range 59.60..75.68), about 15 ms below M0. The M1 target in
§7 is ≤ 62 ms after CF2a and ~55-60 ms after CF3. Neither pass meets it: pass 1 misses by 0.5 ms and pass 2 by
5 ms. CF1's bar (≤ 70 ms) is met in both passes. CF2a's bar (≤ 63 ms) is met in pass 1 only. Evaluate.Run is 1.5-3.4 ms
higher than at M0, because CF1 moves the deferred exec prep onto the first frame. Bake and DiscoverValidate also went
up, by 3-5 ms and about 3 ms. That is consistent with the digest and exec lanes now running beside them rather than
behind a warmup join, but it was not isolated. The dynamic compile shows no gain: 112-118 ms against M0's 107.48 (3
runs), which matches CF3a's own A/B (116.80 vs 118.96). CF3a's claimed −5..8 ms on the dynamic compile is not visible
at this noise level.

G-perf, edits: `build/benchEditLatency.exe --mode baked --rounds 8` on Biped_anim (held=10), each scenario in its own
process. The table gives medians in ms. The M0 column is the F0a baseline.

| id | wall M0 | wall M1 | settle M1 | compile M1 | bake M1 | run M1 | baked |
|---|---|---|---|---|---|---|---|
| ref-steady | 0.52 | 0.37 | 0 | 0 | 0 | 0.37 | 8/8 |
| ref-time | 1.33 | 1.56 | 0 | 0 | 0 | 1.56 | 8/8 |
| T0-leaf | 1.53 | 1.65 | 0 | 0 | 0 | 1.64 | 8/8 |
| T0-hips | 1.44 | 1.22 | 0 | 0 | 0 | 1.21 | 8/8 |
| T1a | 1.32 | 1.52 | 0 | 0 | 0 | 1.51 | 8/8 |
| T1b | 1.49 | 1.88 | 0 | 0 | 0 | 1.87 | 8/8 |
| T1c-guide | 31.97 | 26.94 | 21.20 | 0 | 0 | 5.08 | 8/8 |
| T1c-mover | 32.66 | 25.61 | 20.90 | 0 | 0 | 4.50 | 8/8 |
| T1d | 33.69 | 26.36 | 21.35 | 0 | 0 | 4.87 | 8/8 |
| T2 | 55.50 | 52.35 | 23.05 | 0 | 23.96 | 5.49 | 8/8 |
| T3 (jointElements) | 82.00 | 70.76 | 65.69 | 44.60 | 21.07 | 5.21 | 8/8 |
| T3-weights | 53.48 | 49.78 | 21.75 | 0 | 20.98 | 5.63 | 8/8 |
| T4-add | 89.65 | 67.54 | 62.92 | 45.47 | 22.63 | 4.88 | 4/4 |
| T4-remove | 105.77 | 80.31 | 75.47 | 56.16 | 23.32 | 4.83 | 4/4 |
| T5 (broken) | 61.20 | 41.27 | 41.26 | 20.54 | 0 | 0 | 0/8 |
| T5-repair | 82.30 | 68.25 | 63.98 | 46.70 | 20.37 | 4.26 | 1/1 |

The settle on structurally dirty edits (T1c/T1d/T2/T3-weights) drops from ~28-29 ms to ~21-23 ms (CF2a + CF2b). That
is short of CF2b's *est.* 15-19 ms. The recompiling edits drop by 11-25 ms each. Frame run times on edited frames are
0.6-1.2 ms higher than at M0; this was not isolated.

Verdict: M1's G-parity is green on the build, ctest, python, pose parity, reference parity, log text,
BakedWithParityCheck, cone verification and the M1 item verifiers. The usdview runner check is carried open from M0.
G-perf is recorded. The M1 cold baked target (≤ 62 ms) is missed narrowly: the medians are 62.52 and 67.20 ms over two
7-run passes, with CF3b skipped and CF3c not attempted.

### M2 gate (2026-09-23)

M2 was measured on the working tree with one build throughout. Seven items landed: EP1a, EP1b, EP2a, EP2b, SU1a, EP3a and EP3b. Five items did
not land in this pass: EP1c, EP4a, EP4c, EP4b and EP5'. The pose comparisons are against `gate/base/` (the
workflow's starting build) and `gate/M1/`. Scratch outputs are under the workflow's `scratchpad/impl/M2gate_*`,
`scratchpad/impl/m2_pytests.log`, `scratchpad/impl/M2_cones/` and `scratchpad/gate/M2/`.

G-parity:

| check | result |
|---|---|
| build (`bin\build_rigexec.bat --no-test`) | PASS |
| ctest, full, `-j 8` | PASS, 261/261. That is 216 at M1 plus 45 new entries from the M2 items, for example EP1b's DigestGate variants, SU1a's `*Cones_grain0_*` and EP3b's scoped-clears parity entry. |
| `bin/run_python_tests.sh` | PASS, 17/17 (rc 0 under `set -e`) |
| `--pose-out`/`--joints-out`: 42 stages × {baked, dynamic, parity, guides, noparallel} | PASS: 335/335 files byte-identical to `gate/base/` and 335/335 to `gate/M1/`, 0 missing, and summary.tsv is identical to base (no rc change). Outcomes: 140 ok, 55 no-joints, 15 compile-failed, the same as at the start. |
| run logs (`.log`, 210) | Against `gate/M1/`, with the outdir path normalized, 205 are identical. The other 5 are the 14_VolumeConstrainedSweep runs, where the only change is the source line in the RigExecSphereWeight purpose warning (4823 → 5394), because M2 added code above it in rigEvaluator.cpp. Against `gate/base/`, the only other differences are the M0-sanctioned `RIGEXEC_MEASURE` removal (F0b). After normalizing the path, the warning line number and the `RIGEXEC_MEASURE` line with its `type … count=` histogram, 0 of 210 logs differ. No M2 item changes log text. |
| `BakedWithParityCheck` (`--mode parity`) | PASS: `bakedParityMismatches=0` on all 936 frames (39 compiling stages × 24). That includes Biped, Biped_anim, Biped_everything, 02_TwoBoneIkLeg, 05_TwistRibbonSpine and 09_PropertyMathMovers at 24/24 each. There is no "parity leg INVALID" in any log. |
| `RIGEXEC_BAKED_VERIFY_CONES=1`, serial and parallel, Biped / Biped_anim / Biped_everything, frames 1..8 | PASS: rc 0, 8/8 valid, 0 "cone mismatch" and `bakedParityMismatches=0` in all 6 runs. The poses are byte-identical to M1's cone runs. |
| usdview runners | Not run by this gate. They are still open from M0. |

G-perf, compile (cold rigExecPose on Biped_anim, frames 1..8, medians in ms). No M1 binary was kept, so the M1
columns are the M1 gate's own measurements from earlier in the workflow, on the same machine. While this gate ran,
the machine carried 53-68% CPU load from long-running `find` processes that belong to neither the build nor the bench.

| scope | baked M1 p1/p2 (7+7) | baked M2 p1 (7) | baked M2 p2 (7) | dynamic M1 (3 / 7) | dynamic M2 p1 (3) | dynamic M2 p2 (5) |
|---|---|---|---|---|---|---|
| Compile | 62.52 / 67.20 | 70.99 (min 69.62) | 66.01 (min 62.23) | 112.09 / 117.94 | 136.53 | 126.23 (min 123.43) |
| Compile.SolverSchedule | 10.71 / 10.93 | 13.12 | 12.29 | 37.48 / 44.40 | 45.91 | 43.69 |
| Compile.PrepareRequests | 0.72 / 0.74 | 0.73 | 0.73 | 43.79 / 45.96 | 59.67 | 50.18 |
| Compile.Bake | 22.56 / 24.54 | 26.10 | 23.79 | 0 | 0 | 0 |
| Compile.DiscoverValidate | 19.64 / 19.04 | 20.97 | 18.96 | 20.35 / 18.30 | 21.18 | 20.57 |
| Compile.DigestJoin | 0.51 / 0.53 | 0.55 | 0.49 | 0.45 / 0.46 | 0.48 | 0.47 |
| TapPrepare warmup | 0 | 0 | 0 | 44.14 / 46.72 | 51.05 | 47.94 |
| Digest.Solvers | 48.37 / 49.43 | 54.98 | 50.67 | 53.29 / 52.64 | 57.21 | 56.48 |
| Evaluate.Run | 24.48 / 22.99 | 23.67 | 22.08 | 0 | 0 | 0 |

The baked compile's second pass (66.01) is within M1's range (62.52..67.20), and no M2 item claims a cold-compile
change. The dynamic compile is 8-18 ms above M1's measurement, mostly in PrepareRequests and TapPrepare. Machine load
confounds that, so without an M1 binary it is not attributed. It is an open item for M3, which targets the dynamic
compile.

G-perf, edits: `build/benchEditLatency.exe examples --mode baked --rounds 8` on Biped_anim (held=10), each scenario
in its own process. The table gives medians in ms.

| id | wall M0 | wall M1 | wall M2 | settle M2 | compile M2 | bake M2 | run M2 | steps M2 | baked | M2 target |
|---|---|---|---|---|---|---|---|---|---|---|
| ref-steady | 0.52 | 0.37 | 0.44 | 0 | 0 | 0 | 0.44 | 2/634 | 8/8 | |
| ref-time | 1.33 | 1.56 | 1.45 | 0 | 0 | 0 | 1.44 | 430/634 | 8/8 | |
| T0-leaf | 1.53 | 1.65 | 0.83 | 0 | 0 | 0 | 0.82 | 16/634 | 8/8 | SU1a 0.8-1.0: met |
| T0-hips | 1.44 | 1.22 | 1.49 | 0 | 0 | 0 | 1.48 | 404/634 | 8/8 | |
| T1a | 1.32 | 1.52 | 1.61 | 0 | 0 | 0 | 1.59 | 404/634 | 8/8 | |
| T1b | 1.49 | 1.88 | 1.48 | 0 | 0 | 0 | 1.43 | 404/634 | 8/8 | |
| T1c-guide | 31.97 | 26.94 | 21.81 | 20.77 | 0 | 0 | 0.98 | 2/634 | 8/8 | < 5: not met (needs EP1c) |
| T1c-mover | 32.66 | 25.61 | 22.84 | 20.25 | 0 | 0 | 1.17 | 2/634 | 8/8 | < 5: not met (needs EP1c) |
| T1d | 33.69 | 26.36 | 0.61 | 0 | 0 | 0 | 0.59 | 2/634 | 8/8 | < 5: met (EP1b) |
| T2 | 55.50 | 52.35 | 49.90 | 23.89 | 0 | 22.93 | 1.98 | 634/634 | 8/8 | ≤ 5: not met (needs EP1c + EP4a) |
| T3 (jointElements) | 82.00 | 70.76 | 53.31 | 48.53 | 48.51 | 23.07 | 5.41 | 634/634 | 8/8 | |
| T3-weights | 53.48 | 49.78 | 45.55 | 20.50 | 0 | 22.87 | 2.14 | 634/634 | 8/8 | |
| T4 (all) | n/a | n/a | 61.05 | 56.03 | 56.02 | 23.95 | 5.01 | 635/635 | 8/8 | ≤ 63: met |
| T4-add | 89.65 | 67.54 | 53.44 | 48.48 | 48.47 | 23.34 | 4.77 | 635/635 | 4/4 | |
| T4-remove | 105.77 | 80.31 | 67.38 | 61.76 | 61.75 | 25.34 | 5.59 | 634/634 | 4/4 | |
| T5 (broken) | 61.20 | 41.27 | 0.00 | 0 | 0 | 0 | 0 | - | 0/8 | < 1: met (EP2b); one round of 8 took 218.52 (not isolated) |
| T5-repair | 82.30 | 68.25 | 82.32 | 76.47 | 57.06 | 28.95 | 5.84 | 634/634 | 1/1 | a single sample |

EP3a/EP3b show up in the run column: after a non-avar edit, the run drops from 3.6-5.1 ms to 0.6-1.2 ms (T1c/T1d, 2/634
steps), and T2/T3-weights runs drop from about 5.5 to about 2 ms. SU1a shows up in T0-leaf (16/634 steps, 0.83 ms).
T5-repair is a single sample, and it recompiles, as EP2b requires.

Verdict: M2's G-parity is green on build, ctest (261/261), python (17/17), pose parity (335/335 against base and M1),
log text (only sanctioned differences), BakedWithParityCheck (936/936 frames with 0 mismatches) and cone verification
(6/6). Of the M2 edit targets, T1d, T4 and T5 are met with the items that landed. T1c and T2 are not met, because
their enabling items (EP1c, EP4a, EP4b, EP4c, EP5') have not landed. The baked cold compile holds at M1's level. The
dynamic cold compile reads 8-18 ms higher under machine load and is unattributed.

### M3 gate (2026-09-24)

M3 was measured on the working tree with one build throughout. Two items landed: HP-D0 and HP-D2. HP-D-flip did not
land, because it needs the user's confirmation, so `RIGEXEC_DYNAMIC_RUNS_PROGRAM` is still off by default. HP-D1 did
not land either. The gate's own wording is "G-parity with Dynamic running the program by default". That condition
cannot hold until the flip, so it was checked in its flag-on form instead: every stage runs with the flag set and is
compared to `--mode reference`. The pose comparisons are against `gate/base/` (the workflow's starting build) and
`gate/M2/`. Scratch outputs are under the workflow's `scratchpad/impl/M3gate_*`, `scratchpad/impl/M3_cones/` and
`scratchpad/gate/M3/`.

G-parity:

| check | result |
|---|---|
| build (`bin\build_rigexec.bat --no-test`) | PASS |
| ctest, full, `-j 8` | PASS, 263/263. That is M2's 261 plus the two new entries `testRigExecDynamicDispatch` and `testRigExecDynamicDispatchProgram` (HP-D0, HP-D2). |
| `bin/run_python_tests.sh` | PASS, 17/17 (rc 0 under `set -e`) |
| `--pose-out`/`--joints-out`: 42 stages × {baked, dynamic, parity, guides, noparallel}, flag off | PASS: 335/335 files byte-identical to `gate/base/` and 335/335 to `gate/M2/`, 0 missing, and summary.tsv is identical to base. Outcomes: 140 ok, 55 no-joints, 15 compile-failed, the same as at the start. |
| `--mode reference` on all 42 stages | PASS: 67/67 `.pose`/`.joints.usda` files are byte-identical to the starting build's `--mode dynamic`, and every per-stage rc matches. |
| `RIGEXEC_DYNAMIC_RUNS_PROGRAM=1 --mode dynamic` against `--mode reference`, all 42 stages, frames 1..24 | PASS: 67/67 files byte-identical, and every per-stage rc matches (28 rc 0, 14 rc 1, the same in both modes). In all 39 stages that compile, the program answered every frame (`baked: N/N generation(s), 1 build(s), 1 attempt(s)`). The three stale examples do not compile in either mode. |
| run logs (`.log`, 210) | Against `gate/M2/`, after normalizing the outdir path and the source line of the RigExecSphereWeight purpose warning, 0 of 210 differ. That warning is on the 5 14_VolumeConstrainedSweep runs, and its line moved from 5394 to 5439 because M3 added code above it in rigEvaluator.cpp. Against `gate/base/`, the only other difference is the M0-sanctioned `RIGEXEC_MEASURE` removal (F0b), which also splits 18 biped dynamic/parity lines in the old logs. No M3 item changes flag-off log text. HP-D0's accounting line `baked: N/N generation(s)…` shows up only in flag-on dynamic runs. |
| `BakedWithParityCheck` (`--mode parity`) | PASS: all 936 frames are valid with 0 mismatches (39 compiling stages × 24, the bipeds included), and no log has "parity leg INVALID". |
| `RIGEXEC_BAKED_VERIFY_CONES=1`, serial and parallel, Biped / Biped_anim / Biped_everything, frames 1..8 | PASS: rc 0, 8/8 valid, 8/8 generations, and 0 "cone mismatch" in all 6 runs. The poses are byte-identical to M2's cone runs. An extra check ran the same thing with the flag on under `--mode dynamic` on Biped_anim and Biped_everything, serial and parallel: 4/4 runs are clean, and their poses are byte-identical to the baked cone runs. |
| usdview runners | Not run by this gate. They are still open from M0. |

G-perf, compile (cold rigExecPose on Biped_anim, frames 1..8, medians in ms). All three columns come from one binary
in one session. The flag-off dynamic column is today's dynamic path, so it serves as the same-session "before" for
HP-D0. While the gate ran, the machine carried about 45% CPU load from long-running `find` processes that belong to
neither the build nor the bench.

| scope | baked M2 p1/p2 (7+7) | baked M3 (7) | dynamic M2 p1/p2 (3/5) | dynamic flag off M3 (3) | dynamic flag on M3 (3) |
|---|---|---|---|---|---|
| Compile | 70.99 / 66.01 | 64.25 (min 56.42) | 136.53 / 126.23 | 114.58 | 62.35 (min 61.48) |
| Compile.WarmupJoin | 0 | 0 | n/a | 12.97 | 0 |
| Compile.SolverSchedule | 13.12 / 12.29 | 11.29 | 45.91 / 43.69 | 39.93 | 11.58 |
| Compile.PrepareRequests | 0.73 / 0.73 | 0.74 | 59.67 / 50.18 | 44.97 | 0.76 |
| Compile.Bake | 26.10 / 23.79 | 23.15 | 0 | 0 | 22.19 |
| Compile.DiscoverValidate | 20.97 / 18.96 | 18.91 | 21.18 / 20.57 | 18.27 | 18.15 |
| Compile.DigestJoin | 0.55 / 0.49 | 0.50 | 0.48 / 0.47 | 0.46 | 0.49 |
| TapPrepare warmup | 0 | 0 | 51.05 / 47.94 | 45.19 | 0 |
| Digest.Solvers | 54.98 / 50.67 | 46.40 | 57.21 / 56.48 | 47.60 | 45.24 |
| Evaluate.Run | 23.67 / 22.08 | 21.05 | 0 | 0 | 19.48 |

HP-D0's compile claim holds. With the flag on, the dynamic compile drops to the baked floor: 114.58 → 62.35 ms, against
64.25 baked. The M2 gate left the dynamic compile's 8-18 ms rise open. This session's flag-off dynamic compile
(114.58) is back at M1's level (112.09), so that rise was machine load and not a regression.

G-perf, dynamic frame (Biped_anim, animated frames):

| measure | dynamic flag off | dynamic flag on | baked | spec (HP-D0) |
|---|---|---|---|---|
| untraced ms/frame, `--repeat 41` minus `--repeat 1` over 24 frames, median of 5 | 5.06 | 1.05 | 1.10 | ~4.1 → ~1.2: met |
| profiled `ref-time` wall (F0a), median of 8 | 6.36 | 1.53 | 1.39 | |
| profiled `ref-steady` wall (F0a) | 2.32 | 0.28 | 0.28 | |

G-perf, edits: `build/benchEditLatency.exe examples --rounds 8` on Biped_anim (held=10), with each scenario in its own
process. The table gives wall medians in ms. The dynamic columns are the same binary with the flag off and on.

| id | baked M0 | baked M2 | baked M3 | dynamic flag off | dynamic flag on | flag-on settle / bake / run | flag-on baked |
|---|---|---|---|---|---|---|---|
| ref-steady | 0.52 | 0.44 | 0.28 | 2.32 | 0.28 | 0 / 0 / 0.28 | 8/8 |
| ref-time | 1.33 | 1.45 | 1.39 | 6.36 | 1.53 | 0 / 0 / 1.52 | 8/8 |
| T0-leaf | 1.53 | 0.83 | 0.84 | 6.59 | 0.78 | 0 / 0 / 0.77 | 8/8 |
| T0-hips | 1.44 | 1.49 | 1.23 | 7.25 | 1.12 | 0 / 0 / 1.11 | 8/8 |
| T1a | 1.32 | 1.61 | 1.30 | 6.57 | 1.34 | 0 / 0 / 1.33 | 8/8 |
| T1b | 1.49 | 1.48 | 1.18 | 6.27 | 1.30 | 0 / 0 / 1.29 | 8/8 |
| T1c-guide | 31.97 | 21.81 | 18.88 | 23.83 | 19.03 | 18.22 / 0 / 0.72 | 8/8 |
| T1c-mover | 32.66 | 22.84 | 19.12 | 25.35 | 19.63 | 18.50 / 0 / 0.96 | 8/8 |
| T1d | 33.69 | 0.61 | 0.44 | 4.86 | 0.40 | 0 / 0 / 0.38 | 8/8 |
| T2 | 55.50 | 49.90 | 42.53 | 27.14 | 39.07 | 17.29 / 19.74 / 1.76 | 8/8 |
| T3 (jointElements) | 82.00 | 53.31 | 49.88 | 87.73 | 49.73 | 45.12 / 20.85 / 4.70 | 8/8 |
| T3-weights | 53.48 | 45.55 | 43.74 | 25.50 | 40.30 | 19.39 / 19.66 / 1.87 | 8/8 |
| T4 (all) | n/a | 61.05 | 55.71 | 118.08 | 55.27 | 50.52 / 20.66 / 4.46 | 8/8 |
| T4-add | 89.65 | 53.44 | 48.85 | 92.79 | 46.94 | 42.79 / 20.30 / 4.46 | 4/4 |
| T4-remove | 105.77 | 67.38 | 57.27 | 123.44 | 56.91 | 52.02 / 20.74 / 4.73 | 4/4 |
| T5 (broken) | 61.20 | 0.00 | 0.00 | 0.00 | 0.00 | 0 / 0 / 0 | 0/8 |
| T5-repair | 82.30 | 82.32 | 67.20 | 105.80 | 64.58 | 60.12 / 20.24 / 4.44 | 1/1 (a single sample) |

With the flag on, Dynamic's edit costs match Baked's row for row. Most tiers get cheaper than today's dynamic path:
the steady and animated frames by 4-6x, T3 and T4 by 38-63 ms, and T5-repair by 41 ms. Two tiers get more expensive,
T2 (27.14 → 39.07) and T3-weights (25.50 → 40.30). The walk only re-reads a captured rest or weight, while the
program re-Bakes (about 20 ms), because EP4a has not landed. HP-D-flip's own gate requires that "an index-hitting
value edit must cost less than a ~19 ms Bake" before the default is flipped. That has not been measured, because EP4a
is not in. No M3 item claims a baked-mode change, and the baked columns stay within M2's range or below it under
this session's load.

Verdict: M3's G-parity is green on the build, ctest (263/263), python (17/17), pose parity (335/335 against base and
M2), reference parity (67/67), flag-on dynamic against reference (67/67, 39/39 stages at full generations), log text
(only sanctioned differences), BakedWithParityCheck (936/936 frames with 0 mismatches) and cone verification (6/6,
plus 4/4 flag-on). G-perf is recorded. The flag-on dynamic compile reaches the baked floor, and the untraced dynamic
frame drops from 5.06 to 1.05 ms. The gate's "by default" condition is not met, because HP-D-flip awaits the user's
confirmation. Before the flip, the T2/T3-weights regression under the flag (about +12-15 ms, one Bake) needs EP4a.

### Session totals

The session is the unified-program workflow, M0 through M3, measured on Biped_anim unless a row says otherwise. The
start column is the workflow's starting build (`gate/base_bench_*.txt` and the M0 F0a baseline). The end column is
this M3 gate.

Landed: F0a, F0b, F0c, X0 (M0); CF1, CF2a, CF2b, CF3a (M1); EP1a, EP1b, EP2a, EP2b, SU1a, EP3a, EP3b (M2); HP-D0,
HP-D2 (M3).
Not landed:
- CF3b: skipped, because its F0a gate was not met.
- CF3c.
- EP1c, EP4a, EP4b, EP4c, EP5'.
- HP-D-flip: needs the user's confirmation.
- HP-D1.
- M4-M6.

| measure | start | end | change |
|---|---|---|---|
| cold baked Compile (7 runs) | 83.12 | 64.25 | −18.9 ms (−23%) |
| cold dynamic Compile, today's path (3 runs) | 111.11 | 114.58 | no change within noise |
| cold dynamic Compile, flag on (3 runs) | n/a | 62.35 | −48.8 ms against the start's dynamic |
| dynamic animated frame, untraced | ~4.1 (spec) | 1.05 flag on / 5.06 flag off | the flag-on path is at the baked frame cost |
| baked T0-leaf drag | 1.53 | 0.84 | SU1a |
| baked T1c (non-avar value in the rig) | 31.97-32.66 | 18.88-19.12 | M2 target < 5: not met (needs EP1c) |
| baked T1d (value outside the rig) | 33.69 | 0.44 | M2 target < 1: met |
| baked T2 (captured rest) | 55.50 | 42.53 | M2 target ≤ 5: not met (needs EP1c + EP4a) |
| baked T3 (folded value) | 82.00 | 49.88 | −32 ms |
| baked T4 add / remove | 89.65 / 105.77 | 48.85 / 57.27 (all: 55.71) | M2 target ≤ 63: met |
| baked T5 (broken stage, per frame) | 61.20 | 0.00 | M2 target < 1: met |
| baked T5-repair | 82.30 | 67.20 | single sample |

Every gate in the session was byte-identical to the starting build on all 335 pose/joint files, and on the 67
reference files from M0 on. Each gate had 0 BakedWithParityCheck mismatches over 936 frames and clean cone
verification, serial and parallel. ctest grew from 192 to 263 entries.

Still open:
- The usdview runner check, since M0. Draw, noodles_rename and viewcube are unattributed.
- The M1 cold baked target of ≤ 62 ms. This gate's 7-run median is 64.25 ms, with a minimum of 56.42, under load.
- The M2 targets for T1c and T2.
- The M3 default flip.
