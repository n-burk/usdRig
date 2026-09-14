# Baked program as a step graph (Phase 2)

Status: BUILT. §§1-10 below are the specification the restructure of `RigExecBakedProgram` was
made against, kept verbatim so the rules can still be read as rules; the review findings that
produced them are cited as [P#] (parity) and [S#] (scheduling), and line numbers refer to
bakedProgram.cpp on branch `bake-all/infra` before Phase 2 (Run at 2124-2980). Everything above
§1 says what the code actually does, with the numbers it does it in. Where the two disagree the
deviation is listed, with its reason.

## What was built

`Run` is no longer a straight line. A frame is a serial prologue, a region of STEPS over dense
slots, and a serial epilogue; the steps form a dependency graph derived at Build from the slot
ranges each step declares, and the whole graph is partitioned into clusters that either the serial
executor walks in program order or the parallel one spreads across the work arena. The files are
§9's: `bakedProgramImpl.h` (program state, slot domains, `RigExecBakedStep`, clusters, cones),
`bakedPose.cpp` and `bakedGeometry.cpp` (each domain's bake, step builders and step bodies),
`bakedSchedule.{h,cpp}` (the edge sweep, the cost model, the clustering, both executors, the cone
closures, the calibration mode, the timing replay, the reports), `bakedVerify.cpp` (the run shadow
`RIGEXEC_BAKED_VERIFY_CONES` compares two runs of one frame with) and `bakedProgram.cpp` (the
public surface, `IsBakeable`, the `Build` skeleton, the prologue, the epilogue and
`RigExecComparePoses`).

A skin revision is cut into contiguous vertex chunks at Build (§6); each chunk waits for the
`FinalMatrix`/`BaseMatrix` slots of ITS OWN influences and for nothing else, writes its range of
the revision's own buffer, and the fuse decides afterwards whether the revision applied at all.

Every writer of a pose slot has storage of its own (§3.1). The frames are SSA: a reader binds at
Build to the entry holding the version live where it runs, so no version is ever overwritten and
a step that skipped a run leaves every reader bound to it exactly what it is entitled to. That is
what lets a drag run its own cone instead of the program.

A frame runs only the CLOSURE of what its sources say moved (§7). Sources always run and are
compared by value; a skipped cluster keeps its slots, its diagnostics and its structural counters.
`RIGEXEC_BAKED_VERIFY_CONES=1` runs every generation twice -- once as the cone decided, once
whole, from the state the prologue left -- and reports every slot, counter and diagnostic the two
disagree about as a mismatch on the pose.

Every published value, every diagnostic in order and every compared counter of the three rigs that
bake is byte-identical to the straight line it replaced, in serial and in parallel, at grain 0,
the default grain and 200us, at 1, 8 and 32 chunks, at 512 and 4096 chunk vertices, and with the
cone verifier on in each.

### The environment

| variable | default | what it does |
|---|---|---|
| `RIGEXEC_BAKED_SCHEDULE` | `serial` | `serial` or `parallel`; forced to serial when `RIGEXEC_ENABLE_PARALLEL_EVAL=0` |
| `RIGEXEC_BAKED_GRAIN_US` | `clamp(total/(4P), 5, 50)` | the packing grain; 0 is one step per cluster |
| `RIGEXEC_BAKED_CHUNK_VERTS` | 4096 | vertices per skin chunk before the cap |
| `RIGEXEC_BAKED_MAX_CHUNKS` | 32 | chunks per skin revision; the range grows to meet it |
| `RIGEXEC_BAKED_VERIFY_CONES` | off | run every frame twice and compare, slot by slot |
| `RIGEXEC_BAKED_SCHEDULE_REPORT` | off | the structural report at Build and the run report per frame |
| `RIGEXEC_BAKED_SCHEDULE_CALIBRATE` | off | fit and print a replacement cost table |
| `RIGEXEC_BAKED_STEP_TIMING` | off | sum the three phases and every step kind over N frames (8 when set to 1) and print the table |

### The schedule, on the biped

```
rigExec baked schedule: 458 step(s), 1066 edge(s), 326 provider slot(s)
  mode=serial grain=6.98us concurrency=20
  clusters=57 (288 edge(s)) serial=558.08us criticalPath=294.46us (1.90x)
  ChainStatus 1        CommitApply 2      CommitDelta 2      ComposeSubtree 93
  Constraint 65        Derived 1          InfluenceFold 1    PropagateChunk 4
  ProviderMatrix 252   RevisionChunk 7    RevisionFuse 1     RevisionStatic 1
  Solve 14             SolverCommit 14
  skin revisions:
  /Biped/Rig/Movers/skin_body_geo/body_geo_skin: 7 chunk(s), 26276 vertex(es), 137 influence(s)
    vertices/chunk min 1700 mean 3753.7 max 4096
    |key| min 24 mean 41.3 max 58; 0/7 chunk(s) reach half the influences
    skin serial 280.79us critical path 53.96us (5.20x)
    ready level min 40 max 40 of 44
```

The partition bought what §6 hoped it would and the report says so per asset: no chunk of the
biped's body mesh depends on as much as half of the 137 influences, the mean chunk waits for 41 of
them, and the skin's critical path is 54us against 281us of serial skinning. What it did not buy
is a spread of READY LEVELS -- all seven chunks become runnable at level 40 of 44 -- because the
biped's constraints finalise nearly every joint in the same few levels. The arm vertices no longer
wait for the leg constraints ARITHMETICALLY; on this rig they happen to be ready at the same time
anyway.

### What a frame costs

Measured with the profiler OFF, because on a frame this size the profiler is not an observer:
`RIGEXEC_PROFILE_SCOPE` takes three mutexes even with recording off, the parallel executor takes a
clock pair per step across twenty threads, and the traced frame used to read 1096us against an
untraced 820us. Two untraced instruments replace it. `rigExecPose --repeat N` cycles the frame list
inside one process and reports microseconds per frame, which takes the stage open, the compile and
the bake out of the number; `RIGEXEC_BAKED_STEP_TIMING=N` sums the three phases and every step kind
with two clock reads apiece. It counts only frames that published a pose -- a frame a step hands
back, or one the publication declines, contributes to neither the numerator nor the divisor -- and
under `RIGEXEC_BAKED_VERIFY_CONES=1` the verifier's second whole-program pass is charged to no
phase and kept out of the per-step accumulators, so the table still says what ONE frame costs. Numbers below are `Biped_anim` frames 2-8 on a 20-core box, and
`b705950` is the Phase 1 head -- the straight line, before any of this.

**The frame.** In-process, frames 2-8 cycled 150x, minimum of nine runs, and (for `b705950`, whose
tool has no `--repeat`) the same frames as a 2800-frame and a 700-frame process, differenced:

| | us/frame | user CPU | system CPU |
|---|---|---|---|
| b705950 (straight line) | 757 | 1019us | 329us |
| serial | **662** | 671us | ~5us |
| parallel | 705 | 1210us | 824us |

Serial is 12.5% FASTER than the straight line and runs on half the CPU. §8.4's first half (within
5% of `b705950`) therefore passes with room; its second half -- parallel faster than serial -- does
not, and **`RIGEXEC_BAKED_SCHEDULE` still defaults to `serial`**. What follows is why, measured
rather than argued, so that nobody repeats the experiment.

Versioned storage (§3.1) did not move that. The two libraries were measured A/B through one
`rigExecPose --repeat 150`, twelve interleaved pairs on a busier box: min 666.8 -> 665.1us, median
716.2 -> 700.4us in serial, and 713.7 -> 725.9us (min) in parallel. The carry copies are the only
per-frame work the version table adds and they measure nothing -- compiled out, the serial min
moves 666.2 -> 660.4us, inside a box whose samples are bimodal at 670 and 720. Measure both
libraries in one session before quoting either: a single-sided run of the same pair read +1.6% the
other way an hour earlier.

**Where the frame goes** (`RIGEXEC_BAKED_STEP_TIMING=40`, us/frame; the instrument costs about 5%
of what it reports, so read the shares, not the total). Both executors time a step with a PAIR of
clock reads around the body and nothing else, so the two columns measure the same interval and may
be read against each other; the trace's per-step intervals are a different measurement (serial
shares one read per step boundary there, so a trace interval also covers the bookkeeping under the
step) and the two must not be mixed:

| | serial | parallel |
|---|---|---|
| prologue | 160 | 157 |
| region | 554 | 787 |
| epilogue | 52 | 46 |
| RevisionChunk (7 runs) | 262 | 339 |
| Derived (1 run) | 103 | **336** |
| Constraint (65 runs) | 97 | 118 |
| ComposeSubtree (93 runs) | 32 | 36 |
| ProviderMatrix (252 runs) | 13 | 16 |

**The parallel region is slower because every step in it is slower, not because the schedule is
wrong.** The same bodies, doing the same arithmetic, cost 888us of step time in parallel against
535us in serial. (Those two figures predate the pairing described above: the serial column was
measured with the rolling boundary and so carries the snapshot merge and the skipped-step scan with
it, worth about 2% of the region -- 12us of 550 -- which makes the gap wider, not narrower. Every
later comparison uses the pair.) The three control experiments say where that comes from:

| control | us/frame |
|---|---|
| the whole frame, no step bodies, but the same 57 clusters spawned | 248 |
| the same, with no clusters spawned at all | 243 |
| every step on the calling thread, inside the same `WorkWithScopedParallelism` + `WorkDispatcher` | 714 |

Spawning 57 tasks costs 5us of WALL time, and the envelope costs nothing: `WorkDispatcher` is a
`tbb::task_group` plus a context, and `WorkWithScopedParallelism` is `tbb::this_task_arena::isolate`
with no arena construction in it. Cluster count is not the problem either -- `GRAIN_US=0` (458
clusters), the default (57) and `GRAIN_US=400` (41) all land within noise of 790us. What the
spawning DOES cost is 424us/frame of system time in the workers, and `strace -f -c` over 700 frames
names it exactly:

| | sched_yield | futex |
|---|---|---|
| b705950 | 142098 (203/frame) | 1601 |
| serial | 33542 (48/frame) | 906 |
| parallel | 134527 (192/frame) | 1518 |

Those are TBB workers spinning in their steal loop. A frame of 660us that wakes the arena never
lets them get to sleep, so nineteen threads yield their way through the frame -- including through
the region's memory-bound serial tail, where one thread makes seven passes over a 26k-point mesh.
That is the `Derived` row above: the same bounding box, 103us alone and 336us with the arena awake
beside it. `b705950` pays the same spin for the same reason (its skin kernel calls
`WorkParallelForN` every frame); the serial executor is the only one of the three that leaves the
arena alone, and it is the fastest of the three.

So the honest statement of the ceiling is: **this frame is too small and too memory-bound for the
arena to pay.** 558us of modelled serial work against a 294us critical path is at most 1.9x before
overhead; of the real 660us, 160us is the serial prologue (almost all of it the property chains,
119us, which are USD value resolution and not arithmetic) and about 190us is a strictly serial tail
(the fuse, the chain status sweep and the extent, each a pass over the whole mesh). A frame that is
actually quicker is the evidence that flips the default; the mode is correct at every grain and
every chunk count today, and one environment variable away.

**What a default frame is instrumented with.** Nothing inside a step. No step body opens a profile
scope (`RIGEXEC_PROFILE_SCOPE_CAT` calls `IsEnabled()` three times and each call takes a mutex,
which is why §2.2 keeps them out), and with the profiler off, `RIGEXEC_BAKED_SCHEDULE_REPORT` off
and `RIGEXEC_BAKED_STEP_TIMING` off, neither executor reads a clock at all. What is left is about
twenty scopes in the serial prologue, the serial epilogue and around the region -- sixty
uncontended mutex acquisitions, which is below the run-to-run spread of the frame measurement
itself and is not separately visible in it.

**What was removed, and what was measured and left.** The epilogue's map publication went from
152us to about 50us by filling the published maps from their end (`emplace_hint`) in path order --
about 850 keys that were each a search from the root. Rejected by measurement rather than by
preference: a coarser grain or a cluster cap (cluster count does not move the frame); a persistent
`WorkDispatcher` owned by the program (construction is not the cost); moving `Solve` and
`Constraint` parameter reads into the prologue source pass (worth the 21us those 65 steps lose to
the static input cache's worker-thread bypass, and nothing at all in serial mode); and removing the
allocations from `RigExecResolvedInputs::GetAttribute` (no measurable change -- the cost of a
parameter read is USD's value resolution, about 0.38us of the 0.43us).

**The one number a next stage should act on first.** The skin partition costs more than it buys on
this rig, because a chunk body is a serial loop where an unpartitioned revision calls a kernel that
spreads itself over the arena:

| | serial | parallel |
|---|---|---|
| default (7 chunks) | 662 | 705 |
| `RIGEXEC_BAKED_MAX_CHUNKS=1` | **615** | 628 |

50-90us per frame, and it agrees with the report's own finding above: all seven chunks are ready at
level 40 of 44, so there is no head start to pay for the loss of the data-parallel kernel. The rule
that would settle it is "cut a revision only when its chunks' ready levels differ", and the reason
it is not implemented here is ordering: `PartitionAtBuild` runs before the edge sweep, so the levels
it needs do not exist yet. Running the sweep and `RigExecBakedAssignStepCosts` over the pose steps
first -- geometry steps never precede a pose step, so their absence cannot change a
`ProviderMatrix` level -- would make the levels available where the cut is decided. The defaults
are left where Phase 2 set them until that is done, so that the feature is not switched off on the
strength of one asset.

### What a drag costs

`rigExecPose --drag <prim> <attr> <steps>` is the manipulator's frame: an override placed through
`SetInteractiveOverrides`, a generation asked for, a pose drawn, over and over on one control at one
time. The displacements are a triangle wave of twenty distinct values, so no step ever hands the rig
the value the step before it did -- a ramp that repeated itself would measure a cone skipping
everything and call it a fast drag.

Median of 60 steps, min of three runs, `Biped.usda`, µs per drag step. The `b705950` columns are
this same tool's source compiled against the straight-line library, so the two sides differ only in
the library. The reference worktree's own tool stays exactly as `b705950` wrote it -- backporting
`--repeat`/`--drag` into it would leave a reference build that is no longer the reference, with
nothing in its history to say so -- so the comparison binary is built beside it instead. "wide clear" is the old unconditional `_skinTopologies.Clear()` on every override set:

| control | b705950 dynamic | b705950 baked | serial | parallel | serial, wide clear | parallel, wide clear |
|---|---|---|---|---|---|---|
| `neck_end_ctl` (head end) | 7000 | 1135 | **746** | 769 | 1045 | 1262 |
| `arm_l_fk_wrist_l_bind` | 7108 | 1140 | **565** | 661 | 1014 | 926 |
| `hips_ctl` | 7626 | 1147 | **759** | 814 | 1069 | 1121 |
| `spine_end_ctl` | 7441 | 1177 | **729** | 801 | 1054 | 1087 |

The biped authors no `brow` control -- the `brow_*_bind` prims are joints -- so the head end of the
control chain stands in for the smallest facial cone the rig has.

Two changes account for the drop from 1135-1177 to 565-759. The larger is the topology clear
[S29]: dropping every skin layout on every override set costs 280-460µs of a drag frame, because
the prologue then re-reads and re-compares 105k layout elements that no manipulator touched. It
buys nothing numerically -- `RigExecSkinTopologyCache` keeps the previous layout as a CANDIDATE and
hands back the same pointer for arrays that compare equal, which is exactly why the packet does not
change and the revision does not re-execute -- so the whole cost is the re-read. The rule now is:

> Drop the layouts only for an override set that can REACH a skin mover's layout, and decide
> "can reach" with the same connection walk the layout is read through
> (`RigExecResolvedInputs::GetAttribute`: one authored connection per hop, the resolved map
> consulted at every hop). The set is each skin mover's `rigExec:jointIndices`,
> `rigExec:jointWeights`, `rigExec:elementSize` and `rigExec:skinningMethod`, plus every attribute
> upstream of one of them along that walk -- an override one hop upstream of a connected
> `jointIndices` is the case that separates a re-read from a silently stale deformation. Answer YES
> wherever the predicate is unsure: a computation override names no property to compare, and a rig
> whose movers are not resolved yet has nothing to compare against. The answer is cached and
> dropped exactly where the layouts are (every notice, every recompile), so the two can never
> describe different epochs. `inputs:enabled` and `inputs:defaultWeight` are deliberately NOT in
> the set: they are read per frame and never cached, so an override on one reaches the next
> generation without anything being dropped.
>
> The dynamic path shares the cache, so this changes no published value on either path.
> `testRigExecSkinTopology` holds both halves: an override one hop upstream of a connected
> `rigExec:skinningMethod` drops the layouts and the deformation follows it, and an override on a
> control avar keeps them while the deformation still follows the drag.

The smaller is the cone. With versioned pose storage (§3.1) a drag runs only what it can reach:

| | clusters run, before | after |
|---|---|---|
| `Biped_anim`, a frame already evaluated at that time | 8 of 57 | **1 of 57** |
| `Biped.usda`, a control overridden with its own authored value and held | 8 of 57 | **1 of 57**, 0 revisions executed |
| a constraint-input drag (`elbowTwist_l_bind_aim.inputs:defaultWeight`) | 57 of 57 | **42 of 57** |
| a leaf control drag (`arm_l_fk_wrist_l_bind.avars:rz`) | 57 of 57 | **52 of 57** (the graph's own bound: 52) |
| `spider_legs`, a repeated frame | 0 of 1 | 0 of 1 |

The leaf-control cone is large because the compose partition is coarse -- one `ComposeSubtree` step
covers a whole branch of the provider forest, so a wrist avar dirties the cluster that composes its
whole arm, and every matrix and skin chunk below it. That is a partition question, not a cone one:
the assertion in `testRigExecBakedSchedule` is against the bound the graph itself computes, so a
finer partition tightens the number and the test with it.

The baked drag is still 9-13x the dynamic path's, which is the point of the exercise.

### Deviations from §§1-10, each with its reason

* The slot **kind** of §3 is called a slot DOMAIN in the code (`RigExecBakedSlotDomain`), because
  `RigExecBakedSlotKind` already says what a PROVIDER slot is.
* `InfluenceFold(c, r)` runs BEFORE `RevisionStatic(c, r)` for every operation but a SKIN, and the
  packet depends on it. §6 wants the static packet to be independent of the influence matrices,
  which for a skin revision it now is -- that is what lets a chunk start on its own joints. For
  everything else the assembler reads the table, and splitting `RigExecAssembleParameters` in two
  for operations that are not chunked buys nothing.
* `graphChainsBuilt` / `graphRevisionsBuilt` are summed from per-step counter deltas rather than
  taken as program constants (§4.2 item 7). A chain -- or a derived target -- whose base attribute
  does not read at the frame's time is skipped entirely by the dynamic path and by today's `Run`,
  counters included, and a constant cannot reproduce that. A skipped step keeps these two and
  loses `revisionsExecuted`, which is the one that is an observation.
* A commit's size in the cost model is its CANDIDATES PLUS its propagation pairs, not §5.1's
  `|propagate|` alone. Both halves of the commit walk the candidate table in slot order, and a
  solver batch with forty candidates and no descendants is not free.
* A `Derived` step's size is the CHAIN's vertex count, which §5.1 does not name. `recomputeExtent`
  walks the points it is maintained from and publishes two vectors; sizing it by its own array
  made the fitted per-unit cost 50us, which is the same number saying the model was wrong. A
  `ChainStatus` step's size is the chain's vertex count for the same reason and against §5.1's
  "revisions of the chain": the sweep ends by copying the chain's published point array whole.
* Every recorder of the run's phased-read store declares the whole store up to its own step,
  instead of only its own slot, on a rig where something can look a record up (`phasedReads`).
  Per-step slots order a record against its READERS, which is what §4 asks for, but the records
  reach one container through a fold the executor performs, and two folds at once is a race
  whatever the slots say. Widening a declared write is always sound (§2.5), the widening costs
  nothing on a rig with no read phase, and the alternative -- a lock around the store -- is
  forbidden. Such a rig also runs every cluster of every frame, for the same reason: the store is
  emptied at the head of a run, so a skipped recorder leaves a hole in it rather than last run's
  answer.
* The schedule report is TWO reports. `RigExecBakedScheduleReport` is structural and
  deterministic, so two builds of one stage produce the same text and a test can say so; the
  per-cluster wait and run times of §8.5 are in `RigExecBakedScheduleRunReport`, which needs a
  frame to have happened -- and only the parallel executor stamps them, so after a serial frame
  that report says the run was serial instead of printing a table of zeros that would read as
  "every cluster was free". It does report how many clusters the last run ran, in both modes.
  §8.5's per-skin-revision chunk statistics are printed by the structural half:
  `RigExecBakedGeometryReport` writes them (it owns the chunk keys) and
  `RigExecBakedScheduleReport` calls it.
* §7's sources are "Inputs, ChainBase, PropertyChains, RevisionStatic". Two of those are not the
  whole story in the code and both are named rather than papered over. A `Solve` and a
  `Constraint` read their OWN parameters off the stage every frame -- weights, offsets, axis
  masks, world-up vectors -- so Build walks those inputs once
  (`RigExecBakedDeclareInputDependencies`) and leaves each step saying whether any of them varies
  with time and which override indices reach it; the dirty set names those steps when the time
  moved, or an override stands on one of their inputs, or stood on one last run. And a
  `RevisionStatic` that is NOT a source, because its packet depends on the influence table, is
  marked `externalReads` together with every `Derived` step, and its cluster is dirty every
  run; its own value comparison is what keeps the counters saying what the dynamic path says.
* §7 says "skipped steps keep slots and replay stored diagnostics". They also RESET their deltas,
  which the specification does not say and which a test found: a geometry step writes flags
  beside its values -- "the influence table moved", "the packet moved", "the revision executed" --
  and a delta is the one thing last run's answer is never this run's. A fuse that ran while its
  fold was skipped read last run's "the matrices moved" and executed a revision the dynamic path
  did not. `RigExecBakedSkipGeometryStep` writes what a step that compared nothing means.
* §3.1's version table replaced §7's restore closure outright, rather than being implemented
  beside it. The closure shipped first and was sound; it was also the reason a constraint-only
  drag ran every cluster on the biped. Giving each writer its own storage makes the closure empty
  by construction, so `restoreOf`, `RigExecBakedStep::restorePreds` and the run-time fixpoint are
  gone and what is left is one forward cone. The property they bought is still asserted, now
  against the program's tables: `TestEveryPoseWriteHasItsOwnStorage` checks that no two write
  sites share an entry, that every read and carry names a version live at its own point, and that
  the last-version table names the last write of each slot.
* `RIGEXEC_BAKED_SCHEDULE=parallel` is not the default. §5.2 makes it the default "once §8
  passes"; §8.1, §8.2, §8.3 and the first half of §8.4 pass, and the second half of §8.4 --
  "parallel mode faster than serial on the biped and on the drag benchmark" -- does not. See the
  measurements above.
* The cone verifier (`RIGEXEC_BAKED_VERIFY_CONES=1`) compares the whole of a run's mutable state,
  and "the whole" needed defining rather than assuming: what it does NOT compare, and why each
  item is scratch rather than an answer, is listed beside `RigExecBakedRunShadow`. Two candidates
  for that list were found by turning the comparison on, and only one of them belonged there. A
  chain's `spare` is the other half of the published double buffer, so a run that publishes and a
  run that skips the publication hold different generations' arrays there while agreeing exactly
  about `result`. The second candidate was not scratch at all, and the verifier was right: two
  runs disagreed about a commit's `deltas` while agreeing about every input to them --
  `present`, `frames`, `staged`, `outcome`, `deltaOk` -- which can only happen if one of the two
  did not compute them. It did not: the unsplit commit head reads `B.fin[slot]` for each
  candidate as it stood BEFORE the commit and declared that slot only as a WRITE, so a cone
  could skip the commit in a generation that moved
  the slot and leave the delta measured against the commit's own last answer. The split
  arrangement's `CommitDelta` step had always declared those reads; the head declares them now,
  the verifier compares every delta a candidate is present for, and the disagreement is gone --
  0 differences over the eight bakeable fixtures at `RIGEXEC_BAKED_GRAIN_US=0` against 3 per
  generation before. No published value moved on any fixture here -- every delta the two runs
  disagreed about belonged to a commit with no propagation pairs, so the stale number was read by
  nobody, which is why parity never caught it -- but the undeclared read was the same read on a
  commit WITH pairs, and there a stale delta is a descendant left where the ancestor used to be.
* Its equality counts a NaN as equal to a NaN, which `==` does not. A mover whose inputs the
  kernel rejects publishes the packet it rejected, NaN and all -- that is how the pass-through
  diagnostic names the value -- so a non-finite number is ordinary state for a correct program
  here. Comparing it with `==` reported three mismatches on a generation whose cone had skipped
  nothing.
* §7's first-run dirty set [S28] -- "all pose clusters plus the geometry clusters whose adoption
  did not keep `ran`" -- IS what the code marks dirty on the first run of a program, and it buys
  nothing measurable on any rig in the tree. That is worth writing down rather than leaving as an
  implication. The dirty SET shrinks after a value-edit rebuild (5 clusters of 259 on
  `testRigExecInteractive`'s 256-mover fixture, against all 259 for a run that trusts nothing),
  but the CLOSURE of it does not: every geometry cluster of every rig here is downstream of a pose
  cluster -- a skin chunk reads the matrices of its joints -- so the cone of "all pose clusters"
  covers the whole program. Measured on the biped, `Biped_anim`, `spider_legs` and all eight
  first runs `testRigExecInteractive` makes: 259 of 259, 261 of 261, 514 of 514, 4 of 4, 3 of 3,
  and 57 of 57 on the biped, with and without the rule. The rule is still the rule, because a
  chain that no joint drives (a Phase 3 blend shape off its own control, a lattice on a static
  cage) is exactly the case it was written for. It keeps two guards the literal [S28] set does
  not name, both of them the same sentence about what an adopted `ran` does and does not promise:
  a revision whose `staticDirty` its own source step raised this run is dirty however its
  adoption went, because `ran` came across on the revision's IDENTITY and an edit that also moved
  the mover's parameters leaves it holding an answer to a packet the stage no longer has; and so
  is the base reader of a chain whose authored points moved with the same edit.
  But nobody should read the rule as the answer to "a value edit re-deforms the skin": what stops
  a rebuild re-EXECUTING a revision is the fuse's value comparison, which is why the executed
  counters are right either way, and what would stop it re-running the clusters is a
  shorter dependency from a control to a chunk -- the same ladder §6.1's chunk report measures.

Two rules of §2 and §4.3 that a serial run cannot enforce, and where they are enforced instead:
the per-frame assemblers take no token-registry lock -- every `TfToken(const char *)` on the step
path is hoisted to a `TF_DEFINE_PRIVATE_TOKENS` block in `moverGraph.cpp` and `bakedGeometry.cpp`,
while the BIND-time readers (`RigExecResolveRevisionBinding` and the read-phase metadata) keep
their inline tokens because they run once per generation and off any step; and `Snapshots` is not
a source domain -- every record in the run's store is written by a step, so a step that reads the
store declares the steps before it, which `tests/testRigExecBakedSchedule` then checks like any
other read.

The cost table in `bakedSchedule.cpp` is the one place a machine's numbers are written down. It
was fitted by `RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1` over eight frames of `Biped_anim` and should be
re-fitted whenever the shape of a step changes. Calibration is opt-in, runs the serial executor
whatever the mode asks for, and times each step into that step's own accumulator; `Build` measures
nothing, so the same program produces the same schedule on the same machine however busy it is.

Three things the parallel executor and the cone found that a serial full run could not, recorded
so they are not rediscovered. A `RevisionChunk` reads the chain's running value BEFORE its
revision, which is two things: the earlier revisions' buffers, and the `currentSource` indirection
that says which of them to read. It declared only the buffers, so at one cluster per step a chunk
could overtake the fuse that decides the indirection -- deterministically wrong points on
`tests/testRigExecInteractive`, and invisible in every serial order. It now declares both, and the
rule is asserted by `tests/testRigExecBakedSchedule` on a three-revision chain the suite builds
itself. The stale-delta defect above is the second. And `RigExecStaticInputCache` answers a read
from a worker thread by bypassing itself (its owner-thread rule, moverGraph.h), so a step running
off the evaluator's thread resolves its inputs the long way and gets the same value; what moves is
that cache's bypass COUNTER, which `tests/testRigExecStaticInputCache` asserts on only for its own
fixture. That is the §5.2 "check the static input cache before merging" item, and the check comes
back clean: `_bypasses` is already a `std::atomic<size_t>`, which is the one member of the cache
written off the owning thread.

### What holds it to account

* `tests/testRigExecBakedSchedule` -- every edge forward, every read written or sourced, no two
  steps writing one slot without an edge, the report deterministic, the chunks covering every
  vertex once with no missing influence, the cone closure closed, every pose write holding storage
  of its own, a repeated time running less than the whole program, a control held at its own
  authored value executing nothing, and a constraint-input drag and a leaf-control drag each
  publishing the dynamic path's pose from inside the cone the graph says it may reach.
* `testRigExecBakedScheduleCones_{serial,parallel}` -- the same suite under
  `RIGEXEC_BAKED_VERIFY_CONES=1`. It is the only place the VERIFIER'S OWN comparison is exercised,
  because it is the only suite that drives a NaN into a published packet on purpose: a comparison
  that called a NaN different from itself reported three mismatches on a generation whose cone had
  skipped nothing, and the instrument the next stage is told to debug cones with cried wolf on the
  one fixture that most needed it to be right.
* `testRigExecInteractiveCones_{serial,parallel}` and
  `testRigExecExampleParityCones_{serial,parallel}` -- the two suites that drag, run with
  `RIGEXEC_BAKED_VERIFY_CONES=1` under each schedule, so every generation is compared with a run
  of the whole program from the same starting state.
* `example_parity_*` and the `*BakedParity` suites -- both paths in one generation, compared
  exactly, on every shipped rig that bakes.

---

*Everything below this line is the specification as it was written, before any of it existed.
"Today" in it means the straight line Phase 2 replaced, not the code in the tree. It is kept
because the rules are still the rules -- a Phase 3 group adding an operator reads §§4, 6 and 7 to
know what a step may and may not do -- and because the deviations above are only readable against
what they deviate from.*

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
| SolverPoints | solver s | `solvers[s].ribbonPoints` | SolverSources (prologue) |
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
reads a specific VERSION: the one live at its program point. The storage is therefore SSA -- every
writer of a pose slot gets an entry of its own, and every reader binds at Build to the entry
holding the version live where it runs. Slot i's FIRST version is entry i, which is what the
compose writes; its LAST is entry N + i, laid out densely because the matrices, the phased-read
records and the publication read nothing else; the versions in between are an arena past 2N. A
write site that does NOT write -- a candidate its batch published nothing for, a propagation pair
stepped over, a constraint that passed through -- CARRIES the version it found into its own entry,
so every version a reader can name is well formed however the run went.

A carry is a READ, and is declared as one. The step that performs it -- the head of an unsplit
commit, the `CommitApply` of a split one -- declares `PoseFin` of every candidate and, for a solver
commit, `PoseBase` of every candidate and every descendant, beside the delta reads it already
declared. Those are edges the write-after-write pass raises against the same writers anyway, so
declaring them adds no edge and moves no cone; what it keeps true is the graph's account of what a
step touches, which is what the next writer of a commit step will reason from.
`testRigExecBakedSchedule`'s check (6) asserts it.

Cost: Σ|writes| frames of 96 bytes, sized at Build and never resized in a run -- on the biped 1 777
write sites over 326 slots, 2 015 `fin` + 652 `base` entries, 270 KB. A read is one indirection
through a table Build computed.

This is what makes "clean steps keep last run's values" sound, and unlike the alternative it costs
cone re-execution nothing. The alternative, which earlier drafts of this document specified and
which the first implementation shipped, was a RESTORE CLOSURE: with one storage per slot, the end
of a run holds only the LAST writer's value, so re-running a reader of an earlier version meant
re-running that version's writer first, transitively. It was sound and it was ruinous on a drag --
a constraint read-modify-writes its target, so restoring what it read pulled in the compose that
wrote it and, with it, everything downstream of that compose, which on a biped is the whole
program. Versioned storage retires it by construction: no version is ever overwritten, so nothing
ever has to be re-run to put a value back. See "What a drag costs" for what that changed.

The same sweep tells the edge construction one more thing. A write-after-read edge exists only
where a writer can land on storage a reader is still entitled to, and in a versioned domain it
never can, so §4.1's WAR pass skips `PoseFin`/`PoseBase` (`RigExecBakedIsVersionedDomain`).
Nothing else about §4.1 moves: read-after-write and write-after-write edges still come from the
declared slot RANGES, and the version table is derived from the same lastWriter sweep.

## 4. Steps and the graph

`struct RigExecBakedStep`: `kind`, `payload`, `reads`/`writes` (sorted range lists), `preds`/`succs`,
`cluster`, `cost`, per-step preallocated scratch, `std::vector<std::string> diagnostics` (at most
`kMaxStepDiagnostics = 4` lines per run; every diagnostic site in the walk is terminal [S17]; no
`reserve()` at Build), per-step counter deltas, `bail` flag.

Program order is today's `Run` order. The prologue and epilogue are serial code, not steps.

A ribbon's driver curve is the one solver input that is scene data: the dynamic path reads the
attribute with `UsdAttribute::Get` and hands exec the value as a packet override, honouring neither
connections nor the resolved inputs. It is therefore read the same way and in the PROLOGUE
(`RigExecBakedRunSolverSources`), into `SolverPoints`, and compared by value there -- a step body
may not touch USD, and "the time moved" is never the predicate (§7).

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
4. Joint publication: two passes, not one. The first walks `_jointPaths` order and settles what
   the order is observable through -- the degenerate final-frame diagnostic, and the ONLY bail in
   this block, `!_Usable(restFrames[slot])` for a valid, non-degenerate final frame → return false
   [P33], which now happens before a single key is published rather than with the maps half
   filled. The second fills `jointFramesBase`, `jointFramesFinal` and `jointMatricesFinal` (from
   `FinalMatrix`) in PATH order, with a hint at each map's end, so a key costs one comparison
   instead of a search from the root. The publication lists are not themselves in path order -- the
   biped's joints and controls both arrive in binding order -- so Build sorts a permutation of each
   (`jointPublishOrder`, `controlPublishOrder`, `solverPublishOrder`) and records whether it is
   STRICTLY ascending; a list that names one path twice is published the old assigning way, because
   an emplace would keep the first value where the assignment kept the last. The permutation is
   built with `std::stable_sort` for that fallback's sake: both fill loops walk the permutation
   whichever branch they take, so only a stable order leaves two equal paths in publication order
   and makes last-wins mean what it meant before. Control frames the same. Solver guides: read `B.aggregates` directly under the runtime toggle
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

### 6.1 As built

The chunked skin landed as described above, with six differences the code makes and this section
records so the two do not drift:

* **The packet carries an IDENTITY influence table** (`GeomRevision::packetInfluences`, written once
  at Build). `RevisionStatic` must not depend on the matrices, and `RigExecAssembleSkinParameters`
  puts them in the packet and validates them there -- so the packet gets a table of the right SHAPE
  whose elements trivially pass the assembler's finite/affine check, and the real table's check is
  `InfluenceFold`'s (`RigExecSkinTransformsAreUsable`), which the fuse ANDs in exactly where the
  assembler's answer would have landed. The half of the executed decision that used to read
  `skinTransforms == o.skinTransforms` is the fold's own compare. Consequence, and it is the point:
  `revision.status` for a revision with a bad influence says "ok" where the dynamic path's says
  "moverFailed", and the fuse publishes `moverFailed` anyway through the `!applied` branch -- the
  published token, the diagnostic and the executed counter are identical, the intermediate is not.
* **The step order is Static, Fold, chunks, Fuse for a skin revision** and Fold, Static, chunk, Fuse
  for every other operation, whose packet does carry the matrix it was folded from. The fold reads
  `RevisionPacket` for the skinning method, which decides which form of the table the chunks want.
* **The `inputs:defaultWeight` diagnostic moved from RevisionStatic to RevisionFuse**, which is the
  first step that knows both halves of "the packet is valid". Diagnostic ORDER is unchanged: no step
  between them emits one.
* **Only a skin revision whose layout the epoch fixed (`skinTopologyFixed`) is cut into more than
  one chunk.** A layout that can move within the epoch is one a Build-time cut cannot promise
  anything about, and such a mover already re-reads and re-validates every element of its arrays
  once per frame. Everything else is one chunk over the whole array -- the degenerate case of the
  same step, which then skins against the fold's table through the full-range kernel and keeps the
  threading it has today.
* **A chunked DQS revision builds its own palette per chunk** rather than reading
  `RevisionTransforms`: the split is per matrix, so a chunk's palette agrees entry for entry with
  the revision's over every entry the chunk's own vertices index, and keeping the chunk off the
  fold is what the speculation is for. Per-chunk rows are maintained the same way, entry by entry
  beside the matrices, so a run costs |key| narrowings rather than |influences|. The fold still
  writes the revision's own rows and palette for every skin revision, chunked or not, because the
  fuse's whole-array fallback skins against them and a step may only READ a slot it declared as a
  read. `tests/testRigExecBakedSchedule` covers both halves: that a range deformed against a table
  that is identity outside its key is bit-identical to the whole array (both methods), and that
  the biped skinned with dual quaternions through an interactive override holds parity with the
  dynamic path, which is the only chunked-DQS fixture there is -- every rig that bakes today is
  classicLinear.
* **A skin revision's packet is assembled without a transform matrix.** The fold owns the matrix
  and runs AFTER the static step for a skin, so the matrix available at assembly time would be the
  one last run measured. Nothing reads it (`RigExecAssembleSkinParameters` takes no transform,
  which is also how the dynamic path treats a geometry-domain delta landing on a skin mover), so
  the packet is assembled without one rather than with a stale one.

Two mechanisms exist that the specification does not name:

* **Re-cut in the prologue.** The partition is Build state cut from the authored arrays; the packet
  carries the layout the evaluator's skin topology cache resolved. That cache hands back the SAME
  pointer for a binding that did not move, so the prologue re-cuts (serially, keeping the chunk
  COUNT, which is the step count and may not change) exactly when the pointer changes -- once per
  epoch, and to the same ranges when the arrays are the same, because it runs the same algorithm.
  `RevisionStatic` still checks in O(1) that the partition describes the packet's layout, and a
  `partitionStale` revision is run WHOLE by the fuse: a chunk skinning a vertex against an identity
  it never noticed is a silently wrong deformation, so the keys are trusted only while they are
  provably current. A REFUSED layout (the cache declining the mover) leaves both the partition and
  the handle it was cut from standing, so the comparison sees a packet with no layout at all and
  the revision goes whole -- recording the refusal would make the two handles agree by both being
  null, which is the one answer that comparison must never give. The fallback has a fixture of its
  own (`TestAStalePartitionRunsTheRevisionWhole`), which makes the partition disagree with the
  packet by hand -- no stage can -- and asserts the whole-array run publishes the chunked run's
  points.
* **A chunk that skipped keeps its answer.** Its gate is `(ChainDirty(r-1) || staticDirty ||
  its own key moved) || !ok` -- the first three decided while the key's matrices are copied in,
  which costs nothing extra, and the last so that a chunk with no answer to keep is never the one
  that keeps it.
  Another chunk's joints moving makes the REVISION execute; it does not make this range's vertices
  land anywhere else, and the range of the output buffer still holds their positions. `ok` is
  sticky for the same reason.

Measured on the biped (`tests/testRigExecBakedSchedule` prints it): the one skin revision holds
26 276 vertices over 137 influences and cuts into 7 chunks of 4 096 (the last 1 700), |key| 24 min /
41.3 mean / 58 max, and NO chunk reaches half the influences -- so no vertex range waits for more
than 42% of the rig's joints, against 100% before.

## 7. Cone re-execution (drags and static frames) [S24-S28][P20-P22]

Source steps are the only steps that read outside the program: Inputs, ChainBase, PropertyChains,
SolverSources (all prologue), RevisionStatic, and Phase 3 weight-object readers. Every other step is a pure
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
Run:    dirty = everRan && stamp == lastStamp ? OR of bit(c) for sources that wrote a different value
                                             : ALL pose clusters ∪ geometry clusters with !revision.ran
        closed = OR of coneOf[c] for dirty c
        remaining[c] = closed[c] ? |preds[c] ∩ closed| : skipped
```

That is the whole closure. There is no restore closure beside it: versioned pose storage (§3.1)
leaves the version a clean reader wants in the entry its writer put it in, however often the SLOT
was revised afterwards, so a run never grows to put a value back.

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
   STATUS: measured at a third of every baked drag frame and narrowed -- the clear now fires only
   for an override that can REACH a skin mover's layout along the connection walk the layout is
   read through. See "What a drag costs".

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
   dirty while its target's compose is clean (§3.1: it must publish the dynamic path's pose while
   running strictly fewer clusters than the program has).
4. Performance (informational, `rigExecPose --profile` with prologue / region / epilogue as
   separate lines [S30]): serial mode within 5% of today's `Run` on the biped frame; parallel mode
   faster than serial on the biped and on the drag benchmark. Expectation, not a target: the
   epilogue's map publication (~50-150 µs) is irreducible serial work, so the biped frame should
   land around 2-2.5× faster than serial, not 8× [S30]. A serial regression means per-frame work
   that belongs at Build.
   STATUS: the first half passes -- serial is 662us against `b705950`'s 757us, 12.5% faster, not
   within 5% of it -- and the second half does not: parallel is 705us. `--profile` turned out to be
   the wrong instrument for a frame this size and was replaced by `--repeat` and
   `RIGEXEC_BAKED_STEP_TIMING`; see "What a frame costs" for the numbers and for the three control
   experiments that say where the parallel time goes.
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
