# What is still serial in the dynamic and baked frames, and why the brow joints drag slowly

Date: 2026-09-13. Commit 903c012 (biped-port). Source traces: `reports/biped_anim_landed_dynamic.trace` and
`reports/biped_anim_landed_baked.trace` (`build/rigExecPose examples/biped/Biped_anim.usda --frames 1,2,3,4,5,6,7,8
--profile <out> --mode dynamic|baked`). Question asked: which per-frame scheduling in the two paths could be
parallelized further without a mutex, and what general scheduling changes make interactive drags (the brow
controls) execute better.

Every number is a median over frames 2-8 (frame 1 is a warm-up outlier), minimum over repeated runs, on this
20-core aarch64 box. Unless marked "estimate", a number was measured this session: 49 probe scopes in an
instrumented copy of the tree for the dynamic path, 30 for the baked path, tap-set probes splitting every exec
request, a drag benchmark added to a copy of `rigExecPose`, standalone timing of the usdview gizmo's Python,
and one prototype build per candidate that survived review. The live checkout was never modified. Rule applied
to every evaluation-internal proposal: no new mutex. Allowed mechanisms were fork/join into private per-task
buffers merged on the calling thread, `WorkParallelForN` over disjoint ranges, compile-time level partitioning,
fusing work into an engine that already runs in parallel, and removing dead work where it changes what is left
to parallelize. Every candidate below was then attacked by three independent reviewers (shared state and the
no-mutex rule; exec/USD safety and result equivalence; gain realism against the measurements), most of whom
built the change in a scratch copy and compared outputs byte for byte.

## 1. Summary

- Both traces run every per-frame stage on one thread. The only in-frame parallelism is the skin kernel's
  point-range split (invisible in the trace) and exec's own parallel evaluation engine inside each request.
- The largest untapped serial work in the dynamic path is the constraint commit's descendant propagation:
  1.6 ms of a 6.7 ms frame, hidden inside the 65 constraint scopes. It is not a parallelism problem. About 95%
  of it is `std::map<SdfPath,...>` traversal; a dense-slot version of the same loop runs in ~0.1 ms.
  Parallelizing the residue is worth ~50 us and is not recommended.
- Exec's parallel evaluation engine is a net loss on this rig's small requests. Turning it off with
  `VDF_ENABLE_PARALLEL_EVALUATION_ENGINE=0` takes the live dynamic frame from 6.74 to 5.60 ms with byte-identical
  output, leaves rigExec's own kernel splits untouched, and needs no code change. It is a process-wide launch
  setting, not something rigExec can set from inside.
- The remaining dynamic wins are serial removals (dead parity-only matrices, an unread chain-snapshot record,
  identity-map prefills, per-frame set rebuilds, the skin node's two serial marshalling passes, the derived
  chain's three 26k-point copies, ~1.0 ms together), one level-per-request fusion of the solver batches (~0.4 ms
  with the parallel engine, ~0.07 ms without), and two verified forks: the provider-matrix loop running under
  the authoritative snapshot request (~0.1 ms net) and retiring the frame's dead containers on a worker
  (~0.2 ms).
- Every trace number is a guides-off number: `rigExecPose` disables solver guides, while the library default and
  the imaging bridge keep them on. A viewport or scrub frame pays a fifteenth exec request of ~0.6 ms plus
  ~0.1 ms of override construction on top of everything below (baked: ~0.07 ms).
- The baked frame (~0.7 ms) has no exec in it. Its serial blocks are 756 map inserts (0.11 ms), the property
  chains (0.12 ms, halved by precompiled handles), and a skin kernel whose 20-core split reaches 1.55x. The one
  fork that was built for it (publish under the chain) measured slower, not faster.
- The brow joints are slow for a reason outside the evaluator: the usdview gizmo recomputes the target's rig
  frames in Python on every mouse sample, and that computation is superlinear in RigExec depth. It costs 320 ms
  for a brow joint at depth 20 and 1 ms for the hips control. A per-call memo brings it to 5-7 ms with
  bit-identical frames. Below that, usdview runs the evaluator in dynamic mode (7.3-8.1 ms per move) although
  the baked program places every gizmo override and costs 1.4 ms; publishing adds a flat 2.5 ms per move;
  every stage edit costs a 24.5 ms structure-digest check.

## 2. What the traces show, and what they hide

Dynamic frame, 6.5-6.9 ms. Rig: 326 pose providers (252 joints, 74 controls), 65 constraints, 14 solver
batches, 12 property chains, one 26,276-point skinned mesh with one derived extent, no connected providers, no
ribbon or volume prims.

| stage | us | what is inside |
|---|---:|---|
| constraint steps (65 scopes) | 2040-2150 | 2060 of it is `commitConstraintFrames`: descendant propagation 1642, enumeration 346, write-back 174 |
| 14 solver batches | 1370-1480 | 14 exec requests of one tap each; Compute ~74 us per request, the kernels ~1 us each |
| AuthoritativeSnapshot | 830-930 | one exec request, 1042 taps, 352 overrides: Compute 811, extraction 90, key construction 32 |
| publish gap (no scope) | 705-728 | provider loop 326, joint loop 245, control loop 25, parity-only matrices 130 |
| PoseSeed | 400-460 | Warm (a full Compute, 304) then ComputeWithOverrides (90) plus extraction 47 |
| Chain body_geo.points | 440-510 | GraphEvaluate 267 (skin, parallel inside), Derived extent 166, assembly ~60 |
| seed-to-walk gap (no scope) | 236 | identity-map prefill 73, provider-set rebuild 73, seed map fill 41, intervening-xform queries 15, rest map copy 13 |
| PropertyChains (12) | 150-180 | 6-17 us per chain, all 12 independent |
| teardown (no scope) | 135 | destruction of the frame's local maps and override vectors |

Baked frame, 640-820 us (median ~740; the spread is the skin kernel).

| stage | us | what is inside |
|---|---:|---|
| Chain body_geo.points | ~295 | skin kernel 169 median (80-198 bimodal; 262 serial), copies 26, Derived extent 79-81 (extent 28, packet copy 26) |
| BakedMatrices | 131-133 | 756 `std::map` inserts 114, control publish 12, matrix math 7 |
| PropertyChains (12) | 117-122 | same routine as the dynamic path |
| pose walk (14 batches, 65 constraints) | 138 | solver kernels 12 us in total, constraints 1.7 us each; plus 17 us of per-step profiler scopes |
| BakedCompose | 30 | serial parent-before-child compose |
| BakedInputs | 4 | |

Measurement notes. A profiler scope costs 0.2-0.5 us and takes a mutex; the 79 per-step scopes in the baked
walk are 17 us of the 155 us reported. Run-to-run wall spread is +-80 us on the baked frame and +-2-7% on the
dynamic frame, so anything under ~100 us must be measured as an in-trace scope differential, never as wall
time. Half of the verification measurements were taken while sibling builds loaded the box; those agents
reported ratios and in-run A/B deltas, which are what is quoted here. One rig, one animation, no drag, guides
off: the biped never passes the three-target gate of the existing level-parallel chain walk, and its
rest-override, volume-placement and ribbon paths cost zero here and are not free on rigs that use them.

## 3. Dynamic path

### 3.1 Exec's parallel engine costs more than it saves here (measured; no code change)

exec picks its main executor once at `Exec_Runtime` construction from `VDF_ENABLE_PARALLEL_EVALUATION_ENGINE`
(pxr/exec/exec/runtime.cpp:53) and picks a parallel sub-executor for override requests above the 32-node
small-schedule gate (runtime.cpp:259, vdf/scheduler.cpp:2581). Live binary, quiet box, 3 runs each; the
interleaved A/B repeated under load gave the same 1.1-1.4 ms gap:

| scope | parallel engine | serial engine |
|---|---:|---:|
| frame | 6740 | 5600 |
| PoseSeed | 432 | 267 |
| 14 solver requests | 1069 | 549 |
| AuthoritativeSnapshot | 907 | 787 |
| GraphEvaluate (rigExec's own split) | 261-312 | 321 |
| baked frame | 748 | 742 |

The tap-set probes say why: a one-tap solver request costs 74 us of Compute with the parallel engine and 35 us
without, for a kernel the baked program runs in about 1 us. The engine's task-graph dispatch is the cost.
`PXR_WORK_THREAD_LIMIT=1` reaches a similar frame but loses the skin split (GraphEvaluate 287 -> 449), so the
VDF setting is the right knob.

Review outcome. Output is byte-identical between engines (`--joints-out`, `--targets`, parity mode with zero
mismatches, all 62 registered tests with identical exit codes). Three reviewers re-measured the saving at
800-1100 us; one later 20-frame run under heavy sibling load found no difference (6.17 vs 6.36 ms, with the
solver requests cheaper and the snapshot and geometry dearer), so re-measure on a quiet box before deploying. Constraints: the setting is read and cached before `main`, so `setenv` from inside rigExec does
not flip it; it must be in the process environment (launcher, wrapper, farm or CI env). It is process-wide and
disables parallel evaluation for every other exec client in the host, so it is a deployment knob, not a library
default. It prints a three-line TfEnvSetting banner on stderr unless `TF_ENV_SETTING_ALERTS_ENABLED=0`. rigExec
should read `VdfIsParallelEvaluationEnabled()` into its diagnostics so a deployment that failed to set it is
visible. It is anti-additive with 3.3 below.

### 3.2 Constraint commit propagation: 1.6 ms, and the lever is storage, not threads

`commitConstraintFrames` (rigEvaluator.cpp ~8650-8740) runs 59 times per frame. Per commit it enumerates the
namespace descendants of each candidate by scanning `finalFrames` from `lower_bound`, and per descendant climbs
to the closest candidate, asks `nearestBlocking` (a memo rebuilt every frame, filled by reading `parent:space`
off the stage), recomputes `RigExecPointsToMatrix(before, candidate)` although the candidate is the same for
nearly every descendant, applies the delta, and inserts into a per-commit map. Cost fits 0.48 + 0.673 us x
descendants per step; the 17 spine and hips steps with 122 or more descendants account for 1464 of the 2042 us.
The baked program builds the whole propagation table once at bake and its 65 steps cost 1.7 us each.

Two reviewers rebuilt the loop standalone over the real 326 biped provider paths and the real kernels. The
current shape costs 0.56 us per descendant, matching the in-tree 1642/2952. The same loop over pre-resolved
`RigExecPointFrame*` slots with a vector for the propagated frames costs 0.04 us per descendant; the two kernels
alone are 0.034 us. So the per-descendant cost is red-black-tree traversal with `SdfPath` compares
(`finalFrames.at`, `finalFrames.find`, `candidates.count` per climb level, a `propagated[]` node allocation,
the write-back `finalFrames[path]`), not math and not stage reads (the stage reads over all 326 providers cost
75 us).

What survives review. Precompute at Compile only what is epoch-constant: per step, the dense descendant slot
list and each descendant's parent-slot chain, in the same sorted order the walk uses today. Per commit, resolve
the closest candidate against this frame's candidate set, with the baked program's bail guard (fall back to the
live climb when a precomputed closest is not a candidate this frame; the six solver-batch commits have
runtime-variable candidate sets). Memoize the per-candidate delta lazily, in descendant order, so the
"singular hierarchy delta" diagnostic keeps its position. Keep the all-or-nothing write-back. Expected saving:
1.4-1.9 ms per frame.

What does not survive. The BLOCKED bit cannot be precomputed: `ownsItsPose` reads the `parent:space` value at
the frame time, so a time-sampled `parent:space` flips it within an epoch with no notice, and a value-only edit
leaves the digest unchanged. Keep that predicate per frame (the biped authors no `parent:space`, so the memo is
cheap once the map traffic is gone). The parallel route is refuted by this lens rather than in spite of it:
once the map traffic, the memo and the stage reads are hoisted out of the region as the no-mutex rule
requires, the residue is 0.017 us per descendant, about 50 us per frame, below any fork's cost.

### 3.3 One exec request per dependency level instead of per solver

Compile emits one `_SolverBatch` (one tap set, one request) per solver although `_SolverBatch::solvers` is a
map: 14 requests per frame, 7 at level 0, 2 at level 1. A reviewer built the fusion (7 batches: L0 holds the two
arm FK chains, two two-bone IKs, two leg FK chains and the neck spline IK; L1 the two arm IK/FK blends) and got
byte-identical `--joints-out` and `--targets` output in dynamic, baked and parity modes.

Measured mechanism, contention-normalized against one single-solver request S: today's 7 L0 requests cost
7.9 S; the fused 7-solver request costs 2.75 S; total solver exec 15.9 S -> 9.3 S, -41%, about 420-515 us on
the quiet baseline. The win is exec overlapping the 7 independent aggregate roots inside one request, not the
removal of a fixed per-request cost. Under the serial engine (3.1) the same fusion saves only ~14%, about
70 us, so the two must not be summed.

Required changes: keep one `commitConstraintFrames` per solver in compiled order (the full saving is obtained
without the union commit, and with it the ownership-equivalence argument becomes load-bearing); keep a
per-solver dirty bit inside the fused batch so `pose.solverEvaluations` counts solvers actually recomputed
(four assertions in testRigExecConstraints.cpp at 2818 and 3127 fail otherwise, all on that counter); add a
compile-time level-splitting guard for the unmodeled same-level read-after-write through namespace propagation
(a solver whose frame input is a namespace descendant of another same-level solver's joint); do not fuse a
level whose frame inputs include a connected pose provider. Downside: an interactive edit to one solver's
input re-evaluates the whole level.

### 3.4 The publish region after the snapshot (728 us)

Serial part first, measured in an instrumented copy: `_chainSnapshots.RecordFinal` for all 326 providers has
no reader (every final-phase consumer wants a `VtVec3fArray`; final-phase transform reads are served from
`finalMatrices`) and removing it cuts the provider loop from 329 to 100 us; the `baseProviderMatrices` and
`finalProviderMatrices` build is consumed only inside `if (cpuParityMode)` and is 130-150 us of dead work by
default; `emplace_hint(end())` on the joint publish saves 30-74 us (it saved nothing on the provider loop).
Together 350-400 us. Parallelizing the joint loop itself measured slower than serial (35-51 vs 27 us).

Fork P1, built and verified: the provider-matrix loop (reads only `finalFrames`, `restFrames`,
`xformDerivedBases`) runs on a `WorkDispatcher` task while the AuthoritativeSnapshot exec request runs on the
calling thread. Output identical with both engines; the task finished 476-585 us after the fork while the
request ran to 2160+ us, and the nesting works because exec's engine lives in its own isolating arena. Two
corrections are mandatory: join the task before the `snapshot.IsValid()` check (the failure path pushes to
`pose.diagnostics` while the task would still be writing it), and give the task private buffers whose
diagnostics are spliced at the index the serial walk would have used. Net after the serial part: 90-130 us.

Fork P2 (joint, control and property publish under the chain walk) is refuted as stated in the default
configuration: the solver-guide exec request sits inside the proposed span and pushes diagnostics on the
calling thread, so the task must be joined before the guides block; the untargeted-descendant `finalMatrices`
fill (70-75 us) must run before the fork because `body_geo_skin` reads final-phase matrices from it; the
property publish must not move. Estimated net 100-140 us; not built. Order: serial part, then P1, and P2 only
if the remaining ~175 us justifies a second fork.

### 3.5 Property chains: precompile the handles, do not fork

A reviewer built both halves. Precompiled per-revision handles (prim, attributes, parsed `rigExec:operation`,
resolved weight-object targets, static tokens) plus the attribute's `SdfPath` (so the hot path never calls
`UsdAttribute::GetPath`, which interns a node on every read) take the baked PropertyChains scope from 115 to
69 us with bit-identical output. The parallel version (one task per chain, private buffers, deferred publish)
measured 67 us at best and typically 20-60 us worse than the serial fix: the twelve chains are too small, and
worker threads bypass the static input cache (measured at ~100 us of the scope). Constraints on the serial fix:
precompile classification and handles, never values (the epoch digest is value-blind); rebuild the table in
Compile and on the notices that rebuild chains, not per epoch (a deleted or retyped target attribute must keep
producing its diagnostic); keep the per-frame validity check on the target.

### 3.6 Smaller items, each with a verdict

- Seed-to-walk gap (236 us): replace the per-frame `hierarchicalProviders` set with `_poseSeedFrames.count`
  (73 us, three lines); move the 326-entry identity prefill in `_ComposeInterveningXforms` below its
  `anyIntervening` early-out, refilling with `emplace` (84 us, parity-verified); optionally a copy-on-write rest
  map (13 us). The fork variant is refuted (its payload after these removals is the 15 us of intervening
  queries, and part of its cover is the seed request). The compile-time "is any intervening prim an Xformable
  with authored ops" filter is refuted on equivalence: the digest never hashes xformOps, so authoring one on an
  intervening prim would not recompile.
- Snapshot extraction (`tapSet.cpp` loop over 1042 taps, 90 us): a legal `WorkParallelForN` (the Get path is
  read-only; upstream tests extract concurrently), measured 116 -> 73 us with 64 fixed ranges, `resize` and
  indexed stores, `complete` computed by a serial pass after the join, gated on tap count >= 128 so the 14
  one-tap requests stay serial, wrapped in `WorkWithScopedParallelism`. Worth 30-60 us; the smallest item here.
- The pose seed's Warm pass (304 of the seed's 440 us) is refuted as a target in all three forms. It is the only
  Compute on the main executor per frame and therefore the producer of the cache every later request reads;
  dropping it is byte-identical but makes the frame 0.02-1.2 ms slower depending on load. Warming only the
  override closure warms exactly what the sub-executor cannot read. Skipping it at a held time during a drag
  saves ~30 us per move. One free adjacent fix: guard the Warm with `if (!baseOverrides.empty())`.

### 3.7 What the completeness pass added

- Solver guides. `_solverGuidesEnabled` defaults to true and the imaging bridge reads `pose.solverFrames`, but
  `tools/rigExecPose.cpp` turns guides off, so no trace in this study shows the request. Re-measured with the
  disable behind an env var: the dynamic frame goes from 7530 to 8238 us on today's loaded box (the SolverGuides
  scope is 588 us; ~120 us is building `guideOverrides`, a copy of `baseOverrides` plus one frame override per
  provider), baked +71 us. The fix is the same mechanism as 3.3: fold the guide taps into the authoritative
  snapshot request, or filter the override tuple so `sameGuideInputs` can hold; during a drag, disable guides.
- Skin marshalling inside GraphEvaluate. The skin node collects the whole preceding buffer into a
  `std::vector` through a `VdfReadIterator` (69-89 us), runs the parallel kernel, writes 26,276 points back one
  at a time through a `VdfReadWriteIterator` (55-125 us, bimodal), and `RigExecMoverGraph::Evaluate` then
  value-initializes a fresh `VtVec3fArray` and copies element by element. 155-245 us per frame of serial work
  bracketing the only parallel region of the dynamic chain; run the kernel in place through a random-access
  accessor and extract with `ExtractAsVtArray`.
- Derived-chain tail. Producing the two-point extent copies the 26k graph points three times (`values.basePoints`,
  `parameters.auxPoints`, `deferred.points`) and element-compares them against the previous frame, a compare
  that cannot succeed on an animated rig. About 95 us per frame in both modes.
- Teardown on a worker. The 135 us of `_EvaluateDynamic` local destruction plus 33-75 us of caller-side pose
  destruction (outside every scope) depend on nothing. Move the dead containers into a heap holder and destroy
  them on a `WorkDispatcher` task held by the evaluator: 170-200 us dynamic, 35-80 us baked, no shared state.
- Dense provider indices for the walk itself. 3.2 and 3.4 make two sites dense; the walk's own working set
  (`baseFrames`, `finalFrames`, `restFrames`, `xformDerivedBases`, `finalMatrices`, `constraintDeltas`, the
  per-commit `propagated` map) is still rebuilt per frame over the same 326 epoch-constant keys. A
  compile-assigned index per provider turns every lookup into array indexing and is what makes each later
  fork's private-buffer merge cheap; roughly 200 us beyond 3.2.
- Structural options, not costed: with the serial engine freeing 19 cores, the next frame's exec-free prologue
  (property chains, prologue, ~380 us) could run ahead under the current frame's requests, and two baked frames
  could overlap end to end (~160 us ceiling), but `Run` writes evaluator members (`_resolvedInputs`,
  `_chainSnapshots`), so neither is possible until that state moves into the program.
- On a multi-mesh rig, derived maintenance runs inside the chain task although nothing reads it until the frame
  ends; a third phase after the last chain level, merged in chain order, would take it off the level's critical
  path. Worth zero on the biped.

## 4. Baked path

Nothing in the baked frame touches exec, so the "fuse into exec" mechanism does not apply; the frame is three
serial blocks of similar size plus the skin kernel.

- BakedMatrices is container work: 114 of its 133 us are 756 `std::map<SdfPath,...>` inserts at 0.15 us each,
  7 us is matrix math. Sorting the (path, slot) pairs once at bake and inserting with `emplace_hint(end())`
  (guarding duplicate keys, since a hinted emplace does not overwrite) measured BakedMatrices 135 -> 46 us and
  the frame 809 -> 702 us median in a 26-run interleaved A/B, byte-identical output. The fork of this publish
  under the geometry chain was built with all its preconditions (eager matrices, hoisted fallback decision,
  private diagnostics, property and guide publish above the fork, parity clean) and is worth 28-36 us on top of
  the serial fix; alone, it measured slower in one build (756 node allocations on a TBB worker beside the
  52-task skin split) and 30-60 us faster in another. Do the container change; treat the fork as a late,
  small item.
- Property chains are 16.5% of the frame; the serial fix in 3.5 applies unchanged.
- Skin kernel: 52 tasks at grain 512 reach 1.55x on 20 cores (169 us median, bimodal 80-198; 262 serial). The
  extent (28 us serial) and the 26 us packet copy in `Assemble body_geo` read the same array right after it;
  folding the min/max into the skin pass as a per-task private range would remove both (~54 us, estimate). A
  parallel extent on its own measured 26.7 -> 21.0 us and is rejected. The layout carries 69% zero weights
  (3.1 real influences per point against elementSize 10); compacting it at bake cuts the kernel's work about
  3x. The split saturates at about 4 threads (93 us), so it is bandwidth or fixed-cost bound and folding the
  extent into the same pass reads points already in cache; do that before any fork.
- Rejected on the 10k-cycle rule: level-parallel BakedCompose (30 us total), parallel solvers within a batch
  (12 us total), any partition of the constraint walk (1.7 us per step).

## 5. Interactive drags, and the brow joints

The biped has no brow controls. The brow prims are six leaf `RigExecJoint`s 17-20 levels deep under
`head_tip_bind`, authoring only `rest:space`, listed only as skin influences. Dragging one in usdview runs,
synchronously on the UI thread per mouse sample: gizmo `_UpdateDrag` -> `gizmoPreview.Push` ->
`RigExecImaging_UpdatePreview` -> `SetInteractiveOverrides` -> `SetTime` -> `EvaluateAndPublishResult` -> Hydra
dirties -> `drag.target.Refresh()` -> `_RebuildHandles` -> repaint.

### 5.1 The brow-specific cost is the gizmo's Python frame replica

`gizmoMath.ComputeRigFrames` recomputes the target's rig frames from scratch on every sample. `RestSpace` and
`_ComputedSpace` recurse to the rig root without memoization; for a joint whose `default:space` is identity,
`_ComputedSpace(PARENT_SPACE)` re-enters `ComputeRigFrames` on the parent, which repeats the whole default-space
family one level up. Reproduced here (schema plugin registered, Biped_anim frame 1; a reviewer measured the same
at frame 3 and found 82% of the brow call inside `_ComputedSpace(PARENT_SPACE)`):

| target | RigExec depth | ComputeRigFrames | per-call memo prototype |
|---|---:|---:|---:|
| hips_ctl | 4 | 1.1 ms | 0.85 ms |
| chest_bind | 11 | 38.6 ms | |
| skull_bind | 17 | 182 ms | 4.3-12.6 ms |
| brow_inner_l_bind | 20 | 320 ms | 5.3-6.8 ms |

One brow call is ~330k Python calls: 34k `ScalarAvar` reads, 6k `FindRigRoot` walks, 5.2k `RestSpace`
entries, 5.2k `InterveningXform` calls each constructing a fresh `UsdGeom.XformCache`. The memo (RestSpace,
_ComputedSpace, InterveningXform, FindRigRoot, _FindParentXformable, ScalarAvar and one shared XformCache,
scoped to one top-level call) was verified bit-identical over all 326 rig xformables in five preview scenarios
including pivot mode, and byte-identical on the cycle and refusal probes. Requirements from review: key
`ScalarAvar` on its fallback too; key `_FindParentXformable` on the rig root; keep the memo separate from the
existing `_frameCache` (whose `None` entry is the cycle sentinel); cache successes only, never a raised
`_PoseAuthorityError` or `ValueError`; thread it through the existing `_frameCache` parameter rather than a
module global. Two proposed variants were refuted: holding drag-invariant frames for the whole drag (pivot mode
changes rest, restLocal, P and default per move, and the notice hook that would invalidate the hold is dead
code while a drag is active), and reading the published joint frame from the bridge (it publishes the final
frame where the gizmo needs the base frame, and has no freshness tie to the current sample). After the memo the
Python frame cost is still the largest item of a baked-mode move (5-6 ms of ~10 ms), so it demotes the
candidate rather than retiring it; a depth-30 rig would sit at ~10 ms.

### 5.2 The evaluator does no downstream pruning, and is 5x faster baked

Drag benchmark (C++ evaluator only, frame 3, 30 steps, median, min of 3 runs; usdview also pays ~685 us of
solver guides per dynamic move that the benchmark disables):

| what is dragged | dynamic preview | baked preview | dynamic, authored per step | baked, authored per step |
|---|---:|---:|---:|---:|
| brow_inner_l_bind avars:ty (leaf, depth 20) | 7326 | 1409 | 35420 | 27929 |
| hips_ctl avars:ty (everything moves) | 7971 | 1391 | 33728 | 27842 |
| arm_l_fk_wrist_l_bind avars:rx (leaf control) | 8088 | 1415 | | |
| nothing changed (plain repeat) | 4740 | 540 | | |

- All 14 solver requests and all 65 constraint steps re-run whatever is dragged. Every batch's input vector
  starts with all of `baseOverrides`, so one changed avar fails every `sameInputs`; the constraint walk has no
  input cache. For a brow drag none of that work is needed.
- The baked program placed every override (31 of 31 generations, no refusal; avars are schema-declared, so
  `bind()` registers them as overridable even when unauthored). usdview runs dynamic because nothing in
  `libs/rigExecImaging` calls `SetEvaluationMode`. Parity over 8 frames, with guides, and with drags on the brow,
  hips and torso is clean. Two exceptions found in review: the pivot gizmo lane overrides `rest:tx` and
  `rest:space`, which the program refuses, so pivot drags silently run dynamic; and in the gizmo's Default write
  mode every commit of a non-animated avar lands in the bake's capture index and rebakes (26-35 ms on mouse-up).
- `SetInteractiveOverrides` clears `_staticInputs` and `_skinTopologies` on every call. The skin-layout rebuild
  (re-read and range-check 262,760 indices and weights, then compare against the previous layout to hand back
  the same pointer) costs 228-278 us per move in both modes; the cold static cache adds ~190 us to the property
  chains. A reviewer built the key-set-only clear and measured 600-700 us saved per move.
- The authored lane (a slider that writes per change, or a commit) costs 28-35 ms per step, of which
  `_ComputeStructureDigest` in `_SettleEpoch` is 23.5-24.9 ms and never changed the digest; Compile runs the same
  digest on a `WorkDispatcher` task, `_SettleEpoch` runs it inline. The viewport gizmo authors once, on release,
  so this is a per-drag hitch there and a per-move cost in the Avar Editor slider, the Graph Editor key drag and
  the volume-weight drags.
- Publishing (`EvaluateAndPublishResult` minus `Evaluate`) is a flat 2.55-2.8 ms per move in both modes.
  Measured in C++: `UsdGeomImageable::ComputePurpose` over the 326 published prims is 991 us per generation (a
  single top-down `ComputePurposeInfo` pass over the rig does the same in 216-285 us), the display-colour and
  opacity connection reads 335 us, radius, shape, scale and wire-width reads 124 us, the joint-children map 36 us;
  the store then diffs ~350 prims including the 26,276-point array, and the registry copies every prim into a
  combined snapshot and diffs again. In baked mode publishing costs twice the evaluation.

### 5.3 Recommendations for interaction, in order of measured value

1. Memoize the gizmo's frame helpers per `ComputeRigFrames` call (the 320 ms). Saves 310-390 ms per move on
   deep joints, ~0.25 ms on top-level controls. Python only; requirements in 5.1.
2. Run the imaging bridge in baked mode: request Baked where the evaluator is created (do not clobber an
   explicit `RIGEXEC_EVALUATION_MODE`; it is read once into a static), do not pre-call `IsBakeable` (Build calls
   it), surface the silent per-generation fallback as a diagnostic, and guard the Run-bail loop so a failing
   program is not rebuilt every move. Saves 6.6-7.2 ms per move (83% of the evaluator lane). Land the
   refresh-the-constant refinement with it if Default write mode must stay usable (refresh
   `binding.input.constant`, not `B.avarConstants`; a first time sample still rebakes).
3. Cache the publish side's authored reads per epoch and per time code (purpose via one top-down pass, the
   has-connection bit plus unconnected style values, radius, shape, scale, wire width, prim handles, the
   joint-children map), invalidated from the registry's existing `_OnObjectsChanged`; skip the full-asset
   curvenet traversal when the rig has none; publish once instead of copy-and-diff twice for a single session.
   Prototype measured 1.55 ms (baked) and 1.72 ms (dynamic) saved per move with bit-identical output; ~1.9-2.0 ms
   with the single publish. Reusing whole published prims was bounded at 0.4-0.6 ms and dropped.
4. Clear the static-input and skin-topology caches only when the override key set changes, comparing the full
   (prim, computation, attribute) triple over the union of the outgoing and incoming sets, and additionally
   whenever a key names `rigExec:jointIndices`, `jointWeights` or `elementSize` (a test with successive same-key
   weight overrides breaks without this); make `ClearInteractiveOverrides` a no-op when already empty, because the
   registry calls it on every non-dragged rig every move. Saves 600-700 us per move in both modes.
5. Dynamic mode only: give each solver batch only the avar-channel attribute overrides in its input index,
   fail-open (never filter a computation override or a rest-channel override; both change results, measured),
   disabled for a batch whose frame input has no seeded frame, with an env-gated parity mode that evaluates
   filtered and unfiltered. Measured: brow drag 1136 -> 28 us of solver exec, zero divergence over 280
   comparisons on avar drags; hips drag ~0.1 ms. For the guide request the cheaper equivalent is
   `SetSolverGuidesEnabled(false)` on drag begin (~685 us).
6. Authored lane: replace the structure-digest recompute on every notice with a read-set gate (record the
   property paths and aspects the digest read; skip only when every changed path is absent or was read for a
   sample count the notice did not change; treat any resync or any changed field outside default, timeSamples
   and spline as structural) and split `_structureDirty` into digest and rest-refresh flags. Measured 24-33 ms
   per authored edit; zero per viewport-gizmo move.
7. UI-level items are small once 1 lands: Qt's xcb plugin already compresses motion events, so same-thread
   coalescing saves ~0 here; the Python trims (camera resolve, toolbar sync, ctypes buffer) total ~35 us; an
   off-thread evaluation worker is unsafe without splitting the registry's locked evaluate-publish-broadcast
   unit and guaranteeing no stage authoring during the drag, and unnecessary at 7-8 ms dynamic or 1.4 ms baked.

## 6. Landing order and what not to sum

The candidate estimates add up to about 70% of a frame, which cannot all be real: 3.2 removes a third of the
frame and changes every later proportion, 3.1 halves the per-request cost 3.3 is priced against, and the two
forks in 3.4 hide less once the serial part lands. Re-baseline with guides on before crediting anything.

Dynamic playback: (a) the serial removals that nothing else depends on: skin marshalling, the derived tail,
the parity-only matrices, the unread `RecordFinal`, the prologue items, the property-chain handles, about
1.0 ms; (b) the dense propagation of 3.2, 1.4-1.9 ms, after which the profile has a different shape;
(c) decide the guide request, which on the real baseline is then the largest remaining item; (d) the
exec-engine setting at deployment, 0.8-1.1 ms of the shrunk frame, which also frees 19 cores; (e) the dense
provider indices, so later merges are by index; (f) the forks, re-measured on the new baseline: the
provider loop under the snapshot (nesting verified), the extraction split (may fall under the noise floor
after (d)), the per-level fusion (worth ~70 us after (d)), teardown on a worker last. The seed Warm pass stays.

Baked playback: sorted hinted inserts for the joint maps, the property-chain handles, the derived tail, the
extent fold into the skin pass, then the publish fork only if its ~30 us still clears the noise floor.

Interaction: 5.3 in the order given. Items 1, 2 and 3 alone take a brow drag from ~330 ms to ~5 ms of
evaluator-plus-publish per move, with the remaining Python frame cost then dominant.

## 7. What is not parallelizable, and why

- Two exec requests can never overlap: one `ExecUsdSystem` per stage, shared by every tap set, with no locks.
  2.4 ms of the dynamic frame is inside exec calls; what can move is work around them and the engine choice
  inside them.
- Constraint steps form a total order by construction (constraint i depends on i-1, the authored Movers-stack
  `preceding` contract); a level-parallel constraint walk is blocked at Compile and, given the spine's nested
  write sets, worth little if unblocked. The propagation inside a step is the parallelizable part, and it is
  cheaper to make dense than to fork.
- Work under ~10k cycles does not pay a fork on this box: BakedCompose, the baked solver batches, the
  constraint kernels, a parallel extent on 26k points, the twelve property chains.
- Anything that reads through the static input cache must stay on the calling thread or run uncached.
- The rigExec registered exec computations must stay pure over `VdfContext` (no evaluator member, no stage
  read); that is what makes the engine choice in 3.1 safe and the fusion in 3.3 free of new shared state.
- External validity: every microsecond above was measured on a rig whose chain region is serial and whose 19
  other cores are idle during it. On a rig with three or more meshes the existing level-parallel chain walk
  already uses them, and the forks that hide work behind the chain compete for the same cores. A three-mesh
  biped variant and a drag run are the experiments that tell which gains transfer.

## 9. Collapsing the exec pose system and the mover graph into one system

Asked after the study: would one evaluation system schedule better without losing functionality? Four
independent assessments (the boundary map and its documented rationale, the true pose-domain dependency DAG,
a hand-built unified VDF network prototyped at the biped's scale, and the reverse direction into exec) agree.

What a collapse buys is the boundary, not concurrency. The dynamic frame crosses ten hand-offs per frame:
property-chain results pushed into exec as value overrides and into a parallel store, ribbon driver points
boxed onto stub computations, the seed request copied into three maps, 14 one-tap solver requests (1.0-1.5 ms
of Compute for 12-21 us of kernel math) with a full override-vector rebuild each, the engine-less constraint
walk over maps, 352 overrides pushing the finished pose back into exec for the authoritative request, ~1 ms of
map publication, the assembler into the private geometry network, and the optional guide request. Three of the
four "exec dependencies" turn out to be values computed outside exec and transported in (falloff LUTs, ribbon
points, solver-joint frames, property chains), and the public tap API has no external client in the repo. The
baked program is the collapsed system for the rigs it accepts: 0.7 ms against 6.7 ms.

What a whole-graph scheduler would find is small. The compiled biped DAG (326 provider frames, 14 solvers, 65
constraints, 1538 exact propagation writes, 12 property chains, the skin and the extent) is 463 us of math
serial; an ideal schedule on infinite cores reaches 223 us, and 88% of that is the skin and extent, which are
already parallel inside. The authored-order edge between consecutive constraints costs 44 us of critical path.
The nodes average 0.08 us, so no real scheduler can collect them: the concurrency that pays is geometry
against pose, mesh against mesh, and frame against frame, not constraint against constraint.

The prize only one graph delivers is sparse re-evaluation. Simulated over the real graph, a brow-joint drag
needs 1.1 us of the 128 us pose walk and dirties 423 of 26,276 skinned points; a hips drag needs 88% of the
walk and all points. A hand-built network prototyped at the biped's shape (one node per constraint or solver
step owning its write set, about 85 nodes) costs 0.62-0.85 ms per frame, at parity with the baked op list, and
re-evaluates a drag cone in 22-40 us of pose work; geometry still re-runs whole because the nonlocal kernels
refuse element masks, so a realistic drag is 200-300 us. Two measured constraints on the shape: one node per
joint (about 1950 nodes) would spend 740 us per frame on framework alone, and a pass-through table cannot prune
sparsely because VDF routes associated outputs around the dependency mask. VDF's parallel engine loses at this
granularity exactly as exec's does. The mover graph's scratch-copy idiom around each kernel costs ~90 us per
frame that a random-access accessor removes.

The reverse direction, moving the walk and geometry into exec, is blocked before scheduling is reached: exec's
compiler resolves only `..` and property hops, so a per-rig computation cannot enumerate the Movers namespace;
the one reverse accessor follows attribute connections only and returns providers in nondeterministic order;
registrations are per schema type, not per rig; there is no read-write connector, so every geometry revision
would copy the point array; and overrides are the real exec cost (94-307 us each, scaling with request size),
so a single giant request makes drags worse. A fused single exec request saves ~0.1-0.2 ms.

What a collapse must re-implement rather than inherit: everything in the bakeability refusal list (every
example rig except the biped is refused today, for weight objects on movers, blendShape, ribbon,
emitGuidePoints, lattice, smooth, volumeCorrect, surfaceProject, curvenet, the Ribbon and TwistDistribution
solvers, volume weights, plain-Xformable constraint targets and sources); USD value resolution through exec's
attribute compute (connections, composition, time samples) for connected-space providers and time-varying
rests; the standalone deployment in `libs/rigExecStandalone`, which runs the same registered computations over
an Esf-adapted scene database with no stage and uses the tap set as its parity oracle; ordered, all-or-nothing
commits and ordered diagnostics; read phases within a chain; and the independent CPU oracle.

Recommendation. Build the replacement from the baked program outward, as a dependency graph of steps that
declare the dense slots they read and write, executed in topological order with the option of level-parallel
tasks where a level's nodes are large (geometry) and of dependency-scoped re-evaluation for interaction. Keep
exec as an input source for what only it resolves. Extend operator coverage until the refusal list is empty,
with parity against the dynamic path as the acceptance test. Do not expect intra-frame constraint parallelism
to pay; expect boundary removal (dynamic 6.7 ms to under 1 ms), drag cones, multi-mesh overlap and frame
pipelining to.

## 10. Reproduction

Instrumented copies, read-only for the live tree, under the session scratchpad
(`/tmp/claude-1000/-home-burkard-work-usdRig/03b2e911-3650-40f1-9b9d-8fc405bc1eee/scratchpad/`): `dyn/` (49 dynamic
probes plus tap-set probes; `patch.py`, `patch2.py`), `baked/`, `drag/` (`rigExecPose --drag <prim> <attr> <steps>
[--author] [--plain]`, the `rigExecDragPublish` target, `instrumentation.patch`), and one `verify-*` or
`iverify-*` copy per reviewed candidate with its patch and traces. Exec engine A/B: run `build/rigExecPose
examples/biped/Biped_anim.usda --frames 1,2,3,4,5,6,7,8 --profile <out> --mode dynamic` with and without
`VDF_ENABLE_PARALLEL_EVALUATION_ENGINE=0`. Gizmo timing: `PXR_PLUGINPATH_NAME=build/usd/rigExecSchema/resources
PYTHONPATH=<OpenUSD site-packages>:plugin/rigExecUsdview python3`, then `gizmoMath.ComputeRigFrames(stage, prim, t)`;
without the schema plugin the avar reads resolve nothing and the cost reads 40% low.
