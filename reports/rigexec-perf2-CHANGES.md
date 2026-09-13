# rigexec-perf2: fixed-topology geometry, the exec-keeping tier, and the baked-rig mode

One patch against the base tree (`reports/rigexec-perf.patch` applied), joining three tracks:
**G** (geometry), **T** (the tier that keeps exec) and **B** (the baked-rig mode), plus the
integration work the join itself needed, and the fix-ups three adversarial reviews asked for
(section **R**). 20 files, 9.9k patch lines, 14 new ctest entries.

Everything below is measured on this machine (Linux aarch64, 20 cores), min of N paired runs,
base and merged binaries run back to back in the same session. Load averages are quoted with the
tables at the end.

**Acceptance, re-run on the merged tree after the review fix-ups.** Dynamic-mode `rigExecPose
--joints --targets --joints-out` is byte-identical to the base on `Biped.usda` (1,2,3),
`Biped_layered.usda` (1,2,3), `Biped_anim.usda` (1..8) and on the 5- and 9-mesh stages (1..8);
dynamic output is byte-identical to the base on all 43 shipped example stages; baked mode publishes
the same pose as dynamic on every one of them (23 of the 43 decline the bake and say which feature
stopped them); epoch digest 16236336116187278719; output unchanged with
`RIGEXEC_ENABLE_PARALLEL_EVAL=0` and with `PXR_WORK_THREAD_LIMIT=1`; ctest 63 tests with the failure
set still exactly {`testRigExecCurvenet`, `testUsdNoodles`}, both pre-existing on this box.

---

## G1. Per-epoch skin topology, and epoch-constant base points

**What it avoids.** Every frame the skin assembler re-read `rigExec:jointIndices` and
`rigExec:jointWeights` off the stage (2 MB on the biped), copied them into the packet, and
re-validated all 105k elements; the kernel then re-validated the whole layout again. None of that
can answer differently within an epoch. The authored base points were also re-read and compared
element by element against what the source already held, 26k points at a time, only ever to
conclude "unchanged".

The layout is now a `RigExecSkinTopology` held by `shared_ptr` on `RigExecMoverParameters`,
resolved once through a `RigExecSkinTopologyCache` owned by the evaluator (never a static),
compared by identity in `operator==`. Compile sets `_GraphRevision::skinTopologyFixed` only when
none of `jointIndices`, `jointWeights`, `elementSize` is time-varying, carries an authored
connection, or is the target of a property chain; and because an edit that animates a layout
*after* Compile moves no epoch digest, the first two of those three are re-asked **where the cache
is filled** as well (see R2). Any refusal puts the packet back on the per-frame arrays, as before.
The cache is cleared by every change notice, by every interactive-override change and at the
epoch commit, so a weight-paint edit reaches the very next generation. The per-frame kernel
check is O(1): `validated && influenceCount == transformCount && indexCount == pointCount *
elementSize`. Base points are held on `_LiveGraph` with `basePointsPushed`/`basePointsStatic`,
reset by the same three events.

**Measured.** Animated biped, `Assemble body_geo_skin` 0.26 -> 0.005 ms/frame; `GraphEvaluate`
0.79 -> 0.61 ms. In the baked path the same assembler call goes 0.28 -> 0.003 ms/frame.

**Tests.** `testRigExecSkinTopology`: weight-paint edit after the first Evaluate (asserting the
epoch digest does *not* change, so only the notice can invalidate), partial repaint, index swap,
time-sampled weights refusing the cache, interactive override on the layout and its removal,
connected weights refused, base-point value edits, and a time-varying point count failing
atomically through the O(1) guard and recovering.

## G2. Constant full-strength envelope fast path

**What it avoids.** `inputs:defaultWeight` synthesizes a constant envelope packet for every
unweighted mover. Resolving it produced a 26k-element array of 1.0 and then ran a blend that
`RigExecBlendEnvelope`'s `weight >= 1` branch answers by returning the candidate unchanged -- plus
a full copy of the preceding revision to blend against. When the packet is constant, empty of
values and indices, `defaultWeight == 1` exactly, and the range policy is one `ResolveAll` accepts,
all three are skipped. Exact, not approximate: the skipped branch performs no arithmetic.

**Measured.** Animated biped `GraphEvaluate` 0.80 -> 0.42 ms/frame (one 26k-point copy, one
26k-float resolve and one 26k-point blend per frame).

**Tests.** Covered by the byte-identity gates on every stage (the fast path is taken on all of
them) and by `testRigExecMoverGraph`/`testRigExecBakedMode`, which compare against the blended
result.

## G3. In-kernel parallelism, behind one kill switch

**What it avoids.** The LBS kernel and the envelope blend ran one point at a time on one core.
Both are now split by point range under `WorkParallelForN` (grain 512, no dispatch below 4096
points). A point range is an independent sub-layout -- point *i* reads slots `i*elementSize` and
writes index *i* -- and both kernels loop one point at a time, so no split boundary can land inside
a vectorised block: the parallel result is bit-identical, not merely equal to tolerance.

New `libs/rigExec/parallel.{h,cpp}` carries the only environment switch in this patch:
`RIGEXEC_ENABLE_PARALLEL_EVAL` (TfEnvSetting, default true), checked at every rigExec parallel
region -- these kernels, the chain-level tasks of G4, and the two compile-time tasks the base patch
already had. Off, the work runs inline on the calling thread; it is never skipped.

**Measured.** Animated biped `GraphEvaluate` 0.79 -> 0.24 ms/frame. On the whole merged binary the
switch is worth 0.56 ms/frame on the single-mesh biped (7.41 -> 7.97 with it off) and 5.0 ms/frame
on the 9-mesh stage, and it moves compile from 113 to 157 ms as the digest and the exec warm-up go
inline -- which is how you can see it reaches the compile sites too.

**Tests.** `testRigExecParallelKernels` compares the split result bit-for-bit against the unsplit
one and against an independent double-precision scalar kernel, at sizes below, just above and far
above the threshold; `testRigExecParallelKernelsSerial` runs the same binary with the switch off.
End to end: `rigExecPose` output is byte-identical with `RIGEXEC_ENABLE_PARALLEL_EVAL=0` and with
`PXR_WORK_THREAD_LIMIT=1` (the only difference is TfEnvSetting's own override banner on stderr).

## G4. Level-parallel chain walk, gated

**What it avoids.** Independent geometry chains -- a rig's several skinned meshes -- were walked
one after another on one thread. Compile now partitions `_chainOrder` into dependency levels,
greedily and contiguously, so concatenating the levels reproduces `_chainOrder` exactly. A level
runs one task per chain only when Compile classified it safe: at least three chains, no Curvenet
revision, no phased read of any kind, no current-phase weight object, no weight object named by two
chains. Each task writes only into its own `_ChainWork` buffer (diagnostics, moved properties,
weight fields, control frames, its own snapshots, four counters) and the walk merges those buffers
in chain order, so a parallel level publishes exactly what a serial walk publishes -- no mutex
decides any order. `_liveGraphs` nodes are pre-created at Compile, each task gets its own
`UsdGeomXformCache`, and no exec call happens inside the region.

**Measured.** Worth nothing on the one-mesh biped (its level gate is off: one chain). 5-mesh stage
16.27 -> 8.94 ms/frame; 9-mesh stage 22.62 -> 9.24 ms/frame, of which the level parallelism itself
is 14.29 -> 9.24 (the rest is G1-G3). The 9 chains spread over all 20 tids with the switch on and
run on tid 0 with it off.

**Tests.** `testRigExecChainLevels` (+ `...Serial`): a six-mesh in-memory rig above the per-point
threshold, asserting the classification and walk order, bit-identical points over repeated
generations, identical diagnostics and counters, one-thread vs many-thread equality, failing chains
reporting in chain order rather than completion order, per-chain weight objects resolving to the
right chain, and a lattice reading another mesh's points splitting the walk into two levels.

*(G5, a NEON path for the SIMD skin kernel, was deliberately not attempted: on this aarch64 host
the scalar fallback accumulates in double, so a float NEON kernel would break the byte-identity
gate every acceptance check here rests on, for at most ~0.12 ms of an 8 ms frame.)*

## T1. Epoch-constant rest frames

**What it avoids.** `computeRestFrame` was requested for all 326 providers on every frame. A rest
frame is a function of rest channels that do not move with time, so Compile now pulls them once
through a dedicated request into `_epochRestFrames` and Evaluate seeds from that map. The 326 rest
taps Compile used to add to the authoritative request were dead state -- written, read nowhere --
and were deleted with their map.

Because **the epoch digest hashes no attribute value and no time code**, none of this can be
decided once and left alone. The rule, after R1:

* Compile refuses the epoch path outright when any seed provider's rest channel is connected,
  carries time samples (both `ValueMightBeTimeVarying()` and `GetNumTimeSamples() > 0`, which do
  not imply one another), or is written by a property chain -- the epoch then keeps the old
  per-frame rest taps verbatim;
* the same question is re-asked on every notice that did not recompile, and an epoch whose rests
  have *become* animated or connected is recompiled on the spot rather than refreshed;
* a notice that moved only rest VALUES re-pulls the epoch's rests;
* and an interactive override standing on a rest channel pulls the rests per frame, with the
  override applied, through the same request.

**Measured.** `PoseSeed` 8.06 -> 5.20 ms over three static frames; steady frame -0.2 to -0.4 ms.

**Tests.** `testRigExecEpochConstants` (python): rest edit with and without a recompile, a rest
edit on a second provider, and the observability trap that makes this hard to test -- a rest edit
moves a joint's posed frame and its rest frame together, so the tests compare `jointMatricesFinal`
against a reference stage that carried the same edit before it was ever compiled. Removing the
refresh fails them.

## T2. Missed connection guard, and a per-evaluator static input cache

**What it avoids.** `RigExecResolvedInputs::GetAttribute` called `GetConnections` on every read,
building a target index to be told "empty"; it now calls it only when `HasAuthoredConnections()`.
And the same handful of authored inputs -- a constraint's `inputs:enabled`, its offsets, a weight's
`defaultWeight` -- were resolved through USD again on every frame to be handed the same number
back. `RigExecStaticInputCache` holds one authored value per attribute path, admitted only when the
attribute has no authored connections, no possible time variation and no authored time samples at
all (the third test is not implied by the second -- see R3), cleared on every notice and on
`SetInteractiveOverrides`. An entry answers only the type its first read typed it as. Precedence is preserved: a value this generation already resolved
(a property-chain result, an interactive override) is consulted before the cache.

The cache stamps the thread that cleared it and is bypassed from any other thread, so G4's chain
tasks read through the slow path rather than racing on it.

**Measured.** About -0.3 to -0.5 ms per frame on both stages, on top of T1.

**Tests.** `testRigExecEpochConstants` asserts a time-sampled weight and a connected weight are
never held, and that an `inputs:enabled` edit is followed. Mutation-checked by the track: forcing
admission true, or deleting the notice clear, each fails a test.

## T3. The first frame's warm compute, paid at Compile

**What it avoids.** Every Evaluate warms the shared executor before its override-bearing pull, and
the first warm of a session computes the whole pose-seed network from an empty cache -- most of
what makes the first frame cost three times the frames after it. None of it depends on which frame
is asked for first, so `_poseSeedTaps->Warm()` now runs once inside Compile, at the stage's start
time code. It sits immediately before T1's rest pull, which turns that pull into a copy-out of
values the warm already computed (3.2 -> 0.12 ms).

**The time code must be a real frame, never `UsdTimeCode::Default()`.** A Default pull is a
different mode, not an instant: the values it leaves in the shared executor are time-independent
and a later `ChangeTime` does not invalidate them. Warming at Default reproducibly broke
`testRigExecConstraints` (the autoDetect single-chain IK case); numeric 0 and 1 pass.

**Measured.** First frame of a session 18.96 -> 11.04 ms static, 20.13 -> 11.17 ms animated;
`PoseSeed` summed over 8 animated frames 14.71 -> 4.18 ms. Compile pays 6.8-8.7 ms for it
(`Compile.WarmPoseSeed`), which is work moved off frame 1, not new work.

**Tests.** `testRigExecEpochConstants::TestFirstFrameIsNotTheWarmedFrame`; the Default-vs-numeric
distinction is held by `testRigExecConstraints`, confirmed by mutation.

## B1/B2/B4. `RigExecBakedProgram`: the compiled epoch as an exec-free op list

**What it avoids.** The whole per-frame exec round trip: the pose-seed request, the authoritative
snapshot, 14 solver-batch requests and the serial constraint walk that together cost ~7 ms of a
9 ms frame. The program is built at the end of Compile from the compiled epoch: dense provider
slots in namespace DFS order (so one forward pass composes the hierarchy); an input binding table
that mirrors `GetAttribute`'s connection walk and classifies each input once -- epoch constant,
retained `UsdAttributeQuery`, or a per-frame read through the generation's resolved inputs (an
input is varying if anything on its walk reports `ValueMightBeTimeVarying()` **or** carries a time
sample at all, which is not the same test -- see R4); then an
ordered op list -- property chains (calling the evaluator's own `_EvaluatePropertyChains`, not a
second copy), provider frame compose, the aggregate solver kernels in compiled batch order,
the constraint kernels in `_poseSteps` order with propagation pairs precomputed, `PointsToMatrix`,
the shared geometry kernels, derived extent, and publication into the same `RigExecRigPose` maps
with the same diagnostics. Exec's frame<->matrix round trip is reproduced wherever exec performs
it, so a deep chain does not drift in the last bits.

`IsBakeable(&reasons)` records one deduplicated reason per unsupported feature (connected-space
providers, intervening Xforms, weight objects and volume weights, read phases, geometry-domain
constraints, SingleChainIK, ribbons, curvenets, blend shapes, lattices, profile movers, twist
distributions, any mover outside skin and matrix, animated or connected skin layout, and more), and
Compile stays Dynamic when any is present. `SetEvaluationMode(Dynamic | Baked |
BakedWithParityCheck)` makes Baked a *request*: with no program, a dirty epoch, or `cpuParityMode`
set, Evaluate falls back, so the mode can never change an answer. Parity mode runs both paths,
compares base/final joint frames, joint matrices, control frames, provider transforms, moved
properties and solver frames with exact equality, publishes the **dynamic** generation, counts
disagreements in `RigExecRigPose::bakedParityMismatches` and warns. Python: `Rig.evaluation_mode`,
`Rig.is_bakeable()`, `Rig.bakeability_reasons()`, `Pose.baked_parity_mismatches`. Tool:
`rigExecPose --mode dynamic|baked|parity`.

The skin and weighted-matrix kernels were hoisted out of the mover-graph revision node into shared
`RigExecApplySkinKernel` / `RigExecApplyMatrixKernel`, so the two paths cannot drift -- including
over the SIMD choice, which must be the same on both or they disagree in the last bits.

**Measured.** Animated biped steady frame 9.01 -> 0.71 ms (12.7x), static 8.27 -> 0.61 ms, 5-mesh
16.27 -> 1.75 ms, 9-mesh 22.62 -> 2.72 ms. Compile pays 24-27 ms for the bake.

**Tests.** `testRigExecBakedMode` over `Biped.usda`, `Biped_layered.usda`, `Biped_anim.usda` and
`simple_rig_anim.usd` (a matrix-mover rig, so both geometry operations are compared), frames 1..8,
each on two independent evaluators over two independently opened stages; seven published maps
compared key by key in **both** directions, plus diagnostics and solver override rounds, and the
test refuses to pass if the rig fell back. Plus three existing suites re-run under the parity check
as their own ctest entries (`testRigExecConstraintsBakedParity`, `testRigExecArmBakedParity`,
`testRigExecInteractiveBakedParity`), each failing on the warning text.

## B3. Invalidation: what survives a scene edit

**What it avoids.** Dropping the program on every notice, which made an interactive drag and any
scene edit fall back to the dynamic path. The bake pass now records what it read as it reads it,
into four sets -- because a notice asks four different questions about the same property:
`rebuild` (properties whose *value* decided something the program holds), `named` (every property
the bake asked about at all, since a resync invalidates a retained query and can change which
attribute of a connection walk is selected), `prims` (for a resync naming a subtree) and
`xformPrims` (the Xforms whose *composed* transform bakeability judged). `IsInvalidatedBy(notice)`
asks that index; a hit rebuilds the program alone, without recompiling, because Compile has already
settled whether the epoch moved.

Interactive overrides are placed rather than refused: an override on any attribute of a
registered input's connection walk -- not only its head (see R5) -- sets a flag that forces the
long route, `RigExecResolvedInputs::GetAttribute`, the same walk exec's accessor performs, so the
held value is the dynamic answer by construction. An override on
a folded constant or a prim computation returns false and that one generation runs dynamically.

Two things found while doing it, both fixed here: the mode dispatch used to choose its path before
the epoch settled (so the first frame of every session ran dynamically for no reason -- `_SettleEpoch`
is now hoisted ahead of the dispatch), and the rest chain and default-space ladder are resolved
once at Build through the evaluator's per-generation resolved inputs, so a property chain aimed at
one of those attributes could bake a stale value; `IsBakeable` now refuses that, naming the
attribute.

**Measured** (Biped.usda, 40 generations, dynamic vs baked in the same harness): interactive drag
on a control avar 9.3 -> 1.4 ms with 41/41 generations answered by the program and exactly one
build; an unrelated scene edit 7.4 -> 1.7 ms with **zero** rebuilds; a moved rest (a captured
constant, epoch digest unchanged) rebuilds exactly once and then runs at 0.96 ms against 6.6.

**Tests.** `testRigExecBakedMode` grows two harnesses (edit after the bake, edit before the bake)
that assert both halves of the contract in both directions -- `GetBakedProgramBuildCount` must move
when the index is hit and must *not* move when it is missed, and `GetBakedGenerationCount` proves
the program produced the answers. Scenarios: keyed IK/FK weight + `inputs:enabled` + moved rest
(rebuild expected); a moved foot roll reaching the rig only through float-math movers (no rebuild);
a keyed avar's value moved (no rebuild); a new prim outside the rig (no rebuild). Overrides: a
control avar, a constraint envelope, and three shapes of unplaceable override each asserted to fall
back and agree. Fallback: two non-bakeable stages must report a reason, build no program and
publish the dynamic generation unchanged. Mutation-tested by the track: `IsInvalidatedBy` hardwired
false -> 5005 failures; each unplaceable-override branch disabled -> 584/583/1 failures; a 1e-9
perturbation of one provider frame -> caught by all four suites.

---

## Merge integration (this patch, beyond the three tracks)

### M1. One skin kernel, with G1's topology and G3's parallelism inside it

Track B hoisted the skin kernel out of the revision node into the shared
`RigExecApplySkinKernel`; track G rewrote that same kernel body in place for the epoch topology and
the parallel point split. Resolved in B's favour structurally -- the hoisted function is the single
definition both paths call -- with G's body inside it. This is the only textual conflict in the
patch that could have been resolved by dropping work from either side, and neither was dropped.

**Measured.** G3's split reaches the baked path through this: the baked geometry chain on the
9-mesh stage runs its per-mesh kernels in parallel.

### M2. The baked program uses the evaluator's skin topology cache

`_GeomRevision` now carries `skinTopologyFixed` from the compiled `_GraphRevision`, and the baked
assembler is handed `&E._skinTopologies` when it is set -- the *same* cache the dynamic path uses,
so the two paths cannot even hold different arrays. Without this the program re-read and
re-validated 2 MB of layout every frame while the dynamic path no longer did.

A note for reviewers, since track B flagged this interaction: the layout properties stay in the
capture index's `named` set rather than moving to `rebuild`. They are not folded into the program;
they are held in a cache that every notice clears, so a weight-paint edit reaches the next baked
generation through the re-read, and the packet's identity compare re-runs the kernel. The comment
at the registration site says so.

**Measured.** Baked animated biped: `Assemble body_geo_skin` 0.279 -> 0.003 ms/frame, geometry
chain 0.931 -> 0.243 ms, whole frame **1.474 -> 0.728 ms**.

### M3. G2's envelope fast path in the baked geometry loop

The baked loop applied the envelope with its own `ResolveAll` + blend and copied the preceding
revision to blend against. It now takes the same constant-full-strength fast path
`_RunScratchKernel` takes, by the same predicate, so both paths skip the same work and cannot
disagree about when it is safe to.

The predicate itself is one `inline` function in `moverGraph.h`
(`RigExecEnvelopeIsFullStrength`) that both loops call, so there is nothing for them to disagree
about (see R7).

**Measured.** Included in the 1.474 -> 0.728 ms above.

### M4. The rest refresh belongs to the settled epoch

T's post-notice rest re-pull lived at the head of `Evaluate`, which track B turned into
`_SettleEpoch` plus `_EvaluateDynamic`. It is placed in `_SettleEpoch`, which both paths call
before either decides anything, so the baked path gets it too and its failure is reported the same
way. The guard is unchanged: only the first generation after a notice that did not recompile.

R1 extends this: the same place now also re-asks whether the epoch may hold constant rests at all,
and the per-frame path honours a drag standing on a rest channel.

### M5. Documentation of the two environment settings

`RIGEXEC_ENABLE_PARALLEL_EVAL` (the kill switch) is documented in `README.md` next to
`RIGEXEC_EVALUATION_MODE` (which selects the initial evaluation mode of new evaluators, and is how
the existing suites are re-run under the parity check; it gates no code path). `parallel.h`'s
comment now also names the chain-level tasks G4 added to the switch.

---

## R. Review fix-ups

Three adversarial reviews attacked the merged tree -- dynamic-path semantics and thread safety,
baked-mode fidelity and invalidation, and test coverage by mutation. They raised eleven
blocker/must-fix items. Every one was reproduced first against the merged binary with the base
binary as the control, and every one turned out to be real; nothing was backed out, because in each
case the change could be made safe rather than abandoned. Each fix carries a test that fails
without it -- verified by building the mutant and running the test, twelve times over (the table at
the end of this section).

Four of the eleven items share one root cause, and it is worth naming once: **USD reports
`ValueMightBeTimeVarying() == false` for an attribute whose strongest opinion is exactly one time
sample of a non-composable type** (`UsdStage::_ValueMightBeTimeVaryingFromResolveInfo`), while
`Get(UsdTimeCode::Default())` never consults time samples at all. So a single-keyed attribute has
two legitimate answers, and any cache or capture that treats "not time-varying" as "one answer
forever" serves the wrong one. An animator's first pose key is exactly that shape. Every admission
test in this patch now asks both questions.

### R1. The epoch's rest frames are re-decided, and follow a drag *(blocker; T1, M4)*

T1 moved `computeRestFrame` out of the per-frame pose-seed request into a request pulled once at
Compile. Two holes, both silent, both with the rest half of the rig frozen while the pose half
moved -- which makes the published `jointMatricesFinal` (the rest->pose map) a mixture of two
different rigs:

* **The decision was taken once.** `restMightVary` ran only inside Compile, and the epoch digest
  hashes no rest channel, so authoring the first time sample on `rest:tx`, connecting it, or
  unmuting a sublayer that animates it did not recompile -- and `_RefreshEpochRestFrames` re-pulled
  the same frozen values. Demonstrated four ways on `08_AimEyes.usda`: the base's joint matrix at
  1048 was `(1.158893602, 0, -3.200921998)`, the merged tree's `(5.76822128, 0, 0.6401844)`.
* **The pull carried no overrides.** The base evaluated the same taps inside
  `_poseSeedTaps->Evaluate(time, baseOverrides)`, so an interactive override on a rest channel
  reached them. `BeginPreview("/Biped/Rig/Joints/hips_bind.rest:space")` is an accepted public
  call; driving it moved `jointFramesBase` and left `jointMatricesFinal` at identity, so the drag
  and the commit of that same drag disagreed.

Now: `_ProviderRestMightVary` is one function, used by Compile and by the notice path, and it
refuses a rest channel that is connected, that reports `ValueMightBeTimeVarying()`, that has any
authored time sample, **or that a property chain writes** (the guard the skin layout already had
and this did not). `_SettleEpoch` re-asks it after any notice that did not recompile and rebuilds
the epoch on the spot when the answer changed -- recompiling is the only place the rest taps can go
back into the per-frame request. And when an interactive override stands on a rest channel,
`_EvaluateDynamic` pulls the rests through `_restTaps->Evaluate(time, baseOverrides)` instead of
reading the epoch map, which is exactly what the per-frame taps used to do.

That override test is precise rather than conservative, and it can afford to be: `computeRestFrame`
reads `rest:space` and the six rest avars of the provider and of its RigExec ancestors and nothing
else (`computations.cpp`, `RIGEXEC_REGISTER_XFORMABLE`), and this epoch has no rest channel with an
authored connection, because one would have refused the epoch path outright. A computation override
names something the check cannot inspect, so it counts as a rest override.

**Cost.** Nothing on the steady frame (the check is one lambda over an empty override list). The
notice path -- author an edit, evaluate, forty times over on `Biped.usda` -- is 32.09 ms/generation
with the re-decision and 32.30 ms with it compiled out: the 326-provider scan disappears into the
exec invalidation the notice already caused.

### R2. The skin layout decision is re-taken where the cache is filled *(must-fix; G1)*

Same shape as R1's first hole. `revision.skinTopologyFixed` was computed only inside Compile, and
none of `rigExec:jointIndices`, `jointWeights`, `elementSize` appears in the epoch digest -- so
animating a layout after Compile did not recompile, and `_skinTopologies.Clear()` on every notice
made the cache re-read the arrays once, at whatever frame came first, and freeze them again.
`Biped_anim.usda` with two samples authored on `jointWeights` after Compile: base frame 4 point 0 =
`(1.448139, 87.948181, -0.48982)`, merged `(1.416076, 89.260727, -0.621103)` -- the frame-3 layout,
silently reused.

`RigExecSkinTopologyCache::Resolve` now takes a build callback that may **refuse**, and the skin
assembler re-tests the three attributes for time variation and authored connections inside it. A
refusal is remembered like a layout, so the question costs one answer per notice rather than one
per frame, and a refused mover falls through to the per-frame arrays -- which is what Compile would
have done had the samples been there. The property-chain half of the test stays at Compile, where
the chain set is known.

### R3. The static input cache is keyed to one answer *(must-fix; T2)*

`RigExecStaticInputCache` entries were keyed by `SdfPath` alone and ignored the `time` argument,
while admission only established that the value is the same at every *numeric* time. With one time
sample authored on `AimL.inputs:enabled`, an evaluator that read Default first and then frame 1048
returned frame 1048 with the constraint still enabled: the pose at a frame depended on which time
codes the same evaluator had evaluated before it.

Admission now also requires `GetNumTimeSamples() == 0`. The related typed-entry edge the reviewer
flagged is fixed in the same place: an entry records the type its first read typed it as, and a
later read of a different type goes to the stage instead of being served that read's "no value".

### R4. A single time sample is not an epoch constant *(blocker; B1)*

`_ClassifyInput` decided epoch-constancy with `ValueMightBeTimeVarying()` alone and `_BindInput`
captured at `UsdTimeCode::Default()`, so every input the bake binds -- all eleven control avars,
`inputs:enabled`, `inputs:defaultWeight`, the affect masks, the offsets, the aim vectors and the
TwoBoneIk / SplineIk / BlendPointFrames parameters -- was captured as its schema fallback when it
carried exactly one key. `Biped.usda` plus one key on `hips_ctl.avars:tx`: dynamic
`(25.0000 96.3437 -4.1744)`, baked `(0.0000 96.3437 -4.1744)`, 1080 differing stdout lines, and
`IsBakeable` did not refuse, `Run` did not return false, and no diagnostic was emitted. The file
already knew the hazard and stated it correctly 450 lines further down, in `_AnimatedOrConnected`.

`_ClassifyInput` now uses that same predicate. Both halves of the OR are needed and neither implies
the other: a Ts-spline valued attribute reports no time samples in this USD build while varying.
Dynamic-vs-baked on the one-key stage is now 0 differing lines, as it is with a single key on all
65 of the biped's constraints.

### R5. An override is placed anywhere on an input's connection walk *(must-fix; B3)*

`_Impl::Register` recorded overridable inputs against `input->head.GetPath()` only, although
`_ClassifyInput` had walked a whole authored-connection chain and may have captured the value
several hops upstream. An override standing on such an upstream attribute found no binding, fell
through the `resolvedRoutedPrims` escape (which is keyed by PRIM, and holds every geometry-mover and
property-chain prim), was reported placeable, and was then ignored: `_Read` kept returning the epoch
constant. The shared-envelope idiom -- 65 constraints' `inputs:defaultWeight` connected to one
upstream attribute, and the drag that switches them all off -- moved 131 joints on the dynamic path
and none on the baked one, with `GetBakedGenerationCount() == 1` proving the program ran.

`Register` now inserts every path on the recorded walk into `overridableInputs` against the same
input index. A hit anywhere on the walk sets the flag, and `_Read` then re-resolves through
`RigExecResolvedInputs::GetAttribute` from the head -- the same walk exec's accessor performs,
consulting the generation's resolved values at every step -- so the answer is the dynamic one by
construction. The reviewer's fallback suggestion (refuse an override seen on a walk but not
registered as a head) is not implemented because with walk registration there is no such override
left to refuse; `resolvedRoutedPrims` stays for properties no input walks, all of which the
assemblers do read through `_resolvedInputs`.

### R6. The parity comparator has a seam and a positive test *(must-fix; B4/B5, M4)*

`_CompareBakedPose` -- the whole of `BakedWithParityCheck`, and the only thing in the suite that
catches several classes of bake defect -- had no test that made it find anything. Every assertion
about it, and all three `*BakedParity` ctest entries, say it found *nothing*, which is what a
comparator that does nothing also says. Disabling it with `if (true) return;` passed the whole
suite and every gate.

It is now the free function `RigExecComparePoses(reference, baked, out)`, declared in
`bakedProgram.h`; the evaluator calls it. `testRigExecBakedMode` gains
`TestTheParityComparatorFindsWhatIsThere`, which builds two poses differing in each of the eight
compared domains in turn plus the two asymmetric cases (a key only the reference has, a key only
the baked pose has), and asserts the exact mismatch count and the `"baked parity: "` diagnostic for
each, then all eight at once for a count of 8; and `TestTheParityComparatorIsExact`, which
perturbs one joint matrix of a real `Biped.usda` generation by 1e-9 and demands exactly one
mismatch -- exact equality, not a tolerance. `testRigExecEpochRests` is also registered a second
time under the parity check, since it is the suite that moves rests on purpose, which is what M4 is
about.

### R7. One definition of the full-strength envelope predicate *(must-fix; M3)*

M3's claim was that the baked geometry loop and `_RunScratchKernel` "cannot disagree about when it
is safe to skip" the envelope blend, but they were two hand-copied literal expressions, and no
bakeable stage in `examples/` or `tests/` had a skin mover with a *partial* constant envelope -- so
the predicate was only ever exercised where both answers were the same. It is now one `inline`
`RigExecEnvelopeIsFullStrength` in `moverGraph.h` that both loops call.

Two fixtures make the predicate observable as well as shared: `testRigExecSkinTopology` gains a
half-strength and a zero-strength constant envelope on a rig whose deformation is checkable by hand
(dropping `defaultWeight == 1.0f` from the shared predicate doubles the deformation and fails it),
and `testRigExecBakedMode` runs `Biped_anim.usda` with `inputs:defaultWeight = 0.5` on the skin
mover through both paths.

### R8. The two thread-safety guards are asserted, not hoped for *(must-fix; T2/G4, G4)*

Both guards that make the level-parallel chain walk memory-safe fail *probabilistically* when they
are broken: removing the static cache's owning-thread bypass crashed `testRigExecChainLevels` in 3
runs out of 40, and deleting the `_liveGraphs` pre-creation crashed it in 15 out of 60. ctest runs
a test once, so a real regression landed green three runs in four.

* `RigExecStaticInputCache` gains `GetSize`, `GetHitCount`, `GetBypassCount` and
  `GetRefusalCount`, and the new `testRigExecStaticInputCache` fills the cache on the owning thread,
  reads the same attribute from a `std::thread`, and asserts `handled == false`, that nothing was
  inserted or counted, that `GetBypassCount()` moved, and that the long way round still gives the
  authored value. It also covers admission (connected, time-varying, single-sampled) and the typed
  entry from R3.
* `RigExecRigEvaluator::GetLiveGraphCount()` exposes the node count, and
  `testRigExecChainLevels::TestTheParallelWalkCreatesNoGraphNode` asserts it does not move across
  eight parallel generations -- the invariant the pre-creation exists for, failing on the
  classification rather than on a crash that may not come.
* `TestRepeatedWalksAreIdentical` now runs 60 generations rather than 6, and a third ctest entry,
  `testRigExecChainLevelsRepeat`, runs the whole file 20 times over (0.61 s), so a probabilistic
  failure has twenty chances per CI run instead of one.

### What the fix-ups cost

Nothing measurable. Every re-decision is per notice or per epoch, never per frame; the per-frame
additions are one `GetNumTimeSamples()` at bake time, one counter increment per cache hit, and one
scan of an almost-always-empty override vector. The paired numbers below were re-measured after the
fix-ups and are within noise of the pre-review ones.

### Mutants, and what killed them

Each fix was reverted in its own build of the tree and the suite re-run. Twelve for eleven items,
because three of the items have more than one moving part.

| mutant | killed by |
|---|---|
| `_ClassifyInput` back to `ValueMightBeTimeVarying()` alone | `testRigExecBakedMode` (one key on a control avar; on every constraint enable) |
| `Register` back to head-only override placement | `testRigExecBakedMode` (an override on a shared envelope) |
| `RigExecComparePoses` returns immediately | `testRigExecBakedMode` (the comparator cases) |
| static cache admission without `GetNumTimeSamples() == 0` | `testRigExecStaticInputCache` |
| static cache entry answers any type | `testRigExecStaticInputCache` |
| static cache worker-thread bypass removed | `testRigExecStaticInputCache` |
| skin layout re-test removed from the cache fill | `testRigExecSkinTopology` (weights sampled after Compile) |
| `defaultWeight == 1.0f` dropped from the shared envelope predicate | `testRigExecSkinTopology` (half-strength envelope) |
| rest re-decision disabled in `_SettleEpoch` | `testRigExecEpochRests` (a rest sampled after Compile) |
| override-aware rest pull disabled | `testRigExecEpochRests` (rest:tx and rest:space overrides) |
| property-chain guard dropped from `_ProviderRestMightVary` | `testRigExecEpochRests` (a chain on a rest) |
| `_liveGraphs` pre-creation deleted | `testRigExecChainLevels` (the node-count invariant) |

---

## Results

Base = `copies/merged` (the tree `rigexec-perf.patch` produces). Every "after" number was measured
in the same session as its base, runs interleaved, **after the review fix-ups**. Load average
1.0-1.2 throughout on a 20-core box; times in ms. Steady frame = mean of the frames after the
first.

### The biped, dynamic and baked

| stage | mode | compile | frame 1 | steady frame (mean, min-max) | first-frame premium |
|---|---|---|---|---|---|
| `Biped.usda` (min of 7) | base | 108.1 | 19.93 | 8.66 (7.99-9.32) | +11.27 |
| | **dynamic** | 118.0 | **11.45** | **7.14** (6.74-7.54) | +4.31 |
| | **baked** | 136.8 | **2.28** | **0.63** (0.55-0.72) | +1.65 |
| | base again (control) | 102.3 | 19.25 | 8.24 (7.58-8.90) | +11.01 |
| `Biped_anim.usda` (min of 5) | base | 101.6 | 20.05 | 9.06 (8.66-10.33) | +10.99 |
| | **dynamic** | 112.3 | **11.30** | **7.14** (6.81-7.91) | +4.16 |
| | **baked** | 139.1 | **2.27** | **0.71** (0.62-0.87) | +1.56 |
| | base again (control) | 101.5 | 19.60 | 9.08 (8.73-10.28) | +10.53 |

The base binary was run again after each set as a control, so drift in the box shows up in the
table rather than in the result. Earlier pairings, before the fix-ups, gave the same figures
(9.01 / 7.15 / 0.71 animated) -- the fix-ups cost nothing measurable.

Interactive/notice path, which is where R1's re-decision lives: forty generations of
`Biped.usda`, each preceded by an authored edit, 32.09 ms per generation with the re-decision and
32.30 ms with it compiled out.

### Where the animated frame goes (per frame, frames 3-8, min of 5)

| scope | base | dynamic | baked |
|---|---|---|---|
| whole frame | 9.06 | 7.14 | 0.71 |
| geometry chain | 1.424 | 0.440 | 0.275 |
| derived (extent) | 0.123 | 0.137 | 0.078 |
| `Assemble body_geo_skin` | 0.263 | 0.005 | 0.003 |
| `GraphEvaluate` | 0.792 | 0.247 | n/a (no VdfNetwork) |
| `PoseSeed` | 0.486 | 0.372 | n/a (no exec) |
| authoritative snapshot | 0.899 | 0.814 | n/a (no exec) |
| property chains | 0.323 | 0.141 | 0.116 |

### Multi-mesh stages (5 and 9 skinned meshes, animated, min of 5)

| stage | mode | compile | frame 1 | steady frame |
|---|---|---|---|---|
| mesh5 | base | 106.3 | 33.49 | 15.67 |
| | **dynamic** | 113.2 | 17.38 | **8.98** |
| | **baked** | 143.6 | 8.72 | **1.59** |
| mesh9 | base | 106.5 | 46.35 | 21.54 |
| | **dynamic** | 112.5 | 22.84 | **9.43** |
| | **baked** | 148.6 | 13.47 | **2.73** |

(Pre-review pairings of the same stages: mesh5 16.27 -> 8.94 -> 1.75, mesh9 22.62 -> 9.24 -> 2.72,
and mesh9 dynamic with `RIGEXEC_ENABLE_PARALLEL_EVAL=0` 14.29.)

The two multi-mesh stages are scratchpad fixtures (`$S/g4stages/mesh5.usda`, `mesh9.usda`),
deliberately not shipped in `examples/`; the in-repo fixture for the same code path is the
six-mesh rig `testRigExecChainLevels` builds in memory.

### Against the plan's targets (section 5)

| | plan | measured |
|---|---|---|
| animated frame after G+T | <= 7.6 ms | **7.14 ms** (7.15 / 7.41 in earlier pairings) |
| geometry after G+T | <= 0.6 ms | **0.55 ms** (chain 0.43 + derived 0.12) |
| first frame of a session | <= 12 ms | **11.30 / 11.45 ms** |
| compile after G+T | <= 110 ms | 112.3 animated / 118.0 static -- **2-7% over** |
| baked frame | <= 1.5 ms | **0.71 ms** animated, 0.63 ms static |
| baked geometry | <= 0.3 ms | **0.24 ms** |
| compile including the bake | <= 120 ms | 136.8-139.1 -- **14-16% over**, the bake itself 25-27 ms |

The two compile numbers are the only targets missed. The dynamic overshoot is T3's warm
(6.8-8.7 ms) plus T1's separate rest request (1.7-2.1 ms), and the first of those is work *moved
off* frame 1, which drops by 8-9 ms -- a session that draws one frame is 6 ms ahead, and one that
draws two is ahead on both counts. The baked overshoot is the bake pass itself, which is only paid
when a caller asks for the mode. If the ceilings have to be met literally, the cheapest items to
drop are T1's separate request `Prepare` (at the price of putting the rest taps back in the
per-frame request) and, for the baked ceiling, building the program lazily on the first baked
Evaluate rather than at the end of Compile.

## Tests added

| ctest entry | what it holds |
|---|---|
| `testRigExecSkinTopology` | what may invalidate the epoch skin layout, what Compile must refuse to cache, and what the cache fill must refuse after Compile; a partial constant envelope |
| `testRigExecParallelKernels` | split vs unsplit and 1-thread vs N-thread bit equality of the per-point kernels |
| `testRigExecParallelKernelsSerial` | the same binary with the kill switch off |
| `testRigExecChainLevels` | level classification, walk order, diagnostics order, bit equality of a parallel level over 60 generations, and the `_liveGraphs` node-count invariant |
| `testRigExecChainLevelsSerial` | the same, down the serial branch |
| `testRigExecChainLevelsRepeat` | the same file 20 times over, so a probabilistic thread-safety regression lands red |
| `testRigExecEpochConstants` | rest edits with and without a recompile; what the static input cache may and may not hold |
| `testRigExecEpochRests` | when the epoch may hold the rests once: animated, connected and chain-written rest channels, before and after Compile, and a drag standing on one |
| `testRigExecEpochRestsBakedParity` | the same suite under the exact-equality parity check |
| `testRigExecStaticInputCache` | the cache's three rules -- admission (including the single-sample case), typing, and the worker-thread bypass -- asserted directly |
| `testRigExecBakedMode` | baked == dynamic on four stages; the invalidation contract in both directions; overrides, including one standing on an upstream connection source; single-keyed avars and constraint enables; a half-strength skin envelope; fallback; and the parity comparator in the positive direction |
| `testRigExecConstraintsBakedParity` | 31 baked generations of the constraint suite under exact-equality parity |
| `testRigExecInteractiveBakedParity` | 20 baked generations of the interactive suite, where override placement lives |
| `testRigExecArmBakedParity` | the arm suite under parity (thin: it sets `cpuParityMode`, which the baked path declines by design) |

## Environment settings

Two, both documented in `README.md`:

* `RIGEXEC_ENABLE_PARALLEL_EVAL` (TfEnvSetting, default true) -- the documented kill switch. Off,
  every rigExec parallel region runs inline on the calling thread; nothing is skipped and no result
  changes. Verified end to end: byte-identical output with it off and with `PXR_WORK_THREAD_LIMIT=1`.
* `RIGEXEC_EVALUATION_MODE` (`dynamic`|`baked`|`parity`) -- selects the *initial* value of the
  already-public evaluation mode for evaluators in the process. It gates no code path; it exists so
  the existing suites can be re-run under the parity check (`ctest -R BakedParity`) without
  knowing the mode exists.

---

## Should-fix pass

A fourth pass over the merged tree, closing the eight SHOULD-FIX items the three adversarial
reviews left open after their blocker/must-fix items were fixed. Every item that changes behaviour
carries a test that fails without it (the mutant table at the end); two are comment-only and say so.

### S1. Baked requested on a dirty epoch now builds *(rigEvaluator.cpp, rigEvaluator.h)*

`SetEvaluationMode` built the program only `if (_compiled && !_structureDirty)`, and
`_structureDirty` is raised by **every** notice -- so "edit the scene, then turn Baked on", which is
what a UI does every time the artist has touched anything, left `_bakedProgram` null for the rest of
the epoch. `IsBakeable()` kept saying yes, `Evaluate` kept running dynamically, and nothing said why.

The mode is now a request that survives a dirty epoch: `SetEvaluationMode` still builds immediately
when the epoch is settled, and `Evaluate` builds lazily after `_SettleEpoch` when the mode is not
Dynamic, the rig is compiled, there is no program, and the epoch has not already been refused.
That last clause is a new per-epoch flag, `_bakeRefused`, reset by `Compile`, by
`SetEvaluationMode`, and by a notice that hits the program's capture index: bakeability is a
property of the compiled epoch, so a rig the program cannot express would otherwise pay a
bakeability pass per frame to be told the same thing. `GetBakedProgramBuildAttemptCount()` is new
and observable, because the build *count* never moves for such a rig and so cannot say whether it
was asked once or once a frame.

**Tests.** `testRigExecBakedMode::TestBakedModeRequestedOnADirtyEpoch` -- compile, evaluate frame 1
dynamically, `DefinePrim("/Scratch", "Scope")`, `SetEvaluationMode(Baked)`, then frames 1..5
compared against a dynamic-only evaluator that received the same edit, asserting
`GetBakedProgramBuildCount() > 0` and `GetBakedGenerationCount() > 0` (without which the comparison
would be the dynamic path against itself). And `TestANonBakeableRigFallsBack` now asserts one build
attempt per epoch, before and after four frames.

### S2. Published-state parity across a rebuild *(bakedProgram.{h,cpp}, rigEvaluator.{h,cpp})*

The reviewer's case: a value edit that hits the capture index rebuilds the program while the
dynamic path's `_liveGraphs` stand, so the first baked generation republished
`moverGraphRevisionsCreated` / `moverGraphSchedulesBuilt` and the `mover graph: ...` diagnostic as
if the graphs were new, while the dynamic path published `0 created, 0 schedule(s) built`.

Reproduced, and it turned out to be the visible half of something larger: the program's geometry
revisions **are** the dynamic path's `_liveGraphs` with the VdfNetwork baked away -- each holds the
packet it last ran with and the points it produced -- and a rebuilt program threw all of that away
and re-ran every per-point kernel, on a rebuild inside an epoch *and* on a recompile. The dynamic
path does neither: it keeps its nodes, reconnects whichever survive, and adds only what is new.

`RigExecBakedProgram::AdoptGeometryStateFrom(previous)` now hands that state over, matched exactly
the way the dynamic walk matches its VdfNetwork nodes: by chain target, then by (mover, operation)
identity, with a chain's schedule counted as rebuilt only when the identity **sequence** changed.
`ranOnce` is replaced by a per-revision `created` flag and a per-chain `scheduleDirty` flag, so the
accounting is per node rather than per program. Two details that are not bookkeeping but
correctness:

* the cached RESULT is adopted only up to the first position at which the two identity sequences
  diverge. A revision spliced into or out of a chain changes the point stream every revision after
  it reads -- which is why the dynamic path's VdfNetwork re-executes them -- so from that position
  on the node survives and its result does not. Without this the program published stale points
  after a structural splice (caught by the parity check on `testRigExecInteractive`);
* a time-varying point count now drops **that chain's** state and reports its nodes as created,
  instead of handing the whole generation back to the dynamic path. That is what the dynamic walk
  does (`live.reset()` replaces one target's graph and leaves the others standing), and the old
  behaviour degraded every chain of a rig for one mesh whose vertex count is keyed.

`Compile` retires the outgoing program into a local instead of dropping it, so a Compile that fails
still leaves the rig dynamic (the local goes out of scope at the early return) while a Compile that
succeeds hands the geometry state to the program built at its tail. `SetEvaluationMode` hands it
over too, for Baked <-> parity, where the dispatch changed and nothing about the rig did.

`RigExecComparePoses` is widened from the eight map domains to thirteen: the three mover-graph work
counters, `solverOverrideRounds`, and the diagnostics compared **in order** (the order is the walk
order; the same lines in a different order describe a different walk). Each is its own mismatch
domain with its own text, and the diagnostics are read before the comparator appends anything,
because `out` is allowed to be the reference pose -- which is how the evaluator calls it.

**One domain is deliberately left out, and this is a deviation from the item as written:**
`RigExecRigPose::solverEvaluations`. It counts the solver computations the dependency schedule
actually *requested*, and the dynamic path's per-batch exec cache answers a repeated time code with
unchanged inputs for free (`batch.dirty || batch.time != time || !sameInputs`), while the program
holds no such cache and re-solves. Evaluating the same frame twice therefore gives 0 and 4 -- both
true of the path that reported them, neither an answer to a question the other was asked. Making
them agree would mean either the program inventing a solver cache or the dynamic path giving one
up; it is not a parity defect, and comparing it would have made
`testRigExecConstraintsBakedParity` fail on every rig that is evaluated twice at one time code.
`bakedProgram.h` says so at the declaration, and
`TestTheParityComparatorFindsWhatIsThere` asserts the omission (a differing `solverEvaluations`
produces zero mismatches) so it reads as a decision rather than an oversight.

**Tests.** `testRigExecBakedMode::TestAnInEpochRebuildPublishesTheSameCounters`: `Biped.usda` in
Baked mode, frames 1-2, then `rest:tx = 3.0` authored on `/Biped/Rig/Joints/hips_bind` (a captured
constant, so the program rebuilds; a value, so the epoch digest does not move -- both asserted),
then frames 3-4, with the counters and the diagnostics compared line by line against a dynamic-only
evaluator carrying the same edit and a third evaluator in `BakedWithParityCheck` asserting 0
mismatches over the same sequence. `TestTheParityComparatorFindsWhatIsThere` gains the five new
domains one at a time, the two diagnostics cases (a reordering and a line only one side has), and
an all-at-once case of 13.

### S3. The example sweep, as a ctest *(tests/testRigExecBakedMode.cpp)*

`testRigExecBakedMode::TestEveryExampleStage` globs `examples/*.usd*` and `examples/biped/*.usda`,
skips what does not open or holds no `RigExecRoot`, and for each remaining stage runs frames 1..3
on two independent evaluators over two independently opened stages, comparing all eight published
maps -- `jointFramesBase`, `jointFramesFinal`, `jointMatricesFinal`, `controlFrames`,
`providerXforms`, `providerBaseXforms`, `movedProperties`, `solverFrames` -- key by key in **both**
directions. A stage that bakes must have answered all three generations from the program; a stage
that declines must name at least one reason and must have built no program. One line per stage:

```
  Biped_anim.usda                    bakes
  06_LatticeBulge.usda               declines: mover operation not baked (lattice): ...
  simple_rig_flattened.usd           does not compile
example sweep: 35 rig stage(s), 10 bake, 23 decline
```

35 stages carry a rig, 10 bake and 23 decline (two more hold a rig that does not compile in the
base tree either -- a broken flattened example and a layer fragment meant to be sublayered -- and
are reported and skipped). The whole `testRigExecBakedMode` binary, sweep included, runs in 12.6 s.

### S4. ctest timeouts *(CMakeLists.txt)*

`TIMEOUT 120` on every entry, set in the loop that already walks the directory's `TESTS` property
(so a test added later is covered by having been added). ctest's default is 1500 s, which is long
enough for one hung python test -- a usdview harness waiting on a window that will never appear --
to hold a queue for 25 minutes and then report the failure it would have reported at once. All 63
entries carry it; the slowest real entry is 12.6 s.

### S5. What `RIGEXEC_EVALUATION_MODE` is read *(README.md, rigEvaluator.cpp)*

Both the README paragraph and the comment above `_DefaultEvaluationMode` now say that the variable
is read **once per process**, at the construction of the first evaluator (the function-local static
is initialised on its first call and never re-reads the environment), and cannot be changed
afterwards except through `SetEvaluationMode`, which moves one evaluator.

### S6. The static input cache is invalidated symmetrically *(rigEvaluator.cpp, moverGraph.h, tests/testRigExecInteractive.cpp)*

`SetInteractiveOverrides` cleared `_staticInputs` and `ClearInteractiveOverrides` did not, while
`moverGraph.h` said the owner clears it "whenever interactive overrides change". Both call sites
clear it now, and the comment is precise about what the clear is: **defence in depth, not a
correctness requirement**. Every attribute override is written into the generation's resolved
inputs before anything reads one, and `RigExecResolvedInputs::GetAttribute` consults the cache only
where the resolved map has no entry -- so an overridden attribute is never answered from the cache
in the first place. It is kept because a drag is cheap to re-fill from and an invalidation that
runs on the way in and not on the way out is the shape of bug that is only ever found the hard way.

**Test.** `testRigExecInteractive::TestInteractiveOverrides` step 7: a drag on
`inputs:defaultWeight` of a matrix mover -- authored, unconnected, no time samples, so exactly what
the cache admits, and filled with the authored 1 by the six generations before it. The override
takes the envelope to 0 (the point stops moving), twice so a cache that could fill during the drag
would have had its chance, and the release puts the authored value back, twice. Honest note: this
is a regression guard, not a mutation-killing test -- deleting the `ClearInteractiveOverrides` clear
leaves it passing, which is the same fact as "redundant for correctness".

### S7. The phased-read clause of `_IsChainLevelParallelSafe` *(comment only)*

Rewritten to say what it is: defence in depth against a future edge type, not a hazard the current
dependency graph produces. `addEdge` already records a phased read as an edge, so a phased reader
and the chain that produces what it reads land in different levels, and a phase read *within* a
chain is served from that task's own snapshots -- no level the partition builds today holds a
phased read across its own chains. It stays because the cost is one level's parallelism on a rig
that has any phased read at all, and the alternative is a new edge kind silently reading the
chain-snapshot store mid-level. No code change.

### S8. An unrelated edit no longer re-runs the skin kernel *(moverGraph.{h,cpp}, types.h)*

Every notice clears `RigExecSkinTopologyCache`, including the overwhelming majority that touched no
layout. The refill handed back a **new** `shared_ptr`, packets compare layouts by identity, so the
mover's packet compared unequal and the 26k-point kernel re-executed once per notice although
nothing about the binding had moved. (The pre-G1 code compared the arrays by value and did not.)

`Clear()` now demotes its entries to candidates instead of dropping them, and `Resolve` compares a
freshly built layout against the candidate for that mover (new `RigExecSkinTopology::operator==`,
the one place layouts are compared by value) and hands back the pointer it already had when they
are equal. One array compare per notice instead of one kernel pass per notice. Byte-identical: the
arrays are equal by value, so the kernel input and output are the same; only the counter and the
diagnostic that reports work move.

**Test.** `testRigExecSkinTopology::TestAnUnrelatedEditDoesNotRerunTheKernel`: two identical frames
(the second must execute nothing -- the baseline), a `DefinePrim("/Scratch")` between them, and the
next frame must still report `moverGraphRevisionsExecuted == 0` with the deformation unchanged and
the epoch digest unmoved -- then a real weight edit after it, which must land, so the cache is still
a cache and not a freeze.

### Mutants, and what killed them

| mutant | killed by |
|---|---|
| the lazy build removed from `Evaluate` | `testRigExecBakedMode::TestBakedModeRequestedOnADirtyEpoch` (build and generation counts stay 0) |
| `_bakeRefused` never set | `testRigExecBakedMode::TestANonBakeableRigFallsBack` (4 more build attempts, one per frame) |
| `AdoptGeometryStateFrom` not called | `testRigExecBakedMode::TestAnInEpochRebuildPublishesTheSameCounters` (0 vs 2 created, 0 vs 2 schedules, the mover graph diagnostic, and 3 parity mismatches) |
| the skin-layout candidate lookup disabled | `testRigExecSkinTopology::TestAnUnrelatedEditDoesNotRerunTheKernel` |
| the `ClearInteractiveOverrides` static-cache clear deleted | *nothing* -- and that is the point of S6: the clear is redundant for correctness and is kept as defence in depth |

### Gates, re-run on the whole tree

* Every target builds (`cmake --build build -j8 -- -k`); the only failure is the pre-existing
  usdNoodles `GL/glu.h`.
* ctest: **63 tests, 2 failures, exactly `{testRigExecCurvenet, testUsdNoodles}`** -- both
  pre-existing on this box. All 63 entries carry `TIMEOUT 120`.
* Dynamic `rigExecPose --frames ... --joints --targets --joints-out` is **byte-identical** to the
  `copies/merged` base binary on `Biped.usda` (1,2,3), `Biped_layered.usda` (1,2,3) and
  `Biped_anim.usda` (1..8), stdout and the written `.usda` alike; baked is byte-identical to
  dynamic on all three; epoch digest **16236336116187278719**.
* `RIGEXEC_ENABLE_PARALLEL_EVAL=0` is byte-identical to the parallel build in both modes.
* Paired timing, base binary interleaved with merged2 dynamic and merged2 baked, min of 5, load
  average 2.5-3.1 on this 20-core box (another session was building):

| stage | mode | compile | frame 1 | steady frame (mean, min-max) |
|---|---|---|---|---|
| `Biped.usda` | base | 103.2 | 19.10 | 8.49 (7.86-9.11) |
| | dynamic | 111.5 | 10.85 | **6.80** (6.55-7.06) |
| | baked | 134.3 | 2.28 | **0.62** (0.53-0.70) |
| `Biped_anim.usda` | base | 103.4 | 19.59 | 9.01 (8.47-10.04) |
| | dynamic | 114.2 | 11.40 | **7.16** (6.95-7.82) |
| | baked | 138.1 | 2.28 | **0.69** (0.60-0.94) |

Within noise of the pre-pass figures (dynamic 7.14, baked 0.63/0.71): nothing in this pass runs per
frame. The per-epoch skin-layout compare of S8 is paid once per notice, the rebuild carry of S2
once per rebuild, and the lazy bake check of S1 is two booleans and a pointer test.


## Landing fix: guides gate in the baked publication

Landing this work on top of commit b938864 (which makes `rigExecPose` disable the observational solver
guides and gates the guide request on `SetSolverGuidesEnabled`) exposed one interaction the merged tree could
not show: the baked program still published `solverFrames` for every aggregate solver while the dynamic path
published none, so `rigExecPose --mode parity` reported fourteen mismatches per frame on the biped (one per
solver) although every joint, matrix and point agreed. The program's publication now honours the same gate
as the dynamic walk (`_guideTaps && _solverGuidesEnabled`). Covered by `TestParityModeWithGuidesDisabled` in
`testRigExecBakedMode`, which runs the parity check with guides off (zero mismatches, no solver frames) and
on again (frames published, zero mismatches).
