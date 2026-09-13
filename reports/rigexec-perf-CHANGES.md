# RigExec biped performance: what changed, and what it bought

One patch, `reports/rigexec-perf.patch`, against the `biped-port` working tree (which already carries the
uncommitted profiler work: `libs/rigExec/profiler.h`, the profiler scopes in `rigEvaluator.cpp`, and
`--profile` in `tools/rigExecPose.cpp`). Nine files change -- five of them the engine, three tests and
an example, one a README:

| file | lines |
|---|---|
| `CMakeLists.txt` | +1 / -1 |
| `libs/rigExec/rigEvaluator.cpp` | +354 / -76 |
| `libs/rigExec/rigEvaluator.h` | +6 / -0 |
| `libs/rigExec/tapSet.cpp` | +27 / -2 |
| `libs/rigExec/tapSet.h` | +10 / -0 |
| `tests/testRigExecConstraints.cpp` | +178 / -9 |
| `tests/python/test_rigexec_stage_edits.py` | +41 / -1 |
| `examples/biped/Biped_anim.usda` | +179 / -0 (new) |
| `examples/biped/README.md` | +8 / -0 |

Compile on the biped goes from 444.2 ms to 100.2 ms. A steady-state evaluate goes from 26.7 ms to
7.8 ms on the static stage and from 53.7 ms to 8.8 ms on the animated one. Every number in this file
is a minimum over repeated runs on a quiet machine; the merged-tree numbers are from a paired run
that alternated the pristine and merged binaries run by run so both saw the same load.

Three tests are added and one example file is now shipped, so that the three changes that rewrite a
predicate or a lifetime rule -- E1's expiry rebuild, E3's blocking table at both of its call sites --
are covered by something that fails when they are reverted, and so that the animated stage the
evaluate numbers are measured on is reproducible from this patch alone. See **Tests** below.

Nothing here is gated on an environment variable or a build flag. Every pre-existing profiler scope
still fires; three new ones were added (`Compile.DigestJoin`, `Compile.WarmupJoin`, and
`TapPrepare warmup`). Output is byte-identical to the pristine build on all three biped stages and
the epoch digest is unchanged at 16236336116187278719.

---

## Compile

### C1 - memoize the structure digest (`rigEvaluator.cpp`, `_ComputeStructureDigest`)

`_ComputeStructureDigest` walked the rig and, for every solver prim, followed every attribute
connection and every pose input, then did the same again for every ancestor of every relationship
target. Nothing was remembered between visits, so the same prim's input closure was rebuilt hundreds
of times per digest -- the hips joint alone was visited 270 times. Three function-local caches now
sit in front of that work: one keyed by path for `_CollectPoseInputInfo`, one for each prim's
`appendSolverInputConnections` block, and a per-attribute-path text cache for the emit loop. They are
function-local on purpose: a stage edit between two digests is still seen, because each call starts
with empty caches.

Measured on its own: compile 444.4 -> 206.6 ms, of which `Digest.Solvers` 272.8 -> 30.7 ms.

### C2 - ask whether a connection is authored before composing it (`rigEvaluator.cpp`, 11 call sites)

`UsdAttribute::GetConnections` builds a Pcp property index and a target index even when the attribute
has no authored connection opinion at all, which is the common case on this rig. A file-local helper
`_AuthoredConnections()` returns an empty vector unless `HasAuthoredConnections()` is true, and the
nine value-consuming call sites now go through it. The two sites that use `GetConnections`' boolean
return -- `inheritsNamespacePose` and the animated-input probe -- keep the call and short-circuit on
`HasAuthoredConnections()` first, so the PCP-error case behaves exactly as before. This is sound
because `PcpBuildTargetIndex` derives targets from authored `ConnectionPaths` opinions alone.

Measured on top of C1: compile 206.6 -> 183.1 ms. It is on the evaluate path too, worth about
2 ms/frame there. Standalone it had measured -90 ms; most of that overlap had already been taken by
C1, which removed the repeated calls rather than making each one cheaper.

### C3 - do not validate adjustments that cannot exist (`rigEvaluator.cpp`, `_ValidateAdjustmentPoseConsumers`)

The adjustment-consumer validation walked the pose graph to produce an error that can only ever be
raised when a `RigExecCurvenetAdjustment` prim is present. It now returns true immediately if no such
prim exists anywhere on the stage. The scan is a whole-stage `UsdPrimRange` with instance proxies and
the all-prims predicate, not a rig-subtree walk: the validation itself follows connections and targets
that leave the rig, and an inactive, undefined or instanced Adjustment must still take the slow path.

Measured: compile 183.1 -> 164.2 ms, essentially all of it out of `Compile.DiscoverValidate`
(43.1 -> 22.9 ms). `testRigExecCurvenetAdjuster` and `testRigExecCurvenetAdjustments` carry
Adjustment prims, so they still take the full walk, and both still pass.

### C4 - memoize input closures for the lifetime of one compile (`rigEvaluator.cpp`, `Compile`)

The solver-scheduling and request-preparation passes recomputed `_CollectPoseInputInfo` 2,672 times
over 345 distinct prims and `_CollectAttributeConnectionInputs` 1,151 times over 115 prims. Two
compile-local maps keyed by `SdfPath` now back a `collectAttributeInputs()` lambda used by
`registerInput` and a `poseClosureCache` wrapping the pose-provider closure, so asking twice costs a
lookup. These are kept separate from the digest's own caches because C6 runs the digest on another
thread.

Measured: compile 164.2 -> 154.8 ms, mostly out of `Compile.SolverSchedule`. Less than the 20-30 ms
expected, again because C2 had already made each individual call much cheaper.

### C5 - resolve each prim's purpose once (`rigEvaluator.cpp`, transform-authority pass)

The transform-authority pass called `UsdGeomImageable::ComputePurpose` on the provider once per
descendant, and `ComputePurpose` walks up to the first authored opinion every time, so a deep joint
was resolved once per ancestor provider. The provider's own purpose is now hoisted out of the loop
and a pass-local `purposeCache` serves both the provider and every descendant. `ComputePurpose` posts
no diagnostics and has no side effects, so hoisting it cannot change what the pass reports.

Measured: compile 154.8 -> 141.1 ms, `Compile.DiscoverValidate` 22.2 -> 8.9 ms. (The hoist alone was
worth 4.3 ms; the per-prim memo took it the rest of the way.)

### C6 - compute the digest on a task (`rigEvaluator.cpp`, `Compile`; `CMakeLists.txt`)

Nothing between the digest's computation and its single read touches it, and compile authors nothing
to the stage, so the digest does not have to be on the critical path. It is now computed by a
`WorkDispatcher` task started right after the DiscoverValidate stamp and joined immediately before
`_structureDigest = newDigest`, under a new `Compile.DigestJoin` scope. `newDigest` is declared before
the dispatcher so `WorkDispatcher`'s waiting destructor covers every early return in between. The
digest reads only the stage, the rig path and the (mutex-guarded) profiler, and touches no mutable
member. `CMakeLists.txt` gains `work` in `target_link_libraries(rigExec PUBLIC ...)`.

Measured: compile 141.1 -> 117.2 ms. The digest itself inflates to 33-53 ms on the worker thread when
it runs concurrently with exec's own threaded prepare, and is still completely hidden:
`Compile.DigestJoin` measures 0.01 ms. `Compile.StructureDigest` from here on times only the dispatch.

### C7 - warm the exec network once, wide, off the critical path (`rigEvaluator.cpp`, `Compile`)

`TapPrepare` is exec compilation plus per-request scheduling, and the fourteen per-batch prepares each
paid their own share of compiling the same shared network. A single fused `RigExecTapSet` -- asking
`computePointFrame` and `computeRestFrame` for every joint, control and volume-weight provider, plus
`computePointFrameArray` for every aggregate solver -- is constructed on the compiling thread as soon
as those lists exist, and only its `Prepare()` runs on a `WorkDispatcher` task. It is joined under
`Compile.WarmupJoin` immediately before the first real tap-set construction in the scheduling loop, and
destroyed on the compiling thread after the guides preparation. The tap set's constructor and
destructor mutate `RigExecTapContext::clients`, an unguarded `std::set`, which is why both stay on the
compiling thread; nothing between the task's start and its join enters exec.

Measured: compile 117.2 -> 100.2 ms. `TapPrepare solverBatch` 29.9 -> 5.5 ms across all fourteen,
`poseSeed` 21.6 -> 9.2 ms; the warm-up costs 37-41 ms on the worker and `Compile.WarmupJoin` waits
10-13 ms for it.

This is the one change whose value depends on the machine. The host is heterogeneous (ten fast cores,
ten slow ones) and the warm-up task's own duration is bimodal depending on where TBB puts it: on a
quiet box it wins ~15 ms, and on a box saturated by other work a paired A/B put it ~10 ms behind the
C6-only build. The fused-but-serial variant was built and measured as a fallback and is not one: it
came out ~3 ms worse than no warm-up at all once C1-C6 are in. So the choice is this change or
nothing, and it is three self-contained hunks (the construction plus `Run()`, the `Compile.WarmupJoin`
block, and the `warmupTaps.reset()`) if a future maintainer wants deterministic compile times more
than the 15 ms.

---

## Evaluate

### E1 - stop rebuilding every request on every animated frame (`tapSet.cpp`, `tapSet.h`)

`RigExecTapSet::Prepare` handed `BuildRequest` a time-change callback that set `_prepared = false`, so
every request was rebuilt and re-scheduled on every frame of any stage with time samples -- 26 ms of a
56 ms animated frame, and nothing at all on a static stage. A time change invalidates values, not the
compiled request, so the callback now only sets `_dirty`. The second half of the change matters as
much: `Evaluate` now rebuilds on expiry too, `if (!_prepared || (_request && !_request->IsValid()))`.
Without it, a structural edit that invalidates a request under us -- a prim deactivated and
reactivated, a variant switched away and back, a payload unloaded and reloaded, a delete undone --
would leave the tap set returning nothing for the rest of the session.

Measured: animated steady-state frame 53.8 -> 29.1 ms. The static stage is unaffected, as expected.

The expiry rebuild also fixes a pre-existing bug rather than merely avoiding a new one. On the
pristine tree, deactivating `/Biped/Rig/Joints/hips_bind` and reactivating it leaves every subsequent
evaluate returning `valid=False`, 0 joints and "pose provider input evaluation incomplete", forever;
this reproduces deterministically in all three variants (same time code, across a time change, and
with no evaluate in between). With this change all three recover with 252 joints, and the following
session-layer-override and rewind frames evaluate correctly. Both behaviours were re-confirmed on the
merged tree against the pristine reference build.

### E2 - warm the shared executor before the override-bearing pull (`tapSet.cpp`, `tapSet.h`, `rigEvaluator.cpp`)

`ExecUsdSystem::Compute()` was never called: every one of the evaluator's pulls carried overrides, and
an override-bearing compute runs in a throwaway sub-executor seeded from the main one. With nothing
ever computed into the main executor it started empty every time, so the whole network was recomputed
behind every override. A new `RigExecTapSet::Warm(UsdTimeCode)` -- same prepare/expiry guard and
`ChangeTime` as `Evaluate`, then a plain `Compute()` with the cache view deliberately dropped -- is
called once per `Evaluate` on the pose-seed taps, immediately before the override-bearing evaluate.
Overrides are deliberately not also filtered: that is the same mechanism and the gains do not add.

Measured: static frame 27.8 -> 20.1 ms. Per scope, inside one static frame: `PoseSeed` 6.46 -> 0.37 ms
(the whole scope, warm call included), `AuthoritativeSnapshot` 2.07 -> 1.03 ms, the fourteen
`SolverBatch` scopes 2.94 -> 1.69 ms in total -- the snapshot and the batches get cheaper too, because
they are pulls on the same now-warm executor.

### E3 - a blocking table instead of re-deriving pose ownership per step (`rigEvaluator.cpp`, `Evaluate`)

Namespace propagation stops at a path that owns its own pose. Both walks that need that answer --
`commitConstraintFrames` and `refreshPoseProvider` -- rediscovered it by climbing the namespace and
re-reading `parent:space` off the stage at every step, once per propagated descendant: 13,719 calls a
frame over 194 distinct paths. That is quadratic along a joint chain, and it is exactly why the spine
constraints cost more at the root than at the tip. A per-evaluate memoized `nearestBlocking(path)` now
returns the nearest pose-owning ancestor-or-self, climbing over every path element up to the absolute
root rather than stopping at the first known provider (a provider's parent need not be a provider).
The blocked test becomes: the owner exists, is not `closest`, and lies strictly below it. That is
equivalent because `nearestBlocking(p)` is the deepest owner on the chain from `p` to the root and
`closest` is always a proper ancestor of `p`, so "some node in `[p, closest)` owns its pose" holds
exactly when that deepest owner is itself in that range. The identical replacement is applied at both
walks.

Measured: static frame 20.1 -> 8.9 ms, animated 21.9 -> ~9.9 ms. The largest single win on the
evaluate side.

### E4 - do not run the provider refresh when there is nothing to refresh (`rigEvaluator.cpp`, `Evaluate`)

`_connectedPoseTaps` is empty on this rig, yet `refreshPoseProvider` was entered 544 times a frame to
discover that. It now returns early when `_connectedPoseTaps` is empty, before the pending/active/
complete sets are built. The early return also skips the walk's only other effect, a runtime
"connected pose provider cycle" diagnostic; compile already rejects a cyclic pose DAG, so that
backstop cannot fire on a rig that compiled.

The post-walk sweep still iterates `_poseSeedFrames`, as it always did. An earlier version of this
change also narrowed the sweep to `_connectedPoseTaps`, on the claim that every connected provider is
a pose-seed provider too. That claim is false and the narrowing was wrong: `_connectedPoseTaps` is
built from every key of the pose-input map whose closure reaches a connection, and that map is seeded
from constraint targets, sources, world-up objects, effectors, weight objects and pole objects with no
frame-provider filter, while `_poseSeedFrames` takes only joints, controls and volume-weight types. A
plain `Xform` used as a constraint source, with an attribute connected to a control's `parent:space`,
therefore lands in the connected map and in no other. Refreshing it directly asks exec for a
`computePointFrame` it does not publish; the refresh fails, `Evaluate` returns early, and a rig that
used to evaluate normally -- reporting the unusable constraint input as passed through, which is the
documented behaviour -- returns an invalid pose with zero joints instead. The narrowing bought nothing
measurable either way: the early return above collects the whole win when the connected map is empty,
and when it is not, the walk has to run.

Measured: ~0.3 ms/frame static, ~0.7 ms/frame animated. Smaller than the 1.3 ms measured in isolation,
because E3 had already made the same walk much cheaper.

### E5 - stop asking the xform cache about intervening transforms that do not exist (`rigEvaluator.h`, `rigEvaluator.cpp`)

`_ComposeInterveningXforms` ran 326 `UsdGeomXformCache::ComputeRelativeTransform` calls per frame to
discover, on nearly every rig, that there is nothing between a provider and its anchor. Which
providers have a plain prim standing between them and their anchor is pure namespace topology, so
compile now records it once per epoch: `_poseProviderAnchors` (provider -> anchor path, empty meaning
the asset root) and `_interveningXformProviders`. The pass returns immediately when the candidate list
is empty and otherwise queries the cache only for the candidates. The correction loop itself is
deliberately *not* restricted to the candidates: a provider whose own intervening transform is the
identity must still be corrected when an ancestor anchor was corrected, so shrinking the loop would
silently leave descendants behind on rigs that do have such an Xform. The candidate test is a
deliberate superset of the original prim-level check.

Measured: ~0.3 ms off a static frame, flat within noise on the animated stage. Compile is unchanged --
the new per-epoch pass is a few hundred path walks and does not register. The non-empty path is
covered by `testInterveningXform` and by the intervening-Xform cases in `testRigExecImaging`.

### E6 - hint the pose-seed publish loop (`rigEvaluator.cpp`, `Evaluate`)

The pose-seed publish loop filled three empty `std::map`s in `_poseSeedFrames` key order, so every
insertion is an end insertion and `operator[] =` was paying for a tree descent it did not need. It now
uses `emplace_hint(map.end(), ...)`. Only this loop was converted: the other two publish loops index
their maps from the `_jointPaths` / `_controlPaths` vectors, whose order is not contractually
`SdfPath`-monotone, and `emplace_hint` does not overwrite an existing key where `operator[] =` does --
neither provably monotone nor semantically identical, for about 0.05 ms.

Measured: 0.3-0.6 ms/frame, at the edge of noise but consistent in direction across both stages and
repeated runs.

---

## Tests

Three changes here rewrite a rule rather than memoize a pure function, and the suite as it stood did
not notice when they were reverted. Each now has a test that does; each test was written against the
mutant it is supposed to catch, and each also passes against the *pristine* engine, so it doubles as
an old-versus-new equivalence check on a shape nothing else covers.

**`TestSolverOwnedJointBlocksNamespacePropagation`** (`tests/testRigExecConstraints.cpp`, new) covers
E3's first call site. A constraint moves a control group; under it sit a solver-bound joint with a
control of its own and an unbound joint with a control of its own. The bound pair must keep the frame
the solver wrote and the unbound pair must ride the constraint's delta, checked at two times, and then
again with the solver re-pointed at another joint so the same subtree must ride. The constraint's own
source is solver-driven, which schedules it after every solver batch -- with the constraint first, the
solver's later commit re-derives the subtree from its own output and hides a lost boundary, which is
why the suite's existing IK fixtures could not catch this. A build with the blocking test deleted
outright (`if (false && ...)`) used to pass all 49 ctests and print a byte-identical static biped; it
now fails twelve assertions in `testRigExecConstraints` -- four here and eight in the fixture below,
whose solver-owned descendant is checked in both phases.

**`TestConnectedParentSpaceSolverInputs`** (same file, extended) covers E3's second call site, which
nothing executed at all: the connected provider `Bridge` now has namespace descendants -- one
inheriting its pose, one written by a solver of its own, one owned by a constraint -- so the
base-phase descendant loop runs with both branches taken, and the test asserts a moved base frame and
an unmoved one at every one of its eight checks. Forcing that loop never to block -- which changed
nothing anywhere in the suite or on any biped stage before -- now fails eight assertions. Forcing it *always* to block still passes, and that is a property of the engine rather
than a hole in the test: every descendant with a connected tap of its own re-derives its base frame in
its own refresh immediately afterwards, so the loop's carry-write is only load-bearing for a
descendant the walk skips -- a solver-bound joint -- where blocking is the correct answer. The
carry-write half of that loop appears to be dead by construction; it is left in place rather than
removed, because proving that exhaustively is a separate job.

**`TestDeactivateReactivateRecovers`** (`tests/python/test_rigexec_stage_edits.py`, new) covers E1's
expiry rebuild, the half of E1 that is a behaviour change rather than a deletion. It opens
`examples/ArmShotAnim.usda`, evaluates, deactivates `/Shot/HeroArm/Rig/Joints/Shoulder`, checks the
empty pose that is the reported state while it is gone, reactivates, and requires three joints back at
the original time and at a later one. Without the expiry clause the evaluator never recovers: this
fails on the pristine tree exactly as it fails on a build with the clause removed.

**`examples/biped/Biped_anim.usda`** ships (179 lines, an overlay sublayering `Biped.usda` that keys
six controls over frames 1-8), and the biped README documents it. The animated evaluate numbers in
this file, and the only stage that exercises the animated path at all, were previously not
reproducible by anyone holding the patch. It is also the stage that sees E3: on the static biped every
avar sits at its default and the constraint deltas are near-identity, so a build with blocking deleted
prints identical stdout and differs in 3 lines of a 221 KB joints layer, where the animated stage
differs in 1136 lines of stdout.

---

## Merged result

Paired measurement, pristine and merged binaries alternating run by run on a quiet machine
(load 0.6-1.0), minimum of 7 runs on the static stage and 5 on the animated stage:

| phase | pristine | merged |
|---|---|---|
| Compile | 444.2 | 100.2 |
| Compile.DiscoverValidate | 48.1 | 11.6 |
| Compile.StructureDigest | 273.4 | 0.0 (dispatch only; the digest runs on a worker) |
| Compile.SolverSchedule | 56.9 | 41.2 |
| Compile.PrepareRequests | 60.0 | 43.2 |
| Compile.WarmupJoin | n/a | 7.8 |
| Compile.DigestJoin | n/a | 0.0 |
| Evaluate@1 / @2 / @3 (static) | 31.1 / 28.0 / 26.7 | 19.2 / 9.0 / 7.8 |
| Evaluate@2 .. @8 (animated) | 54.1 / 53.4 / 53.9 / 54.8 / 54.0 / 54.2 / 53.7 | 10.1 / 9.1 / 8.7 / 8.8 / 8.9 / 8.6 / 8.8 |

Correctness, on the merged build against a pristine build of the same tree: stdout and the
`--joints-out` layer byte-identical on `Biped.usda` (frames 1,2,3), `Biped_anim.usda` (frames 1..8) and
`Biped_layered.usda` (frames 1,2,3); stderr empty on all six runs; epoch digest 16236336116187278719;
ctest 47 of 49 passing with the failure set `{testRigExecCurvenet, testUsdNoodles}`, identical to
pristine. `testUsdNoodles` fails because the vendored glew in the usdNoodles dependency cannot compile
on this box (`GL/glu.h` is not installed); that is pre-existing and unrelated.

The same three gates and the same ctest run were repeated on a fresh copy of the checkout with only
this patch applied, so the tree the patch produces is the tree that was measured.

## Follow-ups

- The remaining compile time is `Compile.SolverSchedule` (42.6 ms) and `Compile.PrepareRequests`
  (46.8 ms), of which only about 16 ms is exec preparation. The rest is untraced non-exec work, and it
  is where a per-compile prim index would land -- deliberately not done here, since C1, C2 and C4
  superseded it for the digest, but the schedule and request passes still walk the stage more than once.
- The first evaluate of a session now costs 19.2 ms against a 7.8 ms steady state. That premium is the
  largest single per-session evaluate cost left and nobody has profiled what it is made of.
- The E4 sweep bug has no regression test. Any fixture that reproduces it needs a connected pose tap
  on a prim that publishes no `computePointFrame`, which makes exec post coding errors in a correct
  build as well as a broken one, so the test would be asserting on a rig that is already being
  reported as malformed. The scenario is scripted at `scratchpad/rev/e4_connected.py` instead: a plain
  `Xform` constraint source with an attribute connected to a control's `parent:space`, which must
  evaluate to a valid two-joint pose with the constraint reported as passed through.
- The base-phase carry-write in `refreshPoseProvider` looks dead: every descendant that could take it
  refreshes itself immediately afterwards. Removing it is a separate change with its own proof
  obligation.
