# Evaluate what the drag can reach, and nothing else

Design note, 2026-09-13. Phase 1 design, now partly built.

> **Superseded on the step-graph branch (2026-09-15).** Gates 1 and 2 below
> gated the DYNAMIC constraint walk and solver batches on a dirty set, and
> the `Sparse` / `SparseWithParityCheck` modes that carried them are gone.
> The single-graph configuration answers the same question inside the
> baked program: every step declares the slot ranges it reads and writes,
> and a run executes the CONE of what the sources say moved
> (`docs/baked-step-graph.md` §7, `RigExecBakedComputeClosure`). What this
> note measured still stands -- the closure is real and small, the floor
> was the problem -- and Gate 3 survives as
> `RigExecRigEvaluator::_OverridesReachSkinLayout`, beside the same rule
> for the blend sample shapes. `tests/python/test_rigexec_baked_cone.py`
> and `test_rigexec_skin_layout_overrides.py` are the tests that replaced
> the two this note's gates carried.

IMPLEMENTED: the profiler scopes this note's measurements depend on
(`PublishProviders`, `PublishJoints`, `PublishControls`, `FrameSeed`,
`PoseWalkSetup`), **Gate 3**, and **Gates 1 and 2** behind
`RigExecEvaluationMode::Sparse` / `SparseWithParityCheck`. Gates 4 and 5 are not
built.

Written after the Phase 0 measurements. Two claims in it have been retracted
and corrected since — the size of the publish, and a reported compiler bug that
turned out to be an ABI mismatch (see "A third of the floor used to be
invisible" and "A phantom, and the lesson it cost"). Both corrections are left
in place rather than edited away, because the same root error produced both and
the next person working here should see it.

The measurements are reproducible: `tools/biped/spikes/r1_control_closure.py`
and `tools/biped/spikes/r2_flat_cost.py`, findings and numbers in their module
headers.

## What happens today

Dragging one control re-solves the whole rig, and the cost does not know which
control it was.

| case | min ms | joints downstream | skin points downstream |
|---|---|---|---|
| re-evaluate, nothing changed | 17.7 | — | — |
| fingertip `index_004_l` | 24.9 | 2 of 252 | 203 of 26,276 |
| FK wrist | 24.3 | 33 | 2,964 |
| spine_end_ctl | 25.8 | 141 | 19,758 |
| hips_ctl | 24.9 | 218 | 22,718 |

Wall clock, profiling off, minimum of 20 interleaved samples — this box is
shared and a rig evaluate is TBB-parallel, so the minimum is the only stable
statistic on it. The per-phase table below is a separate profiled run, so its
totals differ by a few percent.

Fingertip and root are within 1% of each other while their downstream work
differs by about a hundredfold. Per phase, ms/frame, **top-level only** — the
profiler's own `Summarize()` totals nested spans too (`ExecEvaluate` sits
inside `SolverBatch`, `Assemble` inside `Chain`), so adding its rows
double-counts; this charges each span to its outermost parent, which is the
only breakdown that adds up to the evaluate:

| phase | nothing changed | fingertip | hips | what the fingertip needs |
|---|---|---|---|---|
| **TOTAL evaluate** | 17.86 | 25.57 | 27.53 | |
| constraint walk | 7.13 | 7.34 | 7.36 | **0 of 98** |
| `SolverBatch*` | 1.09 | 5.71 | 6.82 | 1 of 30 levels |
| `Chain*` (the skin) | 2.63 | 3.40 | 3.55 | 203 of 26,276 points |
| `AuthoritativeSnapshot` | 2.39 | 2.72 | 2.68 | ~6 of ~870 taps |
| (no scope: residual) | 1.82 | 1.73 | 1.81 | — |
| `SolverGuides` | 0.08 | 1.23 | 1.24 | viewport only |
| `PropertyChains` | 0.99 | 0.91 | 0.93 | all of it |
| `PoseSeed` | 0.05 | 0.92 | 1.44 | — |
| `PublishProviders` | 0.61 | 0.58 | 0.61 | ~0 of 370 |
| `PublishJoints` | 0.45 | 0.42 | 0.45 | ~6 of 756 writes |
| `FrameSeed` | 0.39 | 0.38 | 0.42 | — |
| `PoseWalkSetup` | 0.15 | 0.15 | 0.15 | — |
| `PublishControls` | 0.08 | 0.07 | 0.08 | — |

**Only two rows scale with the change at all**: `SolverBatch` and `PoseSeed`.
Every other row is flat, and several are *larger* for the fingertip than for
the root — which is the clearest possible statement that they are proportional
to the rig and not to what moved.

Three facts set the shape of everything below.

**The floor is the problem.** A re-evaluate with *nothing changed* still costs
about 17 ms, and it is four costs — constraint walk 6.95, publish 3.29, skin 2.32,
snapshot 2.23 — none of which has a change-driven gate. The caches that *do*
exist work well: `SolverBatch` falls to 1.05, `PoseSeed` to 0.06,
`SolverGuides` to 0.06. The floor bounds every other case, so it is the first
target and not a footnote: collapse the floor and every drag above it collapses
with it.

**A third of the floor used to be invisible, and instrumenting it corrected
this note.** An earlier revision of this design found 3.3–3.9 ms sitting inside
no profiler scope and attributed it, *from reading the code*, to the publish —
252 × 3 unconditional `std::map` inserts. That reading was wrong. Scoping it
(`PublishProviders`, `PublishJoints`, `PublishControls`, `FrameSeed`,
`PoseWalkSetup`, added to `_EvaluateDynamic` as part of this work) showed the
publish is **1.14 ms**, the per-provider frame seed another 0.39, and ~1.8 ms
is still residual. The gate order below changed as a result. Recorded because
the failure mode is seductive: the code really does do 756 unconditional map
inserts, the story really did fit the number, and it was still wrong.

**The closure is real and small.** A fingertip reaches 2% of the joints, 1 of
30 solver batch levels, none of the 98 constraints, 0.8% of the skin. Hands,
feet and arms sit at 8–18%. Only the root is dense.

## The invariant that makes this safe

> **The sparse dirty closure is a conservative SUPERSET, and every narrowing
> the dynamic path performs is left to the dynamic code that already performs
> it.**

The dynamic path narrows in many places — `nearestBlocking` stops a pose
revision at an independently solved joint, a disabled mover passes through, a
zero weight contributes nothing, an invalid frame aborts a constraint. The
sparse map models **none** of those. It models only structure: who can read
whom. Over-marking is slow; under-marking is a limb in the wrong place. Every
judgement call in the map resolves toward "dirty".

This is also the answer to the finger-root case in `r1`: `index_001_l_bind_fk`
is overwritten by a parent constraint, so its own avars change nothing
visible, and its downstream joints move by ~1e-13. A value-diffed invalidation
would call that clean. The structural closure calls it dirty, which is why
invalidation is structural and never value-diffed.

## Where the map lives, and when it is built

`RigExecSparseMap`, owned by `RigExecRigEvaluator`, built at the end of
`Compile` and dropped whenever `_structureDigest` moves. Exactly the lifetime
`_bakedProgram` has — and for the same reason — with one important
simplification:

**The map holds no values.** The baked program captures constants, so the
epoch digest cannot speak for it and it needs its own capture index and a
`_bakedProgramStale` flag. The sparse map holds only "which prim can be read
by which step", which is precisely what a binding epoch fixes. No value edit
can invalidate it; only a structural edit can, and the digest already catches
those. So there is no capture index, no staleness flag, and no
`_sparseRefused`.

Nothing has to be discovered to build it. Every edge already exists in the
compiled tables:

| edge | source |
|---|---|
| solver ← its prerequisites | `_solverBatches[].dependencies`, `.frameInputs` |
| solver → its joints | `_solverJoints` |
| constraint ← its inputs | `_frameConstraints[].sources`, `.worldUpObject`, `.effector`, `.poleObjects`, `.weightObject` |
| constraint → its targets | `_frameConstraints[].targets`, `.ikChain`, `.pointsTarget` |
| provider ← posed providers it reads | `_poseProviderInputs`, `_poseProviderAnchors` |
| property chain ← / → | `_propertyChains`, `_propertyChainOrder` |
| skin ← its influences | `_graphChains[].binding.influences` |
| chain ← another chain's phase | `_chainOrder`, `_snapshotPoints` |

Plus two namespace rules that are not relationships:

- a control's frame rides to its namespace descendants;
- **a joint's write propagates to its descendants** — the rule
  `commitConstraintFrames(..., propagate)` implements.

That second one is load-bearing enough to deserve a rule of its own: the map's
joint→descendant edges must be generated by **the same function**
`commitConstraintFrames` walks, not a second hand-rolled copy. If the two ever
disagree, sparse is silently wrong. One function, two callers.

## The invalidation signal

**On the hot path: the overrides.** `SetInteractiveOverrides` is handed the
exact `(prim, attribute)` pairs a drag changed. That is a far better signal
than diffing, and it is already the route the gizmo takes
(`rigExecImaging/registry.cpp` `_ResolvePreviewSample` →
`RigExecRigEvaluator::SetInteractiveOverrides`). The dirty seed is the symmetric
difference between the previous override set and the new one — symmetric, so
that *releasing* a drag correctly dirties what it was holding.

**Off the hot path: the notices.** `_OnObjectsChanged` already receives changed
paths. Those seed the same closure. A notice that cannot be resolved to a path
(a resync, a layer mute) dirties everything, which is today's behaviour.

**Time.** A time change dirties every step that reads a time-varying input.
Conservatively, in the first cut, a time change dirties everything — the
animated-scrub case is not what this work is for, and pretending otherwise
would put the hardest correctness question in the first landing.

## The five gates, in payoff order for a fingertip drag

### Gate 1 — the constraint walk. 7.34 ms, 20 of 98 needed. **LANDED**

The largest line in the profile, with no gate on it today at all — 7.15 ms even
when nothing changed.

**The skip rule first published in this note was wrong, and it is corrected
here loudly** (`tools/biped/spikes/r3_constraint_skippability.py`).

It said: skip a constraint whose *inputs* are all clean. That accounts for only
half of what `commitConstraintFrames` does. The other half is propagation — for
every provider descendant `D` of a target, it composes the delta the target just
moved through and applies it to `finalFrames[D]` **as D currently stands**. So a
constraint's output for `D` is a function of `D`'s own current frame, and if `D`
is dirty the constraint must re-run *even though none of its inputs moved,
because one of its outputs did*. That is an edge `D → C` the first closure never
had.

**The corrected rule.** A constraint must run when any of these is dirty:

- one of its source bindings (`sources`, `worldUpObject`, `effector`,
  `poleObjects`, `weightObject`);
- one of the providers it writes (`targets`, `ikChain`, `pointsTarget`);
- **one of the providers it would propagate onto** — every provider descendant
  of a target.

The third is over-approximated on purpose: the real walk narrows it with
`nearestBlocking`, which stops propagation at an independently solved joint.
Per the invariant at the top of this note, the map does not model that
narrowing — over-marking is slow, under-marking is a limb in the wrong place.

**What the correction costs**, constraints that must run:

| control | inputs-only (wrong) | corrected | skippable |
|---|---|---|---|
| fingertip `index_004_l` | 0 | **20** | 78 (80%) |
| `arm_l_fk_wrist` | 13 | 33 | 65 (66%) |
| `clavicle_l_ctl` | 16 | 33 | 65 (66%) |
| `leg_l_ik` / `toe_l` | 16 | 20 | 78 (80%) |
| `spine_end_ctl` | 60 | 62 | 36 (37%) |
| `hips_ctl` | 98 | 98 | 0 (0%) |

A fingertip goes from "no constraints" to 20, and Gate 1 survives comfortably:
~5.9 ms of a 21.4 ms drag rather than the ~7.3 the wrong rule promised, and the
root correctly skips nothing. The extra 20 are the `hand_?_follow` parent
constraints and their neighbours — they target the finger root controls, and
every finger control is a namespace descendant of one, so any finger drag lands
inside their propagation set. Exactly the mechanism the corrected rule exists to
catch, invisible to the rule it replaces.

**REPLAY, NOT SKIP — and this re-orders nothing, which is why it is the right
primitive.** A clean constraint cannot simply be stepped over. `finalFrames` is
seeded from the BASE frames and a later constraint reads whatever stands there
at that moment, so a stepped-over constraint would leave its targets at base and
change what every later step sees. Seeding `finalFrames` from the previous
generation's FINAL values instead is worse: an early constraint would then read a
value that had not been written yet in the true order.

So a clean constraint **replays its recorded outputs** at the same point in the
walk: the providers it wrote last generation — targets plus everything it
propagated onto — are written back, and only the *solve* is avoided. Same
order, same values, none of the work. This needs a per-constraint output record
(`moverPath → {provider → frame}`), refreshed whenever the constraint runs.

Correctness follows directly: if nothing in the constraint's input closure or
output set is dirty, re-running it would produce exactly the recorded outputs,
so replaying them is not an approximation of the answer — it *is* the answer.

**Landed**, `RigExecEvaluationMode::Sparse` / `SparseWithParityCheck`:

| case | dynamic | sparse | saved |
|---|---|---|---|
| **nothing changed** | 16.27 | **8.62** | 7.65 ms (47%) |
| fingertip | 24.32 | 19.56 | 4.75 ms (20%) |
| FK wrist | 24.68 | 20.85 | 3.83 ms (16%) |
| `toe_l` | 25.16 | 21.99 | 3.18 ms (13%) |
| `spine_end_ctl` | 26.43 | 27.36 | **−0.94 ms** |
| `hips_ctl` | 27.48 | 28.33 | **−0.85 ms** |

Constraint steps replayed of 98: fingertip 66, arm 53, toe 72, spine_end 24,
**hips 0** — the root reaches every constraint through the propagation sets, so
replaying any of them would mean the dirty set was too small.

**The dense cases are ~0.9 ms SLOWER**, and that is the honest cost of the
gate: the dirty-set closure is a `std::set<SdfPath>` BFS over a large graph,
paid in full before a root drag discovers it can skip nothing. Acceptable
because the mode is a request and the cases that regress are the ones an
animator holds least often, but it is real and it is the first thing to fix if
Sparse is ever made the default — a `vector<bool>` indexed by a compile-time
node ordinal would remove most of it.

**Two corrections the implementation forced**, both found by the parity gate
rather than by reasoning:

1. **The closure exploded.** Adding both directions of the propagation edge —
   `descendant → constraint` and `constraint → descendant` — makes the graph
   nearly complete: a dirty finger joint reaches the hips constraint (it is a
   descendant of its target), which reaches every joint, which reaches every
   other constraint. Measured: **2 of 98 replayed** where the offline model
   said 78. The fix is that the two edges do not mean the same thing and must
   not share a node. A constraint now has two: `delta` ("what it composes
   moved" → it must re-run *and* everything it writes changes) and the mover
   itself ("it must re-run" → nothing else follows). A dirty descendant
   reaches only the second. That is not a weakening: a constraint forced to
   re-run purely because a descendant moved writes that descendant a new frame
   and every other descendant the same one, because its delta is unchanged.

2. **Parity compared work, not just values.** `RigExecComparePoses` also
   compares the mover-graph counters, the solver rounds and the diagnostics —
   and two generations run back to back in one parity call cannot agree about
   those, because the second reuses every cache the first warmed. It reported
   a constant **2 mismatches on every probe, including the ones that replayed
   nothing**, which is the tell that the disagreement was never about the gate.
   `RigExecComparePoses` gained a `compareWork` parameter, default true so
   baked parity is untouched; sparse parity passes false and compares values.

Gates: `tests/python/test_rigexec_sparse_constraints.py` (in ctest) — parity at
zero, bit-exactness against a fresh rig across five probes × three values plus
the release edge, and the half that matters: the gate must fire for a leaf drag
and must fire **not at all** for the root.

### Gate 2 — the solver batch schedule. 5.71 ms, 1 of 30 levels needed. **LANDED**

There is already a cache here and it is excellent — for the wrong frames. It
copies `baseOverrides`, appends the batch's prerequisites, and compares the
whole vector against the one that produced the standing snapshot. On an
unchanged frame that yields **0 solver evaluations**. On a *dragged* frame it
misses every time, because the interactive override rides in the shared
`baseOverrides` prefix, so every batch's vector differs whether or not that
batch can see the avar.

Measured before Gate 2 — aggregate solvers re-solved:

| | solvers |
|---|---|
| nothing changed | 0 |
| fingertip drag | **24 of 24** |
| root drag | 24 of 24 |

A fingertip drag re-solved the entire rig's solver schedule, exactly as much as
a root drag, when the closure says **1** solver is downstream.

The fix replaces the value comparison with the dirty set: a batch answers from
its standing snapshot when none of its solvers is reachable from what moved.
Structural, so a value that happens to arrive unchanged cannot make a reachable
batch look clean. After:

| | solvers re-solved | batches reused of 30 |
|---|---|---|
| fingertip | **1** | 23 |
| FK wrist | 7 | 17 |
| root | 24 | 0 |

Exactly the 1-of-24 the Phase 0 closure predicted. `SolverBatch*` scopes:
**5.44 ms dynamic → 1.30 ms sparse.**

Per the Gate 1 lesson, the gate is on `sparseActive`: Dynamic builds the input
vector and compares it exactly as before, and its measured cost is unchanged
(constraint walk 7.65, solver batches 5.44). The one addition to that path is a
bool test per batch.

Gates: `tests/python/test_rigexec_sparse_constraints.py` gained
`TestSolverBatchesAreReusedAndKnowWhenNotTo` — reuse must happen for a leaf and
must be **zero** for the root.

### Gate 3 — geometry. 3.40 ms, 203 of 26,276 points needed. **LANDED**

The profile separates two costs and they wanted different answers. Only the
first turned out to be needed.

**`Assemble body_geo_skin`, ~1.8 ms, paid even when nothing changed.** The
cause was not the assembly. `SetInteractiveOverrides` cleared the epoch-scoped
skin LAYOUT cache — the resolved `rigExec:jointIndices` / `jointWeights` /
`elementSize` table — on *every call*, so every mouse sample of a drag re-read
and re-validated the whole 26,276-point binding table. The comment justifying
it was reasonable in general ("an override is a value the static reads must
prefer over the stage, and the skin layout is read through exactly that
route") and wildly over-conservative in fact: a layout is a function of exactly
three attributes on the skin mover, and a drag overrides an avar on a control.

Measured before believing it — `Assemble body_geo_skin`, ms/frame:

| | |
|---|---|
| override set once, then evaluated | 0.17 |
| a fresh override every frame (a drag) | 2.08 |
| **the SAME override re-pushed every frame** | **2.23** |

The third row settles it. Re-pushing an *identical* value changes no answer
anywhere and cost 2.23 ms of assembly and 4.9 ms of evaluate (14.3 → 19.2),
purely to rebuild a table that could not have moved.

The fix is `_SkinLayoutInputNames` / `_OverridesReachSkinLayout`, mirroring the
`_RestInputNames` precedent exactly — including its conservative reading that
a computation override names something the test cannot inspect and therefore
counts. Both edges of a drag are tested, because a layout resolved while an
override stood is falsified by that override going away just as by its
arrival.

Result: `Assemble` 2.08 → **0.06 ms**; `Chain*` 3.40 → **1.66**; wall-clock
fingertip drag 24.9 → **21.4 ms**; no-change floor 17.0 → **16.2**.

**`GraphEvaluate`, ~0.2 → ~0.9 ms.** The mover graph already gates on packet
equality, which is why it costs 0.2 when nothing changed. Per-point sparsity —
running the kernel over 203 points instead of 26,276 — is a further win, but it
changes `RigExecMoverGraph`'s value contract and is precisely the kind of
boundary work that is right 99% of the time. **Not done, and not planned for
the first cut.** The layout gate alone took the fingertip's geometry from 3.40
to 1.66; per-point output masking gets its own phase and its own parity gate,
later, or not at all.

Gates: `tests/python/test_rigexec_sparse_skin_layout.py` (in ctest), holding
both directions — bit-identical across four probes × three values plus the
release edge, and a timing assertion that a layout-naming override *still*
invalidates, because an invalidation that never fires passes every value
assertion ever written.

### Gate 4 — `AuthoritativeSnapshot`. 2.72 ms, ~6 of ~870 taps needed.

The most structural of the four, and **it cannot be made incremental in
place.** `RigExecTapSet::Prepare()` front-loads one batched `ExecUsdRequest`
and `Evaluate(time, overrides)` computes that whole request; the exec API has
no "compute a subset of this request" call. So it needs a different shape, and
there are two candidates:

**(a) Partition the tap set at compile** into per-region requests and evaluate
only dirty regions. Costs: `Prepare()` is already among the most expensive
things `Compile` does and this multiplies it by the region count; and exec's
cross-request caching is what `Warm()` exists to exploit, so many small
requests may lose more than they save.

**(b) Stop paying for the publish.** A large share of that 2.47 ms may not be
exec at all. The snapshot copies ~870 `VtValue`s out, and then the publish does
252 × 3 `std::map<SdfPath, ...>` inserts — red-black tree inserts, every
frame, unconditionally, for joints that did not move. Publishing through a
vector indexed by the compile-time joint ordinal, with map access built lazily
only if a consumer asks for it, removes that without touching exec.

**Measured since the first draft of this note**, and it changes the answer: the
`AuthoritativeSnapshot` scope brackets *only* `_taps->Evaluate`, so the 2.97 ms
is exec compute plus the `VtValue` copy-out, and the publish is the separate
3.79 ms of Gate 3. That means (b) is largely Gate 3's job and this gate is
genuinely exec.

So the remaining question is narrower: is 2.97 ms of exec worth partitioning?
It is 11% of a fingertip drag and it is the hardest of the five. **I would land
Gates 1, 2, 3 and 5 first, re-measure, and only then decide.** A gate that
needs `Prepare()` run N times per epoch to save 2.5 ms is a bad trade if the
other four have already taken the drag to 8 ms; it is a good one if they
have not.

### Gate 5 — the publish. 1.14 ms, and until this work it had no profiler scope.

The one this design found rather than inherited — and the one it then had to
revise downward. Every generation does 252 × 3 `std::map<SdfPath,
RigExecPointFrame>` / `<SdfPath, GfMatrix4d>` inserts — 756 red-black-tree node
allocations — for joints that overwhelmingly did not move, plus a provider walk
over every `finalFrames` entry. Measured, that is 1.14 ms, not the 3.79 ms this
note first claimed. Flat across every probe, so still rig-proportional, but it
is now the smallest of the five gates rather than the third-largest.

**This is where the honest-publication design pays for itself, but only if it
is built the right way.** Carrying the previous pose forward by *copying* those
maps buys nothing: a 252-entry `std::map` copy costs about what 252 inserts
cost. The fix has to remove the per-entry work, not relocate it:

- the joint set is **fixed for the epoch**, so joint results belong in a
  `std::vector` indexed by a compile-time ordinal, copied as one block and
  written only at the ordinals the closure touched;
- `RigExecRigPose`'s `std::map` members become views built lazily, or
  `shared_ptr<const>` handles so an unchanged map is shared and not copied.

The first is the real fix and it also makes the carry-forward free. The second
is the compatibility layer for consumers that index by `SdfPath` today
(`pose.joint_frame(path)`, the imaging bridge, the Python bindings), and it must
not change what any of them return.

Expected: fingertip ~1 ms saved, floor ~1 ms saved. Small — but it is
prerequisite work rather than optional, because the honest-publication
carry-forward has to be built on the vector either way.

## A phantom, and the lesson it cost

RETRACTED. An earlier revision of this note reported a compiler bug: moving
`stampRegion("PoseWalkSetup")` one line earlier appeared to turn
`testRigExecNoAuthoring` into an access violation and
`testRigExecWeightOverlay` into `0xC0000409`. It claimed `_EvaluateDynamic`
was past what MSVC can reliably codegen, and concluded that extracting the
pose walk was a **correctness** prerequisite.

None of that is true. The crashes were an ABI mismatch.
`rigExecImaging/bridge.h` includes `rigEvaluator.h` and therefore `types.h`,
and `types.h` was being edited in another window at the time —
`RigExecBlendSampleData` gained a `shared_ptr` member, changing its size. A
partial rebuild left `rigExec.dll` and `rigExecImaging.dll` disagreeing about
that layout: an access violation in one imaging test, a stack-cookie failure
in another. With the tree settled, the "crashing" placement is green at both
`/O2` and `/Od` (MSVC 19.38.33145, `/O2 /Ob2 /EHsc -std:c++17 -MD
/permissive- /bigobj`, no `/GL`).

The clue was in the data and went unread: **both** failing tests were imaging
tests, which the statement-placement theory never explained and the ABI theory
explains immediately.

The methodological error is worth more than the finding was. Each bisect step
triggered a rebuild, and whether that rebuild happened to resynchronise the two
DLLs depended on which headers ninja saw as newer — so the pass/fail pattern
tracked rebuild scheduling, not the edit under test. **A single non-reproduced
observation was treated as a controlled experiment.** The failing state was
never re-run to confirm it was reproducible before a mechanism was published
for it.

That is the same failure as the publish attribution above: a story that fit the
evidence, believed because it fit, rather than tested against a re-run. Twice
in one day, from the same root.

Two things follow.

- **The extraction is a structural nicety again, not a correctness
  prerequisite.** It is still worth doing — a 2,400-line function with dozens
  of capturing lambdas is hard to reason about and harder to gate — but
  nothing about it is urgent, and Gate 1 may be written inline if that turns
  out simpler.

- **Do not bisect across a shared tree another agent is writing.** Either take
  the file to yourself for the duration, or re-run every state at least twice
  before believing the pattern. This is an argument FOR building Gate 3 first
  and staying out of `_EvaluateDynamic` while the sparse-blend work lands.

## How to turn it on

**The viewport is `Dynamic` by default and stays that way.** Only Gates 1 and 3
are built, and the dense end of the range is not yet understood end to end: a
root drag replays nothing and still pays for asking.

One session switch, read by `_DefaultEvaluationMode()` in the evaluator's
constructor — so it reaches the imaging bridge, the tests, and every tool at
once, because they all build an ordinary evaluator:

    set RIGEXEC_EVALUATION_MODE=sparse        (Windows)
    export RIGEXEC_EVALUATION_MODE=sparse     (POSIX)

Values: `dynamic` (default), `sparse`, `sparseParity`, `baked`, `parity`.
Anything else warns and uses `dynamic` — asking for a mode and quietly getting
another is how a performance claim becomes untrue without anyone noticing.

`sparseParity` is the one to run a suite under: it evaluates both paths and
counts disagreements, so an existing test that never heard of the mode still
fails on one.

A note for whoever extends this next: **there was already a
`RIGEXEC_EVALUATION_MODE`**, read in the evaluator constructor and therefore
already reaching every consumer. A first attempt added a second reader in
`libs/rigExecImaging/bridge.cpp` with its own vocabulary, and the two warned
separately about the same value. Grep for the variable before adding a reader
for it.

## A regression, and a wrong explanation for it

Gate 1's machinery is gated on a sparse mode being *requested*, not merely
present, and that distinction is worth 0.9 ms/frame on the default path.

The first version recorded each constraint's outputs unconditionally. A hips
constraint propagates onto ~230 providers, so that is kilobytes of
`RigExecPointFrame` copying per constraint per frame, 98 times over — paid by
`Dynamic`, which never replays any of it. Measured on the constraint walk for a
fingertip drag: **7.34 ms before Gate 1, 8.25 ms with recording unconditional,
7.49 ms once gated** (the 0.15 ms residual is at this machine's noise floor),
against **3.98 ms** in a sparse mode.

That regression was first blamed, in this note and in a report, on the
dirty-set closure being "a full `std::set<SdfPath>` BFS paid to discover it can
skip nothing". **The BFS measures 0.04 ms/frame and was never the cost.** The
attribution was a mechanism that fit the number, offered without measuring it —
the same slip as the publish size and the phantom miscompile above, and all
three were in asides about secondary numbers rather than in the findings
themselves. Measure before attributing, including in asides.

## The cost of asking, and what is left of it

A drag that can skip nothing pays the sparse question in full and buys nothing
back. That toll was **growing** as gates landed — a root drag was 0.85 ms
slower than the dynamic walk after Gate 1 and **1.52 ms** slower after Gate 2,
because each gate asks another question — and it was the whole argument against
making Sparse the default.

**Fixed, and the fix separated two costs that had been confused.** The dirty
set moved from a `std::set<SdfPath>` rebuilt every generation to generation
stamps over compile-time node ordinals (`_IndexSparseForwardMap`): the walk
became index arithmetic, the ~128 per-constraint and per-batch tests became one
array read each, and a generation now allocates nothing. Min of 6 trials × 12
frames:

| | dynamic | sparse | delta |
|---|---|---|---|
| fingertip | 24.21 | **15.91** | **+8.30 ms (34%)** |
| root | 25.74 | 26.24 | −0.51 ms |

The root penalty fell from −1.52 to **−0.51 ms**, and the residual is *not*
what it was assumed to be. Measured on a root drag, which replays zero
constraints and reuses zero batches, so every difference is structural:

| | ms/frame |
|---|---|
| the dirty set itself | **0.048** |
| solver-batch checks | +0.01 |
| constraint walk | **+0.42** |

So **the cost of asking is essentially gone** (0.048 ms), and what remains is
the cost of being *able* to replay: recording each constraint's outputs so a
later generation could reuse them. That is a different thing and a more
defensible one — it is the price of the capability, not of the question — and
it is why a root drag cannot be made free without giving up the mechanism.

Attributed by measurement rather than by mechanism, deliberately: the previous
attribution of this same regression to the BFS was wrong (see "The cost of
asking" retraction), so this one was localised by scope before being explained.

## How to reproduce every number in this note

All four probes live in `tools/biped/spikes/`, take no setup beyond the
environment below, and print the engine they loaded on every run.

    # The USD install's site-packages MUST be first, or `from pxr import ...`
    # resolves a global copy built against a different USD and the process
    # dies with an access violation on the first Rig construction.
    R=<repo>; U=<usd-install>; B=$R/build          # or your own build dir
    export PYTHONPATH="$U/Lib/site-packages;$B/python;$R/tools/biped"
    export PXR_PLUGINPATH_NAME="$B/usd/rigExecSchema/resources;$B/usd/rigExecImaging/resources"
    export RIGEXEC_USD_INSTALL="$U"
    export PATH="$B:$U/lib:$U/bin:$PATH"

| script | answers | arguments |
|---|---|---|
| `r1_control_closure.py` | how much of the rig one control can reach, empirically and structurally | none |
| `r2_flat_cost.py` | where a drag's time goes, per phase, top-level only | none |
| `r3_constraint_skippability.py` | how many of the 98 constraints a drag can replay | none |
| `r4_sparse_gain.py` | **dynamic vs sparse, per drag, plus the root's per-scope structural cost** | `[stage.usda [rigPath]]` |

**`r4` is the one to re-run.** It defaults to `examples/biped/Biped.usda` and
`/Biped/Rig`; a second rig is one argument, which is how the "every number here
is one biped" gap gets closed.

Its statistic, which is what makes two runs comparable rather than merely
similar:

- **6 trials × 20 interleaved samples**, and the reported number is the
  **minimum**. Contention only ever adds time, so the minimum is the closest
  estimate of the uncontended cost and the only statistic stable across runs on
  a shared box.
- Cases are **interleaved within a trial**, so a burst of someone else's build
  is spread across all of them rather than destroying whichever was running.
  The comparison *between* cases is the whole point.
- The **median is printed beside the minimum**. When they diverge the box was
  busy and only the minimum should be quoted; **when they converge the box was
  quiet**, which is the check that a quiet-machine run actually was one. On the
  contended run recorded here they are far apart — `nothing changed` reports a
  15.98 ms minimum against a 24.61 ms median.
- Drags push a **fresh override value every frame** (`avars:rz`, `1.0 + 0.37i`).
  A repeated identical value would measure the cache rather than the drag —
  which is exactly the distinction that found Gate 3.

The **root per-scope block** is the structural half and is far less noisy than
wall clock: a root drag replays nothing and reuses nothing, so every difference
between the modes is the price of the machinery rather than of the work it
saved. The claims about the cost of asking versus the cost of being able to
replay rest on that block, not on the wall-clock table.

## Should Sparse be the default?

Closer than it was, and the honest answer is still "measure once more".

| case | dynamic | sparse |
|---|---|---|
| nothing changed | 14.05 | **9.37** (−33%) |
| fingertip | 24.21 | **15.91** (−34%) |
| FK wrist | 23.30 | 16.38 (−30%) |
| `toe_l` | 22.62 | 17.54 (−22%) |
| `spine_end_ctl` | ~22.6 | ~22.5 (noise) |
| root | 25.74 | 26.24 (+2%) |

Sparse now wins large below roughly 40% closure and is within noise above it,
with a worst case of about +2% rather than +6%. Wall-clock repeats on this
shared machine swing ±1.5 ms, so the dense cases can only honestly be called
*noise-dominated*, not *neutral*.

What would settle it:

1. **A quiet-machine confirmation** that the dense cases are genuinely
   non-negative. Everything here was measured against other agents' builds.
2. **A second rig.** Every number in this note is one biped, against 33 rig
   stages in `examples/`. A different constraint-to-solver ratio could invert
   the conclusion.
3. Only if those leave it ambiguous: a closure histogram over all 116 controls
   weighted by `examples/biped/Biped_picker.json`, which is what a rigger chose
   to expose and a better proxy for what gets dragged than uniform weighting.

(1) and (2) are cheap and factual; (3) is a judgement about the work rather
than the engine and should not be needed.

## Honest publication

> **A published `RigExecRigPose` is always COMPLETE. Sparse evaluation changes
> which values are RECOMPUTED, never which values are PRESENT.**

The evaluator retains the previous generation's results for the epoch. A sparse
generation begins from the retained `jointFramesBase`, `jointFramesFinal`,
`jointMatricesFinal`, `controlFrames`, `providerXforms`, `providerBaseXforms`,
`solverFrames`, `movedProperties`, `weightFields` and `weightFrames`, and then
overwrites only what the closure recomputed.

"Begins from", not "begins by copying" — see Gate 3. A `std::map` copy costs
about what the inserts it replaces cost, so carry-forward that is implemented as
a copy makes the publish no cheaper and the whole exercise loses a third of its
win. The retained joint results live in a vector indexed by the epoch's fixed
joint ordinal; a sparse generation takes that vector as its starting point and
writes only the ordinals the closure touched.

So, specifically:

- **`jointFramesFinal`** — all 252 joints present every generation. A joint
  outside the closure carries a bit-identical value because it is *the same
  bytes*, not a recomputation that happened to agree. There is no state in
  which a consumer can read a stale frame as fresh, because there is no state
  in which a frame is absent or partial.

- **`movedProperties`** — same. `body_geo.points` is present every generation.
  When the skin did not re-run it is the same `VtValue` holding the same
  copy-on-write `VtArray`, so a consumer comparing handles correctly sees
  "unchanged". This is strictly *better* than today, where a re-run produces a
  fresh array with identical contents and defeats handle comparison.

- **`weightFields` / `weightFrames`** — same carry-forward. Both remain gated
  by `_publishWeightFields`; when publication is off they are empty in both
  paths, unchanged.

What is **not** carried forward is anything whose meaning is "what happened
this time":

- **`diagnostics`** — a diagnostic is a statement about this generation's work,
  and a skipped step did no work and has nothing to say. **This is a visible
  behaviour change and it needs saying out loud:** a rig that emits a standing
  diagnostic every frame today (a permanently `MoverFailed` mover, say) will
  emit it only on the frames its step actually runs. Parity therefore compares
  values, not diagnostics.

- **The work counters** — `moverGraphRevisionsExecuted`, `solverEvaluations`,
  `solverOverrideRounds`, `moverGraphSchedulesBuilt`. These describe work done,
  so they legitimately differ between the two paths, the same way the baked
  counters do. Excluded from parity.

**A consumer holding a pose across generations** sees no change. `RigExecRigPose`
is a value type returned by copy, so a held pose is already a private snapshot
and nothing the evaluator does later can reach into it. That stays true when the
joint results become a shared vector: the sharing is `shared_ptr<const>`, so a
held pose either owns its own copy or shares an immutable one, and in neither
case can a later generation mutate what a consumer is already holding. If the
implementation ever finds itself wanting to mutate a published pose in place,
that is the point to stop and come back to this note.

The three consumers to hold this to account, because they are the ones that
would show a stale frame if it were possible: the imaging bridge
(`libs/rigExecImaging/bridge.h`, which publishes to Hydra), the Python
`pose.joint_frame` / `moved_property` bindings, and `tools/rigExecPose.py`.

## Ordering — nothing is re-ordered

This codebase has been bitten by ordering repeatedly, so each rule gets its own
line and its own answer.

1. **A mover chain applies in reverse add order.** Untouched.
   `_graphChains[target]` is already in execution order and the sparse path
   walks the same vector with the same index. **The one genuinely dangerous
   line in the whole change:** `revisionIndex++` must advance even for a
   skipped revision, or the chain splices onto the wrong graph node and every
   downstream mover is applied to the wrong input. This gets a dedicated
   behavioural test, not an assertion.

2. **`_GetMoverExecutionOrder` walks the Movers namespace bottom-up.**
   Untouched. It runs at Compile and the sparse map is built *from* its output,
   never instead of it.

3. **A joint's write propagates to descendants.** Not merely preserved — it
   becomes an edge in the closure, and it must be generated by the same
   namespace walk `commitConstraintFrames` uses. Note that the real walk
   *narrows* this set (`nearestBlocking` stops propagation at an independently
   solved joint); per the invariant at the top, the map does not model that
   narrowing, because over-marking is safe and under-marking is not.

4. **Constraints down a chain are added tip-first.** Untouched. That is an
   authoring-time rule about the contents of `_frameConstraints`, and
   `_poseSteps` indexes into it. The sparse walk never reorders `_poseSteps` —
   it only skips entries.

5. **`_chainLevels` are contiguous runs of `_chainOrder`.** Untouched. Skipping
   a chain never merges or splits a level; a level whose chains are all clean is
   skipped whole, and a level with one dirty chain runs in compiled order.

6. **Gate 1's replay re-orders nothing, and that is why it is replay rather
   than skip.** A clean constraint writes its recorded outputs at exactly the
   point in `_poseSteps` where it would have solved them, so every later step
   reads what it would have read. The two tempting alternatives both DO
   re-order, which is why neither is used: stepping over the constraint leaves
   its targets at their base frames, changing what every later step sees; and
   seeding `finalFrames` from the previous generation's final values lets an
   early constraint read a value that had not been written yet.

If any future revision of this design re-orders anything, that is a different
design and needs a different note.

## Mode and parity

`RigExecEvaluationMode` gains `Sparse` and `SparseWithParityCheck`, in the
shape the baked work established.

- **`Sparse` is a REQUEST.** If the epoch has no map — structure dirty, compile
  failed, the first frame after a structural edit — `Evaluate` falls back to
  the dynamic path. Setting the mode can never change an answer, only how fast
  it arrives.

- **`SparseWithParityCheck`** runs both and compares with exact equality: key
  sets *and* values for every published map. `Pose.sparseParityMismatches`,
  zero in every other mode including `Sparse` — nothing compared is not the
  same fact as nothing differed, so it is read beside `GetEvaluationMode`.

- **Order within the parity generation matters.** The sparse generation runs
  **first**, then the dynamic one. Running dynamic first would leave every
  cache warm and validate the sparse path against a state it never sees in
  production. (This needs checking against how `BakedWithParityCheck`
  sequences its two generations — if it does the opposite, one of us is wrong
  and it is worth knowing which.)

- **`Sparse` and `Baked` do not compose** in the first cut. If a program exists
  and the mode is `Sparse`, the sparse path wins and the program is unused.
  Stated rather than left to be discovered.

## Two ways these tests failed for people who did not write them

Both are fixed. Both were the same underlying mistake: verifying in the
author's environment and assuming that generalised.

**1. The bootstrap.** Both sparse tests shipped calling
`rigexec_test_env.SetupPluginTest()`, which is the bootstrap for the
*usdview-plugin* tests and deliberately does not put `<build>/python` on
`sys.path`. They passed for the author, whose harness exports `PYTHONPATH`, and
failed with `ModuleNotFoundError: No module named 'rigexec'` for everyone else.
The suite's answer is `from test_rigexec_python import _setup_environment`,
which resolves the build tree from the resources argument `add_test` already
passes; every other Python test uses it and these two now do. Verified with
`PYTHONPATH`, `PXR_PLUGINPATH_NAME` and `RIGEXEC_USD_INSTALL` all unset:
**67/67**.

**2. A stale build, reported as a test failure.** A later run showed 66/67 with
`testRigExecSparseConstraints` failing. Reproduced by running the test against
the other build directories: `AttributeError: '_rigexec.Pose' object has no
attribute 'sparse_solver_batches_reused'`. That binding landed at 19:19;
`build-psd` was configured at 19:05 and `build` at 19:13. Stale code, not a
flake and not an order dependency — the test was run 8 times in a current build
without failing, and its assertions count WORK (`solver_evaluations`,
`sparse_solver_batches_reused`) rather than measure time, so a loaded machine
cannot move them.

The bug worth fixing was the *message*. An `AttributeError` pointing into the
test says nothing about what to do, and cost real time to diagnose. Both files
now call `_RequireEngine([...])` up front, which names the missing symbol, says
to rebuild, and prints the module actually in use:

    this build's _rigexec is older than this test: Pose is missing
    sparse_solver_batches_reused. Rebuild (cmake --build <builddir>) and
    re-run; the module in use is .../build-psd/python/rigexec/__init__.py

The last clause matters at least as much as the first: a `.pyd` from one build
directory will happily load a `rigExec.dll` from another if that one is first
on `PATH`, and that mismatch cost half an hour earlier in this work (see
"A phantom, and the lesson it cost").

## Correctness gates

Nothing ships that is not bit-identical.

- `tools/rigExecPose.py --joints` over `examples/biped/Biped_anim.usda` frames
  1–8, sparse against dynamic, byte-for-byte.
- `SparseWithParityCheck` reporting zero mismatches over the same frames and
  over a synthetic drag on each of the ten `r1` probe controls.
- Every behavioural verifier still passing: `verify_rig.py`, `verify_girdle.py`,
  `verify_spine.py`, `verify_twist.py`, `verify_hand.py`,
  `verify_foot_fingers.py`, `verify_layers.py`.
- The C++ suite green. Baseline confirmed 64/64 in `build-sparse` on
  2026-09-13 after the `avars:footRoll` fix to `testRigExecBakedMode.cpp`.
- `r2_flat_cost.py` re-run and pasted into this note, so the claim that the
  cost stops being flat is a measurement and not a hope.

Three of those deserve to be specific rather than generic:

- **The `revisionIndex` test.** Ordering rule 1 below identifies one line whose
  failure mode is silent and catastrophic. The gate is behavioural, not an
  assertion: a rig with a chain of three or more movers on one target, drag
  something that dirties only the last of them, and compare the resulting
  points against the dynamic path. If the index failed to advance past a
  skipped revision, the chain splices onto the wrong node and the points are
  wrong in a way no assertion inside the walk would notice.

- **The finger-root case.** `index_001_l_bind_fk` and its nine siblings are
  overwritten by the `hand_?_follow` parent constraints, so their own avars
  move nothing an animator can see and their downstream joints move by ~1e-13.
  That is the sharpest available test of "structural, never value-diffed": a
  drag on one of them must produce *bit-identical* output to the dynamic path,
  1e-13 and all. An implementation that quietly rounded it to "clean" would
  pass every other gate here.

- **Which engine answered.** Both spikes print the loaded module and `PATH[0]`
  on every run, because a `_rigexec` `.pyd` built in one build directory will
  load a `rigExec.dll` out of whichever directory is first on `PATH`, and the
  mismatched pair links silently. That cost this investigation half an hour of
  measurements against another build's engine, at 2.5x the true cost, with no
  error anywhere. Any timing claim made about this work should say which
  binary produced it.

## Sequencing

Each gate lands behind the same mode with the same parity gate, so each is
independently shippable and independently revertible. Rough expectations:

Build order revised 2026-09-13: **Gate 3 first.** It needs no change to
`_EvaluateDynamic`'s pose walk, so it is not blocked on the sparse-blend work
landing in that file — and after the ABI-mismatch phantom above, staying out of
a file another agent is actively writing is worth more than the gate ordering.

| after | floor | fingertip | hips |
|---|---|---|---|
| today | 16.9 | 24.9 | 24.9 |
| Gate 3 (skin packet reuse) | ~15 | ~22 | ~24 |
| Gate 1 (constraint walk) | ~8 | ~15 | ~17 |
| Gate 2 (solver batches) | ~7 | ~9 | ~16 |
| Gate 5 (the publish) | ~6 | ~8 | ~15 |
| Gate 4 (snapshot — decide after re-measuring) | ~4 | ~6 | ~13 |

Gates are numbered by payoff; they are BUILT in the order of the table, which
differs in one place. Gate 5 (the publish) is only ~1 ms but it must land
before the honest-publication carry-forward is written, or the carry-forward
gets built as a map copy and has to be redone. Gate 4 is deliberately last
because the case for it depends on where the others leave the number.

The root drag barely improves, which is correct — it genuinely does reach the
whole rig, and a sparse path that "sped it up" would be doing less work than
the answer requires. The honest headline is therefore not "the rig got fast",
it is "the rig now costs what the edit costs".

## What I still need to measure

Stated as gaps rather than buried, because each could change a decision:

1. ~~The split of `AuthoritativeSnapshot` between exec compute and the
   copy/publish.~~ **Answered while writing this note**, without touching the
   evaluator: the scope brackets only `_taps->Evaluate`, and reading the
   trace's nesting showed the publish is a separate, unscoped 3.3–3.9 ms. That
   is what turned four gates into five and moved the publish from a footnote to
   the third-largest line.

2. **Whether a skipped constraint step leaves `_chainSnapshots` consistent**
   for a phased read. `recordFrame` writes a snapshot only where
   `_snapshotPoints` named that mover, so a skipped step writes nothing — and
   the question is whether the retained value is already there or whether the
   consumer sees a hole. Almost no rig uses phased reads, which is exactly why
   a break there would go unnoticed. This is a spike, before Gate 1 lands.

3. **Whether `refreshPoseProvider` has side effects a skipped step needs.** It
   populates `_connectedPoseCache`, and a later step that reads a connected
   pose may depend on an entry an earlier — now skipped — step used to fill.
   This is the most likely place for the first parity failure, and it is worth
   reading `refreshPoseProvider` end to end before writing Gate 1 rather than
   discovering it from a mismatch count.

4. ~~How `BakedWithParityCheck` sequences its two generations.~~ **Answered:**
   it runs baked FIRST, then dynamic, and publishes the dynamic reference so a
   disagreement never changes what consumers see (`rigEvaluator.cpp`, the
   `runBaked` block). That is the same convention this note proposes for
   sparse, so there is nothing to reconcile — and the "publish the reference,
   not the fast path" half is worth copying deliberately rather than
   rediscovering.
