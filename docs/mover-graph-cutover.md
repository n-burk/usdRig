# Mover graph cutover — COMPLETE (2026-07-27)

**Done. There is no compiler, no derived evaluation stage, and no generated
prim anywhere in the engine.** `moverCompiler.{h,cpp}` is deleted; `_evalStage`
collapsed into `_stage`; the evaluation stage IS the source stage.

All four passes are gone, each by a different mechanism, which is why this
took four separate steps rather than one:

| pass | was | now |
|---|---|---|
| 0 — solver→joint binding | authored `rigExec:frameSource`/`frameElement` | in-memory binding + `computePointFrame` value override |
| 1 — frame revisions (aim) | generated application prims | applied in memory from the mover's authored inputs |
| 1.5 — ribbon driver curve | authored `resolvedDriverPoints` + `restDriverPoints` | `RigExecPointsPacket` overrides on two ribbon computations |
| 2 / 3 — point chains, normals/extent | generated application prims | `RigExecMoverGraph` (VDF), with the CPU reference as parity oracle |

Pass 1.5 was last and needed a shape the others did not, because of two exec
constraints worth remembering:

- A `Relationship` accessor requests computations on its **targets** and
  cannot name an attribute of them (prim→attribute accessors are still
  `XXX:TODO` in `exec/computationBuilders.h`). `rigExec:driverCurve` targets
  the curve *prim* in all four authored sites, so the points are unreachable
  from the ribbon.
- An override of a `point3f[]` attribute is type-checked against the
  **element** type (`exec/system.cpp` `_ComputeWithOverrides`), so an array
  cannot be supplied as an override at all.

The fix is to box the array: `RigExecPointsPacket` is a registered exec type
holding one `std::vector<GfVec3f>`, hence a single value an override can
carry. The authoring surface did not change.

The rest of this note is retained as the record of how the cutover was
reasoned about and what it is verified by; the "remaining step" framing below
is historical.

## Why it is atomic

The derived stage carries three things, not one:

1. the generated application prims for **point chains** (Pass 2),
2. the generated application prims for **frame chains** — aim/pose movers on
   transform providers (Pass 1),
3. the solver→joint bindings, `rigExec:frameSource` / `rigExec:frameElement`
   (Pass 0).

(3) is the one that surprises. Deleting `_evalStage` removes the layer those
bindings live in, so every solver-driven joint silently falls back to the
namespace-parent pose. Point chains, frame chains, and joint binding therefore
have to move together, along with tap and Hydra publication, which read results
by generated-prim path today.

There is no intermediate state that both builds and computes correctly, and the
failure mode is wrong geometry rather than a compile error. Run the example
probes (below) as part of the change, not after it.

**Update — (3) is done.** Pass 0 is deleted, `rigExec:frameSource` /
`rigExec:frameElement` are gone from `RIGEXEC_REGISTER_XFORMABLE`, and nothing
authors the solver→joint binding anywhere. See "Joint binding, in memory"
below. What still holds is the rest of the atomicity — point chains, frame
chains, and publication move together.

## A fourth dependency, on the solver side (found 2026-07-27)

The three items above are not the whole of what the derived stage carries.
**Pass 1.5 authors onto `RigExecRibbon` solvers**, and the ribbon's registered
`computePointFrameArray` reads both properties as declared inputs
(`moverKernels.cpp` ~1289):

```
AttributeValue<GfVec3f>(rigExec:restDriverPoints)   // bind-time rest capture
Relationship(rigExec:resolvedDriverPoints)          // -> curve points computeValue
```

This does **not** move with the point and frame chains. Solvers are the
transform network and stay in exec, so deleting the derived stage strands a
ribbon solver's driver binding exactly the way it would have stranded the joint
binding. `05_TwistRibbonSpine.usda` and the ArmRig ribbon exercise it.

`resolvedDriverPoints` is only a prim→`.points` resolution of the user-authored
`rigExec:driverCurve`, so it may be expressible by registering against
`driverCurve` directly (verify relationship forwarding reaches the attribute).
`restDriverPoints` is genuinely compiled state — a bind-time capture — so it
needs a value override, the same mechanism the joint binding now uses.

The lattice mover's `rigExec:restCagePoints` is the same shape but sits on a
*mover*, so it does move with the point chains.

## The OpenExec constraint that shapes all of this

`pxr/exec/exec/computationBuilders.h` offers exactly these input accessors:
`Attribute`, `Relationship`, `Prim`, `Stage`, `NamespaceAncestor`,
`Computation`, `Metadata`, `Constant`. The header carries an explicit
`// XXX:TODO Property, NamespaceParent, NamespaceChildren, etc.`

**There is no reverse-relationship accessor.** A computation registered on a
joint can only reach its solver through a relationship authored *on the joint*.
The solver's `rigExec:joints` points the wrong way and exec cannot traverse it
backwards. This is why Pass 0 existed at all, and it is a property of OpenExec
26.08, not a design choice in RigExec — so "just register it differently" is not
available. Either something is authored on the joint, or the joint's frame is
supplied from outside exec.

The supported way to supply it from outside is
`ExecUsdSystem::ComputeWithOverrides(request, ExecUsdValueOverrideVector&&)`,
which is public (`pxr/exec/execUsd/system.h`). Overriding a joint's
`computePointFrame` makes exec's own `NamespaceAncestor` fallback see the
overridden value, so descendant joints inherit correctly with nothing authored.
That is the mechanism for finishing Pass 0 removal; it costs a second evaluation
pass (solver aggregates first, then the overridden request).

## Joint binding, in memory (done)

`RigExecRigEvaluator` now holds `_jointSolverBinding` (joint → solver, element)
and `_jointSolverArrayTaps` (the posing solvers' `computePointFrameArray`, in
the authoritative request). `Compile()` builds the map from the Phase A
validation, which already resolved and bounds-checked exactly that pair and
threw it away. `Evaluate()` indexes the aggregate directly through
`RigExecExtractElementFrame`.

The extraction math is shared, not copied: `libs/rigExec/frameExtraction.h`
holds `RigExecFrameFromMatrix`, `RigExecElementOutSpace`, and
`RigExecExtractElementFrame`, and `computations.cpp` now calls into it, so the
exec path and the evaluator path cannot drift.

`Evaluate()` then runs two passes. Pass A evaluates `_solverFrameTaps` — its own
request holding just the joint-posing solvers' `computePointFrameArray`. Pass B
evaluates the authoritative request through
`RigExecTapSet::Evaluate(time, overrides)`, with each bound joint's
`computePointFrame` overridden by its extracted element.

> This originally read "solvers read controls, never joints, so the two requests
> cannot cycle." **That was false** and it caused a real bug — see
> "Solver→joint overrides must iterate" below. Pass A is now iterated to a fixed
> point, and solver acyclicity is enforced at compile rather than assumed here.

The override is what made deleting Pass 0 possible rather than just deferring
it. Three consumers needed the bound joint's frame and none of them could be
fixed by reading it evaluator-side, because they live inside exec:
`computeMatrix` (pairs with `computePointFrame`), a frame-chain application on a
bound joint, and the `NamespaceAncestor` fallback an unbound descendant follows.
Overriding the value they all read fixes all three at once.

Verified live twice, not just green. With Pass 0 still authoring, inflating the
element index turned `testRigExecArm` red at four assertions including
`TestShotAnimation`'s 1e-9 wrist check — proving the in-memory extraction
reproduced the authored path exactly. After Pass 0 was deleted, the same
perturbation turns **16** assertions red across `testRigExecArm` and
`testRigExecImaging`, including deformed geometry (lines 595–600) and Hydra
publication — proving the override now drives joint frames, matrices,
deformation, and imaging.

Related cleanup that fell out: the compile-time rejection of a legacy authored
`rigExec:frameSource` is gone. Nothing reads that name now — not a schema
property, not a registered input — so a leftover opinion is inert and failing
the compile over it would reject a rig that evaluates correctly.

The measurement that framed the risk: across all ten example stages there are 24
joints, 19 solver-bound, 5 unbound, and **0 that inherit from a bound
ancestor**. No example exercises the inheritance path, so it would not have been
caught by the suite — the override handles it structurally instead of leaving it
to chance.

## What already exists

| Piece | Where | Status |
|---|---|---|
| All 10 revision ops as `VdfNode`s | `moverGraph.cpp` `_RevisionNode` | tested |
| Chain construction / evaluation | `RigExecMoverGraph` | tested |
| `rigExec:resolved*` → path resolution | `RigExecResolveRevisionBinding` | tested, all 8 bindings |
| Schema type → frozen op | `RigExecRevisionOpForSchema` | tested |
| Packet assembly, matrix | `RigExecAssembleMatrixParameters` | tested |
| Packet assembly, other 9 ops | `RigExecAssembleParameters` | tested |
| Status derivation | `RigExecStatusForParameters` | tested |

`TestAssembleAndEvaluateWithoutDerivedStage` runs resolve → assemble → build →
evaluate and asserts no `__RigExecGenerated` prim and no `resolved*`
relationship exist afterwards.

## Point chains: done, and publishing

`RigExecRigEvaluator` builds `_graphChains` in `Compile()` — one
`_GraphRevision` per (mover, target) carrying its resolved
`RigExecRevisionBinding`, its op, and taps for `computeMatrix` /
`computeWeightPacket` / `computeBlendChannel` / `computePointFrameArray` — and
`Evaluate()` builds a `RigExecMoverGraph` per generation, evaluates it, and
**publishes** it into `movedProperties`. The generated-prim result is still
computed into a local `loweredProperties` as the parity reference, and is
deleted together with the Pass 2 chains.

The harness earned its keep immediately: it caught a real disagreement on
first run rather than letting one ship. The cause was mine —
`RigExecProviderValues` is only filled with `transform` and `weights`, so a
blend revision got no deltas. Chains are therefore compared only when every
revision is `Matrix`; the rest are counted as **deferred** in the parity
diagnostic, never silently skipped.

**All point-chain provider values are supplied**, and every authored-mover chain
in the examples agrees:

| value | source |
|---|---|
| `transform` | `computeMatrix` tap on the resolved provider |
| `weights` | `computeWeightPacket` tap |
| `blendDeltas` | `computeBlendChannel` tap per input, summed |
| `driverFrames` | `computePointFrameArray` tap |
| `basePoints` | authored base off the stage |

Everything else (topology, cage, bind coords, strength, divisions) is a static
read through the binding.

`RigExecSumBlendChannels` lives in `moverGraph.cpp` and is called by **both** the
evaluator and `_BuildBlendMoverParameters` — one definition, so the graph and
kernel paths cannot drift. Same discipline as `frameExtraction.h`.

**Proven end to end**, agreement asserted (not deferral); "0 chain(s) agreed"
fails explicitly:

| rig | chain | revisions | test |
|---|---|---|---|
| `01_FkChainTail` | pure matrix | 4 | `TestMoverGraphParity` |
| `04_BlendShapeFace` | pure blend | 1 | `TestMoverGraphParity` |
| `ArmRig` | volumeCorrect → curve → 3× matrix → blend | 7 | `TestGeometryMovers` |

Negative-controlled: perturbing the graph's source points turns the first two
red; perturbing `basePoints` turns ArmBody red at element 1.

*Note for whoever writes the next control:* perturbing **element 0** of
`basePoints` on ArmRig proves nothing — ArmRig's blend mask is `[0,1,1,0]`, so
element 0 is zeroed and the suite stays green. Perturb every element, or pick an
unmasked one.

**Pass 3 (derived maintenance) is in the graph.** `_graphDerivedChains` keys
the synthesized `RecomputeNormals` / `RecomputeExtent` revisions by the points
target that feeds them, and they are evaluated right after that chain so its
final points are available as `basePoints`. Keying them this way is what models
the cross-chain dependency `_graphChains` alone cannot.

That pass caught its own bug on first run: `RibbonGuides.extent` mismatched
because `RigExecAssembleParameters` never set `params.widths`, so a
BasisCurves extent came out under-reported. `RigExecRevisionBinding` gained a
`widths` path and assemble now reads it.

**Publication comes from the compiled graph.** `pose.movedProperties` is
written from `RigExecMoverGraph::Evaluate` for both points and derived chains.
The generated-prim results are still computed into a local `loweredProperties`
and compared, so a disagreement is reported rather than shipped; that reference
and the Pass 2 lowering go away together.

Negative-controlled at the publication boundary: perturbing the published graph
value fails the geometry assertions, the CPU-parity comparison, and the guide
positions — so publication demonstrably flows from the graph, not the taps.

## Assemble-vs-kernel drift: a regression that shipped, and the audit after it

`RigExecAssembleParameters` is a hand-written peer of the
`_Build*MoverParameters` kernels. **They drifted, and one of the drifts shipped
as visibly broken geometry.**

`07_SurfaceDrape` rendered undeformed in usdview. `_BuildSurfaceMoverParameters`
hardcodes `strength = 1.0f` ("v0.1 attach/project maps fully"); assemble did
`_Float(moverPrim, "inputs:strength", 0.5f)` — and `RigExecSurfaceMover`
declares no such attribute, so the graph silently took the 0.5 default and
projected half way (base y=0.8 → graph 0.4 vs lowered 0.0).

**Why the suite missed it:** a parity mismatch is a *diagnostic string on the
pose*, not a test failure, and `TestMoverGraphParity` only covered 3 of 9 rigs.
Publication switched to the graph while 4 rigs were unverified. The test now
covers **every** example, discovering the rig prim by type so a new example
cannot be silently skipped.

Full audit of the remaining ops (Matrix and VolumeCorrect were already
identical), all now fixed:

| op | drift | visible? |
|---|---|---|
| SurfaceProject | strength 0.5 vs kernel's fixed 1.0 | **yes — shipped broken** |
| Lattice | `auxPoints`/`auxPointsB` **swapped** vs the kernel | no — rest==live at rest |
| Lattice | validity ignored divisions>=2 and cage cardinality | no |
| BlendShape | accepted an invalid weight packet the kernel rejects | no |
| RecomputeNormals | validity ignored missing topology | no |
| Smooth / VolumeCorrect | non-finite strength not rejected | no |
| **all static reads** | `_Array`/`_Float`/`_Enabled` read at `Default()` | no |

The last one is the systemic one: the kernels read the same inputs through exec
`computeValue` **at the evaluated time**, so any animated cage, surface,
topology, strength, or enable diverged. `RigExecAssembleParameters` now takes a
`UsdTimeCode`.

The lattice swap and the time bug are both invisible at default time — the two
cages are equal at rest and nothing is animated. `TestMoverGraphParity` now also
runs `06_LatticeBulge` at **1024** (its cage bulges) and `ArmShotAnim` at
**1010**. Adding those cases is what turned both bugs red; the default-time
suite could not have caught either.

Parity is now machine-checked: `RigExecRigPose::moverGraphParityMismatches` must
be 0 **and** `moverGraphParityAgreements` must be > 0 per rig. Asserting only
"no mismatch" would pass a rig whose chains were never compared — which is
exactly how the SurfaceProject regression shipped. Grepping diagnostic text was
the weak link; counts cannot be misread the same way.

## Solver→joint overrides must iterate (found in review, 2026-07-27)

The two-pass override flow was unsound as first written. The code claimed
"solvers read controls, never joints" — false. `RigExecTwistDistribution` takes
`rigExec:start` / `rigExec:end` as `computePointFrame` inputs, and those
endpoints are routinely joints another solver poses. In `05_TwistRibbonSpine`,
`SpineFK` poses `Root` and `Chest`, `SpineTwist` reads both and poses
`TwistMid`. Computing the aggregates once with no overrides posed `TwistMid`
from its endpoints' *unposed* fallback.

**Parity is structurally blind to this** — the graph and the lowered path
consume the same override, so both are wrong together and agree perfectly. Only
an independent check could find it.

`Evaluate` now iterates the overrides to a fixed point, feeding each round's
result back into the solver-aggregate request. Measured, not assumed:
`solverOverrideRounds` / `solverOverridesConverged` on the pose. Every rig
settles in 1 round except `05_TwistRibbonSpine @1024`, which needs **2** — and
the test asserts `>= 2` there, so the refinement cannot be silently dropped. At
default time the override equals the fallback, so only an animated case exposes
it.

Two follow-on corrections from the same review:

- **Unique joint ownership does not imply an acyclic solver graph.** Two solvers
  can each uniquely pose their own joints while reading each other's. `Compile`
  now builds the solver→solver dependency graph and rejects cycles with the
  offending path. The `N + 1` iteration bound is sound only because of it.

  Edges come from two sources, and both are needed. **Indirect:** a relationship
  targeting a joint that another solver poses (`RigExecTwistDistribution`'s
  `start`/`end`, `RigExecRibbon`'s `startFrame`/`endFrame`/`twistFrames`).
  **Direct:** a relationship targeting another solver — `RigExecBlendPointFrames`
  takes `inputA`/`inputB` as solver paths, so a cycle can run entirely through
  solver→solver edges without touching a joint, and deriving only the joint
  edges would miss it. Neither uses a per-solver-type table, so a new solver
  type cannot escape the check.

  Both rules have their own regression, because one test cannot cover both:

  - `TestSolverCycleRejected` — the **indirect** path. 05 compiles unmodified;
    pointing `SpineFK`'s controls at `TwistMid` (which `SpineTwist` poses, while
    `SpineTwist` already reads `SpineFK`'s joints) closes a loop `Compile`
    rejects by name.
  - `TestPureSolverToSolverCycleRejected` — the **direct** path, with no joint
    in it. 03 already has `IKFKBlend.inputA = ArmFK`; adding
    `ArmFK.controls += IKFKBlend` closes the loop. In 03 only `IKFKBlend`
    carries `rigExec:joints`, so neither edge can come from the indirect rule —
    the test isolates the direct rule rather than passing for the wrong reason.

  `03_IkFkBlendClamp` unmodified — whose blend legitimately reads two solvers —
  still compiles, so the check is not simply rejecting everything.
- **Non-convergence must not publish.** It previously set the flag and carried
  on with the last iterate, reaching `pose.valid = true`. It now returns an
  invalid pose: an unevaluated generation is recoverable, a plausible wrong one
  is not.

Related: `rigExec:mode`, `transformReadPhase`, `cageReadPhase`, and
`surfaceReadPhase` are now rejected at compile if time-sampled. `uniform` is a
convention, not an enforcement — USD permits samples on a uniform attribute, and
these tokens select the compiled operation, so a sampled one could change the op
under a live epoch without moving the binding-epoch digest.

**Next, in order:**

1. Frame chains (Pass 1) — needs a `RigExecPointFrame`-valued node; kernel is
   `_EvaluatePointFrameExpression`. These feed `_frameChainHeads`, which
   resolve joint final frames/matrices and the `final` transform read phase —
   none of which flow through `movedProperties`, which is why publication could
   switch without them.
2. Delete the Pass 2 lowering, `loweredProperties`, `_pointsChainTaps`, and the
   parity comparison together, once frame chains no longer need the generated
   prims.
3. Then `_evalStage` / `_derivedSessionLayer` / `_generatedLayer`,
   `moverCompiler.{h,cpp}`, and the ten application-host schema classes.

## The oracle no longer depends on authoring (2026-07-27)

Direction: **the compiler must not author USD data.** That makes
`RigExecCompileMoverChains` the last violation — it writes generated prims into
a derived layer purely so they can be evaluated.

The obstacle was never the evaluation path (the graph already publishes); it was
that those generated prims were the *parity oracle*, the thing that caught the
SurfaceProject strength bug. Deleting them would have traded a working
regression net for a cleaner schema list.

`_EvaluateChain` — the scalar CPU reference — replaces them. It resolves every
input off the authored stage and authors nothing, and critically it does **not**
call `RigExecAssembleParameters`, so it is a genuinely independent
implementation rather than a second caller of the same packet assembly. A
reference that shared the assembler would have agreed with the SurfaceProject
bug instead of catching it.

It covered 5 of the 7 point-chain ops; `Lattice` and `SurfaceProject` are now
added. `Evaluate` compares the graph against it on every generation and folds
disagreements into `moverGraphParityMismatches`. Negative-controlled: perturbing
the reference turns every rig red.

**All three implementations currently agree** — graph, generated prims, and CPU
reference — across all 9 examples plus the animated cases.

### The two "bind-time captures" are just Default-time reads

`rigExec:restCagePoints` (lattice mover) and `rigExec:restDriverPoints` (ribbon
solver) look like compiled state that would need a value override. They are not:
`moverCompiler` captured both with `a.Get(&value, UsdTimeCode::Default())`, so
reading the cage / driver curve at Default time directly is *identical*. The CPU
reference already does exactly that for the lattice.

That removes the override requirement for the lattice mover entirely. The ribbon
solver still needs one, because it consumes the value through a registered exec
input (`AttributeValue<GfVec3f>(restDriverPointsAttr)`) and exec evaluates at the
current time — so the Default-time array has to be supplied as an override on
that attribute's `computeValue`.

### What still blocks deleting the compiler

1. **Pass 1, frame chains.** Still the live path for aim constraints;
   `_frameChainHeads` resolves joint final frames/matrices and the `final`
   transform read phase. Needs a `RigExecPointFrame`-valued node in the graph
   (kernel: `_EvaluatePointFrameExpression`).
2. **Pass 1.5, ribbon solver.** `resolvedDriverPoints` is a prim→`.points`
   resolution of the authored `rigExec:driverCurve`; `restDriverPoints` needs
   the Default-time override above.
3. Then Pass 2/3 authoring, `loweredProperties`, `_pointsChainTaps`, the
   graph-vs-lowered comparison, `moverCompiler.{h,cpp}`, `_evalStage` /
   `_derivedSessionLayer` / `_generatedLayer`, and the ten application-host
   schema classes all go together — with the CPU reference left as the oracle.

## How to do the rest incrementally

Both remaining items looked atomic and one of them was not — the joint binding
came out cleanly once its verification strategy was clear. Use the same one:

1. Build the graph in `Compile()` **alongside** the existing lowering, changing
   no publication.
2. In `Evaluate()`, evaluate both and compare, reporting disagreement as a
   diagnostic. Green suites then mean the graph reproduces the lowered result
   exactly, while the lowered result is still what ships.
3. Only then switch `movedProperties` to the graph and delete the lowering.

At each step, prove the new path is actually *live* before trusting a green
run — perturb it deliberately and confirm the suites go red. Two green suites
and a silent no-op are indistinguishable otherwise, which is how the
`VdfReadWriteIterator` trap below hid.

## Steps

1. ~~**Joint binding off the derived layer.**~~ **Done.** See "Joint binding,
   in memory" above. Pass 0 is deleted, the two properties are gone from the
   registration, and the binding lives in `_jointSolverBinding` + a value
   override. The derived stage no longer carries item (3).

2. **Frame chains into the graph.** Pass 1's aim/pose revisions need a
   `RigExecPointFrame`-valued node alongside the `GfVec3f` one. Same connector
   shape (READWRITE `previous` → `out`); the kernel is
   `_EvaluatePointFrameExpression` in `moverKernels.cpp`.

3. **Point chains into the graph.** In `Compile()`, for each target in
   `pointsChainHeads` order, `AddPointSource` then one `AddRevision` per mover
   in the existing post-order `_movers` walk. Bindings come from
   `RigExecResolveRevisionBinding`, ops from `RigExecRevisionOpForSchema`.

4. **Per-frame packets.** In `Evaluate()`, tap the dynamic providers named by
   each binding (`computeMatrix`, `computeWeightPacket`, blend channels,
   `computePointFrameArray`) on the **authored** stage, fill
   `RigExecProviderValues`, call `RigExecAssembleParameters`, and feed the
   graph. Note the packets are per-generation constants, so they are rebuilt per
   evaluate, not per node.

5. **Publication.** `movedProperties` comes from `RigExecMoverGraph::Evaluate`
   on each target's final head. Spec §9's "canonical public tap addresses with
   private generated-path resolutions" already models the indirection — only the
   private side changes, from a generated prim path to a `VdfMaskedOutput`.

6. **Delete.** `_evalStage`, `_derivedSessionLayer`, `_generatedLayer`,
   `moverCompiler.{h,cpp}`, the `__RigExecGenerated` scope, and the ten
   generated-application schema classes (regenerate the schema plugin). Then
   `_stage` is the only stage and all taps run against it.

## Verification that must pass

```
cmake --build build && ctest --test-dir build     # 4 suites
bin/run_probe.bat <scratch>/verify_rigs.py        # 9 examples, joint counts
bin/run_probe.bat <scratch>/verify_variants.py    # 3 ArmRig variants + shot
bin/run_probe.bat <scratch>/verify_surface.py     # authoring surface stays clean
bin/run_probe.bat <scratch>/verify_edges.py       # no-joint rig, stale manifests
bin/run_probe.bat <scratch>/verify_no_joint_authoring.py  # nothing leaked on stage
```

Sharpest checks, in order of what they would catch:

- `TestShotAnimation` — wrist joint frame equals the IK solver's end frame to
  1e-9. Only holds if the solver→joint override survived.
- `TestMoverGraphParity` — all 9 examples plus `06@1024`, `ArmShotAnim@1010`,
  `05@1024`. Asserts `moverGraphParityMismatches == 0` **and**
  `moverGraphParityAgreements > 0`, so "checked nothing" cannot pass as
  "everything agreed". Also asserts `05@1024` needs >= 2 override refinement
  rounds, and that 07 / 06@1024 actually leave their authored base.
- `TestSolverCycleRejected` / `TestPureSolverToSolverCycleRejected` — the
  indirect and direct halves of the solver-cycle rule.

Run the animated cases, not just the rest pose. Two whole classes of bug are
invisible at default time: operands that are equal at rest (the lattice rest
cage vs the live cage) and reads that ignore the evaluation time.

## Two traps already paid for

- A revision uses a READWRITE connector, so it writes through
  `VdfReadWriteIterator(ctx, previous)` and must **not** `Allocate`. Allocating
  fails with "output cannot hold a boxed value" and silently degrades to
  pass-through — correct-looking geometry that never moved.
- Anything talking to VDF directly must force `ExecTypeRegistry::GetInstance()`,
  or the executor cannot distinguish registered value types and reads one
  packet as another.
