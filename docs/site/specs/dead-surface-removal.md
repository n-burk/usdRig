# Dead surface removal plan

> **Status 2026-07-27:** steps 1-3 are DONE and verified (build, 4 ctest
> suites, 5 probe suites, `probeCodingError` CLEAN on all phases, usdview
> opened on `08_AimEyes`). Removed: 17 dead private tokens,
> `RigExecPartitionAPI`, `RigExecTapAPI`, `RigExecPointTransformAPI` with its
> registration block, `_ReconstructProviderFrame`, `_ComputeRestFrame`,
> `_ComputeProviderMatrix`, `_ReadLandmarks`, 8 landmark tokens, and the
> applied-API arm of the `frameProviderTypes` whitelist. `_IdentityLandmarks`
> was kept, as planned -- TwoBoneIk and TwistDistribution share it.
>
> **Step 4 (transform publication) is DONE.** `RigExecRigPose::providerXforms`
> carries the revised transform of any constraint-driven `UsdGeomXformable`;
> `RigExecImagingBridge::_FillProviderXforms` writes the previously-dead
> `hasXform`/`xform` channel. `10_AimXformTurret.usda` demonstrates it and
> `TestAimConstraintDrivesXform` asserts the published z-axis actually aims
> (direction, not just presence) and that the parented geometry is absent from
> `movedProperties` -- i.e. carried by hierarchy, not deformed.
>
> Two things learned doing it: a plain `UsdGeomXformable` has no
> `computeRestFrame`, and requesting one is a hard exec failure rather than a
> missing value, so rest taps are now gated to providers that publish one. And
> `08_AimEyes` was left as-is rather than converted -- it is a valid *skinning*
> demonstration with a passing semantic test; 10 is the rigid counterpart.
>
> **Correction 2026-07-27.** The claim above that parented geometry "inherits
> it through Hydra's flatten" was WRONG, and shipped a broken turret: the
> driven Xform animated in usdview's scene index debugger while the viewport
> stayed static. See `docs/hydra-flattening-position.md`. RigExec is installed
> *downstream* of the chain's only `HdFlatteningSceneIndex`, so it must publish
> world-space transforms and dirty the driven subtree itself. Both are now done
> in `RigExecResultsSceneIndex` (`_ComputeDrivenXform`, `_DirtySubtree`), and
> `tests/probeImagingPipeline.cpp` asserts it against the real
> `UsdImagingCreateSceneIndices` chain rather than a hand-built one.
>
> **Step 5 is DONE (2026-07-27). The engine no longer authors anything.**
> `moverCompiler.{h,cpp}` is deleted, along with the derived evaluation
> stage, its anonymous session sublayer, and the generated layer. `_evalStage`
> collapsed into `_stage` — the evaluation stage now IS the source stage.
> `TestArmRig` asserts the inverse of what it used to: no generated scope on
> either stage, and `GetEvaluationStage() == stage`.
>
> The last pass to move was the ribbon's driver-curve resolution, and it
> needed a shape nobody had used before. Two exec constraints ruled out the
> obvious routes: a `Relationship` accessor requests computations on its
> TARGETS and cannot name an attribute of them (so the curve's points are
> unreachable from the ribbon prim, and `rigExec:driverCurve` targets a prim
> in all four authored sites), and an override of a `point3f[]` attribute is
> type-checked against the ELEMENT type (`exec/system.cpp`
> `_ComputeWithOverrides`), so an array cannot be handed over that way.
>
> Resolution: box the array. `RigExecPointsPacket` is a registered exec type
> holding one `std::vector<GfVec3f>`, so it is a single value an override can
> carry. `RigExecRibbon` publishes `rigExec:computeDriverPoints` and
> `rigExec:computeRestDriverPoints`, whose callbacks return empty and which
> the evaluator overrides with the resolved live and Default-time values.
> The authoring surface is untouched — `rigExec:driverCurve` still targets
> the curve prim, as authored.
>
> `probeCodingError`'s phase C was repointed from "compile (no movers)",
> which no longer exists, to a ribbon compile+evaluate on
> `05_TwistRibbonSpine` — the one solver whose inputs arrive entirely as
> overrides, so a bad override key surfaces as a coding error rather than
> as quietly empty geometry.

Inventory and ordered plan for deleting declared-but-unconsumed surface in
RigExec. Every claim below was measured, not assumed — the commands are given
so each can be re-checked before acting on it.

Context: three rounds of deletion this session (the ten application host
schemas, `rigExec:frameSource`/`frameElement`, `rigExec:restCagePoints`) all
turned out to be the same shape — surface left behind by a design change that
nothing removed afterwards. This sweep looks for the rest of it in one pass so
it stops being discovered one accident at a time.

## How each item was measured

```
# Applied API schemas: used by any example? read by any code?
grep -l <name> examples/*.usda | wc -l
grep -rn <name> libs/ --include=*.cpp --include=*.h | wc -l

# Schema properties with no reader
for p in <every rigExec:* in schema.usda>; do
    grep -rn "\"$p\"" libs/ --include=*.cpp --include=*.h | wc -l
done

# Private tokens declared but never dereferenced
for t in <every token in TF_DEFINE_PRIVATE_TOKENS>; do
    grep -c "_tokens->$t" <file>
done
```

## What is dead

### 1. `RigExecPointTransformAPI` — the pre-alignment transform model

**Zero examples apply it. Its properties are read only by its own registration
block** (`computations.cpp` ~245-251), which nothing reaches.

It is the landmark-based transform model (`restPoints`, `posePoints`,
`framePolicy`, `twist`, `parent`, aim/up/reflection axes) that
`RigExecXformable` replaced — `matrix4d rest:space`/`posed:space` plus scalar
avars, documented in the schema as *"mirroring OpenExec's IrXformable contract
exactly (user-directed alignment, 2026-07-25)"*. The alignment superseded the
landmark model; the landmark model was never removed.

`08_AimEyes` is the proof it is inert: its eye joints apply no API at all, and
`TestAimConstraintRevisesJointFrame` asserts the constraint aims within 0.001.

It also would *conflict* if anyone applied it: it registers
`computeRestFrame`/`computeMatrix`, which `RIGEXEC_REGISTER_XFORMABLE` already
registers for `RigExecJoint`/`RigExecControl`.

**Remove:** the class, its registration block, `_ComputeRestFrame`,
`_ComputeProviderMatrix`, `_ReadLandmarks`, the `restPoints` / `posePoints` /
`framePolicy` / `zeroSidePolicy` / `twist` / `parentRel` tokens, and the
`PointTransformAPI` arm of the `frameProviderTypes` whitelist in
`rigEvaluator.cpp`.

**Keep `_IdentityLandmarks`.** Checked: it is shared with `RigExecTwoBoneIk`
(`computations.cpp` 591, 598) and `RigExecTwistDistribution` (751, 753), which
use it as the rest fallback when a provider has no rest frame. Only
`_ReadLandmarks` is exclusive to this block (142, 147, 201, 219).

**Bonus, found while checking the above:** `_ReconstructProviderFrame`
(`computations.cpp` ~138) is defined and **never registered** — one reference in
the file, its own definition. It is the frame-reconstruction body for the
landmark model, already dead independently of whether the API class goes.
Delete it with this item; it is what drags in `framePolicy` / `zeroSidePolicy`
/ `posePoints` and `RigExecReconstructFrame`.

### 2. `RigExecPartitionAPI` and `RigExecTapAPI` — never applied, never read

Zero examples, zero code references. `RigExecTapAPI` describes a tap
alias/role feature the tap set does not consult; `RigExecPartitionAPI`
duplicates `RigExecRoot.rigExec:partition`, which every example does author.

**Remove:** both classes.

### 3. `RigExecControlAPI` and `RigExecMoverAPI` — applied, but nothing reads them

7 and 10 examples apply these; **zero code references either**. They are not
dead in the same sense — they are the authoring *marker* for "this is a
control" / "this is a mover", and `RigExecMoverAPI` declares `rigExec:moves`
and `inputs:enabled`, which the evaluator very much does read *by name*.

**Do not remove.** `rigExec:moves` is the discovery mechanism for the whole
mover walk. The zero code references only mean the engine matches on property
name rather than on API presence. Listed here so a future sweep does not
mistake them for items 1-2.

### 4. Schema properties with no reader

Measured: declared in `schema.usda`, zero references in `libs/`.

| property | authored in | verdict |
|---|---|---|
| `rigExec:channelRole` | 0 examples | remove (with `RigExecControlAPI`'s body, not the class) |
| `rigExec:partitionId`, `rigExec:region` | 0 | remove with `RigExecPartitionAPI` |
| `rigExec:pointsReadPhase` | 0 | remove |
| `rigExec:driverCurveReadPhase` | 0 | remove |
| `rigExec:preserve`, `rigExec:upPolicy` | 2 | **keep** — authored intent for aim; see below |
| `rigExec:stretchPolicy`, `rigExec:unreachablePolicy` | 3 | **keep** — authored IK intent |
| `rigExec:basis`, `rigExec:parameterization`, `rigExec:frameTransport` | 1-2 | **keep** — authored ribbon/curve intent |
| `rigExec:distribution` | 2 | **keep** — authored twist intent |
| `rigExec:partition` | 10 | **keep** — universal, and the rig identity |

The "keep" rows are a different category and the distinction matters: they are
**authored in the examples and silently ignored by the engine**. Deleting them
would discard expressed intent; implementing them is the real fix. Removing
them from the schema while the examples author them would also break those
files.

They are the same hazard as the three math movers — authoring surface with no
behaviour — and should be tracked as such rather than deleted. See item 6.

### 5. Dead private tokens left by this session's deletions

`moverKernels.cpp`: `operation`, `enabledSource`, `transformProvider`,
`weightSource`, `blendChannelSource`, `baseSource`, `sourceRel`,
`movedProvider`, `aimAxisSource`, `aimTargetProvider`, `computeRestFrame`,
`sourceFrame`, `restFrame`, `aimTargetFrame`, `aimWeight`, `aimAxisToken`.

`computations.cpp`: `parentRel`.

All were inputs to the ten application host schemas or to
`RigExecPointFrameMoverApplication`. Pure residue — no behaviour attached.

**Remove:** all seventeen.

### 6. Declared-but-unevaluated movers — RESOLVED: implemented

`RigExecFloatMathMover`, `RigExecVec3fMathMover`, `RigExecMatrixMathMover`
were declared, compile-validated, and never evaluated. The product call went
the way this item recommended: they are implemented, not dropped.

`RigExecRigEvaluator::_EvaluatePropertyChains` runs each target's revisions in
composed post-order over the attribute's authored base, with the kernels in
`rigExecMath/propertyMath.{h,cpp}`. Results land in
`RigExecRigPose::movedProperties` in the property's own type, alongside the
point chains.

The half that mattered is the feedback: a property chain's inputs are all
authored on the mover, so it resolves **before** exec runs and its result is
handed back as an `ExecUsdValueOverride` on the target attribute. `03`'s
`ClampBlendWeight` therefore drives `RigExecBlendPointFrames` for real,
instead of that kernel reimplementing the author's clamp internally.

`TestPropertyMathMoversAreNotEvaluated` is gone, replaced by
`TestPropertyMathMoversAreEvaluated` (asserts the three published *values*),
`TestPropertyMoverFeedsConsumingComputation` (drives the mover somewhere the
consuming kernel's own bound cannot reach, so a change in the joint frames is
proof the override landed), `TestDisabledPropertyMoverPassesThrough`, and
`TestPropertyMoverTypeMismatchRejected`.

Both kinds of consumer see it. A computation reading the attribute gets the
exec value override; packet assembly, which never touches exec, gets the same
value through `RigExecResolvedInputs` — filled from the one chain result before
any input is read. `TestPropertyMoverReachesStaticPacketReads` drives a smooth
mover's `inputs:defaultWeight` to 0 from a property mover and asserts the smoothing
actually stops, with graph/CPU parity proving both routes carry the same
number.

## What is dead but should be FILLED, not removed

`RigExecSnapshotPrim::hasXform` / `xform`. The scene index already builds an
`HdXformSchema` from them (`sceneIndices.cpp` ~415-427); **nothing writes
them** — zero assignments in `libs/`.

This is the missing transform-publication path. Filling it lets a constraint
drive an `Xform` with geometry parented underneath, inheriting through Hydra's
flatten (the chain installs as a `UsdImagingSceneIndexPlugin`, upstream of
flattening), instead of point-deforming rigid geometry at constant weight 1.

Needs, in total:
1. bridge populates `xform`/`hasXform` from the evaluator's final matrix,
2. `frameProviderTypes` in `rigEvaluator.cpp` accepts any `UsdGeomXformable`
   and derives the source frame from its transform via `RigExecFrameFromMatrix`,
3. `08_AimEyes` rebuilt as `Xform` + parented card, dropping its two
   constant-weight matrix movers.

No schema change and no API required — that was the wrong conclusion drawn
earlier in this session and is corrected here.

## Order

Each step ends green: `cmake --build`, `ctest` (4 suites), 5 probe suites.

1. **Dead tokens** (item 5). Zero behaviour, pure text. Do first so the later
   diffs are readable.
2. **`RigExecPartitionAPI`, `RigExecTapAPI`** (item 2). Nothing applies them, so
   regenerating the schema plugin is the only risk.
3. **`RigExecPointTransformAPI`** (item 1). Biggest single removal. Check the
   landmark helpers for shared use *before* deleting them. Regenerate the
   plugin.
4. **Transform publication** (the FILL). Independent of 1-3 and the only item
   that adds capability; do it after the deletions so `08` is rebuilt once,
   against a clean schema.
5. **Ribbon Pass 1.5**, then delete `moverCompiler.{h,cpp}`, `_evalStage`,
   `_derivedSessionLayer`, `_generatedLayer`. This is the last authoring in the
   engine and the end of `docs/mover-graph-cutover.md`.

## Verification each step

```
cmake --build build && ctest --test-dir build
bin/run_probe.bat <scratch>/verify_rigs.py
bin/run_probe.bat <scratch>/verify_variants.py
bin/run_probe.bat <scratch>/verify_surface.py
bin/run_probe.bat <scratch>/verify_edges.py
bin/run_probe.bat <scratch>/verify_no_joint_authoring.py
```

Schema edits additionally require `bin/gen_schema.bat` before building, and a
usdview launch on at least one example — the plugin is loaded at runtime, so a
malformed schema passes the build and fails at stage open.

**Deleting a schema class is not covered by the C++ suites alone.** A class can
be removed while a registration still names it; that fails at
`TfType::FindByName` as a runtime coding error, not a compile error. After each
schema deletion, run `build/probeCodingError.exe examples` and confirm every
phase still reports `CLEAN`.
