# RigExec v0.1-alpha (prototype subset)

A rigging object model and character-computation layer **on top of OpenExec**,
implementing a prototype subset of the architecture in `docs/spec.md`
("Character Rig Execution Engine — Architecture Spec & Implementation Plan
v0.1") against an unchanged OpenUSD **v26.08** build.

Status honesty (from the adversarial validation rounds in
`../.omc/artifacts/ask/`): this is a **v0.1-alpha prototype** — materially
past the round-2 verdict but still not a full Phase 0/1-conformant
release. Since round 2:

- **Generated-application lowering now exists (B4 core)**: point chains
  (matrix + blend-shape movers) and pose-domain frame movers (aim) lower
  into `__RigExecGenerated` session-layer application prims
  (`RigExecPoint3fArrayMoverApplication` /
  `RigExecPointFrameMoverApplication`) evaluated **through OpenExec** —
  scalar-element `AttributeExpression` on `outputs:expression` with the
  passive `outputs:value` bridge extracting exact native `VtVec3fArray`,
  weight packets/blend channels as registered computations, disabled
  pass-through via `SetOutputToReferenceInput`, and a binding-epoch digest
  that recompiles on structural edits. CPU reference kernels are retained
  as the scalar-reference parity path (tested equal).
- Round-2 residuals addressed: authored `rigExec:zeroSidePolicy` wired
  through schema+callbacks; aim==up rejected; parent-up uses the selected
  up role; repeated-eigenspace SVD canonicalization with perturbation
  tests; exact `point3f[]` type and provider checks; full static-weight
  invariance and dynamic/base descriptor matching; blend structural
  failures + canonical input ordering; aim constraint honors its authored
  axis and preserves reflected handedness; incomplete snapshots refuse to
  publish.
- Schema surface completed for §4.1 (abstract WeightObject base,
  Vec3f/Matrix math movers, lattice/surface movers, generated application
  schemas).

Round-3 verdict (artifact in `.omc/artifacts/ask/`): *"the phrase
'working generated-application OpenExec seam' is justified"* — point
chains and final pose revisions extract from actual OpenExec-generated
applications, with CPU kernels relegated to parity mode. "Prototype"
remains essential: the seam proves the expression/passive-bridge
mechanism, not yet the full compilation/lifecycle/address/status
contracts. Post-verdict quick fixes applied (per-(mover,target)
application identity, dependency-aware epoch digest, transactional
layer cleanup on failed lowering, blend NaN rejection, weight
structural-token validation, singular-matrix warning).

Round-4 work addressed the round-3 blockers: **operation/type-specific
application hosts** (`RigExecMatrixPoint3fArrayMoverApplication`,
`RigExecBlendPoint3fArrayMoverApplication` — frozen signatures, no
runtime operation dispatch), **mover-owned `computeMoverParameters` and
`computeMoverStatus`** with compiler-authored `rigExec:resolved*`
lowering relationships (§12.1 pattern), status-gated pass-through in the
expression kernels, **transactional compilation** (validation into
locals; a failed structural edit keeps the previous epoch), **canonical
public tap addresses** with private generated-path resolutions (§9.1),
prototype rejection and ancestor deinstancing, preceding-phase and
duplicate-target rejection, ambiguous-sibling-writer validation
requiring a parent-authored reorder, and a dependency-aware
binding-epoch digest (rel targets, read phases, weight-descriptor shape,
blend membership/activations, joint outputs).

Round-4 verdict (artifact in `../.omc/artifacts/ask/`) accepted the
label: *"v0.1-alpha prototype with working single-target
generated-application OpenExec chains, but without conformant epoch
lifecycle, full dependency validation, multi-target parameter
specialization, or backend/pruning parity."* Its fixable findings were
applied immediately: multi-target blend movers are now explicitly
rejected (mover-owned parameters cannot yet be target-specialized —
rejected rather than silently wrong), per-(mover,target)-index
collision-proof application naming, previous-epoch restore when
authoring a replacement epoch fails, `Prepare()` validity gating before
an epoch commits, competing-cousin-writer validation at the common
ancestor's child order, unsatisfied-final-read rejection by ordinal,
`firstBadAddress` populated via `computePath`, and nearest-ancestor-only
deinstancing.

**Phase 3 Hydra publication now implemented** (`libs/rigExecImaging`,
spec §10): the three filtering scene indices
(`RigExecInternalPrimPruningSceneIndex`,
`RigExecBindingResolvingSceneIndex`, `RigExecResultsSceneIndex`) over an
atomic `RigExecSnapshotStore`; only standard Hydra data crosses the
boundary (flat points/normals primvars, extent min/max, xform matrices,
`HdBlockDataSource` masks for the derivative entries of owned points);
`GetPrim()` never computes — evaluation completes and publishes complete
immutable generations before precise coalesced dirtied notices (§8.2,
§10.3–10.5). Integrated into **stock usdview** via a
`UsdImagingSceneIndexPlugin` (the UsdSkelImaging precedent; see
`docs/hydra-integration-notes.md`) plus a usdview `PluginContainer` that
feeds stage + timeline into evaluation through a C surface and
`UsdUtils.StageCache` — `launch_usdview.bat examples\ArmShotAnim.usda`
shows the arm deforming live from OpenExec evaluation per frame;
`run_testusdview.bat` verifies activation and per-frame publication
headlessly, error-mark clean. Teardown ordering fix: the exec system is
destroyed before its generated sublayer is removed, so removal resyncs
are never delivered to a live system.

**Phase 3 kernel/motion/SIMD/delegate completion**: every declared mover
now executes through its operation/type-specific generated application —
volume correction (centroid scaling to the reference bound volume),
Laplacian smoothing with fixed adjacency, tensor-product Bernstein
lattice with bind-time cage capture, closest-point surface projection,
and the real curve/ribbon pipeline (arc-length-parameterized cubic
B-spline sampling with double-reflection rotation-minimizing frames,
rest-relative rigid transport for ribbon wrap, frame-origin guide
emission) — plus compiler-synthesized recomputeNormals (normal3f[] host)
and recomputeExtent (float3[] host) chains consuming the final
same-generation points (see the spec §7.6 deviation below). Motion samples (spec 10.5 subset): explicit
shutter-offset evaluation retained under one generation fence with
snapshot-backed sampled data sources implementing
GetContributingSampleTimesForInterval. CPU SIMD: an SSE weighted-matrix
kernel behind RIGEXEC_ENABLE_SIMD (default on), parity-gated against the
scalar reference at the spec 13.2 tolerance. Delegate matrix:
`run_testusdview.bat` (Storm) and `run_testusdview.bat Embree` both
verify identical live publications (PRMan-class production delegates are
not installable in this environment; Embree serves as the second
delegate). The spec's baked-export deliverable was excluded by explicit
user direction (no baking).

**Deviation from spec §7.6 (post movers), by user direction (2026-07-25)**:
`RigExecPostMover` no longer exists. Its two genuine deformation
operations are first-class typed movers — `RigExecSmoothMover` and
`RigExecVolumeCorrectMover`, each with `inputs:strength` — consistent
with every other mover schema. Its two derived-property operations
(recomputeNormals / recomputeExtent) are no longer authored at all: the
compiler synthesizes the derived-maintenance applications for every
written points target whose gprim authors the property (the synthesized
hosts are self-realized — `rigExec:mover` targets the host itself, which
owns `computeMoverParameters`/`computeMoverStatus`). Synthesis is
unconditional: the rig-level `rigExec:derived` opt-out was removed by
user direction (2026-07-26), so whether a gprim authors normals/extent is
the only control, and authoring or removing those properties is a
structural edit in the epoch digest. `docs/spec.md` intentionally still carries the
original §7.6 wording as the unmodified contract copy.

**Deviation from spec §4.1/§4.3 (joint and control schemas), by user
direction (2026-07-25)**: joints and controls now mirror OpenExec's Ir
contract exactly, with no Ir deviations. An abstract `RigExecXformable`
is a property-exact IrXformable mirror — matrix4d
rest/default/posed/parent spaces and scalar avars
(`avars:tx…rz`/`rspin`/`rotationOrder`/`unitScaleFactor`, rotations in
degrees). `RigExecJoint` inherits it and adds only the `guide:*` trio
(the exact IrJointScope shape); `RigExecControl` inherits it too —
animation is authored on avars (or `posed:space`), replacing the
point-frame control contract (`rigExec:restPoints`/`posePoints`,
`framePolicy`, `twist` → `avars:rspin`). Wiring is **view-free**
(user-directed 2026-07-25, `RigExecPointFrameView` deleted): a solver owns
an ordered `rel rigExec:joints` list (with optional `int[]
rigExec:jointElements` for non-contiguous selection), and the compiler
binds each listed joint to one element of the solver's
`computePointFrameArray`, which the joint then extracts. That binding is
compiler-private: it lives only in the generated layer on the evaluator's
derived stage, is not declared in the schema (removed by user direction
2026-07-26 — it is machinery, not an authoring surface), and never appears
on the user's stage. Joints so wired are posed only on the compiled
evaluation stage. Unconnected/unwired xformables follow the
namespace-parent's posed space with local rest offsets and avars (Ir's
fallback); a non-identity authored `posed:space` is used directly. Per Ir,
rest spaces are always orthonormalized (the engine enforces it); stretch
survives because the extracted frame carries the posed/rest axis-length
ratios. Guides are Ir-exact: cone height is the
authored `guide:length` only, and the sphere/cone keep unit radii.
Point frames (spec §5) remain the engine's internal value type —
xformables publish `computePointFrame`/`computeRestFrame`/
`computeMatrix`, converted at the schema boundary.

**Deviation from spec §4.1/§7.2 (lowering placement), by user direction
(2026-07-25)**: the compiled graph is engine-internal state, not user
scene description. The evaluator owns a private *derived evaluation
stage* — the source stage's authored layer stack composed under a
private session whose sublayers are `[generated..., source session]` —
and all discovery, validation, lowering, and exec compilation run
there. The source stage is never written: `__RigExecGenerated` does not
appear in its namespace, its session layer stays untouched, and the
imaging side consumes only published snapshots. The graph is cached per
binding epoch and rebuilt from incoming edits (authored layers are
shared, so change processing reaches the derived stage automatically);
the internal-prim pruning scene index remains as defense-in-depth.

**Deviation from spec §4.1 (view-free extraction / solver-owned
membership), by user direction (2026-07-25)**: `RigExecPointFrameView` is
deleted; a solver owns an ordered `rel rigExec:joints` list and the
compiler binds each joint to one aggregate element internally. This is a
deliberate, accepted departure from §4.1's rule that "no aggregate object
owns joint membership": a single ordered list opinion is now authoritative
for membership and element order. USD list-editing still composes joint
*additions* across layers, but implicit element indices depend on final
list order, so a stronger reorder/deletion shifts downstream indices
(`rigExec:jointElements` pins explicit indices where that matters). The
binding is validated up front (Phase A, before any epoch teardown, so an
invalid edit keeps the previous epoch publishable): every target must be a
`RigExecJoint`, no joint may be claimed by two solvers or also author its
own `posed:space` connection, `jointElements` must be `uniform` and
parallel to `joints`, and element indices must be non-negative and in
range for solvers whose frame count is statically knowable.

The binding itself is never authored. OpenExec can only reach a provider
through a relationship on the consuming prim, and `rigExec:joints` runs the
other way (there is no reverse-relationship accessor), so the evaluator keeps
the resolved joint → (solver, element) map in memory and supplies each bound
joint's frame to exec as a **value override** on `computePointFrame`. Every
in-exec consumer — the paired `computeMatrix`, a frame-chain application, and
the `NamespaceAncestor` fallback an unbound descendant follows — reads the
overridden value, so nothing has to be written anywhere. **Consequence:** a
solver-posed joint's frame is complete only through the evaluator; a raw
source-stage tap of `Joint.computePointFrame` returns the fallback pose, so
downstream consumers must read the compiled rig (imaging and mover transform
reads already do). The normative `docs/spec.md` still mirrors the original
Google Doc (§4.1 lists the removed type); deviations are tracked here per
project practice, not by editing the spec mirror.

**Phase 3 Hydra validation matrix completed** (spec §10.3–10.5, §14.5
Hydra bullets; round-6 gap 5): generation notices are now narrow and
dependency-derived — `RigExecSnapshotStore::Publish` diffs each
published prim's leaves against the previous generation and
`NotifyGenerationPublished` maps them to exact locators (`xform/matrix`,
`primvars/points/primvarValue` plus the epoch-owned
velocities/accelerations block entries, `primvars/normals/primvarValue`,
`extent/min|max`) expanded through every rebuilt ancestor with
`HdContainerDataSourceEditor::ComputeDirtyLocators()`; structural
output-set changes (prim enters/leaves the published set, ownership
changes) use universal dirtiness, and identical republication sends no
notice. `motionBlurSupport` capability handling (§10.3.1):
`RigExecImagingBridge::PreflightMotionProfile` fails a multi-sample
profile against a `false` capability bit before any evaluation; absent
leaves the application profile authoritative. `testRigExecImaging` now
covers the §14.5 construction/pull goldens: pre-population attachment,
already-populated wrapping with add-again resync, late-consumer
traversal, live added/removed/dirtied stream filtering with exact
locator preservation, the narrow-locator matrix with sentinel-expansion
assertions, the motion capability matrix with the one-sample non-motion
profile, legacy render-index pickup through Hydra's emulation
(`Hd_UnitTestNullRenderDelegate` + `InsertSceneIndex` — no RigExec
renderer code), and a recursive terminal-enumeration audit proving no
RigExec name crosses the renderer boundary.

Still open for Phase 1/2 conformance (round-4 §C, minus the items the
Phase 3 push closed): stage-owned persistent `ExecUsdSystem` with
off-stage `SdfChangeBlock` epoch diffs, per-target parameter
specialization for fan-out, complete cycle/catalog validation, exact
type/role/ordinal in the tap identity, authored-base PointBased
materialization (velocities/accelerations), aim `upPolicy`/`preserve`
consumption, weight-packet target/domain/cardinality identity fields,
FloatMathMover chains as addressable revisions, FK hierarchy packing,
generated-scope export/flatten exclusion, render-preflight/residency
machinery (§8.4), standalone Esf backend and `.rigpack` (§11), the
complete §14.3 conformance corpus, and PRMan-class delegate coverage.

Provenance: `docs/spec.md` is the verbatim extraction of the Google Doc
(id `1LUr_4YB_W_iMS5ORhM5XwPg_zsZBA05_EXrwbZkvmqM`), re-fetched and
verified byte-identical on 2026-07-25 before the round-4 validation.

OpenUSD is consumed unchanged (spec §3.5): no fork, patch, or modification.
All schemas, computations, math, extraction, and evaluation glue live here.

## Layout

| Path | Spec library | Contents |
|---|---|---|
| `libs/rigExecMath` | rigExecMath | `RigExecPointFrame` (four-point affine pose value, §5.1), reconstruction policies affine/orthogonal/axial/rigid (§5.2), degeneracy ladder (§5.3), Points↔Matrix round trip and SVD-based SRT interop with pinned reflection axis (§5.4), FK chain, analytic two-bone IK with pole/softness/uniform stretch, shortest-arc/log frame blend, swing-twist distribution, weighted-matrix point kernel (§7.4) |
| `libs/rigExecSchema` | rigExecSchema | `schema.usda` — RigExec schema domain (§4.1): Rig, Control, Joint, FkChain, TwoBoneIk, BlendPointFrames, TwistDistribution, Ribbon (all view-free: solvers own `rigExec:joints`), AimConstraint, FloatMathMover, StaticWeight, DynamicWeight, MatrixMover, BlendShapeMover/Input/Sample, CurveMover, SmoothMover, VolumeCorrectMover, and the applied APIs (PointTransformAPI, ControlAPI, MoverAPI, PartitionAPI, TapAPI). Generated as a **codeless** schema plugin (`gen_schema.bat` → `plugin/rigExecSchema/resources`) |
| `libs/rigExec` | rigExecCompute + rigExecUsd | `ExecTypeRegistry` registrations (`RigExecPointFrame`, `RigExecPointFrameArray`), `EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA` computations publishing paired `computePointFrame`/`computeMatrix` on every transform provider and aggregate `computePointFrameArray` on solvers (§5.7, §12.1), the typed tap/snapshot extraction facade over `ExecUsdSystem` (§9), and the rig evaluator with composed post-order mover discovery and the staged CPU geometry pipeline (§4.2, §7). `moverCompiler` compiles each target's mover chain; `moverKernels` holds the operation callbacks; `moverGraph` builds a chain as an in-memory `VdfNetwork` |
| `examples/` | — | `ArmRig.usda` and `ArmShotAnim.usda`, the spec §4.5/§4.6 reference assets |
| `tests/` | rigExecValidation (seed) | math conformance tests and end-to-end exec/animation/geometry tests |
| `docs/` | — | full spec, plus verified OpenExec API references (`exec-api-notes.md`, `execusd-api-notes.md`) |

## What runs through OpenExec today

Controls, joints, FK, two-bone IK, IK/FK blend, twist distribution, and
point-frame views compile and evaluate as registered OpenExec computations,
pulled through batched prepared requests (`RigExecTapSet`), with native USD
value resolution (splines via Ts, sparse timeSamples) supplying every animated
input at explicit `ChangeTime` time codes. Invalidation callbacks only flip
dirty state (spec §6.3); snapshots copy immutable values before crossing
threads (§6.1).

## Compiled mover graph (in progress)

`libs/rigExec/moverGraph.{h,cpp}` builds a target's revision chain as an
in-memory `VdfNetwork` — no derived stage, no generated prims, no schema
types for the revisions. A mover's write-set (`rigExec:moves`) plus its
namespace position already imply a dataflow chain; a `VdfNetwork` is what
that chain is, so materializing it as USD prims cost a second stage and a
recomposition while still not being able to express the optimizations the
graph form makes natural (per-element `VdfMask` splitting one revision
across face sets, cloned legs, sparse recomputation).

Landed so far: **all ten revision ops** — matrix, blendShape, volumeCorrect,
smooth, lattice, surfaceProject, ribbon, emitGuidePoints, recomputeNormals,
recomputeExtent — with `testRigExecMoverGraph` covering chained and mixed-op
composition, weighting, cardinality guards, and every pass-through path
(failed status, invalid packet, kind mismatch). The kernel bodies ported
essentially unchanged: they were already written against `VdfContext`, so
the move from a registered attribute expression to a node's `Compute` is a
change of binding, not of math, and the geometry ops still share one
scratch-collect helper as they did before.

Two behaviours are tightened by the in-place write. Blend masks are resolved
up front, because an in-place write cannot be rolled back partway through a
cardinality failure; and the derived-property hosts (`recomputeNormals` /
`recomputeExtent`) now fail the application when the recomputed cardinality
disagrees with the authored property, rather than silently resizing it.

Two contract notes for the remaining ops: a revision uses a READWRITE
connector, so it writes through `VdfReadWriteIterator(ctx, previous)` and
must **not** `Allocate` (that fails and silently passes through); and any
process talking to VDF directly must force `ExecTypeRegistry::GetInstance()`
or the executor cannot distinguish registered value types.

The binding and packet layers are done too: `RigExecResolveRevisionBinding`
replaces all eight compiler-authored `rigExec:resolved*` relationships with
build-time path resolution (they were always pure path choices — "which
provider supplies the matrix", "which prim's topology", "which cage
points"), and `RigExecAssembleParameters` builds every op's packet from
static stage reads plus evaluated provider values.
`TestAssembleAndEvaluateWithoutDerivedStage` runs the whole path and asserts
no `__RigExecGenerated` prim and no `resolved*` relationship exist after it.

**Done since:** the solver→joint binding is off the derived layer entirely.
Pass 0 is deleted, `rigExec:frameSource` / `rigExec:frameElement` are gone from
the joint registration, and no example stage carries either property on any of
its 24 joints.

**Remaining:** `RigExecRigEvaluator` still calls `RigExecCompileMoverChains`
and still owns a derived stage for the generated application prims. That swap
is atomic — point chains, frame chains, and publication move together, since
tap and Hydra publication still address results by generated-prim path. See
`docs/mover-graph-cutover.md` for the step-by-step and the verification that
must pass.

## Geometry execution

Every geometry mover executes through its operation/type-specific
generated application (`outputs:expression`/`outputs:value` passive
extraction bridge) compiled by OpenExec on the evaluator's private
derived evaluation stage — the source stage is never written (see the
deviation notes above). The staged CPU pipeline in `RigExecRigEvaluator`
is retained solely as an optional scalar reference: with
`cpuParityMode` set, `Evaluate` also runs the same kernels over the
same composed post-order walk into
`RigExecRigPose::movedPropertiesCpu` for exec/CPU parity assertions
(spec §7.4, §13.2). The standalone Esf backend and `.rigpack` (§11) and
inverse solves (§5.5) remain not started, matching the spec's phase
gating.

## Build

1. Build OpenUSD **v26.08** with OpenExec (default-on) from an unmodified
   upstream clone, installed as a sibling of this repo in `..\usd-install`:
   ```
   python OpenUSD\build_scripts\build_usd.py --generator Ninja ^
       --no-materialx --no-examples --no-tutorials --no-docs --embree ^
       ..\usd-install
   ```
   (drop `--embree` if you do not need the Embree renderer, and add
   `--no-usdview` if you do not need the usdview integration)
2. `gen_schema.bat` — generates the codeless schema plugin.
3. ```
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build
   ctest --test-dir build
   ```
   (run inside a VS2022 x64 dev prompt; `PATH` must include
   `usd-install\lib` and `usd-install\bin` to run tests)
