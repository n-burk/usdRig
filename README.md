# RigExec v0.1-alpha (prototype subset)

A rigging object model and character-computation layer **on top of OpenExec**,
implementing a prototype subset of the architecture in `docs/spec.md`
("Character Rig Execution Engine — Architecture Spec & Implementation Plan
v0.1") against an unchanged OpenUSD **v26.08** build.

OpenUSD is consumed unchanged (spec §3.5): no fork, patch, or modification.
All schemas, computations, math, extraction, and evaluation glue live here.

## The one architectural fact

**The engine authors nothing.** There is no compiler, no generated prim, no
derived evaluation stage, and no schema type for a mover revision. The source
stage is never written.

- Transforms and solvers evaluate as registered **OpenExec computations** on
  the source stage.
- Geometry mover chains evaluate through an **in-memory `VdfNetwork`**
  (`RigExecMoverGraph`). A mover's write-set (`rigExec:moves`) plus its
  namespace position already imply a dataflow chain, and a `VdfNetwork` *is*
  that chain — materializing it as USD prims cost a second stage and a
  recomposition while still not expressing the optimizations the graph form
  makes natural (per-element `VdfMask` splitting one revision across face sets,
  cloned legs, sparse recomputation).
- Everything that has to reach OpenExec from outside the relationship graph —
  solver→joint bindings, constraint frame revisions, ribbon driver points —
  arrives as a **value override**, not as authored scene description.

That last point has a consequence worth stating up front: a solver-posed
joint's frame is complete **only through the evaluator**. A raw source-stage
tap of `Joint.computePointFrame` returns the fallback pose. Downstream
consumers must read the compiled rig (imaging and mover transform reads
already do). This is because OpenExec can only reach a provider through a
relationship on the *consuming* prim, and `rigExec:joints` runs the other way
with no reverse-relationship accessor — so the evaluator keeps the resolved
joint → (solver, element) map in memory and supplies each bound joint's frame
as an override on `computePointFrame`. Every in-exec consumer (the paired
`computeMatrix`, a frame-chain application, the `NamespaceAncestor` fallback
an unbound descendant follows) reads the overridden value.

The override set is iterated **to a fixed point**, not computed once: solvers
are not downstream-only of controls — a `RigExecTwistDistribution` reads joints
another solver poses — so each round's aggregate results feed the next round's
overrides until they stop changing. Ribbon driver points reach exec the same
way (`rigExec:computeDriverPoints` / `computeRestDriverPoints`), which is what
allowed the last authoring pass to be deleted.

## Status

This is a **v0.1-alpha prototype**. What that qualifier does and does not mean:

**Working end to end.** Controls, joints, FK chains, analytic two-bone IK,
IK/FK blend, twist distribution, aim constraints, and ribbons compile and
evaluate as OpenExec computations, pulled through batched prepared requests
(`RigExecTapSet`) with native USD value resolution (splines via `Ts`, sparse
timeSamples) supplying every animated input at explicit `ChangeTime` time
codes. All ten geometry revision ops — matrix, blendShape, volumeCorrect,
smooth, lattice, surfaceProject, ribbon, emitGuidePoints, recomputeNormals,
recomputeExtent — execute through the mover graph, with chained and mixed-op
composition, weighting, cardinality guards, and every pass-through path
(failed status, invalid packet, kind mismatch) under test. Hydra publication
(spec §10) drives **stock usdview** live: `launch_usdview.bat
examples\ArmShotAnim.usda` shows the arm deforming from OpenExec evaluation
per frame.

**Prototype means** the seam is proven, not the full contracts. Still open for
Phase 1/2 conformance: stage-owned persistent `ExecUsdSystem` with off-stage
`SdfChangeBlock` epoch diffs, per-target parameter specialization for fan-out
(multi-target blend movers are currently **rejected** rather than silently
wrong), complete cycle/catalog validation, exact type/role/ordinal in the tap
identity, authored-base PointBased materialization
(velocities/accelerations), aim `upPolicy`/`preserve` consumption,
weight-packet target/domain/cardinality identity fields, `FloatMathMover`
chains as addressable revisions, FK hierarchy packing, render-preflight and
residency machinery (§8.4), the standalone Esf backend and `.rigpack` (§11),
the complete §14.3 conformance corpus, and PRMan-class delegate coverage.
The spec's baked-export deliverable was excluded by explicit user direction
(no baking); inverse solves (§5.5) are not started, matching the spec's phase
gating.

**Validation.** The design was reviewed across six adversarial rounds; the
round-4 verdict accepted the label *"v0.1-alpha prototype with working
single-target generated-application OpenExec chains."* Every finding from
those rounds that was fixable was applied, and the architecture has since
moved further — the generated-application/derived-stage mechanism those
rounds examined has been deleted outright in favour of the in-memory graph
described above. Round artifacts live in `../.omc/artifacts/ask/` and are not
part of this repository.

## Verification

| Suite | What it covers |
|---|---|
| `testRigExecMath` | §5 math conformance: reconstruction policies, degeneracy ladder, Points↔Matrix round trip, SVD/SRT with pinned reflection axis, IK, blend, twist |
| `testRigExecArm` | end-to-end exec/animation/geometry against the `examples/` reference assets |
| `testRigExecMoverGraph` | every revision op, chained and mixed composition, weighting, cardinality guards, all pass-through paths |
| `testRigExecImaging` | §14.5 construction/pull goldens, narrow-locator matrix, motion capability matrix, legacy render-index pickup, and a recursive terminal audit proving no RigExec name crosses the renderer boundary |

`ctest --test-dir build` runs all four. Two probes
(`probeCodingError`, `probeImagingPipeline`) need the scene-index plugin
discoverable through `Plug`, so they run from a `run_probe.bat`-style
environment rather than bare ctest. `run_testusdview.bat [renderer]` verifies
live activation and per-frame publication headlessly; both Storm and Embree
produce identical publications.

## Layout

| Path | Spec library | Contents |
|---|---|---|
| `libs/rigExecMath` | rigExecMath | `RigExecPointFrame` (four-point affine pose value, §5.1), reconstruction policies affine/orthogonal/axial/rigid (§5.2), degeneracy ladder (§5.3), Points↔Matrix round trip and SVD-based SRT interop with pinned reflection axis (§5.4), FK chain, analytic two-bone IK with pole/softness/uniform stretch, shortest-arc/log frame blend, swing-twist distribution, weighted-matrix point kernel (§7.4). Pure — no USD deps beyond `gf`/`vt`. Static library, folded into `rigExec` |
| `libs/rigExecSchema` | rigExecSchema | `schema.usda` — the RigExec schema domain (§4.1). Generated as a **codeless** schema plugin (`gen_schema.bat` → `plugin/rigExecSchema/resources`) |
| `libs/rigExec` | rigExecCompute + rigExecUsd | `ExecTypeRegistry` registrations (`RigExecPointFrame`, `RigExecPointFrameArray`), `EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA` computations publishing paired `computePointFrame`/`computeMatrix` on every transform provider and aggregate `computePointFrameArray` on solvers (§5.7, §12.1), the typed tap/snapshot extraction facade over `ExecUsdSystem` (§9), and `RigExecRigEvaluator` with composed post-order mover discovery. `moverGraph` builds a target's revision chain as an in-memory `VdfNetwork`; `moverKernels` holds the operation callbacks |
| `libs/rigExecImaging` | — | Hydra 2.0 publication (§10): the three filtering scene indices over an atomic `RigExecSnapshotStore`, the `UsdImagingSceneIndexPlugin`, and the C activation surface |
| `examples/` | — | `ArmRig.usda` / `ArmShotAnim.usda` (the spec §4.5/§4.6 reference assets) plus ten self-contained animated demo stages — see `examples/README.md` |
| `tests/` | rigExecValidation (seed) | the four ctest suites and the two probes |
| `docs/` | — | full spec mirror, verified OpenExec API references (`exec-api-notes.md`, `execusd-api-notes.md`), Hydra integration notes, and the cutover/removal working notes |

### Schema domain

27 classes, codeless (`skipCodeGeneration = true`), prefix `RigExec`:

- **Abstract bases** — `RigExecXformable` (a property-exact IrXformable
  mirror), `RigExecWeightObject`
- **Core** — `RigExecRig`, `RigExecControl`, `RigExecJoint`
- **Solvers** — `RigExecFkChain`, `RigExecTwoBoneIk`, `RigExecBlendPointFrames`,
  `RigExecTwistDistribution`, `RigExecRibbon`, `RigExecAimConstraint`
- **Movers** — `RigExecMatrixMover`, `RigExecBlendShapeMover` (+ `BlendInput`,
  `BlendSample`), `RigExecCurveMover`, `RigExecLatticeMover`,
  `RigExecSurfaceMover`, `RigExecSmoothMover`, `RigExecVolumeCorrectMover`,
  and the math movers `RigExecFloatMathMover`, `RigExecVec3fMathMover`,
  `RigExecMatrixMathMover`
- **Weights** — `RigExecStaticWeight`, `RigExecDynamicWeight`
- **Applied APIs** — `RigExecControlAPI`, `RigExecMoverAPI`

Geometry stays native `UsdGeom` — points, normals, extent, widths, primvars.
There is no parallel geometry schema.

## Hydra publication

Three filtering scene indices (`RigExecInternalPrimPruningSceneIndex`,
`RigExecBindingResolvingSceneIndex`, `RigExecResultsSceneIndex`) sit over an
atomic `RigExecSnapshotStore`. Only standard Hydra data crosses the boundary:
flat points/normals primvars, extent min/max, xform matrices, and
`HdBlockDataSource` masks for the derivative entries of owned points.
`GetPrim()` never computes — evaluation completes and publishes complete
immutable generations *before* precise coalesced dirtied notices (§8.2,
§10.3–10.5).

Generation notices are narrow and dependency-derived: `Publish` diffs each
prim's leaves against the previous generation and maps them to exact locators,
expanded through every rebuilt ancestor with
`HdContainerDataSourceEditor::ComputeDirtyLocators()`. Structural output-set
changes use universal dirtiness; identical republication sends no notice.
`motionBlurSupport` (§10.3.1) is preflighted — a multi-sample profile fails
against a `false` capability bit before any evaluation.

Because no RigExec name reaches the render delegate, **any delegate works
unmodified**. Storm and Embree are both verified; PRMan-class production
delegates are not installable in this environment.

## Using RigExec with your own OpenUSD

### Requirements

- **OpenUSD v26.08** built with OpenExec (`PXR_BUILD_EXEC=ON`, the default).
  RigExec needs `exec`, `execUsd`, `ef`, and `vdf` present in the install.
- A C++17 toolchain matching your USD build's compiler and ABI.
- In practice Windows/MSVC only — see [Portability](#portability).

### 1. Build against your install

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release ^
      -DUSD_INSTALL_DIR=<your-usd-prefix>
cmake --build build
```

`USD_INSTALL_DIR` defaults to `../usd-install`. This produces two loadable
libraries in `build/`:

| Artifact | Contents |
|---|---|
| `rigExec.dll` | exec type registrations, schema computations, mover graph, tap set, evaluator (`rigExecMath` is static and folded in) |
| `rigExecImaging.dll` | the three scene indices, the `UsdImagingSceneIndexPlugin`, and the C activation surface |

### 2. Schema plugin

The codeless schema plugin is already generated and checked in at
`plugin/rigExecSchema/resources`. Regenerate it only if you edit
`libs/rigExecSchema/schema.usda`:

```
gen_schema.bat
```

Codeless means no compilation and no generated C++ — the domain is registered
purely from `generatedSchema.usda` + `plugInfo.json`.

### 3. Register the plugins

There are **no CMake `install()` rules**; the plugin layout is in-tree and
registration is by environment variable. With `RIG` = this repo's root and
`USD` = your USD prefix:

```
PXR_PLUGINPATH_NAME = %RIG%\plugin\rigExecSchema\resources    (schema domain)
                      %RIG%\plugin\rigExecImaging\resources   (scene-index plugin)
                      %RIG%\plugin\rigExecUsdview             (usdview only)

PYTHONPATH         += %RIG%\plugin\rigExecUsdview             (usdview only)
PATH               += %RIG%\build;%USD%\bin;%USD%\lib
```

(one variable per block above; entries are `;`-separated on Windows)

> **Caveat.** `plugin/rigExecImaging/resources/plugInfo.json` declares
> `"LibraryPath": "../../../build/rigExecImaging.dll"`, i.e. it assumes the
> build tree is `<repo>/build`. If you build elsewhere, edit that path or copy
> the `resources` directory next to your binaries.

`launch_usdview.bat`, `run_probe.bat`, and `run_testusdview.bat` are working
examples of exactly this environment.

### 4. Choose an integration level

**(a) As a library.** Link `rigExec`, construct a `RigExecRigEvaluator` over
your stage, then:

```cpp
RigExecRigPose pose = evaluator.Evaluate(time); // one complete generation
// pose.movedProperties: exact property path -> final native value
// pose.diagnostics:     pass-through / failure / unimplemented reports
```

`Evaluate` compiles on first use and **recompiles itself** whenever the
composed mover topology digest changes, so structural edits need no call from
you (`GetBindingEpochDigest()` exposes that digest if you want to observe
epochs). Call `Compile(&errors)` explicitly only when you want the validation
messages — `Evaluate` discards them, surfacing just a terminal
`"structural recompilation failed"` in `pose.diagnostics`.

`RigExecTapSet` gives typed batched extraction instead of the raw map. No
Hydra involved. Set `cpuParityMode` to also run the scalar reference kernels
into `movedPropertiesCpu` for parity assertions (§7.4, §13.2).

**(b) Into any UsdImaging client.** Loading the `rigExecImaging` plugin is
enough to get the filter chain: its `UsdImagingSceneIndexPlugin` inserts the
three scene indices into every constructed chain automatically, following the
`UsdSkelImaging` precedent (see `docs/hydra-integration-notes.md`).

But the plugin only *builds* the chain — something must **drive evaluation**.
Either call the registry directly from C++:

```cpp
auto &reg = rigExec::RigExecImagingRegistry::GetInstance();
reg.Activate(stage, rigPath, initialTime, &errors);
reg.SetTime(frame);          // evaluate + publish, broadcast to every chain
reg.Deactivate();
```

or use the language-neutral C surface exported from `rigExecImaging.dll`
(0 = success):

```c
int       RigExecImaging_Activate(long long stageCacheId,
                                  const char *rigPath, double initialFrame);
int       RigExecImaging_SetTime(double frame);
void      RigExecImaging_Deactivate(void);
long long RigExecImaging_GetGeneration(void);   // 0 before first publication
```

`stageCacheId` is a `UsdUtilsStageCache` id, so any language that can put a
stage in the cache can activate — that is exactly how the Python usdview
plugin does it via `ctypes`. Order does not matter: chains and activation
rendezvous through a process-global registry reading an atomic snapshot store,
so activation may happen before or after chain construction. Authored edits
anywhere under the rig re-evaluate and republish automatically, so property
edits redraw exactly like timeline changes.

**(c) In stock usdview.** Set the three variables above and launch usdview
normally. The `RigExecUsdviewContainer` `PluginContainer` activates for any
stage carrying a `RigExecRig` prim and feeds stage + timeline into evaluation.

```
launch_usdview.bat examples\ArmShotAnim.usda          # interactive, Storm
launch_usdview.bat examples\ArmShotAnim.usda Embree   # interactive, Embree
run_testusdview.bat                                   # headless verification
```

Joints and aggregate solvers draw as **guide geometry** — a sphere at each
posed frame origin plus a cone along the aim axis.

### Authoring a rig

`examples/` has ten self-contained animated stages, one per feature area (FK
chain, two-bone IK, IK/FK blend, blend shapes, twist ribbon spine, lattice,
surface drape, aim, property math movers, aim xform). Each is a complete
worked example; `examples/README.md` indexes them.

### Portability

Built and tested only on Windows (MSVC 19.36, Python 3.10, Ninja). The C
activation surface uses `__declspec(dllexport)` unconditionally and the helper
scripts are all `.bat`, but the CMake build and the libraries carry no other
platform dependency — a POSIX port is mostly export-macro and script work.

## Build (this repo's own layout)

The scripts here assume the sibling layout this project was developed in —
`OpenUSD/` (upstream clone), `usd-install/` (its install tree), and `usdRig/`
(this repo) side by side. Neither of the first two is part of this repository.

1. Build OpenUSD **v26.08** with OpenExec (default-on) from an unmodified
   upstream clone, installed as a sibling of this repo in `..\usd-install`:
   ```
   python OpenUSD\build_scripts\build_usd.py --generator Ninja ^
       --no-materialx --no-examples --no-tutorials --no-docs --embree ^
       ..\usd-install
   ```
   (drop `--embree` if you do not need the Embree renderer, and add
   `--no-usdview` if you do not need the usdview integration)
2. `gen_schema.bat` — only if you edited `schema.usda`.
3. ```
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build
   ctest --test-dir build
   ```
   (run inside a VS2022 x64 dev prompt; `PATH` must include
   `usd-install\lib` and `usd-install\bin` to run tests)

`build_rigexec.bat` does step 3 plus the environment setup in one shot. The
helper scripts (`build_rigexec.bat`, `launch_usdview.bat`, `gen_schema.bat`,
`run_probe.bat`, `run_testusdview.bat`) carry absolute `D:\work\usdRig\...`
paths — edit the `USD` and `RIG` variables at the top of each for your own
location.

## Deviations from the spec

`docs/spec.md` is an unmodified contract copy and intentionally still carries
the original wording for everything below. Deviations are tracked here, not by
editing the spec mirror.

**§7.2 / §4.1 — lowering placement (user direction, 2026-07-25, and since
superseded further).** The spec lowers the compiled graph into scene
description. It is instead engine-internal state. The intermediate step was a
private derived evaluation stage holding a generated session layer; that has
since been removed entirely along with the compiler itself. Nothing is
authored anywhere. The internal-prim pruning scene index remains as
defense-in-depth only.

**§7.6 — post movers (user direction, 2026-07-25).** `RigExecPostMover` no
longer exists. Its two genuine deformation operations are first-class typed
movers — `RigExecSmoothMover` and `RigExecVolumeCorrectMover`, each with
`inputs:strength` — consistent with every other mover schema. Its two
derived-property operations are no longer authored at all: the evaluator
synthesizes `recomputeNormals` / `recomputeExtent` revisions for every written
points target whose gprim authors the property. Synthesis is unconditional —
the rig-level `rigExec:derived` opt-out was removed (2026-07-26), so whether a
gprim authors normals/extent is the only control, and authoring or removing
those properties is a structural edit in the epoch digest. The derived-property
revisions fail the chain when recomputed cardinality disagrees with the
authored property, rather than silently resizing it.

**§4.1 / §4.3 — joint and control schemas (user direction, 2026-07-25).**
Joints and controls mirror OpenExec's Ir contract exactly, with no Ir
deviations. Abstract `RigExecXformable` is a property-exact `IrXformable`
mirror — matrix4d rest/default/posed/parent spaces and scalar avars
(`avars:tx…rz`, `rspin`, `rotationOrder`, `unitScaleFactor`; rotations in
degrees). `RigExecJoint` inherits it and adds only the `guide:*` trio (the
exact `IrJointScope` shape); `RigExecControl` inherits it too, so animation is
authored on avars (or `posed:space`), replacing the point-frame control
contract. Unconnected xformables follow the namespace-parent's posed space
with local rest offsets and avars (Ir's fallback); a non-identity authored
`posed:space` is used directly. Per Ir, rest spaces are always orthonormalized
— stretch survives because the extracted frame carries the posed/rest
axis-length ratios. Guides are Ir-exact: cone height is the authored
`guide:length` only, and sphere/cone keep unit radii. Point frames (§5) remain
the engine's *internal* value type; xformables publish
`computePointFrame`/`computeRestFrame`/`computeMatrix`, converted at the
schema boundary.

**§4.1 — view-free extraction / solver-owned membership (user direction,
2026-07-25).** `RigExecPointFrameView` is deleted. A solver owns an ordered
`rel rigExec:joints` list (with optional `int[] rigExec:jointElements` for
non-contiguous selection) and each listed joint binds to one element of the
solver's `computePointFrameArray`. This is a deliberate departure from §4.1's
rule that "no aggregate object owns joint membership": a single ordered list
opinion is now authoritative for membership and element order. USD list-editing
still composes joint *additions* across layers, but implicit element indices
depend on final list order, so a stronger reorder or deletion shifts downstream
indices — `rigExec:jointElements` pins explicit indices where that matters.

The binding is validated up front, before any epoch teardown, so an invalid
edit keeps the previous epoch publishable: every target must be a
`RigExecJoint`, no joint may be claimed by two solvers or also author its own
`posed:space` connection, `jointElements` must be `uniform` and parallel to
`joints`, and element indices must be non-negative and in range for solvers
whose frame count is statically knowable. The binding itself is never
authored — see [The one architectural fact](#the-one-architectural-fact).

## Provenance

`docs/spec.md` is the verbatim extraction of the source Google Doc (id
`1LUr_4YB_W_iMS5ORhM5XwPg_zsZBA05_EXrwbZkvmqM`), re-fetched and verified
byte-identical on 2026-07-25.
