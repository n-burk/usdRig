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
codes. All eleven geometry revision ops — matrix, blendShape, volumeCorrect,
smooth, lattice, surfaceProject, ribbon, emitGuidePoints, **curvenet**,
recomputeNormals,
recomputeExtent — execute through the mover graph, with chained and mixed-op
composition, weighting, cardinality guards, and every pass-through path
(failed status, invalid packet, kind mismatch) under test. Hydra publication
(spec §10) drives **stock usdview** live: `bin/launch_usdview.bat
examples\ArmShotAnim.usda` shows the arm deforming from OpenExec evaluation
per frame.

**Curvenets and the Profile Mover.** Pixar's curve-based articulation
(de Goes, Sheffler & Fleischer, SIGGRAPH 2022) is implemented end to end:
`RigExecCurvenet` is a net of cubic splines over a shared control-point
pool, and `RigExecCurvenetMover` cuts the target mesh along it, builds the
cut-aware polygonal Laplacian, and reconstructs the surface from the net's
per-side deformation gradients. Because the curvenet is a
`UsdGeomPointBased`, its knots are posed by the ORDINARY movers — which
is the paper's own rigging model, and needs no curvenet-specific
articulation code. See `docs/curvenet.md` (the papers are in
`docs/papers/`), `examples/12_CurvenetProfile.usda`, and the authoring
panel under the usdview **RigExec ▸ Curvenet Authoring** menu.

**Prototype means** the seam is proven, not the full contracts.

The largest gap is **incremental recompilation**. Today a structural edit
bumps a composed-topology digest, and the next `Evaluate` notices the change
and runs a *full* `Compile()` (`libs/rigExec/rigEvaluator.cpp`). The
conformant design instead keeps a stage-owned persistent `ExecUsdSystem` and
hands it epoch *diffs*, with incoming edits batched inside an
`SdfChangeBlock` so a burst of edits yields one invalidation rather than one
per edit, and the compiled network updates in place instead of being rebuilt.

To be explicit, since the name invites the opposite reading:
`SdfChangeBlock` here is about **consuming** edits somebody else makes to the
stage — it batches change *notification*. It is not an authoring mechanism,
and nothing about it would change the fact that the engine writes nothing
(see [The one architectural fact](#the-one-architectural-fact), which
`testRigExecNoAuthoring` asserts by whole-scene equality).

Also still open for Phase 1/2 conformance: per-target parameter
specialization for fan-out (multi-target blend movers are currently
**rejected** rather than silently wrong), complete cycle/catalog validation,
exact type/role/ordinal in the tap identity, authored-base PointBased
materialization
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
| `testRigExecNoAuthoring` | that the engine authors nothing: the whole composed scene is byte-identical before and after compile, evaluation over six frames, and the full Hydra activate/publish/teardown cycle |
| `testRigExecImaging` | §14.5 construction/pull goldens, narrow-locator matrix, motion capability matrix, legacy render-index pickup, and a recursive terminal audit proving no RigExec name crosses the renderer boundary |
| `testRigExecWeightFields` | the volumetric falloff remap in isolation: band placement, reversed and degenerate bands, invert as a continuous lerp, every baked profile, the three distance functions, and weight-object composition including the non-commutative modes |
| `testRigExecVolumeWeights` | sphere/plane/curve fields driving real matrix movers, composition, the authored falloff spline, and both sample phases — every case asserting `moverGraphParityMismatches == 0`, i.e. that the exec kernels and the CPU oracle independently computed the same field |
| `testRigExecWeightOverlay` | the influence overlay through the scene index: displayColor tracking the weights, upstream primvars surviving, the on/off toggle dirtying structurally, the value-only change dirtying the narrow `primvars/displayColor` locator, and the volume guides for all three shapes |

`ctest --test-dir build` runs all of them. Two probes
(`probeCodingError`, `probeImagingPipeline`) need the scene-index plugin
discoverable through `Plug`, so they run from a `bin/run_probe.bat`-style
environment rather than bare ctest. `bin/run_testusdview.bat [renderer]` verifies
live activation and per-frame publication headlessly; both Storm and Embree
produce identical publications.

`bin/run_testusdview_overlay.bat [renderer]` closes the one gap the C++ suites
cannot: they drive a synthetic scene index upstream, which proves the filter
publishes a `displayColor` but not that usdview's *own* chain carries it. That
script turns the influence overlay on inside a real usdview, reads the terminal
scene index back through the same `HydraObserver` the Hydra Scene Browser uses,
and asserts the colours are a gradient rather than a flat wash — a constant
colour would satisfy every other check and still mean the weights never
arrived.

Those all assert against fixtures they own. For a rig you are *writing*,
`build/rigExecPose` evaluates an arbitrary stage and prints what came out —
what the compiler objected to, where the joints ended up, and how far each
moved property travelled:

```
rigExecPose <stage> [--rig <primPath>] [--frames a,b,c] [--joints] [--targets]
            [--joints-out <file.usda>]
```

It exits non-zero when the rig fails to compile or a generation comes back
invalid, so it can gate a build.

`--joints-out` writes the evaluated joint frames, as asset-space matrices
sampled at every requested frame, to a plain USD layer — a joint path list
and a parallel matrix array per time sample, nothing else. It is deliberately
schema-neutral, because its point is to hand the rig's own answer to
something that is not RigExec: a converter to another skinning schema, or a
comparison against one. `chars/puppetA` uses it to build and check a UsdSkel
copy of the same character.

## Layout

| Path | Spec library | Contents |
|---|---|---|
| `libs/rigExecMath` | rigExecMath | `RigExecPointFrame` (four-point affine pose value, §5.1), reconstruction policies affine/orthogonal/axial/rigid (§5.2), degeneracy ladder (§5.3), Points↔Matrix round trip and SVD-based SRT interop with pinned reflection axis (§5.4), FK chain, analytic two-bone IK with pole/softness/uniform stretch, shortest-arc/log frame blend, swing-twist distribution, weighted-matrix point kernel (§7.4). Pure — no USD deps beyond `gf`/`vt`. Static library, folded into `rigExec` |
| `libs/rigExecSchema` | rigExecSchema | `schema.usda` — the RigExec schema domain (§4.1). Generated as a **codeless** schema plugin (`bin/gen_schema.bat` → `plugin/rigExecSchema/resources`) |
| `libs/rigExec` | rigExecCompute + rigExecUsd | `ExecTypeRegistry` registrations (`RigExecPointFrame`, `RigExecPointFrameArray`), `EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA` computations publishing paired `computePointFrame`/`computeMatrix` on every transform provider and aggregate `computePointFrameArray` on solvers (§5.7, §12.1), the typed tap/snapshot extraction facade over `ExecUsdSystem` (§9), and `RigExecRigEvaluator` with composed post-order mover discovery. `moverGraph` builds a target's revision chain as an in-memory `VdfNetwork`; `moverKernels` holds the operation callbacks |
| `libs/rigExecImaging` | — | Hydra 2.0 publication (§10): the three filtering scene indices over an atomic `RigExecSnapshotStore`, the `UsdImagingSceneIndexPlugin`, and the C activation surface |
| `examples/` | — | `ArmRig.usda` / `ArmShotAnim.usda` (the spec §4.5/§4.6 reference assets) plus ten self-contained animated demo stages — see `examples/README.md` |
| `tests/` | rigExecValidation (seed) | the four ctest suites and the two probes |
| `tools/` | — | `rigExecPose`, which evaluates any rig stage and reports the result — see [Verification](#verification) |
| `docs/` | — | full spec mirror, verified OpenExec API references (`exec-api-notes.md`, `execusd-api-notes.md`), Hydra integration notes, and the cutover/removal working notes |

### Schema domain

32 classes, codeless (`skipCodeGeneration = true`), prefix `RigExec`:

- **Abstract bases** — `RigExecXformable` (a property-exact IrXformable
  mirror), `RigExecWeightObject`, `RigExecVolumeWeight`
- **Core** — `RigExecRoot`, `RigExecControl`, `RigExecJoint`
- **Solvers** — `RigExecFkChain`, `RigExecTwoBoneIk`, `RigExecBlendPointFrames`,
  `RigExecTwistDistribution`, `RigExecRibbon`, `RigExecAimConstraint`
- **Movers** — `RigExecMatrixMover`, `RigExecBlendShapeMover` (+ `BlendInput`,
  `BlendSample`), `RigExecCurveMover`, `RigExecLatticeMover`,
  `RigExecSurfaceMover`, `RigExecSmoothMover`, `RigExecVolumeCorrectMover`,
  and the math movers `RigExecFloatMathMover`, `RigExecVec3fMathMover`,
  `RigExecMatrixMathMover`
- **Weights** — `RigExecStaticWeight`, `RigExecDynamicWeight`, and the
  volumetric field generators `RigExecSphereWeight`, `RigExecPlaneWeight`,
  `RigExecCurveWeight` plus `RigExecCombineWeight`, which folds any of
  them together (see [`docs/volume-weights.md`](docs/volume-weights.md)).
  The volumetric types inherit `RigExecXformable`, not
  `RigExecWeightObject` — a typed schema gets exactly one base and they
  spend it on being *placeable*, so a volume authored inside a joint
  rides that joint with nothing wired. They redeclare the weight-object
  contract verbatim; weight-object identity is by type name.
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
- Only ever compiled with MSVC on Windows x64; the tree is written to build on
  Linux and macOS but that is unverified — see
  [Platform support](#platform-support).

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
bin/gen_schema.bat
```

Codeless means no compilation and no generated C++ — the domain is registered
purely from `generatedSchema.usda` + `plugInfo.json`.

### 3. Install

```
cmake --install build --prefix <your-rigexec-prefix>
```

The layout mirrors OpenUSD's own tree:

```
<prefix>/
  include/rigExec/          rigEvaluator.h, tapSet.h, moverGraph.h, types.h, ...
  include/rigExecMath/      pointFrame.h, solvers.h, geometryKernels.h, ...
  include/rigExecImaging/   registry.h, bridge.h, sceneIndices.h, ...
  lib/                      rigExec.dll, rigExecImaging.dll (+ import libraries)
  lib/usd/rigExecSchema/resources/    plugInfo.json, generatedSchema.usda
  lib/usd/rigExecImaging/resources/   plugInfo.json
  lib/python/rigExecUsdview/          plugInfo.json, rigExecUsdview.py
  lib/cmake/rigExec/                  rigExecConfig.cmake + targets
```

Shared libraries land in `lib/` beside their import libraries rather than
`bin/`, because that is what USD does on Windows — it keeps the installed
plugin's `"LibraryPath": "../../rigExecImaging.dll"` resolving exactly the way
`usd_usdSkelImaging.dll` does.

That `plugInfo.json` and the generated schema plugin's (which names the same
imaging library so Plug can load the compute-extent registration for the
Boundable RigExec types — see `docs/control-guides.md`) are the only ones
naming a library. The imaging one is generated from
`plugin/rigExecImaging/resources/plugInfo.json.in` rather than checked in — the
filename is `.dll` / `.so` / `.dylib` depending on the platform, and
`$<TARGET_FILE_NAME:>` is the only thing that knows it. One generated file
serves both trees: the build copy lands in `build/usd/rigExecImaging/resources/`
so its `../../<library>` hop reaches the build directory exactly as the
installed copy's hop reaches `lib/`.

Every destination is overridable if your tree differs — `RIGEXEC_INSTALL_LIBDIR`,
`RIGEXEC_INSTALL_INCLUDEDIR`, `RIGEXEC_INSTALL_PLUGINDIR`,
`RIGEXEC_INSTALL_PYTHONDIR`, `RIGEXEC_INSTALL_CMAKEDIR`. Installing with
`--prefix <your-usd-prefix>` merges into the USD install itself, putting the
RigExec plugins in the same `lib/usd` directory USD's own plugins live in.

### 4. Register the plugins

With `RIG` = your RigExec prefix and `USD` = your USD prefix:

```
PXR_PLUGINPATH_NAME = %RIG%\lib\usd\rigExecSchema\resources    (schema domain)
                      %RIG%\lib\usd\rigExecImaging\resources   (scene-index plugin)
                      %RIG%\lib\python\rigExecUsdview          (usdview only)

PYTHONPATH         += %RIG%\lib\python\rigExecUsdview          (usdview only)
PATH               += %RIG%\lib;%USD%\bin;%USD%\lib
```

(one variable per block above; entries are `;`-separated on Windows)

`find_package(rigExec)` publishes those same three directories as
`rigExec_PLUGINPATHS`, so a consuming build can compose the variable without
hardcoding the layout.

To run **uninstalled**, point the same variables at the source and build trees
instead — note that BOTH plugin directories are generated into the build
tree, since each `plugInfo.json` has to name the built library. The schema
one does because it declares `implementsComputeExtent`, and that flag is a
promise Plug can load code to satisfy: the checked-in copy under
`plugin/rigExecSchema/resources` is data-only and deliberately does not
carry it, so pointing a host there gives Boundable prims with silently
empty bounds:

```
PXR_PLUGINPATH_NAME = %RIG%\build\usd\rigExecSchema\resources    <-- generated
                      %RIG%\build\usd\rigExecImaging\resources   <-- generated
                      %RIG%\plugin\rigExecUsdview
PATH               += %RIG%\build;%USD%\bin;%USD%\lib
```

`bin/launch_usdview.bat`, `bin/run_probe.bat`, and `bin/run_testusdview.bat` are working
examples of exactly that environment.

### 5. Choose an integration level

**(a) As a library.** Consume the installed CMake package:

```cmake
find_package(rigExec REQUIRED)          # add the prefix to CMAKE_PREFIX_PATH
target_link_libraries(myapp PRIVATE rigExec::rigExec)
```

That carries the include directories and the OpenUSD link interface with it —
`rigExec::rigExecMath` and `rigExec::rigExecImaging` are exported too. Then
construct a `RigExecRigEvaluator` over your stage:

```cpp
RigExecRigPose pose = evaluator.Evaluate(time); // one complete generation
// pose.movedProperties: exact property path -> final native value.
//   VtVec3fArray for a point chain (points/normals/extent), and float /
//   GfVec3f / GfMatrix4d for a property chain (the math movers).
// pose.providerXforms:  driven UsdGeomXformable -> revised asset-space
//   matrix, paired with providerBaseXforms (consumers downstream of
//   Hydra's flatten need both to build the delta).
// pose.jointFramesFinal / jointMatricesFinal: posed joints.
// pose.diagnostics:     pass-through / failure / unimplemented reports
```

Those three are **peer output domains**, not a hierarchy: geometry,
transforms, and properties. A rig publishes any combination of them and needs
no `RigExecJoint` at all — `examples/rigexec_flat.usda` is one aim constraint
between two plain `UsdGeomXformable`s and nothing else. A rig with neither
joints nor movers is still rejected, because it publishes nothing.

**Read phases.** Which *revision* of an input a mover consumes is authored as
metadata on the relationship (or attribute) naming it — `rigExecReadPhase`,
one of `base`, `preceding`, `final`, or an absolute prim path meaning "the
value as of when the composed post-order walk finished with that prim".
Chains evaluate in dependency order; a cyclic phase read fails the compile.
See `examples/13_ReadPhases.usda`.

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
reg.Activate(stage, SdfPath(), initialTime, &errors); // every rig on stage
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
plugin does it via `ctypes`. An empty/null `rigPath` activates every
`RigExecRoot` on the stage and publishes them as one atomic generation; an
explicit path remains available for a single-rig host. Activation is
transactional, so compile or initial-evaluation failure does not replace the
current coherent generation. Order does not matter: chains and activation
rendezvous through a process-global registry reading an atomic snapshot store,
so activation may happen before or after chain construction. The registry is
one-active-stage-per-process; hosts rendering concurrent stages should own one
`RigExecImagingBridge` and snapshot store per stage instead of using this
stock-usdview rendezvous. Authored edits anywhere under an active rig's asset
re-evaluate every rig and republish once, so property edits redraw exactly like
timeline changes.

**(c) In stock usdview.** Set the three variables above and launch usdview
normally. The `RigExecUsdviewContainer` `PluginContainer` activates for any
stage carrying a `RigExecRoot` prim and feeds stage + timeline into evaluation.

```
bin\launch_usdview.bat examples\ArmShotAnim.usda          # interactive, Storm
bin\launch_usdview.bat examples\ArmShotAnim.usda Embree   # interactive, Embree
bin\run_testusdview.bat                                   # headless verification

bin/usdview.sh examples/ArmShotAnim.usda                  # same three, POSIX
bin/usdview.sh examples/ArmShotAnim.usda Embree
bin/run_testusdview.sh

bin/usdview.sh                                            # empty stage to build on
```

Joints and aggregate solvers draw as **guide geometry** — a sphere at each
posed frame origin plus a cone along the aim axis. Both size those
primitives with `guide:radius`, which matters as soon as an asset is not
built at the tens-of-units scale Hydra's fallback radius of 1.0 suits; zero
or negative draws nothing, which is how a solver's diagnostics are turned
off.

### Authoring a rig

`examples/` has self-contained animated stages, one per feature area (FK
chain, two-bone IK, IK/FK blend, blend shapes, twist ribbon spine, lattice,
surface drape, aim, property math movers, aim xform, volume weights,
curvenet profile, read phases), plus `rigexec_flat.usda` — the smallest rig
there is. Each is a complete worked example; `examples/README.md` indexes
them.

### Platform support

| Platform | Status |
|---|---|
| Windows x64 | **Built and tested.** MSVC 19.36, Python 3.10, Ninja — everything documented above is verified here |
| Linux x86_64 | Should build; **not compiled on the platform** |
| macOS x86_64 | Should build; **not compiled on the platform** |
| macOS arm64 (Apple Silicon) | Should build; the arm64 code path is exercised on x86 (see below), the rest is unverified |
| iOS | Core libraries only, and not as a plugin — see [iOS](#ios) |

Be clear about what "should build" means: **no compiler other than MSVC has
been run against this tree.** The portability below is by construction and
review, not a green build on those platforms.

What was made portable:

- `RigExecApplyWeightedMatrixSimd` selects SSE2 on x86 and a scalar
  implementation everywhere else, so arm64 compiles instead of failing on
  `<emmintrin.h>`. The non-SSE path delegates to `RigExecApplyWeightedMatrix`
  — the same scalar reference kernel the parity mode compares SIMD against —
  so on those targets the two agree exactly rather than to tolerance.
  Building with `-DRIGEXEC_DISABLE_SSE2` forces that path on an x86 machine;
  the full ctest suite passes that way, which is how the arm64 code path is
  checked without arm64 hardware.
- The C activation surface uses an export macro (`dllexport` while building,
  `dllimport` for consumers, default visibility on ELF/Mach-O) rather than a
  bare `__declspec(dllexport)`, which was a hard compile failure off MSVC.
- The imaging `plugInfo.json` is generated from a template using
  `$<TARGET_FILE_NAME:>`, so it names `.dll` / `.so` / `.dylib` correctly in
  both the build and install trees.
- Installed binaries get `$ORIGIN` (ELF) or `@loader_path` (Mach-O) on
  `INSTALL_RPATH`, plus `CMAKE_INSTALL_RPATH_USE_LINK_PATH`, so a `dlopen`ed
  `rigExecImaging` resolves `rigExec` beside it and the USD libraries in their
  own prefix. Note that bakes an absolute path to that USD install into the
  installed binaries.
- The tests are host command-line executables driven by ctest, so they are off
  by default when `CMAKE_CROSSCOMPILING`; `RIGEXEC_BUILD_TESTS` overrides.

The helper scripts in `bin/` come in both flavours: a `.bat` for Windows and a
`.sh` twin for Linux/macOS, each pair driving the same tool with the same
arguments. Neither carries an absolute path — every one resolves `RIG` from the
script's own location and expects `usd-install` as a sibling of the checkout, so
setting `RIG`, `USD`, or (POSIX only) `VENV` in the environment overrides that
for a non-standard layout. `bin/_env.bat` and `bin/_env.sh` hold the shared
environment; the per-platform differences — `;` versus `:` separators,
`Lib\site-packages` versus `lib/python3.11/site-packages`, `PATH` versus
`DYLD_LIBRARY_PATH` — are settled there rather than in each script.

#### iOS

The core libraries can compile for iOS once the arm64 path is taken, but the
**plugin-based integration does not carry over**, and none of this has been
attempted:

- The usdview integration (level **c**) simply does not exist — there is no
  usdview host on iOS.
- `PXR_PLUGINPATH_NAME` is not a deployment mechanism for a shipped app.
  Bundle the plugin resources and call `PlugRegistry::RegisterPlugins()` with
  the app-bundle path instead.
- A bare `.dylib` in `lib/` is not loadable by a third-party iOS app —
  dynamic code must ship as an embedded, signed framework. So `rigExecImaging`
  has to be either force-linked into a static/monolithic build behind an
  explicit registration entry point, or packaged as a framework whose binary
  is what `LibraryPath` names.
- The codeless schema domain survives unchanged as bundled resources; it is
  only JSON and `.usda`.

The realistic iOS design is therefore static/monolithic with explicit
registration, consuming RigExec as a library (level **a**). The install layout
here targets desktop and does not produce a framework.

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
2. `bin/gen_schema.bat` — only if you edited `schema.usda`.
3. ```
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build
   ctest --test-dir build
   ```
   (run inside a VS2022 x64 dev prompt; `PATH` must include
   `usd-install\lib` and `usd-install\bin` to run tests)

`bin/build_rigexec.bat` does step 3 plus the environment setup in one shot, and
`bin/build_rigexec.sh` is its POSIX twin (`--no-test` skips the ctest run). The
helper scripts resolve their own paths from `bin/`, so there is nothing to edit
for your own location — see [Platform support](#platform-support) for the
`RIG` / `USD` / `VENV` overrides if `usd-install` is not a sibling of the
checkout.

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

Normal recomputation accumulates each face's Newell normal onto its own
corners, weighted by the interior angle there. That is correct for a
non-planar n-gon and independent of how the polygon would be triangulated —
unlike the per-triangle fan it replaced (2026-08-06), which gave a vertex
adjacent to the fan anchor only one sliver triangle out of the whole face
and could cancel a valid manifold vertex's normal to zero where two faces
meet along a symmetry seam.

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
