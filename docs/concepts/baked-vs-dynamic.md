---
title: Evaluation and independent checks
summary: One production operation graph for native, frozen and runtime evaluation, with optional independent judges.
order: 20
---

UsdRig compiles a rig into one operation graph. Native evaluation reads scene
inputs at the owning-thread sampling boundary, then executes operations over
typed values. Frozen evaluation samples those inputs first and runs detached
state through the same graph. A `.rigexec` binary supplies compiled declarations
and values to the runtime. There is no evaluator mode selector or fallback walk.

## Compilation and execution

Each operation declares its input and output values. The shared compiler binds
reads to exact producers, computes a canonical dependency order, and lowers
clusters without introducing extra causal dependencies. Descriptor emission
order does not require an input producer to be emitted before its consumer.
Changed input values activate their downstream operations; unchanged outputs
remain available to later generations. Serial and parallel execution use the
same graph and readiness rules.

Structural scene edits rebuild the compiled declarations. Cyclic operations
are reported by canonical loop keys and set aside; their outputs are cleared so
a later or held generation cannot reuse stale cyclic results. Unsupported
source declarations or failed compilation produce an invalid pose with a
reason, rather than selecting a second evaluator.

## Optional independent judges

`Rig.cpu_reference` enables the independent scalar point reference. Its original
arithmetic consumes captured source facts and explicit current-generation
phase values. The pose reports `moved_properties_cpu`, `reference_agreements`
and `reference_mismatches`. This check does not select a production engine.

`rigExecPose --cpu-reference` checks that reference coverage matches the
published point chains. `--exec-crosscheck` checks actual operation rows against
OpenExec requests fed with captured original inputs; see
[OpenExec reference checks](#openexec-reference-checks). Exact golden
verification compares published values, status and ordered semantic
diagnostics against independent original captures;
work counters are outside that value contract. CPU point agreement retains its
existing numerical comparison policy and is separate from exact golden checks.

The golden tool supports `--golden-backend native`, `frozen`, or `runtime`.
Runtime visits also require `--golden-program file.rigexec`. Each backend uses
the same explicit first, held, forward, reverse, cold and action visit protocol.
An unsupported checker/backend or interactive/runtime combination fails
explicitly.

## OpenExec reference checks

RigExec builds against [OpenExec](https://openusd.org/release/intro_to_openexec.html)
and registers computations with it, but production evaluation never runs it.
Native, frozen, frame-cache, `.rigexec` runtime and standalone evaluation all
compile and execute the shared operation graph. No environment variable or
option selects OpenExec evaluation, and no failure falls back to it. OpenExec
answers only the requests of one optional check and of tests.

### What is registered

`libs/rigExec` registers nine value types with `ExecTypeRegistry` and these
schema computations with `EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA`:

- Joint, Control and VolumeWeight: `computeRestFrame`, the default-space
  ladder (`computedDefaultSpace` and its parent, avar and posed variants, plus
  expressions on the five `*:space` matrices), `computePointFrame` and
  `computeMatrix`.
- FkChain, TwoBoneIk, BlendPointFrames, TwistDistribution, SplineIk and Ribbon:
  `computePointFrameArray`.
- Static, dynamic, volume and combine weights: `computeWeightPacket`.
  BlendInput and BlendSample: channel and sample descriptors.
- Nine built-in geometry movers (matrix, blend shape, curve, Delta Mush,
  lattice, smooth, surface, volume-correct and wrinkle):
  `computeMoverParameters` and `computeMoverStatus`.
  Affine-frame providers: expressions on `outputs:matrix`.

The registered callbacks call the same kernels as the compiled operations; for
example, the solver glue in `libs/rigExec/solverKernels.h` serves both. A
matching row shows that a registration binds the same inputs as the compiled
operation. It is not an independent check of the arithmetic.

Nothing requests the mover-parameter or affine-frame registrations.
`computeFalloffLut` is a stub that returns an empty table; a check supplies
the compiled table as an override.

### What the check judges

`rigExecPose --exec-crosscheck` enables the check on a native program. For
every compiled step, `RigExecBakedExecCheckRows::Build` records one row. These
step kinds are judged by OpenExec:

| Step kind | Requested computation |
|---|---|
| ComposeSubtree | `computePointFrame`, one row per slot |
| RestCompose | `computeRestFrame`, one row per slot |
| LadderCompose | `computedDefaultSpace`, one row per slot |
| ProviderMatrix, FrameMatrix | `computeMatrix` |
| Solve | `computePointFrameArray` |
| WeightPacket | `computeWeightPacket` |
| VolumePlacements | `computePointFrame`, converted to a volume placement |

Each judged row owns a `RigExecTapSet` with one value key. Before the
operation body runs, the row captures the values the operation read, and the
check passes them to OpenExec as value overrides, so OpenExec recomputes only
that operation. A row fails when an override for a declared input is missing
or dropped, when an override names the row's own result, when OpenExec returns
no value, or when the encoded result differs in any bit.

Other rows are not judged by OpenExec:

- Aim, rotation and parent constraints in supported configurations are judged
  by an independent C++ witness (`libs/rigExec/independentConstraintCheck.h`).
- Other constraints, every other step kind, and rows whose provider slot is
  not a first-frame pose are skip-only `NoEquivalent` rows. Blend channels and
  samples are recorded for presence only.

### Switches and coverage

- `RigExecBakedProgramTesting::EnableExecCrossCheck` builds the rows;
  `rigExecPose --exec-crosscheck` calls it. Operation inputs are captured
  only while rows exist, and the frozen snapshot does not copy them.
- The check is native-only. `rigExecPose` refuses `--exec-crosscheck` with
  `--golden-backend frozen` or `runtime`.
- CTest runs every `example_graph_*` test with
  `--exec-crosscheck --cpu-reference`. The `exec_crosscheck_*` fixtures and
  `testRigExecExecCrossCheck` carry the `exec` label (`ctest -L exec`).
- `testRigExecDefaultSpaces`, `testRigExecArm`, `testRigExecConstraints`,
  `testRigExecStandalone` and `testRigExecInteractive` use `RigExecTapSet`
  requests directly as reference values.
- `TF_DEBUG=RIGEXEC_TAP_TIMING` reports the time spent building and computing
  `ExecUsdSystem` requests.

### Stage edits and shared types

Production invalidation uses the compiled dependency edges. All
`RigExecTapSet` objects on one stage share one `ExecUsdSystem`, created on
first use. The imaging registry calls `RigExecTapSet::PrepareStageChange` on
each source-stage notice. Without tap sets this does nothing; with them, it
retires their requests before a prim removal reaches OpenExec's own listener,
which works around the `EsfUsd` resync defect described in
`tests/python/test_rigexec_stage_edits.py`.

`RigExecValueAddress` and `RigExecValueOverride` are declared in
`libs/rigExec/tapSet.h` but are plain data types. Production uses them for
interactive overrides, falloff tables and input replay, and the header includes
no OpenExec header.

`libs/rigExecStandalone/adapter.{h,cpp}` implements OpenExec's Esf scene
interfaces over `RigExecSceneDb`, but nothing calls it. The
[standalone runtime](../specs/standalone-runtime.md) lowers its scene into the
shared operation graph instead.

## Scene and binary playback

`rigExec:asset` selects a `.rigexec` asset in hosts that support binary playback.
Without that asset selection, the host evaluates the scene program. This is a
choice of input representation and playback source, not an evaluator policy
mode. Rendering the scene requires no exported bake:

```sh
USD/bin/usdrecord --renderer GL --camera /IkAsset/MainCam \
  --frames 1001:1012 docs/examples/two_bone_ik.usda frame.###.png
```

Pass an explicit camera that covers the evaluated motion.

## Exporting: a stage that needs no plugin

`rigexec.export_baked(stage, rig_paths, times, path)` is a different thing
again. It evaluates the rig at the sample times you name and writes a
**standalone standard USD file**: ordinary animated `points`, and joint,
control and revised-provider frames written as plain local
`xformOp:transform:baked` matrices. Geometry topology, materials, primvars and
metadata survive the flattened copy; joint and control prims become `Xform`,
other execution prims become inert `Scope`s, and the runtime schemas, their
properties and connections into them are removed. The result renders anywhere,
with no UsdRig plugin.

```python
import rigexec

baked = rigexec.export_baked(
    stage, ["/Character/Rig"], range(1001, 1050), "character-baked.usdc")
```

You must name **every** active `RigExecRoot` in the stage, so stripping runtime
schemas cannot silently leave another rig unevaluated. The source stage is
never authored, a destination naming a source layer is rejected, and the file is
published by atomic replacement only after every sample succeeds. Treat the
result as a sampled geometry/transform cache: it does not create a new
`UsdSkel` skinning rig, and USD's interpolation between your requested samples
is not a substitute for evaluating nonlinear rig motion at more times.

## Implementation

See [architecture](../specs/spec.md), [frame warming](frame-cache-warming.md),
[bake APIs](../specs/python-bake-inverse.md), and
`tests/testRigExecBakedSchedule.cpp`.

[Godot and baked rigs](tutorial-godot-baked-rig.md) describes a self-contained
`.rigexec` asset with embedded visuals and exposed controllers.

## OpenExec code pointers

Paths are relative to the repository root.

| Role | What | Where |
|---|---|---|
| Production graph | Shared compiler and readiness executor; OpenExec is not called | `libs/rigExecGraph/opGraph.cpp:97`, `libs/rigExecGraph/opGraph.cpp:409` |
| Build and link | `rigExec` links `exec execUsd ef vdf` publicly | `CMakeLists.txt:200-206` |
| | `rigExecStandalone` links `exec esf` | `CMakeLists.txt:227` |
| | `rigExecRuntime` links neither USD nor OpenExec | `CMakeLists.txt:324` |
| | The installed package requires the same OpenUSD targets | `cmake/rigExecConfig.cmake.in:20-25` |
| | The schema plugin's library is `rigExecImaging`; there is no `Exec` plugInfo block | `CMakeLists.txt:2317-2319` |
| | `RigExecLoadComputations()` is an empty link anchor | `libs/rigExec/types.cpp:16` |
| | `probeEsfCompatibility` is built only on request and has no test | `CMakeLists.txt:1667-1669` |
| Value types | Nine `ExecTypeRegistry::RegisterType` calls | `libs/rigExec/types.cpp:206-217` |
| Schema computations | Joint, Control and VolumeWeight providers (macros, then uses) | `libs/rigExec/computations.cpp:445-567`, `:569`, `:571`, `:595` |
| | FkChain, TwoBoneIk, BlendPointFrames, TwistDistribution, SplineIk | `libs/rigExec/computations.cpp:770`, `:957`, `:1094`, `:1157`, `:1384` |
| | Weight packets, blend descriptors and Ribbon | `libs/rigExec/moverKernels.cpp:668-889` |
| | `computeFalloffLut` stub | `libs/rigExec/moverKernels.cpp:735-738` |
| | Mover parameters and status, never requested | `libs/rigExec/movers/matrixMover.cpp:417`, `blendShapeMover.cpp:594`, `curveMover.cpp:643`, `deltaMushMover.cpp:138`, `latticeMover.cpp:264`, `smoothMover.cpp:110`, `surfaceMover.cpp:251`, `volumeCorrectMover.cpp:102`, `wrinkleMover.cpp:167` |
| | Affine-frame expressions, never requested | `libs/rigExec/affineFrameComputations.cpp:231-315` |
| | Production affine-frame operation | `libs/rigExecGraph/providerProgram.cpp:574` |
| Requests | One `ExecUsdSystem` per stage in a weak table, created only here | `libs/rigExec/tapSet.cpp:82-147`, `libs/rigExec/tapSet.cpp:108` |
| | Value keys, request build and preparation | `libs/rigExec/tapSet.cpp:213-255` |
| | Time change, overrides, compute and extraction | `libs/rigExec/tapSet.cpp:316`, `:319-340`, `:344-346`, `:350-357` |
| | `RIGEXEC_TAP_TIMING` | `libs/rigExec/debugCodes.h:19`, `libs/rigExec/tapSet.cpp:44-50` |
| | The only tap-set creator under `libs` | `libs/rigExec/bakedExecCrossCheck.cpp:50` |
| Invalidation | Registry stage-notice hook | `libs/rigExecImaging/registry.cpp:3925` |
| | Prim-removal workaround and its defect | `libs/rigExec/tapSet.cpp:118-136`, `tests/python/test_rigexec_stage_edits.py:7-12` |
| Reference check | Row construction | `libs/rigExec/bakedExecCrossCheckRows.cpp:433-587` |
| | Constraint rows and their independent witness | `libs/rigExec/bakedExecCrossCheckRows.cpp:485-522`, `libs/rigExec/independentConstraintCheck.h` |
| | Override coverage | `libs/rigExec/bakedExecCrossCheck.cpp:37-44`, `:76-86` |
| | Request evaluation and bit-exact comparison | `libs/rigExec/bakedExecCrossCheck.cpp:103`, `:114-119` |
| | Row capture around operation bodies | `libs/rigExec/bakedSchedule.cpp:2362-2364`, `libs/rigExec/bakedPose.cpp:3387`, `libs/rigExec/bakedPose.cpp:5222` |
| | Frozen clone, which omits the rows | `libs/rigExec/frozenSnapshot.cpp:89` |
| | Enable hook | `libs/rigExec/bakedExecCrossCheckRows.cpp:660-665` |
| | `rigExecPose --exec-crosscheck` and backend refusals | `tools/rigExecPose.cpp:1386`, `:1502-1508`, `:1110`, `:1119` |
| | CTest registration | `CMakeLists.txt:559-561`, `:738-742`, `:752-763` |
| Direct test references | Tap sets built by tests | `tests/testRigExecDefaultSpaces.cpp:56`, `tests/testRigExecArm.cpp:198`, `tests/testRigExecConstraints.cpp:2638`, `tests/testRigExecStandalone.cpp:542`, `tests/testRigExecStandalone.cpp:880`, `tests/testRigExecInteractive.cpp:161` |
| Esf | Adapter with no callers | `libs/rigExecStandalone/adapter.h:7-8`, `libs/rigExecStandalone/adapter.cpp:182-186` |
| | Standalone runtime lowering into the shared graph | `libs/rigExecStandalone/sceneRuntime.cpp:4-18` |
| Shared data types | `RigExecValueAddress`, `RigExecValueOverride` | `libs/rigExec/tapSet.h:41-57`, `libs/rigExec/tapSet.h:73-98` |
| | Production uses of the override type | `libs/rigExec/bakedProgram.h:396`, `libs/rigExec/frameCache.h:158` |
