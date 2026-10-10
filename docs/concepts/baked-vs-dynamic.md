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
captured original inputs. Exact golden verification compares published values,
status and ordered semantic diagnostics against independent original captures;
work counters are outside that value contract. CPU point agreement retains its
existing numerical comparison policy and is separate from exact golden checks.

The golden tool supports `--golden-backend native`, `frozen`, or `runtime`.
Runtime visits also require `--golden-program file.rigexec`. Each backend uses
the same explicit first, held, forward, reverse, cold and action visit protocol.
An unsupported checker/backend or interactive/runtime combination fails
explicitly.

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
