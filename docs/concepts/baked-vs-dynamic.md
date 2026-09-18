---
title: Baked and dynamic evaluation
summary: The two ways UsdRig computes a frame, how to switch between them, and what each one is for.
order: 20
---

UsdRig computes the same rig two ways. Both answer with the same numbers — that
is the whole point — but they get there differently. There is also a third
thing that sounds like the second and is not: *exporting* a bake, which
produces a file rather than a mode.

## Dynamic: the live path

Dynamic is the default and the reference. It is live OpenExec evaluation: the
composed scene plus the registered computations are **compiled** into an
execution network, a batch of requested values becomes a reusable schedule, and
each frame **evaluates** by pulling the invalid values and reusing the rest
from cache. Edit the stage and the affected part of the network is
**invalidated** and incrementally rebuilt; structural edits (adding a mover,
changing child order, changing a read phase) begin a new epoch, while ordinary
value edits do not.

This is what you get when you open a rig in `usdview` with the UsdRig imaging
plugin loaded, scrub the timeline, or drag a control. It is also what stock
`usdrecord` gets, with no bake step and no UsdRig flags:

```sh
USD/bin/usdrecord --renderer GL --camera /IkAsset/MainCam \
  --frames 1001:1012 docs/examples/two_bone_ik.usda frame.###.png
```

Pass an explicit `--camera`. Without one, `usdrecord` frames the stage's
authored bounds, which do not cover evaluated motion, so the recording can
frame empty space — every `docs/examples/*.usda` carries a `MainCam` for this.

## Baked: the step-graph program

Baked mode compiles the *same rig* into a step-graph program and runs that
instead. A frame becomes a serial prologue, a region of **steps** over dense
slots, and a serial epilogue. Each step declares the slot ranges it reads and
writes; the dependency edges are computed mechanically from those declarations,
and the graph is partitioned into **clusters** that either a serial executor
walks in program order or a parallel executor spreads across the work arena.

Three things this buys that the straight line could not:

- **Parallelism where the rig actually has it.** A geometry chain no longer
  waits for pose steps it does not depend on, two constraints on disjoint
  subtrees no longer run one after the other, and a skin revision is cut into
  contiguous vertex chunks each waiting only on the matrix slots of *its own*
  influences — the arm vertices stop waiting for the leg constraints.
- **Cone re-execution.** Source steps always run and are compared **by value**;
  a frame then runs only the forward closure ("cone") of the clusters whose
  sources actually changed, so a drag on one control re-runs what that control
  reaches rather than the program. Skipped clusters keep their slots, their
  stored diagnostics and their structural counters.
- **Deterministic ordering.** Build never measures anything, so the schedule
  does not depend on machine state, serial execution in program order is
  byte-identical to the dynamic walk by construction, and a schedule can be
  diffed between two builds of the same stage.

## Switching modes, and parity

Every evaluator starts in `dynamic`. Four things can change that, in strength
order:

1. `SetEvaluationMode` — in Python, the `evaluation_mode` property on a `Rig`,
   set to `"dynamic"`, `"baked"` or `"parity"`. Strongest.
2. `RIGEXEC_EVALUATION_MODE=baked|parity` — sets the initial mode of every
   evaluator in the process. It is read **once per process**, when the first
   evaluator is constructed; changing the environment afterwards has no effect.
3. `uniform bool rigExec:baked = true` on the `RigExecRoot` — the rig asking
   for the program itself. Weakest, consulted only where neither stronger
   request was made. `examples/biped` authors it.

`Rig.evaluation_mode_source` reports which one answered (`default`, `explicit`,
`attribute`). A rig that asked for the bake and was evaluated dynamically anyway
publishes one plain diagnostic per generation naming the first reason — news,
not a failure.

```python
import rigexec
from pxr import Usd

stage = Usd.Stage.Open("docs/examples/two_bone_ik.usda")
rig = rigexec.Rig(stage, "/IkAsset/Rig")
rig.evaluation_mode = "parity"
rig.compile()

if not rig.is_bakeable():
    print("declined:", rig.bakeability_reasons()[:3])

for frame in range(1001, 1013):
    pose = rig.evaluate(frame)
    assert not pose.baked_parity_mismatches, pose.baked_parity_mismatches
```

**Parity mode runs both paths and compares them.** The guarantee it checks is
bit-identity, not "close enough": every published value, every diagnostic *in
order*, and every compared counter of the pose must match the dynamic path, in
serial and in parallel, with and without interactive overrides. A schedule that
is faster and differs in one ulp is wrong. The same check is available from the
command line — `rigExecPose --mode parity` exits non-zero on a disagreement,
and `--mode parity --require-baked` additionally fails when the rig declines the
bake or when fewer generations came from the program than frames were asked
for.

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

## Which to use

- **Authoring and debugging a rig:** dynamic — it is the reference.
- **Playback and drags on a heavy character:** baked.
- **Anything that must not regress:** parity, in CI (`ctest -R BakedParity`
  re-runs the existing suites under the program).
- **Handing a shot to lighting, review, an engine or a machine with no
  plugin:** `export_baked`.
- **Turntables and contact sheets of a live rig:** `usdrecord`, above.

## Current limits

Not every rig bakes. `Rig.is_bakeable()` answers yes or no and
`Rig.bakeability_reasons()` gives the sentences, each naming the prim that
caused it. The families of refusal include:

- providers whose frame comes from a connected computation — a connected-space
  provider, a connected `posed:space`, or a connected `rest:space` whose
  connection ends at a computed space;
- an authored or connected `parent:space`, `parent:defaultSpace`,
  `avars:defaultSpace` or `posed:defaultSpace` on a provider, or a property
  chain that writes one of them; an intervening or animated `Xform` above a
  provider;
- a provider, solver, constraint or weight-object **type** the program does not
  yet bake (the reason names the type), and a solver that publishes guides;
- a constraint naming no target, or one whose target is not a seeded pose
  provider, or is both exec-seeded and xform-derived;
- a read phase that names a **solver** checkpoint. A checkpoint naming a
  constraint does bake; the solver form does not.

The list is a report, not a gate: a rig that declines simply evaluates
dynamically and keeps working.

## Sources

`docs/specs/baked-step-graph.md` (What was built; §1 Why; §2 Non-negotiables;
§5 Clustering and execution; §7 Cone re-execution; Phase 4 deferrals);
`docs/specs/spec.md` §6.1 (compilation, scheduling, evaluation) and §4.2
(epoch-beginning edits); `docs/specs/python-bake-inverse.md`;
`README.md` (Recording with `usdrecord`; the evaluation-mode notes);
`build/python/rigexec/bake.py`; `libs/rigExec/bakedProgram.cpp`
(`IsBakeable`); `tests/testRigExecBakedSchedule.cpp`.
