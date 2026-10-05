# Runtime pose evaluation

The runtime reads binary program tables without linking USD. Pose evaluation
is divided by responsibility:

| File | Responsibility |
| --- | --- |
| `pose.cpp` | Scratch allocation, input prologue, ladder composition |
| `poseSteps.cpp` | Step dispatch, commit deltas, propagation |
| `poseSolvers.cpp` | FK, two-bone IK, spline IK, ribbon, twist solvers |
| `poseConstraints.cpp` | Constraint sources, constraint kernels, single-chain IK |
| `poseInterpolation.cpp` | Radial-basis interpolation from solved wire tables |
| `poseMath.cpp` | Frame decomposition and blending shared by pose operations |

`poseInternal.h` contains private scratch types and phase interfaces. Keep
kernel implementations in their `.cpp` files and retain the baked path's
floating-point operation order and precision. Interpolation consumes the
already-solved weights; regularization and matrix inversion belong to baking.

The runtime pose and binary round-trip tests compare these results against the
baked evaluator. Run the repository's build helper to build and run CTest.

## Posing live

A poseable bake (`RigExecBakeOpts::overridableInputs`) also carries the
property chains as programs (`rigExecBinary/propertyChains.h`). `Execute`
computes them from the selected frame and the values `SetAvar` holds, then
hands each result to the input holders it feeds, so a face slider that only
drives chains moves the mesh. Blend channels bound to a pose interpolator read
the interpolator's live slot, as the baked gather does. Both use the
evaluator's own kernels (`rigExecMath/propertyMathKernel.h`), and
`testRigExecRuntimeLiveFace` holds the biped's runtime to the evaluator point
for point. A file without the chain section replays the recorded chain values.
`geometry.cpp` evaluates geometry revisions, including the quasistatic Wrinkle
mover, with the same pure kernels as the USD evaluator. Wrinkle supports cloth
and surface-strut constraints, point pins, tangent-plane collisions, and its
final displacement blend. Every evaluation starts from the incoming geometry;
playback does not depend on frame history. Wrinkle uses revision opcode 15;
earlier readers reject files containing that opcode. Existing revision ordinals
and binary records retain their meaning.

Plugin movers use revision opcode 16 and the ExternalMovers section. The
runtime stores their bytes and calls the kernel a host installs with
`SetExternalKernel` (see `rigExecBinary/external.h` and
[External mover plugins](../../docs/concepts/external-movers.md)). A type with
no kernel passes its points through, with a warning in every `Execute`.

The frozen frame-cache executor and the provider-only `.rigpack` runtime
remain separate subsets and reject Wrinkle movers. The `.rigexec` binary
runtime supports them.

Playback runs the steps in index order, source steps first, and never reads
their edges. `Open` therefore refuses a file whose step graph that order does
not satisfy: a predecessor at or after its step, predecessor and successor
lists that disagree, or a source step that depends on a step outside the
source pass. It also refuses a cluster graph that is not an acyclic quotient
of the step graph. It does not check that each slot a step reads has an
earlier writer. The rules live in `rigExecBinary/stepGraph.h`, which the
FlatBuffer validator shares.
