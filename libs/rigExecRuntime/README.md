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
