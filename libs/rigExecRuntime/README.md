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
