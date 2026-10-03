# Evaluator implementation

`RigExecRigEvaluator` owns the binding epoch and generation caches. Its public
API is in `rigEvaluator.h`; implementation files divide the work by responsibility.
The `rigEvaluator*` helper headers are private implementation details.

| File | Responsibility |
| --- | --- |
| `rigEvaluator.cpp` | Lifecycle, evaluation-mode selection, baked fallback, generation dispatch |
| `rigEvaluatorCompile.cpp` | Compile attempts, epoch replacement, request preparation, failed-compile memo |
| `rigEvaluatorValidation.cpp` | Mover discovery, scalar connections, weight-domain validation |
| `rigEvaluatorDependencies.cpp` | Rig discovery, pose-input closures, solver-input index |
| `rigEvaluatorChainPlan.cpp` | Chain dependency order, parallel levels, revision snapshots |
| `rigEvaluatorDigest.cpp` | Structural fingerprints and digest settlement |
| `rigEvaluatorNotices.cpp` | USD notice classification and epoch invalidation |
| `rigEvaluatorInputs.cpp` | Shared reads, interactive overrides, value-cache invalidation |
| `rigEvaluatorProperties.cpp` | Property-chain compilation, binding, evaluation |
| `rigEvaluatorPose.cpp` | Constraint dispatch, pose interpolators, rest-frame composition |
| `rigEvaluatorDynamic.cpp` | Solver and constraint execution, pose publication |
| `rigEvaluatorGeometry.cpp` | Weight resolution, skin inputs, scalar geometry oracle |
| `rigEvaluatorGeometryEvaluation.cpp` | Persistent geometry graphs, parallel chain execution, geometry publication |

Compilation discovers and validates inputs, builds candidate bindings and
schedules, prepares requests, then installs the epoch. A failed phase returns
the diagnostic and operation paths through `_CompileFailure`; the attempt owns
cleanup and diagnostic publication. Candidate tables stay local until commit.

A dynamic generation settles the epoch, resolves properties and interactive
overrides, executes the pose schedule, and publishes the final pose. Pose
interpolators then publish weights before `_EvaluateGeometry` consumes the
snapshot and pose results. Independent geometry chains write separate work
buffers, which merge in dependency order.

Keep shared helper declarations in the private headers and single-file helpers
in anonymous namespaces. Evaluation reads the source stage without authoring it.
Run `bin/build_rigexec.bat` (Windows) or `bin/build_rigexec.sh` to build and run
CTest, including dynamic/baked parity, invalidation, and round-trip coverage.

## Frozen evaluation

Frozen evaluation separates stage access from worker execution:

| File | Responsibility |
| --- | --- |
| `frozenContext.cpp` | Public entry points, frame-input containers, serial scopes, purity audit |
| `frozenSampling.cpp` | Stage-side frame sampling and burst caches |
| `frozenPropertySampling.cpp` | Stage-side property-chain discovery, binding, and evaluation |
| `frozenSnapshot.cpp` | Program snapshots and constant patching |
| `frozenDigest.cpp` | Frame-input and epoch-constant fingerprints |
| `frozenWorker.cpp` | Worker prologue, execution, publication, partial-cone restoration |
| `frozenGeometry.cpp` | Worker-side weight and geometry assembly from sampled values |

`frozenContextInternal.h` declares the shared worker state and helpers. Its
field visitors define the input order once for sampling, snapshot capture, and
worker patching. Workers use sampled values and private state; copied USD
handles must remain unused during execution.
