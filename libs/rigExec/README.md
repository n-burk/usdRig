# Evaluator implementation

`RigExecRigEvaluator` owns the source binding epoch and compiled program.
Compilation resolves each consumer's read phase to a typed value version.
The shared operation compiler derives dependencies from those producers,
reports cycles, assigns canonical IDs and lowers safe linear clusters.

| File | Responsibility |
| --- | --- |
| `rigEvaluator.cpp` | Lifecycle, epoch settlement, generation dispatch and publication |
| `rigEvaluatorCompile.cpp` | Discovery, validation, candidate bindings and epoch replacement |
| `rigEvaluatorNotices.cpp` | Source notice classification and invalidation |
| `rigEvaluatorInputs.cpp` | Requested interactive/upstream inputs and admission |
| `bakedProgram.cpp` | Typed input binding and compiled domain records |
| `bakedPose.cpp`, `bakedProperties.cpp`, `bakedGeometry.cpp`, `bakedWeights.cpp` | Domain descriptors and pure operation bodies |
| `bakedOpGraph.cpp`, `bakedOpValues.cpp` | Native graph adapter, exact value keys and unavailable cycle outputs |
| `bakedSchedule.cpp` | Operation dispatch, timing and graph reports |
| `rigExecGraph/opGraph.cpp` | Common producer compiler, SCC reporting, clusters and readiness executor |
| `rigExecGraph/sceneProgramLowering.cpp` | Detached typed graph assembly from composed scene descriptors |

At each evaluation, the host samples source inputs before executing the
common graph. Operations wait for their declared predecessors. Exact
changes in value, type, count, validity and diagnostics determine whether
readers execute; unchanged retained outputs stop propagation. Consumer
reads select their bound versions without querying the source stage in a
computation body. Independent branches use separate output slots and join
before publication.

The evaluator reads the source stage without authoring it. Keep shared
helper declarations in private headers and local helpers in anonymous
namespaces. Run `bin/build_rigexec.bat` or `bin/build_rigexec.sh` to build
and run CTest, including invalidation and binary round-trip coverage.

## Frozen evaluation

Frozen evaluation separates source access from retained worker execution:

| File | Responsibility |
| --- | --- |
| `frozenContext.cpp` | Public frame inputs, serial scopes and purity audit |
| `frozenSampling.cpp`, `frozenPropertySampling.cpp` | Source-side input capture |
| `frozenSnapshot.cpp` | Immutable program snapshots and constant patches |
| `frozenDigest.cpp` | Input and epoch fingerprints |
| `frozenWorker.cpp` | Typed input prologue, common executor and publication |

`frozenContextInternal.h` defines worker state. Field visitors retain the
same input order for sampling, snapshots and patches. Each workspace pins
its program snapshot and owns mutable values and scratch. Workers consume
sampled values through the same domain bodies and readiness executor;
retained USD handles remain unused in worker execution.

## Independent checks

`scalarReference.cpp` and `weightReference.cpp` judge captured inputs
independently of production graph results. `bakedExecCrossCheckRows.cpp`
compares eligible operation rows with owning OpenExec requests and records
explicit reasons when no equivalent row exists; see
[OpenExec reference checks](../../docs/concepts/baked-vs-dynamic.md#openexec-reference-checks). `goldenPose.cpp` encodes
published values with exact floating-point bits; `goldenSuite.cpp` checks
complete evaluator histories. `inputReplay.cpp` records caller actions and
source edits for replay against a separately instrumented original host.
