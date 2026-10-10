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

`geometry.cpp` evaluates geometry revisions, including the quasistatic Wrinkle
mover, with the same pure kernels as the USD evaluator. Wrinkle supports cloth
and surface-strut constraints, point pins, tangent-plane collisions, and its
final displacement blend. Every evaluation starts from the incoming geometry;
playback does not depend on frame history. Wrinkle uses revision opcode 15;
earlier readers reject files containing that opcode. Existing revision ordinals
and binary records retain their meaning.

Delta Mush, lattice and surface revisions also read the settings format 21
adds, through their movers' path reads and leaf sites like every other mover
input: smoothing and frame transport, smoothing weights, explicit edges,
smoothing-only output and a computation space; regular-grid lattices with
their interpolation, origin, spacing, strength, mask and cage and target
spaces; surface snap modes, offset, mask, explicit triangles and surface and
target spaces. Their frame providers are the revision's influences. Default
settings run the legacy kernels; a format-20 file holds none of them and
plays the legacy deformers.

Plugin movers use revision opcode 16 and the file's `external_movers` table. The
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
source pass. It also refuses a read of a slot that no step running before
the reader writes, except in the domains a run fills before any step (avars,
property-chain results, chain bases and solver points), and a cluster graph
that is not an acyclic quotient of the step graph. The retired `Snapshots`
domain and `SnapshotFinals` step kind are refused outright, ahead of every
rule about what a step of a given kind reads or writes. The rules live in
`rigExecBinary/stepGraph.h`, which the FlatBuffer validator shares.

Array inputs (`int[]`, `float[]`, `double[]`, `float2[]` and `float3[]`
attributes the file lists) are set with `SetInputArray` and read back with
`GetInputArrayAt`. The reader copies the elements; a set keeps the default's
element count. An authored set reaches every read of the attribute, the
Default-time ones included. `SetSampledInputArrayAt` takes a stage's own
value at a sampled time, of any count: only the reads at the evaluation time
take it, and each reader judges the count as the evaluators do. Topology is
the exception: a skin's joint indices, a mesh's face counts and indices, a
curve's order and knots and a sparse blend shape's offsets and point indices
are epoch state, which a reader never recompiles, so a sampled set of
another count is refused with the reason. A fixed skin
layout's arrays, a chain's base points and admitted weight-oracle points
read through such inputs. Structural painted arrays and excluded point reads
keep private storage slots, inaccessible to the public input APIs; `ResetInput` returns each to the
file's value, and a skin layout to the one `Open` expanded unless the layout
standing equals it by value, which stays, as the evaluator's layout op keeps
it. The input sampler does not sample array inputs.
