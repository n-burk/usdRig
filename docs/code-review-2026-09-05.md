# Interactive evaluation review — 2026-09-05

The review started from `main` at `c92c040`. The working changes address live
rest edits, reusable execution graphs, dependencies between operator domains,
and schema features that previously advertised behavior without implementing
it. This report describes the current changes, not the original baseline's
remaining-feature list. Final integrated validation: **Release build succeeded;
all 35 CTest suites passed** with `bin\build_rigexec.bat` on Windows against the
unchanged OpenUSD 26.08 installation.

## Correctness and execution fixes

| Priority | Finding and current change | Regression coverage |
|---|---|---|
| P1 | Point graphs and executors were reconstructed on every pull. Target graphs now retain sources, revision parameters, schedules and checkpoints. Only changed packets invalidate downstream revisions; the CPU parity oracle is opt-in. | `testRigExecMoverGraph`: every implemented operation, status toggles, long connected inputs, 2,048-revision dirty suffix. `testRigExecInteractive`: unchanged pulls, value edits and independent branches. |
| P1 | Structural edits discarded all point execution state. Stable revisions now support dependency reconnection, insertion, removal and reordering while preserving unaffected target state. Tap requests share a stage context instead of duplicating complete execution systems per request. | `TestStructuralSplices`, shared-context and deletion coverage in `testRigExecInteractive`; stage-edit Python suite. |
| P1 | The installed OpenUSD resync listener applied a prim predicate to deleted prims. The usdRig-owned tap context now releases requests before its execution system when a removal notice arrives, ahead of the problematic Esf listener, and recreates the system lazily. Extracted snapshots survive. No external OpenUSD source was changed. | Prim-removal and repeated-edit tests. This workaround recreates the shared OpenExec system on deletion; it does not promise zero rebuild work for every structural edit. |
| P1 | IK did not consume complete bound-joint rest frames, and repeated rest edits could become stale after overrides. Solvers consume declared rest dependencies, including optional `rigExec:restJoints`; affected levels are dirtied by transitive input notices and prerequisite values. | Two-bone rest-origin/orientation and repeated-edit tests in `testRigExecConstraints`. |
| P1 | Separate solver and constraint passes could consume stale upstream poses. A unified dependency DAG orders constraints, controls, joints and solver aggregates, including namespace ancestry and connected matrix-space providers. Dirty levels retain successful snapshots; final solver guides use resolved overrides. | Deep dependency chains, constraint→control→IK, ancestor dependencies and cycle cases in `testRigExecConstraints`. |
| P1 | Failed imaging activation changed notice ordering, and external read dependencies did not republish. Activation now preserves the retained listener until commit; transitive read regions select affected rig sessions. | Activation rollback, external reads/rewiring, unrelated-edit and multi-session selectivity tests in `testRigExecImaging`. |
| P2 | Kernel failures could look like successful pass-through. Revision result statuses now carry execution-time validation failures alongside atomic preceding-value fallback. | Invalid kernel packets, cardinality changes and status recovery tests in `testRigExecMoverGraph`. |
| P1 | Geometry constraints published directly over the final point chain, dropping earlier deformers and other constraints on the same points. Their solved deltas now form matrix revisions in the ordinary point chain, keyed by mover identity. Local deltas use the correct row-vector matrix order, and invalid solved frames fail atomically. | `TestGeometryConstraintsCompose`: matrix→constraint→constraint, no-op pull, last/middle edits, zero envelopes, disabling, invalid output and recovery. `TestTransformAndGeometrySpellingsAgree` compares rotated and scaled target spaces; scalar chord-envelope parity is covered in `testRigExecConstraints`. |
| P1 | An invalid rest/default/parent frame could become identity in a matrix-space expression and yield a falsely valid descendant pose. Existing invalid frames now retain their failure through space conversion; missing ancestors still select identity. | Invalid parent rest edit, descendant failure, and recovery in `testRigExecDefaultSpaces`. |

## Completed schema and API surfaces

- **Default spaces and twist:** the six `default:*` scalars, `default:space`,
  `avars:defaultSpace`, `posed:defaultSpace`, `parent:space/defaultSpace` and
  `avars:unitScaleFactor` have live computations. Connected identity is
  authoritative; authored identity selects the documented fallback. Gizmo
  composition and translation writes follow the same units and matrix order.
  `inputs:twistTurns` supplies signed, animatable winding. See
  [default-space formulas](xformable-default-spaces.md) and
  `testRigExecDefaultSpaces`/`testGizmoMath`.
- **Blend shapes:** sample-specific base/preceding/final dependencies replace
  the earlier blanket rejection. Multi-target applications resolve their own
  target-relative packets. Optional surface-frame offsets use area-weighted
  vertex normals and a corresponding rest/posed incident edge; orthonormal
  transport preserves offset magnitude and fails atomically for invalid
  nonzero-offset frames. Coverage is in `testRigExecInteractive` and
  `TestSurfaceOffsets` in `testRigExecMath`.
- **Curvenets:** native `RigExecCurvenetWeight` consumes exact mesh/net property
  dependencies and reuses a bounded cache of geometry factorizations while
  weights animate. Auto-smoothing, parameter-field evaluation, adjustments,
  adjuster superposition and posed-net guide publication have implementation
  and dedicated tests in the current changes. Adjustment controls expose their
  full evaluated frames to gizmos, with scope and units preserved. Unsupported
  uses of these later point-graph frames as earlier pose inputs fail compilation
  explicitly, including indirect dependencies. The Python
  `create_curvenet_weight` helper validates authoring and connects basis/sample
  inputs to the source net. See the curvenet math, adjustment, imaging and
  schema-authoring suites.
- **Baked export:** `export_baked` writes final properties and local transforms
  to a separate flattened standard USD file, strips execution schemas and
  protects source layers. Tests cover nested providers, reset stacks, asset
  scale, default and numeric samples, conflicting ownership, destination
  preservation on failure and repeated export while an older stage stays open.
  The last case exposed a cached-layer bug; the destination layer is now
  reloaded after atomic file replacement.
- **Inverse boundary:** `solve_parameters` accepts a pure forward callback,
  optional analytic Jacobian, bounded finite differences, weights and reference
  regularization. Results distinguish reaching the target from stagnation,
  unreachable limits, non-finite values and iteration exhaustion. It performs
  no USD authoring. See [Python API documentation](python-bake-inverse.md),
  `testRigExecBake` and `testRigExecInverse`.
- **Imaging:** requested samples now include transforms and other published
  output domains rather than only point arrays. Authored reset-stack boundaries
  stop inherited driven-transform deltas, including during motion sampling.
  Volume guides consume moved
  dimensions, posed curvenet guides follow evaluated points, and edits select
  the affected active sessions. Coverage includes `TestCompleteMotionPublication`
  and weight-overlay tests. This is snapshot/data-source coverage; a fresh
  visual usdview acceptance pass is not claimed.
- **Standalone providers and packs:** an independent Esf adapter evaluates
  supported registered providers from an owned typed scene database. Pack
  export/load preserves exact resolved values, blocks, array types and sample
  identities; the runtime retains no source USD stage. Missing direct source
  values and unsupported capabilities produce explicit failures. Raw attributes
  on RigExec prims follow the same block rules as ordinary source attributes;
  inactive connection sources select the destination's authored fallback.
  Interned schema identities remain stable across runtime destruction and
  recreation, matching the lifetime of OpenExec's definition cache. This is a
  provider-only implementation, with its supported contracts documented in
  [standalone runtime](standalone-runtime.md) and [pack format](standalone-pack.md).

## Remaining limits and pending work

- Dirty propagation is at source/revision granularity. Per-point sparse dirty
  masks are not implemented, and nonlocal kernels would need additional rules.
- Retained checkpoints consume memory proportional to the sum of revision
  output sizes. No production memory budget or eviction policy is established.
- The evaluator still scans chains and assembles/compares parameter packets.
  Property-math orchestration, output-map assembly and override delivery also
  have costs beyond dirty kernel execution. The entire frame is not claimed
  to cost only the dirty suffix.
- Structural reconciliation still inspects the affected authored structure;
  prim deletion recreates the shared OpenExec system as described above.
- Curvenet factor-cache lookup compares geometry/layout arrays, and the cache
  is bounded by entry count rather than bytes. Large-scene timing and memory
  qualification remain necessary.
- Adjustment control frames are produced in the point graph. Feeding those
  computed frames into earlier pose operators or MatrixMover inputs is rejected;
  a general dependency graph spanning both evaluation domains is not supplied.
  Reading an adjustment's authored scalar channels remains supported.
- Bake requires every active rig to be selected and explicit sample times.
  It exports a geometry/transform cache, not a new UsdSkel skinning rig.
  Nonlinear rig behavior between samples requires appropriate sampling.
- Numeric inverse solving uses dense normal equations for small selected
  channel subsets. Full-rig automatic landmark/channel discovery, editing
  adapters and manipulation UI are not supplied by this pure API.
- The experimental provider-only [Esf/rigpack path](standalone-pack.md)
  executes registered provider computations against copied resolved data with
  no live USD stage. Whole-rig mover lowering, deployment profiles and resource
  closure packaging remain outside its explicit version-1 provider scope.

## Validation record

`bin\build_rigexec.bat` configured and built the combined changes in Release
mode, then passed **35/35 CTest suites** (13.37 seconds of test execution),
including the final standalone block/fallback and schema-identity regressions.
Coverage includes retained graph splicing and deletion, selective solver
dependencies, repeated rest edits, geometry constraint composition,
surface-frame detail, default spaces, schema authoring, curvenet weights and
adjustments, imaging, gizmos, bake/inverse, and the independent standalone/pack
boundary. `git diff --check` also passed. Tests establish functional regressions;
they do not establish production-scale latency, memory limits, every rendering
delegate's motion behavior or visual host acceptance.
