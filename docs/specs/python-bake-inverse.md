# Python bake and inverse APIs

`rigexec.export_baked(stage, rig_paths, times, path)` evaluates a stage into a
separate standard USD file. Pass every active `RigExecRoot` path and the exact
sample times required by the consumer. `None` means default time; numeric times
are nonnegative frame numbers, matching the Python evaluator's convention.

```python
import rigexec

baked = rigexec.export_baked(
    stage, ["/Character/Rig"], range(1001, 1050), "character-baked.usdc")
```

The exporter flattens the composed scene, preserves ordinary geometry,
topology, materials and primvars, and writes final moved properties at each
requested sample. Joint/control frames and revised standard transform providers
become ordinary local `xformOp:transform:baked` matrices. It converts asset-space
frames against the baked parent at that time, so nested transforms, asset
placement and original reset stacks retain their evaluated world placement.
Joint/control prims become `Xform`; other execution prims become inert `Scope`
nodes. Runtime schemas, their properties and connections into those properties
are removed. The result has no execution bindings or sublayer dependency and
can be read without the RigExec plugin.

The source stage is never authored, and the exporter rejects a destination that
names any source layer. Conflicting outputs from different rigs, failed
evaluations, non-finite frames and singular parent transforms stop the export.
Only after all samples succeed does an atomic file replacement publish the
destination. This is a sampled geometry/transform cache: it does not create a
new `UsdSkel` skinning rig, and USD interpolation between requested samples is
not a substitute for evaluating nonlinear rig motion at additional times.

`rigexec.solve_parameters` is a pure numeric boundary for inverse controllers.
The caller supplies `forward(parameters)` returning a flat vector of selected
landmark coordinates and must keep that callback free of scene authoring and
hidden simulation state. The immutable input tuple can drive a standalone math
kernel or an evaluation adapter with explicit transient overrides. The function
never changes channels or commits a result to USD.

```python
import math
from rigexec import solve_parameters

def endpoint(angles):
    shoulder, elbow = angles
    return (math.cos(shoulder) + math.cos(shoulder + elbow),
            math.sin(shoulder) + math.sin(shoulder + elbow))

result = solve_parameters(
    endpoint, initial=(0.0, 0.5), desired=(1.1, 1.2),
    bounds=((-math.pi, math.pi), (0.0, math.pi)))
if result.converged:
    solved_angles = result.parameters
```

Optional `jacobian(parameters)` supplies output rows by parameter columns.
Without it, the explicit prototype callback is differentiated with central
finite differences clipped to parameter bounds. `weights` multiply individual
residual components. `regularization` adds a squared-distance penalty from
`reference`, which defaults to the initial parameters. Levenberg damping keeps
rank-deficient normal equations solvable and increases when a step fails to
improve the objective. No third-party numeric package is required.

`InverseResult` records parameters, output, convergence reason, iteration and
forward-evaluation counts, weighted residual norm and objective. Success means
the requested target tolerance was reached (by default `1e-5 * character_scale`).
Unreachable limits, stationarity, insufficient relative improvement, iteration
exhaustion and non-finite callback results return explicit unsuccessful
results. Invalid arguments and callback exceptions raise. This dense solver is
for small channel subsets; Jacobian evaluation scales with the parameter count
and the linear solve is cubic in that count. Landmark selection, analytic
controller inverses, channel adapters, gestures, commits and undo remain caller
responsibilities.

Regression coverage is in `testRigExecBake` and `testRigExecInverse`; default
space/twist semantics and gizmo composition are covered by
`testRigExecDefaultSpaces` and `testGizmoMath`.

`rigexec.create_curvenet_weight(stage, path, curvenet, mesh, weights,
auto_smooth=())` authors the native `RigExecCurvenetWeight` schema. It accepts
source paths, USD prim/schema objects or RigExec handles, validates one finite
weight per control-pool point and any auto-smoothing indices, then connects the
mesh points/topology and curvenet points/spline indices through exact-property
relationships. Basis and sampling attributes remain connected to the net;
editing them or the authored weights updates the runtime's existing dependency
graph. The helper returns a strict `SchemaPrim` that can be bound as an
operation's `rigExec:weightObject`. The default range policy is `clamp`.
