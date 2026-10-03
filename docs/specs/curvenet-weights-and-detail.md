# Curvenet weights and surface detail

`RigExecCurvenetWeight` interpolates scalar weights authored on a curvenet's
control pool onto a native mesh. It evaluates centripetal Catmull–Rom or cubic
Bezier sampling stencils, projects samples to mesh triangles, and minimizes
`xᵀ L x + κ ‖B x − S w‖²`, with `κ = 100 * meanEdgeLength`.
This follows the parametrization construction in the
[2026 Pixar talk](https://research.pixar.com/docs/2026.SiggraphTalks.TSG.pdf).
The pure math API accepts multiple weight columns against one factorization.

```python
import rigexec

field = rigexec.create_curvenet_weight(
    stage, "/Character/Rig/ProfileWeights", "/Character/ProfileNet",
    "/Character/Body", weights, auto_smooth=[2, 5])
mover.set_relationship("rigExec:weightObject", [field.path])
field.set_attribute("inputs:weights", edited_weights)
```

There is one input weight per control-pool point. Selected `auto_smooth`
indices are harmonic unknowns along the control net; their input values are
ignored by the solve. Each unknown component needs at least one authored
weight anchor. Mesh components reached by no sample use the explicit
`rigExec:unreachedValue`, default zero. The default `clamp` policy bounds the
result to `[0, 1]`; `strict` rejects any out-of-range result.

The five relationships name exact native properties: `rigExec:weightTarget`
names the destination mesh's `points`, `rigExec:meshFaceCounts` and
`rigExec:meshFaceIndices` name its topology, and `rigExec:curvenetPoints` and
`rigExec:curvenetSplineIndices` name one matching curvenet's pool and layout.
Compilation rejects mismatched owners and bare prim paths. The authoring helper
also connects basis and sampling settings to the curvenet.

Geometry, layout, sampling, and smoothing membership key a bounded, shared
cache of 32 factorizations. Weight-only edits reuse those factors. Authored
geometry edits create a new binding on demand while retaining the surrounding
execution graph. The field reads native authored geometry at the requested
time; it does not implicitly bind the final output of a mover chain. Failed
geometry or weight validation yields an invalid weight packet and an atomic
failed-mover pass-through.

`RigExecBlendShapeMover.rigExec:deltaSpace` controls how sculpt offsets combine
with an earlier mesh deformation:

| Value | Behavior |
|---|---|
| `target` (default) | Add target-space offsets to preceding points. |
| `surfaceFrame` | Rotate rest offsets into corresponding preceding-surface vertex frames. |

Surface frames use area-weighted vertex normals and the longest projected
rest boundary edge, with stable vertex-index ties. The posed frame uses that
same edge, so stretch cannot switch the tangent choice. Frames are orthonormal:
offset magnitude remains unchanged by surface stretch. This defines the
rotation-only detail convention explicitly. Mesh topology must match the
point arrays; a degenerate frame carrying a nonzero offset fails atomically.

Place the blend mover after the Profile Mover in the resolved operation order.
The Profile Mover's cached output supplies the preceding surface, so a sculpt
or activation edit executes the blend suffix without repeating the profile
deformation. Blend sample read phases remain independent of `deltaSpace`.

Regression coverage includes Catmull–Rom connectivity and exact sampling
stencils, constant fields, partition of unity, auto-smoothing anchors, live
weight edits, exact-property rejection/recovery, rotated and stretched detail,
and cached blend updates (`testRigExecCurvenet`, `testRigExecMath`, and
`testRigExecInteractive`).
