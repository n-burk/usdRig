---
title: Compression-driven wrinkles
summary: How compression and a stable fold guide produce quasistatic wrinkles, with pins, attachment bounds, and local collision planes.
order: 35
---

`RigExecWrinkleMover` adds geometric folds where a posed mesh is shorter than
its reference shape. A deterministic guide attached to the mesh's vertex
indices gives neighboring folds a consistent orientation as compression
changes. The solver adjusts edges toward that guide while limiting how far
points move from the incoming surface. Mesh topology, stiffness, pins, and
attachment distance determine the resulting folds.

Place the wrinkle mover after the geometry movers that compress the surface.
It revises their evaluated points without changing the source stage. Each
evaluation constructs its guide and constraints from the same vertex indices,
reference shape, and current pose. There are no velocities, time steps, or
previous-frame state. Playing backward or seeking directly to a pose gives
the same result as playing forward.

## Reference shape and constraints

The incoming and reference point arrays must have the same vertex order and
use target-mesh local coordinates. `inputs:restPoints` supplies a static
reference. When empty, it falls back to the target's authored base points at
the evaluation time. Supply an explicit static reference when the base points
are time-sampled and wrinkles should be measured against one fixed shape.

The solver triangulates polygon faces internally and builds unique structural
edges. Its topology setting adds links as follows:

- `cloth` adds weak constraints between opposite vertices of adjacent
  triangles to couple their folding.
- `surfaceStruts` adds constraints between vertices separated by exactly the
  specified number of edges in the mesh graph. These longer constraints couple
  larger neighborhoods and can produce broader folds. They remain within each
  connected component.

For a link joining reference points `r_i` and `r_j`, the reference distance is
`restLengthScale * length(r_j - r_i)`. A scale above one adds excess length;
a scale below one requires greater compression before that link produces a
fold. Stretched links can also pull points together. These lengths determine
fixed target edge vectors for the current pose; the solve approaches those
vectors with a configurable strength.

The public [SideFX Wrinkle Deformer documentation](https://www.sidefx.com/docs/houdini/nodes/sop/wrinkledeformer.html)
is a behavior reference for rest-distance controls, constraint neighborhoods,
pins, and tangent-plane collision. The ideas of position projection and bounded
wrinkle offsets are described in
[Position Based Dynamics](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf)
and [Wrinkle Meshes](https://matthias-research.github.io/pages/publications/wrinkleMeshes.pdf).
This implementation independently adapts those ideas using a fold guide and
affine edge constraints. The equations below describe that adaptation, rather
than the papers' nonlinear distance solve. See [method references](../references.md)
for full attribution.

## The solve

Let `p_i` be the incoming position and `x_i` the position being solved. A
deterministic hash of each vertex index initializes a scalar phase field
`phi_i`. Fixed graph-smoothing passes favor edges with less compression. This
keeps the phase more consistent along a fold and lets it vary across the
compression direction. The guide is reconstructed from the current pose,
so it needs no stored animation state.

For an edge, `e = p_j - p_i`, reference length `L`, and normalized average
incoming normal `n`, the compressed target vector is:

```text
c = max(0, 1 - dot(e, e) / (L * L))
phase = phi_j - phi_i
s = phase / sqrt(phase * phase + epsilon_phi * epsilon_phi)
h = L * c / sqrt(c + epsilon_c)
target = e + n * s * h
```

The internal regularizers `epsilon_phi` and `epsilon_c` soften phase changes
and compression onset; `epsilon_phi` scales with the phase field's range and
has a small positive floor. A constant phase field gives `s = 0`. The softened
sign `s` chooses a fold orientation without a hard sign switch. The height term approaches zero
smoothly as compression disappears. A stretched edge instead targets
`e * L / length(e)`. All targets and correction
strengths are computed before the iterative solve, including the additional
cloth or strut links.

An iteration applies a vector correction toward each fixed target:

```text
error = (x_j - x_i) - target
q = k * error / (w_i + w_j)
x_i += w_i * q
x_j -= w_j * q
```

Free vertices have `w_i = 1`; pinned vertices have `w_i = 0`. Constraints with
two pinned endpoints cannot move either endpoint. The structural strength `k`
comes from `compressionStiffness` or `stretchStiffness`, according to the
incoming edge strain. When these differ, a smooth transition near zero strain
ramps from their shared minimum toward the selected strength. The lower
strength remains unchanged, and a disabled side remains zero. This prevents
an abrupt correction change as an edge moves between compression and tension.
Additional bend and strut constraints use `bendStiffness`.
Corrections update positions immediately in
deterministic Gauss-Seidel sweeps. These strengths are relaxation controls in
`[0, 1]`, not physical material moduli; iteration count affects the result.

The fixed targets select fold orientations before relaxation. Small pose
changes therefore adjust an existing fold pattern without relying on unstable
buckling from a nearly planar starting shape. Points can move in three
dimensions, including tangentially. A mesh with no meaningful active strain
passes through unchanged.

Each point stays within `maxDisplacement` of its incoming position. This
attachment ball limits broad changes to the posed shape. With tangent-plane
collisions enabled, the offset also satisfies:

```text
dot(x_i - p_i, incomingNormal_i) >= -tangentPlaneInset
```

Area-weighted normals come from the incoming mesh and remain fixed throughout
the solve.
The inset allows inward travel up to that distance; zero permits only the
outward halfspace. Pins remain at their incoming positions.

Optional smoothing averages deformation offsets, `x_i - p_i`, over adjacent
vertices with half-strength Jacobi updates. It does not smooth the incoming
mesh itself. The final offsets are scaled by `wrinkleScale`, then pins,
attachment bounds, and tangent-plane constraints are applied again. The
mover's ordinary envelope and weight field blend the candidate revision into
its point stack.

## Controls

All parameters below use the `inputs:` namespace. Distances use mesh-local
units, so asset scale matters when choosing the attachment radius and inset.

| Parameter | Default | Meaning |
|---|---|---|
| `restPoints` | `[]` | Static reference points; empty uses the target's authored base points. |
| `iterations` | `80` | Number of constraint sweeps; more sweeps give constraints more opportunity to resolve. |
| `topology` | `cloth` | Constraint neighborhood: `cloth` or `surfaceStruts`. |
| `neighborDistance` | `2` | Graph edge distance used for surface struts. |
| `restLengthScale` | `1` | Multiplier on reference distances. |
| `stretchStiffness` | `1` | Correction strength for structural links longer than their reference length in the incoming pose. |
| `compressionStiffness` | `1` | Correction strength for compressed structural links in the incoming pose. |
| `bendStiffness` | `0.1` | Relaxation strength of the additional bend or strut constraints. |
| `maxDisplacement` | `0.2` | Maximum candidate distance from each incoming point. |
| `pinBorders` | `true` | Pins vertices on open polygon boundaries. |
| `pinPoints` | `[]` | Static additional vertex indices pinned to their incoming positions. |
| `tangentPlaneCollisions` | `true` | Prevents travel behind each point's inset tangent plane. |
| `tangentPlaneInset` | `0` | Allowed distance behind the incoming tangent plane. |
| `wrinkleScale` | `1` | Final displacement multiplier, subject to attachment and collision bounds. |
| `smoothingIterations` | `0` | Number of displacement-smoothing passes after constraint solving. |

Start with a sufficiently dense surface and a restrained attachment radius.
Pin seams or attachment points to keep the intended pose. Increase iterations
to resolve the folds further; reduce compression or bend stiffness when the
surface becomes too rigid. Use surface struts for broader neighborhoods, and
increase smoothing gradually when the output is jagged. A weight field can
fade the final effect around attachments.

## Boundaries of the method

The solver adapts the reference-length, attachment, and neighboring-constraint
ideas in *Wrinkle Meshes* to a guide with fixed vector targets. It operates on
the existing mesh topology. It does not construct the paper's separate refined
wrinkle mesh, interpolate attachments onto a coarse mesh, or retain solved
offsets from earlier frames.

- Mesh density, triangulation, vertex ordering, and constraint neighborhoods
  affect the folds. No vertices are added, and finer wrinkles need finer input
  geometry. Large strut distances increase constraint count and solve cost.
- Consistent winding is required for meaningful normals and outward collision
  planes. Coincident split vertices remain disconnected. The solver reads face
  counts and indices; `orientation` and `holeIndices` metadata do not change
  its calculation. Vertices with degenerate incoming normals remain fixed.
- Tangent planes are local travel limits. They do not detect collisions with
  other mesh regions or external geometry. There is no self-collision, VDB
  collider, volume preservation, or guaranteed nonintersection.
- Fixed edge targets approximate the missing length in compressed regions;
  exact edge-length preservation is not enforced. Targets can conflict with
  each other, pins, or attachment limits. Finite iterations also leave some
  target error.
- The guide supports coherent changes under smooth deformation with consistent
  topology and normals. Changing vertex order, topology, pins, or discrete
  settings can change the fold pattern abruptly. Degenerating or flipping
  surface normals can also interrupt continuity. There is no temporal
  relaxation, permanent creasing, or hysteresis. The smoothing control averages
  offsets; it is not the Delta Mush algorithm.
- Point arrays and numeric parameters must be finite, point correspondence
  must match, and topology and pin indices must be valid. Invalid input rejects
  the deformation without publishing a partially revised array.
