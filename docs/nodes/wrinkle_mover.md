# ![Wrinkle Mover](../../icons/concept.png) Wrinkle Mover

*Solves coherent compression-driven folds on an already deformed mesh.*

| | |
|---|---|
| **Node type** | `RigExecWrinkleMover` |
| **Example** | [wrinkle_mover.usda](../examples/wrinkle_mover.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Adds fine folds after skinning or other mesh deformation by
comparing the posed mesh with its reference lengths. A deterministic fold guide
keeps the wrinkle pattern consistent as compression changes. Each pose solves
independently, so forward playback, reverse playback, and direct frame seeking
give the same result.
See [Wrinkle deformation](../concepts/wrinkle-deformation.md) for the supported
method and [algorithm references](../references.md) for its sources.

Adds automatic quasistatic wrinkles to one native mesh points
property. Reference lengths and a deterministic fold guide define target
edge vectors. Iterative projection adjusts the incoming shape in three
dimensions, subject to attachment bounds, pins, and optional local
tangent-plane collisions. The guide follows smooth compression without
previous-frame state, so playback and seeking agree. The common mover
envelope blends the result once.

## How it works

The cloth topology builds links from a triangulated
surface and weak bending links across neighboring triangles.
Surface struts retain the structural edges and replace the bending links
with weak links at exactly neighborDistance hops through triangulated adjacency.
Vertex-index phase values are smoothed along less compressed edges to guide
the folds. Compression and the guide define fixed target edge vectors for the
current pose. Iterations adjust points in three dimensions toward those targets
while keeping them within an attachment radius of the incoming shape. Explicit and boundary
pins hold their incoming positions. Optional tangent-plane collisions restrict
inward motion relative to the incoming surface. The solved displacement can be
scaled and smoothed before the attachment bound and common mover envelope are
applied. Displacement averaging does not transport rest detail like Delta Mush.
Topology changes and degenerate surface normals can interrupt fold continuity.
There is no external-object or self-collision solve.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:moves` | One native mesh points property. | yes |

## Parameters

### Common mover envelope

#### `rigExec:moves`

*Relationship.*

Reserved write-set relationship: exact prim/property targets.

#### `inputs:enabled`

*Type:* `bool`. *Default:* `true`.

Shape-preserving enable. Disabled movers pass their preceding
revision through unchanged (value-only edit).

#### `inputs:defaultWeight`

*Type:* `float`. *Default:* `1`.

Normalized common mover envelope in [0, 1]. With no bound
rigExec:weightObject it broadcasts over every logical element: zero
passes the incoming value through bit-for-bit and one applies the
mover's full-strength result.

#### `rigExec:weightObject`

*Relationship.*

Optional compatible total weight field, at most one target.
When bound, the weight object's values (including its own sparse or
constant fallback) supply the common envelope instead of
inputs:defaultWeight. Target, domain, and cardinality must match each
mover application; an incompatible binding is a compile error. A
multi-target mover may bind only a constant operation envelope whose
weightTarget is the mover prim itself; that one value broadcasts to
every application of the atomic mover.

### Node parameters

#### `inputs:restPoints`

*Type:* `uniform point3f[]`. *Default:* `[]`.

Static, unconnected reference points in target mesh local space,
with the same ordering and count as the target. Empty uses the target's
authored base points at the evaluation time, before its mover chain.

#### `inputs:iterations`

*Type:* `int`. *Default:* `80`.

Constraint solver iteration count in [0,1000]. Zero passes through.

#### `inputs:topology`

*Type:* `uniform token`. *Default:* `"cloth"`.

Valid values: `cloth`, `surfaceStruts`.

Static, unconnected constraint topology. Both modes use
triangulated surface edges. Cloth adds weak across-triangle bend
links; surfaceStruts replaces these with weak links at exactly
neighborDistance hops through triangulated edge adjacency.

#### `inputs:neighborDistance`

*Type:* `int`. *Default:* `2`.

Exact triangulated-edge graph distance for surfaceStruts, in [1,8].

#### `inputs:restLengthScale`

*Type:* `float`. *Default:* `1`.

Finite positive scale on reference constraint lengths.

#### `inputs:stretchStiffness`

*Type:* `float`. *Default:* `1`.

Correction strength for structural links stretched in the incoming pose, finite and in [0,1].

#### `inputs:compressionStiffness`

*Type:* `float`. *Default:* `1`.

Correction strength for structural links compressed in the incoming pose, finite and in [0,1].

#### `inputs:bendStiffness`

*Type:* `float`. *Default:* `0.1`.

Bend-link and surface-strut correction strength, finite and in [0,1].

#### `inputs:maxDisplacement`

*Type:* `float`. *Default:* `0.2`.

Finite nonnegative attachment radius around each incoming point
in mesh local units. The final displacement remains inside this radius,
including after wrinkleScale and displacement smoothing.

#### `inputs:pinBorders`

*Type:* `bool`. *Default:* `true`.

Keep vertices incident to a boundary edge at their incoming position.

#### `inputs:pinPoints`

*Type:* `uniform int[]`. *Default:* `[]`.

Static, unconnected, unique nonnegative indices of additional pinned vertices.

#### `inputs:tangentPlaneCollisions`

*Type:* `bool`. *Default:* `true`.

Prevent inward movement through each incoming vertex's local
tangent plane, offset inward by tangentPlaneInset. This is a local
surface approximation; there is no external or self-collision solve.

#### `inputs:tangentPlaneInset`

*Type:* `float`. *Default:* `0`.

Finite nonnegative inward tangent-plane offset in mesh local units.

#### `inputs:wrinkleScale`

*Type:* `float`. *Default:* `1`.

Finite nonnegative scale on solved displacement, still bounded by maxDisplacement.

#### `inputs:smoothingIterations`

*Type:* `int`. *Default:* `0`.

Displacement-field averaging passes in [0,100]; not rest-detail transport.

## Example

A 33-by-17 quad sheet starts flat. An upstream matrix mover
animates its width from full size to 65% and back. The wrinkle mover responds to
that changing compression with bounded folds while keeping its border pinned.
The authored base sheet supplies rest points and the envelope stays at one.
One displacement-smoothing pass softens the transition to pinned borders.

Open it live with:

```
bin/usdview.sh docs/examples/wrinkle_mover.usda
bin\launch_usdview.bat docs\examples\wrinkle_mover.usda
```

## Tips

- Run after deformation; with the Builder, add the wrinkle mover before the earlier deformer because sibling application order is reversed.
- Mesh resolution, constraint neighborhood, and maximum displacement set the available fold shapes.
- Use cloth for across-triangle bending or surfaceStruts for weak connections at the selected edge-graph distance.
- Rest points, topology, and explicit pins must be static and unconnected. Empty rest points use the target's authored base points before its mover chain.
- The attachment radius also bounds the final result when wrinkleScale exceeds one.
- Keep vertex correspondence, topology, and normals consistent through animation.

## See also

- [Skin Mover](skin_mover.md)
- [Delta Mush Mover](delta_mush_mover.md)
- [Smooth Mover](smooth_mover.md)

---

[RigExec](../index.md)
