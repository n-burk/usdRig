# ![Surface Binding Mover](../../icons/concept.png) Surface Binding Mover

*Follows fixed barycentric attachments with vector offsets.*

| | |
|---|---|
| **Node type** | `RigExecSurfaceBindingMover` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Each affected point blends one or more persistent surface bindings.
The mover stores the binding indices, generalized barycentric weights and
three-component offsets as attributes. It reads the driver's selected point
revision and never searches again for a nearest point. Offsets rotate in a
local tangent, bitangent, normal frame.

Persistent generalized barycentric attachments. Each affected vertex
blends one or more bindings. Offsets are vectors in the current surface's
tangent, bitangent, normal frame. Polygon bindings and optional limit
position/derivative stencils are immutable authored attributes; evaluation
only reads the selected phase of the driver points.

## How it works

`vertexOffsets` groups bindings by affected vertex;
`polygonOffsets` groups `pointIndices` and `barycentricWeights` by binding.
`bindingWeights` combines their positions. Polygon mode uses a Newell normal
and the first nonzero polygon edge as its tangent. Smooth normal mode uses
area-weighted vertex normals interpolated by the binding weights, with the
tangent projected into that normal's plane. Degenerate frames contribute
zero in their collapsed offset directions. `deltaMultiplier` and optional
per-target-point `deltaMultipliers` scale the transported three-component
offset: zero binds directly to the surface, one preserves the saved offset,
and negative finite values are allowed. These are independent of the envelope. Optional masks and strength blend
the resulting position over the incoming point revision.

Limit mode reads authored, factored control-point position and u/v derivative
stencils (`limitOffsets`, `limitIndices`, `limitWeights`, `limitDuWeights`,
`limitDvWeights`). These stencils must come from the chosen subdivision surface
and parameter coordinates; ordinary polygon weights are not limit stencils.
Geometric limit normals come from the derivative cross product. Topology edits
require rebinding and regenerating limit stencils.

With optional `frames`, `surfaceRestMatrix` times the first provider's
rest-to-pose map converts driver points back to surface-local coordinates;
`surfaceToBinding` maps them into binding space. `targetRestMatrix` times the
second provider's map returns bound positions to target point space. Without
frames, the authored matrices alone perform this conversion. Evaluation never
writes to the stage. Dynamic and baked evaluation use the same immutable
payload; binary `.rigexec` export currently rejects this mover because it has
no external payload encoder.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:surface` | Driver point-based prim; final phase follows its completed modifier chain. | yes |
| `rigExec:moves` | One exact native point3f[] points property. | yes |
| `rigExec:frames` | Optional surface and target rest-to-pose providers, in that order. | no |

## Parameters

### Node parameters

#### `rigExec:vertices`

*Type:* `int[]`. *Default:* `[]`.

#### `rigExec:vertexOffsets`

*Type:* `int[]`. *Default:* `[0]`.

#### `rigExec:polygonOffsets`

*Type:* `int[]`. *Default:* `[0]`.

#### `rigExec:pointIndices`

*Type:* `int[]`. *Default:* `[]`.

#### `rigExec:barycentricWeights`

*Type:* `float[]`. *Default:* `[]`.

#### `rigExec:offsets`

*Type:* `vector3f[]`. *Default:* `[]`.

#### `rigExec:bindingWeights`

*Type:* `float[]`. *Default:* `[]`.

#### `rigExec:deltaMultiplier`

*Type:* `float`. *Default:* `1`.

Finite signed scale applied only to transported binding offsets; zero attaches to the surface and one preserves the offsets.

#### `rigExec:deltaMultipliers`

*Type:* `float[]`. *Default:* `[]`.

Optional finite signed offset multipliers indexed by target vertex; empty supplies one at every vertex.

#### `rigExec:mask`

*Type:* `float[]`. *Default:* `[]`.

#### `rigExec:strength`

*Type:* `float`. *Default:* `1`.

#### `rigExec:surfaceMode`

*Type:* `uniform token`. *Default:* `"polygon"`.

Valid values: `polygon`, `limit`.

#### `rigExec:normalMode`

*Type:* `uniform token`. *Default:* `"geometric"`.

Valid values: `geometric`, `smooth`.

#### `rigExec:limitOffsets`

*Type:* `int[]`. *Default:* `[0]`.

#### `rigExec:limitIndices`

*Type:* `int[]`. *Default:* `[]`.

#### `rigExec:limitWeights`

*Type:* `float[]`. *Default:* `[]`.

#### `rigExec:limitDuWeights`

*Type:* `float[]`. *Default:* `[]`.

#### `rigExec:limitDvWeights`

*Type:* `float[]`. *Default:* `[]`.

#### `rigExec:frames`

*Relationship.*

Optional surface and target rest-to-pose providers, in that order.

#### `rigExec:surfaceRestMatrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `rigExec:targetRestMatrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `rigExec:surfaceToBinding`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

## Example

For one triangle binding, author `vertices = [0]`,
`vertexOffsets = [0, 1]`, `polygonOffsets = [0, 3]`,
`pointIndices = [0, 1, 2]`, `barycentricWeights = [0.2, 0.3, 0.5]`,
`bindingWeights = [1]` and `offsets = [(0.1, 0.2, 0.4)]`.
Wire `rigExec:surface` to the driver mesh with a final read phase and
`rigExec:moves` to the driven points property. Apply `RigExecMoverAPI`.

## Tips

- Use polygon/geometric settings for barycentric bindings on the control mesh.
- Limit mode requires position and derivative stencils; missing arrays are diagnosed.
- Surface binding does not replace modifiers that deform the driver cage.

## See also

- [Surface Mover](surface_mover.md)
- [Skin Mover](skin_mover.md)

---

[UsdRig](../index.md)
