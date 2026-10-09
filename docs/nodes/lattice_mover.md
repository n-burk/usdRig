# ![Lattice Mover](../../icons/lattice_mover.png) Lattice Mover

*Deforms points through a legacy cage or a regular interpolation grid.*

| | |
|---|---|
| **Node type** | `RigExecLatticeMover` |
| **Example** | [lattice_mover.usda](../examples/lattice_mover.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

![Lattice Mover effect](../gifs/lattice_mover.gif)

Free-form deformation: a native Points or mesh cage surrounds the
geometry, and posing the cage (default time is the bind, timeSamples are
the posed cage) carries the moved points through tensor-product basis
evaluation. Fewer cage points than mesh vertices drive broad, smooth
shaping — bulges, bends, squash and stretch.

Reads cage points from a native mesh/points prim and writes
exactly one exact native UsdGeomPointBased points property through
tensor-product basis evaluation (spec sections 4.1, 7.5).

## How it works

The default `rigExec:evaluation = legacy` preserves the existing
Bernstein cage evaluator. `regularGrid` interpolates cage displacement from
canonical `origin` and `spacing`, with separate linear, cardinal, B-spline or
Catmull-Rom interpolation on each axis. `divisions` uses x-fastest ordering;
single-point axes are supported. Outside coordinates retain their extrapolated
basis weights while individual cage indices clamp at the boundary.

`strength` and per-point `mask` scale the displacement before the common mover
envelope. `cageMatrix` and `targetMatrix`, optionally followed by the two
`frames` providers, define the coordinate spaces. `pointSpace = common` is for
points already in a shared asset space; `local` is for object-local points.
The cage relationship reads its selected base or final revision.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:cage` | Native Points/mesh prim supplying cage points. | yes |
| `rigExec:moves` | Exact points property to deform. | yes |

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

#### `rigExec:cage`

*Relationship.*

Native mesh/points prim supplying cage control points.

#### `rigExec:basis`

*Type:* `uniform token`. *Default:* `"bspline"`.

Valid values: `bspline`, `bernstein`.

#### `rigExec:divisions`

*Type:* `int3`. *Default:* `(2, 2, 2)`.

#### `rigExec:evaluation`

*Type:* `uniform token`. *Default:* `"legacy"`.

Valid values: `legacy`, `regularGrid`.

Legacy retains the bound Bernstein cage. Regular grid interpolates current cage displacement against canonical origin/spacing at each incoming point.

#### `rigExec:interpolationU`

*Type:* `uniform token`. *Default:* `"bspline"`.

Valid values: `bspline`, `linear`, `cardinal`, `catmullRom`.

#### `rigExec:interpolationV`

*Type:* `uniform token`. *Default:* `"bspline"`.

Valid values: `bspline`, `linear`, `cardinal`, `catmullRom`.

#### `rigExec:interpolationW`

*Type:* `uniform token`. *Default:* `"bspline"`.

Valid values: `bspline`, `linear`, `cardinal`, `catmullRom`.

#### `rigExec:origin`

*Type:* `float3`. *Default:* `(-0.5, -0.5, -0.5)`.

#### `rigExec:spacing`

*Type:* `float3`. *Default:* `(1, 1, 1)`.

#### `rigExec:strength`

*Type:* `float`. *Default:* `1`.

#### `rigExec:mask`

*Type:* `float[]`. *Default:* `[]`.

#### `rigExec:pointSpace`

*Type:* `uniform token`. *Default:* `"local"`.

Valid values: `local`, `common`.

#### `rigExec:cageMatrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `rigExec:targetMatrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `rigExec:frames`

*Relationship.*

Optional cage and target rest-to-pose providers, in that order, applied after their local-to-common matrices.

## Example

A 2×3×2 Bernstein cage bulges its middle layer outward and back,
widening the middle of a vertical strip. Bind and posed cage share the
same point ordering.

Open it live with:

```
bin/usdview.sh docs/examples/lattice_mover.usda
bin\launch_usdview.bat docs\examples\lattice_mover.usda
```

Re-render the GIF above with:

```bat
python docs/render_media.py --page lattice_mover
```

## Tips

- Hide the cage (`visibility = invisible`): it is scaffolding, not geometry.
- Follow a bulge with a Smooth mover to settle the lattice falloff or a Volume Correct mover to hold girth.
- `rigExec:basis` (`bspline` or `bernstein`) is stored on the prim and is not read. `legacy` always uses Bernstein weights. `regularGrid` uses `rigExec:interpolationU`, `rigExec:interpolationV`, and `rigExec:interpolationW`.

## See also

- [Smooth Mover](smooth_mover.md)
- [Volume Correct Mover](volume_correct_mover.md)
- [Matrix Mover](matrix_mover.md)

---

[RigExec](../index.md)
