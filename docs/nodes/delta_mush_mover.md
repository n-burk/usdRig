# ![Delta Mush Mover](../../icons/concept.png) Delta Mush Mover

*Smooths deformation and restores transported rest detail.*

| | |
|---|---|
| **Node type** | `RigExecDeltaMushMover` |
| **Example** | [delta_mush_mover.usda](../examples/delta_mush_mover.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Smooths rest and incoming mesh points, then transports rest detail
into the smoothed deformed surface. Method reference: Mancewicz, Derksen,
and Wilson (2014), [Delta Mush](https://doi.org/10.1145/2614106.2614144).

Native detail-preserving smoothing. Smooth rest and incoming points using fixed rest-derived weights, then transport rest detail in surface frames. The common mover envelope blends the result once.

## How it works

The shared kernel builds edge adjacency from mesh topology and applies
the same smoothing settings to rest and incoming points. It transports the
rest-to-smoothed offset into the deformed local surface frame. Dynamic,
baked, and binary evaluation share `libs/rigExecMath/deltaMushKernel.h`.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:moves` | Mesh points property to deform. | yes |

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

*Type:* `point3f[]`.

#### `inputs:iterations`

*Type:* `int`. *Default:* `10`.

#### `inputs:step`

*Type:* `float`. *Default:* `0.5`.

#### `inputs:pinBorders`

*Type:* `bool`. *Default:* `true`.

#### `inputs:distanceWeight`

*Type:* `float`. *Default:* `0`.

#### `inputs:displacement`

*Type:* `float`. *Default:* `1`.

## Example

The animated mesh demonstrates smoothing with rest-detail restoration.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\delta_mush_mover.usda
```

## Tips

- Place after the deformation to smooth.
- Use compatible rest geometry and topology.

## See also

- [Smooth Mover](smooth_mover.md)
- [Skin Mover](skin_mover.md)

---

[UsdRig](../index.md)
