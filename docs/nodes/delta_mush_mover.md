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

Native detail-preserving smoothing. Smooth reference and incoming points, then transport reference detail in surface frames. Smoothing masks apply within every iteration; the common mover envelope blends the candidate once.

## How it works

The shared kernel builds edge adjacency from mesh topology and applies
the same smoothing settings to rest and incoming points. It transports the
rest-to-smoothed offset into the deformed local surface frame. Native,
frozen and binary evaluation share `libs/rigExecMath/deltaMushKernel.h`.

Defaults preserve the existing rest-weighted smoothing and vertex-frame
transport. Choose `smoothing = simple` or `lengthWeighted` with
`frameTransport = corner` for deformation-dependent smoothing and corner-frame
detail restoration.
`smoothWeights` participates inside every smoothing iteration, separate from
the final envelope. Explicit `edges` preserve loose edges, `onlySmooth`
disables detail restoration, and `displacement` scales restored detail.
`restPoints` stores the actual reference or saved bind coordinates.

`computationToTarget`, optionally followed by the `rigExec:frame` provider,
keeps smoothing in the original object's coordinate space under nonuniform
scale. Saved rest points stay in that computation space.

`.rigexec` exports carry these settings from format revision 21. A revision
20 file has none of them and plays the default smoothing and transport.

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

#### `inputs:smoothing`

*Type:* `uniform token`. *Default:* `"rest"`.

Valid values: `rest`, `simple`, `lengthWeighted`.

Rest uses fixed inverse-distance reference weights and step/displacement in [0,1]. Simple averages incident edge displacements. LengthWeighted recomputes current edge lengths per iteration and divides by length sum times edge valence, using twice the step. The latter modes permit any finite step and detail scale.

#### `inputs:frameTransport`

*Type:* `uniform token`. *Default:* `"vertex"`.

Valid values: `vertex`, `corner`.

Vertex uses an averaged surface normal and reference edge. Corner transports detail in each directed face-corner frame and combines posed corner-angle weights. Degenerate corners contribute zero weight.

#### `inputs:smoothWeights`

*Type:* `float[]`. *Default:* `[]`.

Empty means one at every vertex; otherwise one finite value in [0,1] per vertex, multiplying the smoothing step in both reference and incoming iterations. Distinct from the common final envelope.

#### `inputs:edges`

*Type:* `uniform int[]`. *Default:* `[]`.

Optional flattened unique undirected edge pairs, including all polygon edges and any loose edges. Empty derives polygon edges. Border pinning uses polygon incidence.

#### `inputs:onlySmooth`

*Type:* `bool`. *Default:* `false`.

Return the smoothed incoming shape without restoring detail.

#### `inputs:computationToTarget`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

Nonsingular affine row-vector map from the reference/smoothing coordinate space into the target points space. Reference points remain in computation space; incoming points transform back before smoothing. Explicit restPoints are required for a nonidentity map or frame provider, except in onlySmooth mode.

#### `rigExec:frame`

*Relationship.*

Optional single rest-to-pose frame provider composed after computationToTarget, using the declared transform read phase.

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
