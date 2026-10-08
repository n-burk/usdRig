# ![Layered Skin Mover](../../icons/concept.png) Layered Skin Mover

*Applies masked linear or dual-quaternion skin over a point revision.*

| | |
|---|---|
| **Node type** | `RigExecLayeredSkinMover` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Supports sequential skin revisions with explicit candidate and incoming point spaces.

Masked linear or dual-quaternion skin revision. Candidate skin input may be the base or incoming points. transformInput maps incoming points before mask blending; influences produce candidate points. The common mover envelope remains independent. Binary serialization of this external revision is not supported.

## How it works

The influence palette skins incoming points, or base
points when `useBaseInput`. `transformInput` separately maps incoming points
through `rigExec:transform` before blending the candidate by `inputs:mask`.
The common mover envelope then applies independently. Empty masks mean full
strength; populated masks have one finite [0,1] value per point. Joint index
and weight arrays follow the elementSize skin layout.

The computation and handler live in core. The external revision has no binary payload
encoder; `.rigexec` export rejects it rather than silently omitting it.
Evaluation of USD stages remains supported without the converter plugin.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:moves` | One exact point3f[] points property. | yes |
| `rigExec:influences` | Native joint/control influence providers. | yes |
| `rigExec:transform` | Incoming point map when transformInput is enabled. | no |

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

#### `rigExec:influences`

*Relationship.*

#### `rigExec:jointIndices`

*Type:* `int[]`. *Default:* `[]`.

#### `rigExec:jointWeights`

*Type:* `float[]`. *Default:* `[]`.

#### `rigExec:elementSize`

*Type:* `uniform int`. *Default:* `1`.

#### `rigExec:skinningMethod`

*Type:* `uniform token`. *Default:* `"classicLinear"`.

#### `inputs:mask`

*Type:* `float[]`. *Default:* `[]`.

#### `inputs:useBaseInput`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:transformInput`

*Type:* `uniform bool`. *Default:* `false`.

#### `rigExec:transform`

*Relationship.*

## Example

Two points with a single translation influence of two units in X and masks [0.25,0.75] move by 0.5 and 1.5 units. Apply RigExecMoverAPI and bind the exact points property.

## Tips

- The per-point mask and common mover envelope are independent.
- This external revision has no USD-free binary payload encoder.

## See also

- [Skin Mover](skin_mover.md)
- [Skin Influence](skin_influence.md)

---

[UsdRig](../index.md)
