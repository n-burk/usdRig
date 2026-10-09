# ![Bone Frame](../../icons/concept.png) Bone Frame

*Evaluates joint channels with explicit scale and location inheritance.*

| | |
|---|---|
| **Node type** | `RigExecBoneFrame` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Separate the rotation/scale parent map from the location map when a joint inherits them differently.

Joint channels with separate rotation/scale and location inheritance. Channel rotations are XYZ degrees; matrices use row vectors. The object provider places the evaluated pose in target space.

## How it works

XYZ channel rotations are degrees. `inputs:local` and
`inputs:parentRest` are saved joint-space transforms; `rigExec:sourceObject`
places the evaluated frame in target space. `inputs:inheritScale` is FULL,
NONE, AVERAGE, ALIGNED, FIX_SHEAR, or NONE_LEGACY. Connected joints suppress
channel translation. `inputs:spaceKind` is `pose` (the full map), `rotation`,
or `translation`.

Connect `outputs:matrix` to a joint or control's
`posed:space`. Declare every frame provider the expression reads in
`rigExec:poseInputs`, including object frames, parents and multi-target inputs.
The relationship supplies pose dependency ordering and invalidation; the
specific source/parent relationships supply numerical inputs. Keep the
provider's rest frame separate from its evaluated pose.

These computations live in the shared runtime. A connected pose
expression can make an epoch ineligible for the baked program; ordinary
runtime evaluation then follows the existing dynamic fallback. This does not
provide USD-free binary serialization of the expression graph.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:parent` | Live parent joint frame. | no |
| `rigExec:sourceObject` | Live object frame. | no |
| `rigExec:poseInputs` | All consumed frame providers. | yes |

## Parameters

### Node parameters

#### `inputs:spaceKind`

*Type:* `uniform token`. *Default:* `"pose"`.

#### `outputs:matrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:local`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:parentRest`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:hasParent`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:inheritRotation`

*Type:* `uniform bool`. *Default:* `true`.

#### `inputs:localLocation`

*Type:* `uniform bool`. *Default:* `true`.

#### `inputs:connected`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:inheritScale`

*Type:* `uniform token`. *Default:* `"FULL"`.

#### `inputs:tx`

*Type:* `double`. *Default:* `0`.

#### `inputs:ty`

*Type:* `double`. *Default:* `0`.

#### `inputs:tz`

*Type:* `double`. *Default:* `0`.

#### `inputs:rx`

*Type:* `double`. *Default:* `0`.

#### `inputs:ry`

*Type:* `double`. *Default:* `0`.

#### `inputs:rz`

*Type:* `double`. *Default:* `0`.

#### `inputs:sx`

*Type:* `double`. *Default:* `1`.

#### `inputs:sy`

*Type:* `double`. *Default:* `1`.

#### `inputs:sz`

*Type:* `double`. *Default:* `1`.

#### `rigExec:parent`

*Relationship.*

#### `rigExec:sourceObject`

*Relationship.*

#### `rigExec:poseInputs`

*Relationship.*

All frame providers read by this expression, including object, parent, source and multi-target providers. Required for pose dependency ordering and invalidation; this list does not change the numerical inputs.

## Example

Set local translation to (0,1,0), tx to 2, and sourceObject to an object translated 2 in X. With no parent, the full pose origin is (4,1,0). Connect the output to a joint's posed:space.

## Tips

- Keep rest and pose frames distinct; connecting a live matrix to rest:space changes the rest-to-pose map.
- List every consumed provider in rigExec:poseInputs when using an affine frame expression.

## See also

- [Joint](joint.md)
- [Constraint Frame](constraint_frame.md)

---

[RigExec](../index.md)
