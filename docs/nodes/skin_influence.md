# ![Skin Influence](../../icons/concept.png) Skin Influence

*Builds an explicit owner-follow and rest-to-pose skin matrix.*

| | |
|---|---|
| **Node type** | `RigExecSkinInfluence` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Use a frame proxy when the influence map includes a distinct owner and source object.

Rest-to-pose influence matrix with optional owner follow: inverseMesh * owner * inverse(sourceObject) * inverseBind * source. followOnly retains just the owner follow.

## How it works

With `fromBind`, the prefix is `inverseMesh * owner`;
otherwise it is identity. The full result is prefix times
`inverse(sourceObject) * inverseBind * source`. `followOnly` returns the
prefix alone.

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
| `rigExec:source` | Live influence frame. | yes |
| `rigExec:owner` | Live target owner frame. | no |
| `rigExec:sourceObject` | Live source object frame. | no |
| `rigExec:poseInputs` | All consumed frame providers. | yes |

## Parameters

### Node parameters

#### `outputs:matrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:inverseBind`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:inverseMesh`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:fromBind`

*Type:* `uniform bool`. *Default:* `true`.

#### `inputs:followOnly`

*Type:* `uniform bool`. *Default:* `false`.

#### `rigExec:source`

*Relationship.*

#### `rigExec:sourceObject`

*Relationship.*

#### `rigExec:owner`

*Relationship.*

#### `rigExec:poseInputs`

*Relationship.*

All frame providers read by this expression, including object, parent, source and multi-target providers. Required for pose dependency ordering and invalidation; this list does not change the numerical inputs.

## Example

With identity owner/rest matrices and a source translated two units in X, the output translates points two units in X. Connect it through a joint with identity rest to retain that explicit influence map.

## Tips

- Keep rest and pose frames distinct; connecting a live matrix to rest:space changes the rest-to-pose map.
- List every consumed provider in rigExec:poseInputs when using an affine frame expression.

## See also

- [Skin Mover](skin_mover.md)
- [Layered Skin Mover](layered_skin_mover.md)

---

[RigExec](../index.md)
