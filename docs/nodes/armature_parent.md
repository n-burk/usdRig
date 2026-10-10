# ![Armature Parent](../../icons/concept.png) Armature Parent

*Applies a source rest-to-pose map over an incoming owner frame.*

| | |
|---|---|
| **Node type** | `RigExecArmatureParent` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Retains owner channels while a source influence moves the owner.

Applies a source rest-to-pose map over incoming channels or a supplied incoming frame, optionally retaining its translation.

## How it works

The incoming frame is either `inputs:incoming` or XYZ
channels times `local` times the parent frame. The source map is
`inverse(sourceObject) * inverseBind * source`. `preserveLocation` restores
the incoming translation after applying that map.

Connect `outputs:matrix` to a joint or control's
`posed:space`. Declare every frame provider the expression reads in
`rigExec:poseInputs`, including object frames, parents and multi-target inputs.
The relationship supplies pose dependency ordering and invalidation; the
specific source/parent relationships supply numerical inputs. Keep the
provider's rest frame separate from its evaluated pose.

These computations are operations in the shared evaluation graph: native
evaluation and frame-cache workers run the same stage-free kernels from the
provider's declared inputs, so editing an input re-runs the providers that
read it and their dependents.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:source` | Live source frame. | yes |
| `rigExec:parent` | Live parent frame. | no |
| `rigExec:sourceObject` | Live source object frame. | no |
| `rigExec:poseInputs` | All consumed frame providers. | yes |

## Parameters

### Node parameters

#### `outputs:matrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:local`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:inverseBind`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:incoming`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:useIncoming`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:preserveLocation`

*Type:* `uniform bool`. *Default:* `false`.

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

#### `rigExec:source`

*Relationship.*

#### `rigExec:sourceObject`

*Relationship.*

#### `rigExec:poseInputs`

*Relationship.*

All frame providers read by this expression, including object, parent, source and multi-target providers. Required for pose dependency ordering and invalidation; this list does not change the numerical inputs.

## Example

An incoming owner at (0,3,0), identity inverseBind/sourceObject and a source at (2,0,0) produce (2,3,0). Enable useIncoming to bypass the channel composition.

## Tips

- Keep rest and pose frames distinct; connecting a live matrix to rest:space changes the rest-to-pose map.
- List every consumed provider in rigExec:poseInputs when using an affine frame expression.

## See also

- [Constraint Frame](constraint_frame.md)

---

[UsdRig](../index.md)
