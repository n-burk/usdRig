# ![Mapped Frame](../../icons/concept.png) Mapped Frame

*Transfers source rest-to-pose motion onto a target rest frame.*

| | |
|---|---|
| **Node type** | `RigExecMappedFrame` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

A saved source/target rest pair makes the mapping explicit.

Maps a live source pose onto a target rest frame: targetRest * inverse(sourceRest) * source.

## How it works

The row-vector result is `targetRest * inverse(sourceRest) * source`. Connect `outputs:matrix` to a joint or control's
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
| `rigExec:source` | Live source frame. | yes |
| `rigExec:poseInputs` | All consumed frame providers. | yes |

## Parameters

### Node parameters

#### `outputs:matrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:targetRest`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:sourceRest`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `rigExec:source`

*Relationship.*

#### `rigExec:poseInputs`

*Relationship.*

All frame providers read by this expression, including object, parent, source and multi-target providers. Required for pose dependency ordering and invalidation; this list does not change the numerical inputs.

## Example

An identity sourceRest, a targetRest translated one unit in Y, and a live source translated two units in X produce origin (2,1,0).

## Tips

- Keep rest and pose frames distinct; connecting a live matrix to rest:space changes the rest-to-pose map.
- List every consumed provider in rigExec:poseInputs when using an affine frame expression.

## See also

- [Copy Frame](copy_frame.md)

---

[RigExec](../index.md)
