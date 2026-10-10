# ![Copy Frame](../../icons/concept.png) Copy Frame

*Copies a live frame, optionally retaining incoming translation.*

| | |
|---|---|
| **Node type** | `RigExecCopyFrame` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Use a source frame as the output while preserving location independently when required.

Copies a source provider frame, optionally preserving the incoming translation.

## How it works

`preserveLocation` copies the incoming translation over the source matrix. Connect `outputs:matrix` to a joint or control's
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
| `rigExec:poseInputs` | All consumed frame providers. | yes |

## Parameters

### Node parameters

#### `outputs:matrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:incoming`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:preserveLocation`

*Type:* `uniform bool`. *Default:* `false`.

#### `rigExec:source`

*Relationship.*

#### `rigExec:poseInputs`

*Relationship.*

All frame providers read by this expression, including object, parent, source and multi-target providers. Required for pose dependency ordering and invalidation; this list does not change the numerical inputs.

## Example

Wire a source at (2,0,0). An incoming matrix at (0,3,0) with preserveLocation enabled keeps origin (0,3,0) while copying the source orientation and scale.

## Tips

- Keep rest and pose frames distinct; connecting a live matrix to rest:space changes the rest-to-pose map.
- List every consumed provider in rigExec:poseInputs when using an affine frame expression.

## See also

- [Mapped Frame](mapped_frame.md)

---

[UsdRig](../index.md)
