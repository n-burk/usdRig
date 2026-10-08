# ![Constraint Frame](../../icons/concept.png) Constraint Frame

*Computes ordered affine constraints in explicit owner and target spaces.*

| | |
|---|---|
| **Node type** | `RigExecConstraintFrame` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Connect each result to the next incoming matrix to author an ordered constraint stack.

Ordered affine constraint computation. Incoming and provider frames are row-vector matrices. Owner and target spaces select world, object, local joint or custom coordinates. Operation and mapping attributes specify the explicit numerical contract.

## How it works

Operations include COPY_LOCATION, COPY_ROTATION,
COPY_SCALE, COPY_TRANSFORMS, ARMATURE, ARMATURE_BLEND, DAMPED_TRACK,
STRETCH_TO, PRESERVE_ORIGIN, the supported rotation-normalization operation
LIMIT_ROTATION, and TRANSFORM_LOCATION. Copy and mapping operations use WORLD,
POSE, LOCAL, LOCAL_OWNER_ORIENT or CUSTOM spaces as appropriate; local joint
conversion uses the explicit rest and inheritance inputs. Influence blends
in world space after the operation using affine stretch and quaternion
rotation interpolation.

TRANSFORM_LOCATION maps LOCATION, SCALE or XYZ Euler rotation (radians) into
location. Per-axis source ranges clamp unless `mapExtrapolate`; zero-width
ranges contribute zero. `mapAxes` selects normalized source axes for each
output axis. `mapMix` supports ADD or REPLACE. Other rotation orders and
rotation/scale output mappings are not provided by this operation.

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
| `inputs:incoming` | Previous world-space matrix. | yes |
| `rigExec:source` | Live source frame where used. | no |
| `rigExec:targets`, `rigExec:targetObjects` | Frame lists for ARMATURE_BLEND. | no |
| `rigExec:poseInputs` | All consumed frame providers. | yes |

## Parameters

### Node parameters

#### `outputs:matrix`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:incoming`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:origin`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:inverseBind`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:targetIndices`

*Type:* `int[]`. *Default:* `[]`.

#### `inputs:objectIndices`

*Type:* `int[]`. *Default:* `[]`.

#### `inputs:targetBinds`

*Type:* `matrix4d[]`. *Default:* `[]`.

#### `inputs:targetWeights`

*Type:* `double[]`. *Default:* `[]`.

#### `inputs:pivot`

*Type:* `double3`. *Default:* `(0,0,0)`.

#### `inputs:dualQuaternion`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:currentPivot`

*Type:* `uniform bool`. *Default:* `false`.

#### `rigExec:targets`

*Relationship.*

#### `rigExec:targetObjects`

*Relationship.*

#### `inputs:operation`

*Type:* `uniform token`. *Default:* `"COPY_LOCATION"`.

#### `inputs:influence`

*Type:* `double`. *Default:* `1`.

#### `inputs:ownerSpace`

*Type:* `uniform token`. *Default:* `"WORLD"`.

#### `inputs:targetSpace`

*Type:* `uniform token`. *Default:* `"WORLD"`.

#### `inputs:axisMask`

*Type:* `uniform int`. *Default:* `7`.

#### `inputs:invertMask`

*Type:* `uniform int`. *Default:* `0`.

#### `inputs:offset`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:uniformScale`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:scaleAdd`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:power`

*Type:* `double`. *Default:* `1`.

#### `inputs:rotationMix`

*Type:* `uniform token`. *Default:* `"REPLACE"`.

#### `inputs:mapFrom`

*Type:* `uniform token`. *Default:* `"LOCATION"`.

#### `inputs:mapFromMin`

*Type:* `double3`. *Default:* `(0,0,0)`.

#### `inputs:mapFromMax`

*Type:* `double3`. *Default:* `(1,1,1)`.

#### `inputs:mapToMin`

*Type:* `double3`. *Default:* `(0,0,0)`.

#### `inputs:mapToMax`

*Type:* `double3`. *Default:* `(1,1,1)`.

#### `inputs:mapAxes`

*Type:* `uniform int3`. *Default:* `(0,1,2)`.

#### `inputs:mapExtrapolate`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:mapMix`

*Type:* `uniform token`. *Default:* `"ADD"`.

#### `inputs:removeTargetShear`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:trackAxis`

*Type:* `uniform token`. *Default:* `"TRACK_Y"`.

#### `inputs:keepAxis`

*Type:* `uniform token`. *Default:* `"PLANE_X"`.

#### `inputs:volume`

*Type:* `uniform token`. *Default:* `"NO_VOLUME"`.

#### `inputs:restLength`

*Type:* `double`. *Default:* `1`.

#### `inputs:bulge`

*Type:* `double`. *Default:* `1`.

#### `inputs:bulgeMin`

*Type:* `double`. *Default:* `0`.

#### `inputs:bulgeMax`

*Type:* `double`. *Default:* `1`.

#### `inputs:bulgeSmooth`

*Type:* `double`. *Default:* `0`.

#### `inputs:useBulgeMin`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:useBulgeMax`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:targetOffset`

*Type:* `double3`. *Default:* `(0,0,0)`.

#### `rigExec:source`

*Relationship.*

#### `rigExec:sourceObject`

*Relationship.*

#### `rigExec:ownerObject`

*Relationship.*

#### `rigExec:customSpace`

*Relationship.*

#### `inputs:ownerLocal`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:ownerRest`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:ownerParentRest`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:ownerHasParent`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:ownerInheritRotation`

*Type:* `uniform bool`. *Default:* `true`.

#### `inputs:ownerLocalLocation`

*Type:* `uniform bool`. *Default:* `true`.

#### `inputs:ownerInheritScale`

*Type:* `uniform token`. *Default:* `"FULL"`.

#### `rigExec:ownerParent`

*Relationship.*

#### `inputs:sourceLocal`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:sourceRest`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:sourceParentRest`

*Type:* `matrix4d`. *Default:* `((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))`.

#### `inputs:sourceHasParent`

*Type:* `uniform bool`. *Default:* `false`.

#### `inputs:sourceInheritRotation`

*Type:* `uniform bool`. *Default:* `true`.

#### `inputs:sourceLocalLocation`

*Type:* `uniform bool`. *Default:* `true`.

#### `inputs:sourceInheritScale`

*Type:* `uniform token`. *Default:* `"FULL"`.

#### `rigExec:sourceParent`

*Relationship.*

#### `rigExec:poseInputs`

*Relationship.*

All frame providers read by this expression, including object, parent, source and multi-target providers. Required for pose dependency ordering and invalidation; this list does not change the numerical inputs.

## Example

TRANSFORM_LOCATION maps source X rotation from 0 to pi/3 radians onto output Z from 0 to 0.02. mapAxes=(0,0,0), mapMix=ADD and no extrapolation give 0.01 Z at 30 degrees and clamp at 0.02 beyond 60 degrees.

## Tips

- Keep rest and pose frames distinct; connecting a live matrix to rest:space changes the rest-to-pose map.
- List every consumed provider in rigExec:poseInputs when using an affine frame expression.

## See also

- [Bone Frame](bone_frame.md)
- [Armature Parent](armature_parent.md)

---

[UsdRig](../index.md)
