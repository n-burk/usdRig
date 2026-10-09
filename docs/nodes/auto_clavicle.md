# ![Auto Clavicle](../../icons/concept.png) Auto Clavicle

*Translates a limb-root control as the limb swings, turning it about the clavicle.*

| | |
|---|---|
| **Node type** | `RigExecAutoClavicle` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Carries a limb's root with the limb's own swing, the way a clavicle
lifts when the arm is raised. It runs with the space switches: after the
controls compose and before any solver reads them. It moves
`rigExec:target` by translation only, and that control's namespace
descendants follow, so the FK chain and the IK root move together and keep
their orientation.

The biped authors one per shoulder in `examples/biped/Biped_autoclav.usda`.

Carries a limb's root with the limb's own swing, as a clavicle
lifts when the arm is raised.

It runs where space switches run: after the controls compose and before
any solver reads them. It moves rigExec:target -- the control the limb
hangs from -- by translation only, and its namespace descendants follow,
so the FK chain and the root of the IK solve move together and keep
their orientation.

Each frame, in row-vector convention and the anchor's axes:

d    = the upper limb's direction: the first FK control's rest bone
   axis carried by its posed frame, or the upper bone of a
   two-bone solve from the target's moved origin to
   rigExec:ikTarget, bending toward rigExec:poleControl,
   blended by rigExec:ikBlendAttribute
p_k  = the pose weights of d, measured as the swing that takes the
   basis X axis onto d (rigExec:basis), against
   rigExec:poseRotations
w    = clamp(inputs:gain * amount * sum_k gain_k * clamp(p_k, 0, 1),
         0, 1)
R    = slerp(identity, shortest arc from the rest direction to d, w)

and the target's origin is turned by R about rigExec:pivot's origin. The
rest direction and the bone lengths are read from the default frames, so
at rest R is the identity and the node moves nothing.

The FK direction reads only the first FK control's rotation, which the
translation does not change. The IK root moves with the shift it helps
decide, so the solve and the shift are iterated to their fixed point;
there an IK limb matched to an FK one gives the FK direction back, and a
matched IK/FK switch moves nothing.

## How it works

Each frame, in row-vector convention and the anchor's axes, the limb
direction is the first FK control's rest bone axis carried by its posed
frame, or an estimated two-bone reach from the target's moved origin to
`rigExec:ikTarget`, bending toward `rigExec:poleControl`. Those two
directions blend by `rigExec:ikBlendAttribute` against `rigExec:ikValue`.
Pose weights of that direction, measured as the swing that takes the basis
X axis onto it, are summed with `rigExec:poseGains` and scaled by
`inputs:gain` and the optional amount attribute. The target's origin then
turns by that weight about `rigExec:pivot`.

The rest direction and the bone lengths come from the default frames, so
at rest the node moves nothing. The IK root moves with the shift it helps
decide, so the solve and the shift iterate to a fixed point. A matched
IK/FK pair therefore does not jump when the blend switches.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:target` | The limb-root control that is translated. Exactly one. | yes |
| `rigExec:pivot` | The clavicle control whose posed origin the root turns about. Exactly one. | yes |
| `rigExec:anchor` | The control the limb direction is measured in. It must not be moved by the target. Exactly one. | yes |
| `rigExec:fkControls` | The FK chain's three controls, root to end. | yes |
| `rigExec:ikTarget` | Optional prim the IK half reaches for. Without it the FK direction is used. | no |
| `rigExec:poleControl` | Optional pole the IK estimate bends toward. | no |
| `rigExec:ikBlendAttribute` | Optional float or double selecting FK versus IK. Absent reads as FK. | no |
| `rigExec:amountAttribute` | Optional float or double scaling the effect. Absent reads as 1. | no |

## Parameters

### Node parameters

#### `rigExec:target`

*Relationship.*

The RigExecControl moved: the limb's root control, whose
descendants hold the FK chain and whose frame roots the IK solve.
Exactly one.

#### `rigExec:pivot`

*Relationship.*

The control whose posed origin the root turns about: the
clavicle. Exactly one.

#### `rigExec:anchor`

*Relationship.*

The control the limb's direction is measured in: the chest.
It must not be moved by rigExec:target. Exactly one.

#### `rigExec:fkControls`

*Relationship.*

The FK chain's three controls, root to end. The first gives
the FK direction; the rest frames of all three give the bone axis,
the bone lengths of the IK estimate and the rest root.

#### `rigExec:ikTarget`

*Relationship.*

Optional: the prim whose posed origin the IK half reaches
for, normally the IK effector. Without it the FK direction is used
whatever the blend says.

#### `rigExec:poleControl`

*Relationship.*

Optional: the pole the IK estimate bends toward. Without it
the estimate bends toward the FK direction.

#### `rigExec:ikBlendAttribute`

*Relationship.*

Optional float or double property selecting between the FK
and IK directions, normally the limb's IK/FK switch. Absent reads as
FK.

#### `rigExec:ikValue`

*Type:* `double`. *Default:* `1`.

The value of rigExec:ikBlendAttribute that means fully IK;
1 - ikValue means fully FK, and values between blend the two
directions.

#### `rigExec:amountAttribute`

*Relationship.*

Optional float or double property scaling the effect, the
animator's on/off dial. Absent reads as 1.

#### `inputs:gain`

*Type:* `double`. *Default:* `0.4`.

Scales the summed pose weights into the blend weight w.

#### `rigExec:basis`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

The frame rigExec:poseRotations are expressed in, relative
to the anchor's posed frame: its X axis is the limb axis at a zero
rotation. Only its rotation is read, and it may carry a reflection,
which is how a right limb shares a left limb's poses.

#### `rigExec:poseRotations`

*Type:* `quatf[]`. *Default:* `[]`.

Swing poses of the limb, each a rotation of the basis X axis,
in rigExec:basis. Only the swing about X is measured.

#### `rigExec:poseFalloffs`

*Type:* `float[]`. *Default:* `[]`.

Per-pose falloff, parallel to rigExec:poseRotations, stated
relative to each pose's nearest neighbour as RigExecPose's is.

#### `rigExec:poseGains`

*Type:* `double[]`. *Default:* `[]`.

Per-pose contribution to the blend weight, parallel to
rigExec:poseRotations. A pose that only shapes the interpolation
carries 0.

#### `rigExec:kernel`

*Type:* `uniform token`. *Default:* `"gaussian"`.

Valid values: `gaussian`, `linear`.

The interpolation kernel, as RigExecPoseInterpolator's.

#### `rigExec:regularization`

*Type:* `float`. *Default:* `0`.

Added to the interpolation matrix diagonal, as RigExecPoseInterpolator's.

## Example

Open `examples/biped/Biped_autoclav.usda` over the biped stack. Each
shoulder is a `RigExecAutoClavicle`: `rigExec:target` is the upper-arm
swing control, `rigExec:pivot` is the shoulder, and `avars:autoClav` on
the shoulder is the amount dial (1 by default). Dropping the sublayer
removes the prims and that channel.

## Tips

- `inputs:gain` defaults to 0.4 and scales the summed pose weights.
- `rigExec:kernel` is `gaussian` or `linear`, the same choice as a pose interpolator.
- `rigExec:basis` may carry a reflection so a right limb can share a left limb's poses. Only its rotation is read.
- A pose with gain 0 still shapes the interpolation and does not add lift.

## See also

- [Two-Bone IK](two_bone_ik.md)
- [FK Chain](fk_chain.md)
- [Space Switch](space_switch.md)
- [Pose Interpolator](pose_interpolator.md)

---

[RigExec](../index.md)
