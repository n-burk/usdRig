# ![Auto Clavicle](../../icons/concept.png) Auto Clavicle

*Carries the shoulder with the arm's swing in FK or IK.*

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

Auto Clavicle translates the limb's root control according
to its swing. The controls and joints below that root follow the translation.
The authored swing poses determine how far the shoulder carries the arm.

Carries a limb's root with the limb's own swing, as a clavicle
lifts when the arm is raised.

Its declared frame and control dependencies determine when it runs in
the shared graph. It moves rigExec:target -- the control the limb
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

Frame and scalar reads declare dependencies in the existing
evaluation graph. The target's compose operation applies the shared numerical
kernel; there is no separate clavicle evaluation pass. Descendants of the
target are measured against its entering pose, preventing feedback from the
translation being calculated. Independent providers retain their producer
dependencies. Native, frozen and binary playback use the same swing kernel.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:target` | The one transform provider to translate. | yes |
| `rigExec:pivot`, `rigExec:anchor` | Providers defining the pivot and reference axes. | yes |
| `rigExec:fkControls` | Three controls in upper, lower and terminal order. | yes |
| `rigExec:ikTarget`, `rigExec:poleControl` | IK effector and bend-plane control. | no |
| `rigExec:ikBlendAttribute`, `rigExec:amountAttribute` | Float or double controls for blending and strength. | no |

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

The biped's `Biped_autoclav.usda` layer carries the shoulder
when its arm rises. Setting the shoulder's `avars:autoClav` to zero disables
that translation; intermediate values reduce it.

## Tips

- Use one Auto Clavicle per target provider.
- The native graph currently rejects an independently space-switched descendant used as an entering-pose input.
- Use explicit input read phases when a scalar depends on a property mover.

## See also

- [Two Bone IK](two_bone_ik.md)
- [FK Chain](fk_chain.md)
- [Space Switch](space_switch.md)

---

[UsdRig](../index.md)
