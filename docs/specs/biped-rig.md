# Biped guide

The biped in `examples/biped` demonstrates layered body and face rigging,
pose correctives, a control picker, and touch regions. Its eye shader is
under its own EULA; see [third-party notices](../../THIRD_PARTY_NOTICES.md).

## Open a stage

```
bin/usdview.sh examples/biped/Biped_stack.usda
bin\launch_usdview.bat examples\biped\Biped_stack.usda
```

Open `Biped_body.usda` for the body rig, or `Biped_stack.usda` for the composed
body and face layers. The [asset README](../../examples/biped/README.md)
lists the available layers and their composition.

## Controls

| Area | Controls |
|---|---|
| Root | `hips_ctl` moves the body |
| Spine and neck | Root, middle, and end controls shape the spline chains |
| Girdles | Clavicle and pelvis controls carry their limbs |
| Limbs | IK effectors and poles, plus FK controls |
| IK/FK blend | `avars:ikfk` on `arm_l_params`, `arm_r_params`, `leg_l_params`, and `leg_r_params`; 0 is FK, 1 is IK |
| Feet | Heel, toe, ball-roll, and bank controls beneath the leg IK controls |
| Face | Skull, upper/lower face, jaw, nose, eyes, brows, and mouth layers |

Switch a limb between IK and FK with its button in the Control Picker, or by
right-clicking any of its controls in usdview and choosing **Switch ... to IK**
(or **FK**). The switch first matches the half that takes over to the limb's
current joints, so the limb does not move, and the whole change is one undo
step. In animation write mode the switch keys the channels it sets.

Space switches (`avars:space`, in `Biped_spaces.usda`):

| Control | Spaces (index order) | Default |
|---|---|---|
| `L_ArmIK`, `R_ArmIK` | world, chest, head, hip_swivel, hips | world |
| `L_LegIK`, `R_LegIK` | world, hip_swivel, hips | world |
| `L_ArmPV`, `R_ArmPV` | world, chest, hand | world |
| `L_LegPV`, `R_LegPV` | world, pelvis, foot | foot |
| `L_UpArmSwing`, `R_UpArmSwing` | local, world, hips | world |
| `M_Neck`, `M_Head` | local, world, hips | local |
| `M_Look` | local, world | local |

The IK, pole and look spaces are full parent spaces. The shoulder swing, neck
and head **world** and **hips** spaces are rotation-only (`orient`): the
control keeps its rotation in that space while its position still rides on
the body. "world" is the `Aux` master. A space button in the Control Picker
switches without moving the control: it solves the control's translate and
rotate channels in the new space, and the whole change is one undo step that
keys in animation write mode.

The clavicles follow the arms (`Biped_autoclav.usda`): raising or swinging an
arm turns its shoulder point about the clavicle by a fraction of the arm's
swing, weighted by how far the arm is up, down, forward or back, so the
shoulder lifts with a raised arm. In FK the swing is read from the FK upper
arm; in IK it is estimated from the IK hand and pole. The arm itself keeps its
orientation; only its root moves, and the skinned clavicle follows. The dial is
`avars:autoClav` on `L_Shldr` and `R_Shldr`: 1 (the default) is the full
effect and 0 turns it off. A matched IK/FK switch still leaves the arm where
it is.

Use the RigExec viewport tools and graph editor to edit controls and animate
scalar channels. Keep edits in a separate layer when comparing against the
reference stage. Native transform controls, solver output joints, and
corrective-shape drivers serve different purposes; moving an output joint
does not replace the solver that drives it.

## Composition and evaluation

The layered body separates center, left, and right data. Face and corrective
layers add opinions over that body. Keep the relative file layout intact so
sublayers and asset references resolve. Picker buttons and touch regions are
USD prims and can be overridden in a stronger layer.

For command-line evaluation:

```bat
build\rigExecPose.exe examples\biped\Biped_anim.usda --frames 1,50,100,150,200 --joints
```

Conversion utilities used to produce these assets are not distributed with
the repository. The checked-in stages are the supported starting point for
this example; regenerate procedural examples through their own generators.
