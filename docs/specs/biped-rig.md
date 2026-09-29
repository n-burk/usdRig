# Biped guide

The biped in `examples/biped` demonstrates layered body and face rigging,
pose correctives, a control picker, and touch regions. It remains checked in.
Its source and redistribution license are unresolved; see
[third-party notices](../../THIRD_PARTY_NOTICES.md#provenance-requiring-owner-review).

## Open a stage

```bat
bin\launch_usdview.bat examples\biped\Biped_everything.usda
```

On Linux or macOS, use `bin/usdview.sh` with the same stage path. Open
`Biped_layered.usda` for the body rig, or `Biped_stack.usda` for the composed
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
