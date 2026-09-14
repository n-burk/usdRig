# Biped — the ported Maya character

**Open `Biped_layered.usda`.** That is the whole character: skeleton, rig,
skinned mesh and materials, composed from three side layers. It is
self-contained — clone the repo and open it, nothing else to build or
fetch.

**No Maya and no template data are needed to open it.** The mesh points,
the skin weights, the materials and every rest transform are authored
directly into these files; the only asset paths in the whole stack are the
three relative sublayers below. The Maya data is where this was *derived*
from, and you only need it to regenerate.

    bin\launch_usdview.bat examples\biped\Biped_layered.usda

| file | what |
|---|---|
| `Biped_layered.usda` | **the one to open** — a root sublayering the three below |
| `Biped_layered_center.usda` | spine, neck, hips, chest, the mesh and the materials |
| `Biped_layered_left.usda` | everything on the left side |
| `Biped_layered_right.usda` | the right side, as references onto the left layer plus the ~176 attributes that genuinely differ |
| `Biped.usda` | the same rig flat, in one file, if you want to read it |
| `Biped_anim.usda` | an 8-frame animated overlay on `Biped.usda`, for timing and for testing the animated evaluate path |

`Biped_anim.usda` sublayers `Biped.usda` and keys six controls over frames
1-8. A static stage never re-reads a time sample, so it cannot show what an
animated frame costs or whether the evaluator handles a time change
correctly; that is what this overlay is for:

    build\rigExecPose examples\biped\Biped_anim.usda --frames 1,2,3,4,5,6,7,8 --joints

The layered and flat forms evaluate identically — 485 prims, 110 movers in
the same order, and all 252 joint frames matching to 0.000e+00 cm at rest
and under a test pose. `tools/biped/verify_layers.py` re-proves that on
demand.

## What is in it

252 joints reproducing Maya's rest skeleton to 4.9e-05 cm, a skinned
26,276-point body mesh with 12 UsdPreviewSurface materials (`body_geo`
carries five of them as `GeomSubset`s, since one binding cannot express
five materials over 26,274 faces), and a rig: spline-IK spine and neck,
two-bone IK/FK limbs with a per-limb switch, reverse foot, FK fingers,
twist helpers, and `hips_ctl` as the master body control.

## Driving it

Select a control and open **RigExec → Avar Editor**. The controls worth
starting with:

- `hips_ctl` — the master; moves the whole character
- `torso_ctl` — chest, neck, head and arms, not the legs
- `spine_root_ctl` / `spine_end_ctl` — hip swivel and chest; both twist the
  spine, from their own end
- `arm_?_params` / `leg_?_params` — the small yellow cubes near each wrist
  and ankle. Their `ikfk` channel is the IK/FK switch: **0 = FK, 1 = IK**,
  and the inactive control set fades as you cross. Arms ship in FK, legs in
  IK.
- `bank_?` at each ball of the foot — its `foot:roll` / `foot:bank` dials
  are the foot roll

**RigExec → Viewport Tools** gives move/rotate/scale gizmos that write
avars directly, if you would rather drag than type.

## Why it is fast

`Biped.usda` authors `uniform bool rigExec:baked = true` on its
`RigExecRoot` (and so does `Biped_layered_center.usda`, which is where the
layered variants define theirs), so opening it — here, or through
`rigExecPose` with no `--mode` — evaluates it through the BAKED PROGRAM: the
compiled epoch as a graph of steps over dense slots, with no exec round trip
per frame. It is a request and not an assertion. Every value published is the
same either way — that is proven exactly, frame by frame, by the parity
entries — so what the attribute changes is how fast the character poses, not
how it poses, and a generation the program cannot answer falls back to the
dynamic path and says so on the pose. Delete the line, or pass an explicit
`--mode dynamic`, to drive it the other way.

## Rebuilding it

These files are generated, and checked in so the character can just be
opened. To regenerate:

    bin\run_biped.bat build_biped_rigexec examples\biped\Biped.usda ^
        --spine --twist --fk-follow-parent --materials --layered ^
        --skin-rigexec <path-to>\mesh_manifest.json
    bin\run_biped.bat params examples\biped\Biped.usda --place controls
    bin\run_biped.bat split_layers examples\biped\Biped.usda ^
        examples\biped\Biped_layered.usda

The `params` and `split_layers` steps come after the build, in that order —
`params` adds the IK/FK switches and the opacity wiring, and the split has
to see them. The mesh manifest comes from `tools/biped/extract_maya_mesh.py`
run under Blender against the exported FBX.

Full guide, including what is not finished: [`docs/biped-rig.md`](../../docs/biped-rig.md).
