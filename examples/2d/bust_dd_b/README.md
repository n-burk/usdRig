# Shion — articulated 2D character

The latest bust example is a stomach-up adult anime character with a chestnut
bob, jade eyes, a cropped teal jacket, a fitted ivory blouse and a copper ribbon.
The face, costume, arms and hands share a consistent construction and palette.
The generated rig uses stock RigExec nodes and editable control avars.
It includes its character picker, TouchPose regions and an orthographic camera.

## Open and animate

From the repository root, in Command Prompt:

```bat
bin\launch_usdview.bat examples\2d\bust_dd_b\bust_dd_b_anim.usda --camera /Shion/Camera
```

Use `bust_dd_b_rig.usda` for a neutral expression with open hands beside the
shoulders, or `bust_dd_b_sweep.usda`
for the expression and articulation demonstration. Use scene materials and
the default camera light for the painted colours. The animation is 240 frames at 30 fps; the sweep is
320 frames. Both layers animate controls only. There is no recorded dialogue;
the mouth controls provide speech articulation for animation or external input.

Open **RigExec → Animation Editors → Control Picker** to find Shion's Face, Body, Hands and Secondary
panels. Picker buttons select controls; edit their labelled channels in the
Avar Editor or use the viewport manipulator. Soft slider ranges favour normal
acting. Numeric entry can push the jaw to `-1.5` for stronger expressions.

Enable **TouchPose** to select a control by touching the character. Every
triangle belongs to exactly one region, including both sleeves and individual
fingers. The character uses one drawable mesh with separate materials and
masked deformation stacks, so TouchPose can traverse the whole character.
Controls and lattice guides are available under the `guide` purpose.

## Main controls

L/R refer to **screen left/right**, matching the older Shion example.
All controls are under `/Shion/Rig/Controls/Body_ctl`. Face controls are
nested below `Neck_ctl/Head_ctl`; arm and finger controls follow their joints.

| Control | Channel | Working range and effect |
|---|---|---|
| Body | `tx`, `rz` | sway ±2; lean ±8° |
| Neck / Head | `rz` | neck bend ±8° / head roll ±18° |
| FaceAngle | `ry`, `rx` | horizontal turn ±30° / nod ±20°, positive nod looks down |
| Torso | `ty`, `ry` | breath 0–1 / torso turn ±10° |
| Look | `tx`, `ty`, `tz` | gaze ±1 / iris contraction 0–1 |
| Eye_L / Eye_R | `ty`, `tx`, `tz` | closed −1, neutral 0, wide +1 / smile eye / relaxed lid |
| Brow_L / Brow_R | `ty`, `rz` | height ±1 / angry −20° to worried +20° |
| Mouth | `ty`, `tx` | open 0 to −1, extended to −1.5 / round −1 to wide +1 |
| Mouth | `rz`, `ry` | frown/smile ±20° / asymmetric smile ±20° |
| Teeth | `ty` | teeth reveal 0–1, with the mouth open |
| Tongue | `ty` | protrusion 0–1; use with a small mouth opening |
| Lip | `ty` | optional lower-lip fullness 0–1 |
| Shoulder_L/R | `rz` | shoulder swing, normally within ±25° |
| Elbow_L/R | `rz` | forearm bend from the presented rest pose: L −100° to +35°, R −35° to +100° |
| Hand_L/R | `rz`, `turn` | wrist bend ±55° / projected palm turn ±35° |
| Hand_L/R | `curl`, `tz` | overall hand curl 0–1 / depth −1 to +2 for overlap with hair |
| Thumb/Index/Middle/Ring/Pinky_L/R_0 | `curl` | individual offset −1 to +1 added to hand curl, clamped 0–1 |
| Each finger's `_0`, `_1`, `_2` | `rz` | individual planar joint bend/spread, normally ±25° |
| Fx | `tx`, `ty`, `tz` | blush / sweat / expression hatching |

The 20-unit head establishes the scale: each upper arm is 25 units, each
forearm 22, and each hand about 16 from wrist to middle fingertip. The
shoulders and waist are designed around those lengths. Upper and forearm
sleeve pieces overlap at the elbow with shared fabric colour; the overlap
preserves the silhouette at bent poses. Palm and finger forms are authored
in a wrist-local coordinate frame, including their curl and turn keyforms.
Open fingers have tapered segments and rounded tips. Curling exposes separate
folded finger pads across the palm; the thumb opposes over those pads.

Head turns use authored cheek, jaw and centreline landmarks, coherent eye
planes, and separate hair volumes. The far eye foreshortens without stretching
the near eye, and the bob keeps its silhouette. Turn refinements are distinct
from the expression keyforms, with target-shape and face-fold checks in the
validator.

For a pointing hand, curl the hand and offset the index finger's curl by the
opposite amount. For a raised hand in front of the hair, increase hand depth.
These are limited 2D turns with projected silhouettes and parallax; the rig
does not provide a rear view, full profile, or unrestricted 3D limb rotation.

The lower lip **tightens as the mouth opens or widens**. Jaw drop moves the
lip without increasing its thickness. `Lip.ty` explicitly adds fullness;
leave it at zero for ordinary speech. The speech presets in `acting.py` cover
small openings, A/I/U/O shapes, toothy smiles and closed-mouth pauses.

## Sources and regeneration

| File | Purpose |
|---|---|
| `build_bust.py` | USD rig, deformation stacks, texture output, animation layers |
| `design.py`, `state.py`, `art_face.py` | face construction and expression keyforms |
| `turn.py` | three-quarter silhouette, facial planes and hair projections |
| `art_body.py`, `art_limbs.py`, `art_hair.py` | coordinated costume, limbs and hair artwork |
| `interaction.py` | mesh assembly, picker, ranges, TouchPose and camera |
| `acting.py`, `perf.py` | acting poses, performance, sweep and secondary motion |
| `review_bust.py` | rendered review of actual evaluated USD geometry and textures |
| `verify_bust.py` | structural, motion, lip and dynamic/baked regression checks |

Use the repository's compatible USD Python environment, with numpy, scipy,
Pillow, OpenCV, matplotlib and numba installed. Edit the generators rather
than the generated `.usda` files or textures.

```bat
call bin\_env.bat
"%PY%" examples\2d\bust_dd_b\build_bust.py
"%PY%" examples\2d\bust_dd_b\verify_bust.py
"%PY%" examples\2d\bust_dd_b\review_bust.py --out D:\renders\shion --presets
```

The review script's CPU rasterizer uses RigExec's evaluated points, UVs and
materials. It is useful for expression and silhouette review; use the existing
`examples/2d/bust/render_bust.py --textured` for a Storm viewport render.

```bat
"%PY%" examples\2d\bust\render_bust.py examples\2d\bust_dd_b\bust_dd_b_anim.usda D:\renders\shion\storm --textured --height 70 --center 0,-20 --size 1260x980 --frames 1,44,124,168,182
```

`tests/testUsdviewShion.py` is a graphical test for picker selection,
whole-character TouchPose and live jaw edits. Run it through `testusdview`
with the neutral rig, the RigExec and TouchPose plugins, and `/Shion/Camera`.
