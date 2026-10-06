# Pip -- a rubber-hose cartoon character, rigged with RigExec

Pip is a 1930s "rubber-hose" bellhop: a big round head with pie-cut eyes, a
bean body, bendy noodle limbs, white four-finger gloves and oversized shoes.
He is **flat 2D cut-out art** -- planar meshes in the XY plane with flat
colours and ink outlines -- and he is rigged with exactly the same RigExec
schema the 252-joint 3D biped uses. No application was involved: a Python script
authors plain OpenUSD, and RigExec evaluates it live in stock `usdview` /
Hydra.

| file | what |
|---|---|
| `build_rubberhose.py` | the generator: writes both layers below, deterministically |
| `rubberhose_rig.usda` | the character at rest: geometry + rig under one `RigExecRoot` (`/Pip/Rig`) |
| `rubberhose_anim.usda` | a 120-frame, 24 fps **loop** (frame 121 == frame 1) that sublayers the rig and keys only controls, face dials, two IK length offsets and the hat hand-off |
| `render_preview.py` | offscreen Storm stills/sequences: flat unlit pass + optional lit guide pass |
| `check_limbs.py` | the **limb flip + staging detector**: evaluates the loop with `rigExecPose` and reports every change of bend side of every arm/leg, and every arm crossing the face, hugging the head outline, touching a shoe or merging with a leg (see below) |

```sh
source bin/_env.sh
"$PY" examples/2d/rubberhose/build_rubberhose.py          # regenerate both layers
bin/usdview.sh examples/2d/rubberhose/rubberhose_anim.usda # watch it live (camera /Pip/MainCam)
build/rigExecPose examples/2d/rubberhose/rubberhose_anim.usda --frames 1,57,63,70,87 --joints --targets
"$PY" examples/2d/rubberhose/check_limbs.py --plot limbs.png   # flip check, exit 1 on a problem
RIG=$(pwd) "$PY" examples/2d/rubberhose/render_preview.py \
    examples/2d/rubberhose/rubberhose_anim.usda out/ --frames 1,57,70 --guides 0.9
```

The controls are authored with `purpose = "guide"` so the clean picture
never shows them: turn on *Display -> Display Purposes -> Guide* in usdview
to see and pick them (and the joints, and the head's lattice cage).



## The rig -- common rigging concepts, RigExec schema

| rigging concept | how Pip does it | RigExec types |
|---|---|---|
| root / COG hierarchy | `Root_ctl` > `COG_ctl` > `Hips_ctl`, `Chest_ctl` > `Head_ctl` ...; nesting is the parent space | `RigExecControl` (+ `RigExecControlAPI`) |
| spline-IK spine with **squash & stretch** | three controls shape the body curve; 6 joints ride it, `rigExec:volumeWeights` bulge the body when the chest is pushed down and thin it when stretched | `RigExecSplineIk`, `RigExecJoint`, `RigExecSkinMover` (2 influences/point) |
| mid-spine follow (maintained offset) | `SpineMidFollow` is held halfway between hips and chest by a parent constraint; the animator's belly control rides it | `RigExecParentConstraint` on a control |
| **rubber-hose limbs** | a two-bone IK solves an elbow/knee; a 14-joint spline IK lays a smooth hose through shoulder -> *that elbow* -> hand; a curve mover carries the dense 48-segment strip **and its outline** through the hose frames, so the limb arcs like a hose instead of hinging | `RigExecTwoBoneIk` (joints + `upperLengthOffset`/`lowerLengthOffset`), `RigExecSplineIk`, `RigExecCurveMover` (`ribbon` mode, `primvars:st` bind) |
| stretchy noodle arms | the IK length offsets are animated: the left arm more than doubles in length to reach the hat and still arcs round the head | `rigExec:upperLengthOffset` keyed |
| pole vectors | `ArmPole_*`, `KneePole_*` pick the side the hose bows to (knees bow out); they are keyed per sample on the side of the designed elbow/knee, so they never cross a limb by accident | `rigExec:poleControl` |
| IK hands / planted IK feet | `Hand_*_ctl` (translate) with a `Glove_*_ctl` for rotation/squash; `Foot_*_ctl` under the root so feet stay planted while the COG bobs, `Shoe_*_ctl` tilts/squashes on a sole pivot | `RigExecControl`, `RigExecMatrixMover` (rigid attach) |
| **head squash / stretch / drag lattice** | a 3x3x2 Bernstein cage whose points are themselves skinned to `HeadSquash_ctl` (scale) and `HeadTop_ctl` (drag); every face part is deformed through it, read at `final` | `RigExecLatticeMover`, `RigExecSkinMover` on a `Points` cage |
| 2D head turn with parallax | `FaceSlide_ctl` slides the features across the head: nose 100 %, mouth 90 %, eyes/brows 80 %, cheeks 55 % | `RigExecMatrixMover` with `rigExec:transformSpace` (a localized cluster measured in the head's space) and a per-part envelope |
| **eye look-at** | pie-cut pupils sit on a sphere in front of each eye; aim constraints turn hidden `EyeAim_*` controls at `LookAt_ctl` and the pupils follow that local rotation | `RigExecAimConstraint`, `RigExecMatrixMover` + `transformSpace` |
| **blinks with in-betweens** | upper and lower lids collapse onto the eye rim at rest; `Blink_L/R` close them onto a curved lid line, with a half-lidded in-between at 0.5 | `RigExecBlendShapeMover`, `RigExecBlendInput`, `RigExecBlendSample` (activation 0.5 and 1) |
| **mouth shapes** | closed smile at rest; `MouthOpen` (with an in-between), `MouthOo`, `MouthWide`; tongue and ink outline are part of every shape | the same blend-shape types, plus a `Mouth_ctl` cluster |
| animator dials | `Face_ctl` carries `face:blinkL/R`, `face:mouthOpen/Oo/Wide`; each blend input's `inputs:weight` is **connected** to its dial | attribute connections |
| brows | per-brow clusters | `RigExecMatrixMover` + `transformSpace` |
| **space switch / prop hand-off** | the hat rides the head top; a parent constraint whose envelope is keyed 0 -> 1 -> 0 hands it to the left glove and back (the grab offset is authored in the shot) | `RigExecParentConstraint` (`inputs:defaultWeight`, `translationOffsets`, `rotationOffsets`) |
| draw order / ink | parts are layered by small Z offsets; every outline is geometry in the same mesh as its fill, so the same movers deform both | plain `UsdGeomMesh` + `primvars:displayColor` (uniform) |

Totals: 34 controls, 74 joints, 5 spline IKs, 4 two-bone IKs, 2 aim + 2
parent constraints, 44 mover applications (22 matrix, 10 lattice, 4 curve,
2 skin, 2 blend-shape movers + 5 blend inputs / 8 samples), 23 meshes /
6,478 points. `rigExecPose --repeat` evaluates a frame in about 2.5 ms.

## The performance (`rubberhose_anim.usda`)

| frames | beat |
|---|---|
| 1-49 | four strut beats: the body bobs with a squash on every contact, feet lift alternately (shoe tilt + landing squash); the hand on the lifted knee's side swings in across the belly (elbow out) while the other swings out and up, so both counterbalance the lift and no glove parks on the knee; glove follow-through, head/hat/head-top overlap, eye darts with a face-slide, a blink |
| 49-58 | anticipation: a deep squash (spine volume preservation, head lattice), elbows bowed wide and gloves flared out at knee height, clear of knees and shoes (the left a frame ahead and a little higher), half-shut lids, "ooh", eyes up at the hat |
| 58-64 | spring into a stretch; the left arm whips out and up, its IK lengthening so the hose arcs round the head, and grabs the hat |
| 64-85 | the hat tip: the parent constraint hands the hat to the glove, the curl standing well off the head; big grin, raised brows, the right arm out to the side in a "ta-da", a toe tap |
| 85-97 | hat back on, the head squashes under it; the right arm lands first (~89.5), the left hand lets go, springs up and out clear of the head, then drops on a wide arc and lands three to four frames later (~93) with a bigger overshoot; a blink |
| 97-121 | two more strut beats, back into frame 1 |

Everything is keyed through the rig. Body, feet, arms and face are
periodic cubic splines through keys with automatic (Catmull-Rom style,
overshoot-limited) tangents -- the strut beats are keyed per beat with the
hips leading, chest, head and head top following a few frames apart, and the
weight moving over the support foot before the other lifts; the head-top
drag, hat wobble and hose drag are damped springs driven by the motion
itself. The hand/glove channels while the hat is held are **solved from the
hat** in the generator (FK of the control hierarchy), so the attach and the
release never pop. Samples are written every fifth of a frame -- a grid
that contains every integer frame and two subframe cadences (0.8 and
0.4 frames per output frame) -- so a render never sees an interpolated IK
pose.

### Limb bend continuity (no flips)

A two-bone IK bends toward its pole; when a limb passes through straight,
or its pole crosses the root->end line, the elbow/knee snaps to the other
side. Pip's limbs are therefore *designed* per sample instead of left to the
IK: for every limb the generator decides a signed bow (a side multiplier x
the slack between hose length and chord, plus a spring-driven drag that
lags the swing), turns it into the elbow/knee point, keys the IK's two
length offsets so it solves exactly that point, and places the pole on that
side of the chord. Consequences:

* the legs never lock straight (a soft floor on the knee bow) and the knees
  always bow out; the right arm keeps its bow side through the whole loop;
* the only bend-side changes are the left arm's two **swing-throughs**
  (declared in the anim layer's `customLayerData`
  `rubberhose:intendedBendChanges`), entirely in the picture plane:
  * the launch (frames ~55-65): as the arm whips up past horizontal to curl
    round the head for the hat, one lingering S-curve -- the hose melts
    straight, runs straight through the fastest part of the sweep and
    re-curves on the other side over about four frames;
  * the release (~86-96): the curl stays round the OUTSIDE of the head
    while the hand sets the hat down and springs up and out; only once the
    hand is clear of the head (x > 3) does the hose melt, and it passes
    straight quickly (about 1.5 frames, at the falling hand's top speed of
    ~2.2 units/frame) and re-curves upward, trailing the hand, over about
    three frames. This one is a C1 Hermite S (`RELEASE_*` in the generator)
    so it never rests straight like a rigid clock hand;
* the hose's drag spring is bounded by the bow and fades out smoothly near
  a swing-through, so the rate through straight is continuous (no snap);
* through the crossing itself the IK is held a hair shorter than the chord
  (stretched, pole-independent), so sub-frame evaluation cannot flick it.

`check_limbs.py` measures, per sample and limb, the signed side of the IK's
elbow/knee and of the hose's mid-curve relative to the root->end line, the
bend angle, hose kinks and S-curves, and each hose frame's visible width
(a frame turned edge-on thins the strip to a line). It reports every sign
change with how many frames the limb takes to straighten and re-curve, and
fails on an undeclared flip, a flip faster than 3 frames, a kink, a pinched
or edge-on strip, or an acceleration spike (a pop; declared foot contacts
excepted).

It then checks **staging** on the deformed meshes (a second evaluation with
`rigExecPose --pose-out`; every shape is the convex hull of its evaluated
points, ink included; hose joints under a glove or epaulette are hidden):
an arm whose ink overlaps the head (an arm cut across the face), a hose
running along the head outline with less than 0.3 units of background for
more than 2 frames (a tangent), a glove touching a shoe, and 4+ arm joints
within 0.12 of the same side's leg (arm and leg merging into one black
shape).

The current loop passes all of it: the launch swing-through takes 4.2-4.6
frames and the release 3.2-3.6, no other sign change on any limb; the
arms never cross the head (closest ink gap +0.09), the hat curl stands
at least 0.35 off the head through the hold (it comes within 0.15 only for
about 1.4 frames as the hand arrives on the hat, and within 0.24 as it sets
it down), the gloves stay at least 0.18 from the shoes and at most 2 arm
joints come near a leg. The previous version of the loop, run through the
same check, fails with the hat arm drawn across the face on the release,
tangents all through the hold, the gloves on the shoes in the squash and in
the strut, and the arms merged with the legs in the squash.

## Things this example shows about RigExec

- **Solver reads are positional.** The hose spline IK reads the two-bone
  IK's elbow joint as its `midControl`, so its prim is authored *above* the
  two-bone IK under `Solvers` (bottom sibling runs first). Author it below
  and the hose reads the elbow at rest and stays straight.
- **Localized clusters** (`rigExec:transformSpace`) are what let face parts
  take their local motion in rest space and then go through the head
  lattice exactly once.
- **Deformed cages**: the lattice cage is an ordinary `Points` prim with a
  skin mover on it; the lattice movers read it with
  `rigExecReadPhase = "final"` on `rigExec:cage`.
- **Constraints on controls**: aim and parent constraints may revise
  controls (`EyeAim_*`, `SpineMidFollow`, `Hat_ctl`); movers read them at
  `final`.
