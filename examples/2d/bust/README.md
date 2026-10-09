# 2D mesh-deform bust (layered mesh deformation, rigged with RigExec)

A front-facing anime bust -- back hair, neck, sailor-collar blouse with a bow,
face, ears, layered eyes (whites, irises, pupils, highlights, lids, lashes),
brows, nose, mouth (inner mouth, teeth, tongue, lip lines), a seven-strand
fringe, side locks, an ahoge and a star hair clip -- built as flat, layered
art meshes and rigged **entirely with stock RigExec prims**. No application, no
bake: every frame is evaluated live by RigExec in stock `usdview` / Hydra.

It is the 2D counterpart of the 3D biped: the same rigging vocabulary --
FK chains, skinning, blend shapes with in-betweens, lattices, driven keys,
pose-space interpolation -- drives the parameter/keyform/warp-deformer style
of animation that VTuber models use.

| file | what |
|---|---|
| `build_bust.py` | the generator: writes the three layers below. Deterministic; edit this, not the `.usda` |
| `bust_geo.py` | the art: outlines, triangulation, line art, vertex colours; every feature is a function of its parameters with a fixed topology |
| `bust_rig.usda` | the character at rest: 8 art meshes (16.2k points) + the whole rig |
| `bust_anim.usda` | a 180-frame (6 s @ 30 fps) **looping** performance, sublayering the rig |
| `bust_sweep.usda` | a 120-frame parameter sweep (the rig-reveal demo), sublayering the rig |
| `render_bust.py` | offscreen Storm renders: flat unlit art pass + lit guide pass |

```bash
source bin/_env.sh
"$PY" examples/2d/bust/build_bust.py                 # regenerate the layers
build/rigExecPose.exe examples/2d/bust/bust_anim.usda --frames 1,30,60,90,120,150 --targets
bin/usdview.sh examples/2d/bust/bust_anim.usda       # Display > Purposes > Guide shows the rig
RIG=$(pwd) "$PY" examples/2d/bust/render_bust.py examples/2d/bust/bust_anim.usda out/bust \
    --frames 1,30,86,110 --center 0,-6.5 [--guides 1.0] [--wire]
```

View it in usdview with lighting off (or through `render_bust.py`): the
colours are authored as display values, per vertex, so soft gradients
(blush, iris, hair shading) follow the deformation.

## Rigging concepts and the schema types that implement them

| rigging concept (layered mesh deformation term) | RigExec implementation |
|---|---|
| **Parameters** (ParamAngleX/Y/Z, EyeOpen, EyeBall, Brow, MouthForm, Breath, BodyAngle) | `RigExecControl` handles; the avars ARE the parameter values (degrees or slider units). Face handles are children of the head handle, so they ride the tilt |
| **Keyforms** (the art sculpted at a parameter value) | `RigExecBlendSample` at an activation IN PARAMETER UNITS -- `Turn_R` has keyforms at 15 and 30 degrees, `Blink` an in-between at 0.5 and the closed lid at 1. Offsets stored sparse as `UsdSkelBlendShape` |
| **Parameter -> keyform interpolation** | `RigExecBlendInput` + `RigExecBlendShapeMover`: the channel interpolates its keyforms by weight and holds past the last one |
| **"One parameter drives many deformers"** (driven keys) | the channel's `inputs:weight` is CONNECTED to the avar. Where the opposite side needs the parameter negated (`Turn_L`, `Blink`, `Look_L`, `BrowDown` ...), two `RigExecFloatMathMover`s on the weight read the avar through a connected `inputs:value` (`add`) and flip it (`multiply -1`) |
| **Warp deformer** (head warp with angle keyforms) | `RigExecLatticeMover` on every head layer, all reading ONE 7x9x2 Bernstein cage at the `final` phase. The cage's own keyforms (`Turn_L/R`, `Nod_Up/Down`) are a `RigExecBlendShapeMover` on the cage points |
| **Parallax / depth layers** | the art's z is its draw order AND its depth inside the cage. The cage's z axis spans back hair (z=-2) to fringe/clip (z~1.6), so the one warp moves every layer by a different amount -- the pseudo-3D head turn |
| **Pseudo-3D keyform sculpting** | the cage keyforms are FITTED (least squares through the Bernstein basis the mover evaluates) to an ellipsoid-head turn whose silhouette holds while the face slides around it; fit error is printed by the build |
| **Rotation deformers** (AngleZ, body lean, neck) | `RigExecFkChain` Body -> Neck -> Head (`parentRelative` nested controls); a `RigExecMatrixMover` carries each face layer with the head joint, a `RigExecSkinMover` blends the neck across Body/Neck/Head |
| **Hair physics / secondary motion** | one `RigExecFkChain` per lock (7 fringe strands, 2 side locks, 2 inner side strands, the ahoge, 2 back-hair masses: 14 chains, 44 joints), skinned by `RigExecSkinMover`s with smooth bone weights. Their avars are the output of a damped-spring simulation run by `build_bust.py`, so the hair lags, overshoots and settles after every head move |
| **Mouth vowels (A I U E O)** | a pose-space RBF: `RigExecPoseInterpolator` with five `RigExecPose` swings of the mouth joystick (rx tips it open, ry wide/round). Each pose's `outputs:weight` fans out to TWO channels -- the lip keyform and a jaw-drop keyform on the face -- and the smile dial (rz) is twist, invisible to the swing poses |
| **Clipping masks** (lids over the eyeball) | skin-coloured mask rings around each eye opening: their inner edge IS the lid line (keyformed), their outer edge is fixed, so a closing lid sweeps skin over the iris; the eye white is a band between the lids that collapses on its own |
| **Breathing / body angle** | a second `RigExecLatticeMover` cage around the torso with `Breath` and `BodyTurn_L/R` keyforms, driven by the torso handle |
| **Rig visualisation** | joints, controls and solver guides are synthesized by the imaging plugin; the lattice warps are shown by guide-purpose `BasisCurves` grids deformed by the same lattice movers as the art |

### Parameters (the handles to animate)

| control (under `/Bust/Rig/Controls/Body_ctl/...`) | avar | drives |
|---|---|---|
| `Body_ctl` | `rz`, `tx` | lean about the hips (pivot below frame), sway |
| `Neck_ctl`, `Head_ctl` | `rz` | neck bend, head tilt (layered mesh AngleZ) |
| `Head_ctl/FaceAngle_ctl` | `ry` / `rx` | head turn (AngleX, keyforms at +-15/30) / nod (AngleY, +-10/20; +rx = down) |
| `Head_ctl/Look_ctl` | `tx`, `ty` | eyeballs (+-1) |
| `Head_ctl/Eye_R_ctl`, `Eye_L_ctl` | `ty` / `rz` | blink (-1 closed, in-between at -0.5) / ^^ smile-eye (20) |
| `Head_ctl/Brow_R_ctl`, `Brow_L_ctl` | `ty` / `rz` | up (+1) and down (-1) / worried (+20) and angry (-20) |
| `Head_ctl/Mouth_ctl` | `rx`, `ry` / `rz` | vowel pad: A (30, 0), I (0, 30), U (0, -30), E (21, 21), O (21, -21) / smile (+20), frown (-20) |
| `Torso_ctl` | `ty` / `ry` | breath (0..1) / body turn (+-10) |
| `Head_ctl/<lock>_<n>_ctl` | `rz` | each hair joint (the simulation writes these) |

### Mover stacks (bottom runs first)

```
Movers/Channels/*          float math: parameter -> keyform weights (runs before exec)
Movers/Cages/*Keyforms     blend shapes on the two lattice cages
Movers/Geometry/<Face|Eyes|Brows|Mouth>   Keyforms -> HeadTilt (matrix) -> HeadWarp (lattice)
Movers/Geometry/<FrontHair|BackHair>      HairSkin (FK locks + head) -> HeadWarp (lattice)
Movers/Geometry/Neck       SpineSkin -> HeadWarp -> BodyWarp
Movers/Geometry/Body       Lean (matrix) -> BodyWarp (lattice)
```

## Numbers

- 16,153 art points / 23,068 triangles in 8 meshes; 55 controls, 47 joints,
  15 FK chains, 35 blend channels with 41 keyforms, 24 float-math movers,
  11 lattice movers, 1 pose interpolator (6 poses).
- Compile ~31 ms; evaluation ~48 ms per frame on the CPU evaluator (~20 fps),
  almost all of it in the Bernstein lattice kernel (see below).
- Head-cage fit: mean error 0.04-0.09 units (head is ~21 units wide) at the
  15-30 degree keyforms.

## Notes and limitations found while building it

- A `RigExecFloatMathMover` chain whose target weight is itself CONNECTED
  (e.g. to an avar) starts from the attribute's authored value, not the
  connected one, so `connect + clamp` on the same weight silently yields the
  authored 0. The pattern used here instead reads the avar through the
  math mover's own connected `inputs:value`.
- `RigExecBlendSample` activations must be positive (there is an implicit
  zero sample at 0), so a parameter that runs both ways needs two channels.
- `RigExecPoseInterpolator` measures rotation, and translation as well when
  `rigExec:enableTranslation` is set. The vowel pad is a joystick (swing)
  with translation measurement left off.
- `RigExecLatticeMover` in the default `legacy` evaluation uses the bound
  Bernstein cage and does not read `rigExec:basis`, recomputing it with
  `std::pow` per cage point per point per frame. `regularGrid` instead
  interpolates with `rigExec:interpolationU/V/W` (`linear`, `cardinal`,
  `bspline`, or `catmullRom`). The cage is fitted rather than hand-placed
  precisely because the Bernstein basis is global.
