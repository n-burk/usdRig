# The biped rig: building it, opening it, animating it

A port of the Maya NXT biped into RigExec. This is the practical guide:
how to get the character on screen with its layers composing, what the
controls are, and which parts are finished. If you want to know *why*
something is built the way it is, the module docstrings under
`tools/biped/` carry the reasoning and the measurements — they are not
decoration, several of them record a wrong approach and the number that
disproved it.

Everything is derived from the Maya build, which is ground truth:

    ~/Documents/dev/squarebit/templates/templates/maya/biped/rig/default/data/
        build/joint_positions.data      224 joints, world and local transforms
        build/control_positions.data    361 control curves, CVs and colours
        body_rig.nxt                    the body build graph
    ~/Documents/dev/squarebit/templates/templates/maya/base/rig_bits.nxt
                                        limbs, girdles, foot, param nodes
    ~/Documents/dev/squarebit/core/core/maya/rig/spline.py
                                        the ikSpline spine

Nothing in the rig is hand-authored geometry or invented numbers. If a
value looks arbitrary, it came out of one of those files.

---

## 1. Prerequisites

You need a built OpenUSD (26.08) beside the repo, and RigExec built against
it:

    ~/Documents/dev/squarebit/usd-install     OpenUSD, built
    ~/Documents/dev/squarebit/usdRig          this repo

Build RigExec with `bin\build_rigexec.bat` (or `bin/build_rigexec.sh`).
Check it with `ctest --test-dir build -C Release`.

**Do not use a high `-j`.** A 64-way build filled the disk here: the
transient `.tlog` and object files dwarf the artifacts. `-j 12` is fine.

### Environment

Every script below needs `pxr` and `rigexec` importable and the schema
plugin registered. Do not assemble that by hand — `bin\_env.bat` and
`bin/_env.sh` already get it right, and there is a runner that uses them:

    bin\run_biped.bat                       lists the scripts it can run
    bin\run_biped.bat <script> [args...]    runs one with the env set

One trap worth naming, because it produces fifty lines of error at once:
if you set `PXR_PLUGINPATH_NAME` yourself and point it at the *source*
`plugin/rigExecSchema/resources` as well as the built
`build/usd/rigExecSchema/resources`, Plug registers the schema twice and
every RigExec type fails with "previously provided by plugin
'rigExecSchema'". `_env.bat` points at the generated directory only, which
is also the only one carrying the `LibraryPath` that gives RigExec prims
their bounds.

---

## 2. Build the character

    bin\run_biped.bat build_biped_rigexec biped.usda ^
        --spine --twist --fk-follow-parent --layered ^
        --skin-rigexec <path-to>\mesh_manifest.json

That writes `biped.usda` (flat, ~4 MB) **and** the layered form beside it:

    biped_layered.usda           the root: sublayers the three below
    biped_layered_center.usda    spine, neck, hips, chest, mesh, materials
    biped_layered_left.usda      everything on the left side
    biped_layered_right.usda     the right side, referencing the left

Then add the per-limb IK/FK switches (see §5 for why this is a separate
step for now):

    bin\run_biped.bat params biped.usda

### The flags

| flag | what it does |
|---|---|
| `--spine` | spline-IK spine and neck (the ikSpline analogue) |
| `--twist` | the twist/noTwist/trans helper joints, aim-constrained |
| `--fk-follow-parent` | FK controls follow their parent control |
| `--layered` | also write the center/left/right layered form |
| `--skin-rigexec <manifest>` | author and deform the mesh with RigExec's own skinning |
| `--no-girdles` | omit the clavicle and pelvis controls |
| `--no-feet` / `--no-hands` | omit the reverse foot / FK fingers |
| `--ik-handles-follow-girdle` | nest the IK handles under the clavicle/pelvis controls instead of world (see §4) |
| `--no-torso` | omit `torso_ctl`; the chest pivot then nests under `hips_ctl` directly |
| `--skin-mode {skin,matrix}` | true LBS in one mover, or the older sequential chain |

`--fk-follow-parent` is off by default because it changes what the
*controls* do, not what the rig does: joint results are bit-identical
either way. With it on, rotating the FK shoulder carries the elbow and
wrist controls with it (35° moves them 30.247 cm and 34.403 cm — the same
distances the joints move). With it off the controls stay behind while the
joints move, which is correct but reads as broken.

The mesh manifest comes from `tools/biped/extract_maya_mesh.py`, run under
Blender against the exported FBX. If you do not have one, omit
`--skin-rigexec` and you get the rig without the body mesh.

### Prove it before trusting it

The builder self-checks and prints:

    checked 224 joints, worst 4.91958e-05 cm (index_002_combo_l_driver)
    PASS -- RigExec reproduces Maya's rest skeleton

That compares every joint's evaluated world origin against Maya's recorded
`world_translate`. If it says FAIL, stop — the parent-local rest
composition is wrong and nothing downstream is meaningful.

Then the behavioural gates, which drive controls and measure joints rather
than checking that the rest pose survived:

    bin\run_biped.bat verify_spine   biped.usda
    bin\run_biped.bat verify_girdle  biped.usda
    bin\run_biped.bat verify_rig     biped.usda
    bin\run_biped.bat verify_layers  biped.usda biped_layered.usda

---

## 3. Open it

    bin\launch_usdview.bat biped_layered.usda

Open the **layered** file, not the flat one — that is the composed
character and the thing to look at. Both evaluate identically; see §6.

`launch_usdview.bat` runs an incremental `cmake --build` first. That is
usually what you want, but note the inverse: **a running usdview LOCKS
`build\rigExec.dll`**, so you cannot rebuild the engine while the viewer is
open, and a second launch will fail its build step. Close the window first.

### The panels

Both live under the **RigExec** menu (deliberately not Window).

- **RigExec → Avar Editor** — select a control, get its channels as spin
  boxes and sliders, live. Grouped Translate (cm) / Rotate (deg) / Scale /
  Custom / Other. Channels are discovered by namespace, so a rig's own
  dials show up automatically: `avars:ikfk` on the param nodes, and the
  `foot:roll` / `foot:bank` / `foot:toePlantAngle` dials on `bank_?`.
  A slider drag is one undo entry.
- **RigExec → Execution Stack** — every solver and mover in evaluation
  order, with filters. Use it when something does not move and you want to
  know what wrote it last.
- **RigExec → Viewport Tools** — Maya-style move/rotate/scale gizmos that
  write avars directly on controls and joints, with a Hydra preview
  mid-drag.

---

## 4. What you can animate

In rough order of usefulness:

**Body** (`hips_ctl`, `torso_ctl`). `hips_ctl` is the master: it
carries the whole character rigidly (12.00 cm on every joint for a
12 cm move, 0.0000 cm drift) — except the IK hands and feet, which
stay planted by design, see below. `torso_ctl` nests under it and
carries the chest, neck, head, clavicles and arms while leaving the
hips, pelvis and legs at exactly 0.000. The hierarchy is Maya's:
`hips > {hip_swivel, torso > chest}`.

**Spine and neck** (`spine_root_ctl`, `spine_mid_ctl`,
`spine_end_ctl`, `neck_root_ctl`, `neck_end_ctl` — the neck has no mid
control, as in Maya; its cv1 is aimed by the solver). `spine_root_ctl` is
Maya's `hip_swivel`, `spine_end_ctl` is `chest_top`. Both ends twist the
chain, graded: a 30° roll at the hips gives 29.7 / 23.0 / 17.4 / 11.8 /
6.0 / −0.3 / 0.0 up the spine; the same at the chest grades the other way;
both together give a constant 30°; opposite ends give a wind-up. Stretch
the chain and the joints squash on Maya's volume-weight profile, thinnest
at `spine_3` where the weight peaks at 0.5.

**Girdles** (`clavicle_l/r_ctl` under the chest control, `pelvis_l/r_ctl`
under the hip control). The whole limb rides them — moving a pelvis control
5 cm moves the thigh 5 cm, in both IK and FK.

**Limbs.** Per side: `arm_?_root` (Maya's shoulderSwing), `arm_?_ik`,
`arm_?_pv`, and three FK controls. Same shape for the legs.

**IK/FK switch** — `avars:ikfk` on the four param nodes `arm_l_params`,
`arm_r_params`, `leg_l_params`, `leg_r_params`, small yellow cubes riding
the wrists and ankles. **0 = FK, 1 = IK**, and it interpolates. Note this
is the solver's convention, not Maya's, whose `ikfk` attribute is inverted
— reproducing that would have been a trap, since the value here *is* the
`RigExecBlendPointFrames` weight. Ships as Maya does: arms in FK, legs in
IK.

To key the switch, usdview will not do it (it is a viewer, its property
panel does not author time samples). Use:

    bin\run_biped.bat set_ikfk biped.usda --show
    bin\run_biped.bat set_ikfk biped.usda --all --value 0
    bin\run_biped.bat set_ikfk biped.usda --limb leg_l --key 1:1 --key 24:1 --key 48:0

The keyed form also sets the stage time range, so usdview gives you a
timeline to scrub.

**Feet.** `heel_?`, `toe_?`, `toeBend_?`, `ballRoll_?` and `bank_?` nested
under the leg's IK control. The roll itself is a *dial*, not a control —
`foot:roll` on `bank_?`, which sits at the ball — exactly as Maya drives it
(`bank.tz → SDK → footRoll`). `foot:bank` and `foot:toePlantAngle` live
beside it.

**Hands.** Five FK finger chains per side, posed through the joints' own
avars; no solver is needed because a RigExecJoint carries avars and joints
nest in namespace.

### The body controls, and where the IK handles live

`hips_ctl` is Maya's `hips`: world-aligned at `hips_bind`, and the parent of
everything above the legs' IK handles — `spine_root_pivot` (hip swivel) and
`torso_ctl` (Maya's `torso`, world-aligned at the base of the spine) nest
under it, `spine_end_pivot` (chest) under `torso_ctl`, `neck_root_ctl` under
`spine_end_ctl` and `neck_end_pivot` (head) under `neck_root_ctl` — Maya's
`hips > hips_gimbal > {hip_swivel, torso > chest > chest_ik > chest_top}`,
with the neck nul parent-constrained to `chest_top` and the head under the
neck. Translating `hips_ctl` moves every joint and every skinned point by
exactly its displacement.

The IK effector and pole controls stay in **world**, as in Maya (the handle
hangs off the rig group; its space switch is orient-only and defaults to
local), so a planted hand or foot stays planted when the hips, the swivel
or the pelvis move. `--ik-handles-follow-girdle` nests them under the
clavicle/pelvis control instead, so the hand or foot rides the girdle.

---

## 5. Known gaps

Named here rather than discovered later.

- **A whole-body ROTATION leaves ~0.1 degree in the spline interiors.**
  `hips_ctl ry=30` gives every joint 30.000 degrees with 0.0000 drift
  except spine_0 (0.138 deg), spine_3 (0.023 deg / 0.0003 cm), neck_0
  (0.013 deg) and skull (0.117 deg / 0.0000 cm), which the face
  inherits. That is the spline solver's swing/twist extraction about a
  tilted chain axis; Maya's `decompose_rotation` has the same property.
  A whole-body TRANSLATION is exact: the mesh's min, max and mean
  displacement are all 12.207 cm over 26,276 points.
- **A 25 degree chest swing turns `spine_5_bind` 53.65 degrees**, not
  25 — an artifact of the degree-2 curve's short final segment.
- **Guides draw at the control's pivot with no offset.** Maya's head
  loop is centred 13.4 cm above its pivot, so the head control sits
  that much low. Honest sizes were kept rather than shrinking the
  shape to hide it. A `guide:offsetX/Y/Z` would fix it properly.
- **The length floor holds the spline chain, not the chest control.**
  `chest_bind` is parent-constrained to the chest control (Maya's
  `chest_top_grp` override) and the clavicles nest under it, so those
  still follow the control below the floor. A control-level clamp is
  engine work.
- Not built: `hips_gimbal`, `hipsPivot`, `chest_ik`, the hips space
  switches, auto-clavicle, the param node's `ikfk_switch` snap and
  `length` attributes, `wristTwist_?`/`ankleTwist_?` swing-twist
  decomposition, and `pinky_cup_falloff`.

### Fixed, but worth knowing about

Each of these was a real defect with a non-obvious cause, and the cause
is the reusable part:

- **The neck and head used to fly when the hips swivelled** (25.64 and
  33.08 cm on a 30 degree swivel, while their own parent `chest_bind`
  held at 0.00). Two causes at once: `neck_to_bind` executed BEFORE
  `spine_to_bind` and the chest override, so the chest's absolute
  re-write landed on already-written neck joints; and nothing
  structurally tied the neck to the chest at all. Now `neck_root_ctl`
  nests under `spine_end_ctl` and the head pivot under the neck.
- **A pure body TRANSLATE used to twist the shoulders** 17-28 degrees,
  because `twist_aims` executed before `hips_follow`.
- **The param nodes could not live under `/Controls`** — it tripped a
  pose-dependency cycle. The cause was NOT dependency granularity, as
  two investigations assumed: `_GetMoverExecutionOrder` walks the
  Movers namespace bottom-up, so a chain appended at the bottom
  executes FIRST and waited on the very solvers it feeds.
  `params.py::_execute_last` reorders it to the top.
- **The layered rig stopped compiling** when that reorder landed:
  `split_layers.py` pinned child order from the raw layer specs, which
  do not have `reorder nameChildren` applied, so it threw the reorder
  away. It now pins the COMPOSED order.

The pattern in three of those four is the same, and it is the thing to
suspect first when something in this rig moves that should not: **a
mover chain applies in reverse add order, the Movers stack is walked
bottom-up, and a joint's write propagates to its descendants.**

---

## 6. How the layering works

`tools/biped/split_layers.py` splits a built rig by side. A prim belongs to
a side if its name or an ancestor's carries the side token; everything else
— spine, neck, hips, chest, mesh, materials, the skin mover — is center.

    root .usda      sublayers center, left, right (strongest first)
      _center       92 prims   ~3.7 MB   the mesh dominates
      _left        194 prims   ~143 KB
      _right       194 prims   ~141 KB

The two sides are structurally identical, so the right side is expressed as
**references onto the left layer** plus overrides: 79 right prims reference
their left twin, 240 attributes arrive through the arc, and only 174
attributes and 64 relationships are authored locally — the mirrored
`rest:space`, the blue-vs-red `guide:displayColor`, the mirrored
`inputs:aimVector`, and editor node positions.

Two things here are load-bearing and easy to break:

**It is a `references` arc onto the left layer FILE, not `inherits`.** The
inherits version passed every rest check and then failed the posed one by
30 cm: keying `avars:rz` on the *left* shoulder moved the *right* arm,
because an inherit composes every opinion standing on the left prim —
including animation authored later in a stronger layer. A reference to the
saved layer composes only what that layer holds, so rig data crosses and
animation does not. `specializes` and an internal reference behave the
same way.

**The root layer pins `reorder nameChildren`.** RigExec derives mover
execution order from namespace order, and composing a prim's children from
three sublayers does not preserve the order they had in one layer. Without
the 75 pins the rig does not merely misbehave, it fails to compile:

    Unsatisfied final read: body_geo_skin reads final of pelvis_l_bind
    but a writer with a later ordinal exists

`verify_layers.py` checks this by counterexample on every run — it strips
the pins, confirms the failure, and restores them.

### Proving the layered rig is the same rig

    bin\run_biped.bat verify_layers biped.usda biped_layered.usda

It compares composed namespace, composed properties, mover order, and joint
frames at rest *and* under a test pose, and it checks that keying the left
side alone leaves every right joint at rest. Current result on the shipped
build: 480 prims both sides, 1421 attribute values / 34 connections / 212
relationships identical, 104 movers in identical order, and all 252 joint
frames matching to **0.000e+00 cm** in both poses.

`--payload` switches to a payload per side if you want load-on-demand.
Sublayers are the default because the layers are small and a half-loaded
rig does not compile.

### What is still not shared between the sides

All 30 relationships per side (`rigExec:joints`, `rigExec:controls`,
`rigExec:moves`, the effector/pole/root controls, `inputA`/`inputB`) point
*outside* their prim's scope — the joints live in one hierarchy under
`hips_bind`, the controls are siblings — so no per-prim arc can rebase
them. Sharing them needs the arc on a common ancestor of both solver and
joints, i.e. `/Biped/Rig` itself: a builder restructure where limbs are
authored as class instances inside the joint hierarchy, with skin
influences remapped by path instead of by name. That is a real change, not
a post-process.

---

## 7. Where things live

| path | what |
|---|---|
| `tools/biped/build_biped_rigexec.py` | the builder; start here |
| `tools/biped/limb_frames.py` | the shared limb re-frame, with its invariance proof |
| `tools/biped/girdle.py` | clavicles, pelvis, hips follow, chest override |
| `tools/biped/build_foot.py`, `build_fingers.py` | reverse foot, FK foot, fingers |
| `tools/biped/control_shapes.py` | Maya's shapes, colours and sizes on the controls |
| `tools/biped/params.py` | the per-limb IK/FK param nodes |
| `tools/biped/set_ikfk.py` | set or key the IK/FK blend |
| `tools/biped/split_layers.py` | the center/left/right split |
| `tools/biped/verify_*.py` | behavioural gates |
| `tools/biped/exec_stack.py` | the execution stack, on the command line |
| `plugin/rigExecUsdview/avarEditorUI.py` | the Avar Editor panel |
| `plugin/rigExecUsdview/execStackUI.py` | the Execution Stack panel |

Related: [`control-guides.md`](control-guides.md),
[`viewport-gizmos.md`](viewport-gizmos.md),
[`composition-arcs.md`](composition-arcs.md), [`spec.md`](spec.md).
