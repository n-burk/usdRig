# Biped -- the ported character

**Open `Biped_layered.usda`.** That is the whole character: skeleton, rig,
skinned mesh and materials, composed from three side layers. It is
self-contained -- clone the repo and open it, nothing else to build or
fetch.

**Nothing else is needed to open it.** The mesh points, the skin weights,
the materials and every rest transform are authored directly into these
files; the only asset paths in the whole stack are the relative sublayers
below.

    bin\launch_usdview.bat examples\biped\Biped_layered.usda

| file | what |
|---|---|
| `Biped_layered.usda` | **the one to open** -- a root sublayering the three below |
| `Biped_layered_center.usda` | spine, neck, hips, chest, the mesh and the materials |
| `Biped_layered_left.usda` | everything on the left side |
| `Biped_layered_right.usda` | the right side, as references onto the left layer plus the ~336 attributes that genuinely differ |
| `Biped_face.usda` | the face, as one additive layer over the rig: sublayered by `Biped_stack.usda`, `Biped_all.usda` and `Biped_everything.usda`, and NOT by `Biped_layered.usda` |
| `Biped.usda` | the same rig flat, in one file, if you want to read it |
| `Biped_anim.usda` | an 8-frame animated overlay on `Biped.usda`, for timing and for testing the animated evaluate path |

`Biped_anim.usda` sublayers `Biped.usda` and keys six controls over frames
1-8. A static stage never re-reads a time sample, so it cannot show what an
animated frame costs or whether the evaluator handles a time change
correctly; that is what this overlay is for:

    build\rigExecPose examples\biped\Biped_anim.usda --frames 1,2,3,4,5,6,7,8 --joints

The layered and flat forms evaluate identically -- 576 prims, 110 movers in
the same order, and all 252 joint frames matching to 0.000e+00 cm at rest
and under a test pose.

## What is in it

252 joints reproducing the conventional rest skeleton to 4.9e-05 cm, a skinned
26,276-point body mesh with 12 UsdPreviewSurface materials (`body_geo`
carries five of them as `GeomSubset`s, since one binding cannot express
five materials over 26,274 faces), and a rig: spline-IK spine and neck,
two-bone IK/FK limbs with a per-limb switch, reverse foot, FK finger
controls that follow the arm in IK as well as FK, twist helpers, and
`hips_ctl` as the master body control.

The face arrives as its own sublayer: the jaw group, the jaw compression
that couples the mid-face and the nose to it, the nose's blend between
upper and lower face, and the eye look-ats. Open `Biped_stack.usda`,
`Biped_all.usda` or `Biped_everything.usda` to get it; `Biped_layered.usda`
is the body rig alone.

## Driving it

Select a control and open **RigExec → Avar Editor**. The controls worth
starting with:

- `hips_ctl` -- the master; moves the whole character
- `torso_ctl` -- chest, neck, head and arms, not the legs
- `spine_root_ctl` / `spine_end_ctl` -- hip swivel and chest; both twist the
  spine, from their own end
- `arm_?_params` / `leg_?_params` -- the small yellow cubes near each wrist
  and ankle. Their `ikfk` channel is the IK/FK switch: **0 = FK, 1 = IK**,
  and the inactive control set fades as you cross. Arms ship in FK, legs in
  IK.
- `bank_?` at each ball of the foot -- its `foot:roll` / `foot:bank` dials
  are the foot roll
- the small squares along each finger -- `index_002_l_bind_fk` and its 41
  siblings, nested one under the next. They drive their own joint and
  everything below it, and the whole hand rides the wrist in either IK or
  FK
- `skull_ctl` on the head, carrying the face. `jaw_ctl` at the jaw, with
  `face_upper_ctl` / `face_lower_ctl` above it and `nose_bridge_ctl` and
  `nose_ctl` in between. The brows sit on `head_tip_ctl`, and each
  `brow_main_?_ctl` carries its inner brow and its peak. Three
  dials carry the automatic parts: `face:jawCompression` on `jaw_ctl`
  (0 to 1, the jaw pressing up into the middle of the face),
  `face:noseFollow` on `face_lower_ctl` (how much of the lower face the
  nose carries, 1 by default) and `face:lookAt` on `lookAt_ctl`
- `lookAt_ctl`, the wide plate in front of the eyes. It is top-level,
  beside `hips_ctl`, so a gaze holds while the head turns. `lookRot_ctl`
  sits between the eyes and turns both together; `eye_l_ctl` and
  `eye_r_ctl` sit on the eyes and turn one each

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
## Touching it

**RigExec → TouchPose** lets you pick controls by touching the character
instead of hunting for them. Tick the box, hover the body, and the painted
region under the cursor lights up; click it and the control that owns that
region becomes the selection -- so the Avar Editor and the gizmo follow
straight away.

    bin\run_usdview_touchpose.bat

Three colours, three states, all taken from the authored
`touch_sets.touch`: the region under the **cursor** in its own painted
colour, the region you clicked **last** in `leadColor` (a green), and
anything **else selected** in `selectedColor` (a
neutral). Selecting a control anywhere -- the Control Picker, the outliner -- lights its region here too.

### Selecting

The same rules as the Control Picker, so it does not matter which one you
reach for:

| gesture | what it does |
|---|---|
| click / drag | **replace** -- the selection becomes what you picked |
| Shift | **toggle** -- adds what is not selected, removes what is |
| Ctrl | **remove**, always. The one gesture that can never add. |

Dragging on the body marquees, and the same three rules apply to it. The
band catches any region it **touches**, not only ones it fully encloses -- otherwise a finger or a clavicle would be nearly uncatchable -- and only
regions facing you, so a band across the chest does not quietly select the
back as well.

With the panel focused: **T** toggles TouchPose, **P** toggles Paint,
**Ctrl+A** selects every region, **Ctrl+Shift+A** clears, **Ctrl+I**
inverts, **Ctrl+S** saves.

While you are dragging a gizmo handle, TouchPose **stands down** -- no
cast, no hover highlight -- and picks up again on the first mouse move after
you let go. The selection highlight stays lit throughout and follows the
pose being dragged, because it is drawn by the body's own shader.

The highlight **authors nothing**. It is a Storm shader tint: a Hydra scene
index in `rigExecImaging` gives the body a per-face region id and a small
per-region colour table, and wraps its materials so the lit colour is mixed
with `table[region]`. A hover changes one constant primvar; no prim is
created, shown or hidden, and the stage never sees an edit. Picking runs in
C++ against the posed mesh (a BVH refit per pose). Measured on this
character: a hover costs about **0.3 ms** where it cost 4-8 ms, a region
crossing reaches the screen in about **15 ms** where it took 60-90 ms, and
TouchPose adds almost nothing to a selection change where it added
~200 ms (`bin\run_testusdview_touchpose_bench.bat`).

While the box is ticked the **mesh is not selectable**: a click on the skin
belongs to TouchPose, and on unpainted skin it selects nothing rather than
picking `body_geo`. The **viewport gizmo wins** where the two overlap, so
dragging a manipulator handle over a limb never re-selects the limb. Untick
it and usdview's own picking is back exactly as it was. Alt still drives
the camera throughout.

### Painting regions

Tick **Paint**, pick a region in the list, and **dragging** on the body
gives the faces under the brush to that region; **Shift-drag** takes them
away. Paint is a mode and the brush owns the drag while it is on, so the
marquee stands down -- deliberately, because a modifier that painted in one
mode and subtracted in another is how an animator deletes half a selection
by reflex. The regions stay non-overlapping as you paint -- a face joining
one leaves the other in the same step -- and nothing is written until
**Save regions**, which rewrites `Biped_touch_regions.usda` and never the
rig.

That last part is not just tidiness. An authored edit on any prim inside
the rig's read roots makes OpenExec uncompile and recompile the network,
which costs about 2.3 s on the next evaluate. The paint buffer lives
outside the rig for that reason (and the highlight is not on the stage at
all), and the headless test asserts the rig's generation counter does not
move across a stroke.

### The files

| file | what |
|---|---|
| `Biped_all.usda` | **the one to open** -- the touch regions over the layered rig, so TouchPose and the Control Picker both work on it |
| `Biped_touch.usda` | the same regions over the flat `Biped.usda` |
| `Biped_touch_regions.usda` | **TouchPose alone** -- 98 regions on a `/Biped/TouchPose` scope and nothing else. Read it, validate it, or stack it under another composition. Opening it on its own shows nothing, because it is `over`s with no geometry under them. |

The regions come from the studio's `touch_sets.touch`, which ships 247
painted sets. 98 are written here: a set naming a control this port does
not have yet is **skipped rather than authored**, so nothing downstream
pays to test, highlight and then refuse a region it can never act on. The
skipped count stays in the import report, so the naming gap is still
visible -- it is overwhelmingly the face rig, 56 eyelid sets among it. What
is written covers 16,739 of `body_geo`'s 26,274 faces (64%).

They are **not** `GeomSubset`s under the mesh, which is the obvious place
for them: hdSt collects every face subset whatever family it declares, so
98 of them collided with `body_geo`'s five `materialBind` subsets and cost
16,739 warnings every time the stage opened. On their own scope, with the
faces in a plain `int[]`, the renderer never sees them.

## The whole stack

`Biped_stack.usda` is the character with everything on it, one layer per
thing it is made of. Open it and the Layer Stack view reads top to bottom
in resolution order:

| layer | what it contributes |
| --- | --- |
| `Biped_picker.usda` | the control picker: `RigExecPicker` prims |
| `Biped_touch_regions.usda` | TouchPose regions: `RigExecTouchRegions` |
| `Biped_psd.usda` | pose interpolators, one weight per corrective |
| `Biped_shapes.usdc` | the blend shapes those weights drive |
| `Biped_layered.usda` | the rig, itself four layers |

and that last one brings in four more: `Biped_layered_left.usda`,
`Biped_layered_right.usda` and `Biped_layered_center.usda` stacked on
`Biped_layered_model.usda`. The model is publishable on its own, and the
rig is three files two riggers can work in at once.

Any layer can be left out for a lighter character. The correctives are
the one pair that has to travel together: the interpolators publish
weights nothing reads without the shapes, and the shapes wait for weights
nobody writes without the interpolators. Either alone is inert rather
than broken, which is what lets a shot drop both without editing
anything.

## Everything the rig owns lives under the rig

The picker, the touch regions and the blend shapes are all specific to
one rig, so they sit inside its `RigExecRoot` rather than beside it:

    /Biped/Rig          RigExecRoot
        Joints          the skeleton
        Movers          solvers, constraints, the blend shape mover
        Controls        what an animator grabs
        Solvers
        Shapes          161 UsdSkelBlendShape targets
        PoseInterpolators
        TouchPose       RigExecTouchRegions, 98 painted regions
        Uman            RigExecPicker, the control picker

    /Biped/Geom         the mesh
    /Biped/Materials

A rig published on its own now carries its picker, its touch regions and
its correctives with it, and none of them has to be found by guessing at
a path. The cost is that anything under the rig root is walked by the
evaluator's discovery passes: measured at about 14 microseconds per prim
across them, so all 467 of these add roughly 6 ms to a 395 ms compile.

## The picker is scene data

The picker is `RigExecPicker` prims in the layer stack, found by schema
type rather than by path or filename. A stage carrying two characters
gets two tabs with nothing configured anywhere; each picker names the rig
it drives through `rigExec:picker:rig`, and its buttons resolve inside
it. The panel nests them: one tab per character, its panels inside.

One prim per button is what makes it overridable. To move, recolour,
relabel, retarget or remove a button, write an opinion on that one prim
in a layer you own:

    over "b_L_Hand" {
        float2 ui:position = (338, 148)
        string ui:text = "L Hand"
    }
    over "b_R_MidFoot" (active = false) { }

Deactivating a panel drops its tab; deactivating the picker removes the
character. The panel follows all of it live, so an override appears as
you author it rather than on the next reopen.

Button targets are relationships, so USD remaps them through a rename or
a reparent: a button goes dead only when its control is really deleted,
and a button that resolves to nothing is not drawn at all.

## TouchPose regions are scene data too

Same rule as the picker, and for the same reason. The regions are
`RigExecTouchRegions` holding one `RigExecTouchRegion` per painted set,
found by schema type. Each scope names the mesh it annotates through
`rigExec:touch:mesh`, so a stage carrying three characters keeps three
sets of regions apart without any of them having to sit anywhere in
particular.

They are NOT `GeomSubset`s. hdSt collects every face GeomSubset under a
mesh whatever family it declares, so the touch sets collided with the
`materialBind` subsets that own the same faces for shading: 16,739
warnings on open, and the renderer doing work for data it never draws.

A save writes back to the scope it was read from rather than to a path
derived from the mesh, so moving the regions does not leave a stage with
two scopes and a reader picking whichever it met first.
