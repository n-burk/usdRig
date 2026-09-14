# Biped -- the ported the conventional tool character

**Open `Biped_layered.usda`.** That is the whole character: skeleton, rig,
skinned mesh and materials, composed from three side layers. It is
self-contained -- clone the repo and open it, nothing else to build or
fetch.

**No the conventional tool and no template data are needed to open it.** The mesh points,
the skin weights, the materials and every rest transform are authored
directly into these files; the only asset paths in the whole stack are the
three relative sublayers below. The the conventional tool data is where this was *derived*
from, and you only need it to regenerate.

    bin\launch_usdview.bat examples\biped\Biped_layered.usda

| file | what |
|---|---|
| `Biped_layered.usda` | **the one to open** -- a root sublayering the three below |
| `Biped_layered_center.usda` | spine, neck, hips, chest, the mesh and the materials |
| `Biped_layered_left.usda` | everything on the left side |
| `Biped_layered_right.usda` | the right side, as references onto the left layer plus the ~336 attributes that genuinely differ |
| `Biped.usda` | the same rig flat, in one file, if you want to read it |
| `Biped_anim.usda` | an 8-frame animated overlay on `Biped.usda`, for timing and for testing the animated evaluate path |

`Biped_anim.usda` sublayers `Biped.usda` and keys six controls over frames
1-8. A static stage never re-reads a time sample, so it cannot show what an
animated frame costs or whether the evaluator handles a time change
correctly; that is what this overlay is for:

    build\rigExecPose examples\biped\Biped_anim.usda --frames 1,2,3,4,5,6,7,8 --joints

The layered and flat forms evaluate identically -- 576 prims, 110 movers in
the same order, and all 252 joint frames matching to 0.000e+00 cm at rest
and under a test pose. `tools/biped/verify_layers.py` re-proves that on
demand.

## What is in it

252 joints reproducing the conventional rest skeleton to 4.9e-05 cm, a skinned
26,276-point body mesh with 12 UsdPreviewSurface materials (`body_geo`
carries five of them as `GeomSubset`s, since one binding cannot express
five materials over 26,274 faces), and a rig: spline-IK spine and neck,
two-bone IK/FK limbs with a per-limb switch, reverse foot, FK finger
controls that follow the arm in IK as well as FK, twist helpers, and
`hips_ctl` as the master body control.

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

**RigExec → Viewport Tools** gives move/rotate/scale gizmos that write
avars directly, if you would rather drag than type.

## Touching it

**RigExec → TouchPose** lets you pick controls by touching the character
instead of hunting for them. Tick the box, hover the body, and the painted
region under the cursor lights up; click it and the control that owns that
region becomes the selection -- so the Avar Editor and the gizmo follow
straight away.

    bin\run_usdview_touchpose.bat

Three colours, three states, all taken from the authored
`touch_sets.touch`: the region under the **cursor** in its own painted
colour, the region you clicked **last** in `leadColor` (a green, as the conventional tool's
kLeadSelected is), and anything **else selected** in `selectedColor` (a
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

While you are dragging a gizmo handle, TouchPose **stands down entirely** -- no cast, no highlight, no authoring -- and picks up again on the first
mouse move after you let go. The selection highlight stays lit throughout;
only the hover one goes. Measured at **0.06 ms per mouse move instead of
10.7 ms**, which is what you would otherwise be paying on top of the
manipulation you are actually watching.

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
which costs about 2.3 s on the next evaluate. Both the highlight overlay
and the paint buffer live outside the rig for that reason, and the
headless test asserts the rig's generation counter does not move across a
stroke.

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

To re-import after a repaint in the conventional tool:

    bin\run_touchpose.bat import_touch examples\biped\Biped.usda

That writes both files and verifies what it wrote. `--list` reports the
resolution without writing, and `--keep-unbound` authors the skipped sets
anyway.

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

## Rebuilding it

These files are generated, and checked in so the character can just be
opened. To regenerate:

    bin\run_biped.bat build_biped_rigexec examples\biped\Biped.usda ^
        --spine --twist --fk-follow-parent --materials ^
        --skin-rigexec <path-to>\mesh_manifest.json
    bin\run_biped.bat params examples\biped\Biped.usda --place controls
    bin\run_biped.bat split_layers examples\biped\Biped.usda ^
        examples\biped\Biped_layered.usda
    bin\run_biped.bat picker_xml --export %TEMP%\picker.json ^
        --stage examples\biped\Biped.usda
    bin\run_biped.bat picker_usd %TEMP%\picker.json ^
        examples\biped\Biped_picker.usda ^
        --rig /Biped/Rig --root /Biped/Rig --name Uman
    bin\run_touchpose.bat import_touch examples\biped\Biped.usda

The `params` and `split_layers` steps come after the build, in that order -- `params` adds the IK/FK switches and the opacity wiring, and the split has
to see them.

Note `--layered` is deliberately NOT in that command even though the flag
exists. It writes the layered form DURING the build, which is before
`params` runs, so the layer stack would be stale the moment the IK/FK
switches were added -- and `verify_layers` then fails by 45.8 cm comparing
two stages that hold different poses. Run `split_layers` explicitly, last
but one, and the flag is redundant. The picker export comes last and needs the built stage: it
resolves each button onto the prim it selects, so a control that does not
exist yet leaves its button with nothing to point at and the export drops
it. The JSON in the middle is a temporary: the deliverable is
`Biped_picker.usda`, and nothing at runtime reads the JSON. `import_touch` is last
for the same reason and resolves through the same tables
(`tools/biped/rig_names.py`), from the other end: the picker starts from a
delivery name, TouchPose from a build name. The mesh manifest is derived from the mesh already in
`examples/biped/Biped.usda`.

Full guide, including what is not finished: [`docs/biped-rig.md`](../../docs/biped-rig.md).
