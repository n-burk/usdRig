# ![Touch Region](../../icons/touch_region.png) Touch Region

*A named set of mesh faces that selects the control posing them.*

| | |
|---|---|
| **Node type** | `RigExecTouchRegion` |
| **Example** | [touch_region.usda](../examples/touch_region.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

A touch region is the annotation that makes a character
clickable: a named set of face indices on one mesh, plus the control a click
inside that set should select. Touch a shoulder in the viewport and the
shoulder's control is selected, with no picker window and no knowledge of where
the rig parked its controls. Regions are *not* `GeomSubset`s — hdSt collects
every face subset under a mesh whatever its `familyName`, so touch sets living
there collided with the material-bind sets sharing the same faces
(schema.usda:2532-2536); they sit in their own `RigExecTouchRegions` scope
instead, found by type rather than by position.

One named touch set: the faces it owns and the control a
click inside it selects.

## How it works

Nothing about a touch region runs in an evaluation
phase — the rig evaluator never reads a `rigExec:touch:*` token at all (no
match for `rigExec:touch` anywhere under `libs/rigExec/`), so a region adds no
compile record, no step and no cost to a pose. It is read once at *attach* time
by the TouchPose viewport tool, which finds every `RigExecTouchRegions` scope by
type, filters them by the mesh they name, and reads each typed
`RigExecTouchRegion` child in namespace order
(touchPoseModel.py:73-93, 217-244). From those it builds a flat
face → region-index array — `-1` for unpainted skin — and hands it to the
native mesh once (touchPoseModel.py:167-189); a pick is then a BVH ray cast to a
face plus one array index, and the first target of `rigExec:touch:control`
becomes the prim the selection is replaced with (touchPoseModel.py:239,
touchPoseUI.py:1011-1029). The
region writes nothing to the stage: the highlight is a Storm shader tint fed by
two synthetic primvars, `rigExecTouchRegion` (face → slot) and
`rigExecTouchTable` (slot → colour), added by a scene-index filter
(touchPoseHighlight.h:5-31).

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:touch:faces` | Face indices on the mesh named by the parent scope's `rigExec:touch:mesh`. Indices outside the mesh's face range are dropped silently. | no |
| `rigExec:touch:control` | The prim a click inside this region selects. Only the first target is used; unauthored means the region is skipped by the default reader. | no |
| (parent scope) | A `RigExecTouchRegions` prim, which names the mesh and holds the shared palette. Regions are only found as its children. | yes |

## Parameters

### Region scope

#### `rigExec:touch:mesh`

*Relationship.*

The UsdGeomMesh whose faces these regions index.

#### `rigExec:touch:palette`

*Type:* `uniform color3f[]`. *Default:* `[]`.

One ramp shared by every region, indexed by the
region's own order. On the scope rather than per region: the
overlay needs the whole ramp in a single read.

#### `rigExec:touch:alpha`

*Type:* `uniform float`. *Default:* `1`.

Opacity the overlay draws the regions at.

### Node parameters

#### `rigExec:touch:faces`

*Type:* `uniform int[]`. *Default:* `[]`.

Face indices on the mesh named by the parent's
rigExec:touch:mesh. Face indices, not vertex indices: a pick
resolves to a face and this is the set it is looked up in.

#### `rigExec:touch:elementType`

*Type:* `uniform token`. *Default:* `"face"`.

Valid values: `face`.

What rigExec:touch:faces indexes. Only `face` today;
the token is here so a point or edge set does not need a second
attribute when one is wanted.

#### `rigExec:touch:control`

*Relationship.*

The control a click in this region selects. Unauthored
means the region is drawn but drives nothing, which is how an
unfinished paint reads.

## Example

A two-segment limb, twelve quads of skin, painted into two
regions: `Upper` owns the six faces over the first bone and selects the Upper
control, `Fore` owns the six over the second and selects the Fore control. The
FK chain folds the limb and unfolds it over frames 1001-1016, and the regions
follow the deformation for free because they index faces, not points.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\touch_region.usda
```

## Tips

- Regions must not overlap: the face → region table is built by assignment, so a face claimed twice ends up owned by whichever region comes LAST in namespace order (touchPoseModel.py:172).
- `rigExec:touch:elementType` only allows `face` today, and nothing reads it yet — the exporter writes it and the reader ignores it. It is there so a point or edge set does not need a second attribute later.
- A region binds by path, not by type: the picker just resolves the target and selects it, so an unfinished paint can point at a joint, a solver, anything — but a region with no target is dropped, not drawn inert, unless the host asks for `require_control=False`.

## See also

- [Touch Regions](touch_regions.md)
- [Control](control.md)
- [Matrix Mover](matrix_mover.md)

---

[UsdRig](../index.md)
