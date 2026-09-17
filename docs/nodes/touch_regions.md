# ![Touch Regions](../../icons/touch_regions.png) Touch Regions

*Named face sets that turn the model itself into the control picker.*

| | |
|---|---|
| **Node type** | `RigExecTouchRegions` |
| **Example** | [touch_regions.usda](../examples/touch_regions.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Touch regions make the skin clickable: a named set of faces on one
mesh carries the control a click inside it should select, so picking the
forearm selects the forearm control instead of hunting for a wire shape in
a crowded viewport. A `RigExecTouchRegions` scope names the mesh it
annotates and holds one `RigExecTouchRegion` child per set, plus the one
colour ramp declared for all of them — `rigExec:touch:palette`, indexed by
the region's own order — and the opacity to draw them at. The scope is
found by type, so it
may be parked beside the geometry it annotates or inside the `RigExecRoot`
for a studio that ships one prim holding the whole rig.

Regions are annotation, not rig: a stage evaluates identically with them,
without them, or with half of them unpainted. They are deliberately *not*
`GeomSubset`s — hdSt collects every face subset under a mesh whatever its
`familyName`, so a touch set collided with the `materialBind` subset owning
the same face (16,739 warnings on open), and the sets were moved into a
scope of their own.

A mesh's touch regions: the shared palette, and one
RigExecTouchRegion child per named set.

A SCOPE rather than a bare Typed prim: it is a grouping of
annotations on drawable geometry, and an imageable container costs
nothing and keeps the door open for anything drawable parented under
it later.

THE HIGHLIGHT OVERLAY IS NOT HERE, and that is measured rather than
chosen. It is a real rprim, and a prim inside the asset is inside the
rig's read roots: authoring the hover array there cost 184.89 ms per
region crossing against 3.99 ms at the stage root, with a 16.7 ms
frame. It also cannot carry its own schema type -- a concrete type
with no UsdImaging adapter never becomes an rprim at all. So the
overlay is a plain Mesh at the stage root, named for its asset so a
shot full of characters does not share one.

Found BY TYPE, so the regions may be parented anywhere -- beside the
geometry they annotate, or inside the RigExecRoot for a studio that
ships one prim holding the whole rig deliverable. The mesh is named
by relationship rather than by position, which is what makes that
placement free.

## How it works

Touch regions belong to no compile or evaluation phase: `rigExec:touch:*`
appears nowhere under `libs/rigExec`, so they add nothing to the pose walk.
They are read at UI time by the usdview TouchPose plugin, which traverses
the stage for prims typed `RigExecTouchRegions`, keeps the scopes whose
`rigExec:touch:mesh` targets the mesh being touched (a scope naming no
mesh matches whichever mesh is asked for), takes the first of them that
has region children, and flattens those into a single `face -> region`
int array (−1 for unpainted skin).
That table goes to the native mesh in `rigExecImaging`, which keeps a BVH
over the *posed* triangles and refits it whenever the rig publishes a new
pose, so a pick ray hits the deformed skin and resolves to a face, a
region, and the control that region names. Lighting a region creates,
deletes and authors nothing: the imaging plugin publishes a per-face slot
(the region index + 1, `0` for unpainted skin) as a uniform primvar and
the current colours as a constant `vec4[]` table, and swaps the mesh's
surface terminal for a generated shader that runs the original one and
then mixes `table[slot]` into the lit colour — a hover is one
constant-primvar upload, not a resync.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:touch:mesh` | The `UsdGeomMesh` whose faces every region under this scope indexes. Nothing enforces it: a scope with no target is kept for whichever mesh is asked for, which is only safe while the stage holds one character. | no |
| (children) | One `RigExecTouchRegion` per named set. A scope with no region children is passed over. | yes |
| `rigExec:touch:control` | Per region: the control a click inside it selects. Unenforced, but a region without one is dropped by the reader's default rather than drawn inert. | no |
| `rigExec:touch:faces` | Per region: the face indices it owns on the parent's mesh. Unenforced: a region with none is read, owns no face, and can never be hit. | no |

## Parameters

### Touch region (per child prim)

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

### Node parameters

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

## Example

A three-segment strip skinned by an FK chain, its twelve faces divided
into three named regions — `base_touch`, `mid_touch`, `tip_touch`, four
faces each — every one naming the control that poses that segment. The
`TouchPose` scope targets the single `Skin` mesh and carries a three-colour
ramp and the empty, invisible `Overlay` prim the TouchPose exporter ships
beside the regions. Nothing in the file drives the regions: the FK controls
curl the strip and back, and the face sets ride along on the deformed
skin.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\touch_regions.usda
```

## Tips

- Keep regions disjoint. The lookup is one flat `face -> region` array built by scattering each region's faces in namespace order, so a face claimed twice silently belongs to whichever region is written last; the painting tools maintain the partition for you.
- A region with no `rigExec:touch:control` is skipped by the reader rather than drawn dead — an unbound set must not swallow the click that would otherwise reach usdview's own picking.
- Face indices are scattered against the mesh's live face count and anything out of range is dropped without a word, so a region set exported against different topology fails quietly: re-export the regions whenever the mesh's face count changes.

## See also

- [Touch Region](touch_region.md)
- [Control](control.md)
- [FK Chain](fk_chain.md)

---

[RigExec nodes](../index.md)
