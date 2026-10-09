# ![Curve Mover](../../icons/curve_mover.png) Curve Mover

*Transports points through a solver's frame array, or emits its frame origins.*

| | |
|---|---|
| **Node type** | `RigExecCurveMover` |
| **Example** | [curve_mover.usda](../examples/curve_mover.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

![Curve Mover effect](../gifs/curve_mover.gif)

Frame transport for geometry. In `ribbon` mode each moved point
carries a bind parameter that picks a spot in the driver frame array, and
the point is mapped by the rest-relative rigid transform read there — so a
dense mesh spread across the parameter bends, while a prop whose every
vertex shares one parameter rides rigidly. In `emitGuidePoints` mode the
mover skips the bind and writes the frame origins straight out as points.
Nothing in either mode samples a curve: the frames are the input, and any
solver that publishes a frame array — a ribbon, an FK chain, spline IK, a
twist distribution — can supply them.

Reads standard basis-curve properties for wire, spline-IK,
ribbon, and curve-coordinate movement and writes an exact native
UsdGeomPointBased points property (spec section 7.5).

## How it works

The mover runs as one revision on its `rigExec:moves` target's point
chain, reading the driver solver's frames after the pose phase has solved
them. It builds one rigid map per element from that element's rest
landmarks to its posed landmarks, then places each point by clamping its
bind u to [0, 1], scaling it across the array, and blending the two
neighbouring maps' results — so the array's cardinality, not a curve, sets
the resolution of the transport. Guide emission needs no bind at all: it
copies frame origin `i` to point `i`, and fails the revision if the counts
differ. The compiler maintains the moved mesh's normals and extent
afterwards.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:driverFrames` | Any solver publishing a frame array (ribbon, FK chain, spline IK, twist distribution). | yes |
| `rigExec:bindCoordinates` | Per-point `primvars:st`; u picks the spot in the frame array. | ribbon mode |
| `rigExec:moves` | Exact native points property to write. | yes |
| `rigExec:driverCurve` | Unread on this node — the frames already carry the curve's shape. | no |

## Parameters

### Common mover envelope

#### `rigExec:moves`

*Relationship.*

Reserved write-set relationship: exact prim/property targets.

#### `inputs:enabled`

*Type:* `bool`. *Default:* `true`.

Shape-preserving enable. Disabled movers pass their preceding
revision through unchanged (value-only edit).

#### `inputs:defaultWeight`

*Type:* `float`. *Default:* `1`.

Normalized common mover envelope in [0, 1]. With no bound
rigExec:weightObject it broadcasts over every logical element: zero
passes the incoming value through bit-for-bit and one applies the
mover's full-strength result.

#### `rigExec:weightObject`

*Relationship.*

Optional compatible total weight field, at most one target.
When bound, the weight object's values (including its own sparse or
constant fallback) supply the common envelope instead of
inputs:defaultWeight. Target, domain, and cardinality must match each
mover application; an incompatible binding is a compile error. A
multi-target mover may bind only a constant operation envelope whose
weightTarget is the mover prim itself; that one value broadcasts to
every application of the atomic mover.

### Node parameters

#### `rigExec:driverCurve`

*Relationship.*

#### `rigExec:driverFrames`

*Relationship.*

#### `rigExec:bindCoordinates`

*Relationship.*

#### `rigExec:mode`

*Type:* `uniform token`. *Default:* `"ribbon"`.

Valid values: `ribbon`, `emitGuidePoints`, `wire`.

ribbon transports points along a ribbon aggregate's frames;
emitGuidePoints writes its sample positions; wire moves each point by
the displacement of rigExec:driverCurve (a UsdGeomNurbsCurves) at the
parameter it was bound to: p' = p + f(d) * (C(u) - C0(u)), with (u, d)
per point from rigExec:bindCoordinates, C the curve's posed control
points at rigExec:driverCurve's read phase and C0 its authored
ones. With
a sparse weight object the bind table may be sparse too: one (u, d)
per weighted point, in the weight object's index order.

#### `rigExec:driverTransforms`

*Relationship.*

wire: move the driver curve's control points by matrix
providers instead of reading its points: one provider per unique
control point, or one for all. A periodic curve's repeated points
take their unique point's provider. The posed point is
C0 + w (M C0 - C0), M the provider measured against its entry in
rigExec:driverTransformSpaces and w its inputs:driverWeights entry.
No chain runs on the curve, so its points need no phased read;
rigExecReadPhase metadata on this relationship chooses base or
final for the providers.

#### `rigExec:driverTransformSpaces`

*Relationship.*

wire: the space each driver transform is measured against,
parallel to rigExec:driverTransforms, or one for all, or none.

#### `rigExec:driverDeltaFrame`

*Type:* `uniform token`. *Default:* `"local"`.

Valid values: `local`, `posed`.

wire: WHICH FRAME the driver's offset from its space is
applied in. Only meaningful with rigExec:driverTransformSpaces.

local is what the measurement M = T * S^-1 gives, and in USD's
row-vector convention that product is exactly the driver's LOCAL
matrix: world = local * parent, so T * S^-1 cancels the space
entirely. The curve's control points are world positions, so
applying it to them uses a local offset as though it were a world
one -- the offset points wherever it pointed at the bind pose, no
matter where the space has since been carried.

posed conjugates that offset into the space's current frame,
S^-1 * T, which is S^-1 * local * S. The offset then rides the
space: same measurement gives 69.93 degrees against the arm's
70.00, with the length preserved exactly.

Both are IDENTITY when the driver sits at its space, at rest and
posed alike, so neither makes the curve move merely because the
space moved -- that is the skin's job and would be a double
transform. They differ only once an animator touches the control.

local remains the default because the shipped face rig was built
and measured against it; posed is what a wire riding a deforming
limb wants. Set per mover, so the two can coexist while rigs
migrate.

#### `rigExec:pointFrame`

*Type:* `uniform token`. *Default:* `"rest"`.

Valid values: `rest`, `posed`.

wire: WHICH FRAME the points this wire moves are
already in when it runs.

A wire adds a displacement read off its driver curve to the
points it moves. The curve, the bind distances measured against
it and the mesh rest points are all authored at the ASSET'S
scale; a skin is what carries those points out of it. So the
same displacement means two different things depending on where
in the chain the wire sits, and only the asset knows which.

rest is a wire that runs BEFORE the skin on its target. Its
displacement stays in the asset's units, because the skin will
apply the rig's scale to it afterwards and applying it here too
applies it twice.

posed is a wire that runs AFTER it, adding to points a skin has
already carried into the rig's posed frame. Its displacement is
carried there too: the scale the driver measurement took out
goes back on, and nothing else does, so the wire moves the posed
mesh as far as the posed rig moved.

The two corrections are in OPPOSITE directions, which is why one
blanket rule cannot serve both families and why the wire has to
say. Which one a given wire is, is not a rigging choice -- it is
where the compiler put it relative to its skin -- so a build tool
should author it from the compiled mover order rather than from a
list kept by hand.

Both are IDENTICAL while nothing above the rig is scaled,
because the scale in question measures one. A wire that names
neither posed frame keeps the plain measurement bit for bit.

#### `rigExec:space`

*Relationship.*

wire: the prim whose movement away from its rest carries
the whole rig -- normally a TRS master. Only meaningful with
rigExec:driverTransforms and rigExec:pointFrame = "posed".

A posed wire adds a displacement computed in the uncarried frame
to points a skin has already carried. With a space named, both
control polygons are transformed by that space's rest->pose map
(its computeMatrix) before the wire is evaluated, so the
displacement is carried with the points; the scale-only correction
of rigExec:pointFrame is then not applied beside it. Read at the
phase declared on rigExec:driverTransforms.

#### `inputs:driverWeights`

*Type:* `float[]`. *Default:* `[]`.

wire: per unique control point weight of its driver
transform, or one for all; empty is 1.

#### `rigExec:driverBaseTransforms`

*Relationship.*

wire: an optional base motion applied to the curve AND its
rest before the driver transforms, as a wire whose base curve rides
the same deformers as its driver: B = C0 + wb (Mb C0 - C0), then
C = B + w (M B - B), and the wire deforms by C - B. Same entry rules
as rigExec:driverTransforms.

#### `rigExec:driverBaseTransformSpaces`

*Relationship.*

wire: the spaces of rigExec:driverBaseTransforms.

#### `inputs:driverBaseWeights`

*Type:* `float[]`. *Default:* `[]`.

wire: per unique control point weight of the base motion.

#### `inputs:dropoffDistance`

*Type:* `float`. *Default:* `0`.

wire: distance over which a point's pull from the curve
falls to zero, as 1 - smoothstep(0, dropoffDistance, d) with d the
point's rest distance from the curve. 0 applies no falloff.

## Example

Three controls stacked up Y are the only animation, and an FK chain
that claims no joints turns them into a three-element frame array. One
curve mover spreads a 45-point streamer -- five columns across, nine rows
up -- over the whole u range so it curls and twists between the frames, a
second pins all four corners of each card to a single u so the cards ride
rigidly at 0.15, 0.5 and 0.85, and a third in `emitGuidePoints` mode
writes the same three frame origins out as a gold guide curve. The
streamer sits wholly at negative x and the cards at positive x, so the
gold line up the middle is the third mover's output on its own -- the
frame origins themselves, one point per element. There is no ribbon, no
joint and no driver curve in the file.

Open it live with:

```
bin/usdview.sh docs/examples/curve_mover.usda
bin\launch_usdview.bat docs\examples\curve_mover.usda
```

Re-render the GIF above with:

```bat
python docs/render_media.py --page curve_mover
```

## Tips

- Any frame-publishing solver drives this mover — an FK chain that claims no joints is a legal frame source, and the array's element count is the transport resolution.
- `emitGuidePoints` writes one point per frame: give the target exactly as many points as the solver has elements or the revision fails and passes through unchanged.
- Bind u is clamped and scaled across the array, so equal u on every vertex of a prop rides one spot rigidly and a spread of u bends; u = 0 rides the first element's map exactly, u = 1 the last.

## See also

- [Ribbon](ribbon.md)
- [FK Chain](fk_chain.md)
- [Matrix Mover](matrix_mover.md)

---

[RigExec](../index.md)
