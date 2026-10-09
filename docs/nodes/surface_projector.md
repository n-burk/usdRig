# ![Surface Projector](../../icons/concept.png) Surface Projector

*Publishes a frame riding a deforming surface to its shader, as a matrix primvar.*

| | |
|---|---|
| **Node type** | `RigExecSurfaceProjector` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

A surface projector hands a shader a frame that rides a
deforming surface — the canonical case is an iris and pupil projected onto
an eyeball that the lids, the socket and the look all deform. It casts a
ray at the surface's rest points to pick a point on it, follows that point
onto the surface's final points, and publishes the motion as a constant
`matrix4d` primvar on the surface for the material to read. It writes
nothing to the stage and moves no point; the geometry is exactly what the
surface's other movers make it, and the frame reaches only the renderer.

Publishes a frame that rides a deforming surface to the
renderer, as a constant matrix primvar on that surface. It writes no
stage value and moves no point: the surface named by rigExec:moves
keeps exactly the geometry its other movers produce, and the frame
reaches only imaging and other consumers of the evaluated pose.

A ray in the surface's object space picks a MATERIAL point on the
surface's base points -- a triangle and a place inside it. The frame
there (its position, the interpolated vertex normal, and rigExec:rayUp
for roll) is evaluated again on the surface's final points, after
every mover on the surface's chain has run, and the motion between
the two frames is the projector's own. rigExec:projectionMode chooses
whether the posed frame follows that material point or re-casts the
ray at the posed surface.

When rigExec:sources names a frame provider, the ray moves with it,
and in material mode the source's rotation is applied as the look (a
re-cast already carries it). The published matrix, in row-vector
convention, is

rigExec:shaderOffset * look * delta

where delta is the hit frame's rest-to-posed motion, scaled about the
rest hit by the source's and rigExec:space's uniform scale.
rigExec:shaderDialSources optionally packs up to sixteen scalar
controls into a second matrix primvar for the same shader.

Both primvars are derived targets of the surface's point chain: they
are recomputed when the chain's final points or a source frame
change, and the dynamic path, the baked program, the frame cache and
the .rigexec runtime compute them with one kernel.

## How it works

The projector is compiled into a derived target of the surface's
point chain, beside the normals and extent the compiler already keeps
current, so it runs after every mover on that surface and again whenever
the chain's final points or one of its source frames change. The ray —
`rigExec:rayOrigin` and `rigExec:rayDirection` in the surface's object
space — is moved by the `rigExec:sources` frame, measured against
`rigExec:sourceSpace` so that motion common to the eye and its socket
cancels. A Möller–Trumbore cast against the base points names a triangle
and a place inside it; the frame there (position, interpolated vertex
normal, `rigExec:rayUp` for roll) is evaluated on the base and the final
points, and `rigExec:projectionMode` chooses whether the posed frame
follows that material point or re-casts at the posed surface. The
published matrix is `rigExec:shaderOffset * look * delta`. Up to sixteen
scalar `rigExec:shaderDialSources` are packed into a second matrix primvar
for the same material. The dynamic path, the baked program, the frame
cache and the `.rigexec` runtime all run one header-only kernel.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:moves` | The surface's `points` property; the primvars are published on that prim. | yes |
| `rigExec:sources` | The frame provider that aims the ray — normally the eye's bind joint. | no |
| `rigExec:sourceSpace` | The sibling space the source is measured in — normally the socket the bind joint hangs from. | no |
| `rigExec:space` | The master whose motion carries the whole rig, taken back off the rest ray. | no |
| `rigExec:shaderDialSources` | Scalar properties (usually control avars) packed into `rigExec:shaderDialPrimvar`. | no |

## Parameters

### Node parameters

#### `rigExec:space`

*Relationship.*

The prim whose movement away from its rest carries
the whole rig -- normally a TRS master.

The rest ray is built from the source's BASE frame, which still
composes through the masters' posed avars, while the base points
it is cast at carry nothing. Naming the space takes its
rest-to-posed matrix back off the rest ray, so the cast meets the
surface wherever the master has moved the rig, and puts the
master's uniform scale back on the published matrix. A master at
its rest contributes identity, and a rig naming no space is
unchanged. Distinct from rigExec:sourceSpace, which shapes the
ray's own motion rather than undoing the rig's carry.

#### `rigExec:sourceSpace`

*Relationship.*

The SIBLING SPACE rigExec:sources is measured against --
for an eye, the socket bone its bind joint hangs from.

With it, the source's motion relative to this space is applied to
the authored rigExec:rayOrigin, rigExec:rayDirection and
rigExec:rayUp. Motion common to both frames -- the head, the face
-- cancels, so moving them carries the projector and only the
source's own motion aims the ray. The same space is used for the
look term, so turning the head does not read as the eye looking.

Without it, the source's posed frame IS the ray: its translation,
Z axis and Y axis, mapped into the surface's object space through
the surface's authored transform, which is read once at compile
and so must not vary with time.

#### `rigExec:projectionMode`

*Type:* `uniform token`. *Default:* `"material"`.

Valid values: `material`, `reproject`.

WHICH SURFACE the posed frame is read from.

material (the default) casts the ray ONCE, at the base surface,
and the posed frame is where that same material point ended up.
The projector then rides the skin: a deformer that slides the
surface along the ray carries the projector with it.

reproject casts the ray AGAIN at the final surface, so the frame
is wherever the ray meets the deformed surface now, and a surface
sliding along the ray changes nothing. The rest frame is still
the first cast's, because that is the projector's own placement
and rigExec:shaderOffset is quoted against it. A re-cast that
misses falls back to the material point, with a diagnostic.

#### `rigExec:rayOrigin`

*Type:* `double3`. *Default:* `(0, 0, 0)`.

Ray origin in the target's own object space. The default
is the object origin, which for a mesh authored about its own
centre is where a look vector would start.

#### `rigExec:rayDirection`

*Type:* `double3`. *Default:* `(0, 0, 1)`.

Ray direction in the target's own object space; need not
be normalized. The hit is the nearest intersection strictly in
front of the origin, tested two-sided because a ray starting
inside a closed surface leaves through a back-facing polygon.

#### `rigExec:rayUp`

*Type:* `double3`. *Default:* `(0, 1, 0)`.

Fixes the roll the normal alone cannot. Orthogonalized
against the hit normal; parallel to it (or degenerate) falls back
to the least-aligned principal axis, so a frame always resolves
rather than failing at a pole.

#### `rigExec:sources`

*Relationship.*

Frame the projector is anchored to -- the eye's bind
joint, normally. Its posed frame is what a look-at reaches the
iris through: the control turns the joint, the joint turns the
projector, and the iris goes with it. Without this the projector
only follows the surface's deformation and a look-at moves
nothing.

#### `rigExec:shaderOffset`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

The projector's own placement relative to the source
frame, pre-multiplied into the published primvar. The rig hangs
the projector off its frame with a scale -- the radius the
shader's unit eyeball is quoted against -- and that scale has to
travel with the matrix, because the surface prim's transform is
its placement and no longer carries it.

#### `rigExec:shaderPrimvar`

*Type:* `uniform token`. *Default:* `"eyeProjector"`.

Name of the constant matrix primvar published on the
surface prim carrying the projector's frame. A shader reads it as
HdGet_<name>(), which for a glslfx sourceAsset requires the name
to appear in that shader's "attributes" configuration block --
Storm filters every other primvar out before the shader sees
it.

#### `rigExec:shaderDialSources`

*Relationship.*

Scalar properties an animator turns, published to the
surface prim's shader. Ordered: entry i lands in slot i of the
matrix named by rigExec:shaderDialPrimvar, row i/4 column i%4, so
sixteen dials fit and the shader reads a fixed slot.

Packed into a matrix rather than published as sixteen float
primvars because a matrix primvar is a road that already exists
end to end -- the projector's own frame travels it -- and because
one primvar with one dirty flag is cheaper to push per frame than
sixteen. The shader unpacks it; nothing else needs to know.

How a dial reaches a shader at all: the value lives on a control
as an ordinary avar, so it keys, it shows in the avar editor, and
it undoes like any other channel.

#### `rigExec:shaderDialPrimvar`

*Type:* `uniform token`. *Default:* `""`.

Name of the constant matrix primvar carrying the packed
rigExec:shaderDialSources. Empty publishes nothing. Same rule as
rigExec:shaderPrimvar: a glslfx sourceAsset sees the name only if
its own "attributes" block lists it.

## Example

The biped's eyes are the worked example
([Biped_eyes.usda](../../examples/biped/Biped_eyes.usda)). Each eye mesh
has one projector whose source is the eye's bind joint and whose source
space is its socket, so turning the head carries the iris and only the
eye's own look aims it. The material, a glslfx shader in
`examples/biped/shaders`, reads `eyeProjector` to place the iris and
pupil and `eyeDials` for the pupil and iris sizes and offsets, which are
avars on each eye control. Nothing on the eye mesh moves; open the stage
in `usdview` and turn an eye control to watch the iris slide over the
ball while the ball itself stays still.

## Tips

- A glslfx material sees a primvar only if its own `attributes` block names it: Storm filters every other primvar out before the shader runs.
- The projector does not show in a GIF of the plain viewport: its whole output is a primvar, so it is visible only through a material that reads it.
- Name `rigExec:sourceSpace`. Without it the ray is the source's world frame mapped through the surface's authored transform. That transform must be static either way: a projector onto a surface whose transform is animated fails to compile.
- Use `reproject` when a deformer slides the surface along the look (a socket stretch dragging the eyeball) and the iris should stay on the line of sight; `material` keeps it on the same skin.

## See also

- [Surface Mover](surface_mover.md)
- [Aim Constraint](aim_constraint.md)
- [Control](control.md)

---

[RigExec](../index.md)
