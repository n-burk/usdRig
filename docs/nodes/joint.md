# ![Joint](../../icons/joint.png) Joint

*A posed output of the rig: solvers write it, movers read it.*

| | |
|---|---|
| **Node type** | `RigExecJoint` |
| **Example** | [joint.usda](../examples/joint.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

![Joint effect](../gifs/joint.gif)

A joint is where solved posing becomes readable data. Exactly one
solver may pose a joint through `rigExec:joints`, but that frame is not the
last word: any number of pose-phase constraints may then revise the same
joint through `rigExec:moves`, in stack order, before matrix movers read it
to carry geometry. Joints nest in the namespace to form the hierarchy, and
usdview draws each joint as a guide sphere with a cone to every nested
child.

The joint of an armature-based rig, mirroring OpenExec's
IrJointScope exactly: a RigExecXformable specialized so guides can be
drawn (a sphere at the posed origin and one cone to each nested child
joint, purpose guide). Hierarchy is namespace nesting (a child joint is
authored inside its parent joint), and guide links derive directly from
the evaluated parent and child origins.

## How it works

The compiler binds each joint to at most one aggregate solver;
the pose phase evaluates that solver and extracts the joint's element from
its frame array, then runs the authored constraint steps, each reading the
frame the step before it left and writing a revised one over it. A joint
nothing claims is still evaluated — it follows its namespace parent's posed
space with its own rest offset and avars — and a joint that is both claimed
and constrained ends the phase with the constraint's answer. A mover reads
the base frame the solver wrote unless it asks for the `final` phase, which
is the same frame with every pose revision folded in.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| (posed by) | At most one solver's `rigExec:joints` names this joint and supplies its frame; a second claim is a compile error. | - |
| (revised by) | Any number of pose constraints name this joint on `rigExec:moves` and revise its frame in stack order. | - |

## Parameters

### Transform provider

#### `rest:tx`

*Type:* `double`. *Default:* `0`.

#### `rest:ty`

*Type:* `double`. *Default:* `0`.

#### `rest:tz`

*Type:* `double`. *Default:* `0`.

#### `rest:rx`

*Type:* `double`. *Default:* `0`.

#### `rest:ry`

*Type:* `double`. *Default:* `0`.

#### `rest:rz`

*Type:* `double`. *Default:* `0`.

#### `rest:space`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

Bind transform relative to the namespace frame provider's rest frame; always orthonormalized. A provider with no RigExec ancestor resolves against identity, so its rest:space is its local-to-world bind transform.

#### `default:tx`

*Type:* `double`. *Default:* `0`.

#### `default:ty`

*Type:* `double`. *Default:* `0`.

#### `default:tz`

*Type:* `double`. *Default:* `0`.

#### `default:rx`

*Type:* `double`. *Default:* `0`.

#### `default:ry`

*Type:* `double`. *Default:* `0`.

#### `default:rz`

*Type:* `double`. *Default:* `0`.

#### `default:space`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

Local-to-world zero position for posing. Its computed fallback
is compose(default translation/XYZ rotation) * rest * inverse(parent
rest) * parent:defaultSpace. The rest is unaffected by default edits.
A connection is authoritative, including identity. Otherwise a
non-identity authored matrix overrides the computed fallback; an
identity value selects that fallback.

#### `posed:space`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

Final local-to-world transform. For a joint this is
supplied by the compiler's private solver binding (view-free
extraction); it may also be authored or
connected directly. Unwired xformables follow the parent chain
with rest offsets and avars.

#### `posed:defaultSpace`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

Controller-adjusted zero pose; falls back to avars:defaultSpace.
Used by the local-avars pose path before parent motion. A connection
overrides the fallback, as does a non-identity authored matrix.

#### `avars:tx`

*Type:* `double`. *Default:* `0`.

#### `avars:ty`

*Type:* `double`. *Default:* `0`.

#### `avars:tz`

*Type:* `double`. *Default:* `0`.

#### `avars:rx`

*Type:* `double`. *Default:* `0`.

#### `avars:ry`

*Type:* `double`. *Default:* `0`.

#### `avars:rz`

*Type:* `double`. *Default:* `0`.

#### `avars:rspin`

*Type:* `double`. *Default:* `0`.

#### `avars:rotationOrder`

*Type:* `token`. *Default:* `"XYZ"`.

Valid values: `XYZ`, `XZY`, `YXZ`, `YZX`, `ZXY`, `ZYX`.

#### `avars:defaultSpace`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

Zero pose supplied to avar evaluation; falls back to the
computed default:space. A connection or non-identity authored matrix
selects another zero pose.

#### `avars:unitScaleFactor`

*Type:* `double`. *Default:* `1`.

Multiplier converting translation avars into local distance units before the selected default and parent transforms. Rotations and scales are unaffected.

#### `parent:space`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

Selected parent's posed local-to-world space. Falls back to
the nearest namespace provider's computePointFrame; identity when
there is none. Connections (including identity) or a non-identity
authored matrix select a different parent space.

#### `parent:defaultSpace`

*Type:* `matrix4d`. *Default:* `( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )`.

Selected parent's default local-to-world space. Falls back
to the nearest namespace provider's computeDefaultFrame. Connections
(including identity) or a non-identity authored matrix override it.
The avar pose is avars * posed:defaultSpace * inverse(parent:defaultSpace)
* parent:space, in row-vector convention.

### Node parameters

#### `avars:sx`

*Type:* `double`. *Default:* `1`.

Local X scale applied before rotation and translation; finite magnitudes below 1e-4 resolve to signed 1e-4, strict authoring rejects non-finite values, and raw non-finite USD resolves to identity on this axis.

#### `avars:sy`

*Type:* `double`. *Default:* `1`.

Local Y scale applied before rotation and translation; finite magnitudes below 1e-4 resolve to signed 1e-4, strict authoring rejects non-finite values, and raw non-finite USD resolves to identity on this axis.

#### `avars:sz`

*Type:* `double`. *Default:* `1`.

Local Z scale applied before rotation and translation; finite magnitudes below 1e-4 resolve to signed 1e-4, strict authoring rejects non-finite values, and raw non-finite USD resolves to identity on this axis.

#### `purpose`

*Type:* `uniform token`. *Default:* `"guide"`.

Render purpose, overriding UsdGeomImageable's `default`
fallback: joint and solver guides are DIAGNOSTICS, drawn when a
viewer asks for guides.

This is the stock attribute rather than a RigExec-specific token so
that the bounds and the drawing cannot disagree: UsdGeomBBoxCache
classifies a prim's extent by this exact attribute, and the results
scene index stamps the same resolved value onto the guide it
synthesizes. A control keeps the inherited `default` -- it is the
rig's interaction surface, not a diagnostic -- and authoring
`guide` on one moves both its drawing and its bounds together.

#### `guide:radius`

*Type:* `double`. *Default:* `1.0`.

Radius of BOTH guide primitives: the sphere at the posed
origin and every cone drawn to a nested child joint. Without this the
guides use Hydra's fallback radius of 1.0 -- correct at arm scale,
far larger than the geometry on a small asset. Zero or negative draws
no guides at all.

#### `guide:displayColor`

*Type:* `color3f`. *Default:* `(1.0, 0.3, 0.3)`.

#### `guide:displayOpacity`

*Type:* `float`. *Default:* `0.5`.

## Example

A three-joint arm whose wrist is written twice. The FK chain poses
shoulder, elbow, and wrist from three controls as the elbow control curls 70
degrees, then an aim constraint re-aims the wrist at a fixed anchor in the
pose phase. Each panel is skinned rigidly to its own joint and reads the
`final` phase, so the hand keeps pointing at the anchor while the forearm
swings out from under it.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\joint.usda
```

Re-render the GIF above with:

```bat
python docs/render_media.py --page joint
```

## Tips

- Nest joints (Elbow inside Shoulder) so hierarchy, guides, and FK composition all agree — and remember `rest:space` is measured from the parent joint's rest, not from the world.
- Only one solver may pose a joint — a second `rigExec:joints` claim fails the compile with "is posed by two solvers" — but constraints are not solvers: as many as you like can revise that same joint afterwards, each one reading what the previous step left.
- A solver whose output another solver reads is naming joints for their rests, not claiming them, so an IK and an FK chain can both list the chain that their IK/FK blend actually poses.

## See also

- [FK Chain](fk_chain.md)
- [Aim Constraint](aim_constraint.md)
- [Matrix Mover](matrix_mover.md)

---

[RigExec nodes](../index.md)
