# Curvenets and profile deformation

A curvenet is a connected set of cubic curves whose control points drive a
surface deformation. RigExec implements this using the published method in
de Goes, Sheffler, and Fleischer, [Character Articulation through Profile
Curves](https://doi.org/10.1145/3528223.3530060) (2022). See the
[method references](../references.md) for related work and source links.

## Authoring a net

`RigExecCurvenet` is a point-based prim. Its native `points` array holds the
shared control-point pool; `rigExec:splineIndices` contains four indices per
cubic spline. Shared indices define connections between splines.

| Property | Meaning |
|---|---|
| `points` | Projection-pose control points |
| `rigExec:splineIndices` | Four control-pool indices per cubic spline |
| `rigExec:basis` | `bezier` or `catmullRom` |
| `rigExec:samplesPerSpline` | Sampling density; default 5 |

For Bezier splines, the first and last indices are knots and the middle two
are handles. Catmull-Rom knots use entries 1 and 2. Existing geometry movers
can animate the point pool.

## Deforming a mesh

`RigExecCurvenetMover` reads one net through `rigExec:curvenet` and targets the
mesh's points through the standard mover API. `rigExec:restPose` selects
`projection` or `preceding`, and `rigExec:curvenetReadPhase` selects `base`,
`preceding`, or `final`. A weight object or `inputs:defaultWeight` controls
the deformation envelope.

Binding cuts mesh elements along projected curves and prepares sparse systems
for deformation. Posing the net reuses the bind when topology and neutral
geometry are unchanged. Curve-side frames support discontinuities across a
curve rather than averaging both sides together. Components not reached by
the net remain at rest and are diagnosed.

## Deformation-relative adjustments

`RigExecCurvenetAdjustment` exposes control channels for a knot. The adjuster
mover applies them in the frame of its incoming point revision. At an
intersection, frames follow incident tangents. Along curves, transported
rotations from neighboring intersections blend by arc distance. A straight
isolated curve has no observable twist and uses the shortest rotation.

```python
control = builder.add_curvenet_adjustment("Cheek", net, knot_index=4)
control.set_avar_translation(0, 0.1, 0)
tangent = control.add_tangent("Outgoing", index=20)
tangent.set_avar_translation(0, 0.05, 0)
chain.add_curvenet_adjuster_mover("Adjust", [control], target=net.path + ".points")
```

Knot controls can move incident Bezier handles. Tangent adjustments are
children of knot controls and inherit their adjusted frames. Catmull-Rom
controls have no separate Bezier handles. Invalid bindings, duplicate point
controls, or non-finite transforms fail validation.

## Viewer tools

Open the curvenet panel from the RigExec menu. It authors the net and binding
on the stage and maintains a design-pose display in the session layer.

| Mode | Interaction |
|---|---|
| Draw | Place knots on the mesh; drag to set a tangent; connect to existing knots |
| Select | Pick knots or handles, or select a group with a marquee |
| Move | Move selected controls, with surface projection where enabled |
| Tangent | Edit, flatten, or mirror tangent handles |

Topology operations include weld, break, split, merge, reverse, close, and
delete. Bind to Mesh creates the mover and reports binding diagnostics.
The imaging bridge publishes evaluated curves separately from the panel's
neutral authoring display.

## Limits and verification

Adjustment frames currently feed guides and viewport manipulation, but cannot
feed the earlier pose pass. Solver/constraint inputs and transform providers
that depend on those frames are rejected at compilation. Authored scalar
adjustment channels remain readable.

Rest or topology changes rebuild the affected bind; pose-only edits reuse its
cut mesh and factorization. The sampled posed-net guide uses the generation's
sample nearest zero, while mesh and transform outputs carry motion samples.
Profile deformation has no separate scalar parity oracle; its numerical
behavior is covered by focused curvenet tests.

Code lives in `libs/rigExecMath/curvenet.cpp`, `cutMesh.cpp`,
`profileMover.cpp`, and the curvenet mover adapters. For scalar fields and
transported sculpt detail, see [weights and surface detail](curvenet-weights-and-detail.md).
