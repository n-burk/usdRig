# RigExec examples

Every file is a self-contained, animated stage (frames 1001-1048) with one
`RigExecRig`. View any of them live with:

```
launch_usdview.bat examples\<file>.usda
```

The rigExecUsdview plugin activates automatically for stages carrying a
`RigExecRig` prim and republishes OpenExec-evaluated results on every
timeline change.

Joints and aggregate solvers draw as **guide geometry** (a sphere at each
posed frame origin plus a cone along the aim axis, aligned with OpenExec's
`IrJointScope` guide contract): enable *Display → Display Purposes →
Guide* in usdview to see them. Joints style via `guide:length` /
`guide:displayColor` / `guide:displayOpacity` (unauthored length falls
back to the posed bone length); solvers draw one guide per aggregate
frame element with their own `guide:displayColor`/`guide:displayOpacity`.

## The original arm

- **ArmRig.usda** — the spec §4.5 arm asset: FK + IK + blend, twist
  distribution, ribbon, layered weighted-matrix skinning, blend shape,
  volume correction, guide emission, and a `rigComplexity` variant set
  (`film` / `animation` / `preview`; preview disables the volume-correct,
  ribbon-wrap, and guide-emission movers).
- **ArmShotAnim.usda** — the spec §4.6 shot: references the arm asset,
  selects the `animation` variant, and layers IK animation, an IK/FK
  switch spline, and a driven-weight spline on top.

## Numbered series (one focus area per file)

- **01_FkChainTail.usda** — `RigExecFkChain` in action: four tail controls
  author local rotations through `rigExec:posePoints`; the chain composes
  them down the hierarchy while animated `rigExec:twist` on the base rolls
  the whole tail. Joints index the aggregate result and dense/sparse
  weighted `RigExecMatrixMover`s skin a strip mesh; normals and extent
  are maintained automatically by the compiler.
- **02_TwoBoneIkLeg.usda** — `RigExecTwoBoneIk`: a planted/reaching foot
  effector with a knee pole; an `inputs:stretch` spline ramps 0→1 so the
  out-of-reach pose visibly stretches the chain under
  `clampWithSoftness`. A `RigExecDynamicWeight` modulates painted knee
  weights with an animated driver before skinning.
- **03_IkFkBlendClamp.usda** — `RigExecBlendPointFrames` blending an FK
  chain against a two-bone IK, plus a pose-phase `RigExecFloatMathMover`
  that clamps the deliberately overdriven blend weight (-0.25→1.3) to
  [0, 1] — a mover targeting an exact float property instead of geometry.
- **04_BlendShapeFace.usda** — `RigExecBlendShapeMover` with two
  independently composed `RigExecBlendInput` channels: Smile carries an
  in-between `RigExecBlendSample` at activation 0.5 plus a full target;
  BrowRaise a single target. A constant-representation weight object
  scales the summed delta.
- **05_TwistRibbonSpine.usda** — `RigExecRibbon` sampling an animated
  native BasisCurves driver with rotation-minimizing frames; a
  `RigExecCurveMover` in ribbon mode transports the spine strip via its
  `primvars:st` bind coordinates and a second one emits the frame origins
  as guide points. A `RigExecTwistDistribution` between the FK root and
  chest joints drives a mid-spine joint that skins a fin card, making the
  interpolated twist visible.
- **06_LatticeBulge.usda** — `RigExecLatticeMover` with a 2x2x3 Bernstein
  cage (default-time points are the bind cage, timeSamples the posed
  cage; ordering is x-fastest, then y, then z) bulging a slab, then a
  `RigExecSmoothMover` and a `RigExecVolumeCorrectMover` reacting to the
  bulge.
- **07_SurfaceDrape.usda** — `RigExecSurfaceMover` in both `project` and
  `attach` modes: two sticker patches are pulled onto a ground mesh whose
  wave animates through native points timeSamples (read at the `base`
  phase), so the stickers ride the moving surface.
- **08_AimEyes.usda** — pose-phase `RigExecAimConstraint`s: two sibling
  constraints re-aim the eye joints' z-axes at an animated look-at
  control with ramped weights; geometry-phase matrix movers read the
  posed joints at the `final` phase to carry the eye cards.
- **11_VolumeWeights.usda** — volumetric weight objects: nothing is
  painted. A `RigExecSphereWeight` authored *inside* the shoulder joint
  rides it with nothing wired (a volume weight is a `RigExecXformable`,
  so it follows its namespace-parent's posed space); a
  `RigExecCombineWeight` multiplies a *bounded* `RigExecPlaneWeight`
  gradient by a second sphere to clip the mid joint's influence to one
  side; and a `RigExecCurveWeight` measures distance to a driver curve
  through a hand-drawn falloff spline. `inputs:falloffMax` is animated,
  so the shoulder's influence visibly widens over the shot. The plane
  shows the two axes apart: `inputs:falloffMin`/`falloffMax` run *along*
  `rigExec:planeAxis` and slide the drawn surfaces, while
  `inputs:extentU`/`extentV` size them *across* it and, under
  `rigExec:planeBounds = "bounded"`, stop the field at that rectangle. See
  [`docs/volume-weights.md`](../docs/volume-weights.md).

## Authoring conventions the engine expects

- One `RigExecRig` per stage (the usdview plugin and imaging bridge
  activate the first one found). The rig declares no membership lists:
  controls, joints, and movers are discovered from the namespace beneath
  it. At least one `RigExecJoint` must exist under the rig — that is what
  the rig publishes.
- Solvers live under `<rig>/Solvers`, movers under `<rig>/Movers`; mover
  order is the post-order namespace walk (deepest child applies first),
  with `reorder nameChildren = ["Pose", "Geometry"]` keeping pose movers
  ahead of geometry movers.
- Two movers writing the same target must be nested, never siblings.
- Joints and controls follow OpenExec's Ir contract exactly (no
  deviations): both are `RigExecXformable`s with orthonormal
  local-to-world `matrix4d rest:space`, avars for animation (rotations
  in degrees; `avars:rspin` is the twist channel). Solver output is
  **view-free**: a solver owns an ordered `rel rigExec:joints` list and
  the compiler binds each listed joint to one element of that solver's
  aggregate result. The binding is internal — it exists only inside the
  compiler's private evaluation stage, is not a schema property, and is
  never authored on your stage (there is no `RigExecPointFrameView`
  either). Authoring the solver's `rigExec:joints` list is the whole job. Joint hierarchy is prim nesting;
  an xformable with only a `rest:space` follows its rest (or its
  namespace parent) — see `08_AimEyes` for static joints posed
  downstream by movers. Guide cones use the authored `guide:length`
  only, with Ir's unit-radius primitives.
- Typed geometry movers (matrix, blend shape, smooth, volume correct,
  lattice, surface, curve) write native `UsdGeomPointBased` `point3f[]`
  `points` attributes only; smooth/volumeCorrect/lattice (like blend
  shape and matrix movers) take exactly one canonical target in v0.1.
- Normals and extent are never authored as movers: the compiler
  synthesizes derived-maintenance applications for every written points
  target whose gprim authors the property, reading the final
  same-generation points. Vertex-normal recomputation is mesh-only —
  authored normals on a moved `Points`/`BasisCurves` target fail compile
  rather than go silently stale. Synthesis is unconditional: a gprim that
  authors normals/extent always has them maintained, so shedding that work
  for a preview LOD means not authoring the properties on that gprim.
- Bind-time (rest) values — lattice cages, ribbon driver curves, blend
  target points — are captured at `UsdTimeCode::Default()`, so author a
  default opinion alongside any timeSamples.
- In v0.1-alpha the ribbon solver derives its frames from the driver
  curve only (posed vs bind-time rest); `startFrame`/`endFrame`/
  `twistFrames` relationships are authored contract but not yet folded
  into the transported frames.
