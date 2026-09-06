# RigExec examples

Every file is a self-contained, animated stage (frames 1001-1048) with one
`RigExecRoot`. View any of them live with:

```
bin/launch_usdview.bat examples\<file>.usda
```

The rigExecUsdview plugin activates automatically for stages carrying a
`RigExecRoot` prim and republishes OpenExec-evaluated results on every
timeline change.

Joints and aggregate solvers draw as **guide geometry**: each joint draws a
sphere at its posed origin and a cone to every nested child joint, while
solvers draw from their aggregate frame aim axes. Enable *Display → Display
Purposes → Guide* in usdview to see them. Joints style via `guide:radius` /
`guide:displayColor` / `guide:displayOpacity`; solvers draw one guide per
aggregate frame element with their own radius, color, and opacity.

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
  weights with an animated driver before skinning. Its bone lengths are
  left unauthored, so they are measured from the bound joints' rests on
  every evaluation: move a joint's rest and the limb re-proportions
  itself. `03` and `ArmRig` bind their joints through
  `RigExecBlendPointFrames` instead, so their solvers have no rests to
  measure and must author the two absolute lengths.
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
- **09_PropertyMathMovers.usda** — the property output domain:
  `RigExecFloatMathMover`, `RigExecVec3fMathMover`, and
  `RigExecMatrixMathMover` each revise an exact scalar/vector/matrix
  attribute instead of a `point3f[]` array. A property chain resolves off
  the authored stage before exec runs, so its result is handed back to
  exec as the attribute's value — which is how `03`'s clamped weight
  reaches `RigExecBlendPointFrames`. A witness card skinned by a matrix
  mover sits alongside, so one rig shows both domains.
- **10_AimXformTurret.usda** — a `RigExecAimConstraint` driving a plain
  `UsdGeomXform`, with the barrel mesh parented underneath: one matrix
  for a rigid object instead of point-deforming every vertex at constant
  weight 1 (the counterpart to `08`'s skinning model). The engine
  publishes world-space transforms and dirties the driven subtree itself,
  because RigExec installs downstream of the chain's flattening index.
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
- **12_CurvenetProfile.usda** — curvenets and the Profile Mover
  (de Goes, Sheffler & Fleischer, SIGGRAPH 2022). Three profile rings
  joined by four longitudinal rails around a tube; every ring knot is
  shared by two ring spans and two rails, which is what makes it an
  *intersection* and lets §3 deduce the frames, widths and twist that
  nobody authors. The net's knots are posed by an ORDINARY
  `RigExecMatrixMover` driven by an FK joint through a weight object —
  76 pool points against 208 tube vertices the rig never mentions — and
  `RigExecCurvenetMover` propagates that onto the surface. Re-mesh the
  tube and the same net still articulates it. **Generated** by
  `build_curvenet_example.py`; edit that, not the `.usda`. See
  [`docs/curvenet.md`](../docs/curvenet.md).
- **13_ReadPhases.usda** — read phases as property metadata. A Slab is
  deformed through a cage that is itself deformed by two movers, and the
  lattice declares which cage it wants. `base` leaves the slab alone,
  `/…/Movers/Cage/CageLift` gives the lifted-but-not-twisted cage, and
  `final` gives both — three different results from one rig with no other
  edit. Cyclic phase reads are rejected at compile.
- **rigexec_flat.usda** — the smallest rig that exists, and a flattened
  capture of the shape an interactive session produces: one aim
  constraint, no joints at all, and both ends plain `UsdGeomXformable`s.
  A rig's outputs are joint frames, driven transforms, and revised
  properties in any combination — a joint is one of them, not a
  precondition.

## Authoring conventions the engine expects

- One `RigExecRoot` per stage (the usdview plugin and imaging bridge
  activate the first one found). The rig declares no membership lists:
  controls, joints, and movers are discovered from the namespace beneath
  it. At least one `RigExecJoint` must exist under the rig — that is what
  the rig publishes.
- Solvers live under `<rig>/Solvers`, movers under `<rig>/Movers`; mover
  order is a reverse-sibling post-order walk: descendants apply before their
  mover parent and sibling rows apply bottom-to-top. For example,
  `reorder nameChildren = ["Geometry", "Pose"]` displays Pose below Geometry,
  so pose movers execute first and geometry movers can consume final pose.
- Two movers writing the same target may be siblings or nested; the final
  composed hierarchy always supplies their deterministic stack order.
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
  downstream by movers. Joint guide cones derive their direction and length
  from evaluated parent/child origins; leaf joints draw only their sphere.
- Typed geometry movers (matrix, blend shape, smooth, volume correct,
  lattice, surface, curve) write native `UsdGeomPointBased` `point3f[]`
  `points` attributes only; smooth/volumeCorrect/lattice (like blend
  shape and matrix movers) take exactly one canonical target in v0.1.
- A mover's output is not restricted to a joint frame. Three domains
  exist and a rig may publish any combination of them, including a rig
  with no `RigExecJoint` at all: **geometry** (the point chains above),
  **transforms** (`RigExecAimConstraint` on any `UsdGeomXformable` — its
  revised matrix is published and the subtree rides along), and
  **properties** (the three math movers, over an exact `float`,
  `float3`/`vector3f`/`point3f`/`normal3f`/`color3f`, or `matrix4d`
  attribute — exactly one target each, and the target's value type must
  match the mover's static type). A rig with neither joints nor movers is
  still rejected: it publishes nothing.
- Property-chain results reach every consumer. A computation reading the
  attribute gets an exec value override; packet assembly, which never touches
  exec, gets the same value through the evaluator's resolved-input set. Both
  are filled from the one chain result before any input is read.
- **Read phases** decide *which revision* of an input a mover consumes, and
  are authored as metadata on the relationship (or attribute) that names it:

  ```
  rel rigExec:cage = </Asset/Geom/Cage> (
      rigExecReadPhase = "final"
  )
  ```

  `base` (the authored value, and the default), `preceding` (the value just
  before this mover, in its own chain), `final` (after every writer), or an
  absolute prim path — the value as of when the reverse-sibling post-order
  walk finished with that prim. A mover path means "right after it applied";
  a grouping `Scope` means "after everything beneath it", because post-order
  visits a parent last. The field is `rigExecReadPhase`, not
  `rigExec:readPhase`: USD metadata names take no namespace, and metadata
  follows the target assignment rather than preceding it. The older
  role-named attributes (`rigExec:cageReadPhase`, …) still work; metadata
  wins when both are authored. Editing a phase is structural. Chains are
  evaluated in dependency order and a cyclic phase read fails the compile.
  Bind-time (rest) reads always take the authored value — a phase has no
  meaning for the neutral pose a deformation is measured against.
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
