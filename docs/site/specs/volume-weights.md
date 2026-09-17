# Volumetric weight objects

Spec §4.1, volumetric extension. Added 2026-08-07.

A weight object publishes a total scalar field over the logical elements
of one exact prim or property target. Until now RigExec had two: an
authored table (`RigExecStaticWeight`) and a scalar-driven modulation of
one (`RigExecDynamicWeight`). Neither knows where anything *is* — a
painted map is the only way a point gets a weight, and painting is the
one thing a composition-native, code-authored rig cannot do.

This adds four more, three of which **generate** a field from a placed
volume:

| Type | Base | Distance function |
|---|---|---|
| `RigExecVolumeWeight` | abstract, `RigExecXformable` | — |
| `RigExecSphereWeight` | `RigExecVolumeWeight` | `\|(px/sx, py/sy, pz/sz)\|` |
| `RigExecPlaneWeight` | `RigExecVolumeWeight` | signed `p[axis]`, optionally clipped to a rectangle |
| `RigExecCurveWeight` | `RigExecVolumeWeight` | distance to the curve's polyline |
| `RigExecCombineWeight` | `RigExecWeightObject` | folds other weight objects |

## The remap

Every volume shares one distance-to-weight remap:

```
u = clamp01((d - falloffMin) / (falloffMax - falloffMin))
u = lerp(u, 1 - u, invert)
w = strength * Curve(1 - u)
```

so the field is fully ON at `inputs:falloffMin` and fully OFF at
`inputs:falloffMax` — for a sphere they read directly as inner and outer
radii.

- `falloffMax < falloffMin` is legal and flips the ramp. The signed
  denominator does it; there is no special case.
- `falloffMax == falloffMin` is the one degenerate case, and means a hard
  step at that distance.
- `inputs:invert` is a **float**, not a bool, so it can be animated or
  driven like any other avar. It lerps between the two ramps, so an
  animated invert sweeps through a flat field at 0.5 rather than popping.
- `inputs:strength` is deliberately **not clamped by the kernel**.
  `rigExec:rangePolicy` is the authority on out-of-range weights, and
  clamping in the kernel would hide a strict-policy violation. Volume
  weights default to `clamp` rather than the authored types' `strict`,
  because strength and invert are animatable and an artist scrubbing
  strength past one should see the field saturate, not invalidate the rig
  mid-drag.

`Curve` is a lookup table baked once per binding epoch, from either a
named `rigExec:falloffProfile` (`linear`, `smooth`, `easeIn`, `easeOut`,
`constant`) or — when the profile is `curve` — the Ts spline authored on
`rigExec:falloffCurve`. Both land in the same table, so the hot loop has
exactly one remap path and switching between a preset and a hand-drawn
curve changes only numbers. Every profile pins `f(0) = 0` and `f(1) = 1`,
so switching never moves the band's endpoints, only its shape between
them.

## Three decisions worth knowing

### 1. A volume weight is an `RigExecXformable`, not a `RigExecWeightObject`

usdGenSchema permits a typed schema exactly one base — *"Schemas can only
inherit from one other schema at most"* (`usdGenSchema:512-522`, verified
empirically). The three `RigExecWeightObject` contract properties are
therefore **redeclared** on `RigExecVolumeWeight`, so the composed prim
definition is identical; what is lost is only `IsA`-testability, and
RigExec already dispatches weight objects by type *name* everywhere
(`rigEvaluator.cpp` did this for `RigExecDynamicWeight` long before this
change). `_IsWeightObjectType` in `rigEvaluator.cpp` is now the single
place that knows the set.

Spending the one base on the xformable side is what buys the behaviour a
rigger needs: the volume is one selectable, framable, `UsdGeomBoundable`
prim, positioned by the same Ir-aligned avars as every control and joint
— and because an unwired xformable follows its namespace-parent
xformable's posed space, **a volume authored inside a joint rides that
joint with nothing wired**. See `ShoulderVolume` in
`examples/11_VolumeWeights.usda`.

### 2. Shape comes from floats, never from the transform

The posed frame is orthonormalized before a guide is drawn (guides are
rigid). A scale baked into the transform would therefore deform the field
without deforming the drawn guide, and the artist would be painting with
a shape they cannot see. So the rigid placement positions the volume and
`inputs:scaleX/Y/Z` plus `inputs:falloffMin/falloffMax` are the sole
authority on its size. Guide and field agree by construction.

Both the exec kernel and the CPU oracle call `RemoveScaleShear()` on the
placement before inverting it, for exactly this reason.

### 3. The falloff curve cannot be an exec input

OpenExec 26.08 has no accessor for an attribute's spline — `// XXX:TODO
Accessors for AnimSpline` is still open in
`exec/computationBuilders.h:584`. A computation resolves an attribute at
*one* time; a falloff curve is the whole function.

So the curve is **epoch-structural**: resampled to a
`RigExecFalloffLut` at `Compile`, and delivered to the kernel as a
`RigExecValueOverride` on a stub `computeFalloffLut` prim computation.
That is the same shape the ribbon already uses for its driver-curve
points, and for the same class of reason. The consequence is that the
curve's *shape* is a rig-authoring parameter rather than an animation
channel — which is also what keeps the field epoch-shape-stable. The
animatable knobs are `falloffMin`/`falloffMax`, `invert`, and `strength`,
all of which are ordinary per-frame exec inputs.

Editing the curve changes the structure digest and begins a new epoch;
the knots are hashed, the animatable floats deliberately are not.

## The plane's two axes: the band along it, the extents across it

A plane weight carries two independent sizes and conflating them is the
one mistake this shape invites.

- `inputs:falloffMin`/`falloffMax` are distances **along**
  `rigExec:planeAxis`. They are the band, and they decide where the
  gradient starts and stops — the same meaning they have on every other
  volume.
- `inputs:extentU`/`extentV` are half-extents **across** it, in the two
  in-plane axes `U = (axis + 1) % 3` and `V = (axis + 2) % 3` (for the
  default `y` that is Z and X). They decide how far the sheet reaches.

They are different directions and different quantities, and neither can
be derived from the other. The guide used to try: with no authored
extent it sized its drawn square `max(1, |falloffMin|, |falloffMax|)`, so
a rigger dragging the band watched the plane grow and shrink instead of
watching its two surfaces separate. Adding the extents fixes that from
both ends — the square now has a real size, and the band now only slides
it.

`rigExec:planeBounds` then turns that size into a boundary:

| | field |
|---|---|
| `unbounded` (default) | signed axis distance everywhere; the plane is infinite |
| `bounded` | the same distance inside the rectangle, **exactly zero** outside it |

Containment is **hard** — a point one epsilon outside gets zero, not a
ramped-down weight — which is what was asked for and the only rule that
makes the affected region exactly the drawn rectangle. Softening the
border is composition's job: multiply the bounded plane by a sphere and
the edge inherits that volume's falloff. That is the same answer this
extension gives to every other "but softer" question.

`unbounded` is the default because it is what a half-space falloff wants
and what every plane weight authored before these properties existed
had. It is also **structural** — it selects which field function runs, so
it is hashed into the epoch digest next to `rigExec:planeAxis`, while the
extents are ordinary per-frame floats and deliberately are not.

The extents size the drawn guide either way, so an unbounded plane still
gets a square an artist can aim; only `planeBounds` decides whether that
square is a boundary or a label. The two read differently on purpose:
**bounded draws the closed rectangle alone** (what you see is what the
field covers), **unbounded adds four short outward ticks** at the edge
midpoints saying "this continues". The ticks are a separate curve element
rather than extra segments on the rectangle because in `geometry` draw
mode the rectangle is a mesh and cannot carry them, and they are a
fraction of the extent rather than a fixed length so they survive being
drawn at any scale.

Both evaluation paths implement the bound — the exec kernel in
`moverKernels.cpp` and the CPU oracle in `rigEvaluator.cpp` — and the
parity harness is what holds them together. Removing it from the exec
side alone gives

```
plane-bounded: diagnostic: cpu reference parity: value mismatch on
    /Asset/Geom/M.points at element 3
plane-bounded: 1 graph/CPU parity MISMATCHES
```

which is exactly the failure `TestPlaneBounded` exists to catch: a bound
applied on one path still moves points, just not the same ones.

## Sample phase

`rigExec:samplePhase` decides which points the distance is measured
against.

- **`reference`** (default) samples the static authored base points, so a
  point keeps the weight its bind pose earned — the behaviour of a
  painted map, and what a matrix mover wants so its own output cannot
  feed back into its own weights. `rel rigExec:sampleSource` optionally
  overrides *what* is sampled without changing what is weighted, which is
  how one mesh is weighted by another mesh's shape.
- **`current`** samples the points as they stand at that operation's
  position in the mover stack, so the volume grabs whatever is inside it
  right now. Order dependent by construction: the same volume at two
  points in the stack legitimately yields two different fields.

`current` cannot come from exec. A revision node's only inputs are its
parameters, its status, and the read-write point buffer, and the
parameters are baked as a VDF constant when the graph is built — nothing
in the packet can depend on a value the graph has not computed yet. What
*can* be done, and is: `RigExecMoverGraph::Evaluate` is const and takes
any masked output, so the evaluator evaluates the chain built **so far**,
measures against that, and bakes the result into the next revision's
parameters. See the graph build loop in `rigEvaluator.cpp::Evaluate`.

**Know the cost before reaching for it.** Each `Evaluate` builds its own
request, schedule, and executor over the whole prefix, so a chain of *N*
revisions containing *K* current-phase weights does *O(N·K)* point work —
quadratic if every mover in a long chain samples `current`. `reference` is
the default precisely because it is free: the field is a per-generation
constant that exec computes once. Reach for `current` where the behaviour
is actually wanted (a volume that should grab whatever has been deformed
into it), not as a general-purpose "more correct" setting.

The obvious optimisation — reuse one executor across the prefix
evaluations instead of rebuilding per revision — is available and
deliberately not taken yet: nothing in the repo needs it, and the naive
version is the one whose correctness is easy to see against the CPU
oracle.

## Composition

`RigExecCombineWeight` folds an ordered list of weight objects. Every
input is resolved to a dense field over the same element count before
folding, so a painted `RigExecStaticWeight`, a driven
`RigExecDynamicWeight`, and a generated volume compose without any of
them knowing about the others.

`multiply`, `add`, `max`, `min`, and `average` are order independent.
`subtract` and `overlay` are **not**: they seed from the first input and
fold the rest in authored target order. That is the one place this
extension departs from the spec §7.2 rule that target-list permutation
cannot change a result, and it is stated rather than hidden — authors who
need a permutation-proof composition use a commutative mode.

Combines nest, and nesting is how mixed modes are expressed: one combine
carries one mode, so a relationship never has to carry per-target
metadata. (Maya's falloff list attaches a mode to each entry; a USD
relationship cannot, and nesting is the composition-native answer.)

A length mismatch between inputs is a **structural error**, not a
truncated fold — a short input would silently read as identity over its
missing tail.

## Where the code lives

| Concern | File |
|---|---|
| Schema | `libs/rigExecSchema/schema.usda` (volumetric section) |
| Pure kernels | `libs/rigExecMath/weightFields.{h,cpp}` |
| Exec computations | `libs/rigExec/moverKernels.cpp` (`_Build*WeightPacket`) |
| `computeMatrix` etc. | `libs/rigExec/computations.cpp` (`RIGEXEC_REGISTER_XFORMABLE(RigExecVolumeWeight)`) |
| LUT packet type | `libs/rigExec/types.h` (`RigExecFalloffLut`) |
| CPU oracle, epoch, overrides | `libs/rigExec/rigEvaluator.cpp` |
| Resolved field for tools | `RigExecRigPose::weightFields` |
| Authoring UI | `plugin/rigExecUsdview/volumeWeightUI.py` |
| Influence overlay + guides | `libs/rigExecImaging/` (`RigExecImaging_SetWeightOverlay`) |
| Example | `examples/11_VolumeWeights.usda` |
| Tests | `tests/testRigExecWeightFields.cpp`, `tests/testRigExecVolumeWeights.cpp`, `tests/testRigExecWeightOverlay.cpp`, `bin/run_testusdview_overlay.bat` |

## Registering computations on an abstract base

`RIGEXEC_REGISTER_XFORMABLE` is invoked on the **abstract**
`RigExecVolumeWeight`, unlike `RigExecJoint`/`RigExecControl` which are
concrete. Exec composes a prim's computation set by walking its full
ancestor type vector strongest-to-weakest
(`exec/definitionRegistry.cpp::_GetFullyExpandedSchemaTypeVector`), so
the three concrete volume weights inherit the three xformable
computations from the base.

It *has* to be the base here, because the concrete types already carry an
`EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA` block in `moverKernels.cpp` for
`computeWeightPacket`, and one schema may only be opened once: the macro
emits a whole registration function plus its `TF_REGISTRY_FUNCTION` per
invocation, and a second block for the same schema is a second,
independent registration pass over a type the first pass has already
marked complete. Splitting by *type* rather than by *file* keeps each
schema opened exactly once.

## The influence overlay, and the trap under it

`RigExecImaging_SetWeightOverlay(primPath)` paints a weight object's
resolved field onto its target geometry as a vertex `displayColor`, grey
at 0 to red at 1 — the Rhythm & Hues Voodoo idiom. Pass `""` to turn it
off. The volume's own falloffMin and falloffMax iso-surfaces draw as wire
guides through the same synthesis protocol the control and joint guides
use.

**A primvar appearing is a resync, not a dirty.**
`HdSceneIndexAdapterSceneDelegate` caches each rprim's primvar
*descriptors* and rebuilds them from `PrimsAdded`. A dirty — even
`HdDataSourceLocatorSet::UniversalSet()` — re-pulls values for the
descriptors it already has. So turning the overlay on while the mesh was
already synced left a perfectly correct red `displayColor` sitting in the
scene index one hop upstream, with Storm never asking for it and the mesh
rendering grey forever.

Every scene-index-level assertion passed throughout. The bug was only
visible in the framebuffer, and the measurements are worth keeping
because they are what distinguishes the two hypotheses:

| | red-ish pixels |
|---|---|
| overlay off | 1009 |
| overlay on, dirty-notice path | 997 |
| overlay on, cold renderer rebuild | 3456 |

A cold rebuild rendering red is what proved the data path was already
correct and the fault was purely invalidation. The fix re-announces the
prim with its upstream type on the on/off transition
(`sceneIndices.cpp`, the structural arm of `NotifyGenerationPublished`).

`bin/run_testusdview_overlay.bat` (and `bin/run_testusdview_overlay.sh`) now
asserts on **pixels**, not just on the scene index, and that assertion is
mutation-verified: disabling the resync makes it fail while every other test in
the suite still passes.

It asserts on the *difference* between the overlay-off and overlay-on frames,
not on how much red is in either one. RigExec draws its own guide geometry, the
volume rings and bars are red, and they cover far more of the viewport than the
strip does — so an absolute red-pixel count is mostly guides, and it shifts
whenever guide drawing changes. Differencing cancels them, since they are
identical in both frames. The measurement is a fraction of the frame rather
than a pixel count so that window size and HiDPI do not enter into it: the
overlay moves 2.8% of the frame, a disabled resync moves 0.0%, and the
threshold sits at 0.5%.

## The second invalidation trap: who hears an edit first

The overlay repainted correctly on the *toggle* and then appeared frozen
while a rigger dragged `inputs:falloffMax`. It was not frozen — it was
**one edit late**, and so was the geometry. A drag hides that almost
perfectly: every mouse-move step draws the previous step's answer, so the
picture moves and only the final value is wrong.

`RigExecImagingRegistry::_OnObjectsChanged` re-evaluates from inside
`UsdNotice::ObjectsChanged` dispatch. `Tf_NoticeRegistry::_Register`
**prepends** its deliverer, so listeners run most-recently-registered
*first* — and OpenExec's `ExecUsdSystem::_NoticeListener`, the thing that
drops every cached computed value when an attribute is authored, is
constructed by the `ExecUsdSystem` constructor inside `Compile()`.
Registering our listener after `Compile()` therefore put us **ahead of
exec**, and every generation an edit published was computed from exec's
pre-edit cache.

The measurements that separate this from a missing dirty notice:

| | published field |
|---|---|
| stage resolves `falloffMax` = 9 inside our callback | the 4.46 field |
| a *second* `Evaluate` in the same dispatch | still the 4.46 field |
| an `Evaluate` after the dispatch ended | the 9 field |
| the same edit sequence outside any notice | fresh on the first `Evaluate` |

Red-ish pixels while dragging `falloffMax` 4.46 → 9 → 1.5 → 4.46:

| | before | after |
|---|---|---|
| 4.46 | 3432 | 3441 |
| 9 | 3433 | 6076 |
| 1.5 | 6063 | 2142 |
| 4.46 | 2157 | 3456 |

The fix is to register the listener **first**, before `Compile()`, which
puts it at the back of the delivery list and keeps it there: every
recompile builds a new `ExecUsdSystem` that prepends ahead of it again.

Two things are worth carrying forward. First, a direct `SetTime()` call
cannot see this bug — by then the dispatch has ended and exec is
invalidated — which is why a suite full of `SetTime` assertions passed
throughout. `TestAuthoredEditRepublishesFreshField` in
`testRigExecWeightOverlay.cpp` authors to the stage and lets the notice do
the republishing, and it is mutation-verified: moving the registration
back after `Compile()` fails it and nothing else. Second, a plain default
`Set` and the spline knot the authoring panel writes are **not**
equivalent through USD's change classification — only the spline knot
reproduced, so the test covers both and the spline arm is the one that
catches it.

## The third cause of the same symptom: the wrong frame

Two edit-freshness bugs above, and a third that presents identically and
lives somewhere else entirely: the usdview plugin published the **frame
the artist had just left**.

`RootDataModel.currentFrame`'s setter emits `currentFrameChanged(value)`
and only *then* assigns `self._currentFrame`
(`Usdviewq/rootDataModel.py:158-160`), so
`RigExecUsdviewContainer._OnFrameChanged` re-reading
`dataModel.currentFrame` was handed the previous frame. Every
publication still happened, on time and in order, with the wrong pose in
it.

That is worse than it sounds, because it compounds with the authoring
panel. `volumeWeightUI.SetAtTime` writes a spline knot **at the frame
the artist is looking at**; an engine sitting one frame back resolves the
knot they did not touch, so scrubbing `inputs:falloffMax` changes the
picture not at all. Measured on `11_VolumeWeights` at frame 1024, with
the overlay on and the band scrubbed through the panel's own call:

| | red-ish pixels | published field |
|---|---|---|
| before, `falloffMax` 2 | 12228 | frozen at the frame-1001 field |
| before, `falloffMax` 8 | 12250 | the same frozen field |
| after, `falloffMax` 2 | 2182 | `0.87, 0.48, 0.05, 0, …` |
| after, `falloffMax` 8 | 5635 | `0.99, 0.95, 0.88, 0.79, …` |

The fix is to use the frame the signal carries
(`rigExecUsdview.py::_OnFrameChanged`), and the reason it survived so
long is that `testUsdviewRigExec.py` counted **generations** rather than
checking which frame was in them. It now drives one frame both ways --
through `currentFrame` and through `RigExecImaging_SetTime`, which is
unambiguous by construction -- and compares the published points.
Mutation-verified: restoring the property read fails that comparison
(`(0.0, 9.5, 0.0)` vs `(0.06, 10.13, 0.48)`) while
`bin/run_testusdview_overlay.bat` and all nine ctest binaries still pass,
which is exactly the blindness that let it ship.

## What the epoch digest is and is not tested for

`TestPlaneBounded` asserts the bounded *field*, and it passes with
`rigExec:planeBounds` removed from the structure digest entirely -- each
`Resolve()` builds a fresh evaluator, so there is no stale epoch for an
unhashed token to strand. The digest therefore needs its own case, and
`TestPlaneBoundsEpochSplit` is it: one evaluator, edited twice.

- `inputs:extentU/V` must change the field and **not** the digest. A
  digest that hashes an animatable float rebuilds the whole rig on every
  mouse-move of the scrub row that drives it.
- `rigExec:planeBounds` must change **both**, and raise the `epoch
  rebuilt` diagnostic.

Mutation-verified in both directions: dropping the `planeBounds` hash
fails the second half, and adding an `extentU` hash fails the first,
each with nothing else in the suite noticing.

`TestPlaneBoundedInvalidExtents` covers the remaining arm, where the two
implementations could disagree quietly: `bounded` with a non-positive
extent is rejected by the exec kernel (an invalid packet) and by the CPU
oracle (an error), and both have to mean **pass-through** downstream. A
kernel that read a negative extent as "unbounded" would move points the
oracle leaves alone and only parity would see it.

## A build trap this change walked into

Adding members to `RigExecRigEvaluator` and `RigExecRigPose` changes their
layout. Building only the `rigExec` target leaves `testRigExecArm.exe`,
`rigExecImaging.dll` and friends linked against the *old* layout, which
presents as a segfault and a `0xc0000374` heap corruption rather than a
link error. Any header change under `libs/rigExec/` needs a full
`cmake --build build`, not a targeted one.
