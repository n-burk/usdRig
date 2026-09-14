# Optional rest-position guides for IK solvers

Date: 2026-09-06. Status: awaiting review.

## 0. Request

"The ik solver should optionally draw the rest positions of the joints
under its influence."

Answered during brainstorming: the switch is a **schema attribute on the
solver** (not a usdview preference); it draws the **rest chain plus an
axis triad** at each rest frame; it covers **both** `RigExecTwoBoneIk`
and `RigExecSingleChainIkConstraint`; and a rest guide **selects the
solver** that drew it, keeping the pipeline's existing picking rule.

## 1. Why this exists

`docs/viewport-gizmos.md` documents Pivot mode as the way to
re-proportion a limb: the manipulator sits on a joint's rest frame, and
a `RigExecTwoBoneIk` measures its bone lengths from those rests on every
evaluation, so the drag moves the solve. The rest frames themselves have
never been visible. An animator dragging a knee pivot is manipulating a
frame they cannot see, on a limb drawn somewhere else entirely.

This draws them, and only when asked: they are diagnostics, not part of
the rig's interaction surface.

## 2. Facts the design relies on

Each was read in the tree at `bc0f0e2` plus the working-tree changes,
and each is load-bearing.

1. **Solvers already draw guides.** `bridge.cpp:783-806` walks
   `pose.solverFrames[solverPath]` and appends a sphere plus an
   aim-fallback cone per frame, styled from the solver prim's `guide:*`
   attributes through `_ReadGuideStyle`. A `RigExecTwoBoneIk` already
   draws its solved `[root, mid, end]` chain today.
2. **Joints draw a sphere at each posed origin and a cone to every
   nested child** (`bridge.cpp:742-780`), the link frame built by
   `_JointLinkFrame` (`bridge.cpp:122+`). The rest chain wants exactly
   this shape, so it reuses both helpers rather than inventing a second
   convention.
3. **Every influenced joint's rest frame is already resolved inside
   `Evaluate`.** `rigEvaluator.cpp:6690-6700` seeds
   `restFrames[provider]` for *every reachable RigExec frame provider*
   from `_poseSeedRests`. `rigEvaluator.cpp:7456` already looks up a
   single-chain constraint's `ikChain` joints in that map. **No new
   exec taps are needed for either solver type.**
4. **The two solvers derive their joint set differently.**
   `RigExecTwoBoneIk` binds a fixed `[root, mid, end]` through
   `rigExec:joints` — or, under the working-tree consumer-inference
   change, inherits them from the downstream consumer that owns them
   (`_ImpliedIkLengths::joints`, `::restsFrom`,
   `rigEvaluator.cpp:3684+`). `RigExecSingleChainIkConstraint` uses the
   inclusive namespace chain between `rigExec:firstJoint` and
   `rigExec:endJoint`, held as `constraint.ikChain`, variable length.
5. **Wire guides are already linear nonperiodic basisCurves.**
   `_BuildControlGuideShapes` (`sceneIndices.cpp:725-833`) emits them
   for every wire control-guide shape. An axis triad is three 2-vertex
   linear curves — new geometry, not new machinery.
6. **Synthesized guides are named by prefix** — `rigGuideSphere_`,
   `rigGuideCone_` — parsed and built by `_ParseGuideName` / `_GuideName`
   (`sceneIndices.cpp:351-387`), announced and dirtied through
   `_RefreshAnnouncedGuides` (`sceneIndices.cpp:256-273`).
7. **A guide inherits its parent's `primOrigin`**
   (`sceneIndices.cpp:483-498`), which is what makes a guide select the
   prim it hangs off. This is why rest guides select the solver: it is
   the rule, not a new decision.
8. **Guide evaluation never gates the rig snapshot.**
   `rigEvaluator.cpp:7866-7891`: "Observational solver guides never gate
   the rig snapshot: an incomplete guide evaluation degrades to a
   diagnostic." Rest guides adopt the same contract.
9. **Applied API schemas are established here** —
   `RigExecControlAPI` and `RigExecMoverAPI` (`schema.usda:33`, `:49`),
   both `singleApply`. `RigExecTwoBoneIk` inherits `Boundable`
   (`schema.usda:414`) while `RigExecSingleChainIkConstraint` inherits
   the abstract `RigExecConstraint` (`schema.usda:900`, `:686`), so the
   two share no concrete base to hang attributes on.

## 3. Design

### 3.1 Schema: one applied API, two types

A new single-apply API schema, applied to both solver types, so the
attribute set is declared once and one code path reads it:

```
class "RigExecRestGuideAPI" (
    inherits = </APISchemaBase>
    customData = {
        string className = "RestGuideAPI"
        token apiSchemaType = "singleApply"
    }
)
{
    uniform bool rigExec:drawRestGuides = false
    color3f guide:restDisplayColor = (0.30, 0.55, 0.80)
    float guide:restDisplayOpacity = 0.5
    double guide:restRadius = 1.0
}
```

Four decisions inside that block:

- **`uniform bool`, defaulting off.** Rest guides are a diagnostic an
  author turns on while rigging. Defaulting off means every existing
  asset draws exactly what it drew before.
- **A separate colour and opacity pair, not `guide:displayColor`.**
  Both chains hang off the *same* solver prim. Sharing the style would
  make the rest ghost indistinguishable from the solved chain beside it,
  which defeats the feature. The default is cool and half-transparent
  against the warm opaque `(1.0, 0.85, 0.2)` guide default.
- **A separate `guide:restRadius`, not the existing `guide:radius`.**
  `RigExecSingleChainIkConstraint` has no `guide:radius` at all, and on
  `RigExecTwoBoneIk` reusing it would tie the ghost's size to the solved
  chain's. One attribute on the API covers both types uniformly.
- **No per-solver "draw chain / draw triads" split.** Two switches for
  one diagnostic is a setting nobody wants to find. If the triads prove
  too noisy in practice, that is a reason to change what is drawn, not
  to add a second boolean.

`rigBuilder` gains `SetDrawRestGuides(bool)` on both solver handles,
exposed to Python as `set_draw_rest_guides`, following the existing
handle-setter pattern.

### 3.2 Evaluator: publish, unconditionally

`RigExecPose` gains a peer to `solverFrames`:

```cpp
/// Solver path -> the REST frames of the joints it influences, in chain
/// order (guide drawing and inspection). Peer to solverFrames, which
/// holds the same chain as SOLVED.
std::map<SdfPath, std::vector<RigExecPointFrame>> solverRestFrames;
```

Filled in `Evaluate` from the already-resolved `restFrames` map (fact 3),
after the pose walk:

- `RigExecTwoBoneIk` — the three joints the solver measures, taken from
  the `_ImpliedIkLengths` record where one exists, otherwise from
  `rigExec:joints` directly. Taking them from the record means the
  consumer-inherited case (a solver feeding a blend that owns the
  joints) draws the rests it actually measures, and the diagnostic
  matches the `via <consumer>` message `Evaluate` already emits.
- `RigExecSingleChainIkConstraint` — `constraint.ikChain`, in order.

**The attribute is not read here.** The evaluator always publishes;
`bridge.cpp` decides whether to draw. This keeps `drawRestGuides` out of
`_ComputeStructureDigest` — it changes nothing about the compiled
network, so toggling it must not force a recompile epoch. The cost is a
handful of matrices per solver per evaluation.

A joint missing from `restFrames`, or an invalid frame, drops that
element and appends a diagnostic. It never fails the pose (fact 8).

### 3.3 Imaging: a third guide element kind

`RigExecPublishedPrim` gains vectors parallel to the existing guide
ones — `restGuideFrames`, `restGuideLengths`, `restGuideRadii`,
`restGuideDrawSpheres` — plus `hasRestGuides`, `restGuideColor` and
`restGuideOpacity`. Parallel rather than appended into `guideFrames`
because the two chains are styled differently; sharing the vector would
make per-element styling impossible.

`bridge.cpp` gains a loop beside the solver-guide loop at `:783`:

1. Read `rigExec:drawRestGuides` off the solver prim. False or absent →
   publish nothing, and erase the prim if it holds nothing else, exactly
   as the existing loop does at `:807-810`.
2. For each consecutive pair of rest frames, build the link with
   `_JointLinkFrame` and append through `_AppendGuideFrame` with
   `useAimFallback = false` and `drawSphere = !any` — the joint
   convention (fact 2), not the solver loop's `+X` aim fallback, because
   here the successor frame is known.
3. The last frame gets a sphere-only element, as a leaf joint does.
4. One `rigGuideAxes_<i>` element per rest frame.

`sceneIndices.cpp` gains the `rigGuideAxes_` prefix in `_ParseGuideName`
/ `_GuideName` and a builder emitting linear nonperiodic basisCurves:
6 points, `curveVertexCounts = [2, 2, 2]`, axis length
`guide:restRadius * 3`, in the frame's local space, placed by the frame
the same way the control-guide shapes are.

**Triad colour is per-axis RGB, not `guide:restDisplayColor`.** X red,
Y green, Z blue, as a `vertex`-interpolated `displayColor` with six
values. The triad exists to show orientation; a monochrome triad cannot
tell X from Z and would not be worth its geometry.
`guide:restDisplayOpacity` still applies. The chain — spheres and
cones — uses `guide:restDisplayColor`.

Guides inherit `primOrigin` from the solver unchanged, so a click
selects the solver (fact 7).

### 3.4 Bounds

`_AccumulateGuideBounds` (`registry.cpp:656-680`) gains a second loop
over the rest vectors, identical in form to the first, plus the triad
extent (`restRadius * 3` along each axis from each rest origin). Without
this, framing a rig whose rest chain sits away from its posed one puts
the camera on empty space.

## 4. Degradation

Every failure is a diagnostic and a missing guide, never a failed pose:

- a joint with no resolvable rest frame — that element is skipped;
- a solver binding fewer joints than its chain needs — nothing drawn,
  matching the existing "binds fewer than three elements" diagnostic;
- a non-positive `guide:restRadius` — `_AppendGuideFrame` already
  rejects it (`bridge.cpp:110-112`), so nothing is drawn, consistent
  with every other guide;
- coincident consecutive rests — `_JointLinkFrame` fails, the cone is
  skipped, the sphere still draws.

An IK solver with both absolute bone lengths authored still draws its
rest guides. Its joints are still influenced; only the *measurement* is
off, and the gizmo toolbar already says so through
`FrozenIkSolvers` — "…has an authored bone length; this rest edit will
not move the solve".

## 5. Testing

- **`testRigExecImaging.cpp`** — rest guides published when the
  attribute is on and absent when off, for both solver types; a
  both-lengths-authored `RigExecTwoBoneIk` still publishes them; the
  consumer-inherited joint set draws the joints the solver measures; a
  missing rest frame degrades to a diagnostic with the pose still valid.
- **Bounds** — a rig whose rest chain sits clear of its posed chain
  reports a bound covering both, and covering the triads.
- **Scene index** — `rigGuideAxes_<i>` round-trips through
  `_ParseGuideName` / `_GuideName`; the triad is basisCurves with
  `curveVertexCounts = [2, 2, 2]` and a 6-value vertex `displayColor`.
- **Schema** — regenerate `plugin/rigExecSchema/resources`; a
  `test_rigexec_schema_authoring.py` case applies `RigExecRestGuideAPI`
  to both types and round-trips the builder setters.
- **usdview** — a generation check that toggling
  `rigExec:drawRestGuides` on a live stage redraws. This is the one
  claim in the design that rests on the snapshot republishing for an
  attribute the digest does not cover (section 3.2); **verify it early
  during implementation**, and if a plain value change does not
  republish, the fix is to include the attribute in the imaging change
  tracking — not to put it in the structure digest.

## 6. Out of scope

- Rest guides on any other solver (`RigExecFkChain`,
  `RigExecBlendPointFrames`, `RigExecTwistDistribution`). The API schema
  makes adding one later a one-line application plus a joint-set
  derivation, which is the right shape for that decision to arrive in.
- A usdview toolbar override. Rejected during brainstorming: the switch
  belongs to the rig, and a view-only duplicate would be a second source
  of truth for the same question.
- Drawing rest guides for joints no solver influences. A joint's own
  rest frame is reachable through Pivot mode; this feature is about
  seeing what a *solver* is working from.
- Making a rest guide draggable. The Pivot manipulator already edits
  these frames, and a second interaction surface for the same value is
  how the two disagree.
