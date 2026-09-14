# Manipulate through Hydra, author once on release

Design note, 2026-09-10. IMPLEMENTED the same day; the "what was built"
section at the end records where it differs from the plan above it, which is
in two places that only showed up against the real code.

## What happens today

A gizmo drag authors to the USD stage on every mouse move.

`GizmoController._UpdateDrag` (plugin/rigExecUsdview/gizmoUI.py:3097)
calls `gizmoDrag.ApplyDrag`, which reaches a `Target.Apply*`, which
reaches one choke point: `Writer.Set`
(plugin/rigExecUsdview/gizmoMath.py:843). That authors. In
`WRITE_ANIMATION` mode it authors a *spline knot* (`SetAnimated`,
gizmoMath.py:798) at the current time, per mouse move.

Each of those writes is a `UsdNotice::ObjectsChanged`, which
`RigExecImagingRegistry::_OnObjectsChanged`
(libs/rigExecImaging/registry.cpp:514) turns into an exec invalidation, a
re-evaluation, and a published generation. So the viewport is correct
during a drag, and the price is an authoring round trip per mouse
sample: exec invalidated through the authoring stage, a spec rewritten in
the edit target layer, notices broadcast to every panel watching the
stage, and -- in animation mode -- a knot authored, re-authored, and
re-authored at the same time code.

The undo stack is already drag-granular: `_BeginDrag` captures an
`AttributeSnapshot` per attribute and `_EndDrag` commits one entry
(gizmoUI.py:3062, 3145). That is the one part of this that is already
right, and it survives unchanged.

## Why that is the wrong shape

The stage is the document. A drag in progress is not a document change --
it is a question the artist has not finished asking. Authoring each
sample of it means:

* every intermediate value is a real edit to a real layer, visible to
  every observer of the stage, including panels that have no business
  seeing an unfinished gesture;
* exec is invalidated through the authoring stage at interactive rates,
  which is the most expensive way to get a new pose out of a rig;
* animation mode leaves knot churn behind -- the final knot is correct,
  but it was written once per mouse move to get there;
* the cost of a drag scales with how the artist moves the mouse rather
  than with what they changed.

## The reference architecture

ImGuiHydraEditor (https://github.com/raph080/ImGuiHydraEditor) puts a
filtering scene index in the Hydra chain that holds per-prim transform
overrides in memory. Its manipulator writes the manipulated matrix into
that scene index every frame -- `Viewport::_UpdateTransformGuizmo` calls
`_xformSceneIndex->SetXform(primPath, ...)` -- and `GetPrim` overlays the
cached matrix onto the upstream prim's data source, sending an
`HdXformSchema` dirty notice. Nothing is written back to USD at all;
closing the app loses the edit.

The half to take is the direction of flow: the manipulator talks to
Hydra, not to the stage. The half to add is the commit -- on release the
result becomes one authored edit on one undo entry.

## How it maps onto RigExec

A rig manipulation is not a transform override, and that is the whole
reason the mapping is not a copy. Dragging a control's `avars:tx` has to
re-run the rig -- solvers, joints, deformation -- so overriding one
prim's xform in Hydra would move the manipulator and nothing else. The
equivalent of their xform filter is the rig evaluator itself, and the
equivalent of `SetXform` is an evaluation-time value override.

That machinery already exists, one level below a public API:

* `RigExecValueOverride` (libs/rigExec/tapSet.h:75) carries either a prim
  computation or a **named attribute**, plus a value;
* `RigExecTapSet::Evaluate` turns those into an
  `ExecUsdValueOverrideVector` and calls
  `ExecUsdSystem::ComputeWithOverrides` (libs/rigExec/tapSet.cpp:229-244);
* `RigExecRigEvaluator::Evaluate` already assembles a `baseOverrides`
  vector prefixed to every request of the generation
  (libs/rigExec/rigEvaluator.cpp:6610), today carrying resolved property
  chains and ribbon driver points.

So an authored avar can already be given a different value for one
evaluation without touching the stage. What is missing is a way for the
application to say so, and a manipulator that says it.

### Lane 1: rig targets (control, joint, pivot)

```
mouse move -> Writer (preview mode) -> interactive overrides
           -> evaluator.Evaluate(time)        [stage untouched]
           -> snapshot generation -> results scene index -> viewport
```

* `RigExecRigEvaluator` gains `SetInteractiveOverrides(...)` and
  `ClearInteractiveOverrides()`. The vector is merged into
  `baseOverrides` at the top of `Evaluate` and -- this is the subtle
  part -- into `_resolvedInputs` as well, because property chains and the
  CPU oracle read values by a route exec overrides do not cover
  (rigEvaluator.cpp:6632-6637 documents the two delivery routes that must
  not disagree). An override on an avar that a math mover reads has to
  reach both, or the mover computes from the authored value while exec
  computes from the preview one.
* `RigExecImagingRegistry` gains `SetInteractiveOverrides(rigPath, ...)`,
  which sets them on the session's evaluator and republishes at the
  current time. `SetWeightOverlay` (registry.h:60, registry.cpp:474) is
  the existing example of exactly this "change state, republish now"
  shape, including its republish-outside-the-lock rule
  (registry.cpp:504).
* Two `extern "C"` entry points beside the seven already there
  (registry.cpp:1366-1482), since the plugin reaches C++ through ctypes
  (plugin/rigExecUsdview/rigExecUsdview.py:63-86).

### Lane 2: plain Xform targets

`XformPoseTarget` and `XformPivotTarget` drive a prim the rig evaluator
knows nothing about, so there is no generation to put a preview in. This
lane is ImGuiHydraEditor's design taken literally: a filtering scene
index holding `{path: matrix}`, overlaying `HdXformSchema` in `GetPrim`,
dirtying the xform locator on set. It joins the chain the registry
already builds (`RegisterChain`, registry.h:38), so one preview
mechanism covers both lanes from the manipulator's point of view.

(Taken literally is what this got wrong -- see "What was built": the matrix
has to be a world-space delta, not a local transform, because this chain
flattens upstream of the filter.)

### Drag lifecycle

| Phase | Today | After |
|---|---|---|
| `_BeginDrag` | snapshot attrs, capture base values | unchanged, plus open a preview session |
| `_UpdateDrag` | author every channel, per move | push overrides, author nothing |
| `_EndDrag` | commit the recorder | author the final values once, then clear the preview |
| `_AbortDrag` | restore the snapshot | clear the preview; nothing was ever authored |

`Writer` is the only place that authors, so preview is a mode on
`Writer`: collect `{attrPath: value}` instead of calling `attr.Set`, and
hand the collection to the registry. The drag maths needs no change at
all, because `Target.BeginDrag` captures base values and every `Apply*`
recomputes from that capture rather than from what it last wrote
(gizmoMath.py:1078-1085). That one property is what makes this change
small instead of sweeping.

### Where the manipulator reads its own frame

During a preview the stage still holds the pre-drag values, so anything that
recomputes the gizmo's frame from the stage would leave the handles behind
while the geometry moves.

The plan here was to read the posed frame back from the published generation --
the same channel that drew it, through
`RigExecImaging_GetControlFrameAssetSpace` (registry.cpp:1426), which already
exists and is already used for curvenet adjustments. What was built instead is
the other option the plan called worse: the frame maths reads the uncommitted
values directly, because `Writer.Set` publishes each one into `gizmoMath`'s
preview map and every read there consults it first.

The objection to that was "a second implementation of the pose, and the two
would drift". It does not hold, because gizmoMath ALREADY computes the pose
from attributes -- that is what the module is, and `test_gizmo_math` asserts it
against the native evaluator on every run. Reading a preview value instead of
an authored one changes the inputs to that computation, not the computation.
And it has two properties the read-back does not: it works for every target
kind, including a plain `Xformable` and a pivot, neither of which has a
published control frame; and it needs no round trip through a generation to
draw a handle.

The read-back stays where it was, for the one case that genuinely cannot be
recomputed from authored USD (`RigExecCurvenetAdjustment`).

## What changes observably

* The stage is not modified until the mouse is released. The graph
  editor, the Layer Opinions panel, and any other stage observer show the
  pre-drag values for the duration of the gesture and the committed value
  the moment it ends. That is Maya's behaviour, and it is the point of
  the change rather than a side effect of it.
* Animation mode authors one knot per drag instead of one per mouse
  sample.
* Undo granularity is unchanged: one entry per drag, as today.
* An aborted drag authors nothing, so it cannot leave a spec behind in
  the edit target layer.

## Open questions

1. Lane 2 doubles the surface of the change. Rig-only is a coherent first
   step if plain-Xform drags keep authoring live, at the cost of two
   manipulators that behave differently.
2. Whether the preview path is unconditional or a setting. A setting
   keeps both paths alive and testable; it also means the old one never
   dies.
3. `SetTime` mid-drag (scrubbing while dragging): the overrides are
   time-independent values, so they would apply at the new time too.
   Probably right, but it should be decided rather than discovered.

## Work plan

Each stage is independently testable and leaves the tree working.

1. Evaluator: `SetInteractiveOverrides` / `ClearInteractiveOverrides`,
   merged into both delivery routes. C++ test: a rig whose avar override
   moves joints while the stage is provably unedited, and a property chain
   reading the same avar agreeing with exec.
2. Registry and bridge: per-session overrides, republish-now, the two C
   entry points. Covered through the existing imaging test.
3. Python: `Writer` preview mode and the four lifecycle hooks, plus frame
   read-back from the published generation. Headless test that preview
   authors nothing and commit authors once.
4. testusdview: a drag whose mid-gesture assertion is that the edit target
   layer holds no spec for the avar, and whose post-release assertion is
   the committed value and a single undo entry.
5. Lane 2, if in scope: the xform override scene index and the two xform
   targets.

## What was built

Both lanes, in one pass, replacing the old live-authoring path rather than
sitting beside it behind a setting.

Two things differ from the plan above, and both were found by the code rather
than by thinking harder about it.

**The xform lane stores a world-space DELTA, not a local matrix.** The plan
copied ImGuiHydraEditor's `SetXform` literally. That is right for their chain
and wrong for this one: by the time a prim reaches the RigExec filters its
transform is already flattened -- which is why the results index marks its own
driven matrices `resetXformStack = true` (`_ComputeDrivenXform`) -- so a local
matrix would drop every ancestor, and overriding only the dragged prim would
leave its children at the parent's old place. What the filter holds is a
post-multiplied delta:

    xform'(X) = xform(X) . delta   for X at or under the previewed prim

which is the move when applied to the prim and the same move when applied to a
descendant, costs one multiply in `GetPrim`, and is the quantity a manipulator
already computes. `TestXformPreviewDelta` asserts the child's position with a
number that distinguishes all three candidate behaviours.

**Interactive overrides are applied on BOTH sides of the property chains.** The
plan said "after", so that a held drag outranks the rig's own arithmetic. That
is only half of it: an override on a value a chain READS -- a control avar
feeding a math mover -- has to be in place before the chain runs, or dragging
that control moves everything except what the mover drives. Which of the two
situations an override is in is not knowable in `Evaluate`, so it is applied
twice, and the published result follows it as well (otherwise the viewport
would show the chain's arithmetic while every exec consumer saw the held
value -- the disagreement between the two delivery routes that the evaluator
already warns about).

One residual, deliberately: creating an `xformOp` is still authored during the
drag. A value has nowhere to live until its attribute exists, and the preview
is keyed by attribute path, so the first sample of a drag on a prim with no ops
pays for creating them -- once per drag, not once per sample, which
`TestOneNoticePerDrag` pins. An aborted drag removes them, because the undo
recorder captured their non-existence.

The Writer turned out to own more than "where the value lands". It publishes
each collected value into `gizmoMath`'s preview map as it collects it, because
a Preserve Children pivot drag writes the parent and then computes the
children's compensation FROM the parent's new state. While those writes were
authored immediately an ordinary stage read saw them; now the uncommitted value
has to be visible the same way, or the second half of such a drag computes
from the first half's pre-drag values. That one line is what made the change
small: every other read in the frame maths gets the preview for free.

### Where it lives

| Piece | File |
|---|---|
| Evaluation-time overrides | `RigExecRigEvaluator::SetInteractiveOverrides`, libs/rigExec/rigEvaluator.{h,cpp} |
| Xform lane | `RigExecXformOverrideSceneIndex`, libs/rigExecImaging/sceneIndices.{h,cpp} |
| Preview session, lane routing, delta composition | `RigExecImagingRegistry::BeginPreview` / `UpdatePreview` / `EndPreview`, libs/rigExecImaging/registry.cpp |
| C entry points | `RigExecImaging_BeginPreview` / `_UpdatePreview` / `_EndPreview` |
| Collect-then-author | `gizmoMath.Writer`, plugin/rigExecUsdview/gizmoMath.py |
| Preview protocol | plugin/rigExecUsdview/gizmoPreview.py |
| Drag lifecycle | `GizmoController._BeginDrag` / `_UpdateDrag` / `_EndDrag` / `_AbortDrag` |

### Tests

| What | Where |
|---|---|
| Overrides change the pose and author nothing; both chain orderings; clear restores | `TestInteractiveOverrides`, tests/testRigExecInteractive.cpp |
| The delta reaches Hydra and carries descendants; dirty notices; release restores | `TestXformPreviewDelta`, tests/testRigExecImaging.cpp |
| A whole drag is zero notices, and its release is one | `TestOneNoticePerDrag`, tests/python/test_gizmo_math.py |
| Collect vs author, warnings at the commit, an abandoned drag | `TestWriter`, tests/python/test_gizmo_math.py |
| The protocol: declare once, numbers per sample, arity, no sink | tests/python/test_gizmo_preview.py |
| A real drag in the real app: no spec mid-drag, handles following, one undo entry, Escape | `_TestPreviewThenCommit`, tests/testUsdviewGizmo.py |

### Still open

The third open question above -- scrubbing the timeline mid-drag -- is
unresolved by design rather than by omission. The overrides are
time-independent values, so they apply at the new time too, which is probably
right and is certainly what happens; nothing tests it yet.
