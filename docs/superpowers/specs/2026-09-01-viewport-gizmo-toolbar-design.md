# Viewport gizmo toolbar for usdview (translate / rotate / scale, undoable)

Date: 2026-09-01. Status: approved for implementation under the assumptions
in section 1 (the request was executed autonomously; the assumptions are the
interpretation a careful colleague would make and are the first thing to
revisit if the result feels wrong).

## 0. Request

"Similar to the weight/volume visualizer UI, make a usdview viewport toolbar
that lets me invoke undoable/redoable (Ctrl-Z / Ctrl-Shift-Z) gizmos. The
first tool is a translate/rotate/scale gizmo that takes the default/rest
position into account and modifies the avars; it can switch between authoring
to animation or default, and pivot or pose, for xforms, joints or controllers."

## 1. Assumptions (decisions made without a live user)

1. **Animation vs Default** = *where in time the value lands*. Animation
   writes at usdview's current frame using the same rule the volume weight
   panel uses (`volumeWeightUI.SetAtTime`: a `Ts.Spline` knot for double
   attributes, a time sample otherwise). Default writes the attribute's
   default value (`attr.Set(value)`). The rig's `default:*` channels are
   *not* read by the evaluator (`libs/rigExec/computations.cpp` reads only
   `rest:*`, `avars:*`, `posed:space`), so they are not what "default" means.
2. **Pose vs Pivot** = *which channel set the gizmo edits*. Pose edits
   `avars:tx/ty/tz`, `avars:rx/ry/rz`, `avars:sx/sy/sz`. Pivot edits the rest
   offset `rest:tx/ty/tz`, `rest:rx/ry/rz` (the local frame the avars are
   applied in; moving it moves where the control "lives" and the pose rides
   along, like moving a Maya pivot without compensation). Rest spaces are
   orthonormalized by the evaluator, so **scale is unavailable in Pivot mode**
   for rig prims. For plain xforms Pivot edits the `UsdGeomXformCommonAPI`
   pivot.
3. **Targets**: exactly one prim, usdview's focus prim. Supported kinds are
   `RigExecControl`, `RigExecJoint` (the "controllers" and "joints") and any
   `UsdGeomXformable` whose op stack is `UsdGeomXformCommonAPI`-compatible
   (the "xforms"). Anything else shows a reason in the toolbar status and
   draws no gizmo. Multi-selection is out of scope.
4. **Solver-posed joints** (any joint targeted by a solver's `rigExec:joints`)
   and xformables with a connected or non-identity `posed:space` are not
   editable through avars (the evaluator ignores avars for them), so the
   gizmo is disabled for them with a reason. An avar with an authored
   connection (e.g. `avars:rz.connect`) is likewise refused. A control that
   a constraint names in `rigExec:moves` publishes a revised frame; the
   gizmo draws at the unconstrained frame and this is a documented
   limitation, not an error.
5. **Gizmo axes are the target's local frame** (the rest frame carried into
   the posed parent space, exactly the frame the avars are expressed in).
   No world/local orientation toggle in v1.
6. **Edit target** is whatever `stage.GetEditTarget()` is (usdview starts on
   the session layer), matching both existing panels. Undo restores the same
   layer it captured from.
7. The overlay is drawn with QPainter on a transparent child widget of the
   stage view rather than as session-layer prims (the curvenet approach):
   a gizmo must be screen-constant, unoccluded, and must not trigger rig
   re-evaluation on every camera move.

## 2. Evaluator facts the design relies on

From `libs/rigExec/computations.cpp` (row-vector convention: leftmost
factor applies first):

```
avars  = S(sx,sy,sz) * R(rotationOrder; rx,ry,rz) * Rspin(x; rspin) * T(tx,ty,tz)
rest   = orthonormalize( compose(rest:tx..rz, no scale, order XYZ) * rest:space )
world  = avars * rest * parentRest^-1 * parentPosed
```

where parent is the nearest namespace-ancestor `RigExecXformable`; a rig
carries no stage placement of its own, so asset space is the world transform
of the `RigExecRoot`'s parent prim (`UsdGeom.XformCache`). Defining
`P = rest * parentRest^-1 * parentPosed` (the frame the avars live in) gives:

- Pose translate: `Δ(tx,ty,tz) = worldDelta * linear(P)^-1`
- Pose rotate by world rotation `Rw`:
  `R_new = R_old * linear(P) * Rw * linear(P)^-1`, then
  `R_new * Rspin^-1` is decomposed into `avars:rotationOrder` Euler angles,
  choosing the solution nearest the previous angles.
- Pose scale: multiply `sx/sy/sz` (local axes), then apply the evaluator's
  floor (`RigExecNormalizeAvarScale`: finite magnitudes below 1e-4 become
  signed 1e-4) so the written value is what the viewport shows.
- Pivot: with `Q = rest:space * parentRest^-1 * parentPosed` and
  `restLocal = compose(rest:t, rest:r)`, the gizmo sits at `restLocal * Q`
  and translate/rotate deltas are mapped through `linear(Q)^-1` the same way.

The evaluator observes `Usd.Notice.ObjectsChanged` and republishes
synchronously; nothing needs to be told to re-evaluate. `RigExecImaging`
exports no frame query, so the Python side replicates the composition above
(controls and unwired joints only; see assumption 4). The replica is
verified in the unit tests against the native evaluator through the
`_rigexec` Python binding (`_rigexec.Rig(stage, rigPath).compile()`,
`.evaluate(time).control_frame(path).to_matrix4()`, asset space).

## 3. Architecture

All new code lives in `plugin/rigExecUsdview/`, following the repo rule that
everything above the Qt banner is Qt-free and headlessly testable.

### 3.1 `rigExecUndo.py` (Qt-free)

- `AttributeSnapshot.Capture(layer, specPath)` records, for one attribute
  spec in one layer: whether the spec exists, its default (or absence), its
  spline (copy, or absence), and its time samples `{t: v}`.
  `Restore()` puts exactly that state back inside an `Sdf.ChangeBlock`,
  removing the spec if it did not exist.
- `Edit(label, entries)` where entries are `(layer, specPath, before, after)`.
- `UndoStack`: `Push(edit)`, `Undo()`, `Redo()`, `CanUndo()`, `CanRedo()`,
  `UndoText()`, `RedoText()`, `Clear()`, `AddListener(fn)`; bounded to 200
  edits; a push clears the redo branch.
- `EditRecorder(stage, attrPaths)`: `Begin()` captures "before" for each
  attribute at `stage.GetEditTarget().MapToSpecPath(path)` in the edit
  target's layer; `Commit(label)` captures "after" and returns an `Edit`
  (or `None` if nothing changed); `Abort()` restores "before".

### 3.2 `gizmoMath.py` (Qt-free)

- Euler helpers matching the evaluator: `ComposeAvarMatrix(...)`,
  `RotationMatrixFromEuler(order, rx, ry, rz)`,
  `DecomposeEuler(matrix, order, hint=None)`.
- `RigFrames(stage, prim, time)` → the replica of section 2 for a
  `RigExecXformable`: `posedWorld`, `P`, `Q`, `restLocal`, `avars` values,
  plus `reason` when the prim is not editable (solver-posed, posed:space
  authority, not under a `RigExecRoot`).
- Targets (one class each, same interface):
  `RigPoseTarget`, `RigPivotTarget`, `XformPoseTarget`, `XformPivotTarget`.
  `MakeTarget(stage, prim, channelMode, time)` → target or `(None, reason)`.
  Interface: `gizmoMatrix()` (orthonormal world frame to draw at),
  `supportsScale`, `attributePaths()`, `BeginDrag()` (records base values),
  `ApplyTranslate(worldDelta)`, `ApplyRotate(worldAxis, degrees)`,
  `ApplyScale(axisIndexOrNone, factor)`. Every `Apply*` computes from the
  base values (never incrementally) and writes through a `Writer`.
- `Writer(stage, time, writeMode)`: `Set(attr, value)` → `SetAtTime` rule
  for `ANIMATION`, `attr.Set(value)` for `DEFAULT`; `Warnings()` lists
  attributes whose spline/time samples outrank the default when in
  `DEFAULT` mode.
### 3.2b `gizmoScreen.py` (Qt-free)

Screen-space helpers, pure functions over `Gf` types so they can be unit
tested with a synthetic `Gf.Camera`: `ProjectPoint(viewProj, viewport, p)`,
`WorldPerPixel(...)`, `BuildHandles(tool, gizmoMatrix, camera, viewport,
pixelRatio)`, `HitTest(handles, x, y, radius)`, `AxisDragParameter(...)`,
`PlaneDragDelta(...)`, `RotationDragAngle(...)`, `ScaleDragFactor(...)`.

### 3.3 `gizmoUI.py` (Qt)

- `ViewportToolbar(QToolBar)`: text-labelled checkable tool buttons in an
  exclusive group (`Select`, `Translate`, `Rotate`, `Scale`), a
  `Channels` pair (`Pose` | `Pivot`), a `Write` pair (`Animation` |
  `Default`), `Undo` / `Redo` buttons and a status label that always says why
  a gizmo is absent. Inserted at index 0 of the stage view's parent
  layout (`appController._ui.glFrame.layout()`), above the view.
- `GizmoOverlay(QWidget)`: transparent, mouse-transparent child of the stage
  view, resized with it, paints the handles for the active tool from the
  controller's projected geometry. Colours: X red, Y green, Z blue, hover and
  active handle yellow, centre handle white.
- `GizmoController(QObject)`: owns tool state, the target, the drag, and the
  event filter on the stage view. Press on a handle (left button, no
  Alt/Meta) starts a drag and consumes the event; press elsewhere passes
  through so usdview keeps its own picking. Move during a drag writes values
  inside `Sdf.ChangeBlock` and calls `usdviewApi.UpdateViewport()`. Release
  commits the `Edit` to the undo stack. Escape during a drag aborts and
  restores. The overlay repaints on view paint/resize events,
  `signalFrustumChanged`, selection changes, frame changes and
  `ObjectsChanged` notices.
- Shortcuts: `Ctrl+Z` → undo, `Ctrl+Shift+Z` and `QKeySequence.Redo` →
  redo, `Qt.ApplicationShortcut` context on the main window, so they work
  wherever focus is.
- `InstallViewportTools(usdviewApi, undoStack)` → controller (idempotent);
  `HandleScreenPositions()` and a few other methods stay public for tests.

### 3.4 `rigExecUsdview.py` (container)

- Lazily creates the shared `UndoStack` and installs the viewport tools on
  the first `signalStageReplaced` (plugins load before the stage view
  exists). The stack is cleared on stage replacement. A `RigExec →
  Viewport Tools` menu item toggles the toolbar. Headless contexts (no Qt,
  no stage view) skip installation exactly as the panels do.

## 4. Data flow of one drag

1. Press: hit-test projected handles; on a hit, `EditRecorder.Begin()`
   snapshots the attribute specs, `target.BeginDrag()` records base values.
2. Move: convert the screen delta into a world-space translate / rotate
   angle / scale factor with the pure helpers, call `target.Apply*`, which
   writes via `Writer` inside a change block; the evaluator republishes;
   overlay and viewport update.
3. Release: `EditRecorder.Commit(label)`; push to the undo stack; the toolbar
   enables Undo. Undo/redo restore the spec snapshots in the captured layer.

## 5. Error handling

- No target / unsupported prim / solver-posed joint / incompatible xform op
  stack → no gizmo, status label explains.
- `DEFAULT` write with an outranking spline → the write still happens and
  the status label warns.
- Stage replaced mid-drag → drag aborted, stack cleared.
- Undo of an edit whose layer is gone → the entry is dropped with a warning.

## 6. Testing

- `tests/python/test_gizmo_math.py` (Qt-free, registered in CMake like the
  other Python tests): Euler round-trips for all six orders with hint
  continuity; the frame replica compared against `_rigexec` evaluation on
  a two-level control chain with rotated `rest:space` and non-zero rest
  offsets; pose translate / rotate / scale on that chain; pivot mode edits `rest:*`; solver-posed joint
  refused; `XformCommonAPI` targets; snapshot capture/restore for default,
  spline knot, time sample and absent spec; undo/redo/redo-branch clearing;
  `Writer` animation vs default including the spline-outranks-default
  warning.
- `tests/testUsdviewGizmo.py` + `bin/run_testusdview_gizmo.sh` (testusdview
  on `examples/ArmShotAnim.usda`): installs the tools, selects a control,
  drives synthetic mouse events through the projected handle positions,
  asserts the avar changed at the current frame, Ctrl+Z restores it,
  Ctrl+Shift+Z re-applies it, Default mode writes the default, Pivot mode
  writes `rest:*`, and a plain Xform edits its xformOps. Saves a window
  grab to `$RIGEXEC_GIZMO_SHOT` when set, and prints `RIGEXEC_GIZMO_OK`.

## 7. Out of scope (deliberately)

World/local orientation toggle, multi-prim editing, snapping, hotkeys for
tool switching, undo integration for the existing panels (the stack is
shared and they can adopt it later), a C++ frame-query export.
