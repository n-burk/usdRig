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

## 8. Maya manipulator parity (added 2026-09-01, user direction)

The user directed mid-implementation: "the gizmos should be drawn and
manipulate as well as have the options exactly like Autodesk Maya's
viewport manipulators." This section is the binding definition of that
parity for this project; it supersedes sections 3.2b and 3.3 where they
differ. Reference: Autodesk Maya Move / Rotate / Scale Tool documentation
(Maya 2016-2023). Options that only make sense for polygon components
(Preserve UVs, Tweak mode, Soft Select, Symmetry, Transform Constraint,
Smart Duplicate, snap to live polygon / curve / point) are out of scope
and stated so in the docs.

### 8.1 Colours and sizing

- X red `(1, 0, 0)`, Y green `(0, 1, 0)`, Z blue `(0, 0, 1)`; the handle
  under the mouse (pre-selection) is drawn in Maya's pale highlight
  `(1.0, 0.85, 0.4)`; the SELECTED handle (the last one dragged, which
  middle-drag reuses) is yellow `(1, 1, 0)`; the view-plane / view-axis
  handles are light blue `(0.4, 0.75, 1.0)`; the free-rotate sphere
  silhouette is grey `(0.6, 0.6, 0.6)` at 50% opacity.
- Manipulator size is a session setting in logical pixels (default 90);
  `+` / `-` keys grow / shrink it by 10% (Maya's "+ / - to resize
  handles"). Line width 2 px logical; handles are screen-constant.
- Axis handles that point at the camera (projected length below the lock
  threshold) are drawn dimmed at 40% opacity and cannot be grabbed.

### 8.2 Move manipulator (Maya Move Tool, hotkey W)

Drawn: three axis lines from the origin ending in solid cone arrowheads
(tip at the axis end; cone base radius 5% of the manipulator size); three
planar handles: small filled squares whose side is 15% of the size,
placed in each axis pair's plane at 30% along both axes, each coloured
like the axis PERPENDICULAR to its plane (YZ red, XZ green, XY blue);
a centre square (side 12% of the size) in light blue for view-plane
moves.

Manipulate: axis drag moves along that axis (screen-projected travel, as
implemented); planar drag moves in that plane by ray/plane intersection
so the grabbed point stays under the cursor; centre drag moves in the
camera plane. `Ctrl` + axis drag moves in the plane perpendicular to that
axis (Maya). Middle-mouse drag anywhere in the viewport repeats the
selected (yellow) handle without having to hit it. Values written are
the channel values the drag implies in the target's channel space
(section 2), whatever orientation the handles are drawn in.

Options (toolbar "Tool Settings" for the active tool):
- Axis Orientation: `World` (default), `Object` (handles follow the
  target's posed orientation), `Parent` (handles follow the space the
  channels are expressed in: P / Q for rig prims, the parent xform for
  plain xforms). Maya's Component / Normal / Along Live Object / Custom
  are out of scope.
- Step Snap (Maya "Discrete move", default off) with Step Size (default
  1.0 units): deltas are quantised to multiples of the step, relative to
  the drag start. Holding `J` enables it for the duration of the drag.
  Holding `X` snaps the resulting translation to a grid of the step size
  (absolute, in the drag frame).
- Preserve Children (default off): for a plain xform whose children are
  XformCommonAPI-compatible xformables with a zero pivot, the children's
  world transforms are re-authored after the drag so they do not move.
  Not available for rig prims (children of a control are rig-evaluated);
  the option is disabled with a reason in that case.
- Edit Pivot (Maya `D` hold / `Insert` toggle): maps to the Channels
  Pivot mode; `D` and `Insert` toggle Pivot / Pose.

### 8.3 Rotate manipulator (Maya Rotate Tool, hotkey E)

Drawn: three rings in X/Y/Z colours at the manipulator radius, with the
BACK half of each ring (points whose depth is behind the ring centre
along the view direction) hidden; an outer light-blue ring at 1.25x the
radius facing the camera (view-axis rotation); a grey sphere silhouette
at the radius for free rotation. While dragging a ring, a pie slice from
the drag-start angle to the current angle is filled in the ring's colour
at 30% opacity in the ring's plane (Maya's rotation amount display), and
the angle in degrees is shown in the status label.

Manipulate: ring drag rotates about that ring's axis by the angle swept
around the ring centre on screen, accumulated continuously across the
drag (no wrap at 180, Maya keeps counting); view ring rotates about the
camera view direction; free rotate (drag inside the sphere, not on a
ring; Maya "Free Rotate", default on, can be turned off) is a virtual
trackball: the axis is perpendicular to the mouse travel in the camera
plane and dragging one diameter sweeps 180 degrees. Middle-drag
anywhere repeats the selected ring.

Options:
- Rotate Axis: `Object` (default), `World`, `Gimbal` (each ring changes
  exactly one Euler channel: for rotation order (i, j, k) applied i
  first, the k ring is the parent-space k axis, the j ring is the j axis
  rotated by Rk, the i ring is the object-space i axis).
- Step Snap (Maya "Snap rotate", default off), Step Size default 15
  degrees; `J` hold enables it.
- Free Rotate (default on).
- Preserve Children as in 8.2.
- Edit Pivot as in 8.2.

### 8.4 Scale manipulator (Maya Scale Tool, hotkey R)

Drawn: three axis lines ending in solid cubes (drawn as filled squares,
side 8% of the size); a centre cube (side 12%) for uniform scale; three
planar handles as in 8.2 (2-axis scale).

Manipulate: axis drag scales that axis by (distance of the cursor's
projection along the axis from the origin) / (distance at press), so
dragging the handle to the origin gives 0 and through it flips the sign
unless Prevent Negative Scale is on (then clamped to 1e-4); planar drag
scales the two axes of the plane by the same rule measured along the
plane diagonal; centre drag scales uniformly by horizontal travel
(`1 + dx / size`). Middle-drag anywhere repeats the selected handle.
Scale is always applied in the target's own axes; the Axis Orientation
option changes only where the handles are drawn (Maya scales in object
space when the orientation is not aligned; we keep the values on the
target's channels).

Options: Axis Orientation `World` (default) / `Object` / `Parent`; Step
Snap (default off, step 1.0); Prevent Negative Scale (default off);
Preserve Children; Edit Pivot.

### 8.5 Hotkeys (Maya defaults, plus the user's undo keys)

`Q` select, `W` move, `E` rotate, `R` scale; `+` / `-` manipulator
size; `D` (toggle) and `Insert` edit pivot; `J` hold step snap, `X` hold
grid snap (move only); `Ctrl+Z` undo; `Ctrl+Shift+Z`, `Shift+Z`
(Maya) and `Ctrl+Y` redo; `Escape` aborts a drag. Tool hotkeys are
active only while the stage view has focus so they cannot shadow text
fields; undo / redo are application-wide. Any key already bound by
usdview's main window is left to usdview and documented as skipped.

### 8.6 Tool Settings UI

The toolbar keeps `Select / Move / Rotate / Scale`, `Channels`,
`Write`, `Undo / Redo`, and the status label. A `Tool Settings` button
opens a small floating panel (a `QDockWidget`-free `QWidget` window
parented to usdview, like the volume weight panel) whose rows depend on
the active tool: Axis Orientation combo, Step Snap checkbox + Step Size
spin box, Free Rotate checkbox (rotate), Prevent Negative Scale checkbox
(scale), Preserve Children checkbox, Manipulator Size spin box, and a
`Reset Tool` button restoring the Maya defaults. Settings persist for the
session only.
