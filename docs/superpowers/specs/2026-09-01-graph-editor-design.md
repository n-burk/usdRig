# Graph Editor for usdview animation curves (Maya-style)

Date: 2026-09-01. Status: approved for implementation under the assumptions
in section 1 (executed autonomously; the assumptions are the first thing to
revisit if the result feels wrong). Companion to
`2026-09-01-viewport-gizmo-toolbar-design.md`, whose undo stack, writer and
plugin conventions this design reuses.

## 0. Request

"The animation/pose authoring mode should author a Ts spline. We also need
an animation graph similar to Maya's where we can adjust values of the
animation curves (the properties selected in usdview or all animated
properties of the prim), change interpolation types and adjust spline
handles."

## 1. Assumptions and facts

1. **Ts splines are scalar.** In this USD build `Ts.Spline.IsSupportedValueType`
   is true for `double`, `float` (and `half`) and false for `double3`. Rig
   avars (`avars:tx` ... `avars:sz`, `rest:*`) are doubles and the gizmo's
   Animation mode already authors a spline knot for them (`gizmoMath.
   SetAnimated`); plain-xform `xformOp:translate/rotateXYZ/scale` are
   vector-typed and receive time samples because USD cannot store a spline
   on them. This is a USD limitation, stated in the docs; the graph editor
   edits splines only.
2. **New knots get Maya's default tangents**: both tangents `Ts.
   TangentAlgorithmAutoEase` (Maya "Auto"), `Ts.InterpCurve` for the
   following segment, Bezier curve type. `SetAnimated` is changed to set
   these algorithms; existing knots are left alone.
3. **Curve set** = the attributes selected in usdview's property browser
   (`dataModel.selection.getPropPaths()`) when at least one is a spline-
   capable scalar attribute; otherwise every attribute of the selected
   prim(s) that `HasSpline()` or is a scalar rig avar/rest channel (so an
   unanimated `avars:tx` can be keyed from the editor). Property selection
   and prim selection changes refresh the set. A curve list on the left
   (Maya's outliner column) toggles visibility per curve.
4. **Writes** go to the edit target (usdview: session layer) as whole-spline
   `attr.SetSpline(spline)` writes, one undo step per gesture, through the
   shared `rigExecUndo.UndoStack` and `EditRecorder` so Ctrl+Z / Ctrl+Shift+Z
   from the viewport toolbar also undo graph edits. The evaluator re-
   publishes on the resulting notice; nothing else drives the viewport.
5. **Time is in frames**; usdview's current frame is the playhead; dragging
   the playhead (or clicking the time ruler) scrubs usdview
   (`dataModel.currentFrame = Usd.TimeCode(f)`).
6. Out of scope (Maya features that need more than splines or are
   secondary): buffer curves, normalized/stacked display, retime tool,
   lattice deform, region tool, bookmarks, sound, key reduction, Euler
   filter, "Clamped" and "Plateau" tangent types (mapped to Auto with a note),
   "Step Next", channel-box-driven key insertion on non-spline attributes.

## 2. Maya parity definition

### 2.1 Layout

A `RigExec → Graph Editor` window (`QWidget`, `Qt.Window`, parented to
usdview's main window, singleton like the volume weight panel):

- Top row (Maya's key stats + toolbar): `Time` and `Value` double spin
  boxes for the selected keys (blank when the selection differs; typing
  moves/sets all selected keys); buttons `Frame All (A)`, `Frame Selected
  (F)`, `Insert Key (I)`, `Delete (Del)`; tangent buttons `Auto`, `Spline`,
  `Linear`, `Flat`, `Step`; `Break`, `Unify`; `Weighted` toggle; `Snap
  Frames` toggle (default on); `Infinity` pre/post combos (`Constant`,
  `Linear`, `Cycle`, `Cycle w/ Offset`, `Oscillate`); a status label.
- Left column: the curve list, one row per curve: colour swatch, prim
  name, attribute name, visibility checkbox; selecting rows isolates those
  curves (Maya's outliner behaviour); `Show All`.
- Centre: the graph canvas.

### 2.2 Canvas drawing

- Time along X (frames), value along Y (up). Grid lines at "nice" frame /
  value steps with labels on the bottom / left rulers; a vertical playhead
  at the current frame with the frame number; usdview's start/end frames
  shaded outside the range.
- Curves sampled with `Ts.Spline.Sample(interval, timeScale, valueScale,
  tolerance)` and drawn as polylines in their curve colour; extrapolated
  parts drawn dashed.
- Curve colours: `tx/rx/sx` red, `ty/ry/sy` green, `tz/rz/sz` blue,
  `rspin` yellow, others from a fixed palette cycling (Maya's per-curve
  colouring).
- Keys: filled squares (6 px) in the curve colour; selected keys yellow;
  hovered key pale yellow. Dual-valued knots draw both the pre-value and
  value squares.
- Tangent handles for SELECTED keys only (Maya): a line from the key to
  the handle end, end box 5 px; in-tangent and out-tangent both drawn for
  `InterpCurve` neighbours; handle end = `(t ± width, v ± width * slope)`
  in curve space; when the tangent algorithm is not `Custom` the handle is
  drawn hollow (locked) and dragging it switches the key to custom.
- Held segments draw as steps; linear segments as straight lines (these
  fall out of `Sample`).

### 2.3 Navigation

Alt + middle drag pans, Alt + right drag zooms (Maya camera keys); the
mouse wheel zooms about the cursor; `A` frames all visible curves, `F`
frames the selected keys (or all if none); dragging on the bottom ruler
scrubs the playhead; Home resets to the stage range.

### 2.4 Selection and editing (each gesture = one undo step)

- Left click a key selects it (Shift adds/removes; Ctrl toggles); marquee
  drag on empty space selects keys inside the rectangle; Escape clears.
- Left drag on a selected key moves all selected keys in time and value
  (Shift after press constrains to the dominant axis, Maya); times snap to
  whole frames while `Snap Frames` is on; a moved key never crosses its
  neighbours (clamped, Maya keeps key order).
- Middle drag moves the selected keys without changing the selection
  (Maya's "move nearest picked key" behaviour).
- Dragging a tangent handle end changes that tangent's slope (angle) and,
  when `Weighted` is on, its width; unified tangents (default) mirror the
  slope to the other side; `Break` makes the two sides independent;
  `Unify` re-links them (out slope copied to in).
- `Insert Key` / `I` inserts a key on every visible curve at the current
  frame with the curve's evaluated value and Auto tangents; double-click
  on a curve inserts a key at that time on that curve; `Delete`/`Backspace`
  removes selected keys.
- Tangent type buttons apply to the selected keys (both sides unless the
  In / Out radio narrows it): `Auto` → `TangentAlgorithmAutoEase`; `Spline`
  → custom slope `(v_next - v_prev) / (t_next - t_prev)` (Catmull-Rom,
  Maya's spline); `Linear` → the segment AFTER the key becomes
  `InterpLinear` (and before it for the In side); `Flat` → custom slope 0;
  `Step` → the segment after the key becomes `InterpHeld`. Any curve-type
  choice restores `InterpCurve` on the affected segments.
- `Weighted` toggles between width-preserving drags (weighted) and drags
  that only change slope with the width kept at its current value.
- `Infinity` pre/post combos set `SetPreExtrapolation` / `SetPostExtrapolation`
  on the selected curves: Constant → `ExtrapHeld`, Linear →
  `ExtrapLinear`, Cycle → `ExtrapLoopReset`, Cycle w/ Offset →
  `ExtrapLoopRepeat`, Oscillate → `ExtrapLoopOscillate` (the mapping is
  verified against the Ts docs by the implementer and stated in the docs).
- Numeric fields: editing `Time` or `Value` applies to all selected keys as
  one step.

### 2.5 Integration

- Opened from `RigExec → Graph Editor` and from a `Graph…` button on the
  viewport toolbar; `Ctrl+Z` / `Ctrl+Shift+Z` work while the editor has
  focus (its own QActions on the window, sharing the stack).
- usdview `currentFrameChanged` moves the playhead; `signalStageReplaced`
  clears the editor; `ObjectsChanged` on a listed attribute refreshes its
  curve (so gizmo drags show up live).

## 3. Architecture

`plugin/rigExecUsdview/`:

- `graphModel.py` (Qt-free): `CurveRef` (prim path, attribute name, colour),
  `DiscoverCurves(stage, propPaths, primPaths) -> list[CurveRef]`,
  `AuthorKnot(spline, time, value)` (Maya-default knot; used by
  `gizmoMath.SetAnimated`), the edit operations on `Ts.Spline` copies —
  `MoveKeys`, `InsertKey`, `DeleteKeys`, `SetTangentType`, `SetTangent`,
  `BreakTangents`, `UnifyTangents`, `SetExtrapolation`, `SnapTime` — the
  Maya→Ts mapping tables, and `ApplySpline(stage, attrPath, spline,
  undoStack, label)` (EditRecorder + `attr.SetSpline`).
- `graphScreen.py` (Qt-free): `ViewTransform` (frames/values ↔ pixels,
  pan, zoom about a point, frame-all / frame-selected from key extents),
  `NiceStep(range, pixels)`, `SamplePolylines(spline, transform,
  interval)`, key / tangent hit-testing, marquee containment, drag
  resolution (pixel delta → time/value delta with snapping and neighbour
  clamping, tangent end → slope/width).
- `graphEditorUI.py` (Qt): `GraphEditorPanel`, `CurveListWidget`,
  `GraphCanvas`, hotkeys, undo QActions; `OpenGraphEditor(usdviewApi,
  undoStack)`.
- `rigExecUsdview.py`: menu item; `gizmoUI.py`: the `Graph…` toolbar button.

## 4. Testing

- `tests/python/test_graph_model.py`: discovery from property vs prim
  selection; `AuthorKnot` defaults; each edit op on synthetic splines
  (values after `Eval`, tangent algorithms, interpolation modes,
  extrapolation); snapping and neighbour clamping; `ApplySpline` writes the
  session layer and one undo restores it.
- `tests/python/test_graph_screen.py`: transform round trips, nice steps,
  frame-all extents, hit-testing, drag resolution.
- `tests/testUsdviewGraphEditor.py` + `bin/run_testusdview_graph.sh`: open
  the arm shot, select `HandIK`, open the editor, assert the three
  translate curves list, drag a key (session spline changes, viewport
  frame value changes), Ctrl+Z restores, insert / delete key, tangent
  type buttons, tangent handle drag, infinity, property-selection
  filtering, playhead scrub; screenshot to `$RIGEXEC_GRAPH_SHOT`; prints
  `RIGEXEC_GRAPH_OK`.
