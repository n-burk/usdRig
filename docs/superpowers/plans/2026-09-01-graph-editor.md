# Graph Editor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Maya-style Graph Editor window in usdview that lists the animation curves of the selected properties (or all animated properties of the selected prim), lets the artist move keys, insert/delete keys, change tangent types, drag tangent handles and set infinity, all undoable through the shared undo stack; and Maya-default (auto) tangents on knots the gizmo authors.

**Architecture:** Two Qt-free modules (`graphModel.py` for curve discovery and Ts.Spline edit operations, `graphScreen.py` for view transforms, sampling, hit-testing and drag resolution) drive one Qt module (`graphEditorUI.py`: panel, curve list, canvas), opened from the RigExec menu and the viewport toolbar; writes go through `rigExecUndo.EditRecorder` + `attr.SetSpline`.

**Tech Stack:** Python 3.11, OpenUSD PR-4156 (`Ts`, `Usd`, `Sdf`, `Gf`), PySide6 via `pxr.Usdviewq.qt`, testusdview, CMake/ctest.

**Spec:** `docs/superpowers/specs/2026-09-01-graph-editor-design.md` (binding). Reuse and follow `docs/superpowers/specs/2026-09-01-viewport-gizmo-toolbar-design.md` for undo, writer and plugin conventions.

## Global Constraints

- Environment: `. bin/_env.sh`; headless tests run via `bin/run_python_tests.sh <name>` (add new tests to its default list and to `SCHEMA_TESTS` if they read argv); register in `CMakeLists.txt` next to `testGizmoDrag` (same foreach list) — the working tree carries the owner's unrelated CMake edits: touch only the registration lines.
- Style: as `volumeWeightUI.py` / the gizmo modules — Qt only from `pxr.Usdviewq.qt`; PascalCase methods, `_onXxx` slots, UPPER_SNAKE constants, 79 columns, why-docstrings citing file:line; tests are plain scripts with `_Check`, `(name, callable)` groups, an `..._OK` banner, and `rigexec_test_env.SetupPluginTest()` above the pxr import.
- Qt-free: `graphModel.py`, `graphScreen.py`.
- Ts facts (verified in this build): `Ts.Spline("double")`, `Ts.Knot(typeName=, time=, value=, nextInterp=)`, `Get/SetPreTanSlope/Width`, `Get/SetPostTanSlope/Width`, `Get/SetPreTanAlgorithm/PostTanAlgorithm` (`Ts.TangentAlgorithmNone/AutoEase/Custom`), `Ts.InterpHeld/Linear/Curve`, `Ts.ExtrapHeld/Linear/LoopRepeat/LoopReset/LoopOscillate`, `spline.GetKnots()` is a `Ts.KnotMap` (`.values()`), `spline.SetKnot/RemoveKnot/Eval/EvalDerivative`, `spline.Sample(Gf.Interval(t0, t1), timeScale, valueScale, tolerance, withSources=False).polylines` (see `/Users/burkard/work/usd-pr4156/pxr/usdImaging/usdviewq/splineViewer.py:179-218` and `:320-420` for a working reference of sampling and tangent geometry), `spline.Set/GetPreExtrapolation`, `Ts.Extrapolation(mode)`. `Ts.Knot.GetCurveType` is deprecated (warns) — use the spline's curve type.
- All edits are whole-spline writes through `rigExecUndo.EditRecorder(stage, [attrPath])` → `attr.SetSpline(spline)` → `Commit(label)` → `undoStack.Push`.
- Workers do not commit; the controller commits.

---

### Task G1: `graphModel.py` — curve discovery and Ts edit operations (+ Maya-default knots in the gizmo writer)

**Files:** Create `plugin/rigExecUsdview/graphModel.py`, `tests/python/test_graph_model.py`; Modify `plugin/rigExecUsdview/gizmoMath.py` (`SetAnimated` only) and `tests/python/test_gizmo_math.py` (its writer group asserts the new tangent defaults).

**Interfaces produced:**
- `class CurveRef(primPath: Sdf.Path, attrName: str, color: (r, g, b))` with `attrPath` property and `Label()` (`"<primName>.<attrName>"`); `CurveColor(attrName, index) -> (r, g, b)` (tx/rx/sx red `(0.95, 0.3, 0.3)`, ty/ry/sy green `(0.3, 0.85, 0.3)`, tz/rz/sz blue `(0.35, 0.55, 1.0)`, rspin yellow `(0.95, 0.85, 0.3)`, else a 6-colour palette by index).
- `DiscoverCurves(stage, propPaths, primPaths) -> list[CurveRef]`: spline-capable scalar attributes among `propPaths` if any qualifies; else, per prim in `primPaths`, every attribute that `HasSpline()` plus the rig avar/rest scalar channels of RigExec controls/joints (`gizmoMath.AVAR_T + AVAR_R + AVAR_S + (AVAR_RSPIN,) + REST_T + REST_R`), in that order, deduplicated.
- `AuthorKnot(spline, time, value) -> Ts.Knot`: creates or updates the knot at `time` with `value`, `nextInterp=Ts.InterpCurve`, pre and post tangent algorithm `AutoEase` when creating (an existing knot keeps its tangents), `spline.SetKnot`, returns the knot.
- `gizmoMath.SetAnimated` uses `AuthorKnot` for the spline arm (import graphModel lazily inside the function to avoid import cycles).
- Edit operations, each taking a `Ts.Spline` COPY and mutating it (callers copy with `Ts.Spline(spline)`): `MoveKeys(spline, times, dt, dv, snapFrames=True) -> list[float]` (returns the new times; clamps so keys never cross neighbours: a moved key's new time is limited to the open interval between the nearest unmoved neighbours, minus 1 frame when snapping), `InsertKey(spline, time) -> Ts.Knot` (value = `spline.Eval(time)`, Auto tangents, or the existing knot), `DeleteKeys(spline, times)`, `SetTangentType(spline, times, mode, side="both")` with `mode in ("auto", "spline", "linear", "flat", "step")` per spec 2.4, `SetTangent(spline, time, side, slope, width=None)` (custom algorithm; `width=None` keeps the width), `BreakTangents(spline, times)` (marks custom on both sides so they move independently; returns nothing), `UnifyTangents(spline, times)` (copies the out slope to the in slope, custom), `SetExtrapolation(spline, pre=None, post=None)` with names `"constant" | "linear" | "cycle" | "cycle_offset" | "oscillate"` mapped per spec 2.4 (verify the LoopRepeat/LoopReset meaning against the Ts headers `/Users/burkard/work/usd-pr4156/pxr/base/ts/types.h` and record the result in the docstring), `SnapTime(t) -> float` (round half away from zero), `KeyNeighbours(spline, time) -> (prevTime | None, nextTime | None)`, `IsUnified(knot) -> bool`.
- `ApplySpline(stage, attrPath, spline, undoStack, label) -> bool`: `EditRecorder([attrPath]).Begin()`, `attr.SetSpline(spline)`, `Commit(label)` → `Push` when changed; returns whether an edit was pushed.

**Tests** (`test_graph_model.py`, groups): discovery from property selection (a selected `avars:tx` on HandIK-like control; a selected vector attribute is ignored), discovery from prim selection (splined attrs first then unanimated avars, no duplicates, colours), `AuthorKnot` defaults and update-in-place, `MoveKeys` incl. snapping and neighbour clamping, `InsertKey` value equals `Eval`, `DeleteKeys`, each tangent mode's observable effect (algorithm / interpolation / slope values, `Eval` at midpoints for `flat` vs `spline`), `SetTangent` slope-only vs width, break/unify, extrapolation mapping (assert `GetPreExtrapolation().mode`), `ApplySpline` writes the session layer and one `undoStack.Undo()` restores the previous spline exactly. Also update `TestWriter` in `test_gizmo_math.py` to assert the knot the writer authors has AutoEase tangents on both sides.

### Task G2: `graphScreen.py` — view transform, sampling, hit-testing, drag resolution

**Files:** Create `plugin/rigExecUsdview/graphScreen.py`, `tests/python/test_graph_screen.py`.

**Interfaces produced:**
- `class ViewTransform(width, height, margins=(48, 24, 24, 28))` with `timeRange`, `valueRange` (floats), `TimeToX/XToTime`, `ValueToY/YToValue`, `ToPixel(t, v)`, `FromPixel(x, y)`, `Pan(dx, dy)` (pixels), `ZoomAbout(x, y, factorX, factorY)`, `Frame(tMin, tMax, vMin, vMax, padding=0.1)` (degenerate ranges get a ±1 pad), `PlotRect()`.
- `NiceStep(rangeLength, pixels, targetPixels=60) -> float` (1/2/5 × 10^n), `GridLines(transform) -> (timeTicks, valueTicks)`.
- `SamplePolylines(spline, transform, interval=None) -> list[list[(x, y)]]` in PIXELS (uses `spline.Sample` with the transform's scales; empty spline → []).
- `class KeyGlyph(curveIndex, time, value, x, y, selected)`; `class TangentGlyph(curveIndex, time, side, x, y, slope, width, locked)`; `KeyGlyphs(curves, splines, transform, selection)`, `TangentGlyphs(...)` for the selected keys of `InterpCurve` neighbours per spec 2.2.
- `HitKey(keys, x, y, radius=6)`, `HitTangent(tangents, x, y, radius=6)`, `KeysInRect(keys, rect)`.
- `ResolveKeyDrag(transform, press, current, constrainAxis=None) -> (dt, dv)` (pixel delta → time/value delta; `constrainAxis` "time" | "value" chosen from the dominant axis when Shift is held).
- `ResolveTangentDrag(transform, keyPixel, handlePixel, weighted, currentWidth) -> (slope, width)` (slope from the handle vector in curve space; width from its time extent when weighted, else `currentWidth`; in-tangents use the mirrored vector).
- `HitCurve(polylines, x, y, radius=5) -> curveIndex | None`, `CurveTimeAtX(transform, x)`.

**Tests**: transform round trips and Frame; NiceStep values; sampling of a two-knot spline hits the knot pixels; glyph positions; hit priorities; marquee; key drag with constraint; tangent drag slope/width for post and pre sides; curve hit.

### Task G3: `graphEditorUI.py` — panel, curve list, canvas, interaction, wiring

**Files:** Create `plugin/rigExecUsdview/graphEditorUI.py`; Modify `plugin/rigExecUsdview/rigExecUsdview.py` (menu item `Graph Editor`, lazy import like the other panels) and `plugin/rigExecUsdview/gizmoUI.py` (a `Graph…` action on the toolbar that opens it).

**Requirements** (spec section 2 is the checklist): layout 2.1, drawing 2.2 (QPainter; curve colours; dashed extrapolation; playhead; rulers; shaded outside stage range), navigation 2.3, selection and editing 2.4 (each gesture one `ApplySpline` call at release; live preview during the drag writes the spline every move event so the viewport follows, with the EditRecorder bracketing the whole gesture), integration 2.5 (property/prim selection signals, `currentFrameChanged` uses the signal's frame, `signalStageReplaced`, `ObjectsChanged` on listed attributes refreshes from the stage, undo/redo QActions on the panel window sharing the stack, `Graph…` toolbar button, `RigExec → Graph Editor` menu). Public for tests: `OpenGraphEditor(usdviewApi, undoStack) -> panel`, `GetGraphEditor()`, `panel.Curves()`, `panel.SetVisibleCurves(labels)`, `panel.canvas.KeyPixels() -> {curveLabel: [(x, y, time)]}` (logical px), `panel.canvas.TangentPixels()`, `panel.SelectedKeys()`, `panel.SetTangentType(mode)`, `panel.InsertKeyAtCurrentFrame()`, `panel.DeleteSelectedKeys()`, `panel.SetInfinity(pre, post)`, `panel.FrameAll()`, `panel.SetSnapFrames(bool)`, `panel.SetWeighted(bool)`, `panel.Status()`.

Smoke test through testusdview with screenshots (`/tmp/graph_smoke_*.png`): open the arm shot, select HandIK, open the editor, frame all, screenshot; select a key, screenshot (tangent handles visible); LOOK at the PNGs and iterate until they match spec 2.1-2.2.

### Task G4: end-to-end test, runner, docs

**Files:** Create `tests/testUsdviewGraphEditor.py`, `bin/run_testusdview_graph.sh`; Create `docs/graph-editor.md`; Modify `docs/viewport-gizmos.md` (one paragraph: Animation mode authors Ts spline knots with auto tangents for scalar channels; vector xformOps get time samples because Ts is scalar-only; link to the graph editor doc); Modify `README.md` (one bullet next to the gizmo bullet).

**Test sections** (each proves the behaviour on the stage, not just "changed"): curves listed for HandIK (`avars:tx/ty/tz` from the file's splines, plus unanimated avars), property-selection filtering via `dataModel.selection.addPropPath`, key drag moves the knot's time and value in the SESSION layer spline and changes `attr.Get(frame)`, Ctrl+Z restores the spline exactly, insert key at the current frame equals `Eval`, delete key, tangent type Flat gives zero slope and Step gives `InterpHeld`, tangent handle drag changes the post slope (and width when Weighted), break/unify, infinity mapping, playhead scrub changes `dataModel.currentFrame`, marquee selection count, screenshot to `$RIGEXEC_GRAPH_SHOT`, `RIGEXEC_GRAPH_OK`.
