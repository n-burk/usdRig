# Gizmo Snapping Implementation Plan

> **For the executing agent:** work task by task, in order. Each task
> ends with its tests passing; do not start the next on a failing one.
> Do NOT commit — the reviewer commits after checking.

**Goal:** Maya-style grid / point / edge / surface snapping for the
viewport gizmo's Move tool, and an absolute degree grid on Rotate's
Gimbal rings, as momentary holds (`X` / `C` / `V`) and a sticky `Snap:`
mode.

**Architecture:** One new Qt-free module `gizmoSnap.py` owns every snap
number (the world-grid rule, the handle constraint, screen-space
ranking, the perspective-correct edge parameter). `gizmoDrag.py` picks
the active mode and turns a candidate into a plain world delta, which
the existing `Target.ApplyTranslate` writes — **no new `Target` API**.
`gizmoUI.py` supplies the hydra pick through an injected resolver so
`gizmoDrag` stays Qt-free.

**Spec:** `docs/superpowers/specs/2026-09-03-gizmo-snapping-design.md` —
read it first; every rule and number below comes from it.

## Global Constraints

- **Environment:** `. bin/_env.sh` before every command. Headless tests:
  `bin/run_python_tests.sh [name]`. End-to-end: `bin/run_testusdview_gizmo.sh`.
  `bin/run_testusdview_viewcube.sh` must also still pass (it shares the
  stage view).
- **Style:** match `gizmoDrag.py` / `gizmoScreen.py`: `from pxr import
  Gf, ...`, Qt only through `pxr.Usdviewq.qt` and only in `gizmoUI.py`,
  PascalCase functions and methods, `_onXxx` for Qt slots, UPPER_SNAKE
  constants, 79-column lines, comments that explain WHY and cite
  `file:line` evidence.
- **Qt-free rule:** `gizmoSnap.py`, `gizmoDrag.py`, `gizmoScreen.py`,
  `gizmoSettings.py` and `gizmoMath.py` must not import Qt.
- **Row-vector convention** (`Gf.Matrix4d`, `v * M`). In Python
  `Gf.Vec3d * Gf.Matrix4d` does not exist: use `M.Transform(p)` for
  points and `M.TransformDir(d)` for directions. Angles in degrees at
  every public boundary.
- **Do not change these signatures**, or existing tests and call sites
  break: `ApplyDrag(state, current, settings, holdSnap=..., holdGrid=...,
  ctrl=...)` keeps those keyword names (new keywords may be *added* with
  defaults), and `Target.ApplyTranslate`'s keyword set stays exactly
  `snapStep` / `snapAbsolute` — five tests assert the kwarg dict by
  equality.
- **Never publish a snap marker through `Handles()`**: two end-to-end
  assertions require `HandleScreenPositions() == {}` for the Select tool.
- **Append to `Status()`, never replace a branch**: three end-to-end
  assertions substring-match the existing text (`"deg"`, `"outranked"`,
  `"unavailable"`).
- **Tests are plain scripts** (no pytest): `_Check(cond, msg)` raising
  `AssertionError`, a `main()` running `(name, callable)` groups, exit 0
  with a printed `..._OK` banner.
- **Files you may touch:** only those a task names. The working tree
  carries unrelated uncommitted user work and a just-finished ViewCube
  feature; never reformat, revert, stash, checkout or reset anything,
  and never `git commit` / `git add`.
- **Report:** append a section per task to
  `.superpowers/sdd/2026-09-03-gizmo-snapping/progress.md`: what was
  built, the exact commands run with their last output line, and any
  deviation with its reason.

---

### Task 1: `gizmoSnap.py` — the Qt-free snap maths

**Files:**
- Create: `plugin/rigExecUsdview/gizmoSnap.py`
- Create: `tests/python/test_gizmo_snap.py`
- Modify: `plugin/rigExecUsdview/gizmoScreen.py` — refactor
  `_PointSegmentDistance` (around line 370) to return `(distance, t)`
  and update its callers; add a projection variant that also returns the
  clip-space `w` (the existing `ProjectPoint` throws it away, and the
  edge parameter needs it). Keep `ProjectPoint`'s own signature and
  behaviour unchanged so nothing else breaks. Add a `worldOrigin=None`
  keyword to `Handle.__init__` after `worldCenter`
  (`gizmoScreen.py:111-117`) storing `self.worldOrigin = worldOrigin if
  worldOrigin is not None else worldCenter` so no existing caller
  changes behaviour, and add `extra.setdefault("worldOrigin", origin)`
  beside the existing `extra.setdefault("worldCenter", origin)` in
  `BuildHandles._Make` (`gizmoScreen.py:285`). Extend the `Handle`
  docstring: `worldOrigin` is the point `GizmoMatrix()` places, for
  EVERY handle kind; code needing the pivot must read it, not
  `worldCenter` (spec 3, 4.2).
- Modify: `tests/python/test_gizmo_screen.py` — assert
  `handle.worldOrigin == gizmoMatrix.ExtractTranslation()` for all
  handle kinds and that a plane handle's `worldCenter` still differs
  from it.
- Modify: `plugin/rigExecUsdview/gizmoSettings.py` — add ONLY the five
  `SNAP_*` token constants beside the `ORIENT_*` block, so `gizmoSnap`
  can re-export them and there is one spelling. `_SNAP_LABELS`,
  `_SNAP_CHOICES`, the `snapMode` field and `gridSize` are Task 2's; do
  not add a `_FIELDS` entry here — without its `MayaDefaults` default it
  is a `KeyError` in every `ToolSettings` (spec 4.5).
- Modify: `CMakeLists.txt` — register `testGizmoSnap` beside
  `testGizmoDrag` in the `add_test` block and in the
  `foreach(_python_test …)` list.
- Modify: `bin/run_python_tests.sh` — add `test_gizmo_snap` to the
  default `TESTS` list (no schema argument).

**Interfaces (produced):**

```python
# Tokens re-exported from gizmoSettings so there is one spelling.
SNAP_OFF, SNAP_GRID, SNAP_POINT, SNAP_EDGE, SNAP_SURFACE

SNAP_PIXELS = 12.0       # logical px; scaled by the device ratio at the
                         # call site, the way gizmoUI._HitTest does
PICK_MOVE_PIXELS = 3.0   # PHYSICAL px, unlike SNAP_PIXELS above:
                         # it is compared against DragState.press /
                         # .current, which gizmoUI._Position already
                         # scaled by the device ratio
                         # (gizmoUI.py:1636-1652), and gizmoDrag is
                         # Qt-free so it cannot scale one itself.

class SnapCandidate(object):
    point     # Gf.Vec3d, world: where the pivot should land
    normal    # Gf.Vec3d, world; zero when the source has none
    kind      # one of the SNAP_* tokens
    primPath  # Sdf.Path or None
    index     # int, vertex / CV index, -1 when not applicable
    screen    # (x, y) PHYSICAL pixels, for the overlay marker

# --- the rule every mode shares -----------------------------------------
def ConstrainToHandle(handle, pivot0, worldPoint, ctrl=False) -> Gf.Vec3d
#   axis (no ctrl): pivot0 + a * Gf.Dot(W - pivot0, a)
#   plane, or axis with ctrl: W - n * Gf.Dot(W - pivot0, n)
#   centre / anything else: W unchanged
#   (spec 4.2; Ctrl+axis is a PLANE, matching TranslateDelta's branch)

# --- grid ----------------------------------------------------------------
def GridPoint(handle, pivot0, unsnapped, gridSize, ctrl=False) -> Gf.Vec3d
#   Rounds only the components the handle may change (spec 4.3):
#   axis -> P + a * (SnapValue(dot(P,a), g) - dot(P,a));
#   plane / ctrl+axis -> round the two in-plane world components;
#   centre -> round all three. Halves away from zero. When the
#   direction the handle acts on is not world-aligned it falls back to
#   the pivot0-relative form of spec 4.3 -- quantise the distance
#   travelled -- on the same directions.
def DirectionIsWorldAligned(direction, tol=1e-6) -> int | None
#   The index of the world axis `direction` lies along (sign ignored),
#   or None. GridPoint calls it on handle.worldAxis for an axis handle
#   and on the PLANE NORMAL -- handle.worldNormal for a plane handle
#   (worldAxis is None there, gizmoScreen.py:322), handle.worldAxis for
#   Ctrl+axis (gizmoDrag.py:186-188). None means "no grid coordinate in
#   this frame": the caller degrades to quantising the distance
#   travelled from pivot0 in the directions the handle may move
#   (spec 4.3), and says so in the status line -- gizmoDrag calls it
#   again itself to word that clause.

# --- screen-space ranking ------------------------------------------------
def NearestPoint(points, cursor, viewProjection, viewport, radius)
        -> (index, Gf.Vec3d, (x, y)) | None
#   Projects each world point, drops those behind the eye, returns the
#   nearest within `radius` PHYSICAL pixels.
def SegmentScreenParameter(a2d, b2d, cursor) -> (distance, t)
#   t is the clamped screen-space foot on the projected segment.
def WorldParameterFromScreen(t, wa, wb) -> float
#   The perspective-correct inverse: s = t*wa / ((1-t)*wb + t*wa).
#   wa, wb are the endpoints' clip-space w. Returns t when either w is
#   not positive (the caller must have dropped such segments).
def NearestSegment(segments, cursor, viewProjection, viewport, radius)
        -> (segmentIndex, Gf.Vec3d, (x, y)) | None
#   `segments` are (worldA, worldB) pairs. Drops any endpoint with
#   w <= 0 (spec 3: such a segment has no valid screen segment; do not
#   try to clip it). Returns the world point at the recovered parameter.

# --- candidate extraction (pure Usd, no Qt) ------------------------------
def MeshEdges(faceVertexCounts, faceVertexIndices) -> [(i, j)]
#   Unique undirected edges, each face treated as a closed loop.
def CurveSegments(vertexCounts, pointCount) -> [(i, j)]
#   Consecutive CVs within each curve, never across curve boundaries.
```

**Notes for the implementer**
- `gizmoSnap.py` may import `Gf` and `Sdf`; it must not import Qt and
  must not import `gizmoUI`.
- Reuse `gizmoScreen.ViewProjection` / `ProjectPoint` rather than writing
  a second projector; the spec's section 3 records that it already
  agrees with the pick frustum to 1.5 px.
- Rounding halves away from zero is `gizmoScreen.SnapAbsolute` /
  `_RoundToStep`'s existing rule (`gizmoScreen.py:650-685`) — call it
  rather than re-implementing.

- [ ] **Step 1: Write the failing test** `tests/python/test_gizmo_snap.py`,
  bootstrapping like `test_gizmo_screen.py` and reusing its synthetic
  cameras. Groups, matching spec section 6 items 2, 3, 4 and 6:
  `constrain` (per handle kind, including Ctrl+axis as a plane),
  `grid` (world landing, halves away from zero, axis-only component,
  non-world-aligned frame degrading to the relative form, per
  `DirectionIsWorldAligned` returning None),
  `ranking` (nearest in pixels; a case where world-nearest and
  screen-nearest differ; behind-the-eye dropped; outside-radius is
  `None`), `edge` (the round-trip assertion: foreshortened segment,
  recover the world point, reproject, land within 1e-6 px of the screen
  foot; plus a `w <= 0` segment dropped), and `topology` (`MeshEdges`
  dedupes shared edges; `CurveSegments` never joins two curves).
  Run it and watch it fail on the missing module.
- [ ] **Step 2: Implement** `gizmoSnap.py` and the two `gizmoScreen`
  changes. Re-run until green, then run the whole
  `bin/run_python_tests.sh` — every existing suite must still pass,
  especially `test_gizmo_screen` after the `_PointSegmentDistance`
  refactor.
- [ ] **Step 3: Register** the test in `CMakeLists.txt` and
  `bin/run_python_tests.sh`; re-run both.
- [ ] **Step 4: Report.**

---

### Task 2: settings, drag wiring, and the `X` redefinition

**Files:**
- Modify: `plugin/rigExecUsdview/gizmoSettings.py`
- Modify: `plugin/rigExecUsdview/gizmoDrag.py`
- Modify: `tests/python/test_gizmo_settings.py`
- Modify: `tests/python/test_gizmo_drag.py`
- Modify: `docs/viewport-gizmos.md` (the `J` / `X` paragraph, about
  lines 149-172)
- Modify: `docs/superpowers/specs/2026-09-01-viewport-gizmo-toolbar-design.md`
  (the two bullets that state `X` snaps the channel result: section 8.2
  and the Tool Settings section)

**gizmoSettings** (spec 4.5): beside the five `SNAP_*` tokens Task 1
added, `_SNAP_LABELS` + `SnapLabel()`, `_SNAP_CHOICES` +
`SnapChoices(tool)` (Move all five; Rotate `(SNAP_OFF, SNAP_GRID)`;
Scale and Select absent from `_SNAP_CHOICES` so `SnapChoices` returns
`()`, exactly as `_ORIENT_CHOICES` omits `TOOL_SELECT`) mirroring the
orientation shapes. Add exactly one entry to `_FIELDS`, `"snapMode"`,
and `"snapMode": SNAP_OFF` to `MayaDefaults` **in the same edit** — a
field without a default is a `KeyError` the first time a `ToolSettings`
is constructed. That fails `tests/python/test_gizmo_settings.py` and
`tests/python/test_gizmo_drag.py` at run time (both call `MayaDefaults`
from inside test bodies, not at import) and `tests/testUsdviewGizmo.py`
at plugin load, where `gizmoUI.py:930` constructs `GizmoSettings()`.
`test_gizmo_math.py` and `test_gizmo_screen.py` never import
`gizmoSettings` and are unaffected.
Add `gridSize` to `GizmoSettings` beside `manipulatorSize`,
clamped in `__setattr__` the same way (default 1.0, floor 1e-4), and
deliberately **not** touched by `Reset(tool)`.

**gizmoDrag** (spec 4.2-4.4):
- `DragState` gains `snapResolver = None` (a callable
  `(x, y, mode) -> SnapCandidate | None`, injected by `gizmoUI` once at
  `_BeginDrag`; the mode is passed on every call, never bound at the
  press), `snap = None` (the live candidate), `snapKind = None` (the
  mode `snap` was resolved for), `lastPickPoint = None` (the throttle),
  `rigWritten = frozenset()` (prims whose points a mover writes, which
  Point and Edge reject) and `rigWrittenByTarget = frozenset()` (the
  subset written by a mover under the dragged target's `RigRootPath()`,
  which Surface must reject too) — spec 4.4 step 3a.
- `ApplyDrag` gains keyword-only `snapMode=None` and `gridSize=1.0`,
  **after** the existing `holdSnap` / `holdGrid` / `ctrl` keywords,
  which keep their names. `snapMode` is the ALREADY-RESOLVED active mode
  supplied by the caller (the holds live in `gizmoUI`, so only it can
  resolve them); `None` means "resolve from `settings.snapMode` alone",
  which is what the Qt-free tests use. `gridSize` must be a keyword
  because it lives on `GizmoSettings` while `settings` here is a
  `ToolSettings`.
- A new module-level
  `ActiveSnapMode(settings, tool, holdGrid, holdPoint, holdEdge)`
  returns a 2-tuple `(mode, reason)` — always a tuple. `reason` is `""`
  when the mode is what was asked for and a short status clause when the
  request was downgraded. It imports `gizmoSettings` (which imports only
  `gizmoScreen`, so no cycle). **`gizmoUI` is its only production
  caller** — it needs the mode with no drag in flight, for the
  armed-mode hover marker and the status clause (spec 4.5) — and passes
  the result down as `ApplyDrag(..., snapMode=mode)`. `ApplyDrag` calls
  `ActiveSnapMode(settings, state.tool, holdGrid, False, False)` itself
  only when `snapMode is None`. Precedence, highest first: `holdPoint` →
  `SNAP_POINT`, `holdEdge` → `SNAP_EDGE`, `holdGrid` → `SNAP_GRID`, then
  the sticky `settings.snapMode` (spec 1.7). Press order is irrelevant;
  only the boolean flags are read. If the winning mode is not in
  `gizmoSettings.SnapChoices(tool)`, return `(SNAP_OFF, reason)` naming
  the mode and the tool — do **not** fall through to a lower hold, or a
  Rotate drag with `V`+`X` held would silently grid-snap while the
  status line claims Point is unavailable.
- `_Translate` gains the snap path: resolve the candidate (grid needs no
  resolver and writes its own `SNAP_GRID` candidate into `state.snap`;
  the other three call `state.snapResolver(x, y, mode)` with the mode in
  force for THIS event, subject to the `PICK_MOVE_PIXELS` throttle,
  reusing `state.snap` in between). The throttle is keyed on cursor
  travel AND on the mode: `state.snap` may only be reused while
  `mode == state.snapKind`. If the mode differs, or `state.lastPickPoint
  is None`, clear `state.snap` and pick immediately whatever the travel.
  Set `state.snapKind = mode` and `state.lastPickPoint = current` after
  every resolve, the grid one included, because `_DragKey` /
  `_ReleaseHold` re-run the drag at the same cursor point
  (`gizmoUI.py:1855-1903`). Then
  `constrained = ConstrainToHandle(state.handle, pivot0, W, ctrl)` and
  `target.ApplyTranslate(constrained - pivot0)` with **no** `snapStep`,
  where `pivot0` is `Gf.Vec3d(state.handle.worldOrigin)` — never
  `worldCenter`, which is offset on a planar handle (spec 3).
  No candidate means **no write at all** — the object does not move
  (spec 1.4).
- `TranslateSnap` keeps its name and its relative-step behaviour but
  **stops returning `absolute=True` for `holdGrid`**; grid now goes
  through the world path. Update its docstring to say what changed and
  why.
- `_Rotate`: `SNAP_GRID` for Rotate is defined **only** for a gimbal
  ring drag (`state.gimbal and handle.kind == "ring"`). Do **not** add a
  keyword to `ApplyRotateChannel` — `gizmoMath.py` is not a file this
  task may touch, and the plan promises no new `Target` API. Instead
  `DragState.__init__` sets `self.rotationBase = None`, then
  `_state = getattr(target, "RotationState", lambda: None)()` and, if
  not `None`, `self.rotationBase = list(_state[1])`. The press is the
  only correct moment: `gizmoUI._BeginDrag` builds the `DragState` after
  `target.BeginDrag()` and before any write (`gizmoUI.py:1928-1936`),
  while `RotationState()` reads LIVE from the stage
  (`gizmoMath.py:850-855, 1009-1011, 1401-1406`) and mid-drag would
  return what the drag has already written. Use `getattr`: this file's
  `FakeTarget` has none today and `_State` builds a `DragState` for the
  translate and scale tests too. Then, on the gimbal-ring branch with
  `SNAP_GRID` active and `rotationBase` not `None` — `step` is
  `settings.stepSize`, since Rotate's grid is degrees and ignores
  `gridSize` (spec 1.8):
  ```
  base = state.rotationBase[handle.axisIndex]
  landed = gizmoScreen.SnapAbsolute(base + angle, step)
  state.angle = landed - base
  state.target.ApplyRotateChannel(handle.axisIndex, state.angle,
                                  snapStep=None)
  ```
  Set `state.angle` from the absolute result as shown rather than
  through `_DisplayAngle`, or the pie wedge and the `deg` readout
  disagree with what was written. On every other rotate handle
  `SNAP_GRID` is inert and `_Rotate` records the reason `Grid needs a
  Gimbal ring` on the `DragState` for the status clause —
  `ActiveSnapMode` cannot make this call, it sees neither the handle nor
  `state.gimbal`. Add `RotationState` to `FakeTarget` returning
  `("xyz", [0.0, 0.0, 0.0])` (the base class has it, so the double
  should) and a Qt-free test for both branches.

**Tests to rewrite, not delete** in `test_gizmo_drag.py` — each pins the
old `X` and fails or goes vacuous otherwise:
- `TestTranslateSnapping`'s `"X is the absolute grid"` and `"X wins when
  both are held"` assertions: `holdGrid` no longer sends any keyword, so
  both become `target.LastKwargs() == {"snapStep": None, "snapAbsolute":
  False}` plus a check that the returned delta is the grid-snapped one.
- the `raw == held == grid == both` invariant, `"the world delta is
  never touched by the snapping"`: with this test's own handle and
  camera the unsnapped X delta is 0.28813 and `gridSize` defaults to
  1.0, so the grid path returns (0, 0, 0). Rewrite as
  `_Check(raw == held, ...)` plus `_Check(grid == both and grid != raw,
  "the grid path snaps the world delta; J and the option do not")`. Note
  `both` equals `grid`, not `raw`: `X` still outranks `J`.
- the zero-step case, `"a zero step is no step, not a division by
  zero"`: `stepSize` no longer drives grid, so drop `holdGrid=True` from
  that call and keep it as the `TranslateSnap` guard it was, and **add**
  a `gridSize` case beside it — `ApplyDrag(state, current, settings,
  holdGrid=True, gridSize=1e-4)` still writes and returns a delta that
  is a multiple of 1e-4, proving grid reads `gridSize` and is unaffected
  by `stepSize = 0.0`.
- `TestTranslateSnapping`'s docstring and the module docstring, which
  both state the world delta is handed over untouched — no longer true
  on the grid path.
- `TestTranslateSnapChoice`: all five lines pin `TranslateSnap` by tuple
  equality. `holdGrid` no longer influences `TranslateSnap` at all — the
  grid goes through `gizmoSnap.GridPoint` — so it must now return
  `TranslateSnap(settings, False, True) == (None, False)` and
  `TranslateSnap(settings, True, True) == (2.0, False)`; the other three
  are unchanged.

Rewrite each for the new contract and **add** the spec's item 5
regression: with a parent rotated 45° about Y, the world position lands
on the grid while the channel values do not. Add `TestActiveSnapMode`
pinning the full precedence table (spec item 1) the way
`TestTranslateSnapChoice` pins `X` over `J`, and `TestSnapThrottle`
implementing spec item 7a (cursor coordinates are PHYSICAL px and this
file's `RATIO` is 1.0, so 1 px is 1 unit).

Extend `test_gizmo_settings.py` with `snapMode` in `TestToolDefaults`'
field list, and a `TestSnapChoices` mirroring `TestOrientationChoices`
**with a narrower loop**: assert `SnapChoices(TOOL_TRANSLATE) ==
(SNAP_OFF, SNAP_GRID, SNAP_POINT, SNAP_EDGE, SNAP_SURFACE)`,
`SnapChoices(TOOL_ROTATE) == (SNAP_OFF, SNAP_GRID)`,
`SnapChoices(TOOL_SCALE) == () and SnapChoices(TOOL_SELECT) == ()`,
`MayaDefaults(tool).snapMode == SNAP_OFF` for all four, and run the
"default is among its own tool's choices" check over
`(TOOL_TRANSLATE, TOOL_ROTATE)` only — the two empty tuples would fail
it by construction, the same reason the existing orientation loop
excludes `TOOL_SELECT`.

**Docs**: rewrite the `J` / `X` paragraph of `docs/viewport-gizmos.md` and
the two gizmo-spec bullets to state the new meaning, and note that
`Target`'s `snapAbsolute` remains but is no longer bound by the UI.

- [ ] **Step 1: Implement** settings, then drag, then the docs.
- [ ] **Step 2: Rewrite and extend the tests**; run
  `bin/run_python_tests.sh` (all) until green.
- [ ] **Step 3: Report**, listing exactly which existing assertions
  changed meaning and why.

---

### Task 3: `gizmoUI.py` — pick, holds, controls, marker

**Files:**
- Modify: `plugin/rigExecUsdview/gizmoUI.py` only.

Work in this order:

1. **The pick.** A `_SnapPick(x, y)` helper on `GizmoController`
   wrapping `view.computePickFrustum` and
   `_getRenderer().TestIntersection` with `showGuides = False` (spec 3 —
   `view.pick` cannot be used, the plugin forces `displayGuide = True`),
   mirroring `curvenetUI.SurfacePicker.Pick` (`curvenetUI.py:530-553`)
   including its `inBounds` early out and its
   `Tf.Warn`-and-return-`None` error handling. Duplicate those ~25 lines
   with a comment naming the original; do **not** import `curvenetUI`.
   The bypass needs `UsdImagingGL` from `pxr` and `OpenGL.GL as GL`,
   imported the way `stageView.py` does, and a **fresh**
   `UsdImagingGL.RenderParams()` per pick built from the view settings —
   never a mutated `view._renderParams`, which `pick()` reuses for the
   artist's own picks (spec 3).
2. **The resolver.** `_ResolveSnap(x, y, mode)` turns a pick into a
   `gizmoSnap.SnapCandidate`: reject a hit whose prim path is the
   dragged prim, an ancestor or a descendant of it; **only when
   `target.RigRootPath()` is not `None`**, anything at or under that
   path; and anything in the `rigWritten` set (spec 4.4 step 3a), built
   once in `_BeginDrag` as
   `{t.GetPrimPath() for p in Usd.PrimRange(stage.GetPseudoRoot())
   for r in (p.GetRelationship("rigExec:moves"),
   p.GetRelationship("rigExec:weightTarget")) if r
   for t in r.GetTargets()}`, and in the same traversal the subset whose
   writing prim `p` is at or under `target.RigRootPath()` (empty when
   that is `None`) as `rigWrittenByTarget`; both kept on the
   `DragState`. Point and Edge reject a hit in `rigWritten`; Surface
   takes its point from the renderer and rejects one only when it is in
   `rigWrittenByTarget`, the case where the drag itself re-poses the
   geometry under the ray (spec 4.4 step 3a).
   `Target.RigRootPath()` is `None` for every xform target
   (`gizmoMath.py:825-836`; only `_RigTarget` overrides it at
   `:964-966`) and `Sdf.Path.HasPrefix(None)` raises
   `Boost.Python.ArgumentError`, so guard it the way
   `gizmoMath.NoticeAffectsTarget` already does (`gizmoMath.py:342`:
   `if rigRootPath is not None and prim.HasPrefix(rigRootPath)`). A
   plain xform target has no rig root and is excluded by prim path,
   ancestry and descendancy alone — this is the path Task 4's
   `/Shot/SnapProbe` Xform takes; then per mode take the hit point
   (surface), the nearest point (point) or the nearest segment point
   (edge), carrying the prim's points to world with
   `UsdGeom.XformCache(time).GetLocalToWorldTransform(prim)` — add
   `UsdGeom` to `gizmoUI.py`'s `from pxr import Gf, Tf, Usd` (line 67);
   it is not imported today. Install it on the `DragState` at
   `_BeginDrag` as `drag.snapResolver = self._ResolveSnap` — the bound
   method itself, never a lambda that captures the mode: the holds
   change mid-drag (`_DragKey` / `_ReleaseHold`,
   `gizmoUI.py:1855-1898`), so a captured mode would make a mid-drag `V`
   or `C` inert and fail spec item 13.
3. **Holds.** `X` already exists. Add `C` (edge) and `V` (point) to
   `ViewportHotkeyFilter`. usdview binds both — `C` = *Auto Compute
   Clipping Planes* (`mainWindowUI.py:1598`, connected
   `appController.py:833`), `V` = *Show USD Validation*
   (`mainWindowUI.py:1481`, connected `appController.py:824`) — so they
   follow the `J` / `Escape` sharing rule, NOT the tool-key rule. The
   tool-key gate (`gizmoUI.py:1789-1812`) tests only visibility, window,
   typing focus and cursor-over-view; it carries **no tool condition**,
   so gating C/V there would swallow both keys in Select, Rotate and
   Scale too, permanently breaking `docs/viewport-gizmos.md:169-176` and
   toolbar-spec section 8.5. Instead:
   - Add `Key_C` and `Key_V` to `_DRAG_KEYS` (`gizmoUI.py:1727`) and
     handle them in `_DragKey`: while a drag is live they are claimed
     unconditionally, exactly as `J` and `X` are, so usdview's actions
     never fire mid-drag. A tool that does not offer the mode still
     claims the key and reports it inert in the status line (spec 1.2
     and 5).
   - Outside a drag, handle them in `_ToolKey` and return **False** —
     leaving the key to usdview — unless `self._target is not None` and
     the mode is in `gizmoSettings.SnapChoices(self._tool)`, i.e. only
     the Move tool arms them. Only then set the hold flag, refresh the
     hover candidate/marker (spec 4.5) and return True.
   - Release through the existing `_ReleaseHold` / `_ClearHolds` path
     (which already no-ops when the flag was never set) or the flag
     survives the drag.

   The flags are `self._holdEdge` (`C`) and `self._holdPoint` (`V`),
   declared beside `_holdSnap` / `_holdGrid` (`gizmoUI.py:942-943`),
   released in `_ReleaseHold` (`:1887-1895`) and cleared in
   `_ClearHolds` (`:1968-1970`). `_UpdateDrag` (`gizmoUI.py:1943-1958`)
   is the route into the maths and must be edited in this step:

   ```python
   settings = self.settings.For(drag.tool)
   mode, reason = gizmoDrag.ActiveSnapMode(
       settings, drag.tool, self._holdGrid,
       self._holdPoint, self._holdEdge)
   self._snapMode, self._snapReason = mode, reason
   gizmoDrag.ApplyDrag(
       drag, point, settings,
       holdSnap=self._holdSnap, holdGrid=self._holdGrid,
       ctrl=drag.ctrl, snapMode=mode,
       gridSize=self.settings.gridSize)
   ```

   Without this the `C`/`V` holds, the `Snap:` dropdown and the
   `Grid Size` spin box are all inert, and the `gridSize` half fails
   silently — every grid snap would quietly use the 1.0 default.
4. **Controls.** One `Snap:` `QToolButton` with an `InstantPopup` menu of
   mutually exclusive checkable actions, after `_BuildWrite()`'s
   separator, its text showing the active mode; mirrored in
   `ViewportToolbar.Sync()`. In `ToolSettingsPanel.Rebuild()`, a
   `Snap To` combo built from `gizmoSettings.SnapChoices(tool)` exactly
   like the orientation combo **but only when that tuple is non-empty**
   (for Scale it is empty; skip the row and the spin box entirely,
   leaving Scale's `snapMode` at `SNAP_OFF`), a `Grid Size` spin box
   (range 1e-4 to 1e5, 4 decimals) whenever `SNAP_GRID` is offered,
   matching `Sync()` lines guarded by the same non-empty test, and for
   Rotate a wrapped label saying point/edge/surface are Move-only and
   that Grid needs a Gimbal ring. Slots guarded by `_updating`, with
   matching lines in `Sync()`.
5. **Marker.** A `SnapCandidate()` accessor returning the live candidate
   or `None`, painted **last** in `GizmoOverlay.paintEvent` — never
   through `Handles()`. A diamond plus crosshair in a new `COLOR_SNAP`,
   with the kind-specific extra of spec 4.5, and a faint leader from the
   handle centre. Refresh the candidate on hover too when a mode is
   armed, using the same mode-keyed throttle as the drag path, so
   switching the `Snap:` dropdown without moving the cursor re-picks
   instead of redrawing the stale marker, gating any `toolbar.Sync()` on
   the candidate actually having changed.
6. **Status.** Build the snap clause of spec 4.5 once in a helper,
   `_SnapClause()`, returning `""` when no snap is active or armed, and
   append it on **both** returning branches of `gizmoUI.Status()`: the
   live-rotate early return at `gizmoUI.py:1393-1395`
   (`return "%s %s  %.1f deg%s" % (verb, target.label, self._drag.angle,
   self._SnapClause())`) and the general branch, where it goes into
   `text` immediately after the `[Pivot|Pose / Default|Animation]`
   bracket, before the skipped-children and warning suffixes. Rotate's
   grid mode and the "Move only" message are only visible while a rotate
   drag is live, so the rotate branch is not optional. The clause must
   keep `"deg"` in the rotate string (`tests/testUsdviewGizmo.py:521`)
   and must not disturb `"outranked"` / `"unavailable"`.
   `GizmoController` also gains a `SnapMode()` accessor returning the
   mode actually in force (`gizmoDrag.ActiveSnapMode` evaluated with the
   live holds) — the toolbar button text, the status clause and the
   end-to-end hold assertions (spec items 12 and 13) all need it.

- [ ] **Step 1: Implement** in that order.
- [ ] **Step 2: Smoke** with `bin/run_testusdview_gizmo.sh` (must still
  print `RIGEXEC_GIZMO_OK`) and `bin/run_testusdview_viewcube.sh`, then
  `RIGEXEC_GIZMO_SHOT=.superpowers/sdd/2026-09-03-gizmo-snapping/snap.png
  bin/run_testusdview_gizmo.sh` and look at the grab: the toolbar shows
  the `Snap:` button and nothing has been pushed into the overflow
  chevron.
- [ ] **Step 3: Report** (mention the grab path).

---

### Task 4: end-to-end test

**Files:**
- Modify: `tests/testUsdviewGizmo.py` — add a snapping section
  implementing spec section 6 items 8-15, on `examples/ArmShotAnim.usda`
  with a scratch `/Shot/SnapProbe` Xform as the moved object, defined
  and removed the way the file already does for `GizmoTestChild`
  (`UsdGeom.Xform.Define` / `stage.RemovePrim`), placed onto the
  geometry with
  `UsdGeom.XformCommonAPI(probe).SetTranslate(Gf.Vec3d(4, 10, 2))`, and
  a scratch undeformed `/Shot/SnapTarget` Mesh plus invisible
  `/Shot/SnapDecoy` Points authored into the SESSION layer (spec 3:
  every mesh and curve in the asset is evaluator-posed, so no landing
  assertion may use ArmBody or RibbonGuides). Hide the RigExec overlay
  for the duration and restore it, and save and restore the camera:

  ```python
  saved = (camera.rotTheta, camera.rotPhi, camera.dist)
  d.Select('/Shot/SnapTarget')
  appController._frameSelection()
  d.Pump()
  d.Select('/Shot/SnapProbe')      # d.Select CLEARS first: the gizmo
  controller.SetTool(gizmoUI.TOOL_TRANSLATE)   # target must be the probe
  d.Pump()
  ...                              # items 8-15
  camera.rotTheta, camera.rotPhi, camera.dist = saved
  for path in ('/Shot/SnapDecoy', '/Shot/SnapTarget', '/Shot/SnapProbe'):
      stage.RemovePrim(Sdf.Path(path))
  d.Pump()
  ```

  Do not `AddPrimToSelection` a second prim on top of the probe: a
  two-prim selection changes what the controller targets. Restoring the
  camera matters because the later sections and the `RIGEXEC_GIZMO_SHOT`
  grab were written for the original one.
- The runner `bin/run_testusdview_gizmo.sh` needs no change.

- [ ] **Step 1: Write the section.** `_Driver` gains
  `KeyDown(key, modifiers=None)` and `KeyUp(key, modifiers=None)`
  wrapping `QTest.keyPress` / `QTest.keyRelease` on `self.view`, with
  the same `self.view.setFocus()` before and `self.Pump()` after, and
  the same PySide6/PySide2 `QtTest` import `Key()` uses. Every hold
  assertion (spec items 12 and 13) uses those, never `Key()`: `Key()` is
  `QTest.keyClick` (`tests/testUsdviewGizmo.py:132`), a press
  immediately followed by a release, so the hold is armed and cleared
  before the next mouse move — probed, `controller._holdGrid` is `False`
  after a mid-drag `keyClick(Key_X)` and `True` across a move after
  `keyPress(Key_X)`. `_Driver.Drag()` is atomic, so a hold test must
  hand-roll `Press` / `Move` / `KeyDown` / `Move` / `KeyUp` / `Release`
  rather than call it. Then run `bin/run_testusdview_gizmo.sh` until it
  prints `RIGEXEC_GIZMO_OK` again with the new assertions in place. Then
  run `bin/run_python_tests.sh` and `bin/run_testusdview_viewcube.sh`.
- [ ] **Step 2: Report.**

---

### Task 5: documentation

**Files:**
- Modify: `docs/viewport-gizmos.md` — a `## Snapping` section: the four
  Move modes and what each lands on, the `X` / `C` / `V` holds and the
  `Snap:` dropdown (outside a drag `C` and `V` belong to the gizmo only
  while Move is active with a target, and otherwise stay usdview's),
  their precedence, the pivot-is-what-lands rule, per-handle constraint,
  nothing-under-the-cursor does nothing, rig-driven geometry being no
  target at all, Rotate's absolute degree grid on a Gimbal ring,
  `Grid Size`, what is out of scope, and how to test it. Fold in the
  `J`/`X` rewrite from Task 2 rather than repeating it.
- Modify: `README.md` — no new bullet (the gizmo doc is already linked);
  change nothing unless the gizmo bullet's wording is now wrong.

- [ ] **Step 1: Write it.**
- [ ] **Step 2: Report**, ending with every file created and modified
  across all five tasks.
