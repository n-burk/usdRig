# Maya-style grid / component / surface snapping for the viewport gizmo

Date: 2026-09-03. Status: approved for implementation under the
assumptions in section 1 (executed autonomously — Opus planned, the Meta
Muse 1.3 contributor model implemented, Opus validated — so the
assumptions are the interpretation a careful colleague would make and
are the first thing to revisit if the result feels wrong).

## 0. Request

"Implement Autodesk Maya style grid/component/surface snapping for
move/rotate tools."

## 1. Assumptions and scope

1. **What ships in v1.** For the **Move** tool, four snap modes, each of
   which places the manipulator pivot on a world-space target:
   - **Grid** — the world grid, spacing `gridSize`. Maya's `X`.
   - **Point** — the nearest vertex or curve CV of the prim under the
     cursor. Maya's `V`.
   - **Edge** — the nearest point *on* the nearest edge segment of that
     prim (mesh edges, and consecutive CVs of a curve). Maya's `C`,
     widened from curves to mesh edges because that is what "component
     snapping" means to a modeller.
   - **Surface** — the point on the prim under the cursor where the
     cursor ray hits it.
   For the **Rotate** tool, two: the existing relative degree step
   (Step Snap / held `J`), unchanged, and **Grid**, redefined for rotate
   as an *absolute* degree grid, so the dragged Euler channel lands on a
   multiple of the step rather than the delta doing so. Grid is offered
   only on a **Gimbal ring** drag — the one rotate route that writes a
   single channel — and is inert on a world-axis ring or the
   free-rotate ball, which reach the drawn rotation by moving all three
   channels (section 4.3).
2. **Rotate has no point/edge/surface snapping**, because Maya has none:
   `X`, `C` and `V` are Move-tool snaps there. Holding one while
   rotating is inert and the status line says
   `snap: Point is Move only` rather than silently ignoring the key.
   Inventing an "aim at the snapped point" rotate mode is explicitly out
   of scope: it has no Maya reference, no axis convention for a gimbal
   ring, and no natural undo label.
3. **The pivot is what lands on the target**, never the grabbed handle
   and never the cursor — Maya's rule. The drag's handle then constrains
   *where on the target* it may land (section 4.2).
4. **Nothing under the cursor means the object does not move.** Maya
   does not fall back to free dragging, and reproducing that (with a
   status message) is better than "improving" it. The grid is infinite,
   so this case cannot arise for Grid.
5. **The Scale tool gains nothing.** Its Step Snap already quantises the
   resulting scale channel, which is the only snap Maya offers it.
6. **Out of scope for v1**, each a follow-up, none blocking the request:
   Maya's Make Live (nominating one construction surface), snap to view
   planes, snap to projected centre, snap to network, Snap Align
   Objects, face-centre snapping, multi-prim snapping, and reorienting
   the object to a surface normal on a Move snap (Maya does not do this
   either).
7. **Snap modes are both momentary holds and a sticky setting**, as in
   Maya: `X` / `C` / `V` held during a drag, and a `Snap:` dropdown that
   stays on. A held key outranks the sticky mode. When more than one
   hold is down the more specific target wins: Point (`V`) > Edge (`C`)
   > Grid (`X`). A vertex fully determines where the pivot lands, an
   edge constrains it to a line and the grid only to a lattice, so the
   narrower target says everything the wider one would — the same
   reasoning that already makes `X` outrank `J`
   (`gizmoDrag.TranslateSnap`, `docs/viewport-gizmos.md:168`).
   Precedence depends only on which keys are down, never on the order
   they were pressed, so no press-order state is kept, and releasing the
   winner falls straight through to the next hold still down, then to
   the sticky `Snap:` mode. Surface has no Maya hold key, so it is
   reachable only from the dropdown; no key is invented for it.
8. **`gridSize` is a scene property, not a per-tool one.** usdview draws
   no grid at all, so there is nothing on screen to match and the world
   grid through the origin is the only unambiguous meaning. Rotate's
   "grid" is degrees, which `stepSize` already is, so Rotate's grid mode
   uses `stepSize` and ignores `gridSize`.

## 2. The behaviour change this makes to a shipped feature

Today a held `X` sets `snapAbsolute=True`, and `gizmoMath._SnapTranslation`
(`gizmoMath.py:604-619`) rounds the **channel** values `avars:tx/ty/tz`,
`rest:t*` or the xformOp. That equals Maya's grid snap only when the
channel frame is the world frame: no parent transform, no rest offset,
World axis orientation. Under a posed parent — the normal rig case — the
channels land on round numbers while the world position is arbitrary,
which is the opposite of what Maya does.

Measured on `ArmRig.usda` with `rest:ry=25`, `avars:rspin=11` and an
asset root scaled (2, 0.5, 3): asking for the world point
(−4.25, 11.75, 2.5) through `snapStep=1.0, snapAbsolute=True` missed it
by 0.245 world units; the delta form landed it to 2e−15.

**`X` is therefore redefined to snap the world pivot.** This is a
deliberate, announced break of documented and tested behaviour:
`tests/python/test_gizmo_drag.py:294-307` pins the old contract,
`docs/viewport-gizmos.md:164-172` states it as a feature, and the gizmo
design spec repeats it. All four are rewritten in the same change. An
artist who relied on "my `tx` values come out round" loses that; the
`snapAbsolute` parameter stays on `Target` (it is the correct name for
channel-absolute and remains tested) but nothing in the UI binds it any
more, and its docstring says so.

## 3. Facts the design relies on

All verified by probe or by reading, 2026-09-03.

- **No new `Target` API is needed.** `handle.worldCenter` is the gizmo
  origin for the axis, centre, ring, view and sphere kinds, but a PLANAR
  handle carries its SQUARE's centre instead: `gizmoScreen.py:316`
  computes `squareCenter = origin + (a + b) * (PLANE_OFFSET * length)`
  and `:324` passes it as `worldCenter=`, and the `Handle` docstring
  (`gizmoScreen.py:94-96`) says so. At the default 90 px manipulator
  that is ~0.35 world units, ~35 physical px, from the origin, and it
  grows with `manipulatorSize` and camera distance. `TranslateDelta` is
  right to use it (`gizmoDrag.py:186-195`) — the grabbed square must
  stay under the cursor, and the offset lies in the handle's own plane
  so it does not change `PlaneDelta`'s plane — but a pivot computation
  has no such licence. So `gizmoScreen.BuildHandles._Make` also stamps
  every handle with `worldOrigin=origin`, and throughout sections 4.2
  and 4.3 `pivot0` means `Gf.Vec3d(handle.worldOrigin)`, never
  `handle.worldCenter`. Then "put the pivot at world point W" is
  `ApplyTranslate(W − handle.worldOrigin)` with `snapStep=None`, which
  every existing `Target` subclass already implements exactly: each
  `ApplyTranslate` is affine in the world delta through
  `_Linear(frame).GetInverse()` and is recomputed from the `BeginDrag`
  base. Verified per subclass, landing error 2e−15 (`RigPoseTarget`),
  4.4e−16 (`RigPivotTarget`), 5.7e−15 (`XformPoseTarget`), 4.1e−7
  (`XformPivotTarget`, whose pivot channel is `Gf.Vec3f`, so float32 is
  the floor).
- **`StageView.pick(frustum)`** (`stageView.py:2224`) wraps
  `UsdImagingGL.Engine.TestIntersection`, returns hits carrying
  `hitPrimPath` / `hitPoint` / `hitNormal`, and honours the viewer's
  visibility, purpose and complexity settings. Measured about 1.3 ms per
  call offscreen, with a 5.7-18 ms Hydra sync on the first pick after a
  topology or visibility change. It hardcodes `stage.GetPseudoRoot()` as
  the pick root (`stageView.py:2270`), so a subtree cannot be excluded
  through it; exclusion is by filtering the returned hit's path.
  `curvenetUI.SurfacePicker` (`curvenetUI.py:476-553`) frames the call
  the way this feature needs — the pick frustum, the `inBounds` early
  out, the `Tf.Warn`-and-return-`None` degradation — and
  `curvenetUI._OnDrag` already calls it on every mouse-move of a knot
  drag (`curvenetUI.py:1096-1097`), so the per-move budget is proven in
  this repo.

  Invisible prims are excluded for free, but GUIDE prims are **not**:
  `pick` sets `showGuides = viewSettings.displayGuide`
  (`stageView.py:2250`) and `rigExecUsdview.py:583-588` forces
  `displayGuide = True` whenever a rig activates, so every RigExec
  control, joint and solver guide is pickable and is drawn in front of
  the model. Measured on `ArmShotAnim.usda` from the end-to-end test's
  camera: all 196 non-empty hits of a 40x30 viewport sweep are prims
  under `/Shot/HeroArm/Rig`. The joint and solver prims are not
  `UsdGeom.PointBased`, so section 4.4 steps 4-5 produce no candidate
  at all for them. The `showGuides = False` bypass below drops joints
  and solvers only: `RigExecControl` leaves `purpose` at the stock
  `UsdGeomImageable` `default` (schema.usda:121-196), so controls stay
  pickable and the resolver rejects them by prim type instead (section
  4.4 step 2a). The snap pick therefore **bypasses `StageView.pick`**:
  after
  `inBounds, frustum = view.computePickFrustum(x, y)`, call
  `view.makeCurrent()`, `GL.glDepthMask(GL.GL_TRUE)`, then
  `view._getRenderer().TestIntersection(pickParams,
  frustum.ComputeViewMatrix(), frustum.ComputeProjectionMatrix(),
  stage.GetPseudoRoot(), params)` with `pickParams =
  UsdImagingGL.Engine.PickParams()` / `resolveMode =
  "resolveNearestToCenter"` and a **fresh** `params =
  UsdImagingGL.RenderParams()` carrying `frame`, `complexity`,
  `drawMode`, `showProxy`, `showRender`, `enableSceneMaterials`,
  `enableSceneLights` from the view settings but `showGuides = False`.
  Build a fresh `RenderParams`; never mutate `view._renderParams`, which
  `pick()` reuses for the artist's own picks. Never toggle
  `viewSettings.displayGuide` around the pick either: it fires a
  visibility signal and a repaint. Measured 1.42 ms per pick on this
  route. If `_getRenderer()` is unavailable, degrade per section 5.
- **A mouse-move already costs** `ApplyDrag` → target write →
  `Refresh()` → `_RebuildHandles()` (which calls `resolveCamera()`) →
  repaint → `UpdateViewport()`, plus a rig re-evaluation. At 60 Hz the
  frame is 16.7 ms; snapping gets at most 4 ms of it.
- **Candidates come from the picked prim only.** Picking first and then
  searching that one prim's points is O(one prim) per move, inherits
  visibility and occlusion from the renderer, and avoids a stage
  traversal that would crawl on a production asset while looking fine on
  this repo's 4- and 5-point examples.
- **Screen distance, not world distance,** decides which candidate wins:
  any world tolerance small enough to separate two neighbouring vertices
  is a few pixels on screen, and any tolerance big enough to hit
  reliably swallows its neighbours. `curvenetUI.py:499-503` documents
  the same conclusion for knots. `gizmoScreen.ProjectPoint`
  (`gizmoScreen.py:144-151`) is the projector and already matches the
  pick frustum to within 1.5 px (`testUsdviewCurvenetMove.py:50-73`).
- **Edge snapping must be done in screen space with a
  perspective-correct inverse.** Probed on a foreshortened segment: the
  naive world-space lerp of the screen parameter is off by hundreds of
  pixels, and the 3D ray/segment closest approach fails outright,
  because an edge running away from the camera is nearly parallel to the
  pick rays through it. The correct form takes the screen-space foot
  `t` of the cursor on the projected segment and converts it to the
  world parameter with `s = t·wa / ((1−t)·wb + t·wa)`, where `wa`, `wb`
  are the clip-space `w` of the endpoints. `gizmoScreen._PointSegmentDistance`
  (`gizmoScreen.py:370-379`) already computes that `t` and throws it
  away. A segment with an endpoint at or behind the eye plane
  (`w <= 0`) has no valid screen segment and is dropped, not clipped.
- **Keys.** `X` is unclaimed by usdview. `C` is *Auto Compute Clipping
  Planes* (`mainWindowUI.py:1598`, connected at `appController.py:833`)
  and `V` is *Show USD Validation* (`mainWindowUI.py:1481`, connected at
  `appController.py:824`). Both are ordinary window shortcuts, so an
  application-level filter that accepts the `ShortcutOverride` suppresses
  them completely — the mechanism the gizmo already uses for `J`
  (`gizmoUI.py:41-46`). They are claimed unconditionally while a drag is
  live. Outside a drag they are claimed only while the Move tool is
  active with a target, because that is the only tool that offers
  point/edge snapping (section 4.5); with Select, Rotate or Scale
  active, `_ToolKey` must return False so usdview's own `C` and `V`
  still fire. The tool-key gate itself carries no tool condition
  (`gizmoUI.py:1789-1812`), so "gate like the tool keys" is not
  sufficient: the extra tool/target test is what keeps the toolbar
  spec's section 8.5 rule that a key usdview's main window binds is left
  to usdview.
- **`examples/ArmShotAnim.usda`, and what a test may and may not assert
  on it.** All these prims have identity local and world transforms
  (probed), but `/Shot/HeroArm/Geom/ArmBody.points` and
  `/Shot/HeroArm/Geom/RibbonGuides.points` are **driven by the rig**
  (`rigExec:moves`, `ArmRig.usda:307, 318, 325, 335, 345, 356, 372`):
  `RigExecResultsSceneIndex` overlays the evaluated array into Hydra, so
  `UsdGeom.PointBased.GetPointsAttr()` returns the REST array, which is
  neither what Hydra draws nor what the pick returns. Measured at frame
  1001: ArmBody's authored corner (8, 9.5, 0) is drawn at
  (8.014195, 9.601627, −0.075393), so a pick aimed at the authored
  corner misses the mesh; `RibbonGuides` is drawn only over x 2.683 to
  5.317, not 0 to 8. There is no way to read the evaluated array back
  from the plugin: `_rigexec` is not on usdview's PYTHONPATH
  (`bin/_env.sh:26`, `bin/launch.sh:75`) and `librigExecImaging` exports
  no points accessor. **Therefore no landing assertion may use ArmBody
  or RibbonGuides**; section 6 authors its own undeformed snap target.
  Usable as authored: `/Shot/HeroArm/Geom/RibbonDriver` and
  `/Shot/HeroArm/Targets/BicepFlexTarget`, both authored invisible.

## 4. Design

### 4.1 Where the code lives

| Module | Qt? | Gains |
|---|---|---|
| `gizmoSnap.py` (new) | no | `SnapCandidate`, the world-grid rule, `ConstrainToHandle`, the screen-space ranking, the perspective-correct edge parameter, candidate extraction from a picked prim's point arrays |
| `gizmoDrag.py` | no | chooses the active mode, calls `gizmoSnap`, turns a candidate into a plain world delta |
| `gizmoSettings.py` | no | one new `ToolSettings` field `snapMode`, `SnapChoices` / `SnapLabel`, and `gridSize` on `GizmoSettings` |
| `gizmoScreen.py` | no | `_PointSegmentDistance` refactored to return `(distance, t)`; `ProjectPoint` gains a variant that also returns clip `w`; `Handle` gains `worldOrigin`, the point `GizmoMatrix()` places, for every kind |
| `gizmoUI.py` | yes | the pick, per-move candidate resolution, the `C`/`V` holds, the toolbar dropdown, the panel rows, the overlay marker, the status clause |

`gizmoDrag` must stay Qt-free, so the pick is **injected**: `DragState`
carries a `snapResolver(x, y, mode) -> SnapCandidate | None` supplied by
`gizmoUI` at `_BeginDrag`. The mode is an *argument*, never captured at
the press: `_DragKey` / `_ReleaseHold` (`gizmoUI.py:1855-1898`) change
the holds while the drag is live and re-run them through `_ReapplyDrag`
(`gizmoUI.py:1900-1903`), so the mode in force is per-event data. Grid
snap needs no picking and so stays fully testable headlessly; the
picking half is stubbed in the Qt-free tests.

Do not import `curvenetUI` from `gizmoUI` — that pulls a whole Qt panel
into the gizmo's import graph. Duplicate the ~25 lines of the picker
with a comment pointing at the original.

### 4.2 The one rule every mode shares

Every mode produces a world point `W`. The handle then constrains it,
and the write is always the same:

```
constrained = ConstrainToHandle(handle, pivot0, W, ctrl)
target.ApplyTranslate(constrained - pivot0)     # snapStep=None
```

with `pivot0 = Gf.Vec3d(handle.worldOrigin)` — the gizmo origin, frozen
at the press; for a planar handle this is NOT `handle.worldCenter`
(section 3), and `ApplyTranslate(constrained − handle.worldCenter)`
would land the offset SQUARE on the target, leaving the pivot ~0.35
world units away — and

- **axis** handle, no Ctrl: `pivot0 + axis · dot(W − pivot0, axis)` —
  the object slides along the axis and stops level with the target.
- **axis** handle with Ctrl, or a **plane** handle:
  `W − normal · dot(W − pivot0, normal)`. Ctrl+axis is a plane
  (`gizmoDrag.py:186-195`), and the constraint must branch the same way
  or Ctrl+axis+snap silently does the wrong thing.
- **centre** handle: `W` unchanged.

One function serves grid, point, edge and surface, which is what makes
adding Make Live later cheap.

### 4.3 Grid

`W` is the unsnapped position `pivot0 + TranslateDelta(state)` with only
the coordinates the handle may change rounded to `gridSize`, the rest
kept.

The handle supplies the directions the grid may act on. For an **axis**
handle that is `handle.worldAxis`; for a **plane** handle it is the
plane with normal `handle.worldNormal` — a plane handle's `worldAxis` is
`None` (`gizmoScreen.py:322-324`); for **Ctrl+axis** it is the plane
whose normal is `handle.worldAxis`, matching `TranslateDelta`
(`gizmoDrag.py:186-188`). A direction is *world-aligned* when it is
within `1e-6` of ±X, ±Y or ±Z.

- axis handle, world-aligned axis `a`:
  `P + a·(SnapValue(dot(P,a), g) − dot(P,a))`.
- plane handle or Ctrl+axis, world-aligned NORMAL `n`: round the two
  world components perpendicular to `n`, keep the `n` component.
- centre: round all three world components.
- **Not world-aligned** (Object / Parent / Gimbal orientation, where
  `axes[i] = frame.TransformDir(_AXES[i])` is arbitrary,
  `gizmoScreen.py:275-281`): there is no grid coordinate in that frame,
  so do not invent one — quantise the distance travelled from `pivot0`
  in the directions the handle may move. Axis handle:
  `pivot0 + a·SnapValue(dot(P − pivot0, a), g)`. Plane handle or
  Ctrl+axis: `pivot0 + u·SnapValue(dot(P − pivot0, u), g) +
  v·SnapValue(dot(P − pivot0, v), g)`, with `(u, v)` a deterministic
  orthonormal basis of the plane: let `e` be the world axis with the
  smallest `|dot(e, n)|` (ties X before Y before Z),
  `u = normalize(e − n·dot(e, n))`, `v = Gf.Cross(n, u)`. The status
  line then says `grid: relative (frame not world-aligned)` instead of
  `grid: world`.

Rounding the projected point and then re-projecting leaves the
constraint line whenever the axis is not world-aligned; projecting a
rounded point leaves the grid. Only the form above does both, and it
degenerates to Maya's behaviour exactly when the axis is a world axis.
Halves round away from zero, the rule `gizmoScreen.SnapAbsolute`
(`gizmoScreen.py:673-685`) already implements and which
`gizmoMath._SnapValue` documents.

For **Rotate**, Grid means the dragged **gimbal** ring's own Euler
channel lands on a multiple of `stepSize`. It is offered only when
`state.gimbal and handle.kind == "ring"` — the one route that already
writes a single channel (`gizmoDrag.py:284-292`). On a world-axis ring
or the free-rotate ball, `ApplyRotate` reaches the drawn rotation by
moving all three Euler channels (probed: 20° about world Y under a 45°-Y
parent takes (7, 3, 11) to (11.054, 22.610, 11.912)), so there is no
single "resulting channel" to quantise: Grid is inert there and the
status reads `snap: Grid needs a Gimbal ring`. That is the DEFAULT case,
since `MayaDefaults` gives Rotate `ORIENT_OBJECT`.

This is **not** the existing `_SnapValue` path. Every `Apply*` applies
`snapStep` to the DELTA (`gizmoMath.py:1033, 1092, 1429`; probed,
`ApplyRotateChannel(1, 40.0, snapStep=15.0)` from base `ry=3` lands 48,
not 45) and none has an absolute mode. `Target`'s rotate keyword set
does **not** change; `DragState` captures the base once at the press and
`gizmoDrag` does the absolute arithmetic itself.

### 4.4 Point, Edge and Surface

All three start from one throttled pick of the prim under the cursor:

1. `Pick(x, y)` — the `showGuides = False` bypass of section 3, not
   `StageView.pick` — gives `(primPath, hitPoint, hitNormal)` or `None`.
2. Reject the hit when its path is the dragged prim, an ancestor or a
   descendant of it, or — for a rig target, i.e. only when
   `RigRootPath()` is not `None` (base `Target` returns `None`,
   `gizmoMath.py:825-836`; the `_RigTarget` override is `:964-966`) —
   anything under that path. Without this the pick renders the stage as
   the drag has already written it and the object glues itself to the
   cursor ray. Step 3a adds the prims a mover deforms, which are
   siblings of the rig rather than under it.
3. **Surface**: `W = hitPoint`. Done.
3a. **Point and Edge reject rig-deformed prims.** The authored `points`
    array is the drawn geometry only for a prim no mover writes. Build,
    once at `_BeginDrag`, the set of prim paths whose `points` any
    `rigExec:moves` or `rigExec:weightTarget` relationship targets,
    collected over the WHOLE stage by traversal the way
    `rigExecUsdview._FindRigPaths` does (`rigExecUsdview.py:438-445`) —
    **not** through the hit prim's ancestry: the deformed Geom prims are
    siblings of the rig and `gizmoMath.FindRigRoot()` returns `None` for
    `/Shot/HeroArm/Geom/ArmBody` (probed). Keep it on the `DragState`,
    together with the subset whose writing mover is under the dragged
    target's `RigRootPath()` — the Surface case below. The rig graph
    cannot change mid-drag, so neither set needs a cache, and neither
    may hang off `SolverPosedCache`, which invalidates on
    `rigExec:joints` and would answer stale for a `moves` edit. A Point
    or Edge hit on a prim in the first set yields no candidate and the
    status reads `snap: <prim> is rig-deformed`.
    Surface is normally unaffected — `hitPoint` comes from the renderer
    — **except** when the mover writing those points is under the
    dragged target's `RigRootPath()`: the drag then re-poses the
    geometry under the ray each mouse-move, the same feedback loop step
    2 prevents (`rigExecPose --frames 1001,1024 --targets` reports
    `ArmBody.points moved max 3.71394`). Reject that case too, same
    message.
    Consequence the status line must state rather than hide: on a scene
    whose only geometry is one rigged character, dragging that rig's
    controls leaves Point/Edge/Surface with no candidate and the object
    does not move (assumption 4). The clause reads
    `snap: no target (rig-driven geometry excluded)`, not a bare
    `no target`.
4. **Point**: read the prim's point array
   (`UsdGeom.PointBased.GetPointsAttr`), project every point, and take
   the one nearest the cursor within `SNAP_PIXELS` (12 logical px,
   scaled by the device pixel ratio). Carry the points to world with
   `UsdGeom.XformCache(time).GetLocalToWorldTransform(prim)`.
   `ComputeRigFrames` is **not** applicable: it composes frames from
   `rest:*` / `avars:*` / `posed:space` only and never reads xformOps
   (`gizmoMath.py:445, 365, 378, 384`), so on a picked Mesh or
   BasisCurves it returns the rig frame and silently drops the prim's
   own transform (probed: a mesh with `xformOp:translate=(0, 7, 0)`
   under a RigExecRoot comes back with the y gone and `reason` empty);
   it returns ASSET-space frames, not a world matrix; and its 4th
   argument is the path SET from `SolverPosedCache.For(root)`
   (`gizmoMath.py:465` does `prim.GetPath() in solverPosed`), not the
   cache — passing the cache raises `TypeError: argument of type
   'SolverPosedCache' is not iterable`. Known v1 limitation: a prim
   whose transform is republished by
   `RigExecResultsSceneIndex::_ComputeDrivenXform` (a constraint-driven
   Xform, `examples/10_AimXformTurret.usda`) draws at a matrix
   `XformCache` does not know about, so Point/Edge candidates on it are
   offset. Surface is unaffected: `hitPoint` comes from the renderer.
5. **Edge**: build the candidate segments — mesh edges from
   `faceVertexIndices` / `faceVertexCounts`, curve segments from
   consecutive CVs — drop any with an endpoint at `w <= 0`, take the
   nearest projected segment within `SNAP_PIXELS`, and convert its
   screen foot to the world point with the perspective-correct inverse
   of section 3.
6. Nothing within the radius, or no hit at all: return `None`, the
   object does not move, and the status says so.

**Throttle**: skip the pick unless the cursor has moved more than
`PICK_MOVE_PIXELS` since the last one, and reuse the previous candidate
in between — but only while the active mode is unchanged. The cache is
keyed on the **active mode as well as the point**: a candidate carries
its `kind`, and `gizmoUI._ReapplyDrag` (`gizmoUI.py:1900-1903`) re-runs
`ApplyDrag` at the *identical* `current` on every hold press and
release, so without mode invalidation, pressing `C` while a `V`
candidate is cached yields an "edge" snap that is really the cached
point, with the status line and the marker both claiming Edge.
`PICK_MOVE_PIXELS` is in PHYSICAL pixels because the throttle lives in
Qt-free `gizmoDrag` and compares `DragState.current` against the last
pick point, and `gizmoUI._Position` hands those in already multiplied by
`devicePixelRatioF()` (`gizmoUI.py:1636-1652`). `SNAP_PIXELS` is the
opposite case: its only consumer is the `gizmoUI` resolver, which scales
it at the call site the way `_HitTest` does.

### 4.5 Settings, controls and feedback

- `gizmoSettings._FIELDS` gains exactly one entry, `snapMode`, defaulting
  to `SNAP_OFF` for every tool in `MayaDefaults`. `_FIELDS` and
  `MayaDefaults` must change in the same edit: `ToolSettings.__init__`
  does `values[name]` for every field, so a field without a default is a
  `KeyError` the first time a `ToolSettings` is constructed. That fails
  `tests/python/test_gizmo_settings.py` and
  `tests/python/test_gizmo_drag.py` at run time (both call
  `MayaDefaults` from inside test bodies, not at import) and
  `tests/testUsdviewGizmo.py` at plugin load, where `gizmoUI.py:930`
  constructs `GizmoSettings()`. `test_gizmo_math.py` and
  `test_gizmo_screen.py` never import `gizmoSettings` and are
  unaffected.
- `SNAP_OFF / SNAP_GRID / SNAP_POINT / SNAP_EDGE / SNAP_SURFACE` tokens,
  `SnapLabel()` and `SnapChoices(tool)` mirror the existing
  `ORIENT_*` / `OrientationLabel` / `OrientationChoices` shapes.
  Move offers all five; Rotate offers off and grid; Scale and Select
  offer none.
- `gridSize` lives on `GizmoSettings` beside `manipulatorSize`, clamped
  and notified the same way, default 1.0, floor 1e-4. `Reset(tool)`
  therefore leaves it alone, which is right: resetting the Move tool
  should not move the world grid.
- **Toolbar: one** `Snap:` `QToolButton` with an `InstantPopup` menu of
  mutually exclusive checkable actions, labelled with the active mode.
  Not four toggles: the row already overflows at usdview's default
  viewport width (`gizmoUI.py:421-432`), and no test would catch that
  because the end-to-end test resizes to 1800x1000 first.
- **Tool Settings panel**: when `SnapChoices(tool)` is non-empty, a
  `Snap To` combo built from it exactly like the orientation combo, plus
  a `Grid Size` spin box whenever `SNAP_GRID` is among them. When it is
  empty (Scale) both rows are omitted — an empty combo would hand
  `currentData() is None` to the slot — and the matching lines in
  `Sync()` are guarded by the same test, the way `freeRotate` /
  `preventNegativeScale` already are (`gizmoUI.py:784-788`): `_Clear()`
  `deleteLater()`s the row widgets (`gizmoUI.py:665-671`), so an
  unguarded `Sync()` on a later notification touches a deleted C++
  object and raises `RuntimeError`. Rotate gets a wrapped label saying
  point/edge/surface snapping is Move-only and that Grid needs a Gimbal
  ring.
- **Overlay**: a new `SnapCandidate()` accessor on the controller,
  painted **last** in `paintEvent` and never published through
  `Handles()` — two existing assertions require
  `HandleScreenPositions() == {}` for the Select tool. A diamond plus a
  crosshair, with a kind-specific extra: a short normal tick for
  surface, the two adjacent projected segments for edge, world-axis
  ticks for grid. A faint leader from the handle centre to the candidate
  while the two differ. The marker also shows on hover when a mode is
  armed, because "hold V and see what lights up" is half of what makes
  Maya's snapping usable.
- **Status**: a snap clause is **appended** to whichever branch
  `Status()` returns, never replacing one — three end-to-end assertions
  substring-match the existing text — and the live-rotate branch
  (`gizmoUI.py:1393-1395`) returns *early*, so Rotate's grid mode and the
  `snap: Point is Move only` message of section 1.2 need the clause
  appended there too. It is shown only while a snap is active or armed,
  and placed early in the string, before the skipped-children and
  warning suffixes, because the label elides.

### 4.6 Undo

Unchanged. A snapped drag writes the same attributes through the same
`EditRecorder`, so it is one undo step exactly as an unsnapped one is.

## 5. Error handling

- Pick raises, or the renderer is unavailable: `Tf.Warn` once and treat
  it as "no candidate"; never let it kill the drag. `_getRenderer()` and
  the app-controller reach are private, so both need the same
  try/except that degrades rather than raising.
- A collapsed channel frame (`|det| < 1e-12`): the snap is dropped, the
  same bail `gizmoMath.py:1296-1301` already makes.
- A mode the active tool does not offer: inert, with the reason in the
  status line.
- No stage, or a stage with no geometry: grid still works; the other
  three report no candidate.

## 6. Testing

**Qt-free** (`tests/python/test_gizmo_snap.py`, plus additions to
`test_gizmo_drag.py` — items 1, 5 and 7a, which need a `DragState` — and
to `test_gizmo_settings.py`), with synthetic cameras and candidate
lists, in the `_Check` style of the existing files:

1. Mode and hold precedence as a table, like the existing
   `TestTranslateSnapChoice`: nothing held with each sticky mode; each
   hold alone; `V`+`X`, `C`+`X`, `V`+`C`, `V`+`C`+`X`; each hold against
   a conflicting sticky mode; and every hold on `TOOL_ROTATE` /
   `TOOL_SCALE` returning `SNAP_OFF` with a non-empty reason.
2. Nearest candidate in **pixels**, including a case where the
   world-nearest and screen-nearest candidates differ, a candidate
   behind the eye, and one outside the radius.
3. `ConstrainToHandle` per handle kind: centre lands on the candidate,
   axis lands on its projection onto the axis line, plane lands in the
   plane, Ctrl+axis behaves as a plane.
4. World grid: a pivot at (1.4, −2.6, 0.2) with `gridSize` 1.0 lands the
   **world** point on (1, −3, 0); halves go away from zero; with an axis
   handle only the reachable component moves.
5. **The assertion that proves the feature exists**: on an in-memory
   stage with a parent rotated 45° about Y, the world position lands on
   the grid while the channel values do not. Today's `X` fails this by
   construction.
6. The perspective-correct edge parameter, tested by round trip: build a
   foreshortened segment, pick a cursor, recover the world point,
   reproject it, and assert it lands within 1e−6 px of the screen-space
   foot. That single assertion catches every sign and `w`-ordering
   error.
7. Settings: `snapMode` defaults to `SNAP_OFF` for all four tools; for
   the tools that offer any choices (Move and Rotate) the default is
   among `SnapChoices(tool)`; `SnapChoices(TOOL_SCALE)` and
   `SnapChoices(TOOL_SELECT)` are both `()`, exactly as
   `OrientationChoices(TOOL_SELECT)` already is, so those two are
   asserted separately and never fed to the "default is offered" loop.
   Move offers all five; Rotate offers off and grid only.
7a. **Throttle**, with a counting stub `snapResolver` on the `DragState`
    (section 4.1's injection is what makes this headless): the first
    move picks; a second move 1 px away reuses the cached candidate
    without calling the resolver; a move 5 px from the last pick calls
    it again; re-running `ApplyDrag` at the same `current` with a
    different active mode calls it a third time and returns a candidate
    whose `kind` is the new mode.

**testusdview** (additions to `tests/testUsdviewGizmo.py`, on
`examples/ArmShotAnim.usda`), only what a synthetic camera cannot do.
The asset's own meshes and curves are rig-deformed (section 3), so both
the moved object and everything it lands on are scratch prims: a
`/Shot/SnapProbe` Xform placed at (4, 10, 2) as the moved object, and an
undeformed one-quad Mesh `/Shot/SnapTarget` authored into the SESSION
layer with corners (0, 9.5, 3), (8, 9.5, 3), (8, 10.5, 3), (0, 10.5, 3)
— no mover writes to it, so section 4.4 step 3a lets it through and its
authored points are what Hydra draws. All are defined and removed the
way the file already defines and removes a temporary child:

8. Point snap at the pixel of `/Shot/SnapTarget`'s (8, 9.5, 3) corner
   lands the pivot there within 1e−4.
9. Edge snap aimed at the pixel of (4, 9.5, 3) lands *on* SnapTarget's
   bottom edge (y=9.5, z=3, 0 < x < 8) and not on either endpoint of
   that edge.
10. Surface snap aimed inside the quad lands on it: |z − 3| < 1e−3 and
    inside its x/y bounds.
11. Invisible prims are never candidates. Author, into the session layer
    beside `/Shot/SnapTarget` (item 8) and removed with it, an invisible
    `UsdGeom.Points` prim `/Shot/SnapDecoy` whose points are
    `/Shot/SnapTarget`'s four corners displaced by (0, 0, 0.5) toward
    the camera, so it would win every Point pick were it visible. Assert
    that at the pixel of `SnapTarget`'s (0, 9.5, 3) corner the reported
    candidate prim is `/Shot/SnapTarget` and the candidate point is
    (0, 9.5, 3) — not the decoy's (0, 9.5, 3.5) — and that
    `/Shot/SnapDecoy` never appears as a candidate prim. Do **not**
    build this on `BicepFlexTarget` and `ArmBody`: ArmBody is
    rig-deformed and is rejected by section 4.4 step 3a, only two of
    BicepFlexTarget's four points coincide with an ArmBody corner
    (probed), and those two are off screen under this file's camera.
12. The `C` and `V` holds are consumed by the gizmo and never reach
    usdview. Run this BEFORE any other `C` or `V` in the file:
    `_showUsdValidation` (`appController.py:2721-2726`) never sets
    `_usdValidationWidget` back to `None`, and a `C` that got through
    flips `viewSettings.autoComputeClippingPlanes` for every later
    projection, so one leak poisons the rest of the run. Record
    `clip = appController._ui.actionAuto_Compute_Clipping_Planes
    .isChecked()` first. With a drag live, press and HOLD `V`
    (`QTest.keyPress`, not `keyClick`), assert both
    `appController._usdValidationWidget is None` and
    `controller.SnapMode() == gizmoSettings.SNAP_POINT`, then release
    it; same for `C` with `isChecked() == clip` and `SNAP_EDGE`. Both
    halves are required: the usdview half alone passes if the key never
    dispatched, the gizmo half alone passes if the gizmo acted but let
    the shortcut fire too. (Baseline probed 2026-09-03: with today's
    code a `C` and a `V` delivered to the view during a live drag DO
    toggle the action and DO create the widget, so these assertions are
    not vacuous.)
13. Two holds and release ordering, through `_Driver.KeyDown` / `KeyUp`
    and `controller.SnapMode()`, never `Key()`: hold `X`, then `V`, and
    `SnapMode()` is `SNAP_POINT` (Point outranks Grid, section 1.7);
    release `X` and it is still `SNAP_POINT`; release `V` and the next
    drag is unsnapped.
14. The snap clause reaches `StatusBar().FullText()`.
15. One snapped drag is one undo step, and Ctrl+Z restores the pre-snap
    value.

Items 8-11 need the view set up first, in this order, and everything
restored afterwards:

- **Re-frame on the scratch geometry.** Under the camera the earlier
  sections leave (`HandIK` framed, `rotTheta=35`, `rotPhi=−25`,
  `dist *= 2.6`) ArmBody's `x=0` corners project to x = −347.2 and
  −294.4, outside testusdview's 597x540 physical viewport, and
  `computePickFrustum` reports `inImageBounds=False` — the resolver
  returns `None` there. An identity-placed probe is thousands of pixels
  off screen too.
- **Hide the RigExec overlay**, and restore it afterwards. It draws in
  front of `Geom` from every practical camera and wins every pick
  (section 3). This is a debuggability measure only; the shipped fix is
  the `showGuides = False` pick of section 3, which the assertions rely
  on.
- Then check the targets are on screen and separated by more than the
  snap radius, the isolation discipline
  `testUsdviewCurvenetMove.py:84-97` already follows.

**Regressions to expect and handle in the same change**: the two
`test_gizmo_drag.py` tests that pin `X`'s old meaning, the `J`/`X`
paragraph of `docs/viewport-gizmos.md`, and the two bullets of the gizmo
design spec. `ApplyDrag`'s existing `holdSnap=` / `holdGrid=` keyword
names and `Target.ApplyTranslate`'s exact keyword set must not change,
or five kwarg-equality assertions and every call site break.
