# Maya-style ViewCube for the usdview viewport

Date: 2026-09-02. Status: approved for implementation under the assumptions
in section 1 (the request was executed autonomously — Fable planned, the
Meta Muse 1.3 contributor model implemented, Fable checked — so the
assumptions are the interpretation a careful colleague would make and are
the first thing to revisit if the result feels wrong).

## 0. Request

"Add a Maya viewport style viewcube into the 3D viewport, complete with
switching to side views when clicking on the cube sides."

## 1. Assumptions (decisions made without a live user)

1. **Which viewport**: usdview's stage view, through the existing
   `rigExecUsdview` plugin container — the same host the gizmo toolbar
   uses (`plugin/rigExecUsdview/gizmoUI.py`). Nothing is added to
   usdview itself.
2. **What "Maya viewcube" means for v1** (parity subset, YAGNI applied):
   - a small labelled cube (FRONT / BACK / LEFT / RIGHT / TOP / BOTTOM) in
     the top-right corner of the viewport, oriented like the current
     camera, drawn with a mild perspective;
   - 26 click regions — 6 faces, 12 edges, 8 corners — each of which
     orbits the free camera to that view, keeping the orbit centre and
     distance, with a short animated transition;
   - hover pre-highlight of the region under the cursor;
   - dragging on the cube tumbles the camera (Maya lets you orbit by
     dragging the cube);
   - a "home" glyph that returns to the default perspective view and
     frames usdview's current selection;
   - a menu toggle (`RigExec → View Cube`) to hide and show it.
   Out of scope for v1 (each is a follow-up, none blocks the request):
   the rotate-90° / roll arrows Maya shows while sitting on a face, the
   compass ring, the orthographic/perspective context menu, and any
   persisted settings.
3. **Camera semantics**: a click always lands on usdview's *free* camera.
   If a camera prim is the active view, the click first switches to the
   free camera the way usdview's own Alt-drag does
   (`StageView.switchToFreeCamera`), then orbits. The cube itself always
   reflects whatever camera is actually rendering, prim or free.
4. **Up axis**: the labels follow `UsdGeomGetStageUpAxis`. FRONT is the
   view usdview starts a free camera in (`rotTheta = rotPhi = 0`): for a
   Y-up stage the camera sits on +Z looking down −Z; for a Z-up stage it
   sits on −Y looking down +Y. RIGHT is the +X side, TOP is the up axis.
   This matches Maya's own face assignment in both up-axis modes.
5. **Roll**: a face/edge/corner click yields an unrolled view (roll 0),
   which is what Maya does. `FreeCamera` has no public roll setter; the
   private `_rotPsi` is cleared under a `hasattr` guard, with the same
   justification the plugin already uses for reaching
   `_UsdviewApi__appController._stageView`.
6. **Top/bottom heading**: a pure TOP or BOTTOM view has no heading of its
   own, so it keeps the camera's current `rotTheta` snapped to the nearest
   multiple of 90°, as Maya keeps the current heading when you click TOP
   from a perspective view.
7. **Home view** is the FRONT-TOP-RIGHT corner view (camera direction
   (1, 1, 1)/√3 in the up-space frame of section 3), followed by
   usdview's own frame-selection (`StageView.updateView(True, True)`),
   which frames the whole stage when nothing is selected.
8. The cube is an ordinary (mouse-accepting) child widget of the stage
   view, drawn with QPainter — the gizmo overlay's technique, verified to
   composite over usdview's `QOpenGLWidget`. Nothing is authored into any
   layer, and no camera move re-enters the evaluator.

## 2. usdview facts the design relies on

All from the PR-4156 install
(`/Users/burkard/work/usd-install/lib/python3.11/site-packages/pxr/Usdviewq`).

- `FreeCamera` (`freeCamera.py:106-131`) builds its transform as
  `T(0,0,dist) · Rz(−ψ) · Rx(−φ) · Ry(−θ) · YZUpInv · T(center)` in
  row-vector convention, where `YZUpInv` is the identity for Y-up stages
  and a +90° rotation about X for Z-up stages. `rotTheta` and `rotPhi`
  are public properties whose setters emit `signalFrustumChanged`;
  `_rotPsi` (roll) has no public setter; `Tumble(dTheta, dPhi)` adds to
  both angles.
- Measured (2026-09-02, with a throwaway probe that built `FreeCamera`
  objects and read `computeGfCamera(...).frustum`; the unit test in
  section 6 repeats the measurement), the camera
  position direction **from the orbit centre toward the camera**, in the
  up-space frame of section 3, is

      p(θ, φ) = (−sin θ · cos φ,  sin φ,  cos θ · cos φ)

  so θ = 0, φ = 0 is FRONT (camera on +Z_up), θ = 90 puts the camera on
  −X_up (LEFT), θ = −90 / 270 on +X_up (RIGHT), θ = 180 on −Z_up (BACK),
  φ = 90 on +Y_up (TOP, screen-up = −Z_up), φ = −90 on −Y_up (BOTTOM,
  screen-up = +Z_up). For Z-up stages the same angles give the same
  up-space directions; only the up-space→world mapping differs. The
  measured table (both up modes, worst error 4.4e−16; the angle columns
  are what the module's `DirectionForAngles` must reproduce):

      (θ, φ)                                  direction (up-space)
      (0,   0)                                (0, 0, 1)          FRONT
      (90,  0)                                (−1, 0, 0)         LEFT
      (−90, 0)                                (1, 0, 0)          RIGHT
      (270, 0)                                (1, 0, 0)          RIGHT
      (180, 0)                                (0, 0, −1)         BACK
      (0,   90)                               (0, 1, 0)          TOP
      (0,  −90)                               (0, −1, 0)         BOTTOM
      (45,  0)                                (−√½, 0, √½)
      (0,   45)                               (0, √½, √½)
      (−45, asin(1/√3) = 35.26438968…°)       (1, 1, 1)/√3       home corner

  The screen-up vector for the same angles, in up-space, is
  `up(θ, φ) = (sin θ · sin φ, cos φ, −cos θ · sin φ)` (verified against
  `frustum.ComputeUpVector()` in both up modes, error < 1e−15).
- The view settings model forwards a free camera's `signalFrustumChanged`
  as `signalFreeCameraSettingChanged` (`viewSettingsDataModel.py:832-833`
  connects it to `_frustumChanged`, `:393-398`, which re-emits), and the
  `StageView` itself repaints on it (`stageView.py:855-856` connects it to
  `_onFreeCameraSettingChanged`, `:2184-2188`, which calls
  `switchToFreeCamera()` then `update()`; `switchToFreeCamera()` is a
  no-op while `cameraPrim is None`). `appController` does not listen to
  this signal — it only connects `signalSettingChanged` to menu / HUD
  refreshes. So assigning `rotTheta` / `rotPhi` repaints the viewport
  without the plugin calling `updateGL` — the plugin still calls it,
  cheaply, so the behaviour does not depend on that wiring. A camera-prim
  switch (`viewSettings.cameraPrim = prim`) only emits
  `signalVisibleSettingChanged` → `StageView.update`; its frustum signal
  arrives with the next paint's `resolveCamera()`, so the controller also
  refreshes on `viewSettings.signalSettingChanged`.
- `StageView.resolveCamera()` (`stageView.py:1589-1628`) returns the
  rendering `Gf.Camera` (prim or free) conformed to the viewport and emits
  `signalFrustumChanged` only when the frustum changed, so calling it from
  a `signalFrustumChanged` handler cannot loop. Its frustum's
  `ComputeViewDirection()` / `ComputeUpVector()` give the world-space
  camera basis the cube is oriented by.
- `StageView.switchToFreeCamera(computeAndSetClosestDistance=True)`
  (`stageView.py:2045-2076`) converts a prim camera into an equivalent free
  camera and is a no-op when the free camera is already active.
- usdview tumbles at `0.25°` per **physical** pixel
  (`stageView.py:2134-2135`); Alt / Meta on a press is usdview's camera
  navigation claim (`stageView.py:2099`).
- `StageView.updateView(resetCam=True, forceComputeBBox=True)` is what
  usdview's *Frame Selection* does (`appController.py:2475-2479`).
- Plugins load before the stage view exists; the gizmo's
  `InstallViewportTools` (`gizmoUI.py:2034-2063`) retries on a
  zero-length `QTimer` up to 20 times, and the container drives the
  install from `_OnStageReplaced`. The cube installs the same way.

## 3. Geometry model

**Up-space** is a right-handed frame whose +Y is the stage's up axis and
whose +Z points toward the FRONT camera. For a Y-up stage up-space *is*
world space. For a Z-up stage

    world = (x_up, −z_up, y_up)        up-space = (x_w, z_w, −y_w)

(checked against the measured free-camera positions: θ = φ = 0 on a Z-up
stage puts the camera at world (0, −dist, 0) = up-space (0, 0, +dist)).

**Regions.** The six faces carry up-space normals FRONT (0,0,1), BACK
(0,0,−1), RIGHT (1,0,0), LEFT (−1,0,0), TOP (0,1,0), BOTTOM (0,−1,0). A
region is a non-empty set of at most one face per axis; its direction is
the normalised sum of its faces' normals; its name joins the face names
in the fixed order *front|back*, *top|bottom*, *left|right* with `-`:
`front`, `front-top`, `top-right`, `front-top-right`, `back-bottom-left`
… — 26 in all.

**Angles for a direction** `d` (unit, up-space), given the current
`rotTheta`:

    φ = asin(d.y)                                   (degrees)
    θ = atan2(−d.x, d.z)      if |d.y| < 1 − 1e−9
    θ = nearest multiple of 90° to the current θ    otherwise (pure TOP / BOTTOM)

then θ is unwrapped to the representative within ±180° of the current θ
so the animated orbit takes the short way round.

**Cube projection.** Cube corners at (±1, ±1, ±1) in up-space, converted
to world. With the camera basis `right = view × up`, `up`, `view` (world
unit vectors from the resolved camera) a world vector `w` has view
coordinates `(w·right, w·up, −w·view)`, +z toward the camera. The eye
sits at `(0, 0, D)` in view space with `D = 4` (cube half-size 1); a
point projects to

    sx = cx + R · x · D / (D − z)        sy = cy − R · y · D / (D − z)

with `R` the face half-size in pixels at `z = 0` and `(cx, cy)` the cube
centre in widget pixels (so a face facing the camera, at `z = +1`,
projects larger than `R` by `D/(D−1) = 4/3`, and the opposite face
smaller by `D/(D+1) = 4/5`). A face is visible iff
`(eye − faceCentre) · faceNormal > 0` in view space; a convex cube's
visible faces never overlap, so no depth sort is needed. `R` is
`RADIUS_FRACTION · WIDGET_SIZE` with `RADIUS_FRACTION = 0.24` (26.4 px
in a 110 px widget): the projected silhouette is at most `1.9085 · R` on
the corner views (`1.8856 · R` on face views, `1.7321 · R` on edge
views), 50.4 px, inside the 53 px the widget allows; labels are drawn
on the faces, so nothing is reserved outside the silhouette. A unit test
pins the fit for all 26 canonical views in both up modes.

**Hit-testing** is a ray cast, not a polygon test, so faces, edges and
corners come out of one computation: the pixel maps back to the `z = 0`
plane, the ray from the eye through it is carried to up-space, and a slab
intersection with `[−1, 1]³` yields the entry face and the in-plane
coordinates `(u, v)`. A coordinate beyond `±0.5` adds that neighbouring
face to the region; so the middle of a face is the face, its border band
is an edge and its corner square is a corner. **Highlight polygons** are
the projected sub-quads of each visible face the region touches — on
each such face exactly the set of `(u, v)` the hit test assigns to the
region: along each in-plane axis the coordinate spans `[−0.5, 0.5]` when
the region has no face on that axis, and the outer half toward the
region's neighbouring face when it has one (`[0.5, 1]` if that
neighbour's normal is +axis, `[−1, −0.5]` if it is −axis). So a face's
polygon is the middle `[−0.5, 0.5]²` square, an edge's is a 0.5 × 1 band
on each of its two faces (never including the corner squares), and a
corner's is the 0.5 × 0.5 outer square on each of its three faces.
Highlight polygons and hit zones are the same sets, and the polygons are
also what the end-to-end test clicks.

**Labels** are drawn through `QTransform.squareToQuad` (projective,
supported by PySide6 6.11) so they sit on the face in perspective. The
text is laid out in a `LABEL_BOX` (100-unit) square with a bold
`LABEL_PIXELS` (20 px) font and an extra `1/LABEL_BOX` painter scale
maps that box onto the unit square, because Qt resolves a `QFont` to an
integer pixel size before the painter transform applies (a sub-pixel
point size paints nothing; a 1 px floor blows one glyph over the face).
Each
face has an in-plane (right, up) pair so the text reads upright in that
face's canonical view: FRONT (+X, +Y), BACK (−X, +Y), RIGHT (−Z, +Y),
LEFT (+Z, +Y), TOP (+X, −Z), BOTTOM (+X, +Z), all in up-space.

## 4. Components

| Module | Qt? | Responsibility |
|---|---|---|
| `plugin/rigExecUsdview/viewCubeMath.py` | no | up-space mapping, the 26 regions, angle ↔ direction, cube projection, ray-cast hit test, highlight polygons, angle interpolation |
| `plugin/rigExecUsdview/viewCubeUI.py` | yes | `ViewCubeWidget` (paint, hover, click, drag, home glyph), `ViewCubeController` (camera basis, orbit animation, install / visibility), `InstallViewCube` / `GetController` |
| `plugin/rigExecUsdview/rigExecUsdview.py` | no | `_EnsureViewCube` from `_OnStageReplaced`, `RigExec → View Cube` toggle |
| `tests/python/test_viewcube_math.py` | no (imports `FreeCamera` for a cross-check) | headless math test, registered in CMake and `bin/run_python_tests.sh` |
| `tests/testUsdviewViewCube.py`, `bin/run_testusdview_viewcube.sh` | yes | end-to-end in testusdview on `examples/ArmShotAnim.usda` |
| `docs/view-cube.md`, `README.md` | — | user documentation |

### 4.1 `ViewCubeController` behaviour

- Installed once per process on the stage view; reads the up axis from
  the data model's stage on install and on every `signalStageReplaced`.
- On `signalFrustumChanged`, on view `Resize` / `Show`, and after its own
  camera writes, it recomputes the camera basis from
  `view.resolveCamera()` and repaints the widget. Each refresh also
  recomputes the hover at the last mouse point (`RefreshHover` from
  `_RefreshBasis`), so the highlight and the pointing hand follow the
  pixel with no mouse motion, through a click's orbit, a tumble, an
  animated transition, and usdview's own Alt-drag; a leave clears it.
  The widget is placed at
  the view's top-right (`MARGIN` px in) and re-raised on a zero-length
  timer so it ends above the gizmo overlay, which raises itself
  synchronously in its own `Resize` handler.
- `Orbit(regionName)` and `OrbitTo(theta, phi)`: switch to the free
  camera, then animate `rotTheta` / `rotPhi` from the current values to
  the target with a smoothstep over `animationMs` (default 250; 0 means
  immediate, which the tests use), on a `QTimer`; the last tick writes the
  exact target and clears roll. A new orbit, a drag, or a stage
  replacement cancels a running animation. `IsAnimating()` reports it.
- `Home()`: `switchToFreeCamera`, `updateView(True, True)` (frame), then
  `OrbitTo` the home angles.
- Drag: a left press on the cube or the home glyph with no Alt /
  Meta arms a drag; a plain left press on the empty widget background
  is `ignore()`d too so usdview's prim picking still works in that
  corner; motion beyond `DRAG_THRESHOLD` logical pixels starts
  tumbling with
  `freeCamera.Tumble(0.25·dx·ratio, 0.25·dy·ratio)` per move (physical
  pixels, usdview's rate), where `dx, dy` are measured from the last
  point that tumbled — the press point for the move that crosses the
  threshold — so the dead zone is applied, not swallowed, and a drag
  from A to B tumbles by exactly `0.25 · (B − A) · ratio`; a release that
  never crossed the threshold is a click on the region (or the home
  glyph) under the press point. Presses with Alt / Meta or a non-left
  button are `ignore()`d so usdview's own navigation and menus keep them,
  and because Qt keeps routing that gesture's moves and release to the
  widget under the press, those are `ignore()`d too whenever the press
  was not the cube's.
- `SetVisible` / `IsVisible` show and hide the widget; the container's
  menu item toggles them.
- Test accessors: `RegionAt(widgetPoint)`, `HoverRegion()`,
  `ScreenPointForRegion(name)` (a widget-local point inside the region's
  largest visible highlight polygon, or `None` when the region is not on
  a visible face), `HomePoint()`, `CurrentAngles()`, `Basis()`, `Widget()`.

### 4.2 Painting

Faces filled light grey, edges 1 px darker grey, labels bold sans in dark
grey; the hovered region's polygons filled in Maya's pale highlight blue;
whole widget at 0.6 opacity while the cursor is elsewhere and 1.0 while
it is over the widget. The home glyph is a 14 px house outline in the
widget's top-left corner, highlighted the same way on hover. The cursor is
a pointing hand over a region or the glyph.

## 5. Error handling

- No stage view (headless / C++ tests): `InstallViewCube` returns `None`
  after the bounded retries and the container warns once, exactly like
  the gizmo.
- `resolveCamera()` raising (no renderer yet): the controller keeps
  the last basis — the identity, i.e. the FRONT view, before any
  camera has resolved — and refreshes on the next frustum change.
- A destroyed stage view (`destroyed` signal): the controller drops its
  widget and timer references and answers every later call as a no-op.
- The free camera missing (`viewSettings.freeCamera is None`) after a
  `switchToFreeCamera`: the click is dropped with a `Tf.Warn`, never a
  traceback.

## 6. Testing

- `tests/python/test_viewcube_math.py` (headless, `_Check` style, exit 0
  with a `VIEWCUBE_MATH_OK` banner): 26 unique regions with unit
  directions and canonical names; angle ↔ direction round trip for all
  26 in both up modes; TOP / BOTTOM heading snap; unwrap takes the short
  way; projection facts for the identity basis (front face visible and
  centred, back face hidden; 1 visible face for a face view, 2 for an
  edge view, 3 for a corner view); the silhouette fits the widget for all
  26 views; ray-cast hit test agrees with the highlight-polygon centroids
  for every region reachable from every canonical view, and misses
  outside the cube; **cross-check against usdview's real `FreeCamera`**:
  for every region and both up modes, the angles the module computes
  place the camera at the region's world direction and, for the six
  faces, give the expected screen-up.
- `tests/testUsdviewViewCube.py`: installed on stage load; placed
  top-right; clicks on FRONT, then on the FRONT-RIGHT edge, RIGHT,
  RIGHT-TOP, TOP, the FRONT-TOP-RIGHT corner, FRONT-BOTTOM and BOTTOM
  (each clicked from a view in which its face is visible) assert the
  resolved camera's view direction and up vector on the stage (Y-up);
  hover reports the region and clears on leave; an animated orbit is
  reported by `IsAnimating()` and converges within a bounded pump loop; a
  drag tumbles by usdview's rate and does not orbit; an Alt-press (and
  its move and release) propagates to usdview's own camera navigation and
  does not orbit; a camera prim authored in the session layer orients
  the cube and a click on the RIGHT face (the only face visible from a
  camera looking down −X) switches back to the free camera; Home reaches
  the home direction and frames the selection; `SetVisible` hides the
  widget and the container's command and menu item exist. Prints
  `RIGEXEC_VIEWCUBE_OK`. `RIGEXEC_VIEWCUBE_SHOT` saves a window grab.
