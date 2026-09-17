# View cube (usdview)

Design spec `docs/superpowers/specs/2026-09-02-view-cube-design.md`.
Added 2026-09-03.

The RigExec usdview plugin adds a Maya-style view cube to the
top-right corner of the viewport: a small labelled cube that shows
the camera orientation and orbits the camera when clicked.
`RigExec → Viewport → View Cube` toggles it; like `Viewport Tools`, the menu
item is a toggle rather than a window because the cube lives inside
the viewport frame, not in a floating panel.

The cube is an ordinary mouse-accepting child widget of the stage
view (`ViewCubeWidget` in `plugin/rigExecUsdview/viewCubeUI.py`),
painted with `QPainter` — the gizmo overlay's technique, verified
to composite over usdview's `QOpenGLWidget`. Nothing is authored
into any layer, and no camera move re-enters the evaluator. All
geometry (regions, angles, projection, hit testing) lives Qt-free
in `plugin/rigExecUsdview/viewCubeMath.py`, which is what the
headless test exercises.

## Clicking a face, edge or corner

The cube has 26 click regions: 6 faces (`front`, `back`, `left`,
`right`, `top`, `bottom`), 12 edges (`front-top`, `top-right`, …)
and 8 corners (`front-top-right`, `back-bottom-left`, …). Region
names join face names in the fixed order front|back, top|bottom,
left|right. Only the faces turned toward the camera can be hit —
from a face view exactly one face is visible, from an edge view
two, from a corner view three — so some regions have no point from
some views (`ScreenPointForRegion` returns `None` there).

A click orbits the free camera to that view
(`ViewCubeController.Orbit` → `OrbitTo`): the orbit centre and
distance are kept, the roll is cleared to 0 (Maya lands unrolled;
`FreeCamera` has no public roll setter, so `_rotPsi` is cleared
under a `hasattr` guard), and a pure TOP or BOTTOM view keeps the
camera's current heading snapped to the nearest multiple of 90°
(`SnapHeading`), as Maya keeps the current heading when TOP is
clicked from a perspective view.

## Hover, drag, home

The region under the cursor is pre-highlighted in Maya's pale
highlight blue before it is clicked, and the cursor becomes a
pointing hand over it. The highlight and the pointing hand follow
the pixel rather than the region: they are recomputed on every
camera move — a click's orbit, a tumble, an animated transition,
even usdview's own Alt-drag — with no mouse motion, and leaving
the cube clears them. The widget rests at 0.6 opacity and comes
up to full opacity under the cursor (`IDLE_OPACITY`).

Dragging on the cube tumbles the camera instead of orbiting:
`freeCamera.Tumble` at usdview's own rate, 0.25° per physical
pixel (`TUMBLE_RATE`, scaled by the view's `devicePixelRatioF`).
A press must travel 4 logical pixels first (`DRAG_THRESHOLD`), so
an ordinary click never nudges the camera; a release that never
crossed the threshold is a click. Presses with Alt or Meta, or a
non-left button, are ignored so usdview's own Alt-drag navigation
keeps them — including the rest of that gesture. Only the cube
and the home glyph take presses: a click on the empty widget
background falls through to usdview's own picking.

The house glyph in the widget's top-left corner is Home:
`ViewCubeController.Home` runs usdview's own Frame Selection
(`updateView(resetCam=True, forceComputeBBox=True)`, which frames
the whole stage when nothing is selected) and then orbits to the
FRONT-TOP-RIGHT corner view (`HOME_REGION`).

## Up axis

The labels follow `UsdGeomGetStageUpAxis`. FRONT is the view
usdview starts a free camera in (`rotTheta = rotPhi = 0`): on a
Y-up stage the camera sits on +Z looking down −Z, on a Z-up stage
on −Y looking down +Y. RIGHT is the +X side, TOP is the up axis —
Maya's own face assignment in both modes.

## Animation

Orbits animate with a smoothstep over `animationMs` (default 250;
`ANIMATION_MS`), taking the short way round the heading
(`Unwrap`). Setting `animationMs` to 0 jumps immediately, which
is what the tests use. `IsAnimating` reports a running orbit; a
new orbit, a drag, or a stage replacement cancels it.

## Camera prims

A click follows a prim camera the same way usdview's Alt-drag
does: the cube itself always reflects whatever camera is actually
rendering (its basis comes from `view.resolveCamera()`, so it
tracks a camera prim), while a click first switches to the free
camera (`switchToFreeCamera`) and then orbits.

## Out of scope

Maya's rotate-90° / roll arrows, the compass ring, the
orthographic/perspective context menu, and persisted settings
were excluded by the design. The cube has no persisted
settings; visibility is its only state and it resets when
usdview restarts.

## Tests

- `bin/run_python_tests.sh test_viewcube_math` — headless and
  Qt-free: the 26 regions, the angle ↔ direction round trip, the
  TOP/BOTTOM snap, the projection fit, the ray-cast hit test, and
  a cross-check of every region against usdview's real
  `FreeCamera` in both up modes.
- `bin/run_testusdview_viewcube.sh` —
  `tests/testUsdviewViewCube.py` under `testusdview` on
  `examples/ArmShotAnim.usda`. It drives synthetic mouse events
  through the cube's own projected region points and asserts what
  the camera did: face/edge/corner clicks, hover, the animated
  orbit, drag-to-tumble at usdview's rate, Alt-press propagation,
  the camera-prim round trip, Home and the menu toggle. It prints
  `RIGEXEC_VIEWCUBE_OK` and saves a window grab to
  `$RIGEXEC_VIEWCUBE_SHOT` when that is set.
