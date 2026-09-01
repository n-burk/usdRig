# Viewport gizmos (usdview)

Design spec `docs/superpowers/specs/2026-09-01-viewport-gizmo-toolbar-design.md`.
Added 2026-09-01.

The RigExec usdview plugin adds a toolbar directly above the viewport with
an undoable Move / Rotate / Scale manipulator. `RigExec → Viewport Tools`
toggles it; the menu item is a toggle rather than a window because the
toolbar lives inside the viewport frame, not in a floating panel.

It is two rows: the tool, channel, write, undo and settings controls, and
under them a full-width status line. The status line has a row of its own
because a `QToolBar` folds whatever does not fit into an overflow chevron,
and at usdview's default viewport width the status was the first thing to
disappear — exactly when an artist most needs to read it.

The manipulator is drawn on a transparent child widget of the stage view,
not as prims in the session layer the way the curvenet authoring guides
are. A gizmo has to be screen-constant, unoccluded by the geometry it
sits in, and must not make the evaluator recompute the rig every time the
camera moves; none of that is true of a drawn prim.

## What it edits

| Selection | Pose channels | Pivot channels |
|---|---|---|
| `RigExecControl`, `RigExecJoint` | `avars:tx/ty/tz`, `avars:rx/ry/rz`, `avars:sx/sy/sz` | `rest:tx/ty/tz`, `rest:rx/ry/rz` — no scale |
| any `UsdGeomXformable` with an `XformCommonAPI`-compatible op stack | translate / rotate / scale ops | `xformOp:translate:pivot`, translate only |

Exactly one prim is edited: usdview's focus prim. Multi-selection is out
of scope.

**Pose** moves the character. **Pivot** moves the frame the pose is
expressed in — the rest offset for a rig prim — so the control keeps its
animation but "lives" somewhere else, like moving a Maya pivot without
compensation. Scale is unavailable in Pivot mode on a rig prim because
the evaluator orthonormalizes rest spaces, so a scale written there would
be silently discarded; the toolbar says so rather than accepting a drag
that does nothing.

The gizmo sits at the evaluated frame. For a rig prim that is
`avars * rest * parentRest⁻¹ * parentPosed`, placed by the asset root's
world transform (`libs/rigExec/computations.cpp`), so a rest offset or an
animated parent is already accounted for. Its axes are the target's local
frame, the one the avars are expressed in, unless the Axis Orientation
option asks for another.

## Where the value lands

**Write: Animation** authors a spline knot at usdview's current frame for
a double attribute and a time sample for anything else, by the same rule
the volume weight panel uses (`volumeWeightUI.SetAtTime`). **Write:
Default** authors the attribute's default instead.

A default is invisible whenever a spline or time samples exist on the
same attribute, because value resolution ranks them above it. That is a
trap worth naming: the drag succeeds, the layer changes, and the viewport
does not move. The status label reports it — `the default is outranked by
its spline` — rather than letting the tool look broken.

Everything is authored into the current edit target. usdview starts on
the session layer, so by default a session's gizmo work disappears when
usdview closes, exactly like the other RigExec panels.

The rig's `default:*` channels are unrelated. The evaluator reads only
`rest:*`, `avars:*` and `posed:space`, so "Default" here means the USD
default value, not a rig default pose.

## Undo

Every drag is one undo step, on the same `rigExecUndo` stack the other
panels can adopt later. `Ctrl+Z` undoes; `Ctrl+Shift+Z`, `Shift+Z`
(Maya's redo) and `Ctrl+Y` all redo. They are application shortcuts, so
they work wherever focus is in the window. The toolbar's Undo and Redo
buttons name the step they would reverse on their tooltips, and both go
grey while a drag is live, because undoing the edit a held mouse button
is still writing would leave the two disagreeing.

Undo restores the exact attribute spec in the layer the drag edited,
removing the spec entirely when the drag was what created it — an undone
gizmo edit leaves no trace in the layer, not a zero. `Escape` during a
drag aborts it and pushes nothing. The stack is cleared when the stage is
replaced, because the specs it holds belong to layers that are gone.

## Maya parity

The manipulators follow Autodesk Maya's Move / Rotate / Scale tools,
which is the vocabulary an animator already has.

- **Move**: three axis arrows with cone heads, three planar squares
  coloured for the axis perpendicular to their plane, and a light-blue
  centre square for camera-plane moves. `Ctrl` + an axis drag moves in
  the plane perpendicular to that axis.
- **Rotate**: three rings with their back halves hidden so they stay
  tellable apart where they cross, a light-blue view-axis ring outside
  them, and a grey free-rotate ball behind them. While a ring is
  dragged, a pie slice fills the swept angle in the ring's colour and the
  status label shows the angle in degrees. The angle accumulates past
  180° instead of wrapping, as Maya's does.
- **Scale**: axis lines ending in cubes, a centre cube for uniform
  scale, and the same planar handles for two-axis scale. An axis drag
  scales by the ratio of the cursor's distance from the origin to its
  distance at the press, so dragging a handle to the origin gives zero
  and through it flips the sign.

The handle under the cursor is highlighted before it is grabbed, the last
handle dragged stays yellow, and a middle-drag anywhere in the viewport
repeats that handle without having to hit it again. An axis pointing at
the camera is dimmed and cannot be grabbed, because a foreshortened axis
turns a few pixels of travel into an enormous move.

The `Settings…` button opens the **Tool Settings** window (floating, one
row set per tool). It offers Axis
Orientation (World / Object / Parent, and Gimbal for Rotate, where each
ring changes exactly one Euler channel), Step Snap with a Step Size, Free
Rotate, Prevent Negative Scale, Preserve Children, the manipulator size,
and Reset Tool restoring Maya's defaults. Settings last for the session.

Preserve Children re-authors a plain xform's children after the drag so
they keep their world transforms. It is unavailable on a rig prim,
because the evaluator places a control's children and re-authoring them
would fight it; the checkbox is disabled with that reason on its tooltip.
Children it cannot hold still — an incompatible op stack, a non-zero
pivot, a mirrored transform — are counted in the status label rather than
moved silently.

### Hotkeys

| Key | Action |
|---|---|
| `Q` / `W` / `E` / `R` | Select / Move / Rotate / Scale |
| `+` / `-` | grow / shrink the manipulator by 10% |
| `D`, `Insert` | toggle Pivot editing |
| `J` (hold) | Step Snap for the duration of the drag |
| `X` (hold) | snap the result to a grid of the step size (Move) |
| `Escape` | abort the drag in progress |
| `Ctrl+Z` | undo |
| `Ctrl+Shift+Z`, `Shift+Z`, `Ctrl+Y` | redo |

The tool keys act while the pointer is over the viewport and no text
field has keyboard focus, so typing a prim name into the search box
cannot switch tools out from under you. Undo and redo work anywhere in
the window, and `Escape` aborts a drag whatever has focus.

`J` and `X` are both step snapping and they mean different things. `J`
quantises the movement, so an object that started off the step grid moves
in whole steps and stays off it — Maya's Discrete Move. `X` quantises the
result, so the object lands on the grid however the drag started — Maya's
grid snap. Holding both, `X` wins, because it fully determines where the
object ends up and the relative step then says nothing. Snapping is always
applied to the channel values that get written, never to the world delta,
so an object under a rotated parent still moves in whole steps.

Two keys are shared with usdview rather than taken from it. `J` is
usdview's Toggle Framed View; since Maya's `J` only means anything while
dragging, a live drag claims it and the rest of the time it still frames
the view. `Escape` is usdview's own focus reset, and is likewise claimed
only while a drag is live. `W` is declared in usdview as Watch Window,
but that action is disabled and connected to nothing, so it was free to
take.

### Known differences from Maya

- **`Ctrl` + left-click on macOS** is turned into a right-button press by
  Qt before anything sees it, so Ctrl-clicking an axis cannot start a
  drag there. The gesture that works everywhere is to grab the axis
  first and then hold `Ctrl`.

## When there is no gizmo

The status label always says why, rather than leaving an empty viewport
to interpret:

- a joint posed by a solver (named in some solver's `rigExec:joints`) —
  the evaluator ignores avars for it;
- a prim with a connected or authored `posed:space`, for the same reason;
- an avar with an authored connection, e.g. `avars:rz.connect`;
- an xformOp stack `XformCommonAPI` cannot represent;
- a prim with no transform at all, such as a `RigExecRoot`;
- nothing selected.

A control that a constraint names in `rigExec:moves` publishes a revised
frame. The gizmo draws at its unconstrained frame and edits the avars
under the constraint; that is a documented limitation, not an error.

## Out of scope

Multi-prim editing and a C++ frame-query export were excluded by the
design. From Maya's tool options, everything that only means something
for polygon components is excluded as well: Preserve UVs, Tweak mode,
Soft Select, Symmetry, Transform Constraints, Smart Duplicate, and
snapping to a live surface, a curve or a point. Maya's Component, Normal,
Along Live Object and Custom axis orientations are excluded for the same
reason.

## Tests

- `bin/run_python_tests.sh` — headless and Qt-free: the frame replica
  against the native evaluator, the channel math for all three tools,
  edit targets and write modes, undo snapshots, screen projection and
  hit testing, the drag rules that turn a mouse move into an `Apply*`
  call, and the per-tool settings defaults.
- `bin/run_testusdview_gizmo.sh` — `tests/testUsdviewGizmo.py` under
  `testusdview` on `examples/ArmShotAnim.usda`. It drives synthetic mouse
  and key events through the gizmo's own projected handle positions and
  asserts what landed on the stage: the avar at the frame, the spline
  knot in the session layer, the Ctrl+Z / Ctrl+Shift+Z round trip, the
  outranked-default warning, `rest:*` in Pivot mode, an xform's op stack,
  and the Maya behaviours (planar handles, `Ctrl` + axis, middle-drag
  repeat, step snap, the view ring, gimbal rings, free rotate, the scale
  ratio rule, Preserve Children and the hotkeys). It prints `RIGEXEC_GIZMO_OK` and
  saves a window grab to `$RIGEXEC_GIZMO_SHOT` when that is set.
