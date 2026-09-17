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

The controls are **glyphs, not words** (`gizmoIcons`), and each one's
tooltip names it and says what it does. Words made the row as wide as the
platform's UI font: on Windows at 10pt it wanted 856 px against the ~600 the
default viewport gives, which put Snap, Undo, Redo, Settings and Graph into
that same chevron. Glyphs are the width we choose, so the whole row fits
everywhere. The one exception is the `Snap:` button, which keeps its text
because its label reports the mode in force rather than naming the button.

The artwork in `plugin/rigExecUsdview/icons/` is generated — one image per
glyph through the Codex CLI's built-in image generation, then normalised into
a set by [`tools/bakeGizmoIcons.py`](../../tools/bakeGizmoIcons.py), which keys
the black ground out to alpha and brings every glyph to a common extent and a
common stroke weight. That script's docstring carries the prompt, so the set
can be regenerated rather than only admired. `gizmoIcons` tints the white art
at draw time, which is why one file reads on both the dark toolbar and the
blue highlight behind a checked tool — and it falls back to drawing each glyph
with `QPainterPath` if a file is missing, so a broken install is a plainer
toolbar rather than a row of blank buttons.

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
compensation. A rest offset is relative to the parent frame provider, so
a Pivot drag carries the joint's whole subtree with it; hold `B` (or tick
Preserve Children) to hold the immediate children on their own world
rests instead, which is how you re-proportion one bone without dragging
the rest of the limb. Scale is unavailable in Pivot mode on a rig prim because
the evaluator orthonormalizes rest spaces, so a scale written there would
be silently discarded; the toolbar says so rather than accepting a drag
that does nothing.

In **Pose** mode the gizmo sits at the evaluated frame. For a rig prim
that is `avars * rest * parentRest⁻¹ * parentPosed`, placed by the asset
root's world transform (`libs/rigExec/computations.cpp`), so a rest
offset or an animated parent is already accounted for. Its axes are the
target's local frame, the one the avars are expressed in, unless the Axis
Orientation option asks for another.

In **Pivot** mode it sits on the rest frame itself —
`orthonormalize(compose(rest:t, rest:r) * rest:space) * parentRest`,
placed by the same asset transform. `computeRestFrame` composes those
channels against `rest:space` and then carries the result into the
namespace ancestor's rest frame, so a rest frame is relative to the
parent's REST: it owes nothing to the parent's pose, and nothing to a
solver posing the joint. The manipulator is therefore anchored on the
frame its own channels define, which is the point of a pivot mode — drag
it and the value you are editing moves with it, whatever the character
happens to be doing at the current frame. Expect it to sit away from the
posed geometry whenever the rig is animated off its rest pose.

## When the value lands

While the mouse is down, nowhere. A drag in progress is not an edit to the
document — it is a question the artist has not finished asking — so the
manipulator feeds its uncommitted values straight to Hydra and the stage still
holds the value it had before the drag started. Letting go authors the result,
once.

That is visible, and it is meant to be. The [graph editor](graph-editor.md),
the [Layer Opinions](../../plugin/rigExecUsdview/layerOpinionsUI.py) panel and
anything else watching the stage show the pre-drag value for the length of the
gesture and the committed one the moment it ends. The viewport, the
manipulator's own handles, and the guides all follow every mouse sample, which
is the only place the in-between values were ever wanted.

It also means an abandoned drag — `Esc`, or releasing outside — leaves the
layer exactly as it found it, because there is nothing to take back.

A rig control's preview goes through the evaluator: the dragged avars are
supplied as evaluation-time overrides
(`RigExecRigEvaluator::SetInteractiveOverrides`), the rig re-runs, and the
generation Hydra draws is the previewed one. A plain `Xformable` has no rig to
re-run, so its preview is a transform override in the Hydra chain
(`RigExecXformOverrideSceneIndex`), which carries its children the way an
authored edit would. Both are in memory only; the design note is
[docs/superpowers/specs/2026-09-10-hydra-preview-manipulation-design.md](../superpowers/specs/2026-09-10-hydra-preview-manipulation-design.md).

## Where the value lands

**Write: Animation** authors a spline knot at usdview's current frame for
a double attribute and a time sample for anything else, by the same rule
the volume weight panel uses (`volumeWeightUI.SetAtTime`). **Write:
Default** authors the attribute's default instead. Either way it is authored
on release, so a drag across fifty mouse samples leaves one knot behind rather
than fifty rewrites of one.

The knot Animation mode authors is Maya's default new key: Auto tangents
on both sides and a curve segment after it. It comes from the same
`graphModel.AuthorKnot` the graph editor's Insert Key uses, so a gizmo
drag and a graph insert produce the same key, and re-keying a channel an
artist has already shaped keeps that shape. "Anything else" is mostly the
vector ops: `Ts` splines are scalar-only in this USD build, so a rig
avar (`double`) gets a knot while `xformOp:translate` on a plain xform
(`double3`) gets a time sample, because USD cannot store a spline on it
at all. That is why the [graph editor](graph-editor.md) lists a control's
avars and not a plain xform's ops — there is no curve to draw.

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
panels can adopt later. It was already one step before the manipulator
deferred its authoring, and it stays one now for a simpler reason: the drag
authors once, so there is one edit to record. `Ctrl+Z` undoes; `Ctrl+Shift+Z`, `Shift+Z`
(Maya's redo) and `Ctrl+Y` all redo. They are application shortcuts, so
they work wherever focus is in the window. The toolbar's Undo and Redo
buttons name the step they would reverse on their tooltips, and both go
grey while a drag is live, because undoing the edit a held mouse button
is still writing would leave the two disagreeing.

Undo restores the exact attribute spec in the layer the drag edited —
default, spline and time samples alike — and removes the property spec
entirely when the drag was what created it, so an undone edit puts back
the value you had rather than a zero. What it does not remove are the
empty `over` prim specs the first drag on a prim had to create to hold
that property: they stay in the session layer, contribute nothing to
composition and are harmless. `Escape` during a drag aborts it and
pushes nothing. The stack is cleared when the stage is replaced, because
the specs it holds belong to layers that are gone.

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
| `X` (hold) | snap the world pivot to the grid (Move; Rotate: Gimbal ring only) |
| `C` (hold) | snap the pivot to the nearest edge under the cursor (Move) |
| `V` (hold) | snap the pivot to the nearest vertex under the cursor (Move) |
| `B` (hold) | Preserve Children for the drag: hold immediate child joints on their world rests (Pivot) |
| `Escape` | abort the drag in progress |
| `Ctrl+Z` | undo |
| `Ctrl+Shift+Z`, `Shift+Z`, `Ctrl+Y` | redo |

The tool keys act while the pointer is over the viewport and no text
field has keyboard focus, so typing a prim name into the search box
cannot switch tools out from under you. Undo and redo work anywhere in
the window. `Escape` aborts a drag from anywhere in the window too —
including with the cursor off the viewport — but it goes through the
same typing gate as the tool keys, so it does nothing while a text field
has keyboard focus: that `Escape` belongs to the field. Move the focus
off the field to get it back.

`B` must be held BEFORE the drag starts: the undo record is opened with
the set of channels the drag may write, and arming compensation once that
is open would move the children outside it. Pressed mid-drag it arms the
next one.

`J` is relative step snapping while `X`, `C` and `V` snap the world
pivot to the grid, an edge and a vertex instead. All of that lives under
Snapping below, including why `X` still outranks `J` when both are held.

Four keys are shared with usdview rather than taken from it. `J` is
usdview's Toggle Framed View; since Maya's `J` only means anything while
dragging, a live drag claims it and the rest of the time it still frames
the view. `C` (Auto Compute Clipping Planes) and `V` (Show USD
Validation) work the same way, except that outside a drag they arm edge
and point snapping only while the Move tool is active with a target —
everywhere else they stay usdview's. `Escape` is usdview's own focus
reset, and is likewise claimed only while a drag is live. `W` is declared in usdview as Watch Window,
but that action is disabled and connected to nothing, so it was free to
take.

### Known differences from Maya

- **`Ctrl` + left-click on macOS** is turned into a right-button press by
  Qt before anything sees it, so Ctrl-clicking an axis cannot start a
  drag there. The gesture that works everywhere is to grab the axis
  first and then hold `Ctrl`.

## Snapping

The Move tool snaps the way Maya's does: four modes, each of which puts
the manipulator PIVOT on a world-space target — never the grabbed handle
and never the cursor.

- **Grid** — the world grid through the origin, spaced `Grid Size` apart.
- **Point** — the nearest vertex or curve CV of the prim under the
  cursor, within 12 pixels of it.
- **Edge** — the nearest point on the nearest edge segment of that prim:
  mesh edges, or consecutive CVs of a curve.
- **Surface** — the point on the prim under the cursor where the cursor
  ray hits it.

A mode is armed either for one drag, by holding `X` (grid), `V` (point)
or `C` (edge), or stickily, from the `Snap:` button on the toolbar —
bare `Snap` when off — mirrored as `Snap To` in the Tool Settings
window. Surface has no hold key, Maya gives it none, so it is
dropdown-only. A held key outranks the sticky mode, and when several
holds are down the more specific target wins — Point over Edge over Grid
— regardless of the order they were pressed; releasing the winner falls
through to the next hold still down, then to the sticky mode.

The grabbed handle constrains where on the target the pivot may land. An
axis drag slides along its axis and stops level with the target; a plane
handle — or an axis with `Ctrl`, which drags in a plane — lands in its
plane; the centre handle lands on the target unchanged. A snapped drag
is still one undo step, exactly like an unsnapped one.

With a point, edge or surface snap armed and nothing suitable under the
cursor, the object does not move. That is Maya's behaviour, not a bug:
there is no fallback to free dragging. (The grid is infinite, so Grid
always has somewhere to land.) Rig-driven geometry is never a target at
all: its authored points are not what the renderer draws, so Point and
Edge refuse it, and Surface does too wherever the drag itself would
re-pose the geometry under the ray. On a scene whose only geometry is
one rigged character, dragging that rig's controls therefore leaves
these modes with nothing to land on, and the status line says so rather
than a bare complaint.

Rotate's grid is a different thing: an absolute degree grid of `Step
Size`, on a Gimbal ring only. The dragged Euler channel lands on a
multiple of the step. It is Gimbal-ring-only because that is the one
rotate route that writes a single channel; a world-axis ring or the
free-rotate ball reaches the drawn rotation by moving all three
channels, so there is no single resulting channel to quantise and Grid
stays inert there. Point, edge and surface snapping are Move-only:
holding one while rotating is inert. Scale gains nothing — its Step Snap
already quantises the resulting channel, which is the only snap Maya
offers it.

`Grid Size` lives in the Tool Settings window beside `Snap To`, and is
session-wide rather than per-tool: resetting the Move tool does not move
the world grid. Rotate's grid ignores it — degrees already have `Step
Size`. `J` is the other half of this story: relative step snapping, the
movement advancing in whole steps so an object that started off the grid
stays off it (Maya's Discrete Move), while `X` snaps the WORLD pivot
however the drag started — under a posed parent the world position lands
on round numbers while the channel values do not, which is the opposite
of what the old channel-space `X` did. Holding both, `X` still outranks
`J`, because it fully determines where the object ends up and the
relative step then says nothing. (`Target`'s `snapAbsolute` parameter
stays — it is the correct name for channel-absolute and remains tested —
but nothing in the UI binds it any more.)

While a snap is armed, the status line says which and what happened:
`grid: world` for the Move grid, or `grid: relative (frame not
world-aligned)` when the handle points nowhere near a world axis and the
travel from the pivot is quantised instead; `grid: 15 deg` for Rotate's
grid; `snap: Point`, `snap: Edge` or `snap: Surface` for a landing; and,
when there is nothing to land on or the request makes no sense, the
reason — `snap: no target`, `snap: <prim> is rig-deformed`, `snap: Point
is Move only`, `snap: Grid needs a Gimbal ring`. The landing also gets
an orange marker: a diamond and crosshair, with a short normal tick for
a surface, the two halves of the picked segment for an edge, and
world-axis ticks for the grid, plus a faint leader joining it to the
handle while the two differ. The marker shows on hover too while a mode
is armed, so holding `V` lights up what the pivot would land on before
anything is grabbed.

Out of scope, each a follow-up: nominating one construction surface
(Maya's Make Live), snap to view planes, to a projected centre or to a
network, Snap Align Objects, face-centre snapping, multi-prim snapping,
and reorienting the object to the surface normal on a Move snap (Maya
does not do that either).

To try it: `bin/run_python_tests.sh` covers the snap maths, the drag
rules (precedence, the per-handle constraint, the no-target no-write
rule) and the per-tool choices; `bin/run_testusdview_gizmo.sh` drives
sticky and held snaps at projected pixels on `examples/ArmShotAnim.usda`
and asserts what landed, including that usdview's own `C` and `V` still
fire outside a Move drag.

## When there is no gizmo

The status label always says why, rather than leaving an empty viewport
to interpret:

- **in Pose mode only** — a joint posed by a solver (named in some
  solver's `rigExec:joints`), or a prim with a connected or authored
  `posed:space`, or a descendant of either: the evaluator ignores avars
  for those. Pivot mode still works on all of them, because none of it
  touches `rest:t/r` — the evaluator overrides `computePointFrame` alone,
  and a `RigExecTwoBoneIk` measures its bone lengths *from* the bound
  joints' rest frames;
- an avar with an authored connection, e.g. `avars:rz.connect` (in Pivot
  mode, a connected `rest:` channel);
- an xformOp stack `XformCommonAPI` cannot represent;
- a prim with no transform at all, such as a `RigExecRoot`;
- nothing selected.

A control that a constraint names in `rigExec:moves` publishes a revised
frame. The gizmo draws at its unconstrained frame and edits the avars
under the constraint; that is a documented limitation, not an error.

## Editing a pivot under an IK solver

Dragging the pivot of a joint bound to a `RigExecTwoBoneIk` is how you
re-proportion the limb: the solver measures each bone between the bound
joints' rest origins on every evaluation, so moving a rest changes the
bone length and the solve follows immediately, with no recompile.

Three things about it are worth knowing before the viewport surprises
you:

- **Only the middle joint visibly moves.** The root joint's position is
  pinned by `rigExec:rootControl` and the end joint's by
  `rigExec:effectorControl`, so dragging either one's pivot re-proportions
  the limb around it rather than moving the joint you grabbed. The knee
  is where you see the result.
- **Rotation does not reach the solve.** Only rest *origins* are
  measured, so `rest:r` cannot change a bone length. It still sets the
  joint's bind orientation, which is why the Rotate tool stays available.
- **There is no bone-length attribute.** `rigExec:upperLength` and
  `rigExec:lowerLength` were removed from the schema; the kernel measures
  both bones from the rests of the joints in `rigExec:joints` on every
  evaluation. So a pivot drag always reaches the solve and the tool
  carries no advisory. They used to be an opt-out that froze a bone,
  which read as a broken middle joint rather than as a mode switch --
  and the property editor could not reveal it either, showing the schema
  fallback of `1` while the solve ran on the measured value. To tune a
  bone, author `rigExec:upperLengthOffset` / `rigExec:lowerLengthOffset`,
  deltas on the measured length that keep it tracking the rests.

## Out of scope

Multi-prim editing and a C++ frame-query export were excluded by the
design. From Maya's tool options, everything that only means something
for polygon components is excluded as well: Preserve UVs, Tweak mode,
Soft Select, Symmetry, Transform Constraints and Smart Duplicate.
Maya's Component, Normal, Along Live Object and Custom axis orientations
are excluded for the same reason. The snapping follow-ups — Make Live,
view-plane and network snaps, Snap Align, face centres, multi-prim
snapping — are listed under Snapping above.

## Tests

- `bin/run_python_tests.sh` — headless and Qt-free: the frame replica
  against the native evaluator, the channel math for all three tools,
  edit targets and write modes, undo snapshots, screen projection and
  hit testing, the snap maths and the drag rules that turn a mouse move
  into an `Apply*` call (including the snap precedence and throttle),
  and the per-tool settings defaults.
- `bin/run_testusdview_gizmo.sh` — `tests/testUsdviewGizmo.py` under
  `testusdview` on `examples/ArmShotAnim.usda`. It drives synthetic mouse
  and key events through the gizmo's own projected handle positions and
  asserts what landed on the stage: the avar at the frame, the spline
  knot in the session layer, the Ctrl+Z / Ctrl+Shift+Z round trip, the
  outranked-default warning, `rest:*` in Pivot mode, an xform's op stack,
  and the Maya behaviours (planar handles, `Ctrl` + axis, middle-drag
  repeat, step snap, the view ring, gimbal rings, free rotate, the scale
  ratio rule, Preserve Children, the hotkeys and the snap modes — sticky
  and held landings, the status clause, one undo step). It prints
  `RIGEXEC_GIZMO_OK ... snapping` and
  saves a window grab to `$RIGEXEC_GIZMO_SHOT` when that is set.
