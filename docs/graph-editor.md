# Graph editor (usdview)

Design spec `docs/superpowers/specs/2026-09-01-graph-editor-design.md`.
Added 2026-09-01.

The RigExec usdview plugin adds a Maya-style graph editor: a floating
window that plots the animation curves of the current selection and lets
you move keys, reshape tangents and set interpolation with the mouse.
`RigExec → Graph Editor` opens it, and so does the `Graph…` button on the
[viewport gizmo toolbar](viewport-gizmos.md). There is one editor per
session; asking for it again raises the window you already have.

It is a window rather than a dock because a graph editor is a second
workspace, not a strip of controls — an animator wants it beside the
viewport at whatever size the curves need, and usdview's own panels have
no room to give.

## What it edits

Curves are `Ts` splines, and only `Ts` splines. A curve is a scalar
attribute — `double`, `float`, `half` — that either already carries a
spline or is a rig avar / rest channel that could. Vector attributes
cannot hold a spline at all, so `xformOp:translate` on a plain xform
never appears in the list, however animated it is. That limitation and
what it means for the gizmo toolbar are covered under
[Where the value lands](viewport-gizmos.md#where-the-value-lands).

The curve set follows usdview's selection:

| Selection | Curves shown |
|---|---|
| one or more spline-capable properties in the property browser | exactly those |
| otherwise, the selected prim(s) | every attribute with a spline, then the rig `avars:*` / `rest:*` channels that have none |

Picking a property wins because an artist who clicked `avars:tx` wants
that channel, not the control's other twelve. A property selection of
vectors and tokens only is not a curve choice, so it falls through to the
prims. The unanimated channels are listed on purpose: that is how a
control that has never been keyed gets its first key from here.

Both selections are live. Changing the prim or the property selection
rebuilds the list, and an edit made anywhere else — a gizmo drag, a
script — refreshes the affected curve as its change notice arrives.

## Where the value lands

Everything is authored into the current edit target. usdview starts on
the session layer, so by default a session's graph work disappears when
usdview closes, exactly like the other RigExec panels.

Every write is a **whole spline**, and every one of them goes through
`graphModel.ApplySpline`. The editor replaces the curve rather than
patching a knot, because Ts has no partial-authoring API and because it
makes the undo record trivially correct: one gesture, one snapshot, one
step. A drag writes the whole spline on every mouse move so the viewport
follows the mouse, but each of those writes is recomputed from the spline
the gesture *started* with, never from the previous one, so a slow drag
cannot accumulate rounding or re-clamp its own clamp.

A gesture that leaves a curve with **no keys at all** clears the layer's
spline opinion instead of authoring an empty one, and drops the
attribute spec too once it holds nothing else. An empty spline is still
a spline: it would outrank every weaker opinion and resolve to no value,
so the attribute would read back as `None` and the rig would collapse.
Deleting the last key means "this layer no longer animates the
attribute", which is a cleared opinion rather than an empty curve — so
on a channel the file already animates, deleting all of the session's
keys brings the file's animation back rather than blanking it. The
status line says so when it happens — `session keys cleared; file
animation shows through` — because a delete that leaves keys on screen
otherwise reads as a delete that did not work. The clear happens inside
the same undo bracket as any other write, so one `Ctrl+Z` puts the
spline back exactly.

## Undo

One gesture is one undo step. A drag across forty mouse moves, a tangent
button pressed with three curves selected, a number typed into the Time
field — each pushes a single step. `Ctrl+Z` undoes and `Ctrl+Shift+Z`
redoes, on the same `rigExecUndo` stack the viewport gizmos push onto, so
one history covers both tools and it does not matter which window has
focus.

When the gizmo toolbar is present the editor *adopts* the toolbar's Undo
and Redo actions rather than registering its own. Two actions carrying
the same application shortcut make every `Ctrl+Z` ambiguous, and Qt
resolves an ambiguous shortcut by firing neither.

## Layout

Two rows of controls above a curve list and the canvas.

- **Row one**: `Time` and `Value` fields for the selected keys, blank
  when they disagree and applying to all of them when typed into;
  `Frame All`, `Frame Selected`, `Insert Key`, `Delete`; the `Snap
  Frames` and `Weighted` toggles.
- **Row two**: the tangent-type buttons `Auto` `Spline` `Linear` `Flat`
  `Step`, the `In` / `Out` / `Both` side radios, `Break` and `Unify`,
  and the pre / post `Infinity` combos.
- **Left column**: one row per curve with a colour swatch, the prim
  name, the attribute name and a visibility checkbox. Selecting rows
  isolates those curves, Maya's outliner behaviour; `Show All` clears
  both the isolation and the hidden set.
- **Bottom**: a status line — how many curves, how many are drawn, how
  many keys are selected, the current frame.

The controls are on two rows rather than the one the spec drew because a
single row overflows usdview's window width, and an overflowing Qt row
shrinks its checkboxes under their own labels rather than clipping.

On the canvas, time runs along X in frames and value up Y. Curves are
sampled and drawn as polylines in the curve's colour, dashed where they
are extrapolated; `tx/rx/sx` are red, `ty/ry/sy` green, `tz/rz/sz` blue,
`rspin` yellow and everything else takes a colour from a fixed palette.
Keys are filled squares, yellow when selected. Tangent handles are drawn
for selected keys only, hollow when the tangent is automatic — a hollow
handle is a locked one, and dragging it is what unlocks it. The playhead
is a vertical line at usdview's current frame with the frame number, and
the frames outside the stage's range are shaded.

## Navigating

| Gesture | Effect |
|---|---|
| `Alt` + middle drag | pan |
| `Alt` + right drag | zoom, about the point the drag started |
| mouse wheel | zoom about the cursor |
| `A` | frame all visible curves |
| `F` | frame the selected keys, or all of them if none are selected |
| `Home` | back to the stage's frame range |
| drag on the bottom ruler | scrub usdview's playhead |

Scrubbing goes through usdview's own `setFrame`, so the frame slider, the
frame field and the viewport all follow. Framing frames the *keys*, not
the sampled curve, so a Bezier overshoot can sit outside the plot — Maya
does the same.

## Editing

Every gesture below is one undo step.

- **Select** a key by clicking it; `Shift` or `Ctrl` adds and removes;
  dragging on empty space marquees; `Escape` clears the selection.
- **Move** keys by dragging any selected one. All selected keys move
  together, in time and value. Holding `Shift` after the press
  constrains the move to whichever axis dominates. Times snap to whole
  frames while `Snap Frames` is on, and a key never crosses its
  neighbours — Maya keeps key order.
- **Move without re-picking**: a middle drag moves the selected keys
  from anywhere on the canvas and leaves the selection alone, Maya's
  "move nearest picked key".
- **Insert** a key on every visible curve at the current frame with
  `Insert Key` or `I`. The key takes the value the curve already
  evaluates to there, so inserting never moves the character. A curve
  with no keys at all has nothing to evaluate, so it is keyed from the
  attribute's resolved value instead — this is how an unanimated avar
  gets its first key. Double-clicking on a curve inserts a key at that
  time on that curve alone.
- **Delete** the selected keys with `Delete` or `Backspace`.
- **Type** an exact `Time` or `Value` for the selected keys. `Time`
  moves the whole selection so its earliest key lands on the number,
  since several keys cannot share one time. It uses the number exactly
  and does not honour `Snap Frames`: a typed frame *is* the exact value,
  and only drags snap. A typed frame that is already held by a key
  outside the selection is refused outright, with nothing moved and
  `Frame N already has a key; nothing moved.` on the status line — a
  number in a field gives no hint that it is about to consume another
  key, the way dragging one over it does.
- **Drag a tangent handle** to change its slope. With `Weighted` off the
  handle keeps the length it had and only the angle changes; with
  `Weighted` on, dragging outward lengthens it. Dragging a handle always
  switches that side to a custom tangent, because an automatic side
  would recompute the slope away on the next read.

## Tangent types

The buttons apply to the selected keys, on the side the `In` / `Out` /
`Both` radios name. Note that a "segment" belongs to the key before it,
so the In side of a key means the segment that arrives at it.

| Button | What it does in Ts |
|---|---|
| `Auto` | `TsTangentAlgorithmAutoEase` on the side, and `TsInterpCurve` on the segment |
| `Spline` | a custom slope of `(v_next − v_prev) / (t_next − t_prev)`, Catmull-Rom, on `TsInterpCurve` |
| `Linear` | `TsInterpLinear` on the segment; the tangents are left alone, because linear is a segment mode in Ts, not a tangent |
| `Flat` | a custom slope of zero on `TsInterpCurve` |
| `Step` | `TsInterpHeld` on the segment; again a segment mode, not a tangent |

`Auto`, `Spline` and `Flat` all restore `TsInterpCurve` on the segments
they touch, so a key can go straight from `Step` back to a curve.

**Break** unlocks a key's two tangents so they move independently, and
does it without changing the curve: the resolved slope and width of each
automatic side are written back as custom values, freezing exactly the
shape on screen before handing it to the mouse. **Unify** re-links them
by copying the out slope to the in side. Ts has no broken flag, so
break-ness is recorded in the knot's `customData` under `rigExec` —
Maya remembers a key was broken even while the two sides still happen to
agree, and two sides can be broken and identical at once.

Maya's **Clamped** and **Plateau** are out of scope; ask for `Auto`.

## Infinity

The `Infinity` combos set the pre and post extrapolation of every curve
the editor is showing. Row selection in the curve list *is* isolation, so
what is drawn is what an animator would call selected.

| Maya | Ts |
|---|---|
| Constant | `TsExtrapHeld` |
| Linear | `TsExtrapLinear` |
| Cycle | `TsExtrapLoopReset` |
| Cycle with Offset | `TsExtrapLoopRepeat` |
| Oscillate | `TsExtrapLoopOscillate` |

The two looping names invert what they suggest, so they are easy to map
backwards. `TsExtrapLoopReset` repeats the curve *exactly*, with a
discontinuous join unless the ends already match — that is Maya's Cycle.
`TsExtrapLoopRepeat` repeats it *offset* so the ends meet, which is what
keeps a walk cycle travelling — that is Maya's Cycle with Offset.

Two Ts behaviours to know. A looping infinity on a single-knot spline
behaves as Held, by Ts's own rule. And a `Linear` or looping
extrapolation running off a knot whose tangent algorithm is `None`
samples as NaN; the canvas drops non-finite samples, so such a tail
simply does not draw.

## Hotkeys

`A` frame all · `F` frame selected · `Home` stage range · `I` insert key
· `Delete` / `Backspace` delete keys · `Escape` cancel the live drag, or
clear the selection · `Ctrl+Z` / `Ctrl+Shift+Z` undo and redo.

The keys are read by an application-level event filter installed when the
window opens, not by the widgets, and it claims a key only for events
belonging to the editor's own visible window and only when focus is not
in a text field or spin box.

That filter exists because two of these keys are usdview's. `Escape` is
swallowed by usdview's own event filter to reset focus from the mouse
position, and bare `F` is routed to the 3D viewport's Frame Selected,
both application-wide whichever window is active. The editor's filter is
installed later, and Qt runs the most recently installed application
filter first, so while the graph editor is the focused window those two
keys mean framing and cancelling *here*. usdview's other bare-letter
shortcuts (`I`, `V`, `J`, `W`, `C`) are actions on the main window with
window-level context, so they are inert while this window is active.

## Out of scope

Maya features that need more than splines, or that are secondary to
curve editing: buffer curves, normalized and stacked display, the retime
tool, lattice deform, the region tool, bookmarks, sound, key reduction,
the Euler filter, the Clamped and Plateau tangent types, Step Next, and
channel-box-driven key insertion on attributes that cannot hold a
spline.

Two smaller differences from Maya are known and deliberate. A held
segment draws with no vertical riser, because `Ts.Spline.Sample` splits
the polyline at the discontinuity rather than reporting the jump. And
Frame All frames the keys rather than the sampled curve, so an overshoot
can leave the plot.

## Tests

- `bin/run_python_tests.sh` — headless and Qt-free: `test_graph_model`
  covers curve discovery from property and prim selections, the new-key
  defaults, every edit operation on synthetic splines (values after
  `Eval`, tangent algorithms, interpolation and extrapolation modes),
  frame snapping, neighbour clamping, and the session-layer write with
  its undo; `test_graph_screen` covers the transform round trips, nice
  grid steps, frame-all extents, key and tangent hit-testing, marquee
  containment and drag resolution.
- `bin/run_testusdview_graph.sh` — `tests/testUsdviewGraphEditor.py`
  under `testusdview` on `examples/ArmShotAnim.usda`. It opens the
  editor on the viewport toolbar's undo stack and drives synthetic mouse
  and key events at the pixels the canvas itself reports for its keys
  and tangent handles, asserting what landed on the stage: the curve set
  for a prim and for a property selection, a key drag written into the
  session layer as a snapped whole-frame move, one `Ctrl+Z` restoring
  the pre-drag spline exactly, an inserted key holding the value the
  curve already evaluated to, delete, `Flat` and `Step`, a tangent
  handle drag weighted and unweighted, break and unify, the infinity
  mapping checked by evaluating the stage outside the key range, the
  ruler scrub and marquee selection. It prints `RIGEXEC_GRAPH_OK` and
  saves a window grab to `$RIGEXEC_GRAPH_SHOT` when that is set.
