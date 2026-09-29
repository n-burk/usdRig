# Viewport tools

**RigExec → Viewport → Viewport Tools** shows the Move, Rotate, and Scale
toolbar. It provides transform handles, editing modes, snapping, settings,
and undo. Tooltips identify each button. See the [graph editor](graph-editor.md)
for editing animation curves.

## Pose and pivot

Pose mode edits animation parameters at the evaluated control frame. Pivot
mode edits rest placement, so the control keeps its animation in a new rest
frame. Rest offsets are relative to the parent frame provider. Moving a rest
frame normally carries its subtree; Preserve Children holds immediate child
rests in world space. Enable it before starting the drag.

Scale is unavailable for rig pivots because rig rest spaces are
orthonormalized. Plain transforms and rig controls have different supported
channel sets; the toolbar reports unavailable operations.

## Preview and commit

A live drag sends in-memory overrides to the evaluator or Hydra transform
overlay. The stage keeps its committed values during preview. Releasing the
drag authors one edit; Escape cancels it. Stage-dependent panels therefore
show committed values until the gesture ends, while the viewport shows the
preview. Undo records the committed operation.

**Write: Animation** writes a spline knot for supported scalar attributes and
a time sample for other attributes. **Write: Default** writes a default value.
Existing splines or samples may override a default; the status label reports
that condition. Edits use the current edit target.

## Handles and settings

Move offers axis, plane, and camera-plane handles. Holding Ctrl while dragging
an axis moves in its perpendicular plane. Rotate offers axis rings, a view
ring, and free rotation; accumulated angles can exceed 180 degrees. Scale
uses the cursor-distance ratio from the handle origin; negative scale can be
disabled in Settings.

Settings include World/Object/Parent orientation, Gimbal rotation, step size,
free rotation, negative-scale prevention, Preserve Children, and handle size.
Axes viewed end-on are disabled when projection makes the drag unstable.
Settings last for the session.

## Snapping

Step Snap quantizes a delta. Grid, point, edge, and surface snapping position
the pivot on a world-space target. Grid/point/edge snap takes precedence over
relative step snap. Surface snapping is selected through the toolbar.
Snap targets come from visible evaluated geometry where supported.

## Shortcuts

| Key | Action |
|---|---|
| `Q`, `W`, `E`, `R` | Select, Move, Rotate, Scale |
| `D` / `Insert` | Toggle pivot editing |
| `+` / `-` | Change handle size |
| Hold `J` | Step snap during a drag |
| Hold `X` | Grid snap |
| Hold `C` / `V` | Edge / vertex snap |
| Hold `B` before dragging | Preserve immediate child rests in pivot mode |
| `Escape` | Cancel the active drag |
| `Ctrl+Z` | Undo |
| `Ctrl+Shift+Z`, `Shift+Z`, `Ctrl+Y` | Redo |

Tool shortcuts yield to text-entry widgets. Shared usdview shortcuts remain
available outside applicable manipulation gestures. On macOS, start an axis
drag before pressing Ctrl because Qt can treat Ctrl-click as a right click.

## Implementation and tests

Qt-free interaction code lives in `gizmoMath.py`, `gizmoScreen.py`,
`gizmoDrag.py`, and `gizmoSnap.py` under `plugin/rigExecUsdview`.
`gizmoUI.py` connects it to usdview and the shared undo stack.
Run the corresponding `test_gizmo_*` suites with the Python test helper;
`bin/run_testusdview_gizmo.sh` or `.bat` checks the viewport path.
