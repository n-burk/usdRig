# Graph editor

Open **RigExec → Animation Editors → Graph Editor** to edit scalar `Ts`
animation splines on the selected prims or properties. The editor supports
`double`, `float`, and `half` curves. Vector attributes use time samples and
do not appear as editable scalar splines.

## Editing

Select curves in the list to isolate them. Drag keys to change time and value;
drag tangent handles to change their slope or length. Moved keys retain their
order. Insert Key adds a knot without replacing the rest of the curve.
New keys use Auto tangents; editing an existing key preserves its tangent setup.

Edits use the shared undo stack when the viewport toolbar is present. They
are authored to the active edit target. Broken tangent state is stored in
the knot's `rigExec` custom data because equal slopes do not imply unified
tangents. Unify copies the outgoing slope to the incoming side.

## Extrapolation

The pre/post Infinity controls apply to the visible curves.

| Setting | USD mode | Result |
|---|---|---|
| Constant | `TsExtrapHeld` | Hold the endpoint value |
| Linear | `TsExtrapLinear` | Extend the endpoint slope |
| Cycle | `TsExtrapLoopReset` | Repeat the curve without a value offset |
| Cycle with Offset | `TsExtrapLoopRepeat` | Offset each repetition to join endpoints |
| Oscillate | `TsExtrapLoopOscillate` | Alternate forward and backward repetitions |

A loop on a single-key spline behaves as Held. Non-finite samples are not
drawn. Frame All fits the keys; a tangent overshoot may extend outside that view.

## Shortcuts

| Key | Action |
|---|---|
| `A` / `F` | Frame all / selected keys |
| `Home` | Frame the stage range |
| `I` | Insert key |
| `Delete` / `Backspace` | Delete selected keys |
| `Escape` | Cancel a drag or clear selection |
| `Ctrl+Z` / `Ctrl+Shift+Z` | Undo / redo |

Shortcuts apply to the editor window and yield to text-entry widgets.
Clamped/Plateau tangents, audio, retiming tools, and Euler filtering are not
implemented. The Qt-free model is tested by `test_graph_model` and
`test_graph_screen`; `bin/run_testusdview_graph.sh` or `.bat` checks the UI.
