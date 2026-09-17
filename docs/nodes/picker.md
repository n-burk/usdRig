# ![Picker](../../icons/picker.png) Picker

*A character's control picker panel, shipped as scene data.*

| | |
|---|---|
| **Node type** | `RigExecPicker` |
| **Example** | [picker.usda](../examples/picker.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

The animator's button board: a 2D panel of clickable shapes that
select the rig's controls, flip its switches, and zero its pose. A picker
is not a sidecar file beside the rig — it is `RigExecPicker` prims in the
rig's own layer stack, so it composes, it layers, and an animator can
override one button with an `over` instead of re-authoring anything.
Open it from usdview's **RigExec ▸ Control Picker** menu.

A character's control picker: one tab in the picker panel,
holding one RigExecPickerPanel per sub-tab.

Discovery is by TYPE, never by path. The panel traverses the stage for
RigExecPicker prims and builds a tab for each, so a stage carrying two
characters gets two tabs with no code and no configuration. Order
within the panel follows ui:order, then the prim's namespace order.

A picker names the rig it drives through rigExec:picker:rig. Left
unauthored, buttons resolve against the whole stage, which is right
for the single-character case and wrong the moment there are two.

## How it works

Nothing here is evaluated and nothing here is drawn in the viewport:
the picker runs in no rig phase, the compiler never reads it, and the
schema classes are not imageable. The usdview panel finds pickers BY TYPE
— `Usd.PrimRange` over the stage, `IsA(RigExecPicker)`, pruning any prim
whose type starts with `RigExec` and is not the root or a picker — and
builds one character tab per picker, sorted on `ui:order`
(`plugin/rigExecUsdview/pickerScene.py:181-222`). Each picker reads its
`RigExecPickerPanel` children as sub-tabs and their `RigExecPickerButton`
grandchildren as items, then resolves every button target against the rig
named by `rigExec:picker:rig` — controls and joints only — to decide which
buttons are live (`pickerScene.py:152-178`, `:235-266`). Clicks drive
usdview's own selection; the stage is only written when a switch button
edits its attribute, or the `zero_ctrls` command clears authored pose
avars.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:picker:rig` | The `RigExecRoot` whose controls and joints the buttons resolve against. Unauthored means the whole stage, which is right until a shot holds two characters. | no |
| (child prims) | `RigExecPickerPanel` children — direct children only — become the sub-tabs, in `ui:order`. | no |

## Parameters

### Node parameters

#### `ui:label`

*Type:* `uniform string`. *Default:* `""`.

Tab label. Empty means use the prim name, which is the
usual case: a prim called `Uman` needs no second opinion about
what to call its tab.

#### `ui:order`

*Type:* `uniform int`. *Default:* `0`.

Sort key among sibling pickers. Ties break on prim order.

#### `rigExec:picker:rig`

*Relationship.*

The RigExecRoot this picker drives. Buttons resolve their
control targets inside it. Unauthored means the whole stage.

## Example

`picker.usda` is a two-bone arm with an IK/FK blend and the panel that
drives it: one `RigExecPicker` holding a `Body` panel with a backdrop
decoration, select buttons for the shoulder, elbow, wrist, pole and hand,
an `All FK` button that selects three controls at once, an IK/FK switch on
`ArmParams.avars:ikfk`, and a `Zero Ctrls` command. Nothing in the stage
is animated — the picker is interface, not motion — so open it in usdview
and click rather than scrubbing.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\picker.usda
```

## Tips

- A picker can be parented anywhere except inside rig graph: discovery prunes joints, controls, solvers, movers and weights, but not the `RigExecRoot`, so beside the rig or inside it both work (`pickerScene.py:213-222`).
- Layer the picker rather than editing it: an `over` on one button moves, recolours, relabels or retargets it, and `active = false` on a button, a panel or the whole picker removes it — all asserted in `tests/python/test_picker_scene.py:189-256`.
- Two characters on a stage give two tabs with nothing configured, but name each picker's rig anyway: without it liveness is computed against every control and joint on the whole stage (`pickerScene.py:246-266`, `pickerModel.py:311-322`).

## See also

- [Picker Panel](picker_panel.md)
- [Picker Button](picker_button.md)
- [Control](control.md)

---

[RigExec nodes](../index.md)
