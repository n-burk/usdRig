# ![Picker Panel](../../icons/picker_panel.png) Picker Panel

*One sub-tab of a picker: a 2D canvas of buttons.*

| | |
|---|---|
| **Node type** | `RigExecPickerPanel` |
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

A picker's sub-tab — Body, Face, Hands — and the coordinate space its
buttons are placed in. The origin is the TOP LEFT with y increasing
downward, matching every picker authoring tool and the Qt widget the panel
is drawn into, so positions copied out of a conventional picker land where
they did there. A panel holds nothing but `RigExecPickerButton` children
and a background colour.

One sub-tab of a RigExecPicker: Body, Face, and so on. Holds
RigExecPickerButton children in a 2D coordinate space whose origin is
the top left, y increasing downward, matching every picker authoring
tool and the Qt widget this is drawn into.

## How it works

Read in usdview only, in no rig phase: `pickerScene.read()` takes the
picker's children that are `RigExecPickerPanel`, sorts them on `ui:order`
with namespace order breaking ties, and makes one sub-tab each; its own
children that are `RigExecPickerButton` become that tab's items
(`plugin/rigExecUsdview/pickerScene.py:152-178`). `ui:background` is
converted from the schema's 0-1 colour to 8-bit for Qt, and `ui:label`
falls back to the prim name. `ui:size` is the declared extent, but the
view fits the tab to the BUTTONS' bounding box plus a 12-unit margin when
the panel has any, and only falls back to `ui:size` for an empty panel
(`pickerModel.py:325-349`).

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| (child prims) | `RigExecPickerButton` children — direct children only — are this tab's items. | no |
| (none) | A panel names nothing else; it is placed by being a child of its `RigExecPicker`. | - |

## Parameters

### Node parameters

#### `ui:label`

*Type:* `uniform string`. *Default:* `""`.

Sub-tab label; empty means use the prim name.

#### `ui:order`

*Type:* `uniform int`. *Default:* `0`.

Sort key among sibling panels.

#### `ui:size`

*Type:* `uniform float2`. *Default:* `(400, 600)`.

Panel extent in picker units, before the view's fit scale.

#### `ui:background`

*Type:* `uniform color4f`. *Default:* `(0.16, 0.16, 0.16, 1)`.

Panel fill behind every button.

## Example

`picker.usda` carries a single `Body` panel, 200 by 260 picker units on
a dark grey ground, holding nine buttons: a backdrop, two mode-filtered
pairs, a multi-control `All FK` button, an IK/FK switch and a `Zero Ctrls`
command. Its buttons happen to fill the declared `ui:size` exactly, so the
fitted view and the declaration agree.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\picker.usda
```

## Tips

- Author button positions as if `ui:size` were the frame, but do not rely on it: the tab is fitted to the buttons, so a panel split out of a shared canvas still reads (`pickerModel.py:325-349`).
- y grows DOWNWARD in panel units — a button at y=24 is near the top of the tab, not the bottom (`schema.usda:2379-2382`).
- `ui:label` is the tab caption and empty means the prim name, so a panel named `Body` needs no label at all (`pickerScene.py:164`).

## See also

- [Picker](picker.md)
- [Picker Button](picker_button.md)

---

[UsdRig](../index.md)
