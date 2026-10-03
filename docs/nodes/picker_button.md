# ![Picker Button](../../icons/picker_button.png) Picker Button

*One clickable shape: selects controls, flips a switch, or decorates.*

| | |
|---|---|
| **Node type** | `RigExecPickerButton` |
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

A button is one shape in a panel and one of four things, decided by
what it names rather than by a flag: a SELECT button lists prims in
`rigExec:picker:controls`, a SWITCH edits the one attribute in
`rigExec:picker:attribute`, a COMMAND runs the named action in
`rigExec:picker:command`, and a button naming none of them is DECORATION —
a backdrop, a silhouette, a label. Everything else on the class is
appearance and visibility: outline, colours, text, draw order, and
which half of an IK/FK pair a button belongs to.

One button. Selects the prims in rigExec:picker:controls, or
edits the attribute in rigExec:picker:attribute, or neither -- a
button with no target of either kind is a decoration: the body
silhouette, a backdrop, a label.

LIVENESS is the resolve result, not a flag. A relationship whose
targets have all gone is an empty target list, and the panel draws
that button as unavailable. USD remaps the targets through renames
and reparents on its own, which a path string in a sidecar file
cannot do, so a control that MOVES keeps its button and only a
control that is DELETED loses one.

## How it works

usdview reads buttons when the panel opens and after any change to
the prims; no rig phase touches them. Liveness is the resolve result, not
a flag: `pickerScene` turns the relationships into target paths and the
model marks a selecting or switching button live only when every target —
or, for a switch, the prim owning the attribute — is a `RigExecControl` or
`RigExecJoint` inside the rig named by `rigExec:picker:rig`, or anywhere on
the stage when the picker names none (`pickerModel.py:89-98`,
`pickerScene.py:235-243`, `:246-266`); a command button the panel
implements is live on its own (`pickerModel.py:89-90`). A button that
resolves to nothing is not drawn at all, though `coverage()` still counts
it for the panel's status line (`pickerModel.py:222-223`, `:296-306`,
`pickerUI.py:804-817`). A click selects the targets in order, or cycles the
switch's attribute one step and writes it back as a float
(`pickerUI.py:819-838`, `:984-998`); the hit test is the button's bounding
box rather than its drawn outline, and `ui:size` is clamped up to 4 units
so a fingertip button is still catchable (`pickerModel.py:23`, `:41-42`,
`:240-262`).

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:picker:controls` | The `RigExecControl` or `RigExecJoint` prims this button selects, in order. | no |
| `rigExec:picker:attribute` | The single attribute a switch edits, as a property path on a control or joint. Mutually exclusive with `rigExec:picker:controls`; on a plain click the attribute wins. | no |
| `rigExec:picker:modeDial` | The attribute `rigExec:picker:mode` is tested against — the limb's IK/FK dial. | only with `rigExec:picker:mode` |

## Parameters

### Node parameters

#### `ui:position`

*Type:* `uniform float2`. *Default:* `(0, 0)`.

Top-left corner in panel units.

#### `ui:size`

*Type:* `uniform float2`. *Default:* `(10, 10)`.

Width and height in panel units.

#### `ui:depth`

*Type:* `uniform float`. *Default:* `0`.

Painter's-algorithm order; higher draws later, on top.

#### `ui:rotation`

*Type:* `uniform float`. *Default:* `0`.

Degrees clockwise about the button's own centre.

#### `ui:shape`

*Type:* `uniform token`. *Default:* `"rectangle"`.

Valid values: `rectangle`, `roundedRectangle`, `circle`, `ellipse`, `hexagon`, `triangle`, `trapezoid`, `polygon`, `bezier`, `widgetControl`, `slider`.

Outline. `polygon` reads ui:polygon, `bezier` reads
ui:bezier, and every other token is generated from ui:size.

#### `ui:roundness`

*Type:* `uniform float`. *Default:* `0`.

Corner radius fraction for roundedRectangle, 0 to 1.

#### `ui:polygon`

*Type:* `uniform float2[]`. *Default:* `[]`.

Outline vertices for ui:shape = polygon, panel units.

#### `ui:bezier`

*Type:* `uniform float2[]`. *Default:* `[]`.

Cubic control points for ui:shape = bezier, THREE per
knot -- on-curve point, out-tangent, in-tangent -- flattened,
with each span completed by the following knot's in-tangent.

#### `ui:fill`

*Type:* `uniform color4f`. *Default:* `(0.5, 0.5, 0.5, 1)`.

Interior colour, linear, alpha premultiplied by the view.

#### `ui:stroke`

*Type:* `uniform color4f`. *Default:* `(0, 0, 0, 0.8)`.

Outline colour.

#### `ui:strokeWidth`

*Type:* `uniform float`. *Default:* `1`.

Outline width in panel units.

#### `ui:text`

*Type:* `uniform string`. *Default:* `""`.

Label drawn inside the button.

#### `ui:fontSize`

*Type:* `uniform float`. *Default:* `8`.

Label size in panel units.

#### `ui:bold`

*Type:* `uniform bool`. *Default:* `false`.

Label weight.

#### `ui:textAlign`

*Type:* `uniform token`. *Default:* `"center"`.

Valid values: `left`, `center`, `right`.

Horizontal label alignment inside the button.

#### `ui:textColor`

*Type:* `uniform color4f`. *Default:* `(0, 0, 0, 1)`.

Label colour.

#### `ui:valueColor`

*Type:* `uniform color4f`. *Default:* `(0, 0, 0, 1)`.

Colour of the VALUE a switch or dial displays, as
distinct from its label: `Foot` is drawn in ui:textColor and the
`IK` beside it in this.

#### `ui:direction`

*Type:* `uniform token`. *Default:* `"top"`.

Valid values: `top`, `bottom`, `left`, `right`.

Which edge a trapezoid or triangle points at.

#### `ui:slopeLeft`

*Type:* `uniform float`. *Default:* `0`.

Trapezoid left-edge inset, as a fraction of ui:size width.

#### `ui:slopeRight`

*Type:* `uniform float`. *Default:* `0`.

Trapezoid right-edge inset, as a fraction of ui:size width.

#### `ui:alternate`

*Type:* `uniform bool`. *Default:* `false`.

This button shares a slot with another and is the one
hidden at rest. Where rigExec:picker:mode is authored that is the
authority and this is ignored: a stacked IK/FK pair is a mode
pair, not an alternate.

#### `rigExec:picker:command`

*Type:* `uniform token`. *Default:* `""`.

A named action this button runs, instead of selecting or
editing. The panel implements a fixed set and ignores the rest, so
a picker may carry actions a given host cannot perform without
those buttons pretending to work.

#### `rigExec:picker:controls`

*Relationship.*

The prims this button selects, in order. Targets may be
RigExecControls or directly-posed RigExecJoints; the picker treats
both as selectable because the avar editor edits both.

#### `rigExec:picker:attribute`

*Relationship.*

The single attribute this button edits, for a switch or a
dial. Mutually exclusive with rigExec:picker:controls.

#### `rigExec:picker:attributeLabels`

*Type:* `uniform token[]`. *Default:* `[]`.

Labels for the values rigExec:picker:attribute cycles
through, low to high, e.g. ["IK", "FK"]. The button shows the
label matching the attribute's CURRENT value; it never caches the
value it was authored with.

#### `rigExec:picker:mode`

*Type:* `uniform token`. *Default:* `""`.

Valid values: `, `, `, `.

Show this button only while its limb is in this mode.
Empty means always shown, which is right for anything that is not
half of an IK/FK pair.

#### `rigExec:picker:modeDial`

*Relationship.*

The attribute rigExec:picker:mode is tested against. Only
meaningful when that token is authored.

## Example

`picker.usda` has one of each kind in a single panel: `b_Shoulder`
selects one control, `b_Arm` selects three at once, `b_ArmIkFk` switches
`ArmParams.avars:ikfk` between the labels `FK` and `IK`, `b_Zero` runs the
`zero_ctrls` command, and `backdrop` names nothing and is decoration. The
elbow/wrist pair carries `rigExec:picker:mode = "fk"` and the pole/hand
pair `"ik"`, both dialled off `ArmParams.avars:ikfk`, so half the limb's
buttons swap out when the switch is clicked.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\picker.usda
```

## Tips

- Leave `ui:text` unauthored on a command button. The panel falls back to the command's own title (`Zero Ctrls`) and then dispatches on that LABEL, so a friendlier caption makes the button inert (`pickerScene.py:130-131`, `pickerModel.py:28`, `:77-80`).
- Author a switch's attribute on a control or a joint — a custom `avars:ikfk` on a params control, connected onward to the blend — because an attribute on a solver prim fails the liveness test and the button is never drawn (`pickerModel.py:91-93`, `:222-223`).
- `rigExec:picker:mode` is ignored without `rigExec:picker:modeDial`, and a dial resting between 0.001 and 0.999 counts as neither mode, so both halves stay reachable mid-handover (`pickerScene.py:140-142`, `pickerUI.py:950-956`).

## See also

- [Picker](picker.md)
- [Picker Panel](picker_panel.md)
- [Blend Point Frames](blend_point_frames.md)

---

[UsdRig](../index.md)
