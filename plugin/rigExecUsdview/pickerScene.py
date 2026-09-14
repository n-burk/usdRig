"""Read control pickers out of the stage.

The picker used to be a sidecar JSON file found by guessing at the
stage's filename. Now it is scene data: `RigExecPicker` prims composed
into the rig's layer stack, discovered BY TYPE, so a stage carrying three
characters gets three pickers with no configuration anywhere.

What that buys, beyond tidiness:

  * an animator overrides one button with an `over` on one prim, and
    `active = false` removes a button, both in a layer they own;
  * button targets are relationships, so USD remaps them through a
    rename or a reparent, where a baked path string would go dead;
  * liveness is just "did the targets resolve against this stage".

This module turns those prims into the same `Panel` and `Button` objects
`pickerModel` already builds from JSON, so the drawing, hit-testing and
selection code has exactly one path through it.
"""
from pxr import Sdf, Tf, Usd

import pickerModel

# Resolved once. `IsA` takes a TfType, and looking it up per prim during
# a traversal of a few thousand is the kind of thing that makes a panel
# feel slow for no reason.
PICKER_TYPE = Tf.Type.FindByName("RigExecPicker")
RIG_TYPE = Tf.Type.FindByName("RigExecRoot")
CONTROL_TYPE = Tf.Type.FindByName("RigExecControl")
JOINT_TYPE = Tf.Type.FindByName("RigExecJoint")
PANEL_TYPE = Tf.Type.FindByName("RigExecPickerPanel")
BUTTON_TYPE = Tf.Type.FindByName("RigExecPickerButton")
# The rig root is a scope, not graph: a picker may sit inside it.
PICKER_ROOT_SAFE = "RigExecRoot"


def _v(prim, name, default=None):
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        return default
    value = attr.Get()
    return default if value is None else value


def _rgba(value, default=(128, 128, 128, 255)):
    """Schema colours are 0-1; Button and QColor want 0-255."""
    if value is None:
        return list(default)
    return [int(round(max(0.0, min(1.0, float(c))) * 255.0))
            for c in tuple(value)[:4]]


def _pairs(value):
    return [(float(p[0]), float(p[1])) for p in (value or [])]


def _button_record(prim, panel_id):
    """One RigExecPickerButton as the record `Button` expects."""
    pos = _v(prim, "ui:position", (0.0, 0.0))
    size = _v(prim, "ui:size", (10.0, 10.0))

    controls = prim.GetRelationship("rigExec:picker:controls")
    targets = [str(p) for p in controls.GetTargets()] if controls else []
    # `objects` is what separates a DEAD button from a DECORATION: a
    # backdrop names nothing, while a button whose control was deleted
    # named something that is no longer there. The authored-ness of the
    # relationship is the only thing that tells those apart.
    objects = []
    if controls and controls.HasAuthoredTargets():
        objects = [prim.GetName()]

    attr_target = None
    attr_rel = prim.GetRelationship("rigExec:picker:attribute")
    attr_paths = attr_rel.GetTargets() if attr_rel else []
    if attr_paths:
        path = attr_paths[0]
        attr_target = {
            "path": str(path.GetPrimPath()),
            "attr": path.name,
            # Labels are authored in the rig's own order, so nothing here
            # is inverted; the converter reversed them once at bake time
            # rather than shipping a flag to be re-interpreted.
            "enum": [str(t) for t in
                     (_v(prim, "rigExec:picker:attributeLabels") or [])],
            "invert": False,
        }

    dial = None
    dial_rel = prim.GetRelationship("rigExec:picker:modeDial")
    dial_paths = dial_rel.GetTargets() if dial_rel else []
    if dial_paths:
        dial = str(dial_paths[0])

    # The bezier outline is flattened THREE points to a knot on the way
    # in, because USD has no ragged array. Three, not four: a picker knot
    # is (on-curve, out-tangent, in-tangent), with the following knot's
    # in-tangent completing each span. Regrouping by four silently shears
    # every silhouette, and it still draws, which is what makes it worth
    # saying out loud here.
    flat = _pairs(_v(prim, "ui:bezier"))
    bezier = [flat[i:i + 3] for i in range(0, len(flat) - 2, 3)]

    command = str(_v(prim, "rigExec:picker:command", ""))
    if command:
        kind = "commandButton"
    elif attr_target:
        kind = "attributeButton"
    else:
        kind = "selectButton"

    return {
        "id": prim.GetName(),
        "type": kind,
        "parent": panel_id,
        "z": float(_v(prim, "ui:depth", 0.0)),
        "x": float(pos[0]), "y": float(pos[1]),
        "w": float(size[0]), "h": float(size[1]),
        "shape": str(_v(prim, "ui:shape", "rectangle")),
        "roundness": float(_v(prim, "ui:roundness", 0.0)),
        "direction": str(_v(prim, "ui:direction", "top")),
        "leftSlope": float(_v(prim, "ui:slopeLeft", 0.0)),
        "rightSlope": float(_v(prim, "ui:slopeRight", 0.0)),
        "polygon": _pairs(_v(prim, "ui:polygon")),
        "bezier": bezier,
        "rotation": float(_v(prim, "ui:rotation", 0.0)),
        "alternate": bool(_v(prim, "ui:alternate", False)),
        "fill": _rgba(_v(prim, "ui:fill")),
        "stroke": _rgba(_v(prim, "ui:stroke"), (0, 0, 0, 200)),
        "strokeWidth": float(_v(prim, "ui:strokeWidth", 1.0)),
        "text": (str(_v(prim, "ui:text", ""))
                 or command.replace("_", " ").title()),
        "fontSize": float(_v(prim, "ui:fontSize", 8.0)),
        "bold": bool(_v(prim, "ui:bold", False)),
        "hAlign": str(_v(prim, "ui:textAlign", "center")),
        "textColor": _rgba(_v(prim, "ui:textColor"), (0, 0, 0, 255)),
        "valueColor": _rgba(_v(prim, "ui:valueColor"), (0, 0, 0, 255)),
        "objects": objects,
        "targets": targets,
        "attrTarget": attr_target,
        # A mode with no dial cannot be tested, so it is not a mode.
        "mode": (str(_v(prim, "rigExec:picker:mode", "")) or None
                 if dial else None),
        "dial": dial,
    }


def _order(prim):
    """Sort key: explicit ui:order first, then namespace order."""
    return int(_v(prim, "ui:order", 0) or 0)


def read(picker_prim, live_paths=None):
    """One RigExecPicker prim as a `pickerModel.Picker`."""
    panels, items = [], []
    # GetChildren() filters on the default predicate, which is what
    # makes `active = false` on a panel or a button simply remove it.
    children = [c for c in picker_prim.GetChildren()
                if c.IsA(PANEL_TYPE)]
    for panel in sorted(children, key=_order):
        panel_id = str(panel.GetPath())
        size = _v(panel, "ui:size", (400.0, 600.0))
        panels.append({
            "id": panel_id,
            "label": str(_v(panel, "ui:label", "")) or panel.GetName(),
            "x": 0.0, "y": 0.0,
            "w": float(size[0]), "h": float(size[1]),
            "fill": _rgba(_v(panel, "ui:background"), (41, 41, 41, 255)),
        })
        items.extend(
            _button_record(b, panel_id) for b in panel.GetChildren()
            if b.IsA(BUTTON_TYPE))

    picker = pickerModel.Picker({"panels": panels, "items": items},
                                live_paths)
    picker.name = (str(_v(picker_prim, "ui:label", ""))
                   or picker_prim.GetName())
    picker.path = str(picker_prim.GetPath())
    return picker


def find(stage):
    """Every picker on the stage, in tab order.

    BY SCHEMA, never by path or filename. `IsA` rather than a type-name
    string compare, so a studio that subclasses RigExecPicker for its own
    metadata is still found by this.

    A picker may live ANYWHERE: beside the rig, under the asset root, or
    inside the RigExecRoot itself if a studio would rather ship one prim
    that holds the entire rig deliverable. That is a placement decision,
    and this deliberately does not make it for them.

    What the walk does assume is that a picker is never a CHILD of rig
    graph: there is no picker inside a joint, a control, a solver or a
    mover. Pruning there skips the 900-odd prims that make up the graph
    while still finding a picker parented to the rig root or to a scope
    under it. It also does not descend into a picker it has found, since
    everything below one is panels and buttons.
    """
    if stage is None:
        return []
    found = []
    walk = iter(Usd.PrimRange(stage.GetPseudoRoot()))
    for prim in walk:
        if prim.IsA(PICKER_TYPE):
            found.append(prim)
            walk.PruneChildren()
        elif _IsRigGraph(prim):
            walk.PruneChildren()
    return sorted(found, key=_order)


def _IsRigGraph(prim):
    """A rig graph prim: joints, controls, solvers, movers, weights.

    Everything a picker is not, and nothing a picker can be inside of.
    The RigExecRoot itself is NOT one of these -- it is the scope the
    graph lives in, and a picker is allowed to sit in it.
    """
    name = prim.GetTypeName()
    return bool(name) and name != PICKER_ROOT_SAFE and str(name).startswith(
        "RigExec") and not str(name).startswith("RigExecPicker")


def rig_for(picker_prim):
    """The RigExecRoot a picker drives, or None for "the whole stage"."""
    rel = picker_prim.GetRelationship("rigExec:picker:rig")
    targets = rel.GetTargets() if rel else []
    if not targets:
        return None
    prim = picker_prim.GetStage().GetPrimAtPath(targets[0])
    return prim if prim and prim.IsValid() else None


def _live_under(root):
    """Selectable prims inside one rig.

    Controls AND joints: the biped's fingers are RigExecJoints posed
    through their own avars rather than controls, and a button that
    selects one is a working button.
    """
    return set(str(p.GetPath()) for p in Usd.PrimRange(root)
               if p.IsA(CONTROL_TYPE) or p.IsA(JOINT_TYPE))


def load_all(stage):
    """Every picker on the stage, ready to draw. May be empty.

    Each picker resolves its buttons against the rig it NAMES, so on a
    stage with two characters neither one's liveness depends on the
    other being loaded. A picker that names no rig falls back to the
    whole stage, which is the right answer for the single-character case
    and the only answer available without the relationship.
    """
    prims = find(stage)
    if not prims:
        return []
    live_by_rig = {}
    pickers = []
    for prim in prims:
        rig = rig_for(prim)
        key = str(rig.GetPath()) if rig else ""
        if key not in live_by_rig:
            live_by_rig[key] = (_live_under(rig) if rig
                                else pickerModel.live_control_paths(stage))
        pickers.append(read(prim, live_by_rig[key]))
    return pickers
