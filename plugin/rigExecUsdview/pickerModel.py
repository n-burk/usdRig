"""The control picker's rules, with no Qt in sight.

A picker is SCENE DATA: `RigExecPicker` prims composed into the rig's own
layer stack. `pickerScene` reads them off the stage and hands the records
here; this module owns what they MEAN, and knows nothing about where they
were stored.

What this module decides:

  * which buttons are LIVE -- their controls exist on this stage -- so
    the panel never draws a button that cannot be pressed;
  * what a click means: replace the selection, add to it, or take from it;
  * which button matches the current selection, for the reverse highlight;
  * which half of an IK/FK pair is showing, from the limb's own dial.

Buttons that name no control at all (backdrops, the body silhouette) are
`decoration`: drawn, never interactive.
"""

# Picker panels are 400x600 and there are two of them side by side. A
# button
# smaller than this is a finger tip; do not let a click miss it.
MIN_HIT = 4.0

# The command buttons this panel actually implements. Anything else a
# picker
# carries is a script for the conventional tool, which we cannot run, and is left inert.
IMPLEMENTED_COMMANDS = ("zero_ctrls",)


class Button(object):
    """One picker item, ready to draw and to click."""

    def __init__(self, record, live_paths=None):
        self.id = record.get("id") or ""
        self.type = record.get("type") or ""
        self.parent = record.get("parent") or ""
        self.z = float(record.get("z") or 0.0)
        self.x = float(record.get("x") or 0.0)
        self.y = float(record.get("y") or 0.0)
        self.w = max(float(record.get("w") or 1.0), MIN_HIT)
        self.h = max(float(record.get("h") or 1.0), MIN_HIT)
        self.shape = record.get("shape") or "rectangle"
        self.roundness = float(record.get("roundness") or 0.0)
        self.direction = record.get("direction") or "top"
        self.left_slope = float(record.get("leftSlope") or 0.0)
        self.right_slope = float(record.get("rightSlope") or 0.0)
        self.polygon = [tuple(p) for p in (record.get("polygon") or [])]
        self.bezier = [[tuple(p) for p in pt]
                       for pt in (record.get("bezier") or [])]
        self.rotation = float(record.get("rotation") or 0.0)
        self.alternate = bool(record.get("alternate"))
        self.fill = tuple(record.get("fill") or (128, 128, 128, 255))
        self.stroke = tuple(record.get("stroke") or (0, 0, 0, 200))
        self.stroke_width = float(record.get("strokeWidth") or 1.0)
        self.text = record.get("text") or ""
        self.font_size = float(record.get("fontSize") or 8.0)
        self.bold = bool(record.get("bold"))
        self.h_align = record.get("hAlign") or "center"
        self.text_color = tuple(record.get("textColor") or (0, 0, 0, 255))
        self.value_color = tuple(record.get("valueColor")
                                 or (235, 235, 235, 255))
        self.checkbox = bool(record.get("checkbox"))
        self.checked = bool(record.get("checked"))
        self.value = record.get("value") or ""
        self.objects = list(record.get("objects") or [])
        self.targets = list(record.get("targets") or [])
        # An attribute this button drives, rather than a selection it
        # makes: {path, attr, enum, invert}. The IK/FK switches.
        self.attr_target = record.get("attrTarget") or None
        # "ik" / "fk" when this button belongs to one side of a limb's
        # IK/FK switch, else None. Baked from the opacity wiring.
        # A commandButton this panel implements itself. A picker stores a
        # the conventional tool python script per button, which is not runnable here, so
        # the LABEL is the binding -- `Zero Ctrls` and friends. Buttons we
        # have no action for stay inert rather than pretending.
        self.command = None
        if (record.get("type") == "commandButton"
                and (record.get("text") or "").strip()):
            self.command = (record["text"].strip().lower()
                            .replace(" ", "_"))
        self.mode = record.get("mode") or None
        self.dial = record.get("dial") or None

        # Live = every target it names exists on this stage. A button whose
        # control we have not ported is drawn dimmed rather than hidden:
        # the animator can see the rig is incomplete there, which a missing
        # button would not tell them.
        if self.command in IMPLEMENTED_COMMANDS:
            self.live = True
        elif self.attr_target:
            self.live = (live_paths is None
                         or self.attr_target["path"] in live_paths)
        elif live_paths is None:
            self.live = bool(self.targets)
        else:
            self.live = bool(self.targets) and all(
                t in live_paths for t in self.targets)

    @property
    def decoration(self):
        """Drawn, never clickable: backdrops and the body silhouette."""
        return (not self.objects and not self.attr_target
                and self.command not in IMPLEMENTED_COMMANDS)

    def label_for(self, current):
        """The label that goes with the value already on the attribute.

        The READ side of `next_value`, and the picker needs it because the
        labels are BAKED from the conventional tool: `attributeButton675` ships value "FK"
        because L_Arm was in FK when the picker was exported, and it kept
        saying so however the rig's own `avars:ikfk` read. Measured in
        usdview on Biped_all: with arm_l ikfk at 1.0 the reopened panel
        still drew "Hand FK".
        """
        target = self.attr_target or {}
        labels = target.get("enum") or []
        if not labels:
            return None
        try:
            index = int(round(float(current)))
        except (TypeError, ValueError):
            return None
        if target.get("invert"):
            index = len(labels) - 1 - index
        if 0 <= index < len(labels):
            return labels[index]
        return None

    def next_value(self, current):
        """The value a click should write, given what is there now.

        An enum button cycles. the conventional enum on the IK/FK switches is
        "IK,FK" -- index 0 IK, 1 FK -- while ours is the blend weight,
        0 FK and 1 IK. `invert` carries that, so the label the animator
        reads and the number we write agree.
        """
        target = self.attr_target or {}
        labels = target.get("enum") or []
        if not labels:
            return None, None
        # One step past whatever the attribute reads as now; `label_for`
        # owns the inversion so the read and the write cannot disagree.
        here = self.label_for(current) or labels[0]
        index = (labels.index(here) + 1) % len(labels)
        value = index
        if target.get("invert"):
            value = len(labels) - 1 - index
        return float(value), labels[index]

    def __repr__(self):
        return "<Button %s %s %s>" % (self.id, self.shape,
                                      ",".join(self.targets) or "-")


class Panel(object):
    def __init__(self, record):
        self.id = record.get("id") or ""
        self.label = record.get("label") or ""
        self.x = float(record.get("x") or 0.0)
        self.y = float(record.get("y") or 0.0)
        self.w = float(record.get("w") or 400.0)
        self.h = float(record.get("h") or 600.0)
        self.fill = tuple(record.get("fill") or (68, 68, 68, 255))


class Picker(object):
    """A parsed picker: panels, buttons, and what is live on this stage."""

    def __init__(self, document, live_paths=None):
        self.source = document.get("source") or ""
        self.panels = [Panel(p) for p in document.get("panels") or []]
        self.buttons = [Button(b, live_paths)
                        for b in document.get("items") or []]
        # Draw order is zValue ascending, the same rule the exporter used.
        self.buttons.sort(key=lambda b: b.z)

    # -- lookups ---------------------------------------------------------

    def panel(self, panel_id):
        for panel in self.panels:
            if panel.id == panel_id:
                return panel
        return None

    def visible(self, panel_id=None, all_states=False, modes=None,
                edit=False):
        """Buttons to draw, in painter order.

        Two things are left OUT rather than drawn:

          * a button whose control this rig does not have. Dimming them was
            the first answer and the user's verdict was that an unusable
            button is clutter -- the picker is for driving the rig, not for
            documenting what the rig lacks. `verify`/`coverage()` still
            report them, so the gap stays measurable.
          * the inactive half of a limb's IK/FK switch. `modes` maps a
            limb's dial path to its current mode, so a limb in IK shows its
            IK controls only, which is what the fade wiring already does to
            the viewport gizmos.
        """
        out = []
        for button in self.buttons:
            if panel_id is not None and button.parent != panel_id:
                continue
            # The positional "alternate state" heuristic must NOT apply to a
            # button that carries a real mode. A picker stacks a limb's IK
            # and FK
            # buttons in the SAME slot -- that stack IS the IK/FK pair -- so
            # the heuristic flagged `L_ArmIK` as an alternate of `L_Foot`
            # and hid it in both states. Where the opacity wiring told us
            # the mode, that is the authority; the heuristic only covers
            # stacks we know nothing about.
            if button.alternate and not all_states and not button.mode:
                continue
            # A button that resolves to nothing is NOT DRAWN. This spent
            # a while as a `pass` -- the docstring said the buttons were
            # dropped and the code kept drawing them dimmed -- and the
            # dimmed ones read as a broken picker every time somebody
            # opened it. The picker is for driving the rig; `coverage()`
            # still counts them, so the gap stays measurable.
            if not button.decoration and not button.live and not edit:
                continue
            if modes and button.mode and button.dial:
                if modes.get(button.dial) not in (None, button.mode):
                    continue
            out.append(button)
        return out

    def dials(self):
        """Every IK/FK dial the buttons reference, as attribute paths."""
        found = []
        for button in self.buttons:
            for path in (button.dial,
                         (button.attr_target or {}).get("path")):
                if path and path not in found:
                    found.append(path)
        return found

    def hits(self, panel_id, x, y, modes=None, edit=False):
        """Clickable buttons under a panel-space point, topmost first.

        Rectangular containment, not the drawn path: the authored hit test is
        the bounding box, and a finger tip button is 6x6 px -- asking an
        animator to land inside a bezier that small would be unkind.
        """
        found = []
        # In edit mode the modes filter is off too: authoring a mapping
        # means reaching the half of the limb that is not currently live.
        for button in self.visible(panel_id, modes=None if edit else modes,
                                   edit=edit):
            if button.decoration:
                continue
            # Outside edit mode an unported button is inert -- drawn, but
            # it neither selects nor swallows the click.
            if not button.live and not edit:
                continue
            if (button.x <= x <= button.x + button.w
                    and button.y <= y <= button.y + button.h):
                found.append(button)
        found.reverse()
        return found

    def within(self, panel_id, x0, y0, x1, y1, modes=None, edit=False):
        """Selectable buttons whose box intersects a marquee rectangle.

        Intersection, not containment: a marquee that has to swallow a
        button whole makes the long thin ones (a spine segment, a finger)
        almost impossible to catch, and every DCC marquee this is modelled
        on touches rather than encloses.
        """
        lo_x, hi_x = (x0, x1) if x0 <= x1 else (x1, x0)
        lo_y, hi_y = (y0, y1) if y0 <= y1 else (y1, y0)
        found = []
        for button in self.visible(panel_id, modes=None if edit else modes,
                                   edit=edit):
            if button.decoration or (not button.live and not edit):
                continue
            if (button.x <= hi_x and button.x + button.w >= lo_x
                    and button.y <= hi_y and button.y + button.h >= lo_y):
                found.append(button)
        return found

    def buttons_for(self, paths):
        """Every button whose targets are all in `paths` -- the highlight.

        All, not any: the "select every finger" button names 21 controls,
        and lighting it up because one finger is selected would be noise.
        """
        wanted = set(str(p) for p in paths)
        if not wanted:
            return []
        return [b for b in self.buttons
                if b.targets and wanted.issuperset(b.targets)]

    def coverage(self):
        """(live, dead, decoration) counts, for the status line."""
        live = dead = deco = 0
        for button in self.buttons:
            if button.decoration:
                deco += 1
            elif button.live:
                live += 1
            else:
                dead += 1
        return live, dead, deco


# -------------------------------------------------------------- loading

def live_control_paths(stage):
    """Every prim a picker button may legitimately select.

    Controls AND joints. The biped's fingers are RigExecJoints posed
    through their own avars rather than controls, and the Avar Editor
    edits a joint exactly as it edits a control -- so a finger button that
    selects a joint is a working button, not a broken one.
    """
    if stage is None:
        return set()
    return set(str(p.GetPath()) for p in stage.Traverse()
               if str(p.GetTypeName()) in ("RigExecControl", "RigExecJoint"))
