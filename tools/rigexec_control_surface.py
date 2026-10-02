#!/usr/bin/env python
"""Keep the picker and TouchPose pointing at the controls a rig actually has.

WHY THIS EXISTS. A picker layout and a set of TouchPose regions are authored
against whatever the rig had at the time. Add controls later and both go
stale in the same quiet way: a button or a region still names the JOINT it
used to select, so it still works -- it selects a joint, whose avars the
Avar Editor will happily edit -- and nothing anywhere reports that the
control built to drive that joint is the thing the animator wanted. The
biped's face was exactly this: 22 picker buttons and 15 touch regions all
naming face joints, every one of them now driven by a control.

So retargeting is not a fix-up, it is the step that has to run whenever a
rig gains controls. Two operations, both driven by the same (control,
joint) pairs the builder already returns:

  RETARGET  every picker button and touch region that names a joint we now
            drive is re-pointed at the control that drives it.
  ADD       a control that nothing selects yet gets a button, placed by the
            projection FITTED FROM THE BUTTONS THAT ARE ALREADY THERE.

That last part is what makes the result usable rather than merely correct.
The existing buttons are an artist's layout of a face; fitting world
position to panel position across them recovers that layout's own framing,
so a new control lands where a person would have put it instead of on a
grid in the corner. With sixteen wired buttons the fit is well determined,
and it degrades honestly: too few samples and the function says so rather
than inventing a placement.

Everything is authored into a layer you hand in, so this composes as an
addition over a published picker rather than editing it.

Usage as a library:

    import rigexec_control_surface as surface
    surface.retarget(stage, layer, pairs, positions)
    surface.add_buttons(stage, layer, panel, missing, positions)
"""
import sys

from pxr import Gf, Sdf, Usd


PICKER_BUTTON = "RigExecPickerButton"
PICKER_PANEL = "RigExecPickerPanel"
TOUCH_REGION = "RigExecTouchRegion"

BUTTON_CONTROLS = "rigExec:picker:controls"
TOUCH_CONTROL = "rigExec:touch:control"

# What a control added by this module looks like before anyone styles it.
DEFAULT_SIZE = (22.0, 22.0)
DEFAULT_FILL = (0.85, 0.72, 0.25, 1.0)
DEFAULT_STROKE = (0.0, 0.0, 0.0, 0.5)


def _targets(prim, name):
    rel = prim.GetRelationship(name)
    return list(rel.GetTargets()) if rel else []


def _prims_of_type(stage, type_name):
    return [p for p in stage.Traverse() if p.GetTypeName() == type_name]


def is_hidden(stage, path):
    """A twin or a pivot: drawn with no guide, so not selectable."""
    prim = stage.GetPrimAtPath(Sdf.Path(str(path)))
    if not prim or not prim.IsValid():
        return True
    for axis in "XYZ":
        attr = prim.GetAttribute("guide:scale%s" % axis)
        if attr and attr.Get() == 0.0:
            return True
    radius = prim.GetAttribute("guide:radius")
    return bool(radius and radius.Get() == 0.0)


def _is_joint(stage, path):
    prim = stage.GetPrimAtPath(Sdf.Path(str(path)))
    return bool(prim and prim.IsValid() and
                prim.GetTypeName() == "RigExecJoint")


def is_selectable_control(stage, path):
    """A RigExecControl an animator can actually see and grab."""
    prim = stage.GetPrimAtPath(Sdf.Path(str(path)))
    return (prim and prim.IsValid() and
            prim.GetTypeName() == "RigExecControl" and
            not is_hidden(stage, path))


# ---------------------------------------------------------------------------
# Retargeting
# ---------------------------------------------------------------------------

def retarget(stage, layer, pairs, verbose=True):
    """Re-point buttons and regions from joints onto the controls driving them.

    `pairs` maps a joint prim path to the control prim path that now drives
    it. Both are full paths, because a picker button names a path and a name
    is not unique on a rig with a left and a right of everything.

    Returns dict(buttons=[...], regions=[...]) naming what moved.
    """
    moved = {"buttons": [], "regions": []}
    with Usd.EditContext(stage, Usd.EditTarget(layer)):
        for prim in _prims_of_type(stage, PICKER_BUTTON):
            current = _targets(prim, BUTTON_CONTROLS)
            wanted = [pairs.get(str(t), str(t)) for t in current]
            if wanted == [str(t) for t in current]:
                continue
            prim.CreateRelationship(BUTTON_CONTROLS).SetTargets(
                [Sdf.Path(w) for w in wanted])
            moved["buttons"].append((prim.GetName(),
                                     [w.rsplit("/", 1)[-1] for w in wanted]))

        for prim in _prims_of_type(stage, TOUCH_REGION):
            current = _targets(prim, TOUCH_CONTROL)
            wanted = [pairs.get(str(t), str(t)) for t in current]
            if wanted == [str(t) for t in current]:
                continue
            prim.CreateRelationship(TOUCH_CONTROL).SetTargets(
                [Sdf.Path(w) for w in wanted])
            moved["regions"].append((prim.GetName(),
                                     [w.rsplit("/", 1)[-1] for w in wanted]))

    if verbose:
        print("  retargeted %d picker button(s) and %d touch region(s)"
              % (len(moved["buttons"]), len(moved["regions"])))
        for name, to in moved["regions"]:
            print("    region %-24s -> %s" % (name, ", ".join(to)))
    return moved


def selected_controls(stage):
    """Every prim path any picker button or touch region already names."""
    named = set()
    for type_name, rel in ((PICKER_BUTTON, BUTTON_CONTROLS),
                           (TOUCH_REGION, TOUCH_CONTROL)):
        for prim in _prims_of_type(stage, type_name):
            for target in _targets(prim, rel):
                named.add(str(target))
    return named


# ---------------------------------------------------------------------------
# Placing what is missing
# ---------------------------------------------------------------------------

def fit_projection(stage, panel, positions, axis_pairs=((0, 0), (1, 1))):
    """Recover a panel's own world-to-panel mapping from its wired buttons.

    Least squares on each axis independently: panel = scale * world + offset.
    Independent rather than a full affine because a picker layout is a
    FRONT ELEVATION -- an artist lines features up on a grid, they do not
    shear it -- and two one-dimensional fits cannot introduce a rotation
    that the layout does not have.

    `axis_pairs` says which world axis feeds which panel axis: X to x and Y
    to y for a face seen from the front. Returns (fx, fy) as (scale, offset)
    pairs, or None when too few buttons carry a position to fit from.
    """
    samples = [[], []]
    for prim in Usd.PrimRange(panel):
        if prim.GetTypeName() != PICKER_BUTTON:
            continue
        pos = prim.GetAttribute("ui:position")
        size = prim.GetAttribute("ui:size")
        if not pos or pos.Get() is None:
            continue
        where = pos.Get()
        extent = size.Get() if size and size.Get() is not None else (0, 0)
        # The button's CENTRE: ui:position is its top-left corner, and a
        # brow button is twice the width of an eye one, so fitting corners
        # would bake each button's size into the mapping.
        centre = (where[0] + extent[0] * 0.5, where[1] + extent[1] * 0.5)
        for target in _targets(prim, BUTTON_CONTROLS):
            world = positions.get(str(target))
            if world is None:
                continue
            for k, (world_axis, panel_axis) in enumerate(axis_pairs):
                samples[k].append((world[world_axis], centre[panel_axis]))

    fits = []
    for pairs in samples:
        if len(pairs) < 3:
            return None
        n = float(len(pairs))
        sx = sum(a for a, _ in pairs)
        sy = sum(b for _, b in pairs)
        sxx = sum(a * a for a, _ in pairs)
        sxy = sum(a * b for a, b in pairs)
        denominator = n * sxx - sx * sx
        if abs(denominator) < 1e-9:
            return None
        scale = (n * sxy - sx * sy) / denominator
        fits.append((scale, (sy - scale * sx) / n))
    return tuple(fits)


def _clear_spot(stage, panel, centre, size, step_factor=1.25, tries=40):
    """The nearest centre at or below \\p centre whose box overlaps no button."""
    boxes = []
    for prim in Usd.PrimRange(panel):
        if prim.GetTypeName() != PICKER_BUTTON:
            continue
        pos = prim.GetAttribute("ui:position")
        ext = prim.GetAttribute("ui:size")
        if not pos or pos.Get() is None or not ext or ext.Get() is None:
            continue
        (x, y), (w, h) = pos.Get(), ext.Get()
        boxes.append((x, y, x + w, y + h))
    cx, cy = centre
    for _ in range(tries):
        x0, y0 = cx - size[0] * 0.5, cy - size[1] * 0.5
        x1, y1 = x0 + size[0], y0 + size[1]
        if not any(x0 < bx1 and bx0 < x1 and y0 < by1 and by0 < y1
                   for bx0, by0, bx1, by1 in boxes):
            return (cx, cy)
        cy += size[1] * step_factor
    return (cx, cy)


def add_buttons(stage, layer, panel, entries, positions, fit=None,
                size=DEFAULT_SIZE, fill=DEFAULT_FILL, verbose=True):
    """A button per entry, placed by `fit`, under `panel`.

    `entries` is [(prim_path, label)]. Anything with no world position, or
    any entry at all when the panel could not be fitted, is reported and
    skipped rather than placed somewhere invented.
    """
    if fit is None:
        fit = fit_projection(stage, panel, positions)
    if fit is None:
        if verbose:
            print("  picker: NOT placing %d control(s) -- %s has too few "
                  "wired buttons to fit a layout from"
                  % (len(entries), panel.GetName()))
        return []

    (sx, ox), (sy, oy) = fit
    added, skipped = [], []
    with Usd.EditContext(stage, Usd.EditTarget(layer)):
        for path, label in entries:
            world = positions.get(path)
            if world is None:
                skipped.append(label)
                continue
            centre = (sx * world[0] + ox, sy * world[1] + oy)
            # A front elevation drops DEPTH, so two controls stacked in front
            # of each other -- the look plate and lookRot -- project to the
            # same spot, and a button on top of another is one nobody can
            # click. Step down until the box is clear of every button on the
            # panel, including ones placed earlier in this same call.
            centre = _clear_spot(stage, panel, centre, size)
            name = "b_" + label.replace(":", "_")
            button = stage.DefinePrim(
                panel.GetPath().AppendChild(name), PICKER_BUTTON)
            button.CreateAttribute(
                "ui:position", Sdf.ValueTypeNames.Float2, True,
                Sdf.VariabilityUniform).Set(
                    Gf.Vec2f(centre[0] - size[0] * 0.5,
                             centre[1] - size[1] * 0.5))
            button.CreateAttribute(
                "ui:size", Sdf.ValueTypeNames.Float2, True,
                Sdf.VariabilityUniform).Set(Gf.Vec2f(*size))
            button.CreateAttribute(
                "ui:shape", Sdf.ValueTypeNames.Token, True,
                Sdf.VariabilityUniform).Set("rectangle")
            button.CreateAttribute(
                "ui:roundness", Sdf.ValueTypeNames.Float, True,
                Sdf.VariabilityUniform).Set(40.0)
            button.CreateAttribute(
                "ui:fill", Sdf.ValueTypeNames.Color4f, True,
                Sdf.VariabilityUniform).Set(Gf.Vec4f(*fill))
            button.CreateAttribute(
                "ui:stroke", Sdf.ValueTypeNames.Color4f, True,
                Sdf.VariabilityUniform).Set(Gf.Vec4f(*DEFAULT_STROKE))
            button.CreateRelationship(BUTTON_CONTROLS).SetTargets(
                [Sdf.Path(path)])
            added.append((name, centre))

    if verbose:
        print("  picker: added %d button(s) to %s"
              % (len(added), panel.GetName()))
        for name, centre in added:
            print("    %-26s at (%.0f, %.0f)" % (name, centre[0], centre[1]))
        if skipped:
            print("    no world position, skipped: %s" % ", ".join(skipped))
    return added


def panel_named(stage, label):
    """The panel whose ui:label or prim name matches, or None."""
    for prim in _prims_of_type(stage, PICKER_PANEL):
        attr = prim.GetAttribute("ui:label")
        name = (attr.Get() if attr and attr.Get() else prim.GetName())
        if name.lower() == label.lower():
            return prim
    return None


def set_panel_label(stage, layer, panel, label, verbose=True):
    """Rename a panel's TAB without renaming the prim.

    The prim name is identity -- an override anyone authored against this
    panel is keyed on it -- so the tab text is the thing to change.
    """
    with Usd.EditContext(stage, Usd.EditTarget(layer)):
        panel.CreateAttribute("ui:label", Sdf.ValueTypeNames.String, True,
                              Sdf.VariabilityUniform).Set(label)
    if verbose:
        print("  picker: %s tab labelled %r" % (panel.GetName(), label))


# ---------------------------------------------------------------------------
# What drives what, read off the rig itself
# ---------------------------------------------------------------------------

def driving_controls(stage, rig_root="/Biped/Rig", verbose=False):
    """joint prim path -> the control path that poses it, for the whole rig.

    Read from the rig rather than handed in by whatever built it, so this
    stays true for controls nobody has written a builder for yet. Two
    sources, which between them cover how a joint gets posed:

      constraints  rigExec:moves names the joint, rigExec:sources the
                   control. A constraint writes its target's frame outright,
                   so its source IS the thing an animator should select.
      FK chains    rigExec:joints and rigExec:controls, positionally paired.

    A joint written by more than one thing keeps the FIRST control found, in
    traversal order. That is a choice and not a law: a joint with two
    drivers has no single right answer, and picking the first keeps the
    result stable across runs instead of depending on dictionary order.
    """
    # ALL the candidates per joint, then a choice between them. A joint is
    # commonly written more than once -- the nose bridge has a parent
    # constraint from its control AND a scale constraint from a hidden twin
    # -- and taking whichever the traversal met first picked the twin, which
    # is not a thing anyone should be able to select.
    candidates = {}

    def offer(joint, control, rank):
        if joint and control:
            candidates.setdefault(str(joint), []).append((rank, str(control)))

    for prim in stage.Traverse():
        type_name = prim.GetTypeName()
        if type_name.endswith("Constraint"):
            moves = _targets(prim, "rigExec:moves")
            sources = _targets(prim, "rigExec:sources")
            if not moves or not sources:
                continue
            # A parent constraint writes the whole frame and is what poses
            # the joint; a scale or rotation constraint trims one channel of
            # a frame something else already set.
            rank = 0 if type_name == "RigExecParentConstraint" else 1
            offer(moves[0], sources[0], rank)
        elif type_name == "RigExecFkChain":
            joints = _targets(prim, "rigExec:joints")
            controls = _targets(prim, "rigExec:controls")
            for joint, control in zip(joints, controls):
                offer(joint, control, 0)

    def hidden(path):
        return is_hidden(stage, path)

    driver = {}
    for joint, offers in candidates.items():
        # Visible first, then whole-frame writers, then authoring order.
        offers.sort(key=lambda o: (hidden(o[1]), o[0]))
        driver[joint] = offers[0][1]

    # A source that is ITSELF driven -- a twin, a pivot -- is not what an
    # animator wants to select. Follow the chain to something no constraint
    # writes, which is the control the animator actually holds.
    resolved = {}
    for joint, control in driver.items():
        seen = {joint}
        while control in driver and control not in seen:
            seen.add(control)
            control = driver[control]
        resolved[joint] = control

    if verbose:
        print("  %d joint(s) have a driving control" % len(resolved))
    return resolved


def defining_layer(stage, prim_path, property_name):
    """The strongest layer that actually carries this property.

    A published picker has to be EDITED IN ITS OWN LAYER. Authoring the
    retarget anywhere weaker composes underneath the published opinion and
    changes nothing -- silently, because a relationship that loses is not an
    error. So the edit goes where the opinion already is.
    """
    path = Sdf.Path(str(prim_path)).AppendProperty(property_name)
    for layer in stage.GetLayerStack():
        if layer.GetPropertyAtPath(path) is not None:
            return layer
    return None


def update(stage, rig_root="/Biped/Rig", panel_label=None, rename_to=None,
           add_missing=True, dry_run=False, verbose=True):
    """Bring every picker button and touch region up to date with the rig.

    Edits each opinion in the layer that carries it, so a published picker
    layer and a published regions layer are each updated in place rather
    than shadowed by a weaker one that would lose to them.
    """
    pairs = driving_controls(stage, rig_root, verbose=verbose)

    retargets = []
    # A control MOVED rather than added -- re-nested under a new parent, as
    # the eye look targets were when lookRot went in above them -- leaves
    # every button and region naming its old path, which no longer resolves.
    # Found again by name, but only when exactly one selectable control
    # still carries that name: a left/right pair or any other ambiguity is
    # left dead and reported, never guessed at.
    controls_by_name = {}
    for control in _prims_of_type(stage, "RigExecControl"):
        if is_selectable_control(stage, control.GetPath()):
            controls_by_name.setdefault(control.GetName(), []).append(
                str(control.GetPath()))

    def moved(target):
        prim_at = stage.GetPrimAtPath(Sdf.Path(target))
        if prim_at and prim_at.IsValid():
            return None
        found = controls_by_name.get(Sdf.Path(target).name, [])
        return found[0] if len(found) == 1 else None

    unresolved = []
    for type_name, rel in ((PICKER_BUTTON, BUTTON_CONTROLS),
                           (TOUCH_REGION, TOUCH_CONTROL)):
        for prim in _prims_of_type(stage, type_name):
            original = [str(t) for t in _targets(prim, rel)]
            current = []
            for target in original:
                new_path = moved(target)
                if new_path is None and not stage.GetPrimAtPath(
                        Sdf.Path(target)):
                    unresolved.append((prim.GetName(), target))
                current.append(new_path or target)
            # ONLY A JOINT MOVES, AND ONLY ONTO A REAL CONTROL. A button
            # already naming a control is an authored choice -- `arm_l_params`
            # is the IK/FK switch and is driven by nothing an animator wants
            # instead -- and a "driver" that turns out to be another joint
            # (a spline IK's internal chain for skull_bind) is not a control
            # at all. Both were in the first dry run of this.
            wanted = []
            for target in current:
                candidate = pairs.get(target, target)
                if (candidate != target and
                        _is_joint(stage, target) and
                        is_selectable_control(stage, candidate)):
                    wanted.append(candidate)
                else:
                    wanted.append(target)
            if wanted != original:
                retargets.append((prim, rel, original, wanted))

    if verbose and unresolved:
        print("  %d target(s) name a prim that no longer exists and no single "
              "control could be found for:" % len(unresolved))
        for owner, target in unresolved[:20]:
            print("    %-26s %s" % (owner, target))
    if verbose:
        print("  %d button(s)/region(s) name a joint that now has a control"
              % len(retargets))
        for prim, _rel, current, wanted in retargets[:40]:
            print("    %-26s %s -> %s"
                  % (prim.GetName(),
                     ", ".join(c.rsplit("/", 1)[-1] for c in current),
                     ", ".join(w.rsplit("/", 1)[-1] for w in wanted)))
    if not dry_run:
        for prim, rel, _current, wanted in retargets:
            layer = defining_layer(stage, prim.GetPath(), rel)
            if layer is None:
                continue
            with Usd.EditContext(stage, Usd.EditTarget(layer)):
                prim.CreateRelationship(rel).SetTargets(
                    [Sdf.Path(w) for w in wanted])

    placed = []
    if panel_label:
        panel = panel_named(stage, panel_label)
        if panel is None:
            if verbose:
                print("  no panel labelled %r" % panel_label)
        else:
            layer = defining_layer(stage, panel.GetPath(), "ui:label") \
                or stage.GetLayerStack()[0]
            if rename_to and not dry_run:
                set_panel_label(stage, layer, panel, rename_to,
                                verbose=verbose)
            if add_missing:
                positions = _control_positions(stage, rig_root)
                # What the retargets in THIS run will select, as well as
                # what already is. Otherwise a dry run offers to add a
                # button for every control the same run is about to point
                # an existing button at.
                already = selected_controls(stage)
                for _prim, _rel, _current, wanted_targets in retargets:
                    already.update(wanted_targets)
                entries = [(str(p.GetPath()), p.GetName())
                           for p in _prims_of_type(stage, "RigExecControl")
                           if str(p.GetPath()) not in already
                           and str(p.GetPath()) in positions
                           and is_selectable_control(stage, p.GetPath())]
                entries = [e for e in entries
                           if _near_panel(e[0], panel, positions, pairs)]
                if dry_run:
                    if verbose:
                        print("  would add %d button(s): %s"
                              % (len(entries), [n for _p, n in entries]))
                else:
                    placed = add_buttons(stage, layer, panel, entries,
                                         positions, verbose=verbose)
    return {"retargeted": len(retargets), "added": len(placed)}


def _control_positions(stage, rig_root):
    """Asset-space rest origin per control and joint, for placing buttons."""
    import rigexec
    rig = rigexec.Rig(stage, rig_root)
    rig.compile()
    rest = rig.evaluate(0.0)
    out = {}
    for path in rest.control_paths():
        out[str(path)] = Gf.Matrix4d(
            *rest.control_frame(str(path)).to_matrix4()).ExtractTranslation()
    for path in rest.joint_paths():
        out[str(path)] = Gf.Matrix4d(
            *rest.joint_frame(str(path)).to_matrix4()).ExtractTranslation()
    return out


def _near_panel(path, panel, positions, pairs, margin=1.6):
    """Whether a control belongs on this panel, by where the panel looks.

    A panel covers a REGION of the character -- the face one covers a head
    -- and the only statement of which region is the buttons already on it.
    So the bounds of what those buttons point at is the test, grown by a
    margin so a control just outside the existing spread still lands rather
    than being dropped for being new.

    Without this, "every control nothing selects" would put the toes on the
    face tab.
    """
    known = []
    for prim in Usd.PrimRange(panel):
        if prim.GetTypeName() != PICKER_BUTTON:
            continue
        for target in _targets(prim, BUTTON_CONTROLS):
            where = positions.get(str(target))
            if where is not None:
                known.append(where)
    if len(known) < 3:
        return False
    lo = [min(k[i] for k in known) for i in range(3)]
    hi = [max(k[i] for k in known) for i in range(3)]
    centre = [(lo[i] + hi[i]) * 0.5 for i in range(3)]
    half = [max((hi[i] - lo[i]) * 0.5 * margin, 1.0) for i in range(3)]
    where = positions.get(path)
    return where is not None and all(
        abs(where[i] - centre[i]) <= half[i] for i in range(3))


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stage")
    parser.add_argument("--rig-root", default="/Biped/Rig")
    parser.add_argument("--panel", default="Facial",
                        help="panel to add unreferenced controls to")
    parser.add_argument("--rename-panel", default=None,
                        help="new tab label for that panel")
    parser.add_argument("--no-add", dest="add", action="store_false")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    import rigexec
    rigexec.load_schema_plugin()
    stage = Usd.Stage.Open(args.stage)
    if stage is None:
        raise SystemExit("cannot open %s" % args.stage)

    result = update(stage, args.rig_root, args.panel, args.rename_panel,
                    args.add, args.dry_run)
    if args.dry_run:
        print("  dry run: nothing written")
        return 0
    written = set()
    for layer in stage.GetLayerStack():
        if layer.dirty:
            layer.Save()
            written.add(layer.identifier)
    print("  retargeted %d, added %d; saved %d layer(s)"
          % (result["retargeted"], result["added"], len(written)))
    for identifier in sorted(written):
        print("    %s" % identifier)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
