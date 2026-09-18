#
# TouchPose awareness for the node-graph editor.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#

"""What usdNoodles needs to know about TouchPose regions.

TouchPose paints regions of a mesh and binds each one to a rig control.
The data is plain USD -- no schema, no typed prims -- so nothing in the
editor recognises it by type. It is laid out as:

    /Biped/TouchPose                     Scope   (the group)
        touchpose:mesh        string
        touchpose:palette     color3f[]
        touchpose:alpha       float
        touchpose:leadColor   color3f
        touchpose:selectedColor color3f
      /Biped/TouchPose/<region>_touch    Scope   (one per painted region)
        touchpose:faces       int[]      face indices on the mesh
        touchpose:elementType token
        touchpose:hilight     int
        touchpose:color       color3f
        touchpose:control     rel   ->   the control the region selects

The regions used to be `UsdGeomSubset`s under the mesh; they are not any
more, because hdSt collects every face GeomSubset whatever its
familyName and they cost 16,739 `SanitizeGeomSubsets` warnings on open.
This module keys off the `touchpose:` properties and NOT off prim type
or prim path, so it survives that kind of move: a region is a prim with
`touchpose:faces`, wherever it lives.

Everything here is pure USD reads -- no Qt -- so it imports headless.
"""

from __future__ import annotations

FACES_ATTR = "touchpose:faces"
CONTROL_REL = "touchpose:control"
MESH_ATTR = "touchpose:mesh"
PALETTE_ATTR = "touchpose:palette"
NAMESPACE = "touchpose:"

# The group is identified by the two properties only the group carries.
# `touchpose:alpha` and the two state colours are deliberately NOT in this
# list: a file written before they existed still has to read as a group.
_GROUP_MARKERS = (MESH_ATTR, PALETTE_ATTR)


def is_touch_region(prim) -> bool:
    """True for one painted region: the prim that owns the face indices."""
    if not prim or not prim.IsValid():
        return False
    return bool(prim.HasAttribute(FACES_ATTR))


def is_touch_group(prim) -> bool:
    """True for the scope the regions hang off (the palette carrier)."""
    if not prim or not prim.IsValid():
        return False
    if is_touch_region(prim):
        return False
    return any(prim.HasAttribute(name) for name in _GROUP_MARKERS)


def is_touch_prim(prim) -> bool:
    """True for either half of the layout."""
    return is_touch_region(prim) or is_touch_group(prim)


def control_paths(prim) -> list:
    """The prim paths one region drives, [] when it binds nothing.

    A region with no control is a region the pick loop refuses to act on;
    the importer drops those, but a hand-written or older layer can still
    carry one, and it must not become a broken link.
    """
    if not is_touch_region(prim):
        return []
    rel = prim.GetRelationship(CONTROL_REL)
    if not rel or not rel.IsValid():
        return []
    return [path for path in rel.GetTargets() if path.IsPrimPath()]


def group_regions(group_prim) -> list:
    """The region prims under a group, in USD order.

    ``GetAllChildren`` and not ``GetChildren``, for the same reason
    ``find_touch_groups`` walks all prims: see there. Inactive children are
    dropped by hand, because that is the one thing the default child
    predicate was doing that we still want.
    """
    if not group_prim or not group_prim.IsValid():
        return []
    return [
        child
        for child in group_prim.GetAllChildren()
        if child.IsActive() and is_touch_region(child)
    ]


def find_touch_groups(stage) -> list:
    """Every TouchPose group on a stage, wherever it was composed in.

    A full traverse, pruned below each group: on the shipped biped the
    whole stage is 695 prims and the traverse measures at 1 ms, which is
    two orders of magnitude under the per-paint back-reference scan the
    link rebuild already pays (that one collects links for EVERY prim).
    Searching by property rather than by a hardcoded `/Biped/TouchPose`
    means a second character, or a renamed root, still finds its regions.

    ALL prims, not the default predicate. A regions layer is written as an
    `over` on the character (so it can be re-imported without rewriting 4
    MB of rig), and everything under an `over` ancestor is undefined:
    measured on `Biped_touch_regions.usda` opened on its own, the default
    traversal yields 0 prims and the all-prims traversal yields 102. The
    editor is a graph over authored opinions, so the layer alone has to
    read as the graph it is.
    """
    if stage is None:
        return []

    from pxr import Usd

    groups = []
    it = iter(Usd.PrimRange(stage.GetPseudoRoot(), Usd.PrimAllPrimsPredicate))
    for prim in it:
        if not is_touch_group(prim):
            continue
        # Regions are this prim's children; nothing below a group can be
        # another group.
        it.PruneChildren()
        if prim.IsActive():
            groups.append(prim)
    return groups


def collect_graph_prims(group_prim, stage=None) -> list:
    """A group, its regions, and the controls those regions drive.

    The controls are part of the answer and not an extra: a region node
    on its own says "this patch of skin is called shoulder_l_touch",
    while the same node next to the control it selects is the thing a
    rigger opened the graph to see. Returned in draw order -- group,
    regions, then controls -- with duplicates removed, so a caller can
    add them straight to a graph.
    """
    if not is_touch_group(group_prim):
        return []

    stage = stage if stage is not None else group_prim.GetStage()
    ordered = [group_prim]
    seen = {group_prim.GetPath()}
    controls = []

    for region in group_regions(group_prim):
        if region.GetPath() not in seen:
            seen.add(region.GetPath())
            ordered.append(region)
        for path in control_paths(region):
            if path in seen:
                continue
            control = stage.GetPrimAtPath(path)
            if control and control.IsValid():
                seen.add(path)
                controls.append(control)

    ordered.extend(controls)
    return ordered
