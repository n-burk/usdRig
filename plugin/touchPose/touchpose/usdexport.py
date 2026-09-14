"""Touch sets -> `UsdGeomSubset` overs. `pxr` only: no Qt, no ctypes.

Stock USD, no schema work. A touch region is a named set of faces on a
mesh with some data hanging off it, and `UsdGeomSubset` is exactly that,
so the port needs no new prim type to hold the data -- only two custom
properties on a standard subset:

    over "Biped" { over "Geom" { over "body_geo" {
        uniform token subsetFamily:touchpose:L0:familyType = "nonOverlapping"
        def GeomSubset "thumb_003_r_touch" {
            uniform token elementType = "face"
            uniform token familyName  = "touchpose:L0"
            int[] indices = [21416, ...]
            rel   touchpose:control = </Biped/Rig/.../thumb_003_r_bind>
            int   touchpose:hilight = 0
            color3f touchpose:color = (0.038, 0.449, 0.508)
        }
    }}}

WHY `nonOverlapping` AND NOT `partition`. Measured, not assumed: the 247
shipped sets reference 23,867 distinct faces of `body_geo` with a maximum
multiplicity of 1 -- no face belongs to two regions -- but 2,407 faces
belong to no region at all (soles, inner mouth, scalp under hair). That
is precisely `nonOverlapping`: `partition` would be a lie USD is entitled
to complain about, and `unrestricted` would throw away the one fact that
makes a face -> region lookup a flat 26k array instead of a search.

WHY AN `over` IN ITS OWN LAYER. The touch data is delivery data that gets
re-exported whenever the studio repaints regions, and the mesh layer is
4 MB of authored geometry. Authoring subsets INTO the mesh layer would
mean rewriting that file on every touch re-import and make a bad import
hard to back out. As its own sublayer it is one file to delete. The
written file sublayers the rig so it opens directly in usdview; strip the
`subLayers` line and it is a pure overlay to stack under something else.

WHAT DOES NOT SURVIVE. Face sets on meshes this port does not have --
hair, teeth, tongue, corneas, lashes: 6,934 faces over 8 meshes -- are
reported and dropped. There is no sane fudge: those indices index a mesh
that is not here, and writing them against `body_geo` would silently
light up the wrong part of the body.
"""
import os

from pxr import Gf, Sdf, Usd, UsdGeom, Vt

FAMILY = "touchpose:L0"
CONTROL_REL = "touchpose:control"
HILIGHT_ATTR = "touchpose:hilight"
COLOR_ATTR = "touchpose:color"
PALETTE_ATTR = "touchpose:palette"
ALPHA_ATTR = "touchpose:alpha"
# The authored STATE colours, carried in the same group record as the
# palette (`leadColor`, `selectedColor`). Written out because the pick
# loop needs a lead colour and a selected colour and the file already
# says what they are -- inventing two would mean the port and the the conventional tool
# tool disagreed about what "selected" looks like.
LEAD_COLOR_ATTR = "touchpose:leadColor"
SELECTED_COLOR_ATTR = "touchpose:selectedColor"
FACES_ATTR = "touchpose:faces"
ELEMENT_ATTR = "touchpose:elementType"
MESH_ATTR = "touchpose:mesh"
# Where the regions live. NOT under the mesh: hdSt collects every face
# GeomSubset under a mesh whatever its familyName, so touch regions
# collided with the materialBind subsets and cost 16,739
# `SanitizeGeomSubsets` warnings -- and their startup time -- for data
# the renderer never uses. A sibling scope carries the same face
# indices with no type the renderer cares about.
#
# The NAME is a default, not a location. This used to be the absolute
# path "/Biped/TouchPose", which meant TouchPose worked for exactly one
# character. The reader finds the scope by schema type, so where it sits
# is the rig's decision, including inside its RigExecRoot.
SCOPE_NAME = "TouchPose"
# The shipped highlight surface, one per regions scope.
OVERLAY_NAME = "Overlay"


def scope_path_for(mesh_path, rig_stage=None):
    """Where a new export puts the regions.

    INSIDE THE RIG when there is one. The regions name that rig's
    controls and the overlay is the surface they light, so both belong
    under its RigExecRoot: a rig published on its own carries its touch
    data with it, and a shot gets one set per asset rather than one
    global. Falls back to a sibling of the geometry when the stage has
    no rig, which is the paint-only case.
    """
    if rig_stage is not None:
        for prim in rig_stage.Traverse():
            if prim.GetTypeName() == "RigExecRoot":
                return prim.GetPath().AppendChild(SCOPE_NAME)
    return _beside_the_asset(mesh_path)


def _beside_the_asset(mesh_path):
    """The default regions scope for a mesh: beside its asset root.

    `/Show/Char/Geom/body_geo` -> `/Show/Char/TouchPose`.
    """
    # The mesh's GRANDPARENT, so `<asset>/Geom/body_geo` puts the
    # regions at `<asset>/TouchPose`, a sibling of the Geom scope. Taking
    # the first path component instead would put a `/Show/Char/...` mesh's
    # regions at `/Show/TouchPose`, outside the asset entirely.
    path = Sdf.Path(mesh_path)
    climbed = 0
    for _ in range(2):
        parent = path.GetParentPath()
        if parent.isEmpty or parent.IsAbsoluteRootPath():
            break
        path = parent
        climbed += 1
    if not climbed:
        # A mesh sitting at the root has no asset scope to be a sibling
        # of, so the regions go beside it rather than underneath it.
        return Sdf.Path.absoluteRootPath.AppendChild(SCOPE_NAME)
    return path.AppendChild(SCOPE_NAME)

# --- node-graph placement -------------------------------------------------
#
# usdNoodles reads these two off any prim it shows, so the regions are
# laid out and coloured AT AUTHORING TIME rather than by the editor.
# Without a position every node sits at the origin and 98 regions land in
# one pile; with one the editor and the loader agree about where things
# are, because the values ARE the editor's own.
#
# THE 1/1000 IS NOT DECORATION. `NodeModel._writePositionToUsd` divides by
# 1000 on the way in and multiplies by 1000 on the way out, so a position
# written in display units comes back a thousand times too far away the
# first time a rigger drags the node. Same convention, same constant, as
# `tools/biped/build_biped_rigexec.py::layout_for_noodles`.
POS_ATTR = "ui:nodegraph:node:pos"
DISPLAY_COLOR_ATTR = "ui:nodegraph:node:displayColor"
# The rig's columns are 1.6 apart and its Controls column is x=0, so this
# is one clear column to the left of everything: links leave the region
# column rightwards and never cross back over it. A region node measures
# 985 units wide (see below), so the column still clears x=0 by 0.6.
COLUMN_X = -1.6
# THE ROW PITCH IS MEASURED, AND IT IS NOT THE RIG'S.
# `build_biped_rigexec.layout_for_noodles` stacks rig nodes 0.11 apart and
# this started as a copy of that number. Then the nodes were measured
# through the editor's own layout (`GraphModel.calculateNodeSize`, the
# shipped Poppins metrics, default render config): a region node is 784
# display units tall and a rig control node is 4,879 to 6,244. A 0.11
# pitch is 110 units, so the rig's own layout overlaps its nodes by
# roughly fifty to one, and copying it would have stacked the 98 regions
# seven deep -- a different way of piling up at the origin. 0.9 clears a
# region node by 116 units.
ROW_PITCH = 0.9
# The group sits a pitch clear above the first region, so it reads as the
# head of the column rather than as row zero of it.
GROUP_GAP = 0.25


class Report(object):
    """What the import did, in numbers, so the caller can print them."""

    def __init__(self):
        self.written = []          # (set name, control path, face count)
        self.unresolved = []       # (set name, control name)
        self.skipped_unbound = 0   # of those, how many were NOT authored
        self.empty = []            # set name -- bound, but no faces painted
        self.skipped_meshes = {}   # mesh -> faces dropped
        self.out_of_range = []     # (set name, index) -- would have been a bug

    def summary(self):
        faces = sum(n for _s, _c, n in self.written)
        lines = [
            "wrote %d subsets, %d faces" % (len(self.written), faces),
            "%d sets name a control we do not have (%d of them skipped, "
            "not authored)" % (len(self.unresolved), self.skipped_unbound),
            "%d sets have no faces in the file at all" % len(self.empty),
        ]
        if self.skipped_meshes:
            lines.append("meshes not in this USD, skipped: " + ", ".join(
                "%s (%d faces)" % (m, n)
                for m, n in sorted(self.skipped_meshes.items())))
        if self.out_of_range:
            lines.append("!! %d face indices past the end of the mesh"
                         % len(self.out_of_range))
        return "\n".join(lines)


def _identifier(name):
    # Every shipped set name is already a legal USD identifier; this is
    # here so a hand-added one with a dash or a leading digit does not
    # abort the whole import at authoring time.
    safe = "".join(c if (c.isalnum() or c == "_") else "_" for c in name)
    return safe if (safe and not safe[0].isdigit()) else "_" + safe


def _new_layer(out_path, stage=None, sublayer=False):
    """An empty region layer, optionally sublayering the rig it is for."""
    if os.path.exists(out_path):
        os.remove(out_path)
    layer = Sdf.Layer.CreateNew(out_path)
    if sublayer and stage is not None:
        # Relative so the pair can be moved together.
        layer.subLayerPaths.append(
            "./" + os.path.basename(stage.GetRootLayer().realPath
                                    or stage.GetRootLayer().identifier))
    return layer


def _write_region(out_stage, scope, name, control, color, hilight, faces):
    """One region prim, typed. OFF the mesh -- see SCOPE_NAME for why."""
    prim = out_stage.DefinePrim(
        scope.GetPath().AppendChild(_identifier(name)),
        "RigExecTouchRegion")
    # The typed properties the reader looks for first. The custom
    # `touchpose:*` attributes written below stay, so a file opened by a
    # viewer built before the schema existed still works.
    prim.GetAttribute("rigExec:touch:faces").Set(Vt.IntArray(list(faces)))
    prim.GetAttribute("rigExec:touch:elementType").Set(UsdGeom.Tokens.face)
    if control:
        prim.GetRelationship("rigExec:touch:control").SetTargets(
            [Sdf.Path(control)])
    prim.CreateAttribute(FACES_ATTR, Sdf.ValueTypeNames.IntArray,
                         custom=True).Set(Vt.IntArray(list(faces)))
    prim.CreateAttribute(ELEMENT_ATTR, Sdf.ValueTypeNames.Token,
                         custom=True).Set(UsdGeom.Tokens.face)
    prim.CreateAttribute(HILIGHT_ATTR, Sdf.ValueTypeNames.Int,
                         custom=True).Set(int(hilight))
    prim.CreateAttribute(COLOR_ATTR, Sdf.ValueTypeNames.Color3f,
                         custom=True).Set(Gf.Vec3f(*color))
    if control:
        prim.CreateRelationship(CONTROL_REL, custom=True).SetTargets(
            [Sdf.Path(control)])
    return prim


def _write_overlay(out_stage, scope):
    """Ship the highlight surface with the regions it lights.

    Empty and invisible. The geometry is the deformed mesh's, which does
    not exist until a rig has been evaluated, so the runtime fills the
    points in a session layer; what ships is the declaration, so a shot
    never creates one and two assets never share one.
    """
    prim = out_stage.DefinePrim(
        scope.GetPath().AppendChild(OVERLAY_NAME), "RigExecTouchOverlay")
    # The points the runtime writes are already in WORLD space, out of
    # the terminal scene index. Nothing above this is Xformable on a
    # RigExec asset, but an asset whose root IS an Xform would double
    # transform the highlight, and resetting costs nothing where it was
    # never going to happen.
    UsdGeom.Xformable(prim).SetResetXformStack(True)
    return prim


def _write_palette(scope, palette, alpha, lead, selected):
    """One ramp and two state colours, shared by every region."""
    scope.GetAttribute("rigExec:touch:palette").Set(
        [Gf.Vec3f(*c) for c in (palette or [])])
    scope.GetAttribute("rigExec:touch:alpha").Set(float(alpha))
    scope.CreateAttribute(PALETTE_ATTR, Sdf.ValueTypeNames.Color3fArray,
                          custom=True).Set(
        [Gf.Vec3f(*c) for c in (palette or [])])
    scope.CreateAttribute(ALPHA_ATTR, Sdf.ValueTypeNames.Float,
                          custom=True).Set(float(alpha))
    scope.CreateAttribute(LEAD_COLOR_ATTR, Sdf.ValueTypeNames.Color3f,
                          custom=True).Set(Gf.Vec3f(*lead))
    scope.CreateAttribute(SELECTED_COLOR_ATTR, Sdf.ValueTypeNames.Color3f,
                          custom=True).Set(Gf.Vec3f(*selected))


def _control_row_key(stage, path):
    """Where in the rig's own node layout a control sits, for ordering.

    Returns (y, x) read off the control's `ui:nodegraph:node:pos`, or None
    when the rig has no layout for it -- on the shipped biped that is 4 of
    the 98 controls (the four `*_params` prims), which then sort last.
    """
    if stage is None or path is None:
        return None
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    attr = prim.GetAttribute(POS_ATTR)
    if not attr or not attr.IsValid() or not attr.HasAuthoredValue():
        return None
    pos = attr.Get()
    if pos is None:
        return None
    return (float(pos[1]), float(pos[0]))


def layout_for_noodles(out_stage, scope, rig_stage=None):
    """Place the group and its regions so usdNoodles can be read.

    One column, one region per row, ordered by where in the RIG's layout
    the control each region drives already sits. So the noodles between
    the two columns run roughly parallel instead of crossing 98 times.

    WHY A COLUMN AND NOT ONE ROW PER CONTROL. Putting each region at its
    control's own y was the first attempt and it is wrong twice over:
    measured on the shipped biped, 14 pairs of controls share a row (they
    are in different columns of the rig layout), so those regions would
    have been authored on top of each other -- and the controls span
    x=0.18 to x=6.44, which would have scattered the regions through the
    middle of the solver and joint columns instead of leaving them a
    column of their own.

    Returns how many prims were placed, the group included.
    """
    regions = []
    for child in scope.GetChildren():
        faces = child.GetAttribute(FACES_ATTR)
        if not faces or not faces.IsValid():
            continue
        rel = child.GetRelationship(CONTROL_REL)
        targets = rel.GetTargets() if rel else []
        key = _control_row_key(rig_stage, targets[0] if targets else None)
        regions.append((key is None, key or (0.0, 0.0), child.GetName(), child))

    regions.sort(key=lambda row: (row[0], row[1], row[2]))

    placed = 0
    for row, (_unplaced, _key, _name, prim) in enumerate(regions):
        _set_pos(prim, COLUMN_X, row * ROW_PITCH)
        color = prim.GetAttribute(COLOR_ATTR)
        if color and color.IsValid() and color.Get() is not None:
            # The node wears the colour the region is painted in, so the
            # graph and the viewport name the same region the same way.
            prim.CreateAttribute(DISPLAY_COLOR_ATTR,
                                 Sdf.ValueTypeNames.Color3f).Set(color.Get())
        placed += 1

    _set_pos(scope, COLUMN_X, -(ROW_PITCH + GROUP_GAP))
    lead = scope.GetAttribute(LEAD_COLOR_ATTR)
    if lead and lead.IsValid() and lead.Get() is not None:
        scope.CreateAttribute(DISPLAY_COLOR_ATTR,
                              Sdf.ValueTypeNames.Color3f).Set(lead.Get())
    placed += 1
    return placed


def _set_pos(prim, x, y):
    """Author `ui:nodegraph:node:pos`, in the editor's own units."""
    attr = prim.GetAttribute(POS_ATTR)
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(POS_ATTR, Sdf.ValueTypeNames.Float2)
    attr.Set(Gf.Vec2f(x, y))


def _over_the_ancestors(out_layer, *paths):
    """Put these prims' PARENTS back to `over`.

    `DefinePrim` DEFINES its ancestors on the way down, so the
    `over "Biped" { over "Geom" { over "body_geo"` this file is supposed
    to be comes out as `def`s with no type name. That composes the same
    with the rig underneath -- the rig's own specifier and type win the
    merge -- but the layer is meant to be readable and stackable ON ITS
    OWN, and one that DEFINES the character's geometry scope is a trap:
    drop it over the wrong asset and it creates `/Biped/Geom/body_geo`
    out of nothing instead of failing to find it.
    """
    for path in paths:
        if path is None:
            continue
        # The prim itself keeps its own specifier -- the regions scope is
        # DEFINED here, that is the point of the layer -- and only its
        # ancestors become overs. The mesh is passed as an ancestor of
        # nothing, so it is overred too, which is what it should be.
        ancestor = Sdf.Path(path)
        while ancestor != Sdf.Path.absoluteRootPath:
            spec = out_layer.GetPrimAtPath(ancestor)
            if spec is not None:
                spec.specifier = Sdf.SpecifierOver
            ancestor = ancestor.GetParentPath()


def save_regions(out_path, mesh_path, rows, palette=None, alpha=1.0,
                 lead=(0.054, 0.420, 0.187),
                 selected=(0.277, 0.277, 0.277),
                 stage=None, sublayer=False, scope_path=None):
    """Write regions straight from data, with no `.touch` file involved.

    `scope_path` is where the regions go. The caller passes the scope the
    model was READ from, so a save lands where the regions already live
    rather than at a path re-derived from the mesh -- re-deriving it is
    how a stage ends up with two scopes and a reader picking whichever
    it happened to meet first.

    `rows` is what `touchPoseModel.TouchModel.Rows()` hands back:
    (name, control path or None, colour, hilight, face indices). This is
    the PAINT path -- the animator edited the regions in the viewport and
    is saving them -- and it writes the identical layout `author` writes,
    through the same three helpers, so a hand-painted file and an
    imported one are the same kind of file.

    Regions with no faces are the caller's business to drop; anything
    handed over here is written.
    """
    out_layer = _new_layer(out_path, stage=stage, sublayer=sublayer)
    out_stage = Usd.Stage.Open(out_layer)
    out_stage.SetEditTarget(Usd.EditTarget(out_layer))
    out_stage.OverridePrim(mesh_path)
    scope = out_stage.DefinePrim(
        (Sdf.Path(scope_path) if scope_path
         else scope_path_for(mesh_path, rig_stage=stage)),
        "RigExecTouchRegions")
    scope.GetRelationship("rigExec:touch:mesh").SetTargets(
        [Sdf.Path(mesh_path)])
    scope.CreateAttribute(MESH_ATTR, Sdf.ValueTypeNames.String,
                          custom=True).Set(mesh_path)
    for name, control, color, hilight, faces in rows:
        _write_region(out_stage, scope, name, control, color, hilight, faces)
    _write_palette(scope, palette, alpha, lead, selected)
    _write_overlay(out_stage, scope)
    layout_for_noodles(out_stage, scope, rig_stage=stage)
    _over_the_ancestors(out_layer, mesh_path,
                        scope.GetPath().GetParentPath())
    out_layer.Save()
    return len(rows)


def author(doc, stage, out_path, mesh_path="/Biped/Geom/body_geo",
           resolve=None, family=FAMILY, sublayer=True, skip_unbound=True):
    """Write `doc`'s sets as GeomSubset overs on `mesh_path`.

    `resolve` maps a BUILD control name to a prim path, or returns None.
    With no resolver the subsets are still written, with no control
    relationship -- useful for looking at the regions before the naming
    is sorted out.

    WHY `skip_unbound` DEFAULTS TO TRUE, reversing what Phase 0 argued.
    Phase 0 wrote the unbound sets anyway so a missing region would read
    as a naming gap rather than a paint gap. In practice they are 142 of
    the 247 shipped sets -- overwhelmingly the face rig, and 56 eyelid
    sets between them -- and a subset with no control is a region the
    pick loop must test, highlight and then refuse to act on. Nothing
    downstream can use one, so nothing downstream should pay for one:
    with no resolver at all every set is still authored (that is the
    "look at the regions before the naming is sorted" case), and it is
    only a set whose control was LOOKED FOR and not found that is
    dropped. The count stays in the report so the gap is still visible.
    """
    mesh_prim = stage.GetPrimAtPath(mesh_path)
    if not mesh_prim or not mesh_prim.IsValid():
        raise ValueError("no mesh at %s" % mesh_path)
    mesh_name = mesh_prim.GetName()
    counts = UsdGeom.Mesh(mesh_prim).GetFaceVertexCountsAttr().Get()
    face_count = len(counts) if counts is not None else 0

    out_layer = _new_layer(out_path, stage=stage, sublayer=sublayer)
    out_stage = Usd.Stage.Open(out_layer)
    out_stage.SetEditTarget(Usd.EditTarget(out_layer))

    over = out_stage.OverridePrim(mesh_path)
    scope = out_stage.DefinePrim(
        scope_path_for(mesh_path, rig_stage=stage),
        "RigExecTouchRegions")
    scope.GetRelationship("rigExec:touch:mesh").SetTargets(
        [Sdf.Path(mesh_path)])
    scope.CreateAttribute(MESH_ATTR, Sdf.ValueTypeNames.String,
                          custom=True).Set(mesh_path)
    report = Report()

    for touch in doc.sets:
        if touch.face_count == 0:
            report.empty.append(touch.name)
            continue
        for mesh in touch.meshes:
            if mesh != mesh_name:
                report.skipped_meshes[mesh] = (
                    report.skipped_meshes.get(mesh, 0)
                    + len(touch.faces_on(mesh)))
        indices = touch.faces_on(mesh_name)
        if not indices:
            continue
        bad = [i for i in indices if i >= face_count]
        if bad:
            report.out_of_range.extend((touch.name, i) for i in bad)
            indices = [i for i in indices if i < face_count]
            if not indices:
                continue

        # RESOLVED BEFORE ANYTHING IS AUTHORED, not after. A skipped set
        # has to leave the layer untouched; deciding after Define() would
        # leave an empty `def GeomSubset` behind for every one of them.
        path = resolve(touch.control) if resolve else None
        if not path and skip_unbound and resolve is not None:
            report.unresolved.append((touch.name, touch.control))
            report.skipped_unbound += 1
            continue

        _write_region(out_stage, scope, touch.name, path, touch.color,
                      touch.hilight, indices)
        if path:
            report.written.append((touch.name, path, len(indices)))
        else:
            # No resolver was supplied, so nothing was looked for and
            # nothing can be said to be missing: author the region and
            # let the caller decide.
            report.unresolved.append((touch.name, touch.control))
            report.written.append((touch.name, None, len(indices)))

    _write_palette(scope, doc.palette.colors, doc.palette.alpha,
                   doc.palette.lead, doc.palette.selected)
    _write_overlay(out_stage, scope)
    layout_for_noodles(out_stage, scope, rig_stage=stage)
    _over_the_ancestors(out_layer, mesh_path,
                        scope.GetPath().GetParentPath())
    out_layer.Save()
    return report


def read_back(stage, mesh_path="/Biped/Geom/body_geo", family=FAMILY):
    """The inverse: subsets on a composed stage -> [(name, path, indices)].

    The pick path needs this, and having it here means the round trip is
    testable without the UI.
    """
    out = []
    # The regions live on the sibling scope now; the old subset layout is
    # still read so a layer written before the move still round-trips.
    scopes = [p for p in stage.Traverse()
              if p.GetTypeName() == "RigExecTouchRegions"]
    scope = scopes[0] if scopes else None
    if scope and scope.IsValid():
        for child in scope.GetChildren():
            faces = child.GetAttribute(FACES_ATTR)
            if not faces or not faces.IsValid():
                continue
            rel = child.GetRelationship(CONTROL_REL)
            targets = rel.GetTargets() if rel else []
            out.append((child.GetName(),
                        str(targets[0]) if targets else None,
                        faces.Get()))
        if out:
            return out

    prim = stage.GetPrimAtPath(mesh_path)
    if not prim or not prim.IsValid():
        return out
    for child in prim.GetChildren():
        subset = UsdGeom.Subset(child)
        if not subset:
            continue
        if subset.GetFamilyNameAttr().Get() != family:
            continue
        rel = child.GetRelationship(CONTROL_REL)
        targets = rel.GetTargets() if rel else []
        out.append((child.GetName(),
                    str(targets[0]) if targets else None,
                    subset.GetIndicesAttr().Get()))
    return out
