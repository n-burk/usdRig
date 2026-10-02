"""TouchPose's model: the regions, their colours, and the painting edits.

No Qt, and no geometry arithmetic either. The panel in `touchPoseUI` is the
mouse, the menu and the selection; this module is the data a test can
assert; and every question about the MESH -- which face a ray hits, which
regions a marquee touches, which faces a brush covers, where a face is --
is answered in C++ by `touchPoseNative.NativeMesh` (rigExecImaging,
libs/rigExecImaging/touchPoseMesh.cpp).

WHY THE GEOMETRY MOVED. Measured in usdview on the biped's 26,274-face
body, the numpy version paid 1.6 ms per ray on every hover sample, rebuilt
its triangle edge arrays whenever the pose moved, and could not follow the
mesh's world transform at all (the ray was world space, the points were
local). The native mesh keeps a BVH over the POSED triangles, refit in
parallel when the rig publishes a new pose, and casts in microseconds.

WHY THERE IS NO OVERLAY GEOMETRY HERE ANY MORE. The highlight used to be a
second mesh built from the region's faces, lifted off the skin and authored
into the session layer on every region crossing. It is now a Storm shader
tint on the body itself (libs/rigExecImaging/touchPoseHighlight.h): the
model's only contribution is the face -> region table and the per-region
colours, which it hands to the native side once and again after a stroke.

THE LOOKUP. `region_of` is a flat int32 array, face -> region index, -1 for
unpainted skin. The touch sets do not overlap (measured: max multiplicity
1 over 23,867 faces), and painting keeps it that way -- a face joining one
region leaves the other in the same step -- so a pick is one array index.
"""
import numpy
from pxr import Gf

import touchPoseNative


FAMILY = "touchpose:L0"
CONTROL_REL = "touchpose:control"
COLOR_ATTR = "touchpose:color"
HILIGHT_ATTR = "touchpose:hilight"
FACES_ATTR = "touchpose:faces"
PALETTE_ATTR = "touchpose:palette"
ALPHA_ATTR = "touchpose:alpha"
LEAD_COLOR_ATTR = "touchpose:leadColor"
SELECTED_COLOR_ATTR = "touchpose:selectedColor"

# What the three states look like when the file does not say. The hues
# are the studio's: `leadColor` in `touch_sets.touch` is a green and
# `selectedColor` a neutral, which is also the conventional kLeadSelected /
# kSelected vocabulary, so the port agrees with the tool it came from.
DEFAULT_LEAD_COLOR = (0.054, 0.420, 0.187)
DEFAULT_SELECTED_COLOR = (0.277, 0.277, 0.277)

# What the brightest channel of each state colour is scaled to. See
# TouchModel.StateColors for why it is a scale and not a mix to white.
LEAD_VALUE = 0.95
SELECTED_VALUE = 0.62
HOVER_VALUE = 1.0

# The paint brush, as a fraction of the mesh's bounding-box diagonal. On
# the biped's ~180 cm diagonal this is 3.6 cm, which covers a handful of
# faces on a limb and a dozen on the torso -- about a fingertip.
BRUSH_FRACTION = 0.02

# The typed properties, which are how the regions are found. Found BY TYPE,
# so a rig may park them wherever it likes -- beside the geometry they
# annotate, or inside its RigExecRoot.
REGIONS_TYPE = "RigExecTouchRegions"
REGION_TYPE = "RigExecTouchRegion"
TYPED_FACES = "rigExec:touch:faces"
TYPED_CONTROL = "rigExec:touch:control"
TYPED_MESH = "rigExec:touch:mesh"
TYPED_LAYER_NAME = "rigExec:touch:layerName"
TYPED_LAYER_ORDER = "rigExec:touch:layerOrder"


def FindRegionScopes(stage, mesh_path=None):
    """Every touch-region scope on the stage, by schema type.

    With `mesh_path`, only the scopes annotating THAT mesh: a scope names
    its mesh by relationship, so a stage with three characters keeps
    three sets of regions apart without any of them having to sit in a
    particular place.
    """
    if stage is None:
        return []
    found = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != REGIONS_TYPE:
            continue
        if mesh_path:
            rel = prim.GetRelationship(TYPED_MESH)
            targets = [str(t) for t in (rel.GetTargets() if rel else [])]
            if targets and str(mesh_path) not in targets:
                continue
        found.append(prim)
    return found


def LayerLabel(scope):
    """The name a scope goes by in the editor's layer switch."""
    attr = scope.GetAttribute(TYPED_LAYER_NAME) if scope else None
    name = attr.Get() if attr and attr.IsValid() else None
    return str(name) if name else (scope.GetName() if scope else "")


def FindLayers(stage, mesh_path=None):
    """The touch layers on *mesh_path*, in the order the editor lists them.

    One RigExecTouchRegions scope is one layer. A mesh may carry several
    -- a body set and a face set, say -- and exactly one is live at a
    time, which is what lets each own its own paint without the two
    having to agree about a face.

    Ordered by `rigExec:touch:layerOrder`, ties broken on the label and
    then the path so the list is total and a file with no order authored
    still opens the same way twice. The FIRST entry is the one that opens
    live.

    Scopes with no regions are dropped: an empty scope is a placeholder,
    and offering it in the switch would let the editor open on nothing.
    """
    layers = []
    for scope in FindRegionScopes(stage, mesh_path):
        if not any(c.GetTypeName() == REGION_TYPE or c.HasAttribute(FACES_ATTR)
                   for c in scope.GetChildren()):
            continue
        attr = scope.GetAttribute(TYPED_LAYER_ORDER)
        order = attr.Get() if attr and attr.IsValid() else None
        layers.append((int(order or 0), LayerLabel(scope),
                       str(scope.GetPath()), scope))
    layers.sort(key=lambda row: (row[0], row[1], row[2]))
    return [(label, path, scope) for _order, label, path, scope in layers]


def MeshOfScope(scope):
    """The mesh a region scope annotates, as a path string, or None.

    Its own call because the caller usually HAS the scope already --
    `FindLayers` hands it back -- and re-finding it means another full
    stage traversal. Measured on Biped_stack.usda, opening the three
    sets cost 72.3 ms with each one re-walking the stage to learn its
    mesh and 49.9 ms when the scope it already had was asked instead --
    against 45.2 ms for the body set on its own, so the two eyeballs
    add about 5 ms to a load.
    """
    if not scope:
        return None
    rel = scope.GetRelationship(TYPED_MESH)
    targets = rel.GetTargets() if rel else []
    if targets:
        return str(targets[0])
    attr = scope.GetAttribute("touchpose:mesh")
    value = attr.Get() if attr and attr.IsValid() else None
    return str(value) if value else None


def _MeshOfLayer(stage, layer=None):
    """The mesh a touch layer annotates, by label or scope path.

    Unset picks the first layer FindLayers offers, which is the one the
    editor opens live -- so the default answer is the default layer's
    mesh rather than a mesh named in advance.
    """
    scopes = FindRegionScopes(stage)
    if not scopes:
        return None
    chosen = None
    if layer:
        for scope in scopes:
            if str(scope.GetPath()) == str(layer) or LayerLabel(scope) == layer:
                chosen = scope
                break
    if chosen is None:
        layers = FindLayers(stage)
        if not layers:
            return None
        chosen = layers[0][2]
    return MeshOfScope(chosen)


def _TouchValue(prim, typed, custom):
    """A region property, typed first and custom as the fallback.

    A file exported before the schema existed carries only the custom
    `touchpose:*` attributes, and there is no reason to make somebody
    re-export to open one."""
    attr = prim.GetAttribute(typed)
    if attr and attr.IsValid():
        value = attr.Get()
        if value is not None and len(value):
            return value
    attr = prim.GetAttribute(custom)
    return attr.Get() if attr and attr.IsValid() else None


def _TouchTargets(prim, typed, custom):
    """A region relationship, typed first, custom as the fallback."""
    for name in (typed, custom):
        rel = prim.GetRelationship(name)
        if rel:
            targets = rel.GetTargets()
            if targets:
                return targets
    return []


class Region(object):
    """One touch set: a name, the faces it owns, and what it drives."""

    __slots__ = ("index", "name", "control", "color", "hilight", "faces")

    # `faces` is rebound wholesale rather than mutated in place: consumers
    # compare identity or length, and an in-place append would leave them
    # stale.

    def __init__(self, index, name, control, color, hilight, faces):
        self.index = index
        self.name = name
        self.control = control
        self.color = color
        self.hilight = hilight
        self.faces = faces

    @property
    def label(self):
        """What to show a human: the region, minus the `_touch` suffix."""
        return self.name[:-6] if self.name.endswith("_touch") else self.name

    def __repr__(self):
        return "<Region %s %d faces -> %s>" % (self.name, len(self.faces),
                                               self.control)


class TouchModel(object):
    """Regions + colours + a native posed mesh. Build once, sync per pose."""

    def __init__(self, counts, vertex_indices, regions, mesh_path=None,
                 scope_path=None, native=None, points=None):
        self.mesh_path = mesh_path
        # The regions scope this model was READ from. A save goes back
        # here rather than to a path re-derived from the mesh, or moving
        # the regions leaves a stage with two scopes and a reader picking
        # whichever it met first.
        self.scope_path = scope_path
        # The layer switch: every layer on this mesh as (label, path), and
        # the label of the one this model holds. Defaulted here so a model
        # built by hand -- every test does -- reads as a single unnamed
        # layer rather than raising on a missing attribute.
        self.layers = []
        self.layer_name = ""
        self.counts = numpy.asarray(counts, dtype=numpy.int32)
        self.vertex_indices = numpy.asarray(vertex_indices, dtype=numpy.int32)
        self.face_count = int(len(self.counts))
        self.regions = list(regions)

        # face -> region index, -1 for unpainted skin plus everything whose
        # region was skipped for having no control.
        self.region_of = numpy.full(self.face_count, -1, dtype=numpy.int32)
        for region in self.regions:
            if len(region.faces):
                faces = numpy.asarray(region.faces, dtype=numpy.int32)
                faces = faces[(faces >= 0) & (faces < self.face_count)]
                self.region_of[faces] = region.index

        self.dirty = False
        self.palette = []
        self.alpha = 1.0
        self.lead_color = DEFAULT_LEAD_COLOR
        self.selected_color = DEFAULT_SELECTED_COLOR

        if native is None:
            point_count = (len(points) if points is not None else
                           (int(self.vertex_indices.max()) + 1
                            if len(self.vertex_indices) else 0))
            native = touchPoseNative.NativeMesh.FromTopology(
                self.counts, self.vertex_indices, point_count, mesh_path)
        self.native = native
        if points is not None:
            self.native.SetPoints(points)
        self.native.SetFaceRegions(self.region_of, len(self.regions))
        self.PushColors()

    # -- construction ----------------------------------------------------

    @classmethod
    def FromStage(cls, stage, mesh_path=None,
                  family=FAMILY, require_control=True, layer=None):
        """Read the mesh and one of its touch layers off a composed stage.

        `require_control` drops a region that binds nothing: a region the
        click cannot act on must not swallow the click that would otherwise
        reach usdview's own picking.

        `layer` names which layer to open -- its label or its scope path.
        Unset opens the first one `FindLayers` lists, and a name that
        matches nothing falls back to that same first layer rather than
        opening empty, because a stale name in a saved session should not
        look like a rig with no regions.
        """
        from pxr import UsdGeom

        # The mesh comes from the LAYER when none is named. A scope states
        # the mesh its faces index, so the eyes' layer brings the eyeball
        # with it -- where a hardcoded body mesh could only ever highlight
        # the body, whatever layer was live.
        if mesh_path is None:
            mesh_path = _MeshOfLayer(stage, layer) or "/Biped/Geom/body_geo"

        prim = stage.GetPrimAtPath(mesh_path)
        if not prim or not prim.IsValid():
            raise ValueError("no mesh at %s" % mesh_path)
        mesh = UsdGeom.Mesh(prim)
        counts = mesh.GetFaceVertexCountsAttr().Get()
        vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()
        if counts is None or vertex_indices is None:
            raise ValueError("%s has no topology" % mesh_path)

        # TWO LAYOUTS: the typed RigExecTouchRegions scope, and the older
        # GeomSubsets under the mesh, still read so a file written before
        # the move keeps working.
        sources = []
        found_scope = None
        layers = FindLayers(stage, mesh_path)
        if layers:
            chosen = None
            if layer is not None:
                want = str(layer)
                chosen = next((s for label, path, s in layers
                               if want in (label, path)), None)
            found_scope = chosen if chosen is not None else layers[0][2]
            sources = [c for c in found_scope.GetChildren()
                       if c.GetTypeName() == REGION_TYPE
                       or c.HasAttribute(FACES_ATTR)]
        legacy = not sources

        regions = []
        for child in (sources or prim.GetChildren()):
            subset = None
            if legacy:
                subset = UsdGeom.Subset(child)
                if not subset:
                    continue
                if subset.GetFamilyNameAttr().Get() != family:
                    continue
            targets = _TouchTargets(child, TYPED_CONTROL, CONTROL_REL)
            control = str(targets[0]) if targets else None
            if require_control and not control:
                continue
            indices = (subset.GetIndicesAttr().Get() if legacy
                       else _TouchValue(child, TYPED_FACES, FACES_ATTR))
            color = child.GetAttribute(COLOR_ATTR).Get() \
                if child.HasAttribute(COLOR_ATTR) else None
            hilight = child.GetAttribute(HILIGHT_ATTR).Get() \
                if child.HasAttribute(HILIGHT_ATTR) else 0
            regions.append(Region(
                len(regions), child.GetName(), control,
                tuple(color) if color is not None else (1.0, 0.6, 0.0),
                int(hilight or 0),
                numpy.asarray(indices if indices is not None else [],
                              dtype=numpy.int32)))

        native = touchPoseNative.NativeMesh.FromStage(stage, mesh_path)
        model = cls(counts, vertex_indices, regions, mesh_path=mesh_path,
                    scope_path=(str(found_scope.GetPath())
                                if found_scope else None),
                    native=native)
        # What the switch offers and where it is standing. Carried on the
        # model so the panel does not have to re-walk the stage to draw
        # its own layer row.
        model.layers = [(label, path) for label, path, _s in layers]
        model.layer_name = LayerLabel(found_scope) if found_scope else ""
        # The palette lives on the scope the regions came from, or on the
        # mesh for the legacy layout. (This read the LAST scope the search
        # loop visited, which with two characters is not necessarily the
        # one whose regions were used.)
        model.ReadPalette(found_scope if found_scope else prim)
        return model

    def ReadPalette(self, prim):
        """Adopt the palette and the two state colours off a prim."""
        def _color(name, fallback):
            attr = prim.GetAttribute(name) if prim else None
            value = attr.Get() if attr and attr.IsValid() else None
            return tuple(value) if value is not None else fallback

        self.lead_color = _color(LEAD_COLOR_ATTR, DEFAULT_LEAD_COLOR)
        self.selected_color = _color(SELECTED_COLOR_ATTR,
                                     DEFAULT_SELECTED_COLOR)
        attr = prim.GetAttribute(ALPHA_ATTR) if prim else None
        value = attr.Get() if attr and attr.IsValid() else None
        self.alpha = float(value) if value is not None else 1.0
        attr = prim.GetAttribute(PALETTE_ATTR) if prim else None
        value = attr.Get() if attr and attr.IsValid() else None
        self.palette = [tuple(c) for c in value] if value else []
        self.PushColors()
        return self

    def Close(self):
        """Release the native mesh, and with it any highlight it drew."""
        if self.native is not None:
            self.native.Close()

    # -- the two colour sets ---------------------------------------------
    #
    # The `.touch` file carries TWO.
    #
    #   * `touchpose:color` per region -- the set's own `touchColor`,
    #     randomised by the authoring editor so every region differs. That
    #     is the EDIT set, right exactly when all regions are drawn at once.
    #
    #   * `touchpose:palette`, six colours on the scope, indexed per region
    #     by `touchpose:hilight`. That is the HIGHLIGHT set, drawn for the
    #     one region under the cursor.

    def HoverColor(self, region):
        """What the ONE region under the cursor is drawn in.

        SCALED to full value rather than mixed toward white: mixing
        desaturates, and a desaturated hover is a pale neutral -- which is
        exactly what SELECTED is.
        """
        return _ScaleTo(self.HilightColor(region), HOVER_VALUE)

    def HilightColor(self, region):
        """The region's palette colour, unscaled. Its own if no palette."""
        if not self.palette:
            return tuple(region.color)
        return tuple(self.palette[int(region.hilight) % len(self.palette)])

    def EditColor(self, region):
        """What a region is drawn in when ALL of them are drawn."""
        return _ScaleTo(region.color, HOVER_VALUE)

    def StateColors(self):
        """What LEAD and SELECTED look like, as {state: (r, g, b)}.

        Both are scaled up, by scaling rather than mixing toward white: a
        pure scale leaves chromaticity untouched, so lead stays exactly as
        green as authored and selected stays exactly neutral.
        """
        return {
            "lead": _ScaleTo(self.lead_color, LEAD_VALUE),
            "selected": _ScaleTo(self.selected_color, SELECTED_VALUE),
        }

    def ColorTables(self):
        """(hover, edit) per-region colour arrays, (N, 3) float32."""
        hover = numpy.zeros((len(self.regions), 3), dtype=numpy.float32)
        edit = numpy.zeros((len(self.regions), 3), dtype=numpy.float32)
        for region in self.regions:
            hover[region.index] = self.HoverColor(region)
            edit[region.index] = self.EditColor(region)
        return hover, edit

    def PushColors(self):
        """Hand the per-region colours to the native highlight."""
        if getattr(self, "native", None) is None:
            return
        hover, edit = self.ColorTables()
        self.native.SetRegionColors(hover, edit)

    def AllFaces(self):
        """Every painted face, once, in face order."""
        return numpy.nonzero(self.region_of >= 0)[0].astype(numpy.int32)

    def FaceColors(self, faces, editing=True):
        """One colour per face, as an (N, 3) float32 array. Unowned grey.

        A lookup table and one gather; the table's LAST row is the unowned
        colour, which is what makes `region_of`'s -1 index it for free.
        """
        faces = numpy.asarray(faces, dtype=numpy.int32)
        table = numpy.full((len(self.regions) + 1, 3), 0.5,
                           dtype=numpy.float32)
        for region in self.regions:
            table[region.index] = numpy.float32(
                self.EditColor(region) if editing
                else self.HoverColor(region))
        return table[self.region_of[faces]]

    # -- pose --------------------------------------------------------------

    def SyncPose(self, time=None, force=False):
        """Follow what the viewport draws. True when the mesh moved."""
        return self.native.SyncPose(time, force)

    def SetPoints(self, points):
        """Explicit posed points (local space), for a host with no rig."""
        self.native.SetPoints(points)
        return self

    @property
    def points(self):
        """The CURRENT local points, copied out -- for tests and tools."""
        return self.native.Points()

    @property
    def diagonal(self):
        bounds = self.native.Bounds()
        if bounds is None:
            return 0.0
        lo, hi = bounds
        return float(numpy.linalg.norm(numpy.subtract(hi, lo)))

    # -- the cast --------------------------------------------------------

    def Cast(self, origin, direction):
        """Nearest face along a world ray, as (face, t), or (-1, -1.0)."""
        length = float(numpy.linalg.norm(numpy.asarray(direction, float)))
        if length < 1e-12:
            return -1, -1.0
        return self.native.Cast(origin, direction)

    def RegionAt(self, origin, direction):
        """The `Region` the ray lands in, or None."""
        face, _t = self.Cast(origin, direction)
        return self.RegionOfFace(face)

    def RegionOfFace(self, face):
        if face < 0 or face >= self.face_count:
            return None
        index = int(self.region_of[face])
        return self.regions[index] if index >= 0 else None

    def FaceCentroid(self, face):
        """One face's WORLD centroid, from the current pose."""
        return self.native.FaceCentroid(face)

    def FacesOf(self, regions):
        """The union of several regions' faces, as one sorted array."""
        arrays = [numpy.asarray(r.faces, numpy.int32) for r in regions
                  if len(r.faces)]
        if not arrays:
            return numpy.zeros(0, numpy.int32)
        return numpy.unique(numpy.concatenate(arrays))

    def RegionsFor(self, paths):
        """Every region whose control is in `paths`, in region order."""
        wanted = set(str(p) for p in paths)
        return [r for r in self.regions if r.control in wanted]

    # -- marquee ---------------------------------------------------------

    def RegionsInRect(self, view_projection, width, height, eye,
                      x0, y0, x1, y1):
        """Regions with a FRONT-FACING face centroid inside the rect.

        TOUCHES, not encloses -- the same rule the Control Picker's marquee
        uses: requiring a region to be fully swept makes a long thin one
        (a finger, a clavicle) nearly uncatchable. `view_projection` is a
        row-vector world-to-clip Gf.Matrix4d; pixels are top-left, y down.
        """
        indices = self.native.RegionsInRect(view_projection, width, height,
                                            eye, x0, y0, x1, y1)
        return [self.regions[i] for i in indices
                if 0 <= i < len(self.regions)]

    # -- authoring -------------------------------------------------------

    @property
    def brush(self):
        """The paint radius in stage units, scaled to the mesh."""
        return max(self.diagonal * BRUSH_FRACTION, 1e-4)

    def Brush(self, point, direction=None, radius=None):
        """Faces within the brush of a world point, as an index array.

        A world-space ball, and faces whose normal points away from the
        view ray are dropped, so a stroke on the front of a thigh does not
        paint the back of it as well.
        """
        radius = self.brush if radius is None else float(radius)
        return self.native.Brush(point, direction, radius)

    def _PushRegions(self):
        self.native.SetFaceRegions(self.region_of, len(self.regions))

    def SnapshotFaces(self):
        """The face -> region table as it stands, for an undo entry.

        A whole int32 copy per stroke: the table is one entry per face,
        so on a 26k-face character that is 100 kB, taken once when the
        brush goes down. Recording only the faces a dab touched would be
        smaller and wrong -- a dab takes faces OFF their previous owners,
        so restoring it means restoring the regions' own face lists too,
        and those are rebuilt from this table by RestoreFaces.
        """
        return numpy.array(self.region_of, dtype=numpy.int32)

    def RestoreFaces(self, table):
        """Put the face -> region table back, and the regions with it."""
        self.region_of = numpy.array(table, dtype=numpy.int32)
        for region in self.regions:
            region.faces = numpy.flatnonzero(
                self.region_of == region.index).astype(numpy.int32)
        self.dirty = True
        self._PushRegions()

    def AssignFaces(self, region, faces):
        """Give `faces` to `region`, taking them off whoever had them.

        Returns the regions that changed, `region` included.
        """
        faces = numpy.unique(numpy.asarray(faces, dtype=numpy.int32))
        faces = faces[(faces >= 0) & (faces < self.face_count)]
        if not len(faces):
            return []
        previous = numpy.unique(self.region_of[faces])
        self.region_of[faces] = region.index
        touched = [region]
        for index in previous.tolist():
            if index < 0 or index == region.index:
                continue
            donor = self.regions[index]
            donor.faces = numpy.setdiff1d(donor.faces, faces,
                                          assume_unique=False)
            touched.append(donor)
        region.faces = numpy.union1d(region.faces, faces).astype(numpy.int32)
        self.dirty = True
        self._PushRegions()
        return touched

    def EraseFaces(self, faces):
        """Take `faces` out of whatever region owns them."""
        faces = numpy.unique(numpy.asarray(faces, dtype=numpy.int32))
        faces = faces[(faces >= 0) & (faces < self.face_count)]
        owners = numpy.unique(self.region_of[faces])
        owners = owners[owners >= 0]
        if not len(owners):
            return []
        self.region_of[faces] = -1
        touched = []
        for index in owners.tolist():
            region = self.regions[index]
            region.faces = numpy.setdiff1d(region.faces, faces,
                                           assume_unique=False)
            touched.append(region)
        self.dirty = True
        self._PushRegions()
        return touched

    def Rows(self):
        """The regions as plain tuples, for the writer. Empty ones dropped."""
        return [(r.name, r.control, tuple(r.color), int(r.hilight),
                 r.faces.tolist())
                for r in self.regions if len(r.faces)]

    # -- reporting -------------------------------------------------------

    def Coverage(self):
        """(regions, faces covered, faces of the mesh) -- for the status."""
        covered = int((self.region_of >= 0).sum())
        return len(self.regions), covered, self.face_count


def RayThroughPixel(frustum, x, y, width, height):
    """A world-space ray through a pixel of a viewport, as (origin, dir).

    Pixel coordinates are Qt's -- origin top-left, y down -- and the
    frustum's normalised window is origin-centre, y UP, which is the flip
    in the middle here.
    """
    if width <= 0 or height <= 0:
        return None
    nx = (2.0 * (x + 0.5) / float(width)) - 1.0
    ny = 1.0 - (2.0 * (y + 0.5) / float(height))
    ray = frustum.ComputePickRay(Gf.Vec2d(nx, ny))
    start = ray.startPoint
    direction = ray.direction
    return ((float(start[0]), float(start[1]), float(start[2])),
            (float(direction[0]), float(direction[1]), float(direction[2])))


def _ScaleTo(color, value):
    """Scale a colour so its brightest channel is `value`.

    A pure scale, so hue and saturation are exactly the authored ones. A
    black input comes back as a neutral at `value`, which is at least
    visible.
    """
    peak = max(color)
    if peak <= 1e-6:
        return (value, value, value)
    factor = value / peak
    return tuple(min(1.0, c * factor) for c in color)
