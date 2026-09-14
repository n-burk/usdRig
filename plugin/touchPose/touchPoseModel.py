"""TouchPose's pick loop, with no Qt in it.

The panel in `touchPoseUI` is the mouse, the menu and the session-layer
authoring; everything that has an answer a test can assert lives here and
is driven by plain arrays. That split is the one `avarEditorModel` /
`avarEditorUI` already use, and it is what lets the arithmetic below be
tested without an app: `tests/python/test_touchpose_model.py` builds a
six-face cube, binds two regions to it, and casts rays at it.

THE LOOP, and why each step is the shape it is -- Phase 0 measured all
four numbers on the real 26,274-face biped, so none of this is a guess:

  1. DEFORMED POINTS. `body_geo` is skinned by RigExec and its posed
     points live in Hydra, not on the stage: `GetPointsAttr().Get()`
     returns the REST mesh and would pick the wrong face the moment the
     character moves. The points come in through `SetPoints`, which the
     UI feeds from `HydraObserver` (0.10 ms) -- and only when the rig's
     generation counter moves, because rebuilding the derived arrays
     costs 6.13 ms and nothing changes them between poses.

  2. FACE UNDER THE CURSOR, by exact ray-triangle over the whole mesh.
     Not nearest-centroid: on a real pick that answered face 96 where the
     exact cast answered 19,667, because the nearest centroid to a point
     on a limb is frequently on the FAR side of that limb. The exact cast
     is also the cheaper of the two once the triangle edges are cached --
     2.63 ms against `StageView.pick()`'s 3.71 ms -- and it needs no GL
     context, which is why a headless test can run it.

  3. FACE -> REGION, a flat int32 array rather than a search, because the
     touch sets provably do not overlap (`nonOverlapping`, measured: max
     multiplicity 1 over 23,867 faces). 0.0002 ms.

  4. REGION -> OVERLAY GEOMETRY. The highlight cannot ride `displayColor`
     on `body_geo`: it is bound to `UsdPreviewSurface`, Storm reads
     `diffuseColor`, and the primvar moved 0 of 80,730 sampled pixels
     with the materials bound against 7,383 with them blocked. So the
     region is drawn as a SECOND MESH -- its own faces, its own points,
     no material binding at all, so Storm's fallback material applies and
     that one does read `displayColor`. `OverlayGeometry` builds it.

AND A SECOND MESH IS STILL THE ANSWER, though not for R2's reason.
Asked for a highlight that "feels like a shader over the mesh", spike R5
found the shader: a `UsdPrimvarReader_float3` spliced into each bound
material's `diffuseColor` in the session layer, reading a per-face tint
primvar. It renders exactly right -- with the tint set to the asset's
own colours it is 53 of 80,730 pixels from the untouched asset, which is
the frame's own noise floor, and with a region lit it tints that region
on the body's own shaded surface with no geometry added.

It costs 164 ms per region crossing, against the mesh overlay's 8.9 ms.
The control in that spike is what settles it: the identical 26,274-value
array costs 2.0 ms authored on a root-level prim and 164.2 ms authored
on `body_geo`, with RigExec's generation counter moving every time. A
tint has to live on the mesh, the mesh is inside the rig's read roots,
and every edit there re-runs the rig. Getting the tint to Hydra without
going through the stage is a scene-index filter in rigExecImaging, not
something this layer can do -- so the overlay stays, and R6 took its
lift down by 5.5x to 15x instead (see OFFSET_FRACTION).
"""
import numpy
from pxr import Gf


# How far the overlay mesh is pushed off the skin, as a fraction of the
# REGION's own bounding-box diagonal -- not the whole mesh's, which is
# what this used to be and is why the lift was "way too noticeable". One
# constant over the body gave a fingertip the same 0.34 cm as the torso,
# and 0.34 cm is a third of a fingertip.
#
# Spike R6 swept the lift and counted, per offset, how many of the
# patch's own pixels lost the depth test to the body it lies on. The
# z-fight is gone by 0.005 cm and the curve is FLAT from there to the
# 0.3365 cm that shipped:
#
#     lift (cm)      0.000  0.001  0.002  0.005  0.010 ... 0.336
#     hips_touch     98.5%  35.7%  15.7%  11.7%  11.7%     11.7%
#     toe_r_touch    95.4%  39.3%  30.1%  30.2%  30.2%     27.8%
#     face_lower     91.5%  69.1%  67.0%  67.0%  67.0%     67.0%
#
# (The floor each region settles on is geometry genuinely in front of
# the patch -- an arm across a thigh -- which no lift can win back.)
#
# So the constraint the old number existed for is satisfied 67x over.
# Keeping the same 0.0015 against the REGION's diagonal gives hips
# 0.061 cm, toe_r 0.024 cm and a fingertip the floor below -- 5.5x to
# 14x less lift than shipped, all of it still 5x clear of the measured
# z-fight threshold.
OFFSET_FRACTION = 0.0015

# ...with a floor, as a fraction of the MESH's diagonal, because a
# region can be arbitrarily small (three faces on a nail) and the depth
# buffer's resolution does not shrink with it. 0.0224 cm on the biped,
# 4.5x the 0.005 cm R6 measured as enough.
MIN_OFFSET_FRACTION = 0.0001

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
# faces on a limb and a dozen on the torso -- about a fingertip, which is
# what a fingertip-sized brush feels like.
BRUSH_FRACTION = 0.02
# The typed properties, which are how the regions are found. There is
# deliberately no path constant here any more: this was
# `SCOPE_PATH = "/Biped/TouchPose"`, an absolute path with one
# character's name in it, so TouchPose worked for exactly one asset and
# failed silently on every other. Regions are found BY TYPE now, which
# also means a rig may park them wherever it likes -- beside the geometry
# they annotate, or inside its RigExecRoot.
REGIONS_TYPE = "RigExecTouchRegions"
REGION_TYPE = "RigExecTouchRegion"
TYPED_FACES = "rigExec:touch:faces"
TYPED_CONTROL = "rigExec:touch:control"
TYPED_MESH = "rigExec:touch:mesh"


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

    # `faces` is rebound wholesale rather than mutated in place: every
    # consumer (the overlay key, the region_of table) compares identity
    # or length, and an in-place append would leave both stale.

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
    """Topology + regions + the cast. Build once, re-point per pose."""

    def __init__(self, counts, vertex_indices, regions, mesh_path=None,
                 scope_path=None):
        self.mesh_path = mesh_path
        # The regions scope this model was READ from. A save goes back
        # here rather than to a path re-derived from the mesh, or moving
        # the regions leaves a stage with two scopes and a reader picking
        # whichever it met first.
        self.scope_path = scope_path
        self.counts = numpy.asarray(counts, dtype=numpy.int32)
        self.vertex_indices = numpy.asarray(vertex_indices, dtype=numpy.int32)
        self.face_count = int(len(self.counts))
        self.regions = list(regions)

        # Where each face's corners start in `vertex_indices`.
        self._starts = numpy.concatenate(
            ([0], numpy.cumsum(self.counts)[:-1])).astype(numpy.int32)

        self._Triangulate()

        # face -> region index, -1 for the ~30% of `body_geo` nobody
        # painted (soles, inner mouth, scalp under hair) plus everything
        # whose region was skipped for having no control.
        self.region_of = numpy.full(self.face_count, -1, dtype=numpy.int32)
        for region in self.regions:
            if len(region.faces):
                self.region_of[region.faces] = region.index

        self.points = None
        self._centroids = None
        self.dirty = False
        self._V0 = self._e1 = self._e2 = None
        self._lo = self._hi = None
        self._diagonal = 0.0
        self.palette = []
        self.alpha = 1.0
        self.lead_color = DEFAULT_LEAD_COLOR
        self.selected_color = DEFAULT_SELECTED_COLOR

    # -- construction ----------------------------------------------------

    @classmethod
    def FromStage(cls, stage, mesh_path="/Biped/Geom/body_geo",
                  family=FAMILY, require_control=True):
        """Read the mesh and its touch subsets off a composed stage.

        `require_control` drops a subset that binds nothing. The importer
        now skips those at authoring time, so on a freshly imported layer
        this filters nothing -- it is here because an older
        `Biped_touch.usda` (or one made with `--keep-unbound`) still has
        them, and a region the click cannot act on must not swallow the
        click that would otherwise reach usdview's own picking.
        """
        from pxr import UsdGeom

        prim = stage.GetPrimAtPath(mesh_path)
        if not prim or not prim.IsValid():
            raise ValueError("no mesh at %s" % mesh_path)
        mesh = UsdGeom.Mesh(prim)
        counts = mesh.GetFaceVertexCountsAttr().Get()
        vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()
        if counts is None or vertex_indices is None:
            raise ValueError("%s has no topology" % mesh_path)

        # TWO LAYOUTS. The regions now live on a sibling `/…/TouchPose`
        # scope, because a face GeomSubset under the mesh is collected by
        # hdSt whatever its family and collided with the materialBind
        # subsets -- 16,739 warnings on open, for data the renderer never
        # uses. The old subset layout is still read so a `.usda` written
        # before the move keeps working.
        sources = []
        # Bound even when the stage has no typed scope at all: the legacy
        # GeomSubset layout leaves this loop without entering it, and
        # ReadPalette below still has to be told what to read from.
        scope = None
        found_scope = None
        for scope in FindRegionScopes(stage, mesh_path):
            sources.extend(c for c in scope.GetChildren()
                           if c.GetTypeName() == REGION_TYPE
                           or c.HasAttribute(FACES_ATTR))
            if sources:
                found_scope = scope
                break
        legacy = not sources

        regions = []
        for child in (sources or prim.GetChildren()):
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

        model = cls(counts, vertex_indices, regions, mesh_path=mesh_path,
                    scope_path=(str(found_scope.GetPath())
                                if found_scope else None))
        model.ReadPalette(scope if (scope and scope.IsValid()) else prim)
        points = mesh.GetPointsAttr().Get()
        if points is not None:
            model.SetPoints(points)
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
        return self

    # -- the two colour sets ---------------------------------------------
    #
    # The `.touch` file carries TWO, and the port only ever used one.
    #
    #   * `touchpose:color` per region -- the set's own `touchColor`,
    #     which the studio's editor randomises so that every region is a
    #     different colour. That is the EDIT set, and it is the right one
    #     exactly when all the regions are drawn at once, because then
    #     telling one from its neighbour is the whole job.
    #
    #   * `touchpose:palette`, six colours on the scope, indexed per
    #     region by `touchpose:hilight` (0..5; the shipped file is 148
    #     sets at 0, 76 at 3, 13 at 2, 10 at 1). That is the HIGHLIGHT
    #     set, and the the conventional tool shape picks `shaders[colorIndex]` out of it
    #     for the one item under the cursor
    #     (src/TouchPose/touchShape.cpp:3083-3098).
    #
    # Six colours over 247 sets means neighbours frequently share one,
    # which would be fatal if they were drawn together and costs nothing
    # when they are not: in control mode exactly one region is ever lit.
    # So the mode chooses the set, which is what
    # `touch_sets_tree_widget._apply_decoration_colors` does in the
    # original ("Edit mode -> random shader color... Normal -> highlight
    # palette color").

    def HoverColor(self, region):
        """What the ONE region under the cursor is drawn in.

        The highlight set: `touchpose:palette[touchpose:hilight]`, which
        is what the the conventional tool shape draws for a hovered item. Falls back to
        the region's own colour when the layer carries no palette, so an
        older touch file still lights up.

        SCALED to full value rather than mixed toward white, for a
        reason beyond brightness: mixing desaturates, and a desaturated
        hover is a pale neutral -- which is exactly what SELECTED is. On
        `thigh_fk_l` the mixed version came out (202, 192, 201) on screen
        against selected's (179, 177, 177), 20.7 apart on a threshold of
        20. Scaling keeps the hue at full strength and the two states a
        long way apart whatever colour the region was painted.
        """
        return _ScaleTo(self.HilightColor(region), HOVER_VALUE)

    def HilightColor(self, region):
        """The region's palette colour, unscaled. Its own if no palette."""
        if not self.palette:
            return tuple(region.color)
        return tuple(self.palette[int(region.hilight) % len(self.palette)])

    def EditColor(self, region):
        """What a region is drawn in when ALL of them are drawn.

        The region's own `touchpose:color`, scaled the same way the
        hover is -- the edit set exists to separate neighbours, and two
        adjacent regions that were painted 0.03 apart in the file are
        0.03 apart on screen unless they are lifted.
        """
        return _ScaleTo(region.color, HOVER_VALUE)

    def AllFaces(self):
        """Every painted face, once, in face order."""
        return numpy.nonzero(self.region_of >= 0)[0].astype(numpy.int32)

    def FaceColors(self, faces, editing=True):
        """One colour per face of a patch, as an (N, 3) float32 array.

        For the edit-mode patch, which is every region at once and so
        cannot be one flat colour. Faces with no region -- there are
        none in `AllFaces`, but a caller may hand over any list -- come
        back mid-grey rather than black, which would read as a hole.

        A LOOKUP TABLE and one gather, not a mask per region: the masked
        version is one pass over the whole face list per region, which on
        the edit patch is 98 passes over 16,739 faces and measured as the
        bulk of a 37.9 ms brush dab. The table's LAST row is the
        unowned colour, which is what makes `region_of`'s -1 index it for
        free.
        """
        faces = numpy.asarray(faces, dtype=numpy.int32)
        table = numpy.full((len(self.regions) + 1, 3), 0.5,
                           dtype=numpy.float32)
        for region in self.regions:
            table[region.index] = numpy.float32(
                self.EditColor(region) if editing
                else self.HoverColor(region))
        return table[self.region_of[faces]]

    def FaceOffsets(self, faces):
        """The per-face lift for a patch that holds SEVERAL regions.

        `OffsetFor` answers for one region and the resident overlay is
        every region at once, so the lift has to travel with the face
        rather than with the patch -- otherwise one number has to serve
        a 12-face finger and a 1,200-face torso, and the sweep behind
        OFFSET_FRACTION says it cannot.

        The same lookup-table-and-gather shape as `FaceColors`, and for
        the same reason: the table's LAST row is the floor, which is
        what makes `region_of`'s -1 index it for free.
        """
        faces = numpy.asarray(faces, dtype=numpy.int32)
        table = numpy.full(len(self.regions) + 1, self.min_offset,
                           dtype=numpy.float32)
        for region in self.regions:
            if len(region.faces):
                table[region.index] = self.OffsetFor(region)
        return table[self.region_of[faces]]

    def StateColors(self):
        """What LEAD and SELECTED look like, as {state: (r, g, b)}.

        HOVER is per-region and handled elsewhere. These two come from
        the `.touch` file: `leadColor` is a green and `selectedColor` a
        neutral grey, which is also the conventional kLeadSelected / kSelected
        reading, so the port agrees with the tool the data came from.

        BOTH ARE SCALED UP, and by scaling rather than by mixing toward
        white. The authored values are dark -- lead (0.054, 0.420,
        0.187), selected (0.277, 0.277, 0.277) -- because the the conventional tool tool
        draws them flat over an unshaded viewport, where here they sit on
        a shaded body and have to beat it. Mixing toward white was tried
        first and is wrong: at enough lift to be visible it dragged both
        colours to nearly the same pale value, 0.128 apart in RGB, which
        is the one thing these two must never be. A pure scale leaves
        chromaticity untouched, so lead stays exactly as green as it was
        authored and selected stays exactly neutral, and the pair ends up
        0.63 apart. LEAD_VALUE is the brighter of the two on purpose: the
        lead is the one the animator just touched.
        """
        return {
            "lead": _ScaleTo(self.lead_color, LEAD_VALUE),
            "selected": _ScaleTo(self.selected_color, SELECTED_VALUE),
        }

    def _Triangulate(self):
        """Fan-triangulate once; every triangle remembers its face.

        A quad becomes two triangles that carry the SAME face index, so a
        triangle hit is a face answer with no second lookup. The index
        arithmetic is deliberately ugly: the readable version (one
        `numpy.arange` per face in a comprehension) measured 37 ms on
        `body_geo` against 1 ms for this.
        """
        counts = self.counts
        tri_per_face = counts - 2
        total = int(tri_per_face.sum())
        self.tri_face = numpy.repeat(
            numpy.arange(self.face_count, dtype=numpy.int32), tri_per_face)
        offsets = numpy.concatenate(([0], numpy.cumsum(tri_per_face)[:-1]))
        local = (numpy.arange(total, dtype=numpy.int32)
                 - numpy.repeat(offsets, tri_per_face).astype(numpy.int32))
        base = numpy.repeat(self._starts, tri_per_face)
        self.tris = numpy.stack([self.vertex_indices[base],
                                 self.vertex_indices[base + local + 1],
                                 self.vertex_indices[base + local + 2]],
                                axis=1)

    # -- per-pose --------------------------------------------------------

    def SetPoints(self, points):
        """Adopt a new set of deformed points and recache what they imply.

        Called on the rig's generation signal, NOT per hover: the read
        itself is 0.10 ms but the derived arrays below are 6.13 ms, and
        nothing between two poses can change them.
        """
        P = numpy.asarray(points, dtype=numpy.float32)
        if P.ndim != 2 or P.shape[1] != 3:
            raise ValueError("points must be Nx3, got %r" % (P.shape,))
        self.points = P
        self._centroids = None      # the pose moved; the brush must too
        self._V0 = P[self.tris[:, 0]]
        self._e1 = P[self.tris[:, 1]] - self._V0
        self._e2 = P[self.tris[:, 2]] - self._V0
        lo, hi = P.min(axis=0), P.max(axis=0)
        self._lo, self._hi = lo, hi
        self._diagonal = float(numpy.linalg.norm(hi - lo))
        return self

    @property
    def min_offset(self):
        """The smallest lift any patch gets, in stage units.

        A floor, not the lift: `OverlayGeometry` scales the lift to the
        region it is drawing. See OFFSET_FRACTION for the sweep.
        """
        return max(self._diagonal * MIN_OFFSET_FRACTION, 1e-5)

    def OffsetFor(self, region):
        """How far off the skin ONE region's patch sits, in stage units.

        Here rather than only inside `OverlayGeometry` so a test and the
        panel can ask the same question the drawing code answers.
        """
        faces = numpy.asarray(getattr(region, "faces", region),
                              dtype=numpy.int32)
        if self.points is None or not len(faces):
            return self.min_offset
        return self._Lift(self.points[self._Corners(faces)[0]])

    def _Lift(self, points):
        """The lift for a patch, from the patch's own extent."""
        span = points.max(axis=0) - points.min(axis=0)
        return max(float(numpy.linalg.norm(span)) * OFFSET_FRACTION,
                   self.min_offset)

    # -- the cast --------------------------------------------------------

    def Cast(self, origin, direction):
        """Nearest front face along the ray, as (face, t), or (-1, -1).

        Vectorised Moller-Trumbore over every triangle, with the edges
        already cached by `SetPoints`. No spatial index: 52,548 triangles
        is 2.63 ms flat, which is 16% of a 16.7 ms frame and cheaper than
        the GPU pick it replaces -- a BVH would be faster still and is
        one more thing to keep in sync with the pose.
        """
        if self._V0 is None:
            return -1, -1.0
        origin = numpy.asarray(origin, dtype=numpy.float32)
        direction = numpy.asarray(direction, dtype=numpy.float32)
        length = float(numpy.linalg.norm(direction))
        if length < 1e-12:
            return -1, -1.0
        direction = direction / length

        # THE BOX FIRST. Most hovers are not over the character at all --
        # a grid over the viewport found 2,041 empty samples for every
        # 160 on a region -- and every one of them used to pay the full
        # 52,548-triangle cast at 2.6 ms. A slab test against the mesh's
        # own bounds is six comparisons and answers those for free.
        if not self._HitsBounds(origin, direction):
            return -1, -1.0

        pv = numpy.cross(direction, self._e2)
        det = numpy.einsum("ij,ij->i", self._e1, pv)
        live = numpy.abs(det) > 1e-8
        inv = numpy.zeros_like(det)
        inv[live] = 1.0 / det[live]
        tv = origin - self._V0
        u = numpy.einsum("ij,ij->i", tv, pv) * inv
        qv = numpy.cross(tv, self._e1)
        v = numpy.einsum("ij,ij->i", numpy.broadcast_to(direction, tv.shape),
                         qv) * inv
        t = numpy.einsum("ij,ij->i", self._e2, qv) * inv
        hit = live & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t > 1e-4)
        if not hit.any():
            return -1, -1.0
        rows = numpy.nonzero(hit)[0]
        nearest = rows[int(numpy.argmin(t[rows]))]
        return int(self.tri_face[nearest]), float(t[nearest])

    def _HitsBounds(self, origin, direction):
        """Slab test of a normalised ray against the mesh's own bounds.

        Padded by the overlay's floor lift so a ray that grazes the
        silhouette -- which is exactly where a real pick lands -- is
        never rejected by a rounding error at the box face. A
        division-free form is not worth it here: this runs once per
        hover and the array it saves is 52,548 rows long.
        """
        if self._lo is None:
            return True
        pad = self.min_offset
        lo = self._lo - pad
        hi = self._hi + pad
        near, far = -numpy.inf, numpy.inf
        for axis in range(3):
            d = float(direction[axis])
            o = float(origin[axis])
            if abs(d) < 1e-12:
                if o < lo[axis] or o > hi[axis]:
                    return False
                continue
            t0 = (lo[axis] - o) / d
            t1 = (hi[axis] - o) / d
            if t0 > t1:
                t0, t1 = t1, t0
            near = max(near, t0)
            far = min(far, t1)
            if near > far:
                return False
        return far >= 0.0

    def RegionAt(self, origin, direction):
        """The `Region` the ray lands in, or None. Step 2 + step 3."""
        face, _t = self.Cast(origin, direction)
        return self.RegionOfFace(face)

    def RegionOfFace(self, face):
        if face < 0 or face >= self.face_count:
            return None
        index = int(self.region_of[face])
        return self.regions[index] if index >= 0 else None

    def FaceCentroid(self, face):
        """One face's centre, from the CURRENT points."""
        start = int(self._starts[face])
        size = int(self.counts[face])
        corners = self.vertex_indices[start:start + size]
        return self.points[corners].mean(axis=0)

    # -- the highlight ---------------------------------------------------

    def FacesOf(self, regions):
        """The union of several regions' faces, as one sorted array.

        Sorted and de-duplicated even though the regions provably do not
        overlap: two selected regions can share a control, and the same
        region would then be counted twice.
        """
        arrays = [numpy.asarray(r.faces, numpy.int32) for r in regions
                  if len(r.faces)]
        if not arrays:
            return numpy.zeros(0, numpy.int32)
        return numpy.unique(numpy.concatenate(arrays))

    def RegionsFor(self, paths):
        """Every region whose control is in `paths`, in region order."""
        wanted = set(str(p) for p in paths)
        return [r for r in self.regions if r.control in wanted]

    def OverlayGeometry(self, region, offset=None):
        """The region's faces as a standalone mesh, lifted off the skin.

        Returns (points, faceVertexCounts, faceVertexIndices) as numpy
        arrays -- arrays, not lists, because the caller feeds them
        straight to `Vt.*Array.FromNumpy` and a detour through Python
        lists is pure cost on a thousand-face region.

        The points are the region's own corners only -- a few hundred,
        not 26k -- each displaced along the average normal of the region
        faces that share it, so the patch clears the body it is lying on
        instead of z-fighting with it. Displacing along the NORMAL rather
        than toward the camera keeps this function camera-free and
        therefore testable; on a closed surface the viewer is on the
        outward side anyway, so the two agree in the case that matters.

        FULLY VECTORISED, and that is not a style preference. The
        readable version -- one loop iteration per face accumulating its
        normal -- measured 24 ms of a 27 ms hover on a 1,000-face thigh
        region, against a 16.7 ms frame. Written this way the same call
        is under a millisecond.
        """
        if self.points is None:
            raise RuntimeError("SetPoints first")
        # A `Region`, or a bare face array -- the selection highlight
        # draws SEVERAL regions as one patch, and one prim carrying the
        # union costs one Hydra resync where one prim per region would
        # cost as many as the animator has selected.
        faces = numpy.asarray(getattr(region, "faces", region),
                              dtype=numpy.int32)
        empty = numpy.zeros(0, numpy.int32)
        if not len(faces):
            return numpy.zeros((0, 3), numpy.float32), empty, empty
        unique, remap, counts, offsets = self._Corners(faces)
        local = self.points[unique].astype(numpy.float32)

        # Per-face normal from the first corner triangle, accumulated
        # onto every corner of that face. Newell would be more correct
        # for a concave quad; on a polygon cage at this scale the
        # difference is far below the offset itself.
        first = offsets.astype(numpy.int32)
        a = local[remap[first]]
        b = local[remap[first + 1]]
        c = local[remap[first + 2]]
        faceNormals = numpy.cross(b - a, c - a)
        lengths = numpy.linalg.norm(faceNormals, axis=1)
        safe = lengths > 1e-12
        faceNormals[safe] /= lengths[safe][:, None]

        normals = numpy.zeros_like(local)
        # `add.at` rather than `normals[remap] +=`: fancy-index assignment
        # keeps only the LAST write per repeated index, which would give
        # every shared corner one face's normal instead of the average.
        numpy.add.at(normals, remap,
                     numpy.repeat(faceNormals, counts, axis=0))
        lengths = numpy.linalg.norm(normals, axis=1)
        safe = lengths > 1e-9
        normals[safe] /= lengths[safe][:, None]

        # From the PATCH's own extent, not the body's: see
        # OFFSET_FRACTION for the sweep that says why.
        if offset is None:
            lift = self._Lift(local)
        else:
            lift = numpy.asarray(offset, dtype=numpy.float32)
            if lift.ndim == 0:
                lift = float(lift)
            else:
                # One lift PER FACE, which the resident overlay needs:
                # it carries every region and they do not agree. Scatter
                # it onto the corners taking the LARGEST claim, so a
                # vertex two regions share clears both of them --
                # `maximum.at` and not fancy indexing for the same
                # reason the normals above use `add.at`.
                if len(lift) != len(counts):
                    raise ValueError("%d offsets for %d faces"
                                     % (len(lift), len(counts)))
                perVertex = numpy.zeros(len(local), numpy.float32)
                numpy.maximum.at(perVertex, remap,
                                 numpy.repeat(lift, counts))
                lift = perVertex[:, None]
        return (local + normals * lift, counts.astype(numpy.int32), remap)

    def _Corners(self, faces):
        """The corner indices of `faces`, compacted and renumbered.

        Returns (unique vertex indices, per-corner remap into them,
        per-face counts, per-face start offsets into the remap). Split
        out of `OverlayGeometry` so `OffsetFor` can ask what a patch's
        extent is without building the patch.
        """
        counts = self.counts[faces]
        starts = self._starts[faces]
        total = int(counts.sum())
        offsets = numpy.concatenate(([0], numpy.cumsum(counts)[:-1]))
        step = (numpy.arange(total, dtype=numpy.int32)
                - numpy.repeat(offsets, counts).astype(numpy.int32))
        corners = self.vertex_indices[numpy.repeat(starts, counts) + step]
        # Compact to the unique vertices this patch touches, and
        # renumber -- the overlay must not carry 26k points to draw 200.
        unique, remap = numpy.unique(corners, return_inverse=True)
        return unique, remap.astype(numpy.int32).reshape(-1), counts, offsets

    # -- marquee ---------------------------------------------------------

    def FacePixels(self, viewProjection, width, height):
        """Every face centroid in PHYSICAL pixels, plus a validity mask.

        `viewProjection` is a 4x4 row-vector matrix (USD's convention) as
        a nested sequence, so this stays Qt-free and Gf-free -- the
        caller pulls it off the frustum. The mask is false for centroids
        behind the eye, which project to a mirrored point in front of it
        and would otherwise be caught by a marquee on the far side of the
        frame.
        """
        centroids = self.Centroids()
        matrix = numpy.asarray(viewProjection, dtype=numpy.float64)
        homogeneous = numpy.empty((len(centroids), 4), dtype=numpy.float64)
        homogeneous[:, :3] = centroids
        homogeneous[:, 3] = 1.0
        clip = homogeneous @ matrix
        w = clip[:, 3]
        valid = w > 1e-9
        safe = numpy.where(valid, w, 1.0)
        ndc = clip[:, :2] / safe[:, None]
        pixels = numpy.empty_like(ndc)
        pixels[:, 0] = (ndc[:, 0] * 0.5 + 0.5) * width - 0.5
        pixels[:, 1] = (0.5 - ndc[:, 1] * 0.5) * height - 0.5
        return pixels.astype(numpy.float32), valid

    def FrontFacing(self, eye):
        """Mask of faces whose normal points back toward `eye`.

        Without it a marquee over the front of the torso also catches
        every region on the BACK of it, because a centroid is just a
        point and the band has no depth. Exactly the same normal
        approximation the brush and the overlay patch use.
        """
        if self.points is None:
            raise RuntimeError("SetPoints first")
        v = self.vertex_indices
        starts = self._starts
        a = self.points[v[starts]]
        b = self.points[v[starts + 1]]
        c = self.points[v[starts + 2]]
        normals = numpy.cross(b - a, c - a)
        toEye = numpy.asarray(eye, dtype=numpy.float32) - a
        return numpy.einsum("ij,ij->i", normals, toEye) > 0.0

    def RegionsInRect(self, pixels, mask, x0, y0, x1, y1):
        """Regions with at least one masked-in face centroid in the rect.

        TOUCHES, not encloses -- the same rule the Control Picker's
        marquee uses, and for the same reason: requiring a region to be
        fully swept makes a long thin one (a finger, a clavicle) nearly
        uncatchable.
        """
        lo_x, hi_x = (x0, x1) if x0 <= x1 else (x1, x0)
        lo_y, hi_y = (y0, y1) if y0 <= y1 else (y1, y0)
        inside = (mask
                  & (pixels[:, 0] >= lo_x) & (pixels[:, 0] <= hi_x)
                  & (pixels[:, 1] >= lo_y) & (pixels[:, 1] <= hi_y))
        caught = numpy.unique(self.region_of[inside])
        return [self.regions[i] for i in caught.tolist() if i >= 0]

    # -- authoring -------------------------------------------------------

    def Centroids(self):
        """Every face's centre, from the CURRENT points. Cached per pose.

        6.13 ms to build on `body_geo`, which is why it is not built at
        load: only painting needs it, and a session that never paints
        never pays. Invalidated by `SetPoints`.
        """
        if self._centroids is not None:
            return self._centroids
        if self.points is None:
            raise RuntimeError("SetPoints first")
        out = numpy.empty((self.face_count, 3), dtype=numpy.float32)
        for size in numpy.unique(self.counts):
            rows = numpy.nonzero(self.counts == size)[0]
            gather = (self._starts[rows][:, None]
                      + numpy.arange(size)[None, :])
            out[rows] = self.points[self.vertex_indices[gather]].mean(axis=1)
        self._centroids = out
        return out

    @property
    def brush(self):
        """The paint radius in stage units, scaled to the mesh."""
        return max(self._diagonal * BRUSH_FRACTION, 1e-4)

    def Brush(self, point, direction=None, radius=None):
        """Faces within the brush of a world point, as an index array.

        A world-space ball rather than a screen-space disc, and that is a
        cost decision: a screen disc would mean one ray cast per pixel of
        the brush, at 2.6 ms each, where this is one vectorised distance
        test over 26k centroids. `direction` is the view ray; faces whose
        normal points away from it are dropped, so a stroke on the front
        of a thigh does not silently paint the back of it as well.
        """
        centroids = self.Centroids()
        radius = self.brush if radius is None else float(radius)
        point = numpy.asarray(point, dtype=numpy.float32)
        delta = centroids - point
        inside = numpy.einsum("ij,ij->i", delta, delta) <= radius * radius
        faces = numpy.nonzero(inside)[0].astype(numpy.int32)
        if direction is None or not len(faces):
            return faces
        # Face normal from the first corner triangle, same approximation
        # the overlay patch uses.
        starts = self._starts[faces]
        v = self.vertex_indices
        a = self.points[v[starts]]
        b = self.points[v[starts + 1]]
        c = self.points[v[starts + 2]]
        normals = numpy.cross(b - a, c - a)
        direction = numpy.asarray(direction, dtype=numpy.float32)
        return faces[numpy.einsum("ij,j->i", normals, direction) < 0.0]

    def AssignFaces(self, region, faces):
        """Give `faces` to `region`, taking them off whoever had them.

        The regions are non-overlapping by construction -- that is what
        makes `region_of` a flat array index instead of a search -- so a
        face joining one region must LEAVE the other in the same step.
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
        return touched

    def Rows(self):
        """The regions as plain tuples, for the writer.

        Empty regions are dropped: a region painted down to nothing is
        the animator saying they do not want it, and writing an empty one
        back is the "author things for no reason" the skip rule exists to
        avoid.
        """
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
    in the middle here. Kept out of the UI file because it is arithmetic
    with a right answer, and `test_touchpose_model` checks it against a
    frustum built by hand.
    """
    if width <= 0 or height <= 0:
        return None
    nx = (2.0 * (x + 0.5) / float(width)) - 1.0
    ny = 1.0 - (2.0 * (y + 0.5) / float(height))
    # `ComputePickRay` is the frustum's own pixel->ray, in the same
    # normalised window space `StageView.computePickFrustum` uses, so
    # a TouchPose pick and a usdview pick aim at the same place.
    ray = frustum.ComputePickRay(Gf.Vec2d(nx, ny))
    start = ray.startPoint
    direction = ray.direction
    return ((float(start[0]), float(start[1]), float(start[2])),
            (float(direction[0]), float(direction[1]), float(direction[2])))


def _ScaleTo(color, value):
    """Scale a colour so its brightest channel is `value`.

    A pure scale, so the hue and the saturation are exactly the authored
    ones and only the brightness changes. A black input has nothing to
    scale and comes back as a neutral at `value`, which is at least
    visible.
    """
    peak = max(color)
    if peak <= 1e-6:
        return (value, value, value)
    factor = value / peak
    return tuple(min(1.0, c * factor) for c in color)

