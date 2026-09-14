#!/usr/bin/env python
"""
Headless test for plugin/touchPose/touchPoseModel.py and the importer's
skip rule: reading touch regions off a stage, the exact ray cast, the
face -> region lookup, the pick following DEFORMED points rather than the
rest mesh, the overlay patch the highlight is made of, and the pixel ->
ray arithmetic the viewport hands the cast.

Everything runs on an in-memory six-face cube -- no Qt, no usdview, no
GL. That is the point of the model/UI split: the only things the app has
to be running for are the mouse and the pixels, and those are asserted in
tests/testUsdviewTouchPose.py instead.

Usage: test_touchpose_model.py [schema resource dir]
"""
import os
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

# rigexec_test_env puts plugin/rigExecUsdview on the path; touchPose is
# its own directory and is added here rather than by editing the shared
# bootstrap, which every other plugin test depends on.
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
for _extra in (os.path.join(_ROOT, "plugin", "touchPose"),):
    if _extra not in sys.path:
        sys.path.insert(0, _extra)

import numpy  # noqa: E402
from pxr import Tf, Gf, Sdf, Usd, UsdGeom  # noqa: E402

import touchPoseModel  # noqa: E402
from touchpose import usdexport  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


# A unit cube centred on the origin, one quad per side. Face 0 is +Z
# (front), face 1 is -Z (back) -- the pair that makes "did the cast pick
# the NEAR surface" a real question rather than a formality.
_CUBE_POINTS = [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
                (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1)]
_CUBE_FACES = [
    [0, 1, 2, 3],      # +Z
    [5, 4, 7, 6],      # -Z
    [1, 5, 6, 2],      # +X
    [4, 0, 3, 7],      # -X
    [3, 2, 6, 7],      # +Y
    [4, 5, 1, 0],      # -Y
]


def _Stage():
    """A cube with two touch regions, one of which binds nothing.

    The unbound one is the case the importer now skips at author time;
    it is here so `require_control` is asserted rather than assumed,
    because an already-imported `Biped_touch.usda` from before the rule
    changed still contains them.
    """
    layer = Sdf.Layer.CreateAnonymous("cube.usda")
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)

    mesh = UsdGeom.Mesh.Define(stage, "/Body")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in _CUBE_POINTS])
    mesh.CreateFaceVertexCountsAttr([4] * len(_CUBE_FACES))
    mesh.CreateFaceVertexIndicesAttr(
        [i for face in _CUBE_FACES for i in face])

    control = UsdGeom.Xform.Define(stage, "/Rig/front_ctl")

    front = UsdGeom.Subset.Define(stage, "/Body/front_touch")
    front.CreateElementTypeAttr(UsdGeom.Tokens.face)
    front.CreateFamilyNameAttr(touchPoseModel.FAMILY)
    front.CreateIndicesAttr([0])
    front.GetPrim().CreateRelationship(
        touchPoseModel.CONTROL_REL, custom=True).SetTargets(
            [control.GetPath()])
    front.GetPrim().CreateAttribute(
        touchPoseModel.COLOR_ATTR, Sdf.ValueTypeNames.Color3f,
        custom=True).Set(Gf.Vec3f(0.1, 0.4, 0.9))

    sides = UsdGeom.Subset.Define(stage, "/Body/sides_touch")
    sides.CreateElementTypeAttr(UsdGeom.Tokens.face)
    sides.CreateFamilyNameAttr(touchPoseModel.FAMILY)
    sides.CreateIndicesAttr([2, 3])
    sides.GetPrim().CreateRelationship(
        touchPoseModel.CONTROL_REL, custom=True).SetTargets(
            [control.GetPath()])

    # The state colours live where the palette does. Deliberately dark,
    # as the authored ones are, so the lift in `StateColors` is asserted
    # rather than assumed.
    mesh.GetPrim().CreateAttribute(
        touchPoseModel.LEAD_COLOR_ATTR, Sdf.ValueTypeNames.Color3f,
        custom=True).Set(Gf.Vec3f(0.05, 0.42, 0.19))
    mesh.GetPrim().CreateAttribute(
        touchPoseModel.SELECTED_COLOR_ATTR, Sdf.ValueTypeNames.Color3f,
        custom=True).Set(Gf.Vec3f(0.28, 0.28, 0.28))

    orphan = UsdGeom.Subset.Define(stage, "/Body/orphan_touch")
    orphan.CreateElementTypeAttr(UsdGeom.Tokens.face)
    orphan.CreateFamilyNameAttr(touchPoseModel.FAMILY)
    orphan.CreateIndicesAttr([4])
    return stage


def TestReading():
    stage = _Stage()

    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")
    names = sorted(r.name for r in model.regions)
    _Check(names == ["front_touch", "sides_touch"],
           "the unbound subset is dropped: %s" % names)
    _Check(model.face_count == 6, "six faces, got %d" % model.face_count)

    regions, covered, faces = model.Coverage()
    _Check((regions, covered, faces) == (2, 3, 6),
           "coverage is 2 regions over 3 of 6 faces, got %s"
           % ((regions, covered, faces),))

    front = model.RegionOfFace(0)
    _Check(front is not None and front.name == "front_touch",
           "face 0 is the front region, got %s" % front)
    _Check(front.label == "front", "the label drops _touch: %r" % front.label)
    _Check(model.RegionOfFace(1) is None,
           "face 1 belongs to no region (the orphan was dropped)")
    _Check(model.RegionOfFace(4) is None,
           "the orphan's face is unowned, not owned by nothing-in-particular")
    _Check(model.RegionOfFace(-1) is None and model.RegionOfFace(99) is None,
           "an out-of-range face answers None rather than raising")

    keep = touchPoseModel.TouchModel.FromStage(stage, "/Body",
                                               require_control=False)
    _Check(len(keep.regions) == 3,
           "require_control=False keeps the orphan: %d" % len(keep.regions))
    print("  reading: 2 of 3 subsets bind a control and are kept, "
          "3 of 6 faces covered")


def TestCast():
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")

    # Straight at the front face from +Z. The NEAR surface must win: the
    # same ray also passes through face 1 at t = 12.
    face, t = model.Cast((0.0, 0.0, 10.0), (0.0, 0.0, -1.0))
    _Check(face == 0, "the ray hits the front face, got %d" % face)
    _Check(abs(t - 9.0) < 1e-3, "at t = 9, got %.4f" % t)
    _Check(model.RegionAt((0.0, 0.0, 10.0), (0.0, 0.0, -1.0)).name
           == "front_touch", "and resolves to the front region")

    # ...and from behind it must be face 1, which is the half of the
    # problem nearest-centroid gets wrong on a real body.
    face, _t = model.Cast((0.0, 0.0, -10.0), (0.0, 0.0, 1.0))
    _Check(face == 1, "from behind the cast hits the back face, got %d" % face)
    _Check(model.RegionOfFace(face) is None,
           "the back face is in no region, so the pick is a miss")

    # A ray that misses entirely.
    face, _t = model.Cast((5.0, 5.0, 10.0), (0.0, 0.0, -1.0))
    _Check(face == -1, "a miss answers -1, got %d" % face)
    _Check(model.RegionAt((5.0, 5.0, 10.0), (0.0, 0.0, -1.0)) is None,
           "and no region")

    # The +X side, which belongs to the two-face region.
    region = model.RegionAt((10.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    _Check(region is not None and region.name == "sides_touch",
           "the +X face belongs to sides_touch, got %s" % region)

    # A degenerate direction must not raise or answer nonsense.
    _Check(model.Cast((0, 0, 10), (0, 0, 0)) == (-1, -1.0),
           "a zero-length direction answers a clean miss")
    print("  cast: near face wins over far, miss answers -1, "
          "region resolved on all six sides")


def TestFollowsTheDeformedMesh():
    """The pick must follow the POSED points, not the authored ones.

    This is the whole reason the points come out of Hydra instead of off
    the stage. Simulated here by translating the cube by hand: a ray that
    hit the rest cube must miss once it moves away, and a ray aimed where
    it moved TO must hit.
    """
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")

    aimed_at_rest = ((0.0, 0.0, 10.0), (0.0, 0.0, -1.0))
    _Check(model.Cast(*aimed_at_rest)[0] == 0, "hits at rest")

    posed = numpy.asarray(_CUBE_POINTS, dtype=numpy.float32) \
        + numpy.asarray([0.0, 12.0, 0.0], dtype=numpy.float32)
    model.SetPoints(posed)

    _Check(model.Cast(*aimed_at_rest)[0] == -1,
           "the same ray now MISSES -- it was aimed at the rest mesh")
    face, _t = model.Cast((0.0, 12.0, 10.0), (0.0, 0.0, -1.0))
    _Check(face == 0,
           "and a ray aimed where the cube moved to hits face 0, got %d"
           % face)
    _Check(model.RegionOfFace(face).name == "front_touch",
           "resolving to the same region it did at rest")

    # The offset the highlight uses scales with the geometry, so it must
    # be recomputed from the new points rather than frozen at load.
    _Check(model.min_offset > 0.0, "the overlay offset floor is positive")
    _Check(model.OffsetFor(model.RegionOfFace(0)) > 0.0,
           "and a region's own lift is positive")
    print("  deformation: a rest-aimed ray misses a posed mesh and a "
          "pose-aimed ray hits the same region")


def TestOverlayGeometry():
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")
    region = model.RegionOfFace(0)

    points, counts, indices = model.OverlayGeometry(region)
    counts, indices = counts.tolist(), indices.tolist()
    _Check(counts == [4], "one quad, got %s" % counts)
    _Check(len(points) == 4,
           "compacted to the 4 corners it uses, not the cube's 8: %d"
           % len(points))
    _Check(sorted(indices) == [0, 1, 2, 3],
           "indices renumbered into the compact points: %s" % indices)

    # The patch must sit OFF the surface by the offset, along +Z here,
    # or it z-fights with the body it is lying on.
    lift = model.OffsetFor(region)
    _Check(abs(float(points[:, 2].min()) - (1.0 + lift)) < 1e-4,
           "lifted by the offset (%.5f) along the face normal: z = %.5f"
           % (lift, float(points[:, 2].min())))
    _Check(abs(float(points[:, 2].max()) - (1.0 + lift)) < 1e-4,
           "all four corners, evenly")

    # The two-face region: shared corners must be shared in the output.
    sides = model.RegionOfFace(2)
    points, counts, indices = model.OverlayGeometry(sides)
    counts, indices = counts.tolist(), indices.tolist()
    _Check(counts == [4, 4], "two quads, got %s" % counts)
    _Check(len(points) == 8,
           "+X and -X share no corner on a cube, so 8 points: %d"
           % len(points))
    _Check(len(indices) == 8, "eight corners referenced: %d" % len(indices))

    # ...and a region sharing an edge really does weld.
    welded = touchPoseModel.Region(0, "welded_touch", "/Rig/front_ctl",
                                   (1, 1, 1), 0,
                                   numpy.asarray([0, 2], numpy.int32))
    points, counts, _i = model.OverlayGeometry(welded)
    counts = counts.tolist()
    _Check(counts == [4, 4] and len(points) == 6,
           "+Z and +X share an edge, so 6 points not 8: %d" % len(points))

    empty = touchPoseModel.Region(0, "empty_touch", None, (1, 1, 1), 0,
                                  numpy.asarray([], numpy.int32))
    points, counts, indices = model.OverlayGeometry(empty)
    _Check(len(points) == 0 and len(counts) == 0 and len(indices) == 0,
           "an empty region makes an empty patch rather than raising")
    print("  overlay: compact points, welded shared edges, lifted %.5f "
          "off the skin" % lift)


def TestStateColors():
    """The three states have to be three DIFFERENT colours.

    Hover is per-region; lead and selected come from the `.touch` file.
    What is asserted here is that the lift keeps the hue and raises the
    value -- a lead that stopped being green, or a selected that stopped
    being neutral, would no longer agree with the the conventional tool tool the data
    came from.
    """
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")

    _Check(abs(model.lead_color[1] - 0.42) < 1e-3,
           "the authored lead colour was read: %s" % (model.lead_color,))
    _Check(abs(model.selected_color[0] - 0.28) < 1e-3,
           "and the selected one: %s" % (model.selected_color,))

    colors = model.StateColors()
    lead, selected = colors["lead"], colors["selected"]
    _Check(lead[1] > model.lead_color[1],
           "lead is lifted: %s from %s" % (lead, model.lead_color))
    # A pure scale, so the ratios between the channels are untouched --
    # that is what keeps the two states apart after the lift.
    _Check(abs(lead[0] / lead[1] - model.lead_color[0] / model.lead_color[1])
           < 1e-5,
           "...by a SCALE, so the hue is unchanged: %s from %s"
           % (lead, model.lead_color))
    _Check(lead[1] > selected[1],
           "and lead is the brighter of the two: %.3f vs %.3f"
           % (lead[1], selected[1]))
    _Check(lead[1] > lead[0] and lead[1] > lead[2],
           "...and is still GREEN, as the conventional kLeadSelected is: %s" % (lead,))
    _Check(max(selected) - min(selected) < 1e-6,
           "selected is still NEUTRAL: %s" % (selected,))
    _Check(min(selected) > model.selected_color[0],
           "...and lifted: %s from %s" % (selected, model.selected_color))

    hover = model.HoverColor(model.RegionOfFace(0))
    for a, b, what in ((lead, selected, "lead vs selected"),
                       (lead, hover, "lead vs hover"),
                       (selected, hover, "selected vs hover")):
        distance = sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5
        _Check(distance > 0.2,
               "%s are distinguishable: %.3f apart (%s / %s)"
               % (what, distance, a, b))

    # Every region's hover colour must clear BOTH state colours, not
    # just the one region the fixture happens to look at.
    for candidate in model.regions:
        got = model.HoverColor(candidate)
        for state, value in colors.items():
            distance = sum((x - y) ** 2 for x, y in zip(got, value)) ** 0.5
            _Check(distance > 0.2,
                   "%s's hover %s clears %s %s: %.3f"
                   % (candidate.label, tuple(round(c, 2) for c in got),
                      state, tuple(round(c, 2) for c in value), distance))

    # With no colours authored at all the model still answers, with the
    # studio's values as the fallback -- an older touch layer must not
    # leave the selection highlight black.
    bare = touchPoseModel.TouchModel([4], [0, 1, 2, 3], [])
    _Check(bare.StateColors()["lead"][1] > 0.4,
           "an unauthored stage still gets a visible lead colour: %s"
           % (bare.StateColors(),))
    print("  state colours: lead %s (green), selected %s (neutral), both "
          "lifted and all three states apart" % (
              tuple(round(c, 3) for c in lead),
              tuple(round(c, 3) for c in selected)))


def TestMergedOverlay():
    """Several regions draw as ONE patch, not one prim each."""
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")
    front = model.RegionOfFace(0)
    sides = model.RegionOfFace(2)

    faces = model.FacesOf([front, sides])
    _Check(sorted(faces.tolist()) == [0, 2, 3],
           "the union of the two regions' faces: %s" % faces.tolist())

    points, counts, indices = model.OverlayGeometry(faces)
    _Check(counts.tolist() == [4, 4, 4],
           "three quads in one patch, got %s" % counts.tolist())

    # The same region twice must not double up -- two selected regions
    # can name the same control.
    _Check(model.FacesOf([front, front]).tolist() == [0],
           "a repeated region contributes its faces once")
    _Check(len(model.FacesOf([])) == 0, "no regions, no faces")

    # And the path the selection layer actually takes: control -> regions.
    control = front.control
    got = model.RegionsFor([control])
    _Check(sorted(r.name for r in got) == ["front_touch", "sides_touch"],
           "both regions share the one control in this fixture: %s"
           % [r.name for r in got])
    _Check(model.RegionsFor(["/nothing/here"]) == [],
           "an unselected path lights nothing")
    print("  merged overlay: 2 regions -> 3 quads in one patch, "
          "duplicates collapsed")


def TestRayThroughPixel():
    """Pixel -> ray, including the y flip that is easy to get backwards."""
    frustum = Gf.Frustum()
    frustum.SetPositionAndRotationFromMatrix(Gf.Matrix4d(1.0))
    frustum.SetProjectionType(Gf.Frustum.Perspective)
    frustum.SetPerspective(60.0, 1.0, 1.0, 100.0)
    # A camera at the origin looking down -Z, which is USD's convention.

    origin, direction = touchPoseModel.RayThroughPixel(frustum, 99.5, 99.5,
                                                       200, 200)
    # `ComputePickRay` starts the ray on the NEAR plane, not at the eye.
    # That is what a pick wants -- geometry between the eye and the near
    # plane is not on screen and must not be picked -- and it is the
    # difference that would otherwise show up as a t offset of `near`.
    _Check(abs(origin[0]) < 1e-6 and abs(origin[1]) < 1e-6
           and abs(origin[2] + 1.0) < 1e-6,
           "the ray starts on the near plane, down -Z: %s" % (origin,))
    _Check(abs(direction[0]) < 1e-3 and abs(direction[1]) < 1e-3
           and direction[2] < 0,
           "the centre pixel looks straight down -Z: %s" % (direction,))

    # Top of the frame is +Y in the world; Qt's y grows DOWNWARD, so a
    # small pixel y must give a positive world y. Getting this backwards
    # is invisible on a symmetric character and wrong everywhere else.
    _origin, up = touchPoseModel.RayThroughPixel(frustum, 100, 10, 200, 200)
    _Check(up[1] > 0.0,
           "pixel y=10 (near the TOP) points up in world space: %s" % (up,))
    _origin, down = touchPoseModel.RayThroughPixel(frustum, 100, 190, 200, 200)
    _Check(down[1] < 0.0, "and y=190 points down: %s" % (down,))
    _origin, right = touchPoseModel.RayThroughPixel(frustum, 190, 100, 200, 200)
    _Check(right[0] > 0.0, "pixel x=190 points +X: %s" % (right,))

    _Check(touchPoseModel.RayThroughPixel(frustum, 0, 0, 0, 0) is None,
           "a zero-sized viewport answers None rather than dividing by it")
    print("  pixel -> ray: centre is on axis, y flips, x does not")


def TestMarqueeMath():
    """Screen rect -> regions, with no viewport involved.

    The band is the one part of the selection path that is pure
    arithmetic, so it is checked here rather than only in the app: a
    camera looking down -Z at the cube, a rect over one half of the
    frame, and the regions that should fall in it.
    """
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")

    frustum = Gf.Frustum()
    frustum.SetPositionAndRotationFromMatrix(
        Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 0, 10)))
    frustum.SetProjectionType(Gf.Frustum.Perspective)
    frustum.SetPerspective(60.0, 1.0, 1.0, 100.0)
    matrix = frustum.ComputeViewMatrix() * frustum.ComputeProjectionMatrix()
    rows = [[matrix[r][c] for c in range(4)] for r in range(4)]

    pixels, valid = model.FacePixels(rows, 200, 200)
    _Check(len(pixels) == 6 and valid.all(),
           "six faces projected, all in front of the eye: %s" % valid)
    # The +Z face is dead centre; the +Y face is above centre, which in
    # Qt's downward y means a SMALLER pixel y.
    _Check(abs(pixels[0][0] - 99.5) < 1.0 and abs(pixels[0][1] - 99.5) < 1.0,
           "the front face lands in the middle: %s" % (pixels[0],))
    _Check(pixels[4][1] < pixels[0][1],
           "the +Y face is higher up the screen: %.1f vs %.1f"
           % (pixels[4][1], pixels[0][1]))

    front = model.FrontFacing((0.0, 0.0, 10.0))
    _Check(front[0] and not front[1],
           "the +Z face faces the eye and the -Z one does not: %s" % front)

    caught = model.RegionsInRect(pixels, front, 0, 0, 200, 200)
    names = sorted(r.name for r in caught)
    _Check("front_touch" in names,
           "a band over the whole frame catches the front region: %s"
           % names)
    _Check(len(model.RegionsInRect(pixels, front, 0, 0, 5, 5)) == 0,
           "a band in the corner catches nothing")

    # Reversed drags are the same rect -- an animator drags both ways.
    _Check([r.name for r in model.RegionsInRect(pixels, front,
                                                200, 200, 0, 0)] ==
           [r.name for r in caught],
           "a band dragged up-left is the same band")

    # The back face is excluded by the mask, not by the rect: without the
    # facing test a band over the front of a body also catches the back.
    everything = numpy.ones(model.face_count, dtype=bool)
    _Check(len(model.RegionsInRect(pixels, everything, 0, 0, 200, 200))
           >= len(caught),
           "with no facing mask the same band catches at least as much")
    print("  marquee: projection, facing mask and rect test agree on a "
          "cube from a known camera")


def TestPainting():
    """Faces move between regions without ever overlapping.

    Non-overlap is not cosmetic: `region_of` is a flat array index
    precisely because no face belongs to two regions, so a face joining
    one region has to LEAVE the other in the same step or the lookup
    starts lying.
    """
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")
    front = model.RegionOfFace(0)
    sides = model.RegionOfFace(2)
    _Check(len(front.faces) == 1 and len(sides.faces) == 2,
           "starting sizes: %d and %d" % (len(front.faces), len(sides.faces)))
    _Check(not model.dirty, "a freshly read model is clean")

    touched = model.AssignFaces(front, [2])
    _Check(sorted(len(r.faces) for r in touched) == [1, 2],
           "both regions changed: %s"
           % [(r.name, len(r.faces)) for r in touched])
    _Check(sorted(front.faces.tolist()) == [0, 2],
           "the face joined the front region: %s" % front.faces.tolist())
    _Check(sorted(sides.faces.tolist()) == [3],
           "...and LEFT the other one: %s" % sides.faces.tolist())
    _Check(model.RegionOfFace(2).index == front.index,
           "and the lookup agrees")
    _Check(model.dirty, "the model knows it has unsaved edits")

    # An unowned face can be painted too, and painting a face a region
    # already owns is a no-op it must survive.
    model.AssignFaces(front, [1, 0])
    _Check(sorted(front.faces.tolist()) == [0, 1, 2],
           "an unowned face joins: %s" % front.faces.tolist())

    erased = model.EraseFaces([1, 2])
    _Check([r.name for r in erased] == ["front_touch"],
           "erase reports who lost faces: %s" % [r.name for r in erased])
    _Check(front.faces.tolist() == [0],
           "and takes them off: %s" % front.faces.tolist())
    _Check(model.RegionOfFace(1) is None and model.RegionOfFace(2) is None,
           "the lookup agrees after an erase")
    _Check(model.EraseFaces([5]) == [],
           "erasing an unowned face changes nothing")

    # Out-of-range indices must be dropped, not crash or corrupt.
    _Check(model.AssignFaces(front, [99, -3]) == [],
           "indices past the end of the mesh are ignored")

    # The brush is a ball around a point, and it must not reach through
    # the cube to the far side.
    faces = model.Brush((0.0, 0.0, 1.0), (0.0, 0.0, -1.0), radius=0.5)
    _Check(faces.tolist() == [0],
           "a brush on the front face touches only it: %s" % faces.tolist())
    wide = model.Brush((0.0, 0.0, 0.0), None, radius=5.0)
    _Check(len(wide) == 6, "a brush big enough reaches every face: %d"
           % len(wide))
    facing = model.Brush((0.0, 0.0, 0.0), (0.0, 0.0, -1.0), radius=5.0)
    _Check(len(facing) < len(wide),
           "...but with a view direction the back faces drop out: %d of %d"
           % (len(facing), len(wide)))
    _Check(0 in facing.tolist() and 1 not in facing.tolist(),
           "the front face is in and the back one is not: %s"
           % facing.tolist())
    print("  painting: faces move between regions with no overlap, erase "
          "and out-of-range are safe, the brush respects facing")


def TestSaveRoundTrip():
    """What is painted is what is saved is what is read back."""
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")
    front = model.RegionOfFace(0)
    model.AssignFaces(front, [1, 2, 3])

    out = os.path.join(os.environ.get("TEMP", "."),
                       "touchpose_paint_test.usda")
    if os.path.exists(out):
        os.remove(out)
    written = usdexport.save_regions(
        out, "/Body", model.Rows(), palette=model.palette,
        alpha=model.alpha, lead=model.lead_color,
        selected=model.selected_color)
    _Check(written == 1,
           "the emptied region was dropped, so one row went out: %d"
           % written)

    # Read it back on a stage that has the mesh under it, which is how it
    # is used -- the layer alone is `over`s and composes to nothing.
    layer = Sdf.Layer.CreateAnonymous("combined.usda")
    layer.subLayerPaths.append(out)
    combined = Usd.Stage.Open(layer)
    UsdGeom.Mesh.Define(combined, "/Body").GetPrim()
    mesh = UsdGeom.Mesh(combined.GetPrimAtPath("/Body"))
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in _CUBE_POINTS])
    mesh.CreateFaceVertexCountsAttr([4] * len(_CUBE_FACES))
    mesh.CreateFaceVertexIndicesAttr(
        [i for face in _CUBE_FACES for i in face])
    UsdGeom.Xform.Define(combined, "/Rig/front_ctl")

    back = touchPoseModel.TouchModel.FromStage(combined, "/Body")
    _Check(len(back.regions) == 1,
           "one region read back, got %d" % len(back.regions))
    _Check(sorted(back.regions[0].faces.tolist()) == [0, 1, 2, 3],
           "with the painted faces: %s" % back.regions[0].faces.tolist())
    _Check(back.regions[0].control == front.control,
           "and still bound to %s, got %s"
           % (front.control, back.regions[0].control))
    _Check(abs(back.lead_color[1] - model.lead_color[1]) < 1e-5,
           "the state colours survived the save: %s vs %s"
           % (back.lead_color, model.lead_color))

    # Non-overlap survives too, which is what the pick loop depends on.
    counts = {}
    for region in back.regions:
        for face in region.faces.tolist():
            counts[face] = counts.get(face, 0) + 1
    _Check(max(counts.values()) == 1,
           "no face is in two regions after a round trip: %s" % counts)

    os.remove(out)
    print("  save: 1 region / 4 painted faces round-tripped, empty region "
          "dropped, colours and non-overlap intact")


def TestImporterSkipsUnboundSets():
    """The rule the user asked for, asserted on numbers not on intent.

    A set naming a control the rig does not have is not authored at all,
    so nothing downstream pays to test, highlight and then refuse it --
    but the count stays in the report so the naming gap is still visible.
    """
    stage = _Stage()

    class _Set(object):
        def __init__(self, name, faces):
            self.name = name
            self.control = name[:-6]
            self.hilight = 0
            self.color = (0.5, 0.5, 0.5)
            self._faces = faces

        @property
        def face_count(self):
            return len(self._faces)

        @property
        def meshes(self):
            return ["Body"]

        def faces_on(self, mesh):
            return list(self._faces) if mesh == "Body" else []

    class _Palette(object):
        colors = [(1.0, 0.0, 0.0)]
        alpha = 0.5
        lead = (0.05, 0.42, 0.19)
        selected = (0.28, 0.28, 0.28)

    class _Doc(object):
        palette = _Palette()
        sets = [_Set("kept_touch", [0, 1]), _Set("gone_touch", [2]),
                _Set("also_kept_touch", [3])]

    known = {"kept": "/Rig/front_ctl", "also_kept": "/Rig/front_ctl"}
    out = os.path.join(os.environ.get("TEMP", "."),
                       "touchpose_skip_test.usda")
    if os.path.exists(out):
        os.remove(out)
    report = usdexport.author(_Doc(), stage, out, mesh_path="/Body",
                              resolve=lambda name: known.get(name),
                              sublayer=False)
    _Check(len(report.written) == 2,
           "two of three sets authored, got %d" % len(report.written))
    _Check(report.skipped_unbound == 1,
           "one skipped, counted: %d" % report.skipped_unbound)
    _Check([n for n, _c in report.unresolved] == ["gone_touch"],
           "and named in the report: %s" % report.unresolved)

    written = Usd.Stage.Open(out)
    mesh = written.GetPrimAtPath("/Body")
    _Check(mesh.GetSpecifier() == Sdf.SpecifierOver,
           "the layer OVERS the mesh rather than defining it -- stacked "
           "over the wrong asset it must fail to find the mesh, not "
           "conjure one; got %s" % mesh.GetSpecifier())
    # The regions live on their own scope, NOT under the mesh: a face
    # GeomSubset is collected by hdSt whatever family it declares, and
    # 98 of them collided with `body_geo`'s five materialBind subsets for
    # 16,739 warnings on open, over data the renderer never draws.
    # The path is DERIVED from the mesh, not a constant: it used to be
    # the literal "/Biped/TouchPose", so TouchPose worked for exactly one
    # character. Here the mesh is /Body, which has no asset scope above
    # it, so the regions land beside it at /TouchPose.
    expected = usdexport.scope_path_for("/Body")
    scope = written.GetPrimAtPath(expected)
    _Check(scope and scope.IsValid(),
           "the regions scope was authored at %s" % expected)
    _Check(scope.GetTypeName() == "RigExecTouchRegions"
           or not Tf.Type.FindByName("RigExecTouchRegions"),
           "and it is typed, got %s" % scope.GetTypeName())
    children = sorted(p.GetName() for p in scope.GetAllChildren())
    # The scope also carries the shipped highlight overlay, which is the
    # point of shipping it: a shot creates no prim for it.
    _Check("Overlay" in children,
           "the scope ships its own highlight overlay: %s" % children)
    overlay = scope.GetStage().GetPrimAtPath(
        scope.GetPath().AppendChild("Overlay"))
    _Check(str(overlay.GetTypeName()) == "RigExecTouchOverlay"
           or not Tf.Type.FindByName("RigExecTouchOverlay"),
           "and it is typed, got %s" % overlay.GetTypeName())
    subsets = [c for c in children if c != "Overlay"]
    _Check(subsets == ["also_kept_touch", "kept_touch"],
           "the skipped set left NOTHING behind, not an empty prim: %s"
           % subsets)
    _Check(not list(mesh.GetAllChildren()),
           "and nothing at all was hung off the mesh: %s"
           % [p.GetName() for p in mesh.GetAllChildren()])
    _Check(scope.GetAttribute(usdexport.LEAD_COLOR_ATTR).Get() is not None
           and scope.GetAttribute(
               usdexport.SELECTED_COLOR_ATTR).Get() is not None,
           "the state colours went out with the palette")

    # ...and with no resolver at all nothing was looked for, so nothing
    # is missing and every set is authored.
    out2 = out.replace(".usda", "_all.usda")
    if os.path.exists(out2):
        os.remove(out2)
    report = usdexport.author(_Doc(), stage, out2, mesh_path="/Body",
                              resolve=None, sublayer=False)
    _Check(len(report.written) == 3 and report.skipped_unbound == 0,
           "with no resolver all three are authored: %d written, %d skipped"
           % (len(report.written), report.skipped_unbound))
    for path in (out, out2):
        if os.path.exists(path):
            os.remove(path)
    print("  importer: 2 of 3 authored, 1 skipped and counted, "
          "no empty prims left behind")


def TestColorSets():
    """The `.touch` file carries TWO colour sets and the mode picks one.

    `touchpose:color` is per region and is what edit mode draws every
    region in at once; `touchpose:palette[touchpose:hilight]` is the six
    -colour highlight ramp the the conventional tool shape draws the ONE hovered region
    in. The port only ever used the first, so this asserts both arrive
    and that they are genuinely different answers.
    """
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")

    # No palette authored: the highlight set has to fall back to the
    # region's own colour rather than to black, or an older touch layer
    # stops lighting up at all.
    region = model.RegionOfFace(0)
    _Check(model.HilightColor(region) == tuple(region.color),
           "with no palette the highlight colour is the region's own: %s"
           % (model.HilightColor(region),))

    model.palette = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    region.hilight = 1
    _Check(model.HilightColor(region) == (0.0, 1.0, 0.0),
           "hilight 1 indexes palette[1]: %s"
           % (model.HilightColor(region),))
    region.hilight = 7
    _Check(model.HilightColor(region) == (0.0, 1.0, 0.0),
           "...and an out-of-range index wraps rather than throwing: %s"
           % (model.HilightColor(region),))
    region.hilight = 1

    edit = model.EditColor(region)
    hover = model.HoverColor(region)
    _Check(edit != hover,
           "the two sets now give different answers: edit %s, hover %s"
           % (edit, hover))
    _Check(abs(hover[1] - 1.0) < 1e-6 and hover[0] == 0.0,
           "the hover colour is the palette entry, lifted: %s" % (hover,))

    # Every painted face, and a colour per face for the edit-mode patch.
    faces = model.AllFaces()
    _Check(sorted(faces.tolist()) == [0, 2, 3],
           "AllFaces is every painted face, once: %s" % faces.tolist())
    colors = model.FaceColors(faces, editing=True)
    _Check(colors.shape == (3, 3),
           "one colour per face: %s" % (colors.shape,))
    front = model.RegionOfFace(0)
    sides = model.RegionOfFace(2)

    def _Same(a, b):
        # float32 out of the array against Python floats out of the
        # model, so this is a tolerance and not an equality.
        return max(abs(float(x) - float(y)) for x, y in zip(a, b)) < 1e-6

    _Check(_Same(colors[0], model.EditColor(front)),
           "face 0 got its own region's edit colour: %s" % (colors[0],))
    _Check(_Same(colors[1], model.EditColor(sides))
           and _Same(colors[2], model.EditColor(sides)),
           "and both of the other region's faces got its: %s" % (colors[1:],))
    _Check(not _Same(colors[0], colors[1]),
           "the two regions are drawn apart, which is the whole point of "
           "the edit set: %s vs %s" % (colors[0], colors[1]))
    print("  colour sets: palette drives hover, per-region colours drive "
          "the edit patch, %d faces coloured in one array" % len(colors))


def TestOffsetScalesWithTheRegion():
    """The lift is the REGION's size, not the body's, with a floor.

    Spike R6 measured the z-fight gone by 0.005 cm on a 224 cm biped and
    flat from there to the 0.34 cm that shipped, so the lift can follow
    the region instead of the body. What is asserted here is that it
    DOES follow it -- a big patch lifts further than a small one -- and
    that the floor catches a patch small enough to fall through.
    """
    stage = _Stage()
    model = touchPoseModel.TouchModel.FromStage(stage, "/Body")

    big = model.RegionOfFace(2)         # two faces, opposite sides
    small = model.RegionOfFace(0)       # one face
    _Check(model.OffsetFor(big) > model.OffsetFor(small),
           "a bigger region lifts further: %.5f vs %.5f"
           % (model.OffsetFor(big), model.OffsetFor(small)))
    _Check(model.OffsetFor(small) >= model.min_offset,
           "and nothing lifts less than the floor: %.5f vs %.5f"
           % (model.OffsetFor(small), model.min_offset))

    # The floor has to bite on a region small enough to need it. A single
    # face of the cube spans 2.0, so shrink the constant instead of
    # building a second fixture.
    saved = touchPoseModel.OFFSET_FRACTION
    try:
        touchPoseModel.OFFSET_FRACTION = 1e-9
        _Check(abs(model.OffsetFor(small) - model.min_offset) < 1e-12,
               "with a vanishing fraction the floor is what is used: "
               "%.8f" % model.OffsetFor(small))
    finally:
        touchPoseModel.OFFSET_FRACTION = saved

    # And the patch is actually built at that lift, not at some other
    # number computed elsewhere.
    points, _counts, _indices = model.OverlayGeometry(small)
    _Check(abs(float(points[:, 2].min()) - (1.0 + model.OffsetFor(small)))
           < 1e-4,
           "OverlayGeometry lifts by exactly OffsetFor: z = %.5f, "
           "expected %.5f" % (float(points[:, 2].min()),
                              1.0 + model.OffsetFor(small)))
    print("  offset: %.5f for a 1-face region, %.5f for a 2-face one, "
          "floor %.5f" % (model.OffsetFor(small), model.OffsetFor(big),
                          model.min_offset))


def main():
    print("touchPoseModel:")
    TestReading()
    TestCast()
    TestFollowsTheDeformedMesh()
    TestOverlayGeometry()
    TestOffsetScalesWithTheRegion()
    TestStateColors()
    TestColorSets()
    TestMergedOverlay()
    TestRayThroughPixel()
    TestMarqueeMath()
    TestPainting()
    TestSaveRoundTrip()
    TestImporterSkipsUnboundSets()
    print("TOUCHPOSE_MODEL_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
