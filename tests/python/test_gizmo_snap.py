#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoSnap.py using the
synthetic Gf.Camera test_gizmo_screen.py uses -- at (0, 0, 10)
looking down -Z into an 800x600 viewport, so +X is screen-right
and +Y is screen-up and the gizmo origin projects to (400, 300).

What is asserted is the snap maths Tasks 2-4 will call: the shared
handle constraint, the world-grid rule (including its pivot-relative
fallback), screen-space ranking, the perspective-correct edge
parameter by round trip, and the mesh/curve topology helpers.

Usage: test_gizmo_snap.py [ignored]
"""
import math
import sys

# Sibling module: this script's own directory is sys.path[0]. It must
# run before the pxr import so pxr resolves from the configured USD
# install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf  # noqa: E402

import gizmoScreen as gs  # noqa: E402
import gizmoSettings as gset  # noqa: E402
import gizmoSnap as snap  # noqa: E402

VIEWPORT = (0, 0, 800, 600)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _Camera():
    camera = Gf.Camera()
    camera.transform = Gf.Matrix4d(1.0).SetTranslate(
        Gf.Vec3d(0, 0, 10))
    # usdview's StageView conforms the camera window to the viewport
    # before drawing; without the same conformance here a world unit
    # would cover a different number of pixels horizontally than
    # vertically and the projections would not be square.
    camera.verticalAperture = (camera.horizontalAperture
                               * VIEWPORT[3] / float(VIEWPORT[2]))
    return camera


def _Dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _Handles(matrix=None, camera=None):
    base = matrix if matrix is not None else Gf.Matrix4d(1.0)
    cam = camera if camera is not None else _Camera()
    return {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, base, cam, VIEWPORT, 1.0)}


def TestConstrain():
    handles = _Handles()
    pivot = Gf.Vec3d(1, 2, 3)
    world = Gf.Vec3d(4, 5, 6)
    centre = handles["center"]
    got = snap.ConstrainToHandle(centre, pivot, world)
    _Check(_Close(got[0], 4.0) and _Close(got[1], 5.0)
           and _Close(got[2], 6.0),
           "centre lands on the candidate: %s" % (got,))
    axis = handles["x"]
    got = snap.ConstrainToHandle(axis, pivot, world)
    _Check(_Close(got[0], 4.0) and _Close(got[1], 2.0)
           and _Close(got[2], 3.0),
           "axis projects onto its line: %s" % (got,))
    plane = handles["xy"]
    got = snap.ConstrainToHandle(plane, pivot, world)
    _Check(_Close(got[0], 4.0) and _Close(got[1], 5.0)
           and _Close(got[2], 3.0),
           "plane drops the normal component: %s" % (got,))
    got = snap.ConstrainToHandle(axis, pivot, world, ctrl=True)
    _Check(_Close(got[0], 1.0) and _Close(got[1], 5.0)
           and _Close(got[2], 6.0),
           "Ctrl+axis is a plane, not a line: %s" % (got,))
    again = snap.ConstrainToHandle(plane, pivot, world, ctrl=True)
    _Check(_Close(again[0], 4.0) and _Close(again[1], 5.0)
           and _Close(again[2], 3.0),
           "Ctrl does not change a plane handle: %s" % (again,))
    # The tokens are re-exported from gizmoSettings so there is one
    # spelling; the pixel constants are the spec section 4.4 numbers.
    _Check(snap.SNAP_OFF == gset.SNAP_OFF
           and snap.SNAP_GRID == gset.SNAP_GRID
           and snap.SNAP_POINT == gset.SNAP_POINT
           and snap.SNAP_EDGE == gset.SNAP_EDGE
           and snap.SNAP_SURFACE == gset.SNAP_SURFACE,
           "snap tokens re-export gizmoSettings")
    _Check(_Close(snap.SNAP_PIXELS, 12.0)
           and _Close(snap.PICK_MOVE_PIXELS, 3.0),
           "snap radii: %s %s"
           % (snap.SNAP_PIXELS, snap.PICK_MOVE_PIXELS))
    cand = snap.SnapCandidate(Gf.Vec3d(1, 2, 3),
                              Gf.Vec3d(0, 0, 1),
                              snap.SNAP_SURFACE)
    _Check(cand.index == -1 and cand.primPath is None,
           "candidate defaults: index -1, no prim")
    _Check(_Close(cand.point[0], 1.0)
           and _Close(cand.normal[2], 1.0)
           and cand.kind == snap.SNAP_SURFACE,
           "candidate carries point/normal/kind")


def TestGrid():
    pivot = Gf.Vec3d(0.4, -0.3, 0.25)  # off the grid and off every axis line
    placed = Gf.Matrix4d(1.0).SetTranslate(pivot)
    handles = _Handles(matrix=placed)
    _Check((Gf.Vec3d(handles["x"].worldOrigin) - pivot).GetLength()
           < 1e-9,
           "the handles sit on the pivot, as gizmoDrag passes them")
    centre = handles["center"]
    axis = handles["x"]
    plane = handles["xy"]
    # World-absolute means independence from pivot0: the same
    # unsnapped point lands identically from the origin and from
    # the pivot, which a pivot-relative mutant cannot match
    # (spec section 2 redefines X to snap the world pivot).
    for p0 in (Gf.Vec3d(0, 0, 0), pivot):
        got = snap.GridPoint(centre, p0,
                             Gf.Vec3d(1.4, -2.6, 0.2), 1.0)
        _Check(_Close(got[0], 1.0) and _Close(got[1], -3.0)
               and _Close(got[2], 0.0),
               "centre lands on the world grid whatever the pivot: %s"
               % (got,))
        got = snap.GridPoint(centre, p0,
                             Gf.Vec3d(0.5, -0.5, 1.5), 1.0)
        _Check(_Close(got[0], 1.0) and _Close(got[1], -1.0)
               and _Close(got[2], 2.0),
               "halves away from zero whatever the pivot: %s"
               % (got,))
        got = snap.GridPoint(axis, p0,
                             Gf.Vec3d(1.4, 2.6, 0.2), 1.0)
        _Check(_Close(got[0], 1.0) and _Close(got[1], 2.6)
               and _Close(got[2], 0.2),
               "axis rounds the world X, not the travel: %s"
               % (got,))
        got = snap.GridPoint(plane, p0,
                             Gf.Vec3d(1.4, -2.6, 5.0), 1.0)
        _Check(_Close(got[0], 1.0) and _Close(got[1], -3.0)
               and _Close(got[2], 5.0),
               "plane rounds the world pair: %s" % (got,))
        got = snap.GridPoint(axis, p0,
                             Gf.Vec3d(1.4, 2.6, 3.2), 1.0,
                             ctrl=True)
        _Check(_Close(got[0], 1.4) and _Close(got[1], 3.0)
               and _Close(got[2], 3.0),
               "Ctrl+axis rounds in-plane pair whatever pivot: %s"
               % (got,))
    _Check(snap.DirectionIsWorldAligned(Gf.Vec3d(1, 0, 0)) == 0,
           "+X is axis 0")
    _Check(snap.DirectionIsWorldAligned(Gf.Vec3d(-1, 0, 0)) == 0,
           "sign is ignored")
    _Check(snap.DirectionIsWorldAligned(Gf.Vec3d(0, 1, 0)) == 1,
           "+Y is axis 1")
    _Check(snap.DirectionIsWorldAligned(Gf.Vec3d(0, 0, -1)) == 2,
           "-Z is axis 2")
    _Check(snap.DirectionIsWorldAligned(Gf.Vec3d(1, 1, 0)) is None,
           "diagonal is not world-aligned")
    _Check(snap.DirectionIsWorldAligned(
        Gf.Vec3d(0.70710678, 0.70710678, 0)) is None,
           "45 degrees is not world-aligned")
    # A frame no world direction lines up with has no grid coordinate
    # in it, so the axis quantises the distance travelled instead.
    # R*T keeps the pivot as the origin with rotated axes: Gf is
    # row-vector (v*M), so BuildHandles reads the origin from
    # ExtractTranslation and the directions from TransformDir
    # (gizmoScreen.py:292-308), and only R*T gives both at once.
    rot = Gf.Matrix4d(1.0).SetRotate(
        Gf.Rotation(Gf.Vec3d(0, 1, 0), 45))
    tilted = rot * Gf.Matrix4d(1.0).SetTranslate(pivot)
    turned = _Handles(matrix=tilted)
    tiltedAxis = turned["x"]
    _Check(snap.DirectionIsWorldAligned(
        tiltedAxis.worldAxis) is None,
           "45-degree Y frame is not world-aligned: %s"
           % (tiltedAxis.worldAxis,))
    _Check((Gf.Vec3d(tiltedAxis.worldOrigin) - pivot).GetLength()
           < 1e-9,
           "tilted handles sit on the pivot too")
    along = Gf.Vec3d(tiltedAxis.worldAxis).GetNormalized()
    unsnapped = pivot + along * 1.4
    got = snap.GridPoint(tiltedAxis, pivot, unsnapped, 1.0)
    want = pivot + along * 1.0
    _Check((got - want).GetLength() < 1e-9,
           "relative fallback rounds the travel 1.4 -> 1: %s" % (got,))
    _Check(abs(got[0] - round(got[0])) > 1e-6,
           "the fallback is off the world grid: %s" % (got,))
    # The same fallback for a tilted plane: the result stays in the
    # plane through the pivot on a quantised in-plane lattice.
    tiltedPlane = turned["xy"]
    normal = Gf.Vec3d(tiltedPlane.worldNormal).GetNormalized()
    _Check(snap.DirectionIsWorldAligned(normal) is None,
           "tilted plane normal is not world-aligned")
    raw = Gf.Vec3d(1.4, 2.6, 0.0)
    # Project onto the plane through the pivot (spec 4.3); adding
    # the pivot twice coincides only at the origin.
    inPlane = raw - normal * Gf.Dot(raw - pivot, normal)
    got = snap.GridPoint(tiltedPlane, pivot, inPlane, 1.0)
    _Check(abs(Gf.Dot(got - pivot, normal)) < 1e-9,
           "relative plane result stays in the plane: %s" % (got,))
    # Recompute the spec 4.3 basis independently: e is the world axis
    # least parallel to n (ties X before Y before Z).
    dots = [abs(Gf.Dot(Gf.Vec3d(1, 0, 0), normal)),
            abs(Gf.Dot(Gf.Vec3d(0, 1, 0), normal)),
            abs(Gf.Dot(Gf.Vec3d(0, 0, 1), normal))]
    pick = min(range(3), key=lambda i: dots[i])
    basis = [Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0),
             Gf.Vec3d(0, 0, 1)][pick]
    unit = (basis - normal * Gf.Dot(basis, normal)).GetNormalized()
    other = Gf.Cross(normal, unit).GetNormalized()
    travel = inPlane - pivot
    want = pivot + unit * gs.SnapAbsolute(Gf.Dot(travel, unit), 1.0) \
        + other * gs.SnapAbsolute(Gf.Dot(travel, other), 1.0)
    _Check((got - want).GetLength() < 1e-9,
           "relative plane uses the (u, v) lattice: %s vs %s"
           % (got, want))


def TestRanking():
    camera = _Camera()
    viewProj = gs.ViewProjection(camera)
    points = [Gf.Vec3d(0, 0, 0), Gf.Vec3d(1, 0, 0),
              Gf.Vec3d(0, 1, 0)]
    found = snap.NearestPoint(points, (585, 300), viewProj,
                              VIEWPORT, 30.0)
    _Check(found is not None and found[0] == 1,
           "nearest in pixels wins: %s" % (found,))
    _Check(_Close(found[1][0], 1.0)
           and _Dist(found[2], (590.8852309502929, 300.0)) < 1e-6,
           "ranking returns the world point and its screen: %s"
           % (found,))
    # Screen-nearest and world-nearest differ: (1, 0, -9) is nine
    # times further from the origin than (1, 0, 0) yet projects a
    # hundred pixels closer to the viewport centre.
    near = Gf.Vec3d(1, 0, 0)
    far = Gf.Vec3d(1, 0, -9)
    _Check(near.GetLength() < far.GetLength(),
           "far is world-further by construction")
    nearScreen = gs.ProjectPoint(viewProj, VIEWPORT, near)
    farScreen = gs.ProjectPoint(viewProj, VIEWPORT, far)
    _Check(_Dist(farScreen, (400, 300))
           < _Dist(nearScreen, (400, 300)),
           "yet far is screen-nearer: %s vs %s"
           % (farScreen, nearScreen))
    found = snap.NearestPoint([near, far], (400, 300), viewProj,
                              VIEWPORT, 250.0)
    _Check(found is not None and found[0] == 1,
           "screen distance decides, not world: %s" % (found,))
    behind = Gf.Vec3d(0, 0, 20)
    _Check(gs.ProjectPoint(viewProj, VIEWPORT, behind) is None,
           "the probe point is really behind the eye")
    found = snap.NearestPoint([behind, Gf.Vec3d(0, 0, 0)],
                              (400, 300), viewProj, VIEWPORT, 50.0)
    _Check(found is not None and found[0] == 1,
           "behind the eye is dropped: %s" % (found,))
    _Check(snap.NearestPoint([behind], (400, 300), viewProj,
                             VIEWPORT, 50.0) is None,
           "only behind the eye is no candidate")
    _Check(snap.NearestPoint(points, (700, 500), viewProj,
                             VIEWPORT, 12.0) is None,
           "outside the radius is None")


def TestEdge():
    camera = _Camera()
    viewProj = gs.ViewProjection(camera)
    worldA = Gf.Vec3d(-1, 0, 2)
    worldB = Gf.Vec3d(2, 1, -10)
    screenA = gs.ProjectPoint(viewProj, VIEWPORT, worldA)
    screenB = gs.ProjectPoint(viewProj, VIEWPORT, worldB)
    clipA = Gf.Vec4d(worldA[0], worldA[1], worldA[2], 1.0) * viewProj
    clipB = Gf.Vec4d(worldB[0], worldB[1], worldB[2], 1.0) * viewProj
    posA, wA = gs.ProjectPointWithW(viewProj, VIEWPORT, worldA)
    posB, wB = gs.ProjectPointWithW(viewProj, VIEWPORT, worldB)
    _Check(_Dist(posA, screenA) < 1e-9 and _Close(wA, clipA[3]),
           "ProjectPointWithW matches ProjectPoint and clip w")
    _Check(_Dist(posB, screenB) < 1e-9 and _Close(wB, clipB[3]),
           "ProjectPointWithW at the far end too")
    for probeT in (0.3, 0.5, 0.7):
        foot = (screenA[0] + probeT * (screenB[0] - screenA[0]),
                screenA[1] + probeT * (screenB[1] - screenA[1]))
        worldT = snap.WorldParameterFromScreen(probeT, wA, wB)
        world = worldA + (worldB - worldA) * worldT
        back = gs.ProjectPoint(viewProj, VIEWPORT, world)
        _Check(_Dist(back, foot) < 1e-6,
               "round trip t=%.1f lands on its foot: %s vs %s"
               % (probeT, back, foot))
    # The segment is foreshortened enough that the naive world lerp
    # is tens of pixels off, so the round trip above is not vacuous.
    foot = (screenA[0] + 0.3 * (screenB[0] - screenA[0]),
            screenA[1] + 0.3 * (screenB[1] - screenA[1]))
    naive = gs.ProjectPoint(viewProj, VIEWPORT,
                            worldA + (worldB - worldA) * 0.3)
    _Check(_Dist(naive, foot) > 10.0,
           "naive lerp is far off on this segment: %s vs %s"
           % (naive, foot))
    dist, gotT = snap.SegmentScreenParameter(screenA, screenB, foot)
    _Check(_Close(gotT, 0.3, 1e-9) and dist < 1e-9,
           "the foot parameterises to its own t: %s %s"
           % (dist, gotT))
    past = (screenB[0] + (screenB[0] - screenA[0]),
            screenB[1] + (screenB[1] - screenA[1]))
    _, clamped = snap.SegmentScreenParameter(screenA, screenB, past)
    _Check(_Close(clamped, 1.0),
           "past the far end clamps to 1: %s" % (clamped,))
    _Check(_Close(snap.WorldParameterFromScreen(0.3, -1.0, 5.0),
                  0.3)
           and _Close(snap.WorldParameterFromScreen(0.3, 5.0, -2.0),
                      0.3),
           "non-positive w returns t untouched")
    found = snap.NearestSegment([(worldA, worldB)], foot, viewProj,
                                VIEWPORT, 50.0)
    _Check(found is not None and found[0] == 0,
           "the foreshortened segment is found: %s" % (found,))
    _Check(_Dist(found[2], foot) < 1e-6,
           "its screen is the foot: %s vs %s" % (found[2], foot))
    back = gs.ProjectPoint(viewProj, VIEWPORT, found[1])
    _Check(_Dist(back, foot) < 1e-6,
           "its world reprojects onto the foot: %s" % (back,))
    badA = Gf.Vec3d(0, 0, 20)
    _Check(gs.ProjectPoint(viewProj, VIEWPORT, badA) is None,
           "the bad endpoint is really behind the eye")
    _Check(snap.NearestSegment([(badA, worldA)], (400, 300),
                               viewProj, VIEWPORT, 50.0) is None,
           "a segment with w <= 0 is dropped, not clipped")
    mixed = snap.NearestSegment([(badA, worldA), (worldA, worldB)],
                                foot, viewProj, VIEWPORT, 50.0)
    _Check(mixed is not None and mixed[0] == 1,
           "a good segment still wins past a bad one: %s" % (mixed,))
    _Check(snap.NearestSegment([(worldA, worldB)], (700, 500),
                               viewProj, VIEWPORT, 12.0) is None,
           "outside the radius is None")


def TestTopology():
    edges = snap.MeshEdges([4], [0, 1, 2, 3])
    _Check(len(edges) == 4, "a quad closes its loop: %s" % (edges,))
    _Check(set(map(tuple, [tuple(sorted(e)) for e in edges]))
           == set([(0, 1), (1, 2), (2, 3), (0, 3)]),
           "quad edges: %s" % (edges,))
    shared = snap.MeshEdges([3, 3], [0, 1, 2, 1, 3, 2])
    flat = [tuple(sorted(e)) for e in shared]
    _Check(len(shared) == 5, "shared edge deduped: %s" % (shared,))
    _Check(flat.count((1, 2)) == 1,
           "the shared edge appears once: %s" % (shared,))
    segs = snap.CurveSegments([3, 2], 5)
    _Check(segs == [(0, 1), (1, 2), (3, 4)],
           "consecutive CVs, never across curves: %s" % (segs,))
    _Check((2, 3) not in segs, "two curves are not joined")
    _Check(snap.CurveSegments([1], 1) == [],
           "a lone CV has no segment")
    _Check(snap.MeshEdges([], []) == [],
           "no faces is no edges")


def main():
    groups = [
        ("constrain", TestConstrain),
        ("grid", TestGrid),
        ("ranking", TestRanking),
        ("edge", TestEdge),
        ("topology", TestTopology),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_SNAP_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
