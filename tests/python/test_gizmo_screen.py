#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoScreen.py using a synthetic
Gf.Camera at (0, 0, 10) looking down -Z into an 800x600 viewport, so
+X is screen-right and +Y is screen-up and the gizmo origin projects to
(400, 300). Usage: test_gizmo_screen.py [ignored]
"""
import math
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf  # noqa: E402

import gizmoScreen as gs  # noqa: E402

VIEWPORT = (0, 0, 800, 600)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _Camera():
    camera = Gf.Camera()
    camera.transform = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 0, 10))
    # The default aperture is Academy 1.37, which does not match the
    # 800x600 viewport, so a world unit would cover a different number of
    # pixels horizontally than vertically.  usdview's StageView conforms
    # the camera window to the viewport before drawing, so the synthetic
    # camera has to conform too or the "square pixels" the gizmo layout
    # assumes are not square.
    camera.verticalAperture = (camera.horizontalAperture
                               * VIEWPORT[3] / float(VIEWPORT[2]))
    return camera


def _Dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def TestProjection():
    camera = _Camera()
    vp = gs.ViewProjection(camera)
    centre = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(0, 0, 0))
    _Check(_Close(centre[0], 400, 1e-3) and _Close(centre[1], 300, 1e-3),
           "origin projects to the viewport centre: %s" % (centre,))
    right = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(1, 0, 0))
    _Check(right[0] > 400 and _Close(right[1], 300, 1e-3), "+X is right")
    up = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(0, 1, 0))
    _Check(up[1] < 300 and _Close(up[0], 400, 1e-3), "+Y is up")
    _Check(gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(0, 0, 20)) is None,
           "behind the eye is None")
    viewDir, upVec, rightVec = gs.CameraBasis(camera)
    _Check(_Close(viewDir[2], -1.0) and _Close(upVec[1], 1.0)
           and _Close(rightVec[0], 1.0), "camera basis")
    wpp = gs.WorldPerPixel(camera, VIEWPORT, Gf.Vec3d(0, 0, 0))
    one = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(wpp, 0, 0))
    _Check(_Close(one[0], 401.0, 1e-3), "world-per-pixel: %s" % (one,))


def TestTranslateHandles():
    camera = _Camera()
    handles = gs.BuildHandles(gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera,
                              VIEWPORT, 1.0)
    byName = {h.name: h for h in handles}
    # Maya's Move manipulator: three axes, three planar handles and the
    # view-plane centre (design section 8.2).
    _Check(set(byName) == {"x", "y", "z", "xy", "yz", "xz", "center"},
           "move handles: %s" % sorted(byName))
    x = byName["x"]
    _Check(x.kind == "axis" and x.axisIndex == 0
           and _Close(x.points[1][0], 400 + gs.GIZMO_PIXELS, 1e-3)
           and _Close(x.points[1][1], 300, 1e-3),
           "x axis is GIZMO_PIXELS long on screen: %s" % (x.points,))
    y = byName["y"]
    _Check(_Close(y.points[1][1], 300 - gs.GIZMO_PIXELS, 1e-3), "y up")
    z = byName["z"]
    _Check(_Dist(z.points[0], z.points[1]) < 1.0,
           "z axis points at the camera and projects to a point")
    # A foreshortened axis is still drawn but cannot be dragged: the
    # artist orbits a few degrees rather than dragging an axis that
    # points at the camera.
    _Check(z.grabbable is False, "z axis is not grabbable")
    _Check(x.grabbable is True, "x axis is grabbable")
    _Check(_Close(x.worldLength, gs.GIZMO_PIXELS
                  * gs.WorldPerPixel(camera, VIEWPORT, Gf.Vec3d(0, 0, 0)),
                  1e-9), "world length matches the pixel length")
    _Check(byName["center"].kind == "center"
           and _Close(byName["center"].points[0][0], 400, 1e-3), "centre")
    # Pixel ratio scales the on-screen size.
    hi = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 2.0)}
    _Check(_Close(hi["x"].points[1][0], 400 + 2 * gs.GIZMO_PIXELS, 1e-3),
           "pixel ratio 2 doubles the physical length")
    # A rotated gizmo frame rotates the handles.
    rotated = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))
    rx = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, rotated, camera, VIEWPORT, 1.0)}["x"]
    _Check(_Close(rx.points[1][1], 300 - gs.GIZMO_PIXELS, 1e-3)
           and _Close(rx.worldAxis[1], 1.0), "local X now points up")
    # Origin behind the eye: no handles at all.
    behind = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 0, 30))
    _Check(gs.BuildHandles(gs.TOOL_TRANSLATE, behind, camera, VIEWPORT, 1.0)
           == [], "nothing to draw behind the camera")


def TestRotateHandles():
    camera = _Camera()
    handles = gs.BuildHandles(gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera,
                              VIEWPORT, 1.0)
    byName = {h.name: h for h in handles}
    # Maya's Rotate manipulator adds the view-axis ring and the
    # free-rotate sphere to the three axis rings (design section 8.3).
    _Check(set(byName) == {"x", "y", "z", "view", "free"},
           "rotate handles: %s" % sorted(byName))
    z = byName["z"]
    _Check(z.kind == "ring" and len(z.points) == gs.RING_SEGMENTS, "ring")
    radius = gs.GIZMO_PIXELS * gs.RING_FRACTION
    _Check(all(_Close(_Dist(p, (400, 300)), radius, 0.5) for p in z.points),
           "z ring is a circle of RING_FRACTION * GIZMO_PIXELS")
    xs = [p[0] for p in byName["x"].points]
    _Check(max(xs) - min(xs) < 1.0, "x ring is edge-on: a vertical line")


def TestHitTest():
    camera = _Camera()
    handles = gs.BuildHandles(gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera,
                              VIEWPORT, 1.0)
    hit = gs.HitTest(handles, 445, 303, gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "x", "x axis hit")
    hit = gs.HitTest(handles, 398, 255, gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "y", "y axis hit")
    hit = gs.HitTest(handles, 401, 299, gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "center",
           "centre wins over the axes that start there: %s"
           % (hit and hit.name))
    _Check(gs.HitTest(handles, 600, 600, gs.HIT_PIXELS) is None, "miss")
    # The z axis collapses onto the centre; clicking its tip is the artist
    # aiming at the centre handle, not at an axis they cannot drag.
    zPoint = {h.name: h for h in handles}["z"].points[1]
    hit = gs.HitTest(handles, zPoint[0], zPoint[1], gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "center",
           "the ungrabbable z axis does not steal its own tip: %s"
           % (hit and hit.name))
    # An axis 2 degrees off the view direction is ungrabbable too.
    edgeOn = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), 88))
    edge = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, edgeOn, camera, VIEWPORT, 1.0)}
    ex = edge["x"]
    _Check(ex.grabbable is False,
           "an x axis 2 degrees off the view direction is not grabbable: "
           "%s px" % _Dist(ex.points[0], ex.points[1]))
    hit = gs.HitTest(list(edge.values()), ex.points[1][0], ex.points[1][1],
                     gs.HIT_PIXELS)
    _Check(hit is None or hit.name != "x",
           "an ungrabbable axis is never picked: %s" % (hit and hit.name))
    rings = gs.BuildHandles(gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera,
                            VIEWPORT, 1.0)
    radius = gs.GIZMO_PIXELS * gs.RING_FRACTION
    hit = gs.HitTest(rings, 400 + radius * 0.7071, 300 - radius * 0.7071,
                     gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "z", "z ring hit at 45 degrees")


def TestDragMath():
    camera = _Camera()
    handles = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)}
    x = handles["x"]
    t = gs.AxisDragParameter(x, (400, 300), (445, 310))
    _Check(_Close(t, 0.5, 1e-9), "half the handle length: %s" % t)
    delta = x.worldAxis * (t * x.worldLength)
    vp = gs.ViewProjection(camera)
    moved = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(delta))
    _Check(_Close(moved[0], 445.0, 1e-3), "the world delta projects back "
           "to the mouse travel: %s" % (moved,))
    # A degenerate (foreshortened) axis does not explode.
    z = handles["z"]
    _Check(abs(gs.AxisDragParameter(z, (400, 300), (500, 300)))
           <= 100.0 / gs.MIN_AXIS_PIXELS + 1e-9,
           "foreshortened axis is clamped to the MIN_AXIS_PIXELS floor")
    wpp = gs.WorldPerPixel(camera, VIEWPORT, Gf.Vec3d(0, 0, 0))
    plane = gs.PlaneDragDelta(camera, VIEWPORT, Gf.Vec3d(0, 0, 0),
                              (400, 300), (410, 290))
    _Check(_Close(plane[0], 10 * wpp, 1e-9) and _Close(plane[1], 10 * wpp,
                                                         1e-9)
           and _Close(plane[2], 0.0), "screen-plane delta: %s" % plane)
    _Check(gs.AxisFacesCamera(camera, Gf.Vec3d(0, 0, 1)), "+Z faces us")
    _Check(not gs.AxisFacesCamera(camera, Gf.Vec3d(0, 0, -1)), "-Z away")
    angle = gs.RotationDragAngle((400, 300), (500, 300), (400, 200), True)
    _Check(_Close(angle, 90.0, 1e-9), "CCW on screen is +90 facing: %s"
           % angle)
    angle = gs.RotationDragAngle((400, 300), (500, 300), (400, 200), False)
    _Check(_Close(angle, -90.0, 1e-9), "sign flips for an axis pointing "
           "away")
    angle = gs.RotationDragAngle((400, 300), (500, 300), (300, 301), True)
    _Check(179.0 < angle <= 180.0 or -180.0 <= angle < -179.0,
           "wrapped into (-180, 180]: %s" % angle)
    scale = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_SCALE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)}
    _Check(set(scale) == {"x", "y", "z", "xy", "yz", "xz", "center"},
           "scale handles: %s" % sorted(scale))
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (460, 300), 1.0)
    _Check(_Close(f, 1.5, 1e-9), "60 px along x: %s" % f)
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (400, 360), 1.0)
    _Check(_Close(f, 1.0, 1e-9), "perpendicular travel does nothing")
    f = gs.ScaleDragFactor(scale["center"], (400, 300), (340, 300), 1.0)
    _Check(_Close(f, 0.5, 1e-9), "centre: horizontal travel, uniform")
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (0, 300), 1.0)
    _Check(f >= 0.01, "factor is floored")


def TestMayaTranslateHandles():
    camera = _Camera()
    byName = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)}
    _Check(set(byName) == {"x", "y", "z", "xy", "yz", "xz", "center"},
           "Maya move handles: %s" % sorted(byName))
    xy = byName["xy"]
    _Check(xy.kind == "plane" and xy.axisIndex == 2
           and xy.color == (0.0, 0.0, 1.0), "xy plane is blue (normal z)")
    _Check(byName["yz"].color == (1.0, 0.0, 0.0)
           and byName["xz"].color == (0.0, 1.0, 0.0), "plane colours")
    xs = [p[0] for p in xy.points]
    ys = [p[1] for p in xy.points]
    side = gs.GIZMO_PIXELS * gs.PLANE_SIDE
    _Check(_Close(max(xs) - min(xs), side, 0.5)
           and _Close(max(ys) - min(ys), side, 0.5),
           "xy square side is PLANE_SIDE * size on screen")
    centre = ((max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0)
    off = gs.GIZMO_PIXELS * gs.PLANE_OFFSET
    _Check(_Close(centre[0], 400 + off, 0.5)
           and _Close(centre[1], 300 - off, 0.5),
           "xy square sits at PLANE_OFFSET along +x and +y: %s" % (centre,))
    _Check(_Close(xy.worldNormal[2], 1.0), "xy plane normal is +z")
    _Check(byName["center"].color == gs.COLOR_VIEW, "centre is light blue")
    # Axis orientation: world axes with a rotated gizmo frame.
    rotated = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))
    world = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, rotated, camera, VIEWPORT, 1.0,
        orientation=Gf.Matrix4d(1.0))}
    _Check(_Close(world["x"].worldAxis[0], 1.0)
           and _Close(world["x"].points[1][0], 400 + gs.GIZMO_PIXELS, 1e-3),
           "orientation=identity draws world axes despite the frame")
    # Manipulator size.
    big = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0,
        sizePixels=180.0)}
    _Check(_Close(big["x"].points[1][0], 580, 1e-3), "sizePixels honoured")


def TestMayaRotateHandles():
    camera = _Camera()
    byName = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)}
    _Check(set(byName) == {"x", "y", "z", "view", "free"},
           "Maya rotate handles: %s" % sorted(byName))
    view = byName["view"]
    radius = gs.GIZMO_PIXELS * gs.RING_FRACTION * gs.VIEW_RING_FRACTION
    _Check(view.kind == "view" and view.color == gs.COLOR_VIEW
           and all(_Close(_Dist(p, (400, 300)), radius, 0.5)
                   for p in view.points), "view ring is 1.25x, light blue")
    _Check(_Close(view.worldAxis[2], 1.0), "view axis points at the camera")
    free = byName["free"]
    _Check(free.kind == "sphere"
           and _Close(free.radiusPixels, gs.GIZMO_PIXELS * gs.RING_FRACTION,
                      1e-6), "free-rotate disc at the ring radius")
    z = byName["z"]
    # A fully visible ring is one closed arc: the first point is repeated
    # at the end so it draws and picks as a circle rather than a circle
    # with a notch between the last index and the first.
    _Check(len(z.frontPoints) == 1 and sum(len(a) for a in z.frontPoints)
           == gs.RING_SEGMENTS + 1,
           "z ring faces the camera: fully visible and closed")
    _Check(z.frontPoints[0][0] == z.frontPoints[0][-1],
           "fully visible ring closes on itself")
    _Check(byName["view"].frontPoints[0][0]
           == byName["view"].frontPoints[0][-1], "view ring closes too")
    x = byName["x"]
    front = sum(len(a) for a in x.frontPoints)
    _Check(gs.RING_SEGMENTS * 0.4 <= front <= gs.RING_SEGMENTS * 0.6,
           "edge-on x ring shows about half its points: %d" % front)
    _Check(x.frontPoints[0][0] != x.frontPoints[0][-1],
           "a half-visible arc stays open: its ends are the horizon")
    _Check(all(p[2] >= -1e-6 for p in x.frontWorld),
           "front half = points on the camera side of the ring centre")
    # The closing segment is pickable, not a dead gap.
    big = gs.BuildHandles(gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera,
                          VIEWPORT, 1.0, sizePixels=180.0)
    bigZ = {h.name: h for h in big}["z"]
    seam = ((bigZ.points[-1][0] + bigZ.points[0][0]) / 2.0,
            (bigZ.points[-1][1] + bigZ.points[0][1]) / 2.0)
    hit = gs.HitTest(big, seam[0], seam[1], gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "z",
           "the seam between the last and first ring point is pickable: %s"
           % (hit and hit.name))
    # Gimbal axes override the ring axes.
    axes = [Gf.Vec3d(0, 1, 0), Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 0, 1)]
    gimbal = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0,
        gimbalAxes=axes)}
    _Check(_Close(gimbal["x"].worldAxis[1], 1.0), "x ring uses gimbal axis")
    none = gs.BuildHandles(gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera,
                           VIEWPORT, 1.0, freeRotate=False)
    _Check("free" not in {h.name for h in none}, "freeRotate=False")
    # Hit priority: ring beats view ring beats sphere; back half not hit.
    hit = gs.HitTest(list(byName.values()), 400 + radius, 300, gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "view", "view ring hit")
    hit = gs.HitTest(list(byName.values()), 400 + 20, 300 - 20, gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "free", "inside disc: free")
    _Check(gs.HitTest(list(byName.values()), 700, 300, gs.HIT_PIXELS)
           is None, "outside everything")


def TestMayaScaleHandles():
    camera = _Camera()
    byName = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_SCALE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)}
    _Check(set(byName) == {"x", "y", "z", "xy", "yz", "xz", "center"},
           "Maya scale handles: %s" % sorted(byName))
    origin = (400, 300)
    x = byName["x"]
    f = gs.MayaScaleFactor(x, origin, (445, 300), (490, 300), True)
    _Check(_Close(f, 2.0, 1e-9), "press at half length, drag to the tip: 2x")
    # Design section 8.4 defines the factor as (cursor distance from the
    # origin along the axis) / (that distance at press). Pressing 45 px
    # out and dragging to 45 px on the far side is -45/45, so the factor
    # is -1: same size, mirrored. The sign flip is what matters here.
    f = gs.MayaScaleFactor(x, origin, (445, 300), (355, 300), True)
    _Check(_Close(f, -1.0, 1e-9), "through the origin flips the sign")
    f = gs.MayaScaleFactor(x, origin, (445, 300), (355, 300), False)
    _Check(_Close(f, 1e-4, 1e-12), "Prevent Negative Scale clamps")
    c = byName["center"]
    f = gs.MayaScaleFactor(c, origin, (400, 300), (445, 300), True)
    _Check(_Close(f, 1.5, 1e-9), "centre: 1 + dx / size")
    xy = byName["xy"]
    d = (xy.worldCenterScreen[0] - 400, xy.worldCenterScreen[1] - 300)
    press = (400 + d[0], 300 + d[1])
    current = (400 + 2 * d[0], 300 + 2 * d[1])
    f = gs.MayaScaleFactor(xy, origin, press, current, True)
    _Check(_Close(f, 2.0, 1e-6), "plane handle: ratio along the diagonal")


def TestMayaDragMath():
    camera = _Camera()
    delta = gs.RayPlaneDragDelta(camera, VIEWPORT, Gf.Vec3d(0, 0, 0),
                                 Gf.Vec3d(0, 0, 1), (400, 300), (410, 290))
    wpp = gs.WorldPerPixel(camera, VIEWPORT, Gf.Vec3d(0, 0, 0))
    _Check(_Close(delta[0], 10 * wpp, 1e-6) and _Close(delta[1], 10 * wpp,
                                                          1e-6)
           and _Close(delta[2], 0.0, 1e-9),
           "ray/plane on the z=0 plane matches the camera-plane delta")
    delta = gs.RayPlaneDragDelta(camera, VIEWPORT, Gf.Vec3d(0, 0, 0),
                                 Gf.Vec3d(1, 0, 0), (400, 300), (400, 290))
    _Check(_Close(delta[0], 0.0, 1e-9) and delta[1] > 0.0,
           "yz plane: no x component, moves up: %s" % delta)
    total = gs.AccumulateAngle(170.0, 170.0, -175.0)
    _Check(_Close(total, 185.0, 1e-9), "accumulates through the wrap")
    axis, degrees = gs.TrackballRotation(camera, (400, 300), (490, 300),
                                         90.0)
    _Check(_Close(axis[1], 1.0, 1e-9) and _Close(degrees, 90.0, 1e-9),
           "dragging right one radius: +90 about +Y: %s %s" % (axis,
                                                               degrees))
    _Check(gs.TrackballRotation(camera, (400, 300), (400, 300), 90.0)
           is None, "no travel: None")
    _Check(_Close(gs.SnapRelative(2.4, 1.0), 2.0)
           and _Close(gs.SnapRelative(-2.6, 1.0), -3.0)
           and _Close(gs.SnapRelative(37.0, 15.0), 30.0), "relative snap")
    v = gs.SnapAbsolute(Gf.Vec3d(0.4, 1.6, -0.5), 1.0)
    _Check(v == Gf.Vec3d(0.0, 2.0, -0.0) or v == Gf.Vec3d(0.0, 2.0, 0.0)
           or _Close(v[2], -1.0), "vector snap: %s" % v)
    rings = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)}
    z = rings["z"]
    r = gs.GIZMO_PIXELS * gs.RING_FRACTION
    t0 = gs.RingParameter(z, (400 + r, 300))
    pie = gs.PiePolygon(z, t0, 90.0)
    _Check(pie[0] == z.center and len(pie) >= gs.RING_SEGMENTS // 4,
           "pie starts at the centre and walks a quarter of the ring")
    last = pie[-1]
    _Check(_Close(last[0], 400, 2.0) and _Close(last[1], 300 - r, 2.0),
           "+90 sweep facing the camera ends at the top: %s" % (last,))
    # The wedge must end under the cursor whichever way the ring's axis
    # points. The sweep is degrees about the ring's OWN world axis, which
    # is what RotationDragAngle produces and what the controller applies,
    # so the walk follows the sweep sign and not the screen winding.
    away = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))
    awayZ = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_ROTATE, away, camera, VIEWPORT, 1.0)}["z"]
    _Check(_Close(awayZ.worldAxis[2], -1.0)
           and not gs.AxisFacesCamera(camera, awayZ.worldAxis),
           "the flipped frame's z ring points away from the camera")
    for ring in (z, awayZ):
        press, current = (400 + r, 300), (400, 300 - r)
        sweep = gs.RotationDragAngle(
            ring.center, press, current,
            gs.AxisFacesCamera(camera, ring.worldAxis))
        end = gs.PiePolygon(ring, gs.RingParameter(ring, press), sweep)[-1]
        _Check(_Close(end[0], current[0], 2.0)
               and _Close(end[1], current[1], 2.0),
               "the wedge ends under the cursor (sweep %.1f): %s"
               % (sweep, (end,)))


def main():
    groups = [
        ("projection", TestProjection),
        ("translate handles", TestTranslateHandles),
        ("rotate handles", TestRotateHandles),
        ("hit test", TestHitTest),
        ("drag math", TestDragMath),
        ("maya translate handles", TestMayaTranslateHandles),
        ("maya rotate handles", TestMayaRotateHandles),
        ("maya scale handles", TestMayaScaleHandles),
        ("maya drag math", TestMayaDragMath),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_SCREEN_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
