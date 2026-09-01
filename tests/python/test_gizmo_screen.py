#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoScreen.py using a synthetic
Gf.Camera at (0, 0, 10) looking down -Z into an 800x600 viewport, so
+X is screen-right and +Y is screen-up and the gizmo origin projects to
(400, 300). Usage: test_gizmo_screen.py [ignored]
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(
    os.path.join(_HERE, "..", "..", "plugin", "rigExecUsdview")))

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
    _Check(set(byName) == {"x", "y", "z", "center"}, "four handles")
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
    _Check(set(byName) == {"x", "y", "z"}, "three rings")
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
    _Check(set(scale) == {"x", "y", "z", "center"}, "scale handles")
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (460, 300), 1.0)
    _Check(_Close(f, 1.5, 1e-9), "60 px along x: %s" % f)
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (400, 360), 1.0)
    _Check(_Close(f, 1.0, 1e-9), "perpendicular travel does nothing")
    f = gs.ScaleDragFactor(scale["center"], (400, 300), (340, 300), 1.0)
    _Check(_Close(f, 0.5, 1e-9), "centre: horizontal travel, uniform")
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (0, 300), 1.0)
    _Check(f >= 0.01, "factor is floored")


def main():
    groups = [
        ("projection", TestProjection),
        ("translate handles", TestTranslateHandles),
        ("rotate handles", TestRotateHandles),
        ("hit test", TestHitTest),
        ("drag math", TestDragMath),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_SCREEN_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
