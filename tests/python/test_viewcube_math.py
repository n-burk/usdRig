#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/viewCubeMath.py: the 26 cube
regions, the free-camera angle convention, the cube projection and the
ray-cast hit test, all driven with synthetic camera bases so no Qt and
no stage are needed.

The angle table repeats the measurement of spec section 2 (verified
against usdview's real FreeCamera in both up modes); the last group
repeats it live against FreeCamera itself.

Usage: test_viewcube_math.py [ignored]
"""
import math
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf  # noqa: E402

import viewCubeMath as vm  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _VecClose(a, b, tol=1e-6):
    return all(abs(a[i] - b[i]) <= tol for i in range(3))


def _MatClose(a, b, tol=1e-9):
    return all(abs(a[i][j] - b[i][j]) <= tol
               for i in range(4) for j in range(4))


def _PolygonArea(points):
    total = 0.0
    for i in range(len(points)):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % len(points)]
        total += x0 * y1 - x1 * y0
    return abs(total) * 0.5


def TestRegions():
    _Check(len(vm.REGIONS) == 26, "26 regions: %d" % len(vm.REGIONS))
    _Check("front-top-right" in vm.REGIONS, "home corner is canonical")
    _Check("right-top-front" not in vm.REGIONS,
           "no non-canonical name is registered")
    counts = {1: 0, 2: 0, 3: 0}
    for name, region in sorted(vm.REGIONS.items()):
        # The name is the canonical join of the faces: front|back, then
        # top|bottom, then left|right.
        _Check(name == vm.RegionName(region.faces),
               "region name is canonical: %s" % name)
        _Check(1 <= len(region.faces) <= 3, "1-3 faces: %s" % name)
        counts[len(region.faces)] += 1
        total = Gf.Vec3d(0, 0, 0)
        for face in region.faces:
            total += vm.FACE_NORMALS[face]
        want = total.GetNormalized()
        _Check(_VecClose(region.direction, want, 1e-9),
               "%s points along its face normals" % name)
        _Check(_Close(region.direction.GetLength(), 1.0, 1e-9),
               "%s is unit: %s" % (name, region.direction))
    _Check(counts == {1: 6, 2: 12, 3: 8},
           "6 faces, 12 edges, 8 corners: %s" % (counts,))
    _Check(vm.RegionName(("right", "top", "front")) == "front-top-right",
           "RegionName sorts any order into canonical order")
    _Check(vm.RegionName(("left", "bottom", "back")) == "back-bottom-left",
           "RegionName: back-bottom-left")
    _Check(vm.RegionForFaces(("top", "front")).name == "front-top",
           "RegionForFaces accepts any order")
    _Check(vm.RegionForFaces(("right", "front", "top")).name
           == "front-top-right", "RegionForFaces: corner")
    for face in vm.FACES:
        right, up = vm.FACE_AXES[face]
        _Check(_Close(right.GetLength(), 1.0, 1e-9)
               and _Close(up.GetLength(), 1.0, 1e-9)
               and _Close(Gf.Dot(right, up), 0.0, 1e-9),
               "%s label axes are orthonormal" % face)
        _Check(_VecClose(Gf.Cross(right, up), vm.FACE_NORMALS[face],
                         1e-9),
               "%s: right x up == normal" % face)


def TestAngles():
    # Spec section 2: the measured FreeCamera position directions.
    homePhi = math.degrees(math.asin(1.0 / math.sqrt(3.0)))
    cases = [
        ((0, 0), (0, 0, 1)),
        ((90, 0), (-1, 0, 0)),
        ((-90, 0), (1, 0, 0)),
        ((270, 0), (1, 0, 0)),
        ((180, 0), (0, 0, -1)),
        ((0, 90), (0, 1, 0)),
        ((0, -90), (0, -1, 0)),
        ((45, 0), (-math.sqrt(0.5), 0, math.sqrt(0.5))),
        ((0, 45), (0, math.sqrt(0.5), math.sqrt(0.5))),
        ((-45, homePhi), (1, 1, 1)),
    ]
    for (theta, phi), want in cases:
        got = vm.DirectionForAngles(theta, phi)
        if want == (1, 1, 1):
            wantVec = Gf.Vec3d(1, 1, 1).GetNormalized()
        else:
            wantVec = Gf.Vec3d(*want)
        _Check(_VecClose(got, wantVec, 1e-9),
               "DirectionForAngles(%s, %s) = %s, want %s"
               % (theta, phi, got, wantVec))
    # Round trip everywhere off the poles (at |phi| = 90 the heading is
    # not recoverable and the snap rule applies instead).
    for theta in range(-180, 181, 15):
        for phi in range(-80, 81, 10):
            back = vm.AnglesForDirection(
                vm.DirectionForAngles(theta, phi), theta)
            _Check(_Close(back[0], theta, 1e-6)
                   and _Close(back[1], phi, 1e-6),
                   "round trip (%s, %s) -> %s" % (theta, phi, back))
    # A pure TOP / BOTTOM view keeps the current heading snapped to 90.
    for pole, name in (((0, 1, 0), "TOP"), ((0, -1, 0), "BOTTOM")):
        pole = Gf.Vec3d(*pole)
        _Check(_Close(vm.AnglesForDirection(pole, 100)[0], 90.0, 1e-9),
               "%s keeps heading 100 as 90" % name)
        _Check(_Close(vm.AnglesForDirection(pole, 140)[0], 180.0, 1e-9),
               "%s keeps heading 140 as 180" % name)
        _Check(_Close(vm.AnglesForDirection(pole, -100)[0], -90.0, 1e-9),
               "%s keeps heading -100 as -90" % name)
    _Check(_Close(vm.Unwrap(350, 10), -10.0, 1e-9), "Unwrap wraps down")
    _Check(_Close(vm.Unwrap(-170, 170), 190.0, 1e-9), "Unwrap wraps up")
    _Check(_Close(vm.Unwrap(0, 0), 0.0, 1e-9), "Unwrap identity")
    _Check(_Close(vm.Unwrap(180, 0), 180.0, 1e-9), "Unwrap keeps +180")
    ends = vm.LerpAngles((10.0, 20.0), (350.0, -40.0), 0.0)
    _Check(ends == (10.0, 20.0),
           "LerpAngles hits t=0 exactly: %s" % (ends,))
    ends = vm.LerpAngles((10.0, 20.0), (350.0, -40.0), 1.0)
    _Check(ends == (350.0, -40.0),
           "LerpAngles hits t=1 exactly: %s" % (ends,))
    _Check(_Close(vm.Smoothstep(0.5), 0.5, 1e-12), "Smoothstep midpoint")
    _Check(vm.Smoothstep(-1.0) == 0.0, "Smoothstep clamps below")
    _Check(vm.Smoothstep(2.0) == 1.0, "Smoothstep clamps above")


def TestFrames():
    toWorld = vm.UpSpaceToWorld(True)
    _Check(_VecClose(toWorld.TransformDir(Gf.Vec3d(0, 0, 1)),
                     Gf.Vec3d(0, -1, 0), 1e-9),
           "Z-up FRONT (+Z_up) sits on world -Y")
    _Check(_VecClose(toWorld.TransformDir(Gf.Vec3d(0, 1, 0)),
                     Gf.Vec3d(0, 0, 1), 1e-9),
           "Z-up up (+Y_up) is world +Z")
    for isZUp in (False, True):
        there = vm.UpSpaceToWorld(isZUp)
        back = vm.WorldToUpSpace(isZUp)
        _Check(_MatClose(there * back, Gf.Matrix4d(1.0)),
               "the two frames are inverses (isZUp=%s)" % isZUp)
    ident = vm.BasisForAngles(0, 0, False)
    _Check(_VecClose(ident.right, vm.IDENTITY_BASIS.right, 1e-12)
           and _VecClose(ident.up, vm.IDENTITY_BASIS.up, 1e-12)
           and _VecClose(ident.view, vm.IDENTITY_BASIS.view, 1e-12),
           "BasisForAngles(0, 0, Y-up) is the identity basis")
    zUp = vm.BasisForAngles(0, 0, True)
    _Check(_VecClose(zUp.view, Gf.Vec3d(0, 1, 0), 1e-9),
           "Z-up FRONT looks down +Y: %s" % zUp.view)
    _Check(_VecClose(zUp.up, Gf.Vec3d(0, 0, 1), 1e-9),
           "Z-up FRONT is up +Z: %s" % zUp.up)


def _BasisForRegion(region, isZUp):
    theta, phi = vm.AnglesForDirection(region.direction, 0.0)
    return vm.BasisForAngles(theta, phi, isZUp)


def TestProjection():
    basis = vm.IDENTITY_BASIS
    faces = {f.name: f for f in vm.ProjectCube(basis, False, 30.0,
                                               (55.0, 55.0))}
    front = faces["front"]
    _Check(front.visible, "the front face faces the camera")
    _Check(front.polygon == [(15.0, 15.0), (95.0, 15.0),
                             (95.0, 95.0), (15.0, 95.0)],
           "front is a centred 80px square, TL TR BR BL: %s"
           % (front.polygon,))
    _Check(not faces["back"].visible, "the back face is hidden")
    back = faces["back"].polygon
    width = max(p[0] for p in back) - min(p[0] for p in back)
    height = max(p[1] for p in back) - min(p[1] for p in back)
    _Check(_Close(width, 48.0, 1e-9) and _Close(height, 48.0, 1e-9),
           "back is a 48px square (R * 4/5): %s" % (back,))
    # A convex cube shows 1 face head-on, 2 on an edge, 3 on a corner.
    for name, region in sorted(vm.REGIONS.items()):
        want = len(region.faces)
        for isZUp in (False, True):
            viewBasis = _BasisForRegion(region, isZUp)
            seen = vm.VisibleFaces(vm.ProjectCube(viewBasis, isZUp,
                                                  26.4, (55.0, 55.0)))
            _Check(len(seen) == want,
                   "%s shows %d faces (isZUp=%s): %d"
                   % (name, want, isZUp, len(seen)))
    # The silhouette fits the widget on all 26 canonical views.
    limit = vm.WIDGET_SIZE / 2.0 - 2.0
    worst = 0.0
    for name, region in sorted(vm.REGIONS.items()):
        for isZUp in (False, True):
            radius = vm.RADIUS_FRACTION * vm.WIDGET_SIZE
            got = vm.SilhouetteRadius(_BasisForRegion(region, isZUp),
                                      isZUp, radius)
            worst = max(worst, got)
            _Check(got <= limit,
                   "%s silhouette %s fits in %s (isZUp=%s)"
                   % (name, got, limit, isZUp))
    _Check(worst <= 1.9085 * vm.RADIUS_FRACTION * vm.WIDGET_SIZE + 0.1,
           "worst silhouette is the corner view: %s" % worst)
    _Check(_Close(vm.SilhouetteRadius(vm.IDENTITY_BASIS, False, 30.0),
                     30.0 * 4.0 / 3.0 * math.sqrt(2.0), 1e-9),
           "identity silhouette is the front corners at 40*sqrt(2)")


def TestHitTest():
    basis = vm.IDENTITY_BASIS
    args = (basis, False, 30.0, (55.0, 55.0))
    _Check(vm.HitTest(*args, (55.0, 55.0)).name == "front",
           "the centre is the front face")
    _Check(vm.HitTest(*args, (85.0, 55.0)).name == "front-right",
           "u=0.75 is the right edge band")
    _Check(vm.HitTest(*args, (85.0, 25.0)).name == "front-top-right",
           "the corner square is the corner")
    _Check(vm.HitTest(*args, (32.5, 77.5)).name == "front-bottom-left",
           "the opposite corner square is the opposite corner")
    _Check(vm.HitTest(*args, (500.0, 500.0)) is None, "a miss is None")
    scale = 30.0 * vm.EYE_DISTANCE / (vm.EYE_DISTANCE - 1.0)
    inner = 55.0 + scale * vm.ZONE_LIMIT
    _Check(vm.HitTest(*args, (inner - 0.4, 55.0)).name == "front",
           "just inside ZONE_LIMIT is still the face")
    _Check(vm.HitTest(*args, (inner + 0.4, 55.0)).name == "front-right",
           "just beyond ZONE_LIMIT is the edge band")
    band = vm.RegionPolygons(vm.REGIONS["front-right"], *args)[0]
    _Check(_Close(min(p[0] for p in band), inner, 1e-9),
           "the band's inner edge sits at the ZONE_LIMIT boundary")
    # Every region is reachable through its own highlight: the point the
    # module reports for a region hit-tests back to that region, on
    # every canonical view, in both up modes.
    for viewName, view in sorted(vm.REGIONS.items()):
        for isZUp in (False, True):
            viewBasis = _BasisForRegion(view, isZUp)
            visible = {f.name for f in vm.VisibleFaces(vm.ProjectCube(
                viewBasis, isZUp, 26.4, (55.0, 55.0)))}
            for name, region in sorted(vm.REGIONS.items()):
                point = vm.RegionPoint(region, viewBasis, isZUp, 26.4,
                                       (55.0, 55.0))
                touches = bool(set(region.faces) & visible)
                if not touches:
                    _Check(point is None,
                           "%s touches no visible face from %s "
                           "(isZUp=%s) yet reports %s"
                           % (name, viewName, isZUp, point))
                    continue
                _Check(point is not None,
                       "%s is on a visible face from %s (isZUp=%s)"
                       % (name, viewName, isZUp))
                hit = vm.HitTest(viewBasis, isZUp, 26.4, (55.0, 55.0),
                                 point)
                _Check(hit is not None and hit.name == name,
                       "%s hit-tests back to itself from %s "
                       "(isZUp=%s): %s"
                       % (name, viewName, isZUp, hit))
    _Check(vm.RegionPoint(vm.REGIONS["back"], *args) is None,
           "the hidden back face has no point from the front view")
    # The canonical views are all mirror-symmetric, so nothing above
    # pins RegionPoint's "largest polygon" rule; a skewed view breaks
    # the symmetry (front band ~243.5 px^2, right band ~656.9 px^2).
    skew = (vm.BasisForAngles(-60.0, 0.0, False), False, 26.4,
            (55.0, 55.0))
    bands = vm.RegionPolygons(vm.REGIONS["front-right"], *skew)
    _Check(len(bands) == 2,
           "front-right shows both faces from the skewed view")
    _Check(_PolygonArea(bands[1]) > _PolygonArea(bands[0]),
           "the right band is larger than the foreshortened "
           "front band")
    want = (sum(p[0] for p in bands[1]) / 4.0,
            sum(p[1] for p in bands[1]) / 4.0)
    got = vm.RegionPoint(vm.REGIONS["front-right"], *skew)
    _Check(_Close(got[0], want[0], 1e-9)
           and _Close(got[1], want[1], 1e-9),
           "RegionPoint is the centroid of the LARGEST band: "
           "%s vs %s" % (got, want))
    _Check(vm.HitTest(*skew, got).name == "front-right",
           "and it hit-tests to front-right")
    # On the front plane the projective scale is uniform, so the area
    # ratios are exact: an edge band is half the middle square and a
    # corner square a quarter (never the corner squares for an edge).
    middle = _PolygonArea(vm.RegionPolygons(vm.REGIONS["front"],
                                            *args)[0])
    edge = vm.RegionPolygons(vm.REGIONS["front-right"], *args)
    _Check(len(edge) == 1, "one visible face for the edge: %d" % len(edge))
    _Check(_Close(_PolygonArea(edge[0]), 0.5 * middle, 1e-9),
           "edge band is half the middle square")
    corner = vm.RegionPolygons(vm.REGIONS["front-top-right"], *args)
    _Check(len(corner) == 1, "one visible face for the corner")
    _Check(_Close(_PolygonArea(corner[0]), 0.25 * middle, 1e-9),
           "corner square is a quarter of the middle square")


def TestFreeCamera():
    # Deferred: freeCamera imports PySide6, which the earlier groups
    # do not need.
    from pxr.Usdviewq.freeCamera import FreeCamera
    box = Gf.BBox3d(Gf.Range3d(Gf.Vec3d(-1), Gf.Vec3d(1)))
    for isZUp in (False, True):
        toWorld = vm.UpSpaceToWorld(isZUp)
        for name, region in sorted(vm.REGIONS.items()):
            theta, phi = vm.AnglesForDirection(region.direction, 0.0)
            cam = FreeCamera(isZUp)
            cam.rotTheta = theta
            cam.rotPhi = phi
            cam.center = Gf.Vec3d(0)
            cam.dist = 10.0
            frustum = cam.computeGfCamera(box).frustum
            want = toWorld.TransformDir(region.direction)
            _Check(_VecClose(frustum.position / 10.0, want, 1e-6),
                   "%s sits at its world direction (isZUp=%s): %s"
                   % (name, isZUp, frustum.position))
            _Check(_VecClose(frustum.ComputeViewDirection(), -want,
                             1e-6),
                   "%s looks back at the centre (isZUp=%s)"
                   % (name, isZUp))
            basis = vm.BasisFromFrustum(frustum)
            expect = vm.BasisForAngles(theta, phi, isZUp)
            _Check(_VecClose(basis.right, expect.right, 1e-6)
                   and _VecClose(basis.up, expect.up, 1e-6)
                   and _VecClose(basis.view, expect.view, 1e-6),
                   "%s basis matches FreeCamera (isZUp=%s)"
                   % (name, isZUp))
    for isZUp in (False, True):
        toWorld = vm.UpSpaceToWorld(isZUp)
        for face in vm.FACES:
            region = vm.REGIONS[face]
            theta, phi = vm.AnglesForDirection(region.direction, 0.0)
            cam = FreeCamera(isZUp)
            cam.rotTheta = theta
            cam.rotPhi = phi
            cam.center = Gf.Vec3d(0)
            cam.dist = 10.0
            frustum = cam.computeGfCamera(box).frustum
            wantUp = toWorld.TransformDir(vm.FACE_AXES[face][1])
            _Check(_VecClose(frustum.ComputeUpVector(), wantUp, 1e-6),
                   "%s screen-up is its label up (isZUp=%s): %s"
                   % (face, isZUp, frustum.ComputeUpVector()))


def main():
    groups = [
        ("regions", TestRegions),
        ("angles", TestAngles),
        ("frames", TestFrames),
        ("projection", TestProjection),
        ("hit test", TestHitTest),
        ("freecamera", TestFreeCamera),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("VIEWCUBE_MATH_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
