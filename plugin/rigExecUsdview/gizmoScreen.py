#
# RigExec usdview gizmo: screen-space geometry. Pure functions over Gf so
# the handle layout, hit-testing and drag mapping are testable with a
# synthetic Gf.Camera and no Qt.
#
# Pixel space is the PHYSICAL pixel space of StageView.computePickFrustum
# (curvenetUI.SurfacePicker.Project documents the mapping); callers
# convert Qt's logical coordinates with devicePixelRatioF() and back.
#
import math

from pxr import Gf

TOOL_TRANSLATE = "translate"
TOOL_ROTATE = "rotate"
TOOL_SCALE = "scale"

# Handle sizes in LOGICAL pixels (multiplied by the device pixel ratio).
GIZMO_PIXELS = 90.0
HIT_PIXELS = 8.0
CENTER_PIXELS = 6.0
RING_FRACTION = 0.85
RING_SEGMENTS = 48
SCALE_PIXELS_PER_UNIT = 120.0

# Maya manipulator geometry, as fractions of the manipulator size (design
# section 8.1-8.4).  PLANE_OFFSET places each planar handle 30% out along
# both of its axes; PLANE_SIDE is the square's side; CENTER_SIDE the
# view-plane / uniform-scale square; CUBE_SIDE the scale axis cubes;
# CONE_RADIUS the base radius of the move arrowheads.  The view-axis ring
# is drawn outside the axis rings so it can be grabbed on its own.
PLANE_OFFSET = 0.30
PLANE_SIDE = 0.15
CENTER_SIDE = 0.12
CUBE_SIDE = 0.08
CONE_RADIUS = 0.05
VIEW_RING_FRACTION = 1.25

# Maya's manipulator palette.  The axis colours are the flat primaries
# Maya uses, not softened pastels, so a screenshot matches Maya's.
COLOR_VIEW = (0.4, 0.75, 1.0)
COLOR_HOVER = (1.0, 0.85, 0.4)
COLOR_SELECTED = (1.0, 1.0, 0.0)
COLOR_SPHERE = (0.6, 0.6, 0.6)

# Prevent Negative Scale clamps to this rather than to zero: a zero scale
# is not invertible, so the artist could never drag back out of it.
MIN_SCALE_FACTOR = 1e-4

# A planar handle seen nearly edge-on collapses onto one of its axes and
# would steal that axis's picks, since planes outrank axes in HitTest.
# Below this fraction of its face-on area it stops being grabbable, the
# same bargain MIN_AXIS_PIXELS strikes for a foreshortened axis.
MIN_PLANE_AREA_FRACTION = 0.2

# Below this |dot(planeNormal, rayDirection)| the ray/plane intersection
# is numerically worthless (the plane is edge-on), so the drag falls back
# to sliding in the camera plane.
_MIN_PLANE_FACING = 0.05

# Shortest projected axis that may still be grabbed, in LOGICAL pixels.
# An axis pointing nearly at the camera has almost no screen direction, so
# every pixel of mouse travel becomes a huge world move: at 4 px a 100 px
# drag would push the object 11 world units with the camera 10 units away.
# Every other DCC gizmo answers this the same way -- the axis goes
# ungrabbable and the artist orbits a few degrees before dragging it.
MIN_AXIS_PIXELS = 12.0

_AXES = (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
_AXIS_NAMES = ("x", "y", "z")
_AXIS_COLORS = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

# Planar handles, as (name, normal axis, first axis, second axis).  Each
# is coloured like the axis PERPENDICULAR to its plane, so the yz square
# is red, xz green and xy blue (design section 8.2).
_PLANE_SPECS = (("yz", 0, 1, 2), ("xz", 1, 0, 2), ("xy", 2, 0, 1))

# HitTest resolves ties by this order, not by distance.  The centre wins
# because every axis starts there; planes beat axes because their squares
# sit on top of the axis lines; the free-rotate disc is last because it
# covers the whole manipulator and would otherwise swallow every ring.
_HIT_PRIORITY = ("center", "plane", "axis", "ring", "view", "sphere")


class Handle(object):
    """
    One drawable, pickable piece of the gizmo.

    `kind` is one of "axis", "plane", "ring", "view", "sphere" or
    "center".  `points` is that kind's projected geometry: two endpoints
    for an axis, four corners for a plane, RING_SEGMENTS points for a
    ring or the view ring, and a single point for the centre and for the
    free-rotate disc (whose size is `radiusPixels`).

    `worldCenter` is the world point the handle is centred on -- the
    square's centre for a plane, the gizmo origin for everything else --
    and `worldCenterScreen` is its projection.  A planar handle also
    carries `worldNormal`.

    Rings carry `frontPoints`, the runs of projected points on the
    camera side of the ring centre, and `frontWorld`, those points in
    world space.  Maya hides the back half of each ring so the three
    rings stay tellable apart; drawing and picking both use the front.

    `grabbable` is False for an axis that is too foreshortened to drag
    (see MIN_AXIS_PIXELS) or a plane too edge-on to aim at (see
    MIN_PLANE_AREA_FRACTION). Such a handle is still returned so it can
    be drawn dimmed -- the artist needs to see that the handle is there
    and why it will not respond -- but HitTest refuses to pick it.
    """

    def __init__(self, name, kind, axisIndex, points, worldAxis,
                 worldLength, color, center, grabbable=True,
                 worldCenter=None, worldCenterScreen=None, worldNormal=None,
                 frontPoints=None, frontWorld=None, visible=True,
                 radiusPixels=0.0, sizePixels=0.0):
        self.name = name
        self.kind = kind
        self.axisIndex = axisIndex
        self.points = points
        self.worldAxis = worldAxis
        self.worldLength = worldLength
        self.color = color
        self.center = center
        self.grabbable = grabbable
        self.worldCenter = worldCenter
        self.worldCenterScreen = worldCenterScreen
        self.worldNormal = worldNormal
        self.frontPoints = frontPoints if frontPoints is not None else []
        self.frontWorld = frontWorld if frontWorld is not None else []
        self.visible = visible
        self.radiusPixels = radiusPixels
        self.sizePixels = sizePixels

    def __repr__(self):
        return "<Handle %s %s%s>" % (
            self.name, self.kind, "" if self.grabbable else " (locked)")


def ViewProjection(camera):
    frustum = camera.frustum
    return frustum.ComputeViewMatrix() * frustum.ComputeProjectionMatrix()


def ProjectPoint(viewProj, viewport, p):
    """World -> physical pixels; None behind the eye."""
    clip = Gf.Vec4d(p[0], p[1], p[2], 1.0) * viewProj
    if clip[3] <= 1e-9:
        return None
    ndcX, ndcY = clip[0] / clip[3], clip[1] / clip[3]
    return ((ndcX + 1.0) * 0.5 * viewport[2] + viewport[0],
            (1.0 - ndcY) * 0.5 * viewport[3] + viewport[1])


def CameraBasis(camera):
    frustum = camera.frustum
    viewDir = Gf.Vec3d(frustum.ComputeViewDirection()).GetNormalized()
    up = Gf.Vec3d(frustum.ComputeUpVector()).GetNormalized()
    right = Gf.Cross(viewDir, up).GetNormalized()
    return viewDir, up, right


def WorldPerPixel(camera, viewport, worldPoint):
    """World units per physical pixel in the camera plane at worldPoint."""
    _, _, right = CameraBasis(camera)
    vp = ViewProjection(camera)
    a = ProjectPoint(vp, viewport, worldPoint)
    b = ProjectPoint(vp, viewport, Gf.Vec3d(worldPoint) + right)
    if a is None or b is None:
        return None
    pixels = math.hypot(b[0] - a[0], b[1] - a[1])
    if pixels < 1e-9:
        return None
    return 1.0 / pixels


def _PolygonArea(points):
    """Unsigned area of a projected polygon, in square pixels."""
    total = 0.0
    for i in range(len(points)):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % len(points)]
        total += x0 * y1 - x1 * y0
    return abs(total) * 0.5


def _RingBasis(axis, fallback):
    """
    An orthonormal pair spanning the plane perpendicular to `axis`.

    Gram-Schmidt against `fallback` rather than just taking two of the
    frame's other axes: the Gimbal orientation hands us three axes that
    are NOT mutually perpendicular, and a ring must still be a circle in
    its own axis's plane. For an orthonormal frame this returns exactly
    the other two axes, so the ordinary case is unchanged.
    """
    u = Gf.Vec3d(fallback) - Gf.Vec3d(axis) * Gf.Dot(fallback, axis)
    if u.GetLength() < 1e-9:
        for candidate in _AXES:
            u = Gf.Vec3d(candidate) - Gf.Vec3d(axis) * Gf.Dot(candidate, axis)
            if u.GetLength() >= 1e-9:
                break
    u = u.GetNormalized()
    return u, Gf.Cross(axis, u).GetNormalized()


def _FrontRuns(mask):
    """
    Maximal cyclic runs of True in `mask`, as lists of indices.

    A ring that faces the camera is one run of every point, not two runs
    split at index 0, so the caller can draw it as a single polyline.
    """
    count = len(mask)
    if not any(mask):
        return []
    if all(mask):
        return [list(range(count))]
    start = next(i for i in range(count) if mask[i] and not mask[i - 1])
    runs, run = [], []
    for step in range(count):
        index = (start + step) % count
        if mask[index]:
            run.append(index)
        elif run:
            runs.append(run)
            run = []
    if run:
        runs.append(run)
    return runs


def _ProjectRing(vp, viewport, origin, u, v, radius):
    """Projected and world points of a RING_SEGMENTS circle, or None."""
    screen, world = [], []
    for step in range(RING_SEGMENTS):
        theta = 2.0 * math.pi * step / RING_SEGMENTS
        point = origin + (u * math.cos(theta) + v * math.sin(theta)) * radius
        projected = ProjectPoint(vp, viewport, point)
        if projected is None:
            return None, None
        screen.append(projected)
        world.append(point)
    return screen, world


def BuildHandles(tool, gizmoMatrix, camera, viewport, pixelRatio,
                 sizePixels=GIZMO_PIXELS, orientation=None, gimbalAxes=None,
                 freeRotate=True):
    """
    Maya's manipulator for `tool`, laid out in screen space.

    `gizmoMatrix` places the manipulator; `orientation` (a Gf.Matrix4d
    whose rows are the world axes to draw along) overrides the direction
    of the handles without moving them, which is what the Axis
    Orientation option switches between World, Object and Parent.
    `gimbalAxes` replaces the three rotate ring axes for Gimbal mode.

    Returns [] when the origin is behind the eye or the frame is
    degenerate.
    """
    origin = Gf.Vec3d(gizmoMatrix.ExtractTranslation())
    wpp = WorldPerPixel(camera, viewport, origin)
    if wpp is None:
        return []
    vp = ViewProjection(camera)
    center = ProjectPoint(vp, viewport, origin)
    if center is None:
        return []
    pixels = sizePixels * pixelRatio
    length = pixels * wpp
    frame = gizmoMatrix if orientation is None else orientation
    axes = []
    for i in range(3):
        axis = Gf.Vec3d(frame.TransformDir(_AXES[i]))
        if axis.GetLength() < 1e-12:
            return []
        axes.append(axis.GetNormalized())

    def _Make(name, kind, axisIndex, points, worldAxis, worldLength, color,
              **extra):
        extra.setdefault("worldCenter", origin)
        extra.setdefault("worldCenterScreen", center)
        extra.setdefault("sizePixels", pixels)
        return Handle(name, kind, axisIndex, points, worldAxis, worldLength,
                      color, center, **extra)

    handles = []
    if tool in (TOOL_TRANSLATE, TOOL_SCALE):
        for i in range(3):
            end = ProjectPoint(vp, viewport, origin + axes[i] * length)
            if end is None:
                continue
            screen = math.hypot(end[0] - center[0], end[1] - center[1])
            handles.append(_Make(
                _AXIS_NAMES[i], "axis", i, [center, end], axes[i], length,
                _AXIS_COLORS[i],
                grabbable=screen >= MIN_AXIS_PIXELS * pixelRatio))
        half = PLANE_SIDE * 0.5
        for name, normalIndex, first, second in _PLANE_SPECS:
            a, b = axes[first], axes[second]
            corners = []
            for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                point = origin + (a * ((PLANE_OFFSET + sa * half) * length)
                                  + b * ((PLANE_OFFSET + sb * half) * length))
                projected = ProjectPoint(vp, viewport, point)
                if projected is None:
                    corners = None
                    break
                corners.append(projected)
            if corners is None:
                continue
            squareCenter = origin + (a + b) * (PLANE_OFFSET * length)
            squareScreen = ProjectPoint(vp, viewport, squareCenter)
            if squareScreen is None:
                continue
            faceOn = (PLANE_SIDE * pixels) ** 2
            handles.append(_Make(
                name, "plane", normalIndex, corners, None, PLANE_SIDE * length,
                _AXIS_COLORS[normalIndex],
                worldNormal=axes[normalIndex], worldCenter=squareCenter,
                worldCenterScreen=squareScreen,
                grabbable=_PolygonArea(corners)
                >= faceOn * MIN_PLANE_AREA_FRACTION))
        handles.append(_Make(
            "center", "center", None, [center], None, length, COLOR_VIEW))
    elif tool == TOOL_ROTATE:
        radius = length * RING_FRACTION
        viewDir, up, right = CameraBasis(camera)
        toCamera = -viewDir
        ringAxes = list(gimbalAxes) if gimbalAxes else axes
        for i in range(3):
            axis = Gf.Vec3d(ringAxes[i]).GetNormalized()
            u, v = _RingBasis(axis, ringAxes[(i + 1) % 3])
            screen, world = _ProjectRing(vp, viewport, origin, u, v, radius)
            if screen is None:
                continue
            # Maya hides the half of each ring that is behind the ring
            # centre, so three overlapping circles stay readable.
            mask = [Gf.Dot(p - origin, toCamera) >= -1e-9 for p in world]
            runs = _FrontRuns(mask)
            handles.append(_Make(
                _AXIS_NAMES[i], "ring", i, screen, axis, radius,
                _AXIS_COLORS[i],
                frontPoints=[[screen[k] for k in run] for run in runs],
                frontWorld=[world[k] for run in runs for k in run],
                visible=bool(runs)))
        viewRadius = radius * VIEW_RING_FRACTION
        screen, world = _ProjectRing(vp, viewport, origin, right, up,
                                     viewRadius)
        if screen is not None:
            handles.append(_Make(
                "view", "view", None, screen, toCamera, viewRadius,
                COLOR_VIEW, frontPoints=[list(screen)], frontWorld=world))
        if freeRotate:
            handles.append(_Make(
                "free", "sphere", None, [center], toCamera, radius,
                COLOR_SPHERE, radiusPixels=pixels * RING_FRACTION))
    return handles


def _PointSegmentDistance(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _PointInPolygon(p, points):
    """Even-odd containment test for a projected convex-ish polygon."""
    inside = False
    count = len(points)
    for i in range(count):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % count]
        if (y0 > p[1]) != (y1 > p[1]):
            crossing = x0 + (p[1] - y0) / (y1 - y0) * (x1 - x0)
            if p[0] < crossing:
                inside = not inside
    return inside


def _PolylineDistance(p, points, closed):
    """Nearest distance from p to a polyline, or None when it is empty."""
    if not points:
        return None
    if len(points) == 1:
        return math.hypot(p[0] - points[0][0], p[1] - points[0][1])
    last = len(points) if closed else len(points) - 1
    return min(_PointSegmentDistance(p, points[i],
                                     points[(i + 1) % len(points)])
               for i in range(last))


def _HandleDistance(handle, p):
    """Screen distance from p to the handle, or None when unpickable."""
    kind = handle.kind
    if kind in ("center", "sphere"):
        return math.hypot(p[0] - handle.points[0][0],
                          p[1] - handle.points[0][1])
    if kind == "axis":
        return _PointSegmentDistance(p, handle.points[0], handle.points[1])
    if kind == "plane":
        if _PointInPolygon(p, handle.points):
            return 0.0
        return _PolylineDistance(p, handle.points, True)
    if kind == "ring":
        # Only the front half is drawn, so only the front half is aimed
        # at. Each run is an open polyline: closing it would add a chord
        # straight across the manipulator.
        distances = [_PolylineDistance(p, run, False)
                     for run in handle.frontPoints]
        distances = [d for d in distances if d is not None]
        return min(distances) if distances else None
    return _PolylineDistance(p, handle.points, True)


def HitTest(handles, x, y, radius):
    """
    The handle under (x, y), or None.

    Resolution is by kind first (_HIT_PRIORITY) and only then by
    distance, because the handles deliberately overlap: every axis starts
    at the centre, the planar squares sit on the axis lines, and the
    free-rotate disc covers all three rings. Picking the nearest edge
    outright would make the centre and the planes almost unreachable.

    `radius` is the pick tolerance in physical pixels for everything with
    an outline; the free-rotate disc instead claims its whole interior,
    which is how Maya's works.

    Handles marked not grabbable are skipped, so a foreshortened axis or
    an edge-on plane cannot be picked by accident -- it has collapsed
    onto something else the artist is more likely to have meant.
    """
    p = (x, y)
    pickable = [h for h in handles if h.grabbable]
    for kind in _HIT_PRIORITY:
        best = None
        for handle in pickable:
            if handle.kind != kind:
                continue
            distance = _HandleDistance(handle, p)
            if distance is None:
                continue
            limit = handle.radiusPixels if kind == "sphere" else radius
            if distance <= limit and (best is None or distance < best[0]):
                best = (distance, handle)
        if best is not None:
            return best[1]
    return None


def AxisDragParameter(handle, press, current):
    """
    Mouse travel projected onto the handle's screen direction, as a
    fraction of the handle's screen length (so 1.0 == one handle length
    == handle.worldLength in world units). The screen length is floored at
    MIN_AXIS_PIXELS so a few pixels cannot become a huge move. That is a
    safety net only: BuildHandles already marks such an axis ungrabbable,
    so a drag should never start on one.
    """
    ax, ay = handle.points[0]
    bx, by = handle.points[-1]
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < MIN_AXIS_PIXELS:
        if length < 1e-9:
            return 0.0
        dx = dx / length * MIN_AXIS_PIXELS
        dy = dy / length * MIN_AXIS_PIXELS
        length = MIN_AXIS_PIXELS
    ux, uy = dx / length, dy / length
    travel = (current[0] - press[0]) * ux + (current[1] - press[1]) * uy
    return travel / length


def PlaneDragDelta(camera, viewport, worldOrigin, press, current):
    """World delta for a drag in the camera plane through worldOrigin."""
    wpp = WorldPerPixel(camera, viewport, worldOrigin)
    if wpp is None:
        return Gf.Vec3d(0, 0, 0)
    _, up, right = CameraBasis(camera)
    dx = current[0] - press[0]
    dy = current[1] - press[1]
    return right * (dx * wpp) - up * (dy * wpp)


def AxisFacesCamera(camera, worldAxis):
    viewDir, _, _ = CameraBasis(camera)
    return Gf.Dot(Gf.Vec3d(worldAxis), viewDir) < 0.0


def RotationDragAngle(center, press, current, axisFacesCamera):
    """
    Degrees swept around `center` from press to current, positive for
    counter-clockwise ON SCREEN when the rotation axis points at the
    camera (right-hand rule seen from the axis tip), wrapped to
    (-180, 180]. Screen y grows downward, hence the negation.
    """
    a0 = math.atan2(-(press[1] - center[1]), press[0] - center[0])
    a1 = math.atan2(-(current[1] - center[1]), current[0] - center[0])
    degrees = math.degrees(a1 - a0)
    while degrees > 180.0:
        degrees -= 360.0
    while degrees <= -180.0:
        degrees += 360.0
    return degrees if axisFacesCamera else -degrees


def ScaleDragFactor(handle, press, current, pixelRatio):
    """
    1 + travel / SCALE_PIXELS_PER_UNIT, floored at 0.01.

    Returns 1.0 unchanged for an axis too foreshortened to grab: it has no
    usable screen direction, so any travel along it would be noise
    amplified into a huge scale.
    """
    if handle.kind == "center":
        travel = current[0] - press[0]
    else:
        ax, ay = handle.points[0]
        bx, by = handle.points[-1]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if handle.kind == "axis" and length < MIN_AXIS_PIXELS * pixelRatio:
            return 1.0
        if length < 1e-9:
            travel = current[0] - press[0]
        else:
            travel = ((current[0] - press[0]) * dx
                      + (current[1] - press[1]) * dy) / length
    return max(0.01, 1.0 + travel / (SCALE_PIXELS_PER_UNIT * pixelRatio))


def _PickRay(camera, viewport, point):
    """
    A world ray through a physical-pixel point.

    The normalisation is the one StageView.computePickFrustum uses, so a
    gizmo drag and a usdview pick agree about where the cursor is. Gf's
    Python bindings expose ComputePickRay but not ComputeRay; both lie
    along the same line through the eye, and a plane intersection only
    cares about the line, so the near-plane origin costs nothing here.
    """
    ndcX = (point[0] - viewport[0]) / float(viewport[2]) * 2.0 - 1.0
    ndcY = -((point[1] - viewport[1]) / float(viewport[3]) * 2.0 - 1.0)
    return camera.frustum.ComputePickRay(Gf.Vec2d(ndcX, ndcY))


def _RayPlanePoint(ray, plane):
    """Where `ray` meets `plane`, or None when they do not meet."""
    hit = ray.Intersect(plane)
    if not hit[0]:
        return None
    return Gf.Vec3d(ray.GetPoint(hit[1]))


def RayPlaneDragDelta(camera, viewport, worldOrigin, worldNormal, press,
                      current):
    """
    World delta for a drag in the plane through worldOrigin.

    Ray/plane intersection rather than scaled screen travel, so the point
    the artist grabbed stays under the cursor as the plane recedes --
    Maya's planar handles behave this way and a plain screen mapping
    visibly slides away from the cursor in a perspective view.

    Falls back to the camera-plane mapping when the plane is edge-on:
    the intersection is then either at infinity or wildly unstable.
    """
    normal = Gf.Vec3d(worldNormal)
    if normal.GetLength() < 1e-12:
        return PlaneDragDelta(camera, viewport, worldOrigin, press, current)
    normal = normal.GetNormalized()
    pressRay = _PickRay(camera, viewport, press)
    if abs(Gf.Dot(normal, Gf.Vec3d(pressRay.direction))) < _MIN_PLANE_FACING:
        return PlaneDragDelta(camera, viewport, worldOrigin, press, current)
    plane = Gf.Plane(normal, Gf.Vec3d(worldOrigin))
    start = _RayPlanePoint(pressRay, plane)
    end = _RayPlanePoint(_PickRay(camera, viewport, current), plane)
    if start is None or end is None:
        return PlaneDragDelta(camera, viewport, worldOrigin, press, current)
    return end - start


def AccumulateAngle(total, previous, current):
    """
    `total` plus the shortest way round from `previous` to `current`.

    RotationDragAngle wraps into (-180, 180], but a rotate drag has to
    keep counting: Maya lets one sweep run to 400 degrees. Accumulating
    the wrapped step rather than the raw difference is what makes the
    crossing at 180 invisible.
    """
    delta = current - previous
    while delta > 180.0:
        delta -= 360.0
    while delta <= -180.0:
        delta += 360.0
    return total + delta


def TrackballRotation(camera, press, current, radiusPixels):
    """
    (axis, degrees) for a free-rotate drag, or None when nothing moved.

    A virtual trackball: the axis is perpendicular to the mouse travel in
    the camera plane, so the surface follows the cursor, and dragging one
    diameter sweeps a half turn.
    """
    dx = current[0] - press[0]
    dy = current[1] - press[1]
    travel = math.hypot(dx, dy)
    if travel < 1e-9 or radiusPixels <= 0.0:
        return None
    viewDir, up, right = CameraBasis(camera)
    motion = right * dx - up * dy          # screen y grows downward
    axis = Gf.Cross(motion, viewDir)
    if axis.GetLength() < 1e-12:
        return None
    return axis.GetNormalized(), travel / (2.0 * radiusPixels) * 180.0


def MayaScaleFactor(handle, origin2d, press, current, allowNegative):
    """
    Maya's scale ratio: how far the cursor is from the manipulator origin
    along the handle, over how far it was when the drag started.

    Dragging the handle onto the origin therefore gives 0 and carrying it
    through gives a mirrored negative, unless Prevent Negative Scale is
    on. The centre handle is uniform scale and uses horizontal travel
    against the manipulator size instead, since it has no direction.
    """
    if handle.kind == "center":
        size = handle.sizePixels if handle.sizePixels > 1e-9 else 1.0
        factor = 1.0 + (current[0] - press[0]) / size
    else:
        if handle.kind == "plane":
            dx = handle.worldCenterScreen[0] - origin2d[0]
            dy = handle.worldCenterScreen[1] - origin2d[1]
        else:
            dx = handle.points[-1][0] - handle.points[0][0]
            dy = handle.points[-1][1] - handle.points[0][1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return 1.0
        ux, uy = dx / length, dy / length
        at = ((press[0] - origin2d[0]) * ux + (press[1] - origin2d[1]) * uy)
        now = ((current[0] - origin2d[0]) * ux
               + (current[1] - origin2d[1]) * uy)
        if abs(at) < 1e-9:
            return 1.0
        factor = now / at
    if not allowNegative:
        factor = max(MIN_SCALE_FACTOR, factor)
    return factor


def _RoundToStep(value, step):
    """Nearest multiple of `step`, rounding halves away from zero."""
    if step <= 0.0:
        return value
    quotient = value / step
    if quotient >= 0.0:
        return math.floor(quotient + 0.5) * step
    return math.ceil(quotient - 0.5) * step


def SnapRelative(value, step):
    """
    Quantise a DELTA to a multiple of `step` (Maya's Discrete move /
    Snap rotate). Relative to the drag start, so an object that began off
    the grid stays off it and only moves in whole steps.

    Accepts a float or a Gf.Vec3d.
    """
    if isinstance(value, Gf.Vec3d):
        return Gf.Vec3d(*[_RoundToStep(v, step) for v in value])
    return _RoundToStep(value, step)


def SnapAbsolute(value, step):
    """
    Quantise a POSITION onto a grid of `step` (Maya's `X` hold). Same
    arithmetic as SnapRelative but a different meaning, and the two are
    separate names because a caller must not confuse a delta with a
    position: this one lands the object ON the grid.

    Accepts a float or a Gf.Vec3d.
    """
    if isinstance(value, Gf.Vec3d):
        return Gf.Vec3d(*[_RoundToStep(v, step) for v in value])
    return _RoundToStep(value, step)


def _IndexIsCounterClockwise(points):
    """
    True when walking `points` by increasing index turns the way the
    artist reads as counter-clockwise. Screen y grows downward, so the
    shoelace sign is measured on y-flipped points.
    """
    total = 0.0
    for i in range(len(points)):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % len(points)]
        total += x1 * y0 - x0 * y1
    return total >= 0.0


def RingParameter(handle, point2d):
    """The ring parameter, in radians, of the ring point nearest point2d."""
    points = handle.points
    if not points:
        return 0.0
    best, bestIndex = None, 0
    for index, p in enumerate(points):
        distance = math.hypot(point2d[0] - p[0], point2d[1] - p[1])
        if best is None or distance < best:
            best, bestIndex = distance, index
    return 2.0 * math.pi * bestIndex / len(points)


def PiePolygon(handle, startParameter, sweepDegrees):
    """
    Maya's rotation-amount wedge: the manipulator centre followed by the
    arc from `startParameter` through `sweepDegrees`.

    The walk direction comes from the ring's own projected winding, so
    the wedge follows the cursor whichever way the ring happens to face.
    """
    points = handle.points
    count = len(points)
    if count == 0:
        return [handle.center]
    start = int(round(startParameter / (2.0 * math.pi) * count)) % count
    steps = int(round(abs(sweepDegrees) / 360.0 * count))
    forward = _IndexIsCounterClockwise(points)
    direction = 1 if (sweepDegrees >= 0.0) == forward else -1
    wedge = [handle.center]
    for step in range(steps + 1):
        wedge.append(points[(start + direction * step) % count])
    return wedge
