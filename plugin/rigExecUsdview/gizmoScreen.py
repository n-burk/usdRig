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
_MIN_AXIS_PIXELS = 4.0

_AXES = (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
_AXIS_NAMES = ("x", "y", "z")
_AXIS_COLORS = ((0.95, 0.25, 0.25), (0.35, 0.85, 0.3), (0.3, 0.5, 1.0))
_CENTER_COLOR = (0.95, 0.95, 0.95)


class Handle(object):
    def __init__(self, name, kind, axisIndex, points, worldAxis,
                 worldLength, color, center):
        self.name = name
        self.kind = kind
        self.axisIndex = axisIndex
        self.points = points
        self.worldAxis = worldAxis
        self.worldLength = worldLength
        self.color = color
        self.center = center

    def __repr__(self):
        return "<Handle %s %s>" % (self.name, self.kind)


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


def BuildHandles(tool, gizmoMatrix, camera, viewport, pixelRatio):
    """
    Screen-constant handles for `tool` at `gizmoMatrix` (world; its rows
    are the local axes). Empty when the origin is behind the eye or the
    frame is degenerate.
    """
    origin = Gf.Vec3d(gizmoMatrix.ExtractTranslation())
    wpp = WorldPerPixel(camera, viewport, origin)
    if wpp is None:
        return []
    vp = ViewProjection(camera)
    center = ProjectPoint(vp, viewport, origin)
    if center is None:
        return []
    length = GIZMO_PIXELS * pixelRatio * wpp
    axes = []
    for i in range(3):
        axis = Gf.Vec3d(gizmoMatrix.TransformDir(_AXES[i]))
        if axis.GetLength() < 1e-12:
            return []
        axes.append(axis.GetNormalized())

    handles = []
    if tool in (TOOL_TRANSLATE, TOOL_SCALE):
        for i in range(3):
            end = ProjectPoint(vp, viewport, origin + axes[i] * length)
            if end is None:
                continue
            handles.append(Handle(
                _AXIS_NAMES[i], "axis", i, [center, end], axes[i], length,
                _AXIS_COLORS[i], center))
        handles.append(Handle(
            "center", "center", None, [center], None, length,
            _CENTER_COLOR, center))
    elif tool == TOOL_ROTATE:
        radius = length * RING_FRACTION
        for i in range(3):
            u, v = axes[(i + 1) % 3], axes[(i + 2) % 3]
            points = []
            for s in range(RING_SEGMENTS):
                theta = 2.0 * math.pi * s / RING_SEGMENTS
                p = ProjectPoint(vp, viewport, origin + (
                    u * math.cos(theta) + v * math.sin(theta)) * radius)
                if p is None:
                    points = None
                    break
                points.append(p)
            if points is None:
                continue
            handles.append(Handle(
                _AXIS_NAMES[i], "ring", i, points, axes[i], radius,
                _AXIS_COLORS[i], center))
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


def _HandleDistance(handle, p):
    if handle.kind == "center":
        return math.hypot(p[0] - handle.points[0][0],
                          p[1] - handle.points[0][1])
    if handle.kind == "axis":
        return _PointSegmentDistance(p, handle.points[0], handle.points[1])
    points = handle.points
    return min(_PointSegmentDistance(p, points[i],
                                     points[(i + 1) % len(points)])
               for i in range(len(points)))


def HitTest(handles, x, y, radius):
    """
    The handle under (x, y) within `radius` pixels, or None. The centre
    handle wins when it is hit at all: every axis starts there, and the
    small square is the thing the artist aimed at.
    """
    p = (x, y)
    for handle in handles:
        if handle.kind == "center" and _HandleDistance(handle, p) <= radius:
            return handle
    best = None
    for handle in handles:
        d = _HandleDistance(handle, p)
        if d <= radius and (best is None or d < best[0]):
            best = (d, handle)
    return best[1] if best else None


def AxisDragParameter(handle, press, current):
    """
    Mouse travel projected onto the handle's screen direction, as a
    fraction of the handle's screen length (so 1.0 == one handle length
    == handle.worldLength in world units). A foreshortened axis has its
    screen length floored so a few pixels cannot become a huge move.
    """
    ax, ay = handle.points[0]
    bx, by = handle.points[-1]
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < _MIN_AXIS_PIXELS:
        if length < 1e-9:
            return 0.0
        dx = dx / length * _MIN_AXIS_PIXELS
        dy = dy / length * _MIN_AXIS_PIXELS
        length = _MIN_AXIS_PIXELS
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
    """1 + travel / SCALE_PIXELS_PER_UNIT, floored at 0.01."""
    if handle.kind == "center":
        travel = current[0] - press[0]
    else:
        ax, ay = handle.points[0]
        bx, by = handle.points[-1]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-9:
            travel = current[0] - press[0]
        else:
            travel = ((current[0] - press[0]) * dx
                      + (current[1] - press[1]) * dy) / length
    return max(0.01, 1.0 + travel / (SCALE_PIXELS_PER_UNIT * pixelRatio))
