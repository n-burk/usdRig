#
# RigExec usdview gizmo: Maya-style snap maths. Qt-free, so the
# world-grid rule, the handle constraint, screen-space ranking and
# the perspective-correct edge parameter are testable with a
# synthetic Gf.Camera and no stage.
#
# Layering: gizmoScreen owns the projector (ViewProjection,
# ProjectPoint) and the rounding rule (SnapAbsolute); this module
# owns every snap number built on them. gizmoDrag picks the active
# mode and turns a candidate into a plain world delta; gizmoUI
# supplies the hydra pick through an injected resolver so this file
# and gizmoDrag stay Qt-free. It must not import Qt and must not
# import gizmoUI, which would pull a whole Qt panel into the import
# graph.
#

import math

from pxr import Gf, Sdf

try:
    import gizmoScreen
    import gizmoSettings
except ImportError:                    # loader that did not add our dir
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoScreen
    import gizmoSettings

# One spelling: the tokens live in gizmoSettings beside ORIENT_* so
# Tasks 2-4 and the tests name the same strings.
SNAP_OFF = gizmoSettings.SNAP_OFF
SNAP_GRID = gizmoSettings.SNAP_GRID
SNAP_POINT = gizmoSettings.SNAP_POINT
SNAP_EDGE = gizmoSettings.SNAP_EDGE
SNAP_SURFACE = gizmoSettings.SNAP_SURFACE

# Screen radius for a point/edge candidate, in LOGICAL pixels; the
# Qt shell scales it by the device ratio at the call site the way
# gizmoUI._HitTest does (snapping design section 4.4).
SNAP_PIXELS = 12.0
# Pick-throttle travel, in PHYSICAL pixels: it is compared against
# DragState.press/.current, which gizmoUI._Position already scaled
# by devicePixelRatioF (gizmoUI.py:1636-1652), and this module is
# Qt-free so it cannot scale one itself.
PICK_MOVE_PIXELS = 3.0


class SnapCandidate(object):
    """Where the pivot should land, and what it landed on.

    `point` is world, `normal` is world (zero when the source has
    none), `kind` is one of the SNAP_* tokens, `primPath` is an
    Sdf.Path or None, `index` is the vertex/CV index (-1 when not
    applicable) and `screen` is the (x, y) PHYSICAL-pixel projection
    for the overlay marker.
    """

    def __init__(self, point, normal=None, kind=None, primPath=None,
                 index=-1, screen=None):
        self.point = Gf.Vec3d(point)
        if normal is None:
            self.normal = Gf.Vec3d(0, 0, 0)
        else:
            self.normal = Gf.Vec3d(normal)
        self.kind = kind if kind is not None else SNAP_OFF
        if primPath is None or isinstance(primPath, Sdf.Path):
            self.primPath = primPath
        else:
            self.primPath = Sdf.Path(str(primPath))
        self.index = int(index)
        self.screen = screen

    def __repr__(self):
        return "<SnapCandidate %s %s>" % (self.kind, self.point)


def ConstrainToHandle(handle, pivot0, worldPoint, ctrl=False):
    """Constrain a world target to what the handle may move along.

    Axis (no Ctrl) slides along its line, a plane -- or an axis with
    Ctrl, which TranslateDelta also treats as a plane
    (gizmoDrag.py:186-188) -- drops the normal component, and the
    centre (or anything else) lands unchanged (spec 4.2).
    """
    kind = getattr(handle, "kind", None)
    origin = Gf.Vec3d(pivot0)
    target = Gf.Vec3d(worldPoint)
    if kind == "axis" and not ctrl:
        axis = getattr(handle, "worldAxis", None)
        if axis is None:
            return target
        direction = Gf.Vec3d(axis)
        if direction.GetLength() < 1e-12:
            return target
        direction = direction.GetNormalized()
        return origin + direction * Gf.Dot(target - origin,
                                           direction)
    if kind == "plane" or (kind == "axis" and ctrl):
        if kind == "plane":
            raw = getattr(handle, "worldNormal", None)
        else:
            raw = getattr(handle, "worldAxis", None)
        if raw is None:
            return target
        normal = Gf.Vec3d(raw)
        if normal.GetLength() < 1e-12:
            return target
        normal = normal.GetNormalized()
        return target - normal * Gf.Dot(target - origin, normal)
    return target


def DirectionIsWorldAligned(direction, tol=1e-6):
    """Index of the world axis `direction` lies along, or None.

    The sign is ignored. GridPoint calls it on the axis for an axis
    handle and on the plane normal otherwise; None means there is no
    grid coordinate in that frame, so the caller degrades to
    quantising the travel from the pivot (spec 4.3).
    """
    axis = Gf.Vec3d(direction)
    if axis.GetLength() < 1e-12:
        return None
    unit = axis.GetNormalized()
    for index, candidate in enumerate((Gf.Vec3d(1, 0, 0),
                                      Gf.Vec3d(0, 1, 0),
                                      Gf.Vec3d(0, 0, 1))):
        # Within tol of +axis or -axis alike: a -X handle still
        # snaps the world X coordinate.
        if ((unit - candidate).GetLength() <= tol
                or (unit + candidate).GetLength() <= tol):
            return index
    return None


def _PlaneBasis(normal):
    """Deterministic orthonormal (u, v) spanning the plane (spec 4.3).

    e is the world axis least parallel to n (ties X before Y before
    Z), u is e with the normal removed, v completes the frame.
    """
    axes = (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0),
            Gf.Vec3d(0, 0, 1))
    dots = [abs(Gf.Dot(axis, normal)) for axis in axes]
    # min keeps the first minimal entry, which is the X-before-Y
    # -before-Z tie-break the spec asks for.
    pick = min(range(3), key=lambda i: dots[i])
    along = axes[pick] - normal * Gf.Dot(axes[pick], normal)
    if along.GetLength() < 1e-12:
        return None, None
    unit = along.GetNormalized()
    return unit, Gf.Cross(normal, unit).GetNormalized()


def GridPoint(handle, pivot0, unsnapped, gridSize, ctrl=False):
    """Snap the unsnapped pivot to the world grid (spec 4.3).

    Only the components the handle may change move: an axis rounds
    along itself, a plane (or Ctrl+axis) rounds the two in-plane
    world components, the centre rounds all three, halves away from
    zero through gizmoScreen.SnapAbsolute (gizmoScreen.py:673-685).
    A handle pointing nowhere near a world axis has no grid
    coordinate in its frame, so it quantises the travel from the
    pivot on the same directions instead.
    """
    if gridSize is None or gridSize <= 0.0:
        return Gf.Vec3d(unsnapped)
    origin = Gf.Vec3d(pivot0)
    position = Gf.Vec3d(unsnapped)
    kind = getattr(handle, "kind", None)
    if kind == "axis" and not ctrl:
        raw = getattr(handle, "worldAxis", None)
        if raw is None or Gf.Vec3d(raw).GetLength() < 1e-12:
            return gizmoScreen.SnapAbsolute(position, gridSize)
        axis = Gf.Vec3d(raw).GetNormalized()
        if DirectionIsWorldAligned(axis) is not None:
            along = Gf.Dot(position, axis)
            snapped = gizmoScreen.SnapAbsolute(along, gridSize)
            return position + axis * (snapped - along)
        travel = Gf.Dot(position - origin, axis)
        return origin + axis * gizmoScreen.SnapAbsolute(travel,
                                                       gridSize)
    if kind == "plane" or (kind == "axis" and ctrl):
        if kind == "plane":
            raw = getattr(handle, "worldNormal", None)
        else:
            raw = getattr(handle, "worldAxis", None)
        if raw is None or Gf.Vec3d(raw).GetLength() < 1e-12:
            return gizmoScreen.SnapAbsolute(position, gridSize)
        normal = Gf.Vec3d(raw).GetNormalized()
        facing = DirectionIsWorldAligned(normal)
        if facing is not None:
            kept = [position[0], position[1], position[2]]
            for index in range(3):
                if index != facing:
                    kept[index] = gizmoScreen.SnapAbsolute(
                        kept[index], gridSize)
            return Gf.Vec3d(kept[0], kept[1], kept[2])
        across, other = _PlaneBasis(normal)
        if across is None:
            return gizmoScreen.SnapAbsolute(position, gridSize)
        travel = position - origin
        return origin + across * gizmoScreen.SnapAbsolute(
            Gf.Dot(travel, across), gridSize) \
            + other * gizmoScreen.SnapAbsolute(
                Gf.Dot(travel, other), gridSize)
    return gizmoScreen.SnapAbsolute(position, gridSize)


def NearestPoint(points, cursor, viewProjection, viewport, radius):
    """Nearest world point to the cursor, in screen pixels.

    Each point is projected with gizmoScreen.ProjectPoint
    (gizmoScreen.py:144-151) -- the projector that already agrees
    with the pick frustum to 1.5 px -- points behind the eye are
    dropped, and the winner is the smallest screen distance within
    `radius` PHYSICAL pixels, or None (spec 4.4).
    """
    best = None
    for index, world in enumerate(points):
        screen = gizmoScreen.ProjectPoint(viewProjection, viewport,
                                          world)
        if screen is None:
            continue
        distance = math.hypot(cursor[0] - screen[0],
                              cursor[1] - screen[1])
        if distance <= radius and (best is None
                                   or distance < best[0]):
            best = (distance, index, Gf.Vec3d(world), screen)
    if best is None:
        return None
    return (best[1], best[2], best[3])


def SegmentScreenParameter(a2d, b2d, cursor):
    """(distance, t) of the cursor foot on the projected segment.

    t is the clamped screen-space foot; the world parameter comes
    from WorldParameterFromScreen. Reuses gizmoScreen's helper
    (gizmoScreen.py:398-407) rather than a second copy.
    """
    return gizmoScreen._PointSegmentDistance(cursor, a2d, b2d)


def WorldParameterFromScreen(t, wa, wb):
    """Perspective-correct world parameter for screen foot t.

    s = t*wa / ((1-t)*wb + t*wa) with wa/wb the endpoints'
    clip-space w (spec section 3). Returns t when either w is not
    positive; the caller must have dropped such segments already.
    """
    if wa <= 0.0 or wb <= 0.0:
        return t
    denom = (1.0 - t) * wb + t * wa
    if abs(denom) < 1e-12:
        return t
    return t * wa / denom


def NearestSegment(segments, cursor, viewProjection, viewport,
                   radius):
    """Nearest world point on any segment to the cursor (spec 4.4).

    `segments` are (worldA, worldB) pairs. Any endpoint with w <= 0
    has no valid screen segment and drops the whole segment, never
    clipped (spec section 3). Returns (segmentIndex, worldPoint,
    screenFoot) at the perspective-corrected parameter, or None.
    """
    best = None
    for index, ends in enumerate(segments):
        worldA, worldB = ends
        screenA, wA = gizmoScreen.ProjectPointWithW(
            viewProjection, viewport, worldA)
        screenB, wB = gizmoScreen.ProjectPointWithW(
            viewProjection, viewport, worldB)
        if wA <= 0.0 or wB <= 0.0:
            continue
        if screenA is None or screenB is None:
            continue
        distance, param = SegmentScreenParameter(screenA, screenB,
                                                 cursor)
        if distance > radius:
            continue
        if best is not None and distance >= best[0]:
            continue
        along = WorldParameterFromScreen(param, wA, wB)
        pointA, pointB = Gf.Vec3d(worldA), Gf.Vec3d(worldB)
        world = pointA + (pointB - pointA) * along
        foot = (screenA[0] + param * (screenB[0] - screenA[0]),
                screenA[1] + param * (screenB[1] - screenA[1]))
        best = (distance, index, world, foot)
    if best is None:
        return None
    return (best[1], best[2], best[3])


def MeshEdges(faceVertexCounts, faceVertexIndices):
    """Unique undirected edges, each face a closed loop (spec 4.4).

    Shared edges appear once so a subdivided quad does not vote
    twice for its interior.
    """
    edges = []
    seen = set()
    offset = 0
    for count in faceVertexCounts:
        face = faceVertexIndices[offset:offset + count]
        offset += count
        for step in range(count):
            first = face[step]
            second = face[(step + 1) % count]
            key = (first, second) if first < second else \
                (second, first)
            if key not in seen:
                seen.add(key)
                edges.append(key)
    return edges


def CurveSegments(vertexCounts, pointCount):
    """Consecutive CV pairs within each curve (spec 4.4).

    Never joins two curves: the break between one curve's last CV
    and the next curve's first is not a segment. `pointCount` is
    accepted for the UsdGeom call-site shape and ignored beyond it.
    """
    _ = pointCount
    segments = []
    offset = 0
    for count in vertexCounts:
        for step in range(count - 1):
            segments.append((offset + step, offset + step + 1))
        offset += count
    return segments
