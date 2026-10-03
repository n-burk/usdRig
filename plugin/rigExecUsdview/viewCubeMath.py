#
# RigExec usdview view cube: orientation math. Pure functions over Gf for
# the 26 cube regions, the free-camera angle convention, the cube
# projection and the ray-cast hit test, so the whole geometry is testable
# headlessly with a synthetic camera basis and no Qt.
#
# This module is Qt-free on purpose (like gizmoScreen.py): viewCubeUI.py
# owns every widget, while this module owns every number. The angle
# convention follows usdview's FreeCamera transform chain
# (freeCamera.py:106-131, row-vector T(0,0,dist) Rz(-psi) Rx(-phi)
# Ry(-theta) YZUpInv T(center)); the position direction p(theta, phi)
# and the screen-up vector below reproduce its frustum in both up modes
# to < 1e-15 (spec section 2).
#
import math

from pxr import Gf


# Frames -----------------------------------------------------------------
#
# Up-space is a right-handed frame whose +Y is the stage's up axis and
# whose +Z points toward the FRONT camera. For Y-up stages it is world
# space; for Z-up stages world = (x_up, -z_up, y_up) (checked against the
# measured free-camera positions: theta = phi = 0 on a Z-up stage puts
# the camera at world (0, -dist, 0) = up-space (0, 0, +dist)).
def UpSpaceToWorld(isZUp):
    """Up-space frame to world; row-vector, use M.TransformDir(v)."""
    if not isZUp:
        return Gf.Matrix4d(1.0)
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, -1.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 1.0)


def WorldToUpSpace(isZUp):
    """World frame to up-space; the inverse of UpSpaceToWorld."""
    if not isZUp:
        return Gf.Matrix4d(1.0)
    return Gf.Matrix4d(1.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, -1.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 1.0)


# Regions ----------------------------------------------------------------
FACE_FRONT, FACE_BACK, FACE_RIGHT, FACE_LEFT, FACE_TOP, FACE_BOTTOM = \
    "front", "back", "right", "left", "top", "bottom"
FACES = (FACE_FRONT, FACE_BACK, FACE_RIGHT, FACE_LEFT, FACE_TOP,
         FACE_BOTTOM)

# Up-space normals (spec section 3).
FACE_NORMALS = {
    FACE_FRONT: Gf.Vec3d(0, 0, 1),
    FACE_BACK: Gf.Vec3d(0, 0, -1),
    FACE_RIGHT: Gf.Vec3d(1, 0, 0),
    FACE_LEFT: Gf.Vec3d(-1, 0, 0),
    FACE_TOP: Gf.Vec3d(0, 1, 0),
    FACE_BOTTOM: Gf.Vec3d(0, -1, 0),
}

FACE_LABELS = {
    FACE_FRONT: "FRONT",
    FACE_BACK: "BACK",
    FACE_RIGHT: "RIGHT",
    FACE_LEFT: "LEFT",
    FACE_TOP: "TOP",
    FACE_BOTTOM: "BOTTOM",
}

# In-plane (right, up) up-space axes per face, so each label reads
# upright in that face's canonical view (spec section 3).
FACE_AXES = {
    FACE_FRONT: (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0)),
    FACE_BACK: (Gf.Vec3d(-1, 0, 0), Gf.Vec3d(0, 1, 0)),
    FACE_RIGHT: (Gf.Vec3d(0, 0, -1), Gf.Vec3d(0, 1, 0)),
    FACE_LEFT: (Gf.Vec3d(0, 0, 1), Gf.Vec3d(0, 1, 0)),
    FACE_TOP: (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 0, -1)),
    FACE_BOTTOM: (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 0, 1)),
}

# Canonical name order: front|back, then top|bottom, then left|right.
_FACE_ORDER = {
    FACE_FRONT: (0, 0),
    FACE_BACK: (0, 1),
    FACE_TOP: (1, 0),
    FACE_BOTTOM: (1, 1),
    FACE_LEFT: (2, 0),
    FACE_RIGHT: (2, 1),
}

# Reverse lookup from an axis-aligned normal to its face, for the hit
# test's neighbour faces.
_NORMAL_TO_FACE = {(int(n[0]), int(n[1]), int(n[2])): name
                          for name, n in FACE_NORMALS.items()}


class Region(object):
    """One of the 26 click regions: faces, name and up-space direction."""

    def __init__(self, name, faces, direction):
        self.name = name
        self.faces = faces
        self.direction = direction

    def __repr__(self):
        return "<Region %s>" % self.name


def RegionName(faces):
    """Canonical name for face names: sorted, joined with '-'."""
    if isinstance(faces, str):
        faces = (faces,)
    ordered = sorted(faces, key=lambda f: _FACE_ORDER[f])
    seen = set()
    for face in ordered:
        axis = _FaceAxis(face)
        if axis in seen:
            raise ValueError("two faces on one axis: %s" % (faces,))
        seen.add(axis)
    if not ordered:
        raise ValueError("a region needs at least one face")
    return "-".join(ordered)


def _FaceAxis(face):
    """The 0/1/2 axis the face's normal lies on."""
    normal = FACE_NORMALS[face]
    for axis in range(3):
        if abs(normal[axis]) > 0.5:
            return axis
    raise ValueError("face has no axis: %s" % face)


def RegionForFaces(faces):
    """The Region for face names in any order (or a single name)."""
    if isinstance(faces, str):
        faces = (faces,)
    return REGIONS[RegionName(faces)]


def _BuildRegions():
    groups = ((FACE_FRONT, FACE_BACK), (FACE_TOP, FACE_BOTTOM),
              (FACE_LEFT, FACE_RIGHT))
    regions = {}
    for first in (None,) + groups[0]:
        for second in (None,) + groups[1]:
            for third in (None,) + groups[2]:
                faces = tuple(f for f in (first, second, third)
                              if f is not None)
                if not faces:
                    continue
                total = Gf.Vec3d(0, 0, 0)
                for face in faces:
                    total += FACE_NORMALS[face]
                name = RegionName(faces)
                regions[name] = Region(name, tuple(sorted(
                    faces, key=lambda f: _FACE_ORDER[f])),
                    total.GetNormalized())
    return regions


REGIONS = _BuildRegions()

# A hit-test coordinate beyond this (in face units where the face spans
# [-1, 1]) belongs to the neighbouring face's band (spec section 3).
ZONE_LIMIT = 0.5


# Angles -----------------------------------------------------------------
#
# Measured against FreeCamera (spec section 2, worst error 4.4e-16): the
# camera position direction from the orbit centre toward the camera, in
# up-space, is p(theta, phi) below, so theta = 0, phi = 0 is FRONT.
def DirectionForAngles(theta, phi):
    """Up-space unit direction from the centre toward the camera."""
    radians = math.radians(theta)
    elevation = math.radians(phi)
    sine, cosine = math.sin(radians), math.cos(radians)
    sinePhi, cosinePhi = math.sin(elevation), math.cos(elevation)
    return Gf.Vec3d(-sine * cosinePhi, sinePhi, cosine * cosinePhi)


def SnapHeading(theta):
    """The nearest multiple of 90 degrees to the heading."""
    return round(theta / 90.0) * 90.0


def Unwrap(theta, reference):
    """theta shifted by whole turns to within (-180, 180] of reference."""
    delta = (theta - reference) % 360.0
    if delta > 180.0:
        delta -= 360.0
    return reference + delta


def AnglesForDirection(direction, currentTheta):
    """(theta, phi) placing the camera along unit up-space `direction`.

    A pure TOP / BOTTOM view has no heading of its own, so it keeps the
    camera's current heading snapped to 90 degrees (spec section 1.6);
    theta is unwrapped to the representative within +-180 of the current
    one so the animated orbit takes the short way round.
    """
    side = max(-1.0, min(1.0, direction[1]))
    phi = math.degrees(math.asin(side))
    if abs(side) >= 1.0 - 1e-9:
        theta = SnapHeading(currentTheta)
    else:
        theta = math.degrees(math.atan2(-direction[0], direction[2]))
    return (Unwrap(theta, currentTheta), phi)


def Smoothstep(t):
    """3t^2 - 2t^3, clamped to [0, 1]; the orbit easing."""
    clamped = max(0.0, min(1.0, t))
    return clamped * clamped * (3.0 - 2.0 * clamped)


def LerpAngles(fromAngles, toAngles, t):
    """Linear interpolation; `toAngles` must already be unwrapped."""
    return (fromAngles[0] + (toAngles[0] - fromAngles[0]) * t,
            fromAngles[1] + (toAngles[1] - fromAngles[1]) * t)


# The corner view the home glyph orbits to (spec section 1.7).
HOME_REGION = "front-top-right"


# Camera basis -----------------------------------------------------------
class Basis(object):
    """World-space camera frame: right, up and view (looking) vectors."""

    def __init__(self, right, up, view):
        self.right = Gf.Vec3d(right)
        self.up = Gf.Vec3d(up)
        self.view = Gf.Vec3d(view)

    def ToView(self, world):
        """World vector to view coordinates (w.right, w.up, -w.view)."""
        direction = Gf.Vec3d(world)
        return Gf.Vec3d(Gf.Dot(direction, self.right),
                        Gf.Dot(direction, self.up),
                        -Gf.Dot(direction, self.view))

    def FromView(self, view):
        """View coordinates back to a world vector."""
        coords = Gf.Vec3d(view)
        return (self.right * coords[0] + self.up * coords[1]
                - self.view * coords[2])


def BasisFromFrustum(frustum):
    """Basis from a Gf.Frustum's view direction and up vector."""
    view = Gf.Vec3d(frustum.ComputeViewDirection()).GetNormalized()
    up = Gf.Vec3d(frustum.ComputeUpVector()).GetNormalized()
    right = Gf.Cross(view, up).GetNormalized()
    return Basis(right, up, view)


def BasisForAngles(theta, phi, isZUp):
    """Exactly the basis FreeCamera(isZUp) gives for theta/phi, roll 0.

    Closed form in up-space (verified against FreeCamera in both up
    modes, error < 1e-15): view = -p(theta, phi) and up =
    (sin(theta) sin(phi), cos(phi), -cos(theta) sin(phi)), both carried
    to world.
    """
    radians = math.radians(theta)
    elevation = math.radians(phi)
    sine, cosine = math.sin(radians), math.cos(radians)
    sinePhi, cosinePhi = math.sin(elevation), math.cos(elevation)
    toWorld = UpSpaceToWorld(isZUp)
    view = toWorld.TransformDir(Gf.Vec3d(sine * cosinePhi, -sinePhi,
                                         -cosine * cosinePhi))
    up = toWorld.TransformDir(Gf.Vec3d(sine * sinePhi, cosinePhi,
                                       -cosine * sinePhi))
    return Basis(Gf.Cross(view, up), up, view)


# The basis of the FRONT view on a Y-up stage.
IDENTITY_BASIS = Basis(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0),
                       Gf.Vec3d(0, 0, -1))


# Projection -------------------------------------------------------------
# View-space z of the eye; the cube half-size is 1 (spec section 3).
EYE_DISTANCE = 4.0
# The square widget's side in logical pixels.
WIDGET_SIZE = 110.0
# R = RADIUS_FRACTION * WIDGET_SIZE is the face half-size in pixels at
# view-space z = 0; the worst silhouette (corner views, 1.9085 * R =
# 50.4 px) fits inside WIDGET_SIZE / 2 - 2.
RADIUS_FRACTION = 0.24
# Widget offset from the view's top-right corner in logical pixels.
MARGIN = 12.0
# Home glyph size in pixels, in the widget's top-left corner.
HOME_GLYPH_SIZE = 14.0


class ProjectedFace(object):
    """One cube face in widget pixels: corners, visibility and depth."""

    def __init__(self, name, polygon, visible, depth, quad):
        self.name = name
        # Four (x, y) corners in FACE corner order: (-1,+1), (+1,+1),
        # (+1,-1), (-1,-1) in the face's (right, up) axes, i.e.
        # top-left, top-right, bottom-right, bottom-left.
        self.polygon = polygon
        self.visible = visible
        # View-space z of the face centre.
        self.depth = depth
        # The same corners as (u, v) in [-1, 1]^2.
        self.quad = quad


def Project(pointUp, basis, isZUp, radius, centre):
    """Up-space cube point to widget pixels (x, y)."""
    world = UpSpaceToWorld(isZUp).Transform(Gf.Vec3d(pointUp))
    coords = basis.ToView(world)
    scale = radius * EYE_DISTANCE / (EYE_DISTANCE - coords[2])
    return (centre[0] + coords[0] * scale,
            centre[1] - coords[1] * scale)


def ProjectCube(basis, isZUp, radius, centre):
    """The six faces in FACES order, with visibility and depth."""
    toWorld = UpSpaceToWorld(isZUp)
    eye = Gf.Vec3d(0, 0, EYE_DISTANCE)
    corners = ((-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0))
    faces = []
    for name in FACES:
        normal = FACE_NORMALS[name]
        right, up = FACE_AXES[name]
        faceCentre = Gf.Vec3d(normal)
        polygon, quad = [], []
        for side, height in corners:
            corner = faceCentre + right * side + up * height
            polygon.append(Project(corner, basis, isZUp, radius,
                                   centre))
            quad.append((side, height))
        # A face is visible iff the eye is on the outer side of its
        # plane; a convex cube's visible faces never overlap, so no
        # depth sort is needed (spec section 3).
        centreView = basis.ToView(toWorld.Transform(faceCentre))
        normalView = basis.ToView(
            toWorld.TransformDir(Gf.Vec3d(normal)))
        facing = Gf.Dot(eye - centreView, normalView)
        faces.append(ProjectedFace(name, polygon, facing > 0.0,
                                   centreView[2], quad))
    return faces


def VisibleFaces(faces):
    """The visible ProjectedFaces of a ProjectCube result."""
    return [face for face in faces if face.visible]


def HitTest(basis, isZUp, radius, centre, point):
    """The Region under widget pixel `point`, or None on a miss.

    A ray cast, not a polygon test, so faces, edges and corners come
    out of one computation: the pixel maps back to the z = 0 plane,
    the ray from the eye through it is carried to up-space, and a slab
    intersection with [-1, 1]^3 yields the entry face; an in-plane
    coordinate beyond +-ZONE_LIMIT adds that neighbouring face.
    """
    plane = ((point[0] - centre[0]) / radius,
             (centre[1] - point[1]) / radius)
    toUp = WorldToUpSpace(isZUp)
    # FromView is a pure rotation (no translation), so it carries the
    # eye point and the ray direction alike.
    origin = toUp.Transform(
        basis.FromView(Gf.Vec3d(0, 0, EYE_DISTANCE)))
    direction = toUp.TransformDir(basis.FromView(
        Gf.Vec3d(plane[0], plane[1], -EYE_DISTANCE)))
    low, high = -1.0, 1.0
    closest, farthest = float("-inf"), float("inf")
    entry = None
    for axis in range(3):
        offset, slope = origin[axis], direction[axis]
        if abs(slope) < 1e-12:
            if offset < low or offset > high:
                return None
            continue
        if slope > 0.0:
            near, far, sign = ((low - offset) / slope,
                               (high - offset) / slope, -1)
        else:
            near, far, sign = ((high - offset) / slope,
                               (low - offset) / slope, 1)
        if near > closest:
            closest, entry = near, (axis, sign)
        if far < farthest:
            farthest = far
        if closest > farthest:
            return None
    if entry is None or farthest < 0.0:
        return None
    axis, sign = entry
    key = [0, 0, 0]
    key[axis] = sign
    faces = [_NORMAL_TO_FACE[tuple(key)]]
    hit = origin + direction * closest
    right, up = FACE_AXES[faces[0]]
    for coord, axisVec in ((Gf.Dot(hit, right), right),
                           (Gf.Dot(hit, up), up)):
        if abs(coord) > ZONE_LIMIT:
            sense = 1 if coord > 0.0 else -1
            neighbour = axisVec * sense
            key = (int(round(neighbour[0])), int(round(neighbour[1])),
                   int(round(neighbour[2])))
            name = _NORMAL_TO_FACE.get(key)
            if name is not None and name not in faces:
                faces.append(name)
    return RegionForFaces(faces)


def RegionPolygons(region, basis, isZUp, radius, centre):
    """The region's highlight polygons: projected sub-quads on each of
    its VISIBLE faces -- exactly the HitTest zones, so a face's polygon
    is the middle [-ZONE_LIMIT, ZONE_LIMIT]^2 square, an edge's a band
    on each of its two faces (never the corner squares), and a corner's
    the outer square on each of its three faces."""
    if isinstance(region, str):
        region = REGIONS[region]
    projected = {face.name: face
                 for face in ProjectCube(basis, isZUp, radius, centre)}
    # Which cube axis each region face's normal lies on, and its sign.
    onAxis = {}
    for face in region.faces:
        onAxis[_FaceAxis(face)] = FACE_NORMALS[face][_FaceAxis(face)]
    polygons = []
    for face in region.faces:
        shown = projected[face]
        if not shown.visible:
            continue
        faceCentre = Gf.Vec3d(FACE_NORMALS[face])
        right, up = FACE_AXES[face]
        spans = []
        for axisVec in (right, up):
            axis = _FaceAxisOf(axisVec)
            sign = onAxis.get(axis)
            if sign is None:
                spans.append((-ZONE_LIMIT, ZONE_LIMIT))
            elif sign * axisVec[axis] > 0.0:
                spans.append((ZONE_LIMIT, 1.0))
            else:
                spans.append((-1.0, -ZONE_LIMIT))
        (lowU, highU), (lowV, highV) = spans
        polygon = []
        for side, height in ((lowU, highV), (highU, highV),
                             (highU, lowV), (lowU, lowV)):
            corner = faceCentre + right * side + up * height
            polygon.append(Project(corner, basis, isZUp, radius,
                                   centre))
        polygons.append(polygon)
    return polygons


def _FaceAxisOf(direction):
    """The 0/1/2 axis an axis-aligned unit vector lies on."""
    for axis in range(3):
        if abs(direction[axis]) > 0.5:
            return axis
    raise ValueError("not axis-aligned: %s" % (direction,))


def _PolygonArea(points):
    """Unsigned area of a projected polygon, in square pixels."""
    total = 0.0
    for index in range(len(points)):
        here, there = points[index], points[(index + 1) % len(points)]
        total += here[0] * there[1] - there[0] * here[1]
    return abs(total) * 0.5


def RegionPoint(region, basis, isZUp, radius, centre):
    """A widget-local point inside the region, or None when none of its
    faces is visible."""
    if isinstance(region, str):
        region = REGIONS[region]
    polygons = RegionPolygons(region, basis, isZUp, radius, centre)
    if not polygons:
        return None
    biggest = max(polygons, key=_PolygonArea)
    count = len(biggest)
    return (sum(p[0] for p in biggest) / count,
            sum(p[1] for p in biggest) / count)


def SilhouetteRadius(basis, isZUp, radius):
    """Max distance of any projected cube corner from the centre."""
    centre = (0.0, 0.0)
    worst = 0.0
    for cornerX in (-1.0, 1.0):
        for cornerY in (-1.0, 1.0):
            for cornerZ in (-1.0, 1.0):
                corner = Gf.Vec3d(cornerX, cornerY, cornerZ)
                pixel = Project(corner, basis, isZUp, radius, centre)
                worst = max(worst, math.hypot(pixel[0], pixel[1]))
    return worst
