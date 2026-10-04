#
# The pose-reader visualization, with no Qt in it.
#
# What an interpolator's falloff LOOKS like, as world-space geometry, and
# that geometry projected to the screen as plain polygons and circles. The
# overlay in `poseReaderOverlay` only paints what this hands it, so every
# number that decides where a cone points or how wide a sphere is can be
# tested headlessly against the engine.
#
# WHAT IS DRAWN, and why it is exact rather than suggestive. Each pose
# authors its own falloff -- `rigExec:rotationRadius` in radians and
# `rigExec:translationRadius` in scene units -- and the evaluator solves
# with exactly those (rigEvaluatorPose.cpp _CompilePoseInterpolators). So
# the shapes here are the solver's own supports, measured in the solver's
# own frame:
#
#   rotation     the driver's local rotation relative to its REST, carried
#                by the parent's CURRENT pose. With row-vector matrices,
#                     Ref = driverRest * parentRest^-1 * parentFinal
#                and a pose with quaternion q aims the twist axis along
#                axis * M(q) * Ref. A swing pose is drawn as a CONE about
#                that direction; a twist pose as a FAN about the axis.
#   translation  the driver's origin relative to its rest, in that same
#                rest frame (solverKernels.cpp RigExecFrameTranslation),
#                so pose T sits at T * Ref and is drawn as a SPHERE.
#
# Two nested shapes per pose, the convention pose readers use: the OUTER
# one is the support (distance 1 in the kernel's units -- a linear kernel
# reaches zero there, a gaussian e^-1), the INNER "core" is where the
# kernel is still at one half. The kernel is the RAW falloff, before the
# solve and the normalisation; the number on a label is the published,
# normalised weight. The two disagree exactly where poses overlap, which
# is the thing this view exists to show.
#
import math

from pxr import Gf, Sdf, Usd, UsdGeom

INTERPOLATOR = "RigExecPoseInterpolator"
POSE = "RigExecPose"
PROVIDERS = ("RigExecJoint", "RigExecControl")

# The parts the panel toggles. Every one is drawn only when the master
# switch is on, and the master switch is off by default.
PART_CONES = "cones"
PART_CORES = "cores"
PART_TWIST = "twist"
PART_SPHERES = "spheres"
PART_DRIVER = "driver"
PART_LABELS = "labels"
PART_OVERLAP = "overlap"
PARTS = (PART_CONES, PART_CORES, PART_TWIST, PART_SPHERES, PART_DRIVER,
         PART_LABELS, PART_OVERLAP)

SCOPE_SELECTED = "selected"
SCOPE_FIRING = "firing"
SCOPE_ALL = "all"
SCOPES = (SCOPE_SELECTED, SCOPE_FIRING, SCOPE_ALL)

# Overlap levels, strongest first. COINCIDENT is two supports whose centres
# sit closer than a quarter of the smaller one: under a normalised linear
# kernel that pair can drive the weights far outside 0..1.
OVERLAP_COINCIDENT = "coincident"
OVERLAP_CORES = "cores"
OVERLAP_SUPPORTS = "supports"
OVERLAP_LEVELS = (OVERLAP_COINCIDENT, OVERLAP_CORES, OVERLAP_SUPPORTS)
COINCIDENT_FRACTION = 0.25

# A weight below this does not brighten a shape.
LIVE = 1e-3
# A pose earns a label only past this: at a few hundredths every pose of a
# normalised RBF carries some weight, and labelling them all buries the one
# that is firing.
LABEL_LIVE = 0.05

# Per-pose colours, cycled. Saturated enough to tell apart on a
# grey-shaded body; the neutral is drawn in grey because it
# is the rest pose, not a corrective.
PALETTE = (
    (0.95, 0.45, 0.30), (0.30, 0.70, 0.95), (0.55, 0.85, 0.35),
    (0.95, 0.80, 0.25), (0.75, 0.45, 0.95), (0.25, 0.85, 0.75),
    (0.95, 0.40, 0.65), (0.60, 0.65, 0.95),
)
NEUTRAL_COLOR = (0.70, 0.72, 0.76)
DRIVER_COLOR = (1.0, 0.95, 0.55)
OVERLAP_COLORS = {
    OVERLAP_COINCIDENT: (1.0, 0.25, 0.25),
    OVERLAP_CORES: (1.0, 0.60, 0.20),
    OVERLAP_SUPPORTS: (0.95, 0.90, 0.30),
}

# Every rotation reader draws at this length, in scene units, times the
# panel's Size slider: one size for all of them, so they read the same
# wherever they sit. Display only -- the falloff ANGLES are the rig's.
READER_LENGTH = 15.0

# A cone wider than this is drawn at this: past ~179 degrees the support
# is the whole sphere and the silhouette degenerates.
MAX_HALF_ANGLE = math.radians(179.0)

# Samples around a base circle, a fan or a sphere outline.
CIRCLE_SAMPLES = 40
FAN_SAMPLES = 24


class Settings(object):
    """What the panel asked to see. Off by default; every part on."""

    def __init__(self):
        self.enabled = False
        self.parts = {part: True for part in PARTS}
        self.scope = SCOPE_SELECTED
        self.size = 1.0
        self.opacity = 1.0

    def Shows(self, part):
        return bool(self.enabled and self.parts.get(part, False))


def KernelCoreFraction(kernel):
    """The kernel distance at which a pose's raw weight is one half.

    Linear is max(0, 1 - d): one half at d = 0.5. Gaussian is exp(-d^2):
    one half at d = sqrt(ln 2) ~ 0.833. The evaluator treats any kernel
    name but "linear" as gaussian, and so does this.
    """
    return 0.5 if kernel == "linear" else math.sqrt(math.log(2.0))


def KernelWeight(kernel, distance):
    """The raw falloff at a kernel distance (1 = the pose's own radius)."""
    if kernel == "linear":
        return max(0.0, 1.0 - distance)
    return math.exp(-(distance * distance))


# -- shapes ---------------------------------------------------------------

class _Shape(object):
    """One pose's drawing: what every kind carries."""

    kind = None

    def __init__(self, pose):
        self.path = pose["path"]
        self.name = pose["name"]
        self.weight = pose["weight"]
        self.neutral = pose["name"] == "neutral"
        self.color = NEUTRAL_COLOR if self.neutral else pose["color"]

    def Tip(self):
        """Where a label or an overlap line attaches."""
        raise NotImplementedError


class Cone(_Shape):
    """A swing pose: apex at the driver, axis toward the pose."""

    kind = "cone"

    def __init__(self, pose, apex, axis, halfAngle, coreAngle, length):
        super(Cone, self).__init__(pose)
        self.apex = Gf.Vec3d(apex)
        self.axis = Gf.Vec3d(axis).GetNormalized()
        self.halfAngle = min(max(halfAngle, 0.0), MAX_HALF_ANGLE)
        self.coreAngle = min(max(coreAngle, 0.0), MAX_HALF_ANGLE)
        self.length = length

    def Tip(self):
        return self.apex + self.axis * self.length


class Fan(_Shape):
    """A twist pose: an arc about the twist axis, centred on its angle."""

    kind = "fan"

    def __init__(self, pose, origin, axis, reference, angle, halfWidth,
                 coreWidth, radius):
        super(Fan, self).__init__(pose)
        self.origin = Gf.Vec3d(origin)
        self.axis = Gf.Vec3d(axis).GetNormalized()
        # The zero-twist direction, made exactly perpendicular so the arc
        # is a circle and not an ellipse when Ref carries a little shear.
        ref = Gf.Vec3d(reference)
        ref = (ref - self.axis * Gf.Dot(ref, self.axis))
        self.reference = (ref.GetNormalized() if ref.GetLength() > 1e-12
                          else _AnyPerpendicular(self.axis))
        self.angle = angle
        self.halfWidth = max(halfWidth, 0.0)
        self.coreWidth = max(coreWidth, 0.0)
        self.radius = radius

    def PointAt(self, angle, radius=None):
        r = self.radius if radius is None else radius
        side = Gf.Cross(self.axis, self.reference)
        return (self.origin + (self.reference * math.cos(angle) +
                               side * math.sin(angle)) * r)

    def Tip(self):
        return self.PointAt(self.angle)


class Sphere(_Shape):
    """A translation pose: the support ball around the pose position."""

    kind = "sphere"

    def __init__(self, pose, centre, radius, coreRadius):
        super(Sphere, self).__init__(pose)
        self.centre = Gf.Vec3d(centre)
        self.radius = max(radius, 0.0)
        self.coreRadius = max(coreRadius, 0.0)

    def Tip(self):
        return self.centre


class Reader(object):
    """One interpolator, ready to draw -- or a reason it cannot be."""

    def __init__(self, path, name, driver):
        self.path = path
        self.name = name
        self.driver = driver
        self.kind = None          # rotation | translation | numeric
        self.kernel = "gaussian"
        self.shapes = []
        self.overlaps = []        # (shape, shape, level)
        self.note = ""            # why there is nothing to draw
        self.length = READER_LENGTH
        # The live driver: where it is and where its twist axis points.
        self.liveOrigin = None
        self.liveAxis = None
        self.liveTwist = None     # radians, for twist readers

    @property
    def drawable(self):
        return bool(self.shapes)

    @property
    def firing(self):
        return any((not s.neutral) and abs(s.weight) > LIVE
                   for s in self.shapes)


# -- frames ---------------------------------------------------------------

def _Get(prim, name, default):
    attr = prim.GetAttribute(name)
    value = attr.Get() if attr and attr.IsValid() else None
    return default if value is None else value


def _AxisVector(token):
    """The twist axis the evaluator reads: X unless Y or Z is named."""
    token = str(token)
    if token == "Y":
        return Gf.Vec3d(0.0, 1.0, 0.0)
    if token == "Z":
        return Gf.Vec3d(0.0, 0.0, 1.0)
    return Gf.Vec3d(1.0, 0.0, 0.0)


def _AnyPerpendicular(axis):
    """A unit vector at right angles to `axis`, chosen stably."""
    trial = (Gf.Vec3d(0.0, 1.0, 0.0) if abs(axis[1]) < 0.9
             else Gf.Vec3d(0.0, 0.0, 1.0))
    side = Gf.Cross(axis, trial)
    return side.GetNormalized()


def _Matrix(value):
    """A frame's matrix, however the binding hands it over."""
    if isinstance(value, Gf.Matrix4d):
        return Gf.Matrix4d(value)
    flat = list(value)
    if len(flat) == 4:
        return Gf.Matrix4d(*[list(row) for row in flat])
    return Gf.Matrix4d(*flat)


def FinalFrame(pose, path):
    """The evaluated, asset-space frame of a joint or control, or None.

    Joints and controls publish into different maps, so both are asked;
    a degenerate or missing frame is None rather than a guess.
    """
    text = str(path)
    for read in (lambda: pose.joint_frame(text, True),
                 lambda: pose.control_frame(text)):
        try:
            frame = read()
        except Exception:
            continue
        if frame is None or not getattr(frame, "valid", True):
            continue
        if getattr(frame, "degenerate", False):
            continue
        return _Matrix(frame.to_matrix4())
    return None


def ProviderParent(prim, rigRoot):
    """The nearest joint or control ancestor, as the evaluator picks it.

    The driver's local rotation is measured against its nearest
    frame-publishing ancestor (rigEvaluatorPose.cpp), stopping at the
    rig's own parent. None for a driver with no such ancestor, which the
    evaluator measures against identity.
    """
    stop = rigRoot.GetPath().GetParentPath() if rigRoot else None
    walk = prim.GetParent()
    while walk and walk.IsValid() and not walk.IsPseudoRoot():
        if stop is not None and walk.GetPath() == stop:
            return None
        if str(walk.GetTypeName()) in PROVIDERS:
            return walk
        walk = walk.GetParent()
    return None


def RigRoot(stage, prim):
    """The RigExecRoot above `prim`, or the first one on the stage."""
    walk = prim
    while walk and walk.IsValid() and not walk.IsPseudoRoot():
        if str(walk.GetTypeName()) == "RigExecRoot":
            return walk
        walk = walk.GetParent()
    for candidate in stage.Traverse():
        if str(candidate.GetTypeName()) == "RigExecRoot":
            return candidate
    return None


def AssetToWorld(stage, rigRoot, time):
    """Where the asset sits: the world transform of the rig's parent."""
    if rigRoot is None:
        return Gf.Matrix4d(1.0)
    parent = rigRoot.GetParent()
    if not parent or parent.IsPseudoRoot():
        return Gf.Matrix4d(1.0)
    cache = UsdGeom.XformCache(time)
    return cache.GetLocalToWorldTransform(parent)


def _RestMatrix(prim, time, restSpace, cache=None):
    """The orthonormal rest frame, asset space; identity without a prim.

    `cache` maps prim paths to rest matrices. Rest frames do not move
    while a pose is being dragged, and recomputing one walks its whole
    ancestor chain -- MEASURED at 87% of a 52-reader build (280 ms) -- so
    the panel keeps the cache and clears it on a rest edit or a new stage.
    """
    if prim is None:
        return Gf.Matrix4d(1.0)
    path = prim.GetPath()
    if cache is not None and path in cache:
        return Gf.Matrix4d(cache[path])
    matrix = Gf.Matrix4d(restSpace(prim, time))
    if cache is not None:
        cache[path] = Gf.Matrix4d(matrix)
    return matrix


def _QuatMatrix(quat):
    """M(q), the row-vector rotation matrix the evaluator's quats mean."""
    real = quat.GetReal()
    imag = Gf.Vec3d(quat.GetImaginary())
    q = Gf.Quatd(real, imag)
    if q.GetLength() < 1e-12:
        return Gf.Matrix4d(1.0)
    return Gf.Matrix4d(Gf.Rotation(q.GetNormalized()), Gf.Vec3d(0.0))


def _RotationOnly(matrix):
    """A matrix's rotation, with translation, scale and shear removed."""
    m = Gf.Matrix4d(matrix)
    m.SetTranslateOnly(Gf.Vec3d(0.0))
    m.Orthonormalize(False)
    return m


def TwistAngle(quat, axis):
    """The twist of a quaternion about a unit axis, in (-pi, pi]."""
    real = quat.GetReal()
    imag = Gf.Vec3d(quat.GetImaginary())
    angle = 2.0 * math.atan2(Gf.Dot(imag, axis), real)
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


# -- building -------------------------------------------------------------

def _Poses(prim, weights):
    """The enabled poses, as the evaluator reads them, with their weights."""
    out = []
    index = 0
    for child in prim.GetChildren():
        if str(child.GetTypeName()) != POSE:
            continue
        if not bool(_Get(child, "inputs:enabled", True)):
            continue
        name = child.GetName()
        color = PALETTE[index % len(PALETTE)]
        if name != "neutral":
            index += 1
        quat = _Get(child, "rigExec:rotation", Gf.Quatf(1.0))
        out.append({
            "path": child.GetPath(),
            "name": name,
            "color": color,
            "weight": float(weights.get(child.GetPath(), 0.0)),
            "quat": quat,
            "translation": Gf.Vec3d(_Get(child, "rigExec:translation",
                                         Gf.Vec3f(0.0))),
            "type": str(_Get(child, "rigExec:poseType", "swing")),
            "rotationRadius": float(_Get(child, "rigExec:rotationRadius",
                                         0.0)),
            "translationRadius": float(_Get(
                child, "rigExec:translationRadius", 0.0)),
        })
    return out


def BuildReader(stage, prim, pose, time, weights, restSpace,
                sizeScale=1.0, restCache=None):
    """One interpolator as a Reader. Never raises on a bad rig.

    `restSpace(prim, time)` is the orthonormal asset-space rest frame --
    gizmoMath.RestSpace in the panel, which replicates the evaluator's
    computeRestFrame. `weights` maps pose prim paths to the published
    weight.
    """
    rel = prim.GetRelationship("rigExec:driver")
    targets = rel.GetTargets() if rel else []
    driverPath = targets[0] if targets else None
    reader = Reader(prim.GetPath(), prim.GetName(), driverPath)
    reader.kernel = str(_Get(prim, "rigExec:kernel", "gaussian"))
    core = KernelCoreFraction(reader.kernel)

    numeric = prim.GetRelationship("rigExec:driverAttributes")
    if numeric and numeric.GetTargets():
        reader.kind = "numeric"
        reader.note = ("driven by attribute values, which have no place "
                       "in the viewport")
        return reader
    if driverPath is None:
        reader.note = "no rigExec:driver"
        return reader
    driver = stage.GetPrimAtPath(driverPath)
    if not driver or not driver.IsValid():
        reader.note = "driver %s is not on the stage" % driverPath
        return reader

    rigRoot = RigRoot(stage, prim)
    toWorld = AssetToWorld(stage, rigRoot, time)
    parent = ProviderParent(driver, rigRoot)
    finalD = FinalFrame(pose, driverPath)
    finalP = FinalFrame(pose, parent.GetPath()) if parent else \
        Gf.Matrix4d(1.0)
    if finalD is None or finalP is None:
        reader.note = "the evaluator published no frame for the driver"
        return reader
    try:
        restD = _RestMatrix(driver, time, restSpace, restCache)
        restP = _RestMatrix(parent, time, restSpace, restCache)
    except Exception as error:
        reader.note = "no rest frame: %s" % error
        return reader

    # Ref: the driver's rest carried by its parent's current pose, in world.
    restLocal = restD * restP.GetInverse()
    ref = restLocal * finalP * toWorld
    liveWorld = finalD * toWorld
    axis = _AxisVector(_Get(prim, "rigExec:twistAxis", "X"))
    origin = liveWorld.ExtractTranslation()
    reader.liveOrigin = origin
    reader.liveAxis = liveWorld.TransformDir(axis).GetNormalized()

    rotation = bool(_Get(prim, "rigExec:enableRotation", True))
    translation = bool(_Get(prim, "rigExec:enableTranslation", False))
    poses = _Poses(prim, weights)
    if not poses:
        reader.note = "every pose is disabled"
        return reader

    if rotation:
        reader.kind = "rotation"
        reader.length = READER_LENGTH * sizeScale
        refRot = _RotationOnly(ref)
        worldAxis = refRot.TransformDir(axis).GetNormalized()
        worldRef = refRot.TransformDir(_AnyPerpendicular(axis))
        # The live twist, measured the way the evaluator measures the
        # driver: delta = restLocal^-1 * local, here as matrices.
        local = _RotationOnly(finalD * finalP.GetInverse())
        delta = local * _RotationOnly(restLocal).GetInverse()
        reader.liveTwist = TwistAngle(
            _RotationOnly(delta).ExtractRotationQuat(), axis)
        for entry in poses:
            width = max(entry["rotationRadius"], 0.0)
            if entry["type"] == "twist":
                reader.shapes.append(Fan(
                    entry, origin, worldAxis, worldRef,
                    TwistAngle(entry["quat"], axis), width, width * core,
                    reader.length * 0.6))
            else:
                direction = refRot.TransformDir(
                    _QuatMatrix(entry["quat"]).TransformDir(axis))
                reader.shapes.append(Cone(
                    entry, origin, direction, width, width * core,
                    reader.length))
    elif translation:
        reader.kind = "translation"
        scale = ref.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)).GetLength()
        for entry in poses:
            radius = max(entry["translationRadius"], 0.0) * scale
            reader.shapes.append(Sphere(
                entry, ref.Transform(entry["translation"]), radius,
                radius * core))
    else:
        reader.note = "measures neither rotation nor translation"
        return reader

    reader.overlaps = Overlaps(reader.shapes)
    return reader


def Build(stage, interpolators, pose, time, restSpace, sizeScale=1.0,
          restCache=None):
    """A Reader per interpolator prim path, in the order given.

    `interpolators` is anything with `.path` and `.poses` (each pose with
    `.path` and `.weight`) -- shapeEditorModel.Interpolator -- so the
    weights shown are the ones the panel already holds.
    """
    readers = []
    for interp in interpolators:
        prim = stage.GetPrimAtPath(interp.path)
        if not prim or not prim.IsValid():
            continue
        weights = {p.path: p.weight for p in getattr(interp, "poses", ())}
        try:
            readers.append(BuildReader(stage, prim, pose, time, weights,
                                       restSpace, sizeScale, restCache))
        except Exception as error:
            reader = Reader(prim.GetPath(), prim.GetName(), None)
            reader.note = "could not be drawn: %s" % error
            readers.append(reader)
    return readers


# -- overlap --------------------------------------------------------------

def _AngleBetween(a, b):
    return math.acos(max(-1.0, min(1.0, Gf.Dot(a, b))))


def _Wrap(angle):
    angle = math.fmod(angle, 2.0 * math.pi)
    if angle > math.pi:
        angle -= 2.0 * math.pi
    if angle < -math.pi:
        angle += 2.0 * math.pi
    return abs(angle)


def _Level(gap, outerA, outerB, coreA, coreB):
    if gap < COINCIDENT_FRACTION * min(outerA, outerB):
        return OVERLAP_COINCIDENT
    if gap < coreA + coreB:
        return OVERLAP_CORES
    if gap < outerA + outerB:
        return OVERLAP_SUPPORTS
    return None


def Overlaps(shapes):
    """Every pair of same-kind supports that intersect, strongest first.

    Measured on the quantity each kind falls off along: the angle between
    cone axes, the twist difference between fans, the distance between
    sphere centres. A zero-width support overlaps nothing.
    """
    found = []
    for i in range(len(shapes)):
        a = shapes[i]
        for j in range(i + 1, len(shapes)):
            b = shapes[j]
            if a.kind != b.kind:
                continue
            if a.kind == "cone":
                if a.halfAngle <= 0.0 or b.halfAngle <= 0.0:
                    continue
                level = _Level(_AngleBetween(a.axis, b.axis), a.halfAngle,
                               b.halfAngle, a.coreAngle, b.coreAngle)
            elif a.kind == "fan":
                if a.halfWidth <= 0.0 or b.halfWidth <= 0.0:
                    continue
                level = _Level(_Wrap(a.angle - b.angle), a.halfWidth,
                               b.halfWidth, a.coreWidth, b.coreWidth)
            else:
                if a.radius <= 0.0 or b.radius <= 0.0:
                    continue
                level = _Level((a.centre - b.centre).GetLength(), a.radius,
                               b.radius, a.coreRadius, b.coreRadius)
            if level is not None:
                found.append((a, b, level))
    found.sort(key=lambda entry: OVERLAP_LEVELS.index(entry[2]))
    return found


def OverlapSummary(readers):
    """{level: count} over the readers given."""
    counts = {level: 0 for level in OVERLAP_LEVELS}
    for reader in readers:
        for _a, _b, level in reader.overlaps:
            counts[level] += 1
    return counts


# -- scope ----------------------------------------------------------------

def Visible(readers, settings, selected=()):
    """The readers the scope asks for, drawable ones only.

    `selected` is a set of interpolator prim paths. SELECTED with nothing
    selected draws nothing -- an empty selection is not "everything",
    which on the biped is 65 readers on top of each other.
    """
    if not settings.enabled:
        return []
    drawable = [r for r in readers if r.drawable]
    if settings.scope == SCOPE_ALL:
        return drawable
    if settings.scope == SCOPE_FIRING:
        return [r for r in drawable if r.firing]
    chosen = set(Sdf.Path(str(p)) for p in selected)
    return [r for r in drawable if r.path in chosen]


def InterpolatorsFor(stage, paths):
    """Interpolator paths for a usdview selection.

    An interpolator selects itself, a pose selects its interpolator, and
    a driver selects every interpolator it drives -- the joint is the
    thing an animator actually clicks.
    """
    paths = [Sdf.Path(str(p)) for p in paths]
    found = set()
    drivers = set()
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        kind = str(prim.GetTypeName())
        if kind == INTERPOLATOR:
            found.add(path)
        elif kind == POSE and str(prim.GetParent().GetTypeName()) == \
                INTERPOLATOR:
            found.add(prim.GetParent().GetPath())
        else:
            drivers.add(path)
    if drivers:
        for prim in stage.Traverse():
            if str(prim.GetTypeName()) != INTERPOLATOR:
                continue
            rel = prim.GetRelationship("rigExec:driver")
            for target in (rel.GetTargets() if rel else []):
                if target in drivers:
                    found.add(prim.GetPath())
    return found


# -- screen ---------------------------------------------------------------
#
# The projection half: world shapes to 2D draw operations in PHYSICAL
# pixels, the space gizmoScreen projects into. Kept here so the
# silhouettes are tested without a display.

class DrawOp(object):
    """One thing to paint. `points` are physical pixels."""

    def __init__(self, kind, points=None, color=None, alpha=1.0,
                 fill=None, fillAlpha=0.0, width=1.0, dashed=False,
                 text=None, centre=None, radius=0.0, depth=0.0,
                 owner=None):
        self.kind = kind          # polygon | polyline | circle | dot | text
        self.points = points or []
        self.color = color
        self.alpha = alpha
        self.fill = fill
        self.fillAlpha = fillAlpha
        self.width = width
        self.dashed = dashed
        self.text = text
        self.centre = centre
        self.radius = radius
        self.depth = depth
        # The pose prim this op draws, or None for the driver and the
        # overlap lines -- what a caller asks "how bright is that pose".
        self.owner = owner


def _Cross2(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def ConvexHull(points):
    """The 2D convex hull, counter-clockwise (monotone chain)."""
    pts = sorted(set((round(p[0], 6), round(p[1], 6)) for p in points))
    if len(pts) <= 2:
        return pts
    lower = []
    for p in pts:
        while len(lower) >= 2 and _Cross2(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and _Cross2(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def ConeRim(apex, axis, halfAngle, length, samples=CIRCLE_SAMPLES):
    """The cone's rim as world points: a spherical cap's edge.

    The rim is where a ray at `halfAngle` from the axis meets the sphere of
    radius `length` about the apex, so a wide support stays a sensible
    shape instead of a plane at infinity.
    """
    centre = apex + axis * (length * math.cos(halfAngle))
    rho = length * math.sin(halfAngle)
    u = _AnyPerpendicular(axis)
    v = Gf.Cross(axis, u)
    return [centre + (u * math.cos(t) + v * math.sin(t)) * rho
            for t in (2.0 * math.pi * k / samples for k in range(samples))]


def _Alpha(settings, weight):
    """Fill opacity: the panel's base, brightened by the live weight."""
    lit = min(1.0, abs(weight))
    return max(0.0, min(1.0, settings.opacity * (0.30 + 0.70 * lit)))


class Projector(object):
    """World points to physical pixels through one camera and viewport."""

    def __init__(self, viewProj, viewport, cameraRight=None):
        self.viewProj = viewProj
        self.viewport = viewport
        self.right = Gf.Vec3d(cameraRight) if cameraRight is not None \
            else None

    def Point(self, p):
        clip = Gf.Vec4d(p[0], p[1], p[2], 1.0) * self.viewProj
        if clip[3] <= 1e-9:
            return None
        x, y = clip[0] / clip[3], clip[1] / clip[3]
        vp = self.viewport
        return ((x + 1.0) * 0.5 * vp[2] + vp[0],
                (1.0 - y) * 0.5 * vp[3] + vp[1])

    def Depth(self, p):
        clip = Gf.Vec4d(p[0], p[1], p[2], 1.0) * self.viewProj
        return clip[3]

    def Points(self, points):
        out = [self.Point(p) for p in points]
        return None if any(p is None for p in out) else out

    def Radius(self, centre, radius):
        """A world radius at `centre` as pixels, along the camera's right."""
        if self.right is None or radius <= 0.0:
            return 0.0
        a = self.Point(centre)
        b = self.Point(Gf.Vec3d(centre) + self.right * radius)
        if a is None or b is None:
            return 0.0
        return math.hypot(b[0] - a[0], b[1] - a[1])


def _ConeOps(shape, settings, projector, ops):
    lit = abs(shape.weight) > LIVE
    alpha = _Alpha(settings, shape.weight)
    depth = projector.Depth(shape.apex + shape.axis * (shape.length * 0.5))
    apex = projector.Point(shape.apex)
    if apex is None:
        return
    for angle, part, fillScale in ((shape.halfAngle, PART_CONES, 1.0),
                                   (shape.coreAngle, PART_CORES, 1.6)):
        if not settings.Shows(part) or angle <= 0.0:
            continue
        rim = projector.Points(ConeRim(shape.apex, shape.axis, angle,
                                       shape.length))
        if rim is None:
            continue
        hull = ConvexHull([apex] + rim)
        ops.append(DrawOp(
            "polygon", hull, color=shape.color,
            alpha=min(1.0, alpha * 2.0) if part == PART_CONES else 0.0,
            fill=shape.color, fillAlpha=min(1.0, alpha * fillScale) * 0.6,
            width=2.0 if lit else 1.0, dashed=shape.weight < -LIVE,
            depth=depth, owner=shape.path))
        if part == PART_CONES:
            ops.append(DrawOp("polyline", rim + [rim[0]], color=shape.color,
                              alpha=min(1.0, alpha * 2.2),
                              width=2.0 if lit else 1.0,
                              dashed=shape.weight < -LIVE, depth=depth,
                              owner=shape.path))


def _FanOps(shape, settings, projector, ops):
    if not settings.Shows(PART_TWIST):
        return
    lit = abs(shape.weight) > LIVE
    alpha = _Alpha(settings, shape.weight)
    depth = projector.Depth(shape.origin)
    centre = projector.Point(shape.origin)
    if centre is None:
        return
    for width, fillScale, outline in ((shape.halfWidth, 1.0, True),
                                      (shape.coreWidth, 1.6, False)):
        if width <= 0.0:
            continue
        span = min(width, math.pi)
        arc = [shape.PointAt(shape.angle - span + 2.0 * span * k /
                             FAN_SAMPLES) for k in range(FAN_SAMPLES + 1)]
        arc2 = projector.Points(arc)
        if arc2 is None:
            continue
        ops.append(DrawOp(
            "polygon", [centre] + arc2, color=shape.color,
            alpha=min(1.0, alpha * 2.0) if outline else 0.0,
            fill=shape.color, fillAlpha=min(1.0, alpha * fillScale) * 0.6,
            width=2.0 if lit else 1.0, dashed=shape.weight < -LIVE,
            depth=depth, owner=shape.path))


def _SphereOps(shape, settings, projector, ops):
    if not settings.Shows(PART_SPHERES):
        return
    lit = abs(shape.weight) > LIVE
    alpha = _Alpha(settings, shape.weight)
    centre = projector.Point(shape.centre)
    if centre is None:
        return
    depth = projector.Depth(shape.centre)
    for radius, fillScale, outline in ((shape.radius, 1.0, True),
                                       (shape.coreRadius, 1.6, False)):
        if not outline and not settings.Shows(PART_CORES):
            continue
        pixels = projector.Radius(shape.centre, radius)
        if pixels <= 0.0:
            continue
        ops.append(DrawOp(
            "circle", centre=centre, radius=pixels, color=shape.color,
            alpha=min(1.0, alpha * 2.2) if outline else 0.0,
            fill=shape.color, fillAlpha=min(1.0, alpha * fillScale) * 0.5,
            width=2.0 if lit else 1.0, dashed=shape.weight < -LIVE,
            depth=depth, owner=shape.path))


def ScreenOps(readers, settings, projector):
    """Every draw operation for the readers, back to front."""
    ops = []
    if not settings.enabled:
        return ops
    for reader in readers:
        for shape in reader.shapes:
            if shape.kind == "cone":
                _ConeOps(shape, settings, projector, ops)
            elif shape.kind == "fan":
                _FanOps(shape, settings, projector, ops)
            else:
                _SphereOps(shape, settings, projector, ops)
    # Painter's order: farthest first, so a near support tints a far one
    # and overlap reads as a deeper colour where two fills cross.
    ops.sort(key=lambda op: -op.depth)

    top = []
    for reader in readers:
        if settings.Shows(PART_OVERLAP):
            for a, b, level in reader.overlaps:
                pts = projector.Points([a.Tip(), b.Tip()])
                if pts is None:
                    continue
                top.append(DrawOp("polyline", pts,
                                  color=OVERLAP_COLORS[level], alpha=0.95,
                                  width=3.0 if level ==
                                  OVERLAP_COINCIDENT else 2.0,
                                  dashed=level == OVERLAP_SUPPORTS))
        if settings.Shows(PART_DRIVER) and reader.liveOrigin is not None:
            origin = projector.Point(reader.liveOrigin)
            if origin is None:
                continue
            if reader.kind == "rotation" and reader.liveAxis is not None:
                tip = projector.Point(reader.liveOrigin +
                                      reader.liveAxis * reader.length * 1.15)
                if tip is not None:
                    top.append(DrawOp("polyline", [origin, tip],
                                      color=DRIVER_COLOR, alpha=1.0,
                                      width=2.5))
                    top.append(DrawOp("dot", centre=tip, radius=4.0,
                                      color=DRIVER_COLOR, alpha=1.0))
                fans = [s for s in reader.shapes if s.kind == "fan"]
                if fans and reader.liveTwist is not None:
                    tick = projector.Points([
                        fans[0].PointAt(reader.liveTwist,
                                        fans[0].radius * 0.75),
                        fans[0].PointAt(reader.liveTwist,
                                        fans[0].radius * 1.15)])
                    if tick is not None:
                        top.append(DrawOp("polyline", tick,
                                          color=DRIVER_COLOR, alpha=1.0,
                                          width=2.5))
            else:
                top.append(DrawOp("dot", centre=origin, radius=5.0,
                                  color=DRIVER_COLOR, alpha=1.0))
        if settings.Shows(PART_LABELS):
            for shape in reader.shapes:
                if abs(shape.weight) < LABEL_LIVE:
                    continue
                anchor = projector.Point(shape.Tip())
                if anchor is None:
                    continue
                top.append(DrawOp(
                    "text", centre=anchor, color=shape.color,
                    alpha=1.0,
                    text="%s %.2f" % (shape.name, shape.weight)))
    return ops + top
