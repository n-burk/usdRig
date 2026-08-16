#
# RigExec usdview plugin: authoring UI for volumetric weight objects
# (RigExecSphereWeight, RigExecPlaneWeight, RigExecCurveWeight and
# RigExecCombineWeight).
#
# The attribute widgets at the top of this file -- SetAtTime,
# AttributeValueMayHaveChanged, AttributeValueWidget and
# AttributeValueSelectWidget -- are adapted from Pixar's OpenUSD sample
# plugin at
#   extras/exec/examples/invertibleRigsExample/
#       invertibleRigsExampleUsdviewPlugin/plugin.py
# Copyright 2026 Pixar, licensed under the terms set forth in the
# LICENSE.txt file available at https://openusd.org/license.  The
# additions here are the 'sensitivity' knob on the munging line edit, the
# uniform-variability guard in SetAtTime, and Detach() so a rebuilt panel
# can revoke its notice keys before Qt deletes the C++ widgets.
#
# Everything above the "Qt widgets" banner is deliberately free of Qt:
# the panel is a thin driver over module-level authoring functions so the
# authoring rules can be tested headlessly.
#
from pxr import Gf, Sdf, Tf, Ts, Usd, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets


# The concrete volume weight types this panel can create and edit.  The
# order is the order of the Create buttons.
VOLUME_WEIGHT_TYPE_NAMES = (
    "RigExecSphereWeight",
    "RigExecPlaneWeight",
    "RigExecCurveWeight",
    "RigExecCombineWeight",
)

# RigExecCombineWeight inherits RigExecWeightObject, not RigExecXformable,
# so it has no placement avars and no falloff of its own.  Keeping the
# distinction in one place stops every section of the panel from
# re-deriving it.
PLACED_WEIGHT_TYPE_NAMES = (
    "RigExecSphereWeight",
    "RigExecPlaneWeight",
    "RigExecCurveWeight",
)

# Anything that may legally appear in rigExec:inputWeights.  The two
# authored weight types predate the volumetric ones but fold identically.
WEIGHT_OBJECT_TYPE_NAMES = VOLUME_WEIGHT_TYPE_NAMES + (
    "RigExecStaticWeight",
    "RigExecDynamicWeight",
)

# The named analytic profiles EnsureDefaultSpline can bake into a spline.
# 'curve' is excluded: it is the token that *reads* the spline, so it is
# not itself a shape.
SPLINE_PROFILE_NAMES = (
    "linear",
    "smooth",
    "easeIn",
    "easeOut",
    "constant",
)


# ---------------------------------------------------------------------
# Attribute authoring helpers (no Qt)
# ---------------------------------------------------------------------

def SetAtTime(attr, value, time):
    """
    Sets a value on an attribute at a specified time.

    If setting at the default time, a default value is written.  If time
    samples have already been authored on the attribute (which would
    override a spline), or if the attribute's value type isn't supported
    by splines, authors a time sample at the given time.  Otherwise the
    value is written as a knot in a Ts.Spline.  If no knot exists at that
    time, creates a new knot with curve interpolation for the segment
    that follows it.

    Adapted from Pixar's invertibleRigsExample usdview plugin.  The one
    behavioural addition is the uniform guard: half of what this panel
    edits is `uniform token` (falloffProfile, planeAxis, samplePhase,
    rangePolicy, drawMode), and a uniform attribute cannot legally carry
    a time sample, so those always go to the default.
    """
    if attr.GetVariability() == Sdf.VariabilityUniform:
        attr.Set(value)
        return

    valueType = attr.GetTypeName().type

    # If we're setting at the default time, or if the attribute's value
    # type isn't supported by splines, or if the attribute already has
    # time samples, use Set to write to the default or to time samples.
    if (time.IsDefault() or attr.GetNumTimeSamples() > 0 or
            not Ts.Spline.IsSupportedValueType(valueType)):
        attr.Set(value, time)
        return

    # Otherwise, write the value as a knot on the attribute's spline.
    frame = time.GetValue()
    spline = attr.GetSpline()
    knot = spline.GetKnot(frame)
    if knot:
        knot.SetValue(value)
    else:
        knot = Ts.Knot(
            typeName=valueType.typeName, time=frame, value=value,
            nextInterp=Ts.InterpCurve)
    spline.SetKnot(knot)
    attr.SetSpline(spline)


def SetVisibleAtTime(attr, value, time=None):
    """
    Authors value so that it is what the attribute actually RESOLVES to
    at `time`, and writes the plainest scene description that achieves
    that.

    This exists because usdview's currentFrame is never the default:
    appController.py:1409 sets it to Usd.TimeCode(0.0) for a stage with
    no time samples and to the first sample otherwise.  So the moment an
    artist scrubs avars:tx, SetAtTime gives that attribute a spline knot
    at frame 0 -- and a spline outranks a default in value resolution.
    A later attr.Set(value), which writes the default, would then be
    completely invisible: "Snap to selection" would silently do nothing.

    So: write the default while the attribute is still a plain default
    (a freshly created weight, which should not be born animated), and
    write at `time` once the attribute has a spline or time samples to
    outrank.
    """
    if attr is None or not attr:
        return False

    if time is None or time.IsDefault():
        attr.Set(value)
        return True

    hasSpline = attr.HasSpline() if hasattr(attr, "HasSpline") else False
    if hasSpline or attr.GetNumTimeSamples() > 0:
        SetAtTime(attr, value, time)
    else:
        attr.Set(value)
    return True


def AttributeValueMayHaveChanged(attrPath, notice):
    """
    Returns True if the given notice indicates that the value of the
    attribute at attrPath may have changed.

    Adapted from Pixar's invertibleRigsExample usdview plugin.
    """
    if attrPath in notice.GetChangedInfoOnlyPaths():
        return True

    for path in notice.GetResyncedPaths():
        if attrPath.HasPrefix(path):
            return True

    return False


# ---------------------------------------------------------------------
# Falloff spline authoring (no Qt)
# ---------------------------------------------------------------------
#
# The falloff spline is a shape over the NORMALIZED band parameter, not
# over time: x = 0 is the outer end of the band (inputs:falloffMax) and
# x = 1 is the inner end (inputs:falloffMin).  Ts.Spline.Eval() is happy
# at arbitrary x and extrapolates held outside the authored knots, which
# is exactly the saturating behaviour a falloff wants, so the engine can
# evaluate it without clamping first.

def _MakeKnot(x, y, interp=None, preSlope=0.0, preWidth=0.0,
              postSlope=0.0, postWidth=0.0):
    """
    Builds a float-typed Ts.Knot at (x, y).

    Ts.Spline() with no arguments defaults to a DOUBLE value type and
    rejects float knots, so every knot this module makes is constructed
    with an explicit float typeName to match rigExec:falloffCurve.
    """
    knot = Ts.Knot(
        typeName="float", time=float(x), value=float(y),
        nextInterp=Ts.InterpCurve if interp is None else interp)
    knot.SetPreTanSlope(float(preSlope))
    knot.SetPreTanWidth(float(preWidth))
    knot.SetPostTanSlope(float(postSlope))
    knot.SetPostTanWidth(float(postWidth))
    return knot


def _MakeProfileSpline(profile):
    """
    Returns a fresh float Ts.Spline carrying the named analytic profile.

    The curved profiles are shaped with TANGENTS rather than with extra
    knots: two knots the artist can grab is a far friendlier starting
    point than a six-knot approximation of a smoothstep, and the bezier
    reproduces the analytic shape closely enough that switching profile
    to `curve` is visually a no-op.
    """
    spline = Ts.Spline("float")

    if profile == "linear":
        spline.SetKnot(_MakeKnot(0.0, 0.0, Ts.InterpLinear))
        spline.SetKnot(_MakeKnot(1.0, 1.0, Ts.InterpLinear))
    elif profile == "smooth":
        # Zero slope at both ends with 1/3 widths is smoothstep to
        # within a thousandth: 0.157 / 0.500 / 0.843 at the quarters.
        spline.SetKnot(_MakeKnot(0.0, 0.0, postWidth=0.33))
        spline.SetKnot(_MakeKnot(1.0, 1.0, preWidth=0.33))
    elif profile == "easeIn":
        # Flat departure, steep arrival: the whole curve sits below the
        # diagonal, so the field stays near zero well into the band.
        spline.SetKnot(_MakeKnot(0.0, 0.0, postWidth=0.55))
        spline.SetKnot(_MakeKnot(1.0, 1.0, preSlope=1.5, preWidth=0.2))
    elif profile == "easeOut":
        # The mirror of easeIn: steep departure, flat arrival, so the
        # curve sits above the diagonal.
        spline.SetKnot(_MakeKnot(0.0, 0.0, postSlope=1.5, postWidth=0.2))
        spline.SetKnot(_MakeKnot(1.0, 1.0, preWidth=0.55))
    elif profile == "constant":
        # Held knots at full weight: the band is a hard region rather
        # than a ramp, which is what a mask wants.
        spline.SetKnot(_MakeKnot(0.0, 1.0, Ts.InterpHeld))
        spline.SetKnot(_MakeKnot(1.0, 1.0, Ts.InterpHeld))
    else:
        raise ValueError("unknown falloff profile %r" % (profile,))

    return spline


def EnsureDefaultSpline(attr, profile, force=False):
    """
    Authors a starting spline for the named profile onto a float
    attribute (rigExec:falloffCurve).

    Returns the spline that is now on the attribute.  With force=False
    an existing spline that already has at least two knots is left
    alone, so merely opening the panel never stomps an artist's curve;
    the preset buttons pass force=True because overwriting is the whole
    point of pressing one.
    """
    if attr is None or not attr:
        raise ValueError("EnsureDefaultSpline needs a valid attribute")

    if not force:
        existing = attr.GetSpline()
        if existing and len(existing.GetKnots()) >= 2:
            return existing

    spline = _MakeProfileSpline(profile)
    attr.SetSpline(spline)
    return attr.GetSpline()


def GetSplineKnotPoints(attr):
    """
    Returns the attribute's spline knots as a list of (x, y) tuples
    sorted by x.

    Ts.Spline.GetKnots() returns a KnotMap that iterates as float TIMES,
    not as knot objects, so the times have to be turned back into knots
    one at a time.
    """
    if attr is None or not attr:
        return []

    spline = attr.GetSpline()
    if not spline:
        return []

    points = []
    for time in spline.GetKnots():
        knot = spline.GetKnot(time)
        if knot:
            points.append((float(knot.GetTime()), float(knot.GetValue())))
    points.sort(key=lambda point: point[0])
    return points


def InsertSplineKnot(attr, x, y):
    """
    Inserts a curve-interpolated knot at (x, y), clamped into the unit
    square.  Returns the x the knot actually landed on.
    """
    x = min(max(float(x), 0.0), 1.0)
    y = min(max(float(y), 0.0), 1.0)

    spline = attr.GetSpline()
    if not spline:
        spline = Ts.Spline("float")
    spline.SetKnot(_MakeKnot(x, y))
    attr.SetSpline(spline)
    return x


def MoveSplineKnot(attr, oldX, newX, newY, pinEndpoints=True):
    """
    Moves the knot at oldX to (newX, newY), clamped into the unit
    square.  Returns the x the knot ended up at, or None if there was no
    knot at oldX.

    With pinEndpoints the first and last knots keep their x: they define
    where the band starts and ends, and letting them slide inward would
    silently change what falloffMin/falloffMax mean.  Their y is still
    free, which is how an artist authors a ramp that never reaches full
    strength.
    """
    spline = attr.GetSpline()
    if not spline:
        return None

    knot = spline.GetKnot(float(oldX))
    if not knot:
        return None

    times = sorted(float(t) for t in spline.GetKnots())
    newX = min(max(float(newX), 0.0), 1.0)
    newY = min(max(float(newY), 0.0), 1.0)

    if pinEndpoints and times and float(oldX) in (times[0], times[-1]):
        newX = float(oldX)
    else:
        # Two knots at the same time is not a thing a KnotMap can hold;
        # nudge off any occupied slot rather than silently eating the
        # neighbour.
        for time in times:
            if time != float(oldX) and abs(time - newX) < 1e-6:
                newX = float(oldX)
                break

    if newX != float(oldX):
        spline.RemoveKnot(float(oldX))
        knot.SetTime(newX)
    knot.SetValue(newY)
    spline.SetKnot(knot)
    attr.SetSpline(spline)
    return newX


def DeleteSplineKnot(attr, x, minimumKnots=2):
    """
    Removes the knot at x.  Returns True if it was removed.

    Refuses to go below minimumKnots: a spline with one knot evaluates
    to a constant everywhere and there is no way back to a ramp by
    dragging, so the floor keeps the editor usable.
    """
    spline = attr.GetSpline()
    if not spline:
        return False

    times = sorted(float(t) for t in spline.GetKnots())
    if len(times) <= minimumKnots:
        return False
    if float(x) not in times:
        return False

    spline.RemoveKnot(float(x))
    attr.SetSpline(spline)
    return True


# ---------------------------------------------------------------------
# Prim creation and placement (no Qt)
# ---------------------------------------------------------------------

def FindRigPrim(stage):
    """
    Returns the first RigExecRoot prim on the stage, or None.

    Traversal by typeName rather than by schema type mirrors what
    rigExecUsdview.py already does to decide whether to engage: the
    generated schema may or may not be registered in this process, and
    the type name is authored either way.
    """
    if stage is None:
        return None
    for prim in stage.Traverse():
        if prim.GetTypeName() == "RigExecRoot":
            return prim
    return None


def FindWeightsScopeAncestor(prim):
    """
    Returns the nearest ancestor (or the prim itself) that looks like a
    .../Rig/Weights scope, or None.

    'Looks like' is a name match on Weights with a RigExec-typed
    ancestor: rigs in the wild nest their weights differently, and the
    panel only needs somewhere sensible to hang a new prim.
    """
    if not prim:
        return None

    walker = prim
    while walker and not walker.IsPseudoRoot():
        if walker.GetName() == "Weights":
            ancestor = walker.GetParent()
            while ancestor and not ancestor.IsPseudoRoot():
                if str(ancestor.GetTypeName()).startswith("RigExec"):
                    return walker
                ancestor = ancestor.GetParent()
            return walker
        walker = walker.GetParent()
    return None


def ChooseWeightParentPrim(stage, selectedPrims):
    """
    Decides where a newly created volume weight should live and returns
    that prim, defining it if necessary.

    The order is deliberately "obey the artist first":
      1. a selected RigExecJoint -- weights authored while a joint is
         selected almost always belong to that joint, and parenting them
         there means the joint's transform carries the volume;
      2. a selected prim already inside a .../Rig/Weights scope -- reuse
         the scope the artist is clearly working in;
      3. a Weights scope under the stage's RigExecRoot, created on
         demand;
      4. /Weights as a last resort on a stage with no rig at all.
    """
    for prim in selectedPrims or ():
        if prim and prim.GetTypeName() == "RigExecJoint":
            return prim

    for prim in selectedPrims or ():
        scope = FindWeightsScopeAncestor(prim)
        if scope:
            return scope

    rig = FindRigPrim(stage)
    if rig:
        return stage.DefinePrim(rig.GetPath().AppendChild("Weights"), "Scope")

    return stage.DefinePrim(Sdf.Path("/Weights"), "Scope")


def MakeUniquePrimName(parentPrim, baseName):
    """
    Returns baseName, or baseName with the lowest free integer suffix.
    """
    if not parentPrim or not parentPrim.GetChild(baseName):
        return baseName
    index = 1
    while parentPrim.GetChild("%s%d" % (baseName, index)):
        index += 1
    return "%s%d" % (baseName, index)


def DefaultPrimNameForType(typeName):
    """
    Returns the base prim name used for a newly created weight type,
    e.g. RigExecSphereWeight -> SphereWeight.
    """
    if typeName.startswith("RigExec"):
        return typeName[len("RigExec"):]
    return typeName


def FindPointsPropertyPath(prim):
    """
    Returns the .points property path of a point-based prim, or None.

    rigExec:weightTarget names the exact property being weighted rather
    than the prim, so the field and the points it grabs cannot drift
    apart when a prim carries more than one point set.
    """
    if not prim:
        return None
    if not UsdGeom.PointBased(prim):
        return None
    return prim.GetPath().AppendProperty(UsdGeom.Tokens.points)


def ComputeCentreAndSize(prim, time=None):
    """
    Returns (Gf.Vec3d centre, float size) for a prim: the midpoint of
    its world bound and the largest axis of that bound.

    Falls back to the prim's world-space origin with a unit size when
    the prim has no computable extent, and to the world origin when
    there is no prim at all, so callers never have to special-case an
    empty selection.
    """
    if time is None:
        time = Usd.TimeCode.Default()

    if not prim:
        return Gf.Vec3d(0.0, 0.0, 0.0), 1.0

    # Guides are included so that snapping to an existing weight volume
    # (whose purpose is "guide") lands on the volume, not on the origin.
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide]
    try:
        bboxCache = UsdGeom.BBoxCache(time, purposes)
        bound = bboxCache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not bound.IsEmpty():
            centre = Gf.Vec3d(bound.GetMidpoint())
            extent = bound.GetSize()
            size = max(float(extent[0]), float(extent[1]),
                       float(extent[2]))
            if size > 0.0:
                return centre, size
            return centre, 1.0
    except Tf.ErrorException:
        pass

    try:
        xformCache = UsdGeom.XformCache(time)
        matrix = xformCache.GetLocalToWorldTransform(prim)
        return Gf.Vec3d(matrix.ExtractTranslation()), 1.0
    except Tf.ErrorException:
        return Gf.Vec3d(0.0, 0.0, 0.0), 1.0


def GetWeightTargetPrim(prim):
    """
    Returns the prim that owns the property rigExec:weightTarget points
    at, or None.
    """
    if not prim:
        return None
    relationship = prim.GetRelationship("rigExec:weightTarget")
    if not relationship:
        return None
    stage = prim.GetStage()
    for path in relationship.GetTargets():
        target = stage.GetPrimAtPath(path.GetPrimPath())
        if target:
            return target
    return None


def ComputeScrubScale(prim, time=None):
    """
    Returns the stage-unit scale a scrub on this weight's parameters
    should be measured against.

    It has to be the WEIGHTED GEOMETRY's size, not the weight prim's: a
    volume weight is a UsdGeomBoundable with no authored extent and no
    registered extent computation, so ComputeCentreAndSize on the weight
    itself always falls through to the 1.0 fallback.  Deriving the
    sensitivity from that made every scrub 0.005 units per pixel
    regardless of scene scale -- roughly 4000 pixels of drag to double a
    falloff radius on a character measured in tens of units.
    """
    target = GetWeightTargetPrim(prim)
    if target is not None:
        _, size = ComputeCentreAndSize(target, time)
        if size > 1e-6:
            return float(size)

    _, size = ComputeCentreAndSize(prim, time)
    return float(size)


def SetPlacementFromPoint(prim, centre, time=None):
    """
    Authors avars:tx/ty/tz so a placed volume weight sits at centre.

    These are the animatable placement avars rather than xformOps: a
    RigExecXformable is posed through its avars, and authoring an
    xformOp here would put the volume somewhere the engine does not
    look.

    Authored through SetVisibleAtTime so that snapping a weight whose
    avars the artist has already scrubbed (and so turned into splines)
    actually moves it -- see that function.
    """
    for name, value in (("avars:tx", centre[0]),
                        ("avars:ty", centre[1]),
                        ("avars:tz", centre[2])):
        attr = prim.GetAttribute(name)
        if attr:
            SetVisibleAtTime(attr, float(value), time)


def SeedFalloffForSize(prim, size, time=None):
    """
    Authors inputs:falloffMin/falloffMax proportional to a target's
    bounding box.

    Without this a new sphere weight is a unit radius lost inside a
    character and the artist's first act is always the same corrective
    scrub.  A quarter of the largest bbox axis is a blob that reads
    immediately at character scale.

    A plane weight gets a SIGNED band straddling zero instead, because
    its distance is the signed coordinate along rigExec:planeAxis: the
    useful default there is a gradient across the plane, not a slab
    hugging one side of it.

    Its inputs:extentU/extentV are seeded here too, and have to be: the
    plane guide is sized by the extents alone now (it used to derive a
    half-size from the band), so a new plane on a character measured in
    tens of units would otherwise draw a one-unit square nobody can see.
    Half-extents of size/2 give a square spanning the target's largest
    bbox axis.
    """
    radius = max(float(size) * 0.25, 1e-4)

    minAttr = prim.GetAttribute("inputs:falloffMin")
    maxAttr = prim.GetAttribute("inputs:falloffMax")
    if prim.GetTypeName() == "RigExecPlaneWeight":
        SetVisibleAtTime(minAttr, float(-radius), time)
        SetVisibleAtTime(maxAttr, float(radius), time)
        extent = max(float(size) * 0.5, 1e-4)
        for name in ("inputs:extentU", "inputs:extentV"):
            SetVisibleAtTime(prim.GetAttribute(name), float(extent), time)
    else:
        SetVisibleAtTime(minAttr, 0.0, time)
        SetVisibleAtTime(maxAttr, float(radius), time)
    return radius


def CreateVolumeWeightPrim(stage, typeName, selectedPrims=None, time=None):
    """
    Creates one volume weight prim, wires it to the selection and
    returns it.

    Everything authored here lands in the stage's CURRENT EDIT TARGET,
    which in usdview is the session layer until the artist picks a
    different one -- see the note in VolumeWeightPanel.

    The rules, all of which exist so a freshly created volume is
    immediately useful rather than an invisible unit sphere at the
    origin:
      * parent scope per ChooseWeightParentPrim;
      * uniquely named after its type;
      * rigExec:weightTarget bound to the selected mesh's .points when
        exactly one point-based prim is selected;
      * placed at that prim's bbox centre through the placement avars;
      * falloff seeded from that prim's bbox size.
    """
    if typeName not in VOLUME_WEIGHT_TYPE_NAMES:
        raise ValueError("not a volume weight type: %r" % (typeName,))
    if time is None:
        time = Usd.TimeCode.Default()

    selectedPrims = [p for p in (selectedPrims or ()) if p]

    parent = ChooseWeightParentPrim(stage, selectedPrims)
    name = MakeUniquePrimName(parent, DefaultPrimNameForType(typeName))
    prim = stage.DefinePrim(parent.GetPath().AppendChild(name), typeName)

    # Exactly one point-based prim: unambiguous, so bind it.  Two would
    # be a guess, and a wrong weightTarget is worse than none.
    pointBased = [p for p in selectedPrims if UsdGeom.PointBased(p)]
    targetPrim = pointBased[0] if len(pointBased) == 1 else None

    if targetPrim is not None:
        pointsPath = FindPointsPropertyPath(targetPrim)
        relationship = prim.GetRelationship("rigExec:weightTarget")
        if relationship and pointsPath:
            relationship.SetTargets([pointsPath])

    # A curve weight is useless without a curve; if the artist selected
    # one alongside the mesh, bind it too.
    if typeName == "RigExecCurveWeight":
        curvePrims = [p for p in selectedPrims
                      if p.GetTypeName() == "BasisCurves" and
                      p is not targetPrim]
        if len(curvePrims) == 1:
            curveRel = prim.GetRelationship("rigExec:curve")
            curvePath = FindPointsPropertyPath(curvePrims[0])
            if curveRel and curvePath:
                curveRel.SetTargets([curvePath])

    # RigExecCombineWeight has no placement and no falloff of its own;
    # it only folds what its inputs already computed.
    if typeName in PLACED_WEIGHT_TYPE_NAMES:
        sizeSource = targetPrim if targetPrim is not None else (
            selectedPrims[0] if selectedPrims else None)
        centre, size = ComputeCentreAndSize(sizeSource, time)
        SetPlacementFromPoint(prim, centre, time)
        SeedFalloffForSize(prim, size, time)

    return prim


def SnapWeightToPrim(weightPrim, sourcePrim, time=None):
    """
    Moves a placed volume weight onto another prim's bbox centre.
    Returns the centre it moved to, or None when it could not.
    """
    if not weightPrim or weightPrim.GetTypeName() not in \
            PLACED_WEIGHT_TYPE_NAMES:
        return None
    if not sourcePrim:
        return None

    centre, _ = ComputeCentreAndSize(sourcePrim, time)
    SetPlacementFromPoint(weightPrim, centre, time)
    return centre


# ---------------------------------------------------------------------
# Combine weight input list (no Qt)
# ---------------------------------------------------------------------

def GetInputWeightPaths(prim):
    """
    Returns the ordered rigExec:inputWeights targets as Sdf.Paths.
    """
    if not prim:
        return []
    relationship = prim.GetRelationship("rigExec:inputWeights")
    if not relationship:
        return []
    return list(relationship.GetTargets())


def SetInputWeightPaths(prim, paths):
    """
    Replaces rigExec:inputWeights wholesale.  Order matters for the
    non-commutative combine modes (subtract, overlay), so the list is
    authored as a list rather than merged.
    """
    relationship = prim.GetRelationship("rigExec:inputWeights")
    if not relationship:
        return False
    return relationship.SetTargets([Sdf.Path(p) for p in paths])


def AddInputWeightPaths(prim, paths):
    """
    Appends paths to rigExec:inputWeights, skipping duplicates and the
    combine prim itself.  Returns the number appended.
    """
    current = GetInputWeightPaths(prim)
    existing = set(current)
    added = 0
    for path in paths:
        path = Sdf.Path(path)
        if path in existing or path == prim.GetPath():
            continue
        current.append(path)
        existing.add(path)
        added += 1
    if added:
        SetInputWeightPaths(prim, current)
    return added


def RemoveInputWeightPaths(prim, paths):
    """
    Removes paths from rigExec:inputWeights.  Returns the number
    removed.
    """
    doomed = set(Sdf.Path(p) for p in paths)
    current = GetInputWeightPaths(prim)
    kept = [p for p in current if p not in doomed]
    removed = len(current) - len(kept)
    if removed:
        SetInputWeightPaths(prim, kept)
    return removed


def IsVolumeWeightPrim(prim):
    """True for the four concrete types this panel edits."""
    return bool(prim) and prim.GetTypeName() in VOLUME_WEIGHT_TYPE_NAMES


def IsWeightObjectPrim(prim):
    """True for anything that may feed rigExec:inputWeights."""
    return bool(prim) and prim.GetTypeName() in WEIGHT_OBJECT_TYPE_NAMES


# ---------------------------------------------------------------------
# Qt widgets
# ---------------------------------------------------------------------

def _EventPos(event):
    """
    Returns a QPointF for a mouse event on either PySide binding.

    PySide6 deprecates QMouseEvent.pos() in favour of position(); the
    reference plugin predates that, so normalize here once instead of
    at every call site.
    """
    if hasattr(event, "position"):
        return event.position()
    return QtCore.QPointF(event.pos())


def _DetachWidget(widget):
    """
    Revokes a widget's notice key, if it has one, before Qt destroys it.

    Without this a rebuilt panel leaves live listeners pointing at
    widgets whose C++ half Qt has already deleted, and the next stage
    edit crashes usdview.
    """
    if widget is None:
        return
    detach = getattr(widget, "Detach", None)
    if detach is not None:
        detach()


def _ClearLayout(layout):
    """
    Empties a layout, detaching every widget on the way out.

    QFormLayout is special-cased because takeAt() empties a row but
    leaves the ROW itself behind: a panel that rebuilds on every
    selection change would grow a blank row per parameter per rebuild
    and march off the bottom of the window.  removeRow() is the call
    that actually shrinks the form.
    """
    if isinstance(layout, QtWidgets.QFormLayout):
        for row in reversed(range(layout.rowCount())):
            for role in (QtWidgets.QFormLayout.LabelRole,
                         QtWidgets.QFormLayout.FieldRole,
                         QtWidgets.QFormLayout.SpanningRole):
                item = layout.itemAt(row, role)
                if item is not None:
                    _DetachWidget(item.widget())
            layout.removeRow(row)
        return

    while layout.count():
        item = layout.takeAt(0)
        widget = item.widget()
        if widget is not None:
            _DetachWidget(widget)
            widget.setParent(None)
            widget.deleteLater()
            continue
        child = item.layout()
        if child is not None:
            _ClearLayout(child)


class AttributeValueWidget(QtWidgets.QLineEdit):
    """
    A widget for displaying and authoring the value of a UsdAttribute.

    The widget text is initialized with the current value of the
    attribute.  If the attribute's value type supports munging (int or
    floating point), then the text is editable by entering a value, or
    by munging (clicking inside the text box and dragging the mouse
    horizontally).

    Adapted from Pixar's invertibleRigsExample usdview plugin.  The
    'sensitivity' keyword is the local addition: a falloff radius on a
    character measured in tens of units needs a much coarser scrub than
    inputs:invert, which lives in 0..1 and is unusable at the reference
    plugin's fixed 0.02 per pixel.
    """

    __mungButton = QtCore.Qt.MouseButton.LeftButton

    def __init__(self, usdAttribute, usdviewApi, parent=None,
                 sensitivity=0.02, minimum=None, maximum=None):
        super(AttributeValueWidget, self).__init__(parent)

        self._usdviewApi = usdviewApi
        self._usdAttribute = usdAttribute
        self._sensitivity = float(sensitivity)
        self._minimum = minimum
        self._maximum = maximum
        self._noticeKey = None

        # To enable "mung" behavior, this needs to track the position of
        # the mouse and the initial value when the button is clicked.
        self._mungBaseMousePos = None
        self._mungBaseValue = None

        # Configure the widget for the attribute's value type.
        self._valueTypeName = self._usdAttribute.GetTypeName()
        if self._valueTypeName == Sdf.ValueTypeNames.Int:
            self.setValidator(QtGui.QIntValidator(parent=self))
        elif AttributeValueWidget._IsFloatingPointType(self._valueTypeName):
            self.setValidator(QtGui.QDoubleValidator(parent=self))
        elif (self._valueTypeName == Sdf.ValueTypeNames.Token or
              self._valueTypeName == Sdf.ValueTypeNames.String):
            pass
        else:
            self.setReadOnly(True)

        # Set the initial value of the widget, and maintain a cache of
        # the last text value set on the widget.  This cached value is
        # used to avoid unwanted authoring due to spurious
        # editingFinished signals invoking _onEditingFinished().
        self._cachedText = self._TypedValueToString(
            self._usdAttribute.Get(self._usdviewApi.dataModel.currentFrame))
        self.setText(self._cachedText)

        self._usdviewApi.dataModel.currentFrameChanged.connect(
            self._onCurrentFrameChanged)
        self.editingFinished.connect(self._onEditingFinished)

        self._noticeKey = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self._onObjectsChanged,
            self._usdAttribute.GetStage())

    def Detach(self):
        """
        Revokes the notice key and disconnects the frame signal so this
        widget can be destroyed safely during a panel rebuild.
        """
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        try:
            self._usdviewApi.dataModel.currentFrameChanged.disconnect(
                self._onCurrentFrameChanged)
        except (RuntimeError, TypeError):
            pass

    @staticmethod
    def _IsFloatingPointType(valueTypeName):
        """
        Returns true if valueTypeName is a floating point value type
        (regardless of precision).
        """
        return (valueTypeName == Sdf.ValueTypeNames.Double or
                valueTypeName == Sdf.ValueTypeNames.Float or
                valueTypeName == Sdf.ValueTypeNames.Half)

    @staticmethod
    def _IsNumericType(valueTypeName):
        """
        Returns true if valueTypeName is either an integer or floating
        point type.
        """
        return (AttributeValueWidget._IsFloatingPointType(valueTypeName) or
                valueTypeName == Sdf.ValueTypeNames.Int)

    def _TypedValueToString(self, typedValue):
        """
        Makes sure typed values are displayed nicely in the widget.
        Primarily, rounds floating point values to a few decimals.
        """
        if typedValue is None:
            return ""
        if AttributeValueWidget._IsFloatingPointType(self._valueTypeName):
            return str(round(typedValue, 3))
        return str(typedValue)

    def _StringToTypedValue(self, textValue):
        """
        Attempts to convert a string to a Python type corresponding to
        valueTypeName.  For numeric types, failure results in a zero
        value.  Raises TypeError for unsupported types.
        """
        if self._valueTypeName == Sdf.ValueTypeNames.Int:
            try:
                newValue = int(textValue)
            except ValueError:
                newValue = 0
        elif AttributeValueWidget._IsFloatingPointType(self._valueTypeName):
            try:
                newValue = float(textValue)
            except ValueError:
                newValue = 0.0
        elif (self._valueTypeName == Sdf.ValueTypeNames.Token or
              self._valueTypeName == Sdf.ValueTypeNames.String):
            newValue = textValue
        else:
            raise TypeError(
                "ValueTypeName %s not supported." % self._valueTypeName)

        return newValue

    def _ClampValue(self, value):
        """
        Applies the optional range given at construction.  invert and
        strength are meaningful in 0..1 and a scrub that runs off the
        end is an accident, not an intent.
        """
        if self._minimum is not None:
            value = max(value, self._minimum)
        if self._maximum is not None:
            value = min(value, self._maximum)
        return value

    def _authorValue(self, newValue):
        """
        Authors newValue to the underlying UsdAttribute at the current
        time, in the stage's current edit target.
        """
        currentTime = self._usdviewApi.dataModel.currentFrame
        if AttributeValueWidget._IsNumericType(self._valueTypeName):
            newValue = self._ClampValue(newValue)

        try:
            SetAtTime(self._usdAttribute, newValue, currentTime)
        except Exception as err:
            Tf.Warn("Failed to set <%s> to value: %s : %s"
                    % (self._usdAttribute.GetPath(), newValue, err))

    def _onCurrentFrameChanged(self, frame):
        """
        Handler for changes to currentFrame: re-read and redisplay.
        """
        self._cachedText = self._TypedValueToString(
            self._usdAttribute.Get(frame))
        self.setText(self._cachedText)

    def _onEditingFinished(self):
        """
        When the user is done editing, author the value.
        """
        if self._cachedText == self.text():
            return

        try:
            newValue = self._StringToTypedValue(self.text())
        except TypeError:
            return

        self._authorValue(newValue)

    def _onObjectsChanged(self, notice, stage):
        """
        Handler for Usd's ObjectsChanged notice: re-read and redisplay.
        """
        if not AttributeValueMayHaveChanged(
                self._usdAttribute.GetPath(), notice):
            return

        currentTime = self._usdviewApi.dataModel.currentFrame
        value = self._usdAttribute.Get(currentTime)

        # UsdAttribute.Get() can return None -- no authored value and no
        # schema fallback, or an attribute holding only an empty spline.
        if value is None:
            return

        self._cachedText = self._TypedValueToString(value)
        self.setText(self._cachedText)

    def mousePressEvent(self, event):
        """
        Begins a mung operation.
        """
        if (event.buttons() == self.__mungButton and not self.isReadOnly()
                and self._IsNumericType(self._valueTypeName)):
            self._mungBaseMousePos = _EventPos(event)
            self._mungBaseValue = self._StringToTypedValue(self.text())
            self.setCursor(QtGui.QCursor(QtCore.Qt.SizeHorCursor))
            self.selectAll()
        else:
            super(AttributeValueWidget, self).mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        """
        Ends a mung operation.
        """
        if event.button() == self.__mungButton and not self.isReadOnly():
            self._mungBaseMousePos = None
            self._mungBaseValue = None
            self.setCursor(QtGui.QCursor(QtCore.Qt.IBeamCursor))
        else:
            super(AttributeValueWidget, self).mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        """
        If munging, updates the UsdAttribute value in response to the
        mouse position.
        """
        if (event.buttons() == self.__mungButton and self._mungBaseMousePos
                and self._IsNumericType(self._valueTypeName)):
            delta = _EventPos(event).x() - self._mungBaseMousePos.x()
            delta *= self._sensitivity
            value = self._mungBaseValue + delta
            if self._valueTypeName == Sdf.ValueTypeNames.Int:
                value = int(round(value))
            self._authorValue(value)
            self.selectAll()
        else:
            super(AttributeValueWidget, self).mouseMoveEvent(event)


class AttributeValueSelectWidget(QtWidgets.QComboBox):
    """
    A widget for displaying and authoring the value of a UsdAttribute
    whose value is selected from a list of possible values.

    Adapted from Pixar's invertibleRigsExample usdview plugin; Detach()
    and the valueChanged signal are the local additions.
    """

    # Emitted after a successful author so the panel can react to, say,
    # falloffProfile becoming "curve" without re-reading the stage.
    valueChanged = QtCore.Signal(str)

    def __init__(self, usdAttribute, usdviewApi, parent=None):
        super(AttributeValueSelectWidget, self).__init__(parent)

        self._usdviewApi = usdviewApi
        self._usdAttribute = usdAttribute
        self._noticeKey = None

        if self._usdAttribute is None or not self._usdAttribute:
            self.setEnabled(False)
            return

        if self._usdAttribute.HasMetadata("allowedTokens"):
            self._allowedTokens = [
                str(x) for x in
                self._usdAttribute.GetMetadata("allowedTokens")]
            self.addItems(self._allowedTokens)
        else:
            self._allowedTokens = []
            Tf.Warn("No combo box items for <%s>"
                    % self._usdAttribute.GetPath())

        current = self._usdAttribute.Get(
            self._usdviewApi.dataModel.currentFrame)
        if current is not None:
            self.setCurrentText(str(current))

        self._usdviewApi.dataModel.currentFrameChanged.connect(
            self._onCurrentFrameChanged)
        self.currentIndexChanged.connect(self._onSelectionChanged)

        self._noticeKey = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self._onObjectsChanged,
            self._usdAttribute.GetStage())

    def Detach(self):
        """
        Revokes the notice key and disconnects the frame signal.
        """
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        try:
            self._usdviewApi.dataModel.currentFrameChanged.disconnect(
                self._onCurrentFrameChanged)
        except (RuntimeError, TypeError):
            pass

    def _onCurrentFrameChanged(self, frame):
        """
        Re-read the attribute and update the selected item.  Signals are
        blocked so the resulting currentIndexChanged does not author
        the value straight back.
        """
        value = self._usdAttribute.Get(time=frame)
        if value is None:
            return
        self.blockSignals(True)
        self.setCurrentText(str(value))
        self.blockSignals(False)

    def _onSelectionChanged(self, index):
        """
        When the selection changes, author the value to the attribute.
        """
        if index < 0 or index >= len(self._allowedTokens):
            return
        newValue = self._allowedTokens[index]
        currentTime = self._usdviewApi.dataModel.currentFrame

        try:
            SetAtTime(self._usdAttribute, newValue, currentTime)
        except Exception as err:
            Tf.Warn("Failed to set <%s> to value: %s : %s"
                    % (self._usdAttribute.GetPath(), newValue, err))
            return

        self.valueChanged.emit(newValue)

    def _onObjectsChanged(self, notice, stage):
        """
        Re-read the attribute and update the selected item, blocking
        signals for the same reason as _onCurrentFrameChanged.
        """
        if not AttributeValueMayHaveChanged(
                self._usdAttribute.GetPath(), notice):
            return
        value = self._usdAttribute.Get(
            self._usdviewApi.dataModel.currentFrame)
        if value is None:
            return
        self.blockSignals(True)
        self.setCurrentText(str(value))
        self.blockSignals(False)


class AttributeValueSliderWidget(QtWidgets.QWidget):
    """
    A munging line edit paired with a horizontal slider, for the
    parameters whose USEFUL range is 0..1 (inputs:invert,
    inputs:strength).

    The slider is a convenience, not the mechanism: the line edit is
    still the authority and still scrubs, so this widget behaves
    identically to a bare AttributeValueWidget if the slider is ignored.

    editMinimum/editMaximum are the hard limits the LINE EDIT enforces,
    and they are deliberately separate from the slider's travel.
    inputs:strength is a final multiplier, so 0..1 is the range worth
    giving a slider but not a range the artist may be held to: clamping
    the edit there silently turned a typed 2.5 into 1.0.  Pass None to
    leave that end open.
    """

    __resolution = 1000

    def __init__(self, usdAttribute, usdviewApi, parent=None,
                 minimum=0.0, maximum=1.0,
                 editMinimum=0.0, editMaximum=1.0):
        super(AttributeValueSliderWidget, self).__init__(parent)

        self._usdviewApi = usdviewApi
        self._usdAttribute = usdAttribute
        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._noticeKey = None

        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.setLayout(layout)

        # A 0..1 parameter wants a fine scrub; 400 pixels of drag should
        # cross the whole range rather than eight times over.
        self._edit = AttributeValueWidget(
            usdAttribute, usdviewApi, parent=self,
            sensitivity=(self._maximum - self._minimum) / 400.0,
            minimum=editMinimum, maximum=editMaximum)
        self._edit.setMaximumWidth(70)
        layout.addWidget(self._edit)

        self._slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self._slider.setMinimum(0)
        self._slider.setMaximum(self.__resolution)
        self._slider.setValue(self._ValueToSlider(
            usdAttribute.Get(usdviewApi.dataModel.currentFrame)))
        self._slider.valueChanged.connect(self._onSliderChanged)
        layout.addWidget(self._slider)

        self._noticeKey = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self._onObjectsChanged,
            self._usdAttribute.GetStage())

    def Detach(self):
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        self._edit.Detach()

    def _ValueToSlider(self, value):
        if value is None:
            value = self._minimum
        span = self._maximum - self._minimum
        if span <= 0.0:
            return 0
        fraction = (float(value) - self._minimum) / span
        return int(round(min(max(fraction, 0.0), 1.0) * self.__resolution))

    def _SliderToValue(self, position):
        span = self._maximum - self._minimum
        return self._minimum + span * (float(position) / self.__resolution)

    def _onSliderChanged(self, position):
        value = self._SliderToValue(position)
        try:
            SetAtTime(self._usdAttribute, value,
                      self._usdviewApi.dataModel.currentFrame)
        except Exception as err:
            Tf.Warn("Failed to set <%s> to value: %s : %s"
                    % (self._usdAttribute.GetPath(), value, err))

    def _onObjectsChanged(self, notice, stage):
        if not AttributeValueMayHaveChanged(
                self._usdAttribute.GetPath(), notice):
            return
        value = self._usdAttribute.Get(
            self._usdviewApi.dataModel.currentFrame)
        if value is None:
            return
        self._slider.blockSignals(True)
        self._slider.setValue(self._ValueToSlider(value))
        self._slider.blockSignals(False)


class FalloffCurveWidget(QtWidgets.QWidget):
    """
    A direct-manipulation editor for the Ts spline authored on
    rigExec:falloffCurve.

    The horizontal axis is the NORMALIZED band parameter r: r = 0 is the
    outer end of the band (inputs:falloffMax, where the field is off)
    and r = 1 is the inner end (inputs:falloffMin, where it is on).  The
    vertical axis is the resulting weight in 0..1.  Both ends are
    labelled because the direction is the one thing about a falloff
    curve that is impossible to guess from the picture.

    Interaction:
      * left-drag a knot handle to move it (clamped to the unit square;
        the first and last knots keep their x);
      * left-click empty plot area to insert a knot there;
      * right-click or double-click a knot to delete it, unless only two
        remain.
    """

    __hitRadius = 8.0
    __knotRadius = 4.0
    __sampleCount = 96

    def __init__(self, usdviewApi, usdAttribute=None, parent=None):
        super(FalloffCurveWidget, self).__init__(parent)

        self._usdviewApi = usdviewApi
        self._usdAttribute = None
        self._noticeKey = None
        self._dragKnotX = None

        self.setMinimumHeight(170)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)
        self.setMouseTracking(False)
        self.setToolTip(
            "Left-drag a point to move it, left-click to insert, "
            "right-click a point to delete.")

        self.SetAttribute(usdAttribute)

    # -- binding ------------------------------------------------------

    def SetAttribute(self, usdAttribute):
        """
        Rebinds the editor to a different rigExec:falloffCurve
        attribute (or to None).

        The widget is rebound rather than rebuilt when the selection
        changes so that a drag in progress is the only thing that can be
        interrupted, and so the notice registration churns once per
        selection instead of once per repaint.
        """
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None

        self._usdAttribute = usdAttribute if (
            usdAttribute is not None and usdAttribute) else None
        self._dragKnotX = None

        if self._usdAttribute is not None:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged,
                self._onObjectsChanged,
                self._usdAttribute.GetStage())

        self.update()

    def Detach(self):
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None

    def _onObjectsChanged(self, notice, stage):
        """
        External edits -- another panel, the interpreter, a layer reload
        -- must show up here, so repaint from the stage on any notice
        that touches this attribute.
        """
        if self._usdAttribute is None:
            return
        if AttributeValueMayHaveChanged(
                self._usdAttribute.GetPath(), notice):
            self.update()

    # -- geometry -----------------------------------------------------

    def _PlotRect(self):
        """
        The bordered plot box, inset to leave room for axis labels.
        """
        return QtCore.QRectF(
            36.0, 10.0,
            max(float(self.width()) - 48.0, 1.0),
            max(float(self.height()) - 34.0, 1.0))

    def _ToPixels(self, rect, x, y):
        return QtCore.QPointF(
            rect.left() + float(x) * rect.width(),
            rect.bottom() - float(y) * rect.height())

    def _ToUnit(self, rect, point):
        x = (point.x() - rect.left()) / rect.width()
        y = (rect.bottom() - point.y()) / rect.height()
        return (min(max(x, 0.0), 1.0), min(max(y, 0.0), 1.0))

    def _KnotPoints(self):
        return GetSplineKnotPoints(self._usdAttribute)

    def _HitKnot(self, rect, position):
        """
        Returns the x of the knot under the cursor, or None.
        """
        best = None
        bestDistance = self.__hitRadius
        for x, y in self._KnotPoints():
            pixel = self._ToPixels(rect, x, y)
            distance = ((pixel.x() - position.x()) ** 2 +
                        (pixel.y() - position.y()) ** 2) ** 0.5
            if distance <= bestDistance:
                best = x
                bestDistance = distance
        return best

    # -- painting -----------------------------------------------------

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        rect = self._PlotRect()
        palette = self.palette()
        textColor = palette.color(QtGui.QPalette.WindowText)

        # Plot box.
        painter.fillRect(rect, palette.color(QtGui.QPalette.Base))
        gridPen = QtGui.QPen(QtGui.QColor(textColor.red(), textColor.green(),
                                          textColor.blue(), 48))
        gridPen.setWidth(1)
        painter.setPen(gridPen)
        for step in range(1, 4):
            fraction = step / 4.0
            painter.drawLine(
                self._ToPixels(rect, fraction, 0.0),
                self._ToPixels(rect, fraction, 1.0))
            painter.drawLine(
                self._ToPixels(rect, 0.0, fraction),
                self._ToPixels(rect, 1.0, fraction))

        borderPen = QtGui.QPen(QtGui.QColor(textColor.red(), textColor.green(),
                                            textColor.blue(), 160))
        borderPen.setWidth(1)
        painter.setPen(borderPen)
        painter.drawRect(rect)

        # Axis labels.  r = 0 is the OUTER end of the band, r = 1 the
        # inner end, which is the opposite of what most people assume
        # from a left-to-right ramp, hence the words.
        font = painter.font()
        font.setPointSizeF(max(font.pointSizeF() - 1.0, 6.0))
        painter.setFont(font)
        painter.setPen(QtGui.QPen(textColor))
        painter.drawText(
            QtCore.QRectF(rect.left(), rect.bottom() + 2.0,
                          rect.width() * 0.5, 16.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            "outer (falloffMax)")
        painter.drawText(
            QtCore.QRectF(rect.left() + rect.width() * 0.5,
                          rect.bottom() + 2.0, rect.width() * 0.5, 16.0),
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
            "inner (falloffMin)")
        painter.drawText(
            QtCore.QRectF(0.0, rect.top() - 6.0, 32.0, 14.0),
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, "w=1")
        painter.drawText(
            QtCore.QRectF(0.0, rect.bottom() - 8.0, 32.0, 14.0),
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, "w=0")

        if self._usdAttribute is None:
            painter.setPen(QtGui.QPen(
                QtGui.QColor(textColor.red(), textColor.green(),
                             textColor.blue(), 140)))
            painter.drawText(rect, QtCore.Qt.AlignCenter,
                             "no falloff curve attribute")
            painter.end()
            return

        spline = self._usdAttribute.GetSpline()
        knots = self._KnotPoints()
        if not spline or not knots:
            painter.setPen(QtGui.QPen(
                QtGui.QColor(textColor.red(), textColor.green(),
                             textColor.blue(), 140)))
            painter.drawText(rect, QtCore.Qt.AlignCenter,
                             "no spline authored -- pick a preset below")
            painter.end()
            return

        # The curve itself, sampled densely enough that a bezier segment
        # reads as a curve rather than as a chord.
        path = QtGui.QPainterPath()
        for index in range(self.__sampleCount + 1):
            x = float(index) / self.__sampleCount
            y = float(spline.Eval(x))
            point = self._ToPixels(rect, x, min(max(y, 0.0), 1.0))
            if index == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)

        curvePen = QtGui.QPen(QtGui.QColor(230, 90, 70))
        curvePen.setWidthF(2.0)
        painter.setPen(curvePen)
        painter.drawPath(path)

        # Knot handles.
        painter.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20)))
        for x, y in knots:
            active = (self._dragKnotX is not None and
                      abs(x - self._dragKnotX) < 1e-9)
            painter.setBrush(QtGui.QBrush(
                QtGui.QColor(255, 220, 90) if active
                else QtGui.QColor(250, 250, 250)))
            painter.drawEllipse(
                self._ToPixels(rect, x, min(max(y, 0.0), 1.0)),
                self.__knotRadius, self.__knotRadius)

        painter.end()

    # -- interaction --------------------------------------------------

    def mousePressEvent(self, event):
        if self._usdAttribute is None:
            return

        rect = self._PlotRect()
        position = _EventPos(event)
        knotX = self._HitKnot(rect, position)

        if event.button() == QtCore.Qt.RightButton:
            if knotX is not None:
                if not DeleteSplineKnot(self._usdAttribute, knotX):
                    Tf.Warn("Refusing to delete: a falloff curve needs at "
                            "least two knots.")
                self.update()
            return

        if event.button() != QtCore.Qt.LeftButton:
            super(FalloffCurveWidget, self).mousePressEvent(event)
            return

        if knotX is not None:
            self._dragKnotX = knotX
            self.update()
            return

        if not rect.contains(position):
            return

        x, y = self._ToUnit(rect, position)
        self._dragKnotX = InsertSplineKnot(self._usdAttribute, x, y)
        self.update()

    def mouseMoveEvent(self, event):
        if self._usdAttribute is None or self._dragKnotX is None:
            return

        rect = self._PlotRect()
        x, y = self._ToUnit(rect, _EventPos(event))
        # Authoring live during the drag is what makes the viewport
        # overlay follow the curve; the edit target absorbs the churn.
        newX = MoveSplineKnot(self._usdAttribute, self._dragKnotX, x, y)
        if newX is not None:
            self._dragKnotX = newX
        self.update()

    def mouseReleaseEvent(self, event):
        if self._dragKnotX is None:
            return
        if self._usdAttribute is not None:
            rect = self._PlotRect()
            x, y = self._ToUnit(rect, _EventPos(event))
            MoveSplineKnot(self._usdAttribute, self._dragKnotX, x, y)
        self._dragKnotX = None
        self.update()

    def mouseDoubleClickEvent(self, event):
        if self._usdAttribute is None:
            return
        rect = self._PlotRect()
        knotX = self._HitKnot(rect, _EventPos(event))
        if knotX is None:
            return
        if not DeleteSplineKnot(self._usdAttribute, knotX):
            Tf.Warn("Refusing to delete: a falloff curve needs at least "
                    "two knots.")
        self._dragKnotX = None
        self.update()


class VolumeWeightPanel(QtWidgets.QWidget):
    """
    The volumetric weight authoring window.

    One instance per usdview session; it follows the prim selection and
    rebuilds its parameter, placement and combine sections for whichever
    volume weight is selected.
    """

    __instance = None

    @classmethod
    def GetInstance(cls, usdviewApi, setWeightOverlay=None,
                    hasWeightOverlay=None):
        """
        Returns the single panel for this session, creating it once.
        """
        if cls.__instance is None:
            cls.__instance = VolumeWeightPanel(
                usdviewApi, setWeightOverlay, hasWeightOverlay)
        return cls.__instance

    def __init__(self, usdviewApi, setWeightOverlay=None,
                 hasWeightOverlay=None):
        super(VolumeWeightPanel, self).__init__(
            usdviewApi.qMainWindow, QtCore.Qt.WindowType.Window)

        self._usdviewApi = usdviewApi
        self._setWeightOverlay = setWeightOverlay
        self._hasWeightOverlay = hasWeightOverlay
        self._currentPrim = None

        self.setWindowTitle("RigExec: Volume Weights")
        self.setGeometry(0, 0, 460, 760)

        self._BuildUI()

        # The prim selection lives on the selection data model, and the
        # signal it emits is signalPrimSelectionChanged(added, removed)
        # -- see Usdviewq/selectionDataModel.py.
        self._usdviewApi.dataModel.selection.signalPrimSelectionChanged \
            .connect(self._onPrimSelectionChanged)
        self._usdviewApi.dataModel.signalStageReplaced.connect(
            self._onStageReplaced)

        self._RebuildForSelection()

    # -- construction -------------------------------------------------

    def _BuildUI(self):
        outer = QtWidgets.QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)
        self.setLayout(outer)

        scrollArea = QtWidgets.QScrollArea()
        scrollArea.setWidgetResizable(True)
        outer.addWidget(scrollArea)

        body = QtWidgets.QWidget()
        scrollArea.setWidget(body)

        self._rootLayout = QtWidgets.QVBoxLayout()
        self._rootLayout.setContentsMargins(10, 10, 10, 10)
        self._rootLayout.setSpacing(8)
        body.setLayout(self._rootLayout)

        self._BuildEditTargetLabel()
        self._BuildCreateSection()
        self._BuildSelectionLabel()
        self._BuildParametersSection()
        self._BuildPlacementSection()
        self._BuildFalloffSection()
        self._BuildCombineSection()
        self._BuildOverlaySection()
        self._rootLayout.addStretch(1)

    def _BuildEditTargetLabel(self):
        # EDIT TARGET DISCIPLINE: every Set/SetSpline/SetTargets in this
        # panel goes through the plain Usd authoring API, which always
        # writes into stage.GetEditTarget().  usdview starts with the
        # SESSION layer as its edit target, so by default nothing this
        # panel authors survives closing the app -- it is a scratch pad.
        # Switching the edit target to the root layer (usdview's Edit
        # Target menu, or the Layer Stack view) is what makes weight
        # objects persist.  The label spells out which one is live right
        # now so nobody loses an afternoon of volume placement.
        self._editTargetLabel = QtWidgets.QLabel()
        self._editTargetLabel.setWordWrap(True)
        self._editTargetLabel.setToolTip(
            "All authoring goes into the stage's current edit target. "
            "usdview defaults to the session layer, which is discarded "
            "when the app closes; switch to the root layer to persist.")
        self._rootLayout.addWidget(self._editTargetLabel)

    def _BuildCreateSection(self):
        group = QtWidgets.QGroupBox("Create")
        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        group.setLayout(layout)

        for typeName in VOLUME_WEIGHT_TYPE_NAMES:
            label = DefaultPrimNameForType(typeName).replace("Weight", "")
            button = QtWidgets.QPushButton(label)
            button.setToolTip("Create a %s" % typeName)
            button.clicked.connect(
                lambda checked=False, name=typeName: self._onCreate(name))
            layout.addWidget(button)

        self._rootLayout.addWidget(group)

    def _BuildSelectionLabel(self):
        self._selectionLabel = QtWidgets.QLabel()
        self._selectionLabel.setWordWrap(True)
        self._rootLayout.addWidget(self._selectionLabel)

    def _BuildParametersSection(self):
        self._parametersGroup = QtWidgets.QGroupBox("Parameters")
        self._parametersLayout = QtWidgets.QFormLayout()
        self._parametersLayout.setLabelAlignment(
            QtCore.Qt.AlignmentFlag.AlignRight)
        self._parametersLayout.setFieldGrowthPolicy(
            QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        self._parametersLayout.setContentsMargins(10, 10, 10, 10)
        self._parametersGroup.setLayout(self._parametersLayout)
        self._rootLayout.addWidget(self._parametersGroup)

    def _BuildPlacementSection(self):
        self._placementGroup = QtWidgets.QGroupBox("Placement")
        placementOuter = QtWidgets.QVBoxLayout()
        placementOuter.setContentsMargins(10, 10, 10, 10)
        self._placementGroup.setLayout(placementOuter)

        self._placementLayout = QtWidgets.QFormLayout()
        self._placementLayout.setLabelAlignment(
            QtCore.Qt.AlignmentFlag.AlignRight)
        self._placementLayout.setFieldGrowthPolicy(
            QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        placementOuter.addLayout(self._placementLayout)

        self._snapButton = QtWidgets.QPushButton("Snap to selection")
        self._snapButton.setToolTip(
            "Move this volume to the bounding-box centre of the other "
            "selected prim.")
        self._snapButton.clicked.connect(self._onSnapToSelection)
        placementOuter.addWidget(self._snapButton)

        self._rootLayout.addWidget(self._placementGroup)

    def _BuildFalloffSection(self):
        self._falloffGroup = QtWidgets.QGroupBox("Falloff curve")
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        self._falloffGroup.setLayout(layout)

        presetRow = QtWidgets.QHBoxLayout()
        presetRow.addWidget(QtWidgets.QLabel("Preset:"))
        self._presetButtons = []
        for profile in SPLINE_PROFILE_NAMES:
            button = QtWidgets.QPushButton(profile)
            button.setToolTip(
                "Overwrite rigExec:falloffCurve with the %s shape and "
                "switch rigExec:falloffProfile to 'curve'." % profile)
            button.clicked.connect(
                lambda checked=False, name=profile: self._onPreset(name))
            presetRow.addWidget(button)
            self._presetButtons.append(button)
        layout.addLayout(presetRow)

        self._curveWidget = FalloffCurveWidget(self._usdviewApi, None, self)
        layout.addWidget(self._curveWidget)

        self._curveHint = QtWidgets.QLabel()
        self._curveHint.setWordWrap(True)
        layout.addWidget(self._curveHint)

        self._rootLayout.addWidget(self._falloffGroup)

    def _BuildCombineSection(self):
        self._combineGroup = QtWidgets.QGroupBox("Combine inputs")
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        self._combineGroup.setLayout(layout)

        self._combineModeLayout = QtWidgets.QFormLayout()
        self._combineModeLayout.setLabelAlignment(
            QtCore.Qt.AlignmentFlag.AlignRight)
        layout.addLayout(self._combineModeLayout)

        self._inputWeightsList = QtWidgets.QListWidget()
        self._inputWeightsList.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection)
        self._inputWeightsList.setMinimumHeight(90)
        self._inputWeightsList.setToolTip(
            "Order matters for subtract and overlay.")
        layout.addWidget(self._inputWeightsList)

        buttonRow = QtWidgets.QHBoxLayout()
        addButton = QtWidgets.QPushButton("Add from selection")
        addButton.clicked.connect(self._onAddInputWeights)
        buttonRow.addWidget(addButton)
        removeButton = QtWidgets.QPushButton("Remove")
        removeButton.clicked.connect(self._onRemoveInputWeights)
        buttonRow.addWidget(removeButton)
        layout.addLayout(buttonRow)

        self._rootLayout.addWidget(self._combineGroup)

    def _BuildOverlaySection(self):
        group = QtWidgets.QGroupBox("Influence overlay")
        layout = QtWidgets.QVBoxLayout()
        group.setLayout(layout)

        self._overlayCheckBox = QtWidgets.QCheckBox("Show influence")
        self._overlayCheckBox.setToolTip(
            "Paint the chosen weight object's influence onto its target "
            "geometry as a red gradient in the viewport.")
        self._overlayCheckBox.stateChanged.connect(self._onOverlayToggled)
        layout.addWidget(self._overlayCheckBox)

        # An EXPLICIT target, not the tree selection.
        #
        # The first cut drove the overlay from the selection, which meant
        # the checkbox sat disabled until you happened to select a weight
        # object -- and a disabled checkbox that explains nothing reads
        # exactly like a broken one. It also fought the actual workflow:
        # you want to watch an influence while dragging the JOINT.
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Painting:"))
        self._overlayTargetCombo = QtWidgets.QComboBox()
        self._overlayTargetCombo.setToolTip(
            "Which weight object's field to paint. Independent of the "
            "viewport selection, so the influence stays visible while "
            "you drag the joint that drives it.")
        self._overlayTargetCombo.currentIndexChanged.connect(
            self._onOverlayTargetChanged)
        row.addWidget(self._overlayTargetCombo, 1)
        layout.addLayout(row)

        self._overlayStatusLabel = QtWidgets.QLabel("")
        self._overlayStatusLabel.setWordWrap(True)
        layout.addWidget(self._overlayStatusLabel)

        self._rootLayout.addWidget(group)

    def _OverlayTargetPath(self):
        """The prim path the overlay should paint, or '' for none."""
        if self._overlayTargetCombo.count() == 0:
            return ""
        return str(self._overlayTargetCombo.currentData() or "")

    def _RefreshOverlayTargets(self, preferred=None):
        """
        Repopulates the target list, keeping the current choice when it
        survives and otherwise preferring \\p preferred (normally the
        selected weight object, so selecting one still does the
        convenient thing without the overlay DEPENDING on it).
        """
        combo = self._overlayTargetCombo
        previous = self._OverlayTargetPath()
        weights = self._WeightObjectsOnStage()

        combo.blockSignals(True)
        combo.clear()
        for prim in weights:
            path = str(prim.GetPath())
            combo.addItem("%s  (%s)" % (prim.GetName(), prim.GetTypeName()),
                          path)
        wanted = None
        if previous and previous in [str(p.GetPath()) for p in weights]:
            wanted = previous
        elif preferred is not None:
            wanted = str(preferred.GetPath())
        if wanted:
            index = combo.findData(wanted)
            if index >= 0:
                combo.setCurrentIndex(index)
        combo.blockSignals(False)

    # -- rebuild ------------------------------------------------------

    def _CurrentWeightPrim(self):
        """
        Returns the first selected prim that is a volume weight, or
        None.

        The stage check is not defensive noise: signalStageReplaced fires
        with dataModel.stage already None when usdview tears a stage down,
        and usdviewApi.selectedPrims walks that null stage
        (selectionDataModel.py:711 -> stage.GetPrimAtPath). Without this
        the panel raises on every stage close.
        """
        if self._usdviewApi.dataModel.stage is None:
            return None
        for prim in self._usdviewApi.selectedPrims:
            if IsVolumeWeightPrim(prim):
                return prim
        return None

    def _WeightObjectsOnStage(self):
        """
        Every weight object on the stage, in namespace order.

        The overlay target list is drawn from this rather than from the
        selection because a rigger wants to look at an influence while
        selecting and dragging something ELSE -- the joint, the control,
        the mesh. Tying the overlay to the tree selection makes the one
        thing you want to watch disappear the moment you touch anything.
        """
        stage = self._usdviewApi.dataModel.stage
        if stage is None:
            return []
        return [prim for prim in stage.TraverseAll()
                if IsWeightObjectPrim(prim)]

    def _onStageReplaced(self):
        self._RebuildForSelection()

    def _onPrimSelectionChanged(self, added, removed):
        """
        Handler for
        selectionDataModel.signalPrimSelectionChanged(added, removed).
        """
        self._RebuildForSelection()

    def _RebuildForSelection(self):
        stage = self._usdviewApi.dataModel.stage
        if stage is not None:
            layer = stage.GetEditTarget().GetLayer()
            isSession = layer == stage.GetSessionLayer()
            self._editTargetLabel.setText(
                "Edit target: %s%s"
                % (layer.GetDisplayName() or layer.identifier,
                   "  (session layer -- not saved with the stage)"
                   if isSession else ""))
        else:
            self._editTargetLabel.setText("Edit target: no stage")

        prim = self._CurrentWeightPrim()
        self._currentPrim = prim

        _ClearLayout(self._parametersLayout)
        _ClearLayout(self._placementLayout)
        _ClearLayout(self._combineModeLayout)
        self._inputWeightsList.clear()

        if prim is None:
            self._selectionLabel.setText(
                "Select a volume weight prim to edit it, or create one "
                "above.")
            self._parametersGroup.setEnabled(False)
            self._placementGroup.setEnabled(False)
            self._falloffGroup.setEnabled(False)
            self._combineGroup.setVisible(False)
            self._curveWidget.SetAttribute(None)
            self._SyncOverlayCheckBox(None)
            return

        self._selectionLabel.setText(
            "Editing <b>%s</b> (%s)" % (prim.GetPath(), prim.GetTypeName()))
        self._parametersGroup.setEnabled(True)

        self._PopulateParameters(prim)
        self._PopulatePlacement(prim)
        self._PopulateFalloff(prim)
        self._PopulateCombine(prim)
        self._SyncOverlayCheckBox(prim)

    def _AddAttributeRow(self, layout, prim, attrName, label=None,
                         sensitivity=0.02, normalized=False,
                         editMaximum=1.0):
        """
        Adds one attribute row, choosing the widget from the attribute's
        type: a combo for tokens with allowedTokens, a slider+edit for
        the 0..1 parameters, a munging edit for everything else.

        Returns the widget, or None when the prim does not have the
        attribute -- which is how scaleX/Y/Z and planeAxis disappear on
        the types that lack them without any per-type branching here.
        """
        attr = prim.GetAttribute(attrName)
        if not attr:
            return None

        if (attr.GetTypeName() == Sdf.ValueTypeNames.Token and
                attr.HasMetadata("allowedTokens")):
            widget = AttributeValueSelectWidget(attr, self._usdviewApi)
        elif normalized:
            widget = AttributeValueSliderWidget(
                attr, self._usdviewApi, editMaximum=editMaximum)
        else:
            widget = AttributeValueWidget(
                attr, self._usdviewApi, sensitivity=sensitivity)

        layout.addRow(label or attrName.split(":")[-1], widget)
        return widget

    def _PopulateParameters(self, prim):
        # A falloff radius is measured in stage units and may be tens or
        # hundreds; the scale factors and the normalized parameters are
        # not.  Sensitivity is per-row for exactly that reason.
        size = ComputeScrubScale(prim, self._usdviewApi.frame)
        radiusSensitivity = max(float(size), 1.0) / 200.0

        self._AddAttributeRow(
            self._parametersLayout, prim, "inputs:falloffMin",
            "falloffMin", sensitivity=radiusSensitivity)
        self._AddAttributeRow(
            self._parametersLayout, prim, "inputs:falloffMax",
            "falloffMax", sensitivity=radiusSensitivity)
        self._AddAttributeRow(
            self._parametersLayout, prim, "inputs:invert", "invert",
            normalized=True)
        # strength is a final multiplier, not a normalized parameter: the
        # slider still travels 0..1 because that is the useful range, but
        # the edit is left open above it so an artist can over-drive.
        self._AddAttributeRow(
            self._parametersLayout, prim, "inputs:strength", "strength",
            normalized=True, editMaximum=None)
        for axis in ("X", "Y", "Z"):
            self._AddAttributeRow(
                self._parametersLayout, prim, "inputs:scale%s" % axis,
                "scale%s" % axis, sensitivity=0.005)
        self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:planeAxis", "planeAxis")
        self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:planeBounds",
            "planeBounds")
        # The extents are stage-unit distances like the falloff band, not
        # the 0..1 factors scaleX/Y/Z are, so they scrub at the band's
        # sensitivity rather than the scale factors'.  All three rows drop
        # themselves on the types that lack the attribute.
        for name in ("U", "V"):
            self._AddAttributeRow(
                self._parametersLayout, prim, "inputs:extent%s" % name,
                "extent%s" % name, sensitivity=radiusSensitivity)

        profileWidget = self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:falloffProfile",
            "falloffProfile")
        if isinstance(profileWidget, AttributeValueSelectWidget):
            # The curve editor is only meaningful when the profile
            # actually reads the spline, so track the token live.
            profileWidget.valueChanged.connect(self._onProfileChanged)

        self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:samplePhase",
            "samplePhase")
        self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:rangePolicy",
            "rangePolicy")
        self._AddAttributeRow(
            self._parametersLayout, prim, "guide:drawMode", "guide:drawMode")

        targets = []
        relationship = prim.GetRelationship("rigExec:weightTarget")
        if relationship:
            targets = [str(p) for p in relationship.GetTargets()]
        targetLabel = QtWidgets.QLabel(
            ", ".join(targets) if targets else "<not bound>")
        targetLabel.setWordWrap(True)
        self._parametersLayout.addRow("weightTarget", targetLabel)

    def _PopulatePlacement(self, prim):
        placed = prim.GetTypeName() in PLACED_WEIGHT_TYPE_NAMES
        self._placementGroup.setEnabled(placed)
        if not placed:
            return

        size = ComputeScrubScale(prim, self._usdviewApi.frame)
        translateSensitivity = max(float(size), 1.0) / 200.0

        for axis in ("tx", "ty", "tz"):
            self._AddAttributeRow(
                self._placementLayout, prim, "avars:%s" % axis, axis,
                sensitivity=translateSensitivity)
        # Rotations are in degrees, so a quarter degree per pixel gives
        # a full turn in a comfortable drag.
        for axis in ("rx", "ry", "rz"):
            self._AddAttributeRow(
                self._placementLayout, prim, "avars:%s" % axis, axis,
                sensitivity=0.25)

    def _PopulateFalloff(self, prim):
        attr = prim.GetAttribute("rigExec:falloffCurve")
        self._falloffGroup.setEnabled(bool(attr))
        self._curveWidget.SetAttribute(attr if attr else None)

        if not attr:
            self._curveHint.setText(
                "%s has no falloff curve." % prim.GetTypeName())
            self._curveWidget.setEnabled(False)
            return

        profileAttr = prim.GetAttribute("rigExec:falloffProfile")
        profile = profileAttr.Get() if profileAttr else None
        self._SetCurveEnabledForProfile(profile)

    def _SetCurveEnabledForProfile(self, profile):
        isCurve = (profile == "curve")
        self._curveWidget.setEnabled(isCurve)
        if isCurve:
            self._curveHint.setText(
                "rigExec:falloffProfile is 'curve': this spline is what "
                "the field evaluates.")
        else:
            self._curveHint.setText(
                "rigExec:falloffProfile is '%s': the analytic profile is "
                "in use and this spline is ignored. Press a preset to "
                "bake it into the curve and switch."
                % (profile if profile else "unset",))

    def _PopulateCombine(self, prim):
        isCombine = prim.GetTypeName() == "RigExecCombineWeight"
        self._combineGroup.setVisible(isCombine)
        if not isCombine:
            return

        self._AddAttributeRow(
            self._combineModeLayout, prim, "rigExec:combineMode",
            "combineMode")

        for path in GetInputWeightPaths(prim):
            self._inputWeightsList.addItem(str(path))

    # -- actions ------------------------------------------------------

    def _onCreate(self, typeName):
        stage = self._usdviewApi.dataModel.stage
        if stage is None:
            Tf.Warn("No stage: cannot create %s" % typeName)
            return

        try:
            prim = CreateVolumeWeightPrim(
                stage, typeName, list(self._usdviewApi.selectedPrims),
                self._usdviewApi.frame)
        except Exception as err:
            Tf.Warn("Failed to create %s: %s" % (typeName, err))
            return

        # Selecting the new prim is what makes the panel show it, and it
        # also puts it under the artist's cursor in the prim browser.
        self._usdviewApi.ClearPrimSelection()
        self._usdviewApi.AddPrimToSelection(prim)
        self._usdviewApi.UpdateGUI()
        self._RebuildForSelection()

    def _onSnapToSelection(self):
        prim = self._currentPrim
        if prim is None:
            return

        source = None
        for candidate in self._usdviewApi.selectedPrims:
            if candidate and candidate.GetPath() != prim.GetPath():
                source = candidate
                break
        if source is None:
            Tf.Warn("Snap needs a second prim selected alongside the "
                    "volume weight.")
            return

        SnapWeightToPrim(prim, source, self._usdviewApi.frame)
        self._RebuildForSelection()

    def _onPreset(self, profile):
        prim = self._currentPrim
        if prim is None:
            return

        attr = prim.GetAttribute("rigExec:falloffCurve")
        if not attr:
            return

        try:
            EnsureDefaultSpline(attr, profile, force=True)
        except Exception as err:
            Tf.Warn("Failed to author %s spline on <%s>: %s"
                    % (profile, attr.GetPath(), err))
            return

        # Baking a shape the field would ignore is a trap, so the preset
        # buttons also flip the profile token to the one that reads it.
        profileAttr = prim.GetAttribute("rigExec:falloffProfile")
        if profileAttr:
            profileAttr.Set("curve")

        self._RebuildForSelection()

    def _onProfileChanged(self, value):
        self._SetCurveEnabledForProfile(value)

    def _onAddInputWeights(self):
        prim = self._currentPrim
        if prim is None or prim.GetTypeName() != "RigExecCombineWeight":
            return

        paths = [p.GetPath() for p in self._usdviewApi.selectedPrims
                 if IsWeightObjectPrim(p) and p.GetPath() != prim.GetPath()]
        if not paths:
            Tf.Warn("Select one or more weight objects to add.")
            return

        AddInputWeightPaths(prim, paths)
        self._RebuildForSelection()

    def _onRemoveInputWeights(self):
        prim = self._currentPrim
        if prim is None or prim.GetTypeName() != "RigExecCombineWeight":
            return

        paths = [item.text() for item in
                 self._inputWeightsList.selectedItems()]
        if not paths:
            return

        RemoveInputWeightPaths(prim, paths)
        self._RebuildForSelection()

    # -- viewport overlay ---------------------------------------------

    def _SyncOverlayCheckBox(self, prim):
        """
        Enables the overlay checkbox only when a weight is selected AND
        the imaging library actually exports the entry point.
        """
        available = True
        if self._hasWeightOverlay is not None:
            try:
                available = bool(self._hasWeightOverlay())
            except Exception:
                available = False
        elif self._setWeightOverlay is None:
            available = False

        self._RefreshOverlayTargets(preferred=prim)
        hasTarget = self._overlayTargetCombo.count() > 0

        self._overlayCheckBox.blockSignals(True)
        self._overlayCheckBox.setEnabled(available and hasTarget)
        if not available:
            self._overlayCheckBox.setChecked(False)
        self._overlayCheckBox.blockSignals(False)
        self._overlayTargetCombo.setEnabled(available and hasTarget)

        # Whatever state it is in, SAY SO. A disabled checkbox with no
        # explanation is indistinguishable from a broken one, which is
        # precisely how this landed the first time.
        if not available:
            self._overlayStatusLabel.setText(
                "rigExecImaging does not export "
                "RigExecImaging_SetWeightOverlay -- rebuild the imaging "
                "library to enable the overlay.")
        elif not hasTarget:
            self._overlayStatusLabel.setText(
                "No weight objects on this stage yet. Create one above.")
        else:
            self._overlayStatusLabel.setText(
                "A weight object is only painted once a MOVER consumes it "
                "-- an unbound weight has no resolved field to show.")

        # A checked box whose target changed must repoint rather than
        # keep painting the old one.
        if available and self._overlayCheckBox.isChecked():
            self._ApplyOverlay()

    def _ApplyOverlay(self):
        """Pushes the current on/off + target state to the imaging library."""
        if self._setWeightOverlay is None:
            return
        path = (self._OverlayTargetPath()
                if self._overlayCheckBox.isChecked() else "")
        try:
            self._setWeightOverlay(path)
        except Exception as err:
            Tf.Warn("RigExecImaging_SetWeightOverlay failed: %s" % err)
            return
        # The scene index change does not itself schedule a repaint.
        self._usdviewApi.UpdateViewport()

    def _onOverlayToggled(self, state):
        self._ApplyOverlay()

    def _onOverlayTargetChanged(self, index):
        if self._overlayCheckBox.isChecked():
            self._ApplyOverlay()


def OpenVolumeWeightPanel(usdviewApi, setWeightOverlay=None,
                          hasWeightOverlay=None):
    """
    Command-plugin entry point: shows the single panel instance.
    """
    panel = VolumeWeightPanel.GetInstance(
        usdviewApi, setWeightOverlay, hasWeightOverlay)
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
