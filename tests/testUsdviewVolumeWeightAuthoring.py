#
# Headless verification for the volume weight authoring UI.
#
# Everything the usdview panel does to the stage lives in module-level
# functions in volumeWeightUI so it can be exercised without a display,
# without a QApplication and without constructing a single QWidget --
# which is exactly what this test does. The Qt import is still made
# (volumeWeightUI is a UI module and imports it at module scope), so a
# broken PySide install is reported here as a clear message rather than
# as an opaque traceback out of usdview.
#
# Run with:
#   set PYTHONPATH=D:\work\usdRig\usd-install\lib\site-packages
#   set PATH=D:\work\usdRig\usd-install\bin;D:\work\usdRig\usd-install\lib;%PATH%
#   python testUsdviewVolumeWeightAuthoring.py
#
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PLUGIN_DIR = os.path.normpath(
    os.path.join(_HERE, "..", "plugin", "rigExecUsdview"))
_SCHEMA_RESOURCES = os.path.normpath(
    os.path.join(_HERE, "..", "plugin", "rigExecSchema", "resources"))

sys.path.insert(0, _PLUGIN_DIR)

from pxr import Gf, Plug, Sdf, Ts, Usd, UsdGeom


def _ImportUI():
    """
    Imports volumeWeightUI, turning a headless Qt failure into a
    diagnosis instead of a stack trace.
    """
    try:
        import volumeWeightUI
    except ImportError as error:
        raise AssertionError(
            "could not import volumeWeightUI from %s: %s\n"
            "If this names PySide, the usdview Qt binding is missing "
            "from this interpreter; the pure authoring helpers cannot "
            "be tested separately because they live in the same module."
            % (_PLUGIN_DIR, error))
    return volumeWeightUI


def _RegisterSchema():
    plugins = Plug.Registry().RegisterPlugins(_SCHEMA_RESOURCES)
    if not plugins:
        raise AssertionError(
            "no plugin registered from %s" % _SCHEMA_RESOURCES)
    if not Usd.SchemaRegistry().IsConcrete("RigExecSphereWeight"):
        raise AssertionError(
            "RigExecSphereWeight is not a registered concrete schema")
    return plugins


def _MakeStage():
    """
    A minimal rig: a body mesh spanning 8 x 4 x 4 units and a
    RigExecRoot with one joint, which is enough to drive every branch of
    the creation rules.
    """
    stage = Usd.Stage.CreateInMemory()

    mesh = UsdGeom.Mesh.Define(stage, "/Asset/Geom/Body")
    mesh.CreatePointsAttr([
        Gf.Vec3f(-4.0, -2.0, -2.0), Gf.Vec3f(4.0, -2.0, -2.0),
        Gf.Vec3f(4.0, 2.0, 2.0), Gf.Vec3f(-4.0, 2.0, 2.0)])
    mesh.CreateExtentAttr([
        Gf.Vec3f(-4.0, -2.0, -2.0), Gf.Vec3f(4.0, 2.0, 2.0)])

    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Joints/Shoulder", "RigExecJoint")

    return stage, mesh.GetPrim()


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tolerance=1e-4):
    return abs(float(a) - float(b)) <= tolerance


# ---------------------------------------------------------------------

def TestPrimCreation(ui):
    stage, meshPrim = _MakeStage()

    created = {}
    for typeName in ui.VOLUME_WEIGHT_TYPE_NAMES:
        prim = ui.CreateVolumeWeightPrim(
            stage, typeName, [meshPrim], Usd.TimeCode.Default())
        _Check(bool(prim), "%s was not created" % typeName)
        _Check(prim.GetTypeName() == typeName,
               "%s came back as %s" % (typeName, prim.GetTypeName()))
        created[typeName] = prim
        print("  created %-24s at %s" % (typeName, prim.GetPath()))

    # Rule: the parent scope is a Weights scope under the RigExecRoot
    # when nothing rig-ish is selected.
    for typeName, prim in created.items():
        _Check(str(prim.GetPath()).startswith("/Asset/Rig/Weights/"),
               "%s landed outside the rig's Weights scope: %s"
               % (typeName, prim.GetPath()))

    # Rule: the single selected point-based prim becomes weightTarget.
    for typeName, prim in created.items():
        targets = prim.GetRelationship("rigExec:weightTarget").GetTargets()
        _Check(targets == [Sdf.Path("/Asset/Geom/Body.points")],
               "%s weightTarget is %s" % (typeName, targets))
    print("  weightTarget bound to /Asset/Geom/Body.points on all 4 types")

    # Rule: unique naming.
    again = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim], Usd.TimeCode.Default())
    _Check(again.GetName() == "SphereWeight1",
           "second sphere weight is named %s" % again.GetName())
    third = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim], Usd.TimeCode.Default())
    _Check(third.GetName() == "SphereWeight2",
           "third sphere weight is named %s" % third.GetName())
    print("  unique naming: %s, %s, %s"
          % (created["RigExecSphereWeight"].GetName(),
             again.GetName(), third.GetName()))

    # Rule: placed at the target's bbox centre. The mesh spans
    # -4..4 / -2..2 / -2..2, so the centre is the origin; shift the mesh
    # and check the placement follows.
    UsdGeom.Xformable(meshPrim).AddTranslateOp().Set(Gf.Vec3d(10.0, 5.0, 1.0))
    moved = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim], Usd.TimeCode.Default())
    placement = tuple(moved.GetAttribute("avars:t%s" % axis).Get()
                      for axis in "xyz")
    _Check(all(_Close(a, b) for a, b in zip(placement, (10.0, 5.0, 1.0))),
           "placement avars are %s, expected (10, 5, 1)" % (placement,))
    print("  placement avars follow the target bbox centre: %s"
          % (placement,))

    # Rule: the falloff is seeded from the bbox, not left at the unit
    # schema fallback. Largest axis is 8, so the radius is 2.
    falloffMax = moved.GetAttribute("inputs:falloffMax").Get()
    _Check(_Close(falloffMax, 2.0),
           "sphere falloffMax seeded to %s, expected 2.0" % falloffMax)
    falloffMin = moved.GetAttribute("inputs:falloffMin").Get()
    _Check(_Close(falloffMin, 0.0),
           "sphere falloffMin seeded to %s, expected 0.0" % falloffMin)

    plane = created["RigExecPlaneWeight"]
    planeMin = plane.GetAttribute("inputs:falloffMin").Get()
    planeMax = plane.GetAttribute("inputs:falloffMax").Get()
    _Check(_Close(planeMin, -2.0) and _Close(planeMax, 2.0),
           "plane band is (%s, %s), expected (-2, 2)" % (planeMin, planeMax))
    print("  falloff seeded from bbox: sphere 0..%s, plane %s..%s"
          % (falloffMax, planeMin, planeMax))

    # A plane's drawn size now comes from the extents alone -- the guide
    # no longer derives a half-size from the band -- so a new plane whose
    # extents were left at the unit schema fallback would be an
    # invisible speck on anything bigger than a unit cube. Largest bbox
    # axis is 8, so the half-extents are 4 and the square spans it.
    planeU = plane.GetAttribute("inputs:extentU").Get()
    planeV = plane.GetAttribute("inputs:extentV").Get()
    _Check(_Close(planeU, 4.0) and _Close(planeV, 4.0),
           "plane extents are (%s, %s), expected (4, 4)" % (planeU, planeV))
    _Check(plane.GetAttribute("rigExec:planeBounds").Get() == "unbounded",
           "a new plane weight is born %s, expected unbounded"
           % plane.GetAttribute("rigExec:planeBounds").Get())
    print("  plane extents seeded from bbox: %s x %s, bounds=%s"
          % (planeU, planeV,
             plane.GetAttribute("rigExec:planeBounds").Get()))

    # Rule: a selected RigExecJoint wins as the parent scope.
    joint = stage.GetPrimAtPath("/Asset/Rig/Joints/Shoulder")
    onJoint = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [joint, meshPrim],
        Usd.TimeCode.Default())
    _Check(onJoint.GetPath().GetParentPath() == joint.GetPath(),
           "joint-parented weight landed at %s" % onJoint.GetPath())
    print("  selected joint used as parent: %s" % onJoint.GetPath())

    # Rule: a prim already inside a Weights scope reuses that scope.
    inScope = ui.CreateVolumeWeightPrim(
        stage, "RigExecPlaneWeight", [created["RigExecSphereWeight"]],
        Usd.TimeCode.Default())
    _Check(str(inScope.GetPath()).startswith("/Asset/Rig/Weights/"),
           "scope reuse landed at %s" % inScope.GetPath())
    print("  existing Weights scope reused: %s" % inScope.GetPath())

    # Combine weight has no placement avars at all (it inherits
    # RigExecWeightObject, not RigExecXformable).
    combine = created["RigExecCombineWeight"]
    _Check(not combine.GetAttribute("avars:tx"),
           "RigExecCombineWeight unexpectedly has avars:tx")
    _Check(not combine.GetAttribute("rigExec:falloffCurve"),
           "RigExecCombineWeight unexpectedly has rigExec:falloffCurve")
    print("  RigExecCombineWeight correctly has no placement or falloff")

    return stage, created


def TestWeightTargetAuthoring(ui):
    stage, meshPrim = _MakeStage()
    prim = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [], Usd.TimeCode.Default())

    targets = prim.GetRelationship("rigExec:weightTarget").GetTargets()
    _Check(targets == [],
           "weightTarget bound with nothing selected: %s" % targets)

    pointsPath = ui.FindPointsPropertyPath(meshPrim)
    _Check(pointsPath == Sdf.Path("/Asset/Geom/Body.points"),
           "FindPointsPropertyPath returned %s" % pointsPath)
    _Check(ui.FindPointsPropertyPath(prim) is None,
           "FindPointsPropertyPath accepted a non point-based prim")

    prim.GetRelationship("rigExec:weightTarget").SetTargets([pointsPath])
    targets = prim.GetRelationship("rigExec:weightTarget").GetTargets()
    _Check(targets == [pointsPath],
           "weightTarget round trip gave %s" % targets)
    print("  weightTarget set/read round trip: %s" % targets[0])

    # Two point-based prims selected is ambiguous, so nothing is bound.
    other = UsdGeom.Mesh.Define(stage, "/Asset/Geom/Head").GetPrim()
    ambiguous = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim, other],
        Usd.TimeCode.Default())
    targets = ambiguous.GetRelationship("rigExec:weightTarget").GetTargets()
    _Check(targets == [],
           "ambiguous selection bound weightTarget to %s" % targets)
    print("  two meshes selected leaves weightTarget unbound")


def TestDefaultSplines(ui):
    stage, meshPrim = _MakeStage()
    prim = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim], Usd.TimeCode.Default())
    attr = prim.GetAttribute("rigExec:falloffCurve")
    _Check(bool(attr), "no rigExec:falloffCurve attribute")

    samples = (0.0, 0.25, 0.5, 0.75, 1.0)
    values = {}

    for profile in ui.SPLINE_PROFILE_NAMES:
        spline = ui.EnsureDefaultSpline(attr, profile, force=True)
        _Check(bool(spline), "%s authored no spline" % profile)
        _Check(attr.HasSpline(),
               "%s did not land on the attribute" % profile)
        knots = ui.GetSplineKnotPoints(attr)
        _Check(len(knots) == 2,
               "%s authored %d knots, expected 2" % (profile, len(knots)))
        evaluated = [float(spline.Eval(x)) for x in samples]
        values[profile] = evaluated
        print("  %-9s knots=%s eval=%s"
              % (profile,
                 [(round(x, 3), round(y, 3)) for x, y in knots],
                 [round(v, 4) for v in evaluated]))

    # Endpoints are pinned for every ramp profile.
    for profile in ("linear", "smooth", "easeIn", "easeOut"):
        evaluated = values[profile]
        _Check(_Close(evaluated[0], 0.0),
               "%s starts at %s, expected 0" % (profile, evaluated[0]))
        _Check(_Close(evaluated[-1], 1.0),
               "%s ends at %s, expected 1" % (profile, evaluated[-1]))

    # Every ramp profile is monotone non-decreasing: a falloff that dips
    # would make the field non-monotone in distance, which no artist
    # expects from a named profile.
    for profile in ("linear", "smooth", "easeIn", "easeOut"):
        evaluated = values[profile]
        for index in range(1, len(evaluated)):
            _Check(evaluated[index] >= evaluated[index - 1] - 1e-6,
                   "%s is not monotone: %s" % (profile, evaluated))

    # linear is the diagonal.
    for x, value in zip(samples, values["linear"]):
        _Check(_Close(value, x),
               "linear(%s) = %s" % (x, value))

    # smooth is symmetric about the midpoint and passes through 0.5.
    smooth = values["smooth"]
    _Check(_Close(smooth[2], 0.5),
           "smooth(0.5) = %s, expected 0.5" % smooth[2])
    _Check(smooth[1] < 0.25, "smooth(0.25) = %s, expected < 0.25" % smooth[1])
    _Check(smooth[3] > 0.75, "smooth(0.75) = %s, expected > 0.75" % smooth[3])
    _Check(_Close(smooth[1], 1.0 - smooth[3], 1e-3),
           "smooth is not symmetric: %s vs %s" % (smooth[1], smooth[3]))

    # easeIn hugs the floor: strictly below linear in the interior.
    for x, value in zip(samples[1:-1], values["easeIn"][1:-1]):
        _Check(value < x - 1e-3,
               "easeIn(%s) = %s, expected below linear" % (x, value))

    # easeOut is its mirror: strictly above linear in the interior.
    for x, value in zip(samples[1:-1], values["easeOut"][1:-1]):
        _Check(value > x + 1e-3,
               "easeOut(%s) = %s, expected above linear" % (x, value))

    # constant is a held pair at full weight, so every sample is 1.
    for x, value in zip(samples, values["constant"]):
        _Check(_Close(value, 1.0),
               "constant(%s) = %s, expected 1.0" % (x, value))
    constantKnots = ui.GetSplineKnotPoints(attr)
    heldSpline = attr.GetSpline()
    for time in heldSpline.GetKnots():
        knot = heldSpline.GetKnot(time)
        _Check(knot.GetNextInterpolation() == Ts.InterpHeld,
               "constant knot at %s is not held" % time)
    print("  constant knots are Ts.InterpHeld: %s" % (constantKnots,))

    # EnsureDefaultSpline without force must not stomp an existing
    # curve.
    ui.EnsureDefaultSpline(attr, "linear", force=True)
    ui.EnsureDefaultSpline(attr, "constant", force=False)
    preserved = [round(float(attr.GetSpline().Eval(x)), 4) for x in samples]
    _Check(preserved == [0.0, 0.25, 0.5, 0.75, 1.0],
           "force=False overwrote the existing spline: %s" % preserved)
    print("  force=False preserves an authored spline: %s" % preserved)

    # And the authored scene description really is a spline.
    ui.EnsureDefaultSpline(attr, "smooth", force=True)
    authored = stage.GetRootLayer().ExportToString()
    _Check("rigExec:falloffCurve.spline" in authored,
           "no spline in the exported layer:\n%s" % authored)
    print("  authored scene description carries "
          "'rigExec:falloffCurve.spline'")


def TestKnotRoundTrip(ui):
    stage, meshPrim = _MakeStage()
    prim = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim], Usd.TimeCode.Default())
    attr = prim.GetAttribute("rigExec:falloffCurve")
    ui.EnsureDefaultSpline(attr, "linear", force=True)

    _Check(ui.GetSplineKnotPoints(attr) == [(0.0, 0.0), (1.0, 1.0)],
           "linear did not start at the diagonal: %s"
           % ui.GetSplineKnotPoints(attr))

    # Insert.
    x = ui.InsertSplineKnot(attr, 0.5, 0.9)
    _Check(_Close(x, 0.5), "insert returned %s" % x)
    knots = ui.GetSplineKnotPoints(attr)
    _Check(len(knots) == 3, "insert produced %d knots" % len(knots))
    _Check(_Close(knots[1][0], 0.5) and _Close(knots[1][1], 0.9),
           "inserted knot is %s" % (knots[1],))
    _Check(_Close(attr.GetSpline().Eval(0.5), 0.9),
           "Eval(0.5) = %s after insert" % attr.GetSpline().Eval(0.5))
    print("  insert: %s" % (knots,))

    # Insert clamps into the unit square.
    clamped = ui.InsertSplineKnot(attr, 1.7, -0.4)
    _Check(_Close(clamped, 1.0),
           "insert past the right edge landed at %s" % clamped)
    _Check(_Close(ui.GetSplineKnotPoints(attr)[-1][1], 0.0),
           "insert below the floor was not clamped: %s"
           % (ui.GetSplineKnotPoints(attr)[-1],))
    # Put the endpoint back where linear had it.
    ui.MoveSplineKnot(attr, 1.0, 1.0, 1.0)
    print("  insert clamps out-of-range coordinates into [0, 1]")

    # Move an interior knot.
    moved = ui.MoveSplineKnot(attr, 0.5, 0.3, 0.2)
    _Check(_Close(moved, 0.3), "move returned %s" % moved)
    knots = ui.GetSplineKnotPoints(attr)
    _Check(_Close(knots[1][0], 0.3) and _Close(knots[1][1], 0.2),
           "moved knot is %s" % (knots[1],))
    _Check(_Close(attr.GetSpline().Eval(0.3), 0.2),
           "Eval(0.3) = %s after move" % attr.GetSpline().Eval(0.3))
    print("  move: %s" % (knots,))

    # Move clamps into the unit square.
    ui.MoveSplineKnot(attr, 0.3, -2.0, 5.0)
    knots = ui.GetSplineKnotPoints(attr)
    _Check(_Close(knots[0][0], 0.0) and _Close(knots[1][1], 1.0),
           "move did not clamp: %s" % (knots,))
    ui.MoveSplineKnot(attr, knots[1][0], 0.4, 0.6)
    print("  move clamps out-of-range coordinates into [0, 1]")

    # The first and last knots keep their x.
    knots = ui.GetSplineKnotPoints(attr)
    firstX, lastX = knots[0][0], knots[-1][0]
    ui.MoveSplineKnot(attr, firstX, 0.42, 0.3)
    ui.MoveSplineKnot(attr, lastX, 0.58, 0.7)
    knots = ui.GetSplineKnotPoints(attr)
    _Check(_Close(knots[0][0], 0.0) and _Close(knots[-1][0], 1.0),
           "endpoint x moved off 0/1: %s" % (knots,))
    _Check(_Close(knots[0][1], 0.3) and _Close(knots[-1][1], 0.7),
           "endpoint y did not move: %s" % (knots,))
    print("  endpoints pinned in x, free in y: %s" % (knots,))

    # Moving a knot that does not exist is a no-op, not a crash.
    _Check(ui.MoveSplineKnot(attr, 0.123456, 0.5, 0.5) is None,
           "move of a nonexistent knot did not return None")

    # Delete.
    interiorX = ui.GetSplineKnotPoints(attr)[1][0]
    _Check(ui.DeleteSplineKnot(attr, interiorX),
           "delete of the interior knot failed")
    knots = ui.GetSplineKnotPoints(attr)
    _Check(len(knots) == 2, "delete left %d knots" % len(knots))
    print("  delete: %s" % (knots,))

    # And the floor: two knots is the minimum.
    _Check(not ui.DeleteSplineKnot(attr, knots[0][0]),
           "delete went below two knots")
    _Check(len(ui.GetSplineKnotPoints(attr)) == 2,
           "the refused delete still removed a knot")
    print("  delete refused at the two-knot floor")


def TestCombineInputs(ui):
    stage, meshPrim = _MakeStage()
    sphere = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim], Usd.TimeCode.Default())
    plane = ui.CreateVolumeWeightPrim(
        stage, "RigExecPlaneWeight", [meshPrim], Usd.TimeCode.Default())
    combine = ui.CreateVolumeWeightPrim(
        stage, "RigExecCombineWeight", [meshPrim], Usd.TimeCode.Default())

    _Check(ui.GetInputWeightPaths(combine) == [],
           "a new combine weight already has inputs")

    added = ui.AddInputWeightPaths(
        combine, [sphere.GetPath(), plane.GetPath()])
    _Check(added == 2, "added %d input weights, expected 2" % added)
    paths = ui.GetInputWeightPaths(combine)
    _Check(paths == [sphere.GetPath(), plane.GetPath()],
           "input weights are %s" % paths)
    print("  inputWeights: %s" % [str(p) for p in paths])

    # Duplicates and self-reference are refused.
    added = ui.AddInputWeightPaths(
        combine, [sphere.GetPath(), combine.GetPath()])
    _Check(added == 0, "duplicate/self add appended %d" % added)
    _Check(ui.GetInputWeightPaths(combine) == paths,
           "duplicate/self add changed the list")
    print("  duplicate and self-reference adds refused")

    removed = ui.RemoveInputWeightPaths(combine, [sphere.GetPath()])
    _Check(removed == 1, "removed %d, expected 1" % removed)
    _Check(ui.GetInputWeightPaths(combine) == [plane.GetPath()],
           "after removal: %s" % ui.GetInputWeightPaths(combine))
    print("  removal leaves: %s"
          % [str(p) for p in ui.GetInputWeightPaths(combine)])

    _Check(ui.IsWeightObjectPrim(sphere) and ui.IsWeightObjectPrim(combine),
           "IsWeightObjectPrim rejected a weight object")
    _Check(not ui.IsWeightObjectPrim(meshPrim),
           "IsWeightObjectPrim accepted a mesh")
    _Check(ui.IsVolumeWeightPrim(plane) and not
           ui.IsVolumeWeightPrim(meshPrim),
           "IsVolumeWeightPrim misclassified")


def TestSnapAndEditTarget(ui):
    stage, meshPrim = _MakeStage()
    sphere = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [], Usd.TimeCode.Default())

    centre = ui.SnapWeightToPrim(sphere, meshPrim, Usd.TimeCode.Default())
    _Check(centre is not None, "snap returned None")
    placement = tuple(sphere.GetAttribute("avars:t%s" % axis).Get()
                      for axis in "xyz")
    _Check(all(_Close(a, b) for a, b in zip(placement, (0.0, 0.0, 0.0))),
           "snap put the weight at %s" % (placement,))
    print("  snap to /Asset/Geom/Body centre: %s" % (placement,))

    combine = ui.CreateVolumeWeightPrim(
        stage, "RigExecCombineWeight", [], Usd.TimeCode.Default())
    _Check(ui.SnapWeightToPrim(combine, meshPrim) is None,
           "snap accepted a combine weight, which has no placement")
    print("  snap refuses RigExecCombineWeight")

    # EDIT TARGET: the panel authors through the plain Usd API, so
    # everything must land in whatever layer the stage's edit target
    # names. Prove it by pointing the edit target at the session layer.
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    sessionWeight = ui.CreateVolumeWeightPrim(
        stage, "RigExecPlaneWeight", [meshPrim], Usd.TimeCode.Default())
    ui.EnsureDefaultSpline(
        sessionWeight.GetAttribute("rigExec:falloffCurve"), "smooth",
        force=True)

    sessionText = stage.GetSessionLayer().ExportToString()
    rootText = stage.GetRootLayer().ExportToString()
    _Check(sessionWeight.GetName() in sessionText,
           "session-targeted authoring did not reach the session layer")
    _Check(sessionWeight.GetName() not in rootText,
           "session-targeted authoring leaked into the root layer")
    print("  authoring follows stage.GetEditTarget() into the session "
          "layer")
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))


def TestSetAtTimeUniformGuard(ui):
    stage, meshPrim = _MakeStage()
    prim = ui.CreateVolumeWeightPrim(
        stage, "RigExecPlaneWeight", [meshPrim], Usd.TimeCode.Default())

    # rigExec:planeAxis is a uniform token: authoring it at a numeric
    # frame must go to the default, because uniform attributes cannot
    # carry time samples.
    axisAttr = prim.GetAttribute("rigExec:planeAxis")
    _Check(axisAttr.GetVariability() == Sdf.VariabilityUniform,
           "rigExec:planeAxis is not uniform")
    ui.SetAtTime(axisAttr, "z", Usd.TimeCode(12.0))
    _Check(axisAttr.Get() == "z",
           "uniform token authored as %s" % axisAttr.Get())
    _Check(axisAttr.GetNumTimeSamples() == 0,
           "uniform token grew %d time samples"
           % axisAttr.GetNumTimeSamples())
    print("  SetAtTime routes uniform tokens to the default: planeAxis=%s"
          % axisAttr.Get())

    # A varying float at a numeric frame becomes a spline knot.
    strengthAttr = prim.GetAttribute("inputs:strength")
    ui.SetAtTime(strengthAttr, 0.25, Usd.TimeCode(7.0))
    _Check(strengthAttr.HasSpline(),
           "inputs:strength did not become a spline")
    _Check(_Close(strengthAttr.Get(Usd.TimeCode(7.0)), 0.25),
           "inputs:strength at frame 7 is %s"
           % strengthAttr.Get(Usd.TimeCode(7.0)))
    print("  SetAtTime writes a varying float as a spline knot: "
          "strength@7 = %s" % strengthAttr.Get(Usd.TimeCode(7.0)))

    # And at the default time it is a plain default, not a knot.
    other = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim], Usd.TimeCode.Default())
    invertAttr = other.GetAttribute("inputs:invert")
    ui.SetAtTime(invertAttr, 0.75, Usd.TimeCode.Default())
    _Check(not invertAttr.HasSpline(),
           "default-time authoring created a spline")
    _Check(_Close(invertAttr.Get(), 0.75),
           "inputs:invert default is %s" % invertAttr.Get())
    print("  SetAtTime at the default time writes a default: invert=%s"
          % invertAttr.Get())


def TestAuthoringAtUsdviewsRealFrame(ui):
    """
    usdview's currentFrame is NEVER Usd.TimeCode.Default().

    appController.py:1409 sets dataModel.currentFrame = Usd.TimeCode(0.0)
    for a stage with no time samples, and to the first time sample
    otherwise. Every widget edit in the panel therefore reaches
    SetAtTime at a NUMERIC time, which turns the touched attribute into
    a spline -- and a spline outranks a default in value resolution.

    Anything that later authors a default on the same attribute is then
    invisible. That is exactly what "Snap to selection" used to do.
    """
    frame = Usd.TimeCode(0.0)
    stage, meshPrim = _MakeStage()
    sphere = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [], frame)

    tx = sphere.GetAttribute("avars:tx")

    # A freshly created weight must NOT be born animated: creation at a
    # numeric frame still writes plain defaults.
    _Check(not tx.HasSpline(),
           "creation at frame 0 gave avars:tx a spline")
    print("  creation at frame 0 writes plain defaults, not knots")

    # Now the artist scrubs, which is what AttributeValueWidget does.
    ui.SetAtTime(tx, 99.0, frame)
    _Check(tx.HasSpline(), "the scrub did not produce a spline")
    _Check(_Close(tx.Get(frame), 99.0),
           "avars:tx after the scrub is %s" % tx.Get(frame))

    # ...and then presses Snap. It must actually move the volume.
    centre = ui.SnapWeightToPrim(sphere, meshPrim, frame)
    _Check(centre is not None, "snap returned None")
    resolved = tuple(sphere.GetAttribute("avars:t%s" % axis).Get(frame)
                     for axis in "xyz")
    _Check(all(_Close(a, b) for a, b in zip(resolved, (0.0, 0.0, 0.0))),
           "snap did not take effect at frame 0: avars resolve to %s "
           "(the default is masked by the scrubbed spline)" % (resolved,))
    print("  snap after a scrub takes effect at frame 0: %s" % (resolved,))

    # Same hazard on the falloff band.
    falloffMax = sphere.GetAttribute("inputs:falloffMax")
    ui.SetAtTime(falloffMax, 42.0, frame)
    ui.SeedFalloffForSize(sphere, 8.0, frame)
    _Check(_Close(falloffMax.Get(frame), 2.0),
           "reseeding the falloff at frame 0 resolved to %s, expected 2.0"
           % falloffMax.Get(frame))
    print("  reseeding the falloff after a scrub resolves to %s"
          % falloffMax.Get(frame))

    # SetVisibleAtTime leaves an untouched attribute as a plain default
    # rather than promoting it to a one-knot animation curve.
    clean = ui.CreateVolumeWeightPrim(
        stage, "RigExecPlaneWeight", [], frame)
    ui.SetVisibleAtTime(clean.GetAttribute("avars:ty"), 3.0, frame)
    _Check(not clean.GetAttribute("avars:ty").HasSpline(),
           "SetVisibleAtTime promoted a clean default to a spline")
    _Check(_Close(clean.GetAttribute("avars:ty").Get(frame), 3.0),
           "SetVisibleAtTime wrote %s"
           % clean.GetAttribute("avars:ty").Get(frame))
    print("  SetVisibleAtTime keeps a clean attribute on the default")


def TestScrubScale(ui):
    """
    The mouse-drag sensitivity has to be measured against the WEIGHTED
    GEOMETRY, not against the weight prim.

    A volume weight is a UsdGeomBoundable with no authored extent and no
    registered extent computation, so its own world bound is always
    empty and ComputeCentreAndSize falls through to the 1.0 fallback.
    Deriving sensitivity from that pins every scrub at 0.005 units per
    pixel at any scene scale.
    """
    frame = Usd.TimeCode(0.0)
    stage = Usd.Stage.CreateInMemory()
    mesh = UsdGeom.Mesh.Define(stage, "/Asset/Geom/Body")
    mesh.CreatePointsAttr([
        Gf.Vec3f(-40.0, -20.0, -20.0), Gf.Vec3f(40.0, 20.0, 20.0)])
    mesh.CreateExtentAttr([
        Gf.Vec3f(-40.0, -20.0, -20.0), Gf.Vec3f(40.0, 20.0, 20.0)])
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    meshPrim = mesh.GetPrim()

    bound = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [meshPrim], frame)

    target = ui.GetWeightTargetPrim(bound)
    _Check(target is not None and target.GetPath() == meshPrim.GetPath(),
           "GetWeightTargetPrim returned %s" % target)

    ownSize = ui.ComputeCentreAndSize(bound, frame)[1]
    _Check(_Close(ownSize, 1.0),
           "the weight prim's own bound is %s, not the 1.0 fallback -- "
           "this test's premise no longer holds" % ownSize)

    scale = ui.ComputeScrubScale(bound, frame)
    _Check(_Close(scale, 80.0),
           "ComputeScrubScale gave %s, expected the target's 80.0" % scale)
    print("  scrub scale follows the weightTarget: %s (own bound %s)"
          % (scale, ownSize))
    print("  sensitivity: %s per pixel, not %s"
          % (max(scale, 1.0) / 200.0, max(ownSize, 1.0) / 200.0))

    # An unbound weight must degrade gracefully, not raise.
    unbound = ui.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [], frame)
    _Check(ui.GetWeightTargetPrim(unbound) is None,
           "an unbound weight reported a target prim")
    _Check(_Close(ui.ComputeScrubScale(unbound, frame), 1.0),
           "unbound scrub scale is %s" % ui.ComputeScrubScale(unbound, frame))
    print("  unbound weight falls back to 1.0 without raising")


def main():
    print("plugin dir : %s" % _PLUGIN_DIR)
    print("schema dir : %s" % _SCHEMA_RESOURCES)

    _RegisterSchema()
    print("schema     : registered, RigExecSphereWeight is concrete")

    ui = _ImportUI()
    print("module     : imported %s" % ui.__file__)
    print("qt binding : %s" % ui.QtCore.__name__)

    # Nothing below constructs a QWidget: the panel is a driver over
    # these functions, and this is the proof that they stand alone.
    _Check(ui.QtWidgets.QApplication.instance() is None,
           "a QApplication was created; the helpers are not Qt-free")

    tests = [
        ("prim creation", TestPrimCreation),
        ("weightTarget authoring", TestWeightTargetAuthoring),
        ("default splines", TestDefaultSplines),
        ("knot insert/move/delete round trip", TestKnotRoundTrip),
        ("combine inputs", TestCombineInputs),
        ("snap and edit target", TestSnapAndEditTarget),
        ("SetAtTime variability", TestSetAtTimeUniformGuard),
        ("authoring at usdview's real frame", TestAuthoringAtUsdviewsRealFrame),
        ("scrub scale", TestScrubScale),
    ]

    for name, test in tests:
        print("")
        print("[%s]" % name)
        test(ui)

    print("")
    _Check(ui.QtWidgets.QApplication.instance() is None,
           "a QApplication appeared during the run")
    print("RIGEXEC_VOLUME_WEIGHT_AUTHORING_OK (%d groups, no QWidget "
          "constructed)" % len(tests))
    return 0


if __name__ == "__main__":
    sys.exit(main())
