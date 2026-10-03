#
# Headless verification for the curvenet authoring UI.
#
# Every stage edit the usdview panel makes lives in a module-level function in
# curvenetUI, so all of it is exercisable without a display, a QApplication or
# a single QWidget -- which is what this does. The Qt import still happens
# (curvenetUI is a UI module and imports it at module scope), so a broken
# PySide install is reported here as a clear message rather than as an opaque
# traceback out of usdview.
#
# Run with:
#   set PYTHONPATH=C:\path\to\usd-install\lib\site-packages
#   set PATH=C:\path\to\usd-install\bin;C:\path\to\usd-install\lib;%PATH%
#   python testUsdviewCurvenetAuthoring.py
#
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PLUGIN_DIR = os.path.normpath(
    os.path.join(_HERE, "..", "plugin", "rigExecUsdview"))
_SCHEMA_RESOURCES = os.path.normpath(
    os.path.join(_HERE, "..", "plugin", "rigExecSchema", "resources"))

sys.path.insert(0, _PLUGIN_DIR)

from pxr import Gf, Plug, Sdf, Usd, UsdGeom, Vt


failures = []


def Check(condition, message):
    if not condition:
        failures.append(message)
        print("FAIL: %s" % message)


def _ImportUI():
    try:
        import curvenetUI
    except ImportError as error:
        raise AssertionError(
            "could not import curvenetUI from %s: %s\n"
            "If this names PySide, the usdview Qt binding is missing from "
            "this interpreter." % (_PLUGIN_DIR, error))
    return curvenetUI


def _RegisterSchema():
    Plug.Registry().RegisterPlugins(_SCHEMA_RESOURCES)
    if not Usd.SchemaRegistry().IsConcrete("RigExecCurvenet"):
        raise AssertionError(
            "RigExecCurvenet is not a registered concrete schema; run "
            "gen_schema.bat")


def _MakeStage():
    """An asset with a rig and a flat grid to draw on."""
    stage = Usd.Stage.CreateInMemory()
    asset = UsdGeom.Xform.Define(stage, "/Asset")
    stage.SetDefaultPrim(asset.GetPrim())
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")

    mesh = UsdGeom.Mesh.Define(stage, "/Asset/Geom/Grid")
    points, counts, indices = [], [], []
    n = 6
    for y in range(n + 1):
        for x in range(n + 1):
            points.append(Gf.Vec3f(float(x), 0.0, float(y)))
    for y in range(n):
        for x in range(n):
            a = y * (n + 1) + x
            counts.append(4)
            indices.extend([a, a + 1, a + n + 2, a + n + 1])
    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    normals = mesh.CreateNormalsAttr(
        Vt.Vec3fArray([Gf.Vec3f(0, 1, 0)] * len(points)))
    normals.SetMetadata("interpolation", "vertex")
    return stage, mesh.GetPrim()


def TestCreateAndDraw(ui, stage):
    parent = stage.GetPrimAtPath("/Asset/Geom")
    net = ui.CreateCurvenet(stage, parent)
    Check(net.GetTypeName() == "RigExecCurvenet",
          "created prim is a RigExecCurvenet")
    Check(net.IsA(UsdGeom.PointBased),
          "a curvenet must be a UsdGeomPointBased so ordinary movers can "
          "pose its knots")

    # Draw a plus: a centre knot with four spokes, welded at the centre.
    centre = ui.AppendKnot(net, Gf.Vec3f(3, 0, 3))
    tips = [ui.AppendKnot(net, Gf.Vec3f(*p))
            for p in [(6, 0, 3), (3, 0, 6), (0, 0, 3), (3, 0, 0)]]
    for tip in tips:
        ui.AddSpline(net, centre, tip)

    topology = ui.Topology(len(ui.GetPoints(net)), ui.GetSplines(net))
    Check(len(topology.splines) == 4, "four splines authored")
    Check(topology.kind[centre] == ui.KIND_INTERSECTION,
          "a knot shared by four splines is an intersection, got %s"
          % topology.kind[centre])
    for tip in tips:
        Check(topology.kind[tip] == ui.KIND_ANCHOR,
              "an endpoint on one spline is an anchor")
    Check(len(topology.curves) == 4,
          "the plus decomposes into four curves, got %d"
          % len(topology.curves))
    Check(not topology.IsolatedCurves(),
          "no curve of a plus is isolated: each has an intersection end")
    return net, centre, tips


def TestWeldMakesIntersections(ui, stage):
    """Two separate splines become one net only when their ends are welded."""
    net = ui.CreateCurvenet(stage, stage.GetPrimAtPath("/Asset/Geom"),
                            "WeldNet")
    a0 = ui.AppendKnot(net, Gf.Vec3f(0, 0, 0))
    a1 = ui.AppendKnot(net, Gf.Vec3f(2, 0, 0))
    b0 = ui.AppendKnot(net, Gf.Vec3f(2.0001, 0, 0))
    b1 = ui.AppendKnot(net, Gf.Vec3f(4, 0, 0))
    ui.AddSpline(net, a0, a1)
    ui.AddSpline(net, b0, b1)

    topology = ui.Topology(len(ui.GetPoints(net)), ui.GetSplines(net))
    Check(len(topology.curves) == 2, "unwelded: two separate curves")
    Check(len(topology.IsolatedCurves()) == 2,
          "unwelded curves are isolated, so section 3 gives them a "
          "rotation-only gradient")

    ui.WeldKnots(net, a1, b0)
    topology = ui.Topology(len(ui.GetPoints(net)), ui.GetSplines(net))
    Check(topology.kind[a1] == ui.KIND_INTERIOR,
          "welding two endpoints gives valence 2, which is INTERIOR to a "
          "curve and not an intersection (section 3 needs three)")
    Check(len(topology.curves) == 1,
          "the two splines are now one curve, got %d" % len(topology.curves))

    # A third spline through the same knot is what makes it an intersection.
    c1 = ui.AppendKnot(net, Gf.Vec3f(2, 0, 2))
    ui.AddSpline(net, a1, c1)
    topology = ui.Topology(len(ui.GetPoints(net)), ui.GetSplines(net))
    Check(topology.kind[a1] == ui.KIND_INTERSECTION,
          "a third spline makes the shared endpoint an intersection")
    Check(len(topology.curves) == 3,
          "an intersection splits the chain: three curves, got %d"
          % len(topology.curves))


def TestBreakUndoesWeld(ui, stage):
    net = ui.CreateCurvenet(stage, stage.GetPrimAtPath("/Asset/Geom"),
                            "BreakNet")
    centre = ui.AppendKnot(net, Gf.Vec3f(0, 0, 0))
    for p in [(1, 0, 0), (0, 0, 1), (-1, 0, 0)]:
        ui.AddSpline(net, centre, ui.AppendKnot(net, Gf.Vec3f(*p)))
    topology = ui.Topology(len(ui.GetPoints(net)), ui.GetSplines(net))
    Check(topology.kind[centre] == ui.KIND_INTERSECTION, "welded centre")

    broke = ui.BreakKnot(net, centre)
    Check(broke == 2, "breaking a 3-valence knot detaches two splines, got %d"
          % broke)
    topology = ui.Topology(len(ui.GetPoints(net)), ui.GetSplines(net))
    Check(topology.kind[centre] == ui.KIND_ANCHOR,
          "after breaking, the original knot serves one spline")
    Check(len(topology.curves) == 3, "three separate curves after break")


def TestSplitIsExact(ui, stage):
    """de Casteljau: splitting must not move the curve."""
    net = ui.CreateCurvenet(stage, stage.GetPrimAtPath("/Asset/Geom"),
                            "SplitNet")
    a = ui.AppendKnot(net, Gf.Vec3f(0, 0, 0))
    b = ui.AppendKnot(net, Gf.Vec3f(3, 0, 0))
    ui.AddSpline(net, a, b)
    # Shape it so the test is not about a straight line.
    points = ui.GetPoints(net)
    splines = ui.GetSplines(net)
    points[splines[1]] = Gf.Vec3f(1, 0, 2)
    points[splines[2]] = Gf.Vec3f(2, 0, -1)
    ui.SetPoints(net, points)

    def Sample(prim, count):
        pts = ui.GetPoints(prim)
        out = []
        for i in range(0, len(ui.GetSplines(prim)), 4):
            s = ui.GetSplines(prim)[i:i + 4]
            for k in range(count + 1):
                out.append(ui.EvalBezier(pts[s[0]], pts[s[1]], pts[s[2]],
                                         pts[s[3]], k / float(count)))
        return out

    before = Sample(net, 32)
    ui.SplitSpline(net, 0)
    Check(len(ui.GetSplines(net)) == 8, "split produced two splines")
    after = Sample(net, 16)

    # Every point of the split curve must lie on the original curve.
    worst = 0.0
    for point in after:
        nearest = min((point - other).GetLength() for other in before)
        worst = max(worst, nearest)
    Check(worst < 1e-5,
          "splitting moved the curve by %g; de Casteljau must be exact"
          % worst)


def TestProjectAndFlatten(ui, stage, mesh):
    net = ui.CreateCurvenet(stage, stage.GetPrimAtPath("/Asset/Geom"),
                            "ProjectNet")
    a = ui.AppendKnot(net, Gf.Vec3f(1.0, 2.5, 1.0))   # well off the surface
    b = ui.AppendKnot(net, Gf.Vec3f(4.0, -1.5, 2.0))
    ui.AddSpline(net, a, b)

    moved = ui.ProjectKnots(net, mesh)
    Check(moved > 0, "projection moved something")
    for point in ui.GetPoints(net):
        Check(abs(point[1]) < 1e-5,
              "a projected point must lie on the y=0 grid, got y=%g"
              % point[1])

    # Push a handle off the surface, then flatten it back into the tangent
    # plane (which for this grid is y = 0).
    points = ui.GetPoints(net)
    handle = ui.GetSplines(net)[1]
    points[handle] = Gf.Vec3f(points[handle][0], 3.0, points[handle][2])
    ui.SetPoints(net, points)
    ui.FlattenTangents(net, mesh)
    flattened = ui.GetPoints(net)[handle]
    Check(abs(flattened[1]) < 1e-5,
          "flatten must put the handle in the tangent plane, got y=%g"
          % flattened[1])


def TestBind(ui, stage, mesh):
    net = ui.CreateCurvenet(stage, stage.GetPrimAtPath("/Asset/Geom"),
                            "BindNet")
    centre = ui.AppendKnot(net, Gf.Vec3f(3, 0, 3))
    for p in [(6, 0, 3), (3, 0, 6), (0, 0, 3)]:
        ui.AddSpline(net, centre, ui.AppendKnot(net, Gf.Vec3f(*p)))

    rig = ui.FindRigPrim(stage)
    Check(rig is not None, "found the rig")
    mover = ui.BindCurvenet(stage, net, mesh, rig)
    Check(mover.GetTypeName() == "RigExecCurvenetMover",
          "bind creates a RigExecCurvenetMover")
    Check(mover.HasAPI("RigExecMoverAPI"),
          "the curvenet mover carries RigExecMoverAPI")
    curvenetTargets = mover.GetRelationship("rigExec:curvenet").GetTargets()
    Check(curvenetTargets == [net.GetPath()],
          "the mover names the curvenet")
    moves = mover.GetRelationship("rigExec:moves")
    Check(moves and not moves.IsCustom(),
          "rigExec:moves comes from RigExecMoverAPI, not a custom property")
    movesTargets = moves.GetTargets()
    Check(movesTargets == [mesh.GetPath().AppendProperty("points")],
          "the mover writes the mesh's points, got %s" % movesTargets)
    defaultWeight = mover.GetAttribute("inputs:defaultWeight")
    Check(defaultWeight and not defaultWeight.IsCustom(),
          "inputs:defaultWeight comes from RigExecMoverAPI")
    Check(defaultWeight.Get() == 1.0,
          "the curvenet mover defaults to full influence")
    Check(not mover.HasProperty("inputs:strength"),
          "the curvenet binder does not author retired inputs:strength")

    warnings = ui.CheckBindPreconditions(stage, mesh, rig)
    Check(not warnings,
          "a vertex-normal mesh at the asset root should bind clean, got %s"
          % warnings)

    # The two silent failures the panel warns about.
    mesh.GetAttribute("normals").SetMetadata("interpolation", "faceVarying")
    warnings = ui.CheckBindPreconditions(stage, mesh, rig)
    Check(any("faceVarying" in w for w in warnings),
          "faceVarying normals must be reported: they defeat derived normal "
          "maintenance")
    mesh.GetAttribute("normals").SetMetadata("interpolation", "vertex")

    UsdGeom.Xform.Define(stage, "/Asset/Geom").AddTranslateOp().Set(
        Gf.Vec3d(0, 5, 0))
    warnings = ui.CheckBindPreconditions(stage, mesh, rig)
    Check(any("identity relative to the asset root" in w for w in warnings),
          "a non-identity ancestor transform must be reported: point movers "
          "apply asset-space values straight to local points")


def main():
    _RegisterSchema()
    ui = _ImportUI()
    stage, mesh = _MakeStage()

    print("TestCreateAndDraw")
    TestCreateAndDraw(ui, stage)
    print("TestWeldMakesIntersections")
    TestWeldMakesIntersections(ui, stage)
    print("TestBreakUndoesWeld")
    TestBreakUndoesWeld(ui, stage)
    print("TestSplitIsExact")
    TestSplitIsExact(ui, stage)
    print("TestProjectAndFlatten")
    TestProjectAndFlatten(ui, stage, mesh)
    print("TestBind")
    TestBind(ui, stage, mesh)

    if failures:
        print("testUsdviewCurvenetAuthoring: %d failure(s)" % len(failures))
        return 1
    print("testUsdviewCurvenetAuthoring: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
