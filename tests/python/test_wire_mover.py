#
# RigExecCurveMover "wire": points follow a NURBS driver curve.
#
# A 21 x 5 grid lies along a degree-2 open NURBS curve. At frame 10 the
# curve's middle control point is lifted; the grid points bound to the
# curve must lift by exactly the curve's own displacement at their bind
# parameter, scaled by the dropoff falloff, and nothing may move at rest.
# The same rig is then evaluated in parity mode, which compares the baked
# program with the dynamic walk exactly.
#
import math
import os
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd, UsdGeom, Vt  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


CVS = [(-10, 0, 0), (-6, 0, 0), (-2, 0, 0), (2, 0, 0), (6, 0, 0), (10, 0, 0)]
ORDER = 3
# Open (clamped) knots for 6 control points of order 3.
KNOTS = [0, 0, 0, 1, 2, 3, 4, 4, 4]
DROPOFF = 8.0


def _Stage():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    joints = stage.DefinePrim("/Asset/Rig/Joints", "Scope")
    joint = stage.DefinePrim("/Asset/Rig/Joints/Root", "RigExecJoint")
    joint.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))

    # The grid: x across the curve's length, z away from it.
    mesh = UsdGeom.Mesh.Define(stage, "/Asset/Geom/grid")
    pts, counts, indices = [], [], []
    nx, nz = 21, 5
    for k in range(nz):
        for i in range(nx):
            pts.append(Gf.Vec3f(-10.0 + i, 0.0, float(k) * 2.0))
    for k in range(nz - 1):
        for i in range(nx - 1):
            a = k * nx + i
            counts.append(4)
            indices.extend([a, a + 1, a + nx + 1, a + nx])
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)

    curve = UsdGeom.NurbsCurves.Define(stage, "/Asset/Rig/Curves/wire_curve")
    curve.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*c) for c in CVS]))
    curve.CreateOrderAttr(Vt.IntArray([ORDER]))
    curve.CreateKnotsAttr(Vt.DoubleArray([float(k) for k in KNOTS]))
    curve.CreateCurveVertexCountsAttr(Vt.IntArray([len(CVS)]))
    lifted = list(CVS)
    lifted[2] = (-2, 3, 0)
    lifted[3] = (2, 3, 0)
    pointsAttr = curve.GetPointsAttr()
    pointsAttr.Set(Vt.Vec3fArray([Gf.Vec3f(*c) for c in CVS]), 1.0)
    pointsAttr.Set(Vt.Vec3fArray([Gf.Vec3f(*c) for c in lifted]), 10.0)

    import _rigexec
    binds = _rigexec.bind_wire([tuple(p) for p in pts], [tuple(c) for c in CVS],
                               ORDER, [float(k) for k in KNOTS])
    holder = stage.DefinePrim("/Asset/Rig/Curves/wire_bind", "Scope")
    bindAttr = holder.CreateAttribute("rigExec:bindCoordinates",
                                      Sdf.ValueTypeNames.Float2Array)
    bindAttr.Set(Vt.Vec2fArray([Gf.Vec2f(*b) for b in binds]))

    mover = stage.DefinePrim("/Asset/Rig/Movers/grid_wire", "RigExecCurveMover")
    mover.GetRelationship("rigExec:moves").SetTargets(
        [Sdf.Path("/Asset/Geom/grid.points")])
    mover.GetAttribute("rigExec:mode").Set("wire")
    mover.GetRelationship("rigExec:driverCurve").SetTargets(
        [curve.GetPath()])
    mover.GetRelationship("rigExec:bindCoordinates").SetTargets(
        [bindAttr.GetPath()])
    mover.CreateAttribute("inputs:dropoffDistance",
                          Sdf.ValueTypeNames.Float).Set(DROPOFF)
    return stage, pts, binds, lifted


def _Nurbs(cvs, u):
    """Reference de Boor evaluation in Python."""
    p = ORDER - 1
    n = len(cvs)
    u = min(max(u, KNOTS[p]), KNOTS[n])
    s = p
    while s + 1 < n and KNOTS[s + 1] <= u:
        s += 1
    d = [Gf.Vec3d(*cvs[s - p + j]) for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            i = s - p + j
            denom = KNOTS[i + p + 1 - r] - KNOTS[i]
            a = (u - KNOTS[i]) / denom if denom > 0 else 0.0
            d[j] = d[j - 1] * (1.0 - a) + d[j] * a
    return d[p]


def main():
    _RegisterSchema()
    import _rigexec
    stage, pts, binds, lifted = _Stage()

    # Bind coordinates: a point on the curve binds at distance 0.
    _Check(abs(binds[10][1]) < 1e-4,
           "a point on the curve binds at distance %g" % binds[10][1])
    _Check(abs(binds[10 + 21 * 2][1] - 4.0) < 1e-3,
           "a point 4 cm off the curve binds at 4 cm: %g"
           % binds[10 + 21 * 2][1])

    for mode in ("reference", "parity"):
        rig = _rigexec.Rig(stage, "/Asset/Rig")
        rig.compile()
        rig.evaluation_mode = mode
        rest = rig.evaluate(1.0)
        restPts = rest.moved_property("/Asset/Geom/grid.points")
        worst = max((Gf.Vec3d(*a) - Gf.Vec3d(*b)).GetLength()
                    for a, b in zip(restPts, pts))
        _Check(worst < 1e-5, "%s: nothing moves at rest (%g)" % (mode, worst))

        posed = rig.evaluate(10.0)
        _Check(posed.baked_parity_mismatches == 0,
               "%s: baked and dynamic disagree" % mode)
        got = posed.moved_property("/Asset/Geom/grid.points")
        worst = 0.0
        for i, (p, b) in enumerate(zip(pts, binds)):
            u, d = float(b[0]), float(b[1])
            s = min(max(d / DROPOFF, 0.0), 1.0)
            f = 1.0 - s * s * (3.0 - 2.0 * s)
            delta = _Nurbs(lifted, u) - _Nurbs(CVS, u)
            want = Gf.Vec3d(*p) + delta * f
            worst = max(worst, (Gf.Vec3d(*got[i]) - want).GetLength())
        _Check(worst < 1e-4,
               "%s: every point follows the curve by the reference wire "
               "maths (worst %g)" % (mode, worst))
        middle = Gf.Vec3d(*got[10]) - Gf.Vec3d(*pts[10])
        _Check(middle[1] > 1.0,
               "%s: the point under the lifted span rises (%s)" % (mode, middle))
        far = Gf.Vec3d(*got[10 + 21 * 4]) - Gf.Vec3d(*pts[10 + 21 * 4])
        _Check(far.GetLength() < 1e-6,
               "%s: a point beyond the dropoff does not move (%s)" % (mode, far))
        print("  ok: %s wire" % mode)
    # A sparse weight field: only the named points move, each scaled.
    sparse = {10: 1.0, 10 + 21: 0.5, 5: 0.25}
    weight = stage.DefinePrim("/Asset/Rig/Weights/wire", "RigExecStaticWeight")
    weight.GetRelationship("rigExec:weightTarget").SetTargets(
        [Sdf.Path("/Asset/Geom/grid.points")])
    weight.GetAttribute("rigExec:representation").Set("sparse")
    weight.GetAttribute("rigExec:indices").Set(Vt.IntArray(sorted(sparse)))
    weight.GetAttribute("rigExec:values").Set(
        Vt.FloatArray([sparse[k] for k in sorted(sparse)]))
    weight.GetAttribute("rigExec:defaultWeight").Set(0.0)
    stage.GetPrimAtPath("/Asset/Rig/Movers/grid_wire").GetRelationship(
        "rigExec:weightObject").SetTargets([weight.GetPath()])
    for mode in ("reference", "parity"):
        rig = _rigexec.Rig(stage, "/Asset/Rig")
        rig.compile()
        rig.evaluation_mode = mode
        posed = rig.evaluate(10.0)
        _Check(posed.baked_parity_mismatches == 0,
               "%s: sparse baked and dynamic disagree" % mode)
        got = posed.moved_property("/Asset/Geom/grid.points")
        worst = 0.0
        for i, (p, b) in enumerate(zip(pts, binds)):
            u, d = float(b[0]), float(b[1])
            s = min(max(d / DROPOFF, 0.0), 1.0)
            f = (1.0 - s * s * (3.0 - 2.0 * s)) * sparse.get(i, 0.0)
            want = Gf.Vec3d(*p) + (_Nurbs(lifted, u) - _Nurbs(CVS, u)) * f
            worst = max(worst, (Gf.Vec3d(*got[i]) - want).GetLength())
        _Check(worst < 1e-4, "%s: sparse wire (worst %g)" % (mode, worst))
        print("  ok: %s sparse wire" % mode)
    # The bind table sparse as well, parallel to the weight indices.
    bind_prim = stage.GetPrimAtPath("/Asset/Rig/Curves/wire_bind")
    bind_prim.GetAttribute("rigExec:bindCoordinates").Set(Vt.Vec2fArray(
        [Gf.Vec2f(*binds[k]) for k in sorted(sparse)]))
    for mode in ("reference", "parity"):
        rig = _rigexec.Rig(stage, "/Asset/Rig")
        rig.compile()
        rig.evaluation_mode = mode
        again = rig.evaluate(10.0)
        _Check(again.baked_parity_mismatches == 0,
               "%s: sparse binds, baked and dynamic disagree" % mode)
        moved = again.moved_property("/Asset/Geom/grid.points")
        worst = max((Gf.Vec3d(*a) - Gf.Vec3d(*b)).GetLength()
                    for a, b in zip(moved, got))
        _Check(worst < 1e-5, "%s: sparse binds match full binds (%g)"
               % (mode, worst))
        print("  ok: %s sparse bind table" % mode)
    print("RIGEXEC_WIRE_MOVER_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
