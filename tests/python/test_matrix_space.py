#
# RigExecMatrixMover rigExec:transformSpace: a localized cluster.
#
# A handle nests under a head control. The mover reads the handle measured
# against the head, so the points take the handle's own rotation about its
# rest pivot and none of the head's motion -- the geometry is deformed at
# rest, before the skin that carries the head. The weights are a sparse
# field: unnamed points never move, named ones blend by their weight. Both
# evaluation paths must agree (parity mode compares them exactly).
#
import math
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


PIVOT = Gf.Vec3d(2.0, 5.0, 0.0)
WEIGHTS = {1: 1.0, 3: 0.5, 6: 0.25}


def _Stage():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Joints", "Scope")
    joint = stage.DefinePrim("/Asset/Rig/Joints/Root", "RigExecJoint")
    joint.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))

    pts = [Gf.Vec3f(float(i), 5.0, 1.0) for i in range(8)]
    geo = UsdGeom.Points.Define(stage, "/Asset/Geom/cloud")
    geo.CreatePointsAttr(Vt.Vec3fArray(pts))

    stage.DefinePrim("/Asset/Rig/Controls", "Scope")
    head = stage.DefinePrim("/Asset/Rig/Controls/head", "RigExecControl")
    head.GetAttribute("rest:space").Set(
        Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 3, 0)))
    handle = stage.DefinePrim("/Asset/Rig/Controls/head/handle",
                              "RigExecControl")
    # Parent-local rest: the handle sits at PIVOT in asset space.
    handle.GetAttribute("rest:space").Set(
        Gf.Matrix4d(1.0).SetTranslate(PIVOT - Gf.Vec3d(0, 3, 0)))
    head.GetAttribute("avars:ty").Set(0.0, 1.0)
    head.GetAttribute("avars:ty").Set(7.0, 2.0)
    head.GetAttribute("avars:ry").Set(0.0, 1.0)
    head.GetAttribute("avars:ry").Set(30.0, 2.0)
    handle.GetAttribute("avars:rz").Set(0.0, 1.0)
    handle.GetAttribute("avars:rz").Set(40.0, 2.0)

    weight = stage.DefinePrim("/Asset/Rig/Weights/cluster",
                              "RigExecStaticWeight")
    weight.GetRelationship("rigExec:weightTarget").SetTargets(
        [Sdf.Path("/Asset/Geom/cloud.points")])
    weight.GetAttribute("rigExec:representation").Set("sparse")
    keys = sorted(WEIGHTS)
    weight.GetAttribute("rigExec:indices").Set(Vt.IntArray(keys))
    weight.GetAttribute("rigExec:values").Set(
        Vt.FloatArray([WEIGHTS[k] for k in keys]))
    weight.GetAttribute("rigExec:defaultWeight").Set(0.0)

    mover = stage.DefinePrim("/Asset/Rig/Movers/cluster", "RigExecMatrixMover")
    mover.AddAppliedSchema("RigExecMoverAPI")
    mover.GetRelationship("rigExec:moves").SetTargets(
        [Sdf.Path("/Asset/Geom/cloud.points")])
    mover.GetRelationship("rigExec:transform").SetTargets([handle.GetPath()])
    mover.GetRelationship("rigExec:transformSpace").SetTargets(
        [head.GetPath()])
    mover.GetRelationship("rigExec:weightObject").SetTargets(
        [weight.GetPath()])
    return stage, pts


def main():
    _RegisterSchema()
    import _rigexec
    stage, pts = _Stage()
    angle = math.radians(40.0)
    for mode in ("dynamic", "parity"):
        rig = _rigexec.Rig(stage, "/Asset/Rig")
        _Check(rig.compile() is not False, "%s: compiles" % mode)
        rig.evaluation_mode = mode
        rest = rig.evaluate(1.0).moved_property("/Asset/Geom/cloud.points")
        worst = max((Gf.Vec3d(*a) - Gf.Vec3d(*b)).GetLength()
                    for a, b in zip(rest, pts))
        _Check(worst < 1e-5, "%s: nothing moves at rest (%g)" % (mode, worst))

        pose = rig.evaluate(2.0)
        _Check(pose.baked_parity_mismatches == 0,
               "%s: baked and dynamic disagree" % mode)
        got = pose.moved_property("/Asset/Geom/cloud.points")
        for i, p in enumerate(pts):
            w = WEIGHTS.get(i, 0.0)
            d = Gf.Vec3d(p) - PIVOT
            turned = PIVOT + Gf.Vec3d(d[0] * math.cos(angle) -
                                      d[1] * math.sin(angle),
                                      d[0] * math.sin(angle) +
                                      d[1] * math.cos(angle), d[2])
            want = Gf.Vec3d(p) + (turned - Gf.Vec3d(p)) * w
            err = (Gf.Vec3d(*got[i]) - want).GetLength()
            _Check(err < 1e-4,
                   "%s: point %d at %s, expected %s (the head's motion must "
                   "not reach it)" % (mode, i, tuple(got[i]), want))
        print("  ok: %s localized sparse cluster" % mode)
    print("RIGEXEC_MATRIX_SPACE_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
