"""Read-only probe 3: a PositionConstraint whose sources are (a) a nested
unclaimed RigExecJoint and (b) a plain UsdGeomXform, both under an IK-posed
end joint. Does the constraint see the solved motion?"""
import os, sys, pathlib
REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "tests" / "python"))
sys.argv = [sys.argv[0]]
import rigexec_test_env
rigexec_test_env.SetupPluginTest()
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))
from pxr import Usd, UsdGeom, Gf, Sdf, Vt
import rigexec
rigexec.load_schema_plugin()


def build(source_kind):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Asset")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")

    def prov(path, typ, t):
        p = stage.DefinePrim(path, typ)
        p.GetAttribute("rest:tx").Set(t[0])
        p.GetAttribute("rest:ty").Set(t[1])
        p.GetAttribute("rest:tz").Set(t[2])
        return p

    R = "/Asset/Rig"
    root_c = prov(R + "/Controls/Root", "RigExecControl", (0, 10, 0))
    eff_c = prov(R + "/Controls/Eff", "RigExecControl", (0, 0, 0))
    eff_c.GetAttribute("avars:tx").Set(0.0, 1.0)
    eff_c.GetAttribute("avars:tx").Set(4.0, 2.0)
    eff_c.GetAttribute("avars:ty").Set(0.0, 1.0)
    eff_c.GetAttribute("avars:ty").Set(3.0, 2.0)
    pole_c = prov(R + "/Controls/Pole", "RigExecControl", (0, 5, 10))
    follower = prov(R + "/Controls/Follower", "RigExecControl", (0, 0, 0))
    a = prov(R + "/Joints/A", "RigExecJoint", (0, 10, 0))
    b = prov(R + "/Joints/A/B", "RigExecJoint", (0, -5, 0.5))
    c = prov(R + "/Joints/A/B/C", "RigExecJoint", (0, -5, -0.5))
    if source_kind == "joint":
        src = prov(R + "/Joints/A/B/C/Heel", "RigExecJoint", (2, 0, 1))
    else:
        x = UsdGeom.Xform.Define(stage, R + "/Joints/A/B/C/Heel")
        x.AddTranslateOp().Set(Gf.Vec3d(2, 0, 1))
        src = x.GetPrim()
    s = stage.DefinePrim(R + "/Solvers/Ik", "RigExecTwoBoneIk")
    s.GetRelationship("rigExec:rootControl").SetTargets([root_c.GetPath()])
    s.GetRelationship("rigExec:effectorControl").SetTargets([eff_c.GetPath()])
    s.GetRelationship("rigExec:poleControl").SetTargets([pole_c.GetPath()])
    s.GetRelationship("rigExec:joints").SetTargets([a.GetPath(), b.GetPath(), c.GetPath()])
    con = stage.DefinePrim(R + "/Movers/Follow", "RigExecPositionConstraint")
    con.ApplyAPI("RigExecMoverAPI")
    con.CreateRelationship("rigExec:moves").SetTargets([follower.GetPath()])
    con.CreateRelationship("rigExec:sources").SetTargets([src.GetPath()])
    return stage


def org(pose, p):
    try:
        return tuple(round(v, 4) for v in pose.control_frame(p).origin)
    except Exception as e:
        return "ERR %s" % e


for kind in ("joint", "xform"):
    stage = build(kind)
    rig = rigexec.Rig(stage, "/Asset/Rig")
    rig.compile()
    print("== source", kind, "bakeable", rig.is_bakeable(), rig.bakeability_reasons()[:2])
    for t in (1.0, 2.0):
        pose = rig.evaluate(t)
        print("  t", t, "C", tuple(round(v, 4) for v in pose.joint_frame("/Asset/Rig/Joints/A/B/C", True).origin),
              "Follower", org(pose, "/Asset/Rig/Controls/Follower"),
              [d for d in pose.diagnostics if "mover graph" not in d][:2])
