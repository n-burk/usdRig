"""Read-only probe 2: TwoBoneIk-posed end joint with nested unclaimed joint,
nested control, time-varying effector, dynamic vs parity mode; plus a
nested joint under a solver-bound joint whose PARENT is constraint-moved."""
import os, sys, pathlib
REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "tests" / "python"))
sys.argv = [sys.argv[0]]
import rigexec_test_env
rigexec_test_env.SetupPluginTest()
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))
from pxr import Usd, UsdGeom, Gf, Sdf
import rigexec
rigexec.load_schema_plugin()


def build():
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Rig", "RigExecRoot")

    def prov(path, typ, t):
        p = stage.DefinePrim(path, typ)
        p.GetAttribute("rest:tx").Set(t[0])
        p.GetAttribute("rest:ty").Set(t[1])
        p.GetAttribute("rest:tz").Set(t[2])
        return p

    root_c = prov("/Rig/Controls/Root", "RigExecControl", (0, 10, 0))
    eff_c = prov("/Rig/Controls/Eff", "RigExecControl", (0, 0, 0))
    eff_c.GetAttribute("avars:tx").Set(0.0, 1.0)
    eff_c.GetAttribute("avars:tx").Set(4.0, 2.0)
    eff_c.GetAttribute("avars:ty").Set(0.0, 1.0)
    eff_c.GetAttribute("avars:ty").Set(3.0, 2.0)
    pole_c = prov("/Rig/Controls/Pole", "RigExecControl", (0, 5, 10))
    a = prov("/Rig/Joints/A", "RigExecJoint", (0, 10, 0))
    b = prov("/Rig/Joints/A/B", "RigExecJoint", (0, -5, 0.5))
    c = prov("/Rig/Joints/A/B/C", "RigExecJoint", (0, -5, -0.5))
    piv = prov("/Rig/Joints/A/B/C/Pivot", "RigExecJoint", (2, 0, 1))     # unclaimed
    piv2 = prov("/Rig/Joints/A/B/C/Pivot/Deeper", "RigExecJoint", (1, 0, 0))
    sock = prov("/Rig/Joints/A/B/C/Sock", "RigExecControl", (0, 0, 3))
    s = stage.DefinePrim("/Rig/Solvers/Ik", "RigExecTwoBoneIk")
    s.GetRelationship("rigExec:rootControl").SetTargets([root_c.GetPath()])
    s.GetRelationship("rigExec:effectorControl").SetTargets([eff_c.GetPath()])
    s.GetRelationship("rigExec:poleControl").SetTargets([pole_c.GetPath()])
    s.GetRelationship("rigExec:joints").SetTargets([a.GetPath(), b.GetPath(), c.GetPath()])
    return stage


def frames(pose, p, joint=True):
    try:
        f = pose.joint_frame(p, True) if joint else pose.control_frame(p)
        return tuple(round(v, 4) for v in f.origin) if hasattr(f, "origin") else str(f)
    except Exception as e:
        return "ERR %s" % e


for mode in ("dynamic", "parity", "baked"):
    stage = build()
    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    rig.evaluation_mode = mode
    print("== mode", mode, "bakeable", rig.is_bakeable(), rig.bakeability_reasons()[:3])
    for t in (1.0, 2.0):
        pose = rig.evaluate(t)
        print(" t", t, "C", frames(pose, "/Rig/Joints/A/B/C"),
              "Pivot", frames(pose, "/Rig/Joints/A/B/C/Pivot"),
              "Deeper", frames(pose, "/Rig/Joints/A/B/C/Pivot/Deeper"),
              "Sock", frames(pose, "/Rig/Joints/A/B/C/Sock", False),
              "parity", getattr(pose, "baked_parity_mismatches", None))
        print("   diag", [d for d in pose.diagnostics if "mover graph" not in d][:3])
