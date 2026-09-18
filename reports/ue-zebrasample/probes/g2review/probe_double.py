"""Read-only probe 4: FloatMathMover inputs:value connected to a double avar;
and the 'animated unless rig-driven' blend pattern with a connected envelope."""
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

stage = Usd.Stage.CreateInMemory()
stage.DefinePrim("/Rig", "RigExecRoot")
ctl = stage.DefinePrim("/Rig/Controls/Squash", "RigExecControl")
ctl.GetAttribute("avars:tx").Set(0.75)
dial = ctl.CreateAttribute("face:rigSquash", Sdf.ValueTypeNames.Float)
dial.Set(0.4)
switch = ctl.CreateAttribute("face:useRig", Sdf.ValueTypeNames.Float)
switch.Set(0.0, 1.0)
switch.Set(1.0, 2.0)
curves = stage.DefinePrim("/Rig/Curves", "Scope")
c1 = curves.CreateAttribute("curves:fromDouble", Sdf.ValueTypeNames.Float)
c1.Set(0.0)
c2 = curves.CreateAttribute("curves:headSquash", Sdf.ValueTypeNames.Float)
c2.Set(-0.3, 1.0)   # "baked animation" value
c2.Set(-0.3, 2.0)

m1 = stage.DefinePrim("/Rig/Movers/FromDouble", "RigExecFloatMathMover")
m1.ApplyAPI("RigExecMoverAPI")
m1.CreateRelationship("rigExec:moves").SetTargets([c1.GetPath()])
m1.GetAttribute("rigExec:operation").Set("blend")
m1.GetAttribute("inputs:value").Set(-9.0)
m1.GetAttribute("inputs:value").AddConnection(ctl.GetAttribute("avars:tx").GetPath())

m2 = stage.DefinePrim("/Rig/Movers/CurveSource", "RigExecFloatMathMover")
m2.ApplyAPI("RigExecMoverAPI")
m2.CreateRelationship("rigExec:moves").SetTargets([c2.GetPath()])
m2.GetAttribute("rigExec:operation").Set("blend")
m2.GetAttribute("inputs:value").AddConnection(dial.GetPath())
m2.GetAttribute("inputs:defaultWeight").AddConnection(switch.GetPath())

rig = rigexec.Rig(stage, "/Rig")
try:
    rig.compile()
except Exception as e:
    print("compile error:", e)
for t in (1.0, 2.0):
    pose = rig.evaluate(t)
    out = {}
    for p in ("/Rig/Curves.curves:fromDouble", "/Rig/Curves.curves:headSquash"):
        try:
            out[p] = pose.moved_property(p)
        except Exception as e:
            out[p] = "ERR %s" % e
    print("t", t, out, [d for d in pose.diagnostics if "mover graph" not in d][:3])
