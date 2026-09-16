"""Read-only probe: does an unclaimed joint/control nested under a
solver-posed joint follow the solver output in the native evaluator?"""
import os, sys, pathlib
REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "tests" / "python"))
sys.argv = [sys.argv[0]]
import rigexec_test_env
rigexec_test_env.SetupPluginTest()
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))
os.environ.setdefault("RIGEXEC_DLL_PATH", os.pathsep.join([
    str(REPO / "build"), str(REPO.parent / "usd-install" / "bin"),
    str(REPO.parent / "usd-install" / "lib")]))
from pxr import Usd, UsdGeom, Gf, Sdf
import rigexec
rigexec.load_schema_plugin(str(REPO / "build" / "plugin" / "rigExecSchema" / "resources")
                           if (REPO / "build" / "plugin" / "rigExecSchema" / "resources").is_dir() else None)

stage = Usd.Stage.CreateInMemory()
stage.DefinePrim("/Rig", "RigExecRoot")

def prov(path, typ, x=0.0):
    p = stage.DefinePrim(path, typ)
    p.GetAttribute("rest:tx").Set(x)
    return p

ctl = prov("/Rig/Controls/C", "RigExecControl", 0)
ctl.GetAttribute("avars:rz").Set(90.0)
j = prov("/Rig/Joints/J", "RigExecJoint", 0)
k = prov("/Rig/Joints/J/K", "RigExecJoint", 5)          # unclaimed nested joint
s = prov("/Rig/Joints/J/Sock", "RigExecControl", 3)     # nested control
x = stage.DefinePrim("/Rig/Joints/J/Null", "Xform")       # plain xform
UsdGeom.Xformable(x).AddTranslateOp().Set(Gf.Vec3d(7, 0, 0))
solver = stage.DefinePrim("/Rig/Solvers/Fk", "RigExecFkChain")
solver.GetRelationship("rigExec:controls").SetTargets([ctl.GetPath()])
solver.GetRelationship("rigExec:joints").SetTargets([j.GetPath()])

rig = rigexec.Rig(stage, "/Rig")
rig.compile()
pose = rig.evaluate(-1.0)
def show(label, fn):
    try:
        print(label, fn())
    except Exception as e:  # noqa
        print(label, "ERR", e)
show("J final", lambda: pose.joint_frame("/Rig/Joints/J", True))
show("K final", lambda: pose.joint_frame("/Rig/Joints/J/K", True))
show("Sock control", lambda: pose.control_frame("/Rig/Joints/J/Sock"))
print([m for m in dir(pose) if not m.startswith("_")])
try:
    print("diagnostics", pose.diagnostics)
except Exception as e:
    print("diag err", e)
