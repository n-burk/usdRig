"""The two eye projectors must answer deformation identically.

`RigExecSurfaceProjector` builds its ray from the SOURCE FRAME's Z axis,
not from `rigExec:rayDirection`. The two eye bind joints are 180 degrees
apart about Y rather than mirrored --

    eye_l_def   z=+1   forward
    eye_r_def   z=-1   backward

-- so the right eye cast into the BACK of its own eyeball. It still
published a matrix, derived from the back surface, which answered
deformation differently: at HeadwireLow ty=2, with the two sides getting
identical geometry, the left projector turned 17.65 degrees and the
right 6.80.

The fix is a child joint under eye_r_def carrying the 180 back, used as
the projector's source: same origin, same look tracking, facing forward.

The look is switched OFF here. With `avars:lookAt=1` the two eyes
legitimately aim differently at a fixed target, which masks everything
else; at 0 any difference left is a real fault.
"""
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_STACK = os.path.join(_REPO, "examples", "biped", "Biped_stack.usda")
_RIG = "/Biped/Rig"
# the character's own asymmetry is well under this; the fault was 10 deg
_ROT_TOL = 0.5
_TRA_TOL = 0.02


def main():
    _setup_environment()
    from pxr import Gf, Sdf, Usd
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)

    stage = Usd.Stage.Open(_STACK)
    assert stage, _STACK
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    rig = rigexec.Rig(stage, _RIG)
    rig.evaluation_mode = "baked"

    def ctl(name):
        for p in stage.Traverse():
            if p.GetName() == name and str(p.GetTypeName()) == "RigExecControl":
                return p
        return None

    look = ctl("M_Look")
    assert look, "no M_Look"
    la = look.GetAttribute("avars:lookAt")
    assert a_valid(la), "M_Look has no avars:lookAt"
    la.Set(0.0)

    # 1. the right projector's source must face forward.
    for side in ("l", "r"):
        proj = None
        for p in stage.Traverse():
            if p.GetName() == "%s_eye_projector" % side:
                proj = p
                break
        assert proj, "no %s_eye_projector" % side
        src = proj.GetRelationship("rigExec:sources").GetTargets()
        assert src, "%s_eye_projector has no source" % side
        pose = rig.evaluate(0)
        assert pose.valid, [d for d in pose.diagnostics if "error" in d]
        m = Gf.Matrix4d(*[float(x) for x in
                          pose.joint_frame(str(src[0])).to_matrix4()])
        z = Gf.Vec3d(m[2][0], m[2][1], m[2][2]).GetNormalized()
        assert z[2] > 0.5, (
            "%s_eye_projector's source %s points BACKWARD (row2 z=%.4f); the "
            "ray is the source frame's Z, so it casts into the back of the "
            "eyeball" % (side, src[0].name, z[2]))
    print("    both projector sources face forward")

    # 2. identical response to every deformer, with the look off.
    def setv(name, at, v):
        c = ctl(name)
        if not c:
            return False
        a = c.GetAttribute("avars:" + at)
        if not a_valid(a):
            a = c.CreateAttribute("avars:" + at, Sdf.ValueTypeNames.Double)
        a.Set(v)
        return True

    def mats():
        p = rig.evaluate(0)
        assert p.valid, [d for d in p.diagnostics if "error" in d]
        return {s: Gf.Matrix4d(*[float(x) for x in p.shader_matrix(
            "/Biped/Geom/%s_eye_geo" % s, "eyeProjector")])
            for s in ("l", "r")}

    def delta(a, b):
        t = (b.ExtractTranslation() - a.ExtractTranslation()).GetLength()
        q = (a.GetInverse() * b).ExtractRotationQuat()
        return t, math.degrees(2 * math.acos(
            max(-1.0, min(1.0, abs(q.GetReal())))))

    rest = mats()
    cases = [("M_HeadwireTop", "ty", 2.0), ("M_HeadwireTop", "ty", -2.0),
             ("M_HeadwireMid", "ty", 2.0), ("M_HeadwireLow", "ty", 2.0),
             ("M_Head", "ry", 40.0), ("M_Head", "rz", 22.0)]
    checked = 0
    for name, at, v in cases:
        if not setv(name, at, v):
            continue
        now = mats()
        setv(name, at, 0.0)
        lt, lr = delta(rest["l"], now["l"])
        rt, rr = delta(rest["r"], now["r"])
        checked += 1
        assert abs(lr - rr) < _ROT_TOL and abs(lt - rt) < _TRA_TOL, (
            "%s %s=%g: the projectors disagree -- L %.3f/%.2f deg vs "
            "R %.3f/%.2f deg" % (name, at, v, lt, lr, rt, rr))
    assert checked >= 5, "only %d cases ran" % checked
    print("    %d deformers move both projectors identically" % checked)
    la.Set(1.0)
    print("OK: the eye projectors are symmetric")


def a_valid(a):
    return bool(a) and a.IsValid()


if __name__ == "__main__":
    main()
