"""A pose interpolator's samples must be distinct, and mirrored.

The right deltoid ballooned because `shoulder_r_up_45` carried a stale
`rigExec:rotation` sitting 1.1 degrees from its own `neutral`. Two RBF
samples at effectively the same point, with a gaussian kernel and
normalize on, is a degenerate solve: the two weights ran to +199 and
-197 where every healthy weight sits inside 0..1.

The blend shape deltas were never the problem -- all 30 mirrored shape
pairs matched exactly. Only the cached driver rotations were stale, and
only on shoulder_r_driver: 6 of 63 mirrored pose pairs, three of them by
about 90 degrees.

Three guards, cheapest first:

1. No rotation-driven pose may coincide with its interpolator's neutral.
   This is the one that actually catches a degenerate solve.
2. Mirrored pose pairs must turn through the SAME ANGLE. Axis is not
   compared -- the left and right joints have mirrored rest orientations
   so their axes legitimately differ, and the character's own asymmetry
   shows up as a couple of degrees (up_0 reads 3.6). The three real
   faults were 89-91 degrees out, far outside that.
3. Sweeping both arms, no weight may run away.
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
_INTERPS = _RIG + "/PoseInterpolators"

# The character is not perfectly symmetric: up_0 sits 3.6 degrees off its
# mirror with everything working. The faults were 89-91 out.
_ANGLE_TOL = 15.0
# Coincident samples are the degenerate case. shoulder_r_up_45 was 1.1.
_NEUTRAL_TOL = 5.0
_WEIGHT_MAX = 1.5


def _ang(q):
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(q.GetReal())))))


def main():
    _setup_environment()
    from pxr import Sdf, Usd
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)

    stage = Usd.Stage.Open(_STACK)
    assert stage, _STACK
    root = stage.GetPrimAtPath(_INTERPS)
    assert root, _INTERPS

    # 1. no rotation-driven pose may sit on top of its neutral.
    checked = 0
    for interp in root.GetChildren():
        rot = interp.GetAttribute("rigExec:enableRotation")
        if not (rot and rot.IsValid() and rot.Get()):
            continue          # translation-driven: identity is by design
        n = interp.GetPrimAtPath("neutral") if False else \
            stage.GetPrimAtPath(interp.GetPath().AppendChild("neutral"))
        if not n:
            continue
        na = n.GetAttribute("rigExec:rotation")
        if not (na and na.IsValid() and na.Get() is not None):
            continue
        nang = _ang(na.Get())
        for p in interp.GetChildren():
            if p.GetName() == "neutral":
                continue
            a = p.GetAttribute("rigExec:rotation")
            if not (a and a.IsValid() and a.Get() is not None):
                continue
            checked += 1
            d = abs(_ang(a.Get()) - nang)
            assert d >= _NEUTRAL_TOL, (
                "%s/%s sits %.2f deg from neutral -- two RBF samples at the "
                "same point make the solve degenerate and the weights run "
                "away (this read +199 when it broke)"
                % (interp.GetName(), p.GetName(), d))
    print("    %d rotation poses, none coincident with their neutral" % checked)

    # 2. mirrored poses turn through the same angle.
    angs = {}
    for interp in root.GetChildren():
        for p in interp.GetChildren():
            a = p.GetAttribute("rigExec:rotation")
            if a and a.IsValid() and a.Get() is not None:
                angs[(interp.GetName(), p.GetName())] = _ang(a.Get())
    pairs = 0
    for (iname, pname), la in sorted(angs.items()):
        if "_l_" not in iname and "_l_" not in pname:
            continue
        key = (iname.replace("_l_", "_r_"), pname.replace("_l_", "_r_"))
        if key not in angs:
            continue
        pairs += 1
        assert abs(la - angs[key]) < _ANGLE_TOL, (
            "%s turns %.1f deg but its mirror %s turns %.1f -- a stale "
            "cached driver rotation" % (pname, la, key[1], angs[key]))
    assert pairs > 40, "only %d mirrored pairs found" % pairs
    print("    %d mirrored pose pairs agree in angle" % pairs)

    # 3. and nothing runs away over the arms' range.
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    rig = rigexec.Rig(stage, _RIG)
    rig.evaluation_mode = "baked"

    def ctl(name):
        for p in stage.Traverse():
            if p.GetName() == name and str(p.GetTypeName()) == "RigExecControl":
                return p
        return None

    def setv(prim, at, v):
        a = prim.GetAttribute("avars:" + at)
        if not (a and a.IsValid()):
            a = prim.CreateAttribute("avars:" + at, Sdf.ValueTypeNames.Double)
        a.Set(v)

    worst, where = 0.0, None
    for name in ("L_UpArm", "R_UpArm"):
        prim = ctl(name)
        assert prim, "no control %s" % name
        for ax in ("rx", "ry", "rz"):
            for v in range(-135, 136, 45):
                setv(prim, ax, float(v))
                pose = rig.evaluate(0)
                assert pose.valid, [d for d in pose.diagnostics if "error" in d]
                for m in pose.moved_properties():
                    if not m.endswith("outputs:weight"):
                        continue
                    val = pose.moved_property(m)
                    if abs(val) > abs(worst):
                        worst, where = val, "%s %s=%d %s" % (
                            name, ax, v, m.split("/")[-1])
                setv(prim, ax, 0.0)
    assert abs(worst) < _WEIGHT_MAX, (
        "a pose weight ran to %.3f at %s -- the interpolator is degenerate"
        % (worst, where))
    print("    arms swept, largest pose weight %.4f (%s)" % (worst, where))
    print("OK: the pose interpolators are well conditioned")


if __name__ == "__main__":
    main()
