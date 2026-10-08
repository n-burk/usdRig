"""A drag is a sweep, and the skin must move smoothly through it.

Every other check in this suite sets a control to ONE value and looks
at the result. That misses the whole class of bug an animator hits
first, because they do not type values -- they drag, and a deformer
that is correct at both ends of a drag can still jump in the middle.

This file exists because that happened. A branch added to
RigExecPartialTransform was taken or not depending on a threshold the
control crossed mid-drag, so the answer flipped between two formulas
that disagree. Every single-value check still passed -- the worst
displacement across twenty face controls read 2.40, perfectly
healthy -- while the animator's report was "moving clusters and wires
on the face seem to make the mesh jitter and explode". Measured as a
sweep it was obvious: twelve of twenty-six face controls jumped, the
worst by 2.45 units in one step of a drag whose whole travel was 1.16.

So the measure here is the JUMP between neighbouring steps, against
the step the sweep would take if the motion were even. A smooth
deformer sits at 1; a jump is tens or thousands.

The controls are the cluster-driven ones, which is where the failure
was: the cheeks, the mouth corners, the chin, the lips. A wire and a
headwire are included as the other half of the animator's sentence.

Usage:
    python test_rigexec_face_smooth.py [schema_resources_dir]
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
_MESH = "/Biped/Geom/body_geo.points"

# Eight, not the twenty-six the sweep was found with: each one costs
# thirteen full-stack evaluations and the suite has a timeout. These are
# the families that failed -- a mouth corner, two cheeks, the chin, a
# lip, a sneer -- plus a wire-driven control and a head wire, which were
# smooth throughout and are here to keep them that way.
_CASES = [("L_Mouth", "ty"), ("L_Cheek", "tz"), ("L_CheekPuff", "tx"),
          ("M_Chin", "ty"), ("M_UpLip", "ty"), ("L_Sneer", "ty"),
          ("M_Nose", "ty"), ("M_HeadwireMid", "tx")]
_STEPS = 12
_SPAN = 2.0

# A sweep of a linear deformer sits at exactly 1.0; a curve or a
# falloff that steepens can reach a few. The failure this guards was
# 12.8 at its mildest and 536.8 at its worst, so five separates them
# with an order of magnitude to spare either way.
_RATIO = 5.0
# And a ratio means nothing on a control that barely moves: 0.05 units
# in one step is below what an animator can see.
_FLOOR = 0.05


def _control(stage, name):
    for prim in stage.Traverse():
        if (prim.GetName() == name
                and str(prim.GetTypeName()) == "RigExecControl"):
            return prim
    raise AssertionError("no RigExecControl named " + name)


def _points(rig):
    pose = rig.evaluate(0)
    assert pose.valid, [d for d in pose.diagnostics
                        if not d.startswith("warning:")]
    return [tuple(v) for v in pose.moved_property(_MESH)]


def _worst(a, b):
    out = 0.0
    if a == b:
        return 0.0
    for p, q in zip(a, b):
        if p == q:
            continue
        d = math.dist(p, q)
        if d > out:
            out = d
    return out


def main():
    _setup_environment()
    from pxr import Usd
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)

    stage = Usd.Stage.Open(_STACK)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    rig = rigexec.Rig(stage, _RIG)
    rig.cpu_reference = True

    failures = []
    for name, key in _CASES:
        prim = _control(stage, name)
        attr = prim.GetAttribute("avars:" + key)
        assert attr and attr.IsValid(), "%s has no avars:%s" % (name, key)
        frames = []
        for step in range(_STEPS + 1):
            attr.Set(_SPAN * step / float(_STEPS))
            frames.append(_points(rig))
        attr.Set(0.0)

        travel = _worst(frames[0], frames[-1])
        jump = max(_worst(frames[i - 1], frames[i])
                   for i in range(1, len(frames)))
        # The step an even sweep would take. A control that moves
        # nothing at all is a different failure and not this file's.
        assert travel > 1e-6, ("%s.avars:%s moves no skin at all" %
                               (name, key))
        even = travel / _STEPS
        ratio = jump / even
        print("    %-16s %-4s travel %7.4f  worst step %7.4f  x%.1f"
              % (name, key, travel, jump, ratio))
        if ratio > _RATIO and jump > _FLOOR:
            failures.append("%s.avars:%s jumps %.4f in one step of a "
                            "%.4f sweep (x%.1f)"
                            % (name, key, jump, travel, ratio))

    assert not failures, failures
    print("OK: %d face controls sweep smoothly" % len(_CASES))


if __name__ == "__main__":
    main()
