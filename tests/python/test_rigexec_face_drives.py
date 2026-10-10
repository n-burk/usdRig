"""Every face control this file names moves the mesh it is for.

Deliberately behavioural, not structural. The obvious structural rule --
"a constraint runs after whatever poses its source" -- is NOT an
invariant of this rig: the pose stack is positional, so a finger-follow
constraint reading wrist_l_def legitimately runs before the spine
movers that pose the arm above it. A check built on that rule reported
zero on Biped_stack.usda and two hundred violations on Biped.usda,
which is how I found out it was measuring nothing.

What IS checkable is the outcome. A control that drives nothing is a
control the animator cannot use, and that is what shipped:

  - the look-at moved the eye CONTROL and not the eyeball. Two
    independent causes, and neither alone showed anything. The pose
    stack ordinal is the reverse of composed namespace order, so
    `face_eyes` ran L_Eye_drives_eye_l_def (153) BEFORE lookRot_aim_pose
    (154) and eye_l_follow_pose (156) and copied a stale control; and
    l_eye_geo was skinned solely to eye_l_trans_def, a joint nothing
    drives which does not exist in the source rig at all.
    Both look controls measured exactly 0.0000 on the mesh.

  - the lash proxies lost every weight when Biped_lashes.usda was
    deleted rather than retargeted, so the blink clusters reached
    nothing.

Numbers below are the measured post-fix values; the thresholds are
floors well under them, because the point is that the drive EXISTS, not
that it never changes.

Usage:
    python test_rigexec_face_drives.py [schema_resources_dir]
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
_CTL = _RIG + "/Main/Shot/Aux/Controls"
_SKULL = (_CTL + "/M_Body/M_Torso/M_Chest/M_ChestTop/M_Neck/M_Head"
          "/M_HeadGimbal/skull_follow/M_Skull")
_UP = _SKULL + "/M_UpFace"
# Sdf paths do not resolve "..", so the jaw is spelled from _SKULL.
_JAW = _SKULL + "/M_LoFace/M_Jaw"

# (label, control, avar, value, mesh, floor). Measured 2026-09-24.
#
# THE LOOK IS NOT IN THIS TABLE. It drives the IRIS, not the eyeball:
# the ball is a sphere skinned to eye_?_trans_def, which rides the socket
# and never turns, and the iris is placed by the surface projector's
# shader matrix. Asserting that the look moves l_eye_geo asserted the
# BUG -- the eyeballs were bound to the aimed bone eye_?_def and rolled
# inside their own sockets after the socket stretch. _check_look_drives_
# the_iris below asserts the real contract, in both directions: the ball
# must NOT move and the iris matrix MUST.
_DRIVES = [
    # lid -> lash proxy, the weights Biped_lashes.usda carries.
    # measured 1.2009
    ("L_UpLid ty", _UP + "/L_EyeSocket/L_UpLid", "ty", -1.0,
     "lash_upper_l_proxy", 0.4),
    # jaw -> the mouth interior, which already worked and must stay.
    ("M_Jaw rx", _JAW, "rx", 10.0,
     "teeth_lower_geo", 0.5),
    ("M_Jaw rx (tongue)", _JAW, "rx", 10.0,
     "tongue_geo", 0.5),
]


def _points(rig, mesh):
    pose = rig.evaluate(0)
    assert pose.valid, [d for d in pose.diagnostics
                        if not d.startswith("warning:")]
    key = "/Biped/Geom/%s.points" % mesh
    assert key in pose.moved_properties(), "%s is not deformed at all" % mesh
    return [tuple(q) for q in pose.moved_property(key)]


def _check_look_drives_the_iris(failures):
    """The look moves the IRIS and leaves the eyeball alone.

    Both halves matter. Only checking the iris would pass with the ball
    bound back onto the aimed bone, which is the bug this replaced;
    only checking the ball would pass with the look severed entirely.
    """
    import rigexec
    from pxr import Sdf, Usd

    stage = Usd.Stage.Open(_STACK)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))

    # M_Look's space avar now ships at 0 -- the eyes ride their sockets
    # until an animator asks for the look -- so switch it on to test it.
    # It was face:lookAt, then briefly avars:space, and is now
    # avars:lookAt -- `space` belongs to M_Look's real space switch.
    # other control uses for the same kind of channel.
    for p in stage.Traverse():
        if p.GetName() == "M_Look" and str(p.GetTypeName()) == "RigExecControl":
            a = p.GetAttribute("avars:lookAt")
            assert a and a.IsValid(), "M_Look has no avars:lookAt"
            a.Set(1.0)
            break

    rig = rigexec.Rig(stage, _RIG)
    rig.cpu_reference = True

    def read():
        pose = rig.evaluate(0)
        assert pose.valid, [d for d in pose.diagnostics if "error" in d]
        keys = sorted(str(k) for k in pose.shader_matrix_keys())
        mats = {str(k): list(pose.shader_matrix(k[0], k[1]))
                for k in pose.shader_matrix_keys()}
        return ([tuple(v) for v in pose.moved_property(
                    "/Biped/Geom/l_eye_geo.points")], keys, mats)

    restBall, keys, restMats = read()
    if not keys:
        failures.append("the eye projector published no shader matrix")
        return
    prim = stage.GetPrimAtPath(_CTL + "/M_Look")
    for avar, value in (("tx", 20.0), ("ty", 15.0)):
        attr = prim.GetAttribute("avars:" + avar)
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("avars:" + avar,
                                        Sdf.ValueTypeNames.Double)
        attr.Set(value)
        ball, _k, mats = read()
        rolled = max(math.dist(a, b) for a, b in zip(restBall, ball))
        iris = max(max(abs(a - b) for a, b in zip(restMats[k], mats[k]))
                   for k in restMats)
        print("    %-20s -> eyeball %8.4f (wants 0)   iris %8.4f (wants > 0)"
              % ("M_Look " + avar, rolled, iris))
        if rolled > 1e-6:
            failures.append(
                "M_Look %s ROLLS the eyeball by %.4f -- it must be bound to "
                "eye_?_trans_def, not the aimed eye_?_def" % (avar, rolled))
        if iris <= 1e-4:
            failures.append(
                "M_Look %s does not move the iris (%.6f)" % (avar, iris))
        attr.Set(0.0)


def main():
    _setup_environment()
    import rigexec
    from pxr import Sdf, Usd
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)

    failures = []
    for label, control, avar, value, mesh, floor in _DRIVES:
        stage = Usd.Stage.Open(_STACK)
        stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
        prim = stage.GetPrimAtPath(control)
        assert prim, control
        rig = rigexec.Rig(stage, _RIG)
        rig.cpu_reference = True
        rest = _points(rig, mesh)
        attr = prim.GetAttribute("avars:" + avar)
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("avars:" + avar,
                                        Sdf.ValueTypeNames.Double)
        attr.Set(value)
        moved = max(math.dist(a, b)
                    for a, b in zip(rest, _points(rig, mesh)))
        ok = moved > floor
        print("    %-20s -> %-20s %8.4f  (floor %.2f) %s"
              % (label, mesh, moved, floor, "" if ok else "<-- DRIVES NOTHING"))
        if not ok:
            failures.append("%s moves %s by only %.4f" % (label, mesh, moved))
    _check_look_drives_the_iris(failures)
    assert not failures, failures
    print("OK: every named face control drives its mesh")


if __name__ == "__main__":
    main()
