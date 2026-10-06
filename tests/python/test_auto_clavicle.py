"""The biped's auto clavicles carry the shoulder with the arm's swing.

Headless test for RigExecAutoClavicle on examples/biped/Biped_autoclav.usda:

  1. At rest nothing moves, in FK and in IK.
  2. Raising the arm in FK lifts the shoulder swing control and the shoulder
     joint with it; twisting the arm about its own axis moves nothing.
  3. avars:autoClav scales the effect: 0 moves nothing, 0.5 moves less.
  4. Raising the IK hand lifts the shoulder too.
  5. The dynamic evaluator and the baked program agree bit for bit over FK,
     IK, a half blend, the dial and a swing held in its local space.

The poses are applied as interactive overrides, never authored.

Usage: test_auto_clavicle.py [<generated schema resources dir>]
"""
import os
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

from pxr import Sdf, Usd  # noqa: E402

import ikfkMatch  # noqa: E402

_failures = []

C = "/Biped/Rig/Main/Shot/Aux/Controls"
CT = C + "/M_Body/M_Torso/M_Chest/M_ChestTop"
SHLDR = CT + "/L_Shldr"
SWING = SHLDR + "/L_UpArmSwing"
UPARM = SWING + "/L_UpArm"
SHOULDER = ("/Biped/Rig/Main/Shot/Aux/Joints/hips_def/spine_0_def/"
            "spine_1_def/spine_2_def/spine_3_def/spine_4_def/spine_5_def/"
            "chest_def/clavicle_l_def/shoulder_l_def")


def _Check(condition, message):
    if not condition:
        _failures.append(message)
        print("FAIL: %s" % message)


class _Session(object):
    def __init__(self, mode=None):
        import _rigexec
        self.stage = Usd.Stage.Open(os.path.join(
            _ROOT, "examples", "biped", "Biped_stack.usda"))
        self.rig = _rigexec.Rig(self.stage, "/Biped/Rig")
        self.rig.compile()
        if mode is not None:
            self.rig.evaluation_mode = mode
        self.rig.publish_weight_fields = False

    def Pose(self, overrides):
        self.rig.set_interactive_overrides(
            [(path, name, float(value))
             for (path, name), value in overrides.items()])
        return self.rig.evaluate(1.0)


def _Moved(a, b, path, joint=False):
    frame = ikfkMatch.JointFrame if joint else ikfkMatch.ControlFrame
    return (frame(a, path).ExtractTranslation() -
            frame(b, path).ExtractTranslation())


def TestBehaviour(session):
    rest = session.Pose({})
    ik = {(C + "/L_Arm", "avars:ikfk"): 1.0}
    _Check(_Moved(session.Pose(ik), rest, SWING).GetLength() < 1e-9,
           "the IK arm at rest moves nothing")

    raised = {(UPARM, "avars:ry"): -80.0}
    up = session.Pose(raised)
    lift = _Moved(up, rest, SWING)
    _Check(lift[1] > 5.0, "raising the arm lifts the shoulder (%.3f)"
           % lift[1])
    _Check((_Moved(up, rest, SHOULDER, joint=True) - lift).GetLength() < 1e-6,
           "the shoulder joint rides the moved swing")

    twisted = session.Pose({(UPARM, "avars:rx"): 60.0})
    _Check(_Moved(twisted, rest, SWING).GetLength() < 1e-9,
           "twisting the arm moves nothing")

    off = dict(raised)
    off[(SHLDR, "avars:autoClav")] = 0.0
    _Check(_Moved(session.Pose(off), rest, SWING).GetLength() < 1e-9,
           "autoClav 0 moves nothing")
    half = dict(raised)
    half[(SHLDR, "avars:autoClav")] = 0.5
    halfLift = _Moved(session.Pose(half), rest, SWING).GetLength()
    _Check(0.0 < halfLift < lift.GetLength(),
           "autoClav 0.5 moves less (%.3f of %.3f)"
           % (halfLift, lift.GetLength()))

    handUp = dict(ik)
    handUp[(C + "/L_ArmIK", "avars:ty")] = 40.0
    ikLift = _Moved(session.Pose(handUp), rest, SWING)
    _Check(ikLift[1] > 1.0, "raising the IK hand lifts the shoulder (%.3f)"
           % ikLift[1])


def TestParity(baked, dynamic):
    poses = [
        {(UPARM, "avars:ry"): -80.0},
        {(UPARM, "avars:rz"): 60.0, (UPARM, "avars:ry"): -30.0},
        {(UPARM, "avars:ry"): -80.0, (SHLDR, "avars:autoClav"): 0.5},
        {(C + "/L_Arm", "avars:ikfk"): 1.0, (C + "/L_ArmIK", "avars:ty"): 40.0,
         (C + "/L_ArmIK", "avars:tz"): 15.0},
        {(C + "/L_Arm", "avars:ikfk"): 0.5, (C + "/L_ArmIK", "avars:ty"): 40.0,
         (UPARM, "avars:ry"): -60.0},
        {(SWING, "avars:space"): 0.0, (UPARM, "avars:ry"): -70.0,
         (C + "/M_Body/M_Torso/M_Chest", "avars:rz"): 25.0},
    ]
    for k, pose in enumerate(poses):
        a, b = baked.Pose(pose), dynamic.Pose(pose)
        worst = 0.0
        for path, joint in ((SWING, False), (SHOULDER, True)):
            frame = ikfkMatch.JointFrame if joint else ikfkMatch.ControlFrame
            fa, fb = frame(a, path), frame(b, path)
            worst = max(worst, max(abs(fa[r][c] - fb[r][c])
                                   for r in range(4) for c in range(4)))
        _Check(worst == 0.0, "pose %d: baked and dynamic agree (%.3g)"
               % (k, worst))


def main():
    baked = _Session()
    TestBehaviour(baked)
    dynamic = _Session("dynamic")
    TestBehaviour(dynamic)
    TestParity(baked, dynamic)
    if _failures:
        print("%d failure(s)" % len(_failures))
        return 1
    print("test_auto_clavicle: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
