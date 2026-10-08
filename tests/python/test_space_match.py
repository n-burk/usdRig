"""The biped's spaces are the ones it ships with, and switching one
keeps the control where it is.

Headless test for the biped's space switches (examples/biped/
Biped_spaces.usda) and plugin/rigExecUsdview/spaceMatch.py:

  1. Every space control carries its spaces, in their order,
     with its default: the IK arms world/chest/head/hip_swivel/hips, the IK
     legs world/hip_swivel/hips, the poles world/chest/hand and
     world/pelvis/foot, the shoulder swings, neck and head local/world/hips
     (the swings default to world), the look target local/world.
  2. The shoulder swing, neck and head world and hips spaces are
     rotation-only: with the chest turned, the control stays where the
     chest carries it (its position matches local) and keeps its rest
     rotation. The auto clavicles are held off for this, since they move
     the swing on purpose.
  3. Independent shared-graph evaluations agree on those frames.
  4. A space switch planned by spaceMatch leaves the control's posed frame
     where it was, for a rotation-only space and for a full one.

The pose under test is applied as interactive overrides, never authored.

Usage: test_space_match.py [<generated schema resources dir>]
"""
import os
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

from pxr import Sdf, Usd  # noqa: E402

import ikfkMatch  # noqa: E402
import spaceMatch  # noqa: E402

_failures = []
_POSITION = 1e-6
_PARITY = 1e-9

C = "/Biped/Rig/Main/Shot/Aux/Controls"
CT = C + "/M_Body/M_Torso/M_Chest/M_ChestTop"
CHEST = C + "/M_Body/M_Torso/M_Chest"
SWING = CT + "/L_Shldr/L_UpArmSwing"
NECK = CT + "/M_Neck"
HEAD = CT + "/M_Neck/M_Head"
TIME = Usd.TimeCode(1.0)


def _Check(condition, message):
    if not condition:
        _failures.append(message)
        print("FAIL: %s" % message)


def _Attr(path, name):
    return Sdf.Path(path).AppendProperty(name)


class _Session(object):
    """A compiled biped and an evaluate(overrides, time) over a base pose."""

    def __init__(self):
        import _rigexec
        self.stage = Usd.Stage.Open(os.path.join(
            _ROOT, "examples", "biped", "Biped_stack.usda"))
        self.rig = _rigexec.Rig(self.stage, "/Biped/Rig")
        self.rig.compile()
        self.rig.publish_weight_fields = False
        self.base = {}

    def Evaluate(self, overrides, time):
        merged = dict(self.base)
        merged.update(overrides)
        self.rig.set_interactive_overrides(
            [(str(p.GetPrimPath()), p.name, float(v))
             for p, v in merged.items()])
        return self.rig.evaluate(time.GetValue())

    def Frame(self, path, overrides):
        return ikfkMatch.ControlFrame(self.Evaluate(overrides, TIME),
                                      Sdf.Path(path))


def _With(pose, attr, value):
    out = dict(pose)
    out[attr] = value
    return out


def _Max(a, b, rows=range(4)):
    return max(abs(a[i][j] - b[i][j]) for i in rows for j in range(4))


def TestSpaceLists(session):
    expected = {
        C + "/L_ArmIK": (["world", "chest", "head", "hip_swivel", "hips"], 0),
        C + "/R_ArmIK": (["world", "chest", "head", "hip_swivel", "hips"], 0),
        C + "/L_LegIK": (["world", "hip_swivel", "hips"], 0),
        C + "/R_LegIK": (["world", "hip_swivel", "hips"], 0),
        C + "/L_ArmPV": (["world", "chest", "hand"], 0),
        C + "/R_ArmPV": (["world", "chest", "hand"], 0),
        C + "/L_LegPV": (["world", "pelvis", "foot"], 2),
        C + "/R_LegPV": (["world", "pelvis", "foot"], 2),
        SWING: (["local", "world", "hips"], 1),
        CT + "/R_Shldr/R_UpArmSwing": (["local", "world", "hips"], 1),
        NECK: (["local", "world", "hips"], 0),
        HEAD: (["local", "world", "hips"], 0),
        C + "/M_Look": (["local", "world"], 0),
    }
    found = {}
    for prim in session.stage.Traverse():
        if prim.GetTypeName() != "RigExecSpaceSwitch":
            continue
        target = prim.GetRelationship("rigExec:target").GetTargets()[0]
        found[str(target)] = prim
    for control, (labels, default) in expected.items():
        prim = found.get(control)
        _Check(prim is not None, "%s has a space switch" % control)
        if prim is None:
            continue
        got = list(prim.GetAttribute("rigExec:spaceLabels").Get())
        _Check(got == labels, "%s spaces %s, expected %s"
               % (control, got, labels))
        attr = session.stage.GetAttributeAtPath(_Attr(control, "avars:space"))
        _Check(attr and attr.Get() == default,
               "%s defaults to space %s, got %s"
               % (control, default, attr.Get() if attr else None))
    for stale in (CT + "/L_Shldr/L_UpArmSwing/L_UpArm",
                  HEAD + "/M_HeadGimbal/skull_follow/M_Skull"):
        _Check(stale not in found, "%s no longer has a space" % stale)


def TestRotationOnly(session):
    # The auto clavicles off: turning the chest under a world-held arm swings
    # the arm against the chest, which lifts the shoulder by design, and this
    # is about the space alone.
    turned = {_Attr(CHEST, "avars:rz"): -30.0, _Attr(CHEST, "avars:ry"): 20.0,
              _Attr(CT + "/L_Shldr", "avars:autoClav"): 0.0,
              _Attr(CT + "/R_Shldr", "avars:autoClav"): 0.0}
    for control in (SWING, NECK, HEAD):
        space = _Attr(control, "avars:space")
        rest = session.Frame(control, {space: 1.0})
        local = session.Frame(control, _With(turned, space, 0.0))
        for index, name in ((1, "world"), (2, "hips")):
            held = session.Frame(control, _With(turned, space, index))
            _Check(_Max(held, local, rows=[3]) < _POSITION,
                   "%s in %s rides on the chest (%.3g off)"
                   % (control, name, _Max(held, local, rows=[3])))
            _Check(_Max(held, rest, rows=range(3)) < _POSITION,
                   "%s in %s keeps its rotation (%.3g off)"
                   % (control, name, _Max(held, rest, rows=range(3))))
        _Check(_Max(local, rest, rows=range(3)) > 0.1,
               "%s in local turns with the chest" % control)


def TestParity(baked, fresh):
    turned = {_Attr(CHEST, "avars:rz"): -30.0, _Attr(CHEST, "avars:ry"): 20.0,
              _Attr(C + "/M_Body", "avars:ry"): 25.0}
    for control in (SWING, NECK, HEAD):
        space = _Attr(control, "avars:space")
        for index in (0, 1, 2):
            pose = _With(turned, space, index)
            a = baked.Frame(control, pose)
            b = fresh.Frame(control, pose)
            _Check(_Max(a, b) < _PARITY,
                   "%s space %d: reused and fresh graphs agree (%.3g)"
                   % (control, index, _Max(a, b)))


def TestMatch(session):
    cases = [
        # Rotation-only: the swing posed in local, held in world.
        (SWING, {_Attr(CHEST, "avars:rz"): -30.0,
                 _Attr(SWING, "avars:ry"): 15.0,
                 _Attr(SWING, "avars:space"): 0.0}, 1.0),
        (HEAD, {_Attr(CHEST, "avars:rx"): -25.0,
                _Attr(HEAD, "avars:rx"): -10.0,
                _Attr(HEAD, "avars:space"): 1.0}, 0.0),
        # Full: an IK hand moved in world, carried into the chest.
        (C + "/L_ArmIK", {_Attr(CHEST, "avars:rz"): -20.0,
                          _Attr(C + "/L_ArmIK", "avars:tx"): 5.0,
                          _Attr(C + "/L_ArmIK", "avars:rz"): 30.0,
                          _Attr(C + "/L_ArmIK", "avars:space"): 0.0}, 1.0),
    ]
    for control, pose, value in cases:
        session.base = dict(pose)
        before = session.Frame(control, {})
        space = _Attr(control, "avars:space")
        _Check(spaceMatch.SwitchTarget(session.stage, space)
               == Sdf.Path(control),
               "SwitchTarget finds %s from its dial" % control)
        plan = spaceMatch.Plan(session.stage, control, space, value, TIME,
                               session.Evaluate, current=session.base)
        after = session.Frame(control, plan)
        unmatched = session.Frame(control, {space: value})
        _Check(_Max(after, before) < _POSITION,
               "%s switched to space %g stays put (%.3g off)"
               % (control, value, _Max(after, before)))
        _Check(_Max(unmatched, before) > 1e-3,
               "%s would have moved without matching" % control)
    session.base = {}


def main():
    baked = _Session()
    TestSpaceLists(baked)
    TestRotationOnly(baked)
    fresh = _Session()
    TestParity(baked, fresh)
    TestMatch(baked)
    if _failures:
        print("%d failure(s)" % len(_failures))
        return 1
    print("test_space_match: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
