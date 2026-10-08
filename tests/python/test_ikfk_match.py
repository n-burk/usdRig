"""IK/FK matching reproduces the limb's joints exactly.

Headless test for plugin/rigExecUsdview/ikfkMatch.py on the biped:

  1. Discovery finds the four limbs from the rig's own wiring: the switch,
     both halves, the IK control above the effector (through a foot's roll
     stack), and the pole.
  2. On random poses, matching either half and switching to it moves none
     of the limb's joints, on all four limbs, in both evaluation modes.
  3. A round trip FK -> IK -> FK returns the FK channels the limb started
     with.
  4. A stretched IK limb is matched in FK by translation, never scale.
  5. A foot's roll channels are kept, not written, when a leg goes to IK.
  6. Rig-written channels are never written.
  7. Plan switches to the half that is not driving.

The pose under test is applied as interactive overrides, never authored.

Usage: test_ikfk_match.py [<generated schema resources dir>]
"""
import os
import random
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

from pxr import Usd  # noqa: E402

import ikfkMatch  # noqa: E402

_failures = []
# Exact up to the evaluator's own round-off.
_POSITION = 1e-6
_DEGREES = 1e-3


def _Check(condition, message):
    if not condition:
        _failures.append(message)
        print("FAIL: %s" % message)


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


TIME = Usd.TimeCode(1.0)


def _Hinge(limb):
    # Every biped limb bends its middle joint about rz, negative bending it
    # the way it already bends (measured; a two-bone IK bends one way only).
    return limb.fkControls[1].AppendProperty("avars:rz"), -1.0


def _RandomPose(limb, toIk, rng):
    """A pose on the half that is driving, the switch on that half."""
    pose = {limb.switchPath: limb.fkValue if toIk else limb.ikValue}
    if toIk:
        for control in (limb.fkControls[0], limb.fkControls[2]):
            for r in ("rx", "ry", "rz"):
                pose[control.AppendProperty("avars:" + r)] = \
                    rng.uniform(-40.0, 40.0)
        hinge, sign = _Hinge(limb)
        pose[hinge] = sign * rng.uniform(5.0, 70.0)
    else:
        for t in ("tx", "ty", "tz"):
            pose[limb.ikControl.AppendProperty("avars:" + t)] = \
                rng.uniform(-12.0, 12.0)
        for r in ("rx", "ry", "rz"):
            pose[limb.ikControl.AppendProperty("avars:" + r)] = \
                rng.uniform(-40.0, 40.0)
        for t in ("tx", "ty", "tz"):
            pose[limb.pole.AppendProperty("avars:" + t)] = \
                rng.uniform(-15.0, 15.0)
    return pose


def TestDiscovery(session):
    limbs = ikfkMatch.FindLimbs(session.stage)
    found = {l.switchControl.name: (l.ikControl.name, l.pole.name,
                                    [c.name for c in l.fkControls])
             for l in limbs}
    expected = {
        "L_Arm": ("L_ArmIK", "L_ArmPV", ["L_UpArm", "L_LoArm", "L_Hand"]),
        "R_Arm": ("R_ArmIK", "R_ArmPV", ["R_UpArm", "R_LoArm", "R_Hand"]),
        "L_Leg": ("L_LegIK", "L_LegPV", ["L_UpLeg", "L_LoLeg", "L_Foot"]),
        "R_Leg": ("R_LegIK", "R_LegPV", ["R_UpLeg", "R_LoLeg", "R_Foot"]),
    }
    _Check(found == expected, "discovery: %r" % found)
    toe = [l for l in limbs if l.switchControl.name == "L_Leg"][0]
    roll = session.stage.GetPrimAtPath(toe.effector).GetParent().GetPath()
    _Check(ikfkMatch.LimbFor(limbs, roll) is toe,
           "a control in the foot's roll stack belongs to its leg")
    return limbs


def TestRandomPoses(session, limbs, label):
    rng = random.Random(11)
    written = ikfkMatch.RigWritten(session.stage)
    for limb in limbs:
        session.base = {}
        rest = ikfkMatch.MeasureRest(limb, session.stage, session.Evaluate,
                                     TIME)
        for trial in range(5):
            for toIk in (True, False):
                session.base = _RandomPose(limb, toIk, rng)
                values, chosen = ikfkMatch.Plan(
                    limb, session.stage, session.Evaluate, TIME, toIk=toIk,
                    rest=rest, written=written, current=session.base)
                _Check(chosen == toIk, "plan honours toIk")
                position, degrees = ikfkMatch.Residual(
                    limb, session.stage, session.Evaluate, TIME, values)
                _Check(position < _POSITION and degrees < _DEGREES,
                       "%s %s %s trial %d: joints moved %.3g / %.3g deg"
                       % (label, limb.switchControl.name,
                          "FK->IK" if toIk else "IK->FK", trial, position,
                          degrees))


def TestRoundTrip(session, limbs):
    rng = random.Random(23)
    for limb in limbs:
        session.base = {}
        rest = ikfkMatch.MeasureRest(limb, session.stage, session.Evaluate,
                                     TIME)
        start = _RandomPose(limb, True, rng)
        session.base = dict(start)
        toIk, _ = ikfkMatch.Plan(limb, session.stage, session.Evaluate, TIME,
                                 toIk=True, rest=rest, current=session.base)
        session.base.update(toIk)
        back, _ = ikfkMatch.Plan(limb, session.stage, session.Evaluate, TIME,
                                 toIk=False, rest=rest, current=session.base)
        worst = 0.0
        for path, value in start.items():
            if path == limb.switchPath or path not in back:
                continue
            # Angles may come back a full turn away; the rig cannot tell.
            delta = (back[path] - value + 180.0) % 360.0 - 180.0
            worst = max(worst, abs(delta))
        _Check(worst < 1e-6, "%s round trip FK->IK->FK drifts %.3g"
               % (limb.switchControl.name, worst))


def TestStretch(session, limbs):
    limb = [l for l in limbs if l.switchControl.name == "L_Arm"][0]
    session.base = {}
    rest = ikfkMatch.MeasureRest(limb, session.stage, session.Evaluate, TIME)
    # Far beyond reach: the IK stretches the limb.
    session.base = {limb.switchPath: limb.ikValue,
                    limb.ikControl.AppendProperty("avars:tx"): 25.0,
                    limb.ikControl.AppendProperty("avars:ty"): -20.0}
    values, _ = ikfkMatch.Plan(limb, session.stage, session.Evaluate, TIME,
                               toIk=False, rest=rest, current=session.base)
    position, degrees = ikfkMatch.Residual(limb, session.stage,
                                           session.Evaluate, TIME, values)
    _Check(position < _POSITION and degrees < _DEGREES,
           "stretched IK -> FK moved joints %.3g / %.3g" % (position, degrees))
    scales = [p for p in values if p.name in ("avars:sx", "avars:sy",
                                              "avars:sz")]
    _Check(not scales, "a match never writes scale: %r" % scales)
    # The limb stretches by its own channels, which lengthen the FK chain
    # as they do the IK: the stretch lands there, not on a translation.
    top = values.get(limb.switchControl.AppendProperty("avars:stretchTop"),
                     1.0)
    _Check(top > 1.01, "the stretch lands on stretchTop (%.4f)" % top)
    moved = values.get(limb.fkControls[1].AppendProperty("avars:tx"), 0.0)
    _Check(abs(moved) < 1e-4,
           "the FK translation stays put (tx %.3g)" % moved)


def TestFootRollKept(session, limbs):
    limb = [l for l in limbs if l.switchControl.name == "L_Leg"][0]
    session.base = {}
    rest = ikfkMatch.MeasureRest(limb, session.stage, session.Evaluate, TIME)
    rng = random.Random(5)
    session.base = _RandomPose(limb, True, rng)
    roll = limb.switchControl.AppendProperty("avars:footRoll")
    session.base[roll] = 25.0
    values, _ = ikfkMatch.Plan(limb, session.stage, session.Evaluate, TIME,
                               toIk=True, rest=rest, current=session.base)
    position, degrees = ikfkMatch.Residual(limb, session.stage,
                                           session.Evaluate, TIME, values)
    _Check(position < _POSITION and degrees < _DEGREES,
           "leg with foot roll -> IK moved joints %.3g / %.3g"
           % (position, degrees))
    touched = [p for p in values if p.GetPrimPath() not in
               (limb.ikControl, limb.pole, limb.switchControl)]
    _Check(not touched and roll not in values,
           "only the IK control, pole and switch are written: %r" % touched)


def TestRigWrittenUntouched(session, limbs):
    written = ikfkMatch.RigWritten(session.stage)
    rng = random.Random(3)
    for limb in limbs:
        session.base = _RandomPose(limb, True, rng)
        values, _ = ikfkMatch.Plan(limb, session.stage, session.Evaluate,
                                   TIME, toIk=True, current=session.base)
        overlap = [p for p in values if p in written]
        _Check(not overlap, "%s writes rig-written %r"
               % (limb.switchControl.name, overlap))


def TestPlanPicksTheOtherHalf(session, limbs):
    limb = limbs[0]
    session.base = {limb.switchPath: limb.fkValue}
    values, toIk = ikfkMatch.Plan(limb, session.stage, session.Evaluate,
                                  TIME, current=session.base)
    _Check(toIk and values[limb.switchPath] == limb.ikValue,
           "from FK, Plan switches to IK")
    session.base = {limb.switchPath: limb.ikValue}
    values, toIk = ikfkMatch.Plan(limb, session.stage, session.Evaluate,
                                  TIME, current=session.base)
    _Check(not toIk and values[limb.switchPath] == limb.fkValue,
           "from IK, Plan switches to FK")


def main():
    session = _Session()
    limbs = TestDiscovery(session)
    TestRandomPoses(session, limbs, "graph")
    TestRoundTrip(session, limbs)
    TestStretch(session, limbs)
    TestFootRollKept(session, limbs)
    TestRigWrittenUntouched(session, limbs)
    TestPlanPicksTheOtherHalf(session, limbs)
    if _failures:
        print("%d failure(s)" % len(_failures))
        return 1
    print("test_ikfk_match: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
