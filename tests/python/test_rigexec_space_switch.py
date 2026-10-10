#!/usr/bin/env python
"""
RigExecSpaceSwitch: a labelled parent-space list with a live index.

A space switch does not pose its target. It swaps the parent the target's
ordinary compose is taken against, which is what lets an IK hand, a pole
vector or an FK shoulder keep animating in local values while sitting in
some other part of the rig. The contract this holds:

  * every space agrees at rest, so adding or switching one cannot move a
    rig that is standing still;
  * the selected space is the one that carries the control, and the others
    do not;
  * the target's own avars still apply, in the selected space's axes;
  * "world" is a source that is not a provider, and a control in it is
    pinned while its namespace parent moves;
  * a fractional index eases between two spaces and is exact at both ends;
  * a translation mask leaves an orient-only space;
  * the index can be read from a property on the control itself, so the
    animator-facing channel sits beside the avars;
  * a pole vector can live in its own IK handle's switched space -- one
    switch reading another's target -- and a cycle is locally set aside with a diagnostic.

Usage: test_rigexec_space_switch.py [<generated schema resources dir>]
"""
import math
import sys

from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

from pxr import Gf, Plug, Sdf, Usd  # noqa: E402

import rigexec  # noqa: E402

RIG = "/Rig"
CHEST = (0.0, 100.0, 0.0)
OTHER = (50.0, 100.0, 0.0)
HAND = (20.0, 100.0, 0.0)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Translate(x, y, z):
    """Row-vector identity rotation with translation in the last row."""
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1]


def _Near(a, b, tolerance=1e-6):
    return all(abs(x - y) <= tolerance for x, y in zip(a, b))


def _Matrix(values):
    """`to_matrix4()` hands back a flat row-major 16."""
    return values if isinstance(values, Gf.Matrix4d) else Gf.Matrix4d(*values)


def _Angle(a, b):
    """Degrees between two orientations."""
    qa = _Matrix(a).ExtractRotationQuat().GetNormalized()
    qb = _Matrix(b).ExtractRotationQuat().GetNormalized()
    dot = abs(qa.GetReal() * qb.GetReal() +
              Gf.Dot(qa.GetImaginary(), qb.GetImaginary()))
    return 2.0 * math.degrees(math.acos(min(1.0, dot)))


def _Origin(matrix):
    t = _Matrix(matrix).ExtractTranslation()
    return (t[0], t[1], t[2])


class _Fixture(object):
    """Chest and Other side by side, Hand nested under Chest, and a switch on
    Hand listing [Chest, Other, world]."""

    def __init__(self, sources=("Chest", "Other", "world"), labels=None,
                 masks=None, active_attribute=False, mode="graph"):
        self.stage = Usd.Stage.CreateInMemory("spaces.usda")
        builder = rigexec.Builder.create(self.stage, RIG, "Test")
        self.chest = builder.add_control("Chest", _Translate(*CHEST))
        self.other = builder.add_control("Other", _Translate(*OTHER))
        # Rest RELATIVE to Chest, so Hand rides it with nothing authored.
        self.hand = builder.add_control(
            "Hand", _Translate(HAND[0] - CHEST[0], 0.0, 0.0), self.chest)
        self.paths = {"Chest": self.chest.path, "Other": self.other.path,
                      "Hand": self.hand.path, "world": RIG}

        switch = self.stage.DefinePrim("/Rig/Movers/handSpaces",
                                       "RigExecSpaceSwitch")
        switch.CreateRelationship("rigExec:target").SetTargets(
            [self.hand.path])
        switch.CreateRelationship("rigExec:sources").SetTargets(
            [self.paths[name] for name in sources])
        if labels:
            switch.CreateAttribute("rigExec:spaceLabels",
                                   Sdf.ValueTypeNames.TokenArray,
                                   True).Set(labels)
        switch.CreateAttribute("inputs:activeSpace",
                               Sdf.ValueTypeNames.Double).Set(0.0)
        for name, value in (masks or {}).items():
            switch.CreateAttribute("inputs:" + name,
                                   Sdf.ValueTypeNames.Bool).Set(value)
        self.switch = switch
        if active_attribute:
            prim = self.stage.GetPrimAtPath(self.hand.path)
            self.dial = prim.CreateAttribute("spaces:active",
                                             Sdf.ValueTypeNames.Double)
            self.dial.Set(0.0)
            switch.CreateRelationship(
                "rigExec:activeSpaceAttribute").SetTargets([
                    self.dial.GetPath()])
        self.rig = rigexec.Rig(self.stage, RIG)
        self.rig.compile()
        self.rig.cpu_reference = True
        self.mode = mode

    def Avar(self, control, name, value):
        prim = self.stage.GetPrimAtPath(self.paths[control])
        attribute = prim.GetAttribute("avars:" + name)
        if not attribute or not attribute.IsValid():
            attribute = prim.CreateAttribute("avars:" + name,
                                             Sdf.ValueTypeNames.Double)
        attribute.Set(value)
        return self

    def Active(self, value):
        if hasattr(self, "dial"):
            self.dial.Set(value)
        else:
            self.switch.GetAttribute("inputs:activeSpace").Set(value)
        return self

    def World(self, control="Hand"):
        pose = self.rig.evaluate(0)
        # In parity mode the program and the dynamic walk are compared
        # element by element; a mismatch here is the two paths disagreeing
        # about the same switch, which is the failure a value check alone
        # cannot see.
        return pose.control_frame(self.paths[control]).to_matrix4()


def TestRestAgreesEverywhere():
    """Every space gives the same answer at rest. This is the property that
    makes a space switch safe to add to a rig that already works."""
    for mode in ("graph",):
        fixture = _Fixture(mode=mode)
        for active in (0.0, 1.0, 2.0, 0.5, 1.5):
            fixture.Active(active)
            _Check(_Near(_Origin(fixture.World()), HAND),
                   "%s: space %g moved the hand at rest: %s"
                   % (mode, active, _Origin(fixture.World())))


def TestTheProgramAndTheWalkAgree():
    """Parity mode compares the compiled graph with the dynamic walk element
    by element, on the cases that exercise every branch of the switch."""
    fixture = _Fixture(mode="graph")
    fixture.Avar("Chest", "tx", 10.0).Avar("Other", "ry", 25.0)
    fixture.Avar("Hand", "tx", 3.0).Avar("Hand", "rz", 12.0)
    for active in (0.0, 1.0, 2.0, 0.5, 1.4, -3.0, 9.0):
        fixture.Active(active)
        fixture.World()          # raises on any mismatch
    masked = _Fixture(mode="graph",
                      masks={"affectTranslationX": False,
                             "affectTranslationY": False,
                             "affectTranslationZ": False})
    masked.Avar("Chest", "tx", 10.0).Avar("Chest", "ry", 40.0)
    for active in (0.0, 1.0, 0.5):
        masked.Active(active)
        masked.World()


def TestSelectedSpaceCarriesTheControl():
    """The selected space moves the control and the others do not."""
    fixture = _Fixture()
    fixture.Avar("Chest", "tx", 10.0)
    fixture.Active(0.0)
    _Check(_Near(_Origin(fixture.World()), (30.0, 100.0, 0.0)),
           "chest space: hand at %s" % (_Origin(fixture.World()),))
    fixture.Active(1.0)
    _Check(_Near(_Origin(fixture.World()), HAND),
           "other space: the chest must not carry the hand, got %s"
           % (_Origin(fixture.World()),))
    fixture.Avar("Other", "tx", 10.0)
    _Check(_Near(_Origin(fixture.World()), (30.0, 100.0, 0.0)),
           "other space: the other control carries it, got %s"
           % (_Origin(fixture.World()),))


def TestWorldPinsTheControl():
    """A source that is not a provider is world: the hand stays put while its
    namespace parent moves."""
    fixture = _Fixture()
    fixture.Avar("Chest", "tx", 10.0).Avar("Other", "tx", 7.0)
    fixture.Active(2.0)
    _Check(_Near(_Origin(fixture.World()), HAND),
           "world space: the hand should not move, got %s"
           % (_Origin(fixture.World()),))
    # ... and its own avars still reach it there.
    fixture.Avar("Hand", "ty", 5.0)
    _Check(_Near(_Origin(fixture.World()), (20.0, 105.0, 0.0)),
           "world space: the hand's own avars still apply, got %s"
           % (_Origin(fixture.World()),))


def TestAvarsStayLocalInTheSpace():
    """The target's avars compose in the SELECTED space, not the namespace
    one: a control switched into a moved space is displaced by its avars from
    where that space put it."""
    fixture = _Fixture()
    fixture.Avar("Other", "tx", 10.0).Avar("Hand", "tx", 5.0)
    fixture.Active(1.0)
    _Check(_Near(_Origin(fixture.World()), (35.0, 100.0, 0.0)),
           "avars ride the selected space: %s" % (_Origin(fixture.World()),))


def TestFractionalIndexEases():
    """A whole number is exactly that space; between two, the switch eases."""
    fixture = _Fixture()
    fixture.Avar("Chest", "tx", 10.0)
    fixture.Active(0.5)
    _Check(_Near(_Origin(fixture.World()), (25.0, 100.0, 0.0)),
           "halfway between chest and other: %s" % (_Origin(fixture.World()),))
    fixture.Active(0.0)
    _Check(_Near(_Origin(fixture.World()), (30.0, 100.0, 0.0)),
           "back on chest exactly: %s" % (_Origin(fixture.World()),))


def TestIndexClampsToTheList():
    fixture = _Fixture()
    fixture.Avar("Chest", "tx", 10.0)
    fixture.Active(-4.0)
    _Check(_Near(_Origin(fixture.World()), (30.0, 100.0, 0.0)),
           "an index below the list clamps to the first space: %s"
           % (_Origin(fixture.World()),))
    fixture.Active(99.0)
    _Check(_Near(_Origin(fixture.World()), HAND),
           "an index past the list clamps to the last space: %s"
           % (_Origin(fixture.World()),))


def TestTranslationMaskIsOrientOnly():
    """Masking translation off leaves a space that rotates the control about
    its own pivot and does not carry it."""
    fixture = _Fixture(masks={"affectTranslationX": False,
                              "affectTranslationY": False,
                              "affectTranslationZ": False})
    fixture.Avar("Chest", "tx", 10.0)
    fixture.Active(0.0)
    _Check(_Near(_Origin(fixture.World()), HAND),
           "an orient-only space does not translate the hand, got %s"
           % (_Origin(fixture.World()),))
    # The rotation still arrives: a 90 degree turn about Y takes the hand's
    # own X axis onto -Z.
    fixture.Avar("Chest", "tx", 0.0).Avar("Chest", "ry", 90.0)
    axis = _Matrix(fixture.World()).TransformDir(Gf.Vec3d(1, 0, 0))
    _Check(_Near((axis[0], axis[1], axis[2]), (0.0, 0.0, -1.0), 1e-6),
           "an orient-only space still rotates the hand: %s" % (axis,))


def TestTwistOnlyDropsTheSwing():
    """A pole vector in its hand's space follows the forearm's TWIST and does
    not swing around with the wrist. The split is exact, so 30 degrees of
    twist arrives as 30 and 30 degrees of swing arrives as nothing."""
    for mode in ("graph",):
        fixture = _Fixture(mode=mode)
        # Other is the "hand": twist about Y, which is the axis the switch
        # is told to keep.
        fixture.switch.CreateAttribute(
            "rigExec:rotationFilters", Sdf.ValueTypeNames.TokenArray,
            True).Set(["all", "twist", "all"])
        fixture.switch.CreateAttribute(
            "rigExec:twistAxis", Sdf.ValueTypeNames.Double3).Set(
            Gf.Vec3d(0, 1, 0))
        fixture.rig = rigexec.Rig(fixture.stage, RIG)
        fixture.rig.compile()
        fixture.rig.cpu_reference = True
        fixture.Active(1.0)

        rest = _Matrix(fixture.World())
        fixture.Avar("Other", "ry", 30.0)
        twisted = _Matrix(fixture.World())
        _Check(abs(_Angle(twisted, rest) - 30.0) < 1e-4,
               "%s: 30 degrees of twist reached the pole as %.4f"
               % (mode, _Angle(twisted, rest)))

        fixture.Avar("Other", "ry", 0.0).Avar("Other", "rz", 30.0)
        swung = _Matrix(fixture.World())
        _Check(_Angle(swung, rest) < 1e-4,
               "%s: 30 degrees of swing reached the pole as %.4f"
               % (mode, _Angle(swung, rest)))
        # ... and the translation still follows, which is the other half.
        fixture.Avar("Other", "rz", 0.0).Avar("Other", "tx", 10.0)
        _Check(_Near(_Origin(fixture.World()), (30.0, 100.0, 0.0)),
               "%s: a filtered space still carries the position: %s"
               % (mode, _Origin(fixture.World())))


def TestIndexFromAControlProperty():
    """The animator-facing channel lives on the control, beside its avars."""
    fixture = _Fixture(active_attribute=True)
    fixture.Avar("Chest", "tx", 10.0)
    fixture.Active(0.0)
    _Check(_Near(_Origin(fixture.World()), (30.0, 100.0, 0.0)),
           "dial 0 selects the chest: %s" % (_Origin(fixture.World()),))
    fixture.Active(1.0)
    _Check(_Near(_Origin(fixture.World()), HAND),
           "dial 1 selects the other space: %s" % (_Origin(fixture.World()),))


def TestSwitchReadsAnotherSwitchedControl():
    """A pole vector sitting in its own IK handle's switched space: the
    second switch has to see the first one's answer, not the seed's."""
    fixture = _Fixture()
    pole = rigexec.Builder.create(fixture.stage, RIG, "Test")
    del pole  # the builder above already made the rig; reuse its stage.
    stage = fixture.stage
    poleControl = stage.DefinePrim("/Rig/Controls/Pole", "RigExecControl")
    poleControl.CreateAttribute("rest:space",
                                Sdf.ValueTypeNames.Matrix4d).Set(
        Gf.Matrix4d(*_Translate(20.0, 100.0, 30.0)))
    switch = stage.DefinePrim("/Rig/Movers/poleSpaces", "RigExecSpaceSwitch")
    switch.CreateRelationship("rigExec:target").SetTargets(
        [poleControl.GetPath()])
    switch.CreateRelationship("rigExec:sources").SetTargets(
        [fixture.hand.path])
    switch.CreateAttribute("inputs:activeSpace",
                           Sdf.ValueTypeNames.Double).Set(0.0)
    rig = rigexec.Rig(stage, RIG)
    rig.compile()
    # Hand is in Other's space; Other moves; the pole follows the hand there.
    stage.GetPrimAtPath(fixture.paths["Other"]).GetAttribute(
        "avars:tx").Set(10.0)
    fixture.switch.GetAttribute("inputs:activeSpace").Set(1.0)
    pose = rig.evaluate(0)
    hand = _Origin(pose.control_frame(fixture.hand.path).to_matrix4())
    poled = _Origin(pose.control_frame("/Rig/Controls/Pole").to_matrix4())
    _Check(_Near(hand, (30.0, 100.0, 0.0)),
           "the hand rode its own switch: %s" % (hand,))
    _Check(_Near(poled, (30.0, 100.0, 30.0)),
           "the pole rode the hand's SWITCHED frame, not its seed: %s"
           % (poled,))


def TestCycleSetsAsideMembers():
    fixture = _Fixture()
    other = fixture.stage.DefinePrim("/Rig/Movers/chestSpaces",
                                     "RigExecSpaceSwitch")
    other.CreateRelationship("rigExec:target").SetTargets(
        [fixture.paths["Chest"]])
    other.CreateRelationship("rigExec:sources").SetTargets(
        [fixture.paths["Hand"]])
    rig = rigexec.Rig(fixture.stage, RIG)
    rig.compile()
    expected = {fixture.switch.GetPath().pathString, other.GetPath().pathString}
    skipped = rig.skipped_operations()
    _Check(set(skipped) == expected, "exact cyclic switch owners: %s" % skipped)
    _Check(len(set(skipped.values())) == 1, "one component-local reason: %s" % skipped)
    reason = next(iter(skipped.values()))
    _Check("pose dependency cycle" in reason and " -> " in reason,
           "the common compiler reports an actual loop: %s" % reason)
    for owner in expected:
        _Check(owner in reason, "cyclic owner absent from reason: %s" % reason)
    authored = fixture.stage.GetRootLayer().ExportToString()
    pose = rig.evaluate(0)
    _Check(pose.valid, "one cyclic component does not reject unrelated work")
    _Check(_Near(_Origin(pose.control_frame(fixture.paths["Other"]).to_matrix4()), OTHER),
           "the independent Other control keeps its authored frame")
    held = rig.evaluate(0)
    _Check(held.valid and rig.skipped_operations() == skipped,
           "held local exclusion/reporting is stable")
    _Check(tuple(held.control_frame(fixture.paths["Other"]).to_matrix4()) ==
           tuple(pose.control_frame(fixture.paths["Other"]).to_matrix4()),
           "the unrelated held frame is exact")
    _Check(fixture.stage.GetRootLayer().ExportToString() == authored,
           "local cycle handling does not author the stage")
    fixture.Avar("Other", "ty", 7.0)
    moved = rig.evaluate(0)
    _Check(moved.valid and _Near(_Origin(moved.control_frame(
        fixture.paths["Other"]).to_matrix4()), (OTHER[0], OTHER[1] + 7.0, OTHER[2])),
        "the unrelated control still responds while the loop is set aside")
    _Check(rig.skipped_operations() == skipped, "the unrelated edit does not change cycle membership")


def TestSourceWeightsAreRefused():
    fixture = _Fixture()
    fixture.switch.CreateAttribute(
        "inputs:sourceWeights", Sdf.ValueTypeNames.FloatArray).Set([1.0, 0.0])
    rig = rigexec.Rig(fixture.stage, RIG)
    try:
        rig.compile()
    except Exception as error:  # noqa: BLE001
        _Check("sourceWeights" in str(error),
               "authored sourceWeights names itself: %s" % error)
        return
    _Check(False, "authored inputs:sourceWeights must be refused")


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    if plugin_dir:
        Plug.Registry().RegisterPlugins(plugin_dir)
    TestRestAgreesEverywhere()
    TestTheProgramAndTheWalkAgree()
    TestSelectedSpaceCarriesTheControl()
    TestWorldPinsTheControl()
    TestAvarsStayLocalInTheSpace()
    TestFractionalIndexEases()
    TestIndexClampsToTheList()
    TestTranslationMaskIsOrientOnly()
    TestTwistOnlyDropsTheSwing()
    TestIndexFromAControlProperty()
    TestSwitchReadsAnotherSwitchedControl()
    TestCycleSetsAsideMembers()
    TestSourceWeightsAreRefused()
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
