#!/usr/bin/env python
"""
What the evaluator is allowed to settle once per epoch, and what it must
still re-read on every frame.

  * a provider's rest frame, which is a function of rest:space and the six
    rest avars of the provider and its RigExec ancestors. The rests are
    pulled once at Compile and refreshed when an edit arrives, so an edit to
    a rest channel must still move the joint -- with a recompile, and also
    without one, because the epoch digest hashes wiring, not values.

  * a static input read (inputs:enabled, inputs:defaultWeight, offsets), held
    per evaluator for values that are neither connected nor time-varying. An
    edit to such an input must reach the very next evaluate, and an input
    that IS connected or time-sampled must never be held at all.

  * the pose-seed request's warm compute, moved into Compile. It happens at
    the stage's start time code, and the frame asked for first can be any
    other frame, which must still get its own values.

Usage: test_epoch_constants.py [<generated schema resources dir>]
Requires the native _rigexec binding (build-python/python on PYTHONPATH).
"""
import os
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXAMPLE = os.path.normpath(os.path.join(
    _HERE, "..", "..", "examples", "08_AimEyes.usda"))
_RIG = "/EyesAsset/Rig"
_EYE_L = "/EyesAsset/Rig/Joints/EyeL"
_EYE_R = "/EyesAsset/Rig/Joints/EyeR"
_AIM_L = "/EyesAsset/Rig/Movers/Pose/AimL"
_AIM_R = "/EyesAsset/Rig/Movers/Pose/AimR"
# The aim weights ramp from 0 at 1001 to 1 at 1012 and hold; the look-at
# control swings away from its default position over the shot and is back at
# it by 1048, so 1032 is the frame that disagrees most with a Default read.
_UNAIMED, _AIMED, _SWUNG = 1001.0, 1048.0, 1032.0


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecJoint"),
           "RigExecJoint schema is not registered")


def _Open():
    """The example, with edits going to the session layer."""
    stage = Usd.Stage.Open(_EXAMPLE)
    _Check(stage, "example stage opens")
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    return stage


def _Origin(pose, joint):
    frame = pose.joint_frame(joint, True)
    _Check(frame.valid, "%s has no valid frame" % joint)
    return Gf.Vec3d(*frame.origin)


def _ZAxis(pose, joint):
    frame = pose.joint_frame(joint, True)
    _Check(frame.valid, "%s has no valid frame" % joint)
    return Gf.Vec3d(*frame.z_axis)


def _MoveRest(stage, joint, dx):
    """Translates a joint's authored rest:space by dx in x."""
    attribute = stage.GetPrimAtPath(joint).GetAttribute("rest:space")
    _Check(attribute, "%s authors no rest:space" % joint)
    matrix = Gf.Matrix4d(attribute.Get())
    row = matrix.GetRow(3)
    matrix.SetRow(3, Gf.Vec4d(row[0] + dx, row[1], row[2], row[3]))
    attribute.Set(matrix)


def _Pose(rig, time):
    """Everything a stale rest frame would show up in.

    joint_matrix is the joint's rest-to-pose map, so it is the direct
    reading of the rest frames; the origins come along to catch anything
    that moves the pose without moving the map.
    """
    pose = rig.evaluate(time)
    _Check(pose.valid, "pose at %g is not valid" % time)
    return {joint: (tuple(pose.joint_matrix(joint)),
                    tuple(pose.joint_frame(joint, True).origin))
            for joint in (_EYE_L, _EYE_R)}


def _CheckSame(subject, reference, message):
    for joint, (matrix, origin) in reference.items():
        got = subject[joint]
        for a, b in zip(got[0], matrix):
            _Check(abs(a - b) < 1e-12,
                   "%s: %s matrix %s != %s" % (message, joint, got[0], matrix))
        for a, b in zip(got[1], origin):
            _Check(abs(a - b) < 1e-12,
                   "%s: %s origin %s != %s" % (message, joint, got[1], origin))


def _CheckDiffers(subject, reference, message):
    for joint, (matrix, origin) in reference.items():
        if any(abs(a - b) > 1e-9 for a, b in zip(subject[joint][0], matrix)):
            return
    raise AssertionError(message)


def _Reference(edit):
    """The pose of a stage that carried the edit before it was compiled.

    A rest edit made before the compile cannot be stale by construction, so
    this is the answer the same edit made after the compile owes.
    """
    import _rigexec
    stage = _Open()
    edit(stage)
    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    return _Pose(rig, _AIMED)


def TestRestEditAfterRecompile():
    """A rest edit plus an explicit recompile rebuilds the epoch's rests."""
    import _rigexec
    edit = lambda stage: _MoveRest(stage, _EYE_L, 3.0)  # noqa: E731
    reference = _Reference(edit)
    stage = _Open()
    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    before = _Pose(rig, _AIMED)
    _CheckDiffers(before, reference,
                  "the 3.0 rest edit changes no joint matrix at all; this "
                  "test needs an edit the pose can be stale about")
    edit(stage)
    rig.compile()
    _CheckSame(_Pose(rig, _AIMED), reference,
               "after a rest edit and an explicit recompile")


def TestRestEditWithoutRecompile():
    """A rest edit alone reaches the next evaluate.

    A value edit does not change the epoch digest -- the digest hashes
    wiring, sample counts and cardinalities, never values -- so nothing
    recompiles here, and the epoch's rest frames have to be refreshed on
    their own.
    """
    import _rigexec
    edit = lambda stage: _MoveRest(stage, _EYE_L, 3.0)  # noqa: E731
    reference = _Reference(edit)
    stage = _Open()
    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    digest = rig.binding_epoch_digest()
    _Pose(rig, _AIMED)
    edit(stage)
    after = _Pose(rig, _AIMED)
    _Check(rig.binding_epoch_digest() == digest,
           "a rest value edit changed the epoch digest; this test asserts "
           "the no-recompile path and needs an edit that does not recompile")
    _CheckSame(after, reference, "after a rest edit with no recompile")


def TestRestEditOnTheOtherEye():
    """The refreshed rests are the whole map, not the one prim that moved."""
    import _rigexec
    edit = lambda stage: _MoveRest(stage, _EYE_R, -2.0)  # noqa: E731
    reference = _Reference(edit)
    stage = _Open()
    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    before = _Pose(rig, _UNAIMED)
    edit(stage)
    _CheckDiffers(_Pose(rig, _AIMED), before,
                  "the -2.0 rest edit changed nothing")
    _CheckSame(_Pose(rig, _AIMED), reference,
               "after a rest edit on the other eye")


def TestFirstFrameIsNotTheWarmedFrame():
    """The frame asked for first gets its own values, not the warm's.

    Compile warms the pose-seed request so the first evaluate does not pay
    for the whole network; that warm happens at the stage's start time, and
    the first evaluate can be any other frame. Asking for a frame once right
    after the compile and again after another frame has to give one answer:
    if the warm decided it, the first answer is the warm's and the third is
    the frame's.

    The warm is deliberately not taken at Default -- a Default pull leaves
    time-independent values behind that a later frame does not invalidate.
    The case that catches THAT is the autoDetect single-chain IK in
    testRigExecConstraints; this one is the general invariant.
    """
    import _rigexec
    stage = _Open()
    # An avar that exists only as time samples: read at Default it is the
    # schema fallback, and a warm that took that reading would hold the
    # look-at control 10 units away from where 1032 puts it.
    look = stage.GetPrimAtPath("/EyesAsset/Rig/Controls/LookAt")
    tz = look.GetAttribute("avars:tz")
    _Check(tz, "LookAt has no avars:tz")
    tz.Set(0.0, _UNAIMED)
    tz.Set(10.0, _SWUNG)
    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    first = _Pose(rig, _SWUNG)
    _Pose(rig, _UNAIMED)
    third = _Pose(rig, _SWUNG)
    _CheckSame(first, third,
               "the first evaluate of %g after a compile" % _SWUNG)


def TestEnabledEditFollows():
    """Flipping a held input reaches the very next evaluate.

    inputs:enabled is exactly the shape the static cache admits: authored,
    unconnected, no time samples. It is authored true before the compile so
    the first evaluate reads and holds it, then flipped -- with no recompile,
    because a value edit does not change the epoch digest.
    """
    import _rigexec
    stage = _Open()
    enabled = stage.GetPrimAtPath(_AIM_L).GetAttribute("inputs:enabled")
    _Check(enabled, "AimL has no inputs:enabled attribute")
    enabled.Set(True)
    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    digest = rig.binding_epoch_digest()
    aimed = _ZAxis(rig.evaluate(_AIMED), _EYE_L)
    enabled.Set(False)
    off = _ZAxis(rig.evaluate(_AIMED), _EYE_L)
    _Check(rig.binding_epoch_digest() == digest,
           "disabling the constraint changed the epoch digest; this test "
           "asserts the no-recompile path")
    _Check((aimed - off).GetLength() > 1e-6,
           "EyeL aimed the same way with the constraint disabled (%s vs %s); "
           "the held inputs:enabled outlived the edit" % (aimed, off))
    enabled.Set(True)
    again = _ZAxis(rig.evaluate(_AIMED), _EYE_L)
    _Check((aimed - again).GetLength() < 1e-9,
           "re-enabling the constraint did not restore the aim (%s vs %s)"
           % (aimed, again))


def TestTimeSampledWeightIsNeverHeld():
    """A time-sampled input is re-read on every frame.

    The aim weights ramp 0 -> 1 over 1001..1012. Holding the first frame's
    weight would leave the eyes unaimed for the whole shot.
    """
    import _rigexec
    stage = _Open()
    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    unaimed = _ZAxis(rig.evaluate(_UNAIMED), _EYE_L)
    aimed = _ZAxis(rig.evaluate(_AIMED), _EYE_L)
    _Check((aimed - unaimed).GetLength() > 1e-6,
           "EyeL aims identically at %g and %g (%s); the time-sampled "
           "inputs:defaultWeight was read once and held"
           % (_UNAIMED, _AIMED, aimed))
    # ... and reading it in the other order gives the same two answers.
    back = _ZAxis(rig.evaluate(_UNAIMED), _EYE_L)
    _Check((back - unaimed).GetLength() < 1e-9,
           "EyeL did not return to its unaimed frame at %g" % _UNAIMED)


def TestConnectedWeightIsNeverHeld():
    """A connected input follows its source, however the source moves."""
    import _rigexec
    stage = _Open()
    control = stage.GetPrimAtPath("/EyesAsset/Rig/Controls/LookAt")
    driver = control.CreateAttribute(
        "rigExecTestWeight", Sdf.ValueTypeNames.Float, custom=True)
    driver.Set(0.0, _UNAIMED)
    driver.Set(1.0, _AIMED)
    weight = stage.GetPrimAtPath(_AIM_R).GetAttribute("inputs:defaultWeight")
    _Check(weight, "AimR has no inputs:defaultWeight attribute")
    # The authored spline would win over the connection; clear it so the
    # connection is the only opinion this test is reading.
    weight.Clear()
    weight.AddConnection(driver.GetPath())
    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    unaimed = _ZAxis(rig.evaluate(_UNAIMED), _EYE_R)
    aimed = _ZAxis(rig.evaluate(_AIMED), _EYE_R)
    _Check((aimed - unaimed).GetLength() > 1e-6,
           "EyeR aims identically at %g and %g (%s); the connected "
           "inputs:defaultWeight was read once and held"
           % (_UNAIMED, _AIMED, aimed))


def main():
    _RegisterSchema()
    groups = [
        ("rest edit after recompile", TestRestEditAfterRecompile),
        ("rest edit without recompile", TestRestEditWithoutRecompile),
        ("rest edit on the other eye", TestRestEditOnTheOtherEye),
        ("first frame is not the warmed frame",
         TestFirstFrameIsNotTheWarmedFrame),
        ("edit to a held input follows", TestEnabledEditFollows),
        ("time-sampled input is never held", TestTimeSampledWeightIsNeverHeld),
        ("connected input is never held", TestConnectedWeightIsNeverHeld),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_EPOCH_CONSTANTS_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
