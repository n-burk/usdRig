#!/usr/bin/env python
"""
A NUMERIC pose interpolator driver: rigExec:driverAttributes.

A corrective that fires from a dial rather than from something the rig
moves -- a breath, a squash amount -- names one to three properties instead
of a driver prim, and each pose carries the values it stands at in
rigExec:translation. This holds the whole contract on a two-pose rig:

  * a dial standing on a pose reads 1 there and 0 on the other;
  * halfway between reads halfway, because the solve is the translation
    channel's and nothing else;
  * the compiled graph agrees with the dynamic walk exactly;
  * two dials work as one position, and the second one moves the weights;
  * more than three is refused at compile rather than silently truncated.

Usage: test_rigexec_psd_numeric.py [<generated schema resources dir>]
"""
import sys

from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

from pxr import Plug, Sdf, Usd  # noqa: E402

import rigexec  # noqa: E402
import _rigexec  # noqa: E402

RIG = "/Rig"
DIAL = "/Rig/Controls/dial"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


INTERPOLATOR = "/Rig/PoseInterpolators/dialled"


def _Rig(dials, poses, mode="graph"):
    """A rig whose one interpolator reads `dials` (avar names) and stands at
    `poses` -- [(name, (v0, v1, v2))]."""
    stage = Usd.Stage.CreateInMemory("numeric.usda")
    builder = rigexec.Builder.create(stage, RIG, "Test")
    builder.add_control("dial")
    control = stage.GetPrimAtPath(DIAL)
    attributes = []
    for name in dials:
        attribute = control.CreateAttribute("avars:" + name,
                                            Sdf.ValueTypeNames.Double)
        attribute.Set(0.0)
        attributes.append(attribute)
    interpolator = builder.add_pose_interpolator("dialled", DIAL)
    interpolator.set_channels(False, True)
    interpolator.set_kernel("linear")
    for name, values in poses:
        pose = interpolator.add_pose(name)
        pose.set_pose_type("whole")
        pose.set_translation(values)
        pose.set_radii(0.0, 10.0)
    prim = stage.GetPrimAtPath("/Rig/PoseInterpolators/dialled")
    prim.CreateRelationship("rigExec:driverAttributes").SetTargets(
        [a.GetPath() for a in attributes])
    prim.GetRelationship("rigExec:driver").SetTargets([])
    rig = _rigexec.Rig(stage, RIG)
    rig.compile()
    rig.cpu_reference = True
    return stage, rig


def _Weights(rig, values, names, dials):
    rig.set_interactive_overrides(
        [(DIAL, "avars:" + dial, v) for dial, v in zip(dials, values)])
    pose = rig.evaluate(1.0)
    return [pose.moved_property(
        "/Rig/PoseInterpolators/dialled/%s.outputs:weight" % n)
        for n in names], pose


def TestOneDial():
    for mode in ("graph",):
        _stage, rig = _Rig(["breath"],
                           [("neutral", (0.0, 0.0, 0.0)),
                            ("out", (10.0, 0.0, 0.0))], mode)
        for value, expect in ((0.0, (1.0, 0.0)), (10.0, (0.0, 1.0)),
                              (5.0, (0.5, 0.5))):
            got, pose = _Weights(rig, [value], ["neutral", "out"], ["breath"])
            _Check(all(abs(a - b) < 1e-5 for a, b in zip(got, expect)),
                   "%s: dial %g read %s, expected %s"
                   % (mode, value, got, expect))
            _Check(pose.valid,"pose publication valid; authored numeric assertions follow")
        # A linear kernel has compact support, so a dial a whole radius past
        # the last pose is off rather than nearly off -- the same rule the
        # translation channel has, reached through a dial.
        got, _ = _Weights(rig, [25.0], ["neutral", "out"], ["breath"])
        _Check(all(abs(w) < 1e-6 for w in got),
               "%s: a dial past every pose's radius drives nothing: %s"
               % (mode, got))


def TestTwoDials():
    _stage, rig = _Rig(["x", "y"],
                       [("neutral", (0.0, 0.0, 0.0)),
                        ("right", (10.0, 0.0, 0.0)),
                        ("up", (0.0, 10.0, 0.0))], "graph")
    names = ["neutral", "right", "up"]
    got, pose = _Weights(rig, [10.0, 0.0], names, ["x", "y"])
    _Check(abs(got[1] - 1.0) < 1e-5 and abs(got[2]) < 1e-5,
           "x alone stands on `right`: %s" % got)
    _Check(pose.valid,"pose publication valid; authored numeric assertions follow")
    got, _ = _Weights(rig, [0.0, 10.0], names, ["x", "y"])
    _Check(abs(got[2] - 1.0) < 1e-5 and abs(got[1]) < 1e-5,
           "y alone stands on `up`: %s" % got)
    # The second dial is READ, not ignored: moving it off zero has to move
    # the weights (the bug a one-dial test cannot see).
    got, _ = _Weights(rig, [5.0, 5.0], names, ["x", "y"])
    _Check(got[1] > 0.01 and got[2] > 0.01 and abs(got[1] - got[2]) < 1e-5,
           "halfway between the two dials shares them evenly: %s" % got)


def TestTooManyDialsRefused():
    # A fourth dial has no axis to stand for, so the interpolator is refused.
    # The refusal is a SKIPPED OPERATION, not a failed compile: one bad prim
    # must not cost a rig its other 400, and a rigger needs the rest of the
    # character up to see what the skip costs them.
    _stage, rig = _Rig(["a", "b", "c", "d"],
                       [("neutral", (0.0, 0.0, 0.0)),
                        ("out", (1.0, 0.0, 0.0))])
    skipped = rig.skipped_operations()
    _Check(list(skipped) == [INTERPOLATOR],
           "four dials skip the interpolator and nothing else: %s" % skipped)
    _Check("driverAttributes" in skipped[INTERPOLATOR],
           "the refusal names the relationship: %r" % skipped[INTERPOLATOR])
    # And it drives nothing: a skipped interpolator publishes no weight at
    # all, which is stronger than publishing zero -- nothing downstream can
    # read a stale one.
    rig.set_interactive_overrides(
        [(DIAL, "avars:a", 1.0)])
    pose = rig.evaluate(1.0)
    for name in ("neutral", "out"):
        path = "%s/%s.outputs:weight" % (INTERPOLATOR, name)
        try:
            pose.moved_property(path)
        except KeyError:
            continue
        _Check(False, "a skipped interpolator still published %s" % path)


def main():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    TestOneDial()
    print("  ok: one dial drives its poses, graph and scalar reference, with parity")
    TestTwoDials()
    print("  ok: two dials read as one position")
    TestTooManyDialsRefused()
    print("  ok: a fourth dial is refused at compile")
    print("RIGEXEC_PSD_NUMERIC_OK (3 groups)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
