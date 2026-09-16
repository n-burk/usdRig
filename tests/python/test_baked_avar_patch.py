#
# Avar edits patch a baked program in place instead of rebuilding it.
#
# What a released gizmo drag and its undo author, in both authoring modes:
#   * the first value a layer holds for an avar creates its property spec
#     (a property RESYNC naming only typeName), and undo removes it (a
#     resync naming nothing);
#   * Animation mode keys the release as a spline knot, which turns a
#     constant avar into a per-frame input, and the undo turns it back.
# Each step must answer exactly what the dynamic evaluator answers (parity
# mode compares the two with exact equality) and must not rebake. A
# connection, which does change structure, still rebakes.
#
import os
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Plug, Sdf, Ts, Usd  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXAMPLE = os.path.normpath(os.path.join(
    _HERE, "..", "..", "examples", "08_AimEyes.usda"))
_RIG = "/EyesAsset/Rig"
_LOOK = "/EyesAsset/Rig/Controls/LookAt"
_CHANNEL = "avars:tz"
_TIMES = (1001.0, 1024.0, 1048.0)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecJoint"),
           "RigExecJoint schema is not registered")


def _Bakes(rig):
    return sum(1 for e in rig.profile_events() if e["name"] == "Compile.Bake")


def _Evaluate(rig, label):
    origins = []
    for time in _TIMES:
        pose = rig.evaluate(time)
        _Check(pose.valid, "%s: pose at %g is not valid" % (label, time))
        _Check(pose.baked_parity_mismatches == 0,
               "%s: baked and dynamic disagree at %g (%d mismatches)"
               % (label, time, pose.baked_parity_mismatches))
        origins.append(tuple(pose.control_frame(_LOOK).origin))
    return origins


def _RemoveSpec(stage):
    layer = stage.GetSessionLayer()
    spec = layer.GetPropertyAtPath(_LOOK + "." + _CHANNEL)
    _Check(spec, "no session spec to remove")
    layer.GetPrimAtPath(_LOOK).RemoveProperty(spec)


def main():
    _RegisterSchema()
    import _rigexec
    stage = Usd.Stage.Open(_EXAMPLE)
    _Check(stage, "example stage opens")
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    attribute = stage.GetPrimAtPath(_LOOK).GetAttribute(_CHANNEL)
    _Check(attribute, "%s has no %s" % (_LOOK, _CHANNEL))
    _Check(not attribute.ValueMightBeTimeVarying(),
           "%s must start constant for this test" % _CHANNEL)

    rig = _rigexec.Rig(stage, _RIG)
    rig.compile()
    _Check(rig.is_bakeable(), "example is not bakeable: %s"
           % rig.bakeability_reasons())
    rig.evaluation_mode = "parity"
    rig.profiling_enabled = True
    rest = _Evaluate(rig, "start")
    baseline = _Bakes(rig)

    def step(label, edit, expectMoved):
        edit()
        origins = _Evaluate(rig, label)
        _Check(_Bakes(rig) == baseline,
               "%s rebaked the program" % label)
        moved = origins != rest
        _Check(moved == expectMoved, "%s: control %s" % (
            label, "did not move" if expectMoved else "did not return"))
        print("  ok: %s" % label)

    def key():
        spline = Ts.Spline("double")
        for time, value in ((1001.0, 2.0), (1048.0, -3.0)):
            knot = Ts.Knot()
            knot.SetTime(time)
            knot.SetValue(value)
            spline.SetKnot(knot)
        attribute.SetSpline(spline)

    def rekey():
        spline = attribute.GetSpline()
        knot = Ts.Knot()
        knot.SetTime(1024.0)
        knot.SetValue(5.0)
        spline.SetKnot(knot)
        attribute.SetSpline(spline)

    step("first value creates the spec", lambda: attribute.Set(1.5), True)
    step("second value", lambda: attribute.Set(-0.5), True)
    step("undo removes the spec", lambda: _RemoveSpec(stage), False)
    step("first key animates the avar", key, True)
    step("another knot", rekey, True)
    step("undo of the key", lambda: _RemoveSpec(stage), False)

    # A connection is structure: it has to rebake, and still agree.
    attribute.SetConnections(
        [Sdf.Path(_LOOK).AppendProperty("avars:rx")])
    _Evaluate(rig, "connection")
    _Check(_Bakes(rig) > baseline, "a connection did not rebake")
    print("  ok: a connection still rebakes")
    print("RIGEXEC_BAKED_AVAR_PATCH_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
