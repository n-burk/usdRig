#
# RigExecFloatMathMover "curve": a float through piecewise-linear keys.
#
# A channel is animated across and past its keys. The mover must return
# the keys' linear interpolation inside them and extend the first and last
# segments outside them, blend by its envelope, agree between the dynamic
# walk and the baked program (parity mode), and refuse unsorted keys or a
# curve on a vec3f mover at compile.
#
import math
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd, Vt  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


# Upper-lid blink keys: lid angle -> cluster rotation.
KEYS = [(-20.0, -15.0), (0.0, 0.0), (20.0, 35.0), (40.0, 65.0)]
FRAMES = {1.0: -40.0, 2.0: -20.0, 3.0: -5.0, 4.0: 0.0, 5.0: 10.0,
          6.0: 30.0, 7.0: 40.0, 8.0: 60.0}


def _Reference(x):
    if x <= KEYS[1][0]:
        a, b = KEYS[0], KEYS[1]
    elif x >= KEYS[-2][0]:
        a, b = KEYS[-2], KEYS[-1]
    else:
        i = next(i for i in range(1, len(KEYS)) if KEYS[i][0] >= x)
        a, b = KEYS[i - 1], KEYS[i]
    return a[1] + (x - a[0]) * (b[1] - a[1]) / (b[0] - a[0])


def _Stage(keys=KEYS, weight=1.0):
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Joints", "Scope")
    joint = stage.DefinePrim("/Asset/Rig/Joints/Root", "RigExecJoint")
    joint.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    channels = stage.DefinePrim("/Asset/Rig/Channels", "Scope")
    dial = channels.CreateAttribute("rigExec:dial", Sdf.ValueTypeNames.Float)
    dial.Set(0.0)
    for frame, value in FRAMES.items():
        dial.Set(value, frame)
    mover = stage.DefinePrim("/Asset/Rig/Movers/blink", "RigExecFloatMathMover")
    mover.AddAppliedSchema("RigExecMoverAPI")
    mover.GetRelationship("rigExec:moves").SetTargets([dial.GetPath()])
    mover.GetAttribute("rigExec:operation").Set("curve")
    mover.GetAttribute("inputs:keys").Set(
        Vt.Vec2fArray([Gf.Vec2f(*k) for k in keys]))
    mover.CreateAttribute("inputs:defaultWeight",
                          Sdf.ValueTypeNames.Float).Set(weight)
    return stage, dial.GetPath()


def TestDrivenAvar():
    """A curve mover writing a hidden control's avar poses the control.

    The lid's ty drives a pivot's rx through keys, the way a driven key
    turns a control channel into a rotation. Checked on both paths, and the
    pivot's frame must actually turn, not only the attribute.
    """
    import _rigexec
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Joints", "Scope")
    joint = stage.DefinePrim("/Asset/Rig/Joints/Root", "RigExecJoint")
    joint.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    stage.DefinePrim("/Asset/Rig/Controls", "Scope")
    lid = stage.DefinePrim("/Asset/Rig/Controls/lid", "RigExecControl")
    lid.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    ty = lid.GetAttribute("avars:ty")
    for frame, value in ((1.0, 0.0), (2.0, 5.0), (3.0, -10.0)):
        ty.Set(value, frame)
    pivot = stage.DefinePrim("/Asset/Rig/Controls/pivot", "RigExecControl")
    pivot.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    mover = stage.DefinePrim("/Asset/Rig/Movers/lid_to_rx",
                             "RigExecFloatMathMover")
    mover.AddAppliedSchema("RigExecMoverAPI")
    mover.GetRelationship("rigExec:moves").SetTargets(
        [pivot.GetAttribute("avars:rx").GetPath()])
    # rx starts at 0 and the first step adds ty to it.
    add = stage.DefinePrim("/Asset/Rig/Movers/lid_to_rx_0_add",
                           "RigExecFloatMathMover")
    add.AddAppliedSchema("RigExecMoverAPI")
    add.GetRelationship("rigExec:moves").SetTargets(
        [pivot.GetAttribute("avars:rx").GetPath()])
    add.GetAttribute("rigExec:operation").Set("add")
    add.GetAttribute("inputs:value").SetConnections([ty.GetPath()])
    mover.GetAttribute("rigExec:operation").Set("curve")
    mover.GetAttribute("inputs:keys").Set(Vt.Vec2fArray(
        [Gf.Vec2f(-10, 40), Gf.Vec2f(0, 0), Gf.Vec2f(10, -40)]))
    # Movers run bottom to top: the add first, then the curve.
    stage.GetPrimAtPath("/Asset/Rig/Movers").SetChildrenReorder(
        ["lid_to_rx", "lid_to_rx_0_add"])
    for mode in ("reference", "parity"):
        rig = _rigexec.Rig(stage, "/Asset/Rig")
        rig.compile()
        rig.evaluation_mode = mode
        for frame, want in ((1.0, 0.0), (2.0, -20.0), (3.0, 40.0)):
            pose = rig.evaluate(frame)
            _Check(pose.baked_parity_mismatches == 0,
                   "%s: baked and dynamic disagree at %g" % (mode, frame))
            got = pose.moved_property("/Asset/Rig/Controls/pivot.avars:rx")
            _Check(abs(got - want) < 1e-4,
                   "%s: rx %g, expected %g" % (mode, got, want))
            m = Gf.Matrix4d(*pose.control_frame(
                "/Asset/Rig/Controls/pivot").to_matrix4())
            y = m.TransformDir(Gf.Vec3d(0, 1, 0))
            angle = math.degrees(math.atan2(y[2], y[1]))
            _Check(abs(angle - want) < 1e-3,
                   "%s: the pivot turned %g degrees, expected %g"
                   % (mode, angle, want))
    print("  ok: a curve mover drives a control avar")


def _HermiteReference(keys, tangents, x):
    if x <= keys[0][0]:
        return keys[0][1] + (x - keys[0][0]) * tangents[0][0]
    if x >= keys[-1][0]:
        return keys[-1][1] + (x - keys[-1][0]) * tangents[-1][1]
    i = next(i for i in range(1, len(keys)) if keys[i][0] >= x)
    (xa, ya), (xb, yb) = keys[i - 1], keys[i]
    h = xb - xa
    t = (x - xa) / h
    return ((2 * t ** 3 - 3 * t ** 2 + 1) * ya +
            (t ** 3 - 2 * t ** 2 + t) * h * tangents[i - 1][1] +
            (-2 * t ** 3 + 3 * t ** 2) * yb +
            (t ** 3 - t ** 2) * h * tangents[i][0])


def TestHermite():
    """Keys with tangents evaluate as a cubic Hermite, like a driven key.

    The source rig's upper blink turn: a spline key in the middle, so the
    curve genuinely bends between keys, and linear infinity past both ends.
    """
    import _rigexec
    keys = [(-20.0, -15.0), (0.0, 0.0), (20.0, 35.0), (40.0, 65.0)]
    tangents = [(0.75, 0.75), (1.25, 1.25), (1.75, 1.5), (1.5, 1.5)]
    stage, dialPath = _Stage(keys=keys)
    stage.GetPrimAtPath("/Asset/Rig/Movers/blink").GetAttribute(
        "inputs:tangents").Set(Vt.Vec2fArray(
            [Gf.Vec2f(*t) for t in tangents]))
    for mode in ("reference", "parity"):
        rig = _rigexec.Rig(stage, "/Asset/Rig")
        rig.compile()
        rig.evaluation_mode = mode
        for frame, x in sorted(FRAMES.items()):
            pose = rig.evaluate(frame)
            _Check(pose.baked_parity_mismatches == 0,
                   "%s: parity at %g" % (mode, frame))
            got = pose.moved_property(str(dialPath))
            want = _HermiteReference(keys, tangents, x)
            _Check(abs(got - want) < 1e-3,
                   "%s: hermite(%g) = %g, expected %g" % (mode, x, got, want))
    # And it is not the linear answer between keys.
    _Check(abs(_HermiteReference(keys, tangents, 10.0) - 17.5) > 0.1,
           "the reference bends")
    print("  ok: hermite tangents")


def main():
    _RegisterSchema()
    import _rigexec

    _Check(abs(_Reference(-40.0) + 30.0) < 1e-6, "reference extrapolates")

    stage, dialPath = _Stage()
    for mode in ("reference", "parity"):
        rig = _rigexec.Rig(stage, "/Asset/Rig")
        rig.compile()
        rig.evaluation_mode = mode
        for frame, x in sorted(FRAMES.items()):
            pose = rig.evaluate(frame)
            _Check(pose.baked_parity_mismatches == 0,
                   "%s: baked and dynamic disagree at %g" % (mode, frame))
            got = pose.moved_property(str(dialPath))
            want = _Reference(x)
            _Check(abs(got - want) < 1e-4,
                   "%s: curve(%g) = %g, expected %g" % (mode, x, got, want))
        print("  ok: %s curve" % mode)

    # The envelope mixes back toward the incoming value.
    stage, dialPath = _Stage(weight=0.5)
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    rig.compile()
    got = rig.evaluate(6.0).moved_property(str(dialPath))
    want = 30.0 + 0.5 * (_Reference(30.0) - 30.0)
    _Check(abs(got - want) < 1e-4, "half weight: %g, expected %g" % (got, want))

    # A single key is a constant.
    stage, dialPath = _Stage(keys=[(0.0, 7.0)])
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    rig.compile()
    got = rig.evaluate(8.0).moved_property(str(dialPath))
    _Check(abs(got - 7.0) < 1e-6, "one key is a constant: %g" % got)
    print("  ok: envelope and constant")

    # Unsorted keys are refused at compile.
    stage, _ = _Stage(keys=[(10.0, 0.0), (0.0, 1.0)])
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    refused = False
    try:
        refused = not rig.compile()
    except Exception:
        refused = True
    _Check(refused, "unsorted keys must not compile")

    # A curve on a vec3f mover is refused at compile.
    stage, _ = _Stage()
    vec = stage.GetPrimAtPath("/Asset/Rig/Channels").CreateAttribute(
        "rigExec:offset", Sdf.ValueTypeNames.Float3)
    vec.Set(Gf.Vec3f(0, 0, 0))
    bad = stage.DefinePrim("/Asset/Rig/Movers/vec", "RigExecVec3fMathMover")
    bad.AddAppliedSchema("RigExecMoverAPI")
    bad.GetRelationship("rigExec:moves").SetTargets([vec.GetPath()])
    bad.GetAttribute("rigExec:operation").Set("curve")
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    refused = False
    try:
        refused = not rig.compile()
    except Exception:
        refused = True
    _Check(refused, "curve on a vec3f mover must not compile")
    print("  ok: compile refusals")

    TestDrivenAvar()
    TestHermite()
    print("RIGEXEC_FLOAT_CURVE_OP_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
