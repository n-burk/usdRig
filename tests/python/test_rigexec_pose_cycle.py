"""A follower of solved joints reads the version at its own stack place.

The shape under test is the conventional per-limb param node: a control under
<rig>/Controls, parent-constrained from the limb's end joint, whose scalar
`avars:ikfk` is what the limb's blend solver reads through
`inputs:weight.connect`. Authored on the biped, the follower-at-the-bottom
arrangement was once rejected with a "pose dependency cycle among:"
naming sixty constraints and four solvers -- the blend reading the param
control, the control written by a constraint that reads the ankle, the
ankle written by the blend.

That rejection belonged to the two-phase compiler. Under the unified
pose stack (spec 4.2) a FRAME read is positional: a step reads the
version standing at its own place in the stack, so a reader below a
writer sees the earlier version and that is not a contradiction. The
follower below the solvers reads the pre-solve ankle; the same follower
after the solvers reads the final ankle. Neither arrangement cycles,
and neither does the shape that used to be the genuine loop -- the IK's
effector constrained from the IK's own joint -- because every one of
its reads is positional too.

So this asserts three things on a minimal leg with a reverse foot:

  1. the follower at the BOTTOM of the Movers stack compiles, and the
     param control tracks the REST ankle -- the version standing where
     the follower runs, before the solvers -- while the solved ankle
     demonstrably moves on without it;
  2. the same rig with the Solvers scope reordered last -- solvers
     first, then the constraints -- compiles, the blend still reads the
     scalar, and the param control tracks the solved ankle in FK and
     in IK;
  3. the former genuine loop compiles and evaluates valid in either
     Movers order.

Usage:
    python test_rigexec_pose_cycle.py [schema_resources_dir]
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

RIG = "/Rig"
BLEND = RIG + "/Solvers/IkFk"
PARAMS = RIG + "/Controls/Params"
ANKLE = RIG + "/Joints/Thigh/Knee/Ankle"
# Ankle rest from _build's chain: thigh at 0, knee -3 below it, ankle -3
# below that.
ANKLE_REST = (0.0, -6.0, 0.0)


def _at(y):
    """Row-vector identity rotation with a translation down Y."""
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, y, 0, 1]


def _origin(frame):
    m = frame.to_matrix4()
    return (m[12], m[13], m[14])


def _distance(a, b):
    return sum((p - q) ** 2 for p, q in zip(a, b)) ** 0.5


def _build(rigexec, Usd, Sdf, follower_on_top, effector_constrained):
    """A leg with a reverse foot, a blend on a param control's scalar,
    and a follower of the ankle. Returns (stage, rig).

    Chains are created in the order tail, foot, param_follow, which is the
    order the biped's builder produces them, so param_follow is the BOTTOM
    of the stack -- first to execute -- unless `follower_on_top` moves it.
    """
    stage = Usd.Stage.CreateInMemory()
    builder = rigexec.Builder.create(stage, RIG)

    thigh = builder.add_joint("Thigh", _at(0.0))
    knee = builder.add_joint("Knee", _at(-3.0), parent_joint=thigh)
    ankle = builder.add_joint("Ankle", _at(-3.0), parent_joint=knee)
    joints = [thigh, knee, ankle]

    leg_root = builder.add_control("LegRoot")
    leg_ik = builder.add_control("LegIk")
    pole = builder.add_control("LegPv")
    # Inside reach (3 + 3 against 5) so the chain bends toward the pole.
    leg_ik.set_avar_translation(0.0, -5.0, 0.0)
    pole.set_avar_translation(0.0, -2.5, 3.0)

    # The reverse foot, as the biped nests it: a pivot JOINT under the IK
    # control, rotation-constrained from a bank control, and the effector
    # target under that pivot. The IK therefore legitimately waits on the
    # foot constraint.
    pivot = stage.DefinePrim(Sdf.Path(leg_ik.path).AppendChild("Pivot"),
                             "RigExecJoint")
    # A joint under a control keeps the control's purpose, as the biped's
    # pivots do; otherwise the compiler warns that the extents disagree.
    pivot.CreateAttribute("purpose", Sdf.ValueTypeNames.Token).Set("default")
    foot = stage.DefinePrim(pivot.GetPath().AppendChild("Foot"),
                            "RigExecControl")
    bank = builder.add_control("Bank")

    fk_controls = [builder.add_control("Fk" + j.name) for j in joints]
    fk = builder.add_fk_chain("Fk", controls=fk_controls, joints=joints)

    effector = str(foot.GetPath())
    if effector_constrained:
        # The genuine loop: the effector control is itself driven from the
        # ankle the IK poses.
        effector = builder.add_control("Foot2")
    ik = builder.add_two_bone_ik("Ik", leg_root, effector, pole)
    ik.set_joints(joints)
    blend = builder.add_blend_point_frames("IkFk", fk, ik, 1.0)
    blend.set_joints(joints)

    # Downstream of the loop but not on it: a constraint that reads the
    # ankle and feeds nothing. Created first, so it sits at the top.
    tail = builder.add_control("Tail")
    builder.new_mover_chain("tail").add_position_constraint(
        "tail_from_ankle", tail, [ankle])

    foot_chain = builder.new_mover_chain("foot")
    foot_chain.add_rotation_constraint("pivot_from_bank",
                                       str(pivot.GetPath()), [bank])
    if effector_constrained:
        foot_chain.add_parent_constraint("foot_from_ankle", effector,
                                         [ankle])

    # the conventional param node: a control under Controls, parent-constrained from
    # the end joint, carrying the dial the blend reads as a SCALAR.
    params = builder.add_control("Params")
    dial = stage.GetPrimAtPath(params.path).CreateAttribute(
        "avars:ikfk", Sdf.ValueTypeNames.Float)
    dial.Set(1.0)
    stage.GetPrimAtPath(blend.path).GetAttribute(
        "inputs:weight").SetConnections(
            [Sdf.Path(params.path).AppendProperty("avars:ikfk")])
    follow = builder.new_mover_chain("param_follow")
    follow.add_parent_constraint("params_to_ankle", params, [ankle])

    if follower_on_top:
        movers = stage.GetPrimAtPath(RIG + "/Movers")
        names = [c.GetName() for c in movers.GetChildren()]
        names.remove("param_follow")
        movers.SetChildrenReorder(["param_follow"] + names)

    return stage, rigexec.Rig(stage, RIG)


def _compile_error(rig):
    try:
        rig.compile()
    except ValueError as error:
        return str(error)
    return None


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    _setup_environment()

    from pxr import Sdf, Usd
    import rigexec

    rigexec.load_schema_plugin(plugin_dir)

    # --- 1. follower below the solvers: compiles, reads the rest --------
    stage, rig = _build(rigexec, Usd, Sdf, follower_on_top=False,
                        effector_constrained=False)
    message = _compile_error(rig)
    assert message is None, (
        "a frame read from below its writer is positional, not a cycle: %s"
        % message)
    pose = rig.evaluate(1.0)
    assert pose.valid
    params = _origin(pose.control_frame(PARAMS))
    ankle = _origin(pose.joint_frame(ANKLE))
    assert _distance(params, ANKLE_REST) < 1e-6, (
        "the follower runs before the solvers, so it tracks the rest "
        "ankle: %s" % (params,))
    gap = _distance(params, ankle)
    assert gap > 0.5, (
        "the solved ankle moved on without it: gap %.6f" % gap)
    print("bottom of the stack: compiles; params tracks rest %s, gap to "
          "solved %.4f" % (params, gap))

    # --- 2. solvers first: the follower tracks the solved ankle ---------
    stage, rig = _build(rigexec, Usd, Sdf, follower_on_top=True,
                        effector_constrained=False)
    stage.GetPrimAtPath(RIG).SetChildrenReorder(
        ["Joints", "Controls", "Movers", "Solvers"])
    message = _compile_error(rig)
    assert message is None, (
        "the solvers-first ordering should compile: %s" % message)
    dial = stage.GetPrimAtPath(PARAMS).GetAttribute("avars:ikfk")
    weight = stage.GetPrimAtPath(BLEND).GetAttribute("inputs:weight")
    assert weight.GetConnections() == [dial.GetPath()], (
        "the blend reads the constrained control's scalar")
    gaps = {}
    ankles = {}
    for value in (0.0, 1.0):
        dial.Set(value)
        pose = rig.evaluate(1.0)
        ankles[value] = _origin(pose.joint_frame(ANKLE))
        gaps[value] = _distance(ankles[value],
                                _origin(pose.control_frame(PARAMS)))
        assert gaps[value] < 1e-6, (
            "ikfk = %g: the param control sits %.6f from the ankle"
            % (value, gaps[value]))
    moved = _distance(ankles[0.0], ankles[1.0])
    assert moved > 0.5, (
        "the blend is live: the ankle moved %.6f between FK and IK" % moved)
    print("top of the stack: compiles; gap FK %.6f, IK %.6f; the ankle "
          "moved %.4f between the two" % (gaps[0.0], gaps[1.0], moved))

    # --- 3. the former genuine loop: ordered reads, valid pose ---------
    for on_top in (False, True):
        stage, rig = _build(rigexec, Usd, Sdf, follower_on_top=on_top,
                            effector_constrained=True)
        message = _compile_error(rig)
        assert message is None, (
            "positional reads resolve the old loop (follower on top: %s): "
            "%s" % (on_top, message))
        pose = rig.evaluate(1.0)
        assert pose.valid, "follower on top: %s" % on_top
    print("former loop: compiles and evaluates valid in both orders")
    print("OK")


if __name__ == "__main__":
    main()
