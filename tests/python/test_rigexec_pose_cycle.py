"""A follower of solved joints is not a cycle; a solver reading its own
follower is -- and the compiler has to say WHICH.

The shape under test is Maya's per-limb param node: a control under
<rig>/Controls, parent-constrained from the limb's end joint, whose scalar
`avars:ikfk` is what the limb's blend solver reads through
`inputs:weight.connect`. Authored on the biped it was rejected with a
"pose dependency cycle among:" followed by sixty constraints and four
solvers, and that list was read as a prim-granular false cycle -- the
blend reading the param control, the control written by a constraint that
reads the ankle, the ankle written by the blend. Bisecting it showed the
scalar read was never an edge at all: retargeting the constraint at a
fresh control nothing reads, or disconnecting the weight, changed nothing.

What actually closed the loop is the Movers STACK. Its namespace executes
bottom-up (rigEvaluator.cpp, _GetMoverExecutionOrder), constraints keep
that authored order, and a chain the builder appends lands at the bottom
-- so the follower ran FIRST, before the reverse-foot constraints the leg
IK's effector sits under. Follower waits on the blend, the blend on the
IK, the IK on the foot constraint, and the foot constraint, by stack
order, on the follower. The arms never cycled because nothing constrained
sits above their effector. The same rig with the follower chain at the top
of the stack compiles and tracks to 0.000000 cm.

So this asserts three things on a minimal leg with a reverse foot:

  1. the follower at the BOTTOM of the stack is reported as a cycle, and
     the report names the loop -- follower, blend, IK, foot constraint --
     and NOT the unrelated step downstream of it;
  2. the follower at the TOP compiles, the blend still reads the scalar
     the constraint's target carries, and the param control tracks the
     ankle in FK and in IK;
  3. a genuine loop -- the IK's effector control itself constrained from
     the IK's own joint -- fails in either order, and the report names it.

Usage:
    python test_rigexec_pose_cycle.py [schema_resources_dir]
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

RIG = "/Rig"
FOLLOWER = RIG + "/Movers/param_follow/params_to_ankle"
FOOT_CONSTRAINT = RIG + "/Movers/foot/pivot_from_bank"
TAIL_CONSTRAINT = RIG + "/Movers/tail/tail_from_ankle"
EFFECTOR_CONSTRAINT = RIG + "/Movers/foot/foot_from_ankle"
BLEND = RIG + "/Solvers/IkFk"
IK = RIG + "/Solvers/Ik"
FK = RIG + "/Solvers/Fk"
PARAMS = RIG + "/Controls/Params"
ANKLE = RIG + "/Joints/Thigh/Knee/Ankle"


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

    # Maya's param node: a control under Controls, parent-constrained from
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


def _loop_part(message):
    """The named loop, without the parenthetical that counts the rest."""
    line = next(l for l in message.splitlines()
                if "pose dependency cycle" in l)
    return line.split(" (each step", 1)[0]


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    _setup_environment()

    from pxr import Sdf, Usd
    import rigexec

    rigexec.load_schema_plugin(plugin_dir)

    # --- 1. follower at the bottom of the stack: a cycle, named -----------
    stage, rig = _build(rigexec, Usd, Sdf, follower_on_top=False,
                        effector_constrained=False)
    message = _compile_error(rig)
    assert message and "pose dependency cycle" in message, message
    for member in (FOLLOWER, BLEND, IK, FOOT_CONSTRAINT):
        assert member in message, "%s missing from: %s" % (member, message)
    assert FK not in message, "the FK chain has no wait: %s" % message
    loop = _loop_part(message)
    assert " -> " in loop, "the report walks the loop: %s" % message
    assert TAIL_CONSTRAINT not in loop, (
        "the tail constraint waits on the loop but is not on it: %s"
        % message)
    assert "1 further pose step waits on the loop" in message, message
    print("bottom of the stack: %s" % loop.strip())

    # --- 2. follower at the top: compiles, reads the scalar, tracks -------
    stage, rig = _build(rigexec, Usd, Sdf, follower_on_top=True,
                        effector_constrained=False)
    message = _compile_error(rig)
    assert message is None, (
        "the same rig with the follower executing last: %s" % message)
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

    # --- 3. a genuine loop fails in either order, and is named ------------
    for on_top in (False, True):
        stage, rig = _build(rigexec, Usd, Sdf, follower_on_top=on_top,
                            effector_constrained=True)
        message = _compile_error(rig)
        assert message and "pose dependency cycle" in message, (
            "an effector constrained from its own IK's joint compiled "
            "(follower on top: %s)" % on_top)
        loop = _loop_part(message)
        for member in (EFFECTOR_CONSTRAINT, BLEND, IK):
            assert member in loop, "%s missing from: %s" % (member, message)
    print("genuine loop: rejected in both orders; %s" % loop.strip())
    print("OK")


if __name__ == "__main__":
    main()
