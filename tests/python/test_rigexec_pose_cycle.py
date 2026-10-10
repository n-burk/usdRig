"""A follower of solved joints is not a cycle; a solver reading its own
follower is -- and the compiler has to say WHICH.

The shape under test is the conventional per-limb param node: a control under
<rig>/Controls, parent-constrained from the limb's end joint, whose scalar
`avars:ikfk` is what the limb's blend solver reads through
`inputs:weight.connect`. The scalar read is not a pose edge: the constraint
writes the control's frame, not its avars.

Constraints are ordered only by the frames they read and write, never by
their place in the Movers stack alone. A chain the builder appends lands at
the bottom of the stack, so the follower sits below the reverse-foot
constraint the leg IK's effector sits under; the two share no frame, so
nothing orders them, and the follower runs after the blend it reads. The
follower waits on the blend, the blend on the IK, the IK on the foot
constraint -- a chain, not a loop.

So this asserts three things on a minimal leg with a reverse foot:

  1. the follower at the BOTTOM of the stack compiles with no pose cycle,
     and its pose matches the follower-on-top arrangement at ikfk 0 and 1,
     graph and scalar reference;
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
    of the stack -- first in stack order -- unless `follower_on_top` moves
    it.

    The SCOPES are ordered the way the shipped biped orders them (Joints,
    Movers, Controls, Solvers), and under the unified pose stack that is
    load-bearing rather than cosmetic: execution is the reverse composed
    pre-order of the WHOLE rig (spec §4.2), so a scope's place in
    nameChildren decides whether every mover runs before or after every
    solver. Left in creation order the builder emits Solvers before Movers,
    which executes every constraint ahead of every solver -- no position
    within the Movers stack could then put the follower after the blend,
    and the shape this test is about would not arise at all.
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

    # The shipped scope order (see the docstring). Authored after the
    # scopes exist, because the builder creates them on first use.
    root = stage.GetPrimAtPath(RIG)
    present = [c.GetName() for c in root.GetChildren()]
    shipped = [n for n in ("Joints", "Movers", "Controls", "Solvers")
               if n in present]
    root.SetChildrenReorder(
        shipped + [n for n in present if n not in shipped])

    return stage, rigexec.Rig(stage, RIG)


def _cycle_report(rig):
    """The compiler's verdict on a cycle, however it is delivered.

    A cycle is no one member's fault, so the compiler sets EVERY member
    aside and compiles the rest of the rig -- one bad loop must not cost a
    character its other four hundred operations. The report is then the
    reason those operations carry, which is the same string a whole-rig
    refusal used to raise. Returns (report, skipped_paths), or (None, {})
    when the rig compiled clean.
    """
    try:
        rig.compile()
    except ValueError as error:
        return str(error), {}
    skipped = rig.skipped_operations()
    reasons = {reason for reason in skipped.values()}
    if len(reasons) > 1:
        # More than one unrelated refusal: hand back all of them rather
        # than pick, so an assertion names what actually happened.
        return "\n".join(sorted(reasons)), skipped
    return (reasons.pop() if reasons else None), skipped


def _pose_values(pose):
    """Every joint frame (final, and base where published) and control
    frame of a pose, by path, as flat tuples."""
    values = {}
    for path in pose.joint_paths():
        values["final " + path] = tuple(
            pose.joint_frame(path, True).to_matrix4())
        try:
            values["base " + path] = tuple(
                pose.joint_frame(path, False).to_matrix4())
        except KeyError:
            pass
    for path in pose.control_paths():
        values["control " + path] = tuple(
            pose.control_frame(path).to_matrix4())
    return values


def _poses_by_dial(rig, stage, mode):
    """The rig's pose values at ikfk 0 and 1 in the given evaluation
    mode."""
    rig.cpu_reference = True
    dial = stage.GetPrimAtPath(PARAMS).GetAttribute("avars:ikfk")
    out = {}
    for value in (0.0, 1.0):
        dial.Set(value)
        pose = rig.evaluate(1.0)
        assert pose.valid, "%s, ikfk = %g: invalid pose" % (mode, value)
        assert rig.op_graph(), (
            "ikfk = %g: the compiled graph did not answer" % value)
        gap = _distance(_origin(pose.joint_frame(ANKLE)),
                        _origin(pose.control_frame(PARAMS)))
        assert gap < 1e-6, (
            "%s, ikfk = %g: the param control sits %.6f from the ankle"
            % (mode, value, gap))
        out[value] = _pose_values(pose)
    return out


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

    # --- 1. follower at the bottom of the stack: compiles, same pose -----
    bottom_stage, bottom = _build(rigexec, Usd, Sdf, follower_on_top=False,
                                  effector_constrained=False)
    message, skipped = _cycle_report(bottom)
    assert message is None, (
        "the follower at the bottom of the stack: %s" % message)
    assert not skipped, "operations set aside: %s" % sorted(skipped)
    top_stage, top = _build(rigexec, Usd, Sdf, follower_on_top=True,
                            effector_constrained=False)
    message, skipped = _cycle_report(top)
    assert message is None and not skipped, message
    pass # Compile/evaluate validates the single graph.
    for mode in ("graph",):
        below = _poses_by_dial(bottom, bottom_stage, mode)
        above = _poses_by_dial(top, top_stage, mode)
        for value in (0.0, 1.0):
            assert below[value].keys() == above[value].keys(), (
                "%s, ikfk = %g: different frames published" % (mode, value))
            for key, frame in below[value].items():
                worst = max(abs(p - q)
                            for p, q in zip(frame, above[value][key]))
                assert worst < 1e-6, (
                    "%s, ikfk = %g: %s differs by %g between the follower "
                    "at the bottom and at the top" % (mode, value, key,
                                                      worst))
    print("bottom of the stack: compiles; graph and scalar reference poses match "
          "the follower on top at ikfk 0 and 1")

    # --- 2. follower at the top: compiles, reads the scalar, tracks -------
    stage, rig = _build(rigexec, Usd, Sdf, follower_on_top=True,
                        effector_constrained=False)
    message, _skipped = _cycle_report(rig)
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

    # --- 3. a genuine loop is set aside in either order, and is named ------------
    follower = RIG + "/Movers/param_follow/params_to_ankle"
    for on_top in (False, True):
        stage, rig = _build(rigexec, Usd, Sdf, follower_on_top=on_top,
                            effector_constrained=True)
        message, skipped = _cycle_report(rig)
        assert message and "pose dependency cycle" in message, (
            "an effector constrained from its own IK's joint compiled "
            "(follower on top: %s)" % on_top)
        loop = _loop_part(message)
        assert " -> " in loop, "the report walks the loop: %s" % message
        for member in (EFFECTOR_CONSTRAINT, BLEND, IK):
            assert member in loop, "%s missing from: %s" % (member, message)
        assert TAIL_CONSTRAINT not in loop, (
            "the tail constraint waits on the loop but is not on it: %s"
            % message)
        assert FK not in message, "the FK chain has no wait: %s" % message
        # D3 keeps dependents outside the SCC. Their exact authored identities
        # remain in the common graph instead of a retired pose-level wait count.
        live_owners = {node["label"] for node in rig.op_graph()}
        for kept in (TAIL_CONSTRAINT, follower, FK):
            assert kept in live_owners, "live dependent missing: %s" % kept
        # The loop's members, exactly, are what was set aside: the tail
        # constraint and the follower wait on the loop but are not on it,
        # so they keep running.
        assert set(skipped) == {EFFECTOR_CONSTRAINT, IK, BLEND}, (
            "set aside: %s" % sorted(skipped))
        for kept in (TAIL_CONSTRAINT, follower, FK):
            assert kept not in skipped, (
                "%s was set aside: %s" % (kept, sorted(skipped)))
        authored = stage.GetRootLayer().ExportToString()
        pose = rig.evaluate(1.0)
        assert pose.valid, "one local SCC does not reject independent work"
        held = rig.evaluate(1.0)
        assert held.valid and rig.skipped_operations() == skipped
        assert stage.GetRootLayer().ExportToString() == authored, (
            "cycle reporting/evaluation must not author the stage")
    print("genuine loop: locally excluded in both orders; %s" % loop.strip())
    print("OK")


if __name__ == "__main__":
    main()
