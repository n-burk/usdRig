"""RigExecFkChain rigExec:controlSpace end to end from Python.

The same four-joint chain is built two ways. `world` (the default): sibling
controls with asset-space rests, and the solver composes the chain,
W_i = W_(i-1) . A_i. `parentRelative`: each control nested under the one
before it with a parent-relative rest, so its posed frame travels with its
parent (Maya FK style) and the solver takes each control's asset-space delta
as-is. The contract this asserts:

  1. `world` with the attribute unauthored and `world` authored explicitly
     are bit-identical: the default path is untouched.
  2. In `parentRelative`, posing the root moves the CHILD CONTROL frames,
     and the JOINT frames match `world` for the same avars.
  3. In `parentRelative`, a child control's own avars apply exactly once.

Usage:
    python test_rigexec_fk_control_space.py [schema_resources_dir]
"""

import pathlib
import sys

# The environment bootstrap (USD site-packages, build-tree python directory,
# DLL directories) is shared with the main binding test.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402


def _translate(x, y, z):
    """Row-vector identity rotation with translation in the last row."""
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1]


def _close(a, b, eps=1e-9):
    return abs(a - b) < eps


def _close_vec(a, b, eps=1e-9):
    return len(a) == len(b) and all(_close(p, q, eps) for p, q in zip(a, b))


def _origin(m):
    return tuple(m[12:15])


# Joints at (0,0,0), (10,0,0), (20,0,0), (25,2,1) with one control per joint.
JOINT_LOCALS = [(0, 0, 0), (10, 0, 0), (10, 0, 0), (5, 2, 1)]
CONTROL_WORLDS = [(0, 0, 0), (10, 0, 0), (20, 0, 0), (25, 2, 1)]

# Avar sets: control index -> {avar: value}.
SCENARIOS = {
    "rest": {},
    "root_rz90": {0: {"rz": 90.0}},
    "child_rz90": {1: {"rz": 90.0}},
    "stacked": {0: {"rz": 90.0}, 1: {"rz": 45.0, "ty": 3.0},
                2: {"rx": 30.0, "tx": 2.0}, 3: {"ry": -20.0, "sx": 1.5}},
    "general": {0: {"rx": 10.0, "ry": 20.0, "rz": 30.0,
                    "tx": 1.0, "ty": 2.0, "tz": 3.0},
                2: {"rx": -70.0, "sy": 0.5}},
}


def _build(mode, avars):
    """Author, pose, compile and evaluate one chain.

    mode: None leaves rigExec:controlSpace unauthored; 'world' authors the
    default explicitly; 'parentRelative' nests the controls and authors it.
    """
    from pxr import Sdf, Usd
    import rigexec

    stage = Usd.Stage.CreateInMemory()
    builder = rigexec.Builder.create(stage, "/Rig")

    joints = []
    for i, t in enumerate(JOINT_LOCALS):
        joints.append(builder.add_joint(
            "J%d" % i, _translate(*t), joints[-1] if joints else None))

    controls = []
    for i in range(len(joints)):
        if mode == "parentRelative" and i > 0:
            # Nested under the previous control with a rest relative to it,
            # exactly like the joints.
            controls.append(builder.add_control(
                "C%d" % i, _translate(*JOINT_LOCALS[i]), controls[-1]))
        else:
            controls.append(builder.add_control(
                "C%d" % i, _translate(*CONTROL_WORLDS[i])))

    fk = builder.add_fk_chain("Fk", controls, joints)
    if mode is not None:
        fk.set_control_space(mode)

    for i, per in avars.items():
        prim = stage.GetPrimAtPath(controls[i].path)
        for name, value in per.items():
            a = prim.GetAttribute("avars:" + name)
            if not a or not a.IsValid():
                a = prim.CreateAttribute(
                    "avars:" + name, Sdf.ValueTypeNames.Double)
            a.Set(value)

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    pose = rig.evaluate(0)
    return {
        "stage": stage,
        "fk": fk,
        "control_paths": [c.path for c in controls],
        "joints": [pose.joint_frame(j.path).to_matrix4() for j in joints],
        "controls": [pose.control_frame(c.path).to_matrix4()
                     for c in controls],
        "solver": [f.to_matrix4() for f in pose.solver_frames(fk.path)],
    }


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    _setup_environment()

    import rigexec

    rigexec.load_schema_plugin(plugin_dir)
    assert "RigExecFkChain" in rigexec.schema.names()

    # -- Authoring: the attribute, its fallback, and the nested layout -----
    absent = _build(None, {})
    prim = absent["stage"].GetPrimAtPath(absent["fk"].path)
    assert prim.GetTypeName() == "RigExecFkChain", prim.GetTypeName()
    attr = prim.GetAttribute("rigExec:controlSpace")
    assert attr and attr.Get() == "world", attr
    assert not attr.IsAuthored()
    assert absent["control_paths"] == [
        "/Rig/Controls/C%d" % i for i in range(4)], absent["control_paths"]

    explicit = _build("world", {})
    attr = explicit["stage"].GetPrimAtPath(
        explicit["fk"].path).GetAttribute("rigExec:controlSpace")
    assert attr.IsAuthored() and attr.Get() == "world", attr.Get()

    nested = _build("parentRelative", {})
    attr = nested["stage"].GetPrimAtPath(
        nested["fk"].path).GetAttribute("rigExec:controlSpace")
    assert attr.Get() == "parentRelative", attr.Get()
    assert nested["control_paths"] == [
        "/Rig/Controls/C0", "/Rig/Controls/C0/C1", "/Rig/Controls/C0/C1/C2",
        "/Rig/Controls/C0/C1/C2/C3"], nested["control_paths"]
    # The nested controls' composed rests are the same asset-space frames.
    for i, m in enumerate(nested["controls"]):
        assert _close_vec(_origin(m), CONTROL_WORLDS[i]), (i, _origin(m))

    # Only the two documented tokens are accepted.
    try:
        nested["fk"].set_control_space("local")
    except ValueError as e:
        assert "parentRelative" in str(e), e
    else:
        raise AssertionError("set_control_space accepted an unknown token")

    # -- Every scenario, all three builds --------------------------------
    for name, avars in SCENARIOS.items():
        d = _build(None, avars)
        w = _build("world", avars)
        p = _build("parentRelative", avars)

        # 1. The default path is untouched: unauthored == world, bit for bit.
        assert d["joints"] == w["joints"], (name, d["joints"], w["joints"])
        assert d["solver"] == w["solver"], name
        assert d["controls"] == w["controls"], name

        # 2. Same deformation in both spaces, every joint and solver
        # element, all sixteen entries.
        for i, (a, b) in enumerate(zip(w["joints"], p["joints"])):
            assert _close_vec(a, b), (name, i, a, b)
        for i, (a, b) in enumerate(zip(w["solver"], p["solver"])):
            assert _close_vec(a, b), (name, i, a, b)
        print("  %-10s joints %s" % (
            name, [tuple(round(v, 3) for v in _origin(m))
                   for m in p["joints"]]))

    # -- 1. Today's numbers for the default path ---------------------------
    w = _build(None, SCENARIOS["root_rz90"])
    expect = [(0, 0, 0), (0, 10, 0), (0, 20, 0), (-2, 25, 1)]
    for m, e in zip(w["joints"], expect):
        assert _close_vec(_origin(m), e), (_origin(m), e)
    # ...and the flat controls stay at rest: the gizmos get left behind.
    for m, e in zip(w["controls"], CONTROL_WORLDS):
        assert _close_vec(_origin(m), e), (_origin(m), e)

    # -- 2. parentRelative: the root's 90 about Z carries the child controls
    p = _build("parentRelative", SCENARIOS["root_rz90"])
    for i, (m, e) in enumerate(zip(p["controls"], expect)):
        assert _close_vec(_origin(m), e), (i, _origin(m), e)
        if i > 0:
            assert not _close_vec(_origin(m), CONTROL_WORLDS[i], 1e-3), \
                "child control %d did not move" % i
    for m, e in zip(p["joints"], expect):
        assert _close_vec(_origin(m), e), (_origin(m), e)

    # -- 3. A child's own avar applies once: rz 90 on C1 (at (10,0,0))
    # swings J2 to (10,10,0) and J3 to (8,15,1), and C2/C3 travel with C1.
    # Applied twice (180 degrees) J2 would be back at the origin.
    expect = [(0, 0, 0), (10, 0, 0), (10, 10, 0), (8, 15, 1)]
    p = _build("parentRelative", SCENARIOS["child_rz90"])
    for m, e in zip(p["joints"], expect):
        assert _close_vec(_origin(m), e), (_origin(m), e)
    for m, e in zip(p["controls"], expect):
        assert _close_vec(_origin(m), e), (_origin(m), e)
    w = _build("world", SCENARIOS["child_rz90"])
    for m, e in zip(w["joints"], expect):
        assert _close_vec(_origin(m), e), (_origin(m), e)

    print("OK: FK chain controlSpace end to end from Python")


if __name__ == "__main__":
    main()
