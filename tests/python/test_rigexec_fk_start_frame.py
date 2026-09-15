"""RigExecFkChain rigExec:startFrame end to end from Python.

The bug this relationship exists for, reproduced exactly as it was first
measured. A four-joint chain `A -> B -> C -> D`; an FkChain drives `[B, C,
D]` only, so `A` is the thing the chain HANGS FROM. Posing `A` by
`avars:rz = 90` used to leave B at (10,0,0) and C at (20,0,0) -- the
solver writes ABSOLUTE world frames and nothing told it where its chain
starts. The finger chains of a hand built that way would detach from the
wrist the moment the arm moved.

What this asserts:

  1. unauthored is the historical behaviour, to the digit: posing A moves
     B/C/D by exactly nothing, and the chain's own avars still solve the
     way they always did;
  2. authoring the relationship while the start provider is AT REST is
     bit-identical to leaving it unauthored -- the synthetic base element
     is an exact no-op at S = identity, in both control spaces;
  3. authored and posed, the whole chain rides the provider: every joint
     lands on S applied to where it was without the pose, for all sixteen
     matrix entries, and the chain's own avars still apply on top exactly
     once;
  4. cardinality is untouched -- the solver still publishes one frame per
     control, so no joint's element index moves;
  5. `world` and `parentRelative` give the same joints with a start frame,
     as they do without one.

Usage:
    python test_rigexec_fk_start_frame.py [schema_resources_dir]
"""

import math
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


def _mul(a, b):
    """4x4 row-major times 4x4 row-major, as flat 16-lists."""
    out = [0.0] * 16
    for r in range(4):
        for c in range(4):
            out[r * 4 + c] = sum(a[r * 4 + k] * b[k * 4 + c] for k in range(4))
    return out


def _rz(deg):
    """Row-vector rotation about Z, the same convention the rig uses."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


# A at the origin, then B, C, D down +X. The FK chain drives B, C and D;
# A is only ever the start frame.
JOINT_LOCALS = [(0, 0, 0), (10, 0, 0), (10, 0, 0), (5, 2, 1)]
JOINT_WORLDS = [(0, 0, 0), (10, 0, 0), (20, 0, 0), (25, 2, 1)]
CHAIN = (1, 2, 3)  # indices of B, C, D

# Avar sets for the chain's own controls: control index (within the chain)
# -> {avar: value}.
SCENARIOS = {
    "rest": {},
    "chain_root_rz90": {0: {"rz": 90.0}},
    "chain_mid_rz45": {1: {"rz": 45.0}},
    "stacked": {0: {"rz": 30.0, "ty": 2.0}, 1: {"rx": 20.0},
                2: {"ry": -15.0, "sx": 1.5, "tz": 1.0}},
}

# What the start provider is posed with (its own avars, on joint A).
START_POSES = {
    "start_rest": {},
    "start_rz90": {"rz": 90.0},
    "start_general": {"rx": 12.0, "ry": -25.0, "rz": 40.0,
                      "tx": 3.0, "ty": -1.0, "tz": 2.0},
}


def _build(mode, avars, start_avars, with_start):
    """Author, pose, compile and evaluate one chain.

    mode: None leaves rigExec:controlSpace unauthored; 'parentRelative'
    nests the chain's controls and authors it.
    with_start: author rigExec:startFrame at joint A.
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
    for n, j in enumerate(CHAIN):
        if mode == "parentRelative" and n > 0:
            controls.append(builder.add_control(
                "C%d" % j, _translate(*JOINT_LOCALS[j]), controls[-1]))
        else:
            controls.append(builder.add_control(
                "C%d" % j, _translate(*JOINT_WORLDS[j])))

    fk = builder.add_fk_chain("Fk", controls, [joints[j] for j in CHAIN])
    if mode is not None:
        fk.set_control_space(mode)
    if with_start:
        fk.set_start_frame(joints[0].path)

    def _set(path, per):
        prim = stage.GetPrimAtPath(path)
        for name, value in per.items():
            a = prim.GetAttribute("avars:" + name)
            if not a or not a.IsValid():
                a = prim.CreateAttribute(
                    "avars:" + name, Sdf.ValueTypeNames.Double)
            a.Set(value)

    for i, per in avars.items():
        _set(controls[i].path, per)
    _set(joints[0].path, start_avars)

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    pose = rig.evaluate(0)
    return {
        "stage": stage,
        "fk": fk,
        "start": pose.joint_frame(joints[0].path).to_matrix4(),
        "joints": [pose.joint_frame(joints[j].path).to_matrix4()
                   for j in CHAIN],
        "solver": [f.to_matrix4() for f in pose.solver_frames(fk.path)],
    }


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    _setup_environment()

    import rigexec

    rigexec.load_schema_plugin(plugin_dir)
    assert "RigExecFkChain" in rigexec.schema.names()

    # -- Authoring ---------------------------------------------------------
    absent = _build(None, {}, {}, False)
    prim = absent["stage"].GetPrimAtPath(absent["fk"].path)
    rel = prim.GetRelationship("rigExec:startFrame")
    assert rel, "RigExecFkChain has no rigExec:startFrame"
    assert not rel.GetTargets(), rel.GetTargets()

    wired = _build(None, {}, {}, True)
    rel = wired["stage"].GetPrimAtPath(
        wired["fk"].path).GetRelationship("rigExec:startFrame")
    assert [str(t) for t in rel.GetTargets()] == ["/Rig/Joints/J0"], \
        rel.GetTargets()

    # A start frame must be a frame provider, not any old prim.
    try:
        wired["fk"].set_start_frame(wired["fk"].path)
    except Exception as e:  # ValueError from the binding's argument check
        assert "RigExecJoint" in str(e), e
    else:
        raise AssertionError("set_start_frame accepted a non-provider")

    # -- 4. Cardinality is untouched: one frame per control, start excluded
    for d in (absent, wired):
        assert len(d["solver"]) == len(CHAIN), len(d["solver"])

    # -- 1. THE BUG, unauthored: posing the start joint moves nothing ------
    stuck = _build(None, {}, START_POSES["start_rz90"], False)
    for m, e in zip(stuck["joints"], [JOINT_WORLDS[j] for j in CHAIN]):
        assert _close_vec(_origin(m), e), (_origin(m), e)
    # The start joint itself really did turn -- it is the CHAIN that
    # ignored it. J0 sits at the origin, so read the rotation, not the
    # origin: its +X row should now point along +Y.
    assert _close_vec(stuck["start"][0:3], (0, 1, 0), 1e-9), \
        stuck["start"][0:3]
    print("  unauthored, start joint rz=90: chain stays at %s  (the bug)"
          % [tuple(round(v, 3) for v in _origin(m))
             for m in stuck["joints"]])

    # -- 2. Authored but at rest == unauthored, bit for bit ----------------
    for mode in (None, "world", "parentRelative"):
        for name, avars in SCENARIOS.items():
            off = _build(mode, avars, {}, False)
            on = _build(mode, avars, {}, True)
            assert off["joints"] == on["joints"], (mode, name)
            assert off["solver"] == on["solver"], (mode, name)

    # -- 3. Authored and posed: the chain rides the provider ---------------
    # rz=90 on J0 (at the origin) is a plain rotation of the whole chain.
    ride = _build(None, {}, START_POSES["start_rz90"], True)
    expect = [(0, 10, 0), (0, 20, 0), (-2, 25, 1)]
    for m, e in zip(ride["joints"], expect):
        assert _close_vec(_origin(m), e), (_origin(m), e)
    print("  authored,   start joint rz=90: chain rides to %s"
          % [tuple(round(v, 3) for v in _origin(m)) for m in ride["joints"]])

    # The general statement, for every combination and all sixteen entries:
    # posing the start provider applies its rest->pose map S on top of
    # whatever the chain was already doing. J0's rest is the identity, so
    # S is just its posed frame.
    for mode in (None, "parentRelative"):
        for cname, avars in SCENARIOS.items():
            base = _build(mode, avars, {}, True)
            for sname, start in START_POSES.items():
                moved = _build(mode, avars, start, True)
                s = moved["start"]
                for i, (b, m) in enumerate(zip(base["joints"],
                                               moved["joints"])):
                    assert _close_vec(_mul(b, s), m, 1e-9), \
                        (mode, cname, sname, i, _mul(b, s), m)

    # A concrete number for the one that matters most: the chain's own
    # avars and the start pose compose, they do not fight. Chain root
    # rz=90 puts C at (10,10,0) and D at (8,15,1); a further rz=90 on the
    # start joint rotates that whole result to (-10,10,0) and (-15,8,1).
    both = _build(None, SCENARIOS["chain_root_rz90"],
                  START_POSES["start_rz90"], True)
    expect = [(0, 10, 0), (-10, 10, 0), (-15, 8, 1)]
    for m, e in zip(both["joints"], expect):
        assert _close_vec(_origin(m), e, 1e-9), (_origin(m), e)

    # -- 5. Both control spaces agree, with a start frame as without ------
    for cname, avars in SCENARIOS.items():
        for sname, start in START_POSES.items():
            w = _build("world", avars, start, True)
            p = _build("parentRelative", avars, start, True)
            for i, (a, b) in enumerate(zip(w["joints"], p["joints"])):
                assert _close_vec(a, b, 1e-9), (cname, sname, i, a, b)
            for i, (a, b) in enumerate(zip(w["solver"], p["solver"])):
                assert _close_vec(a, b, 1e-9), (cname, sname, i, a, b)

    print("OK: FK chain startFrame end to end from Python")


if __name__ == "__main__":
    main()
