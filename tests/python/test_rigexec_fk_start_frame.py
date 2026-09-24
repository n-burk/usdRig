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

The relationship can also be DERIVED instead of authored. With
`rigExec:startFramePolicy = "parent"` and no authored targets, the
compile walks the joint hierarchy -- nearest namespace ancestor of the
chain's joints that is a joint or control -- and hangs the chain from
it exactly as if the relationship named it:

  6. derived is bit-identical to authored, in every mode, scenario and
     start pose, in reference, baked and parity evaluation;
  7. authored targets win silently: policy plus the relationship is the
     relationship alone, never applied twice;
  8. a chain whose joints span providers, or with no provider ancestor,
     falls back to the absolute solve with a compile warning;
  9. the read is positional (spec section 4.2): a chain ordered before
     its provider's writer reads the pre-write frame and warns naming
     the reorder; reordered after the writer it reads fresh.

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


def _build(mode, avars, start_avars, with_start, policy=None,
           chain=CHAIN, evaluation_mode="reference", precompile=True):
    """Author, pose, compile and evaluate one chain.

    mode: None leaves rigExec:controlSpace unauthored; 'parentRelative'
    nests the chain's controls and authors it.
    with_start: author rigExec:startFrame at joint A.
    policy: author rigExec:startFramePolicy when not None.
    chain: the joint indices the chain drives (default B, C, D).
    precompile: False skips the explicit rig.compile(), so the first
    evaluate compiles and its diagnostics carry the compile warnings.
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
    for n, j in enumerate(chain):
        if mode == "parentRelative" and n > 0:
            controls.append(builder.add_control(
                "C%d" % j, _translate(*JOINT_LOCALS[j]), controls[-1]))
        else:
            controls.append(builder.add_control(
                "C%d" % j, _translate(*JOINT_WORLDS[j])))

    fk = builder.add_fk_chain("Fk", controls, [joints[j] for j in chain])
    if mode is not None:
        fk.set_control_space(mode)
    if with_start:
        fk.set_start_frame(joints[0].path)
    if policy is not None:
        assert hasattr(fk, "set_start_frame_policy"), \
            "this build's _rigexec is older than this test: Rig is " \
            "missing set_start_frame_policy. Rebuild and re-run."
        fk.set_start_frame_policy(policy)

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
    if precompile:
        rig.compile()
    rig.evaluation_mode = evaluation_mode
    pose = rig.evaluate(0)
    out = {
        "stage": stage,
        "fk": fk,
        "start": pose.joint_frame(joints[0].path).to_matrix4(),
        "joints": [pose.joint_frame(joints[j].path).to_matrix4()
                   for j in chain],
        "solver": [f.to_matrix4() for f in pose.solver_frames(fk.path)],
        "diagnostics": list(pose.diagnostics),
    }
    if evaluation_mode == "parity":
        out["parity_mismatches"] = pose.baked_parity_mismatches
    return out


def _set_avar(stage, path, per):
    from pxr import Sdf
    prim = stage.GetPrimAtPath(path)
    for name, value in per.items():
        a = prim.GetAttribute("avars:" + name)
        if not a or not a.IsValid():
            a = prim.CreateAttribute(
                "avars:" + name, Sdf.ValueTypeNames.Double)
        a.Set(value)


def _build_stacked(bc_first, start_rz):
    """Two chains: one writes A, one over [B, C] hangs from A by policy.

    bc_first: author the hanging chain first in the file, so it executes
    AFTER the A chain (bottom sibling executes first) and reads fresh.
    False reverses both: the hanging chain runs first and reads the
    pre-write A. No precompile, so diagnostics carry compile warnings.
    """
    from pxr import Usd
    import rigexec

    stage = Usd.Stage.CreateInMemory()
    builder = rigexec.Builder.create(stage, "/Rig")
    joints = []
    for i, t in enumerate(JOINT_LOCALS[:3]):
        joints.append(builder.add_joint(
            "J%d" % i, _translate(*t), joints[-1] if joints else None))
    ca = builder.add_control("CA", _translate(*JOINT_WORLDS[0]))
    cb = [builder.add_control("C%d" % j, _translate(*JOINT_WORLDS[j]))
          for j in (1, 2)]

    order = []

    def _add_a():
        builder.add_fk_chain("FkA", [ca], [joints[0]])
        order.append("FkA")

    def _add_bc():
        fk = builder.add_fk_chain("FkBC", cb, [joints[1], joints[2]])
        fk.set_start_frame_policy("parent")
        order.append("FkBC")

    if bc_first:
        _add_bc()
        _add_a()
    else:
        _add_a()
        _add_bc()
    _set_avar(stage, ca.path, {"rz": start_rz})

    # Pin the premise: builder call order is file order.
    kids = [p.GetName() for p in
            stage.GetPrimAtPath("/Rig/Solvers").GetChildren()]
    assert kids == order, kids

    rig = rigexec.Rig(stage, "/Rig")
    pose = rig.evaluate(0)
    return {
        "stage": stage,
        "b": pose.joint_frame(joints[1].path).to_matrix4(),
        "c": pose.joint_frame(joints[2].path).to_matrix4(),
        "diagnostics": list(pose.diagnostics),
    }


def _build_fork():
    """Two roots, one chain over joints from both: policy must refuse."""
    from pxr import Usd
    import rigexec

    stage = Usd.Stage.CreateInMemory()
    builder = rigexec.Builder.create(stage, "/Rig")
    a0 = builder.add_joint("A0", _translate(0, 0, 0), None)
    a1 = builder.add_joint("A1", _translate(10, 0, 0), a0)
    b0 = builder.add_joint("B0", _translate(0, 10, 0), None)
    b1 = builder.add_joint("B1", _translate(0, 10, 0), b0)
    ca = builder.add_control("CA1", _translate(10, 0, 0))
    cb = builder.add_control("CB1", _translate(0, 20, 0))
    fk = builder.add_fk_chain("Fk", [ca, cb], [a1, b1])
    fk.set_start_frame_policy("parent")
    _set_avar(stage, a0.path, {"rz": 90.0})

    rig = rigexec.Rig(stage, "/Rig")
    pose = rig.evaluate(0)
    return {
        "stage": stage,
        "fk": fk,
        "a1": pose.joint_frame(a1.path).to_matrix4(),
        "b1": pose.joint_frame(b1.path).to_matrix4(),
        "diagnostics": list(pose.diagnostics),
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

    # -- 6. Derived is authored, bit for bit -------------------------------
    for mode in (None, "world", "parentRelative"):
        for cname, avars in SCENARIOS.items():
            for sname, start in START_POSES.items():
                authored = _build(mode, avars, start, True)
                derived = _build(mode, avars, start, False,
                                 policy="parent")
                assert authored["joints"] == derived["joints"], \
                    (mode, cname, sname)
                assert authored["solver"] == derived["solver"], \
                    (mode, cname, sname)
    # The mechanism, not just the behaviour: the compile materialized
    # the hierarchy walk as a session-layer target on the relationship.
    prim = derived["stage"].GetPrimAtPath(derived["fk"].path)
    assert [str(t) for t in
            prim.GetRelationship("rigExec:startFrame").GetTargets()] == \
        ["/Rig/Joints/J0"]
    print("  policy=parent, no relationship: identical to authored")

    # -- 6b. ... in baked and parity evaluation too ------------------------
    for mode in ("baked", "parity"):
        got = _build("parentRelative", SCENARIOS["stacked"],
                     START_POSES["start_general"], False,
                     policy="parent", evaluation_mode=mode)
        want = _build("parentRelative", SCENARIOS["stacked"],
                      START_POSES["start_general"], True)
        for i, (a, b) in enumerate(zip(want["joints"], got["joints"])):
            assert _close_vec(a, b, 1e-9), (mode, i, a, b)
        for i, (a, b) in enumerate(zip(want["solver"], got["solver"])):
            assert _close_vec(a, b, 1e-9), (mode, i, a, b)
        if mode == "parity":
            assert got["parity_mismatches"] == 0, got["parity_mismatches"]

    # -- 7. Authored wins: policy plus the relationship is the
    # relationship alone. Ran twice it would square the start map; ran
    # once it is bit-identical to authored-only.
    for mode in (None, "parentRelative"):
        for cname, avars in SCENARIOS.items():
            for sname, start in START_POSES.items():
                authored = _build(mode, avars, start, True)
                both = _build(mode, avars, start, True, policy="parent")
                assert authored["joints"] == both["joints"], \
                    (mode, cname, sname)
                assert authored["solver"] == both["solver"], \
                    (mode, cname, sname)

    # -- 8. The policy never guesses ---------------------------------------
    # Joints spanning two roots: absolute solve plus a warning, and no
    # derived target left on the relationship.
    fork = _build_fork()
    assert _close_vec(_origin(fork["a1"]), (10, 0, 0)), _origin(fork["a1"])
    assert _close_vec(_origin(fork["b1"]), (0, 20, 0)), _origin(fork["b1"])
    assert any("span providers" in d for d in fork["diagnostics"]), \
        fork["diagnostics"]
    rel = fork["stage"].GetPrimAtPath(
        fork["fk"].path).GetRelationship("rigExec:startFrame")
    assert not rel.GetTargets(), rel.GetTargets()
    # No provider ancestor at all (the chain IS the root): same deal.
    root = _build(None, {}, {}, False, policy="parent", chain=(0,),
                  precompile=False)
    assert any("no RigExecJoint/RigExecControl ancestor" in d
               for d in root["diagnostics"]), root["diagnostics"]
    rel = root["stage"].GetPrimAtPath(
        root["fk"].path).GetRelationship("rigExec:startFrame")
    assert not rel.GetTargets(), rel.GetTargets()

    # -- 9. The read is positional -----------------------------------------
    # Chain ordered BEFORE its provider's writer: inference still wires
    # the relationship, but the chain reads the pre-write frame and the
    # compile warns naming the reorder.
    stale = _build_stacked(False, 90.0)
    assert _close_vec(_origin(stale["b"]), (10, 0, 0)), _origin(stale["b"])
    assert _close_vec(_origin(stale["c"]), (20, 0, 0)), _origin(stale["c"])
    assert any("pre-write frame" in d for d in stale["diagnostics"]), \
        stale["diagnostics"]
    rel = stale["stage"].GetPrimAtPath(
        "/Rig/Solvers/FkBC").GetRelationship("rigExec:startFrame")
    assert [str(t) for t in rel.GetTargets()] == ["/Rig/Joints/J0"], \
        rel.GetTargets()
    print("  misordered chain: frozen at rest, warns naming the reorder")
    # Same rig, hanging chain first in the file (executes after the
    # writer): follows, with no warning.
    fresh = _build_stacked(True, 90.0)
    assert _close_vec(_origin(fresh["b"]), (0, 10, 0)), _origin(fresh["b"])
    assert _close_vec(_origin(fresh["c"]), (0, 20, 0)), _origin(fresh["c"])
    assert not any("pre-write frame" in d
                   for d in fresh["diagnostics"]), fresh["diagnostics"]
    print("  reordered chain:  rides to (0,10,0) / (0,20,0), silent")

    print("OK: FK chain startFrame end to end from Python")


if __name__ == "__main__":
    main()
