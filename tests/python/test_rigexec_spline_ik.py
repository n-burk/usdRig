"""End-to-end RigExecSplineIk from Python.

Authors a seven-joint chain, three controls, and a spline IK solver entirely
through the rigexec package, animates the end control, and asserts through
the native Rig wrapper that the bound joints stretch along the curve, that
the per-joint volume-preservation scale reaches the joint frames AND the
joint matrices as a non-uniform (1, s, s), and that the end control's twist
is distributed linearly along the chain.

Usage:
    python test_rigexec_spline_ik.py [schema_resources_dir]
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


def _length(a, b):
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))


def _sub(a, b):
    return [p - q for p, q in zip(a, b)]


def _dot(a, b):
    return sum(p * q for p, q in zip(a, b))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _unit(v):
    n = math.sqrt(_dot(v, v))
    return [p / n for p in v]


def _close(a, b, eps=1e-6):
    return abs(a - b) < eps


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    _setup_environment()

    from pxr import Usd
    import rigexec

    rigexec.load_schema_plugin(plugin_dir)
    assert "RigExecSplineIk" in rigexec.schema.names()

    stage = Usd.Stage.CreateInMemory()
    builder = rigexec.Builder.create(stage, "/Rig")

    # A straight chain along +X: joint i at rest x = i, aiming +X, up +Y.
    joints = [builder.add_joint("J%d" % i, _translate(i, 0, 0))
              for i in range(7)]
    root = builder.add_control("Root", _translate(0, 0, 0))
    mid = builder.add_control("Mid", _translate(3, 0, 0))
    end = builder.add_control("End", _translate(6, 0, 0))

    weights = [0.0, 0.1429, 0.5, 1.0, 0.25, 0.3571, 0.0714]
    spine = builder.add_spline_ik("Spine", root, mid, end, joints=joints)
    assert spine.valid and spine.path == "/Rig/Solvers/Spine", spine.path
    spine.set_volume_weights(weights)
    spine.set_rest_length("curve")
    spine.set_preserve_volume(1.0)
    spine.set_mid_follow_weight(0.5)

    prim = stage.GetPrimAtPath(spine.path)
    assert prim.GetTypeName() == "RigExecSplineIk", prim.GetTypeName()
    assert [str(p) for p in prim.GetRelationship("rigExec:joints").GetTargets()] \
        == [j.path for j in joints]
    assert [str(p) for p in prim.GetRelationship("rigExec:rootControl").GetTargets()] \
        == [root.path]
    assert list(prim.GetAttribute("rigExec:volumeWeights").Get()) == \
        [round(w, 6) for w in weights] or \
        all(_close(a, b) for a, b in
            zip(prim.GetAttribute("rigExec:volumeWeights").Get(), weights))

    # Animate: rest at frame 0; at frame 100 the end control is pulled +3
    # along the chain with the mid control riding on its follow point; at
    # frame 200 the end control additionally twists -60 degrees about +X.
    def key(handle, attr, samples):
        a = stage.GetPrimAtPath(handle.path).GetAttribute(attr)
        assert a.IsValid(), attr
        for t, v in samples:
            a.Set(v, Usd.TimeCode(t))

    key(end, "avars:tx", [(0, 0.0), (100, 3.0), (200, 3.0)])
    key(mid, "avars:tx", [(0, 0.0), (100, 1.5), (200, 1.5)])
    key(end, "avars:rx", [(0, 0.0), (100, 0.0), (200, -60.0)])

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()

    # Frame 0: rest reproduced exactly (a straight chain has no residual).
    pose0 = rig.evaluate(0)
    assert len(pose0.solver_frames(spine.path)) == 7
    for i, j in enumerate(joints):
        f = pose0.joint_frame(j.path)
        assert f.valid and not f.degenerate
        assert all(_close(a, b) for a, b in zip(f.origin, (i, 0, 0))), f.origin
        assert _close(_length(f.y_axis, f.origin), 1.0)

    # Frame 100: ratio 1.5, joints at 1.5 spacing, s = 1 - w_i * 0.5 on the
    # Y and Z handles, X untouched.
    pose100 = rig.evaluate(100)
    for i, j in enumerate(joints):
        f = pose100.joint_frame(j.path)
        assert _close(f.origin[0], 1.5 * i) and _close(f.origin[1], 0.0) \
            and _close(f.origin[2], 0.0), (i, f.origin)
        s = 1.0 - weights[i] * 0.5
        assert _close(_length(f.x_axis, f.origin), 1.0, 1e-5), (i, f.x_axis)
        assert _close(_length(f.y_axis, f.origin), s, 1e-5), (i, f.y_axis)
        assert _close(_length(f.z_axis, f.origin), s, 1e-5), (i, f.z_axis)
    m = pose100.joint_matrix(joints[3].path)
    assert _close(math.sqrt(_dot(m[0:3], m[0:3])), 1.0, 1e-5), m
    assert _close(math.sqrt(_dot(m[4:7], m[4:7])), 0.5, 1e-5), m
    assert _close(math.sqrt(_dot(m[8:11], m[8:11])), 0.5, 1e-5), m

    # Frame 200: end twist -60 degrees, linear in t_i = i / 6 -- equal
    # increments between equally spaced joints.
    pose200 = rig.evaluate(200)
    angles = []
    for i, j in enumerate(joints):
        f0 = pose100.joint_frame(j.path)
        f1 = pose200.joint_frame(j.path)
        x = _unit(_sub(f1.x_axis, f1.origin))
        y0 = _unit(_sub(f0.y_axis, f0.origin))
        y1 = _unit(_sub(f1.y_axis, f1.origin))
        angles.append(math.degrees(
            math.atan2(_dot(_cross(y0, y1), x), _dot(y0, y1))))
    assert _close(angles[0], 0.0, 1e-5), angles
    assert _close(angles[6], -60.0, 1e-5), angles
    for i in range(1, 7):
        assert _close(angles[i] - angles[i - 1], -10.0, 1e-5), angles

    # The length floor (inputs:minLengthRatio): at frame 300 the end
    # control is driven 4.5 back toward the root (chord 1.5 of 6) with the
    # mid on its follow point. Off, the chain crumples; at 0.5 the end CV
    # is held 3 ahead of the root along the root's chain axis and the
    # joints lie at exactly half spacing, in order.
    key(end, "avars:tx", [(300, -4.5)])
    key(mid, "avars:tx", [(300, -2.25)])
    key(end, "avars:rx", [(300, 0.0)])
    pose300 = rig.evaluate(300)
    crumpled = pose300.joint_frame(joints[6].path).origin[0]
    assert crumpled < 3.0, crumpled
    if not hasattr(spine, "set_min_length_ratio"):
        # A build of the bindings that predates the floor still passes
        # everything above; say so rather than failing a stale build tree.
        print("SKIP: this rigexec build has no set_min_length_ratio "
              "(inputs:minLengthRatio); floor case not run")
        print("OK: spline IK end to end from Python")
        return
    spine.set_min_length_ratio(0.5)
    assert _close(prim.GetAttribute("inputs:minLengthRatio").Get(), 0.5)
    pose300 = rig.evaluate(300)
    for i, j in enumerate(joints):
        f = pose300.joint_frame(j.path)
        assert _close(f.origin[0], 0.5 * i, 1e-6) and _close(f.origin[1], 0.0)             and _close(f.origin[2], 0.0), (i, f.origin)
    # Inert at rest and under the stretch: frames 0 and 100 unchanged.
    for i, j in enumerate(joints):
        assert all(_close(a, b) for a, b in zip(
            rig.evaluate(0).joint_frame(j.path).origin, (i, 0, 0)))
        assert _close(rig.evaluate(100).joint_frame(j.path).origin[0],
                      1.5 * i)

    print("OK: spline IK end to end from Python")


if __name__ == "__main__":
    main()
