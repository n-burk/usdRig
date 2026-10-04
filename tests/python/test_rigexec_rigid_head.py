"""The head and shoulders stay rigid when the spine squashes.

M_HipSwivel ty=-50 stretches the spine 2.45x. The spline IK's volume
preservation answers with a per-joint squash -- spine_3_def goes to
(1, 0.275, 0.275) -- which is correct and wanted. What was NOT wanted
is that it reached the head: the cranium grew 6.56% taller and 1.64%
narrower, and the shoulders went with it.

The scale MAGNITUDES were never wrong. Every joint from the clavicle up
already had a scale constraint putting its scale back to 1. What rode
through was SHEAR: composing the squashed parent frame with each
child's rotated rest offset raises a shear term, and a constraint that
replaced translation, rotation and scale rebuilt the frame from the
input's own decomposition -- so the shear it had never been asked about
survived. It accumulated 0.0587 at spine_0_def, 0.2327 by spine_5_def,
and arrived at skull_def and face_upper_def as 0.157, about nine
degrees.

The rig's authored behaviour cannot produce this: its joints compensate
their parent's scale, so none inherits it and none carries shear.

Three things are asserted, because fixing any one of them alone is easy
and wrong:

  1. every joint from the clavicle up has a RIGID posed frame;
  2. the spine's squash is still there (deleting it would pass 1);
  3. Main s=2 still scales every joint by exactly 2 (refusing to
     inherit any scale at all would pass 1 and 2).

Usage:
    python test_rigexec_rigid_head.py [schema_resources_dir]
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_STACK = os.path.join(_REPO, "examples", "biped", "Biped_stack.usda")
_RIG = "/Biped/Rig"

# Everything at or above the clavicle: the two families the animator
# reported. Named rather than derived from the namespace so that a
# joint disappearing from the rig fails this file instead of silently
# shrinking its coverage.
_RIGID = [
    "clavicle_l_def", "clavicle_r_def", "shoulder_l_def", "shoulder_r_def",
    "clavicle_trans_l_def", "clavicle_trans_r_def",
    "neck_base_def", "neck_0_def", "neck_1_def", "neck_2_def", "neck_3_def",
    "skull_def", "face_upper_def", "face_mid_def", "face_lower_def",
    "head_tip_def", "jaw_def", "teeth_upper_def", "teeth_lower_def",
    "nose_def", "nose_bridge_def",
    "eyeSocket_l_def", "eyeSocket_r_def", "eye_l_def", "eye_r_def",
    "brow_main_l_def", "brow_main_r_def",
]

# The squash that must survive. spine_3_def carries volumeWeight 0.5,
# the largest on the chain, so it is where the effect is biggest.
_SQUASHED = "spine_3_def"

# A joint frame is built from unit axes, so a rigid frame reads 1.0 on
# every axis length and 0.0 on every pair of axis directions. 1e-4 is
# three orders under the 0.157 shear this file exists to catch, and one
# under the 0.0031 scale it dragged along with it.
_EPS = 1e-4


def _control(stage, name):
    for prim in stage.Traverse():
        if (prim.GetName() == name
                and str(prim.GetTypeName()) == "RigExecControl"):
            return prim
    raise AssertionError("no RigExecControl named " + name)


def _set(prim, avar, value):
    from pxr import Sdf
    a = prim.GetAttribute("avars:" + avar)
    if not a or not a.IsValid():
        a = prim.CreateAttribute("avars:" + avar, Sdf.ValueTypeNames.Double)
    a.Set(value)


def _frames(rig):
    """name -> (axis lengths, worst axis dot) of every posed joint frame."""
    from pxr import Gf
    pose = rig.evaluate(0)
    assert pose.valid, [d for d in pose.diagnostics
                        if not d.startswith("warning:")]
    out = {}
    for path in pose.joint_paths():
        f = pose.joint_frame(path, True)
        if not f.valid:
            continue
        o = Gf.Vec3d(*f.origin)
        axes = [Gf.Vec3d(*f.x_axis) - o, Gf.Vec3d(*f.y_axis) - o,
                Gf.Vec3d(*f.z_axis) - o]
        lengths = [v.GetLength() for v in axes]
        if min(lengths) <= 0.0:
            continue
        unit = [v / l for v, l in zip(axes, lengths)]
        skew = max(abs(Gf.Dot(unit[0], unit[1])),
                   abs(Gf.Dot(unit[0], unit[2])),
                   abs(Gf.Dot(unit[1], unit[2])))
        out[str(path).rsplit("/", 1)[-1]] = (lengths, skew)
    return out


def _points(rig, prop):
    pose = rig.evaluate(0)
    assert pose.valid
    return [tuple(v) for v in pose.moved_property(prop)]


def _check_head_is_rigid(rig, hip):
    for value in (-50.0, 50.0):
        _set(hip, "ty", value)
        frames = _frames(rig)
        missing = [n for n in _RIGID if n not in frames]
        assert not missing, ("joints gone from the rig", missing)
        bad = []
        for name in _RIGID:
            lengths, skew = frames[name]
            if skew > _EPS or max(abs(l - 1.0) for l in lengths) > _EPS:
                bad.append("%s %s skew %.5f"
                           % (name, ["%.5f" % l for l in lengths], skew))
        assert not bad, ("M_HipSwivel ty=%g deforms the head" % value, bad)

        squash = frames[_SQUASHED][0]
        assert abs(squash[0] - 1.0) < 1e-3, (
            "the spine's aim axis should not scale", squash)
        # Both directions, and both are a change of at least 10%: ty=-50
        # stretches the spine and thins it to 0.2731, ty=+50 compresses
        # it and fattens it to 1.2494.
        assert min(abs(squash[1] - 1.0), abs(squash[2] - 1.0)) > 0.1, (
            "the spine's volume preservation has stopped working", squash)
        assert frames[_SQUASHED][1] < _EPS, (
            "the spine itself is sheared", frames[_SQUASHED])
        print("    ty=%-6g head rigid on %d joints; %s goes to %.4f"
              % (value, len(_RIGID), _SQUASHED, squash[1]))
    _set(hip, "ty", 0.0)


def _check_the_rig_still_scales(rig, main):
    for axis in ("sx", "sy", "sz"):
        _set(main, axis, 2.0)
    frames = _frames(rig)
    bad = [(n, ["%.5f" % l for l in v[0]], v[1])
           for n, v in frames.items()
           if max(abs(l - 2.0) for l in v[0]) > 1e-3 or v[1] > _EPS]
    for axis in ("sx", "sy", "sz"):
        _set(main, axis, 1.0)
    assert not bad, ("Main s=2 no longer reaches every joint", bad[:8])
    print("    Main s=2 scales all %d joints by exactly 2" % len(frames))


def _check_the_cranium_does_not_change(rig, hip):
    """The joints are the cause; the mesh is what the animator sees."""
    _set(hip, "ty", 0.0)
    rest = _points(rig, "/Biped/Geom/body_geo.points")
    cranium = [i for i, q in enumerate(rest) if q[1] > 170.0]
    assert len(cranium) > 50, len(cranium)
    top = max(cranium, key=lambda i: rest[i][1])
    low = min(cranium, key=lambda i: rest[i][1])
    wide = max((math.dist(rest[i], rest[j]), i, j)
               for i in cranium[::7] for j in cranium[::7])
    pairs = [("height", top, low), ("width", wide[1], wide[2])]
    for value in (-50.0, 50.0):
        _set(hip, "ty", value)
        moved = _points(rig, "/Biped/Geom/body_geo.points")
        for label, i, j in pairs:
            was = math.dist(rest[i], rest[j])
            now = math.dist(moved[i], moved[j])
            change = abs(now - was) / was
            # 6.56% and 1.64% measured before the fix; 0.1% is the floor
            # that separates those from the arithmetic.
            assert change < 1e-3, (
                "cranium %s changes %.3f%% at ty=%g"
                % (label, 100 * change, value))
        print("    ty=%-6g cranium height and width unchanged" % value)
    _set(hip, "ty", 0.0)


def main():
    _setup_environment()
    from pxr import Usd
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)

    stage = Usd.Stage.Open(_STACK)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    hip = _control(stage, "M_HipSwivel")
    main_ctl = _control(stage, "Main")

    # Baked only: it is the path the biped runs, and one compile is what
    # keeps this file inside the suite's timeout.
    rig = rigexec.Rig(stage, _RIG)
    rig.evaluation_mode = "baked"

    _check_head_is_rigid(rig, hip)
    _check_the_rig_still_scales(rig, main_ctl)
    _check_the_cranium_does_not_change(rig, hip)
    print("OK: the spine squashes, the head does not")


if __name__ == "__main__":
    main()
