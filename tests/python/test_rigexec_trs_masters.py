"""The TRS masters carry the rig and never deform it.

Main / Shot / Aux wrap Controls and Joints, so the whole rig is
localized to those three transforms. What that has to mean, and what
this file measures:

  1. moving a master carries every joint by ONE map -- the same S for
     all of them -- so nothing deforms when a master moves;
  2. scaling a master carries every joint by one UNIFORMLY SCALED map,
     so the rig arrives at the same pose, just bigger;
  3. a control posed first still rides that one map afterwards, which
     is the claim that deformation happens in master space;
  4. scaling a master and putting it back leaves the rig exactly where
     it started -- bit for bit, not nearly -- so a control posed after
     the round trip lands where it would have without it.

(4) is the guard for "after scaling the trs nodes, when I zero
everything out and scale is back to 1, turning the head breaks the
face": the evaluator is not where that happens, and this test is what
says so the next time somebody wonders.

Per evaluated file (Biped.usda, Biped_stack.usda), per mode (dynamic,
baked).

Usage:
    python test_rigexec_trs_masters.py [schema_resources_dir]
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_FILES = [
    os.path.join(_REPO, "examples", "biped", "Biped.usda"),
    os.path.join(_REPO, "examples", "biped", "Biped_stack.usda"),
]
_RIG = "/Biped/Rig"
_MAIN = _RIG + "/Main"
_SHOT = _MAIN + "/Shot"
_AUX = _SHOT + "/Aux"
_CTL = _AUX + "/Controls"
_JNT = _AUX + "/Joints"
_ROOT_JOINT = _JNT + "/hips_def"
_HEAD = _CTL + "/M_Body/M_Torso/M_Chest/M_ChestTop/M_Neck/M_Head"
_TORSO = _CTL + "/M_Body/M_Torso"
# The squetch clusters hang off this one. A torso turn does not
# reach them, so a scale guard that poses only the torso passes
# while they are out by 25 units.
_HEADWIRE = _CTL + "/headwire_top_follow/M_HeadwireTop"
# The skinned body. Absent from the standalone assemblies, which carry
# the rig without the model -- the mesh check skips itself there rather
# than failing on a file that was never meant to have one.
_BODY = "/Biped/Geom/body_geo.points"

# The carry is a product of composed matrices, so it is exact only to
# the rounding of that product. The biped's deepest joints are seventeen
# matrices from the root (hips -> six spine -> chest -> four neck ->
# skull -> face_upper -> head_tip -> a brow cluster), and under a
# three-axis master rotation those accumulate 1.8e-06 of pure
# double-precision drift -- measured, with the carry otherwise exact.
# 1e-5 on a rig whose joints sit ~150 units from the origin is about
# 1e-7 relative, which still catches anything real by four orders of
# magnitude: the bug this file was written for missed by 3.8 UNITS.
_EPS = 1e-5


def _mul(a, b):
    out = [0.0] * 16
    for r in range(4):
        for c in range(4):
            out[r * 4 + c] = sum(a[r * 4 + k] * b[k * 4 + c]
                                 for k in range(4))
    return out


def _inv(m):
    """Inverse of a row-vector 4x4 whose last column is (0,0,0,1).

    Not the rigid inverse the sibling biped tests use: a scaled
    master's map is not rigid, and inverting it as though it were would
    hide exactly the error this file is looking for.
    """
    a = [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]]
    det = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
           - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
           + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    assert abs(det) > 1e-12, det
    inv = [[0.0] * 3 for _ in range(3)]
    for r in range(3):
        for c in range(3):
            r0, r1 = [k for k in range(3) if k != r]
            c0, c1 = [k for k in range(3) if k != c]
            minor = a[r0][c0] * a[r1][c1] - a[r0][c1] * a[r1][c0]
            # Transposed on assignment: the cofactor matrix's transpose
            # is the adjugate.
            inv[c][r] = ((-1) ** (r + c)) * minor / det
    t = m[12:15]
    nt = [-sum(t[k] * inv[k][c] for k in range(3)) for c in range(3)]
    return [inv[0][0], inv[0][1], inv[0][2], 0,
            inv[1][0], inv[1][1], inv[1][2], 0,
            inv[2][0], inv[2][1], inv[2][2], 0,
            nt[0], nt[1], nt[2], 1]


def _set_avar(stage, path, name, value):
    from pxr import Sdf
    prim = stage.GetPrimAtPath(path)
    assert prim, path
    a = prim.GetAttribute("avars:" + name)
    if not a or not a.IsValid():
        a = prim.CreateAttribute("avars:" + name, Sdf.ValueTypeNames.Double)
    a.Set(value)


def _zero(stage, path, names):
    for n in names:
        _set_avar(stage, path, n, 1.0 if n.startswith("s") else 0.0)


def _points(rig, stage):
    """The skinned body's deformed points, or () if this file has none."""
    pose = rig.evaluate(0)
    assert pose.valid, [d for d in pose.diagnostics if "error" in d]
    properties = pose.moved_properties()
    if _BODY not in properties:
        return ()
    return [tuple(p) for p in properties[_BODY]]


def _all_points(rig):
    """Every moved mesh, keyed by property path.

    Not just _BODY. The deformers that break rigidity are attached
    per-mesh, so a guard watching one mesh is a guard that passes while
    the hair is 1.1 units out and the lashes 0.29: measured, on this rig,
    with body_geo reading 0.12.
    """
    pose = rig.evaluate(0)
    assert pose.valid, [d for d in pose.diagnostics if "error" in d]
    properties = pose.moved_properties()
    return {m: [tuple(v) for v in properties[m]]
            for m in sorted(properties) if m.endswith(".points")}


def _rides(before, after, s, label, eps):
    """Assert every mesh point rides the joints' carry map `s` exactly.

    No best-fit: `s` is read off the root joint, which check 1 has
    already established is the ONE map the whole rig rides. A point that
    needs a different map is a point some deformer moved in the wrong
    frame, and the difference is the error in rig units.
    """
    worst, culprit = 0.0, None
    for mesh, pts in before.items():
        for p, q in zip(pts, after[mesh]):
            for k in range(3):
                v = p[0] * s[k] + p[1] * s[4 + k] + p[2] * s[8 + k] + s[12 + k]
                if abs(q[k] - v) > worst:
                    worst, culprit = abs(q[k] - v), mesh
    assert worst < eps, (label, culprit, worst)
    print("    %-28s %d points ride one map (worst %.2e)"
          % (label, sum(len(v) for v in before.values()), worst))


def _carry(rest, moved, joints, label):
    """Assert every joint rides ONE map, and return it.

    The map is read off the root joint and then held against all the
    others: a rig that deformed would need a different map per joint,
    which is precisely what fails here.
    """
    s = _mul(_inv(rest[_ROOT_JOINT]), moved[_ROOT_JOINT])
    worst, culprit = 0.0, None
    for j in joints:
        want = _mul(rest[j], s)
        err = max(abs(p - q) for p, q in zip(want, moved[j]))
        if err > worst:
            worst, culprit = err, j
    assert worst < _EPS, (label, culprit, worst)
    print("    %-28s %d joints ride one map (worst %.2e)"
          % (label, len(joints), worst))
    return s


def _scale_of(s):
    """The uniform scale in a carry map, and how far from uniform it is."""
    rows = [s[0:3], s[4:7], s[8:11]]
    lens = [math.sqrt(sum(v * v for v in r)) for r in rows]
    return sum(lens) / 3.0, max(lens) - min(lens)


def _check_file(path, mode):
    from pxr import Usd
    import rigexec

    print("  %s [%s]" % (os.path.basename(path), mode))
    stage = Usd.Stage.Open(path)
    assert stage, path
    # The session layer, so the sequence below never touches the asset.
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    rig = rigexec.Rig(stage, _RIG)
    rig.cpu_reference = True

    def _frames():
        pose = rig.evaluate(0)
        assert pose.valid, [d for d in pose.diagnostics if "error" in d]
        return {j: pose.joint_matrix(j) for j in pose.joint_paths()}

    rest = _frames()
    joints = sorted(rest)
    assert len(joints) > 200, len(joints)
    assert _ROOT_JOINT in rest, _ROOT_JOINT

    # 1. A rigid master move carries the rig and deforms nothing.
    _set_avar(stage, _MAIN, "tx", 25.0)
    _set_avar(stage, _MAIN, "ty", -10.0)
    _set_avar(stage, _MAIN, "rz", 20.0)
    s = _carry(rest, _frames(), joints, "Main moved")
    scale, spread = _scale_of(s)
    assert abs(scale - 1.0) < _EPS and spread < _EPS, (scale, spread)
    assert math.dist(s[12:15], (0, 0, 0)) > 1.0, s[12:15]
    _zero(stage, _MAIN, ("tx", "ty", "rz"))

    # 1b. And a rotation on all three axes, which is the one that finds a
    #     space switch filtering the carry away: a single-axis turn can
    #     leave a twist-filtered source looking almost right.
    _set_avar(stage, _MAIN, "rx", 15.0)
    _set_avar(stage, _MAIN, "ry", -35.0)
    _set_avar(stage, _MAIN, "rz", 20.0)
    s = _carry(rest, _frames(), joints, "Main turned on 3 axes")
    scale, spread = _scale_of(s)
    assert abs(scale - 1.0) < _EPS and spread < _EPS, (scale, spread)
    _zero(stage, _MAIN, ("rx", "ry", "rz"))

    # 2. A uniform master scale carries it to the same pose, bigger.
    for ax in "xyz":
        _set_avar(stage, _SHOT, "s" + ax, 2.0)
    s = _carry(rest, _frames(), joints, "Shot scaled x2")
    scale, spread = _scale_of(s)
    assert abs(scale - 2.0) < 1e-9 and spread < 1e-9, (scale, spread)
    _zero(stage, _SHOT, ("sx", "sy", "sz"))

    # 3. Deformation is local: pose a control first, and the masters
    #    still carry the POSED rig by one map.
    _set_avar(stage, _HEAD, "ry", 40.0)
    posed = _frames()
    head_shift = max(
        math.dist(posed[j][12:15], rest[j][12:15]) for j in joints)
    assert head_shift > 0.5, head_shift
    _set_avar(stage, _AUX, "tx", 17.0)
    for ax in "xyz":
        _set_avar(stage, _AUX, "s" + ax, 1.5)
    _carry(posed, _frames(), joints, "Aux moved over a pose")
    _zero(stage, _AUX, ("tx", "sx", "sy", "sz"))

    # 3b. The SKINNED MESH scales with the masters, not just the joints.
    #     The curve wires deform points rather than frames, so a wire
    #     measuring its driver's offset in the wrong units moves the mesh
    #     while every joint above stays exactly right -- invisible to
    #     every check above, and invisible at unit scale by
    #     construction. Pose first: the bug only appears once a driver
    #     has somewhere to move.
    #     Two poses, because the two deformer families that get this
    #     wrong are reached by different controls. A torso turn drives
    #     the curve wires; a headwire control drives the squetch
    #     CLUSTERS, which the torso turn leaves at the floor -- so a
    #     guard with only the first one passes while the second is out
    #     by 25 units.
    for label, control, avar, value in (
            ("wires", _TORSO, "rz", 15.0),
            ("clusters", _HEADWIRE, "tx", 2.0)):
        # `not prim`, never `is None`: GetPrimAtPath hands back an INVALID
        # prim for a path the stage does not have, which is truthy-looking
        # enough to sail past an identity test and fail three lines later.
        if not stage.GetPrimAtPath(control):
            continue
        _set_avar(stage, control, avar, value)
        one = _points(rig, stage)
        if one:
            for ax in "xyz":
                _set_avar(stage, _MAIN, "s" + ax, 2.0)
            two = _points(rig, stage)
            worst = max(max(abs(b - 2.0 * a) for a, b in zip(pa, pb))
                        for pa, pb in zip(one, two))
            # 1e-4 on a mesh whose points sit ~150 units out. The two
            # bugs this guards missed by 1.54 and 3.62 units (wires) and
            # 25.02 (clusters).
            assert worst < 1e-4, ("skinned mesh under master scale",
                                  label, worst)
            print("    %-28s %d points scale exactly (worst %.2e)"
                  % ("skin follows: " + label, len(one), worst))
            for ax in "xyz":
                _set_avar(stage, _MAIN, "s" + ax, 1.0)
        _set_avar(stage, control, avar, 0.0)

    # 3c. The skinned MESH rides the master's carry under a TRANSLATION,
    #     not only under a scale. 3b above measures scale alone, and
    #     scale is the one case the old half-fix already handled: a
    #     cluster whose points are already posed conjugated by its
    #     space's SCALE and by nothing else, so a translated master left
    #     head_top_aim_cluster dragging body_geo 5.70 units out of shape
    #     while every joint above it stayed exactly right and every
    #     scale check here passed.
    #
    #     Posed first and across EVERY mesh, for the reasons 3b gives.
    #     The map comes from the joints, so this asserts the mesh and the
    #     skeleton agree -- which is the whole claim.
    if not stage.GetPrimAtPath(_HEADWIRE):
        pass
    else:
        _set_avar(stage, _HEADWIRE, "tx", 2.0)
        _set_avar(stage, _HEAD, "rz", 22.0)
        before_pts = _all_points(rig)
        if before_pts:
            before_j = _frames()
            # Translate and ROTATE, because the two are not the same test.
            # A translation does not change a difference vector, so every
            # deformer that computes a displacement at rest and adds it to
            # carried points is INVISIBLE under one: the nine posed curve
            # wires read 0.000015 translated and 1.10 rotated, the hair
            # wire alone being 0.76 of it. The three-axis case is last
            # because a single axis can leave a filtered carry looking
            # almost right, exactly as 1b guards for the joints.
            for label, avars in (
                    ("translate", (("tx", 40.0), ("tz", 25.0))),
                    ("rotate", (("ry", 35.0),)),
                    ("rotate 3-axis + translate",
                     (("rx", 12.0), ("ry", 35.0), ("rz", -8.0),
                      ("tx", 40.0))),
            ):
                for name, value in avars:
                    _set_avar(stage, _MAIN, name, value)
                s = _carry(before_j, _frames(), joints,
                           "Main %s over a face pose" % label)
                # 2e-4 on points ~150 units out. The composed carry is a
                # product of matrices and the three-axis case accumulates
                # 4.8e-05 of pure double-precision drift, measured with it
                # otherwise exact; the two bugs this guards missed by 5.70
                # units (clusters) and 1.10 (wires), four orders clear.
                _rides(before_pts, _all_points(rig),
                       s, "skin follows: " + label, 2e-4)
                _zero(stage, _MAIN, [n for n, _ in avars])
        _zero(stage, _HEADWIRE, ("tx",))
        _set_avar(stage, _HEAD, "rz", 0.0)

    # 4. Scale up, scale back, and the pose is the one it started with.
    #    Exact: a rest frame captured while the masters were scaled
    #    would survive the restore and show up here.
    for ax in "xyz":
        _set_avar(stage, _MAIN, "s" + ax, 2.0)
    _frames()
    for ax in "xyz":
        _set_avar(stage, _MAIN, "s" + ax, 1.0)
    back = _frames()
    worst = max(max(abs(p - q) for p, q in zip(posed[j], back[j]))
                for j in joints)
    assert worst == 0.0, ("scale round trip", worst)
    print("    %-28s %d joints bit-identical"
          % ("scale round trip", len(joints)))
    _set_avar(stage, _HEAD, "ry", 0.0)


def main():
    _setup_environment()
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)
    for path in _FILES:
        for mode in ("graph",):
            _check_file(path, mode)
    print("OK: the TRS masters carry the rig and never deform it")


if __name__ == "__main__":
    main()
