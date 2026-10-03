"""The biped hand follows the wrist, both layers, both paths.

Guards the regression this file was written for: rotating an FK wrist
control moved the finger CONTROLS (via hand_*_follow) but left the
finger JOINTS -- and the skinned hand -- frozen, because the finger
FkChains solved absolute. The chains now carry
rigExec:startFramePolicy = "parent" (the compile derives the wrist from
the joint hierarchy) and execute after the arm blends (Solvers order).

Per evaluated file (Biped.usda, Biped_layered.usda), per mode (reference,
baked):

  1. wrist FK ry=30 carries every left finger joint rigidly: each lands
     on S applied to its rest, where S is the wrist's own rest->pose
     map. Exact once (R^2 would miss by ~units, frozen by ~3);
  2. a finger curl composes with the wrist carry (wrist+curl == S of
     curl-only), and an IK move at half blend rides the same S;
  3. no startFrame diagnostics (no pre-write, spanning, or missing
     provider warnings);
  4. the arm itself still solves (elbow pose moves the wrist).

Usage:
    python test_rigexec_biped_hand.py [schema_resources_dir]
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
    os.path.join(_REPO, "examples", "biped", "Biped_layered.usda"),
]
_RIG = "/Biped/Rig"
_WRIST_CTL = (_RIG + "/Controls/hips_ctl/torso_ctl/spine_end_pivot/"
              "spine_end_ctl/clavicle_l_ctl/arm_l_root/"
              "arm_l_fk_shoulder_l_bind/arm_l_fk_elbow_l_bind/"
              "arm_l_fk_wrist_l_bind")
_SHOULDER_CTL = (_RIG + "/Controls/hips_ctl/torso_ctl/spine_end_pivot/"
                "spine_end_ctl/clavicle_l_ctl/arm_l_root/"
                "arm_l_fk_shoulder_l_bind")
_ELBOW = (_RIG + "/Joints/hips_bind/spine_0_bind/spine_1_bind/spine_2_bind/"
         "spine_3_bind/spine_4_bind/spine_5_bind/chest_bind/"
         "clavicle_l_bind/shoulder_l_bind/elbow_l_bind")
_WRIST = (_RIG + "/Joints/hips_bind/spine_0_bind/spine_1_bind/spine_2_bind/"
          "spine_3_bind/spine_4_bind/spine_5_bind/chest_bind/"
          "clavicle_l_bind/shoulder_l_bind/elbow_l_bind/wrist_l_bind")
_INDEX_FK = (_RIG + "/Controls/index_001_l_bind_fk_follow/"
             "index_001_l_bind_fk")
_ARM_PARAMS = _RIG + "/Controls/arm_l_params"
_ARM_IK = _RIG + "/Controls/arm_l_ik"

_FINGERS = ("index_", "middle_", "ring_", "pinky", "thumb")
_EPS = 1e-6


def _mul(a, b):
    out = [0.0] * 16
    for r in range(4):
        for c in range(4):
            out[r * 4 + c] = sum(a[r * 4 + k] * b[k * 4 + c]
                                 for k in range(4))
    return out


def _inv_rigid(m):
    """Inverse of a row-vector rigid 4x4 (rotation + last-row shift)."""
    r = [m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]]
    t = m[12:15]
    nt = [-sum(t[k] * r[k * 3 + c] for k in range(3)) for c in range(3)]
    return [r[0], r[1], r[2], 0, r[3], r[4], r[5], 0,
            r[6], r[7], r[8], 0, nt[0], nt[1], nt[2], 1]


def _close(a, b, eps=_EPS):
    return abs(a - b) < eps


def _close_vec(a, b, eps=_EPS):
    return len(a) == len(b) and all(_close(p, q, eps) for p, q in zip(a, b))


def _set_avar(stage, path, name, value):
    from pxr import Sdf
    prim = stage.GetPrimAtPath(path)
    assert prim, path
    a = prim.GetAttribute("avars:" + name)
    if not a or not a.IsValid():
        a = prim.CreateAttribute("avars:" + name, Sdf.ValueTypeNames.Double)
    a.Set(value)


def _finger_joints(stage):
    from pxr import Usd
    wrist = stage.GetPrimAtPath(_WRIST)
    assert wrist, _WRIST
    out = []
    for p in Usd.PrimRange(wrist):
        if p.GetTypeName() != "RigExecJoint":
            continue
        n = p.GetName()
        if any(k in n for k in _FINGERS) and "driver" not in n \
                and "combo" not in n:
            out.append(p.GetPath().pathString)
    out.sort()
    assert len(out) >= 20, out
    return out


def _check_file(path, mode):
    from pxr import Usd
    import rigexec

    print("  %s [%s]" % (os.path.basename(path), mode))
    stage = Usd.Stage.Open(path)
    assert stage, path
    joints = _finger_joints(stage)
    rig = rigexec.Rig(stage, _RIG)
    rig.evaluation_mode = mode

    def _frames():
        pose = rig.evaluate(0)
        mats = {j: pose.joint_frame(j).to_matrix4() for j in joints}
        mats[_WRIST] = pose.joint_frame(_WRIST).to_matrix4()
        mats[_ELBOW] = pose.joint_frame(_ELBOW).to_matrix4()
        return pose, mats

    # Rest first: this evaluate compiles, so its diagnostics carry the
    # compile warnings.
    pose, rest = _frames()
    for bad in ("pre-write frame", "span providers",
                "no RigExecJoint/RigExecControl ancestor"):
        assert not any(bad in d for d in pose.diagnostics), \
            [d for d in pose.diagnostics if bad in d]

    # 1. The wrist carry, exactly once.
    _set_avar(stage, _WRIST_CTL, "ry", 30.0)
    _, moved = _frames()
    s = _mul(_inv_rigid(rest[_WRIST]), moved[_WRIST])
    worst = 0.0
    for j in joints:
        want = _mul(rest[j], s)
        err = max(abs(p - q) for p, q in zip(want, moved[j]))
        worst = max(worst, err)
        assert _close_vec(want, moved[j]), (j, want, moved[j])
    print("    %d finger joints ride S exactly (worst err %.2e)"
          % (len(joints), worst))

    # 2. A curl composes with the carry: wrist+curl == S of curl-only.
    _set_avar(stage, _INDEX_FK, "rz", 40.0)
    _, both = _frames()
    _set_avar(stage, _WRIST_CTL, "ry", 0.0)
    _, curled = _frames()
    for j in joints:
        want = _mul(curled[j], s)
        assert _close_vec(want, both[j]), (j, want, both[j])
    print("    wrist+curl == S of curl-only on all %d joints"
          % len(joints))

    # 2b. The anchor is the joint, so the carry holds at every IK/FK
    # weight: half blend plus an IK move rides the same S invariant.
    _set_avar(stage, _INDEX_FK, "rz", 0.0)
    _set_avar(stage, _ARM_PARAMS, "ikfk", 0.5)
    _set_avar(stage, _ARM_IK, "tx", 10.0)
    _, ik = _frames()
    s_ik = _mul(_inv_rigid(rest[_WRIST]), ik[_WRIST])
    for j in joints:
        want = _mul(rest[j], s_ik)
        assert _close_vec(want, ik[j]), (j, want, ik[j])
    shift = math.dist(ik[_WRIST][12:15], rest[_WRIST][12:15])
    assert shift > 1.0, shift
    print("    half-blend IK move %.3f carries all %d joints"
          % (shift, len(joints)))

    # 4. The arm still solves (chain-internal, post-reorder smoke).
    _set_avar(stage, _INDEX_FK, "rz", 0.0)
    _set_avar(stage, _ARM_PARAMS, "ikfk", 0.0)
    _set_avar(stage, _ARM_IK, "tx", 0.0)
    _set_avar(stage, _SHOULDER_CTL, "rx", 15.0)
    _set_avar(stage, _SHOULDER_CTL, "rz", 15.0)
    _, swung = _frames()
    shift = math.dist(swung[_ELBOW][12:15], rest[_ELBOW][12:15])
    assert shift > 1.0, shift
    print("    shoulder bend swings the elbow %.3f" % shift)


def main():
    _setup_environment()
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)
    for path in _FILES:
        for mode in ("reference", "baked"):
            _check_file(path, mode)
    print("OK: biped hand follows the wrist")


if __name__ == "__main__":
    main()
