#!/usr/bin/env python
"""Behavioural gate for the girdles (girdle.py) and the spine's twist.

Drives controls and measures joints; the rest pose alone proves nothing
about a rig that ignores its controls. Every number printed is measured
from an evaluated pose.

  * clavicle -- each clavicle control moves its joint by exactly the
    control's own displacement; moving AND rotating spine_end_ctl carries
    the clavicle controls and joints together (control origin == joint
    origin after the move).
  * pelvis   -- the same for pelvis_?_ctl under spine_root_ctl, and
    hips_bind rides spine_root_ctl (parent + scale).
  * twist    -- rolling spine_root_ctl about the spine axis produces a
    GRADED twist up the chain (full at spine_0, zero at the chest);
    rolling spine_end_ctl the mirror gradient; both together a constant
    roll. Reported per joint.
  * rest     -- with every avar cleared, all joints sit on Maya's
    world_translate to 1e-3 cm.
  * isolate  -- the end control leaves the pelvis alone, the pelvis
    controls leave the clavicles alone.

Usage: verify_girdle.py <rig.usda> [--rig-root /Biped/Rig]
"""
import argparse
import math
import os
import sys

from pxr import Gf, Sdf, Usd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_biped_skel import DATA, load_joints  # noqa: E402

import rigexec  # noqa: E402

SPINE = ["spine_0_bind", "spine_1_bind", "spine_2_bind", "spine_3_bind",
         "spine_4_bind", "spine_5_bind", "chest_bind"]


class Rig(object):
    def __init__(self, path, rig_root):
        self.stage = Usd.Stage.Open(path)
        self.rig = rigexec.Rig(self.stage, rig_root)
        self.rig.compile()
        self.root = rig_root
        self.joints = {str(p).rsplit("/", 1)[-1]: p
                       for p in self.rig.evaluate(0.0).joint_paths()}
        self.controls = {str(p).rsplit("/", 1)[-1]: p
                         for p in self.rig.evaluate(0.0).control_paths()}

    def control_prim(self, ctl):
        path = self.controls.get(ctl)
        if path is None:
            raise SystemExit("no control %s (have %s)"
                             % (ctl, ", ".join(sorted(self.controls))))
        return self.stage.GetPrimAtPath(path)

    def avar(self, ctl, name, value):
        prim = self.control_prim(ctl)
        attr = prim.GetAttribute("avars:%s" % name)
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("avars:%s" % name,
                                        Sdf.ValueTypeNames.Double)
        attr.Set(value)

    def clear(self, *ctls):
        for ctl in ctls:
            for a in self.control_prim(ctl).GetAttributes():
                if a.GetName().startswith("avars:"):
                    a.Clear()

    def pose(self):
        return self.rig.evaluate(0.0)

    def joint_m(self, pose, j):
        return Gf.Matrix4d(*pose.joint_frame(self.joints[j]).to_matrix4())

    def joint_o(self, pose, j):
        return self.joint_m(pose, j).ExtractTranslation()

    def ctl_m(self, pose, c):
        return Gf.Matrix4d(*pose.control_frame(self.controls[c])
                           .to_matrix4())

    def ctl_o(self, pose, c):
        return self.ctl_m(pose, c).ExtractTranslation()


def twist_about_x(posed, rest):
    """Twist of the rest->pose rotation about the joint's own rest +X.

    `posed = D_local * rest` (row-vector), so D_local = posed * rest^-1 is
    the delta in the rest frame's own axes; its swing-twist split about
    (1,0,0) is 2*atan2(q.x, q.w), sign-corrected to the w >= 0 cover. The
    spine joints aim +X at their child, so this is the roll about the
    bone.
    """
    d = posed * rest.GetInverse()
    d.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
    d = d.GetOrthonormalized()
    q = d.ExtractRotationQuat()
    w, v = q.GetReal(), q.GetImaginary()
    if w < 0:
        w, v = -w, -v
    return math.degrees(2.0 * math.atan2(v[0], w))


def check(label, ok, detail):
    print("  %-40s %s  %s" % (label, "[ok]" if ok else "[FAIL]", detail))
    return 0 if ok else 1


def near(a, b, tol):
    return (a - b).GetLength() <= tol


def verify_girdle(r, tag, controls, parent_ctl, parent_joint, isolate_from):
    """One girdle: `controls` is [(ctl, joint), ...] nested under parent_ctl."""
    print("")
    print("%s (under %s):" % (tag, parent_ctl))
    bad = 0
    rest = r.pose()
    rest_o = {j: r.joint_o(rest, j) for _, j in controls}

    # --- each control moves its own joint by exactly its own displacement
    for ctl, joint in controls:
        r.avar(ctl, "tx", 4.0)
        r.avar(ctl, "ty", -3.0)
        r.avar(ctl, "tz", 2.0)
        p = r.pose()
        dj = r.joint_o(p, joint) - rest_o[joint]
        dc = r.ctl_o(p, ctl) - r.ctl_o(rest, ctl)
        bad += check("%s moves %s" % (ctl, joint),
                     dj.GetLength() > 5.0 and near(dj, dc, 1e-6),
                     "joint %.3f cm, control %.3f cm, gap %.2g"
                     % (dj.GetLength(), dc.GetLength(),
                        (dj - dc).GetLength()))
        # the sibling girdle joint must not have moved
        for octl, ojoint in controls:
            if ojoint != joint:
                dd = (r.joint_o(p, ojoint) - rest_o[ojoint]).GetLength()
                bad += check("  ...and leaves %s alone" % ojoint,
                             dd < 1e-6, "%.2g cm" % dd)
        r.clear(ctl)

    # --- the parent control carries controls AND joints: translate
    r.avar(parent_ctl, "tx", 6.0)
    r.avar(parent_ctl, "tz", -4.0)
    p = r.pose()
    dp = r.ctl_o(p, parent_ctl) - r.ctl_o(rest, parent_ctl)
    for ctl, joint in controls:
        dj = r.joint_o(p, joint) - rest_o[joint]
        dc = r.ctl_o(p, ctl) - r.ctl_o(rest, ctl)
        bad += check("%s translate carries %s + joint" % (parent_ctl, ctl),
                     near(dj, dp, 1e-6) and near(dc, dp, 1e-6),
                     "parent %.2f, control %.2f, joint %.2f cm"
                     % (dp.GetLength(), dc.GetLength(), dj.GetLength()))
    if parent_joint:
        dj = r.joint_o(p, parent_joint) - r.joint_o(rest, parent_joint)
        bad += check("%s translate carries %s" % (parent_ctl, parent_joint),
                     near(dj, dp, 1e-6), "%.2f cm" % dj.GetLength())
    r.clear(parent_ctl)

    # --- the parent control carries them: rotate (a swing, not a roll)
    r.avar(parent_ctl, "ry", 25.0)
    p = r.pose()
    for ctl, joint in controls:
        dj = (r.joint_o(p, joint) - rest_o[joint]).GetLength()
        gap = (r.joint_o(p, joint) - r.ctl_o(p, ctl)).GetLength()
        bad += check("%s rotate swings %s with its control" % (parent_ctl,
                                                              joint),
                     dj > 0.5 and gap < 1e-6,
                     "joint moved %.2f cm, control-joint gap %.2g cm"
                     % (dj, gap))
    if parent_joint:
        dm = r.joint_m(p, parent_joint) * r.joint_m(
            rest, parent_joint).GetInverse()
        dm.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
        ang = abs(dm.GetOrthonormalized().ExtractRotation().GetAngle())
        bad += check("%s rotate turns %s" % (parent_ctl, parent_joint),
                     abs(ang - 25.0) < 1e-3, "%.3f deg" % ang)
    r.clear(parent_ctl)

    # --- isolation from the other girdle's driver
    r.avar(isolate_from, "tx", 10.0)
    r.avar(isolate_from, "ry", 20.0)
    p = r.pose()
    worst = max((r.joint_o(p, j) - rest_o[j]).GetLength()
                for _, j in controls)
    bad += check("%s leaves the %s alone" % (isolate_from, tag),
                 worst < 1e-6, "worst %.2g cm" % worst)
    r.clear(isolate_from)
    return bad


def verify_twist(r):
    print("")
    print("spine twist (roll about each joint's own bone axis, degrees):")
    bad = 0
    rest = r.pose()
    rest_m = {j: r.joint_m(rest, j) for j in SPINE + ["hips_bind"]}

    def profile():
        p = r.pose()
        return [twist_about_x(r.joint_m(p, j), rest_m[j]) for j in SPINE]

    def fmt(prof):
        return "[%s]" % ", ".join("%.1f" % t for t in prof)

    # hip swivel rolls: full at the root, fading to nothing at the chest
    r.avar("spine_root_ctl", "rx", 30.0)
    root = profile()
    hips = twist_about_x(r.joint_m(r.pose(), "hips_bind"),
                         rest_m["hips_bind"])
    r.clear("spine_root_ctl")
    graded = all(root[i] >= root[i + 1] - 0.5 for i in range(len(root) - 1))
    bad += check("spine_root_ctl rx=30: graded twist root->chest",
                 abs(root[0] - 30.0) < 1.0 and abs(root[-1]) < 1.0
                 and graded and (root[0] - root[3]) > 10.0,
                 fmt(root))
    bad += check("  ...and hips_bind rolls with the swivel",
                 abs(hips - 30.0) < 1.0, "%.2f deg" % hips)

    # chest rolls: nothing at the root, full at the chest
    r.avar("spine_end_ctl", "rx", 30.0)
    end = profile()
    r.clear("spine_end_ctl")
    graded = all(end[i] <= end[i + 1] + 0.5 for i in range(len(end) - 1))
    bad += check("spine_end_ctl rx=30: graded twist chest->root",
                 abs(end[0]) < 1.0 and abs(end[-1] - 30.0) < 1.0
                 and graded and (end[-1] - end[3]) > 10.0,
                 fmt(end))

    # both: a constant roll along the whole chain
    r.avar("spine_root_ctl", "rx", 30.0)
    r.avar("spine_end_ctl", "rx", 30.0)
    both = profile()
    r.clear("spine_root_ctl", "spine_end_ctl")
    bad += check("both rx=30: constant roll",
                 all(abs(t - 30.0) < 1.0 for t in both), fmt(both))

    # opposite: a 60-degree wind-up across the chain
    r.avar("spine_root_ctl", "rx", -30.0)
    r.avar("spine_end_ctl", "rx", 30.0)
    opp = profile()
    r.clear("spine_root_ctl", "spine_end_ctl")
    bad += check("root -30 / end +30: -30 -> +30 wind-up",
                 abs(opp[0] + 30.0) < 1.0 and abs(opp[-1] - 30.0) < 1.0,
                 fmt(opp))
    return bad


def verify_rest(r):
    print("")
    print("rest pose after every avar is cleared:")
    data, _parent = load_joints(os.path.join(DATA, "joint_positions.data"))
    p = r.pose()
    worst, at, n = 0.0, None, 0
    for name, jp in r.joints.items():
        want = data.get(name, {}).get("world_translate")
        if not want:
            continue
        got = r.joint_o(p, name)
        d = max(abs(got[i] - want[i]) for i in range(3))
        n += 1
        if d > worst:
            worst, at = d, name
    return check("all joints on Maya's world_translate", worst < 1e-3,
                 "%d joints, worst %.3g cm (%s)" % (n, worst, at))


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("rig")
    ap.add_argument("--rig-root", default="/Biped/Rig")
    args = ap.parse_args(argv)

    rigexec.load_schema_plugin()
    r = Rig(args.rig, args.rig_root)

    bad = 0
    bad += verify_girdle(
        r, "clavicles",
        [("clavicle_l_ctl", "clavicle_l_bind"),
         ("clavicle_r_ctl", "clavicle_r_bind")],
        "spine_end_ctl", "chest_bind", isolate_from="spine_root_ctl")
    bad += verify_girdle(
        r, "pelvis",
        [("pelvis_l_ctl", "pelvis_l_bind"),
         ("pelvis_r_ctl", "pelvis_r_bind")],
        "spine_root_ctl", "hips_bind", isolate_from="spine_end_ctl")
    bad += verify_twist(r)
    bad += verify_rest(r)

    # Informational: what the limb roots do under the girdle. The limb
    # joints are solver-owned, so they follow their own root controls, not
    # the pelvis/clavicle joints -- reported, not asserted.
    print("")
    print("limb roots under the girdles (informational):")
    rest = r.pose()
    for ctl, joint, limb in (("pelvis_l_ctl", "pelvis_l_bind",
                              "thigh_l_bind"),
                             ("clavicle_l_ctl", "clavicle_l_bind",
                              "shoulder_l_bind")):
        if limb not in r.joints:
            continue
        r.avar(ctl, "tx", 5.0)
        p = r.pose()
        dj = (r.joint_o(p, joint) - r.joint_o(rest, joint)).GetLength()
        dl = (r.joint_o(p, limb) - r.joint_o(rest, limb)).GetLength()
        r.clear(ctl)
        print("  %s +5: %s moved %.2f cm, %s moved %.2f cm"
              % (ctl, joint, dj, limb, dl))

    print("")
    print("all girdle checks passed" if not bad
          else "%d CHECK(S) FAILED" % bad)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
