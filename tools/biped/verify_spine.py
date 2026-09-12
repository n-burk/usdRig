#!/usr/bin/env python
"""Behavioural gate for the spline-IK spine and neck.

The rest check in the builder only proves the offsets cancel the solver's
degree-2 residual. It says nothing about whether the controls DO anything,
and a spine that holds rest perfectly while ignoring its chest control
would sail through it. So this drives each control and asserts on the
result:

  * bend     -- translating the end control sideways moves the whole chain,
                graded from root (least) to tip (most). A rigid chain that
                just teleported the tip would fail the grading test.
  * mid      -- (spine only) the mid control bows the chain when its OWN
                avars are set; it sits at the midpoint of the root and end
                controls and aims at the end while they move; and while
                its avars are untouched it contributes NOTHING to the
                solve: its origin equals the solver's follow point,
                0.5 * rootMap(midRest) + 0.5 * endMap(midRest), so the
                mid offset the solver adds to cv1/cv2 is zero. It does not
                inherit the end control's roll.
  * neck     -- has no mid control (Maya's neck has none); the first
                neck joint aims at the head because cv1 rides a helper at
                the root aimed at the head (Maya's neck_head_aim).
  * squash   -- pulling the end control along the chain axis stretches it,
                and the volume weights must then SHRINK the joints' scale.
                Maya's profile peaks in the middle (spline.py 329-342), so
                the mid joint must thin more than the ends.
  * twist    -- rolling the end control has to reach the tip.
  * pivots   -- the pivot controls (hip_swivel / chest / head) sit 35% of
                the curve's arc length along the chain from their control;
                rotating one swings its control about that point; the
                pivot's `default:ty` dial moves the pivot WITHOUT moving
                the control, and takes time samples.
  * collapse -- driving the end control onto the root must not shrink the
                chain past the solver's length floor or fold it: every
                joint keeps aiming forward along the root's chain axis and
                the joints stay ordered along it. Skipped, loudly, when the
                loaded RigExec build predates `inputs:minLengthRatio`.
  * isolate  -- the neck controls must not move the spine at all, while the
                spine controls must carry the neck, since the neck hangs off
                chest_bind.

Usage: verify_spine.py <rig.usda> [--rig-root /Biped/Rig]
"""
import argparse
import math
import sys

from pxr import Gf, Sdf, Usd

import rigexec


def origin(pose, path):
    m = pose.joint_frame(path).to_matrix4()
    return Gf.Vec3d(m[12], m[13], m[14])


def scale_of(pose, path):
    m = Gf.Matrix4d(*pose.joint_frame(path).to_matrix4())
    # Row lengths of the 3x3 are the axis scales under the row-vector
    # convention this codebase uses throughout.
    return [Gf.Vec3d(m[r][0], m[r][1], m[r][2]).GetLength()
            for r in range(3)]


def row(m, r):
    return Gf.Vec3d(m[r][0], m[r][1], m[r][2])


class Spine(object):
    def __init__(self, path, rig_root):
        self.stage = Usd.Stage.Open(path)
        self.rig = rigexec.Rig(self.stage, rig_root)
        self.rig.compile()
        self.root = rig_root
        pose = self.rig.evaluate(0.0)
        self.joints = {str(p).rsplit("/", 1)[-1]: p
                       for p in pose.joint_paths()}
        # Controls by leaf name: the spine controls nest under their pivot
        # controls and the mid under its follow helper, so a flat
        # Controls/<name> path no longer resolves them.
        self.controls = {str(p).rsplit("/", 1)[-1]: str(p)
                         for p in pose.control_paths()}

    def has_control(self, ctl):
        return ctl in self.controls

    def control(self, ctl):
        path = self.controls.get(ctl)
        if path is None:
            raise SystemExit("no control %s (have %s)"
                             % (ctl, ", ".join(sorted(self.controls))))
        return self.stage.GetPrimAtPath(path)

    def set_attr(self, ctl, name, value, time=None):
        prim = self.control(ctl)
        attr = prim.GetAttribute(name)
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Double)
        if time is None:
            attr.Set(value)
        else:
            attr.Set(value, time)

    def avar(self, ctl, name, value):
        self.set_attr(ctl, "avars:%s" % name, value)

    def clear(self, *ctls):
        for ctl in ctls:
            for a in self.control(ctl).GetAttributes():
                if a.GetName().startswith("avars:"):
                    a.Clear()

    def pose(self, time=0.0):
        return self.rig.evaluate(time)

    def ctl_m(self, ctl, pose=None):
        pose = pose or self.pose()
        return Gf.Matrix4d(*pose.control_frame(self.controls[ctl])
                           .to_matrix4())

    def ctl_o(self, ctl, pose=None):
        return self.ctl_m(ctl, pose).ExtractTranslation()

    def ctl_rest(self, ctl):
        """A control's rest in asset space, composed up the namespace
        (rest:space is parent-local under a control ancestor)."""
        m = Gf.Matrix4d(1.0)
        prim = self.control(ctl)
        while prim and prim.IsValid():
            attr = prim.GetAttribute("rest:space")
            if attr and attr.IsValid() and attr.Get() is not None:
                m = m * Gf.Matrix4d(attr.Get())
            prim = prim.GetParent()
            if prim and prim.GetTypeName() != "RigExecControl":
                break
        return m

    def ctl_map(self, ctl, pose=None):
        """The control's rest->pose affine map (row-vector: p * map)."""
        return self.ctl_rest(ctl).GetInverse() * self.ctl_m(ctl, pose)

    def to_local(self, ctl, world_vec):
        """A world direction expressed in a control's avar axes.

        `avars:tx/ty/tz` are offsets in the control's OWN rest frame, not
        world -- and the spine controls inherit the spine joints'
        orientation, which is nothing like world axes. Pulling "up" by
        setting ty is therefore wrong; the pull has to be rotated into the
        control frame first. Getting this backwards made the first version
        of this test report the chain SHRINKING 34.4 -> 13.7 cm under what
        was meant to be a 30% stretch.
        """
        m = self.ctl_m(ctl)
        m.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
        m = m.GetOrthonormalized()
        return m.GetInverse().TransformDir(world_vec)

    def pull(self, ctl, world_vec):
        """Set a control's translation avars to a WORLD displacement."""
        v = self.to_local(ctl, world_vec)
        for comp, value in (("tx", v[0]), ("ty", v[1]), ("tz", v[2])):
            self.avar(ctl, comp, value)


def check(label, ok, detail):
    print("  %-38s %s  %s" % (label, "[ok]" if ok else "[FAIL]", detail))
    return 0 if ok else 1


def skip(label, detail):
    print("  %-38s [skip]  %s" % (label, detail))


def angle_between(a, b):
    c = Gf.Dot(a.GetNormalized(), b.GetNormalized())
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def verify(tag, joints, sp):
    print("")
    print("%s:" % tag)
    bad = 0
    rest = {}
    for j in joints:
        rest[j] = origin(sp.pose(), sp.joints[j])
    end_ctl = "%s_end_ctl" % tag
    root_ctl = "%s_root_ctl" % tag
    mid_ctl = "%s_mid_ctl" % tag

    # --- bend: end control sideways ---
    sp.avar(end_ctl, "tx", 15.0)
    pose = sp.pose()
    moved = [(origin(pose, sp.joints[j]) - rest[j]).GetLength()
             for j in joints]
    graded = all(moved[i] <= moved[i + 1] + 1e-6
                 for i in range(len(moved) - 1))
    bad += check("bend: chain follows end ctl",
                 moved[-1] > 5.0 and moved[1] > 0.05,
                 "root %.2f -> tip %.2f cm" % (moved[0], moved[-1]))
    bad += check("bend: graded root to tip", graded,
                 "[%s] cm" % ", ".join("%.2f" % m for m in moved))
    sp.clear(end_ctl)

    if sp.has_control(mid_ctl):
        # --- mid control bows the middle when ITS avars are set ---
        sp.avar(mid_ctl, "tz", 12.0)
        pose = sp.pose()
        mid = len(joints) // 2
        dmid = (origin(pose, sp.joints[joints[mid]])
                - rest[joints[mid]]).GetLength()
        dtip = (origin(pose, sp.joints[joints[-1]])
                - rest[joints[-1]]).GetLength()
        bad += check("mid ctl bows the chain", dmid > 1.0,
                     "mid %.2f cm, tip %.2f cm" % (dmid, dtip))
        sp.clear(mid_ctl)

        # --- mid follows the midpoint of root and end, translation only,
        #     and contributes nothing while untouched ---
        mid_rest = sp.ctl_rest(mid_ctl).ExtractTranslation()
        sp.avar(end_ctl, "tx", 15.0)
        sp.avar(end_ctl, "tz", 8.0)
        sp.avar(root_ctl, "rx", 20.0)
        pose = sp.pose()
        follow = (sp.ctl_map(root_ctl, pose).Transform(mid_rest) * 0.5 +
                  sp.ctl_map(end_ctl, pose).Transform(mid_rest) * 0.5)
        got = sp.ctl_o(mid_ctl, pose)
        midpoint = (sp.ctl_o(root_ctl, pose) + sp.ctl_o(end_ctl, pose)) * 0.5
        bad += check("mid sits on the solver's follow point",
                     (got - follow).GetLength() < 1e-6,
                     "off by %.3g cm (so the untouched mid adds 0 to "
                     "cv1/cv2); %.2f cm from the midpoint of the two "
                     "origins, since it carries its rest offset"
                     % ((got - follow).GetLength(),
                        (got - midpoint).GetLength()))
        # It may aim at the end control: the rest direction mid->end,
        # carried by the mid's rest->pose rotation, must point at the end
        # control's posed origin.
        v_rest = (sp.ctl_rest(end_ctl).ExtractTranslation() - mid_rest)
        r_mid = sp.ctl_map(mid_ctl, pose)
        r_mid.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
        v_now = r_mid.GetOrthonormalized().TransformDir(v_rest)
        to_end = sp.ctl_o(end_ctl, pose) - got
        aim_off = angle_between(v_now, to_end)
        bad += check("mid aims at the end control", aim_off < 1e-6,
                     "%.2g deg off the line to %s" % (aim_off, end_ctl))
        sp.clear(end_ctl, root_ctl)

        # --- mid does not inherit rotation: a 40 degree turn of the end
        #     or root control turns the mid only by what the aim needs to
        #     keep pointing at the end (a few degrees at most). Its
        #     POSITION is the carried midpoint, so it does shift a little
        #     as the rotated control carries its half of the follow point
        #     round -- Maya's pointOnCurve(0.5) follow moves the same way
        #     when cv2 orbits the chest. ---
        #     Precisely: the mid's rotation delta has NO roll about its
        #     own aim axis -- the only rotation it picks up is the swing
        #     that keeps it pointed at the end.
        local_aim = sp.to_local(mid_ctl, sp.ctl_o(end_ctl) - sp.ctl_o(mid_ctl))
        for who, comp in ((end_ctl, "rz"), (root_ctl, "rz"),
                          (end_ctl, "rx")):
            before = sp.ctl_m(mid_ctl)
            sp.avar(who, comp, 40.0)
            after = sp.ctl_m(mid_ctl)
            # Row-vector convention: after = d * before, so d is the delta
            # in the mid's OWN (local) axes, where the aim direction is
            # the constant `local_aim`.
            d = (after * before.GetInverse())
            d.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
            d = d.GetOrthonormalized()
            q = d.ExtractRotationQuat()
            if q.GetReal() < 0:
                q = -q
            roll = math.degrees(2.0 * math.atan2(
                Gf.Dot(q.GetImaginary(), local_aim.GetNormalized()),
                q.GetReal()))
            swing = abs(d.ExtractRotation().GetAngle())
            moved = (after.ExtractTranslation() -
                     before.ExtractTranslation()).GetLength()
            bad += check("mid does not inherit %s %s=40" % (who, comp),
                         abs(roll) < 1e-6,
                         "roll about its aim %.2g deg; swung %.2f deg to "
                         "keep aiming, carried %.2f cm" % (roll, swing, moved))
            sp.clear(who)
    else:
        bad += check("no mid control", not sp.has_control(mid_ctl),
                     "%s absent; interior CVs come from the root aim "
                     "helper and the end control" % mid_ctl)
        # cv1 rides a helper at the root aimed at the end, so the first
        # joint's bone must point at the head -- with the head moved 15 cm
        # sideways, well inside a few degrees, where a rigid cv1 (the
        # spine's model) leaves it pointing along the rest chain.
        sp.avar(end_ctl, "tz", 15.0)
        pose = sp.pose()
        m0 = Gf.Matrix4d(*pose.joint_frame(sp.joints[joints[0]]).to_matrix4())
        to_head = (origin(pose, sp.joints[joints[-1]]) -
                   origin(pose, sp.joints[joints[0]]))
        rest_axis = rest[joints[-1]] - rest[joints[0]]
        off = angle_between(row(m0, 0), to_head)
        swung = angle_between(rest_axis, to_head)
        bad += check("root joint aims at the moved head", off < 0.35 * swung,
                     "%s bone %.2f deg off the root->head line, which "
                     "swung %.2f deg" % (joints[0], off, swung))
        sp.clear(end_ctl)

    # --- squash: stretch along the chain axis thins the joints ---
    axis = rest[joints[-1]] - rest[joints[0]]
    length = axis.GetLength()
    axis = axis / length
    rest_scale = {}
    for j in joints:
        rest_scale[j] = scale_of(sp.pose(), sp.joints[j])
    sp.pull(end_ctl, axis * (length * 0.30))
    pose = sp.pose()
    stretched = (origin(pose, sp.joints[joints[-1]])
                 - origin(pose, sp.joints[joints[0]])).GetLength()
    ratios = []
    for j in joints:
        s0 = rest_scale[j]
        s1 = scale_of(pose, sp.joints[j])
        # The squash axes are the two perpendicular to the bone, so take
        # the mean of the two smallest and leave the bone axis out.
        r = sorted(s1[k] / s0[k] for k in range(3))[:2]
        ratios.append(sum(r) / 2.0)
    bad += check("squash: chain stretched", stretched > length * 1.15,
                 "%.1f -> %.1f cm (%.0f%%)"
                 % (length, stretched, 100.0 * stretched / length))
    bad += check("squash: joints thin under stretch",
                 all(r < 0.999 for r in ratios[1:-1]),
                 "[%s]" % ", ".join("%.3f" % r for r in ratios))
    bad += check("squash: middle thins most",
                 min(ratios) == min(ratios[1:-1]),
                 "thinnest at %s (%.3f)"
                 % (joints[ratios.index(min(ratios))], min(ratios)))
    sp.clear(end_ctl)

    # --- twist reaches the tip ---
    rest_m = Gf.Matrix4d(*sp.pose().joint_frame(sp.joints[joints[-1]])
                         .to_matrix4())
    sp.avar(end_ctl, "ry", 30.0)
    pose = sp.pose()
    got = Gf.Matrix4d(*pose.joint_frame(sp.joints[joints[-1]]).to_matrix4())
    delta = (got * rest_m.GetInverse()).GetOrthonormalized()
    ang = abs(delta.ExtractRotation().GetAngle())
    bad += check("twist reaches the tip", ang > 5.0,
                 "tip rolled %.2f deg" % ang)
    sp.clear(end_ctl)
    return bad


def verify_pivot(sp, pivot, ctl, far_joint, joints, watch):
    """One offset-pivot control: placement, swing, and the animatable dial.

    `watch` is the joint that must move when the pivot turns: the chain's
    root for the root pivot (cv0/cv1 ride the root control), the tip for
    an end pivot (cv2/cv3 and chest_bind/skull_bind ride the end control).
    """
    print("")
    print("%s (pivot of %s):" % (pivot, ctl))
    bad = 0
    if not sp.has_control(pivot):
        return check("pivot control exists", False, "no %s" % pivot)
    pprim = sp.control(pivot)
    p_rest = sp.ctl_rest(pivot).ExtractTranslation()
    c_rest = sp.ctl_rest(ctl).ExtractTranslation()
    far = sp.ctl_rest(far_joint).ExtractTranslation() \
        if sp.has_control(far_joint) else origin(sp.pose(),
                                                 sp.joints[far_joint])
    arc = pprim.GetAttribute("pivot:arcLength").Get()
    height = pprim.GetAttribute("pivot:height").Get()
    want = arc * height / 10.0
    dist = (p_rest - c_rest).GetLength()
    on_line = angle_between(p_rest - c_rest, far - c_rest)
    bad += check("pivot %.0f%% of arc along the chain" % (height * 10),
                 abs(dist - want) < 1e-6 and on_line < 1e-6,
                 "%.2f cm from %s toward %s (arc %.2f cm x %.1f/10), "
                 "%.2g deg off the line" % (dist, ctl, far_joint, arc,
                                            height, on_line))
    # The dial axis is the pivot's local +Y and it points along the chain.
    py = row(sp.ctl_rest(pivot), 1)
    bad += check("dial axis (+Y) points along the chain",
                 angle_between(py, far - c_rest) < 1e-6,
                 "%.2g deg" % angle_between(py, far - c_rest))

    # Rotating the pivot swings the control about the pivot point.
    sp.avar(pivot, "rz", 30.0)
    pose = sp.pose()
    c = sp.ctl_o(ctl, pose)
    p = sp.ctl_o(pivot, pose)
    chordlen = 2.0 * dist * math.sin(math.radians(15.0))
    bad += check("pivot rz=30 swings the control about it",
                 abs((c - p).GetLength() - dist) < 1e-6 and
                 abs((c - c_rest).GetLength() - chordlen) < 1e-6 and
                 (p - p_rest).GetLength() < 1e-9,
                 "control moved %.2f cm (expect %.2f), still %.2f cm from "
                 "the pivot, pivot itself moved %.2g"
                 % ((c - c_rest).GetLength(), chordlen,
                    (c - p).GetLength(), (p - p_rest).GetLength()))
    # And the chain went with it.
    watched = origin(pose, sp.joints[watch])
    sp.clear(pivot)
    watched_rest = origin(sp.pose(), sp.joints[watch])
    bad += check("...and the chain follows",
                 (watched - watched_rest).GetLength() > 1.0,
                 "%s moved %.2f cm" % (watch,
                                       (watched - watched_rest).GetLength()))

    # The dial: default:ty moves the pivot, not the control; then a
    # rotation happens about the moved pivot.
    dial = pprim.GetAttribute("default:ty")
    dial.Set(10.0)
    pose = sp.pose()
    c = sp.ctl_o(ctl, pose)
    p = sp.ctl_o(pivot, pose)
    bad += check("dial default:ty=10 moves only the pivot",
                 (c - c_rest).GetLength() < 1e-9 and
                 abs((p - p_rest).GetLength() - 10.0) < 1e-9 and
                 angle_between(p - p_rest, far - c_rest) < 1e-6,
                 "control moved %.2g cm, pivot %.2f cm along the chain"
                 % ((c - c_rest).GetLength(), (p - p_rest).GetLength()))
    sp.avar(pivot, "rz", 30.0)
    pose = sp.pose()
    c = sp.ctl_o(ctl, pose)
    p = sp.ctl_o(pivot, pose)
    bad += check("...and rz=30 then swings about the moved pivot",
                 abs((c - p).GetLength() - (dist + 10.0)) < 1e-6,
                 "control %.2f cm from the moved pivot (expect %.2f)"
                 % ((c - p).GetLength(), dist + 10.0))
    # Animatable: time samples on the dial.
    dial.Clear()
    dial.Set(0.0, 1.0)
    dial.Set(10.0, 2.0)
    d1 = (sp.ctl_o(ctl, sp.pose(1.0)) - sp.ctl_o(pivot, sp.pose(1.0))).GetLength()
    d2 = (sp.ctl_o(ctl, sp.pose(2.0)) - sp.ctl_o(pivot, sp.pose(2.0))).GetLength()
    bad += check("dial takes time samples",
                 abs(d1 - dist) < 1e-6 and abs(d2 - dist - 10.0) < 1e-6,
                 "pivot distance %.2f cm at t=1, %.2f cm at t=2" % (d1, d2))
    dial.Clear()
    dial.Set(0.0)
    sp.clear(pivot)
    return bad


def solver_has_floor(sp, tag):
    """Whether the loaded RigExec build knows inputs:minLengthRatio."""
    reg = Usd.SchemaRegistry()
    defn = reg.FindConcretePrimDefinition("RigExecSplineIk")
    return bool(defn) and "inputs:minLengthRatio" in defn.GetPropertyNames()


def verify_collapse(sp, tag, joints):
    print("")
    print("%s collapse:" % tag)
    solver = None
    for prim in sp.stage.Traverse():
        if (prim.GetTypeName() == "RigExecSplineIk" and
                prim.GetName().startswith(tag)):
            solver = prim
    attr = solver.GetAttribute("inputs:minLengthRatio") if solver else None
    floor = attr.Get() if attr and attr.IsValid() else None
    if not solver_has_floor(sp, tag):
        skip("length floor", "this RigExec build predates "
             "inputs:minLengthRatio (authored %s on %s, inert until the "
             "solver is rebuilt)" % (floor, solver.GetName() if solver else "?"))
        return 0
    if not floor:
        return check("length floor authored", False,
                     "inputs:minLengthRatio is %s on %s" % (floor, solver))
    bad = 0
    end_ctl = "%s_end_ctl" % tag
    root_ctl = "%s_root_ctl" % tag
    rest = [origin(sp.pose(), sp.joints[j]) for j in joints]
    chord = rest[-1] - rest[0]
    length = chord.GetLength()
    axis = chord / length
    for fraction in (0.6, 0.9, 1.0, 1.2):
        # Push the end control straight down the chain toward (and past)
        # the root.
        sp.pull(end_ctl, axis * (-fraction * length))
        pose = sp.pose()
        pos = [origin(pose, sp.joints[j]) for j in joints]
        along = [Gf.Dot(p - pos[0], axis) for p in pos]
        ordered = all(along[i] < along[i + 1] for i in range(len(along) - 1))
        forward = all(Gf.Dot(row(Gf.Matrix4d(*pose.joint_frame(
            sp.joints[j]).to_matrix4()), 0), axis) > 0.0 for j in joints)
        span = along[-1] - along[0]
        bad += check("end pushed %.0f%% onto the root" % (fraction * 100),
                     ordered and forward and span >= floor * length - 1e-6,
                     "root->tip along the axis %.2f cm = %.2f of rest "
                     "(floor %.2f); ordered %s, all bones forward %s"
                     % (span, span / length, floor, ordered, forward))
        sp.clear(end_ctl)
    # The floor is a per-shot dial: off, the chain does crumple.
    attr.Set(0.0)
    sp.pull(end_ctl, axis * (-1.0 * length))
    pose = sp.pose()
    pos = [origin(pose, sp.joints[j]) for j in joints]
    span = Gf.Dot(pos[-1] - pos[0], axis)
    bad += check("floor 0 lets it crumple (dial works)",
                 span < 0.25 * length,
                 "root->tip %.2f of rest with the floor off" % (span / length))
    attr.Set(floor)
    sp.clear(end_ctl)
    return bad


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("rig")
    ap.add_argument("--rig-root", default="/Biped/Rig")
    args = ap.parse_args(argv)

    rigexec.load_schema_plugin()
    sp = Spine(args.rig, args.rig_root)

    spine = ["spine_0_bind", "spine_1_bind", "spine_2_bind", "spine_3_bind",
             "spine_4_bind", "spine_5_bind", "chest_bind"]
    neck = ["neck_0_bind", "neck_1_bind", "neck_2_bind", "neck_3_bind",
            "skull_bind"]

    bad = verify("spine", spine, sp)
    bad += verify("neck", neck, sp)

    bad += verify_pivot(sp, "spine_root_pivot", "spine_root_ctl",
                        "spine_end_ctl", spine, spine[0])
    bad += verify_pivot(sp, "spine_end_pivot", "spine_end_ctl",
                        "spine_root_ctl", spine, spine[-1])
    bad += verify_pivot(sp, "neck_end_pivot", "neck_end_ctl",
                        "neck_root_ctl", neck, neck[-1])

    bad += verify_collapse(sp, "spine", spine)
    bad += verify_collapse(sp, "neck", neck)

    print("")
    print("isolation:")
    rest_spine = {}
    for j in spine:
        rest_spine[j] = origin(sp.pose(), sp.joints[j])
    sp.avar("neck_end_ctl", "tx", 15.0)
    pose = sp.pose()
    worst = max((origin(pose, sp.joints[j]) - rest_spine[j]).GetLength()
                for j in spine)
    bad += check("neck ctl leaves the spine alone", worst < 1e-6,
                 "worst %.3g cm" % worst)
    sp.clear("neck_end_ctl")

    rest_neck = {}
    for j in neck:
        rest_neck[j] = origin(sp.pose(), sp.joints[j])
    sp.avar("spine_end_ctl", "tx", 15.0)
    pose = sp.pose()
    carried = min((origin(pose, sp.joints[j]) - rest_neck[j]).GetLength()
                  for j in neck)
    bad += check("spine ctl carries the neck along", carried > 5.0,
                 "least-moved neck joint %.2f cm" % carried)
    sp.clear("spine_end_ctl")

    print("")
    print("all spine checks passed" if not bad
          else "%d CHECK(S) FAILED" % bad)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
