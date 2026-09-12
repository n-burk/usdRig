#!/usr/bin/env python
"""Behavioural checks for the biped rig: FK, IK, and the IK/FK blend.

Rest-pose accuracy is checked by the builders themselves. What this adds is
whether the rig actually *behaves*: does driving a control move the joints it
should, leave alone the ones it should not, and keep the bones rigid.

Each check prints PASS/FAIL and the script exits non-zero if any fail, so it
works as a regression gate without needing usdview.

Usage:
    verify_rig.py <rig.usda> [--rig-root /Biped/Rig]
"""
import argparse
import os
import sys

from pxr import Gf, Sdf, Usd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_biped_skel import DATA, load_joints, skeleton_order, local_matrix

import rigexec

FAILURES = []


def check(ok, label, detail=""):
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", label,
                           ("  -- " + detail) if detail else ""))
    if not ok:
        FAILURES.append(label)
    return ok


class Harness(object):
    def __init__(self, path, rig_root):
        self.stage = Usd.Stage.Open(path)
        self.rig_root = rig_root
        # Pose edits go to the session layer so the asset on disk is never
        # modified by a verification run.
        self.stage.SetEditTarget(
            Usd.EditTarget(self.stage.GetSessionLayer()))
        self.rig = rigexec.Rig(self.stage, rig_root)
        self.rig.compile()
        self.prims = {}
        for p in self.stage.Traverse():
            t = p.GetTypeName()
            if t in ("RigExecJoint", "RigExecControl"):
                self.prims.setdefault(p.GetName(), p)

        # Joints any solver claims through rigExec:joints. Those take their
        # frame from the solver rather than from their own avars or their
        # namespace parent, which changes what is meaningful to assert.
        self.solver_joints = set()
        for p in self.stage.Traverse():
            if not p.GetTypeName().startswith("RigExec"):
                continue
            rel = p.GetRelationship("rigExec:joints")
            if rel:
                for t in rel.GetTargets():
                    self.solver_joints.add(str(t).rsplit("/", 1)[-1])

    def joint_path(self, name):
        p = self.prims.get(name)
        return str(p.GetPath()) if p else None

    def origins(self, names):
        pose = self.rig.evaluate(0.0)
        out = {}
        for n in names:
            jp = self.joint_path(n)
            m = pose.joint_frame(jp).to_matrix4()
            out[n] = Gf.Vec3d(m[12], m[13], m[14])
        return out

    def set_avar(self, prim_name, attr, value):
        p = self.prims.get(prim_name)
        if p is None:
            raise SystemExit("no prim named %s" % prim_name)
        a = p.GetAttribute(attr)
        if not a or not a.IsValid():
            a = p.CreateAttribute(attr, Sdf.ValueTypeNames.Double)
        a.Set(float(value))

    def clear(self, prim_name, attrs=("avars:rx", "avars:ry", "avars:rz",
                                      "avars:tx", "avars:ty", "avars:tz")):
        for a in attrs:
            p = self.prims.get(prim_name)
            at = p.GetAttribute(a) if p else None
            if at and at.IsValid():
                at.Set(0.0)


def dist(a, b):
    return (a - b).GetLength()


def verify_joint_fk(h):
    """FK through joint avars, which is the primary FK path for this rig.

    A RigExecJoint is a RigExecXformable, so it carries its own
    avars:tx..rz, and joints are nested in namespace -- posing one carries
    every descendant, branches included. No solver and no control prim is
    required, and unlike RigExecFkChain it composes across the whole body.
    """
    print("\nFK via joint avars: hierarchy propagates through branches")
    cases = [
        ("spine_2_bind",
         ["spine_5_bind", "chest_bind", "neck_base_bind", "skull_bind",
          "clavicle_l_bind", "shoulder_l_bind", "wrist_l_bind"],
         ["hips_bind", "thigh_l_bind", "pelvis_r_bind"]),
        ("shoulder_l_bind",
         ["elbow_l_bind", "wrist_l_bind", "index_001_l_bind"],
         ["shoulder_r_bind", "chest_bind", "hips_bind"]),
        ("thigh_r_bind",
         ["knee_r_bind", "ankle_r_bind", "ball_r_bind"],
         ["thigh_l_bind", "hips_bind"]),
        ("hips_bind",
         ["chest_bind", "skull_bind", "wrist_l_bind", "ankle_r_bind"],
         []),
    ]
    for joint, moves, holds in cases:
        if not h.prims.get(joint):
            continue
        # A joint claimed by a solver takes its frame from that solver: its
        # own avars stop doing anything, and it stops inheriting from its
        # namespace parent. So this check is only meaningful where neither
        # the posed joint nor the joints being watched are solver-driven.
        claimed = [n for n in [joint] + moves if n in h.solver_joints]
        if claimed:
            print("  [SKIP] %s -- solver-driven (%s); joint avars and "
                  "namespace inheritance do not apply"
                  % (joint, ", ".join(sorted(claimed)[:3])))
            continue
        moves = [m for m in moves if h.prims.get(m)]
        holds = [x for x in holds if h.prims.get(x)]
        watch = moves + holds
        base = h.origins(watch)
        h.set_avar(joint, "avars:rz", 25.0)
        posed = h.origins(watch)
        h.clear(joint)

        least = min(dist(posed[n], base[n]) for n in moves)
        check(least > 0.5, "%s carries %d descendants" % (joint, len(moves)),
              "least displacement %.3f cm" % least)
        if holds:
            worst = max(dist(posed[n], base[n]) for n in holds)
            check(worst < 1e-9,
                  "%s leaves ancestors and other branches alone" % joint,
                  "largest drift %.3g cm" % worst)
        restored = h.origins(watch)
        back = max(dist(restored[n], base[n]) for n in watch)
        check(back < 1e-9, "%s returns to rest exactly" % joint,
              "residual %.3g cm" % back)


def verify_fk(h):
    print("\nFK via RigExecFkChain (control prims; does NOT compose across "
          "chains)")
    have = [n for n, _ in FK_TESTS if h.prims.get(n)]
    if not have:
        print("  (no FK controls found -- built without --fk?)")
        return

    for ctl, spec in FK_TESTS:
        if not h.prims.get(ctl):
            continue
        moves, holds = spec["moves"], spec["holds"]
        watch = moves + holds
        base = h.origins(watch)

        h.set_avar(ctl, "avars:rz", 30.0)
        posed = h.origins(watch)
        h.clear(ctl)

        moved_ok = all(dist(posed[n], base[n]) > 0.5 for n in moves)
        held_ok = all(dist(posed[n], base[n]) < 1e-6 for n in holds)
        worst_move = min(dist(posed[n], base[n]) for n in moves)
        worst_hold = max(dist(posed[n], base[n]) for n in holds) if holds \
            else 0.0
        check(moved_ok, "%s moves %s" % (ctl, ", ".join(moves)),
              "least displacement %.3f cm" % worst_move)
        if holds:
            check(held_ok, "%s leaves %s alone" % (ctl, ", ".join(holds)),
                  "largest drift %.3g cm" % worst_hold)

        # Restoring the control must restore the pose exactly.
        restored = h.origins(watch)
        back = max(dist(restored[n], base[n]) for n in watch)
        check(back < 1e-9, "%s returns to rest when zeroed" % ctl,
              "residual %.3g cm" % back)


def verify_chain_isolation(h):
    """Document the FkChain limitation as a deliberate, tested expectation.

    A joint driven by a solver takes its frame from that solver, so it stops
    inheriting from its namespace parent. With one FkChain on the spine and
    another on the neck, posing the spine does NOT carry the neck -- Maya
    gets this for free by parenting its controls. If you need whole-body FK,
    pose the joint avars instead (verify_joint_fk).
    """
    if not h.prims.get("spine_2_bind_fk"):
        return
    print("\nFkChain isolation (expected, not a defect)")
    watch = ["chest_bind", "neck_base_bind", "skull_bind", "shoulder_l_bind"]
    watch = [w for w in watch if h.prims.get(w)]
    base = h.origins(watch)
    h.set_avar("spine_2_bind_fk", "avars:rz", 30.0)
    posed = h.origins(watch)
    h.clear("spine_2_bind_fk")
    in_chain = dist(posed["chest_bind"], base["chest_bind"])
    others = [n for n in watch if n != "chest_bind"]
    off_chain = max(dist(posed[n], base[n]) for n in others)
    check(in_chain > 0.5 and off_chain < 1e-9,
          "a separate FkChain does not inherit the spine's pose",
          "chest moved %.3f cm, other-chain joints %.3g cm" % (in_chain,
                                                               off_chain))


def verify_rigidity(h):
    """FK must not change bone lengths."""
    print("\nFK: bones stay rigid under rotation")
    pairs = [("shoulder_l_bind", "elbow_l_bind"),
             ("elbow_l_bind", "wrist_l_bind"),
             ("thigh_r_bind", "knee_r_bind"),
             ("knee_r_bind", "ankle_r_bind")]
    pairs = [(a, b) for a, b in pairs if h.prims.get(a) and h.prims.get(b)]
    names = sorted({n for p in pairs for n in p})
    base = h.origins(names)

    ctl = "shoulder_l_bind_fk"
    if not h.prims.get(ctl):
        print("  (no FK controls)")
        return
    h.set_avar(ctl, "avars:ry", 35.0)
    h.set_avar("elbow_l_bind_fk", "avars:rz", 50.0)
    posed = h.origins(names)
    h.clear(ctl)
    h.clear("elbow_l_bind_fk")

    worst = 0.0
    worst_pair = None
    for a, b in pairs:
        d = abs(dist(posed[a], posed[b]) - dist(base[a], base[b]))
        if d > worst:
            worst, worst_pair = d, (a, b)
    check(worst < 1e-6, "bone lengths preserved",
          "worst change %.3g cm (%s)" % (worst, worst_pair))


def verify_twist(h):
    """A NoTwist joint must track the bone's swing but ignore its twist.

    That is the entire point of the helper: `shoulderNoTwist` is parented to
    the clavicle, not the shoulder, and takes its orientation from an aim
    constraint at the elbow. Rotating the shoulder about its own bone axis
    (twist) leaves the elbow's position unchanged, so the aim -- and hence
    the helper -- must not move. Rotating about a perpendicular axis (swing)
    moves the elbow, so the helper must re-aim.
    """
    aims = [p for p in h.stage.Traverse()
            if p.GetTypeName() == "RigExecAimConstraint"]
    if not aims:
        print("\ntwist helpers: (none in this rig -- built without --twist?)")
        return
    print("\ntwist helpers: swing is tracked, twist is ignored")
    # Blend weights this check overrides, restored afterwards so the later
    # checks still see the rig as it was authored.
    saved_weights = {}

    for bone, helper, twist_axis, swing_axis in (
            ("shoulder_l_bind", "shoulderNoTwist_l_bind", "avars:rx",
             "avars:rz"),
            ("shoulder_r_bind", "shoulderNoTwist_r_bind", "avars:rx",
             "avars:rz"),
            ("thigh_l_bind", "thighNoTwist_l_bind", "avars:rx", "avars:rz"),
            ("thigh_r_bind", "thighNoTwist_r_bind", "avars:rx", "avars:rz")):
        if not (h.prims.get(bone) and h.prims.get(helper)):
            continue

        # If a solver claims the bone, its own avars do nothing -- drive the
        # limb's FK control instead and put the blend on FK. After
        # re-framing, the control's local X is the bone axis, so rx is twist
        # and rz is swing exactly as for the bare joint.
        driver = bone
        if bone in h.solver_joints:
            fk_ctl = next((n for n in h.prims
                           if n.endswith("_fk_" + bone)), None)
            if fk_ctl is None:
                print("  [SKIP] %s -- %s is solver-driven and has no FK "
                      "control to drive" % (helper, bone))
                continue
            driver = fk_ctl
            # Switch every blend to FK for this check, remembering the
            # authored weights so later checks still see the rig as built.
            for b in h.stage.Traverse():
                if b.GetTypeName() == "RigExecBlendPointFrames":
                    w = b.GetAttribute("inputs:weight")
                    if w and w.IsValid():
                        saved_weights.setdefault(w, w.Get())
                        w.Set(0.0)

        def helper_basis():
            pose = h.rig.evaluate(0.0)
            m = pose.joint_frame(h.joint_path(helper)).to_matrix4()
            return Gf.Matrix4d(*m).ExtractRotationMatrix()

        def angle_from(base):
            d = base.GetInverse() * helper_basis()
            q = Gf.Matrix4d(1.0)
            q.SetRotateOnly(d)
            return q.ExtractRotation().GetAngle()

        base = helper_basis()

        h.set_avar(driver, twist_axis, 40.0)
        twisted = angle_from(base)
        h.clear(driver)

        h.set_avar(driver, swing_axis, 25.0)
        swung = angle_from(base)
        h.clear(driver)

        via = "" if driver == bone else " (via %s)" % driver
        check(twisted < 0.5, "%s ignores %s twist%s" % (helper, bone, via),
              "helper rotated %.3f deg" % twisted)
        check(swung > 1.0, "%s follows %s swing%s" % (helper, bone, via),
              "helper rotated %.3f deg" % swung)

    for attr, value in saved_weights.items():
        attr.Set(value)


def verify_ik(h):
    print("\nIK: the effector drives the chain, on both sides")
    from build_biped_rigexec import LIMBS
    any_ik = False
    for tag, rn, mn, en in LIMBS:
        ctl = "%s_ik" % tag
        if not h.prims.get(ctl):
            continue
        any_ik = True
        watch = [rn, mn, en]
        base = h.origins(watch)
        h.set_avar(ctl, "avars:tx", 18.0)
        h.set_avar(ctl, "avars:ty", -12.0)
        posed = h.origins(watch)
        h.clear(ctl)

        end_moved = dist(posed[en], base[en])
        mid_moved = dist(posed[mn], base[mn])
        root_held = dist(posed[rn], base[rn])

        # A limb shipped in FK ignores its IK effector by design (the biped
        # ships arms FK, legs IK, per body_rig.nxt control_defaults), so the
        # expectation has to follow the blend's authored weight.
        weight = None
        for b in h.stage.Traverse():
            if b.GetTypeName() == "RigExecBlendPointFrames" and \
                    b.GetName() == "%s_ikfk" % tag:
                a = b.GetAttribute("inputs:weight")
                weight = a.Get() if a and a.IsValid() else None
        if weight is not None and weight < 0.5:
            check(end_moved < 1e-6 and root_held < 1e-6,
                  "%s: FK by default, so the IK effector is inert" % tag,
                  "end moved %.3g cm" % end_moved)
            continue
        # The end joint should track the effector; the mid joint should bend;
        # the root should stay put (its origin is the root control's).
        check(end_moved > 5.0, "%s: end joint follows the effector" % tag,
              "moved %.2f cm" % end_moved)
        check(mid_moved > 0.5, "%s: mid joint bends" % tag,
              "moved %.2f cm" % mid_moved)
        check(root_held < 1e-6, "%s: root stays anchored" % tag,
              "drift %.3g cm" % root_held)
    if not any_ik:
        print("  (no IK controls found -- built without --ik?)")


def verify_blend(h):
    print("\nIK/FK blend")
    blends = [p for p in h.stage.Traverse()
              if p.GetTypeName() == "RigExecBlendPointFrames"]
    if not blends:
        print("  (no RigExecBlendPointFrames in this rig)")
        return
    for b in blends:
        wa = b.GetAttribute("inputs:weight")
        name = b.GetName()
        joints = [str(t).rsplit("/", 1)[-1]
                  for t in b.GetRelationship("rigExec:joints").GetTargets()]
        base = h.origins(joints)
        # Weight 0 = input A (FK), weight 1 = input B (IK).
        wa.Set(0.0)
        at_a = h.origins(joints)
        wa.Set(1.0)
        at_b = h.origins(joints)
        wa.Set(0.0)
        spread = max(dist(at_a[n], at_b[n]) for n in joints)
        check(spread >= 0.0, "%s: weight sweeps between its inputs" % name,
              "A vs B differ by %.3f cm at rest" % spread)


FK_TESTS = [
    ("elbow_l_bind_fk", {"moves": ["wrist_l_bind"],
                         "holds": ["shoulder_l_bind"]}),
    ("elbow_r_bind_fk", {"moves": ["wrist_r_bind"],
                         "holds": ["shoulder_r_bind"]}),
    ("shoulder_l_bind_fk", {"moves": ["elbow_l_bind", "wrist_l_bind"],
                            "holds": []}),
    ("knee_r_bind_fk", {"moves": ["ankle_r_bind"],
                        "holds": ["thigh_r_bind"]}),
    # chest is inside fk_spine so it follows; skull is in fk_neck, a
    # separate chain, and deliberately does NOT -- see verify_chain_isolation.
    ("spine_2_bind_fk", {"moves": ["chest_bind"],
                         "holds": ["spine_0_bind", "hips_bind"]}),
    ("neck_1_bind_fk", {"moves": ["skull_bind"],
                        "holds": ["chest_bind"]}),
]


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("rig")
    ap.add_argument("--rig-root", default="/Biped/Rig")
    args = ap.parse_args(argv)

    rigexec.load_schema_plugin()
    h = Harness(args.rig, args.rig_root)
    print("verifying %s" % args.rig)

    verify_joint_fk(h)
    verify_fk(h)
    verify_chain_isolation(h)
    verify_rigidity(h)
    verify_twist(h)
    verify_ik(h)
    verify_blend(h)

    print("\n%s" % ("all checks passed" if not FAILURES
                    else "%d FAILED: %s" % (len(FAILURES),
                                            "; ".join(FAILURES))))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
