#!/usr/bin/env python
"""Prove the reverse foot, FK foot and FK fingers on a test stage.

Builds the biped's joints and limb IK/FK/blend the way build_biped_rigexec
does (through its own functions), adds both feet and hands from build_foot /
build_fingers, then asserts:

  * rest holds: every bind joint within 1e-3 cm of joint_positions.data
  * IK foot: heel, ball and toe roll each turn the right pivot by the dialled
    angle and carry ankle/ball/toe RIGIDLY about that pivot (expected points
    computed independently from the data); bank tips the foot about the
    bankIn/bankOut edge; the toe-plant handover and the roll clamp
  * the foot's own controls (heel, toeBend) and the ball joint avars in IK
  * the foot rides `leg_?_ik`, in IK and in FK; in FK the IK foot is inert
  * fingers: each chain curls the right joints and nothing else; cups carry
    their digits; the hand rides the arm in FK (weight 0) and IK (weight 1)
    with every hand joint's offset from the wrist preserved, and the fingers
    still curl inside the moved frame; the wrist's FK scale reaches them
  * everything returns to rest exactly (< 1e-9) when zeroed

Usage:
    verify_foot_fingers.py [--out rig.usda]
"""
import argparse
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_biped_skel import DATA, load_joints, skeleton_order  # noqa: E402
from limb_frames import reframe_limbs  # noqa: E402
from build_biped_rigexec import add_limb_iks, flatten  # noqa: E402
from build_foot import (add_reverse_foot, add_fk_foot, apply_foot_shapes,  # noqa: E402
                        pivot_worlds, foot_joints)
from build_fingers import add_fk_fingers, finger_chains  # noqa: E402

import rigexec  # noqa: E402

FAILURES = []
RIG_ROOT = "/Biped/Rig"


def check(ok, label, detail=""):
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", label,
                           ("  -- " + detail) if detail else ""))
    if not ok:
        FAILURES.append(label)
    return ok


def build_test_stage(out_path=None):
    data, parent = load_joints(os.path.join(DATA, "joint_positions.data"))
    ordered = skeleton_order(data, parent)
    rigexec.load_schema_plugin()
    stage = (Usd.Stage.CreateNew(out_path) if out_path
             else Usd.Stage.CreateInMemory())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    builder = rigexec.Builder.create(stage, RIG_ROOT)
    worlds, locals_, poles = reframe_limbs(data, parent, ordered, verbose=False)
    handles = {}
    for name, path in ordered:
        handles[name] = builder.add_joint(name, flatten(locals_[name]),
                                          handles.get(parent.get(name)))
    joint_prim_paths = {n: "%s/Joints/%s" % (RIG_ROOT, p) for n, p in ordered}
    print("limb IK (as build_biped_rigexec.add_limb_iks, with blend):")
    limbs = add_limb_iks(builder, worlds, joint_prim_paths, poles, True)
    by_tag = {tag: (ik, eff, ename, blend) for tag, ik, eff, ename, blend in limbs}

    print("\nreverse feet:")
    feet = {}
    for side in ("l", "r"):
        ik, eff, ename, blend = by_tag["leg_%s" % side]
        feet[side] = add_reverse_foot(builder, stage, side, worlds,
                                      joint_prim_paths, eff, ik, blend, data)
        apply_foot_shapes(stage, feet[side])
    print("\nFK feet:")
    fk_feet = {s: add_fk_foot(stage, s, joint_prim_paths, feet[s])
               for s in ("l", "r")}
    print("\nhands:")
    hands = {s: add_fk_fingers(stage, s, joint_prim_paths) for s in ("l", "r")}

    root_prim = stage.GetPrimAtPath(RIG_ROOT)
    root_prim.CreateAttribute("rigExec:restFrameVersion",
                              Sdf.ValueTypeNames.Int, True,
                              Sdf.VariabilityUniform).Set(2)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Biped"))
    if out_path:
        stage.GetRootLayer().Save()
        print("\nwrote %s" % out_path)
    return dict(stage=stage, data=data, ordered=ordered, worlds=worlds,
                joint_prim_paths=joint_prim_paths, limbs=by_tag, feet=feet,
                fk_feet=fk_feet, hands=hands)


class H(object):
    """Evaluate the rig and read frames; edits go to the session layer."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.stage = ctx["stage"]
        self.stage.SetEditTarget(Usd.EditTarget(self.stage.GetSessionLayer()))
        self.rig = rigexec.Rig(self.stage, RIG_ROOT)
        self.rig.compile()
        self.jp = ctx["joint_prim_paths"]
        self.touched = []

    def pose(self):
        return self.rig.evaluate(0.0)

    def M(self, pose, path):
        try:
            f = pose.joint_frame(path)
        except KeyError:
            f = pose.control_frame(path)
        return Gf.Matrix4d(*f.to_matrix4())

    def O(self, pose, path):
        return self.M(pose, path).ExtractTranslation()

    def joint(self, pose, name):
        return self.M(pose, self.jp[name])

    def set(self, path, attr, value, typ=Sdf.ValueTypeNames.Double):
        prim = self.stage.GetPrimAtPath(path)
        a = prim.GetAttribute(attr)
        if not a or not a.IsValid():
            a = prim.CreateAttribute(attr, typ)
        self.touched.append((a, a.Get()))
        a.Set(value)

    def reset(self):
        for a, v in reversed(self.touched):
            a.Set(v)
        self.touched = []

    def diagnostics(self, pose):
        d = pose.diagnostics
        if callable(d):
            d = d()
        return [x for x in (d or []) if "mover graph" not in x]


def dist(a, b):
    return (a - b).GetLength()


def rot_about(point, pivot, axis, degrees):
    r = Gf.Matrix4d(Gf.Rotation(Gf.Vec3d(axis), degrees), Gf.Vec3d(0))
    return pivot + r.Transform(point - pivot)


def rot_row(m, i):
    return Gf.Vec3d(m[i][0], m[i][1], m[i][2]).GetNormalized()


def local_angle(rest, posed, axis_index):
    """Signed angle (deg) of posed relative to rest about rest's local axis."""
    r4, p4 = Gf.Matrix4d(rest), Gf.Matrix4d(posed)
    r4.SetTranslateOnly(Gf.Vec3d(0))
    p4.SetTranslateOnly(Gf.Vec3d(0))
    rr = (r4.GetInverse() * p4).ExtractRotation()
    ang = rr.GetAngle()
    ax = rr.GetAxis()
    want = rot_row(rest, axis_index)
    sign = 1.0 if Gf.Dot(ax, want) >= 0 else -1.0
    ang = sign * ang
    while ang > 180.0:
        ang -= 360.0
    while ang <= -180.0:
        ang += 360.0
    return ang


def verify_rest(h):
    print("\nrest: RigExec world origins vs Maya world_translate")
    pose = h.pose()
    worst, at, n = 0.0, None, 0
    for name, _ in h.ctx["ordered"]:
        want = h.ctx["data"][name].get("world_translate")
        if not want:
            continue
        got = h.O(pose, h.jp[name])
        d = max(abs(got[i] - want[i]) for i in range(3))
        n += 1
        if d > worst:
            worst, at = d, name
    check(worst < 1e-3, "all %d bind joints at rest" % n,
          "worst %.3g cm (%s)" % (worst, at))
    diags = h.diagnostics(pose)
    check(not diags, "no evaluation diagnostics",
          "; ".join(diags[:3]) if diags else "")


def foot_context(h, side):
    data = h.ctx["data"]
    pw, _ = pivot_worlds(data, side)
    ankle, ball, toe = foot_joints(side)
    rest = {n: Gf.Vec3d(*data[n]["world_translate"]) for n in (ankle, ball, toe)}
    P = lambda n: pw["%s_%s_pivot" % (n, side)]  # noqa: E731
    return P, rest, (ankle, ball, toe)


def verify_foot_ik(h, side):
    print("\nIK foot %s: roll and bank networks" % side)
    feet = h.ctx["feet"][side]
    dials = feet["dials"]
    P, rest, (ankle, ball, toe) = foot_context(h, side)
    knee = "knee_%s_bind" % side
    thigh = "thigh_%s_bind" % side
    piv = feet["pivots"]
    base = h.pose()
    base_o = {n: h.O(base, h.jp[n]) for n in (ankle, ball, toe, knee, thigh)}
    rest_frames = {n: h.M(base, p) for n, p in piv.items()}
    heel_ax = rot_row(P("heel"), 0)      # pivot-local X in world
    heel_pt = P("heel").ExtractTranslation()
    ball_pt = P("ballRoll").ExtractTranslation()
    toe_pt = P("toe").ExtractTranslation()

    # Frames written by the aim constraints carry ~1e-5 cm of single-
    # precision noise (the kernel evidently works in float); everything
    # else lands at 1e-13. AIM_TOL is used where an aim's output is read.
    AIM_TOL = 1e-4
    # Envelope weights are float32 (inputs:defaultWeight), so a dialled
    # angle of R * w is exact to about 120 * 6e-8 = 1e-5 degrees.
    ANG_TOL = 1e-4

    def rigid(pose, names, pivot, axis, deg, label, tol=AIM_TOL):
        worst = 0.0
        for n in names:
            want = rot_about(rest[n], pivot, axis, deg)
            worst = max(worst, dist(h.O(pose, h.jp[n]), want))
        return check(worst < tol, label, "worst %.3g cm" % worst)

    parent_of = {n: str(Sdf.Path(p).GetParentPath()) for n, p in piv.items()}
    rest_local = {n: rest_frames[n] * h.M(base, parent_of[n]).GetInverse()
                  for n in piv}

    def pivot_angle(pose, name, axis_index):
        """The pivot's OWN rotation: relative to its parent's posed frame,
        so a child pivot's reading excludes its parent's roll."""
        local = h.M(pose, piv[name]) * h.M(pose, parent_of[name]).GetInverse()
        return local_angle(rest_local[name], local, axis_index)

    # 1. heel roll
    h.set(feet["controls"]["bank"], "foot:roll", -30.0, Sdf.ValueTypeNames.Float)
    pose = h.pose()
    a = pivot_angle(pose, "heel_%s_roll" % side, 0)
    check(abs(a + 30.0) < ANG_TOL, "roll -30 turns the heel pivot -30",
          "%.4f deg" % a)
    rigid(pose, (ankle, ball, toe), heel_pt, heel_ax, -30.0,
          "roll -30 carries ankle/ball/toe rigidly about the heel")
    check(dist(h.O(pose, h.jp[knee]), base_o[knee]) > 0.5 and
          dist(h.O(pose, h.jp[thigh]), base_o[thigh]) < 1e-9,
          "the leg follows: knee bends, thigh anchored",
          "knee moved %.2f cm" % dist(h.O(pose, h.jp[knee]), base_o[knee]))
    h.reset()

    # 2. ball roll (below the plant angle)
    h.set(feet["controls"]["bank"], "foot:roll", 30.0, Sdf.ValueTypeNames.Float)
    pose = h.pose()
    a = pivot_angle(pose, "ballRoll_%s_up" % side, 0)
    check(abs(a - 30.0) < ANG_TOL, "roll 30 turns ballRoll +30 (heel, toe 0)",
          "ballRoll %.4f, heel %.4f, toe %.4f"
          % (a, pivot_angle(pose, "heel_%s_roll" % side, 0),
             pivot_angle(pose, "toe_%s_roll" % side, 0)))
    rigid(pose, (toe,), ball_pt, heel_ax, 30.0,
          "roll 30 lifts the toe rigidly about the ball")
    check(dist(h.O(pose, h.jp[ankle]), base_o[ankle]) < AIM_TOL and
          dist(h.O(pose, h.jp[ball]), base_o[ball]) < AIM_TOL,
          "roll 30 leaves ankle and ball in place")
    bm = h.joint(pose, ball)
    want_rot = Gf.Matrix4d(Gf.Rotation(heel_ax, 30.0), Gf.Vec3d(0))
    got = h.joint(base, ball) * want_rot
    got.SetTranslateOnly(bm.ExtractTranslation())
    check(Gf.IsClose(bm, got, 1e-5), "the ball joint rotates by exactly the ball roll",
          "max matrix difference %.3g" % max(abs(bm[r][c] - got[r][c]) for r in range(4) for c in range(4)))
    h.reset()

    # 3. toe roll: past the plant angle
    h.set(feet["controls"]["bank"], "foot:roll", 90.0, Sdf.ValueTypeNames.Float)
    pose = h.pose()
    t_a = pivot_angle(pose, "toe_%s_roll" % side, 0)
    up_a = pivot_angle(pose, "ballRoll_%s_up" % side, 0)
    dn_a = pivot_angle(pose, "ballRoll_%s_down" % side, 0)
    check(abs(t_a - 30) < ANG_TOL and abs(up_a - 60) < ANG_TOL and abs(dn_a + 30) < ANG_TOL,
          "roll 90 (plant 60): toe 30, ballRoll 60 - 30",
          "toe %.3f up %.3f down %.3f" % (t_a, up_a, dn_a))
    rigid(pose, (ankle, ball), toe_pt, heel_ax, 30.0,
          "roll 90 carries ankle and ball rigidly about the toe tip")
    ball_now = rot_about(rest[ball], toe_pt, heel_ax, 30.0)
    want_toe = ball_now + Gf.Matrix4d(Gf.Rotation(heel_ax, 60.0), Gf.Vec3d(0)).Transform(rest[toe] - rest[ball])
    check(dist(h.O(pose, h.jp[toe]), want_toe) < AIM_TOL,
          "the toe joint sits on the toe target (ball aims at it)",
          "%.3g cm" % dist(h.O(pose, h.jp[toe]), want_toe))
    h.set(feet["controls"]["bank"], "foot:toePlantAngle", 80.0, Sdf.ValueTypeNames.Float)
    pose = h.pose()
    t_a = pivot_angle(pose, "toe_%s_roll" % side, 0)
    up_a = pivot_angle(pose, "ballRoll_%s_up" % side, 0)
    dn_a = pivot_angle(pose, "ballRoll_%s_down" % side, 0)
    weights = ["%s=%.4f" % (c.rsplit("/", 1)[-1], pose.moved_property(c + ".inputs:defaultWeight"))
               for c in feet["constraints"] if "Roll" in c or "toe_" in c]
    check(abs(t_a - 10) < ANG_TOL and abs(up_a - 80) < ANG_TOL and abs(dn_a + 10) < ANG_TOL,
          "toePlantAngle is live: plant 80 -> toe 10, ballRoll 80 - 10",
          "toe %.3f up %.3f down %.3f; envelopes %s" % (t_a, up_a, dn_a, " ".join(weights)))
    h.reset()

    # 4. clamp
    h.set(feet["controls"]["bank"], "foot:roll", -500.0, Sdf.ValueTypeNames.Float)
    pose = h.pose()
    a = pivot_angle(pose, "heel_%s_roll" % side, 0)
    check(abs(a + 120.0) < ANG_TOL, "roll -500 clamps the heel at -120", "%.3f" % a)
    h.reset()

    # 5. bank
    for value, pname, edge in ((20.0, "bankOut_%s_pivot" % side, "outer"),
                               (-20.0, "bankIn_%s_pivot" % side, "inner")):
        h.set(feet["controls"]["bank"], "foot:bank", value, Sdf.ValueTypeNames.Float)
        pose = h.pose()
        a = pivot_angle(pose, pname, 2)
        other = ("bankIn_%s_pivot" if edge == "outer" else "bankOut_%s_pivot") % side
        check(abs(a + value) < ANG_TOL and abs(pivot_angle(pose, other, 2)) < 1e-9,
              "bank %+.0f turns %s by %+.0f about its Z, the other edge 0"
              % (value, pname, -value), "%.4f deg" % a)
        pm = rest_frames[pname]
        rigid(pose, (ankle, ball, toe), pm.ExtractTranslation(), rot_row(pm, 2),
              -value, "bank %+.0f tips the foot rigidly about the %s edge"
              % (value, edge))
        h.reset()

    # 6. the animator's cubes
    h.set(feet["controls"]["heel"], "avars:rx", -20.0)
    pose = h.pose()
    rigid(pose, (ankle, ball, toe), heel_pt, heel_ax, -20.0,
          "heel control rx -20 rolls the foot about the heel")
    h.reset()
    h.set(feet["controls"]["toeBend"], "avars:rx", 20.0)
    pose = h.pose()
    rigid(pose, (ankle,), ball_pt, heel_ax, 20.0,
          "toeBend rx 20 swings the ankle about the ball")
    check(dist(h.O(pose, h.jp[ball]), base_o[ball]) < AIM_TOL and
          dist(h.O(pose, h.jp[toe]), base_o[toe]) < AIM_TOL,
          "toeBend leaves ball and toe planted",
          "ball %.3g, toe %.3g cm" % (dist(h.O(pose, h.jp[ball]), base_o[ball]),
                                      dist(h.O(pose, h.jp[toe]), base_o[toe])))
    h.reset()

    # 7. the ball joint's own avars in IK == the same avars in FK
    blend_w = (h.ctx["limbs"]["leg_%s" % side][3].path, "inputs:weight")
    h.set(h.jp[ball], "avars:rz", 25.0)
    pose = h.pose()
    ik_ball, ik_toe = h.joint(pose, ball), h.joint(pose, toe)
    check(dist(h.O(pose, h.jp[ankle]), base_o[ankle]) < AIM_TOL and
          dist(h.O(pose, h.jp[toe]), base_o[toe]) > 0.5,
          "IK: ball avars rz 25 move the toe, not the ankle")
    h.set(blend_w[0], blend_w[1], 0.0, Sdf.ValueTypeNames.Float)
    pose = h.pose()
    check(Gf.IsClose(h.joint(pose, ball), ik_ball, 1e-5) and
          Gf.IsClose(h.joint(pose, toe), ik_toe, 1e-5),
          "the same ball avars give the identical ball/toe frames in FK")
    h.reset()

    # 8. the foot rides the leg IK control
    # Translation avars are in the control's own rest frame, whose X runs
    # down the leg; (-6, 2, 0) is 6 cm up the leg and 2 cm forward, which
    # shortens the thigh-to-ankle distance and so is reachable. (A move
    # that lengthened it past full reach would be clamped by the TwoBoneIk
    # -- correct, but not a rigidity test.)
    # The right control's frame is the mirrored ankle frame (X up the leg,
    # Y backward), so its local move is negated to get the same world move.
    ik_ctl = h.ctx["limbs"]["leg_%s" % side][1].path
    ik_rest = Gf.Matrix4d(h.stage.GetPrimAtPath(ik_ctl).GetAttribute("rest:space").Get())
    local = Gf.Vec3d(-6, 2, 0) if side == "l" else Gf.Vec3d(6, -2, 0)
    shift = local * ik_rest.ExtractRotationMatrix()
    h.set(ik_ctl, "avars:tx", local[0])
    h.set(ik_ctl, "avars:ty", local[1])
    pose = h.pose()
    worst = max(dist(h.O(pose, h.jp[n]), base_o[n] + shift)
                for n in (ankle, ball, toe))
    check(worst < AIM_TOL, "leg_%s_ik avars (%.0f,%.0f,0) carry ankle/ball/toe rigidly" % (side, local[0], local[1]),
          "world shift (%.2f %.2f %.2f); worst %.3g cm; knee moved %.2f cm"
          % (shift[0], shift[1], shift[2], worst, dist(h.O(pose, h.jp[knee]), base_o[knee])))
    h.reset()

    # 9. FK leg: the IK foot is inert, the FK foot works
    h.set(blend_w[0], blend_w[1], 0.0, Sdf.ValueTypeNames.Float)
    h.set(feet["controls"]["bank"], "foot:roll", -30.0, Sdf.ValueTypeNames.Float)
    pose = h.pose()
    worst = max(dist(h.O(pose, h.jp[n]), base_o[n]) for n in (ankle, ball, toe))
    check(worst < 1e-9, "FK leg: foot:roll moves nothing", "worst %.3g cm" % worst)
    fk_ankle = "%s/Controls/leg_%s_fk_ankle_%s_bind" % (RIG_ROOT, side, side)
    h.set(fk_ankle, "avars:rz", 20.0)
    pose = h.pose()
    check(dist(h.O(pose, h.jp[ball]), base_o[ball]) > 0.5 and
          dist(h.O(pose, h.jp[toe]), base_o[toe]) > 0.5 and
          dist(h.O(pose, h.jp[ankle]), base_o[ankle]) < 1e-9,
          "FK leg: the ankle FK control carries ball and toe")
    h.set(h.jp[ball], "avars:rz", 30.0)
    pose2 = h.pose()
    check(dist(h.O(pose2, h.jp[toe]), h.O(pose, h.jp[toe])) > 0.5,
          "FK leg: ball avars carry the toe inside the posed ankle")
    fk_thigh = "%s/Controls/leg_%s_fk_thigh_%s_bind" % (RIG_ROOT, side, side)
    h.set(fk_thigh, "avars:rz", 15.0)
    pose3 = h.pose()
    check(dist(h.O(pose3, h.jp[ankle]), base_o[ankle]) > 5.0,
          "FK leg: the thigh FK control carries the whole foot")
    h.reset()

    pose = h.pose()
    worst = max(dist(h.O(pose, h.jp[n]), base_o[n]) for n in (ankle, ball, toe, knee))
    check(worst < 1e-9, "foot %s returns to rest exactly" % side, "%.3g cm" % worst)


def verify_hand(h, side):
    print("\nhand %s: fingers, cups, and riding the arm" % side)
    hand = h.ctx["hands"][side]
    chains = finger_chains(side)
    all_hand = sorted({j for c in chains.values() for j in c})
    wrist = "wrist_%s_bind" % side
    base = h.pose()
    base_o = {n: h.O(base, h.jp[n]) for n in all_hand + [wrist]}
    rel_rest = {n: h.joint(base, n) * h.joint(base, wrist).GetInverse()
                for n in all_hand}

    def curl_check(pose, chain, idx, label):
        joints = chains[chain]
        moves = joints[idx + 1:]
        holds = [j for j in all_hand if j not in moves and j != joints[idx]] + [wrist]
        least = min(dist(h.O(pose, h.jp[n]), base_o[n]) for n in moves)
        worst = max(dist(h.O(pose, h.jp[n]), base_o[n]) for n in holds)
        check(least > 0.3 and worst < 1e-9, label,
              "%d joints moved (least %.2f cm); %d others held (worst %.3g)"
              % (len(moves), least, len(holds), worst))

    # one curl per chain, at a mid joint
    for chain, idx in (("index", 1), ("middle", 2), ("ring", 1),
                       ("pinky", 2), ("thumb", 1)):
        j = chains[chain][idx]
        h.set(h.jp[j], "avars:rz", 40.0)
        pose = h.pose()
        curl_check(pose, chain, idx, "%s: %s rz 40 curls only its descendants"
                   % (chain, j))
        # the joint's own local rotation is exactly the avar
        parent = chains[chain][idx - 1] if idx else wrist
        loc_rest = h.joint(base, j) * h.joint(base, parent).GetInverse()
        loc_pose = h.joint(pose, j) * h.joint(pose, parent).GetInverse()
        ang = (loc_pose * loc_rest.GetInverse()).ExtractRotation().GetAngle()
        check(abs(ang - 40.0) < 1e-6, "%s: local rotation is 40 deg" % chain,
              "%.4f" % ang)
        h.reset()

    # the cups are first-class links
    for cup, chain in (("pinky", "pinky"), ("thumb", "thumb")):
        h.set(hand["cups"][cup], "avars:ry", 30.0)
        pose = h.pose()
        curl_check(pose, chain, 0, "%sCup ry 30 carries every %s joint and "
                   "nothing else" % (cup, chain))
        h.reset()

    # the hand rides the arm: FK (weight 0 is the shipped default) and IK
    limb = h.ctx["limbs"]["arm_%s" % side]
    blend_path = limb[3].path
    fk_shoulder = "%s/Controls/arm_%s_fk_shoulder_%s_bind" % (RIG_ROOT, side, side)
    fk_wrist = "%s/Controls/arm_%s_fk_wrist_%s_bind" % (RIG_ROOT, side, side)
    ik_ctl = limb[1].path

    def rides(pose, label):
        least = min(dist(h.O(pose, h.jp[n]), base_o[n]) for n in all_hand)
        worst = max((h.joint(pose, n) * h.joint(pose, wrist).GetInverse() -
                     rel_rest[n]).GetRow(3).GetLength() +
                    max(abs(x) for r in range(3) for x in
                        (h.joint(pose, n) * h.joint(pose, wrist).GetInverse() -
                         rel_rest[n]).GetRow(r))
                    for n in all_hand)
        check(least > 0.3 and worst < 1e-6, label,
              "least moved %.2f cm; wrist-relative frames drift %.3g" % (least, worst))

    for weight, driver, attr, value, mode in (
            (0.0, fk_shoulder, "avars:ry", 30.0, "FK"),
            (0.0, fk_wrist, "avars:rx", 45.0, "FK wrist"),
            (1.0, ik_ctl, "avars:tx", -10.0, "IK"),
            (1.0, ik_ctl, "avars:rz", 40.0, "IK rotate")):
        h.set(blend_path, "inputs:weight", weight, Sdf.ValueTypeNames.Float)
        h.set(driver, attr, value)
        pose = h.pose()
        rides(pose, "%s: %s %s %+.0f carries wrist, cups and all %d finger "
              "joints" % (mode, driver.rsplit("/", 1)[-1], attr, value, len(all_hand)))
        # and a finger still curls inside the moved frame
        j = chains["index"][1]
        h.set(h.jp[j], "avars:rz", 40.0)
        pose2 = h.pose()
        parent = chains["index"][0]
        loc_rest = h.joint(base, j) * h.joint(base, parent).GetInverse()
        loc_pose = h.joint(pose2, j) * h.joint(pose2, parent).GetInverse()
        ang = (loc_pose * loc_rest.GetInverse()).ExtractRotation().GetAngle()
        tip_moved = dist(h.O(pose2, h.jp[chains["index"][4]]),
                         h.O(pose, h.jp[chains["index"][4]]))
        held = dist(h.O(pose2, h.jp[parent]), h.O(pose, h.jp[parent]))
        check(abs(ang - 40) < 1e-6 and tip_moved > 0.3 and held < 1e-9,
              "%s: index_002 still curls 40 deg inside the moved hand" % mode,
              "tip moved %.2f cm" % tip_moved)
        h.reset()

    # wrist scale reaches the fingers (Maya: wrist_blend.scale -> wrist.scale,
    # segmentScaleCompensate off)
    h.set(blend_path, "inputs:weight", 0.0, Sdf.ValueTypeNames.Float)
    h.set(fk_wrist, "avars:sx", 1.5)
    pose = h.pose()
    j = chains["index"][0]
    rest_off = rel_rest[j].ExtractTranslation()
    off = (h.joint(pose, j) * h.joint(pose, wrist).GetInverse()).ExtractTranslation()
    wm = h.joint(pose, wrist)
    xlen = Gf.Vec3d(wm[0][0], wm[0][1], wm[0][2]).GetLength()
    check(abs(xlen - 1.5) < 1e-6, "FK wrist sx 1.5 scales the wrist frame's X",
          "|X| = %.4f" % xlen)
    # relative to the SCALED wrist frame the offset is unchanged, i.e. in
    # world it grew 1.5x along the wrist X
    world_off = h.O(pose, h.jp[j]) - h.O(pose, h.jp[wrist])
    want = Gf.Vec3d(rest_off[0] * 1.5, rest_off[1], rest_off[2]) * h.joint(base, wrist).ExtractRotationMatrix()
    check(dist(world_off, want) < 1e-6 and dist(off, rest_off) < 1e-6,
          "the finger base rides the scaled wrist axis (scale travels)",
          "offset %.3f -> %.3f cm along wrist X" % (rest_off[0], (world_off * h.joint(base, wrist).ExtractRotationMatrix().GetInverse())[0]))
    h.reset()

    pose = h.pose()
    worst = max(dist(h.O(pose, h.jp[n]), base_o[n]) for n in all_hand + [wrist])
    check(worst < 1e-9, "hand %s returns to rest exactly" % side, "%.3g cm" % worst)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=None, help="also write the test rig here")
    args = ap.parse_args(argv)
    ctx = build_test_stage(args.out)
    h = H(ctx)
    verify_rest(h)
    for side in ("l", "r"):
        verify_foot_ik(h, side)
    for side in ("l", "r"):
        verify_hand(h, side)
    print("\nfinal rest check after every edit was reverted:")
    verify_rest(h)
    print("\n%s" % ("all checks passed" if not FAILURES
                    else "%d FAILED:\n  %s" % (len(FAILURES), "\n  ".join(FAILURES))))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
