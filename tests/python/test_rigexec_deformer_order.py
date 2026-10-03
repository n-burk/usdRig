"""The deformation stack runs in the rig's authored order.

Deformer order is not cosmetic on this rig: the head wires are the first
four deformers on the face and everything downstream is shaped by what
they leave behind. Several scopes had been authored with `reorder
nameChildren` set to the SOURCE order, which runs them backwards -- the
pose stack ordinal is the reverse of composed namespace order -- so the
eye stack ran blink-first, socket-last, exactly inverted.

Measured against deformer_order_sync.data, counting only inversions
whose two deformers share weighted vertices (disjoint regions cannot
care):

    before                          1094 inversions, 379 real
    scopes sorted internally         884              (all in-scope gone)
    + geometry scopes reordered      595               89 real

The residual 89 need the head wire and head cluster scopes SPLIT out of
bodyWires_geo/squetch_geo, which would break every reference into those
prims for 17 fewer conflicts; deliberately not done.

This test pins the execution order itself rather than the source data,
which lives outside the repo and testers do not have.
"""
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_STACK = os.path.join(_REPO, "examples", "biped", "Biped_stack.usda")
_RIG = "/Biped/Rig"
_BODY = "/Biped/Geom/body_geo.points"

# Execution order of every mover on body_geo, grouped by the scope it
# lives in, as the rig runs it today.
_SCOPE_RUN = [
    ("shapes_body_geo", [
        "body_geo_shapes",
    ]),
    ("face_shapes_geo", [
        "body_geo_face_shapes",
    ]),
    ("eyes_l_geo", [
        "blinkLower_l_cluster",
        "blinkUpper_l_cluster",
        "blinkUpper_l_curve_wire",
        "blinkLower_l_curve_wire",
        "lid_squash_lower_l_cluster",
        "lid_squash_upper_l_cluster",
        "socketStretch_l_cluster",
        "socketLine_lower_l_cluster",
        "socketLine_upper_l_cluster",
        "lid_l_curve_wire",
        "socketLift_l_cluster",
        "socket_l_curve_wire",
    ]),
    ("eyes_r_geo", [
        "blinkLower_r_cluster",
        "blinkUpper_r_cluster",
        "blinkUpper_r_curve_wire",
        "blinkLower_r_curve_wire",
        "lid_squash_lower_r_cluster",
        "lid_squash_upper_r_cluster",
        "socketStretch_r_cluster",
        "socketLine_lower_r_cluster",
        "socketLine_upper_r_cluster",
        "lid_r_curve_wire",
        "socketLift_r_cluster",
        "socket_r_curve_wire",
    ]),
    ("brows_geo", [
        "brow_corrugator_l_cluster",
        "brow_corrugator_r_cluster",
    ]),
    ("cheeks_geo", [
        "nostril_flare_l_cluster",
        "nostril_r_cluster",
        "nostril_l_cluster",
        "nostril_flare_r_cluster",
        "chin_cluster",
        "cheek_l_cluster",
        "cheek_r_cluster",
        "septum_cluster",
        "ear_l_cluster",
        "ear_r_cluster",
        "cheekLift_r_cluster",
        "cheekLift_l_cluster",
        "cheekPuff_l_cluster",
        "cheekPuff_r_cluster",
        "sneer_l_cluster",
        "sneer_r_cluster",
        "cheek_inner_r_cluster",
        "cheek_inner_l_cluster",
    ]),
    ("mouth_geo", [
        "lip_upper_r_cluster",
        "lip_upper_l_cluster",
        "lip_lower_l_cluster",
        "lip_lower_r_cluster",
        "lip_lower_cluster",
        "lip_upper_cluster",
        "lip_upper_translate_cluster",
        "lip_lower_translate_cluster",
        "mouthMain_cluster",
        "lip_main_wire",
        "lip_lower_center_cluster",
        "lip_upper_center_cluster",
        "lip_wire",
    ]),
    ("squetch_geo", [
        "neck_shift_cluster",
        "neck_shift_scale_cluster",
        "head_low_cluster",
        "head_top_cluster",
        "head_low_aim_cluster",
        "head_top_aim_cluster",
    ]),
    ("bodyWires_geo", [
        "head_low_wire_body_geo",
        "head_wire",
    ]),
    ("skin_body_geo", [
        "body_geo_skin",
    ]),
    ("bendy_geo", [
        "spine_curve_wire",
        "arm_l_curve_wire",
        "arm_r_curve_wire",
        "leg_l_curve_wire",
        "leg_r_curve_wire",
    ]),
]

# The head is what breaks when this slips, so state its invariants
# outright as well as pinning the sequence. Read as evaluation order the
# source appears to put the head wires FIRST (ranks 1-4); read correctly,
# as history order, they are ranks 119-121 and shape the face LAST, on top
# of the lip, cheek and eye correctives.
_HEAD_LAST = ["head_wire", "head_low_wire_body_geo",
              "head_top_cluster", "head_low_cluster"]
_BEFORE_HEAD = ["lip_main_wire", "lip_wire", "mouthMain_cluster",
               "cheek_l_cluster", "cheek_r_cluster",
               "socket_l_curve_wire", "socket_r_curve_wire",
               "blinkUpper_l_cluster", "blinkUpper_r_cluster"]


def main():
    _setup_environment()
    from pxr import Usd
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)

    stage = Usd.Stage.Open(_STACK)
    assert stage, _STACK
    rig = rigexec.Rig(stage, _RIG)
    pose = rig.evaluate(0)
    assert pose.valid, [d for d in pose.diagnostics if "error" in d]

    seq = [e["path"] for e in rig.mover_order()
           if _BODY in [str(t) for t in e.get("targets", [])]]
    names = [p.rsplit("/", 1)[-1] for p in seq]
    at = {n: i for i, n in enumerate(names)}

    got = []
    for p in seq:
        sc = p.split("/Movers/")[1].split("/")[0]
        if not got or got[-1][0] != sc:
            got.append((sc, []))
        got[-1][1].append(p.rsplit("/", 1)[-1])
    got = [(s, list(v)) for s, v in got]

    if got != _SCOPE_RUN:
        want_s = [s for s, _ in _SCOPE_RUN]
        got_s = [s for s, _ in got]
        if want_s != got_s:
            raise AssertionError("scope run order changed: want %s, got %s"
                                 % (want_s, got_s))
        for (s, w), (_, g) in zip(_SCOPE_RUN, got):
            if w != g:
                raise AssertionError("scope %s runs differently: want %s, got %s"
                                     % (s, w, g))
    print("    body_geo runs %d deformers in %d scopes, order unchanged"
          % (len(names), len(got)))

    # 2. the head deformers shape the face LAST, on top of the rest.
    for h in _HEAD_LAST:
        assert h in at, "%s no longer deforms body_geo" % h
    for h in _HEAD_LAST:
        for earlier in _BEFORE_HEAD:
            if earlier not in at:
                continue
            assert at[h] > at[earlier], (
                "%s must run AFTER %s (the head wires are source ranks "
                "119-121, the last thing to shape the face); got %d vs %d"
                % (h, earlier, at[h], at[earlier]))
    print("    the four head deformers run after the lip, cheek and eye stacks")

    # 3. the lash proxies take the head LAST, on top of the lid shaping.
    #    lashes_geo used to run at execution slot 0, ahead of every pose
    #    scope, which put the head wire (rank 121) in the middle of the
    #    lid stack: 13 of 28 pairs inverted, now 2. The residual two are
    #    socketStretch (rank 62) ahead of the blink wires (16, 17), which
    #    share the lashes_geo scope with the blink clusters (11, 13) and
    #    cannot both be right without splitting that scope.
    for side in ("l", "r"):
        mesh = "/Biped/Geom/lash_upper_%s_proxy.points" % side
        run = [e["path"].rsplit("/", 1)[-1] for e in rig.mover_order()
               if mesh in [str(t) for t in e.get("targets", [])]]
        assert run, "nothing deforms lash_upper_%s_proxy" % side
        pos = {n: i for i, n in enumerate(run)}
        last = ["head_top_cluster_lash_upper_%s_proxy" % side,
                "head_wire_lash_upper_%s_proxy" % side]
        before = ["lid_%s_curve_wire_on_lash_upper_%s_proxy" % (side, side),
                  "blinkLower_%s_curve_wire_on_lash_upper_%s_proxy" % (side, side),
                  "blinkUpper_%s_curve_wire_on_lash_upper_%s_proxy" % (side, side),
                  "blinkUpper_%s_cluster_on_lash_upper_%s_proxy" % (side, side),
                  "blinkLower_%s_cluster_on_lash_upper_%s_proxy" % (side, side)]
        for h in last:
            assert h in pos, "%s no longer deforms the lash proxy" % h
            for earlier in before:
                if earlier not in pos:
                    continue
                assert pos[h] > pos[earlier], (
                    "%s must run AFTER %s on the lash proxy; got %d vs %d"
                    % (h, earlier, pos[h], pos[earlier]))
        assert pos[last[0]] < pos[last[1]], (
            "head_top_cluster must precede head_wire on the lash proxy")
    print("    the lash proxies take the head wire and cluster last")

    # 4. the limb wires run AFTER the skin.
    #    Read as evaluation order the source seems to put them before it,
    #    but the file is HISTORY order -- it ends with the blendShape
    #    and tweak4, and a tweak is always first in a deformation chain.
    #    Backwards it reads tweak, blendShape, the blink and lash wires,
    #    sync_geo_skinCluster at 20, then the limb wires at 29-33. The rig
    #    author confirmed it: the arm wires belong after the skinCluster.
    #    Position alone is not the contract -- a wire moved across the skin
    #    and left on pointFrame "rest" reads unskinned points and the limb
    #    tears -- so the frames are asserted with it.
    skin = [i for i, p in enumerate(seq) if p.endswith("body_geo_skin")]
    assert skin, "no body_geo_skin on body_geo"
    for wire in ("arm_l_curve_wire", "arm_r_curve_wire",
                 "leg_l_curve_wire", "leg_r_curve_wire"):
        assert wire in at, "%s no longer deforms body_geo" % wire
        assert at[wire] > skin[0], (
            "%s must run AFTER the skin; it is at %d and the skin at %d"
            % (wire, at[wire], skin[0]))
        prim = stage.GetPrimAtPath(seq[at[wire]])
        for attr, want in (("rigExec:pointFrame", "posed"),
                           ("rigExec:driverDeltaFrame", "posed")):
            a = prim.GetAttribute(attr)
            assert a and a.IsValid() and a.Get() == want, (
                "%s runs post-skin so %s must be '%s', got %r"
                % (wire, attr, want, a.Get() if a and a.IsValid() else None))
    print("    the arm and leg wires run after the skin, in the posed frame")
    print("OK: the deformation stack runs in the authored order")


if __name__ == "__main__":
    main()
