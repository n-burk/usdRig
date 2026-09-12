#!/usr/bin/env python
"""The Maya biped's hand -- cups and fingers -- as RigExec FK.

Ground truth is `rig_bits.nxt:/arm/hand` (`create`, `hand_scale`,
`pinky_cup_falloff`). Maya builds, per hand:

  * a `hand_?_grp` point+orient constrained to `wrist_?_bind` with
    `wrist.s -> grp.s`, so the whole hand rides the wrist;
  * per chain (`index/middle/ring_001..005`, `pinkyCup > pinky_001..005`,
    `thumbCup > thumb_001..004`) a square control per joint, each control
    parented under the previous one, point+orient constraining its joint;
    the last joint of each chain gets no control;
  * `ctrl.sx * ctrl_ik_follow.sx -> jnt.sx` on the chain roots and
    `ctrl.s -> jnt.s` below, with segmentScaleCompensate OFF so scale
    travels down the chain;
  * `wrist_?_bind_blend.scale -> wrist_?_bind.scale` (`hand_scale`);
  * the pinky cup dragging the ring/middle bases along at 0.65 / 0.35
    through a `blendMatrix` on an offset nul (`pinky_cup_falloff`).

In RigExec the joints ARE the controls. A RigExecJoint is a RigExecXformable
with `avars:tx..rz` and `avars:sx..sz`, joints nest in namespace, and a
joint's avars carry every descendant (verified by the caller: spine_2 avars
carried 7 descendants). So

  * the hand rides the wrist by namespace -- `pinkyCup_?_bind`,
    `thumbCup_?_bind` and the three finger bases are children of
    `wrist_?_bind`. The wrist is claimed by the arm's RigExecBlendPointFrames,
    and a solver-claimed joint's unclaimed children still follow it, in FK
    (weight 0) and IK (weight 1) alike -- proven in verify_foot_fingers.py;
  * a chain's control set is its joints' avars; `pinkyCup_?_bind` and
    `thumbCup_?_bind` are first-class links in their chains, exactly as in
    the Maya hierarchy;
  * scale travels: a joint's avars:sx scales its frame's X axis and every
    descendant composes on that frame (verified: parent sx=2 moved a child at
    x=5 to x=10), which is Maya's segmentScaleCompensate=0. The wrist's scale
    reaches the fingers the same way, from the FK wrist control's scale
    avars through the FkChain's affine map.

There is nothing to author for that; `add_fk_fingers` resolves and checks
the chains and hands back their prim paths. What it does NOT reproduce:

  * `pinky_cup_falloff` (ring/middle bases following the pinky cup at
    0.65/0.35, additively with their own animation). RigExec's rotation
    constraint blends TOWARD a source rather than adding to the incoming
    pose, so a 0.65 envelope would also damp the ring's own avars to 35%;
    the additive form needs the ring's `parent:space` re-parented onto a
    partially-rotated offset frame, and that offset frame's rotation is a
    constraint output that the exec-side `parent:space` connection would
    read one round late. Not built rather than built wrong; see the report.
  * `_ik_follow.sx` in the scale product: the IK-phalange system
    (`/arm/hand/ik_fingers`) is empty in this rig, so the factor is 1.
"""
import os
import sys

from pxr import Sdf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def finger_chains(side):
    """Chain name -> joints root-first, as `/arm/hand.joint_list` lists them."""
    return {
        "index": ["index_%03d_%s_bind" % (i, side) for i in range(1, 6)],
        "middle": ["middle_%03d_%s_bind" % (i, side) for i in range(1, 6)],
        "ring": ["ring_%03d_%s_bind" % (i, side) for i in range(1, 6)],
        "pinky": ["pinkyCup_%s_bind" % side] +
                 ["pinky_%03d_%s_bind" % (i, side) for i in range(1, 6)],
        "thumb": ["thumbCup_%s_bind" % side] +
                 ["thumb_%03d_%s_bind" % (i, side) for i in range(1, 5)],
    }


def add_fk_fingers(stage, side, joint_prim_paths, verbose=True):
    """Resolve one hand's FK finger chains.

    Returns {"side", "wrist", "chains": {name: [joint prim paths]},
    "cups": {"pinky": path, "thumb": path}}. Raises when a chain is broken
    or a joint is not a namespace child of the previous one, which would
    mean the skeleton no longer matches the Maya hand.
    """
    wrist = "wrist_%s_bind" % side
    if wrist not in joint_prim_paths:
        raise SystemExit("hand %s: no %s" % (side, wrist))
    chains = {}
    for name, joints in finger_chains(side).items():
        missing = [j for j in joints if j not in joint_prim_paths]
        if missing:
            raise SystemExit("hand %s: %s chain is missing %s"
                             % (side, name, ", ".join(missing)))
        paths = [joint_prim_paths[j] for j in joints]
        prev = joint_prim_paths[wrist]
        for j, p in zip(joints, paths):
            prim = stage.GetPrimAtPath(p)
            if not prim or prim.GetTypeName() != "RigExecJoint":
                raise SystemExit("hand %s: %s is not a RigExecJoint" % (side, j))
            if str(prim.GetPath().GetParentPath()) != prev:
                raise SystemExit("hand %s: %s is not nested under %s"
                                 % (side, j, prev.rsplit("/", 1)[-1]))
            prev = p
        chains[name] = paths
    made = {"side": side, "wrist": joint_prim_paths[wrist], "chains": chains,
            "cups": {"pinky": joint_prim_paths["pinkyCup_%s_bind" % side],
                     "thumb": joint_prim_paths["thumbCup_%s_bind" % side]}}
    if verbose:
        print("  hand_%s: %d chains under %s (%s), FK via joint avars"
              % (side, len(chains), wrist,
                 ", ".join("%s:%d" % (n, len(c))
                           for n, c in sorted(chains.items()))))
    return made


def set_finger_pose(stage, hand, chain, index, rx=0.0, ry=0.0, rz=0.0):
    """Convenience: pose one finger joint's rotation avars (degrees)."""
    prim = stage.GetPrimAtPath(hand["chains"][chain][index])
    for name, value in (("avars:rx", rx), ("avars:ry", ry), ("avars:rz", rz)):
        attr = prim.GetAttribute(name)
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Double)
        attr.Set(float(value))


if __name__ == "__main__":
    for side in ("l", "r"):
        for name, joints in sorted(finger_chains(side).items()):
            print("%s %-6s %s" % (side, name, " > ".join(joints)))
