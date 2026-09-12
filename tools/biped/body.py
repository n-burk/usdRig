"""The biped's body controls: `hips`, the control that carries the whole
character, and `torso`, the FK swing between the hips and the chest.

Ground truth is the Maya build (`templates/maya/base/rig_bits.nxt`,
`/spine/controls/fk`), where the control hierarchy above the spine is

    hipsPivot_grp                (movable pivot; not reproduced)
      hips                       cube, matched to hips_bind, NO rotation
        hips_gimbal              (gimbal twin of hips; not reproduced)
          hip_swivel             our spine_root_ctl (+ spine_root_pivot)
          spine_mid              our spine_mid_ctl
          torso                  cube, world-aligned, at the average of
            chest                spine_0/spine_1  (/spine/controls/fk/torso/
              chest_ik             position)
                chest_top        our spine_end_ctl (+ spine_end_pivot)

  * `/spine/controls/fk/hips`: control_name 'hips', match_to_node
    'hips_bind', match_rotation False, parent hipsPivot_grp -- so it sits
    at hips_bind's translation with world axes.
  * `/spine/controls/fk/hips_gimbal`: parent 'hips', matched to it. Every
    spine control hangs off the gimbal, not off hips directly; the gimbal
    is an extra rotation channel for the same pivot and is skipped here.
  * `/spine/controls/ik/hip_swivel`: parent 'hips_gimbal'.
  * `/spine/controls/fk/torso`: parent 'hips_gimbal'; `/torso/position`
    puts its nul at `STAGE.hip_matrix` (the hipsPivot control's world
    matrix: hips translation, identity rotation) and then moves it to
    `rig_transform.get_average_position(spine_0_bind, spine_1_bind)`.
  * `/spine/controls/fk/chest`: parent ${torso_control}. chest_ik nests
    under chest and chest_top under chest_ik, so the chest pivot control
    and everything on it -- the clavicles, the arms, the neck (which is
    parent-constrained to chest_top, /neck/head_pivot_connect line 97) --
    swings with torso.
  * `/spine/mid_ik`: parent 'hips_gimbal'.
  * `/spine/hips_space_target`: a transform following `hips` (point +
    rotation constraints) that the space switches on the arms' FK, the
    neck and the head offer as the "hips" space. Our controls nest
    instead, which is Maya's default ("local") space for all of them.

Why torso is not a pass-through: it rotates about the BASE of the spine
with world axes, while the chest pivot (spine_end_pivot, Maya's `chest`
with pivotHeight) rotates about a point 35% of the curve below the chest
and hip_swivel rotates the spine's root end. It is also one of Maya's
`preserve_length` rotate_controls (`/spine/preserve_length` line 7), so
it is a first-class animation control there, and the pickwalk chain
(`body_rig.nxt /post_build/pick_walking/body` line 40) is
head > neck > chest_top > chest > torso > hip_swivel > hips.

What this maps onto in RigExec: namespace nesting, the same device
girdle.py uses for the clavicle and pelvis nuls. A nested control's rest
is parent-local (`world * inverse(parentWorld)`, row-vector), so
`add_body_controls` returns the handles and the builder nests the spine's
pivot controls under them.
"""
from pxr import Gf

HIPS_JOINT = "hips_bind"
TORSO_BETWEEN = ("spine_0_bind", "spine_1_bind")


def flatten(m):
    return [m[r][c] for r in range(4) for c in range(4)]


def world_aligned_at(p):
    """Identity rotation at translation `p`: Maya's match_rotation=False."""
    m = Gf.Matrix4d(1.0)
    m.SetTranslateOnly(Gf.Vec3d(p))
    return m


def add_body_controls(builder, stage, worlds, with_torso=True):
    """`hips_ctl` (Maya `hips`) and, optionally, `torso_ctl` under it.

    Both are world-aligned, as Maya's are. Returns dict(hips=handle,
    torso=handle or None, hips_world=Gf.Matrix4d, torso_world=...).
    """
    if HIPS_JOINT not in worlds:
        raise SystemExit("body controls: no %s in the rig" % HIPS_JOINT)
    hips_w = world_aligned_at(worlds[HIPS_JOINT].ExtractTranslation())
    hips = builder.add_control("hips_ctl", flatten(hips_w))
    print("  %-10s world-aligned at %s (%.2f %.2f %.2f); Maya 'hips'"
          % ("hips_ctl", HIPS_JOINT, *hips_w.ExtractTranslation()))

    torso = None
    torso_w = None
    if with_torso and all(j in worlds for j in TORSO_BETWEEN):
        a, b = (worlds[j].ExtractTranslation() for j in TORSO_BETWEEN)
        torso_w = world_aligned_at((a + b) * 0.5)
        torso = builder.add_control(
            "torso_ctl", flatten(torso_w * hips_w.GetInverse()), hips)
        print("  %-10s nested under hips_ctl, world-aligned at the "
              "midpoint of %s and %s (%.2f %.2f %.2f); Maya 'torso'"
              % ("torso_ctl", TORSO_BETWEEN[0], TORSO_BETWEEN[1],
                 *torso_w.ExtractTranslation()))
    elif with_torso:
        print("  torso_ctl: SKIPPED, missing %s" % ", ".join(
            j for j in TORSO_BETWEEN if j not in worlds))
    return dict(hips=hips, torso=torso, hips_world=hips_w,
                torso_world=torso_w)
