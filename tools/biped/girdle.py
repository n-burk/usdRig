"""The shoulder and hip girdles of the biped, and the hips following the
hip swivel -- the pieces that hang the clavicles off the chest and the
pelvis off the hip swivel.

Ground truth is the Maya build (`templates/maya/base/rig_bits.nxt`):

  * `/leg/pelvis/control`  : the pelvis control is matched to
    `pelvis_?_bind` and its nul is parented under `hip_swivel`
    (rig_bits.nxt line 8608). `/arm/clavicle/control` matches the clavicle
    control to `clavicle_?_bind`; `/arm/limb/create/clavicle_connect`
    parents its nul under the anchor `chest_top` (line 1170).
  * The joint is POINT-constrained to the control -- `mc.pointConstraint(
    clavicle_control, clavicle_joint)` (line 1159) -- and the orient
    constraint is deliberately commented out (line 1155: "getting rid of
    the orient constraint for now ... allow us to translate and move limbs
    together"). The same node builds the pelvis, with `clavicle_control`
    = `pelvis_?` and `clavicle_anchor` = `hip_swivel` (lines 8328-8333).
    The clavicle's rotation reaches the skin through the aim-constrained
    `clavicle_trans_?_bind` instead (body_rig.nxt
    `/load/joints/connect_twist/left/arm/clavicle_translate`), which
    `build_biped_rigexec.add_twist_aims` already reproduces.
  * `/spine/controls/ik/hip_pivot` line 58-59: `parentConstraint(
    hip_swivel_grp, hips_bind, mo=1)` and `hip_swivel_grp.s -> hips_bind.s`
    (rig_bits.nxt 13489-13490). `hip_swivel_grp` sits at the control's own
    matrix under it, so this is "hips_bind follows hip_swivel with a
    maintained offset".
  * `/spine/controls/ik/chest_top/connect` lines 8-18 (rig_bits.nxt
    13418-13424): the spline's own constraints on `chest_bind` are DELETED
    and replaced with point + orient constraints from `chest_top_grp`
    (mo=1), a group at chest_bind's matrix under `chest_top`. The scale
    connection from the ik joint stays. So chest_bind rigidly follows the
    chest control, position and rotation, and takes its squash from the
    spline.
  * Twist. spline.py 213-265: a start locator at ik joint 0 and an end
    locator at the tip each get `decompose_rotation` (a swing-twist split
    about the chain axis); `ikHandle.roll = start.decomposeTwist` and
    `ikHandle.twist = end.decomposeTwist - start.decomposeTwist`. The start
    locator is orient-constrained from `hip_swivel_grp` (rig_bits 13550)
    and the end one from `chest_top_grp` (rig_bits 13412). RigExecSplineIk
    implements exactly this internally (libs/rigExecMath/splineIk.cpp
    511-539: roll = rootTwist, twist = endTwist - rootTwist, joint twist =
    roll + twist * t_i), reading the root/end controls' rest->pose
    rotation, so nothing has to be wired for the swivel and chest to twist
    the spine: `verify_girdle.py` measures it.

How this maps onto RigExec:

  * Maya's "nul under a control" is namespace NESTING here: `add_control(
    name, rest, parent)` composes the child's frame through its ancestor,
    so a pelvis control nested under `spine_root_ctl` travels with it.
    Nested rests are parent-local, `world * inverse(parentWorld)` in the
    row-vector convention every other builder helper uses.
  * The point constraints are `RigExecPositionConstraint`s; the chest and
    hips follow through `RigExecParentConstraint` + `RigExecScaleConstraint`
    with the offset that `_offset_in_source_space` computes (RigExec has no
    maintainOffset; the offset is composed in the source's space).

ORDER. A mover chain applies in reverse add order, both across chains
(the chain created last executes first) and within one, and a joint's
write propagates rigidly to every descendant. So:

  * `add_hips_follow` writes `hips_bind`, the root of everything. It must
    execute BEFORE the spine's `_to_bind` constraints (which write
    spine_0..chest absolutely from the solved duplicate chain) or its
    propagation would drag them off the spline again. Create its chain
    AFTER `finish_spline_chains`.
  * `add_girdles` writes the clavicle and pelvis joints, and overrides
    chest_bind. Those must execute AFTER the spine writes (chest_bind is
    the clavicles' parent; the spline writes chest_bind). Create its chain
    BEFORE the spine's `bind_spline_chain` -- but after
    `add_spline_ik_chain`, because the controls nest under the spline's
    root/end controls. Within the chain the clavicles are added first and
    chest_bind last, so chest executes first and the clavicles land after
    its propagation.
  * The NECK's bind chain must execute AFTER the girdles' chest_bind
    override (the neck joints descend from chest_bind, and its re-write
    propagates onto anything already written), so it is created BEFORE
    `add_girdles`. Created after the spine's, as it once was, it ran
    before both and a 30 degree hip swivel moved neck_0_bind 25.64 cm
    with chest_bind at 0.00. The builder's build() has the full table.
  * Skinning stays first-created (last-executed), as the builder already
    does; the twist aims are created right after it so they run after
    every hierarchy write.
"""
from pxr import Gf

# (control name, bind joint, which spline control it hangs from)
CLAVICLES = [("clavicle_%s_ctl" % s, "clavicle_%s_bind" % s) for s in "lr"]
PELVES = [("pelvis_%s_ctl" % s, "pelvis_%s_bind" % s) for s in "lr"]

HIPS_JOINT = "hips_bind"
CHEST_JOINT = "chest_bind"


def flatten(m):
    """GfMatrix4d -> the 16 row-major doubles the builder expects."""
    return [m[r][c] for r in range(4) for c in range(4)]


def offset_in_source_space(target_rest, source_rest):
    """The constant offset a RigExecParentConstraint needs to hold rest.

    RigExec composes the authored offset as `offset * source` (row-vector),
    i.e. in the source's local space, so the offset that reproduces the
    rest relationship is `C = target_rest * inverse(source_rest)`, constant
    under pose. Returned as (translation, rotationXYZ degrees), which is how
    the schema stores it. Same maths as the builder's
    `_offset_in_source_space`, duplicated so this module does not import
    the builder (which would run its argparse-level imports).
    """
    c = target_rest * source_rest.GetInverse()
    t = c.ExtractTranslation()
    r = c.ExtractRotation().Decompose(Gf.Vec3d(0, 0, 1), Gf.Vec3d(0, 1, 0),
                                      Gf.Vec3d(1, 0, 0))
    # Decompose returns Z, Y, X for those axes; the schema wants XYZ.
    return (t[0], t[1], t[2]), (r[2], r[1], r[0])


def _control_rest(stage, ctl):
    """A control's rest in ASSET space, composed up the namespace.

    `rest:space` is parent-local for a control with a control ancestor
    (rigExec:restFrameVersion 2), so it is composed through every
    RigExecControl ancestor -- the same walk as the builder's
    `control_world_rest`. This used to read `rest:space` alone, which
    was right only while the spine controls were top-level; they now
    nest under their pivot controls (spine_root_pivot / spine_end_pivot),
    and a parent-local rest treated as world would put the clavicle
    nests and the chest/hips offsets a whole pivot-offset off.
    """
    m = Gf.Matrix4d(1.0)
    prim = stage.GetPrimAtPath(ctl.path)
    while prim and prim.IsValid():
        attr = prim.GetAttribute("rest:space")
        if attr and attr.IsValid() and attr.Get() is not None:
            m = m * Gf.Matrix4d(attr.Get())
        prim = prim.GetParent()
        if prim and prim.GetTypeName() != "RigExecControl":
            break
    return m


def add_nested_control(builder, stage, name, world_rest, parent_ctl):
    """A control nested under `parent_ctl`, sitting at `world_rest`.

    The rest is parent-local (row-vector: local = world * inverse(parent)),
    the rule `add_fk_controls` in the builder uses, so the control's posed
    frame composes through the parent and travels with it -- Maya's nul
    parented under the anchor control.
    """
    parent_rest = _control_rest(stage, parent_ctl)
    local = world_rest * parent_rest.GetInverse()
    return builder.add_control(name, flatten(local), parent_ctl)


def add_girdles(builder, stage, worlds, joint_prim_paths, root_ctl, end_ctl,
                chest_follows_end=True, chain_name="girdles"):
    """Clavicle and pelvis controls plus the constraints that hang them.

    Call AFTER `add_spline_ik_chain` (the controls nest under its root and
    end controls) and BEFORE `finish_spline_chains` (so this chain executes
    after the spine's writes -- see the module docstring for why).

    `root_ctl` / `end_ctl` are the spine's `spine_root_ctl` (Maya's
    hip_swivel) and `spine_end_ctl` (Maya's chest_top).

    With `chest_follows_end`, chest_bind is parent-constrained to the end
    control, which is what Maya does (`/spine/controls/ik/chest_top/connect`
    replaces the spline's constraints on chest_bind with point + orient
    from `chest_top_grp`). Scale still comes from the spline's scale
    constraint, as in Maya, because a parent constraint does not touch it.

    Returns dict(controls={name: handle}, chain=chain, constraints=[...]).
    """
    controls = {}
    made = []

    for name, joint in CLAVICLES:
        if joint in worlds:
            controls[name] = add_nested_control(
                builder, stage, name, worlds[joint], end_ctl)
    for name, joint in PELVES:
        if joint in worlds:
            controls[name] = add_nested_control(
                builder, stage, name, worlds[joint], root_ctl)

    chain = builder.new_mover_chain(chain_name)

    # Add order is the REVERSE of execution. The children go in first so
    # they run last, after chest_bind's write has propagated to them.
    for name, joint in CLAVICLES + PELVES:
        ctl = controls.get(name)
        if ctl is None:
            print("  %s: SKIPPED, no %s" % (name, joint))
            continue
        # Maya: pointConstraint(control, joint) -- translation only; the
        # joint's rotation stays with its parent's, and the control's rest
        # IS the joint's rest so no offset is needed.
        c = chain.add_position_constraint("%s_drives_%s" % (name, joint),
                                          joint_prim_paths[joint], [ctl])
        made.append((name, joint, c))
        print("  %-14s nested under %-14s point-drives %s"
              % (name, (end_ctl if name.startswith("clav") else
                        root_ctl).name, joint))

    if chest_follows_end and CHEST_JOINT in worlds:
        end_rest = _control_rest(stage, end_ctl)
        t, r = offset_in_source_space(worlds[CHEST_JOINT], end_rest)
        pc = chain.add_parent_constraint("%s_drives_%s"
                                         % (end_ctl.name, CHEST_JOINT),
                                         joint_prim_paths[CHEST_JOINT],
                                         [end_ctl])
        pc.set_translation_offsets([t])
        pc.set_rotation_offsets([r])
        made.append((end_ctl.name, CHEST_JOINT, pc))
        print("  %-14s parent-drives %s (Maya: chest_top_grp point+orient "
              "mo=1, replacing the spline's)" % (end_ctl.name, CHEST_JOINT))

    return dict(controls=controls, chain=chain, constraints=made)


def add_hips_follow(builder, stage, worlds, joint_prim_paths, root_ctl,
                    chain_name="hips_follow"):
    """hips_bind follows the hip swivel: parent + scale constraint, offset.

    Maya: `parentConstraint(hip_swivel_grp, hips_bind, mo=1)` and
    `hip_swivel_grp.s -> hips_bind.s` (rig_bits.nxt 13489-13490).

    Call AFTER `finish_spline_chains`, so this chain is created last and
    therefore EXECUTES FIRST: hips_bind's write propagates to every
    descendant (the whole spine, both legs, the neck, the arms), and the
    spine's own absolute writes then land on top of it. Created any
    earlier, the propagation would re-move the spine after the spline had
    placed it.
    """
    if HIPS_JOINT not in worlds:
        print("  %s: SKIPPED, no such joint" % HIPS_JOINT)
        return None
    root_rest = _control_rest(stage, root_ctl)
    t, r = offset_in_source_space(worlds[HIPS_JOINT], root_rest)
    chain = builder.new_mover_chain(chain_name)
    # Scale first (runs last) so it lands on the parent-constrained frame.
    sc = chain.add_scale_constraint("%s_scales_%s"
                                    % (root_ctl.name, HIPS_JOINT),
                                    joint_prim_paths[HIPS_JOINT], [root_ctl])
    pc = chain.add_parent_constraint("%s_drives_%s"
                                     % (root_ctl.name, HIPS_JOINT),
                                     joint_prim_paths[HIPS_JOINT], [root_ctl])
    pc.set_translation_offsets([t])
    pc.set_rotation_offsets([r])
    print("  %-14s parent+scale-drives %s, offset t=(%.2f %.2f %.2f) "
          "r=(%.1f %.1f %.1f)" % (root_ctl.name, HIPS_JOINT, t[0], t[1],
                                  t[2], r[0], r[1], r[2]))
    return dict(chain=chain, parent=pc, scale=sc)
