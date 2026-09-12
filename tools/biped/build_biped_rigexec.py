#!/usr/bin/env python
"""Author the Maya biped's bind skeleton as RigExec joint prims.

Why this exists alongside `build_biped_skel.py`: a `UsdSkelSkeleton` is a
single prim holding parallel arrays, so its joints are not prims -- you
cannot select one in usdview, and there is nothing for a rig to drive.
RigExec joints ARE prims, nested in namespace, which is what makes them
individually selectable and drivable by constraints and solvers.

Rest is parent-local (`rigExec:restFrameVersion = 2`): RigExec composes a
joint's rest frame as

    orthonormalize(compose(rest:t, rest:r) * rest:space) * restWorld(parent)

so `rest:space` is the joint's transform *relative to its parent joint*, not
asset space. Handing it a world matrix -- as the older
`examples/python/make_fk_arm.py` does, from before the parent-local
migration -- makes the transforms accumulate down the chain. This script
passes local matrices and then proves the composition by evaluating the rig
and comparing every joint's world origin against the `world_translate`
recorded by Maya.

Usage:
    build_biped_rigexec.py <out.usda> [--skin <biped_skel.usda>]

`--skin` sublayers an existing UsdSkel asset so the mesh shows up in the
same stage as the joints. The two are not yet wired together: RigExec
publishes joint *guides* to Hydra but not joint transforms, so a UsdSkel
mesh will not follow RigExec joints without a bridge that does not exist in
this repo. Phase 2 decides whether that bridge or RigExec's own
MatrixMover skinning is the route.
"""
import argparse
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, Vt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_biped_skel import (DATA, load_joints, local_matrix,
                              parse_weights, skeleton_order)

import rigexec


def flatten(m):
    """GfMatrix4d -> the 16 row-major doubles add_joint expects."""
    return [m[r][c] for r in range(4) for c in range(4)]


# LIMBS, POLE_OFFSET and pole_position live in limb_frames so the skel
# builder and this one cannot drift apart on limb geometry.
from limb_frames import LIMBS, POLE_OFFSET, pole_position  # noqa: E402


# Shipping IK/FK state per limb, from body_rig.nxt:/apply/control_defaults.
# 0 = FK (blend input A), 1 = IK (blend input B).
BLEND_DEFAULTS = {"arm_l": 0.0, "arm_r": 0.0, "leg_l": 1.0, "leg_r": 1.0}


# The animatable IK/FK dial. It lives on the limb's ROOT control
# (shoulderSwing / thighSwing) because that control is meaningful in both
# modes and so is never the one hidden, and the blend solver's
# `inputs:weight` is CONNECTED to it -- verified to interpolate linearly:
# at 0.00/0.25/0.50/0.75/1.00 a 20 cm effector move reached the wrist as
# 0.00/5.00/10.00/15.00/20.00 cm.
#
# Semantics follow the solver, not Maya: 0 = FK, 1 = IK. Maya's own `ikfk`
# attribute is inverted (1 = FK there), which is worth knowing when reading
# body_rig.nxt but would be a trap to reproduce here, where the value is
# literally the RigExecBlendPointFrames weight.
IKFK_AVAR = "avars:ikfk"


def control_world_rest(stage, path):
    """A control's rest in ASSET space, composed up the namespace.

    `rest:space` is parent-local for any control that has a control
    ancestor, exactly as it is for joints. Reading it directly off a
    nested control and treating it as world is what sent the rest
    check from 4.9e-05 cm to 131.884 cm at shoulder_l_bind: the
    girdle controls are nested under the spine controls, so their
    rest:space is relative to the chest or the hips, not the asset.
    """
    m = Gf.Matrix4d(1.0)
    prim = stage.GetPrimAtPath(path)
    while prim and prim.IsValid():
        attr = prim.GetAttribute("rest:space")
        if attr and attr.IsValid() and attr.Get() is not None:
            m = m * Gf.Matrix4d(attr.Get())
        prim = prim.GetParent()
        if prim and prim.GetTypeName() != "RigExecControl":
            break
    return m


def add_limb_iks(builder, stage, worlds, by_name, poles, with_blend=False,
                 fk_follow_parent=False, girdle_parents=None,
                 ik_handles_world=True):
    """RigExecTwoBoneIk per limb, optionally with an FK chain and IK/FK blend.

    Controls sit under the rig's /Controls scope with no RigExec ancestor,
    so their rest is asset space and takes a world matrix -- unlike the
    joints, whose rest is parent-local.

    The IK effector and pole controls are TOP-LEVEL (asset space) by
    default, which is Maya's arrangement: `rig_bits.nxt /limb/create/ik/
    controls` creates `leg_ik_?` / `arm_ik_?` and the pole vector with
    `parent=ikfk_group`, and `/limb/create` lines 18-19 parent that group
    under `rig`, so the handle lives in world and a planted foot stays
    planted when the hips, the swivel or the pelvis move. The control's
    space switch (`/leg/limb/ik_spaces`: hip_swivel / hips) is ORIENT-only
    (`/space_switch` constraint_type='orient') and defaults to local, so
    it never moves the handle either. `ik_handles_world=False` is the
    older arrangement, nesting them under the girdle control so the hand
    or foot rides the clavicle or pelvis; it is kept reachable through
    `--ik-handles-follow-girdle`.

    With `with_blend`, the stock `examples/03_IkFkBlendClamp.usda` topology
    is used: an FK chain is input A, the IK is input B, and the
    RigExecBlendPointFrames owns `rigExec:joints`. The solvers still name the
    same joints, which for the IK is only a rest reference for measuring bone
    lengths. weight 0 = FK, weight 1 = IK.

    This works with zero extra prims only because the joints carry the
    solver's own basis (see limb_frames.py). Bound to Maya's frames, the
    blend would reproduce the right-limb 180-degree roll at weight 1 and pass
    through a 90-degree garbage frame at weight 0.5.
    """
    made = []
    for tag, rname, mname, ename in LIMBS:
        missing = [n for n in (rname, mname, ename) if n not in worlds]
        if missing:
            print("  %s: SKIPPED, missing %s" % (tag, ", ".join(missing)))
            continue

        rw, mw, ew = worlds[rname], worlds[mname], worlds[ename]
        joints = [by_name[rname], by_name[mname], by_name[ename]]

        # The limb's ROOT control is nested under its girdle control when
        # one exists, so the limb rides the clavicle or the pelvis the way
        # Maya's limb nul does (`rig_bits.nxt:/arm/limb/create/
        # clavicle_connect` parents the limb group under the clavicle
        # control). It has to be NESTING, not a constraint: solver-posed
        # joints are excluded from constraint propagation
        # (`rigEvaluator.cpp:7414` -- hierarchicalProviders are the
        # namespace-posed ones only), which is why driving the girdle
        # control moved the clavicle joint but left the shoulder at 0.00 cm.
        girdle = (girdle_parents or {}).get(tag)
        g_rest = (control_world_rest(stage, girdle.path)
                  if girdle is not None else Gf.Matrix4d(1.0))
        if girdle is not None:
            root_ctl = builder.add_control(
                "%s_root" % tag, flatten(rw * g_rest.GetInverse()), girdle)
        else:
            root_ctl = builder.add_control("%s_root" % tag, flatten(rw))
        pole_at = poles.get(tag) or pole_position(
            rw.ExtractTranslation(), mw.ExtractTranslation(),
            ew.ExtractTranslation())
        pole_rest = Gf.Matrix4d(1.0).SetTranslate(pole_at)
        if girdle is not None and not ik_handles_world:
            # Opt-in (`--ik-handles-follow-girdle`): the EFFECTOR and
            # POLE ride the girdle as well, not just the chain root.
            # With only the root nested, moving clavicle_l_ctl 10 cm
            # carried every FK control and every arm joint by 10.000 cm
            # in FK -- but in IK the wrist stayed at 0.000 and the
            # elbow went to 12.339, because the effector was still
            # pinned in asset space while the chain root had moved.
            # That is the planted-foot behaviour Maya has and the
            # default here now; this branch is the departure, for a
            # rig where the hand or foot should ride the girdle. It
            # was the default once, and swivelling the hips then
            # carried the planted IK feet 44.15 cm.
            eff_ctl = builder.add_control(
                "%s_ik" % tag, flatten(ew * g_rest.GetInverse()),
                girdle)
            pole_ctl = builder.add_control(
                "%s_pv" % tag,
                flatten(pole_rest * g_rest.GetInverse()), girdle)
        else:
            eff_ctl = builder.add_control("%s_ik" % tag, flatten(ew))
            pole_ctl = builder.add_control("%s_pv" % tag,
                                          flatten(pole_rest))

        ik = builder.add_two_bone_ik("%s_twoBoneIk" % tag, root_ctl,
                                     eff_ctl, pole_ctl)
        ik.set_joints(joints)
        # The root rest Y is now the bend direction, so the degenerate-pole
        # fallback holds the rest pose with no preferred bend.
        ik.set_preferred_bend_radians(0.0)

        blend = None
        if with_blend:
            chain_names = (rname, mname, ename)
            fk_ctls = add_fk_controls(
                builder, worlds, chain_names,
                lambda n: "%s_fk_%s" % (tag, n), fk_follow_parent,
                root_parent=girdle, stage=stage)
            fk = builder.add_fk_chain("%s_fkChain" % tag, fk_ctls, joints)
            if fk_follow_parent:
                fk.set_control_space("parentRelative")
            # RigExecBlendPointFrames: weight 0 = input A (FK), 1 = input B
            # (IK). Maya ships the arms in FK and the legs in IK --
            # body_rig.nxt:/apply/control_defaults sets
            # arm_ikfk_default="fk" and leg_ikfk_default="ik" (note Maya's
            # own `ikfk` attribute is inverted: 1 = FK there).
            default = BLEND_DEFAULTS.get(tag, 1.0)
            blend = builder.add_blend_point_frames(
                "%s_ikfk" % tag, fk, ik, default)
            blend.set_joints(joints)

            # The dial: one animatable float per limb, on the root control.
            dial = stage.GetPrimAtPath(root_ctl.path).CreateAttribute(
                IKFK_AVAR, Sdf.ValueTypeNames.Float)
            dial.Set(default)
            stage.GetPrimAtPath(blend.path).GetAttribute(
                "inputs:weight").SetConnections(
                    [Sdf.Path("%s.%s" % (root_ctl.path, IKFK_AVAR))])

        made.append((tag, ik, eff_ctl, ename, blend))
        print("  %-6s root=%-16s mid=%-14s end=%-14s pole=(%.1f %.1f %.1f)%s"
              % (tag, rname, mname, ename, pole_at[0], pole_at[1], pole_at[2],
                 ("  + FK chain, blend, %s_root.%s = %.0f"
                  % (tag, IKFK_AVAR, BLEND_DEFAULTS.get(tag, 1.0)))
                 if with_blend else ""))
    return made


# The FK control sets the Maya build creates, as contiguous parent chains.
# Arms/legs come from rig_bits.nxt:/limb (`joint_list`); spine and neck are
# driven by an ikSpline in Maya (core/maya/rig/spline.py) whose duplicated
# chain point/orient-constrains the bind joints -- an FK chain is the
# straightforward RigExec stand-in for now, and is noted as such.
FK_CHAINS = [
    ("fk_arm_l", ["shoulder_l_bind", "elbow_l_bind", "wrist_l_bind"]),
    ("fk_arm_r", ["shoulder_r_bind", "elbow_r_bind", "wrist_r_bind"]),
    ("fk_leg_l", ["thigh_l_bind", "knee_l_bind", "ankle_l_bind"]),
    ("fk_leg_r", ["thigh_r_bind", "knee_r_bind", "ankle_r_bind"]),
    ("fk_spine", ["spine_0_bind", "spine_1_bind", "spine_2_bind",
                  "spine_3_bind", "spine_4_bind", "spine_5_bind",
                  "chest_bind"]),
    ("fk_neck", ["neck_base_bind", "neck_0_bind", "neck_1_bind",
                 "neck_2_bind", "neck_3_bind", "skull_bind"]),
]


def add_fk_controls(builder, worlds, joints, name_of, follow_parent,
                    root_parent=None, stage=None):
    """The controls for one FK chain, flat or nested.

    Nesting is what makes an FK control FOLLOW its parent control -- the
    shoulder moving the elbow control, the elbow moving the wrist -- which
    is how Maya behaves and what the animator expects. It works because a
    control prim's frame is composed through its namespace ancestor, so a
    nested control already publishes `W_parent . restLocal . delta`; the
    chain solver is then told (`set_control_space("parentRelative")`) to
    treat every element as a chain root so it does not compose the parent a
    second time. Nesting alone, with the solver left in `world` mode,
    applies the parent's rotation TWICE.

    A nested control's rest is therefore parent-local, exactly like a
    joint's: `world * inverse(parentWorld)`, the same rule
    `limb_frames.py` uses.
    """
    controls = []
    for i, j in enumerate(joints):
        if follow_parent and i:
            rest = worlds[j] * worlds[joints[i - 1]].GetInverse()
            controls.append(builder.add_control(name_of(j), flatten(rest),
                                                controls[-1]))
        elif not i and root_parent is not None:
            # Element 0 under the girdle control. Safe in BOTH control
            # spaces: the solver treats the first element as a chain root
            # either way, so nesting it adds the girdle's motion without
            # anything composing it twice.
            p_rest = control_world_rest(stage, root_parent.path)
            controls.append(builder.add_control(
                name_of(j), flatten(worlds[j] * p_rest.GetInverse()),
                root_parent))
        else:
            controls.append(builder.add_control(name_of(j),
                                                flatten(worlds[j])))
    return controls


def add_fk_chains(builder, worlds, joint_prim_paths, parent, chains=None,
                  follow_parent=False):
    """One RigExecFkChain per control set, with a control per joint.

    RigExecFkChain is strictly linear -- element i's parent is element i-1
    (`libs/rigExec/computations.cpp:566-568`) -- so each joint list has to be
    a contiguous parent chain, which is asserted rather than assumed.

    The controls go under the rig's /Controls scope with no RigExec ancestor,
    so their rest is asset space and takes a world matrix. Only the ORDER of
    `rigExec:controls` establishes the chain; they are not nested.

    Unlike the two-bone solver, FK imposes no aim convention: it composes
    `W_i = W_parent . A_i` from each control's rest-to-pose delta
    (`libs/rigExecMath/solvers.h:25-33`), so the mirrored right limb needs no
    special handling here.
    """
    made = []
    for name, joints in (chains or FK_CHAINS):
        missing = [j for j in joints if j not in worlds]
        if missing:
            print("  %s: SKIPPED, missing %s" % (name, ", ".join(missing)))
            continue
        broken = [(joints[i - 1], joints[i]) for i in range(1, len(joints))
                  if parent.get(joints[i]) != joints[i - 1]]
        if broken:
            raise SystemExit(
                "%s is not a contiguous parent chain: %s"
                % (name, ", ".join("%s is not the parent of %s" % b
                                   for b in broken)))

        controls = add_fk_controls(builder, worlds, joints,
                                   lambda j: "%s_fk" % j, follow_parent)
        chain = builder.add_fk_chain(
            name, controls, [joint_prim_paths[j] for j in joints])
        if follow_parent:
            chain.set_control_space("parentRelative")
        made.append((name, chain, joints, controls))
        print("  %-10s %d controls -> %s%s"
              % (name, len(controls), " > ".join(joints),
                 "  (nested, follows parent)" if follow_parent else ""))
    return made


# The spine chain the Maya build drives with an ikSpline
# (rig_bits.nxt:/spine -> core/maya/rig/spline.py). `hips_bind` is the parent
# and is NOT part of the chain: it is the root the spline hangs from.
SPINE_JOINTS = ["spine_0_bind", "spine_1_bind", "spine_2_bind",
                "spine_3_bind", "spine_4_bind", "spine_5_bind",
                "chest_bind"]
SPINE_ROOT = "hips_bind"

NECK_JOINTS = ["neck_0_bind", "neck_1_bind", "neck_2_bind", "neck_3_bind",
               "skull_bind"]
NECK_ROOT = "neck_base_bind"

# Maya's per-joint volume weights (spline.py lines 329-342), the profile
# that makes the middle of the chain thin most under stretch. Parallel to
# the joint lists above, so the full chain is used -- including chest_bind
# and skull_bind, because RigExecSplineIk takes its four CVs from joints
# [0], [1], [-2], [-1], which is exactly Maya's curve through spine_0,
# spine_1, spine_5, chest.
SPINE_VOLUME_WEIGHTS = [0.1429, 0.2857, 0.4286, 0.5, 0.3571, 0.2143,
                        0.0714]
NECK_VOLUME_WEIGHTS = [0.16, 0.32, 0.4, 0.24, 0.08]


# Maya's pivotHeight dial: 0..10 remapped onto 0..arcLength of the ik curve
# (rig_bits.nxt /spine/controls/ik/chest_pivot, /spine/controls/ik/hip_pivot,
# /neck/head_pivot_connect: `remapValue` inputMax 10, outputMax
# curveInfo.arcLength). The shipped value is 3.5 on all three (attrs
# chest_pivot, hip_swivel_pivot, head_pivot), i.e. 35% of the chain.
PIVOT_HEIGHT = 3.5

# The length floor the RigExecSplineIk solver holds, per chain, as a
# fraction of the rest root->end chord. Maya has NO such floor -- its
# ikSpline scales every joint's tx by curveInfo.arcLength / restLength
# (spline.py 183-204) and the chain crumples to whatever the clusters
# give it; `preserve_length` (spline.py 345-512) only compensates the
# ROTATION of torso/chest/chest_ik, scaling the curve back to rest length
# from cv0 along the hips' Y. The floor is the requested departure: below
# it the end CVs are held ahead of the root along the root's own chain
# axis, so the chain neither shrinks past it nor folds back on itself. It
# is `inputs:minLengthRatio` on the solver prim, animatable per shot, and
# 0 switches it off. The fold-free threshold is chord > |cv1 - cv0| +
# |cv3 - cv2| (the two tangent legs), printed at build time.
MIN_LENGTH_RATIO = {"spine": 0.5, "neck": 0.5}


def hide_guide(stage, ctl):
    """Switch a helper control's guide off, build_foot.py's convention:
    a zero guide:scale, never purpose=guide, which the children inherit
    and which would hide every animator control nested underneath."""
    prim = stage.GetPrimAtPath(ctl.path)
    for axis in "XYZ":
        prim.GetAttribute("guide:scale%s" % axis).Set(0.0)
    return prim


def quadratic_bspline_arc_length(cvs, samples=1024):
    """Arc length of the open degree-2 four-CV B-spline the solver builds
    (libs/rigExecMath/splineIk.h): span 0 is the quadratic Bezier
    (cv0, cv1, (cv1 + cv2) / 2), span 1 is ((cv1 + cv2) / 2, cv2, cv3).
    Maya's `curveInfo.arcLength` of the same degree-2 curve through the
    same four joints is what remaps pivotHeight in rig_bits.nxt. A chord
    polyline at 1024 samples per span is within 1e-6 relative on a spine."""
    mid = (cvs[1] + cvs[2]) * 0.5
    total = 0.0
    for q0, q1, q2 in ((cvs[0], cvs[1], mid), (mid, cvs[2], cvs[3])):
        prev = q0
        for i in range(1, samples + 1):
            t = i / float(samples)
            s = 1.0 - t
            p = q0 * (s * s) + q1 * (2.0 * s * t) + q2 * (t * t)
            total += (p - prev).GetLength()
            prev = p
    return total


def pivot_frame(anchor_world, toward, distance):
    """World rest of a pivot control: `distance` cm from `anchor_world`'s
    origin along the straight line to the point `toward`, local +Y on that
    line (so `default:ty` slides the pivot along the chain and a circle
    guide, normal +Y, is a belt around the body), local X the anchor's own
    X made perpendicular, Z closing a right-handed basis.

    Maya puts the head pivot on exactly this line (head_pivot_connect aims
    headPivot_aim_nul at the neck control and slides along it); the chest
    and hip pivots go along the dominant WORLD axis toward the other end
    (get_distance_vector on the chest->hips vector), which for an upright
    spine is the same line to within the spine's lean. The chord is used
    for all three here because the RigExec controls are joint-oriented
    rather than world-aligned like Maya's, so a world axis would not be
    expressible as a control-local one.
    """
    o = anchor_world.ExtractTranslation()
    y = (toward - o).GetNormalized()
    x = Gf.Vec3d(anchor_world[0][0], anchor_world[0][1], anchor_world[0][2])
    x = x - y * Gf.Dot(x, y)
    if x.GetLength() < 1e-6:
        x = Gf.Vec3d(anchor_world[2][0], anchor_world[2][1],
                     anchor_world[2][2])
        x = x - y * Gf.Dot(x, y)
    x.Normalize()
    z = Gf.Cross(x, y)
    p = o + y * distance
    m = Gf.Matrix4d(1.0)
    m.SetRow(0, Gf.Vec4d(x[0], x[1], x[2], 0.0))
    m.SetRow(1, Gf.Vec4d(y[0], y[1], y[2], 0.0))
    m.SetRow(2, Gf.Vec4d(z[0], z[1], z[2], 0.0))
    m.SetRow(3, Gf.Vec4d(p[0], p[1], p[2], 1.0))
    return m


def add_pivot_control(builder, stage, name, child_name, child_world, toward,
                      arc_length, height=PIVOT_HEIGHT, parent=None):
    """A control that rotates `child_name` about an offset pivot.

    `parent` nests the PIVOT under another control (the hips body control
    for the hip swivel, torso for the chest, the neck control for the
    head), with a parent-local rest. The child's authored `default:space`
    stays its WORLD rest: the composition rule below cancels the pivot's
    default either way, and with a parent P the child composes to
    D_child * inverse(D_pivot) * (D_pivot * inverse(D_parent) * P_parent)
    = D_child * inverse(D_parent) * P_parent -- it rides the parent
    rigidly and its own zero pose is untouched. verify_spine.py's pivot
    checks (dial moves only the pivot) run against the nested form.

    Maya gives one control a `rotatePivot` (`.rp`) driven by pivotHeight
    (rig_bits.nxt chest_pivot / hip_pivot / head_pivot_connect), so the
    chest control's rotation happens about a point 35% down the spine. A
    RigExecControl rotates about its own frame origin and has no pivot
    attribute, so the pivot is expressed with the xformable's OWN
    composition rule (schema.usda RigExecXformable: pose = avars *
    posed:defaultSpace * inverse(parent:defaultSpace) * parent:space):

      * `name` (the pivot) is a top-level control whose rest sits AT the
        pivot point -- rotating it rotates everything nested under it
        about that point, exactly Maya's `.rp`;
      * `child_name` is nested under it with a parent-local rest, so it,
        the solver end it drives and every control nested under it (the
        clavicles) all swing about the pivot;
      * the child's `default:space` is AUTHORED to its world rest, which
        pins its zero pose. The pivot's zero pose can then be moved with
        `default:ty` (cm along the chain; 0 = the build pivot) WITHOUT
        moving the child: at zero avars the child composes to
        D_child * inverse(D_pivot) * D_pivot = D_child regardless of where
        the pivot's default went, and a pivot rotation R composes to
        D_child * inverse(D_pivot) * R * D_pivot, a rotation about the
        moved pivot. That is Maya's animatable pivotHeight. `default:ty`
        is a schema double, so it takes time samples like an avar.

    Returns (pivot, child) handles. The pivot carries `pivot:height`
    (the Maya dial value used), `pivot:arcLength` and `pivot:restOffset`
    (cm) as custom attributes for reference.
    """
    distance = arc_length * height / 10.0
    pw = pivot_frame(child_world, toward, distance)
    if parent is not None:
        p_rest = control_world_rest(stage, parent.path)
        pivot = builder.add_control(name, flatten(pw * p_rest.GetInverse()),
                                    parent)
    else:
        pivot = builder.add_control(name, flatten(pw))
    child = builder.add_control(child_name, flatten(child_world * pw.GetInverse()),
                                pivot)
    stage.GetPrimAtPath(child.path).GetAttribute("default:space").Set(
        Gf.Matrix4d(child_world))
    pprim = stage.GetPrimAtPath(pivot.path)
    pprim.GetAttribute("default:ty").Set(0.0)
    for attr, value in (("pivot:height", height),
                        ("pivot:arcLength", arc_length),
                        ("pivot:restOffset", distance)):
        pprim.CreateAttribute(attr, Sdf.ValueTypeNames.Double, True).Set(
            float(value))
    return pivot, child


def _aim_vector_toward(frame_world, target_world_point):
    """A world direction expressed in a frame's rest axes, unit length:
    what an aim constraint's inputs:aimVector has to be so that the
    constraint is already satisfied at rest."""
    rot = Gf.Matrix4d(frame_world)
    rot.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
    d = target_world_point - frame_world.ExtractTranslation()
    return rot.GetInverse().TransformDir(d).GetNormalized()


def add_spline_ik_chain(builder, stage, worlds, joint_prim_paths, tag, joints,
                        volume_weights, mid_index=None, mid="follow",
                        pivots=("root", "end"), pivot_height=PIVOT_HEIGHT,
                        min_length_ratio=None, root_parent=None,
                        end_parent=None):
    """A control-driven spline spine, using the RigExecSplineIk solver.

    `root_parent` / `end_parent` nest the chain's root and end controls
    (or their pivot controls, when they have one) under another control,
    parent-local rest, so they FOLLOW it. `end_parent="root"` nests the
    end under this chain's own root control. That is the Maya parenting:

      * spine: hip_swivel (root) under hips_gimbal, chest (end pivot)
        under torso under hips_gimbal (`/spine/controls/ik/hip_swivel`
        parent='hips_gimbal'; `/spine/controls/fk/chest` parent=torso;
        `/spine/controls/fk/torso` parent='hips_gimbal') -- see body.py;
      * neck: the neck control's nul is parent-constrained to chest_top
        (`/neck/head_pivot_connect` line 97, mo=1), and the head control
        is parented under the neck control (`/neck/controls/head`
        parent=${neck_control}). So root_parent=spine_end_ctl and
        end_parent="root".

    Left top-level (the original form), the neck did not follow the
    chest at all, and the neck JOINTS were then moved by whatever wrote
    an ancestor after them: swivelling the hips 30 degrees carried
    neck_0_bind 25.64 cm and skull_bind 33.08 cm while chest_bind held
    at 0.00. The nesting is what ties the neck to the chest; the bind
    chain ORDER (see build()) is what stops the propagation.

    This replaces the earlier RigExecRibbon attempt, which could not work:
    the ribbon reads its driver curve as NATIVE scene data, so a curve
    reshaped by control-driven movers is invisible to it. Measured both
    ways -- curve animated by native timeSamples: joints move 1.1 to 12.0
    cm, cleanly graded; the same curve reshaped by movers: 0.000 cm. A
    solver that takes CONTROL frames and builds the spline internally is
    pose-phase throughout, so the problem disappears.

    The solver takes its four CVs from joints [0], [1], [-2], [-1] --
    Maya's degree-2 curve through spine_0, spine_1, spine_5 and chest --
    so the whole chain is passed, terminal joint included. cv0/cv1 ride
    the root control, cv2/cv3 the end control (Maya: clusters[0:2] under
    hip_swivel_grp, clusters[2:] under chest_ik, rig_bits.nxt
    /spine/controls/ik/hip_swivel/connect and /chest_top/connect).

    `mid` selects what shapes the interior:

      "follow"  Maya's spine_mid (/spine/mid_ik): a control that sits at
                the MIDPOINT of the root and end controls and aims at the
                end, but whose auto-placement contributes nothing to the
                curve -- Maya localizes its cluster to the `_follow` node
                (rig_cluster.localize(cluster, spine_mid_follow)), so only
                the control's LOCAL transform deforms cv1/cv2. Here:
                `<tag>_mid_follow` is a hidden helper carried by a
                translation-only RigExecParentConstraint from the root and
                end controls at 0.5/0.5 with per-source maintained offsets
                -- algebraically the solver's own follow point, 0.5 *
                rootMap(midRest) + 0.5 * endMap(midRest) (splineIk.cpp
                RigExecSplineIkPoseCvs) -- and a RigExecAimConstraint at
                the end control; `<tag>_mid_ctl` is nested under it at
                identity. The solver reads the nested control, so its
                offset from the follow point is exactly the animator's
                local translation: zero until the avars are touched
                (measured 3.6e-15 cm against the mid-less solve), and the
                mid inherits neither control's rotation (the solver ignores
                mid rotation anyway; the helper only aims).
                Not reproduced: Maya's follow_twist averaging of the start
                and end twist onto the mid control's display orientation.

      "aim"     Maya's neck (/neck/head_pivot_connect lines 343-347): no
                mid control at all; cluster[1] is parented under
                `neck_head_aim`, a joint AT the neck control's origin
                aim-constrained at the head. Here the SOLVER does it:
                `rigExec:rootTangent = "aim"` turns cv1 about cv0 from the
                root's posed chain axis onto the chord to the end CV --
                after the length floor, so a head driven onto or past the
                neck root cannot flip the aim (an aim-constrained helper
                control was tried first and did exactly that at 120%
                compression, because the floor then lifted the end along
                the helper's backward axis). It adds no twist. The solver's
                mid is a hidden `<tag>_cv1` nested rigidly under the root
                with midFollowWeight 0, so its offset is identically zero
                and cv2 stays with the end control alone -- which a
                mid-driven aim could not give, since the solver adds the
                mid offset to cv2 as well as cv1.

    `pivots` names which of the root/end controls get an offset rotation
    pivot (see add_pivot_control): "root" makes `<tag>_root_pivot` the
    parent of `<tag>_root_ctl` (Maya's hip_swivel with hip_swivel_pivot),
    "end" makes `<tag>_end_pivot` the parent of `<tag>_end_ctl` (chest
    with chest_pivot; head with head_pivot). Maya's neck control has no
    pivotHeight, so the neck passes ("end",).

    `inputs:roll` and `inputs:twist` are DEGREES (the solver converts;
    only `preferredBendRadians` is suffixed for radians in this codebase).
    """
    from girdle import offset_in_source_space as _offset_in_source_space

    n = len(joints)
    mid_joint = joints[mid_index if mid_index is not None else n // 2]
    root_w, end_w = worlds[joints[0]], worlds[joints[-1]]
    cvs = [worlds[j].ExtractTranslation()
           for j in (joints[0], joints[1], joints[-2], joints[-1])]
    arc = quadratic_bspline_arc_length(cvs)
    chord = (cvs[3] - cvs[0]).GetLength()
    fold_free = ((cvs[1] - cvs[0]).GetLength() +
                 (cvs[3] - cvs[2]).GetLength()) / chord

    def nested(name, world, parent):
        if parent is None:
            return builder.add_control(name, flatten(world))
        p_rest = control_world_rest(stage, parent.path)
        return builder.add_control(name, flatten(world * p_rest.GetInverse()),
                                   parent)

    root_pivot = end_pivot = None
    if "root" in pivots:
        root_pivot, root_ctl = add_pivot_control(
            builder, stage, "%s_root_pivot" % tag, "%s_root_ctl" % tag,
            root_w, cvs[3], arc, pivot_height, parent=root_parent)
    else:
        root_ctl = nested("%s_root_ctl" % tag, root_w, root_parent)
    if end_parent == "root":
        end_parent = root_ctl
    if "end" in pivots:
        end_pivot, end_ctl = add_pivot_control(
            builder, stage, "%s_end_pivot" % tag, "%s_end_ctl" % tag,
            end_w, cvs[0], arc, pivot_height, parent=end_parent)
    else:
        end_ctl = nested("%s_end_ctl" % tag, end_w, end_parent)

    solver_root = root_ctl
    mid_ctl = None
    if mid == "follow":
        mid_w = worlds[mid_joint]
        follow = builder.add_control("%s_mid_follow" % tag, flatten(mid_w))
        hide_guide(stage, follow)
        mid_ctl = builder.add_control("%s_mid_ctl" % tag,
                                      flatten(Gf.Matrix4d(1.0)), follow)
        def make_movers(builder=builder, follow=follow, root_ctl=root_ctl,
                        end_ctl=end_ctl, mid_w=mid_w, root_w=root_w,
                        end_w=end_w, target=cvs[3]):
            chain = builder.new_mover_chain("%s_mid_follow" % tag)
            # Reverse add order: the aim goes in FIRST so it executes
            # LAST, after the position has landed (it reads the helper's
            # own origin to find its direction).
            aim = chain.add_aim_constraint("%s_mid_follow_aim" % tag,
                                           follow, [end_ctl])
            v = _aim_vector_toward(mid_w, target)
            aim.set_aim_vector(v[0], v[1], v[2])
            aim.set_world_up_type("none")
            pc = chain.add_parent_constraint("%s_mid_follow_pos" % tag,
                                             follow, [root_ctl, end_ctl],
                                             [0.5, 0.5])
            offsets = [_offset_in_source_space(mid_w, src)
                       for src in (root_w, end_w)]
            pc.set_translation_offsets([t for t, _ in offsets])
            pc.set_rotation_offsets([r for _, r in offsets])
            pc.set_affect_rotation(False, False, False)
            return chain
        solver_mid = mid_ctl
        follow_weight = 0.5
        mid_note = ("mid=%s_mid_ctl nested under a follow helper "
                    "(midpoint of root/end, aimed at end)" % tag)
    elif mid == "aim":
        # The solver does the aiming (rigExec:rootTangent = "aim") against
        # the FLOORED end, so no helper chain is needed and a head driven
        # past the neck root cannot flip the aim. The solver still needs a
        # mid control: a hidden helper nested rigidly under the root at
        # cv1's rest, with midFollowWeight 0, whose offset is identically
        # zero -- cv2 stays with the end control alone.
        cv1_ctl = builder.add_control(
            "%s_cv1" % tag, flatten(worlds[joints[1]] * root_w.GetInverse()),
            root_ctl)
        hide_guide(stage, cv1_ctl)
        make_movers = None
        solver_mid = cv1_ctl
        follow_weight = 0.0
        mid_note = ("no mid control; cv1 aimed at the end by the solver "
                    "(rootTangent=aim, Maya neck_head_aim)")
    else:
        raise ValueError("mid must be 'follow' or 'aim', got %r" % (mid,))

    # The solver drives a DUPLICATE chain, not the bind joints, and the
    # bind joints are then constrained from it with a maintained offset --
    # which is what Maya does (`spline.py` duplicates, solves, then
    # point/orient-constrains back with mo=1). It is not ceremony: a
    # degree-2 curve interpolates only its first and last CV, so interior
    # joints carry a residual (measured 0.39-0.90 cm on the spine) and the
    # tip overshoots the curve end by chain-minus-curve (0.118 cm). Driving
    # the bind joints directly left the rest pose 1.386 cm out at the brow,
    # because those residuals compound down the face. The offsets absorb
    # them exactly.
    dup_paths = []
    dup_parent = None
    for i, j in enumerate(joints):
        local = (Gf.Matrix4d(worlds[j]) if i == 0
                 else worlds[j] * worlds[joints[i - 1]].GetInverse())
        dup = builder.add_joint("%s_ik_%s" % (tag, j), flatten(local),
                                dup_parent)
        dup_paths.append(dup.path)
        dup_parent = dup

    solver = builder.add_spline_ik("%s_splineIk" % tag, solver_root,
                                   solver_mid, end_ctl, dup_paths)
    solver.set_volume_weights(volume_weights)
    # "curve" means restArcLength is the curve's own length, so ratio == 1
    # at rest -- Maya reads curveInfo.arcLength, and the consequence (the
    # tip overshooting the curve end by ~0.12 cm) is what its mo=1
    # constraints absorb.
    solver.set_rest_length("curve")
    solver.set_preserve_volume(1.0)
    solver.set_mid_follow_weight(follow_weight)

    if min_length_ratio is None:
        min_length_ratio = MIN_LENGTH_RATIO.get(tag, 0.0)
    if mid == "aim":
        if hasattr(solver, "set_root_tangent"):
            solver.set_root_tangent("aim")
        else:
            stage.GetPrimAtPath(solver.path).CreateAttribute(
                "rigExec:rootTangent", Sdf.ValueTypeNames.Token, True,
                Sdf.VariabilityUniform).Set("aim")
            mid_note += (" -- AUTHORED BUT INERT: this rigexec build has no "
                         "set_root_tangent, cv1 rides the root rigidly")
    if hasattr(solver, "set_min_length_ratio"):
        solver.set_min_length_ratio(min_length_ratio)
        floor_note = "solver floor"
    else:
        # A build of the bindings that predates the floor: author the
        # attribute plain so the rig is right once the solver is rebuilt,
        # and say so rather than silently building without it.
        stage.GetPrimAtPath(solver.path).CreateAttribute(
            "inputs:minLengthRatio", Sdf.ValueTypeNames.Double).Set(
                float(min_length_ratio))
        floor_note = ("solver floor AUTHORED BUT INERT: this rigexec "
                      "build has no set_min_length_ratio")

    print("  %-6s %d joints; root=%s%s end=%s%s; %s; squash weights %s"
          % (tag, n, root_ctl.name,
             " (pivot %s)" % root_pivot.name if root_pivot else "",
             end_ctl.name,
             " (pivot %s)" % end_pivot.name if end_pivot else "",
             mid_note,
             "[%s]" % ", ".join("%.3g" % w for w in volume_weights)))
    print("  %-6s curve arc %.2f cm, chord %.2f cm; pivots at %.0f%% = "
          "%.2f cm; length floor %.2f of chord (%s); fold-free above "
          "%.2f of chord"
          % (tag, arc, chord, pivot_height * 10.0,
             arc * pivot_height / 10.0, min_length_ratio, floor_note,
             fold_free))
    # The helper movers are NOT created here. A mover chain executes in
    # reverse creation order, and these read nothing but plain controls
    # while the solvers read what they write; created here (first) they
    # would execute last, after the `_to_bind` chains that read the
    # solvers -- a pose dependency cycle the compiler rejects (it did:
    # "pose dependency cycle among: ... spine_mid_follow ... neck_root_aim
    # ... spine_to_bind ... Solvers/spine_splineIk"). build() calls
    # `make_movers` for every spline chain LAST, so they execute FIRST.
    return dict(tag=tag, solver=solver, joints=joints, dup=dup_paths,
                controls=(root_ctl, mid_ctl, end_ctl),
                pivots=dict(root=root_pivot, end=end_pivot),
                arc_length=arc, chord=chord, fold_free=fold_free,
                make_movers=make_movers)


def add_spline_chain(builder, stage, worlds, joint_prim_paths, tag,
                     joints, root, sample_count=96,
                     geom_scope="/Biped/Geom"):
    """A curve-driven spline chain: RigExec's answer to Maya's ikSpline.

    Maya (`core/maya/rig/spline.py`) duplicates the chain, runs an
    `ikHandle -sol ikSplineSolver` on the duplicate, and point/orient
    constrains the bind joints back from it. RigExec expresses the same idea
    directly: `RigExecRibbon` samples a driver curve and poses its
    `rigExec:joints` along it, with `rigExec:jointElements` picking which
    sample each joint binds. So no duplicate chain is needed -- the ribbon
    writes the bind joints.

    Three pieces, mirroring the Maya setup:

      * a `BasisCurves` driver, cubic bspline, whose CVs start on the chain;
      * one `RigExecMatrixMover` + `RigExecStaticWeight` per control over
        the curve's `points`, which is the analogue of Maya's skinCluster on
        the spline curve -- moving a control reshapes the curve and the
        joints follow;
      * a `RigExecTwistDistribution` between the root and end controls fed
        into the ribbon's `rigExec:twistFrames`, which is what spreads twist
        along the chain instead of snapping it at the ends.

    `parameterization = "arcLength"` keeps the samples evenly spaced along
    the curve as it bends, and `frameTransport = "rotationMinimizing"` is
    the analogue of the ikSpline's advanced twist: it carries the frame
    along the curve without letting it spin.
    """
    n = len(joints)
    origins = [worlds[j].ExtractTranslation() for j in joints]

    # Driver CVs ARE the joint rest positions, with the ends duplicated:
    # a catmullRom curve interpolates every CV except its first and last,
    # which act as tangent handles. A bspline would not pass through any of
    # them -- the first attempt used one and the solved spine sat inside the
    # rest span at both ends.
    cvs = [origins[0]] + list(origins) + [origins[-1]]
    cv_count = len(cvs)

    UsdGeom.Scope.Define(stage, geom_scope)
    curve_path = "%s/%s_curve" % (geom_scope, tag)
    curve = UsdGeom.BasisCurves.Define(stage, curve_path)
    curve.CreateTypeAttr(UsdGeom.Tokens.cubic)
    curve.CreateBasisAttr(UsdGeom.Tokens.catmullRom)
    curve.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curve.CreateCurveVertexCountsAttr(Vt.IntArray([cv_count]))
    curve.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(p) for p in cvs]))
    curve.CreateWidthsAttr(Vt.FloatArray([0.5] * cv_count))
    curve.GetWidthsAttr().SetMetadata("interpolation", "vertex")
    curve.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    points_target = curve_path + ".points"

    # Controls: root, a mid/waist, and the end. Three is what the Maya rig
    # gives the animator (hip_pivot / hip_swivel, a waist, chest_top).
    root_ctl = builder.add_control("%s_root_ctl" % tag,
                                   flatten(worlds[root]))
    mid_ctl = builder.add_control("%s_mid_ctl" % tag,
                                  flatten(worlds[joints[n // 2]]))
    end_ctl = builder.add_control("%s_end_ctl" % tag,
                                 flatten(worlds[joints[-1]]))

    # A 3-control spline weighting: the ends own their CV outright and the
    # interior CVs blend, which is the standard cluster layout Maya's curve
    # skinCluster produces.
    ramps = {root_ctl: [], mid_ctl: [], end_ctl: []}
    for i in range(cv_count):
        u = i / float(cv_count - 1)
        mid_w = max(0.0, 1.0 - abs(u - 0.5) * 2.0)
        root_w = max(0.0, 1.0 - u * 2.0)
        end_w = max(0.0, (u - 0.5) * 2.0)
        total = root_w + mid_w + end_w
        ramps[root_ctl].append(root_w / total)
        ramps[mid_ctl].append(mid_w / total)
        ramps[end_ctl].append(end_w / total)

    chain = builder.new_mover_chain("%s_curve_drive" % tag, points_target)
    # Reversed so the movers execute root-first (a chain applies in reverse
    # add order).
    for ctl in (end_ctl, mid_ctl, root_ctl):
        vals = ramps[ctl]
        w = builder.add_static_weight("%s_%s_w" % (tag, ctl.name),
                                      points_target, vals,
                                      list(range(cv_count)), 0.0)
        chain.add_matrix_mover("%s_%s_mover" % (tag, ctl.name), ctl, w,
                               points_target, "final")

    # The ribbon distributes `sampleCount` frames UNIFORMLY (arcLength or
    # parametric are the only options), but these chains are not evenly
    # spaced -- spine_0->spine_1 is 7.77cm against 6.49cm for the rest. So
    # oversample and let `rigExec:jointElements` bind each joint to the
    # frame nearest its own rest arc length, which is exactly what that
    # attribute exists for. Quantisation error is chain_length/(samples-1).
    seg = [0.0]
    for i in range(1, n):
        seg.append(seg[-1] + (origins[i] - origins[i - 1]).GetLength())
    total = seg[-1] or 1.0
    samples = max(sample_count, n)
    elements = [min(samples - 1, int(round(seg[i] / total * (samples - 1))))
                for i in range(n)]
    quant = total / float(samples - 1)

    twist = builder.add_twist_distribution("%s_twist" % tag, root_ctl,
                                           end_ctl, samples)
    twist.set_weights([i / float(samples - 1) for i in range(samples)])

    # The ribbon drives a DUPLICATE chain, not the bind joints -- which is
    # what `spline.py` does: it duplicates the chain, runs the ikSpline on
    # the duplicate, then point/orient-constrains the bind joints back with
    # `mo=1`. That indirection is not incidental. Maya's spline curve is
    # degree-2 through only four CVs and deliberately does NOT pass through
    # the interior joints, so the solved chain never sits on the bind chain
    # at rest; the maintained offset absorbs the difference. Binding the
    # ribbon straight to the bind joints (the first attempt here) inherited
    # both that positional error and a constant 90-degree frame difference
    # between the ribbon's transported frames and Maya's joint orientations.
    #
    # The offsets cannot be computed from the authored rests -- they are the
    # difference between each bind joint's rest and the ribbon's ACTUAL
    # solved frame at rest, so the caller evaluates once and fills them in
    # (see finish_spline_chains). That is precisely what `mo=1` captures.
    # Duplicate chain, rests mirroring the bind chain so the ribbon measures
    # the same lengths. Chained to each other, so each local is relative to
    # the previous duplicate rather than to the bind hierarchy.
    dup_paths = []
    dup_parent = None
    for i, j in enumerate(joints):
        if i == 0:
            local = Gf.Matrix4d(worlds[j])
        else:
            local = worlds[j] * worlds[joints[i - 1]].GetInverse()
        dup = builder.add_joint("%s_ik_%s" % (tag, j), flatten(local),
                                dup_parent)
        dup_paths.append(dup)
        dup_parent = dup

    ribbon = builder.add_ribbon("%s_ribbon" % tag, curve_path, samples)
    # startFrame is the chain's own root CONTROL, not the root joint. Reading
    # a joint here creates a pose cycle: the neck ribbon's natural start is
    # neck_base_bind, which descends from chest_bind and so from the
    # spine joints that the spine's own constraints write, and the
    # dependency is resolved coarsely enough that the compiler sees a loop.
    # Controls are written by no mover, so they break it -- and Maya does the
    # same thing, hanging the neck spline off the `neck` control.
    ribbon.set_start_frame(root_ctl)
    ribbon.set_end_frame(end_ctl)
    ribbon.set_twist_frames([twist.path])
    ribbon.set_parameterization("arcLength")
    # The curve is reshaped by the control-driven matrix movers above, and
    # `rigExec:driverCurveReadPhase` defaults to "base" -- i.e. the curve as
    # authored, before any mover touches it. Left at the default the whole
    # spline is inert: the controls move the CVs and the ribbon never sees
    # it. "final" is what makes the ribbon read the driven curve.
    ribbon.set_driver_curve_read_phase("final")
    ribbon.set_joints([d.path for d in dup_paths])
    ribbon.set_joint_elements(elements)

    print("  %-6s %d joints on a %d-CV catmullRom curve, 3 controls, "
          "%d samples (+/-%.2f cm quantisation), elements %s"
          % (tag, n, cv_count, samples, quant * 0.5, elements))
    return dict(tag=tag, joints=joints, dup=[d.path for d in dup_paths],
                curve=curve_path, ribbon=ribbon, twist=twist,
                controls=(root_ctl, mid_ctl, end_ctl))
    return dict(tag=tag, curve=curve_path, ribbon=ribbon, twist=twist,
                controls=(root_ctl, mid_ctl, end_ctl), joints=joints)


# The aim-driven twist helpers, from body_rig.nxt:/left/connect_twist.
# These matter more than their obscurity suggests: 54 of the body skin's 137
# influences are twist/noTwist/trans joints, and in Maya they are DRIVEN, not
# static. Authored as rests alone, the forearm and shin do not twist.
#
# Each entry is (driven, aim_at, bone_from, sign):
#   bone_from  the joint whose local translate gives the bone axis
#   sign       +1 aims down the bone at the child, -1 aims back at the parent
# Maya uses `rig_transform.get_distance_vector`, i.e. the largest-magnitude
# signed component of that translate, so the axis is read from the data
# rather than hard-coded per side.
TWIST_AIMS = [
    ("shoulderNoTwist_l_bind", "elbow_l_bind", "elbow_l_bind", +1),
    ("shoulderNoTwist_r_bind", "elbow_r_bind", "elbow_r_bind", +1),
    ("elbowTwist_l_bind", "shoulder_l_bind", "elbow_l_bind", -1),
    ("elbowTwist_r_bind", "shoulder_r_bind", "elbow_r_bind", -1),
    ("clavicle_trans_l_bind", "clavicle_l_bind", "shoulder_l_bind", -1),
    ("clavicle_trans_r_bind", "clavicle_r_bind", "shoulder_r_bind", -1),
    ("thighNoTwist_l_bind", "knee_l_bind", "knee_l_bind", +1),
    ("thighNoTwist_r_bind", "knee_r_bind", "knee_r_bind", +1),
    ("kneeTwist_l_bind", "thigh_l_bind", "knee_l_bind", -1),
    ("kneeTwist_r_bind", "thigh_r_bind", "knee_r_bind", -1),
    ("pelvis_trans_l_bind", "pelvis_l_bind", "thigh_l_bind", -1),
    ("pelvis_trans_r_bind", "pelvis_r_bind", "thigh_r_bind", -1),
]


def dominant_axis(translate, sign=1):
    """Maya's get_distance_vector: the signed dominant component as a unit."""
    i = max(range(3), key=lambda k: abs(translate[k]))
    v = [0.0, 0.0, 0.0]
    v[i] = (1.0 if translate[i] >= 0 else -1.0) * sign
    return tuple(v)


def add_twist_aims(builder, data, joint_prim_paths):
    """Aim-constrain the twist helpers the way connect_twist does.

    Maya builds these as `aimConstraint(..., upVector=(0,0,0),
    worldUpType='none')`, which constrains ONLY the aim axis and leaves roll
    alone. RigExec's `rigExec:worldUpType` carries Maya's vocabulary exactly
    (`sceneUp|objectUp|objectRotationUp|vector|none`), so `none` reproduces
    it -- and because the aim axis already points at its target at rest, the
    rest pose is preserved rather than snapped.

    NOT covered here: `wristTwist_?` and `ankleTwist_?`, which Maya drives
    through `rig_transform.decompose_rotation` -- a swing/twist decomposition
    feeding `joint.decomposeTwist`, not an aim. RigExec has
    `RigExecTwistDistribution` which is the likely home for those, but the
    mapping is not worked out yet.
    """
    chain = builder.new_mover_chain("twist_aims")
    made = []
    for driven, aim_at, bone_from, sign in TWIST_AIMS:
        if driven not in joint_prim_paths or aim_at not in joint_prim_paths:
            print("  %s: SKIPPED (missing joint)" % driven)
            continue
        axis = dominant_axis(data[bone_from]["translate"], sign)
        c = chain.add_aim_constraint("%s_aim" % driven,
                                     joint_prim_paths[driven],
                                     [joint_prim_paths[aim_at]])
        c.set_aim_vector(axis[0], axis[1], axis[2])
        c.set_world_up_type("none")
        c.set_up_vector(0.0, 0.0, 0.0)
        made.append((driven, aim_at, axis))
        print("  %-24s aims at %-20s along (%.0f %.0f %.0f)"
              % (driven, aim_at, axis[0], axis[1], axis[2]))
    return made


# The girdles -- clavicle and pelvis controls plus the hips follow --
# now live in `girdle.py`. They were rewritten there against the Maya
# graph (`rig_bits.nxt`: the clavicle/pelvis nuls are PARENTED under
# chest_top/hip_swivel, and the joints are POINT-constrained only, the
# orient deliberately omitted at line 1155 so limbs can be translated
# together). The version that used to sit here carried each girdle
# control with a constraint instead of namespace nesting, added its
# constraints root-before-children -- inverted, since a chain applies in
# reverse add order -- and referenced names it never defined.

def layout_for_noodles(stage, rig_root):
    """Author ui:nodegraph:node:pos so the rig is legible in usdNoodles.

    usdNoodles discovers node types from the UsdSchemaRegistry, so every
    RigExec prim already shows up as a node without any editor-side work --
    but with no authored position they all stack at the origin. Laying them
    out in columns by role (controls -> solvers -> joints, with movers and
    weights off to the side) makes the graph navigable.

    Positions are stored divided by 1000: `NodeModel._writePositionToUsdRaw`
    applies a 1/1000 scale when it writes, so these values and the ones the
    editor writes back are in the same units.
    """
    columns = [
        ("Controls", 0.0),
        ("Solvers", 1.6),
        ("Joints", 3.2),
        ("Weights", 4.8),
        ("Movers", 6.4),
    ]
    placed = 0
    for scope_name, x in columns:
        scope = stage.GetPrimAtPath("%s/%s" % (rig_root, scope_name))
        if not scope or not scope.IsValid():
            continue
        row = 0
        # Traverse so nested joints and nested movers each get their own row
        # rather than only the top level of the scope.
        for prim in Usd.PrimRange(scope):
            if not prim.GetTypeName().startswith("RigExec"):
                continue
            depth = len(prim.GetPath().pathString.split("/")) - \
                len(scope.GetPath().pathString.split("/"))
            attr = prim.GetAttribute("ui:nodegraph:node:pos")
            if not attr or not attr.IsValid():
                attr = prim.CreateAttribute("ui:nodegraph:node:pos",
                                            Sdf.ValueTypeNames.Float2)
            attr.Set(Gf.Vec2f(x + depth * 0.18, row * 0.11))
            row += 1
            placed += 1
    print("laid out %d nodes for usdNoodles" % placed)
    return placed


def finish_spline_chains(builder, stage, rig_root, chains, worlds,
                         joint_prim_paths):
    """Constrain the bind joints from the ribbon-driven duplicate chains.

    This is the second half of the ikSpline port, and it has to run after a
    first evaluation because the offsets are not derivable from the authored
    rests. Maya's `pointConstraint(ik, bind, mo=1)` captures the difference
    between the driver's *solved* transform and the driven joint's transform
    at the moment the constraint is made. Here that is exactly

        offset = bind_rest * inverse(ribbon_solved_frame_at_rest)

    which absorbs both of the discrepancies that defeated the first attempt:
    the curve does not pass through the interior joints (Maya's does not
    either -- it is degree 2 through four CVs), and the ribbon's transported
    frames sit at a constant 90 degrees to Maya's joint orientations.
    """
    solved = solve_spline_rest(stage, rig_root, chains)
    total = 0
    for info in chains:
        total += bind_spline_chain(builder, info, solved, worlds,
                                   joint_prim_paths)
    return total


def solve_spline_rest(stage, rig_root, chains):
    """The solved rest frame of every duplicate joint, {path: Matrix4d}.

    One compile + evaluate for all the chains, so `bind_spline_chain` can
    be called for each chain at the point in the build where its mover
    chain has to be CREATED -- which is not one point for all of them,
    since creation order is execution order reversed (see build()).
    """
    rig = rigexec.Rig(stage, rig_root)
    rig.compile()
    pose = rig.evaluate(0.0)
    out = {}
    for info in chains:
        for dup in info["dup"]:
            out[dup] = Gf.Matrix4d(*pose.joint_frame(dup).to_matrix4())
    return out


def bind_spline_chain(builder, info, solved, worlds, joint_prim_paths):
    """Create one spline's `<tag>_to_bind` chain from pre-solved frames.

    A chain per spline, NOT one shared chain. A mover chain is one
    dependency unit, so a single chain holding both the spine and neck
    constraints would write the spine joints AND -- through the neck
    solver's start frame -- read them. Separate chains leave the
    dependency a DAG, and, since creation order is execution order
    reversed, let the caller place each chain: the neck's is created
    BEFORE the girdle chain so it executes after chest_bind's final
    write (see build()).
    """
    from girdle import offset_in_source_space as _offset_in_source_space

    total = 0
    tag = info["tag"]
    chain = builder.new_mover_chain("%s_to_bind" % tag)
    drifts = []
    # ADD ORDER MATTERS, and it is the reverse of what reads naturally.
    # A mover chain applies in reverse add order, so adding root-first
    # would EXECUTE tip-first -- and a joint's write propagates to its
    # descendants, so the root writing last re-scaled every joint below
    # it. That compounded the squash down the chain: the bind joints
    # came out 0.957, 0.875, 0.763, 0.649, 0.582, 0.545, 0.533, which is
    # just the running product of the correct per-joint profile
    # (0.957 x 0.914 = 0.875, and so on) instead of the profile itself.
    # Adding tip-first makes the root execute first, so each joint's own
    # absolute write lands after it has inherited its parent's.
    for joint, dup in reversed(list(zip(info["joints"], info["dup"]))):
        solved_m = solved[dup]
        rest = worlds[joint]
        drifts.append((solved_m.ExtractTranslation() -
                       rest.ExtractTranslation()).GetLength())
        t, r = _offset_in_source_space(rest, solved_m)
        pc = chain.add_parent_constraint(
            "%s_to_%s" % (dup.rsplit("/", 1)[-1], joint),
            joint_prim_paths[joint], [dup])
        pc.set_translation_offsets([t])
        pc.set_rotation_offsets([r])
        total += 1
        # A parent constraint carries translation and rotation only, so
        # the squetch never reached the bind joints without this: the
        # duplicates thinned exactly on Maya's profile (0.957, 0.914,
        # 0.871, 0.849, 0.892, 0.935, 0.979 under a 30% stretch, so
        # thinnest at spine_3 where the volume weight peaks at 0.5)
        # while every bind joint stayed at 1.000. Maya's spline.py
        # makes a separate scaleConstraint for the same reason.
        chain.add_scale_constraint(
            "%s_scale_to_%s" % (dup.rsplit("/", 1)[-1], joint),
            joint_prim_paths[joint], [dup])
        total += 1
    print("  %-6s %d joints: parent + scale constraint each; the "
          "solved chain sat %.2f-%.2f cm off the bind joints at rest, "
          "now absorbed by the offsets"
          % (tag, len(info["joints"]), min(drifts), max(drifts)))
    return total


def add_rigexec_skinning(builder, stage, manifest_path, mesh_name,
                         joint_prim_paths, rig_root,
                         skin_mode="skin"):
    """Deform the mesh with RigExec's own movers instead of UsdSkel.

    One `RigExecStaticWeight` plus one `RigExecMatrixMover` per influence,
    which is the pattern every skinned example in the repo uses
    (`01_FkChainTail.usda`, `ArmRig.usda`, `examples/python/make_fk_arm.py`).

    This is NOT linear blend skinning and will not match Maya at any vertex
    with more than one influence. Each mover computes

        p' = p + w * (T*p - p)

    over the *previous* mover's output, so the joints compose sequentially
    rather than as the weighted sum `sum(w_i * T_i * p_rest)` that a Maya
    skinCluster evaluates. Single-influence vertices agree exactly; blended
    ones diverge, and the result depends on mover order. Bake through
    `bake_rigexec_to_skel.py` when you need Maya-matching deformation.
    """
    import json

    man = json.load(open(manifest_path))
    if mesh_name not in man:
        raise SystemExit("%s not in manifest" % mesh_name)
    mesh = man[mesh_name]

    pts = [Gf.Vec3f(mesh["points"][i * 3] * 100.0,
                    mesh["points"][i * 3 + 1] * 100.0,
                    mesh["points"][i * 3 + 2] * 100.0)
           for i in range(mesh["pointCount"])]

    UsdGeom.Scope.Define(stage, "/Biped/Geom")
    geom = UsdGeom.Mesh.Define(stage, "/Biped/Geom/%s" % mesh_name)
    geom.CreatePointsAttr(Vt.Vec3fArray(pts))
    geom.CreateFaceVertexCountsAttr(Vt.IntArray(mesh["faceVertexCounts"]))
    geom.CreateFaceVertexIndicesAttr(Vt.IntArray(mesh["faceVertexIndices"]))
    geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    geom.CreateExtentAttr(Vt.Vec3fArray(
        UsdGeom.PointBased(geom).ComputeExtent(Vt.Vec3fArray(pts))))
    target = "/Biped/Geom/%s.points" % mesh_name
    print("  authored mesh %s (%d points)" % (target, len(pts)))

    wts_file = os.path.join(DATA, "skin_wts",
                            "%s__%s_skinCluster.xml" % (mesh_name, mesh_name))
    inf_order, tables, _attrs, _rest = parse_weights(wts_file)
    usable = [s for s in inf_order if s in joint_prim_paths and tables[s]]
    skipped = [s for s in inf_order if s not in joint_prim_paths]
    if skipped:
        raise SystemExit("influences missing from the rig: %s"
                         % ", ".join(skipped))
    print("  influences: %d (%d carry weights)"
          % (len(inf_order), len(usable)))

    if skin_mode == "skin":
        # One RigExecSkinMover for the whole mesh: true linear blend
        # skinning, `p' = (1 - sum w) * p + sum w_i * (T_i * p)`, which is
        # exactly `sum w_i T_i p` once the weights are normalised. The
        # sequential RigExecMatrixMover path below is kept because the
        # repo's own examples use it, but it is NOT linear blend skinning
        # and diverges from Maya at every blended vertex -- measured up to
        # 9.2 cm over 80% of this mesh.
        #
        # Influence ORDER defines the index space, so it is captured once
        # here and the per-point indices are built against it.
        order = [s for s in inf_order if s in joint_prim_paths]
        index_of = {s: i for i, s in enumerate(order)}
        per_vertex = [[] for _ in pts]
        for src in usable:
            i = index_of[src]
            for vtx, w in tables[src].items():
                if w != 0.0 and vtx < len(per_vertex):
                    per_vertex[vtx].append((i, w))

        element_size = max(len(v) for v in per_vertex)
        idx_arr, w_arr, renorm = [], [], 0
        for v in per_vertex:
            v = sorted(v, key=lambda iw: -iw[1])[:element_size]
            total_w = sum(w for _, w in v)
            # The XML rounds weights to 3 decimals, so per-vertex sums drift
            # off 1.0. The kernel keeps `(1 - sum w)` of the rest point
            # rather than renormalising, so normalise here or the skin
            # quietly retains a fraction of its bind pose.
            if v and abs(total_w - 1.0) > 1e-6:
                v = [(i, w / total_w) for i, w in v]
                renorm += 1
            while len(v) < element_size:
                v.append((0, 0.0))
            idx_arr.extend(i for i, _ in v)
            w_arr.extend(w for _, w in v)

        chain = builder.new_mover_chain("skin_%s" % mesh_name, target)
        skin = chain.add_skin_mover(
            "%s_skin" % mesh_name,
            [joint_prim_paths[s] for s in order], None, target, "final")
        skin.set_joint_influences(idx_arr, w_arr, element_size)
        skin.set_skinning_method("classicLinear")
        # The influence TRANSFORMS must be read at the final phase, not the
        # default base. The read_phase argument above governs the points
        # this mover consumes; this governs the joint matrices it skins
        # with. Left at base, the skin is driven by pre-pose joint frames --
        # it misses the twist aim constraints entirely and disagrees with
        # UsdSkel by up to 12.4 cm around the shoulder, even though the
        # kernel itself is bit-exact. It only looks correct on a rig with no
        # solvers or constraints, which is exactly how I first verified it.
        skin.set_transform_read_phase("final")
        print("  one RigExecSkinMover: %d influences, elementSize %d, "
              "%d vertices renormalised" % (len(order), element_size, renorm))
        return target, 1

    weights = {}
    for src in usable:
        table = tables[src]
        idx = sorted(table)
        weights[src] = builder.add_static_weight(
            "%s_weight" % src, target,
            [table[i] for i in idx], idx, 0.0)

    # A mover chain applies in REVERSE add order (rigexec/__init__.py:37-39),
    # so adding the influences reversed makes them execute in the
    # skinCluster's own matrix order -- the nearest thing to a defined
    # reference ordering for a model that is order-dependent anyway.
    chain = builder.new_mover_chain("skin_%s" % mesh_name, target)
    for src in reversed(usable):
        chain.add_matrix_mover("%s_mover" % src, joint_prim_paths[src],
                               weights[src], target, "final")
    print("  added %d matrix movers" % len(usable))
    return target, len(usable)


def add_feet_and_hands(builder, stage, worlds, joint_prim_paths, data,
                       limbs, with_feet=True, with_hands=True):
    """Wire the reverse/FK foot and the FK fingers onto the built limbs.

    These were written and verified as standalone modules
    (`build_foot.py`, `build_fingers.py`, 104/104 checks in
    `verify_foot_fingers.py`) against a purpose-built test stage. This is
    the wiring onto the real rig, and it has to come AFTER the limb IK
    because the reverse foot re-targets the leg solver's effector control
    and gates its aim constraints on the leg's IK/FK blend weight -- it
    needs the solver and blend handles to exist.

    It also has to come after the skin mover was AUTHORED, since a mover
    chain applies in reverse add order: the skin mover is authored first so
    it evaluates last, and every constraint the foot adds must run before
    it or the compiler rejects the final-phase read outright.
    """
    made = {"feet": {}, "hands": {}}
    by_tag = {tag: (ik, eff, blend) for tag, ik, eff, _end, blend in limbs}

    if with_feet:
        print("\nreverse + FK foot:")
        for side in ("l", "r"):
            entry = by_tag.get("leg_%s" % side)
            if entry is None:
                print("  foot_%s: SKIPPED, no leg_%s IK" % (side, side))
                continue
            ik, eff, blend = entry
            from build_foot import (add_fk_foot, add_reverse_foot,
                                    apply_foot_shapes)
            foot = add_reverse_foot(builder, stage, side, worlds,
                                    joint_prim_paths, eff, ik, blend,
                                    data=data)
            add_fk_foot(stage, side, joint_prim_paths, foot)
            apply_foot_shapes(stage, foot)
            made["feet"][side] = foot

    if with_hands:
        print("\nFK fingers:")
        from build_fingers import add_fk_fingers
        for side in ("l", "r"):
            made["hands"][side] = add_fk_fingers(stage, side,
                                                 joint_prim_paths)
    return made


def build(out_path, skin_path=None, rig_root="/Biped/Rig",
          with_ik=True, manifest=None, mesh_name="body_geo",
          with_fk=False, with_twist=False, with_blend=True,
          with_spine=False, with_girdles=True,
          skin_mode="skin", fk_follow_parent=False,
          with_feet=True, with_hands=True, layered=False,
          ik_handles_follow_girdle=False, with_torso=True,
          with_materials=False):
    data, parent = load_joints(os.path.join(DATA, "joint_positions.data"))
    ordered = skeleton_order(data, parent)
    print("joints to author: %d" % len(ordered))

    rigexec.load_schema_plugin()
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)

    if skin_path:
        # Relative so the pair stays movable together.
        rel = os.path.relpath(skin_path, os.path.dirname(
            os.path.abspath(out_path))).replace("\\", "/")
        stage.GetRootLayer().subLayerPaths.append(rel)
        print("sublayered skin: %s" % rel)

    builder = rigexec.Builder.create(stage, rig_root)

    # The limb root/mid frames are replaced with the basis
    # RigExecTwoBoneIk publishes. build_biped_skel.py applies the identical
    # re-frame to its rest/bind transforms; the two must not diverge. See
    # tools/biped/limb_frames.py.
    from limb_frames import reframe_limbs
    print("re-framing limb joints for RigExecTwoBoneIk:")
    worlds, locals_, poles = reframe_limbs(data, parent, ordered)

    handles = {}
    for name, path in ordered:
        p = parent.get(name)
        handles[name] = builder.add_joint(name, flatten(locals_[name]),
                                          handles.get(p))

    # Two-bone IK, off by default. Binding RigExecTwoBoneIk straight to the
    # bind joints rolls the right limb's root and mid joints 180 degrees
    # about the bone axis: this rig mirrors its right side by negating the
    # bone translation (ankle_r translate is (-42.840, 0, 0) against the
    # left's +42.840), and the solver builds its chain frames with a fixed
    # aim convention, so a limb whose bones run down local -X comes out
    # rolled. The pole side does not change it -- flipping the pole leaves
    # the right limb at 180 and puts the left limb there instead.
    #
    # The Maya build never hits this because it does not drive the bind
    # joints directly: ikfk.py:164-185 duplicates them into _fk/_ik/_blend
    # chains, solves on _ik, blends, and constrains the bind joints from
    # _blend. Reproducing that indirection is the fix, and is Phase 3.
    joint_prim_paths = {n: "%s/Joints/%s" % (rig_root, p)
                        for n, p in ordered}

    girdles = None
    fk = []
    if with_fk:
        print("\nFK chains:")
        chains = FK_CHAINS
        if with_spine:
            # The spline IK already owns the spine and neck joints.
            # Leaving fk_spine/fk_neck in would put a SECOND solver on
            # the same joints -- two writers, and the animator moving
            # one set of controls while the other silently fights it.
            # Maya has the same exclusivity: spline.py drives those
            # joints and there is no parallel FK set over them.
            chains = [(n, j) for n, j in FK_CHAINS
                      if n not in ("fk_spine", "fk_neck")]
            print("  (spine and neck FK chains omitted: the spline "
                  "IK drives those joints)")
        fk = add_fk_chains(builder, worlds, joint_prim_paths, parent,
                           chains, fk_follow_parent)

    # Skinning is authored FIRST so it runs LAST. A mover chain applies in
    # reverse add order, so the chain created last is evaluated first -- and
    # a skin mover reading the FINAL joint transforms must come after every
    # constraint that writes a joint, or the compiler rejects it outright:
    #   "Unsatisfied final read: body_geo_skin reads final of
    #    thighNoTwist_l_bind but a writer with a later ordinal exists"
    # which is a good error, caught at compile rather than showing up as a
    # silently mis-skinned shoulder.
    if manifest:
        print("\nRigExec-native skinning:")
        add_rigexec_skinning(builder, stage, manifest, mesh_name,
                             joint_prim_paths, rig_root, skin_mode)

    if with_twist:
        # Created right after the skin chain so it EXECUTES right before
        # it, after every write to the joint hierarchy (hips_follow, the
        # spline bind chains, the girdles, the feet). It used to be
        # created after the spine block, and so ran BEFORE hips_follow
        # and the girdles, aiming at joints that had not been moved
        # yet: a pure 12.2 cm translate of the whole body turned
        # shoulderNoTwist_?_bind 17-28 degrees and thighNoTwist_?_bind
        # 15 degrees, and a 15 cm chest translate turned
        # shoulderNoTwist 35.4 degrees (measured on the shipped rig
        # too, so it predates the body control). Here the same
        # translate leaves every twist joint at 0.0000 cm / 0.0000
        # degrees of rigid drift and the skinned mesh at exactly the
        # body's displacement over all 26276 points. Rest check and
        # every gate re-proved after the move.
        print("\ntwist aim constraints:")
        add_twist_aims(builder, data, joint_prim_paths)

    if with_spine:
        # The body controls first: hips (Maya `hips`, the control that
        # carries the whole character) and torso between it and the
        # chest. The spine's pivot controls nest under them.
        print("\nbody controls (Maya rig_bits.nxt /spine/controls/fk: "
              "hips > hips_gimbal > {hip_swivel, torso > chest}):")
        from body import add_body_controls
        body = add_body_controls(builder, stage, worlds,
                                 with_torso=with_torso)
        chest_parent = body["torso"] or body["hips"]

        print("\nspline IK (control-driven, the ikSpline analogue):")
        # Spine: three controls (hip_swivel, spine_mid, chest_top) with
        # pivots on the root and end; neck: root and head only, the head
        # with a pivot, cv1 aimed from the root (rig_bits.nxt /neck has
        # no pivotHeight on the neck control and no mid control at all).
        # Parenting: hip_swivel under hips, chest under torso; the neck
        # control under the chest control, the head under the neck
        # (see add_spline_ik_chain).
        spine_chain = add_spline_ik_chain(
            builder, stage, worlds, joint_prim_paths,
            "spine", SPINE_JOINTS, SPINE_VOLUME_WEIGHTS,
            mid="follow", pivots=("root", "end"),
            root_parent=body["hips"], end_parent=chest_parent)
        neck_chain = add_spline_ik_chain(
            builder, stage, worlds, joint_prim_paths,
            "neck", NECK_JOINTS, NECK_VOLUME_WEIGHTS,
            mid="aim", pivots=("end",),
            root_parent=spine_chain["controls"][2], end_parent="root")
        spline_chains = [spine_chain, neck_chain]

        # ORDER IS LOAD-BEARING. A mover chain executes in REVERSE
        # creation order, and a joint's write propagates to every
        # descendant not re-written afterwards, so the chains that
        # write the spine hierarchy are created in the reverse of the
        # order they must run:
        #
        #   executes   created   chain           writes
        #   1st        4th       hips_follow     hips_bind (root of all)
        #   2nd        3rd       spine_to_bind   spine_0 .. chest_bind
        #   3rd        2nd       girdles         chest_bind (override),
        #                                        clavicles, pelves
        #   4th        1st       neck_to_bind    neck_0 .. skull_bind
        #
        # Each established by counterexample:
        #   * girdles before spine_to_bind (executing after) -- the
        #     other way round the rest check still passed but the
        #     clavicles sat 17 cm off their controls and a 25 degree
        #     chest swing turned chest_bind 62.8 degrees;
        #   * hips_follow created last (executing first) -- created
        #     earlier, the hip twist DOUBLED: 59.7 degrees for a 30
        #     degree roll;
        #   * neck_to_bind created FIRST (executing last). With both
        #     spline chains bound in one call the neck's chain was
        #     created after the spine's and so executed BEFORE it and
        #     before the girdles' chest override: the neck joints were
        #     written absolutely, then chest_bind's later re-write
        #     (rest * inverse(hips-propagated)) propagated onto them.
        #     Swivelling the hips 30 degrees moved neck_0_bind 25.64 cm
        #     and skull_bind 33.08 cm with chest_bind at 0.00.
        #     verify_spine.py's "hips swivel" check guards this.
        print("  binding the bind joints (Maya's mo=1):")
        solved = solve_spline_rest(stage, rig_root, spline_chains)
        bind_spline_chain(builder, neck_chain, solved, worlds,
                          joint_prim_paths)

        if with_girdles:
            print("\ngirdles (Maya rig_bits.nxt: the clavicle and "
                  "pelvis nuls are PARENTED under chest_top / "
                  "hip_swivel, and the joints are "
                  "point-constrained only):")
            from girdle import add_girdles
            spine_ctls = spine_chain["controls"]
            girdles = add_girdles(builder, stage, worlds,
                                  joint_prim_paths, spine_ctls[0],
                                  spine_ctls[2],
                                  chest_follows_end=True)

        print("  binding the spine's bind joints:")
        bind_spline_chain(builder, spine_chain, solved, worlds,
                          joint_prim_paths)

        if with_girdles:
            from girdle import add_hips_follow
            add_hips_follow(builder, stage, worlds, joint_prim_paths,
                            spine_chain["controls"][0])

    limbs = []
    if with_ik:
        print("\nlimb IK:")
        joint_prim_paths = {n: "%s/Joints/%s" % (rig_root, p)
                            for n, p in ordered}
        # arm -> clavicle, leg -> pelvis: Maya's limb-nul parenting.
        girdle_parents = {}
        if girdles:
            for tag, ctl_name in (("arm_l", "clavicle_l_ctl"),
                                  ("arm_r", "clavicle_r_ctl"),
                                  ("leg_l", "pelvis_l_ctl"),
                                  ("leg_r", "pelvis_r_ctl")):
                ctl = girdles["controls"].get(ctl_name)
                if ctl is not None:
                    girdle_parents[tag] = ctl
        limbs = add_limb_iks(builder, stage, worlds, joint_prim_paths,
                             poles, with_blend, fk_follow_parent,
                             girdle_parents,
                             ik_handles_world=not ik_handles_follow_girdle)

    if with_feet or with_hands:
        add_feet_and_hands(builder, stage, worlds, joint_prim_paths, data,
                           limbs, with_feet, with_hands)

    if with_spine:
        # LAST, so they execute FIRST: the spine mid's follow helper and
        # the neck's root aim helper read only plain controls and feed the
        # spline solvers, which everything downstream reads. See the
        # note at the end of add_spline_ik_chain.
        print("\nspline helper movers (created last, executed first):")
        for info in spline_chains:
            if info["make_movers"] is None:
                print("  %s: none needed" % info["tag"])
                continue
            chain = info["make_movers"]()
            print("  %s" % getattr(chain, "name", getattr(chain, "path", chain)))

    root_prim = stage.GetPrimAtPath(rig_root)
    # Examples stamp this so a reader can tell parent-local rests from the
    # older asset-space ones.
    attr = root_prim.CreateAttribute(
        "rigExec:restFrameVersion", Sdf.ValueTypeNames.Int, True,
        Sdf.VariabilityUniform)
    attr.Set(2)

    # Maya's shapes, colours and sizes on the controls, read from
    # control_positions.data and the studio colour module.
    print("\ncontrol shapes from Maya:")
    from control_shapes import apply_control_shapes
    apply_control_shapes(stage, rig_root, verbose=False)

    layout_for_noodles(stage, rig_root)

    stage.SetDefaultPrim(stage.GetPrimAtPath("/Biped")
                         or root_prim)
    stage.GetRootLayer().Save()
    if with_materials:
        # After the mesh exists, before the save. Twelve
        # UsdPreviewSurface materials from the FBX, with body_geo's
        # five slots as GeomSubsets in the materialBind family -- one
        # binding cannot express five materials over 26,274 faces.
        # There are no textures in the source, so these are flat
        # values; see materials.py for the Metallic caveat.
        print("\nmaterials from the FBX:")
        from materials import apply_materials
        bound = apply_materials(stage)
        print("  bound %d material slot(s) on the body mesh" % bound)
        stage.GetRootLayer().Save()

    print("wrote %s (%.1f MB)" % (out_path, os.path.getsize(out_path) / 1e6))

    if layered:
        # The layered form is a POST-PROCESS, not a different build. The
        # right side becomes a `references` arc onto the LEFT LAYER FILE
        # plus the ~52 attributes that genuinely differ. It is a file
        # reference and not `inherits` for a measured reason: an inherit
        # composes every opinion standing on the left prim, animation
        # included, so keying the left shoulder moved the RIGHT arm by up
        # to 25.5 cm. A reference to the saved left layer composes only
        # what that layer holds -- rig data flows across, animation does
        # not. See split_layers.py.
        print("\nlayered form:")
        from split_layers import split
        split(out_path, os.path.splitext(out_path)[0] + "_layered.usda")

    # --- Prove it, rather than trusting the nesting. ---
    print("\ncompiling the rig...")
    rig = rigexec.Rig(stage, rig_root)
    rig.compile()
    pose = rig.evaluate(0.0)

    joint_paths = pose.joint_paths()
    print("evaluated joints: %d" % len(joint_paths))

    worst, worst_at = 0.0, None
    checked = 0
    by_name = {}
    for jp in joint_paths:
        by_name[str(jp).rsplit("/", 1)[-1]] = jp
    for name, _ in ordered:
        jp = by_name.get(name)
        want = data[name].get("world_translate")
        if jp is None or not want:
            continue
        # joint_matrix() is the rest-to-pose affine DELTA (identity at rest),
        # not a world transform -- the world origin comes off the joint's
        # point frame, the way tests/python/test_rest_local.py reads it.
        m = pose.joint_frame(jp).to_matrix4()
        got = (m[12], m[13], m[14])
        d = max(abs(got[i] - want[i]) for i in range(3))
        checked += 1
        if d > worst:
            worst, worst_at = d, name
    print("rest world origin vs Maya world_translate:")
    print("  checked %d joints, worst %.6g cm (%s)" % (checked, worst, worst_at))
    if worst > 1e-3:
        print("  FAIL -- parent-local rest is not composing to Maya's world")
        return 1
    print("  PASS -- RigExec reproduces Maya's rest skeleton")

    # --- The IK has to hold the rest pose, then actually bend. ---
    if limbs:
        print("\nlimb IK behaviour:")
        for tag, ik, eff_ctl, ename, _blend in limbs:
            jp = by_name.get(ename)
            want = data[ename]["world_translate"]
            m = pose.joint_frame(jp).to_matrix4()
            held = max(abs(m[12 + i] - want[i]) for i in range(3))

            # Pull the effector 25cm along +X and re-evaluate; the end joint
            # is expected to track the effector, so it should move about as
            # far as the effector did.
            eff_ctl.set_avar_translation(25.0, 0.0, 0.0)
            moved_pose = rig.evaluate(0.0)
            m2 = moved_pose.joint_frame(jp).to_matrix4()
            delta = max(abs(m2[12 + i] - m[12 + i]) for i in range(3))
            eff_ctl.set_avar_translation(0.0, 0.0, 0.0)

            # A limb shipped in FK ignores its IK effector by design, so the
            # expectation flips with the blend's authored weight.
            in_ik = BLEND_DEFAULTS.get(tag, 1.0) > 0.5 or not with_blend
            if in_ik:
                verdict = "ok" if held < 1e-3 and delta > 1.0 else "PROBLEM"
                note = "IK live, effector+25cm moved end %.2f cm" % delta
            else:
                verdict = "ok" if held < 1e-3 and delta < 1e-6 else "PROBLEM"
                note = ("FK by default, so the effector is correctly inert "
                        "(%.2f cm)" % delta)
            print("  %-7s rest held to %.3g cm, %s  [%s]"
                  % (tag, held, note, verdict))

    diags = pose.diagnostics
    if callable(diags):
        diags = diags()
    if diags:
        print("\ndiagnostics:")
        for d in diags[:10]:
            print("  %s" % d)
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out")
    ap.add_argument("--skin", default=None,
                    help="UsdSkel asset to sublayer for the mesh")
    ap.add_argument("--rig-root", default="/Biped/Rig")
    ap.add_argument("--no-ik", dest="ik", action="store_false",
                    help="omit the two-bone IK solvers")
    ap.add_argument("--no-blend", dest="blend", action="store_false",
                    help="omit the FK chain and RigExecBlendPointFrames "
                         "that sit on each IK limb")
    ap.add_argument("--skin-rigexec", dest="manifest", default=None,
                    metavar="MANIFEST",
                    help="author the mesh from this extract_maya_mesh.py "
                         "manifest and deform it with RigExec movers "
                         "instead of UsdSkel")
    ap.add_argument("--mesh", default="body_geo")
    ap.add_argument("--skin-mode", choices=("skin", "matrix"), default="skin",
                    help="'skin' uses one RigExecSkinMover (true linear "
                         "blend skinning); 'matrix' uses the older "
                         "one-RigExecMatrixMover-per-joint chain, which is "
                         "sequential composition and not LBS")
    ap.add_argument("--no-girdles", dest="girdles", action="store_false",
                    help="omit the hip_swivel/chest_top girdle controls")
    ap.add_argument("--spine", action="store_true",
                    help="drive the spine and neck with RigExecRibbon on a "
                         "control-driven curve, the ikSpline analogue")
    ap.add_argument("--twist", action="store_true",
                    help="aim-constrain the twist/noTwist/trans helper "
                         "joints the way body_rig.nxt connect_twist does")
    ap.add_argument("--ik-handles-follow-girdle",
                    dest="ik_handles_follow_girdle", action="store_true",
                    help="nest the IK effector and pole controls under "
                         "the clavicle/pelvis control so a hand or foot "
                         "rides the girdle. The default keeps them in "
                         "asset space, which is Maya's arrangement (the "
                         "IK handle hangs off the rig group, so a "
                         "planted foot stays planted when the hips, "
                         "swivel or pelvis move)")
    ap.add_argument("--no-torso", dest="torso", action="store_false",
                    help="omit torso_ctl, the FK swing between hips_ctl "
                         "and the chest pivot (Maya's `torso`); the "
                         "chest pivot then nests under hips_ctl directly")
    ap.add_argument("--materials", action="store_true",
                    help="author the FBX's 12 UsdPreviewSurface "
                         "materials and bind them, with body_geo's "
                         "five slots as GeomSubsets")
    ap.add_argument("--layered", action="store_true",
                    help="also write a layered form beside the flat "
                         "one: a root composing center/left/right, "
                         "where the right side REFERENCES the left "
                         "layer and overrides only what differs")
    ap.add_argument("--no-feet", dest="feet", action="store_false",
                    help="omit the reverse/FK foot on both legs")
    ap.add_argument("--no-hands", dest="hands", action="store_false",
                    help="omit the FK finger chains")
    ap.add_argument("--fk-follow-parent", dest="fk_follow_parent",
                    action="store_true",
                    help="nest each FK control under the previous one and "
                         "put the chain in rigExec:controlSpace = "
                         "parentRelative, so the shoulder control carries "
                         "the elbow control and the elbow the wrist. Off by "
                         "default: it changes what the controls do, not "
                         "what the joints do")
    ap.add_argument("--fk", action="store_true",
                    help="add RigExecFkChain solvers with a control per "
                         "joint for both limbs, the spine and the neck")
    args = ap.parse_args(argv)
    if args.manifest and args.skin:
        ap.error("--skin and --skin-rigexec are alternatives: the first "
                 "sublayers a UsdSkel-driven mesh, the second authors its "
                 "own. Binding both would have two systems deforming the "
                 "same points.")
    return build(args.out, args.skin, args.rig_root, args.ik,
                 args.manifest, args.mesh, args.fk, args.twist,
                 args.blend, args.spine, args.girdles,
                 args.skin_mode, args.fk_follow_parent,
                 args.feet, args.hands, args.layered,
                 args.ik_handles_follow_girdle, args.torso,
                 args.materials)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
