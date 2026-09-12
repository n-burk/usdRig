#!/usr/bin/env python
"""The Maya biped's reverse foot and FK foot, expressed in RigExec.

Ground truth is `rig_bits.nxt:/foot` plus `core/maya/rig/ikfk.py:IkFkFoot`.
What Maya builds, and what each part becomes here:

  Maya                                          RigExec (this module)
  ----------------------------------------      --------------------------------------
  pivot joints ankle > bankIn > bankOut >       RigExecJoint prims nested UNDER the
    heel > ball > toe > {toeBend, ballRoll}       leg's IK control, rests straight from
    parented under leg_ik_mpivot_?_grp            joint_positions.data. Namespace nesting
                                                  is what makes the foot ride `leg_?_ik`.
  cube controls heel/toe/toeBend wrapping       RigExecControl nested at the pivot; the
    their pivot                                   next pivot nests under the control.
  ankle RP handle under toeBend pivot           `ankle_?_ikTarget` control under toeBend;
                                                  the leg's RigExecTwoBoneIk effector is
                                                  re-targeted to it.
  ball ikSCsolver handle under toeBend,         two RigExecAimConstraints with
  toe ikSCsolver handle under ballRoll            worldUpType=objectRotationUp aimed at
                                                  `ball_?_ikTarget` / `toe_?_ikTarget`.
  bank.tz -> SDK -> footRoll -> setRange /      float dials `foot:roll`, `foot:bank`,
    condition -> heel/ball/toe nul.rx,            `foot:toePlantAngle` on `bank_?`, each
  bank.tx -> remapValue -> bankIn/Out.rz          pivot's rotation a RigExecRotationConstraint
    with transformLimits                          toward a fixed-angle twin whose envelope
                                                  is a RigExecFloatMathMover chain.
  ballRoll pivot orientConstrained to the       `ballRoll_?_fk` frame whose avars are
    ball FK control (mo=True)                     CONNECTED from the ball joint's avars.
  ankle/ball/toe _ik/_fk/_blend duplicates      not needed: the aim constraints gate on
    blended into the bind joints                  the leg blend's inputs:weight instead.

## Why not RigExecSingleChainIkConstraint

It is FBX SingleChainIK, not Maya's ikSCsolver. Two differences matter here,
both verified on a synthetic chain (see the report that shipped with this
file):

  1. `_AimedBasis` (libs/rigExecMath/singleChainIk.cpp) sets the first
     joint's LOCAL X to the solved child direction. The foot joints are not
     framed bone-along-X -- `ball_l_bind` sits at (7.08, 14.0, 0) in the
     ankle's frame -- so at rest the constraint rotates the ankle 63 degrees.
     Rest does not hold.
  2. For a two-joint chain the effector's rotation never reaches the first
     joint's roll (only the solve PLANE of 3+ joint chains uses it), while the
     END joint's basis is replaced by the effector's outright.

Maya's ikSCsolver on a two-joint chain is "aim the start joint at the handle,
roll from the handle's orientation". That is precisely an aim constraint with
`objectRotationUp` on the handle, which holds rest exactly and follows the
pivots rigidly (verified: identity at rest; a 30-degree pivot turn moved the
child to the rigidly rotated point to 1e-15). Using the SCIK would need a
duplicate bone-along-X chain plus parent constraints back to the bind joints
-- Maya's own indirection -- and would STILL not carry the handle roll.

## Why the dials are float attributes, not the bank control's tx/tz

RigExecFloatMathMover targets must be `float` and scalar connections are
type-strict (`_ValidateScalarConnection`), while every avar is `double`.
Verified: targeting `bank_l.avars:tz` is refused with "has type double;
expected float". So the animator's inputs are `float` custom attributes --
exactly Maya's `use_attributes` mode, where `footRoll`/`footRock` on the param
node drive the bank control. A double-typed math mover, or float->double
connection coercion, would let the tx/tz avars drive this directly; nothing
here would change but the connection source.

## How a float becomes a rotation

The one thing RigExec computes from a float pre-solver is a constraint's
envelope. A RigExecRotationConstraint slerps between the incoming orientation
and its source, and slerp is LINEAR in angle, so a pivot rotation-constrained
toward a twin rotated by a fixed `theta0` about the drive axis, with envelope
`w`, sits at exactly `theta0 * w` (verified: roll -22.5 -> heel -22.5, -45 ->
-45, clamp at -90). The envelope is a FloatMath chain whose `add` mover's
`inputs:value` is connected to the dial; `clamp` bounds may be connected too,
which is what keeps `toePlantAngle` live. A constraint's output does reach a
solver that reads it (verified: a constrained pivot carrying a TwoBoneIk
effector moved the IK end joint), so the leg follows the rolled foot.

Angles are therefore clamped at `roll_range` / `bank_range` (defaults 120 and
90 degrees) instead of Maya's 360: a slerp twin must stay short of 180.
"""
import os
import sys

from pxr import Gf, Sdf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_biped_skel import DATA, load_joints, local_matrix  # noqa: E402

import rigexec  # noqa: E402


def flatten(m):
    return [m[r][c] for r in range(4) for c in range(4)]


def _path_of(thing):
    """A rigexec handle, a Usd.Prim, an Sdf.Path or a string -> path string."""
    if hasattr(thing, "path"):
        return str(thing.path)
    if hasattr(thing, "GetPath"):
        return str(thing.GetPath())
    return str(thing)


def _define(stage, path, type_name, rest, hidden):
    """A nested provider. `hidden` helpers draw no guide at all.

    They are NOT given purpose=guide: purpose is inherited, so a guide
    pivot would hide every animator control nested beneath it, and a
    RigExecJoint's schema default IS guide -- so the pivots author
    `default` explicitly and switch their sphere/cone guide off with
    guide:radius = 0 (documented as "draws no guides at all"); hidden
    controls do the same with a zero guide:scale.
    """
    prim = stage.DefinePrim(Sdf.Path(path), type_name)
    prim.GetAttribute("rest:space").Set(Gf.Matrix4d(rest))
    prim.GetAttribute("purpose").Set("default")
    if type_name == "RigExecControl":
        rigexec.ControlAPI.apply(prim)
        if hidden:
            for axis in "XYZ":
                prim.GetAttribute("guide:scale%s" % axis).Set(0.0)
    elif hidden:
        prim.GetAttribute("guide:radius").Set(0.0)
    return prim


def _connect(prim, attr_name, source_path, type_name=Sdf.ValueTypeNames.Float):
    attr = prim.GetAttribute(attr_name)
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(attr_name, type_name)
    attr.SetConnections([Sdf.Path(str(source_path))])
    return attr


def _rot(m):
    """Rotation-only copy of a matrix (translation zeroed)."""
    r = Gf.Matrix4d(m)
    r.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
    return r


def pivot_worlds(data, side):
    """World rest matrices of the `ankle_?_pivot` chain from joint data.

    These 18 joints are rooted at `ankle_?_pivot`, not `bind`, so
    `skeleton_order` drops them and nobody else authors them. Composition is
    `local_matrix` down the dagPath, same as the bind skeleton.
    """
    root = "ankle_%s_pivot" % side
    if root not in data:
        raise SystemExit("no %s in joint data" % root)
    worlds, parents = {}, {}
    names = [n for n, r in data.items()
             if r["dagPath"].split("|")[1:2] == [root]]
    names.sort(key=lambda n: data[n]["dagPath"].count("|"))
    for n in names:
        chain = [p for p in data[n]["dagPath"].split("|") if p]
        parent = chain[-2] if len(chain) > 1 else None
        parents[n] = parent
        lm = local_matrix(data[n])
        worlds[n] = lm if parent is None else lm * worlds[parent]
    for n in names:
        want = data[n].get("world_translate")
        got = worlds[n].ExtractTranslation()
        if want and max(abs(got[i] - want[i]) for i in range(3)) > 1e-3:
            raise SystemExit("%s: composed pivot rest disagrees with "
                             "world_translate" % n)
    return worlds, parents


# Foot joints per side, in the order Maya's IkFkFoot takes them.
def foot_joints(side):
    return ["ankle_%s_bind" % side, "ball_%s_bind" % side, "toe_%s_bind" % side]


def _weight_chain(builder, stage, name, target_attr_path, steps):
    """A FloatMath chain over `target_attr_path` (authored 0).

    `steps` is the ORDER OF EVALUATION: [(op, value_or_None, connect_to,
    bounds)], first step first. Mover chains run in reverse add order, so the
    steps are added reversed.
    """
    chain = builder.new_mover_chain(name, target_attr_path)
    for i, (op, value, source, bounds) in reversed(list(enumerate(steps))):
        mover = chain.add_float_math_mover("%s_%d_%s" % (name, i, op), op,
                                           float(value or 0.0))
        prim = stage.GetPrimAtPath(mover.path)
        if bounds is not None:
            lo, hi = bounds
            mover.set_bounds(float(lo if not isinstance(lo, str) else 0.0),
                             float(hi if not isinstance(hi, str) else 1.0))
            if isinstance(lo, str):
                _connect(prim, "inputs:min", lo)
            if isinstance(hi, str):
                _connect(prim, "inputs:max", hi)
        if source is not None:
            _connect(prim, "inputs:value", source)
    return chain


def _driven_pivot(stage, parent_path, name, rest_local, axis, theta0,
                  weight_steps, made, pending):
    """A pivot joint whose rotation about `axis` will be theta0 * envelope.

    Authors the joint and its fixed-angle twin (a hidden control at the same
    rest with a constant avar) and queues the constraint for
    `_constrain_pivots`, which adds the RigExecRotationConstraint and the
    FloatMath chain on its envelope once the whole hierarchy exists.
    """
    joint = _define(stage, "%s/%s" % (parent_path, name), "RigExecJoint",
                    rest_local, True)
    twin = _define(stage, "%s/%s_twin" % (parent_path, name),
                   "RigExecControl", rest_local, True)
    twin.GetAttribute("avars:r%s" % axis).Set(float(theta0))
    made["pivots"][name] = str(joint.GetPath())
    pending.append((name, str(joint.GetPath()), str(twin.GetPath()),
                    weight_steps))
    return joint


def _constrain_pivots(builder, stage, pending, made):
    """Rotation constraints for the queued pivots, parents executing first.

    Ordering is load-bearing. The compiler chains every frame constraint to
    the one before it in EXECUTION order, and mover execution is reversed
    namespace order (the last chain added runs first). Two consequences:

      * these must run before the leg's TwoBoneIk, whose effector sits
        under them, and the foot's aim constraints must run after that
        solver -- so the aims are added BEFORE this is called, and this
        runs the pivots; anything else is reported as a pose cycle;
      * a child pivot's incoming frame and its twin both derive from its
        parent pivot, so parents run first: the queue (root first) is added
        in reverse.
    """
    for name, joint_path, twin_path, steps in reversed(pending):
        chain = builder.new_mover_chain("%s_rc" % name)
        rc = chain.add_rotation_constraint("%s_rotation" % name, joint_path,
                                           [twin_path])
        stage.GetPrimAtPath(rc.path).GetAttribute(
            "inputs:defaultWeight").Set(0.0)
        _weight_chain(builder, stage, "%s_w" % name,
                      rc.path + ".inputs:defaultWeight", steps)
        made["constraints"].append(rc.path)


def add_reverse_foot(builder, stage, side, worlds, joint_prim_paths,
                     ik_control, two_bone_ik, blend=None, data=None,
                     toe_plant_angle=60.0, roll_range=120.0,
                     bank_range=90.0, verbose=True):
    """Build one side's reverse foot and hook it to the leg IK.

    builder          the rigexec.Builder that owns the rig
    stage            its stage
    side             "l" or "r"
    worlds           name -> world rest matrix of the bind joints (the
                     re-framed set from limb_frames.reframe_limbs)
    joint_prim_paths name -> RigExecJoint prim path
    ik_control       the `leg_?_ik` control (handle or path); the pivot
                     chain nests under it
    two_bone_ik      the leg's RigExecTwoBoneIk (handle or path); its
                     rigExec:effectorControl is re-targeted to the ankle
                     target under toeBend
    blend            the leg's RigExecBlendPointFrames (handle or path) or
                     None; when given, the foot's aim constraints gate on
                     its inputs:weight so the foot is IK exactly when the
                     leg is
    data             joint_positions.data records; loaded when None

    Returns a dict of everything authored (paths), keyed by role.
    """
    if data is None:
        data, _parent = load_joints(os.path.join(DATA, "joint_positions.data"))
    ankle, ball, toe = foot_joints(side)
    for n in (ankle, ball, toe):
        if n not in worlds or n not in joint_prim_paths:
            raise SystemExit("reverse foot %s: missing %s" % (side, n))

    ik_path = _path_of(ik_control)
    ik_prim = stage.GetPrimAtPath(ik_path)
    if not ik_prim or ik_prim.GetTypeName() != "RigExecControl":
        raise SystemExit("ik_control %s is not a RigExecControl" % ik_path)
    # The pivots are authored as children of the IK control, so their rest
    # has to be expressed relative to that control's ASSET-space frame. The
    # control's own `rest:space` is that frame only while it is a top-level
    # control; once it is nested -- which it now is, because the IK handle
    # rides the pelvis control so the foot follows the hip -- `rest:space`
    # is parent-local and the pivots would land at the pelvis's offset.
    # Composing up the namespace covers both cases, so this is no longer a
    # reason to refuse the input.
    ik_rest = Gf.Matrix4d(1.0)
    walk = ik_prim
    while walk and walk.IsValid() and walk.GetTypeName() == "RigExecControl":
        attr = walk.GetAttribute("rest:space")
        if attr and attr.IsValid() and attr.Get() is not None:
            ik_rest = ik_rest * Gf.Matrix4d(attr.Get())
        walk = walk.GetParent()

    pw, pparent = pivot_worlds(data, side)
    P = lambda n: pw["%s_%s_pivot" % (n, side)]  # noqa: E731
    made = {"side": side, "pivots": {}, "controls": {}, "targets": {},
            "constraints": [], "dials": {}}
    pending = []

    # --- Pivot chain, nested under the IK control -------------------------
    # Each rest is parent-local, so the ankle pivot is expressed in the IK
    # control's space and the rest follow their Maya parents. Where a Maya
    # pivot has a control, the CONTROL is the prim at that pivot (a
    # RigExecControl rotates about its own origin, which is exactly what
    # Maya's control + child pivot pair does) and the next pivot nests under
    # it.
    def local(world, parent_world):
        return world * parent_world.GetInverse()

    ankle_piv = _define(stage, "%s/ankle_%s_pivot" % (ik_path, side),
                        "RigExecJoint", local(P("ankle"), ik_rest), True)
    made["pivots"]["ankle_%s_pivot" % side] = str(ankle_piv.GetPath())

    # Dials on the bank control (Maya's bank_? sits under the ankle pivot at
    # the ball's position, orientation of the pivot).
    bank_rest = Gf.Matrix4d(_rot(P("ankle")))
    bank_rest.SetTranslateOnly(worlds[ball].ExtractTranslation())
    bank = _define(stage, "%s/bank_%s" % (ankle_piv.GetPath(), side),
                   "RigExecControl", local(bank_rest, P("ankle")), False)
    roll = bank.CreateAttribute("foot:roll", Sdf.ValueTypeNames.Float)
    roll.Set(0.0)
    bank_dial = bank.CreateAttribute("foot:bank", Sdf.ValueTypeNames.Float)
    bank_dial.Set(0.0)
    plant = bank.CreateAttribute("foot:toePlantAngle",
                                 Sdf.ValueTypeNames.Float)
    plant.Set(float(toe_plant_angle))
    for a, doc in ((roll, "Foot roll in degrees: negative rocks onto the "
                          "heel, positive rolls over the ball and, past "
                          "foot:toePlantAngle, onto the toe tip."),
                   (bank_dial, "Bank in degrees: positive tips the foot onto "
                               "its outer edge, negative onto the inner."),
                   (plant, "Roll angle at which the ball stops and the toe "
                           "takes over (Maya leg_?.toePlantAngle).")):
        a.SetDocumentation(doc)
    made["controls"]["bank"] = str(bank.GetPath())
    made["dials"] = {"roll": str(roll.GetPath()),
                     "bank": str(bank_dial.GetPath()),
                     "toePlantAngle": str(plant.GetPath())}
    R, B = float(roll_range), float(bank_range)
    roll_p, bank_p, plant_p = (str(roll.GetPath()), str(bank_dial.GetPath()),
                               str(plant.GetPath()))

    # bankIn.rz = max(0, -bank)  -> twin +B, w = clamp(-bank / B, 0, 1)
    # bankOut.rz = min(0, -bank) -> twin -B, w = clamp( bank / B, 0, 1)
    # Same in both feet's pivot-local frames: the right chain's Rx(180)
    # jointOrient is what mirrors it, exactly as Maya's negative remap
    # range does there.
    bank_in = _driven_pivot(
        stage, str(ankle_piv.GetPath()), "bankIn_%s_pivot" % side,
        local(P("bankIn"), P("ankle")), "z", +B,
        [("add", None, bank_p, None), ("multiply", -1.0 / B, None, None),
         ("clamp", None, None, (0.0, 1.0))], made, pending)
    bank_out = _driven_pivot(
        stage, str(bank_in.GetPath()), "bankOut_%s_pivot" % side,
        local(P("bankOut"), P("bankIn")), "z", -B,
        [("add", None, bank_p, None), ("multiply", 1.0 / B, None, None),
         ("clamp", None, None, (0.0, 1.0))], made, pending)

    # heel.rx = min(roll, 0) -> twin -R, w = clamp(-roll / R, 0, 1)
    heel_roll = _driven_pivot(
        stage, str(bank_out.GetPath()), "heel_%s_roll" % side,
        local(P("heel"), P("bankOut")), "x", -R,
        [("add", None, roll_p, None), ("multiply", -1.0 / R, None, None),
         ("clamp", None, None, (0.0, 1.0))], made, pending)
    heel = _define(stage, "%s/heel_%s" % (heel_roll.GetPath(), side),
                   "RigExecControl", Gf.Matrix4d(1.0), False)
    made["controls"]["heel"] = str(heel.GetPath())

    # ball pivot: no control, nothing drives it (Maya skips pivot_list[3]).
    ball_piv = _define(stage, "%s/ball_%s_pivot" % (heel.GetPath(), side),
                       "RigExecJoint", local(P("ball"), P("heel")), True)
    made["pivots"]["ball_%s_pivot" % side] = str(ball_piv.GetPath())

    # toe.rx = max(roll - plant, 0) -> twin +R, w = clamp((roll-plant)/R,0,1)
    toe_roll = _driven_pivot(
        stage, str(ball_piv.GetPath()), "toe_%s_roll" % side,
        local(P("toe"), P("ball")), "x", +R,
        [("add", None, plant_p, None), ("multiply", -1.0, None, None),
         ("add", None, roll_p, None), ("multiply", 1.0 / R, None, None),
         ("clamp", None, None, (0.0, 1.0))], made, pending)
    toe_ctl = _define(stage, "%s/toe_%s" % (toe_roll.GetPath(), side),
                      "RigExecControl", Gf.Matrix4d(1.0), False)
    made["controls"]["toe"] = str(toe_ctl.GetPath())

    # toeBend: animator control under the toe pivot, carrying the ankle RP
    # target and the ball SC target -- Maya parents both handles here.
    toe_bend = _define(stage, "%s/toeBend_%s" % (toe_ctl.GetPath(), side),
                       "RigExecControl", local(P("toeBend"), P("toe")), False)
    made["controls"]["toeBend"] = str(toe_bend.GetPath())

    ankle_target = _define(
        stage, "%s/ankle_%s_ikTarget" % (toe_bend.GetPath(), side),
        "RigExecControl", local(worlds[ankle], P("toeBend")), True)
    # The ball target carries the ANKLE's rest orientation so the ankle aim's
    # up vector reads identity at rest.
    ball_target_rest = Gf.Matrix4d(_rot(worlds[ankle]))
    ball_target_rest.SetTranslateOnly(worlds[ball].ExtractTranslation())
    ball_target = _define(
        stage, "%s/ball_%s_ikTarget" % (toe_bend.GetPath(), side),
        "RigExecControl", local(ball_target_rest, P("toeBend")), True)
    made["targets"]["ankle"] = str(ankle_target.GetPath())
    made["targets"]["ball"] = str(ball_target.GetPath())

    # ballRoll: Maya's condition network is the tent
    #     ball = clamp(roll, 0, plant) - clamp(roll - plant, 0, plant)
    # -- the ball rolls up to the plant angle, then straightens back out as
    # the toe takes over. Two nested pivots about the same local X add.
    ball_roll_up = _driven_pivot(
        stage, str(toe_ctl.GetPath()), "ballRoll_%s_up" % side,
        local(P("ballRoll"), P("toe")), "x", +R,
        [("add", None, roll_p, None), ("clamp", None, None, (0.0, plant_p)),
         ("multiply", 1.0 / R, None, None)], made, pending)
    ball_roll_down = _driven_pivot(
        stage, str(ball_roll_up.GetPath()),
        "ballRoll_%s_down" % side, Gf.Matrix4d(1.0), "x", -R,
        [("add", None, plant_p, None), ("multiply", -1.0, None, None),
         ("add", None, roll_p, None), ("clamp", None, None, (0.0, plant_p)),
         ("multiply", 1.0 / R, None, None)], made, pending)
    # VISIBLE. I had this hidden on the belief that Maya deletes its shape
    # -- the data says otherwise: `ballRoll_l` in control_positions.data is
    # a 16-CV curve, half-extent (6.62, 4.14, 0.29), colour 6, i.e. a flat
    # plate across the ball of the foot. Hiding it left an animator able to
    # see `toe_?` and `heel_?` with nothing between them.
    ball_roll = _define(stage, "%s/ballRoll_%s" % (ball_roll_down.GetPath(),
                                                   side),
                        "RigExecControl", Gf.Matrix4d(1.0), False)
    made["controls"]["ballRoll"] = str(ball_roll.GetPath())

    # The frame Maya orient-constrains to the ball FK control (mo=True): the
    # ball joint's rest orientation, rigidly attached to the ballRoll pivot.
    # add_fk_foot connects the ball joint's rotation avars into it, which is
    # what makes the ball control work in IK as well as FK.
    ball_fk_rest = Gf.Matrix4d(_rot(worlds[ball]))
    ball_fk_rest.SetTranslateOnly(P("ballRoll").ExtractTranslation())
    ball_fk = _define(stage, "%s/ballRoll_%s_fk" % (ball_roll.GetPath(), side),
                      "RigExecControl", local(ball_fk_rest, P("ballRoll")),
                      True)
    made["targets"]["ballFk"] = str(ball_fk.GetPath())
    toe_target_rest = Gf.Matrix4d(_rot(worlds[ball]))
    toe_target_rest.SetTranslateOnly(worlds[toe].ExtractTranslation())
    toe_target = _define(
        stage, "%s/toe_%s_ikTarget" % (ball_fk.GetPath(), side),
        "RigExecControl", local(toe_target_rest, ball_fk_rest), True)
    made["targets"]["toe"] = str(toe_target.GetPath())

    # --- Hook to the leg: the ankle target is the TwoBoneIk effector -------
    ik_solver = stage.GetPrimAtPath(_path_of(two_bone_ik))
    if not ik_solver or ik_solver.GetTypeName() != "RigExecTwoBoneIk":
        raise SystemExit("two_bone_ik %s is not a RigExecTwoBoneIk"
                         % _path_of(two_bone_ik))
    ik_solver.GetRelationship("rigExec:effectorControl").SetTargets(
        [ankle_target.GetPath()])
    made["twoBoneIk"] = str(ik_solver.GetPath())

    # --- The two "ikSCsolver" handles as aim constraints -------------------
    # ankle aims at the ball target, ball at the toe target. The aim vector
    # is the child's rest direction in the joint's own frame and the up
    # vector its local Z (perpendicular: both children lie in local XY), so
    # the rest pose is the constraint's own fixed point. One chain each,
    # ordered so the ankle's revision reaches the ball's incoming frame.
    weight_source = None
    if blend is not None:
        bprim = stage.GetPrimAtPath(_path_of(blend))
        if not bprim or bprim.GetTypeName() != "RigExecBlendPointFrames":
            raise SystemExit("blend %s is not a RigExecBlendPointFrames"
                             % _path_of(blend))
        weight_source = str(bprim.GetPath().AppendProperty("inputs:weight"))
        made["blend"] = str(bprim.GetPath())

    # Added ball-first: the last chain added executes first, and the
    # ball's incoming frame must already carry the ankle's revision.
    for jname, cname, target_prim in ((ball, toe, toe_target),
                                      (ankle, ball, ball_target)):
        rest_local = worlds[cname] * worlds[jname].GetInverse()
        aim = rest_local.ExtractTranslation().GetNormalized()
        if abs(aim[2]) > 1e-6:
            raise SystemExit("%s: child is not in the joint's XY plane; "
                             "pick another up axis" % jname)
        chain = builder.new_mover_chain("%s_scAim" % jname)
        c = chain.add_aim_constraint("%s_aim" % jname,
                                     joint_prim_paths[jname],
                                     [str(target_prim.GetPath())])
        c.set_aim_vector(aim[0], aim[1], aim[2])
        c.set_up_vector(0.0, 0.0, 1.0)
        c.set_world_up_type("objectRotationUp")
        c.set_world_up_vector(0.0, 0.0, 1.0)
        c.set_world_up_object(str(target_prim.GetPath()))
        cprim = stage.GetPrimAtPath(c.path)
        if weight_source:
            _connect(cprim, "inputs:defaultWeight", weight_source)
        made["constraints"].append(c.path)
        made["aims"] = [c.path] + made.get("aims", [])

    # Last, so they execute first: the pivots, parents before children.
    _constrain_pivots(builder, stage, pending, made)

    if verbose:
        print("  foot_%s: %d pivots under %s, dials on %s, effector -> %s, "
              "aims gated by %s"
              % (side, len(made["pivots"]), ik_path.rsplit("/", 1)[-1],
                 made["controls"]["bank"].rsplit("/", 1)[-1],
                 made["targets"]["ankle"].rsplit("/", 1)[-1],
                 "blend weight" if weight_source else "nothing (always IK)"))
    return made


def add_fk_foot(stage, side, joint_prim_paths, reverse_foot=None,
                verbose=True):
    """The FK foot: ankle, ball, toe and the toe digits.

    A RigExecJoint carries its own avars and joints nest in namespace, so
    the FK foot needs no solver: posing `ball_?_bind.avars:*` carries the toe
    and every toe digit, and the ankle FK control the leg already has (its
    RigExecFkChain) carries the whole foot. This authors nothing structural
    for that.

    What it does author, given the reverse foot: Maya orient-constrains the
    ballRoll pivot to the ball FK control with maintainOffset, which is what
    lets the same ball control work in IK. Here the ball joint's rotation
    avars are CONNECTED (double -> double, verified honoured) into the
    `ballRoll_?_fk` frame, which carries the ball's rest orientation at the
    ballRoll pivot -- so in IK the toe target rotates with the ball's avars
    and the ball aim reproduces them exactly; in FK the avars act directly.
    Same control, same numbers, both modes.
    """
    ankle, ball, toe = foot_joints(side)
    for n in (ankle, ball, toe):
        if n not in joint_prim_paths:
            raise SystemExit("fk foot %s: missing %s" % (side, n))
    digits = sorted(n for n in joint_prim_paths
                    if n.startswith("toe") and n.endswith("_%s_bind" % side)
                    and n != toe)
    made = {"side": side, "ankle": joint_prim_paths[ankle],
            "ball": joint_prim_paths[ball], "toe": joint_prim_paths[toe],
            "digits": [joint_prim_paths[d] for d in digits],
            "connected": []}
    if reverse_foot is not None:
        fk = stage.GetPrimAtPath(reverse_foot["targets"]["ballFk"])
        bj = stage.GetPrimAtPath(joint_prim_paths[ball])
        order = bj.GetAttribute("avars:rotationOrder").Get()
        fk.GetAttribute("avars:rotationOrder").Set(order or "XYZ")
        for a in ("avars:rx", "avars:ry", "avars:rz"):
            _connect(fk, a, bj.GetPath().AppendProperty(a),
                     Sdf.ValueTypeNames.Double)
            made["connected"].append(a)
    if verbose:
        print("  fk foot_%s: ankle via the leg FK control, ball/toe via joint "
              "avars, %d toe digits%s"
              % (side, len(digits),
                 "; ball avars connected into the IK foot's ballRoll frame"
                 if reverse_foot else ""))
    return made


def apply_foot_shapes(stage, made, verbose=False):
    """Maya's shapes/colours/sizes on the foot's animator controls.

    control_shapes.CONTROL_MAP only knows the limb/girdle controls, so this
    does the same lookup for the foot's. `toeBend_?` is absent from
    control_positions.data (pruned from the delivered rig) and keeps a small
    cube.
    """
    from control_shapes import load_control_data, load_palette
    palette = load_palette()
    maya = load_control_data()
    side = made["side"]
    done = 0
    for role, maya_name in (("heel", "heel_%s" % side),
                            ("toe", "toe_%s" % side),
                            ("toeBend", "toeBend_%s" % side),
                            ("ballRoll", "ballRoll_%s" % side),
                            ("bank", "bank_%s" % side)):
        prim = stage.GetPrimAtPath(made["controls"][role])
        prim.CreateAttribute("guide:shape", Sdf.ValueTypeNames.Token, True,
                             Sdf.VariabilityUniform).Set("cube")
        rec = maya.get(maya_name)
        ext = rec["extent"] if rec else [2.0, 2.0, 2.0]
        for axis, value in zip("XYZ", ext):
            prim.CreateAttribute("guide:scale%s" % axis,
                                 Sdf.ValueTypeNames.Double).Set(
                                     float(value) if value > 1e-6 else 1.0)
        rgb = palette.get(rec["color"]) if rec else None
        if rgb:
            prim.CreateAttribute("guide:displayColor",
                                 Sdf.ValueTypeNames.Color3f).Set(
                                     Gf.Vec3f(*rgb))
        done += 1
        if verbose:
            print("    %-12s <- %-10s scale=(%.1f %.1f %.1f)%s"
                  % (prim.GetName(), maya_name, ext[0], ext[1], ext[2],
                     "" if rec else "  (no Maya data, default cube)"))
    return done


if __name__ == "__main__":
    # The full proof lives in verify_foot_fingers.py; this just shows the
    # pivot data resolves.
    data, _ = load_joints(os.path.join(DATA, "joint_positions.data"))
    for side in ("l", "r"):
        pw, pp = pivot_worlds(data, side)
        print("foot_%s pivots:" % side)
        for n in sorted(pw, key=lambda k: data[k]["dagPath"].count("|")):
            t = pw[n].ExtractTranslation()
            print("  %-20s parent=%-18s world=(%.3f %.3f %.3f)"
                  % (n, pp[n], t[0], t[1], t[2]))
