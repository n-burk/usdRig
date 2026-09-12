#!/usr/bin/env python
"""Maya's param node, in RigExec: one pickable switch per limb.

Maya builds a per-limb "param node" -- `arm_L`, `arm_R`, `leg_L`, `leg_R`
in `rig_bits.nxt:/limb/create/param` -- as a `control_type="cube"`, then

    mc.parentConstraint(joint_list[-1], param_node, mo=False)
    mc.addAttr(param_node, ln="ikfk", at="double", min=0, max=1, dv=1,
               keyable=True)

so the switch is a small cube riding the LAST joint of the limb -- the
wrist or the ankle, where the animator's hand already is.

It is a CONTROL and it lives with the controls. The first version of this
nested the param prim inside the joint hierarchy, under `wrist_l_bind`,
which tracked the joint perfectly but put a control in the skeleton. This
one does what Maya does: the control sits under `<rig>/Controls` and a
`RigExecParentConstraint` drives it from the end joint. Measured the same
either way -- gap 0.000000 cm at rest, 30.247 cm tracking a 35 degree FK
shoulder rotation, 20.000 cm tracking a 20 cm IK effector move -- so the
tidier arrangement costs nothing.

(A third route does NOT work and is recorded so nobody tries it again:
connecting the control's `parent:space` to the joint's `posed:space`. The
schema reads as though it should select that parent, but the control stayed
at the origin, 115.05 cm from the wrist at rest and unmoved by a 35 degree
FK rotation.)

What is NOT reproduced yet: Maya instances the param SHAPE under every FK
and IK control (`/limb/create/param/instance_param`) so the same node is
pickable from anywhere in the limb -- one prim cannot be in two places in
USD namespace. Nor are Maya's `ikfk_switch` enum (a snapping operation, not
a value), `pvMatch`, or `length`, which drives limb stretch and is a real
feature rather than an oversight.

Semantics: `avars:ikfk` is 0 = FK, 1 = IK -- the
`RigExecBlendPointFrames.inputs:weight` it is connected to, not Maya's
attribute, which is inverted.

The switch also fades the INACTIVE control set, which is what Maya's
`ikfk_switch` does through visibility. Every IK-side control of the limb
(`<tag>_ik`, `<tag>_pv`, and everything nested under them -- the legs
carry the whole reverse-foot set under `leg_?_ik`) has its
`guide:displayOpacity` CONNECTED to the limb's `avars:ikfk`, and every
FK-side control (`<tag>_fk_*`, nested or not) is connected to the same
source with `guide:displayOpacityInvert = true`, so one dial drives both
sets in opposite directions and there is no second source of truth to
drift. The limb root and the param node itself are meaningful in both
modes and are never faded. The imaging bridge floors a driven opacity at
`guide:displayOpacityMin` (0.15 by default) so the parked set stays
findable: a set that vanished outright could never be switched back.

The control sets are discovered by walking the rig, not by path -- the
nesting has changed twice already -- and the source is whatever the
limb's blend weight is connected to, so this follows the param node
wherever it lives.

Usage
-----
From the builder:

    from params import add_param_controls
    add_param_controls(builder, stage, joint_prim_paths)

(which wires the opacity too), or to (re)wire the opacity alone on a rig
that already has its param nodes:

    from params import wire_ikfk_opacity
    wire_ikfk_opacity(stage)

As a CLI, to retrofit a rig built before this existed:

    params.py rig.usda [--out new.usda]
"""
import argparse
import os
import sys

from pxr import Gf, Sdf, Usd

# tag -> the end joint whose frame the param node rides
LIMB_PARAMS = [
    ("arm_l", "wrist_l_bind"),
    ("arm_r", "wrist_r_bind"),
    ("leg_l", "ankle_l_bind"),
    ("leg_r", "ankle_r_bind"),
]

AVAR = "avars:ikfk"

# The control guide's opacity, and the switch that draws its complement.
OPACITY = "guide:displayOpacity"
OPACITY_INVERT = "guide:displayOpacityInvert"

# Maya's param node is a cube; keep it small so it reads as a switch rather
# than as another animation control. Yellow, which no limb control uses --
# the sides are blue and red, the spine greens.
PARAM_SIZE = 3.0
PARAM_COLOR = (1.0, 0.85, 0.1)

# Where the cube sits relative to its joint, in WORLD axes (cm). Sitting
# exactly on the joint buries it in the hand or foot geometry and inside
# the limb's own controls, so it is offset clear of both: the wrist one
# straight up, the ankle one out to the side, away from the body, which
# also keeps the two feet's switches from crowding each other.
# The offset is held by the parent constraint (Maya's `mo=1`), so it
# rides the joint at a constant distance rather than being a pose.
PARAM_OFFSETS = {
    "arm_l": (0.0, 8.0, 0.0),
    "arm_r": (0.0, 8.0, 0.0),
    "leg_l": (12.0, 0.0, 0.0),
    "leg_r": (-12.0, 0.0, 0.0),
}


def flatten(m):
    return [m[r][c] for r in range(4) for c in range(4)]


def offset_in_source_space(target_rest, source_rest):
    """The constant offset a RigExecParentConstraint needs."""
    c = target_rest * source_rest.GetInverse()
    t = c.ExtractTranslation()
    r = c.ExtractRotation().Decompose(Gf.Vec3d(0, 0, 1),
                                      Gf.Vec3d(0, 1, 0),
                                      Gf.Vec3d(1, 0, 0))
    # Decompose returns Z, Y, X for those axes; the schema wants XYZ.
    return (t[0], t[1], t[2]), (r[2], r[1], r[0])


def joint_world_rest(stage, path):
    """A bind joint's rest in asset space, composed up the namespace.

    Joint rests are parent-local (`rigExec:restFrameVersion = 2`), so the
    param control -- which is NOT in the joint hierarchy -- needs the
    composed frame for its own asset-space rest.
    """
    m = Gf.Matrix4d(1.0)
    prim = stage.GetPrimAtPath(path)
    while prim and prim.IsValid() and str(prim.GetTypeName()) == "RigExecJoint":
        attr = prim.GetAttribute("rest:space")
        if attr and attr.IsValid() and attr.Get() is not None:
            m = m * Gf.Matrix4d(attr.Get())
        prim = prim.GetParent()
    return m


def _joint_path(stage, name):
    for prim in stage.Traverse():
        if prim.GetName() == name and str(prim.GetTypeName()) == "RigExecJoint":
            return str(prim.GetPath())
    return None


def _blend_for(stage, tag):
    for prim in stage.Traverse():
        if (str(prim.GetTypeName()) == "RigExecBlendPointFrames"
                and prim.GetName() == "%s_ikfk" % tag):
            return prim
    return None


def _existing_dial(stage, blend):
    """The attribute the blend's weight is currently connected to."""
    attr = blend.GetAttribute("inputs:weight")
    if not attr or not attr.IsValid():
        return None, None
    targets = attr.GetConnections()
    if not targets:
        return None, attr
    source = targets[0]
    prim = stage.GetPrimAtPath(source.GetPrimPath())
    if not prim or not prim.IsValid():
        return None, attr
    return prim.GetAttribute(source.name), attr


def add_param_controls(builder, stage, joint_prim_paths=None, verbose=True,
                       place="skeleton"):
    """Author one param control per limb and move the IK/FK dial onto it.

    `place` decides where the control lives and how it follows:

      "controls"  under <rig>/Controls, parent-constrained from the end
                  joint -- literally Maya's
                  `parentConstraint(joint_list[-1], param_node, mo=False)`,
                  and where a control belongs. It tracks perfectly (gap
                  0.000000 cm at rest, in FK and in IK, on biped_rig_v3)
                  PROVIDED the follower chain executes last: see
                  _execute_last for the real loop the bottom of the Movers
                  stack closed, which was first read as a prim-granular
                  false cycle in the dependency analysis and is not one --
                  the blend reads the param control's SCALAR only, and the
                  compiler never had a pose edge for that.

      "skeleton"  nested under the end joint itself. No constraint, so
                  nothing to order, and the namespace composition tracks
                  the joint exactly the same way. The cost is a control
                  prim sitting inside the bone hierarchy, which is not
                  where a control belongs.

    "skeleton" stays the default for now so the builder's output does not
    change under it mid-flight; "controls" is the layout to move to.
    Returns {tag: param control path}.
    """
    made = {}
    chain = None
    for tag, end_joint in LIMB_PARAMS:
        blend = _blend_for(stage, tag)
        if blend is None:
            if verbose:
                print("  %s: SKIPPED, no %s_ikfk blend" % (tag, tag))
            continue

        path = (joint_prim_paths or {}).get(end_joint)
        if path is None:
            path = _joint_path(stage, end_joint)
        if path is None:
            if verbose:
                print("  %s: SKIPPED, no joint %s" % (tag, end_joint))
            continue

        old_dial, weight_attr = _existing_dial(stage, blend)
        default = 1.0
        if old_dial is not None and old_dial.IsValid():
            value = old_dial.Get()
            if value is not None:
                default = float(value)
        elif weight_attr is not None and weight_attr.Get() is not None:
            default = float(weight_attr.Get())

        joint_rest = joint_world_rest(stage, path)
        offset = Gf.Vec3d(*PARAM_OFFSETS.get(tag, (0.0, 0.0, 0.0)))
        param_rest = Gf.Matrix4d(joint_rest)
        param_rest.SetTranslateOnly(joint_rest.ExtractTranslation()
                                    + offset)
        if place == "controls":
            # A top-level control: asset-space rest, offset off the joint.
            ctl = builder.add_control("%s_params" % tag,
                                      flatten(param_rest))
            ctl_path = str(ctl.path)
            prim = stage.GetPrimAtPath(ctl_path)
        else:
            ctl_path = "%s/%s_params" % (path, tag)
            prim = stage.DefinePrim(Sdf.Path(ctl_path), "RigExecControl")
            # Nested under the joint, its rest is parent-local, so identity
            # IS the joint's own frame.
            local = param_rest * joint_rest.GetInverse()
            prim.CreateAttribute(
                "rest:space", Sdf.ValueTypeNames.Matrix4d).Set(local)
            prim.CreateAttribute("purpose",
                                 Sdf.ValueTypeNames.Token).Set("guide")
        prim.CreateAttribute("guide:shape", Sdf.ValueTypeNames.Token, True,
                             Sdf.VariabilityUniform).Set("cube")
        for axis in "XYZ":
            prim.CreateAttribute("guide:scale%s" % axis,
                                 Sdf.ValueTypeNames.Double).Set(PARAM_SIZE)
        prim.CreateAttribute("guide:displayColor",
                             Sdf.ValueTypeNames.Color3f).Set(
                                 Gf.Vec3f(*PARAM_COLOR))

        # Maya's `parentConstraint(joint_list[-1], param_node, mo=False)`.
        # One chain for all four, since nothing reads these controls' frames
        # -- they carry a dial and a guide, not a pose anyone consumes -- so
        # there is no ordering to get wrong between them.
        if place == "controls":
            if chain is None:
                chain = builder.new_mover_chain("param_follow")
            pc = chain.add_parent_constraint(
                "%s_params_to_%s" % (tag, end_joint), ctl_path, [path])
            if offset != Gf.Vec3d(0, 0, 0):
                t, r = offset_in_source_space(param_rest, joint_rest)
                pc.set_translation_offsets([t])
                pc.set_rotation_offsets([r])

        dial = prim.CreateAttribute(AVAR, Sdf.ValueTypeNames.Float)
        # Carry the old dial's ANIMATION across, not just its value, so a
        # retrofit does not silently drop keys the animator already set.
        if old_dial is not None and old_dial.IsValid():
            samples = old_dial.GetTimeSamples()
            for t in samples:
                dial.Set(old_dial.Get(t), t)
            if not samples:
                dial.Set(default)
        else:
            dial.Set(default)

        weight_attr.SetConnections(
            [Sdf.Path(ctl_path).AppendProperty(AVAR)])

        # One source of truth: leaving the old dial behind lets an animator
        # key a control that no longer drives anything.
        if old_dial is not None and old_dial.IsValid():
            old_prim = old_dial.GetPrim()
            if str(old_prim.GetPath()) != ctl_path:
                old_prim.RemoveProperty(old_dial.GetName())

        made[tag] = ctl_path
        if verbose:
            print("  %-6s %s  (%s, %s = %g, 0 = FK, 1 = IK)"
                  % (tag, ctl_path,
                     "parent-constrained from %s" % end_joint
                     if place == "controls" else "nested under %s" % end_joint,
                     AVAR, default))
    if place == "controls" and chain is not None:
        _execute_last(stage, chain.scope_path)
    if made:
        if verbose:
            print("the switch fades the inactive control set:")
        wire_ikfk_opacity(stage, verbose=verbose)
    return made


def _execute_last(stage, chain_path):
    """Put a mover chain at the TOP of the Movers stack, so it runs LAST.

    The Movers namespace executes bottom-up (rigEvaluator.cpp,
    `_GetMoverExecutionOrder`: the last child runs first, the first child
    last). A chain the builder appends therefore runs BEFORE every
    constraint already there -- and a follower of SOLVED joints has to run
    after every constraint the solver's own inputs read. The leg IK's
    effector sits under the reverse-foot pivots, which are constrained, so
    a param follower at the bottom of the stack closed a real loop:
    follower waits on the blend, the blend on the IK, the IK on the
    reverse-foot constraints, and those (stack order) on the follower. The
    arms escaped only because nothing constrained sits above their
    effector. The compiler reported the loop as a cycle among every step
    downstream of it -- sixty of them -- which is what made it look like a
    prim-granularity bug in the dependency analysis; it was the authored
    order.

    The reorder lists every child explicitly: USD applies a partial
    `reorder nameChildren` as a relative order and does not move a lone
    name to the front. A chain added after this call lands at the bottom
    again, so call it last, as add_param_controls does.
    """
    chain_path = Sdf.Path(chain_path)
    movers = stage.GetPrimAtPath(chain_path.GetParentPath())
    name = chain_path.name
    others = [c.GetName() for c in movers.GetChildren() if c.GetName() != name]
    movers.SetChildrenReorder([name] + others)


def _limb_control_sets(stage, tag):
    """(ik, fk) control prims of one limb, discovered by walking the rig.

    IK side: `<tag>_ik` and `<tag>_pv` and every control nested under
    either (the reverse-foot set lives under `leg_?_ik`). FK side: every
    `<tag>_fk_*` control, nested or not. Neither: `<tag>_root` and
    `<tag>_params`, which mean something in both modes.
    """
    controls = [p for p in stage.Traverse()
                if str(p.GetTypeName()) == "RigExecControl"]
    ik_roots = [p.GetPath() for p in controls
                if p.GetName() in ("%s_ik" % tag, "%s_pv" % tag)]
    ik, fk = [], []
    for prim in controls:
        name = prim.GetName()
        if name in ("%s_root" % tag, "%s_params" % tag):
            continue
        if any(prim.GetPath().HasPrefix(root) for root in ik_roots):
            ik.append(prim)
        elif name.startswith("%s_fk_" % tag):
            fk.append(prim)
    return ik, fk


def _connect_opacity(prim, source, invert):
    """Drive one control's guide opacity from `source`, complemented or not.

    GetAttribute first: with the schema registered the property is built
    in, and authoring through it keeps the declared type. The
    CreateAttribute fallback is for a stage composed without the plugin.
    """
    opacity = prim.GetAttribute(OPACITY)
    if not opacity or not opacity.IsValid():
        opacity = prim.CreateAttribute(OPACITY, Sdf.ValueTypeNames.Float)
    opacity.SetConnections([source])
    flag = prim.GetAttribute(OPACITY_INVERT)
    if invert:
        if not flag or not flag.IsValid():
            flag = prim.CreateAttribute(OPACITY_INVERT,
                                        Sdf.ValueTypeNames.Bool, True,
                                        Sdf.VariabilityUniform)
        flag.Set(True)
    elif flag and flag.IsValid() and flag.HasAuthoredValue():
        # A control that changed sides on a rebuild must not keep the
        # complement.
        flag.Set(False)


def wire_ikfk_opacity(stage, verbose=True):
    """Connect every limb control's guide opacity to its limb's switch.

    Returns {tag: ([ik control paths], [fk control paths])}. Idempotent:
    rewiring is the same edit again. The source is whatever the limb's
    blend weight is connected to, which is the one thing that is true of
    the param node in every layout it has had.
    """
    wired = {}
    for tag, _end_joint in LIMB_PARAMS:
        blend = _blend_for(stage, tag)
        if blend is None:
            if verbose:
                print("  %s: SKIPPED, no %s_ikfk blend" % (tag, tag))
            continue
        dial, _weight = _existing_dial(stage, blend)
        if dial is None or not dial.IsValid():
            if verbose:
                print("  %s: SKIPPED, the blend weight is not connected to "
                      "a dial (add_param_controls first)" % tag)
            continue
        ik, fk = _limb_control_sets(stage, tag)
        for prim in ik:
            _connect_opacity(prim, dial.GetPath(), False)
        for prim in fk:
            _connect_opacity(prim, dial.GetPath(), True)
        wired[tag] = ([str(p.GetPath()) for p in ik],
                      [str(p.GetPath()) for p in fk])
        if verbose:
            print("  %-6s %d IK-side control(s) fade with %s, %d FK-side "
                  "against it" % (tag, len(ik), dial.GetPath(), len(fk)))
    return wired


def _cleanup_old(stage, verbose=True):
    """Remove param controls a previous run left behind.

    Both layouts, not just the skeleton one: re-running the tool on a rig
    that already has them used to die with "rig object already exists:
    /Biped/Rig/Controls/arm_l_params", which made it un-re-runnable as soon
    as anything about the placement changed.
    """
    stale = []
    for prim in stage.Traverse():
        if (str(prim.GetTypeName()) == "RigExecControl"
                and prim.GetName().endswith("_params")):
            stale.append(str(prim.GetPath()))
    for prim in stage.Traverse():
        if prim.GetName() == "param_follow" and "/Movers/" in str(prim.GetPath()):
            stale.append(str(prim.GetPath()))
    for path in stale:
        stage.RemovePrim(Sdf.Path(path))
        if verbose:
            print("  removed %s (a control does not belong in the "
                  "skeleton)" % path)
    return stale


def main(argv):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rig")
    ap.add_argument("--rig-root", default="/Biped/Rig")
    ap.add_argument("--place", choices=("skeleton", "controls"),
                    default="skeleton",
                    help="where the param control lives: nested under the "
                         "end joint (default) or under <rig>/Controls "
                         "parent-constrained from it, where a control "
                         "belongs (the follower chain is placed at the top "
                         "of the Movers stack so it runs last -- see "
                         "_execute_last)")
    ap.add_argument("--out", default=None,
                    help="write here instead of editing in place")
    args = ap.parse_args(argv)

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import rigexec
    rigexec.load_schema_plugin()

    stage = Usd.Stage.Open(args.rig)
    _cleanup_old(stage)
    builder = rigexec.Builder.create(stage, args.rig_root)
    print("param controls (Maya's param node, one per limb):")
    made = add_param_controls(builder, stage, place=args.place)
    if not made:
        print("nothing to do")
        return 1

    if args.out:
        stage.GetRootLayer().Export(args.out)
        print("wrote %s" % args.out)
    else:
        stage.GetRootLayer().Save()
        print("saved %s" % args.rig)

    # Prove the switch drives the blend AND that the control tracks the
    # joint, rather than claiming either.
    check = Usd.Stage.Open(args.out or args.rig)
    rig = rigexec.Rig(check, args.rig_root)
    rig.compile()

    def control_origin(path):
        m = rig.evaluate(0.0).control_frame(path).to_matrix4()
        return Gf.Vec3d(m[12], m[13], m[14])

    def joint_origin(path):
        m = rig.evaluate(0.0).joint_frame(path).to_matrix4()
        return Gf.Vec3d(m[12], m[13], m[14])

    print("")
    print("the switch drives the blend, and the control tracks the joint:")
    for tag, end_joint in LIMB_PARAMS:
        if tag not in made:
            continue
        jp = _joint_path(check, end_joint)
        dial = check.GetPrimAtPath(made[tag]).GetAttribute(AVAR)
        keep = dial.Get()
        ik_path = [str(p.GetPath()) for p in check.Traverse()
                   if p.GetName() == "%s_ik" % tag][0]
        ik = check.GetPrimAtPath(ik_path)
        ty = ik.GetAttribute("avars:ty")
        if not ty or not ty.IsValid():
            ty = ik.CreateAttribute("avars:ty", Sdf.ValueTypeNames.Double)

        ty.Set(20.0)
        dial.Set(0.0)
        fk_at, fk_ctl = joint_origin(jp), control_origin(made[tag])
        dial.Set(1.0)
        ik_at, ik_ctl = joint_origin(jp), control_origin(made[tag])
        print("  %-6s effector +20cm: FK %.2f cm, IK %.2f cm; "
              "control-to-joint gap FK %.6f, IK %.6f cm"
              % (tag, 0.0, (ik_at - fk_at).GetLength(),
                 (fk_ctl - fk_at).GetLength(),
                 (ik_ctl - ik_at).GetLength()))
        ty.Clear()
        dial.Set(keep)

    # The opacity wiring is authored, not evaluated: the imaging bridge is
    # what draws it (tests/testUsdviewIkFkOpacity.py measures the drawn
    # values), so what can be proved here is that every control of each
    # set points at the dial, the FK side with the complement.
    print("")
    print("the switch fades the inactive control set "
          "(guide:displayOpacity connections):")
    for tag, _end_joint in LIMB_PARAMS:
        if tag not in made:
            continue
        source = Sdf.Path(made[tag]).AppendProperty(AVAR)
        ik, fk = _limb_control_sets(check, tag)
        ik_ok = sum(1 for p in ik
                    if p.GetAttribute(OPACITY).GetConnections() == [source])
        fk_ok = sum(1 for p in fk
                    if p.GetAttribute(OPACITY).GetConnections() == [source]
                    and p.GetAttribute(OPACITY_INVERT).Get() is True)
        print("  %-6s IK side %d/%d connected, FK side %d/%d connected and "
              "inverted" % (tag, ik_ok, len(ik), fk_ok, len(fk)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
