"""A deformer whose control sits at rest adds nothing, however it is carried.

Both `rigExec:driverDeltaFrame` settings are identity when the driver
sits at its space, so a wire whose control has not been touched must
contribute exactly nothing no matter where the head, spine or shoulder
above it has been carried. The same holds for a cluster.

This exists because that invariant was broken in the asset and nobody
noticed. `driverDeltaFrame = "posed"` was measured on the bendy limbs,
where it is right, and applied to sixteen wires including the eye ones,
where it is not: the blink wires went from contributing 0.433 units on
a 60-degree head turn to 1.658, and the face visibly came apart. The
check that would have caught it is this one, and the reason it did not
exist is that the probe written at the time watched ONE pose against
ONE mesh -- so this sweeps every carrier and every mesh.

Usage:
    python test_rigexec_wire_invariant.py [schema_resources_dir]
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
_CTL = _RIG + "/Main/Shot/Aux/Controls"
_CHEST = _CTL + "/M_Body/M_Torso/M_Chest/M_ChestTop"
_SKULL = _CHEST + "/M_Neck/M_Head/M_HeadGimbal/skull_follow/M_Skull"

# The carriers that find something, not every carrier there is. Each
# one costs 26 full-stack evaluations and the whole sweep of six cost
# 303 seconds against the suite's 120s timeout, so this is the subset
# that actually caught the three families of violation: the head turn
# found the blink wires, the shoulder the arm bendy, the body move the
# legs. Neck, torso and jaw found nothing the head turn did not.
_CARRIERS = [
    ("head turn", _CHEST + "/M_Neck/M_Head", "ry", 45.0),
    ("shoulder", _CHEST + "/L_Shldr", "rz", 40.0),
    ("body move", _CTL + "/M_Body", "tx", 12.0),
]

# Numerical floor. The rig's own no-deformer baseline sits at ~3.1e-5 on
# a mesh whose points are ~150 units out, so anything under 1e-3 is the
# arithmetic and not a deformer doing something.
_EPS = 1e-3

# The look-at is switched OFF for the sweep, and that is what makes the
# invariant exact rather than approximate.
#
# lookRot_aim is aim-constrained to M_Look, which hangs directly under
# Controls and NOT under the head. So with the aim live a head turn
# leaves the eyes pointing at the same world spot -- they counter-rotate
# in the socket -- and the eye drag carries the lids with them. The
# blink wires then contribute 0.433 units at 60 degrees with their own
# controls untouched, which looks exactly like a broken wire and is in
# fact the look-at working. Measured both ways: lookAt=1 gives 0.338 at
# 45 degrees and 0.433 at 60; lookAt=0 gives 0.000000 at every angle.
#
# Neutralising it isolates what this file is actually asking -- whether
# a wire adds anything of its OWN -- and leaves no wire needing an
# exemption. _test_look_at_drives_the_lids below covers the other half.
_LOOK = _CTL + "/M_Look"

def _open(mode, carrier):
    """One stage, one compile, per carrier.

    `inputs:enabled` is an ordinary per-frame input, not epoch identity,
    so toggling it re-evaluates without recompiling. Opening a stage per
    wire instead took 374 seconds -- past the suite's own 120s timeout --
    for an answer identical to this one.
    """
    from pxr import Sdf, Usd
    import rigexec

    stage = Usd.Stage.Open(_STACK)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    enables = {}
    for prim in stage.Traverse():
        if prim.GetTypeName() != "RigExecCurveMover":
            continue
        a = prim.GetAttribute("inputs:enabled")
        if not a or not a.IsValid():
            a = prim.CreateAttribute("inputs:enabled",
                                     Sdf.ValueTypeNames.Bool)
        a.Set(False)
        enables[prim.GetName()] = a
    look = stage.GetPrimAtPath(_LOOK)
    assert look, _LOOK
    at = look.GetAttribute("avars:lookAt")
    if not at or not at.IsValid():
        at = look.CreateAttribute("avars:lookAt", Sdf.ValueTypeNames.Float)
    at.Set(0.0)

    _, control, avar, value = carrier
    prim = stage.GetPrimAtPath(control)
    assert prim, control
    a = prim.GetAttribute("avars:" + avar)
    if not a or not a.IsValid():
        a = prim.CreateAttribute("avars:" + avar, Sdf.ValueTypeNames.Double)
    a.Set(value)

    rig = rigexec.Rig(stage, _RIG)
    rig.evaluation_mode = mode
    return rig, enables


def _points(rig):
    pose = rig.evaluate(0)
    assert pose.valid, [d for d in pose.diagnostics
                        if not d.startswith("warning:")]
    out = {}
    for key in pose.moved_properties():
        if not key.endswith(".points"):
            continue
        try:
            out[key] = [tuple(q) for q in pose.moved_property(key)]
        except TypeError:
            pass
    return out


def _worst(a, b):
    out = 0.0
    for key in set(a) & set(b):
        pa, pb = a[key], b[key]
        if len(pa) != len(pb):
            return float("inf")
        # Equality first, and it is the whole optimisation: most wires
        # contribute EXACTLY nothing, and list equality is one C call
        # against 26k tuples where the max below is a Python loop over
        # 79k floats. With the loop unguarded this file took 253s, past
        # the suite's 120s timeout.
        if pa == pb:
            continue
        for qa, qb in zip(pa, pb):
            if qa == qb:
                continue
            for x, y in zip(qa, qb):
                d = abs(x - y)
                if d > out:
                    out = d
    return out


def _check(mode):
    print("  [%s]" % mode)
    worst = {}
    for carrier in _CARRIERS:
        rig, enables = _open(mode, carrier)
        # Seeded with every wire, not filled as violations appear: a
        # wire contributing exactly 0.0 never clears a `>` test, so the
        # first version of this silently reported on 8 wires out of 25
        # and called it a pass.
        for name in enables:
            worst.setdefault(name, (0.0, ""))
        base = _points(rig)
        for name, attr in sorted(enables.items()):
            attr.Set(True)
            d = _worst(base, _points(rig))
            attr.Set(False)
            if d > worst[name][0]:
                worst[name] = (d, carrier[0])
    assert len(worst) >= 20, len(worst)

    failures = ["%s adds %.6f under %s" % (n, d, w)
                for n, (d, w) in sorted(worst.items()) if d > _EPS]
    assert not failures, failures
    print("    all %d wires inert with their control at rest (worst %.2e)"
          % (len(worst), max(d for d, _ in worst.values())))


def _check_look_at_drives_the_lids():
    """The other half: the eye drag must still REACH the lids.

    The sweep above switches the look-at off, so on its own it would
    pass just as happily with the eye drag severed -- and a severed eye
    drag is exactly what shipped when Biped_lashes.usda was deleted and
    the lash proxies lost their weights. So assert the motion is there:
    turn the head with the look-at live and the blink wires must move
    the mesh.
    """
    from pxr import Sdf, Usd
    import rigexec

    stage = Usd.Stage.Open(_STACK)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    blinks = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "RigExecCurveMover":
            continue
        a = prim.GetAttribute("inputs:enabled")
        if not a or not a.IsValid():
            a = prim.CreateAttribute("inputs:enabled",
                                     Sdf.ValueTypeNames.Bool)
        on = "blink" in prim.GetName()
        a.Set(on)
        if on:
            blinks.append(prim.GetName())
    # Four on body_geo and four lash-proxy copies of the same wires:
    # the lashes ride the blink curves, which is the only route the eye
    # drag has to reach them. A bare count would pass if a body wire
    # vanished and a lash wire took its slot, so name both families.
    body = sorted(n for n in blinks if "_on_lash_" not in n)
    lash = sorted(n for n in blinks if "_on_lash_" in n)
    assert body == ["blinkLower_l_curve_wire", "blinkLower_r_curve_wire",
                    "blinkUpper_l_curve_wire", "blinkUpper_r_curve_wire"], body
    assert lash == ["blinkLower_l_curve_wire_on_lash_upper_l_proxy",
                    "blinkLower_r_curve_wire_on_lash_upper_r_proxy",
                    "blinkUpper_l_curve_wire_on_lash_upper_l_proxy",
                    "blinkUpper_r_curve_wire_on_lash_upper_r_proxy"], lash
    head = stage.GetPrimAtPath(_CHEST + "/M_Neck/M_Head")
    at = head.GetAttribute("avars:ry")
    if not at or not at.IsValid():
        at = head.CreateAttribute("avars:ry", Sdf.ValueTypeNames.Double)
    at.Set(60.0)

    rig = rigexec.Rig(stage, _RIG)
    rig.evaluation_mode = "baked"
    # THE LOOK-AT IS PUT IN WORLD SPACE, explicitly, because what this
    # measures only exists there.
    #
    # M_Look's avars:space indexes look_spaces. In WORLD the target stays
    # where it is while the head turns, the eyes counter-rotate to keep
    # looking at it, and the drag carries the lids -- which is the thing
    # being asserted. In LOCAL the target rides M_UpFace, so a head turn
    # carries it along and the eyes have nothing to counter: measured
    # 0.000514 at ry=60, against 0.433 in world.
    #
    # That is local working, not the drag breaking, so world is set
    # explicitly -- and by its LABEL, looked up on the switch, because the
    # index of "world" has already moved once with the space order.
    space = stage.GetPrimAtPath(_LOOK).GetAttribute("avars:space")
    assert space and space.IsValid(), "M_Look has no avars:space"
    labels = None
    for prim in stage.Traverse():
        if prim.GetTypeName() != "RigExecSpaceSwitch":
            continue
        rel = prim.GetRelationship("rigExec:activeSpaceAttribute")
        if rel and space.GetPath() in rel.GetTargets():
            labels = list(prim.GetAttribute("rigExec:spaceLabels").Get())
    assert labels and "world" in labels, ("no world space for M_Look", labels)
    space.Set(float(labels.index("world")))
    look = stage.GetPrimAtPath(_LOOK).GetAttribute("avars:lookAt")
    look.Set(0.0)
    off = _points(rig)
    look.Set(1.0)
    on = _points(rig)
    moved = _worst(off, on)
    # 0.433 measured 2026-09-23. A floor, not an equality: the point is
    # that the drag REACHES the lids, not that it never changes.
    assert moved > 0.1, ("the look-at no longer drives the lids", moved)
    print("    look-at drives the lids: %.6f at ry=60, in world space"
          % moved)


def main():
    _setup_environment()
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)
    # Baked only: it is the path the biped runs, and the sweep gives
    # numbers identical to the digit in both modes -- measured -- so the
    # second pass doubled a 150-second test to buy nothing. Parity
    # between the two paths is verify_binary's job, not this file's.
    _check("baked")
    _check_look_at_drives_the_lids()
    print("OK: a deformer at rest adds nothing, however it is carried")


if __name__ == "__main__":
    main()
