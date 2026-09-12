#!/usr/bin/env python
"""Set or KEY the IK/FK blend on the biped's limbs.

Why this exists: the dial is `avars:ikfk` on each limb's root control
(`arm_l_root`, `arm_r_root`, `leg_l_root`, `leg_r_root`), and the blend
solver's `inputs:weight` is connected to it. 0 is FK, 1 is IK, and it
interpolates linearly in between. Setting a constant value is easy enough
by hand, but *animating* it means authoring timeSamples, and usdview is a
viewer -- its property panel will not key an attribute for you. So this
does it.

The value is the RigExecBlendPointFrames weight, not Maya's `ikfk`
attribute, which is inverted (1 = FK there). 0 = FK, 1 = IK, everywhere in
this port.

Usage
-----
Constant, one limb or several:

    set_ikfk.py rig.usda --limb arm_l --value 1
    set_ikfk.py rig.usda --limb arm_l --limb arm_r --value 1
    set_ikfk.py rig.usda --all --value 0

Key a transition -- frame:value pairs, in order:

    set_ikfk.py rig.usda --limb leg_l --key 1:1 --key 24:1 --key 48:0

Scrub the timeline in usdview and the limb hands over from IK to FK
between frames 24 and 48. Without `--out` the stage is edited in place;
`--out` writes a copy and leaves the input alone.

`--show` prints the current state of every dial and does nothing else.
"""
import argparse
import sys

from pxr import Sdf, Usd

LIMBS = ("arm_l", "arm_r", "leg_l", "leg_r")
AVAR = "avars:ikfk"


def dial(stage, limb, rig_root):
    """The dial attribute for one limb, or None with a reason printed."""
    path = "%s/Controls/%s_root" % (rig_root, limb)
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        print("  %-6s no control at %s" % (limb, path))
        return None
    attr = prim.GetAttribute(AVAR)
    if not attr or not attr.IsValid():
        # A rig built without --no-blend always has it; a rig built with
        # the blend omitted has no blend to drive, so creating one here
        # would be a dial wired to nothing.
        print("  %-6s has no %s (built without the IK/FK blend?)"
              % (limb, AVAR))
        return None
    return attr


def show(stage, rig_root):
    print("IK/FK dials (0 = FK, 1 = IK):")
    for limb in LIMBS:
        attr = dial(stage, limb, rig_root)
        if attr is None:
            continue
        times = attr.GetTimeSamples()
        if times:
            keys = ", ".join("%g:%g" % (t, attr.Get(t)) for t in times)
            print("  %-6s keyed  %s" % (limb, keys))
        else:
            print("  %-6s %g" % (limb, attr.Get()))
    return 0


def main(argv):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rig")
    ap.add_argument("--rig-root", default="/Biped/Rig")
    ap.add_argument("--limb", action="append", choices=LIMBS, default=[],
                    help="limb to change; repeatable")
    ap.add_argument("--all", action="store_true", help="every limb")
    ap.add_argument("--value", type=float, default=None,
                    help="constant blend value, 0 = FK, 1 = IK")
    ap.add_argument("--key", action="append", default=[], metavar="FRAME:VAL",
                    help="a timeSample; repeatable, in order")
    ap.add_argument("--out", default=None,
                    help="write here instead of editing in place")
    ap.add_argument("--show", action="store_true",
                    help="print the current dials and exit")
    args = ap.parse_args(argv)

    stage = Usd.Stage.Open(args.rig)
    if args.show:
        return show(stage, args.rig_root)

    limbs = list(LIMBS) if args.all else args.limb
    if not limbs:
        ap.error("pass --limb (repeatable) or --all")
    if (args.value is None) == (not args.key):
        ap.error("pass exactly one of --value or --key")

    keys = []
    for spec in args.key:
        if ":" not in spec:
            ap.error("--key wants FRAME:VALUE, got %r" % spec)
        frame, value = spec.split(":", 1)
        try:
            keys.append((float(frame), float(value)))
        except ValueError:
            ap.error("--key wants numbers, got %r" % spec)

    changed = 0
    for limb in limbs:
        attr = dial(stage, limb, args.rig_root)
        if attr is None:
            continue
        if args.value is not None:
            # Clear any keys first, or a leftover timeSample would win over
            # the default value at every frame and the "constant" would not
            # be constant.
            attr.Clear()
            attr.Set(float(args.value))
            print("  %-6s = %g" % (limb, args.value))
        else:
            attr.Clear()
            for frame, value in keys:
                attr.Set(float(value), frame)
            print("  %-6s keyed %s"
                  % (limb, ", ".join("%g:%g" % k for k in keys)))
        changed += 1

    if not changed:
        print("nothing changed")
        return 1

    if args.out:
        stage.GetRootLayer().Export(args.out)
        print("wrote %s" % args.out)
    else:
        stage.GetRootLayer().Save()
        print("saved %s" % args.rig)

    if keys:
        lo = min(k[0] for k in keys)
        hi = max(k[0] for k in keys)
        # Without a time range usdview shows no timeline to scrub, so the
        # keys would be invisible even though they are there.
        stage.SetStartTimeCode(min(lo, stage.GetStartTimeCode() or lo))
        stage.SetEndTimeCode(max(hi, stage.GetEndTimeCode() or hi))
        (stage.GetRootLayer().Export(args.out) if args.out
         else stage.GetRootLayer().Save())
        print("  time range %g to %g" % (stage.GetStartTimeCode(),
                                         stage.GetEndTimeCode()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
