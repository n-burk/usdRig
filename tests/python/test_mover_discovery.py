#
# Movers are discovered by WHAT they are, not where they sit.
#
# Carrying rigExec:moves is what makes a prim a mover. `Movers` is the
# conventional scope for them, as `Solvers` is for solvers, and nothing
# more: a mover under `Ops`, straight under the rig root, or nested under a
# control is found and ordered the same way (spec §4.2). The order is the
# one walk over the whole rig -- bottom composed sibling first -- so two
# movers in two different scopes stack by where their scopes sit. And the
# structure digest takes the same walk as compile: adding a mover outside
# `Movers` after a compile must recompile, not keep running the old stack.
#
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd, UsdGeom, Vt  # noqa: E402

POINTS = "/Asset/Geom/cloud.points"
REST = [Gf.Vec3f(float(i), 1.0, 0.0) for i in range(4)]


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Stage(scopes):
    """
    A rig whose root holds `scopes` in that composed order, each an empty
    Scope, plus a Controls scope first. Two controls, identity rests:
    `shift` translates +1 in X at time 2, `grow` scales X by 2 at time 2.
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Controls", "Scope")
    for name in scopes:
        stage.DefinePrim("/Asset/Rig/" + name, "Scope")
    for name in ("shift", "grow"):
        control = stage.DefinePrim("/Asset/Rig/Controls/" + name,
                                   "RigExecControl")
        control.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    shift = stage.GetAttributeAtPath("/Asset/Rig/Controls/shift.avars:tx")
    shift.Set(0.0, 1.0)
    shift.Set(1.0, 2.0)
    grow = stage.GetAttributeAtPath("/Asset/Rig/Controls/grow.avars:sx")
    grow.Set(1.0, 1.0)
    grow.Set(2.0, 2.0)
    cloud = UsdGeom.Points.Define(stage, "/Asset/Geom/cloud")
    cloud.CreatePointsAttr(Vt.Vec3fArray(REST))
    return stage


def _Mover(stage, path, control):
    mover = stage.DefinePrim(path, "RigExecMatrixMover")
    mover.AddAppliedSchema("RigExecMoverAPI")
    mover.GetRelationship("rigExec:moves").SetTargets([Sdf.Path(POINTS)])
    mover.GetRelationship("rigExec:transform").SetTargets(
        [Sdf.Path("/Asset/Rig/Controls/" + control)])
    return mover


def _Points(rig, time, mode):
    pose = rig.evaluate(time)
    _Check(pose.valid, "%s: pose at %g is valid" % (mode, time))
    _Check(pose.baked_parity_mismatches == 0,
           "%s: baked and dynamic agree at %g" % (mode, time))
    moved = pose.moved_property(POINTS)
    return [tuple(round(c, 5) for c in p) for p in moved]


def _Rig(stage, mode):
    import _rigexec
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    _Check(rig.compile() is not False, "%s: compiles" % mode)
    rig.evaluation_mode = mode
    return rig


def TestAMoverOutsideMoversIsDiscovered(mode):
    '''The reported case: a matrix mover under `Ops`, not `Movers`.'''
    stage = _Stage(["Ops"])
    _Mover(stage, "/Asset/Rig/Ops/world", "shift")
    rig = _Rig(stage, mode)
    _Check([m["path"] for m in rig.mover_order()] == ["/Asset/Rig/Ops/world"],
           "%s: the mover is discovered: %s" % (mode, rig.mover_order()))
    got = _Points(rig, 2.0, mode)
    _Check(got == [(p[0] + 1.0, p[1], p[2]) for p in REST],
           "%s: the points follow the control: %s" % (mode, got))


def TestAnywhereUnderTheRigIsDiscovered(mode):
    '''Straight under the rig root, and nested under a control.'''
    stage = _Stage([])
    _Mover(stage, "/Asset/Rig/atRoot", "shift")
    _Mover(stage, "/Asset/Rig/Controls/grow/follow", "grow")
    rig = _Rig(stage, mode)
    found = sorted(m["path"] for m in rig.mover_order())
    _Check(found == ["/Asset/Rig/Controls/grow/follow", "/Asset/Rig/atRoot"],
           "%s: both are discovered: %s" % (mode, found))


def TestScopesStackInWalkOrder(mode):
    '''
    Two movers on one target in two scopes: the bottom scope's runs first.
    Scale-then-translate and translate-then-scale differ, so the points
    say which order ran -- and swapping the scopes swaps it.
    '''
    for scopes, first, want in (
            (["A", "B"], "B", lambda p: (2.0 * p[0] + 1.0, p[1], p[2])),
            (["B", "A"], "A", lambda p: (2.0 * (p[0] + 1.0), p[1], p[2]))):
        stage = _Stage(scopes)
        _Mover(stage, "/Asset/Rig/A/shift", "shift")
        _Mover(stage, "/Asset/Rig/B/grow", "grow")
        rig = _Rig(stage, mode)
        order = [m["path"] for m in rig.mover_order()]
        _Check(order[0].startswith("/Asset/Rig/%s/" % first),
               "%s: with scopes %s the bottom one runs first: %s"
               % (mode, scopes, order))
        got = _Points(rig, 2.0, mode)
        _Check(got == [want(p) for p in REST],
               "%s: with scopes %s the points are %s" % (mode, scopes, got))


def TestAMoverAddedOutsideMoversRecompiles(mode):
    '''
    A structural edit anywhere under the rig reaches the next evaluate: the
    digest takes the same whole-rig walk as compile. Were it still fenced
    to `Movers`, the new mover would never run.
    '''
    stage = _Stage(["Ops"])
    rig = _Rig(stage, mode)
    _Check(rig.mover_order() == [], "%s: no movers yet" % mode)
    _Mover(stage, "/Asset/Rig/Ops/world", "shift")
    got = _Points(rig, 2.0, mode)
    _Check(got == [(p[0] + 1.0, p[1], p[2]) for p in REST],
           "%s: the mover added after compile runs: %s" % (mode, got))
    stage.GetPrimAtPath("/Asset/Rig/Ops/world").SetActive(False)
    pose = rig.evaluate(2.0)
    _Check(POINTS not in pose.moved_properties() and rig.mover_order() == [],
           "%s: and deactivating it stops it -- nothing writes the points "
           "any more: %s" % (mode, rig.mover_order()))


def main():
    _RegisterSchema()
    tests = [
        ("a mover outside Movers is discovered",
         TestAMoverOutsideMoversIsDiscovered),
        ("anywhere under the rig is discovered",
         TestAnywhereUnderTheRigIsDiscovered),
        ("scopes stack in walk order", TestScopesStackInWalkOrder),
        ("a mover added outside Movers recompiles",
         TestAMoverAddedOutsideMoversRecompiles),
    ]
    for mode in ("reference", "parity"):
        for name, fn in tests:
            fn(mode)
            print("  ok: %s %s" % (mode, name))
    print("RIGEXEC_MOVER_DISCOVERY_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
