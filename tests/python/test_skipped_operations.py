#
# A broken operation is set aside, not fatal.
#
# The reported case: a rig layer opened on its own, before the model layer
# that supplies its geometry is composed in. Its mover targets a mesh that
# is not on this stage. That used to fail the whole compile, so nothing
# published and not even the controls drew. Now the mover is warned about
# and left out, and everything else -- the controls, and any operation that
# is fine -- compiles and evaluates. Supplying the missing geometry is a
# structural edit: the next evaluate recompiles and the mover runs.
#
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd, UsdGeom, Vt  # noqa: E402

RIG = "/puppet/RigRoot"
CONTROL = RIG + "/Controls/worldRoot"
BROKEN = RIG + "/Ops/world"
GOOD = RIG + "/Ops/card"
BODY = "/puppet/Geometry/Body.points"
CARD = "/puppet/Geometry/Card.points"
REST = [Gf.Vec3f(float(i), 0.0, 0.0) for i in range(3)]


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Mover(stage, path, target):
    mover = stage.DefinePrim(path, "RigExecMatrixMover")
    mover.AddAppliedSchema("RigExecMoverAPI")
    mover.GetRelationship("rigExec:moves").SetTargets([Sdf.Path(target)])
    mover.GetRelationship("rigExec:transform").SetTargets(
        [Sdf.Path(CONTROL)])


def _RigLayerAlone(withCard=False):
    """The rig layer as it is opened on its own: no Body on the stage."""
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/puppet", "Scope")
    stage.DefinePrim(RIG, "RigExecRoot")
    control = stage.DefinePrim(CONTROL, "RigExecControl")
    control.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    tx = control.GetAttribute("avars:tx")
    tx.Set(0.0, 1.0)
    tx.Set(1.0, 2.0)
    _Mover(stage, BROKEN, BODY)
    if withCard:
        card = UsdGeom.Points.Define(stage, "/puppet/Geometry/Card")
        card.CreatePointsAttr(Vt.Vec3fArray(REST))
        _Mover(stage, GOOD, CARD)
    return stage


def _Rig(stage, mode):
    import _rigexec
    rig = _rigexec.Rig(stage, RIG)
    rig.compile()
    rig.evaluation_mode = mode
    return rig


def TestTheControlStillDraws(mode):
    rig = _Rig(_RigLayerAlone(), mode)
    skipped = rig.skipped_operations()
    _Check(list(skipped) == [BROKEN],
           "%s: the mover with a missing target is set aside: %s"
           % (mode, skipped))
    _Check("missing prim /puppet/Geometry/Body" in skipped[BROKEN],
           "%s: and the reason is kept: %r" % (mode, skipped[BROKEN]))
    pose = rig.evaluate(2.0)
    _Check(pose.valid, "%s: the rig evaluates" % mode)
    _Check(list(pose.control_paths()) == [CONTROL],
           "%s: the control publishes, so it draws: %s"
           % (mode, pose.control_paths()))
    _Check(tuple(pose.control_frame(CONTROL).origin) == (1.0, 0.0, 0.0),
           "%s: and it poses" % mode)


def TestTheGoodOperationStillRuns(mode):
    rig = _Rig(_RigLayerAlone(withCard=True), mode)
    _Check(list(rig.skipped_operations()) == [BROKEN],
           "%s: only the broken mover is set aside: %s"
           % (mode, rig.skipped_operations()))
    _Check([m["path"] for m in rig.mover_order()] == [GOOD],
           "%s: the good one compiles: %s" % (mode, rig.mover_order()))
    pose = rig.evaluate(2.0)
    _Check(pose.baked_parity_mismatches == 0,
           "%s: baked and dynamic agree" % mode)
    got = [tuple(p) for p in pose.moved_property(CARD)]
    _Check(got == [(p[0] + 1.0, p[1], p[2]) for p in REST],
           "%s: and moves its card: %s" % (mode, got))


def TestSupplyingTheTargetBringsItBack(mode):
    '''Composing the geometry in is structural: the mover runs again.'''
    stage = _RigLayerAlone()
    rig = _Rig(stage, mode)
    _Check(list(rig.skipped_operations()) == [BROKEN],
           "%s: set aside while Body is missing" % mode)
    body = UsdGeom.Mesh.Define(stage, "/puppet/Geometry/Body")
    body.CreatePointsAttr(Vt.Vec3fArray(REST))
    body.CreateFaceVertexCountsAttr(Vt.IntArray([3]))
    body.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2]))
    pose = rig.evaluate(2.0)
    _Check(rig.skipped_operations() == {},
           "%s: nothing is skipped once Body exists: %s"
           % (mode, rig.skipped_operations()))
    got = [tuple(p) for p in pose.moved_property(BODY)]
    _Check(got == [(p[0] + 1.0, p[1], p[2]) for p in REST],
           "%s: and the body follows the control: %s" % (mode, got))


def TestARigWithNothingElseStillFails(mode):
    '''
    Setting the only operation aside leaves nothing to publish: that is
    the rig's failure, not the operation's, and it still fails -- saying
    why the operation went, not only that nothing is left.
    '''
    import _rigexec
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim(RIG, "RigExecRoot")
    _Mover(stage, BROKEN, BODY)
    rig = _rigexec.Rig(stage, RIG)
    try:
        rig.compile()
    except Exception as error:
        text = str(error)
        _Check("missing prim /puppet/Geometry/Body" in text,
               "%s: the operation's reason is reported: %s" % (mode, text))
        return
    raise AssertionError("%s: a rig with nothing to publish compiled" % mode)


def main():
    _RegisterSchema()
    tests = [
        ("the control still draws", TestTheControlStillDraws),
        ("the good operation still runs", TestTheGoodOperationStillRuns),
        ("supplying the target brings it back",
         TestSupplyingTheTargetBringsItBack),
        ("a rig with nothing else still fails",
         TestARigWithNothingElseStillFails),
    ]
    for mode in ("reference", "parity"):
        for name, fn in tests:
            fn(mode)
            print("  ok: %s %s" % (mode, name))
    print("RIGEXEC_SKIPPED_OPERATIONS_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
