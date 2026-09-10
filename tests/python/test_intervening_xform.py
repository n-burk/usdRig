#!/usr/bin/env python
"""
Headless test: an Xformable between the asset root and a RigExec provider
is composed into that provider's frames.

Exec resolves a provider's parent space through a NamespaceAncestor that
only RigExec types satisfy, so a `Scope` is correctly skipped and an
`Xform` used to be silently dropped with it -- rotating it moved nothing.
RigExecRigEvaluator::_ComposeInterveningXforms folds it back in, at
evaluation, from the stage. Nothing is authored: the rig follows the
Xform wherever the author put it, and no layer is rewritten.

See docs/superpowers/specs/2026-09-09-intervening-xform-design.md.

Usage: test_intervening_xform.py [<generated schema resources dir>]
Requires the native _rigexec binding (build/python on PYTHONPATH).
"""
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd, UsdGeom  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered")


# --------------------------------------------------------------------
# Fixtures. Built in memory rather than read from examples/, so the test
# says what it depends on and no shipped asset can drift underneath it.
# --------------------------------------------------------------------

def _Joint(stage, path, tx=0.0, ty=0.0, tz=0.0):
    prim = stage.DefinePrim(path, "RigExecJoint")
    for name, value in (("rest:tx", tx), ("rest:ty", ty), ("rest:tz", tz)):
        prim.CreateAttribute(name, Sdf.ValueTypeNames.Double).Set(value)
    return prim


def _Control(stage, path, tx=0.0, ty=0.0, tz=0.0):
    prim = stage.DefinePrim(path, "RigExecControl")
    for name, value in (("rest:tx", tx), ("rest:ty", ty), ("rest:tz", tz)):
        prim.CreateAttribute(name, Sdf.ValueTypeNames.Double).Set(value)
    return prim


def _Stage(midTranslate=None, innerTranslate=None):
    """
    /Asset/Rig/[Mid/]Joints/Root/[Inner/]Child plus a control.

    `Mid` sits between the rig root and every provider; `Inner` sits
    between two joints of one chain, which is the case a uniform
    right-multiply gets wrong.
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset", "Xform")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")

    base = "/Asset/Rig"
    if midTranslate is not None:
        base = "/Asset/Rig/Mid"
        UsdGeom.Xformable(stage.DefinePrim(base, "Xform")) \
            .AddTranslateOp().Set(Gf.Vec3d(*midTranslate))

    stage.DefinePrim(base + "/Joints", "Scope")
    _Joint(stage, base + "/Joints/Root", ty=2.0)

    childParent = base + "/Joints/Root"
    if innerTranslate is not None:
        childParent = base + "/Joints/Root/Inner"
        UsdGeom.Xformable(stage.DefinePrim(childParent, "Xform")) \
            .AddTranslateOp().Set(Gf.Vec3d(*innerTranslate))
    _Joint(stage, childParent + "/Child", tx=3.0)

    stage.DefinePrim(base + "/Controls", "Scope")
    _Control(stage, base + "/Controls/Handle", tx=5.0)
    return stage, "/Asset/Rig"


def _Origins(stage, rigPath):
    """
    {leaf name: asset-space origin} for every provider in the rig.

    Read from the FRAME, not from joint_matrix: that one is the
    rest-to-pose delta, which is identity on a rig with no solver and no
    avars and would report every joint at the origin.
    """
    import _rigexec
    rig = _rigexec.Rig(stage, rigPath)
    rig.compile()
    pose = rig.evaluate(1.0)
    out = {}
    for path in pose.joint_paths():
        out[path.rsplit("/", 1)[1]] = Gf.Vec3d(
            *pose.joint_frame(path, True).to_matrix4()[12:15])
    for path in pose.control_paths():
        out[path.rsplit("/", 1)[1]] = Gf.Vec3d(
            *pose.control_frame(path).to_matrix4()[12:15])
    return out


def _Close(a, b, tolerance=1e-9):
    return all(abs(a[i] - b[i]) < tolerance for i in range(3))


# --------------------------------------------------------------------


def TestNoInterveningXformIsUnchanged():
    """
    The shape every shipped example has must not move by a floating-point
    hair. The composition is skipped outright when no provider has an
    intervening Xformable, so this is also the fast path.
    """
    stage, rigPath = _Stage()
    origins = _Origins(stage, rigPath)
    _Check(_Close(origins["Root"], Gf.Vec3d(0, 2, 0)),
           "Root sits at its authored rest: %s" % (origins["Root"],))
    _Check(_Close(origins["Child"], Gf.Vec3d(3, 2, 0)),
           "Child is parent-local off Root: %s" % (origins["Child"],))
    _Check(_Close(origins["Handle"], Gf.Vec3d(5, 0, 0)),
           "the control is where it was authored: %s" % (origins["Handle"],))


def TestInterveningXformMovesEveryProvider():
    """
    The reported case: one Xform containing the whole rig. Every provider
    under it moves by exactly that transform, joints and controls alike.
    """
    stage, rigPath = _Stage(midTranslate=(10, 0, 0))
    origins = _Origins(stage, rigPath)
    _Check(_Close(origins["Root"], Gf.Vec3d(10, 2, 0)),
           "the chain root follows the Xform: %s" % (origins["Root"],))
    _Check(_Close(origins["Child"], Gf.Vec3d(13, 2, 0)),
           "and so does the joint under it: %s" % (origins["Child"],))
    _Check(_Close(origins["Handle"], Gf.Vec3d(15, 0, 0)),
           "and the control, which has no RigExec ancestor either: %s"
           % (origins["Handle"],))


def TestRotationIsAppliedInTheRightOrder():
    """
    A rotation, not a translation: composing on the wrong side of the
    rest offset is invisible under translation alone and obvious here.

    Root's rest is +2 in Y. Rotating the intervening Xform 90 degrees
    about Z sends +Y to -X, so Root must land at (-2, 0, 0). Getting the
    order backwards leaves it at (0, 2, 0) rotated about its own origin,
    which is still (0, 2, 0).
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset", "Xform")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    UsdGeom.Xformable(stage.DefinePrim("/Asset/Rig/Mid", "Xform")) \
        .AddRotateZOp().Set(90.0)
    stage.DefinePrim("/Asset/Rig/Mid/Joints", "Scope")
    _Joint(stage, "/Asset/Rig/Mid/Joints/Root", ty=2.0)
    _Joint(stage, "/Asset/Rig/Mid/Joints/Root/Child", tx=3.0)

    origins = _Origins(stage, "/Asset/Rig")
    _Check(_Close(origins["Root"], Gf.Vec3d(-2, 0, 0), 1e-9),
           "+Y rest under a 90-degree Z rotation lands on -X: %s"
           % (origins["Root"],))
    # Child is +3 in X off Root, which the same rotation sends to +Y.
    _Check(_Close(origins["Child"], Gf.Vec3d(-2, 3, 0), 1e-9),
           "and the child rotates with it rather than about the origin: %s"
           % (origins["Child"],))


def TestXformBetweenTwoJointsAppliesAtItsOwnLevel():
    """
    The case a uniform right-multiply gets wrong, and the reason the
    implementation walks parents-before-children and divides each frame
    by its anchor's uncorrected value.

    Root at +2Y under Mid(+10X); Inner(+100Z) between Root and Child;
    Child at +3X off Inner. The correct Child is
    Root + Inner + Child = (10+3, 2, 100). A single right-multiply by the
    composed chain would put Inner's +100Z outside Root's contribution
    and still reach (13, 2, 100) under pure translation -- so the test
    also rotates Mid, which makes the two answers differ.
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset", "Xform")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    mid = UsdGeom.Xformable(stage.DefinePrim("/Asset/Rig/Mid", "Xform"))
    mid.AddRotateZOp().Set(90.0)
    stage.DefinePrim("/Asset/Rig/Mid/Joints", "Scope")
    _Joint(stage, "/Asset/Rig/Mid/Joints/Root", ty=2.0)
    UsdGeom.Xformable(
        stage.DefinePrim("/Asset/Rig/Mid/Joints/Root/Inner", "Xform")) \
        .AddTranslateOp().Set(Gf.Vec3d(0, 0, 100))
    _Joint(stage, "/Asset/Rig/Mid/Joints/Root/Inner/Child", tx=3.0)

    origins = _Origins(stage, "/Asset/Rig")
    # Root: rest +2Y, rotated 90 about Z -> (-2, 0, 0).
    _Check(_Close(origins["Root"], Gf.Vec3d(-2, 0, 0), 1e-9),
           "Root: %s" % (origins["Root"],))
    # Child: (+3X off Inner) + (Inner's +100Z) applied in ROOT's space,
    # then the whole thing rotated with Mid. Root's space is already
    # rotated, so Child's +3X becomes +3Y and Inner's +100Z is unturned.
    _Check(_Close(origins["Child"], Gf.Vec3d(-2, 3, 100), 1e-9),
           "the inner Xform applies between the two joints, not after "
           "both: %s" % (origins["Child"],))


def TestNestedRigIsUnaffectedByAnXformAboveTheAssetRoot():
    """
    An Xform ABOVE the asset root is not this evaluator's business: the
    frames are asset-space, and imaging composes the asset root's world
    transform separately. Composing it here as well would double-apply
    it, which is the trap the imaging comment at sceneIndices.cpp:581
    describes.
    """
    stage, rigPath = _Stage()
    before = _Origins(stage, rigPath)
    UsdGeom.Xformable(stage.GetPrimAtPath("/Asset")) \
        .AddTranslateOp().Set(Gf.Vec3d(50, 60, 70))
    after = _Origins(stage, rigPath)
    for name in sorted(before):
        _Check(_Close(before[name], after[name]),
               "%s is asset-space and must not move when the asset does: "
               "%s -> %s" % (name, before[name], after[name]))


def TestTheXformIsReadEveryEvaluation():
    """
    Composed at execution time, not cached from compile and not authored.
    Editing the Xform and re-evaluating the SAME evaluator must follow it
    -- a stale pose that looks plausible is the failure mode this whole
    change risks.
    """
    import _rigexec
    stage, rigPath = _Stage(midTranslate=(10, 0, 0))
    rig = _rigexec.Rig(stage, rigPath)
    rig.compile()

    def rootOrigin():
        pose = rig.evaluate(1.0)
        path = [p for p in pose.joint_paths() if p.endswith("/Root")][0]
        return Gf.Vec3d(*pose.joint_frame(path, True).to_matrix4()[12:15])

    _Check(_Close(rootOrigin(), Gf.Vec3d(10, 2, 0)),
           "first evaluation follows the Xform: %s" % (rootOrigin(),))

    op = UsdGeom.Xformable(
        stage.GetPrimAtPath(rigPath + "/Mid")).GetOrderedXformOps()[0]
    op.Set(Gf.Vec3d(-4, 0, 0))
    _Check(_Close(rootOrigin(), Gf.Vec3d(-4, 2, 0)),
           "and the next one follows the edit, on the same evaluator: %s"
           % (rootOrigin(),))


def TestNothingIsAuthored():
    """
    The requirement this feature was given (user-directed 2026-09-10):
    the Xform is not baked into the USD. Evaluating must leave every
    layer byte-identical.
    """
    stage, rigPath = _Stage(midTranslate=(10, 0, 0))
    layer = stage.GetRootLayer()
    before = layer.ExportToString()
    _Origins(stage, rigPath)
    _Origins(stage, rigPath)
    _Check(layer.ExportToString() == before,
           "evaluation authored something into the layer:\n%s"
           % layer.ExportToString())


def TestTheManipulatorAgreesWithThePose():
    """
    gizmoMath reimplements the rest composition in Python so the
    manipulator can draw a pivot without evaluating the rig, and it
    walked past an intervening Xform for exactly the same reason exec
    did. If the two disagree, the gizmo draws and drags a joint at the
    place it sat before its own Xform applied, while the pose puts it
    somewhere else -- which is worse than the bug they both had.

    Compared against the POSED frame, which on a rig with no solver and
    no avars is the rest frame: the binding publishes no rest accessor.
    """
    import gizmoMath

    for translate, rotate in (((10, 0, 0), None),
                              ((0, 0, 0), 90.0),
                              ((4, -3, 2), 35.0)):
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Asset", "Xform")
        stage.DefinePrim("/Asset/Rig", "RigExecRoot")
        mid = UsdGeom.Xformable(stage.DefinePrim("/Asset/Rig/Mid", "Xform"))
        mid.AddTranslateOp().Set(Gf.Vec3d(*translate))
        if rotate is not None:
            mid.AddRotateZOp().Set(rotate)
        stage.DefinePrim("/Asset/Rig/Mid/Joints", "Scope")
        _Joint(stage, "/Asset/Rig/Mid/Joints/Root", ty=2.0)
        _Joint(stage, "/Asset/Rig/Mid/Joints/Root/Child", tx=3.0)

        posed = _Origins(stage, "/Asset/Rig")
        for name in ("Root", "Child"):
            prim = stage.GetPrimAtPath(
                "/Asset/Rig/Mid/Joints/Root"
                + ("" if name == "Root" else "/Child"))
            rest = gizmoMath.RestSpace(prim, Usd.TimeCode(1.0))
            origin = Gf.Vec3d(*[rest[3][i] for i in range(3)])
            _Check(_Close(origin, posed[name], 1e-9),
                   "%s: the manipulator says %s, the pose says %s "
                   "(translate=%s rotate=%s)"
                   % (name, origin, posed[name], translate, rotate))


def TestTheManipulatorIsUnchangedWithoutOne():
    """The fast path, on the gizmo side: no Xform, no difference."""
    import gizmoMath
    stage, rigPath = _Stage()
    prim = stage.GetPrimAtPath(rigPath + "/Joints/Root")
    rest = gizmoMath.RestSpace(prim, Usd.TimeCode(1.0))
    origin = Gf.Vec3d(*[rest[3][i] for i in range(3)])
    _Check(_Close(origin, Gf.Vec3d(0, 2, 0)),
           "the manipulator still reads the authored rest: %s" % (origin,))


def main():
    _RegisterSchema()
    groups = [
        ("no intervening Xform is unchanged",
         TestNoInterveningXformIsUnchanged),
        ("an intervening Xform moves every provider",
         TestInterveningXformMovesEveryProvider),
        ("rotation applies in the right order",
         TestRotationIsAppliedInTheRightOrder),
        ("an Xform between two joints applies at its own level",
         TestXformBetweenTwoJointsAppliesAtItsOwnLevel),
        ("an Xform above the asset root is not composed twice",
         TestNestedRigIsUnaffectedByAnXformAboveTheAssetRoot),
        ("the Xform is read every evaluation",
         TestTheXformIsReadEveryEvaluation),
        ("nothing is authored", TestNothingIsAuthored),
        ("the manipulator agrees with the pose",
         TestTheManipulatorAgreesWithThePose),
        ("the manipulator is unchanged without one",
         TestTheManipulatorIsUnchangedWithoutOne),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("INTERVENING_XFORM_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
