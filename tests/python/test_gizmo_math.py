#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoMath.py.

Usage: test_gizmo_math.py [<generated schema resources dir>]

The frame replica is checked against the native evaluator through the
_rigexec binding, which must be importable (build-python/python on
PYTHONPATH). A MISSING binding is a test FAILURE, not a skip: under
ctest stdout is hidden on a passing run, so a printed skip line would
let the comparison disappear silently. Set RIGEXEC_GIZMO_ALLOW_NO_NATIVE
to run the pure-python groups without it. A binding that is present but
broken always raises, and is never mistaken for an absent one.
"""
import importlib.util
import math
import os
import random
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Tf, Ts, Usd, UsdGeom  # noqa: E402

import gizmoMath  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _MatClose(a, b, tol=1e-6):
    return all(_Close(a[r][c], b[r][c], tol)
               for r in range(4) for c in range(4))


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered")


def _Rot(axis, deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(axis, deg))
    return m


def _CaptureStderr(fn):
    """
    Run `fn` with file descriptor 2 redirected, and return what it wrote.

    TF_WARN is raised in C++ and printed by the diagnostic manager, so a
    Python-level hook does not see it, and Tf.DiagnosticNotice is not
    bound in this USD build. Redirecting the descriptor catches it
    whatever layer it comes from.
    """
    import tempfile
    handle = tempfile.TemporaryFile(mode="w+b")
    sys.stderr.flush()
    saved = os.dup(2)
    os.dup2(handle.fileno(), 2)
    try:
        fn()
    finally:
        sys.stderr.flush()
        os.dup2(saved, 2)
        os.close(saved)
    handle.seek(0)
    text = handle.read().decode("utf-8", "replace")
    handle.close()
    return text


def _NativeModule():
    """
    The _rigexec binding, or None when its absence is explicitly allowed.

    find_spec answers only "is it on the path", so a binding that IS
    there but fails to load (stale against the current libs, wrong
    python ABI) reaches the plain import below and raises with its real
    error. A bare `except ImportError` around the import would swallow
    that and report it as "not on PYTHONPATH", which is both wrong and,
    because ctest hides stdout on a passing run, invisible.
    """
    if importlib.util.find_spec("_rigexec") is None:
        if os.environ.get("RIGEXEC_GIZMO_ALLOW_NO_NATIVE"):
            print("  (native _rigexec comparison skipped: "
                  "RIGEXEC_GIZMO_ALLOW_NO_NATIVE is set)")
            return None
        raise AssertionError(
            "native _rigexec comparison unavailable: put "
            "build-python/python on PYTHONPATH or set "
            "RIGEXEC_GIZMO_ALLOW_NO_NATIVE=1")
    import _rigexec
    return _rigexec


def TestEulerRoundTrip():
    rng = random.Random(7)
    for order in gizmoMath.ROTATION_ORDERS:
        for _ in range(60):
            angles = [rng.uniform(-170.0, 170.0) for _ in range(3)]
            m = gizmoMath.RotationFromEuler(order, *angles)
            back = gizmoMath.DecomposeEuler(m, order, hint=angles)
            _Check(all(_Close(a, b, 1e-6) for a, b in zip(angles, back)),
                   "%s: %s -> %s" % (order, angles, back))
            m2 = gizmoMath.RotationFromEuler(order, *back)
            _Check(_MatClose(m, m2), "%s matrix round trip" % order)
    # No hint: the matrix still round-trips (angles may differ).
    m = gizmoMath.RotationFromEuler("ZXY", 200.0, 30.0, -100.0)
    back = gizmoMath.DecomposeEuler(m, "ZXY")
    _Check(_MatClose(m, gizmoMath.RotationFromEuler("ZXY", *back)),
           "hintless decomposition reproduces the matrix")
    # Gimbal lock: beta = +-90 keeps the hint's alpha and still matches.
    for beta in (90.0, -90.0):
        m = gizmoMath.RotationFromEuler("XYZ", 25.0, beta, 40.0)
        back = gizmoMath.DecomposeEuler(m, "XYZ", hint=(25.0, beta, 40.0))
        _Check(_MatClose(m, gizmoMath.RotationFromEuler("XYZ", *back), 1e-5),
               "gimbal lock %s -> %s" % (beta, back))
    # The convention matches _ComposeAvars: XYZ is Rx * Ry * Rz (row-vector).
    m = gizmoMath.RotationFromEuler("XYZ", 10.0, 20.0, 30.0)
    ref = _Rot(Gf.Vec3d(1, 0, 0), 10.0) * _Rot(Gf.Vec3d(0, 1, 0), 20.0) \
        * _Rot(Gf.Vec3d(0, 0, 1), 30.0)
    _Check(_MatClose(m, ref), "row-vector order Rx*Ry*Rz")


def TestComposeAvarMatrix():
    m = gizmoMath.ComposeAvarMatrix(1, 2, 3, 2, 2, 2, 0, 0, 90, 0, "XYZ")
    # Gf's Python bindings have no Vec3d * Matrix4d: Transform() is the
    # row-vector point product (v * m with the homogeneous divide).
    p = m.Transform(Gf.Vec3d(1, 0, 0))
    # scale 2 -> (2,0,0); rotate 90 about Z -> (0,2,0); translate -> (1,4,3)
    _Check(all(_Close(p[i], e, 1e-9) for i, e in enumerate((1, 4, 3))),
           "S*R*T order: %s" % p)
    _Check(gizmoMath.NormalizeAvarScale(0.0) == 1e-4, "zero scale floors")
    _Check(gizmoMath.NormalizeAvarScale(-1e-9) == -1e-4, "signed floor")
    _Check(math.copysign(1.0, gizmoMath.NormalizeAvarScale(-0.0)) < 0,
           "-0.0 keeps its sign")
    _Check(gizmoMath.NormalizeAvarScale(float("nan")) == 1.0, "nan -> 1")
    _Check(gizmoMath.NormalizeAvarScale(3.0) == 3.0, "ordinary passes")
    spin = gizmoMath.ComposeAvarMatrix(0, 0, 0, 1, 1, 1, 0, 0, 0, 90, "XYZ")
    q = spin.TransformDir(Gf.Vec3d(0, 1, 0))
    _Check(_Close(q[2], 1.0, 1e-9), "rspin rotates about local +X")


def _ChainStage():
    stage = Usd.Stage.CreateInMemory()
    asset = UsdGeom.Xform.Define(stage, "/Asset")
    asset.AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Controls", "Scope")
    parent = stage.DefinePrim("/Asset/Rig/Controls/Parent", "RigExecControl")
    child = stage.DefinePrim(
        "/Asset/Rig/Controls/Parent/Child", "RigExecControl")
    space = _Rot(Gf.Vec3d(0, 0, 1), 90.0)
    space.SetTranslateOnly(Gf.Vec3d(0, 5, 0))
    parent.GetAttribute("rest:space").Set(space)
    parent.GetAttribute("avars:tx").Set(1.0)
    parent.GetAttribute("avars:rz").Set(30.0)
    parent.GetAttribute("avars:sx").Set(2.0)
    child.GetAttribute("rest:space").Set(space)
    child.GetAttribute("rest:tx").Set(3.0)
    child.GetAttribute("rest:ry").Set(45.0)
    child.GetAttribute("avars:ty").Set(0.5)
    child.GetAttribute("avars:rx").Set(20.0)
    child.GetAttribute("avars:rotationOrder").Set("ZYX")
    return stage, parent, child


def TestRigFramesReplica():
    stage, parent, child = _ChainStage()
    t = Usd.TimeCode.Default()
    pf = gizmoMath.ComputeRigFrames(stage, parent, t)
    cf = gizmoMath.ComputeRigFrames(stage, child, t)
    _Check(pf.reason == "" and cf.reason == "", "chain is editable")
    _Check(pf.rigRoot.GetPath() == Sdf.Path("/Asset/Rig"), "root found")
    # Parent: posed = avars * rest (no ancestor xformable).
    expectedRest = gizmoMath.RestSpace(parent, t)
    _Check(_MatClose(pf.rest, expectedRest), "parent rest")
    _Check(_MatClose(pf.P, expectedRest), "parent P == rest")
    _Check(_MatClose(pf.posed, gizmoMath.AvarsMatrix(parent, t) * pf.rest),
           "parent posed = avars * rest")
    # Child: world = avars * rest * parentRest^-1 * parentPosed.
    expectedP = cf.rest * pf.rest.GetInverse() * pf.posed
    _Check(_MatClose(cf.P, expectedP), "child P")
    _Check(_MatClose(cf.posed, gizmoMath.AvarsMatrix(child, t) * expectedP),
           "child posed")
    _Check(_Close(cf.assetToWorld[3][0], 100.0), "asset placement")
    _Check(pf.rest.GetOrthonormalized(False) == pf.rest
           or _MatClose(pf.rest, pf.rest.GetOrthonormalized(False)),
           "rest is orthonormal")
    # Native comparison. Absent binding fails unless explicitly allowed.
    _rigexec = _NativeModule()
    if _rigexec is None:
        return
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    rig.compile()
    pose = rig.evaluate(-1.0)
    for prim, frames in ((parent, pf), (child, cf)):
        native = pose.control_frame(str(prim.GetPath())).to_matrix4()
        nm = Gf.Matrix4d(*native)
        _Check(_MatClose(nm, frames.posed, 1e-5),
               "%s replica vs native:\n%s\n%s" % (prim.GetName(), nm,
                                                  frames.posed))


def TestVolumeWeightScale():
    """
    A RigExecVolumeWeight must NOT take its scale from avars.

    computations.cpp registers the abstract base with
    readScaleAvars = false, because volume shape is owned by
    inputs:scaleX/Y/Z. The generated schema does not even declare
    avars:sx/sy/sz on the concrete volume weights -- RigExecControl has
    all three, RigExecSphereWeight has none -- so reaching this bug needs
    the scale authored as a custom attribute, which is exactly what a
    gizmo writing avars blindly would do.
    """
    stage, parent, child = _ChainStage()
    t = Usd.TimeCode.Default()
    sphere = stage.DefinePrim("/Asset/Rig/Weights/Blob", "RigExecSphereWeight")
    sphere.GetAttribute("avars:tx").Set(2.0)
    sphere.GetAttribute("avars:rz").Set(25.0)
    sphere.CreateAttribute("avars:sx", Sdf.ValueTypeNames.Double).Set(3.0)

    _Check(gizmoMath.IsRigXformable(sphere), "a volume weight is xformable")
    _Check(not gizmoMath.ReadsScaleAvars(sphere),
           "a volume weight must not read scale avars")
    _Check(gizmoMath.ReadsScaleAvars(parent), "a control reads scale avars")

    sf = gizmoMath.ComputeRigFrames(stage, sphere, t)

    def _AxisLengths(m):
        return [Gf.Vec3d(m[r][0], m[r][1], m[r][2]).GetLength()
                for r in range(3)]

    for r, length in enumerate(_AxisLengths(sf.posed)):
        _Check(_Close(length, 1.0, 1e-9),
               "volume weight axis %d has length %s; avars:sx leaked into "
               "the placement" % (r, length))
    # An xformable parented under a volume weight inherits the rigid frame.
    under = stage.DefinePrim("/Asset/Rig/Weights/Blob/J", "RigExecJoint")
    uf = gizmoMath.ComputeRigFrames(stage, under, t)
    for r, length in enumerate(_AxisLengths(uf.parentPosed)):
        _Check(_Close(length, 1.0, 1e-9),
               "parentPosed axis %d has length %s under a volume weight"
               % (r, length))

    _rigexec = _NativeModule()
    if _rigexec is None:
        return
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    rig.compile()
    pose = rig.evaluate(-1.0)
    # weight_frame returns the 16-number matrix directly, not a PointFrame
    # (python/_rigexec.cpp, the weight_frame binding uses _Mat4ToVec).
    nm = Gf.Matrix4d(*pose.weight_frame(str(sphere.GetPath())))
    _Check(_MatClose(nm, sf.posed, 1e-5),
           "volume weight replica vs native:\n%s\n%s" % (nm, sf.posed))


def TestRigFramesReasons():
    stage, parent, child = _ChainStage()
    t = Usd.TimeCode.Default()
    solver = stage.DefinePrim("/Asset/Rig/Solvers/Fk", "RigExecFkChain")
    joint = stage.DefinePrim("/Asset/Rig/Joints/J", "RigExecJoint")
    solver.GetRelationship("rigExec:joints").SetTargets([joint.GetPath()])
    jf = gizmoMath.ComputeRigFrames(stage, joint, t)
    _Check("solver" in jf.reason, "solver-posed joint refused: %r"
           % jf.reason)
    childJoint = stage.DefinePrim("/Asset/Rig/Joints/J/K", "RigExecJoint")
    kf = gizmoMath.ComputeRigFrames(stage, childJoint, t)
    _Check(kf.reason != "", "child of a solver-posed joint is refused")
    posed = child.GetAttribute("posed:space")
    posed.Set(Gf.Matrix4d(2.0))
    _Check("posed:space" in gizmoMath.ComputeRigFrames(stage, child, t).reason,
           "authored posed:space refused")
    posed.Set(Gf.Matrix4d(1.0))
    _Check(gizmoMath.ComputeRigFrames(stage, child, t).reason == "",
           "identity posed:space is fine")
    posed.AddConnection(parent.GetAttribute("posed:space").GetPath())
    _Check("connected" in gizmoMath.ComputeRigFrames(stage, child, t).reason,
           "connected posed:space refused")
    loose = stage.DefinePrim("/Loose", "RigExecControl")
    _Check("RigExecRoot" in gizmoMath.ComputeRigFrames(stage, loose, t).reason,
           "control outside a rig refused")


def TestWriter():
    stage, parent, child = _ChainStage()
    attr = child.GetAttribute("avars:tx")
    anim = gizmoMath.Writer(stage, Usd.TimeCode(1001.0),
                            gizmoMath.WRITE_ANIMATION)
    anim.Set(attr, 2.0)
    _Check(attr.HasSpline() and len(attr.GetSpline().GetKnots()) == 1,
           "animation mode writes a spline knot")
    _Check(_Close(attr.Get(Usd.TimeCode(1001.0)), 2.0), "knot value")
    # Maya's default new key (graphModel.AuthorKnot), so a gizmo drag and
    # a graph-editor insert produce the same knot.
    knot = attr.GetSpline().GetKnot(1001.0)
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmAutoEase
           and knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmAutoEase,
           "the authored knot has AutoEase tangents on both sides")
    _Check(knot.GetNextInterpolation() == Ts.InterpCurve,
           "and a curve segment after it")
    anim.Set(attr, 3.0)
    _Check(len(attr.GetSpline().GetKnots()) == 1
           and _Close(attr.Get(Usd.TimeCode(1001.0)), 3.0),
           "re-writing the same frame updates the knot")
    _Check(anim.Warnings() == [], "no warnings in animation mode")
    default = gizmoMath.Writer(stage, Usd.TimeCode(1001.0),
                               gizmoMath.WRITE_DEFAULT)
    default.Set(attr, 9.0)
    _Check(stage.GetRootLayer().GetAttributeAtPath(
        attr.GetPath()).default == 9.0, "default mode writes the default")
    _Check(len(default.Warnings()) == 1
           and "avars:tx" in default.Warnings()[0],
           "default outranked by the spline is reported: %s"
           % default.Warnings())
    clean = child.GetAttribute("avars:ty")
    default.Set(clean, 1.0)
    _Check(not clean.HasSpline() and clean.Get() == 1.0, "plain default")
    # A vector attribute (xformOp) gets a time sample, not a spline.
    xf = UsdGeom.Xform.Define(stage, "/Asset/Box")
    op = xf.AddTranslateOp()
    anim.Set(op.GetAttr(), Gf.Vec3d(1, 2, 3))
    _Check(op.GetAttr().GetNumTimeSamples() == 1, "vec3 -> time sample")


def _Drag(target, fn):
    target.BeginDrag()
    fn()


def TestRigPoseTarget():
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None and reason == "", "child is a target")
    _Check(target.kind == "rig-pose" and target.supportsScale, "kind")
    before = gizmoMath.ComputeRigFrames(stage, child, time)
    gizmo = target.GizmoMatrix()
    origin = gizmo.ExtractTranslation()
    expectedOrigin = (before.posed * before.assetToWorld).ExtractTranslation()
    _Check(all(_Close(origin[i], expectedOrigin[i]) for i in range(3)),
           "gizmo sits at the posed world origin")
    # Translate by a world delta: the world origin moves by exactly that.
    delta = Gf.Vec3d(0.7, -1.3, 2.1)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    after = gizmoMath.ComputeRigFrames(stage, child, time)
    moved = (after.posed * after.assetToWorld).ExtractTranslation()
    _Check(all(_Close(moved[i], expectedOrigin[i] + delta[i], 1e-6)
               for i in range(3)), "world translate maps onto avars:t")
    _Check(all(_Close(after.rest[r][c], before.rest[r][c])
               for r in range(4) for c in range(4)),
           "pose mode never touches rest")
    # Rotate about world Z by 30: the world linear part rotates by Rz(30).
    target.Refresh()
    base = gizmoMath.ComputeRigFrames(stage, child, time)
    baseWorld = (base.posed * base.assetToWorld).GetOrthonormalized(False)
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(0, 0, 1), 30.0))
    rotated = gizmoMath.ComputeRigFrames(stage, child, time)
    rotWorld = (rotated.posed * rotated.assetToWorld)\
        .GetOrthonormalized(False)
    expected = baseWorld * _Rot(Gf.Vec3d(0, 0, 1), 30.0)
    expected.SetTranslateOnly(rotWorld.ExtractTranslation())
    _Check(_MatClose(rotWorld, expected, 1e-6),
           "world rotate maps onto avars:r (order ZYX):\n%s\n%s"
           % (rotWorld, expected))
    _Check(_Close(child.GetAttribute("avars:rspin").Get(), 0.0),
           "rspin untouched")
    # Scale along local X by 1.5 multiplies avars:sx only.
    target.Refresh()
    _Drag(target, lambda: target.ApplyScale(0, 1.5))
    _Check(_Close(child.GetAttribute("avars:sx").Get(), 1.5), "sx scaled")
    _Check(_Close(child.GetAttribute("avars:sy").Get(), 1.0), "sy kept")
    target.Refresh()
    _Drag(target, lambda: target.ApplyScale(None, 2.0))
    _Check(_Close(child.GetAttribute("avars:sx").Get(), 3.0)
           and _Close(child.GetAttribute("avars:sz").Get(), 2.0),
           "uniform scale multiplies every axis from the drag base")
    target.Refresh()
    _Drag(target, lambda: target.ApplyScale(1, 0.0))
    _Check(_Close(child.GetAttribute("avars:sy").Get(), 1e-4),
           "scale floor applied on write")
    paths = target.AttributePaths()
    _Check(Sdf.Path("/Asset/Rig/Controls/Parent/Child.avars:tx") in paths
           and Sdf.Path("/Asset/Rig/Controls/Parent/Child.avars:sz") in paths
           and len(paths) == 9, "pose target owns the nine avars")
    # Refusals surface as reasons.
    joint = stage.DefinePrim("/Asset/Rig/Joints/J", "RigExecJoint")
    solver = stage.DefinePrim("/Asset/Rig/Solvers/Fk", "RigExecFkChain")
    solver.GetRelationship("rigExec:joints").SetTargets([joint.GetPath()])
    refused, reason = gizmoMath.MakeTarget(
        stage, joint, gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and "solver" in reason, "solver-posed refused")
    # A volume weight is a RigExecXformable but has its own panel and no
    # scale avars, so the gizmo declines it by concrete type (spec 1.3).
    blob = stage.DefinePrim("/Asset/Rig/Weights/Blob", "RigExecSphereWeight")
    refused, reason = gizmoMath.MakeTarget(
        stage, blob, gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and "volume weight" in reason,
           "volume weight refused: %r" % reason)
    free = stage.DefinePrim("/Asset/Rig/Joints/Free", "RigExecJoint")
    accepted, reason = gizmoMath.MakeTarget(
        stage, free, gizmoMath.CHANNELS_POSE, writer)
    _Check(accepted is not None and accepted.kind == "rig-pose",
           "an unwired joint is a target: %r" % reason)
    child.GetAttribute("avars:rz").AddConnection(
        parent.GetAttribute("avars:rz").GetPath())
    refused, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and "avars:rz" in reason,
           "connected avar refused: %r" % reason)


def TestRigPivotTarget():
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(target is not None and target.kind == "rig-pivot", reason)
    _Check(target.supportsTranslate and target.supportsRotate
           and not target.supportsScale, "pivot: no scale")
    before = gizmoMath.ComputeRigFrames(stage, child, time)
    expectedOrigin = (before.restLocal * before.Q * before.assetToWorld)\
        .ExtractTranslation()
    origin = target.GizmoMatrix().ExtractTranslation()
    _Check(all(_Close(origin[i], expectedOrigin[i]) for i in range(3)),
           "pivot gizmo sits at the rest frame origin")
    avarsBefore = [child.GetAttribute(n).Get() for n in gizmoMath.AVAR_T]
    delta = Gf.Vec3d(1.0, 2.0, -0.5)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    after = gizmoMath.ComputeRigFrames(stage, child, time)
    moved = (after.restLocal * after.Q * after.assetToWorld)\
        .ExtractTranslation()
    _Check(all(_Close(moved[i], expectedOrigin[i] + delta[i], 1e-6)
               for i in range(3)), "pivot translate maps onto rest:t")
    _Check([child.GetAttribute(n).Get() for n in gizmoMath.AVAR_T]
           == avarsBefore, "pivot mode never touches avars")
    target.Refresh()
    base = (after.restLocal * after.Q * after.assetToWorld)\
        .GetOrthonormalized(False)
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(1, 0, 0), -20.0))
    rotated = gizmoMath.ComputeRigFrames(stage, child, time)
    rotWorld = (rotated.restLocal * rotated.Q * rotated.assetToWorld)\
        .GetOrthonormalized(False)
    expected = base * _Rot(Gf.Vec3d(1, 0, 0), -20.0)
    expected.SetTranslateOnly(rotWorld.ExtractTranslation())
    _Check(_MatClose(rotWorld, expected, 1e-6), "pivot rotate onto rest:r")
    _Check(len(target.AttributePaths()) == 6, "six rest channels")


def TestXformTargets():
    stage = Usd.Stage.CreateInMemory()
    world = UsdGeom.Xform.Define(stage, "/World")
    world.AddRotateZOp().Set(90.0)
    box = UsdGeom.Xform.Define(stage, "/World/Box")
    api = UsdGeom.XformCommonAPI(box)
    api.SetTranslate(Gf.Vec3d(1, 0, 0))
    api.SetRotate(Gf.Vec3f(0, 0, 45))
    api.SetPivot(Gf.Vec3f(0, 2, 0))
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, box.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None and target.kind == "xform-pose", reason)
    cache = UsdGeom.XformCache(time)
    expected = cache.GetLocalToWorldTransform(box.GetPrim())\
        .ExtractTranslation()
    # Maya centres all three manipulators on the point the rotate and
    # scale ops turn about, which for this op stack is pivot + translate
    # in parent space (design spec section 8.2), not the local origin.
    startVectors = api.GetXformVectors(time)
    startParent = cache.GetParentToWorldTransform(box.GetPrim())
    expectedGizmo = startParent.Transform(
        Gf.Vec3d(startVectors[3]) + startVectors[0])
    gizmo = target.GizmoMatrix()
    origin = gizmo.ExtractTranslation()
    _Check(all(_Close(origin[i], expectedGizmo[i]) for i in range(3)),
           "xform gizmo at the pivot in world: %s vs %s"
           % (origin, expectedGizmo))
    objectWorld = cache.GetLocalToWorldTransform(box.GetPrim())\
        .GetOrthonormalized(False)
    _Check(all(_Close(gizmo[r][c], objectWorld[r][c])
               for r in range(3) for c in range(3)),
           "the gizmo keeps the object's orientation")
    delta = Gf.Vec3d(0, 3, 0)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    cache.Clear()
    moved = cache.GetLocalToWorldTransform(box.GetPrim()).ExtractTranslation()
    _Check(all(_Close(moved[i], expected[i] + delta[i], 1e-6)
               for i in range(3)), "world translate through a rotated "
           "parent lands on xformOp:translate: %s" % moved)
    t = api.GetXformVectors(time)[0]
    # (0,3,0) * Rz(-90) = (3,0,0) in parent space, added to (1,0,0).
    _Check(_Close(t[0], 4.0, 1e-6) and _Close(t[1], 0.0, 1e-6),
           "parent-space translate value: %s" % t)
    target.Refresh()
    baseWorld = cache.GetLocalToWorldTransform(box.GetPrim())\
        .GetOrthonormalized(False)
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(0, 0, 1), 10.0))
    cache.Clear()
    rotWorld = cache.GetLocalToWorldTransform(box.GetPrim())\
        .GetOrthonormalized(False)
    exp = baseWorld * _Rot(Gf.Vec3d(0, 0, 1), 10.0)
    exp.SetTranslateOnly(rotWorld.ExtractTranslation())
    _Check(_MatClose(rotWorld, exp, 1e-5), "xform rotate")
    _Check(_Close(api.GetXformVectors(time)[1][2], 55.0, 1e-5),
           "rotateXYZ z = 55: %s" % (api.GetXformVectors(time)[1],))
    target.Refresh()
    _Drag(target, lambda: target.ApplyScale(None, 2.0))
    _Check(api.GetXformVectors(time)[2] == Gf.Vec3f(2, 2, 2), "scale")
    # Pivot target moves only the pivot, in parent space.
    pivotTarget, reason = gizmoMath.MakeTarget(
        stage, box.GetPrim(), gizmoMath.CHANNELS_PIVOT, writer)
    _Check(pivotTarget is not None and pivotTarget.kind == "xform-pivot",
           reason)
    _Check(pivotTarget.supportsTranslate and not pivotTarget.supportsRotate
           and not pivotTarget.supportsScale, "xform pivot: translate only")
    vectors = api.GetXformVectors(time)
    parentWorld = cache.GetParentToWorldTransform(box.GetPrim())
    expectedPivot = parentWorld.Transform(
        Gf.Vec3d(vectors[3]) + vectors[0])
    got = pivotTarget.GizmoMatrix().ExtractTranslation()
    _Check(all(_Close(got[i], expectedPivot[i], 1e-6) for i in range(3)),
           "pivot gizmo at (pivot + translate) in parent space")
    _Drag(pivotTarget, lambda: pivotTarget.ApplyTranslate(Gf.Vec3d(0, 1, 0)))
    pivot = api.GetXformVectors(time)[3]
    _Check(_Close(pivot[0], 1.0, 1e-5) and _Close(pivot[1], 2.0, 1e-5),
           "pivot moved by the parent-space delta: %s" % pivot)
    # Incompatible op stacks are refused with a reason.
    odd = UsdGeom.Xform.Define(stage, "/Odd")
    odd.AddTransformOp().Set(Gf.Matrix4d(1.0))
    refused, reason = gizmoMath.MakeTarget(
        stage, odd.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and "XformCommonAPI" in reason, reason)
    scope = stage.DefinePrim("/Scope", "Scope")
    refused, reason = gizmoMath.MakeTarget(
        stage, scope, gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and reason != "", "non-xformable refused")


def TestGimbalAndFrames():
    """
    Maya's Gimbal rotate axes and the frames the handles are drawn in
    (design spec section 8.2 Axis Orientation, 8.3 Rotate Axis).

    The gimbal contract is exact, not approximate: dragging the ring for
    channel j must move channel j by the dragged angle and leave the
    other two alone, whatever the rotation order and the current angles.

    Checked twice over. First on the chain AS IS, whose parent carries
    avars:sx = 2 and so hands the child a SHEARED channel frame: that is
    the case ApplyRotate cannot serve (its contract is that the drawn
    frame turns by the dragged angle, and under shear the rotation doing
    that moves all three channels), so Gimbal mode goes through
    ApplyRotateChannel, which is exact there. Then with the shear
    removed, where GimbalAxes fed back into ApplyRotate must agree with
    it -- that is what makes the drawn ring axes the right ones.
    """
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    # Non-zero angles on all three channels, plus a non-zero avars:rspin:
    # the spin sits BETWEEN the Euler product and P, so a ring axis that
    # ignored it would turn the wrong channels.
    child.GetAttribute("avars:ry").Set(-35.0)
    child.GetAttribute("avars:rz").Set(50.0)
    child.GetAttribute("avars:rspin").Set(15.0)
    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, reason)
    order, angles = target.RotationState()
    _Check(order == "ZYX", "rotation order comes from the avar: %s" % order)
    _Check(all(_Close(a, b) for a, b in zip(angles, (20.0, -35.0, 50.0))),
           "rotation state angles: %s" % (angles,))
    frames = gizmoMath.ComputeRigFrames(stage, child, time)
    origin = target.GizmoMatrix().ExtractTranslation()
    _Check(_MatClose(target.ObjectFrame(), target.GizmoMatrix()),
           "ObjectFrame is the gizmo matrix")
    expected = (frames.P * frames.assetToWorld).GetOrthonormalized(False)
    expected.SetTranslateOnly(origin)
    _Check(_MatClose(target.ChannelFrame(), expected),
           "channel frame is rotation(P) at the gizmo origin:\n%s\n%s"
           % (target.ChannelFrame(), expected))
    expectedGimbal = (_Rot(Gf.Vec3d(1, 0, 0), 15.0) * frames.P
                      * frames.assetToWorld).GetOrthonormalized(False)
    expectedGimbal.SetTranslateOnly(origin)
    _Check(_MatClose(target.GimbalFrame(), expectedGimbal),
           "the gimbal frame carries avars:rspin:\n%s\n%s"
           % (target.GimbalFrame(), expectedGimbal))
    # The channel frame really is sheared here, so this is the case
    # ApplyRotate cannot serve.
    channelLinear = frames.P * frames.assetToWorld
    lengths = [Gf.Vec3d(channelLinear[r][0], channelLinear[r][1],
                        channelLinear[r][2]).GetLength() for r in range(3)]
    _Check(not _Close(lengths[0], lengths[1], 1e-3),
           "the fixture's channel frame must be sheared: %s" % lengths)
    for j in range(3):
        target.Refresh()
        order, base = target.RotationState()
        axes = gizmoMath.GimbalAxes(order, base[0], base[1], base[2],
                                    target.GimbalFrame())
        _Check(_Close(Gf.Vec3d(axes[j]).GetLength(), 1.0),
               "ring axis %d is a unit vector" % j)
        _Drag(target, lambda: target.ApplyRotateChannel(j, 10.0))
        after = target.RotationState()[1]
        for i in range(3):
            want = base[i] + (10.0 if i == j else 0.0)
            _Check(_Close(after[i], want, 1e-9),
                   "sheared: ring %d must move only channel %d: %s -> %s"
                   % (j, j, base, after))
    # With the shear gone, the drawn ring axes and ApplyRotate agree with
    # ApplyRotateChannel -- so the rings are drawn about the right axes.
    parent.GetAttribute("avars:sx").Set(1.0)
    for j in range(3):
        target.Refresh()
        order, base = target.RotationState()
        axes = gizmoMath.GimbalAxes(order, base[0], base[1], base[2],
                                    target.GimbalFrame())
        _Drag(target, lambda: target.ApplyRotate(axes[j], 10.0))
        after = target.RotationState()[1]
        for i in range(3):
            want = base[i] + (10.0 if i == j else 0.0)
            _Check(_Close(after[i], want, 1e-6),
                   "rigid: ring %d must move only channel %d: %s -> %s"
                   % (j, j, base, after))
    # Pivot mode: XYZ rest angles, expressed in Q. Q rides on the
    # parent's avars, so it has to be re-read after the scale change.
    frames = gizmoMath.ComputeRigFrames(stage, child, time)
    pivot, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(pivot is not None, reason)
    pivotOrder, pivotAngles = pivot.RotationState()
    _Check(pivotOrder == "XYZ" and _Close(pivotAngles[1], 45.0),
           "pivot rotation state is the XYZ rest angles: %s" % (pivotAngles,))
    pivotExpected = (frames.Q * frames.assetToWorld)\
        .GetOrthonormalized(False)
    pivotExpected.SetTranslateOnly(pivot.GizmoMatrix().ExtractTranslation())
    _Check(_MatClose(pivot.ChannelFrame(), pivotExpected),
           "the pivot channel frame is Q")
    _Drag(pivot, lambda: pivot.ApplyRotateChannel(2, 12.0))
    _Check(_Close(child.GetAttribute("rest:rz").Get(), 12.0)
           and _Close(child.GetAttribute("rest:ry").Get(), 45.0),
           "the pivot gimbal ring moves one rest:r channel")
    # A plain xform: the channel frame is the parent, and there is no spin.
    spin = UsdGeom.Xform.Define(stage, "/Asset/Spin")
    spin.AddRotateZOp().Set(90.0)
    box = UsdGeom.Xform.Define(stage, "/Asset/Spin/Box")
    UsdGeom.XformCommonAPI(box).SetRotate(Gf.Vec3f(10, 20, 30))
    xTarget, reason = gizmoMath.MakeTarget(
        stage, box.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(xTarget is not None, reason)
    xOrder, xAngles = xTarget.RotationState()
    _Check(xOrder == "XYZ" and _Close(xAngles[2], 30.0, 1e-4),
           "xform rotation state: %s %s" % (xOrder, xAngles))
    xExpected = UsdGeom.XformCache(time)\
        .GetParentToWorldTransform(box.GetPrim()).GetOrthonormalized(False)
    xExpected.SetTranslateOnly(xTarget.GizmoMatrix().ExtractTranslation())
    _Check(_MatClose(xTarget.ChannelFrame(), xExpected),
           "the xform channel frame is the parent frame")
    _Check(_MatClose(xTarget.GimbalFrame(), xTarget.ChannelFrame()),
           "a plain xform has no spin between its channels and its parent")
    for j in range(3):
        xTarget.Refresh()
        xOrder, base = xTarget.RotationState()
        axes = gizmoMath.GimbalAxes(xOrder, base[0], base[1], base[2],
                                    xTarget.GimbalFrame())
        _Drag(xTarget, lambda: xTarget.ApplyRotate(axes[j], -7.0))
        after = xTarget.RotationState()[1]
        for i in range(3):
            want = base[i] + (-7.0 if i == j else 0.0)
            _Check(_Close(after[i], want, 1e-4),
                   "xform ring %d must move only channel %d: %s -> %s"
                   % (j, j, base, after))
    xTarget.Refresh()
    xOrder, base = xTarget.RotationState()
    _Drag(xTarget, lambda: xTarget.ApplyRotateChannel(1, 5.0))
    after = xTarget.RotationState()[1]
    _Check(_Close(after[1], base[1] + 5.0, 1e-4)
           and _Close(after[0], base[0], 1e-4)
           and _Close(after[2], base[2], 1e-4),
           "the xform gimbal ring moves one rotate component: %s -> %s"
           % (base, after))
    xPivot, reason = gizmoMath.MakeTarget(
        stage, box.GetPrim(), gizmoMath.CHANNELS_PIVOT, writer)
    _Check(xPivot is not None and xPivot.RotationState() is None,
           "the xform pivot has no rotation channels")
    # The xform pivot has no rotation channels, so Gimbal mode is a
    # no-op on it rather than an error.
    _Drag(xPivot, lambda: xPivot.ApplyRotateChannel(0, 30.0))
    _Check(_Close(xTarget.RotationState()[1][0], after[0], 1e-4),
           "the xform pivot ignores a gimbal ring")


def TestPreserveChildren():
    """
    Maya's Preserve Children (design spec section 8.2), default off: the
    children keep their world transforms while the parent is dragged.
    """
    stage = Usd.Stage.CreateInMemory()
    parent = UsdGeom.Xform.Define(stage, "/P")
    UsdGeom.XformCommonAPI(parent).SetTranslate(Gf.Vec3d(1, 0, 0))
    child = UsdGeom.Xform.Define(stage, "/P/C")
    childApi = UsdGeom.XformCommonAPI(child)
    childApi.SetTranslate(Gf.Vec3d(0, 2, 0))
    childApi.SetRotate(Gf.Vec3f(0, 0, 30))
    # A child with a non-zero pivot is out of scope: it rides along.
    offset = UsdGeom.Xform.Define(stage, "/P/Offset")
    offsetApi = UsdGeom.XformCommonAPI(offset)
    offsetApi.SetTranslate(Gf.Vec3d(0, 0, 1))
    offsetApi.SetPivot(Gf.Vec3f(0, 1, 0))
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, parent.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None and target.supportsPreserveChildren,
           "a plain xform supports Preserve Children: %s" % reason)
    _Check(not target.preserveChildren, "Preserve Children defaults to off")
    cache = UsdGeom.XformCache(time)
    before = cache.GetLocalToWorldTransform(child.GetPrim())
    offsetBefore = cache.GetLocalToWorldTransform(offset.GetPrim())
    target.SetPreserveChildren(True)
    paths = target.AttributePaths()
    _Check(Sdf.Path("/P/C.xformOp:translate") in paths
           and Sdf.Path("/P/C.xformOp:rotateXYZ") in paths,
           "a preserved child's channels join the undo set: %s" % paths)
    _Check(Sdf.Path("/P/Offset.xformOp:translate") not in paths,
           "a pivoted child is not preserved")
    for label, apply in (
            ("translate", lambda: target.ApplyTranslate(Gf.Vec3d(3, -1, 2))),
            ("rotate", lambda: target.ApplyRotate(Gf.Vec3d(0, 0, 1), 25.0)),
            ("scale", lambda: target.ApplyScale(None, 2.0))):
        target.Refresh()
        _Drag(target, apply)
        cache.Clear()
        now = cache.GetLocalToWorldTransform(child.GetPrim())
        _Check(_MatClose(now, before, 1e-6),
               "the child holds still through a %s:\n%s\n%s"
               % (label, now, before))
    cache.Clear()
    _Check(not _MatClose(cache.GetLocalToWorldTransform(offset.GetPrim()),
                         offsetBefore, 1e-6),
           "the pivoted child rides along, as documented")
    # Off: the child rides along with the parent.
    target.SetPreserveChildren(False)
    target.Refresh()
    _Drag(target, lambda: target.ApplyTranslate(Gf.Vec3d(0, 5, 0)))
    cache.Clear()
    _Check(not _MatClose(cache.GetLocalToWorldTransform(child.GetPrim()),
                         before, 1e-6),
           "with Preserve Children off the child moves")
    _Check(Sdf.Path("/P/C.xformOp:translate") not in target.AttributePaths(),
           "off: no child channels in the undo set")
    # Rig prims and the xform pivot refuse, with a reason.
    rigStage, rigParent, rigChild = _ChainStage()
    rigWriter = gizmoMath.Writer(rigStage, time, gizmoMath.WRITE_DEFAULT)
    rigTarget, reason = gizmoMath.MakeTarget(
        rigStage, rigChild, gizmoMath.CHANNELS_POSE, rigWriter)
    _Check(rigTarget is not None, reason)
    _Check(not rigTarget.supportsPreserveChildren
           and "rig" in rigTarget.preserveChildrenReason,
           "a rig target refuses: %r" % rigTarget.preserveChildrenReason)
    rigTarget.SetPreserveChildren(True)
    _Check(not rigTarget.preserveChildren,
           "the refusal cannot be overridden")
    pivotTarget, reason = gizmoMath.MakeTarget(
        stage, parent.GetPrim(), gizmoMath.CHANNELS_PIVOT, writer)
    _Check(pivotTarget is not None, reason)
    _Check(not pivotTarget.supportsPreserveChildren
           and pivotTarget.preserveChildrenReason != "",
           "the xform pivot refuses Preserve Children")
    # A plain Xform GROUP inside a rig. RigExecXformable inherits
    # Boundable, so a RigExecControl IS a UsdGeom.Xformable with an
    # empty, hence XformCommonAPI-compatible, op stack and a zero pivot:
    # without an explicit guard the compensation would author xformOps
    # on it, which the evaluator never reads and the project forbids.
    # Nothing under a RigExecRoot is compensated, and every skip is
    # reported so the toolbar can say why the children did not follow.
    grp = UsdGeom.Xform.Define(rigStage, "/Asset/Rig/Grp")
    UsdGeom.XformCommonAPI(grp).SetTranslate(Gf.Vec3d(2, 0, 0))
    ctl = rigStage.DefinePrim("/Asset/Rig/Grp/Ctl", "RigExecControl")
    ctl.GetAttribute("avars:tx").Set(1.0)
    plain = UsdGeom.Xform.Define(rigStage, "/Asset/Rig/Grp/Plain")
    UsdGeom.XformCommonAPI(plain).SetTranslate(Gf.Vec3d(0, 4, 0))
    _Check(ctl.IsA(UsdGeom.Xformable)
           and bool(UsdGeom.XformCommonAPI(ctl)),
           "a RigExecControl looks like a compensable xformable")
    grpTarget, reason = gizmoMath.MakeTarget(
        rigStage, grp.GetPrim(), gizmoMath.CHANNELS_POSE, rigWriter)
    _Check(grpTarget is not None and grpTarget.supportsPreserveChildren,
           "an Xform group inside a rig is still an xform target: %s"
           % reason)
    grpTarget.SetPreserveChildren(True)
    paths = grpTarget.AttributePaths()
    _Check(not any(str(p).startswith("/Asset/Rig/Grp/") for p in paths),
           "no child under a rig joins the undo set: %s" % paths)
    _Check(sorted(grpTarget.skippedChildren) == sorted([
        "Ctl is placed by the rig, not by xformOps",
        "Plain is placed by the rig, not by xformOps"]),
           "both children are reported skipped: %s"
           % grpTarget.skippedChildren)
    _Drag(grpTarget, lambda: grpTarget.ApplyTranslate(Gf.Vec3d(0, 0, 3)))
    # GetPropertyNames lists the schema's own xformOpOrder, so only
    # AUTHORED names answer "did the compensation write here".
    _Check(not [n for n in ctl.GetAuthoredPropertyNames()
                if n.startswith("xformOp")],
           "no xformOp authored on the RigExec child: %s"
           % ctl.GetAuthoredPropertyNames())
    _Check(not [n for n in plain.GetPrim().GetAuthoredPropertyNames()
                if n.startswith("xformOp:rotate")
                or n.startswith("xformOp:scale")],
           "no new op authored on the plain child under the rig either: %s"
           % plain.GetPrim().GetAuthoredPropertyNames())


def _AxisAlignedRigStage():
    """
    A rig with one control and no rest spaces, so P and Q are the
    identity and a world delta IS the channel delta. The snap numbers
    below are then readable instead of being wrapped in a rotation.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Asset")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    return stage, stage.DefinePrim("/Asset/Rig/Ctl", "RigExecControl")


def TestSnapAndPlanarScale():
    """
    Maya's planar (two-axis) scale handles and Step Snap (design spec
    8.2, 8.4). Snapping is done here, not in the controller, because it
    has to happen in CHANNEL space: quantising the world delta would put
    the written values off the grid whenever the channel frame is turned.
    """
    stage, ctl = _AxisAlignedRigStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, ctl, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, reason)
    channel = target.ChannelFrame()
    _Check(all(_Close(channel[r][c], 1.0 if r == c else 0.0)
               for r in range(3) for c in range(3)),
           "the fixture's channel frame is axis-aligned:\n%s" % channel)
    # No snapStep: the delta lands untouched.
    _Drag(target, lambda: target.ApplyTranslate(Gf.Vec3d(0.7, 0.2, 0.0)))
    _Check(_Close(ctl.GetAttribute("avars:tx").Get(), 0.7)
           and _Close(ctl.GetAttribute("avars:ty").Get(), 0.2),
           "no snapStep leaves the delta alone")
    # Relative snap (Maya's J hold) quantises the DELTA.
    for name in gizmoMath.AVAR_T:
        ctl.GetAttribute(name).Set(0.0)
    _Drag(target, lambda: target.ApplyTranslate(
        Gf.Vec3d(0.7, 0.2, 0.0), snapStep=0.5))
    _Check(_Close(ctl.GetAttribute("avars:tx").Get(), 0.5)
           and _Close(ctl.GetAttribute("avars:ty").Get(), 0.0),
           "relative step snap quantises the channel delta: %s"
           % [ctl.GetAttribute(n).Get() for n in gizmoMath.AVAR_T])
    # Absolute snap (Maya's X grid hold) quantises the RESULT, which is
    # what makes it different: the same delta is swallowed by relative.
    ctl.GetAttribute("avars:tx").Set(1.3)
    _Drag(target, lambda: target.ApplyTranslate(
        Gf.Vec3d(0.2, 0.0, 0.0), snapStep=0.5, snapAbsolute=True))
    _Check(_Close(ctl.GetAttribute("avars:tx").Get(), 1.5),
           "absolute step snap puts the value on the grid: %s"
           % ctl.GetAttribute("avars:tx").Get())
    ctl.GetAttribute("avars:tx").Set(1.3)
    _Drag(target, lambda: target.ApplyTranslate(
        Gf.Vec3d(0.2, 0.0, 0.0), snapStep=0.5))
    _Check(_Close(ctl.GetAttribute("avars:tx").Get(), 1.3),
           "relative snap swallows a sub-step delta: %s"
           % ctl.GetAttribute("avars:tx").Get())
    # Scale: snap, then the planar and uniform axis selections.
    _Drag(target, lambda: target.ApplyScale(0, 1.37, snapStep=0.5))
    _Check(_Close(ctl.GetAttribute("avars:sx").Get(), 1.5)
           and _Close(ctl.GetAttribute("avars:sy").Get(), 1.0),
           "scale step snap: %s"
           % [ctl.GetAttribute(n).Get() for n in gizmoMath.AVAR_S])
    for name in gizmoMath.AVAR_S:
        ctl.GetAttribute(name).Set(1.0)
    _Drag(target, lambda: target.ApplyScale((0, 1), 3.0))
    _Check(_Close(ctl.GetAttribute("avars:sx").Get(), 3.0)
           and _Close(ctl.GetAttribute("avars:sy").Get(), 3.0)
           and _Close(ctl.GetAttribute("avars:sz").Get(), 1.0),
           "the XY planar handle scales exactly two channels: %s"
           % [ctl.GetAttribute(n).Get() for n in gizmoMath.AVAR_S])
    _Drag(target, lambda: target.ApplyScale([1, 2], 0.5))
    _Check(_Close(ctl.GetAttribute("avars:sx").Get(), 3.0)
           and _Close(ctl.GetAttribute("avars:sy").Get(), 1.5)
           and _Close(ctl.GetAttribute("avars:sz").Get(), 0.5),
           "a list of axes works too: %s"
           % [ctl.GetAttribute(n).Get() for n in gizmoMath.AVAR_S])
    # Any iterable of ints, not a list of blessed classes: the
    # controller hands over whatever its handle description carries.
    for name in gizmoMath.AVAR_S:
        ctl.GetAttribute(name).Set(1.0)
    _Drag(target, lambda: target.ApplyScale((i for i in (0, 2)), 5.0))
    _Check(_Close(ctl.GetAttribute("avars:sx").Get(), 5.0)
           and _Close(ctl.GetAttribute("avars:sy").Get(), 1.0)
           and _Close(ctl.GetAttribute("avars:sz").Get(), 5.0),
           "a generator of axes works: %s"
           % [ctl.GetAttribute(n).Get() for n in gizmoMath.AVAR_S])

    class _PlaneAxes(object):
        """A sequence-like handle description, as the controller has."""

        def __init__(self, *axes):
            self._axes = axes

        def __len__(self):
            return len(self._axes)

        def __getitem__(self, index):
            return self._axes[index]

    for name in gizmoMath.AVAR_S:
        ctl.GetAttribute(name).Set(1.0)
    _Drag(target, lambda: target.ApplyScale(_PlaneAxes(1, 2), 7.0))
    _Check(_Close(ctl.GetAttribute("avars:sx").Get(), 1.0)
           and _Close(ctl.GetAttribute("avars:sy").Get(), 7.0)
           and _Close(ctl.GetAttribute("avars:sz").Get(), 7.0),
           "a sequence-like object works: %s"
           % [ctl.GetAttribute(n).Get() for n in gizmoMath.AVAR_S])
    _Check(gizmoMath._ScaleAxes(2) == (2,)
           and gizmoMath._ScaleAxes(None) == (0, 1, 2),
           "an int is still one axis and None is still all three")
    # The snap knobs are keyword-only: positionally, the third argument
    # to ApplyTranslate would be snapAbsolute, and a mix-up would turn
    # grid snapping on without anyone asking.
    target.BeginDrag()
    raised = False
    try:
        target.ApplyTranslate(Gf.Vec3d(0.7, 0, 0), 0.5)
    except TypeError:
        raised = True
    _Check(raised, "a positional snapStep must be refused")
    for method, args in ((target.ApplyRotate, (Gf.Vec3d(0, 0, 1), 37.0)),
                         (target.ApplyRotateChannel, (0, 37.0)),
                         (target.ApplyScale, (0, 1.37))):
        raised = False
        try:
            method(*(args + (15.0,)))
        except TypeError:
            raised = True
        _Check(raised, "%s must refuse a positional snapStep"
               % method.__name__)
    _Drag(target, lambda: target.ApplyScale((1,), 0.0, snapStep=0.5))
    _Check(_Close(ctl.GetAttribute("avars:sy").Get(), 1e-4),
           "the 1e-4 floor still runs after the snap: %s"
           % ctl.GetAttribute("avars:sy").Get())
    # Rotate: the ANGLE is snapped, relative to the drag base.
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(0, 0, 1), 37.0,
                                             snapStep=15.0))
    _Check(_Close(ctl.GetAttribute("avars:rz").Get(), 30.0, 1e-6),
           "rotate step snap: %s" % ctl.GetAttribute("avars:rz").Get())
    _Drag(target, lambda: target.ApplyRotateChannel(0, 37.0, snapStep=15.0))
    _Check(_Close(ctl.GetAttribute("avars:rx").Get(), 30.0)
           and _Close(ctl.GetAttribute("avars:rz").Get(), 30.0, 1e-6),
           "gimbal ring step snap: %s"
           % [ctl.GetAttribute(n).Get() for n in gizmoMath.AVAR_R])
    # Pivot mode snaps its own channels.
    pivotTarget, reason = gizmoMath.MakeTarget(
        stage, ctl, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(pivotTarget is not None, reason)
    _Drag(pivotTarget, lambda: pivotTarget.ApplyTranslate(
        Gf.Vec3d(0.7, 0.0, 0.0), snapStep=0.5))
    _Check(_Close(ctl.GetAttribute("rest:tx").Get(), 0.5),
           "the rig pivot honours step snap: %s"
           % ctl.GetAttribute("rest:tx").Get())
    _Drag(pivotTarget, lambda: pivotTarget.ApplyRotateChannel(
        1, 37.0, snapStep=15.0))
    _Check(_Close(ctl.GetAttribute("rest:ry").Get(), 30.0),
           "the rig pivot honours a snapped gimbal ring")
    # The same two knobs on the plain xform targets.
    box = UsdGeom.Xform.Define(stage, "/Asset/Box")
    api = UsdGeom.XformCommonAPI(box)
    xTarget, reason = gizmoMath.MakeTarget(
        stage, box.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(xTarget is not None, reason)
    _Drag(xTarget, lambda: xTarget.ApplyScale((0, 2), 4.0))
    _Check(api.GetXformVectors(time)[2] == Gf.Vec3f(4, 1, 4),
           "xform planar scale: %s" % (api.GetXformVectors(time)[2],))
    _Drag(xTarget, lambda: xTarget.ApplyScale(None, 1.1, snapStep=1.0))
    _Check(api.GetXformVectors(time)[2] == Gf.Vec3f(4, 1, 4),
           "a snapped uniform scale lands back on the grid: %s"
           % (api.GetXformVectors(time)[2],))
    _Drag(xTarget, lambda: xTarget.ApplyTranslate(
        Gf.Vec3d(0.7, 0.2, 0.0), snapStep=0.5))
    t = api.GetXformVectors(time)[0]
    _Check(_Close(t[0], 0.5, 1e-6) and _Close(t[1], 0.0, 1e-6),
           "xform translate step snap: %s" % (t,))
    _Drag(xTarget, lambda: xTarget.ApplyRotate(Gf.Vec3d(0, 0, 1), 37.0,
                                               snapStep=15.0))
    _Check(_Close(api.GetXformVectors(time)[1][2], 30.0, 1e-4),
           "xform rotate step snap: %s" % (api.GetXformVectors(time)[1],))
    xPivot, reason = gizmoMath.MakeTarget(
        stage, box.GetPrim(), gizmoMath.CHANNELS_PIVOT, writer)
    _Check(xPivot is not None, reason)
    _Drag(xPivot, lambda: xPivot.ApplyTranslate(
        Gf.Vec3d(0.0, 0.7, 0.0), snapStep=0.5))
    _Check(_Close(api.GetXformVectors(time)[3][1], 0.5, 1e-6),
           "xform pivot step snap: %s" % (api.GetXformVectors(time)[3],))


def TestXformOpNoise():
    """
    A drag on a plain Xform with no authored ops must author only the op
    it writes, and must not make USD complain.

    Creating all four common ops for every drag put three ops the drag
    never writes into xformOpOrder -- scene description the user did not
    ask for, including a zero pivot pair on a prim that had none, and
    three more specs for an undo to remove. Each caller now creates only
    the op it is about to write.

    The silence check is a separate guarantee, not the reason for the
    narrowing: the "Unable to get attribute associated with the xformOp"
    burst an end-to-end run reported was traced to the undo stack
    restoring op attributes and xformOpOrder in separate change blocks,
    which is rigExecUndo's to fix. This asserts that a drag through this
    module composes cleanly on its own, so the two cannot be confused
    again.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Shot")
    box = UsdGeom.Xform.Define(stage, "/Shot/HeroArm")
    prim = box.GetPrim()
    time = Usd.TimeCode.Default()
    # usdview edits the session layer; keep the prim and its ops in
    # different layers, as they are in the app.
    stage.SetEditTarget(stage.GetSessionLayer())
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, prim, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, reason)

    def _TranslateDrag():
        target.BeginDrag()
        # Recompose between events, the way the viewport does: that is
        # what turns an inconsistent op stack into a printed warning.
        for step in range(4):
            target.ApplyTranslate(Gf.Vec3d(step + 1.0, 0, 0))
            UsdGeom.XformCache(time).GetLocalToWorldTransform(prim)
            target.Refresh()
            target.GizmoMatrix()

    noise = _CaptureStderr(_TranslateDrag)
    _Check("Unable to get attribute" not in noise,
           "a drag on a bare Xform must not warn:\n%s" % noise.strip())
    _Check(noise.strip() == "", "a drag on a bare Xform is silent:\n%s"
           % noise.strip())
    _Check(sorted(prim.GetAuthoredPropertyNames())
           == ["xformOp:translate", "xformOpOrder"],
           "a translate drag authors only the translate op: %s"
           % sorted(prim.GetAuthoredPropertyNames()))
    _Check(list(prim.GetAttribute("xformOpOrder").Get())
           == ["xformOp:translate"],
           "xformOpOrder names only the op that was written: %s"
           % (prim.GetAttribute("xformOpOrder").Get(),))
    # Each tool adds its own op, and only its own.
    _Drag(target, lambda: target.ApplyScale(None, 2.0))
    _Check("xformOp:scale" in prim.GetAuthoredPropertyNames()
           and "xformOp:rotateXYZ" not in prim.GetAuthoredPropertyNames()
           and "xformOp:translate:pivot"
           not in prim.GetAuthoredPropertyNames(),
           "a scale drag adds the scale op alone: %s"
           % sorted(prim.GetAuthoredPropertyNames()))
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(0, 0, 1), 20.0))
    _Check("xformOp:rotateXYZ" in prim.GetAuthoredPropertyNames()
           and "xformOp:translate:pivot"
           not in prim.GetAuthoredPropertyNames(),
           "a rotate drag adds the rotate op alone: %s"
           % sorted(prim.GetAuthoredPropertyNames()))
    # The pivot op appears only when pivot mode writes it.
    pivotTarget, reason = gizmoMath.MakeTarget(
        stage, prim, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(pivotTarget is not None, reason)
    _Drag(pivotTarget, lambda: pivotTarget.ApplyTranslate(Gf.Vec3d(0, 1, 0)))
    _Check("xformOp:translate:pivot" in prim.GetAuthoredPropertyNames(),
           "pivot mode still creates the pivot op: %s"
           % sorted(prim.GetAuthoredPropertyNames()))
    # The values all survived the narrower op creation.
    vectors = UsdGeom.XformCommonAPI(prim).GetXformVectors(time)
    _Check(_Close(vectors[0][0], 4.0) and vectors[2] == Gf.Vec3f(2, 2, 2)
           and _Close(vectors[1][2], 20.0, 1e-4)
           and _Close(vectors[3][1], 1.0, 1e-6),
           "every channel still holds what its drag wrote: %s" % (vectors,))


class _NoticeCounter(object):
    """Counts Usd.Notice.ObjectsChanged rounds on one stage."""

    def __init__(self, stage):
        self.count = 0
        # The key must outlive the listener: dropping it revokes it.
        self._key = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._OnChanged, stage)

    def _OnChanged(self, notice, sender):
        self.count += 1

    def Revoke(self):
        self._key.Revoke()


def TestXformLocalMatrix():
    """
    _XformCommonMatrix must reproduce GetLocalTransformation exactly.

    It is what a Preserve Children drag compensates against, composed
    instead of read back because the write that produces it is inside
    the same Sdf.ChangeBlock. If the composition rule is wrong the
    children drift and nothing else notices, so it is pinned against USD
    itself for every rotation order and for a non-zero pivot.
    """
    stage = Usd.Stage.CreateInMemory()
    time = Usd.TimeCode.Default()
    t = Gf.Vec3d(1.0, -2.0, 3.5)
    p = Gf.Vec3f(0.5, -1.0, 2.0)
    r = Gf.Vec3f(10.0, 20.0, 30.0)
    s = Gf.Vec3f(2.0, 3.0, 4.0)
    for name in gizmoMath.ROTATION_ORDERS:
        order = getattr(UsdGeom.XformCommonAPI, "RotationOrder" + name)
        xform = UsdGeom.Xform.Define(stage, "/X" + name)
        api = UsdGeom.XformCommonAPI(xform.GetPrim())
        api.SetTranslate(t)
        api.SetPivot(p)
        api.SetRotate(r, order)
        api.SetScale(s)
        _Check(_MatClose(gizmoMath._XformCommonMatrix(t, p, r, s, name),
                         xform.GetLocalTransformation(time), 1e-6),
               "%s: composed local matrix must equal USD's" % name)
    # A zero pivot is the common case and must collapse to plain TRS.
    zero = UsdGeom.Xform.Define(stage, "/Z")
    zeroApi = UsdGeom.XformCommonAPI(zero.GetPrim())
    zeroApi.SetTranslate(t)
    zeroApi.SetRotate(r)
    zeroApi.SetScale(s)
    _Check(_MatClose(
        gizmoMath._XformCommonMatrix(t, Gf.Vec3f(0, 0, 0), r, s, "XYZ"),
        zero.GetLocalTransformation(time), 1e-6), "zero pivot")


def TestOneNoticePerApply():
    """
    One Apply* must reach the stage as ONE ObjectsChanged once the ops
    it writes exist (design spec 3.3 and 4.2).

    The RigExec evaluator republishes synchronously on the notice, so an
    unbatched Preserve Children drag cost 1 + 3N recompositions per
    mouse-move for N children, all discarded but the last -- invisible
    in a test that only checks the values, and very visible in a scene.

    The exception is an event that has to CREATE an op: that authoring
    cannot happen inside the block (see TestReferencedPrimDrag), so it
    costs one extra notice. The last group here pins that to once per
    drag rather than once per event, which is the whole trade.
    """
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, reason)
    target.BeginDrag()
    counter = _NoticeCounter(stage)
    target.ApplyTranslate(Gf.Vec3d(0.25, -0.5, 0.75))
    _Check(counter.count == 1,
           "a rig ApplyTranslate writes three avars in one notice, got %d"
           % counter.count)
    counter.Revoke()

    # An xform pose target with Preserve Children on and two children:
    # the op write plus 3 channels on each child, still one notice.
    # Every prim here carries a full T/R/S stack already, so nothing in
    # this group has an op to create and the count is the steady-state
    # cost of a mouse-move mid-drag.
    xstage = Usd.Stage.CreateInMemory()
    group = UsdGeom.Xform.Define(xstage, "/P")
    groupApi = UsdGeom.XformCommonAPI(group)
    groupApi.SetTranslate(Gf.Vec3d(1, 0, 0))
    groupApi.SetRotate(Gf.Vec3f(0, 0, 0))
    groupApi.SetScale(Gf.Vec3f(1, 1, 1))
    kids = []
    for i, name in enumerate(("A", "B")):
        kid = UsdGeom.Xform.Define(xstage, "/P/" + name)
        kidApi = UsdGeom.XformCommonAPI(kid)
        kidApi.SetTranslate(Gf.Vec3d(0, 2 + i, 0))
        kidApi.SetRotate(Gf.Vec3f(0, 0, 30 * (i + 1)))
        kidApi.SetScale(Gf.Vec3f(1, 1, 1))
        kids.append(kid)
    cache = UsdGeom.XformCache(time)
    before = [cache.GetLocalToWorldTransform(k.GetPrim()) for k in kids]
    xwriter = gizmoMath.Writer(xstage, time, gizmoMath.WRITE_DEFAULT)
    xtarget, reason = gizmoMath.MakeTarget(
        xstage, group.GetPrim(), gizmoMath.CHANNELS_POSE, xwriter)
    _Check(xtarget is not None, reason)
    xtarget.SetPreserveChildren(True)
    _Check(xtarget.preserveChildren, "Preserve Children is on")
    xtarget.AttributePaths()
    xtarget.BeginDrag()
    counter = _NoticeCounter(xstage)
    xtarget.ApplyTranslate(Gf.Vec3d(0, 5, 0))
    _Check(counter.count == 1,
           "an xform ApplyTranslate compensating two children must fire "
           "exactly one ObjectsChanged, got %d" % counter.count)
    counter.Revoke()
    # ... and the batching must not have cost the compensation itself:
    # the children have to be exactly where they were.
    cache.Clear()
    for kid, was in zip(kids, before):
        _Check(_MatClose(cache.GetLocalToWorldTransform(kid.GetPrim()),
                         was, 1e-5),
               "%s held its world transform through the batched write"
               % kid.GetPath())
    _Check(_Close(UsdGeom.XformCommonAPI(group).GetXformVectors(time)[0][1],
                  5.0, 1e-6), "and the parent actually moved")
    # A second move is one notice too, now that the ops already exist.
    xtarget.BeginDrag()
    counter = _NoticeCounter(xstage)
    xtarget.ApplyRotate(Gf.Vec3d(0, 0, 1), 15.0)
    _Check(counter.count == 1,
           "a second Apply* is one notice as well, got %d" % counter.count)
    counter.Revoke()

    # A prim with NO ops pays for creating them, and pays ONCE.
    #
    # Op creation cannot go in the block (TestReferencedPrimDrag), and
    # it is not cheap: measured on this USD build, CreateXformOps for a
    # single missing op costs three change rounds -- the op attribute,
    # then xformOpOrder, then xformOpOrder's value -- and nothing at all
    # when the ops already exist. So the first event of a drag on a bare
    # prim is 3 + 1 and every event after it is 1. What matters is that
    # the cost is once per drag, not once per mouse-move; the assertion
    # is written to say exactly that rather than to pin a magic number.
    bare = UsdGeom.Xform.Define(xstage, "/Bare")
    bareTarget, reason = gizmoMath.MakeTarget(
        xstage, bare.GetPrim(), gizmoMath.CHANNELS_POSE, xwriter)
    _Check(bareTarget is not None, reason)
    bareTarget.BeginDrag()
    counter = _NoticeCounter(xstage)
    bareTarget.ApplyTranslate(Gf.Vec3d(1, 0, 0))
    first = counter.count
    counter.count = 0
    bareTarget.ApplyTranslate(Gf.Vec3d(2, 0, 0))
    second = counter.count
    counter.count = 0
    bareTarget.ApplyTranslate(Gf.Vec3d(3, 0, 0))
    _Check(second == 1 and counter.count == 1,
           "once the op exists every further event is one notice: "
           "first=%d, second=%d, third=%d" % (first, second, counter.count))
    _Check(first > second,
           "and the creation cost fell on the first event: %d" % first)
    counter.Revoke()
    _Check(_Close(UsdGeom.XformCommonAPI(bare).GetXformVectors(time)[0][0],
                  3.0, 1e-6), "and every event landed")


def TestResetXformStack():
    """
    A target whose op stack starts with !resetXformStack! sits in WORLD
    space, not its parent's.

    XformCommonAPI accepts such a stack, so the gizmo can be handed one.
    GetParentToWorldTransform is only the parent's CTM and does not
    consult the flag, so taking it at face value would draw the
    manipulator in the wrong place and -- worse, because it is silent --
    compensate Preserve Children against a frame the prim is not in.
    """
    stage = Usd.Stage.CreateInMemory()
    time = Usd.TimeCode.Default()
    root = UsdGeom.Xform.Define(stage, "/Root")
    root.AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    prim = UsdGeom.Xform.Define(stage, "/Root/T")
    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(1, 2, 3))
    api.SetRotate(Gf.Vec3f(0, 0, 30))
    api.SetScale(Gf.Vec3f(1, 1, 1))
    prim.SetResetXformStack(True)
    _Check(bool(UsdGeom.XformCommonAPI(prim.GetPrim())),
           "a reset stack is still XformCommonAPI-compatible")
    kid = UsdGeom.Xform.Define(stage, "/Root/T/Kid")
    kidApi = UsdGeom.XformCommonAPI(kid)
    kidApi.SetTranslate(Gf.Vec3d(0, 4, 0))
    kidApi.SetRotate(Gf.Vec3f(0, 0, 0))
    kidApi.SetScale(Gf.Vec3f(1, 1, 1))
    cache = UsdGeom.XformCache(time)
    before = cache.GetLocalToWorldTransform(kid.GetPrim())
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, prim.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, reason)
    _Check(_MatClose(target.GizmoMatrix(),
                     cache.GetLocalToWorldTransform(prim.GetPrim())
                     .GetOrthonormalized(False), 1e-6),
           "the gizmo is drawn where the prim actually is, not 100 units "
           "away: %s" % target.GizmoMatrix().ExtractTranslation())
    # A world delta maps straight onto the channels, the parent's 100
    # units of translate being reset away.
    target.SetPreserveChildren(True)
    target.AttributePaths()
    target.BeginDrag()
    target.ApplyTranslate(Gf.Vec3d(0, 5, 0))
    _Check(_Close(api.GetXformVectors(time)[0][1], 7.0, 1e-6),
           "the world delta reached the channel unscaled: %s"
           % (api.GetXformVectors(time)[0],))
    cache.Clear()
    _Check(_MatClose(cache.GetLocalToWorldTransform(kid.GetPrim()),
                     before, 1e-5),
           "and the child held its world transform")


def _ReferencedStage():
    """
    A shot layer referencing an asset, which is the ordinary layout the
    in-memory fixtures above do not reproduce.

    `/World` references an anonymous layer's `/Asset`, so `/World/Group`
    and `/World/Group/Kid` have specs ONLY in the referenced layer and
    none anywhere in the root layer stack. That is what makes the prim
    index reject a spec authored during a change block: `AddXformOp`
    validates the op it just created with a composed stage query, and
    until change processing rescans the prim the new spec is invisible.
    Every other fixture here defines its prims in the root layer, where
    the node already has specs and the query happens to succeed.

    The child carries a translate op only, so a Preserve Children drag
    has to CREATE its rotate and scale ops.
    """
    asset = Sdf.Layer.CreateAnonymous("gizmoAsset.usda")
    assetStage = Usd.Stage.Open(asset)
    UsdGeom.Xform.Define(assetStage, "/Asset")
    group = UsdGeom.Xform.Define(assetStage, "/Asset/Group")
    group.AddTranslateOp().Set(Gf.Vec3d(1, 0, 0))
    kid = UsdGeom.Xform.Define(assetStage, "/Asset/Group/Kid")
    kid.AddTranslateOp().Set(Gf.Vec3d(0, 2, 0))
    stage = Usd.Stage.CreateInMemory()
    world = UsdGeom.Xform.Define(stage, "/World")
    world.GetPrim().GetReferences().AddReference(asset.identifier, "/Asset")
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    # The anonymous layer is returned so the caller holds it: nothing
    # else owns it, and a dropped layer leaves the reference dangling.
    return stage, asset


def TestReferencedPrimDrag():
    """
    A drag on a prim reached only through a reference must not raise,
    and must not leave a half-created op stack behind.

    Creating an xform op inside the change block that writes it fails
    exactly here, and it fails on the FIRST event of a drag -- the one
    that would otherwise author the op -- so the exception ate that
    event's target write and left the child with a rotate op but no
    scale op. Op creation therefore happens before the block; only the
    value writes are batched.
    """
    stage, _layer = _ReferencedStage()
    time = Usd.TimeCode.Default()
    group = stage.GetPrimAtPath("/World/Group")
    kid = stage.GetPrimAtPath("/World/Group/Kid")
    _Check(group and kid, "the reference composed")
    _Check(not stage.GetRootLayer().GetPrimAtPath("/World/Group"),
           "the referenced prims have no root-layer spec")
    cache = UsdGeom.XformCache(time)
    before = cache.GetLocalToWorldTransform(kid)
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, group, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, reason)
    target.SetPreserveChildren(True)
    target.AttributePaths()
    target.BeginDrag()
    target.ApplyTranslate(Gf.Vec3d(0, 5, 0))
    cache.Clear()
    _Check(_Close(UsdGeom.XformCommonAPI(group).GetXformVectors(time)[0][1],
                  5.0, 1e-6),
           "the target's own write survived the first event")
    _Check(_MatClose(cache.GetLocalToWorldTransform(kid), before, 1e-5),
           "the referenced child held its world transform")
    authored = [n for n in kid.GetAuthoredPropertyNames()
                if n.startswith("xformOp")]
    _Check("xformOp:rotateXYZ" in authored and "xformOp:scale" in authored,
           "both compensation ops were created, not just the first: %s"
           % authored)
    # A second event on the same prim, with the ops now in place.
    target.BeginDrag()
    target.ApplyTranslate(Gf.Vec3d(0, 0, 3))
    cache.Clear()
    _Check(_MatClose(cache.GetLocalToWorldTransform(kid), before, 1e-5),
           "and again on the next event")


def TestNoticeFilter():
    """
    NoticeAffectsTarget and SolverPosedCache: what the controller uses to
    keep an unrelated stage edit from costing a rig walk, a
    resolveCamera() and a full manipulator reprojection.
    """
    ctl = Sdf.Path("/Asset/Rig/Controls/Parent/Child")
    root = Sdf.Path("/Asset/Rig")
    none = []
    _Check(gizmoMath.NoticeAffectsTarget(none, none, None, None),
           "no target: every notice is relevant, it may create one")
    _Check(not gizmoMath.NoticeAffectsTarget(none, none, ctl, root),
           "an empty notice affects nothing")
    _Check(gizmoMath.NoticeAffectsTarget(
        none, [ctl.AppendProperty("avars:tx")], ctl, root),
           "a property of the target prim is the target moving")
    _Check(gizmoMath.NoticeAffectsTarget(
        none, [Sdf.Path("/Asset/Rig/Controls/Parent.avars:tx")], ctl, root),
           "an ancestor's avar carries the target")
    _Check(gizmoMath.NoticeAffectsTarget(
        [Sdf.Path("/Asset")], none, ctl, root),
           "the asset root is an ancestor too")
    _Check(gizmoMath.NoticeAffectsTarget(
        none, [Sdf.Path("/Asset/Rig/Solvers/Ik.rigExec:joints")], ctl, root),
           "anything under the rig root can pose a rig target")
    _Check(not gizmoMath.NoticeAffectsTarget(
        none, [Sdf.Path("/Asset/Rig2/Ctl.avars:tx")], ctl, root),
           "another rig does not")
    _Check(not gizmoMath.NoticeAffectsTarget(
        none, [Sdf.Path("/Asset/Geo/Mesh.points")], ctl, root),
           "and neither does an unrelated prim")
    # An xform target has no rig root: only its own ancestors count, and
    # a descendant (a compensated child) does not move the gizmo.
    box = Sdf.Path("/World/Group/Box")
    _Check(gizmoMath.NoticeAffectsTarget(
        none, [Sdf.Path("/World/Group.xformOp:translate")], box, None),
           "an xform follows its ancestors")
    _Check(not gizmoMath.NoticeAffectsTarget(
        none, [Sdf.Path("/World/Group/Box/Kid.xformOp:translate")],
        box, None), "but not its children")

    stage, parent, child = _ChainStage()
    cache = gizmoMath.SolverPosedCache()
    rigRoot = stage.GetPrimAtPath(root)
    _Check(cache.For(rigRoot) == set(), "no solver in the plain chain")
    _Check(cache.For(None) == set(), "and no root answers empty")
    solver = stage.DefinePrim("/Asset/Rig/Ik", "RigExecSingleChainIk")
    solver.CreateRelationship("rigExec:joints").SetTargets(
        [child.GetPath()])
    _Check(cache.For(rigRoot) == set(),
           "the memo is stale until the caller invalidates it")
    cache.InvalidateResynced([solver.GetPath()])
    _Check(cache.For(rigRoot) == {child.GetPath()},
           "a resync inside the rig drops the memo: %s" % cache.For(rigRoot))
    cache.InvalidateResynced([Sdf.Path("/Asset")])
    _Check(cache._byRoot == {}, "a resync ABOVE the rig drops it as well")
    cache.For(rigRoot)
    cache.InvalidateResynced([Sdf.Path("/Other")])
    _Check(cache._byRoot != {}, "an unrelated resync keeps it")
    cache.Clear()
    _Check(cache._byRoot == {}, "Clear drops everything (stage replaced)")
    # A rig target handed the cache reads through it, and answers the
    # rig root path the filter needs.
    writer = gizmoMath.Writer(stage, Usd.TimeCode.Default(),
                              gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, parent, gizmoMath.CHANNELS_POSE, writer, cache)
    _Check(target is not None, reason)
    _Check(target.RigRootPath() == root,
           "a rig target reports its root: %s" % target.RigRootPath())
    _Check(cache._byRoot.get(root) == {child.GetPath()},
           "MakeTarget filled the memo")


class _CacheListener(object):
    """
    Drives a SolverPosedCache from REAL ObjectsChanged notices, exactly
    the way GizmoController._onObjectsChanged does, and records the last
    notice's paths so a test can assert what kind of change it was.
    """

    def __init__(self, stage, cache):
        self.cache = cache
        self.resynced = []
        self.changed = []
        self._key = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._OnChanged, stage)

    def _OnChanged(self, notice, sender):
        self.resynced = list(notice.GetResyncedPaths())
        self.changed = list(notice.GetChangedInfoOnlyPaths())
        if self.resynced:
            self.cache.InvalidateResynced(self.resynced)
        if self.changed:
            self.cache.InvalidateChanged(self.changed)

    def Revoke(self):
        self._key.Revoke()


def TestSolverCacheRetarget():
    """
    Re-pointing an existing rigExec:joints relationship must reach the
    memo, and it does NOT arrive as a resync.

    Driven through a real listener rather than by calling the
    invalidators directly, because the whole bug was a wrong belief
    about which notice USD sends: SetTargets on a relationship that
    already has targets is info-only, so a memo invalidated on resync
    alone kept handing out an editable target for a control the solver
    had just taken over.
    """
    stage, parent, child = _ChainStage()
    root = Sdf.Path("/Asset/Rig")
    rigRoot = stage.GetPrimAtPath(root)
    cache = gizmoMath.SolverPosedCache()
    solver = stage.DefinePrim("/Asset/Rig/Ik", "RigExecSingleChainIk")
    rel = solver.CreateRelationship(gizmoMath.SOLVER_JOINTS_REL)
    listener = _CacheListener(stage, cache)
    try:
        rel.SetTargets([child.GetPath()])
        _Check(cache.For(rigRoot) == {child.GetPath()},
               "the first SetTargets reached the memo: %s"
               % cache.For(rigRoot))
        # The one that used to be missed: the relationship already has
        # targets, so USD reports an info-only change.
        rel.SetTargets([parent.GetPath()])
        _Check(not listener.resynced and listener.changed,
               "retargeting is info-only, not a resync: resynced=%s "
               "changed=%s" % (listener.resynced, listener.changed))
        _Check(cache.For(rigRoot) == {parent.GetPath()},
               "the retarget reached the memo: %s" % cache.For(rigRoot))
        # Clearing goes back to a resync, and must land too.
        rel.ClearTargets(True)
        _Check(cache.For(rigRoot) == set(),
               "clearing the targets reached the memo: %s"
               % cache.For(rigRoot))
        # An unrelated info-only edit must NOT throw the memo away.
        # The FIRST Set creates the attribute spec, which is a resync
        # under the rig root and legitimately drops it; the second only
        # changes a value, which is the case that has to be kept.
        child.GetAttribute("avars:tx").Set(3.0)
        cache.For(rigRoot)
        child.GetAttribute("avars:tx").Set(4.0)
        _Check(not listener.resynced,
               "the second avar Set is info-only: %s" % listener.resynced)
        _Check(root in cache._byRoot,
               "an avar value change keeps the memo")
    finally:
        listener.Revoke()


def main():
    _RegisterSchema()
    groups = [
        ("euler round trip", TestEulerRoundTrip),
        ("compose avars", TestComposeAvarMatrix),
        ("rig frames replica", TestRigFramesReplica),
        ("volume weight scale", TestVolumeWeightScale),
        ("rig frames reasons", TestRigFramesReasons),
        ("writer", TestWriter),
        ("rig pose target", TestRigPoseTarget),
        ("rig pivot target", TestRigPivotTarget),
        ("xform targets", TestXformTargets),
        ("gimbal + frames", TestGimbalAndFrames),
        ("preserve children", TestPreserveChildren),
        ("snap + planar scale", TestSnapAndPlanarScale),
        ("xformOp noise", TestXformOpNoise),
        ("xform local matrix", TestXformLocalMatrix),
        ("one notice per apply", TestOneNoticePerApply),
        ("referenced prim drag", TestReferencedPrimDrag),
        ("reset xform stack", TestResetXformStack),
        ("notice filter", TestNoticeFilter),
        ("solver cache retarget", TestSolverCacheRetarget),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_MATH_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
