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


def TestDefaultSpacesAndUnits():
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    parent.GetAttribute("default:tx").Set(2.0)
    parent.GetAttribute("default:rz").Set(25.0)
    child.GetAttribute("default:ty").Set(4.0)
    child.GetAttribute("default:rx").Set(15.0)
    child.GetAttribute("avars:unitScaleFactor").Set(2.5)
    native = _NativeModule()
    rig = native.Rig(stage, "/Asset/Rig") if native else None
    if rig:
        rig.compile()

    def check_native():
        frames = gizmoMath.ComputeRigFrames(stage, child, time)
        _Check(frames.reason == "", frames.reason)
        if rig:
            matrix = Gf.Matrix4d(*rig.evaluate(-1.0).control_frame(
                str(child.GetPath())).to_matrix4())
            _Check(_MatClose(matrix, frames.posed, 1e-5),
                   "default spaces replica vs native:\n%s\n%s"
                   % (matrix, frames.posed))
        return frames

    check_native()
    for name, amount in (("default:space", 30.0),
                         ("avars:defaultSpace", 40.0),
                         ("posed:defaultSpace", 50.0),
                         ("parent:defaultSpace", 20.0),
                         ("parent:space", 60.0)):
        matrix = _Rot(Gf.Vec3d(0, 1, 0), amount)
        matrix.SetTranslateOnly(Gf.Vec3d(amount, 2, 1))
        child.GetAttribute(name).Set(matrix)
        check_native()
    driver = stage.DefinePrim("/Asset/Rig/Driver", "Scope")
    source = driver.CreateAttribute("space", Sdf.ValueTypeNames.Matrix4d)
    source.Set(Gf.Matrix4d(1.0))
    child.GetAttribute("posed:defaultSpace").SetConnections([source.GetPath()])
    check_native()
    source.Set(_Rot(Gf.Vec3d(0, 0, 1), 33.0))
    check_native()

    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    for units in (2.5, -0.5):
        child.GetAttribute("avars:unitScaleFactor").Set(units)
        before = check_native()
        target, reason = gizmoMath.MakeTarget(
            stage, child, gizmoMath.CHANNELS_POSE, writer)
        _Check(target is not None, reason)
        origin = (before.posed * before.assetToWorld).ExtractTranslation()
        delta = Gf.Vec3d(0.7, -1.3, 2.1)
        _Drag(target, lambda: target.ApplyTranslate(delta))
        after = check_native()
        moved = (after.posed * after.assetToWorld).ExtractTranslation()
        _Check((moved - origin - delta).GetLength() < 1e-6,
               "unit-scaled world translation preserves the requested delta")
    pivot, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(pivot is None and "independently of rest" in reason,
           "explicit default-space pivot edits identify the authoritative source")
    child.GetAttribute("avars:unitScaleFactor").Set(0.0)
    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is None and "unit scale" in reason,
           "zero unit scale cannot be inverted for a translation drag")
    # A matrix connection can expose a solver-owned frame from a different
    # namespace. Refuse that unavailable pose rather than use its authored rest.
    joint = stage.DefinePrim("/Asset/Rig/J", "RigExecJoint")
    relay = stage.DefinePrim("/Asset/Rig/J/Relay", "RigExecControl")
    solver = stage.DefinePrim("/Asset/Rig/Solver", "RigExecFkChain")
    solver.GetRelationship("rigExec:joints").SetTargets([joint.GetPath()])
    goal = stage.DefinePrim("/Asset/Rig/Goal", "RigExecControl")
    goal.GetAttribute("parent:space").SetConnections(
        [relay.GetAttribute("parent:space").GetPath()])
    target, reason = gizmoMath.MakeTarget(
        stage, goal, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is None and "space source" in reason and "solver" in reason,
           "connected solver-space origin must not use an unsolved replica")
    # A directly selected parent matrix removes namespace pose inheritance.
    replacement = Gf.Matrix4d(1.0)
    replacement.SetTranslateOnly(Gf.Vec3d(10, 2, 0))
    relay.GetAttribute("parent:space").Set(replacement)
    target, reason = gizmoMath.MakeTarget(
        stage, relay, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None and reason == "",
           "explicit parent space remains editable below a solver-owned joint")


def TestCurvenetAdjustmentFrames():
    native = _NativeModule()
    if native is None:
        return
    import rigexec
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Asset").AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    builder = rigexec.Builder.create(stage, "/Asset/Rig")
    net = builder.add_curvenet("Net", [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0),
                                       (0, 1, 0), (0, 2, 0), (0, 3, 0)])
    net.add_spline(0, 1, 2, 3)
    net.add_spline(0, 4, 5, 6)
    netToAsset = _Rot(Gf.Vec3d(0, 1, 0), 20)
    netToAsset.SetTranslateOnly(Gf.Vec3d(10, 20, 30))
    UsdGeom.Xformable(stage.GetPrimAtPath(net.path)).AddTransformOp().Set(netToAsset)
    knot = builder.add_curvenet_adjustment("Knot", net.path, 0)
    knot.set_avar_translation(1, 0, 0)
    knot.set_avar_scale(2, 1, 1)
    prim = stage.GetPrimAtPath(knot.path)
    prim.GetAttribute("avars:unitScaleFactor").Set(2.5)
    warp = builder.add_control("Warp")
    warp.set_avar_rotation(0, 0, 90)
    warp.set_avar_translation(5, 6, 7)
    chain = builder.new_mover_chain("Shape", net.path + ".points")
    chain.add_curvenet_adjuster_mover("Adjust", [knot.path])
    chain.add_matrix_mover("Warp", warp.path)
    rig = native.Rig(stage, "/Asset/Rig")
    rig.compile()
    time = Usd.TimeCode(1)

    def published(queryStage, queryPath, queryTime):
        if queryStage != stage or str(queryPath) != knot.path or queryTime != time:
            return None
        return Gf.Matrix4d(*rig.evaluate(1).control_frame(knot.path).to_matrix4())

    gizmoMath.SetPublishedControlFrameReader(published)
    try:
        frames = gizmoMath.ComputeRigFrames(stage, prim, time)
        _Check(frames.reason == "", frames.reason)
        _Check(_MatClose(frames.posed, published(stage, prim.GetPath(), time)),
               "adjustment uses the evaluated knot frame with scale")
        _Check((frames.posed.ExtractTranslation() - netToAsset.Transform(
            Gf.Vec3d(5, 8.5, 7))).GetLength() < 1e-5,
            "native adjustment control frame is in asset space")
        _Check(_MatClose(gizmoMath.AvarsMatrix(prim, time) * frames.P, frames.posed),
               "adjustment avar scope reconstructs the published frame")
        _Check(_Close(frames.unitScale, 2.5), "adjustment translation units")
        writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
        target, reason = gizmoMath.MakeTarget(stage, prim, gizmoMath.CHANNELS_POSE, writer)
        _Check(target is not None, reason)
        origin = (frames.posed * frames.assetToWorld).ExtractTranslation()
        delta = Gf.Vec3d(0.7, -1.3, 2.1)
        _Drag(target, lambda: target.ApplyTranslate(delta))
        after = gizmoMath.ComputeRigFrames(stage, prim, time)
        moved = (after.posed * after.assetToWorld).ExtractTranslation()
        _Check((moved - origin - delta).GetLength() < 1e-5,
               "posed knot drag follows preceding deformation and unit scale")
        pivot, reason = gizmoMath.MakeTarget(stage, prim, gizmoMath.CHANNELS_PIVOT, writer)
        _Check(pivot is None and "preceding deformation" in reason,
               "automatic knot pivot identifies its source")
        stale = gizmoMath.ComputeRigFrames(stage, prim, Usd.TimeCode(2))
        _Check("Activate" in stale.reason, "stale published frames cannot drive a drag")
    finally:
        gizmoMath.SetPublishedControlFrameReader(None)


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
    """
    A drag collects; a release authors (design note: docs/superpowers/specs/
    2026-09-10-hydra-preview-manipulation-design.md).

    The two halves are asserted separately, because the whole point of the
    split is that the first one touches nothing: while the mouse is down the
    stage still holds the pre-drag value, and what the artist sees comes from
    Hydra instead.
    """
    stage, parent, child = _ChainStage()
    attr = child.GetAttribute("avars:tx")
    anim = gizmoMath.Writer(stage, Usd.TimeCode(1001.0),
                            gizmoMath.WRITE_ANIMATION)

    # Collecting. Nothing is authored -- not a knot, not a sample, not a
    # default -- and the value is held where the preview channel reads it.
    anim.Set(attr, 2.0)
    _Check(not attr.HasSpline() and attr.GetNumTimeSamples() == 0
           and stage.GetRootLayer().GetAttributeAtPath(attr.GetPath()) is None,
           "a collected value authors nothing")
    pending = anim.Pending()
    _Check(list(pending.keys()) == [attr.GetPath()]
           and _Close(pending[attr.GetPath()], 2.0),
           "the collected value is pending")

    # Several samples of one drag: the last one is what gets authored, and the
    # ones before it never existed as far as the document is concerned.
    anim.Set(attr, 2.5)
    anim.Set(attr, 3.0)
    _Check(len(anim.Pending()) == 1, "a channel is pending once, at its last "
           "value")
    _Check(anim.Warnings() == [], "nothing to warn about before authoring")

    # Releasing.
    authored = anim.CommitToStage()
    _Check(authored == [attr.GetPath()], "the commit reports what it authored")
    _Check(attr.HasSpline() and len(attr.GetSpline().GetKnots()) == 1,
           "animation mode writes ONE spline knot for the whole drag")
    _Check(_Close(attr.Get(Usd.TimeCode(1001.0)), 3.0), "knot value")
    # the conventional default new key (graphModel.AuthorKnot), so a gizmo drag and
    # a graph-editor insert produce the same knot.
    knot = attr.GetSpline().GetKnot(1001.0)
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmAutoEase
           and knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmAutoEase,
           "the authored knot has AutoEase tangents on both sides")
    _Check(knot.GetNextInterpolation() == Ts.InterpCurve,
           "and a curve segment after it")

    # A second drag over the same frame updates that knot rather than adding
    # one beside it.
    anim.Set(attr, 4.0)
    anim.CommitToStage()
    _Check(len(attr.GetSpline().GetKnots()) == 1
           and _Close(attr.Get(Usd.TimeCode(1001.0)), 4.0),
           "re-writing the same frame updates the knot")
    _Check(anim.Warnings() == [], "no warnings in animation mode")

    # An abandoned drag: collected, then dropped. Nothing reaches the stage,
    # which is why an aborted gizmo drag has nothing to undo.
    ty = child.GetAttribute("avars:ty")
    anim.Set(ty, 7.0)
    anim.Clear()
    _Check(anim.Pending() == {}, "Clear drops the collected values")
    _Check(anim.CommitToStage() == [] and ty.Get() != 7.0,
           "a cleared drag authors nothing")

    default = gizmoMath.Writer(stage, Usd.TimeCode(1001.0),
                               gizmoMath.WRITE_DEFAULT)
    default.Set(attr, 9.0)
    # The warning belongs to authoring, not to collecting: it says what the
    # artist will see, and until the value is authored there is nothing to see.
    _Check(default.Warnings() == [], "the outranked-default warning waits for "
           "the commit")
    default.CommitToStage()
    _Check(stage.GetRootLayer().GetAttributeAtPath(
        attr.GetPath()).default == 9.0, "default mode writes the default")
    _Check(len(default.Warnings()) == 1
           and "avars:tx" in default.Warnings()[0],
           "default outranked by the spline is reported: %s"
           % default.Warnings())
    clean = child.GetAttribute("avars:ty")
    default.Set(clean, 1.0)
    default.CommitToStage()
    _Check(not clean.HasSpline() and clean.Get() == 1.0, "plain default")
    # A vector attribute (xformOp) gets a time sample, not a spline.
    xf = UsdGeom.Xform.Define(stage, "/Asset/Box")
    op = xf.AddTranslateOp()
    anim.Set(op.GetAttr(), Gf.Vec3d(1, 2, 3))
    anim.CommitToStage()
    _Check(op.GetAttr().GetNumTimeSamples() == 1, "vec3 -> time sample")


def _Drag(target, fn):
    """
    One complete gesture: press, apply, release.

    The release is what authors. A drag COLLECTS its values and hands them to
    Hydra (gizmoMath.Writer), so a test that stopped after fn() would be
    asserting against the pre-drag stage -- the values are real from the
    artist's point of view all through the drag, and real on the stage from
    here.
    """
    target.BeginDrag()
    fn()
    target.writer.CommitToStage()
    target.Refresh()


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
    # Parent-relative rest: the pivot frame is the local rest carried into
    # the parent's REST frame (not its pose, which is what keeps the pivot
    # still under an animated ancestor).
    expectedOrigin = (before.restLocal * before.Qrest * before.parentRest
                      * before.assetToWorld).ExtractTranslation()
    origin = target.GizmoMatrix().ExtractTranslation()
    _Check(all(_Close(origin[i], expectedOrigin[i]) for i in range(3)),
           "pivot gizmo sits at the rest frame origin")
    avarsBefore = [child.GetAttribute(n).Get() for n in gizmoMath.AVAR_T]
    delta = Gf.Vec3d(1.0, 2.0, -0.5)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    after = gizmoMath.ComputeRigFrames(stage, child, time)
    moved = (after.restLocal * after.Qrest * after.parentRest
             * after.assetToWorld).ExtractTranslation()
    _Check(all(_Close(moved[i], expectedOrigin[i] + delta[i], 1e-6)
               for i in range(3)), "pivot translate maps onto rest:t")
    _Check([child.GetAttribute(n).Get() for n in gizmoMath.AVAR_T]
           == avarsBefore, "pivot mode never touches avars")
    target.Refresh()
    base = (after.restLocal * after.Qrest * after.parentRest
            * after.assetToWorld).GetOrthonormalized(False)
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(1, 0, 0), -20.0))
    rotated = gizmoMath.ComputeRigFrames(stage, child, time)
    rotWorld = (rotated.restLocal * rotated.Qrest * rotated.parentRest
                * rotated.assetToWorld).GetOrthonormalized(False)
    expected = base * _Rot(Gf.Vec3d(1, 0, 0), -20.0)
    expected.SetTranslateOnly(rotWorld.ExtractTranslation())
    _Check(_MatClose(rotWorld, expected, 1e-6), "pivot rotate onto rest:r")
    _Check(len(target.AttributePaths()) == 6, "six rest channels")


def TestPivotUnderSolver():
    """
    A joint a solver poses keeps its pivot: the evaluator overrides only
    computePointFrame, and measures a TwoBoneIk's bone lengths from the
    bound joints' rest frames, so rest:t/r stays live, authored data.
    The manipulator therefore sits on the rest frame -- which owes
    nothing to the parent's pose, let alone to the solve.
    """
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    solver = stage.DefinePrim("/Asset/Rig/Solvers/Ik", "RigExecTwoBoneIk")
    solver.GetRelationship("rigExec:joints").SetTargets([child.GetPath()])

    refused, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and "solver" in reason,
           "pose stays refused on a solver-posed joint: %r" % reason)

    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(target is not None and target.kind == "rig-pivot",
           "pivot is offered on a solver-posed joint: %r" % reason)

    frames = gizmoMath.ComputeRigFrames(stage, child, time)
    restWorld = frames.rest * frames.assetToWorld
    posedWorld = frames.restLocal * frames.Q * frames.assetToWorld
    _Check(not _MatClose(restWorld, posedWorld, 1e-6),
           "the animated parent makes rest and posed frames differ, so "
           "the assertion below can tell them apart")
    expected = restWorld.ExtractTranslation()
    origin = target.GizmoMatrix().ExtractTranslation()
    _Check(all(_Close(origin[i], expected[i]) for i in range(3)),
           "pivot anchors on the rest frame, not the parent's pose: "
           "%s vs %s" % (origin, expected))

    delta = Gf.Vec3d(1.0, 2.0, -0.5)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    moved = (gizmoMath.ComputeRigFrames(stage, child, time).rest
             * frames.assetToWorld).ExtractTranslation()
    _Check(all(_Close(moved[i], expected[i] + delta[i], 1e-6)
               for i in range(3)),
           "a pivot drag moves the rest frame by the world delta")


def _SolverChainStage():
    """
    Three joints in ONE namespace chain, all named by one TwoBoneIk.

    The shape of examples/components/spider_leg_ik.usd and of
    examples/02_TwoBoneIkLeg.usda: every joint below the chain root has a
    solver-posed ANCESTOR, which is what TestPivotUnderSolver's two-prim
    stage never produces (it binds the leaf, whose parent is untouched).
    """
    stage = Usd.Stage.CreateInMemory()
    asset = UsdGeom.Xform.Define(stage, "/Asset")
    asset.AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Joints", "Scope")
    joints = []
    path = "/Asset/Rig/Joints"
    for index, name in enumerate(("Root", "Mid", "End")):
        path = "%s/%s" % (path, name)
        joint = stage.DefinePrim(path, "RigExecJoint")
        space = _Rot(Gf.Vec3d(0, 0, 1), 20.0 * (index + 1))
        space.SetTranslateOnly(Gf.Vec3d(0, 4.0 * index, 0))
        joint.GetAttribute("rest:space").Set(space)
        joint.GetAttribute("rest:tx").Set(float(index) + 1.0)
        # Authored and ignored: the solver owns the pose, which is the
        # whole reason Pose is refused and Pivot must not be.
        joint.GetAttribute("avars:ty").Set(0.5)
        joints.append(joint)
    solver = stage.DefinePrim("/Asset/Rig/Solvers/Ik", "RigExecTwoBoneIk")
    solver.GetRelationship("rigExec:joints").SetTargets(
        [joint.GetPath() for joint in joints])
    return stage, joints


def TestPivotBelowSolverPosedJoint():
    """
    The joints BELOW the one a solver names keep their pivots too.

    Resolving the POSE side of such a joint needs its ancestor's posed
    frame, which the replica cannot reproduce for a solver-driven
    ancestor and correctly gives up on. That give-up used to travel
    through _RefuseBothModes and take Pivot with it, leaving every joint
    under a chain root unmanipulable -- the mid joint above all, which is
    exactly where a re-proportioned limb is visible
    (examples/components/spider_leg_ik.usd: ankle and foot refused).
    rest:t/r is authored data the solver READS, so it stays editable.
    """
    stage, joints = _SolverChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    cache = gizmoMath.SolverPosedCache()

    origins = []
    for joint in joints:
        refused, reason = gizmoMath.MakeTarget(
            stage, joint, gizmoMath.CHANNELS_POSE, writer, cache)
        _Check(refused is None and "solver" in reason,
               "%s: pose stays refused under the solver: %r"
               % (joint.GetName(), reason))

        target, reason = gizmoMath.MakeTarget(
            stage, joint, gizmoMath.CHANNELS_PIVOT, writer, cache)
        _Check(target is not None and target.kind == "rig-pivot",
               "%s: pivot is offered below a solver-posed joint: %r"
               % (joint.GetName(), reason))

        frames = gizmoMath.ComputeRigFrames(stage, joint, time)
        _Check(frames.pivotReason == "",
               "%s: nothing blocks the pivot: %r"
               % (joint.GetName(), frames.pivotReason))
        expected = (gizmoMath.RestSpace(joint, time)
                    * frames.assetToWorld).ExtractTranslation()
        origin = target.GizmoMatrix().ExtractTranslation()
        _Check(all(_Close(origin[i], expected[i]) for i in range(3)),
               "%s: pivot anchors on its own rest frame, %s vs %s"
               % (joint.GetName(), origin, expected))
        origins.append(origin)

    # Each joint's rest frame is its own: a manipulator that collapsed
    # onto a shared frame would pass the check above vacuously.
    _Check(len(set(tuple(round(v, 6) for v in o) for o in origins)) == 3,
           "the three rest frames are distinct: %s" % (origins,))

    mid = joints[1]
    target, reason = gizmoMath.MakeTarget(
        stage, mid, gizmoMath.CHANNELS_PIVOT, writer, cache)
    _Check(target is not None, reason)
    delta = Gf.Vec3d(1.0, 2.0, -0.5)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    moved = (gizmoMath.RestSpace(mid, time)
             * gizmoMath.ComputeRigFrames(stage, mid, time).assetToWorld
             ).ExtractTranslation()
    _Check(all(_Close(moved[i], origins[1][i] + delta[i], 1e-6)
               for i in range(3)),
           "a pivot drag on the mid joint moves its rest frame by the "
           "world delta: %s vs %s" % (moved, origins[1] + delta))
    _Check(_Close(mid.GetAttribute("avars:ty").Get(), 0.5),
           "the pivot drag never touched an avar")


def TestPivotDrivesIkSolve():
    """
    The far end of what the pivot is FOR: dragging it moves the solve,
    so the viewport moves.

    A TwoBoneIk with unauthored absolute lengths measures each bone
    between the bound joints' rest origins on every evaluation, so a rest
    edit re-proportions the limb. The root and the end stay pinned by
    rigExec:rootControl and rigExec:effectorControl -- the MID joint is
    where the result shows, which is why refusing its pivot (it is always
    a descendant of the solver-posed chain root) made the feature look
    broken even though the solver was reading rests all along.

    Native, because only the evaluator can answer whether a solve moved.
    """
    _rigexec = _NativeModule()
    if _rigexec is None:
        return
    stage = Usd.Stage.CreateInMemory()
    builder = _rigexec.Builder.create(stage, "/Rig")

    def _At(y):
        m = Gf.Matrix4d(1.0)
        m.SetTranslateOnly(Gf.Vec3d(0, y, 0))
        return [m[r][c] for r in range(4) for c in range(4)]

    root = builder.add_joint("Root", _At(0.0))
    mid = builder.add_joint("Mid", _At(-3.0), parent_joint=root)
    # Parent-relative rest: two 3-unit steps down Y, world 0 / -3 / -6.
    end = builder.add_joint("End", _At(-3.0), parent_joint=mid)
    rootControl = builder.add_control("RootCtl")
    effector = builder.add_control("EffectorCtl")
    pole = builder.add_control("PoleCtl")
    # Inside reach (3 + 3 against 5), so the chain bends toward the pole
    # instead of resolving through the unreachable policy.
    effector.set_avar_translation(0.0, -5.0, 0.0)
    pole.set_avar_translation(0.0, -2.5, 3.0)
    ik = builder.add_two_bone_ik("Ik", rootControl, effector, pole)
    ik.set_joints([root, mid, end])

    rig = _rigexec.Rig(stage, "/Rig")
    rig.compile()  # raises ValueError with every message on failure

    def _Solved():
        # World origins, not joint_matrix: that is the rest-to-pose delta,
        # and a pivot drag moves the REST, so the delta changes for a joint
        # whose world position the solve is holding perfectly still.
        pose = rig.evaluate(1.0)
        return {handle.name: Gf.Matrix4d(
                    *pose.joint_frame(str(handle.path), True).to_matrix4())
                .ExtractTranslation()
                for handle in (root, mid, end)}

    before = _Solved()
    time = Usd.TimeCode.Default()
    midPrim = stage.GetPrimAtPath(Sdf.Path(str(mid.path)))
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, midPrim, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(target is not None and target.kind == "rig-pivot",
           "the mid joint offers a pivot: %r" % reason)
    _Check(target.Advisory() == "",
           "unauthored lengths: the drag is not reported inert, got %r"
           % target.Advisory())

    # The manipulator is on the REST frame, not on the solve: the knee is
    # bent toward the pole, the rest chain is straight down Y.
    origin = target.GizmoMatrix().ExtractTranslation()
    _Check(_Close(origin[1], -3.0) and _Close(origin[2], 0.0),
           "the pivot draws at the rest origin, not the solved knee "
           "(%s vs solved %s)" % (origin, before["Mid"]))
    _Check(abs(before["Mid"][2] - origin[2]) > 0.1,
           "the solve bends the knee away from its rest, so the check "
           "above is not vacuous: solved %s" % (before["Mid"],))

    # Shorten the upper bone. The lower one keeps its length: rest is
    # parent-relative, so lifting the mid carries the end's rest with it
    # rather than stretching the segment between them.
    _Drag(target, lambda: target.ApplyTranslate(Gf.Vec3d(0.0, 1.0, 0.0)))
    rig.compile()
    after = _Solved()

    _Check(not _Close(after["Mid"][1], before["Mid"][1], 1e-4)
           or not _Close(after["Mid"][2], before["Mid"][2], 1e-4),
           "the pivot drag moved the solved mid joint: %s -> %s"
           % (before["Mid"], after["Mid"]))
    for name in ("Root", "End"):
        _Check(all(_Close(after[name][i], before[name][i], 1e-4)
                   for i in range(3)),
               "%s stays pinned by its control: %s -> %s"
               % (name, before[name], after[name]))


def TestIkLengthAdvisoryIsGone():
    """
    Bone lengths are measured from the bound joints' rests, and
    rigExec:upperLength / rigExec:lowerLength no longer exist in the
    schema at all.
    So a pivot drag ALWAYS reaches the solve and the manipulator has
    nothing to warn about -- the advisory this used to carry described
    an opt-out that no longer exists.
    """
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    solver = stage.DefinePrim("/Asset/Rig/Solvers/Ik", "RigExecTwoBoneIk")
    solver.GetRelationship("rigExec:joints").SetTargets([child.GetPath()])

    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(target is not None, reason)
    _Check(target.Advisory() == "",
           "lengths measure the rests: no advisory, got %r"
           % target.Advisory())

    # Even a leftover custom opinion from an asset saved against the old
    # schema cannot freeze the bone -- there is no absolute length input
    # any more, and Compile rejects the stale property outright.
    solver.CreateAttribute("rigExec:upperLength", Sdf.ValueTypeNames.Double,
                           custom=True).Set(4.0)
    target.Refresh()
    _Check(target.Advisory() == "",
           "no length can freeze a bone any more, got %r"
           % target.Advisory())

    # A pose target never carried it either: a rest-channel concern.
    poseTarget, _ = gizmoMath.MakeTarget(
        stage, parent, gizmoMath.CHANNELS_POSE, writer)
    _Check(poseTarget is not None and poseTarget.Advisory() == "",
           "an unbound control has nothing to advise about")


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
    # All three manipulators are centred on the point the rotate and
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
    the conventional Gimbal rotate axes and the frames the handles are drawn in
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
    # Pivot mode: XYZ rest angles, expressed in Qrest. Unlike Q it does
    # NOT ride on the parent's avars -- rest:t/r are authored against
    # rest:space alone -- so the scale change above cannot move it.
    frames = gizmoMath.ComputeRigFrames(stage, child, time)
    pivot, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(pivot is not None, reason)
    pivotOrder, pivotAngles = pivot.RotationState()
    _Check(pivotOrder == "XYZ" and _Close(pivotAngles[1], 45.0),
           "pivot rotation state is the XYZ rest angles: %s" % (pivotAngles,))
    pivotExpected = (frames.Qrest * frames.parentRest
                     * frames.assetToWorld).GetOrthonormalized(False)
    pivotExpected.SetTranslateOnly(pivot.GizmoMatrix().ExtractTranslation())
    _Check(_MatClose(pivot.ChannelFrame(), pivotExpected),
           "the pivot channel frame is Qrest * parentRest")
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


def TestRigPivotPreservesChildren():
    """
    Preserve Children on a rig pivot holds the child's WORLD rest still.

    Rest offsets are parent-relative, so a pivot drag carries the subtree
    by default -- that is the point. With Preserve Children on, each
    immediate child's own rest:t/r absorb the parent's delta so the child
    stays where it was, in translation and in rotation alike, since the
    correction is a full matrix identity rather than a vector subtraction.
    """
    for describe, drag in (
            ("translate", lambda tgt: tgt.ApplyTranslate(
                Gf.Vec3d(1.0, 2.0, -0.5))),
            ("rotate", lambda tgt: tgt.ApplyRotate(
                Gf.Vec3d(0, 0, 1), 25.0))):
        stage, parent, child = _ChainStage()
        time = Usd.TimeCode.Default()
        childRestBefore = gizmoMath.RestSpace(child, time)

        writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
        target, reason = gizmoMath.MakeTarget(
            stage, parent, gizmoMath.CHANNELS_PIVOT, writer)
        _Check(target is not None, reason)
        _Check(target.supportsPreserveChildren,
               "a rig pivot supports Preserve Children")
        target.SetPreserveChildren(True)
        _Check(target.preserveChildren, "Preserve Children stayed off")
        paths = target.AttributePaths()
        _Check(any(str(p).startswith(str(child.GetPath())) for p in paths),
               "the child's channels are declared for undo: %s"
               % [str(p) for p in paths])

        _Drag(target, lambda: drag(target))

        childRestAfter = gizmoMath.RestSpace(child, time)
        _Check(_MatClose(childRestAfter, childRestBefore, 1e-6),
               "%s: the child's world rest moved:\n%s\n%s"
               % (describe, childRestAfter, childRestBefore))


def TestRigPivotCarriesChildrenWhenOff():
    """Without Preserve Children the subtree follows -- the new default."""
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    before = gizmoMath.RestSpace(child, time).ExtractTranslation()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, parent, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(target is not None, reason)
    target.SetPreserveChildren(False)
    delta = Gf.Vec3d(1.0, 2.0, -0.5)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    after = gizmoMath.RestSpace(child, time).ExtractTranslation()
    moved = Gf.Vec3d(after) - Gf.Vec3d(before)
    _Check(all(_Close(moved[i], delta[i], 1e-6) for i in range(3)),
           "the child should follow the parent by %s, moved %s"
           % (delta, moved))


def TestPreserveChildren():
    """
    the conventional Preserve Children (design spec section 8.2), default off: the
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
    the conventional planar (two-axis) scale handles and Step Snap (design spec
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
    # Relative snap (the conventional J hold) quantises the DELTA.
    for name in gizmoMath.AVAR_T:
        ctl.GetAttribute(name).Set(0.0)
    _Drag(target, lambda: target.ApplyTranslate(
        Gf.Vec3d(0.7, 0.2, 0.0), snapStep=0.5))
    _Check(_Close(ctl.GetAttribute("avars:tx").Get(), 0.5)
           and _Close(ctl.GetAttribute("avars:ty").Get(), 0.0),
           "relative step snap quantises the channel delta: %s"
           % [ctl.GetAttribute(n).Get() for n in gizmoMath.AVAR_T])
    # Absolute snap (the conventional X grid hold) quantises the RESULT, which is
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


def TestOneNoticePerDrag():
    """
    A WHOLE DRAG must reach the stage as one ObjectsChanged, at its release
    (design note: docs/superpowers/specs/
    2026-09-10-hydra-preview-manipulation-design.md).

    This used to be one notice per Apply*, which was the best available answer
    while every mouse sample authored: the RigExec evaluator republishes
    synchronously on the notice, so an unbatched Preserve Children drag cost
    1 + 3N recompositions per sample for N children, all discarded but the
    last. Batching each sample into one notice took that to one per sample.

    Collecting takes it to ZERO per sample. The artist still sees every sample,
    through the preview channel into Hydra, and the document hears about the
    drag once -- when it is over and there is something to hear about.

    The exception is an event that has to CREATE an op. That authoring cannot
    be deferred: the value has nowhere to live until the op exists, and the
    preview is keyed by attribute path. The last group pins that cost to once
    per drag rather than once per sample, which is the whole trade.
    """
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, reason)
    target.BeginDrag()
    counter = _NoticeCounter(stage)
    # Three samples of one gesture, as a real drag arrives.
    target.ApplyTranslate(Gf.Vec3d(0.25, -0.5, 0.75))
    target.ApplyTranslate(Gf.Vec3d(0.50, -0.5, 0.75))
    target.ApplyTranslate(Gf.Vec3d(0.75, -0.5, 0.75))
    _Check(counter.count == 0,
           "a rig drag in progress must not touch the stage at all, got %d "
           "notice(s)" % counter.count)
    # The value the last sample collected, in the channel frame the drag
    # writes -- taken from the Writer rather than computed here, because what
    # is being asserted is that THIS is what reaches the stage, not what the
    # world delta maps to.
    txPath = child.GetPath().AppendProperty("avars:tx")
    lastSample = writer.Pending()[txPath]
    writer.CommitToStage()
    _Check(counter.count == 1,
           "the release authors three avars in ONE notice, got %d"
           % counter.count)
    counter.Revoke()
    # And the value that landed is the last sample's, not the first's.
    _Check(_Close(child.GetAttribute("avars:tx").Get(time), lastSample, 1e-9),
           "the committed value is the one the drag ended on")
    _Check(writer.Pending() == {},
           "and the commit emptied the collection")

    # An xform pose target with Preserve Children on and two children: the op
    # writes plus 3 channels on each child, still nothing until the release.
    # Every prim here carries a full T/R/S stack already, so nothing in this
    # group has an op to create.
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
    xtarget.ApplyTranslate(Gf.Vec3d(0, 3, 0))
    xtarget.ApplyTranslate(Gf.Vec3d(0, 5, 0))
    _Check(counter.count == 0,
           "an xform drag compensating two children must not touch the stage "
           "either, got %d notice(s)" % counter.count)
    xwriter.CommitToStage()
    _Check(counter.count == 1,
           "the release fires exactly one ObjectsChanged, got %d"
           % counter.count)
    counter.Revoke()
    # ... and deferring must not have cost the compensation itself: the
    # children have to be exactly where they were.
    cache.Clear()
    for kid, was in zip(kids, before):
        _Check(_MatClose(cache.GetLocalToWorldTransform(kid.GetPrim()),
                         was, 1e-5),
               "%s held its world transform through the deferred write"
               % kid.GetPath())
    _Check(_Close(UsdGeom.XformCommonAPI(group).GetXformVectors(time)[0][1],
                  5.0, 1e-6), "and the parent actually moved")

    # A prim with NO ops pays for creating them, and pays ONCE.
    #
    # Op creation is authored structure and cannot be deferred: measured on
    # this USD build, CreateXformOps for a single missing op costs three change
    # rounds -- the op attribute, then xformOpOrder, then xformOpOrder's value
    # -- and nothing at all when the ops already exist. So the first sample of
    # a drag on a bare prim pays that and every sample after it pays nothing.
    # The assertion is written to say exactly that rather than to pin a magic
    # number.
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
    _Check(second == 0 and counter.count == 0,
           "once the op exists every further sample is silent: "
           "first=%d, second=%d, third=%d" % (first, second, counter.count))
    _Check(first > 0,
           "and the creation cost fell on the first sample alone: %d" % first)
    counter.count = 0
    xwriter.CommitToStage()
    _Check(counter.count == 1, "the release is one notice, got %d"
           % counter.count)
    counter.Revoke()
    _Check(_Close(UsdGeom.XformCommonAPI(bare).GetXformVectors(time)[0][0],
                  3.0, 1e-6), "and the last sample is what landed")


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
    writer.CommitToStage()   # a drag collects; the release authors
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
    # The release: a drag collects and authors here, so the stage reads below
    # are reads of a finished gesture. What this test is about happens earlier
    # regardless -- creating an op is authored structure and still happens
    # during the drag, which is exactly why it cannot be inside a change block.
    writer.CommitToStage()
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
    writer.CommitToStage()
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


def _GroupStage():
    """
    A rig shaped like the biped's arm: a namespace CHAIN of three
    controls plus a control off to one side, and a solver-posed joint
    the gizmo has to refuse.

    The chain is the part that matters. The biped's FK controls are
    nested (rigExec:controlSpace = "parentRelative"), so a group drag
    that hands each selected control its own copy of the delta moves a
    child once per selected ancestor; this stage reproduces that in
    three prims instead of 116.
    """
    stage = Usd.Stage.CreateInMemory()
    asset = UsdGeom.Xform.Define(stage, "/Asset")
    asset.AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Controls", "Scope")

    space = _Rot(Gf.Vec3d(0, 0, 1), 90.0)
    space.SetTranslateOnly(Gf.Vec3d(0, 5, 0))
    root = stage.DefinePrim("/Asset/Rig/Controls/Root", "RigExecControl")
    root.GetAttribute("rest:space").Set(space)
    root.GetAttribute("avars:rz").Set(30.0)
    mid = stage.DefinePrim("/Asset/Rig/Controls/Root/Mid", "RigExecControl")
    mid.GetAttribute("rest:tx").Set(6.0)
    mid.GetAttribute("avars:ry").Set(15.0)
    tip = stage.DefinePrim("/Asset/Rig/Controls/Root/Mid/Tip",
                           "RigExecControl")
    tip.GetAttribute("rest:tx").Set(4.0)
    tip.GetAttribute("avars:rx").Set(-20.0)

    sideSpace = _Rot(Gf.Vec3d(0, 1, 0), 40.0)
    sideSpace.SetTranslateOnly(Gf.Vec3d(0, -8, 12))
    side = stage.DefinePrim("/Asset/Rig/Controls/Side", "RigExecControl")
    side.GetAttribute("rest:space").Set(sideSpace)
    side.GetAttribute("avars:ty").Set(2.0)

    # The stand-in for the biped's 15 parent-constrained controls: a prim
    # the rig writes outright, whose avars are inert.
    posedCtl = stage.DefinePrim("/Asset/Rig/Controls/Driven",
                                "RigExecControl")
    posedCtl.GetAttribute("rest:space").Set(sideSpace)
    mover = stage.DefinePrim("/Asset/Rig/Movers/Follow",
                             "RigExecParentConstraint")
    mover.GetRelationship("rigExec:moves").SetTargets([posedCtl.GetPath()])
    return stage, root, mid, tip, side, posedCtl


def _GroupOrigin(stage, prim, time):
    frames = gizmoMath.ComputeRigFrames(stage, prim, time)
    return (frames.posed * frames.assetToWorld).ExtractTranslation()


def _GroupWorld(stage, prim, time):
    frames = gizmoMath.ComputeRigFrames(stage, prim, time)
    return (frames.posed * frames.assetToWorld).GetOrthonormalized(False)


def _Group(stage, prims, time, channels=None):
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeGroupTarget(
        stage, prims, channels or gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, "group refused: %s" % reason)
    return target, writer


def TestGroupPivotAndFrame():
    """
    The pivot is the centroid of the members' evaluated origins,
    oriented like the LEAD (last-selected) control -- the the conventional tool/Blender
    convention.
    """
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    group, _writer = _Group(stage, [mid, tip, side], time)
    _Check(group.kind == "group" and group.label == "3 controls",
           "label: %r" % group.label)
    origins = [_GroupOrigin(stage, p, time) for p in (mid, tip, side)]
    centroid = (Gf.Vec3d(origins[0]) + Gf.Vec3d(origins[1])
                + Gf.Vec3d(origins[2])) / 3.0
    pivot = group.Pivot()
    _Check(all(_Close(pivot[i], centroid[i], 1e-9) for i in range(3)),
           "pivot is the centroid: %s vs %s" % (pivot, centroid))
    gizmo = group.GizmoMatrix()
    _Check(all(_Close(gizmo.ExtractTranslation()[i], centroid[i], 1e-9)
               for i in range(3)), "the gizmo sits on the pivot")
    lead, _ = gizmoMath.MakeTarget(stage, side, gizmoMath.CHANNELS_POSE,
                                   gizmoMath.Writer(stage, time,
                                                    gizmoMath.WRITE_DEFAULT))
    leadRotation = lead.GizmoMatrix().GetOrthonormalized(False)
    _Check(all(_Close(gizmo[r][c], leadRotation[r][c], 1e-9)
               for r in range(3) for c in range(3)),
           "the group frame is the LEAD's orientation:\n%s\n%s"
           % (gizmo, leadRotation))
    # ... and the other two orientation modes are the lead's too, moved
    # onto the pivot, so the Axis Orientation option keeps working.
    for name in ("ChannelFrame", "GimbalFrame"):
        groupFrame = getattr(group, name)()
        leadFrame = getattr(lead, name)()
        _Check(all(_Close(groupFrame[r][c], leadFrame[r][c], 1e-9)
                   for r in range(3) for c in range(3)),
               "%s follows the lead" % name)
        _Check(all(_Close(groupFrame.ExtractTranslation()[i], centroid[i],
                          1e-9) for i in range(3)),
               "%s sits on the pivot" % name)
    # A gimbal ring is one Euler channel of one prim; a group has none.
    _Check(group.RotationState() is None,
           "a real group declines gimbal rings")
    # ... but a group of ONE is not a group: it keeps its member's rings,
    # which is what a selection where everything but the lead was refused
    # falls back to.
    single, _writer = _Group(stage, [side], time)
    _Check(single.RotationState() is not None
           and single.label == side.GetName(),
           "a group of one passes the member through: %r" % single.label)


def TestGroupTranslate():
    """
    One world delta, expressed in every member's OWN space, and the
    members carried by another member are left to ride.
    """
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    selected = [mid, tip, side]
    before = {p.GetName(): _GroupOrigin(stage, p, time) for p in selected}
    untouched = _GroupOrigin(stage, root, time)
    group, writer = _Group(stage, selected, time)
    group.BeginDrag()
    # Tip is a namespace descendant of Mid, so Mid is its nearest
    # selected ancestor and Tip is walked after it.
    _Check(group._ancestor == [None, 0, None],
           "nearest selected ancestors: %s" % group._ancestor)
    _Check(group._order.index(0) < group._order.index(1),
           "shallowest first: %s" % group._order)
    delta = Gf.Vec3d(2.5, -3.25, 7.0)
    group.ApplyTranslate(delta)
    # ONE writer for the whole group: this dict is what gizmoPreview.Push
    # sends in a single UpdatePreview per mouse sample.
    pending = writer.Pending()
    # Two members author; Tip's remainder is the identity because Mid
    # already carried it the whole way, so it writes nothing -- an
    # outcome of the arithmetic, not of a rule (GroupTarget._Solve).
    _Check(len(pending) == 6,
           "six translate avars in one push: %d" % len(pending))
    writer.CommitToStage()
    group.Refresh()
    for prim in selected:
        after = _GroupOrigin(stage, prim, time)
        start = before[prim.GetName()]
        _Check(all(_Close(after[i], start[i] + delta[i], 1e-9)
                   for i in range(3)),
               "%s moved by the group delta: %s vs %s"
               % (prim.GetName(), Gf.Vec3d(after) - Gf.Vec3d(start), delta))
    _Check(all(_Close(_GroupOrigin(stage, root, time)[i], untouched[i], 1e-9)
               for i in range(3)), "an unselected control did not move")
    # The carried member's own channels were never written.
    for name in gizmoMath.AVAR_T:
        _Check(not tip.GetAttribute(name).HasAuthoredValue(),
               "the carried control authored nothing: %s" % name)
    # Every member's channels are in ONE undo list, de-duplicated.
    paths = group.AttributePaths()
    _Check(len(paths) == len(set(paths)) == 27,
           "nine avars per member, once each: %d" % len(paths))
    for prim in selected:
        _Check(prim.GetPath().AppendProperty("avars:tx") in paths,
               "%s is in the undo record" % prim.GetName())


def TestGroupTranslateIsNotAvailableInWorldUnits():
    """
    The per-member conversion is real, not a world-space memo: the
    members' channel frames are rotated 90 and 40 degrees apart here, so
    the SAME world delta lands on completely different avar values.
    """
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    group, writer = _Group(stage, [mid, side], time)
    group.BeginDrag()
    group.ApplyTranslate(Gf.Vec3d(0, 0, 5))
    writer.CommitToStage()
    midValues = [mid.GetAttribute(n).Get(time) for n in gizmoMath.AVAR_T]
    sideValues = [side.GetAttribute(n).Get(time) for n in gizmoMath.AVAR_T]
    _Check(max(abs(midValues[i] - sideValues[i]) for i in range(3)) > 1.0,
           "different spaces, different channel values: %s vs %s"
           % (midValues, sideValues))


def TestGroupRotate():
    """
    A rotation about the SHARED pivot: every member turns by the angle
    AND orbits to where the rigid rotation puts its origin.
    """
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    selected = [mid, tip, side]
    before = {p.GetName(): _GroupOrigin(stage, p, time) for p in selected}
    beforeWorld = {p.GetName(): _GroupWorld(stage, p, time)
                   for p in selected}
    untouched = _GroupOrigin(stage, root, time)
    group, writer = _Group(stage, selected, time)
    group.BeginDrag()
    pivot = Gf.Vec3d(group.Pivot())
    axis = Gf.Vec3d(0, 0, 1)
    group.ApplyRotate(axis, 30.0)
    writer.CommitToStage()
    rotation = _Rot(axis, 30.0)
    for prim in selected:
        name = prim.GetName()
        after = _GroupOrigin(stage, prim, time)
        predicted = pivot + rotation.TransformDir(
            Gf.Vec3d(before[name]) - pivot)
        _Check(all(_Close(after[i], predicted[i], 1e-9) for i in range(3)),
               "%s landed on the rigid rotation: %s vs %s"
               % (name, after, predicted))
        world = _GroupWorld(stage, prim, time)
        expected = beforeWorld[name] * rotation
        _Check(all(_Close(world[r][c], expected[r][c], 1e-9)
                   for r in range(3) for c in range(3)),
               "%s turned by 30 degrees about the world axis" % name)
        turn = (beforeWorld[name].GetInverse() * world).ExtractRotation()
        _Check(_Close(turn.GetAngle(), 30.0, 1e-6),
               "%s turned %.6f degrees" % (name, turn.GetAngle()))
    _Check(all(_Close(_GroupOrigin(stage, root, time)[i], untouched[i], 1e-9)
               for i in range(3)), "an unselected control did not move")
    # The orbit is a TRANSLATE as well as a turn: a member off the pivot
    # writes both, or it would rotate in place and tear the group apart.
    _Check(mid.GetAttribute("avars:tx").HasAuthoredValue()
           and mid.GetAttribute("avars:rz").HasAuthoredValue(),
           "an off-pivot member both turned and orbited")


def TestGroupRotateOnlyMemberIsReported():
    """
    A member whose channels cannot express the orbit keeps its turn and
    SAYS SO. Half-applying it in silence is the one outcome the group
    path must not have.
    """
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    group, writer = _Group(stage, [mid, side], time)
    # There is no rotate-only Target in the module today; the flag is the
    # contract every Apply* honours, so setting it is the whole case.
    group.members[1].supportsTranslate = False
    group.BeginDrag()
    before = _GroupOrigin(stage, side, time)
    channels = [side.GetAttribute(n).Get(time) for n in gizmoMath.AVAR_T]
    group.ApplyRotate(Gf.Vec3d(0, 0, 1), 30.0)
    writer.CommitToStage()
    after = _GroupOrigin(stage, side, time)
    _Check(all(_Close(after[i], before[i], 1e-9) for i in range(3)),
           "the rotate-only member turned in place")
    _Check("cannot translate" in group.Advisory(),
           "and the status line says so: %r" % group.Advisory())
    for name, value in zip(gizmoMath.AVAR_T, channels):
        now = side.GetAttribute(name).Get(time)
        _Check((value is None and now is None)
               or _Close(now or 0.0, value or 0.0, 1e-12),
               "nothing half-applied onto %s: %s -> %s"
               % (name, value, now))


def TestGroupScale():
    """Scale about the shared pivot: own channels, plus the orbit."""
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    selected = [mid, side]
    before = {p.GetName(): _GroupOrigin(stage, p, time) for p in selected}
    group, writer = _Group(stage, selected, time)
    group.BeginDrag()
    pivot = Gf.Vec3d(group.Pivot())
    group.ApplyScale(None, 2.0)
    writer.CommitToStage()
    for prim in selected:
        name = prim.GetName()
        after = _GroupOrigin(stage, prim, time)
        predicted = pivot + (Gf.Vec3d(before[name]) - pivot) * 2.0
        _Check(all(_Close(after[i], predicted[i], 1e-9) for i in range(3)),
               "%s scaled about the pivot: %s vs %s"
               % (name, after, predicted))
        for channel in gizmoMath.AVAR_S:
            _Check(_Close(prim.GetAttribute(channel).Get(time), 2.0, 1e-9),
                   "%s.%s doubled" % (name, channel))


def TestGroupStepSnapIsQuantisedOnce():
    """
    Step Snap quantises the WORLD delta once, not each member's channels
    in its own space -- which would move the members by different
    amounts and tear the group apart.
    """
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    selected = [mid, side]
    before = {p.GetName(): _GroupOrigin(stage, p, time) for p in selected}
    group, writer = _Group(stage, selected, time)
    group.BeginDrag()
    group.ApplyTranslate(Gf.Vec3d(2.4, 0, 0), snapStep=1.0)
    writer.CommitToStage()
    moved = []
    for prim in selected:
        after = _GroupOrigin(stage, prim, time)
        moved.append(Gf.Vec3d(after) - Gf.Vec3d(before[prim.GetName()]))
    _Check(all(_Close(moved[0][i], 2.0 if i == 0 else 0.0, 1e-9)
               for i in range(3)),
           "the world delta snapped to the step: %s" % (moved[0],))
    _Check(all(_Close(moved[0][i], moved[1][i], 1e-9) for i in range(3)),
           "and every member moved by the same one: %s vs %s"
           % (moved[0], moved[1]))


def TestGroupSkipsWhatTheRigOverwrites():
    """
    A control a RigExecParentConstraint writes is left out of the group
    and NAMED, never dragged into values the rig throws away.
    """
    stage, root, mid, tip, side, driven = _GroupStage()
    time = Usd.TimeCode.Default()
    solverPosed = gizmoMath.SolverPosedPaths(
        stage.GetPrimAtPath("/Asset/Rig"))
    _Check(driven.GetPath() in solverPosed,
           "the stage really does overwrite it: %s" % solverPosed)
    before = _GroupOrigin(stage, driven, time)
    group, writer = _Group(stage, [driven, mid, side], time)
    _Check([m.label for m in group.members] == ["Mid", "Side"],
           "the overwritten control is not a member: %s"
           % [m.label for m in group.members])
    _Check(len(group.skipped) == 1 and group.skipped[0][0] == "Driven",
           "and it is named: %s" % (group.skipped,))
    _Check("Driven" in group.Advisory()
           and "overwriting mover" in group.Advisory(),
           "the status line carries the reason: %r" % group.Advisory())
    group.BeginDrag()
    group.ApplyTranslate(Gf.Vec3d(0, 0, 9))
    group.ApplyRotate(Gf.Vec3d(0, 0, 1), 25.0)
    writer.CommitToStage()
    after = _GroupOrigin(stage, driven, time)
    _Check(all(_Close(after[i], before[i], 1e-9) for i in range(3)),
           "the refused control did not move")
    for name in gizmoMath.AVAR_T + gizmoMath.AVAR_R:
        _Check(not driven.GetAttribute(name).HasAuthoredValue(),
               "nothing was authored onto %s.%s" % (driven.GetName(), name))
    # A selection of nothing BUT refusals still answers with the reason.
    refused, reason = gizmoMath.MakeGroupTarget(
        stage, [driven], gizmoMath.CHANNELS_POSE,
        gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT))
    _Check(refused is None and "overwriting mover" in reason,
           "an all-refused selection reports the lead's reason: %r" % reason)


def TestGroupSolverPosedJointIsSkipped():
    """The existing rigExec:joints refusal reaches a group too."""
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    joint = stage.DefinePrim("/Asset/Rig/Joints/J", "RigExecJoint")
    solver = stage.DefinePrim("/Asset/Rig/Solvers/Fk", "RigExecFkChain")
    solver.GetRelationship("rigExec:joints").SetTargets([joint.GetPath()])
    group, writer = _Group(stage, [joint, mid, side], time)
    _Check([m.label for m in group.members] == ["Mid", "Side"],
           "the solver-posed joint is not a member")
    _Check("solver" in group.Advisory(),
           "and the reason is the solver's: %r" % group.Advisory())


def TestGroupCarriesOnPoseProviders():
    """
    PoseProviderPaths follows what _ComputeRigFrames actually resolves,
    which is not always the namespace parent.
    """
    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    providers = gizmoMath.PoseProviderPaths(tip, time)
    _Check(mid.GetPath() in providers and root.GetPath() in providers,
           "the namespace chain carries the tip: %s" % providers)
    _Check(Sdf.Path("/Asset") in providers,
           "and so does the asset xform that places the rig: %s" % providers)
    _Check(side.GetPath() not in providers, "a sibling does not")
    # A parent:space CONNECTION redirects the pose parent, and the group
    # has to follow it or it would hand a carried control its own delta.
    side.GetAttribute("parent:space").AddConnection(
        mid.GetAttribute("parent:space").GetPath())
    providers = gizmoMath.PoseProviderPaths(side, time)
    _Check(mid.GetPath() in providers,
           "a connected parent:space is followed: %s" % providers)


def _GroupRotateRun(mode, degrees=30.0):
    """
    One 30-degree group rotate on a fresh stage in `mode`.

    Fresh every time, because each run authors: reusing the stage would
    measure the second rotation on top of the first.
    """
    stage, _root, mid, tip, side, _driven = _GroupStage()
    time = Usd.TimeCode.Default()
    prims = [mid, tip, side]
    before = [Gf.Vec3d(_GroupOrigin(stage, p, time)) for p in prims]
    beforeWorld = [_GroupWorld(stage, p, time) for p in prims]
    group, writer = _Group(stage, prims, time)
    group.SetPivotMode(mode)
    group.BeginDrag()
    pivot = Gf.Vec3d(group.Pivot())
    group.ApplyRotate(Gf.Vec3d(0, 0, 1), degrees)
    writer.CommitToStage()
    after = [Gf.Vec3d(_GroupOrigin(stage, p, time)) for p in prims]
    return stage, prims, before, beforeWorld, after, pivot


def _Centroid3(points):
    return (points[0] + points[1] + points[2]) / 3.0


def TestGroupPivotModes():
    """
    WHERE a group turns: the CENTRE by default, and the two other
    answers a DCC offers.

    The default is the point of the option. "Rotate these together"
    means about their middle -- the centroid has to come out of the
    rotation exactly where it went in -- not about whichever control
    happened to be clicked last.
    """
    rotation = _Rot(Gf.Vec3d(0, 0, 1), 30.0)

    # -- the default: about the centre, and the centre does not move.
    _stage, prims, before, _bw, after, pivot = _GroupRotateRun(
        gizmoMath.GROUP_PIVOT_CENTER)
    _Check(all(_Close(pivot[i], _Centroid3(before)[i], 1e-9)
               for i in range(3)),
           "the default pivot IS the centroid: %s vs %s"
           % (pivot, _Centroid3(before)))
    drift = _Centroid3(after) - _Centroid3(before)
    _Check(drift.GetLength() < 1e-9,
           "and a rotation about it leaves the centroid where it was: %s"
           % (drift,))
    for prim, start, end in zip(prims, before, after):
        predicted = pivot + rotation.TransformDir(start - pivot)
        _Check((end - predicted).GetLength() < 1e-9,
               "%s is where the rigid rotation about the centre puts it: "
               "%s vs %s" % (prim.GetName(), end, predicted))

    # -- lead: the last-selected control is the one that stays put.
    _stage, prims, before, _bw, after, pivot = _GroupRotateRun(
        gizmoMath.GROUP_PIVOT_LEAD)
    _Check(all(_Close(pivot[i], before[2][i], 1e-9) for i in range(3)),
           "the lead pivot is the LAST selected control's origin")
    _Check((after[2] - before[2]).GetLength() < 1e-9,
           "so the lead does not move: %s" % (after[2] - before[2],))
    _Check((_Centroid3(after) - _Centroid3(before)).GetLength() > 0.5,
           "and the centroid does -- which is the whole difference from "
           "the default")

    # -- individual origins: everyone turns, no DRIVER moves.
    _stage, prims, before, _bw, after, _pivot = _GroupRotateRun(
        gizmoMath.GROUP_PIVOT_INDIVIDUAL)
    _Check((after[0] - before[0]).GetLength() < 1e-9
           and (after[2] - before[2]).GetLength() < 1e-9,
           "each driver turned about its own origin and stayed there")
    # The carried member moves, and correctly so: it rides on an
    # ancestor, and the ancestor turned. This is the one pivot mode that
    # is not a rigid motion of the selection, by design.
    _Check((after[1] - before[1]).GetLength() > 0.5,
           "a carried control still rides on the driver it hangs from")


def TestGroupPivotModeFallsBackToTheCentre():
    """A stale or unknown setting must not be able to break a drag."""
    stage, _root, mid, _tip, side, _driven = _GroupStage()
    time = Usd.TimeCode.Default()
    group, _writer = _Group(stage, [mid, side], time)
    group.SetPivotMode("a mode from a future version")
    _Check(group.pivotMode == gizmoMath.GROUP_PIVOT_CENTER,
           "an unknown pivot mode is the centre: %r" % group.pivotMode)


def TestGroupRotateTurnsEveryoneWhateverThePivot():
    """
    Under a RIGID pivot the angle is the angle: every member turns by
    exactly what was dragged, wherever the pivot is. Asserted on its own
    because "what moves" and "how far it turns" are easy to conflate,
    and a mistake in the orbit maths shows up here first.

    INDIVIDUAL ORIGINS is the exception, and deliberately so. Each
    selected control turns about itself, so a control that hangs off
    another selected control gets its own turn AND its ancestor's: Mid
    and Side turn 30 degrees, and Tip -- a child of Mid -- turns 60.
    That is Blender's behaviour for this mode, and it is the reason an
    animator reaches for it: selecting a finger chain and dragging the
    ring CURLS the finger instead of swinging it rigidly.
    """
    time = Usd.TimeCode.Default()
    for mode in (gizmoMath.GROUP_PIVOT_CENTER, gizmoMath.GROUP_PIVOT_LEAD):
        stage, prims, _b, beforeWorld, _a, _p = _GroupRotateRun(mode)
        for prim, start in zip(prims, beforeWorld):
            turn = (start.GetInverse()
                    * _GroupWorld(stage, prim, time)).ExtractRotation()
            _Check(_Close(turn.GetAngle(), 30.0, 1e-6),
                   "%s turned %.6f degrees in %s mode"
                   % (prim.GetName(), turn.GetAngle(), mode))
    stage, prims, _b, beforeWorld, _a, _p = _GroupRotateRun(
        gizmoMath.GROUP_PIVOT_INDIVIDUAL)
    expected = {"Mid": 30.0, "Tip": 60.0, "Side": 30.0}
    for prim, start in zip(prims, beforeWorld):
        turn = (start.GetInverse()
                * _GroupWorld(stage, prim, time)).ExtractRotation()
        _Check(_Close(turn.GetAngle(), expected[prim.GetName()], 1e-6),
               "%s turned %.6f degrees, expected %.1f"
               % (prim.GetName(), turn.GetAngle(),
                  expected[prim.GetName()]))


def TestGroupLocalAxesAreTheLeads():
    """
    What "Local" means for a multi-selection: the LAST-SELECTED
    control's frame, moved onto the group pivot.

    The axes and the pivot are independent -- this asserts the pairing
    the controller relies on (gizmoUI._Orientation feeds ObjectFrame()
    to gizmoScreen, and the pivot comes from GizmoMatrix()).
    """
    stage, _root, mid, tip, side, _driven = _GroupStage()
    time = Usd.TimeCode.Default()
    group, _writer = _Group(stage, [mid, tip, side], time)
    leadWorld = _GroupWorld(stage, side, time)
    frame = group.ObjectFrame()
    _Check(all(_Close(frame[r][c], leadWorld[r][c], 1e-9)
               for r in range(3) for c in range(3)),
           "Local is the lead's own axes")
    pivot = group.Pivot()
    _Check(all(_Close(frame.ExtractTranslation()[i], pivot[i], 1e-9)
               for i in range(3)),
           "drawn at the group pivot, not at the lead")
    # And they really are a different set of axes from Global here, or
    # the assertion above would be vacuous.
    angle = Gf.Rotation(Gf.Vec3d(0, 0, 1),
                        frame.TransformDir(Gf.Vec3d(0, 0, 1))).GetAngle()
    _Check(angle > 1.0,
           "the lead's Z is not the world Z: %.4f degrees apart" % angle)

def TestGroupRemainderSolve():
    """
    Every selected member ends up where the pivot mode says, and WHO
    authored is a result rather than a rule.

    This is the test that would have caught the skip. A rigid pivot
    leaves a member whose ancestor is also selected exactly where it
    belongs, so its remainder is the identity and it writes nothing --
    but it still has to LAND there, to the same tolerance as the member
    that did the writing. Individual Origins is not rigid, so every
    member's remainder is its own turn and every member writes.
    """
    rotation = _Rot(Gf.Vec3d(0, 0, 1), 30.0)

    def Run(mode):
        stage, _root, mid, tip, side, _driven = _GroupStage()
        time = Usd.TimeCode.Default()
        prims = [mid, tip, side]
        before = [Gf.Matrix4d(_GroupWorld(stage, p, time)) for p in prims]
        origins = [Gf.Vec3d(_GroupOrigin(stage, p, time)) for p in prims]
        group, writer = _Group(stage, prims, time)
        group.SetPivotMode(mode)
        group.BeginDrag()
        pivot = Gf.Vec3d(group.Pivot())
        group.ApplyRotate(Gf.Vec3d(0, 0, 1), 30.0)
        wrote = set(path.GetPrimPath() for path in writer.Pending())
        writer.CommitToStage()
        return stage, prims, before, origins, pivot, wrote

    # -- rigid: one member writes, all three land -------------------
    for mode in (gizmoMath.GROUP_PIVOT_CENTER, gizmoMath.GROUP_PIVOT_LEAD):
        stage, prims, before, origins, pivot, wrote = Run(mode)
        time = Usd.TimeCode.Default()
        _Check(tuple(sorted(str(p) for p in wrote))
               == ("/Asset/Rig/Controls/Root/Mid", "/Asset/Rig/Controls/Side"),
               "%s: only the members with a non-identity remainder wrote: "
               "%s" % (mode, sorted(str(p) for p in wrote)))
        for prim, start, origin in zip(prims, before, origins):
            predicted = pivot + rotation.TransformDir(origin - pivot)
            landed = Gf.Vec3d(_GroupOrigin(stage, prim, time))
            _Check((landed - predicted).GetLength() < 1e-9,
                   "%s in %s landed on the rigid prediction whether it "
                   "wrote or not: %s vs %s"
                   % (prim.GetName(), mode, landed, predicted))
            world = _GroupWorld(stage, prim, time)
            expected = start * rotation
            expected.SetTranslateOnly(world.ExtractTranslation())
            _Check(_MatClose(world, expected, 1e-9),
                   "%s in %s turned by the world rotation, not merely by "
                   "the same angle" % (prim.GetName(), mode))

    # -- individual: every member writes ----------------------------
    stage, prims, before, origins, pivot, wrote = Run(
        gizmoMath.GROUP_PIVOT_INDIVIDUAL)
    time = Usd.TimeCode.Default()
    _Check(len(wrote) == 3,
           "Individual Origins reaches every selected control, nested or "
           "not: %s" % sorted(str(p) for p in wrote))
    # And the AXIS is right for the nested one, not just the angle. A
    # descendant's channel frame has already been turned by its
    # ancestor when its own Apply* runs, and without the compensation in
    # _Author the rotation it writes lands about a conjugated axis --
    # same angle, wrong direction, which an angle-only assertion misses.
    for prim, start in zip(prims, before):
        turn = (start.GetInverse()
                * _GroupWorld(stage, prim, time)).ExtractRotation()
        axis = Gf.Vec3d(turn.GetAxis())
        _Check(abs(abs(axis[2]) - 1.0) < 1e-9,
               "%s turned about the WORLD Z it was dragged about: %s"
               % (prim.GetName(), axis))


def TestGroupTranslateCarriesWithoutDoubling():
    """
    The remainder is also what stops a translate double-applying.

    Measured with the solve removed on Biped.usda, {fk shoulder, fk
    elbow, fk wrist} asked to move 10 cm along +X moved 10.0000,
    20.0000 and 30.0000 -- one copy of the delta per selected ancestor.
    Here the chain is Root > Mid > Tip with Mid and Tip selected.
    """
    stage, root, mid, tip, side, _driven = _GroupStage()
    time = Usd.TimeCode.Default()
    prims = [mid, tip]
    before = [Gf.Vec3d(_GroupOrigin(stage, p, time)) for p in prims]
    group, writer = _Group(stage, prims, time)
    group.BeginDrag()
    delta = Gf.Vec3d(4.0, -2.0, 1.5)
    group.ApplyTranslate(delta)
    wrote = set(path.GetPrimPath() for path in writer.Pending())
    writer.CommitToStage()
    _Check(wrote == set([mid.GetPath()]),
           "the ancestor writes and the descendant's remainder is nothing: "
           "%s" % sorted(str(p) for p in wrote))
    for prim, start in zip(prims, before):
        moved = Gf.Vec3d(_GroupOrigin(stage, prim, time)) - start
        _Check((moved - delta).GetLength() < 1e-9,
               "%s moved by ONE copy of the delta: %s vs %s"
               % (prim.GetName(), moved, delta))

def TestGroupIsOneUndoEntry():
    """
    A group drag takes back in ONE step, not one per control.

    This is the controller's _BeginDrag / _EndDrag bracket run without
    Qt: one rigExecUndo.EditRecorder over GroupTarget.AttributePaths(),
    the whole drag authored by one Writer.CommitToStage(), one Edit
    pushed. The assertion that matters is the LAST one -- a single
    Undo() has to put every member back, or the artist gets half a pose
    per Ctrl+Z.
    """
    import rigExecUndo

    stage, root, mid, tip, side, _ = _GroupStage()
    time = Usd.TimeCode.Default()
    selected = [mid, tip, side]
    before = {p.GetName(): _GroupOrigin(stage, p, time) for p in selected}
    group, writer = _Group(stage, selected, time)

    recorder = rigExecUndo.EditRecorder(stage, group.AttributePaths())
    recorder.Begin()
    group.BeginDrag()
    for sample in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(2, -1, 0),
                   Gf.Vec3d(3, -2, 4)):
        # Every sample recomputes from the drag base, so the stage sees
        # nothing until the release: three mouse-moves, one edit.
        group.ApplyTranslate(sample)
        group.ApplyRotate(Gf.Vec3d(0, 1, 0), 12.0)
    _Check(not mid.GetAttribute("avars:tx").HasAuthoredValue(),
           "a drag in progress has authored nothing")
    writer.CommitToStage()
    edit = recorder.Commit("Move 3 controls")
    _Check(edit is not None and edit.label == "Move 3 controls",
           "one Edit for the whole drag")
    touched = set(entry.specPath.GetPrimPath() for entry in edit.entries)
    _Check(touched == set([mid.GetPath(), side.GetPath()]),
           "it covers every DRIVEN member and nothing else: %s" % touched)

    stack = rigExecUndo.UndoStack()
    stack.Push(edit)
    _Check(stack.CanUndo() and not stack.CanRedo(), "one entry on the stack")
    _Check(stack.Undo() and not stack.CanUndo(),
           "one Undo empties the stack: a group drag is ONE step")
    for prim in selected:
        after = _GroupOrigin(stage, prim, time)
        start = before[prim.GetName()]
        _Check(all(_Close(after[i], start[i], 1e-9) for i in range(3)),
               "%s is back where it started: %s"
               % (prim.GetName(), Gf.Vec3d(after) - Gf.Vec3d(start)))
    _Check(stack.Redo(), "and it redoes as one too")
    for prim in selected:
        after = _GroupOrigin(stage, prim, time)
        start = before[prim.GetName()]
        _Check(any(not _Close(after[i], start[i], 1e-9) for i in range(3)),
               "%s moved again" % prim.GetName())

def TestGroupOnBiped():
    """
    The real rig, with numbers: three nested FK arm controls plus a
    spine control, moved and turned as a group.

    Kept here rather than in a probe script because this is the case the
    synthetic stages above are a model OF -- nested FK controls
    (rigExec:controlSpace = "parentRelative"), a control in a completely
    different space, and 15 controls a RigExecParentConstraint
    overwrites. Skipped when the example is not checked out.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "examples", "biped", "Biped.usda")
    if not os.path.isfile(path):
        return
    time = Usd.TimeCode.Default()
    control = "/Biped/Rig/Controls"
    arm = (control + "/hips_ctl/torso_ctl/spine_end_pivot/spine_end_ctl"
           + "/clavicle_l_ctl/arm_l_root")
    shoulder = arm + "/arm_l_fk_shoulder_l_bind"
    elbow = shoulder + "/arm_l_fk_elbow_l_bind"
    wrist = elbow + "/arm_l_fk_wrist_l_bind"
    spine = control + "/hips_ctl/spine_root_pivot/spine_root_ctl"
    # The finger ROOT is now posable, and the prim the rig overwrites is
    # the FOLLOW HELPER above it. `build_fingers.py` used to parent-
    # constrain the root control itself to the wrist so the hand rode the
    # arm -- but a constraint overwrites the frame it writes, so the
    # control's own avars were discarded and ten finger roots moved 0 of
    # 26,276 skinned points on any avar. The constraint now targets a
    # helper and the control hangs off it by namespace, so it carries the
    # hand AND stays posable (measured: 612 points, 8.18 cm on rz=30).
    #
    # This case wants a prim the rig really does overwrite, so it uses the
    # helper. `arm_l_params` would do as well.
    fingerHelper = control + "/index_001_l_bind_fk_follow"
    finger = fingerHelper + "/index_001_l_bind_fk"
    witnesses = [control + "/hips_ctl", control + "/arm_l_ik"]

    def Open():
        # A fresh session layer over the CACHED file layer: Usd.Stage.Open
        # on a path reuses the globally cached SdfLayer, so avars authored
        # by one case would leak into the next.
        stage = Usd.Stage.Open(Sdf.Layer.FindOrOpen(path),
                               Sdf.Layer.CreateAnonymous())
        stage.SetEditTarget(stage.GetSessionLayer())
        return stage

    def Origin(stage, prim):
        frames = gizmoMath.ComputeRigFrames(stage, prim, time)
        return (frames.posed * frames.assetToWorld).ExtractTranslation()

    def World(stage, prim):
        frames = gizmoMath.ComputeRigFrames(stage, prim, time)
        return (frames.posed
                * frames.assetToWorld).GetOrthonormalized(False)

    # -- translate --------------------------------------------------
    stage = Open()
    selected = [stage.GetPrimAtPath(p)
                for p in (shoulder, elbow, wrist, spine)]
    before = [Origin(stage, p) for p in selected]
    beforeWitness = [Origin(stage, stage.GetPrimAtPath(p))
                     for p in witnesses]
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    group, reason = gizmoMath.MakeGroupTarget(
        stage, selected, gizmoMath.CHANNELS_POSE, writer)
    _Check(group is not None and group.label == "4 controls", reason)
    group.BeginDrag()
    # Only the topmost of the FK chain drives; the other two ride on it.
    _Check(group._ancestor == [None, 0, 1, None],
           "each FK control's nearest selected ancestor: %s"
           % group._ancestor)
    delta = Gf.Vec3d(3.5, -7.25, 11.0)
    group.ApplyTranslate(delta)
    _Check(len(writer.Pending()) == 6,
           "one preview push, two drivers: %d" % len(writer.Pending()))
    writer.CommitToStage()
    for prim, start in zip(selected, before):
        after = Origin(stage, prim)
        _Check(all(_Close(after[i], start[i] + delta[i], 1e-9)
                   for i in range(3)),
               "%s moved by the group delta: %s"
               % (prim.GetName(), Gf.Vec3d(after) - Gf.Vec3d(start)))
    for name, start in zip(witnesses, beforeWitness):
        after = Origin(stage, stage.GetPrimAtPath(name))
        _Check(all(_Close(after[i], start[i], 1e-9) for i in range(3)),
               "%s did not move" % name)

    # -- rotate -----------------------------------------------------
    stage = Open()
    selected = [stage.GetPrimAtPath(p)
                for p in (shoulder, elbow, wrist, spine)]
    before = [Origin(stage, p) for p in selected]
    beforeWorld = [World(stage, p) for p in selected]
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    group, _reason = gizmoMath.MakeGroupTarget(
        stage, selected, gizmoMath.CHANNELS_POSE, writer)
    group.BeginDrag()
    pivot = Gf.Vec3d(group.Pivot())
    centroid = (Gf.Vec3d(before[0]) + Gf.Vec3d(before[1])
                + Gf.Vec3d(before[2]) + Gf.Vec3d(before[3])) / 4.0
    _Check(all(_Close(pivot[i], centroid[i], 1e-9) for i in range(3)),
           "the biped pivot is the centroid: %s vs %s" % (pivot, centroid))
    axis = Gf.Vec3d(0, 0, 1)
    group.ApplyRotate(axis, 30.0)
    writer.CommitToStage()
    rotation = _Rot(axis, 30.0)
    for prim, start, startWorld in zip(selected, before, beforeWorld):
        after = Origin(stage, prim)
        predicted = pivot + rotation.TransformDir(Gf.Vec3d(start) - pivot)
        _Check(all(_Close(after[i], predicted[i], 1e-7) for i in range(3)),
               "%s landed on the rigid rotation: %s vs %s"
               % (prim.GetName(), after, predicted))
        turn = (startWorld.GetInverse() * World(stage, prim))\
            .ExtractRotation()
        _Check(_Close(turn.GetAngle(), 30.0, 1e-6),
               "%s turned %.6f degrees" % (prim.GetName(), turn.GetAngle()))

    # -- a mixed set with one the rig overwrites ---------------------
    stage = Open()
    selected = [stage.GetPrimAtPath(p)
                for p in (shoulder, fingerHelper, spine)]
    before = [Origin(stage, p) for p in selected]
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    group, _reason = gizmoMath.MakeGroupTarget(
        stage, selected, gizmoMath.CHANNELS_POSE, writer)
    _Check([m.prim.GetName() for m in group.members]
           == ["arm_l_fk_shoulder_l_bind", "spine_root_ctl"],
           "the parent-constrained follow helper is not a member: %s"
           % [m.prim.GetName() for m in group.members])
    group.BeginDrag()
    group.ApplyTranslate(Gf.Vec3d(0, 0, 9))
    writer.CommitToStage()
    after = Origin(stage, selected[1])
    _Check(all(_Close(after[i], before[1][i], 1e-9) for i in range(3)),
           "index_001_l_bind_fk_follow did not move: %s"
           % (Gf.Vec3d(after) - Gf.Vec3d(before[1])))
    session = stage.GetSessionLayer()
    _Check(not any(session.GetAttributeAtPath(
        selected[1].GetPath().AppendProperty(n))
        for n in gizmoMath.AVAR_T + gizmoMath.AVAR_R),
        "and nothing was authored onto it")

def main():
    _RegisterSchema()
    groups = [
        ("euler round trip", TestEulerRoundTrip),
        ("compose avars", TestComposeAvarMatrix),
        ("rig frames replica", TestRigFramesReplica),
        ("default spaces and units", TestDefaultSpacesAndUnits),
        ("curvenet adjustment frames", TestCurvenetAdjustmentFrames),
        ("volume weight scale", TestVolumeWeightScale),
        ("rig frames reasons", TestRigFramesReasons),
        ("writer", TestWriter),
        ("rig pose target", TestRigPoseTarget),
        ("rig pivot target", TestRigPivotTarget),
        ("pivot under solver", TestPivotUnderSolver),
        ("pivot below a solver-posed joint",
         TestPivotBelowSolverPosedJoint),
        ("pivot drives the ik solve", TestPivotDrivesIkSolve),
        ("ik length advisory is gone", TestIkLengthAdvisoryIsGone),
        ("xform targets", TestXformTargets),
        ("gimbal + frames", TestGimbalAndFrames),
        ("preserve children", TestPreserveChildren),
        ("rig pivot preserves children",
         TestRigPivotPreservesChildren),
        ("rig pivot carries children",
         TestRigPivotCarriesChildrenWhenOff),
        ("snap + planar scale", TestSnapAndPlanarScale),
        ("xformOp noise", TestXformOpNoise),
        ("xform local matrix", TestXformLocalMatrix),
        ("one notice per drag", TestOneNoticePerDrag),
        ("referenced prim drag", TestReferencedPrimDrag),
        ("reset xform stack", TestResetXformStack),
        ("notice filter", TestNoticeFilter),
        ("solver cache retarget", TestSolverCacheRetarget),
        ("group pivot and frame", TestGroupPivotAndFrame),
        ("group translate", TestGroupTranslate),
        ("group delta per member space",
         TestGroupTranslateIsNotAvailableInWorldUnits),
        ("group rotate about the shared pivot", TestGroupRotate),
        ("group rotate-only member is reported",
         TestGroupRotateOnlyMemberIsReported),
        ("group scale", TestGroupScale),
        ("group step snap", TestGroupStepSnapIsQuantisedOnce),
        ("group skips what the rig overwrites",
         TestGroupSkipsWhatTheRigOverwrites),
        ("group skips a solver-posed joint",
         TestGroupSolverPosedJointIsSkipped),
        ("group pose providers", TestGroupCarriesOnPoseProviders),
        ("group remainder solve", TestGroupRemainderSolve),
        ("group translate does not double",
         TestGroupTranslateCarriesWithoutDoubling),
        ("group pivot modes", TestGroupPivotModes),
        ("group pivot mode fallback",
         TestGroupPivotModeFallsBackToTheCentre),
        ("group rotate turns everyone",
         TestGroupRotateTurnsEveryoneWhateverThePivot),
        ("group local axes are the lead's",
         TestGroupLocalAxesAreTheLeads),
        ("group is one undo entry", TestGroupIsOneUndoEntry),
        ("group on the biped", TestGroupOnBiped),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_MATH_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
