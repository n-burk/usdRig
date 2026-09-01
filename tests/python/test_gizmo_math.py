#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoMath.py.

Usage: test_gizmo_math.py [<generated schema resources dir>]

The frame replica is checked against the native evaluator when the
_rigexec binding is importable (build-python/python on PYTHONPATH); the
check is reported as skipped otherwise, never silently passed.
"""
import glob
import importlib.util
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(
    os.path.join(_HERE, "..", "..", "plugin", "rigExecUsdview")))


def _SetupPxr():
    """
    Make pxr importable when the ambient PYTHONPATH does not carry it.

    ctest sets ENVIRONMENT PYTHONPATH to the build-tree python directory
    ALONE -- a multi-entry value cannot be passed on Windows, where ctest
    splits the property on every ';' (see the comment in CMakeLists.txt)
    -- so a test run under ctest gets no site-packages at all. The other
    python tests bootstrap themselves for the same reason; resolve pxr
    from the configured USD install, never from a global copy whose
    binary modules would not load against this tree.
    """
    if importlib.util.find_spec("pxr") is not None:
        return
    usdInstall = os.environ.get("RIGEXEC_USD_INSTALL") or os.path.normpath(
        os.path.join(_HERE, "..", "..", "..", "usd-install"))
    candidates = [os.path.join(usdInstall, "Lib", "site-packages")]
    candidates.extend(sorted(glob.glob(os.path.join(
        usdInstall, "lib", "python*", "site-packages"))))
    for site in candidates:
        if os.path.isdir(site) and site not in sys.path:
            sys.path.insert(0, site)


_SetupPxr()

from pxr import Gf, Plug, Sdf, Usd, UsdGeom  # noqa: E402

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
    # Native comparison when the binding is available.
    try:
        import _rigexec
    except ImportError:
        print("  (native _rigexec comparison skipped: module not on "
              "PYTHONPATH; run via bin/run_python_tests.sh)")
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


def main():
    _RegisterSchema()
    groups = [
        ("euler round trip", TestEulerRoundTrip),
        ("compose avars", TestComposeAvarMatrix),
        ("rig frames replica", TestRigFramesReplica),
        ("rig frames reasons", TestRigFramesReasons),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_MATH_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
