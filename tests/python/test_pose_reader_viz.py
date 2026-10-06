"""The pose-reader visualization draws the solver's own supports.

Headless test for plugin/shapeEditor/poseReaderModel.py. Everything the
overlay paints comes out of that module, so this pins the geometry
against the ENGINE rather than against itself:

  1. At rest every rotation reader's neutral cone points exactly along
     the live driver axis, and every translation reader's driver sits
     exactly on its neutral sphere -- the frame Ref is right.
  2. With the jaw opened to a pose's own angle, the translation driver
     sits on THAT pose's sphere centre, which the engine confirms by
     publishing it at weight 1.
  3. On a synthetic rig with a parent that is not at identity, the driver
     placed exactly at a non-trivial pose rotation is published at weight
     1 by the engine AND its cone axis coincides with the live axis -- the
     quaternion convention is the evaluator's, end to end.
  4. Overlap detection, scope and the screen projection behave.

Usage: test_pose_reader_viz.py [<generated schema resources dir>]
"""
import math
import os
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _extra in (os.path.join(_ROOT, "plugin", "shapeEditor"),):
    if _extra not in sys.path:
        sys.path.insert(0, _extra)

from pxr import Gf, Plug, Sdf, Usd  # noqa: E402

import gizmoMath  # noqa: E402
import poseReaderModel as viz  # noqa: E402
import shapeEditorModel  # noqa: E402

_failures = []


def _Check(condition, message):
    if not condition:
        _failures.append(message)
        print("FAIL: %s" % message)


def _Biped():
    return Usd.Stage.Open(os.path.join(_ROOT, "examples", "biped",
                                       "Biped_stack.usda"))


def _Evaluate(stage):
    import _rigexec
    rig = _rigexec.Rig(stage, "/Biped/Rig")
    rig.compile()
    return rig


def _Readers(stage, pose):
    interps = shapeEditorModel.Discover(stage)
    shapeEditorModel.ReadWeights(interps, pose)
    return viz.Build(stage, interps, pose, Usd.TimeCode.Default(),
                     gizmoMath.RestSpace)


def TestDefaults():
    settings = viz.Settings()
    _Check(settings.enabled is False, "the visualization is off by default")
    _Check(all(settings.parts[p] for p in viz.PARTS),
           "every part is on once the master switch is")
    _Check(viz.Visible([], settings) == [], "off draws nothing")
    _Check(abs(viz.KernelCoreFraction("linear") - 0.5) < 1e-12,
           "a linear kernel is at one half halfway out")
    core = viz.KernelCoreFraction("gaussian")
    _Check(abs(viz.KernelWeight("gaussian", core) - 0.5) < 1e-12,
           "the gaussian core is where the kernel reads one half")


def TestRestFrames():
    stage = _Biped()
    rig = _Evaluate(stage)
    readers = _Readers(stage, rig.evaluate(0))
    rotation = [r for r in readers if r.kind == "rotation"]
    translation = [r for r in readers if r.kind == "translation"]
    numeric = [r for r in readers if r.kind == "numeric"]
    _Check(len(rotation) >= 30 and len(translation) >= 15,
           "the biped's readers resolve: %d rotation, %d translation"
           % (len(rotation), len(translation)))
    _Check(len(numeric) == 2 and all(not r.drawable for r in numeric),
           "the numeric readers are reported, not drawn")
    worst = 0.0
    for reader in rotation:
        for shape in reader.shapes:
            if shape.neutral and shape.kind == "cone":
                worst = max(worst, math.degrees(viz._AngleBetween(
                    shape.axis, reader.liveAxis)))
    print("    neutral cone vs live axis at rest, worst: %.2e deg" % worst)
    _Check(worst < 1e-4, "every neutral cone points along the live driver "
           "at rest: worst %.2e deg" % worst)
    worst = 0.0
    for reader in translation:
        for shape in reader.shapes:
            if shape.neutral:
                worst = max(worst,
                            (shape.centre - reader.liveOrigin).GetLength())
    print("    driver vs neutral sphere at rest, worst: %.2e" % worst)
    _Check(worst < 1e-4, "every translation driver sits on its neutral "
           "sphere at rest: worst %.2e" % worst)


def TestJawPose():
    stage = _Biped()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    jaw = stage.GetPrimAtPath(
        "/Biped/Rig/Main/Shot/Aux/Controls/M_Body/M_Torso/M_Chest/"
        "M_ChestTop/M_Neck/M_Head/M_HeadGimbal/skull_follow/M_Skull/"
        "M_LoFace/M_Jaw")
    _Check(jaw.IsValid(), "the jaw control exists")
    jaw.GetAttribute("avars:rx").Set(22.0)
    rig = _Evaluate(stage)
    readers = _Readers(stage, rig.evaluate(0))
    reader = next((r for r in readers
                   if r.name == "jaw__mouth_corner_l"), None)
    _Check(reader is not None and reader.kind == "translation",
           "the left mouth-corner reader is a translation reader")
    if reader is None:
        return
    shape = next(s for s in reader.shapes
                 if s.name == "neutral_jaw_open_22")
    _Check(abs(shape.weight - 1.0) < 1e-3,
           "the engine publishes neutral_jaw_open_22 at 1 with the jaw at "
           "22: %.6f" % shape.weight)
    gap = (shape.centre - reader.liveOrigin).GetLength()
    print("    jaw 22: weight %.6f, driver to sphere centre %.2e, "
          "%d coincident pair(s)" % (shape.weight, gap, len(
              [o for o in reader.overlaps
               if o[2] == viz.OVERLAP_COINCIDENT])))
    _Check(gap < 1e-3, "and the driver sits on that pose's sphere centre: "
           "%.2e" % gap)
    coincident = [o for o in reader.overlaps
                  if o[2] == viz.OVERLAP_COINCIDENT]
    _Check(len(coincident) >= 5,
           "pairs a twentieth of a radius apart are flagged coincident: %d"
           % len(coincident))


def _SyntheticRig(rx, ry, rz):
    """A parent with a non-identity rest and pose, a driver under it, and
    an interpolator with a neutral and one pose at (rx, ry, rz)."""
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset", "Xform")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    parent = stage.DefinePrim("/Asset/Rig/Parent", "RigExecControl")
    rest = Gf.Matrix4d(Gf.Rotation(Gf.Vec3d(0, 0, 1), 30.0) *
                       Gf.Rotation(Gf.Vec3d(1, 0, 0), 20.0),
                       Gf.Vec3d(5, 10, 0))
    parent.GetAttribute("rest:space").Set(rest)
    parent.GetAttribute("avars:ry").Set(25.0)
    driver = stage.DefinePrim("/Asset/Rig/Parent/Driver", "RigExecControl")
    driver.GetAttribute("rest:space").Set(
        Gf.Matrix4d(Gf.Rotation(Gf.Vec3d(0, 1, 0), -40.0),
                    Gf.Vec3d(0, 0, 0)) * Gf.Matrix4d(1.0).SetTranslate(
                        Gf.Vec3d(12, 0, 0)) * rest)
    driver.GetAttribute("avars:rx").Set(rx)
    driver.GetAttribute("avars:ry").Set(ry)
    driver.GetAttribute("avars:rz").Set(rz)

    interp = stage.DefinePrim("/Asset/Rig/PoseInterpolators/reader",
                              "RigExecPoseInterpolator")
    interp.CreateRelationship("rigExec:driver").SetTargets(
        [driver.GetPath()])
    interp.GetAttribute("rigExec:kernel").Set("linear")
    target = gizmoMath.ComposeAvarMatrix(0, 0, 0, 1, 1, 1, rx, ry, rz, 0,
                                         "XYZ").ExtractRotationQuat()
    for name, quat in (("neutral", Gf.Quatf(1.0)),
                       ("bent", Gf.Quatf(target))):
        pose = stage.DefinePrim(interp.GetPath().AppendChild(name),
                                "RigExecPose")
        pose.GetAttribute("rigExec:rotation").Set(quat)
        pose.GetAttribute("rigExec:rotationRadius").Set(
            float(math.radians(40.0)))
        pose.GetAttribute("rigExec:poseType").Set("swing")
    return stage


def TestQuaternionConvention():
    import _rigexec
    stage = _SyntheticRig(20.0, 35.0, -50.0)
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    rig.compile()
    pose = rig.evaluate(0)
    interps = shapeEditorModel.Discover(stage, "/Asset/Rig")
    shapeEditorModel.ReadWeights(interps, pose)
    readers = viz.Build(stage, interps, pose, Usd.TimeCode.Default(),
                        gizmoMath.RestSpace)
    _Check(len(readers) == 1 and readers[0].kind == "rotation",
           "the synthetic reader resolves as rotation")
    if not readers:
        return
    reader = readers[0]
    bent = next(s for s in reader.shapes if s.name == "bent")
    neutral = next(s for s in reader.shapes if s.name == "neutral")
    _Check(abs(bent.weight - 1.0) < 1e-4,
           "the engine reads the driver at the pose it was placed at: %.6f"
           % bent.weight)
    error = math.degrees(viz._AngleBetween(bent.axis, reader.liveAxis))
    _Check(error < 1e-4, "and that pose's cone points along the live axis: "
           "%.2e deg" % error)
    apart = math.degrees(viz._AngleBetween(neutral.axis, bent.axis))
    print("    synthetic: weight %.6f, cone vs live %.2e deg, pose %.1f deg "
          "from neutral" % (bent.weight, error, apart))
    _Check(apart > 20.0, "a real rotation, not a degenerate one: %.1f deg"
           % apart)


def TestScopeAndScreen():
    stage = _Biped()
    rig = _Evaluate(stage)
    readers = _Readers(stage, rig.evaluate(0))
    settings = viz.Settings()
    settings.enabled = True
    _Check(viz.Visible(readers, settings) == [],
           "SELECTED with nothing selected draws nothing")
    settings.scope = viz.SCOPE_ALL
    drawable = viz.Visible(readers, settings)
    _Check(len(drawable) == sum(1 for r in readers if r.drawable),
           "ALL draws every drawable reader")
    settings.scope = viz.SCOPE_SELECTED
    some = drawable[0]
    chosen = viz.Visible(readers, settings, {some.path})
    _Check([r.path for r in chosen] == [some.path],
           "SELECTED draws exactly the selection")
    found = viz.InterpolatorsFor(stage, [some.driver])
    _Check(some.path in found, "selecting a driver selects its readers")

    hull = viz.ConvexHull([(0, 0), (4, 0), (4, 4), (0, 4), (2, 2), (1, 3)])
    _Check(len(hull) == 4, "the hull of a square and two interior points "
           "is the square: %s" % (hull,))

    # A straight-down orthographic-ish projector over the cone's apex.
    cam = Gf.Camera()
    cam.transform = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 0, 200))
    vp = (0, 0, 800, 800)
    frustum = cam.frustum
    proj = viz.Projector(frustum.ComputeViewMatrix() *
                         frustum.ComputeProjectionMatrix(), vp,
                         Gf.Vec3d(1, 0, 0))
    ops = viz.ScreenOps([some], settings, proj)
    _Check(len(ops) > 0, "a visible reader produces draw operations")
    settings.enabled = False
    _Check(viz.ScreenOps([some], settings, proj) == [],
           "and none at all once switched off")


def main():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    for name, fn in (("defaults", TestDefaults),
                     ("rest frames", TestRestFrames),
                     ("jaw pose", TestJawPose),
                     ("quaternion convention", TestQuaternionConvention),
                     ("scope and screen", TestScopeAndScreen)):
        fn()
        print("  ok: %s" % name if not _failures else "  ran: %s" % name)
    if _failures:
        print("%d FAILURE(S)" % len(_failures))
        return 1
    print("RIGEXEC_POSE_READER_VIZ_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
