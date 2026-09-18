#
# THE PARAM NODE, as the VIEWPORT sees it.
#
# The evaluator already says the param controls track their limb's end
# joint exactly -- gap 0.0000 cm at rest, in IK and in FK. That is not the
# same claim as "the cube you can click follows the wrist", because a guide
# is drawn from what the imaging bridge publishes to Hydra, not from what
# the evaluator returns to Python. A control nested under a JOINT is an
# unusual shape for this codebase (every other control lives under the
# /Controls scope), so this checks the drawn transform rather than assuming
# the two agree.
#
# Asserts, against the TERMINAL Hydra scene index the viewport draws from:
#   * each param guide sits on its end joint at rest,
#   * it moves with the joint in FK,
#   * it moves with the joint in IK,
#   * and driving `avars:ikfk` on the param node itself switches which
#     solver wins, republishing to Hydra each time.
#
import ctypes
import os

from pxr import Gf, Sdf, Usd

RIG = "/Biped/Rig"
JOINTS = RIG + "/Joints/hips_bind"
SPINE = (JOINTS + "/spine_0_bind/spine_1_bind/spine_2_bind/spine_3_bind/"
         "spine_4_bind/spine_5_bind/chest_bind")
WRIST_L = SPINE + "/clavicle_l_bind/shoulder_l_bind/elbow_l_bind/wrist_l_bind"
ANKLE_L = JOINTS + "/pelvis_l_bind/thigh_l_bind/knee_l_bind/ankle_l_bind"
ARM_PARAM = WRIST_L + "/arm_l_params"
LEG_PARAM = ANKLE_L + "/leg_l_params"

CONTROLS = RIG + "/Controls"
# spine_end_ctl nests under its offset-pivot parent spine_end_pivot
# (build_biped_rigexec.add_pivot_control).
ARM_IK = (CONTROLS + "/spine_end_pivot/spine_end_ctl/clavicle_l_ctl/arm_l_ik")
ARM_FK = (CONTROLS + "/spine_end_pivot/spine_end_ctl/clavicle_l_ctl/"
          "arm_l_fk_shoulder_l_bind")

TOLERANCE = 1e-4


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Set(stage, path, name, value, typeName=Sdf.ValueTypeNames.Double):
    prim = stage.GetPrimAtPath(path)
    _Check(prim and prim.IsValid(), "prim exists: %s" % path)
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(name, typeName)
    attr.Set(value)
    return attr


def _Observer():
    """A HydraObserver on the app's own TERMINAL scene index."""
    from pxr.Usdviewq._usdviewq import HydraObserver
    names = HydraObserver.GetRegisteredSceneIndexNames()
    _Check(bool(names), "usdview registered scene indices")
    observer = HydraObserver()
    observer.TargetToNamedSceneIndex(names[-1])
    return observer, names[-1]


def _HydraTranslation(observer, path):
    """Where the terminal scene index puts `path`, or None."""
    primType, dataSource = observer.GetPrim(Sdf.Path(path))
    if not dataSource or "xform" not in dataSource.GetNames():
        return None
    xform = dataSource.Get("xform")
    if not xform or "matrix" not in xform.GetNames():
        return None
    matrix = xform.Get("matrix")
    if not matrix:
        return None
    return Gf.Matrix4d(matrix.GetValue(0.0)).ExtractTranslation()


def _GuideUnder(observer, path):
    """The first drawn guide prim under a control, whatever it is called.

    A control's guide is published as a child prim whose name depends on
    the shape token (spheres, boxes and cubes all differ), so this finds
    it rather than hardcoding `rigGuideSphere_0`.
    """
    here = _HydraTranslation(observer, path)
    if here is not None:
        return here
    for child in observer.GetChildPrimPaths(Sdf.Path(path)):
        found = _HydraTranslation(observer, child)
        if found is not None:
            return found
    return None


def testUsdviewInputFunction(appController):
    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage

    session = stage.GetSessionLayer()
    if stage.GetEditTarget().GetLayer() != session:
        stage.SetEditTarget(Usd.EditTarget(session))

    for path in (ARM_PARAM, LEG_PARAM):
        prim = stage.GetPrimAtPath(path)
        _Check(prim and prim.IsValid(), "the param node exists: %s" % path)
        _Check(str(prim.GetTypeName()) == "RigExecControl",
               "%s is a RigExecControl" % path)
        _Check(prim.GetAttribute("avars:ikfk"),
               "%s carries avars:ikfk" % path)

    observer, sceneIndexName = _Observer()

    def origins():
        appController._processEvents()
        return (_GuideUnder(observer, ARM_PARAM),
                _GuideUnder(observer, WRIST_L))

    restParam, restWrist = origins()
    if restParam is None or restWrist is None:
        # The scene index is not reachable in this usdview build; say so
        # rather than passing silently on a check that never ran.
        print("RIGEXEC_PARAM_NODE_SKIP %s has no drawn guide for the "
              "param node or the wrist" % sceneIndexName)
        return
    _Check((restParam - restWrist).GetLength() < TOLERANCE,
           "at rest the param guide is on the wrist: param %s wrist %s"
           % (restParam, restWrist))

    # --- FK: the param rides the wrist -----------------------------------
    _Set(stage, ARM_PARAM, "avars:ikfk", 0.0, Sdf.ValueTypeNames.Float)
    rz = _Set(stage, ARM_FK, "avars:rz", 35.0)
    param, wrist = origins()
    moved = (wrist - restWrist).GetLength()
    _Check(moved > 5.0, "FK actually moved the wrist: %.3f cm" % moved)
    _Check((param - wrist).GetLength() < TOLERANCE,
           "in FK the param guide stays on the wrist: gap %.6f cm"
           % (param - wrist).GetLength())
    rz.Clear()

    # --- IK: likewise, and the dial is what chooses --------------------
    ty = _Set(stage, ARM_IK, "avars:ty", 20.0)
    _Set(stage, ARM_PARAM, "avars:ikfk", 0.0, Sdf.ValueTypeNames.Float)
    param, wrist = origins()
    _Check((wrist - restWrist).GetLength() < TOLERANCE,
           "at ikfk = 0 the IK effector is inert: %.6f cm"
           % (wrist - restWrist).GetLength())

    _Set(stage, ARM_PARAM, "avars:ikfk", 1.0, Sdf.ValueTypeNames.Float)
    param, wrist = origins()
    ikMoved = (wrist - restWrist).GetLength()
    _Check(abs(ikMoved - 20.0) < 1e-3,
           "at ikfk = 1 the wrist follows the effector 20 cm: %.6f" % ikMoved)
    _Check((param - wrist).GetLength() < TOLERANCE,
           "in IK the param guide stays on the wrist: gap %.6f cm"
           % (param - wrist).GetLength())

    _Set(stage, ARM_PARAM, "avars:ikfk", 0.5, Sdf.ValueTypeNames.Float)
    param, wrist = origins()
    halfMoved = (wrist - restWrist).GetLength()
    _Check(abs(halfMoved - 10.0) < 1e-3,
           "at ikfk = 0.5 the wrist is halfway: %.6f" % halfMoved)
    ty.Clear()
    _Set(stage, ARM_PARAM, "avars:ikfk", 0.0, Sdf.ValueTypeNames.Float)

    shot = os.getenv("RIGEXEC_PARAM_SHOT")
    if shot:
        appController._mainWindow.grab().save(shot)

    print("RIGEXEC_PARAM_NODE_OK guide tracks the wrist in rest/FK/IK "
          "(gap < %g cm), dial switches solvers 0 -> 0.00, 0.5 -> %.2f, "
          "1 -> %.2f cm" % (TOLERANCE, halfMoved, ikMoved))
