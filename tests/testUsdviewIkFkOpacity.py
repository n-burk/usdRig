#
# THE IK/FK SWITCH FADES THE INACTIVE CONTROL SET, end to end.
#
# The user's words: the switch "needs to drive the solver which should
# change between IK and FK and change the opacity of the control". The
# solver half is covered by testUsdviewAvarEditor.py. This is the opacity
# half, and it exists because the obvious authoring --
# `guide:displayOpacity.connect = </limb_params.avars:ikfk>` -- compiled,
# was accepted, and changed nothing on screen: UsdAttribute::Get never
# follows a connection, so the imaging bridge drew the local value.
#
# So every assertion here is about the DRAWN opacity, read from the
# terminal Hydra scene index usdview renders from (the displayOpacity
# primvar on each control's synthesized rigGuideCtrl child), never about
# the authored attribute. And the edits are made on a DIFFERENT prim from
# the guides they fade -- the param node carries the dial, the limb
# controls carry the guides -- which is exactly the invalidation a plain
# per-prim read could miss.
#
# Opened on biped_rig_v3.usda, whose arm switches are parked at 0 (FK)
# and leg switches at 1 (IK). The wiring itself is authored into the
# SESSION layer by tools/biped/params.py, the same code the builder and
# the retrofit CLI run; the file is never written.
#
# Set RIGEXEC_IKFK_OPACITY_SHOT=/path.png to save a window grab.
#
import ctypes
import os
import sys

from pxr import Sdf, Usd

RIG = "/Biped/Rig"
GUIDE_CHILD = "rigGuideCtrl"
# The schema default of guide:displayOpacityMin.
FLOOR = 0.15
EPS = 1e-6

# Per limb: the controls that must fade WITH the switch (IK side), AGAINST
# it (FK side), and never (root and the param node). The reverse-foot set
# under leg_l_ik is the reason the IK side is a subtree walk and not a
# name list in params.py; a sample of it is checked here by name.
LIMBS = {
    "arm_l": {
        "ik": ["arm_l_ik", "arm_l_pv"],
        "fk": ["arm_l_fk_shoulder_l_bind", "arm_l_fk_elbow_l_bind",
               "arm_l_fk_wrist_l_bind"],
        "keep": ["arm_l_root", "arm_l_params"],
    },
    "leg_l": {
        "ik": ["leg_l_ik", "leg_l_pv", "bank_l", "heel_l", "toe_l",
               "toeBend_l", "ballRoll_l"],
        "fk": ["leg_l_fk_thigh_l_bind", "leg_l_fk_knee_l_bind",
               "leg_l_fk_ankle_l_bind"],
        "keep": ["leg_l_root", "leg_l_params"],
    },
}


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _LoadDll():
    from rigExecUsdview import ImagingLibraryPath
    dll = ctypes.CDLL(ImagingLibraryPath())
    dll.RigExecImaging_GetGeneration.restype = ctypes.c_longlong
    return dll


def _Observer():
    """A HydraObserver on the app's own TERMINAL scene index."""
    from pxr.Usdviewq._usdviewq import HydraObserver
    names = HydraObserver.GetRegisteredSceneIndexNames()
    _Check(bool(names), "usdview registered scene indices")
    observer = HydraObserver()
    observer.TargetToNamedSceneIndex(names[-1])
    return observer, names[-1]


def _RepoRoot(stage):
    """Where tools/biped is: the helpers export RIG, and the test stage
    lives in the repository root either way."""
    root = os.environ.get("RIG")
    if root and os.path.isdir(os.path.join(root, "tools", "biped")):
        return root
    return os.path.dirname(os.path.abspath(stage.GetRootLayer().realPath))


def _ControlPath(stage, name):
    """The one RigExecControl called `name`, wherever the rig nests it."""
    found = [p.GetPath() for p in stage.Traverse()
             if p.GetName() == name and p.GetTypeName() == "RigExecControl"]
    _Check(len(found) == 1,
           "exactly one control named %s: %s" % (name, found))
    return str(found[0])


def _DrawnOpacity(observer, controlPath):
    """
    The displayOpacity primvar the terminal scene index carries on the
    control's synthesized guide, or None when there is no such guide or
    it carries no opacity.
    """
    primType, dataSource = observer.GetPrim(
        Sdf.Path(controlPath).AppendChild(GUIDE_CHILD))
    if not dataSource or "primvars" not in dataSource.GetNames():
        return None
    primvars = dataSource.Get("primvars")
    if not primvars or "displayOpacity" not in primvars.GetNames():
        return None
    opacity = primvars.Get("displayOpacity")
    if not opacity or "primvarValue" not in opacity.GetNames():
        return None
    value = opacity.Get("primvarValue")
    if not value:
        return None
    held = value.GetValue(0.0)
    return float(held[0])


def _Snapshot(observer, paths):
    return dict((name, _DrawnOpacity(observer, path))
                for name, path in paths.items())


def _Near(a, b):
    return a is not None and abs(a - b) < EPS


def _Expect(observer, paths, names, expected, why):
    for name in names:
        drawn = _DrawnOpacity(observer, paths[name])
        _Check(_Near(drawn, expected),
               "%s draws %s, expected %.4f (%s)" % (name, drawn, expected,
                                                    why))


def testUsdviewInputFunction(appController):
    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    dll = _LoadDll()
    _Check(dll.RigExecImaging_GetGeneration() >= 1,
           "RigExec activated on the biped")

    # Every edit goes to the session layer; the checked-in rig is never
    # touched (and nothing here saves).
    session = stage.GetSessionLayer()
    if stage.GetEditTarget().GetLayer() != session:
        stage.SetEditTarget(Usd.EditTarget(session))
    _Check(stage.GetEditTarget().GetLayer() == session,
           "the edit target is the session layer")

    sys.path.insert(0, os.path.join(_RepoRoot(stage), "tools", "biped"))
    import params

    observer, sceneIndexName = _Observer()
    paths = {}
    for limb in LIMBS.values():
        for names in limb.values():
            for name in names:
                paths[name] = _ControlPath(stage, name)

    # --- 0. before wiring, every guide draws its authored opacity --------
    # (1.0 is the control schema's default and nothing in the file
    # overrides it), and the terminal scene index HAS every guide.
    for name, path in paths.items():
        drawn = _DrawnOpacity(observer, path)
        _Check(drawn is not None,
               "the terminal scene index (%s) draws %s's guide"
               % (sceneIndexName, name))
        _Check(_Near(drawn, 1.0),
               "%s starts at its authored opacity 1.0: %s" % (name, drawn))

    # --- 1. the wiring alone republishes, and fades the parked sets -------
    # arm_l is parked at 0 (FK) in the file, leg_l at 1 (IK).
    generation0 = dll.RigExecImaging_GetGeneration()
    wired = params.wire_ikfk_opacity(stage, verbose=False)
    appController._processEvents()
    generation1 = dll.RigExecImaging_GetGeneration()
    _Check(generation1 > generation0,
           "authoring the connections republished: generation %d -> %d"
           % (generation0, generation1))
    for tag in ("arm_l", "arm_r", "leg_l", "leg_r"):
        _Check(tag in wired, "%s was wired: %s" % (tag, sorted(wired)))
    ikArm, fkArm = wired["arm_l"]
    ikLeg, fkLeg = wired["leg_l"]
    _Check(sorted(ikArm) == sorted(paths[n] for n in LIMBS["arm_l"]["ik"]),
           "the arm's IK side is exactly ik + pv: %s" % ikArm)
    _Check(sorted(fkArm) == sorted(paths[n] for n in LIMBS["arm_l"]["fk"]),
           "the arm's FK side is the nested fk chain: %s" % fkArm)
    for name in LIMBS["leg_l"]["ik"]:
        _Check(paths[name] in ikLeg,
               "%s (reverse foot, under leg_l_ik) is on the leg's IK side"
               % name)
    for name in LIMBS["leg_l"]["fk"]:
        _Check(paths[name] in fkLeg, "%s is on the leg's FK side" % name)
    for tag, limb in LIMBS.items():
        for name in limb["keep"]:
            _Check(paths[name] not in wired[tag][0] and
                   paths[name] not in wired[tag][1],
                   "%s is on neither side" % name)

    dialArm = stage.GetPrimAtPath(paths["arm_l_params"]).GetAttribute(
        "avars:ikfk")
    dialLeg = stage.GetPrimAtPath(paths["leg_l_params"]).GetAttribute(
        "avars:ikfk")
    _Check(dialArm and abs(dialArm.Get()) < EPS,
           "arm_l is parked in FK: ikfk = %s" % dialArm.Get())
    _Check(dialLeg and abs(dialLeg.Get() - 1.0) < EPS,
           "leg_l is parked in IK: ikfk = %s" % dialLeg.Get())
    ikOpacity = stage.GetPrimAtPath(paths["arm_l_ik"]).GetAttribute(
        "guide:displayOpacity")
    _Check(ikOpacity.GetConnections() == [dialArm.GetPath()],
           "arm_l_ik's opacity is connected to the dial: %s"
           % ikOpacity.GetConnections())
    _Check(abs(ikOpacity.Get() - 1.0) < EPS,
           "...and its LOCAL value is still 1.0 (Get does not follow the "
           "connection): %s" % ikOpacity.Get())

    _Expect(observer, paths, LIMBS["arm_l"]["ik"], FLOOR,
            "arm in FK: the IK controls are parked at the floor")
    _Expect(observer, paths, LIMBS["arm_l"]["fk"], 1.0,
            "arm in FK: the FK controls are fully drawn")
    _Expect(observer, paths, LIMBS["leg_l"]["ik"], 1.0,
            "leg in IK: the IK controls, reverse foot included, are full")
    _Expect(observer, paths, LIMBS["leg_l"]["fk"], FLOOR,
            "leg in IK: the FK controls are parked at the floor")
    for limb in LIMBS.values():
        _Expect(observer, paths, limb["keep"], 1.0,
                "the root and the param node never fade")

    # --- 2. driving the dial on the PARAM prim refades the guides --------
    # The edit lands on arm_l_params; the guides that change hang off six
    # other prims. Each step must republish and the numbers must be the
    # dial's value on one side and its complement, floored, on the other.
    steps = ((1.0, 1.0, FLOOR), (0.25, 0.25, 0.75), (0.5, 0.5, 0.5),
             (0.9, 0.9, FLOOR), (0.0, FLOOR, 1.0))
    trace = []
    for value, ikExpected, fkExpected in steps:
        before = dll.RigExecImaging_GetGeneration()
        dialArm.Set(value)
        appController._processEvents()
        after = dll.RigExecImaging_GetGeneration()
        _Check(after > before,
               "ikfk = %g republished: generation %d -> %d"
               % (value, before, after))
        _Expect(observer, paths, LIMBS["arm_l"]["ik"], ikExpected,
                "ikfk = %g" % value)
        _Expect(observer, paths, LIMBS["arm_l"]["fk"], fkExpected,
                "ikfk = %g, inverted" % value)
        _Expect(observer, paths, LIMBS["arm_l"]["keep"], 1.0,
                "ikfk = %g leaves the root and param node alone" % value)
        trace.append((value, _DrawnOpacity(observer, paths["arm_l_ik"]),
                      _DrawnOpacity(observer,
                                    paths["arm_l_fk_shoulder_l_bind"])))
    # The other arm did not move: its own dial is untouched.
    _Check(_Near(_DrawnOpacity(observer, _ControlPath(stage, "arm_r_ik")),
                 FLOOR),
           "arm_r's IK control is still parked; its dial was not driven")

    # --- 3. the leg, the other way, reverse foot included -----------------
    dialLeg.Set(0.0)
    appController._processEvents()
    _Expect(observer, paths, LIMBS["leg_l"]["ik"], FLOOR,
            "leg switched to FK: the whole IK set, reverse foot included")
    _Expect(observer, paths, LIMBS["leg_l"]["fk"], 1.0,
            "leg switched to FK: the FK chain is full")
    dialLeg.Set(1.0)
    appController._processEvents()
    _Expect(observer, paths, LIMBS["leg_l"]["ik"], 1.0, "leg back in IK")
    _Expect(observer, paths, LIMBS["leg_l"]["fk"], FLOOR, "leg back in IK")

    # --- 4. the floor is authorable, and zero is a true fade-out ---------
    armIk = stage.GetPrimAtPath(paths["arm_l_ik"])
    floorAttr = armIk.GetAttribute("guide:displayOpacityMin")
    _Check(floorAttr, "the schema declares guide:displayOpacityMin")
    _Check(abs(floorAttr.Get() - FLOOR) < EPS,
           "its fallback is %.2f: %s" % (FLOOR, floorAttr.Get()))
    floorAttr.Set(0.0)
    appController._processEvents()
    _Check(_Near(_DrawnOpacity(observer, paths["arm_l_ik"]), 0.0),
           "with the floor at 0 the parked IK control fades out entirely: %s"
           % _DrawnOpacity(observer, paths["arm_l_ik"]))
    _Check(_Near(_DrawnOpacity(observer, paths["arm_l_pv"]), FLOOR),
           "...and only that control; the pole vector keeps the default "
           "floor: %s" % _DrawnOpacity(observer, paths["arm_l_pv"]))
    floorAttr.Set(0.4)
    appController._processEvents()
    _Check(_Near(_DrawnOpacity(observer, paths["arm_l_ik"]), 0.4),
           "an authored floor of 0.4 is what draws: %s"
           % _DrawnOpacity(observer, paths["arm_l_ik"]))
    floorAttr.Clear()
    appController._processEvents()
    _Check(_Near(_DrawnOpacity(observer, paths["arm_l_ik"]), FLOOR),
           "cleared, the default floor is back: %s"
           % _DrawnOpacity(observer, paths["arm_l_ik"]))

    # --- 5. disconnecting restores the local value, floor ignored ---------
    fkShoulder = stage.GetPrimAtPath(paths["arm_l_fk_shoulder_l_bind"])
    fkOpacity = fkShoulder.GetAttribute("guide:displayOpacity")
    fkOpacity.ClearConnections()
    fkOpacity.Set(0.05)
    appController._processEvents()
    _Check(_Near(_DrawnOpacity(observer, paths["arm_l_fk_shoulder_l_bind"]),
                 0.05),
           "an unconnected opacity draws exactly what it says, invert and "
           "floor ignored: %s"
           % _DrawnOpacity(observer, paths["arm_l_fk_shoulder_l_bind"]))
    _Check(_Near(_DrawnOpacity(observer, paths["arm_l_fk_elbow_l_bind"]),
                 1.0),
           "its nested child is still wired and still full")

    shot = os.environ.get("RIGEXEC_IKFK_OPACITY_SHOT")
    if shot:
        appController._mainWindow.grab().save(shot)

    print("RIGEXEC_IKFK_OPACITY_OK wired %d limbs in the session layer; "
          "arm_l ikfk -> (ik, fk) drawn: %s; leg_l reverse foot follows; "
          "floor %.2f authorable; generation %d -> %d; si=%s"
          % (len(wired),
             ", ".join("%g -> (%.2f, %.2f)" % t for t in trace),
             FLOOR, generation0, dll.RigExecImaging_GetGeneration(),
             sceneIndexName))
