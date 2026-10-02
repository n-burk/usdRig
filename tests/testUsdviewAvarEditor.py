#
# THE AVAR EDITOR, end to end.
#
# tests/python/test_avar_editor_model.py covers every rule in
# avarEditorModel against an in-memory stage and never touches a
# widget. What it cannot see is the half this script exists for: that
# the container registers the menu item in the RigExec menu (and not
# the Window menu), that the panel follows usdview's own selection,
# and that driving a row's spin box or slider through the real Qt
# signals moves the EVALUATED rig by the amount the maths says --
# checked against the native evaluator, against the imaging library's
# generation counter (the viewport's republish), and against the
# terminal Hydra scene index the viewport draws from.
#
# Opened on examples/biped/Biped_anim_ikfk.usda: 254 joints, an IK/FK blend
# per limb that reads a CUSTOM `avars:ikfk` float, and a file-animated
# ikfk on the left arm that the animated-channel policy has to keep
# intact. Every edit lands in the SESSION layer; the file is never
# written.
#
# Set RIGEXEC_AVARS_SHOT=/path.png to save a window grab.
#
import ctypes
import math
import os

from pxr import Gf, Sdf, Usd

# The DELIVERED paths. The FK arm hangs off the body in namespace -- the
# scratch stage this was written against had it flat under Controls -- and
# the IK handles and the per-limb ikfk dials sit at the top.
RIG = "/Biped/Rig"
CONTROLS = RIG + "/Main/Shot/Aux/Controls"
FK_ARM = CONTROLS + "/M_Body/M_Torso/M_Chest/M_ChestTop/L_Shldr/L_UpArmSwing"
FK_SHOULDER = FK_ARM + "/L_UpArm"
ARM_L_ROOT = CONTROLS + "/L_Arm"
ARM_R_ROOT = CONTROLS + "/R_Arm"
ARM_R_IK = CONTROLS + "/R_ArmIK"
ARM_L_IK = CONTROLS + "/L_ArmIK"
SPINE = ("/Biped/Rig/Main/Shot/Aux/Joints/hips_def/spine_0_def/spine_1_def/"
         "spine_2_def/spine_3_def/spine_4_def/spine_5_def/chest_def")
SHOULDER_L = SPINE + "/clavicle_l_def/shoulder_l_def"
ELBOW_L = SHOULDER_L + "/elbow_l_def"
WRIST_R = SPINE + "/clavicle_r_def/shoulder_r_def/elbow_r_def/wrist_r_def"

# The FK shoulder control's pivot is the shoulder joint; a 30 degree
# rotation about its local Z carries the elbow, 25.5068 cm away, along a
# chord of 2 r sin(15 deg).
ELBOW_RADIUS = 25.5068
ROTATE_DEGREES = 30.0
EXPECTED_CHORD = 2.0 * ELBOW_RADIUS * math.sin(math.radians(
    ROTATE_DEGREES / 2.0))
IK_TRANSLATE = -20.0


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


def _HydraTranslation(observer, path):
    """
    Where the terminal scene index puts the prim at `path`: the
    translation of its xform matrix, or None when there is no such
    prim or it carries no xform.
    """
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


class _Evaluator(object):
    """
    Joint origins from the native evaluator, over usdview's own layers.

    A TWIN stage, not usdview's: the rigexec binding needs a Python-owned
    stage (the `__owner` capsule) and usdview's is a weak wrapper once the
    container has put it in the StageCache. Opening a second stage over
    the same root and session layers reads every edit the panel makes,
    because the layers are what it edits.
    """

    def __init__(self, stage):
        import rigexec
        self.stage = Usd.Stage.Open(stage.GetRootLayer(),
                                    stage.GetSessionLayer())
        self.rig = rigexec.Rig(self.stage, RIG)
        self.rig.compile()

    def Origin(self, path, frame):
        pose = self.rig.evaluate(float(frame))
        return Gf.Vec3d(*pose.joint_frame(path, True).origin)


def _RigExecMenu(appController):
    from pxr.Usdviewq.qt import QtWidgets
    menus = {}
    for child in appController._mainWindow.menuBar().children():
        if isinstance(child, QtWidgets.QMenu):
            menus[str(child.title()).replace("&", "")] = child
    return menus


def _Submenu(menu, title):
    for action in menu.actions():
        if action.menu() is not None and action.text() == title:
            return action.menu()
    return None


def _Select(appController, path, *more):
    stage = appController._usdviewApi.stage
    prim = stage.GetPrimAtPath(path)
    _Check(prim, "the biped has %s" % path)
    # setPrim, not ClearPrimSelection + AddPrimToSelection: clearing
    # leaves the pseudo-root focused (see layer opinions test).
    selection = appController._dataModel.selection
    selection.setPrim(prim)
    for extra in more:
        selection.addPrim(stage.GetPrimAtPath(extra))
    appController._processEvents()
    return prim


def _Scrub(appController, row, pixels, modifiers=None):
    """A horizontal drag across a row's field, as Qt delivers one.

    Press, a few moves past the slop, release -- the same event stream a
    mouse produces, without depending on the widget's width or on the
    offscreen platform delivering real input.
    """
    from pxr.Usdviewq.qt import QtCore, QtGui
    field = row.field
    _Check(field is not None, "%s has a field" % row.channel.shortName)
    mods = modifiers or QtCore.Qt.NoModifier
    start = QtCore.QPointF(10.0, 8.0)

    def _send(kind, x, buttons):
        point = QtCore.QPointF(x, 8.0)
        event = QtGui.QMouseEvent(kind, point, QtCore.Qt.LeftButton,
                                  buttons, mods)
        appController._processEvents()
        field.event(event)

    _send(QtCore.QEvent.MouseButtonPress, start.x(), QtCore.Qt.LeftButton)
    for i in (1, 2, 3, 4):
        _send(QtCore.QEvent.MouseMove, start.x() + pixels * i / 4.0,
              QtCore.Qt.LeftButton)
    _send(QtCore.QEvent.MouseButtonRelease, start.x() + pixels,
          QtCore.Qt.NoButton)
    appController._processEvents()


def _TypeValue(appController, row, text):
    """Type into a row's field the way a person does.

    The field commits on `editingFinished`, which Qt emits on Enter and
    on focus-out; `setText` alone changes the text without committing,
    exactly as typing without pressing Enter does. So the test sets the
    text and then emits the signal, which is the same path the keystroke
    takes and does not depend on the widget having focus in an offscreen
    session.
    """
    field = row.field
    _Check(field is not None, "%s has a typed field" % row.channel.shortName)
    field.setText(text)
    field.editingFinished.emit()
    appController._processEvents()


def _UndoDepth(stack):
    return len(stack._undo)


def testUsdviewInputFunction(appController):
    import avarEditorUI
    import avarEditorModel as model
    import rigExecUsdview

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    dll = _LoadDll()
    _Check(dll.RigExecImaging_GetGeneration() >= 1,
           "RigExec activated on the biped")

    # Every edit goes to the session layer; the checked-in file is never
    # touched (and nothing here saves).
    session = stage.GetSessionLayer()
    if stage.GetEditTarget().GetLayer() != session:
        stage.SetEditTarget(Usd.EditTarget(session))
    _Check(stage.GetEditTarget().GetLayer() == session,
           "the edit target is the session layer")

    # --- 1. the container registered the menu item ----------------------
    reg = appController._plugRegistry
    _Check(reg.getCommandPlugin(
        "RigExecUsdviewContainer.avarEditor") is not None,
        "RigExec -> Avar Editor command is registered")
    menus = _RigExecMenu(appController)
    _Check("RigExec" in menus, "there is a RigExec menu: %s" % sorted(menus))
    rigMenu = _Submenu(menus["RigExec"], "General Editors")
    _Check(rigMenu is not None, "RigExec has a General Editors submenu")
    _Check("Avar Editor" in [a.text() for a in rigMenu.actions()],
           "RigExec -> General Editors has an Avar Editor item")
    window = menus.get("Window")
    if window is not None:
        _Check("Avar Editor" not in [a.text() for a in window.actions()],
               "the panel is in the RigExec menu ONLY, not Window")

    # --- 2. the MENU path opens it on the selection ----------------------
    # Playback STOPPED first. This stage declares a time range, which
    # gives usdview a live timeline: left playing, the frame advances
    # between the test reading it and the panel reading it, and the two
    # disagree about a number neither of them got wrong. Then frame 1 is
    # asked for and read back, because the timeline has the last word on
    # which frame is current.
    appController._dataModel.playing = False
    appController._processEvents()
    appController._dataModel.currentFrame = Usd.TimeCode(1)
    appController._processEvents()
    frame = appController._dataModel.currentFrame
    _Select(appController, FK_SHOULDER)
    action = [a for a in rigMenu.actions() if a.text() == "Avar Editor"][0]
    action.trigger()
    appController._processEvents()
    panel = avarEditorUI.AvarEditorPanel._instance
    _Check(panel is not None, "triggering the menu item opened the panel")
    _Check(panel.isVisible(), "the panel it opened is visible")
    _Check(panel._header.toolTip() == FK_SHOULDER,
           "it built on the current selection: %r" % panel._header.toolTip())
    container = rigExecUsdview._container
    undo = container._UndoStack()
    _Check(panel._undo is undo,
           "the menu path hands it the container's shared undo stack")
    _Check(panel.Frame() == frame,
           "the panel is on frame %s, the app on %s"
           % (panel.Frame(), frame))
    _Check(panel.WriteMode() == model.WRITE_ANIMATION,
           "Animation is the default write mode, like the gizmo")

    # --- 3. the channels are the schema's, in the stage's units ---------
    names = [row.name for row in panel.Rows()]
    for expected in ("avars:tx", "avars:ty", "avars:tz", "avars:rx",
                     "avars:ry", "avars:rz", "avars:rspin", "avars:sx",
                     "avars:sy", "avars:sz", "avars:rotationOrder",
                     "avars:unitScaleFactor"):
        _Check(expected in names, "%s is listed: %s" % (expected, names))
    _Check("avars:ikfk" not in names,
           "an FK control has no ikfk dial of its own")
    _Check(panel.Row("tx").unit.text() == "cm",
           "translation is labelled in the stage's centimetres: %r"
           % panel.Row("tx").unit.text())
    _Check(panel.Row("rz").unit.text() == "deg", "rotation is in degrees")
    # The spans survive on the model -- they still decide a channel's
    # decimals -- even though no slider draws them any more.
    _Check(panel.Row("rz").SliderRange() == (-180.0, 180.0),
           "rotate spans +-180: %s" % (panel.Row("rz").SliderRange(),))
    _Check(panel.Row("tx").SliderRange() == (-100.0, 100.0),
           "translate spans one metre of centimetres: %s"
           % (panel.Row("tx").SliderRange(),))
    _Check(panel.Row("sx").SliderRange() == (0.0, 2.0),
           "scale sits around 1.0: %s" % (panel.Row("sx").SliderRange(),))
    _Check(panel.Row("sx").field.text() == "1",
           "scale shows its rest value 1.0, got %r"
           % panel.Row("sx").field.text())
    order = panel.Row("rotationOrder")
    _Check(order.combo is not None and order.combo.count() == 6,
           "rotationOrder is a combo over the six allowed tokens")

    # --- 4. a typed rotation moves the evaluated elbow ------------------
    evaluator = _Evaluator(stage)
    observer, sceneIndexName = _Observer()
    shoulder0 = evaluator.Origin(SHOULDER_L, 1)
    elbow0 = evaluator.Origin(ELBOW_L, 1)
    _Check(abs((elbow0 - shoulder0).GetLength() - ELBOW_RADIUS) < 1e-3,
           "the elbow is %.4f from the shoulder at rest"
           % (elbow0 - shoulder0).GetLength())
    hydraElbow0 = _HydraTranslation(observer, ELBOW_L + "/rigGuideSphere_0")
    _Check(hydraElbow0 is not None,
           "the terminal scene index (%s) has the elbow's guide sphere"
           % sceneIndexName)
    generation0 = dll.RigExecImaging_GetGeneration()
    depth0 = _UndoDepth(undo)

    rz = panel.Row("rz")
    rzAttr = stage.GetPrimAtPath(FK_SHOULDER).GetAttribute("avars:rz")
    _Check(session.GetAttributeAtPath(FK_SHOULDER + ".avars:rz") is None,
           "rz starts unauthored in the session layer")
    # setValue on a spin box emits valueChanged exactly as Enter does.
    _TypeValue(appController, rz, str(ROTATE_DEGREES))

    _Check(abs(rzAttr.Get(frame) - ROTATE_DEGREES) < 1e-9,
           "the composed rz at frame 1 is 30: %s" % rzAttr.Get(frame))
    spec = session.GetAttributeAtPath(FK_SHOULDER + ".avars:rz")
    _Check(spec is not None and spec.HasSpline(),
           "Animation mode keyed a spline knot in the session layer")
    _Check(1.0 in list(spec.GetSpline().GetKnots()),
           "the knot is at frame 1: %s" % list(spec.GetSpline().GetKnots()))
    _Check(rz.badge.text() == "key", "the row shows it is keyed here: %r"
           % rz.badge.text())

    shoulder1 = evaluator.Origin(SHOULDER_L, 1)
    elbow1 = evaluator.Origin(ELBOW_L, 1)
    chord = (elbow1 - elbow0).GetLength()
    _Check(abs(chord - EXPECTED_CHORD) < 1e-6,
           "the elbow moved along a %.6f chord (expected %.6f)"
           % (chord, EXPECTED_CHORD))
    _Check((shoulder1 - shoulder0).GetLength() < 1e-9,
           "the shoulder itself stayed put")

    generation1 = dll.RigExecImaging_GetGeneration()
    _Check(generation1 > generation0,
           "the imaging library republished: generation %d -> %d"
           % (generation0, generation1))
    hydraElbow1 = _HydraTranslation(observer, ELBOW_L + "/rigGuideSphere_0")
    hydraChord = (hydraElbow1 - hydraElbow0).GetLength()
    _Check(abs(hydraChord - EXPECTED_CHORD) < 1e-4,
           "the viewport's scene index moved the elbow guide by %.6f"
           % hydraChord)
    _Check(_UndoDepth(undo) == depth0 + 1,
           "the edit is one entry on the shared undo stack")
    _Check(undo.UndoText() == "Set rz", "labelled: %r" % undo.UndoText())

    # --- 5. undo / redo through the shared stack -------------------------
    undo.Undo()
    appController._processEvents()
    _Check(abs(rzAttr.Get(frame)) < 1e-9, "undo put rz back to 0")
    _Check((evaluator.Origin(ELBOW_L, 1) - elbow0).GetLength() < 1e-9,
           "and the elbow back to rest")
    _Check(abs(float(rz.field.text() or 0)) < 1e-9,
           "the row refreshed on the undo notice: %s" % float(rz.field.text() or 0))
    undo.Redo()
    appController._processEvents()
    _Check(abs(rzAttr.Get(frame) - ROTATE_DEGREES) < 1e-9,
           "redo restored 30")

    # --- 6. reset on a keyed channel keys the rest value -----------------
    rz.resetButton.click()
    appController._processEvents()
    _Check(abs(rzAttr.Get(frame)) < 1e-9,
           "reset keyed 0 at frame 1: %s" % rzAttr.Get(frame))
    _Check((evaluator.Origin(ELBOW_L, 1) - elbow0).GetLength() < 1e-9,
           "the elbow is back at rest")
    _Check(undo.UndoText() == "Reset rz", "reset is its own undo entry")

    # --- 7. the custom ikfk dial drives the blend, from a slider --------
    _Select(appController, ARM_R_ROOT)
    _Check(panel._header.toolTip() == ARM_R_ROOT,
           "the panel followed the selection")
    ikfk = panel.Row("ikfk")
    _Check(ikfk is not None, "the custom avars:ikfk is discovered by prefix")
    _Check(ikfk.channel.kind == model.KIND_CUSTOM and ikfk.field is not None,
           "and it is a custom float with a typed field")
    _Check(ikfk.SliderRange() == (0.0, 1.0),
           "a bare float avar still reports a 0..1 span: %s"
           % (ikfk.SliderRange(),))
    _Check(ikfk.field.text() in ("0", "0.0"),
           "it starts at 0 = FK, got %r" % ikfk.field.text())
    ikfkAttr = stage.GetPrimAtPath(ARM_R_ROOT).GetAttribute("avars:ikfk")

    wrist0 = evaluator.Origin(WRIST_R, 1)
    depth1 = _UndoDepth(undo)
    _TypeValue(appController, ikfk, "1")
    _Check(abs(ikfkAttr.Get(frame) - 1.0) < 1e-9,
           "typing wrote ikfk = 1 (IK): %s" % ikfkAttr.Get(frame))
    _Check(_UndoDepth(undo) == depth1 + 1,
           "one typed value is ONE undo entry")
    _Check(undo.UndoText() == "Set ikfk", "labelled: %r" % undo.UndoText())

    # From the pose the switch LEFT, not from the FK one: switching a limb
    # to IK moves its end joint by whatever the two solutions disagree
    # about at rest (3.32 cm on this rig's right arm), and measuring the
    # handle's travel from before the switch adds that in.
    wristAtIK = evaluator.Origin(WRIST_R, 1)
    _Select(appController, ARM_R_IK)
    _TypeValue(appController, panel.Row("ty"), str(IK_TRANSLATE))
    wristIK = evaluator.Origin(WRIST_R, 1)
    ikMoved = (wristIK - wristAtIK).GetLength()
    _Check(abs(ikMoved - abs(IK_TRANSLATE)) < 1e-6,
           "in IK the wrist followed the IK control by %.6f cm" % ikMoved)

    _Select(appController, ARM_R_ROOT)
    _TypeValue(appController, panel.Row("ikfk"), str(0.0))
    wristFK = evaluator.Origin(WRIST_R, 1)
    fkMoved = (wristFK - wrist0).GetLength()
    _Check(fkMoved < 1e-9,
           "back in FK the same IK offset moves the wrist %.6f (nothing)"
           % fkMoved)
    _Select(appController, ARM_R_IK)
    panel.Row("ty").resetButton.click()
    appController._processEvents()

    # --- 8. an animated channel keeps its other keys ---------------------
    # L_UpArmSwing.avars:ikfk is keyed IN THE FILE at 1:0, 24:0, 48:1. A
    # session-layer sample at frame 36 alone would outrank all three and
    # hold 0.25 across the whole shot; the panel promotes the file's keys
    # into the session layer first, so only frame 36 changes.
    animAttr = stage.GetPrimAtPath(ARM_L_ROOT).GetAttribute("avars:ikfk")
    _Check(animAttr.GetNumTimeSamples() == 3, "the fixture is as authored")
    _Check(abs(animAttr.Get(Usd.TimeCode(36)) - 0.5) < 1e-9,
           "frame 36 interpolates to 0.5")
    appController._dataModel.currentFrame = Usd.TimeCode(36)
    appController._processEvents()
    _Select(appController, ARM_L_ROOT)
    animRow = panel.Row("ikfk")
    _Check(animRow.badge.text() == "anim",
           "the row says the channel is animated: %r" % animRow.badge.text())
    _Check(abs(float(animRow.field.text() or 0) - 0.5) < 1e-9,
           "and shows the interpolated 0.5 at frame 36")
    _TypeValue(appController, animRow, str(0.25))
    values = [animAttr.Get(Usd.TimeCode(t)) for t in (1, 24, 36, 48)]
    _Check(abs(values[2] - 0.25) < 1e-9, "frame 36 is now 0.25: %s" % values)
    _Check(abs(values[0]) < 1e-9 and abs(values[1]) < 1e-9
           and abs(values[3] - 1.0) < 1e-9,
           "frames 1, 24 and 48 kept their file keys: %s" % values)
    _Check(animRow.badge.text() == "key", "frame 36 is keyed now")
    sessionSamples = session.ListTimeSamplesForPath(
        Sdf.Path(ARM_L_ROOT + ".avars:ikfk"))
    _Check(sorted(sessionSamples) == [1.0, 24.0, 36.0, 48.0],
           "the session layer holds the promoted keys plus the new one: %s"
           % sessionSamples)
    undo.Undo()
    appController._processEvents()
    _Check(session.GetAttributeAtPath(ARM_L_ROOT + ".avars:ikfk") is None,
           "undo removed the whole session spec, promotion included")
    _Check(abs(animAttr.Get(Usd.TimeCode(36)) - 0.5) < 1e-9,
           "and frame 36 reads 0.5 from the file again")

    # Default mode on that channel does NOT stomp silently. It does
    # stomp -- a session-layer default is a stronger opinion than the
    # file's time samples, so the whole shot reads 0.75 -- and the
    # status line says exactly that, and one Ctrl+Z takes it back.
    panel.SetWriteMode(model.WRITE_DEFAULT)
    _TypeValue(appController, animRow, str(0.75))
    _Check("HIDES" in panel._status.text(),
           "Default mode warned that the default hides the keys: %r"
           % panel._status.text())
    _Check(abs(animAttr.Get(Usd.TimeCode(36)) - 0.75) < 1e-9
           and abs(animAttr.Get(Usd.TimeCode(48)) - 0.75) < 1e-9,
           "the session default won at every frame: %s"
           % [animAttr.Get(Usd.TimeCode(t)) for t in (36, 48)])
    undo.Undo()
    panel.SetWriteMode(model.WRITE_ANIMATION)
    appController._processEvents()
    _Check(abs(animAttr.Get(Usd.TimeCode(36)) - 0.5) < 1e-9,
           "undo let the file's curve through again")

    # --- 9. reset all, as one entry ---------------------------------------
    appController._dataModel.currentFrame = frame
    appController._processEvents()
    _Select(appController, ARM_L_IK)
    _TypeValue(appController, panel.Row("tx"), str(3.0))
    _TypeValue(appController, panel.Row("ry"), str(12.0))
    ikPrim = stage.GetPrimAtPath(ARM_L_IK)
    _Check(abs(ikPrim.GetAttribute("avars:tx").Get(frame) - 3.0) < 1e-9
           and abs(ikPrim.GetAttribute("avars:ry").Get(frame) - 12.0) < 1e-9,
           "two channels were set")
    depth2 = _UndoDepth(undo)
    panel._resetAll.click()
    appController._processEvents()
    _Check(abs(ikPrim.GetAttribute("avars:tx").Get(frame)) < 1e-9
           and abs(ikPrim.GetAttribute("avars:ry").Get(frame)) < 1e-9,
           "Reset All returned both to rest")
    _Check(_UndoDepth(undo) == depth2 + 1, "Reset All is one undo entry")

    # --- 10. multi-selection shows the LAST picked and edits together ---
    #
    # The panel follows the tail of the selection, not usdview's focus
    # prim: focus stays put on a shift-add, so following it showed the
    # first control of a group while the hand was on the second.
    _Select(appController, ARM_L_IK, ARM_R_IK)
    _Check(panel._header.toolTip() == ARM_R_IK,
           "the LAST selected is shown: %r" % panel._header.toolTip())
    _Check("1 more selected" in panel._note.text(),
           "and the panel says so: %r" % panel._note.text())
    _Check("editing 2 prims together" in panel._status.text(),
           "the status says what an edit will reach: %r"
           % panel._status.text())

    # An edit reaches BOTH, in one undo entry, because both carry tx.
    leftIk = stage.GetPrimAtPath(ARM_L_IK)
    rightIk = stage.GetPrimAtPath(ARM_R_IK)
    depth3 = _UndoDepth(undo)
    _TypeValue(appController, panel.Row("tx"), "4")
    _Check(abs(leftIk.GetAttribute("avars:tx").Get(frame) - 4.0) < 1e-9,
           "the other selected prim moved too: %s"
           % leftIk.GetAttribute("avars:tx").Get(frame))
    _Check(abs(rightIk.GetAttribute("avars:tx").Get(frame) - 4.0) < 1e-9,
           "and so did the shown one")
    _Check(_UndoDepth(undo) == depth3 + 1,
           "the pair is ONE undo entry, not one per prim")
    undo.Undo()
    appController._processEvents()
    _Check(abs(leftIk.GetAttribute("avars:tx").Get(frame)) < 1e-9
           and abs(rightIk.GetAttribute("avars:tx").Get(frame)) < 1e-9,
           "and one Ctrl+Z puts both back")

    # --- 10b. dragging in a field scrubs it, at three speeds ------------
    from pxr.Usdviewq.qt import QtCore
    _Select(appController, ARM_R_IK)
    tx = panel.Row("tx")
    ikPrim = stage.GetPrimAtPath(ARM_R_IK)
    attr = ikPrim.GetAttribute("avars:tx")

    _TypeValue(appController, tx, "0")
    depth = _UndoDepth(undo)
    _Scrub(appController, tx, 40)
    plain = attr.Get(frame)
    _Check(abs(plain) > 1e-6, "a drag moved the value: %s" % plain)
    _Check(_UndoDepth(undo) == depth + 1,
           "the whole drag is ONE undo entry, not one per sample")

    _TypeValue(appController, tx, "0")
    _Scrub(appController, tx, 40, QtCore.Qt.ControlModifier)
    slow = attr.Get(frame)
    _TypeValue(appController, tx, "0")
    _Scrub(appController, tx, 40,
           QtCore.Qt.ControlModifier | QtCore.Qt.ShiftModifier)
    fast = attr.Get(frame)
    _Check(abs(slow) < abs(plain) < abs(fast),
           "Ctrl is finer and Ctrl+Shift coarser: %.4f < %.4f < %.4f"
           % (abs(slow), abs(plain), abs(fast)))
    _Check(abs(abs(slow) * 10.0 - abs(plain)) < 1e-4,
           "Ctrl is exactly ten times finer: %s vs %s" % (slow, plain))
    _Check(abs(abs(fast) - abs(plain) * 10.0) < 1e-3,
           "Ctrl+Shift is exactly ten times coarser: %s vs %s"
           % (fast, plain))
    _TypeValue(appController, tx, "0")

    # --- 11. a joint's avars are editable too ----------------------------
    _Select(appController, ELBOW_L)
    _Check(panel._header.toolTip() == ELBOW_L and panel.Row("rz") is not None,
           "a RigExecJoint lists its avars: %s"
           % [r.name for r in panel.Rows()])

    shot = os.environ.get("RIGEXEC_AVARS_SHOT")
    if shot:
        _Select(appController, ARM_R_ROOT)
        appController._processEvents()
        panel.grab().save(shot)

    print("RIGEXEC_AVAR_EDITOR_OK menu, selection, rz=%g -> elbow chord "
          "%.6f (hydra %.6f), ikfk slider, animated keys kept, reset all, "
          "generation %d -> %d, si=%s"
          % (ROTATE_DEGREES, chord, hydraChord, generation0, generation1,
             sceneIndexName))
