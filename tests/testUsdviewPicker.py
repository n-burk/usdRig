#
# THE CONTROL PICKER, end to end.
#
# The point of the panel is that clicking a button selects the control in
# usdview's OWN data model -- which is what makes the Avar Editor and the
# viewport gizmo follow it without the picker knowing anything about avars.
# So that is what this asserts: a click through the real Qt signal changes
# `dataModel.selection`, and a selection made elsewhere lights the matching
# button.
#
# It also pins the two facts that would silently rot:
#   * the picker is read from SCENE DATA -- RigExecPicker prims in the
#     stage's layer stack, found by type -- with one outer tab per
#     character and the panels nested inside it;
#   * a stage WITHOUT the picker layer has no picker at all -- there is
#     no sidecar file and no fallback to one;
#   * every "live" button's targets really are RigExecControl prims on this
#     stage, so liveness reflects the rig rather than a stale bake.
#
import os

from pxr import Sdf, Usd

RIG = "/Biped/Rig"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets


def _RigExecMenu(appController):
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


def _Trigger(menu, title):
    for action in menu.actions():
        if action.text() == title:
            action.trigger()
            return True
    return False


def testUsdviewInputFunction(appController):
    import pickerUI
    import pickerModel
    import pickerScene

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage

    # --- 1. registered, in the RigExec menu only ------------------------
    reg = appController._plugRegistry
    _Check(reg.getCommandPlugin("RigExecUsdviewContainer.picker") is not None,
           "RigExec -> Control Picker is registered")
    menus = _RigExecMenu(appController)
    _Check("RigExec" in menus, "there is a RigExec menu: %s" % sorted(menus))
    rigMenu = _Submenu(menus["RigExec"], "Animation Editors")
    _Check(rigMenu is not None, "RigExec has an Animation Editors submenu")
    _Check("Control Picker" in [a.text() for a in rigMenu.actions()],
           "RigExec -> Animation Editors carries it")
    window = menus.get("Window")
    if window is not None:
        _Check("Control Picker" not in [a.text() for a in window.actions()],
               "and the Window menu does not")

    # --- 2. the picker is SCENE DATA, found by type ---------------------
    prims = pickerScene.find(stage)
    _Check(prims, "the stage carries at least one RigExecPicker prim")
    for prim in prims:
        _Check(str(prim.GetTypeName()) == "RigExecPicker",
               "%s is a RigExecPicker" % prim.GetPath())

    # NO SIDECAR, and no fallback to one. A stage without the picker
    # layer simply has no picker, which is what makes the layer the
    # single place the picker comes from.
    _Check(not hasattr(pickerModel, "load_for"),
           "the JSON loader is gone from pickerModel")
    other = Usd.Stage.Open(os.path.join(os.path.dirname(
        stage.GetRootLayer().realPath), "Biped.usda"))
    _Check(not pickerScene.find(other),
           "a stage without the picker layer carries no picker")

    _Check(_Trigger(rigMenu, "Control Picker"), "the menu item triggered")
    appController._processEvents()
    panel = pickerUI.PickerPanel.GetInstance(api)
    _Check(panel is not None, "the panel was created")
    picker = panel._picker
    _Check(picker is not None, "the panel loaded a picker")

    live, dead, deco = picker.coverage()
    _Check(live > 20, "a useful number of buttons are live, got %d" % live)
    _Check(dead == 0,
           "no button is drawn that cannot be clicked, got %d" % dead)

    # TABS NEST: one outer tab per character, its panels inside. The
    # Facial panel is skipped while the face rig is unported, so the
    # inner count is "the panels we build views for", not len(panels).
    _Check(panel._tabs.count() == len(panel._pickers),
           "one outer tab per character: %d vs %d"
           % (panel._tabs.count(), len(panel._pickers)))
    inner = panel._tabs.widget(0)
    _Check(isinstance(inner, QtWidgets.QTabWidget),
           "the character tab holds a tab widget, got %r" % type(inner))
    _Check(inner.count() == len(panel._views),
           "one inner tab per built panel: %d vs %d"
           % (inner.count(), len(panel._views)))
    _Check(panel._tabs.tabText(0) == picker.name,
           "the outer tab is named for the character: %r vs %r"
           % (panel._tabs.tabText(0), picker.name))

    # --- 3. every live button really points at a control on this stage --
    for button in picker.buttons:
        if not button.live:
            continue
        for target in button.targets:
            prim = stage.GetPrimAtPath(target)
            _Check(prim and prim.IsValid(),
                   "live button %s targets a real prim: %s"
                   % (button.id, target))
            _Check(str(prim.GetTypeName())
                   in ("RigExecControl", "RigExecJoint"),
                   "%s is drivable (control or joint), got %s"
                   % (target, prim.GetTypeName()))

    # --- 4. a CLICK selects, through usdview's data model ---------------
    body = next((p for p in picker.panels if p.label == "Body"),
                picker.panels[0])
    view = next(v for v in panel._views if v._panel.id == body.id)
    target = next(b for b in picker.visible(body.id)
                  if b.live and len(b.targets) == 1)

    selection = appController._dataModel.selection
    selection.clearPrims()
    appController._processEvents()

    view.picked.emit([target], "replace")
    appController._processEvents()
    chosen = [str(p.GetPath()) for p in selection.getPrims()
              if p and p.IsValid()]
    _Check(chosen == target.targets,
           "the click selected %s, got %s" % (target.targets, chosen))

    # ...and Shift adds rather than replaces.
    second = next(b for b in picker.visible(body.id)
                  if b.live and len(b.targets) == 1
                  and b.targets != target.targets)
    view.picked.emit([second], "toggle")
    appController._processEvents()
    chosen = set(str(p.GetPath()) for p in selection.getPrims()
                 if p and p.IsValid())
    _Check(chosen == set(target.targets) | set(second.targets),
           "shift-click TOGGLED the second one in: expected %s, got %s"
           % (set(target.targets) | set(second.targets), chosen))

    # ...and shift on something already selected takes it back out, which
    # is the half that makes the gesture usable for trimming.
    view.picked.emit([second], "toggle")
    appController._processEvents()
    chosen = set(str(p.GetPath()) for p in selection.getPrims()
                 if p and p.IsValid())
    _Check(chosen == set(target.targets),
           "shift again removed it: expected %s, got %s"
           % (set(target.targets), chosen))

    # Ctrl only ever subtracts.
    view.picked.emit([target], "remove")
    appController._processEvents()
    # `clearPrims` leaves the PSEUDO-ROOT selected, not nothing -- so
    # "empty" here means "no real prim", which is what the animator sees.
    left = [str(p.GetPath()) for p in selection.getPrims()
            if p and p.IsValid() and not p.IsPseudoRoot()]
    _Check(not left, "ctrl-click removed the last one, got %s" % left)
    view.picked.emit([target, second], "replace")
    appController._processEvents()

    # A marquee is the same path: buttons whose box the band touches.
    band = picker.within(body.id, target.x - 1, target.y - 1,
                         target.x + target.w + 1, target.y + target.h + 1)
    _Check(any(b.id == target.id for b in band),
           "the marquee catches a button it overlaps (%d found)" % len(band))

    # --- 5. selecting elsewhere lights the button ----------------------
    selection.clearPrims()
    selection.setPrim(stage.GetPrimAtPath(target.targets[0]))
    appController._processEvents()
    panel._OnSelectionChanged()
    _Check(view._selected == set(target.targets),
           "the view mirrors the selection: %s" % view._selected)
    lit = picker.buttons_for(target.targets)
    _Check(any(b.id == target.id for b in lit),
           "and the matching button is among those lit (%d)" % len(lit))

    selection.clearPrims()
    appController._processEvents()

    # --- 6. the IK/FK buttons actually switch ---------------------------
    # Attribute buttons are no longer only the limb switches: the space
    # switches are bound the same way (twelve of them, on spaces:active),
    # so the four this section is about are picked out by the channel
    # they drive rather than by being all there is.
    bound = [b for b in picker.buttons if b.attr_target]
    switches = [b for b in bound
                if b.attr_target["attr"] == "avars:ikfk"]
    _Check(len(switches) == 4,
           "all four limb IK/FK switches are bound, got %d of %d attribute "
           "buttons" % (len(switches), len(bound)))
    # avars:space is the eye look-at's weight. It was called face:lookAt
    # until it was renamed to the name every other control uses for the
    # same kind of channel, and this list was not moved with it -- which
    # nothing caught, because this test runs from bin/run_testusdview_
    # picker and is not one of the 198 in ctest.
    _Check(all(b.attr_target["attr"] in ("avars:ikfk", "avars:space",
                                         "spaces:active")
               for b in bound),
           "every attribute button drives a channel this test knows: %s"
           % sorted(set(b.attr_target["attr"] for b in bound)))
    switch = switches[0]
    target = switch.attr_target
    prim = stage.GetPrimAtPath(target["path"])
    _Check(prim and prim.IsValid(),
           "the switch points at a real control: %s" % target["path"])
    attr = prim.GetAttribute(target["attr"])
    _Check(attr and attr.IsValid(),
           "%s carries %s" % (target["path"], target["attr"]))

    before = float(attr.Get())
    view.picked.emit([switch], "replace")
    appController._processEvents()
    after = float(attr.Get())
    _Check(after != before,
           "clicking the switch changed %s: %g -> %g"
           % (target["attr"], before, after))
    _Check(after in (0.0, 1.0), "and wrote a blend weight, got %g" % after)

    # the conventional enum is "IK,FK" (0 = IK) while ours is the blend weight
    # (0 = FK). If that inversion is wrong the label reads backwards, so
    # assert the label and the number agree.
    _Check((switch.value == "IK") == (after == 1.0),
           "label %r agrees with weight %g" % (switch.value, after))

    view.picked.emit([switch], "replace")
    appController._processEvents()
    _Check(float(attr.Get()) == before,
           "clicking again cycled back to %g" % before)






    drawn = picker.visible(body.id)
    inert = [b for b in drawn if not b.live and not b.decoration]
    _Check(not inert,
           "nothing drawn is unpickable, got %d of %d"
           % (len(inert), len(drawn)))
    _Check(all(b.live or b.decoration for b in drawn),
           "every drawn button is either live or a decoration")

    # And an override that KILLS a control takes its button with it.
    victim = next(b for b in drawn if b.live and not b.decoration)
    session = Usd.EditTarget(stage.GetSessionLayer())
    with Usd.EditContext(stage, session):
        for target in victim.targets:
            stage.OverridePrim(Sdf.Path(target)).SetActive(False)
    again = pickerScene.load_all(stage)[0]
    still = [b for b in again.visible(body.id) if b.id == victim.id]
    _Check(not still,
           "deactivating the control removed its button: %s" % victim.id)
    stage.GetSessionLayer().Clear()

    modes = panel._RefreshModes() or {}
    _Check(modes, "the panel read the limb dials, got %s" % modes)
    with_modes = picker.visible(body.id, modes=modes)
    _Check(len(with_modes) < len(drawn),
           "applying the modes removes the inactive half: %d vs %d"
           % (len(with_modes), len(drawn)))

    # Flip every arm to IK and the drawn set must change again -- this is
    # the thing the user asked for, so assert the effect not the plumbing.
    flipped = dict(modes)
    for path in flipped:
        # CASE-INSENSITIVE. The dials are on the delivered control names
        # (L_Arm, R_Arm), not the build names (arm_l), so a lowercase
        # "arm" matched nothing and the flip was a no-op comparing a set
        # against itself.
        if "arm" in path.lower():
            flipped[path] = "ik" if flipped[path] == "fk" else "fk"
    after = picker.visible(body.id, modes=flipped)
    _Check(len(after) != len(with_modes),
           "switching the arms changes what is drawn: %d -> %d"
           % (len(with_modes), len(after)))

    # ...and the panel must notice a dial moved from OUTSIDE it -- the
    # Avar Editor, a gizmo on `avars:ikfk`, an undo, a scrub. It used to
    # refresh only on its own switch click, so measured on Biped_stack,
    # setting L_Arm back to 0 (FK) from outside still drew L_ArmIK and
    # L_ArmPV and still hid L_UpArm/L_LoArm/L_Hand until the panel was
    # reopened. That is the "the IK controls are always on" report.
    #
    # L_Arm, the DELIVERED name. This said "arm_l", the build name, and
    # matched nothing once the controls were renamed -- the generator
    # raised StopIteration rather than failing an assertion, which is
    # why it read as a crash and not as a stale test.
    dial = next(p for p in modes if "L_Arm" in p)
    was = modes[dial]
    attr = stage.GetAttributeAtPath(Sdf.Path(dial))
    knob = next(b for b in picker.buttons if b.attr_target
                and dial.startswith(b.attr_target["path"] + "."))
    label = knob.value
    attr.Set(1.0 if was == "fk" else 0.0)
    appController._processEvents()
    _Check(view._modes.get(dial) != was,
           "an outside edit to %s reached the picker: still %r"
           % (dial, view._modes.get(dial)))
    # The label is BAKED at export, so it has to be read back off the
    # attribute too or the panel contradicts the half it is drawing.
    _Check(knob.value != label,
           "and the switch label followed the rig: %r -> %r"
           % (label, knob.value))
    attr.Set(0.0 if was == "fk" else 1.0)
    appController._processEvents()
    _Check(view._modes.get(dial) == was and knob.value == label,
           "and back again: %r / %r" % (view._modes.get(dial), knob.value))

    # --- 8. Zero Ctrls -------------------------------------------------
    zero = next((b for b in picker.buttons if b.command == "zero_ctrls"),
                None)
    _Check(zero is not None, "the picker has a Zero Ctrls button")
    _Check(zero.live and not zero.decoration,
           "and it is live and clickable")

    # Pose two controls, zero ONLY one of them via the selection, and the
    # other must keep its pose -- that is the difference between "zero the
    # selection" and "zero everything".
    # Fresh handles: `target` was rebound to an attr_target dict by the
    # IK/FK section above.
    singles = [b for b in picker.visible(body.id)
               if b.live and len(b.targets) == 1]
    a_path, b_path = singles[0].targets[0], singles[1].targets[0]
    for path in (a_path, b_path):
        prim = stage.GetPrimAtPath(path)
        attr = prim.GetAttribute("avars:tx")
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("avars:tx", Sdf.ValueTypeNames.Double)
        attr.Set(4.0)
    appController._processEvents()

    selection.clearPrims()
    selection.setPrim(stage.GetPrimAtPath(a_path))
    appController._processEvents()
    view.picked.emit([zero], "replace")
    appController._processEvents()

    a_attr = stage.GetPrimAtPath(a_path).GetAttribute("avars:tx")
    b_attr = stage.GetPrimAtPath(b_path).GetAttribute("avars:tx")
    _Check(not a_attr.HasAuthoredValue(),
           "the selected control was zeroed (opinion cleared)")
    _Check(b_attr.HasAuthoredValue() and float(b_attr.Get()) == 4.0,
           "the unselected one kept its pose, got %s" % b_attr.Get())

    # --- 9. and it is undoable -----------------------------------------
    #
    # This is the assertion the bug needed. The panel always LOOKED like
    # it recorded -- it opened an undo scope around the clear -- but the
    # scope covered an empty list of paths, which snapshots nothing and
    # pushes nothing, and OpenPickerPanel threw away the stack it was
    # handed on top of that. So zeroing was a one-way door, and zeroing
    # with nothing selected took the whole rig through it.
    undo = getattr(panel, "_undo", None)
    _Check(undo is not None, "the picker panel was handed the undo stack")
    _Check(undo.CanUndo(), "zeroing pushed an undo entry")
    undo.Undo()
    appController._processEvents()
    a_attr = stage.GetPrimAtPath(a_path).GetAttribute("avars:tx")
    _Check(a_attr.HasAuthoredValue() and float(a_attr.Get()) == 4.0,
           "undo put the zeroed pose back, got %s"
           % (a_attr.Get() if a_attr.HasAuthoredValue() else "nothing"))
    undo.Redo()
    appController._processEvents()
    _Check(not stage.GetPrimAtPath(a_path)
           .GetAttribute("avars:tx").HasAuthoredValue(),
           "redo zeroed it again")

    # Nothing selected -> everything.
    selection.clearPrims()
    appController._processEvents()
    view.picked.emit([zero], "replace")
    appController._processEvents()
    _Check(not b_attr.HasAuthoredValue(),
           "with nothing selected it zeroed the whole rig")

    _Check(undo.CanUndo(), "zeroing the whole rig pushed an undo entry too")
    undo.Undo()
    appController._processEvents()
    b_attr = stage.GetPrimAtPath(b_path).GetAttribute("avars:tx")
    _Check(b_attr.HasAuthoredValue() and float(b_attr.Get()) == 4.0,
           "the whole-rig zero came back in one step, got %s"
           % (b_attr.Get() if b_attr.HasAuthoredValue() else "nothing"))

    # F FRAMES THE SELECTION FROM THE PICKER, as it does over the
    # viewport. It does not arrive on its own: usdview routes F through
    # one application-wide AppEventFilter that only reaches widgets in
    # the main window, and the picker is a QDialog -- its own top-level
    # window -- so the key landed and nothing framed.
    #
    # The ACTION is what is watched, not the camera. Framing moves the
    # camera by an amount that depends on the selection's bound and the
    # viewport's aspect, so asserting on the camera would be asserting
    # on usdview's framing maths; what this owns is that the picker asks
    # for it.
    actionType = getattr(QtGui, "QAction", None) or QtWidgets.QAction
    frameAction = appController._mainWindow.findChild(
        actionType, "actionFrame_Selected")
    _Check(frameAction is not None,
           "usdview has no actionFrame_Selected to frame through")
    fired = []
    frameAction.triggered.connect(lambda *a: fired.append(1))

    panel._FrameSelection()
    _Check(len(fired) == 1,
           "the picker's frame helper did not trigger Frame Selected")

    panel.keyPressEvent(QtGui.QKeyEvent(
        QtCore.QEvent.Type.KeyPress, QtCore.Qt.Key_F,
        QtCore.Qt.NoModifier))
    appController._processEvents()
    _Check(len(fired) == 2,
           "F in the picker did not frame the selection (fired %d times)"
           % len(fired))

    # A modifier is somebody else's shortcut, so the panel must not claim it.
    panel.keyPressEvent(QtGui.QKeyEvent(
        QtCore.QEvent.Type.KeyPress, QtCore.Qt.Key_F,
        QtCore.Qt.ControlModifier))
    appController._processEvents()
    _Check(len(fired) == 2,
           "Ctrl+F was swallowed by the picker and framed the selection")

    shot = os.getenv("RIGEXEC_PICKER_SHOT")
    if shot:
        panel.grab().save(shot)

    print("RIGEXEC_PICKER_OK %d live / %d dead / %d decoration, %d panels; "
          "click selects, shift adds, reverse highlight ok; %d IK/FK "
          "switches bound and cycling (%s -> %g and back); Body draws %d "
          "with modes applied, %d without"
          % (live, dead, deco, len(picker.panels), len(switches),
             switch.attr_target["attr"], float(attr.Get()),
             len(with_modes), len(drawn)))
