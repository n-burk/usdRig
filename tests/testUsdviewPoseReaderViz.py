#!/usr/bin/env python
"""testusdview: the Shape Editor's pose-reader overlay draws, live.

The geometry is pinned headlessly by tests/python/test_pose_reader_viz.py;
this proves the Qt half in a real usdview -- that the overlay is OFF until
asked for, follows the viewport selection, brightens as a pose fires,
answers its part and scope toggles, and goes away again.

Set RIGEXEC_POSE_READER_SHOT=/path.png to save a window grab of the
overlay with the left arm posed.

Usage (see bin/test/run_testusdview_pose_readers.*):
    testusdview --testScript tests/testUsdviewPoseReaderViz.py \
        examples/biped/Biped_stack.usda
"""
import os
import sys

from pxr import Sdf, Usd

_FAILURES = []

SHOULDER_DRIVER = "shoulder_l_driver"
ARM = ("/Biped/Rig/Main/Shot/Aux/Controls/M_Body/M_Torso/M_Chest/"
       "M_ChestTop/L_Shldr/L_UpArmSwing/L_UpArm")


def _Check(condition, message):
    if not condition:
        _FAILURES.append(message)
        print("FAIL: %s" % message)
    else:
        print("ok: %s" % message)


def _Root(stage):
    path = stage.GetRootLayer().realPath
    walk = os.path.dirname(os.path.abspath(path))
    while walk and not os.path.isdir(os.path.join(walk, "plugin")):
        parent = os.path.dirname(walk)
        if parent == walk:
            return None
        walk = parent
    return walk


def _Kinds(view):
    ops = view.Ops() if view is not None else []
    out = {}
    for op in ops:
        out[op.kind] = out.get(op.kind, 0) + 1
    return out


def testUsdviewInputFunction(appController):
    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    session = stage.GetSessionLayer()
    stage.SetEditTarget(Usd.EditTarget(session))

    root = _Root(stage)
    for extra in (os.path.join(root, "plugin", "shapeEditor"),
                  os.path.join(root, "plugin", "rigExecUsdview")):
        if extra not in sys.path:
            sys.path.insert(0, extra)
    import poseReaderModel as viz
    import shapeEditorUI

    panel = shapeEditorUI.ShapeEditorPanel.GetInstance(api)
    panel.show()
    appController._processEvents()

    # --- 1. off by default ------------------------------------------------
    _Check(not panel._vizGroup.isChecked(),
           "the viewport group is unticked by default")
    _Check(panel._vizView is None,
           "no overlay exists until it is asked for")

    interp = next((i for i in panel._interpolators
                   if i.name == SHOULDER_DRIVER), None)
    _Check(interp is not None and interp.driver is not None,
           "the left shoulder interpolator and its driver are found")
    if interp is None:
        appController._mainWindow.close()
        return

    # --- 2. on, following the selection -----------------------------------
    api.dataModel.selection.setPrimPath(interp.driver)
    appController._processEvents()
    panel._vizGroup.setChecked(True)
    appController._processEvents()
    panel.Refresh()
    appController._processEvents()
    view = panel._vizView
    _Check(view is not None and view.overlay is not None,
           "ticking the group creates the overlay over the stage view")
    _Check(view is not None and view.overlay.isVisible(),
           "and shows it")
    _Check(panel._vizSelected == {interp.path},
           "selecting the driver selects its interpolator: %s"
           % sorted(str(p) for p in panel._vizSelected))
    kinds = _Kinds(view)
    camera = view._Camera() if view is not None else None
    _Check(kinds.get("polygon", 0) >= len(interp.poses),
           "a cone silhouette per pose reaches the screen: %s "
           "(panel says %r; %d reader(s); camera %s)"
           % (kinds, panel._vizInfo.text(),
              len(view._readers) if view is not None else -1,
              "ok" if camera is not None else "NONE"))
    _Check(kinds.get("dot", 0) >= 1, "and the live driver is drawn")

    # --- 3. a pose fires and the view brightens ---------------------------
    def Fill(posePath):
        """The brightest fill drawn for one pose."""
        return max((op.fillAlpha for op in view.Ops()
                    if op.owner == posePath), default=0.0)

    restFill = {p.path: Fill(p.path) for p in interp.poses}
    stage.GetPrimAtPath(ARM).GetAttribute("avars:rz").Set(55.0)
    appController._processEvents()
    panel.Refresh()
    appController._processEvents()
    firing = [p for p in interp.poses
              if not p.is_neutral and abs(p.weight) > viz.LABEL_LIVE]
    _Check(firing, "raising the arm fires a non-neutral shoulder pose: %s"
           % [(p.name, round(p.weight, 3)) for p in interp.poses])
    top = max(firing, key=lambda p: p.weight) if firing else None
    if top is not None:
        _Check(Fill(top.path) > restFill[top.path],
               "%s, now at %.3f, is drawn brighter than at rest "
               "(%.3f -> %.3f)" % (top.name, top.weight,
                                   restFill[top.path], Fill(top.path)))
    texts = [op.text for op in view.Ops() if op.kind == "text"]
    _Check(any(p.name in t for p in firing for t in texts),
           "and its label carries its published weight: %s" % texts[:4])

    # --- 3b. a row picked in the panel survives a viewport selection ------
    # Posing means clicking the arm control, which REPLACES the viewport
    # selection; the reader picked in the tree has to stay drawn.
    def Row():
        """The interpolator's CURRENT row: a rebuild deletes the old one."""
        return next((panel._tree.topLevelItem(i)
                     for i in range(panel._tree.topLevelItemCount())
                     if panel._tree.topLevelItem(i).data(
                         0, shapeEditorUI.QtCore.Qt.UserRole + 1)
                     == str(interp.path)), None)

    row = Row()
    _Check(row is not None, "the interpolator has a row in the tree")
    if row is not None:
        panel._tree.clearSelection()
        row.setSelected(True)
        appController._processEvents()
        api.dataModel.selection.setPrimPath(ARM)
        appController._processEvents()
        _Check(interp.path in panel._vizSelected and
               any(op.owner is not None for op in view.Ops()),
               "a reader picked in the panel stays drawn after the arm is "
               "selected in the viewport")
        panel._Populate()
        appController._processEvents()
        _Check(Row() is not None and interp.path in panel._vizSelected and
               any(i.isSelected() for i in panel._tree.selectedItems()),
               "and survives the tree being rebuilt")
        panel._tree.clearSelection()
        appController._processEvents()
        _Check(not any(op.owner is not None for op in view.Ops()),
               "clearing the pick, with the arm selected, draws nothing")

        # A pure DESELECT in the viewport unpicks it in the panel too:
        # pick the row (which selects the driver), ADD the arm -- the pick
        # stays -- then remove the driver, as Ctrl+right-click does.
        selection = api.dataModel.selection
        Row().setSelected(True)
        appController._processEvents()
        selection.addPrimPath(ARM)
        appController._processEvents()
        _Check(interp.path in panel._vizSelected,
               "adding the arm to the selection keeps the pick")
        selection.removePrimPath(interp.driver)
        appController._processEvents()
        _Check(interp.path not in panel._vizSelected and
               not Row().isSelected() and
               not any(op.owner is not None for op in view.Ops()),
               "deselecting the driver in the viewport unpicks its row and "
               "clears its readers")
        selection.setPrimPath(interp.driver)
        appController._processEvents()

    shot = os.environ.get("RIGEXEC_POSE_READER_SHOT")
    if shot:
        appController._mainWindow.grab().save(shot)
        print("saved %s" % shot)

    # --- 4. the part and scope toggles ------------------------------------
    full = len(view.Ops())
    panel._vizParts[viz.PART_CONES].setChecked(False)
    panel._vizParts[viz.PART_CORES].setChecked(False)
    appController._processEvents()
    _Check(len(view.Ops()) < full,
           "turning the cones and cores off removes them: %d -> %d"
           % (full, len(view.Ops())))
    panel._vizParts[viz.PART_CONES].setChecked(True)
    panel._vizParts[viz.PART_CORES].setChecked(True)
    index = panel._vizScope.findData(viz.SCOPE_ALL)
    panel._vizScope.setCurrentIndex(index)
    appController._processEvents()
    _Check(len(view.Ops()) > full,
           "ALL draws more than one interpolator: %d -> %d"
           % (full, len(view.Ops())))

    # --- 5. off again ------------------------------------------------------
    panel._vizGroup.setChecked(False)
    appController._processEvents()
    _Check(view.Ops() == [] and not view.overlay.isVisible(),
           "unticking clears and hides the overlay")

    stage.GetPrimAtPath(ARM).GetAttribute("avars:rz").Clear()
    panel.close()
    appController._processEvents()
    if _FAILURES:
        print("%d FAILURE(S)" % len(_FAILURES))
        sys.exit(1)
    print("RIGEXEC_USDVIEW_POSE_READER_VIZ_OK")
    appController._mainWindow.close()
