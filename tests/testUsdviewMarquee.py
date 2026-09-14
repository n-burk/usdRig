#
# VIEWPORT BOX SELECTION, end to end in the app.
#
# The marquee arithmetic is asserted Qt-free in
# tests/python/test_gizmo_marquee.py. What was never asserted anywhere is
# the PLUMBING: that a press-drag-release on the stage view reaches
# GizmoController, becomes a band, and lands in usdview's own selection.
# That is the half an animator actually uses, and the half that can break
# without a single unit test noticing.
#
# So this drives both routes and checks they agree:
#   * Marquee() called directly -- the arithmetic wired to the selection;
#   * synthetic QMouseEvents through eventFilter -- the real gesture.
#
import os

from pxr import Gf, Sdf, Usd, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

RIG = "/Biped/Rig"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Controller(appController):
    import gizmoUI
    controller = gizmoUI._controller
    _Check(controller is not None,
           "the viewport tools installed themselves on stage open")
    return controller


def _Band(appController, controller):
    """A band, in PHYSICAL pixels, that covers the whole character."""
    import gizmoMarquee
    stage = appController._dataModel.stage
    positions = gizmoMarquee.ScreenPositions(
        stage, *_CameraBits(controller),
        time=appController._dataModel.currentFrame)
    _Check(positions,
           "controls project to screen at all (got %d)" % len(positions))
    xs = [p[0] for p in positions.values()]
    ys = [p[1] for p in positions.values()]
    return (min(xs) - 10, min(ys) - 10, max(xs) + 10, max(ys) + 10), positions


def _CameraBits(controller):
    camera, viewport, _ratio = controller._Camera()
    return camera, viewport


def _Selected(appController):
    return set(str(p.GetPath())
               for p in appController._dataModel.selection.getPrims()
               if p and p.IsValid() and not p.IsPseudoRoot())


def testUsdviewInputFunction(appController):
    import gizmoMarquee
    import gizmoUI

    appController._processEvents()
    stage = appController._dataModel.stage
    controller = _Controller(appController)
    view = controller._view
    _Check(view is not None, "the controller found usdview's stage view")

    # --- 1. the controls are there and selectable ----------------------
    selectable = gizmoMarquee.SelectablePaths(stage)
    _Check(len(selectable) > 50,
           "a useful number of controls are box-selectable, got %d"
           % len(selectable))

    # --- 2. Marquee() puts them in usdview's selection ------------------
    band, positions = _Band(appController, controller)
    appController._dataModel.selection.clearPrims()
    appController._processEvents()

    got = controller.Marquee(band[0], band[1], band[2], band[3])
    appController._processEvents()
    _Check(got, "a band over the whole character catches something")
    chosen = _Selected(appController)
    _Check(chosen == set(got),
           "and every one of them reached usdview's selection: %d vs %d"
           % (len(chosen), len(got)))
    _Check(len(chosen) > 50,
           "which is most of the rig, got %d" % len(chosen))

    # --- 3. a band over ONE control catches just it ---------------------
    path, (px, py) = sorted(positions.items())[0]
    appController._dataModel.selection.clearPrims()
    appController._processEvents()
    controller.Marquee(px - 2, py - 2, px + 2, py + 2)
    appController._processEvents()
    tight = _Selected(appController)
    _Check(path in tight,
           "a tight band round %s caught it, got %s"
           % (path.rsplit("/", 1)[-1], sorted(p.rsplit("/", 1)[-1]
                                              for p in tight)))

    # --- 4. THE REAL GESTURE, through the event filter ------------------
    # This is the part no test covered. A press, three moves past the
    # slop, and a release, delivered exactly as Qt would deliver them.
    appController._dataModel.selection.clearPrims()
    appController._processEvents()

    ratio = controller._Ratio()
    x0, y0, x1, y1 = [v / ratio for v in band]

    def _Mouse(kind, x, y, buttons=QtCore.Qt.LeftButton,
               button=QtCore.Qt.LeftButton):
        pos = QtCore.QPointF(float(x), float(y))
        return QtGui.QMouseEvent(kind, pos, view.mapToGlobal(pos.toPoint()),
                                 button, buttons, QtCore.Qt.NoModifier)

    controller.eventFilter(view, _Mouse(QtCore.QEvent.MouseButtonPress,
                                        x0, y0))
    _Check(controller._marqueeAt is not None,
           "the press armed a marquee")

    steps = 4
    for i in range(1, steps + 1):
        mx = x0 + (x1 - x0) * i / float(steps)
        my = y0 + (y1 - y0) * i / float(steps)
        controller.eventFilter(view, _Mouse(QtCore.QEvent.MouseMove, mx, my))
    _Check(controller._marqueeBand is not None,
           "dragging past the slop built a band, got %r"
           % (controller._marqueeBand,))

    controller.eventFilter(view, _Mouse(QtCore.QEvent.MouseButtonRelease,
                                        x1, y1, buttons=QtCore.Qt.NoButton))
    appController._processEvents()

    dragged = _Selected(appController)
    _Check(dragged,
           "the DRAG selected something: the band was %r, the direct call "
           "caught %d" % (controller._marqueeBand, len(got)))
    _Check(len(dragged) > 50,
           "and it caught most of the rig, got %d" % len(dragged))
    _Check(controller._marqueeAt is None and controller._marqueeBand is None,
           "and the release cleared the marquee state")

    # --- 5. the modes ---------------------------------------------------
    _Check(gizmoMarquee.Mode(ctrl=False, shift=False)
           == gizmoMarquee.MODE_REPLACE, "plain replaces")
    _Check(gizmoMarquee.Mode(ctrl=False, shift=True)
           == gizmoMarquee.MODE_TOGGLE, "shift toggles")
    _Check(gizmoMarquee.Mode(ctrl=True, shift=False)
           == gizmoMarquee.MODE_REMOVE, "ctrl removes")

    # Two controls FAR APART on screen. Taking the first two by path
    # picks neighbours that a 4-pixel band catches together, which says
    # nothing about toggling.
    ordered = sorted(positions.items(), key=lambda kv: kv[1][0])
    two = [ordered[0], ordered[-1]]
    _Check(abs(two[0][1][0] - two[1][1][0]) > 40,
           "the two probe controls are far enough apart to band separately")

    appController._dataModel.selection.clearPrims()
    appController._processEvents()
    for path, (px, py) in two:
        controller.Marquee(px - 2, py - 2, px + 2, py + 2,
                           gizmoMarquee.MODE_TOGGLE)
        appController._processEvents()
    picked = _Selected(appController)
    _Check(set(p for p, _ in two) <= picked,
           "two toggles caught both controls, got %s"
           % sorted(p.rsplit("/", 1)[-1] for p in picked))

    before = len(picked)
    path, (px, py) = two[0]
    controller.Marquee(px - 2, py - 2, px + 2, py + 2,
                       gizmoMarquee.MODE_REMOVE)
    appController._processEvents()
    left = _Selected(appController)
    _Check(path not in left and len(left) < before,
           "ctrl removed it: %d -> %d, %s gone = %s"
           % (before, len(left), path.rsplit("/", 1)[-1],
              path not in left))

    # --- 6. AND IT STILL WORKS WITH TOUCHPOSE ON ------------------------
    # TouchPose owns the drag while it is active, and its regions only
    # cover the mesh. Before this, turning TouchPose on silently took
    # box selection of the controls away -- a box round the IK controls
    # caught nothing, and nothing said why.
    touch = None
    try:
        import touchPoseUI
        touch = touchPoseUI.TouchPoseController._instance
        if touch is None:
            # The controller is a singleton built when the panel opens,
            # so open it the way an animator does.
            for child in appController._mainWindow.menuBar().children():
                if not isinstance(child, QtWidgets.QMenu):
                    continue
                if str(child.title()).replace("&", "") != "RigExec":
                    continue
                for action in child.actions():
                    if action.text() == "TouchPose":
                        action.trigger()
                        break
            appController._processEvents()
            touch = touchPoseUI.TouchPoseController._instance
    except Exception as error:
        print("  (TouchPose unavailable: %s)" % error)
    if touch is not None:
        was = touch.active
        touch.SetActive(True)
        appController._processEvents()
        _Check(touch.active, "TouchPose reports itself active")
        _Check(controller._TouchPoseActive(),
               "and the gizmo sees it, so the native marquee stands down")

        appController._dataModel.selection.clearPrims()
        appController._processEvents()
        names = touch.Marquee(band[0], band[1], band[2], band[3])
        appController._processEvents()
        withTouch = _Selected(appController)
        _Check(len(withTouch) > 50,
               "with TouchPose ON the same band still catches the "
               "controls, got %d (regions: %d)" % (len(withTouch),
                                                   len(names)))
        touch.SetActive(was)
        appController._processEvents()
    else:
        print("  (no TouchPose controller on this stage; step 6 skipped)")

    appController._dataModel.selection.clearPrims()
    print("RIGEXEC_MARQUEE_OK %d selectable; direct band caught %d, the "
          "drag caught %d; tight band, toggle and remove correct; "
          "TouchPose-on band caught %d"
          % (len(selectable), len(got), len(dragged),
             len(withTouch) if touch is not None else -1))
