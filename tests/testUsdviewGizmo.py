#
# THE VIEWPORT GIZMO TOOLBAR, end to end.
#
# Written for the same reason as testUsdviewCurvenetMove.py: the headless
# tests in tests/python call the math and the edit helpers directly and
# never go near a camera, so they cannot see the one thing a manipulator
# is -- a PROJECTION that a mouse has to be able to hit. This script opens
# examples/ArmShotAnim.usda (the container installs the toolbar when the
# stage loads), selects the HandIK control and drives synthetic mouse and
# key events through the gizmo's own projected handle positions. Every
# assertion is about what landed on the stage afterwards, not about what
# the controller thinks it did.
#
# Set RIGEXEC_GIZMO_SHOT=/path.png to save a window grab for inspection.
#
import math
import os

from pxr import Gf, Sdf, UsdGeom

CONTROL = "/Shot/HeroArm/Rig/Controls/HandIK"
XFORM = "/Shot/HeroArm"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Mouse(QtCore, QtGui, view, kind, x, y, button=None, buttons=None,
           modifiers=None):
    """
    One synthetic mouse event in the view's LOGICAL pixels, which is what
    HandleScreenPositions() reports and what Qt delivers.

    `button` is separate from `buttons` because the controller reads
    event.button() to tell a left grab from Maya's middle-drag-repeats-
    the-selected-handle; a helper that hard-codes LeftButton can send a
    "middle" press the controller will never recognise as one.
    """
    pos = QtCore.QPointF(float(x), float(y))
    glob = view.mapToGlobal(QtCore.QPoint(int(x), int(y)))
    globF = QtCore.QPointF(float(glob.x()), float(glob.y()))
    if button is None:
        button = QtCore.Qt.LeftButton
    if buttons is None:
        buttons = button
    if modifiers is None:
        modifiers = QtCore.Qt.NoModifier
    return QtGui.QMouseEvent(kind, pos, globF, button, buttons, modifiers)


def _Lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


class _Driver(object):
    """Mouse, keyboard and selection, in the units the controller uses."""

    def __init__(self, appController, controller):
        from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets
        import gizmoUI
        self.QtCore, self.QtGui, self.QtWidgets = QtCore, QtGui, QtWidgets
        self.app = appController
        self.api = appController._usdviewApi
        self.controller = controller
        self.view = gizmoUI.StageView(self.api)

    def Pump(self):
        self.app._processEvents()

    def Ratio(self):
        return self.view.devicePixelRatioF()

    def Send(self, kind, x, y, **kw):
        self.QtWidgets.QApplication.sendEvent(
            self.view, _Mouse(self.QtCore, self.QtGui, self.view, kind,
                              x, y, **kw))
        self.Pump()

    def Press(self, point, **kw):
        self.Send(self.QtCore.QEvent.Type.MouseButtonPress, *point, **kw)

    def Move(self, point, **kw):
        self.Send(self.QtCore.QEvent.Type.MouseMove, *point, **kw)

    def Release(self, point, button=None):
        self.Send(self.QtCore.QEvent.Type.MouseButtonRelease, *point,
                  button=button, buttons=self.QtCore.Qt.NoButton)

    def Drag(self, start, end, steps=4, button=None):
        self.Press(start, button=button)
        _Check(self.controller.IsDragging(),
               "a press at %s started a drag (handles: %s)" % (
                   (start,), sorted(self.controller.HandleScreenPositions())))
        for i in range(1, steps + 1):
            self.Move(_Lerp(start, end, i / float(steps)),
                      button=self.QtCore.Qt.NoButton,
                      buttons=button or self.QtCore.Qt.LeftButton)
        self.Release(end, button=button)
        _Check(not self.controller.IsDragging(), "the release ended the drag")

    def AxisPoints(self, name):
        positions = self.controller.HandleScreenPositions()
        _Check(name in positions, "handle %r is present, not just %s" % (
            name, sorted(positions)))
        return positions[name][0], positions[name][-1]

    def DragAxis(self, name, fraction=0.35, start=0.5):
        """Slide along an axis handle by `fraction` of its screen length."""
        a, b = self.AxisPoints(name)
        self.Drag(_Lerp(a, b, start), _Lerp(a, b, start + fraction))

    def DragRing(self, name, quarter=6):
        positions = self.controller.HandleScreenPositions()
        _Check(name in positions, "ring %r is present, not just %s" % (
            name, sorted(positions)))
        points = positions[name]
        self.Drag(points[0], points[quarter % len(points)])

    def Key(self, key, modifiers=None):
        # QtTest is the one Qt module pxr.Usdviewq.qt does not re-export, so
        # it has to come from the binding directly. That is fine in a test;
        # the plugin modules themselves still go through Usdviewq.qt.
        try:
            from PySide6 import QtTest
        except ImportError:
            from PySide2 import QtTest
        if modifiers is None:
            modifiers = self.QtCore.Qt.NoModifier
        self.view.setFocus()
        QtTest.QTest.keyClick(self.view, key, modifiers)
        self.Pump()

    def Select(self, path):
        prim = self.api.stage.GetPrimAtPath(path)
        _Check(prim, "the stage has a prim at %s" % path)
        self.api.ClearPrimSelection()
        self.api.AddPrimToSelection(prim)
        self.Pump()
        return prim

    def HandleAt(self, point):
        """The handle a left click at `point` would grab, by name."""
        import gizmoScreen
        import gizmoUI
        ratio = self.Ratio()
        hit = gizmoScreen.HitTest(self.controller.Handles(),
                                  point[0] * ratio, point[1] * ratio,
                                  gizmoUI.HIT_PIXELS * ratio)
        return hit.name if hit is not None else None

    def FreeRotatePoint(self):
        """
        A point inside the free-rotate ball that is not on a ring.

        The rings outrank the sphere in HitTest by design (the ball
        covers all three of them), and an obliquely viewed ring projects
        to an ellipse that passes close to the centre, so "somewhere near
        the middle" is not good enough -- the answer depends on the
        camera. Ask the same HitTest the controller uses instead of
        guessing.
        """
        import gizmoScreen
        import gizmoUI
        handles = self.controller.Handles()
        free = None
        for handle in handles:
            if handle.name == "free":
                free = handle
        _Check(free is not None, "the rotate manipulator has a free-rotate "
               "ball (handles: %s)" % [h.name for h in handles])
        ratio = self.Ratio()
        cx, cy = free.points[0]
        radius = free.radiusPixels
        for fraction in (0.45, 0.3, 0.6, 0.15, 0.75):
            for step in range(24):
                angle = step * math.pi / 12.0
                x = cx + radius * fraction * math.cos(angle)
                y = cy + radius * fraction * math.sin(angle)
                hit = gizmoScreen.HitTest(handles, x, y,
                                          gizmoUI.HIT_PIXELS * ratio)
                if hit is not None and hit.name == "free":
                    return (x / ratio, y / ratio)
        raise AssertionError(
            "some point inside the free-rotate ball resolves to it rather "
            "than to a ring; from this camera none of 120 candidates did")


def _Values(prim, names, frame):
    return [prim.GetAttribute(n).Get(frame) for n in names]


def _Changed(before, after, tolerance=1e-6):
    return [i for i in range(len(before))
            if abs((before[i] or 0.0) - (after[i] or 0.0)) > tolerance]


def testUsdviewInputFunction(appController):
    import gizmoMath
    import gizmoSettings
    import gizmoUI

    appController._processEvents()
    controller = gizmoUI.GetController()
    _Check(controller is not None,
           "the container installed the viewport tools on stage load")
    d = _Driver(appController, controller)
    stage = d.api.stage
    session = stage.GetSessionLayer()
    _Check(stage.GetEditTarget().GetLayer() == session,
           "usdview's edit target is the session layer, which is where "
           "every assertion below looks for the authored spec")
    frame = d.api.frame

    # testusdview's window is never activated by the window manager, so
    # QApplication.activeWindow() is None and Qt's shortcut map refuses
    # EVERY shortcut before it looks at the key -- Ctrl+Z would silently
    # do nothing here for reasons that have nothing to do with the gizmo.
    # Supplying the activation the window manager would is what lets the
    # undo/redo assertions below exercise the real key path rather than
    # calling the actions by hand.
    d.QtWidgets.QApplication.setActiveWindow(appController._mainWindow)
    d.Pump()
    _Check(d.QtWidgets.QApplication.activeWindow() is not None,
           "the main window is active, so application shortcuts dispatch")

    # Widen the window so the toolbar is not folded into QToolBar's
    # overflow chevron, which is where testusdview's default width puts
    # the status label -- the one control a reader of the grab most wants
    # to see. The rendered viewport stays 597x540 whatever this says:
    # testusdview pins it with SetPhysicalWindowSize for reproducible test
    # images, which is why the grab shows a small render in a wide window.
    # Done here rather than before the grab because every drag below reads
    # its coordinates from the controller's own projections, so the layout
    # must settle before anything is projected.
    appController._mainWindow.resize(1800, 1000)
    d.Pump()

    # Frame the control and tumble off the default straight-down-Z view.
    # Not cosmetic: down -Z the Z axis projects to a point, so its handle
    # is locked and ungrabbable, the rotate rings degenerate to lines, and
    # the X arrow lands outside testusdview's 597 px viewport. Every drag
    # below needs three usable axes, so the test asks for the view an
    # animator would actually work in.
    d.Select(CONTROL)
    appController._frameSelection()
    camera = d.api.dataModel.viewSettings.freeCamera
    if camera is not None:
        camera.rotTheta = 35.0
        camera.rotPhi = -25.0
        camera.dist = camera.dist * 2.6
    d.Pump()

    # --- 1. Translate a control in Animation mode, undo, redo ------------
    prim = d.Select(CONTROL)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    tx = prim.GetAttribute("avars:tx")
    before = tx.Get(frame)
    d.DragAxis("x")
    afterX = tx.Get(frame)
    _Check(abs(afterX - before) > 1e-6, "the x drag changed avars:tx at the "
           "frame: %s -> %s" % (before, afterX))
    spec = session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
    _Check(spec is not None and spec.HasSpline(),
           "animation mode authored a spline knot in the session layer")
    _Check(controller.undoStack.CanUndo(), "the drag pushed one edit")
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    _Check(abs(tx.Get(frame) - before) < 1e-9,
           "Ctrl+Z restored avars:tx: %s" % tx.Get(frame))
    _Check(session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
           is None, "the undo removed the session spec the drag created")
    d.Key(d.QtCore.Qt.Key_Z,
          d.QtCore.Qt.ControlModifier | d.QtCore.Qt.ShiftModifier)
    _Check(abs(tx.Get(frame) - afterX) < 1e-9,
           "Ctrl+Shift+Z re-applied avars:tx: %s" % tx.Get(frame))
    controller.Undo()
    d.Pump()

    # --- 2. Centre handle moves in the camera plane ----------------------
    positions = controller.HandleScreenPositions()
    _Check("center" in positions, "the move manipulator has a centre handle")
    c = positions["center"][0]
    beforeT = _Values(prim, gizmoMath.AVAR_T, frame)
    d.Drag(c, (c[0] + 30, c[1] - 30))
    afterT = _Values(prim, gizmoMath.AVAR_T, frame)
    _Check(len(_Changed(beforeT, afterT)) >= 2,
           "the centre drag moved the control in the camera plane, which "
           "is oblique to every axis, so at least two avars change: "
           "%s -> %s" % (beforeT, afterT))
    controller.Undo()
    d.Pump()

    # --- 3. Default mode writes the default and warns --------------------
    controller.SetWriteMode(gizmoMath.WRITE_DEFAULT)
    d.Pump()
    d.DragAxis("x")
    spec = session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
    _Check(spec is not None and spec.HasDefaultValue()
           and not spec.HasSpline(),
           "default mode authored a default, not a knot")
    _Check("outranked" in controller.Status(),
           "the status warns that the file's spline outranks the default: "
           "%r" % controller.Status())
    controller.Undo()
    d.Pump()
    controller.SetWriteMode(gizmoMath.WRITE_ANIMATION)
    d.Pump()

    # --- 4. Rotate and scale ---------------------------------------------
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    rBefore = _Values(prim, gizmoMath.AVAR_R, frame)
    d.DragRing("z")
    rAfter = _Values(prim, gizmoMath.AVAR_R, frame)
    _Check(_Changed(rBefore, rAfter),
           "the ring drag changed a rotation avar: %s -> %s"
           % (rBefore, rAfter))
    controller.Undo()
    d.Pump()
    controller.SetTool(gizmoUI.TOOL_SCALE)
    d.Pump()
    sx = prim.GetAttribute("avars:sx")
    d.DragAxis("x")
    _Check(abs(sx.Get(frame) - 1.0) > 1e-6,
           "the scale drag changed avars:sx: %s" % sx.Get(frame))
    controller.Undo()
    d.Pump()

    # --- 5. Pivot mode edits rest:*, never avars -------------------------
    controller.SetChannels(gizmoMath.CHANNELS_PIVOT)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    _Check(controller.Target().kind == "rig-pivot",
           "pivot mode on a control resolves to a rig-pivot target, not %r"
           % controller.Target().kind)
    d.DragAxis("x")
    # rest:tx is unauthored on the rig, so the attribute only exists once
    # the pivot drag has written it -- fetched here rather than before.
    restTx = prim.GetAttribute("rest:tx")
    _Check(restTx and abs(restTx.Get(frame)) > 1e-6,
           "the pivot drag wrote rest:tx: %s"
           % (restTx.Get(frame) if restTx else None))
    _Check(abs(tx.Get(frame) - before) < 1e-9,
           "the pivot drag left the avars alone: avars:tx is %s"
           % tx.Get(frame))
    controller.SetTool(gizmoUI.TOOL_SCALE)
    d.Pump()
    _Check(controller.Target() is not None
           and "unavailable" in controller.Status(),
           "scale is refused in pivot mode (rest spaces are "
           "orthonormalized): %r" % controller.Status())
    _Check(controller.HandleScreenPositions() == {},
           "a refused tool draws no handles: %s"
           % sorted(controller.HandleScreenPositions()))
    controller.Undo()
    d.Pump()
    _Check(session.GetAttributeAtPath(Sdf.Path(CONTROL + ".rest:tx"))
           is None, "the undo removed the rest:tx spec the pivot drag "
           "created, the same way it does for an avar")
    controller.SetChannels(gizmoMath.CHANNELS_POSE)
    d.Pump()

    # --- 6. A plain Xform edits its xformOps ------------------------------
    d.Select(XFORM)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None
           and controller.Target().kind == "xform-pose", controller.Reason())
    xformOp = stage.GetAttributeAtPath(Sdf.Path(XFORM + ".xformOp:translate"))
    opBefore = xformOp.Get(frame) if xformOp else None
    d.DragAxis("y")
    opPath = Sdf.Path(XFORM + ".xformOp:translate")
    op = session.GetAttributeAtPath(opPath)
    _Check(op is not None,
           "the drag authored xformOp:translate in the session layer")
    # Read the VALUE through the stage, not spec.default: Animation mode
    # writes a double3 as a time sample, so the spec's default is None and
    # comparing it to the zero vector proves nothing.
    opAfter = stage.GetAttributeAtPath(opPath).Get(frame)
    _Check(opAfter is not None and Gf.Vec3d(opAfter).GetLength() > 1e-6
           and opAfter != opBefore,
           "the authored translate is non-zero and differs from before the "
           "drag: %s -> %s" % (opBefore, opAfter))
    controller.Undo()
    d.Pump()
    _Check(session.GetAttributeAtPath(
        Sdf.Path(XFORM + ".xformOp:translate")) is None,
        "the undo removed the xformOp spec the drag created")

    # --- 6b. Maya parity: planar handle, middle-drag, step snap, view
    #         ring, gimbal, free rotate, scale ratio, hotkeys ------------
    prim = d.Select(CONTROL)
    controller.SetChannels(gizmoMath.CHANNELS_POSE)
    controller.SetWriteMode(gizmoMath.WRITE_ANIMATION)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    positions = controller.HandleScreenPositions()
    _Check({"xy", "yz", "xz", "center"} <= set(positions),
           "the move manipulator has Maya's three planar handles and a "
           "centre: %s" % sorted(positions))
    corners = positions["xy"]
    _Check(len(corners) == 4, "a planar handle reports its four corners")
    cx = sum(p[0] for p in corners) / 4.0
    cy = sum(p[1] for p in corners) / 4.0
    beforeT = _Values(prim, gizmoMath.AVAR_T, frame)
    d.Drag((cx, cy), (cx + 25, cy - 25))
    afterT = _Values(prim, gizmoMath.AVAR_T, frame)
    changed = _Changed(beforeT, afterT)
    _Check(changed and 2 not in changed,
           "the XY planar drag moved the control in X and Y only: %s -> %s"
           % (beforeT, afterT))
    _Check(controller.SelectedHandleName() == "xy",
           "the dragged handle is the selected (yellow) one: %r"
           % controller.SelectedHandleName())

    # Maya's middle-drag anywhere repeats the selected handle.
    mid = (positions["center"][0][0] + 150,
           positions["center"][0][1] + 120)
    _Check(d.HandleAt(mid) is None,
           "the middle-drag point is clear of every handle, so only the "
           "repeat rule can explain a drag starting there: %r"
           % d.HandleAt(mid))
    d.Drag(mid, (mid[0] + 25, mid[1] - 25),
           button=d.QtCore.Qt.MiddleButton)
    movedT = _Values(prim, gizmoMath.AVAR_T, frame)
    movedBy = _Changed(afterT, movedT)
    _Check(movedBy and 2 not in movedBy,
           "the middle drag repeated the XY handle away from it: %s -> %s"
           % (afterT, movedT))
    controller.Undo()
    controller.Undo()
    d.Pump()

    # Ctrl + an axis drag moves in the plane PERPENDICULAR to that axis
    # (Maya). The modifier is applied to the MOVES, not the press: Qt
    # turns Ctrl+left-click into a right-button press on macOS, so "grab
    # the axis, then hold Ctrl" is the gesture that works everywhere.
    # Run twice on identical geometry, plain then Ctrl, so the assertion
    # is "Ctrl changed which channels moved" rather than the weaker
    # "something moved".
    a, b = d.AxisPoints("x")
    grab = _Lerp(a, b, 0.5)
    away = (grab[0] + 40, grab[1] - 30)
    beforeT = _Values(prim, gizmoMath.AVAR_T, frame)
    d.Drag(grab, away)
    plainMoved = _Changed(beforeT, _Values(prim, gizmoMath.AVAR_T, frame))
    _Check(plainMoved == [0],
           "without Ctrl the same drag moves along X only: %s" % plainMoved)
    controller.Undo()
    d.Pump()
    d.Press(grab)
    _Check(controller.IsDragging(), "the X arrow started a drag")
    for point in (_Lerp(grab, away, 0.5), away):
        d.Move(point, button=d.QtCore.Qt.NoButton,
               buttons=d.QtCore.Qt.LeftButton,
               modifiers=d.QtCore.Qt.ControlModifier)
    d.Release(away)
    ctrlMoved = _Changed(beforeT, _Values(prim, gizmoMath.AVAR_T, frame))
    _Check(ctrlMoved and 0 not in ctrlMoved,
           "Ctrl + the X arrow moves in the YZ plane instead: X is "
           "unchanged and Y or Z moved, got %s (%s -> %s)"
           % (ctrlMoved, beforeT, _Values(prim, gizmoMath.AVAR_T, frame)))
    controller.Undo()
    d.Pump()

    # Step snap quantises the written channel delta. The same drag geometry
    # is run twice -- once with snapping off to measure the raw delta, then
    # with it on -- and the step is deliberately 0.6 of that raw delta, so
    # `loose / step` is about 1.67 and can never be a whole number. That is
    # the whole point: with any step that divides the raw delta, "the
    # result is a multiple of the step" is also true of an unsnapped drag,
    # and the assertion would pass with the feature switched off.
    base = tx.Get(frame)
    d.DragAxis("x", 0.3)
    loose = tx.Get(frame) - base
    controller.Undo()
    d.Pump()
    _Check(abs(loose) > 1e-3,
           "the unsnapped reference drag moved far enough to snap: %s"
           % loose)
    step = abs(loose) * 0.6
    settings = controller.settings.For(gizmoUI.TOOL_TRANSLATE)
    settings.stepSize = step
    settings.stepSnap = True
    d.Pump()
    d.DragAxis("x", 0.3)
    snapped = tx.Get(frame) - base
    multiple = snapped / step
    _Check(abs(snapped) > 1e-9 and abs(multiple - round(multiple)) < 1e-6,
           "step snap wrote a whole multiple of the %.4f step: %.6f is "
           "%.4f steps" % (step, snapped, multiple))
    _Check(abs(snapped - loose) > 1e-3,
           "the snap actually moved the value: the same drag wrote %.6f "
           "unsnapped and %.6f snapped, which is what an ignored stepSnap "
           "would also produce" % (loose, snapped))
    settings.stepSnap = False
    controller.Undo()
    d.Pump()

    # Rotate: view ring (with the live angle readout), gimbal, free rotate.
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    d.Pump()
    positions = controller.HandleScreenPositions()
    _Check({"x", "y", "z", "view", "free"} <= set(positions),
           "the rotate manipulator has Maya's three rings, the view ring "
           "and the free-rotate ball: %s" % sorted(positions))
    rBefore = _Values(prim, gizmoMath.AVAR_R, frame)
    ring = positions["view"]
    d.Press(ring[0])
    _Check(controller.IsDragging(), "the view ring started a drag")
    d.Move(ring[len(ring) // 8])
    d.Move(ring[len(ring) // 4])
    _Check(abs(controller.DragAngle()) > 1.0,
           "the live drag reports the swept angle: %s deg"
           % controller.DragAngle())
    _Check("deg" in controller.Status(),
           "the status shows the rotation amount while dragging: %r"
           % controller.Status())
    d.Release(ring[len(ring) // 4])
    _Check(_Changed(rBefore, _Values(prim, gizmoMath.AVAR_R, frame)),
           "the view ring rotated the control")
    controller.Undo()
    d.Pump()

    controller.settings.For(gizmoUI.TOOL_ROTATE).orientation = \
        gizmoSettings.ORIENT_GIMBAL
    d.Pump()
    rBefore = _Values(prim, gizmoMath.AVAR_R, frame)
    d.DragRing("z", 6)
    rAfter = _Values(prim, gizmoMath.AVAR_R, frame)
    _Check(_Changed(rBefore, rAfter) == [2],
           "the gimbal Z ring changes avars:rz alone: %s -> %s"
           % (rBefore, rAfter))
    controller.Undo()
    controller.settings.For(gizmoUI.TOOL_ROTATE).orientation = \
        gizmoSettings.ORIENT_OBJECT
    d.Pump()

    rBefore = _Values(prim, gizmoMath.AVAR_R, frame)
    freeStart = d.FreeRotatePoint()
    d.Drag(freeStart, (freeStart[0] + 40, freeStart[1] + 12))
    _Check(_Changed(rBefore, _Values(prim, gizmoMath.AVAR_R, frame)),
           "free rotate (the virtual trackball) rotated the control")
    controller.Undo()
    d.Pump()

    # Scale: Maya's ratio rule, then Prevent Negative Scale.
    controller.SetTool(gizmoUI.TOOL_SCALE)
    d.Pump()
    a, b = d.AxisPoints("x")
    d.Drag(_Lerp(a, b, 0.5), b)
    _Check(abs(sx.Get(frame) - 2.0) < 1e-3,
           "dragging the X handle from half length to the tip doubles "
           "avars:sx (Maya's distance-ratio rule): %s" % sx.Get(frame))
    controller.Undo()
    d.Pump()
    controller.settings.For(gizmoUI.TOOL_SCALE).preventNegativeScale = True
    d.Pump()
    a, b = d.AxisPoints("x")
    d.Drag(_Lerp(a, b, 0.5), _Lerp(a, b, -0.5))
    _Check(sx.Get(frame) > 0.0,
           "Prevent Negative Scale keeps avars:sx positive when the cursor "
           "is dragged through the origin: %s" % sx.Get(frame))
    controller.settings.For(gizmoUI.TOOL_SCALE).preventNegativeScale = False
    controller.Undo()
    d.Pump()

    # Hotkeys: Q/W/E/R tools, D toggles pivot, + / - resize.
    d.Key(d.QtCore.Qt.Key_W)
    _Check(controller.Tool() == gizmoUI.TOOL_TRANSLATE, "W selects Move")
    d.Key(d.QtCore.Qt.Key_E)
    _Check(controller.Tool() == gizmoUI.TOOL_ROTATE, "E selects Rotate")
    d.Key(d.QtCore.Qt.Key_R)
    _Check(controller.Tool() == gizmoUI.TOOL_SCALE, "R selects Scale")
    d.Key(d.QtCore.Qt.Key_D)
    _Check(controller.Channels() == gizmoMath.CHANNELS_PIVOT,
           "D toggles Edit Pivot on")
    d.Key(d.QtCore.Qt.Key_D)
    _Check(controller.Channels() == gizmoMath.CHANNELS_POSE,
           "D toggles Edit Pivot off")
    size = controller.settings.manipulatorSize
    d.Key(d.QtCore.Qt.Key_Plus)
    _Check(controller.settings.manipulatorSize > size,
           "+ grows the manipulator: %s" % controller.settings.manipulatorSize)
    d.Key(d.QtCore.Qt.Key_Minus)
    _Check(abs(controller.settings.manipulatorSize - size) < 1e-6,
           "- shrinks it back to %s: %s"
           % (size, controller.settings.manipulatorSize))
    d.Key(d.QtCore.Qt.Key_Q)
    _Check(controller.Tool() == gizmoUI.TOOL_SELECT, "Q selects the Select "
           "tool")

    # Escape aborts a live drag without pushing an edit. usdview binds
    # Escape itself, so the gizmo has to win it only while a drag is live;
    # what is asserted here is that behaviour, not how it is delivered.
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    a, b = d.AxisPoints("x")
    controller.undoStack.Clear()
    v0 = tx.Get(frame)
    d.Press(_Lerp(a, b, 0.5))
    d.Move(_Lerp(a, b, 0.85))
    _Check(abs(tx.Get(frame) - v0) > 1e-6, "the drag is live and has moved")
    d.Key(d.QtCore.Qt.Key_Escape)
    _Check(not controller.IsDragging(), "Escape ended the drag")
    _Check(abs(tx.Get(frame) - v0) < 1e-9,
           "Escape restored avars:tx to %s: %s" % (v0, tx.Get(frame)))
    _Check(not controller.undoStack.CanUndo(),
           "an aborted drag pushed nothing onto the undo stack")

    # Redo aliases: Shift+Z (Maya) and Ctrl+Y both redo.
    d.DragAxis("x")
    v1 = tx.Get(frame)
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ShiftModifier)
    _Check(abs(tx.Get(frame) - v1) < 1e-9,
           "Shift+Z redoes: %s" % tx.Get(frame))
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    d.Key(d.QtCore.Qt.Key_Y, d.QtCore.Qt.ControlModifier)
    _Check(abs(tx.Get(frame) - v1) < 1e-9,
           "Ctrl+Y redoes: %s" % tx.Get(frame))
    controller.Undo()
    d.Pump()
    _Check(abs(tx.Get(frame) - v0) < 1e-9,
           "the control is back where it started: %s" % tx.Get(frame))

    # Undo and redo must survive RigExec -> Viewport Tools being turned
    # off. The edits stay on the stack, so the shortcuts that reach them
    # cannot live only on the toolbar: a hidden widget's actions do not
    # fire, and the artist would be left holding un-undoable drags.
    d.DragAxis("x")
    vHidden = tx.Get(frame)
    _Check(abs(vHidden - v0) > 1e-6, "the drag moved the control")
    controller.SetVisible(False)
    d.Pump()
    _Check(not controller.toolbar.isVisible(),
           "the toolbar is hidden")
    _Check(controller.undoStack.CanUndo(),
           "the edit is still on the stack with the toolbar hidden")
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    _Check(abs(tx.Get(frame) - v0) < 1e-9,
           "Ctrl+Z still undoes with the viewport tools hidden: %s"
           % tx.Get(frame))
    d.Key(d.QtCore.Qt.Key_Z,
          d.QtCore.Qt.ControlModifier | d.QtCore.Qt.ShiftModifier)
    _Check(abs(tx.Get(frame) - vHidden) < 1e-9,
           "and Ctrl+Shift+Z still redoes: %s" % tx.Get(frame))
    controller.SetVisible(True)
    d.Pump()
    controller.Undo()
    d.Pump()
    _Check(abs(tx.Get(frame) - v0) < 1e-9,
           "back to the start again: %s" % tx.Get(frame))

    # Preserve Children holds a child xform's world transform still.
    child = UsdGeom.Xform.Define(stage, XFORM + "/GizmoTestChild")
    UsdGeom.XformCommonAPI(child).SetTranslate(Gf.Vec3d(1, 2, 3))
    childPath = child.GetPath()
    childOpPath = childPath.AppendProperty("xformOp:translate")
    d.Select(XFORM)
    controller.settings.For(gizmoUI.TOOL_TRANSLATE).preserveChildren = True
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None
           and controller.Target().supportsPreserveChildren,
           "Preserve Children is available on a plain xform")
    cache = UsdGeom.XformCache(frame)
    childWorld = cache.GetLocalToWorldTransform(child.GetPrim())
    d.DragAxis("y")
    parent = session.GetAttributeAtPath(
        Sdf.Path(XFORM + ".xformOp:translate"))
    _Check(parent is not None, "the parent xform did move")
    cache.Clear()
    childAfter = cache.GetLocalToWorldTransform(child.GetPrim())
    _Check(all(abs(childAfter[r][c] - childWorld[r][c]) < 1e-5
               for r in range(4) for c in range(4)),
           "Preserve Children kept the child's world transform: %s -> %s"
           % (childWorld.ExtractTranslation(),
              childAfter.ExtractTranslation()))
    controller.settings.For(gizmoUI.TOOL_TRANSLATE).preserveChildren = False
    controller.Undo()
    d.Pump()
    # One drag is one undo step, compensated children included: the parent's
    # op spec is gone and the child is back on the translate it was defined
    # with, not left holding the value the compensation wrote.
    _Check(session.GetAttributeAtPath(
        Sdf.Path(XFORM + ".xformOp:translate")) is None,
        "the undo removed the parent's xformOp spec")
    _Check(stage.GetAttributeAtPath(childOpPath).Get(frame)
           == Gf.Vec3d(1, 2, 3),
           "the same undo step reverted the child's compensation: %s"
           % stage.GetAttributeAtPath(childOpPath).Get(frame))
    # Removing the prim is plain cleanup. It used to raise "Applying
    # predicate to invalid prim" out of OpenExec's resync handler
    # (pxr/exec/esfUsd/stageData.cpp) whenever a compiled evaluator was
    # attached; tests/python/test_rigexec_stage_edits.py pins that fix, and
    # an exception here is a real regression, not something to guard.
    stage.RemovePrim(childPath)
    d.Pump()
    _Check(not stage.GetPrimAtPath(childPath),
           "the test's temporary child prim is gone from the stage")

    # --- 7. The Select tool draws nothing and grabs nothing --------------
    controller.SetTool(gizmoUI.TOOL_SELECT)
    d.Pump()
    _Check(controller.HandleScreenPositions() == {},
           "the Select tool draws no handles: %s"
           % sorted(controller.HandleScreenPositions()))
    d.Select(CONTROL)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    a, b = d.AxisPoints("x")
    controller.SetTool(gizmoUI.TOOL_SELECT)
    d.Pump()
    d.Press(_Lerp(a, b, 0.5))
    _Check(not controller.IsDragging(),
           "a click where the arrow used to be is left to usdview's picker")
    d.Release(_Lerp(a, b, 0.5))

    shot = os.environ.get("RIGEXEC_GIZMO_SHOT")
    if shot:
        d.Select(CONTROL)
        controller.SetTool(gizmoUI.TOOL_TRANSLATE)
        d.Pump()
        d.view.window().grab().save(shot)

    print("RIGEXEC_GIZMO_OK translate/rotate/scale, undo/redo, default, "
          "pivot, xform, maya parity")
