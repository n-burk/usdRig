#
# THE VIEW CUBE, end to end.
#
# The headless tests in tests/python call viewCubeMath directly and never
# go near a camera, so they cannot see the one thing a view cube is -- a
# PROJECTION that a mouse has to be able to hit. This script opens
# examples/ArmShotAnim.usda (the container installs the cube when the
# stage loads) and drives synthetic mouse events through the cube's own
# projected region points. Every assertion is about what the camera does
# afterwards, not about what the controller thinks it did.
#
# Set RIGEXEC_VIEWCUBE_SHOT=/path.png to save a window grab.
#
import os

from pxr import Gf, Sdf, UsdGeom


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-3):
    return all(abs(a[i] - b[i]) <= tol for i in range(3))


def _Mouse(QtCore, QtGui, widget, kind, x, y, button=None,
           buttons=None, modifiers=None):
    """
    One synthetic mouse event in the cube widget's LOGICAL pixels, which
    is what ScreenPointForRegion() reports and what Qt delivers.
    """
    pos = QtCore.QPointF(float(x), float(y))
    glob = widget.mapToGlobal(QtCore.QPoint(int(x), int(y)))
    globF = QtCore.QPointF(float(glob.x()), float(glob.y()))
    if button is None:
        button = QtCore.Qt.LeftButton
    if buttons is None:
        buttons = button
    if modifiers is None:
        modifiers = QtCore.Qt.NoModifier
    return QtGui.QMouseEvent(kind, pos, globF, button, buttons,
                             modifiers)


def _Lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


class _Driver(object):
    """Mouse and camera reads, in the units the controller uses."""

    def __init__(self, appController, controller):
        from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets
        import viewCubeUI
        self.QtCore, self.QtGui, self.QtWidgets = QtCore, QtGui, \
            QtWidgets
        self.app = appController
        self.api = appController._usdviewApi
        self.controller = controller
        self.view = viewCubeUI.StageView(self.api)
        self.widget = controller.Widget()

    def Pump(self):
        self.app._processEvents()

    def PointFor(self, name):
        pt = self.controller.ScreenPointForRegion(name)
        _Check(pt is not None,
               "region '%s' is not on a visible face from the "
               "current view (theta, phi = %s)"
               % (name, self.controller.CurrentAngles()))
        return pt

    def Send(self, kind, x, y, **kw):
        self.QtWidgets.QApplication.sendEvent(
            self.widget, _Mouse(self.QtCore, self.QtGui, self.widget,
                                kind, x, y, **kw))
        self.Pump()

    def Press(self, point, **kw):
        self.Send(self.QtCore.QEvent.Type.MouseButtonPress, *point,
                  **kw)

    def Move(self, point, **kw):
        self.Send(self.QtCore.QEvent.Type.MouseMove, *point, **kw)

    def Release(self, point, button=None):
        self.Send(self.QtCore.QEvent.Type.MouseButtonRelease, *point,
                  button=button, buttons=self.QtCore.Qt.NoButton)

    def Click(self, point):
        self.Press(point)
        self.Release(point, button=self.QtCore.Qt.LeftButton)

    def Drag(self, start, end, steps=4):
        self.Press(start)
        for i in range(1, steps + 1):
            self.Move(_Lerp(start, end, i / float(steps)),
                      button=self.QtCore.Qt.NoButton,
                      buttons=self.QtCore.Qt.LeftButton)
        self.Release(end, button=self.QtCore.Qt.LeftButton)

    def Hover(self, point):
        self.Move(point, button=self.QtCore.Qt.NoButton,
                  buttons=self.QtCore.Qt.NoButton)

    def ViewDirection(self):
        return self.view.resolveCamera()[0].frustum.\
            ComputeViewDirection()

    def UpVector(self):
        return self.view.resolveCamera()[0].frustum.ComputeUpVector()

    def Angles(self):
        cam = self.api.dataModel.viewSettings.freeCamera
        return (cam.rotTheta, cam.rotPhi)

    def WaitAnimation(self, timeoutMs=3000):
        import time
        deadline = time.time() + timeoutMs / 1000.0
        seen = []
        while self.controller.IsAnimating():
            self.Pump()
            seen.append(self.Angles()[0])
            if time.time() > deadline:
                _Check(False, "the orbit animation did not "
                       "converge within %d ms" % timeoutMs)
        self.Pump()
        return seen


def testUsdviewInputFunction(appController):
    import viewCubeMath as vm
    import viewCubeUI
    import gizmoUI

    appController._processEvents()
    controller = viewCubeUI.GetController()
    _Check(controller is not None,
           "the container installed the view cube on stage load")
    d = _Driver(appController, controller)
    stage = d.api.stage
    session = stage.GetSessionLayer()
    _Check(stage.GetEditTarget().GetLayer() == session,
           "usdview's edit target is the session layer, which is where "
           "section 7 authors its camera prim")
    _Check(not controller.IsZUp(),
           "ArmShotAnim.usda is Y-up, which is what every direction "
           "below is written in")
    viewSettings = d.api.dataModel.viewSettings

    # --- 1. installed: on the stage view, top-right --------------------
    widget = controller.Widget()
    _Check(widget is not None, "the cube has a widget")
    _Check(widget.parentWidget() is gizmoUI.StageView(d.api),
           "the cube is a child of the stage view")
    _Check(widget.isVisible(), "the cube is visible on stage load")
    # The rendered viewport stays 597x540 whatever this says:
    # testusdview pins it with SetPhysicalWindowSize for reproducible
    # test images. Done here so the layout settles before anything is
    # placed or projected.
    appController._mainWindow.resize(1800, 1000)
    d.Pump()
    g = widget.geometry()
    _Check(g.x() + g.width() == d.view.width() - int(vm.MARGIN)
           and g.y() == int(vm.MARGIN),
           "the cube sits MARGIN px off the viewport's top-right: "
           "geometry %s on a %sx%s view"
           % (g, d.view.width(), d.view.height()))

    # --- 2. face clicks orbit the free camera --------------------------
    controller.animationMs = 0
    d.Click(d.PointFor("front"))
    _Check(viewSettings.cameraPrim is None,
           "clicking a face lands on the free camera")
    _Check(_Close(d.ViewDirection(), Gf.Vec3d(0, 0, -1)),
           "FRONT faces the stage head-on: %s" % d.ViewDirection())
    _Check(_Close(d.UpVector(), Gf.Vec3d(0, 1, 0)),
           "FRONT keeps world up: %s" % d.UpVector())
    cam = viewSettings.freeCamera
    center0, dist0 = Gf.Vec3d(cam.center), cam.dist

    def _CheckKept():
        _Check(_Close(cam.center, center0, 1e-9)
               and abs(cam.dist - dist0) < 1e-9,
               "clicks orbit in place: centre %s (was %s), dist %s "
               "(was %s)" % (cam.center, center0, cam.dist, dist0))

    d.Click(d.PointFor("front-right"))
    _Check(_Close(d.ViewDirection(),
                  Gf.Vec3d(-1, 0, -1).GetNormalized()),
           "the FRONT-RIGHT edge splits the difference: %s"
           % d.ViewDirection())
    _CheckKept()
    d.Click(d.PointFor("right"))
    _Check(_Close(d.ViewDirection(), Gf.Vec3d(-1, 0, 0)),
           "RIGHT looks down -X: %s" % d.ViewDirection())
    _Check(_Close(d.UpVector(), Gf.Vec3d(0, 1, 0)),
           "RIGHT keeps world up: %s" % d.UpVector())
    _CheckKept()
    # The plan writes "right-top" here, but the canonical name is
    # "top-right" (faces sort front|back, top|bottom, left|right, so
    # "right-top" is not in REGIONS at all); same faces, same orbit.
    d.Click(d.PointFor("top-right"))
    _Check(_Close(d.ViewDirection(),
                  Gf.Vec3d(-1, -1, 0).GetNormalized()),
           "the TOP-RIGHT edge looks down from halfway up: %s"
           % d.ViewDirection())
    _CheckKept()
    d.Click(d.PointFor("top"))
    _Check(_Close(d.ViewDirection(), Gf.Vec3d(0, -1, 0)),
           "TOP looks straight down: %s" % d.ViewDirection())
    _Check(any(_Close(d.UpVector(), axis) for axis in
               (Gf.Vec3d(1, 0, 0), Gf.Vec3d(-1, 0, 0),
                Gf.Vec3d(0, 0, 1), Gf.Vec3d(0, 0, -1))),
           "TOP snaps its heading to a multiple of 90: up %s"
           % d.UpVector())
    _CheckKept()
    # From TOP only the top face is visible, so the corner is clicked
    # through its corner square on the top face.
    d.Click(d.PointFor("front-top-right"))
    _Check(_Close(d.ViewDirection(),
                  Gf.Vec3d(-1, -1, -1).GetNormalized()),
           "the FRONT-TOP-RIGHT corner looks down the diagonal: %s"
           % d.ViewDirection())
    _CheckKept()
    # From the corner view the bottom face is hidden, so the edge is
    # clicked through its strip on the still-visible front face --
    # never through ScreenPointForRegion("bottom"), which is None.
    _Check(controller.ScreenPointForRegion("bottom") is None,
           "BOTTOM has no point while its face is hidden")
    d.Click(d.PointFor("front-bottom"))
    _Check(_Close(d.ViewDirection(),
                  Gf.Vec3d(0, 1, -1).GetNormalized()),
           "the FRONT-BOTTOM edge looks up from below: %s"
           % d.ViewDirection())
    _Check(_Close(d.UpVector(),
                  Gf.Vec3d(0, 1, 1).GetNormalized()),
           "the FRONT-BOTTOM edge tilts world up back: %s"
           % d.UpVector())
    _CheckKept()
    d.Click(d.PointFor("bottom"))
    _Check(_Close(d.ViewDirection(), Gf.Vec3d(0, 1, 0)),
           "BOTTOM looks straight up: %s" % d.ViewDirection())
    _Check(_Close(d.UpVector(), Gf.Vec3d(0, 0, 1)),
           "BOTTOM snaps its heading from 0: up %s" % d.UpVector())
    _CheckKept()

    # --- 3. hover pre-highlights, then clears on leave -----------------
    # Section 2 left the camera at BOTTOM, from which only the bottom
    # face is visible.
    controller.Orbit("front")
    d.Pump()
    frontPt = d.PointFor("front")
    d.Hover(frontPt)
    _Check(controller.HoverRegion() == "front",
           "hovering the FRONT face reports it: %s"
           % controller.HoverRegion())
    # The orbit moved the camera under a still cursor: the hover
    # follows the pixel (RefreshHover on every basis change), not
    # the stale region.
    controller.Orbit("back")
    d.Pump()
    want = controller.RegionAt(frontPt)
    wantName = want.name if want is not None else None
    _Check(wantName != "front",
           "the orbit moved a different face under the pixel")
    _Check(controller.HoverRegion() == wantName,
           "the hover follows the pixel after the orbit: hover %s, "
           "pixel now covers %s"
           % (controller.HoverRegion(), wantName))
    controller.Orbit("front")
    d.Pump()
    d.Hover(controller.HomePoint())
    _Check(controller.HoverRegion() == "home",
           "hovering the house reports home: %s"
           % controller.HoverRegion())
    # Arm a face region, not the glyph, so the checks below
    # pin the region hit test (they fail with a region name,
    # not "home", when mutated).
    d.Hover(d.PointFor("front"))
    d.QtWidgets.QApplication.sendEvent(
        widget, d.QtCore.QEvent(d.QtCore.QEvent.Type.Leave))
    d.Pump()
    _Check(controller.HoverRegion() is None,
           "leaving the cube clears the highlight: %s"
           % controller.HoverRegion())
    # A basis change must not resurrect the hover from the stale
    # point: the Leave cleared _lastPoint, so there is no pixel to
    # follow any more.
    controller.Orbit("top")
    d.Pump()
    controller._RefreshBasis()
    _Check(controller.HoverRegion() is None,
           "a basis change after the cursor left does not resurrect "
           "the hover: %s" % controller.HoverRegion())
    _Check(widget.cursor().shape() == d.QtCore.Qt.ArrowCursor,
           "and the pointing hand is gone too")
    # Section 4 orbits to BACK from wherever it starts, but leave
    # the camera where this section found it anyway.
    controller.Orbit("front")
    d.Pump()

    # --- 4. the orbit animates, then converges -------------------------
    controller.animationMs = 200
    theta0 = d.Angles()[0]
    controller.Orbit("back")
    _Check(controller.IsAnimating(),
           "a 200 ms orbit is still flying just after it starts")
    seen = d.WaitAnimation()
    theta1 = d.Angles()[0]
    lo, hi = sorted((theta0, theta1))
    _Check(any(lo + 1.0 < t < hi - 1.0 for t in seen),
           "the orbit jumped to BACK instead of interpolating: "
           "theta %s -> %s, no pose in between (%d samples)"
           % (theta0, theta1, len(seen)))
    _Check(_Close(d.ViewDirection(), Gf.Vec3d(0, 0, 1)),
           "the animated orbit lands on BACK: %s" % d.ViewDirection())
    controller.animationMs = 0

    # --- 5. dragging tumbles by usdview's rate, never orbits -----------
    # Section 4 left the camera at BACK.
    controller.Orbit("front")
    d.Pump()
    cam = viewSettings.freeCamera
    theta0, phi0 = d.Angles()
    ratio = d.view.devicePixelRatioF()
    c = controller.CubeCentre()
    d.Drag(c, (c[0] + 40.0, c[1]))
    _Check(abs(cam.rotTheta - theta0 - 0.25 * 40.0 * ratio) < 0.5,
           "the drag tumbled %.3f deg, not 0.25 * 40 * ratio %s "
           "(theta %s -> %s)"
           % (cam.rotTheta - theta0, ratio, theta0, cam.rotTheta))
    _Check(abs(cam.rotPhi - phi0) < 1e-9,
           "the horizontal drag left the elevation alone: %s -> %s"
           % (phi0, cam.rotPhi))
    # Again in 2 px steps, so the first move lands inside
    # DRAG_THRESHOLD: the dead zone is applied, not swallowed,
    # and the total is the same whatever the number of moves.
    theta1 = cam.rotTheta
    d.Drag(c, (c[0] + 40.0, c[1]), steps=20)
    _Check(abs(cam.rotTheta - theta1 - 0.25 * 40.0 * ratio) < 0.5,
           "the 20-step drag tumbled %.3f deg, not 0.25 * 40 * "
           "ratio %s: the dead zone was swallowed, not applied"
           % (cam.rotTheta - theta1, ratio))
    viewDir = Gf.Vec3d(d.ViewDirection()).GetNormalized()
    for name, region in sorted(vm.REGIONS.items()):
        want = Gf.Vec3d(-region.direction).GetNormalized()
        _Check(not _Close(viewDir, want),
               "a drag is a tumble, not an orbit: it landed on %s"
               % name)
    controller.Orbit("front")
    d.Pump()
    p = d.PointFor("front-right")
    d.Press(p)
    d.Move((p[0] + 2.0, p[1]), button=d.QtCore.Qt.NoButton,
           buttons=d.QtCore.Qt.LeftButton)
    d.Release((p[0] + 2.0, p[1]),
              button=d.QtCore.Qt.LeftButton)
    _Check(_Close(d.ViewDirection(),
                  Gf.Vec3d(-1, 0, -1).GetNormalized()),
           "a 2 px wiggle inside the dead zone is still a click: "
           "%s" % d.ViewDirection())

    # --- 6. an Alt-press belongs to usdview, not the cube --------------
    controller.Orbit("front")
    d.Pump()
    pressAt = d.PointFor("front")
    press = _Mouse(d.QtCore, d.QtGui, widget,
                   d.QtCore.QEvent.Type.MouseButtonPress, *pressAt,
                   button=d.QtCore.Qt.LeftButton,
                   buttons=d.QtCore.Qt.LeftButton,
                   modifiers=d.QtCore.Qt.AltModifier)
    widget.event(press)
    _Check(not press.isAccepted(),
           "the cube ignored the Alt-press (it is usdview's)")
    press2 = _Mouse(d.QtCore, d.QtGui, widget,
                    d.QtCore.QEvent.Type.MouseButtonPress, *pressAt,
                    button=d.QtCore.Qt.LeftButton,
                    buttons=d.QtCore.Qt.LeftButton,
                    modifiers=d.QtCore.Qt.AltModifier)
    d.QtWidgets.QApplication.sendEvent(widget, press2)
    d.Pump()
    _Check(d.view._cameraMode == "tumble"
           and d.view._dragActive is True,
           "the ignored press reached StageView.mousePressEvent "
           "(mode %s, drag %s)"
           % (d.view._cameraMode, d.view._dragActive))
    ratioAlt = d.view.devicePixelRatioF()
    thetaBefore = d.Angles()[0]
    move = _Mouse(d.QtCore, d.QtGui, widget,
                  d.QtCore.QEvent.Type.MouseMove,
                  pressAt[0] + 30.0, pressAt[1],
                  button=d.QtCore.Qt.NoButton,
                  buttons=d.QtCore.Qt.LeftButton,
                  modifiers=d.QtCore.Qt.AltModifier)
    d.QtWidgets.QApplication.sendEvent(widget, move)
    d.Pump()
    _Check(abs(d.Angles()[0] - thetaBefore
               - 0.25 * 30.0 * ratioAlt) < 0.5,
           "the ignored Alt move reached "
           "StageView.mouseMoveEvent, so usdview's own Alt-drag "
           "still tumbles over the cube: theta %s -> %s, "
           "wanted +%s"
           % (thetaBefore, d.Angles()[0],
              0.25 * 30.0 * ratioAlt))
    # Re-baseline: usdview's own Alt-drag above legitimately tumbled,
    # so "the Alt gesture orbited nothing" is asserted across the
    # release only.
    anglesBefore = d.Angles()
    movedAt = (pressAt[0] + 30.0, pressAt[1])
    release = _Mouse(d.QtCore, d.QtGui, widget,
                     d.QtCore.QEvent.Type.MouseButtonRelease,
                     *movedAt, button=d.QtCore.Qt.LeftButton,
                     buttons=d.QtCore.Qt.NoButton,
                     modifiers=d.QtCore.Qt.AltModifier)
    d.QtWidgets.QApplication.sendEvent(widget, release)
    d.Pump()
    _Check(d.view._cameraMode == "none"
           and d.view._dragActive is False,
           "the ignored release reached StageView.mouseReleaseEvent "
           "(mode %s, drag %s)"
           % (d.view._cameraMode, d.view._dragActive))
    region = controller.RegionAt(widget._lastPoint)
    _Check(d.Angles() == anglesBefore
           and controller.HoverRegion() ==
           (region.name if region is not None else None),
           "the Alt gesture tumbled usdview's way and left the "
           "hover on the pixel under the cursor: hover %s, "
           "pixel %s" % (controller.HoverRegion(), region))

    # --- 6b. the empty widget background is usdview's ------------------
    bg = None
    for gy in range(int(vm.WIDGET_SIZE)):
        for gx in range(int(vm.WIDGET_SIZE)):
            q = (gx + 0.5, gy + 0.5)
            if controller.RegionAt(q) is None and \
                    not controller.HomeRect().contains(
                        d.QtCore.QPointF(q[0], q[1])):
                bg = q
                break
        if bg is not None:
            break
    _Check(bg is not None,
           "the widget rect has a point that is neither cube nor "
           "home glyph")
    controller.animationMs = 600
    controller.Orbit("back")
    d.Pump()
    _Check(controller.IsAnimating(), "the 600 ms orbit is flying")
    d.view._cameraMode = "none"
    bgPress = _Mouse(d.QtCore, d.QtGui, widget,
                     d.QtCore.QEvent.Type.MouseButtonPress, bg[0],
                     bg[1],
                     button=d.QtCore.Qt.LeftButton,
                     buttons=d.QtCore.Qt.LeftButton)
    widget.event(bgPress)
    _Check(not bgPress.isAccepted(),
           "the cube ignored the background press (it is usdview's)")
    d.Press(bg)
    _Check(d.view._cameraMode == "pick",
           "the background press reached StageView's pick branch: %s"
           % d.view._cameraMode)
    _Check(controller.IsAnimating(),
           "a background press does not cancel a running orbit")
    d.Release(bg, button=d.QtCore.Qt.LeftButton)
    d.WaitAnimation()
    controller.animationMs = 0
    controller.Orbit("front")
    d.Pump()

    # --- 7. a camera prim orients the cube; a click leaves it ----------
    camPath = Sdf.Path("/ViewCubeTestCam")
    camPrim = UsdGeom.Camera.Define(stage, camPath)
    camPrim.AddTranslateOp().Set(Gf.Vec3d(0, 0, 50))
    # ZYX, so the 30 deg Z is a camera-space ROLL applied
    # before the yaw: FromGfCamera then decomposes it as
    # _rotPsi = -30, which is the only way a rolled free
    # camera arises (spec 1.5).
    camPrim.AddRotateZYXOp().Set(Gf.Vec3d(0, 90, 30))
    # The schema default focusDistance 0 would make FromGfCamera derive
    # dist = 0 (freeCamera.py:140-147).
    camPrim.CreateFocusDistanceAttr(50.0)
    viewSettings.cameraPrim = camPrim.GetPrim()
    d.Pump()
    primView = UsdGeom.Camera(camPrim.GetPrim()).GetCamera(
        d.api.frame).frustum.ComputeViewDirection()
    _Check(_Close(primView, Gf.Vec3d(-1, 0, 0)),
           "the test camera sits on +Z looking down -X: %s" % primView)
    _Check(_Close(controller.Basis().view, primView),
           "the cube shows the prim camera's orientation: %s vs %s"
           % (controller.Basis().view, primView))
    # Only the RIGHT face faces a camera looking down -X: FRONT is
    # edge-on to it.
    _Check(controller.ScreenPointForRegion("right") is not None,
           "RIGHT faces the prim camera")
    _Check(controller.ScreenPointForRegion("front") is None,
           "FRONT is edge-on to the prim camera, so it has no point")
    # Arm the stale heading while the FREE camera is active: writing
    # rotTheta with a prim camera on emits signalFreeCameraSettingChanged
    # and switchToFreeCamera rebuilds the free camera off the prim
    # (stageView.py:2044-2071, :2184-2188), erasing the arming.
    viewSettings.cameraPrim = None
    d.Pump()
    viewSettings.freeCamera.rotTheta = 140.0
    d.Pump()
    viewSettings.cameraPrim = camPrim.GetPrim()
    d.Pump()
    _Check(abs(viewSettings.freeCamera.rotTheta - 140.0) < 1e-9,
           "the free camera is the stale pre-prim one: theta %s"
           % viewSettings.freeCamera.rotTheta)
    controller.Orbit("top")
    d.Pump()
    _Check(_Close(d.UpVector(), Gf.Vec3d(-1, 0, 0)),
           "TOP snaps the PRIM camera's heading, not the stale "
           "free camera's: up %s" % d.UpVector())
    viewSettings.cameraPrim = camPrim.GetPrim()   # re-arm for RIGHT
    d.Pump()
    d.Click(d.PointFor("right"))
    _Check(viewSettings.cameraPrim is None,
           "clicking RIGHT switched back to the free camera")
    _Check(abs(viewSettings.freeCamera._rotPsi) < 1e-9,
           "the click did not clear the camera roll: _rotPsi %s"
           % viewSettings.freeCamera._rotPsi)
    _Check(_Close(d.ViewDirection(), Gf.Vec3d(-1, 0, 0)),
           "RIGHT looks down -X: %s" % d.ViewDirection())
    _Check(_Close(d.UpVector(), Gf.Vec3d(0, 1, 0)),
           "RIGHT keeps world up: %s" % d.UpVector())
    cam = viewSettings.freeCamera
    _Check(abs(cam.dist - 50.0) < 1e-6
           and _Close(cam.center, Gf.Vec3d(-50, 0, 50), 1e-6),
           "the free camera kept the prim's frame (centre %s, dist %s)"
           % (cam.center, cam.dist))
    stage.RemovePrim(camPath)
    d.Pump()

    # --- 8. home re-frames, then takes the home corner view ------------
    cam = viewSettings.freeCamera
    cam.center = Gf.Vec3d(123, 456, 789)
    cam.dist = 9999.0
    d.Pump()
    d.Click(controller.HomePoint())
    _Check(not controller.IsAnimating(),
           "the 0 ms home orbit is already there")
    _Check(_Close(d.ViewDirection(),
                  (-Gf.Vec3d(1, 1, 1).GetNormalized())),
           "home looks down the (1, 1, 1) diagonal: %s"
           % d.ViewDirection())
    _Check(_Close(cam.center,
                  d.view._selectionBBox.ComputeCentroid(), 1e-6)
           and 0.0 < cam.dist < 9999.0,
           "Home did not re-frame the selection: center %s dist %s"
           % (cam.center, cam.dist))

    # --- 9. visibility and registration --------------------------------
    # Arm a face region, not the home glyph left by section 8,
    # so the hidden-orbit check pins the region hit test.
    frontPt = d.PointFor("front")
    d.Hover(frontPt)
    controller.SetVisible(False)
    d.Pump()
    _Check(not widget.isVisible(), "SetVisible(False) hides the cube")
    # Pin the isVisible() guard on its own: re-arm the point
    # the hide cleared, so only the guard keeps the hidden
    # refresh from arming a highlight.
    widget._lastPoint = frontPt          # the hide cleared it
    controller.Orbit("top")
    d.Pump()
    controller._RefreshBasis()
    _Check(controller.HoverRegion() is None,
           "a hidden cube arms no highlight even with a last "
           "point: %s" % controller.HoverRegion())
    controller.SetVisible(True)
    d.Pump()
    _Check(widget.isVisible(), "SetVisible(True) shows it again")
    # Qt sends no synthetic move on show or enter, so the Enter
    # itself must arm the highlight from the real cursor, never
    # the event's position (a plain QEvent(Enter) carries none,
    # and QWidget::event static_casts it to QEnterEvent* before
    # dispatch, so reading position() segfaults instead).
    topPt = d.PointFor("top")
    d.QtGui.QCursor.setPos(widget.mapToGlobal(
        d.QtCore.QPoint(int(topPt[0]), int(topPt[1]))))
    d.Pump()  # let the cursor settle: pos() lags setPos()
    d.QtWidgets.QApplication.sendEvent(
        widget, d.QtCore.QEvent(d.QtCore.QEvent.Type.Enter))
    d.Pump()
    _Check(controller.HoverRegion() == "top",
           "entering the cube arms the highlight at once: %s"
           % controller.HoverRegion())
    d.QtWidgets.QApplication.sendEvent(
        widget, d.QtCore.QEvent(d.QtCore.QEvent.Type.Leave))
    d.Pump()
    reg = appController._plugRegistry
    _Check(reg.getCommandPlugin(
        "RigExecUsdviewContainer.viewCube") is not None,
        "RigExec -> View Cube command is not registered")
    menus = [c for c in
             appController._mainWindow.menuBar().children()
             if isinstance(c, d.QtWidgets.QMenu)
             and str(c.title()).replace("&", "") == "RigExec"]
    menus = [a.menu() for a in (menus[0].actions() if len(menus) == 1
                                else [])
             if a.menu() is not None and a.text() == "Viewport"]
    _Check(len(menus) == 1 and "View Cube" in
           [a.text() for a in menus[0].actions()],
           "RigExec -> Viewport has no View Cube item")

    # --- 10. grab -------------------------------------------------------
    shot = os.environ.get("RIGEXEC_VIEWCUBE_SHOT")
    if shot:
        controller.Orbit("front")
        d.Pump()
        d.view.window().grab().save(shot)

    print("RIGEXEC_VIEWCUBE_OK faces/edges/corners, hover, animation, "
          "drag, alt, camera prim, home, toggle")
