#
# Does what the artist draws actually REACH THE RENDERER?
#
# Checking the stage is not enough: the display prims were being authored
# correctly the whole time and still nothing appeared, because they carried
# purpose="guide" and usdview does not draw guides unless the viewer has
# switched them on. So this asserts against the TERMINAL SCENE INDEX -- the
# last thing before the render delegate -- which is as close to "it is on
# screen" as a headless test can get.
#
from pxr import Sdf


def _Observer():
    from pxr.Usdviewq._usdviewq import HydraObserver
    names = HydraObserver.GetRegisteredSceneIndexNames()
    if not names:
        raise AssertionError("no registered scene indices")
    observer = HydraObserver()
    observer.TargetToNamedSceneIndex(names[-1])
    return observer


def _PrimvarValue(dataSource, name):
    if not dataSource or "primvars" not in dataSource.GetNames():
        return None
    primvars = dataSource.Get("primvars")
    entry = primvars.Get(name)
    if not entry:
        return None
    value = entry.Get("primvarValue")
    return value.GetValue(0.0) if value else None


def _Require(observer, path, what):
    primType, dataSource = observer.GetPrim(path)
    if not dataSource:
        raise AssertionError("%s (%s) is not in the terminal scene index"
                             % (path, what))
    points = _PrimvarValue(dataSource, "points")
    if not points:
        raise AssertionError("%s (%s) reached the renderer with no points"
                             % (path, what))
    # Hydra carries the RENDER TAG here, not the USD purpose token: default
    # purpose arrives as "geometry", which is the one that draws with no
    # viewer setting. "guide", "proxy" and "render" all need the viewer to
    # have that purpose switched on -- which is exactly how an authoring aid
    # ends up invisible and looking broken.
    purpose = dataSource.Get("purpose")
    resolved = purpose.Get("purpose").GetValue(0.0) if purpose else None
    if resolved not in (None, "geometry", "default"):
        raise AssertionError(
            "%s (%s) reaches the renderer with render tag %r, which only "
            "draws when the viewer has that purpose enabled"
            % (path, what, resolved))
    return primType, len(points)


def testUsdviewInputFunction(appController):
    import curvenetUI
    from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

    api = appController._usdviewApi
    stage = api.stage
    panel = curvenetUI.CurvenetPanel.GetInstance(api)

    # A FRESH curvenet, the way an artist starts: New, then draw.
    parent = stage.GetPrimAtPath("/puppetA")
    net = curvenetUI.CreateCurvenet(stage, parent, "DrawnNet")
    panel._curvenetPath = net.GetPath()
    panel._Refresh()

    view = panel._picker.StageView()
    ratio = view.devicePixelRatioF()
    body = stage.GetPrimAtPath("/puppetA/root/body_geo/node_0_Retopology")
    api.ClearPrimSelection()
    api.AddPrimToSelection(body)
    appController._frameSelection()
    appController._processEvents()

    width, height = api.viewportSize
    target = None
    for row in range(1, 10):
        for column in range(1, 10):
            x, y = int(width * column / 10.0), int(height * row / 10.0)
            if panel._picker.Pick(int(x * ratio), int(y * ratio)) is not None:
                target = (x, y)
                break
        if target:
            break
    if target is None:
        raise AssertionError("nothing pickable")

    def click(x, y):
        pos = QtCore.QPointF(float(x), float(y))
        glob = view.mapToGlobal(QtCore.QPoint(int(x), int(y)))
        event = QtGui.QMouseEvent(
            QtCore.QEvent.Type.MouseButtonPress, pos,
            QtCore.QPointF(float(glob.x()), float(glob.y())),
            QtCore.Qt.LeftButton, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier)
        QtWidgets.QApplication.sendEvent(view, event)
        appController._processEvents()

    panel._SetMode(curvenetUI.MODE_DRAW)

    # ONE click. A single knot has no spline yet, and this is precisely the
    # moment the tool used to show nothing at all.
    click(*target)
    if len(curvenetUI.GetPoints(net)) != 1:
        raise AssertionError("first click placed no knot")
    appController._processEvents()
    observer = _Observer()
    kind, count = _Require(observer, panel._KnotDisplayPath(),
                           "knot markers after ONE click")
    print("  after 1 click: %s at %s with %d point(s)"
          % (kind, panel._KnotDisplayPath(), count))

    # Second click: a DIFFERENT knot, and therefore a spline.
    #
    # Asserted separately, because the way this failed was that the second
    # click welded to the first knot instead of placing a new one -- the
    # click-radius fell back to a stage-unit guess many times the size of the
    # whole character, so every click was "on" the knot already there.
    click(target[0] + 14, target[1] + 8)
    appController._processEvents()
    if len(curvenetUI.GetPoints(net)) < 2:
        raise AssertionError(
            "the second click did not place a second knot -- it welded to "
            "the first (click radius %g against a net of %d point(s))"
            % (panel._KnotTolerance(), len(curvenetUI.GetPoints(net))))
    if len(curvenetUI.GetSplines(net)) // 4 != 1:
        raise AssertionError(
            "two knots produced %d splines, expected 1"
            % (len(curvenetUI.GetSplines(net)) // 4))
    observer = _Observer()
    kind, count = _Require(observer, panel._DisplayPath(),
                           "spline curves after two clicks")
    print("  after 2 clicks: %s at %s with %d vertices"
          % (kind, panel._DisplayPath(), count))
    _Require(observer, panel._KnotDisplayPath(), "knot markers")

    # Toggling the checkbox HIDES rather than removes, and must not raise.
    #
    # Removing was the original design and it was wrong twice over: on a
    # stage carrying a RigExecRoot every prim add/remove resyncs and
    # re-evaluates the whole rig, and that re-evaluation posts a Tf error
    # which Python then raises out of whatever call comes next -- here, out
    # of the checkbox handler itself.
    from pxr import UsdGeom
    panel._showNet.setChecked(False)
    panel._UpdateDisplay()
    for label, p in (("curves", panel._DisplayPath()),
                     ("knots", panel._KnotDisplayPath())):
        prim = stage.GetPrimAtPath(p)
        if not prim:
            raise AssertionError("unchecking REMOVED the %s prim; it should "
                                 "only be hidden" % label)
        vis = UsdGeom.Imageable(prim).GetVisibilityAttr().Get()
        if vis != UsdGeom.Tokens.invisible:
            raise AssertionError("unchecking left %s visible (%s)"
                                 % (label, vis))
    # Idempotent: this used to throw the second time through.
    panel._UpdateDisplay()

    panel._showNet.setChecked(True)
    panel._UpdateDisplay()
    appController._processEvents()
    observer = _Observer()
    _Require(observer, panel._KnotDisplayPath(), "knots after re-checking")

    print("RIGEXEC_CURVENET_VISIBLE_OK")
