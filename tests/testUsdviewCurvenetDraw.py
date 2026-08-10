#
# DRAW MODE, end to end: does a click in the viewport actually place a knot?
#
# This sends real QMouseEvents down the same path a user's click takes, and
# it exists because the headless authoring test cannot see this class of bug
# at all -- it exercises the stage edits directly, never the viewport.
#
# The specific failure it was written for: usdview's own mousePressEvent
# multiplies by devicePixelRatioF before picking ("only necessary because
# this is a QGLWidget", stageView.py), because computePickFrustum divides by
# a PHYSICAL-pixel viewport while Qt reports LOGICAL pixels. Omitting that
# costs nothing at ratio 1.0 -- which is what a headless test runs at -- and
# on a HiDPI display sends every pick to a fraction of where the artist
# clicked, so placing a knot silently does nothing. The ratio assertion below
# is therefore written to hold at ANY ratio, including 1.0.
#


def _MakeMouseEvent(QtCore, QtGui, view, kind, x, y, modifiers=None):
    pos = QtCore.QPointF(float(x), float(y))
    glob = view.mapToGlobal(QtCore.QPoint(int(x), int(y)))
    globF = QtCore.QPointF(float(glob.x()), float(glob.y()))
    if modifiers is None:
        modifiers = QtCore.Qt.NoModifier
    return QtGui.QMouseEvent(kind, pos, globF, QtCore.Qt.LeftButton,
                             QtCore.Qt.LeftButton, modifiers)


def testUsdviewInputFunction(appController):
    import curvenetUI
    from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

    api = appController._usdviewApi
    stage = api.stage
    panel = curvenetUI.CurvenetPanel.GetInstance(api)
    net = stage.GetPrimAtPath(panel._curvenetPath)
    view = panel._picker.StageView()
    if view is None:
        raise AssertionError("no stage view")

    ratio = view.devicePixelRatioF()

    # 1. The coordinate convention, asserted independently of this machine's
    #    ratio: _Position must scale logical -> physical the way usdview does.
    probe = _MakeMouseEvent(QtCore, QtGui, view,
                            QtCore.QEvent.Type.MouseButtonPress, 100, 50)
    got = panel._Position(probe)
    want = (int(round(100 * ratio)), int(round(50 * ratio)))
    if got != want:
        raise AssertionError(
            "_Position returned %s, expected %s at devicePixelRatio %.3f -- "
            "pick coordinates must be physical pixels, as usdview's own "
            "mousePressEvent computes them" % (got, want, ratio))

    # 2. Frame the body and find a viewport point that actually covers it.
    body = stage.GetPrimAtPath("/puppetA/root/body_geo/node_0_Retopology")
    api.ClearPrimSelection()
    api.AddPrimToSelection(body)
    appController._frameSelection()
    appController._processEvents()

    width, height = api.viewportSize
    target = None
    for row in range(1, 10):
        for column in range(1, 10):
            x = int(width * column / 10.0)
            y = int(height * row / 10.0)
            if panel._picker.Pick(int(x * ratio), int(y * ratio)) is not None:
                target = (x, y)
                break
        if target:
            break
    if target is None:
        raise AssertionError(
            "no point on a 9x9 grid over a framed puppet picks the model")

    # 3. Draw mode: a press must place a knot.
    panel._SetMode(curvenetUI.MODE_DRAW)
    if not panel._filterInstalled:
        raise AssertionError("draw mode did not install the event filter")

    before = len(curvenetUI.GetPoints(net))
    QtWidgets.QApplication.sendEvent(
        view, _MakeMouseEvent(QtCore, QtGui, view,
                              QtCore.QEvent.Type.MouseButtonPress, *target))
    appController._processEvents()
    after = len(curvenetUI.GetPoints(net))
    if after != before + 1:
        raise AssertionError(
            "a left-press on the model added no knot (points %d -> %d, "
            "status %r)" % (before, after, panel._status.text()))

    # 4. A second press must connect the two with a spline.
    splinesBefore = len(curvenetUI.GetSplines(net)) // 4
    QtWidgets.QApplication.sendEvent(
        view, _MakeMouseEvent(QtCore, QtGui, view,
                              QtCore.QEvent.Type.MouseButtonPress,
                              target[0] + 10, target[1] + 10))
    appController._processEvents()
    splinesAfter = len(curvenetUI.GetSplines(net)) // 4
    if splinesAfter != splinesBefore + 1:
        raise AssertionError(
            "a second press did not create a spline (%d -> %d, status %r)"
            % (splinesBefore, splinesAfter, panel._status.text()))

    # 5. Alt must reach usdview so the artist can still orbit while drawing.
    pointsBefore = len(curvenetUI.GetPoints(net))
    consumed = panel.eventFilter(
        view, _MakeMouseEvent(QtCore, QtGui, view,
                              QtCore.QEvent.Type.MouseButtonPress,
                              target[0], target[1],
                              QtCore.Qt.AltModifier))
    if consumed:
        raise AssertionError(
            "draw mode swallowed an Alt-click; usdview claims Alt for camera "
            "manipulation and the artist could not orbit while drawing")
    if len(curvenetUI.GetPoints(net)) != pointsBefore:
        raise AssertionError("an Alt-click placed a knot")

    # 6. A click that MISSES must say so rather than appear to do nothing.
    panel._SetStatus("")
    missed = False
    for row in range(1, 10):
        x, y = int(width * 0.02), int(height * row / 10.0)
        if panel._picker.Pick(int(x * ratio), int(y * ratio)) is None:
            panel.eventFilter(
                view, _MakeMouseEvent(QtCore, QtGui, view,
                                      QtCore.QEvent.Type.MouseButtonPress,
                                      x, y))
            missed = True
            break
    if missed and "off the model" not in panel._status.text():
        raise AssertionError(
            "a click off the model reported %r; silence there is what makes "
            "the tool look broken" % panel._status.text())

    print("RIGEXEC_CURVENET_DRAW_OK ratio=%.3f target=%s points now %d"
          % (ratio, target, len(curvenetUI.GetPoints(net))))
