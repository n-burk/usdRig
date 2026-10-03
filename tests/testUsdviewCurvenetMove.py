#
# SELECT AND MOVE, end to end.
#
# Written because drawing worked while selecting did not, and the headless
# tests could not see the difference: they call the edit helpers directly and
# never go near a camera. Selection is the one operation that depends on
# PROJECTION -- matching a click to a knot on screen -- so it can only be
# tested with a real camera and real mouse events.
#
import math


def _Mouse(QtCore, QtGui, view, kind, x, y, buttons=None, modifiers=None):
    pos = QtCore.QPointF(float(x), float(y))
    glob = view.mapToGlobal(QtCore.QPoint(int(x), int(y)))
    globF = QtCore.QPointF(float(glob.x()), float(glob.y()))
    if buttons is None:
        buttons = QtCore.Qt.LeftButton
    if modifiers is None:
        modifiers = QtCore.Qt.NoModifier
    return QtGui.QMouseEvent(kind, pos, globF, QtCore.Qt.LeftButton,
                             buttons, modifiers)


def testUsdviewInputFunction(appController):
    import curvenetUI
    from pxr import Gf
    from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

    api = appController._usdviewApi
    stage = api.stage
    panel = curvenetUI.CurvenetPanel.GetInstance(api)
    net = stage.GetPrimAtPath(panel._curvenetPath)
    view = panel._picker.StageView()
    ratio = view.devicePixelRatioF()

    # Frame the NET, not the whole character: 114 control points spread over
    # a head that is a fifth of the body means every knot has a neighbour
    # within a few pixels at full-body framing, which is not a view anybody
    # would edit in anyway.
    api.ClearPrimSelection()
    api.AddPrimToSelection(net)
    appController._frameSelection()
    appController._processEvents()

    points = curvenetUI.GetPoints(net)
    topology = curvenetUI.Topology(len(points), curvenetUI.GetSplines(net))
    projected = panel._picker.Project(panel._WorldPoints())

    # 1. Project must agree with the pick frustum. Checked by PICKING first
    #    and projecting the result back, which isolates the projection: going
    #    the other way (project a knot, pick there) fails on any knot facing
    #    away, because the pick lands on the near surface instead.
    viewport = view.computeWindowViewport()
    checked = 0
    for row in range(2, 9):
        for column in range(2, 9):
            px = viewport[0] + viewport[2] * column / 10.0
            py = viewport[1] + viewport[3] * row / 10.0
            hit = panel._picker.Pick(int(px), int(py))
            if hit is None:
                continue
            back = panel._picker.Project([hit[1]])[0]
            if back is None:
                raise AssertionError("a picked point projected to nothing")
            error = math.hypot(back[0] - px, back[1] - py)
            if error > 1.5:
                raise AssertionError(
                    "pick at (%.1f, %.1f) projects back to (%.1f, %.1f), "
                    "%.2f px away -- Project does not match the pick frustum, "
                    "so screen-space selection is aiming at the wrong place"
                    % (px, py, back[0], back[1], error))
            checked += 1
    if checked < 5:
        raise AssertionError("only %d points round-tripped" % checked)

    # 2. Selecting a knot by clicking where it is drawn.
    #
    # The target has to be UNAMBIGUOUS: a net wraps right round the head, so
    # plenty of knots project on top of each other, and where two overlap the
    # nearer one legitimately wins. Requiring an isolated target is what makes
    # the assertion about selection rather than about which knot happens to
    # be in front.
    def onScreen(screen, margin=24):
        return (viewport[0] + margin < screen[0] <
                viewport[0] + viewport[2] - margin and
                viewport[1] + margin < screen[1] <
                viewport[1] + viewport[3] - margin)

    def isolated(index, screen, kinds, clearance):
        for other, otherScreen in enumerate(projected):
            if other == index or otherScreen is None:
                continue
            if kinds is not None and topology.kind[other] not in kinds:
                continue
            if math.hypot(otherScreen[0] - screen[0],
                          otherScreen[1] - screen[1]) < clearance:
                return False
        return True

    # The rule the panel documents: a KNOT beats a handle, then nearer the
    # camera, then nearer the cursor. Asserted directly rather than by
    # finding an isolated knot -- a ring viewed down its axis projects its
    # front and back knots onto the SAME pixel, so isolation does not exist
    # on this net from most angles.
    eye = view.resolveCamera()[0].frustum.GetPosition()
    world = panel._WorldPoints()
    limit = panel._PICK_RADIUS_PIXELS * ratio

    bucket = max(2.0 * ratio, 1.0)

    def expectedAt(px, py):
        best, bestRank = -1, None
        for i, s in enumerate(projected):
            if s is None:
                continue
            d = math.hypot(s[0] - px, s[1] - py)
            if d > limit:
                continue
            rank = (int(d / bucket),
                    1 if topology.kind[i] == curvenetUI.KIND_HANDLE else 0,
                    (world[i] - eye).GetLength(), d)
            if best < 0 or rank < bestRank:
                best, bestRank = i, rank
        return best

    target = None
    for index, screen in enumerate(projected):
        if screen is None or topology.kind[index] == curvenetUI.KIND_HANDLE:
            continue
        if onScreen(screen) and expectedAt(*screen) >= 0:
            target = (index, screen)
            break
    if target is None:
        raise AssertionError("no knot on screen to click")
    index, screen = target
    index = expectedAt(*screen)
    if topology.kind[index] == curvenetUI.KIND_HANDLE:
        raise AssertionError(
            "a handle outranked every knot under the cursor; knots are "
            "supposed to win that tie")

    panel._SetMode(curvenetUI.MODE_MOVE)
    logical = (screen[0] / ratio, screen[1] / ratio)
    QtWidgets.QApplication.sendEvent(
        view, _Mouse(QtCore, QtGui, view,
                     QtCore.QEvent.Type.MouseButtonPress, *logical))
    appController._processEvents()
    if panel._dragKnot != index:
        raise AssertionError(
            "clicking exactly where knot %d is drawn selected %d instead "
            "(status %r)" % (index, panel._dragKnot, panel._status.text()))

    # 3. Dragging must actually move it.
    before = Gf.Vec3d(curvenetUI.GetPoints(net)[index])
    moved = False
    for offset in (18, 30, 46, 64):
        QtWidgets.QApplication.sendEvent(
            view, _Mouse(QtCore, QtGui, view, QtCore.QEvent.Type.MouseMove,
                         logical[0] + offset, logical[1] + offset))
        appController._processEvents()
        if (Gf.Vec3d(curvenetUI.GetPoints(net)[index]) - before).GetLength() \
                > 1e-6:
            moved = True
            break
    if not moved:
        raise AssertionError(
            "a left-drag did not move the selected knot (status %r)"
            % panel._status.text())
    QtWidgets.QApplication.sendEvent(
        view, _Mouse(QtCore, QtGui, view,
                     QtCore.QEvent.Type.MouseButtonRelease, logical[0] + 64,
                     logical[1] + 64, buttons=QtCore.Qt.NoButton))
    appController._processEvents()
    if panel._dragKnot != -1:
        raise AssertionError("release did not end the drag")

    # 4. A click far from everything selects nothing, and says so.
    panel._SetStatus("")
    QtWidgets.QApplication.sendEvent(
        view, _Mouse(QtCore, QtGui, view,
                     QtCore.QEvent.Type.MouseButtonPress,
                     logical[0] + 200, logical[1] + 200))
    appController._processEvents()
    if panel._dragKnot != -1:
        raise AssertionError("a click 200px away still selected knot %d"
                             % panel._dragKnot)
    if "within" not in panel._status.text():
        raise AssertionError("a miss reported %r" % panel._status.text())

    # 5. Handles are selectable too, or a curve cannot be shaped.
    # Re-projected: the knot moved above, so the old positions are stale.
    projected = panel._picker.Project(panel._WorldPoints())
    world = panel._WorldPoints()
    handle = None
    for i, screen2 in enumerate(projected):
        if screen2 is None or topology.kind[i] != curvenetUI.KIND_HANDLE:
            continue
        # Only where no KNOT is under the cursor, since knots win the tie by
        # design; a handle is reachable exactly in the gaps between them.
        if onScreen(screen2) and expectedAt(*screen2) == i:
            handle = (i, screen2)
            break
    if handle is None:
        raise AssertionError(
            "no tangent handle is reachable: every one has a knot within the "
            "pick radius, so tangents could never be shaped")
    hIndex, hScreen = handle
    QtWidgets.QApplication.sendEvent(
        view, _Mouse(QtCore, QtGui, view,
                     QtCore.QEvent.Type.MouseButtonPress,
                     hScreen[0] / ratio, hScreen[1] / ratio))
    appController._processEvents()
    if panel._dragKnot != hIndex:
        raise AssertionError(
            "clicking a tangent handle selected %d, expected %d"
            % (panel._dragKnot, hIndex))

    print("RIGEXEC_CURVENET_MOVE_OK round-tripped %d knots, moved knot %d, "
          "selected handle %d" % (checked, index, hIndex))
