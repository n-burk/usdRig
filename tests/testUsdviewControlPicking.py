"""Real GPU and mouse picking of hairline controls, including occlusion."""
from pxr import Gf, Sdf
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets


def testUsdviewInputFunction(app):
    import gizmoUI
    view = app._stageView
    model = app._dataModel
    controller = gizmoUI.GetController(app._usdviewApi)
    controller.SetTool(gizmoUI.TOOL_SELECT)
    model.viewSettings.freeCamera.rotTheta = 0
    model.viewSettings.freeCamera.rotPhi = 0
    for _ in range(4):
        app._processEvents()
    camera, _ = view.resolveCamera()
    projection = camera.frustum.ComputeViewMatrix() * camera.frustum.ComputeProjectionMatrix()
    viewport = view.computeWindowViewport()
    ratio = view.devicePixelRatioF()

    def screen(point):
        p = projection.Transform(Gf.Vec3d(*point))
        return (viewport[0] + (p[0] + 1) * viewport[2] / 2,
                viewport[1] + (1 - p[1]) * viewport[3] / 2)

    def path(x, y):
        inside, frustum = view.computePickFrustum(x, y)
        assert inside
        hits = view.pick(frustum)
        return hits[0].hitPrimPath if hits else Sdf.Path.emptyPath

    x, y = screen((0, 1, 0))
    _, frustum = view.computePickFrustum(x, y + 3 * ratio)
    original = controller._controlPicking.original(frustum)
    assert original and original[0].hitPrimPath == Sdf.Path('/Rig/Surface')
    # A click near the curve must select it instead of the mesh beneath.
    for offset in (-3, 0, 3):
        assert path(x, y + offset * ratio) == Sdf.Path('/Rig/Wire')
    assert path(x, y + 12 * ratio) == Sdf.Path('/Rig/Surface')
    assert path(*screen((0, -1, -1))) == Sdf.Path('/Rig/Surface')
    model.viewSettings.displayGuide = False
    assert path(x, y) == Sdf.Path('/Rig/Surface')
    model.viewSettings.displayGuide = True

    model.selection.clearPrims()
    pos = QtCore.QPointF(x / ratio, y / ratio + 3)
    for kind, buttons in ((QtCore.QEvent.MouseButtonPress, QtCore.Qt.LeftButton),
                          (QtCore.QEvent.MouseButtonRelease, QtCore.Qt.NoButton)):
        event = QtGui.QMouseEvent(kind, pos, QtCore.QPointF(view.mapToGlobal(pos.toPoint())),
                                 QtCore.Qt.LeftButton, buttons, QtCore.Qt.NoModifier)
        QtWidgets.QApplication.sendEvent(view, event)
        app._processEvents()
    assert model.selection.getPrimPaths() == [Sdf.Path('/Rig/Wire')]
    # The same query follows the evaluated control after a channel edit.
    stage = model.stage
    stage.SetEditTarget(stage.GetSessionLayer())
    stage.GetPrimAtPath('/Rig/Wire').GetAttribute('avars:ty').Set(-0.5)
    for _ in range(4):
        app._processEvents()
    assert path(*screen((0, 0.5, 0))) == Sdf.Path('/Rig/Wire')
    assert path(x, y) == Sdf.Path('/Rig/Surface')
    print('RIGEXEC_CONTROL_PICKING_OK', flush=True)
