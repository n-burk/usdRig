"""testusdview script: rename a prim by double-clicking its node in the
Noodles Editor, inside a real usdview.

Two things had to be true for that to work, and neither is visible to the
unit tests in plugin/usdNoodles/testenv:

  * usdview has to be running THIS editor. An OpenUSD built with noodles
    installs an older copy as pxr.UsdNoodles, which has no rename at all; the
    in-repo UsdNoodles takes its place (_supersedeInstalledCopy). The runner
    registers both on purpose.
  * after the rename, the prim browser has to follow. Selecting the new path
    before usdview rebuilds its browser failed inside usdview's own slot
    (AttributeError in PrimTreeWidget.updateSelection) and left the old
    selection behind.

Asserts: plugins loaded; the Noodles Editor command is the in-repo one; a
double-click on the node opens the inline editor seeded with the prim name;
typing a name and Return renames the prim and its children; usdview's
selection and prim browser are on the new path. Prints
RIGEXEC_NOODLES_RENAME_OK.

The editor is closed before returning: a Noodles GL widget left open while
testusdview tears the app down reports GL errors from usdview's own
viewport, which testusdview treats as a failure.
"""
import os


def _pump(app, n=20):
    for _ in range(n):
        app.processEvents()


def testUsdviewInputFunction(appController):
    import sys

    from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

    app = QtWidgets.QApplication.instance()

    registry = appController._plugRegistry
    assert registry is not None, (
        "usdview loaded no plugins -- a duplicate command name?")
    command = registry.getCommandPlugin("NoodlesPluginContainer.ShowNoodlesEditor")
    assert command is not None, "no Noodles Editor command"
    assert command._callback.__module__.startswith("UsdNoodles."), (
        "the Noodles Editor is %s, not the in-repo UsdNoodles"
        % command._callback.__module__)
    installed = sys.modules.get("pxr.UsdNoodles")
    assert installed is None or getattr(installed, "SUPERSEDED_BY", None), (
        "the installed pxr.UsdNoodles was imported: %s"
        % getattr(installed, "__file__", installed))

    command.run()
    _pump(app, 50)

    from UsdNoodles.graphView import GraphView

    views = [w for w in app.allWidgets() if isinstance(w, GraphView)]
    assert views, "no GraphView after opening the Noodles Editor"
    view = views[0]
    _pump(app, 50)

    stage = appController._dataModel.stage
    nodeId = "/World"

    # The editor opens on an empty canvas (GraphView.loadStage); prims become
    # nodes through 'A' on a prim-tree selection, as they do for a user.
    from pxr import Sdf
    selection = appController._dataModel.selection
    selection.clearPrims()
    selection.addPrim(stage.GetPrimAtPath(Sdf.Path(nodeId)))
    _pump(app, 10)
    view.addNodesFromPrimTreeSelection()
    _pump(app, 40)
    assert nodeId in view.nodes, sorted(view.nodes)

    # Aim at the title: the top of the node, horizontally centred.
    node = view.nodes[nodeId]
    worldX = float(node.position[0]) + float(node.size[0]) * 0.5
    worldY = float(node.position[1]) + 8.0
    point = QtCore.QPointF(
        (worldX - view.panX) * view.zoom, (worldY - view.panY) * view.zoom)

    def mouse(kind):
        buttons = (QtCore.Qt.NoButton if kind == QtCore.QEvent.MouseButtonRelease
                   else QtCore.Qt.LeftButton)
        app.sendEvent(view, QtGui.QMouseEvent(
            kind, point, view.mapToGlobal(point.toPoint()),
            QtCore.Qt.LeftButton, buttons, QtCore.Qt.NoModifier))
        _pump(app, 2)

    def key(code, text=""):
        app.sendEvent(view, QtGui.QKeyEvent(
            QtCore.QEvent.KeyPress, code, QtCore.Qt.NoModifier, text))
        _pump(app, 2)

    # What Qt delivers for a double click.
    mouse(QtCore.QEvent.MouseButtonPress)
    mouse(QtCore.QEvent.MouseButtonRelease)
    mouse(QtCore.QEvent.MouseButtonDblClick)
    mouse(QtCore.QEvent.MouseButtonRelease)
    assert view._renamingNodeId == nodeId, (
        "double-click did not open the editor (renaming %r)" % view._renamingNodeId)
    assert view._renameInput.text == "World", view._renameInput.text

    shot = os.environ.get("RIGEXEC_NOODLES_SHOT")
    if shot:
        view.repaint()
        _pump(app, 5)
        view.grabFramebuffer().save(shot)

    for _ in range(len(view._renameInput.text)):
        key(QtCore.Qt.Key_Backspace)
    for ch in "Stage":
        key(getattr(QtCore.Qt, "Key_" + ch.upper()), ch)
    key(QtCore.Qt.Key_Return)
    _pump(app, 40)

    assert view._renamingNodeId == "", "the editor is still open"
    assert stage.GetPrimAtPath("/Stage/Box"), "the prim was not renamed"
    assert not stage.GetPrimAtPath("/World"), "the old prim is still there"
    assert "/Stage" in view.nodes, sorted(view.nodes)

    selected = [str(p) for p in appController._dataModel.selection.getPrimPaths()]
    assert selected == ["/Stage"], selected
    item = appController._getItemAtPath("/Stage")
    assert item is not None and item.isSelected(), (
        "the prim browser did not follow the rename")

    for widget in list(app.topLevelWidgets()):
        if type(widget).__module__.startswith("UsdNoodles"):
            widget.close()
    view.close()
    _pump(app, 20)
    print("RIGEXEC_NOODLES_RENAME_OK")
