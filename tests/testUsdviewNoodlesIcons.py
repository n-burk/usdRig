"""testusdview script: node title-bar icons, from the authored attribute all
the way to the pixels, inside a real usdview.

What broke, and what this pins down:

  * ``NodeFactory._apply_icon_from_prim`` set a PYTHON attribute on the node
    after the pin setters had run their last ``_sync_content_to_cpp``, so the
    C++ ``NodeData.titleIconPath`` -- the only thing the icon producer reads --
    stayed empty and every node drew the built-in ``box`` icon no matter what
    ``ui:nodegraph:node:icon`` said. ``NodeModel._icon_path`` now writes
    straight through to that field, so the assertion here is on
    ``node.titleIconPath`` and not on any Python mirror.

  * the authored path is RELATIVE TO THE LAYER. Handing the renderer the
    authored string would make the icon depend on the process's working
    directory; it is anchored (USD's resolved path, or the layer's own
    ComputeAbsolutePath when the file does not exist).

  * a missing icon file must degrade to the default silently -- not raise, not
    stop the frame, and not leave a working-directory-relative path behind.

The pixel check is what makes this end to end: the title area is grabbed with
the authored icon and again with the icon cleared, and the two must differ. A
path that reaches C++ but never reaches the GPU would pass every other
assertion here.

Prints RIGEXEC_NOODLES_ICONS_OK.

The editor is closed before returning: a Noodles GL widget left open while
testusdview tears the app down reports GL errors from usdview's own viewport,
which testusdview treats as a failure.
"""
import os


ICONED = "/World/Iconed"
MISSING = "/World/MissingIcon"
NO_ICON = "/World/NoIcon"


def _pump(app, n=20):
    for _ in range(n):
        app.processEvents()


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _sameFile(a, b):
    return os.path.normcase(os.path.normpath(a)) == os.path.normcase(
        os.path.normpath(b)
    )


def testUsdviewInputFunction(appController):
    import sys

    from pxr import Sdf
    from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

    app = QtWidgets.QApplication.instance()

    registry = appController._plugRegistry
    _Check(registry is not None,
           "usdview loaded no plugins -- a duplicate command name?")
    command = registry.getCommandPlugin("NoodlesPluginContainer.ShowNoodlesEditor")
    _Check(command is not None, "no Noodles Editor command")
    _Check(command._callback.__module__.startswith("UsdNoodles."),
           "the Noodles Editor is %s, not the in-repo UsdNoodles"
           % command._callback.__module__)
    installed = sys.modules.get("pxr.UsdNoodles")
    _Check(installed is None or getattr(installed, "SUPERSEDED_BY", None),
           "the installed pxr.UsdNoodles was imported: %s"
           % getattr(installed, "__file__", installed))

    command.run()
    _pump(app, 50)

    from UsdNoodles.graphView import GraphView

    views = [w for w in app.allWidgets() if isinstance(w, GraphView)]
    _Check(bool(views), "no GraphView after opening the Noodles Editor")
    view = views[0]
    _pump(app, 30)

    stage = appController._dataModel.stage
    layerDir = os.path.dirname(stage.GetRootLayer().realPath)

    # --- 1. the three prims become nodes --------------------------------
    selection = appController._dataModel.selection
    selection.clearPrims()
    for path in (ICONED, MISSING, NO_ICON):
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        _Check(bool(prim) and prim.IsValid(), "the fixture has no %s" % path)
        selection.addPrim(prim)
    _pump(app, 10)

    view.addNodesFromPrimTreeSelection()
    _pump(app, 40)

    for path in (ICONED, MISSING, NO_ICON):
        _Check(path in view.nodes,
               "%s did not become a node: %s" % (path, sorted(view.nodes)))

    # --- 2. the authored icon reaches the field the renderer reads ------
    # titleIconPath IS the C++ NodeData field. Empty here is the original bug.
    iconPath = view.nodes[ICONED].titleIconPath
    _Check(bool(iconPath),
           "%s authors ui:nodegraph:node:icon but the node's titleIconPath is "
           "empty -- the icon never reached the C++ icon producer" % ICONED)

    expected = os.path.join(layerDir, "..", "..", "icons", "control.png")
    _Check(os.path.isabs(iconPath),
           "the icon path is not absolute (%r) -- a relative path would be "
           "resolved against the working directory, not the layer" % iconPath)
    _Check(_sameFile(iconPath, expected),
           "the icon resolved to %r, expected the layer-relative %r"
           % (iconPath, os.path.normpath(expected)))
    _Check(os.path.isfile(iconPath), "the resolved icon does not exist: %r"
           % iconPath)

    image = QtGui.QImage(iconPath)
    _Check(not image.isNull(),
           "the resolved icon is not a loadable image: %r" % iconPath)
    _Check(image.width() >= 16 and image.height() >= 16,
           "the icon is %dx%d -- too small to read on a title bar"
           % (image.width(), image.height()))

    # --- 3. a missing file anchors and then degrades silently -----------
    missingPath = view.nodes[MISSING].titleIconPath
    _Check(bool(missingPath),
           "%s authors an icon; the node kept no path at all" % MISSING)
    _Check(os.path.isabs(missingPath),
           "a missing icon left a working-directory-relative path: %r"
           % missingPath)
    _Check(_sameFile(missingPath,
                     os.path.join(layerDir, "..", "..", "icons",
                                  "no_such_icon.png")),
           "the missing icon anchored to %r" % missingPath)
    _Check(not os.path.isfile(missingPath),
           "the fixture's missing icon exists -- the test proves nothing: %r"
           % missingPath)

    # No opinion at all: the C++ producer's own default icon, not a path.
    _Check(view.nodes[NO_ICON].titleIconPath == "",
           "%s authors no icon but carries %r"
           % (NO_ICON, view.nodes[NO_ICON].titleIconPath))

    # --- 4. the icons are actually drawn --------------------------------
    node = view.nodes[ICONED]
    view.zoom = 1.0
    view.panX = float(node.position[0]) - 20.0
    view.panY = float(node.position[1]) - 20.0

    def repaint():
        view._clearRenderCache()
        view._cppIconRenderer.markIconsDirty()
        view.update()
        view.repaint()
        _pump(app, 10)
        return view.grabFramebuffer()

    _Check(view._cppIconRenderer.isInitialized,
           "the icon renderer never initialized")

    withIcon = repaint()
    _Check(not view._cppIconRenderer._cpp.needsIconRebuild(),
           "the icon buffer was never rebuilt -- no frame drew icons")

    # The title area of the node, in image pixels: the icon sits between the
    # fold caret and the title text.
    scale = float(withIcon.width()) / float(max(view.width(), 1))
    titleHeight = float(node.layoutTitleHeight) or 40.0

    def rect():
        x0 = (float(node.position[0]) + 20.0 - view.panX) * view.zoom * scale
        y0 = (float(node.position[1]) - view.panY) * view.zoom * scale
        x1 = (float(node.position[0]) + 120.0 - view.panX) * view.zoom * scale
        y1 = (float(node.position[1]) + titleHeight - view.panY) * view.zoom * scale
        return QtCore.QRect(int(x0), int(y0), int(x1 - x0), int(y1 - y0))

    box = rect().intersected(QtCore.QRect(0, 0, withIcon.width(),
                                          withIcon.height()))
    _Check(box.width() > 8 and box.height() > 8,
           "the title area is off-screen (%s)" % box)

    # Same node, same everything, with the authored icon cleared: the C++
    # producer falls back to its built-in default, so the title area MUST
    # change. It would not if the path never reached the GPU.
    node._icon_path = None
    _Check(node.titleIconPath == "", "clearing the icon did not reach C++")
    withoutIcon = repaint()

    samples = 0
    different = 0
    for y in range(box.top(), box.bottom(), 3):
        for x in range(box.left(), box.right(), 3):
            samples += 1
            if withIcon.pixel(x, y) != withoutIcon.pixel(x, y):
                different += 1
    _Check(samples > 50, "only %d pixels sampled" % samples)
    _Check(different > 20,
           "the authored icon changed %d of %d sampled title pixels -- it is "
           "not being drawn" % (different, samples))

    node._icon_path = iconPath
    shot = os.environ.get("RIGEXEC_NOODLES_SHOT")
    if shot:
        repaint().save(shot)

    for widget in list(app.topLevelWidgets()):
        if type(widget).__module__.startswith("UsdNoodles"):
            widget.close()
    view.close()
    _pump(app, 20)
    print("RIGEXEC_NOODLES_ICONS_OK %s -> %s, missing icon anchored to %s and "
          "fell back, %d/%d title pixels changed when the icon was cleared"
          % (ICONED, os.path.basename(iconPath),
             os.path.basename(missingPath), different, samples))
