#
# The Shape Editor's usdview plugin container.
#
# A container of its own, for the same reason `plugin/touchPose` has one:
# `findOrCreateMenu("RigExec")` is the seam that lets a self-contained
# plugin directory add a menu item without the two directories having to
# import each other. Whichever container loads first creates the menu; the
# rest find it.
#
# The panel is imported lazily inside the callback so this module stays
# importable with no Qt.
#
import os
import sys

from pxr import Tf
from pxr.Usdviewq.plugin import PluginContainer


# usdview does not retain a container that registers no commands, and a
# collected container takes its Qt connections with it.
_container = None


class ShapeEditorContainer(PluginContainer):

    def registerPlugins(self, plugRegistry, plugCtx):
        global _container
        _container = self
        self._api = plugCtx

        # Found through PXR_PLUGINPATH_NAME, which says nothing about
        # module search -- so this directory has to go on sys.path for
        # `shapeEditorUI` to import `shapeEditorModel` beside it.
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)

        self._shapeEditor = plugRegistry.registerCommandPlugin(
            "ShapeEditorContainer.shapeEditor",
            "Shape Editor",
            lambda api: self._OpenPanel(api))

    def configureView(self, plugRegistry, plugUIBuilder):
        menu = plugUIBuilder.findOrCreateMenu("RigExec")
        menu.addItem(self._shapeEditor)

    def _OpenPanel(self, usdviewApi=None):
        try:
            import shapeEditorUI
        except ImportError as error:
            Tf.Warn("shapeEditor: panel unavailable: %s" % error)
            return None
        panel = shapeEditorUI.ShapeEditorPanel.GetInstance(
            usdviewApi or self._api)
        panel.show()
        panel.raise_()
        return panel


Tf.Type.Define(ShapeEditorContainer)
