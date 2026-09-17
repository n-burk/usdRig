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


# RigExec -> submenu placement by rank. A copy of
# rigExecUsdview.AddToRigExecMenu, kept here so this directory does not
# import that one; the rank table lives beside the original.
_SUBMENU_RANKS = {"Viewport": 10, "General Editors": 20,
                  "Animation Editors": 30}
_MENU_RANK = "rigExecMenuRank"


def _PlaceByRank(qMenu, action, rank):
    action.setProperty(_MENU_RANK, rank)
    for other in qMenu.actions():
        if other == action:
            return
        otherRank = other.property(_MENU_RANK)
        if otherRank is not None and otherRank > rank:
            qMenu.removeAction(action)
            qMenu.insertAction(other, action)
            return


def _AddToRigExecMenu(plugUIBuilder, submenu, commandPlugin, rank):
    menu = plugUIBuilder.findOrCreateMenu("RigExec").findOrCreateSubmenu(
        submenu)
    action = menu.addItem(commandPlugin)
    qMenu = action.parent()
    _PlaceByRank(qMenu, action, rank)
    _PlaceByRank(qMenu.parent(), qMenu.menuAction(), _SUBMENU_RANKS[submenu])
    return action


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
        _AddToRigExecMenu(plugUIBuilder, "Animation Editors",
                          self._shapeEditor, 20)

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
