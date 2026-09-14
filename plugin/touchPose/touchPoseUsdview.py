#
# TouchPose's usdview plugin container.
#
# A SECOND container rather than another item in rigExecUsdview's, for
# one reason: `plugin/touchPose` is a self-contained port -- the importer,
# the `.touch` reader, the pick loop and the spikes all live here -- and
# reaching into `plugin/rigExecUsdview/rigExecUsdview.py` to register it
# would make the two directories mutually dependent for the sake of one
# menu line. `findOrCreateMenu("RigExec")` is exactly the seam that makes
# the separation free: both containers ask for the same menu, usdview
# hands them the same QMenu, and the item lands under RigExec -> TouchPose
# whichever container loads first.
#
# The panel is imported lazily inside the callback, as every panel in
# rigExecUsdview is, so that this module stays importable with no Qt.
#
import os
import sys

from pxr import Tf
from pxr.Usdviewq.plugin import PluginContainer


# usdview's plugin loader does not retain a container that registers no
# commands, and a garbage-collected container takes its Qt connections
# with it. rigExecUsdview keeps a module-level reference for the same
# reason; this is that, not a stray global.
_container = None


class TouchPoseContainer(PluginContainer):

    def registerPlugins(self, plugRegistry, plugCtx):
        global _container
        _container = self
        self._api = plugCtx

        # Importable from this directory without it being on PYTHONPATH:
        # the plugin is found through PXR_PLUGINPATH_NAME, which says
        # nothing about module search, and `touchPoseUI` imports
        # `touchPoseModel` beside it.
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)

        self._touchPose = plugRegistry.registerCommandPlugin(
            "TouchPoseContainer.touchPose",
            "TouchPose",
            lambda api: self._OpenPanel(api))

    def configureView(self, plugRegistry, plugUIBuilder):
        menu = plugUIBuilder.findOrCreateMenu("RigExec")
        menu.addItem(self._touchPose)

    def _OpenPanel(self, usdviewApi=None):
        try:
            import touchPoseUI
        except ImportError as error:
            Tf.Warn("touchPose: panel unavailable: %s" % error)
            return None
        return touchPoseUI.OpenTouchPosePanel(usdviewApi or self._api)


Tf.Type.Define(TouchPoseContainer)
