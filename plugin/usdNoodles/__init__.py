#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#

from pxr import Tf
Tf.PreparePythonModule()
del Tf

# (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

from pathlib import Path

from pxr import Plug, Tf


# Sample graphs the editor cycles through for its "load a test graph"
# action. Three saved production graphs used to live here as well; each
# carried asset paths from the scene it was captured in, which is not ours
# to publish -- and a saved graph is a record of one specific scene, so
# there is little left of it once the paths are gone. Gitignored rather
# than scrubbed.
TEST_GRAPH_FILES = [
    "test_single_node.json",
]


def _getAssetsPath():
    plug = Plug.Registry().GetPluginWithName("UsdNoodles")

    if plug:
        resourcePath = Path(plug.resourcePath)
        assetsInResources = resourcePath / "assets"
        if assetsInResources.exists():
            return assetsInResources
        else:
            return resourcePath
    else:
        resourcePath = Path(__file__).parent
        assetsPath = resourcePath / "assets"
        if assetsPath.exists():
            return assetsPath
        return resourcePath


try:
    from ._qt_editor import (  # noqa: F401
        GetEditorManager,
        GetLayerEditorManager,
        NoodlesEditorManager,
        NoodlesEditorWidget,
        NoodlesEditorWindow,
        NoodlesLayerEditorManager,
        NoodlesLayerEditorWidget,
        NoodlesLayerEditorWindow,
        NoodlesPluginContainer,
        ShowNoodlesEditor,
        ShowNoodlesEditorForBlueprint,
        ShowNoodlesEditorForContainer,
        ShowNoodlesLayerEditor,
    )

    _HAS_QT = True
except ImportError:
    _HAS_QT = False

if _HAS_QT:
    # Tf.Type.Define uses __module__ to build the type path that must match
    # plugInfo.json ("UsdNoodles.NoodlesPluginContainer"). Since the class
    # lives in _qt_editor, we patch __module__ so the plugin registry finds it.
    NoodlesPluginContainer.__module__ = __name__
    Tf.Type.Define(NoodlesPluginContainer)


_INSTALLED_COPY = "pxr.UsdNoodles"


def _supersedeInstalledCopy():
    """Make this copy the process's only Noodles Editor.

    An OpenUSD built from PR #4156 with noodles installs the editor this
    package was localized from as ``pxr.UsdNoodles``, on the standard plugin
    path. When both are registered, usdview loads both, both containers
    register the same command names, and usdview answers the duplicate by
    loading *no* plugins. Leaving this copy unregistered avoids that but runs
    the installed editor instead, which predates everything added here --
    double-click rename among it -- so the editor silently lacks features.

    usdview loads plugins in name order, and "UsdNoodles" sorts before
    "pxr.UsdNoodles", so this runs first. It puts a stand-in module in the
    installed package's place, whose container registers nothing. usdview's
    ``import pxr.UsdNoodles`` then finds the stand-in, and the installed
    package, native module included, is never imported. If something did import
    it first, its container is disarmed instead.
    """
    import sys
    import types

    if Plug.Registry().GetPluginWithName(_INSTALLED_COPY) is None:
        return

    from pxr.Usdviewq import plugin as usdviewPlugin

    def _registerNothing(self, plugRegistry, plugCtx):
        pass

    def _configureNothing(self, plugRegistry, plugUIBuilder):
        pass

    installed = sys.modules.get(_INSTALLED_COPY)
    if installed is not None:
        container = getattr(installed, "NoodlesPluginContainer", None)
        if container is not None:
            container.registerPlugins = _registerNothing
            container.configureView = _configureNothing
        return

    # Tf.Type.Define names the type from __module__ and __name__, so the
    # stand-in claims exactly the type the installed plugInfo.json declares.
    standIn = types.new_class(
        "NoodlesPluginContainer", (usdviewPlugin.PluginContainer,))
    standIn.__module__ = _INSTALLED_COPY
    standIn.__doc__ = "Superseded by UsdNoodles; registers nothing."
    standIn.registerPlugins = _registerNothing
    standIn.configureView = _configureNothing

    module = types.ModuleType(
        _INSTALLED_COPY, "Stand-in: the in-repo UsdNoodles supersedes this copy.")
    module.NoodlesPluginContainer = standIn
    module.SUPERSEDED_BY = __name__
    sys.modules[_INSTALLED_COPY] = module
    import pxr
    pxr.UsdNoodles = module
    Tf.Type.Define(standIn)


if _HAS_QT:
    _supersedeInstalledCopy()
