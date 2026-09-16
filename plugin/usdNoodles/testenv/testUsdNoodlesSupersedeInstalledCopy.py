#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
This package takes the place of an installed pxr.UsdNoodles.

An OpenUSD built with noodles installs the editor this package was localized
from, on the standard plugin path. usdview loads every plugin container it
finds; the two register the same command names, and a duplicate makes usdview
load no plugins at all. Before this package took over, the launchers kept it
unregistered in that case, so usdview ran the older installed editor and
everything added here -- double-click rename among it -- silently did nothing.

The check mirrors usdview's own loadPlugins (Usdviewq/plugin.py) for the two
Noodles plugins -- same discovery, same name order, same registry -- in a
fresh interpreter, because what is being tested is exactly what a process
imports and in what order. When the USD install has no pxr.UsdNoodles, a
plugInfo.json declaring one is registered, so the check runs either way.
"""

import json
import os
import subprocess
import sys
import textwrap
import unittest

_CHILD = textwrap.dedent(
    r"""
    import importlib.util, json, os, sys, tempfile
    from pxr import Plug

    registry = Plug.Registry()
    declared = registry.GetPluginWithName("pxr.UsdNoodles") is not None
    if not declared:
        directory = tempfile.mkdtemp()
        with open(os.path.join(directory, "plugInfo.json"), "w") as f:
            json.dump({"Plugins": [{
                "Type": "python", "Name": "pxr.UsdNoodles", "ResourcePath": ".",
                "Info": {"Types": {"pxr.UsdNoodles.NoodlesPluginContainer": {
                    "bases": ["pxr.Usdviewq.plugin.PluginContainer"]}}}}]}, f)
        registry.RegisterPlugins(directory)

    spec = importlib.util.find_spec("UsdNoodles")
    if registry.GetPluginWithName("UsdNoodles") is None:
        registry.RegisterPlugins(os.path.dirname(spec.origin))

    try:
        from pxr.Usdviewq import plugin as usdviewPlugin
    except ImportError as e:
        print(json.dumps({"skip": "usdview/Qt not importable: %s" % e}))
        sys.exit(0)

    byPlugin = {}
    for containerType in Plug.Registry.GetAllDerivedTypes(
            usdviewPlugin.PluginContainerTfType):
        plugin = Plug.Registry().GetPluginForType(containerType)
        if plugin.name in ("UsdNoodles", "pxr.UsdNoodles"):
            byPlugin.setdefault(plugin, []).append(containerType)

    order, missing, containers = [], [], []
    for plugin in sorted(byPlugin, key=lambda p: p.name):
        order.append(plugin.name)
        plugin.Load()
        for containerType in sorted(byPlugin[plugin], key=lambda t: t.typeName):
            if containerType.pythonClass is None:
                missing.append(containerType.typeName)
                continue
            containers.append(containerType.pythonClass())

    commands = usdviewPlugin.PluginRegistry(None)
    duplicate = None
    for container in containers:
        try:
            container.registerPlugins(commands, None)
        except usdviewPlugin.DuplicateCommandPlugin as e:
            duplicate = str(e)
            break

    editor = commands.getCommandPlugin("NoodlesPluginContainer.ShowNoodlesEditor")
    installed = sys.modules.get("pxr.UsdNoodles")
    print(json.dumps({
        "declared": declared,
        "order": order,
        "missing": missing,
        "duplicate": duplicate,
        "editorModule": editor._callback.__module__ if editor else None,
        "supersededBy": getattr(installed, "SUPERSEDED_BY", None),
        "installedSubmodules": sorted(
            m for m in sys.modules if m.startswith("pxr.UsdNoodles.")),
    }))
    """
)


class SupersedeInstalledCopyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        proc = subprocess.run(
            [sys.executable, "-c", _CHILD],
            capture_output=True,
            text=True,
            timeout=300,
            env=dict(os.environ),
        )
        lines = [ln for ln in proc.stdout.splitlines() if ln.startswith("{")]
        if proc.returncode != 0 or not lines:
            raise AssertionError(
                "child interpreter failed (%d):\n%s\n%s"
                % (proc.returncode, proc.stdout, proc.stderr)
            )
        cls.result = json.loads(lines[-1])
        if "skip" in cls.result:
            raise unittest.SkipTest(cls.result["skip"])

    def test_this_package_loads_first(self):
        """The stand-in only works because usdview loads plugins by name."""
        self.assertEqual(self.result["order"], ["UsdNoodles", "pxr.UsdNoodles"])

    def test_no_duplicate_command_stops_plugin_loading(self):
        self.assertIsNone(self.result["duplicate"])

    def test_the_editor_command_is_this_package(self):
        self.assertTrue(
            (self.result["editorModule"] or "").startswith("UsdNoodles."),
            self.result["editorModule"],
        )

    def test_the_installed_container_is_the_stand_in(self):
        self.assertEqual(self.result["missing"], [])
        self.assertEqual(self.result["supersededBy"], "UsdNoodles")

    def test_the_installed_package_is_never_imported(self):
        self.assertEqual(self.result["installedSubmodules"], [])


if __name__ == "__main__":
    unittest.main()
