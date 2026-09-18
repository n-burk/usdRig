#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
Unit tests for node icon resolution via the ui:nodegraph:node:icon attribute.

These call NodeFactory's OWN resolver rather than a copy of it: a copy passes
while the real thing is broken, which is exactly how the authored icon could
stop reaching the renderer without a test noticing.

What they pin down is the path, not the pixels (the drawing end to end is
tests/testUsdviewNoodlesIcons.py):

  * a resolved asset path is used as is -- USD already anchored it;
  * an UNRESOLVED relative path (a missing file) is anchored to the LAYER that
    authored the opinion, never left relative, because a relative path handed
    to the renderer would resolve against whatever directory usdview happened
    to be started in;
  * nothing authored leaves the node's icon unset, so the renderer's own
    default is used.

Uses mock prims to avoid SIGSEGV during interpreter shutdown caused by USD
C++ static destructors when real Usd.Stage objects are created in test
binaries (see testUsdNoodlesNodeLibraries.py for the same pattern).
"""

import posixpath
import types
import unittest
from unittest.mock import MagicMock

try:
    from UsdNoodles.nodeFactory import NodeFactory

    _has_factory = True
except ImportError:  # pragma: no cover - depends on the built extension
    _has_factory = False

_ICON_ATTR_NAME = "ui:nodegraph:node:icon"
_LAYER_DIR = "/assets/shots/010"


def _mock_attr(path="", resolved_path="", authored=True, has_attr=True,
               layer_dir=_LAYER_DIR):
    """An attribute mock shaped like Usd.Attribute for the icon resolver."""
    attr = MagicMock()

    if not has_attr:
        attr.IsValid.return_value = False
        return attr

    attr.IsValid.return_value = True
    attr.HasAuthoredValue.return_value = authored

    asset = MagicMock()
    asset.resolvedPath = resolved_path
    asset.path = path
    attr.Get.return_value = asset if authored else None

    # The strongest opinion's layer is what a relative path anchors to.
    spec = MagicMock()
    if layer_dir is None:
        spec.layer = None
    else:
        spec.layer.ComputeAbsolutePath.side_effect = (
            lambda relative: posixpath.normpath(
                posixpath.join(layer_dir, relative)
            )
        )
    attr.GetPropertyStack.return_value = [spec]
    return attr


def _mock_prim(**kwargs):
    prim = MagicMock()
    prim.GetAttribute.return_value = _mock_attr(**kwargs)
    return prim


@unittest.skipUnless(_has_factory, "UsdNoodles.nodeFactory not available")
class TestResolveIconAsset(unittest.TestCase):
    """NodeFactory._resolve_icon_asset: authored opinion -> filesystem path."""

    def test_resolved_path_is_used(self):
        attr = _mock_attr(path="../../icons/control.png",
                          resolved_path="/assets/icons/control.png")
        self.assertEqual(NodeFactory._resolve_icon_asset(attr),
                         "/assets/icons/control.png")

    def test_unresolved_relative_path_anchors_to_the_layer(self):
        # A missing file: USD leaves resolvedPath empty, and the authored path
        # is still relative to the layer, not to the working directory.
        attr = _mock_attr(path="../../icons/gone.png", resolved_path="")
        self.assertEqual(NodeFactory._resolve_icon_asset(attr),
                         "/assets/icons/gone.png")

    def test_unresolved_path_without_a_layer_is_left_alone(self):
        # Nothing to anchor to (no property spec carries a layer): the authored
        # path is returned rather than dropped, so the renderer can still fall
        # back on its own.
        attr = _mock_attr(path="icons/gone.png", resolved_path="",
                          layer_dir=None)
        self.assertEqual(NodeFactory._resolve_icon_asset(attr),
                         "icons/gone.png")

    def test_empty_asset_path_resolves_to_nothing(self):
        attr = _mock_attr(path="", resolved_path="")
        self.assertEqual(NodeFactory._resolve_icon_asset(attr), "")


@unittest.skipUnless(_has_factory, "UsdNoodles.nodeFactory not available")
class TestApplyIconFromPrim(unittest.TestCase):
    """NodeFactory._apply_icon_from_prim: what lands on the node."""

    @staticmethod
    def _node():
        # A stand-in for NodeModel: _icon_path is all this touches (on the real
        # model that assignment writes through to the C++ titleIconPath field).
        return types.SimpleNamespace(_icon_path=None)

    def test_icon_from_nodegraph_api(self):
        node = self._node()
        NodeFactory._apply_icon_from_prim(
            node, _mock_prim(path="custom.png",
                             resolved_path="/icons/custom.png"))
        self.assertEqual(node._icon_path, "/icons/custom.png")

    def test_no_icon_authored(self):
        node = self._node()
        NodeFactory._apply_icon_from_prim(node, _mock_prim(has_attr=False))
        self.assertIsNone(node._icon_path)

    def test_icon_attr_created_but_no_value(self):
        node = self._node()
        NodeFactory._apply_icon_from_prim(node, _mock_prim(authored=False))
        self.assertIsNone(node._icon_path)

    def test_icon_empty_path_not_set(self):
        node = self._node()
        NodeFactory._apply_icon_from_prim(node, _mock_prim(path="",
                                                           resolved_path=""))
        self.assertIsNone(node._icon_path)


if __name__ == "__main__":
    unittest.main()
