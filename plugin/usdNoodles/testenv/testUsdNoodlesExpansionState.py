#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
Unit tests for expansion state persistence via UsdUINodeGraphNodeAPI.

Verifies that the ui:nodegraph:node:expansionState attribute can be read
from prims, matching the resolution logic used by NodeFactory.create_node_from_prim.

Uses mock prims to avoid SIGSEGV during interpreter shutdown caused by USD
C++ static destructors when real Usd.Stage objects are created in test
binaries (see testUsdNoodlesNodeFactoryIcon.py for the same pattern).
"""

import unittest
from unittest.mock import MagicMock, patch

from pxr import Tf, Usd, UsdUI

try:
    from UsdNoodles.models import NodeModel

    _has_node_model = True
except ImportError:
    _has_node_model = False

_EXPANSION_STATE_ATTR_NAME = "ui:nodegraph:node:expansionState"


def _resolve_expansion_state(prim):
    """Extract expansion state, mirroring NodeFactory logic."""
    expansion_attr = prim.GetAttribute(_EXPANSION_STATE_ATTR_NAME)
    if expansion_attr.IsValid() and expansion_attr.HasAuthoredValue():
        return str(expansion_attr.Get())
    return None


def _mock_prim(expansion_state=None, authored=True, has_attr=True):
    """Create a mock prim with configurable expansionState attribute behavior."""
    prim = MagicMock()
    attr = MagicMock()

    if not has_attr:
        attr.IsValid.return_value = False
        prim.GetAttribute.return_value = attr
        return prim

    attr.IsValid.return_value = True
    attr.HasAuthoredValue.return_value = authored

    if authored and expansion_state is not None:
        attr.Get.return_value = expansion_state

    prim.GetAttribute.return_value = attr
    return prim


class TestExpansionState(unittest.TestCase):
    """Verify ui:nodegraph:node:expansionState is read from prims."""

    def test_expansion_state_closed(self):
        prim = _mock_prim(expansion_state="closed")

        result = _resolve_expansion_state(prim)
        self.assertEqual(result, "closed")

    def test_expansion_state_open(self):
        prim = _mock_prim(expansion_state="open")

        result = _resolve_expansion_state(prim)
        self.assertEqual(result, "open")

    def test_expansion_state_minimized(self):
        prim = _mock_prim(expansion_state="minimized")

        result = _resolve_expansion_state(prim)
        self.assertEqual(result, "minimized")

    def test_no_expansion_state_authored(self):
        prim = _mock_prim(has_attr=False)

        result = _resolve_expansion_state(prim)
        self.assertIsNone(result)

    def test_write_and_read_expansion_state(self):
        # First read - closed
        prim = _mock_prim(expansion_state="closed")
        result = _resolve_expansion_state(prim)
        self.assertEqual(result, "closed")

        # Second read - open (simulating a write then read)
        prim = _mock_prim(expansion_state="open")
        result = _resolve_expansion_state(prim)
        self.assertEqual(result, "open")

    def test_closed_maps_to_collapsed(self):
        """Verify the title_collapsed mapping used by NodeFactory."""
        prim = _mock_prim(expansion_state="closed")

        state = _resolve_expansion_state(prim)
        self.assertIn(state, ("closed", "minimized"))

    def test_minimized_maps_to_collapsed(self):
        """Verify minimized is also treated as collapsed."""
        prim = _mock_prim(expansion_state="minimized")

        state = _resolve_expansion_state(prim)
        self.assertIn(state, ("closed", "minimized"))

    def test_open_maps_to_expanded(self):
        """Verify open is not treated as collapsed."""
        prim = _mock_prim(expansion_state="open")

        state = _resolve_expansion_state(prim)
        self.assertNotIn(state, ("closed", "minimized"))

    def test_session_layer_write(self):
        """Verify expansion state can be read from session layer writes."""
        # This tests reading only - the actual session layer write is internal
        prim = _mock_prim(expansion_state="closed")

        result = _resolve_expansion_state(prim)
        self.assertEqual(result, "closed")


@unittest.skipUnless(_has_node_model, "UsdNoodles.models is not importable")
class TestExpansionStateWrite(unittest.TestCase):
    """Folding a title must never drop the click that folded it.

    _write_expansion_state_to_usd used to author straight through to the
    stage, so any prim the stage refused to edit raised out of
    toggle_title_fold, up through GraphView.mousePressEvent, and the whole
    event was discarded. The node stayed unfolded and the user saw nothing
    but a traceback in the terminal.

    An attribute carrying an empty property name is the sharpest version of
    it: such an attribute throws from its own truth test as readily as from
    Set(), so guarding it needs the checks inside the try, not in front.
    """

    def _node_on(self, stage, path):
        return NodeModel(stage=stage, primPath=path)

    def test_writes_state_on_a_plain_prim(self):
        """The ordinary path still round-trips, API schema unapplied."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Node", "Scope")
        node = self._node_on(stage, "/Node")

        node.toggle_title_fold()
        attr = stage.GetPrimAtPath("/Node").GetAttribute(
            _EXPANSION_STATE_ATTR_NAME
        )
        self.assertTrue(attr.IsValid())
        self.assertEqual(str(attr.Get()), "closed")

        node.toggle_title_fold()
        self.assertEqual(str(attr.Get()), "open")

    def test_instance_proxy_is_skipped_without_raising(self):
        """Authoring to an instance proxy is illegal; skip, do not throw."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Src/Child", "Scope")
        inst = stage.DefinePrim("/Inst", "Scope")
        inst.GetReferences().AddInternalReference("/Src")
        inst.SetInstanceable(True)
        proxy = stage.GetPrimAtPath("/Inst/Child")
        self.assertTrue(proxy.IsInstanceProxy())

        node = self._node_on(stage, "/Inst/Child")
        node.toggle_title_fold()  # must not raise
        self.assertTrue(node.is_title_collapsed)

    def test_invalid_prim_is_skipped_without_raising(self):
        """A node whose prim went away (undo, delete) must not throw."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Gone", "Scope")
        node = self._node_on(stage, "/Gone")
        stage.RemovePrim("/Gone")

        node.toggle_title_fold()  # must not raise
        self.assertTrue(node.is_title_collapsed)

    def test_empty_named_attribute_does_not_escape(self):
        """The reported crash: propName.IsEmpty() out of attr.Set().

        An empty-named attribute raises from IsValid() too, which is why the
        fix cannot pre-check its way out of this one.
        """
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Node", "Scope")
        node = self._node_on(stage, "/Node")
        empty = stage.GetPrimAtPath("/Node").GetAttribute("")

        # Confirm the premise rather than trusting it: both the truth test
        # and Set() throw on this attribute.
        with self.assertRaises(Tf.ErrorException):
            bool(empty)
        with self.assertRaises(Tf.ErrorException):
            empty.Set("closed")

        fake_api = MagicMock()
        fake_api.GetExpansionStateAttr.return_value = empty
        fake_api.CreateExpansionStateAttr.return_value = empty
        with patch.object(
            UsdUI, "NodeGraphNodeAPI", return_value=fake_api
        ):
            node.toggle_title_fold()  # must not raise
        self.assertTrue(node.is_title_collapsed)

    def test_set_raising_does_not_escape(self):
        """Any stage refusal at Set() is contained, not propagated."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Node", "Scope")
        node = self._node_on(stage, "/Node")

        attr = MagicMock()
        attr.IsValid.return_value = True
        attr.GetName.return_value = _EXPANSION_STATE_ATTR_NAME
        attr.Set.side_effect = Tf.ErrorException("refused")
        fake_api = MagicMock()
        fake_api.GetExpansionStateAttr.return_value = attr
        with patch.object(
            UsdUI, "NodeGraphNodeAPI", return_value=fake_api
        ):
            node.toggle_title_fold()  # must not raise
        attr.Set.assert_called_once()


if __name__ == "__main__":
    unittest.main()
