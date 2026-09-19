#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Unit tests for the ``NodeModel`` value-row caches.

The paint pass asks for value rows every frame and the notice path asks on
every property change, so the rows are layered: one schema walk per cache
epoch (shared with the pin getters), time-independent templates built once,
and per-frame components re-read against them. What is asserted:

  * one schema walk serves the pin getters, the pin-type getters and the
    value rows together;
  * a frame change re-reads components but keeps the templates;
  * ``invalidate_value_row(name)`` refreshes just that row in place;
  * the property index answers ``has_value_row`` / ``value_row_for_property``;
  * the layout epoch bumps exactly where slots can change, and
    ``invalidateCache`` drops the geometry derived from them.

In-memory stage only: no Qt, no GL, no ``GraphView``.
"""

import unittest
from unittest.mock import patch

from pxr import Sdf, Usd

try:
    from UsdNoodles import models as _models
    from UsdNoodles.models import NodeModel

    _has_models = True
except ImportError as e:  # pragma: no cover - import guard
    print(f"UsdNoodles.models import failed: {e}")
    _has_models = False


def _stage_with(props):
    """An in-memory stage whose /Values prim carries ``{name: (type, value)}``."""
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/Values", "Scope")
    for name, (type_name, value) in props.items():
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Find(type_name))
        if value is not None:
            attr.Set(value)
    return stage, prim


@unittest.skipUnless(_has_models, "UsdNoodles.models not available")
class TestPinPropertiesSharedWalk(unittest.TestCase):
    def test_one_walk_serves_pins_types_and_rows(self):
        stage, _prim = _stage_with({"a": ("float", 1.0), "b": ("float", 2.0)})
        node = NodeModel(stage, "/Values")
        real = _models.get_schema_aware_pin_properties
        calls = []

        def _counting(prim, *args, **kwargs):
            calls.append(prim)
            return real(prim, *args, **kwargs)

        with patch.object(
            _models, "get_schema_aware_pin_properties", _counting
        ):
            _ = node.inputPins
            _ = node.outputPins
            _ = node.inputPinTypes
            _ = node.outputPinTypes
            rows = node.value_rows(Usd.TimeCode(1.0))
        self.assertEqual(len(calls), 1, "expected a single schema walk")
        self.assertIn(("a", False), rows)
        self.assertIn(("b", False), rows)


@unittest.skipUnless(_has_models, "UsdNoodles.models not available")
class TestValueRowTimeSplit(unittest.TestCase):
    def _node(self):
        stage, prim = _stage_with({"a": ("float", 1.0), "b": ("float", 2.0)})
        attr = prim.GetAttribute("b")
        attr.Set(2.0, Usd.TimeCode(1.0))
        attr.Set(20.0, Usd.TimeCode(11.0))
        return NodeModel(stage, "/Values"), prim

    def test_same_frame_returns_the_cached_dict(self):
        node, _prim = self._node()
        rows = node.value_rows(Usd.TimeCode(1.0))
        self.assertIs(node.value_rows(Usd.TimeCode(1.0)), rows)

    def test_frame_change_keeps_templates_refreshes_components(self):
        node, _prim = self._node()
        rows = node.value_rows(Usd.TimeCode(1.0))
        templates = node._value_row_templates()
        self.assertEqual(rows[("b", False)].components, (2.0,))
        rows2 = node.value_rows(Usd.TimeCode(11.0))
        self.assertIsNot(rows2, rows)
        self.assertIs(node._value_row_templates(), templates)
        self.assertEqual(rows2[("b", False)].components, (20.0,))
        # The static row stayed identical without a schema re-walk.
        self.assertEqual(rows2[("a", False)].components, (1.0,))

    def test_per_property_invalidation_refreshes_one_row_in_place(self):
        node, prim = self._node()
        rows = node.value_rows(Usd.TimeCode(1.0))
        row_a = rows[("a", False)]
        # "b" is time-sampled in this fixture, so the new value must be a
        # sample at the cached frame: a Default would lose to the sample.
        prim.GetAttribute("b").Set(9.0, Usd.TimeCode(1.0))
        node.invalidate_value_row("b")
        rows2 = node.value_rows(Usd.TimeCode(1.0))
        self.assertIs(rows2, rows, "sibling rows must not be rebuilt")
        self.assertIs(rows2[("a", False)], row_a)
        self.assertEqual(rows2[("b", False)].components, (9.0,))

    def test_unknown_property_invalidation_is_a_noop(self):
        node, _prim = self._node()
        rows = node.value_rows(Usd.TimeCode(1.0))
        node.invalidate_value_row("no-such-property")
        self.assertIs(node.value_rows(Usd.TimeCode(1.0)), rows)

    def test_whole_dict_drop_keeps_templates(self):
        node, _prim = self._node()
        node.value_rows(Usd.TimeCode(1.0))
        templates = node._value_row_templates()
        node.invalidate_value_row()
        self.assertIsNone(node._value_rows)
        node.value_rows(Usd.TimeCode(1.0))
        self.assertIs(node._value_row_templates(), templates)

    def test_property_index_lookup(self):
        node, _prim = self._node()
        self.assertTrue(node.has_value_row("a"))
        self.assertFalse(node.has_value_row("no-such-property"))
        row = node.value_row_for_property("a", Usd.TimeCode(1.0))
        self.assertIsNotNone(row)
        self.assertEqual(row.property_name, "a")
        self.assertIsNone(node.value_row_for_property("no-such-property"))


@unittest.skipUnless(_has_models, "UsdNoodles.models not available")
class TestRowLayoutEpoch(unittest.TestCase):
    def test_display_rebuild_bumps_the_epoch(self):
        node = NodeModel()
        before = node._row_layout_version
        node.inputPins = ["x"]
        self.assertGreater(node._row_layout_version, before)

    def test_invalidate_cache_drops_row_geometry(self):
        node = NodeModel()
        node.inputPins = ["x"]
        node._value_row_layout = (node._row_layout_version, ["x"], [0], [0], set())
        node.invalidateCache()
        self.assertIsNone(node._value_row_layout)
        self.assertIsNone(node._pin_properties_cache)
        self.assertIsNone(node._value_row_static)
        self.assertIsNone(node._value_rows_by_property)


if __name__ == "__main__":
    unittest.main()
