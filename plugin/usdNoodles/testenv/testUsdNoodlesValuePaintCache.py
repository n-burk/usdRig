#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Unit tests for the ``GraphView`` value-paint caches.

The value pass runs per frame over every visible row, so everything it
derives from slowly-changing inputs is cached: row-layout snapshots keyed
by the layout epoch, fitted texts keyed by (value, width, font), glyph
vertices keyed by (character, size), and text-band geometry. What is
asserted:

  * the row-layout snapshot is reused within an epoch and rebuilt after a
    layout bump or an ``invalidateCache``-style drop;
  * fitted texts are reused until the value, width or font changes;
  * each glyph is generated once per size and translated into place
    (including the italic shear);
  * the text band is computed once per (row height, fonts);
  * hover hit-testing never builds geometry below the paint LOD gate.

``GraphView`` methods are invoked unbound on ``SimpleNamespace`` fixtures,
so no widget, GL context or font atlas is needed.
"""

import unittest
from types import SimpleNamespace

from pxr import Sdf, Usd

try:
    from UsdNoodles import noodlesValues as nv
    from UsdNoodles.graphView import GraphView

    _has_graph = True
except ImportError as e:  # pragma: no cover - import guard
    print(f"value paint cache imports failed: {e}")
    _has_graph = False


def _stage_with(props):
    """An in-memory stage whose /Values prim carries ``{name: (type, value)}``."""
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/Values", "Scope")
    for name, (type_name, value) in props.items():
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Find(type_name))
        if value is not None:
            attr.Set(value)
    return stage, prim


@unittest.skipUnless(_has_graph, "GraphView modules not available")
class TestValueRowLayoutCache(unittest.TestCase):
    def _node(self):
        return SimpleNamespace(
            inputPins=["a", "b"],
            outputPins=["o"],
            inputRowSlots=[0, 1],
            inputRowKinds=[0, 0],
            outputRowSlots=[0],
            _row_layout_version=3,
            _value_row_layout=None,
        )

    def test_snapshot_reused_within_epoch(self):
        view = SimpleNamespace()
        node = self._node()
        first = GraphView._valueRowLayout(view, node)
        self.assertEqual(first[0], ["a", "b"])
        self.assertEqual(first[1], [0, 1])
        self.assertEqual(first[3], {0})
        self.assertIsNotNone(node._value_row_layout)
        # Mutated vectors without a layout bump are invisible: the epoch
        # says nothing changed.
        node.inputRowSlots = [5, 6]
        second = GraphView._valueRowLayout(view, node)
        self.assertEqual(second[1], [0, 1])

    def test_layout_bump_rebuilds_snapshot(self):
        view = SimpleNamespace()
        node = self._node()
        GraphView._valueRowLayout(view, node)
        node.inputRowSlots = [5, 6]
        node._row_layout_version = 4
        _pins, slots, _kinds, _occupied = GraphView._valueRowLayout(view, node)
        self.assertEqual(slots, [5, 6])

    def test_dropped_cache_rebuilds(self):
        view = SimpleNamespace()
        node = self._node()
        GraphView._valueRowLayout(view, node)
        node.inputRowSlots = [5, 6]
        node._value_row_layout = None  # what invalidateCache does
        _pins, slots, _kinds, _occupied = GraphView._valueRowLayout(view, node)
        self.assertEqual(slots, [5, 6])


@unittest.skipUnless(_has_graph, "GraphView modules not available")
class TestFittedTextCache(unittest.TestCase):
    def test_fit_reused_until_value_width_or_font_changes(self):
        stage, prim = _stage_with({"v": ("float3", (1.0, 2.0, 3.0))})
        row = nv.build_value_row(stage, prim, "v", "v")
        view = SimpleNamespace(_valueFitCache={})
        calls = []

        def _measure(text):
            calls.append(text)
            return len(text) * 10.0

        first = GraphView._valueFittedTexts(view, "n", row, 100.0, 20.0, _measure)
        measured = len(calls)
        self.assertGreater(measured, 0)
        second = GraphView._valueFittedTexts(view, "n", row, 100.0, 20.0, _measure)
        self.assertEqual(second, first)
        self.assertEqual(len(calls), measured, "cache hit must not re-measure")
        # A new value refits.
        changed = row._replace(components=(9.0, 9.0, 9.0))
        GraphView._valueFittedTexts(view, "n", changed, 100.0, 20.0, _measure)
        self.assertGreater(len(calls), measured)
        # So do a new width and a new font size.
        measured = len(calls)
        GraphView._valueFittedTexts(view, "n", row, 50.0, 20.0, _measure)
        GraphView._valueFittedTexts(view, "n", row, 100.0, 24.0, _measure)
        self.assertGreater(len(calls), measured)


class _FakeTextRenderer:
    """Deterministic stand-in: one 6-vertex quad per character."""

    def __init__(self):
        self.gen_calls = 0
        self.width_calls = 0

    def calculateTextWidth(self, text, size):
        self.width_calls += 1
        return len(text) * size * 0.5

    def generateTextVertices(
        self, text, cx, cy, depth, size, out, node_index=0.0
    ):
        self.gen_calls += 1
        for i, _ch in enumerate(text):
            x0 = cx + i * size * 0.5
            for v in range(6):
                out.extend([x0 + v, cy + v, depth, 0.0, 0.0, node_index])
        return cx, len(text)


@unittest.skipUnless(_has_graph, "GraphView modules not available")
class TestGlyphCache(unittest.TestCase):
    def _view(self):
        view = SimpleNamespace(
            _valueGlyphCache={},
            _valueDigitAdvanceCache={},
            textRenderer=_FakeTextRenderer(),
        )
        view._valueDigitAdvance = GraphView._valueDigitAdvance.__get__(view)
        view._valueGlyph = GraphView._valueGlyph.__get__(view)
        return view

    def test_each_glyph_generated_once_per_size(self):
        view = self._view()
        first = GraphView._valueGlyph(view, "a", 10.0)
        second = GraphView._valueGlyph(view, "a", 10.0)
        self.assertIs(first, second)
        self.assertEqual(view.textRenderer.gen_calls, 1)
        GraphView._valueGlyph(view, "a", 12.0)
        self.assertEqual(view.textRenderer.gen_calls, 2)

    def test_tabular_emit_translates_cached_glyphs(self):
        view = self._view()
        out = []
        width = GraphView._emitValueText(
            view, "12", 100.0, 50.0, 3.0, 10.0, out, tabular=True
        )
        # Digits ride the widest-digit advance: 2 * 5.0 at size 10.
        self.assertAlmostEqual(width, 10.0)
        self.assertEqual(len(out), 2 * 6 * 6)
        # First glyph translated to the cursor, depth baked in.
        self.assertAlmostEqual(out[0], 100.0)
        self.assertAlmostEqual(out[1], 50.0)
        self.assertAlmostEqual(out[2], 3.0)
        # Second glyph starts one advance later.
        self.assertAlmostEqual(out[36], 105.0)

    def test_italic_shear_matches_unrolled_math(self):
        view = self._view()
        out = []
        GraphView._emitValueText(
            view, "a", 100.0, 50.0, 3.0, 10.0, out, italic=0.2
        )
        # x' = x + (baseline - y) * shear, per vertex.
        self.assertAlmostEqual(out[0], 100.0 + (50.0 - 50.0) * 0.2)
        self.assertAlmostEqual(out[6], 101.0 + (50.0 - 51.0) * 0.2)
        self.assertAlmostEqual(out[8], 3.0)


@unittest.skipUnless(_has_graph, "GraphView modules not available")
class TestTextBandCache(unittest.TestCase):
    def test_band_computed_once_per_inputs(self):
        atlas = SimpleNamespace(ascender=0.8, descender=-0.2, lineHeight=1.0)
        view = SimpleNamespace(
            textRenderer=SimpleNamespace(font_atlas=atlas),
            _cachedNodePinFontSize=18.0,
            _cachedValueFontScale=0.85,
            _valueTextBandCache={},
        )
        view._valueFontSize = GraphView._valueFontSize.__get__(view)
        first = GraphView._valueTextBand(view, 100.0)
        self.assertIs(GraphView._valueTextBand(view, 100.0), first)
        self.assertEqual(len(first), 2)


@unittest.skipUnless(_has_graph, "GraphView modules not available")
class TestValueHitTestLodGate(unittest.TestCase):
    def test_below_lod_gate_never_builds_geometry(self):
        def _boom(_node):
            raise AssertionError("geometry must not run below the LOD gate")

        view = SimpleNamespace(
            _cachedShowAttributeValues=True,
            nodeIdUnderCursor="n",
            nodes={"n": SimpleNamespace(layoutPortLineHeight=2.0)},
            zoom=1.0,
            _cachedValueMinPixelHeight=9.0,
            _valueRowGeometry=_boom,
            _asWorldXY=GraphView._asWorldXY,
        )
        self.assertIsNone(GraphView._valueCellAtPoint(view, (5.0, 5.0)))

    def test_above_lod_gate_reaches_geometry(self):
        seen = []

        def _record(node):
            seen.append(node)
            return iter([])

        node = SimpleNamespace(layoutPortLineHeight=100.0)
        view = SimpleNamespace(
            _cachedShowAttributeValues=True,
            nodeIdUnderCursor="n",
            nodes={"n": node},
            zoom=1.0,
            _cachedValueMinPixelHeight=9.0,
            _valueRowGeometry=_record,
            _asWorldXY=GraphView._asWorldXY,
        )
        self.assertIsNone(GraphView._valueCellAtPoint(view, (5.0, 5.0)))
        self.assertEqual(seen, [node])


if __name__ == "__main__":
    unittest.main()
