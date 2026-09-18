#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Geometry tests for the inline attribute value cells.

The load-bearing assertion is the first class in this file:

    the value cell and the two CONNECTION GUTTERS are disjoint.

``GraphView._findRowEdgeDragStart`` claims the outer tenth of a node's width
on each side, for the full height of a row band, as "press here to start a
connection". If a value cell ever reached into that band, a press meant for
a value would start a link instead -- the one regression this whole feature
must never cause. It is asserted structurally, over a grid of node widths
and row heights, rather than trusted to a comment.

The row-suppression rules are mirrored here against plain-object fakes
(NOT MagicMock: ``getattr(mock, "_title_collapsed", False)`` returns a
truthy Mock rather than the default, which would silently short-circuit
exactly the branch under test -- the same trap
``testUsdNoodlesPinDragInteractions.py`` records).
"""

import unittest

try:
    from UsdNoodles.widgets import valueCell as vc

    _HAS_CELL = True
except ImportError as e:  # pragma: no cover - import guard
    print(f"UsdNoodles.widgets.valueCell import failed: {e}")
    _HAS_CELL = False


NODE_WIDTHS = (400.0, 600.0, 1200.0)
ROW_HEIGHTS = (40.0, 85.0, 160.0)
NODE_X = (-500.0, 0.0, 1234.5)


def _width(text):
    return len(text) * 10.0


class _Node:
    """A node with just the fields the row geometry reads."""

    def __init__(
        self,
        position=(0.0, 0.0),
        size=(600.0, 400.0),
        input_pins=None,
        output_pins=None,
        input_slots=None,
        output_slots=None,
        input_kinds=None,
        output_kinds=None,
        title_collapsed=False,
        port_start_y=100.0,
        line_height=85.0,
        cell_width=300.0,
    ):
        self.position = position
        self.size = size
        self.inputPins = list(input_pins or [])
        self.outputPins = list(output_pins or [])
        self.inputRowSlots = list(
            input_slots if input_slots is not None else range(len(self.inputPins))
        )
        self.outputRowSlots = list(
            output_slots if output_slots is not None else range(len(self.outputPins))
        )
        self.inputRowKinds = list(input_kinds or [0] * len(self.inputPins))
        self.outputRowKinds = list(output_kinds or [0] * len(self.outputPins))
        self._title_collapsed = title_collapsed
        self.layoutPortStartY = port_start_y
        self.layoutPortLineHeight = line_height
        self._value_cell_width = cell_width


def _rows_for(node):
    """The row-suppression rules of ``GraphView._valueRowGeometry``.

    Mirrored rather than imported, because GraphView cannot be imported
    headlessly (module-level GL). Kept byte-faithful to the production
    method; the production method is the one that ships.
    """
    if getattr(node, "_title_collapsed", False):
        return []
    cell_w = float(getattr(node, "_value_cell_width", 0.0) or 0.0)
    if cell_w <= 0.0:
        return []
    nx = float(node.position[0])
    nw = float(node.size[0])
    h = float(node.layoutPortLineHeight)
    port_start_y = float(node.position[1]) + float(node.layoutPortStartY)

    occupied = set(
        node.outputRowSlots[i] if i < len(node.outputRowSlots) else i
        for i in range(len(node.outputPins))
    )

    out = []
    for i, pin in enumerate(node.inputPins):
        kind = node.inputRowKinds[i] if i < len(node.inputRowKinds) else 0
        if kind in (1, 2):
            continue
        slot = node.inputRowSlots[i] if i < len(node.inputRowSlots) else i
        if slot in occupied:
            continue
        rect = vc.cell_rect(nx, nw, port_start_y + slot * h, h, cell_w)
        if rect is None:
            continue
        out.append((pin, slot, rect))
    return out


@unittest.skipUnless(_HAS_CELL, "UsdNoodles.widgets.valueCell unavailable")
class TestDisjointness(unittest.TestCase):
    """The invariant. Everything else in the feature depends on it."""

    def test_cell_never_touches_either_gutter(self):
        for nx in NODE_X:
            for nw in NODE_WIDTHS:
                for h in ROW_HEIGHTS:
                    for want in (1.0, 3.0 * h, 10.0 * h, 10_000.0):
                        rect = vc.cell_rect(nx, nw, 0.0, h, want)
                        if rect is None:
                            continue
                        left_gutter, right_gutter = vc.gutter_rects(nx, nw, 0.0, h)
                        msg = "nx=%s nw=%s h=%s want=%s -> %s" % (
                            nx,
                            nw,
                            h,
                            want,
                            (rect,),
                        )
                        self.assertLess(rect.right, right_gutter.left, msg)
                        self.assertGreater(rect.left, left_gutter.right, msg)

    def test_cell_right_is_strictly_inside_the_drag_band(self):
        for nw in NODE_WIDTHS:
            for h in ROW_HEIGHTS:
                rect = vc.cell_rect(0.0, nw, 0.0, h, 10.0 * h)
                if rect is None:
                    continue
                self.assertLess(rect.right, nw * 0.9)

    def test_the_gap_to_the_gutter_is_exactly_the_padding(self):
        nw, h = 600.0, 85.0
        rect = vc.cell_rect(0.0, nw, 0.0, h, 300.0)
        _left, right_gutter = vc.gutter_rects(0.0, nw, 0.0, h)
        self.assertAlmostEqual(right_gutter.left - rect.right, vc.PAD_RATIO * h)

    def test_a_cell_squeezed_under_the_floor_is_dropped_entirely(self):
        # A node so narrow that honouring both gutters leaves no room: the
        # answer is no cell, not a two-pixel one.
        self.assertIsNone(vc.cell_rect(0.0, 100.0, 0.0, 160.0, 300.0))

    def test_sub_cells_stay_inside_the_cell(self):
        for h in ROW_HEIGHTS:
            rect = vc.cell_rect(0.0, 1200.0, 0.0, h, 8.0 * h)
            subs = vc.sub_rects(rect, 4, h)
            self.assertGreaterEqual(subs[0].left, rect.left)
            self.assertLessEqual(subs[-1].right, rect.right + 1e-9)

    def test_a_point_in_the_gutter_is_never_in_the_cell(self):
        nw, h = 600.0, 85.0
        rect = vc.cell_rect(0.0, nw, 0.0, h, 300.0)
        for x in (0.0, 30.0, 59.9, 540.1, 570.0, 599.9):
            self.assertFalse(vc.rect_contains(rect, x, h * 0.5), x)


@unittest.skipUnless(_HAS_CELL, "UsdNoodles.widgets.valueCell unavailable")
class TestSubCells(unittest.TestCase):
    def test_partition_sums_back_to_the_cell_width(self):
        for count in (2, 3, 4):
            for h in ROW_HEIGHTS:
                rect = vc.cell_rect(0.0, 1200.0, 0.0, h, 9.0 * h)
                subs = vc.sub_rects(rect, count, h)
                self.assertEqual(len(subs), count)
                total = sum(vc.rect_width(s) for s in subs)
                total += (count - 1) * vc.SUB_GAP_RATIO * h
                self.assertAlmostEqual(total, vc.rect_width(rect), places=6)

    def test_sub_cells_are_equal_and_ordered(self):
        h = 85.0
        rect = vc.cell_rect(0.0, 1200.0, 0.0, h, 600.0)
        subs = vc.sub_rects(rect, 3, h)
        widths = [vc.rect_width(s) for s in subs]
        self.assertAlmostEqual(widths[0], widths[1])
        self.assertAlmostEqual(widths[1], widths[2])
        self.assertLess(subs[0].right, subs[1].left)
        self.assertLess(subs[1].right, subs[2].left)

    def test_a_swatch_inset_shifts_the_components_right(self):
        h = 85.0
        rect = vc.cell_rect(0.0, 1200.0, 0.0, h, 600.0)
        plain = vc.sub_rects(rect, 3, h)
        inset = vc.sub_rects(rect, 3, h, vc.swatch_inset(h))
        self.assertGreater(inset[0].left, plain[0].left)
        self.assertAlmostEqual(inset[-1].right, plain[-1].right)
        swatch = vc.swatch_rect(rect, h)
        self.assertAlmostEqual(swatch.left, rect.left)
        self.assertLessEqual(swatch.right, inset[0].left)

    def test_an_unreadable_vector_drops_its_cell_but_a_scalar_keeps_one(self):
        h = 160.0
        # 2.0*h wide is the narrowest cell_rect will produce at all.
        rect = vc.cell_rect(0.0, 4000.0, 0.0, h, 2.0 * h)
        self.assertIsNotNone(vc.sub_rects_fit(rect, 1, h))
        self.assertIsNone(vc.sub_rects_fit(rect, 3, h))

    def test_text_width_leaves_the_inner_padding_alone(self):
        h = 85.0
        rect = vc.cell_rect(0.0, 1200.0, 0.0, h, 600.0)
        sub = vc.sub_rects(rect, 1, h)[0]
        self.assertAlmostEqual(
            vc.text_max_width(sub, h),
            vc.rect_width(sub) - 2.0 * vc.INNER_PAD_RATIO * h,
        )


@unittest.skipUnless(_HAS_CELL, "UsdNoodles.widgets.valueCell unavailable")
class TestWidening(unittest.TestCase):
    def test_the_widened_node_still_fits_gutter_padding_and_cell(self):
        for nw0 in NODE_WIDTHS:
            for h in ROW_HEIGHTS:
                cell_w = vc.clamp_cell_width(5.0 * h, h)
                reserve = vc.reserve_width(cell_w, h)
                nw = vc.widened_node_width(nw0, reserve)
                # Gutter + padding + cell + the ORIGINAL label width all fit.
                self.assertAlmostEqual(
                    nw - nw * vc.GUTTER_RATIO - vc.PAD_RATIO * h - cell_w,
                    nw0,
                    places=6,
                )
                rect = vc.cell_rect(0.0, nw, 0.0, h, cell_w)
                self.assertIsNotNone(rect)
                self.assertAlmostEqual(vc.rect_width(rect), cell_w, places=6)
                self.assertLessEqual(rect.left, nw0 + 1e-6)

    def test_widening_is_idempotent_given_the_same_base(self):
        # _calculateNodeSize re-runs layoutNode first every time, so the base
        # width is always the un-widened one: the growth cannot compound.
        reserve = vc.reserve_width(300.0, 85.0)
        once = vc.widened_node_width(600.0, reserve)
        twice = vc.widened_node_width(600.0, reserve)
        self.assertEqual(once, twice)

    def test_no_reserve_means_no_widening(self):
        self.assertEqual(vc.widened_node_width(600.0, 0.0), 600.0)
        self.assertEqual(vc.reserve_width(0.0, 85.0), 0.0)

    def test_measured_width_grows_with_components_and_the_swatch(self):
        h = 85.0
        one = vc.measure_row_width(("1.0000",), _width, h)
        three = vc.measure_row_width(("1.0000", "2.0000", "3.0000"), _width, h)
        self.assertGreater(three, one)
        with_swatch = vc.measure_row_width(
            ("1.0000", "2.0000", "3.0000"), _width, h, has_swatch=True
        )
        self.assertAlmostEqual(with_swatch - three, vc.swatch_inset(h))

    def test_clamp_keeps_the_reserve_inside_the_agreed_band(self):
        h = 85.0
        self.assertAlmostEqual(
            vc.clamp_cell_width(1.0, h), vc.MIN_RESERVE_W_RATIO * h
        )
        self.assertAlmostEqual(
            vc.clamp_cell_width(1e6, h), vc.MAX_RESERVE_W_RATIO * h
        )
        self.assertAlmostEqual(vc.clamp_cell_width(5.0 * h, h), 5.0 * h)

    def test_a_node_too_narrow_for_its_pins_carries_no_cell(self):
        # nodePortWidth 40 -> getPortWidth() 20 -> the floor is nw >= 300.
        self.assertFalse(vc.node_is_wide_enough(200.0, 20.0))
        self.assertTrue(vc.node_is_wide_enough(600.0, 20.0))


@unittest.skipUnless(_HAS_CELL, "UsdNoodles.widgets.valueCell unavailable")
class TestRowSuppression(unittest.TestCase):
    def test_a_plain_node_shows_every_input_row(self):
        node = _Node(input_pins=["a", "b", "c"])
        self.assertEqual([r[0] for r in _rows_for(node)], ["a", "b", "c"])

    def test_a_collapsed_title_shows_nothing(self):
        node = _Node(input_pins=["a", "b"], title_collapsed=True)
        self.assertEqual(_rows_for(node), [])

    def test_group_headers_show_nothing_but_their_children_do(self):
        # Row kinds are 0=normal, 1=folded header, 2=unfolded header,
        # 3=child (noodles core/NodeData.h:48). Only the two header kinds
        # are suppressed; a child is a real property row and keeps its cell.
        node = _Node(
            input_pins=["foldedHdr", "openHdr", "child", "plain"],
            input_kinds=[1, 2, 3, 0],
        )
        self.assertEqual([r[0] for r in _rows_for(node)], ["child", "plain"])

    def test_a_slot_shared_with_an_output_row_loses_its_cell(self):
        # The output label is right-aligned into exactly the space the cell
        # wants, so the cell is the one that gives way.
        node = _Node(
            input_pins=["a", "b"],
            input_slots=[0, 1],
            output_pins=["out:r"],
            output_slots=[1],
        )
        self.assertEqual([r[0] for r in _rows_for(node)], ["a"])

    def test_no_reserved_width_means_no_cells(self):
        node = _Node(input_pins=["a"], cell_width=0.0)
        self.assertEqual(_rows_for(node), [])

    def test_rows_land_on_their_own_slot_bands(self):
        node = _Node(input_pins=["a", "b", "c"], port_start_y=100.0, line_height=85.0)
        rows = _rows_for(node)
        for pin, slot, rect in rows:
            row_top = 100.0 + slot * 85.0
            self.assertGreaterEqual(rect.top, row_top)
            self.assertLessEqual(rect.bottom, row_top + 85.0)

    def test_the_node_position_offsets_every_row(self):
        moved = _Node(input_pins=["a"], position=(1000.0, 2000.0))
        base = _Node(input_pins=["a"], position=(0.0, 0.0))
        moved_rect = _rows_for(moved)[0][2]
        base_rect = _rows_for(base)[0][2]
        self.assertAlmostEqual(moved_rect.left - base_rect.left, 1000.0)
        self.assertAlmostEqual(moved_rect.top - base_rect.top, 2000.0)


@unittest.skipUnless(_HAS_CELL, "UsdNoodles.widgets.valueCell unavailable")
class TestTheme(unittest.TestCase):
    """Every cell colour is DERIVED from the node theme, never fixed."""

    def _theme(self, high=90, low=86, accent=(1.0, 1.0, 0.117, 1.0)):
        return vc.build_theme(
            high,
            low,
            accent,
            (0.7, 0.8, 0.0, 1.0),
            (0.235, 0.235, 0.235, 1.0),
            (0.45, 0.62, 0.85, 1.0),
            (0.95, 0.72, 0.25, 1.0),
            (0.93, 0.93, 0.95, 1.0),
            (0.93, 0.93, 0.95, 0.42),
        )

    def test_the_body_is_the_node_background(self):
        theme = self._theme(high=90, low=86)
        self.assertAlmostEqual(theme.body, (90 + 86) * 0.5 / 255.0)

    def test_a_lighter_node_gives_lighter_cells(self):
        dark = self._theme(high=40, low=36)
        light = self._theme(high=200, low=196)
        self.assertLess(dark.fillAuthored[0], light.fillAuthored[0])
        self.assertLess(dark.hairline[0], light.hairline[0])
        self.assertLess(dark.popupBg[0], light.popupBg[0])

    def test_the_accent_follows_the_selection_stroke(self):
        theme = self._theme(accent=(0.1, 0.4, 0.9, 1.0))
        self.assertAlmostEqual(theme.accent[0], 0.1)
        self.assertAlmostEqual(theme.accent[2], 0.9)
        self.assertAlmostEqual(theme.fillAccent[2], 0.9)
        self.assertAlmostEqual(theme.selection[1], 0.4)

    def test_the_pill_is_a_tint_and_not_a_coat_of_paint(self):
        # Anything much above this buries the number underneath it.
        theme = self._theme()
        self.assertLess(theme.fillAuthored[3], 0.35)
        self.assertLess(theme.fillHover[3], 0.35)
        self.assertLess(theme.fillAccent[3], 0.20)

    def test_hover_lifts_and_the_resting_fill_sinks(self):
        theme = self._theme()
        self.assertGreater(theme.fillHover[0], theme.body)
        self.assertLess(theme.fillAuthored[0], theme.body)

    def test_authored_and_fallback_text_come_from_the_settings(self):
        theme = self._theme()
        self.assertEqual(theme.textNormal, (0.93, 0.93, 0.95, 1.0))
        self.assertEqual(theme.textDim, (0.93, 0.93, 0.95, 0.42))
        self.assertLess(theme.textDim[3], theme.textNormal[3])

    def test_the_bool_switch_wears_the_port_colours(self):
        theme = self._theme()
        self.assertEqual(theme.boolOn, (0.7, 0.8, 0.0, 1.0))
        self.assertEqual(theme.boolOff, (0.235, 0.235, 0.235, 1.0))

    def test_axis_tints_are_one_per_component_and_muted(self):
        self.assertEqual(len(vc.AXIS_COLORS), 4)
        self.assertEqual(vc.AXIS_LABELS[:3], ("x", "y", "z"))
        self.assertEqual(vc.COLOR_AXIS_LABELS[:3], ("r", "g", "b"))
        for color in vc.AXIS_COLORS:
            # Muted: nothing fully saturated next to a number.
            self.assertLess(max(color) - min(color), 0.5)


if __name__ == "__main__":
    unittest.main()
