#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
The Tab hotbox's list can be scrolled with a scrollbar, not only the wheel.

Two things are worth testing beyond "a rectangle is drawn". The first is that
the bar and the wheel agree about how far down the list goes -- two independent
notions of that disagree at exactly the bottom, which is where it shows. The
second is that grabbing the bar does not also pick the node behind it: the
track overlaps the item rows, so a press that fell through to row selection
would create a node and close the popup on the way to scrolling.

The widgets render through OpenGL, so these exercise geometry and event
handling directly rather than drawing anything.
"""

import unittest
from unittest.mock import MagicMock

try:
    from UsdNoodles.widgets.selectableList import (
        SelectableList,
        SelectableListItem,
    )
    from UsdNoodles.widgets.nodeCreationHotbox import (
        NodeCreationHotbox,
        NodeTypeGroup,
    )

    _has_widgets = True
except ImportError:
    _has_widgets = False


ROW = 24.0          # SelectableList.item_height
VISIBLE = 10


def _list(count, groups=False, visible=VISIBLE):
    widget = SelectableList(max_visible_items=visible)
    widget.set_items([
        SelectableListItem("item%02d" % i,
                           group=("group%d" % (i // 5)) if groups else "")
        for i in range(count)
    ])
    # What render() would record.
    rows = min(visible, widget._total_rows_for_items(0, count))
    widget.bounds = (100.0, 200.0, 300.0, rows * ROW)
    return widget


class _Alpha:
    """The one animated property OverlayWidget asks its animator for."""

    def __init__(self):
        self.speed = 8.0
        self.target = 0.0
        self.current = 0.0


class _Animator:
    """Enough animator for a widget to exist without a running frame loop."""

    def __init__(self):
        self.properties = []

    def addProperty(self):
        prop = _Alpha()
        self.properties.append(prop)
        return prop


class _Point:
    """Stand-in for QPoint."""

    def __init__(self, x, y):
        self._x, self._y = x, y

    def x(self):
        return self._x

    def y(self):
        return self._y


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class ScrollbarGeometryTest(unittest.TestCase):
    def test_no_bar_when_the_list_fits(self):
        """A full-length thumb is furniture that says nothing."""
        widget = _list(5)

        self.assertEqual(widget.max_scroll_offset(), 0)
        self.assertIsNone(widget.scrollbar_metrics())
        self.assertFalse(widget.scrollbar_hit(396.0, 210.0))

    def test_bar_appears_and_sits_on_the_right_edge(self):
        widget = _list(60)
        metrics = widget.scrollbar_metrics()
        self.assertIsNotNone(metrics)
        track_x, track_y, track_w, track_h, _ty, _th = metrics

        x, y, width, height = widget.bounds
        self.assertAlmostEqual(track_x + track_w, x + width)
        self.assertAlmostEqual(track_y, y)
        self.assertAlmostEqual(track_h, height)

    def test_thumb_is_proportional_and_stays_inside_the_track(self):
        widget = _list(60)
        _tx, track_y, _tw, track_h, thumb_y, thumb_h = widget.scrollbar_metrics()

        # 10 visible of 60 rows.
        self.assertAlmostEqual(thumb_h, track_h * 10.0 / 60.0, places=5)
        self.assertGreaterEqual(thumb_y, track_y)
        self.assertLessEqual(thumb_y + thumb_h, track_h + track_y + 1e-6)

    def test_a_huge_list_still_has_a_grabbable_thumb(self):
        """Proportional sizing alone gives a two-pixel thumb nobody can hit."""
        widget = _list(4000)
        _tx, _ty, _tw, track_h, _thy, thumb_h = widget.scrollbar_metrics()

        self.assertGreaterEqual(thumb_h, widget.scrollbar_min_thumb)
        self.assertLessEqual(thumb_h, track_h)

    def test_thumb_travels_from_top_to_bottom_with_the_offset(self):
        widget = _list(60)
        _tx, track_y, _tw, track_h, top_y, thumb_h = widget.scrollbar_metrics()
        self.assertAlmostEqual(top_y, track_y)

        widget.scroll_offset = widget.max_scroll_offset()
        _tx, _ty, _tw, _th, bottom_y, _thh = widget.scrollbar_metrics()
        self.assertAlmostEqual(bottom_y + thumb_h, track_y + track_h, places=5)

    def test_group_headers_count_toward_the_thumb(self):
        """Headers consume rows, so a grouped list scrolls further."""
        plain = _list(60)
        grouped = _list(60, groups=True)

        self.assertGreater(grouped.max_scroll_offset(),
                           plain.max_scroll_offset())
        self.assertIsNotNone(grouped.scrollbar_metrics())


class _Atlas:
    ascender = 0.8
    lineHeight = 1.2


class _TextRenderer:
    """Enough text renderer for the list to lay itself out."""

    font_atlas = _Atlas()

    def calculateTextWidth(self, text, size):
        return len(text) * size * 0.5

    def generateTextVertices(self, text, x, y, depth, size, out, node_index=0.0):
        out.append((text, x, y))

    def drawTextVertices(self, data, projection, color, disable_depth=False):
        pass


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class ScrollbarRenderTest(unittest.TestCase):
    """The bar must actually reach the draw call.

    _drawNodeVertices takes cornerRadius and strokeColor as REQUIRED
    positional arguments, and the hotbox wraps its whole render in a
    try/except that turns any failure into a Tf.Warn -- so a scrollbar drawn
    with the wrong arity is not an error anyone sees, it is simply a
    scrollbar that never appears. That is what these assert against.
    """

    def _render(self, count):
        widget = _list(count)
        quads = []
        draws = []

        def node_vertex(x, y, z, u, v, w, h, *rest):
            quads.append((x, y, w, h))
            return (x, y, w, h)

        def draw_fn(data, projection, corner_radius, stroke_color, generation=0):
            draws.append({"count": len(data), "radius": corner_radius,
                          "stroke": stroke_color, "generation": generation})

        widget.render(_TextRenderer(), 100.0, 200.0, 0.5, 300.0,
                      node_vertex, draw_fn, object(), alpha=1.0)
        return widget, quads, draws

    def test_a_fitting_list_draws_no_bar(self):
        widget, quads, draws = self._render(5)

        self.assertIsNotNone(widget.bounds, "render must record its bounds")
        bar_w = widget.scrollbar_width
        self.assertFalse([q for q in quads if q[2] == bar_w],
                         "a list that fits drew a scrollbar")

    def test_an_overflowing_list_draws_track_and_thumb(self):
        widget, quads, draws = self._render(60)

        bar_w = widget.scrollbar_width
        bars = [q for q in quads if q[2] == bar_w]
        if len(bars) != 12:
            print("  scrollbar quads reaching the vertex builder: %d "
                  "(expected 12 = two rects x 6 vertices)" % len(bars))
        self.assertEqual(len(bars), 12)

        # Two draw calls, six vertices each, and the thumb rounded.
        bar_draws = [d for d in draws if d["count"] == 6][-2:]
        self.assertEqual(len(bar_draws), 2)
        self.assertAlmostEqual(bar_draws[0]["radius"], 0.0)
        self.assertGreater(bar_draws[1]["radius"], 0.0)

    def test_rows_stop_short_of_the_bar(self):
        """A selected row's highlight must not run under the track."""
        widget, quads, _draws = self._render(60)

        track_x = widget.scrollbar_metrics()[0]
        bar_w = widget.scrollbar_width
        # Each rect emits six VERTICES, and the right-edge ones already sit
        # at rect_x + rect_w -- so the rightmost vertex is the row's right
        # edge, not x + w.
        rows = [q for q in quads if q[2] != bar_w]
        self.assertTrue(rows, "no row background was drawn")
        rightmost = max(x for x, _y, _w, _h in rows)
        if rightmost > track_x + 1e-6:
            print("  a row reaches x=%.1f but the track starts at %.1f -- the "
                  "highlight runs under the scrollbar" % (rightmost, track_x))
        self.assertLessEqual(rightmost, track_x + 1e-6)

    def test_render_survives_an_empty_filter_result(self):
        widget = _list(60)
        widget.set_filter("nothing matches this")
        widget.render(_TextRenderer(), 100.0, 200.0, 0.5, 300.0,
                      lambda *a: None, lambda *a, **k: None, object())
        self.assertIsNone(widget.scrollbar_metrics())


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class ScrollbarDragTest(unittest.TestCase):
    def setUp(self):
        self.widget = _list(60)
        (self.tx, self.ty, self.tw, self.th,
         self.thumb_y, self.thumb_h) = self.widget.scrollbar_metrics()
        self.travel = self.th - self.thumb_h

    def test_dragging_the_thumb_spans_the_whole_list(self):
        self.widget.begin_scroll_drag(self.thumb_y)

        self.widget.update_scroll_drag(self.ty + self.travel)
        self.assertEqual(self.widget.scroll_offset,
                         self.widget.max_scroll_offset())

        self.widget.update_scroll_drag(self.ty)
        self.assertEqual(self.widget.scroll_offset, 0)
        self.widget.end_scroll_drag()

    def test_overshoot_clamps_rather_than_wrapping(self):
        self.widget.begin_scroll_drag(self.thumb_y)

        self.widget.update_scroll_drag(self.ty + self.travel * 5)
        self.assertEqual(self.widget.scroll_offset,
                         self.widget.max_scroll_offset())
        self.widget.update_scroll_drag(self.ty - 500)
        self.assertEqual(self.widget.scroll_offset, 0)
        self.widget.end_scroll_drag()

    def test_grabbing_mid_thumb_does_not_teleport_it(self):
        """The thumb must not snap its own centre under the cursor."""
        self.widget.scroll_offset = 25
        _tx, _ty, _tw, _th, thumb_y, thumb_h = self.widget.scrollbar_metrics()

        self.widget.begin_scroll_drag(thumb_y + thumb_h / 2)
        self.widget.update_scroll_drag(thumb_y + thumb_h / 2)

        self.assertEqual(self.widget.scroll_offset, 25)
        self.widget.end_scroll_drag()

    def test_clicking_the_bare_track_jumps_there(self):
        self.widget.scroll_offset = 0

        self.widget.begin_scroll_drag(self.ty + self.th - 2)

        self.assertGreater(self.widget.scroll_offset, 0)
        self.widget.end_scroll_drag()

    def test_the_bar_and_the_wheel_agree_on_the_limit(self):
        """Two notions of "how far down" disagree exactly at the bottom."""
        self.widget.scroll_offset = 0
        self.widget.scroll(10 ** 6)
        by_wheel = self.widget.scroll_offset

        # Back to the top, and re-read: the thumb moved with the offset, so
        # the setUp geometry no longer describes where it is.
        self.widget.scroll_offset = 0
        _tx, track_y, _tw, track_h, thumb_y, thumb_h = \
            self.widget.scrollbar_metrics()
        self.widget.begin_scroll_drag(thumb_y)
        self.widget.update_scroll_drag(track_y + track_h - thumb_h)
        self.widget.end_scroll_drag()

        self.assertEqual(by_wheel, self.widget.scroll_offset)
        self.assertEqual(by_wheel, self.widget.max_scroll_offset())

    def test_selection_is_pulled_back_into_view(self):
        """Dragging moves the window; a selection left outside comes back."""
        self.widget.selected_index = 0

        self.widget.begin_scroll_drag(self.thumb_y)
        self.widget.update_scroll_drag(self.ty + self.travel)
        self.widget.end_scroll_drag()

        visible = self.widget._items_fitting_in_rows(
            self.widget.scroll_offset, self.widget.max_visible_items)
        self.assertGreaterEqual(self.widget.selected_index,
                                self.widget.scroll_offset)
        self.assertLess(self.widget.selected_index,
                        self.widget.scroll_offset + visible)

    def test_dragging_flag_clears_on_release(self):
        self.widget.begin_scroll_drag(self.thumb_y)
        self.assertTrue(self.widget.is_scroll_dragging())

        self.assertTrue(self.widget.end_scroll_drag())
        self.assertFalse(self.widget.is_scroll_dragging())
        self.assertFalse(self.widget.end_scroll_drag())


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class SmoothScrollTest(unittest.TestCase):
    """Scrolling is measured in pixels, not whole items.

    An integer item offset moves the list a whole row at a time however
    finely the events arrive, which is the step you feel on a trackpad. And
    the wheel handler used to integer-divide angleDelta by a 120-unit notch,
    so a trackpad's single-digit deltas floored to zero and nothing moved at
    all until enough had been discarded to cross a notch.
    """

    def setUp(self):
        self.widget = _list(60)

    def test_a_one_pixel_delta_moves_the_list(self):
        """The dead zone is the whole complaint."""
        self.widget.scroll_px = 0.0

        for delta in (1.0, 2.0, 3.0):
            before = self.widget.scroll_px
            self.widget.scroll_pixels(delta)
            self.assertGreater(self.widget.scroll_px, before,
                               "a %gpx delta was swallowed" % delta)

    def test_sub_row_positions_are_reachable(self):
        self.widget.scroll_px = 0.0
        self.widget.scroll_pixels(7.0)

        first_row, sub = self.widget.first_visible_row()
        self.assertEqual(first_row, 0)
        self.assertAlmostEqual(sub, 7.0)

    def test_a_decelerating_stream_keeps_moving(self):
        """macOS sends the momentum tail as ever-smaller deltas."""
        self.widget.scroll_px = 0.0
        positions = []
        for delta in (40.0, 28.0, 19.0, 12.0, 7.0, 4.0, 2.0, 1.0):
            self.widget.scroll_pixels(delta)
            positions.append(self.widget.scroll_px)

        self.assertEqual(positions, sorted(positions))
        self.assertEqual(len(set(positions)), len(positions),
                         "the tail of the momentum stopped moving the list")

    def test_pixel_scrolling_clamps_at_both_ends(self):
        self.widget.scroll_pixels(10 ** 9)
        self.assertAlmostEqual(self.widget.scroll_px,
                               self.widget.max_scroll_px())
        self.widget.scroll_pixels(-10 ** 9)
        self.assertAlmostEqual(self.widget.scroll_px, 0.0)

    def test_hit_testing_follows_the_sub_row_offset(self):
        """Rows move under the cursor, so the hit test has to move with them."""
        plain = _list(60)
        plain.scroll_px = 0.0
        at_rest = plain.item_index_at_pixel(30.0)

        plain.scroll_px = 18.0  # 30 + 18 crosses into the next row
        shifted = plain.item_index_at_pixel(30.0)

        self.assertIsNotNone(at_rest)
        self.assertEqual(shifted, at_rest + 1)

    def test_group_headers_are_not_selectable(self):
        grouped = _list(20, groups=True)
        grouped.scroll_px = 0.0

        self.assertIsNone(grouped.item_index_at_pixel(5.0), "a header was hit")
        self.assertEqual(grouped.item_index_at_pixel(30.0), 0)

    def test_scroll_offset_still_reads_and_writes(self):
        """It is derived now, but every caller still speaks item indices."""
        self.widget.scroll_offset = 20
        self.assertEqual(self.widget.scroll_offset, 20)
        self.assertGreater(self.widget.scroll_px, 0.0)

        self.widget.scroll_offset = 0
        self.assertEqual(self.widget.scroll_offset, 0)
        self.assertAlmostEqual(self.widget.scroll_px, 0.0)

    def test_scrolling_to_an_item_brings_its_header(self):
        """Otherwise offset 0 hides the first group header immediately."""
        grouped = _list(60, groups=True)
        grouped.scroll_offset = 0

        self.assertAlmostEqual(grouped.scroll_px, 0.0)
        self.assertEqual(grouped._rows[0][0], "header")


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class StableHeightTest(unittest.TestCase):
    """The popup must not resize while it is being scrolled.

    calculate_size measured the rows actually spanned by a window starting
    at scroll_offset. Group headers occupy rows, so how many rows a 20-item
    budget spends depends on how many headers land inside the window --
    slide it past one and the total flips between 19 and 20 rows. The popup
    is laid out from its bottom edge, so the top edge jumped by item_height
    at whatever scroll positions the group boundaries happened to fall on.
    """

    def setUp(self):
        # Uneven groups, the way a real node menu is. Even groups hide this:
        # the header cadence has to beat against the window for it to show.
        items = []
        for group, count in enumerate([5, 3, 9, 4, 7, 2, 11, 6]):
            items.extend(
                SelectableListItem("node%02d_%02d" % (group, i),
                                   group="Group%d" % group)
                for i in range(count))
        self.widget = SelectableList(max_visible_items=20)
        self.widget.set_items(items)
        self.renderer = _TextRenderer()

    def _heights(self):
        seen = []
        limit = int(self.widget.max_scroll_px())
        for px in range(0, limit + 1, 4):
            self.widget.scroll_px = float(px)
            seen.append((px, self.widget.calculate_size(self.renderer)[1]))
        return seen

    def test_height_never_changes_while_scrolling(self):
        seen = self._heights()
        distinct = sorted({h for _px, h in seen})

        if len(distinct) > 1:
            first = seen[0][1]
            at = next(px for px, h in seen if h != first)
            print("  height changed from %.0f to %.0f at scroll_px=%d -- the "
                  "popup's top edge moves %+.0fpx mid-scroll"
                  % (first, dict(seen)[at], at, first - dict(seen)[at]))
        self.assertEqual(len(distinct), 1,
                         "popup height varies with scroll: %s" % distinct)

    def test_width_never_changes_while_scrolling(self):
        self.widget.scroll_px = 0.0
        widths = []
        limit = int(self.widget.max_scroll_px())
        for px in range(0, limit + 1, 4):
            self.widget.scroll_px = float(px)
            widths.append(self.widget.calculate_size(self.renderer)[0])

        self.assertEqual(len(set(widths)), 1)

    def test_height_is_the_band_render_and_the_clip_agree_on(self):
        """Layout, render and scissor must come from one expression."""
        _w, h = self.widget.calculate_size(self.renderer)
        band = min(self.widget.view_height(), self.widget.content_height())

        self.assertAlmostEqual(h, band + self.widget.padding * 2)

    def test_height_still_shrinks_for_a_short_list(self):
        """Scroll-independent must not mean fixed -- a stub list stays small."""
        self.widget.set_items([SelectableListItem("only")])
        _w, h = self.widget.calculate_size(self.renderer)

        self.assertLess(h, self.widget.view_height())

    def test_height_is_stable_across_a_filter_that_shortens_the_list(self):
        """Filtering may resize; scrolling the filtered result may not."""
        self.widget.set_filter("node02")
        heights = []
        limit = int(self.widget.max_scroll_px())
        for px in range(0, limit + 1, 4):
            self.widget.scroll_px = float(px)
            heights.append(self.widget.calculate_size(self.renderer)[1])

        self.assertEqual(len(set(heights)), 1)


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class ClipAlignmentTest(unittest.TestCase):
    """render, the clip rectangle and the hit test must share one geometry.

    calculate_size reserves view_height() + padding*2 for the list, and the
    popup lays it out at that size. render drew rows flush at the layout
    origin instead of inside that reservation, so the top row butted against
    the search field and was sliced there while the bottom finished padding*2
    short -- which is what "it still clips at the top and bottom" was.
    """

    def setUp(self):
        self.widget = _list(180, groups=True, visible=20)
        self.list_y = 100.0
        self.alloc_w, self.alloc_h = self.widget.calculate_size(_TextRenderer())
        self.top = self.widget.content_top(self.list_y)
        self.clip_h = min(self.widget.view_height(),
                          self.widget.content_height())

    def test_rows_sit_inside_the_space_the_popup_reserved(self):
        self.assertGreaterEqual(self.top, self.list_y)
        self.assertLessEqual(self.top + self.clip_h,
                             self.list_y + self.alloc_h + 1e-6)

    def test_the_reservation_is_spent_as_padding_on_both_sides(self):
        above = self.top - self.list_y
        below = (self.list_y + self.alloc_h) - (self.top + self.clip_h)

        self.assertAlmostEqual(above, self.widget.padding)
        self.assertAlmostEqual(below, self.widget.padding)

    def test_at_rest_the_first_row_is_not_cut(self):
        """Scrolled to the top, nothing should be sliced at all."""
        self.widget.scroll_px = 0.0
        _row, sub = self.widget.first_visible_row()

        self.assertAlmostEqual(sub, 0.0)

    def test_render_draws_no_row_above_the_clip(self):
        quads = []
        self.widget.scroll_px = 0.0
        self.widget.render(
            _TextRenderer(), 100.0, self.list_y, 0.5, 300.0,
            lambda x, y, z, u, v, w, h, *rest: quads.append((x, y, w, h)),
            lambda *a, **k: None, object(), alpha=1.0)

        tops = [y for _x, y, _w, _h in quads]
        self.assertTrue(tops, "nothing was drawn")
        if min(tops) < self.top - 1e-6:
            print("  a row was drawn at y=%.1f, above the clip top %.1f -- it "
                  "will be sliced" % (min(tops), self.top))
        self.assertGreaterEqual(min(tops), self.top - 1e-6)

    def test_bounds_are_the_clipped_area_not_the_allocation(self):
        """The scrollbar hangs off bounds, so it inherits the same inset."""
        self.widget.scroll_px = 0.0
        self.widget.render(
            _TextRenderer(), 100.0, self.list_y, 0.5, 300.0,
            lambda *a: None, lambda *a, **k: None, object(), alpha=1.0)

        _x, y, _w, h = self.widget.bounds
        self.assertAlmostEqual(y, self.top)
        self.assertAlmostEqual(h, self.clip_h)


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class KeyboardNavigationTest(unittest.TestCase):
    """Arrowing through a long list must still drag the window along.

    Run on a watchdog thread on purpose. The auto-scroll used to be a
    `while self.scroll_offset < self.selected_index: self.scroll_offset += 1`
    search, and once scroll_offset became a derived property that clamps at
    the bottom, the write stopped moving it and the read kept returning the
    same index -- an infinite loop. A plain assertion cannot catch that: the
    suite simply stops. This turns it into a failure.
    """

    def _bounded(self, fn, seconds=10.0):
        import threading
        box = {}

        def run():
            try:
                box["value"] = fn()
            except Exception as error:  # pragma: no cover - surfaced below
                box["error"] = error

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        thread.join(seconds)
        if thread.is_alive():
            raise AssertionError(
                "selection auto-scroll did not terminate within %gs -- it is "
                "searching for a scroll offset that can no longer move"
                % seconds)
        if "error" in box:
            raise box["error"]
        return box["value"]

    def _walk(self, groups):
        widget = _list(60, groups=groups)

        def go():
            widget.selected_index = 0
            widget.scroll_px = 0.0
            for _ in range(59):
                widget.move_selection(1)
            at_end = (widget.selected_index, widget.scroll_px)
            for _ in range(59):
                widget.move_selection(-1)
            return at_end, (widget.selected_index, widget.scroll_px)

        return widget, self._bounded(go)

    def test_arrowing_down_scrolls_and_terminates(self):
        widget, ((sel, px), _back) = self._walk(groups=False)

        self.assertEqual(sel, 59)
        self.assertGreater(px, 0.0, "the window never followed the selection")
        row = widget._row_of_item(sel)
        self.assertLessEqual(widget.scroll_px, row * widget.item_height + 1e-6)

    def test_arrowing_back_up_returns_to_the_top(self):
        _widget, (_end, (sel, px)) = self._walk(groups=False)

        self.assertEqual(sel, 0)
        self.assertAlmostEqual(px, 0.0)

    def test_group_headers_do_not_break_the_walk(self):
        widget, ((sel, px), (back_sel, back_px)) = self._walk(groups=True)

        self.assertEqual(sel, 59)
        self.assertGreater(px, 0.0)
        self.assertEqual(back_sel, 0)
        self.assertAlmostEqual(back_px, 0.0)

    def test_the_selected_row_ends_up_inside_the_window(self):
        widget, _ = self._walk(groups=True)

        row = widget._row_of_item(widget.selected_index)
        top = row * widget.item_height
        self.assertGreaterEqual(top, widget.scroll_px - 1e-6)
        self.assertLessEqual(top + widget.item_height,
                             widget.scroll_px + widget.view_height() + 1e-6)


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class PopupWidthTest(unittest.TestCase):
    """The box must not resize under the cursor while the list scrolls."""

    def test_width_is_the_same_at_every_scroll_position(self):
        widget = _list(60)
        # Names of deliberately different lengths, so a window-based
        # measurement would visibly change.
        for i, item in enumerate(widget.filtered_items):
            item.name = "n" * (4 + (i % 30))
        widget.apply_filter()

        widths = []
        for px in (0.0, 137.0, 400.0, widget.max_scroll_px()):
            widget.scroll_px = px
            widths.append(round(widget.calculate_size(_TextRenderer())[0], 3))

        if len(set(widths)) != 1:
            print("  popup width across scroll positions: %r -- the box "
                  "resizes as rows come into view" % (widths,))
        self.assertEqual(len(set(widths)), 1)

    def test_width_still_tracks_the_filter(self):
        """Adapting to the filter is the part worth keeping."""
        widget = _list(60)
        for i, item in enumerate(widget.filtered_items):
            item.name = "short" if i else "a" * 60
        widget.apply_filter()
        wide = widget.calculate_size(_TextRenderer())[0]

        widget.set_filter("short")
        narrow = widget.calculate_size(_TextRenderer())[0]

        self.assertLess(narrow, wide)


@unittest.skipUnless(_has_widgets, "usdNoodles widgets not available")
class HotboxScrollbarEventsTest(unittest.TestCase):
    """The bar overlaps the rows, so press order is the whole story."""

    def setUp(self):
        self.hotbox = NodeCreationHotbox(_Animator())
        self.hotbox.configure(
            [NodeTypeGroup("Group%d" % (i // 5), ["Node%02d" % i])
             for i in range(60)],
            max_visible=VISIBLE)
        self.created = []
        self.hotbox.on_create = lambda data, pos: self.created.append(data)

        # What show() + a rendered frame would establish. show() only moves
        # the alpha TARGET; without a frame loop to advance it, is_visible
        # stays False and every handler below returns early.
        self.hotbox.show()
        self.hotbox._alpha.current = 1.0
        self.hotbox._bounds = (100.0, 100.0, 300.0, 400.0)
        widget = self.hotbox._selectable_list
        widget.bounds = (100.0, 200.0, 300.0, VISIBLE * ROW)
        self.metrics = widget.scrollbar_metrics()
        self.assertIsNotNone(self.metrics, "fixture must overflow")

    def test_pressing_the_scrollbar_does_not_create_a_node(self):
        track_x, track_y, track_w, _th, _thy, _thh = self.metrics

        handled = self.hotbox.handle_mouse_press(
            _Point(track_x + track_w / 2, track_y + 5))

        self.assertTrue(handled)
        if self.created:
            print("  pressing the scrollbar created %r -- the press fell "
                  "through to the row behind the track" % (self.created,))
        self.assertEqual(self.created, [])
        self.assertTrue(self.hotbox.is_visible, "the popup closed on a scroll")
        self.assertTrue(self.hotbox._selectable_list.is_scroll_dragging())

    def test_pressing_a_row_left_of_the_bar_still_creates(self):
        """The guard must not swallow ordinary clicks."""
        track_x, track_y, _tw, _th, _thy, _thh = self.metrics

        self.hotbox.handle_mouse_press(_Point(track_x - 40, track_y + 5))

        self.assertEqual(len(self.created), 1)

    def test_a_drag_keeps_following_outside_the_popup(self):
        track_x, track_y, track_w, track_h, thumb_y, _thh = self.metrics
        self.hotbox.handle_mouse_press(
            _Point(track_x + track_w / 2, thumb_y + 1))

        # Well outside _bounds on both axes.
        handled = self.hotbox.handle_mouse_move(_Point(-500, track_y + track_h))

        self.assertTrue(handled)
        self.assertGreater(self.hotbox._selectable_list.scroll_offset, 0)
        self.assertEqual(self.created, [])

    def test_release_ends_the_drag(self):
        track_x, track_y, track_w, _th, thumb_y, _thh = self.metrics
        self.hotbox.handle_mouse_press(
            _Point(track_x + track_w / 2, thumb_y + 1))

        self.assertTrue(self.hotbox.handle_mouse_release(_Point(0, 0)))
        self.assertFalse(self.hotbox._selectable_list.is_scroll_dragging())
        self.assertFalse(self.hotbox.handle_mouse_release(_Point(0, 0)))

    def test_reopening_never_inherits_a_held_thumb(self):
        """Clicking outside dismisses without a release."""
        track_x, track_y, track_w, _th, thumb_y, _thh = self.metrics
        self.hotbox.handle_mouse_press(
            _Point(track_x + track_w / 2, thumb_y + 1))
        self.assertTrue(self.hotbox._selectable_list.is_scroll_dragging())

        self.hotbox.hide()
        self.hotbox._alpha.current = 0.0
        self.hotbox.show()
        self.hotbox._alpha.current = 1.0

        self.assertFalse(self.hotbox._selectable_list.is_scroll_dragging())


try:
    from UsdNoodles.graphView import GraphView

    _has_graph_view = True
except ImportError:
    _has_graph_view = False


class _Delta:
    def __init__(self, y):
        self._y = y

    def y(self):
        return self._y


class _Wheel:
    """A QWheelEvent stand-in carrying either resolution."""

    def __init__(self, pixels=0, degrees=0):
        self._pixels = _Delta(pixels)
        self._degrees = _Delta(degrees)

    def pixelDelta(self):
        return self._pixels

    def angleDelta(self):
        return self._degrees


@unittest.skipUnless(_has_graph_view and _has_widgets, "graphView unavailable")
class WheelRoutingTest(unittest.TestCase):
    """Where the choppiness actually lived.

    The old handler was `-angleDelta().y() // 120`. A trackpad's angleDelta
    arrives in single digits, so that floored to 0 and the list did not move
    until enough events had been thrown away to cross a 120-unit notch --
    then it jumped a whole row.
    """

    def setUp(self):
        self.hotbox = NodeCreationHotbox(_Animator())
        self.hotbox.configure(
            [NodeTypeGroup("G%d" % (i // 5), ["N%02d" % i]) for i in range(60)],
            max_visible=VISIBLE)
        self.hotbox.show()
        self.hotbox._alpha.current = 1.0

        self.view = MagicMock()
        self.view.nodeCreationHotbox = self.hotbox

    def _wheel(self, **kwargs):
        GraphView.wheelEvent(self.view, _Wheel(**kwargs))

    def test_a_small_trackpad_delta_scrolls(self):
        self.hotbox._selectable_list.scroll_px = 0.0

        # What a trackpad actually sends: pixels, and few of them.
        self._wheel(pixels=-3, degrees=-2)

        moved = self.hotbox._selectable_list.scroll_px
        if moved <= 0:
            print("  a 3px trackpad delta moved the list %r -- it is being "
                  "quantised away" % moved)
        self.assertGreater(moved, 0.0)

    def test_pixels_win_over_degrees(self):
        """Both arrive together on macOS; the pixels are the precise one."""
        self.hotbox._selectable_list.scroll_px = 0.0
        self._wheel(pixels=-10, degrees=-120)

        self.assertAlmostEqual(self.hotbox._selectable_list.scroll_px, 10.0)

    def test_a_real_wheel_notch_still_moves_three_rows(self):
        self.hotbox._selectable_list.scroll_px = 0.0
        self._wheel(pixels=0, degrees=-120)

        row = self.hotbox._selectable_list.row_height()
        self.assertAlmostEqual(self.hotbox._selectable_list.scroll_px, 3 * row)

    def test_scrolling_up_at_the_top_stays_put(self):
        self.hotbox._selectable_list.scroll_px = 0.0
        self._wheel(pixels=40)
        self.assertAlmostEqual(self.hotbox._selectable_list.scroll_px, 0.0)


if __name__ == "__main__":
    unittest.main()
