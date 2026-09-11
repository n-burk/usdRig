#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#

#

"""
Reusable scrollable list widget with selection and filtering support.

Provides a list widget that can display grouped items, supports selection,
filtering, and scrolling through large lists.
"""


class SelectableListItem:
    """
    A single item in the selectable list.

    Attributes:
        name: Display name of the item
        group: Group/category name (for grouping items)
        data: Arbitrary data payload for callbacks
    """

    def __init__(self, name, group="", data=None):
        self.name = name
        self.group = group
        self.data = data if data is not None else name


class SelectableList:
    """
    Scrollable list with selection and filtering.

    Features:
    - Grouped items with headers
    - Selection highlighting
    - Keyboard navigation (up/down with scrolling)
    - Text-based filtering with swappable algorithms
    - Automatic scroll-to-selected
    """

    def __init__(self, max_visible_items=10):
        """
        Initialize the selectable list.

        Args:
            max_visible_items: Maximum number of items to display at once
        """
        self.items = []
        self.filtered_items = []
        self.selected_index = 0

        # Scrolling is measured in PIXELS, not items.
        #
        # An integer item offset can only ever move the list a whole row at a
        # time, which is the step you feel on a trackpad no matter how finely
        # the events arrive. Every row -- item or group header -- is
        # item_height tall, so a flat row list makes the pixel<->row mapping
        # linear and the sub-row offset is just a remainder.
        #
        # scroll_offset survives as a derived property, because the hit test,
        # the selection clamp and the callers all speak in item indices.
        self.scroll_px = 0.0
        self._rows = []  # ("header", group) | ("item", filtered_index)

        # The one row budget. view_height() derives from it, and every other
        # measurement -- layout, render, clip, hit test -- derives from that.
        self.max_visible_items = max_visible_items
        self.filter_text = ""
        self.filter_fn = self.substring_filter

        # Visual configuration
        self.font_size = 16.0
        self.item_height = 24.0
        self.group_header_height = 24.0  # Same as item_height for constant sizing
        self.padding = 8.0
        self.item_color = (0.85, 0.85, 0.85, 1.0)
        self.selected_color = (0.3, 0.5, 0.8, 1.0)
        self.group_header_color = (0.6, 0.6, 0.7, 1.0)
        self.group_header_bg_color = (
            0.08,
            0.08,
            0.10,
            0.95,
        )  # Slightly darker background
        self.background_color = (0.1, 0.1, 0.12, 0.95)

        # Scrollbar. Drawn only when the list does not fit, because a track
        # with a full-length thumb is furniture that says nothing.
        self.scrollbar_width = 8.0
        self.scrollbar_gap = 2.0
        self.scrollbar_min_thumb = 24.0
        self.scrollbar_track_color = (0.16, 0.16, 0.19, 0.95)
        self.scrollbar_thumb_color = (0.42, 0.44, 0.50, 1.0)
        self.scrollbar_thumb_active_color = (0.55, 0.62, 0.78, 1.0)

        # Mouse hover support
        self.hover_index = -1
        self.bounds = None  # (x, y, width, height) of last render

        # Scrollbar drag. _scroll_drag_grab is where in the thumb the press
        # landed, so the thumb does not jump its own centre under the cursor
        # on the first pixel of movement.
        self._scroll_dragging = False
        self._scroll_drag_grab = 0.0

    def set_items(self, items):
        """
        Populate the list with items.

        Args:
            items: List of SelectableListItem objects
        """
        self.items = items
        self.apply_filter()

    def set_filter(self, text):
        """
        Apply a filter to the list.

        Args:
            text: Filter text to match against item names
        """
        self.filter_text = text.lower()
        self.apply_filter()

        # Reset selection to first item
        self.selected_index = 0
        self.scroll_offset = 0

    def apply_filter(self):
        """Apply the current filter to items."""
        if not self.filter_text:
            self.filtered_items = self.items[:]
        else:
            self.filtered_items = [
                item
                for item in self.items
                if self.filter_fn(self.filter_text, item.name.lower())
            ]
        # The row layout IS the filtered list, flattened -- rebuilding it
        # here is what keeps the two from drifting apart.
        self._rebuild_rows()

    # ---- row layout ---------------------------------------------------

    def _rebuild_rows(self):
        """Flatten filtered_items into the rows that actually get drawn."""
        rows = []
        last_group = None
        for index, item in enumerate(self.filtered_items):
            if item.group and item.group != last_group:
                rows.append(("header", item.group))
                last_group = item.group
            rows.append(("item", index))
        self._rows = rows
        self.scroll_px = max(0.0, min(self.scroll_px, self.max_scroll_px()))

    def content_top(self, list_y):
        """Where the first row is drawn, given the list's layout origin.

        The clip rectangle and the hit test both need this, and all three
        disagreeing by a padding is exactly what made the list look cut.
        """
        return list_y + self.padding

    def row_height(self):
        """Every row is one height -- items and headers alike."""
        return self.item_height

    def view_height(self):
        """Pixel height of the scroll window."""
        return self.max_visible_items * self.item_height

    def content_height(self):
        return len(self._rows) * self.item_height

    def max_scroll_px(self):
        return max(0.0, self.content_height() - self.view_height())

    def scroll_pixels(self, dy):
        """Scroll by dy pixels (positive = down). Returns True if it moved."""
        before = self.scroll_px
        self.scroll_px = max(0.0, min(self.scroll_px + float(dy),
                                      self.max_scroll_px()))
        if self.scroll_px != before:
            self._clamp_selection_into_view()
        return self.scroll_px != before

    def first_visible_row(self):
        """(row index, sub-row pixel offset) the render starts at."""
        row = int(self.scroll_px // self.item_height)
        return row, self.scroll_px - row * self.item_height

    def row_at_pixel(self, y_from_list_top):
        """The row index under a y offset measured from the list's top."""
        return int((y_from_list_top + self.scroll_px) // self.item_height)

    def item_index_at_pixel(self, y_from_list_top):
        """The filtered item index under a y offset, or None on a header."""
        row = self.row_at_pixel(y_from_list_top)
        if 0 <= row < len(self._rows):
            kind, payload = self._rows[row]
            if kind == "item":
                return payload
        return None

    def _row_of_item(self, index):
        for row, (kind, payload) in enumerate(self._rows):
            if kind == "item" and payload == index:
                return row
        return None

    def _clamp_selection_into_view(self):
        """Keep the selection inside the window the user is looking at."""
        if not self.filtered_items:
            return
        row = self._row_of_item(self.selected_index)
        if row is None:
            return
        top = self.scroll_px
        bottom = top + self.view_height()
        row_top = row * self.item_height
        if row_top < top:
            visible = self.item_index_at_pixel(0)
            if visible is not None:
                self.selected_index = visible
        elif row_top + self.item_height > bottom:
            visible = self.item_index_at_pixel(self.view_height() - 1)
            if visible is not None:
                self.selected_index = visible

    @property
    def scroll_offset(self):
        """Index of the first item at or below the scroll position.

        Kept because the hit test, the selection clamp and every caller are
        written in item indices; only the storage moved to pixels.
        """
        first_row, _sub = self.first_visible_row()
        for kind, payload in self._rows[first_row:]:
            if kind == "item":
                return payload
        return max(0, len(self.filtered_items) - 1)

    @scroll_offset.setter
    def scroll_offset(self, index):
        row = self._row_of_item(index)
        if row is None:
            self.scroll_px = 0.0
            return
        # Bring the item's own group header with it. Without this,
        # scroll_offset = 0 lands one row down and the first header is
        # scrolled off before the list has moved at all.
        if row > 0 and self._rows[row - 1][0] == "header":
            row -= 1
        self.scroll_px = max(0.0, min(row * self.item_height,
                                      self.max_scroll_px()))

    def _items_fitting_in_rows(self, start, max_rows):
        """Count how many items fit starting from `start` within `max_rows` rows.

        Group headers consume an extra row when a new group begins.

        Returns:
            Number of items (not rows) that can be rendered.
        """
        rows_used = 0
        items_fit = 0
        last_group = None
        for i in range(start, len(self.filtered_items)):
            item = self.filtered_items[i]
            rows_needed = 1  # the item itself
            if item.group and item.group != last_group:
                rows_needed += 1  # group header
            if rows_used + rows_needed > max_rows:
                break
            rows_used += rows_needed
            last_group = item.group
            items_fit += 1
        return items_fit

    def _total_rows_for_items(self, start, end):
        """Count total rendered rows (items + group headers) for a range.

        Args:
            start: Starting item index (inclusive)
            end: Ending item index (exclusive)

        Returns:
            Total number of rows including group headers.
        """
        end = min(end, len(self.filtered_items))
        rows = 0
        last_group = None
        for i in range(start, end):
            item = self.filtered_items[i]
            if item.group and item.group != last_group:
                rows += 1  # group header
                last_group = item.group
            rows += 1  # the item
        return rows

    def move_selection(self, delta):
        """
        Move the selection up or down.

        Args:
            delta: Number of items to move (-1 for up, 1 for down)
        """
        if not self.filtered_items:
            return

        self.selected_index = max(
            0, min(self.selected_index + delta, len(self.filtered_items) - 1)
        )

        self.scroll_selection_into_view()

    def scroll_selection_into_view(self):
        """Move the window the least amount that shows the selected row.

        Computed, not searched. This used to walk `scroll_offset += 1` until
        the selection fitted, which cannot terminate now that scroll_offset
        is derived from the pixel position and clamps at the bottom: past the
        last scrollable row the write is a no-op, the read returns the same
        index, and the loop spins forever. In pixels the answer is
        arithmetic -- the row's top, or its bottom minus one window.
        """
        row = self._row_of_item(self.selected_index)
        if row is None:
            return
        row_top = row * self.item_height
        # Bring a group header with the first row of its group, so arrowing
        # up to the top of a group does not hide the label for it.
        if row > 0 and self._rows[row - 1][0] == "header":
            row_top -= self.item_height

        if row_top < self.scroll_px:
            self.scroll_px = row_top
        else:
            row_bottom = row * self.item_height + self.item_height
            if row_bottom > self.scroll_px + self.view_height():
                self.scroll_px = row_bottom - self.view_height()
        self.scroll_px = max(0.0, min(self.scroll_px, self.max_scroll_px()))

    def scroll(self, steps):
        """
        Scroll the list by a number of items.

        Args:
            steps: Number of items to scroll (positive = down, negative = up)
        """
        if not self.filtered_items:
            return

        self.scroll_pixels(steps * self.item_height)


    def max_scroll_offset(self):
        """The largest scroll_offset that still shows content.

        The last offset where all remaining items (plus their group headers)
        still exceed the visible row budget -- past that there is nothing
        below to scroll to. Extracted from scroll() so the scrollbar sizes
        its thumb against the same number the wheel scrolls against; two
        independent notions of "how far down can this go" would disagree at
        exactly the bottom of the list, which is where it shows.
        """
        limit = self.max_scroll_px()
        if limit <= 0:
            return 0
        row = int(limit // self.item_height)
        for kind, payload in self._rows[row:]:
            if kind == "item":
                return payload
        return max(0, len(self.filtered_items) - 1)

    def scrollbar_metrics(self):
        """Track and thumb geometry, or None when the list fits.

        Returns (track_x, track_y, track_w, track_h, thumb_y, thumb_h) in the
        same screen coordinates the last render used. None means no bar is
        drawn and none can be hit -- the caller does not have to ask whether
        the list overflows, because that IS this returning None.
        """
        if not self.bounds or not self.filtered_items:
            return None
        if self.max_scroll_px() <= 0:
            return None

        x, y, width, height = self.bounds
        track_w = self.scrollbar_width
        track_x = x + width - track_w
        track_y = y
        track_h = height
        if track_h <= 0:
            return None

        # Proportional, then floored: a 400-item list would otherwise give a
        # two-pixel thumb nobody can grab.
        content_h = max(1.0, self.content_height())
        thumb_h = max(self.scrollbar_min_thumb,
                      track_h * self.view_height() / content_h)
        thumb_h = min(thumb_h, track_h)

        # From pixels, not from the item index: the thumb then moves with the
        # same resolution the list does, instead of snapping a row at a time
        # under a smoothly moving list.
        travel = track_h - thumb_h
        limit = self.max_scroll_px()
        fraction = (self.scroll_px / limit) if limit > 0 else 0.0
        thumb_y = track_y + travel * max(0.0, min(1.0, fraction))
        return (track_x, track_y, track_w, track_h, thumb_y, thumb_h)

    def scrollbar_hit(self, px, py):
        """True when (px, py) is on the scrollbar track."""
        metrics = self.scrollbar_metrics()
        if not metrics:
            return False
        track_x, track_y, track_w, track_h, _thumb_y, _thumb_h = metrics
        return (track_x <= px <= track_x + track_w
                and track_y <= py <= track_y + track_h)

    def begin_scroll_drag(self, py):
        """Grab the thumb, or jump to the clicked position on the track."""
        metrics = self.scrollbar_metrics()
        if not metrics:
            return False
        _tx, track_y, _tw, track_h, thumb_y, thumb_h = metrics
        if thumb_y <= py <= thumb_y + thumb_h:
            self._scroll_drag_grab = py - thumb_y
        else:
            # Clicking the track centres the thumb where you clicked, which
            # is what makes a long list navigable without dragging at all.
            self._scroll_drag_grab = thumb_h * 0.5
            self._scroll_to_thumb_top(py - self._scroll_drag_grab,
                                      track_y, track_h, thumb_h)
        self._scroll_dragging = True
        return True

    def update_scroll_drag(self, py):
        """Follow the cursor while the thumb is held."""
        if not self._scroll_dragging:
            return False
        metrics = self.scrollbar_metrics()
        if not metrics:
            return False
        _tx, track_y, _tw, track_h, _thumb_y, thumb_h = metrics
        self._scroll_to_thumb_top(py - self._scroll_drag_grab,
                                  track_y, track_h, thumb_h)
        return True

    def end_scroll_drag(self):
        """Release the thumb."""
        was = self._scroll_dragging
        self._scroll_dragging = False
        self._scroll_drag_grab = 0.0
        return was

    def is_scroll_dragging(self):
        return self._scroll_dragging

    def _scroll_to_thumb_top(self, thumb_top, track_y, track_h, thumb_h):
        """Map a thumb position back to a scroll offset."""
        travel = track_h - thumb_h
        limit = self.max_scroll_px()
        if travel <= 0 or limit <= 0:
            self.scroll_px = 0.0
            return
        fraction = (thumb_top - track_y) / travel
        fraction = max(0.0, min(1.0, fraction))
        self.scroll_px = fraction * limit
        # Dragging moves the window, not the selection -- but a selection
        # scrolled out of view has to come back into it, the same rule
        # scroll() follows.
        self._clamp_selection_into_view()

    def get_selected_item(self):
        """
        Get the currently selected item.

        Returns:
            SelectableListItem or None if no selection
        """
        if 0 <= self.selected_index < len(self.filtered_items):
            return self.filtered_items[self.selected_index]
        return None

    def get_visible_items(self):
        """
        Get the items in the current scroll window.

        Returns:
            List of (item, is_selected) tuples
        """
        visible = []
        end_index = min(
            self.scroll_offset + self.max_visible_items, len(self.filtered_items)
        )

        for i in range(self.scroll_offset, end_index):
            item = self.filtered_items[i]
            is_selected = i == self.selected_index
            visible.append((item, is_selected))

        return visible

    def calculate_size(self, text_renderer):
        """
        Calculate the bounding box size for layout.

        Width is measured over ALL filtered items, not the scroll window.

        It used to measure the visible window so the panel could shrink to
        whatever names were on screen -- but that makes the width a function
        of the scroll position, so the box resizes under the cursor on every
        row that comes into view. Scrolling a list should move the list, not
        reshape the window around it. The width still tracks the FILTER,
        which is the part that is worth adapting to.

        Height is the same window the renderer and the clip rectangle use,
        and is likewise independent of the scroll position. It shrinks only
        when the whole content is shorter than the window.

        It used to measure the rows actually spanned by the window starting
        at scroll_offset. Group headers are rows too, so how many rows a
        20-item budget spends depends on how many headers fall inside it --
        slide the window past a header and the count flips between 19 and
        20. The popup is laid out from its bottom edge, so that one-row
        difference walked the top edge up and down by item_height at
        irregular points in the scroll.

        Args:
            text_renderer: TextRenderer instance for measuring text

        Returns:
            tuple: (width, height) in pixels
        """
        if not text_renderer or not text_renderer.font_atlas:
            return (300.0, 200.0)

        # Measured over every filtered item, so the width is independent of
        # where the list happens to be scrolled to.
        max_width = 200.0
        for item in self.filtered_items:
            item_width = text_renderer.calculateTextWidth(item.name, self.font_size)
            max_width = max(max_width, item_width)

        # Also measure group headers (include the "▸ " prefix)
        last_group = None
        for i in range(len(self.filtered_items)):
            item = self.filtered_items[i]
            if item.group and item.group != last_group:
                group_width = text_renderer.calculateTextWidth(
                    f"▸ {item.group}", self.font_size
                )
                max_width = max(max_width, group_width)
                last_group = item.group

        # The exact band render draws into and the scissor clips to, plus the
        # padding content_top() insets by. One expression, four consumers.
        visible_h = min(self.view_height(), self.content_height())
        total_height = visible_h + (self.padding * 2)
        total_width = max_width + self.padding * 2

        return (total_width, total_height)

    def render(
        self,
        text_renderer,
        x,
        y,
        depth,
        width,
        node_vertex_class,
        draw_fn,
        projection,
        alpha=1.0,
    ):
        """
        Render the selectable list.

        Args:
            text_renderer: TextRenderer instance
            x, y: Position in screen coordinates
            depth: Z-depth for rendering
            width: Width of the list
            node_vertex_class: NodeVertex class for creating geometry
            draw_fn: Function to draw node vertices
            projection: Projection matrix for rendering
            alpha: Overall alpha for the widget (0.0 to 1.0)
        """
        if not text_renderer or not text_renderer.font_atlas:
            return

        if not self.filtered_items:
            self.bounds = None
            return

        # Rows are inset by the list's own padding, which is the space
        # calculate_size already reserves for them: it returns
        # view_height() + padding * 2, and the popup lays the list out at
        # that size. Drawing flush at y instead spent that reservation on
        # nothing -- the top row butted straight against the search field
        # and got sliced there, while the bottom finished padding*2 short of
        # where the popup expected it.
        content_y = y + self.padding

        # Bounds are recorded BEFORE anything is drawn, because
        # scrollbar_metrics reads them -- and the bar has to be positioned
        # from the same rectangle the rows are laid out in.
        visible_h = min(self.view_height(), self.content_height())
        self.bounds = (x, content_y, width, visible_h)

        # Rows stop short of the bar rather than running under it: a selected
        # row's highlight spans the full width, and drawing the bar on top of
        # it would read as the highlight being clipped.
        content_width = width
        if self.scrollbar_metrics():
            content_width = width - (self.scrollbar_width + self.scrollbar_gap)

        # Draw whole rows from the row layout, shifted up by the sub-row
        # remainder. That shift is the entire difference between a list that
        # steps and one that glides; the caller clips the overhang.
        first_row, sub = self.first_visible_row()
        current_y = content_y - sub
        # One extra row: with any sub-row offset the last one is partly
        # visible, and stopping at max_visible_items would leave a gap the
        # height of the remainder at the bottom.
        end_row = min(len(self._rows), first_row + self.max_visible_items + 1)

        for row in range(first_row, end_row):
            kind, payload = self._rows[row]
            if kind == "header":
                self._render_group_header(
                    text_renderer,
                    payload,
                    x,
                    current_y,
                    depth,
                    content_width,
                    node_vertex_class,
                    draw_fn,
                    projection,
                    alpha,
                )
            else:
                item = self.filtered_items[payload]
                render = (self._render_selected_item
                          if payload == self.selected_index
                          else self._render_item)
                render(
                    text_renderer,
                    item.name,
                    x,
                    current_y,
                    depth,
                    content_width,
                    node_vertex_class,
                    draw_fn,
                    projection,
                    alpha,
                )
            current_y += self.item_height

        # Last, so it sits above the rows it belongs to.
        self._render_scrollbar(depth, node_vertex_class, draw_fn, projection,
                               alpha)

    def _append_quad(self, out, node_vertex_class, x, y, w, h, depth, color,
                     alpha):
        """Two triangles for one filled rect, in the convention used above."""
        r = int(color[0] * 255)
        g = int(color[1] * 255)
        b = int(color[2] * 255)
        a = int(color[3] * 255 * alpha)
        corners = ((x, y), (x + w, y), (x, y + h),
                   (x + w, y), (x + w, y + h), (x, y + h))
        for cx, cy in corners:
            out.append(node_vertex_class(
                float(cx), float(cy), depth,
                float(cx - x), float(cy - y), float(w), float(h),
                r, g, b, a, 1.0, 0, 0, 0, 255, 0.0))

    def _render_scrollbar(self, depth, node_vertex_class, draw_fn, projection,
                          alpha):
        """Draw the track and thumb, if the list overflows."""
        metrics = self.scrollbar_metrics()
        if not metrics:
            return
        track_x, track_y, track_w, track_h, thumb_y, thumb_h = metrics

        # Two draw calls, not one batch: cornerRadius is per call, and the
        # thumb is a pill while the track is square.
        track = []
        self._append_quad(track, node_vertex_class, track_x, track_y,
                          track_w, track_h, depth,
                          self.scrollbar_track_color, alpha)
        if track:
            draw_fn(track, projection, 0.0, (0, 0, 0, 0), generation=-1)

        thumb = []
        thumb_color = (self.scrollbar_thumb_active_color if self._scroll_dragging
                       else self.scrollbar_thumb_color)
        self._append_quad(thumb, node_vertex_class, track_x, thumb_y,
                          track_w, thumb_h, depth, thumb_color, alpha)
        if thumb:
            draw_fn(thumb, projection, track_w * 0.5, (0, 0, 0, 0),
                    generation=-1)

    def _render_group_header(
        self,
        text_renderer,
        group_name,
        x,
        y,
        depth,
        width,
        node_vertex_class,
        draw_fn,
        projection,
        alpha,
    ):
        """Render a group header with darker background."""
        # Draw darker background for header
        bg_vertex_data = []
        bg_r = int(self.group_header_bg_color[0] * 255)
        bg_g = int(self.group_header_bg_color[1] * 255)
        bg_b = int(self.group_header_bg_color[2] * 255)
        bg_a = int(self.group_header_bg_color[3] * 255 * alpha)

        bg_vertex_data.append(
            node_vertex_class(
                float(x),
                float(y),
                depth,
                0.0,
                0.0,
                float(width),
                float(self.group_header_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x + width),
                float(y),
                depth,
                float(width),
                0.0,
                float(width),
                float(self.group_header_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x),
                float(y + self.group_header_height),
                depth,
                0.0,
                float(self.group_header_height),
                float(width),
                float(self.group_header_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x + width),
                float(y),
                depth,
                float(width),
                0.0,
                float(width),
                float(self.group_header_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x + width),
                float(y + self.group_header_height),
                depth,
                float(width),
                float(self.group_header_height),
                float(width),
                float(self.group_header_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x),
                float(y + self.group_header_height),
                depth,
                0.0,
                float(self.group_header_height),
                float(width),
                float(self.group_header_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )

        draw_fn(bg_vertex_data, projection, 0.0, (0, 0, 0, 0), generation=-1)

        # Draw group header text centered vertically in the row
        text_x = x + self.padding
        text_y = (
            y
            + (self.group_header_height / 2)
            + (text_renderer.font_atlas.ascender * self.font_size * 0.5)
        )

        text_vertex_data = []
        text_renderer.generateTextVertices(
            f"▸ {group_name}",
            text_x,
            text_y,
            depth,
            self.font_size,
            text_vertex_data,
            node_index=-1.0,
        )
        if text_vertex_data:
            header_color = (
                self.group_header_color[0],
                self.group_header_color[1],
                self.group_header_color[2],
                alpha,
            )
            text_renderer.drawTextVertices(
                text_vertex_data, projection, header_color, disable_depth=True
            )

    def _render_item(
        self,
        text_renderer,
        item_name,
        x,
        y,
        depth,
        width,
        node_vertex_class,
        draw_fn,
        projection,
        alpha,
    ):
        """Render a normal (unselected) item."""
        text_x = x + self.padding * 2
        text_y = (
            y
            + (self.item_height / 2)
            + (text_renderer.font_atlas.ascender * self.font_size * 0.5)
        )

        text_vertex_data = []
        text_renderer.generateTextVertices(
            item_name,
            text_x,
            text_y,
            depth,
            self.font_size,
            text_vertex_data,
            node_index=-1.0,
        )
        if text_vertex_data:
            item_color = (
                self.item_color[0],
                self.item_color[1],
                self.item_color[2],
                alpha,
            )
            text_renderer.drawTextVertices(
                text_vertex_data, projection, item_color, disable_depth=True
            )

    def _render_selected_item(
        self,
        text_renderer,
        item_name,
        x,
        y,
        depth,
        width,
        node_vertex_class,
        draw_fn,
        projection,
        alpha,
    ):
        """Render a selected (highlighted) item."""
        # Draw selection background
        bg_vertex_data = []
        bg_r = int(self.selected_color[0] * 255)
        bg_g = int(self.selected_color[1] * 255)
        bg_b = int(self.selected_color[2] * 255)
        bg_a = int(self.selected_color[3] * 255 * alpha)

        bg_vertex_data.append(
            node_vertex_class(
                float(x),
                float(y),
                depth,
                0.0,
                0.0,
                float(width),
                float(self.item_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x + width),
                float(y),
                depth,
                float(width),
                0.0,
                float(width),
                float(self.item_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x),
                float(y + self.item_height),
                depth,
                0.0,
                float(self.item_height),
                float(width),
                float(self.item_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x + width),
                float(y),
                depth,
                float(width),
                0.0,
                float(width),
                float(self.item_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x + width),
                float(y + self.item_height),
                depth,
                float(width),
                float(self.item_height),
                float(width),
                float(self.item_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
        bg_vertex_data.append(
            node_vertex_class(
                float(x),
                float(y + self.item_height),
                depth,
                0.0,
                float(self.item_height),
                float(width),
                float(self.item_height),
                bg_r,
                bg_g,
                bg_b,
                bg_a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )

        draw_fn(bg_vertex_data, projection, 0.0, (0, 0, 0, 0), generation=-1)

        # Draw item text with symbol
        text_x = x + self.padding * 2
        text_y = (
            y
            + (self.item_height / 2)
            + (text_renderer.font_atlas.ascender * self.font_size * 0.5)
        )

        text_vertex_data = []
        text_renderer.generateTextVertices(
            f"► {item_name}",
            text_x,
            text_y,
            depth,
            self.font_size,
            text_vertex_data,
            node_index=-1.0,
        )
        if text_vertex_data:
            # Use white text for selected items
            selected_text_color = (1.0, 1.0, 1.0, alpha)
            text_renderer.drawTextVertices(
                text_vertex_data, projection, selected_text_color, disable_depth=True
            )

    @staticmethod
    def substring_filter(filter_text, item_name):
        """
        Default filter - match anywhere (case-insensitive).

        Args:
            filter_text: Filter string (already lowercased)
            item_name: Item name to check (already lowercased)

        Returns:
            bool: True if item matches filter
        """
        return filter_text in item_name

    @staticmethod
    def prefix_filter(filter_text, item_name):
        """
        Prefix filter - match from start only (case-insensitive).

        Args:
            filter_text: Filter string (already lowercased)
            item_name: Item name to check (already lowercased)

        Returns:
            bool: True if item matches filter
        """
        return item_name.startswith(filter_text)
