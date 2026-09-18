#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#

"""Geometry for the inline attribute value cells drawn on property rows.

Every function here takes plain numbers and returns plain numbers, so the
one invariant the whole feature rests on is unit-testable with no GL and no
Qt (``testenv/testUsdNoodlesValueCellGeometry.py``):

    the value cell and the two connection gutters are DISJOINT.

The gutters are ``GraphView._findRowEdgeDragStart``'s ``edge_band`` -- the
outer tenth of the node width on each side, for the full height of the row
band -- and they are what starts a connection drag.  Nothing in this module
is allowed to touch them: the cell stops a further ``PAD`` short of the
right gutter, and drops itself entirely rather than overlap.
"""

from collections import namedtuple


# Ratios.  Everything is expressed against the row height H so the cells
# scale with the renderer's font settings exactly as the rows do.
GUTTER_RATIO = 0.1  # MUST match graphView._findRowEdgeDragStart's edge_band
PAD_RATIO = 0.25  # dead space between the cell and the gutter
CELL_TOP_RATIO = 0.12
CELL_HEIGHT_RATIO = 0.76
SUB_GAP_RATIO = 0.10
INNER_PAD_RATIO = 0.18
SWATCH_RATIO = 0.5
# A pill, not a box: small enough to read as an inset in the row rather than
# as a widget dropped on top of it. 0.075 * H is ~8 world units at the
# shipped row height, which is 3-4 px at the zoom a graph is usually read at
# and stays proportional to the node's own nodeCornerRadius as fonts change.
CORNER_RATIO = 0.075

MIN_CELL_W_RATIO = 2.0  # below this the cell is dropped entirely
MIN_RESERVE_W_RATIO = 2.6  # the narrowest width we ask a node to reserve
MAX_RESERVE_W_RATIO = 10.0  # the widest; past this, elide instead of widen
# A vector whose sub-cells get thinner than this (after their inner padding)
# drops its cell. Measured against the row height because that is what the
# font size is derived from: at the shipped defaults a row is ~105 units and
# the value font ~31, so 0.8 * H is about five characters -- enough for a
# number, not enough to pretend three of them are readable.
MIN_VECTOR_SUB_W_RATIO = 0.8

Rect = namedtuple("Rect", "left top right bottom")


def rect_contains(rect, x, y):
    return rect.left <= x <= rect.right and rect.top <= y <= rect.bottom


def rect_width(rect):
    return rect.right - rect.left


def gutter_rects(nx, nw, row_top, h):
    """The two connection-drag bands for one row: (left, right).

    This is the zone ``_findRowEdgeDragStart`` claims.  It is reproduced
    here only so the tests can assert the cell never enters it.
    """
    g = nw * GUTTER_RATIO
    return (
        Rect(nx, row_top, nx + g, row_top + h),
        Rect(nx + nw - g, row_top, nx + nw, row_top + h),
    )


def cell_rect(nx, nw, row_top, h, cell_w, top=None, height=None):
    """The value cell for one row, or None when it cannot fit safely.

    Right-aligned against the inner edge of the right gutter, minus PAD.
    If honouring that would push the left edge into the LEFT gutter the
    cell shrinks; if the shrink takes it under ``MIN_CELL_W_RATIO * h``
    there is no cell at all, because a two-pixel number is worse than none.

    ``top`` / ``height`` let the caller place the pill on the ROW LABEL's
    own baseline (see GraphView._valueTextBand) so the value and the label
    read as one line. They default to a share of the row height, which is
    what the geometry tests measure and what a caller without font metrics
    gets.
    """
    if h <= 0.0 or cell_w <= 0.0 or nw <= 0.0:
        return None
    g = nw * GUTTER_RATIO
    pad = PAD_RATIO * h
    right = nx + nw - g - pad
    left = right - cell_w
    left_limit = nx + g + pad
    if left < left_limit:
        left = left_limit
    if right - left < MIN_CELL_W_RATIO * h:
        return None
    if top is None:
        top = row_top + CELL_TOP_RATIO * h
    if height is None:
        height = CELL_HEIGHT_RATIO * h
    return Rect(left, top, right, top + height)


def swatch_rect(rect, h):
    """The colour chip at the left of a colour cell (display only)."""
    side = SWATCH_RATIO * h
    inset = (rect.bottom - rect.top - side) * 0.5
    top = rect.top + max(0.0, inset)
    return Rect(rect.left, top, rect.left + side, top + side)


def swatch_inset(h):
    """Horizontal space a colour swatch takes off the front of a cell."""
    return SWATCH_RATIO * h + SUB_GAP_RATIO * h


def sub_rects(rect, count, h, inset_left=0.0):
    """Partition *rect* into *count* equal sub-cells separated by a gap."""
    if count <= 0:
        return []
    gap = SUB_GAP_RATIO * h
    left = rect.left + inset_left
    total = rect.right - left
    sub_w = (total - (count - 1) * gap) / float(count)
    if sub_w <= 0.0:
        return []
    out = []
    for i in range(count):
        x0 = left + i * (sub_w + gap)
        out.append(Rect(x0, rect.top, x0 + sub_w, rect.bottom))
    return out


def sub_rects_fit(rect, count, h, inset_left=0.0):
    """``sub_rects``, but None when a VECTOR's components would be unreadable.

    A scalar keeps its cell however narrow it gets -- one short number is
    still worth showing.  A vector whose sub-cells fall under
    ``MIN_VECTOR_SUB_W_RATIO * h`` is not: three illegible numbers read as
    noise, so the row goes back to looking exactly as it does today.
    """
    subs = sub_rects(rect, count, h, inset_left)
    if not subs:
        return None
    if count > 1:
        inner = INNER_PAD_RATIO * h
        if rect_width(subs[0]) - 2.0 * inner < MIN_VECTOR_SUB_W_RATIO * h:
            return None
    return subs


def text_max_width(sub, h):
    """Room for glyphs inside one sub-cell."""
    return max(0.0, rect_width(sub) - 2.0 * INNER_PAD_RATIO * h)


def measure_row_width(texts, measure, h, has_swatch=False):
    """Width a row's value cell wants, from its unelided component texts."""
    if not texts:
        return 0.0
    inner = INNER_PAD_RATIO * h
    gap = SUB_GAP_RATIO * h
    width = 2.0 * inner * len(texts)
    width += sum(float(measure(t)) for t in texts)
    width += (len(texts) - 1) * gap
    if has_swatch:
        width += swatch_inset(h)
    return width


def clamp_cell_width(measured, h):
    """The measured want, clamped to the band a node is willing to grow by."""
    return max(MIN_RESERVE_W_RATIO * h, min(float(measured), MAX_RESERVE_W_RATIO * h))


def reserve_width(cell_w, h):
    """Extra node width the cells need: the cell itself plus its padding.

    The gutter is NOT included here.  ``widened_node_width`` solves for a
    width whose own ``nw * GUTTER_RATIO`` gutter still fits beside this.
    """
    if cell_w <= 0.0:
        return 0.0
    return cell_w + PAD_RATIO * h


def widened_node_width(nw0, reserve):
    """Solve ``nw' = nw0 + reserve + GUTTER_RATIO * nw'`` for ``nw'``.

    Widening by ``reserve`` alone would not do: the gutter is a FRACTION of
    the width, so it grows with the node and would eat back into the space
    just reserved.  This is the closed form that leaves a full gutter, the
    padding and the cell all intact beside the original label width.
    """
    if reserve <= 0.0:
        return nw0
    return (nw0 + reserve) / (1.0 - GUTTER_RATIO)


def node_is_wide_enough(nw, port_width):
    """Whether a node is wide enough to carry a cell without crowding pins."""
    return nw * GUTTER_RATIO >= port_width * 1.5


def corner_radius(h):
    return CORNER_RATIO * h


# ---------------------------------------------------------------------------
# Theme
#
# Every colour a value cell draws is DERIVED from the node theme the rest of
# the editor already reads (nodeBgHigh / nodeBgLow / nodeBgAlpha, the port
# colours, the selection stroke). Nothing here is a fixed grey: change the
# node background and the pills follow it, which is what makes them read as
# part of the node rather than as a widget pasted over it.
# ---------------------------------------------------------------------------

# Axis tints for vector components. A convention, not a theme value -- x/y/z
# are red/green/blue everywhere in this industry -- but muted hard, because
# they are a 3-character hint beside a number, not a gizmo.
AXIS_COLORS = (
    (0.84, 0.42, 0.44),
    (0.50, 0.78, 0.44),
    (0.44, 0.62, 0.90),
    (0.74, 0.72, 0.78),
)
AXIS_LABELS = ("x", "y", "z", "w")
COLOR_AXIS_LABELS = ("r", "g", "b", "a")

Theme = namedtuple(
    "Theme",
    "body fillAuthored fillHover fillAccent fillEdit fillError hairline "
    "textNormal textDim textConnected textAnimated accent error "
    "boolOn boolOff caret selection popupBg popupRow",
)


def _mix(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(len(a)))


def _scaled(value, factor, alpha):
    v = max(0.0, min(1.0, value * factor))
    return (v, v, v * 1.04, alpha)


def build_theme(node_bg_high, node_bg_low, accent, port_on, port_off, connected,
                animated, authored, fallback):
    """Derive every value-cell colour from the node theme.

    ``node_bg_high`` / ``node_bg_low`` are the 0-255 node-body values the C++
    renderer gradients between; their mean IS the node body, so the pill is
    expressed as a percentage of it rather than as a colour of its own.
    """
    body = (float(node_bg_high) + float(node_bg_low)) * 0.5 / 255.0
    accent = tuple(accent)
    return Theme(
        body=body,
        # ~8% darker than the row once composited: an inset, not a slab.
        fillAuthored=_scaled(body, 0.60, 0.22),
        # ~9% lighter: the row lifts under the cursor.
        fillHover=_scaled(body, 1.90, 0.20),
        # A TINT, not a coat of paint: the accent is a saturated yellow and
        # anything above a whisper of it buries the number underneath.
        fillAccent=(accent[0], accent[1], accent[2], 0.11),
        # The editor is a well sunk into the row, the way a text field looks
        # anywhere else; the accent is its OUTLINE, not its fill.
        fillEdit=_scaled(body, 0.34, 0.62),
        fillError=(0.95, 0.35, 0.30, 0.14),
        hairline=_scaled(body, 0.42, 0.85),
        textNormal=tuple(authored),
        textDim=tuple(fallback),
        textConnected=tuple(connected),
        textAnimated=tuple(animated),
        accent=(accent[0], accent[1], accent[2], 1.0),
        error=(0.95, 0.35, 0.30, 1.0),
        boolOn=tuple(port_on),
        boolOff=tuple(port_off),
        caret=_mix(accent[:3], (1.0, 1.0, 1.0), 0.35) + (1.0,),
        selection=(accent[0], accent[1], accent[2], 0.30),
        popupBg=_scaled(body, 0.72, 0.97),
        popupRow=(accent[0], accent[1], accent[2], 0.18),
    )


def append_quad(out, node_vertex_class, rect, depth, color, alpha=1.0):
    """Two triangles for one filled rect, in the NodeVertex convention.

    NodeVertex is (x, y, z, u, v, w, h, r, g, b, a, innerStroke,
    sr, sg, sb, sa, selected) -- see widgets/textInputWidget.py:198.
    """
    x = float(rect.left)
    y = float(rect.top)
    w = float(rect.right - rect.left)
    h = float(rect.bottom - rect.top)
    r = int(max(0.0, min(1.0, color[0])) * 255)
    g = int(max(0.0, min(1.0, color[1])) * 255)
    b = int(max(0.0, min(1.0, color[2])) * 255)
    a = int(max(0.0, min(1.0, color[3] * alpha)) * 255)
    corners = (
        (x, y),
        (x + w, y),
        (x, y + h),
        (x + w, y),
        (x + w, y + h),
        (x, y + h),
    )
    for cx, cy in corners:
        out.append(
            node_vertex_class(
                float(cx),
                float(cy),
                depth,
                float(cx - x),
                float(cy - y),
                w,
                h,
                r,
                g,
                b,
                a,
                1.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )
