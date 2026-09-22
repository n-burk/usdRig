#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Layered auto-layout for the node graph, lined up on the pins.

Two entry points, both pure (plain dicts and tuples in and out -- no Qt,
no GL, no USD), deterministic (no jitter, stable tie-breaks) and linear
in the graph:

* :func:`layout_graph` arranges a whole graph: ranks left to right,
  pins level, over-tall ranks wrap into sub-columns, cycles share a
  column, and unwired nodes park in a grid underneath.
* :func:`place_node` positions ONE new node beside the neighbors it wires
  to, for the add-node path where everything else must stay put.

Every edge ranks, attribute or relationship: the view passes each
rendered noodle straight through, because any connection the user reads
is a connection the layout should respect. Only dangling endpoints (a
node off the canvas) and self-loops are dropped, since there is nothing
to rank them against.

A column is a depth: rank is the longest path from a source, so data
flows left to right and same-rank nodes share a column. Sources are
pulled right beside their shallowest consumer rather than stranded in
column zero. Within a column, what lines up is the PINS, not the boxes:
a lone edge levels its two pins exactly, and a fan of dependents stacks
in the column beside its stem, centred so the median member pin lands on
the stem's pin. A rank taller than the whole flow is wide wraps into
sub-columns -- and so does any rank past MAX_COLUMN_NODES members, which
is the only cap that can bound never-laid-out rows of height zero.

Cycles are best effort by construction: strongly connected components are
condensed first, so no feedback edge can loop the ranking or stretch the
columns. A cycle is one block, its members stacked unbroken in one
column in member order.
"""

from __future__ import annotations

import heapq
from statistics import median


COLUMN_GAP = 150.0
ROW_GAP = 60.0

# A rank past this many members wraps into sub-columns however short
# its rows are: never-laid-out rows report height zero, so the count cap
# is the only one that can bound them.
MAX_COLUMN_NODES = 12

# Column gap as a fraction of the steepest link crossing it, clamped into
# ``[COLUMN_GAP, _MAX_GAP_FACTOR * COLUMN_GAP]``. Only links between
# NEIGHBORING columns count: a link that reaches across several columns
# already has all the horizontal run its curve could want, while a fan
# from one pin down to the next column over is the case that renders as a
# vertical stripe unless the gap opens up.
_GAP_SLOPE = 0.35
_MAX_GAP_FACTOR = 6.0


def layout_graph(sizes, edges):
    """``{node_id: (x, y)}`` for every id in *sizes*.

    *sizes* maps each id to its ``(width, height)``. *edges* is an
    iterable of ``(source_id, target_id)`` data-flow pairs, or of
    ``(source_id, target_id, source_dy, target_dy)`` where the offsets
    locate each end of the noodle relative to its own node's top edge.
    A 2-tuple defaults both offsets to the node's half height, so the
    link aims at the centres exactly as a centred box layout would.

    Edges with an endpoint missing from *sizes* and self-loops are
    ignored. Nodes left with no edge at all lay out as a grid below
    everything that is wired.
    """
    ids = list(sizes)
    if not ids:
        return {}
    index_of = {node_id: i for i, node_id in enumerate(ids)}
    succ, pred, incident, pin_dy = _adjacency(ids, edges, index_of, sizes)

    wired = [node_id for node_id in ids if incident[node_id]]
    # Scopes and other unwired prims: they have no place in the flow,
    # so the flow should not have to make room for them.
    singletons = [node_id for node_id in ids if not incident[node_id]]

    positions = {}
    if wired:
        positions = _layout_wired(
            wired, sizes, succ, pred, incident, pin_dy, index_of
        )
    _place_singletons(positions, singletons, sizes)
    return positions


def _place_singletons(positions, singletons, sizes):
    """Shelf-pack the unwired nodes into a grid below *positions*.

    A rig is mostly scopes: unwired prims outnumber the operators and,
    threaded into the columns, they wedge between the ranks and push the
    flow apart. Parked underneath in reading order they stay findable and
    cost the flow no room at all. The grid is as wide as the wired
    block (or square, whichever is wider), so it reads as a footer rather
    than as a second graph. A graph with nothing wired is just the grid.
    """
    if not singletons:
        return positions
    if positions:
        left = min(x for x, _y in positions.values())
        right = max(x + sizes[n][0] for n, (x, _y) in positions.items())
        bottom = max(y + sizes[n][1] for n, (_x, y) in positions.items())
        width = right - left
        top = bottom + 2.0 * ROW_GAP
    else:
        left, width, top = 0.0, 0.0, 0.0

    area = sum(sizes[n][0] * sizes[n][1] for n in singletons)
    row_target = max(width, area**0.5)
    rows = [[]]
    row_widths = [0.0]
    for node_id in singletons:
        node_width = sizes[node_id][0]
        if rows[-1] and row_widths[-1] + COLUMN_GAP + node_width > row_target:
            rows.append([])
            row_widths.append(0.0)
        rows[-1].append(node_id)
        row_widths[-1] += (COLUMN_GAP if row_widths[-1] else 0.0) + node_width

    y_cursor = top
    for row in rows:
        x_cursor = left
        row_height = 0.0
        for node_id in row:
            positions[node_id] = (x_cursor, y_cursor)
            x_cursor += sizes[node_id][0] + COLUMN_GAP
            row_height = max(row_height, sizes[node_id][1])
        y_cursor += row_height + ROW_GAP
    return positions


def place_node(size, predecessors, successors, occupants, fallback):
    """``(x, y)`` for one new node of *size* ``(w, h)``.

    *predecessors* / *successors* are ``(x, y, w, h)`` rects of the new
    node's already-placed input / output neighbors; *occupants* holds the
    same for every placed node (for de-overlap); *fallback* is the
    ``(x, y)`` used when the node wires to nothing on the canvas.

    A node with inputs lands one column past its rightmost input, at the
    median height of its inputs; with only outputs, one column before its
    leftmost output. The choice then slides down to the first vertical
    gap that fits, so the new node never lands on another.
    """
    width, height = size
    if predecessors:
        x = max(rx + rw for rx, _ry, rw, _rh in predecessors) + COLUMN_GAP
        preferred_y = median(ry + rh * 0.5 for _rx, ry, _rw, rh in predecessors)
        preferred_y -= height * 0.5
    elif successors:
        x = min(rx for rx, _ry, _rw, _rh in successors) - COLUMN_GAP - width
        preferred_y = median(ry + rh * 0.5 for _rx, ry, _rw, rh in successors)
        preferred_y -= height * 0.5
    else:
        x, preferred_y = fallback
    return (x, _first_free_y(x, width, preferred_y, height, occupants))


def _adjacency(ids, edges, index_of, sizes):
    """``(successors, predecessors, incident, pin_dy)`` over *ids*.

    The first two are deduplicated per node pair. *incident* keeps every
    edge, both ways round, as ``(other, other_dy, own_dy)``, which is
    what the column gaps measure their steepness from. *pin_dy* maps each
    ``(source, target)`` pair to the SMALLEST source offset that joins
    them, so a target wired to two pins of one source anchors to the
    upper pin.

    Self-loops and edges naming ids outside *sizes* are dropped: a loop
    is not a depth constraint, and a dangling endpoint names a node that
    is not on the canvas.
    """
    succ = {node_id: set() for node_id in ids}
    pred = {node_id: set() for node_id in ids}
    incident = {node_id: [] for node_id in ids}
    pin_dy = {}
    for edge in edges:
        src, dst = edge[0], edge[1]
        if src == dst or src not in pred or dst not in pred:
            continue
        if len(edge) >= 4:
            src_dy, dst_dy = float(edge[2]), float(edge[3])
        else:
            src_dy = sizes[src][1] * 0.5
            dst_dy = sizes[dst][1] * 0.5
        incident[src].append((dst, dst_dy, src_dy))
        incident[dst].append((src, src_dy, dst_dy))
        pair = (src, dst)
        if pair not in pin_dy or src_dy < pin_dy[pair]:
            pin_dy[pair] = src_dy
        succ[src].add(dst)
        pred[dst].add(src)
    # Sorted neighbor lists keep every downstream traversal deterministic.
    for node_id in ids:
        succ[node_id] = sorted(succ[node_id], key=index_of.__getitem__)
        pred[node_id] = sorted(pred[node_id], key=index_of.__getitem__)
    return succ, pred, incident, pin_dy


def _strongly_connected(component, succ):
    """``(member_to_group, groups)`` via iterative Tarjan.

    Groups arrive in discovery (reverse-topological) order over the
    deterministically ordered neighbor lists; members keep insertion
    order within their group.
    """
    index_of = {}
    low = {}
    on_stack = set()
    stack = []
    member_to_group = {}
    groups = []
    counter = 0
    for root in component:
        if root in index_of:
            continue
        work = [(root, iter(succ[root]))]
        while work:
            node, it = work[-1]
            if node not in index_of:
                index_of[node] = low[node] = counter
                counter += 1
                stack.append(node)
                on_stack.add(node)
            descended = False
            for nxt in it:
                if nxt not in index_of:
                    work.append((nxt, iter(succ[nxt])))
                    descended = True
                    break
                elif nxt in on_stack:
                    if index_of[nxt] < low[node]:
                        low[node] = index_of[nxt]
            if descended:
                continue
            work.pop()
            if work and low[node] < low[work[-1][0]]:
                low[work[-1][0]] = low[node]
            if low[node] == index_of[node]:
                members = []
                while True:
                    member = stack.pop()
                    on_stack.discard(member)
                    members.append(member)
                    member_to_group[member] = len(groups)
                    if member == node:
                        break
                members.reverse()
                groups.append(members)
    return member_to_group, groups


def _rank_groups(groups, gsucc, gpred, min_index):
    """Longest-path rank of every group, sources pulled beside use.

    Rank is depth: a group with no predecessors sits in rank zero and
    every other group sits one past its deepest predecessor, so data
    flows left to right and same-rank groups share a column. Sources
    are then pulled right to one rank left of their shallowest
    consumer: longest path strands a constant used once, deep in the
    graph, the whole width of the canvas from its only consumer. Only
    sourceless groups move, and only right of rank zero, so no edge
    ever points backwards.
    """
    count = len(groups)
    indegree = [len(gpred[group]) for group in range(count)]
    ready = [(min_index[group], group)
             for group in range(count) if not indegree[group]]
    heapq.heapify(ready)
    ranks = [0] * count
    while ready:
        _, group = heapq.heappop(ready)
        for target in sorted(gsucc[group], key=min_index.__getitem__):
            if ranks[target] < ranks[group] + 1:
                ranks[target] = ranks[group] + 1
            indegree[target] -= 1
            if not indegree[target]:
                heapq.heappush(ready, (min_index[target], target))
    for group in range(count):
        if gpred[group] or not gsucc[group]:
            continue
        shallowest = min(ranks[target] for target in gsucc[group])
        if shallowest > 0:
            ranks[group] = shallowest - 1
    return ranks


def _wrap_columns(groups, ranks, widths, heights, min_index, gpred):
    """``(group_column, columns, key_feeder)``: ranks split into columns.

    Members of a rank order by feeder -- groups fed from outside order
    by their earliest feeder, then input order, so each fan stays one
    contiguous run -- with unfed groups last in input order. A rank
    taller than the whole flow is wide wraps into sub-columns, greedily
    filled in that order, and so does any rank past MAX_COLUMN_NODES
    members: never-laid-out rows report height zero, so no height cap
    can bound them and only the count cap does. Sub-columns keep rank
    order, so everything stays left of what it feeds. *columns* lists
    the ordered member groups of each final column, left to right.
    """
    count = len(groups)
    key_feeder = [
        min((min_index[pred] for pred in gpred[group]), default=None)
        for group in range(count)
    ]
    by_rank = {}
    for group in range(count):
        by_rank.setdefault(ranks[group], []).append(group)
    ordered_ranks = sorted(by_rank)
    flow_width = sum(
        max(widths[group] for group in by_rank[rank])
        for rank in ordered_ranks
    ) + COLUMN_GAP * (len(ordered_ranks) - 1)
    group_column = [0] * count
    columns = []
    for rank in ordered_ranks:
        members = sorted(
            by_rank[rank],
            key=lambda g: (key_feeder[g] is None, key_feeder[g] or 0,
                           min_index[g]),
        )
        chunk = []
        chunk_height = 0.0
        for group in members:
            if chunk and (len(chunk) + 1 > MAX_COLUMN_NODES
                          or chunk_height + heights[group] > flow_width):
                for member in chunk:
                    group_column[member] = len(columns)
                columns.append(chunk)
                chunk = []
                chunk_height = 0.0
            chunk.append(group)
            chunk_height += heights[group]
        for member in chunk:
            group_column[member] = len(columns)
        columns.append(chunk)
    return group_column, columns, key_feeder


def _place_columns(groups, group_of, columns, key_feeder, sizes, pred,
                   incident, pin_dy, index_of, min_index):
    """Top y of every group: pins level, fans centre, nothing overlaps.

    Columns place left to right, so every outside feeder of a group is
    already placed: each member anchors through its outside in-edges at
    the median of (feeder pin - own pin), and a lone edge levels its
    two pins exactly. Members sharing a feeder stack as one unit,
    centred so the median member pin lands on the unit anchor -- an
    odd fan's middle pin is level with its stem's -- and a cycle's
    members stay one unbroken block in member order. Units stack top
    to bottom and slide down past whatever is already there, so
    placement never overlaps; exactness yields to that slide only when
    the column is crowded.
    """
    count = len(groups)
    offsets = []
    for members in groups:
        cursor = 0.0
        table = {}
        for member in members:
            table[member] = cursor
            cursor += sizes[member][1] + ROW_GAP
        offsets.append(table)
    tops = [0.0] * count
    for column in columns:
        cursor = None
        unit = []
        for group in column + [None]:
            if group is not None and (
                    not unit or key_feeder[group] == key_feeder[unit[0]]):
                unit.append(group)
                continue
            tops, cursor = _place_unit(
                unit, groups, group_of, offsets, sizes, pred, incident,
                pin_dy, index_of, min_index, tops, cursor)
            unit = [] if group is None else [group]
    return tops


def _place_unit(unit, groups, group_of, offsets, sizes, pred, incident,
                pin_dy, index_of, min_index, tops, cursor):
    """Place one feeder-sharing run of groups; return ``(tops, cursor)``."""
    ordered = []
    for group in sorted(unit, key=min_index.__getitem__):
        ordered.extend(groups[group])
    anchored = []
    for member in ordered:
        implied = []
        pins = []
        for feeder in sorted(pred[member], key=index_of.__getitem__):
            if group_of[feeder] == group_of[member]:
                continue
            feeder_group = group_of[feeder]
            own_dy = min(entry[2] for entry in incident[member]
                         if entry[0] == feeder)
            pins.append(own_dy)
            implied.append(
                tops[feeder_group] + offsets[feeder_group][feeder]
                + pin_dy[(feeder, member)] - own_dy)
        if implied:
            anchored.append((member, median(implied), median(pins)))
    rel = {}
    cursor_rel = 0.0
    for member in ordered:
        rel[member] = cursor_rel
        cursor_rel += sizes[member][1] + ROW_GAP
    if anchored:
        shift = (median(anchor for _, anchor, _ in anchored)
                 + median(pin for _, _, pin in anchored)
                 - median(rel[member] + pin
                          for member, _, pin in anchored))
        top = shift if cursor is None else max(shift, cursor)
    else:
        top = 0.0 if cursor is None else cursor
    seen = set()
    for member in ordered:
        group = group_of[member]
        if group not in seen:
            seen.add(group)
            tops[group] = top + rel[member] - offsets[group][member]
    span = cursor_rel - ROW_GAP if ordered else 0.0
    return tops, top + span + ROW_GAP


def _layout_wired(wired, sizes, succ, pred, incident, pin_dy, index_of):
    """``{node_id: (x, y)}`` for every wired node."""
    group_of, groups = _strongly_connected(wired, succ)
    # A cycle is one block: its members share a column and stack inside
    # it, so the block is as wide as its widest member and as tall as
    # all of them.
    widths = [max(sizes[m][0] for m in members) for members in groups]
    heights = [
        sum(sizes[m][1] for m in members) + ROW_GAP * (len(members) - 1)
        for members in groups
    ]
    min_index = [min(index_of[m] for m in members) for members in groups]
    gsucc = [set() for _ in groups]
    gpred = [set() for _ in groups]
    for member in wired:
        for nxt in succ[member]:
            target = group_of[nxt]
            if target != group_of[member]:
                gsucc[group_of[member]].add(target)
                gpred[target].add(group_of[member])
    ranks = _rank_groups(groups, gsucc, gpred, min_index)
    group_column, columns, key_feeder = _wrap_columns(
        groups, ranks, widths, heights, min_index, gpred)
    tops = _place_columns(
        groups, group_of, columns, key_feeder, sizes, pred, incident,
        pin_dy, index_of, min_index)
    xs = _column_positions(
        group_column, widths, groups, group_of, incident, tops, sizes
    )
    positions = {}
    for group, members in enumerate(groups):
        cursor = tops[group]
        for member in members:
            positions[member] = (xs[group_column[group]], cursor)
            cursor += sizes[member][1] + ROW_GAP
    top = min(y for _x, y in positions.values())
    return {n: (x, y - top) for n, (x, y) in positions.items()}


def _column_positions(columns, widths, groups, group_of, incident, tops, sizes):
    """x of every column, the gap widening for the steepest link.

    A bezier is drawn with horizontal tangents, so its rise has to fit
    inside its run: at the 150px minimum a link that drops a thousand
    pixels renders as a vertical stripe through whatever is behind it. A
    fan of dependents hangs off one pin and drops away from it, so this
    is the common case, not the exception.
    """
    count = max(columns) + 1 if columns else 0
    column_width = [0.0] * count
    for group, width in enumerate(widths):
        if width > column_width[columns[group]]:
            column_width[columns[group]] = width

    member_top = {}
    for group, members in enumerate(groups):
        cursor = tops[group]
        for member in members:
            member_top[member] = cursor
            cursor += sizes[member][1] + ROW_GAP

    spans = [0.0] * max(count - 1, 0)
    for node_id in member_top:
        here = columns[group_of[node_id]]
        for other, other_dy, own_dy in incident[node_id]:
            there = columns[group_of[other]]
            # Only the step to the very next column: a link reaching
            # further already has all the run its curve could want, and
            # charging every gap it crosses would inflate the whole page.
            if there != here + 1:
                continue
            rise = abs(
                (member_top[node_id] + own_dy) - (member_top[other] + other_dy)
            )
            if rise > spans[here]:
                spans[here] = rise

    xs = []
    cursor = 0.0
    for column in range(count):
        xs.append(cursor)
        cursor += column_width[column]
        if column < count - 1:
            gap = _GAP_SLOPE * spans[column]
            cursor += min(max(gap, COLUMN_GAP), _MAX_GAP_FACTOR * COLUMN_GAP)
    return xs


def _first_free_y(x, width, preferred_y, height, occupants):
    """First ``y >= preferred_y`` whose rect clears *occupants*.

    Only occupants overlapping the ``[x, x + width]`` band constrain the
    answer, each padded by ``ROW_GAP`` so neighbors never touch.
    """
    spans = []
    for ox, oy, ow, oh in occupants:
        if ox + ow <= x or ox >= x + width:
            continue
        spans.append((oy - ROW_GAP, oy + oh + ROW_GAP))
    spans.sort()
    cursor = preferred_y
    for start, end in spans:
        if end <= cursor:
            continue
        if start >= cursor + height:
            break
        cursor = end
    return cursor
