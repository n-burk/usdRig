#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Layered, pin-aligned auto-layout for the node graph.

Two entry points, both pure (plain dicts and tuples in and out -- no Qt,
no GL, no USD), deterministic (no jitter, stable tie-breaks) and linear
in the graph plus the skyline merges:

* :func:`layout_graph` arranges a whole graph: every node hangs off the
  node that feeds it, grouped by WHICH ROW of that node feeds it and
  levelled pin to pin, cycles share a column, and unwired nodes park in
  a grid underneath.
* :func:`place_node` positions ONE new node beside the neighbors it wires
  to, for the add-node path where everything else must stay put.

Every edge ranks, attribute or relationship: the view passes each
rendered noodle straight through, because any connection the user reads
is a connection the layout should respect. Only dangling endpoints (a
node off the canvas) are dropped, by the caller, since there is nothing
to rank them against.

A column is a depth, and what lines up inside it is PINS. A node is a
list of rows, the rows are the meaning, and the noodle leaves a row --
not the middle of a box. Centering the boxes of a 1000px node and a
3000px one leaves the link sweeping the height of the target for no
reason anybody reading the graph can see. So:

* everything a node feeds lands in the next column and STACKS there,
  which keeps a fan-out together where the eye can count it;
* dependents fed by DIFFERENT rows of a node stack in the order those
  rows appear in it, so scanning down the parent is scanning down the
  picture, and each parent's fan stays one unbroken run;
* a node sits where its own pin LEVELS with the pins it feeds -- exactly,
  for a single link; at the median, for a fan, so one very tall dependent
  cannot drag the stem off the rest;
* a node that also depends on something deeper is pushed right past it,
  since a column is a depth and depth is what puts a node after its
  inputs;
* a column too tall to read as one list wraps into sub-columns, placed
  left of whatever the pile feeds (see `_wrap_tall_columns`).

Two things follow. Each node takes exactly ONE parent, the first one that
feeds it, because it needs a single home and that home is the row it was
introduced in; every other in-edge is drawn but places nothing beyond
pushing the node further right. And blocks are kept apart by a per-column
SKYLINE rather than a bounding box, so one tall sink at the end of a
column does not shove the unrelated block below it down by its whole
height.

Cycles are best effort by construction: strongly connected components are
condensed first, so no feedback edge can loop the recursion or stretch
the columns. A cycle is one entry of the outline, its members stacked in
one column.
"""

from __future__ import annotations

import heapq
from statistics import median


COLUMN_GAP = 150.0
ROW_GAP = 60.0

# Most nodes a column may hold before it wraps into sub-columns. The
# height cap in `_wrap_tall_columns` is the one that usually bites; this
# is what answers a pile of rows that have never been laid out, which
# report height zero and so cannot be bounded by any height at all.
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
    ``source_dy`` is what groups a node's dependents into rows; a 2-tuple
    defaults both offsets to the node's half height, which puts every
    dependent of that node in one row.

    Edges with an endpoint missing from *sizes* and self-loops are
    ignored. Nodes left with no edge at all lay out as a grid below
    everything that is wired.
    """
    ids = list(sizes)
    if not ids:
        return {}
    index_of = {node_id: i for i, node_id in enumerate(ids)}
    succ, pred, incident, pin_dy, pin_pair = _adjacency(
        ids, edges, index_of, sizes
    )

    wired = [node_id for node_id in ids if incident[node_id]]
    # Scopes and other unwired prims: they have no place in the outline,
    # so the outline should not have to make room for them.
    singletons = [node_id for node_id in ids if not incident[node_id]]

    positions = {}
    if wired:
        positions = _layout_outline(
            wired, sizes, succ, incident, pin_dy, pin_pair, index_of
        )
    _place_singletons(positions, singletons, sizes)
    return positions


def _place_singletons(positions, singletons, sizes):
    """Shelf-pack the unwired nodes into a grid below *positions*.

    A rig is mostly scopes: unwired prims outnumber the operators and,
    threaded into the outline, they wedge between the trees and push the
    flow apart. Parked underneath in reading order they stay findable and
    cost the outline no room at all. The grid is as wide as the wired
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
    them, so a target wired to two rows of one source hangs off the upper
    row and appears in the outline once.

    Self-loops and edges naming ids outside *sizes* are dropped: a loop
    is not a depth constraint, and a dangling endpoint names a node that
    is not on the canvas.
    """
    succ = {node_id: set() for node_id in ids}
    pred = {node_id: set() for node_id in ids}
    incident = {node_id: [] for node_id in ids}
    pin_dy = {}
    pin_pair = {}
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
            # Both ends of the edge the outline hangs the target from, so
            # the placement can level the two pins rather than the boxes.
            pin_pair[pair] = (src_dy, dst_dy)
        succ[src].add(dst)
        pred[dst].add(src)
    # Sorted neighbor lists keep every downstream traversal deterministic.
    for node_id in ids:
        succ[node_id] = sorted(succ[node_id], key=index_of.__getitem__)
        pred[node_id] = sorted(pred[node_id], key=index_of.__getitem__)
    return succ, pred, incident, pin_dy, pin_pair


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


def _row_groups(group, members, succ, group_of, pin_dy, pin_pair, index_of,
                min_index, sizes, member_dy):
    """``(rows, pins)`` -- the group's dependents, bucketed by feeding row.

    *rows* is a list of lists: one list per source row, rows top to
    bottom, dependents within a row in input order. A dependent reached
    from several rows belongs to the topmost one only, so it appears in
    the outline exactly once. A cycle's rows are read member by member,
    in member order, which keeps a condensed group's outline as stable as
    a plain node's.

    *pins* maps ``(group, target)`` to the ``(source_dy, target_dy)`` of
    the edge the target is filed under, both measured from their own
    GROUP's top edge so a condensed cycle's member offset is already in
    them. That pair is what levels the two pins at placement time.
    """
    best = {}
    pins = {}
    for position, member in enumerate(members):
        for nxt in succ[member]:
            target = group_of[nxt]
            if target == group:
                continue
            key = (position, pin_dy[(member, nxt)])
            if target not in best or key < best[target]:
                best[target] = key
                src_dy, dst_dy = pin_pair[(member, nxt)]
                pins[(group, target)] = (
                    member_dy[member] + src_dy,
                    member_dy[nxt] + dst_dy,
                )
    buckets = {}
    for target, key in best.items():
        buckets.setdefault(key, []).append(target)
    rows = []
    for key in sorted(buckets):
        rows.append(sorted(buckets[key], key=min_index.__getitem__))
    return rows, pins


def _columns(rows_of, predecessors_of, groups, index_of):
    """Depth of every group, in columns, by longest path.

    One column per depth: everything a node feeds lands in the next
    column, so a fan-out STACKS there rather than running away to the
    right. Every in-edge pushes its target right, which is how a node
    that also depends on something deeper lands past it.

    Sources are then pulled right, to the tightest column their own
    dependents allow. Longest path leaves every source in column zero,
    which strands a constant used once, deep in the graph, the whole
    width of the canvas from its only consumer. Only sources move, and
    only right, so no node can overtake a predecessor -- it has none.
    """
    count = len(groups)
    min_index = [min(index_of[m] for m in members) for members in groups]
    indegree = [len(predecessors_of[g]) for g in range(count)]
    ready = [(min_index[g], g) for g in range(count) if not indegree[g]]
    heapq.heapify(ready)
    columns = [0] * count
    while ready:
        _, group = heapq.heappop(ready)
        for row in rows_of[group]:
            for target in row:
                if columns[target] < columns[group] + 1:
                    columns[target] = columns[group] + 1
                indegree[target] -= 1
                if not indegree[target]:
                    heapq.heappush(ready, (min_index[target], target))
    for group in range(count):
        if predecessors_of[group]:
            continue
        tightest = None
        for row in rows_of[group]:
            for target in row:
                slot = columns[target] - 1
                if tightest is None or slot < tightest:
                    tightest = slot
        if tightest is not None:
            columns[group] = tightest
    base = min(columns)
    return [column - base for column in columns], min_index


def _wrap_tall_columns(columns, widths, heights, min_index):
    """*columns* again, with any tower split into sub-columns.

    A column is a column while the eye can run down it. Six two-thousand
    pixel rows feeding one node are twelve thousand pixels of pile
    against a flow four thousand wide: that is a tower, and no amount of
    scrolling makes it read as one list. Such a column is cut into
    contiguous chunks that each sit in a column of their own, immediately
    left of whatever the pile feeds, and everything downstream shifts
    right to make the room.

    Two caps, and the SMALLER wins. The height cap is how many nodes fit
    in the width the flow already has -- measured on the nodes alone,
    because the gaps between them are what is being re-spent. The count
    cap is ``MAX_COLUMN_NODES``, and it is the one that answers a pile of
    never-laid-out rows: those report height zero, so no height can bound
    them and only a count can.

    Chunks keep the column's own order, so a wrapped pile reads left to
    right exactly as it read top to bottom.
    """
    if not columns:
        return columns
    count = max(columns) + 1
    members_of = [[] for _ in range(count)]
    for group in sorted(range(len(columns)), key=min_index.__getitem__):
        members_of[columns[group]].append(group)

    column_width = [0.0] * count
    for group, width in enumerate(widths):
        if width > column_width[columns[group]]:
            column_width[columns[group]] = width
    flow_width = sum(column_width) + COLUMN_GAP * max(count - 1, 0)

    chunks_of = []
    for column in range(count):
        members = members_of[column]
        pile = sum(heights[g] for g in members) + ROW_GAP * max(
            len(members) - 1, 0
        )
        if len(members) < 2 or pile <= flow_width:
            chunks_of.append([members])
            continue
        # How many fit across the width the flow already spends.
        cap = 0
        used = 0.0
        for group in members:
            used += heights[group]
            if used > flow_width and cap:
                break
            cap += 1
        cap = min(cap or 1, MAX_COLUMN_NODES)
        chunks_of.append(
            [members[i:i + cap] for i in range(0, len(members), cap)]
        )

    shifted = list(columns)
    offset = 0
    for column in range(count):
        for index, chunk in enumerate(chunks_of[column]):
            for group in chunk:
                shifted[group] = column + offset + index
        offset += len(chunks_of[column]) - 1
    return shifted


def _tree_parents(predecessors_of, columns, min_index):
    """One home per node: the first thing that feeds it.

    An outline entry sits under one heading, and of everything that feeds
    a node the earliest -- leftmost -- is the one whose row it belongs
    to. Every other in-edge stays drawn but places nothing: it has
    already had its say, by pushing this node further right.

    Filing a node under its NEAREST feeder instead would read as well on
    paper and keeps the tree links shorter, but it moves a node out of
    the row that introduced it: a joint chain's last link would adopt the
    node the source also drives, lifting it out of the source's bottom
    row and into the joint row. Ties go to the earliest input, so the
    choice never depends on set order.
    """
    parents = []
    for group, feeders in enumerate(predecessors_of):
        best = None
        for feeder in feeders:
            key = (columns[feeder], min_index[feeder])
            if best is None or key < best[0]:
                best = (key, feeder)
        parents.append(best[1] if best else None)
    return parents


def _clearance(base, incoming):
    """Smallest downward shift that drops *incoming* clear of *base*.

    Both are skylines: ``{column: (top, bottom)}``. Only shared columns
    can collide, which is the whole point of keeping a skyline instead of
    a bounding box -- a block only pays for the columns it actually
    occupies, so a tall sink at the end of one row does not push the next
    row down past its own height.
    """
    if not base or not incoming:
        return 0.0
    shift = 0.0
    if len(incoming) <= len(base):
        for column, span in incoming.items():
            other = base.get(column)
            if other is not None and other[1] + ROW_GAP - span[0] > shift:
                shift = other[1] + ROW_GAP - span[0]
    else:
        for column, span in base.items():
            other = incoming.get(column)
            if other is not None and span[1] + ROW_GAP - other[0] > shift:
                shift = span[1] + ROW_GAP - other[0]
    return shift


def _merge(base, incoming, shift):
    """*base* widened to cover *incoming* moved down by *shift*.

    An empty base with nothing to shift adopts the incoming skyline whole
    rather than copying it, which keeps a long chain of only children
    linear instead of quadratic. Every skyline is consumed exactly once,
    by its parent, so there is nobody left to notice.
    """
    if not base and not shift:
        return incoming
    for column, (top, bottom) in incoming.items():
        top += shift
        bottom += shift
        current = base.get(column)
        if current is None:
            base[column] = (top, bottom)
        else:
            base[column] = (
                top if top < current[0] else current[0],
                bottom if bottom > current[1] else current[1],
            )
    return base


def _layout_outline(wired, sizes, succ, incident, pin_dy, pin_pair,
                    index_of):
    """``{node_id: (x, y)}`` for every wired node."""
    group_of, groups = _strongly_connected(wired, succ)
    # A cycle is one outline entry: its members share a column and stack
    # inside it, so the block is as wide as its widest member and as tall
    # as all of them.
    widths = [max(sizes[m][0] for m in members) for members in groups]
    heights = [
        sum(sizes[m][1] for m in members) + ROW_GAP * (len(members) - 1)
        for members in groups
    ]

    min_index = [min(index_of[m] for m in members) for members in groups]
    # Each member's offset from the top of its own group's block, so a
    # pin on the second member of a cycle is measured from the block.
    member_dy = {}
    for members in groups:
        cursor = 0.0
        for member in members:
            member_dy[member] = cursor
            cursor += sizes[member][1] + ROW_GAP
    rows_of = []
    pin_of = {}
    for g, members in enumerate(groups):
        rows, pins = _row_groups(
            g, members, succ, group_of, pin_dy, pin_pair, index_of,
            min_index, sizes, member_dy
        )
        rows_of.append(rows)
        pin_of.update(pins)
    predecessors_of = [set() for _ in groups]
    for group, rows in enumerate(rows_of):
        for row in rows:
            for target in row:
                predecessors_of[target].add(group)

    columns, min_index = _columns(rows_of, predecessors_of, groups, index_of)
    columns = _wrap_tall_columns(columns, widths, heights, min_index)
    parents = _tree_parents(predecessors_of, columns, min_index)
    children_of = []
    for group, rows in enumerate(rows_of):
        kept_rows = []
        for row in rows:
            kept = [t for t in row if parents[t] == group]
            if kept:
                kept_rows.append(kept)
        children_of.append(kept_rows)

    roots = sorted(
        (g for g in range(len(groups)) if parents[g] is None),
        key=min_index.__getitem__,
    )
    # Post-order without recursion: a rig chain can be deeper than the
    # interpreter's stack allows, and an arrange must never raise.
    visit = list(reversed(roots))
    post = []
    while visit:
        group = visit.pop()
        post.append(group)
        for row in children_of[group]:
            visit.extend(row)
    post.reverse()

    own_y = [0.0] * len(groups)
    child_dy = [0.0] * len(groups)
    skyline = [None] * len(groups)
    for group in post:
        block = {}
        row_targets = []
        for row in children_of[group]:
            # One row of dependents, stacked in the column beside this
            # node, each pushed clear of the one before it wherever the
            # two share a column.
            row_block = {}
            placed = []
            for target in row:
                shift = _clearance(row_block, skyline[target])
                placed.append((target, shift))
                row_block = _merge(row_block, skyline[target], shift)
                skyline[target] = None
            drop = _clearance(block, row_block)
            for target, shift in placed:
                child_dy[target] = drop + shift
            block = _merge(block, row_block, drop)
            row_targets.extend(row)
        if not row_targets:
            own_y[group] = 0.0
        else:
            # Levelled on the PINS, not the boxes. Each child asks this
            # node to sit where its own pin meets the pin that feeds it;
            # with one child that is exact, and with a fan the median
            # answer is the one that leaves the stem in the middle of
            # what it reaches. Medians, not midpoints, because one very
            # tall dependent should not drag the stem off the rest.
            wanted = []
            for target in row_targets:
                src_dy, dst_dy = pin_of.get(
                    (group, target), (heights[group] * 0.5, heights[target] * 0.5)
                )
                wanted.append(
                    child_dy[target] + own_y[target] + dst_dy - src_dy
                )
            own_y[group] = median(wanted)
        block = _merge(
            block,
            {columns[group]: (own_y[group], own_y[group] + heights[group])},
            0.0,
        )
        skyline[group] = block

    origin = [0.0] * len(groups)
    stacked = {}
    for root in roots:
        drop = _clearance(stacked, skyline[root])
        if drop:
            # Two trees that share columns read as one picture unless the
            # gap between them is plainly bigger than the gap inside them.
            drop += ROW_GAP
        origin[root] = drop
        stacked = _merge(stacked, skyline[root], drop)
        skyline[root] = None
    for group in reversed(post):
        for row in children_of[group]:
            for target in row:
                origin[target] = origin[group] + child_dy[target]

    tops = [origin[g] + own_y[g] for g in range(len(groups))]
    xs = _column_positions(
        columns, widths, groups, group_of, incident, tops, sizes
    )

    positions = {}
    for group, members in enumerate(groups):
        cursor = tops[group]
        for member in members:
            positions[member] = (xs[columns[group]], cursor)
            cursor += sizes[member][1] + ROW_GAP
    top = min(y for _x, y in positions.values())
    return {n: (x, y - top) for n, (x, y) in positions.items()}


def _column_positions(columns, widths, groups, group_of, incident, tops, sizes):
    """x of every column, the gap widening for the steepest link.

    A bezier is drawn with horizontal tangents, so its rise has to fit
    inside its run: at the 150px minimum a link that drops a thousand
    pixels renders as a vertical stripe through whatever is behind it. A
    row of dependents hangs off one pin and drops away from it, so this
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
