#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Layered left-to-right auto-layout for the node graph.

Two entry points, both pure (plain dicts and tuples in and out -- no Qt,
no GL, no USD), deterministic (no jitter, stable tie-breaks), and linear:

* :func:`layout_graph` arranges a whole graph: connections run left to
  right, cycles share a layer, over-tall layers wrap into staggered
  sub-columns, and disconnected components tile into rows.
* :func:`place_node` positions ONE new node beside the neighbors it wires
  to, for the add-node path where everything else must stay put.

Every edge ranks, attribute or relationship: the view passes each
rendered noodle straight through, because any connection the user reads
is a connection the layout should respect. Only dangling endpoints (a
node off the canvas) are dropped, by the caller, since there is nothing
to rank them against.

Cycles are best effort by construction: strongly connected components are
condensed before layering, so no feedback edge can stretch the layering
or loop it -- cycle members share one layer and are ordered as a block,
which keeps the contending groups adjacent instead of smeared across
columns.
"""

from __future__ import annotations

import heapq
from statistics import median


COLUMN_GAP = 150.0
ROW_GAP = 60.0

# A layer taller than this wraps into staggered sub-columns instead of
# one unreadable pile: same logical layer (no ordering is implied between
# the sub-columns), consecutive x positions, each packed and relaxed on
# its own. The cap is a HEIGHT, not a count, because readability is
# height: twelve 100px rows are a fine column, but rig nodes run 500 to
# 3000px tall, and a handful of those already needs staggering. Tall fans
# land one-per-column and relax into a readable row beside their node.
MAX_COLUMN_HEIGHT = 3000.0

# Second wrap cap, whichever bites first: past twelve rows a slice wraps
# even when short, so small-node fans stay narrow, while the height cap
# above wraps tall fans sooner. Zero-height (never laid out) rows would
# absorb any height cap whole, which is what makes the count cap load
# bearing rather than redundant.
MAX_COLUMN_NODES = 12

# Down-right step per wrapped slice: after relaxation centers each slice
# on its neighbors, slice origins cascade one row-gap each, so a wrapped
# layer reads as a diagonal staircase instead of aligned stripes. Uniform
# per-slice shifts over x-disjoint slices cannot overlap anything.
CASCADE_DY = ROW_GAP

# Barycenter crossing-reduction sweeps and vertical-relaxation passes for
# the full layout. Small constants: each sweep is O(edges), and two is
# where the visible gain stops.
_BARYCENTER_SWEEPS = 2
_RELAX_PASSES = 2


def layout_graph(sizes, edges):
    """``{node_id: (x, y)}`` for every id in *sizes*.

    *sizes* maps each id to its ``(width, height)``; *edges* is an
    iterable of ``(source_id, target_id)`` data-flow pairs. Edges with an
    endpoint missing from *sizes* and self-loops are ignored. Runs in
    O(V + E) with small constants.
    """
    ids = list(sizes)
    if not ids:
        return {}
    index_of = {node_id: i for i, node_id in enumerate(ids)}
    succ, pred = _adjacency(ids, edges, index_of)

    pieces = []
    for component in _components(ids, succ, pred, index_of):
        local = _layout_component(component, sizes, succ, pred, index_of)
        if not local:
            continue
        left = min(x for x, _y in local.values())
        top = min(y for _x, y in local.values())
        right = max(x + sizes[n][0] for n, (x, _y) in local.items())
        bottom = max(y + sizes[n][1] for n, (_x, y) in local.items())
        pieces.append((local, right - left, bottom - top))
    return _tile_components(pieces)


def _tile_components(pieces):
    """``{node_id: (x, y)}`` with components flowed into rows.

    Every component lays out from its own origin; stacking them all in
    one strip piles hundreds of graphs into the same x band, which reads
    as one giant column. Instead they shelf-pack left to right, wrapping
    to a new row past a square-ish target width, largest first (the order
    they arrive in). A single component is one row and never moves.
    """
    if not pieces:
        return {}
    if len(pieces) == 1:
        (local, _w, _h), = pieces
        left = min(x for x, _y in local.values())
        top = min(y for _x, y in local.values())
        return {n: (x - left, y - top) for n, (x, y) in local.items()}

    total_area = sum(w * h for _local, w, h in pieces)
    row_target = max(
        max(w for _local, w, _h in pieces), total_area**0.5
    )
    rows = [[]]
    row_widths = [0.0]
    for piece in pieces:
        _local, w, _h = piece
        if rows[-1] and row_widths[-1] + COLUMN_GAP + w > row_target:
            rows.append([])
            row_widths.append(0.0)
        rows[-1].append(piece)
        row_widths[-1] += (COLUMN_GAP if row_widths[-1] else 0.0) + w

    positions = {}
    y_cursor = 0.0
    for row in rows:
        x_cursor = 0.0
        row_height = 0.0
        for local, w, h in row:
            left = min(x for x, _y in local.values())
            top = min(y for _x, y in local.values())
            for node_id, (x, y) in local.items():
                positions[node_id] = (x - left + x_cursor, y - top + y_cursor)
            x_cursor += w + COLUMN_GAP
            row_height = max(row_height, h)
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


def _adjacency(ids, edges, index_of):
    """Deduplicated ``(successors, predecessors)`` maps over *ids*.

    Self-loops and edges naming ids outside *sizes* are dropped: a loop
    is not a layering constraint, and a dangling endpoint names a node
    that is not on the canvas.
    """
    succ = {node_id: set() for node_id in ids}
    pred = {node_id: set() for node_id in ids}
    for src, dst in edges:
        if src == dst or src not in pred or dst not in pred:
            continue
        if dst not in succ[src]:
            succ[src].add(dst)
            pred[dst].add(src)
    # Sorted neighbor lists keep every downstream traversal deterministic.
    for node_id in ids:
        succ[node_id] = sorted(succ[node_id], key=index_of.__getitem__)
        pred[node_id] = sorted(pred[node_id], key=index_of.__getitem__)
    return succ, pred


def _components(ids, succ, pred, index_of):
    """Undirected connected components, largest first.

    Order is ``(-size, min member index)`` so repeated layouts of the
    same graph stack components identically.
    """
    parent = {node_id: node_id for node_id in ids}

    def find(node_id):
        root = node_id
        while parent[root] != root:
            root = parent[root]
        while parent[node_id] != root:
            parent[node_id], node_id = root, parent[node_id]
        return root

    for node_id in ids:
        for other in succ[node_id]:
            ra, rb = find(node_id), find(other)
            if ra != rb:
                parent[max(ra, rb, key=index_of.__getitem__)] = min(
                    ra, rb, key=index_of.__getitem__
                )
    groups = {}
    for node_id in ids:
        groups.setdefault(find(node_id), []).append(node_id)
    return sorted(
        groups.values(), key=lambda g: (-len(g), index_of[g[0]])
    )


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


def _layering(component, succ, pred, index_of):
    """``(columns, block_of, block_rank)`` by longest path over condensed SCCs.

    Condensing first is what makes cycles best effort instead of fatal:
    every feedback edge stays inside one group, so layering sees a DAG
    and cycle members land in one column however tangled they are.
    ``block_of`` maps each node to its cycle group and ``block_rank`` each
    group to its lowest member index, for block-aware ordering.
    """
    member_to_group, groups = _strongly_connected(component, succ)
    dag_succ = [set() for _ in groups]
    indegree = [0] * len(groups)
    for node_id in component:
        here = member_to_group[node_id]
        for nxt in succ[node_id]:
            there = member_to_group[nxt]
            if there != here and there not in dag_succ[here]:
                dag_succ[here].add(there)
                indegree[there] += 1
    # Kahn's algorithm, deterministic: the ready group with the lowest
    # member index lays out first.
    ready = [
        (min(index_of[m] for m in groups[g]), g)
        for g in range(len(groups))
        if indegree[g] == 0
    ]
    heapq.heapify(ready)
    group_layer = [0] * len(groups)
    while ready:
        _, group = heapq.heappop(ready)
        for nxt in dag_succ[group]:
            if group_layer[nxt] < group_layer[group] + 1:
                group_layer[nxt] = group_layer[group] + 1
            indegree[nxt] -= 1
            if indegree[nxt] == 0:
                heapq.heappush(
                    ready, (min(index_of[m] for m in groups[nxt]), nxt)
                )
    layers = {}
    for node_id in component:
        layers[node_id] = group_layer[member_to_group[node_id]]
    columns = {}
    for node_id in component:
        columns.setdefault(layers[node_id], []).append(node_id)
    ordered = [columns[k] for k in sorted(columns)]
    block_rank = {
        group: min(index_of[m] for m in members)
        for group, members in enumerate(groups)
    }
    return ordered, member_to_group, block_rank


def _order_layers(columns, succ, pred, index_of, block_of, block_rank):
    """Barycenter crossing reduction, two sweeps, in place.

    Cycle members order as one block (shared barycenter key), so a
    feedback pair is never split apart by an unrelated row sliding
    between its members. Each sweep is O(edges in the layer pair).
    """
    for sweep in range(_BARYCENTER_SWEEPS):
        if sweep % 2 == 0:
            pairs = [
                (moving, fixed, pred)
                for fixed, moving in zip(columns[:-1], columns[1:])
            ]
        else:
            pairs = [
                (moving, fixed, succ)
                for fixed, moving in zip(
                    reversed(columns[1:]), reversed(columns[:-1])
                )
            ]
        for moving, fixed, neighbors in pairs:
            if not fixed or not moving:
                continue
            rank = {node_id: i for i, node_id in enumerate(fixed)}
            bary = {
                node_id: _mean(
                    rank[n] for n in neighbors[node_id] if n in rank
                )
                for node_id in moving
            }
            sums = {}
            counts = {}
            for node_id in moving:
                value = bary[node_id]
                if value is None:
                    continue
                group = block_of[node_id]
                sums[group] = sums.get(group, 0.0) + value
                counts[group] = counts.get(group, 0) + 1

            def _key(node_id):
                group = block_of[node_id]
                if group not in counts:
                    return (True, 0.0, block_rank[group], index_of[node_id])
                return (
                    False,
                    sums[group] / counts[group],
                    block_rank[group],
                    index_of[node_id],
                )

            moving.sort(key=_key)


def _mean(values):
    total = 0.0
    count = 0
    for value in values:
        total += value
        count += 1
    return total / count if count else None


def _layout_component(component, sizes, succ, pred, index_of):
    """``{node_id: (x, y)}`` for one connected component at local origin."""
    columns, block_of, block_rank = _layering(component, succ, pred, index_of)
    _order_layers(columns, succ, pred, index_of, block_of, block_rank)

    # Over-tall layers wrap into staggered sub-columns: contiguous chunks
    # of the ordered layer, so barycenter-adjacent rows stay neighbors.
    # Each sub-column gets its own x slot, packing and relaxation.
    # ``slice_index`` counts slices within their layer (reset per layer)
    # for the diagonal cascade below.
    sub_columns = []
    slice_index = []
    for column in columns:
        for index, chunk in enumerate(_wrap_column(column, sizes)):
            sub_columns.append(chunk)
            slice_index.append(index)
    home_of = {}
    for slot, sub in enumerate(sub_columns):
        for node_id in sub:
            home_of[node_id] = slot

    widths = [max(sizes[n][0] for n in sub) for sub in sub_columns]
    xs = []
    cursor = 0.0
    for width in widths:
        xs.append(cursor)
        cursor += width + COLUMN_GAP

    positions = {}
    for slot, sub in enumerate(sub_columns):
        y = 0.0
        for node_id in sub:
            positions[node_id] = [xs[slot], y]
            y += sizes[node_id][1] + ROW_GAP

    # Vertical relaxation: shift whole sub-columns (never reorder) toward
    # the median of their wired neighbors, so fans center on their stems.
    # Uniform shifts cannot overlap a sub-column with itself, and
    # sub-columns are disjoint in x, so no pass can introduce an overlap.
    for _ in range(_RELAX_PASSES):
        for slot, sub in enumerate(sub_columns):
            column_center = median(
                positions[node_id][1] + sizes[node_id][1] * 0.5
                for node_id in sub
            )
            neighbor_centers = []
            for node_id in sub:
                for other in succ[node_id]:
                    if home_of[other] != slot:
                        neighbor_centers.append(
                            positions[other][1] + sizes[other][1] * 0.5
                        )
                for other in pred[node_id]:
                    if home_of[other] != slot:
                        neighbor_centers.append(
                            positions[other][1] + sizes[other][1] * 0.5
                        )
            if not neighbor_centers:
                continue
            shift = median(neighbor_centers) - column_center
            if shift:
                for node_id in sub:
                    positions[node_id][1] += shift

    # Diagonal cascade, AFTER relaxation (which would erase it: slices
    # sharing one neighbor median recenter identically). Slice origins
    # step down-right per slice, so a wrapped layer reads as a staircase.
    # Uniform per-slice shifts over x-disjoint slices cannot overlap.
    for slot, sub in enumerate(sub_columns):
        step = slice_index[slot] * CASCADE_DY
        if step:
            for node_id in sub:
                positions[node_id][1] += step
    return {node_id: (x, y) for node_id, (x, y) in positions.items()}


def _wrap_column(column, sizes):
    """Contiguous row-chunks of *column* within both caps.

    Rows accumulate until the next would exceed MAX_COLUMN_HEIGHT or the
    chunk reaches MAX_COLUMN_NODES; every chunk holds at least one row.
    Chunk height counts packed rows plus the gaps between them (no
    trailing gap), exactly as the packing above lays them out.
    """
    chunks = []
    current = []
    current_height = 0.0
    for node_id in column:
        row = sizes[node_id][1]
        if current and (
            len(current) >= MAX_COLUMN_NODES
            or current_height + ROW_GAP + row > MAX_COLUMN_HEIGHT
        ):
            chunks.append(current)
            current = []
            current_height = 0.0
        if current:
            current_height += ROW_GAP
        current.append(node_id)
        current_height += row
    if current:
        chunks.append(current)
    return chunks


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
