#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Unit tests for ``UsdNoodles.nodeLayout`` and its view adapter.

The layout core is pure (dicts in, dict out), so most of this file needs
no Qt, no GL and no USD beyond importing the package. What is asserted,
in order of how much it would cost to get wrong:

  * data flows left to right: every edge's source sits left of its
    target unless a cycle forbids it;
  * the layout lines up PINS, not boxes: a lone edge levels its two
    endpoints exactly, a fan-out centres on its stem, and the column gap
    opens up when a link has to drop;
  * a source stands beside what it feeds instead of in column zero;
  * cycles are best effort: members share one column, ordering stays
    deterministic, and nothing hangs or explodes;
  * nothing ever overlaps, on hand-built graphs and seeded fuzz;
  * unwired nodes park in a grid below the flow rather than inside it;
  * ``place_node`` lands one new node beside its neighbors (or the
    fallback), sliding only it to a free slot;
  * ``GraphView._placeAddedNodes`` moves only (0, 0) additions and never
    ranks by a relationship link;
  * the whole thing is fast: a 3000-node DAG lays out in a fraction of
    the generous bound below.
"""

import time
import unittest
from random import Random
from statistics import median
from types import SimpleNamespace
from unittest.mock import MagicMock

try:
    from UsdNoodles import nodeLayout as nl

    _has_layout = True
except ImportError as e:  # pragma: no cover - import guard
    print(f"UsdNoodles.nodeLayout import failed: {e}")
    _has_layout = False

try:
    from UsdNoodles.graphView import GraphView

    _has_graph = True
except ImportError as e:  # pragma: no cover - import guard
    print(f"GraphView import failed: {e}")
    _has_graph = False


def _sizes(*ids, w=200.0, h=100.0):
    return {node_id: (w, h) for node_id in ids}


def _assert_no_overlap(test, sizes, positions):
    boxes = [
        (x, y, x + sizes[n][0], y + sizes[n][1])
        for n, (x, y) in positions.items()
    ]
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            overlaps = a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]
            test.assertFalse(
                overlaps, "nodes %d and %d overlap" % (i, j)
            )


@unittest.skipUnless(_has_layout, "UsdNoodles.nodeLayout unavailable")
class TestLayoutGraph(unittest.TestCase):
    def test_empty_graph(self):
        self.assertEqual(nl.layout_graph({}, []), {})

    def test_single_node_at_origin(self):
        self.assertEqual(nl.layout_graph({"A": (200.0, 100.0)}, []), {"A": (0.0, 0.0)})

    def test_chain_ranks_left_to_right(self):
        pos = nl.layout_graph(
            _sizes("A", "B", "C"), [("A", "B"), ("B", "C")]
        )
        self.assertLess(pos["A"][0], pos["B"][0])
        self.assertLess(pos["B"][0], pos["C"][0])
        self.assertEqual(pos["A"][1], pos["B"][1])
        self.assertEqual(pos["B"][1], pos["C"][1])

    def test_fan_out_shares_a_column(self):
        pos = nl.layout_graph(
            _sizes("A", "B", "C"), [("A", "B"), ("A", "C")]
        )
        self.assertLess(pos["A"][0], pos["B"][0])
        self.assertEqual(pos["B"][0], pos["C"][0])
        self.assertNotEqual(pos["B"][1], pos["C"][1])
        _assert_no_overlap(self, _sizes("A", "B", "C"), pos)

    def test_diamond(self):
        pos = nl.layout_graph(
            _sizes("A", "B", "C", "D"),
            [("A", "B"), ("A", "C"), ("B", "D"), ("C", "D")],
        )
        self.assertLess(pos["A"][0], pos["B"][0])
        self.assertEqual(pos["B"][0], pos["C"][0])
        self.assertLess(pos["B"][0], pos["D"][0])
        _assert_no_overlap(self, _sizes("A", "B", "C", "D"), pos)

    def test_cycle_shares_one_column(self):
        pos = nl.layout_graph(
            _sizes("A", "B", "C"), [("A", "B"), ("B", "C"), ("C", "A")]
        )
        self.assertEqual(pos["A"][0], pos["B"][0])
        self.assertEqual(pos["B"][0], pos["C"][0])
        self.assertEqual(len({pos[n][1] for n in "ABC"}), 3)
        _assert_no_overlap(self, _sizes("A", "B", "C"), pos)

    def test_feedback_pair_sits_right_of_its_driver(self):
        pos = nl.layout_graph(
            _sizes("A", "B", "C"), [("A", "B"), ("B", "C"), ("C", "B")]
        )
        self.assertLess(pos["A"][0], pos["B"][0])
        self.assertEqual(pos["B"][0], pos["C"][0])
        _assert_no_overlap(self, _sizes("A", "B", "C"), pos)

    def test_cycle_members_stay_adjacent(self):
        # A feedback pair whose members pull toward opposite rows, plus an
        # unrelated row aimed between them: the pair orders as one block,
        # never split by the outsider sliding between its members.
        pos = nl.layout_graph(
            _sizes("A", "B", "M", "X", "Y"),
            [("A", "B"), ("B", "A"), ("A", "X"), ("B", "Y"), ("M", "X")],
        )
        self.assertEqual(pos["A"][0], pos["B"][0])
        column = "".join(
            sorted(
                [n for n in "ABMXY" if pos[n][0] == pos["A"][0]],
                key=lambda n: pos[n][1],
            )
        )
        self.assertTrue(
            "AB" in column or "BA" in column,
            "cycle members split apart: %s" % column,
        )

    def test_self_loop_is_ignored(self):
        pos = nl.layout_graph(_sizes("A"), [("A", "A")])
        self.assertEqual(pos, {"A": (0.0, 0.0)})

    def test_dangling_edges_are_ignored(self):
        pos = nl.layout_graph(_sizes("A"), [("A", "gone"), ("gone", "A")])
        self.assertEqual(pos, {"A": (0.0, 0.0)})

    def test_disconnected_components_stack_without_overlap(self):
        sizes = _sizes("A", "B", "C", "D")
        pos = nl.layout_graph(sizes, [("A", "B"), ("C", "D")])
        self.assertEqual(set(pos), {"A", "B", "C", "D"})
        _assert_no_overlap(self, sizes, pos)
        # Two stacked components: one sits strictly below the other.
        tops = sorted({pos["A"][1], pos["C"][1]})
        self.assertEqual(len(tops), 2)


@unittest.skipUnless(_has_layout, "UsdNoodles.nodeLayout unavailable")
class TestPinAlignment(unittest.TestCase):
    """The point of the whole exercise: what lines up is the pins."""

    def test_one_edge_levels_its_two_pins_exactly(self):
        # A 100px-down output feeding a 2500px-down input: centering the
        # boxes would leave the noodle sweeping the height of the target,
        # so the pins, not the boxes, are what agree.
        sizes = {"A": (200.0, 1000.0), "B": (200.0, 3000.0)}
        pos = nl.layout_graph(sizes, [("A", "B", 100.0, 2500.0)])
        self.assertEqual(pos["A"][1] + 100.0, pos["B"][1] + 2500.0)

    def test_two_tuple_edges_aim_at_the_centres(self):
        # No offsets given: each end defaults to its own half height, so
        # the boxes centre on each other exactly as they used to.
        sizes = {"A": (200.0, 1000.0), "B": (200.0, 3000.0)}
        pos = nl.layout_graph(sizes, [("A", "B")])
        self.assertEqual(pos["A"][1] + 500.0, pos["B"][1] + 1500.0)

    def test_mixed_tuple_lengths_lay_out_together(self):
        sizes = _sizes("A", "B", "C")
        pos = nl.layout_graph(sizes, [("A", "B", 10.0, 90.0), ("B", "C")])
        self.assertEqual(pos["A"][1] + 10.0, pos["B"][1] + 90.0)
        self.assertLess(pos["A"][0], pos["B"][0])
        self.assertLess(pos["B"][0], pos["C"][0])

    def test_fan_out_is_contiguous_and_centred_on_its_stem(self):
        # Three outputs of one node: they stack unbroken in the column
        # beside it, and the middle one is level with the stem's pin.
        sizes = {"A": (2000.0, 200.0)}
        sizes.update({n: (2000.0, 1000.0) for n in "BCD"})
        pos = nl.layout_graph(
            sizes, [("A", "B"), ("A", "C"), ("A", "D")]
        )
        self.assertEqual(len({pos[n][0] for n in "BCD"}), 1)
        self.assertGreater(pos["B"][0], pos["A"][0])
        self.assertEqual(
            median(pos[n][1] + 500.0 for n in "BCD"), pos["A"][1] + 100.0
        )
        _assert_no_overlap(self, sizes, pos)

    def test_two_fans_in_one_column_stay_unbroken(self):
        # Two stems feeding one column: each fan is a contiguous run, so
        # neither is interleaved with the other's rows.
        sizes = {n: (2000.0, 600.0) for n in ("S", "A", "X")}
        sizes.update({n: (2000.0, 600.0) for n in ("B", "C", "Y", "Z")})
        pos = nl.layout_graph(
            sizes,
            [
                ("S", "A"), ("S", "X"),
                ("A", "B"), ("A", "C"), ("X", "Y"), ("X", "Z"),
            ],
        )
        column = sorted(("B", "C", "Y", "Z"), key=lambda n: pos[n][1])
        self.assertEqual(len({pos[n][0] for n in column}), 1)
        self.assertIn(
            "".join(column),
            ("BCYZ", "CBYZ", "BCZY", "CBZY", "YZBC", "ZYBC", "YZCB", "ZYCB"),
            "fans interleaved: %s" % "".join(column),
        )

    def test_source_is_pulled_beside_its_consumer(self):
        # S feeds only the last node of a chain: it belongs one column
        # left of that node, not back at the start with A.
        sizes = _sizes("A", "B", "C", "D", "S")
        pos = nl.layout_graph(
            sizes, [("A", "B"), ("B", "C"), ("C", "D"), ("S", "D")]
        )
        self.assertEqual(pos["S"][0], pos["C"][0])
        self.assertLess(pos["S"][0], pos["D"][0])

    def test_column_gap_opens_up_for_a_steep_link(self):
        # Two tall outputs cannot both be level with their stem, so the
        # links have to drop; the gap grows so the bezier has the run to
        # do it in, while a level pair keeps the tight minimum.
        level = {"A": (2000.0, 2000.0), "B": (2000.0, 2000.0)}
        level_pos = nl.layout_graph(level, [("A", "B")])
        tight = level_pos["B"][0] - (level_pos["A"][0] + 2000.0)
        self.assertEqual(tight, nl.COLUMN_GAP)

        steep = {"A": (2000.0, 200.0), "B": (2000.0, 2000.0), "C": (2000.0, 2000.0)}
        steep_pos = nl.layout_graph(steep, [("A", "B"), ("A", "C")])
        opened = steep_pos["B"][0] - (steep_pos["A"][0] + 2000.0)
        self.assertGreater(opened, tight)
        self.assertLessEqual(opened, 6.0 * nl.COLUMN_GAP)


@unittest.skipUnless(_has_layout, "UsdNoodles.nodeLayout unavailable")
class TestColumnWrapping(unittest.TestCase):
    def test_pile_taller_than_the_flow_is_wide_wraps(self):
        # Six 2000px rows feeding one node stack 12000px high against a
        # flow barely 4000px wide: that is a tower, not a column, so it
        # wraps into sub-columns left of the sink.
        sources = ["s%d" % i for i in range(6)]
        sizes = {"sink": (2000.0, 400.0)}
        sizes.update({s: (2000.0, 2000.0) for s in sources})
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        source_xs = sorted({pos[s][0] for s in sources})
        self.assertGreater(len(source_xs), 1)
        self.assertLess(max(source_xs), pos["sink"][0])
        _assert_no_overlap(self, sizes, pos)

    def test_a_pile_that_fits_is_left_whole(self):
        # The same six rows in a graph wide enough to carry them: no
        # wrap, because the column is no taller than the flow is wide.
        sources = ["s%d" % i for i in range(6)]
        sizes = {"sink": (9000.0, 400.0)}
        sizes.update({s: (9000.0, 2000.0) for s in sources})
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        self.assertEqual(len({pos[s][0] for s in sources}), 1)
        _assert_no_overlap(self, sizes, pos)

    def test_wrap_chunks_the_ordered_layer_contiguously(self):
        # Indistinguishable sources keep insertion order, so each wrapped
        # sub-column is a run of neighbors, never a shuffle.
        sources = ["s%d" % i for i in range(25)]
        sizes = _sizes("sink", *sources)
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        by_column = {}
        for source in sources:
            by_column.setdefault(pos[source][0], []).append(source)
        self.assertGreater(len(by_column), 1)
        seen = 0
        for x in sorted(by_column):
            chunk = by_column[x]
            self.assertEqual(chunk, sources[seen:seen + len(chunk)])
            seen += len(chunk)
        self.assertEqual(seen, len(sources))

    def test_zero_height_rows_fall_back_to_the_count_cap(self):
        # Never-laid-out rows report height 0, so no height cap can bound
        # them; the count cap does: [12, 12, 6].
        sources = ["s%d" % i for i in range(30)]
        sizes = {"sink": (200.0, 100.0)}
        sizes.update({s: (200.0, 0.0) for s in sources})
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        self.assertEqual(len({pos[s][0] for s in sources}), 3)
        self.assertEqual(
            len([s for s in sources if pos[s][0] == pos["s0"][0]]),
            nl.MAX_COLUMN_NODES,
        )
        self.assertEqual(len(pos), 31)


@unittest.skipUnless(_has_layout, "UsdNoodles.nodeLayout unavailable")
class TestSingletons(unittest.TestCase):
    def test_singletons_park_below_the_connected_block(self):
        sizes = _sizes("A", "B", "x", "y", "z")
        pos = nl.layout_graph(sizes, [("A", "B")])
        flow_bottom = max(pos[n][1] + sizes[n][1] for n in ("A", "B"))
        for node_id in ("x", "y", "z"):
            self.assertGreaterEqual(
                pos[node_id][1], flow_bottom + 2.0 * nl.ROW_GAP
            )
        _assert_no_overlap(self, sizes, pos)

    def test_singletons_never_widen_the_flow(self):
        # Parked underneath, the grid costs the flow no horizontal room:
        # the wired pair sits exactly where it would with no scopes at all.
        sizes = _sizes("A", "B", *["s%d" % i for i in range(20)])
        pos = nl.layout_graph(sizes, [("A", "B")])
        bare = nl.layout_graph(_sizes("A", "B"), [("A", "B")])
        self.assertEqual(pos["A"], bare["A"])
        self.assertEqual(pos["B"], bare["B"])

    def test_many_singletons_tile_into_rows(self):
        # A hundred unwired nodes must flow into a grid, not one strip:
        # 200-wide nodes against a ~1400 target row fit four across.
        ids = ["n%d" % i for i in range(100)]
        pos = nl.layout_graph(_sizes(*ids), [])
        self.assertEqual(len(pos), 100)
        self.assertEqual(
            sorted({pos[n][0] for n in ids}), [0.0, 350.0, 700.0, 1050.0]
        )
        rows = sorted({pos[n][1] for n in ids})
        self.assertEqual(len(rows), 25)
        _assert_no_overlap(self, _sizes(*ids), pos)


@unittest.skipUnless(_has_layout, "UsdNoodles.nodeLayout unavailable")
class TestDeterminism(unittest.TestCase):
    def test_deterministic_under_edge_reorder(self):
        sizes = {n: (200.0, 100.0) for n in "ABCDEFGH"}
        edges = [
            ("A", "B"), ("B", "C"), ("C", "D"), ("A", "D"),
            ("E", "F"), ("F", "G"), ("G", "E"), ("D", "H"),
        ]
        first = nl.layout_graph(sizes, edges)
        second = nl.layout_graph(sizes, list(reversed(edges)))
        self.assertEqual(first, second)

    def test_shuffled_edges_lay_out_identically(self):
        rng = Random(20260918)
        ids = ["n%d" % i for i in range(40)]
        sizes = {n: (200.0 + 40.0 * (i % 4), 100.0 + 30.0 * (i % 5))
                 for i, n in enumerate(ids)}
        edges = []
        for i in range(39):
            edges.append((ids[i], ids[i + 1], 10.0 * (i % 3), 20.0 * (i % 4)))
        for i in range(0, 36, 3):
            edges.append((ids[i], ids[i + 3], 5.0, 15.0))
        expected = nl.layout_graph(sizes, edges)
        for _ in range(5):
            shuffled = list(edges)
            rng.shuffle(shuffled)
            self.assertEqual(nl.layout_graph(sizes, shuffled), expected)

    def test_shuffled_nodes_still_place_everything(self):
        # Node insertion order IS the tie-break, so the picture may
        # differ; what may not differ is that every node is placed, once,
        # without overlap, and without blowing up.
        rng = Random(4242)
        ids = ["n%d" % i for i in range(40)]
        sizes = {n: (200.0, 100.0 + 20.0 * (i % 6)) for i, n in enumerate(ids)}
        edges = [(ids[i], ids[i + 1]) for i in range(39)]
        edges += [(ids[i], ids[i + 5]) for i in range(0, 30, 5)]
        for _ in range(5):
            order = list(ids)
            rng.shuffle(order)
            shuffled_sizes = {n: sizes[n] for n in order}
            shuffled_edges = list(edges)
            rng.shuffle(shuffled_edges)
            pos = nl.layout_graph(shuffled_sizes, shuffled_edges)
            self.assertEqual(set(pos), set(ids))
            _assert_no_overlap(self, sizes, pos)

    def test_fuzz_never_overlaps_and_places_everything(self):
        rng = Random(20260918)
        for trial in range(5):
            count = 150
            ids = ["n%d" % i for i in range(count)]
            sizes = {
                node_id: (100.0 + rng.randrange(5) * 40.0, 80.0 + rng.randrange(3) * 30.0)
                for node_id in ids
            }
            edges = [
                (rng.choice(ids), rng.choice(ids), rng.randrange(80), rng.randrange(80))
                for _ in range(count * 3)
            ]
            pos = nl.layout_graph(sizes, edges)
            self.assertEqual(set(pos), set(ids), "trial %d" % trial)
            _assert_no_overlap(self, sizes, pos)

    def test_large_dag_is_fast(self):
        layers, width = 60, 50
        ids = [(layer, i) for layer in range(layers) for i in range(width)]
        sizes = {node_id: (200.0, 100.0) for node_id in ids}
        rng = Random(7)
        edges = []
        for layer in range(layers - 1):
            for i in range(width):
                for _ in range(4):
                    edges.append(((layer, i), (layer + 1, rng.randrange(width))))
        started = time.time()
        pos = nl.layout_graph(sizes, edges)
        elapsed = time.time() - started
        self.assertEqual(len(pos), layers * width)
        # ~60ms measured; the bound only catches algorithmic blowups.
        self.assertLess(elapsed, 5.0, "3000-node layout took %.2fs" % elapsed)


@unittest.skipUnless(_has_layout, "UsdNoodles.nodeLayout unavailable")
class TestPlaceNode(unittest.TestCase):
    def test_with_inputs_lands_past_the_rightmost(self):
        x, y = nl.place_node(
            (200.0, 100.0),
            [(0.0, 0.0, 200.0, 100.0), (0.0, 200.0, 200.0, 100.0)],
            [],
            [],
            (999.0, 999.0),
        )
        self.assertEqual(x, 200.0 + nl.COLUMN_GAP)
        self.assertEqual(y, 150.0 - 50.0)

    def test_with_outputs_only_lands_before_the_leftmost(self):
        x, y = nl.place_node(
            (200.0, 100.0),
            [],
            [(500.0, 300.0, 200.0, 100.0)],
            [],
            (999.0, 999.0),
        )
        self.assertEqual(x, 500.0 - nl.COLUMN_GAP - 200.0)
        self.assertEqual(y, 300.0)

    def test_inputs_win_over_outputs(self):
        x, _y = nl.place_node(
            (200.0, 100.0),
            [(0.0, 0.0, 200.0, 100.0)],
            [(900.0, 0.0, 200.0, 100.0)],
            [],
            (999.0, 999.0),
        )
        self.assertEqual(x, 200.0 + nl.COLUMN_GAP)

    def test_unconnected_takes_the_fallback(self):
        self.assertEqual(
            nl.place_node((200.0, 100.0), [], [], [], (111.0, 222.0)),
            (111.0, 222.0),
        )

    def test_slides_to_the_first_free_gap(self):
        # Preferred y is 0; the band is occupied 0..100, so the node must
        # clear it by ROW_GAP, not teleport.
        x, y = nl.place_node(
            (200.0, 100.0),
            [],
            [],
            [(350.0, 0.0, 200.0, 100.0)],
            (350.0, 0.0),
        )
        self.assertEqual(x, 350.0)
        self.assertEqual(y, 100.0 + nl.ROW_GAP)

    def test_ignores_occupants_outside_the_band(self):
        x, y = nl.place_node(
            (200.0, 100.0),
            [(0.0, 0.0, 200.0, 100.0)],
            [],
            [(0.0, 0.0, 200.0, 100.0)],  # the input, left of the band
            (999.0, 999.0),
        )
        self.assertEqual((x, y), (350.0, 0.0))


def _tracked_node(node_id, x, y, w=200.0, h=100.0):
    node = SimpleNamespace(
        id=node_id,
        position=(x, y),
        size=(w, h),
        inputLinks=[],
        outputLinks=[],
    )

    def _set(pos, _node=node):
        _node.position = (float(pos[0]), float(pos[1]))

    node.setDisplayPosition = MagicMock(side_effect=_set)
    return node


def _link(src, dst, relationship=False):
    return SimpleNamespace(
        sourceNodeId=src,
        targetNodeId=dst,
        is_relationship_link=relationship,
    )


@unittest.skipUnless(_has_graph, "GraphView not available")
class TestPlaceAddedNodes(unittest.TestCase):
    def test_connected_addition_lands_beside_its_input(self):
        pred = _tracked_node("P", 500.0, 500.0)
        new = _tracked_node("N", 0.0, 0.0)
        new.inputLinks = [_link("P", "N")]
        view = SimpleNamespace(nodes={"P": pred, "N": new})
        placed = GraphView._placeAddedNodes(view, [new], fallback=(0.0, 0.0))
        self.assertEqual(placed, [new])
        self.assertEqual(new.position, (850.0, 500.0))
        pred.setDisplayPosition.assert_not_called()

    def test_authored_position_is_untouched(self):
        pred = _tracked_node("P", 500.0, 500.0)
        keep = _tracked_node("K", 10.0, 20.0)
        keep.inputLinks = [_link("P", "K")]
        view = SimpleNamespace(nodes={"P": pred, "K": keep})
        placed = GraphView._placeAddedNodes(view, [keep], fallback=(0.0, 0.0))
        self.assertEqual(placed, [])
        self.assertEqual(keep.position, (10.0, 20.0))
        keep.setDisplayPosition.assert_not_called()

    def test_relationship_neighbor_places_like_any_link(self):
        # A relationship is still a noodle the user reads: the new node
        # stands beside it, not at the fallback.
        other = _tracked_node("O", 500.0, 500.0)
        new = _tracked_node("N", 0.0, 0.0)
        new.inputLinks = [_link("O", "N", relationship=True)]
        view = SimpleNamespace(nodes={"O": other, "N": new})
        GraphView._placeAddedNodes(view, [new], fallback=(111.0, 222.0))
        self.assertEqual(new.position, (850.0, 500.0))

    def test_batch_places_sequentially(self):
        pred = _tracked_node("P", 500.0, 500.0)
        first = _tracked_node("A", 0.0, 0.0)
        second = _tracked_node("B", 0.0, 0.0)
        first.inputLinks = [_link("P", "A")]
        second.inputLinks = [_link("A", "B")]
        view = SimpleNamespace(nodes={"P": pred, "A": first, "B": second})
        placed = GraphView._placeAddedNodes(
            view, [first, second], fallback=(0.0, 0.0)
        )
        self.assertEqual(placed, [first, second])
        # B wires from A's NEW position, not its (0, 0) birthplace.
        self.assertEqual(first.position, (850.0, 500.0))
        self.assertEqual(second.position, (1200.0, 500.0))


if __name__ == "__main__":
    unittest.main()
