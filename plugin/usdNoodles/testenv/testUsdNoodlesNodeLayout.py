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
  * cycles are best effort: members share one column, ordering stays
    deterministic, and nothing hangs or explodes;
  * nothing ever overlaps, on hand-built graphs and seeded fuzz;
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

    def test_wide_fan_in_staggers_across_columns(self):
        # The spine-solver shape: many dependencies feeding one node. They
        # must wrap into readable staggered columns, not one pile.
        # 25 100px rows hit the count cap first: [12, 12, 1].
        sources = ["s%d" % i for i in range(25)]
        sizes = _sizes("sink", *sources)
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        source_xs = sorted({pos[s][0] for s in sources})
        self.assertEqual(len(source_xs), 3)
        self.assertLess(max(source_xs), pos["sink"][0])
        _assert_no_overlap(self, sizes, pos)

    def test_wrap_chunks_the_ordered_layer_contiguously(self):
        # Indistinguishable sources keep insertion order, so the wrap is
        # [0:12], [12:24], [24:] and the singleton is the last source.
        sources = ["s%d" % i for i in range(25)]
        sizes = _sizes("sink", *sources)
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        last_x = pos["s24"][0]
        self.assertEqual(
            [s for s in sources if pos[s][0] == last_x], ["s24"]
        )
        self.assertEqual(
            len([s for s in sources if pos[s][0] == pos["s0"][0]]),
            nl.MAX_COLUMN_NODES,
        )

    def test_tall_fan_wraps_by_height_not_count(self):
        # Six 1500px rows are only six nodes but 9000+ units: each must
        # take its own column, forming a readable row beside the sink.
        # (A count cap alone would pile them into one column.)
        sources = ["s%d" % i for i in range(6)]
        sizes = {"sink": (200.0, 100.0)}
        sizes.update({s: (200.0, 1500.0) for s in sources})
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        self.assertEqual(len({pos[s][0] for s in sources}), 6)
        self.assertLess(
            max(pos[s][0] for s in sources), pos["sink"][0]
        )
        _assert_no_overlap(self, sizes, pos)

    def test_wrapped_slices_cascade_down_right(self):
        # Equal-height slices recenter identically, so their origins differ
        # by exactly one cascade step: the wrapped layer reads diagonal.
        sources = ["s%d" % i for i in range(24)]
        sizes = _sizes("sink", *sources)
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        first_x, second_x = sorted({pos[s][0] for s in sources})
        y0 = min(pos[s][1] for s in sources if pos[s][0] == first_x)
        y1 = min(pos[s][1] for s in sources if pos[s][0] == second_x)
        self.assertEqual(y1 - y0, nl.CASCADE_DY)

    def test_zero_height_rows_fall_back_to_the_count_cap(self):
        # Never-laid-out rows report height 0 and would absorb any height
        # cap whole; the count cap bounds the slice instead: [12, 12, 6].
        sources = ["s%d" % i for i in range(30)]
        sizes = {"sink": (200.0, 100.0)}
        sizes.update({s: (200.0, 0.0) for s in sources})
        pos = nl.layout_graph(sizes, [(s, "sink") for s in sources])
        self.assertEqual(len({pos[s][0] for s in sources}), 3)
        self.assertEqual(len(pos), 31)

    def test_many_components_tile_into_rows(self):
        # A hundred isolated nodes must flow into a grid, not one strip:
        # 200-wide pieces against a ~1400 target row fit four across.
        ids = ["n%d" % i for i in range(100)]
        pos = nl.layout_graph(_sizes(*ids), [])
        self.assertEqual(len(pos), 100)
        self.assertEqual(
            sorted({pos[n][0] for n in ids}), [0.0, 350.0, 700.0, 1050.0]
        )
        rows = sorted({pos[n][1] for n in ids})
        self.assertEqual(len(rows), 25)
        _assert_no_overlap(self, _sizes(*ids), pos)

    def test_deterministic_under_edge_reorder(self):
        sizes = {n: (200.0, 100.0) for n in "ABCDEFGH"}
        edges = [
            ("A", "B"), ("B", "C"), ("C", "D"), ("A", "D"),
            ("E", "F"), ("F", "G"), ("G", "E"), ("D", "H"),
        ]
        first = nl.layout_graph(sizes, edges)
        second = nl.layout_graph(sizes, list(reversed(edges)))
        self.assertEqual(first, second)

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
                (rng.choice(ids), rng.choice(ids)) for _ in range(count * 3)
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
        # ~30ms measured; the bound only catches algorithmic blowups.
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
