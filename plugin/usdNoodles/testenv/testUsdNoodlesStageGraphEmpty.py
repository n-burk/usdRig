#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Stage graphs open empty and reload curated.

Opening Noodles on a stage graphs NO prims: the canvas is the user's --
'A' puts prims on it and 'D' takes them off -- and a reload after undo
preserves exactly that membership instead of re-running an opening scan.
``NodeGraphStage.load`` keeps its root-children scan as the explicit
loader primitive (TouchPose discovery is tested against it directly);
the view just no longer invokes it on open or reload.
"""

import unittest
from unittest.mock import MagicMock

from pxr import Usd

try:
    from UsdNoodles import _usdNoodles as _noodles
    from UsdNoodles.graphView import GraphView
    from UsdNoodles.nodeFactory import NodeFactory
    from UsdNoodles.nodeGraphStage import NodeGraphStage
    from UsdNoodles.nodeLibs.usdPrimLibrary import UsdPrimLibrary

    _has_graph = True
except ImportError as e:  # pragma: no cover - import guard
    print(f"stage-graph empty imports failed: {e}")
    _has_graph = False


def _stage_with_scannable_roots():
    """A stage whose root children a scan WOULD graph.

    ``/S`` is found by the hardcoded-type fallback too; ``/X`` only
    through the prim library. ``/S/N`` is the nested curated node.
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/S", "Shader")
    stage.DefinePrim("/S/N", "Shader")
    stage.DefinePrim("/X", "Xform")
    return stage


def _font_metrics():
    fontMetrics = _noodles.FontMetrics()
    fontMetrics.ascender = 0.8
    fontMetrics.descender = -0.2
    fontMetrics.lineHeight = 1.2
    return fontMetrics


def _measure(text, size):
    return len(text) * size * 0.5


class _OpenStubView:
    """The slice of GraphView that the real ``loadStage`` drives."""

    def __init__(self, stage):
        self.nodeGraph = None
        self._currentPrimPath = "sentinel, loadStage must clear this"
        self._nodeFactory = NodeFactory([UsdPrimLibrary()])
        self.textRenderer = MagicMock()
        self.textRenderer.calculateTextWidth = _measure
        self.fontAtlas = _font_metrics()
        self._noticeHandler = MagicMock()
        self._seedNodeZOrders = MagicMock()
        self._gridPlaceNodes = MagicMock()
        self._resetRenderCachesAfterBulkMove = MagicMock()
        self._initUndoTracking = MagicMock()
        self.frameAll = MagicMock()
        self.update = MagicMock()
        self._stage = stage

    @property
    def nodes(self):
        return self.nodeGraph.nodes

    def loadStage(self, stage):
        return GraphView.loadStage(self, stage)


@unittest.skipUnless(_has_graph, "GraphView modules not available")
class OpenStageGraphsNothingTest(unittest.TestCase):
    def test_premise_the_scan_would_find_the_roots(self):
        """Without this, the test below is vacuous."""
        stage = _stage_with_scannable_roots()
        graph = NodeGraphStage()
        graph.setStage(stage)
        graph.load(
            stage,
            _measure,
            _font_metrics(),
            nodeFactory=NodeFactory([UsdPrimLibrary()]),
        )
        self.assertIn("/S", graph.nodes)
        self.assertIn("/X", graph.nodes)

    def test_load_stage_graphs_no_prims(self):
        view = _OpenStubView(_stage_with_scannable_roots())
        view.loadStage(view._stage)
        self.assertEqual(dict(view.nodes), {})

    def test_load_stage_still_sets_up_the_graph(self):
        stage = _stage_with_scannable_roots()
        view = _OpenStubView(stage)
        view.loadStage(stage)
        self.assertIsInstance(view.nodeGraph, NodeGraphStage)
        self.assertIs(view.nodeGraph.getStage(), stage)
        self.assertTrue(view.nodeGraph.syncSelectionToPrimTree)
        self.assertTrue(view.nodeGraph.linksChanged)
        self.assertIsNone(view._currentPrimPath)
        view._noticeHandler.register.assert_called_once_with(stage)
        view._initUndoTracking.assert_called_once_with(stage)
        view._seedNodeZOrders.assert_called_once_with()
        view._resetRenderCachesAfterBulkMove.assert_called_once_with()
        view.frameAll.assert_not_called()


class _ReloadStubView(_OpenStubView):
    """The slice of GraphView that the real ``_reloadCurrentGraph`` drives."""

    def __init__(self, stage):
        super().__init__(stage)
        self.nodeGraph = NodeGraphStage()
        self.nodeGraph.setStage(stage)
        self._currentPrimPath = None
        self._nextZOrder = 0
        self._clearValueInteractionState = MagicMock()
        self._clearRenderCache = MagicMock()
        self._nodeRenderManager = MagicMock()
        self._cppIconRenderer = MagicMock()
        self._nodeTransformFrame = MagicMock()
        self.textChanged = False
        self.linksChanged = False

    def _addPrimAsNode(self, prim, stage, addedNodeIds):
        return GraphView._addPrimAsNode(self, prim, stage, addedNodeIds)

    def _populateNodeLinksFromPrim(self, node, prim):
        return GraphView._populateNodeLinksFromPrim(self, node, prim)

    def _nodeHasAuthoredPosition(self, node):
        return GraphView._nodeHasAuthoredPosition(self, node)

    def _reloadCurrentGraph(self):
        return GraphView._reloadCurrentGraph(self)


@unittest.skipUnless(_has_graph, "GraphView modules not available")
class ReloadAddsNothingTest(unittest.TestCase):
    def test_reload_preserves_the_curated_canvas(self):
        stage = _stage_with_scannable_roots()
        view = _ReloadStubView(stage)
        added = view._addPrimAsNode(stage.GetPrimAtPath("/S/N"), stage, [])
        self.assertTrue(added, "the nested curated node must land on canvas")
        self.assertEqual(set(view.nodes), {"/S/N"})

        view._reloadCurrentGraph()

        self.assertEqual(
            set(view.nodes),
            {"/S/N"},
            "a reload must not scan the loader's roots onto the canvas",
        )


if __name__ == "__main__":
    unittest.main()
