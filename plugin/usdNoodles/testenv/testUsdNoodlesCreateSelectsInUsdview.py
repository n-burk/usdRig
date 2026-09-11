#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
Creating a node selects the new prim in usdview, not just on the canvas.

"Selected" is two pieces of state. ``node.selected`` lights the node up in the
graph; ``dataModel.selection`` is what the prim tree, the property editor and
the viewport read. Every other selecting path writes both -- click, marquee,
Select All -- and creation wrote only the first, so a freshly created prim was
the one way to leave the two halves of the app disagreeing.

GraphView needs a GL context to construct, so these call its methods against a
stub holding only the attributes they touch (the same approach as
testUsdNoodlesHotboxTargets).
"""

import unittest
from unittest.mock import MagicMock, patch

from pxr import Sdf, Usd, UsdGeom

try:
    from UsdNoodles.graphView import GraphView

    _has_graph_view = True
except ImportError:
    _has_graph_view = False


class _NullContext:
    """Stand-in for dataModel.selection.batchPrimChanges."""

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return False


class _FakeSelection:
    """Records what usdview would end up with selected."""

    def __init__(self):
        self._prims = []
        self.batchPrimChanges = _NullContext()
        self.clearCount = 0

    def clearPrims(self):
        self.clearCount += 1
        self._prims = []

    def addPrim(self, prim):
        self._prims.append(prim)

    def getPrims(self):
        return list(self._prims)

    def getFocusPrim(self):
        return self._prims[-1] if self._prims else None

    def getProps(self):
        return []

    @property
    def paths(self):
        return [str(prim.GetPath()) for prim in self._prims]


class _Node:
    def __init__(self, nodeId):
        self.id = nodeId
        self.selected = False


class _StubGraphView:
    """The slice of GraphView state creation and selection sync read."""

    def __init__(self, stage, frameInPrimTree=False):
        self.selection = _FakeSelection()

        self._usdviewApi = MagicMock()
        self._usdviewApi.stage = stage
        self._usdviewApi.dataModel.selection = self.selection

        self.nodes = {}
        self._selectedNodes = set()
        self._syncingSelection = False
        self._frameInPrimTreeOnSelect = frameInPrimTree
        self._frameInPrimTree = MagicMock()

        self._profiler = MagicMock()
        self._nodeRenderManager = MagicMock()
        self._showPopupMessage = MagicMock()
        self.update = MagicMock()
        self.textChanged = False

        self.nodeGraph = MagicMock()
        self.nodeGraph.getStage.return_value = stage
        # Creation turns this on itself; start it off so the test proves it.
        self.nodeGraph.syncSelectionToPrimTree = False

    # The real implementations under test.
    def clearSelection(self):
        return GraphView.clearSelection(self)

    def _syncSelectionToPrimtree(self):
        return GraphView._syncSelectionToPrimtree(self)

    def _onNodeCreated(self, prim, prim_path, stage, identifier):
        return GraphView._onNodeCreated(self, prim, prim_path, stage, identifier)


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class CreateSelectsInUsdviewTest(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(self.stage, "/World")
        self.newPrim = UsdGeom.Cube.Define(self.stage, "/World/Cube").GetPrim()
        self.newPath = Sdf.Path("/World/Cube")

    def _create(self, view, nodeId="/World/Cube"):
        # Set on the stub, not patched onto GraphView: the stub is not a
        # GraphView instance, so a class-level patch would never resolve.
        view._addNewNodeFast = MagicMock(return_value=nodeId)
        with patch("UsdNoodles.graphView._push_undo_command") as pushUndo:
            view.nodes[nodeId] = _Node(nodeId)
            view._onNodeCreated(
                self.newPrim, self.newPath, self.stage, "Cube"
            )
        return pushUndo

    def test_new_prim_is_selected_in_usdview(self):
        view = _StubGraphView(self.stage)

        self._create(view)

        if view.selection.paths != ["/World/Cube"]:
            print(
                "  usdview selection after create: %r -- the node editor "
                "selected the node but usdview was left alone"
                % (view.selection.paths,)
            )
        self.assertEqual(view.selection.paths, ["/World/Cube"])

    def test_node_is_also_selected_on_the_canvas(self):
        """The half that already worked, kept honest."""
        view = _StubGraphView(self.stage)

        self._create(view)

        self.assertEqual(view._selectedNodes, {"/World/Cube"})
        self.assertTrue(view.nodes["/World/Cube"].selected)

    def test_previous_selection_is_replaced_not_added_to(self):
        view = _StubGraphView(self.stage)
        other = UsdGeom.Xform.Define(self.stage, "/World/Other").GetPrim()
        view.selection.addPrim(other)

        self._create(view)

        self.assertEqual(view.selection.paths, ["/World/Cube"])

    def test_creation_enables_the_sync_itself(self):
        """A graph with syncing off still selects the prim it just made."""
        view = _StubGraphView(self.stage)
        self.assertFalse(view.nodeGraph.syncSelectionToPrimTree)

        self._create(view)

        self.assertTrue(view.nodeGraph.syncSelectionToPrimTree)
        self.assertEqual(view.selection.paths, ["/World/Cube"])

    def test_failed_add_leaves_the_selection_alone(self):
        view = _StubGraphView(self.stage)
        other = UsdGeom.Xform.Define(self.stage, "/World/Other").GetPrim()
        view.selection.addPrim(other)

        view._addNewNodeFast = MagicMock(return_value=None)
        with patch("UsdNoodles.graphView._push_undo_command"):
            view._onNodeCreated(
                self.newPrim, self.newPath, self.stage, "Cube"
            )

        view._showPopupMessage.assert_called_once()
        self.assertEqual(view.selection.paths, ["/World/Other"])

    def test_prim_tree_is_framed_when_the_option_is_on(self):
        view = _StubGraphView(self.stage, frameInPrimTree=True)

        self._create(view)

        view._frameInPrimTree.assert_called_once()
        framed = view._frameInPrimTree.call_args[0][0]
        self.assertEqual(str(framed.GetPath()), "/World/Cube")

    def test_prim_tree_is_not_framed_when_the_option_is_off(self):
        view = _StubGraphView(self.stage, frameInPrimTree=False)

        self._create(view)

        view._frameInPrimTree.assert_not_called()


if __name__ == "__main__":
    unittest.main()
