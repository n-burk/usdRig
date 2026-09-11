#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
The graph must agree with the stage after a structural change.

USD reports a rename as a resync of the old path (now empty) and a resync of
the new one. _handleUsdChanges only ever invalidated caches for paths already
in self.nodes, so it could not see either half: the old node stayed, drawing
from its cached name and refusing every operation that goes through the stage
— it looked present but would not select — and the renamed prim got no node at
all until a manual reload.

These drive the reconciliation directly against a real Usd.Stage, with a stub
standing in for the GL-dependent parts of GraphView, in the same style as the
other graphView tests here.
"""

from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

try:
    from pxr import Sdf, Usd, UsdGeom
    from UsdNoodles.graphView import GraphView

    _has_graph = True
except ImportError as error:  # pragma: no cover - import probe
    print("stage validation imports failed: %s" % error)
    _has_graph = False


def _node_for(stage, path):
    """The slice of NodeModel the reconciliation reads."""
    return SimpleNamespace(
        id=path,
        selected=False,
        getUsdPrim=lambda: stage.GetPrimAtPath(path),
    )


def _make_view(stage, paths, scope="/", selected=()):
    nodes = {path: _node_for(stage, path) for path in paths}
    view = SimpleNamespace(
        nodeGraph=SimpleNamespace(getStage=lambda: stage, nodes=nodes),
        nodes=nodes,
        _selectedNodes=set(selected),
        _marqueeBaseSelectedNodes=set(),
        _marqueePrevSelectedNodes=set(),
        _spatialIndex=MagicMock(),
        _currentPrimPath=None if scope == "/" else scope,
        _addNewNodeFast=MagicMock(side_effect=lambda prim, _stage: str(prim.GetPath())),
    )
    # The real scope rule, not a reimplementation of it: a stub that guessed
    # would pass while the shipped rule was wrong.
    view._graphScopeParentPath = lambda: GraphView._graphScopeParentPath(view)
    return view


@unittest.skipUnless(_has_graph, "graphView module not available")
class ValidateNodesAgainstStageTest(unittest.TestCase):
    """Nodes whose prim is gone must not survive a resync."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        for name in ("Alpha", "Keeper"):
            UsdGeom.Sphere.Define(self.stage, "/%s" % name)

    def test_renamed_prim_leaves_no_node_behind(self) -> None:
        view = _make_view(self.stage, ["/Alpha", "/Keeper"])
        edit = Sdf.BatchNamespaceEdit()
        edit.Add("/Alpha", "/Renamed")
        self.assertTrue(self.stage.GetEditTarget().GetLayer().Apply(edit))

        removed = GraphView._validateNodesAgainstStage(view)

        self.assertEqual(removed, ["/Alpha"])
        self.assertNotIn("/Alpha", view.nodes)
        self.assertIn("/Keeper", view.nodes)

    def test_deleted_prim_leaves_no_node_behind(self) -> None:
        view = _make_view(self.stage, ["/Alpha", "/Keeper"])
        self.stage.RemovePrim("/Alpha")

        self.assertEqual(GraphView._validateNodesAgainstStage(view), ["/Alpha"])
        self.assertEqual(sorted(view.nodes), ["/Keeper"])

    def test_deactivated_prim_leaves_no_node_behind(self) -> None:
        """The loaders skip inactive children, so the view must too."""
        view = _make_view(self.stage, ["/Alpha", "/Keeper"])
        self.stage.GetPrimAtPath("/Alpha").SetActive(False)

        self.assertEqual(GraphView._validateNodesAgainstStage(view), ["/Alpha"])
        self.assertEqual(sorted(view.nodes), ["/Keeper"])

    def test_a_healthy_graph_is_left_alone(self) -> None:
        view = _make_view(self.stage, ["/Alpha", "/Keeper"])
        self.assertEqual(GraphView._validateNodesAgainstStage(view), [])
        self.assertEqual(sorted(view.nodes), ["/Alpha", "/Keeper"])

    def test_removal_clears_selection_and_the_spatial_index(self) -> None:
        """A dropped id left in either place is a click on a dead node."""
        view = _make_view(self.stage, ["/Alpha", "/Keeper"], selected=["/Alpha"])
        view._marqueeBaseSelectedNodes.add("/Alpha")
        view._marqueePrevSelectedNodes.add("/Alpha")
        self.stage.RemovePrim("/Alpha")

        GraphView._validateNodesAgainstStage(view)

        self.assertNotIn("/Alpha", view._selectedNodes)
        self.assertNotIn("/Alpha", view._marqueeBaseSelectedNodes)
        self.assertNotIn("/Alpha", view._marqueePrevSelectedNodes)
        view._spatialIndex.removeNode.assert_called_once_with("/Alpha")

    def test_synthesized_io_nodes_are_never_dropped(self) -> None:
        """Blueprint I/O nodes carry '#' and have no prim to validate."""
        view = _make_view(self.stage, ["/Alpha"])
        view.nodes["/Bp#input"] = SimpleNamespace(
            id="/Bp#input", selected=False, getUsdPrim=lambda: None)

        self.assertEqual(GraphView._validateNodesAgainstStage(view), [])
        self.assertIn("/Bp#input", view.nodes)


@unittest.skipUnless(_has_graph, "graphView module not available")
class AdoptPrimsNewToScopeTest(unittest.TestCase):
    """The other half of a rename: the new prim needs a node."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.Sphere.Define(self.stage, "/Keeper")

    def test_new_prim_in_scope_is_adopted(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Renamed")
        view = _make_view(self.stage, ["/Keeper"])

        added = GraphView._adoptPrimsNewToScope(view, {"/Renamed"})

        self.assertEqual(added, ["/Renamed"])

    def test_prim_already_shown_is_not_added_twice(self) -> None:
        view = _make_view(self.stage, ["/Keeper"])
        self.assertEqual(GraphView._adoptPrimsNewToScope(view, {"/Keeper"}), [])
        view._addNewNodeFast.assert_not_called()

    def test_prim_outside_the_scope_is_ignored(self) -> None:
        """A stage-root graph must not adopt a prim nested under another."""
        UsdGeom.Sphere.Define(self.stage, "/Keeper/Child")
        view = _make_view(self.stage, ["/Keeper"])

        self.assertEqual(GraphView._adoptPrimsNewToScope(view, {"/Keeper/Child"}), [])

    def test_container_scope_adopts_only_its_own_children(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Keeper/Child")
        UsdGeom.Sphere.Define(self.stage, "/Elsewhere")
        view = _make_view(self.stage, [], scope="/Keeper")
        # _graphScopeParentPath only consults _currentPrimPath for the
        # blueprint graphs, so stand one in.
        from UsdNoodles.nodeGraphBlueprint import NodeGraphBlueprint

        view.nodeGraph = NodeGraphBlueprint()
        view.nodeGraph.setStage(self.stage)

        added = GraphView._adoptPrimsNewToScope(
            view, {"/Keeper/Child", "/Elsewhere"})

        self.assertEqual(added, ["/Keeper/Child"])

    def test_removed_and_property_paths_are_ignored(self) -> None:
        view = _make_view(self.stage, ["/Keeper"])
        self.assertEqual(
            GraphView._adoptPrimsNewToScope(
                view, {"/Gone", "/Keeper.radius"}),
            [])

    def test_inactive_prim_is_not_adopted(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Hidden")
        self.stage.GetPrimAtPath("/Hidden").SetActive(False)
        view = _make_view(self.stage, ["/Keeper"])

        self.assertEqual(GraphView._adoptPrimsNewToScope(view, {"/Hidden"}), [])



@unittest.skipUnless(_has_graph, "graphView module not available")
class PathRelationTest(unittest.TestCase):
    """``_isPathRelated``: what counts as "this change touches that node".

    The resync loop asks it of every node when a resynced path is not itself a
    node id, and USD names both child prims and properties in that set. Getting
    it wrong is silent in both directions: too narrow and a new pin never
    appears, too wide and editing one prim invalidates the caches of every
    sibling whose name happens to start the same way.
    """

    def _related(self, a, b):
        from UsdNoodles.graphView import _isPathRelated

        # Symmetric by construction; assert that rather than assuming it.
        forward = _isPathRelated(a, b)
        self.assertEqual(forward, _isPathRelated(b, a))
        return forward

    def test_a_path_is_related_to_itself(self):
        self.assertTrue(self._related("/World/Cube", "/World/Cube"))

    def test_a_child_prim_is_related_to_its_ancestor(self):
        self.assertTrue(self._related("/World/Cube/Leg", "/World/Cube"))
        self.assertTrue(self._related("/World/Cube/Leg/Toe", "/World"))

    def test_a_property_is_related_to_its_prim(self):
        """Creating or removing an attribute resyncs the PROPERTY path, and
        that is what makes a new pin show up on the node."""
        self.assertTrue(self._related("/World/Cube.inputs:x", "/World/Cube"))
        self.assertTrue(self._related("/World/Cube.myRel", "/World/Cube"))

    def test_a_name_prefix_is_not_a_relation(self):
        self.assertFalse(self._related("/World/Cube", "/World/CubeTwo"))
        self.assertFalse(self._related("/World/Cube.inputs:x", "/World/Cub"))
        self.assertFalse(self._related("/World/Cube/Leg", "/World/Cub"))

    def test_unrelated_paths_are_unrelated(self):
        self.assertFalse(self._related("/A/B", "/C/D"))

    def test_everything_is_under_the_pseudo_root(self):
        self.assertTrue(self._related("/", "/World"))
        self.assertTrue(self._related("/", "/World/Cube.inputs:x"))


@unittest.skipUnless(_has_graph, "graphView module not available")
class NodeIdPrimPathTest(unittest.TestCase):
    """``_nodeIdPrimPath``: node ids are prim paths, except when they are not."""

    def _primPath(self, nodeId):
        from UsdNoodles.graphView import _nodeIdPrimPath

        return _nodeIdPrimPath(nodeId)

    def test_a_plain_node_id_is_already_a_prim_path(self):
        self.assertEqual(self._primPath("/World/Cube"), "/World/Cube")

    def test_a_synthesized_io_node_keeps_only_the_blueprint_path(self):
        self.assertEqual(
            self._primPath("/World/Bp#input:drive:x"), "/World/Bp"
        )



@unittest.skipUnless(_has_graph, "graphView module not available")
class MovedPathsClassificationTest(unittest.TestCase):
    """``UsdNoticeHandler._movedPaths``: which resyncs are a prim MOVING.

    A namespace move arrives as two ordinary resyncs -- one path gone, one path
    appeared -- which is the same shape as a delete plus an unrelated create.
    ``GetPrimResyncType`` is what tells them apart, and everything downstream
    (following the node instead of dropping it) rests on this mapping being
    exactly right. Driven against real USD notices rather than fakes, because
    the classification is USD's, not ours.
    """

    def setUp(self):
        from pxr import Tf
        from UsdNoodles.usdNoticeHandler import UsdNoticeHandler

        self.Tf = Tf
        self.UsdNoticeHandler = UsdNoticeHandler
        self.stage = Usd.Stage.CreateInMemory()
        self.stage.DefinePrim("/World", "Xform")
        self.stage.DefinePrim("/World/A", "Xform")
        self.stage.DefinePrim("/World/A/Kid", "Xform")
        self.seen = []
        self._key = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._onChanged, self.stage
        )

    def tearDown(self):
        self._key.Revoke()

    def _onChanged(self, notice, sender):
        self.seen.append(self.UsdNoticeHandler._movedPaths(notice))

    def _editor(self):
        options = Usd.NamespaceEditor.EditOptions()
        options.allowRelocatesAuthoring = False
        return Usd.NamespaceEditor(self.stage, options)

    def _lastMoved(self):
        return self.seen[-1] if self.seen else None

    def test_a_rename_reports_the_pair(self):
        editor = self._editor()
        editor.RenamePrim(self.stage.GetPrimAtPath("/World/A"), "B")
        self.assertTrue(editor.ApplyEdits())

        self.assertEqual(self._lastMoved(), {"/World/A": "/World/B"})

    def test_a_reparent_reports_the_pair(self):
        editor = self._editor()
        editor.ReparentPrim(
            self.stage.GetPrimAtPath("/World/A/Kid"), self.stage.GetPseudoRoot()
        )
        self.assertTrue(editor.ApplyEdits())

        self.assertEqual(self._lastMoved(), {"/World/A/Kid": "/Kid"})

    def test_a_rename_and_reparent_at_once_reports_the_pair(self):
        editor = self._editor()
        editor.ReparentPrim(
            self.stage.GetPrimAtPath("/World/A/Kid"),
            self.stage.GetPseudoRoot(),
            "Moved",
        )
        self.assertTrue(editor.ApplyEdits())

        self.assertEqual(self._lastMoved(), {"/World/A/Kid": "/Moved"})

    def test_a_delete_is_not_a_move(self):
        """The distinction the whole design turns on -- a deleted prim's node
        must still be dropped."""
        self.stage.RemovePrim("/World/A")

        self.assertEqual(self._lastMoved(), {})

    def test_a_create_is_not_a_move(self):
        self.stage.DefinePrim("/World/New", "Xform")

        self.assertEqual(self._lastMoved(), {})

    def test_a_deactivation_is_not_a_move(self):
        self.stage.GetPrimAtPath("/World/A").SetActive(False)

        self.assertEqual(self._lastMoved(), {})

    def test_a_property_resync_is_not_a_move(self):
        """Property paths appear in the resynced set and must not raise here."""
        self.stage.GetPrimAtPath("/World/A").CreateAttribute(
            "drive", Sdf.ValueTypeNames.Float
        )

        self.assertEqual(self._lastMoved(), {})

    def test_only_the_source_half_is_reported(self):
        """The destination half carries the same pair backwards; reporting both
        would put a spurious new->old entry in the map."""
        editor = self._editor()
        editor.RenamePrim(self.stage.GetPrimAtPath("/World/A"), "B")
        self.assertTrue(editor.ApplyEdits())

        self.assertNotIn("/World/B", self._lastMoved())

    def test_a_usd_without_the_classifier_reports_nothing(self):
        """Older USD builds have no GetPrimResyncType; the caller then falls
        back to plain drop-and-adopt rather than raising."""
        notice = SimpleNamespace(GetResyncedPaths=lambda: [Sdf.Path("/World/A")])

        self.assertEqual(self.UsdNoticeHandler._movedPaths(notice), {})

    def test_a_classifier_that_raises_is_survived(self):
        def boom(_path):
            raise RuntimeError("no")

        notice = SimpleNamespace(
            GetResyncedPaths=lambda: [Sdf.Path("/World/A")],
            GetPrimResyncType=boom,
        )

        self.assertEqual(self.UsdNoticeHandler._movedPaths(notice), {})


if __name__ == "__main__":
    unittest.main()
