#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
Renaming a prim from the graph, and everything the rename has to drag with it.

Two halves. The first is the namespace edit itself: a prim's name is its
identity, so children, relationship targets, attribute connections and internal
composition arcs all name the old path and all have to move. Those are the
assertions that matter, because every one of them fails SILENTLY when it is
missed -- a stale relationship target does not error, it just stops resolving.

The second is the inline editor double-click opens: what commits, what
cancels, and what a rejected name does.

GraphView needs a GL context to construct, so the editor tests call its methods
against a stub holding only the attributes they touch (the same approach as
testUsdNoodlesHotboxTargets).
"""

import unittest
from unittest.mock import MagicMock

from pxr import Gf, Sdf, Usd

try:
    from UsdNoodles.primRename import (
        INVALID_NAME,
        NAME_TAKEN,
        REFUSED,
        RENAMED,
        UNCHANGED,
        rename_prim,
        validate_new_name,
    )

    _has_prim_rename = True
except ImportError:
    _has_prim_rename = False

try:
    from UsdNoodles.graphView import GraphView

    _has_graph_view = True
except ImportError:
    _has_graph_view = False


_WIRED = """#usda 1.0

def Xform "World"
{
    def Xform "Old"
    {
        custom float attrA = 1

        def Cube "Child"
        {
            double size = 2
        }
    }

    def Xform "Ref" (
        references = </World/Old>
    )
    {
    }

    def Xform "Inh" (
        inherits = </World/Old>
    )
    {
    }

    def Xform "Spec" (
        specializes = </World/Old>
    )
    {
    }

    def Xform "Pointer"
    {
        rel myRel = [</World/Old>, </World/Old/Child>]
        custom float attrB = 0
        custom float attrB.connect = </World/Old.attrA>
    }
}
"""


def _wiredStage():
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().ImportFromString(_WIRED)
    return stage


@unittest.skipUnless(_has_prim_rename, "primRename module not available")
class RenameReconcilesTest(unittest.TestCase):
    """Everything that named the old path has to name the new one."""

    def setUp(self):
        self.stage = _wiredStage()
        self.prim = self.stage.GetPrimAtPath("/World/Old")

    def _rename(self, name="New"):
        status, newPath, message = rename_prim(self.stage, self.prim, name)
        self.assertEqual(status, RENAMED, message)
        return newPath

    def test_the_prim_itself_moves(self):
        newPath = self._rename()

        self.assertEqual(str(newPath), "/World/New")
        self.assertTrue(self.stage.GetPrimAtPath("/World/New").IsValid())
        self.assertFalse(self.stage.GetPrimAtPath("/World/Old").IsValid())

    def test_children_come_with_it(self):
        self._rename()

        child = self.stage.GetPrimAtPath("/World/New/Child")
        self.assertTrue(child.IsValid())
        self.assertEqual(child.GetAttribute("size").Get(), 2)
        self.assertFalse(self.stage.GetPrimAtPath("/World/Old/Child").IsValid())

    def test_relationship_targets_are_retargeted(self):
        """Both the prim itself and a path THROUGH it."""
        self._rename()

        rel = self.stage.GetRelationshipAtPath("/World/Pointer.myRel")
        self.assertEqual(
            [str(p) for p in rel.GetTargets()],
            ["/World/New", "/World/New/Child"],
        )

    def test_attribute_connections_are_retargeted(self):
        self._rename()

        attr = self.stage.GetAttributeAtPath("/World/Pointer.attrB")
        self.assertEqual(
            [str(p) for p in attr.GetConnections()], ["/World/New.attrA"]
        )

    def test_internal_composition_arcs_are_retargeted(self):
        self._rename()

        text = self.stage.GetRootLayer().ExportToString()
        self.assertNotIn("</World/Old>", text)
        for arc in ("references", "inherits", "specializes"):
            self.assertIn(arc, text)
        # Each of the three arc prims still composes the renamed prim.
        for name in ("Ref", "Inh", "Spec"):
            spec = self.stage.GetRootLayer().GetPrimAtPath(f"/World/{name}")
            self.assertIsNotNone(spec)

    def test_nothing_anywhere_still_names_the_old_path(self):
        """The catch-all: one grep over the whole layer."""
        self._rename()

        text = self.stage.GetRootLayer().ExportToString()
        if "/World/Old" in text:
            print(
                "  the layer still mentions /World/Old after the rename:\n%s"
                % text
            )
        self.assertNotIn("/World/Old", text)


@unittest.skipUnless(_has_prim_rename, "primRename module not available")
class RenameRefusalsTest(unittest.TestCase):
    """A rejected rename authors nothing."""

    def setUp(self):
        self.stage = _wiredStage()
        self.prim = self.stage.GetPrimAtPath("/World/Old")
        self.before = self.stage.GetRootLayer().ExportToString()

    def _assertUntouched(self):
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.before)

    def test_invalid_identifiers_are_rejected(self):
        for bad in ("9lives", "has space", "has-dash", "ns:name", ""):
            status, newPath, message = rename_prim(self.stage, self.prim, bad)
            self.assertEqual(status, INVALID_NAME, f"{bad!r} was accepted")
            self.assertIsNone(newPath)
            self.assertTrue(message)
        self._assertUntouched()

    def test_a_sibling_name_is_rejected(self):
        status, newPath, message = rename_prim(self.stage, self.prim, "Pointer")

        self.assertEqual(status, NAME_TAKEN)
        self.assertIsNone(newPath)
        self.assertIn("already exists", message)
        self._assertUntouched()

    def test_the_same_name_is_a_no_op(self):
        status, newPath, _ = rename_prim(self.stage, self.prim, "Old")

        self.assertEqual(status, UNCHANGED)
        self.assertIsNone(newPath)
        self._assertUntouched()

    def test_validate_agrees_with_rename(self):
        """The per-keystroke check must not accept what the edit rejects."""
        for name in ("New", "9lives", "Pointer", "Old", ""):
            expected, _ = validate_new_name(self.prim, name)
            stage = _wiredStage()
            actual, _, _ = rename_prim(stage, stage.GetPrimAtPath("/World/Old"), name)
            self.assertEqual(expected, actual, f"disagreed about {name!r}")


@unittest.skipUnless(_has_prim_rename, "primRename module not available")
class RenameAcrossAReferenceTest(unittest.TestCase):
    """A prim that arrives across a reference is refused, not relocated."""

    def setUp(self):
        self.ref = Sdf.Layer.CreateAnonymous("ref.usda")
        self.ref.ImportFromString(
            '#usda 1.0\n\ndef Xform "Asset"\n{\n    def Cube "Geo"\n    {\n    }\n}\n'
        )
        self.root = Sdf.Layer.CreateAnonymous("root.usda")
        self.root.ImportFromString(
            '#usda 1.0\n\ndef Xform "World"\n{\n    def Xform "Inst" (\n'
            "        references = @%s@</Asset>\n    )\n    {\n    }\n}\n"
            % self.ref.identifier
        )
        self.stage = Usd.Stage.Open(self.root)

    def test_refused_with_usds_own_reason(self):
        prim = self.stage.GetPrimAtPath("/World/Inst/Geo")
        self.assertTrue(prim.IsValid())
        before = self.root.ExportToString()

        status, newPath, message = rename_prim(self.stage, prim, "GeoRenamed")

        self.assertEqual(status, REFUSED)
        self.assertIsNone(newPath)
        self.assertIn("relocates", message)
        # Nothing authored: no relocates entry, no override.
        self.assertEqual(self.root.ExportToString(), before)
        self.assertNotIn("relocates", self.root.ExportToString())

    def test_a_local_prim_in_the_same_stage_still_renames(self):
        """The refusal is about the arc, not about the stage having one."""
        self.stage.DefinePrim("/World/Local", "Xform")

        status, newPath, message = rename_prim(
            self.stage, self.stage.GetPrimAtPath("/World/Local"), "Renamed"
        )

        self.assertEqual(status, RENAMED, message)
        self.assertEqual(str(newPath), "/World/Renamed")


class _Node:
    def __init__(self, nodeId):
        self.id = nodeId
        self.selected = False


class _StubGraphView:
    """The slice of GraphView state the inline rename touches.

    The node re-keying runs for real (``_remapMovedNodes`` is the production
    method, not a mock), because a stub of it would assert nothing about the
    thing that actually broke. Only the widget-level edges -- popups, repaint,
    the prim-tree sync -- are mocked. ``_reloadCurrentGraph`` stays a mock so
    tests can assert it is NOT reached; reaching it is the bug.
    """

    def __init__(self, stage):
        from UsdNoodles.widgets.textInputWidget import TextInputWidget

        self._usdviewApi = MagicMock()
        self._usdviewApi.stage = stage

        self._renamingNodeId = ""
        self._renameInput = TextInputWidget()

        self._selectedNodes = set()

        self.nodeGraph = MagicMock()
        self.nodeGraph.getStage.return_value = stage
        self.nodeGraph.nodes = {}

        # Everything else _remapMovedNodes carries across a move.
        self._marqueeBaseSelectedNodes = set()
        self._marqueePrevSelectedNodes = set()
        self._reconciledPositionVersions = {}
        self.nodeIdUnderCursor = ""
        self._hoveredPort = None
        self._tooltipProperty = None
        self._dragLinkSourceNode = None
        self._dragLinkSyntheticTarget = None
        self._dragLinkResolvedTarget = None
        self._reconnectOldSourceNodeId = None
        self._reconnectOldTargetNodeId = None
        self._currentPrimPath = None
        self._spatialIndex = MagicMock()

        # Camera. Deliberately not (0, 0, 1): a test that asserts these survive
        # a rename has to be able to tell "unchanged" from "reset".
        self.zoom = 0.35
        self.panX = 137.0
        self.panY = -42.0

        self.linksChanged = False
        self.textChanged = False
        self._syncingSelection = False
        self._nextZOrder = 0
        self.fontAtlas = None

        # _resetRenderCachesAfterBulkMove runs for real (it is what sets
        # linksChanged / textChanged); only the GL-side objects it drives are
        # mocked.
        self._clearRenderCache = MagicMock()
        self.textRenderer = MagicMock()
        self._cppIconRenderer = MagicMock()
        self._nodeRenderManager = MagicMock()
        self._nodeTransformFrame = MagicMock()

        self._invalidateBackReferenceLinkDataCache = MagicMock()
        self._showPopupMessage = MagicMock()
        self._showWarningPopup = MagicMock()
        self._syncSelectionToPrimtree = MagicMock()
        self._reloadCurrentGraph = MagicMock()
        self.update = MagicMock()

    # Mirrors the real GraphView.nodes: a read-only view of nodeGraph.nodes,
    # which _remapMovedNodes replaces wholesale when it re-keys.
    @property
    def nodes(self):
        return self.nodeGraph.nodes

    def clearSelection(self):
        for node in self.nodes.values():
            node.selected = False
        self._selectedNodes.clear()

    def _beginRename(self, nodeId):
        return GraphView._beginRename(self, nodeId)

    def _cancelRename(self):
        return GraphView._cancelRename(self)

    def _commitRename(self):
        return GraphView._commitRename(self)

    def _remapMovedNodes(self, movedPaths):
        return GraphView._remapMovedNodes(self, movedPaths)

    def _resetRenderCachesAfterBulkMove(self, rebuild_links=True):
        return GraphView._resetRenderCachesAfterBulkMove(self, rebuild_links)

    def _handleUsdChanges(self, resynced, infoOnly, movedPaths=None):
        return GraphView._handleUsdChanges(self, resynced, infoOnly, movedPaths)

    def _validateNodesAgainstStage(self):
        return GraphView._validateNodesAgainstStage(self)

    def _adoptPrimsNewToScope(self, resyncedPaths):
        return GraphView._adoptPrimsNewToScope(self, resyncedPaths)

    def _graphScopeParentPath(self):
        return GraphView._graphScopeParentPath(self)

    def _isPinPropertyChange(self, propName):
        return GraphView._isPinPropertyChange(self, propName)

    def _addNewNodeFast(self, prim, stage):
        """Enough of the real thing for reconciliation: a model at the path."""
        from UsdNoodles.models import NodeModel

        pathStr = str(prim.GetPath())
        if pathStr in self.nodes:
            return None
        self.nodes[pathStr] = NodeModel(stage=stage, primPath=prim.GetPath())
        return pathStr


@unittest.skipUnless(
    _has_graph_view and _has_prim_rename, "graphView module not available"
)
class InlineRenameEditorTest(unittest.TestCase):
    """What the double-click editor does."""

    def setUp(self):
        self.stage = _wiredStage()
        self.view = _StubGraphView(self.stage)
        # Only the source node. The destination is NOT pre-seeded: the commit
        # has to move this node there itself, which is the whole point.
        self.view.nodes["/World/Old"] = _Node("/World/Old")

    def test_begin_seeds_the_field_with_the_current_name(self):
        self.view._beginRename("/World/Old")

        self.assertEqual(self.view._renamingNodeId, "/World/Old")
        self.assertEqual(self.view._renameInput.text, "Old")
        self.assertEqual(self.view._renameInput.cursor_position, 3)

    def test_commit_renames_the_prim(self):
        self.view._beginRename("/World/Old")
        self.view._renameInput.text = "New"

        self.view._commitRename()

        self.assertTrue(self.stage.GetPrimAtPath("/World/New").IsValid())
        self.assertFalse(self.stage.GetPrimAtPath("/World/Old").IsValid())
        self.assertEqual(self.view._renamingNodeId, "")

    def test_commit_selects_the_renamed_prim_in_usdview(self):
        self.view._beginRename("/World/Old")
        self.view._renameInput.text = "New"

        self.view._commitRename()

        self.assertEqual(self.view._selectedNodes, {"/World/New"})
        self.view._syncSelectionToPrimtree.assert_called_once()

    def test_cancel_authors_nothing(self):
        before = self.stage.GetRootLayer().ExportToString()
        self.view._beginRename("/World/Old")
        self.view._renameInput.text = "New"

        self.view._cancelRename()

        self.assertEqual(self.view._renamingNodeId, "")
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)

    def test_a_fixable_name_keeps_the_editor_open(self):
        """A typo should not cost the user the whole edit."""
        self.view._beginRename("/World/Old")
        self.view._renameInput.text = "9lives"

        self.view._commitRename()

        self.assertEqual(self.view._renamingNodeId, "/World/Old")
        self.view._showWarningPopup.assert_called_once()
        self.assertTrue(self.stage.GetPrimAtPath("/World/Old").IsValid())

    def test_a_taken_name_keeps_the_editor_open(self):
        self.view._beginRename("/World/Old")
        self.view._renameInput.text = "Pointer"

        self.view._commitRename()

        self.assertEqual(self.view._renamingNodeId, "/World/Old")
        self.view._showWarningPopup.assert_called_once()

    def test_committing_the_same_name_just_closes(self):
        self.view._beginRename("/World/Old")

        self.view._commitRename()

        self.assertEqual(self.view._renamingNodeId, "")
        self.view._showWarningPopup.assert_not_called()
        self.assertTrue(self.stage.GetPrimAtPath("/World/Old").IsValid())

    def test_surrounding_whitespace_is_trimmed(self):
        self.view._beginRename("/World/Old")
        self.view._renameInput.text = "  New  "

        self.view._commitRename()

        self.assertTrue(self.stage.GetPrimAtPath("/World/New").IsValid())

    def test_a_synthesized_io_node_has_no_name_to_change(self):
        self.view.nodes["/World/Old#in"] = _Node("/World/Old#in")

        self.view._beginRename("/World/Old#in")

        self.assertEqual(self.view._renamingNodeId, "")
        self.view._showPopupMessage.assert_called_once()

    def test_a_prim_deleted_under_the_editor_is_reported(self):
        self.view._beginRename("/World/Old")
        self.stage.RemovePrim("/World/Old")
        self.view._renameInput.text = "New"

        self.view._commitRename()

        self.assertEqual(self.view._renamingNodeId, "")
        self.view._showWarningPopup.assert_called_once()


_CURATED = """#usda 1.0

def Xform "World"
{
    def Xform "Deep"
    {
        def Xform "Node"
        {
        }

        def Xform "NodeTwo"
        {
        }

        def Xform "Child"
        {
        }
    }
}

def Xform "Other"
{
}
"""


def _curatedStage():
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().ImportFromString(_CURATED)
    return stage


@unittest.skipUnless(
    _has_graph_view and _has_prim_rename, "graphView module not available"
)
class RenameKeepsTheGraphTest(unittest.TestCase):
    """A rename changes a name. It does not change the graph or the camera.

    The canvas is a curated set: 'A' puts prims on it at any depth and 'D'
    takes them off, neither of which touches USD, so no loader can re-derive
    it. Answering a rename by re-running a loader therefore replaces the user's
    graph with the loader's -- for a stage graph, the stage's top-level prims
    and nothing else. These tests hold the line on the alternative: the node
    follows its prim and everything else stays put.

    Real NodeModels over a real in-memory stage, because the failure modes are
    all in USD-facing state -- an expired prim, a position with no authored
    value behind it -- that a SimpleNamespace cannot reproduce.
    """

    def setUp(self):
        from UsdNoodles.models import NodeModel

        self.stage = _curatedStage()
        self.view = _StubGraphView(self.stage)
        self.NodeModel = NodeModel

        # A nested node (what 'A' produces, and what a stage loader can never
        # find) plus an unrelated top-level one to prove nothing else moves.
        self.node = self._addNode("/World/Deep/Node")
        self.other = self._addNode("/Other")

    def _addNode(self, path, displayPosition=(500.0, 300.0)):
        node = self.NodeModel(stage=self.stage, primPath=Sdf.Path(path))
        if displayPosition is not None:
            node.setDisplayPosition(Gf.Vec2d(*displayPosition))
        self.view.nodes[path] = node
        return node

    def _rename(self, nodeId, newName):
        self.view._beginRename(nodeId)
        self.view._renameInput.text = newName
        self.view._commitRename()

    def test_the_node_follows_its_prim(self):
        self._rename("/World/Deep/Node", "Renamed")

        self.assertEqual(
            set(self.view.nodes), {"/World/Deep/Renamed", "/Other"}
        )

    def test_the_rest_of_the_graph_is_untouched(self):
        self._rename("/World/Deep/Node", "Renamed")

        self.assertIs(self.view.nodes["/Other"], self.other)
        self.assertEqual(tuple(self.other.position), (500.0, 300.0))
        self.assertEqual(self.other.name, "Other")

    def test_it_is_the_same_node_not_a_replacement(self):
        """Identity, not equality: _lastSelectedNode and friends hold the
        object, so a rebuilt model would strand them on a discarded one."""
        self._rename("/World/Deep/Node", "Renamed")

        self.assertIs(self.view.nodes["/World/Deep/Renamed"], self.node)

    def test_the_graph_is_never_reloaded(self):
        self._rename("/World/Deep/Node", "Renamed")

        self.view._reloadCurrentGraph.assert_not_called()

    def test_the_camera_does_not_move(self):
        self._rename("/World/Deep/Node", "Renamed")

        self.assertEqual(self.view.panX, 137.0)
        self.assertEqual(self.view.panY, -42.0)
        self.assertEqual(self.view.zoom, 0.35)

    def test_a_display_only_position_survives(self):
        """Grid placement never authors ui:nodegraph:node:pos, so there is
        nothing on the stage to restore it from -- losing it drops the node on
        the origin."""
        self.assertFalse(
            self.node.getUsdPrim()
            .GetAttribute("ui:nodegraph:node:pos")
            .HasValue()
        )

        self._rename("/World/Deep/Node", "Renamed")

        self.assertEqual(
            tuple(self.view.nodes["/World/Deep/Renamed"].position), (500.0, 300.0)
        )

    def test_an_authored_position_is_still_read_from_usd(self):
        """The rename carries the attribute with the prim, and the node reads
        it back from the new path -- no display-only cache standing in for it."""
        prim = self.stage.GetPrimAtPath("/World/Deep/Child")
        prim.CreateAttribute("ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2).Set(
            Gf.Vec2f(1.0, 2.0)
        )
        self._addNode("/World/Deep/Child", displayPosition=None)

        self._rename("/World/Deep/Child", "Renamed")

        node = self.view.nodes["/World/Deep/Renamed"]
        self.assertTrue(node.hasAuthoredPosition())
        # The getter scales USD's small units up by 1000 for display.
        self.assertEqual(tuple(node.position), (1000.0, 2000.0))

    def test_the_title_becomes_the_new_name(self):
        """Read it first, so the cache actually holds the OLD name: an
        unprimed getter would fall through to the (now-repointed) prim and
        report the right answer whether or not the re-key updated it."""
        self.assertEqual(self.node.name, "Node")
        self.assertEqual(self.node._name, "Node")

        self._rename("/World/Deep/Node", "Renamed")

        self.assertEqual(self.view.nodes["/World/Deep/Renamed"]._name, "Renamed")
        self.assertEqual(self.view.nodes["/World/Deep/Renamed"].name, "Renamed")

    def test_the_model_is_repointed_at_the_live_prim(self):
        """The Usd.Prim captured at construction expires on the namespace edit.
        Left expired, the next resync has _validateNodesAgainstStage drop the
        node as 'its prim is gone'."""
        self._rename("/World/Deep/Node", "Renamed")

        prim = self.view.nodes["/World/Deep/Renamed"].getUsdPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(str(prim.GetPath()), "/World/Deep/Renamed")

    def test_descendant_nodes_move_with_their_ancestor(self):
        """USD reports only the prim that moved; the children move silently."""
        child = self._addNode("/World/Deep/Child")

        self._rename("/World/Deep", "Deeper")

        self.assertIs(self.view.nodes["/World/Deeper/Child"], child)
        self.assertIs(self.view.nodes["/World/Deeper/Node"], self.node)
        self.assertTrue(
            self.view.nodes["/World/Deeper/Child"].getUsdPrim().IsValid()
        )

    def test_a_sibling_sharing_a_name_prefix_is_not_dragged_along(self):
        """/World/Deep/Node is not an ancestor of /World/Deep/NodeTwo, however
        much the strings look like it."""
        sibling = self._addNode("/World/Deep/NodeTwo")

        self._rename("/World/Deep/Node", "Renamed")

        self.assertIn("/World/Deep/NodeTwo", self.view.nodes)
        self.assertIs(self.view.nodes["/World/Deep/NodeTwo"], sibling)
        self.assertEqual(tuple(sibling.position), (500.0, 300.0))

    def test_selection_and_hover_state_follow_the_node(self):
        self.view._selectedNodes.add("/World/Deep/Node")
        self.view._marqueeBaseSelectedNodes.add("/World/Deep/Node")
        self.view._marqueePrevSelectedNodes.add("/World/Deep/Node")
        self.view.nodeIdUnderCursor = "/World/Deep/Node"
        self.view._hoveredPort = ("/World/Deep/Node", "out", True)

        self._rename("/World/Deep/Node", "Renamed")

        self.assertEqual(self.view._marqueeBaseSelectedNodes, {"/World/Deep/Renamed"})
        self.assertEqual(self.view._marqueePrevSelectedNodes, {"/World/Deep/Renamed"})
        self.assertEqual(self.view.nodeIdUnderCursor, "/World/Deep/Renamed")
        self.assertEqual(
            self.view._hoveredPort, ("/World/Deep/Renamed", "out", True)
        )

    def test_the_dead_id_leaves_the_spatial_index_immediately(self):
        """The link rebuild that refills the index is deferred to the next
        paint; a click can land before it."""
        self._rename("/World/Deep/Node", "Renamed")

        self.view._spatialIndex.removeNode.assert_any_call("/World/Deep/Node")

    def test_links_are_scheduled_for_a_rebuild(self):
        """USD retargeted the connections and relationships that named the old
        path; the in-memory LinkData still names it until they are re-derived."""
        self._rename("/World/Deep/Node", "Renamed")

        self.assertTrue(self.view.linksChanged)
        self.view._invalidateBackReferenceLinkDataCache.assert_called()

    def test_the_renamed_node_ends_up_selected(self):
        self._rename("/World/Deep/Node", "Renamed")

        self.assertEqual(self.view._selectedNodes, {"/World/Deep/Renamed"})

    def test_a_graph_scope_that_moved_is_retargeted(self):
        """A blueprint/container graph measures reconciliation against
        _currentPrimPath; leaving it on a dead path empties the graph."""
        self.view._currentPrimPath = Sdf.Path("/World/Deep")

        self.view._remapMovedNodes({"/World": "/World2"})

        self.assertEqual(str(self.view._currentPrimPath), "/World2/Deep")

    def test_several_independent_moves_apply_in_one_pass(self):
        sibling = self._addNode("/World/Deep/NodeTwo")

        self.view._remapMovedNodes(
            {
                "/World/Deep/Node": "/World/Deep/Alpha",
                "/World/Deep/NodeTwo": "/World/Deep/Beta",
            }
        )

        self.assertIs(self.view.nodes["/World/Deep/Alpha"], self.node)
        self.assertIs(self.view.nodes["/World/Deep/Beta"], sibling)
        self.assertNotIn("/World/Deep/Node", self.view.nodes)
        self.assertNotIn("/World/Deep/NodeTwo", self.view.nodes)

    def test_a_move_whose_destination_is_another_moves_source(self):
        """A shuffle in one pass must not have one entry eat the other."""
        sibling = self._addNode("/World/Deep/NodeTwo")

        self.view._remapMovedNodes(
            {
                "/World/Deep/Node": "/World/Deep/NodeTwo",
                "/World/Deep/NodeTwo": "/World/Deep/Child",
            }
        )

        self.assertIs(self.view.nodes["/World/Deep/NodeTwo"], self.node)
        self.assertIs(self.view.nodes["/World/Deep/Child"], sibling)
        self.assertEqual(len(self.view.nodes), 3)

    def test_remapping_the_same_move_twice_is_a_no_op(self):
        """The notice and the authoring site both call it; whichever runs
        second must not double-apply."""
        self._rename("/World/Deep/Node", "Renamed")
        before = dict(self.view.nodes)

        applied = self.view._remapMovedNodes(
            {"/World/Deep/Node": "/World/Deep/Renamed"}
        )

        self.assertEqual(applied, {})
        self.assertEqual(self.view.nodes, before)

    def test_the_node_order_is_preserved(self):
        """Dict order is draw and iteration order; a pop-and-re-insert would
        send every renamed node to the back of the graph."""
        first = self._addNode("/World/Deep/NodeTwo")
        last = self._addNode("/World/Deep/Child")
        before = list(self.view.nodes)

        self._rename("/World/Deep/NodeTwo", "Renamed")

        expected = ["/World/Deep/Renamed" if n == "/World/Deep/NodeTwo" else n
                    for n in before]
        self.assertEqual(list(self.view.nodes), expected)
        self.assertIs(self.view.nodes["/World/Deep/Renamed"], first)
        self.assertIs(self.view.nodes["/World/Deep/Child"], last)

    def test_a_synthesized_io_node_keeps_its_pin_name(self):
        """Blueprint I/O nodes borrow the blueprint's path for their id but are
        not prim-backed; only the id moves."""
        io = self.NodeModel()
        io.id = "/World/Deep#input:drive:x"
        io.name = "x"
        self.view.nodes[io.id] = io

        self.view._remapMovedNodes({"/World/Deep": "/World/Deeper"})

        self.assertIn("/World/Deeper#input:drive:x", self.view.nodes)
        self.assertEqual(self.view.nodes["/World/Deeper#input:drive:x"].name, "x")


@unittest.skipUnless(
    _has_graph_view and _has_prim_rename, "graphView module not available"
)
class RenameFromOutsideTheEditorTest(unittest.TestCase):
    """A rename authored anywhere on the stage, arriving only as a notice.

    usdview, a script, or a second editor window can rename a prim without
    going through _commitRename. USD classifies the resync pair for us
    (RenameSource / RenameDestination), so the same re-key applies.
    """

    def setUp(self):
        from UsdNoodles.models import NodeModel
        from UsdNoodles.usdNoticeHandler import UsdNoticeHandler

        self.stage = _curatedStage()
        self.view = _StubGraphView(self.stage)
        node = NodeModel(stage=self.stage, primPath=Sdf.Path("/World/Deep/Node"))
        node.setDisplayPosition(Gf.Vec2d(500.0, 300.0))
        self.view.nodes["/World/Deep/Node"] = node
        self.node = node

        self.handler = UsdNoticeHandler(self.view)
        self.handler.register(self.stage)
        self.addCleanup(self.handler.unregister)

    def _renameOnTheStage(self, path, newName):
        options = Usd.NamespaceEditor.EditOptions()
        options.allowRelocatesAuthoring = False
        editor = Usd.NamespaceEditor(self.stage, options)
        editor.RenamePrim(self.stage.GetPrimAtPath(path), newName)
        self.assertTrue(editor.ApplyEdits())

    def test_usd_reports_the_pair_as_a_rename(self):
        """The whole design rests on this classification being available."""
        self._renameOnTheStage("/World/Deep/Node", "Elsewhere")

        self.assertIn("/World/Deep/Elsewhere", self.view.nodes)

    def test_the_node_is_not_dropped_as_stale(self):
        self._renameOnTheStage("/World/Deep/Node", "Elsewhere")

        self.assertIs(self.view.nodes["/World/Deep/Elsewhere"], self.node)
        self.assertEqual(len(self.view.nodes), 1)

    def test_the_display_position_survives_the_resync(self):
        """_handleUsdChanges invalidates the node's cache right after the
        re-key; a display-only position has nothing in USD to come back from."""
        self._renameOnTheStage("/World/Deep/Node", "Elsewhere")

        self.assertEqual(
            tuple(self.view.nodes["/World/Deep/Elsewhere"].position), (500.0, 300.0)
        )

    def test_an_unrelated_delete_still_drops_its_node(self):
        """The rename handling must not blunt ordinary reconciliation."""
        from UsdNoodles.models import NodeModel

        self.view.nodes["/Other"] = NodeModel(
            stage=self.stage, primPath=Sdf.Path("/Other")
        )

        self.stage.RemovePrim("/Other")

        self.assertNotIn("/Other", self.view.nodes)


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class ResyncStillReachesTheOwningNodeTest(unittest.TestCase):
    """A resynced path that is not itself a node id still invalidates its node.

    USD names properties in the resynced set -- creating or removing an
    attribute resyncs ``/Prim.attrName`` -- and that is the signal that puts a
    new pin on the node. The boundary-safe prefix test that stops
    ``/World/Cube`` matching ``/World/CubeTwo`` has to keep letting this
    through, so it treats ``.`` as a separator alongside ``/``.
    """

    def setUp(self):
        from UsdNoodles.models import NodeModel
        from UsdNoodles.usdNoticeHandler import UsdNoticeHandler

        self.stage = _curatedStage()
        self.view = _StubGraphView(self.stage)
        self.node = NodeModel(stage=self.stage, primPath=Sdf.Path("/World/Deep/Node"))
        self.view.nodes["/World/Deep/Node"] = self.node
        # A sibling whose name starts the same way, to catch over-matching.
        self.sibling = NodeModel(
            stage=self.stage, primPath=Sdf.Path("/World/Deep/NodeTwo")
        )
        self.view.nodes["/World/Deep/NodeTwo"] = self.sibling

        handler = UsdNoticeHandler(self.view)
        handler.register(self.stage)
        self.addCleanup(handler.unregister)

    def test_a_new_attribute_shows_up_as_a_pin(self):
        """The resynced path is /World/Deep/Node.drive, not the node id."""
        self.assertNotIn("drive", self.node.inputPins)

        self.stage.GetPrimAtPath("/World/Deep/Node").CreateAttribute(
            "drive", Sdf.ValueTypeNames.Float
        )

        self.assertIn("drive", self.node.inputPins)

    def test_an_externally_moved_node_picks_up_its_new_position(self):
        """The cache the position getter reads is NodeModel._position. The
        handler used to null a name NodeModel does not have, so authoring
        ui:nodegraph:node:pos from outside the editor never moved the node."""
        prim = self.stage.GetPrimAtPath("/World/Deep/Node")
        attr = prim.CreateAttribute(
            "ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2
        )
        attr.Set(Gf.Vec2f(1.0, 1.0))
        self.assertEqual(tuple(self.node.position), (1000.0, 1000.0))

        attr.Set(Gf.Vec2f(4.0, 5.0))

        self.assertEqual(tuple(self.node.position), (4000.0, 5000.0))

    def test_the_similarly_named_sibling_is_not_invalidated(self):
        """/World/Deep/Node is a string prefix of /World/Deep/NodeTwo but not
        an ancestor of it, so a resync of the first must leave the second's
        caches alone.

        The resynced path is deliberately NOT a node here -- when it is, the
        loop matches it exactly and never reaches the ancestor scan that this
        is about.
        """
        del self.view.nodes["/World/Deep/Node"]
        self.assertEqual(self.sibling.name, "NodeTwo")

        self.stage.RemovePrim("/World/Deep/Node")

        # A populated _name is the observable trace of "was not invalidated".
        self.assertEqual(self.sibling._name, "NodeTwo")


@unittest.skipUnless(
    _has_graph_view and _has_prim_rename, "graphView module not available"
)
class RenameUndoTest(unittest.TestCase):
    """The undo/redo pair a commit pushes."""

    def setUp(self):
        self.stage = _wiredStage()
        self.gv = MagicMock()
        self.gv.nodeGraph.getStage.return_value = self.stage

    def test_undo_puts_the_name_and_the_wiring_back(self):
        from UsdNoodles.graphView import _make_rename_undo

        before = self.stage.GetRootLayer().ExportToString()
        status, newPath, message = rename_prim(
            self.stage, self.stage.GetPrimAtPath("/World/Old"), "New"
        )
        self.assertEqual(status, RENAMED, message)

        undo = _make_rename_undo(self.gv, "/World", "New", "Old")
        undo()

        # Not just the name: the relationship targets and connections the
        # rename retargeted have to come back with it.
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)

    def test_redo_reapplies_it(self):
        from UsdNoodles.graphView import _make_rename_undo

        rename_prim(self.stage, self.stage.GetPrimAtPath("/World/Old"), "New")
        after = self.stage.GetRootLayer().ExportToString()

        _make_rename_undo(self.gv, "/World", "New", "Old")()
        _make_rename_undo(self.gv, "/World", "Old", "New")()

        self.assertEqual(self.stage.GetRootLayer().ExportToString(), after)

    def test_a_missing_prim_is_warned_about_not_raised(self):
        from UsdNoodles.graphView import _make_rename_undo

        _make_rename_undo(self.gv, "/World", "NotThere", "Whatever")()

    def test_the_undo_does_not_reload_the_graph(self):
        """Ctrl+Z after a rename is the same bug wearing a different hat: the
        undo callback used to answer by re-running a loader."""
        from UsdNoodles.graphView import _make_rename_undo

        rename_prim(self.stage, self.stage.GetPrimAtPath("/World/Old"), "New")

        _make_rename_undo(self.gv, "/World", "New", "Old")()

        self.gv._reloadCurrentGraph.assert_not_called()
        self.gv._remapMovedNodes.assert_called_once_with(
            {"/World/New": "/World/Old"}
        )


@unittest.skipUnless(
    _has_graph_view and _has_prim_rename, "graphView module not available"
)
class RenameUndoKeepsTheGraphTest(unittest.TestCase):
    """Undo and redo of a rename move the node, like the rename itself did.

    Driven against a real view stub rather than a bare MagicMock, so the
    re-keying actually happens and can be asserted on.
    """

    def setUp(self):
        from UsdNoodles.models import NodeModel

        self.stage = _curatedStage()
        self.view = _StubGraphView(self.stage)
        self.node = NodeModel(stage=self.stage, primPath=Sdf.Path("/World/Deep/Node"))
        self.node.setDisplayPosition(Gf.Vec2d(500.0, 300.0))
        self.view.nodes["/World/Deep/Node"] = self.node
        self.other = NodeModel(stage=self.stage, primPath=Sdf.Path("/Other"))
        self.view.nodes["/Other"] = self.other

        self.view._beginRename("/World/Deep/Node")
        self.view._renameInput.text = "Renamed"
        self.view._commitRename()

    def _undo(self, fromName, toName):
        from UsdNoodles.graphView import _make_rename_undo

        _make_rename_undo(self.view, "/World/Deep", fromName, toName)()

    def test_undo_brings_the_node_back_under_the_old_name(self):
        self._undo("Renamed", "Node")

        self.assertEqual(set(self.view.nodes), {"/World/Deep/Node", "/Other"})
        self.assertIs(self.view.nodes["/World/Deep/Node"], self.node)

    def test_redo_takes_it_forward_again(self):
        self._undo("Renamed", "Node")
        self._undo("Node", "Renamed")

        self.assertEqual(set(self.view.nodes), {"/World/Deep/Renamed", "/Other"})
        self.assertIs(self.view.nodes["/World/Deep/Renamed"], self.node)

    def test_the_graph_is_never_reloaded(self):
        self._undo("Renamed", "Node")

        self.view._reloadCurrentGraph.assert_not_called()

    def test_positions_and_camera_survive_the_round_trip(self):
        self._undo("Renamed", "Node")
        self._undo("Node", "Renamed")

        self.assertEqual(
            tuple(self.view.nodes["/World/Deep/Renamed"].position), (500.0, 300.0)
        )
        self.assertEqual(self.view.panX, 137.0)
        self.assertEqual(self.view.panY, -42.0)
        self.assertEqual(self.view.zoom, 0.35)


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class RenameOwnsTheKeyboardTest(unittest.TestCase):
    """While the editor is open, no single-letter shortcut may fire.

    The shortcut table below the rename check is single letters -- Z is undo,
    Y is redo, 1-9 select links. Typing a name containing any of them has to
    reach the text field and nothing else.
    """

    def setUp(self):
        try:
            from PySide6 import QtCore, QtGui
        except ImportError:
            self.skipTest("PySide6 not available")
        self.QtCore, self.QtGui = QtCore, QtGui

        self.stage = _wiredStage()
        self.view = _StubGraphView(self.stage)
        self.view.nodeCreationHotbox = MagicMock()
        self.view.nodeCreationHotbox.is_showing = False
        self.view._performUndo = MagicMock()
        self.view._performRedo = MagicMock()
        self.view.spacePressed = False

    def _key(self, key, text=""):
        return self.QtGui.QKeyEvent(
            self.QtCore.QEvent.KeyPress, key, self.QtCore.Qt.NoModifier, text
        )

    def test_letters_go_to_the_field_not_to_shortcuts(self):
        self.view._beginRename("/World/Old")
        self.view._renameInput.text = ""
        self.view._renameInput.cursor_position = 0

        for ch, key in (("Z", self.QtCore.Qt.Key_Z), ("Y", self.QtCore.Qt.Key_Y)):
            GraphView.keyPressEvent(self.view, self._key(key, ch))

        self.view._performUndo.assert_not_called()
        self.view._performRedo.assert_not_called()
        self.assertEqual(self.view._renameInput.text, "ZY")

    def test_escape_cancels(self):
        self.view._beginRename("/World/Old")

        GraphView.keyPressEvent(self.view, self._key(self.QtCore.Qt.Key_Escape))

        self.assertEqual(self.view._renamingNodeId, "")

    def test_return_commits(self):
        self.view._beginRename("/World/Old")
        self.view._renameInput.text = "New"

        GraphView.keyPressEvent(self.view, self._key(self.QtCore.Qt.Key_Return))

        self.assertEqual(self.view._renamingNodeId, "")
        self.assertTrue(self.stage.GetPrimAtPath("/World/New").IsValid())

    def test_shortcuts_still_work_when_not_renaming(self):
        """The guard must not be a permanent keyboard hijack."""
        self.assertEqual(self.view._renamingNodeId, "")

        GraphView.keyPressEvent(
            self.view,
            self.QtGui.QKeyEvent(
                self.QtCore.QEvent.KeyPress,
                self.QtCore.Qt.Key_Z,
                self.QtCore.Qt.ControlModifier,
                "z",
            ),
        )

        self.view._performUndo.assert_called_once()


class _ReloadStubGraphView:
    """The slice of GraphView that ``_reloadCurrentGraph`` drives.

    The loader runs for real against a real stage, because the whole point of
    the test is what the loader does NOT produce.
    """

    def __init__(self, stage):
        from UsdNoodles import _usdNoodles as _noodles
        from UsdNoodles.nodeFactory import NodeFactory
        from UsdNoodles.nodeGraphStage import NodeGraphStage

        fontMetrics = _noodles.FontMetrics()
        fontMetrics.ascender = 0.8
        fontMetrics.descender = -0.2
        fontMetrics.lineHeight = 1.2
        self.fontAtlas = fontMetrics

        self._nodeFactory = NodeFactory([])
        self.textRenderer = MagicMock()
        self.textRenderer.calculateTextWidth = lambda text, size: len(text) * size * 0.5

        self.nodeGraph = NodeGraphStage()
        self.nodeGraph.setStage(stage)
        self._currentPrimPath = None
        self._nextZOrder = 0

        self._noticeHandler = MagicMock()
        self._clearRenderCache = MagicMock()
        self._nodeRenderManager = MagicMock()
        self._cppIconRenderer = MagicMock()
        self._nodeTransformFrame = MagicMock()
        self.update = MagicMock()

        self.zoom = 0.35
        self.panX = 137.0
        self.panY = -42.0

    @property
    def nodes(self):
        return self.nodeGraph.nodes

    def _addPrimAsNode(self, prim, stage, addedNodeIds):
        return GraphView._addPrimAsNode(self, prim, stage, addedNodeIds)

    def _populateNodeLinksFromPrim(self, node, prim):
        return GraphView._populateNodeLinksFromPrim(self, node, prim)

    def _nodeHasAuthoredPosition(self, node):
        return GraphView._nodeHasAuthoredPosition(self, node)

    def _reloadCurrentGraph(self):
        return GraphView._reloadCurrentGraph(self)


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class ReloadKeepsCuratedNodesTest(unittest.TestCase):
    """A reload refreshes the graph. It does not replace it with the loader's.

    ``_reloadCurrentGraph`` runs after every undo and redo, so this is the
    other half of "a rename does not reset the graph": without it, renaming a
    prim leaves the canvas intact and then Ctrl+Z clears it anyway.

    A stage graph's loader walks the pseudo-root's direct children. The nodes
    the user put on the canvas with 'A' are usually nowhere near there -- in a
    typical rig, one top-level prim and everything else nested under it, which
    is exactly the reported "clears out all prims, adds back the root prim".
    """

    def setUp(self):
        self.stage = _curatedStage()
        self.view = _ReloadStubGraphView(self.stage)
        # What the loader finds on its own.
        self.view.nodeGraph.load(
            self.stage,
            self.view.textRenderer.calculateTextWidth,
            self.view.fontAtlas,
            nodeFactory=self.view._nodeFactory,
        )
        self.loaderNodeIds = set(self.view.nodes)
        # ...and a nested prim the user added, which it cannot find.
        prim = self.stage.GetPrimAtPath("/World/Deep/Node")
        self.view._addPrimAsNode(prim, self.stage, [])
        self.view.nodes["/World/Deep/Node"].setDisplayPosition(Gf.Vec2d(500.0, 300.0))

    def test_the_loader_alone_would_lose_the_nested_node(self):
        """The premise. If this ever stops holding, the test below is vacuous."""
        self.assertNotIn("/World/Deep/Node", self.loaderNodeIds)

    def test_a_curated_node_survives_the_reload(self):
        self.view._reloadCurrentGraph()

        self.assertIn("/World/Deep/Node", self.view.nodes)

    def test_the_nodes_the_loader_does_find_are_still_there(self):
        self.view._reloadCurrentGraph()

        self.assertTrue(self.loaderNodeIds.issubset(set(self.view.nodes)))

    def test_a_display_only_position_survives_the_reload(self):
        """The loader has no ui:nodegraph:node:pos to read, so a re-added node
        would otherwise come back stacked on the origin."""
        self.view._reloadCurrentGraph()

        self.assertEqual(
            tuple(self.view.nodes["/World/Deep/Node"].position), (500.0, 300.0)
        )

    def test_the_camera_does_not_move(self):
        self.view._reloadCurrentGraph()

        self.assertEqual(self.view.panX, 137.0)
        self.assertEqual(self.view.panY, -42.0)
        self.assertEqual(self.view.zoom, 0.35)

    def test_a_prim_that_really_went_away_is_not_resurrected(self):
        """Undo of a create deactivates the prim; the loaders skip inactive
        children and so must this."""
        self.stage.GetPrimAtPath("/World/Deep/Node").SetActive(False)

        self.view._reloadCurrentGraph()

        self.assertNotIn("/World/Deep/Node", self.view.nodes)

    def test_a_deleted_prim_is_not_resurrected(self):
        self.stage.RemovePrim("/World/Deep/Node")

        self.view._reloadCurrentGraph()

        self.assertNotIn("/World/Deep/Node", self.view.nodes)

    def test_an_authored_position_is_left_to_usd(self):
        """A reload can be undoing a move, so re-applying the position the node
        had a moment ago would defeat it."""
        from pxr import Sdf

        prim = self.stage.GetPrimAtPath("/World/Deep/Node")
        prim.CreateAttribute("ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2).Set(
            Gf.Vec2f(7.0, 8.0)
        )

        self.view._reloadCurrentGraph()

        self.assertEqual(
            tuple(self.view.nodes["/World/Deep/Node"].position), (7000.0, 8000.0)
        )


if __name__ == "__main__":
    unittest.main()
