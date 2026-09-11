#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
Tests for what the node creation hotbox targets.

Creating a prim parents it under the prim selected in usdview, and choosing an
API schema entry applies that schema to the selected prims instead of creating
anything. Both resolve the selection the same way, so these cover the shared
resolution as well as the two entry points.

GraphView needs a GL context to construct, so these call its methods against a
stub holding only the attributes the methods touch.
"""

import unittest
from unittest.mock import MagicMock, patch

from pxr import Sdf, Usd, UsdGeom

try:
    from UsdNoodles.apiSchemas import API_SCHEMA_KIND
    from UsdNoodles.graphView import GraphView

    _has_graph_view = True
except ImportError:
    _has_graph_view = False


class _StubGraphView:
    """The slice of GraphView state the hotbox target resolution reads."""

    def __init__(
        self,
        stage,
        livePrims=(),
        cachedPrims=(),
        focusPrim=None,
        currentPrimPath=None,
    ):
        selection = MagicMock()
        selection.getPrims.return_value = list(livePrims)
        selection.getFocusPrim.return_value = focusPrim

        self._usdviewApi = MagicMock()
        self._usdviewApi.stage = stage
        self._usdviewApi.dataModel.selection = selection

        self._lastPrimTreeSelection = list(cachedPrims)
        self._currentPrimPath = currentPrimPath

        self._noticeHandler = MagicMock()
        self.nodeGraph = MagicMock()
        self.nodeGraph.getStage.return_value = stage

        self._reloadCurrentGraph = MagicMock()
        self._showPopupMessage = MagicMock()
        self._showWarningPopup = MagicMock()

    # Bind the real implementations under test onto the stub.
    def _resolveSelectedPrims(self):
        return GraphView._resolveSelectedPrims(self)

    def _resolveSelectedParentPrim(self):
        return GraphView._resolveSelectedParentPrim(self)

    def _resolveParentPath(self, stage):
        return GraphView._resolveParentPath(self, stage)

    def _showApiSchemaResult(self, identifier, applied, alreadyPresent, skipped):
        return GraphView._showApiSchemaResult(
            self, identifier, applied, alreadyPresent, skipped
        )


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class ResolveSelectedPrimsTest(unittest.TestCase):
    """Which prims count as "the selection"."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        self.root = self.stage.GetPseudoRoot()
        self.alpha = UsdGeom.Xform.Define(self.stage, "/Alpha").GetPrim()
        self.beta = UsdGeom.Xform.Define(self.stage, "/Beta").GetPrim()

    def _paths(self, prims):
        return [str(prim.GetPath()) for prim in prims]

    def test_live_selection_is_used(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha, self.beta])

        self.assertEqual(self._paths(view._resolveSelectedPrims()), ["/Alpha", "/Beta"])

    def test_pseudo_root_is_dropped(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.root, self.alpha])

        self.assertEqual(self._paths(view._resolveSelectedPrims()), ["/Alpha"])

    def test_duplicates_are_dropped(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha, self.alpha])

        self.assertEqual(self._paths(view._resolveSelectedPrims()), ["/Alpha"])

    def test_cache_covers_a_canvas_click_resetting_the_selection(self) -> None:
        """Clicking the Noodles canvas leaves usdview reporting only the root."""
        view = _StubGraphView(
            self.stage, livePrims=[self.root], cachedPrims=[self.alpha]
        )

        self.assertEqual(self._paths(view._resolveSelectedPrims()), ["/Alpha"])

    def test_richer_cached_selection_wins(self) -> None:
        view = _StubGraphView(
            self.stage,
            livePrims=[self.alpha],
            cachedPrims=[self.alpha, self.beta],
        )

        self.assertEqual(self._paths(view._resolveSelectedPrims()), ["/Alpha", "/Beta"])

    def test_stale_cache_does_not_override_a_richer_live_selection(self) -> None:
        view = _StubGraphView(
            self.stage,
            livePrims=[self.alpha, self.beta],
            cachedPrims=[self.alpha],
        )

        self.assertEqual(self._paths(view._resolveSelectedPrims()), ["/Alpha", "/Beta"])

    def test_nothing_selected_yields_nothing(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.root])

        self.assertEqual(view._resolveSelectedPrims(), [])

    def test_no_usdview_api_yields_nothing(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha])
        view._usdviewApi = None

        self.assertEqual(view._resolveSelectedPrims(), [])


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class ResolveParentPathTest(unittest.TestCase):
    """Where a newly created prim is parented."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        self.alpha = UsdGeom.Xform.Define(self.stage, "/Alpha").GetPrim()
        self.beta = UsdGeom.Xform.Define(self.stage, "/Beta").GetPrim()

    def test_selected_prim_becomes_the_parent(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha])

        self.assertEqual(GraphView._resolveParentPath(view, self.stage), Sdf.Path("/Alpha"))

    def test_focus_prim_wins_over_the_rest_of_the_selection(self) -> None:
        view = _StubGraphView(
            self.stage, livePrims=[self.alpha, self.beta], focusPrim=self.alpha
        )

        self.assertEqual(GraphView._resolveParentPath(view, self.stage), Sdf.Path("/Alpha"))

    def test_last_selected_prim_is_used_without_a_usable_focus(self) -> None:
        """usdview reports the pseudo-root as focus after a canvas click."""
        view = _StubGraphView(
            self.stage,
            livePrims=[self.alpha, self.beta],
            focusPrim=self.stage.GetPseudoRoot(),
        )

        self.assertEqual(GraphView._resolveParentPath(view, self.stage), Sdf.Path("/Beta"))

    def test_selection_beats_the_displayed_container(self) -> None:
        view = _StubGraphView(
            self.stage, livePrims=[self.alpha], currentPrimPath="/Beta"
        )

        self.assertEqual(GraphView._resolveParentPath(view, self.stage), Sdf.Path("/Alpha"))

    def test_displayed_container_is_used_when_nothing_is_selected(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[], currentPrimPath="/Beta")

        self.assertEqual(GraphView._resolveParentPath(view, self.stage), "/Beta")

    def test_falls_back_to_the_stage_when_nothing_is_selected_or_displayed(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[])

        self.assertEqual(GraphView._resolveParentPath(view, self.stage), Sdf.Path("/"))


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class ApplyApiSchemaFromHotboxTest(unittest.TestCase):
    """Choosing an API schema entry applies it to the selected prims."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        self.alpha = UsdGeom.Xform.Define(self.stage, "/Alpha").GetPrim()
        self.beta = UsdGeom.Xform.Define(self.stage, "/Beta").GetPrim()
        self.scope = self.stage.DefinePrim("/Scope", "Scope")

    def _apply(self, view, identifier):
        """Run the apply path, capturing the undo/redo pair instead of pushing it."""
        with patch(
            "UsdNoodles.graphView._push_undo_command"
        ) as pushUndoCommand:
            GraphView._applyApiSchemaFromHotbox(view, identifier)
        return pushUndoCommand

    def test_applies_to_every_selected_prim(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha, self.beta])

        self._apply(view, "PhysicsRigidBodyAPI")

        self.assertIn("PhysicsRigidBodyAPI", self.alpha.GetAppliedSchemas())
        self.assertIn("PhysicsRigidBodyAPI", self.beta.GetAppliedSchemas())
        view._showPopupMessage.assert_called_once_with(
            "Applied PhysicsRigidBodyAPI to 2 prims"
        )

    def test_multiple_apply_schema_gets_an_instance_name_per_prim(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha, self.beta])

        self._apply(view, "CollectionAPI")

        self.assertEqual(
            list(self.alpha.GetAppliedSchemas()), ["CollectionAPI:collection"]
        )
        self.assertEqual(
            list(self.beta.GetAppliedSchemas()), ["CollectionAPI:collection"]
        )

    def test_disallowed_prims_are_skipped_and_reported(self) -> None:
        """VisibilityAPI is Imageable-only, so it must skip a non-Imageable prim."""
        renderPass = self.stage.DefinePrim("/Pass", "RenderPass")
        view = _StubGraphView(self.stage, livePrims=[renderPass, self.alpha])

        self._apply(view, "HydraRenderPassAPI")

        self.assertIn("HydraRenderPassAPI", renderPass.GetAppliedSchemas())
        self.assertNotIn("HydraRenderPassAPI", self.alpha.GetAppliedSchemas())
        message = view._showWarningPopup.call_args[0][0]
        self.assertIn("Applied HydraRenderPassAPI to 1 prim", message)
        self.assertIn("skipped 1: Alpha", message)

    def test_nothing_is_authored_when_every_prim_is_disallowed(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha])

        self._apply(view, "HydraRenderPassAPI")

        self.assertEqual(list(self.alpha.GetAppliedSchemas()), [])
        self.assertIn(
            "Could not apply HydraRenderPassAPI",
            view._showWarningPopup.call_args[0][0],
        )
        view._reloadCurrentGraph.assert_not_called()

    def test_empty_selection_is_reported_without_authoring(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[])

        self._apply(view, "PhysicsRigidBodyAPI")

        view._showWarningPopup.assert_called_once_with(
            "Select a prim to apply PhysicsRigidBodyAPI"
        )
        view._reloadCurrentGraph.assert_not_called()

    def test_graph_is_rebuilt_so_new_pins_appear(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha])

        self._apply(view, "PhysicsRigidBodyAPI")

        view._reloadCurrentGraph.assert_called_once()

    def test_plugin_edits_are_not_reprocessed_as_external_changes(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha])

        self._apply(view, "PhysicsRigidBodyAPI")

        self.assertEqual(
            [call.args[0] for call in view._noticeHandler.setEnabled.call_args_list],
            [False, True],
        )

    def test_undo_removes_the_schema_and_redo_restores_it(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha])

        pushUndoCommand = self._apply(view, "CollectionAPI")

        description, redo, undo = pushUndoCommand.call_args[0]
        self.assertEqual(description, "Apply CollectionAPI")

        undo()
        self.assertEqual(list(self.alpha.GetAppliedSchemas()), [])

        redo()
        self.assertEqual(
            list(self.alpha.GetAppliedSchemas()), ["CollectionAPI:collection"]
        )

    def test_reapplying_an_existing_schema_is_not_undoable(self) -> None:
        """Undo must not strip a schema this invocation did not author.

        ApplyAPI is idempotent, so picking a schema the prim already carries
        authors nothing. Recording it as an edit would let a single undo
        delete the user's pre-existing state.
        """
        self.alpha.ApplyAPI("PhysicsRigidBodyAPI")
        view = _StubGraphView(self.stage, livePrims=[self.alpha])

        pushUndoCommand = self._apply(view, "PhysicsRigidBodyAPI")

        pushUndoCommand.assert_not_called()
        self.assertEqual(
            list(self.alpha.GetAppliedSchemas()), ["PhysicsRigidBodyAPI"]
        )

    def test_schema_inherited_from_a_reference_is_left_alone(self) -> None:
        """Undoing a no-op would author a 'delete apiSchemas' override."""
        asset = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(asset, "/Asset").GetPrim().ApplyAPI(
            "PhysicsRigidBodyAPI"
        )
        self.alpha.GetReferences().AddReference(
            asset.GetRootLayer().identifier, "/Asset"
        )
        view = _StubGraphView(self.stage, livePrims=[self.alpha])

        pushUndoCommand = self._apply(view, "PhysicsRigidBodyAPI")

        pushUndoCommand.assert_not_called()
        self.assertNotIn("delete", self.stage.GetRootLayer().ExportToString())
        self.assertEqual(
            list(self.alpha.GetAppliedSchemas()), ["PhysicsRigidBodyAPI"]
        )

    def test_already_present_prims_are_reported_as_such(self) -> None:
        self.alpha.ApplyAPI("PhysicsRigidBodyAPI")
        view = _StubGraphView(self.stage, livePrims=[self.alpha, self.beta])

        self._apply(view, "PhysicsRigidBodyAPI")

        message = view._showPopupMessage.call_args[0][0]
        self.assertIn("Applied PhysicsRigidBodyAPI to 1 prim", message)
        self.assertIn("1 already had it", message)

    def test_undo_of_a_mixed_batch_only_reverses_what_it_authored(self) -> None:
        self.alpha.ApplyAPI("PhysicsRigidBodyAPI")
        view = _StubGraphView(self.stage, livePrims=[self.alpha, self.beta])

        pushUndoCommand = self._apply(view, "PhysicsRigidBodyAPI")

        _, _, undo = pushUndoCommand.call_args[0]
        undo()

        self.assertEqual(
            list(self.alpha.GetAppliedSchemas()), ["PhysicsRigidBodyAPI"]
        )
        self.assertEqual(list(self.beta.GetAppliedSchemas()), [])


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class HotboxDispatchTest(unittest.TestCase):
    """API schema entries must not go down the prim creation path."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        self.alpha = UsdGeom.Xform.Define(self.stage, "/Alpha").GetPrim()

    def test_api_schema_entry_is_applied_not_created(self) -> None:
        view = _StubGraphView(self.stage, livePrims=[self.alpha])
        view._applyApiSchemaFromHotbox = MagicMock()
        view._resolveParentPath = MagicMock()

        GraphView._createNodeFromHotbox(
            view,
            {"identifier": "MotionAPI", "library": None, "kind": API_SCHEMA_KIND},
            (0, 0),
        )

        view._applyApiSchemaFromHotbox.assert_called_once_with("MotionAPI")
        view._resolveParentPath.assert_not_called()

    def test_prim_entry_still_creates_a_prim(self) -> None:
        library = MagicMock()
        library.create_node.return_value = Sdf.Path("/Alpha/Cube")
        view = _StubGraphView(self.stage, livePrims=[self.alpha])
        view._applyApiSchemaFromHotbox = MagicMock()
        view.zoom = 1.0
        view.panX = 0.0
        view.panY = 0.0
        view._onNodeCreated = MagicMock()

        UsdGeom.Cube.Define(self.stage, "/Alpha/Cube")
        GraphView._createNodeFromHotbox(
            view,
            {"identifier": "Cube", "library": library, "kind": "prim"},
            (0, 0),
        )

        view._applyApiSchemaFromHotbox.assert_not_called()
        self.assertEqual(library.create_node.call_args[0][1], Sdf.Path("/Alpha"))


@unittest.skipUnless(_has_graph_view, "graphView module not available")
class ApiSchemaHotboxEntryTest(unittest.TestCase):
    """The entries the catalog contributes to the hotbox list."""

    def setUp(self) -> None:
        self.entries = GraphView._apiSchemaHotboxItems()

    def test_every_entry_is_marked_with_the_api_suffix(self) -> None:
        """The marker is what tells a schema to apply from a prim to create."""
        self.assertTrue(self.entries)
        for entry in self.entries:
            self.assertTrue(
                entry["name"].endswith(" *api"),
                f"{entry['name']!r} is missing the *api marker",
            )

    def test_the_marker_is_display_only(self) -> None:
        """ApplyAPI takes the bare identifier, so the marker must not reach it."""
        for entry in self.entries:
            self.assertNotIn("*", entry["identifier"])
            self.assertEqual(entry["name"], f"{entry['identifier']} *api")

    def test_entries_are_api_schema_kind_with_no_library(self) -> None:
        for entry in self.entries:
            self.assertEqual(entry["kind"], API_SCHEMA_KIND)
            self.assertIsNone(entry["library"])

    def test_the_marker_is_unique_to_api_entries(self) -> None:
        """It doubles as a filter term, so no prim type may contain it.

        Substring filtering means a marker sharing text with a prim type name
        would pull prim rows in alongside the schemas.
        """
        for entry in self.entries:
            self.assertNotIn("*api", entry["identifier"].lower())


if __name__ == "__main__":
    unittest.main()
