#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
Tests for the applied API schema catalog.

Covers discovery of applied API schemas from the plugin registry, instance
name generation for multiple-apply schemas, applying and removing schemas on
real prims, and the shape of the hotbox entries built from the catalog.
"""

import unittest
from unittest.mock import MagicMock

from pxr import Usd, UsdGeom

try:
    from UsdNoodles.apiSchemas import (
        ALREADY_PRESENT,
        API_SCHEMA_KIND,
        APPLIED,
        SKIPPED,
        apply_named,
        apply_to_prim,
        get_api_schemas,
        has_applied_schema,
        is_multiple_apply,
        make_instance_name,
        remove_from_prim,
    )

    _has_api_schemas = True
except ImportError:
    _has_api_schemas = False

try:
    from UsdNoodles.widgets.nodeCreationHotbox import NodeCreationHotbox

    _has_hotbox = True
except ImportError:
    _has_hotbox = False


@unittest.skipUnless(_has_api_schemas, "apiSchemas module not available")
class ApiSchemaCatalogTest(unittest.TestCase):
    """Discovery of applied API schemas from the plugin registry."""

    def test_catalog_is_not_empty(self) -> None:
        self.assertTrue(get_api_schemas())

    def test_catalog_finds_schemas_from_unloaded_libraries(self) -> None:
        """Discovery must not be limited to already-imported schema libraries.

        A Tf.Type walk sees only a handful of schemas in a fresh process, so
        finding schemas from several distinct plugins proves the catalog goes
        through the plugin registry instead.
        """
        plugins = {schema.plugin for schema in get_api_schemas()}
        self.assertIn("usdGeom", plugins)
        self.assertIn("usdPhysics", plugins)
        self.assertGreater(len(plugins), 3)

    def test_catalog_contains_known_schemas(self) -> None:
        identifiers = {schema.identifier for schema in get_api_schemas()}
        self.assertIn("CollectionAPI", identifiers)
        self.assertIn("MaterialBindingAPI", identifiers)
        self.assertIn("PhysicsRigidBodyAPI", identifiers)

    def test_catalog_excludes_non_applied_schemas(self) -> None:
        """ClipsAPI and ModelAPI are API schemas but are never applied."""
        identifiers = {schema.identifier for schema in get_api_schemas()}
        self.assertNotIn("ClipsAPI", identifiers)
        self.assertNotIn("ModelAPI", identifiers)

    def test_catalog_excludes_concrete_prim_types(self) -> None:
        identifiers = {schema.identifier for schema in get_api_schemas()}
        self.assertNotIn("Xform", identifiers)
        self.assertNotIn("Sphere", identifiers)

    def test_multiple_apply_flag_matches_registry(self) -> None:
        byIdentifier = {schema.identifier: schema for schema in get_api_schemas()}
        self.assertTrue(byIdentifier["CollectionAPI"].isMultipleApply)
        self.assertFalse(byIdentifier["MaterialBindingAPI"].isMultipleApply)

    def test_can_only_apply_to_is_reported(self) -> None:
        byIdentifier = {schema.identifier: schema for schema in get_api_schemas()}
        self.assertEqual(byIdentifier["VisibilityAPI"].canOnlyApplyTo, ["Imageable"])
        self.assertEqual(byIdentifier["MaterialBindingAPI"].canOnlyApplyTo, [])

    def test_plugin_groups_are_contiguous(self) -> None:
        """The list widget repeats a group header if a group is not contiguous."""
        plugins = [schema.plugin for schema in get_api_schemas()]
        starts = [
            plugin
            for index, plugin in enumerate(plugins)
            if index == 0 or plugins[index - 1] != plugin
        ]
        self.assertEqual(len(starts), len(set(starts)))

    def test_catalog_is_cached(self) -> None:
        self.assertIs(get_api_schemas(), get_api_schemas())

    def test_is_multiple_apply(self) -> None:
        self.assertTrue(is_multiple_apply("CollectionAPI"))
        self.assertFalse(is_multiple_apply("MaterialBindingAPI"))
        self.assertFalse(is_multiple_apply("NotARealSchemaAPI"))


@unittest.skipUnless(_has_api_schemas, "apiSchemas module not available")
class InstanceNameTest(unittest.TestCase):
    """Instance name generation for multiple-apply schemas."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        self.prim = UsdGeom.Xform.Define(self.stage, "/Xform").GetPrim()

    def test_name_is_derived_from_identifier(self) -> None:
        self.assertEqual(make_instance_name(self.prim, "CollectionAPI"), "collection")
        self.assertEqual(
            make_instance_name(self.prim, "PhysicsDriveAPI"), "physicsDrive"
        )

    def test_name_increments_when_already_applied(self) -> None:
        self.assertEqual(make_instance_name(self.prim, "CollectionAPI"), "collection")
        self.prim.ApplyAPI("CollectionAPI", "collection")
        self.assertEqual(make_instance_name(self.prim, "CollectionAPI"), "collection1")
        self.prim.ApplyAPI("CollectionAPI", "collection1")
        self.assertEqual(make_instance_name(self.prim, "CollectionAPI"), "collection2")

    def test_generated_name_is_allowed_by_the_registry(self) -> None:
        for identifier in ("CollectionAPI", "CoordSysAPI", "PhysicsLimitAPI"):
            name = make_instance_name(self.prim, identifier)
            self.assertTrue(
                Usd.SchemaRegistry.IsAllowedAPISchemaInstanceName(identifier, name),
                f"{identifier}:{name} is not a legal instance name",
            )


@unittest.skipUnless(_has_api_schemas, "apiSchemas module not available")
class ApplyApiSchemaTest(unittest.TestCase):
    """Applying and removing API schemas on real prims."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        self.prim = UsdGeom.Xform.Define(self.stage, "/Xform").GetPrim()

    def test_apply_single_apply_schema(self) -> None:
        status, instanceName, reason = apply_to_prim(self.prim, "MaterialBindingAPI")

        self.assertEqual(status, APPLIED, reason)
        self.assertEqual(instanceName, "")
        self.assertIn("MaterialBindingAPI", self.prim.GetAppliedSchemas())

    def test_apply_multiple_apply_schema_names_each_instance(self) -> None:
        first = apply_to_prim(self.prim, "CollectionAPI")
        second = apply_to_prim(self.prim, "CollectionAPI")

        self.assertEqual((first[0], first[1]), (APPLIED, "collection"))
        self.assertEqual((second[0], second[1]), (APPLIED, "collection1"))
        self.assertEqual(
            list(self.prim.GetAppliedSchemas()),
            ["CollectionAPI:collection", "CollectionAPI:collection1"],
        )

    def test_reapplying_a_single_apply_schema_authors_nothing(self) -> None:
        """ApplyAPI is idempotent, so a second apply must not read as an edit.

        Reporting it as APPLIED would make it undoable, and undoing it would
        strip the schema the prim already had.
        """
        apply_to_prim(self.prim, "MaterialBindingAPI")

        status, _, _ = apply_to_prim(self.prim, "MaterialBindingAPI")

        self.assertEqual(status, ALREADY_PRESENT)

    def test_schema_inherited_from_a_reference_is_already_present(self) -> None:
        asset = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(asset, "/Asset").GetPrim().ApplyAPI("MaterialBindingAPI")
        self.prim.GetReferences().AddReference(
            asset.GetRootLayer().identifier, "/Asset"
        )

        status, _, _ = apply_to_prim(self.prim, "MaterialBindingAPI")

        self.assertEqual(status, ALREADY_PRESENT)

    def test_has_applied_schema_matches_the_instance(self) -> None:
        apply_to_prim(self.prim, "CollectionAPI")

        self.assertTrue(has_applied_schema(self.prim, "CollectionAPI", "collection"))
        self.assertFalse(has_applied_schema(self.prim, "CollectionAPI", "other"))
        self.assertFalse(has_applied_schema(self.prim, "MaterialBindingAPI"))

    def test_apply_authors_the_schema_properties(self) -> None:
        apply_to_prim(self.prim, "CollectionAPI")

        self.assertTrue(self.prim.GetProperty("collection:collection:includes"))

    def test_disallowed_prim_type_is_skipped_with_a_reason(self) -> None:
        """ApplyAPI ignores canOnlyApplyTo, so the caller must gate on it."""
        status, _, reason = apply_to_prim(self.prim, "HydraRenderPassAPI")

        self.assertEqual(status, SKIPPED)
        self.assertIn("RenderPass", reason)
        self.assertNotIn("HydraRenderPassAPI", self.prim.GetAppliedSchemas())

    def test_unknown_schema_is_reported_without_raising(self) -> None:
        status, _, reason = apply_to_prim(self.prim, "TotallyMadeUpAPI")

        self.assertEqual(status, SKIPPED)
        self.assertTrue(reason)
        self.assertNotIn("\n", reason)

    def test_invalid_prim_is_rejected(self) -> None:
        status, _, reason = apply_to_prim(
            self.stage.GetPrimAtPath("/DoesNotExist"), "MaterialBindingAPI"
        )

        self.assertEqual(status, SKIPPED)
        self.assertEqual(reason, "invalid prim")

    def test_remove_undoes_a_single_apply_schema(self) -> None:
        apply_to_prim(self.prim, "MaterialBindingAPI")

        self.assertTrue(remove_from_prim(self.prim, "MaterialBindingAPI"))
        self.assertNotIn("MaterialBindingAPI", self.prim.GetAppliedSchemas())

    def test_remove_undoes_only_the_named_instance(self) -> None:
        apply_to_prim(self.prim, "CollectionAPI")
        apply_to_prim(self.prim, "CollectionAPI")

        self.assertTrue(remove_from_prim(self.prim, "CollectionAPI", "collection1"))
        self.assertEqual(
            list(self.prim.GetAppliedSchemas()), ["CollectionAPI:collection"]
        )

    def test_redo_reuses_the_original_instance_name(self) -> None:
        _, instanceName, _ = apply_to_prim(self.prim, "CollectionAPI")
        remove_from_prim(self.prim, "CollectionAPI", instanceName)

        status, reason = apply_named(self.prim, "CollectionAPI", instanceName)

        self.assertEqual(status, APPLIED, reason)
        self.assertEqual(
            list(self.prim.GetAppliedSchemas()), ["CollectionAPI:collection"]
        )


@unittest.skipUnless(_has_hotbox, "hotbox widget not available (needs PySide6)")
@unittest.skipUnless(_has_api_schemas, "apiSchemas module not available")
class HotboxApiSchemaItemTest(unittest.TestCase):
    """API schema entries carried through the hotbox to the creation callback."""

    @staticmethod
    def _hotbox() -> NodeCreationHotbox:
        """A hotbox with a stub animator, which only drives the fade alpha."""
        return NodeCreationHotbox(MagicMock())

    def test_item_data_carries_the_entry_kind(self) -> None:
        hotbox = self._hotbox()
        hotbox.configure_from_libraries(
            [
                {
                    "name": "Cube",
                    "family": "usdGeom",
                    "identifier": "Cube",
                    "library": "library-sentinel",
                    "kind": "prim",
                },
                {
                    "name": "MotionAPI *api",
                    "family": "API Schemas: usdGeom",
                    "identifier": "MotionAPI",
                    "library": None,
                    "kind": API_SCHEMA_KIND,
                },
            ]
        )

        items = hotbox._selectable_list.items
        self.assertEqual(items[0].data["kind"], "prim")
        self.assertEqual(items[0].data["library"], "library-sentinel")
        self.assertEqual(items[1].data["kind"], API_SCHEMA_KIND)
        self.assertEqual(items[1].data["identifier"], "MotionAPI")
        self.assertIsNone(items[1].data["library"])

    def test_entry_kind_defaults_to_prim(self) -> None:
        """Callers predating the API schema entries omit 'kind' entirely."""
        hotbox = self._hotbox()
        hotbox.configure_from_libraries(
            [
                {
                    "name": "Cube",
                    "family": "usdGeom",
                    "identifier": "Cube",
                    "library": "library-sentinel",
                }
            ]
        )

        self.assertEqual(hotbox._selectable_list.items[0].data["kind"], "prim")

    def test_api_entries_are_marked_and_searchable(self) -> None:
        """The suffix marks the row and doubles as a filter term."""
        hotbox = self._hotbox()
        hotbox.configure_from_libraries(
            [
                {
                    "name": "Cube",
                    "family": "usdGeom",
                    "identifier": "Cube",
                    "library": None,
                    "kind": "prim",
                },
                {
                    "name": "MotionAPI *api",
                    "family": "API Schemas: usdGeom",
                    "identifier": "MotionAPI",
                    "library": None,
                    "kind": API_SCHEMA_KIND,
                },
            ]
        )
        hotbox._selectable_list.set_filter("*api")

        filtered = hotbox._selectable_list.filtered_items
        self.assertEqual([item.name for item in filtered], ["MotionAPI *api"])


if __name__ == "__main__":
    unittest.main()
