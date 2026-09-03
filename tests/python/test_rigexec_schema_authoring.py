"""Contract tests for RigExec's strict Python authoring layers.

The low-level ``rigexec.schema`` facade mirrors the project's codeless OpenUSD
schemas.  The higher-level ``Builder`` API composes those primitives into a rig
and owns dependency wiring.  These tests deliberately inspect the authored USD
properties as well as the returned handles: a call is only correct when the
stage carries the registered type, applied schemas, property metadata, and
relationship targets promised by the API.

The optional command-line argument is the generated rigExecSchema resources
directory.  CTest supplies it; direct runs discover the normal build/source
locations.  It is removed from ``sys.argv`` before ``unittest.main()`` sees it.
"""

from __future__ import annotations

import math
import os
import pathlib
import platform
import sys
import unittest


_THIS_FILE = pathlib.Path(__file__).resolve()
_REPO_ROOT = _THIS_FILE.parents[2]
_WORKSPACE = _REPO_ROOT.parent
_DLL_DIR_HANDLES = []
_AVAR_SCALE_FLOOR = 1e-4


def _take_schema_resources_argument():
    """Remove and return the CTest resources argument, when present."""
    for argument in list(sys.argv[1:]):
        candidate = pathlib.Path(argument)
        if (candidate / "plugInfo.json").is_file():
            sys.argv.remove(argument)
            return candidate.resolve()
    return None


def _setup_environment():
    """Select one coherent USD, native-module, and schema-plugin build."""
    explicit_resources = _take_schema_resources_argument()
    usd_install = pathlib.Path(
        os.environ.get("RIGEXEC_USD_INSTALL") or
        (_WORKSPACE / "usd-install"))

    site_candidates = [usd_install / "Lib" / "site-packages"]
    site_candidates.extend(sorted(usd_install.glob("lib/python*/site-packages")))
    for site_packages in reversed(site_candidates):
        if site_packages.is_dir():
            sys.path.insert(0, str(site_packages))

    if explicit_resources is not None:
        build_root = explicit_resources.parents[2]
    else:
        build_root = pathlib.Path(
            os.environ.get("RIGEXEC_BUILD_DIR") or (_REPO_ROOT / "build"))

    python_root = build_root / "python"
    python_candidates = [python_root]
    if python_root.is_dir():
        python_candidates.extend(
            child for child in sorted(python_root.iterdir())
            if child.is_dir())
    for candidate in reversed(python_candidates):
        if candidate.is_dir():
            sys.path.insert(0, str(candidate))

    resources_candidates = [
        explicit_resources,
        pathlib.Path(os.environ["RIGEXEC_SCHEMA_RESOURCE_DIR"])
        if os.environ.get("RIGEXEC_SCHEMA_RESOURCE_DIR") else None,
        build_root / "usd" / "rigExecSchema" / "resources",
        _REPO_ROOT / "plugin" / "rigExecSchema" / "resources",
    ]
    resources = next(
        (candidate for candidate in resources_candidates
         if candidate is not None and
         (candidate / "plugInfo.json").is_file()),
        None)
    if resources is None:
        raise RuntimeError("no rigExecSchema resources directory was found")
    os.environ["RIGEXEC_SCHEMA_RESOURCE_DIR"] = str(resources.resolve())

    usd_plugins = usd_install / "lib" / "usd"
    if usd_plugins.is_dir() and not os.environ.get("PXR_PLUGINPATH_NAME"):
        os.environ["PXR_PLUGINPATH_NAME"] = str(usd_plugins)

    if platform.system() == "Windows" and hasattr(os, "add_dll_directory"):
        dll_candidates = [
            usd_install / "lib", usd_install / "bin", build_root,
        ]
        if build_root.is_dir():
            dll_candidates.extend(
                child for child in sorted(build_root.iterdir())
                if child.is_dir())
        for candidate in dll_candidates:
            if not candidate.is_dir():
                continue
            try:
                _DLL_DIR_HANDLES.append(os.add_dll_directory(str(candidate)))
            except OSError:
                pass

    return resources


SCHEMA_RESOURCES = _setup_environment()

from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402
import rigexec  # noqa: E402


# Import intentionally has no plugin-registration side effect. Pin the exact
# generated/source resource selected above before exercising raw SchemaPrim.
rigexec.load_schema_plugin(str(SCHEMA_RESOURCES))


# Keep this independent of rigexec.schema.names(): the facade and registered
# schema are two independently checked sides of the contract.  CustomConstraint
# was intentionally removed; it must not be resurrected by stale Python data.
CONCRETE_SCHEMA_TYPES = (
    "RigExecAimConstraint",
    "RigExecBlendInput",
    "RigExecBlendPointFrames",
    "RigExecBlendSample",
    "RigExecBlendShapeMover",
    "RigExecCombineWeight",
    "RigExecControl",
    "RigExecCurveMover",
    "RigExecCurveWeight",
    "RigExecCurvenet",
    "RigExecCurvenetMover",
    "RigExecDynamicWeight",
    "RigExecFkChain",
    "RigExecFloatMathMover",
    "RigExecJoint",
    "RigExecLatticeMover",
    "RigExecMatrixMathMover",
    "RigExecMatrixMover",
    "RigExecParentConstraint",
    "RigExecPlaneWeight",
    "RigExecPositionConstraint",
    "RigExecRibbon",
    "RigExecRoot",
    "RigExecRotationConstraint",
    "RigExecScaleConstraint",
    "RigExecSingleChainIkConstraint",
    "RigExecSmoothMover",
    "RigExecSphereWeight",
    "RigExecStaticWeight",
    "RigExecSurfaceMover",
    "RigExecTwistDistribution",
    "RigExecTwoBoneIk",
    "RigExecVec3fMathMover",
    "RigExecVolumeCorrectMover",
)

MOVER_SCHEMA_TYPES = frozenset(
    schema_type for schema_type in CONCRETE_SCHEMA_TYPES
    if schema_type.endswith("Mover") or schema_type.endswith("Constraint"))


def _targets(relationship):
    return [str(path) for path in relationship.GetTargets()]


def _applied_schema_names(prim):
    return {str(token) for token in prim.GetAppliedSchemas()}


class _ContractTestCase(unittest.TestCase):
    def require_method(self, value, name):
        self.assertTrue(
            hasattr(value, name),
            "%s.%s is required by the Python authoring contract" %
            (type(value).__name__, name))
        return getattr(value, name)

    def assert_api_property(self, property_value):
        self.assertTrue(property_value)
        self.assertFalse(property_value.IsCustom())


class SchemaFacadeTests(_ContractTestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()

    def test_all_concrete_types_define_and_get(self):
        self.assertEqual(len(CONCRETE_SCHEMA_TYPES), 34)
        self.assertEqual(
            set(rigexec.schema.names()), set(CONCRETE_SCHEMA_TYPES))
        self.assertFalse(hasattr(rigexec.schema, "CustomConstraint"))
        self.assertFalse(hasattr(rigexec.schema, "RigExecCustomConstraint"))

        for schema_type in CONCRETE_SCHEMA_TYPES:
            short_name = schema_type.removeprefix("RigExec")
            path = "/Types/" + short_name
            schema_class = getattr(rigexec.schema, short_name)
            with self.subTest(schema_type=schema_type):
                authored = schema_class.define(self.stage, path)
                self.assertTrue(authored.valid)
                self.assertEqual(authored.path, path)
                self.assertEqual(authored.schema_type, schema_type)

                prim = self.stage.GetPrimAtPath(path)
                self.assertTrue(prim)
                self.assertEqual(str(prim.GetTypeName()), schema_type)

                reopened = schema_class.get(self.stage, path)
                self.assertTrue(reopened.valid)
                self.assertEqual(reopened.path, path)
                self.assertTrue(
                    rigexec.schema.define(self.stage, path, schema_type).valid)

                applied = _applied_schema_names(prim)
                if schema_type == "RigExecControl":
                    self.assertIn("RigExecControlAPI", applied)
                    role = prim.GetAttribute("rigExec:channelRole")
                    self.assert_api_property(role)
                    self.assertEqual(role.GetTypeName(), Sdf.ValueTypeNames.Token)
                    self.assertEqual(role.GetVariability(), Sdf.VariabilityUniform)
                if schema_type in MOVER_SCHEMA_TYPES:
                    self.assertIn("RigExecMoverAPI", applied)
                    self.assert_api_property(
                        prim.GetRelationship("rigExec:moves"))
                    enabled = prim.GetAttribute("inputs:enabled")
                    self.assert_api_property(enabled)
                    self.assertEqual(enabled.GetTypeName(), Sdf.ValueTypeNames.Bool)
                    default_weight = prim.GetAttribute("inputs:defaultWeight")
                    self.assert_api_property(default_weight)
                    self.assertEqual(
                        default_weight.GetTypeName(), Sdf.ValueTypeNames.Float)
                    self.assertEqual(default_weight.Get(), 1.0)
                    self.assert_api_property(
                        prim.GetRelationship("rigExec:weightObject"))
                if schema_type != "RigExecRoot":
                    self.assertIn("NodeGraphNodeAPI", applied)

        with self.assertRaises(ValueError):
            rigexec.schema.Root.get(self.stage, "/Types/Control")

    def test_explicit_api_helpers_apply_and_author_non_custom_properties(self):
        control = rigexec.SchemaPrim.define(
            self.stage, "/Control", "RigExecControl")
        self.assertFalse(control.has_api("RigExecControlAPI"))
        result = rigexec.ControlAPI.apply(control, channel_role="tweak")
        self.assertIs(result, control)
        self.assertTrue(control.has_api("RigExecControlAPI"))
        control_prim = self.stage.GetPrimAtPath("/Control")
        role = control_prim.GetAttribute("rigExec:channelRole")
        self.assert_api_property(role)
        self.assertEqual(str(role.Get()), "tweak")

        mover = rigexec.SchemaPrim.define(
            self.stage, "/Mover", "RigExecMatrixMover")
        result = rigexec.MoverAPI.apply(
            mover, moves=["/Geometry.points"], enabled=False)
        self.assertIs(result, mover)
        self.assertTrue(mover.has_api("RigExecMoverAPI"))
        mover_prim = self.stage.GetPrimAtPath("/Mover")
        moves = mover_prim.GetRelationship("rigExec:moves")
        enabled = mover_prim.GetAttribute("inputs:enabled")
        self.assert_api_property(moves)
        self.assert_api_property(enabled)
        self.assertEqual(_targets(moves), ["/Geometry.points"])
        self.assertIs(enabled.Get(), False)

        joint = rigexec.schema.Joint.define(self.stage, "/Joint")
        with self.assertRaises(TypeError):
            rigexec.ControlAPI.apply(joint)
        with self.assertRaises(TypeError):
            rigexec.MoverAPI.apply(joint, moves=["/Geometry.points"])

        untouched_control = rigexec.SchemaPrim.define(
            self.stage, Sdf.Path("/UntouchedControl"), "RigExecControl")
        with self.assertRaises(TypeError):
            rigexec.ControlAPI.apply(untouched_control, channel_role=12)
        self.assertFalse(untouched_control.has_api("RigExecControlAPI"))

        untouched_mover = rigexec.SchemaPrim.define(
            self.stage, Sdf.Path("/UntouchedMover"), "RigExecMatrixMover")
        with self.assertRaises(TypeError):
            rigexec.MoverAPI.apply(
                untouched_mover, moves=["/Geometry.points"], enabled="yes")
        self.assertFalse(untouched_mover.has_api("RigExecMoverAPI"))

        single_target_mover = rigexec.SchemaPrim.define(
            self.stage, "/SingleTargetMover", "RigExecMatrixMover")
        rigexec.MoverAPI.apply(
            single_target_mover, moves="/Geometry.points")
        self.assertEqual(
            _targets(self.stage.GetPrimAtPath(
                "/SingleTargetMover").GetRelationship("rigExec:moves")),
            ["/Geometry.points"])

        foreign_stage = Usd.Stage.CreateInMemory()
        foreign_control = rigexec.Builder.create(
            foreign_stage, "/ForeignRig").add_control("Control")
        foreign_target_mover = rigexec.SchemaPrim.define(
            self.stage, "/ForeignTargetMover", "RigExecMatrixMover")
        with self.assertRaises(ValueError):
            rigexec.MoverAPI.apply(
                foreign_target_mover, moves=[foreign_control])
        self.assertFalse(foreign_target_mover.has_api("RigExecMoverAPI"))

    def test_undeclared_properties_are_rejected_without_specs(self):
        root = rigexec.schema.Root.define(self.stage, "/Rig")
        prim = self.stage.GetPrimAtPath("/Rig")

        root.set_attribute("rigExec:partition", "Character")
        partition = prim.GetAttribute("rigExec:partition")
        with self.assertRaises((TypeError, ValueError)):
            root.set_attribute(
                "rigExec:partition", "Animated", time=Usd.TimeCode(12))
        self.assertEqual(str(partition.Get()), "Character")
        self.assertEqual(partition.GetTimeSamples(), [])

        with self.assertRaises((KeyError, ValueError)):
            root.set_attribute("rigExec:notDeclared", 3.0)
        self.assertFalse(prim.HasProperty("rigExec:notDeclared"))

        with self.assertRaises(ValueError):
            root.set_relationship("rigExec:notDeclared", ["/Target"])
        self.assertFalse(prim.HasProperty("rigExec:notDeclared"))

        with self.assertRaises(ValueError):
            root.set_relationship("rigExec:partition", ["/Target"])
        self.assertEqual(str(partition.Get()), "Character")

        curvenet = rigexec.schema.Curvenet.define(
            self.stage, "/CurvenetWithoutDeadGuides")
        curvenet_prim = self.stage.GetPrimAtPath(
            "/CurvenetWithoutDeadGuides")
        for name, value in (
                ("guide:displayColor", [0.9, 0.4, 1.0]),
                ("guide:displayOpacity", 1.0),
                ("guide:radius", 0.1)):
            with self.subTest(curvenet_property=name):
                self.assertFalse(curvenet_prim.HasProperty(name))
                with self.assertRaises((KeyError, ValueError)):
                    curvenet.set_attribute(name, value)
                self.assertFalse(curvenet_prim.HasProperty(name))

        mover = rigexec.schema.MatrixMover.define(self.stage, "/Mover")
        enabled = self.stage.GetPrimAtPath("/Mover").GetAttribute(
            "inputs:enabled")
        with self.assertRaises(TypeError):
            mover.set_attribute("inputs:enabled", 1)
        self.assertFalse(enabled.HasAuthoredValueOpinion())
        with self.assertRaises((KeyError, ValueError)):
            mover.set_attribute("rigExec:moves", "wrong property kind")
        moves = self.stage.GetPrimAtPath("/Mover").GetRelationship(
            "rigExec:moves")
        self.assertFalse(moves.HasAuthoredTargets())

    def test_values_are_converted_from_schema_types(self):
        aim = rigexec.schema.AimConstraint.define(self.stage, "/Aim")
        with self.assertRaises(TypeError):
            aim.set_attribute("rigExec:preserve", "origin")
        aim.set_attribute("rigExec:preserve", ["origin", "scale"])
        preserve = self.stage.GetPrimAtPath("/Aim").GetAttribute(
            "rigExec:preserve")
        self.assertEqual(preserve.GetTypeName(), Sdf.ValueTypeNames.TokenArray)
        self.assertFalse(preserve.IsCustom())
        self.assertEqual([str(value) for value in preserve.Get()], [
            "origin", "scale"])

        control = rigexec.schema.Control.define(self.stage, "/Control")
        control.set_attribute("guide:displayColor", [0.2, 0.4, 0.8])
        display_color = self.stage.GetPrimAtPath("/Control").GetAttribute(
            "guide:displayColor")
        self.assertEqual(
            display_color.GetTypeName(), Sdf.ValueTypeNames.Color3f)
        self.assertFalse(display_color.IsCustom())
        self.assertEqual(display_color.Get(), Gf.Vec3f(0.2, 0.4, 0.8))

        curvenet = rigexec.schema.Curvenet.define(self.stage, "/Curvenet")
        identifiers = [2**40, -(2**40)]
        curvenet.set_attribute("ids", identifiers)
        ids = self.stage.GetPrimAtPath("/Curvenet").GetAttribute("ids")
        self.assertEqual(ids.GetTypeName(), Sdf.ValueTypeNames.Int64Array)
        self.assertFalse(ids.IsCustom())
        self.assertEqual(list(ids.Get()), identifiers)

    def test_control_and_joint_avar_scale_schema_contract(self):
        for schema_class, path in (
                (rigexec.schema.Control, "/ScaleControl"),
                (rigexec.schema.Joint, "/ScaleJoint")):
            with self.subTest(path=path):
                authored = schema_class.define(self.stage, path)
                prim = self.stage.GetPrimAtPath(path)
                for name in ("avars:sx", "avars:sy", "avars:sz"):
                    attr = prim.GetAttribute(name)
                    self.assertTrue(attr)
                    self.assertFalse(attr.IsCustom())
                    self.assertEqual(
                        attr.GetTypeName(), Sdf.ValueTypeNames.Double)
                    self.assertEqual(attr.Get(), 1.0)
                    self.assertFalse(attr.HasAuthoredValueOpinion())

                authored.set_attribute("avars:sy", 3.0)
                sy = prim.GetAttribute("avars:sy")
                with self.assertRaises((TypeError, ValueError, RuntimeError)):
                    authored.set_attribute("avars:sy", "not a double")
                self.assertEqual(sy.Get(), 3.0)

                authored.set_attribute("avars:sx", 0.0)
                authored.set_attribute("avars:sy", -0.0)
                authored.set_attribute(
                    "avars:sz", -0.5 * _AVAR_SCALE_FLOOR)
                self.assertEqual(
                    tuple(prim.GetAttribute(name).Get() for name in (
                        "avars:sx", "avars:sy", "avars:sz")),
                    (_AVAR_SCALE_FLOOR, -_AVAR_SCALE_FLOOR,
                     -_AVAR_SCALE_FLOOR))

                for non_finite in (math.nan, math.inf, -math.inf):
                    with self.assertRaises(ValueError):
                        authored.set_attribute("avars:sy", non_finite)
                    self.assertEqual(sy.Get(), -_AVAR_SCALE_FLOOR)

    def test_blend_data_types_get_node_graph_api(self):
        for schema_class, path in (
                (rigexec.schema.BlendInput, "/Input"),
                (rigexec.schema.BlendSample, "/Sample")):
            with self.subTest(path=path):
                schema_class.define(self.stage, path)
                prim = self.stage.GetPrimAtPath(path)
                self.assertIn("NodeGraphNodeAPI", _applied_schema_names(prim))

    def test_canonical_read_phase_metadata_on_attribute_and_relationship(self):
        mover = rigexec.schema.MatrixMover.define(self.stage, "/Mover")
        mover.set_read_phase("rigExec:transform", "preceding")
        mover.set_read_phase(
            "rigExec:transformReadPhase", "/Rig/Movers/Previous")

        prim = self.stage.GetPrimAtPath("/Mover")
        transform = prim.GetRelationship("rigExec:transform")
        legacy_attribute = prim.GetAttribute("rigExec:transformReadPhase")
        self.assertEqual(
            transform.GetMetadata("rigExecReadPhase"), "preceding")
        self.assertEqual(
            legacy_attribute.GetMetadata("rigExecReadPhase"),
            "/Rig/Movers/Previous")

        with self.assertRaises(ValueError):
            mover.set_read_phase("rigExec:notDeclared", "base")
        self.assertFalse(prim.HasProperty("rigExec:notDeclared"))

    def test_define_is_atomic_when_a_standard_api_cannot_be_applied(self):
        schema_class = rigexec.schema.MatrixMover
        original_apis = schema_class.applied_schemas
        schema_class.applied_schemas = (
            *original_apis, "RigExecDefinitelyMissingAPI")
        try:
            with self.assertRaises((ValueError, RuntimeError)):
                schema_class.define(self.stage, "/WouldBePartial")
        finally:
            schema_class.applied_schemas = original_apis

        self.assertFalse(self.stage.GetPrimAtPath("/WouldBePartial"))


class BuilderDependencyTests(_ContractTestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.builder = rigexec.Builder.create(self.stage, "/Rig")
        self.assertIsInstance(self.builder, rigexec.Builder)

    def _make_points(self, path="/Geometry"):
        points = UsdGeom.Points.Define(self.stage, path)
        points.CreatePointsAttr([(0.0, 0.0, 0.0)])
        return points

    def test_control_and_joint_handles_author_avar_scale_atomically(self):
        control = self.builder.add_control("ScaleControl")
        joint = self.builder.add_joint("ScaleJoint")
        for handle, expected in (
                (control, (2.0, 3.0, 4.0)),
                (joint, (0.5, 1.5, 2.5))):
            with self.subTest(handle=handle.path):
                set_avar_scale = self.require_method(
                    handle, "set_avar_scale")
                set_avar_scale(*expected)
                prim = self.stage.GetPrimAtPath(handle.path)
                attrs = tuple(
                    prim.GetAttribute(name)
                    for name in ("avars:sx", "avars:sy", "avars:sz"))
                self.assertEqual(
                    tuple(attr.Get() for attr in attrs), expected)
                for attr in attrs:
                    self.assertFalse(attr.IsCustom())
                    self.assertEqual(
                        attr.GetTypeName(), Sdf.ValueTypeNames.Double)

                # pybind converts the complete call before invoking C++; a
                # bad final component cannot leave sx/sy partially changed.
                with self.assertRaises(TypeError):
                    set_avar_scale(9.0, 8.0, "bad")
                self.assertEqual(
                    tuple(attr.Get() for attr in attrs), expected)

                set_avar_scale(
                    0.0, -0.0, -0.5 * _AVAR_SCALE_FLOOR)
                floored = (
                    _AVAR_SCALE_FLOOR, -_AVAR_SCALE_FLOOR,
                    -_AVAR_SCALE_FLOOR)
                self.assertEqual(
                    tuple(attr.Get() for attr in attrs), floored)

                # All three values are validated before any authored
                # component changes, including valid doubles that are not
                # finite.
                for non_finite in (math.nan, math.inf, -math.inf):
                    with self.assertRaises(ValueError):
                        set_avar_scale(9.0, non_finite, 8.0)
                    self.assertEqual(
                        tuple(attr.Get() for attr in attrs), floored)

    def test_volume_weight_handle_exposes_strict_avar_spin(self):
        self._make_points()
        sphere = self.builder.add_sphere_weight(
            "SpinWeight", "/Geometry.points")
        set_avar_spin = self.require_method(sphere, "set_avar_spin")

        set_avar_spin(37.5)
        prim = self.stage.GetPrimAtPath(sphere.path)
        spin = prim.GetAttribute("avars:rspin")
        self.assertTrue(spin)
        self.assertFalse(spin.IsCustom())
        self.assertEqual(spin.GetTypeName(), Sdf.ValueTypeNames.Double)
        self.assertEqual(spin.Get(), 37.5)

        # pybind rejects an incompatible payload before the strict C++
        # schema-backed setter runs, preserving the authored value.
        with self.assertRaises(TypeError):
            set_avar_spin("not a double")
        self.assertEqual(spin.Get(), 37.5)

    def test_native_handles_have_no_untyped_property_escape_hatch(self):
        data = self.stage.DefinePrim("/Data")
        data.CreateAttribute("value", Sdf.ValueTypeNames.Float).Set(1.0)
        mover = self.builder.new_mover_chain(
            "Strict", "/Data.value").add_float_math_mover(
                "Math", "add", 2.0)

        self.assertFalse(hasattr(mover, "set_attr"))
        self.assertFalse(hasattr(mover, "set_rel"))
        set_attribute = self.require_method(mover, "set_attribute")
        set_relationship = self.require_method(mover, "set_relationship")

        set_attribute("inputs:value", 3.0)
        set_relationship("rigExec:moves", [Sdf.Path("/Data.value")])
        prim = self.stage.GetPrimAtPath(mover.path)
        self.assertEqual(prim.GetAttribute("inputs:value").Get(), 3.0)
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:moves")), ["/Data.value"])
        self.assertFalse(prim.GetAttribute("inputs:value").IsCustom())
        self.assertFalse(prim.GetRelationship("rigExec:moves").IsCustom())

        with self.assertRaises((KeyError, ValueError)):
            set_attribute("inputs:notDeclared", 1.0)
        with self.assertRaises(ValueError):
            set_relationship("rigExec:notDeclared", ["/Data.value"])
        self.assertFalse(prim.HasProperty("inputs:notDeclared"))
        self.assertFalse(prim.HasProperty("rigExec:notDeclared"))

    def test_source_weights_are_cleared_by_unweighted_sources(self):
        target = self.builder.add_control("Target")
        source_a = self.builder.add_control("SourceA")
        source_b = self.builder.add_control("SourceB")
        source_c = self.builder.add_control("SourceC")
        source_d = self.builder.add_control("SourceD")
        chain = self.builder.new_mover_chain("Constraints")
        constraint = chain.add_position_constraint("Position", target.path)

        constraint.set_sources(
            [
                source_a,
                self.stage.GetPrimAtPath(source_b.path),
                Sdf.Path(source_c.path),
                source_d.path,
            ],
            [0.125, 0.25, 0.25, 0.375])
        prim = self.stage.GetPrimAtPath(constraint.path)
        weights = prim.GetAttribute("inputs:sourceWeights")
        self.assertTrue(weights.HasAuthoredValueOpinion())
        self.assertEqual(
            list(weights.Get()), [0.125, 0.25, 0.25, 0.375])

        constraint.set_sources([source_b, source_a])
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:sources")),
            [source_b.path, source_a.path])
        self.assertFalse(weights.HasAuthoredValueOpinion())
        self.assertEqual(list(weights.Get()), [])
        with self.assertRaises(TypeError):
            constraint.set_sources(source_a.path)

        set_source_weights = self.require_method(
            constraint, "set_source_weights")
        set_source_weights([0.6, 0.4])
        self.assertTrue(weights.HasAuthoredValueOpinion())
        set_source_weights([])
        self.assertFalse(weights.HasAuthoredValueOpinion())

        parent = chain.add_parent_constraint("Parent", target)
        parent.set_sources([source_a, source_b])
        parent.set_translation_offsets([(1, 2, 3), (4, 5, 6)])
        parent.set_rotation_offsets([(7, 8, 9), (10, 11, 12)])
        parent_prim = self.stage.GetPrimAtPath(parent.path)
        translations = parent_prim.GetAttribute("inputs:translationOffsets")
        rotations = parent_prim.GetAttribute("inputs:rotationOffsets")
        self.assertTrue(translations.HasAuthoredValueOpinion())
        self.assertTrue(rotations.HasAuthoredValueOpinion())

        # Both offset arrays are parallel to rigExec:sources and must not keep
        # stale entries when the source order or count changes.
        parent.set_sources([source_c])
        self.assertFalse(translations.HasAuthoredValueOpinion())
        self.assertFalse(rotations.HasAuthoredValueOpinion())

    def test_coarse_solver_and_explicit_volume_helpers_wire_dependencies(self):
        self._make_points("/Driven")
        root_control = self.builder.add_control("RootControl")
        end_control = self.builder.add_control("EndControl")
        pole_control = self.builder.add_control("PoleControl")
        root_joint = self.builder.add_joint("RootJoint")
        end_joint = self.builder.add_joint(
            "EndJoint", parent_joint=Sdf.Path(root_joint.path))

        with self.assertRaises(ValueError):
            self.builder.add_control("RootControl")
        self.assertEqual(
            str(self.stage.GetPrimAtPath(root_control.path).GetTypeName()),
            "RigExecControl")

        fk = self.builder.add_fk_chain(
            "Fk",
            controls=[
                root_control,
                self.stage.GetPrimAtPath(end_control.path),
            ],
            joints=[root_joint, Sdf.Path(end_joint.path)])
        fk_prim = self.stage.GetPrimAtPath(fk.path)
        self.assertEqual(
            _targets(fk_prim.GetRelationship("rigExec:controls")),
            [root_control.path, end_control.path])
        self.assertEqual(
            _targets(fk_prim.GetRelationship("rigExec:joints")),
            [root_joint.path, end_joint.path])

        ik = self.builder.add_two_bone_ik(
            "Ik", root_control,
            self.stage.GetPrimAtPath(end_control.path),
            Sdf.Path(pole_control.path))
        ik_prim = self.stage.GetPrimAtPath(ik.path)
        self.assertEqual(
            _targets(ik_prim.GetRelationship("rigExec:rootControl")),
            [root_control.path])
        self.assertEqual(
            _targets(ik_prim.GetRelationship("rigExec:effectorControl")),
            [end_control.path])
        self.assertEqual(
            _targets(ik_prim.GetRelationship("rigExec:poleControl")),
            [pole_control.path])

        twist = self.builder.add_twist_distribution(
            "Twist", root_control, end_control, 2)
        twist.set_joints([root_joint, end_joint])
        twist.set_joint_elements([0, 1])
        twist_prim = self.stage.GetPrimAtPath(twist.path)
        elements = twist_prim.GetAttribute("rigExec:jointElements")
        self.assertTrue(elements.HasAuthoredValueOpinion())
        twist.set_joints([end_joint])
        self.assertFalse(elements.HasAuthoredValueOpinion())

        self.stage.DefinePrim("/Rig/PlacedWeights", "Scope")
        define_sphere = self.require_method(
            self.builder, "define_sphere_weight")
        sphere = define_sphere(
            Sdf.Path("/Rig/PlacedWeights/FaceMask"),
            "/Driven.points", 0.1, 0.8)
        self.assertEqual(sphere.path, "/Rig/PlacedWeights/FaceMask")
        sphere_prim = self.stage.GetPrimAtPath(sphere.path)
        self.assertEqual(
            _targets(sphere_prim.GetRelationship("rigExec:weightTarget")),
            ["/Driven.points"])
        self.assertIn("NodeGraphNodeAPI", _applied_schema_names(sphere_prim))

        sparse = self.builder.add_static_weight(
            "Sparse", "/Driven.points", [0.25, 0.75], [0, 2])
        sparse_prim = self.stage.GetPrimAtPath(sparse.path)
        with self.assertRaises(ValueError):
            sparse.set_indices([0])
        self.assertEqual(
            list(sparse_prim.GetAttribute("rigExec:indices").Get()), [0, 2])
        sparse.set_sparse_values([0.5], [1])
        self.assertEqual(
            list(sparse_prim.GetAttribute("rigExec:values").Get()), [0.5])
        self.assertEqual(
            list(sparse_prim.GetAttribute("rigExec:indices").Get()), [1])
        sparse.set_values([0.1, 0.2])
        self.assertFalse(
            sparse_prim.GetAttribute(
                "rigExec:indices").HasAuthoredValueOpinion())
        self.assertEqual(
            str(sparse_prim.GetAttribute("rigExec:representation").Get()),
            "dense")

        foreign_stage = Usd.Stage.CreateInMemory()
        foreign_control = rigexec.Builder.create(
            foreign_stage, "/ForeignRig").add_control("Foreign")
        with self.assertRaises(ValueError):
            fk.set_controls([foreign_control])
        with self.assertRaises(TypeError):
            fk.set_controls([root_joint.path])
        self.assertEqual(
            _targets(fk_prim.GetRelationship("rigExec:controls")),
            [root_control.path, end_control.path])

        with self.assertRaises(TypeError):
            self.builder.add_fk_chain("BadFk", controls=[root_joint.path])
        self.assertFalse(self.stage.GetPrimAtPath("/Rig/Solvers/BadFk"))

        removed = self.builder.add_control("Removed")
        self.stage.RemovePrim(removed.path)
        with self.assertRaises(ValueError):
            fk.set_controls([removed])

    def test_nested_movers_are_direct_children_and_execute_post_order(self):
        data = self.stage.DefinePrim("/Data")
        data.CreateAttribute("value", Sdf.ValueTypeNames.Float).Set(1.0)
        target = "/Data.value"

        chain = self.builder.new_mover_chain("Stack", target)
        parent = chain.add_float_math_mover("Parent", "add", 10.0)
        below = chain.add_float_math_mover("Below", "add", 1.0)
        nested_chain = self.require_method(chain, "under")(
            parent, default_target=target)
        nested = nested_chain.add_float_math_mover("Nested", "multiply", 2.0)

        self.assertEqual(nested.path, parent.path + "/Nested")
        self.assertEqual(
            str(self.stage.GetPrimAtPath(nested.path).GetParent().GetPath()),
            parent.path)

        rig = rigexec.Rig(self.stage, "/Rig")
        rig.compile()
        self.assertEqual(
            [record["path"] for record in rig.mover_order()],
            [below.path, nested.path, parent.path])

    def test_full_strength_mover_builder_defaults(self):
        self._make_points("/Driven")
        curvenet = rigexec.schema.Curvenet.define(self.stage, "/Net")
        chain = self.builder.new_mover_chain(
            "DefaultEnvelopes", "/Driven.points")

        movers = (
            chain.add_smooth_mover("Smooth"),
            chain.add_volume_correct_mover("Volume"),
            chain.add_curvenet_mover("Profile", curvenet),
        )
        for mover in movers:
            with self.subTest(mover=mover.path):
                prim = self.stage.GetPrimAtPath(mover.path)
                self.assertIn(
                    "RigExecMoverAPI", _applied_schema_names(prim))
                default_weight = prim.GetAttribute("inputs:defaultWeight")
                self.assert_api_property(default_weight)
                self.assertEqual(default_weight.Get(), 1.0)

    def test_single_chain_ik_authors_the_complete_joint_write_set(self):
        root = self.builder.add_joint("Root")
        middle = self.builder.add_joint("Middle", parent_joint=root)
        end = self.builder.add_joint("End", parent_joint=middle)
        effector = self.builder.add_control("Effector")
        pole = self.builder.add_control("Pole")
        move_handles = [root, middle, end]
        move_paths = [root.path, middle.path, end.path]

        chain = self.builder.new_mover_chain("Ik")
        add_ik = self.require_method(
            chain, "add_single_chain_ik_constraint")
        # The coarse form derives the complete inclusive chain from the joint
        # namespace; callers should not have to restate its write set.
        ik = add_ik("Solve", root, end, effector, [pole])

        prim = self.stage.GetPrimAtPath(ik.path)
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:moves")), move_paths)
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:firstJoint")), [root.path])
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:endJoint")), [end.path])
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:effector")), [effector.path])
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:poleVectorObjects")),
            [pole.path])

        # The explicit form remains available for validation-heavy importers.
        explicit = add_ik(
            "SolveExplicit", root, end, effector, moves=move_handles)
        explicit_prim = self.stage.GetPrimAtPath(explicit.path)
        self.assertEqual(
            _targets(explicit_prim.GetRelationship("rigExec:moves")),
            move_paths)

    def test_blend_inputs_are_independent_and_node_graph_backed(self):
        self._make_points("/Base")
        self._make_points("/SmileTarget")
        self._make_points("/FrownTarget")

        add_blend_input = self.require_method(self.builder, "add_blend_input")
        smile = add_blend_input("Smile", 0.25)
        frown = add_blend_input("Frown", 0.75)
        self.assertNotEqual(smile.path, frown.path)

        smile_sample = smile.add_sample("Full", 1.0)
        smile_sample.set_target_points("/SmileTarget.points")
        frown_sample = frown.add_sample("Full", 1.0)
        frown_sample.set_target_points("/FrownTarget.points")

        chain = self.builder.new_mover_chain("Shapes")
        mover = chain.add_blend_shape_mover(
            "Face", target="/Base.points")
        set_blend_inputs = self.require_method(mover, "set_blend_inputs")
        set_blend_inputs([smile, frown])

        mover_prim = self.stage.GetPrimAtPath(mover.path)
        self.assertEqual(
            _targets(mover_prim.GetRelationship("rigExec:blendInputs")),
            [smile.path, frown.path])

        smile_prim = self.stage.GetPrimAtPath(smile.path)
        frown_prim = self.stage.GetPrimAtPath(frown.path)
        self.assertEqual(
            _targets(smile_prim.GetRelationship("rigExec:samples")),
            [smile_sample.path])
        self.assertEqual(
            _targets(frown_prim.GetRelationship("rigExec:samples")),
            [frown_sample.path])

        for path in (
                smile.path, frown.path, smile_sample.path, frown_sample.path):
            with self.subTest(path=path):
                self.assertIn(
                    "NodeGraphNodeAPI",
                    _applied_schema_names(self.stage.GetPrimAtPath(path)))

    def test_curve_mover_accepts_plural_driver_frames_and_metadata_phase(self):
        self._make_points("/Driven")
        driver = UsdGeom.BasisCurves.Define(self.stage, "/Driver")
        driver.CreatePointsAttr([
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (3.0, 0.0, 0.0),
        ])
        driver.CreateCurveVertexCountsAttr([4])
        frame_a = self.builder.add_control("FrameA")
        frame_b = self.builder.add_control("FrameB")

        chain = self.builder.new_mover_chain("Curves")
        add_curve_mover = self.require_method(chain, "add_curve_mover")
        mover = add_curve_mover(
            "Deform",
            "/Driver",
            [frame_a, frame_b],
            target="/Driven.points",
            read_phase="preceding")

        prim = self.stage.GetPrimAtPath(mover.path)
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:moves")),
            ["/Driven.points"])
        self.assertEqual(
            _targets(prim.GetRelationship("rigExec:driverFrames")),
            [frame_a.path, frame_b.path])
        self.assertEqual(
            prim.GetRelationship("rigExec:driverCurve").GetMetadata(
                "rigExecReadPhase"),
            "preceding")
        mover.set_read_phase("final")
        self.assertEqual(
            str(prim.GetAttribute("rigExec:driverCurveReadPhase").Get()),
            "final")
        mover.set_read_phase("rigExec:driverCurve", "base")
        self.assertEqual(
            prim.GetRelationship("rigExec:driverCurve").GetMetadata(
                "rigExecReadPhase"),
            "base")

        single = add_curve_mover(
            "SingleFrame", driver, frame_a.path,
            target="/Driven.points")
        self.assertEqual(
            _targets(self.stage.GetPrimAtPath(single.path).GetRelationship(
                "rigExec:driverFrames")),
            [frame_a.path])

    def test_failed_add_removes_the_partially_authored_prim(self):
        self._make_points("/Driven")
        weight = self.builder.add_static_weight(
            "Weight", "/Driven.points", [1.0])
        chain = self.builder.new_mover_chain("Failures")
        expected_path = chain.scope_path + "/Broken"

        with self.assertRaises((ValueError, RuntimeError)):
            chain.add_matrix_mover(
                "Broken", "", weight.path, "/Driven.points")
        self.assertFalse(self.stage.GetPrimAtPath(expected_path))

        invalid_path = chain.scope_path + "/InvalidPath"
        with self.assertRaises((TypeError, ValueError, RuntimeError)):
            chain.add_float_math_mover(
                "InvalidPath", "add", 1.0, target="not/a/USD/path?")
        self.assertFalse(self.stage.GetPrimAtPath(invalid_path))

        invalid_smooth_weight = chain.scope_path + "/InvalidSmoothWeight"
        with self.assertRaises(ValueError):
            chain.add_smooth_mover("InvalidSmoothWeight", default_weight=2.0)
        self.assertFalse(self.stage.GetPrimAtPath(invalid_smooth_weight))

        invalid_float_weight = chain.scope_path + "/InvalidFloatWeight"
        with self.assertRaises(ValueError):
            chain.add_float_math_mover(
                "InvalidFloatWeight", "add", 1.0,
                default_weight=float("nan"))
        self.assertFalse(self.stage.GetPrimAtPath(invalid_float_weight))

        removed_control = self.builder.add_control("RemovedProvider")
        removed_prim = self.stage.GetPrimAtPath(removed_control.path)
        self.stage.RemovePrim(removed_control.path)
        removed_path = chain.scope_path + "/RemovedProvider"
        with self.assertRaises(ValueError):
            chain.add_matrix_mover(
                "RemovedProvider", removed_prim, weight,
                target="/Driven.points")
        self.assertFalse(self.stage.GetPrimAtPath(removed_path))


if __name__ == "__main__":
    unittest.main(verbosity=2)
