#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""Unit tests for ``UsdNoodles.noodlesValues`` -- the pure value model behind
the inline attribute value cells.

Everything here runs against an in-memory ``Usd.Stage``: no Qt, no GL, no
``GraphView`` (whose module-level GL imports are why
``testUsdNoodlesPinDragInteractions.py`` mirrors logic instead of importing
it). What is asserted, in order of how much it would cost to get wrong:

  * the TYPE WHITELIST -- only simple types get a cell. A matrix, an array,
    an asset path or a quaternion getting one is the failure this guards.
  * ``write_value`` NEVER creates an attribute, and honours uniform /
    time-sampled / default authoring exactly as ``gizmoMath.SetAnimated``
    does.
  * ``value_is_editable`` -- a schema fallback IS editable, a connection is
    not, a locked edit target is not.
  * the mung ladder and ``apply_delta``, including that an untouched
    component is carried through unmodified.
"""

import unittest

from pxr import Gf, Sdf, Usd

try:
    from UsdNoodles import noodlesValues as nv

    _HAS_VALUES = True
except ImportError as e:  # pragma: no cover - import guard
    print(f"UsdNoodles.noodlesValues import failed: {e}")
    _HAS_VALUES = False


def _width(text):
    """A deterministic stand-in for the MSDF text measurer."""
    return len(text) * 10.0


def _stage_with(props):
    """An in-memory stage whose /Values prim carries ``{name: (type, value)}``."""
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/Values", "Scope")
    for name, (type_name, value) in props.items():
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Find(type_name))
        if value is not None:
            attr.Set(value)
    return stage, prim


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestTypeWhitelist(unittest.TestCase):
    ACCEPTED = [
        "bool",
        "int",
        "int64",
        "uint",
        "uint64",
        "half",
        "float",
        "double",
        "string",
        "token",
        "int2",
        "int3",
        "int4",
        "half2",
        "half3",
        "half4",
        "float2",
        "float3",
        "float4",
        "double2",
        "double3",
        "double4",
        "point3f",
        "point3d",
        "point3h",
        "normal3f",
        "normal3d",
        "normal3h",
        "vector3f",
        "vector3d",
        "vector3h",
        "color3f",
        "color3d",
        "color3h",
        "color4f",
        "color4d",
        "color4h",
        "texCoord2f",
        "texCoord2d",
        "texCoord2h",
        "texCoord3f",
        "texCoord3d",
        "texCoord3h",
    ]

    REJECTED = [
        "matrix2d",
        "matrix3d",
        "matrix4d",
        "frame4d",
        "quatf",
        "quatd",
        "quath",
        "asset",
        "timecode",
        "opaque",
        "group",
        "uchar",
        "dictionary",
        "float3[]",
        "int[]",
        "token[]",
        "asset[]",
        "matrix4d[]",
    ]

    def test_accepted_types_are_whitelisted(self):
        for name in self.ACCEPTED:
            self.assertTrue(nv.is_value_type(name), name)

    def test_rejected_types_get_no_cell(self):
        for name in self.REJECTED:
            self.assertFalse(nv.is_value_type(name), name)
            self.assertEqual(nv.component_count(name), 0, name)
            self.assertEqual(nv.cell_kind(name), "", name)

    def test_every_accepted_name_is_a_real_sdf_type(self):
        # The whitelist is only meaningful if it names types USD actually
        # has; a typo here would silently suppress a cell forever.
        for name in self.ACCEPTED:
            self.assertTrue(
                bool(Sdf.ValueTypeNames.Find(name)), "no Sdf type named %s" % name
            )

    def test_component_counts(self):
        self.assertEqual(nv.component_count("float"), 1)
        self.assertEqual(nv.component_count("bool"), 1)
        self.assertEqual(nv.component_count("token"), 1)
        self.assertEqual(nv.component_count("float2"), 2)
        self.assertEqual(nv.component_count("texCoord2f"), 2)
        self.assertEqual(nv.component_count("color3f"), 3)
        self.assertEqual(nv.component_count("point3d"), 3)
        self.assertEqual(nv.component_count("color4f"), 4)
        self.assertEqual(nv.component_count("int4"), 4)

    def test_cell_kinds(self):
        self.assertEqual(nv.cell_kind("bool"), nv.KIND_BOOL)
        self.assertEqual(nv.cell_kind("int"), nv.KIND_INT)
        self.assertEqual(nv.cell_kind("int3"), nv.KIND_INT)
        self.assertEqual(nv.cell_kind("double"), nv.KIND_FLOAT)
        self.assertEqual(nv.cell_kind("color3f"), nv.KIND_FLOAT)
        self.assertEqual(nv.cell_kind("string"), nv.KIND_TEXT)
        self.assertEqual(nv.cell_kind("token"), nv.KIND_TEXT)
        self.assertEqual(nv.cell_kind("token", True), nv.KIND_TOKEN_ENUM)

    def test_only_numbers_are_mungable(self):
        self.assertTrue(nv.is_mungable(nv.KIND_FLOAT))
        self.assertTrue(nv.is_mungable(nv.KIND_INT))
        self.assertFalse(nv.is_mungable(nv.KIND_BOOL))
        self.assertFalse(nv.is_mungable(nv.KIND_TEXT))
        self.assertFalse(nv.is_mungable(nv.KIND_TOKEN_ENUM))

    def test_colour_types(self):
        self.assertTrue(nv.is_color_type("color3f"))
        self.assertTrue(nv.is_color_type("color4d"))
        self.assertFalse(nv.is_color_type("float3"))
        self.assertFalse(nv.is_color_type("normal3f"))


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestLadder(unittest.TestCase):
    def test_step_tracks_magnitude(self):
        self.assertAlmostEqual(nv.ladder_step(1.0, "float"), 0.01)
        self.assertAlmostEqual(nv.ladder_step(-1.0, "float"), 0.01)
        self.assertAlmostEqual(nv.ladder_step(0.0, "float"), 0.01)
        self.assertAlmostEqual(nv.ladder_step(123.0, "float"), 1.0)
        self.assertAlmostEqual(nv.ladder_step(2.5, "double"), 0.01)

    def test_step_is_clamped_at_both_ends(self):
        # A tiny value would otherwise give a step no drag could ever move.
        self.assertAlmostEqual(nv.ladder_step(0.004, "float"), 1e-4)
        self.assertAlmostEqual(nv.ladder_step(1e-30, "float"), 0.01)
        self.assertAlmostEqual(nv.ladder_step(1e30, "float"), 1e3)

    def test_integers_always_step_one(self):
        for value in (0, 1, 999, -50):
            self.assertEqual(nv.ladder_step(value, "int"), 1.0)
            self.assertEqual(nv.ladder_step(value, "int3"), 1.0)

    def test_modifiers(self):
        self.assertEqual(nv.mung_multiplier(), 1.0)
        self.assertEqual(nv.mung_multiplier(shift=True), 10.0)
        self.assertAlmostEqual(nv.mung_multiplier(ctrl=True), 0.1)
        self.assertAlmostEqual(nv.mung_multiplier(shift=True, ctrl=True), 1.0)

    def test_a_hundred_pixel_drag_adds_one_unit_near_one(self):
        base = 1.0
        step = nv.ladder_step(base, "float")
        self.assertAlmostEqual(base + 100 * step * nv.mung_multiplier(), 2.0)


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestApplyDelta(unittest.TestCase):
    def test_untouched_components_are_carried_through_unmodified(self):
        original = (1.25, 2.5, 3.75)
        result = nv.apply_delta(original, 1, 9.5, "float3")
        self.assertEqual(result, (1.25, 9.5, 3.75))
        # Identity, byte for byte: a half that was never touched must not be
        # reformatted and re-parsed on its way through.
        self.assertIs(type(result[0]), float)
        self.assertEqual(result[0], original[0])
        self.assertEqual(result[2], original[2])

    def test_round_trip(self):
        original = (1.0, 2.0, 3.0)
        once = nv.apply_delta(original, 0, 7.0, "float3")
        back = nv.apply_delta(once, 0, 1.0, "float3")
        self.assertEqual(back, original)

    def test_integers_are_rounded(self):
        self.assertEqual(nv.apply_delta((0,), 0, 2.6, "int"), (3,))
        self.assertEqual(nv.apply_delta((0, 0), 1, -2.6, "int2"), (0, -3))

    def test_out_of_range_index_is_a_no_op(self):
        original = (1.0, 2.0)
        self.assertEqual(nv.apply_delta(original, 5, 9.0, "float2"), original)


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestFormatting(unittest.TestCase):
    def test_float_drops_decimals_before_it_elides(self):
        # Wide enough for "0.123" (5 chars) but not "0.1235" (6).
        text = nv.fit_component(0.12345, "float", 55.0, _width)
        self.assertEqual(text, "0.123")
        self.assertNotIn("…", text)

    def test_float_uses_all_four_decimals_when_there_is_room(self):
        self.assertEqual(nv.fit_component(0.12345, "float", 1000.0, _width), "0.1235")

    def test_long_token_is_middle_elided(self):
        text = nv.fit_component(
            "aVeryLongTokenNameIndeed", "token", 100.0, _width
        )
        self.assertIn("…", text)
        self.assertLessEqual(_width(text), 100.0)
        self.assertTrue(text.startswith("aVe"))
        self.assertTrue(text.endswith("ed"))

    def test_short_text_is_untouched(self):
        self.assertEqual(nv.fit_component("world", "token", 1000.0, _width), "world")

    def test_integers_are_never_elided_when_they_fit(self):
        self.assertEqual(nv.fit_component(42, "int", 1000.0, _width), "42")

    def test_bool_text(self):
        self.assertEqual(nv.format_component(True, "bool"), "true")
        self.assertEqual(nv.format_component(False, "bool"), "false")

    def test_every_component_of_one_value_shares_a_format(self):
        # Per-component fitting would give "1.0000" beside "0.00", because
        # '1' measures narrower than '0'. Three unrelated-looking numbers is
        # not what one float3 is.
        texts = nv.fit_components((1.0, 0.0, 22.5), "float3", 55.0, _width)
        self.assertEqual(len(set(len(t.split(".")[1]) for t in texts)), 1)
        self.assertTrue(all(_width(t) <= 55.0 for t in texts), texts)

    def test_shared_format_uses_all_the_room_it_has(self):
        texts = nv.fit_components((1.0, 2.0, 3.0), "float3", 1000.0, _width)
        self.assertEqual(texts, ("1.0000", "2.0000", "3.0000"))

    def test_shared_format_falls_back_to_scalars_for_text(self):
        texts = nv.fit_components(("hello",), "token", 1000.0, _width)
        self.assertEqual(texts, ("hello",))

    def test_parse(self):
        self.assertEqual(nv.parse_component("2.5", "float"), (True, 2.5))
        self.assertEqual(nv.parse_component(" 3 ", "int"), (True, 3))
        self.assertEqual(nv.parse_component("2.6", "int"), (True, 3))
        self.assertEqual(nv.parse_component("world", "token"), (True, "world"))
        self.assertEqual(nv.parse_component("true", "bool"), (True, True))
        self.assertEqual(nv.parse_component("off", "bool"), (True, False))
        self.assertEqual(nv.parse_component("banana", "float")[0], False)
        self.assertEqual(nv.parse_component("", "double")[0], False)


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestReadComponents(unittest.TestCase):
    def test_scalars_and_vectors(self):
        stage, prim = _stage_with(
            {
                "f": ("float", 1.5),
                "d": ("double", 2.5),
                "i": ("int", 3),
                "b": ("bool", True),
                "t": ("token", "hello"),
                "s": ("string", "text"),
                "v": ("float3", Gf.Vec3f(1, 2, 3)),
                "c": ("color3f", Gf.Vec3f(1, 0, 0)),
            }
        )
        self.assertEqual(nv.read_components(prim, "f"), (1.5,))
        self.assertEqual(nv.read_components(prim, "d"), (2.5,))
        self.assertEqual(nv.read_components(prim, "i"), (3,))
        self.assertEqual(nv.read_components(prim, "b"), (True,))
        self.assertEqual(nv.read_components(prim, "t"), ("hello",))
        self.assertEqual(nv.read_components(prim, "s"), ("text",))
        self.assertEqual(nv.read_components(prim, "v"), (1.0, 2.0, 3.0))
        self.assertEqual(nv.read_components(prim, "c"), (1.0, 0.0, 0.0))

    def test_unsupported_types_read_as_nothing(self):
        stage, prim = _stage_with(
            {
                "m": ("matrix4d", Gf.Matrix4d(1.0)),
                "q": ("quatf", Gf.Quatf(1.0)),
                "a": ("float3[]", None),
            }
        )
        self.assertEqual(nv.read_components(prim, "m"), ())
        self.assertEqual(nv.read_components(prim, "q"), ())
        self.assertEqual(nv.read_components(prim, "a"), ())

    def test_unauthored_attribute_reads_as_zero_not_as_a_crash(self):
        stage, prim = _stage_with({"f": ("float", None), "v": ("float3", None)})
        self.assertEqual(nv.read_components(prim, "f"), (0.0,))
        self.assertEqual(nv.read_components(prim, "v"), (0.0, 0.0, 0.0))

    def test_missing_attribute(self):
        stage, prim = _stage_with({})
        self.assertEqual(nv.read_components(prim, "nope"), ())


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestEditability(unittest.TestCase):
    def test_a_plain_attribute_is_editable(self):
        stage, prim = _stage_with({"f": ("float", 1.0)})
        ok, reason = nv.value_is_editable(stage, prim, prim.GetAttribute("f"))
        self.assertTrue(ok, reason)

    def test_a_schema_fallback_is_editable(self):
        # No authored opinion is not read-only: the first edit authors one.
        stage, prim = _stage_with({"f": ("float", None)})
        attr = prim.GetAttribute("f")
        self.assertFalse(attr.HasAuthoredValue())
        ok, reason = nv.value_is_editable(stage, prim, attr)
        self.assertTrue(ok, reason)

    def test_a_connected_attribute_is_read_only(self):
        stage, prim = _stage_with({"src": ("float", 1.0), "dst": ("float", 0.0)})
        prim.GetAttribute("dst").AddConnection(prim.GetAttribute("src").GetPath())
        ok, reason = nv.value_is_editable(stage, prim, prim.GetAttribute("dst"))
        self.assertFalse(ok)
        self.assertEqual(reason, "driven by a connection")

    def test_an_unsupported_type_is_rejected(self):
        stage, prim = _stage_with({"m": ("matrix4d", Gf.Matrix4d(1.0))})
        ok, reason = nv.value_is_editable(stage, prim, prim.GetAttribute("m"))
        self.assertFalse(ok)
        self.assertEqual(reason, "unsupported type")

    def test_a_locked_edit_target_is_rejected(self):
        stage, prim = _stage_with({"f": ("float", 1.0)})
        stage.GetEditTarget().GetLayer().SetPermissionToEdit(False)
        try:
            ok, reason = nv.value_is_editable(stage, prim, prim.GetAttribute("f"))
            self.assertFalse(ok)
            self.assertEqual(reason, "edit target is locked")
        finally:
            stage.GetEditTarget().GetLayer().SetPermissionToEdit(True)

    def test_a_missing_attribute_is_rejected(self):
        stage, prim = _stage_with({})
        ok, reason = nv.value_is_editable(stage, prim, prim.GetAttribute("nope"))
        self.assertFalse(ok)
        self.assertEqual(reason, "no such attribute")


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestValueState(unittest.TestCase):
    def test_authored(self):
        stage, prim = _stage_with({"f": ("float", 1.0)})
        self.assertEqual(
            nv.value_state(prim, prim.GetAttribute("f")), nv.STATE_AUTHORED
        )

    def test_fallback(self):
        stage, prim = _stage_with({"f": ("float", None)})
        self.assertEqual(
            nv.value_state(prim, prim.GetAttribute("f")), nv.STATE_FALLBACK
        )

    def test_animated(self):
        stage, prim = _stage_with({"f": ("float", None)})
        prim.GetAttribute("f").Set(1.0, Usd.TimeCode(1.0))
        self.assertEqual(
            nv.value_state(prim, prim.GetAttribute("f")), nv.STATE_ANIMATED
        )

    def test_connected_wins_over_everything(self):
        stage, prim = _stage_with({"src": ("float", 1.0), "dst": ("float", 2.0)})
        prim.GetAttribute("dst").AddConnection(prim.GetAttribute("src").GetPath())
        self.assertEqual(
            nv.value_state(prim, prim.GetAttribute("dst")), nv.STATE_CONNECTED
        )


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestWriteValue(unittest.TestCase):
    def test_default_authoring_leaves_no_time_samples(self):
        stage, prim = _stage_with({"f": ("float", 1.0)})
        ok, reason = nv.write_value(stage, prim, "f", (2.5,), Usd.TimeCode(12.0))
        self.assertTrue(ok, reason)
        attr = prim.GetAttribute("f")
        self.assertEqual(attr.GetNumTimeSamples(), 0)
        self.assertAlmostEqual(attr.Get(), 2.5)

    def test_an_already_animated_attribute_gets_a_sample(self):
        stage, prim = _stage_with({"f": ("float", None)})
        attr = prim.GetAttribute("f")
        attr.Set(0.0, Usd.TimeCode(1.0))
        attr.Set(1.0, Usd.TimeCode(24.0))
        before = attr.GetNumTimeSamples()
        ok, reason = nv.write_value(stage, prim, "f", (5.0,), Usd.TimeCode(12.0))
        self.assertTrue(ok, reason)
        self.assertEqual(attr.GetNumTimeSamples(), before + 1)
        self.assertAlmostEqual(attr.Get(Usd.TimeCode(12.0)), 5.0)
        # The default is a separate opinion and must not have been touched.
        self.assertIsNone(attr.Get(Usd.TimeCode.Default()))

    def test_mode_default_never_samples(self):
        stage, prim = _stage_with({"f": ("float", None)})
        attr = prim.GetAttribute("f")
        attr.Set(0.0, Usd.TimeCode(1.0))
        before = attr.GetNumTimeSamples()
        ok, reason = nv.write_value(
            stage, prim, "f", (5.0,), Usd.TimeCode(12.0), mode="default"
        )
        self.assertTrue(ok, reason)
        self.assertEqual(attr.GetNumTimeSamples(), before)
        self.assertAlmostEqual(attr.Get(Usd.TimeCode.Default()), 5.0)

    def test_mode_animation_samples_even_without_existing_samples(self):
        stage, prim = _stage_with({"f": ("float", 1.0)})
        ok, reason = nv.write_value(
            stage, prim, "f", (5.0,), Usd.TimeCode(12.0), mode="animation"
        )
        self.assertTrue(ok, reason)
        self.assertEqual(prim.GetAttribute("f").GetNumTimeSamples(), 1)

    def test_uniform_always_authors_at_default(self):
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Values", "Scope")
        attr = prim.CreateAttribute(
            "u", Sdf.ValueTypeNames.Token, variability=Sdf.VariabilityUniform
        )
        attr.Set("a")
        ok, reason = nv.write_value(
            stage, prim, "u", ("b",), Usd.TimeCode(12.0), mode="animation"
        )
        self.assertTrue(ok, reason)
        self.assertEqual(attr.GetNumTimeSamples(), 0)
        self.assertEqual(attr.Get(), "b")

    def test_write_never_creates_an_attribute(self):
        # The whole feature authors with attr.Set and nothing else. A prim
        # gaining a property because someone dragged over empty space would
        # be a far worse bug than a value not changing.
        stage, prim = _stage_with({})
        ok, reason = nv.write_value(stage, prim, "ghost", (1.0,))
        self.assertFalse(ok)
        self.assertEqual(reason, "no such attribute")
        self.assertFalse(bool(prim.GetAttribute("ghost")))
        self.assertNotIn("ghost", prim.GetPropertyNames())

    def test_write_refuses_a_connected_attribute(self):
        stage, prim = _stage_with({"src": ("float", 1.0), "dst": ("float", 2.0)})
        prim.GetAttribute("dst").AddConnection(prim.GetAttribute("src").GetPath())
        ok, reason = nv.write_value(stage, prim, "dst", (9.0,))
        self.assertFalse(ok)
        self.assertEqual(reason, "driven by a connection")
        self.assertAlmostEqual(prim.GetAttribute("dst").Get(), 2.0)

    def test_vector_round_trip(self):
        stage, prim = _stage_with({"v": ("float3", Gf.Vec3f(1, 2, 3))})
        ok, reason = nv.write_value(stage, prim, "v", (4.0, 5.0, 6.0))
        self.assertTrue(ok, reason)
        self.assertEqual(prim.GetAttribute("v").Get(), Gf.Vec3f(4, 5, 6))

    def test_pack_components_per_type(self):
        self.assertEqual(nv.pack_components((1.5,), "float"), 1.5)
        self.assertEqual(nv.pack_components((3,), "int"), 3)
        self.assertEqual(nv.pack_components((True,), "bool"), True)
        self.assertEqual(nv.pack_components(("x",), "token"), "x")
        self.assertEqual(nv.pack_components((1, 2, 3), "float3"), Gf.Vec3f(1, 2, 3))
        self.assertEqual(nv.pack_components((1, 2), "int2"), Gf.Vec2i(1, 2))
        self.assertEqual(
            nv.pack_components((1, 2, 3, 4), "color4f"), Gf.Vec4f(1, 2, 3, 4)
        )
        with self.assertRaises(ValueError):
            nv.pack_components((1,), "matrix4d")


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestAllowedTokens(unittest.TestCase):
    def test_authored_allowed_tokens(self):
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Values", "Scope")
        attr = prim.CreateAttribute("space", Sdf.ValueTypeNames.Token)
        attr.SetMetadata("allowedTokens", ["world", "parentRelative"])
        self.assertEqual(
            nv.allowed_tokens(prim, "space"), ["world", "parentRelative"]
        )

    def test_a_plain_token_has_none(self):
        stage, prim = _stage_with({"t": ("token", "hello")})
        self.assertEqual(nv.allowed_tokens(prim, "t"), [])

    def test_rigexec_fk_chain_control_space(self):
        # The target case. The attribute exists on the composed prim
        # definition with NO opinion authored, which is exactly the path the
        # popup has to work through. Skipped where the schema is not
        # registered (ctest does not put rigExecSchema on the plugin path).
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/S", "RigExecFkChain")
        if not prim or not prim.IsValid() or not prim.GetTypeName():
            self.skipTest("RigExecFkChain schema not registered")
        attr = prim.GetAttribute("rigExec:controlSpace")
        if not attr or not attr.IsValid():
            self.skipTest("RigExecFkChain schema not registered")
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(
            nv.allowed_tokens(prim, "rigExec:controlSpace"),
            ["world", "parentRelative"],
        )
        self.assertEqual(attr.GetVariability(), Sdf.VariabilityUniform)

    def test_a_row_for_an_enum_token_is_a_popup_row(self):
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Values", "Scope")
        attr = prim.CreateAttribute("space", Sdf.ValueTypeNames.Token)
        attr.SetMetadata("allowedTokens", ["world", "parentRelative"])
        attr.Set("world")
        row = nv.build_value_row(stage, prim, "space", "space")
        self.assertIsNotNone(row)
        self.assertEqual(row.kind, nv.KIND_TOKEN_ENUM)
        self.assertEqual(row.tokens, ("world", "parentRelative"))
        self.assertFalse(row.mungable)


@unittest.skipUnless(_HAS_VALUES, "UsdNoodles.noodlesValues unavailable")
class TestBuildValueRow(unittest.TestCase):
    def test_a_simple_row(self):
        stage, prim = _stage_with({"f": ("float", 1.0)})
        row = nv.build_value_row(stage, prim, "f", "f")
        self.assertIsNotNone(row)
        self.assertEqual(row.kind, nv.KIND_FLOAT)
        self.assertEqual(row.count, 1)
        self.assertEqual(row.components, (1.0,))
        self.assertEqual(row.texts, ("1.0000",))
        self.assertTrue(row.mungable)
        self.assertEqual(row.state, nv.STATE_AUTHORED)

    def test_unsupported_types_get_no_row_at_all(self):
        stage, prim = _stage_with(
            {
                "m": ("matrix4d", Gf.Matrix4d(1.0)),
                "q": ("quatf", Gf.Quatf(1.0)),
                "a": ("float3[]", None),
                "tex": ("asset", None),
            }
        )
        for name in ("m", "q", "a", "tex"):
            self.assertIsNone(nv.build_value_row(stage, prim, name, name), name)

    def test_a_connected_row_is_read_only_but_still_shows_its_value(self):
        stage, prim = _stage_with({"src": ("float", 7.0), "dst": ("float", 2.0)})
        prim.GetAttribute("dst").AddConnection(prim.GetAttribute("src").GetPath())
        row = nv.build_value_row(stage, prim, "dst", "dst")
        self.assertIsNotNone(row)
        self.assertFalse(row.editable)
        self.assertFalse(row.mungable)
        self.assertEqual(row.state, nv.STATE_CONNECTED)
        self.assertEqual(row.components, (2.0,))

    def test_a_colour_row_carries_a_swatch(self):
        stage, prim = _stage_with({"c": ("color3f", Gf.Vec3f(1, 0, 0))})
        row = nv.build_value_row(stage, prim, "c", "c")
        self.assertTrue(row.has_swatch)
        self.assertEqual(row.count, 3)

    def test_an_animated_row_reads_at_the_given_frame(self):
        stage, prim = _stage_with({"f": ("float", None)})
        attr = prim.GetAttribute("f")
        attr.Set(0.0, Usd.TimeCode(1.0))
        attr.Set(10.0, Usd.TimeCode(11.0))
        row = nv.build_value_row(stage, prim, "f", "f", Usd.TimeCode(6.0))
        self.assertEqual(row.state, nv.STATE_ANIMATED)
        self.assertAlmostEqual(row.components[0], 5.0, places=4)


if __name__ == "__main__":
    unittest.main()
