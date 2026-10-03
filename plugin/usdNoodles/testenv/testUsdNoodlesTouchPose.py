#!/pxrpythonsubst
#
# TouchPose awareness: regions become nodes, and their control
# relationship becomes a link that lands on a node in the graph.
#
# The end-to-end version of this runs in usdview over the shipped biped
# (tests/testUsdviewNoodlesTouch.py, 98 regions). This one builds the
# same layout in memory so the suite has no asset dependency and so the
# three things that were actually WRONG are pinned individually:
#
#   * NodeGraphStage read the stage's ROOT CHILDREN only, so a group one
#     level down (/Char/TouchPose) produced no region nodes at all;
#   * the group prim is a group, not a node: selecting it in the prim
#     tree has to expand to the regions the way a Container does;
#   * a region is a `Scope`, so the generic pin discovery gave it
#     UsdGeomImageable's proxyPrim/purpose/visibility rows -- three of
#     the eight rows on every region node.
#
import unittest

try:
    from pxr import Gf, Sdf, Usd

    from UsdNoodles.nodeFactory import NodeFactory
    from UsdNoodles.nodeGraphStage import NodeGraphStage
    from UsdNoodles.nodeLibs.touchPoseLibrary import TouchPoseLibrary
    from UsdNoodles.nodeLibs.usdPrimLibrary import UsdPrimLibrary
    from UsdNoodles.touchPose import (
        collect_graph_prims,
        control_paths,
        find_touch_groups,
        group_regions,
        is_touch_group,
        is_touch_region,
    )

    _has_pxr = True
except ImportError:
    _has_pxr = False


REGIONS = ("head_touch", "hand_l_touch", "hand_r_touch")


class _Metrics(object):
    """assets/fonts/Poppins-Regular.json -- the atlas the editor loads."""

    ascender = 1.05
    descender = -0.35
    lineHeight = 1.5


def _text_width(text, font_size):
    """TextRenderer.calculateTextWidth's own no-atlas fallback."""
    return float(len(text)) * font_size * 0.5


def _make_stage(over_root=False, faces=64):
    """A character with a rig and a TouchPose group one level down.

    ``over_root`` writes the character as an `over`, which is how a
    regions-only layer is authored (the mesh layer is 4 MB and must not be
    rewritten on every re-import). Everything under an `over` ancestor is
    undefined, so the default traversal cannot see it -- the case that
    forced find_touch_groups onto the all-prims predicate.
    """
    stage = Usd.Stage.CreateInMemory()
    if over_root:
        root = stage.OverridePrim("/Char")
    else:
        root = stage.DefinePrim("/Char", "Scope")
    stage.DefinePrim("/Char/Rig", "Scope")

    scope = stage.DefinePrim("/Char/TouchPose", "Scope")
    scope.CreateAttribute(
        "touchpose:mesh", Sdf.ValueTypeNames.String, custom=True
    ).Set("/Char/Geom/body")
    scope.CreateAttribute(
        "touchpose:palette", Sdf.ValueTypeNames.Color3fArray, custom=True
    ).Set([Gf.Vec3f(0.0, 0.5, 1.0)])

    for row, name in enumerate(REGIONS):
        control = stage.DefinePrim("/Char/Rig/%s_ctl" % name[:-6], "Scope")
        control.CreateAttribute(
            "ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2
        ).Set(Gf.Vec2f(0.0, row * 0.9))
        region = stage.DefinePrim("/Char/TouchPose/%s" % name, "Scope")
        region.CreateAttribute(
            "touchpose:faces", Sdf.ValueTypeNames.IntArray, custom=True
        ).Set(list(range(faces)))
        region.CreateAttribute(
            "touchpose:elementType", Sdf.ValueTypeNames.Token, custom=True
        ).Set("face")
        region.CreateAttribute(
            "touchpose:hilight", Sdf.ValueTypeNames.Int, custom=True
        ).Set(0)
        region.CreateAttribute(
            "touchpose:color", Sdf.ValueTypeNames.Color3f, custom=True
        ).Set(Gf.Vec3f(1.0, 0.6, 0.0))
        region.CreateRelationship("touchpose:control", custom=True).SetTargets(
            [control.GetPath()]
        )
        region.CreateAttribute(
            "ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2
        ).Set(Gf.Vec2f(-1.6, row * 0.9))

    assert root
    if over_root:
        # DefinePrim DEFINES its ancestors on the way down, so the root has
        # to be put back to `over` afterwards -- the same correction
        # touchpose.usdexport._over_the_ancestors makes on the real layer.
        stage.GetRootLayer().GetPrimAtPath("/Char").specifier = Sdf.SpecifierOver
    return stage


def _load_graph(stage):
    graph = NodeGraphStage()
    graph.setStage(stage)
    graph.load(
        stage,
        _text_width,
        _Metrics(),
        nodeFactory=NodeFactory([TouchPoseLibrary(), UsdPrimLibrary()]),
    )
    return graph


@unittest.skipUnless(_has_pxr, "UsdNoodles not available")
class TestTouchPoseRecognition(unittest.TestCase):
    """What counts as a region, a group, and a control."""

    def test_region_is_found_by_its_faces(self):
        stage = _make_stage()
        region = stage.GetPrimAtPath("/Char/TouchPose/head_touch")
        self.assertTrue(is_touch_region(region))
        self.assertFalse(is_touch_group(region))

    def test_group_is_found_by_its_palette(self):
        stage = _make_stage()
        scope = stage.GetPrimAtPath("/Char/TouchPose")
        self.assertTrue(is_touch_group(scope))
        self.assertFalse(is_touch_region(scope))
        self.assertEqual(len(group_regions(scope)), len(REGIONS))

    def test_plain_scope_is_neither(self):
        stage = _make_stage()
        rig = stage.GetPrimAtPath("/Char/Rig")
        self.assertFalse(is_touch_region(rig))
        self.assertFalse(is_touch_group(rig))

    def test_group_found_under_an_over_root(self):
        """A regions-only layer is an `over`; its prims are undefined."""
        stage = _make_stage(over_root=True)
        self.assertEqual(len(list(stage.Traverse())), 0)
        groups = find_touch_groups(stage)
        self.assertEqual([str(p.GetPath()) for p in groups], ["/Char/TouchPose"])
        self.assertEqual(len(group_regions(groups[0])), len(REGIONS))

    def test_expansion_is_group_then_regions_then_controls(self):
        stage = _make_stage()
        scope = stage.GetPrimAtPath("/Char/TouchPose")
        paths = [str(p.GetPath()) for p in collect_graph_prims(scope, stage)]
        self.assertEqual(paths[0], "/Char/TouchPose")
        self.assertEqual(len(paths), 1 + 2 * len(REGIONS))
        for name in REGIONS:
            self.assertIn("/Char/TouchPose/%s" % name, paths)
            self.assertIn("/Char/Rig/%s_ctl" % name[:-6], paths)


@unittest.skipUnless(_has_pxr, "UsdNoodles not available")
class TestTouchPoseInStageGraph(unittest.TestCase):
    """What the loader usdview calls on open actually produces."""

    def test_regions_and_controls_become_nodes(self):
        stage = _make_stage()
        graph = _load_graph(stage)
        for name in REGIONS:
            self.assertIn("/Char/TouchPose/%s" % name, graph.nodes)
            self.assertIn("/Char/Rig/%s_ctl" % name[:-6], graph.nodes)
        self.assertIn("/Char/TouchPose", graph.nodes)

    def test_control_relationship_is_a_link_onto_a_node(self):
        stage = _make_stage()
        graph = _load_graph(stage)
        drawn = 0
        for name in REGIONS:
            node = graph.nodes["/Char/TouchPose/%s" % name]
            links = [
                link
                for link in list(node.outputLinks)
                if link.sourcePort == "touchpose:control"
            ]
            self.assertEqual(len(links), 1, "%s has %d control links" % (name, len(links)))
            link = links[0]
            self.assertTrue(link.is_relationship_link)
            self.assertEqual(link.targetNodeId, "/Char/Rig/%s_ctl" % name[:-6])
            self.assertIn(link.targetNodeId, graph.nodes)
            drawn += 1
        self.assertEqual(drawn, len(REGIONS))

    def test_a_region_binding_nothing_is_a_node_with_no_link(self):
        stage = _make_stage()
        orphan = stage.DefinePrim("/Char/TouchPose/orphan_touch", "Scope")
        orphan.CreateAttribute(
            "touchpose:faces", Sdf.ValueTypeNames.IntArray, custom=True
        ).Set([1, 2])
        self.assertEqual(control_paths(orphan), [])
        graph = _load_graph(stage)
        node = graph.nodes["/Char/TouchPose/orphan_touch"]
        self.assertEqual(
            [
                link
                for link in list(node.outputLinks)
                if link.sourcePort == "touchpose:control"
            ],
            [],
        )

    def test_authored_positions_are_read_at_the_editor_s_scale(self):
        """1/1000 on the way in, 1000 on the way out. Same constant."""
        stage = _make_stage()
        graph = _load_graph(stage)
        node = graph.nodes["/Char/TouchPose/hand_l_touch"]
        self.assertAlmostEqual(node.position[0], -1600.0, places=2)
        self.assertAlmostEqual(node.position[1], 900.0, places=2)

    def test_placed_regions_do_not_overlap(self):
        stage = _make_stage()
        graph = _load_graph(stage)
        boxes = [
            (
                graph.nodes["/Char/TouchPose/%s" % name].position,
                graph.nodes["/Char/TouchPose/%s" % name].size,
            )
            for name in REGIONS
        ]
        for i, (pos_a, size_a) in enumerate(boxes):
            for pos_b, size_b in boxes[i + 1 :]:
                overlap = (
                    pos_a[0] < pos_b[0] + size_b[0]
                    and pos_b[0] < pos_a[0] + size_a[0]
                    and pos_a[1] < pos_b[1] + size_b[1]
                    and pos_b[1] < pos_a[1] + size_a[1]
                )
                self.assertFalse(overlap, "region nodes overlap at 0.9 pitch")


@unittest.skipUnless(_has_pxr, "UsdNoodles not available")
class TestTouchPosePins(unittest.TestCase):
    """What a region node shows, and what it costs."""

    def _descriptor(self, prim_path, stage=None):
        stage = stage if stage is not None else _make_stage()
        prim = stage.GetPrimAtPath(prim_path)
        return TouchPoseLibrary().create_node_descriptor_from_prim(prim, stage)

    def test_schema_boilerplate_is_dropped(self):
        descriptor = self._descriptor("/Char/TouchPose/head_touch")
        rows = list(descriptor["inputPins"]) + list(descriptor["outputPins"])
        for noise in ("purpose", "visibility", "proxyPrim"):
            self.assertNotIn(noise, rows)

    def test_touchpose_rows_survive(self):
        descriptor = self._descriptor("/Char/TouchPose/head_touch")
        self.assertEqual(
            sorted(descriptor["inputPins"]),
            [
                "touchpose:color",
                "touchpose:elementType",
                "touchpose:faces",
                "touchpose:hilight",
            ],
        )
        self.assertIn("touchpose:control", list(descriptor["outputPins"]))

    def test_the_face_array_costs_a_name_and_not_a_value(self):
        """A row, whatever the array's length.

        usdNoodles reads attribute VALUES for `ui:nodegraph:*` and
        `info:implementationSource` and nowhere else, so a 64-int and a
        20,000-int region must lay out identically. If that ever changes,
        the node's own measured width is where it shows up.
        """
        small = _load_graph(_make_stage(faces=64))
        large = _load_graph(_make_stage(faces=20000))
        small_node = small.nodes["/Char/TouchPose/head_touch"]
        large_node = large.nodes["/Char/TouchPose/head_touch"]
        self.assertEqual(tuple(small_node.size), tuple(large_node.size))
        self.assertIn("touchpose:faces", list(large_node.inputPins))

    def test_the_group_keeps_its_palette_rows(self):
        descriptor = self._descriptor("/Char/TouchPose")
        self.assertEqual(
            sorted(descriptor["inputPins"]),
            ["touchpose:mesh", "touchpose:palette"],
        )

    def test_library_declines_everything_else(self):
        stage = _make_stage()
        library = TouchPoseLibrary()
        self.assertFalse(
            library.can_handle_prim(stage.GetPrimAtPath("/Char/Rig"))
        )
        self.assertIsNone(
            library.create_node_descriptor_from_prim(
                stage.GetPrimAtPath("/Char/Rig"), stage
            )
        )


try:
    from UsdNoodles.graphView import GraphView

    _has_graph_view = True
except ImportError:
    _has_graph_view = False


def _fake_view(stage, prims):
    """The smallest namespace addNodesFromPrimTreeSelection will run on.

    Same shape as testUsdNoodlesAddRemoveNode's: the method is called
    unbound, so only the attributes it touches have to exist.
    """
    from types import SimpleNamespace
    from unittest.mock import MagicMock

    view = SimpleNamespace(
        _usdviewApi=SimpleNamespace(
            dataModel=SimpleNamespace(
                selection=SimpleNamespace(getPrims=lambda: list(prims))
            ),
            stage=stage,
        ),
        nodes={},
        _selectedNodes=set(),
        _lastPrimTreeSelection=[],
        linksChanged=False,
        textChanged=False,
        _nextZOrder=0,
        _clearRenderCache=MagicMock(),
        textRenderer=SimpleNamespace(resetPositionCaches=MagicMock()),
        _cppIconRenderer=SimpleNamespace(resetPositionCaches=MagicMock()),
        _nodeRenderManager=SimpleNamespace(resetNodeQuadCaches=MagicMock()),
        _nodeTransformFrame=SimpleNamespace(reset=MagicMock()),
        _gridPlaceNodes=MagicMock(return_value=[]),
        _viewportAnchor=MagicMock(return_value=(0.0, 0.0)),
        _isVisible=MagicMock(return_value=True),
        _frameNodeBounds=MagicMock(),
        nodeGraph=SimpleNamespace(syncSelectionToPrimTree=False),
        _showPopupMessage=MagicMock(),
        update=MagicMock(),
    )

    def _add(prim, stage_arg, added_node_ids):
        from types import SimpleNamespace as NS

        node_id = str(prim.GetPath())
        view.nodes[node_id] = NS(
            id=node_id, position=Gf.Vec2d(0, 0), size=Gf.Vec2d(1, 1), selected=False
        )
        added_node_ids.append(node_id)
        return True

    view._addPrimAsNode = _add
    return view


@unittest.skipUnless(
    _has_pxr and _has_graph_view, "UsdNoodles graph view not available"
)
class TestTouchPoseGroupExpansion(unittest.TestCase):
    """Selecting the group prim means "show me the regions"."""

    def test_selecting_the_group_adds_regions_and_controls(self):
        stage = _make_stage()
        scope = stage.GetPrimAtPath("/Char/TouchPose")
        view = _fake_view(stage, [scope])
        GraphView.addNodesFromPrimTreeSelection(view)
        self.assertEqual(len(view.nodes), 1 + 2 * len(REGIONS))
        for name in REGIONS:
            self.assertIn("/Char/TouchPose/%s" % name, view.nodes)
            self.assertIn("/Char/Rig/%s_ctl" % name[:-6], view.nodes)

    def test_selecting_one_region_adds_only_that_region(self):
        """Expansion is the GROUP's meaning, not every touchpose prim's."""
        stage = _make_stage()
        region = stage.GetPrimAtPath("/Char/TouchPose/head_touch")
        view = _fake_view(stage, [region])
        GraphView.addNodesFromPrimTreeSelection(view)
        self.assertEqual(list(view.nodes), ["/Char/TouchPose/head_touch"])


if __name__ == "__main__":
    unittest.main()
