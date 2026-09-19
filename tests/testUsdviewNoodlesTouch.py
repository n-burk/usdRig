#
# TOUCHPOSE IN THE NODE GRAPH, end to end, on the stage usdview composed.
#
# The claim under test: the stage loader turns every painted region
# into a node, and the noodle leaving it lands on the control that region
# actually selects. So this asserts the graph NodeGraphStage.load builds
# over usdviewApi's OWN stage, and it asserts COUNTS and ENDPOINTS, not
# that a widget came up. (The editor window itself opens empty -- prims
# reach the canvas via 'A' -- so this is the loader primitive's
# contract, not a picture of what open shows.)
#
# Deliberately no GL and no Noodles panel: the graph model is what has to
# be right, the headless runners must not load an extra panel into the
# app they are asserting against (see bin/_env.sh), and a GL assertion
# would fail for reasons that have nothing to do with TouchPose.
#
# Font metrics are the shipped Poppins atlas's own numbers rather than a
# live atlas, because node SIZE is what proves the regions do not overlap
# and size needs metrics, not a GL context.
#
import os


SCOPE = "/Biped/Rig/TouchPose"
CONTROL_REL = "touchpose:control"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


class _PoppinsMetrics(object):
    """assets/fonts/Poppins-Regular.json, the atlas the editor loads."""

    ascender = 1.05
    descender = -0.35
    lineHeight = 1.5


def _TextWidth(text, fontSize):
    """The editor's own no-atlas fallback (TextRenderer.calculateTextWidth).

    Widths do not decide any assertion here -- only heights and positions
    do -- so the fallback keeps this runnable without a GL context.
    """
    return float(len(text)) * fontSize * 0.5


def _Overlaps(a, b):
    ax, ay = a.position[0], a.position[1]
    bx, by = b.position[0], b.position[1]
    return (ax < bx + b.size[0] and bx < ax + a.size[0]
            and ay < by + b.size[1] and by < ay + a.size[1])


def testUsdviewInputFunction(appController):
    from UsdNoodles.nodeFactory import NodeFactory
    from UsdNoodles.nodeGraphStage import NodeGraphStage
    from UsdNoodles.nodeLibs.registry import NodeLibraryRegistry
    from UsdNoodles.touchPose import (
        collect_graph_prims,
        control_paths,
        find_touch_groups,
        is_touch_group,
        is_touch_region,
    )

    appController._processEvents()
    stage = appController._usdviewApi.stage

    # --- 1. the data is there at all -----------------------------------
    scope = stage.GetPrimAtPath(SCOPE)
    _Check(bool(scope) and scope.IsValid(), "no TouchPose scope at %s" % SCOPE)
    groups = find_touch_groups(stage)
    _Check([str(g.GetPath()) for g in groups] == [SCOPE],
           "find_touch_groups found %s" % [str(g.GetPath()) for g in groups])

    authored = [c for c in scope.GetAllChildren() if is_touch_region(c)]
    bound = [r for r in authored if control_paths(r)]
    _Check(len(authored) > 0, "the scope holds no regions")
    _Check(len(bound) == len(authored),
           "%d of %d regions bind no control"
           % (len(authored) - len(bound), len(authored)))

    # --- 2. the editor's own loader turns them into nodes ---------------
    libraries = NodeLibraryRegistry().discover()
    _Check("TouchPose" in [lib.get_name() for lib in libraries],
           "the TouchPose node library was not discovered: %s"
           % [lib.get_name() for lib in libraries])

    graph = NodeGraphStage()
    graph.setStage(stage)
    graph.load(stage, _TextWidth, _PoppinsMetrics(),
               nodeFactory=NodeFactory(libraries))

    regionNodes = {}
    groupNodes = {}
    for nodeId, node in graph.nodes.items():
        prim = stage.GetPrimAtPath(nodeId)
        if is_touch_region(prim):
            regionNodes[nodeId] = node
        elif is_touch_group(prim):
            groupNodes[nodeId] = node

    _Check(len(regionNodes) == len(authored),
           "%d regions authored, %d became nodes"
           % (len(authored), len(regionNodes)))
    _Check(list(groupNodes) == [SCOPE],
           "the group scope is not a node: %s" % sorted(groupNodes))

    # --- 3. every link lands on the control the region names -----------
    expected = {}
    for region in authored:
        expected[str(region.GetPath())] = str(control_paths(region)[0])

    drawn = {}
    for nodeId, node in regionNodes.items():
        for link in list(node.outputLinks):
            if link.sourcePort != CONTROL_REL:
                continue
            _Check(link.is_relationship_link,
                   "%s is not drawn as a relationship link" % nodeId)
            drawn.setdefault(nodeId, []).append(link.targetNodeId)

    _Check(len(drawn) == len(expected),
           "%d regions have a control link, expected %d"
           % (len(drawn), len(expected)))
    for nodeId, targets in drawn.items():
        _Check(targets == [expected[nodeId]],
               "%s links to %s, expected %s"
               % (nodeId, targets, [expected[nodeId]]))
        target = targets[0]
        _Check(target in graph.nodes,
               "%s links to %s, which is not a node in the graph"
               % (nodeId, target))
        prim = stage.GetPrimAtPath(target)
        _Check(bool(prim) and prim.IsValid()
               and str(prim.GetTypeName()).startswith("RigExec"),
               "%s links to %s, which is a %s and not a rig prim"
               % (nodeId, target, prim.GetTypeName()))

    # --- 4. laid out, not piled up -------------------------------------
    origin = [n for n in regionNodes.values()
              if (n.position[0], n.position[1]) == (0.0, 0.0)]
    _Check(not origin,
           "%d region nodes sit at the origin" % len(origin))
    placed = sorted(regionNodes.values(), key=lambda n: n.position[1])
    overlapping = sum(1 for a, b in zip(placed, placed[1:]) if _Overlaps(a, b))
    _Check(overlapping == 0,
           "%d pairs of region nodes overlap" % overlapping)

    # --- 5. the face array is a row, not a payload ---------------------
    # up to 1,362 ints per region: it must cost a pin NAME and nothing
    # more. Node width is what would blow up if a value were rendered.
    sample = stage.GetPrimAtPath(sorted(regionNodes)[0])
    faces = sample.GetAttribute("touchpose:faces").Get()
    node = regionNodes[str(sample.GetPath())]
    _Check(len(faces) > 100, "sample region has only %d faces" % len(faces))
    _Check(node.size[0] < 4000.0,
           "a region node is %.0f units wide -- the face array is being "
           "rendered" % node.size[0])
    _Check("touchpose:faces" in list(node.inputPins),
           "the faces row is missing: %s" % list(node.inputPins))
    noise = [p for p in list(node.inputPins) + list(node.outputPins)
             if p in ("purpose", "visibility", "proxyPrim")]
    _Check(not noise,
           "schema boilerplate survived on a region node: %s" % noise)

    # --- 6. selecting the group in the prim tree means "the regions" ---
    expanded = [str(p.GetPath()) for p in collect_graph_prims(scope, stage)]
    _Check(expanded[0] == SCOPE, "the group is not first: %s" % expanded[:2])
    _Check(len(expanded) == 1 + len(authored) + len(set(expected.values())),
           "expanding the group yields %d prims, expected %d"
           % (len(expanded), 1 + len(authored) + len(set(expected.values()))))

    shot = os.getenv("RIGEXEC_NOODLES_SHOT")
    if shot:
        appController._mainWindow.grab().save(shot)

    heights = sorted(n.size[1] for n in regionNodes.values())
    print("RIGEXEC_NOODLES_TOUCH_OK %d regions -> %d nodes, %d links all "
          "resolving to RigExec prims, %d distinct positions, 0 overlaps "
          "(node height %.0f), %d nodes in the graph"
          % (len(authored), len(regionNodes), len(drawn),
             len({(n.position[0], n.position[1])
                  for n in regionNodes.values()}),
             heights[len(heights) // 2], len(graph.nodes)))
