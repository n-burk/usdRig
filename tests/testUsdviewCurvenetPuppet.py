#
# testusdview smoke test for the curvenet authoring panel on a real
# character (chars/puppetA/puppetA_curvenet.usda) rather than on the
# synthetic example. Different meshes break different things: puppetA's body
# is quad-dominant with tris, pentagons and hexagons, 110 boundary edges and
# 4 non-manifold ones, which is the kind of surface an artist actually has.
#
def testUsdviewInputFunction(appController):
    import curvenetUI

    api = appController._usdviewApi
    stage = api.stage

    nets = curvenetUI.FindCurvenetPrims(stage)
    if not nets:
        raise AssertionError("no RigExecCurvenet on the puppet stage")
    net = nets[0]

    panel = curvenetUI.CurvenetPanel.GetInstance(api)
    if panel._curvenetPath != net.GetPath():
        raise AssertionError("panel selected %s" % panel._curvenetPath)

    topology = curvenetUI.Topology(len(curvenetUI.GetPoints(net)),
                                   curvenetUI.GetSplines(net))
    counts = topology.CountsByKind()
    intersections = counts.get(curvenetUI.KIND_INTERSECTION, 0)
    if intersections != 18:
        raise AssertionError(
            "expected 18 ring intersections (3 rings x 6 spokes), got %d"
            % intersections)
    if counts.get(curvenetUI.KIND_ANCHOR, 0) != 12:
        raise AssertionError("expected 12 rail anchors, got %d"
                             % counts.get(curvenetUI.KIND_ANCHOR, 0))
    if topology.IsolatedCurves():
        raise AssertionError("the head net should have no isolated curves")

    # The pick path, on the puppet. Frame the head first: testusdview starts
    # on an unframed default camera, and picking is only meaningful once
    # something is actually under the cursor.
    body = stage.GetPrimAtPath("/puppetA/root/body_geo/node_0_Retopology")
    if not body:
        raise AssertionError("no body mesh")
    api.ClearPrimSelection()
    api.AddPrimToSelection(body)
    appController._frameSelection()
    appController._processEvents()

    # Sweep a grid rather than assuming the centre is opaque: this character
    # has outstretched arms and a hollow head shell, so the middle of its
    # bounding box is empty space. Coordinates are PHYSICAL pixels, which is
    # what the pick frustum wants (see curvenetUI._Position).
    width, height = api.viewportSize
    ratio = panel._picker.StageView().devicePixelRatioF()
    hit = None
    for row in range(1, 10):
        for column in range(1, 10):
            hit = panel._picker.Pick(int(width * column / 10.0 * ratio),
                                     int(height * row / 10.0 * ratio))
            if hit is not None:
                break
        if hit is not None:
            break
    if hit is None:
        raise AssertionError(
            "nothing on a 9x9 grid over a framed puppet picks; the "
            "authoring tool cannot place a knot on this asset")
    path, point, normal = hit

    # The panel's own bind preconditions must pass on this asset -- vertex
    # normals and identity to the asset root are what the engine needs.
    rig = curvenetUI.FindRigPrim(stage)
    body = stage.GetPrimAtPath("/puppetA/root/body_geo/node_0_Retopology")
    warnings = curvenetUI.CheckBindPreconditions(stage, body, rig)
    if warnings:
        raise AssertionError("puppetA body fails bind preconditions: %s"
                             % warnings)

    # Drawing must work on this surface: place two knots and connect them.
    before = len(curvenetUI.GetSplines(net)) // 4
    a = curvenetUI.AppendKnot(net, point)
    b = curvenetUI.AppendKnot(net, point + normal * 0.05)
    curvenetUI.AddSpline(net, a, b)
    after = len(curvenetUI.GetSplines(net)) // 4
    if after != before + 1:
        raise AssertionError("AddSpline did not add a spline")

    panel._UpdateDisplay()
    if not stage.GetSessionLayer().GetPrimAtPath(panel._DisplayPath()):
        raise AssertionError("display prim not in the session layer")
    if stage.GetRootLayer().GetPrimAtPath(panel._DisplayPath()):
        raise AssertionError("display prim leaked into the root layer")

    print("RIGEXEC_CURVENET_PUPPET_OK %d intersections, %d anchors, "
          "centre pick on %s" % (intersections,
                                 counts.get(curvenetUI.KIND_ANCHOR, 0), path))
