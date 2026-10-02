#
# testusdview smoke test for the curvenet authoring panel against whatever
# net and model the stage it is given carries.
#
# It was written for one character, naming that character's prims and the
# ring and anchor counts of that character's net. The character is not in
# this repository, so the test could not run in a fresh checkout at all --
# usdview failed to open the stage. The model now comes from the stage
# (curvenet_test_stage.Model), and the counts are replaced by the invariant
# they were standing in for: every point of the net is classified exactly
# once, and no curve is isolated. Those hold for any net; 18 rings and 12
# anchors held only for that one.
#
def testUsdviewInputFunction(appController):
    import curvenetUI
    # testusdview execs this file, so there is no __file__ to locate the
    # tests directory with. curvenetUI is imported from
    # plugin/rigExecUsdview, which fixes the checkout root.
    import os as _os
    import sys as _sys
    _tests = _os.path.join(
        _os.path.dirname(_os.path.dirname(
            _os.path.dirname(_os.path.abspath(curvenetUI.__file__)))),
        "tests")
    if _tests not in _sys.path:
        _sys.path.insert(0, _tests)
    import curvenet_test_stage

    api = appController._usdviewApi
    stage = api.stage

    nets = curvenetUI.FindCurvenetPrims(stage)
    if not nets:
        raise AssertionError("no RigExecCurvenet on the puppet stage")
    net = nets[0]

    panel = curvenetUI.CurvenetPanel.GetInstance(api)
    if panel._curvenetPath != net.GetPath():
        raise AssertionError("panel selected %s" % panel._curvenetPath)

    pointCount = len(curvenetUI.GetPoints(net))
    topology = curvenetUI.Topology(pointCount, curvenetUI.GetSplines(net))
    counts = topology.CountsByKind()
    # Exactly once each: a point the classifier misses is a point the
    # deformation gradient has no rule for, and a point counted twice
    # means two rules disagree about it.
    if sum(counts.values()) != pointCount:
        raise AssertionError(
            "%d point(s) classified for a net of %d: %s"
            % (sum(counts.values()), pointCount, counts))
    if not counts.get(curvenetUI.KIND_INTERSECTION, 0):
        raise AssertionError(
            "the net has no intersections, so nothing in it can express a "
            "width or twist change: %s" % counts)
    if topology.IsolatedCurves():
        raise AssertionError("the head net should have no isolated curves")

    # The pick path, on the puppet. Frame the head first: testusdview starts
    # on an unframed default camera, and picking is only meaningful once
    # something is actually under the cursor.
    body = curvenet_test_stage.Model(stage)[0]
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
    body = curvenet_test_stage.Model(stage)[0]
    warnings = curvenetUI.CheckBindPreconditions(stage, body, rig)
    if warnings:
        raise AssertionError("the model fails bind preconditions: %s"
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

    print("RIGEXEC_CURVENET_PUPPET_OK %d point(s) classified %s, "
          "centre pick on %s"
          % (pointCount,
             ", ".join("%d %s" % (n, kind)
                       for kind, n in sorted(counts.items())), path))
