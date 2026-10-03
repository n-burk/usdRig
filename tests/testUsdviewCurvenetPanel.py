#
# testusdview verification that the curvenet authoring panel WORKS INSIDE
# USDVIEW, which the headless test in testUsdviewCurvenetAuthoring.py cannot
# check: that one exercises the module-level stage edits with no widgets at
# all, so a panel that fails to construct, fails to find the stage view, or
# fails to install its viewport event filter would still pass it.
#
# What is asserted here is everything that depends on the running app:
#   * the panel constructs against the real usdviewApi
#   * it finds the curvenet on the stage and reports the section 3 structure
#   * it can reach the stage view and install/remove its event filter, which
#     is what makes drawing possible at all
#   * the pick path is wired (computePickFrustum + pick exist and are
#     callable) -- the one usdview capability this whole tool rests on
#   * the viewport display prim lands in the SESSION layer, never the
#     artist's file
#
from pxr import Sdf

# curvenetUI is imported by name, not by path: testusdview execs this file
# with no __file__ bound, and run_testusdview_curvenet.bat already puts the
# plugin directory on PYTHONPATH -- which is also how usdview itself finds it.


def testUsdviewInputFunction(appController):
    import curvenetUI

    api = appController._usdviewApi
    stage = api.stage
    if not stage:
        raise AssertionError("no stage")

    nets = curvenetUI.FindCurvenetPrims(stage)
    if not nets:
        raise AssertionError("example 12 should carry a RigExecCurvenet")
    net = nets[0]

    panel = curvenetUI.CurvenetPanel.GetInstance(api)
    if panel is None:
        raise AssertionError("the panel did not construct")
    if panel._curvenetPath != net.GetPath():
        raise AssertionError(
            "panel selected %s, expected %s"
            % (panel._curvenetPath, net.GetPath()))

    # The section 3 readout must actually say something about this net.
    text = panel._structure.text()
    for expected in ("pool points", "splines", "curves", "intersections"):
        if expected not in text:
            raise AssertionError(
                "structure readout is missing %r: %s" % (expected, text))

    topology = curvenetUI.Topology(len(curvenetUI.GetPoints(net)),
                                   curvenetUI.GetSplines(net))
    intersections = sum(1 for k in topology.kind
                        if k == curvenetUI.KIND_INTERSECTION)
    if intersections != 12:
        raise AssertionError(
            "example 12's net should have 12 intersections, got %d"
            % intersections)
    if topology.IsolatedCurves():
        raise AssertionError(
            "example 12's net should have no isolated curves, got %d"
            % len(topology.IsolatedCurves()))

    # The stage view, and the pick path the whole tool rests on.
    view = panel._picker.StageView()
    if view is None:
        raise AssertionError(
            "the panel could not reach usdview's stage view; drawing on the "
            "model is impossible without it")
    for method in ("computePickFrustum", "pick"):
        if not callable(getattr(view, method, None)):
            raise AssertionError("stage view has no %s()" % method)

    # A pick through the middle of the viewport should land on the tube.
    # PHYSICAL pixels: computePickFrustum divides by a physical-pixel
    # viewport, so a logical-pixel argument silently aims somewhere else
    # (see curvenetUI._Position).
    width, height = api.viewportSize
    ratio = view.devicePixelRatioF()
    hit = panel._picker.Pick(int(width / 2 * ratio), int(height / 2 * ratio))
    if hit is None:
        raise AssertionError(
            "a pick at the centre of the viewport returned nothing; the "
            "example's tube fills the frame, so the pick path is broken")
    path, point, _ = hit
    if not str(path).startswith("/CurvenetAsset/Geom/Tube"):
        raise AssertionError("centre pick hit %s, expected the tube" % path)

    # Event filter install/remove, which draw mode depends on.
    panel._SetMode(curvenetUI.MODE_DRAW)
    if not panel._filterInstalled:
        raise AssertionError("draw mode did not install the event filter")
    panel._SetMode(curvenetUI.MODE_OFF)
    if panel._filterInstalled:
        raise AssertionError("leaving draw mode did not remove the filter")

    # The viewport aid belongs to the session layer and nowhere else.
    panel._UpdateDisplay()
    displayPath = panel._DisplayPath()
    if not stage.GetPrimAtPath(displayPath):
        raise AssertionError("no display prim at %s" % displayPath)
    if stage.GetRootLayer().GetPrimAtPath(displayPath):
        raise AssertionError(
            "the display prim leaked into the artist's root layer at %s"
            % displayPath)
    if not stage.GetSessionLayer().GetPrimAtPath(displayPath):
        raise AssertionError(
            "the display prim is not in the session layer")

    print("RIGEXEC_CURVENET_PANEL_OK %d intersections, centre pick on %s"
          % (intersections, path))
